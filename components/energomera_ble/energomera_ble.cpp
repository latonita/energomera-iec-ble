#include "energomera_ble.h"

#include "esphome/core/log.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <esp_gatt_defs.h>
#include <esp_heap_caps.h>

namespace esphome {
namespace energomera_ble {

static const char *const TAG = "energomera_ble";

static const uint8_t ENERGOMERA_SERVICE_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                        0xe2, 0x45, 0xef, 0x8b, 0x00, 0x01, 0x1b, 0xb9};
static const uint8_t ENERGOMERA_VERSION_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                        0xe2, 0x45, 0xef, 0x8b, 0x01, 0x01, 0x1b, 0xb9};
static const uint8_t ENERGOMERA_TX_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                   0xe2, 0x45, 0xef, 0x8b, 0x05, 0x01, 0x1b, 0xb9};
static const uint8_t ENERGOMERA_RESPONSE_UUIDS_128[16][16] = {
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x06, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x07, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x08, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x09, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0a, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0b, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0c, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0d, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0d, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0e, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x0f, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x10, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x11, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x12, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x13, 0x01, 0x1b, 0xb9},
    {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97, 0xe2, 0x45, 0xef, 0x8b, 0x14, 0x01, 0x1b, 0xb9},
};

static const uint16_t FALLBACK_VERSION_OFFSET = 0x0002;
static const uint16_t FALLBACK_TX_OFFSET = 0x0005;
static const uint16_t FALLBACK_RESPONSE_OFFSETS[16] = {0x0009, 0x000C, 0x000F, 0x0012, 0x0015, 0x0018, 0x001B, 0x001E,
                                                       0x001E, 0x0021, 0x0024, 0x0027, 0x002A, 0x002D, 0x0030, 0x0033};

static inline bool uuid_equals_128(const uint8_t *lhs, const uint8_t *rhs) {
  return std::memcmp(lhs, rhs, 16) == 0;
}

static uint8_t apply_even_parity(uint8_t value) {
  uint8_t bit_count = 0;
  for (uint8_t bit = 0; bit < 7; bit++) {
    bit_count += (value >> bit) & 0x01;
  }
  uint8_t with_msb = value | 0x80;
  if ((bit_count & 0x01) != 0) {
    return with_msb;
  }
  return with_msb & 0x7F;
}

EnergomeraBleComponent::EnergomeraBleComponent() : PollingComponent(60 * 1000) {}

void EnergomeraBleComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Energomera BLE component");
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "BLE client parent not configured");
    this->mark_failed();
    return;
  }

  this->sync_address_from_parent_();

  // Configure default security parameters to allow MITM bonding with PIN
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_IO;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t resp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &resp_key, sizeof(resp_key));
}

void EnergomeraBleComponent::loop() {
  if (!this->address_set_)
    this->sync_address_from_parent_();
}

void EnergomeraBleComponent::update() {}

void EnergomeraBleComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Energomera BLE Component");
  if (this->address_set_) {
    ESP_LOGCONFIG(TAG, "  target address: %02X:%02X:%02X:%02X:%02X:%02X", this->target_address_[0],
                  this->target_address_[1], this->target_address_[2], this->target_address_[3],
                  this->target_address_[4], this->target_address_[5]);
  } else if (this->parent_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  target address (parent): %s", this->parent_->address_str().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  target address: not set");
  }
  ESP_LOGCONFIG(TAG, "  version requested: %s", this->version_requested_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  version reported: %s", this->version_reported_ ? "yes" : "no");
}

void EnergomeraBleComponent::set_passkey(uint32_t passkey) {
  char buf[7] = {0};
  snprintf(buf, sizeof(buf), "%06u", passkey % 1000000U);
  this->pin_code_ = buf;
}

void EnergomeraBleComponent::sync_address_from_parent_() {
  if (this->address_set_ || this->parent_ == nullptr)
    return;
  uint8_t *remote = this->parent_->get_remote_bda();
  if (remote == nullptr)
    return;
  bool nonzero = false;
  for (uint8_t i = 0; i < 6; i++) {
    if (remote[i] != 0) {
      nonzero = true;
      break;
    }
  }
  if (!nonzero)
    return;
  std::memcpy(this->target_address_.data(), remote, 6);
  this->address_set_ = true;
  ESP_LOGD(TAG, "Using parent BLE address: %02X:%02X:%02X:%02X:%02X:%02X", remote[0], remote[1], remote[2], remote[3],
           remote[4], remote[5]);
}

bool EnergomeraBleComponent::match_service_uuid_(const esp_bt_uuid_t &uuid) const {
  if (uuid.len == ESP_UUID_LEN_16)
    return uuid.uuid.uuid16 == 0x0100;
  if (uuid.len == ESP_UUID_LEN_32)
    return uuid.uuid.uuid32 == 0x00000100;  // defensive
  if (uuid.len == ESP_UUID_LEN_128)
    return std::memcmp(uuid.uuid.uuid128, ENERGOMERA_SERVICE_UUID_128, sizeof(ENERGOMERA_SERVICE_UUID_128)) == 0;
  return false;
}

void EnergomeraBleComponent::initiate_pairing_(const esp_bd_addr_t remote_bda) {
  if (this->pin_code_.empty()) {
    ESP_LOGD(TAG, "No PIN configured, relying on existing bond");
    return;
  }
  auto *mutable_addr = const_cast<uint8_t *>(remote_bda);
  esp_err_t status = esp_ble_set_encryption(mutable_addr, ESP_BLE_SEC_ENCRYPT_MITM);
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_set_encryption failed: %d", status);
  } else {
    ESP_LOGI(TAG, "Requested encrypted link using configured PIN");
  }
}

void EnergomeraBleComponent::request_firmware_version_() {
  if (this->parent_ == nullptr)
    return;
  if (this->version_requested_)
    return;
  if (this->service_start_handle_ == 0 || this->service_end_handle_ == 0) {
    ESP_LOGW(TAG, "Service handles not resolved, cannot read firmware version yet");
    return;
  }

  if (!this->characteristics_resolved_) {
    if (!this->resolve_characteristics_())
      return;
  }

  if (this->version_char_handle_ == 0) {
    ESP_LOGW(TAG, "Firmware version characteristic (0x0101) not found in service");
    return;
  }

  esp_err_t err = esp_ble_gattc_read_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                          this->version_char_handle_, ESP_GATT_AUTH_REQ_NONE);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to request firmware version read: %d", err);
    return;
  }
  this->version_requested_ = true;
  ESP_LOGD(TAG, "Firmware version read requested (handle 0x%04X)", this->version_char_handle_);
}

bool EnergomeraBleComponent::resolve_characteristics_() {
  if (this->characteristics_resolved_)
    return true;
  if (this->parent_ == nullptr)
    return false;
  if (this->service_start_handle_ == 0 || this->service_end_handle_ == 0) {
    ESP_LOGD(TAG, "Service handles not ready for characteristic resolution");
    return false;
  }

  std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
  this->version_char_handle_ = 0;
  this->tx_char_handle_ = 0;
  this->tx_cccd_handle_ = 0;

  uint16_t count = 0;
  esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
      this->parent_->get_gattc_if(), this->parent_->get_conn_id(), ESP_GATT_DB_CHARACTERISTIC, this->service_start_handle_,
      this->service_end_handle_, ESP_GATT_INVALID_HANDLE, &count);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_get_attr_count failed: %d", status);
  }

  if (status == ESP_GATT_OK && count > 0) {
    auto *char_elems =
        (esp_gattc_char_elem_t *) heap_caps_malloc(sizeof(esp_gattc_char_elem_t) * count, MALLOC_CAP_8BIT);
    if (char_elems != nullptr) {
      status = esp_ble_gattc_get_all_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                          this->service_start_handle_, this->service_end_handle_, char_elems, &count, 0);
      if (status == ESP_GATT_OK) {
        for (uint16_t i = 0; i < count; i++) {
          auto &elem = char_elems[i];
          uint16_t handle = elem.char_handle;
          if (elem.uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGD(TAG, "Characteristic handle=0x%04X uuid16=0x%04X properties=0x%02X", handle,
                     elem.uuid.uuid.uuid16, elem.properties);
            if (elem.uuid.uuid.uuid16 == 0x0101) {
              this->version_char_handle_ = handle;
            }
          } else if (elem.uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGD(TAG,
                     "Characteristic handle=0x%04X uuid128=%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X properties=0x%02X",
                     handle, elem.uuid.uuid.uuid128[15], elem.uuid.uuid.uuid128[14], elem.uuid.uuid.uuid128[13],
                     elem.uuid.uuid.uuid128[12], elem.uuid.uuid.uuid128[11], elem.uuid.uuid.uuid128[10],
                     elem.uuid.uuid.uuid128[9], elem.uuid.uuid.uuid128[8], elem.uuid.uuid.uuid128[7],
                     elem.uuid.uuid.uuid128[6], elem.uuid.uuid.uuid128[5], elem.uuid.uuid.uuid128[4],
                     elem.uuid.uuid.uuid128[3], elem.uuid.uuid.uuid128[2], elem.uuid.uuid.uuid128[1],
                     elem.uuid.uuid.uuid128[0], elem.properties);
            if (uuid_equals_128(elem.uuid.uuid.uuid128, ENERGOMERA_VERSION_UUID_128)) {
              this->version_char_handle_ = handle;
            } else if (uuid_equals_128(elem.uuid.uuid.uuid128, ENERGOMERA_TX_UUID_128)) {
              this->tx_char_handle_ = handle;
            } else {
              for (size_t idx = 0; idx < this->response_char_handles_.size(); idx++) {
                if (uuid_equals_128(elem.uuid.uuid.uuid128, ENERGOMERA_RESPONSE_UUIDS_128[idx])) {
                  this->response_char_handles_[idx] = handle;
                  break;
                }
              }
            }
          }
        }
      } else {
        ESP_LOGW(TAG, "esp_ble_gattc_get_all_char failed: %d", status);
      }
      free(char_elems);
    } else {
      ESP_LOGW(TAG, "Failed to allocate memory for characteristic enumeration");
    }
  }

  const uint16_t service_base = this->service_start_handle_;
  if (this->version_char_handle_ == 0) {
    uint16_t fallback = service_base + FALLBACK_VERSION_OFFSET;
    if (fallback > service_base && fallback < this->service_end_handle_) {
      this->version_char_handle_ = fallback;
      ESP_LOGW(TAG, "Firmware characteristic not enumerated, using fallback handle 0x%04X", fallback);
    }
  }

  if (this->tx_char_handle_ == 0) {
    uint16_t fallback = service_base + FALLBACK_TX_OFFSET;
    if (fallback > service_base && fallback < this->service_end_handle_) {
      this->tx_char_handle_ = fallback;
      ESP_LOGW(TAG, "TX characteristic not enumerated, using fallback handle 0x%04X", fallback);
    }
  }

  for (size_t idx = 0; idx < this->response_char_handles_.size(); idx++) {
    if (this->response_char_handles_[idx] == 0) {
      uint16_t fallback = service_base + FALLBACK_RESPONSE_OFFSETS[idx];
      if (fallback > service_base && fallback < this->service_end_handle_) {
        this->response_char_handles_[idx] = fallback;
        ESP_LOGW(TAG, "Response characteristic %zu not enumerated, using fallback handle 0x%04X", idx, fallback);
      }
    }
  }

  if (this->tx_char_handle_ != 0) {
    this->resolve_tx_descriptors_();
  }

  this->characteristics_resolved_ = (this->version_char_handle_ != 0 && this->tx_char_handle_ != 0);
  return this->characteristics_resolved_;
}

bool EnergomeraBleComponent::resolve_tx_descriptors_() {
  if (this->parent_ == nullptr)
    return false;
  if (this->tx_char_handle_ == 0)
    return false;

  uint16_t count = 0;
  esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
      this->parent_->get_gattc_if(), this->parent_->get_conn_id(), ESP_GATT_DB_DESCRIPTOR, this->service_start_handle_,
      this->service_end_handle_, this->tx_char_handle_, &count);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_get_attr_count(descr) failed: %d", status);
  }

  if (status == ESP_GATT_OK && count > 0) {
    auto *descr_elems =
        (esp_gattc_descr_elem_t *) heap_caps_malloc(sizeof(esp_gattc_descr_elem_t) * count, MALLOC_CAP_8BIT);
    if (descr_elems != nullptr) {
      status = esp_ble_gattc_get_all_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->tx_char_handle_,
                                           descr_elems, &count, 0);
      if (status == ESP_GATT_OK) {
        for (uint16_t idx = 0; idx < count; idx++) {
          auto &elem = descr_elems[idx];
          if (elem.uuid.len == ESP_UUID_LEN_16 && elem.uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
            this->tx_cccd_handle_ = elem.handle;
            break;
          }
        }
      } else {
        ESP_LOGW(TAG, "esp_ble_gattc_get_all_descr failed: %d", status);
      }
      free(descr_elems);
    } else {
      ESP_LOGW(TAG, "Failed to allocate descriptor enumeration buffer");
    }
  }

  if (this->tx_cccd_handle_ == 0 && this->tx_char_handle_ != 0) {
    this->tx_cccd_handle_ = this->tx_char_handle_ + 1;
    ESP_LOGW(TAG, "TX CCCD not enumerated, assuming handle 0x%04X", this->tx_cccd_handle_);
  }

  return this->tx_cccd_handle_ != 0;
}

void EnergomeraBleComponent::enable_notifications_if_needed_() {
  if (this->notifications_enabled_ || this->cccd_write_pending_ || this->notification_failed_)
    return;
  if (this->parent_ == nullptr || this->tx_char_handle_ == 0 || this->tx_cccd_handle_ == 0)
    return;
  if (this->notify_retry_attempts_ >= 5) {
    ESP_LOGW(TAG, "Notification enable retries exhausted; switching to polling");
    this->notification_failed_ = true;
    this->fallback_to_polling_reads_();
    return;
  }

  auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                  this->tx_char_handle_);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed: %d", status);
    this->notify_retry_attempts_++;
    uint32_t attempt = this->notify_retry_attempts_;
    uint32_t delay = 300 + 200 * (attempt > 0 ? (attempt - 1U) : 0U);
    this->schedule_notification_retry_(delay);
    return;
  }

  this->cccd_write_pending_ = true;
  this->notify_retry_attempts_++;

  uint16_t notify_en = 0x0001;
  auto err = esp_ble_gattc_write_char_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                            this->tx_cccd_handle_, sizeof(notify_en), (uint8_t *) &notify_en,
                                            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to enable notifications on handle 0x%04X: %d", this->tx_cccd_handle_, err);
    uint32_t attempt = this->notify_retry_attempts_;
    uint32_t delay = 300 + 200 * (attempt > 0 ? (attempt - 1U) : 0U);
    this->schedule_notification_retry_(delay);
    return;
  }

  ESP_LOGD(TAG, "Notification enable request sent (handle 0x%04X, attempt %u)", this->tx_cccd_handle_,
           this->notify_retry_attempts_);
}

void EnergomeraBleComponent::schedule_notification_retry_(uint32_t delay_ms) {
  if (this->notify_retry_attempts_ >= 5) {
    ESP_LOGW(TAG, "Notification enable retries exhausted; switching to polling");
    this->cccd_write_pending_ = false;
    this->notification_failed_ = true;
    this->fallback_to_polling_reads_();
    return;
  }
  this->cccd_write_pending_ = true;
  this->set_timeout("cccd_retry", delay_ms, [this]() {
    this->cccd_write_pending_ = false;
    this->enable_notifications_if_needed_();
  });
}

void EnergomeraBleComponent::fallback_to_polling_reads_() {
  if (!this->notification_failed_)
    return;
  if (!this->command_pending_ && !this->command_inflight_) {
    this->command_pending_ = true;
  }
  if (!this->response_in_progress_)
    this->issue_next_response_read_();
  this->set_timeout("polling_read_loop", 1500, [this]() {
    if (this->notification_failed_) {
      this->issue_next_response_read_();
      this->fallback_to_polling_reads_();
    }
  });
}

void EnergomeraBleComponent::prepare_et0pe_command_() {
  static const uint8_t RAW_COMMAND[] = {0x2F, 0x3F, 0x21, 0x01, 0x52, 0x31, 0x02, 0x45,
                                        0x54, 0x30, 0x50, 0x45, 0x28, 0x29, 0x03, 0x00};
  this->tx_message_remaining_.assign(RAW_COMMAND, RAW_COMMAND + sizeof(RAW_COMMAND));
  for (auto &byte : this->tx_message_remaining_) {
    byte = apply_even_parity(byte & 0x7F);
  }
  this->tx_fragment_started_ = false;
  this->tx_sequence_counter_ = 0;
}

void EnergomeraBleComponent::try_send_pending_command_() {
  if (!this->command_pending_ || this->command_inflight_ || this->command_complete_)
    return;
  if (!this->notifications_enabled_) {
    ESP_LOGD(TAG, "Notifications not yet enabled; deferring command send");
    return;
  }
  if (this->parent_ == nullptr || this->tx_char_handle_ == 0)
    return;

  if (this->tx_message_remaining_.empty())
    this->prepare_et0pe_command_();

  if (this->tx_message_remaining_.empty()) {
    ESP_LOGW(TAG, "Command payload is empty, nothing to send");
    this->command_pending_ = false;
    return;
  }

  ESP_LOGI(TAG, "Sending ET0PE command (%u bytes payload)", (unsigned) this->tx_message_remaining_.size());
  this->command_pending_ = false;
  this->command_inflight_ = true;
  this->command_complete_ = false;

  if (!this->send_next_fragment_()) {
    ESP_LOGW(TAG, "Failed to send first command fragment");
    this->command_inflight_ = false;
    this->command_pending_ = true;
    this->command_complete_ = false;
  }
}

uint16_t EnergomeraBleComponent::get_max_payload_() const {
  if (this->mtu_ <= 4)
    return 0;
  return this->mtu_ - 4;
}

bool EnergomeraBleComponent::send_next_fragment_() {
  if (this->parent_ == nullptr || this->tx_char_handle_ == 0)
    return false;
  if (this->tx_message_remaining_.empty())
    return false;

  uint16_t max_payload = this->get_max_payload_();
  if (max_payload == 0) {
    ESP_LOGW(TAG, "MTU too small to carry command data");
    return false;
  }

  bool more_after_this = this->tx_message_remaining_.size() > max_payload;
  uint16_t chunk_len = more_after_this ? max_payload : this->tx_message_remaining_.size();

  std::vector<uint8_t> packet(chunk_len + 1);
  if (!this->tx_fragment_started_) {
    this->tx_fragment_started_ = true;
    this->tx_sequence_counter_ = 0;
    if (more_after_this) {
      packet[0] = 0x00;
    } else {
      packet[0] = 0x80;
      if (this->mtu_ > 23)
        packet[0] |= 0x40;
    }
  } else {
    this->tx_sequence_counter_ = (this->tx_sequence_counter_ + 1) & 0x7F;
    packet[0] = this->tx_sequence_counter_;
    if (!more_after_this)
      packet[0] |= 0x80;
  }

  std::copy_n(this->tx_message_remaining_.begin(), chunk_len, packet.begin() + 1);

  esp_err_t status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                              this->tx_char_handle_, packet.size(), packet.data(),
                                              ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_write_char failed: %d", status);
    return false;
  }

  std::string hex;
  hex.reserve(packet.size() * 3);
  for (size_t i = 0; i < packet.size(); i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", packet[i]);
    hex.append(buf);
  }
  ESP_LOGD(TAG, "Command fragment sent (%u bytes): %s", (unsigned) packet.size(), hex.c_str());

  this->tx_message_remaining_.erase(this->tx_message_remaining_.begin(),
                                    this->tx_message_remaining_.begin() + chunk_len);

  if (more_after_this) {
    this->parent_->run_later([this]() { this->send_next_fragment_(); });
  }

  return true;
}

void EnergomeraBleComponent::start_response_sequence_(uint8_t slot_count) {
  uint8_t count = slot_count + 1;  // device expects slots 0..need_read
  if (count == 0)
    count = 1;
  if (count > this->response_char_handles_.size())
    count = this->response_char_handles_.size();

  this->pending_response_handles_.clear();
  this->response_buffer_.clear();
  this->response_in_progress_ = true;

  for (uint8_t idx = 0; idx < count; idx++) {
    uint16_t handle = this->response_char_handles_[idx];
    if (handle == 0) {
      ESP_LOGW(TAG, "Response characteristic index %u not resolved", idx);
      continue;
    }
    this->pending_response_handles_.push_back(handle);
  }

  if (this->pending_response_handles_.empty()) {
    ESP_LOGW(TAG, "No response handles available to read");
    this->response_in_progress_ = false;
    this->command_inflight_ = false;
    return;
  }

  this->issue_next_response_read_();
}

void EnergomeraBleComponent::issue_next_response_read_() {
  if (this->pending_response_handles_.empty()) {
    this->finalize_command_response_();
    return;
  }

  this->current_response_handle_ = this->pending_response_handles_.front();
  esp_err_t status = esp_ble_gattc_read_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                             this->current_response_handle_, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "Failed to request response read (handle 0x%04X): %d", this->current_response_handle_, status);
    this->pending_response_handles_.clear();
    this->finalize_command_response_();
  }
}

void EnergomeraBleComponent::handle_command_read_(
    const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param) {
  if (param.status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "Response read failed (handle 0x%04X): %d", param.handle, param.status);
    this->pending_response_handles_.clear();
    this->finalize_command_response_();
    return;
  }

  for (uint16_t i = 0; i < param.value_len; i++) {
    this->response_buffer_.push_back(param.value[i] & 0x7F);
  }

  if (!this->pending_response_handles_.empty() && this->pending_response_handles_.front() == param.handle) {
    this->pending_response_handles_.erase(this->pending_response_handles_.begin());
  } else {
    auto it = std::find(this->pending_response_handles_.begin(), this->pending_response_handles_.end(), param.handle);
    if (it != this->pending_response_handles_.end())
      this->pending_response_handles_.erase(it);
  }

  this->issue_next_response_read_();
}

void EnergomeraBleComponent::finalize_command_response_() {
  if (!this->response_in_progress_)
    return;

  this->response_in_progress_ = false;
  this->command_inflight_ = false;
  this->command_complete_ = true;
  this->pending_response_handles_.clear();

  if (this->response_buffer_.empty()) {
    ESP_LOGW(TAG, "No response payload received");
    return;
  }

  std::string hex;
  hex.reserve(this->response_buffer_.size() * 3);
  std::string ascii;
  ascii.reserve(this->response_buffer_.size());
  for (auto byte : this->response_buffer_) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", byte);
    hex.append(buf);
    ascii.push_back(std::isprint(static_cast<unsigned char>(byte)) ? static_cast<char>(byte) : '.');
  }

  ESP_LOGI(TAG, "Response payload (%u bytes): %s", (unsigned) this->response_buffer_.size(), hex.c_str());
  ESP_LOGI(TAG, "Response ASCII: %s", ascii.c_str());
}

void EnergomeraBleComponent::handle_notification_(
    const esp_ble_gattc_cb_param_t::gattc_notify_evt_param &param) {
  if (!param.is_notify)
    return;
  if (!param.value || param.value_len == 0) {
    ESP_LOGW(TAG, "Notification with empty payload received");
    return;
  }

  std::string hex;
  hex.reserve(param.value_len * 3);
  for (uint16_t i = 0; i < param.value_len; i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X ", param.value[i]);
    hex.append(buf);
  }

  uint8_t slot_count = param.value[0];
  ESP_LOGD(TAG, "Notification received (need_read=%u): %s", slot_count, hex.c_str());
  this->start_response_sequence_(slot_count);
}

void EnergomeraBleComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                                 esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      if (!this->parent_->check_addr(param->connect.remote_bda))
        break;
      ESP_LOGI(TAG, "GATT client connected");
      this->service_start_handle_ = 0;
      this->service_end_handle_ = 0;
      this->version_char_handle_ = 0;
      this->tx_char_handle_ = 0;
      this->tx_cccd_handle_ = 0;
      this->version_requested_ = false;
      this->version_reported_ = false;
      this->service_search_requested_ = false;
      this->characteristics_resolved_ = false;
      this->notifications_enabled_ = false;
      this->cccd_write_pending_ = false;
      this->command_pending_ = false;
      this->command_inflight_ = false;
      this->command_complete_ = false;
      this->response_in_progress_ = false;
      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      this->notify_retry_attempts_ = 0;
      this->cancel_timeout("cccd_retry");
      std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
      this->pending_response_handles_.clear();
      this->response_buffer_.clear();
      this->tx_message_remaining_.clear();
      this->sync_address_from_parent_();
      this->initiate_pairing_(param->connect.remote_bda);
      break;
    }
    case ESP_GATTC_OPEN_EVT: {
      if (!this->parent_->check_addr(param->open.remote_bda))
        break;
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Failed to open GATT connection: status=%d", param->open.status);
      } else {
        ESP_LOGD(TAG, "GATT connection open (conn_id=%d)", param->open.conn_id);
      }
      break;
    }
    case ESP_GATTC_CFG_MTU_EVT: {
      if (param->cfg_mtu.status == ESP_GATT_OK) {
        this->mtu_ = param->cfg_mtu.mtu;
        ESP_LOGD(TAG, "MTU updated to %d", param->cfg_mtu.mtu);
      } else {
        ESP_LOGW(TAG, "MTU update failed: %d", param->cfg_mtu.status);
      }
      break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
      if (param->search_res.conn_id != this->parent_->get_conn_id())
        break;
      const auto &uuid = param->search_res.srvc_id.uuid;
      if (uuid.len == ESP_UUID_LEN_16) {
        ESP_LOGD(TAG, "Service discovered: uuid16=0x%04X handles=0x%04X-0x%04X", uuid.uuid.uuid16,
                 param->search_res.start_handle, param->search_res.end_handle);
      } else if (uuid.len == ESP_UUID_LEN_32) {
        ESP_LOGD(TAG, "Service discovered: uuid32=0x%08X handles=0x%04X-0x%04X", uuid.uuid.uuid32,
                 param->search_res.start_handle, param->search_res.end_handle);
      } else if (uuid.len == ESP_UUID_LEN_128) {
        char buf[37];
        snprintf(buf, sizeof(buf), "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                 uuid.uuid.uuid128[15], uuid.uuid.uuid128[14], uuid.uuid.uuid128[13], uuid.uuid.uuid128[12],
                 uuid.uuid.uuid128[11], uuid.uuid.uuid128[10], uuid.uuid.uuid128[9], uuid.uuid.uuid128[8],
                 uuid.uuid.uuid128[7], uuid.uuid.uuid128[6], uuid.uuid.uuid128[5], uuid.uuid.uuid128[4],
                 uuid.uuid.uuid128[3], uuid.uuid.uuid128[2], uuid.uuid.uuid128[1], uuid.uuid.uuid128[0]);
        ESP_LOGD(TAG, "Service discovered: uuid128=%s handles=0x%04X-0x%04X", buf, param->search_res.start_handle,
                 param->search_res.end_handle);
      }
      if (this->match_service_uuid_(param->search_res.srvc_id.uuid)) {
        this->service_start_handle_ = param->search_res.start_handle;
        this->service_end_handle_ = param->search_res.end_handle;
        this->service_search_requested_ = false;
        ESP_LOGI(TAG, "Energomera service discovered: 0x%04X-0x%04X", this->service_start_handle_,
                 this->service_end_handle_);
      }
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (param->search_cmpl.conn_id != this->parent_->get_conn_id())
        break;
      if (this->service_start_handle_ == 0) {
        if (!this->service_search_requested_) {
          esp_bt_uuid_t svc_uuid{};
          svc_uuid.len = ESP_UUID_LEN_16;
          svc_uuid.uuid.uuid16 = 0x0100;
          esp_err_t status =
              esp_ble_gattc_search_service(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), &svc_uuid);
          if (status == ESP_OK) {
            this->service_search_requested_ = true;
            ESP_LOGW(TAG, "Energomera service not found; requesting targeted search");
            break;
          }
          ESP_LOGW(TAG, "Energomera service search request failed: %d", status);
        } else {
          ESP_LOGW(TAG, "Energomera service not found during discovery");
        }
        break;
      }
      if (this->resolve_characteristics_())
        this->enable_notifications_if_needed_();
      this->request_firmware_version_();
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id())
        break;
      uint16_t handle = param->read.handle;
      if (handle == this->version_char_handle_) {
        this->version_requested_ = false;
        if (param->read.status != ESP_GATT_OK) {
          ESP_LOGW(TAG, "Firmware version read failed: %d", param->read.status);
          this->command_pending_ = true;
          this->command_complete_ = false;
          this->set_timeout("fw_retry", 300, [this]() {
            if (!this->version_reported_)
              this->request_firmware_version_();
          });
          this->try_send_pending_command_();
          break;
        }
        if (param->read.value_len == 0) {
          ESP_LOGW(TAG, "Firmware version characteristic returned empty value");
          this->command_pending_ = true;
          this->command_complete_ = false;
          this->set_timeout("fw_retry_empty", 300, [this]() {
            if (!this->version_reported_)
              this->request_firmware_version_();
          });
          this->try_send_pending_command_();
          break;
        }
        std::string hex_dump;
        hex_dump.reserve(param->read.value_len * 3);
        for (uint16_t i = 0; i < param->read.value_len; i++) {
          char buf[4];
          snprintf(buf, sizeof(buf), "%02X ", param->read.value[i]);
          hex_dump.append(buf);
        }
        ESP_LOGD(TAG, "Firmware characteristic raw (%u bytes): %s", param->read.value_len, hex_dump.c_str());

        std::string version(reinterpret_cast<const char *>(param->read.value),
                            reinterpret_cast<const char *>(param->read.value) + param->read.value_len);
        auto nul_pos = version.find('\0');
        if (nul_pos != std::string::npos)
          version.resize(nul_pos);
        ESP_LOGI(TAG, "Meter firmware version: %s", version.c_str());
        this->version_reported_ = true;
        this->command_pending_ = true;
        this->command_complete_ = false;
        this->enable_notifications_if_needed_();
        this->try_send_pending_command_();
        break;
      }

      if (!this->pending_response_handles_.empty() ||
          std::find(this->response_char_handles_.begin(), this->response_char_handles_.end(), handle) !=
              this->response_char_handles_.end()) {
        this->handle_command_read_(param->read);
      }
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent_->get_conn_id())
        break;
      if (param->notify.handle == this->tx_char_handle_)
        this->handle_notification_(param->notify);
      break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id())
        break;
      if (param->write.handle != this->tx_cccd_handle_)
        break;
      this->cccd_write_pending_ = false;
      if (param->write.status == ESP_GATT_OK) {
        this->notifications_enabled_ = true;
        this->notify_retry_attempts_ = 0;
        ESP_LOGD(TAG, "Notifications enabled on handle 0x%04X", param->write.handle);
        this->try_send_pending_command_();
      } else {
        ESP_LOGW(TAG, "Failed to enable notifications: %d", param->write.status);
        if (param->write.status == ESP_GATT_INSUF_AUTHENTICATION ||
            param->write.status == ESP_GATT_INSUF_AUTHORIZATION ||
            param->write.status == ESP_GATT_INSUF_ENCRYPTION) {
          uint32_t attempt = this->notify_retry_attempts_ == 0 ? 1 : this->notify_retry_attempts_;
          this->schedule_notification_retry_(300 + 200 * attempt);
        } else if (this->notify_retry_attempts_ < 5) {
          uint32_t attempt = this->notify_retry_attempts_ == 0 ? 1 : this->notify_retry_attempts_;
          this->schedule_notification_retry_(400 + 200 * attempt);
        }
      }
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      if (!this->parent_->check_addr(param->disconnect.remote_bda))
        break;
      ESP_LOGW(TAG, "GATT client disconnected: reason=0x%02X", param->disconnect.reason);
      this->version_requested_ = false;
      this->version_char_handle_ = 0;
      this->tx_char_handle_ = 0;
      this->tx_cccd_handle_ = 0;
      this->characteristics_resolved_ = false;
      this->notifications_enabled_ = false;
      this->cccd_write_pending_ = false;
      this->command_pending_ = false;
      this->command_inflight_ = false;
      this->command_complete_ = false;
      this->response_in_progress_ = false;
      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      this->notify_retry_attempts_ = 0;
      this->notification_failed_ = false;
      this->cancel_timeout("cccd_retry");
      this->cancel_timeout("polling_read_loop");
      std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
      this->pending_response_handles_.clear();
      this->response_buffer_.clear();
      this->tx_message_remaining_.clear();
      break;
    }
    default:
      break;
  }
}

void EnergomeraBleComponent::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
      if (!this->parent_->check_addr(param->ble_security.key_notif.bd_addr))
        break;
      ESP_LOGI(TAG, "Passkey notification: %06u", param->ble_security.key_notif.passkey);
      break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr))
        break;
      if (this->pin_code_.empty()) {
        ESP_LOGW(TAG, "Pairing requested but no PIN configured; replying with 000000");
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
      } else {
        uint32_t pin = static_cast<uint32_t>(std::strtoul(this->pin_code_.c_str(), nullptr, 10));
        ESP_LOGI(TAG, "Supplying PIN %06u", pin);
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, pin);
      }
      break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr))
        break;
      ESP_LOGD(TAG, "Security request received, confirming");
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      if (!this->parent_->check_addr(param->ble_security.auth_cmpl.bd_addr))
        break;
      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGI(TAG, "Pairing completed successfully");
        this->enable_notifications_if_needed_();
        if (!this->version_reported_)
          this->request_firmware_version_();
      } else {
        ESP_LOGE(TAG, "Pairing failed, status=%d", param->ble_security.auth_cmpl.fail_reason);
      }
      break;
    default:
      break;
  }
}

}  // namespace energomera_ble
}  // namespace esphome
