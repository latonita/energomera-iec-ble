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

static inline bool uuid_equals_128(const uint8_t *lhs, const uint8_t *rhs) { return std::memcmp(lhs, rhs, 16) == 0; }

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
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_BOND_MITM;  // Require MITM protection
  esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;                   // We can INPUT (receive PIN)
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t resp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  ESP_LOGI(TAG, "Setting BLE security parameters: auth_req=0x%02X, iocap=%d, passkey=%06u", 
           auth_req, iocap, this->passkey_);
  
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &resp_key, sizeof(resp_key));
}

void EnergomeraBleComponent::loop() {
  if (!this->address_set_)
    this->sync_address_from_parent_();
  //     IDLE,
  // RESOLVING,
  // REQUESTING_FIRMWARE,
  // WAITING_FIRMWARE,
  // ENABLING_NOTIFICATION,
  // WAITING_NOTIFICATION_ENABLE,
  // SENDING_COMMAND,
  // WAITING_NOTIFICATION,
  // READING_RESPONSE,
  // COMPLETE,
  // ERROR,
  // DISCONNECTED
  switch (this->state_) {
    case FsmState::IDLE: {
    } break;

    case FsmState::RESOLVING: {
      this->set_state_(FsmState::REQUESTING_FIRMWARE);
    } break;

    case FsmState::REQUESTING_FIRMWARE: {
      this->request_firmware_version_();
      this->set_state_(FsmState::WAITING_FIRMWARE);
    } break;

    case FsmState::WAITING_FIRMWARE: {
    } break;

    case FsmState::ENABLING_NOTIFICATION: {
    } break;

    case FsmState::WAITING_NOTIFICATION_ENABLE: {
    } break;

    case FsmState::SENDING_COMMAND: {
    } break;

    case FsmState::WAITING_NOTIFICATION: {
    } break;

    case FsmState::READING_RESPONSE: {
    } break;

    case FsmState::COMPLETE:
      // do nothing
      break;

    case FsmState::ERROR:
      // do nothing
      break;
  }
}

void EnergomeraBleComponent::update() {
  if (this->node_state == esp32_ble_tracker::ClientState::ESTABLISHED && !services_logged_) {
    log_discovered_services_();
    services_logged_ = true;
  }
}

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
  if (this->passkey_ == 0) {
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
    this->set_state_(FsmState::ERROR);
    return;
  }
  this->version_requested_ = true;
  this->set_state_(FsmState::WAITING_FIRMWARE);
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
  esp_gatt_status_t status = esp_ble_gattc_get_attr_count(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                                          ESP_GATT_DB_CHARACTERISTIC, this->service_start_handle_,
                                                          this->service_end_handle_, ESP_GATT_INVALID_HANDLE, &count);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_get_attr_count failed: %d", status);
  }

  if (status == ESP_GATT_OK && count > 0) {
    auto *char_elems =
        (esp_gattc_char_elem_t *) heap_caps_malloc(sizeof(esp_gattc_char_elem_t) * count, MALLOC_CAP_8BIT);
    if (char_elems != nullptr) {
      status =
          esp_ble_gattc_get_all_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                     this->service_start_handle_, this->service_end_handle_, char_elems, &count, 0);
      if (status == ESP_GATT_OK) {
        for (uint16_t i = 0; i < count; i++) {
          auto &elem = char_elems[i];
          uint16_t handle = elem.char_handle;
          if (elem.uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGD(TAG, "Characteristic handle=0x%04X uuid16=0x%04X properties=0x%02X", handle, elem.uuid.uuid.uuid16,
                     elem.properties);
            if (elem.uuid.uuid.uuid16 == 0x0101) {
              this->version_char_handle_ = handle;
            }
          } else if (elem.uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGD(TAG,
                     "Characteristic handle=0x%04X "
                     "uuid128=%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X properties=0x%02X",
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
  esp_gatt_status_t status = esp_ble_gattc_get_attr_count(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                                          ESP_GATT_DB_DESCRIPTOR, this->service_start_handle_,
                                                          this->service_end_handle_, this->tx_char_handle_, &count);
  if (status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_get_attr_count(descr) failed: %d", status);
  }

  if (status == ESP_GATT_OK && count > 0) {
    auto *descr_elems =
        (esp_gattc_descr_elem_t *) heap_caps_malloc(sizeof(esp_gattc_descr_elem_t) * count, MALLOC_CAP_8BIT);
    if (descr_elems != nullptr) {
      status = esp_ble_gattc_get_all_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                           this->tx_char_handle_, descr_elems, &count, 0);
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
    this->tx_cccd_handle_ = 0x0020; //this->tx_char_handle_ + 1;
    ESP_LOGW(TAG, "TX CCCD not enumerated, assuming handle 0x%04X", this->tx_cccd_handle_);
  }

  return this->tx_cccd_handle_ != 0;
}

void EnergomeraBleComponent::set_state_(FsmState state) { this->state_ = state; }

void EnergomeraBleComponent::enable_notifications_() {
  if (this->notifications_enabled_)
    return;
  if (!this->link_encrypted_) {
    ESP_LOGD(TAG, "Link not encrypted yet; deferring notification enable");
    this->set_state_(FsmState::WAITING_NOTIFICATION_ENABLE);
    return;
  }
  if (this->parent_ == nullptr || this->tx_char_handle_ == 0 || this->tx_cccd_handle_ == 0) {
    ESP_LOGW(TAG, "Cannot enable notifications: required handles missing");
    this->set_state_(FsmState::ERROR);
    return;
  }

  esp_err_t status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                       this->tx_char_handle_);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed: %d (continuing)", status);
  }

  uint16_t notify_en = 0x0001;
  auto err = esp_ble_gattc_write_char_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                            this->tx_cccd_handle_, sizeof(notify_en), (uint8_t *) &notify_en,
                                            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_MITM);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to enable notifications on handle 0x%04X: %d", this->tx_cccd_handle_, err);
    this->set_state_(FsmState::ERROR);
    return;
  }

  ESP_LOGD(TAG, "Notification enable request sent (handle 0x%04X)", this->tx_cccd_handle_);
  this->set_state_(FsmState::WAITING_NOTIFICATION_ENABLE);
}

void EnergomeraBleComponent::prepare_watch_command_() {
  static const uint8_t RAW_COMMAND[] = {0x2F, 0x3F, 0x21, 0x01, 0x52, 0x31, 0x02, 0x45,
                                        0x54, 0x30, 0x50, 0x45, 0x28, 0x29, 0x03, 0x00};
  this->tx_message_remaining_.assign(RAW_COMMAND, RAW_COMMAND + sizeof(RAW_COMMAND));
  for (auto &byte : this->tx_message_remaining_) {
    byte = apply_even_parity(byte & 0x7F);
  }
  this->tx_fragment_started_ = false;
  this->tx_sequence_counter_ = 0;
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

  esp_err_t status =
      esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->tx_char_handle_,
                               packet.size(), packet.data(), ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
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
  } else {
    this->set_state_(FsmState::WAITING_NOTIFICATION);
  }

  return true;
}

void EnergomeraBleComponent::begin_response_reads_(uint8_t slot_count) {
  uint8_t slots = slot_count + 1;
  if (slots == 0)
    slots = 1;
  if (slots > this->response_char_handles_.size())
    slots = this->response_char_handles_.size();

  this->expected_response_slots_ = slots;
  this->current_response_slot_ = 0;
  this->response_buffer_.clear();

  this->set_state_(FsmState::READING_RESPONSE);
  this->issue_next_response_read_();
}

void EnergomeraBleComponent::issue_next_response_read_() {
  if (this->current_response_slot_ >= this->expected_response_slots_) {
    this->finalize_command_response_();
    return;
  }

  uint16_t handle = this->response_char_handles_[this->current_response_slot_];
  if (handle == 0) {
    ESP_LOGW(TAG, "Response characteristic index %u not resolved", this->current_response_slot_);
    this->current_response_slot_++;
    this->issue_next_response_read_();
    return;
  }

  esp_err_t status = esp_ble_gattc_read_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), handle,
                                             ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "Failed to request response read (handle 0x%04X): %d", handle, status);
    this->set_state_(FsmState::ERROR);
  }
}

void EnergomeraBleComponent::handle_command_read_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param) {
  if (param.status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "Response read failed (handle 0x%04X): %d", param.handle, param.status);
    this->set_state_(FsmState::ERROR);
    return;
  }

  for (uint16_t i = 0; i < param.value_len; i++) {
    this->response_buffer_.push_back(param.value[i] & 0x7F);
  }

  this->current_response_slot_++;
  this->issue_next_response_read_();
}

void EnergomeraBleComponent::finalize_command_response_() {
  this->set_state_(FsmState::COMPLETE);

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
  this->response_buffer_.clear();
  this->set_state_(FsmState::WAITING_NOTIFICATION);
}

void EnergomeraBleComponent::handle_notification_(const esp_ble_gattc_cb_param_t::gattc_notify_evt_param &param) {
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
  this->begin_response_reads_(slot_count);
}

void EnergomeraBleComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                                 esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      if (!this->parent_->check_addr(param->connect.remote_bda))
        break;
      ESP_LOGI(TAG, "GATT client connected");
      this->link_encrypted_ = false;
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
      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      this->tx_message_remaining_.clear();
      this->response_buffer_.clear();
      this->expected_response_slots_ = 0;
      this->current_response_slot_ = 0;
      std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
      // this->set_state_(FsmState::RESOLVING);
      this->sync_address_from_parent_();
      
      // Copy remote address safely before timeout
      std::array<uint8_t, 6> remote_addr;
      std::memcpy(remote_addr.data(), param->connect.remote_bda, 6);
      
      if (this->passkey_ != 0) {
        ESP_LOGI(TAG, "Will initiate MITM pairing with PIN %06u", this->passkey_);
        
        // Remove existing bond to force fresh pairing with PIN
        ESP_LOGI(TAG, "Removing existing bond to force fresh pairing");
        esp_ble_remove_bond_device(const_cast<uint8_t*>(remote_addr.data()));
        
        this->set_timeout(100, [this, remote_addr]() {
          ESP_LOGI(TAG, "Initiating encrypted link with MITM protection");
          esp_err_t result = esp_ble_set_encryption(const_cast<uint8_t*>(remote_addr.data()), ESP_BLE_SEC_ENCRYPT_MITM);
          ESP_LOGI(TAG, "esp_ble_set_encryption result: %d", result);
        });
      } else {
        ESP_LOGW(TAG, "No PIN configured - pairing will rely on existing bond or fail");
      }

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
    case ESP_GATTC_CLOSE_EVT: {
      if (param->search_cmpl.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGD(TAG, "GATT connection closed (conn_id=%d)", param->close.conn_id);
      this->set_state_(FsmState::DISCONNECTED);

    } break;

    case ESP_GATTC_SEARCH_RES_EVT: {
      if (param->search_res.conn_id != this->parent_->get_conn_id())
        break;
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
      ESP_LOGI(TAG, "Service discovery completed");
      ESP_LOGI(TAG, "Connection state: connected=%s, paired=%s, encrypted=%s",
               this->parent_->connected() ? "YES" : "NO",
               this->parent_->is_paired() ? "YES" : "NO", 
               this->link_encrypted_ ? "YES" : "NO");
      
      // Log services immediately before they might be released
      log_discovered_services_();
      
      // Don't set node_state to ESTABLISHED yet - keep services alive
      // this->node_state = esp32_ble_tracker::ClientState::ESTABLISHED;
      this->services_logged_ = false;  // Reset flag to log services on next update
      this->set_state_(FsmState::RESOLVING);
      // if (this->service_start_handle_ == 0) {
      //   if (!this->service_search_requested_) {
      //     esp_bt_uuid_t svc_uuid{};
      //     svc_uuid.len = ESP_UUID_LEN_16;
      //     svc_uuid.uuid.uuid16 = 0x0100;
      //     if (esp_ble_gattc_search_service(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), &svc_uuid) ==
      //         ESP_OK) {
      //       this->service_search_requested_ = true;
      //       ESP_LOGW(TAG, "Energomera service not found; requesting targeted search");
      //       break;
      //     }
      //   }
      //   ESP_LOGW(TAG, "Energomera service not found during discovery");
      //   this->set_state_(FsmState::ERROR);
      //   break;
      // }
      // if (!this->resolve_characteristics_()) {
      //   this->set_state_(FsmState::ERROR);
      //   break;
      // }
      // this->set_state_(FsmState::REQUESTING_FIRMWARE);
      // this->request_firmware_version_();
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id())
        break;
      if (param->read.handle == this->version_char_handle_) {
        this->version_requested_ = false;
        if (param->read.status != ESP_GATT_OK || param->read.value_len == 0) {
          ESP_LOGW(TAG, "Firmware version read failed: %d", param->read.status);
          this->set_state_(FsmState::ERROR);
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
        this->set_state_(FsmState::ENABLING_NOTIFICATION);
        this->set_timeout("enable_notify", 500, [this]() { this->enable_notifications_(); });
        break;
      }
      if (this->state_ == FsmState::READING_RESPONSE)
        this->handle_command_read_(param->read);
      break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id())
        break;
      // if (param->write.handle != this->tx_cccd_handle_)
      //   break;
      // if (param->write.status != ESP_GATT_OK) {
      //   ESP_LOGW(TAG, "Failed to enable notifications: %d", param->write.status);
      //   this->set_state_(FsmState::ERROR);
      //   break;
      // }
      // this->notifications_enabled_ = true;
      // ESP_LOGD(TAG, "Notifications enabled on handle 0x%04X", param->write.handle);
      // this->prepare_watch_command_();
      // if (this->tx_message_remaining_.empty()) {
      //   ESP_LOGW(TAG, "No command payload prepared");
      //   this->set_state_(FsmState::ERROR);
      //   break;
      // }
      // ESP_LOGI(TAG, "Sending watch command (%u bytes payload)", (unsigned) this->tx_message_remaining_.size());
      // this->set_state_(FsmState::SENDING_COMMAND);
      // if (!this->send_next_fragment_())
      //   this->set_state_(FsmState::ERROR);
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent_->get_conn_id())
        break;
      // if (param->notify.handle == this->tx_char_handle_ && this->state_ == FsmState::WAITING_NOTIFICATION)
      //   this->handle_notification_(param->notify);
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      if (!this->parent_->check_addr(param->disconnect.remote_bda))
        break;
      ESP_LOGW(TAG, "GATT client disconnected: reason=0x%02X", param->disconnect.reason);
      this->link_encrypted_ = false;
      this->version_requested_ = false;
      this->version_char_handle_ = 0;
      this->tx_char_handle_ = 0;
      this->tx_cccd_handle_ = 0;
      this->characteristics_resolved_ = false;
      this->notifications_enabled_ = false;
      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      this->tx_message_remaining_.clear();
      this->response_buffer_.clear();
      this->expected_response_slots_ = 0;
      this->current_response_slot_ = 0;
      this->cancel_timeout("enable_notify");
      std::fill(this->response_char_handles_.begin(), this->response_char_handles_.end(), 0);
      this->set_state_(FsmState::IDLE);
      break;
    }
    default:
      break;
  }
}

void EnergomeraBleComponent::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  ESP_LOGI(TAG, "GAP Event: %d", event);
  
  switch (event) {
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
      if (!this->parent_->check_addr(param->ble_security.key_notif.bd_addr)) {
        ESP_LOGW(TAG, "Passkey notification for wrong device - ignoring");
        break;
      }
      ESP_LOGE(TAG, "*** Passkey notification: %06u ***", param->ble_security.key_notif.passkey);
      break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        ESP_LOGW(TAG, "Passkey request for wrong device - ignoring");
        break;
      }
      ESP_LOGE(TAG, "*** Passkey request - supplying PIN %06u ***", this->passkey_);
      esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, this->passkey_);
      break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        ESP_LOGW(TAG, "Security request for wrong device - ignoring");
        break;
      }
      ESP_LOGE(TAG, "*** Security request received, confirming ***");
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;

    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
      // Note: This event might not have bd_addr in param structure
      ESP_LOGE(TAG, "*** Bond removal completed ***");
      break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      if (!this->parent_->check_addr(param->ble_security.auth_cmpl.bd_addr)) {
        ESP_LOGW(TAG, "Auth completion for wrong device - ignoring");
        break;
      }

      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGE(TAG, "*** Pairing completed successfully ***");
        ESP_LOGI(TAG, "Auth mode: 0x%02X, Key present: 0x%02X", 
                 param->ble_security.auth_cmpl.auth_mode,
                 param->ble_security.auth_cmpl.key_present);
        this->set_timeout(500, [this]() {
          this->link_encrypted_ = true;
          ESP_LOGI(TAG, "Link encryption status updated");
          // this->set_state_(FsmState::RESOLVING);
        });

        // if (!this->version_reported_)
        //   this->request_firmware_version_();

        // else if (this->state_ == FsmState::WAITING_NOTIFICATION_ENABLE)
        //   this->set_timeout("enable_notify", 100, [this]() { this->enable_notifications_(); });
      } else {
        ESP_LOGE(TAG, "*** Pairing FAILED, reason=%d ***", param->ble_security.auth_cmpl.fail_reason);
        this->link_encrypted_ = false;
      }
      break;
    default:
      break;
  }
}

void EnergomeraBleComponent::log_discovered_services_() {
  ESP_LOGI(TAG, "=== Discovered BLE Services ===");

  // Try common service UUIDs
  struct {
    uint16_t uuid;
    const char *name;
  } common_services[] = {
      {0x1800, "Generic Access"},         {0x1801, "Generic Attribute"},
      {0x180A, "Device Information"},     {0x180F, "Battery Service"},
      {0x1812, "Human Interface Device"}, {0x181A, "Environmental Sensing"},
      {0x181B, "Body Composition"},       {0x181C, "User Data"},
      {0x181D, "Weight Scale"},
  };

  for (auto &svc : common_services) {
    auto *service = this->parent_->get_service(svc.uuid);
    if (service != nullptr) {
      ESP_LOGI(TAG, "  %s (0x%04X): handles 0x%04X-0x%04X", svc.name, svc.uuid, service->start_handle,
               service->end_handle);

      // Parse characteristics if not already done
      if (!service->parsed) {
        service->parse_characteristics();
      }

      // Log characteristics
      for (auto *chr : service->characteristics) {
        ESP_LOGI(TAG, "    Characteristic: %s (handle: 0x%04X, props: 0x%02X)", chr->uuid.to_string().c_str(),
                 chr->handle, chr->properties);
      }
    }
  }

  static const esphome::esp32_ble_tracker::ESPBTUUID ENERGOMERA_SERVICE_UUID =
      esphome::esp32_ble_tracker::ESPBTUUID::from_raw("b91b0100-8bef-45e2-97c3-1cd862d914df");
  static const esphome::esp32_ble_tracker::ESPBTUUID ENERGOMERA_VERSION_UUID =
      esphome::esp32_ble_tracker::ESPBTUUID::from_raw("b91b0101-8bef-45e2-97c3-1cd862d914df");
  static const esphome::esp32_ble_tracker::ESPBTUUID ENERGOMERA_TX_UUID =
      esphome::esp32_ble_tracker::ESPBTUUID::from_raw("b91b0105-8bef-45e2-97c3-1cd862d914df");

  auto *energomera_service = this->parent_->get_service(ENERGOMERA_SERVICE_UUID);
  if (energomera_service != nullptr) {
    ESP_LOGI(TAG, "  Energomera Service (%s): handles 0x%04X-0x%04X", ENERGOMERA_SERVICE_UUID.to_string().c_str(),
             energomera_service->start_handle, energomera_service->end_handle);

    uint16_t start_handle = energomera_service->start_handle;
    uint16_t end_handle = energomera_service->end_handle;
    
    // If end_handle is 0xFFFF, use a reasonable range
    if (end_handle == 0xFFFF) {
      ESP_LOGW(TAG, "Service end_handle is 0xFFFF - using reasonable range");
      end_handle = start_handle + 0x30;  // Conservative range
      ESP_LOGI(TAG, "Using calculated end_handle: 0x%04X", end_handle);
    }
    
    // FORCE characteristic discovery using ESP-IDF directly
    ESP_LOGI(TAG, "Manual characteristic discovery in range 0x%04X-0x%04X", start_handle, end_handle);
    
    // Get characteristic count first
    uint16_t char_count = 0;
    esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
        this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
        ESP_GATT_DB_CHARACTERISTIC, start_handle, end_handle, 
        ESP_GATT_INVALID_HANDLE, &char_count);
    
    ESP_LOGI(TAG, "Characteristic count query: status=%d, count=%d", status, char_count);
    
    if (status == ESP_GATT_OK && char_count > 0) {
      // Allocate buffer for all characteristics
      esp_gattc_char_elem_t *char_elems = (esp_gattc_char_elem_t*)malloc(sizeof(esp_gattc_char_elem_t) * char_count);
      if (char_elems != nullptr) {
        status = esp_ble_gattc_get_all_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                            start_handle, end_handle, char_elems, &char_count, 0);
        
        ESP_LOGI(TAG, "get_all_char: status=%d, returned_count=%d", status, char_count);
        
        if (status == ESP_GATT_OK) {
          for (uint16_t i = 0; i < char_count; i++) {
            esp_gattc_char_elem_t *elem = &char_elems[i];
            
            if (elem->uuid.len == ESP_UUID_LEN_16) {
              ESP_LOGI(TAG, "  Char[%d]: UUID16=0x%04X, handle=0x%04X, props=0x%02X",
                      i, elem->uuid.uuid.uuid16, elem->char_handle, elem->properties);
            } else if (elem->uuid.len == ESP_UUID_LEN_128) {
              ESP_LOGI(TAG, "  Char[%d]: UUID128=%02X%02X%02X%02X-..., handle=0x%04X, props=0x%02X",
                      i, elem->uuid.uuid.uuid128[15], elem->uuid.uuid.uuid128[14], 
                      elem->uuid.uuid.uuid128[13], elem->uuid.uuid.uuid128[12],
                      elem->char_handle, elem->properties);
              
              // Check for your specific characteristics
              if (memcmp(elem->uuid.uuid.uuid128, ENERGOMERA_VERSION_UUID_128, 16) == 0) {
                ESP_LOGI(TAG, "    ✓ FOUND VERSION CHARACTERISTIC at handle 0x%04X", elem->char_handle);
              } else if (memcmp(elem->uuid.uuid.uuid128, ENERGOMERA_TX_UUID_128, 16) == 0) {
                ESP_LOGI(TAG, "    ✓ FOUND TX CHARACTERISTIC at handle 0x%04X", elem->char_handle);
              }
            }
          }
        }
        free(char_elems);
      }
    } else {
      ESP_LOGW(TAG, "No characteristics found using direct ESP-IDF calls");
    }
    
    // Also try calling the base class parse_characteristics()
    if (!energomera_service->parsed) {
      ESP_LOGI(TAG, "Trying base class parse_characteristics()");
      energomera_service->parse_characteristics();
    }
    
    ESP_LOGI(TAG, "Service now has %d characteristics", energomera_service->characteristics.size());
    for (auto *chr : energomera_service->characteristics) {
      ESP_LOGI(TAG, "  Characteristic: %s (handle: 0x%04X, props: 0x%02X)", 
               chr->uuid.to_string().c_str(), chr->handle, chr->properties);
    }

    // Check for specific characteristics
    auto *version_char = energomera_service->get_characteristic(ENERGOMERA_VERSION_UUID);
    if (version_char != nullptr) {
      ESP_LOGI(TAG, "✓ Version Characteristic found at handle 0x%04X", version_char->handle);
    } else {
      ESP_LOGI(TAG, "✗ Version Characteristic NOT FOUND");
    }
    
    auto *tx_char = energomera_service->get_characteristic(ENERGOMERA_TX_UUID);
    if (tx_char != nullptr) {
      ESP_LOGI(TAG, "✓ TX Characteristic found at handle 0x%04X", tx_char->handle);
    } else {
      ESP_LOGI(TAG, "✗ TX Characteristic NOT FOUND");
    }



  } else {
    ESP_LOGI(TAG, "  Energomera Service (%s): NOT FOUND", ENERGOMERA_SERVICE_UUID.to_string().c_str());
  }

  ESP_LOGI(TAG, "=== End of Services ===");
}

}  // namespace energomera_ble
}  // namespace esphome
