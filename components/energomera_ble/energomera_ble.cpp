#include "energomera_ble.h"

#include "esphome/core/log.h"

#include <cstring>
#include <cstdlib>
#include <cstdio>

namespace esphome {
namespace energomera_ble {

static const char *const TAG = "energomera_ble";

static const uint8_t ENERGOMERA_SERVICE_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                        0xe2, 0x45, 0xef, 0x8b, 0x00, 0x01, 0x1b, 0xb9};
static const uint8_t ENERGOMERA_VERSION_UUID_128[16] = {0xdf, 0x14, 0xd9, 0x62, 0xd8, 0x1c, 0xc3, 0x97,
                                                        0xe2, 0x45, 0xef, 0x8b, 0x01, 0x01, 0x1b, 0xb9};

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

  if (this->version_char_handle_ == 0) {
    bool resolved = false;
    uint16_t count = 0;
    esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
        this->parent_->get_gattc_if(), this->parent_->get_conn_id(), ESP_GATT_DB_CHARACTERISTIC,
        this->service_start_handle_, this->service_end_handle_, ESP_GATT_INVALID_HANDLE, &count);
    if (status == ESP_GATT_OK && count > 0) {
      auto *char_elems = (esp_gattc_char_elem_t *) heap_caps_malloc(sizeof(esp_gattc_char_elem_t) * count, MALLOC_CAP_8BIT);
      if (char_elems != nullptr) {
        status = esp_ble_gattc_get_all_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                            this->service_start_handle_, this->service_end_handle_, char_elems, &count, 0);
        if (status == ESP_GATT_OK) {
          for (uint16_t i = 0; i < count; i++) {
            auto &elem = char_elems[i];
            if (elem.uuid.len == ESP_UUID_LEN_16) {
              ESP_LOGD(TAG, "Characteristic handle=0x%04X uuid16=0x%04X properties=0x%02X", elem.char_handle,
                       elem.uuid.uuid.uuid16, elem.properties);
              if (elem.uuid.uuid.uuid16 == 0x0101) {
                this->version_char_handle_ = elem.char_handle;
                resolved = true;
                break;
              }
            }
          }
        }
        free(char_elems);
      }
    }

    if (!resolved) {
      uint16_t fallback = this->service_start_handle_ + 1;  // characteristic value handle typically start+1
      if (fallback > this->service_start_handle_ && fallback < this->service_start_handle_ + 0x40) {
        this->version_char_handle_ = fallback;
        resolved = true;
        ESP_LOGW(TAG, "Firmware characteristic not enumerated, using fallback handle 0x%04X", fallback);
      }
    }

    if (!resolved) {
      ESP_LOGW(TAG, "Firmware version characteristic (0x0101) not found in service");
      return;
    }
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
      this->version_requested_ = false;
      this->version_reported_ = false;
      this->service_search_requested_ = false;
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
      this->request_firmware_version_();
      break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id())
        break;
      if (param->read.handle != this->version_char_handle_)
        break;
      this->version_requested_ = false;
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Firmware version read failed: %d", param->read.status);
        break;
      }
      if (param->read.value_len == 0) {
        ESP_LOGW(TAG, "Firmware version characteristic returned empty value");
        break;
      }
      std::string version(reinterpret_cast<const char *>(param->read.value),
                          reinterpret_cast<const char *>(param->read.value) + param->read.value_len);
      auto nul_pos = version.find('\0');
      if (nul_pos != std::string::npos)
        version.resize(nul_pos);
      ESP_LOGI(TAG, "Meter firmware version: %s", version.c_str());
      this->version_reported_ = true;
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      if (!this->parent_->check_addr(param->disconnect.remote_bda))
        break;
      ESP_LOGW(TAG, "GATT client disconnected: reason=0x%02X", param->disconnect.reason);
      this->version_requested_ = false;
      this->version_char_handle_ = 0;
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
