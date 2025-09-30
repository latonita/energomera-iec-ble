#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include <esp_gattc_api.h>

#include <array>
#include <string>

namespace esphome {
namespace energomera_ble {

class EnergomeraBleComponent : public PollingComponent, public ble_client::BLEClientNode {
 public:
  EnergomeraBleComponent();

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_pin_code(const std::string &pin_code) { this->pin_code_ = pin_code; }
  void set_passkey(uint32_t passkey);
  void set_meter_address(const std::string &address) { (void) address; }
  void set_receive_timeout_ms(uint32_t timeout) { (void) timeout; }
  void set_delay_between_requests_ms(uint32_t delay) { (void) delay; }
  void set_reboot_after_failure(uint16_t value) { (void) value; }

#ifdef USE_BINARY_SENSOR
  void set_indicator(binary_sensor::BinarySensor *indicator) { (void) indicator; }
#endif
#ifdef USE_TIME
  void set_time_source(time::RealTimeClock *time) { (void) time; }
#endif

 protected:
  void initiate_pairing_(const esp_bd_addr_t remote_bda);
  void request_firmware_version_();
  void sync_address_from_parent_();

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  bool match_service_uuid_(const esp_bt_uuid_t &uuid) const;

  // Cached handles
  uint16_t service_start_handle_{0};
  uint16_t service_end_handle_{0};
  uint16_t version_char_handle_{0};

  std::array<uint8_t, 6> target_address_{};
  bool address_set_{false};
  bool version_requested_{false};
  bool version_reported_{false};
  bool service_search_requested_{false};
  std::string pin_code_;
};

}  // namespace energomera_ble
}  // namespace esphome
