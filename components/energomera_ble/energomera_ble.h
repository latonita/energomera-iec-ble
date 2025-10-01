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
#include <vector>

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
  bool resolve_characteristics_();
  bool resolve_tx_descriptors_();
  void enable_notifications_if_needed_();
  void prepare_et0pe_command_();
  void try_send_pending_command_();
  bool send_next_fragment_();
  void start_response_sequence_(uint8_t slot_count);
  void issue_next_response_read_();
  void handle_command_read_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param);
  void finalize_command_response_();
  void handle_notification_(const esp_ble_gattc_cb_param_t::gattc_notify_evt_param &param);
  uint16_t get_max_payload_() const;
  void schedule_notification_retry_(uint32_t delay_ms);

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  bool match_service_uuid_(const esp_bt_uuid_t &uuid) const;

  // Cached handles
  uint16_t service_start_handle_{0};
  uint16_t service_end_handle_{0};
  uint16_t version_char_handle_{0};
  uint16_t tx_char_handle_{0};
  uint16_t tx_cccd_handle_{0};
  std::array<uint16_t, 16> response_char_handles_{};

  std::array<uint8_t, 6> target_address_{};
  bool address_set_{false};
  bool version_requested_{false};
  bool version_reported_{false};
  bool service_search_requested_{false};
  bool characteristics_resolved_{false};
  bool notifications_enabled_{false};
  bool cccd_write_pending_{false};
  bool command_pending_{false};
  bool command_inflight_{false};
  bool command_complete_{false};
  bool response_in_progress_{false};
  uint8_t tx_sequence_counter_{0};
  bool tx_fragment_started_{false};
  uint16_t mtu_{23};
  uint8_t notify_retry_attempts_{0};
  std::vector<uint8_t> tx_message_remaining_;
  std::vector<uint16_t> pending_response_handles_;
  std::vector<uint8_t> response_buffer_;
  uint16_t current_response_handle_{0};
  std::string pin_code_;
};

}  // namespace energomera_ble
}  // namespace esphome
