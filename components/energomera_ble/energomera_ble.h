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

  void set_passkey(uint32_t passkey) { this->passkey_ = passkey % 1000000U; };

  void set_meter_address(const std::string &address) { (void) address; }
  void set_receive_timeout_ms(uint32_t timeout) { (void) timeout; }
  void set_delay_between_requests_ms(uint32_t delay) { (void) delay; }
  void set_reboot_after_failure(uint16_t value) { (void) value; }


  void remove_bonding();

#ifdef USE_BINARY_SENSOR
  void set_indicator(binary_sensor::BinarySensor *indicator) { (void) indicator; }
#endif
#ifdef USE_TIME
  void set_time_source(time::RealTimeClock *time) { (void) time; }
#endif

 protected:
  void log_discovered_services_();

  void initiate_pairing_(const esp_bd_addr_t remote_bda);
  void request_firmware_version_();
  void sync_address_from_parent_();
  bool resolve_characteristics_();
  bool resolve_tx_descriptors_();

  enum class FsmState : uint8_t {
    IDLE,
    RESOLVING,
    REQUESTING_FIRMWARE,
    WAITING_FIRMWARE,
    ENABLING_NOTIFICATION,
    WAITING_NOTIFICATION_ENABLE,
    SENDING_COMMAND,
    WAITING_NOTIFICATION,
    READING_RESPONSE,
    COMPLETE,
    ERROR,
    DISCONNECTED
  };

  void set_state_(FsmState state);
  void enable_notifications_();
  void prepare_watch_command_();
  bool send_next_fragment_();
  void begin_response_reads_(uint8_t slots);
  void issue_next_response_read_();
  void handle_command_read_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param);
  void finalize_command_response_();
  void handle_notification_(const esp_ble_gattc_cb_param_t::gattc_notify_evt_param &param);
  uint16_t get_max_payload_() const;

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
  uint8_t tx_sequence_counter_{0};
  bool tx_fragment_started_{false};
  uint16_t mtu_{23};
  std::vector<uint8_t> tx_message_remaining_;
  std::vector<uint8_t> response_buffer_;
  uint8_t expected_response_slots_{0};
  uint8_t current_response_slot_{0};
  FsmState state_{FsmState::IDLE};
  bool link_encrypted_{false};
  //  std::string pin_code_;

  uint32_t passkey_{0};

  bool services_logged_{false};
};

}  // namespace energomera_ble
}  // namespace esphome
