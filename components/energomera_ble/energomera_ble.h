#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <esp_gattc_api.h>

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include <cstdint>
#include <string>
#include <memory>
#include <map>
#include <list>

#include "energomera_ble_sensor.h"

namespace esphome {
namespace energomera_ble {

static const size_t MAX_IN_BUF_SIZE = 256;
static const size_t MAX_OUT_BUF_SIZE = 84;

const uint8_t VAL_NUM = 12;
using ValueRefsArray = std::array<char *, VAL_NUM>;

using SensorMap = std::multimap<std::string, EnergomeraBleSensorBase *>;
using SingleRequests = std::list<std::string>;

using FrameStopFunction = std::function<bool(uint8_t *buf, size_t size)>;
using ReadFunction = std::function<size_t()>;

/*

handle = 0x0002, char properties = 0x02, char value handle = 0x0003, uuid = 00002a00-0000-1000-8000-00805f9b34fb
handle = 0x0004, char properties = 0x02, char value handle = 0x0005, uuid = 00002a01-0000-1000-8000-00805f9b34fb
handle = 0x0006, char properties = 0x02, char value handle = 0x0007, uuid = 00002a04-0000-1000-8000-00805f9b34fb
handle = 0x000a, char properties = 0x02, char value handle = 0x000b, uuid = 00002a23-0000-1000-8000-00805f9b34fb
handle = 0x000c, char properties = 0x02, char value handle = 0x000d, uuid = 00002a24-0000-1000-8000-00805f9b34fb
handle = 0x000e, char properties = 0x02, char value handle = 0x000f, uuid = 00002a25-0000-1000-8000-00805f9b34fb
handle = 0x0010, char properties = 0x02, char value handle = 0x0011, uuid = 00002a26-0000-1000-8000-00805f9b34fb
handle = 0x0012, char properties = 0x02, char value handle = 0x0013, uuid = 00002a27-0000-1000-8000-00805f9b34fb
handle = 0x0014, char properties = 0x02, char value handle = 0x0015, uuid = 00002a28-0000-1000-8000-00805f9b34fb
handle = 0x0016, char properties = 0x02, char value handle = 0x0017, uuid = 00002a29-0000-1000-8000-00805f9b34fb
handle = 0x0018, char properties = 0x02, char value handle = 0x0019, uuid = 00002a2a-0000-1000-8000-00805f9b34fb
handle = 0x001a, char properties = 0x02, char value handle = 0x001b, uuid = 00002a50-0000-1000-8000-00805f9b34fb
handle = 0x001d, char properties = 0x02, char value handle = 0x001e, uuid = b91b0101-8bef-45e2-97c3-1cd862d914df
handle = 0x0020, char properties = 0x5a, char value handle = 0x0021, uuid = b91b0105-8bef-45e2-97c3-1cd862d914df
handle = 0x0024, char properties = 0x02, char value handle = 0x0025, uuid = b91b0106-8bef-45e2-97c3-1cd862d914df
handle = 0x0027, char properties = 0x02, char value handle = 0x0028, uuid = b91b0107-8bef-45e2-97c3-1cd862d914df
handle = 0x002a, char properties = 0x02, char value handle = 0x002b, uuid = b91b0108-8bef-45e2-97c3-1cd862d914df
handle = 0x002d, char properties = 0x02, char value handle = 0x002e, uuid = b91b0109-8bef-45e2-97c3-1cd862d914df
handle = 0x0030, char properties = 0x02, char value handle = 0x0031, uuid = b91b010a-8bef-45e2-97c3-1cd862d914df
handle = 0x0033, char properties = 0x02, char value handle = 0x0034, uuid = b91b010b-8bef-45e2-97c3-1cd862d914df
handle = 0x0036, char properties = 0x02, char value handle = 0x0037, uuid = b91b010c-8bef-45e2-97c3-1cd862d914df
handle = 0x0039, char properties = 0x02, char value handle = 0x003a, uuid = b91b010d-8bef-45e2-97c3-1cd862d914df
handle = 0x003c, char properties = 0x02, char value handle = 0x003d, uuid = b91b010e-8bef-45e2-97c3-1cd862d914df
handle = 0x003f, char properties = 0x02, char value handle = 0x0040, uuid = b91b010f-8bef-45e2-97c3-1cd862d914df
handle = 0x0042, char properties = 0x02, char value handle = 0x0043, uuid = b91b0110-8bef-45e2-97c3-1cd862d914df
handle = 0x0045, char properties = 0x02, char value handle = 0x0046, uuid = b91b0111-8bef-45e2-97c3-1cd862d914df
handle = 0x0048, char properties = 0x02, char value handle = 0x0049, uuid = b91b0112-8bef-45e2-97c3-1cd862d914df
handle = 0x004b, char properties = 0x02, char value handle = 0x004c, uuid = b91b0113-8bef-45e2-97c3-1cd862d914df
handle = 0x004e, char properties = 0x02, char value handle = 0x004f, uuid = b91b0114-8bef-45e2-97c3-1cd862d914df

*/




// static const char *SERVICE_UUID = "b91b0100-8bef-45e2-97c3-1cd862d914df";
// static const char *TX_CHAR_UUID = "b91b0105-8bef-45e2-97c3-1cd862d914df";
// static const char *RX_CHAR_UUID = "b91b0106-8bef-45e2-97c3-1cd862d914df";
// namespace espbt = esphome::esp32_ble_tracker;

// static const espbt::ESPBTUUID SERVICE_UUID = espbt::ESPBTUUID::from_raw("b91b0100-8bef-45e2-97c3-1cd862d914df");
// static const espbt::ESPBTUUID CHAR_TX_UUID = espbt::ESPBTUUID::from_raw("b91b0105-8bef-45e2-97c3-1cd862d914df");
// static const espbt::ESPBTUUID CHAR_RX_UUID = espbt::ESPBTUUID::from_raw("b91b0106-8bef-45e2-97c3-1cd862d914df");


class EnergomeraBleComponent : public PollingComponent, public ble_client::BLEClientNode {
 public:
  EnergomeraBleComponent();

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_pin_code(const std::string &pin_code) { this->pin_code_ = pin_code; }

 protected:
  enum class ClientState : uint8_t {
    IDLE,
    SCANNING,
    CONNECTING,
    PAIRING,
    DISCOVERING,
    ENABLING_NOTIFICATIONS,
    READY,
    ERROR,
  };

  void change_state_(ClientState next_state);
  void start_scan_();
  void stop_scan_();
  void request_connect_(); // uses parent BLE client address
  void initiate_pairing_();
  void start_service_discovery_();
  void enable_notifications_();
  void reset_with_backoff_(const char *reason = nullptr);
  void sync_address_from_parent_();

  // BLEClientNode overrides
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  // Helpers for UUID matching
  bool match_service_uuid_(const esp_bt_uuid_t &uuid) const;
  bool match_characteristic_uuid_(const esp_bt_uuid_t &uuid, const esp_bt_uuid_t &target) const;

  // Cached handles
  uint16_t service_start_handle_{0};
  uint16_t service_end_handle_{0};
  uint16_t tx_char_handle_{0};
  uint16_t notify_cccd_handle_{0};

  ClientState state_{ClientState::IDLE};
  std::array<uint8_t, 6> target_address_{};
  bool address_set_{false};
  std::string pin_code_;
  uint32_t state_deadline_{0};
  uint8_t retry_count_{0};

  static constexpr uint32_t kScanDurationMs = 15000;
  static constexpr uint32_t kOperationTimeoutMs = 30000;
  static constexpr uint8_t kMaxRetries = 5;
};


/*
class EnergomeraBleComponent : public PollingComponent, public ble_client::BLEClientNode {
 public:
  EnergomeraBleComponent() : tag_(generateTag()){};

  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::BLUETOOTH; };

  void set_meter_address(const std::string &addr) { this->meter_address_ = addr; };
  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; };
  void set_delay_between_requests_ms(uint32_t delay) { this->delay_between_requests_ms_ = delay; };

  void register_sensor(EnergomeraBleSensorBase *sensor);
  void set_reboot_after_failure(uint16_t number_of_failures) { this->failures_before_reboot_ = number_of_failures; }

  void queue_single_read(const std::string &req);

  virtual void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) override;
  virtual void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

#ifdef USE_TIME
  void set_time_source(time::RealTimeClock *rtc) { this->time_source_ = rtc; };
  void sync_device_time();  // set current time from RTC
#endif
  void set_device_time(uint32_t timestamp);  // set time from given timestamp


  // BLE starts //////////////////////////////////////////////////////////////////////////////////////////
  void set_passkey(uint32_t passkey) { this->passkey_ = passkey; };
  void set_pin_code(const std::string &pin_code) { this->pin_code_ = pin_code; }
  void change_state_(ClientState next_state);
  void start_scan_();
  void stop_scan_();
  void request_connect_(); // uses parent BLE client address
  void initiate_pairing_();
  void start_service_discovery_();
  void enable_notifications_();
  void reset_with_backoff_(const char *reason = nullptr);
  void sync_address_from_parent_();

  // BLEClientNode overrides
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  // Helpers for UUID matching
  bool match_service_uuid_(const esp_bt_uuid_t &uuid) const;
  bool match_characteristic_uuid_(const esp_bt_uuid_t &uuid, const esp_bt_uuid_t &target) const;

  // Cached handles
  uint16_t service_start_handle_{0};
  uint16_t service_end_handle_{0};
  uint16_t tx_char_handle_{0};
  uint16_t notify_cccd_handle_{0};

  ClientState state_{ClientState::IDLE};
  std::array<uint8_t, 6> target_address_{};
  bool address_set_{false};
  std::string pin_code_;
  uint32_t state_deadline_{0};
  uint8_t retry_count_{0};

  static constexpr uint32_t kScanDurationMs = 15000;
  static constexpr uint32_t kOperationTimeoutMs = 30000;
  static constexpr uint8_t kMaxRetries = 5;


  // BLE ends //////////////////////////////////////////////////////////////////////////////////////////

 protected:
  uint32_t passkey_{0};
  std::string meter_address_{""};
  uint32_t receive_timeout_ms_{500};
  uint32_t delay_between_requests_ms_{50};


#ifdef USE_TIME
  time::RealTimeClock *time_source_{nullptr};
#endif

  SensorMap sensors_;
  SingleRequests single_requests_;

  sensor::Sensor *crc_errors_per_session_sensor_{};

  uint32_t time_to_set_{0};
  uint32_t time_to_set_requested_at_ms_{0};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    TRY_LOCK_BUS,
    WAIT,
    WAITING_FOR_RESPONSE,
    OPEN_SESSION,
    OPEN_SESSION_GET_ID,
    SET_BAUD,
    ACK_START_GET_INFO,
    GET_DATE,
    GET_TIME,
    CORRECT_TIME,
    RECV_CORRECTION_RESULT,
    DATA_ENQ,
    DATA_RECV,
    DATA_NEXT,
    CLOSE_SESSION,
    PUBLISH,
    SINGLE_READ,
    SINGLE_READ_ACK,
  } state_{State::NOT_INITIALIZED};
  State last_reported_state_{State::NOT_INITIALIZED};

  struct {
    uint32_t start_time{0};
    uint32_t delay_ms{0};
    State next_state{State::IDLE};
  } wait_;

  bool is_idling() const { return this->state_ == State::WAIT || this->state_ == State::IDLE; };

  void set_next_state_(State next_state) { state_ = next_state; };
  void set_next_state_delayed_(uint32_t ms, State next_state);

  void read_reply_and_go_next_state_(ReadFunction read_fn, State next_state, uint8_t retries, bool mission_critical,
                                     bool check_crc);
  struct {
    ReadFunction read_fn;
    State next_state;
    bool mission_critical;
    bool check_crc;
    uint8_t tries_max;
    uint8_t tries_counter;
    uint32_t err_crc;
    uint32_t err_invalid_frames;
  } reading_state_{nullptr, State::IDLE, false, false, 0, 0, 0, 0};
  size_t received_frame_size_{0};

  uint32_t baud_rate_handshake_{9600};
  uint32_t baud_rate_{9600};

  uint32_t last_rx_time_{0};

  struct {
    uint8_t in[MAX_IN_BUF_SIZE];
    size_t amount_in;
    uint8_t out[MAX_OUT_BUF_SIZE];
    size_t amount_out;
  } buffers_;

  void clear_rx_buffers_();

  
  uint8_t calculate_crc_prog_frame_(uint8_t *data, size_t length, bool set_crc = false);
  bool check_crc_prog_frame_(uint8_t *data, size_t length);

  void prepare_frame_(const uint8_t *data, size_t length);
  void prepare_prog_frame_(const char *request, bool write = false);
  void prepare_non_session_prog_frame_(const char *request);
  void prepare_ctime_frame_(uint8_t hh, uint8_t mm, uint8_t ss);

  void send_frame_(const uint8_t *data, size_t length);
  void send_frame_prepared_();

  size_t receive_frame_(FrameStopFunction stop_fn);
  size_t receive_frame_ascii_();
  size_t receive_frame_ack_nack_();
  size_t receive_prog_frame_(uint8_t start_byte, bool accept_ack_and_nack = false);

  inline void update_last_rx_time_() { this->last_rx_time_ = millis(); }
  bool check_wait_timeout_() { return millis() - wait_.start_time >= wait_.delay_ms; }
  bool check_rx_timeout_() { return millis() - this->last_rx_time_ >= receive_timeout_ms_; }

  char *extract_meter_id_(size_t frame_size);
  uint8_t get_values_from_brackets_(char *line, ValueRefsArray &vals);
  char *get_nth_value_from_csv_(char *line, uint8_t idx);
  bool set_sensor_value_(EnergomeraBleSensorBase *sensor, ValueRefsArray &vals);

  void report_failure(bool failure);
  void abort_mission_();

  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);

  struct Stats {
    uint32_t connections_tried_{0};
    uint32_t crc_errors_{0};
    uint32_t crc_errors_recovered_{0};
    uint32_t invalid_frames_{0};
    uint8_t failures_{0};

    float crc_errors_per_session() const { return (float) crc_errors_ / connections_tried_; }
  } stats_;
  void stats_dump_();

  uint8_t failures_before_reboot_{0};

  struct LoopState {
    uint32_t session_started_ms{0};             // start of session
    SensorMap::iterator request_iter{nullptr};  // talking to meter
    SensorMap::iterator sensor_iter{nullptr};   // publishing sensor values
  } loop_state_;

 private:
  static uint8_t next_obj_id_;
  std::string tag_;

  static std::string generateTag();

  // Data structures for time synchronization
  char meter_datetime_str_[20]{};

  esp_err_t write_array(uint8_t *data, size_t length);
  void receive_data(uint8_t *data, size_t length);
};
*/
}  // namespace energomera_ble
}  // namespace esphome
