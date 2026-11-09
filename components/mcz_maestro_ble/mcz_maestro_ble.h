#pragma once

#ifdef USE_ESP32

#include <cmath>
#include <deque>
#include <string>

#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef MCZ_USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include <esp_gattc_api.h>

#include "mcz_proto.h"

namespace esphome::mcz_maestro_ble {

namespace espbt = esphome::esp32_ble_tracker;

class MczClimate;
class MczSelect;
class MczSwitch;

// Talks to the MCZ Maestro+ display over its own BLE service (0xABF0), the same
// path the MCZ app uses. Port of src/main.cpp: one Modbus read in flight at a
// time (a read reply carries no address), capability scan -> alarm log ->
// cyclic status poll, plus the "##" status broadcast the display pushes.
class MczMaestroBle : public Component, public ble_client::BLEClientNode {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; }
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void set_poll_interval(uint32_t ms) { this->poll_interval_ = ms; }
  void set_pair_on_connect(bool v) { this->pair_on_connect_ = v; }
  void set_climate(MczClimate *c) { this->climate_ = c; }
  void set_power_select(MczSelect *s) { this->power_select_ = s; }
  void set_mode_select(MczSelect *s) { this->mode_select_ = s; }
  void set_fan_select(MczSelect *s) { this->fan_select_ = s; }
  void set_silent_switch(MczSwitch *s) { this->silent_switch_ = s; }
#ifdef MCZ_USE_TIME
  void set_time(time::RealTimeClock *t) { this->time_ = t; }
#endif

#ifdef USE_SENSOR
  SUB_SENSOR(room_temperature)
  SUB_SENSOR(fumes_temperature)
  SUB_SENSOR(board_temperature)
  SUB_SENSOR(boiler_temperature)
  SUB_SENSOR(puffer_temperature)
  SUB_SENSOR(setpoint)
  SUB_SENSOR(power_level)
  SUB_SENSOR(fan_level)
  SUB_SENSOR(combustion_fan_speed)
  SUB_SENSOR(exhaust_fan_speed)
  SUB_SENSOR(ignitions)
  SUB_SENSOR(working_time)
  SUB_SENSOR(alarm_code)
  SUB_SENSOR(active)
#endif
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(connected)
  SUB_BINARY_SENSOR(running)
  SUB_BINARY_SENSOR(alarm)
  SUB_BINARY_SENSOR(chrono)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(state)
  SUB_TEXT_SENSOR(mode)
  SUB_TEXT_SENSOR(serial_number)
  SUB_TEXT_SENSOR(model)
  SUB_TEXT_SENSOR(alarm_history)
#endif

  // ---- Command API (used by the entity classes) ----
  bool set_setpoint(float celsius);
  bool set_power(int level);       // 1..5
  bool set_mode(int mode);         // 0..4
  bool set_on(bool on);            // via the 0x038A toggle, only if state differs
  bool set_fan(int level);         // 0 = auto, 1..5
  bool set_silent(bool on);
  bool reset_alarm();              // only if an alarm is active
  bool sync_clock();               // needs time_id

  bool is_ready() const { return this->node_state == espbt::ClientState::ESTABLISHED && this->write_handle_ != 0; }
  float room() const { return this->room_; }
  float setpoint() const { return this->setpoint_c_; }
  int phase() const { return this->phase_; }
  int power() const { return this->power_; }
  int mode() const { return this->mode_; }
  int fan_live() const { return this->fan_live_; }
  int fan_set() const { return this->fan_set_; }
  int fan_levels() const { return this->fan_levels_; }

 protected:
  void write_frame_(const uint8_t *pdu, size_t len);
  void send_read_(uint16_t reg, uint16_t count);
  // Writes are queued and sent one at a time in a gap between reads, then
  // confirmed by the display's echo. The display handles one request at a
  // time: a frame sent while a read is still in flight can be dropped silently.
  struct PendingWrite {
    uint16_t reg;
    uint16_t vals[5];
    uint8_t n;
    uint8_t tries_left;
  };
  void send_write_(uint16_t reg, uint16_t val, uint8_t attempts = 3);
  void send_write_multi_(uint16_t reg, const uint16_t *vals, uint8_t n);
  void transmit_write_(const PendingWrite &w);
  bool service_writes_();  // true while writes own the link (reads must wait)
  void on_write_echo_(uint16_t reg);

  void on_notify_(const uint8_t *data, size_t len);
  void apply_reg_(uint16_t reg, uint16_t val);
  void on_read_block_(uint16_t base, const uint8_t *data, uint8_t bytes);

  void cap_scan_step_();
  void alarm_scan_step_();
  void poll_step_();
  void clock_tick_();
  void reset_link_();
  void publish_climate_();
  void publish_selects_();
  void publish_silent_();
  void publish_alarm_history_();

  // link
  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};
  uint32_t counter_{mcz::COUNTER_START};
  bool mtu_set_{false};
  bool pair_on_connect_{false};
  uint16_t last_read_base_{0};
  uint32_t last_step_ms_{0};
  uint32_t last_tx_ms_{0};
  std::deque<PendingWrite> writes_;
  bool awaiting_echo_{false};
  uint32_t write_sent_ms_{0};
  uint32_t poll_interval_{2500};
  uint8_t cap_idx_{0};
  uint8_t alarm_idx_{0};
  uint8_t poll_idx_{0};
  mcz::BroadcastAssembler broadcast_;

  // capabilities
  bool caps_done_{false};
  bool hydro_{false};
  uint8_t fan_levels_{5};

  // state
  float room_{NAN};
  float setpoint_c_{NAN};
  int phase_{-1};
  int state_{-1};
  int power_{-1};
  int mode_{-1};
  int fan_live_{-1};
  int fan_set_{-1};
  int flags_{-1};
  int silent_{-1};
  int alarm_{-1};
  uint16_t work_lo_{0};
  std::string serial_;
  std::string model_;

  // alarm log
  uint8_t alarm_codes_[100]{};
  int alarm_index_{-1};
  int alarm_num_{0};
  int alarm_prev_{-1};
  bool alarm_done_{false};
  bool alarm_dirty_{true};

  // clock sync
  uint32_t clock_last_ms_{0};
  bool clock_done_{false};

  MczClimate *climate_{nullptr};
  MczSelect *power_select_{nullptr};
  MczSelect *mode_select_{nullptr};
  MczSelect *fan_select_{nullptr};
  MczSwitch *silent_switch_{nullptr};
#ifdef MCZ_USE_TIME
  time::RealTimeClock *time_{nullptr};
#endif
};

}  // namespace esphome::mcz_maestro_ble

#endif  // USE_ESP32
