#include "mcz_maestro_ble.h"
#include "mcz_entities.h"

#ifdef USE_ESP32

#include <algorithm>

#include <esp_gatt_common_api.h>

#include "esphome/components/esp32_ble/ble.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::mcz_maestro_ble {

static const char *const TAG = "mcz_maestro_ble";

using namespace mcz::regs;

static const uint32_t STEP_SPACING_MS = 700;  // > response latency: one read outstanding
static const uint32_t CLOCK_RESYNC_MS = 24UL * 60 * 60 * 1000;
static const uint32_t WRITE_GAP_MS = 400;      // after the last frame, so an in-flight read can answer
static const uint32_t ECHO_TIMEOUT_MS = 1500;  // per write attempt

void MczMaestroBle::setup() { this->reset_link_(); }

void MczMaestroBle::dump_config() {
  ESP_LOGCONFIG(TAG, "MCZ Maestro+ BLE:");
  ESP_LOGCONFIG(TAG, "  Poll interval: %ums", (unsigned) this->poll_interval_);
  ESP_LOGCONFIG(TAG, "  Pair on connect: %s", YESNO(this->pair_on_connect_));
#ifdef MCZ_USE_TIME
  ESP_LOGCONFIG(TAG, "  Clock sync: %s", YESNO(this->time_ != nullptr));
#endif
}

// ---- Link --------------------------------------------------------------------

void MczMaestroBle::reset_link_() {
  this->write_handle_ = 0;
  this->notify_handle_ = 0;
  this->caps_done_ = false;
  this->cap_idx_ = 0;
  this->alarm_idx_ = 0;
  this->poll_idx_ = 0;
  this->alarm_done_ = false;
  this->alarm_dirty_ = true;
  this->alarm_index_ = -1;
  this->alarm_prev_ = -1;
  this->clock_done_ = false;
  if (!this->writes_.empty())
    ESP_LOGW(TAG, "link lost, dropping %u queued write(s)", (unsigned) this->writes_.size());
  this->writes_.clear();  // never replay stale commands after a reconnect
  this->awaiting_echo_ = false;
#ifdef USE_BINARY_SENSOR
  if (this->connected_binary_sensor_ != nullptr)
    this->connected_binary_sensor_->publish_state(false);
#endif
}

void MczMaestroBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                        esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT:
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "[%s] connected", this->parent()->address_str());
        if (this->pair_on_connect_)
          this->parent()->pair();
      }
      break;
    case ESP_GATTC_CFG_MTU_EVT:
      ESP_LOGI(TAG, "[%s] MTU %u%s", this->parent()->address_str(), param->cfg_mtu.mtu,
               param->cfg_mtu.mtu < 200 ? " - too small for the status broadcast" : "");
      break;
    case ESP_GATTC_DISCONNECT_EVT:
      ESP_LOGW(TAG, "[%s] disconnected", this->parent()->address_str());
      this->reset_link_();
      break;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *wr = this->parent()->get_characteristic(mcz::SERVICE_UUID, mcz::CHAR_WRITE_UUID);
      auto *nt = this->parent()->get_characteristic(mcz::SERVICE_UUID, mcz::CHAR_NOTIFY_UUID);
      if (wr == nullptr || nt == nullptr) {
        ESP_LOGE(TAG, "[%s] service 0xABF0 / chars 0xABF1+0xABF2 not found - not an MCZ display?",
                 this->parent()->address_str());
        break;
      }
      this->write_handle_ = wr->handle;
      this->notify_handle_ = nt->handle;
      auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                      this->parent()->get_remote_bda(), nt->handle);
      if (status)
        ESP_LOGW(TAG, "register_for_notify failed, status=%d", status);
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      if (param->reg_for_notify.handle != this->notify_handle_)
        break;
      this->node_state = espbt::ClientState::ESTABLISHED;
      this->last_step_ms_ = millis();
      ESP_LOGI(TAG, "[%s] ready, starting capability scan", this->parent()->address_str());
#ifdef USE_BINARY_SENSOR
      if (this->connected_binary_sensor_ != nullptr)
        this->connected_binary_sensor_->publish_state(true);
#endif
      break;
    case ESP_GATTC_NOTIFY_EVT:
      if (param->notify.handle == this->notify_handle_)
        this->on_notify_(param->notify.value, param->notify.value_len);
      break;
    default:
      break;
  }
}

void MczMaestroBle::write_frame_(const uint8_t *pdu, size_t len) {
  if (!this->is_ready()) {
    ESP_LOGW(TAG, "not connected, frame dropped");
    return;
  }
  uint8_t ct[mcz::MAX_FRAME];
  size_t n = mcz::build_frame(this->counter_++, pdu, len, ct);
  if (n == 0)
    return;
  auto status = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->write_handle_, n, ct, ESP_GATT_WRITE_TYPE_NO_RSP,
                                         ESP_GATT_AUTH_REQ_NONE);
  if (status)
    ESP_LOGW(TAG, "write_char failed, status=%d", status);
  this->last_tx_ms_ = millis();
}

void MczMaestroBle::send_read_(uint16_t reg, uint16_t count) {
  this->last_read_base_ = reg;  // a fn03 reply carries no address
  uint8_t pdu[6];
  ESP_LOGV(TAG, "read 0x%04X x%u", reg, count);
  this->write_frame_(pdu, mcz::pdu_read(pdu, reg, count));
}

void MczMaestroBle::send_write_(uint16_t reg, uint16_t val, uint8_t attempts) {
  PendingWrite w{reg, {val}, 1, attempts};
  ESP_LOGD(TAG, "queue write 0x%04X = 0x%04X", reg, val);
  this->writes_.push_back(w);
}

void MczMaestroBle::send_write_multi_(uint16_t reg, const uint16_t *vals, uint8_t n) {
  if (n == 0 || n > 5)
    return;
  PendingWrite w{reg, {}, n, 3};
  memcpy(w.vals, vals, n * sizeof(uint16_t));
  ESP_LOGD(TAG, "queue write-multi 0x%04X x%u", reg, n);
  this->writes_.push_back(w);
}

void MczMaestroBle::transmit_write_(const PendingWrite &w) {
  uint8_t pdu[mcz::MAX_PDU];
  size_t len = w.n == 1 ? mcz::pdu_write(pdu, w.reg, w.vals[0]) : mcz::pdu_write_multi(pdu, w.reg, w.vals, w.n);
  if (len == 0)
    return;
  ESP_LOGD(TAG, "write 0x%04X (%u reg)", w.reg, w.n);
  this->write_frame_(pdu, len);
}

bool MczMaestroBle::service_writes_() {
  uint32_t now = millis();
  if (this->awaiting_echo_) {
    if (now - this->write_sent_ms_ < ECHO_TIMEOUT_MS)
      return true;
    auto &w = this->writes_.front();
    if (w.tries_left > 0) {
      ESP_LOGW(TAG, "write 0x%04X not confirmed, retrying (%u left)", w.reg, w.tries_left);
      w.tries_left--;
      this->transmit_write_(w);
      this->write_sent_ms_ = now;
      return true;
    }
    ESP_LOGW(TAG, "write 0x%04X not confirmed, giving up", w.reg);
    this->writes_.pop_front();
    this->awaiting_echo_ = false;
    return true;
  }
  if (this->writes_.empty())
    return false;
  if (now - this->last_tx_ms_ < WRITE_GAP_MS)
    return true;
  auto &w = this->writes_.front();
  w.tries_left--;
  this->transmit_write_(w);
  this->awaiting_echo_ = true;
  this->write_sent_ms_ = now;
  return true;
}

void MczMaestroBle::on_write_echo_(uint16_t reg) {
  if (!this->awaiting_echo_ || this->writes_.empty() || this->writes_.front().reg != reg)
    return;
  ESP_LOGD(TAG, "write 0x%04X confirmed", reg);
  this->writes_.pop_front();
  this->awaiting_echo_ = false;
  this->last_step_ms_ = 0;  // poll right away so the new value shows up
}

// ---- Notify ------------------------------------------------------------------

void MczMaestroBle::on_notify_(const uint8_t *data, size_t len) {
  if (mcz::is_broadcast(data, len)) {
    auto r = this->broadcast_.feed(data, len);
    if (r == mcz::BroadcastAssembler::COMPLETE) {
      const uint8_t *img = this->broadcast_.image();
      size_t n = this->broadcast_.image_len();
      ESP_LOGV(TAG, "broadcast: %u registers", (unsigned) (n / 2));
      for (size_t i = 0; i + 1 < n; i += 2)
        this->apply_reg_(STATUS_BASE + (uint16_t) (i / 2), (img[i] << 8) | img[i + 1]);
    } else if (r == mcz::BroadcastAssembler::DROPPED) {
      ESP_LOGV(TAG, "broadcast fragment dropped (len %u)", (unsigned) len);
    }
    return;
  }
  if (len % 16 || len < 32) {
    ESP_LOGW(TAG, "short notify (%u B) - MTU too small?", (unsigned) len);
    return;
  }
  uint8_t pt[512];
  if (len > sizeof(pt))
    return;
  const uint8_t *mb;
  size_t mb_len;
  uint32_t ctr;
  if (!mcz::decode_modbus(data, len, pt, &mb, &mb_len, &ctr)) {
    ESP_LOGW(TAG, "bad frame (crc/padding), discarded");
    return;
  }
  if (mb_len >= 5 && mb[1] == 0x03) {
    this->on_read_block_(this->last_read_base_, mb + 3, mb[2]);
  } else if (mb_len >= 6 && mb[1] == 0x06) {
    uint16_t reg = (mb[2] << 8) | mb[3], val = (mb[4] << 8) | mb[5];
    ESP_LOGD(TAG, "write echo 0x%04X = 0x%04X", reg, val);
    this->apply_reg_(reg, val);
    this->on_write_echo_(reg);
  } else if (mb_len >= 6 && mb[1] == 0x10) {
    this->on_write_echo_((mb[2] << 8) | mb[3]);
  } else if (mb_len >= 3 && (mb[1] & 0x80)) {
    ESP_LOGW(TAG, "modbus exception fn=0x%02X code=%u", mb[1], mb[2]);
  }
}

void MczMaestroBle::on_read_block_(uint16_t base, const uint8_t *data, uint8_t bytes) {
  if (base == REG_SERIAL) {
    std::string s;
    for (uint8_t i = 0; i < bytes; i++)
      if (data[i] >= 32 && data[i] < 127)
        s += (char) data[i];
    while (!s.empty() && s.back() == ' ')
      s.pop_back();
    if (!s.empty() && s != this->serial_) {
      this->serial_ = s;
      ESP_LOGI(TAG, "serial number: %s", s.c_str());
#ifdef USE_TEXT_SENSOR
      if (this->serial_number_text_sensor_ != nullptr)
        this->serial_number_text_sensor_->publish_state(s);
#endif
    }
    return;
  }
  for (uint8_t i = 0; i + 1 < bytes; i += 2)
    this->apply_reg_(base + i / 2, (data[i] << 8) | data[i + 1]);
}

// ---- Register -> state -----------------------------------------------------

#ifdef USE_SENSOR
static void pub(sensor::Sensor *s, float v) {
  if (s != nullptr && (!s->has_state() || s->get_raw_state() != v))
    s->publish_state(v);
}
#endif
#ifdef USE_BINARY_SENSOR
static void pub(binary_sensor::BinarySensor *s, bool v) {
  if (s != nullptr)
    s->publish_state(v);  // BinarySensor already de-duplicates
}
#endif
#ifdef USE_TEXT_SENSOR
static void pub(text_sensor::TextSensor *s, const std::string &v) {
  if (s != nullptr && (!s->has_state() || s->get_state() != v))
    s->publish_state(v);
}
#endif

static float c10(uint16_t v) { return v / 10.0f; }

void MczMaestroBle::apply_reg_(uint16_t reg, uint16_t val) {
  if (reg >= REG_ALARM_CODE0 && reg <= REG_ALARM_CODE_LAST && ((reg - REG_ALARM_CODE0) % 4) == 0) {
    this->alarm_codes_[(reg - REG_ALARM_CODE0) / 4] = (uint8_t) val;
    return;
  }
  if (reg >= REG_BANCADATI + 1 && reg <= REG_BANCADATI + 8) {
    if (reg == REG_BANCADATI + 1)
      this->model_.clear();
    char a = (char) (val >> 8), b = (char) (val & 0xFF);
    if (a >= 32 && a < 127)
      this->model_ += a;
    if (b >= 32 && b < 127)
      this->model_ += b;
#ifdef USE_TEXT_SENSOR
    if (reg == REG_BANCADATI + 8)
      pub(this->model_text_sensor_, this->model_);
#endif
    return;
  }
  switch (reg) {
    // capabilities
    case REG_MAXPOT_IDRO:
      this->hydro_ = (val != SENTINEL16 && val > 0);
      break;
    case REG_VVEN1_LO:
      this->fan_levels_ = ((val & 0xFF) != 0) + ((val >> 8) != 0);
      break;
    case REG_VVEN1_LO + 1:
      this->fan_levels_ += ((val & 0xFF) != 0) + ((val >> 8) != 0);
      break;
    case REG_VVEN1_HI:
      this->fan_levels_ += ((val & 0xFF) != 0) + ((val >> 8) != 0);
      this->fan_levels_ = std::clamp<uint8_t>(this->fan_levels_, 1, 5);
      break;
    // status
    case REG_ROOM:
      if (this->room_ != c10(val)) {
        this->room_ = c10(val);
        this->publish_climate_();
      }
#ifdef USE_SENSOR
      pub(this->room_temperature_sensor_, c10(val));
#endif
      break;
    case REG_SETPOINT:
      if (this->setpoint_c_ != c10(val)) {
        this->setpoint_c_ = c10(val);
        this->publish_climate_();
      }
#ifdef USE_SENSOR
      pub(this->setpoint_sensor_, c10(val));
#endif
      break;
    case REG_PHASE:
      if (this->phase_ != (int) val) {
        this->phase_ = val;
        this->publish_climate_();
#ifdef USE_BINARY_SENSOR
        pub(this->running_binary_sensor_, val == 3);
#endif
      }
      break;
    case REG_STATE:
      if (this->state_ != (int) val) {
        const char *name = mcz::state_name(val);
        ESP_LOGI(TAG, "state 0x%04X -> 0x%04X (%s)", (unsigned) (this->state_ < 0 ? 0 : this->state_), val,
                 name[0] ? name : "unknown");
        this->state_ = val;
        this->publish_climate_();
#ifdef USE_TEXT_SENSOR
        if (this->state_text_sensor_ != nullptr) {
          char buf[16];
          if (!name[0]) {
            snprintf(buf, sizeof(buf), "0x%04X", val);
            name = buf;
          }
          pub(this->state_text_sensor_, name);
        }
#endif
      }
      break;
    case REG_POWER:
      if (this->power_ != (int) val) {
        this->power_ = val;
        this->publish_selects_();
      }
#ifdef USE_SENSOR
      pub(this->power_level_sensor_, val);
#endif
      break;
    case REG_MODE:
    case REG_MODE_LIVE:
      if (this->mode_ != (int) val) {
        this->mode_ = val;
        this->publish_selects_();
#ifdef USE_TEXT_SENSOR
        pub(this->mode_text_sensor_, mcz::mode_name(val));
#endif
      }
      break;
    case REG_FAN_LIVE:
      if (this->fan_live_ != (int) val) {
        this->fan_live_ = val;
        this->publish_selects_();
      }
#ifdef USE_SENSOR
      pub(this->fan_level_sensor_, val);
#endif
      break;
    case REG_FAN_SET:  // 1..5 fixed, 6 = auto
      if (this->fan_set_ != (int) val) {
        this->fan_set_ = val;
        this->publish_selects_();
      }
      break;
    case REG_FLAGS:
      if (this->flags_ != (int) val) {
        this->flags_ = val;
        this->silent_ = (val >> 5) & 1;
        this->publish_silent_();
#ifdef USE_BINARY_SENSOR
        pub(this->chrono_binary_sensor_, (val >> 6) & 1);
#endif
      }
      break;
    case REG_SILENT:
      if (val <= 1 && this->silent_ != (int) val) {
        this->silent_ = val;
        this->publish_silent_();
      }
      break;
    case REG_ALARM: {
      int code = val & 0xFF;
      if (code != 0 && code != this->alarm_prev_)
        this->alarm_dirty_ = true;  // new alarm -> refresh history
      this->alarm_prev_ = code;
      this->alarm_ = code;
#ifdef USE_SENSOR
      pub(this->alarm_code_sensor_, code);
#endif
#ifdef USE_BINARY_SENSOR
      pub(this->alarm_binary_sensor_, code != 0);
#endif
      break;
    }
    case REG_ALARM_IDX:
      this->alarm_index_ = val >> 8;
      this->alarm_num_ = val & 0xFF;
      break;
    case REG_WORK_LO:
      this->work_lo_ = val;
      break;
    case REG_WORK_HI:
#ifdef USE_SENSOR
      pub(this->working_time_sensor_, ((((uint32_t) val << 16) | this->work_lo_) / 3600.0f));
#endif
      break;
#ifdef USE_SENSOR
    case REG_FUMES:
      pub(this->fumes_temperature_sensor_, c10(val));
      break;
    case REG_BOARD:
      pub(this->board_temperature_sensor_, c10(val));
      break;
    case REG_BOILER_T:
      if (this->hydro_)
        pub(this->boiler_temperature_sensor_, c10(val));
      break;
    case REG_PUFFER_T:
      if (this->hydro_)
        pub(this->puffer_temperature_sensor_, c10(val));
      break;
    case REG_IGNIT:
      pub(this->ignitions_sensor_, val);
      break;
    case REG_FAN_COMB:
      pub(this->combustion_fan_speed_sensor_, val);
      break;
    case REG_FAN_ROOM:
      pub(this->exhaust_fan_speed_sensor_, val);
      break;
    case REG_ACTIVE:
      pub(this->active_sensor_, val);
      break;
#endif
    default:
      break;
  }
}

void MczMaestroBle::publish_climate_() {
#ifdef USE_CLIMATE
  if (this->climate_ != nullptr)
    this->climate_->update_from_parent();
#endif
}

void MczMaestroBle::publish_selects_() {
#ifdef USE_SELECT
  for (auto *s : {this->power_select_, this->mode_select_, this->fan_select_})
    if (s != nullptr)
      s->update_from_parent();
#endif
}

void MczMaestroBle::publish_silent_() {
#ifdef USE_SWITCH
  if (this->silent_switch_ != nullptr && this->silent_ >= 0)
    this->silent_switch_->publish_state(this->silent_ == 1);
#endif
}

void MczMaestroBle::publish_alarm_history_() {
#ifdef USE_TEXT_SENSOR
  if (this->alarm_history_text_sensor_ == nullptr)
    return;
  std::string s;
  int cnt = this->alarm_num_ < 10 ? this->alarm_num_ : 10;
  for (int k = 0; k < cnt && this->alarm_index_ >= 0; k++) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%sA%02u", s.empty() ? "" : " ",
             this->alarm_codes_[(this->alarm_index_ - 1 - k + 100) % 100]);
    s += buf;
  }
  pub(this->alarm_history_text_sensor_, s.empty() ? std::string("none") : s);
#endif
}

// ---- Scheduler: exactly one read in flight -----------------------------------

void MczMaestroBle::loop() {
  // Bluedroid's local MTU defaults to 23; the "##" broadcast needs a large one.
  // Must be set once the BLE stack is up and before the first connect.
  if (!this->mtu_set_ && esp32_ble::global_ble != nullptr && esp32_ble::global_ble->is_active()) {
    esp_err_t err = esp_ble_gatt_set_local_mtu(mcz::MTU);
    if (err != ESP_OK)
      ESP_LOGW(TAG, "set_local_mtu(%u) failed: %d", mcz::MTU, err);
    this->mtu_set_ = true;
  }
  if (!this->is_ready())
    return;
  this->clock_tick_();
  if (this->service_writes_())
    return;
  if (!this->caps_done_) {
    this->cap_scan_step_();
  } else if (!this->alarm_done_ || this->alarm_dirty_) {
    this->alarm_scan_step_();
  } else {
    this->poll_step_();
  }
}

void MczMaestroBle::cap_scan_step_() {
  uint32_t now = millis();
  if (now - this->last_step_ms_ < STEP_SPACING_MS)
    return;
  this->last_step_ms_ = now;
  switch (this->cap_idx_++) {
    case 0: this->send_read_(REG_SET_BOILER, 13); break;   // 0x0407..0x0413
    case 1: this->send_read_(REG_ABI_VEN12, 2); break;
    case 2: this->send_read_(REG_VVEN1_LO, 3); break;      // fan levels
    case 3: this->send_read_(REG_TIPO_APP, 1); break;
    case 4: this->send_read_(REG_CLIMA, 1); break;
    case 5: this->send_read_(REG_BANCADATI, 9); break;     // model id (ASCII)
    default:
      this->caps_done_ = true;
      ESP_LOGI(TAG, "capabilities: %s, fan levels %u, model '%s'", this->hydro_ ? "HYDRO" : "AIR",
               this->fan_levels_, this->model_.c_str());
      break;
  }
}

void MczMaestroBle::alarm_scan_step_() {
  uint32_t now = millis();
  if (now - this->last_step_ms_ < STEP_SPACING_MS)
    return;
  this->last_step_ms_ = now;
  int start = (this->alarm_index_ - 10 + 100) % 100;  // oldest of the last 10
  switch (this->alarm_idx_) {
    case 0:
      this->alarm_index_ = -1;
      this->send_read_(REG_ALARM_IDX, 1);
      break;
    case 1: {
      if (this->alarm_index_ < 0) {  // index reply not in yet -> retry
        this->alarm_idx_ = 0;
        return;
      }
      int first = (start <= 90) ? 10 : (100 - start);
      this->send_read_(REG_ALARM_LOG + 4 * start, 4 * first);
      break;
    }
    case 2:
      if (start > 90)  // wrapped part from slot 0
        this->send_read_(REG_ALARM_LOG, 4 * ((start + 10) - 100));
      break;
    default:
      this->alarm_done_ = true;
      this->alarm_dirty_ = false;
      this->alarm_idx_ = 0;
      this->publish_alarm_history_();
      return;
  }
  this->alarm_idx_++;
}

void MczMaestroBle::poll_step_() {
  uint32_t now = millis();
  if (now - this->last_step_ms_ < this->poll_interval_)
    return;
  this->last_step_ms_ = now;
  uint8_t steps = this->serial_.empty() ? 4 : 3;
  switch (this->poll_idx_ % steps) {
    case 0: this->send_read_(0x02BC, 51); break;  // room/board/fumes temps
    case 1: this->send_read_(0x0320, 77); break;  // phase, flags, counters, fans
    case 2: this->send_read_(0x03E9, 18); break;  // mode, power, silent, setpoint, fan set
    case 3: this->send_read_(REG_SERIAL, 8); break;
  }
  this->poll_idx_ = (this->poll_idx_ + 1) % steps;
}

void MczMaestroBle::clock_tick_() {
#ifdef MCZ_USE_TIME
  if (this->time_ == nullptr || !this->caps_done_)
    return;
  if (this->clock_done_ && millis() - this->clock_last_ms_ < CLOCK_RESYNC_MS)
    return;
  if (this->sync_clock()) {
    this->clock_done_ = true;
    this->clock_last_ms_ = millis();
  }
#endif
}

// ---- Commands ----------------------------------------------------------------

bool MczMaestroBle::set_setpoint(float c) {
  if (!this->is_ready() || std::isnan(c) || c < 5.0f || c > 45.0f)
    return false;
  this->send_write_(REG_SETPOINT, (uint16_t) (c * 10.0f + 0.5f));
  return true;
}

bool MczMaestroBle::set_power(int n) {
  if (!this->is_ready() || n < 1 || n > 5)
    return false;
  this->send_write_(REG_POWER, n);
  return true;
}

bool MczMaestroBle::set_mode(int m) {
  if (!this->is_ready() || m < 0 || m >= mcz::MODE_COUNT)
    return false;
  this->send_write_(REG_MODE, m);
  return true;
}

bool MczMaestroBle::set_on(bool on) {
  if (!this->is_ready())
    return false;
  // 0x038A is a toggle: only press it when the state actually has to change.
  if (this->phase_ < 0) {
    ESP_LOGW(TAG, "phase unknown, refusing a blind power toggle");
    return false;
  }
  bool standby = this->phase_ == 1;
  if (on == !standby) {
    ESP_LOGD(TAG, "already %s (phase %d)", on ? "on" : "off", this->phase_);
    return false;
  }
  ESP_LOGI(TAG, "%s: power toggle (phase %d)", on ? "ON" : "OFF", this->phase_);
  this->send_write_(REG_ONOFF, 1, 1);  // toggle: never retry, a lost echo could flip it back
  return true;
}

bool MczMaestroBle::set_fan(int level) {
  if (!this->is_ready() || level < 0 || level > 5)
    return false;
  this->send_write_(REG_FAN_SET, level == 0 ? 6 : level);
  return true;
}

bool MczMaestroBle::set_silent(bool on) {
  if (!this->is_ready())
    return false;
  this->send_write_(REG_SILENT, on ? 1 : 0);
  return true;
}

bool MczMaestroBle::reset_alarm() {
  if (!this->is_ready() || this->alarm_ <= 0) {
    ESP_LOGI(TAG, "no active alarm, no reset sent");
    return false;
  }
  this->send_write_(REG_ALARM_RESET, 1);
  return true;
}

bool MczMaestroBle::sync_clock() {
#ifdef MCZ_USE_TIME
  if (!this->is_ready() || this->time_ == nullptr)
    return false;
  ESPTime t = this->time_->now();
  if (!t.is_valid() || t.year < 2024 || t.year > 2099)
    return false;
  int wd = (t.day_of_week + 5) % 7;  // ESPTime 1=Sunday..7=Saturday -> 0=Monday..6=Sunday
  // The display only commits the DATE when 900-903 go as ONE fn 0x10 write; 904 = commit.
  uint16_t vals[5] = {
      (uint16_t) ((t.month << 8) | t.day_of_month),
      (uint16_t) t.year,
      (uint16_t) ((t.minute << 8) | t.hour),
      (uint16_t) ((wd << 8) | t.second),
      (uint16_t) (1 << 8),
  };
  this->send_write_multi_(REG_DT_DAYMON, vals, 5);
  ESP_LOGI(TAG, "clock set to %04d-%02d-%02d %02d:%02d:%02d", t.year, t.month, t.day_of_month, t.hour, t.minute,
           t.second);
  return true;
#else
  ESP_LOGW(TAG, "clock sync needs time_id");
  return false;
#endif
}

// ---- Entities ----------------------------------------------------------------

#ifdef USE_CLIMATE
climate::ClimateTraits MczClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);
  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT});
  traits.set_visual_min_temperature(5.0f);
  traits.set_visual_max_temperature(45.0f);
  traits.set_visual_target_temperature_step(0.5f);
  traits.set_visual_current_temperature_step(0.1f);
  return traits;
}

void MczClimate::update_from_parent() {
  auto *p = this->parent_;
  this->current_temperature = p->room();
  this->target_temperature = p->setpoint();
  if (p->phase() < 0) {
    this->mode = climate::CLIMATE_MODE_OFF;
    this->action = climate::CLIMATE_ACTION_OFF;
  } else if (p->phase() == 1) {
    this->mode = climate::CLIMATE_MODE_OFF;
    this->action = climate::CLIMATE_ACTION_OFF;
  } else {
    this->mode = climate::CLIMATE_MODE_HEAT;
    this->action = p->phase() == 3 ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
  }
  this->publish_state();
}

void MczClimate::control(const climate::ClimateCall &call) {
  if (call.get_target_temperature().has_value())
    this->parent_->set_setpoint(*call.get_target_temperature());
  if (call.get_mode().has_value())
    this->parent_->set_on(*call.get_mode() != climate::CLIMATE_MODE_OFF);
  // State follows from the write echo / next poll, not optimistically.
}
#endif

#ifdef USE_SELECT
void MczSelect::update_from_parent() {
  auto *p = this->parent_;
  int idx = -1;
  switch (this->type_) {
    case SelectType::POWER:
      idx = p->power() - 1;  // "1".."5"
      break;
    case SelectType::MODE:
      idx = p->mode();
      break;
    case SelectType::FAN:
      // "Auto","1".."5": the set register says auto (6); the live one shows the current level
      idx = p->fan_set() == 6 ? 0 : (p->fan_set() >= 1 ? p->fan_set() : p->fan_live());
      break;
  }
  if (idx >= 0 && (size_t) idx < this->size())
    this->publish_state((size_t) idx);
}

void MczSelect::control(size_t index) {
  switch (this->type_) {
    case SelectType::POWER:
      this->parent_->set_power(index + 1);
      break;
    case SelectType::MODE:
      this->parent_->set_mode(index);
      break;
    case SelectType::FAN:
      this->parent_->set_fan(index);
      break;
  }
}
#endif

#ifdef USE_SWITCH
void MczSwitch::write_state(bool state) { this->parent_->set_silent(state); }
#endif

#ifdef USE_BUTTON
void MczButton::press_action() {
  if (this->type_ == ButtonType::ALARM_RESET)
    this->parent_->reset_alarm();
  else
    this->parent_->sync_clock();
}
#endif

}  // namespace esphome::mcz_maestro_ble

#endif  // USE_ESP32
