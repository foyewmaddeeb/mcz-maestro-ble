#pragma once

#ifdef USE_ESP32

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "mcz_maestro_ble.h"

#ifdef USE_CLIMATE
#include "esphome/components/climate/climate.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

namespace esphome::mcz_maestro_ble {

#ifdef USE_CLIMATE
// On/off + setpoint + room temperature. Power/mode/fan are separate selects.
class MczClimate : public climate::Climate, public Parented<MczMaestroBle> {
 public:
  climate::ClimateTraits traits() override;
  void update_from_parent();

 protected:
  void control(const climate::ClimateCall &call) override;
};
#endif

#ifdef USE_SELECT
enum class SelectType : uint8_t { POWER, MODE, FAN };

class MczSelect : public select::Select, public Parented<MczMaestroBle> {
 public:
  void set_type(SelectType t) { this->type_ = t; }
  void update_from_parent();

 protected:
  void control(size_t index) override;
  SelectType type_{SelectType::POWER};
};
#endif

#ifdef USE_SWITCH
class MczSwitch : public switch_::Switch, public Parented<MczMaestroBle> {
 protected:
  void write_state(bool state) override;
};
#endif

#ifdef USE_BUTTON
enum class ButtonType : uint8_t { ALARM_RESET, SYNC_CLOCK };

class MczButton : public button::Button, public Parented<MczMaestroBle> {
 public:
  void set_type(ButtonType t) { this->type_ = t; }

 protected:
  void press_action() override;
  ButtonType type_{ButtonType::ALARM_RESET};
};
#endif

}  // namespace esphome::mcz_maestro_ble

#endif  // USE_ESP32
