// oven.h — central state model + command API for the MCZ oven.
#pragma once
#include <Arduino.h>

// ---- Register map ------------------------------------------------------------
// Lives in the shared protocol core (also used by the ESPHome component).
#include "mcz_proto.h"
using namespace mcz::regs;

// Detected stove capabilities. Filled once after connect (see capScan in main.cpp);
// all frontends read this to hide functions the hardware does not have.
struct OvenCaps {
  bool     detected  = false;  // capability scan finished
  bool     hydro     = false;  // water-heating stove (max_pot_idro valid)
  bool     boiler    = false;  // boiler (DHW) circuit present
  bool     puffer    = false;  // buffer tank present
  bool     clima     = false;  // conditioner block present
  uint8_t  fanCount  = 1;      // number of enabled room fans (>=1 typical)
  uint8_t  fanLevels = 5;      // selectable fan speed steps (1..5)
  uint16_t tipoApp   = SENTINEL16;
  uint16_t maxPotIdro= SENTINEL16;
  String   bancaDati;          // model database code (e.g. "ET60")
};
extern OvenCaps g_caps;

static const float TEMP_MIN_C = 5.0f, TEMP_MAX_C = 45.0f;  // Plausibility limits for setpoint

// ---- Running oven state ----------------------------------------------------
struct OvenState {
  float    roomC      = NAN;   // Room temperature
  float    boilerC    = NAN;   // Boiler water temperature (Hydro, 0x02BF)
  float    pufferC    = NAN;   // Buffer tank temperature (Hydro, 0x02C0)
  float    hystAmbNeg = NAN;   // Ambient hysteresis - (0x03F3)
  float    hystAmbPos = NAN;   // Ambient hysteresis + (0x03F4)
  float    hystSSNeg  = NAN;   // Start/Stop (Ecostop) hysteresis - (0x03F5)
  float    hystSSPos  = NAN;   // Start/Stop (Ecostop) hysteresis + (0x03F6)
  float    pumpMinOn  = NAN;   // Circulation pump min on-temperature (0x0403)
  float    pufferSet  = NAN;   // Buffer/water setpoint set_puffer (0x0412)
  float    boardC     = NAN;   // Control board temperature
  float    fumesC     = NAN;   // Flue gas temperature
  float    setpointC  = NAN;   // Setpoint temperature
  int8_t   power      = -1;    // Power 1..5
  int8_t   mode       = -1;    // 0=Manual,1=Auto,2=Overnight,3=Comfort,4=Turbo
  int8_t   phase      = -1;    // coarse 0x0322: 1=Off, 3=On
  int32_t  state      = -1;    // Fine phase 0x0320 (code, see stateName)
  int16_t  flags      = -1;    // Bit field 0x0332
  int32_t  ignitions  = -1;    // Ignitions
  int32_t  worktimeMin= -1;    // Total working time in minutes
  int32_t  powerTimeMin[5] = {-1,-1,-1,-1,-1};  // Time in power 1..5 (minutes)
  int32_t  fanComb    = -1;    // Combustion fan RPM
  int32_t  fanRoom    = -1;    // Flue gas fan RPM
  int32_t  active     = -1;    // App value "active" (0x02C9)
  int8_t   fanLevel   = -1;    // Live control fan level (0x0324), 1..5
  int16_t  alarm      = -1;    // Live alarm code (0x0323 low byte): 0 = no alarm, >0 = active "Axx"
  int16_t  alarmHist[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};  // last 10 alarm codes, newest first (-1=empty, 0=none)
  bool     bleOnline  = false; // BLE link to oven active
  uint32_t lastUpdateMs = 0;   // millis() of the last value update
  uint32_t lastRxMs   = 0;     // millis() of the last valid BLE response (for status light)
  uint32_t seq        = 0;     // increments on EVERY content change (for publish-on-change)
};
extern OvenState g_oven;

// Device identity (for unique MQTT/HA IDs with multiple ovens):
extern String g_ovenMac;     // Oven BLE MAC without ':' (e.g. "0ab0fa7cab43") -> topic/ID base
extern String g_ovenSerial;  // Serial number from register 0x0ADC (ASCII) -> display name
extern String g_adcHex;      // Raw bytes of 0x0ADC as hex (diagnostic, if no ASCII serial)

inline bool ovenRunning(){ return g_oven.phase == 3; }
inline bool ovenSilent(){ return g_oven.flags >= 0 && ((g_oven.flags >> 5) & 1); }  // Flags Bit5
static const int OVEN_MODE_COUNT = mcz::MODE_COUNT;
inline const char* modeName(int m){ return mcz::mode_name(m); }
// Fine phase/operating state from REG_STATE (0x0320)
inline const char* stateName(int s){ return mcz::state_name(s); }

// Sort one register (address+value) into g_oven; bumps seq on change.
// Source irrelevant (poll read or ##-broadcast). Implemented in main.cpp.
void ovenApplyReg(uint16_t reg, uint16_t val);

// ---- Command API (used by Serial/Display/MQTT) -----------------------------
// Return true if the write command was issued (value plausible + BLE ready).
bool ovenSetTemp(float celsius);
bool ovenSetPower(int level);    // 1..5
bool ovenSetMode(int mode);      // 0..4 (see REG_MODE enum)
bool ovenSetOnOff(bool on);      // TODO: write register still unknown 
bool ovenSetFan(int level);      // 0 = Auto (writes 6), 1..5 = fixed level -> REG_FAN_SET
bool ovenSetSilent(bool on);     // Silent mode on/off -> REG_SILENT
bool ovenSetTempParam(uint16_t reg, float celsius);  // write a °C-scaled setting (value = C*10)
bool ovenSetClock();             // set the oven RTC from NTP (writes 900-903 + trigger 904)
bool ovenResetAlarm();           // acknowledge the active alarm -> REG_ALARM_RESET=1 (only if one is active)
