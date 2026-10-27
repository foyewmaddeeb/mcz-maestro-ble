// oven.h — central state model + command API for the MCZ oven.
#pragma once
#include <Arduino.h>

// ---- Verified registers ----------------------
// Control (write, function 06):
static const uint16_t REG_SETPOINT = 0x03F7;  // Setpoint temperature, value = °C * 10
static const uint16_t REG_POWER    = 0x03EB;  // Power 1..5
static const uint16_t REG_MODE     = 0x03E9;  // Operating mode, enum 0x03E9:
                                              // 0=Manual,1=Auto,2=Overnight (verified),
                                              // 3=Comfort,4=Turbo (ASSUMED, verify via read test)
static const uint16_t REG_ONOFF    = 0x038A;  // Power TOGGLE: write 1 = toggle on/off
static const uint16_t REG_ALARM_RESET = 0x038B; // com_reset_allarm (907): write 1 = acknowledge alarm
                                              // (same command type 13 as com_on_off; from register DB)
// Date/time block (status_exchange). Byte layout VERIFIED by read-back on the device
// (binary, not BCD; full year; weekday 0=Monday..6=Sunday). Note: hi/lo is the reverse
// of the informal spec -> use exactly this mapping.
static const uint16_t REG_DT_DAYMON  = 0x0384; // 900: hi=month(1-12), lo=day(1-31)
static const uint16_t REG_DT_YEAR    = 0x0385; // 901: full year (e.g. 2026)
static const uint16_t REG_DT_HRMIN   = 0x0386; // 902: hi=minute(0-59), lo=hour(0-23)
static const uint16_t REG_DT_SECWD   = 0x0387; // 903: hi=weekday(0=Mon..6=Sun), lo=second(0-59)
static const uint16_t REG_DT_TRIGGER = 0x0388; // 904: hi=set_orodatario, lo=am_pm -> write 0x0100 to commit
// Status (read):
static const uint16_t REG_ROOM     = 0x02BC;  // Room temp  ÷10
static const uint16_t REG_BOILER_T = 0x02BF;  // temp_caldaia (703): boiler water temp ÷10 (Hydro)
static const uint16_t REG_PUFFER_T = 0x02C0;  // temp_puffer  (704): buffer tank temp ÷10 (Hydro)
static const uint16_t REG_BOARD    = 0x02C1;  // Control board temp ÷10
static const uint16_t REG_FUMES    = 0x02C5;  // Flue gas temp ÷10
static const uint16_t REG_STATE    = 0x0320;  // Fine phase/state (16-bit code, see stateName)
static const uint16_t REG_PHASE    = 0x0322;  // coarse: 1=Off/Standby, 3=On (incl. start sequence)
static const uint16_t REG_MODE_LIVE= 0x032E;  // Operating mode mirror (same enum 0..4)
static const uint16_t REG_ALARM    = 0x0323;  // tipo_allarme = LOW byte (0 = no active alarm)
static const uint16_t REG_ALARM_IDX= 0x07EA;  // index_allarme (hi) / num_allarmi (lo) — alarm log head
static const uint16_t REG_FLAGS    = 0x0332;  // Bit field: Bit6=Chrono, Bit5=Silent
static const uint16_t REG_IGNIT    = 0x0334;  // Ignition counter
static const uint16_t REG_WORK_LO  = 0x0340;  // Total working time (sec, 32-bit LE word)
static const uint16_t REG_WORK_HI  = 0x0341;
static const uint16_t REG_PTIME_LO = 0x0336;  // Time in power 1..5: 5x 32-bit (sec),
static const uint16_t REG_PTIME_HI = 0x033F;  // low word first, 0x0336..0x033F
static const uint16_t REG_ACTIVE   = 0x02C9;  // App value "active" 
static const uint16_t REG_FAN_COMB = 0x02CE;  // Combustion fan RPM
static const uint16_t REG_FAN_ROOM = 0x02D1;  // Flue gas/exhaust fan RPM
static const uint16_t REG_FAN_SET  = 0x03FA;  // Write: 1..5 = fixed level, 6 = auto ventilation
static const uint16_t REG_FAN_LIVE = 0x0324;  // Live fan level (follows REG_FAN_SET; automatic under Auto)
static const uint16_t REG_SILENT   = 0x03EC;  // Write: 1 = Silent on, 0 = off (mirrors Flags Bit5)
// Hydro settings (read/write, °C ÷10) — verified against the app on the Hydro oven:
static const uint16_t REG_HYST_AMB_NEG = 0x03F3; // ist_neg_amb: ambient hysteresis negative
static const uint16_t REG_HYST_AMB_POS = 0x03F4; // ist_pos_amb: ambient hysteresis positive
static const uint16_t REG_HYST_SS_NEG  = 0x03F5; // ist_eco_neg_amb: Start/Stop (Ecostop) hysteresis neg
static const uint16_t REG_HYST_SS_POS  = 0x03F6; // ist_eco_pos_amb: Start/Stop (Ecostop) hysteresis pos
static const uint16_t REG_PUMP_MIN_ON  = 0x0403; // temp_min_circ_on: circulation pump min on-temperature

// ---- Capability detection (which hardware the stove actually has) -----------
// Register numbers from the app decoder (libencdec.so) = Modbus addresses.
// Sentinel 0xFFFF = feature absent/not configured (verified on the device).
static const uint16_t REG_SET_BOILER  = 0x0407;  // set_boiler (1031): valid -> boiler circuit
static const uint16_t REG_MAXPOT_IDRO = 0x040F;  // max_pot_idro (1039): >0 & !=0xFFFF -> HYDRO
static const uint16_t REG_SET_PUFFER  = 0x0412;  // set_puffer (1042): valid -> buffer tank
static const uint16_t REG_VVEN1_LO    = 0x05FD;  // v_ven1 v0..v5 (1533..1535): fan-1 speed table
static const uint16_t REG_VVEN1_HI    = 0x05FF;  //   (byte-packed; count of non-zero = fan levels)
static const uint16_t REG_TIPO_APP    = 0x06A4;  // tipo_app (1700): device type id
static const uint16_t REG_ABI_VEN12   = 0x06F4;  // abi_ven1/2_tlc_app (1780): fan-enable bytes
static const uint16_t REG_ABI_VEN34   = 0x06F5;  // abi_ven3/4_tlc_app (1781)
static const uint16_t REG_CLIMA       = 0x0372;  // stato_clima (882): conditioner block
static const uint16_t REG_BANCADATI   = 0x07C6;  // rev_banca_dati (1990) + banca_dati name (1991..1998)
static const uint16_t SENTINEL16      = 0xFFFF;

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
static const int OVEN_MODE_COUNT = 5;
inline const char* modeName(int m){
  switch(m){ case 0:return "Manual";  case 1:return "Auto";  case 2:return "Overnight";
             case 3:return "Comfort"; case 4:return "Turbo"; default:return "?"; }
}

// Fine phase/operating state from REG_STATE (0x0320)
inline const char* stateName(int s){
  switch(s){
    case 0x0000: return "Off";
    case 0x0101: return "Cleaning";
    case 0x0201: return "Loading";
    case 0x0301: return "Start 1";
    case 0x0401: return "Start 2";
    case 0x0501: return "Stabilization";
    case 0x0601: return "Anti-condensation";
    case 0x0202: return "On";
    case 0x0103: return "Turning off";
    default:     return "";
  }
}

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
