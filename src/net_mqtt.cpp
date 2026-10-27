// net_mqtt.cpp — WiFi + MQTT + Home Assistant discovery.
// Device identity = oven BLE MAC (unique across multiple ovens); display name incl.
// serial number. Both are only known after the BLE connect -> topics/discovery are
// built lazily once the identity is known.
#include "net_mqtt.h"

#if USE_MQTT
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"
#include "appconfig.h"
#include "oven.h"

static WiFiClient   wifiClient;
static PubSubClient mqtt(wifiClient);

static String T_BASE, T_STATE, T_AVAIL, T_SET_TEMP, T_SET_POWER, T_SET_MODE, T_SET_ONOFF, T_SET_FAN, T_SET_SILENT, T_SET_ALARM_RESET;
static String g_id, g_devName;

static bool     g_idReady     = false;
static String   g_discSerial;                  // serial number that was in the last discovery
static uint32_t g_lastPubSeq  = 0xFFFFFFFF;
static uint32_t g_lastPubMs   = 0;
static bool     g_lastAvailBle= false;
static const uint32_t HEARTBEAT_MS = 30000;

// Hydro-only settings exposed as HA number entities (read/write, °C). One table drives
// discovery, the state JSON and the command handling. object_id = state-JSON key = set/<id> topic.
struct HydroNum { const char* oid; const char* name; uint16_t reg; float mn, mx, step; bool needsPuffer; };
static const HydroNum HYDRO_NUMS[] = {
  {"hyst_amb_neg", "Ambient hysteresis -",        REG_HYST_AMB_NEG, 0, 20, 0.1f, false},
  {"hyst_amb_pos", "Ambient hysteresis +",        REG_HYST_AMB_POS, 0, 20, 0.1f, false},
  {"ss_neg",       "Start/Stop hysteresis -",     REG_HYST_SS_NEG,  0, 20, 0.1f, false},
  {"ss_pos",       "Start/Stop hysteresis +",     REG_HYST_SS_POS,  0, 20, 0.1f, false},
  {"pump_min_on",  "Circulation pump min. temp.", REG_PUMP_MIN_ON, 20, 90, 1.0f, false},
  {"water_set",    "Water temperature setpoint",  REG_SET_PUFFER,  30, 85, 1.0f, true },  // set_puffer
};
static const int HYDRO_NUM_CNT = sizeof(HYDRO_NUMS)/sizeof(HYDRO_NUMS[0]);
static bool hydroNumAvail(const HydroNum& h){ return !h.needsPuffer || g_caps.puffer; }
static float hydroVal(uint16_t reg){    // current OvenState value for a hydro-param register
  switch(reg){
    case REG_HYST_AMB_NEG: return g_oven.hystAmbNeg;
    case REG_HYST_AMB_POS: return g_oven.hystAmbPos;
    case REG_HYST_SS_NEG:  return g_oven.hystSSNeg;
    case REG_HYST_SS_POS:  return g_oven.hystSSPos;
    case REG_PUMP_MIN_ON:  return g_oven.pumpMinOn;
    case REG_SET_PUFFER:   return g_oven.pufferSet;
  } return NAN;
}

// ---- Identity / Topics ----------------------------------------------------
static void buildDevName(){
  // Name uses the same MAC as MQTT topic/ID (g_id) -> immediately correct, no waiting for serial.
  g_devName = String(DEVICE_NAME) + " (" + g_id + ")";
}
static bool buildIdentity(){
  String cfg = DEVICE_ID;                       // optional override from config.h
  String id  = cfg.length() ? cfg : g_ovenMac;  // otherwise: oven BLE MAC
  if (id.length() == 0) return false;           // no identity yet (BLE not connected)
  g_id        = id;
  T_BASE      = String("mcz/") + g_id;
  T_STATE     = T_BASE + "/state";
  T_AVAIL     = T_BASE + "/availability";
  T_SET_TEMP  = T_BASE + "/set/temp";
  T_SET_POWER = T_BASE + "/set/power";
  T_SET_MODE  = T_BASE + "/set/mode";
  T_SET_ONOFF = T_BASE + "/set/onoff";
  T_SET_FAN   = T_BASE + "/set/fan";
  T_SET_SILENT= T_BASE + "/set/silent";
  T_SET_ALARM_RESET = T_BASE + "/set/alarm_reset";
  buildDevName();
  return true;
}

// ---- Helpers --------------------------------------------------------------
static void addDevice(JsonObject o){
  JsonObject dev = o["device"].to<JsonObject>();
  dev["identifiers"].to<JsonArray>().add(g_id);
  dev["name"]         = g_devName;
  dev["manufacturer"] = "MCZ";
  // Model = Maestro + detected database id (banca_dati) + AIR/HYDRO once known.
  String model = "Maestro";
  if (g_caps.detected){
    if (g_caps.bancaDati.length()) model += " " + g_caps.bancaDati;
    model += g_caps.hydro ? " (Hydro)" : " (Air)";
  }
  dev["model"]        = model;
  if (g_ovenSerial.length()) dev["serial_number"] = g_ovenSerial;
}
static void publishJson(const String& topic, JsonDocument& doc, bool retain){
  char buf[2048];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  bool ok = mqtt.publish(topic.c_str(), (const uint8_t*)buf, n, retain);
  if (!ok){ mqtt.loop(); delay(10); ok = mqtt.publish(topic.c_str(), (const uint8_t*)buf, n, retain); }
  if (!ok) Serial.printf("!! MQTT publish failed (%s, %u B) - buffer/network?\n", topic.c_str(), (unsigned)n);
  // Flush the socket so the discovery burst does not overflow the TCP send buffer
  // (otherwise individual publishes fail -> HA does not create entities).
  mqtt.loop(); delay(2);
}
static String discoTopic(const char* component, const char* objectId){
  return String(HA_DISCOVERY_PREFIX) + "/" + component + "/" + g_id + "_" + objectId + "/config";
}
static void addSensor(const char* objectId, const char* name, const char* valTpl,
                      const char* unit, const char* devClass, const char* stateClass){
  JsonDocument d;
  d["name"]               = name;
  d["unique_id"]          = g_id + "_" + objectId;
  d["availability_topic"] = T_AVAIL;
  d["state_topic"]        = T_STATE;
  d["value_template"]     = valTpl;
  if (unit && *unit)             d["unit_of_measurement"] = unit;
  if (devClass && *devClass)     d["device_class"] = devClass;
  if (stateClass && *stateClass) d["state_class"] = stateClass;
  addDevice(d.as<JsonObject>());
  publishJson(discoTopic("sensor", objectId), d, true);
}

static void publishDiscovery(){
  buildDevName();                 // always build display name from current serial number
  { // climate
    JsonDocument d;
    d["name"]      = "Stove";
    d["unique_id"] = g_id + "_climate";
    d["availability_topic"] = T_AVAIL;
    d["current_temperature_topic"]    = T_STATE;
    d["current_temperature_template"] = "{{ value_json.room }}";
    d["temperature_command_topic"]    = T_SET_TEMP;
    d["temperature_state_topic"]      = T_STATE;
    d["temperature_state_template"]   = "{{ value_json.setpoint }}";
    d["min_temp"] = TEMP_MIN_C; d["max_temp"] = TEMP_MAX_C; d["temp_step"] = 0.5;
    d["temperature_unit"] = "C";
    JsonArray modes = d["modes"].to<JsonArray>(); modes.add("off"); modes.add("heat");
    d["mode_command_topic"]  = T_SET_ONOFF;
    d["mode_state_topic"]    = T_STATE;
    d["mode_state_template"] = "{{ 'heat' if value_json.running else 'off' }}";
    JsonArray pm = d["preset_modes"].to<JsonArray>();
    pm.add("Comfort"); pm.add("Overnight"); pm.add("Turbo"); pm.add("Auto"); pm.add("Manual");
    d["preset_mode_command_topic"]  = T_SET_MODE;
    d["preset_mode_state_topic"]    = T_STATE;
    d["preset_mode_value_template"] = "{{ value_json.mode_name }}";   // HA key is *_value_template here
    addDevice(d.as<JsonObject>());
    publishJson(discoTopic("climate", "climate"), d, true);
  }
  { // number: power
    JsonDocument d;
    d["name"]      = "Power";
    d["unique_id"] = g_id + "_power";
    d["availability_topic"] = T_AVAIL;
    d["command_topic"] = T_SET_POWER;
    d["state_topic"]   = T_STATE;
    d["value_template"]= "{{ value_json.power }}";
    d["min"] = 1; d["max"] = 5; d["step"] = 1; d["mode"] = "slider";
    addDevice(d.as<JsonObject>());
    publishJson(discoTopic("number", "power"), d, true);
  }
  // Fan/Silent only exist if the stove actually has a controllable fan (capability scan).
  int fanLevels = g_caps.detected ? g_caps.fanLevels : 5;
  bool hasFan   = !g_caps.detected || g_caps.fanCount > 0;   // before detection: assume yes
  if (hasFan){ // select: fan level (Auto + 1..fanLevels, per detected hardware)
    JsonDocument d;
    d["name"]      = "Fan";
    d["unique_id"] = g_id + "_fan";
    d["availability_topic"] = T_AVAIL;
    d["command_topic"] = T_SET_FAN;
    d["state_topic"]   = T_STATE;
    // Live level 0x0324 (1..N) -> option; Auto is not distinguishable from the actual value.
    d["value_template"]= "{{ value_json.fan_level | string if value_json.fan_level is defined else '' }}";
    JsonArray op = d["options"].to<JsonArray>();
    op.add("Auto");
    for (int i=1;i<=fanLevels;i++){ char b[4]; snprintf(b,sizeof(b),"%d",i); op.add(b); }
    addDevice(d.as<JsonObject>());
    publishJson(discoTopic("select", "fan"), d, true);
  }
  if (hasFan){ // switch: silent mode
    JsonDocument d;
    d["name"]      = "Silent";
    d["unique_id"] = g_id + "_silent";
    d["availability_topic"] = T_AVAIL;
    d["command_topic"] = T_SET_SILENT;
    d["state_topic"]   = T_STATE;
    d["value_template"]= "{{ 'ON' if value_json.silent else 'OFF' }}";
    d["payload_on"]  = "on"; d["payload_off"] = "off";
    d["state_on"]    = "ON"; d["state_off"]   = "OFF";
    d["icon"]        = "mdi:volume-off";
    addDevice(d.as<JsonObject>());
    publishJson(discoTopic("switch", "silent"), d, true);
  }
  // Hydro-only sensors: published solely when the capability scan found the circuit.
  if (g_caps.hydro)  addSensor("boiler_temp", "Boiler temperature", "{{ value_json.boiler_temp }}", "°C", "temperature", "measurement");
  if (g_caps.puffer) addSensor("puffer_temp", "Buffer temperature", "{{ value_json.puffer_temp }}", "°C", "temperature", "measurement");
  if (g_caps.hydro) for (int i=0;i<HYDRO_NUM_CNT;i++){   // hydro settings as writable number entities
    const HydroNum& h = HYDRO_NUMS[i];
    if (!hydroNumAvail(h)) continue;                     // e.g. water setpoint only if a puffer exists
    JsonDocument d;
    d["name"]      = h.name;
    d["unique_id"] = g_id + "_" + h.oid;
    d["availability_topic"] = T_AVAIL;
    d["command_topic"] = T_BASE + "/set/" + h.oid;
    d["state_topic"]   = T_STATE;
    d["value_template"]= String("{{ value_json.") + h.oid + " }}";
    d["min"] = h.mn; d["max"] = h.mx; d["step"] = h.step;
    d["unit_of_measurement"] = "°C"; d["mode"] = "box"; d["entity_category"] = "config";
    addDevice(d.as<JsonObject>());
    publishJson(discoTopic("number", h.oid), d, true);
  }
  addSensor("fumes", "Flue gas temperature", "{{ value_json.fumes }}",  "°C", "temperature", "measurement");
  addSensor("board", "Control board temperature", "{{ value_json.board }}", "°C", "temperature", "measurement");
  addSensor("room",  "Room temperature",     "{{ value_json.room }}",   "°C", "temperature", "measurement");
  addSensor("fan_room", "Flue gas fan",  "{{ value_json.fan_room }}", "rpm", "", "measurement");
  addSensor("fan_comb", "Combustion fan", "{{ value_json.fan_comb }}", "rpm", "", "measurement");
  addSensor("active", "Active",             "{{ value_json.active }}", "", "", "measurement");
  addSensor("phase", "Phase", "{{ value_json.phase_name }}", "", "", "");
  addSensor("ignitions", "Ignitions", "{{ value_json.ignitions }}", "", "", "total_increasing");
  addSensor("worktime", "Total working time", "{{ value_json.worktime_min }}", "min", "duration", "total_increasing");
  for (int i=1;i<=5;i++){
    char oid[12], name[20], tpl[40];
    snprintf(oid,  sizeof(oid),  "ptime%d", i);
    snprintf(name, sizeof(name), "Time power %d", i);
    snprintf(tpl,  sizeof(tpl),  "{{ value_json.pt%d }}", i);
    addSensor(oid, name, tpl, "min", "duration", "total_increasing");
  }
  // Live alarm (0x0323): is the oven in an alarm RIGHT NOW? + button to acknowledge it.
  { JsonDocument d;
    d["name"]="Alarm"; d["unique_id"]=g_id+"_alarm_active";
    d["availability_topic"]=T_AVAIL; d["state_topic"]=T_STATE;
    d["value_template"]="{{ 'ON' if value_json.alarm_active else 'OFF' }}";
    d["device_class"]="problem";
    d["json_attributes_topic"]=T_STATE;
    d["json_attributes_template"]="{{ {'code': value_json.alarm} | tojson }}";
    addDevice(d.as<JsonObject>()); publishJson(discoTopic("binary_sensor","alarm_active"), d, true);
  }
  { JsonDocument d;
    d["name"]="Acknowledge alarm"; d["unique_id"]=g_id+"_alarm_reset";
    d["availability_topic"]=T_AVAIL; d["command_topic"]=T_SET_ALARM_RESET;
    d["payload_press"]="reset"; d["icon"]="mdi:alert-remove-outline";
    addDevice(d.as<JsonObject>()); publishJson(discoTopic("button","alarm_reset"), d, true);
  }
  // Alarm log (all ovens): last alarm + history (raw Axx codes; meanings are model-specific)
  { JsonDocument d;
    d["name"]="Last alarm"; d["unique_id"]=g_id+"_alarm_last";
    d["availability_topic"]=T_AVAIL; d["state_topic"]=T_STATE;
    d["value_template"]="{{ value_json.alarm_last }}"; d["icon"]="mdi:alert-circle-outline";
    addDevice(d.as<JsonObject>()); publishJson(discoTopic("sensor","alarm_last"), d, true);
  }
  { JsonDocument d;
    d["name"]="Alarm history"; d["unique_id"]=g_id+"_alarm_hist";
    d["availability_topic"]=T_AVAIL; d["state_topic"]=T_STATE;
    d["value_template"]="{{ value_json.alarm_hist }}";
    d["json_attributes_topic"]=T_STATE;
    d["json_attributes_template"]="{{ {'codes': value_json.alarm_codes} | tojson }}";
    d["icon"]="mdi:history";
    addDevice(d.as<JsonObject>()); publishJson(discoTopic("sensor","alarm_hist"), d, true);
  }
}

static void publishState(){
  JsonDocument d;
  if (!isnan(g_oven.roomC))     d["room"]     = roundf(g_oven.roomC*10)/10.0;
  if (!isnan(g_oven.setpointC)) d["setpoint"] = roundf(g_oven.setpointC*10)/10.0;
  if (!isnan(g_oven.boardC))    d["board"]    = roundf(g_oven.boardC*10)/10.0;
  if (!isnan(g_oven.fumesC))    d["fumes"]    = roundf(g_oven.fumesC*10)/10.0;
  if (g_oven.power>=0)          d["power"]    = g_oven.power;
  if (g_oven.mode>=0){ d["mode"] = g_oven.mode; d["mode_name"] = modeName(g_oven.mode); }
  if (g_oven.phase>=0){ d["phase"] = g_oven.phase; d["running"] = ovenRunning(); }
  if (g_oven.state>=0){
    d["state"] = g_oven.state;                 // raw fine-phase code (0x0320)
    const char* sn = stateName(g_oven.state);
    if (sn[0]) d["phase_name"] = sn;
    else { char b[12]; snprintf(b,sizeof(b),"0x%04X",(unsigned)g_oven.state); d["phase_name"] = b; }
  }
  if (g_oven.fanLevel>=0)      d["fan_level"] = g_oven.fanLevel;
  if (g_oven.fanRoom>=0)       d["fan_room"]  = g_oven.fanRoom;   // flue gas fan RPM
  if (g_oven.fanComb>=0)       d["fan_comb"]  = g_oven.fanComb;   // combustion fan RPM
  if (g_oven.active>=0)        d["active"]    = g_oven.active;    // app value "active"
  if (g_oven.flags>=0){ d["chrono"] = (g_oven.flags>>6)&1; d["silent"] = (g_oven.flags>>5)&1; }
  if (g_oven.ignitions>=0)   d["ignitions"]    = g_oven.ignitions;
  if (g_oven.worktimeMin>=0) d["worktime_min"] = g_oven.worktimeMin;
  for (int i=0;i<5;i++) if (g_oven.powerTimeMin[i]>=0){
    char k[8]; snprintf(k,sizeof(k),"pt%d",i+1); d[k] = g_oven.powerTimeMin[i];
  }
  d["ble"] = g_oven.bleOnline;
  d["seq"] = g_oven.seq;
  if (g_ovenSerial.length()) d["serial"] = g_ovenSerial;
  if (g_oven.alarm >= 0){                        // live alarm: "A12" while active, else "none"
    d["alarm_active"] = g_oven.alarm > 0;
    if (g_oven.alarm > 0){ char a[8]; snprintf(a,sizeof(a),"A%d",g_oven.alarm); d["alarm"] = a; }
    else d["alarm"] = "none";
  }
  if (g_oven.alarmHist[0] >= 0){                 // alarm log (raw Axx codes, newest first)
    char la[8];
    if (g_oven.alarmHist[0] > 0) snprintf(la,sizeof(la),"A%d",g_oven.alarmHist[0]); else strncpy(la,"none",sizeof(la));
    d["alarm_last"] = la;
    String hist; JsonArray codes = d["alarm_codes"].to<JsonArray>();
    for (int k=0;k<10 && g_oven.alarmHist[k]>=0;k++){
      if (g_oven.alarmHist[k] <= 0) continue;
      if (hist.length()) hist += ",";
      hist += "A"; hist += g_oven.alarmHist[k];
      codes.add(g_oven.alarmHist[k]);
    }
    d["alarm_hist"] = hist;
  }
  if (g_caps.detected){                          // detected hardware capabilities
    d["hydro"]      = g_caps.hydro;
    d["fan_count"]  = g_caps.fanCount;
    d["fan_levels"] = g_caps.fanLevels;
    d["boiler"]     = g_caps.boiler;
    d["puffer"]     = g_caps.puffer;
    if (g_caps.bancaDati.length()) d["model"] = g_caps.bancaDati;
    // Hydro live temperatures (only meaningful / published on water-heating stoves)
    if (g_caps.hydro  && !isnan(g_oven.boilerC)) d["boiler_temp"] = roundf(g_oven.boilerC*10)/10.0;
    if (g_caps.puffer && !isnan(g_oven.pufferC)) d["puffer_temp"] = roundf(g_oven.pufferC*10)/10.0;
    if (g_caps.hydro) for (int i=0;i<HYDRO_NUM_CNT;i++){   // hydro settings
      float v = hydroVal(HYDRO_NUMS[i].reg);
      if (!isnan(v)) d[HYDRO_NUMS[i].oid] = roundf(v*10)/10.0;
    }
  }
  publishJson(T_STATE, d, true);
}

static void publishAvail(bool online){
  mqtt.publish(T_AVAIL.c_str(), online ? "online" : "offline", true);
}

// ---- Command callback -----------------------------------------------------
static int parseMode(const String& s){
  String t = s; t.trim(); t.toLowerCase();
  if (t=="0" || t=="manuell" || t=="manual")             return 0;
  if (t=="1" || t=="auto")                               return 1;
  if (t=="2" || t=="nacht" || t=="night" || t=="overnight") return 2;
  if (t=="3" || t=="comfort" || t=="komfort")            return 3;
  if (t=="4" || t=="turbo")                              return 4;
  return -1;
}
static void onMqtt(char* topic, byte* payload, unsigned int len){
  String t(topic), msg; msg.reserve(len);
  for (unsigned i=0;i<len;i++) msg += (char)payload[i];
  msg.trim();
  if      (t == T_SET_TEMP)  ovenSetTemp(msg.toFloat());
  else if (t == T_SET_POWER) ovenSetPower(msg.toInt());
  else if (t == T_SET_MODE)  ovenSetMode(parseMode(msg));
  else if (t == T_SET_ONOFF){ String m=msg; m.toLowerCase();
    if (m=="heat"||m=="on"||m=="1")  ovenSetOnOff(true);
    else if (m=="off"||m=="0")       ovenSetOnOff(false);
  }
  else if (t == T_SET_FAN){ String m=msg; m.toLowerCase();
    ovenSetFan(m=="auto" ? 0 : msg.toInt());        // "Auto"/"0" -> Auto, "1".."5" -> level
  }
  else if (t == T_SET_ALARM_RESET) ovenResetAlarm();
  else if (t == T_SET_SILENT){ String m=msg; m.toLowerCase();
    if (m=="on"||m=="1"||m=="true")   ovenSetSilent(true);
    else if (m=="off"||m=="0"||m=="false") ovenSetSilent(false);
  }
  else for (int i=0;i<HYDRO_NUM_CNT;i++){                  // hydro settings (set/<oid>)
    if (t == T_BASE + "/set/" + HYDRO_NUMS[i].oid){ ovenSetTempParam(HYDRO_NUMS[i].reg, msg.toFloat()); break; }
  }
}

static bool mqttConnect(){
  String cid = String("mcz-") + g_id;
  bool ok;
  if (g_cfg.mqttUser.length() > 0)
    ok = mqtt.connect(cid.c_str(), g_cfg.mqttUser.c_str(), g_cfg.mqttPass.c_str(),
                      T_AVAIL.c_str(), 0, true, "offline");
  else
    ok = mqtt.connect(cid.c_str(), nullptr, nullptr, T_AVAIL.c_str(), 0, true, "offline");
  if (!ok) return false;
  Serial.printf(">> MQTT connected (id=%s).\n", g_id.c_str());
  mqtt.subscribe(T_SET_TEMP.c_str());
  mqtt.subscribe(T_SET_POWER.c_str());
  mqtt.subscribe(T_SET_MODE.c_str());
  mqtt.subscribe(T_SET_ONOFF.c_str());
  mqtt.subscribe(T_SET_FAN.c_str());
  mqtt.subscribe(T_SET_SILENT.c_str());
  mqtt.subscribe(T_SET_ALARM_RESET.c_str());
  for (int i=0;i<HYDRO_NUM_CNT;i++)                 // hydro settings (harmless if not a hydro oven)
    mqtt.subscribe((T_BASE + "/set/" + HYDRO_NUMS[i].oid).c_str());
  publishDiscovery(); g_discSerial = g_ovenSerial;  // name = MAC -> immediately correct
  publishAvail(g_oven.bleOnline); g_lastAvailBle = g_oven.bleOnline;
  publishState();                     // send state IMMEDIATELY (do not wait for the next tick)
  g_lastPubSeq = g_oven.seq; g_lastPubMs = millis();
  return true;
}

// ---- API ------------------------------------------------------------------
void netBegin(){
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);                // BLE+WiFi coexistence: modem sleep MUST be on
  WiFi.begin(g_cfg.wifiSsid.c_str(), g_cfg.wifiPass.c_str());
  Serial.printf(">> WiFi: connecting to '%s' ...\n", g_cfg.wifiSsid.c_str());
  mqtt.setServer(g_cfg.mqttHost.c_str(), g_cfg.mqttPort);
  mqtt.setBufferSize(2048);          // large HA discovery payloads (climate ~1 kB)
  mqtt.setCallback(onMqtt);
}

void netTick(){
  static uint32_t lastWifi=0, lastMqtt=0;
  uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED){
    if (now - lastWifi > 5000){ lastWifi = now; WiFi.reconnect(); }
    return;
  }
  if (!g_idReady){                    // wait for identity (oven MAC)
    if (buildIdentity()){ g_idReady = true; Serial.printf(">> MQTT ID: %s\n", g_id.c_str()); }
    else return;
  }
  if (!mqtt.connected()){
    if (now - lastMqtt > 3000){ lastMqtt = now; mqttConnect(); }
    return;
  }
  mqtt.loop();
  if (g_oven.bleOnline != g_lastAvailBle){ publishAvail(g_oven.bleOnline); g_lastAvailBle = g_oven.bleOnline; }
  // Serial number AND the capability scan only complete after the connect -> re-publish
  // discovery once each becomes known (serial_number metadata; fan entity/level count
  // matched to the detected hardware).
  static bool g_discCaps = false;
  if (g_ovenSerial != g_discSerial || g_caps.detected != g_discCaps){
    publishDiscovery(); g_discSerial = g_ovenSerial; g_discCaps = g_caps.detected;
  }
  // Publish on change, but throttled to >=1s apart (less heap churn / MQTT load over hours);
  // plus a 30s heartbeat so HA stays fresh even without changes.
  bool changed = (g_oven.seq != g_lastPubSeq) && (now - g_lastPubMs >= 1000);
  if (changed || now - g_lastPubMs > HEARTBEAT_MS){
    publishState(); g_lastPubSeq = g_oven.seq; g_lastPubMs = now;
  }
}

bool netWifiUp(){ return WiFi.status()==WL_CONNECTED; }
bool netMqttUp(){ return mqtt.connected(); }
#else
void netBegin(){}
void netTick(){}
bool netWifiUp(){ return false; }
bool netMqttUp(){ return false; }
#endif
