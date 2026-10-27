// =============================================================================
// Protocol 
//   - Write to abf1 (UUID 0xABF1), responses as Notify on abf2 (0xABF2).
//   - Frame = AES-128-CBC(KEY1, IV) of:
//         [4 byte counter (LE)][16 byte token][Modbus-RTU-PDU][PKCS#7 padding to 16]
//   - Inside: standard Modbus RTU, slave 0x01:
//         Function 0x03 = read holding register:   01 03 regHi regLo cntHi cntLo crcLo crcHi
//         Function 0x06 = write single register:   01 06 regHi regLo valHi valLo crcLo crcHi
//         CRC16 (Modbus, poly 0xA001), appended as 2 bytes little-endian.
//
// This lets the sketch both READ and CONTROL the oven via the ORIGINAL module BLE.
//
// Serial commands (monitor 115200):
//   r <regHex> <count>     -> Modbus read   (e.g. "r 02bc 33" reads 0x02BC, 51 registers)
//   w <regHex> <valHex>    -> Modbus write  (e.g. "w 0320 0001")  ** changes the oven! **
//   ctr <hex>              -> set counter start value (default 0x6A418000)
//   poll                   -> reads 0x02BC x0x33 (main status block), like the app
//   help
//
// =============================================================================

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "mbedtls/aes.h"
#include <WiFi.h>          // RSSI in heartbeat
#include <time.h>          // NTP time for the oven clock (getLocalTime/configTzTime)
#include "oven.h"
#include "net_mqtt.h"
#include "display.h"
#include "appconfig.h"
#include "ble_api.h"
#include "chrono.h"
#if defined(__has_include)
#  if __has_include("config.h")
#    include "config.h"
#  endif
#endif
#ifndef TARGET_MAC
#  define TARGET_MAC ""     // empty = first available MCZ_EP; otherwise fixed oven MAC (config.h)
#endif
#ifndef OVEN_TZ
#  define OVEN_TZ "CET-1CEST,M3.5.0,M10.5.0/3"   // fallback
#endif


// ---- Crypto constants (firmware-verified) ----------------------------------
static const uint8_t KEY1[16] = {0x6e,0x29,0x6b,0x0b,0xbb,0x1d,0x43,0xf3,0x6e,0x47,0xf7,0x2e,0x7b,0x6f,0x2e,0x77};
static const uint8_t IV0 [16] = {0xda,0x1a,0x55,0x73,0x49,0xf2,0x5c,0x64,0x1b,0x1a,0x36,0x8a,0xf5,0xb2,0x18,0xa7};
static const uint8_t TOKEN[16]= {0x31,0xdd,0x34,0x51,0x26,0x39,0x37,0x7b,0x05,0xa2,0x51,0x0d,0xe7,0x25,0xfc,0x75};

static uint32_t g_counter = 0x6A418000;   // start value; incremented per frame

static const char *TARGET_PREFIX = "MCZ_EP";
#define SCAN_SECONDS 8

static NimBLEAdvertisedDevice    *g_target=nullptr;
static bool                       g_doConnect=false, g_connected=false;
static volatile bool              g_linkUp=false;
static NimBLEScan                *g_scan=nullptr;
static NimBLEClient              *g_client=nullptr;
static NimBLERemoteCharacteristic *g_abf1=nullptr, *g_abf2=nullptr;

OvenState g_oven;                       // Single Source of Truth (oven.h)
OvenCaps  g_caps;                       // detected hardware capabilities (oven.h)
// ---- Alarm log (ring buffer, 100 slots; code at 0x07EE+4N) ----
static uint8_t g_alarmCode[100];        // raw code per slot (filled during the log block read)
static int     g_alarmIndex = -1;       // index_allarme (write head)
static int     g_alarmNum   = 0;        // num_allarmi (total logged, capped 100)
static int     g_alarmLivePrev = -1;    // last live tipo_allarme (0x0323) to detect new alarms
static bool    g_alarmDone=false, g_alarmDirty=true, g_alarmScanning=false;
String    g_ovenMac;                    // oven BLE MAC without ':' (identity)
String    g_ovenSerial;                 // serial number from 0x0ADC
String    g_adcHex;                     // raw 0x0ADC (hex) for diagnostics
static uint16_t g_lastReadBase = 0;     // base address of the last sent 0x03 read
// Synchronous block-read capture (bleReadRegs, used by chrono.cpp etc.)
static volatile bool g_rawCapture = false;
static uint16_t      g_rawBase = 0;
static uint16_t*     g_rawDst = nullptr;
static volatile int  g_rawGot = -1;
static bool     g_logRaw = true;        // verbose Modbus dumps (quieter via 'log off')
static const uint32_t POLL_INTERVAL_MS = 2500;  // auto-poll rate per status block

// ---- Modbus CRC16 ----------------------------------------------------------
static uint16_t modbusCRC(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i=0;i<n;i++){ crc ^= d[i];
    for (int b=0;b<8;b++) crc = (crc&1) ? (crc>>1)^0xA001 : (crc>>1); }
  return crc;
}

// ---- AES-128-CBC (mbedtls), IV is copied (not destroyed) -------------------
static void aesEnc(const uint8_t *in, uint8_t *out, size_t len) {
  mbedtls_aes_context a; mbedtls_aes_init(&a); mbedtls_aes_setkey_enc(&a, KEY1, 128);
  uint8_t iv[16]; memcpy(iv, IV0, 16);
  mbedtls_aes_crypt_cbc(&a, MBEDTLS_AES_ENCRYPT, len, iv, in, out);
  mbedtls_aes_free(&a);
}
static void aesDec(const uint8_t *in, uint8_t *out, size_t len) {
  mbedtls_aes_context a; mbedtls_aes_init(&a); mbedtls_aes_setkey_dec(&a, KEY1, 128);
  uint8_t iv[16]; memcpy(iv, IV0, 16);
  mbedtls_aes_crypt_cbc(&a, MBEDTLS_AES_DECRYPT, len, iv, in, out);
  mbedtls_aes_free(&a);
}

// ---- Build frame: Modbus PDU (without CRC) -> encrypted BLE frame ----------
// pdu: e.g. {01,03,reg_hi,reg_lo,cnt_hi,cnt_lo}; CRC is appended here.
static size_t buildFrame(const uint8_t *pdu, size_t pduLen, uint8_t *outCipher) {
  uint8_t plain[64]; size_t p=0;
  uint32_t c = g_counter++;
  plain[p++]=c&0xFF; plain[p++]=(c>>8)&0xFF; plain[p++]=(c>>16)&0xFF; plain[p++]=(c>>24)&0xFF;
  memcpy(plain+p, TOKEN, 16); p+=16;
  memcpy(plain+p, pdu, pduLen); p+=pduLen;
  uint16_t crc = modbusCRC(pdu, pduLen);
  plain[p++]=crc&0xFF; plain[p++]=crc>>8;
  size_t pad = 16 - (p % 16); if (pad==0) pad=16;
  for (size_t i=0;i<pad;i++) plain[p++]=(uint8_t)pad;   // PKCS#7
  aesEnc(plain, outCipher, p);
  return p;
}

static void writeFrame(const uint8_t *pdu, size_t pduLen) {
  if (!g_abf1) { Serial.println("!! abf1 not available"); return; }
  uint8_t ct[64]; size_t n = buildFrame(pdu, pduLen, ct);
  Serial.print(">> abf1 <- "); for(size_t i=0;i<n;i++) Serial.printf("%02X",ct[i]); Serial.println();
  g_abf1->writeValue(ct, n, false);   // WriteNoResponse
}

static void sendRead(uint16_t reg, uint16_t count) {
  g_lastReadBase = reg;   // response (function 03) carries no address -> remember base here
  uint8_t pdu[6]={0x01,0x03,(uint8_t)(reg>>8),(uint8_t)reg,(uint8_t)(count>>8),(uint8_t)count};
  Serial.printf("Modbus READ  reg=0x%04X count=%u\n", reg, count); writeFrame(pdu,6);
}
static void sendWrite(uint16_t reg, uint16_t val) {
  uint8_t pdu[6]={0x01,0x06,(uint8_t)(reg>>8),(uint8_t)reg,(uint8_t)(val>>8),(uint8_t)val};
  Serial.printf("Modbus WRITE reg=0x%04X val=0x%04X\n", reg, val); writeFrame(pdu,6);
}
// Modbus function 0x10 = write multiple registers (atomic block). Needed e.g. for the
// date/time set, where the module only commits the date as one transaction.
static void sendWriteMulti(uint16_t reg, const uint16_t* vals, uint8_t n) {
  if(n==0 || n>24) return;
  uint8_t pdu[7+48];
  pdu[0]=0x01; pdu[1]=0x10; pdu[2]=(uint8_t)(reg>>8); pdu[3]=(uint8_t)reg;
  pdu[4]=0; pdu[5]=n; pdu[6]=(uint8_t)(n*2);
  for(uint8_t i=0;i<n;i++){ pdu[7+2*i]=(uint8_t)(vals[i]>>8); pdu[7+2*i+1]=(uint8_t)(vals[i]&0xFF); }
  Serial.printf("Modbus WRITE-MULTI reg=0x%04X n=%u\n", reg, n);
  writeFrame(pdu, 7+n*2);
}
// Synchronous block read (ble_api.h): send fn03 read, wait for the notify response.
// Call from loop context (e.g. a serial command), never from a BLE callback.
bool bleReadRegs(uint16_t reg, uint16_t count, uint16_t* dst){
  if(!g_connected || !g_abf1 || !dst || count==0 || count>125) return false;
  g_rawDst=dst; g_rawBase=reg; g_rawGot=-1; g_rawCapture=true;
  sendRead(reg, count);
  uint32_t t0=millis();
  while(g_rawGot<0 && millis()-t0<1500) delay(5);   // await response
  g_rawCapture=false;
  return g_rawGot>=(int)count;
}

// ---- Register -> OvenState (source irrelevant: poll read or ##-broadcast) ---
static void bumpSeq(){ g_oven.seq++; g_oven.lastUpdateMs = millis(); }
void ovenApplyReg(uint16_t reg, uint16_t val){
  static uint16_t workLo = 0;        // low word of the 32-bit work time (persists between calls)
  // Time in power level 1..5 (0x0336..0x033F): 5x 32-bit seconds, low word first -> minutes
  if (reg >= REG_PTIME_LO && reg <= REG_PTIME_HI){
    static uint16_t ptLo[5] = {0};
    int idx = (reg - REG_PTIME_LO) / 2;          // 0..4
    if ((reg & 1) == 0){ ptLo[idx] = val; }       // even address = low word
    else {                                        // odd = high word -> minutes
      int32_t m = (int32_t)((((uint32_t)val<<16) | ptLo[idx]) / 60);
      if (g_oven.powerTimeMin[idx] != m){ g_oven.powerTimeMin[idx] = m; bumpSeq(); }
    }
    return;
  }
  // alarm log codes: code register at 0x07EE + 4*N (N=0..99) -> per-slot buffer
  if (reg >= 0x07EE && reg <= 0x097A && ((reg - 0x07EE) % 4) == 0){
    g_alarmCode[(reg - 0x07EE)/4] = (uint8_t)val; return;
  }
  // banca_dati name (0x07C7..0x07CE): ASCII, 2 chars per register -> capability id
  if (reg >= REG_BANCADATI+1 && reg <= 0x07CE){
    if (reg == REG_BANCADATI+1) g_caps.bancaDati = "";
    char a=(char)(val>>8), b=(char)(val&0xFF);
    if (a>=32 && a<127) g_caps.bancaDati += a;
    if (b>=32 && b<127) g_caps.bancaDati += b;
    return;
  }
  switch(reg){
    // ---- Capability registers (filled once during capScan, see below) ----
    case REG_MAXPOT_IDRO: g_caps.maxPotIdro=val; g_caps.hydro=(val!=SENTINEL16 && val>0); break;
    case REG_SET_BOILER:  g_caps.boiler=(val!=SENTINEL16); break;
    case REG_SET_PUFFER: { g_caps.puffer=(val!=SENTINEL16);
        float c=(val==SENTINEL16)?NAN:val/10.0f;
        if(g_oven.pufferSet!=c && !(isnan(g_oven.pufferSet)&&isnan(c))){ g_oven.pufferSet=c; bumpSeq(); } } break;
    case REG_TIPO_APP:    g_caps.tipoApp=val; break;
    case REG_CLIMA:       g_caps.clima=(val!=0 && val!=SENTINEL16); break;
    case REG_ABI_VEN12:   g_caps.fanCount = (uint8_t)(((val&0xFF)!=0)+((val>>8)!=0)); break;   // fan1/2
    case REG_ABI_VEN34:   g_caps.fanCount += (uint8_t)(((val&0xFF)!=0)+((val>>8)!=0)); break;  // +fan3/4
    case REG_VVEN1_LO:    g_caps.fanLevels = (uint8_t)(((val&0xFF)!=0)+((val>>8)!=0)); break;   // v_ven1 v0..v1
    case 0x05FE:          g_caps.fanLevels += (uint8_t)(((val&0xFF)!=0)+((val>>8)!=0)); break;  // v2..v3
    case REG_VVEN1_HI:    g_caps.fanLevels += (uint8_t)(((val&0xFF)!=0)+((val>>8)!=0));         // v4..v5
                          if(g_caps.fanLevels>5) g_caps.fanLevels=5;
                          if(g_caps.fanLevels<1) g_caps.fanLevels=1; break;
    case REG_ROOM:    { float c=val/10.0f; if(g_oven.roomC!=c)    {g_oven.roomC=c;     bumpSeq();} } break;
    case REG_BOILER_T:{ float c=val/10.0f; if(g_oven.boilerC!=c)  {g_oven.boilerC=c;   bumpSeq();} } break;
    case REG_PUFFER_T:{ float c=val/10.0f; if(g_oven.pufferC!=c)  {g_oven.pufferC=c;   bumpSeq();} } break;
    case REG_HYST_AMB_NEG:{ float c=val/10.0f; if(g_oven.hystAmbNeg!=c){g_oven.hystAmbNeg=c;bumpSeq();} } break;
    case REG_HYST_AMB_POS:{ float c=val/10.0f; if(g_oven.hystAmbPos!=c){g_oven.hystAmbPos=c;bumpSeq();} } break;
    case REG_HYST_SS_NEG: { float c=val/10.0f; if(g_oven.hystSSNeg !=c){g_oven.hystSSNeg =c;bumpSeq();} } break;
    case REG_HYST_SS_POS: { float c=val/10.0f; if(g_oven.hystSSPos !=c){g_oven.hystSSPos =c;bumpSeq();} } break;
    case REG_PUMP_MIN_ON: { float c=val/10.0f; if(g_oven.pumpMinOn !=c){g_oven.pumpMinOn =c;bumpSeq();} } break;
    case REG_BOARD:   { float c=val/10.0f; if(g_oven.boardC!=c)   {g_oven.boardC=c;    bumpSeq();} } break;
    case REG_FUMES:   { float c=val/10.0f; if(g_oven.fumesC!=c)   {g_oven.fumesC=c;    bumpSeq();} } break;
    case REG_SETPOINT:{ float c=val/10.0f; if(g_oven.setpointC!=c){g_oven.setpointC=c; bumpSeq();} } break;
    case REG_POWER:    if(g_oven.power !=(int8_t)val){ g_oven.power =(int8_t)val; bumpSeq(); } break;
    case REG_MODE:                                                   // setting + live mirror
    case REG_MODE_LIVE:if(g_oven.mode  !=(int8_t)val){ g_oven.mode  =(int8_t)val; bumpSeq(); } break;
    case REG_STATE:    if(g_oven.state !=(int32_t)val){
        // ALWAYS log fine-phase changes (even with 'log off') + context.
        const char* sn = stateName((int)val);
        Serial.printf("[PHASE] t=%lus  0x%04X -> 0x%04X (%s)  Fumes=%.1fC  Fan=%ld rpm\n",
                      (unsigned long)(millis()/1000), (unsigned)(g_oven.state<0?0:g_oven.state),
                      (unsigned)val, sn[0]?sn:"unknown",
                      isnan(g_oven.fumesC)?0.0f:g_oven.fumesC, (long)g_oven.fanComb);
        g_oven.state =(int32_t)val; bumpSeq(); } break;
    case REG_PHASE:    if(g_oven.phase !=(int8_t)val){ g_oven.phase =(int8_t)val; bumpSeq(); } break;
    case REG_ALARM:  { int code=val&0xFF;                               // live tipo_allarme (low byte)
        if(code!=0 && code!=g_alarmLivePrev) g_alarmDirty=true;         // new alarm -> refresh history
        g_alarmLivePrev=code;
        if(g_oven.alarm!=code){ g_oven.alarm=(int16_t)code; bumpSeq(); } } break;
    case REG_ALARM_IDX: g_alarmIndex=(val>>8); g_alarmNum=(val&0xFF); break;  // head / count
    case REG_FLAGS:    if(g_oven.flags !=(int16_t)val){g_oven.flags =(int16_t)val;bumpSeq(); } break;
    case REG_IGNIT:    if(g_oven.ignitions!=(int32_t)val){g_oven.ignitions=(int32_t)val;bumpSeq();} break;
    case REG_ACTIVE:   if(g_oven.active !=(int32_t)val){g_oven.active =(int32_t)val;bumpSeq();} break;
    case REG_FAN_COMB: if(g_oven.fanComb!=(int32_t)val){g_oven.fanComb=(int32_t)val;bumpSeq();} break;
    case REG_FAN_ROOM: if(g_oven.fanRoom!=(int32_t)val){g_oven.fanRoom=(int32_t)val;bumpSeq();} break;
    case REG_FAN_LIVE: if(g_oven.fanLevel!=(int8_t)val){g_oven.fanLevel=(int8_t)val;bumpSeq();} break;
    case REG_WORK_LO:  workLo = val; break;                          // remember low; minutes at HI
    case REG_WORK_HI:  { int32_t m=(int32_t)((((uint32_t)val<<16)|workLo)/60);
                         if(g_oven.worktimeMin!=m){g_oven.worktimeMin=m;bumpSeq();} } break;
    default: break;
  }
}

// ---- Command API (Serial/Display/MQTT) — writes via function 06 ------------
bool ovenSetTemp(float c){
  if(c<TEMP_MIN_C||c>TEMP_MAX_C){ Serial.printf("!! Setpoint out of range (%.1f..%.1f C)\n",TEMP_MIN_C,TEMP_MAX_C); return false; }
  uint16_t v=(uint16_t)(c*10.0f+0.5f);
  Serial.printf(">> Setpoint %.1f C -> reg 0x%04X = %u\n", c, REG_SETPOINT, v);
  sendWrite(REG_SETPOINT, v); return true;
}
bool ovenSetPower(int n){
  if(n<1||n>5){ Serial.println("!! Power 1..5"); return false; }
  Serial.printf(">> Power %d -> reg 0x%04X\n", n, REG_POWER);
  sendWrite(REG_POWER,(uint16_t)n); return true;
}
bool ovenSetMode(int m){
  if(m<0||m>=OVEN_MODE_COUNT){ Serial.println("!! Mode 0=Manual 1=Auto 2=Overnight 3=Comfort 4=Turbo"); return false; }
  Serial.printf(">> Mode %d (%s) -> reg 0x%04X\n", m, modeName(m), REG_MODE);
  sendWrite(REG_MODE,(uint16_t)m); return true;
}
bool ovenSetOnOff(bool on){
  // 0x038A is a TOGGLE (write 1 = power button). Derive the desired state from the
  // phase and only toggle when necessary (phase 1 = off/standby, otherwise = on/running).
  if (g_oven.phase < 0){
    Serial.println("!! Oven phase unknown (no status yet) - sending power toggle 0x038A=1");
    sendWrite(REG_ONOFF, 1); return true;
  }
  bool standby = (g_oven.phase == 1);
  if (on == !standby){            // desired state already matches the actual state
    Serial.printf(">> Oven is already %s (phase %d) - no toggle\n", on?"on":"off", g_oven.phase);
    return false;
  }
  Serial.printf(">> %s: power toggle 0x038A=1 (phase was %d)\n", on?"ON":"OFF", g_oven.phase);
  sendWrite(REG_ONOFF, 1);
  return true;
}
bool ovenResetAlarm(){
  // Only acknowledge when a live alarm is actually pending (never a blind command write).
  if (g_oven.alarm <= 0){
    Serial.printf(">> No active alarm (live code %d) - no reset sent\n", g_oven.alarm);
    return false;
  }
  Serial.printf(">> Acknowledge alarm A%d: 0x%04X=1\n", g_oven.alarm, REG_ALARM_RESET);
  sendWrite(REG_ALARM_RESET, 1);
  return true;
}
bool ovenSetFan(int level){
  // 0 = Auto (writes 6), 1..5 = fixed level. 
  if(level<0||level>5){ Serial.println("!! Fan 0=Auto, 1..5=level"); return false; }
  uint16_t v = level==0 ? 6 : (uint16_t)level;
  Serial.printf(">> Fan %s -> reg 0x%04X = %u\n", level==0?"Auto":String(level).c_str(), REG_FAN_SET, v);
  sendWrite(REG_FAN_SET, v); return true;
}
bool ovenSetSilent(bool on){
  Serial.printf(">> Silent %s -> reg 0x%04X = %u\n", on?"ON":"OFF", REG_SILENT, on?1:0);
  sendWrite(REG_SILENT, on?1:0); return true;
}
// Write a °C-scaled setting (hydro hysteresis / pump min temp). Value = celsius*10.
bool ovenSetTempParam(uint16_t reg, float celsius){
  if(celsius < 0.0f || celsius > 100.0f){ Serial.printf("!! param 0x%04X out of range (0..100 C)\n", reg); return false; }
  uint16_t v = (uint16_t)(celsius*10.0f + 0.5f);
  Serial.printf(">> param 0x%04X = %.1f C -> %u\n", reg, celsius, v);
  sendWrite(reg, v); return true;
}
// Set the oven RTC from NTP time. Layout verified by read-back (see oven.h REG_DT_*).
// Writes 900-903 (local wall-clock time) then triggers 904 to commit into the RTC.
// Timezone OVEN_TZ comes from config.h (fallback defined above).
bool ovenSetClock(){
  if(!g_connected || !g_abf1){ Serial.println("!! clock: BLE not connected"); return false; }
  struct tm t;
  if(!getLocalTime(&t, 2000)){ Serial.println("!! clock: no NTP time yet (WiFi/NTP not ready)"); return false; }
  int year=t.tm_year+1900, mon=t.tm_mon+1, day=t.tm_mday;
  int hh=t.tm_hour, mm=t.tm_min, ss=t.tm_sec;
  int wd=(t.tm_wday+6)%7;                       // tm_wday 0=Sun..6=Sat -> oven 0=Mon..6=Sun
  if(year<2024||year>2099||mon<1||mon>12||day<1||day>31){ Serial.println("!! clock: implausible time, aborting"); return false; }
  // The module only commits the DATE when 900-903 are written as ONE Fn 0x10 transaction
  // (individual Fn 06 writes commit the time but not the date -> verified). 904 = trigger.
  uint16_t vals[5] = {
    (uint16_t)((mon<<8)|day),    // 900 hi=month, lo=day
    (uint16_t)year,              // 901 full year
    (uint16_t)((mm<<8)|hh),      // 902 hi=minute, lo=hour
    (uint16_t)((wd<<8)|ss),      // 903 hi=weekday(module recomputes from date), lo=second
    (uint16_t)(1<<8)             // 904 set_orodatario = HIGH byte -> 0x0100 commits into RTC
  };
  sendWriteMulti(REG_DT_DAYMON, vals, 5);
  Serial.printf(">> Clock set via NTP (Fn16): %04d-%02d-%02d %02d:%02d:%02d (weekday %d, 0=Mon)\n",
                year, mon, day, hh, mm, ss, wd);
  return true;
}

// ---- ##-broadcast: reassembly + decoded status store -----------------------
static const uint16_t STATUS_BASE = 0x02BA;
static uint8_t  g_bcBuf[1024];        // reassembly buffer (ciphertext frag01+frag02)
static size_t   g_bcLen = 0;
static bool     g_bcHave01 = false;
static uint8_t  g_status[768];        // last decoded register payload (from STATUS_BASE)
static size_t   g_statusLen = 0;
static uint32_t g_statusCtr = 0;

// 16-bit register value from last broadcast (-1 = not included / no broadcast)
static int regVal(uint16_t reg){
  if (reg < STATUS_BASE) return -1;
  size_t off = (size_t)(reg - STATUS_BASE) * 2;
  if (off + 1 >= g_statusLen) return -1;
  return (g_status[off] << 8) | g_status[off+1];
}

// Status output from the central OvenState (filled by poll reads AND ##-broadcast).
static void printStatus(){
  Serial.printf("---- Oven status (BLE=%s, seq=%u, age=%lus) ----\n",
                g_oven.bleOnline?"online":"offline", (unsigned)g_oven.seq,
                g_oven.lastUpdateMs? (unsigned long)((millis()-g_oven.lastUpdateMs)/1000) : 0UL);
  if (g_caps.detected)          Serial.printf("  Type             = %s, fans=%d, fanLevels=%d%s%s%s (banca_dati '%s')\n",
                                              g_caps.hydro?"HYDRO":"AIR", g_caps.fanCount, g_caps.fanLevels,
                                              g_caps.boiler?", boiler":"", g_caps.puffer?", puffer":"",
                                              g_caps.clima?", clima":"", g_caps.bancaDati.c_str());
  if (!isnan(g_oven.roomC))     Serial.printf("  Room temp        = %.1f C\n", g_oven.roomC);
  if (!isnan(g_oven.setpointC)) Serial.printf("  Setpoint         = %.1f C\n", g_oven.setpointC);
  if (g_caps.hydro && !isnan(g_oven.boilerC)) Serial.printf("  Boiler temp      = %.1f C\n", g_oven.boilerC);
  if (g_caps.puffer && !isnan(g_oven.pufferC)) Serial.printf("  Puffer temp      = %.1f C\n", g_oven.pufferC);
  if (g_caps.hydro && !isnan(g_oven.hystAmbNeg)) Serial.printf("  Hyst amb -/+     = %.1f / %.1f C\n", g_oven.hystAmbNeg, g_oven.hystAmbPos);
  if (g_caps.hydro && !isnan(g_oven.hystSSNeg))  Serial.printf("  Hyst Start/Stop  = %.1f / %.1f C\n", g_oven.hystSSNeg, g_oven.hystSSPos);
  if (g_caps.hydro && !isnan(g_oven.pumpMinOn))  Serial.printf("  Pump min-on temp = %.1f C\n", g_oven.pumpMinOn);
  if (g_caps.puffer && !isnan(g_oven.pufferSet)) Serial.printf("  Water setpoint   = %.1f C\n", g_oven.pufferSet);
  if (!isnan(g_oven.boardC))    Serial.printf("  Control board T  = %.1f C\n", g_oven.boardC);
  if (!isnan(g_oven.fumesC))    Serial.printf("  Fumes temp       = %.1f C\n", g_oven.fumesC);
  if (g_oven.power>=0)          Serial.printf("  Power            = %d\n", g_oven.power);
  if (g_oven.mode>=0)           Serial.printf("  Mode             = %d (%s)\n", g_oven.mode, modeName(g_oven.mode));
  if (g_oven.state>=0){ const char* sn=stateName((int)g_oven.state);
                                Serial.printf("  Phase 0x0320     = 0x%04X (%s)\n", (unsigned)g_oven.state, sn[0]?sn:"unknown"); }
  else if (g_oven.phase>=0)     Serial.printf("  Phase            = %d (%s)\n", g_oven.phase, g_oven.phase==3?"On":g_oven.phase==1?"Off":"?");
  if (g_oven.fanLevel>=0)       Serial.printf("  Fan level        = %d\n", g_oven.fanLevel);
  if (g_oven.fanRoom>=0)        Serial.printf("  Fumes fan        = %ld rpm\n", (long)g_oven.fanRoom);
  if (g_oven.fanComb>=0)        Serial.printf("  Combustion fan   = %ld rpm\n", (long)g_oven.fanComb);
  if (g_oven.active>=0)         Serial.printf("  Active           = %ld\n", (long)g_oven.active);
  if (g_oven.flags>=0)          Serial.printf("  Flags 0x0332     = 0x%02X (Chrono=%d Silent=%d)\n",
                                              g_oven.flags, (g_oven.flags>>6)&1, (g_oven.flags>>5)&1);
  if (g_oven.alarm>=0){ Serial.print("  Active alarm     = ");
    if(g_oven.alarm>0) Serial.printf("A%d  ('ack' to acknowledge)\n", g_oven.alarm); else Serial.println("none"); }
  if (g_oven.alarmHist[0]>=0){  Serial.print("  Last alarm       = ");
    if(g_oven.alarmHist[0]>0) Serial.printf("A%d\n", g_oven.alarmHist[0]); else Serial.println("none");
    Serial.print("  Alarm history    =");
    for(int k=0;k<10 && g_oven.alarmHist[k]>=0;k++) Serial.printf(" A%d", g_oven.alarmHist[k]);
    Serial.println(); }
  if (g_oven.ignitions>=0)      Serial.printf("  Ignitions        = %ld\n", (long)g_oven.ignitions);
  if (g_oven.worktimeMin>=0)    Serial.printf("  Total work time  = %ld min\n", (long)g_oven.worktimeMin);
  for (int i=0;i<5;i++) if (g_oven.powerTimeMin[i]>=0)
    Serial.printf("  Time power %d     = %ld min\n", i+1, (long)g_oven.powerTimeMin[i]);
}

// ---- Notify (abf2): decrypt + parse Modbus ---------------------------------
static void notifyCB(NimBLERemoteCharacteristic*, uint8_t *data, size_t len, bool) {
  // Module broadcast on abf2: "##"-framed (0x23 0x23 [type] [frag]). Non-Modbus,
  // often fragmented (type=02, frag=01/02), length NEVER divisible by 16 -> safely
  // distinguishable from an AES-Modbus frame (always a multiple of 16). This is the
  // module's status/provisioning push; it is acknowledged but not (yet) evaluated.
  // Important: intercept BEFORE the AES check, otherwise misread as broken Modbus.
  if (len>=4 && data[0]==0x23 && data[1]==0x23 && (len%16)) {
    uint8_t frag = data[3]; size_t pl = len - 4;
    if (frag==0x01){ g_bcLen=0; g_bcHave01=true; }           // new broadcast begins
    else if (frag!=0x02){
      Serial.printf("[##] type=%02X frag=%02X len=%u (unknown, ignored)\n",
                    data[2], frag, (unsigned)len); return;
    }
    if (!g_bcHave01) return;                                  // frag02 without frag01 -> discard
    if (g_bcLen + pl > sizeof(g_bcBuf)){ g_bcHave01=false; Serial.println("[##] buffer full"); return; }
    memcpy(g_bcBuf + g_bcLen, data+4, pl); g_bcLen += pl;
    if (frag != 0x02) return;                                 // frag01 buffered, wait for frag02
    g_bcHave01 = false;
    if (g_bcLen % 16){ Serial.printf("[##] reassembled %u B (not /16)\n",(unsigned)g_bcLen); return; }
    static uint8_t pt[1024];
    if (g_bcLen > sizeof(pt)) return;
    aesDec(g_bcBuf, pt, g_bcLen);
    if (memcmp(pt+4, TOKEN, 16) != 0){ Serial.println("[##] Token mismatch -> discarded"); return; }
    g_oven.lastRxMs = millis();                               // valid broadcast (indicator)
    uint8_t pad = pt[g_bcLen-1]; size_t end = (pad>=1 && pad<=16) ? g_bcLen-pad : g_bcLen;
    if (end < 22) return;
    size_t plen = end - 20;                                   // after [4 counter][16 token]
    if (plen > sizeof(g_status)) plen = sizeof(g_status);
    memcpy(g_status, pt+20, plen); g_statusLen = plen;
    g_statusCtr = pt[0]|(pt[1]<<8)|(pt[2]<<16)|((uint32_t)pt[3]<<24);
    for (size_t i=0; i+1 < plen; i+=2)                         // whole image -> OvenState
      ovenApplyReg(STATUS_BASE + (uint16_t)(i/2), (g_status[i]<<8)|g_status[i+1]);
    int rt=regVal(0x02BC), ph=regVal(0x0322);
    Serial.printf("[##] Status ctr=%08X regs=%u Room=%.1fC Phase=%s ('status' for details)\n",
                  g_statusCtr, (unsigned)(g_statusLen/2), rt/10.0, ph==3?"Running":ph==1?"Off":"?");
    return;
  }
  if (len%16 || len<32) {
    Serial.printf("[abf2] raw %u B: ",(unsigned)len);
    for(size_t i=0;i<len;i++) Serial.printf("%02X",data[i]); Serial.println();
    Serial.println("       (not a full AES frame -> MTU too small? see 'MTU 517' on connect)");
    return;
  }
  uint8_t pt[256]; if (len>sizeof(pt)) return;
  aesDec(data, pt, len);
  uint32_t ctr = pt[0]|(pt[1]<<8)|(pt[2]<<16)|((uint32_t)pt[3]<<24);
  // pt[4..19] = token (for verification), pt[20..] = Modbus + PKCS#7
  uint8_t pad = pt[len-1]; size_t bodyEnd = (pad>=1 && pad<=16) ? len-pad : len;
  if (bodyEnd < 24) { Serial.println("[abf2] too short"); return; }
  const uint8_t *mb = pt+20; size_t mbLen = bodyEnd-20;
  uint16_t crcRx = mb[mbLen-2] | (mb[mbLen-1]<<8);
  bool crcOk = (modbusCRC(mb, mbLen-2) == crcRx);
  if (!crcOk){ Serial.printf("[abf2] ctr=%08X crc=BAD (discarded)\n", ctr); return; }
  g_oven.lastRxMs = millis();                                 // valid BLE response (indicator)
  if (g_logRaw){                                               // 'log off' silences the auto-poll
    Serial.printf("[abf2] ctr=%08X crc=OK  modbus: ", ctr);
    for (size_t i=0;i<mbLen;i++) Serial.printf("%02X ", mb[i]); Serial.println();
  }
  // Function 03 response: [01][03][bytecount][data...][crc]
  if (mbLen>=5 && mb[1]==0x03) {
    uint8_t bc = mb[2];
    if (g_logRaw) Serial.printf("       %u data bytes (%u registers 16bit): ", bc, bc/2);
    for (uint8_t i=0;i+1<bc;i+=2) {
      uint16_t r = (mb[3+i]<<8)|mb[3+i+1];
      if (g_logRaw) Serial.printf("%u ", r);
      ovenApplyReg(g_lastReadBase + (uint16_t)(i/2), r);       // read response -> OvenState
    }
    if (g_logRaw) Serial.println();
    if (g_rawCapture && g_lastReadBase==g_rawBase && g_rawDst){  // synchronous block read (bleReadRegs)
      int n=bc/2; for(int i=0;i<n;i++) g_rawDst[i]=(mb[3+2*i]<<8)|mb[3+2*i+1];
      g_rawGot=n;
    }
    if (g_lastReadBase==0x0ADC){                                // serial number (ASCII) + raw diagnostics
      String hexs, asc;
      for(uint8_t i=0;i<bc;i++){ uint8_t b=mb[3+i]; char h[3]; sprintf(h,"%02X",b); hexs+=h;
                                 if(b>=32&&b<127) asc+=(char)b; }
      g_adcHex = hexs; asc.trim();
      if (g_ovenSerial.length()==0 && asc.length()){ g_ovenSerial=asc; Serial.printf(">> Serial number: %s\n", asc.c_str()); }
    }
  } else if (mbLen>=6 && mb[1]==0x06) {
    uint16_t reg=(mb[2]<<8)|mb[3], val=(mb[4]<<8)|mb[5];
    Serial.printf("       WRITE echo reg=0x%04X val=0x%04X (accepted)\n", reg, val);
    ovenApplyReg(reg, val);                                    // WRITE echo confirmed -> apply
  }
}

// Oven scan list for the system menu: collect visible MCZ_EP devices WITHOUT auto-connect.
struct OvenScanItem { String mac; String name; };
static const int    SCANLIST_MAX = 12;
static OvenScanItem g_scanItems[SCANLIST_MAX];
static int          g_scanCount = 0;
static bool         g_scanListMode = false;
static bool         g_serialScan = false;         // print scan list when a serial 'scan' finishes

// ---- BLE boilerplate (NimBLE) ---------------------------------------------
class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    bool isMcz = (dev->getName().rfind(TARGET_PREFIX, 0)==0);
    if (g_scanListMode){                       // collect mode: only list, don't connect
      if(!isMcz) return;
      String raw = dev->getAddress().toString().c_str();
      for(int i=0;i<g_scanCount;i++) if(g_scanItems[i].mac==raw) return;   // dedup
      if(g_scanCount<SCANLIST_MAX){ g_scanItems[g_scanCount].mac=raw;
        g_scanItems[g_scanCount].name=String(dev->getName().c_str()); g_scanCount++; }
      return;
    }
    if (g_target) return;
    String want = g_cfg.targetMac; want.toLowerCase(); want.replace(":","");
    String mac  = dev->getAddress().toString().c_str(); mac.toLowerCase(); mac.replace(":","");
    bool match = want.length() ? (mac == want) : isMcz;   // fixed MAC or first available MCZ_EP
    if (match) {
      Serial.printf(">> Target: %s\n", dev->getAddress().toString().c_str());
      g_target=new NimBLEAdvertisedDevice(*dev); g_doConnect=true; g_scan->stop(); } }
};
static ScanCB g_scanCb;
class CliCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*) override { g_linkUp=true; Serial.println(">> connected."); }
  void onDisconnect(NimBLEClient*) override { g_linkUp=false; g_connected=false; g_abf1=g_abf2=nullptr;
    g_oven.bleOnline=false; Serial.println(">> disconnected."); }
};
static CliCB g_cliCb;
static void setupChars() {
  NimBLERemoteService *s=g_client->getService(NimBLEUUID((uint16_t)0xABF0));
  if(!s){Serial.println("!! 0xABF0 missing");return;}
  g_abf1=s->getCharacteristic(NimBLEUUID((uint16_t)0xABF1));
  g_abf2=s->getCharacteristic(NimBLEUUID((uint16_t)0xABF2));
  Serial.printf("   abf1=%s abf2=%s\n", g_abf1?"ok":"-", g_abf2?"ok":"-");
  if (g_abf2 && g_abf2->canNotify()) { g_abf2->subscribe(true, notifyCB); Serial.println("   abf2 notify subscribed."); }
  g_caps = OvenCaps{};              // fresh capability scan for this (possibly different) oven
  g_alarmDone=false; g_alarmDirty=true; g_alarmScanning=false; g_alarmIndex=-1; g_alarmLivePrev=-1;
  for(int k=0;k<10;k++) g_oven.alarmHist[k]=-1;
  g_oven.bleOnline = true;
  Serial.println("\n>> Ready. 'help' for commands. Start with 'poll' or 'r 02bc 33'.\n");
}
static bool connectTarget() {
  // Remember oven BLE MAC as identity (without ':') -> unique MQTT/HA IDs.
  { String m = g_target->getAddress().toString().c_str(); m.replace(":",""); m.toLowerCase(); g_ovenMac = m; }
  if(g_client){ NimBLEDevice::deleteClient(g_client); g_client=nullptr; }   // never leak a client
  g_client = NimBLEDevice::createClient();
  g_client->setClientCallbacks(&g_cliCb, false);   // false: don't delete callback object
  if(!g_client->connect(g_target)){ Serial.println(">> Connect error");
    NimBLEDevice::deleteClient(g_client); g_client=nullptr; return false; }
  g_linkUp = true;   // NimBLE connect() is synchronous
  setupChars(); g_connected=true; return true;
}
static void setupSecurity() {
  // "Just works" pairing: bonding + Secure Connections, no MITM/IO capability.
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
}

// ---- Serial commands -------------------------------------------------------
// Control registers/limits + command API come from oven.h.
static long hx(const String&s){ return strtol(s.c_str(),nullptr,16); }
static void handleLine(String line){
  line.trim(); if(!line.length()) return; String low=line; low.toLowerCase();
  if(low=="help"){
    Serial.println("temp <c> | power <1-5> | mode <0-4> | fan <auto|1-5> | silent <on|off> | "
                   "on | off | ack | settime | alarms | getchrono | status | poll | scan | target <mac|none> | log <on|off> | r <regHex> <count> | w <regHex> <valHex> | wm <regHex> <v..> | ctr <hex> | help"); return; }
  if(low=="poll"){ sendRead(0x02BC,0x33); return; }
  if(low=="status"){ printStatus(); return; }
  if(low=="on"){ ovenSetOnOff(true); return; }
  if(low=="off"){ ovenSetOnOff(false); return; }
  if(low=="ack"){ ovenResetAlarm(); return; }
  if(low=="silent on"){ ovenSetSilent(true); return; }
  if(low=="silent off"){ ovenSetSilent(false); return; }
  if(low=="settime"){ ovenSetClock(); return; }
  if(low=="getchrono"){ chronoPrint(); return; }
  if(low=="alarms"){ g_alarmDirty=true;               // force a fresh log read + print current
    Serial.print(">> Alarm history (newest first):");
    for(int k=0;k<10 && g_oven.alarmHist[k]>=0;k++) Serial.printf(" A%d", g_oven.alarmHist[k]);
    Serial.printf("  (index=%d, num=%d)\n", g_alarmIndex, g_alarmNum); return; }
  if(low=="log on"){ g_logRaw=true; Serial.println(">> Raw log ON"); return; }
  if(low=="log off"){ g_logRaw=false; Serial.println(">> Raw log OFF"); return; }
  if(low=="scan"){ g_serialScan=true; bleScanListStart();
    Serial.println(">> Scanning ~8s for MCZ_EP ovens..."); return; }
  if(low=="target"){ Serial.printf(">> target = '%s'\n",
    g_cfg.targetMac.length()? g_cfg.targetMac.c_str() : "(nearest MCZ_EP)"); return; }
  if(low.startsWith("target ")){
#if defined(USE_DISPLAY) && (USE_DISPLAY == 1)
    String m=line.substring(7); m.trim();
    if(m=="none"||m=="-"||m=="") m="";
    g_cfg.targetMac=m; configSave();
    Serial.printf(">> target='%s' saved -> restarting...\n", m.length()? m.c_str() : "(nearest)");
    delay(300); ESP.restart();
#else
    Serial.println(">> headless build: target is fixed by config.h TARGET_MAC (edit + reflash). NVS is ignored here.");
#endif
    return; }
  if(low.startsWith("wm ")){                      // wm <regHex> <v0Hex> <v1Hex> ... (Fn 0x10)
    int p=line.indexOf(' '), p2=(p>=0)?line.indexOf(' ',p+1):-1;
    if(p2<0){ Serial.println("wm <reg> <v0> <v1> ..."); return; }
    uint16_t reg=(uint16_t)hx(line.substring(p+1,p2)); uint16_t vals[24]; uint8_t n=0;
    int s=p2+1;
    while(s<(int)line.length() && n<24){ int e=line.indexOf(' ',s); if(e<0)e=line.length();
      String tok=line.substring(s,e); tok.trim(); if(tok.length()) vals[n++]=(uint16_t)hx(tok); s=e+1; }
    if(n) sendWriteMulti(reg,vals,n); return;
  }
  int s1=line.indexOf(' '), s2=(s1>=0)?line.indexOf(' ',s1+1):-1;
  if(low.startsWith("ctr") && s1>=0){ g_counter=(uint32_t)strtoul(line.substring(s1+1).c_str(),nullptr,16);
    Serial.printf("Counter=0x%08X\n",g_counter); return; }
  if(low.startsWith("r ") && s1>=0 && s2>=0){ sendRead((uint16_t)hx(line.substring(s1+1,s2)),(uint16_t)strtol(line.substring(s2+1).c_str(),nullptr,10)); return; }
  if(low.startsWith("w ") && s1>=0 && s2>=0){ sendWrite((uint16_t)hx(line.substring(s1+1,s2)),(uint16_t)hx(line.substring(s2+1))); return; }
  // ---- convenience commands -> central command API (oven.h) ----
  if(low.startsWith("temp ")  && s1>=0){ ovenSetTemp(line.substring(s1+1).toFloat()); return; }
  if(low.startsWith("power ") && s1>=0){ ovenSetPower(line.substring(s1+1).toInt());  return; }
  if(low.startsWith("mode ")  && s1>=0){ ovenSetMode(line.substring(s1+1).toInt());   return; }
  if(low.startsWith("fan ")   && s1>=0){ String a=line.substring(s1+1); a.trim(); a.toLowerCase();
    ovenSetFan(a=="auto"?0:a.toInt()); return; }
  Serial.println("Unknown. 'help'.");
}

// Non-blocking BLE scan (callback form) -> loop()/display stay reactive during the
// scan (the blocking 2-arg form would have frozen setup()/loop() for ~8 s).
static bool g_scanActive = false;
static void scanDoneCB(NimBLEScanResults res){
  g_scanActive = false;
  if (g_scanListMode){
    // Rebuild the oven list from the FINAL results: the MCZ name is only reliably
    // present here (the live onResult adv packet often has no name yet) -> the display
    // picker and serial list actually show the ovens.
    g_scanCount = 0;
    for (int i=0; i<res.getCount() && g_scanCount<SCANLIST_MAX; i++){
      NimBLEAdvertisedDevice d = res.getDevice(i);
      String nm = d.getName().c_str();
      if (nm.indexOf("MCZ") < 0) continue;
      String mac = d.getAddress().toString().c_str();
      bool dup=false; for(int j=0;j<g_scanCount;j++) if(g_scanItems[j].mac==mac){ dup=true; break; }
      if(!dup){ g_scanItems[g_scanCount].mac=mac; g_scanItems[g_scanCount].name=nm; g_scanCount++; }
    }
  }
  if (g_serialScan){
    g_serialScan = false;
    // Diagnostic: list ALL advertised BLE devices (not just MCZ_EP), so an oven that
    // advertises under a different name is still visible. MCZ_EP ones are marked.
    int total = res.getCount();
    Serial.printf(">> Scan done: %d BLE device(s) total, %d named MCZ_EP\n", total, g_scanCount);
    for (int i=0;i<total;i++){
      NimBLEAdvertisedDevice d = res.getDevice(i);
      String nm = d.getName().c_str();
      bool mcz = (nm.indexOf("MCZ")>=0) || (d.getAddress().toString().find("MCZ")!=std::string::npos);
      Serial.printf("   %-17s  rssi=%-4d  name='%s'%s\n",
        d.getAddress().toString().c_str(), d.getRSSI(), nm.c_str(), mcz?"   <-- MCZ":"");
    }
    Serial.println(">> Pick the oven with:  target <mac>   (or 'target none' = nearest MCZ_EP)");
    bleScanListStop();                            // resume normal auto-connect
  }
}
static void startScan(){
  if (g_scanActive) return;
  g_scan->clearResults(); g_scanActive = true;
  g_scan->start(SCAN_SECONDS, scanDoneCB, false);
}
// ---- Oven scan list (ble_api.h) -------------------------------------------
void bleScanListStart(){
  if (g_client && g_connected) g_client->disconnect();
  if (g_target){ delete g_target; g_target=nullptr; }
  g_connected=false; g_scanCount=0; g_scanListMode=true;
  startScan();
}
void bleScanListStop(){ g_scanListMode=false; g_scanCount=0; }   // normal auto-connect afterwards
bool bleScanListBusy(){ return g_scanActive; }
int  bleScanListCount(){ return g_scanCount; }
bool bleScanListGet(int i, String& mac, String& name){
  if (i<0 || i>=g_scanCount) return false;
  mac=g_scanItems[i].mac; name=g_scanItems[i].name; return true;
}
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\n=== MCZ BLE-Modbus-Controller ===");
  configLoad();               // runtime config from NVS (defaults from config.h)
  displayInstance().begin();  // display FIRST -> UI visible immediately (instead of ~8s white)
  NimBLEDevice::init(""); NimBLEDevice::setMTU(517); setupSecurity();
  g_scan=NimBLEDevice::getScan(); g_scan->setAdvertisedDeviceCallbacks(&g_scanCb);
  g_scan->setActiveScan(true); g_scan->setInterval(100); g_scan->setWindow(99);
  netBegin();                 // WiFi + MQTT (no-op with -DUSE_MQTT=0)
  configTzTime(OVEN_TZ, "pool.ntp.org");   // NTP -> oven clock (timezone from config.h)
  startScan();                // non-blocking
  for (int i=0;i<6;i++){ displayInstance().tick(); delay(15); }   // draw UI initially
}
// One-time capability scan right after connect: reads the feature registers so the
// UI/MQTT only offer functions the hardware actually has (Hydro? boiler/puffer? how
// many fans / fan levels?). Runs one read at a time (spaced) BEFORE normal polling,
// so responses are never mis-paired (single outstanding read via g_lastReadBase).
static void capScan(){
  static uint32_t last=0; static uint8_t idx=0;
  if(!g_connected || !g_abf1) return;
  if(g_caps.detected){ idx=0; return; }           // done (reset idx for next connect)
  uint32_t now=millis();
  if(now-last < 700) return;                        // > response latency -> one read outstanding
  last=now;
  switch(idx){
    case 0: sendRead(REG_SET_BOILER, 13); break;    // 0x0407..0x0413: set_boiler, max_pot_idro, set_puffer
    case 1: sendRead(REG_ABI_VEN12,   2); break;    // 0x06F4/0x06F5: fan-enable bytes -> fan count
    case 2: sendRead(REG_VVEN1_LO,    3); break;    // 0x05FD..0x05FF: fan-1 speed table -> fan levels
    case 3: sendRead(REG_TIPO_APP,    1); break;    // device type
    case 4: sendRead(REG_CLIMA,       1); break;    // conditioner block
    case 5: sendRead(REG_BANCADATI,   9); break;    // model database id (ASCII)
    case 6: if(g_caps.hydro) sendRead(REG_HYST_AMB_NEG, 4); break;  // 0x03F3..0x03F6 hysteresis (Hydro only)
    case 7: if(g_caps.hydro) sendRead(REG_PUMP_MIN_ON,  1); break;  // 0x0403 pump min on-temp (Hydro only)
    default:
      g_caps.detected = true;
      Serial.printf(">> Capabilities: %s, fans=%d, fanLevels=%d, boiler=%d, puffer=%d, clima=%d, banca_dati='%s', tipo_app=%u\n",
        g_caps.hydro?"HYDRO":"AIR", g_caps.fanCount, g_caps.fanLevels,
        g_caps.boiler, g_caps.puffer, g_caps.clima, g_caps.bancaDati.c_str(), g_caps.tipoApp);
      return;
  }
  idx++;
}
// Read the alarm-log ring buffer (index + the last-10 code entries) into g_oven.alarmHist.
// Runs once after the capability scan, and again whenever a new live alarm appears
// (g_alarmDirty). While scanning it pauses pollTick so only one read is ever outstanding.
static void alarmScan(){
  static uint32_t last=0; static uint8_t idx=0;
  if(!g_connected || !g_abf1 || !g_caps.detected) return;
  if(g_alarmDone && !g_alarmDirty){ g_alarmScanning=false; return; }
  g_alarmScanning=true;
  uint32_t now=millis(); if(now-last < 700) return; last=now;
  switch(idx){
    case 0: g_alarmIndex=-1; sendRead(REG_ALARM_IDX, 1); break;      // index_allarme / num_allarmi
    case 1: {
      if(g_alarmIndex<0){ idx=0; return; }                           // retry if index not applied yet
      int start=(g_alarmIndex-10+100)%100;                           // oldest of the last 10
      int first=(start<=90)?10:(100-start);                          // entries before the 0-wrap
      sendRead((uint16_t)(0x07EC + 4*start), (uint16_t)(4*first)); break;
    }
    case 2: {
      int start=(g_alarmIndex-10+100)%100;
      if(start<=90) break;                                           // no wrap
      sendRead(0x07EC, (uint16_t)(4*((start+10)-100))); break;       // wrapped part from slot 0
    }
    default: {
      int cnt = g_alarmNum<10 ? g_alarmNum : 10;
      for(int k=0;k<10;k++){
        if(k>=cnt || g_alarmIndex<0){ g_oven.alarmHist[k]=-1; continue; }
        g_oven.alarmHist[k]=(int16_t)g_alarmCode[(g_alarmIndex-1-k+100)%100];
      }
      g_alarmDone=true; g_alarmDirty=false; g_alarmScanning=false; bumpSeq();
      Serial.print(">> Alarm history (newest first):");
      for(int k=0;k<10 && g_oven.alarmHist[k]>=0;k++) Serial.printf(" A%d", g_oven.alarmHist[k]);
      Serial.println();
      idx=0; return;
    }
  }
  idx++;
}
// Non-blocking auto-poll: keeps OvenState fresh. Cyclically one block at a time,
// so only one read is ever outstanding (response has no address -> g_lastReadBase).
static void pollTick(){
  static uint32_t last=0; static uint8_t idx=0;
  if(!g_connected || !g_abf1) return;
  if(!g_caps.detected){ capScan(); return; }        // capability scan first, then poll
  if(g_alarmScanning) return;                        // let the alarm-log read finish (single read)
  uint32_t now=millis();
  if(now-last < POLL_INTERVAL_MS) return;
  last=now;
  // Always poll status blocks; while the serial number is missing, also read 0x0ADC
  // as the 4th step (does NOT block the status blocks -> no starvation if an oven
  // provides no ASCII serial there).
  bool needSerial = (g_ovenSerial.length()==0);
  uint8_t steps = needSerial ? 4 : 3;
  switch(idx % steps){
    case 0: sendRead(0x02BC, 51); break;   // room/control board/fumes temp
    case 1: sendRead(0x0320, 77); break;   // phase, flags, counter, fans
    case 2: sendRead(0x03E9, 15); break;   // mode, power, setpoint
    case 3: sendRead(0x0ADC,  8); break;   // serial number (only while unknown)
  }
  idx = (idx+1) % steps;
}
// Feed display inputs to the command API + render periodically.
static void displayTick(){
  displayInstance().tick();                 // every loop (LVGL handler)
  InputEvent e = displayInstance().poll();
  switch(e.type){
    case InputEvent::SET_TEMP:  ovenSetTemp(e.fval);  break;
    case InputEvent::SET_POWER: ovenSetPower(e.ival);  break;
    case InputEvent::SET_MODE:  ovenSetMode(e.ival);   break;
    case InputEvent::ON:        ovenSetOnOff(true);    break;
    case InputEvent::OFF:       ovenSetOnOff(false);   break;
    default: break;
  }
  static uint32_t last=0; uint32_t now=millis();
  if(now-last > 500){ last=now; displayInstance().render(g_oven); }  // ~2 Hz
}
// Auto-sync the oven RTC from NTP: once shortly after connect+NTP is ready, then daily.
static void clockTick(){
  static uint32_t lastSet=0; static bool done=false;
  if(!g_connected || !g_abf1 || !g_caps.detected) return;   // after capability scan settles
  uint32_t now=millis();
  if(done && (now-lastSet) < 24UL*60*60*1000) return;       // re-sync every 24h (RTC drifts)
  struct tm t;
  if(!getLocalTime(&t, 5)) return;                          // NTP not synced yet -> retry later
  if(t.tm_year+1900 < 2024) return;
  if(ovenSetClock()){ lastSet=now; done=true; }
}
// Periodic diagnostic so instability (heap leak / WiFi drop / reboot) is visible in the log.
static void heartbeat(){
  static uint32_t last=0; uint32_t now=millis();
  if(now-last < 60000) return; last=now;
  Serial.printf("[hb] up=%lus heap=%u min=%u ble=%d wifi=%d rssi=%d mqtt=%d\n",
    (unsigned long)(now/1000), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
    g_connected?1:0, netWifiUp()?1:0, netWifiUp()?(int)WiFi.RSSI():0, netMqttUp()?1:0);
}
void loop(){
  static String line;
  while(Serial.available()){ char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){ if(line.length()){handleLine(line);line="";} } else line+=c; }
  if(g_doConnect){ g_doConnect=false;
    if(!connectTarget()){ delete g_target; g_target=nullptr; startScan(); } return; }
  // BLE link dropped: free the stale client/target so a fresh scan+reconnect can start
  // (otherwise g_target stays set and the rescan below never runs -> stuck offline).
  if(!g_connected && g_client){
    NimBLEDevice::deleteClient(g_client); g_client=nullptr;
    if(g_target){ delete g_target; g_target=nullptr; }
    g_abf1=g_abf2=nullptr;
    Serial.println(">> BLE cleanup after disconnect -> rescanning");
  }
  if(!g_connected && !g_target && !g_scanActive){ startScan(); }
  heartbeat();
  alarmScan();      // read alarm-log history (pauses pollTick while reading)
  pollTick();
  netTick();
  clockTick();      // auto-sync oven RTC from NTP (via Fn 0x10; once after connect, then daily)
  displayTick();
  delay(20);
}
