// mcz_proto.h — framework-neutral MCZ Maestro+ BLE protocol core.
//
// Shared by the PlatformIO/Arduino firmware (src/) and the ESPHome component in
// this folder. Plain C++ + mbedtls only: no Arduino String/Serial, no ESPHome
// headers, so both builds use one implementation of the wire protocol.
//
// Wire format (see src/main.cpp header for the full description):
//   write to 0xABF1, responses as notify on 0xABF2.
//   frame = AES-128-CBC(KEY1, IV0) of
//           [4 byte counter LE][16 byte token][Modbus RTU PDU + CRC16][PKCS#7]
//   "##"-framed notifies (0x23 0x23 type frag, length not a multiple of 16) are
//   the module's fragmented status broadcast: an encrypted register image
//   starting at STATUS_BASE.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "mbedtls/aes.h"

namespace mcz {

// ---- GATT -------------------------------------------------------------------
static const uint16_t SERVICE_UUID = 0xABF0;
static const uint16_t CHAR_WRITE_UUID = 0xABF1;
static const uint16_t CHAR_NOTIFY_UUID = 0xABF2;
static const char *const NAME_PREFIX = "MCZ_EP";
static const uint16_t MTU = 517;

// ---- Crypto constants (firmware-verified, shared across this firmware generation)
static const uint8_t KEY1[16] = {0x6e, 0x29, 0x6b, 0x0b, 0xbb, 0x1d, 0x43, 0xf3,
                                 0x6e, 0x47, 0xf7, 0x2e, 0x7b, 0x6f, 0x2e, 0x77};
static const uint8_t IV0[16] = {0xda, 0x1a, 0x55, 0x73, 0x49, 0xf2, 0x5c, 0x64,
                                0x1b, 0x1a, 0x36, 0x8a, 0xf5, 0xb2, 0x18, 0xa7};
static const uint8_t TOKEN[16] = {0x31, 0xdd, 0x34, 0x51, 0x26, 0x39, 0x37, 0x7b,
                                  0x05, 0xa2, 0x51, 0x0d, 0xe7, 0x25, 0xfc, 0x75};
static const uint32_t COUNTER_START = 0x6A418000;

// Largest frame build_frame() can produce: 20 header + 7+48 PDU (write-multi, 24 regs)
// + 2 CRC = 77 -> padded to 80.
static const size_t MAX_FRAME = 80;
static const size_t MAX_PDU = 7 + 48;

// ---- Register map -----------------------------------------------------------
namespace regs {
// Control (write, function 06)
static const uint16_t REG_SETPOINT = 0x03F7;     // setpoint, value = °C * 10
static const uint16_t REG_POWER = 0x03EB;        // power 1..5
static const uint16_t REG_MODE = 0x03E9;         // 0=Manual,1=Auto,2=Overnight (verified), 3=Comfort,4=Turbo (assumed)
static const uint16_t REG_ONOFF = 0x038A;        // power TOGGLE: write 1
static const uint16_t REG_ALARM_RESET = 0x038B;  // write 1 = acknowledge alarm
// Date/time block (binary, full year, weekday 0=Monday)
static const uint16_t REG_DT_DAYMON = 0x0384;   // hi=month lo=day
static const uint16_t REG_DT_YEAR = 0x0385;     // full year
static const uint16_t REG_DT_HRMIN = 0x0386;    // hi=minute lo=hour
static const uint16_t REG_DT_SECWD = 0x0387;    // hi=weekday lo=second
static const uint16_t REG_DT_TRIGGER = 0x0388;  // write 0x0100 to commit
// Status (read)
static const uint16_t REG_ROOM = 0x02BC;       // ÷10
static const uint16_t REG_BOILER_T = 0x02BF;   // ÷10 (hydro)
static const uint16_t REG_PUFFER_T = 0x02C0;   // ÷10 (hydro)
static const uint16_t REG_BOARD = 0x02C1;      // ÷10
static const uint16_t REG_FUMES = 0x02C5;      // ÷10
static const uint16_t REG_STATE = 0x0320;      // fine phase code, see state_name()
static const uint16_t REG_PHASE = 0x0322;      // coarse: 1=Off/Standby, 3=On
static const uint16_t REG_MODE_LIVE = 0x032E;  // mode mirror
static const uint16_t REG_ALARM = 0x0323;      // low byte = live alarm code
static const uint16_t REG_ALARM_IDX = 0x07EA;  // hi=index_allarme lo=num_allarmi
static const uint16_t REG_FLAGS = 0x0332;      // bit6=Chrono bit5=Silent
static const uint16_t REG_IGNIT = 0x0334;      // ignition counter
static const uint16_t REG_WORK_LO = 0x0340;    // total working time, 32-bit seconds
static const uint16_t REG_WORK_HI = 0x0341;
static const uint16_t REG_PTIME_LO = 0x0336;   // time in power 1..5: 5x 32-bit seconds
static const uint16_t REG_PTIME_HI = 0x033F;
static const uint16_t REG_ACTIVE = 0x02C9;     // app value "active"
static const uint16_t REG_FAN_COMB = 0x02CE;   // combustion fan RPM
static const uint16_t REG_FAN_ROOM = 0x02D1;   // flue gas/exhaust fan RPM
static const uint16_t REG_FAN_SET = 0x03FA;    // write 1..5 = level, 6 = auto
static const uint16_t REG_FAN_LIVE = 0x0324;   // live fan level
static const uint16_t REG_SILENT = 0x03EC;     // write 1/0
// Hydro settings (°C ÷10)
static const uint16_t REG_HYST_AMB_NEG = 0x03F3;
static const uint16_t REG_HYST_AMB_POS = 0x03F4;
static const uint16_t REG_HYST_SS_NEG = 0x03F5;
static const uint16_t REG_HYST_SS_POS = 0x03F6;
static const uint16_t REG_PUMP_MIN_ON = 0x0403;
// Capability detection (0xFFFF = absent)
static const uint16_t REG_SET_BOILER = 0x0407;
static const uint16_t REG_MAXPOT_IDRO = 0x040F;
static const uint16_t REG_SET_PUFFER = 0x0412;
static const uint16_t REG_VVEN1_LO = 0x05FD;
static const uint16_t REG_VVEN1_HI = 0x05FF;
static const uint16_t REG_TIPO_APP = 0x06A4;
static const uint16_t REG_ABI_VEN12 = 0x06F4;
static const uint16_t REG_ABI_VEN34 = 0x06F5;
static const uint16_t REG_CLIMA = 0x0372;
static const uint16_t REG_BANCADATI = 0x07C6;
static const uint16_t REG_SERIAL = 0x0ADC;   // serial number, ASCII, 8 registers
static const uint16_t SENTINEL16 = 0xFFFF;
// Alarm log ring buffer: 100 slots of 4 registers from 0x07EC, code at 0x07EE + 4N
static const uint16_t REG_ALARM_LOG = 0x07EC;
static const uint16_t REG_ALARM_CODE0 = 0x07EE;
static const uint16_t REG_ALARM_CODE_LAST = 0x097A;
// Base address of the "##" broadcast register image
static const uint16_t STATUS_BASE = 0x02BA;
}  // namespace regs

static const int MODE_COUNT = 5;
inline const char *mode_name(int m) {
  switch (m) {
    case 0: return "Manual";
    case 1: return "Auto";
    case 2: return "Overnight";
    case 3: return "Comfort";
    case 4: return "Turbo";
    default: return "?";
  }
}

// Fine phase from REG_STATE; "" for unknown codes.
inline const char *state_name(int s) {
  switch (s) {
    case 0x0000: return "Off";
    case 0x0101: return "Cleaning";
    case 0x0201: return "Loading";
    case 0x0301: return "Start 1";
    case 0x0401: return "Start 2";
    case 0x0501: return "Stabilization";
    case 0x0601: return "Anti-condensation";
    case 0x0202: return "On";
    case 0x0103: return "Turning off";
    default: return "";
  }
}

// ---- Modbus CRC16 (poly 0xA001) ----------------------------------------------
inline uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

// ---- AES-128-CBC with the fixed key/IV (IV copied, len multiple of 16) -------
inline void aes_encrypt(const uint8_t *in, uint8_t *out, size_t len) {
  mbedtls_aes_context a;
  mbedtls_aes_init(&a);
  mbedtls_aes_setkey_enc(&a, KEY1, 128);
  uint8_t iv[16];
  memcpy(iv, IV0, 16);
  mbedtls_aes_crypt_cbc(&a, MBEDTLS_AES_ENCRYPT, len, iv, in, out);
  mbedtls_aes_free(&a);
}
inline void aes_decrypt(const uint8_t *in, uint8_t *out, size_t len) {
  mbedtls_aes_context a;
  mbedtls_aes_init(&a);
  mbedtls_aes_setkey_dec(&a, KEY1, 128);
  uint8_t iv[16];
  memcpy(iv, IV0, 16);
  mbedtls_aes_crypt_cbc(&a, MBEDTLS_AES_DECRYPT, len, iv, in, out);
  mbedtls_aes_free(&a);
}

// ---- PDU builders (slave 0x01, CRC is added by build_frame) -------------------
inline size_t pdu_read(uint8_t *pdu, uint16_t reg, uint16_t count) {
  const uint8_t p[6] = {0x01, 0x03, (uint8_t) (reg >> 8), (uint8_t) reg, (uint8_t) (count >> 8), (uint8_t) count};
  memcpy(pdu, p, 6);
  return 6;
}
inline size_t pdu_write(uint8_t *pdu, uint16_t reg, uint16_t val) {
  const uint8_t p[6] = {0x01, 0x06, (uint8_t) (reg >> 8), (uint8_t) reg, (uint8_t) (val >> 8), (uint8_t) val};
  memcpy(pdu, p, 6);
  return 6;
}
// Function 0x10, n = 1..24 registers. Returns 0 if n is out of range.
inline size_t pdu_write_multi(uint8_t *pdu, uint16_t reg, const uint16_t *vals, uint8_t n) {
  if (n == 0 || n > 24) return 0;
  pdu[0] = 0x01;
  pdu[1] = 0x10;
  pdu[2] = (uint8_t) (reg >> 8);
  pdu[3] = (uint8_t) reg;
  pdu[4] = 0;
  pdu[5] = n;
  pdu[6] = (uint8_t) (n * 2);
  for (uint8_t i = 0; i < n; i++) {
    pdu[7 + 2 * i] = (uint8_t) (vals[i] >> 8);
    pdu[7 + 2 * i + 1] = (uint8_t) (vals[i] & 0xFF);
  }
  return 7 + n * 2;
}

// Encrypt one PDU into out (>= MAX_FRAME bytes). Returns the frame length, 0 on error.
inline size_t build_frame(uint32_t counter, const uint8_t *pdu, size_t pdu_len, uint8_t *out) {
  if (pdu_len > MAX_PDU) return 0;
  uint8_t plain[MAX_FRAME];
  size_t p = 0;
  plain[p++] = counter & 0xFF;
  plain[p++] = (counter >> 8) & 0xFF;
  plain[p++] = (counter >> 16) & 0xFF;
  plain[p++] = (counter >> 24) & 0xFF;
  memcpy(plain + p, TOKEN, 16);
  p += 16;
  memcpy(plain + p, pdu, pdu_len);
  p += pdu_len;
  uint16_t crc = crc16(pdu, pdu_len);
  plain[p++] = crc & 0xFF;
  plain[p++] = crc >> 8;
  size_t pad = 16 - (p % 16);  // PKCS#7: 1..16
  for (size_t i = 0; i < pad; i++) plain[p++] = (uint8_t) pad;
  aes_encrypt(plain, out, p);
  return p;
}

// ---- Notify decoding ---------------------------------------------------------
inline bool is_broadcast(const uint8_t *data, size_t len) {
  return len >= 4 && data[0] == 0x23 && data[1] == 0x23 && (len % 16) != 0;
}

// Decrypt an AES-Modbus notify into pt (>= len bytes) and check the CRC.
// On success *mb points into pt at the Modbus RTU frame (incl. CRC) and
// *mb_len is its length; *counter is the frame counter.
inline bool decode_modbus(const uint8_t *data, size_t len, uint8_t *pt, const uint8_t **mb, size_t *mb_len,
                          uint32_t *counter) {
  if (len % 16 || len < 32) return false;
  aes_decrypt(data, pt, len);
  *counter = pt[0] | (pt[1] << 8) | (pt[2] << 16) | ((uint32_t) pt[3] << 24);
  uint8_t pad = pt[len - 1];
  size_t end = (pad >= 1 && pad <= 16) ? len - pad : len;
  if (end < 24) return false;
  *mb = pt + 20;
  *mb_len = end - 20;
  uint16_t crc_rx = (*mb)[*mb_len - 2] | ((*mb)[*mb_len - 1] << 8);
  return crc16(*mb, *mb_len - 2) == crc_rx;
}

// Reassembles the fragmented "##" status broadcast (frag 01 + frag 02).
class BroadcastAssembler {
 public:
  enum Result { PENDING, COMPLETE, DROPPED };

  // Feed one "##" notify. On COMPLETE, image()/image_len() hold the decrypted
  // big-endian register image starting at regs::STATUS_BASE.
  Result feed(const uint8_t *data, size_t len) {
    if (len < 4) return DROPPED;
    uint8_t frag = data[3];
    size_t pl = len - 4;
    if (frag == 0x01) {
      len_ = 0;
      have01_ = true;
    } else if (frag != 0x02) {
      return DROPPED;
    }
    if (!have01_) return DROPPED;
    if (len_ + pl > sizeof(buf_)) {
      have01_ = false;
      return DROPPED;
    }
    memcpy(buf_ + len_, data + 4, pl);
    len_ += pl;
    if (frag != 0x02) return PENDING;
    have01_ = false;
    if (len_ % 16) return DROPPED;
    aes_decrypt(buf_, pt_, len_);
    if (memcmp(pt_ + 4, TOKEN, 16) != 0) return DROPPED;
    uint8_t pad = pt_[len_ - 1];
    size_t end = (pad >= 1 && pad <= 16) ? len_ - pad : len_;
    if (end < 22) return DROPPED;
    counter_ = pt_[0] | (pt_[1] << 8) | (pt_[2] << 16) | ((uint32_t) pt_[3] << 24);
    image_len_ = end - 20;
    return COMPLETE;
  }
  const uint8_t *image() const { return pt_ + 20; }
  size_t image_len() const { return image_len_; }
  uint32_t counter() const { return counter_; }

 private:
  uint8_t buf_[1024];
  uint8_t pt_[1024];
  size_t len_{0};
  size_t image_len_{0};
  uint32_t counter_{0};
  bool have01_{false};
};

}  // namespace mcz
