// Host test of mcz_proto.h against vectors from gen_vectors.py.
#include <cstdio>
#include "mcz_proto.h"
#include "vectors.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

int main() {
  // build_frame == independent implementation
  uint8_t pdu[8], ct[mcz::MAX_FRAME];
  size_t n = mcz::build_frame(mcz::COUNTER_START, pdu, mcz::pdu_read(pdu, 0x02BC, 51), ct);
  CHECK(n == sizeof(V_READ_REQ));
  CHECK(memcmp(ct, V_READ_REQ, n) == 0);

  // decode a fn03 reply
  uint8_t pt[256];
  const uint8_t *mb; size_t mb_len; uint32_t ctr;
  CHECK(mcz::decode_modbus(V_READ_RESP, sizeof(V_READ_RESP), pt, &mb, &mb_len, &ctr));
  CHECK(ctr == 0x11223344);
  CHECK(mb[1] == 0x03 && mb[2] == 6);
  CHECK(((mb[3] << 8) | mb[4]) == 215 && ((mb[5] << 8) | mb[6]) == 0x0202 && ((mb[7] << 8) | mb[8]) == 0xFFFF);

  // a corrupted frame must fail the CRC check
  uint8_t bad[sizeof(V_READ_RESP)]; memcpy(bad, V_READ_RESP, sizeof(bad)); bad[20] ^= 1;
  CHECK(!mcz::decode_modbus(bad, sizeof(bad), pt, &mb, &mb_len, &ctr));

  // broadcast reassembly
  mcz::BroadcastAssembler a;
  CHECK(mcz::is_broadcast(V_BC1, sizeof(V_BC1)) && mcz::is_broadcast(V_BC2, sizeof(V_BC2)));
  CHECK(a.feed(V_BC2, sizeof(V_BC2)) == mcz::BroadcastAssembler::DROPPED);  // frag 02 alone
  CHECK(a.feed(V_BC1, sizeof(V_BC1)) == mcz::BroadcastAssembler::PENDING);
  CHECK(a.feed(V_BC2, sizeof(V_BC2)) == mcz::BroadcastAssembler::COMPLETE);
  CHECK(a.counter() == 0x55667788);
  CHECK(a.image_len() == 400);
  const uint8_t *img = a.image();
  CHECK(((img[0] << 8) | img[1]) == 0x1000 && ((img[398] << 8) | img[399]) == 0x1000 + 199);

  // write-multi bounds
  uint16_t v[25] = {0};
  uint8_t big[mcz::MAX_PDU + 8];
  CHECK(mcz::pdu_write_multi(big, 0x0384, v, 25) == 0);
  CHECK(mcz::pdu_write_multi(big, 0x0384, v, 24) == mcz::MAX_PDU);
  CHECK(mcz::build_frame(1, big, mcz::MAX_PDU, ct) == mcz::MAX_FRAME);

  printf(fails ? "%d FAILED\n" : "all ok\n", fails);
  return fails ? 1 : 0;
}
