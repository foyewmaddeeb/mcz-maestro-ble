"""Independent Python implementation of the MCZ BLE framing -> vectors.h.

Written from the protocol description (AES-128-CBC, counter, token, Modbus RTU,
PKCS#7, "##" broadcast), not from mcz_proto.h, so the C++ test cross-checks it.
"""
import sys
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

KEY = bytes.fromhex("6e296b0bbb1d43f36e47f72e7b6f2e77")
IV = bytes.fromhex("da1a557349f25c641b1a368af5b218a7")
TOKEN = bytes.fromhex("31dd34512639377b05a2510de725fc75")


def crc16(b):
    crc = 0xFFFF
    for x in b:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def enc(pt):
    e = Cipher(algorithms.AES(KEY), modes.CBC(IV)).encryptor()
    return e.update(pt) + e.finalize()


def frame(counter, pdu):
    body = counter.to_bytes(4, "little") + TOKEN + pdu + crc16(pdu).to_bytes(2, "little")
    pad = 16 - len(body) % 16
    return enc(body + bytes([pad]) * pad)


def arr(name, b):
    return f"static const uint8_t {name}[] = {{{', '.join(f'0x{x:02x}' for x in b)}}};\n"


out = []
# 1. read request 0x02BC x51 with the start counter
out.append(arr("V_READ_REQ", frame(0x6A418000, bytes([1, 3, 0x02, 0xBC, 0, 51]))))
# 2. fn03 reply: 3 registers 215, 0x0202, 0xFFFF
regs = [215, 0x0202, 0xFFFF]
data = b"".join(r.to_bytes(2, "big") for r in regs)
out.append(arr("V_READ_RESP", frame(0x11223344, bytes([1, 3, len(data)]) + data)))
# 3. "##" broadcast: 200 registers from 0x02BA, value = 0x1000 + index
image = b"".join((0x1000 + i).to_bytes(2, "big") for i in range(200))
body = (0x55667788).to_bytes(4, "little") + TOKEN + image
pad = 16 - len(body) % 16
ct = enc(body + bytes([pad]) * pad)
cut = 237  # fragment payloads are not multiples of 16 -> each notify len % 16 != 0
f1 = b"##\x02\x01" + ct[:cut]
f2 = b"##\x02\x02" + ct[cut:]
assert len(f1) % 16 and len(f2) % 16, "pick another cut"
out.append(arr("V_BC1", f1) + arr("V_BC2", f2))
sys.stdout.write("#pragma once\n#include <cstdint>\n" + "".join(out))
