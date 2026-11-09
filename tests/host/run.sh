#!/bin/sh
# Host test of the shared protocol core. Needs g++, OpenSSL headers, python3-cryptography.
set -e
cd "$(dirname "$0")"
out=$(mktemp -d)
python3 -I gen_vectors.py > "$out/vectors.h"
g++ -std=gnu++17 -Wall -Wextra -I. -I"$out" -I../../components/mcz_maestro_ble test_proto.cpp -o "$out/test_proto" -lcrypto
"$out/test_proto"
rm -rf "$out"
