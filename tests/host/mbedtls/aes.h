// Host-only shim: the subset of mbedtls/aes.h used by mcz_proto.h, on OpenSSL.
#pragma once
#include <openssl/evp.h>
#include <cstring>

#define MBEDTLS_AES_ENCRYPT 1
#define MBEDTLS_AES_DECRYPT 0

struct mbedtls_aes_context { unsigned char key[16]; };
inline void mbedtls_aes_init(mbedtls_aes_context *) {}
inline void mbedtls_aes_free(mbedtls_aes_context *) {}
inline int mbedtls_aes_setkey_enc(mbedtls_aes_context *c, const unsigned char *k, unsigned) { memcpy(c->key, k, 16); return 0; }
inline int mbedtls_aes_setkey_dec(mbedtls_aes_context *c, const unsigned char *k, unsigned) { memcpy(c->key, k, 16); return 0; }
inline int mbedtls_aes_crypt_cbc(mbedtls_aes_context *c, int mode, size_t len, unsigned char iv[16],
                                 const unsigned char *in, unsigned char *out) {
  EVP_CIPHER_CTX *x = EVP_CIPHER_CTX_new();
  EVP_CipherInit_ex(x, EVP_aes_128_cbc(), nullptr, c->key, iv, mode);
  EVP_CIPHER_CTX_set_padding(x, 0);
  int n = 0, m = 0;
  EVP_CipherUpdate(x, out, &n, in, (int) len);
  EVP_CipherFinal_ex(x, out + n, &m);
  EVP_CIPHER_CTX_free(x);
  return 0;
}
