/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief On-device TLS private key and CSR generation API.
 */

#ifndef ZEPHYR_INCLUDE_NET_TLS_CREDENTIALS_KEYGEN_H_
#define ZEPHYR_INCLUDE_NET_TLS_CREDENTIALS_KEYGEN_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/net/tls_credentials.h>
#include <psa/crypto.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief TLS credentials on-device key generation
 * @defgroup tls_credentials_keygen TLS credentials on-device key generation
 * @since 4.5
 * @version 0.1.0
 * @ingroup tls_credentials
 * @{
 */

/**
 * @brief Generate a device private key for a security tag.
 *
 * Creates a persistent, non-exportable ECC P-256 key pair in PSA key storage and registers it
 * with the TLS credentials module as an opaque key (@ref TLS_CREDENTIAL_PRIVATE_KEY_PSA).
 * The private key never leaves the device. The PSA key ID is derived from the security tag.
 *
 * @note Only a single security tag can hold an on-device key registration at a time.
 *       Delete the existing key to use a different security tag.
 *
 * @param sec_tag Security tag the key is associated with.
 *
 * @retval 0 On success.
 * @retval -EINVAL The security tag does not map to a valid PSA key ID.
 * @retval -EALREADY A key already exists for the security tag.
 * @retval -ENOTSUP A key is already registered for a different security tag.
 * @retval -EIO PSA operation failed.
 * @return Other negative errno from @ref tls_credential_add.
 */
int tls_credentials_key_generate(sec_tag_t sec_tag);

/**
 * @brief Delete the device private key of a security tag.
 *
 * Destroys the PSA key and removes its TLS credential registration.
 *
 * @param sec_tag Security tag the key is associated with.
 *
 * @retval 0 On success. Deleting a key that does not exist is not an error.
 * @retval -EINVAL The security tag does not map to a valid PSA key ID.
 * @retval -EIO PSA operation failed.
 */
int tls_credentials_key_delete(sec_tag_t sec_tag);

/**
 * @brief Register an existing device private key with the TLS credentials module.
 *
 * The registration is not persistent, so call this after every boot, before the key is used
 * in a TLS connection. Registering a key that is already registered is not an error.
 *
 * @param sec_tag Security tag the key is associated with.
 *
 * @retval 0 On success.
 * @retval -ENOENT No key exists for the security tag.
 * @retval -ENOTSUP A key is already registered for a different security tag.
 * @return Other negative errno from @ref tls_credential_add.
 */
int tls_credentials_key_restore(sec_tag_t sec_tag);

/**
 * @brief Check whether a device private key exists in PSA key storage.
 *
 * PSA key storage is the source of truth, so the result does not depend on the key being
 * registered with the TLS credentials module.
 *
 * @param sec_tag Security tag the key is associated with.
 *
 * @retval true A key exists for the security tag.
 * @retval false Otherwise.
 */
bool tls_credentials_key_exists(sec_tag_t sec_tag);

/**
 * @brief Get the PSA key ID that holds the device private key of a security tag.
 *
 * @param sec_tag Security tag the key is associated with.
 *
 * @return The PSA key ID.
 */
psa_key_id_t tls_credentials_key_id_get(sec_tag_t sec_tag);

/**
 * @brief Generate a Certificate Signing Request (CSR) for the device private key.
 *
 * The CSR is signed by the on-device key through PSA and written in DER format.
 *
 * @param sec_tag  Security tag whose key signs the CSR.
 * @param subject  X.509 subject name, for example "CN=my-device,O=Nordic".
 * @param out      Buffer for the DER-encoded CSR.
 * @param out_size Size of @p out in bytes.
 * @param out_len  Number of bytes written to @p out.
 *
 * @retval 0 On success.
 * @retval -EINVAL Invalid argument or subject name.
 * @retval -EIO PSA or mbedTLS operation failed.
 */
int tls_credentials_csr_generate(sec_tag_t sec_tag, const char *subject, uint8_t *out,
				 size_t out_size, size_t *out_len);

/**
 * @brief Export the public key of the device key pair.
 *
 * The key is exported in SEC1 uncompressed point format (0x04 || X || Y), which is 65 bytes
 * for ECC P-256. Host tools can compare it with the public key of the installed device
 * certificate. Requires @kconfig{CONFIG_TLS_CREDENTIALS_KEYGEN_VERIFY}.
 *
 * @param sec_tag  Security tag whose key is exported.
 * @param out      Buffer for the public key.
 * @param out_size Size of @p out in bytes.
 * @param out_len  Number of bytes written to @p out.
 *
 * @retval 0 On success.
 * @retval -ENOTSUP @kconfig{CONFIG_TLS_CREDENTIALS_KEYGEN_VERIFY} is disabled.
 * @retval -EINVAL Invalid argument.
 * @retval -EIO PSA operation failed.
 */
int tls_credentials_pubkey_get(sec_tag_t sec_tag, uint8_t *out, size_t out_size, size_t *out_len);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_NET_TLS_CREDENTIALS_KEYGEN_H_ */
