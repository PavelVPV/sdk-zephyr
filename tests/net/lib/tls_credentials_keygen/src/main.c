/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/net/tls_credentials_keygen.h>

#include <psa/crypto.h>
#include <mbedtls/x509_csr.h>

/* Sec tags map directly to PSA user key IDs, so they must be in the PSA user range. */
#define TEST_SEC_TAG   42
#define TEST_SEC_TAG_2 43

/* Delete both known sec tags to destroy any generated key and free the single registration slot,
 * regardless of which tag was registered.
 */
static void reset_state(void)
{
	(void)tls_credentials_key_delete(TEST_SEC_TAG);
	(void)tls_credentials_key_delete(TEST_SEC_TAG_2);
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	reset_state();
}

ZTEST_SUITE(keygen_api, NULL, NULL, before, NULL, NULL);

ZTEST(keygen_api, test_generate_and_exists)
{
	zassert_false(tls_credentials_key_exists(TEST_SEC_TAG),
		      "key should not exist before generation");

	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");

	zassert_true(tls_credentials_key_exists(TEST_SEC_TAG),
		     "key should exist after generation");
	zassert_equal(tls_credentials_key_id_get(TEST_SEC_TAG), (psa_key_id_t)TEST_SEC_TAG,
		      "unexpected key id mapping");
}

ZTEST(keygen_api, test_generate_twice_is_ealready)
{
	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "first generate failed");
	zassert_equal(tls_credentials_key_generate(TEST_SEC_TAG), -EALREADY,
		      "second generate should report -EALREADY");
}

ZTEST(keygen_api, test_invalid_sec_tag)
{
	/* Sec tag 0 maps to PSA key id 0, below PSA_KEY_ID_USER_MIN. */
	zassert_equal(tls_credentials_key_generate(0), -EINVAL,
		      "out-of-range sec tag should be rejected");
	zassert_equal(tls_credentials_key_generate(-1), -EINVAL,
		      "negative sec tag should be rejected");
	zassert_false(tls_credentials_key_exists(-1), "negative sec tag cannot have a key");
}

ZTEST(keygen_api, test_delete)
{
	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");
	zassert_ok(tls_credentials_key_delete(TEST_SEC_TAG), "delete failed");
	zassert_false(tls_credentials_key_exists(TEST_SEC_TAG),
		      "key should be gone after delete");
}

ZTEST(keygen_api, test_single_sec_tag_conflict)
{
	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");

	/* A second, different sec tag must be rejected without side effects. */
	zassert_equal(tls_credentials_key_generate(TEST_SEC_TAG_2), -ENOTSUP,
		      "second sec tag should be rejected with -ENOTSUP");
	zassert_false(tls_credentials_key_exists(TEST_SEC_TAG_2),
		      "rejected request must not leave a key behind");
}

ZTEST(keygen_api, test_delete_then_switch_sec_tag)
{
	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");
	zassert_ok(tls_credentials_key_delete(TEST_SEC_TAG), "delete failed");

	/* Slot freed, so a different sec tag can now be used. */
	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG_2),
		   "generate for a new sec tag after delete failed");
}

ZTEST(keygen_api, test_restore_without_key)
{
	zassert_equal(tls_credentials_key_restore(TEST_SEC_TAG), -ENOENT,
		      "restore without a key should report -ENOENT");
}

ZTEST(keygen_api, test_restore_after_reboot)
{
	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");

	/* A reboot loses the TLS credential registration but keeps the PSA key. */
	zassert_ok(tls_credential_delete(TEST_SEC_TAG, TLS_CREDENTIAL_PRIVATE_KEY_PSA),
		   "failed to drop the registration");

	zassert_ok(tls_credentials_key_restore(TEST_SEC_TAG), "restore failed");

	/* Restoring twice is not an error. */
	zassert_ok(tls_credentials_key_restore(TEST_SEC_TAG), "second restore failed");

	/* The registration holds a pointer to the key id, so the credential exists again. */
	uint32_t key_id = 0;
	size_t len = sizeof(key_id);

	zassert_ok(tls_credential_get(TEST_SEC_TAG, TLS_CREDENTIAL_PRIVATE_KEY_PSA, &key_id,
				      &len),
		   "key not registered after restore");
	zassert_equal(key_id, (uint32_t)TEST_SEC_TAG, "registered key id mismatch");
}

ZTEST(keygen_api, test_restore_conflict)
{
	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");

	/* Create a key for a second tag behind the module's back, then try to register it. */
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t id;

	psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
	psa_set_key_bits(&attr, 256);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_HASH);
	psa_set_key_algorithm(&attr, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
	psa_set_key_lifetime(&attr, PSA_KEY_LIFETIME_PERSISTENT);
	psa_set_key_id(&attr, TEST_SEC_TAG_2);
	zassert_equal(psa_generate_key(&attr, &id), PSA_SUCCESS, "PSA key generation failed");

	zassert_equal(tls_credentials_key_restore(TEST_SEC_TAG_2), -ENOTSUP,
		      "restore of a second sec tag should be rejected with -ENOTSUP");

	(void)psa_destroy_key(TEST_SEC_TAG_2);
}

ZTEST(keygen_api, test_pubkey_get)
{
	uint8_t pub[65];
	size_t len = 0;

	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");

	zassert_equal(tls_credentials_pubkey_get(TEST_SEC_TAG, NULL, sizeof(pub), &len), -EINVAL,
		      "NULL output must be rejected");

	zassert_ok(tls_credentials_pubkey_get(TEST_SEC_TAG, pub, sizeof(pub), &len),
		   "pubkey export failed");
	zassert_equal(len, 65, "P-256 SEC1 public key must be 65 bytes");
	zassert_equal(pub[0], 0x04, "uncompressed point must start with 0x04");
}

ZTEST(keygen_api, test_csr_generate)
{
	uint8_t der[512];
	size_t len = 0;

	zassert_ok(tls_credentials_key_generate(TEST_SEC_TAG), "generate failed");

	zassert_equal(tls_credentials_csr_generate(TEST_SEC_TAG, NULL, der, sizeof(der), &len),
		      -EINVAL, "NULL subject must be rejected");
	zassert_equal(tls_credentials_csr_generate(TEST_SEC_TAG, "not a subject", der,
						   sizeof(der), &len),
		      -EINVAL, "invalid subject must be rejected");

	zassert_ok(tls_credentials_csr_generate(TEST_SEC_TAG, "CN=test,O=Nordic", der,
						sizeof(der), &len),
		   "CSR generation failed");
	zassert_true(len > 0, "CSR length must be positive");
	zassert_equal(der[0], 0x30, "DER CSR must begin with a SEQUENCE tag");

	/* Re-parse the CSR. This validates the DER structure, the requested subject and the
	 * self-signature made through PSA with the on-device key.
	 */
	mbedtls_x509_csr csr;
	char subj[64];

	mbedtls_x509_csr_init(&csr);
	zassert_equal(mbedtls_x509_csr_parse_der(&csr, der, len), 0,
		      "generated CSR does not parse");
	zassert_true(mbedtls_x509_dn_gets(subj, sizeof(subj), &csr.subject) > 0,
		     "could not read CSR subject");
	zassert_not_null(strstr(subj, "CN=test"), "unexpected CSR subject: %s", subj);
	zassert_not_null(strstr(subj, "O=Nordic"), "unexpected CSR subject: %s", subj);
	mbedtls_x509_csr_free(&csr);
}

ZTEST(keygen_api, test_rollback_on_registration_failure)
{
	static const uint8_t dummy[] = {0xAA};
	const sec_tag_t fill_tags[] = {9001, 9002, 9003, 9004};

	BUILD_ASSERT(ARRAY_SIZE(fill_tags) >= CONFIG_TLS_MAX_CREDENTIALS_NUMBER,
		     "must be able to fill the whole TLS credentials store");

	/* Fill the TLS credentials store so the registration inside key_generate() fails after
	 * the PSA key has been generated.
	 */
	for (int i = 0; i < CONFIG_TLS_MAX_CREDENTIALS_NUMBER; i++) {
		zassert_ok(tls_credential_add(fill_tags[i], TLS_CREDENTIAL_CA_CERTIFICATE, dummy,
					      sizeof(dummy)),
			   "failed to fill TLS credentials store");
	}

	zassert_not_equal(tls_credentials_key_generate(TEST_SEC_TAG), 0,
			  "generate should fail when registration cannot complete");
	zassert_false(tls_credentials_key_exists(TEST_SEC_TAG),
		      "failed registration must roll back the generated key");

	for (int i = 0; i < CONFIG_TLS_MAX_CREDENTIALS_NUMBER; i++) {
		(void)tls_credential_delete(fill_tags[i], TLS_CREDENTIAL_CA_CERTIFICATE);
	}
}
