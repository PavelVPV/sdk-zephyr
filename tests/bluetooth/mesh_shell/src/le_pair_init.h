/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @defgroup bt_mesh_le_pair_init LE Pairing Initiator model
 * @{
 * @brief API for the LE Pairing Initiator model.
 */

#ifndef BT_MESH_LE_PAIR_INIT_H__
#define BT_MESH_LE_PAIR_INIT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <bluetooth/mesh/models.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/mesh/access.h>

// FIXME: Collision with DM
#define BT_MESH_VENDOR_COMPANY_ID 0x0059
#define BT_MESH_MODEL_ID_LE_PAIR_INIT 0x000D

/** @def BT_MESH_MODEL_LE_PAIRING_INIT
 *
 * @brief LE Pairing Initiator model entry.
 *
 * @param[in] _srv Pointer to a @ref bt_mesh_dm_srv instance.
 */
#define BT_MESH_MODEL_LE_PAIR_INIT(_cli)                                                             \
	BT_MESH_MODEL_VND_CB(BT_MESH_VENDOR_COMPANY_ID, BT_MESH_MODEL_ID_LE_PAIR_INIT,             \
			     _bt_mesh_le_pair_init_op, NULL,                                       \
			 BT_MESH_MODEL_USER_DATA(struct bt_mesh_le_pair_init,     \
						 _cli),                        \
			     &_bt_mesh_le_pair_init_cb)

struct bt_mesh_le_pair_init {
	void (*passkey)(struct bt_mesh_le_pair_init *cli, bool status, uint32_t passkey);
	struct bt_mesh_model *model;
};

int bt_mesh_le_pair_init_passkey_reset(struct bt_mesh_le_pair_init *cli,
					struct bt_mesh_msg_ctx *ctx);

/** @cond INTERNAL_HIDDEN */

extern const struct bt_mesh_model_op _bt_mesh_le_pair_init_op[];
extern const struct bt_mesh_model_cb _bt_mesh_le_pair_init_cb;

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* BT_MESH_LE_PAIR_INIT_H__ */

/** @} */
