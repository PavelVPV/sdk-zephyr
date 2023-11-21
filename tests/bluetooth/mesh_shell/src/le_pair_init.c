/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "le_pair_init.h"
#include "../../../subsys/bluetooth/mesh/model_utils.h"
#include "mesh/access.h"

#define LOG_LEVEL CONFIG_BT_MESH_MODEL_LOG_LEVEL
#include "zephyr/logging/log.h"
LOG_MODULE_REGISTER(bt_mesh_le_pair_init);

#define BT_MESH_LE_PAIR_OP BT_MESH_MODEL_OP_3(0x11, BT_MESH_VENDOR_COMPANY_ID)

#define BT_MESH_LE_PAIR_OP_RESET 0x00
#define BT_MESH_LE_PAIR_OP_STATUS 0x01

#define STATUS_PASSKEY_SET 0x00
#define STATUS_PASSKEY_NOT_SET 0x01

static int handle_status(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			 struct net_buf_simple *buf)
{
	struct bt_mesh_le_pair_init *cli = model->user_data;

	uint32_t passkey = 0;
	uint8_t status;

	if (buf->len != 4) {
		LOG_ERR("Invalid length: %d", buf->len);
		return -EINVAL;
	}

	status = net_buf_simple_pull_u8(buf);
	if (status == STATUS_PASSKEY_SET) {
		passkey = net_buf_simple_pull_le24(buf);
	}

	cli->passkey(cli, status == STATUS_PASSKEY_SET, passkey);

	return 0;
}

static int handle_op(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
		     struct net_buf_simple *buf)
{
	uint8_t op;

	op = net_buf_simple_pull_u8(buf);
	switch (op) {
	case BT_MESH_LE_PAIR_OP_STATUS:
		return handle_status(model, ctx, buf);
	default:
		LOG_WRN("Unknown opcode: %d", op);
	}

	return 0;
}

const struct bt_mesh_model_op _bt_mesh_le_pair_init_op[] = {
	{ BT_MESH_LE_PAIR_OP, BT_MESH_LEN_MIN(1), handle_op, },
	BT_MESH_MODEL_OP_END,
};

static int bt_mesh_le_pair_init_init(struct bt_mesh_model *model)
{
	struct bt_mesh_le_pair_init *cli = model->user_data;

	cli->model = model;

	return 0;
}

const struct bt_mesh_model_cb _bt_mesh_le_pair_init_cb = {
	.init = bt_mesh_le_pair_init_init,
};

int bt_mesh_le_pair_init_passkey_reset(struct bt_mesh_le_pair_init *cli,
					struct bt_mesh_msg_ctx *ctx)
{
	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_LE_PAIR_OP, 1);
	bt_mesh_model_msg_init(&msg, BT_MESH_LE_PAIR_OP);

	net_buf_simple_add_u8(&msg, BT_MESH_LE_PAIR_OP_RESET);

	return bt_mesh_model_send(cli->model, ctx, &msg, NULL, NULL);
}
