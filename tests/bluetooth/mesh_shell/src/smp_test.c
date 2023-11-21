/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/init.h>

#include <zephyr/shell/shell.h>

/* .. include_startingpoint_mesh_smp_dfu_rst_1 */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "le_pair_init.h"

#define TARGET_ADDR 0x000A

static struct bt_mesh_le_pair_init *le_pair_init;
static struct bt_conn *g_conn;
static struct bt_gatt_discover_params discover;

/* SMP service.
 * {8D53DC1D-1DB7-4CD3-868B-8A527460AA84}
 */
static struct bt_uuid_128 smp_bt_svc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x8d53dc1d, 0x1db7, 0x4cd3, 0x868b, 0x8a527460aa84));

/* SMP characteristic; used for both requests and responses.
 * {DA2E7828-FBCE-4E01-AE9E-261174997C48}
 */
static struct bt_uuid_128 smp_bt_chr_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xda2e7828, 0xfbce, 0x4e01, 0xae9e, 0x261174997c48));

static struct bt_uuid_16 cccd = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

static void scan_start(void);
static void scan_stop(void);

static uint32_t g_passkey;

static struct bt_gatt_read_params read_params;

static bool g_req_auth = true;

static void passkey_cb(struct bt_mesh_le_pair_init *mod, bool status, uint32_t passkey)
{
	int err;

	printk("Passkey received %s: %u\n", status ? "succ" : "NOT succ", passkey);

	if (!status) {
		(void)bt_conn_auth_cancel(g_conn);
		return;
	}

	if (!g_conn) {
		printk("Uknown connection!\n");
		return;
	}

	g_passkey = passkey;

	/* Attempt to read characteristic should trigger pairing request. */
	err = bt_gatt_read(g_conn, &read_params);
	if (err) {
		printk("CCCD read failed (err %d)\n", err);
	}
}

static void cancel(struct bt_conn *conn)
{
	printk("%s:%d\n", __func__, __LINE__);
}

static void pairing_confirm(struct bt_conn *conn)
{
	int err;

	printk("%s:%d\n", __func__, __LINE__);

	err = bt_conn_auth_pairing_confirm(conn);
	if (err) {
		printf("Can't confirm pairing (err: %d)\n", err);
	}
}

static void passkey_entry(struct bt_conn *conn)
{
	int err;

	err = bt_conn_auth_passkey_entry(g_conn, g_passkey);
	if (err) {
		printk("Failed to set passkey: %d\n", err);
		bt_conn_auth_cancel(conn);
	}
}

static enum bt_security_err pairing_accept(struct bt_conn *conn,
					   const struct bt_conn_pairing_feat *const feat)
{
	if (conn != g_conn) {
		printk("Pairing request from unknown connection (%p), expected from (%p)", conn,
		       g_conn);
		return BT_SECURITY_ERR_UNSPECIFIED;
	}

	return BT_SECURITY_ERR_SUCCESS;
}

static struct bt_conn_auth_cb auth_cb = {
	.pairing_accept = pairing_accept,
	.passkey_entry = passkey_entry,
	.cancel = cancel,
	.pairing_confirm = pairing_confirm,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("Pairing complete\n");

	scan_start();
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	printk("Pairing failed\n");

	scan_start();
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static uint8_t read_func(struct bt_conn *conn, uint8_t err,
			 struct bt_gatt_read_params *params,
			 const void *data, uint16_t length)
{
	printk("Read error code: %d\n", err);

	return BT_GATT_ITER_STOP;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("GATT Services Discover complete\nh");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (!bt_uuid_cmp(discover.uuid, (struct bt_uuid *)&smp_bt_svc_uuid)) {
		printk("SMP Service discovered\n");
		discover.uuid = (struct bt_uuid *)&smp_bt_chr_uuid;
		discover.start_handle = attr->handle + 1;
		discover.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover);
		if (err) {
			printk("Discover GATT data in char failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover.uuid, (struct bt_uuid *)&smp_bt_chr_uuid)) {
		printk("SMP Characteristic discovered\n");

		discover.uuid = (struct bt_uuid *)&cccd;
		discover.start_handle = attr->handle + 2;
		discover.type = BT_GATT_DISCOVER_DESCRIPTOR;

		err = bt_gatt_discover(conn, &discover);
		if (err) {
			printk("Discover GATT CCCD failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover.uuid, (struct bt_uuid *)&cccd)) {
		printk("SMP CCCD discovered\n");

		/* Prepare read params */
		read_params.func = read_func;
		read_params.handle_count = 1;
		read_params.single.handle = attr->handle;

		struct bt_mesh_msg_ctx ctx = BT_MESH_MSG_CTX_INIT_APP(0, TARGET_ADDR);

		if (g_req_auth) {
			/* Ready to request the passkey. */
			err = bt_mesh_le_pair_init_passkey_reset(le_pair_init, &ctx);
			if (err) {
				printk("Failed to request passkey: %d\n", err);
			} else {
				printk("Waiting for the passkey from another device...\n");
			}
		} else {
			/* Attempt to read characteristic should trigger pairing request. */
			err = bt_gatt_read(g_conn, &read_params);
			if (err) {
				printk("CCCD read failed (err %d)\n", err);
			}
		}
	}

	return BT_GATT_ITER_STOP;
}

static void gatt_connected(struct bt_conn *conn, uint8_t conn_err)
{
	struct bt_conn_info info;
	int err;

	printk("conn %p err 0x%02x\n", (void *)conn, conn_err);

	bt_conn_get_info(conn, &info);
	if (info.role != BT_CONN_ROLE_CENTRAL || conn != g_conn) {
		return;
	}

	/* Restore mesh scanning after connecting. */
	err = bt_mesh_scan_enable();
	if (err) {
		printk("Unable to start mesh scan again: %d\n", err);
	}

	if (conn_err) {
		printk("Failed to connect GATT Services(%u)\n", conn_err);

		bt_conn_unref(g_conn);

		scan_start();
		return;
	}

	discover.uuid = (struct bt_uuid *)&smp_bt_svc_uuid;
	discover.func = discover_func;
	discover.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover.type = BT_GATT_DISCOVER_PRIMARY;
	err = bt_gatt_discover(conn, &discover);
	if (err) {
		printk("Unable discover GATT Services (err %d)\n", err);
	}
}

static void gatt_disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info info;

	printk("dis conn %p reason 0x%02x\n", (void *)conn, reason);

	bt_conn_get_info(conn, &info);
	if (info.role != BT_CONN_ROLE_CENTRAL || g_conn != conn) {
		return;
	}

	bt_conn_unref(g_conn);
}

static int gatt_connect(const bt_addr_le_t *addr)
{
	int err;

	err = bt_mesh_scan_disable();
	if (err) {
		printk("Unable to stop mesh scan: %d\n", err);
		return err;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &g_conn);
	if (err) {
		printk("Connection start failed (err:%d)\n", err);
	}

	return err;
}

static void device_found(const struct bt_le_scan_recv_info *info)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, addr, sizeof(addr));

	if (!bt_mesh_is_provisioned() ||
	    NULL == bt_mesh_cdb_node_get(TARGET_ADDR)) {
		printk("Device %s found, but not provisioned yet\n", addr);
		return;
	}

	printk("Connecting to %s\n", addr);

	scan_stop();
	gatt_connect(info->addr);
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	if (info->adv_type != BT_GAP_ADV_TYPE_ADV_IND) {
		return;
	}

	while (buf->len > 1) {
		struct net_buf_simple_state state;
		uint8_t len, type;

		len = net_buf_simple_pull_u8(buf);
		/* Check for early termination */
		if (len == 0U) {
			return;
		}

		if (len > buf->len) {
			printk("AD malformed\n");
			return;
		}

		net_buf_simple_save(buf, &state);

		type = net_buf_simple_pull_u8(buf);

		buf->len = len - 1;

		switch (type) {
		case BT_DATA_UUID128_ALL:
			if (buf->len == 16 &&
			    !memcmp(buf->data, smp_bt_svc_uuid.val, 16)) {
				device_found(info);
				return;
			}
			break;
#if 0
		case BT_DATA_NAME_COMPLETE:
			const uint8_t name[] = "Mesh DFU Distributor";

			if (!memcmp(name, buf->data, MIN(len, strlen(name)))) {
				device_found(info);
				return;
			}
			break;
#endif
		default:
			break;
		}

		net_buf_simple_restore(buf, &state);
		net_buf_simple_pull(buf, len);
	}
}

static struct bt_le_scan_cb scan_cb = {
	.recv = scan_recv,
};

static void scan_start(void)
{
	printk("Scanning started\n");
	bt_le_scan_cb_register(&scan_cb);
//	bt_mesh_scan_active_set(true);
}

static void scan_stop(void)
{
	printk("Scanning stopped\n");
	bt_le_scan_cb_unregister(&scan_cb);
//	bt_mesh_scan_active_set(false);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = gatt_connected,
	.disconnected = gatt_disconnected,
};

static int cmd_scan_start(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	if (argc > 1) {
		g_req_auth = shell_strtobool(argv[1], 0, &err);
	}

	scan_start();
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(smp_cmds,
	SHELL_CMD_ARG(scan-start, NULL, NULL, cmd_scan_start, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(smp, &smp_cmds, "SMP test shell commands", NULL, 1, 1);

int smp_bt_auth_init(struct bt_mesh_le_pair_init *mod)
{
	int err;

	err = bt_conn_auth_cb_register(&auth_cb);
	if (err) {
		printk("Can't reg bt conn auth cb (err: %d)\n", err);
		return err;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Can't reg auth info cb (err: %d)\n", err);
		return err;
	}

	le_pair_init = mod;
	le_pair_init->passkey = passkey_cb;

	return 0;
}
