/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <zephyr/kernel.h>

#include <zephyr/shell/shell.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/bluetooth/mesh/shell.h>

/**************************************************************************************************/

/* Use logging subsystem for printing Config Client responses to the prints along with mesh stack
 * log messages and their timestamps.
 */
#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mesh_shell);

/* Define Config Client callbacks to print responses on get messages sent asynchronously. */

static void cfg_cli_comp_data(struct bt_mesh_cfg_cli *cli, uint16_t addr, uint8_t page,
			  struct net_buf_simple *buf)
{
	struct bt_mesh_comp_p0_elem elem;
	struct bt_mesh_comp_p0 comp;
	int err;

	if (page != 0 && page != 128) {
		LOG_WRN("Unexpected CDP %d", page);
		return;
	}

	err = bt_mesh_comp_p0_get(&comp, buf);

	if (err) {
		LOG_ERR("Couldn't parse Composition data (err %d)", err);
		return;
	}

	LOG_INF("Got Composition Data for 0x%04x, page: %d:",
		    addr, page);
	LOG_INF("\tCID      0x%04x", comp.cid);
	LOG_INF("\tPID      0x%04x", comp.pid);
	LOG_INF("\tVID      0x%04x", comp.vid);
	LOG_INF("\tCRPL     0x%04x", comp.crpl);
	LOG_INF("\tFeatures 0x%04x", comp.feat);

	while (bt_mesh_comp_p0_elem_pull(&comp, &elem)) {
		int i;

		LOG_INF("\tElement @ 0x%04x:", elem.loc);

		if (elem.nsig) {
			LOG_INF("\t\tSIG Models:");
		} else {
			LOG_INF("\t\tNo SIG Models");
		}

		for (i = 0; i < elem.nsig; i++) {
			uint16_t mod_id = bt_mesh_comp_p0_elem_mod(&elem, i);

			LOG_INF("\t\t\t0x%04x", mod_id);
		}

		if (elem.nvnd) {
			LOG_INF("\t\tVendor Models:");
		} else {
			LOG_INF("\t\tNo Vendor Models");
		}

		for (i = 0; i < elem.nvnd; i++) {
			struct bt_mesh_mod_id_vnd mod =
				bt_mesh_comp_p0_elem_mod_vnd(&elem, i);

			LOG_INF("\t\t\tCompany 0x%04x: 0x%04x",
				    mod.company, mod.id);
		}
	}
}

static void cfg_cli_beacon_status(struct bt_mesh_cfg_cli *cli, uint16_t addr,
				uint8_t status)
{
	LOG_INF("Beacon state is 0x%02x", status);
}

static void cfg_cli_ttl_status(struct bt_mesh_cfg_cli *cli, uint16_t addr,
				uint8_t ttl)
{
	LOG_INF("Default TTL is 0x%02x", ttl);
}

static void cfg_cli_friend_status(struct bt_mesh_cfg_cli *cli, uint16_t addr,
				uint8_t frnd)
{
	LOG_INF("Friend is set to 0x%02x", frnd);
}

static void cfg_cli_gatt_proxy_status(struct bt_mesh_cfg_cli *cli, uint16_t addr,
				uint8_t proxy)
{
	LOG_INF("GATT Proxy is set to 0x%02x", proxy);
}

static void cfg_cli_network_transmit_status(struct bt_mesh_cfg_cli *cli, uint16_t addr,
					uint8_t transmit)
{
	LOG_INF("Transmit 0x%02x (count %u interval %ums)", transmit,
		    BT_MESH_TRANSMIT_COUNT(transmit), BT_MESH_TRANSMIT_INT(transmit));
}

static void cfg_cli_relay_status(struct bt_mesh_cfg_cli *cli, uint16_t addr,
			     uint8_t relay, uint8_t transmit)
{
	LOG_INF("Relay is 0x%02x, Transmit 0x%02x (count %u interval %ums)", relay,
		    transmit, BT_MESH_TRANSMIT_COUNT(transmit), BT_MESH_TRANSMIT_INT(transmit));
}

static void cfg_cli_net_key_list(struct bt_mesh_cfg_cli *cli, uint16_t addr,
			    struct net_buf_simple *buf)
{
	uint16_t keys[16];
	size_t cnt;
	int err;

	cnt = ARRAY_SIZE(keys);

	err = bt_mesh_key_idx_unpack_list(buf, keys, &cnt);
	if (err) {
		LOG_ERR("Failed to unpack net keys: %d", err);
		return;
	}

	LOG_INF("NetKeys known by 0x%04x:", addr);
	for (int i = 0; i < cnt; i++) {
		LOG_INF("\t0x%03x", keys[i]);
	}
}

static void cfg_cli_app_key_list(struct bt_mesh_cfg_cli *cli, uint16_t addr, uint8_t status,
			    uint16_t net_idx, struct net_buf_simple *buf)
{
	uint16_t keys[16];
	size_t cnt;
	int err;

	cnt = ARRAY_SIZE(keys);

	if (status) {
		LOG_ERR("AppKeyGet failed with status 0x%02x", status);
		return;
	}

	err = bt_mesh_key_idx_unpack_list(buf, keys, &cnt);
	if (err) {
		LOG_ERR("Failed to unpack app keys: %d", err);
		return;
	}

	LOG_INF("AppKeys for NetKey 0x%03x known by 0x%04x:", net_idx,
		    addr);
	for (int i = 0; i < cnt; i++) {
		LOG_INF("\t0x%03x", keys[i]);
	}
}

static void cfg_cli_hb_pub_status(struct bt_mesh_cfg_cli *cli, uint16_t addr, uint8_t status,
			      struct bt_mesh_cfg_cli_hb_pub *pub)
{
	LOG_INF("Heartbeat publication:");
	LOG_INF("\tdst 0x%04x count 0x%02x period 0x%02x", pub->dst, pub->count, pub->period);
	LOG_INF("\tttl 0x%02x feat 0x%04x net_idx 0x%04x", pub->ttl, pub->feat, pub->net_idx);
}

static void cfg_cli_hb_sub_status(struct bt_mesh_cfg_cli *cli, uint16_t addr, uint8_t status,
				 struct bt_mesh_cfg_cli_hb_sub *sub)
{
	LOG_INF("Heartbeat Subscription:\n"
		"\tSource:      0x%04x\n"
		"\tDestination: 0x%04x\n"
		"\tPeriodLog:   0x%02x\n"
		"\tCountLog:    0x%02x\n"
		"\tMinHops:     %u\n"
		"\tMaxHops:     %u",
		sub->src, sub->dst, sub->period, sub->count, sub->min, sub->max);
}

static const struct bt_mesh_cfg_cli_cb cfg_cli_cb = {
	.comp_data = cfg_cli_comp_data,
	.beacon_status = cfg_cli_beacon_status,
	.ttl_status = cfg_cli_ttl_status,
	.friend_status = cfg_cli_friend_status,
	.gatt_proxy_status = cfg_cli_gatt_proxy_status,
	.network_transmit_status = cfg_cli_network_transmit_status,
	.relay_status = cfg_cli_relay_status,
	.net_key_list = cfg_cli_net_key_list,
	.app_key_list = cfg_cli_app_key_list,
	.hb_pub_status = cfg_cli_hb_pub_status,
	.hb_sub_status = cfg_cli_hb_sub_status,
};

static struct bt_mesh_cfg_cli cfg_cli = {
	.cb = &cfg_cli_cb,
};

/**************************************************************************************************/

#if defined(CONFIG_BT_MESH_DFD_SRV)
static struct bt_mesh_dfd_srv dfd_srv;
#endif

#if defined(CONFIG_BT_MESH_SAR_CFG_CLI)
static struct bt_mesh_sar_cfg_cli sar_cfg_cli;
#endif

#if defined(CONFIG_BT_MESH_PRIV_BEACON_CLI)
static struct bt_mesh_priv_beacon_cli priv_beacon_cli;
#endif

#if defined(CONFIG_BT_MESH_SOL_PDU_RPL_CLI)
static struct bt_mesh_sol_pdu_rpl_cli srpl_cli;
#endif


#if defined(CONFIG_BT_MESH_OD_PRIV_PROXY_CLI)
static struct bt_mesh_od_priv_proxy_cli od_priv_proxy_cli;
#endif

#if defined(CONFIG_BT_MESH_LARGE_COMP_DATA_CLI)
struct bt_mesh_large_comp_data_cli large_comp_data_cli;
#endif

BT_MESH_SHELL_HEALTH_PUB_DEFINE(health_pub);

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&bt_mesh_shell_health_srv, &health_pub),
	BT_MESH_MODEL_HEALTH_CLI(&bt_mesh_shell_health_cli),
#if defined(CONFIG_BT_MESH_DFD_SRV)
	BT_MESH_MODEL_DFD_SRV(&dfd_srv),
#else
#if defined(CONFIG_BT_MESH_SHELL_DFU_SRV)
	BT_MESH_MODEL_DFU_SRV(&bt_mesh_shell_dfu_srv),
#elif defined(CONFIG_BT_MESH_SHELL_BLOB_SRV)
	BT_MESH_MODEL_BLOB_SRV(&bt_mesh_shell_blob_srv),
#endif
#if defined(CONFIG_BT_MESH_SHELL_DFU_CLI)
	BT_MESH_MODEL_DFU_CLI(&bt_mesh_shell_dfu_cli),
#elif defined(CONFIG_BT_MESH_SHELL_BLOB_CLI)
	BT_MESH_MODEL_BLOB_CLI(&bt_mesh_shell_blob_cli),
#endif
#endif /* CONFIG_BT_MESH_DFD_SRV */
#if defined(CONFIG_BT_MESH_SHELL_RPR_CLI)
	BT_MESH_MODEL_RPR_CLI(&bt_mesh_shell_rpr_cli),
#endif
#if defined(CONFIG_BT_MESH_RPR_SRV)
	BT_MESH_MODEL_RPR_SRV,
#endif

#if defined(CONFIG_BT_MESH_SAR_CFG_SRV)
	BT_MESH_MODEL_SAR_CFG_SRV,
#endif
#if defined(CONFIG_BT_MESH_SAR_CFG_CLI)
	BT_MESH_MODEL_SAR_CFG_CLI(&sar_cfg_cli),
#endif

#if defined(CONFIG_BT_MESH_OP_AGG_SRV)
	BT_MESH_MODEL_OP_AGG_SRV,
#endif
#if defined(CONFIG_BT_MESH_OP_AGG_CLI)
	BT_MESH_MODEL_OP_AGG_CLI,
#endif

#if defined(CONFIG_BT_MESH_LARGE_COMP_DATA_SRV)
	BT_MESH_MODEL_LARGE_COMP_DATA_SRV,
#endif
#if defined(CONFIG_BT_MESH_LARGE_COMP_DATA_CLI)
	BT_MESH_MODEL_LARGE_COMP_DATA_CLI(&large_comp_data_cli),
#endif

#if defined(CONFIG_BT_MESH_PRIV_BEACON_SRV)
	BT_MESH_MODEL_PRIV_BEACON_SRV,
#endif
#if defined(CONFIG_BT_MESH_PRIV_BEACON_CLI)
	BT_MESH_MODEL_PRIV_BEACON_CLI(&priv_beacon_cli),
#endif
#if defined(CONFIG_BT_MESH_OD_PRIV_PROXY_CLI)
	BT_MESH_MODEL_OD_PRIV_PROXY_CLI(&od_priv_proxy_cli),
#endif
#if defined(CONFIG_BT_MESH_SOL_PDU_RPL_CLI)
	BT_MESH_MODEL_SOL_PDU_RPL_CLI(&srpl_cli),
#endif
#if defined(CONFIG_BT_MESH_OD_PRIV_PROXY_SRV)
	BT_MESH_MODEL_OD_PRIV_PROXY_SRV,
#endif
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/**************************************************************************************************/

static int cmd_get_info(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	/* Request Config Composition Data Status message. */
	err = bt_mesh_cfg_cli_comp_data_get(bt_mesh_shell_target_ctx.net_idx,
					bt_mesh_shell_target_ctx.dst, 0, NULL, NULL);
	if (err) {
		shell_error(sh, "Getting composition failed (err %d)", err);
		return 0;
	}

	/* Request Config Beacon Get message. */
	err = bt_mesh_cfg_cli_beacon_get(bt_mesh_shell_target_ctx.net_idx,
				     bt_mesh_shell_target_ctx.dst, NULL);
	if (err) {
		shell_error(sh, "Getting Beacon failed (err %d)", err);
		return 0;
	}

	/* Request Config TTL Status message. */
	err = bt_mesh_cfg_cli_ttl_get(bt_mesh_shell_target_ctx.net_idx,
				  bt_mesh_shell_target_ctx.dst, NULL);
	if (err) {
		shell_error(sh, "Getting TTL failed (err %d)", err);
		return 0;
	}

	/* Request Config Friend Status message. */
	err = bt_mesh_cfg_cli_friend_get(bt_mesh_shell_target_ctx.net_idx,
				     bt_mesh_shell_target_ctx.dst, NULL);
	if (err) {
		shell_error(sh, "Getting Friend failed (err %d)", err);
		return 0;
	}

	/* Request Config GATT Proxy Status message. */
	err = bt_mesh_cfg_cli_gatt_proxy_get(bt_mesh_shell_target_ctx.net_idx,
					 bt_mesh_shell_target_ctx.dst, NULL);
	if (err) {
		shell_error(sh, "Getting GATT Proxy failed (err %d)", err);
		return 0;
	}

	/* Request Network Transmit Status message. */
	err = bt_mesh_cfg_cli_net_transmit_get(bt_mesh_shell_target_ctx.net_idx,
					   bt_mesh_shell_target_ctx.dst, NULL);
	if (err) {
		shell_error(sh, "Getting Network Transmit State failed (err %d)", err);
		return 0;
	}

	/* Request Config Relay Status message. */
	err = bt_mesh_cfg_cli_relay_get(bt_mesh_shell_target_ctx.net_idx,
				    bt_mesh_shell_target_ctx.dst, NULL, NULL);
	if (err) {
		shell_error(sh, "Getting Relay failed (err %d)", err);
		return 0;
	}

	/* Raquest Config NetKey List message. */
	err = bt_mesh_cfg_cli_net_key_get(bt_mesh_shell_target_ctx.net_idx,
				      bt_mesh_shell_target_ctx.dst, NULL, NULL);
	if (err) {
		shell_error(sh, "Getting NetKey List failed (err %d)", err);
		return 0;
	}

	/* Raquest Config AppKey List message. */
	err = bt_mesh_cfg_cli_app_key_get(bt_mesh_shell_target_ctx.net_idx,
				      bt_mesh_shell_target_ctx.dst, bt_mesh_shell_target_ctx.net_idx, NULL, NULL, NULL);
	if (err) {
		shell_error(sh, "Getting AppKey List failed (err %d)", err);
		return 0;
	}

	/* Request Heartbeat Subscription Status message. */
	err = bt_mesh_cfg_cli_hb_sub_get(bt_mesh_shell_target_ctx.net_idx,
					 bt_mesh_shell_target_ctx.dst, NULL, NULL);
	if (err) {
		shell_error(sh, "Getting Heartbeat Subscription Status failed (err %d)", err);
		return 0;
	}

	/* Request Heartbeat Publication Status message. */
	err = bt_mesh_cfg_cli_hb_pub_get(bt_mesh_shell_target_ctx.net_idx,
					 bt_mesh_shell_target_ctx.dst, NULL, NULL);
	if (err) {
		shell_error(sh, "Getting Heartbeat Publication Status failed (err %d)", err);
		return 0;
	}

	shell_print(sh, "All messages are pushed");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(demo_cmds,
	SHELL_CMD_ARG(get-info, NULL, NULL, cmd_get_info, 1, 0),
	SHELL_SUBCMD_SET_END);

static int demo_cmds_help(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(
		sh,
		"\nOpAgg & SAR demo shell commands\n");

	if (argc == 1) {
		shell_help(sh);
		return 0;
	}

	shell_error(sh, "%s unknown command: %s", argv[0], argv[1]);
	return -EINVAL;
}

SHELL_CMD_ARG_REGISTER(demo, &demo_cmds, "OpAgg & SAR Cfg demo commands", demo_cmds_help, 1, 1);

/**************************************************************************************************/

static void bt_ready(int err)
{
	if (err && err != -EALREADY) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&bt_mesh_shell_prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d)\n", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	printk("Mesh initialized\n");

	if (bt_mesh_is_provisioned()) {
		printk("Mesh network restored from flash\n");
	} else {
		printk("Use \"prov pb-adv on\" or \"prov pb-gatt on\" to "
			    "enable advertising\n");
	}
}

int main(void)
{
	int err;

	printk("Initializing...\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err && err != -EALREADY) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	printk("Press the <Tab> button for supported commands.\n");
	printk("Before any Mesh commands you must run \"mesh init\"\n");
	return 0;
}
