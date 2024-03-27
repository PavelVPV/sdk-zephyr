/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/stats/stats.h>
#include <zephyr/usb/usb_device.h>

#ifdef CONFIG_MCUMGR_GRP_FS
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#endif
#ifdef CONFIG_MCUMGR_GRP_STAT
#include <zephyr/mgmt/mcumgr/grp/stat_mgmt/stat_mgmt.h>
#endif

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(smp_sample);

#include "common.h"

#define STORAGE_PARTITION_LABEL	storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION_LABEL)

/* Define an example stats group; approximates seconds since boot. */
STATS_SECT_START(smp_svr_stats)
STATS_SECT_ENTRY(ticks)
STATS_SECT_END;

/* Assign a name to the `ticks` stat. */
STATS_NAME_START(smp_svr_stats)
STATS_NAME(smp_svr_stats, ticks)
STATS_NAME_END(smp_svr_stats);

/* Define an instance of the stats group. */
STATS_SECT_DECL(smp_svr_stats) smp_svr_stats;

#ifdef CONFIG_MCUMGR_GRP_FS
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);
static struct fs_mount_t littlefs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &cstorage,
	.storage_dev = (void *)STORAGE_PARTITION_ID,
	.mnt_point = "/lfs1"
};
#endif

#include <zephyr/shell/shell.h>
#include <zephyr/dfu/mcuboot.h>

static int cmd_request_update(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	err = boot_request_upgrade(BOOT_UPGRADE_TEST);
	shell_print(sh, "boot_request_upgrade() returned %d", err);

	return 0;
}

static int cmd_img_confirm(const struct shell *sh, size_t argc, char *argv[])
{
	int err;

	err = boot_write_img_confirmed();
	shell_print(sh, "boot_write_img_confirmed() returned %d", err);

	return 0;
}

static int cmd_is_confirmed(const struct shell *sh, size_t argc, char *argv[])
{
	bool is_confirmed;

	is_confirmed = boot_is_img_confirmed();
	shell_print(sh, "boot_is_img_confirmed() returned %d", is_confirmed);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(boot_cmds,
	SHELL_CMD_ARG(request-update, NULL, NULL, cmd_request_update, 1, 0),
	SHELL_CMD_ARG(img-confirm, NULL, NULL, cmd_img_confirm, 1, 0),
	SHELL_CMD_ARG(is-confirmed, NULL, NULL, cmd_is_confirmed, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_ARG_REGISTER(boot, &boot_cmds, "boot commands", NULL, 1, 1);

int main(void)
{
	int rc = STATS_INIT_AND_REG(smp_svr_stats, STATS_SIZE_32,
				    "smp_svr_stats");

	if (rc < 0) {
		LOG_ERR("Error initializing stats system [%d]", rc);
	}

	/* Register the built-in mcumgr command handlers. */
#ifdef CONFIG_MCUMGR_GRP_FS
	rc = fs_mount(&littlefs_mnt);
	if (rc < 0) {
		LOG_ERR("Error mounting littlefs [%d]", rc);
	}
#endif

#ifdef CONFIG_MCUMGR_TRANSPORT_BT
	start_smp_bluetooth_adverts();
#endif

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		rc = usb_enable(NULL);

		/* Ignore EALREADY error as USB CDC is likely already initialised */
		if (rc != 0 && rc != -EALREADY) {
			LOG_ERR("Failed to enable USB");
			return 0;
		}
	}
	/* using __TIME__ ensure that a new binary will be built on every
	 * compile which is convenient when testing firmware upgrade.
	 */
	LOG_INF("build time: " __DATE__ " " __TIME__);

	/* The system work queue handles all incoming mcumgr requests.  Let the
	 * main thread idle while the mcumgr server runs.
	 */
	while (1) {
		k_sleep(K_MSEC(1000));
		STATS_INC(smp_svr_stats, ticks);
	}
	return 0;
}
