#!/usr/bin/env bash
# Copyright (c) 2022 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

# EATT test
simulation_id="l2cap_deadlock"
verbosity_level=2
EXECUTE_TIMEOUT=240

bsim_exe=./bs_${BOARD_TS}_tests_bsim_bluetooth_host_l2cap_deadlock_prj_conf

cd ${BSIM_OUT_PATH}/bin

Execute "${bsim_exe}" -v=${verbosity_level} -s=${simulation_id} -d=0 -testid=tester -rs=43
Execute "${bsim_exe}" -v=${verbosity_level} -s=${simulation_id} -d=1 -testid=dut -rs=42

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} -D=2 -sim_length=800e6 $@

wait_for_background_jobs
