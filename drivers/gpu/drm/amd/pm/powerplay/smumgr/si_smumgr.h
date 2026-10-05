/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * SI (Southern Islands) SMU manager for PowerPlay.
 *
 * Oland-first bring-up: SMC6 (smu_6_0) messaging and SRAM access ported
 * from legacy-dpm/si_smc.c and exposed via struct pp_smumgr_func so
 * si_hwmgr can drive it like CI (ci_smumgr.c).
 *
 * Phase 2: firmware upload, firmware header parsing, SMC
 * start/stop/halt/resume, SRAM dword and soft-register access, and the
 * legacy per-message SMC timeouts.
 *
 * Phase 3: SMC table population ported from si_dpm.c. SI has one
 * monolithic SISLANDS_SMC_STATETABLE (initial/ACPI/ULV/driver states)
 * that is built once at DPM enable (init_smc_table) and whose
 * driverState is re-uploaded on every power state switch
 * (si_smc_upload_sw_state), unlike the per-level SMU7 tables. The
 * populate_all_graphic/memory_levels smum hooks therefore stay no-ops;
 * si_hwmgr drives the uploads below directly in its enable/set-state
 * sequences, mirroring legacy si_dpm_enable/si_dpm_set_power_state.
 */
#ifndef _SI_SMUMANAGER_H_
#define _SI_SMUMANAGER_H_

#include "sislands_smc.h"
#include <pp_endian.h>
#include "ppatomctrl.h"

/* SI SMC SRAM limit for Tahiti/Pitcairn/Verde/Oland/Hainan
 * (legacy si_dpm.c: si_pi->sram_end = SMC_RAM_END = 0x20000).
 */
#define SI_SMC_RAM_END 0x20000

struct si_mc_reg_entry {
	uint32_t mclk_max;
	uint32_t mc_data[SMC_SISLANDS_MC_REGISTER_ARRAY_SIZE];
};

/* Field names match legacy si_dpm.h so the MC reg table code ports 1:1. */
struct si_mc_reg_table {
	uint8_t last;
	uint8_t num_entries;
	uint16_t valid_flag;
	struct si_mc_reg_entry mc_reg_table_entry[MAX_AC_TIMING_ENTRIES];
	SMC_SIslands_MCRegisterAddress mc_reg_address[SMC_SISLANDS_MC_REGISTER_ARRAY_SIZE];
};

struct si_smumgr {
	uint32_t sram_end;
	/* SMC SRAM offsets from SISLANDS_SMC_FIRMWARE_HEADER_* */
	uint32_t state_table_start;
	uint32_t soft_regs_start;
	uint32_t mc_reg_table_start;
	uint32_t arb_table_start;
	uint32_t cac_table_start;
	uint32_t dte_table_start;
	uint32_t spll_table_start;
	uint32_t papm_cfg_table_start;
	uint32_t fan_table_start;
	/* scratch SMC images (legacy si_pi->smc_statetable & co.) */
	SISLANDS_SMC_STATETABLE smc_statetable;
	SMC_SIslands_MCRegisters smc_mc_reg_table;
	PP_SIslands_PAPMParameters papm_parm;
	struct si_mc_reg_table mc_reg_table;
};

extern const struct pp_smumgr_func si_smu_funcs;

/* SRAM / soft register access for si_hwmgr. All take the amdgpu
 * smc_idx_lock like legacy si_smc.c since SMC_IND_INDEX_0 is shared
 * with adev->smc_rreg/wreg.
 */
int si_smc_copy_bytes(struct pp_hwmgr *hwmgr, uint32_t smc_start_address,
		      const uint8_t *src, uint32_t byte_count, uint32_t limit);
int si_smc_read_sram_dword(struct pp_hwmgr *hwmgr, uint32_t smc_address,
			   uint32_t *value, uint32_t limit);
int si_smc_write_sram_dword(struct pp_hwmgr *hwmgr, uint32_t smc_address,
			    uint32_t value, uint32_t limit);
int si_smc_read_soft_register(struct pp_hwmgr *hwmgr, uint16_t reg_offset,
			      uint32_t *value);
int si_smc_write_soft_register(struct pp_hwmgr *hwmgr, uint16_t reg_offset,
			       uint32_t value);

/* SMC lifecycle (legacy amdgpu_si_*_smc / si_dpm_start_smc). */
bool si_smc_is_running(struct pp_hwmgr *hwmgr);
void si_smc_start(struct pp_hwmgr *hwmgr);
void si_smc_reset(struct pp_hwmgr *hwmgr);
void si_smc_clock(struct pp_hwmgr *hwmgr, bool enable);
int si_smc_program_jump_on_start(struct pp_hwmgr *hwmgr);
int si_smc_wait_for_inactive(struct pp_hwmgr *hwmgr);
int si_smc_halt(struct pp_hwmgr *hwmgr);
int si_smc_resume(struct pp_hwmgr *hwmgr);
/* si_dpm_start_smc(): jump-on-start, release reset, enable clock. */
void si_smc_dpm_start(struct pp_hwmgr *hwmgr);
/* si_dpm_stop_smc(): reset, disable clock. */
void si_smc_dpm_stop(struct pp_hwmgr *hwmgr);
/* si_upload_firmware(): reset + clock off + ucode copy. */
int si_smc_upload_firmware(struct pp_hwmgr *hwmgr);

struct si_power_state;

/* SMC table population/upload used by the si_hwmgr enable and
 * set-power-state sequences (legacy si_dpm.c names minus the prefix).
 * All of them require si_process_firmware_header() to have run.
 */
int si_smc_init_arb_table_index(struct pp_hwmgr *hwmgr);
int si_smc_initial_switch_from_arb_f0_to_f1(struct pp_hwmgr *hwmgr);
int si_smc_force_switch_to_arb_f0(struct pp_hwmgr *hwmgr);
int si_smc_init_spll_table(struct pp_hwmgr *hwmgr);
int si_smc_populate_mc_reg_table(struct pp_hwmgr *hwmgr,
				 const struct si_power_state *boot_state);
int si_smc_upload_mc_reg_table(struct pp_hwmgr *hwmgr,
			       const struct si_power_state *new_state);
int si_smc_upload_sw_state(struct pp_hwmgr *hwmgr,
			   const struct si_power_state *new_state);
int si_smc_upload_ulv_state(struct pp_hwmgr *hwmgr);
int si_smc_upload_smc_data(struct pp_hwmgr *hwmgr);
int si_smc_program_memory_timing_parameters(struct pp_hwmgr *hwmgr,
					    const struct si_power_state *new_state);
void si_smc_program_response_times(struct pp_hwmgr *hwmgr);
int si_smc_populate_smc_tdp_limits(struct pp_hwmgr *hwmgr);
int si_smc_populate_smc_tdp_limits_2(struct pp_hwmgr *hwmgr);
int si_smc_initialize_smc_dte_tables(struct pp_hwmgr *hwmgr);
int si_smc_initialize_smc_cac_tables(struct pp_hwmgr *hwmgr);

#endif
