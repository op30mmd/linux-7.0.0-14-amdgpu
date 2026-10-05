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
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include <drm/drm_fixed.h>
#include <drm/amd_asic_type.h>
#include "smumgr.h"
#include "pp_debug.h"
#include "si_smumgr.h"
#include "si_hwmgr.h"
#include "ppsmc.h"
#include "hardwaremanager.h"
#include "cgs_common.h"
#include "atom.h"
#include "pptable.h"
#include "r600_dpm.h"

#include "smu/smu_6_0_d.h"
#include "smu/smu_6_0_sh_mask.h"

#include "gca/gfx_6_0_d.h"
#include "gca/gfx_6_0_sh_mask.h"

#include "gmc/gmc_6_0_d.h"
#include "gmc/gmc_6_0_sh_mask.h"

#include "dce/dce_6_0_d.h"
#include "dce/dce_6_0_sh_mask.h"

#include "bif/bif_3_0_d.h"

/* SMC SYSCON indirect addresses and bits (amdgpu sid.h).
 * smu_6_0_sh_mask.h carries no field macros for these, so they are
 * accessed through cgs_read/write_ind_register(CGS_IND_REG__SMC), which
 * maps onto adev->smc_rreg/wreg and takes smc_idx_lock.
 */
#define SI_SMC_SYSCON_RESET_CNTL	0x80000000
#define SI_SMC_SYSCON_RESET_RST_REG	(1 << 0)
#define SI_SMC_SYSCON_CLOCK_CNTL_0	0x80000004
#define SI_SMC_SYSCON_CK_DISABLE	(1 << 0)
#define SI_SMC_SYSCON_CKEN		(1 << 24)

static inline uint32_t si_smc_ind_read(struct pp_hwmgr *hwmgr, uint32_t addr)
{
	return cgs_read_ind_register(hwmgr->device, CGS_IND_REG__SMC, addr);
}

static inline void si_smc_ind_write(struct pp_hwmgr *hwmgr, uint32_t addr,
				    uint32_t val)
{
	cgs_write_ind_register(hwmgr->device, CGS_IND_REG__SMC, addr, val);
}

/* Caller holds adev->smc_idx_lock. */
static int si_set_smc_sram_address(struct pp_hwmgr *hwmgr,
				   uint32_t smc_address, uint32_t limit)
{
	if (smc_address & 3)
		return -EINVAL;
	if ((smc_address + 3) > limit)
		return -EINVAL;

	cgs_write_register(hwmgr->device, mmSMC_IND_INDEX_0, smc_address);
	PHM_WRITE_FIELD(hwmgr->device, SMC_IND_ACCESS_CNTL,
			AUTO_INCREMENT_IND_0, 0);

	return 0;
}

int si_smc_copy_bytes(struct pp_hwmgr *hwmgr, uint32_t smc_start_address,
		      const uint8_t *src, uint32_t byte_count, uint32_t limit)
{
	struct amdgpu_device *adev = hwmgr->adev;
	unsigned long flags;
	int ret = 0;
	uint32_t data, original_data, addr, extra_shift;

	if (smc_start_address & 3)
		return -EINVAL;
	if ((smc_start_address + byte_count) > limit)
		return -EINVAL;

	addr = smc_start_address;

	spin_lock_irqsave(&adev->smc_idx_lock, flags);
	while (byte_count >= 4) {
		/* SMC address space is BE */
		data = (src[0] << 24) | (src[1] << 16) | (src[2] << 8) | src[3];

		ret = si_set_smc_sram_address(hwmgr, addr, limit);
		if (ret)
			goto done;

		cgs_write_register(hwmgr->device, mmSMC_IND_DATA_0, data);

		src += 4;
		byte_count -= 4;
		addr += 4;
	}

	/* RMW for the final bytes */
	if (byte_count > 0) {
		data = 0;

		ret = si_set_smc_sram_address(hwmgr, addr, limit);
		if (ret)
			goto done;

		original_data = cgs_read_register(hwmgr->device, mmSMC_IND_DATA_0);
		extra_shift = 8 * (4 - byte_count);

		while (byte_count > 0) {
			/* SMC address space is BE */
			data = (data << 8) + *src++;
			byte_count--;
		}

		data <<= extra_shift;
		data |= (original_data & ~((~0UL) << extra_shift));

		ret = si_set_smc_sram_address(hwmgr, addr, limit);
		if (ret)
			goto done;

		cgs_write_register(hwmgr->device, mmSMC_IND_DATA_0, data);
	}

done:
	spin_unlock_irqrestore(&adev->smc_idx_lock, flags);

	return ret;
}

int si_smc_read_sram_dword(struct pp_hwmgr *hwmgr, uint32_t smc_address,
			   uint32_t *value, uint32_t limit)
{
	struct amdgpu_device *adev = hwmgr->adev;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&adev->smc_idx_lock, flags);
	ret = si_set_smc_sram_address(hwmgr, smc_address, limit);
	if (ret == 0)
		*value = cgs_read_register(hwmgr->device, mmSMC_IND_DATA_0);
	spin_unlock_irqrestore(&adev->smc_idx_lock, flags);

	return ret;
}

int si_smc_write_sram_dword(struct pp_hwmgr *hwmgr, uint32_t smc_address,
			    uint32_t value, uint32_t limit)
{
	struct amdgpu_device *adev = hwmgr->adev;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&adev->smc_idx_lock, flags);
	ret = si_set_smc_sram_address(hwmgr, smc_address, limit);
	if (ret == 0)
		cgs_write_register(hwmgr->device, mmSMC_IND_DATA_0, value);
	spin_unlock_irqrestore(&adev->smc_idx_lock, flags);

	return ret;
}

int si_smc_read_soft_register(struct pp_hwmgr *hwmgr, uint16_t reg_offset,
			      uint32_t *value)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;

	return si_smc_read_sram_dword(hwmgr,
				      smu_data->soft_regs_start + reg_offset,
				      value, smu_data->sram_end);
}

int si_smc_write_soft_register(struct pp_hwmgr *hwmgr, uint16_t reg_offset,
			       uint32_t value)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;

	return si_smc_write_sram_dword(hwmgr,
				       smu_data->soft_regs_start + reg_offset,
				       value, smu_data->sram_end);
}

void si_smc_start(struct pp_hwmgr *hwmgr)
{
	uint32_t tmp = si_smc_ind_read(hwmgr, SI_SMC_SYSCON_RESET_CNTL);

	tmp &= ~SI_SMC_SYSCON_RESET_RST_REG;

	si_smc_ind_write(hwmgr, SI_SMC_SYSCON_RESET_CNTL, tmp);
}

void si_smc_reset(struct pp_hwmgr *hwmgr)
{
	uint32_t tmp;

	/* Legacy amdgpu_si_reset_smc(): four dummy CB_CGTT_SCLK_CTRL reads
	 * settle the SCLK gating before pulling the SMC reset line.
	 */
	cgs_read_register(hwmgr->device, mmCB_CGTT_SCLK_CTRL);
	cgs_read_register(hwmgr->device, mmCB_CGTT_SCLK_CTRL);
	cgs_read_register(hwmgr->device, mmCB_CGTT_SCLK_CTRL);
	cgs_read_register(hwmgr->device, mmCB_CGTT_SCLK_CTRL);

	tmp = si_smc_ind_read(hwmgr, SI_SMC_SYSCON_RESET_CNTL) |
	      SI_SMC_SYSCON_RESET_RST_REG;
	si_smc_ind_write(hwmgr, SI_SMC_SYSCON_RESET_CNTL, tmp);
}

int si_smc_program_jump_on_start(struct pp_hwmgr *hwmgr)
{
	static const uint8_t data[] = { 0x0E, 0x00, 0x40, 0x40 };

	return si_smc_copy_bytes(hwmgr, 0x0, data, 4, sizeof(data) + 1);
}

void si_smc_clock(struct pp_hwmgr *hwmgr, bool enable)
{
	uint32_t tmp = si_smc_ind_read(hwmgr, SI_SMC_SYSCON_CLOCK_CNTL_0);

	if (enable)
		tmp &= ~SI_SMC_SYSCON_CK_DISABLE;
	else
		tmp |= SI_SMC_SYSCON_CK_DISABLE;

	si_smc_ind_write(hwmgr, SI_SMC_SYSCON_CLOCK_CNTL_0, tmp);
}

bool si_smc_is_running(struct pp_hwmgr *hwmgr)
{
	uint32_t rst = si_smc_ind_read(hwmgr, SI_SMC_SYSCON_RESET_CNTL);
	uint32_t clk = si_smc_ind_read(hwmgr, SI_SMC_SYSCON_CLOCK_CNTL_0);

	if (!(rst & SI_SMC_SYSCON_RESET_RST_REG) &&
	    !(clk & SI_SMC_SYSCON_CK_DISABLE))
		return true;

	return false;
}

/* Raw SMC message path. Returns the PPSMC_Result byte (0 on timeout)
 * exactly like legacy amdgpu_si_send_msg_to_smc() so callers that need
 * the distinction between "failed" and "timed out" still get it.
 */
static PPSMC_Result si_smc_send_msg_raw(struct pp_hwmgr *hwmgr, PPSMC_Msg msg)
{
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t tmp;
	int i;
	int usec_timeout;

	/* SMC seems to process some messages exceptionally slowly. */
	switch (msg) {
	case PPSMC_MSG_NoForcedLevel:
	case PPSMC_MSG_SetEnabledLevels:
	case PPSMC_MSG_SetForcedLevels:
	case PPSMC_MSG_DisableULV:
	case PPSMC_MSG_SwitchToSwState:
		usec_timeout = 1000000; /* 1 sec */
		break;
	default:
		usec_timeout = 200000; /* 200 ms */
		break;
	}

	if (!si_smc_is_running(hwmgr))
		return PPSMC_Result_Failed;

	cgs_write_register(hwmgr->device, mmSMC_MESSAGE_0, msg);

	for (i = 0; i < usec_timeout; i++) {
		tmp = cgs_read_register(hwmgr->device, mmSMC_RESP_0);
		if (tmp != 0)
			break;
		udelay(1);
	}

	tmp = cgs_read_register(hwmgr->device, mmSMC_RESP_0);
	if (tmp == 0) {
		drm_warn(adev_to_drm(adev),
			 "%s timeout on message: %x (SMC_SCRATCH0: %x)\n",
			 __func__, msg,
			 cgs_read_register(hwmgr->device, mmSMC_SCRATCH0));
	}

	return (PPSMC_Result)tmp;
}

int si_smc_wait_for_inactive(struct pp_hwmgr *hwmgr)
{
	uint32_t tmp;
	int i;

	if (!si_smc_is_running(hwmgr))
		return 0;

	for (i = 0; i < hwmgr->usec_timeout; i++) {
		tmp = si_smc_ind_read(hwmgr, SI_SMC_SYSCON_CLOCK_CNTL_0);
		if ((tmp & SI_SMC_SYSCON_CKEN) == 0)
			break;
		udelay(1);
	}

	/* Legacy amdgpu_si_wait_for_smc_inactive() reports OK even on
	 * timeout; keep that so the halt/resume sequence matches.
	 */
	return 0;
}

int si_smc_halt(struct pp_hwmgr *hwmgr)
{
	if (si_smc_send_msg_raw(hwmgr, PPSMC_MSG_Halt) != PPSMC_Result_OK)
		return -EINVAL;

	return si_smc_wait_for_inactive(hwmgr);
}

int si_smc_resume(struct pp_hwmgr *hwmgr)
{
	if (si_smc_send_msg_raw(hwmgr, PPSMC_FlushDataCache) != PPSMC_Result_OK)
		return -EINVAL;

	return (si_smc_send_msg_raw(hwmgr, PPSMC_MSG_Resume) == PPSMC_Result_OK) ?
		0 : -EINVAL;
}

void si_smc_dpm_start(struct pp_hwmgr *hwmgr)
{
	si_smc_program_jump_on_start(hwmgr);
	si_smc_start(hwmgr);
	si_smc_clock(hwmgr, true);
}

void si_smc_dpm_stop(struct pp_hwmgr *hwmgr)
{
	si_smc_reset(hwmgr);
	si_smc_clock(hwmgr, false);
}

static int si_load_smc_ucode(struct pp_hwmgr *hwmgr, uint32_t limit)
{
	struct amdgpu_device *adev = hwmgr->adev;
	struct cgs_firmware_info info = {0};
	unsigned long flags;
	uint32_t ucode_start_address;
	uint32_t ucode_size;
	const uint8_t *src;
	uint32_t data;
	int ret;

	/* cgs requests adev->pm.fw by ASIC name (amdgpu_cgs.c) and hands
	 * back the ucode array + start address from the SMC v1 header.
	 */
	ret = cgs_get_firmware_info(hwmgr->device, CGS_UCODE_ID_SMU, &info);
	if (ret)
		return ret;

	hwmgr->is_kicker = info.is_kicker;
	hwmgr->smu_version = info.version;
	ucode_start_address = info.ucode_start_address;
	ucode_size = info.image_size;
	src = (const uint8_t *)info.kptr;

	if (!src)
		return -EINVAL;
	if (ucode_size & 3)
		return -EINVAL;
	if ((ucode_start_address + ucode_size) > limit) {
		pr_err("si SMC ucode 0x%x+0x%x exceeds SRAM limit 0x%x\n",
		       ucode_start_address, ucode_size, limit);
		return -EINVAL;
	}

	spin_lock_irqsave(&adev->smc_idx_lock, flags);
	cgs_write_register(hwmgr->device, mmSMC_IND_INDEX_0, ucode_start_address);
	PHM_WRITE_FIELD(hwmgr->device, SMC_IND_ACCESS_CNTL,
			AUTO_INCREMENT_IND_0, 1);
	while (ucode_size >= 4) {
		/* SMC address space is BE */
		data = (src[0] << 24) | (src[1] << 16) | (src[2] << 8) | src[3];

		cgs_write_register(hwmgr->device, mmSMC_IND_DATA_0, data);

		src += 4;
		ucode_size -= 4;
	}
	PHM_WRITE_FIELD(hwmgr->device, SMC_IND_ACCESS_CNTL,
			AUTO_INCREMENT_IND_0, 0);
	spin_unlock_irqrestore(&adev->smc_idx_lock, flags);

	return 0;
}

int si_smc_upload_firmware(struct pp_hwmgr *hwmgr)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;

	si_smc_reset(hwmgr);
	si_smc_clock(hwmgr, false);

	return si_load_smc_ucode(hwmgr, smu_data->sram_end);
}

static int si_read_fw_header_dword(struct pp_hwmgr *hwmgr, uint32_t offset,
				   uint32_t *value)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;

	return si_smc_read_sram_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_LOCATION + offset,
				      value, smu_data->sram_end);
}

/* Legacy si_process_firmware_header(): the SMC image carries SRAM
 * offsets for every table the driver fills in. Must run after
 * si_smc_upload_firmware() and before any table upload.
 */
static int si_process_firmware_header(struct pp_hwmgr *hwmgr)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	uint32_t tmp;
	int ret;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_stateTable, &tmp);
	if (ret)
		return ret;
	smu_data->state_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_softRegisters, &tmp);
	if (ret)
		return ret;
	smu_data->soft_regs_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_mcRegisterTable, &tmp);
	if (ret)
		return ret;
	smu_data->mc_reg_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_fanTable, &tmp);
	if (ret)
		return ret;
	smu_data->fan_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_mcArbDramAutoRefreshTable,
				      &tmp);
	if (ret)
		return ret;
	smu_data->arb_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_CacConfigTable, &tmp);
	if (ret)
		return ret;
	smu_data->cac_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_DteConfiguration, &tmp);
	if (ret)
		return ret;
	smu_data->dte_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_spllTable, &tmp);
	if (ret)
		return ret;
	smu_data->spll_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_PAPMParameters, &tmp);
	if (ret)
		return ret;
	smu_data->papm_cfg_table_start = tmp;

	ret = si_read_fw_header_dword(hwmgr,
				      SISLANDS_SMC_FIRMWARE_HEADER_version, &tmp);
	if (ret)
		return ret;
	hwmgr->microcode_version_info.SMC = tmp;

	return 0;
}

/* pp_smumgr_func entry points */

static int si_send_msg_to_smc(struct pp_hwmgr *hwmgr, uint16_t msg)
{
	struct amdgpu_device *adev = hwmgr->adev;
	PPSMC_Result result;

	result = si_smc_send_msg_raw(hwmgr, msg);
	if (result != PPSMC_Result_OK) {
		dev_info(adev->dev,
			 "si failed to send message %x result is %d\n",
			 msg, result);
		return -EINVAL;
	}

	return 0;
}

static int si_send_msg_to_smc_with_parameter(struct pp_hwmgr *hwmgr,
					     uint16_t msg, uint32_t parameter)
{
	/* SI SMC6 has no MSG_ARG register; the parameter rides in
	 * SCRATCH0 (radeon si_send_msg_to_smc_with_parameter).
	 */
	cgs_write_register(hwmgr->device, mmSMC_SCRATCH0, parameter);
	return si_send_msg_to_smc(hwmgr, msg);
}

static uint32_t si_get_argument(struct pp_hwmgr *hwmgr)
{
	return cgs_read_register(hwmgr->device, mmSMC_SCRATCH0);
}

static int si_smu_init(struct pp_hwmgr *hwmgr)
{
	struct si_smumgr *si_priv;

	si_priv = kzalloc(sizeof(struct si_smumgr), GFP_KERNEL);
	if (!si_priv)
		return -ENOMEM;

	si_priv->sram_end = SI_SMC_RAM_END;
	hwmgr->smu_backend = si_priv;

	return 0;
}

static int si_smu_fini(struct pp_hwmgr *hwmgr)
{
	kfree(hwmgr->smu_backend);
	hwmgr->smu_backend = NULL;
	return 0;
}

static int si_start_smu(struct pp_hwmgr *hwmgr)
{
	/* On SI the SMC is uploaded and released inside the hwmgr enable
	 * sequence (legacy si_dpm_enable: upload_firmware ->
	 * process_firmware_header -> tables -> si_dpm_start_smc), not
	 * from pp_dpm_load_fw. Nothing to do here.
	 */
	return 0;
}

/* ------------------------------------------------------------------ */
/* SMC table population, ported from legacy-dpm/si_dpm.c.              */
/* ------------------------------------------------------------------ */

static const struct si_power_state *cast_const_si_ps(const struct pp_hw_power_state *hw_ps)
{
	PP_ASSERT_WITH_CODE((PHM_SIslands_Magic == hw_ps->magic),
			    "Invalid Powerstate Type!",
			    return NULL);
	return (const struct si_power_state *)hw_ps;
}

static inline uint32_t si_rreg(struct pp_hwmgr *hwmgr, uint32_t reg)
{
	return cgs_read_register(hwmgr->device, reg);
}

static inline void si_wreg(struct pp_hwmgr *hwmgr, uint32_t reg, uint32_t val)
{
	cgs_write_register(hwmgr->device, reg, val);
}

static inline void si_wreg_p(struct pp_hwmgr *hwmgr, uint32_t reg,
			     uint32_t val, uint32_t mask)
{
	uint32_t tmp = si_rreg(hwmgr, reg);

	tmp &= mask;
	tmp |= val;
	si_wreg(hwmgr, reg, tmp);
}

/* ---- memory clock ratio / strobe helpers ---- */

static uint8_t si_get_ddr3_mclk_frequency_ratio(uint32_t memory_clock)
{
	uint8_t mc_para_index;

	if (memory_clock < 10000)
		mc_para_index = 0;
	else if (memory_clock >= 80000)
		mc_para_index = 0x0f;
	else
		mc_para_index = (uint8_t)((memory_clock - 10000) / 5000 + 1);
	return mc_para_index;
}

static uint8_t si_get_mclk_frequency_ratio(uint32_t memory_clock, bool strobe_mode)
{
	uint8_t mc_para_index;

	if (strobe_mode) {
		if (memory_clock < 12500)
			mc_para_index = 0x00;
		else if (memory_clock > 47500)
			mc_para_index = 0x0f;
		else
			mc_para_index = (uint8_t)((memory_clock - 10000) / 2500);
	} else {
		if (memory_clock < 65000)
			mc_para_index = 0x00;
		else if (memory_clock > 135000)
			mc_para_index = 0x0f;
		else
			mc_para_index = (uint8_t)((memory_clock - 60000) / 5000);
	}
	return mc_para_index;
}

static uint8_t si_get_strobe_mode_settings(struct pp_hwmgr *hwmgr, uint32_t mclk)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	bool strobe_mode = false;
	uint8_t result = 0;

	if (mclk <= data->mclk_strobe_mode_threshold)
		strobe_mode = true;

	if (adev->gmc.vram_type == AMDGPU_VRAM_TYPE_GDDR5)
		result = si_get_mclk_frequency_ratio(mclk, strobe_mode);
	else
		result = si_get_ddr3_mclk_frequency_ratio(mclk);

	if (strobe_mode)
		result |= SISLANDS_SMC_STROBE_ENABLE;

	return result;
}

/* ---- voltage value helpers ---- */

static bool si_validate_phase_shedding_tables(const struct atom_voltage_table *table,
					      const struct phm_phase_shedding_limits_table *limits)
{
	uint32_t data, num_bits, num_levels;

	if ((table == NULL) || (limits == NULL))
		return false;

	data = table->mask_low;

	num_bits = hweight32(data);

	if (num_bits == 0)
		return false;

	num_levels = (1 << num_bits);

	if (table->count != num_levels)
		return false;

	if (limits->count != (num_levels - 1))
		return false;

	return true;
}

static void si_populate_smc_voltage_table(const struct atom_voltage_table *voltage_table,
					  SISLANDS_SMC_STATETABLE *table)
{
	unsigned int i;

	for (i = 0; i < voltage_table->count; i++)
		table->lowSMIO[i] |= cpu_to_be32(voltage_table->entries[i].smio_low);
}

static int si_populate_smc_voltage_tables(struct pp_hwmgr *hwmgr,
					  SISLANDS_SMC_STATETABLE *table)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint8_t i;

	if (data->voltage_control_svi2) {
		si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_svi_rework_gpio_id_svc,
					   data->svc_gpio_id);
		si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_svi_rework_gpio_id_svd,
					   data->svd_gpio_id);
		si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_svi_rework_plat_type,
					   2);
	} else {
		if (data->vddc_voltage_table.count) {
			si_populate_smc_voltage_table(&data->vddc_voltage_table, table);
			table->voltageMaskTable.lowMask[SISLANDS_SMC_VOLTAGEMASK_VDDC] =
				cpu_to_be32(data->vddc_voltage_table.mask_low);

			for (i = 0; i < data->vddc_voltage_table.count; i++) {
				if (data->max_vddc_in_table <= data->vddc_voltage_table.entries[i].value) {
					table->maxVDDCIndexInPPTable = i;
					break;
				}
			}
		}

		if (data->vddci_voltage_table.count) {
			si_populate_smc_voltage_table(&data->vddci_voltage_table, table);

			table->voltageMaskTable.lowMask[SISLANDS_SMC_VOLTAGEMASK_VDDCI] =
				cpu_to_be32(data->vddci_voltage_table.mask_low);
		}

		if (data->mvdd_voltage_table.count) {
			si_populate_smc_voltage_table(&data->mvdd_voltage_table, table);

			table->voltageMaskTable.lowMask[SISLANDS_SMC_VOLTAGEMASK_MVDD] =
				cpu_to_be32(data->mvdd_voltage_table.mask_low);
		}

		if (data->vddc_phase_shed_control) {
			if (si_validate_phase_shedding_tables(&data->vddc_phase_shed_table,
							      hwmgr->dyn_state.vddc_phase_shed_limits_table)) {
				si_populate_smc_voltage_table(&data->vddc_phase_shed_table, table);

				table->phaseMaskTable.lowMask[SISLANDS_SMC_VOLTAGEMASK_VDDC_PHASE_SHEDDING] =
					cpu_to_be32(data->vddc_phase_shed_table.mask_low);

				si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_phase_shedding_delay,
							   (uint32_t)data->vddc_phase_shed_table.phase_delay);
			} else {
				data->vddc_phase_shed_control = false;
			}
		}
	}

	return 0;
}

static int si_populate_voltage_value(const struct atom_voltage_table *table,
				     uint16_t value, SISLANDS_SMC_VOLTAGE_VALUE *voltage)
{
	unsigned int i;

	for (i = 0; i < table->count; i++) {
		if (value <= table->entries[i].value) {
			voltage->index = (uint8_t)i;
			voltage->value = cpu_to_be16(table->entries[i].value);
			break;
		}
	}

	if (i >= table->count)
		return -EINVAL;

	return 0;
}

static int si_populate_mvdd_value(struct pp_hwmgr *hwmgr, uint32_t mclk,
				  SISLANDS_SMC_VOLTAGE_VALUE *voltage)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (data->mvdd_control) {
		if (mclk <= data->mvdd_split_frequency)
			voltage->index = 0;
		else
			voltage->index = (uint8_t)(data->mvdd_voltage_table.count) - 1;

		voltage->value = cpu_to_be16(data->mvdd_voltage_table.entries[voltage->index].value);
	}
	return 0;
}

static int si_get_std_voltage_value(struct pp_hwmgr *hwmgr,
				    SISLANDS_SMC_VOLTAGE_VALUE *voltage,
				    uint16_t *std_voltage)
{
	const struct phm_cac_leakage_table *cac = hwmgr->dyn_state.cac_leakage_table;
	const struct phm_clock_voltage_dependency_table *dep =
		hwmgr->dyn_state.vddc_dependency_on_sclk;
	uint16_t v_index;
	bool voltage_found = false;

	*std_voltage = be16_to_cpu(voltage->value);

	if (cac) {
		if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
				    PHM_PlatformCaps_NewCACVoltage)) {
			if (dep == NULL)
				return -EINVAL;

			for (v_index = 0; (uint32_t)v_index < dep->count; v_index++) {
				if (be16_to_cpu(voltage->value) ==
				    (uint16_t)dep->entries[v_index].v) {
					voltage_found = true;
					if ((uint32_t)v_index < cac->count)
						*std_voltage = cac->entries[v_index].Vddc;
					else
						*std_voltage = cac->entries[cac->count - 1].Vddc;
					break;
				}
			}

			if (!voltage_found) {
				for (v_index = 0; (uint32_t)v_index < dep->count; v_index++) {
					if (be16_to_cpu(voltage->value) <=
					    (uint16_t)dep->entries[v_index].v) {
						voltage_found = true;
						if ((uint32_t)v_index < cac->count)
							*std_voltage = cac->entries[v_index].Vddc;
						else
							*std_voltage = cac->entries[cac->count - 1].Vddc;
						break;
					}
				}
			}
		} else {
			if ((uint32_t)voltage->index < cac->count)
				*std_voltage = cac->entries[voltage->index].Vddc;
		}
	}

	return 0;
}

static int si_populate_std_voltage_value(uint16_t value, uint8_t index,
					 SISLANDS_SMC_VOLTAGE_VALUE *voltage)
{
	voltage->index = index;
	voltage->value = cpu_to_be16(value);

	return 0;
}

static int si_populate_phase_shedding_value(const struct phm_phase_shedding_limits_table *limits,
					    uint16_t voltage, uint32_t sclk, uint32_t mclk,
					    SISLANDS_SMC_VOLTAGE_VALUE *smc_voltage)
{
	unsigned int i = 0;

	if (limits) {
		for (i = 0; i < limits->count; i++) {
			if ((voltage <= limits->entries[i].Voltage) &&
			    (sclk <= limits->entries[i].Sclk) &&
			    (mclk <= limits->entries[i].Mclk))
				break;
		}
	}

	smc_voltage->phase_settings = (uint8_t)i;

	return 0;
}

/* ---- MC arbiter (legacy ni_copy_and_switch_arb_sets & friends) ---- */

static int si_copy_and_switch_arb_sets(struct pp_hwmgr *hwmgr,
				       uint32_t arb_freq_src, uint32_t arb_freq_dest)
{
	uint32_t mc_arb_dram_timing;
	uint32_t mc_arb_dram_timing2;
	uint32_t burst_time;
	uint32_t mc_cg_config;

	switch (arb_freq_src) {
	case MC_CG_ARB_FREQ_F0:
		mc_arb_dram_timing  = si_rreg(hwmgr, mmMC_ARB_DRAM_TIMING);
		mc_arb_dram_timing2 = si_rreg(hwmgr, mmMC_ARB_DRAM_TIMING2);
		burst_time = (si_rreg(hwmgr, mmMC_ARB_BURST_TIME) & MC_ARB_BURST_TIME__STATE0_MASK) >>
			MC_ARB_BURST_TIME__STATE0__SHIFT;
		break;
	case MC_CG_ARB_FREQ_F1:
		mc_arb_dram_timing  = si_rreg(hwmgr, mmMC_ARB_DRAM_TIMING_1);
		mc_arb_dram_timing2 = si_rreg(hwmgr, mmMC_ARB_DRAM_TIMING2_1);
		burst_time = (si_rreg(hwmgr, mmMC_ARB_BURST_TIME) & MC_ARB_BURST_TIME__STATE1_MASK) >>
			MC_ARB_BURST_TIME__STATE1__SHIFT;
		break;
	case MC_CG_ARB_FREQ_F2:
		mc_arb_dram_timing  = si_rreg(hwmgr, SI_MC_ARB_DRAM_TIMING_2);
		mc_arb_dram_timing2 = si_rreg(hwmgr, SI_MC_ARB_DRAM_TIMING2_2);
		burst_time = (si_rreg(hwmgr, mmMC_ARB_BURST_TIME) & MC_ARB_BURST_TIME__STATE2_MASK) >>
			MC_ARB_BURST_TIME__STATE2__SHIFT;
		break;
	case MC_CG_ARB_FREQ_F3:
		mc_arb_dram_timing  = si_rreg(hwmgr, SI_MC_ARB_DRAM_TIMING_3);
		mc_arb_dram_timing2 = si_rreg(hwmgr, SI_MC_ARB_DRAM_TIMING2_3);
		burst_time = (si_rreg(hwmgr, mmMC_ARB_BURST_TIME) & MC_ARB_BURST_TIME__STATE3_MASK) >>
			MC_ARB_BURST_TIME__STATE3__SHIFT;
		break;
	default:
		return -EINVAL;
	}

	switch (arb_freq_dest) {
	case MC_CG_ARB_FREQ_F0:
		si_wreg(hwmgr, mmMC_ARB_DRAM_TIMING, mc_arb_dram_timing);
		si_wreg(hwmgr, mmMC_ARB_DRAM_TIMING2, mc_arb_dram_timing2);
		si_wreg_p(hwmgr, mmMC_ARB_BURST_TIME,
			  burst_time << MC_ARB_BURST_TIME__STATE0__SHIFT,
			  ~MC_ARB_BURST_TIME__STATE0_MASK);
		break;
	case MC_CG_ARB_FREQ_F1:
		si_wreg(hwmgr, mmMC_ARB_DRAM_TIMING_1, mc_arb_dram_timing);
		si_wreg(hwmgr, mmMC_ARB_DRAM_TIMING2_1, mc_arb_dram_timing2);
		si_wreg_p(hwmgr, mmMC_ARB_BURST_TIME,
			  burst_time << MC_ARB_BURST_TIME__STATE1__SHIFT,
			  ~MC_ARB_BURST_TIME__STATE1_MASK);
		break;
	case MC_CG_ARB_FREQ_F2:
		si_wreg(hwmgr, SI_MC_ARB_DRAM_TIMING_2, mc_arb_dram_timing);
		si_wreg(hwmgr, SI_MC_ARB_DRAM_TIMING2_2, mc_arb_dram_timing2);
		si_wreg_p(hwmgr, mmMC_ARB_BURST_TIME,
			  burst_time << MC_ARB_BURST_TIME__STATE2__SHIFT,
			  ~MC_ARB_BURST_TIME__STATE2_MASK);
		break;
	case MC_CG_ARB_FREQ_F3:
		si_wreg(hwmgr, SI_MC_ARB_DRAM_TIMING_3, mc_arb_dram_timing);
		si_wreg(hwmgr, SI_MC_ARB_DRAM_TIMING2_3, mc_arb_dram_timing2);
		si_wreg_p(hwmgr, mmMC_ARB_BURST_TIME,
			  burst_time << MC_ARB_BURST_TIME__STATE3__SHIFT,
			  ~MC_ARB_BURST_TIME__STATE3_MASK);
		break;
	default:
		return -EINVAL;
	}

	mc_cg_config = si_rreg(hwmgr, mmMC_CG_CONFIG) | 0x0000000F;
	si_wreg(hwmgr, mmMC_CG_CONFIG, mc_cg_config);
	si_wreg_p(hwmgr, mmMC_ARB_CG,
		  arb_freq_dest << MC_ARB_CG__CG_ARB_REQ__SHIFT,
		  ~MC_ARB_CG__CG_ARB_REQ_MASK);

	return 0;
}

int si_smc_init_arb_table_index(struct pp_hwmgr *hwmgr)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	uint32_t tmp;
	int ret;

	ret = si_smc_read_sram_dword(hwmgr, smu_data->arb_table_start,
				     &tmp, smu_data->sram_end);
	if (ret)
		return ret;

	tmp &= 0x00FFFFFF;
	tmp |= MC_CG_ARB_FREQ_F1 << 24;

	return si_smc_write_sram_dword(hwmgr, smu_data->arb_table_start,
				       tmp, smu_data->sram_end);
}

int si_smc_initial_switch_from_arb_f0_to_f1(struct pp_hwmgr *hwmgr)
{
	return si_copy_and_switch_arb_sets(hwmgr, MC_CG_ARB_FREQ_F0, MC_CG_ARB_FREQ_F1);
}

int si_smc_force_switch_to_arb_f0(struct pp_hwmgr *hwmgr)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	uint32_t tmp;
	int ret;

	ret = si_smc_read_sram_dword(hwmgr, smu_data->arb_table_start,
				     &tmp, smu_data->sram_end);
	if (ret)
		return ret;

	tmp = (tmp >> 24) & 0xff;

	if (tmp == MC_CG_ARB_FREQ_F0)
		return 0;

	return si_copy_and_switch_arb_sets(hwmgr, tmp, MC_CG_ARB_FREQ_F0);
}

static uint32_t si_calculate_memory_refresh_rate(struct pp_hwmgr *hwmgr,
						 uint32_t engine_clock)
{
	uint32_t dram_rows;
	uint32_t dram_refresh_rate;
	uint32_t mc_arb_rfsh_rate;
	uint32_t tmp = (si_rreg(hwmgr, mmMC_ARB_RAMCFG) & MC_ARB_RAMCFG__NOOFROWS_MASK) >>
		MC_ARB_RAMCFG__NOOFROWS__SHIFT;

	if (tmp >= 4)
		dram_rows = 16384;
	else
		dram_rows = 1 << (tmp + 10);

	dram_refresh_rate = 1 << ((si_rreg(hwmgr, mmMC_SEQ_MISC0) & 0x3) + 3);
	mc_arb_rfsh_rate = ((engine_clock * 10) * dram_refresh_rate / dram_rows - 32) / 64;

	return mc_arb_rfsh_rate;
}

static int si_populate_memory_timing_parameters(struct pp_hwmgr *hwmgr,
						const struct si_performance_level *pl,
						SMC_SIslands_MCArbDramTimingRegisterSet *arb_regs)
{
	uint32_t dram_timing;
	uint32_t dram_timing2;
	uint32_t burst_time;
	int ret;

	arb_regs->mc_arb_rfsh_rate =
		(uint8_t)si_calculate_memory_refresh_rate(hwmgr, pl->sclk);

	ret = amdgpu_atombios_set_engine_dram_timings(hwmgr->adev, pl->sclk,
						      pl->mclk);
	if (ret)
		return ret;

	dram_timing  = si_rreg(hwmgr, mmMC_ARB_DRAM_TIMING);
	dram_timing2 = si_rreg(hwmgr, mmMC_ARB_DRAM_TIMING2);
	burst_time = si_rreg(hwmgr, mmMC_ARB_BURST_TIME) & MC_ARB_BURST_TIME__STATE0_MASK;

	arb_regs->mc_arb_dram_timing  = cpu_to_be32(dram_timing);
	arb_regs->mc_arb_dram_timing2 = cpu_to_be32(dram_timing2);
	arb_regs->mc_arb_burst_time = (uint8_t)burst_time;

	return 0;
}

/* The MC arbiter counts its DRAM timings and refresh interval in SCLK
 * cycles (the ATOM DynamicMemorySettings table takes both clocks, and
 * the sets it returns for 400/1100 and 730/1100 differ ~2x), but the SMC
 * only loads a level's arbiter set as part of an MCLK switch. A switch
 * between two levels that share an MCLK changes SCLK under the set of
 * the level it came from: stepping up runs the DRAM with timings that
 * are too short in absolute time, stepping down stretches the refresh
 * interval. Legacy and upstream program each set for its own level's
 * SCLK only, so every SCLK-only step up (the VBIOS 400 -> 730 -> 780
 * steps at 1100 MHz, the OD 935 -> 1000 step at 1300 MHz) violates DRAM
 * timings and eventually hangs the gfx ring - the "higher sclks seem to
 * be unstable" that the Oland SCLK clamps paper over.
 *
 * Program every set so it is valid at any SCLK of the levels sharing
 * its MCLK: timings for the highest of them (more cycles is only
 * slower at a lower SCLK), the refresh rate for the lowest (refreshing
 * more often is only slower at a higher SCLK).
 */
static int si_do_program_memory_timing_parameters(struct pp_hwmgr *hwmgr,
						  const struct si_power_state *state,
						  unsigned int first_arb_set)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	SMC_SIslands_MCArbDramTimingRegisterSet arb_regs = { 0 };
	int i, j, ret = 0;

	for (i = 0; i < state->performance_level_count; i++) {
		struct si_performance_level pl = state->performance_levels[i];
		uint32_t min_sclk = pl.sclk, max_sclk = pl.sclk;

		for (j = 0; j < state->performance_level_count; j++) {
			if (state->performance_levels[j].mclk != pl.mclk)
				continue;
			min_sclk = min(min_sclk, state->performance_levels[j].sclk);
			max_sclk = max(max_sclk, state->performance_levels[j].sclk);
		}

		pl.sclk = max_sclk;
		ret = si_populate_memory_timing_parameters(hwmgr, &pl, &arb_regs);
		if (ret)
			break;
		arb_regs.mc_arb_rfsh_rate =
			(uint8_t)si_calculate_memory_refresh_rate(hwmgr, min_sclk);

		pr_debug("si arb set %u: level %d sclk %u mclk %u, timings @ sclk %u, refresh @ sclk %u (rate %u)\n",
			 first_arb_set + i, i, state->performance_levels[i].sclk / 100,
			 pl.mclk / 100, max_sclk / 100, min_sclk / 100,
			 arb_regs.mc_arb_rfsh_rate);
		ret = si_smc_copy_bytes(hwmgr,
					smu_data->arb_table_start +
					offsetof(SMC_SIslands_MCArbDramTimingRegisters, data) +
					sizeof(SMC_SIslands_MCArbDramTimingRegisterSet) * (first_arb_set + i),
					(uint8_t *)&arb_regs,
					sizeof(SMC_SIslands_MCArbDramTimingRegisterSet),
					smu_data->sram_end);
		if (ret)
			break;
	}

	return ret;
}

int si_smc_program_memory_timing_parameters(struct pp_hwmgr *hwmgr,
					    const struct si_power_state *new_state)
{
	return si_do_program_memory_timing_parameters(hwmgr, new_state,
						      SISLANDS_DRIVER_STATE_ARB_INDEX);
}

/* ---- initial / ACPI / ULV states ---- */

static int si_populate_initial_mvdd_value(struct pp_hwmgr *hwmgr,
					  struct SISLANDS_SMC_VOLTAGE_VALUE *voltage)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (data->mvdd_control)
		return si_populate_voltage_value(&data->mvdd_voltage_table,
						 data->mvdd_bootup_value, voltage);

	return 0;
}

static int si_populate_smc_initial_state(struct pp_hwmgr *hwmgr,
					 const struct si_power_state *initial_state,
					 SISLANDS_SMC_STATETABLE *table)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t reg;
	int ret;

	table->initialState.level.mclk.vDLL_CNTL =
		cpu_to_be32(data->clock_registers.dll_cntl);
	table->initialState.level.mclk.vMCLK_PWRMGT_CNTL =
		cpu_to_be32(data->clock_registers.mclk_pwrmgt_cntl);
	table->initialState.level.mclk.vMPLL_AD_FUNC_CNTL =
		cpu_to_be32(data->clock_registers.mpll_ad_func_cntl);
	table->initialState.level.mclk.vMPLL_DQ_FUNC_CNTL =
		cpu_to_be32(data->clock_registers.mpll_dq_func_cntl);
	table->initialState.level.mclk.vMPLL_FUNC_CNTL =
		cpu_to_be32(data->clock_registers.mpll_func_cntl);
	table->initialState.level.mclk.vMPLL_FUNC_CNTL_1 =
		cpu_to_be32(data->clock_registers.mpll_func_cntl_1);
	table->initialState.level.mclk.vMPLL_FUNC_CNTL_2 =
		cpu_to_be32(data->clock_registers.mpll_func_cntl_2);
	table->initialState.level.mclk.vMPLL_SS =
		cpu_to_be32(data->clock_registers.mpll_ss1);
	table->initialState.level.mclk.vMPLL_SS2 =
		cpu_to_be32(data->clock_registers.mpll_ss2);

	table->initialState.level.mclk.mclk_value =
		cpu_to_be32(initial_state->performance_levels[0].mclk);

	table->initialState.level.sclk.vCG_SPLL_FUNC_CNTL =
		cpu_to_be32(data->clock_registers.cg_spll_func_cntl);
	table->initialState.level.sclk.vCG_SPLL_FUNC_CNTL_2 =
		cpu_to_be32(data->clock_registers.cg_spll_func_cntl_2);
	table->initialState.level.sclk.vCG_SPLL_FUNC_CNTL_3 =
		cpu_to_be32(data->clock_registers.cg_spll_func_cntl_3);
	table->initialState.level.sclk.vCG_SPLL_FUNC_CNTL_4 =
		cpu_to_be32(data->clock_registers.cg_spll_func_cntl_4);
	table->initialState.level.sclk.vCG_SPLL_SPREAD_SPECTRUM =
		cpu_to_be32(data->clock_registers.cg_spll_spread_spectrum);
	table->initialState.level.sclk.vCG_SPLL_SPREAD_SPECTRUM_2  =
		cpu_to_be32(data->clock_registers.cg_spll_spread_spectrum_2);

	table->initialState.level.sclk.sclk_value =
		cpu_to_be32(initial_state->performance_levels[0].sclk);

	table->initialState.level.arbRefreshState =
		SISLANDS_INITIAL_STATE_ARB_INDEX;

	table->initialState.level.ACIndex = 0;

	ret = si_populate_voltage_value(&data->vddc_voltage_table,
					initial_state->performance_levels[0].vddc,
					&table->initialState.level.vddc);

	if (!ret) {
		uint16_t std_vddc;

		ret = si_get_std_voltage_value(hwmgr,
					       &table->initialState.level.vddc,
					       &std_vddc);
		if (!ret)
			si_populate_std_voltage_value(std_vddc,
						      table->initialState.level.vddc.index,
						      &table->initialState.level.std_vddc);
	}

	if (data->vddci_control)
		si_populate_voltage_value(&data->vddci_voltage_table,
					  initial_state->performance_levels[0].vddci,
					  &table->initialState.level.vddci);

	if (data->vddc_phase_shed_control)
		si_populate_phase_shedding_value(hwmgr->dyn_state.vddc_phase_shed_limits_table,
						 initial_state->performance_levels[0].vddc,
						 initial_state->performance_levels[0].sclk,
						 initial_state->performance_levels[0].mclk,
						 &table->initialState.level.vddc);

	si_populate_initial_mvdd_value(hwmgr, &table->initialState.level.mvdd);

	reg = 0xffff << CG_AT__CG_R__SHIFT | 0 << CG_AT__CG_L__SHIFT;
	table->initialState.level.aT = cpu_to_be32(reg);
	table->initialState.level.bSP = cpu_to_be32(data->dsp);
	table->initialState.level.gen2PCIE = (uint8_t)data->boot_pcie_gen;

	if (adev->gmc.vram_type == AMDGPU_VRAM_TYPE_GDDR5) {
		table->initialState.level.strobeMode =
			si_get_strobe_mode_settings(hwmgr,
						    initial_state->performance_levels[0].mclk);

		if (initial_state->performance_levels[0].mclk > data->mclk_edc_enable_threshold)
			table->initialState.level.mcFlags = SISLANDS_SMC_MC_EDC_RD_FLAG | SISLANDS_SMC_MC_EDC_WR_FLAG;
		else
			table->initialState.level.mcFlags =  0;
	}

	table->initialState.levelCount = 1;

	table->initialState.flags |= PPSMC_SWSTATE_FLAG_DC;

	table->initialState.level.dpm2.MaxPS = 0;
	table->initialState.level.dpm2.NearTDPDec = 0;
	table->initialState.level.dpm2.AboveSafeInc = 0;
	table->initialState.level.dpm2.BelowSafeInc = 0;
	table->initialState.level.dpm2.PwrEfficiencyRatio = 0;

	reg = SQ_POWER_THROTTLE__MIN_POWER_MASK |
		SQ_POWER_THROTTLE__MAX_POWER_MASK;
	table->initialState.level.SQPowerThrottle = cpu_to_be32(reg);

	reg = SQ_POWER_THROTTLE2__MAX_POWER_DELTA_MASK |
		SQ_POWER_THROTTLE2__SHORT_TERM_INTERVAL_SIZE_MASK |
		SQ_POWER_THROTTLE2__LONG_TERM_INTERVAL_RATIO_MASK;
	table->initialState.level.SQPowerThrottle_2 = cpu_to_be32(reg);

	return 0;
}

static int si_populate_smc_acpi_state(struct pp_hwmgr *hwmgr,
				      SISLANDS_SMC_STATETABLE *table)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t spll_func_cntl = data->clock_registers.cg_spll_func_cntl;
	uint32_t spll_func_cntl_2 = data->clock_registers.cg_spll_func_cntl_2;
	uint32_t spll_func_cntl_3 = data->clock_registers.cg_spll_func_cntl_3;
	uint32_t spll_func_cntl_4 = data->clock_registers.cg_spll_func_cntl_4;
	uint32_t dll_cntl = data->clock_registers.dll_cntl;
	uint32_t mclk_pwrmgt_cntl = data->clock_registers.mclk_pwrmgt_cntl;
	uint32_t mpll_ad_func_cntl = data->clock_registers.mpll_ad_func_cntl;
	uint32_t mpll_dq_func_cntl = data->clock_registers.mpll_dq_func_cntl;
	uint32_t mpll_func_cntl = data->clock_registers.mpll_func_cntl;
	uint32_t mpll_func_cntl_1 = data->clock_registers.mpll_func_cntl_1;
	uint32_t mpll_func_cntl_2 = data->clock_registers.mpll_func_cntl_2;
	uint32_t reg;
	int ret;

	table->ACPIState = table->initialState;

	table->ACPIState.flags &= ~PPSMC_SWSTATE_FLAG_DC;

	if (data->acpi_vddc) {
		ret = si_populate_voltage_value(&data->vddc_voltage_table,
						data->acpi_vddc, &table->ACPIState.level.vddc);
		if (!ret) {
			uint16_t std_vddc;

			ret = si_get_std_voltage_value(hwmgr,
						       &table->ACPIState.level.vddc, &std_vddc);
			if (!ret)
				si_populate_std_voltage_value(std_vddc,
							      table->ACPIState.level.vddc.index,
							      &table->ACPIState.level.std_vddc);
		}
		table->ACPIState.level.gen2PCIE = data->acpi_pcie_gen;

		if (data->vddc_phase_shed_control) {
			si_populate_phase_shedding_value(hwmgr->dyn_state.vddc_phase_shed_limits_table,
							 data->acpi_vddc,
							 0,
							 0,
							 &table->ACPIState.level.vddc);
		}
	} else {
		ret = si_populate_voltage_value(&data->vddc_voltage_table,
						data->min_vddc_in_table, &table->ACPIState.level.vddc);
		if (!ret) {
			uint16_t std_vddc;

			ret = si_get_std_voltage_value(hwmgr,
						       &table->ACPIState.level.vddc, &std_vddc);

			if (!ret)
				si_populate_std_voltage_value(std_vddc,
							      table->ACPIState.level.vddc.index,
							      &table->ACPIState.level.std_vddc);
		}
		table->ACPIState.level.gen2PCIE =
			(uint8_t)si_gen_pcie_gen_support(data->sys_pcie_mask,
							 data->boot_pcie_gen,
							 SI_PCIE_GEN1);

		if (data->vddc_phase_shed_control)
			si_populate_phase_shedding_value(hwmgr->dyn_state.vddc_phase_shed_limits_table,
							 data->min_vddc_in_table,
							 0,
							 0,
							 &table->ACPIState.level.vddc);
	}

	if (data->acpi_vddc) {
		if (data->acpi_vddci)
			si_populate_voltage_value(&data->vddci_voltage_table,
						  data->acpi_vddci,
						  &table->ACPIState.level.vddci);
	}

	mclk_pwrmgt_cntl |= MCLK_PWRMGT_CNTL__MRDCK0_RESET_MASK |
			    MCLK_PWRMGT_CNTL__MRDCK1_RESET_MASK;
	mclk_pwrmgt_cntl &= ~(MCLK_PWRMGT_CNTL__MRDCK0_PDNB_MASK |
			      MCLK_PWRMGT_CNTL__MRDCK1_PDNB_MASK);

	dll_cntl &= ~(DLL_CNTL__MRDCK0_BYPASS_MASK | DLL_CNTL__MRDCK1_BYPASS_MASK);

	spll_func_cntl_2 &= ~CG_SPLL_FUNC_CNTL_2__SCLK_MUX_SEL_MASK;
	spll_func_cntl_2 |= 4 << CG_SPLL_FUNC_CNTL_2__SCLK_MUX_SEL__SHIFT;

	table->ACPIState.level.mclk.vDLL_CNTL =
		cpu_to_be32(dll_cntl);
	table->ACPIState.level.mclk.vMCLK_PWRMGT_CNTL =
		cpu_to_be32(mclk_pwrmgt_cntl);
	table->ACPIState.level.mclk.vMPLL_AD_FUNC_CNTL =
		cpu_to_be32(mpll_ad_func_cntl);
	table->ACPIState.level.mclk.vMPLL_DQ_FUNC_CNTL =
		cpu_to_be32(mpll_dq_func_cntl);
	table->ACPIState.level.mclk.vMPLL_FUNC_CNTL =
		cpu_to_be32(mpll_func_cntl);
	table->ACPIState.level.mclk.vMPLL_FUNC_CNTL_1 =
		cpu_to_be32(mpll_func_cntl_1);
	table->ACPIState.level.mclk.vMPLL_FUNC_CNTL_2 =
		cpu_to_be32(mpll_func_cntl_2);
	table->ACPIState.level.mclk.vMPLL_SS =
		cpu_to_be32(data->clock_registers.mpll_ss1);
	table->ACPIState.level.mclk.vMPLL_SS2 =
		cpu_to_be32(data->clock_registers.mpll_ss2);

	table->ACPIState.level.sclk.vCG_SPLL_FUNC_CNTL =
		cpu_to_be32(spll_func_cntl);
	table->ACPIState.level.sclk.vCG_SPLL_FUNC_CNTL_2 =
		cpu_to_be32(spll_func_cntl_2);
	table->ACPIState.level.sclk.vCG_SPLL_FUNC_CNTL_3 =
		cpu_to_be32(spll_func_cntl_3);
	table->ACPIState.level.sclk.vCG_SPLL_FUNC_CNTL_4 =
		cpu_to_be32(spll_func_cntl_4);

	table->ACPIState.level.mclk.mclk_value = 0;
	table->ACPIState.level.sclk.sclk_value = 0;

	si_populate_mvdd_value(hwmgr, 0, &table->ACPIState.level.mvdd);

	if (data->dynamic_ac_timing)
		table->ACPIState.level.ACIndex = 0;

	table->ACPIState.level.dpm2.MaxPS = 0;
	table->ACPIState.level.dpm2.NearTDPDec = 0;
	table->ACPIState.level.dpm2.AboveSafeInc = 0;
	table->ACPIState.level.dpm2.BelowSafeInc = 0;
	table->ACPIState.level.dpm2.PwrEfficiencyRatio = 0;

	reg = SQ_POWER_THROTTLE__MIN_POWER_MASK | SQ_POWER_THROTTLE__MAX_POWER_MASK;
	table->ACPIState.level.SQPowerThrottle = cpu_to_be32(reg);

	reg = SQ_POWER_THROTTLE2__MAX_POWER_DELTA_MASK |
		SQ_POWER_THROTTLE2__SHORT_TERM_INTERVAL_SIZE_MASK |
		SQ_POWER_THROTTLE2__LONG_TERM_INTERVAL_RATIO_MASK;
	table->ACPIState.level.SQPowerThrottle_2 = cpu_to_be32(reg);

	return 0;
}

/* ---- sclk / mclk register values ---- */

static int si_calculate_sclk_params(struct pp_hwmgr *hwmgr,
				    uint32_t engine_clock,
				    SISLANDS_SMC_SCLK_VALUE *sclk)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	struct atom_clock_dividers dividers;
	uint32_t spll_func_cntl = data->clock_registers.cg_spll_func_cntl;
	uint32_t spll_func_cntl_2 = data->clock_registers.cg_spll_func_cntl_2;
	uint32_t spll_func_cntl_3 = data->clock_registers.cg_spll_func_cntl_3;
	uint32_t spll_func_cntl_4 = data->clock_registers.cg_spll_func_cntl_4;
	uint32_t cg_spll_spread_spectrum = data->clock_registers.cg_spll_spread_spectrum;
	uint32_t cg_spll_spread_spectrum_2 = data->clock_registers.cg_spll_spread_spectrum_2;
	uint64_t tmp;
	uint32_t reference_clock = adev->clock.spll.reference_freq;
	uint32_t reference_divider;
	uint32_t fbdiv;
	int ret;

	ret = amdgpu_atombios_get_clock_dividers(adev, COMPUTE_ENGINE_PLL_PARAM,
						 engine_clock, false, &dividers);
	if (ret)
		return ret;

	reference_divider = 1 + dividers.ref_div;

	tmp = (uint64_t)engine_clock * reference_divider * dividers.post_div * 16384;
	do_div(tmp, reference_clock);
	fbdiv = (uint32_t)tmp;

	spll_func_cntl &= ~(CG_SPLL_FUNC_CNTL__SPLL_PDIV_A_MASK | CG_SPLL_FUNC_CNTL__SPLL_REF_DIV_MASK);
	spll_func_cntl |= dividers.ref_div << CG_SPLL_FUNC_CNTL__SPLL_REF_DIV__SHIFT;
	spll_func_cntl |= dividers.post_div << CG_SPLL_FUNC_CNTL__SPLL_PDIV_A__SHIFT;

	spll_func_cntl_2 &= ~CG_SPLL_FUNC_CNTL_2__SCLK_MUX_SEL_MASK;
	spll_func_cntl_2 |= 2 << CG_SPLL_FUNC_CNTL_2__SCLK_MUX_SEL__SHIFT;

	spll_func_cntl_3 &= ~CG_SPLL_FUNC_CNTL_3__SPLL_FB_DIV_MASK;
	spll_func_cntl_3 |= fbdiv << CG_SPLL_FUNC_CNTL_3__SPLL_FB_DIV__SHIFT;
	spll_func_cntl_3 |= CG_SPLL_FUNC_CNTL_3__SPLL_DITHEN_MASK;

	if (data->sclk_ss) {
		struct amdgpu_atom_ss ss;
		uint32_t vco_freq = engine_clock * dividers.post_div;

		if (amdgpu_atombios_get_asic_ss_info(adev, &ss,
						     ASIC_INTERNAL_ENGINE_SS, vco_freq)) {
			uint32_t clk_s = reference_clock * 5 / (reference_divider * ss.rate);
			uint32_t clk_v = 4 * ss.percentage * fbdiv / (clk_s * 10000);

			cg_spll_spread_spectrum &= ~CG_SPLL_SPREAD_SPECTRUM__CLK_S_MASK;
			cg_spll_spread_spectrum |= clk_s << CG_SPLL_SPREAD_SPECTRUM__CLK_S__SHIFT;
			cg_spll_spread_spectrum |= CG_SPLL_SPREAD_SPECTRUM__SSEN_MASK;

			cg_spll_spread_spectrum_2 &= ~CG_SPLL_SPREAD_SPECTRUM_2__CLK_V_MASK;
			cg_spll_spread_spectrum_2 |= clk_v << CG_SPLL_SPREAD_SPECTRUM_2__CLK_V__SHIFT;
		}
	}

	sclk->sclk_value = engine_clock;
	sclk->vCG_SPLL_FUNC_CNTL = spll_func_cntl;
	sclk->vCG_SPLL_FUNC_CNTL_2 = spll_func_cntl_2;
	sclk->vCG_SPLL_FUNC_CNTL_3 = spll_func_cntl_3;
	sclk->vCG_SPLL_FUNC_CNTL_4 = spll_func_cntl_4;
	sclk->vCG_SPLL_SPREAD_SPECTRUM = cg_spll_spread_spectrum;
	sclk->vCG_SPLL_SPREAD_SPECTRUM_2 = cg_spll_spread_spectrum_2;

	return 0;
}

static int si_populate_sclk_value(struct pp_hwmgr *hwmgr,
				  uint32_t engine_clock,
				  SISLANDS_SMC_SCLK_VALUE *sclk)
{
	SISLANDS_SMC_SCLK_VALUE sclk_tmp;
	int ret;

	ret = si_calculate_sclk_params(hwmgr, engine_clock, &sclk_tmp);
	if (!ret) {
		sclk->sclk_value = cpu_to_be32(sclk_tmp.sclk_value);
		sclk->vCG_SPLL_FUNC_CNTL = cpu_to_be32(sclk_tmp.vCG_SPLL_FUNC_CNTL);
		sclk->vCG_SPLL_FUNC_CNTL_2 = cpu_to_be32(sclk_tmp.vCG_SPLL_FUNC_CNTL_2);
		sclk->vCG_SPLL_FUNC_CNTL_3 = cpu_to_be32(sclk_tmp.vCG_SPLL_FUNC_CNTL_3);
		sclk->vCG_SPLL_FUNC_CNTL_4 = cpu_to_be32(sclk_tmp.vCG_SPLL_FUNC_CNTL_4);
		sclk->vCG_SPLL_SPREAD_SPECTRUM = cpu_to_be32(sclk_tmp.vCG_SPLL_SPREAD_SPECTRUM);
		sclk->vCG_SPLL_SPREAD_SPECTRUM_2 = cpu_to_be32(sclk_tmp.vCG_SPLL_SPREAD_SPECTRUM_2);
	}

	return ret;
}

static int si_populate_mclk_value(struct pp_hwmgr *hwmgr,
				  uint32_t engine_clock,
				  uint32_t memory_clock,
				  SISLANDS_SMC_MCLK_VALUE *mclk,
				  bool strobe_mode,
				  bool dll_state_on)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t  dll_cntl = data->clock_registers.dll_cntl;
	uint32_t  mclk_pwrmgt_cntl = data->clock_registers.mclk_pwrmgt_cntl;
	uint32_t  mpll_ad_func_cntl = data->clock_registers.mpll_ad_func_cntl;
	uint32_t  mpll_dq_func_cntl = data->clock_registers.mpll_dq_func_cntl;
	uint32_t  mpll_func_cntl = data->clock_registers.mpll_func_cntl;
	uint32_t  mpll_func_cntl_1 = data->clock_registers.mpll_func_cntl_1;
	uint32_t  mpll_func_cntl_2 = data->clock_registers.mpll_func_cntl_2;
	uint32_t  mpll_ss1 = data->clock_registers.mpll_ss1;
	uint32_t  mpll_ss2 = data->clock_registers.mpll_ss2;
	struct atom_mpll_param mpll_param;
	int ret;

	ret = amdgpu_atombios_get_memory_pll_dividers(adev, memory_clock, strobe_mode, &mpll_param);
	if (ret)
		return ret;

	mpll_func_cntl &= ~MPLL_FUNC_CNTL__BWCTRL_MASK;
	mpll_func_cntl |= mpll_param.bwcntl << MPLL_FUNC_CNTL__BWCTRL__SHIFT;

	mpll_func_cntl_1 &= ~(MPLL_FUNC_CNTL_1__CLKF_MASK |
			      MPLL_FUNC_CNTL_1__CLKFRAC_MASK |
			      MPLL_FUNC_CNTL_1__VCO_MODE_MASK);
	mpll_func_cntl_1 |= (mpll_param.clkf << MPLL_FUNC_CNTL_1__CLKF__SHIFT) |
		(mpll_param.clkfrac << MPLL_FUNC_CNTL_1__CLKFRAC__SHIFT) |
		(mpll_param.vco_mode << MPLL_FUNC_CNTL_1__VCO_MODE__SHIFT);

	mpll_ad_func_cntl &= ~MPLL_AD_FUNC_CNTL__YCLK_POST_DIV_MASK;
	mpll_ad_func_cntl |= mpll_param.post_div << MPLL_AD_FUNC_CNTL__YCLK_POST_DIV__SHIFT;

	if (adev->gmc.vram_type == AMDGPU_VRAM_TYPE_GDDR5) {
		mpll_dq_func_cntl &= ~(MPLL_DQ_FUNC_CNTL__YCLK_SEL_MASK |
				       MPLL_DQ_FUNC_CNTL__YCLK_POST_DIV_MASK);
		mpll_dq_func_cntl |= (mpll_param.yclk_sel << MPLL_DQ_FUNC_CNTL__YCLK_SEL__SHIFT) |
			(mpll_param.post_div << MPLL_DQ_FUNC_CNTL__YCLK_POST_DIV__SHIFT);
	}

	if (data->mclk_ss) {
		struct amdgpu_atom_ss ss;
		uint32_t freq_nom;
		uint32_t tmp;
		uint32_t reference_clock = adev->clock.mpll.reference_freq;

		if (adev->gmc.vram_type == AMDGPU_VRAM_TYPE_GDDR5)
			freq_nom = memory_clock * 4;
		else
			freq_nom = memory_clock * 2;

		tmp = freq_nom / reference_clock;
		tmp = tmp * tmp;
		if (amdgpu_atombios_get_asic_ss_info(adev, &ss,
						     ASIC_INTERNAL_MEMORY_SS, freq_nom)) {
			uint32_t clks = reference_clock * 5 / ss.rate;
			uint32_t clkv = (uint32_t)((((131 * ss.percentage * ss.rate) / 100) * tmp) / freq_nom);

			mpll_ss1 &= ~MPLL_SS1__CLKV_MASK;
			mpll_ss1 |= clkv << MPLL_SS1__CLKV__SHIFT;

			mpll_ss2 &= ~MPLL_SS2__CLKS_MASK;
			mpll_ss2 |= clks << MPLL_SS2__CLKS__SHIFT;
		}
	}

	mclk_pwrmgt_cntl &= ~MCLK_PWRMGT_CNTL__DLL_SPEED_MASK;
	mclk_pwrmgt_cntl |= mpll_param.dll_speed << MCLK_PWRMGT_CNTL__DLL_SPEED__SHIFT;

	if (dll_state_on)
		mclk_pwrmgt_cntl |= MCLK_PWRMGT_CNTL__MRDCK0_PDNB_MASK |
				    MCLK_PWRMGT_CNTL__MRDCK1_PDNB_MASK;
	else
		mclk_pwrmgt_cntl &= ~(MCLK_PWRMGT_CNTL__MRDCK0_PDNB_MASK |
				      MCLK_PWRMGT_CNTL__MRDCK1_PDNB_MASK);

	mclk->mclk_value = cpu_to_be32(memory_clock);
	mclk->vMPLL_FUNC_CNTL = cpu_to_be32(mpll_func_cntl);
	mclk->vMPLL_FUNC_CNTL_1 = cpu_to_be32(mpll_func_cntl_1);
	mclk->vMPLL_FUNC_CNTL_2 = cpu_to_be32(mpll_func_cntl_2);
	mclk->vMPLL_AD_FUNC_CNTL = cpu_to_be32(mpll_ad_func_cntl);
	mclk->vMPLL_DQ_FUNC_CNTL = cpu_to_be32(mpll_dq_func_cntl);
	mclk->vMCLK_PWRMGT_CNTL = cpu_to_be32(mclk_pwrmgt_cntl);
	mclk->vDLL_CNTL = cpu_to_be32(dll_cntl);
	mclk->vMPLL_SS = cpu_to_be32(mpll_ss1);
	mclk->vMPLL_SS2 = cpu_to_be32(mpll_ss2);

	return 0;
}

static void si_populate_smc_sp(struct pp_hwmgr *hwmgr,
			       const struct si_power_state *ps,
			       SISLANDS_SMC_SWSTATE *smc_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	int i;

	for (i = 0; i < ps->performance_level_count - 1; i++)
		smc_state->levels[i].bSP = cpu_to_be32(data->dsp);

	smc_state->levels[ps->performance_level_count - 1].bSP =
		cpu_to_be32(data->psp);
}

static int si_convert_power_level_to_smc(struct pp_hwmgr *hwmgr,
					 const struct si_performance_level *pl,
					 SISLANDS_SMC_HW_PERFORMANCE_LEVEL *level)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	int ret;
	bool dll_state_on;
	uint16_t std_vddc;

	if (data->pcie_performance_request &&
	    (data->force_pcie_gen != SI_PCIE_GEN_INVALID))
		level->gen2PCIE = (uint8_t)data->force_pcie_gen;
	else
		level->gen2PCIE = (uint8_t)pl->pcie_gen;

	ret = si_populate_sclk_value(hwmgr, pl->sclk, &level->sclk);
	if (ret)
		return ret;

	level->mcFlags =  0;

	if (data->mclk_stutter_mode_threshold &&
	    (pl->mclk <= data->mclk_stutter_mode_threshold) &&
	    !data->uvd_enabled &&
	    (si_rreg(hwmgr, mmDPG_PIPE_STUTTER_CONTROL) & DPG_PIPE_STUTTER_CONTROL__STUTTER_ENABLE_MASK) &&
	    (hwmgr->display_config->num_display <= 2)) {
		level->mcFlags |= SISLANDS_SMC_MC_STUTTER_EN;
	}

	if (adev->gmc.vram_type == AMDGPU_VRAM_TYPE_GDDR5) {
		if (pl->mclk > data->mclk_edc_enable_threshold)
			level->mcFlags |= SISLANDS_SMC_MC_EDC_RD_FLAG;

		if (pl->mclk > data->mclk_edc_wr_enable_threshold)
			level->mcFlags |= SISLANDS_SMC_MC_EDC_WR_FLAG;

		level->strobeMode = si_get_strobe_mode_settings(hwmgr, pl->mclk);

		if (level->strobeMode & SISLANDS_SMC_STROBE_ENABLE) {
			if (si_get_mclk_frequency_ratio(pl->mclk, true) >=
			    ((si_rreg(hwmgr, mmMC_SEQ_MISC7) >> 16) & 0xf))
				dll_state_on = ((si_rreg(hwmgr, mmMC_SEQ_MISC5) >> 1) & 0x1) ? true : false;
			else
				dll_state_on = ((si_rreg(hwmgr, mmMC_SEQ_MISC6) >> 1) & 0x1) ? true : false;
		} else {
			dll_state_on = false;
		}
	} else {
		level->strobeMode = si_get_strobe_mode_settings(hwmgr, pl->mclk);

		dll_state_on = ((si_rreg(hwmgr, mmMC_SEQ_MISC5) >> 1) & 0x1) ? true : false;
	}

	ret = si_populate_mclk_value(hwmgr,
				     pl->sclk,
				     pl->mclk,
				     &level->mclk,
				     (level->strobeMode & SISLANDS_SMC_STROBE_ENABLE) != 0, dll_state_on);
	if (ret)
		return ret;

	ret = si_populate_voltage_value(&data->vddc_voltage_table,
					pl->vddc, &level->vddc);
	if (ret)
		return ret;

	ret = si_get_std_voltage_value(hwmgr, &level->vddc, &std_vddc);
	if (ret)
		return ret;

	ret = si_populate_std_voltage_value(std_vddc,
					    level->vddc.index, &level->std_vddc);
	if (ret)
		return ret;

	if (data->vddci_control) {
		ret = si_populate_voltage_value(&data->vddci_voltage_table,
						pl->vddci, &level->vddci);
		if (ret)
			return ret;
	}

	if (data->vddc_phase_shed_control) {
		ret = si_populate_phase_shedding_value(hwmgr->dyn_state.vddc_phase_shed_limits_table,
						       pl->vddc,
						       pl->sclk,
						       pl->mclk,
						       &level->vddc);
		if (ret)
			return ret;
	}

	level->MaxPoweredUpCU = data->max_cu;

	ret = si_populate_mvdd_value(hwmgr, pl->mclk, &level->mvdd);

	return ret;
}

static int si_populate_ulv_state(struct pp_hwmgr *hwmgr,
				 struct SISLANDS_SMC_SWSTATE_SINGLE *state)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_ulv_param *ulv = &data->ulv;
	uint32_t sclk_in_sr = 1350; /* ??? */
	int ret;

	ret = si_convert_power_level_to_smc(hwmgr, &ulv->pl,
					    &state->level);
	if (!ret) {
		if (data->sclk_deep_sleep) {
			if (sclk_in_sr <= SCLK_MIN_DEEPSLEEP_FREQ)
				state->level.stateFlags |= PPSMC_STATEFLAG_DEEPSLEEP_BYPASS;
			else
				state->level.stateFlags |= PPSMC_STATEFLAG_DEEPSLEEP_THROTTLE;
		}
		if (ulv->one_pcie_lane_in_ulv)
			state->flags |= PPSMC_SWSTATE_FLAG_PCIE_X1;
		state->level.arbRefreshState = (uint8_t)(SISLANDS_ULV_STATE_ARB_INDEX);
		state->level.ACIndex = 1;
		state->level.std_vddc = state->level.vddc;
		state->levelCount = 1;

		state->flags |= PPSMC_SWSTATE_FLAG_DC;
	}

	return ret;
}

static int si_program_ulv_memory_timing_parameters(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct si_ulv_param *ulv = &data->ulv;
	SMC_SIslands_MCArbDramTimingRegisterSet arb_regs = { 0 };
	int ret;

	ret = si_populate_memory_timing_parameters(hwmgr, &ulv->pl,
						   &arb_regs);
	if (ret)
		return ret;

	si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_ulv_volt_change_delay,
				   ulv->volt_change_delay);

	ret = si_smc_copy_bytes(hwmgr,
				smu_data->arb_table_start +
				offsetof(SMC_SIslands_MCArbDramTimingRegisters, data) +
				sizeof(SMC_SIslands_MCArbDramTimingRegisterSet) * SISLANDS_ULV_STATE_ARB_INDEX,
				(uint8_t *)&arb_regs,
				sizeof(SMC_SIslands_MCArbDramTimingRegisterSet),
				smu_data->sram_end);

	return ret;
}

/* pp_smumgr_func.init_smc_table (legacy si_init_smc_table). Builds the
 * whole SISLANDS_SMC_STATETABLE around hwmgr->boot_ps and uploads it.
 */
static int si_init_smc_table(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct amdgpu_device *adev = hwmgr->adev;
	const struct si_power_state *boot_state;
	const struct si_ulv_param *ulv = &data->ulv;
	SISLANDS_SMC_STATETABLE  *table = &smu_data->smc_statetable;
	int ret;
	uint32_t lane_width;
	uint32_t vr_hot_gpio;

	if (!hwmgr->boot_ps)
		return -EINVAL;
	boot_state = cast_const_si_ps(&hwmgr->boot_ps->hardware);
	if (!boot_state || boot_state->performance_level_count == 0)
		return -EINVAL;

	memset(table, 0, sizeof(*table));

	si_populate_smc_voltage_tables(hwmgr, table);

	switch (hwmgr->thermal_controller.ucType) {
	case ATOM_PP_THERMALCONTROLLER_SISLANDS:
	case ATOM_PP_THERMALCONTROLLER_EMC2103_WITH_INTERNAL:
		table->thermalProtectType = PPSMC_THERMAL_PROTECT_TYPE_INTERNAL;
		break;
	case ATOM_PP_THERMALCONTROLLER_NONE:
		table->thermalProtectType = PPSMC_THERMAL_PROTECT_TYPE_NONE;
		break;
	default:
		table->thermalProtectType = PPSMC_THERMAL_PROTECT_TYPE_EXTERNAL;
		break;
	}

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			    PHM_PlatformCaps_AutomaticDCTransition))
		table->systemFlags |= PPSMC_SYSTEMFLAG_GPIO_DC;

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			    PHM_PlatformCaps_RegulatorHot)) {
		if ((adev->pdev->device != 0x6818) && (adev->pdev->device != 0x6819))
			table->systemFlags |= PPSMC_SYSTEMFLAG_REGULATOR_HOT;
	}

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			    PHM_PlatformCaps_StepVddc))
		table->systemFlags |= PPSMC_SYSTEMFLAG_STEPVDDC;

	if (adev->gmc.vram_type == AMDGPU_VRAM_TYPE_GDDR5)
		table->systemFlags |= PPSMC_SYSTEMFLAG_GDDR5;

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			    PHM_PlatformCaps_RevertGPIO5Polarity))
		table->extraFlags |= PPSMC_EXTRAFLAGS_AC2DC_GPIO5_POLARITY_HIGH;

	if (phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			    PHM_PlatformCaps_VRHotGPIOConfigurable)) {
		table->systemFlags |= PPSMC_SYSTEMFLAG_REGULATOR_HOT_PROG_GPIO;
		vr_hot_gpio = data->backbias_response_time;
		si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_vr_hot_gpio,
					   vr_hot_gpio);
	}

	ret = si_populate_smc_initial_state(hwmgr, boot_state, table);
	if (ret)
		return ret;

	ret = si_populate_smc_acpi_state(hwmgr, table);
	if (ret)
		return ret;

	table->driverState.flags = table->initialState.flags;
	table->driverState.levelCount = table->initialState.levelCount;
	table->driverState.levels[0] = table->initialState.level;

	ret = si_do_program_memory_timing_parameters(hwmgr, boot_state,
						     SISLANDS_INITIAL_STATE_ARB_INDEX);
	if (ret)
		return ret;

	if (ulv->supported && ulv->pl.vddc) {
		ret = si_populate_ulv_state(hwmgr, &table->ULVState);
		if (ret)
			return ret;

		ret = si_program_ulv_memory_timing_parameters(hwmgr);
		if (ret)
			return ret;

		si_wreg(hwmgr, mmCG_ULV_CONTROL, ulv->cg_ulv_control);
		si_wreg(hwmgr, mmCG_ULV_PARAMETER, ulv->cg_ulv_parameter);

		lane_width = amdgpu_get_pcie_lanes(adev);
		si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_non_ulv_pcie_link_width, lane_width);
	} else {
		table->ULVState = table->initialState;
	}

	return si_smc_copy_bytes(hwmgr, smu_data->state_table_start,
				 (uint8_t *)table, sizeof(SISLANDS_SMC_STATETABLE),
				 smu_data->sram_end);
}

/* ---- SPLL divider table ---- */

int si_smc_init_spll_table(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	SMC_SISLANDS_SPLL_DIV_TABLE *spll_table;
	SISLANDS_SMC_SCLK_VALUE sclk_params;
	uint32_t fb_div, p_div;
	uint32_t clk_s, clk_v;
	uint32_t sclk = 0;
	int ret = 0;
	uint32_t tmp;
	int i;

	if (smu_data->spll_table_start == 0)
		return -EINVAL;

	spll_table = kzalloc(sizeof(SMC_SISLANDS_SPLL_DIV_TABLE), GFP_KERNEL);
	if (spll_table == NULL)
		return -ENOMEM;

	for (i = 0; i < 256; i++) {
		ret = si_calculate_sclk_params(hwmgr, sclk, &sclk_params);
		if (ret)
			break;
		p_div = (sclk_params.vCG_SPLL_FUNC_CNTL & CG_SPLL_FUNC_CNTL__SPLL_PDIV_A_MASK) >>
			CG_SPLL_FUNC_CNTL__SPLL_PDIV_A__SHIFT;
		fb_div = (sclk_params.vCG_SPLL_FUNC_CNTL_3 & CG_SPLL_FUNC_CNTL_3__SPLL_FB_DIV_MASK) >>
			CG_SPLL_FUNC_CNTL_3__SPLL_FB_DIV__SHIFT;
		clk_s = (sclk_params.vCG_SPLL_SPREAD_SPECTRUM & CG_SPLL_SPREAD_SPECTRUM__CLK_S_MASK) >>
			CG_SPLL_SPREAD_SPECTRUM__CLK_S__SHIFT;
		clk_v = (sclk_params.vCG_SPLL_SPREAD_SPECTRUM_2 & CG_SPLL_SPREAD_SPECTRUM_2__CLK_V_MASK) >>
			CG_SPLL_SPREAD_SPECTRUM_2__CLK_V__SHIFT;

		fb_div &= ~0x00001FFF;
		fb_div >>= 1;
		clk_v >>= 6;

		if (p_div & ~(SMC_SISLANDS_SPLL_DIV_TABLE_PDIV_MASK >> SMC_SISLANDS_SPLL_DIV_TABLE_PDIV_SHIFT))
			ret = -EINVAL;
		if (fb_div & ~(SMC_SISLANDS_SPLL_DIV_TABLE_FBDIV_MASK >> SMC_SISLANDS_SPLL_DIV_TABLE_FBDIV_SHIFT))
			ret = -EINVAL;
		if (clk_s & ~(SMC_SISLANDS_SPLL_DIV_TABLE_CLKS_MASK >> SMC_SISLANDS_SPLL_DIV_TABLE_CLKS_SHIFT))
			ret = -EINVAL;
		if (clk_v & ~(SMC_SISLANDS_SPLL_DIV_TABLE_CLKV_MASK >> SMC_SISLANDS_SPLL_DIV_TABLE_CLKV_SHIFT))
			ret = -EINVAL;

		if (ret)
			break;

		tmp = ((fb_div << SMC_SISLANDS_SPLL_DIV_TABLE_FBDIV_SHIFT) & SMC_SISLANDS_SPLL_DIV_TABLE_FBDIV_MASK) |
			((p_div << SMC_SISLANDS_SPLL_DIV_TABLE_PDIV_SHIFT) & SMC_SISLANDS_SPLL_DIV_TABLE_PDIV_MASK);
		spll_table->freq[i] = cpu_to_be32(tmp);

		tmp = ((clk_v << SMC_SISLANDS_SPLL_DIV_TABLE_CLKV_SHIFT) & SMC_SISLANDS_SPLL_DIV_TABLE_CLKV_MASK) |
			((clk_s << SMC_SISLANDS_SPLL_DIV_TABLE_CLKS_SHIFT) & SMC_SISLANDS_SPLL_DIV_TABLE_CLKS_MASK);
		spll_table->ss[i] = cpu_to_be32(tmp);

		sclk += 512;
	}

	if (!ret)
		ret = si_smc_copy_bytes(hwmgr, smu_data->spll_table_start,
					(uint8_t *)spll_table,
					sizeof(SMC_SISLANDS_SPLL_DIV_TABLE),
					smu_data->sram_end);

	if (ret)
		data->enable_power_containment = false;

	kfree(spll_table);

	return ret;
}

/* ---- power containment / SQ ramping (per driver state) ---- */

static uint32_t si_get_smc_power_scaling_factor(struct pp_hwmgr *hwmgr)
{
	return 1;
}

static uint32_t si_scale_power_for_smc(uint32_t power_in_watts, uint32_t scaling_factor)
{
	return power_in_watts;
}

static uint16_t si_calculate_power_efficiency_ratio(const uint16_t prev_std_vddc,
						    const uint16_t curr_std_vddc)
{
	uint64_t margin = (uint64_t)SISLANDS_DPM2_PWREFFICIENCYRATIO_MARGIN;
	uint64_t prev_vddc = (uint64_t)prev_std_vddc;
	uint64_t curr_vddc = (uint64_t)curr_std_vddc;
	uint64_t pwr_efficiency_ratio, n, d;

	if ((prev_vddc == 0) || (curr_vddc == 0))
		return 0;

	n = div64_u64((uint64_t)1024 * curr_vddc * curr_vddc * ((uint64_t)1000 + margin), (uint64_t)1000);
	d = prev_vddc * prev_vddc;
	pwr_efficiency_ratio = div64_u64(n, d);

	if (pwr_efficiency_ratio > (uint64_t)0xFFFF)
		return 0;

	return (uint16_t)pwr_efficiency_ratio;
}

static bool si_should_disable_uvd_powertune(struct pp_hwmgr *hwmgr,
					    const struct si_power_state *ps)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (data->dyn_powertune_data.disable_uvd_powertune &&
	    ps->vclk && ps->dclk)
		return true;

	return false;
}

static int si_populate_power_containment_values(struct pp_hwmgr *hwmgr,
						const struct si_power_state *state,
						SISLANDS_SMC_SWSTATE *smc_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	SISLANDS_SMC_VOLTAGE_VALUE vddc;
	uint32_t prev_sclk;
	uint32_t max_sclk;
	uint32_t min_sclk;
	uint16_t prev_std_vddc;
	uint16_t curr_std_vddc;
	int i;
	uint16_t pwr_efficiency_ratio;
	uint8_t max_ps_percent;
	bool disable_uvd_power_tune;
	int ret;

	if (data->enable_power_containment == false)
		return 0;

	if (state->performance_level_count == 0)
		return -EINVAL;

	if (smc_state->levelCount != state->performance_level_count)
		return -EINVAL;

	disable_uvd_power_tune = si_should_disable_uvd_powertune(hwmgr, state);

	smc_state->levels[0].dpm2.MaxPS = 0;
	smc_state->levels[0].dpm2.NearTDPDec = 0;
	smc_state->levels[0].dpm2.AboveSafeInc = 0;
	smc_state->levels[0].dpm2.BelowSafeInc = 0;
	smc_state->levels[0].dpm2.PwrEfficiencyRatio = 0;

	for (i = 1; i < state->performance_level_count; i++) {
		prev_sclk = state->performance_levels[i-1].sclk;
		max_sclk  = state->performance_levels[i].sclk;
		if (i == 1)
			max_ps_percent = SISLANDS_DPM2_MAXPS_PERCENT_M;
		else
			max_ps_percent = SISLANDS_DPM2_MAXPS_PERCENT_H;

		if (prev_sclk > max_sclk)
			return -EINVAL;

		if ((max_ps_percent == 0) ||
		    (prev_sclk == max_sclk) ||
		    disable_uvd_power_tune)
			min_sclk = max_sclk;
		else if (i == 1)
			min_sclk = prev_sclk;
		else
			min_sclk = (prev_sclk * (uint32_t)max_ps_percent) / 100;

		if (min_sclk < state->performance_levels[0].sclk)
			min_sclk = state->performance_levels[0].sclk;

		if (min_sclk == 0)
			return -EINVAL;

		ret = si_populate_voltage_value(&data->vddc_voltage_table,
						state->performance_levels[i-1].vddc, &vddc);
		if (ret)
			return ret;

		ret = si_get_std_voltage_value(hwmgr, &vddc, &prev_std_vddc);
		if (ret)
			return ret;

		ret = si_populate_voltage_value(&data->vddc_voltage_table,
						state->performance_levels[i].vddc, &vddc);
		if (ret)
			return ret;

		ret = si_get_std_voltage_value(hwmgr, &vddc, &curr_std_vddc);
		if (ret)
			return ret;

		pwr_efficiency_ratio = si_calculate_power_efficiency_ratio(prev_std_vddc, curr_std_vddc);

		smc_state->levels[i].dpm2.MaxPS = (uint8_t)((SISLANDS_DPM2_MAX_PULSE_SKIP * (max_sclk - min_sclk)) / max_sclk);
		smc_state->levels[i].dpm2.NearTDPDec = SISLANDS_DPM2_NEAR_TDP_DEC;
		smc_state->levels[i].dpm2.AboveSafeInc = SISLANDS_DPM2_ABOVE_SAFE_INC;
		smc_state->levels[i].dpm2.BelowSafeInc = SISLANDS_DPM2_BELOW_SAFE_INC;
		smc_state->levels[i].dpm2.PwrEfficiencyRatio = cpu_to_be16(pwr_efficiency_ratio);
	}

	return 0;
}

static int si_populate_sq_ramping_values(struct pp_hwmgr *hwmgr,
					 const struct si_power_state *state,
					 SISLANDS_SMC_SWSTATE *smc_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t sq_power_throttle, sq_power_throttle2;
	bool enable_sq_ramping = data->enable_sq_ramping;
	int i;

	if (state->performance_level_count == 0)
		return -EINVAL;

	if (smc_state->levelCount != state->performance_level_count)
		return -EINVAL;

	if (hwmgr->platform_descriptor.SQRampingThreshold == 0)
		return -EINVAL;

	if (SISLANDS_DPM2_SQ_RAMP_MAX_POWER > (SQ_POWER_THROTTLE__MAX_POWER_MASK >> SQ_POWER_THROTTLE__MAX_POWER__SHIFT))
		enable_sq_ramping = false;

	if (SISLANDS_DPM2_SQ_RAMP_MIN_POWER > (SQ_POWER_THROTTLE__MIN_POWER_MASK >> SQ_POWER_THROTTLE__MIN_POWER__SHIFT))
		enable_sq_ramping = false;

	if (SISLANDS_DPM2_SQ_RAMP_MAX_POWER_DELTA > (SQ_POWER_THROTTLE2__MAX_POWER_DELTA_MASK >> SQ_POWER_THROTTLE2__MAX_POWER_DELTA__SHIFT))
		enable_sq_ramping = false;

	if (SISLANDS_DPM2_SQ_RAMP_STI_SIZE > (SQ_POWER_THROTTLE2__SHORT_TERM_INTERVAL_SIZE_MASK >> SQ_POWER_THROTTLE2__SHORT_TERM_INTERVAL_SIZE__SHIFT))
		enable_sq_ramping = false;

	if (SISLANDS_DPM2_SQ_RAMP_LTI_RATIO > (SQ_POWER_THROTTLE2__LONG_TERM_INTERVAL_RATIO_MASK >> SQ_POWER_THROTTLE2__LONG_TERM_INTERVAL_RATIO__SHIFT))
		enable_sq_ramping = false;

	for (i = 0; i < state->performance_level_count; i++) {
		sq_power_throttle = 0;
		sq_power_throttle2 = 0;

		if ((state->performance_levels[i].sclk >= hwmgr->platform_descriptor.SQRampingThreshold) &&
		    enable_sq_ramping) {
			sq_power_throttle |= SISLANDS_DPM2_SQ_RAMP_MAX_POWER << SQ_POWER_THROTTLE__MAX_POWER__SHIFT;
			sq_power_throttle |= SISLANDS_DPM2_SQ_RAMP_MIN_POWER << SQ_POWER_THROTTLE__MIN_POWER__SHIFT;
			sq_power_throttle2 |= SISLANDS_DPM2_SQ_RAMP_MAX_POWER_DELTA << SQ_POWER_THROTTLE2__MAX_POWER_DELTA__SHIFT;
			sq_power_throttle2 |= SISLANDS_DPM2_SQ_RAMP_STI_SIZE << SQ_POWER_THROTTLE2__SHORT_TERM_INTERVAL_SIZE__SHIFT;
			sq_power_throttle2 |= SISLANDS_DPM2_SQ_RAMP_LTI_RATIO << SQ_POWER_THROTTLE2__LONG_TERM_INTERVAL_RATIO__SHIFT;
		} else {
			sq_power_throttle |= SQ_POWER_THROTTLE__MAX_POWER_MASK |
					     SQ_POWER_THROTTLE__MIN_POWER_MASK;
			sq_power_throttle2 |= SQ_POWER_THROTTLE2__MAX_POWER_DELTA_MASK |
					      SQ_POWER_THROTTLE2__SHORT_TERM_INTERVAL_SIZE_MASK |
					      SQ_POWER_THROTTLE2__LONG_TERM_INTERVAL_RATIO_MASK;
		}

		smc_state->levels[i].SQPowerThrottle = cpu_to_be32(sq_power_throttle);
		smc_state->levels[i].SQPowerThrottle_2 = cpu_to_be32(sq_power_throttle2);
	}

	return 0;
}

/* ---- driver (sw) state ---- */

/* The activity target the up and down thresholds are built around: the
 * VBIOS-derived value for this pair, scaled by the active profile.
 */
static uint32_t si_target_activity(struct pp_hwmgr *hwmgr, int pair)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t base = (50 / SI_MAX_HARDWARE_POWERLEVELS) * 100 * (pair + 1);
	uint32_t pct = data->profile[hwmgr->power_profile_mode].activity;

	return pct ? base * pct / 100 : base;
}

/* legacy r600_calculate_at() */
static int si_calculate_at(uint32_t t, uint32_t h, uint32_t fh, uint32_t fl,
			   uint32_t *tl, uint32_t *th)
{
	uint32_t k, a, ah, al;
	uint32_t t1;

	if ((fl == 0) || (fh == 0) || (fl > fh))
		return -EINVAL;

	k = (100 * fh) / fl;
	t1 = (t * (k - 100));
	a = (1000 * (100 * h + t1)) / (10000 + (t1 / 100));
	a = (a + 5) / 10;
	ah = ((a * t) + 5000) / 10000;
	al = a - ah;

	*th = t - ah;
	*tl = t + al;

	return 0;
}

static int si_populate_smc_t(struct pp_hwmgr *hwmgr,
			     const struct si_power_state *state,
			     SISLANDS_SMC_SWSTATE *smc_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct si_activity_profile *profile =
		&data->profile[hwmgr->power_profile_mode];
	uint32_t a_t;
	uint32_t t_l, t_h;
	uint32_t high_bsp;
	int i, ret;

	if (state->performance_level_count >= 9)
		return -EINVAL;

	if (state->performance_level_count < 2) {
		a_t = 0xffff << CG_AT__CG_R__SHIFT | 0 << CG_AT__CG_L__SHIFT;
		smc_state->levels[0].aT = cpu_to_be32(a_t);
		return 0;
	}

	smc_state->levels[0].aT = cpu_to_be32(0);

	for (i = 0; i <= state->performance_level_count - 2; i++) {
		bool last = (i == state->performance_level_count - 2);

		if (last && state->collapsed_next_sclk &&
		    state->collapsed_below_top_sclk &&
		    state->vbios_level_count > state->performance_level_count) {
			/* A ladder quirk removed the levels between this one and
			 * the top, so the SMC has one big step where the VBIOS
			 * ladder had several small ones. The same desktop work
			 * looks fh/fl times busier at the low clock than at the
			 * top, so the up threshold must be wide enough to keep a
			 * desktop parked low: compute it from the actual pair
			 * (300 -> 1000: ~30% busy) at the top pair's index. The
			 * down threshold keeps the original top level's (from the
			 * VBIOS pair that ended at the top, ~13%), so the SMC
			 * returns to idle exactly as the full ladder did.
			 */
			uint32_t up_l, up_h, dn_l, dn_h;
			int ti = state->vbios_level_count - 2;

			if (si_calculate_at(si_target_activity(hwmgr, ti),
					    100 * profile->up_hyst,
					    state->performance_levels[i + 1].sclk,
					    state->performance_levels[i].sclk,
					    &up_l, &up_h))
				up_l = (ti + 1) * 1000 + 50 * profile->up_hyst;
			if (si_calculate_at(si_target_activity(hwmgr, ti),
					    100 * profile->down_hyst,
					    state->performance_levels[i + 1].sclk,
					    state->collapsed_below_top_sclk,
					    &dn_l, &dn_h))
				dn_h = (ti + 1) * 1000 - 50 * profile->down_hyst;
			t_l = up_l;
			t_h = dn_h;
			ret = 0;
		} else if (profile->up_hyst != profile->down_hyst) {
			uint32_t up_l, up_h, dn_l, dn_h;

			/* the two thresholds come from one call only while the
			 * profile is symmetric; otherwise each gets its own
			 */
			ret = si_calculate_at(si_target_activity(hwmgr, i),
					      100 * profile->up_hyst,
					      state->performance_levels[i + 1].sclk,
					      state->performance_levels[i].sclk,
					      &up_l, &up_h);
			if (!ret)
				ret = si_calculate_at(si_target_activity(hwmgr, i),
						      100 * profile->down_hyst,
						      state->performance_levels[i + 1].sclk,
						      state->performance_levels[i].sclk,
						      &dn_l, &dn_h);
			t_l = up_l;
			t_h = dn_h;
		} else {
			ret = si_calculate_at(
				si_target_activity(hwmgr, i),
				100 * profile->up_hyst,
				state->performance_levels[i + 1].sclk,
				state->performance_levels[i].sclk,
				&t_l,
				&t_h);
		}

		if (ret) {
			t_h = (i + 1) * 1000 - 50 * profile->down_hyst;
			t_l = (i + 1) * 1000 + 50 * profile->up_hyst;
		}

		a_t = be32_to_cpu(smc_state->levels[i].aT) & ~CG_AT__CG_R_MASK;
		a_t |= (t_l * data->bsp / 20000) << CG_AT__CG_R__SHIFT;
		smc_state->levels[i].aT = cpu_to_be32(a_t);

		high_bsp = (i == state->performance_level_count - 2) ?
			data->pbsp : data->bsp;
		a_t = (0xffff) << CG_AT__CG_R__SHIFT | (t_h * high_bsp / 20000) << CG_AT__CG_L__SHIFT;
		smc_state->levels[i + 1].aT = cpu_to_be32(a_t);

		pr_debug("si smc thresholds pair %d: %u -> %u MHz, target %u, up_hyst %u down_hyst %u, t_l %u t_h %u%s\n",
			 i, state->performance_levels[i].sclk / 100,
			 state->performance_levels[i + 1].sclk / 100,
			 si_target_activity(hwmgr, last ? state->vbios_level_count - 2 : i),
			 profile->up_hyst, profile->down_hyst, t_l, t_h,
			 (last && state->collapsed_next_sclk &&
			  state->collapsed_below_top_sclk) ? " (collapsed)" : "");
	}

	return 0;
}

static int si_convert_power_state_to_smc(struct pp_hwmgr *hwmgr,
					 const struct si_power_state *state,
					 SISLANDS_SMC_SWSTATE *smc_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	int i, ret;
	uint32_t threshold;
	uint32_t sclk_in_sr = 1350; /* ??? */

	if (state->performance_level_count > SI_MAX_HARDWARE_POWERLEVELS)
		return -EINVAL;

	threshold = state->performance_levels[state->performance_level_count-1].sclk * 100 / 100;

	if (state->vclk && state->dclk) {
		data->uvd_enabled = true;
		if (data->smu_uvd_hs)
			smc_state->flags |= PPSMC_SWSTATE_FLAG_UVD;
	} else {
		data->uvd_enabled = false;
	}

	if (state->dc_compatible)
		smc_state->flags |= PPSMC_SWSTATE_FLAG_DC;

	smc_state->levelCount = 0;
	for (i = 0; i < state->performance_level_count; i++) {
		if (data->sclk_deep_sleep) {
			if ((i == 0) || data->sclk_deep_sleep_above_low) {
				if (sclk_in_sr <= SCLK_MIN_DEEPSLEEP_FREQ)
					smc_state->levels[i].stateFlags |= PPSMC_STATEFLAG_DEEPSLEEP_BYPASS;
				else
					smc_state->levels[i].stateFlags |= PPSMC_STATEFLAG_DEEPSLEEP_THROTTLE;
			}
		}

		ret = si_convert_power_level_to_smc(hwmgr, &state->performance_levels[i],
						    &smc_state->levels[i]);
		smc_state->levels[i].arbRefreshState =
			(uint8_t)(SISLANDS_DRIVER_STATE_ARB_INDEX + i);

		if (ret)
			return ret;

		if (data->enable_power_containment)
			smc_state->levels[i].displayWatermark =
				(state->performance_levels[i].sclk < threshold) ?
				PPSMC_DISPLAY_WATERMARK_LOW : PPSMC_DISPLAY_WATERMARK_HIGH;
		else
			smc_state->levels[i].displayWatermark = (i < 2) ?
				PPSMC_DISPLAY_WATERMARK_LOW : PPSMC_DISPLAY_WATERMARK_HIGH;

		if (data->dynamic_ac_timing)
			smc_state->levels[i].ACIndex = SISLANDS_MCREGISTERTABLE_FIRST_DRIVERSTATE_SLOT + i;
		else
			smc_state->levels[i].ACIndex = 0;

		smc_state->levelCount++;
	}

	si_smc_write_soft_register(hwmgr,
				   SI_SMC_SOFT_REGISTER_watermark_threshold,
				   threshold / 512);

	si_populate_smc_sp(hwmgr, state, smc_state);

	ret = si_populate_power_containment_values(hwmgr, state, smc_state);
	if (ret)
		data->enable_power_containment = false;

	ret = si_populate_sq_ramping_values(hwmgr, state, smc_state);
	if (ret)
		data->enable_sq_ramping = false;

	return si_populate_smc_t(hwmgr, state, smc_state);
}

int si_smc_upload_sw_state(struct pp_hwmgr *hwmgr,
			   const struct si_power_state *new_state)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	int ret;
	uint32_t address = smu_data->state_table_start +
		offsetof(SISLANDS_SMC_STATETABLE, driverState);
	SISLANDS_SMC_SWSTATE *smc_state = &smu_data->smc_statetable.driverState;
	size_t state_size;

	if (new_state->performance_level_count == 0 ||
	    new_state->performance_level_count > SI_MAX_HARDWARE_POWERLEVELS)
		return -EINVAL;

	/* driverState's flexible levels[] run into the dpmLevels[] that
	 * follow it in SISLANDS_SMC_STATETABLE, exactly as the legacy
	 * code relies on.
	 */
	state_size = struct_size(smc_state, levels,
				 new_state->performance_level_count);
	memset(smc_state, 0, state_size);

	ret = si_convert_power_state_to_smc(hwmgr, new_state, smc_state);
	if (ret)
		return ret;

	return si_smc_copy_bytes(hwmgr, address, (uint8_t *)smc_state,
				 state_size, smu_data->sram_end);
}

int si_smc_upload_ulv_state(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct si_ulv_param *ulv = &data->ulv;
	int ret = 0;

	if (ulv->supported && ulv->pl.vddc) {
		uint32_t address = smu_data->state_table_start +
			offsetof(SISLANDS_SMC_STATETABLE, ULVState);
		struct SISLANDS_SMC_SWSTATE_SINGLE *smc_state = &smu_data->smc_statetable.ULVState;
		uint32_t state_size = sizeof(struct SISLANDS_SMC_SWSTATE_SINGLE);

		memset(smc_state, 0, state_size);

		ret = si_populate_ulv_state(hwmgr, smc_state);
		if (!ret)
			ret = si_smc_copy_bytes(hwmgr, address, (uint8_t *)smc_state,
						state_size, smu_data->sram_end);
	}

	return ret;
}

int si_smc_upload_smc_data(struct pp_hwmgr *hwmgr)
{
	const struct amd_pp_display_configuration *cfg = hwmgr->display_config;
	uint32_t crtc_index = 0;
	uint32_t mclk_change_block_cp_min = 0;
	uint32_t mclk_change_block_cp_max = 0;

	/* When a display is plugged in, program these so that the SMC
	 * performs MCLK switching when it doesn't cause flickering.
	 * When no display is plugged in, there is no need to restrict
	 * MCLK switching, so program them to zero.
	 */
	if (cfg->num_display) {
		crtc_index = cfg->crtc_index;

		if (cfg->line_time_in_us) {
			mclk_change_block_cp_min = 200 / cfg->line_time_in_us;
			mclk_change_block_cp_max = 100 / cfg->line_time_in_us;
		}
	}

	si_smc_write_soft_register(hwmgr,
				   SI_SMC_SOFT_REGISTER_crtc_index,
				   crtc_index);

	si_smc_write_soft_register(hwmgr,
				   SI_SMC_SOFT_REGISTER_mclk_change_block_cp_min,
				   mclk_change_block_cp_min);

	si_smc_write_soft_register(hwmgr,
				   SI_SMC_SOFT_REGISTER_mclk_change_block_cp_max,
				   mclk_change_block_cp_max);

	return 0;
}

void si_smc_program_response_times(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t voltage_response_time, acpi_delay_time, vbi_time_out;
	uint32_t vddc_dly, acpi_dly, vbi_dly;
	uint32_t reference_clock;

	si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_mvdd_chg_time, 1);

	voltage_response_time = data->voltage_response_time;

	if (voltage_response_time == 0)
		voltage_response_time = 1000;

	/* amdgpu.si_vreg_delay_us: the regulator settle time the SMC honours
	 * before every sclk switch that follows a VDDC rise (the 900->1150 mV
	 * step between levels 1 and 2 on Oland is the one that bites)
	 */
	if (amdgpu_si_vreg_delay_us) {
		voltage_response_time = amdgpu_si_vreg_delay_us;
		dev_info(adev->dev, "si: VDDC settle delay overridden to %u us\n",
			 voltage_response_time);
	}

	acpi_delay_time = 15000;
	vbi_time_out = 100000;

	reference_clock = amdgpu_asic_get_xclk(adev);

	pr_debug("si vreg delay: vbios %u us, effective %u us, refclk %u, vddc_dly %u\n",
		 data->voltage_response_time, voltage_response_time, reference_clock,
		 (voltage_response_time * reference_clock) / 100);

	vddc_dly = (voltage_response_time  * reference_clock) / 100;
	acpi_dly = (acpi_delay_time * reference_clock) / 100;
	vbi_dly  = (vbi_time_out * reference_clock) / 100;

	si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_delay_vreg,  vddc_dly);
	si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_delay_acpi,  acpi_dly);
	si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_mclk_chg_timeout, vbi_dly);
	si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_mc_block_delay, 0xAA);
}

/* ---- TDP limits (Oland OD TDP lives here) ---- */

static int si_calculate_adjusted_tdp_limits(struct pp_hwmgr *hwmgr,
					    bool adjust_polarity,
					    uint32_t tdp_adjustment,
					    uint32_t *tdp_limit,
					    uint32_t *near_tdp_limit)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct phm_platform_descriptor *pd = &hwmgr->platform_descriptor;
	uint32_t adjustment_delta, max_tdp_limit;

	if (tdp_adjustment > data->tdp_od_limit)
		return -EINVAL;

	max_tdp_limit = ((100 + 100) * pd->TDPLimit) / 100;

	if (adjust_polarity) {
		/* Round to nearest so the programmed SMC limit matches the
		 * requested gain (floor division would systematically
		 * under-deliver by up to ~1W).
		 */
		*tdp_limit = DIV_ROUND_CLOSEST((100 + tdp_adjustment) *
					       pd->TDPLimit, 100);
		*near_tdp_limit = pd->nearTDPLimitAdjusted + (*tdp_limit - pd->TDPLimit);
	} else {
		*tdp_limit = ((100 - tdp_adjustment) * pd->TDPLimit) / 100;
		adjustment_delta  = pd->TDPLimit - *tdp_limit;
		if (adjustment_delta < pd->nearTDPLimitAdjusted)
			*near_tdp_limit = pd->nearTDPLimitAdjusted - adjustment_delta;
		else
			*near_tdp_limit = 0;
	}

	if ((*tdp_limit <= 0) || (*tdp_limit > max_tdp_limit))
		return -EINVAL;
	if ((*near_tdp_limit <= 0) || (*near_tdp_limit > *tdp_limit))
		return -EINVAL;

	return 0;
}

int si_smc_populate_smc_tdp_limits(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct amdgpu_device *adev = hwmgr->adev;

	if (data->enable_power_containment) {
		SISLANDS_SMC_STATETABLE *smc_table = &smu_data->smc_statetable;
		PP_SIslands_PAPMParameters *papm_parm;
		const struct phm_ppm_table *ppm = hwmgr->dyn_state.ppm_parameter_table;
		uint32_t scaling_factor = si_get_smc_power_scaling_factor(hwmgr);
		uint32_t tdp_limit;
		uint32_t near_tdp_limit;
		int ret;

		if (scaling_factor == 0)
			return -EINVAL;

		ret = si_calculate_adjusted_tdp_limits(hwmgr,
						       hwmgr->chip_id == CHIP_OLAND,
						       data->od_tdp,
						       &tdp_limit,
						       &near_tdp_limit);
		if (ret)
			return ret;

		if (adev->pdev->device == 0x6611 && adev->pdev->revision == 0x87) {
			/* Workaround buggy powertune on Radeon 430 and 520. */
			tdp_limit = 32;
			near_tdp_limit = 28;
		}

		smc_table->dpm2Params.TDPLimit =
			cpu_to_be32(si_scale_power_for_smc(tdp_limit, scaling_factor) * 1000);
		smc_table->dpm2Params.NearTDPLimit =
			cpu_to_be32(si_scale_power_for_smc(near_tdp_limit, scaling_factor) * 1000);
		smc_table->dpm2Params.SafePowerLimit =
			cpu_to_be32(si_scale_power_for_smc((near_tdp_limit * SISLANDS_DPM2_TDP_SAFE_LIMIT_PERCENT) / 100, scaling_factor) * 1000);

		ret = si_smc_copy_bytes(hwmgr,
					(smu_data->state_table_start + offsetof(SISLANDS_SMC_STATETABLE, dpm2Params) +
					 offsetof(PP_SIslands_DPM2Parameters, TDPLimit)),
					(uint8_t *)(&(smc_table->dpm2Params.TDPLimit)),
					sizeof(uint32_t) * 3,
					smu_data->sram_end);
		if (ret)
			return ret;

		if (data->enable_ppm && ppm) {
			papm_parm = &smu_data->papm_parm;
			memset(papm_parm, 0, sizeof(PP_SIslands_PAPMParameters));
			papm_parm->NearTDPLimitTherm = cpu_to_be32(ppm->dgpu_tdp);
			papm_parm->dGPU_T_Limit = cpu_to_be32(ppm->tj_max);
			papm_parm->dGPU_T_Warning = cpu_to_be32(95);
			papm_parm->dGPU_T_Hysteresis = cpu_to_be32(5);
			papm_parm->PlatformPowerLimit = 0xffffffff;
			papm_parm->NearTDPLimitPAPM = 0xffffffff;

			ret = si_smc_copy_bytes(hwmgr, smu_data->papm_cfg_table_start,
						(uint8_t *)papm_parm,
						sizeof(PP_SIslands_PAPMParameters),
						smu_data->sram_end);
			if (ret)
				return ret;
		}
	}
	return 0;
}

int si_smc_populate_smc_tdp_limits_2(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;

	if (data->enable_power_containment) {
		SISLANDS_SMC_STATETABLE *smc_table = &smu_data->smc_statetable;
		int ret;

		ret = si_smc_copy_bytes(hwmgr,
					(smu_data->state_table_start +
					 offsetof(SISLANDS_SMC_STATETABLE, dpm2Params) +
					 offsetof(PP_SIslands_DPM2Parameters, NearTDPLimit)),
					(uint8_t *)(&(smc_table->dpm2Params.NearTDPLimit)),
					sizeof(uint32_t) * 2,
					smu_data->sram_end);
		if (ret)
			return ret;
	}

	return 0;
}

/* ---- DTE / CAC tables ---- */

int si_smc_initialize_smc_dte_tables(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	int ret = 0;
	struct si_dte_data *dte_data = &data->dte_data;
	Smc_SIslands_DTE_Configuration *dte_tables = NULL;
	uint32_t table_size;
	uint8_t tdep_count;
	uint32_t i;

	if (data->enable_dte == false)
		return 0;

	if (dte_data->k <= 0)
		return -EINVAL;

	dte_tables = kzalloc(sizeof(Smc_SIslands_DTE_Configuration), GFP_KERNEL);
	if (dte_tables == NULL) {
		data->enable_dte = false;
		return -ENOMEM;
	}

	table_size = dte_data->k;

	if (table_size > SMC_SISLANDS_DTE_MAX_FILTER_STAGES)
		table_size = SMC_SISLANDS_DTE_MAX_FILTER_STAGES;

	tdep_count = dte_data->tdep_count;
	if (tdep_count > SMC_SISLANDS_DTE_MAX_TEMPERATURE_DEPENDENT_ARRAY_SIZE)
		tdep_count = SMC_SISLANDS_DTE_MAX_TEMPERATURE_DEPENDENT_ARRAY_SIZE;

	dte_tables->K = cpu_to_be32(table_size);
	dte_tables->T0 = cpu_to_be32(dte_data->t0);
	dte_tables->MaxT = cpu_to_be32(dte_data->max_t);
	dte_tables->WindowSize = dte_data->window_size;
	dte_tables->temp_select = dte_data->temp_select;
	dte_tables->DTE_mode = dte_data->dte_mode;
	dte_tables->Tthreshold = cpu_to_be32(dte_data->t_threshold);

	if (tdep_count > 0)
		table_size--;

	for (i = 0; i < table_size; i++) {
		dte_tables->tau[i] = cpu_to_be32(dte_data->tau[i]);
		dte_tables->R[i]   = cpu_to_be32(dte_data->r[i]);
	}

	dte_tables->Tdep_count = tdep_count;

	for (i = 0; i < (uint32_t)tdep_count; i++) {
		dte_tables->T_limits[i] = dte_data->t_limits[i];
		dte_tables->Tdep_tau[i] = cpu_to_be32(dte_data->tdep_tau[i]);
		dte_tables->Tdep_R[i] = cpu_to_be32(dte_data->tdep_r[i]);
	}

	ret = si_smc_copy_bytes(hwmgr, smu_data->dte_table_start,
				(uint8_t *)dte_tables,
				sizeof(Smc_SIslands_DTE_Configuration),
				smu_data->sram_end);
	kfree(dte_tables);

	return ret;
}

static void si_calculate_leakage_for_v_and_t_formula(const struct si_leakage_coeffients *coeff,
						     uint16_t v, int32_t t, uint32_t ileakage,
						     uint32_t *leakage)
{
	s64 kt, kv, leakage_w, i_leakage, vddc;
	s64 temperature, t_slope, t_intercept, av, bv, t_ref;
	s64 tmp;

	i_leakage = div64_s64(drm_int2fixp(ileakage), 100);
	vddc = div64_s64(drm_int2fixp(v), 1000);
	temperature = div64_s64(drm_int2fixp(t), 1000);

	t_slope = div64_s64(drm_int2fixp(coeff->t_slope), 100000000);
	t_intercept = div64_s64(drm_int2fixp(coeff->t_intercept), 100000000);
	av = div64_s64(drm_int2fixp(coeff->av), 100000000);
	bv = div64_s64(drm_int2fixp(coeff->bv), 100000000);
	t_ref = drm_int2fixp(coeff->t_ref);

	tmp = drm_fixp_mul(t_slope, vddc) + t_intercept;
	kt = drm_fixp_exp(drm_fixp_mul(tmp, temperature));
	kt = drm_fixp_div(kt, drm_fixp_exp(drm_fixp_mul(tmp, t_ref)));
	kv = drm_fixp_mul(av, drm_fixp_exp(drm_fixp_mul(bv, vddc)));

	leakage_w = drm_fixp_mul(drm_fixp_mul(drm_fixp_mul(i_leakage, kt), kv), vddc);

	*leakage = drm_fixp2int(leakage_w * 1000);
}

static void si_calculate_leakage_for_v_formula(const struct si_leakage_coeffients *coeff,
					       const uint32_t fixed_kt, uint16_t v,
					       uint32_t ileakage, uint32_t *leakage)
{
	s64 kt, kv, leakage_w, i_leakage, vddc;

	i_leakage = div64_s64(drm_int2fixp(ileakage), 100);
	vddc = div64_s64(drm_int2fixp(v), 1000);

	kt = div64_s64(drm_int2fixp(fixed_kt), 100000000);
	kv = drm_fixp_mul(div64_s64(drm_int2fixp(coeff->av), 100000000),
			  drm_fixp_exp(drm_fixp_mul(div64_s64(drm_int2fixp(coeff->bv), 100000000), vddc)));

	leakage_w = drm_fixp_mul(drm_fixp_mul(drm_fixp_mul(i_leakage, kt), kv), vddc);

	*leakage = drm_fixp2int(leakage_w * 1000);
}

static uint32_t si_calculate_cac_wintime(struct pp_hwmgr *hwmgr)
{
	uint32_t xclk;
	uint32_t wintime;
	uint32_t cac_window;
	uint32_t cac_window_size;

	xclk = amdgpu_asic_get_xclk((struct amdgpu_device *)hwmgr->adev);

	if (xclk == 0)
		return 0;

	cac_window = si_rreg(hwmgr, mmCG_CAC_CTRL) & CG_CAC_CTRL__CAC_WINDOW_MASK;
	cac_window_size = ((cac_window & 0xFFFF0000) >> 16) * (cac_window & 0x0000FFFF);

	wintime = (cac_window_size * 100) / xclk;

	return wintime;
}

static int si_get_cac_std_voltage_max_min(struct pp_hwmgr *hwmgr,
					  uint16_t *max, uint16_t *min)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct phm_cac_leakage_table *table = hwmgr->dyn_state.cac_leakage_table;
	uint32_t i;
	uint32_t v0_loadline;

	if (table == NULL)
		return -EINVAL;

	*max = 0;
	*min = 0xFFFF;

	for (i = 0; i < table->count; i++) {
		if (table->entries[i].Vddc > *max)
			*max = table->entries[i].Vddc;
		if (table->entries[i].Vddc < *min)
			*min = table->entries[i].Vddc;
	}

	if (data->powertune_data->lkge_lut_v0_percent > 100)
		return -EINVAL;

	v0_loadline = (*min) * (100 - data->powertune_data->lkge_lut_v0_percent) / 100;

	if (v0_loadline > 0xFFFFUL)
		return -EINVAL;

	*min = (uint16_t)v0_loadline;

	if ((*min > *max) || (*max == 0) || (*min == 0))
		return -EINVAL;

	return 0;
}

static uint16_t si_get_cac_std_voltage_step(uint16_t max, uint16_t min)
{
	return ((max - min) + (SMC_SISLANDS_LKGE_LUT_NUM_OF_VOLT_ENTRIES - 1)) /
		SMC_SISLANDS_LKGE_LUT_NUM_OF_VOLT_ENTRIES;
}

static int si_init_dte_leakage_table(struct pp_hwmgr *hwmgr,
				     PP_SIslands_CacConfig *cac_tables,
				     uint16_t vddc_max, uint16_t vddc_min, uint16_t vddc_step,
				     uint16_t t0, uint16_t t_step)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t leakage;
	unsigned int i, j;
	int32_t t;
	uint32_t smc_leakage;
	uint32_t scaling_factor;
	uint16_t voltage;

	scaling_factor = si_get_smc_power_scaling_factor(hwmgr);

	for (i = 0; i < SMC_SISLANDS_LKGE_LUT_NUM_OF_TEMP_ENTRIES ; i++) {
		t = (1000 * (i * t_step + t0));

		for (j = 0; j < SMC_SISLANDS_LKGE_LUT_NUM_OF_VOLT_ENTRIES; j++) {
			voltage = vddc_max - (vddc_step * j);

			si_calculate_leakage_for_v_and_t_formula(&data->powertune_data->leakage_coefficients,
								 voltage,
								 t,
								 data->dyn_powertune_data.cac_leakage,
								 &leakage);

			smc_leakage = si_scale_power_for_smc(leakage, scaling_factor) / 4;

			if (smc_leakage > 0xFFFF)
				smc_leakage = 0xFFFF;

			cac_tables->cac_lkge_lut[i][SMC_SISLANDS_LKGE_LUT_NUM_OF_VOLT_ENTRIES-1-j] =
				cpu_to_be16((uint16_t)smc_leakage);
		}
	}
	return 0;
}

static int si_init_simplified_leakage_table(struct pp_hwmgr *hwmgr,
					    PP_SIslands_CacConfig *cac_tables,
					    uint16_t vddc_max, uint16_t vddc_min, uint16_t vddc_step)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t leakage;
	unsigned int i, j;
	uint32_t smc_leakage;
	uint32_t scaling_factor;
	uint16_t voltage;

	scaling_factor = si_get_smc_power_scaling_factor(hwmgr);

	for (j = 0; j < SMC_SISLANDS_LKGE_LUT_NUM_OF_VOLT_ENTRIES; j++) {
		voltage = vddc_max - (vddc_step * j);

		si_calculate_leakage_for_v_formula(&data->powertune_data->leakage_coefficients,
						   data->powertune_data->fixed_kt,
						   voltage,
						   data->dyn_powertune_data.cac_leakage,
						   &leakage);

		smc_leakage = si_scale_power_for_smc(leakage, scaling_factor) / 4;

		if (smc_leakage > 0xFFFF)
			smc_leakage = 0xFFFF;

		for (i = 0; i < SMC_SISLANDS_LKGE_LUT_NUM_OF_TEMP_ENTRIES ; i++)
			cac_tables->cac_lkge_lut[i][SMC_SISLANDS_LKGE_LUT_NUM_OF_VOLT_ENTRIES-1-j] =
				cpu_to_be16((uint16_t)smc_leakage);
	}
	return 0;
}

int si_smc_initialize_smc_cac_tables(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	PP_SIslands_CacConfig *cac_tables = NULL;
	uint16_t vddc_max, vddc_min, vddc_step;
	uint16_t t0, t_step;
	uint32_t load_line_slope, reg;
	int ret = 0;
	uint32_t ticks_per_us = amdgpu_asic_get_xclk((struct amdgpu_device *)hwmgr->adev) / 100;

	if (data->enable_cac == false || data->powertune_data == NULL)
		return 0;

	cac_tables = kzalloc(sizeof(PP_SIslands_CacConfig), GFP_KERNEL);
	if (!cac_tables)
		return -ENOMEM;

	reg = si_rreg(hwmgr, mmCG_CAC_CTRL) & ~CG_CAC_CTRL__CAC_WINDOW_MASK;
	reg |= (data->powertune_data->cac_window << CG_CAC_CTRL__CAC_WINDOW__SHIFT);
	si_wreg(hwmgr, mmCG_CAC_CTRL, reg);

	data->dyn_powertune_data.cac_leakage = hwmgr->platform_descriptor.CACLeakage;
	data->dyn_powertune_data.dc_pwr_value =
		data->powertune_data->dc_cac[SI_DCCAC_LEVEL_0];
	data->dyn_powertune_data.wintime = si_calculate_cac_wintime(hwmgr);
	data->dyn_powertune_data.shift_n = data->powertune_data->shift_n_default;

	data->dyn_powertune_data.leakage_minimum_temperature = 80 * 1000;

	ret = si_get_cac_std_voltage_max_min(hwmgr, &vddc_max, &vddc_min);
	if (ret)
		goto done_free;

	vddc_step = si_get_cac_std_voltage_step(vddc_max, vddc_min);
	vddc_min = vddc_max - (vddc_step * (SMC_SISLANDS_LKGE_LUT_NUM_OF_VOLT_ENTRIES - 1));
	t_step = 4;
	t0 = 60;

	if (data->enable_dte || data->driver_calculate_cac_leakage)
		ret = si_init_dte_leakage_table(hwmgr, cac_tables,
						vddc_max, vddc_min, vddc_step,
						t0, t_step);
	else
		ret = si_init_simplified_leakage_table(hwmgr, cac_tables,
						       vddc_max, vddc_min, vddc_step);
	if (ret)
		goto done_free;

	load_line_slope = ((uint32_t)hwmgr->platform_descriptor.LoadLineSlope << SMC_SISLANDS_SCALE_R) / 100;

	cac_tables->l2numWin_TDP = cpu_to_be32(data->dyn_powertune_data.l2_lta_window_size);
	cac_tables->lts_truncate_n = data->dyn_powertune_data.lts_truncate;
	cac_tables->SHIFT_N = data->dyn_powertune_data.shift_n;
	cac_tables->lkge_lut_V0 = cpu_to_be32((uint32_t)vddc_min);
	cac_tables->lkge_lut_Vstep = cpu_to_be32((uint32_t)vddc_step);
	cac_tables->R_LL = cpu_to_be32(load_line_slope);
	cac_tables->WinTime = cpu_to_be32(data->dyn_powertune_data.wintime);
	cac_tables->calculation_repeats = cpu_to_be32(2);
	cac_tables->dc_cac = cpu_to_be32(0);
	cac_tables->log2_PG_LKG_SCALE = 12;
	cac_tables->cac_temp = data->powertune_data->operating_temp;
	cac_tables->lkge_lut_T0 = cpu_to_be32((uint32_t)t0);
	cac_tables->lkge_lut_Tstep = cpu_to_be32((uint32_t)t_step);

	ret = si_smc_copy_bytes(hwmgr, smu_data->cac_table_start,
				(uint8_t *)cac_tables,
				sizeof(PP_SIslands_CacConfig),
				smu_data->sram_end);

	if (ret)
		goto done_free;

	ret = si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_ticks_per_us, ticks_per_us);

done_free:
	if (ret) {
		data->enable_cac = false;
		data->enable_power_containment = false;
	}

	kfree(cac_tables);

	return ret;
}

/* ---- MC register table (legacy si_initialize_mc_reg_table & co.) ---- */

static int si_set_mc_special_registers(struct pp_hwmgr *hwmgr,
				       struct si_mc_reg_table *table)
{
	struct amdgpu_device *adev = hwmgr->adev;
	uint8_t i, j, k;
	uint32_t temp_reg;

	for (i = 0, j = table->last; i < table->last; i++) {
		if (j >= SMC_SISLANDS_MC_REGISTER_ARRAY_SIZE)
			return -EINVAL;
		switch (table->mc_reg_address[i].s1) {
		case mmMC_SEQ_MISC1:
			temp_reg = si_rreg(hwmgr, mmMC_PMG_CMD_EMRS);
			table->mc_reg_address[j].s1 = mmMC_PMG_CMD_EMRS;
			table->mc_reg_address[j].s0 = mmMC_SEQ_PMG_CMD_EMRS_LP;
			for (k = 0; k < table->num_entries; k++)
				table->mc_reg_table_entry[k].mc_data[j] =
					((temp_reg & 0xffff0000)) |
					((table->mc_reg_table_entry[k].mc_data[i] & 0xffff0000) >> 16);
			j++;

			if (j >= SMC_SISLANDS_MC_REGISTER_ARRAY_SIZE)
				return -EINVAL;
			temp_reg = si_rreg(hwmgr, mmMC_PMG_CMD_MRS);
			table->mc_reg_address[j].s1 = mmMC_PMG_CMD_MRS;
			table->mc_reg_address[j].s0 = mmMC_SEQ_PMG_CMD_MRS_LP;
			for (k = 0; k < table->num_entries; k++) {
				table->mc_reg_table_entry[k].mc_data[j] =
					(temp_reg & 0xffff0000) |
					(table->mc_reg_table_entry[k].mc_data[i] & 0x0000ffff);
				if (adev->gmc.vram_type != AMDGPU_VRAM_TYPE_GDDR5)
					table->mc_reg_table_entry[k].mc_data[j] |= 0x100;
			}
			j++;

			if (adev->gmc.vram_type != AMDGPU_VRAM_TYPE_GDDR5) {
				if (j >= SMC_SISLANDS_MC_REGISTER_ARRAY_SIZE)
					return -EINVAL;
				table->mc_reg_address[j].s1 = mmMC_PMG_AUTO_CMD;
				table->mc_reg_address[j].s0 = mmMC_PMG_AUTO_CMD;
				for (k = 0; k < table->num_entries; k++)
					table->mc_reg_table_entry[k].mc_data[j] =
						(table->mc_reg_table_entry[k].mc_data[i] & 0xffff0000) >> 16;
				j++;
			}
			break;
		case mmMC_SEQ_RESERVE_M:
			temp_reg = si_rreg(hwmgr, mmMC_PMG_CMD_MRS1);
			table->mc_reg_address[j].s1 = mmMC_PMG_CMD_MRS1;
			table->mc_reg_address[j].s0 = mmMC_SEQ_PMG_CMD_MRS1_LP;
			for (k = 0; k < table->num_entries; k++)
				table->mc_reg_table_entry[k].mc_data[j] =
					(temp_reg & 0xffff0000) |
					(table->mc_reg_table_entry[k].mc_data[i] & 0x0000ffff);
			j++;
			break;
		default:
			break;
		}
	}

	table->last = j;

	return 0;
}

static bool si_check_s0_mc_reg_index(uint16_t in_reg, uint16_t *out_reg)
{
	bool result = true;

	switch (in_reg) {
	case mmMC_SEQ_RAS_TIMING:
		*out_reg = mmMC_SEQ_RAS_TIMING_LP;
		break;
	case mmMC_SEQ_CAS_TIMING:
		*out_reg = mmMC_SEQ_CAS_TIMING_LP;
		break;
	case mmMC_SEQ_MISC_TIMING:
		*out_reg = mmMC_SEQ_MISC_TIMING_LP;
		break;
	case mmMC_SEQ_MISC_TIMING2:
		*out_reg = mmMC_SEQ_MISC_TIMING2_LP;
		break;
	case mmMC_SEQ_RD_CTL_D0:
		*out_reg = mmMC_SEQ_RD_CTL_D0_LP;
		break;
	case mmMC_SEQ_RD_CTL_D1:
		*out_reg = mmMC_SEQ_RD_CTL_D1_LP;
		break;
	case mmMC_SEQ_WR_CTL_D0:
		*out_reg = mmMC_SEQ_WR_CTL_D0_LP;
		break;
	case mmMC_SEQ_WR_CTL_D1:
		*out_reg = mmMC_SEQ_WR_CTL_D1_LP;
		break;
	case mmMC_PMG_CMD_EMRS:
		*out_reg = mmMC_SEQ_PMG_CMD_EMRS_LP;
		break;
	case mmMC_PMG_CMD_MRS:
		*out_reg = mmMC_SEQ_PMG_CMD_MRS_LP;
		break;
	case mmMC_PMG_CMD_MRS1:
		*out_reg = mmMC_SEQ_PMG_CMD_MRS1_LP;
		break;
	case mmMC_SEQ_PMG_TIMING:
		*out_reg = mmMC_SEQ_PMG_TIMING_LP;
		break;
	case mmMC_PMG_CMD_MRS2:
		*out_reg = mmMC_SEQ_PMG_CMD_MRS2_LP;
		break;
	case mmMC_SEQ_WR_CTL_2:
		*out_reg = mmMC_SEQ_WR_CTL_2_LP;
		break;
	default:
		result = false;
		break;
	}

	return result;
}

static void si_set_valid_flag(struct si_mc_reg_table *table)
{
	uint8_t i, j;

	for (i = 0; i < table->last; i++) {
		for (j = 1; j < table->num_entries; j++) {
			if (table->mc_reg_table_entry[j-1].mc_data[i] != table->mc_reg_table_entry[j].mc_data[i]) {
				table->valid_flag |= 1 << i;
				break;
			}
		}
	}
}

static void si_set_s0_mc_reg_index(struct si_mc_reg_table *table)
{
	uint32_t i;
	uint16_t address;

	for (i = 0; i < table->last; i++)
		table->mc_reg_address[i].s0 = si_check_s0_mc_reg_index(table->mc_reg_address[i].s1, &address) ?
			address : table->mc_reg_address[i].s1;
}

static int si_copy_vbios_mc_reg_table(const struct atom_mc_reg_table *table,
				      struct si_mc_reg_table *si_table)
{
	uint8_t i, j;

	if (table->last > SMC_SISLANDS_MC_REGISTER_ARRAY_SIZE)
		return -EINVAL;
	if (table->num_entries > MAX_AC_TIMING_ENTRIES)
		return -EINVAL;

	for (i = 0; i < table->last; i++)
		si_table->mc_reg_address[i].s1 = table->mc_reg_address[i].s1;
	si_table->last = table->last;

	for (i = 0; i < table->num_entries; i++) {
		si_table->mc_reg_table_entry[i].mclk_max =
			table->mc_reg_table_entry[i].mclk_max;
		for (j = 0; j < table->last; j++) {
			si_table->mc_reg_table_entry[i].mc_data[j] =
				table->mc_reg_table_entry[i].mc_data[j];
		}
	}
	si_table->num_entries = table->num_entries;

	return 0;
}

/* legacy rv770_get_memory_module_index() */
static uint8_t si_get_memory_module_index(struct pp_hwmgr *hwmgr)
{
	return (uint8_t)((si_rreg(hwmgr, mmBIOS_SCRATCH_4) >> 16) & 0xff);
}

/* pp_smumgr_func.initialize_mc_reg_table (legacy si_initialize_mc_reg_table).
 * Called from the hwmgr enable sequence when dynamic_ac_timing is on;
 * the caller clears dynamic_ac_timing on failure like legacy.
 */
static int si_initialize_mc_reg_table(struct pp_hwmgr *hwmgr)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct amdgpu_device *adev = hwmgr->adev;
	struct atom_mc_reg_table *table;
	struct si_mc_reg_table *si_table = &smu_data->mc_reg_table;
	uint8_t module_index = si_get_memory_module_index(hwmgr);
	unsigned int i;
	int ret;

	table = kzalloc(sizeof(struct atom_mc_reg_table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	memset(si_table, 0, sizeof(*si_table));

	si_wreg(hwmgr, mmMC_SEQ_RAS_TIMING_LP, si_rreg(hwmgr, mmMC_SEQ_RAS_TIMING));
	si_wreg(hwmgr, mmMC_SEQ_CAS_TIMING_LP, si_rreg(hwmgr, mmMC_SEQ_CAS_TIMING));
	si_wreg(hwmgr, mmMC_SEQ_MISC_TIMING_LP, si_rreg(hwmgr, mmMC_SEQ_MISC_TIMING));
	si_wreg(hwmgr, mmMC_SEQ_MISC_TIMING2_LP, si_rreg(hwmgr, mmMC_SEQ_MISC_TIMING2));
	si_wreg(hwmgr, mmMC_SEQ_PMG_CMD_EMRS_LP, si_rreg(hwmgr, mmMC_PMG_CMD_EMRS));
	si_wreg(hwmgr, mmMC_SEQ_PMG_CMD_MRS_LP, si_rreg(hwmgr, mmMC_PMG_CMD_MRS));
	si_wreg(hwmgr, mmMC_SEQ_PMG_CMD_MRS1_LP, si_rreg(hwmgr, mmMC_PMG_CMD_MRS1));
	si_wreg(hwmgr, mmMC_SEQ_WR_CTL_D0_LP, si_rreg(hwmgr, mmMC_SEQ_WR_CTL_D0));
	si_wreg(hwmgr, mmMC_SEQ_WR_CTL_D1_LP, si_rreg(hwmgr, mmMC_SEQ_WR_CTL_D1));
	si_wreg(hwmgr, mmMC_SEQ_RD_CTL_D0_LP, si_rreg(hwmgr, mmMC_SEQ_RD_CTL_D0));
	si_wreg(hwmgr, mmMC_SEQ_RD_CTL_D1_LP, si_rreg(hwmgr, mmMC_SEQ_RD_CTL_D1));
	si_wreg(hwmgr, mmMC_SEQ_PMG_TIMING_LP, si_rreg(hwmgr, mmMC_SEQ_PMG_TIMING));
	si_wreg(hwmgr, mmMC_SEQ_PMG_CMD_MRS2_LP, si_rreg(hwmgr, mmMC_PMG_CMD_MRS2));
	si_wreg(hwmgr, mmMC_SEQ_WR_CTL_2_LP, si_rreg(hwmgr, mmMC_SEQ_WR_CTL_2));

	ret = amdgpu_atombios_init_mc_reg_table(hwmgr->adev, module_index, table);
	if (ret)
		goto init_mc_done;

	ret = si_copy_vbios_mc_reg_table(table, si_table);
	if (ret)
		goto init_mc_done;

	si_set_s0_mc_reg_index(si_table);

	ret = si_set_mc_special_registers(hwmgr, si_table);
	if (ret)
		goto init_mc_done;

	si_set_valid_flag(si_table);

	/* The entry an MCLK lands in decides its MC_SEQ timings, and an MCLK
	 * above every mclk_max (an OverDrive memory clock, say) silently
	 * reuses the last entry - timings meant for a slower clock, which
	 * corrupts data without faulting. Log the ranges so a report of
	 * corruption at an OD memory clock can be checked against them.
	 */
	for (i = 0; i < si_table->num_entries; i++)
		dev_info(adev->dev, "si: MC timing entry %u for mclk <= %u MHz\n",
			 i, si_table->mc_reg_table_entry[i].mclk_max / 100);

init_mc_done:
	kfree(table);

	return ret;
}

static void si_populate_mc_reg_addresses(struct pp_hwmgr *hwmgr,
					 SMC_SIslands_MCRegisters *mc_reg_table)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	uint32_t i, j;

	for (i = 0, j = 0; j < smu_data->mc_reg_table.last; j++) {
		if (smu_data->mc_reg_table.valid_flag & (1 << j)) {
			if (i >= SMC_SISLANDS_MC_REGISTER_ARRAY_SIZE)
				break;
			mc_reg_table->address[i].s0 =
				cpu_to_be16(smu_data->mc_reg_table.mc_reg_address[j].s0);
			mc_reg_table->address[i].s1 =
				cpu_to_be16(smu_data->mc_reg_table.mc_reg_address[j].s1);
			i++;
		}
	}
	mc_reg_table->last = (uint8_t)i;
}

static void si_convert_mc_registers(const struct si_mc_reg_entry *entry,
				    SMC_SIslands_MCRegisterSet *data,
				    uint32_t num_entries, uint32_t valid_flag)
{
	uint32_t i, j;

	for (i = 0, j = 0; j < num_entries; j++) {
		if (valid_flag & (1 << j)) {
			data->value[i] = cpu_to_be32(entry->mc_data[j]);
			i++;
		}
	}
}

static void si_convert_mc_reg_table_entry_to_smc(struct pp_hwmgr *hwmgr,
						 const struct si_performance_level *pl,
						 SMC_SIslands_MCRegisterSet *mc_reg_table_data)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct device *adev = ((struct amdgpu_device *)hwmgr->adev)->dev;
	uint32_t i = 0;

	for (i = 0; i < smu_data->mc_reg_table.num_entries; i++) {
		if (pl->mclk <= smu_data->mc_reg_table.mc_reg_table_entry[i].mclk_max)
			break;
	}

	if ((i == smu_data->mc_reg_table.num_entries) && (i > 0)) {
		--i;
		/* no MC timing entry covers this clock: the timings of a
		 * slower one get used, which corrupts memory traffic rather
		 * than failing visibly
		 */
		dev_warn_once(adev, "si: mclk %u MHz is above every MC timing entry (last covers %u MHz), reusing it\n",
			      pl->mclk / 100,
			      smu_data->mc_reg_table.mc_reg_table_entry[i].mclk_max / 100);
	}

	pr_debug("si mc timings: mclk %u MHz (vddci %u) -> entry %u (<= %u MHz)\n",
		 pl->mclk / 100, pl->vddci, i,
		 smu_data->mc_reg_table.mc_reg_table_entry[i].mclk_max / 100);

	si_convert_mc_registers(&smu_data->mc_reg_table.mc_reg_table_entry[i],
				mc_reg_table_data, smu_data->mc_reg_table.last,
				smu_data->mc_reg_table.valid_flag);
}

static void si_convert_mc_reg_table_to_smc(struct pp_hwmgr *hwmgr,
					   const struct si_power_state *state,
					   SMC_SIslands_MCRegisters *mc_reg_table)
{
	int i;

	for (i = 0; i < state->performance_level_count; i++) {
		si_convert_mc_reg_table_entry_to_smc(hwmgr,
						     &state->performance_levels[i],
						     &mc_reg_table->data[SISLANDS_MCREGISTERTABLE_FIRST_DRIVERSTATE_SLOT + i]);
	}
}

int si_smc_populate_mc_reg_table(struct pp_hwmgr *hwmgr,
				 const struct si_power_state *boot_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct si_ulv_param *ulv = &data->ulv;
	SMC_SIslands_MCRegisters *smc_mc_reg_table = &smu_data->smc_mc_reg_table;

	memset(smc_mc_reg_table, 0, sizeof(SMC_SIslands_MCRegisters));

	si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_seq_index, 1);

	si_populate_mc_reg_addresses(hwmgr, smc_mc_reg_table);

	si_convert_mc_reg_table_entry_to_smc(hwmgr, &boot_state->performance_levels[0],
					     &smc_mc_reg_table->data[SISLANDS_MCREGISTERTABLE_INITIAL_SLOT]);

	si_convert_mc_registers(&smu_data->mc_reg_table.mc_reg_table_entry[0],
				&smc_mc_reg_table->data[SISLANDS_MCREGISTERTABLE_ACPI_SLOT],
				smu_data->mc_reg_table.last,
				smu_data->mc_reg_table.valid_flag);

	if (ulv->supported && ulv->pl.vddc != 0)
		si_convert_mc_reg_table_entry_to_smc(hwmgr, &ulv->pl,
						     &smc_mc_reg_table->data[SISLANDS_MCREGISTERTABLE_ULV_SLOT]);
	else
		si_convert_mc_registers(&smu_data->mc_reg_table.mc_reg_table_entry[0],
					&smc_mc_reg_table->data[SISLANDS_MCREGISTERTABLE_ULV_SLOT],
					smu_data->mc_reg_table.last,
					smu_data->mc_reg_table.valid_flag);

	si_convert_mc_reg_table_to_smc(hwmgr, boot_state, smc_mc_reg_table);

	return si_smc_copy_bytes(hwmgr, smu_data->mc_reg_table_start,
				 (uint8_t *)smc_mc_reg_table,
				 sizeof(SMC_SIslands_MCRegisters), smu_data->sram_end);
}

int si_smc_upload_mc_reg_table(struct pp_hwmgr *hwmgr,
			       const struct si_power_state *new_state)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	uint32_t address = smu_data->mc_reg_table_start +
		offsetof(SMC_SIslands_MCRegisters,
			 data[SISLANDS_MCREGISTERTABLE_FIRST_DRIVERSTATE_SLOT]);
	SMC_SIslands_MCRegisters *smc_mc_reg_table = &smu_data->smc_mc_reg_table;

	memset(smc_mc_reg_table, 0, sizeof(SMC_SIslands_MCRegisters));

	si_convert_mc_reg_table_to_smc(hwmgr, new_state, smc_mc_reg_table);

	return si_smc_copy_bytes(hwmgr, address,
				 (uint8_t *)&smc_mc_reg_table->data[SISLANDS_MCREGISTERTABLE_FIRST_DRIVERSTATE_SLOT],
				 sizeof(SMC_SIslands_MCRegisterSet) * new_state->performance_level_count,
				 smu_data->sram_end);
}

/* ---- fan table (pp_smumgr_func.thermal_setup_fan_table) ---- */

static int si_thermal_setup_fan_table(struct pp_hwmgr *hwmgr)
{
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	struct amdgpu_device *adev = hwmgr->adev;
	const struct pp_advance_fan_control_parameters *fan =
		&hwmgr->thermal_controller.advanceFanControlParameters;
	PP_SIslands_FanTable fan_table = { FDO_MODE_HARDWARE };
	uint32_t duty100;
	uint32_t t_diff1, t_diff2, pwm_diff1, pwm_diff2;
	uint32_t t_min, t_med, t_high, pwm_min, pwm_med, pwm_high;
	uint16_t fdo_min, slope1, slope2;
	uint32_t reference_clock, tmp;
	int ret;
	uint64_t tmp64;

	if (!phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			     PHM_PlatformCaps_MicrocodeFanControl))
		return 0;

	if (!smu_data->fan_table_start) {
		phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			      PHM_PlatformCaps_MicrocodeFanControl);
		return 0;
	}

	duty100 = (si_rreg(hwmgr, mmCG_FDO_CTRL1) & CG_FDO_CTRL1__FMAX_DUTY100_MASK) >>
		CG_FDO_CTRL1__FMAX_DUTY100__SHIFT;

	if (duty100 == 0) {
		phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			      PHM_PlatformCaps_MicrocodeFanControl);
		return 0;
	}

	/* VBIOS curve, in 0.01 C and 0.01 %, unless amdgpu.si_fan_curve
	 * replaces it (temperatures in C, duty cycles in percent)
	 */
	t_min = fan->usTMin;
	t_med = fan->usTMed;
	t_high = fan->usTHigh;
	pwm_min = fan->usPWMMin;
	pwm_med = fan->usPWMMed;
	pwm_high = fan->usPWMHigh;

	if (amdgpu_si_fan_curve && *amdgpu_si_fan_curve) {
		unsigned int c[6];

		if (sscanf(amdgpu_si_fan_curve, "%u,%u,%u,%u,%u,%u",
			   &c[0], &c[1], &c[2], &c[3], &c[4], &c[5]) == 6 &&
		    c[0] < c[2] && c[2] < c[4] && c[4] <= 130 &&
		    c[1] <= c[3] && c[3] <= c[5] && c[5] <= 100) {
			t_min = c[0] * 100;
			pwm_min = c[1] * 100;
			t_med = c[2] * 100;
			pwm_med = c[3] * 100;
			t_high = c[4] * 100;
			pwm_high = c[5] * 100;
			dev_info(adev->dev,
				 "si: fan curve %u C/%u%% -> %u C/%u%% -> %u C/%u%%\n",
				 c[0], c[1], c[2], c[3], c[4], c[5]);
		} else {
			dev_warn(adev->dev,
				 "si: ignoring malformed si_fan_curve \"%s\", expected tmin,pwmmin,tmed,pwmmed,thigh,pwmhigh with rising values\n",
				 amdgpu_si_fan_curve);
		}
	}

	tmp64 = (uint64_t)pwm_min * duty100;
	do_div(tmp64, 10000);
	fdo_min = (uint16_t)tmp64;

	t_diff1 = t_med - t_min;
	t_diff2 = t_high - t_med;

	pwm_diff1 = pwm_med - pwm_min;
	pwm_diff2 = pwm_high - pwm_med;

	slope1 = (uint16_t)((50 + ((16 * duty100 * pwm_diff1) / t_diff1)) / 100);
	slope2 = (uint16_t)((50 + ((16 * duty100 * pwm_diff2) / t_diff2)) / 100);

	fan_table.temp_min = cpu_to_be16((50 + t_min) / 100);
	fan_table.temp_med = cpu_to_be16((50 + t_med) / 100);
	fan_table.temp_max = cpu_to_be16((50 + max_t(uint32_t, fan->usTMax, t_high)) / 100);
	fan_table.slope1 = cpu_to_be16(slope1);
	fan_table.slope2 = cpu_to_be16(slope2);
	fan_table.fdo_min = cpu_to_be16(fdo_min);
	fan_table.hys_down = cpu_to_be16(fan->ucTHyst);
	fan_table.hys_up = cpu_to_be16(1);
	fan_table.hys_slope = cpu_to_be16(1);
	fan_table.temp_resp_lim = cpu_to_be16(5);
	reference_clock = amdgpu_asic_get_xclk(adev);

	fan_table.refresh_period = cpu_to_be32((fan->ulCycleDelay *
						reference_clock) / 1600);
	fan_table.fdo_max = cpu_to_be16((uint16_t)duty100);

	tmp = (si_rreg(hwmgr, mmCG_MULT_THERMAL_CTRL) & CG_MULT_THERMAL_CTRL__TEMP_SEL_MASK) >>
		CG_MULT_THERMAL_CTRL__TEMP_SEL__SHIFT;
	fan_table.temp_src = (uint8_t)tmp;

	ret = si_smc_copy_bytes(hwmgr,
				smu_data->fan_table_start,
				(uint8_t *)(&fan_table),
				sizeof(fan_table),
				smu_data->sram_end);

	if (ret) {
		pr_err("Failed to load fan table to the SMC.");
		phm_cap_unset(hwmgr->platform_descriptor.platformCaps,
			      PHM_PlatformCaps_MicrocodeFanControl);
	}

	return ret;
}

/* SI has no separate sclk threshold / per-level graphic and memory
 * tables: the whole driver state is (re)built by si_smc_upload_sw_state.
 */
static int si_update_sclk_threshold(struct pp_hwmgr *hwmgr)
{
	return 0;
}

static int si_populate_all_graphic_levels(struct pp_hwmgr *hwmgr)
{
	return 0;
}

static int si_populate_all_memory_levels(struct pp_hwmgr *hwmgr)
{
	return 0;
}

static uint32_t si_get_mac_definition(uint32_t value)
{
	switch (value) {
	case SMU_MAX_LEVELS_GRAPHICS:
	case SMU_MAX_LEVELS_MEMORY:
		/* One SW state, up to SISLANDS_MAX_HARDWARE_POWERLEVELS
		 * hardware levels per state (legacy si_dpm.h).
		 */
		return SISLANDS_MAX_SMC_PERFORMANCE_LEVELS_PER_SWSTATE;
	case SMU_MAX_LEVELS_LINK:
		return 1;
	case SMU_MAX_ENTRIES_SMIO:
		return SISLANDS_MAX_NO_VREG_STEPS;
	default:
		return 0;
	}
}

static bool si_is_dpm_running(struct pp_hwmgr *hwmgr)
{
	return si_smc_is_running(hwmgr);
}

static int si_update_dpm_settings(struct pp_hwmgr *hwmgr, void *profile_setting)
{
	return 0;
}

static int si_update_smc_table(struct pp_hwmgr *hwmgr, uint32_t type)
{
	return 0;
}

static int si_stop_smc(struct pp_hwmgr *hwmgr)
{
	si_smc_dpm_stop(hwmgr);
	return 0;
}

const struct pp_smumgr_func si_smu_funcs = {
	.name = "si_smu",
	.smu_init = si_smu_init,
	.smu_fini = si_smu_fini,
	.start_smu = si_start_smu,
	.check_fw_load_finish = NULL,
	.request_smu_load_fw = NULL,
	.request_smu_load_specific_fw = NULL,
	.send_msg_to_smc = si_send_msg_to_smc,
	.send_msg_to_smc_with_parameter = si_send_msg_to_smc_with_parameter,
	.get_argument = si_get_argument,
	.download_pptable_settings = NULL,
	.upload_pptable_settings = NULL,
	.get_offsetof = NULL,
	.process_firmware_header = si_process_firmware_header,
	.init_smc_table = si_init_smc_table,
	.update_sclk_threshold = si_update_sclk_threshold,
	.thermal_setup_fan_table = si_thermal_setup_fan_table,
	.populate_all_graphic_levels = si_populate_all_graphic_levels,
	.populate_all_memory_levels = si_populate_all_memory_levels,
	.get_mac_definition = si_get_mac_definition,
	.initialize_mc_reg_table = si_initialize_mc_reg_table,
	.is_dpm_running = si_is_dpm_running,
	.update_dpm_settings = si_update_dpm_settings,
	.update_smc_table = si_update_smc_table,
	.stop_smc = si_stop_smc,
};
