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
 * SI PowerPlay HW manager, Oland-first.
 *
 * Phase 2: backend init (legacy si_dpm_init) and VBIOS power-state
 * parsing (legacy si_parse_power_table) on top of the PowerPlay V0
 * pptable parser, plus the state adjust rules.
 *
 * Phase 3: the DPM enable/disable sequences (legacy si_dpm_enable /
 * si_dpm_disable), the power state switch (si_dpm_set_power_state),
 * forced levels, thermal + fan control, sensors and pp_dpm_* listing.
 * SMC table building lives in smumgr/si_smumgr.c. Mapping onto the
 * PowerPlay framework:
 *   asic_setup                       si_dpm_setup_asic
 *   dynamic_state_management_enable  si_dpm_enable (minus thermal start)
 *   start_thermal_controller         si_thermal_start_thermal_controller
 *                                    + late_init si_set_temperature_range
 *   apply_state_adjust_rules         pre_set_power_state
 *   power_state_set                  si_dpm_set_power_state
 *   force_dpm_level                  si_dpm_force_performance_level
 *   display_config_changed           si_program_display_gap
 * Phase 4: PowerTune/CAC/DTE per-ASIC constant tables ported in
 * si_powertune.c, and si.c flips Oland to pp_smu_ip_block.
 *
 * Preserved legacy behaviors (do not regress):
 *  - Oland OD SCLK/MCLK/TDP caps (SI_OLAND_OD_*_MAX) and 10 kHz units.
 *  - Oland MCLK OD + active display => pin all levels to OD MCLK
 *    (uncommitted si_dpm.c MCLK-switching hang fix).
 *  - Oland fan tachometer (fan1_input/target) + power1_cap hwmon.
 *  - Oland DCE6.4 quirk stays in DC/display, not here, but this
 *    backend must not force an MCLK switch behind DC's back.
 */

#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <drm/drm_print.h>
#include <drm/amdgpu_drm.h>
#include <drm/amd_asic_type.h>
#include "pp_debug.h"
#include "hwmgr.h"
#include "smumgr.h"
#include "si_hwmgr.h"
#include "si_smumgr.h"
#include "hardwaremanager.h"
#include "processpptables.h"
#include "ppatomctrl.h"
#include "cgs_common.h"
#include "amd_pcie.h"
#include "atom.h"
#include "smu_helper.h"
#include "ivsrcid/ivsrcid_vislands30.h"
#include "pptable.h"

#include "bif/bif_3_0_d.h"
#include "bif/bif_3_0_sh_mask.h"

#include "gmc/gmc_6_0_d.h"
#include "gmc/gmc_6_0_sh_mask.h"

#include "smu/smu_6_0_d.h"
#include "smu/smu_6_0_sh_mask.h"

#include "r600_dpm.h"

/* MC_SEQ_MISC0 decode from amdgpu sid.h (not in gmc_6_0_sh_mask.h). */
#define SI_MC_SEQ_MISC0_VEN_ID_SHIFT	8
#define SI_MC_SEQ_MISC0_VEN_ID_MASK	0x00000f00
#define SI_MC_SEQ_MISC0_VEN_ID_VALUE	3
#define SI_MC_SEQ_MISC0_REV_ID_SHIFT	12
#define SI_MC_SEQ_MISC0_REV_ID_MASK	0x0000f000
#define SI_MC_SEQ_MISC0_REV_ID_VALUE	1
#define SI_MC_SEQ_MISC0_GDDR5_SHIFT	28
#define SI_MC_SEQ_MISC0_GDDR5_MASK	0xf0000000
#define SI_MC_SEQ_MISC0_GDDR5_VALUE	5

static const unsigned long PhwSIslands_Magic = (unsigned long)(PHM_SIslands_Magic);

static struct si_power_state *cast_phw_si_power_state(
				  struct pp_hw_power_state *hw_ps)
{
	PP_ASSERT_WITH_CODE((PhwSIslands_Magic == hw_ps->magic),
				"Invalid Powerstate Type!",
				 return NULL);

	return (struct si_power_state *)hw_ps;
}

/* ---- leakage voltage (legacy si_get_leakage_vddc & friends) ---- */

static void si_get_leakage_vddc(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	uint16_t vddc, count = 0;
	int i, ret;

	for (i = 0; i < SI_MAX_LEAKAGE_COUNT; i++) {
		ret = amdgpu_atombios_get_leakage_vddc_based_on_leakage_idx(adev,
					&vddc, SI_LEAKAGE_INDEX0 + i);

		if (!ret && (vddc > 0) && (vddc != (SI_LEAKAGE_INDEX0 + i))) {
			data->leakage_voltage.entries[count].voltage = vddc;
			data->leakage_voltage.entries[count].leakage_index =
				SI_LEAKAGE_INDEX0 + i;
			count++;
		}
	}
	data->leakage_voltage.count = count;
}

static int si_get_leakage_voltage_from_leakage_index(struct pp_hwmgr *hwmgr,
						     uint32_t index,
						     uint16_t *leakage_voltage)
{
	struct si_hwmgr *data = hwmgr->backend;
	int i;

	if (leakage_voltage == NULL)
		return -EINVAL;

	if ((index & 0xff00) != 0xff00)
		return -EINVAL;

	if ((index & 0xff) > SI_MAX_LEAKAGE_COUNT + 1)
		return -EINVAL;

	if (index < SI_LEAKAGE_INDEX0)
		return -EINVAL;

	for (i = 0; i < data->leakage_voltage.count; i++) {
		if (data->leakage_voltage.entries[i].leakage_index == index) {
			*leakage_voltage = data->leakage_voltage.entries[i].voltage;
			return 0;
		}
	}
	return -EAGAIN;
}

static int si_patch_single_dependency_table_based_on_leakage(struct pp_hwmgr *hwmgr,
				struct phm_clock_voltage_dependency_table *table)
{
	uint32_t i;
	int j;
	uint16_t leakage_voltage;

	if (table) {
		for (i = 0; i < table->count; i++) {
			switch (si_get_leakage_voltage_from_leakage_index(hwmgr,
								table->entries[i].v,
								&leakage_voltage)) {
			case 0:
				table->entries[i].v = leakage_voltage;
				break;
			case -EAGAIN:
				return -EINVAL;
			case -EINVAL:
			default:
				break;
			}
		}

		for (j = (table->count - 2); j >= 0; j--) {
			table->entries[j].v = (table->entries[j].v <= table->entries[j + 1].v) ?
				table->entries[j].v : table->entries[j + 1].v;
		}
	}
	return 0;
}

static int si_patch_dependency_tables_based_on_leakage(struct pp_hwmgr *hwmgr)
{
	int ret = 0;

	ret = si_patch_single_dependency_table_based_on_leakage(hwmgr,
				hwmgr->dyn_state.vddc_dependency_on_sclk);
	if (ret)
		pr_err("Could not patch vddc_on_sclk leakage table\n");
	ret = si_patch_single_dependency_table_based_on_leakage(hwmgr,
				hwmgr->dyn_state.vddc_dependency_on_mclk);
	if (ret)
		pr_err("Could not patch vddc_on_mclk leakage table\n");
	ret = si_patch_single_dependency_table_based_on_leakage(hwmgr,
				hwmgr->dyn_state.vddci_dependency_on_mclk);
	if (ret)
		pr_err("Could not patch vddci_on_mclk leakage table\n");
	return ret;
}

/* ---- PCIe (legacy si_get_current_pcie_speed / si_gen_pcie_gen_support) ---- */

static uint16_t si_get_current_pcie_speed(struct pp_hwmgr *hwmgr)
{
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t speed_cntl;

	/* PCIE_LC_SPEED_CNTL lives in the PCIE port space on SI
	 * (legacy RREG32_PCIE_PORT), not CGS_IND_REG__PCIE.
	 */
	speed_cntl = RREG32_PCIE_PORT(ixPCIE_LC_SPEED_CNTL) &
		     PCIE_LC_SPEED_CNTL__LC_CURRENT_DATA_RATE_MASK;
	speed_cntl >>= PCIE_LC_SPEED_CNTL__LC_CURRENT_DATA_RATE__SHIFT;

	return (uint16_t)speed_cntl;
}

enum si_pcie_gen si_gen_pcie_gen_support(uint32_t sys_mask,
						enum si_pcie_gen asic_gen,
						enum si_pcie_gen default_gen)
{
	switch (asic_gen) {
	case SI_PCIE_GEN1:
		return SI_PCIE_GEN1;
	case SI_PCIE_GEN2:
		return SI_PCIE_GEN2;
	case SI_PCIE_GEN3:
		return SI_PCIE_GEN3;
	default:
		if ((sys_mask & CAIL_PCIE_LINK_SPEED_SUPPORT_GEN3) &&
		    (default_gen == SI_PCIE_GEN3))
			return SI_PCIE_GEN3;
		else if ((sys_mask & CAIL_PCIE_LINK_SPEED_SUPPORT_GEN2) &&
			 (default_gen == SI_PCIE_GEN2))
			return SI_PCIE_GEN2;
		else
			return SI_PCIE_GEN1;
	}
	return SI_PCIE_GEN1;
}

/* ---- misc ASIC quirks ---- */

static void si_set_max_cu_value(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;

	if (hwmgr->chip_id == CHIP_VERDE) {
		switch (adev->pdev->device) {
		case 0x6820:
		case 0x6825:
		case 0x6821:
		case 0x6823:
		case 0x6827:
			data->max_cu = 10;
			break;
		case 0x682D:
		case 0x6824:
		case 0x682F:
		case 0x6826:
			data->max_cu = 8;
			break;
		case 0x6828:
		case 0x6830:
		case 0x6831:
		case 0x6838:
		case 0x6839:
		case 0x683D:
			data->max_cu = 10;
			break;
		case 0x683B:
		case 0x683F:
		case 0x6829:
			data->max_cu = 8;
			break;
		default:
			data->max_cu = 0;
			break;
		}
	} else {
		data->max_cu = 0;
	}
}

static bool si_is_special_1gb_platform(struct pp_hwmgr *hwmgr)
{
	struct amdgpu_device *adev = hwmgr->adev;
	bool ret = false;
	uint32_t tmp, width, row, column, bank, density;
	bool is_memory_gddr5, is_special;

	tmp = cgs_read_register(hwmgr->device, mmMC_SEQ_MISC0);
	is_memory_gddr5 = (SI_MC_SEQ_MISC0_GDDR5_VALUE ==
			   ((tmp & SI_MC_SEQ_MISC0_GDDR5_MASK) >> SI_MC_SEQ_MISC0_GDDR5_SHIFT));
	is_special = (SI_MC_SEQ_MISC0_REV_ID_VALUE ==
		      ((tmp & SI_MC_SEQ_MISC0_REV_ID_MASK) >> SI_MC_SEQ_MISC0_REV_ID_SHIFT))
		& (SI_MC_SEQ_MISC0_VEN_ID_VALUE ==
		   ((tmp & SI_MC_SEQ_MISC0_VEN_ID_MASK) >> SI_MC_SEQ_MISC0_VEN_ID_SHIFT));

	cgs_write_register(hwmgr->device, mmMC_SEQ_IO_DEBUG_INDEX, 0xb);
	width = ((cgs_read_register(hwmgr->device, mmMC_SEQ_IO_DEBUG_DATA) >> 1) & 1) ? 16 : 32;

	tmp = cgs_read_register(hwmgr->device, mmMC_ARB_RAMCFG);
	row = ((tmp & MC_ARB_RAMCFG__NOOFROWS_MASK) >> MC_ARB_RAMCFG__NOOFROWS__SHIFT) + 10;
	column = ((tmp & MC_ARB_RAMCFG__NOOFCOLS_MASK) >> MC_ARB_RAMCFG__NOOFCOLS__SHIFT) + 8;
	bank = ((tmp & MC_ARB_RAMCFG__NOOFBANK_MASK) >> MC_ARB_RAMCFG__NOOFBANK__SHIFT) + 2;

	density = (1 << (row + column - 20 + bank)) * width;

	if ((adev->pdev->device == 0x6819) &&
	    is_memory_gddr5 && is_special && (density == 0x400))
		ret = true;

	return ret;
}

static void si_get_max_vddc(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint16_t vddc;

	if (amdgpu_atombios_get_max_vddc(hwmgr->adev, 0, 0, &vddc))
		data->max_vddc = 0;
	else
		data->max_vddc = vddc;
}

static void si_get_engine_memory_ss(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_atom_ss ss;

	data->sclk_ss = amdgpu_atombios_get_asic_ss_info(hwmgr->adev, &ss,
							 ASIC_INTERNAL_ENGINE_SS, 0);
	data->mclk_ss = amdgpu_atombios_get_asic_ss_info(hwmgr->adev, &ss,
							 ASIC_INTERNAL_MEMORY_SS, 0);
	if (amdgpu_si_dpm_quirks & SI_QUIRK_NO_SCLK_SS)
		data->sclk_ss = false;

	if (data->sclk_ss || data->mclk_ss)
		data->dynamic_ss = true;
	else
		data->dynamic_ss = false;
}

/* legacy si_dpm_init: fixed 4-entry vddc/dispclk table */
static int si_init_voltage_dependency_on_display_clock_table(struct pp_hwmgr *hwmgr)
{
	struct phm_clock_voltage_dependency_table *table;

	table = kzalloc(struct_size(table, entries, 4), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	table->count = 4;
	table->entries[0].clk = 0;
	table->entries[0].v = 0;
	table->entries[1].clk = 36000;
	table->entries[1].v = 720;
	table->entries[2].clk = 54000;
	table->entries[2].v = 810;
	table->entries[3].clk = 72000;
	table->entries[3].v = 900;

	hwmgr->dyn_state.vddc_dependency_on_display_clock = table;

	return 0;
}


/* ---- VCE states (legacy si_parse_power_table tail) ---- */

static void si_init_vce_states(struct pp_hwmgr *hwmgr)
{
	const struct pp_table_func *pt = hwmgr->pptable_func;
	int i, n;

	hwmgr->num_vce_state_tables = 0;

	if (!pt->pptable_get_number_of_vce_state_table_entries ||
	    !pt->pptable_get_vce_state_table_entry)
		return;

	n = pt->pptable_get_number_of_vce_state_table_entries(hwmgr);
	if (n > AMD_MAX_VCE_LEVELS)
		n = AMD_MAX_VCE_LEVELS;

	for (i = 0; i < n; i++) {
		struct amd_vce_state *vce_state = &hwmgr->vce_states[i];
		const ATOM_PPLIB_SI_CLOCK_INFO *clock_info;
		void *ci = NULL;
		unsigned long flag = 0;

		if (pt->pptable_get_vce_state_table_entry(hwmgr, i, vce_state,
							  &ci, &flag))
			continue;
		if (!ci)
			continue;
		clock_info = ci;

		vce_state->sclk = le16_to_cpu(clock_info->usEngineClockLow) |
				  (clock_info->ucEngineClockHigh << 16);
		vce_state->mclk = le16_to_cpu(clock_info->usMemoryClockLow) |
				  (clock_info->ucMemoryClockHigh << 16);
		vce_state->clk_idx = i;
	}
	hwmgr->num_vce_state_tables = n;
}

/* ---- backend init/fini (legacy si_dpm_init) ---- */

static void si_smc_debugfs_remove(void);

static int si_hwmgr_backend_fini(struct pp_hwmgr *hwmgr)
{
	si_smc_debugfs_remove();
	kfree(hwmgr->dyn_state.vddc_dependency_on_display_clock);
	hwmgr->dyn_state.vddc_dependency_on_display_clock = NULL;

	if (hwmgr->backend)
		kfree(((struct si_hwmgr *)hwmgr->backend)->pristine_ps);
	kfree(hwmgr->backend);
	hwmgr->backend = NULL;
	return 0;
}

/* pp_power_profile_mode. SMC6 has no workload hints, but the thresholds
 * it switches levels on are built by si_populate_smc_t() from a target
 * activity and a hysteresis either side of it, so a profile is those
 * three numbers. Lower activity climbs sooner, wider hysteresis stays
 * put longer; the defaults reproduce the VBIOS-derived behaviour.
 */
static const struct si_activity_profile si_default_profiles[PP_SMC_POWER_PROFILE_COUNT] = {
	[PP_SMC_POWER_PROFILE_BOOTUP_DEFAULT] = { 100, R600_AH_DFLT, R600_AH_DFLT },
	[PP_SMC_POWER_PROFILE_FULLSCREEN3D]   = {  70, R600_AH_DFLT * 2, R600_AH_DFLT },
	[PP_SMC_POWER_PROFILE_POWERSAVING]    = { 130, R600_AH_DFLT, R600_AH_DFLT * 2 },
	[PP_SMC_POWER_PROFILE_VIDEO]          = { 120, R600_AH_DFLT * 3, R600_AH_DFLT * 3 },
	[PP_SMC_POWER_PROFILE_VR]             = {  70, R600_AH_DFLT * 2, R600_AH_DFLT },
	[PP_SMC_POWER_PROFILE_COMPUTE]        = {  60, R600_AH_DFLT * 2, R600_AH_DFLT },
	[PP_SMC_POWER_PROFILE_CUSTOM]         = { 100, R600_AH_DFLT, R600_AH_DFLT },
};
static int si_hwmgr_backend_init(struct pp_hwmgr *hwmgr)
{
	struct amdgpu_device *adev = hwmgr->adev;
	struct si_hwmgr *data;
	struct atom_clock_dividers dividers;
	uint16_t vddc, vddci, mvdd;
	int ret;

	data = kzalloc(sizeof(struct si_hwmgr), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	hwmgr->backend = data;

	data->sys_pcie_mask =
		adev->pm.pcie_gen_mask & CAIL_PCIE_LINK_SPEED_SUPPORT_MASK;
	data->force_pcie_gen = SI_PCIE_GEN_INVALID;
	data->boot_pcie_gen = si_get_current_pcie_speed(hwmgr);

	si_set_max_cu_value(hwmgr);

	si_get_max_vddc(hwmgr);
	/* Oland 0x6611 rev 0x87 (R7 350 / Radeon 430 / 520): the SMC stepping
	 * through the 400 and 730 MHz levels under a heavy load hangs the
	 * gfx ring within seconds, while a direct 300 <-> top transition is
	 * solid (tested to 1000 MHz core / 1300 MHz memory through 20 forced
	 * and 20 SMC-driven ramps with 3x glmark2 on top; every other quirk
	 * combination died within 7 ramps). Upstream's 780 MHz clamp for this
	 * device id only shrank the last step.
	 *
	 * Default to the ladder Adrenalin runs on the same board (GPU-Z):
	 * the full VBIOS ladder with the level below the top scaled by the
	 * OD ratios at its stock voltage, see si_apply_state_adjust_rules().
	 * amdgpu.si_dpm_quirks=0x10 gets the plain two-level ladder,
	 * =0x100 the unscaled VBIOS one.
	 */
	data->dpm_quirks = amdgpu_si_dpm_quirks;
	if (hwmgr->chip_id == CHIP_OLAND && adev->pdev->device == 0x6611 &&
	    adev->pdev->revision == 0x87 &&
	    !(data->dpm_quirks & (SI_QUIRK_FULL_LADDER |
				  SI_QUIRK_TWO_LEVEL_LADDER |
				  SI_QUIRK_DROP_LEVEL_BELOW_TOP))) {
		data->dpm_quirks |= SI_QUIRK_SCALED_MID_LEVEL;
		dev_info(adev->dev, "si: Windows OD-scaled DPM ladder for Oland 0x6611 rev 0x87 (si_dpm_quirks=0x10 two-level, 0x100 unscaled ladder)\n");
	}

	si_get_leakage_vddc(hwmgr);

	/* Leakage-indexed entries in the VDDC dependency tables.
	 *
	 * Legacy si_dpm_init() (and radeon before it) patches these before
	 * amdgpu_parse_extended_power_table() has parsed the tables, so the
	 * patch has always been a no-op there: entries stay as raw 0xff0x
	 * values, si_apply_voltage_dependency_rules() reads them as an
	 * impossibly high voltage and clamps such levels to max_voltage, and
	 * si_get_std_voltage_value() never matches them, which lowers the
	 * top level's std_vddc/PwrEfficiencyRatio and with it PowerTune's
	 * power estimate. On Oland 0x6611 rev 0x87 that accidental extra
	 * voltage on the 730 MHz level is what keeps the FULL ladder's
	 * 2<->3 transitions alive under load (resolving it to its real
	 * 1100 mV hangs the gfx ring within seconds), so the unscaled full
	 * ladder keeps upstream's behaviour. A reshaped ladder gets the
	 * correctly resolved tables, which is also what puts the OD-scaled
	 * level at the 1100 mV Windows runs it at, and PowerTune estimates
	 * the top level's power the way the VBIOS (and Catalyst) intend,
	 * i.e. throttling a power virus earlier instead of cooking the
	 * board.
	 */
	if (data->dpm_quirks & SI_QUIRK_RESHAPED_LADDER)
		si_patch_dependency_tables_based_on_leakage(hwmgr);

	/* what the VBIOS hands us for the clock/voltage envelope, for
	 * comparing against what another OS runs this board at
	 */
	{
		struct phm_clock_voltage_dependency_table *dep;
		unsigned int i;

		dev_info(adev->dev, "si vbios max vddc %u mV, %u leakage entries\n",
			 data->max_vddc, data->leakage_voltage.count);
		for (i = 0; i < data->leakage_voltage.count; i++)
			dev_info(adev->dev, "si   leakage idx 0x%x -> %u mV\n",
				 data->leakage_voltage.entries[i].leakage_index,
				 data->leakage_voltage.entries[i].voltage);
		dep = hwmgr->dyn_state.vddc_dependency_on_sclk;
		for (i = 0; dep && i < dep->count; i++)
			dev_info(adev->dev, "si   vddc_on_sclk[%u]: %u MHz -> %u mV\n", i,
				 dep->entries[i].clk / 100, dep->entries[i].v);
		dep = hwmgr->dyn_state.vddci_dependency_on_mclk;
		for (i = 0; dep && i < dep->count; i++)
			dev_info(adev->dev, "si   vddci_on_mclk[%u]: %u MHz -> %u mV\n", i,
				 dep->entries[i].clk / 100, dep->entries[i].v);
		dep = hwmgr->dyn_state.vddc_dependency_on_mclk;
		for (i = 0; dep && i < dep->count; i++)
			dev_info(adev->dev, "si   vddc_on_mclk[%u]: %u MHz -> %u mV\n", i,
				 dep->entries[i].clk / 100, dep->entries[i].v);
	}

	data->acpi_vddc = 0;
	data->acpi_vddci = 0;
	data->min_vddc_in_table = 0;
	data->max_vddc_in_table = 0;

	/* VBIOS boot state: legacy si_parse_pplib_clock_info overrides the
	 * boot performance level with these in place of the table values.
	 */
	amdgpu_atombios_get_default_voltages(adev, &vddc, &vddci, &mvdd);
	data->vbios_boot_state.sclk_bootup_value = adev->clock.default_sclk;
	data->vbios_boot_state.mclk_bootup_value = adev->clock.default_mclk;
	data->vbios_boot_state.vddc_bootup_value = vddc;
	data->vbios_boot_state.vddci_bootup_value = vddci;
	data->vbios_boot_state.mvdd_bootup_value = mvdd;
	data->mvdd_bootup_value = mvdd;

	/* legacy adev->pm.dpm.tdp_od_limit / power_control: PowerPlay
	 * keeps them in platform_descriptor (processpptables
	 * init_dpm2_parameters). Oland OverDrive: guarantee +20% TDP
	 * headroom even if the BIOS table reports a smaller (or zero)
	 * OD limit.
	 */
	data->tdp_od_limit = hwmgr->platform_descriptor.TDPODLimit;
	data->power_control = phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
					      PHM_PlatformCaps_PowerControl);
	if (hwmgr->chip_id == CHIP_OLAND) {
		if (data->tdp_od_limit < SI_OLAND_OD_TDP_MAX) {
			data->tdp_od_limit = SI_OLAND_OD_TDP_MAX;
			hwmgr->platform_descriptor.TDPODLimit = SI_OLAND_OD_TDP_MAX;
			data->power_control = true;
			phm_cap_set(hwmgr->platform_descriptor.platformCaps,
				    PHM_PlatformCaps_PowerControl);
		}
	}
	data->od_sclk = 0;
	data->od_mclk = 0;
	data->od_tdp = 0;
	data->manual_level_mask = 0;

	memcpy(data->profile, si_default_profiles, sizeof(data->profile));
	hwmgr->power_profile_mode = PP_SMC_POWER_PROFILE_BOOTUP_DEFAULT;
	hwmgr->default_power_profile_mode = PP_SMC_POWER_PROFILE_BOOTUP_DEFAULT;

	/* pp_get/set_power_limit() work in Watts against these; the OD
	 * ceiling is default_power_limit * (100 + TDPODLimit) / 100.
	 */
	hwmgr->default_power_limit = hwmgr->platform_descriptor.TDPLimit;
	hwmgr->power_limit = hwmgr->default_power_limit;

	ret = si_init_voltage_dependency_on_display_clock_table(hwmgr);
	if (ret)
		goto fail;

	pp_tables_get_response_times(hwmgr, &data->voltage_response_time,
				     &data->backbias_response_time);
	if (data->voltage_response_time == 0)
		data->voltage_response_time = SI_VOLTAGERESPONSETIME_DFLT;
	if (data->backbias_response_time == 0)
		data->backbias_response_time = SI_BACKBIASRESPONSETIME_DFLT;

	ret = amdgpu_atombios_get_clock_dividers(adev, COMPUTE_ENGINE_PLL_PARAM,
						 0, false, &dividers);
	if (ret)
		data->ref_div = dividers.ref_div + 1;
	else
		data->ref_div = SI_REFERENCEDIVIDER_DFLT;

	data->smu_uvd_hs = false;

	data->mclk_strobe_mode_threshold = 40000;
	if (si_is_special_1gb_platform(hwmgr))
		data->mclk_stutter_mode_threshold = 0;
	else
		data->mclk_stutter_mode_threshold = data->mclk_strobe_mode_threshold;
	data->mclk_edc_enable_threshold = 40000;
	data->mclk_edc_wr_enable_threshold = 40000;

	data->mclk_rtt_mode_threshold = data->mclk_edc_wr_enable_threshold;

	data->voltage_control =
		amdgpu_atombios_is_voltage_gpio(adev, SET_VOLTAGE_TYPE_ASIC_VDDC,
						VOLTAGE_OBJ_GPIO_LUT);
	if (!data->voltage_control) {
		data->voltage_control_svi2 =
			amdgpu_atombios_is_voltage_gpio(adev, SET_VOLTAGE_TYPE_ASIC_VDDC,
							VOLTAGE_OBJ_SVID2);
		if (data->voltage_control_svi2)
			amdgpu_atombios_get_svi2_info(adev, SET_VOLTAGE_TYPE_ASIC_VDDC,
						      &data->svd_gpio_id,
						      &data->svc_gpio_id);
	}

	data->mvdd_control =
		amdgpu_atombios_is_voltage_gpio(adev, SET_VOLTAGE_TYPE_ASIC_MVDDC,
						VOLTAGE_OBJ_GPIO_LUT);

	data->vddci_control =
		amdgpu_atombios_is_voltage_gpio(adev, SET_VOLTAGE_TYPE_ASIC_VDDCI,
						VOLTAGE_OBJ_GPIO_LUT);
	if (!data->vddci_control)
		data->vddci_control_svi2 =
			amdgpu_atombios_is_voltage_gpio(adev, SET_VOLTAGE_TYPE_ASIC_VDDCI,
							VOLTAGE_OBJ_SVID2);

	data->vddc_phase_shed_control =
		amdgpu_atombios_is_voltage_gpio(adev, SET_VOLTAGE_TYPE_ASIC_VDDC,
						VOLTAGE_OBJ_PHASE_LUT);

	si_get_engine_memory_ss(hwmgr);

	data->asi = SI_ASI_DFLT;
	data->pasi = SI_HASI_DFLT;
	data->vrc = SI_VRC_DFLT;

	data->sclk_deep_sleep = true;
	data->sclk_deep_sleep_above_low = false;

	/* legacy: adev->pm.int_thermal_type != THERMAL_TYPE_NONE */
	data->thermal_protection =
		(hwmgr->thermal_controller.ucType != ATOM_PP_THERMALCONTROLLER_NONE);

	data->dynamic_ac_timing = true;

#if defined(CONFIG_ACPI)
	data->pcie_performance_request =
		amdgpu_acpi_is_pcie_performance_request_supported(adev);
#else
	data->pcie_performance_request = false;
#endif

	hwmgr->dyn_state.mclk_sclk_ratio = 4;
	hwmgr->dyn_state.sclk_mclk_delta = 15000;
	hwmgr->dyn_state.vddc_vddci_delta = 200;
	hwmgr->dyn_state.valid_sclk_values = NULL;
	hwmgr->dyn_state.valid_mclk_values = NULL;

	si_initialize_powertune_defaults(hwmgr);

	si_init_vce_states(hwmgr);

	data->fan_ctrl_is_in_default_mode = true;
	hwmgr->fan_ctrl_is_in_default_mode = true;

	/* Oland-first: OD is the user-visible legacy feature, keep it on.
	 * Other SI dies default the same until per-ASIC validation.
	 */
	hwmgr->od_enabled = true;

	hwmgr->platform_descriptor.hardwareActivityPerformanceLevels =
						SI_MAX_HARDWARE_POWERLEVELS;
	hwmgr->platform_descriptor.hardwarePerformanceLevels = 2;
	hwmgr->platform_descriptor.minimumClocksReductionPercentage = 50;
	hwmgr->platform_descriptor.vbiosInterruptId = 0x20000400; /* IRQ_SOURCE1_SW_INT */
	hwmgr->platform_descriptor.clockStep.engineClock = 500;
	hwmgr->platform_descriptor.clockStep.memoryClock = 500;

	return 0;
fail:
	si_hwmgr_backend_fini(hwmgr);
	return ret;
}

/* ---- power state table (legacy si_parse_power_table) ---- */

static int si_get_num_of_pp_table_entries(struct pp_hwmgr *hwmgr)
{
	unsigned long ret = 0;
	int result;

	result = pp_tables_get_num_of_entries(hwmgr, &ret);
	return result ? 0 : ret;
}

static int si_get_power_state_size(struct pp_hwmgr *hwmgr)
{
	return sizeof(struct si_power_state);
}

static bool si_is_uvd_state(const struct pp_power_state *ps)
{
	/* legacy r600_is_uvd_state() on classification/classification2 */
	return !!(ps->classification.flags & (PP_StateClassificationFlag_Uvd |
					      PP_StateClassificationFlag_HD2 |
					      PP_StateClassificationFlag_UvdHD |
					      PP_StateClassificationFlag_UvdSD |
					      PP_StateClassificationFlag_UvdMVC));
}

/* legacy si_parse_pplib_clock_info(), one call per DPM level. */
static int si_get_pp_table_entry_callback(struct pp_hwmgr *hwmgr,
					  struct pp_hw_power_state *hw_ps,
					  unsigned int index,
					  const void *clock_info)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_power_state *ps = cast_phw_si_power_state(hw_ps);
	const ATOM_PPLIB_SI_CLOCK_INFO *si_clk_info = clock_info;
	struct si_performance_level *pl;
	uint16_t leakage_voltage;
	int ret;

	if (!ps)
		return -EINVAL;

	/* legacy si_parse_power_table: at most SISLANDS_MAX_HARDWARE_POWERLEVELS */
	if (index >= SI_MAX_HARDWARE_POWERLEVELS)
		return 0;

	PP_ASSERT_WITH_CODE(
		(ps->performance_level_count < smum_get_mac_definition(hwmgr, SMU_MAX_LEVELS_GRAPHICS)),
		"Performance levels exceeds SMC limit!",
		return -EINVAL);

	pl = &ps->performance_levels[ps->performance_level_count++];

	pl->sclk = le16_to_cpu(si_clk_info->usEngineClockLow);
	pl->sclk |= si_clk_info->ucEngineClockHigh << 16;
	pl->mclk = le16_to_cpu(si_clk_info->usMemoryClockLow);
	pl->mclk |= si_clk_info->ucMemoryClockHigh << 16;

	pl->vddc = le16_to_cpu(si_clk_info->usVDDC);
	pl->vddci = le16_to_cpu(si_clk_info->usVDDCI);
	pl->flags = le32_to_cpu(si_clk_info->ulFlags);
	pl->pcie_gen = si_gen_pcie_gen_support(data->sys_pcie_mask,
					       data->boot_pcie_gen,
					       si_clk_info->ucPCIEGen);

	/* patch up vddc if necessary */
	ret = si_get_leakage_voltage_from_leakage_index(hwmgr, pl->vddc,
							&leakage_voltage);
	if (ret == 0)
		pl->vddc = leakage_voltage;

	if (data->min_vddc_in_table > pl->vddc)
		data->min_vddc_in_table = pl->vddc;

	if (data->max_vddc_in_table < pl->vddc)
		data->max_vddc_in_table = pl->vddc;

	return 0;
}

/* Called by pp_tables_get_entry() for the boot-classified state, i.e.
 * the "patch up boot state" branch of legacy si_parse_pplib_clock_info.
 */
static int si_patch_boot_state(struct pp_hwmgr *hwmgr,
			       struct pp_hw_power_state *hw_ps)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_power_state *ps = cast_phw_si_power_state(hw_ps);
	struct si_performance_level *pl;
	uint32_t i;

	if (!ps)
		return -EINVAL;

	if (ps->performance_level_count == 0)
		return -EINVAL;

	/* legacy patches every level parsed for the boot state; the VBIOS
	 * boot state carries one level, keep the loop for parity.
	 */
	for (i = 0; i < ps->performance_level_count; i++) {
		pl = &ps->performance_levels[i];
		pl->mclk = data->vbios_boot_state.mclk_bootup_value;
		pl->sclk = data->vbios_boot_state.sclk_bootup_value;
		pl->vddc = data->vbios_boot_state.vddc_bootup_value;
		pl->vddci = data->vbios_boot_state.vddci_bootup_value;
	}

	return 0;
}

static int si_get_pp_table_entry(struct pp_hwmgr *hwmgr,
				 unsigned long entry_index,
				 struct pp_power_state *state)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct si_power_state *ps;
	struct si_performance_level *pl;
	int result;
	unsigned int i;

	/* pp_power_state is allocated with get_power_state_size() extra
	 * bytes after .hardware (psm_init_power_state_table), so the full
	 * si_power_state lives there. Derive the pointer from the state
	 * base + offsetof rather than &state->hardware so FORTIFY does
	 * not flag the memset as a write past the 4-byte member.
	 */
	ps = (struct si_power_state *)((uint8_t *)state +
				       offsetof(struct pp_power_state, hardware));
	memset(ps, 0x00, sizeof(*ps));

	state->hardware.magic = PHM_SIslands_Magic;

	result = pp_tables_get_entry(hwmgr, entry_index, state,
				     si_get_pp_table_entry_callback);
	if (result)
		return result;

	/* legacy si_parse_pplib_non_clock_info: pre-VER2 tables have no
	 * VCLK/DCLK, UVD states fall back to the RV770 defaults.
	 */
	if (state->uvd_clocks.VCLK == 0 && state->uvd_clocks.DCLK == 0 &&
	    si_is_uvd_state(state)) {
		state->uvd_clocks.VCLK = SI_DEFAULT_VCLK_FREQ;
		state->uvd_clocks.DCLK = SI_DEFAULT_DCLK_FREQ;
	}
	ps->vclk = state->uvd_clocks.VCLK;
	ps->dclk = state->uvd_clocks.DCLK;

	/* set DC compatible flag if this state supports DC */
	if (!state->validation.disallowOnDC)
		ps->dc_compatible = true;

	if (ps->performance_level_count == 0)
		return 0;

	pl = &ps->performance_levels[0];

	if (state->classification.flags & PP_StateClassificationFlag_ACPI) {
		data->acpi_vddc = pl->vddc;
		data->acpi_vddci = pl->vddci;
		data->acpi_pcie_gen = pl->pcie_gen;
	}

	if (state->classification.flags & PP_StateClassificationFlag_ULV) {
		/* XXX disable for A0 tahiti */
		data->ulv.supported = false;
		data->ulv.pl = *pl;
		data->ulv.one_pcie_lane_in_ulv = false;
		data->ulv.volt_change_delay = SI_ULVVOLTAGECHANGEDELAY_DFLT;
		data->ulv.cg_ulv_parameter = SI_CGULVPARAMETER_DFLT;
		data->ulv.cg_ulv_control = SI_CGULVCONTROL_DFLT;
	}

	if (state->classification.ui_label == PP_StateUILabel_Performance) {
		/* legacy takes the last parsed level of the performance
		 * state (the loop overwrites per level).
		 */
		pl = &ps->performance_levels[ps->performance_level_count - 1];
		hwmgr->dyn_state.max_clock_voltage_on_ac.sclk = pl->sclk;
		hwmgr->dyn_state.max_clock_voltage_on_ac.mclk = pl->mclk;
		hwmgr->dyn_state.max_clock_voltage_on_ac.vddc = pl->vddc;
		hwmgr->dyn_state.max_clock_voltage_on_ac.vddci = pl->vddci;

		/* make sure dc limits are valid (legacy si_dpm_init tail).
		 * hwmgr_hw_init does the same check, but before the state
		 * table is parsed, i.e. while the AC limit is still zero.
		 */
		if ((hwmgr->dyn_state.max_clock_voltage_on_dc.sclk == 0) ||
		    (hwmgr->dyn_state.max_clock_voltage_on_dc.mclk == 0))
			hwmgr->dyn_state.max_clock_voltage_on_dc =
				hwmgr->dyn_state.max_clock_voltage_on_ac;
	}

	ps->vbios_level_count = ps->performance_level_count;

	/* the VBIOS levels as parsed, before any of the adjust-rule clamps
	 * (the Oland 0x6611 rev 0x87 780 MHz cap among them)
	 */
	dev_info(((struct amdgpu_device *)hwmgr->adev)->dev,
		 "si vbios state %lu (ui label %u, class 0x%x): %u levels\n",
		 entry_index, state->classification.ui_label,
		 state->classification.flags, ps->performance_level_count);
	for (i = 0; i < ps->performance_level_count; i++)
		dev_info(((struct amdgpu_device *)hwmgr->adev)->dev,
			 "si   vbios level %u: sclk %u mclk %u vddc %u vddci %u pcie gen%u\n",
			 i,
			 ps->performance_levels[i].sclk / 100,
			 ps->performance_levels[i].mclk / 100,
			 ps->performance_levels[i].vddc,
			 ps->performance_levels[i].vddci,
			 ps->performance_levels[i].pcie_gen + 1);

	return 0;
}

/* ---- clocks / OD / misc hooks ---- */

static uint32_t si_dpm_get_sclk(struct pp_hwmgr *hwmgr, bool low)
{
	struct pp_power_state *ps = hwmgr->request_ps;
	struct si_power_state *si_ps;

	if (!ps)
		return 0;

	si_ps = cast_phw_si_power_state(&ps->hardware);
	if (!si_ps || si_ps->performance_level_count == 0)
		return 0;

	if (low)
		return si_ps->performance_levels[0].sclk;
	else
		return si_ps->performance_levels[si_ps->performance_level_count - 1].sclk;
}

static uint32_t si_dpm_get_mclk(struct pp_hwmgr *hwmgr, bool low)
{
	struct pp_power_state *ps = hwmgr->request_ps;
	struct si_power_state *si_ps;

	if (!ps)
		return 0;

	si_ps = cast_phw_si_power_state(&ps->hardware);
	if (!si_ps || si_ps->performance_level_count == 0)
		return 0;

	if (low)
		return si_ps->performance_levels[0].mclk;
	else
		return si_ps->performance_levels[si_ps->performance_level_count - 1].mclk;
}

/* Oland OverDrive sysfs semantics, preserved from legacy si_dpm.c:
 * pp_sclk_od / pp_mclk_od are absolute MHz (0 = disabled), not the
 * CI percent delta. Stored internally in 10 kHz units.
 */
static bool si_oland_is_overdrive_supported(struct pp_hwmgr *hwmgr)
{
	return hwmgr->chip_id == CHIP_OLAND;
}

static int si_get_sclk_od(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (!si_oland_is_overdrive_supported(hwmgr))
		return -EOPNOTSUPP;

	return (int)(data->od_sclk / 100);
}

static int si_set_sclk_od(struct pp_hwmgr *hwmgr, uint32_t value)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t od_clk;

	if (!si_oland_is_overdrive_supported(hwmgr))
		return -EOPNOTSUPP;
	if (value == 0) {
		data->od_sclk = 0;
		return 0;
	}
	od_clk = value * 100;
	if (od_clk > SI_OLAND_OD_SCLK_MAX)
		return -EINVAL;
	data->od_sclk = od_clk;
	pr_debug("si pp_sclk_od <- %u MHz\n", value);
	return 0;
}

static int si_get_mclk_od(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (!si_oland_is_overdrive_supported(hwmgr))
		return -EOPNOTSUPP;

	return (int)(data->od_mclk / 100);
}

static int si_set_mclk_od(struct pp_hwmgr *hwmgr, uint32_t value)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t od_clk;

	if (!si_oland_is_overdrive_supported(hwmgr))
		return -EOPNOTSUPP;
	if (value == 0) {
		data->od_mclk = 0;
		return 0;
	}
	od_clk = value * 100;
	if (od_clk > SI_OLAND_OD_MCLK_MAX)
		return -EINVAL;
	data->od_mclk = od_clk;
	pr_debug("si pp_mclk_od <- %u MHz\n", value);
	return 0;
}

/* ---- state adjust rules (legacy btc_* helpers + si_apply_state_adjust_rules) ---- */

static uint16_t si_find_voltage(struct atom_voltage_table *table, uint16_t voltage)
{
	unsigned int i;

	/* voltage tables are built by si_construct_voltage_tables() during
	 * DPM enable; before that, leave the request untouched.
	 */
	if (table->count == 0)
		return voltage;

	for (i = 0; i < table->count; i++)
		if (voltage <= table->entries[i].value)
			return table->entries[i].value;

	return table->entries[table->count - 1].value;
}

static uint32_t si_find_valid_clock(struct phm_clock_array *clocks,
				    uint32_t max_clock, uint32_t requested_clock)
{
	unsigned int i;

	if ((clocks == NULL) || (clocks->count == 0))
		return (requested_clock < max_clock) ? requested_clock : max_clock;

	for (i = 0; i < clocks->count; i++) {
		if (clocks->values[i] >= requested_clock)
			return (clocks->values[i] < max_clock) ? clocks->values[i] : max_clock;
	}

	return (clocks->values[clocks->count - 1] < max_clock) ?
		clocks->values[clocks->count - 1] : max_clock;
}

static void si_get_max_clock_from_voltage_dependency_table(
			struct phm_clock_voltage_dependency_table *table,
			uint32_t *max_clock)
{
	uint32_t i, clock = 0;

	if ((table == NULL) || (table->count == 0)) {
		*max_clock = clock;
		return;
	}

	for (i = 0; i < table->count; i++) {
		if (clock < table->entries[i].clk)
			clock = table->entries[i].clk;
	}
	*max_clock = clock;
}

static void si_apply_voltage_dependency_rules(
			struct phm_clock_voltage_dependency_table *table,
			uint32_t clock, uint16_t max_voltage, uint16_t *voltage)
{
	uint32_t i;

	if ((table == NULL) || (table->count == 0))
		return;

	for (i = 0; i < table->count; i++) {
		if (clock <= table->entries[i].clk) {
			if (*voltage < table->entries[i].v)
				*voltage = (uint16_t)((table->entries[i].v < max_voltage) ?
					   table->entries[i].v : max_voltage);
			return;
		}
	}

	*voltage = (*voltage > max_voltage) ? *voltage : max_voltage;
}

static void si_adjust_clock_combinations(struct pp_hwmgr *hwmgr,
			const struct phm_clock_and_voltage_limits *max_limits,
			struct si_performance_level *pl)
{
	struct phm_dynamic_state_info *dyn = &hwmgr->dyn_state;

	if ((pl->mclk == 0) || (pl->sclk == 0))
		return;

	if (pl->mclk == pl->sclk)
		return;

	if (pl->mclk > pl->sclk) {
		if (((pl->mclk + (pl->sclk - 1)) / pl->sclk) > dyn->mclk_sclk_ratio)
			pl->sclk = si_find_valid_clock(dyn->valid_sclk_values,
						       max_limits->sclk,
						       (pl->mclk +
							(dyn->mclk_sclk_ratio - 1)) /
						       dyn->mclk_sclk_ratio);
	} else {
		if ((pl->sclk - pl->mclk) > dyn->sclk_mclk_delta)
			pl->mclk = si_find_valid_clock(dyn->valid_mclk_values,
						       max_limits->mclk,
						       pl->sclk -
						       dyn->sclk_mclk_delta);
	}
}

static void si_apply_voltage_delta_rules(struct pp_hwmgr *hwmgr,
					 uint16_t max_vddc, uint16_t max_vddci,
					 uint16_t *vddc, uint16_t *vddci)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t delta = hwmgr->dyn_state.vddc_vddci_delta;
	uint16_t new_voltage;

	if ((0 == *vddc) || (0 == *vddci))
		return;

	if (*vddc > *vddci) {
		if ((*vddc - *vddci) > delta) {
			new_voltage = si_find_voltage(&data->vddci_voltage_table,
						      (*vddc - delta));
			*vddci = (new_voltage < max_vddci) ? new_voltage : max_vddci;
		}
	} else {
		if ((*vddci - *vddc) > delta) {
			new_voltage = si_find_voltage(&data->vddc_voltage_table,
						      (*vddci - delta));
			*vddc = (new_voltage < max_vddc) ? new_voltage : max_vddc;
		}
	}
}

static uint16_t si_get_lower_of_leakage_and_vce_voltage(struct pp_hwmgr *hwmgr,
							uint16_t vce_voltage)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint16_t highest_leakage = 0;
	int i;

	for (i = 0; i < data->leakage_voltage.count; i++) {
		if (highest_leakage < data->leakage_voltage.entries[i].voltage)
			highest_leakage = data->leakage_voltage.entries[i].voltage;
	}

	if (data->leakage_voltage.count && (highest_leakage < vce_voltage))
		return highest_leakage;

	return vce_voltage;
}

static int si_get_vce_clock_voltage(struct pp_hwmgr *hwmgr,
				    uint32_t evclk, uint32_t ecclk, uint16_t *voltage)
{
	struct phm_vce_clock_voltage_dependency_table *table =
		hwmgr->dyn_state.vce_clock_voltage_dependency_table;
	uint32_t i;
	int ret = -EINVAL;

	if (((evclk == 0) && (ecclk == 0)) ||
	    (table == NULL) || (table->count == 0)) {
		*voltage = 0;
		return 0;
	}

	for (i = 0; i < table->count; i++) {
		if ((evclk <= table->entries[i].evclk) &&
		    (ecclk <= table->entries[i].ecclk)) {
			*voltage = table->entries[i].v;
			ret = 0;
			break;
		}
	}

	/* if no match return the highest voltage */
	if (ret)
		*voltage = table->entries[table->count - 1].v;

	*voltage = si_get_lower_of_leakage_and_vce_voltage(hwmgr, *voltage);

	return ret;
}

static bool si_dpm_vblank_too_short(struct pp_hwmgr *hwmgr)
{
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t vblank_time = hwmgr->display_config->min_vblank_time;
	/* we never hit the non-gddr5 limit so disable it */
	uint32_t switch_limit = adev->gmc.vram_type == AMDGPU_VRAM_TYPE_GDDR5 ? 450 : 0;

	/* Consider zero vblank time too short and disable MCLK switching.
	 * Note that the vblank time is set to maximum when no displays are attached,
	 * so we'll still enable MCLK switching in that case.
	 */
	if (vblank_time == 0)
		return true;
	else if (vblank_time < switch_limit)
		return true;
	else
		return false;
}

/* PowerPlay adjusts hwmgr->request_ps in place and re-adjusts that same
 * copy on every READJUST/DISPLAY_CONFIG task. The SI rules are not
 * idempotent (the Oland SCLK clamp cuts an OD top back to 780 MHz, OD
 * then scales the level below from that, levels get pinned to the top
 * MCLK when switching is disabled), so start from the pristine pptable
 * entry each time, the way legacy ni_update_requested_ps() re-copied
 * adev->pm.dpm.requested_ps. While UVD is active the UVD classified
 * state is used instead, mirroring legacy
 * amdgpu_dpm_pick_power_state(POWER_STATE_TYPE_INTERNAL_UVD).
 *
 * The pristine entries are a copy of hwmgr->ps taken before the first
 * adjust: ENABLE_USER_STATE (a power_dpm_state write) hands us the table
 * entry itself as the request, so adjusting it in place rewrote the
 * table every later re-seed copies from - "vbios 780/1100 -> 998/1298"
 * for the level below the top after one power_dpm_state=performance.
 */
static void si_reseed_request_state(struct pp_hwmgr *hwmgr,
				    struct pp_power_state *request)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	const struct pp_power_state *state;
	unsigned int i, src = UINT_MAX;

	if (!data || !hwmgr->ps || !hwmgr->ps_size || !hwmgr->num_ps)
		return;

	if (!data->pristine_ps) {
		data->pristine_ps = kmemdup(hwmgr->ps,
					    (size_t)hwmgr->num_ps * hwmgr->ps_size,
					    GFP_KERNEL);
		if (!data->pristine_ps)
			return;
	}

	if (adev->pm.dpm.uvd_active && hwmgr->uvd_ps) {
		src = ((unsigned long)hwmgr->uvd_ps - (unsigned long)hwmgr->ps) /
		      hwmgr->ps_size;
	} else {
		for (i = 0; i < hwmgr->num_ps; i++) {
			state = data->pristine_ps + (size_t)i * hwmgr->ps_size;
			if (state->id == request->id) {
				src = i;
				break;
			}
		}
	}
	if (src >= hwmgr->num_ps)
		return;

	memcpy(request, data->pristine_ps + (size_t)src * hwmgr->ps_size,
	       hwmgr->ps_size);
}

static int si_apply_state_adjust_rules(struct pp_hwmgr *hwmgr,
				       struct pp_power_state *prequest_ps,
				       const struct pp_power_state *pcurrent_ps)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	const struct amd_pp_display_configuration *display_cfg = hwmgr->display_config;
	struct si_power_state *ps;
	struct phm_dynamic_state_info *dyn = &hwmgr->dyn_state;
	struct phm_clock_and_voltage_limits *max_limits;
	const struct amd_vce_state *vce_state = NULL;
	bool disable_mclk_switching = false;
	bool disable_sclk_switching = false;
	uint32_t mclk, sclk;
	uint32_t pre_od_top_sclk = 0, pre_od_top_mclk = 0;
	uint16_t vddc, vddci, min_vce_voltage = 0;
	uint32_t max_sclk_vddc, max_mclk_vddci, max_mclk_vddc;
	uint32_t max_sclk = 0, max_mclk = 0;
	uint32_t high_pixelclock_count = 0;
	int i;

	si_reseed_request_state(hwmgr, prequest_ps);
	ps = cast_phw_si_power_state(&prequest_ps->hardware);
	if (!ps)
		return -EINVAL;

	if (ps->performance_level_count == 0)
		return 0;

	if (hwmgr->chip_id == CHIP_HAINAN) {
		if ((adev->pdev->revision == 0x81) ||
		    (adev->pdev->revision == 0xC3) ||
		    (adev->pdev->device == 0x6660) ||
		    (adev->pdev->device == 0x6664) ||
		    (adev->pdev->device == 0x6665) ||
		    (adev->pdev->device == 0x6667) ||
		    (adev->pdev->device == 0x666F)) {
			max_sclk = 75000;
		}
		if ((adev->pdev->revision == 0xC3) ||
		    (adev->pdev->device == 0x6665)) {
			max_sclk = 60000;
			max_mclk = 80000;
		}
		if ((adev->pdev->device == 0x666f) &&
		    (adev->pdev->revision == 0x00)) {
			max_sclk = 80000;
			max_mclk = 95000;
		}
	} else if (hwmgr->chip_id == CHIP_OLAND) {
		if ((adev->pdev->revision == 0xC7) ||
		    (adev->pdev->revision == 0x80) ||
		    (adev->pdev->revision == 0x81) ||
		    (adev->pdev->revision == 0x83) ||
		    (adev->pdev->revision == 0x87 &&
				adev->pdev->device != 0x6611) ||
		    (adev->pdev->device == 0x6604) ||
		    (adev->pdev->device == 0x6605)) {
			max_sclk = 75000;
		} else if (adev->pdev->revision == 0x87 &&
				adev->pdev->device == 0x6611) {
			/* Radeon 430 and 520 */
			max_sclk = 78000;
		}
	}

	/* We define "high pixelclock" for SI as higher than necessary for 4K 30Hz.
	 * For example, 4K 60Hz and 1080p 144Hz fall into this category.
	 * Find number of such displays connected.
	 */
	for (i = 0; i < display_cfg->num_display; i++) {
		/* The array only contains active displays. */
		if (display_cfg->displays[i].pixel_clock > 297000)
			high_pixelclock_count++;
	}

	/* These are some ad-hoc fixes to some issues observed with SI GPUs.
	 * They are necessary because we don't have something like dce_calcs
	 * for these GPUs to calculate bandwidth requirements.
	 */
	if (high_pixelclock_count) {
		/* Work around flickering lines at the bottom edge
		 * of the screen when using a single 4K 60Hz monitor.
		 */
		disable_mclk_switching = true;

		/* On Oland, we observe some flickering when two 4K 60Hz
		 * displays are connected, possibly because voltage is too low.
		 * Raise the voltage by requiring a higher SCLK.
		 * (Voltage cannot be adjusted independently without also SCLK.)
		 */
		if (high_pixelclock_count > 1 && hwmgr->chip_id == CHIP_OLAND)
			disable_sclk_switching = true;
	}

	/* amdgpu_dpm_enable_vce() keeps adev->pm.dpm.vce_active/vce_level
	 * for the whole SI family regardless of backend.
	 */
	if (adev->pm.dpm.vce_active &&
	    adev->pm.dpm.vce_level < hwmgr->num_vce_state_tables) {
		vce_state = &hwmgr->vce_states[adev->pm.dpm.vce_level];
		ps->evclk = vce_state->evclk;
		ps->ecclk = vce_state->ecclk;
		si_get_vce_clock_voltage(hwmgr, vce_state->evclk, vce_state->ecclk,
					 &min_vce_voltage);
	} else {
		ps->evclk = 0;
		ps->ecclk = 0;
	}

	if ((display_cfg->num_display > 1) ||
	    si_dpm_vblank_too_short(hwmgr))
		disable_mclk_switching = true;

	if (ps->vclk || ps->dclk) {
		disable_mclk_switching = true;
		disable_sclk_switching = true;
	}

	/* amdgpu.si_mclk_switching: a memory clock switch that misses the
	 * vblank window hangs the gfx ring; 0 pins every level to the top
	 * MCLK (the disable_mclk_switching path below), 1 forces switching
	 * on regardless of the display checks above.
	 */
	if (amdgpu_si_mclk_switching == 0)
		disable_mclk_switching = true;
	else if (amdgpu_si_mclk_switching == 1 && !(ps->vclk || ps->dclk))
		disable_mclk_switching = false;

	pr_debug("si state adjust: %u displays, vblank %u us, dispclk %u, high_pixelclock %u, vce %u/%u, uvd_active %d, mclk_sw %s, sclk_sw %s (mclk_parm %d)\n",
		 display_cfg->num_display, display_cfg->min_vblank_time,
		 display_cfg->display_clk, high_pixelclock_count,
		 ps->vclk, ps->dclk, adev->pm.dpm.uvd_active,
		 disable_mclk_switching ? "off" : "on",
		 disable_sclk_switching ? "off" : "on",
		 amdgpu_si_mclk_switching);

	if (adev->pm.ac_power)
		max_limits = &dyn->max_clock_voltage_on_ac;
	else
		max_limits = &dyn->max_clock_voltage_on_dc;

	for (i = ps->performance_level_count - 2; i >= 0; i--) {
		if (ps->performance_levels[i].vddc > ps->performance_levels[i+1].vddc)
			ps->performance_levels[i].vddc = ps->performance_levels[i+1].vddc;
	}
	if (adev->pm.ac_power == false) {
		for (i = 0; i < ps->performance_level_count; i++) {
			if (ps->performance_levels[i].mclk > max_limits->mclk)
				ps->performance_levels[i].mclk = max_limits->mclk;
			if (ps->performance_levels[i].sclk > max_limits->sclk)
				ps->performance_levels[i].sclk = max_limits->sclk;
			if (ps->performance_levels[i].vddc > max_limits->vddc)
				ps->performance_levels[i].vddc = max_limits->vddc;
			if (ps->performance_levels[i].vddci > max_limits->vddci)
				ps->performance_levels[i].vddci = max_limits->vddci;
		}
	}

	/* limit clocks to max supported clocks based on voltage dependency tables */
	si_get_max_clock_from_voltage_dependency_table(dyn->vddc_dependency_on_sclk,
						       &max_sclk_vddc);
	si_get_max_clock_from_voltage_dependency_table(dyn->vddci_dependency_on_mclk,
						       &max_mclk_vddci);
	si_get_max_clock_from_voltage_dependency_table(dyn->vddc_dependency_on_mclk,
						       &max_mclk_vddc);

	for (i = 0; i < ps->performance_level_count; i++) {
		if (max_sclk_vddc) {
			if (ps->performance_levels[i].sclk > max_sclk_vddc)
				ps->performance_levels[i].sclk = max_sclk_vddc;
		}
		if (max_mclk_vddci) {
			if (ps->performance_levels[i].mclk > max_mclk_vddci)
				ps->performance_levels[i].mclk = max_mclk_vddci;
		}
		if (max_mclk_vddc) {
			if (ps->performance_levels[i].mclk > max_mclk_vddc)
				ps->performance_levels[i].mclk = max_mclk_vddc;
		}
		if (max_mclk) {
			if (ps->performance_levels[i].mclk > max_mclk)
				ps->performance_levels[i].mclk = max_mclk;
		}
		if (max_sclk) {
			if (ps->performance_levels[i].sclk > max_sclk)
				ps->performance_levels[i].sclk = max_sclk;
		}
	}

	/* XXX validate the min clocks required for display */

	if (disable_mclk_switching) {
		mclk  = ps->performance_levels[ps->performance_level_count - 1].mclk;
		vddci = ps->performance_levels[ps->performance_level_count - 1].vddci;
	} else {
		mclk = ps->performance_levels[0].mclk;
		vddci = ps->performance_levels[0].vddci;
	}

	if (disable_sclk_switching) {
		sclk = ps->performance_levels[ps->performance_level_count - 1].sclk;
		vddc = ps->performance_levels[ps->performance_level_count - 1].vddc;
	} else {
		sclk = ps->performance_levels[0].sclk;
		vddc = ps->performance_levels[0].vddc;
	}

	if (vce_state) {
		if (sclk < vce_state->sclk)
			sclk = vce_state->sclk;
		if (mclk < vce_state->mclk)
			mclk = vce_state->mclk;
	}

	/* adjusted low state */
	ps->performance_levels[0].sclk = sclk;
	ps->performance_levels[0].mclk = mclk;
	ps->performance_levels[0].vddc = vddc;
	ps->performance_levels[0].vddci = vddci;

	if (disable_sclk_switching) {
		sclk = ps->performance_levels[0].sclk;
		for (i = 1; i < ps->performance_level_count; i++) {
			if (sclk < ps->performance_levels[i].sclk)
				sclk = ps->performance_levels[i].sclk;
		}
		for (i = 0; i < ps->performance_level_count; i++) {
			ps->performance_levels[i].sclk = sclk;
			ps->performance_levels[i].vddc = vddc;
		}
	} else {
		for (i = 1; i < ps->performance_level_count; i++) {
			if (ps->performance_levels[i].sclk < ps->performance_levels[i - 1].sclk)
				ps->performance_levels[i].sclk = ps->performance_levels[i - 1].sclk;
			if (ps->performance_levels[i].vddc < ps->performance_levels[i - 1].vddc)
				ps->performance_levels[i].vddc = ps->performance_levels[i - 1].vddc;
		}
	}

	if (disable_mclk_switching) {
		mclk = ps->performance_levels[0].mclk;
		for (i = 1; i < ps->performance_level_count; i++) {
			if (mclk < ps->performance_levels[i].mclk)
				mclk = ps->performance_levels[i].mclk;
		}
		for (i = 0; i < ps->performance_level_count; i++) {
			ps->performance_levels[i].mclk = mclk;
			ps->performance_levels[i].vddci = vddci;
		}
	} else {
		for (i = 1; i < ps->performance_level_count; i++) {
			if (ps->performance_levels[i].mclk < ps->performance_levels[i - 1].mclk)
				ps->performance_levels[i].mclk = ps->performance_levels[i - 1].mclk;
			if (ps->performance_levels[i].vddci < ps->performance_levels[i - 1].vddci)
				ps->performance_levels[i].vddci = ps->performance_levels[i - 1].vddci;
		}
	}

	for (i = 0; i < ps->performance_level_count; i++)
		si_adjust_clock_combinations(hwmgr, max_limits,
					     &ps->performance_levels[i]);

	for (i = 0; i < ps->performance_level_count; i++) {
		if (ps->performance_levels[i].vddc < min_vce_voltage)
			ps->performance_levels[i].vddc = min_vce_voltage;
		si_apply_voltage_dependency_rules(dyn->vddc_dependency_on_sclk,
						  ps->performance_levels[i].sclk,
						  max_limits->vddc,  &ps->performance_levels[i].vddc);
		si_apply_voltage_dependency_rules(dyn->vddci_dependency_on_mclk,
						  ps->performance_levels[i].mclk,
						  max_limits->vddci, &ps->performance_levels[i].vddci);
		si_apply_voltage_dependency_rules(dyn->vddc_dependency_on_mclk,
						  ps->performance_levels[i].mclk,
						  max_limits->vddc,  &ps->performance_levels[i].vddc);
		si_apply_voltage_dependency_rules(dyn->vddc_dependency_on_display_clock,
						  display_cfg->display_clk,
						  max_limits->vddc,  &ps->performance_levels[i].vddc);
	}

	for (i = 0; i < ps->performance_level_count; i++) {
		si_apply_voltage_delta_rules(hwmgr,
					     max_limits->vddc, max_limits->vddci,
					     &ps->performance_levels[i].vddc,
					     &ps->performance_levels[i].vddci);
	}

	/* Oland OverDrive: allow top performance level to exceed stock
	 * limits up to SI_OLAND_OD_SCLK_MAX / SI_OLAND_OD_MCLK_MAX.
	 * od_sclk/od_mclk are in 10 kHz units, 0 = disabled.
	 *
	 * OD levels intentionally exceed the stock voltage-mapped and
	 * board maxima above, matching Windows (ADL OD caps come from
	 * the same VBIOS PPTable). Requested clocks are never reduced
	 * here. Voltage is still raised to match via the dependency
	 * rules below.
	 */
	if (ps->performance_level_count) {
		pre_od_top_sclk = ps->performance_levels[ps->performance_level_count - 1].sclk;
		pre_od_top_mclk = ps->performance_levels[ps->performance_level_count - 1].mclk;
	}
	if (hwmgr->chip_id == CHIP_OLAND && ps->performance_level_count) {
		int top = ps->performance_level_count - 1;

		if (data->od_sclk) {
			uint32_t od_sclk = min(data->od_sclk, (uint32_t)SI_OLAND_OD_SCLK_MAX);

			ps->performance_levels[top].sclk = od_sclk;
			ps->performance_levels[top].vddc = max_limits->vddc;
			si_apply_voltage_dependency_rules(
				dyn->vddc_dependency_on_sclk,
				od_sclk, max_limits->vddc,
				&ps->performance_levels[top].vddc);
			pr_debug("si OD sclk: pre %u MHz -> od %u MHz @ %u mV (req %u)\n",
				 pre_od_top_sclk / 100, od_sclk / 100,
				 ps->performance_levels[top].vddc, data->od_sclk / 100);
		}
		if (data->od_mclk) {
			uint32_t od_mclk = min(data->od_mclk, (uint32_t)SI_OLAND_OD_MCLK_MAX);

			if (disable_mclk_switching) {
				for (i = 0; i < ps->performance_level_count; i++) {
					ps->performance_levels[i].mclk = od_mclk;
					ps->performance_levels[i].vddci = max_limits->vddci;
					si_apply_voltage_dependency_rules(
						dyn->vddci_dependency_on_mclk,
						od_mclk, max_limits->vddci,
						&ps->performance_levels[i].vddci);
					si_apply_voltage_dependency_rules(
						dyn->vddc_dependency_on_mclk,
						od_mclk, max_limits->vddc,
						&ps->performance_levels[i].vddc);
				}
				pr_debug("si OD mclk: all %d levels pinned to od %u MHz (switching off)\n",
					 ps->performance_level_count, od_mclk / 100);
			} else {
				ps->performance_levels[top].mclk = od_mclk;
				ps->performance_levels[top].vddci = max_limits->vddci;
				si_apply_voltage_dependency_rules(
					dyn->vddci_dependency_on_mclk,
					od_mclk, max_limits->vddci,
					&ps->performance_levels[top].vddci);
				si_apply_voltage_dependency_rules(
					dyn->vddc_dependency_on_mclk,
					od_mclk, max_limits->vddc,
					&ps->performance_levels[top].vddc);
				pr_debug("si OD mclk: top -> od %u MHz @ vddci %u vddc %u (switching on)\n",
					 od_mclk / 100,
					 ps->performance_levels[top].vddci,
					 ps->performance_levels[top].vddc);
			}
		}
	}

	/* The switching-disable flags are set for states that must not change
	 * clocks at all while they run - UVD decode and the high-pixelclock
	 * cases - and the code above honours them by flattening every level
	 * to the state's top clock. The OverDrive block then raises only the
	 * top level, which puts a step back into a state that asked for none:
	 * the SMC switches mid-playback and video glitches on every switch.
	 * Flatten again after OverDrive so the whole state runs at one point.
	 */
	if (disable_sclk_switching || disable_mclk_switching) {
		uint32_t max_sclk = 0, max_mclk = 0, max_vddc = 0, max_vddci = 0;

		for (i = 0; i < ps->performance_level_count; i++) {
			max_sclk = max(max_sclk, ps->performance_levels[i].sclk);
			max_mclk = max(max_mclk, ps->performance_levels[i].mclk);
			max_vddc = max_t(uint32_t, max_vddc, ps->performance_levels[i].vddc);
			max_vddci = max_t(uint32_t, max_vddci, ps->performance_levels[i].vddci);
		}
		for (i = 0; i < ps->performance_level_count; i++) {
			if (disable_sclk_switching) {
				ps->performance_levels[i].sclk = max_sclk;
				ps->performance_levels[i].vddc = max_vddc;
			}
			if (disable_mclk_switching) {
				ps->performance_levels[i].mclk = max_mclk;
				ps->performance_levels[i].vddci = max_vddci;
			}
		}
	}

	/* amdgpu.si_dpm_quirks ladder experiments: the SMC's own stepped
	 * ramp through the intermediate levels under load hangs some boards
	 * where a single forced jump to the top does not; these reshape the
	 * ladder so that the SMC has fewer, or voltage-free, steps to make.
	 */
	if (ps->performance_level_count > 2 &&
	    ps->performance_level_count == ps->vbios_level_count &&
	    (data->dpm_quirks & SI_QUIRK_COLLAPSED_LADDER)) {
		int top = ps->performance_level_count - 1;
		int keep = (data->dpm_quirks & SI_QUIRK_DROP_LEVEL_BELOW_TOP) ?
			   top - 1 : 1;	/* lowest levels that survive: 0..keep-1 */

		/* Really shorten the ladder rather than duplicating the top
		 * level: si_populate_smc_t() derives the SMC's up/down activity
		 * thresholds from the clock ratio of adjacent levels, and
		 * identical neighbours give it degenerate thresholds that make
		 * it flap between them, a full switch sequence each time.
		 * Remember the two VBIOS neighbours the thresholds of the new
		 * last pair are rebuilt from. Only ever collapse the VBIOS
		 * ladder (the PSM re-adjusts a table entry in place).
		 */
		ps->collapsed_next_sclk = ps->performance_levels[keep].sclk;
		ps->collapsed_below_top_sclk = ps->performance_levels[top - 1].sclk;
		ps->performance_levels[keep] = ps->performance_levels[top];
		ps->performance_level_count = keep + 1;
	}

	/* The OverDrive ladder Windows runs on this board (GPU-Z, Adrenalin
	 * OD 1000/1300 on Oland 0x6611 rev 0x87): every VBIOS level stays,
	 * the top one takes the OD clocks, and the level below it is scaled
	 * by the OD ratios at its stock voltages, i.e.
	 *   300/150 @ 800 mV, 400/1100 @ 900, 934/1298 @ 1100, 1000/1300 @ 1150
	 * from the VBIOS 300/150, 400/1100, 730/1100, 780/1100. The ratios
	 * are whole percent, as Windows computes them: 1000/780 -> 128 %,
	 * 730 * 1.28 = 934.4; 1300/1100 -> 118 %, 1100 * 1.18 = 1298.
	 * PowerTune then answers a power virus with a switch to that level
	 * instead of pulse-skipping the top one down to 450 MHz.
	 *
	 * Only a ladder that still switches is scaled. UVD and the
	 * high-pixelclock cases flattened every level to the top clock
	 * above, so levels[top - 1] is no VBIOS neighbour there and scaling
	 * it fabricates an above-top clock ("vbios 1000 MHz, pre_od_top
	 * 730 MHz, scaled 1369 MHz"); likewise the MCLK is left alone when
	 * MCLK switching is off. This relies on si_reseed_request_state()
	 * handing us the VBIOS levels on every adjust, not a state that was
	 * already clamped and scaled.
	 */
	if ((data->dpm_quirks & SI_QUIRK_SCALED_MID_LEVEL) &&
	    !disable_sclk_switching &&
	    ps->performance_level_count > 2 &&
	    ps->performance_level_count == ps->vbios_level_count) {
		int top = ps->performance_level_count - 1;
		const struct si_performance_level *hi = &ps->performance_levels[top];
		struct si_performance_level *mid = &ps->performance_levels[top - 1];
		uint32_t vbios_sclk = mid->sclk, vbios_mclk = mid->mclk;

		if (pre_od_top_sclk && hi->sclk > pre_od_top_sclk)
			mid->sclk = min_t(uint32_t, hi->sclk,
					  mid->sclk * (hi->sclk * 100 / pre_od_top_sclk) / 100);
		if (!disable_mclk_switching &&
		    pre_od_top_mclk && hi->mclk > pre_od_top_mclk)
			mid->mclk = min_t(uint32_t, hi->mclk,
					  mid->mclk * (hi->mclk * 100 / pre_od_top_mclk) / 100);
		pr_debug("si scaled mid: vbios %u/%u MHz -> %u/%u MHz @ %u/%u mV (top %u/%u MHz)\n",
			 vbios_sclk / 100, vbios_mclk / 100,
			 mid->sclk / 100, mid->mclk / 100, mid->vddc, mid->vddci,
			 hi->sclk / 100, hi->mclk / 100);
	}
	if (ps->performance_level_count > 1 &&
	    (data->dpm_quirks & SI_QUIRK_SINGLE_VDDC)) {
		int top = ps->performance_level_count - 1;

		for (i = 0; i < top; i++) {
			ps->performance_levels[i].vddc = ps->performance_levels[top].vddc;
			ps->performance_levels[i].vddci = ps->performance_levels[top].vddci;
		}
	}

	/* Never hand the SMC a level that is identical to the one below it:
	 * si_populate_smc_t() derives the up/down activity thresholds from
	 * the clock ratio of each pair, so equal neighbours give it nothing
	 * to discriminate on and it switches between them for no gain. A
	 * ladder quirk on top of the UVD/high-pixelclock flattening can
	 * produce exactly that, and the gfx ring dies under it.
	 */
	if (data->dpm_quirks & SI_QUIRK_RESHAPED_LADDER) {
		uint32_t keep = 1;

		for (i = 1; i < ps->performance_level_count; i++) {
			if (ps->performance_levels[i].sclk ==
			    ps->performance_levels[keep - 1].sclk &&
			    ps->performance_levels[i].mclk ==
			    ps->performance_levels[keep - 1].mclk)
				continue;
			ps->performance_levels[keep++] = ps->performance_levels[i];
		}
		/* A state whose levels are all one point (UVD, high pixel
		 * clock) keeps two of them: that is the shape upstream has
		 * always uploaded for those states, and a single-level state
		 * is not something this firmware is ever given.
		 */
		if (keep < 2)
			keep = min_t(uint32_t, 2, ps->performance_level_count);

		if (keep != ps->performance_level_count) {
			pr_debug("si dropped %u duplicate level(s), %u left\n",
				 ps->performance_level_count - keep, keep);
			ps->performance_level_count = keep;
		}
	}

	ps->dc_compatible = true;
	for (i = 0; i < ps->performance_level_count; i++) {
		if (ps->performance_levels[i].vddc > dyn->max_clock_voltage_on_dc.vddc)
			ps->dc_compatible = false;
	}

	pr_debug("si adjusted state: %u displays, vblank %u us, mclk switching %s, sclk switching %s\n",
		 display_cfg->num_display, display_cfg->min_vblank_time,
		 disable_mclk_switching ? "off" : "on",
		 disable_sclk_switching ? "off" : "on");
	for (i = 0; i < ps->performance_level_count; i++)
		pr_debug("si   level %d: sclk %u mclk %u vddc %u vddci %u pcie gen%u\n", i,
			 ps->performance_levels[i].sclk / 100,
			 ps->performance_levels[i].mclk / 100,
			 ps->performance_levels[i].vddc,
			 ps->performance_levels[i].vddci,
			 ps->performance_levels[i].pcie_gen + 1);

	return 0;
}

/* ---- register helpers ---- */

static inline uint32_t si_rreg(struct pp_hwmgr *hwmgr, uint32_t reg)
{
	return cgs_read_register(hwmgr->device, reg);
}

static inline void si_wreg(struct pp_hwmgr *hwmgr, uint32_t reg, uint32_t val)
{
	cgs_write_register(hwmgr->device, reg, val);
}

/* legacy WREG32_P(reg, val, mask): reg = (reg & mask) | val */
static inline void si_wreg_p(struct pp_hwmgr *hwmgr, uint32_t reg,
			     uint32_t val, uint32_t mask)
{
	uint32_t tmp = si_rreg(hwmgr, reg);

	tmp &= mask;
	tmp |= val;
	si_wreg(hwmgr, reg, tmp);
}

static inline uint32_t si_rreg_smc(struct pp_hwmgr *hwmgr, uint32_t addr)
{
	return cgs_read_ind_register(hwmgr->device, CGS_IND_REG__SMC, addr);
}

static inline void si_wreg_smc(struct pp_hwmgr *hwmgr, uint32_t addr, uint32_t val)
{
	cgs_write_ind_register(hwmgr->device, CGS_IND_REG__SMC, addr, val);
}

static int si_send_msg(struct pp_hwmgr *hwmgr, uint16_t msg)
{
	return smum_send_msg_to_smc(hwmgr, msg, NULL);
}

static int si_send_msg_with_parameter(struct pp_hwmgr *hwmgr, uint16_t msg,
				      uint32_t parameter)
{
	return smum_send_msg_to_smc_with_parameter(hwmgr, msg, parameter, NULL);
}

static const struct si_power_state *cast_const_phw_si_power_state(
				const struct pp_hw_power_state *hw_ps)
{
	PP_ASSERT_WITH_CODE((PhwSIslands_Magic == hw_ps->magic),
			    "Invalid Powerstate Type!",
			    return NULL);

	return (const struct si_power_state *)hw_ps;
}

/* ---- DPM / thermal event source programming (legacy si_dpm.c) ---- */

static void si_set_dpm_event_sources(struct pp_hwmgr *hwmgr, uint32_t sources)
{
	struct si_hwmgr *data = hwmgr->backend;
	bool want_thermal_protection;
	enum si_dpm_event_src dpm_event_src = SI_DPM_EVENT_SRC_DIGITAL;

	switch (sources) {
	case 0:
	default:
		want_thermal_protection = false;
		break;
	case (1 << SI_DPM_AUTO_THROTTLE_SRC_THERMAL):
		want_thermal_protection = true;
		dpm_event_src = SI_DPM_EVENT_SRC_DIGITAL;
		break;
	case (1 << SI_DPM_AUTO_THROTTLE_SRC_EXTERNAL):
		want_thermal_protection = true;
		dpm_event_src = SI_DPM_EVENT_SRC_EXTERNAL;
		break;
	case ((1 << SI_DPM_AUTO_THROTTLE_SRC_EXTERNAL) |
	      (1 << SI_DPM_AUTO_THROTTLE_SRC_THERMAL)):
		want_thermal_protection = true;
		dpm_event_src = SI_DPM_EVENT_SRC_DIGIAL_OR_EXTERNAL;
		break;
	}

	if (want_thermal_protection) {
		si_wreg_p(hwmgr, mmCG_THERMAL_CTRL,
			  dpm_event_src << CG_THERMAL_CTRL__DPM_EVENT_SRC__SHIFT,
			  ~CG_THERMAL_CTRL__DPM_EVENT_SRC_MASK);
		if (data->thermal_protection)
			si_wreg_p(hwmgr, mmGENERAL_PWRMGT, 0,
				  ~GENERAL_PWRMGT__THERMAL_PROTECTION_DIS_MASK);
	} else {
		si_wreg_p(hwmgr, mmGENERAL_PWRMGT,
			  GENERAL_PWRMGT__THERMAL_PROTECTION_DIS_MASK,
			  ~GENERAL_PWRMGT__THERMAL_PROTECTION_DIS_MASK);
	}
}

static void si_enable_auto_throttle_source(struct pp_hwmgr *hwmgr,
					   enum si_dpm_auto_throttle_src source,
					   bool enable)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (enable) {
		if (!(data->active_auto_throttle_sources & (1 << source))) {
			data->active_auto_throttle_sources |= 1 << source;
			si_set_dpm_event_sources(hwmgr, data->active_auto_throttle_sources);
		}
	} else {
		if (data->active_auto_throttle_sources & (1 << source)) {
			data->active_auto_throttle_sources &= ~(1 << source);
			si_set_dpm_event_sources(hwmgr, data->active_auto_throttle_sources);
		}
	}
}

static void si_start_dpm(struct pp_hwmgr *hwmgr)
{
	si_wreg_p(hwmgr, mmGENERAL_PWRMGT, GENERAL_PWRMGT__GLOBAL_PWRMGT_EN_MASK,
		  ~GENERAL_PWRMGT__GLOBAL_PWRMGT_EN_MASK);
}

static void si_stop_dpm(struct pp_hwmgr *hwmgr)
{
	si_wreg_p(hwmgr, mmGENERAL_PWRMGT, 0, ~GENERAL_PWRMGT__GLOBAL_PWRMGT_EN_MASK);
}

static void si_enable_sclk_control(struct pp_hwmgr *hwmgr, bool enable)
{
	if (enable)
		si_wreg_p(hwmgr, mmSCLK_PWRMGT_CNTL, 0,
			  ~SCLK_PWRMGT_CNTL__SCLK_PWRMGT_OFF_MASK);
	else
		si_wreg_p(hwmgr, mmSCLK_PWRMGT_CNTL,
			  SCLK_PWRMGT_CNTL__SCLK_PWRMGT_OFF_MASK,
			  ~SCLK_PWRMGT_CNTL__SCLK_PWRMGT_OFF_MASK);
}

static int si_restrict_performance_levels_before_switch(struct pp_hwmgr *hwmgr)
{
	if (si_send_msg(hwmgr, PPSMC_MSG_NoForcedLevel))
		return -EINVAL;

	return si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetEnabledLevels, 1) ?
		-EINVAL : 0;
}

static int si_set_sw_state(struct pp_hwmgr *hwmgr)
{
	return si_send_msg(hwmgr, PPSMC_MSG_SwitchToSwState) ? -EINVAL : 0;
}

static int si_reset_to_default(struct pp_hwmgr *hwmgr)
{
	return si_send_msg(hwmgr, PPSMC_MSG_ResetToDefaults) ? -EINVAL : 0;
}

static int si_disable_ulv(struct pp_hwmgr *hwmgr)
{
	return si_send_msg(hwmgr, PPSMC_MSG_DisableULV) ? -EINVAL : 0;
}

static int si_notify_smc_display_change(struct pp_hwmgr *hwmgr, bool has_display)
{
	PPSMC_Msg msg = has_display ? PPSMC_MSG_HasDisplay : PPSMC_MSG_NoDisplay;

	return si_send_msg(hwmgr, msg) ? -EINVAL : 0;
}

static void si_read_clock_registers(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;

	data->clock_registers.cg_spll_func_cntl = si_rreg(hwmgr, mmCG_SPLL_FUNC_CNTL);
	data->clock_registers.cg_spll_func_cntl_2 = si_rreg(hwmgr, mmCG_SPLL_FUNC_CNTL_2);
	data->clock_registers.cg_spll_func_cntl_3 = si_rreg(hwmgr, mmCG_SPLL_FUNC_CNTL_3);
	data->clock_registers.cg_spll_func_cntl_4 = si_rreg(hwmgr, mmCG_SPLL_FUNC_CNTL_4);
	data->clock_registers.cg_spll_spread_spectrum = si_rreg(hwmgr, mmCG_SPLL_SPREAD_SPECTRUM);
	data->clock_registers.cg_spll_spread_spectrum_2 = si_rreg(hwmgr, mmCG_SPLL_SPREAD_SPECTRUM_2);
	data->clock_registers.dll_cntl = si_rreg(hwmgr, mmDLL_CNTL);
	data->clock_registers.mclk_pwrmgt_cntl = si_rreg(hwmgr, mmMCLK_PWRMGT_CNTL);
	data->clock_registers.mpll_ad_func_cntl = si_rreg(hwmgr, mmMPLL_AD_FUNC_CNTL);
	data->clock_registers.mpll_dq_func_cntl = si_rreg(hwmgr, mmMPLL_DQ_FUNC_CNTL);
	data->clock_registers.mpll_func_cntl = si_rreg(hwmgr, mmMPLL_FUNC_CNTL);
	data->clock_registers.mpll_func_cntl_1 = si_rreg(hwmgr, mmMPLL_FUNC_CNTL_1);
	data->clock_registers.mpll_func_cntl_2 = si_rreg(hwmgr, mmMPLL_FUNC_CNTL_2);
	data->clock_registers.mpll_ss1 = si_rreg(hwmgr, mmMPLL_SS1);
	data->clock_registers.mpll_ss2 = si_rreg(hwmgr, mmMPLL_SS2);
}

static void si_enable_thermal_protection(struct pp_hwmgr *hwmgr, bool enable)
{
	if (enable)
		si_wreg_p(hwmgr, mmGENERAL_PWRMGT, 0,
			  ~GENERAL_PWRMGT__THERMAL_PROTECTION_DIS_MASK);
	else
		si_wreg_p(hwmgr, mmGENERAL_PWRMGT,
			  GENERAL_PWRMGT__THERMAL_PROTECTION_DIS_MASK,
			  ~GENERAL_PWRMGT__THERMAL_PROTECTION_DIS_MASK);
}

static void si_enable_acpi_power_management(struct pp_hwmgr *hwmgr)
{
	si_wreg_p(hwmgr, mmGENERAL_PWRMGT, GENERAL_PWRMGT__STATIC_PM_EN_MASK,
		  ~GENERAL_PWRMGT__STATIC_PM_EN_MASK);
}

static void si_program_ds_registers(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t tmp;

	/* DEEP_SLEEP_CLK_SEL field should be 0x10 on tahiti A0 */
	if (hwmgr->chip_id == CHIP_TAHITI && adev->rev_id == 0x0)
		tmp = 0x10;
	else
		tmp = 0x1;

	if (data->sclk_deep_sleep) {
		si_wreg_p(hwmgr, mmMISC_CLK_CNTL,
			  tmp << MISC_CLK_CNTL__DEEP_SLEEP_CLK_SEL__SHIFT,
			  ~MISC_CLK_CNTL__DEEP_SLEEP_CLK_SEL_MASK);
		si_wreg_p(hwmgr, mmCG_SPLL_AUTOSCALE_CNTL,
			  CG_SPLL_AUTOSCALE_CNTL__AUTOSCALE_ON_SS_CLEAR_MASK,
			  ~CG_SPLL_AUTOSCALE_CNTL__AUTOSCALE_ON_SS_CLEAR_MASK);
	}
}

/* legacy si_program_display_gap(), wired to display_config_changed */
static int si_program_display_gap(struct pp_hwmgr *hwmgr)
{
	const struct amd_pp_display_configuration *cfg = hwmgr->display_config;
	uint32_t tmp, pipe;

	tmp = si_rreg(hwmgr, mmCG_DISPLAY_GAP_CNTL) &
		~(CG_DISPLAY_GAP_CNTL__DISP1_GAP_MASK | CG_DISPLAY_GAP_CNTL__DISP2_GAP_MASK);
	if (cfg->num_display > 0)
		tmp |= R600_PM_DISPLAY_GAP_VBLANK_OR_WM << CG_DISPLAY_GAP_CNTL__DISP1_GAP__SHIFT;
	else
		tmp |= R600_PM_DISPLAY_GAP_IGNORE << CG_DISPLAY_GAP_CNTL__DISP1_GAP__SHIFT;

	if (cfg->num_display > 1)
		tmp |= R600_PM_DISPLAY_GAP_VBLANK_OR_WM << CG_DISPLAY_GAP_CNTL__DISP2_GAP__SHIFT;
	else
		tmp |= R600_PM_DISPLAY_GAP_IGNORE << CG_DISPLAY_GAP_CNTL__DISP2_GAP__SHIFT;

	si_wreg(hwmgr, mmCG_DISPLAY_GAP_CNTL, tmp);

	tmp = si_rreg(hwmgr, SI_DCCG_DISP_SLOW_SELECT_REG);
	pipe = (tmp & SI_DCCG_DISP1_SLOW_SELECT_MASK) >> SI_DCCG_DISP1_SLOW_SELECT_SHIFT;

	if (cfg->num_display > 0 && pipe != cfg->crtc_index) {
		pipe = cfg->crtc_index;

		tmp &= ~SI_DCCG_DISP1_SLOW_SELECT_MASK;
		tmp |= pipe << SI_DCCG_DISP1_SLOW_SELECT_SHIFT;
		si_wreg(hwmgr, SI_DCCG_DISP_SLOW_SELECT_REG, tmp);
	}

	/* Setting this to false forces the performance state to low if the crtcs are disabled.
	 * This can be a problem on PowerXpress systems or if you want to use the card
	 * for offscreen rendering or compute if there are no crtcs enabled.
	 */
	return si_notify_smc_display_change(hwmgr, cfg->num_display > 0);
}

static void si_enable_spread_spectrum(struct pp_hwmgr *hwmgr, bool enable)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (enable) {
		if (data->sclk_ss)
			si_wreg_p(hwmgr, mmGENERAL_PWRMGT,
				  GENERAL_PWRMGT__DYN_SPREAD_SPECTRUM_EN_MASK,
				  ~GENERAL_PWRMGT__DYN_SPREAD_SPECTRUM_EN_MASK);
	} else {
		si_wreg_p(hwmgr, mmCG_SPLL_SPREAD_SPECTRUM, 0,
			  ~CG_SPLL_SPREAD_SPECTRUM__SSEN_MASK);
		si_wreg_p(hwmgr, mmGENERAL_PWRMGT, 0,
			  ~GENERAL_PWRMGT__DYN_SPREAD_SPECTRUM_EN_MASK);
	}
}

/* legacy r600_calculate_u_and_p() */
static void si_calculate_u_and_p(uint32_t i, uint32_t r_c, uint32_t p_b,
				 uint32_t *p, uint32_t *u)
{
	uint32_t b_c = 0;
	uint32_t i_c;
	uint32_t tmp;

	i_c = (i * r_c) / 100;
	tmp = i_c >> p_b;

	while (tmp) {
		b_c++;
		tmp >>= 1;
	}

	*u = (b_c + 1) / 2;
	*p = i_c / (1 << (2 * (*u)));
}

static void si_setup_bsp(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t xclk = amdgpu_asic_get_xclk((struct amdgpu_device *)hwmgr->adev);

	si_calculate_u_and_p(data->asi, xclk, 16, &data->bsp, &data->bsu);

	si_calculate_u_and_p(data->pasi, xclk, 16, &data->pbsp, &data->pbsu);

	data->dsp = (data->bsp << CG_BSP__BSP__SHIFT) | (data->bsu << CG_BSP__BSU__SHIFT);
	data->psp = (data->pbsp << CG_BSP__BSP__SHIFT) | (data->pbsu << CG_BSP__BSU__SHIFT);

	si_wreg(hwmgr, mmCG_BSP, data->dsp);
}

static void si_program_git(struct pp_hwmgr *hwmgr)
{
	si_wreg_p(hwmgr, mmCG_GIT, R600_GICST_DFLT << CG_GIT__CG_GICST__SHIFT,
		  ~CG_GIT__CG_GICST_MASK);
}

static const uint32_t si_r600_utc[R600_PM_NUMBER_OF_TC] = {
	R600_UTC_DFLT_00,
	R600_UTC_DFLT_01,
	R600_UTC_DFLT_02,
	R600_UTC_DFLT_03,
	R600_UTC_DFLT_04,
	R600_UTC_DFLT_05,
	R600_UTC_DFLT_06,
	R600_UTC_DFLT_07,
	R600_UTC_DFLT_08,
	R600_UTC_DFLT_09,
	R600_UTC_DFLT_10,
	R600_UTC_DFLT_11,
	R600_UTC_DFLT_12,
	R600_UTC_DFLT_13,
	R600_UTC_DFLT_14,
};

static const uint32_t si_r600_dtc[R600_PM_NUMBER_OF_TC] = {
	R600_DTC_DFLT_00,
	R600_DTC_DFLT_01,
	R600_DTC_DFLT_02,
	R600_DTC_DFLT_03,
	R600_DTC_DFLT_04,
	R600_DTC_DFLT_05,
	R600_DTC_DFLT_06,
	R600_DTC_DFLT_07,
	R600_DTC_DFLT_08,
	R600_DTC_DFLT_09,
	R600_DTC_DFLT_10,
	R600_DTC_DFLT_11,
	R600_DTC_DFLT_12,
	R600_DTC_DFLT_13,
	R600_DTC_DFLT_14,
};

static void si_program_tp(struct pp_hwmgr *hwmgr)
{
	int i;
	enum r600_td td = R600_TD_DFLT;

	for (i = 0; i < R600_PM_NUMBER_OF_TC; i++)
		si_wreg(hwmgr, mmCG_FFCT_0 + i,
			(si_r600_utc[i] << CG_FFCT_0__UTC_0__SHIFT |
			 si_r600_dtc[i] << CG_FFCT_0__DTC_0__SHIFT));

	if (td == R600_TD_AUTO)
		si_wreg_p(hwmgr, mmSCLK_PWRMGT_CNTL, 0,
			  ~SCLK_PWRMGT_CNTL__FIR_FORCE_TREND_SEL_MASK);
	else
		si_wreg_p(hwmgr, mmSCLK_PWRMGT_CNTL,
			  SCLK_PWRMGT_CNTL__FIR_FORCE_TREND_SEL_MASK,
			  ~SCLK_PWRMGT_CNTL__FIR_FORCE_TREND_SEL_MASK);

	if (td == R600_TD_UP)
		si_wreg_p(hwmgr, mmSCLK_PWRMGT_CNTL, 0,
			  ~SCLK_PWRMGT_CNTL__FIR_TREND_MODE_MASK);

	if (td == R600_TD_DOWN)
		si_wreg_p(hwmgr, mmSCLK_PWRMGT_CNTL,
			  SCLK_PWRMGT_CNTL__FIR_TREND_MODE_MASK,
			  ~SCLK_PWRMGT_CNTL__FIR_TREND_MODE_MASK);
}

static void si_program_tpp(struct pp_hwmgr *hwmgr)
{
	si_wreg(hwmgr, mmCG_TPC, R600_TPC_DFLT);
}

static void si_program_sstp(struct pp_hwmgr *hwmgr)
{
	si_wreg(hwmgr, mmCG_SSP, (R600_SSTU_DFLT << CG_SSP__SSTU__SHIFT |
				  R600_SST_DFLT << CG_SSP__SST__SHIFT));
}

static void si_enable_display_gap(struct pp_hwmgr *hwmgr)
{
	uint32_t tmp = si_rreg(hwmgr, mmCG_DISPLAY_GAP_CNTL);

	tmp &= ~(CG_DISPLAY_GAP_CNTL__DISP1_GAP_MASK | CG_DISPLAY_GAP_CNTL__DISP2_GAP_MASK);
	tmp |= (R600_PM_DISPLAY_GAP_IGNORE << CG_DISPLAY_GAP_CNTL__DISP1_GAP__SHIFT |
		R600_PM_DISPLAY_GAP_IGNORE << CG_DISPLAY_GAP_CNTL__DISP2_GAP__SHIFT);

	tmp &= ~(CG_DISPLAY_GAP_CNTL__DISP1_GAP_MCHG_MASK | CG_DISPLAY_GAP_CNTL__DISP2_GAP_MCHG_MASK);
	tmp |= (R600_PM_DISPLAY_GAP_VBLANK << CG_DISPLAY_GAP_CNTL__DISP1_GAP_MCHG__SHIFT |
		R600_PM_DISPLAY_GAP_IGNORE << CG_DISPLAY_GAP_CNTL__DISP2_GAP_MCHG__SHIFT);
	si_wreg(hwmgr, mmCG_DISPLAY_GAP_CNTL, tmp);
}

static void si_program_vc(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;

	si_wreg(hwmgr, mmCG_FTV, data->vrc);
}

static void si_clear_vc(struct pp_hwmgr *hwmgr)
{
	si_wreg(hwmgr, mmCG_FTV, 0);
}

static void si_enable_voltage_control(struct pp_hwmgr *hwmgr, bool enable)
{
	if (enable)
		si_wreg_p(hwmgr, mmGENERAL_PWRMGT, GENERAL_PWRMGT__VOLT_PWRMGT_EN_MASK,
			  ~GENERAL_PWRMGT__VOLT_PWRMGT_EN_MASK);
	else
		si_wreg_p(hwmgr, mmGENERAL_PWRMGT, 0,
			  ~GENERAL_PWRMGT__VOLT_PWRMGT_EN_MASK);
}

/* ---- voltage tables (legacy si_construct_voltage_tables) ---- */

static void si_trim_voltage_table_to_fit_state_table(uint32_t max_voltage_steps,
						     struct atom_voltage_table *voltage_table)
{
	unsigned int i, diff;

	if (voltage_table->count <= max_voltage_steps)
		return;

	diff = voltage_table->count - max_voltage_steps;

	for (i = 0; i < max_voltage_steps; i++)
		voltage_table->entries[i] = voltage_table->entries[i + diff];

	voltage_table->count = max_voltage_steps;
}

static int si_get_svi2_voltage_table(const struct phm_clock_voltage_dependency_table *voltage_dependency_table,
				     struct atom_voltage_table *voltage_table)
{
	uint32_t i;

	if (voltage_dependency_table == NULL)
		return -EINVAL;

	voltage_table->mask_low = 0;
	voltage_table->phase_delay = 0;

	voltage_table->count = voltage_dependency_table->count;
	for (i = 0; i < voltage_table->count; i++) {
		voltage_table->entries[i].value = voltage_dependency_table->entries[i].v;
		voltage_table->entries[i].smio_low = 0;
	}

	return 0;
}

static int si_construct_voltage_tables(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	int ret;

	if (data->voltage_control) {
		ret = amdgpu_atombios_get_voltage_table(adev, VOLTAGE_TYPE_VDDC,
							VOLTAGE_OBJ_GPIO_LUT, &data->vddc_voltage_table);
		if (ret)
			return ret;

		if (data->vddc_voltage_table.count > SISLANDS_MAX_NO_VREG_STEPS)
			si_trim_voltage_table_to_fit_state_table(SISLANDS_MAX_NO_VREG_STEPS,
								 &data->vddc_voltage_table);
	} else if (data->voltage_control_svi2) {
		ret = si_get_svi2_voltage_table(hwmgr->dyn_state.vddc_dependency_on_mclk,
						&data->vddc_voltage_table);
		if (ret)
			return ret;
	} else {
		return -EINVAL;
	}

	if (data->vddci_control) {
		ret = amdgpu_atombios_get_voltage_table(adev, VOLTAGE_TYPE_VDDCI,
							VOLTAGE_OBJ_GPIO_LUT, &data->vddci_voltage_table);
		if (ret)
			return ret;

		if (data->vddci_voltage_table.count > SISLANDS_MAX_NO_VREG_STEPS)
			si_trim_voltage_table_to_fit_state_table(SISLANDS_MAX_NO_VREG_STEPS,
								 &data->vddci_voltage_table);
	}
	if (data->vddci_control_svi2) {
		ret = si_get_svi2_voltage_table(hwmgr->dyn_state.vddci_dependency_on_mclk,
						&data->vddci_voltage_table);
		if (ret)
			return ret;
	}

	if (data->mvdd_control) {
		ret = amdgpu_atombios_get_voltage_table(adev, VOLTAGE_TYPE_MVDDC,
							VOLTAGE_OBJ_GPIO_LUT, &data->mvdd_voltage_table);

		if (ret) {
			data->mvdd_control = false;
			return ret;
		}

		if (data->mvdd_voltage_table.count == 0) {
			data->mvdd_control = false;
			return -EINVAL;
		}

		if (data->mvdd_voltage_table.count > SISLANDS_MAX_NO_VREG_STEPS)
			si_trim_voltage_table_to_fit_state_table(SISLANDS_MAX_NO_VREG_STEPS,
								 &data->mvdd_voltage_table);
	}

	if (data->vddc_phase_shed_control) {
		ret = amdgpu_atombios_get_voltage_table(adev, VOLTAGE_TYPE_VDDC,
							VOLTAGE_OBJ_PHASE_LUT, &data->vddc_phase_shed_table);
		if (ret)
			data->vddc_phase_shed_control = false;

		if ((data->vddc_phase_shed_table.count == 0) ||
		    (data->vddc_phase_shed_table.count > SISLANDS_MAX_NO_VREG_STEPS))
			data->vddc_phase_shed_control = false;
	}

	return 0;
}

/* ---- power containment / CAC control messages ---- */

static bool si_should_disable_uvd_powertune(struct pp_hwmgr *hwmgr,
					    const struct si_power_state *ps)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (data->dyn_powertune_data.disable_uvd_powertune &&
	    ps->vclk && ps->dclk)
		return true;

	return false;
}

static int si_enable_power_containment(struct pp_hwmgr *hwmgr,
				       const struct si_power_state *new_state,
				       bool enable)
{
	struct si_hwmgr *data = hwmgr->backend;
	int ret = 0;

	if (data->enable_power_containment) {
		if (enable) {
			if (!si_should_disable_uvd_powertune(hwmgr, new_state)) {
				if (si_send_msg(hwmgr, PPSMC_TDPClampingActive))
					ret = -EINVAL;
			}
		} else {
			if (si_send_msg(hwmgr, PPSMC_TDPClampingInactive))
				ret = -EINVAL;
		}
	}

	return ret;
}

static int si_program_cac_config_registers(struct pp_hwmgr *hwmgr,
					   const struct si_cac_config_reg *cac_config_regs)
{
	const struct si_cac_config_reg *config_regs = cac_config_regs;
	uint32_t data = 0, offset;

	if (!config_regs)
		return -EINVAL;

	while (config_regs->offset != 0xFFFFFFFF) {
		switch (config_regs->type) {
		case SI_CACCONFIG_CGIND:
			offset = SI_SMC_CG_IND_START + config_regs->offset;
			if (offset < SI_SMC_CG_IND_END)
				data = si_rreg_smc(hwmgr, offset);
			break;
		default:
			data = si_rreg(hwmgr, config_regs->offset);
			break;
		}

		data &= ~config_regs->mask;
		data |= ((config_regs->value << config_regs->shift) & config_regs->mask);

		switch (config_regs->type) {
		case SI_CACCONFIG_CGIND:
			offset = SI_SMC_CG_IND_START + config_regs->offset;
			if (offset < SI_SMC_CG_IND_END)
				si_wreg_smc(hwmgr, offset, data);
			break;
		default:
			si_wreg(hwmgr, config_regs->offset, data);
			break;
		}
		config_regs++;
	}
	return 0;
}

static int si_initialize_hardware_cac_manager(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	int ret;

	if ((data->enable_cac == false) ||
	    (data->cac_configuration_required == false))
		return 0;

	ret = si_program_cac_config_registers(hwmgr, data->lcac_config);
	if (ret)
		return ret;
	ret = si_program_cac_config_registers(hwmgr, data->cac_override);
	if (ret)
		return ret;
	ret = si_program_cac_config_registers(hwmgr, data->cac_weights);
	if (ret)
		return ret;

	return 0;
}

static int si_enable_smc_cac(struct pp_hwmgr *hwmgr,
			     const struct si_power_state *new_state,
			     bool enable)
{
	struct si_hwmgr *data = hwmgr->backend;
	int ret = 0;

	if (data->enable_cac) {
		if (enable) {
			if (!si_should_disable_uvd_powertune(hwmgr, new_state)) {
				if (data->support_cac_long_term_average) {
					if (si_send_msg(hwmgr, PPSMC_CACLongTermAvgEnable))
						data->support_cac_long_term_average = false;
				}

				if (si_send_msg(hwmgr, PPSMC_MSG_EnableCac)) {
					ret = -EINVAL;
					data->cac_enabled = false;
				} else {
					data->cac_enabled = true;
				}

				if (data->enable_dte) {
					if (si_send_msg(hwmgr, PPSMC_MSG_EnableDTE))
						ret = -EINVAL;
				}
			}
		} else if (data->cac_enabled) {
			if (data->enable_dte)
				si_send_msg(hwmgr, PPSMC_MSG_DisableDTE);

			si_send_msg(hwmgr, PPSMC_MSG_DisableCac);

			data->cac_enabled = false;

			if (data->support_cac_long_term_average)
				si_send_msg(hwmgr, PPSMC_CACLongTermAvgDisable);
		}
	}
	return ret;
}

/* ---- PCIe link speed (legacy si_request_link_speed_change_*) ---- */

static enum si_pcie_gen si_get_maximum_link_speed(const struct si_power_state *state)
{
	int i;
	uint16_t pcie_speed, max_speed = 0;

	for (i = 0; i < state->performance_level_count; i++) {
		pcie_speed = state->performance_levels[i].pcie_gen;
		if (max_speed < pcie_speed)
			max_speed = pcie_speed;
	}
	return max_speed;
}

static void si_request_link_speed_change_before_state_change(struct pp_hwmgr *hwmgr,
							     const struct si_power_state *new_state,
							     const struct si_power_state *current_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	enum si_pcie_gen target_link_speed = si_get_maximum_link_speed(new_state);
	enum si_pcie_gen current_link_speed;

	if (data->force_pcie_gen == SI_PCIE_GEN_INVALID)
		current_link_speed = si_get_maximum_link_speed(current_state);
	else
		current_link_speed = data->force_pcie_gen;

	data->force_pcie_gen = SI_PCIE_GEN_INVALID;
	data->pspp_notify_required = false;
	if (target_link_speed > current_link_speed) {
		switch (target_link_speed) {
#if defined(CONFIG_ACPI)
		case SI_PCIE_GEN3:
			if (amdgpu_acpi_pcie_performance_request(adev, PCIE_PERF_REQ_GEN3, false) == 0)
				break;
			data->force_pcie_gen = SI_PCIE_GEN2;
			if (current_link_speed == SI_PCIE_GEN2)
				break;
			fallthrough;
		case SI_PCIE_GEN2:
			if (amdgpu_acpi_pcie_performance_request(adev, PCIE_PERF_REQ_GEN2, false) == 0)
				break;
			fallthrough;
#endif
		default:
			data->force_pcie_gen = si_get_current_pcie_speed(hwmgr);
			break;
		}
	} else {
		if (target_link_speed < current_link_speed)
			data->pspp_notify_required = true;
	}
}

static void si_notify_link_speed_change_after_state_change(struct pp_hwmgr *hwmgr,
							   const struct si_power_state *new_state,
							   const struct si_power_state *current_state)
{
	struct si_hwmgr *data = hwmgr->backend;
	enum si_pcie_gen target_link_speed = si_get_maximum_link_speed(new_state);
	uint8_t request;

	if (data->pspp_notify_required) {
		if (target_link_speed == SI_PCIE_GEN3)
			request = PCIE_PERF_REQ_GEN3;
		else if (target_link_speed == SI_PCIE_GEN2)
			request = PCIE_PERF_REQ_GEN2;
		else
			request = PCIE_PERF_REQ_GEN1;

		if ((request == PCIE_PERF_REQ_GEN1) &&
		    (si_get_current_pcie_speed(hwmgr) > 0))
			return;

#if defined(CONFIG_ACPI)
		amdgpu_acpi_pcie_performance_request(hwmgr->adev, request, false);
#endif
	}
}

/* legacy si_set_pcie_lane_width_in_smc(): the lane width comes from
 * the pp_power_state caps (ATOM_PPLIB_PCIE_LINK_WIDTH), so compare the
 * hwmgr request/current states rather than the hardware states.
 */
static void si_set_pcie_lane_width_in_smc(struct pp_hwmgr *hwmgr)
{
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t lane_width;
	uint32_t new_lane_width, current_lane_width;

	if (!hwmgr->request_ps || !hwmgr->current_ps)
		return;

	new_lane_width = hwmgr->request_ps->pcie.lanes;
	current_lane_width = hwmgr->current_ps->pcie.lanes;

	if (new_lane_width != current_lane_width) {
		amdgpu_set_pcie_lanes(adev, new_lane_width);
		lane_width = amdgpu_get_pcie_lanes(adev);
		si_smc_write_soft_register(hwmgr, SI_SMC_SOFT_REGISTER_non_ulv_pcie_link_width,
					   lane_width);
	}
}

/* ---- UVD / VCE clocks around a state switch ---- */

static void si_set_uvd_clock_before_set_eng_clock(struct pp_hwmgr *hwmgr,
						  const struct si_power_state *new_state,
						  const struct si_power_state *current_state)
{
	if ((new_state->vclk == current_state->vclk) &&
	    (new_state->dclk == current_state->dclk))
		return;

	if (new_state->performance_levels[new_state->performance_level_count - 1].sclk >=
	    current_state->performance_levels[current_state->performance_level_count - 1].sclk)
		return;

	amdgpu_asic_set_uvd_clocks((struct amdgpu_device *)hwmgr->adev,
				   new_state->vclk, new_state->dclk);
}

static void si_set_uvd_clock_after_set_eng_clock(struct pp_hwmgr *hwmgr,
						 const struct si_power_state *new_state,
						 const struct si_power_state *current_state)
{
	if ((new_state->vclk == current_state->vclk) &&
	    (new_state->dclk == current_state->dclk))
		return;

	if (new_state->performance_levels[new_state->performance_level_count - 1].sclk <
	    current_state->performance_levels[current_state->performance_level_count - 1].sclk)
		return;

	amdgpu_asic_set_uvd_clocks((struct amdgpu_device *)hwmgr->adev,
				   new_state->vclk, new_state->dclk);
}

/* legacy si_set_vce_clock(): evclk/ecclk come from the VCE state
 * picked in si_apply_state_adjust_rules().
 */
static void si_set_vce_clock(struct pp_hwmgr *hwmgr,
			     const struct si_power_state *new_rps,
			     const struct si_power_state *old_rps)
{
	struct amdgpu_device *adev = hwmgr->adev;

	if ((old_rps->evclk != new_rps->evclk) ||
	    (old_rps->ecclk != new_rps->ecclk)) {
		/* Turn the clocks on when encoding, off otherwise */
		dev_dbg(adev->dev, "set VCE clocks: %u, %u\n",
			new_rps->evclk, new_rps->ecclk);

		if (new_rps->evclk || new_rps->ecclk) {
			amdgpu_asic_set_vce_clocks(adev, new_rps->evclk,
						   new_rps->ecclk);
			amdgpu_device_ip_set_clockgating_state(
				adev, AMD_IP_BLOCK_TYPE_VCE, AMD_CG_STATE_UNGATE);
			amdgpu_device_ip_set_powergating_state(
				adev, AMD_IP_BLOCK_TYPE_VCE, AMD_PG_STATE_UNGATE);
		} else {
			amdgpu_device_ip_set_powergating_state(
				adev, AMD_IP_BLOCK_TYPE_VCE, AMD_PG_STATE_GATE);
			amdgpu_device_ip_set_clockgating_state(
				adev, AMD_IP_BLOCK_TYPE_VCE, AMD_CG_STATE_GATE);
			amdgpu_asic_set_vce_clocks(adev, 0, 0);
		}
	}
}

/* ---- ULV ---- */

static bool si_is_state_ulv_compatible(struct pp_hwmgr *hwmgr,
				       const struct si_power_state *state)
{
	const struct si_hwmgr *data = hwmgr->backend;
	const struct si_ulv_param *ulv = &data->ulv;
	const struct phm_clock_voltage_dependency_table *dep =
		hwmgr->dyn_state.vddc_dependency_on_display_clock;
	uint32_t i;

	if (state->performance_levels[0].mclk != ulv->pl.mclk)
		return false;

	/* XXX validate against display requirements! */

	if (dep) {
		for (i = 0; i < dep->count; i++) {
			if (hwmgr->display_config->display_clk <= dep->entries[i].clk) {
				if (ulv->pl.vddc < dep->entries[i].v)
					return false;
			}
		}
	}

	if ((state->vclk != 0) || (state->dclk != 0))
		return false;

	return true;
}

static int si_set_power_state_conditionally_enable_ulv(struct pp_hwmgr *hwmgr,
						       const struct si_power_state *new_state)
{
	const struct si_hwmgr *data = hwmgr->backend;
	const struct si_ulv_param *ulv = &data->ulv;

	if (ulv->supported) {
		if (si_is_state_ulv_compatible(hwmgr, new_state))
			return si_send_msg(hwmgr, PPSMC_MSG_EnableULV) ? -EINVAL : 0;
	}
	return 0;
}

/* ---- asic setup / DPM enable / disable (legacy si_dpm_setup_asic,
 *      si_dpm_enable, si_dpm_disable) ---- */

static int si_setup_asic_task(struct pp_hwmgr *hwmgr)
{
	si_read_clock_registers(hwmgr);
	si_enable_acpi_power_management(hwmgr);
	return 0;
}

/* ---- debugfs: SMC SRAM tables as the firmware sees them ----
 *
 * /sys/kernel/debug/amdgpu_si/smc_tables dumps every table region this
 * backend uploads (state table with initial/ACPI/ULV/driver states and
 * the DPM2 parameters, soft registers, MC/ARB tables, CAC/DTE config,
 * SPLL table, PAPM, fan table) as dwords, read back from SMC SRAM. The
 * same file exists in the legacy si_dpm build, so the two can be diffed.
 */
static struct dentry *si_smc_debugfs_dir;

static int si_smc_tables_show(struct seq_file *m, void *unused)
{
	struct pp_hwmgr *hwmgr = m->private;
	struct si_smumgr *smu_data = hwmgr->smu_backend;
	const struct {
		const char *name;
		uint32_t start;
		uint32_t size;
	} regions[] = {
		{ "soft_regs",   smu_data->soft_regs_start,     SI_SMC_SOFT_REGISTER_svi_rework_gpio_id_svc + 4 },
		{ "state_table", smu_data->state_table_start,   sizeof(SISLANDS_SMC_STATETABLE) },
		{ "mc_reg_table", smu_data->mc_reg_table_start, sizeof(SMC_SIslands_MCRegisters) },
		{ "arb_table",   smu_data->arb_table_start,     sizeof(SMC_SIslands_MCArbDramTimingRegisters) },
		{ "cac_table",   smu_data->cac_table_start,     sizeof(PP_SIslands_CacConfig) },
		{ "dte_table",   smu_data->dte_table_start,     sizeof(Smc_SIslands_DTE_Configuration) },
		{ "spll_table",  smu_data->spll_table_start,    sizeof(SMC_SISLANDS_SPLL_DIV_TABLE) },
		{ "papm_cfg",    smu_data->papm_cfg_table_start, sizeof(PP_SIslands_PAPMParameters) },
		{ "fan_table",   smu_data->fan_table_start,     sizeof(PP_SIslands_FanTable) },
	};
	unsigned int r, off;
	uint32_t val;

	seq_printf(m, "backend: powerplay si_hwmgr\n");
	for (r = 0; r < ARRAY_SIZE(regions); r++) {
		seq_printf(m, "\n[%s] start 0x%x size %u\n", regions[r].name,
			   regions[r].start, regions[r].size);
		if (!regions[r].start)
			continue;
		for (off = 0; off < regions[r].size; off += 4) {
			if ((off & 31) == 0)
				seq_printf(m, "%s+%04x:", regions[r].name, off);
			if (si_smc_read_sram_dword(hwmgr, regions[r].start + off,
						   &val, smu_data->sram_end))
				seq_puts(m, " ????????");
			else
				seq_printf(m, " %08x", val);
			if ((off & 31) == 28 || off + 4 >= regions[r].size)
				seq_putc(m, '\n');
		}
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(si_smc_tables);

/* /sys/kernel/debug/amdgpu_si/live_state prints, on demand, the same power
 * management registers and SMC program counter samples that are captured
 * at a GPU hang (see si_dump_state()). Read it at idle, under load and
 * while stepping through the levels for a healthy baseline to compare a
 * hang snapshot against.
 */
static void si_capture_state(struct pp_hwmgr *hwmgr, struct si_state_snapshot *s);
static void si_print_snapshot(struct pp_hwmgr *hwmgr, struct drm_printer *p,
			      const struct si_state_snapshot *s, const char *tag);

static int si_live_state_show(struct seq_file *m, void *unused)
{
	struct pp_hwmgr *hwmgr = m->private;
	struct si_state_snapshot snap;
	struct drm_printer p = drm_seq_file_printer(m);

	si_capture_state(hwmgr, &snap);
	si_print_snapshot(hwmgr, &p, &snap, "live");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(si_live_state);

static void si_smc_debugfs_create(struct pp_hwmgr *hwmgr)
{
	if (si_smc_debugfs_dir)
		return;
	si_smc_debugfs_dir = debugfs_create_dir("amdgpu_si", NULL);
	debugfs_create_file("smc_tables", 0444, si_smc_debugfs_dir, hwmgr,
			    &si_smc_tables_fops);
	debugfs_create_file("live_state", 0444, si_smc_debugfs_dir, hwmgr,
			    &si_live_state_fops);
}

static void si_smc_debugfs_remove(void)
{
	debugfs_remove_recursive(si_smc_debugfs_dir);
	si_smc_debugfs_dir = NULL;
}

static int si_enable_dpm_tasks(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct si_power_state *boot_state;
	int ret;

	if (!hwmgr->boot_ps)
		return -EINVAL;
	boot_state = cast_const_phw_si_power_state(&hwmgr->boot_ps->hardware);
	if (!boot_state)
		return -EINVAL;

	if (si_smc_is_running(hwmgr))
		return -EINVAL;
	if (data->voltage_control || data->voltage_control_svi2)
		si_enable_voltage_control(hwmgr, true);
	if (data->mvdd_control)
		data->mvdd_split_frequency = 30000; /* legacy si_get_mvdd_configuration */
	if (data->voltage_control || data->voltage_control_svi2) {
		ret = si_construct_voltage_tables(hwmgr);
		if (ret) {
			pr_err("si_construct_voltage_tables failed\n");
			return ret;
		}
	}
	if (data->dynamic_ac_timing) {
		ret = smum_initialize_mc_reg_table(hwmgr);
		if (ret)
			data->dynamic_ac_timing = false;
	}
	if (data->dynamic_ss)
		si_enable_spread_spectrum(hwmgr, true);
	if (data->thermal_protection)
		si_enable_thermal_protection(hwmgr, true);
	si_setup_bsp(hwmgr);
	si_program_git(hwmgr);
	si_program_tp(hwmgr);
	si_program_tpp(hwmgr);
	si_program_sstp(hwmgr);
	si_enable_display_gap(hwmgr);
	si_program_vc(hwmgr);
	ret = si_smc_upload_firmware(hwmgr);
	if (ret) {
		pr_err("si_smc_upload_firmware failed\n");
		return ret;
	}
	ret = smum_process_firmware_header(hwmgr);
	if (ret) {
		pr_err("si_process_firmware_header failed\n");
		return ret;
	}
	ret = si_smc_initial_switch_from_arb_f0_to_f1(hwmgr);
	if (ret) {
		pr_err("si_initial_switch_from_arb_f0_to_f1 failed\n");
		return ret;
	}
	ret = smum_init_smc_table(hwmgr);
	if (ret) {
		pr_err("si_init_smc_table failed\n");
		return ret;
	}
	ret = si_smc_init_spll_table(hwmgr);
	if (ret) {
		pr_err("si_init_smc_spll_table failed\n");
		return ret;
	}
	ret = si_smc_init_arb_table_index(hwmgr);
	if (ret) {
		pr_err("si_init_arb_table_index failed\n");
		return ret;
	}
	if (data->dynamic_ac_timing) {
		ret = si_smc_populate_mc_reg_table(hwmgr, boot_state);
		if (ret) {
			pr_err("si_populate_mc_reg_table failed\n");
			return ret;
		}
	}
	ret = si_smc_initialize_smc_cac_tables(hwmgr);
	if (ret) {
		pr_err("si_initialize_smc_cac_tables failed\n");
		return ret;
	}
	ret = si_initialize_hardware_cac_manager(hwmgr);
	if (ret) {
		pr_err("si_initialize_hardware_cac_manager failed\n");
		return ret;
	}
	ret = si_smc_initialize_smc_dte_tables(hwmgr);
	if (ret) {
		pr_err("si_initialize_smc_dte_tables failed\n");
		return ret;
	}
	ret = si_smc_populate_smc_tdp_limits(hwmgr);
	if (ret) {
		pr_err("si_populate_smc_tdp_limits failed\n");
		return ret;
	}
	ret = si_smc_populate_smc_tdp_limits_2(hwmgr);
	if (ret) {
		pr_err("si_populate_smc_tdp_limits_2 failed\n");
		return ret;
	}
	si_smc_program_response_times(hwmgr);
	si_program_ds_registers(hwmgr);
	si_smc_dpm_start(hwmgr);
	ret = si_notify_smc_display_change(hwmgr, false);
	if (ret) {
		pr_err("si_notify_smc_display_change failed\n");
		return ret;
	}
	si_enable_sclk_control(hwmgr, true);
	si_start_dpm(hwmgr);

	si_enable_auto_throttle_source(hwmgr, SI_DPM_AUTO_THROTTLE_SRC_THERMAL, true);

	/* legacy si_thermal_start_thermal_controller() runs from
	 * phm_start_thermal_controller() right after this returns;
	 * legacy ni_update_current_ps(boot_ps) is hwmgr->current_ps,
	 * already the boot state from psm_init_power_state_table().
	 */

	si_smc_debugfs_create(hwmgr);

	dev_info(((struct amdgpu_device *)hwmgr->adev)->dev,
		 "powerplay: SI DPM enabled (%u states, %s)\n", hwmgr->num_ps,
		 data->enable_power_containment ? "power containment on" : "power containment off");

	return 0;
}

/* thermal controller stop, shared by disable and the hwmgr hook */
static int si_thermal_stop_thermal_controller(struct pp_hwmgr *hwmgr);

static int si_disable_dpm_tasks(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct si_power_state *boot_state;

	if (!hwmgr->boot_ps)
		return -EINVAL;
	boot_state = cast_const_phw_si_power_state(&hwmgr->boot_ps->hardware);
	if (!boot_state)
		return -EINVAL;

	if (!si_smc_is_running(hwmgr))
		return 0;
	si_thermal_stop_thermal_controller(hwmgr);
	si_disable_ulv(hwmgr);
	si_clear_vc(hwmgr);
	if (data->thermal_protection)
		si_enable_thermal_protection(hwmgr, false);
	si_enable_power_containment(hwmgr, boot_state, false);
	si_enable_smc_cac(hwmgr, boot_state, false);
	si_enable_spread_spectrum(hwmgr, false);
	si_enable_auto_throttle_source(hwmgr, SI_DPM_AUTO_THROTTLE_SRC_THERMAL, false);
	si_stop_dpm(hwmgr);
	si_reset_to_default(hwmgr);
	smum_stop_smc(hwmgr);
	si_smc_force_switch_to_arb_f0(hwmgr);

	/* legacy ni_update_current_ps(boot_ps): the framework switches
	 * back to the boot state (psm_set_boot_states) before calling us.
	 */
	memcpy(hwmgr->current_ps, hwmgr->boot_ps, hwmgr->ps_size);

	return 0;
}

/* ---- power state switch (legacy si_dpm_set_power_state) ---- */

static int si_power_control_set_level(struct pp_hwmgr *hwmgr)
{
	int ret;

	ret = si_restrict_performance_levels_before_switch(hwmgr);
	if (ret)
		return ret;
	ret = si_smc_halt(hwmgr);
	if (ret)
		return ret;
	ret = si_smc_populate_smc_tdp_limits(hwmgr);
	if (ret)
		return ret;
	ret = si_smc_populate_smc_tdp_limits_2(hwmgr);
	if (ret)
		return ret;
	ret = si_smc_resume(hwmgr);
	if (ret)
		return ret;
	return si_set_sw_state(hwmgr);
}

/* Display parameters last programmed into the SMC by
 * si_smc_upload_smc_data(); compared by
 * si_check_smc_update_required_for_display_configuration().
 */
static void si_snapshot_display_config(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct amd_pp_display_configuration *cfg = hwmgr->display_config;

	data->display_num_display = cfg->num_display;
	data->display_crtc_index = cfg->crtc_index;
	data->display_line_time_in_us = cfg->line_time_in_us;
}

static int si_set_power_state_tasks(struct pp_hwmgr *hwmgr, const void *input)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct phm_set_power_state_input *states =
		(const struct phm_set_power_state_input *)input;
	const struct si_power_state *new_state =
		cast_const_phw_si_power_state(states->pnew_state);
	const struct si_power_state *old_state =
		cast_const_phw_si_power_state(states->pcurrent_state);
	int ret;

	if (!new_state || !old_state)
		return -EINVAL;
	if (new_state->performance_level_count == 0 ||
	    old_state->performance_level_count == 0)
		return -EINVAL;

	pr_debug("si state switch: top sclk %u->%u mclk %u->%u, %u->%u levels\n",
		 old_state->performance_levels[old_state->performance_level_count - 1].sclk / 100,
		 new_state->performance_levels[new_state->performance_level_count - 1].sclk / 100,
		 old_state->performance_levels[old_state->performance_level_count - 1].mclk / 100,
		 new_state->performance_levels[new_state->performance_level_count - 1].mclk / 100,
		 old_state->performance_level_count, new_state->performance_level_count);

	ret = si_disable_ulv(hwmgr);
	if (ret) {
		pr_err("si_disable_ulv failed\n");
		return ret;
	}
	ret = si_restrict_performance_levels_before_switch(hwmgr);
	if (ret) {
		pr_err("si_restrict_performance_levels_before_switch failed\n");
		return ret;
	}
	if (data->pcie_performance_request)
		si_request_link_speed_change_before_state_change(hwmgr, new_state, old_state);
	si_set_uvd_clock_before_set_eng_clock(hwmgr, new_state, old_state);
	ret = si_enable_power_containment(hwmgr, new_state, false);
	if (ret) {
		pr_err("si_enable_power_containment failed\n");
		return ret;
	}
	ret = si_enable_smc_cac(hwmgr, new_state, false);
	if (ret) {
		pr_err("si_enable_smc_cac failed\n");
		return ret;
	}
	ret = si_smc_halt(hwmgr);
	if (ret) {
		pr_err("si_halt_smc failed\n");
		return ret;
	}
	ret = si_smc_upload_sw_state(hwmgr, new_state);
	if (ret) {
		pr_err("si_upload_sw_state failed\n");
		return ret;
	}
	ret = si_smc_upload_smc_data(hwmgr);
	if (ret) {
		pr_err("si_upload_smc_data failed\n");
		return ret;
	}
	si_snapshot_display_config(hwmgr);
	ret = si_smc_upload_ulv_state(hwmgr);
	if (ret) {
		pr_err("si_upload_ulv_state failed\n");
		return ret;
	}
	if (data->dynamic_ac_timing) {
		ret = si_smc_upload_mc_reg_table(hwmgr, new_state);
		if (ret) {
			pr_err("si_upload_mc_reg_table failed\n");
			return ret;
		}
	}
	ret = si_smc_program_memory_timing_parameters(hwmgr, new_state);
	if (ret) {
		pr_err("si_program_memory_timing_parameters failed\n");
		return ret;
	}
	si_set_pcie_lane_width_in_smc(hwmgr);

	ret = si_smc_resume(hwmgr);
	if (ret) {
		pr_err("si_resume_smc failed\n");
		return ret;
	}
	ret = si_set_sw_state(hwmgr);
	if (ret) {
		pr_err("si_set_sw_state failed\n");
		return ret;
	}
	si_set_uvd_clock_after_set_eng_clock(hwmgr, new_state, old_state);
	si_set_vce_clock(hwmgr, new_state, old_state);
	if (data->pcie_performance_request)
		si_notify_link_speed_change_after_state_change(hwmgr, new_state, old_state);
	ret = si_set_power_state_conditionally_enable_ulv(hwmgr, new_state);
	if (ret) {
		pr_err("si_set_power_state_conditionally_enable_ulv failed\n");
		return ret;
	}
	ret = si_enable_smc_cac(hwmgr, new_state, true);
	if (ret) {
		pr_err("si_enable_smc_cac failed\n");
		return ret;
	}
	ret = si_enable_power_containment(hwmgr, new_state, true);
	if (ret) {
		pr_err("si_enable_power_containment failed\n");
		return ret;
	}

	ret = si_power_control_set_level(hwmgr);
	if (ret) {
		pr_err("si_power_control_set_level failed\n");
		return ret;
	}

	return 0;
}

static int si_check_states_equal(struct pp_hwmgr *hwmgr,
				 const struct pp_hw_power_state *pstate1,
				 const struct pp_hw_power_state *pstate2,
				 bool *equal)
{
	const struct si_power_state *ps1 = cast_const_phw_si_power_state(pstate1);
	const struct si_power_state *ps2 = cast_const_phw_si_power_state(pstate2);
	int i;

	if (ps1 == NULL || ps2 == NULL || equal == NULL)
		return -EINVAL;

	if (ps1->performance_level_count != ps2->performance_level_count) {
		*equal = false;
		return 0;
	}

	for (i = 0; i < ps1->performance_level_count; i++) {
		const struct si_performance_level *l1 = &ps1->performance_levels[i];
		const struct si_performance_level *l2 = &ps2->performance_levels[i];

		if (!((l1->mclk == l2->mclk) &&
		      (l1->sclk == l2->sclk) &&
		      (l1->pcie_gen == l2->pcie_gen) &&
		      (l1->vddc == l2->vddc) &&
		      (l1->vddci == l2->vddci))) {
			*equal = false;
			return 0;
		}
	}

	/* If all performance levels are the same try to use the UVD clocks to break the tie.*/
	*equal = ((ps1->vclk == ps2->vclk) && (ps1->dclk == ps2->dclk));
	*equal &= ((ps1->evclk == ps2->evclk) && (ps1->ecclk == ps2->ecclk));

	return 0;
}

/* si_upload_smc_data() programs the crtc index / MCLK change window
 * into the SMC as part of the state switch, so a display change needs
 * a switch even when the adjusted state comes out identical.
 */
static bool si_check_smc_update_required_for_display_configuration(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct amd_pp_display_configuration *cfg = hwmgr->display_config;

	return data->display_num_display != cfg->num_display ||
	       data->display_crtc_index != cfg->crtc_index ||
	       data->display_line_time_in_us != cfg->line_time_in_us;
}

/* ---- thermal / fan (legacy si_thermal_* / si_fan_ctrl_*) ---- */

static int si_thermal_enable_alert(struct pp_hwmgr *hwmgr, bool enable)
{
	uint32_t thermal_int = si_rreg(hwmgr, mmCG_THERMAL_INT);

	if (enable) {
		thermal_int &= ~(CG_THERMAL_INT__THERM_INT_MASK_HIGH_MASK |
				 CG_THERMAL_INT__THERM_INT_MASK_LOW_MASK);
		si_wreg(hwmgr, mmCG_THERMAL_INT, thermal_int);
		if (si_send_msg(hwmgr, PPSMC_MSG_EnableThermalInterrupt)) {
			pr_debug("Could not enable thermal interrupts.\n");
			return -EINVAL;
		}
	} else {
		thermal_int |= CG_THERMAL_INT__THERM_INT_MASK_HIGH_MASK |
			       CG_THERMAL_INT__THERM_INT_MASK_LOW_MASK;
		si_wreg(hwmgr, mmCG_THERMAL_INT, thermal_int);
	}

	return 0;
}

static int si_thermal_set_temperature_range(struct pp_hwmgr *hwmgr,
					    int min_temp, int max_temp)
{
	int low_temp = 0 * 1000;
	int high_temp = 255 * 1000;

	if (low_temp < min_temp)
		low_temp = min_temp;
	if (high_temp > max_temp)
		high_temp = max_temp;
	if (high_temp < low_temp) {
		pr_err("invalid thermal range: %d - %d\n", low_temp, high_temp);
		return -EINVAL;
	}

	si_wreg_p(hwmgr, mmCG_THERMAL_INT,
		  (high_temp / 1000) << CG_THERMAL_INT__DIG_THERM_INTH__SHIFT,
		  ~CG_THERMAL_INT__DIG_THERM_INTH_MASK);
	si_wreg_p(hwmgr, mmCG_THERMAL_INT,
		  (low_temp / 1000) << CG_THERMAL_INT__DIG_THERM_INTL__SHIFT,
		  ~CG_THERMAL_INT__DIG_THERM_INTL_MASK);
	si_wreg_p(hwmgr, mmCG_THERMAL_CTRL,
		  (high_temp / 1000) << CG_THERMAL_CTRL__DIG_THERM_DPM__SHIFT,
		  ~CG_THERMAL_CTRL__DIG_THERM_DPM_MASK);

	return 0;
}

static void si_fan_ctrl_set_static_mode(struct pp_hwmgr *hwmgr, uint32_t mode)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t tmp;

	if (data->fan_ctrl_is_in_default_mode) {
		tmp = (si_rreg(hwmgr, mmCG_FDO_CTRL2) & CG_FDO_CTRL2__FDO_PWM_MODE_MASK) >>
			CG_FDO_CTRL2__FDO_PWM_MODE__SHIFT;
		data->fan_ctrl_default_mode = tmp;
		tmp = (si_rreg(hwmgr, mmCG_FDO_CTRL2) & CG_FDO_CTRL2__TMIN_MASK) >>
			CG_FDO_CTRL2__TMIN__SHIFT;
		data->t_min = tmp;
		data->fan_ctrl_is_in_default_mode = false;
	}

	tmp = si_rreg(hwmgr, mmCG_FDO_CTRL2) & ~CG_FDO_CTRL2__TMIN_MASK;
	tmp |= 0 << CG_FDO_CTRL2__TMIN__SHIFT;
	si_wreg(hwmgr, mmCG_FDO_CTRL2, tmp);

	tmp = si_rreg(hwmgr, mmCG_FDO_CTRL2) & ~CG_FDO_CTRL2__FDO_PWM_MODE_MASK;
	tmp |= mode << CG_FDO_CTRL2__FDO_PWM_MODE__SHIFT;
	si_wreg(hwmgr, mmCG_FDO_CTRL2, tmp);
}

static int si_fan_ctrl_start_smc_fan_control(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (si_send_msg(hwmgr, PPSMC_StartFanControl))
		return -EINVAL;

	data->fan_is_controlled_by_smc = true;
	return 0;
}

static int si_fan_ctrl_stop_smc_fan_control(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (si_send_msg(hwmgr, PPSMC_StopFanControl))
		return -EINVAL;

	data->fan_is_controlled_by_smc = false;
	return 0;
}

static bool si_ucode_fan_control(struct pp_hwmgr *hwmgr)
{
	return phm_cap_enabled(hwmgr->platform_descriptor.platformCaps,
			       PHM_PlatformCaps_MicrocodeFanControl);
}

static int si_fan_ctrl_get_fan_speed_pwm(struct pp_hwmgr *hwmgr, uint32_t *speed)
{
	uint32_t duty, duty100;
	uint64_t tmp64;

	if (!speed)
		return -EINVAL;

	if (hwmgr->thermal_controller.fanInfo.bNoFan)
		return -ENOENT;

	duty100 = (si_rreg(hwmgr, mmCG_FDO_CTRL1) & CG_FDO_CTRL1__FMAX_DUTY100_MASK) >>
		CG_FDO_CTRL1__FMAX_DUTY100__SHIFT;
	duty = (si_rreg(hwmgr, mmCG_THERMAL_STATUS) & CG_THERMAL_STATUS__FDO_PWM_DUTY_MASK) >>
		CG_THERMAL_STATUS__FDO_PWM_DUTY__SHIFT;

	if (duty100 == 0)
		return -EINVAL;

	tmp64 = (uint64_t)duty * 255;
	do_div(tmp64, duty100);
	*speed = min_t(uint32_t, tmp64, 255);

	return 0;
}

static int si_fan_ctrl_set_fan_speed_pwm(struct pp_hwmgr *hwmgr, uint32_t speed)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t tmp;
	uint32_t duty, duty100;
	uint64_t tmp64;

	if (hwmgr->thermal_controller.fanInfo.bNoFan)
		return -ENOENT;

	if (data->fan_is_controlled_by_smc)
		return -EINVAL;

	if (speed > 255)
		return -EINVAL;

	duty100 = (si_rreg(hwmgr, mmCG_FDO_CTRL1) & CG_FDO_CTRL1__FMAX_DUTY100_MASK) >>
		CG_FDO_CTRL1__FMAX_DUTY100__SHIFT;

	if (duty100 == 0)
		return -EINVAL;

	tmp64 = (uint64_t)speed * duty100;
	do_div(tmp64, 255);
	duty = (uint32_t)tmp64;

	tmp = si_rreg(hwmgr, mmCG_FDO_CTRL0) & ~CG_FDO_CTRL0__FDO_STATIC_DUTY_MASK;
	tmp |= duty << CG_FDO_CTRL0__FDO_STATIC_DUTY__SHIFT;
	si_wreg(hwmgr, mmCG_FDO_CTRL0, tmp);

	return 0;
}

static void si_fan_ctrl_set_default_mode(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t tmp;

	if (!data->fan_ctrl_is_in_default_mode) {
		tmp = si_rreg(hwmgr, mmCG_FDO_CTRL2) & ~CG_FDO_CTRL2__FDO_PWM_MODE_MASK;
		tmp |= data->fan_ctrl_default_mode << CG_FDO_CTRL2__FDO_PWM_MODE__SHIFT;
		si_wreg(hwmgr, mmCG_FDO_CTRL2, tmp);

		tmp = si_rreg(hwmgr, mmCG_FDO_CTRL2) & ~CG_FDO_CTRL2__TMIN_MASK;
		tmp |= data->t_min << CG_FDO_CTRL2__TMIN__SHIFT;
		si_wreg(hwmgr, mmCG_FDO_CTRL2, tmp);
		data->fan_ctrl_is_in_default_mode = true;
	}
}

static void si_thermal_start_smc_fan_control(struct pp_hwmgr *hwmgr)
{
	if (si_ucode_fan_control(hwmgr)) {
		si_fan_ctrl_start_smc_fan_control(hwmgr);
		si_fan_ctrl_set_static_mode(hwmgr, SI_FDO_PWM_MODE_STATIC);
	}
}

static void si_set_fan_control_mode(struct pp_hwmgr *hwmgr, uint32_t mode)
{
	if (mode == U32_MAX)
		return;

	if (mode) {
		/* stop auto-manage */
		if (si_ucode_fan_control(hwmgr))
			si_fan_ctrl_stop_smc_fan_control(hwmgr);
		si_fan_ctrl_set_static_mode(hwmgr, mode);
	} else {
		/* restart auto-manage */
		if (si_ucode_fan_control(hwmgr))
			si_thermal_start_smc_fan_control(hwmgr);
		else
			si_fan_ctrl_set_default_mode(hwmgr);
	}
}

static int si_fan_ctrl_reset_fan_speed_to_default(struct pp_hwmgr *hwmgr)
{
	if (hwmgr->thermal_controller.fanInfo.bNoFan)
		return 0;

	/* back to SMC fan control, or the VBIOS default PWM mode without it */
	si_set_fan_control_mode(hwmgr, 0);
	return 0;
}

static uint32_t si_get_fan_control_mode(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t tmp;

	if (data->fan_is_controlled_by_smc)
		return 0;

	tmp = si_rreg(hwmgr, mmCG_FDO_CTRL2) & CG_FDO_CTRL2__FDO_PWM_MODE_MASK;
	return tmp >> CG_FDO_CTRL2__FDO_PWM_MODE__SHIFT;
}

static int si_fan_ctrl_get_fan_speed_rpm(struct pp_hwmgr *hwmgr, uint32_t *speed)
{
	uint32_t tach_period;
	uint32_t xclk = amdgpu_asic_get_xclk((struct amdgpu_device *)hwmgr->adev);

	if (!speed)
		return -EINVAL;

	/* Tachometer readout is only validated on Oland; keep the
	 * historical -EOPNOTSUPP behavior on other SI ASICs.
	 */
	if (hwmgr->chip_id != CHIP_OLAND)
		return -EOPNOTSUPP;

	if (hwmgr->thermal_controller.fanInfo.bNoFan)
		return -ENOENT;

	if (hwmgr->thermal_controller.fanInfo.ucTachometerPulsesPerRevolution == 0)
		return -ENOENT;

	tach_period = (si_rreg(hwmgr, mmCG_TACH_STATUS) & CG_TACH_STATUS__TACH_PERIOD_MASK) >>
		CG_TACH_STATUS__TACH_PERIOD__SHIFT;
	if (tach_period == 0)
		return -ENOENT;

	*speed = 60 * xclk * 10000 / tach_period;

	return 0;
}

static int si_fan_ctrl_set_fan_speed_rpm(struct pp_hwmgr *hwmgr, uint32_t speed)
{
	uint32_t tach_period, tmp;
	uint32_t xclk = amdgpu_asic_get_xclk((struct amdgpu_device *)hwmgr->adev);

	if (hwmgr->chip_id != CHIP_OLAND)
		return -EOPNOTSUPP;

	if (hwmgr->thermal_controller.fanInfo.bNoFan)
		return -ENOENT;

	if (hwmgr->thermal_controller.fanInfo.ucTachometerPulsesPerRevolution == 0)
		return -ENOENT;

	if ((speed < hwmgr->thermal_controller.fanInfo.ulMinRPM) ||
	    (speed > hwmgr->thermal_controller.fanInfo.ulMaxRPM))
		return -EINVAL;

	if (si_ucode_fan_control(hwmgr))
		si_fan_ctrl_stop_smc_fan_control(hwmgr);

	tach_period = 60 * xclk * 10000 / (8 * speed);
	tmp = si_rreg(hwmgr, mmCG_TACH_CTRL) & ~CG_TACH_CTRL__TARGET_PERIOD_MASK;
	tmp |= tach_period << CG_TACH_CTRL__TARGET_PERIOD__SHIFT;
	si_wreg(hwmgr, mmCG_TACH_CTRL, tmp);

	si_fan_ctrl_set_static_mode(hwmgr, SI_FDO_PWM_MODE_STATIC_RPM);

	return 0;
}

static int si_fan_ctrl_get_fan_speed_info(struct pp_hwmgr *hwmgr,
					  struct phm_fan_speed_info *fan_speed_info)
{
	if (hwmgr->thermal_controller.fanInfo.bNoFan)
		return -ENOENT;

	fan_speed_info->supports_percent_read = true;
	fan_speed_info->supports_percent_write = true;
	fan_speed_info->min_percent = 0;
	fan_speed_info->max_percent = 100;

	if (hwmgr->chip_id == CHIP_OLAND &&
	    hwmgr->thermal_controller.fanInfo.ucTachometerPulsesPerRevolution) {
		fan_speed_info->supports_rpm_read = true;
		fan_speed_info->supports_rpm_write = true;
		fan_speed_info->min_rpm = hwmgr->thermal_controller.fanInfo.ulMinRPM;
		fan_speed_info->max_rpm = hwmgr->thermal_controller.fanInfo.ulMaxRPM;
	} else {
		fan_speed_info->min_rpm = 0;
		fan_speed_info->max_rpm = 0;
	}

	return 0;
}

static void si_thermal_initialize(struct pp_hwmgr *hwmgr)
{
	uint32_t tmp;

	if (hwmgr->thermal_controller.fanInfo.ucTachometerPulsesPerRevolution) {
		tmp = si_rreg(hwmgr, mmCG_TACH_CTRL) & ~CG_TACH_CTRL__EDGE_PER_REV_MASK;
		tmp |= (hwmgr->thermal_controller.fanInfo.ucTachometerPulsesPerRevolution - 1) <<
			CG_TACH_CTRL__EDGE_PER_REV__SHIFT;
		si_wreg(hwmgr, mmCG_TACH_CTRL, tmp);
	}

	tmp = si_rreg(hwmgr, mmCG_FDO_CTRL2) & ~CG_FDO_CTRL2__TACH_PWM_RESP_RATE_MASK;
	tmp |= 0x28 << CG_FDO_CTRL2__TACH_PWM_RESP_RATE__SHIFT;
	si_wreg(hwmgr, mmCG_FDO_CTRL2, tmp);
}

/* legacy si_thermal_start_thermal_controller() + late_init's
 * si_set_temperature_range(), driven from phm_start_thermal_controller.
 */
static int si_start_thermal_controller(struct pp_hwmgr *hwmgr,
				       struct PP_TemperatureRange *range)
{
	int ret;

	if (range == NULL)
		return -EINVAL;

	si_thermal_initialize(hwmgr);
	ret = si_thermal_set_temperature_range(hwmgr, range->min, range->max);
	if (ret)
		return ret;
	ret = si_thermal_enable_alert(hwmgr, true);
	if (ret)
		return ret;
	if (si_ucode_fan_control(hwmgr)) {
		ret = si_smc_halt(hwmgr);
		if (ret)
			return ret;
		ret = smum_thermal_setup_fan_table(hwmgr);
		if (ret)
			return ret;
		ret = si_smc_resume(hwmgr);
		if (ret)
			return ret;
		si_thermal_start_smc_fan_control(hwmgr);
	}

	return 0;
}

static int si_thermal_stop_thermal_controller(struct pp_hwmgr *hwmgr)
{
	if (!hwmgr->thermal_controller.fanInfo.bNoFan) {
		si_fan_ctrl_set_default_mode(hwmgr);
		si_fan_ctrl_stop_smc_fan_control(hwmgr);
	}
	return 0;
}

static int si_get_thermal_temperature_range(struct pp_hwmgr *hwmgr,
					    struct PP_TemperatureRange *thermal_data)
{
	/* Legacy R600_TEMP_RANGE_MIN (90 C) was the low trip point for the
	 * legacy thermal work item (drop back to the user state once cool).
	 * PowerPlay has no consumer for it: with DIG_THERM_INTL at 90 C the
	 * high->low interrupt fires at every DPM enable while the GPU sits at
	 * 50 C and phm_irq_process() logs a spurious "GPU under temperature
	 * range" emergency. Use 0 like smu7 so only the high trip is live.
	 */
	thermal_data->min = 0;
	thermal_data->max = SI_TEMP_RANGE_MAX;
	thermal_data->sw_ctf_threshold = SI_TEMP_RANGE_MAX;
	return 0;
}

/* ---- sensors / sysfs ---- */

static int si_dpm_get_temp(struct pp_hwmgr *hwmgr)
{
	uint32_t temp;
	int actual_temp = 0;

	temp = (si_rreg(hwmgr, mmCG_MULT_THERMAL_STATUS) & CG_MULT_THERMAL_STATUS__CTF_TEMP_MASK) >>
		CG_MULT_THERMAL_STATUS__CTF_TEMP__SHIFT;

	if (temp & 0x200)
		actual_temp = 255;
	else
		actual_temp = temp & 0x1ff;

	actual_temp = (actual_temp * 1000);

	return actual_temp;
}

static uint32_t si_get_current_profile_index(struct pp_hwmgr *hwmgr)
{
	return (si_rreg(hwmgr, mmTARGET_AND_CURRENT_PROFILE_INDEX) &
		TARGET_AND_CURRENT_PROFILE_INDEX__CURRENT_STATE_INDEX_MASK) >>
		TARGET_AND_CURRENT_PROFILE_INDEX__CURRENT_STATE_INDEX__SHIFT;
}

/* The SCLK the SPLL is producing right now, in 10 kHz: the inverse of
 * si_smc_calculate_sclk_params(). The SMC owns these registers while
 * DPM runs, so this also shows PowerTune pulling the clock below the
 * level's nominal value, which the level table cannot.
 */
static uint32_t si_get_current_sclk(struct pp_hwmgr *hwmgr)
{
	struct amdgpu_device *adev = hwmgr->adev;
	uint32_t cntl = si_rreg(hwmgr, mmCG_SPLL_FUNC_CNTL);
	uint32_t cntl3 = si_rreg(hwmgr, mmCG_SPLL_FUNC_CNTL_3);
	uint32_t ref_div = ((cntl & CG_SPLL_FUNC_CNTL__SPLL_REF_DIV_MASK) >>
			    CG_SPLL_FUNC_CNTL__SPLL_REF_DIV__SHIFT) + 1;
	uint32_t post_div = (cntl & CG_SPLL_FUNC_CNTL__SPLL_PDIV_A_MASK) >>
			    CG_SPLL_FUNC_CNTL__SPLL_PDIV_A__SHIFT;
	uint32_t fbdiv = (cntl3 & CG_SPLL_FUNC_CNTL_3__SPLL_FB_DIV_MASK) >>
			 CG_SPLL_FUNC_CNTL_3__SPLL_FB_DIV__SHIFT;
	uint32_t div = 16384u * ref_div * post_div;
	uint64_t tmp;

	if (!post_div || !fbdiv || !adev->clock.spll.reference_freq)
		return 0;

	/* round to nearest: the fbdiv the SMC programs was itself truncated
	 * when derived from the level's clock, so plain division lands one
	 * 10 kHz step under it (1000 MHz reads back as 999.99)
	 */
	tmp = (uint64_t)fbdiv * adev->clock.spll.reference_freq + div / 2;
	do_div(tmp, div);
	return (uint32_t)tmp;
}

static int si_read_sensor(struct pp_hwmgr *hwmgr, int idx,
			  void *value, int *size)
{
	const struct si_power_state *ps;
	uint32_t pl_index, sclk;

	/* size must be at least 4 bytes for all sensors */
	if (*size < 4)
		return -EINVAL;

	switch (idx) {
	case AMDGPU_PP_SENSOR_GFX_SCLK:
	case AMDGPU_PP_SENSOR_GFX_MCLK:
		if (!hwmgr->current_ps)
			return -EINVAL;
		ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
		if (!ps)
			return -EINVAL;
		pl_index = si_get_current_profile_index(hwmgr);
		if (pl_index >= ps->performance_level_count)
			return -EINVAL;
		if (idx == AMDGPU_PP_SENSOR_GFX_SCLK) {
			sclk = si_get_current_sclk(hwmgr);
			*((uint32_t *)value) = sclk ? sclk :
				ps->performance_levels[pl_index].sclk;
		} else {
			*((uint32_t *)value) = ps->performance_levels[pl_index].mclk;
		}
		*size = 4;
		return 0;
	case AMDGPU_PP_SENSOR_GPU_TEMP:
		/* Note: AMDGPU_PP_SENSOR_EDGE_TEMP is an alias of AMDGPU_PP_SENSOR_GPU_TEMP */
		*((uint32_t *)value) = si_dpm_get_temp(hwmgr);
		*size = 4;
		return 0;
	case AMDGPU_PP_SENSOR_VDDGFX:
		if (!hwmgr->current_ps)
			return -EINVAL;
		ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
		if (!ps)
			return -EINVAL;
		pl_index = si_get_current_profile_index(hwmgr);
		if (pl_index >= ps->performance_level_count)
			return -EINVAL;
		*((uint32_t *)value) = ps->performance_levels[pl_index].vddc;
		*size = 4;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

/* pp_dpm_sclk / pp_dpm_mclk / pp_dpm_pcie and the pp_od_clk_voltage
 * OD_* sections. SI has no per-domain DPM tables: a performance level
 * of the running state bundles sclk, mclk and pcie gen, so all three
 * files list the same levels with the SMC's current index marked.
 * amdgpu_pm reads these through emit_clock_levels only.
 */
static int si_emit_clock_levels(struct pp_hwmgr *hwmgr,
				enum pp_clock_type type, char *buf, int *offset)
{
	const struct si_power_state *ps;
	uint32_t now;
	int i, size = *offset;

	if (!hwmgr->current_ps)
		return -EINVAL;
	ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
	if (!ps)
		return -EINVAL;

	now = si_get_current_profile_index(hwmgr);

	switch (type) {
	case PP_SCLK:
		for (i = 0; i < ps->performance_level_count; i++)
			size += sysfs_emit_at(buf, size, "%d: %uMhz %s\n", i,
					      ps->performance_levels[i].sclk / 100,
					      (i == now) ? "*" : "");
		break;
	case PP_MCLK:
		for (i = 0; i < ps->performance_level_count; i++)
			size += sysfs_emit_at(buf, size, "%d: %uMhz %s\n", i,
					      ps->performance_levels[i].mclk / 100,
					      (i == now) ? "*" : "");
		break;
	case PP_PCIE:
		for (i = 0; i < ps->performance_level_count; i++)
			size += sysfs_emit_at(buf, size, "%d: gen%u %s\n", i,
					      ps->performance_levels[i].pcie_gen + 1,
					      (i == now) ? "*" : "");
		break;
	case OD_SCLK:
		if (hwmgr->od_enabled) {
			size += sysfs_emit_at(buf, size, "%s:\n", "OD_SCLK");
			for (i = 0; i < ps->performance_level_count; i++)
				size += sysfs_emit_at(buf, size, "%d: %10uMHz %10umV\n", i,
						      ps->performance_levels[i].sclk / 100,
						      ps->performance_levels[i].vddc);
		}
		break;
	case OD_MCLK:
		if (hwmgr->od_enabled) {
			size += sysfs_emit_at(buf, size, "%s:\n", "OD_MCLK");
			for (i = 0; i < ps->performance_level_count; i++)
				size += sysfs_emit_at(buf, size, "%d: %10uMHz %10umV\n", i,
						      ps->performance_levels[i].mclk / 100,
						      ps->performance_levels[i].vddci);
		}
		break;
	case OD_RANGE:
		if (hwmgr->od_enabled && ps->performance_level_count) {
			size += sysfs_emit_at(buf, size, "%s:\n", "OD_RANGE");
			size += sysfs_emit_at(buf, size, "SCLK: %7uMHz %10uMHz\n",
					      ps->performance_levels[0].sclk / 100,
					      SI_OLAND_OD_SCLK_MAX / 100);
			size += sysfs_emit_at(buf, size, "MCLK: %7uMHz %10uMHz\n",
					      ps->performance_levels[0].mclk / 100,
					      SI_OLAND_OD_MCLK_MAX / 100);
		}
		break;
	default:
		break;
	}

	*offset = size;
	return 0;
}

static int si_print_clock_levels(struct pp_hwmgr *hwmgr,
				 enum pp_clock_type type, char *buf)
{
	int size = 0;
	int ret;

	ret = si_emit_clock_levels(hwmgr, type, buf, &size);
	return ret ? ret : size;
}

/* pp_od_clk_voltage writes. Oland OD, as inherited from legacy si_dpm,
 * is a single override of the top performance level's clock (stored in
 * od_sclk/od_mclk and applied by si_apply_state_adjust_rules), so only
 * the top level accepts an edit and voltage is always derived from the
 * dependency tables: "s <top> <MHz>" / "m <top> <MHz>", with an optional
 * trailing mV that must echo the listed value. "r" drops both overrides;
 * "c" needs nothing here, amdgpu_pm follows it with a READJUST task.
 */
static int si_odn_edit_dpm_table(struct pp_hwmgr *hwmgr,
				 enum PP_OD_DPM_TABLE_COMMAND type,
				 long *input, uint32_t size)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct si_power_state *ps;
	uint32_t top, clk;

	if (!hwmgr->od_enabled || !si_oland_is_overdrive_supported(hwmgr))
		return -EOPNOTSUPP;

	switch (type) {
	case PP_OD_RESTORE_DEFAULT_TABLE:
		data->od_sclk = 0;
		data->od_mclk = 0;
		return 0;
	case PP_OD_COMMIT_DPM_TABLE:
		return 0;
	case PP_OD_EDIT_SCLK_VDDC_TABLE:
	case PP_OD_EDIT_MCLK_VDDC_TABLE:
		break;
	default:
		return -EINVAL;
	}

	if (!input || (size != 2 && size != 3))
		return -EINVAL;
	if (!hwmgr->current_ps)
		return -EINVAL;
	ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
	if (!ps || !ps->performance_level_count)
		return -EINVAL;
	top = ps->performance_level_count - 1;

	if (input[0] != top) {
		pr_info("SI OD can only edit the top level (%u)\n", top);
		return -EINVAL;
	}
	if (input[1] <= 0 || input[1] > SI_OLAND_OD_MCLK_MAX / 100)
		return -EINVAL;
	clk = (uint32_t)input[1] * 100;

	if (type == PP_OD_EDIT_SCLK_VDDC_TABLE) {
		if (size == 3 && input[2] != ps->performance_levels[top].vddc)
			return -EINVAL;
		if (clk < ps->performance_levels[0].sclk ||
		    clk > SI_OLAND_OD_SCLK_MAX)
			return -EINVAL;
		data->od_sclk = clk;
	} else {
		if (size == 3 && input[2] != ps->performance_levels[top].vddci)
			return -EINVAL;
		if (clk < ps->performance_levels[0].mclk)
			return -EINVAL;
		data->od_mclk = clk;
	}
	pr_debug("si OD edit %s level %u -> %u MHz (od_sclk %u od_mclk %u)\n",
		 type == PP_OD_EDIT_SCLK_VDDC_TABLE ? "sclk" : "mclk",
		 top, clk / 100, data->od_sclk / 100, data->od_mclk / 100);

	return 0;
}

/* SMC6 exposes the running state's levels through two knobs:
 * SetEnabledLevels(n) enables the n lowest levels, and SetForcedLevels(1)
 * pins the SMC to the highest level (0 releases it). The firmware
 * rejects the pin (PPSMC_Result_Failed, seen on Oland) unless every
 * level is enabled, so it is only usable for "top level only" - the
 * legacy 'high' sequence. A pp_dpm_* mask is rounded to what that can
 * express: its highest bit sets the ceiling, a raised floor or holes
 * cannot be enforced, and a single mid level is a ceiling the SMC
 * climbs to under load and may drop below at idle.
 * Levels are the running state's (hwmgr->current_ps, updated by
 * power_state_management() before phm_force_dpm_levels()).
 */
static int si_apply_level_mask(struct pp_hwmgr *hwmgr, uint32_t mask)
{
	const struct si_power_state *ps;
	uint32_t levels, all, top;
	bool pin_top;

	if (!hwmgr->current_ps)
		return -EINVAL;
	ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
	if (!ps || !ps->performance_level_count)
		return -EINVAL;
	levels = ps->performance_level_count;
	all = GENMASK(levels - 1, 0);

	mask &= all;
	if (!mask)
		mask = all;
	top = fls(mask);	/* number of enabled levels */
	pin_top = (mask == BIT(levels - 1)) && levels > 1;

	pr_debug("si level mask 0x%x of %u levels: enable %u, %s\n",
		 mask, levels, top, pin_top ? "pinned to top" : "auto below ceiling");

	if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetForcedLevels, 0))
		return -EINVAL;
	if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetEnabledLevels, top))
		return -EINVAL;
	if (pin_top &&
	    si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetForcedLevels, 1))
		return -EINVAL;

	return 0;
}

static int si_force_clock_level(struct pp_hwmgr *hwmgr,
				enum pp_clock_type type, uint32_t mask)
{
	struct si_hwmgr *data = hwmgr->backend;
	int ret;

	switch (type) {
	case PP_SCLK:
	case PP_MCLK:
	case PP_PCIE:
		break;
	default:
		return -EINVAL;
	}

	/* one level mask drives all three domains, last write wins */
	ret = si_apply_level_mask(hwmgr, mask);
	if (!ret)
		data->manual_level_mask = mask;
	return ret;
}

/* legacy si_dpm_force_performance_level() for high/low/auto; the
 * profiling levels are mapped onto the bundled SI levels the way smu7
 * picks them (peak = top, min_* = bottom, standard = one below top).
 */
static int si_force_dpm_level(struct pp_hwmgr *hwmgr,
			      enum amd_dpm_forced_level level)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct si_power_state *ps;
	uint32_t levels;
	int ret = 0;

	if (!hwmgr->current_ps)
		return -EINVAL;
	ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
	if (!ps || !ps->performance_level_count)
		return -EINVAL;
	levels = ps->performance_level_count;

	pr_debug("si force dpm level %d (was %d), %u levels\n",
		 level, hwmgr->dpm_level, levels);

	switch (level) {
	case AMD_DPM_FORCED_LEVEL_HIGH:
		if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetEnabledLevels, levels))
			return -EINVAL;
		if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetForcedLevels, 1))
			return -EINVAL;
		break;
	case AMD_DPM_FORCED_LEVEL_LOW:
		if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetForcedLevels, 0))
			return -EINVAL;
		if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetEnabledLevels, 1))
			return -EINVAL;
		break;
	case AMD_DPM_FORCED_LEVEL_AUTO:
		if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetForcedLevels, 0))
			return -EINVAL;
		if (si_send_msg_with_parameter(hwmgr, PPSMC_MSG_SetEnabledLevels, levels))
			return -EINVAL;
		break;
	case AMD_DPM_FORCED_LEVEL_MANUAL:
		/* Entering manual mode starts with every level enabled. A
		 * re-force while already manual is the READJUST after a
		 * state switch, which left the SMC restricted to level 0
		 * (si_restrict_performance_levels_before_switch): put the
		 * pp_dpm_* mask back.
		 */
		if (hwmgr->dpm_level != AMD_DPM_FORCED_LEVEL_MANUAL)
			data->manual_level_mask = 0;
		ret = si_apply_level_mask(hwmgr, data->manual_level_mask);
		break;
	case AMD_DPM_FORCED_LEVEL_PROFILE_PEAK:
		ret = si_apply_level_mask(hwmgr, BIT(levels - 1));
		break;
	case AMD_DPM_FORCED_LEVEL_PROFILE_MIN_SCLK:
	case AMD_DPM_FORCED_LEVEL_PROFILE_MIN_MCLK:
		ret = si_apply_level_mask(hwmgr, BIT(0));
		break;
	case AMD_DPM_FORCED_LEVEL_PROFILE_STANDARD:
		ret = si_apply_level_mask(hwmgr, BIT(levels > 1 ? levels - 2 : 0));
		break;
	case AMD_DPM_FORCED_LEVEL_PROFILE_EXIT:
	default:
		break;
	}
	if (ret)
		return ret;

	/* smu7 parity: peak profiling runs the fan flat out */
	if (level == AMD_DPM_FORCED_LEVEL_PROFILE_PEAK &&
	    hwmgr->dpm_level != AMD_DPM_FORCED_LEVEL_PROFILE_PEAK) {
		si_set_fan_control_mode(hwmgr, AMD_FAN_CTRL_MANUAL);
		si_fan_ctrl_set_fan_speed_pwm(hwmgr, 255);
	} else if (level != AMD_DPM_FORCED_LEVEL_PROFILE_PEAK &&
		   hwmgr->dpm_level == AMD_DPM_FORCED_LEVEL_PROFILE_PEAK) {
		si_fan_ctrl_reset_fan_speed_to_default(hwmgr);
	}

	return 0;
}

/* pp_set_power_limit() hands us Watts (limit 0 was already mapped to
 * hwmgr->default_power_limit and the +TDPODLimit% ceiling enforced).
 * Legacy Oland OD TDP is a percent delta over the VBIOS TDP; the SMC
 * limit is re-programmed by si_power_control_set_level() on the next
 * state switch (the caller dispatches READJUST_POWER_STATE).
 */
static int si_set_power_limit(struct pp_hwmgr *hwmgr, uint32_t n)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t tdp = hwmgr->platform_descriptor.TDPLimit;
	uint32_t adjust;

	if (!si_oland_is_overdrive_supported(hwmgr))
		return -EOPNOTSUPP;
	if (!tdp)
		return -EINVAL;
	if (n < tdp)
		return -EINVAL;
	/* Round to nearest percent so the applied gain matches the
	 * requested limit (floor division would keep up to ~1W of
	 * requested headroom unapplied).
	 */
	adjust = DIV_ROUND_CLOSEST((n - tdp) * 100, tdp);
	if (adjust > SI_OLAND_OD_TDP_MAX)
		return -EINVAL;
	if (adjust > data->tdp_od_limit)
		return -EINVAL;
	data->od_tdp = adjust;
	hwmgr->platform_descriptor.TDPAdjustment = adjust;

	if (data->enable_power_containment)
		return si_power_control_set_level(hwmgr);

	return 0;
}

static void si_powergate_uvd(struct pp_hwmgr *hwmgr, bool bgate)
{
	/* SI UVD has no SMC power gating (legacy si_dpm_powergate_uvd is
	 * #if 0); the UVD state is picked via adev->pm.dpm.uvd_active in
	 * si_reseed_request_state().
	 */
}

static void si_powergate_vce(struct pp_hwmgr *hwmgr, bool bgate)
{
	/* Likewise: VCE clocks follow the state (si_set_vce_clock). */
}

static const struct amdgpu_irq_src_funcs si_irq_funcs = {
	.process = phm_irq_process,
};

/* Legacy si_dpm_sw_init() owned IH src 230/231 (CG_TSS thermal
 * low->high / high->low, same ids as CI/VI); nothing else registers
 * them once the PowerPlay block replaces si_dpm, so an over-temperature
 * event would be dropped. phm_irq_process logs it and arms the SW CTF
 * check the way smu7 does. SI has no GPIO_19 hardware CTF interrupt.
 */

static int si_get_power_profile_mode(struct pp_hwmgr *hwmgr, char *buf)
{
	struct si_hwmgr *data = hwmgr->backend;
	uint32_t i, size = 0;

	if (!buf)
		return -EINVAL;

	phm_get_sysfs_buf(&buf, &size);

	size += sysfs_emit_at(buf, size, "%3s %14s %10s %10s %10s\n",
			      "NUM", "MODE_NAME", "ACTIVITY", "UP_HYST", "DOWN_HYST");

	for (i = 0; i < ARRAY_SIZE(data->profile); i++) {
		/* the SMC-side profiles other ASICs expose have no meaning here */
		if (i > PP_SMC_POWER_PROFILE_CUSTOM)
			break;
		size += sysfs_emit_at(buf, size, "%3u %14s %s %8u%% %10u %10u\n",
				      i, amdgpu_pp_profile_name[i],
				      i == hwmgr->power_profile_mode ? "*" : " ",
				      data->profile[i].activity,
				      data->profile[i].up_hyst,
				      data->profile[i].down_hyst);
	}

	return size;
}

static int si_set_power_profile_mode(struct pp_hwmgr *hwmgr, long *input, uint32_t size)
{
	struct si_hwmgr *data = hwmgr->backend;
	const struct si_power_state *ps;
	uint32_t mode;
	int ret;

	if (!input)
		return -EINVAL;

	mode = input[size];
	if (mode > PP_SMC_POWER_PROFILE_CUSTOM)
		return -EINVAL;

	if (mode == PP_SMC_POWER_PROFILE_CUSTOM) {
		if (size != 3 && size != 0)
			return -EINVAL;
		if (size == 3) {
			if (input[0] < 10 || input[0] > 300 ||
			    input[1] < 1 || input[1] > 100 ||
			    input[2] < 1 || input[2] > 100)
				return -EINVAL;
			data->profile[mode].activity = input[0];
			data->profile[mode].up_hyst = input[1];
			data->profile[mode].down_hyst = input[2];
		}
		/* the framework only ever asks for a profile out of
		 * workload_setting[], which never holds CUSTOM, so this is
		 * always a request from userspace
		 */
		data->custom_profile = true;
	} else if (data->custom_profile && size == 0) {
		/* psm_adjust_power_state_dynamic() re-asserts its workload
		 * choice on every state adjust; leave the custom profile the
		 * user asked for in place
		 */
		return 0;
	}

	if (mode == hwmgr->power_profile_mode)
		return 0;

	if (mode != PP_SMC_POWER_PROFILE_CUSTOM) {
		data->custom_profile = false;
		/* agree with the framework's own accounting, or it reverts
		 * this on the next state adjust
		 */
		hwmgr->workload_mask = 1 << hwmgr->workload_prority[mode];
	}

	hwmgr->power_profile_mode = mode;

	/* the thresholds live in the state table the SMC is running, so the
	 * driver state has to be rebuilt and uploaded for this to take
	 */
	if (!hwmgr->current_ps)
		return 0;
	ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
	if (!ps)
		return 0;

	ret = si_smc_upload_sw_state(hwmgr, ps);
	if (ret) {
		pr_err("si: failed to upload the state for profile %u\n", mode);
		return ret;
	}

	return si_set_sw_state(hwmgr);
}
static int si_register_irq_handlers(struct pp_hwmgr *hwmgr)
{
	struct amdgpu_irq_src *source = kzalloc_obj(struct amdgpu_irq_src);

	if (!source)
		return -ENOMEM;

	source->funcs = &si_irq_funcs;

	amdgpu_irq_add_id((struct amdgpu_device *)hwmgr->adev,
			  AMDGPU_IRQ_CLIENTID_LEGACY,
			  VISLANDS30_IV_SRCID_CG_TSS_THERMAL_LOW_TO_HIGH,
			  source);
	amdgpu_irq_add_id((struct amdgpu_device *)hwmgr->adev,
			  AMDGPU_IRQ_CLIENTID_LEGACY,
			  VISLANDS30_IV_SRCID_CG_TSS_THERMAL_HIGH_TO_LOW,
			  source);

	return 0;
}

static int si_get_performance_level(struct pp_hwmgr *hwmgr,
				    const struct pp_hw_power_state *state,
				    PHM_PerformanceLevelDesignation designation,
				    uint32_t index, PHM_PerformanceLevel *level)
{
	const struct si_power_state *ps;
	uint32_t i;

	if (level == NULL || hwmgr == NULL || state == NULL)
		return -EINVAL;

	ps = cast_const_phw_si_power_state(state);
	if (!ps || ps->performance_level_count == 0)
		return -EINVAL;

	i = index > ps->performance_level_count - 1 ?
			ps->performance_level_count - 1 : index;

	level->coreClock = ps->performance_levels[i].sclk;
	level->memory_clock = ps->performance_levels[i].mclk;

	return 0;
}

static int si_power_off_asic(struct pp_hwmgr *hwmgr)
{
	return si_disable_dpm_tasks(hwmgr);
}

/* ---- GPU hang snapshot ---- */

/* What the SMC and the memory controller were doing when a ring timed
 * out: which DPM level the SMC was in (and heading for), whether it is
 * still executing (PC samples) or stuck on a message, which MC arbiter
 * set and MPLL settings are live, and the display gap / stutter state it
 * synchronises MCLK switches against. Read before the reset clobbers it.
 */
#define SI_DPG_PIPE_STUTTER_CONTROL	0x1B35	/* dce_6_0_d.h, crtc 0 */

/* engine and memory controller status, to see which block a stalled ring is
 * stuck in (offsets from gfx_6_0_d.h / oss_1_0_d.h, kept local so this file
 * does not pull in a second big register header)
 */
#define SI_GRBM_STATUS			0x2004
#define SI_GRBM_STATUS2			0x2002
#define SI_SRBM_STATUS			0x0394
#define SI_CP_STAT			0x21A0
#define SI_CP_BUSY_STAT			0x219F
#define SI_CP_STALLED_STAT1		0x219D
#define SI_CP_STALLED_STAT2		0x219E
#define SI_CP_STALLED_STAT3		0x219C

static const struct {
	const char *name;
	uint32_t reg;
} si_hang_regs[] = {
	{ "TARGET_AND_CURRENT_PROFILE_INDEX", mmTARGET_AND_CURRENT_PROFILE_INDEX },
	{ "GENERAL_PWRMGT",		mmGENERAL_PWRMGT },
	{ "SCLK_PWRMGT_CNTL",		mmSCLK_PWRMGT_CNTL },
	{ "CG_SPLL_FUNC_CNTL",		mmCG_SPLL_FUNC_CNTL },
	{ "CG_SPLL_FUNC_CNTL_3",	mmCG_SPLL_FUNC_CNTL_3 },
	{ "MCLK_PWRMGT_CNTL",		mmMCLK_PWRMGT_CNTL },
	{ "MPLL_FUNC_CNTL",		mmMPLL_FUNC_CNTL },
	{ "MPLL_FUNC_CNTL_1",		mmMPLL_FUNC_CNTL_1 },
	{ "MC_ARB_CG",			mmMC_ARB_CG },
	{ "MC_ARB_DRAM_TIMING",		mmMC_ARB_DRAM_TIMING },
	{ "MC_ARB_DRAM_TIMING_1",	mmMC_ARB_DRAM_TIMING_1 },
	{ "MC_ARB_DRAM_TIMING_2",	SI_MC_ARB_DRAM_TIMING_2 },
	{ "MC_ARB_DRAM_TIMING_3",	SI_MC_ARB_DRAM_TIMING_3 },
	{ "MC_ARB_BURST_TIME",		mmMC_ARB_BURST_TIME },
	{ "MC_SEQ_STATUS_M",		mmMC_SEQ_STATUS_M },
	{ "MC_SEQ_CG",			mmMC_SEQ_CG },
	{ "MC_SEQ_MISC0",		mmMC_SEQ_MISC0 },
	{ "CG_DISPLAY_GAP_CNTL",	mmCG_DISPLAY_GAP_CNTL },
	{ "DPG_PIPE_STUTTER_CONTROL",	SI_DPG_PIPE_STUTTER_CONTROL },
	{ "SMC_MESSAGE_0",		mmSMC_MESSAGE_0 },
	{ "SMC_RESP_0",			mmSMC_RESP_0 },
	{ "GRBM_STATUS",		SI_GRBM_STATUS },
	{ "GRBM_STATUS2",		SI_GRBM_STATUS2 },
	{ "SRBM_STATUS",		SI_SRBM_STATUS },
	{ "CP_STAT",			SI_CP_STAT },
	{ "CP_BUSY_STAT",		SI_CP_BUSY_STAT },
	{ "CP_STALLED_STAT1",		SI_CP_STALLED_STAT1 },
	{ "CP_STALLED_STAT2",		SI_CP_STALLED_STAT2 },
	{ "CP_STALLED_STAT3",		SI_CP_STALLED_STAT3 },
	{ "VM_L2_STATUS",		mmVM_L2_STATUS },
};

static uint32_t si_snap_reg(const struct si_state_snapshot *s, uint32_t reg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(si_hang_regs); i++)
		if (si_hang_regs[i].reg == reg)
			return s->regs[i];
	return 0;
}

static void si_capture_state(struct pp_hwmgr *hwmgr, struct si_state_snapshot *s)
{
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(si_hang_regs) > SI_HANG_SNAPSHOT_MAX_REGS);
	memset(s, 0, sizeof(*s));
	for (i = 0; i < ARRAY_SIZE(si_hang_regs); i++)
		s->regs[i] = si_rreg(hwmgr, si_hang_regs[i].reg);
	for (i = 0; i < SI_HANG_SNAPSHOT_PC_SAMPLES; i++) {
		s->smc_pc[i] = si_rreg_smc(hwmgr, ixSMC_PC_C);
		udelay(20);
	}
}

static void si_print_levels(struct pp_hwmgr *hwmgr, struct drm_printer *p,
			    const char *tag)
{
	const struct si_power_state *ps = NULL;
	int i;

	if (hwmgr->current_ps)
		ps = cast_const_phw_si_power_state(&hwmgr->current_ps->hardware);
	if (!ps)
		return;

	for (i = 0; i < ps->performance_level_count; i++)
		drm_printf(p, "si %s: uploaded level %d: sclk %u mclk %u vddc %u vddci %u\n",
			   tag, i, ps->performance_levels[i].sclk / 100,
			   ps->performance_levels[i].mclk / 100,
			   ps->performance_levels[i].vddc,
			   ps->performance_levels[i].vddci);
}

static void si_print_snapshot(struct pp_hwmgr *hwmgr, struct drm_printer *p,
			      const struct si_state_snapshot *s, const char *tag)
{
	char pc[SI_HANG_SNAPSHOT_PC_SAMPLES * 9 + 1];
	uint32_t idx = si_snap_reg(s, mmTARGET_AND_CURRENT_PROFILE_INDEX);
	uint32_t arb = si_snap_reg(s, mmMC_ARB_CG);
	int i, len = 0;

	drm_printf(p, "si %s: SMC level current %u, target field 0x%x (raw 0x%08x)\n", tag,
		   (uint32_t)((idx & TARGET_AND_CURRENT_PROFILE_INDEX__CURRENT_STATE_INDEX_MASK) >>
			      TARGET_AND_CURRENT_PROFILE_INDEX__CURRENT_STATE_INDEX__SHIFT),
		   idx & 0xf, idx);
	for (i = 0; i < ARRAY_SIZE(si_hang_regs); i++)
		drm_printf(p, "si %s: %-32s 0x%08x\n", tag,
			   si_hang_regs[i].name, s->regs[i]);
	drm_printf(p, "si %s: MC arbiter set req 0x%02x resp 0x%02x (0x0a = F0 ... 0x0d = F3)\n", tag,
		   (uint32_t)((arb & MC_ARB_CG__CG_ARB_REQ_MASK) >> MC_ARB_CG__CG_ARB_REQ__SHIFT),
		   (uint32_t)((arb & MC_ARB_CG__CG_ARB_RESP_MASK) >> MC_ARB_CG__CG_ARB_RESP__SHIFT));
	for (i = 0; i < SI_HANG_SNAPSHOT_PC_SAMPLES; i++)
		len += scnprintf(pc + len, sizeof(pc) - len, " %08x", s->smc_pc[i]);
	drm_printf(p, "si %s: SMC PC samples (20 us apart):%s\n", tag, pc);
	si_print_levels(hwmgr, p, tag);
}

static void si_dump_state(struct pp_hwmgr *hwmgr)
{
	struct si_hwmgr *data = hwmgr->backend;
	struct amdgpu_device *adev = hwmgr->adev;
	struct drm_printer p = drm_info_printer(adev->dev);

	if (!data)
		return;

	si_capture_state(hwmgr, &data->hang.snap);
	data->hang.valid = true;

	/* also to dmesg: the capture scripts keep that even when the
	 * devcoredump is not collected
	 */
	si_print_snapshot(hwmgr, &p, &data->hang.snap, "hang");
}

static void si_print_state(struct pp_hwmgr *hwmgr, struct drm_printer *p)
{
	struct si_hwmgr *data = hwmgr->backend;

	if (!data || !data->hang.valid)
		return;
	drm_printf(p, "\nSI power management at hang\n");
	si_print_snapshot(hwmgr, p, &data->hang.snap, "hang");
}

static const struct pp_hwmgr_func si_hwmgr_funcs = {
	.dump_state = si_dump_state,
	.print_state = si_print_state,
	.backend_init = si_hwmgr_backend_init,
	.backend_fini = si_hwmgr_backend_fini,
	.asic_setup = si_setup_asic_task,
	.dynamic_state_management_enable = si_enable_dpm_tasks,
	.dynamic_state_management_disable = si_disable_dpm_tasks,
	.power_off_asic = si_power_off_asic,
	.get_power_state_size = si_get_power_state_size,
	.get_num_of_pp_table_entries = si_get_num_of_pp_table_entries,
	.get_pp_table_entry = si_get_pp_table_entry,
	.patch_boot_state = si_patch_boot_state,
	.apply_state_adjust_rules = si_apply_state_adjust_rules,
	.power_state_set = si_set_power_state_tasks,
	.check_states_equal = si_check_states_equal,
	.check_smc_update_required_for_display_configuration =
		si_check_smc_update_required_for_display_configuration,
	.display_config_changed = si_program_display_gap,
	.force_dpm_level = si_force_dpm_level,
	.force_clock_level = si_force_clock_level,
	.get_sclk = si_dpm_get_sclk,
	.get_mclk = si_dpm_get_mclk,
	.get_performance_level = si_get_performance_level,
	.powergate_uvd = si_powergate_uvd,
	.powergate_vce = si_powergate_vce,
	.get_sclk_od = si_get_sclk_od,
	.set_sclk_od = si_set_sclk_od,
	.get_mclk_od = si_get_mclk_od,
	.set_mclk_od = si_set_mclk_od,
	.read_sensor = si_read_sensor,
	.print_clock_levels = si_print_clock_levels,
	.emit_clock_levels = si_emit_clock_levels,
	.odn_edit_dpm_table = si_odn_edit_dpm_table,
	.set_power_limit = si_set_power_limit,
	.start_thermal_controller = si_start_thermal_controller,
	.stop_thermal_controller = si_thermal_stop_thermal_controller,
	.get_power_profile_mode = si_get_power_profile_mode,
	.set_power_profile_mode = si_set_power_profile_mode,
	.get_thermal_temperature_range = si_get_thermal_temperature_range,
	.get_fan_speed_info = si_fan_ctrl_get_fan_speed_info,
	.get_fan_speed_pwm = si_fan_ctrl_get_fan_speed_pwm,
	.set_fan_speed_pwm = si_fan_ctrl_set_fan_speed_pwm,
	.get_fan_speed_rpm = si_fan_ctrl_get_fan_speed_rpm,
	.set_fan_speed_rpm = si_fan_ctrl_set_fan_speed_rpm,
	.reset_fan_speed_to_default = si_fan_ctrl_reset_fan_speed_to_default,
	.set_fan_control_mode = si_set_fan_control_mode,
	.get_fan_control_mode = si_get_fan_control_mode,
	.register_irq_handlers = si_register_irq_handlers,
};

int si_init_function_pointers(struct pp_hwmgr *hwmgr)
{
	hwmgr->hwmgr_func = &si_hwmgr_funcs;
	/* SI VBIOS uses the legacy PPLIB layout like CI Bonaire (V0). */
	hwmgr->pptable_func = &pptable_funcs;
	return 0;
}
