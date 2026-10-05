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
 * Backend state mirrors the legacy rv7xx/evergreen/ni/si power_info
 * chain from legacy-dpm/si_dpm.h, flattened into one struct so the
 * si_dpm.c functions port with s/si_pi->/data->/ style edits. Legacy
 * field names are kept on purpose. Do not include legacy si_dpm.h here:
 * it defines a conflicting struct si_mc_reg_table (see si_smumgr.h).
 */
#ifndef _SI_HWMGR_H_
#define _SI_HWMGR_H_

#include "hwmgr.h"
#include "amdgpu_atombios.h"
#include "sislands_smc.h"

/* SI Oland OverDrive limits, mirrored from legacy-dpm/si_dpm.h.
 * Clocks in 10 kHz units (100000 = 1000 MHz), TDP in percent.
 */
#define SI_OLAND_OD_SCLK_MAX 100000
#define SI_OLAND_OD_MCLK_MAX 130000
#define SI_OLAND_OD_TDP_MAX  20

/* legacy si_dpm.h constants used by the state parser / init */
#define SI_MAX_HARDWARE_POWERLEVELS	5
#define SI_LEAKAGE_INDEX0		0xff01
#define SI_MAX_LEAKAGE_COUNT		4
#define SI_ULVVOLTAGECHANGEDELAY_DFLT	1687
#define SI_CGULVPARAMETER_DFLT		0x00040035
#define SI_CGULVCONTROL_DFLT		0x1f007550
#define SI_VRC_DFLT			0xC000B3
#define SI_ASI_DFLT			1000		/* RV770_ASI_DFLT */
#define SI_HASI_DFLT			400000		/* CYPRESS_HASI_DFLT */
#define SI_DEFAULT_VCLK_FREQ		53300		/* 10 kHz */
#define SI_DEFAULT_DCLK_FREQ		40000		/* 10 kHz */
#define SI_VOLTAGERESPONSETIME_DFLT	1000
#define SI_BACKBIASRESPONSETIME_DFLT	1000
#define SI_REFERENCEDIVIDER_DFLT	4

/* SMC state table slot / arbiter indices (legacy si_dpm.h) */
#define SISLANDS_INITIAL_STATE_ARB_INDEX	0
#define SISLANDS_ACPI_STATE_ARB_INDEX		1
#define SISLANDS_ULV_STATE_ARB_INDEX		2
#define SISLANDS_DRIVER_STATE_ARB_INDEX		3

#define SISLANDS_MCREGISTERTABLE_INITIAL_SLOT		0
#define SISLANDS_MCREGISTERTABLE_ACPI_SLOT		1
#define SISLANDS_MCREGISTERTABLE_ULV_SLOT		2
#define SISLANDS_MCREGISTERTABLE_FIRST_DRIVERSTATE_SLOT	3

#define MC_CG_ARB_FREQ_F0	0x0a
#define MC_CG_ARB_FREQ_F1	0x0b
#define MC_CG_ARB_FREQ_F2	0x0c
#define MC_CG_ARB_FREQ_F3	0x0d

#define SCLK_MIN_DEEPSLEEP_FREQ	1350

/* DPM2 / power containment tunables (legacy si_dpm.h) */
#define SISLANDS_DPM2_MAX_PULSE_SKIP			256
#define SISLANDS_DPM2_NEAR_TDP_DEC			10
#define SISLANDS_DPM2_ABOVE_SAFE_INC			5
#define SISLANDS_DPM2_BELOW_SAFE_INC			20
#define SISLANDS_DPM2_TDP_SAFE_LIMIT_PERCENT		80
#define SISLANDS_DPM2_MAXPS_PERCENT_H			99
#define SISLANDS_DPM2_MAXPS_PERCENT_M			99
#define SISLANDS_DPM2_SQ_RAMP_MAX_POWER			0x3FFF
#define SISLANDS_DPM2_SQ_RAMP_MIN_POWER			0x12
#define SISLANDS_DPM2_SQ_RAMP_MAX_POWER_DELTA		0x15
#define SISLANDS_DPM2_SQ_RAMP_STI_SIZE			0x1E
#define SISLANDS_DPM2_SQ_RAMP_LTI_RATIO			0xF
#define SISLANDS_DPM2_PWREFFICIENCYRATIO_MARGIN		10

/* Extra MC arbiter timing sets, not in gmc_6_0_d.h (legacy si_dpm.h) */
#define SI_MC_ARB_DRAM_TIMING_2		0x9fd
#define SI_MC_ARB_DRAM_TIMING_3		0x9fe
#define SI_MC_ARB_DRAM_TIMING2_2	0xa00
#define SI_MC_ARB_DRAM_TIMING2_3	0xa01

/* DCCG display slow select (amdgpu sid.h) */
#define SI_DCCG_DISP_SLOW_SELECT_REG	0x13F
#define SI_DCCG_DISP1_SLOW_SELECT_MASK	(7 << 0)
#define SI_DCCG_DISP1_SLOW_SELECT_SHIFT	0

/* GPU hang snapshot sizes, see si_dump_state() */
#define SI_HANG_SNAPSHOT_MAX_REGS	40
#define SI_HANG_SNAPSHOT_PC_SAMPLES	8

/* CG IND registers are accessed via SMC indirect space (amdgpu sid.h) */
#define SI_SMC_CG_IND_START	0xc0030000
#define SI_SMC_CG_IND_END	0xc0040000

/* fan control modes (CG_FDO_CTRL2.FDO_PWM_MODE) */
#define SI_FDO_PWM_MODE_STATIC		1
#define SI_FDO_PWM_MODE_STATIC_RPM	5

/* legacy R600_TEMP_RANGE_MIN/MAX thermal trip points, millidegrees */
#define SI_TEMP_RANGE_MIN	(90 * 1000)	/* legacy low trip, unused by PowerPlay */
#define SI_TEMP_RANGE_MAX	(120 * 1000)

enum si_dpm_auto_throttle_src {
	SI_DPM_AUTO_THROTTLE_SRC_THERMAL,
	SI_DPM_AUTO_THROTTLE_SRC_EXTERNAL
};

enum si_dpm_event_src {
	SI_DPM_EVENT_SRC_ANALOG = 0,
	SI_DPM_EVENT_SRC_EXTERNAL = 1,
	SI_DPM_EVENT_SRC_DIGITAL = 2,
	SI_DPM_EVENT_SRC_ANALOG_OR_EXTERNAL = 3,
	SI_DPM_EVENT_SRC_DIGIAL_OR_EXTERNAL = 4
};

enum si_pcie_gen {
	SI_PCIE_GEN1 = 0,
	SI_PCIE_GEN2 = 1,
	SI_PCIE_GEN3 = 2,
	SI_PCIE_GEN_INVALID = 0xffff
};

enum si_dc_cac_level {
	SI_DCCAC_LEVEL_0 = 0,
	SI_DCCAC_LEVEL_1,
	SI_DCCAC_LEVEL_2,
	SI_DCCAC_LEVEL_3,
	SI_DCCAC_LEVEL_4,
	SI_DCCAC_LEVEL_5,
	SI_DCCAC_LEVEL_6,
	SI_DCCAC_LEVEL_7,
	SI_DCCAC_MAX_LEVELS
};

enum si_cac_config_reg_type {
	SI_CACCONFIG_MMR = 0,
	SI_CACCONFIG_CGIND,
	SI_CACCONFIG_MAX
};
#define SISLANDS_CACCONFIG_MMR   SI_CACCONFIG_MMR
#define SISLANDS_CACCONFIG_CGIND SI_CACCONFIG_CGIND
#define SISLANDS_CACCONFIG_MAX   SI_CACCONFIG_MAX
#define NISLANDS_DCCAC_MAX_LEVELS SI_DCCAC_MAX_LEVELS


struct si_leakage_coeffients {
	uint32_t at;
	uint32_t bt;
	uint32_t av;
	uint32_t bv;
	int32_t t_slope;
	int32_t t_intercept;
	uint32_t t_ref;
};

struct si_cac_config_reg {
	uint32_t offset;
	uint32_t mask;
	uint32_t shift;
	uint32_t value;
	enum si_cac_config_reg_type type;
};

struct si_powertune_data {
	uint32_t cac_window;
	uint32_t l2_lta_window_size_default;
	uint8_t lts_truncate_default;
	uint8_t shift_n_default;
	uint8_t operating_temp;
	struct si_leakage_coeffients leakage_coefficients;
	uint32_t fixed_kt;
	uint32_t lkge_lut_v0_percent;
	uint8_t dc_cac[SI_DCCAC_MAX_LEVELS];
	bool enable_powertune_by_default;
};

struct si_dyn_powertune_data {
	uint32_t cac_leakage;
	int32_t leakage_minimum_temperature;
	uint32_t wintime;
	uint32_t l2_lta_window_size;
	uint8_t lts_truncate;
	uint8_t shift_n;
	uint8_t dc_pwr_value;
	bool disable_uvd_powertune;
};

struct si_dte_data {
	uint32_t tau[SMC_SISLANDS_DTE_MAX_FILTER_STAGES];
	uint32_t r[SMC_SISLANDS_DTE_MAX_FILTER_STAGES];
	uint32_t k;
	uint32_t t0;
	uint32_t max_t;
	uint8_t window_size;
	uint8_t temp_select;
	uint8_t dte_mode;
	uint8_t tdep_count;
	uint8_t t_limits[SMC_SISLANDS_DTE_MAX_TEMPERATURE_DEPENDENT_ARRAY_SIZE];
	uint32_t tdep_tau[SMC_SISLANDS_DTE_MAX_TEMPERATURE_DEPENDENT_ARRAY_SIZE];
	uint32_t tdep_r[SMC_SISLANDS_DTE_MAX_TEMPERATURE_DEPENDENT_ARRAY_SIZE];
	uint32_t t_threshold;
	bool enable_dte_by_default;
};

struct si_clock_registers {
	uint32_t cg_spll_func_cntl;
	uint32_t cg_spll_func_cntl_2;
	uint32_t cg_spll_func_cntl_3;
	uint32_t cg_spll_func_cntl_4;
	uint32_t cg_spll_spread_spectrum;
	uint32_t cg_spll_spread_spectrum_2;
	uint32_t dll_cntl;
	uint32_t mclk_pwrmgt_cntl;
	uint32_t mpll_ad_func_cntl;
	uint32_t mpll_dq_func_cntl;
	uint32_t mpll_func_cntl;
	uint32_t mpll_func_cntl_1;
	uint32_t mpll_func_cntl_2;
	uint32_t mpll_ss1;
	uint32_t mpll_ss2;
};

struct si_leakage_voltage_entry {
	uint16_t voltage;
	uint16_t leakage_index;
};

struct si_leakage_voltage {
	uint16_t count;
	struct si_leakage_voltage_entry entries[SI_MAX_LEAKAGE_COUNT];
};

/* legacy rv7xx_pl */
struct si_performance_level {
	uint32_t sclk;
	uint32_t mclk;
	uint16_t vddc;
	uint16_t vddci;
	uint32_t flags;
	enum si_pcie_gen pcie_gen;
};

struct si_ulv_param {
	bool supported;
	uint32_t cg_ulv_control;
	uint32_t cg_ulv_parameter;
	uint32_t volt_change_delay;
	struct si_performance_level pl;
	bool one_pcie_lane_in_ulv;
};

/* Hardware power state stored in pp_power_state.hardware (legacy si_ps
 * plus the amdgpu_ps fields the SI state machine consumes).
 */
struct si_power_state {
	uint32_t magic;
	uint16_t performance_level_count;
	/* level count as parsed from the VBIOS, before any ladder quirk
	 * shortened it; keeps the SMC activity thresholds of the top pair
	 * where the full ladder had them (si_populate_smc_t)
	 */
	uint16_t vbios_level_count;
	/* set when a ladder quirk collapsed the levels: the VBIOS sclk of the
	 * level that followed the last surviving low level, and of the level
	 * below the top, so si_populate_smc_t() can rebuild the original
	 * ladder's entry/exit activity thresholds (0 = not collapsed)
	 */
	uint32_t collapsed_next_sclk;
	uint32_t collapsed_below_top_sclk;
	bool dc_compatible;
	struct si_performance_level performance_levels[SISLANDS_MAX_SMC_PERFORMANCE_LEVELS_PER_SWSTATE];
	/* legacy amdgpu_ps: */
	uint32_t vclk;
	uint32_t dclk;
	uint32_t evclk;
	uint32_t ecclk;
};

/* Power management registers and SMC program counter samples, see
 * si_capture_state(): taken at a GPU hang (hwmgr->backend->hang) or on
 * demand for the debugfs live_state baseline.
 */
struct si_state_snapshot {
	uint32_t regs[SI_HANG_SNAPSHOT_MAX_REGS];
	uint32_t smc_pc[SI_HANG_SNAPSHOT_PC_SAMPLES];
};

/* Oland-first backend state. Big SI dies (Tahiti/Pitcairn/Verde) reuse
 * the same struct; Hainan (Oland rebrand, no VCE) is handled by flags.
 */
struct si_hwmgr {
	/* --- legacy rv7xx_power_info --- */
	bool voltage_control; /* vddc */
	bool mvdd_control;
	bool sclk_ss;
	bool mclk_ss;
	bool dynamic_ss;
	bool thermal_protection;
	uint32_t mvdd_split_frequency;
	uint16_t max_vddc;
	uint16_t max_vddc_in_table;
	uint16_t min_vddc_in_table;
	uint16_t acpi_vddc;
	uint32_t ref_div;
	uint32_t active_auto_throttle_sources;
	uint32_t mclk_stutter_mode_threshold;
	uint32_t mclk_strobe_mode_threshold;
	uint32_t mclk_edc_enable_threshold;
	uint32_t bsp;
	uint32_t bsu;
	uint32_t pbsp;
	uint32_t pbsu;
	uint32_t dsp;
	uint32_t psp;
	uint32_t asi;
	uint32_t pasi;
	uint32_t vrc;

	/* --- legacy evergreen_power_info --- */
	bool vddci_control;
	bool dynamic_ac_timing;
	bool abm;
	bool mcls;
	bool pcie_performance_request;
	bool sclk_deep_sleep;
	bool smu_uvd_hs;
	bool uvd_enabled;
	uint16_t acpi_vddci;
	uint32_t mclk_edc_wr_enable_threshold;
	struct atom_voltage_table vddc_voltage_table;
	struct atom_voltage_table vddci_voltage_table;

	/* --- legacy ni_power_info --- */
	uint32_t mclk_rtt_mode_threshold;
	bool support_cac_long_term_average;
	bool cac_enabled;
	bool cac_configuration_required;
	bool driver_calculate_cac_leakage;
	bool enable_power_containment;
	bool enable_cac;
	bool enable_sq_ramping;
	/* legacy ni_pi->current_ps / requested_ps: PowerPlay keeps these
	 * in hwmgr->current_ps / hwmgr->request_ps (struct si_power_state
	 * inside pp_power_state.hardware).
	 */

	/* --- legacy si_power_info --- */
	struct si_clock_registers clock_registers;
	struct atom_voltage_table mvdd_voltage_table;
	struct atom_voltage_table vddc_phase_shed_table;
	struct si_leakage_voltage leakage_voltage;
	uint16_t mvdd_bootup_value;
	struct si_ulv_param ulv;
	uint32_t max_cu;
	/* pcie gen */
	enum si_pcie_gen force_pcie_gen;
	enum si_pcie_gen boot_pcie_gen;
	enum si_pcie_gen acpi_pcie_gen;
	uint32_t sys_pcie_mask;
	/* flags */
	bool enable_dte;
	bool enable_ppm;
	bool vddc_phase_shed_control;
	bool pspp_notify_required;
	bool sclk_deep_sleep_above_low;
	bool voltage_control_svi2;
	bool vddci_control_svi2;
	/* CAC stuff (tables live in si_powertune.c once ported) */
	const struct si_cac_config_reg *cac_weights;
	const struct si_cac_config_reg *lcac_config;
	const struct si_cac_config_reg *cac_override;
	const struct si_powertune_data *powertune_data;
	struct si_dyn_powertune_data dyn_powertune_data;
	/* DTE stuff */
	struct si_dte_data dte_data;
	/* SVI2 */
	uint8_t svd_gpio_id;
	uint8_t svc_gpio_id;
	/* fan control */
	bool fan_ctrl_is_in_default_mode;
	uint32_t t_min;
	uint32_t fan_ctrl_default_mode;
	bool fan_is_controlled_by_smc;

	/* --- PowerPlay-side additions --- */
	/* VBIOS boot clocks/voltages (legacy adev->clock.default_* +
	 * amdgpu_atombios_get_default_voltages) captured for patch_boot_state.
	 */
	struct {
		uint32_t sclk_bootup_value;
		uint32_t mclk_bootup_value;
		uint16_t vddc_bootup_value;
		uint16_t vddci_bootup_value;
		uint16_t mvdd_bootup_value;
	} vbios_boot_state;
	/* legacy adev->pm.dpm.* scalars that PowerPlay has no slot for */
	uint32_t voltage_response_time;
	uint32_t backbias_response_time;
	uint32_t tdp_od_limit;	/* percent, legacy adev->pm.dpm.tdp_od_limit */
	bool power_control;	/* legacy adev->pm.dpm.power_control */
	/* legacy adev->pm.dpm.vce_states[] clock lookups happen in
	 * apply_state_adjust_rules via hwmgr->dyn_state.vce_clock_voltage_dependency_table.
	 */

	/* OverDrive, 10 kHz units, 0 = disabled (legacy si_pi->od_*) */
	uint32_t od_sclk;
	uint32_t od_mclk;
	/* amdgpu.si_dpm_quirks plus the per-board defaults, see si_hwmgr_backend_init */
	unsigned int dpm_quirks;
	/* pp_power_profile_mode: how hard the SMC is asked to chase activity.
	 * activity scales the target the up/down thresholds are built around
	 * (percent of the VBIOS-derived default), up_hyst and down_hyst are
	 * the widths either side of it, in the units R600_AH_DFLT uses.
	 */
	struct si_activity_profile {
		uint32_t activity;
		uint32_t up_hyst;
		uint32_t down_hyst;
	} profile[PP_SMC_POWER_PROFILE_COUNT];
	bool custom_profile;	/* userspace asked for CUSTOM, keep it */

	/* pp_dpm_* level mask while in manual mode, 0 = all levels */
	uint32_t manual_level_mask;
	/* TDP OD, percent (legacy Oland OD TDP control) */
	uint32_t od_tdp;
	/* display config snapshot for check_smc_update_required_for_display_configuration */
	uint32_t display_num_display;
	uint32_t display_crtc_index;
	uint32_t display_line_time_in_us;
	bool vce_enabled;
	bool sclk_dpm_key_disabled;
	bool mclk_dpm_key_disabled;
	bool pcie_dpm_key_disabled;

	/* copy of hwmgr->ps taken before the first adjust, the source every
	 * request is re-seeded from (si_reseed_request_state)
	 */
	void *pristine_ps;

	/* GPU hang snapshot taken before the reset (si_dump_state), printed
	 * into the devcoredump by si_print_state
	 */
	struct {
		bool valid;
		struct si_state_snapshot snap;
	} hang;
};

int si_init_function_pointers(struct pp_hwmgr *hwmgr);
int si_initialize_powertune_defaults(struct pp_hwmgr *hwmgr);

/* shared with si_smumgr.c */
enum si_pcie_gen si_gen_pcie_gen_support(uint32_t sys_mask,
					 enum si_pcie_gen asic_gen,
					 enum si_pcie_gen default_gen);


#endif
