/*
 * arch/arm/mach-tegra/tegra_cl_dvfs.h
 *
 * Copyright (C) 2012 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _TEGRA_CL_DVFS_H_
#define _TEGRA_CL_DVFS_H_

#define MAX_CL_DVFS_VOLTAGES		33

enum tegra_cl_dvfs_force_mode {
	TEGRA_CL_DVFS_FORCE_NONE = 0,
	TEGRA_CL_DVFS_FORCE_FIXED = 1,
	TEGRA_CL_DVFS_FORCE_AUTO = 2,
};

enum tegra_cl_dvfs_pmu_if {
	TEGRA_CL_DVFS_PMU_I2C,
	TEGRA_CL_DVFS_PMU_PWM,
};

struct tegra_cl_dvfs_cfg_param {
	unsigned long	sample_rate;

	enum tegra_cl_dvfs_force_mode force_mode;
	u8		cf;
	u8		ci;
	u8		cg;

	u8		droop_cut_value;
	u8		droop_restore_ramp;
	u8		scale_out_ramp;
};

struct voltage_reg_map {
	u8		reg_value;
	int		reg_uV;
};

struct tegra_cl_dvfs_platform_data {
	enum tegra_cl_dvfs_pmu_if pmu_if;

	union {
		struct {
			unsigned long		fs_rate;
			unsigned long		hs_rate;
			u8			hs_master_code;
			u8			reg;
			u16			slave_addr;
			bool			addr_10;
		} pmu_i2c;
		struct {
			/* FIXME: to be defined */
		} pmu_pwm;
	} u;

	struct voltage_reg_map	*vdd_map;
	int			vdd_map_size;

	struct tegra_cl_dvfs_cfg_param		*cfg_param;
};

struct tegra_cl_dvfs_soc_data {
	int		speedo_id;
	u32		tune0;
	u32		tune1;
	unsigned long	droop_cpu_rate_min;
};

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
void tegra_cl_dvfs_set_plarform_data(struct tegra_cl_dvfs_platform_data *data);
void tegra_cl_dvfs_set_soc_data(struct tegra_cl_dvfs_soc_data *data);
#else
static inline void tegra_cl_dvfs_set_plarform_data(
		struct tegra_cl_dvfs_platform_data *data)
{}
static inline void tegra_cl_dvfs_set_soc_data(
		struct tegra_cl_dvfs_soc_data *data)
{}
#endif

#endif
