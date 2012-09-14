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

#include "dvfs.h"

#define MAX_CL_DVFS_VOLTAGES		33

enum tegra_cl_dvfs_ctrl_mode {
	TEGRA_CL_DVFS_UNINITIALIZED = 0,
	TEGRA_CL_DVFS_DISABLED = 1,
	TEGRA_CL_DVFS_OPEN_LOOP = 2,
	TEGRA_CL_DVFS_CLOSED_LOOP = 3,
};

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
	s8		cg;
	bool		cg_scale;

	u8		droop_cut_value;
	u8		droop_restore_ramp;
	u8		scale_out_ramp;
};

struct voltage_reg_map {
	u8		reg_value;
	int		reg_uV;
};

struct tegra_cl_dvfs_platform_data {
	const char *dfll_clk_name;
	enum tegra_cl_dvfs_pmu_if pmu_if;

	union {
		struct {
			unsigned long		fs_rate;
			unsigned long		hs_rate; /* if 0 - no hs mode */
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

struct tegra_cl_dvfs_dfll_data {
	const char	*dfll_clk_name;
	u32		tune0;
	u32		tune1;
	unsigned long	droop_rate_min;
	unsigned long	out_rate_min;
	int		millivolts_min;
};

struct dfll_rate_req {
	u8	freq;
	u8	scale;
	u8	output;
};

struct tegra_cl_dvfs {
	u32					cl_base;
	struct tegra_cl_dvfs_dfll_data		*dfll_data;
	struct tegra_cl_dvfs_platform_data	*p_data;

	struct dvfs			*safe_dvfs;
	struct clk			*soc_clk;
	struct clk			*ref_clk;
	struct clk			*i2c_clk;
	struct clk			*i2c_fast;
	unsigned long			ref_rate;
	unsigned long			i2c_rate;

	/* output voltage mapping:
	 * legacy dvfs table index -to- cl_dvfs output LUT index
	 * cl_dvfs output LUT index -to- PMU value/voltage pair ptr
	 */
	u8				clk_dvfs_map[MAX_DVFS_FREQS];
	struct voltage_reg_map		*out_map[MAX_CL_DVFS_VOLTAGES];
	u8				num_voltages;
	u8				safe_ouput;

	struct dfll_rate_req		last_req;
	unsigned long			dfll_rate_min;
	enum tegra_cl_dvfs_ctrl_mode	mode;
};

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
void tegra_cl_dvfs_set_platform_data(struct tegra_cl_dvfs_platform_data *data);
void tegra_cl_dvfs_set_dfll_data(struct tegra_cl_dvfs_dfll_data *data);
int tegra_init_cl_dvfs(struct tegra_cl_dvfs *cld);

void tegra_cl_dvfs_disable(struct tegra_cl_dvfs *cld);
int tegra_cl_dvfs_enable(struct tegra_cl_dvfs *cld);
int tegra_cl_dvfs_lock(struct tegra_cl_dvfs *cld);
int tegra_cl_dvfs_unlock(struct tegra_cl_dvfs *cld);
int tegra_cl_dvfs_request_rate(struct tegra_cl_dvfs *cld, unsigned long rate);
unsigned long tegra_cl_dvfs_request_get(struct tegra_cl_dvfs *cld);
#else
static inline void tegra_cl_dvfs_set_platform_data(
		struct tegra_cl_dvfs_platform_data *data)
{}
static inline void tegra_cl_dvfs_set_dfll_data(
		struct tegra_cl_dvfs_dfll_data *data)
{}
static inline int tegra_init_cl_dvfs(struct tegra_cl_dvfs *cld)
{ return -ENOSYS; }

static inline void tegra_cl_dvfs_disable(struct tegra_cl_dvfs *cld)
{}
static inline int tegra_cl_dvfs_enable(struct tegra_cl_dvfs *cld)
{ return -ENOSYS; }
static inline int tegra_cl_dvfs_lock(struct tegra_cl_dvfs *cld)
{ return -ENOSYS; }
static inline int tegra_cl_dvfs_unlock(struct tegra_cl_dvfs *cld)
{ return -ENOSYS; }
static inline int tegra_cl_dvfs_request_rate(
	struct tegra_cl_dvfs *cld, unsigned long rate)
{ return -ENOSYS; }
static inline unsigned long tegra_cl_dvfs_request_get(struct tegra_cl_dvfs *cld)
{ return 0; }
#endif

#endif
