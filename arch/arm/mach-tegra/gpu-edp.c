/*
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/edp.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra_ppm.h>

#include <mach/edp.h>

#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/common.h>

struct gpu_edp_platform_data {
	char *name;
	char *cdev_name;
	char *cap_clk_name;
	char *clk_name;
	int freq_step;
	int imax;
};

struct gpu_edp {
	struct gpu_edp_platform_data *plat_data;

	struct tegra_ppm *ppm;
	struct mutex edp_lock;
	struct clk *cap_clk;

	int temperature_now;
};

static struct gpu_edp_platform_data s_plat_data = {
	.cdev_name = "gpu_edp",
	.cap_clk_name = "edp.gbus",
	.clk_name = "gbus",
	.name = "gpu_edp",
	.freq_step = 12000000,
};

static struct gpu_edp s_gpu = {
	.plat_data = &s_plat_data,
};

#define C_TO_K(c) (c+273)
#define K_TO_C(k) (k-273)
#define MELT_SILICON_K 1687 /* melting point of silicon */

static void _get_trip_info(int *temps, int n_temps,
			   char *cdev_name,
			   struct thermal_trip_info *trips,
			   int *num_trips, int margin)
{
	struct thermal_trip_info *trip_state;
	int i;

	if (!trips || !num_trips)
		return;

	if (n_temps > MAX_THROT_TABLE_SIZE)
		BUG();

	for (i = 0; i < n_temps-1; i++) {
		trip_state = &trips[*num_trips];

		trip_state->cdev_type = cdev_name;
		trip_state->trip_temp =
			(temps[i] * 1000) - margin;
		trip_state->trip_type = THERMAL_TRIP_ACTIVE;
		trip_state->upper = trip_state->lower =
			C_TO_K(temps[i + 1]);

		(*num_trips)++;

		if (*num_trips >= THERMAL_MAX_TRIPS)
			BUG();
	}
}

/* this will be dead code once we use DT for thermals */
void tegra_platform_gpu_edp_init(struct thermal_trip_info *trips,
				int *num_trips, int margin)
{
	int gpu_temps[] = { /* degree celcius (C) */
		20, 50, 70, 75, 80, 85, 90, 95, 100, 105,
	};

	_get_trip_info(gpu_temps, ARRAY_SIZE(gpu_temps),
		       s_gpu.plat_data->cdev_name,
		       trips, num_trips, margin);
}

static int edp_get_cdev_max_state(struct thermal_cooling_device *cdev,
				       unsigned long *max_state)
{
	*max_state = MELT_SILICON_K;
	return 0;
}

static int edp_get_cdev_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long *cur_state)
{
	struct gpu_edp *ctx = cdev->devdata;
	*cur_state = C_TO_K(ctx->temperature_now);
	return 0;
}

static int edp_set_cdev_state(struct thermal_cooling_device *cdev,
				   unsigned long cur_state)
{
	unsigned clk_rate;
	struct gpu_edp *ctx = cdev->devdata;

	if (cur_state == 0)
		return 0;

	mutex_lock(&ctx->edp_lock);

	BUG_ON(cur_state >= MELT_SILICON_K);

	ctx->temperature_now = K_TO_C(cur_state);
	clk_rate = tegra_ppm_get_maxf(ctx->ppm, ctx->plat_data->imax,
				      TEGRA_PPM_UNITS_MILLIAMPS,
				      ctx->temperature_now, 1);

	clk_set_rate(ctx->cap_clk, clk_rate * 1000);

	mutex_unlock(&ctx->edp_lock);

	return 0;
}

/* cooling devices using these ops interpret the "cur_state" as
 * a temperature in Kelvin
 */
static struct thermal_cooling_device_ops edp_cooling_ops = {
	.get_max_state = edp_get_cdev_max_state,
	.get_cur_state = edp_get_cdev_cur_state,
	.set_cur_state = edp_set_cdev_state,
};

static int __init cdev_register(struct gpu_edp *ctx)
{
	struct thermal_cooling_device *edp_cdev;

	edp_cdev = thermal_cooling_device_register(ctx->plat_data->cdev_name,
						   ctx,
						   &edp_cooling_ops);
	if (IS_ERR_OR_NULL(edp_cdev))
		pr_err("Failed to register '%s' cooling device\n",
			ctx->plat_data->cdev_name);

	return PTR_RET(edp_cdev);
}

/* TODO: take this data from DT */
/* NOTE: called by an arch_initcall well before module_initcalls */
void __init tegra_init_gpu_edp_limits(unsigned int regulator_ma)
{
	s_gpu.plat_data->imax = regulator_ma;
}

static int __init tegra_gpu_edp_init(void)
{
	static u32 tegra_chip_id;
	struct clk *gpu_clk;
	struct fv_relation *fv;
	struct tegra_ppm_params *params;
	unsigned iddq_ma;

	tegra_chip_id = tegra_get_chip_id();

	if (tegra_chip_id == TEGRA_CHIPID_TEGRA12)
		params = tegra12x_get_gpu_powermodel_params();
	else if (tegra_chip_id == TEGRA_CHIPID_TEGRA13)
		params = tegra13x_get_gpu_powermodel_params();
	else {
		WARN(true,
		     "Failed GPU EDP mgmt init. No power model available\n");
		return -ENODATA;
	}

	gpu_clk = tegra_get_clock_by_name(s_gpu.plat_data->clk_name);
	fv = fv_relation_create(gpu_clk, s_gpu.plat_data->freq_step, 150,
				      tegra_dvfs_predict_peak_millivolts);
	if (WARN(IS_ERR_OR_NULL(fv),
		 "Failed GPU EDP mgmt init. freq/volt data unavailable\n"))
		return PTR_ERR(fv);

	iddq_ma = tegra_get_gpu_iddq_value();
	pr_debug("%s: %s IDDQ value %d\n",
		 __func__, s_gpu.plat_data->name, iddq_ma);


	s_gpu.cap_clk = tegra_get_clock_by_name(s_gpu.plat_data->cap_clk_name);
	if (!s_gpu.cap_clk)
		pr_err("%s: cannot get clock:%s\n",
		       __func__, s_gpu.plat_data->cap_clk_name);

	s_gpu.ppm = tegra_ppm_create(s_gpu.plat_data->name,
				     fv, params, iddq_ma, NULL);

	if (WARN(IS_ERR_OR_NULL(s_gpu.ppm),
		 "Failed GPU EDP mgmt init. Couldn't create power model\n"))
		return PTR_ERR(s_gpu.ppm);

	s_gpu.temperature_now = 25; /* HACK */

	mutex_init(&s_gpu.edp_lock);
	return cdev_register(&s_gpu);
}
module_init(tegra_gpu_edp_init);

