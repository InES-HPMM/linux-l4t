/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All Rights Reserved.
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
#include <linux/sysedp.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-fuse.h>
#include <linux/regulator/consumer.h>

#include <mach/edp.h>

#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/common.h>

struct cpu_edp_platform_data {
	char *name;
	char *clk_name;
	int freq_step;
	int reg_edp;
	int reg_idle_max;
};
struct cpu_edp {
	struct cpu_edp_platform_data *plat_data;
	struct tegra_ppm *ppm;
	unsigned int *hard_cap;
	unsigned int n_cores;
	int recent_temperature;
};

static struct cpu_edp_platform_data s_plat_data = {
	.clk_name = "cpu_g",
	.name = "cpu_edp",
	.freq_step = 12750000,
};

static struct cpu_edp s_cpu = {
	.plat_data = &s_plat_data,
	.recent_temperature = -273,
};

unsigned int tegra_get_sysedp_max_freq(int cpupwr, int temperature,
				       int online_cpus, int cpu_mode)
{
	if (WARN_ONCE(!s_cpu.ppm, "Init call orering issue!\n"))
		return 0;

	if (cpu_mode != MODE_G) {
		/* TODO: for T210, handle MODE_LP */
		return 0;
	}

	return tegra_ppm_get_maxf(s_cpu.ppm, cpupwr,
				  TEGRA_PPM_UNITS_MILLIWATTS,
				  temperature, online_cpus);
}

unsigned int tegra_get_edp_max_freq(int temperature, int online_cpus,
				    int cpu_mode)
{
	unsigned int res;

	if (WARN_ONCE(!s_cpu.ppm, "Init call orering issue!\n"))
		return 0;

	s_cpu.recent_temperature = temperature;

	if (cpu_mode != MODE_G) {
		/* TODO: for T210, handle MODE_LP */
		return 0;
	}

	res = tegra_ppm_get_maxf(s_cpu.ppm, s_cpu.plat_data->reg_edp,
				 TEGRA_PPM_UNITS_MILLIAMPS,
				 temperature, online_cpus);

	if (s_cpu.hard_cap && s_cpu.hard_cap[online_cpus - 1]
	    && res > s_cpu.hard_cap[online_cpus - 1])
		res = s_cpu.hard_cap[online_cpus - 1];

	return res;
}

unsigned int tegra_get_reg_idle_freq(int temperature, int online_cpus)
{
	unsigned int res;
	WARN_ON(!s_cpu.ppm);

	/* TODO: for T210, handle MODE_LP */
	res = tegra_ppm_get_maxf(s_cpu.ppm, s_cpu.plat_data->reg_idle_max,
				 TEGRA_PPM_UNITS_MILLIAMPS,
				 temperature, online_cpus);

	return res;

}

bool tegra_is_edp_reg_idle_supported(void)
{
	return (s_cpu.plat_data->reg_idle_max != 0);
}

bool tegra_is_cpu_edp_supported(void)
{
	return s_cpu.plat_data->reg_edp != 0;
}

void tegra_recalculate_cpu_edp_limits(void)
{
	if (!tegra_is_cpu_edp_supported())
		return;

	tegra_ppm_drop_cache(s_cpu.ppm);
}

#ifdef CONFIG_DEBUG_FS
static int __init tegra_edp_debugfs_init(struct dentry *edp_dir)
{
	if (!tegra_platform_is_silicon())
		return -ENOSYS;

	if (!edp_dir)
		return -ENOMEM;

	debugfs_create_u32("reg_idle_ma", S_IRUGO | S_IWUSR,
			   edp_dir, &s_cpu.plat_data->reg_idle_max);
	debugfs_create_u32("reg_edp_ma", S_IRUGO | S_IWUSR,
			   edp_dir, &s_cpu.plat_data->reg_edp);
	debugfs_create_u32("temperature", S_IRUGO,
			   edp_dir, &s_cpu.recent_temperature);
	debugfs_create_u32_array("hard_cap", S_IRUGO,
			   edp_dir, s_cpu.hard_cap, s_cpu.n_cores);

	return 0;
}
#else
static int __init tegra_edp_debugfs_init(void)
{ return 0; }
#endif /* CONFIG_DEBUG_FS */

/* TODO: take this data from DT */
/* NOTE: called by an arch_initcall well before module_initcalls */
void __init tegra_init_cpu_edp_limits(unsigned int regulator_ma)
{
	s_cpu.plat_data->reg_edp = regulator_ma;
}

void __init tegra_init_cpu_reg_mode_limits(unsigned int regulator_ma,
					   unsigned int mode)
{
	if (mode == REGULATOR_MODE_IDLE) {
		s_cpu.plat_data->reg_idle_max = regulator_ma;
		return;
	}
	pr_err("%s: Not supported regulator mode 0x%x\n", __func__, mode);
}

/* TODO: eliminate this entirely */
static int edp_find_speedo_idx(int cpu_speedo_id, unsigned int *cpu_speedo_idx)
{
	int i, array_size;
	struct tegra_edp_cpu_powermodel_params *params = NULL;
	u32 tegra_chip_id;

	params = NULL;

	tegra_chip_id = tegra_get_chip_id();

	if (tegra_chip_id == TEGRA_CHIPID_TEGRA12)
		params = tegra12x_get_cpu_powermodel_params(0, &array_size);
	else if (tegra_chip_id == TEGRA_CHIPID_TEGRA13) {
		/* pick edp_params array-index based on package */
		*cpu_speedo_idx = (tegra_package_id() == 1);
		return 0;
	} else
		array_size = 0;

	for (i = 0; i < array_size; i++)
		if (cpu_speedo_id == params[i].cpu_speedo_id) {
			*cpu_speedo_idx = i;
			return 0;
		}

	*cpu_speedo_idx = 0;
	pr_err("%s: couldn't find cpu speedo id %d in freq/voltage LUT\n",
	       __func__, cpu_speedo_id);
	return -EINVAL;
}

static __init int tegra_cpu_edp_init(void)
{
	unsigned int cpu_iddq_ma;
	unsigned int cpu_speedo_idx;
	u32 tegra_chip_id;
	struct fv_relation *fv;
	struct tegra_edp_cpu_powermodel_params *params;
	int ret;
	struct clk *clk_cpu_g =
		tegra_get_clock_by_name(s_cpu.plat_data->clk_name);
	int cpu_speedo_id = tegra_cpu_speedo_id();

	struct dentry *edp_dir;

	cpu_iddq_ma = tegra_get_cpu_iddq_value();
	pr_debug("%s: CPU IDDQ value %d\n", __func__, cpu_iddq_ma);

	ret = edp_find_speedo_idx(cpu_speedo_id, &cpu_speedo_idx);
	BUG_ON(ret);

	tegra_chip_id = tegra_get_chip_id();

	params =
		(tegra_chip_id == TEGRA_CHIPID_TEGRA12) ?
		tegra12x_get_cpu_powermodel_params(cpu_speedo_idx, NULL) :
		(tegra_chip_id == TEGRA_CHIPID_TEGRA13) ?
		tegra13x_get_cpu_powermodel_params(cpu_speedo_idx, NULL) :
		NULL;
	BUG_ON(!params);

	s_cpu.hard_cap = params->safety_cap;
	s_cpu.n_cores = params->common.n_cores;

	fv = fv_relation_create(clk_cpu_g,
				s_cpu.plat_data->freq_step, 220,
				tegra_dvfs_predict_peak_millivolts);

	edp_dir = debugfs_create_dir(s_cpu.plat_data->name, NULL);

	s_cpu.ppm = tegra_ppm_create(s_cpu.plat_data->name,
				     fv, &params->common, cpu_iddq_ma, edp_dir);
	BUG_ON(!s_cpu.ppm);

	tegra_edp_debugfs_init(edp_dir);
	return 0;
}
module_init(tegra_cpu_edp_init);

