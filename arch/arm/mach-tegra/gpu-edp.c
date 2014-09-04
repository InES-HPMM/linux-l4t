/*
 * arch/arm/mach-tegra/edp.c
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
#include <linux/sysedp.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-fuse.h>

#include <mach/edp.h>

#include "dvfs.h"
#include "clock.h"
#include "common.h"

#define OVERRIDE_DEFAULT 6000

#define GPU_FREQ_STEP 12000000

#define IS_T12X		(tegra_chip_id == TEGRA_CHIPID_TEGRA12)
#define IS_T13X		(tegra_chip_id == TEGRA_CHIPID_TEGRA13)
#define IS_T1XX		(IS_T12X || IS_T13X)

static u32 tegra_chip_id = 0xdeadbeef;

static struct tegra_edp_gpu_limits *gpu_edp_limits;
static int gpu_edp_limits_size;
static int gpu_edp_thermal_idx;
static struct clk *gpu_cap_clk;
static DEFINE_MUTEX(gpu_edp_lock);

static struct tegra_edp_freq_voltage_table *freq_voltage_gpu_lut;
static unsigned int freq_voltage_gpu_lut_size;

static unsigned int gpu_edp_regulator_cur;
/* Value to subtract from regulator current limit */
static unsigned int gpu_edp_reg_override_ma = OVERRIDE_DEFAULT;

static struct tegra_edp_gpu_limits gpu_edp_default_limits[] = {
	{85, 350000},
};

static const int gpu_temperatures[] = { /* degree celcius (C) */
	20, 50, 70, 75, 80, 85, 90, 95, 100, 105,
};

static inline s64 edp_pow(s64 val, int pwr)
{
	s64 retval = 1;

	while (val && pwr) {
		if (pwr & 1)
			retval *= val;
		pwr >>= 1;
		if (pwr)
			val *= val;
	}

	return retval;
}

static s64 common_edp_calculate_leakage_calc_step(
			struct tegra_edp_common_powermodel_params *common,
			int iddq_ma, int temp_c,
			unsigned int voltage_mv, int i, int j, int k)
{
	s64 leakage_calc_step;

	/* iddq raised to i */
	leakage_calc_step = common->leakage_consts_ijk[i][j][k] *
						edp_pow(iddq_ma, i);

	/* Convert (mA)^i to (A)^i */
	leakage_calc_step = div64_s64(leakage_calc_step, edp_pow(1000, i));

	/* voltage raised to j */
	leakage_calc_step *= edp_pow(voltage_mv, j);

	/* Convert (mV)^j to (V)^j */
	leakage_calc_step = div64_s64(leakage_calc_step, edp_pow(1000, j));

	/* temp raised to k */
	leakage_calc_step *= edp_pow(temp_c, k);

	/* Convert (C)^k to (scaled_C)^k */
	leakage_calc_step = div64_s64(leakage_calc_step,
					edp_pow(common->temp_scaled, k));

	/* leakage_consts_ijk was scaled */
	leakage_calc_step = div64_s64(leakage_calc_step,
					common->ijk_scaled);

	return leakage_calc_step;
}

static s64 common_edp_calculate_leakage_ma(
			struct tegra_edp_common_powermodel_params *common,
			int iddq_ma, int temp_c,
			unsigned int voltage_mv, int n_cores_idx)
{
	int i, j, k;
	s64 leakage_ma = 0;

	for (i = 0; i <= 3; i++)
		for (j = 0; j <= 3; j++)
			for (k = 0; k <= 3; k++)
				leakage_ma +=
					common_edp_calculate_leakage_calc_step(
						common, iddq_ma,
						temp_c, voltage_mv, i, j, k);

	leakage_ma *= common->leakage_consts_n[n_cores_idx];

	/* leakage_const_n was scaled */
	leakage_ma = div64_s64(leakage_ma, common->consts_scaled);

	/* set floor for leakage current */
	if (leakage_ma <= common->leakage_min)
		leakage_ma = common->leakage_min;

	return leakage_ma;
}

static s64 common_edp_calculate_dynamic_ma(
			struct tegra_edp_common_powermodel_params *common,
			unsigned int voltage_mv, int n_cores_idx,
			unsigned int freq_khz)
{
	s64 dyn_ma;

	/* Convert freq to MHz */
	dyn_ma = voltage_mv * freq_khz / 1000;

	/* Convert mV to V */
	dyn_ma = div64_s64(dyn_ma, 1000);
	dyn_ma *= common->dyn_consts_n[n_cores_idx];

	/* dyn_const_n was scaled */
	dyn_ma = div64_s64(dyn_ma, common->dyn_scaled);

	return dyn_ma;
}

static int edp_relate_freq_voltage(struct clk *c, unsigned int step,
			unsigned int freq_volt_lut_size,
			struct tegra_edp_freq_voltage_table *freq_volt_lut)
{
	unsigned int i, j, freq;
	int voltage_mv;

	for (i = 0, j = 0, freq = 0;
		 i < freq_volt_lut_size;
		 i++, freq += step) {

		/* Predict voltages */
		voltage_mv = tegra_dvfs_predict_peak_millivolts(c, freq);
		if (voltage_mv < 0) {
			pr_err("%s: couldn't predict voltage: freq %u; err %d",
			       __func__, freq, voltage_mv);
			return -EINVAL;
		}

		/* Cache frequency / voltage / voltage constant relationship */
		freq_volt_lut[i].freq = freq;
		freq_volt_lut[i].voltage_mv = voltage_mv;
	}
	return 0;
}

void tegra_platform_gpu_edp_init(struct thermal_trip_info *trips,
				int *num_trips, int margin)
{
	struct thermal_trip_info *trip_state;
	int i;

	if (!trips || !num_trips)
		return;

	if (gpu_edp_limits_size > MAX_THROT_TABLE_SIZE)
		BUG();

	for (i = 0; i < gpu_edp_limits_size-1; i++) {
		trip_state = &trips[*num_trips];

		trip_state->cdev_type = "gpu_edp";
		trip_state->trip_temp =
			(gpu_edp_limits[i].temperature * 1000) - margin;
		trip_state->trip_type = THERMAL_TRIP_ACTIVE;
		trip_state->upper = trip_state->lower = i + 1;

		(*num_trips)++;

		if (*num_trips >= THERMAL_MAX_TRIPS)
			BUG();
	}
}

static unsigned int gpu_edp_calculate_maxf(
				struct tegra_edp_gpu_powermodel_params *params,
				int temp_c, int gpu_iddq_ma)
{
	unsigned int voltage_mv, freq_khz = 0;
	unsigned int cur_effective = gpu_edp_regulator_cur -
				     gpu_edp_reg_override_ma;
	int f;
	s64 leakage_ma = 0, dyn_ma;

	for (f = freq_voltage_gpu_lut_size - 1; f >= 0; f--) {
		freq_khz = freq_voltage_gpu_lut[f].freq / 1000;
		voltage_mv = freq_voltage_gpu_lut[f].voltage_mv;

		/* Calculate leakage current */
		leakage_ma = common_edp_calculate_leakage_ma(
					&params->common, gpu_iddq_ma,
					temp_c, voltage_mv, 0);

		/* Calculate dynamic current */
		dyn_ma = common_edp_calculate_dynamic_ma(&params->common,
					voltage_mv, 0, freq_khz);

		if ((leakage_ma + dyn_ma) <= cur_effective)
			goto end;

		freq_khz = 0;
	}

 end:
	return freq_khz;
}

static int __init start_gpu_edp(void)
{
	const char *cap_name = "edp.gbus";

	gpu_cap_clk = tegra_get_clock_by_name(cap_name);
	if (!gpu_cap_clk) {
		pr_err("gpu_edp_set_cdev_state: cannot get clock:%s\n",
				cap_name);
		return -EINVAL;
	}
	gpu_edp_thermal_idx = 0;

	return 0;
}


static int init_gpu_edp_limits_calculated(void)
{
	unsigned int gpu_minf, gpu_maxf;
	unsigned int limit;
	struct tegra_edp_gpu_limits *gpu_edp_calculated_limits;
	struct tegra_edp_gpu_limits *temp;
	struct tegra_edp_gpu_powermodel_params *params;
	int i, ret;
	unsigned int gpu_iddq_ma;
	struct clk *gpu_clk = clk_get_parent(gpu_cap_clk);

	if (IS_T12X)
		params = tegra12x_get_gpu_powermodel_params();
	else if (IS_T13X)
		params = tegra13x_get_gpu_powermodel_params();
	else
		return -EINVAL;

	gpu_iddq_ma = tegra_get_gpu_iddq_value();
	pr_debug("%s: GPU IDDQ value %d\n", __func__, gpu_iddq_ma);
	gpu_edp_calculated_limits = kmalloc(sizeof(struct tegra_edp_gpu_limits)
				* ARRAY_SIZE(gpu_temperatures), GFP_KERNEL);
	BUG_ON(!gpu_edp_calculated_limits);

	gpu_minf = 0;
	gpu_maxf = clk_get_max_rate(gpu_clk);

	freq_voltage_gpu_lut_size = (gpu_maxf - gpu_minf) / GPU_FREQ_STEP + 1;
	freq_voltage_gpu_lut = kmalloc(
		sizeof(struct tegra_edp_freq_voltage_table)
				   * freq_voltage_gpu_lut_size, GFP_KERNEL);
	if (!freq_voltage_gpu_lut) {
		pr_err("%s: failed alloc mem for gpu freq/voltage LUT\n",
			 __func__);
		kfree(gpu_edp_calculated_limits);
		return -ENOMEM;
	}

	ret = edp_relate_freq_voltage(gpu_clk, GPU_FREQ_STEP,
			freq_voltage_gpu_lut_size, freq_voltage_gpu_lut);

	if (ret) {
		kfree(gpu_edp_calculated_limits);
		kfree(freq_voltage_gpu_lut);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(gpu_temperatures); i++) {
		gpu_edp_calculated_limits[i].temperature =
			gpu_temperatures[i];
		limit = gpu_edp_calculate_maxf(params,
					       gpu_temperatures[i],
					       gpu_iddq_ma);
		if (limit == -EINVAL) {
			kfree(gpu_edp_calculated_limits);
			kfree(freq_voltage_gpu_lut);
			return -EINVAL;
		}

		gpu_edp_calculated_limits[i].freq_limits = limit;
	}

	/*
	 * If this is an EDP table update, need to overwrite old table.
	 * The old table's address must remain valid.
	 */
	if (gpu_edp_limits != gpu_edp_default_limits &&
			gpu_edp_limits != gpu_edp_calculated_limits) {
		temp = gpu_edp_limits;
		gpu_edp_limits = gpu_edp_calculated_limits;
		gpu_edp_limits_size = ARRAY_SIZE(gpu_temperatures);
		kfree(temp);
	} else {
		gpu_edp_limits = gpu_edp_calculated_limits;
		gpu_edp_limits_size = ARRAY_SIZE(gpu_temperatures);
	}

	kfree(freq_voltage_gpu_lut);

	return 0;
}

void __init tegra_init_gpu_edp_limits(unsigned int regulator_ma)
{
	if (tegra_chip_id == 0xdeadbeef)
		tegra_chip_id = tegra_get_chip_id();

	if (!(IS_T12X || IS_T13X))
		return;

	if (!regulator_ma) {
		gpu_edp_limits = gpu_edp_default_limits;
		gpu_edp_limits_size = ARRAY_SIZE(gpu_edp_default_limits);
		return;
	}

	if (start_gpu_edp()) {
		WARN(1, "GPU EDP failed to set initial limits");
		return;
	}

	gpu_edp_regulator_cur = regulator_ma + OVERRIDE_DEFAULT;
	init_gpu_edp_limits_calculated();
}

static int gpu_edp_get_cdev_max_state(struct thermal_cooling_device *cdev,
				       unsigned long *max_state)
{
	*max_state = gpu_edp_limits_size - 1;
	return 0;
}

static int gpu_edp_get_cdev_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long *cur_state)
{
	*cur_state = gpu_edp_thermal_idx;
	return 0;
}

static int gpu_edp_set_cdev_state(struct thermal_cooling_device *cdev,
				   unsigned long cur_state)
{
	unsigned long clk_rate;
	BUG_ON(cur_state >= gpu_edp_limits_size);
	clk_rate = gpu_edp_limits[cur_state].freq_limits;
	mutex_lock(&gpu_edp_lock);
	gpu_edp_thermal_idx = cur_state;
	clk_set_rate(gpu_cap_clk, clk_rate * 1000);
	mutex_unlock(&gpu_edp_lock);
	return 0;
}

static struct thermal_cooling_device_ops gpu_edp_cooling_ops = {
	.get_max_state = gpu_edp_get_cdev_max_state,
	.get_cur_state = gpu_edp_get_cdev_cur_state,
	.set_cur_state = gpu_edp_set_cdev_state,
};

static int __init gpu_edp_cdev_init(void)
{
	struct thermal_cooling_device *gpu_edp_cdev;

	gpu_edp_cdev = thermal_cooling_device_register("gpu_edp", NULL,
					&gpu_edp_cooling_ops);
	if (IS_ERR_OR_NULL(gpu_edp_cdev))
		pr_err("Failed to register 'gpu_edp' cooling device\n");

	return 0;
}
module_init(gpu_edp_cdev_init);

#ifdef CONFIG_DEBUG_FS

static inline void edp_show_edp_table(struct seq_file *s,
				      struct tegra_edp_limits *limits,
				      size_t nlimits, int ncores, int th_idx)
{
	int i, j;
	seq_puts(s, " Temp.");
	for (i = 1; i <= ncores; i++)
		seq_printf(s, "     %d-core", i);
	seq_puts(s, "\n");

	for (i = 0; i < nlimits; i++) {
		seq_printf(s, "%c%3dC:",
			   i == th_idx ? '>' : ' ',
			   limits[i].temperature);
		for (j = 0; j < ncores; j++)
			seq_printf(s, " %10u", limits[i].freq_limits[j]);
		seq_puts(s, "\n");
	}
}

static int gpu_edp_limit_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%u\n", gpu_edp_limits[gpu_edp_thermal_idx].freq_limits);
	return 0;
}

static inline void gpu_edp_show_table(struct seq_file *s)
{
	int i;

	seq_printf(s, "%6s %10s\n",
		   " Temp.", "Freq_limit");
	for (i = 0; i < gpu_edp_limits_size; i++) {
		seq_printf(s, "%3dC: %10u\n",
			   gpu_edp_limits[i].temperature,
			   gpu_edp_limits[i].freq_limits);
	}
}

static int gpu_edp_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "-- VDD_GPU %sEDP table (%umA = %umA - %umA) --\n",
		   gpu_edp_limits == gpu_edp_default_limits ?
		   "**default** " : "",
		   gpu_edp_regulator_cur - gpu_edp_reg_override_ma,
		   gpu_edp_regulator_cur, gpu_edp_reg_override_ma);

	gpu_edp_show_table(s);

	return 0;
}

static int gpu_edp_reg_override_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Limit override: %u mA. Effective limit: %u mA\n",
		   gpu_edp_reg_override_ma,
		   gpu_edp_regulator_cur - gpu_edp_reg_override_ma);
	return 0;
}

static ssize_t gpu_edp_reg_override_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32];
	unsigned long gpu_edp_reg_override_ma_temp;
	unsigned int gpu_edp_reg_override_ma_prev = gpu_edp_reg_override_ma;

	if (!(IS_T12X || IS_T13X))
		return -EINVAL;

	if (sizeof(buf) <= count)
		goto override_err;

	if (copy_from_user(buf, userbuf, count))
		goto override_err;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (kstrtoul(buf, 10, &gpu_edp_reg_override_ma_temp))
		goto override_err;

	if (gpu_edp_reg_override_ma_temp >= gpu_edp_regulator_cur)
		goto override_err;

	if (gpu_edp_reg_override_ma == gpu_edp_reg_override_ma_temp)
		return count;

	gpu_edp_reg_override_ma = gpu_edp_reg_override_ma_temp;
	if (init_gpu_edp_limits_calculated()) {
		/* Revert to previous override value if new value fails */
		gpu_edp_reg_override_ma = gpu_edp_reg_override_ma_prev;
		goto override_err;
	}

	gpu_edp_set_cdev_state(NULL, gpu_edp_thermal_idx);
	pr_info("Reinitialized VDD_GPU EDP table with regulator current limit %u mA\n",
		gpu_edp_regulator_cur - gpu_edp_reg_override_ma);
	return count;

 override_err:
	pr_err("FAILED: Reinitialize VDD_GPU EDP table with override \"%s\"",
	       buf);
	return -EINVAL;
}

static int gpu_edp_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpu_edp_debugfs_show, inode->i_private);
}

static int gpu_edp_limit_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpu_edp_limit_debugfs_show, inode->i_private);
}

static int gpu_edp_reg_override_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpu_edp_reg_override_show, inode->i_private);
}

static const struct file_operations gpu_edp_debugfs_fops = {
	.open		= gpu_edp_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations gpu_edp_limit_debugfs_fops = {
	.open		= gpu_edp_limit_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations gpu_edp_reg_override_debugfs_fops = {
	.open		= gpu_edp_reg_override_open,
	.read		= seq_read,
	.write		= gpu_edp_reg_override_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_gpu_edp_debugfs_init(struct dentry *edp_dir)
{
	struct dentry *vdd_gpu_dir;

	if (!tegra_platform_is_silicon())
		return -ENOSYS;

	vdd_gpu_dir = debugfs_create_dir("vdd_gpu", edp_dir);
	if (!vdd_gpu_dir)
		return -ENOMEM;

	debugfs_create_file("gpu_edp",  S_IRUGO, vdd_gpu_dir, NULL,
				    &gpu_edp_debugfs_fops);
	debugfs_create_file("gpu_edp_limit", S_IRUGO, vdd_gpu_dir,
					  NULL, &gpu_edp_limit_debugfs_fops);
	debugfs_create_file("gpu_edp_reg_override",
				S_IRUGO | S_IWUSR, vdd_gpu_dir, NULL,
				&gpu_edp_reg_override_debugfs_fops);

	return 0;
}

static int __init tegra_edp_debugfs_init(void)
{
	struct dentry *edp_dir;

	if (!tegra_platform_is_silicon())
		return -ENOSYS;

	edp_dir = debugfs_create_dir("edp2", NULL);
	if (!edp_dir)
		return -ENOMEM;

	if (tegra_gpu_edp_debugfs_init(edp_dir))
		return -ENOMEM;

	return 0;
}

late_initcall(tegra_edp_debugfs_init);
#endif /* CONFIG_DEBUG_FS */
