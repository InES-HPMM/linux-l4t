/*
 * arch/arm/mach-tegra/edp.c
 *
 * Copyright (c) 2011-2013, NVIDIA CORPORATION. All Rights Reserved.
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

#include <mach/edp.h>
#include <mach/hardware.h>

#include "fuse.h"
#include "dvfs.h"
#include "clock.h"
#include "cpu-tegra.h"

#define FREQ_STEP 12750000
#define OVERRIDE_DEFAULT 6000

static struct tegra_edp_limits *edp_limits;
static int edp_limits_size;
static unsigned int regulator_cur;
/* Value to subtract from regulator current limit */
static unsigned int edp_reg_override_mA = OVERRIDE_DEFAULT;

static const unsigned int *system_edp_limits;

static struct tegra_system_edp_entry *power_edp_limits;
static int power_edp_limits_size;

/*
 * "Safe entry" to be used when no match for speedo_id /
 * regulator_cur is found; must be the last one
 */
static struct tegra_edp_limits edp_default_limits[] = {
	{85, {1000000, 1000000, 1000000, 1000000} },
};

static struct tegra_system_edp_entry power_edp_default_limits[] = {
	{0, 20, {1000000, 1000000, 1000000, 1000000} },
};

/* Constants for EDP calculations */
static const int temperatures[] = { /* degree celcius (C) */
	23, 40, 50, 60, 70, 74, 78, 82, 86, 90, 94, 98, 102,
};

static const int power_cap_levels[] = { /* milliwatts (mW) */
	  500,  1000,  1500,  2000,  2500,  3000,  3500,
	 4000,  4500,  5000,  5500,  6000,  6500,  7000,
	 7500,  8000,  8500,  9000,  9500, 10000, 10500,
	11000, 11500, 12000, 12500, 13000, 13500, 14000,
	14500, 15000, 15500, 16000, 16500, 17000
};

static struct tegra_edp_freq_voltage_table *freq_voltage_lut_saved;
static unsigned int freq_voltage_lut_size_saved;
static struct tegra_edp_freq_voltage_table *freq_voltage_lut;
static unsigned int freq_voltage_lut_size;

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

/*
 * Find the maximum frequency that results in dynamic and leakage current that
 * is less than the regulator current limit.
 * temp_C - always valid
 * power_mW - valid or -1 (infinite)
 */
static unsigned int edp_calculate_maxf(
				struct tegra_edp_cpu_leakage_params *params,
				int temp_C, int power_mW,
				int iddq_mA,
				int n_cores_idx)
{
	unsigned int voltage_mV, freq_KHz;
	unsigned int cur_effective = regulator_cur - edp_reg_override_mA;
	int f, i, j, k;
	s64 leakage_mA, dyn_mA, leakage_calc_step;
	s64 leakage_mW, dyn_mW;

	for (f = freq_voltage_lut_size - 1; f >= 0; f--) {
		freq_KHz = freq_voltage_lut[f].freq / 1000;
		voltage_mV = freq_voltage_lut[f].voltage_mV;

		/* Constrain Volt-Temp */
		if (params->volt_temp_cap.temperature &&
		    temp_C > params->volt_temp_cap.temperature &&
		    params->volt_temp_cap.voltage_limit_mV &&
		    voltage_mV > params->volt_temp_cap.voltage_limit_mV)
			continue;

		/* Calculate leakage current */
		leakage_mA = 0;
		for (i = 0; i <= 3; i++) {
			for (j = 0; j <= 3; j++) {
				for (k = 0; k <= 3; k++) {
					leakage_calc_step =
						params->leakage_consts_ijk
						[i][j][k] * edp_pow(iddq_mA, i);
					/* Convert (mA)^i to (A)^i */
					leakage_calc_step =
						div64_s64(leakage_calc_step,
							  edp_pow(1000, i));
					leakage_calc_step *=
						edp_pow(voltage_mV, j);
					/* Convert (mV)^j to (V)^j */
					leakage_calc_step =
						div64_s64(leakage_calc_step,
							  edp_pow(1000, j));
					leakage_calc_step *=
						edp_pow(temp_C, k);
					/* Convert (C)^k to (scaled_C)^k */
					leakage_calc_step =
						div64_s64(leakage_calc_step,
						edp_pow(params->temp_scaled,
							k));
					/* leakage_consts_ijk was scaled */
					leakage_calc_step =
						div64_s64(leakage_calc_step,
							  params->ijk_scaled);
					leakage_mA += leakage_calc_step;
				}
			}
		}

		/* if specified, set floor for leakage current */
		if (params->leakage_min && leakage_mA <= params->leakage_min)
			leakage_mA = params->leakage_min;

		/* leakage cannot be negative => leakage model has error */
		if (leakage_mA <= 0) {
			pr_err("VDD_CPU EDP failed: IDDQ too high (%d mA)\n",
			       iddq_mA);
			return -EINVAL;
		}

		leakage_mA *= params->leakage_consts_n[n_cores_idx];

		/* leakage_const_n was scaled */
		leakage_mA = div64_s64(leakage_mA, params->consts_scaled);

		/* Calculate dynamic current */
		dyn_mA = voltage_mV * freq_KHz / 1000;
		/* Convert mV to V */
		dyn_mA = div64_s64(dyn_mA, 1000);
		dyn_mA *= params->dyn_consts_n[n_cores_idx];
		/* dyn_const_n was scaled */
		dyn_mA = div64_s64(dyn_mA, params->dyn_scaled);

		if (power_mW != -1) {
			leakage_mW = leakage_mA * voltage_mV;
			dyn_mW = dyn_mA * voltage_mV;
			if (div64_s64(leakage_mW + dyn_mW, 1000) <= power_mW)
				return freq_KHz;
		} else if ((leakage_mA + dyn_mA) <= cur_effective) {
			return freq_KHz;
		}
	}
	return 0;
}

static int edp_relate_freq_voltage(struct clk *clk_cpu_g,
			unsigned int cpu_speedo_idx,
			unsigned int freq_volt_lut_size,
			struct tegra_edp_freq_voltage_table *freq_volt_lut)
{
	unsigned int i, j, freq;
	int voltage_mV;

	for (i = 0, j = 0, freq = 0;
		 i < freq_volt_lut_size;
		 i++, freq += FREQ_STEP) {

		/* Predict voltages */
		voltage_mV = tegra_dvfs_predict_millivolts(clk_cpu_g, freq);
		if (voltage_mV < 0) {
			pr_err("%s: couldn't predict voltage: freq %u; err %d",
			       __func__, freq, voltage_mV);
			return -EINVAL;
		}

		/* Cache frequency / voltage / voltage constant relationship */
		freq_volt_lut[i].freq = freq;
		freq_volt_lut[i].voltage_mV = voltage_mV;
	}
	return 0;
}

unsigned int tegra_edp_find_maxf(int volt)
{
	unsigned int i;

	for (i = 0; i < freq_voltage_lut_size_saved; i++) {
		if (freq_voltage_lut_saved[i].voltage_mV > volt)
			break;
	}
	return freq_voltage_lut[i - 1].freq;
}


static int edp_find_speedo_idx(int cpu_speedo_id, unsigned int *cpu_speedo_idx)
{
	int i, array_size;
	struct tegra_edp_cpu_leakage_params *params;

	switch (tegra_chip_id) {
	case TEGRA_CHIPID_TEGRA11:
		params = tegra11x_get_leakage_params(0, &array_size);
		break;
	case TEGRA_CHIPID_TEGRA14:
		params = tegra14x_get_leakage_params(0, &array_size);
		break;
	case TEGRA_CHIPID_TEGRA3:
	case TEGRA_CHIPID_TEGRA2:
	default:
		array_size = 0;
		break;
	}

	for (i = 0; i < array_size; i++)
		if (cpu_speedo_id == params[i].cpu_speedo_id) {
			*cpu_speedo_idx = i;
			return 0;
		}

	pr_err("%s: couldn't find cpu speedo id %d in freq/voltage LUT\n",
	       __func__, cpu_speedo_id);
	return -EINVAL;
}

static int init_cpu_edp_limits_calculated(void)
{
	unsigned int temp_idx, n_cores_idx, pwr_idx;
	unsigned int cpu_g_minf, cpu_g_maxf;
	unsigned int iddq_mA;
	unsigned int cpu_speedo_idx;
	unsigned int cap, limit;
	struct tegra_edp_limits *edp_calculated_limits;
	struct tegra_system_edp_entry *power_edp_calc_limits;
	struct tegra_edp_cpu_leakage_params *params;
	int ret;
	struct clk *clk_cpu_g = tegra_get_clock_by_name("cpu_g");
	int cpu_speedo_id = tegra_cpu_speedo_id();

	/* Determine all inputs to EDP formula */
	iddq_mA = tegra_get_cpu_iddq_value();
	ret = edp_find_speedo_idx(cpu_speedo_id, &cpu_speedo_idx);
	if (ret)
		return ret;

	switch (tegra_chip_id) {
	case TEGRA_CHIPID_TEGRA11:
		params = tegra11x_get_leakage_params(cpu_speedo_idx, NULL);
		break;
	case TEGRA_CHIPID_TEGRA14:
		params = tegra14x_get_leakage_params(cpu_speedo_idx, NULL);
		break;
	case TEGRA_CHIPID_TEGRA3:
	case TEGRA_CHIPID_TEGRA2:
	default:
		return -EINVAL;
	}

	edp_calculated_limits = kmalloc(sizeof(struct tegra_edp_limits)
					* ARRAY_SIZE(temperatures), GFP_KERNEL);
	BUG_ON(!edp_calculated_limits);

	power_edp_calc_limits = kmalloc(sizeof(struct tegra_system_edp_entry)
				* ARRAY_SIZE(power_cap_levels), GFP_KERNEL);
	BUG_ON(!power_edp_calc_limits);

	cpu_g_minf = 0;
	cpu_g_maxf = clk_get_max_rate(clk_cpu_g);
	freq_voltage_lut_size = (cpu_g_maxf - cpu_g_minf) / FREQ_STEP + 1;
	freq_voltage_lut = kmalloc(sizeof(struct tegra_edp_freq_voltage_table)
				   * freq_voltage_lut_size, GFP_KERNEL);
	if (!freq_voltage_lut) {
		pr_err("%s: failed alloc mem for freq/voltage LUT\n", __func__);
		kfree(power_edp_calc_limits);
		kfree(edp_calculated_limits);
		return -ENOMEM;
	}

	ret = edp_relate_freq_voltage(clk_cpu_g, cpu_speedo_idx,
				freq_voltage_lut_size, freq_voltage_lut);
	if (ret) {
		kfree(power_edp_calc_limits);
		kfree(edp_calculated_limits);
		kfree(freq_voltage_lut);
		return ret;
	}

	if (freq_voltage_lut_size != freq_voltage_lut_size_saved) {
		/* release previous table if present */
		kfree(freq_voltage_lut_saved);
		/* create table to save */
		freq_voltage_lut_saved =
			kmalloc(sizeof(struct tegra_edp_freq_voltage_table) *
			freq_voltage_lut_size, GFP_KERNEL);
		if (!freq_voltage_lut_saved) {
			pr_err("%s: failed alloc mem for freq/voltage LUT\n",
				__func__);
			kfree(freq_voltage_lut);
			return -ENOMEM;
		}
		freq_voltage_lut_size_saved = freq_voltage_lut_size;
	}
	memcpy(freq_voltage_lut_saved,
		freq_voltage_lut,
		sizeof(struct tegra_edp_freq_voltage_table) *
			freq_voltage_lut_size);

	/* Calculate EDP table */
	for (n_cores_idx = 0; n_cores_idx < NR_CPUS; n_cores_idx++) {
		for (temp_idx = 0;
		     temp_idx < ARRAY_SIZE(temperatures); temp_idx++) {
			edp_calculated_limits[temp_idx].temperature =
				temperatures[temp_idx];
			limit = edp_calculate_maxf(params,
						   temperatures[temp_idx],
						   -1,
						   iddq_mA,
						   n_cores_idx);
			/* apply safety cap if it is specified */
			if (n_cores_idx < 4) {
				cap = params->safety_cap[n_cores_idx];
				if (cap && cap < limit)
					limit = cap;
			}
			edp_calculated_limits[temp_idx].
				freq_limits[n_cores_idx] = limit;
		}

		for (pwr_idx = 0;
		     pwr_idx < ARRAY_SIZE(power_cap_levels); pwr_idx++) {
			power_edp_calc_limits[pwr_idx].power_limit_100mW =
				power_cap_levels[pwr_idx] / 100;
			limit = edp_calculate_maxf(params,
						   90,
						   power_cap_levels[pwr_idx],
						   iddq_mA,
						   n_cores_idx);
			power_edp_calc_limits[pwr_idx].
				freq_limits[n_cores_idx] = limit;
		}
	}

	/*
	 * If this is an EDP table update, need to overwrite old table.
	 * The old table's address must remain valid.
	 */
	if (edp_limits != edp_default_limits) {
		memcpy(edp_limits, edp_calculated_limits,
		       sizeof(struct tegra_edp_limits)
		       * ARRAY_SIZE(temperatures));
		kfree(edp_calculated_limits);
	}
	else {
		edp_limits = edp_calculated_limits;
		edp_limits_size = ARRAY_SIZE(temperatures);
	}

	if (power_edp_limits != power_edp_default_limits) {
		memcpy(power_edp_limits, power_edp_calc_limits,
		       sizeof(struct tegra_system_edp_entry)
		       * ARRAY_SIZE(power_cap_levels));
		kfree(power_edp_calc_limits);
	} else {
		power_edp_limits = power_edp_calc_limits;
		power_edp_limits_size = ARRAY_SIZE(power_cap_levels);
	}

	kfree(freq_voltage_lut);
	return 0;
}

static int __init init_cpu_edp_limits_lookup(void)
{
	int i, j;
	struct tegra_edp_limits *e;
	struct tegra_edp_vdd_cpu_entry *t;
	int tsize = 0;
	int cpu_speedo_id = tegra_cpu_speedo_id();

	if (tegra_chip_id != TEGRA_CHIPID_TEGRA3)
		return -EINVAL;

	t = tegra3x_get_vdd_cpu_map(&tsize);

	for (i = 0; i < tsize; i++) {
		if (t[i].speedo_id == cpu_speedo_id &&
		    t[i].regulator_100mA <= regulator_cur / 100)
			break;
	}

	/* No entry found in tegra_edp_vdd_cpu_map */
	if (i >= tsize)
		return -EINVAL;

	/* Find all rows for this entry */
	for (j = i + 1; j < tsize; j++) {
		if (t[i].speedo_id != t[j].speedo_id ||
		    t[i].regulator_100mA != t[j].regulator_100mA)
			break;
	}

	edp_limits_size = j - i;
	e = kmalloc(sizeof(struct tegra_edp_limits) * edp_limits_size,
		    GFP_KERNEL);
	BUG_ON(!e);

	for (j = 0; j < edp_limits_size; j++) {
		e[j].temperature = (int)t[i+j].temperature;
		e[j].freq_limits[0] = (unsigned int)t[i+j].freq_limits[0]*10000;
		e[j].freq_limits[1] = (unsigned int)t[i+j].freq_limits[1]*10000;
		e[j].freq_limits[2] = (unsigned int)t[i+j].freq_limits[2]*10000;
		e[j].freq_limits[3] = (unsigned int)t[i+j].freq_limits[3]*10000;
	}

	if (edp_limits != edp_default_limits)
		kfree(edp_limits);

	edp_limits = e;
	return 0;
}

void tegra_recalculate_cpu_edp_limits(void)
{
	if (tegra_chip_id == TEGRA_CHIPID_TEGRA11 ||
	    tegra_chip_id == TEGRA_CHIPID_TEGRA14)
		init_cpu_edp_limits_calculated();
}

/*
 * Specify regulator current in mA, e.g. 5000mA
 * Use 0 for default
 */
void __init tegra_init_cpu_edp_limits(unsigned int regulator_mA)
{
	if (!regulator_mA)
		goto end;
	regulator_cur = regulator_mA + OVERRIDE_DEFAULT;

	switch (tegra_chip_id) {
	case TEGRA_CHIPID_TEGRA3:
		if (init_cpu_edp_limits_lookup() == 0)
			return;
		break;
	case TEGRA_CHIPID_TEGRA11:
	case TEGRA_CHIPID_TEGRA14:
		if (init_cpu_edp_limits_calculated() == 0)
			return;
		break;
	case TEGRA_CHIPID_TEGRA2:
	default:
		BUG();
		break;
	}

 end:
	edp_limits = edp_default_limits;
	edp_limits_size = ARRAY_SIZE(edp_default_limits);

	power_edp_limits = power_edp_default_limits;
	power_edp_limits_size = ARRAY_SIZE(power_edp_default_limits);
}

void tegra_get_cpu_edp_limits(const struct tegra_edp_limits **limits, int *size)
{
	*limits = edp_limits;
	*size = edp_limits_size;
}

void __init tegra_init_system_edp_limits(unsigned int power_limit_mW)
{
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int i;
	unsigned int *e;
	struct tegra_system_edp_entry *t;
	int tsize = 0;

	if (tegra_chip_id != TEGRA_CHIPID_TEGRA3) {
		e = NULL;
		goto out;
	}

	t = tegra3x_get_system_edp_map(&tsize);

	if (!power_limit_mW) {
		e = NULL;
		goto out;
	}

	for (i = 0; i < tsize; i++)
		if (t[i].speedo_id == cpu_speedo_id)
			break;

	if (i >= tsize) {
		e = NULL;
		goto out;
	}

	do {
		if (t[i].power_limit_100mW <= power_limit_mW / 100)
			break;
		i++;
	} while (i < tsize && t[i].speedo_id == cpu_speedo_id);

	if (i >= tsize || t[i].speedo_id != cpu_speedo_id)
		i--; /* No low enough entry in the table, use best possible */

	e = kmalloc(sizeof(unsigned int) * 4, GFP_KERNEL);
	BUG_ON(!e);

	e[0] = (unsigned int)t[i].freq_limits[0] * 10000;
	e[1] = (unsigned int)t[i].freq_limits[1] * 10000;
	e[2] = (unsigned int)t[i].freq_limits[2] * 10000;
	e[3] = (unsigned int)t[i].freq_limits[3] * 10000;

out:
	kfree(system_edp_limits);

	system_edp_limits = e;
}

void tegra_get_system_edp_limits(const unsigned int **limits)
{
	*limits = system_edp_limits;
}

void tegra_platform_edp_init(struct thermal_trip_info *trips,
				int *num_trips, int margin)
{
	const struct tegra_edp_limits *cpu_edp_limits;
	struct thermal_trip_info *trip_state;
	int i, cpu_edp_limits_size;

	if (!trips || !num_trips)
		return;

	/* edp capping */
	tegra_get_cpu_edp_limits(&cpu_edp_limits, &cpu_edp_limits_size);

	if (cpu_edp_limits_size > MAX_THROT_TABLE_SIZE)
		BUG();

	for (i = 0; i < cpu_edp_limits_size-1; i++) {
		trip_state = &trips[*num_trips];

		trip_state->cdev_type = "cpu_edp";
		trip_state->trip_temp =
			(cpu_edp_limits[i].temperature * 1000) - margin;
		trip_state->trip_type = THERMAL_TRIP_ACTIVE;
		trip_state->upper = trip_state->lower = i + 1;

		(*num_trips)++;

		if (*num_trips >= THERMAL_MAX_TRIPS)
			BUG();
	}
}

struct tegra_system_edp_entry *tegra_get_system_edp_entries(int *size)
{
	*size = power_edp_limits_size;
	return power_edp_limits;
}

#ifdef CONFIG_DEBUG_FS

static int edp_limit_debugfs_show(struct seq_file *s, void *data)
{
#ifdef CONFIG_CPU_FREQ
	seq_printf(s, "%u\n", tegra_get_edp_limit(NULL));
#endif
	return 0;
}

static int edp_debugfs_show(struct seq_file *s, void *data)
{
	int i, th_idx;

#ifdef CONFIG_CPU_FREQ
	tegra_get_edp_limit(&th_idx);
#else
	th_idx = 0;
#endif
	seq_printf(s, "-- VDD_CPU %sEDP table (%umA = %umA - %umA) --\n",
		   edp_limits == edp_default_limits ? "default " : "",
		   regulator_cur - edp_reg_override_mA,
		   regulator_cur, edp_reg_override_mA);
	seq_printf(s, "%6s %10s %10s %10s %10s\n",
		   " Temp.", "1-core", "2-cores", "3-cores", "4-cores");
	for (i = 0; i < edp_limits_size; i++) {
		seq_printf(s, "%c%3dC: %10u %10u %10u %10u\n",
			   i == th_idx ? '>' : ' ',
			   edp_limits[i].temperature,
			   edp_limits[i].freq_limits[0],
			   edp_limits[i].freq_limits[1],
			   edp_limits[i].freq_limits[2],
			   edp_limits[i].freq_limits[3]);
	}

	if (system_edp_limits) {
		seq_printf(s, "\n-- System EDP table --\n");
		seq_printf(s, "%10u %10u %10u %10u\n",
			   system_edp_limits[0],
			   system_edp_limits[1],
			   system_edp_limits[2],
			   system_edp_limits[3]);
	}

	return 0;
}

static int edp_reg_override_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Limit override: %u mA. Effective limit: %u mA\n",
		   edp_reg_override_mA, regulator_cur - edp_reg_override_mA);
	return 0;
}

static int edp_reg_override_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32], *end;
	unsigned int edp_reg_override_mA_temp;
	unsigned int edp_reg_override_mA_prev = edp_reg_override_mA;

	if (!(tegra_chip_id == TEGRA_CHIPID_TEGRA11 ||
		tegra_chip_id == TEGRA_CHIPID_TEGRA14))
		goto override_err;

	if (sizeof(buf) <= count)
		goto override_err;

	if (copy_from_user(buf, userbuf, count))
		goto override_err;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count]='\0';
	strim(buf);

	edp_reg_override_mA_temp = simple_strtoul(buf, &end, 10);
	if (*end != '\0')
		goto override_err;

	if (edp_reg_override_mA_temp >= regulator_cur)
		goto override_err;

	if (edp_reg_override_mA == edp_reg_override_mA_temp)
		return count;

	edp_reg_override_mA = edp_reg_override_mA_temp;
	if (init_cpu_edp_limits_calculated()) {
		/* Revert to previous override value if new value fails */
		edp_reg_override_mA = edp_reg_override_mA_prev;
		goto override_err;
	}

#ifdef CONFIG_CPU_FREQ
	if (tegra_cpu_set_speed_cap(NULL)) {
		pr_err("FAILED: Set CPU freq cap with new VDD_CPU EDP table\n");
		goto override_out;
	}

	pr_info("Reinitialized VDD_CPU EDP table with regulator current limit"
			" %u mA\n", regulator_cur - edp_reg_override_mA);
#else
	pr_err("FAILED: tegra_cpu_set_speed_cap() does not exist, failed to reinitialize VDD_CPU EDP table");
#endif

	return count;

override_err:
	pr_err("FAILED: Reinitialize VDD_CPU EDP table with override \"%s\"",
	       buf);
#ifdef CONFIG_CPU_FREQ
override_out:
#endif
	return -EINVAL;
}

static int edp_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_debugfs_show, inode->i_private);
}

static int edp_limit_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_limit_debugfs_show, inode->i_private);
}

static int edp_reg_override_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_reg_override_show, inode->i_private);
}

static const struct file_operations edp_debugfs_fops = {
	.open		= edp_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations edp_limit_debugfs_fops = {
	.open		= edp_limit_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations edp_reg_override_debugfs_fops = {
	.open		= edp_reg_override_open,
	.read		= seq_read,
	.write		= edp_reg_override_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_EDP_FRAMEWORK
static __init struct dentry *tegra_edp_debugfs_dir(void)
{
	return edp_debugfs_dir;
}
#else
static __init struct dentry *tegra_edp_debugfs_dir(void)
{
	return debugfs_create_dir("edp", NULL);
}
#endif

static int __init tegra_edp_debugfs_init(void)
{
	struct dentry *d_edp;
	struct dentry *d_edp_limit;
	struct dentry *d_edp_reg_override;
	struct dentry *edp_dir;
	struct dentry *vdd_cpu_dir;

	edp_dir = tegra_edp_debugfs_dir();

	if (!edp_dir)
		goto edp_dir_err;

	vdd_cpu_dir = debugfs_create_dir("vdd_cpu", edp_dir);

	if (!vdd_cpu_dir)
		goto vdd_cpu_dir_err;

	d_edp = debugfs_create_file("edp", S_IRUGO, vdd_cpu_dir, NULL,
				&edp_debugfs_fops);

	if (!d_edp)
		goto edp_err;

	d_edp_limit = debugfs_create_file("edp_limit", S_IRUGO, vdd_cpu_dir,
				NULL, &edp_limit_debugfs_fops);

	if (!d_edp_limit)
		goto edp_limit_err;

	d_edp_reg_override = debugfs_create_file("edp_reg_override",
				S_IRUGO | S_IWUSR, vdd_cpu_dir, NULL,
				&edp_reg_override_debugfs_fops);

	if (!d_edp_reg_override)
		goto edp_reg_override_err;

	if (tegra_core_edp_debugfs_init(edp_dir))
		goto edp_reg_override_err;

	return 0;

edp_reg_override_err:
	debugfs_remove(d_edp_limit);
edp_limit_err:
	debugfs_remove(d_edp);
edp_err:
	debugfs_remove(vdd_cpu_dir);
vdd_cpu_dir_err:
	debugfs_remove(edp_dir);
edp_dir_err:
	return -ENOMEM;
}

late_initcall(tegra_edp_debugfs_init);
#endif /* CONFIG_DEBUG_FS */
