/*
 * arch/arm/mach-tegra/tegra_cl_dvfs.c
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

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/clk.h>

#include "tegra_cl_dvfs.h"
#include "fuse.h"
#include "clock.h"
#include "dvfs.h"

/* FIXME: remove; for now, should be always checked in as "0" */
#define USE_IRAM_TO_TEST		0

#define OUT_MASK			0x3f

#define CL_DVFS_CTRL			0x00
#define CL_DVFS_CONFIG			0x04
#define CL_DVFS_CONFIG_DIV_MASK		0xff

#define CL_DVFS_PARAMS			0x08
#define CL_DVFS_PARAMS_CG_SCALE		(0x1 << 24)
#define CL_DVFS_PARAMS_FORCE_MODE_SHIFT	22
#define CL_DVFS_PARAMS_FORCE_MODE_MASK	(0x3 << CL_DVFS_PARAMS_FORCE_MODE_SHIFT)
#define CL_DVFS_PARAMS_CF_PARAM_SHIFT	16
#define CL_DVFS_PARAMS_CF_PARAM_MASK	(0x3f << CL_DVFS_PARAMS_CF_PARAM_SHIFT)
#define CL_DVFS_PARAMS_CI_PARAM_SHIFT	8
#define CL_DVFS_PARAMS_CI_PARAM_MASK	(0x7 << CL_DVFS_PARAMS_CI_PARAM_SHIFT)
#define CL_DVFS_PARAMS_CG_PARAM_SHIFT	0
#define CL_DVFS_PARAMS_CG_PARAM_MASK	(0xff << CL_DVFS_PARAMS_CG_PARAM_SHIFT)

#define CL_DVFS_TUNE0			0x0c
#define CL_DVFS_TUNE1			0x10

#define CL_DVFS_FREQ_REQ		0x14
#define CL_DVFS_FREQ_REQ_FORCE_ENABLE	(0x1 << 28)
#define CL_DVFS_FREQ_REQ_FORCE_SHIFT	16
#define CL_DVFS_FREQ_REQ_FORCE_MASK     (0xfff << CL_DVFS_FREQ_REQ_FORCE_SHIFT)
#define FORCE_MAX			2047
#define FORCE_MIN			-2048
#define CL_DVFS_FREQ_REQ_SCALE_SHIFT	8
#define CL_DVFS_FREQ_REQ_SCALE_MASK     (0xff << CL_DVFS_FREQ_REQ_SCALE_SHIFT)
#define SCALE_MAX			256
#define CL_DVFS_FREQ_REQ_FREQ_VALID	(0x1 << 7)
#define CL_DVFS_FREQ_REQ_FREQ_SHIFT	0
#define CL_DVFS_FREQ_REQ_FREQ_MASK      (0x7f << CL_DVFS_FREQ_REQ_FREQ_SHIFT)
#define FREQ_MAX			127

#define CL_DVFS_SCALE_RAMP		0x18

#define CL_DVFS_DROOP_CTRL		0x1c
#define CL_DVFS_DROOP_CTRL_MIN_FREQ_SHIFT 16
#define CL_DVFS_DROOP_CTRL_MIN_FREQ_MASK  \
		(0xff << CL_DVFS_DROOP_CTRL_MIN_FREQ_SHIFT)
#define CL_DVFS_DROOP_CTRL_CUT_SHIFT	8
#define CL_DVFS_DROOP_CTRL_CUT_MASK     (0xf << CL_DVFS_DROOP_CTRL_CUT_SHIFT)
#define CL_DVFS_DROOP_CTRL_RAMP_SHIFT	0
#define CL_DVFS_DROOP_CTRL_RAMP_MASK    (0xff << CL_DVFS_DROOP_CTRL_RAMP_SHIFT)

#define CL_DVFS_OUTPUT_CFG		0x20
#define CL_DVFS_OUTPUT_CFG_I2C_ENABLE	(0x1 << 30)
#define CL_DVFS_OUTPUT_CFG_SAFE_SHIFT	24
#define CL_DVFS_OUTPUT_CFG_SAFE_MASK    \
		(OUT_MASK << CL_DVFS_OUTPUT_CFG_SAFE_SHIFT)
#define CL_DVFS_OUTPUT_CFG_MAX_SHIFT	16
#define CL_DVFS_OUTPUT_CFG_MAX_MASK     \
		(OUT_MASK << CL_DVFS_OUTPUT_CFG_MAX_SHIFT)
#define CL_DVFS_OUTPUT_CFG_MIN_SHIFT	8
#define CL_DVFS_OUTPUT_CFG_MIN_MASK     \
		(OUT_MASK << CL_DVFS_OUTPUT_CFG_MIN_SHIFT)

#define CL_DVFS_OUTPUT_FORCE		0x24
#define CL_DVFS_MONITOR_CTRL		0x28
#define CL_DVFS_MONITOR_CTRL_DISABLE	0

#define CL_DVFS_I2C_CFG			0x40
#define CL_DVFS_I2C_CFG_ARB_ENABLE	(0x1 << 20)
#define CL_DVFS_I2C_CFG_HS_CODE_SHIFT	16
#define CL_DVFS_I2C_CFG_HS_CODE_MASK	(0x7 << CL_DVFS_I2C_CFG_HS_CODE_SHIFT)
#define CL_DVFS_I2C_CFG_PACKET_ENABLE	(0x1 << 15)
#define CL_DVFS_I2C_CFG_SIZE_SHIFT	12
#define CL_DVFS_I2C_CFG_SIZE_MASK	(0x7 << CL_DVFS_I2C_CFG_SIZE_SHIFT)
#define CL_DVFS_I2C_CFG_SLAVE_ADDR_10	(0x1 << 10)
#define CL_DVFS_I2C_CFG_SLAVE_ADDR_SHIFT 0
#define CL_DVFS_I2C_CFG_SLAVE_ADDR_MASK	\
		(0x3ff << CL_DVFS_I2C_CFG_SLAVE_ADDR_SHIFT)

#define CL_DVFS_I2C_VDD_REG_ADDR	0x44
#define CL_DVFS_I2C_STS			0x48

#define CL_DVFS_INTR_STS		0x5c
#define CL_DVFS_INTR_EN			0x60
#define CL_DVFS_INTR_MIN_MASK		0x1
#define CL_DVFS_INTR_MAX_MASK		0x2

#define CL_DVFS_I2C_CLK_DIVISOR		0x16c
#define CL_DVFS_I2C_CLK_DIVISOR_MASK	0xffff
#define CL_DVFS_I2C_CLK_DIVISOR_FS_SHIFT 0
#define CL_DVFS_I2C_CLK_DIVISOR_HS_SHIFT 16

#define CL_DVFS_OUTPUT_LUT		0x200

/* Conversion macros (different scales for frequency request, and monitored
   rate is not a typo)*/
#define GET_REQUEST_FREQ(rate, ref_rate)	((rate) / ((ref_rate) / 2))
#define GET_MONITORED_RATE(freq, ref_rate)	((freq) * ((ref_rate) / 4))
#define GET_DROOP_FREQ(rate, ref_rate)		((rate) / ((ref_rate) / 4))
#define GET_DIV(ref_rate, out_rate, scale)	\
		DIV_ROUND_CLOSEST((ref_rate), (out_rate) * (scale))

enum tegra_cl_dvfs_ctrl_mode {
	TEGRA_CL_DVFS_UNINITIALIZED = 0,
	TEGRA_CL_DVFS_DISABLED = 1,
	TEGRA_CL_DVFS_OPEN_LOOP = 2,
	TEGRA_CL_DVFS_CLOSED_LOOP = 3,
};

struct dfll_rate_req {
	u8	freq;
	u8	scale;
	u8	output;
};

struct tegra_cl_dvfs {
	struct tegra_cl_dvfs_soc_data		*soc_data;
	struct tegra_cl_dvfs_platform_data	*p_data;

	struct clk			*cpu_clk;
	struct clk			*soc_clk;
	struct clk			*ref_clk;
	struct clk			*i2c_clk;
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
static struct tegra_cl_dvfs	cl_dvfs;

#if USE_IRAM_TO_TEST
static void __iomem *cl_dvfs_base = IO_ADDRESS(TEGRA_IRAM_BASE + 0x3f000);
#else
static void __iomem *cl_dvfs_base = IO_ADDRESS(TEGRA_CL_DVFS_BASE);
#endif

static inline u32 cl_dvfs_readl(u32 offset)
{
	return __raw_readl((u32)cl_dvfs_base + offset);
}
static inline void cl_dvfs_writel(u32 val, u32 offset)
{
	__raw_writel(val, (u32)cl_dvfs_base + offset);
}
static inline void cl_dvfs_wmb(void)
{
	wmb();
	cl_dvfs_readl(CL_DVFS_CTRL);
}

static inline void dfll_reset(bool assert)
{
	u32 val = assert ? 1 : 0;
	__raw_writel(val, (u32)IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x2f4);
	udelay(2);
}

static inline void output_enable(bool enable)
{
	u32 val = cl_dvfs_readl(CL_DVFS_OUTPUT_CFG);

	/* FIXME: PWM output control */
	if (enable)
		val |= CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
	else
		val &= ~CL_DVFS_OUTPUT_CFG_I2C_ENABLE;

	cl_dvfs_writel(val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb();
}

static inline void set_mode(struct tegra_cl_dvfs *cld,
			    enum tegra_cl_dvfs_ctrl_mode mode)
{
	cld->mode = mode;
	cl_dvfs_writel(mode - 1, CL_DVFS_CTRL);
	cl_dvfs_wmb();
}

static int find_safe_output(
	struct tegra_cl_dvfs *cld, unsigned long rate, u8 *safe_output)
{
	int i;
	int n = cld->cpu_clk->dvfs->num_freqs;
	unsigned long *freqs = cld->cpu_clk->dvfs->freqs;

	for (i = 0; i < n; i++) {
		if (freqs[i] >= rate) {
			*safe_output = cld->clk_dvfs_map[i];
			return 0;
		}
	}
	return -EINVAL;
}

static struct voltage_reg_map *find_vdd_map_entry(
	struct tegra_cl_dvfs *cld, int mV, bool exact)
{
	int i, reg_mV;

	for (i = 0; i < cld->p_data->vdd_map_size; i++) {
		/* round down to 1mV */
		reg_mV = cld->p_data->vdd_map[i].reg_uV / 1000;
		if (mV <= reg_mV)
			break;
	}

	if (i < cld->p_data->vdd_map_size) {
		if (!exact || (mV == reg_mV))
			return &cld->p_data->vdd_map[i];
	}
	return NULL;
}

static void cl_dvfs_init_maps(struct tegra_cl_dvfs *cld)
{
	int i, j, v, v_max;
	const int *millivolts;
	struct voltage_reg_map *m;

	BUILD_BUG_ON(MAX_DVFS_FREQS >= MAX_CL_DVFS_VOLTAGES);
	BUILD_BUG_ON(MAX_CL_DVFS_VOLTAGES > OUT_MASK + 1);
	BUG_ON(!cld->cpu_clk || !cld->cpu_clk->dvfs);

	millivolts = cld->cpu_clk->dvfs->millivolts;
	v_max = cld->cpu_clk->dvfs->max_millivolts;

	for (i = 0, j = 0; i < MAX_DVFS_FREQS; i++) {
		v = millivolts[i];
		m = find_vdd_map_entry(cld, millivolts[i], true);
		BUG_ON(!m);

		if (!j || (m != cld->out_map[j - 1]))
			cld->out_map[j++] = m;
		cld->clk_dvfs_map[i] = j - 1;

		if (v >= v_max)
			break;

		for (;;) {
			v += (v_max - v) / (MAX_CL_DVFS_VOLTAGES - j);
			if (v >= millivolts[i + 1])
				break;

			m = find_vdd_map_entry(cld, v, false);
			BUG_ON(!m);
			if (m != cld->out_map[j - 1])
				cld->out_map[j++] = m;
		}
	}
	BUG_ON(millivolts[i] != v_max);
	BUG_ON(j > MAX_CL_DVFS_VOLTAGES);
	cld->num_voltages = j;
}

static void cl_dvfs_init_pwm_if(struct tegra_cl_dvfs *cld)
{
	/* FIXME: not supported */
}

static void cl_dvfs_init_i2c_if(struct tegra_cl_dvfs *cld)
{
	u32 val;
	struct tegra_cl_dvfs_platform_data *p_data = cld->p_data;
	bool hs_mode = p_data->u.pmu_i2c.hs_master_code &&
		p_data->u.pmu_i2c.hs_rate;

	/* PMU slave address, vdd register offset, and transfer mode */
	val = p_data->u.pmu_i2c.slave_addr << CL_DVFS_I2C_CFG_SLAVE_ADDR_SHIFT;
	if (p_data->u.pmu_i2c.addr_10)
		val |= CL_DVFS_I2C_CFG_SLAVE_ADDR_10;
	if (hs_mode) {
		val |= p_data->u.pmu_i2c.hs_master_code <<
			CL_DVFS_I2C_CFG_HS_CODE_SHIFT;
		val |= CL_DVFS_I2C_CFG_PACKET_ENABLE;
	}
	val |= CL_DVFS_I2C_CFG_SIZE_MASK;
	val |= CL_DVFS_I2C_CFG_ARB_ENABLE;
	cl_dvfs_writel(val, CL_DVFS_I2C_CFG);
	cl_dvfs_writel(p_data->u.pmu_i2c.reg, CL_DVFS_I2C_VDD_REG_ADDR);


	val = GET_DIV(cld->i2c_rate, p_data->u.pmu_i2c.fs_rate, 8);
	BUG_ON(!val || (val > CL_DVFS_I2C_CLK_DIVISOR_MASK));
	val = (val - 1) << CL_DVFS_I2C_CLK_DIVISOR_FS_SHIFT;
	if (hs_mode) {
		u32 div = GET_DIV(cld->i2c_rate, p_data->u.pmu_i2c.hs_rate, 8);
		BUG_ON(!div || (div > CL_DVFS_I2C_CLK_DIVISOR_MASK));
		val |= (div - 1) << CL_DVFS_I2C_CLK_DIVISOR_FS_SHIFT;
	}
	cl_dvfs_writel(val, CL_DVFS_I2C_CLK_DIVISOR);
	cl_dvfs_wmb();
}

static void cl_dvfs_init_out_if(struct tegra_cl_dvfs *cld)
{
	int i;
	u32 val;

	/* disable output, and set output limits, use medium volatge
	   level as safe; disable and clear limit interrupts */
	cld->safe_ouput = cld->num_voltages / 2;
	val = ((cld->num_voltages / 2) << CL_DVFS_OUTPUT_CFG_SAFE_SHIFT) |
		((cld->num_voltages - 1) << CL_DVFS_OUTPUT_CFG_MAX_SHIFT);
	cl_dvfs_writel(val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb();

	cl_dvfs_writel(0, CL_DVFS_OUTPUT_FORCE);
	cl_dvfs_writel(0, CL_DVFS_INTR_EN);
	cl_dvfs_writel(CL_DVFS_INTR_MAX_MASK | CL_DVFS_INTR_MIN_MASK,
		       CL_DVFS_INTR_STS);

	/* fill in LUT table */
	for (i = 0; i < cld->num_voltages; i++) {
		val = cld->out_map[i]->reg_value;
		cl_dvfs_writel(val, CL_DVFS_OUTPUT_LUT + i * 4);
	}
	cl_dvfs_wmb();

	/* configure transport */
	if (cl_dvfs.p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C)
		cl_dvfs_init_i2c_if(cld);
	else
		cl_dvfs_init_pwm_if(cld);
}

static void cl_dvfs_init_cntrl_logic(struct tegra_cl_dvfs *cld)
{
	u32 val;
	struct tegra_cl_dvfs_cfg_param *param = cld->p_data->cfg_param;

	/* configure mode, control loop parameters, DFLL tuning */
	set_mode(cld, TEGRA_CL_DVFS_DISABLED);

	val = GET_DIV(cld->ref_rate, param->sample_rate, 32);
	BUG_ON(val > CL_DVFS_CONFIG_DIV_MASK);
	cl_dvfs_writel(val, CL_DVFS_CONFIG);

	val = (param->force_mode << CL_DVFS_PARAMS_FORCE_MODE_SHIFT) |
		(param->cf << CL_DVFS_PARAMS_CF_PARAM_SHIFT) |
		(param->ci << CL_DVFS_PARAMS_CI_PARAM_SHIFT) |
		(param->cg << CL_DVFS_PARAMS_CG_PARAM_SHIFT);
	cl_dvfs_writel(val, CL_DVFS_PARAMS);

	cl_dvfs_writel(cld->soc_data->tune0, CL_DVFS_TUNE0);
	cl_dvfs_writel(cld->soc_data->tune1, CL_DVFS_TUNE1);

	/* configure droop (skipper 1) and scale (skipper 2) */
	val = GET_DROOP_FREQ(cld->soc_data->droop_cpu_rate_min, cld->ref_rate);
	val <<= CL_DVFS_DROOP_CTRL_MIN_FREQ_SHIFT;
	BUG_ON(val > CL_DVFS_DROOP_CTRL_MIN_FREQ_MASK);
	val |= (param->droop_cut_value << CL_DVFS_DROOP_CTRL_CUT_SHIFT);
	val |= (param->droop_restore_ramp << CL_DVFS_DROOP_CTRL_RAMP_SHIFT);
	cl_dvfs_writel(val, CL_DVFS_DROOP_CTRL);

	/* FIXME: does dfll_rate_min require separate charact entry ? */
	cld->dfll_rate_min = cld->soc_data->droop_cpu_rate_min;

	cld->last_req.freq = 0;
	cld->last_req.output = 0;
	cld->last_req.scale = SCALE_MAX - 1;
	cl_dvfs_writel(CL_DVFS_FREQ_REQ_SCALE_MASK, CL_DVFS_FREQ_REQ);
	cl_dvfs_writel(param->scale_out_ramp, CL_DVFS_SCALE_RAMP);

	/* disable monitoring */
	cl_dvfs_writel(CL_DVFS_MONITOR_CTRL_DISABLE, CL_DVFS_MONITOR_CTRL);
	cl_dvfs_wmb();
}

int __init tegra_init_cl_dvfs(void)
{
	int ret;

#if !USE_IRAM_TO_TEST
#ifndef CONFIG_TEGRA_SILICON_PLATFORM
	u32 netlist, patchid;
	tegra_get_netlist_revision(&netlist, &patchid);
	if (netlist < 13) {
		pr_err("%s: CL-DVFS is not available on net %d\n",
		       __func__, netlist);
		return -ENOSYS;
	}
#endif
#endif
	/* Check platform and SoC data, get i2c clock */
	if (!cl_dvfs.soc_data) {
		pr_err("%s: SoC tuning data is not available\n", __func__);
		return -EINVAL;
	}
	if (!cl_dvfs.p_data || !cl_dvfs.p_data->cfg_param) {
		pr_err("%s: Platform data is not available\n", __func__);
		return -EINVAL;
	}
	if (cl_dvfs.p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C) {
		cl_dvfs.i2c_clk = clk_get_sys("tegra_cl_dvfs", "i2c");
		BUG_ON(IS_ERR_OR_NULL(cl_dvfs.i2c_clk));
		ret = clk_enable(cl_dvfs.i2c_clk);
		if (ret) {
			pr_err("%s: Failed to enable %s\n",
			       __func__, cl_dvfs.i2c_clk->name);
			return ret;
		}
		cl_dvfs.i2c_rate = clk_get_rate(cl_dvfs.i2c_clk);
	} else {
		pr_err("%s: PMU interface is not I2C\n", __func__);
		return -EINVAL;
	}

	/* Enable clocks, release control logic reset (DFLL is still reset) */
	cl_dvfs.ref_clk = tegra_get_clock_by_name("cl_dvfs_ref");
	cl_dvfs.soc_clk = tegra_get_clock_by_name("cl_dvfs_soc");
	cl_dvfs.cpu_clk = tegra_get_clock_by_name("cpu_g");
	BUG_ON(!cl_dvfs.ref_clk || !cl_dvfs.soc_clk || !cl_dvfs.cpu_clk);

	ret = clk_enable(cl_dvfs.ref_clk);
	if (ret) {
		pr_err("%s: Failed to enable %s\n",
		       __func__, cl_dvfs.ref_clk->name);
		return ret;
	}
	ret = clk_enable(cl_dvfs.soc_clk);
	if (ret) {
		pr_err("%s: Failed to enable %s\n",
		       __func__, cl_dvfs.ref_clk->name);
		return ret;
	}
	cl_dvfs.ref_rate = clk_get_rate(cl_dvfs.ref_clk);
	BUG_ON(!cl_dvfs.ref_rate);

	/* Get ready ouput voltage mapping*/
	cl_dvfs_init_maps(&cl_dvfs);

	/* setup PMU interface */
	cl_dvfs_init_out_if(&cl_dvfs);

	/* Configure control registers in disabled mode */
	cl_dvfs_init_cntrl_logic(&cl_dvfs);

	return 0;
}

void tegra_cl_dvfs_set_plarform_data(struct tegra_cl_dvfs_platform_data *data)
{
	cl_dvfs.p_data = data;
}

void tegra_cl_dvfs_set_soc_data(struct tegra_cl_dvfs_soc_data *data)
{
	cl_dvfs.soc_data = data;
}

/*
 * CL_DVFS states:
 *
 * - DISABLED: control logic mode - DISABLE, output interface disabled,
 *   dfll in reset
 * - OPEN_LOOP: control logic mode - OPEN_LOOP, output interface disabled,
 *   dfll is running "unlocked"
 * - CLOSED_LOOP: control logic mode - CLOSED_LOOP, output interface enabled,
 *   dfll is running "locked"
 */

/* Switch from any other state to DISABLED state */
void tegra_cl_dvfs_disable(void)
{
	struct tegra_cl_dvfs *cld = &cl_dvfs;

	if ((cld->mode == TEGRA_CL_DVFS_UNINITIALIZED) ||
	    (cld->mode == TEGRA_CL_DVFS_DISABLED))
		return;

	output_enable(false);
	set_mode(cld, TEGRA_CL_DVFS_DISABLED);
	dfll_reset(true);
}

/* Switch from DISABLE state to OPEN_LOOP state */
int tegra_cl_dvfs_enable(void)
{
	struct tegra_cl_dvfs *cld = &cl_dvfs;

	if (cld->mode == TEGRA_CL_DVFS_UNINITIALIZED) {
		pr_err("%s: Cannot enable DFLL in mode %d\n",
		       __func__, cld->mode);
		return -EINVAL;
	}

	if (cld->mode != TEGRA_CL_DVFS_DISABLED)
		return 0;

	dfll_reset(false);
	set_mode(cld, TEGRA_CL_DVFS_OPEN_LOOP);
	return 0;
}

/* Switch from OPEN_LOOP state to CLOSED_LOOP state */
int tegra_cl_dvfs_lock(void)
{
	u32 val;
	struct tegra_cl_dvfs *cld = &cl_dvfs;
	struct dfll_rate_req *req = &cld->last_req;

	switch (cld->mode) {
	case TEGRA_CL_DVFS_CLOSED_LOOP:
		return 0;

	case TEGRA_CL_DVFS_OPEN_LOOP:
		if (req->freq == 0) {
			pr_err("%s: Cannot lock DFLL at rate 0\n", __func__);
			return -EINVAL;
		}

		/* update control logic setting with last rate request;
		   use request safe output to set safe volatge */
		val = cl_dvfs_readl(CL_DVFS_OUTPUT_CFG);
		val &= ~CL_DVFS_OUTPUT_CFG_SAFE_MASK;
		val |= req->output << CL_DVFS_OUTPUT_CFG_SAFE_SHIFT;
		cl_dvfs_writel(val, CL_DVFS_OUTPUT_CFG);
		cld->safe_ouput = req->output;

		val = req->freq << CL_DVFS_FREQ_REQ_FREQ_SHIFT;
		val |= req->scale << CL_DVFS_FREQ_REQ_SCALE_SHIFT;
		val |= CL_DVFS_FREQ_REQ_FREQ_VALID |
			CL_DVFS_FREQ_REQ_FORCE_ENABLE;
		cl_dvfs_writel(val, CL_DVFS_FREQ_REQ);

		output_enable(true);
		set_mode(cld, TEGRA_CL_DVFS_CLOSED_LOOP);
		return 0;

	default:
		pr_err("%s: Cannot lock DFLL in mode %d\n",
		       __func__, cld->mode);
		return -EINVAL;
	}
}

/* Switch from CLOSED_LOOP state to OPEN_LOOP state */
int tegra_cl_dvfs_unlock(void)
{
	struct tegra_cl_dvfs *cld = &cl_dvfs;

	switch (cld->mode) {
	case TEGRA_CL_DVFS_CLOSED_LOOP:
		output_enable(false);
		set_mode(cld, TEGRA_CL_DVFS_OPEN_LOOP);
		return 0;

	case TEGRA_CL_DVFS_OPEN_LOOP:
		return 0;

	default:
		pr_err("%s: Cannot unlock DFLL in mode %d\n",
		       __func__, cld->mode);
		return -EINVAL;
	}
}
/*
 * Convert requested rate into the control logic settings. In CLOSED_LOOP mode,
 * update new settings immediately to adjust DFLL output rate accordingly.
 * Otherwise, just save them until next switch to closed loop.
 */
int tegra_cl_dvfs_request_rate(unsigned long rate)
{
	u32 val;
	struct dfll_rate_req req;
	struct tegra_cl_dvfs *cld = &cl_dvfs;

	if (cld->mode == TEGRA_CL_DVFS_UNINITIALIZED) {
		pr_err("%s: Cannot set DFLL rate in mode %d\n",
		       __func__, cld->mode);
		return -EINVAL;
	}

	/* Determine DFLL output scale */
	req.scale = SCALE_MAX - 1;
	if (rate < cld->dfll_rate_min) {
		req.scale = rate / DIV_ROUND_UP(cld->dfll_rate_min, SCALE_MAX);
		if (!req.scale) {
			pr_err("%s: Rate %lu is below scalable range\n",
			       __func__, rate);
			return -EINVAL;
		}
		req.scale--;
		rate = cld->dfll_rate_min;
	}

	/* Convert requested rate into frequency request and scale settings */
	val = GET_REQUEST_FREQ(rate, cld->ref_rate);
	if (val > FREQ_MAX) {
		pr_err("%s: Rate %lu is above dfll range\n", __func__, rate);
		return -EINVAL;
	}
	req.freq = val;

	/* Find safe voltage for requested rate */
	if (find_safe_output(cld, rate, &req.output)) {
		pr_err("%s: Failed to find safe output for rate %lu\n",
		       __func__, rate);
		return -EINVAL;
	}

	/* Save validated request, and in CLOSED_LOOP mode actually update
	   control logic settings; use request safe output to set forced
	   voltage */
	cld->last_req = req;

	if (cld->mode == TEGRA_CL_DVFS_CLOSED_LOOP) {
		int force_val = req.output - cld->safe_ouput;
		force_val = (force_val * 128) /
			((s8)cld->p_data->cfg_param->cg);
		force_val = clamp(force_val, FORCE_MIN, FORCE_MAX);
		val |= ((u32)force_val << CL_DVFS_FREQ_REQ_FORCE_SHIFT) &
					CL_DVFS_FREQ_REQ_FORCE_MASK;

		val |= req.scale << CL_DVFS_FREQ_REQ_SCALE_SHIFT;
		val |= CL_DVFS_FREQ_REQ_FREQ_VALID |
			CL_DVFS_FREQ_REQ_FORCE_ENABLE;
		cl_dvfs_writel(val, CL_DVFS_FREQ_REQ);
		cl_dvfs_wmb();
	}
	return 0;
}

#ifdef CONFIG_DEBUG_FS

static struct dentry *cl_dvfs_debugfs_root;

static int enable_get(void *data, u64 *val)
{
	*val = cl_dvfs.mode >= TEGRA_CL_DVFS_OPEN_LOOP;
	return 0;
}
static int enable_set(void *data, u64 val)
{
	if (val) {
		return tegra_cl_dvfs_enable();
	} else {
		tegra_cl_dvfs_disable();
		return 0;
	}
}
DEFINE_SIMPLE_ATTRIBUTE(enable_fops, enable_get, enable_set, "%llu\n");

static int lock_get(void *data, u64 *val)
{
	*val = cl_dvfs.mode == TEGRA_CL_DVFS_CLOSED_LOOP;
	return 0;
}
static int lock_set(void *data, u64 val)
{
	if (val)
		return tegra_cl_dvfs_lock();
	else
		return tegra_cl_dvfs_unlock();
}
DEFINE_SIMPLE_ATTRIBUTE(lock_fops, lock_get, lock_set, "%llu\n");

static int request_get(void *data, u64 *val)
{
	struct tegra_cl_dvfs *cld = &cl_dvfs;
	struct dfll_rate_req *req = &cld->last_req;

	*val = (cld->ref_rate / 2) * req->freq / SCALE_MAX * (req->scale + 1);
	return 0;
}
static int request_set(void *data, u64 val)
{
	return tegra_cl_dvfs_request_rate(val);
}
DEFINE_SIMPLE_ATTRIBUTE(request_fops, request_get, request_set, "%llu\n");

static int __init tegra_cl_dvfs_debug_init(void)
{
	if (cl_dvfs.mode == TEGRA_CL_DVFS_UNINITIALIZED)
		return 0;

	cl_dvfs_debugfs_root = debugfs_create_dir("tegra_cl_dvfs", NULL);
	if (!cl_dvfs_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file("cl_dvfs_enable", S_IRUGO | S_IWUSR,
				  cl_dvfs_debugfs_root, NULL, &enable_fops))
		goto err_out;

	if (!debugfs_create_file("cl_dvfs_lock", S_IRUGO | S_IWUSR,
				 cl_dvfs_debugfs_root, NULL, &lock_fops))
		goto err_out;

	if (!debugfs_create_file("cl_dvfs_request", S_IRUGO | S_IWUSR,
				 cl_dvfs_debugfs_root, NULL, &request_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(cl_dvfs_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_cl_dvfs_debug_init);
#endif
late_initcall(tegra_init_cl_dvfs);
