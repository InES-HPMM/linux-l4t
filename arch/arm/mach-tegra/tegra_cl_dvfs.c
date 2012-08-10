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
#include <linux/module.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/hardware.h>

#include "tegra_cl_dvfs.h"
#include "clock.h"

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
#define CL_DVFS_MONITOR_DATA		0x2c

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
#define CL_DVFS_I2C_STS_I2C_REQ_PENDING	0x1

#define CL_DVFS_INTR_STS		0x5c
#define CL_DVFS_INTR_EN			0x60
#define CL_DVFS_INTR_MIN_MASK		0x1
#define CL_DVFS_INTR_MAX_MASK		0x2

#define CL_DVFS_I2C_CLK_DIVISOR		0x16c
#define CL_DVFS_I2C_CLK_DIVISOR_MASK	0xffff
#define CL_DVFS_I2C_CLK_DIVISOR_FS_SHIFT 16
#define CL_DVFS_I2C_CLK_DIVISOR_HS_SHIFT 0

#define CL_DVFS_OUTPUT_LUT		0x200

#define CL_DVFS_OUTPUT_PENDING_TIMEOUT	1000

/* Conversion macros (different scales for frequency request, and monitored
   rate is not a typo)*/
#define GET_REQUEST_FREQ(rate, ref_rate)	((rate) / ((ref_rate) / 2))
#define GET_REQUEST_RATE(freq, ref_rate)	((freq) * ((ref_rate) / 2))
#define GET_MONITORED_RATE(freq, ref_rate)	((freq) * ((ref_rate) / 4))
#define GET_DROOP_FREQ(rate, ref_rate)		((rate) / ((ref_rate) / 4))
#define ROUND_MIN_RATE(rate, ref_rate)		\
		(DIV_ROUND_UP(rate, (ref_rate) / 2) * ((ref_rate) / 2))
#define GET_DIV(ref_rate, out_rate, scale)	\
		DIV_ROUND_UP((ref_rate), (out_rate) * (scale))

static const char *mode_name[] = {
	[TEGRA_CL_DVFS_UNINITIALIZED] = "uninitialized",
	[TEGRA_CL_DVFS_DISABLED] = "disabled",
	[TEGRA_CL_DVFS_OPEN_LOOP] = "open_loop",
	[TEGRA_CL_DVFS_CLOSED_LOOP] = "closed_loop",
};

static inline u32 cl_dvfs_readl(struct tegra_cl_dvfs *cld, u32 offs)
{
	return __raw_readl(cld->cl_base + offs);
}
static inline void cl_dvfs_writel(struct tegra_cl_dvfs *cld, u32 val, u32 offs)
{
	__raw_writel(val, cld->cl_base + offs);
}
static inline void cl_dvfs_wmb(struct tegra_cl_dvfs *cld)
{
	wmb();
	cl_dvfs_readl(cld, CL_DVFS_CTRL);
}

static inline void output_enable(struct tegra_cl_dvfs *cld, bool enable)
{
	u32 val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);

	/* FIXME: PWM output control */
	if (enable)
		val |= CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
	else
		val &= ~CL_DVFS_OUTPUT_CFG_I2C_ENABLE;

	cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb(cld);

	if (!enable) {
		int i;
		for (i = 0; i < CL_DVFS_OUTPUT_PENDING_TIMEOUT; i++) {
			udelay(1);
			val = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
			if (!(val & CL_DVFS_I2C_STS_I2C_REQ_PENDING))
				return;
		}
		pr_err("%s: I2C pending transaction timeout\n", __func__);
	}
}

static inline void set_mode(struct tegra_cl_dvfs *cld,
			    enum tegra_cl_dvfs_ctrl_mode mode)
{
	cld->mode = mode;
	cl_dvfs_writel(cld, mode - 1, CL_DVFS_CTRL);
	cl_dvfs_wmb(cld);
}

static int find_safe_output(
	struct tegra_cl_dvfs *cld, unsigned long rate, u8 *safe_output)
{
	int i;
	int n = cld->safe_dvfs->num_freqs;
	unsigned long *freqs = cld->safe_dvfs->freqs;

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
	int i, j, v, v_max, n;
	const int *millivolts;
	struct voltage_reg_map *m;

	BUILD_BUG_ON(MAX_CL_DVFS_VOLTAGES > OUT_MASK + 1);
	BUG_ON(!cld->safe_dvfs);

	n = cld->safe_dvfs->num_freqs;
	BUG_ON(n >= MAX_CL_DVFS_VOLTAGES);

	millivolts = cld->safe_dvfs->dfll_millivolts;
	v_max = millivolts[n - 1];

	v = cld->dfll_data->millivolts_min;
	BUG_ON(v > millivolts[0]);

	cld->out_map[0] = find_vdd_map_entry(cld, v, true);
	BUG_ON(!cld->out_map[0]);

	for (i = 0, j = 1; i < n; i++) {
		for (;;) {
			v += max(1, (v_max - v) / (MAX_CL_DVFS_VOLTAGES - j));
			if (v >= millivolts[i])
				break;

			m = find_vdd_map_entry(cld, v, false);
			BUG_ON(!m);
			if (m != cld->out_map[j - 1])
				cld->out_map[j++] = m;
		}

		v = millivolts[i];
		m = find_vdd_map_entry(cld, v, true);
		BUG_ON(!m);
		if (m != cld->out_map[j - 1])
			cld->out_map[j++] = m;
		cld->clk_dvfs_map[i] = j - 1;

		if (v >= v_max)
			break;
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
	u32 val, div;
	struct tegra_cl_dvfs_platform_data *p_data = cld->p_data;
	bool hs_mode = p_data->u.pmu_i2c.hs_rate;

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
	cl_dvfs_writel(cld, val, CL_DVFS_I2C_CFG);
	cl_dvfs_writel(cld, p_data->u.pmu_i2c.reg, CL_DVFS_I2C_VDD_REG_ADDR);


	val = GET_DIV(cld->i2c_rate, p_data->u.pmu_i2c.fs_rate, 8);
	BUG_ON(!val || (val > CL_DVFS_I2C_CLK_DIVISOR_MASK));
	val = (val - 1) << CL_DVFS_I2C_CLK_DIVISOR_FS_SHIFT;
	if (hs_mode) {
		div = GET_DIV(cld->i2c_rate, p_data->u.pmu_i2c.hs_rate, 12);
		BUG_ON(!div || (div > CL_DVFS_I2C_CLK_DIVISOR_MASK));
	} else {
		div = 2;	/* default hs divisor just in case */
	}
	val |= (div - 1) << CL_DVFS_I2C_CLK_DIVISOR_HS_SHIFT;
	cl_dvfs_writel(cld, val, CL_DVFS_I2C_CLK_DIVISOR);
	cl_dvfs_wmb(cld);
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
	cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb(cld);

	cl_dvfs_writel(cld, 0, CL_DVFS_OUTPUT_FORCE);
	cl_dvfs_writel(cld, 0, CL_DVFS_INTR_EN);
	cl_dvfs_writel(cld, CL_DVFS_INTR_MAX_MASK | CL_DVFS_INTR_MIN_MASK,
		       CL_DVFS_INTR_STS);

	/* fill in LUT table */
	for (i = 0; i < cld->num_voltages; i++) {
		val = cld->out_map[i]->reg_value;
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_LUT + i * 4);
	}
	cl_dvfs_wmb(cld);

	/* configure transport */
	if (cld->p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C)
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
	cl_dvfs_writel(cld, val, CL_DVFS_CONFIG);

	val = (param->force_mode << CL_DVFS_PARAMS_FORCE_MODE_SHIFT) |
		(param->cf << CL_DVFS_PARAMS_CF_PARAM_SHIFT) |
		(param->ci << CL_DVFS_PARAMS_CI_PARAM_SHIFT) |
		((u8)param->cg << CL_DVFS_PARAMS_CG_PARAM_SHIFT) |
		(param->cg_scale ? CL_DVFS_PARAMS_CG_SCALE : 0);
	cl_dvfs_writel(cld, val, CL_DVFS_PARAMS);

	cl_dvfs_writel(cld, cld->dfll_data->tune0, CL_DVFS_TUNE0);
	cl_dvfs_writel(cld, cld->dfll_data->tune1, CL_DVFS_TUNE1);

	/* configure droop (skipper 1) and scale (skipper 2) */
	val = GET_DROOP_FREQ(cld->dfll_data->droop_rate_min, cld->ref_rate);
	val <<= CL_DVFS_DROOP_CTRL_MIN_FREQ_SHIFT;
	BUG_ON(val > CL_DVFS_DROOP_CTRL_MIN_FREQ_MASK);
	val |= (param->droop_cut_value << CL_DVFS_DROOP_CTRL_CUT_SHIFT);
	val |= (param->droop_restore_ramp << CL_DVFS_DROOP_CTRL_RAMP_SHIFT);
	cl_dvfs_writel(cld, val, CL_DVFS_DROOP_CTRL);

	/* round minimum rate to request unit (ref_rate/2) boundary */
	cld->dfll_rate_min = cld->dfll_data->out_rate_min;
	cld->dfll_rate_min = ROUND_MIN_RATE(cld->dfll_rate_min, cld->ref_rate);

	cld->last_req.freq = 0;
	cld->last_req.output = 0;
	cld->last_req.scale = SCALE_MAX - 1;
	cl_dvfs_writel(cld, CL_DVFS_FREQ_REQ_SCALE_MASK, CL_DVFS_FREQ_REQ);
	cl_dvfs_writel(cld, param->scale_out_ramp, CL_DVFS_SCALE_RAMP);

	/* disable monitoring */
	cl_dvfs_writel(cld, CL_DVFS_MONITOR_CTRL_DISABLE, CL_DVFS_MONITOR_CTRL);
	cl_dvfs_wmb(cld);
}

static int cl_dvfs_enable_clocks(struct tegra_cl_dvfs *cld)
{
	if (cld->p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C) {
		clk_enable(cld->i2c_clk);
		clk_enable(cld->i2c_fast);
	}
	clk_enable(cld->ref_clk);
	clk_enable(cld->soc_clk);
	return 0;
}

static void cl_dvfs_disable_clocks(struct tegra_cl_dvfs *cld)
{
	if (cld->p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C) {
		clk_disable(cld->i2c_clk);
		clk_disable(cld->i2c_fast);
	}
	clk_disable(cld->ref_clk);
	clk_disable(cld->soc_clk);
}

int __init tegra_init_cl_dvfs(struct tegra_cl_dvfs *cld)
{
	int ret;

	/* Check platform and SoC data, get i2c clock */
	if (!cld->dfll_data) {
		pr_err("%s: SoC tuning data is not available\n", __func__);
		return -EINVAL;
	}
	if (!cld->p_data || !cld->p_data->cfg_param) {
		pr_err("%s: Platform data is not available\n", __func__);
		return -EINVAL;
	}
	if (cld->p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C) {
		ret = clk_enable(cld->i2c_clk);
		if (ret) {
			pr_err("%s: Failed to enable %s\n",
			       __func__, cld->i2c_clk->name);
			return ret;
		}
		ret = clk_enable(cld->i2c_fast);
		if (ret) {
			pr_err("%s: Failed to enable %s\n",
			       __func__, cld->i2c_clk->name);
			return ret;
		}
		cld->i2c_rate = clk_get_rate(cld->i2c_clk);
	} else {
		pr_err("%s: PMU interface is not I2C\n", __func__);
		return -EINVAL;
	}

	/* Enable clocks, release control logic reset */
	ret = clk_enable(cld->ref_clk);
	if (ret) {
		pr_err("%s: Failed to enable %s\n",
		       __func__, cld->ref_clk->name);
		return ret;
	}
	ret = clk_enable(cld->soc_clk);
	if (ret) {
		pr_err("%s: Failed to enable %s\n",
		       __func__, cld->ref_clk->name);
		return ret;
	}
	cld->ref_rate = clk_get_rate(cld->ref_clk);
	BUG_ON(!cld->ref_rate);

	/* Get ready ouput voltage mapping*/
	cl_dvfs_init_maps(cld);

	/* Setup PMU interface */
	cl_dvfs_init_out_if(cld);

	/* Configure control registers in disabled mode and disable clocks */
	cl_dvfs_init_cntrl_logic(cld);
	cl_dvfs_disable_clocks(cld);

	return 0;
}

void tegra_cl_dvfs_set_plarform_data(struct tegra_cl_dvfs_platform_data *data)
{
	struct clk *c = tegra_get_clock_by_name(data->dfll_clk_name);
	if (c && c->u.dfll.cl_dvfs)
		c->u.dfll.cl_dvfs->p_data = data;
}

void tegra_cl_dvfs_set_dfll_data(struct tegra_cl_dvfs_dfll_data *data)
{
	struct clk *c = tegra_get_clock_by_name(data->dfll_clk_name);
	if (c && c->u.dfll.cl_dvfs)
		c->u.dfll.cl_dvfs->dfll_data = data;
}

/*
 * CL_DVFS states:
 *
 * - DISABLED: control logic mode - DISABLED, output interface disabled,
 *   dfll in reset
 * - OPEN_LOOP: control logic mode - OPEN_LOOP, output interface disabled,
 *   dfll is running "unlocked"
 * - CLOSED_LOOP: control logic mode - CLOSED_LOOP, output interface enabled,
 *   dfll is running "locked"
 */

/* Switch from any other state to DISABLED state */
void tegra_cl_dvfs_disable(struct tegra_cl_dvfs *cld)
{
	if ((cld->mode == TEGRA_CL_DVFS_UNINITIALIZED) ||
	    (cld->mode == TEGRA_CL_DVFS_DISABLED))
		return;

	output_enable(cld, false);
	set_mode(cld, TEGRA_CL_DVFS_DISABLED);
	cl_dvfs_disable_clocks(cld);
}

/* Switch from DISABLE state to OPEN_LOOP state */
int tegra_cl_dvfs_enable(struct tegra_cl_dvfs *cld)
{
	if (cld->mode == TEGRA_CL_DVFS_UNINITIALIZED) {
		pr_err("%s: Cannot enable DFLL in %s mode\n",
		       __func__, mode_name[cld->mode]);
		return -EPERM;
	}

	if (cld->mode != TEGRA_CL_DVFS_DISABLED)
		return 0;

	cl_dvfs_enable_clocks(cld);
	set_mode(cld, TEGRA_CL_DVFS_OPEN_LOOP);
	return 0;
}

/* Switch from OPEN_LOOP state to CLOSED_LOOP state */
int tegra_cl_dvfs_lock(struct tegra_cl_dvfs *cld)
{
	u32 val;
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
		val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);
		val &= ~CL_DVFS_OUTPUT_CFG_SAFE_MASK;
		val |= req->output << CL_DVFS_OUTPUT_CFG_SAFE_SHIFT;
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
		cld->safe_ouput = req->output;

		val = req->freq << CL_DVFS_FREQ_REQ_FREQ_SHIFT;
		val |= req->scale << CL_DVFS_FREQ_REQ_SCALE_SHIFT;
		val |= CL_DVFS_FREQ_REQ_FREQ_VALID |
			CL_DVFS_FREQ_REQ_FORCE_ENABLE;
		cl_dvfs_writel(cld, val, CL_DVFS_FREQ_REQ);

		output_enable(cld, true);
		set_mode(cld, TEGRA_CL_DVFS_CLOSED_LOOP);
		return 0;

	default:
		BUG_ON(cld->mode > TEGRA_CL_DVFS_CLOSED_LOOP);
		pr_err("%s: Cannot lock DFLL in %s mode\n",
		       __func__, mode_name[cld->mode]);
		return -EPERM;
	}
}

/* Switch from CLOSED_LOOP state to OPEN_LOOP state */
int tegra_cl_dvfs_unlock(struct tegra_cl_dvfs *cld)
{
	switch (cld->mode) {
	case TEGRA_CL_DVFS_CLOSED_LOOP:
		output_enable(cld, false);
		set_mode(cld, TEGRA_CL_DVFS_OPEN_LOOP);
		return 0;

	case TEGRA_CL_DVFS_OPEN_LOOP:
		return 0;

	default:
		BUG_ON(cld->mode > TEGRA_CL_DVFS_CLOSED_LOOP);
		pr_err("%s: Cannot unlock DFLL in %s mode\n",
		       __func__, mode_name[cld->mode]);
		return -EPERM;
	}
}
/*
 * Convert requested rate into the control logic settings. In CLOSED_LOOP mode,
 * update new settings immediately to adjust DFLL output rate accordingly.
 * Otherwise, just save them until next switch to closed loop.
 */
int tegra_cl_dvfs_request_rate(struct tegra_cl_dvfs *cld, unsigned long rate)
{
	u32 val;
	struct dfll_rate_req req;

	if (cld->mode == TEGRA_CL_DVFS_UNINITIALIZED) {
		pr_err("%s: Cannot set DFLL rate in %s mode\n",
		       __func__, mode_name[cld->mode]);
		return -EPERM;
	}

	/* Determine DFLL output scale */
	req.scale = SCALE_MAX - 1;
	if (rate < cld->dfll_rate_min) {
		req.scale = rate / 1000 * SCALE_MAX /
			(cld->dfll_rate_min / 1000);
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
	rate = GET_REQUEST_RATE(val, cld->ref_rate);

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
		int coef = cld->p_data->cfg_param->cg_scale ? 128 : 16;
		force_val = force_val * coef / cld->p_data->cfg_param->cg;
		force_val = clamp(force_val, FORCE_MIN, FORCE_MAX);
		val |= ((u32)force_val << CL_DVFS_FREQ_REQ_FORCE_SHIFT) &
					CL_DVFS_FREQ_REQ_FORCE_MASK;

		val |= req.scale << CL_DVFS_FREQ_REQ_SCALE_SHIFT;
		val |= CL_DVFS_FREQ_REQ_FREQ_VALID |
			CL_DVFS_FREQ_REQ_FORCE_ENABLE;
		cl_dvfs_writel(cld, val, CL_DVFS_FREQ_REQ);
		cl_dvfs_wmb(cld);
	}
	return 0;
}

unsigned long tegra_cl_dvfs_request_get(struct tegra_cl_dvfs *cld)
{
	struct dfll_rate_req *req = &cld->last_req;
	u32 rate = GET_REQUEST_RATE(req->freq, cld->ref_rate);
	if ((req->scale + 1) < SCALE_MAX) {
		rate = DIV_ROUND_UP(rate / 1000 * (req->scale + 1), SCALE_MAX);
		rate *= 1000;
	}
	return rate;
}

#ifdef CONFIG_DEBUG_FS

static int lock_get(void *data, u64 *val)
{
	struct clk *c = (struct clk *)data;
	*val = c->u.dfll.cl_dvfs->mode == TEGRA_CL_DVFS_CLOSED_LOOP;
	return 0;
}
static int lock_set(void *data, u64 val)
{
	struct clk *c = (struct clk *)data;
	return tegra_clk_cfg_ex(c, TEGRA_CLK_DFLL_LOCK, val);
}
DEFINE_SIMPLE_ATTRIBUTE(lock_fops, lock_get, lock_set, "%llu\n");

static int monitor_get(void *data, u64 *val)
{
	struct tegra_cl_dvfs *cld = ((struct clk *)data)->u.dfll.cl_dvfs;
	*val = cl_dvfs_readl(cld, CL_DVFS_MONITOR_DATA);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(monitor_fops, monitor_get, NULL, "%llu\n");


static int cl_register_show(struct seq_file *s, void *data)
{
	u32 offs;
	struct clk *c = s->private;
	struct tegra_cl_dvfs *cld = c->u.dfll.cl_dvfs;

	seq_printf(s, "CONTROL REGISTERS:\n");
	for (offs = 0; offs <= CL_DVFS_MONITOR_CTRL; offs += 4)
		seq_printf(s, "[0x%02x] = 0x%08x\n",
			   offs, cl_dvfs_readl(cld, offs));

	seq_printf(s, "\nI2C and INTR REGISTERS:\n");
	for (offs = CL_DVFS_I2C_CFG; offs <= CL_DVFS_I2C_STS; offs += 4)
		seq_printf(s, "[0x%02x] = 0x%08x\n",
			   offs, cl_dvfs_readl(cld, offs));

	offs = CL_DVFS_INTR_STS;
	seq_printf(s, "[0x%02x] = 0x%08x\n", offs, cl_dvfs_readl(cld, offs));
	offs = CL_DVFS_INTR_EN;
	seq_printf(s, "[0x%02x] = 0x%08x\n", offs, cl_dvfs_readl(cld, offs));
	offs = CL_DVFS_I2C_CLK_DIVISOR;
	seq_printf(s, "[0x%02x] = 0x%08x\n", offs, cl_dvfs_readl(cld, offs));

	seq_printf(s, "\nLUT:\n");
	for (offs = CL_DVFS_OUTPUT_LUT;
	     offs < CL_DVFS_OUTPUT_LUT + 4 * MAX_CL_DVFS_VOLTAGES;
	     offs += 4)
		seq_printf(s, "[0x%02x] = 0x%08x\n",
			   offs, cl_dvfs_readl(cld, offs));

	return 0;
}

static int cl_register_open(struct inode *inode, struct file *file)
{
	return single_open(file, cl_register_show, inode->i_private);
}

static ssize_t cl_register_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[80];
	u32 offs;
	u32 val;
	struct clk *c = file->f_path.dentry->d_inode->i_private;
	struct tegra_cl_dvfs *cld = c->u.dfll.cl_dvfs;

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (sscanf(buf, "[0x%x] = 0x%x", &offs, &val) != 2)
		return -1;

	cl_dvfs_writel(cld, val, offs & (~0x3));
	return count;
}

static const struct file_operations cl_register_fops = {
	.open		= cl_register_open,
	.read		= seq_read,
	.write		= cl_register_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_cl_dvfs_debug_init(void)
{
	struct dentry *cpu_cl_dvfs_dentry;
	struct clk *dfll_cpu = tegra_get_clock_by_name("dfll_cpu");

	if (!dfll_cpu || !dfll_cpu->dent || (dfll_cpu->state == UNINITIALIZED))
		return 0;

	if (!debugfs_create_file("lock", S_IRUGO | S_IWUSR,
		dfll_cpu->dent, dfll_cpu, &lock_fops))
		goto err_out;

	cpu_cl_dvfs_dentry = debugfs_create_dir("cl_dvfs", dfll_cpu->dent);
	if (!cpu_cl_dvfs_dentry)
		goto err_out;

	if (!debugfs_create_file("monitor", S_IRUGO,
		cpu_cl_dvfs_dentry, dfll_cpu, &monitor_fops))
		goto err_out;

	if (!debugfs_create_file("registers", S_IRUGO | S_IWUSR,
		cpu_cl_dvfs_dentry, dfll_cpu, &cl_register_fops))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(dfll_cpu->dent);
	return -ENOMEM;
}

late_initcall(tegra_cl_dvfs_debug_init);
#endif
