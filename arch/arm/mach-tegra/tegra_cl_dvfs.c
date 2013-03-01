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
#include <linux/platform_device.h>

#include <mach/irqs.h>
#include <mach/hardware.h>

#include "tegra_cl_dvfs.h"
#include "clock.h"
#include "dvfs.h"
#include "iomap.h"

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
#define CL_DVFS_MONITOR_CTRL_FREQ	6
#define CL_DVFS_MONITOR_DATA		0x2c
#define CL_DVFS_MONITOR_DATA_NEW	(0x1 << 16)
#define CL_DVFS_MONITOR_DATA_MASK	0xFFFF

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
#define CL_DVFS_I2C_STS_I2C_LAST_SHIFT	1
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

#define CL_DVFS_CALIBR_TIME		40000
#define CL_DVFS_OUTPUT_PENDING_TIMEOUT	1000
#define CL_DVFS_OUTPUT_RAMP_DELAY	100
#define CL_DVFS_TUNE_HIGH_DELAY		2000

#define CL_DVFS_TUNE_HIGH_MARGIN_STEPS	2

#define CL_DVFS_DYNAMIC_OUTPUT_CFG	0

enum tegra_cl_dvfs_ctrl_mode {
	TEGRA_CL_DVFS_UNINITIALIZED = 0,
	TEGRA_CL_DVFS_DISABLED = 1,
	TEGRA_CL_DVFS_OPEN_LOOP = 2,
	TEGRA_CL_DVFS_CLOSED_LOOP = 3,
};

enum tegra_cl_dvfs_tune_state {
	TEGRA_CL_DVFS_TUNE_LOW = 0,
	TEGRA_CL_DVFS_TUNE_HIGH_REQUEST,
	TEGRA_CL_DVFS_TUNE_HIGH,
};

struct dfll_rate_req {
	u8	freq;
	u8	scale;
	u8	output;
	u8	cap;
};

struct tegra_cl_dvfs {
	void					*cl_base;
	struct tegra_cl_dvfs_platform_data	*p_data;

	struct dvfs			*safe_dvfs;
	struct tegra_cooling_device	*cdev;
	struct work_struct		init_cdev_work;

	struct clk			*soc_clk;
	struct clk			*ref_clk;
	struct clk			*i2c_clk;
	struct clk			*dfll_clk;
	unsigned long			ref_rate;
	unsigned long			i2c_rate;

	/* output voltage mapping:
	 * legacy dvfs table index -to- cl_dvfs output LUT index
	 * cl_dvfs output LUT index -to- PMU value/voltage pair ptr
	 */
	u8				clk_dvfs_map[MAX_DVFS_FREQS];
	struct voltage_reg_map		*out_map[MAX_CL_DVFS_VOLTAGES];
	u8				num_voltages;
	u8				safe_output;
	u8				tune_high_out_start;
	u8				tune_high_out_min;
	u8				cold_out_min;
	u8				minimax_output;
	unsigned long			dvco_rate_min;

	u8				lut_min;
	u8				lut_max;
	int				thermal_idx;
	struct dfll_rate_req		last_req;
	enum tegra_cl_dvfs_tune_state	tune_state;
	enum tegra_cl_dvfs_ctrl_mode	mode;

	struct timer_list		tune_timer;
	unsigned long			tune_delay;
	struct timer_list		calibration_timer;
	unsigned long			calibration_delay;
	ktime_t				last_calibration;

};

/* Conversion macros (different scales for frequency request, and monitored
   rate is not a typo) */
#define RATE_STEP(cld)				((cld)->ref_rate / 2)
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

static inline int output_enable(struct tegra_cl_dvfs *cld)
{
	u32 val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);

	/* FIXME: PWM output control */
	val |= CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
	cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb(cld);
	return  0;
}

static noinline int output_flush_disable(struct tegra_cl_dvfs *cld)
{
	int i;
	u32 sts;
	u32 val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);

	/* Flush transactions in flight, and then disable */
	for (i = 0; i < CL_DVFS_OUTPUT_PENDING_TIMEOUT / 2; i++) {
		sts = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
		udelay(2);
		if (!(sts & CL_DVFS_I2C_STS_I2C_REQ_PENDING)) {
			sts = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
			if (!(sts & CL_DVFS_I2C_STS_I2C_REQ_PENDING)) {
				val &= ~CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
				cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
				wmb();
				sts = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
				if (!(sts & CL_DVFS_I2C_STS_I2C_REQ_PENDING))
					return 0; /* no pending rqst */

				/* Re-enable, continue wait */
				val |= CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
				cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
				wmb();
			}
		}
	}

	/* I2C request is still pending - disable, anyway, but report error */
	val &= ~CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
	cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb(cld);
	return -ETIMEDOUT;
}

static noinline int output_disable_flush(struct tegra_cl_dvfs *cld)
{
	int i;
	u32 sts;
	u32 val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);

	/* Disable output interface right away */
	val &= ~CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
	cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb(cld);

	/* Flush possible transaction in flight */
	for (i = 0; i < CL_DVFS_OUTPUT_PENDING_TIMEOUT / 2; i++) {
		sts = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
		udelay(2);
		if (!(sts & CL_DVFS_I2C_STS_I2C_REQ_PENDING)) {
			sts = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
			if (!(sts & CL_DVFS_I2C_STS_I2C_REQ_PENDING))
				return 0;
		}
	}

	/* I2C request is still pending - report error */
	return -ETIMEDOUT;
}

static inline int output_disable_ol_prepare(struct tegra_cl_dvfs *cld)
{
	/* FIXME: PWM output control */
	/*
	 * If cl-dvfs h/w does not require output to be quiet before disable,
	 * s/w can stop I2C communications at any time (including operations
	 * in closed loop mode), and I2C bus integrity is guaranteed even in
	 * case of flush timeout.
	 */
	if (!cld->p_data->out_quiet_then_disable) {
		int ret = output_disable_flush(cld);
		if (ret)
			pr_debug("cl_dvfs: I2C pending timeout ol_prepare\n");
		return ret;
	}
	return 0;
}

static inline int output_disable_post_ol(struct tegra_cl_dvfs *cld)
{
	/* FIXME: PWM output control */
	/*
	 * If cl-dvfs h/w requires output to be quiet before disable, s/w
	 * should stop I2C communications only after the switch to open loop
	 * mode, and I2C bus integrity is not guaranteed in case of flush
	 * timeout
	*/
	if (cld->p_data->out_quiet_then_disable) {
		int ret = output_flush_disable(cld);
		if (ret)
			pr_err("cl_dvfs: I2C pending timeout post_ol\n");
		return ret;
	}
	return 0;
}

static inline void set_mode(struct tegra_cl_dvfs *cld,
			    enum tegra_cl_dvfs_ctrl_mode mode)
{
	cld->mode = mode;
	cl_dvfs_writel(cld, mode - 1, CL_DVFS_CTRL);
	cl_dvfs_wmb(cld);
}

static inline u8 get_output_min(struct tegra_cl_dvfs *cld)
{
	u32 tune_min, thermal_min;

	tune_min = cld->tune_state == TEGRA_CL_DVFS_TUNE_LOW ?
		0 : cld->tune_high_out_min;
	thermal_min = cld->thermal_idx ? 0 : cld->cold_out_min;

	return max(tune_min, thermal_min);
}

static inline void _load_lut(struct tegra_cl_dvfs *cld)
{
	int i;
	u32 val;

	val = cld->out_map[cld->lut_min]->reg_value;
	for (i = 0; i <= cld->lut_min; i++)
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_LUT + i * 4);

	for (; i < cld->lut_max; i++) {
		val = cld->out_map[i]->reg_value;
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_LUT + i * 4);
	}

	val = cld->out_map[cld->lut_max]->reg_value;
	for (; i < cld->num_voltages; i++)
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_LUT + i * 4);

	cl_dvfs_wmb(cld);
}

static void cl_dvfs_load_lut(struct tegra_cl_dvfs *cld)
{
	u32 val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);
	bool disable_out_for_load = !cld->p_data->out_quiet_then_disable &&
		(val & CL_DVFS_OUTPUT_CFG_I2C_ENABLE);

	if (disable_out_for_load) {
		val &= ~CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
		cl_dvfs_wmb(cld);
		udelay(2); /* 2us (big margin) window for disable propafation */
	}

	_load_lut(cld);

	if (disable_out_for_load) {
		val |= CL_DVFS_OUTPUT_CFG_I2C_ENABLE;
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
		cl_dvfs_wmb(cld);
	}
}

#define set_tune_state(cld, state) \
	do {								\
		cld->tune_state = state;				\
		pr_debug("%s: set tune state %d\n", __func__, state);	\
	} while (0)

static inline void tune_low(struct tegra_cl_dvfs *cld)
{
	if (cld->safe_dvfs->dfll_data.tune_trimmers)
		cld->safe_dvfs->dfll_data.tune_trimmers(false);
	cl_dvfs_writel(cld, cld->safe_dvfs->dfll_data.tune0, CL_DVFS_TUNE0);
	cl_dvfs_wmb(cld);
}

static inline void tune_high(struct tegra_cl_dvfs *cld)
{
	cl_dvfs_writel(cld, cld->safe_dvfs->dfll_data.tune0_high_mv,
		       CL_DVFS_TUNE0);
	cl_dvfs_wmb(cld);
	if (cld->safe_dvfs->dfll_data.tune_trimmers)
		cld->safe_dvfs->dfll_data.tune_trimmers(true);
}

static void set_ol_config(struct tegra_cl_dvfs *cld)
{
	u32 val, out_min;

	/* always tune low (safe) in open loop */
	if (cld->tune_state != TEGRA_CL_DVFS_TUNE_LOW) {
		set_tune_state(cld, TEGRA_CL_DVFS_TUNE_LOW);
		tune_low(cld);

		out_min = get_output_min(cld);
#if CL_DVFS_DYNAMIC_OUTPUT_CFG
		val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);
		val &= ~CL_DVFS_OUTPUT_CFG_MIN_MASK;
		val |= out_min << CL_DVFS_OUTPUT_CFG_MIN_SHIFT;
		cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
#else
		if (cld->lut_min != out_min) {
			cld->lut_min = out_min;
			cl_dvfs_load_lut(cld);
		}
#endif
	}

	/* 1:1 scaling in open loop */
	val = cl_dvfs_readl(cld, CL_DVFS_FREQ_REQ);
	val |= (SCALE_MAX - 1) << CL_DVFS_FREQ_REQ_SCALE_SHIFT;
	val &= ~CL_DVFS_FREQ_REQ_FORCE_ENABLE;
	cl_dvfs_writel(cld, val, CL_DVFS_FREQ_REQ);
}

static void set_cl_config(struct tegra_cl_dvfs *cld, struct dfll_rate_req *req)
{
#if CL_DVFS_DYNAMIC_OUTPUT_CFG
	u32 val;
#endif
	u32 out_max, out_min;

	switch (cld->tune_state) {
	case TEGRA_CL_DVFS_TUNE_LOW:
		if (req->cap > cld->tune_high_out_start) {
			set_tune_state(cld, TEGRA_CL_DVFS_TUNE_HIGH_REQUEST);
			mod_timer(&cld->tune_timer, jiffies + cld->tune_delay);
		}
		break;

	case TEGRA_CL_DVFS_TUNE_HIGH:
	case TEGRA_CL_DVFS_TUNE_HIGH_REQUEST:
		if (req->cap <= cld->tune_high_out_start) {
			set_tune_state(cld, TEGRA_CL_DVFS_TUNE_LOW);
			tune_low(cld);
		}
		break;
	default:
		BUG();
	}

	out_min = get_output_min(cld);
	if (req->cap > (out_min + 1))
		req->output = req->cap - 1;
	else
		req->output = out_min + 1;
	if (req->output == cld->safe_output)
		req->output++;
	out_max = max((u8)(req->output + 1), cld->minimax_output);

#if CL_DVFS_DYNAMIC_OUTPUT_CFG
	val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);
	val &= ~(CL_DVFS_OUTPUT_CFG_MAX_MASK | CL_DVFS_OUTPUT_CFG_MIN_MASK);
	val |= out_max << CL_DVFS_OUTPUT_CFG_MAX_SHIFT;
	val |= out_min << CL_DVFS_OUTPUT_CFG_MIN_SHIFT;
	cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
#else
	if ((cld->lut_min != out_min) || (cld->lut_max != out_max)) {
		cld->lut_min = out_min;
		cld->lut_max = out_max;
		cl_dvfs_load_lut(cld);
	}
#endif
}

static void tune_timer_cb(unsigned long data)
{
	unsigned long flags;
	u32 val, out_min, out_last;
	struct tegra_cl_dvfs *cld = (struct tegra_cl_dvfs *)data;

	clk_lock_save(cld->dfll_clk, &flags);

	/* FIXME: PWM output control */
	if (cld->tune_state == TEGRA_CL_DVFS_TUNE_HIGH_REQUEST) {
#if CL_DVFS_DYNAMIC_OUTPUT_CFG
		val = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);
		out_min = (val >> CL_DVFS_OUTPUT_CFG_MIN_SHIFT) & OUT_MASK;
#else
		out_min = cld->lut_min;
#endif
		val = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
		out_last = (val >> CL_DVFS_I2C_STS_I2C_LAST_SHIFT) & OUT_MASK;

		if (!(val & CL_DVFS_I2C_STS_I2C_REQ_PENDING) &&
		    (out_last >= cld->tune_high_out_min)  &&
		    (out_min >= cld->tune_high_out_min)) {
			udelay(CL_DVFS_OUTPUT_RAMP_DELAY);
			set_tune_state(cld, TEGRA_CL_DVFS_TUNE_HIGH);
			tune_high(cld);
		} else {
			mod_timer(&cld->tune_timer, jiffies + cld->tune_delay);
		}
	}
	clk_unlock_restore(cld->dfll_clk, &flags);
}

static void cl_dvfs_calibrate(struct tegra_cl_dvfs *cld)
{
	u32 val;
	ktime_t now;
	unsigned long data;

	static unsigned long rate_min_low_limit;
	static unsigned long rate_min_high_limit;

	/* Initial dvco min rate is under-estimated - skewed range up */
	if (!rate_min_low_limit) {
		rate_min_low_limit = cld->dvco_rate_min - 2 * RATE_STEP(cld);
		rate_min_high_limit = cld->dvco_rate_min + 6 * RATE_STEP(cld);
	}

	/*
	 *  Enter calibration procedure only if
	 *  - closed loop operations
	 *  - last request engaged clock skipper
	 *  - unrestricted Vmin
	 *  - at least specified time after the last calibration attempt
	 */
	if ((cld->mode != TEGRA_CL_DVFS_CLOSED_LOOP) ||
	    (cld->last_req.scale == (SCALE_MAX - 1)) || get_output_min(cld))
		return;

	now = ktime_get();
	if (ktime_us_delta(now, cld->last_calibration) < CL_DVFS_CALIBR_TIME)
		return;
	cld->last_calibration = now;

	if (cl_dvfs_readl(cld, CL_DVFS_MONITOR_CTRL) !=
	    CL_DVFS_MONITOR_CTRL_FREQ)
		cl_dvfs_writel(cld, CL_DVFS_MONITOR_CTRL_FREQ,
				CL_DVFS_MONITOR_CTRL);

	/* Synchronize with sample period, and get rate measurements */
	data = cl_dvfs_readl(cld, CL_DVFS_MONITOR_DATA);
	do {
		data = cl_dvfs_readl(cld, CL_DVFS_MONITOR_DATA);
	} while (!(data & CL_DVFS_MONITOR_DATA_NEW));
	do {
		data = cl_dvfs_readl(cld, CL_DVFS_MONITOR_DATA);
	} while (!(data & CL_DVFS_MONITOR_DATA_NEW));

	/* Skip calibration if I2C transaction is pending */
	/* FIXME: PWM output control */
	val = cl_dvfs_readl(cld, CL_DVFS_I2C_STS);
	if (val & CL_DVFS_I2C_STS_I2C_REQ_PENDING)
		return;

	/* Adjust minimum rate */
	data &= CL_DVFS_MONITOR_DATA_MASK;
	data = GET_MONITORED_RATE(data, cld->ref_rate);
	if (val || (data < (cld->dvco_rate_min - RATE_STEP(cld))))
		cld->dvco_rate_min -= RATE_STEP(cld);
	else if (data > (cld->dvco_rate_min + RATE_STEP(cld)))
		cld->dvco_rate_min += RATE_STEP(cld);
	else
		return;

	cld->dvco_rate_min = clamp(cld->dvco_rate_min,
				   rate_min_low_limit, rate_min_high_limit);
	mod_timer(&cld->calibration_timer, jiffies + cld->calibration_delay);
	pr_debug("%s: calibrated dvco_rate_min %lu\n",
		 __func__, cld->dvco_rate_min);
}

static void calibration_timer_cb(unsigned long data)
{
	unsigned long flags;
	struct tegra_cl_dvfs *cld = (struct tegra_cl_dvfs *)data;

	clk_lock_save(cld->dfll_clk, &flags);
	cl_dvfs_calibrate(cld);
	clk_unlock_restore(cld->dfll_clk, &flags);
}

static void set_request(struct tegra_cl_dvfs *cld, struct dfll_rate_req *req)
{
	u32 val;
	int force_val = req->output - cld->safe_output;
	int coef = 128; /* FIXME: cld->p_data->cfg_param->cg_scale? */;

	force_val = force_val * coef / cld->p_data->cfg_param->cg;
	force_val = clamp(force_val, FORCE_MIN, FORCE_MAX);

	val = req->freq << CL_DVFS_FREQ_REQ_FREQ_SHIFT;
	val |= req->scale << CL_DVFS_FREQ_REQ_SCALE_SHIFT;
	val |= ((u32)force_val << CL_DVFS_FREQ_REQ_FORCE_SHIFT) &
		CL_DVFS_FREQ_REQ_FORCE_MASK;
	val |= CL_DVFS_FREQ_REQ_FREQ_VALID | CL_DVFS_FREQ_REQ_FORCE_ENABLE;

	cl_dvfs_writel(cld, val, CL_DVFS_FREQ_REQ);
	cl_dvfs_wmb(cld);
}

static u8 find_mv_out_cap(struct tegra_cl_dvfs *cld, int mv)
{
	u8 cap;
	int uv;

	for (cap = 0; cap < cld->num_voltages; cap++) {
		uv = cld->out_map[cap]->reg_uV;
		if (uv >= mv * 1000)
			return cap;
	}
	return cap - 1;	/* maximum possible output */
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

	n = cld->safe_dvfs->num_freqs;
	BUG_ON(n >= MAX_CL_DVFS_VOLTAGES);

	millivolts = cld->safe_dvfs->dfll_millivolts;
	v_max = millivolts[n - 1];

	v = cld->safe_dvfs->dfll_data.min_millivolts;
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
	}
	BUG_ON(j > MAX_CL_DVFS_VOLTAGES);
	cld->num_voltages = j;
}

static void cl_dvfs_init_tuning_thresholds(struct tegra_cl_dvfs *cld)
{
	int mv;

	/*
	 * Convert high tuning voltage threshold into output LUT index, and
	 * add necessary margin.  If voltage threshold is outside operating
	 * range set it at maximum output level to effectively disable tuning
	 * parameters adjustment.
	 */
	cld->tune_high_out_min = cld->num_voltages - 1;
	cld->tune_high_out_start = cld->num_voltages - 1;
	mv = cld->safe_dvfs->dfll_data.tune_high_min_millivolts;
	if (mv >= cld->safe_dvfs->dfll_data.min_millivolts) {
		u8 out_min = find_mv_out_cap(cld, mv);
		if ((out_min + 2) < cld->num_voltages) {
			u8 out_start = out_min + CL_DVFS_TUNE_HIGH_MARGIN_STEPS;
			if (out_start < cld->num_voltages) {
				cld->tune_high_out_min = out_min;
				cld->tune_high_out_start = out_start;
				if (cld->minimax_output <= out_min)
					cld->minimax_output = out_min + 1;
			}
		}
	}
}

static void cl_dvfs_init_cold_output_floor(struct tegra_cl_dvfs *cld)
{
	/*
	 * Convert minimum voltage at low temperature into output LUT index;
	 * make sure there is room for regulation above cold minimum output
	 */
	cld->cold_out_min = find_mv_out_cap(
		cld, cld->safe_dvfs->dvfs_rail->min_millivolts_cold);
	BUG_ON(cld->cold_out_min + 2 >= cld->num_voltages);
	if (cld->minimax_output <= cld->cold_out_min)
		cld->minimax_output = cld->cold_out_min + 1;
}

static void cl_dvfs_init_output_thresholds(struct tegra_cl_dvfs *cld)
{
	cld->minimax_output = 0;
	cl_dvfs_init_tuning_thresholds(cld);
	cl_dvfs_init_cold_output_floor(cld);

	/* make sure safe output is safe at any temperature */
	cld->safe_output = cld->cold_out_min ? : 1;
	if (cld->minimax_output <= cld->safe_output)
		cld->minimax_output = cld->safe_output + 1;
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
	u32 val;

	/*
	 * Disable output, and set safe voltage and output limits;
	 * disable and clear limit interrupts.
	 */
	cld->tune_state = TEGRA_CL_DVFS_TUNE_LOW;
	cld->thermal_idx = 0;
#if CL_DVFS_DYNAMIC_OUTPUT_CFG
	val = get_output_min(cld);
	cld->lut_min = 0;
	cld->lut_max = cld->num_voltages - 1;
#else
	/*
	 * Allow the entire range of LUT indexes, but limit output voltage in
	 * LUT mapping (this "indirect" application of limits is used, because
	 * h/w does not support dynamic change of index limits, but dynamic
	 * reload of LUT is fine).
	 */
	val = 0;
	cld->lut_min = get_output_min(cld);
	cld->lut_max = cld->num_voltages - 1;
#endif

	val = (cld->safe_output << CL_DVFS_OUTPUT_CFG_SAFE_SHIFT) |
		((cld->num_voltages - 1) << CL_DVFS_OUTPUT_CFG_MAX_SHIFT) |
		(val << CL_DVFS_OUTPUT_CFG_MIN_SHIFT);
	cl_dvfs_writel(cld, val, CL_DVFS_OUTPUT_CFG);
	cl_dvfs_wmb(cld);

	cl_dvfs_writel(cld, 0, CL_DVFS_OUTPUT_FORCE);
	cl_dvfs_writel(cld, 0, CL_DVFS_INTR_EN);
	cl_dvfs_writel(cld, CL_DVFS_INTR_MAX_MASK | CL_DVFS_INTR_MIN_MASK,
		       CL_DVFS_INTR_STS);

	/* fill in LUT table */
	cl_dvfs_load_lut(cld);

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

	if (cld->safe_dvfs->dfll_data.tune_trimmers)
		cld->safe_dvfs->dfll_data.tune_trimmers(false);
	cl_dvfs_writel(cld, cld->safe_dvfs->dfll_data.tune0, CL_DVFS_TUNE0);
	cl_dvfs_writel(cld, cld->safe_dvfs->dfll_data.tune1, CL_DVFS_TUNE1);

	/* configure droop (skipper 1) and scale (skipper 2) */
	val = GET_DROOP_FREQ(cld->safe_dvfs->dfll_data.droop_rate_min,
			cld->ref_rate) << CL_DVFS_DROOP_CTRL_MIN_FREQ_SHIFT;
	BUG_ON(val > CL_DVFS_DROOP_CTRL_MIN_FREQ_MASK);
	val |= (param->droop_cut_value << CL_DVFS_DROOP_CTRL_CUT_SHIFT);
	val |= (param->droop_restore_ramp << CL_DVFS_DROOP_CTRL_RAMP_SHIFT);
	cl_dvfs_writel(cld, val, CL_DVFS_DROOP_CTRL);

	/* round minimum rate to request unit (ref_rate/2) boundary */
	cld->dvco_rate_min = cld->safe_dvfs->dfll_data.out_rate_min;
	cld->dvco_rate_min = ROUND_MIN_RATE(cld->dvco_rate_min, cld->ref_rate);

	cld->last_req.cap = 0;
	cld->last_req.freq = 0;
	cld->last_req.output = 0;
	cld->last_req.scale = SCALE_MAX - 1;
	cl_dvfs_writel(cld, CL_DVFS_FREQ_REQ_SCALE_MASK, CL_DVFS_FREQ_REQ);
	cl_dvfs_writel(cld, param->scale_out_ramp, CL_DVFS_SCALE_RAMP);

	/* select frequency for monitoring */
	cl_dvfs_writel(cld, CL_DVFS_MONITOR_CTRL_FREQ, CL_DVFS_MONITOR_CTRL);
	cl_dvfs_wmb(cld);
}

static int cl_dvfs_enable_clocks(struct tegra_cl_dvfs *cld)
{
	if (cld->p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C)
		clk_enable(cld->i2c_clk);

	clk_enable(cld->ref_clk);
	clk_enable(cld->soc_clk);
	return 0;
}

static void cl_dvfs_disable_clocks(struct tegra_cl_dvfs *cld)
{
	if (cld->p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C)
		clk_disable(cld->i2c_clk);

	clk_disable(cld->ref_clk);
	clk_disable(cld->soc_clk);
}

static int cl_dvfs_init(struct tegra_cl_dvfs *cld)
{
	int ret;

	/* Enable output inerface clock */
	if (cld->p_data->pmu_if == TEGRA_CL_DVFS_PMU_I2C) {
		ret = clk_enable(cld->i2c_clk);
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

	/* Enable module clocks, release control logic reset */
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

	/* init tuning timer */
	init_timer(&cld->tune_timer);
	cld->tune_timer.function = tune_timer_cb;
	cld->tune_timer.data = (unsigned long)cld;
	cld->tune_delay = usecs_to_jiffies(CL_DVFS_TUNE_HIGH_DELAY);

	/* init calibration timer */
	init_timer(&cld->calibration_timer);
	cld->calibration_timer.function = calibration_timer_cb;
	cld->calibration_timer.data = (unsigned long)cld;
	cld->calibration_delay = usecs_to_jiffies(CL_DVFS_CALIBR_TIME);

	/* Get ready ouput voltage mapping*/
	cl_dvfs_init_maps(cld);

	/* Setup output range thresholds */
	cl_dvfs_init_output_thresholds(cld);

	/* Setup PMU interface */
	cl_dvfs_init_out_if(cld);

	/* Configure control registers in disabled mode and disable clocks */
	cl_dvfs_init_cntrl_logic(cld);
	cl_dvfs_disable_clocks(cld);

	return 0;
}

/*
 * Re-initialize and enable target device clock in open loop mode. Called
 * directly from SoC clock resume syscore operation. Closed loop will be
 * re-entered in platform syscore ops as well.
 */
void tegra_cl_dvfs_resume(struct tegra_cl_dvfs *cld)
{
	enum tegra_cl_dvfs_ctrl_mode mode = cld->mode;
	struct dfll_rate_req req = cld->last_req;

	cl_dvfs_enable_clocks(cld);

	/* Setup PMU interface, and configure controls in disabled mode */
	cl_dvfs_init_out_if(cld);
	cl_dvfs_init_cntrl_logic(cld);

	cl_dvfs_disable_clocks(cld);

	/* Restore last request and mode */
	cld->last_req = req;
	if (mode != TEGRA_CL_DVFS_DISABLED) {
		set_mode(cld, TEGRA_CL_DVFS_OPEN_LOOP);
		WARN(mode > TEGRA_CL_DVFS_OPEN_LOOP,
		     "DFLL was left locked in suspend\n");
	}
}

#ifdef CONFIG_THERMAL
/* cl_dvfs cooling device */
static int tegra_cl_dvfs_get_cdev_max_state(struct thermal_cooling_device *cdev,
					    unsigned long *max_state)
{
	struct tegra_cl_dvfs *cld = (struct tegra_cl_dvfs *)cdev->devdata;
	*max_state = cld->cdev->trip_temperatures_num;
	return 0;
}

static int tegra_cl_dvfs_get_cdev_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long *cur_state)
{
	struct tegra_cl_dvfs *cld = (struct tegra_cl_dvfs *)cdev->devdata;
	*cur_state = cld->thermal_idx;
	return 0;
}

static int tegra_cl_dvfs_set_cdev_state(struct thermal_cooling_device *cdev,
					unsigned long cur_state)
{
	unsigned long flags;
	struct tegra_cl_dvfs *cld = (struct tegra_cl_dvfs *)cdev->devdata;

	clk_lock_save(cld->dfll_clk, &flags);

	if (cld->thermal_idx != cur_state) {
		cld->thermal_idx = cur_state;
		if (cld->mode == TEGRA_CL_DVFS_CLOSED_LOOP) {
			set_cl_config(cld, &cld->last_req);
			set_request(cld, &cld->last_req);
		}
	}
	clk_unlock_restore(cld->dfll_clk, &flags);
	return 0;
}

static struct thermal_cooling_device_ops tegra_cl_dvfs_cooling_ops = {
	.get_max_state = tegra_cl_dvfs_get_cdev_max_state,
	.get_cur_state = tegra_cl_dvfs_get_cdev_cur_state,
	.set_cur_state = tegra_cl_dvfs_set_cdev_state,
};

static void tegra_cl_dvfs_init_cdev(struct work_struct *work)
{
	struct tegra_cl_dvfs *cld = container_of(
		work, struct tegra_cl_dvfs, init_cdev_work);

	if (!cld->cdev)
		return;

	/* just report error - initialized at WC temperature, anyway */
	if (IS_ERR_OR_NULL(thermal_cooling_device_register(
		cld->cdev->cdev_type, (void *)cld,
		&tegra_cl_dvfs_cooling_ops))) {
		pr_err("tegra cooling device %s failed to register\n",
		       cld->cdev->cdev_type);
		return;
	}
	pr_info("%s cooling device is registered\n", cld->cdev->cdev_type);
}
#endif

#ifdef CONFIG_PM_SLEEP
/*
 * cl_dvfs controls clock/voltage to other devices, including CPU. Therefore,
 * cl_dvfs driver pm suspend callback does not stop cl-dvfs operations. It is
 * only used to enforce cold volatge limit, since SoC may cool down during
 * suspend without waking up. The correct temperature zone after supend will
 * be updated via cl_dvfs cooling device interface during resume of temperature
 * sensor.
 */
static int tegra_cl_dvfs_suspend_cl(struct device *dev)
{
	unsigned long flags;
	struct tegra_cl_dvfs *cld = dev_get_drvdata(dev);

	clk_lock_save(cld->dfll_clk, &flags);
	cld->thermal_idx = 0;
	if (cld->mode == TEGRA_CL_DVFS_CLOSED_LOOP) {
		set_cl_config(cld, &cld->last_req);
		set_request(cld, &cld->last_req);
	}
	clk_unlock_restore(cld->dfll_clk, &flags);

	return 0;
}

static const struct dev_pm_ops tegra_cl_dvfs_pm_ops = {
	.suspend = tegra_cl_dvfs_suspend_cl,
};
#endif

static int __init tegra_cl_dvfs_probe(struct platform_device *pdev)
{
	int ret;
	struct tegra_cl_dvfs_platform_data *p_data;
	struct resource *res;
	struct tegra_cl_dvfs *cld;
	struct clk *ref_clk, *soc_clk, *i2c_clk, *safe_dvfs_clk, *dfll_clk;

	/* Get resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing register base\n");
		return -ENOMEM;
	}

	p_data = pdev->dev.platform_data;
	if (!p_data || !p_data->cfg_param || !p_data->vdd_map) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENODATA;
	}

	ref_clk = clk_get(&pdev->dev, "ref");
	soc_clk = clk_get(&pdev->dev, "soc");
	i2c_clk = clk_get(&pdev->dev, "i2c");
	safe_dvfs_clk = clk_get(&pdev->dev, "safe_dvfs");
	dfll_clk = clk_get(&pdev->dev, p_data->dfll_clk_name);
	if (IS_ERR(ref_clk) || IS_ERR(soc_clk) || IS_ERR(i2c_clk)) {
		dev_err(&pdev->dev, "missing control clock\n");
		return -ENODEV;
	}
	if (IS_ERR(safe_dvfs_clk)) {
		dev_err(&pdev->dev, "missing safe dvfs source clock\n");
		return PTR_ERR(safe_dvfs_clk);
	}
	if (IS_ERR(dfll_clk)) {
		dev_err(&pdev->dev, "missing target dfll clock\n");
		return PTR_ERR(dfll_clk);
	}
	if (!safe_dvfs_clk->dvfs || !safe_dvfs_clk->dvfs->dvfs_rail) {
		dev_err(&pdev->dev, "invalid safe dvfs source\n");
		return -EINVAL;
	}

	/* Allocate cl_dvfs object and populate resource accessors */
	cld = kzalloc(sizeof(*cld), GFP_KERNEL);
	if (!cld) {
		dev_err(&pdev->dev, "failed to allocate cl_dvfs object\n");
		return -ENOMEM;
	}

	cld->cl_base = IO_ADDRESS(res->start);
	cld->p_data = p_data;
	cld->ref_clk = ref_clk;
	cld->soc_clk = soc_clk;
	cld->i2c_clk = i2c_clk;
	cld->dfll_clk = dfll_clk;
	cld->safe_dvfs = safe_dvfs_clk->dvfs;
#ifdef CONFIG_THERMAL
	cld->cdev = cld->safe_dvfs->dvfs_rail->dfll_mode_cdev;
	INIT_WORK(&cld->init_cdev_work, tegra_cl_dvfs_init_cdev);
#endif
	/* Initialize cl_dvfs */
	ret = cl_dvfs_init(cld);
	if (ret) {
		kfree(cld);
		return ret;
	}

	platform_set_drvdata(pdev, cld);

	/*
	 * Schedule cooling device registration as a separate work to address
	 * the following race: when cl_dvfs is probed the DFLL child clock
	 * (e.g., CPU) cannot be changed; on the other hand cooling device
	 * registration will update the entire thermal zone, and may trigger
	 * rate change of the target clock
	 */
	if (cld->cdev)
		schedule_work(&cld->init_cdev_work);
	return 0;
}

static struct platform_driver tegra_cl_dvfs_driver = {
	.driver         = {
		.name   = "tegra_cl_dvfs",
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm = &tegra_cl_dvfs_pm_ops,
#endif
	},
};

int __init tegra_init_cl_dvfs(void)
{
	return platform_driver_probe(&tegra_cl_dvfs_driver,
				     tegra_cl_dvfs_probe);
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
	switch (cld->mode) {
	case TEGRA_CL_DVFS_CLOSED_LOOP:
		WARN(1, "DFLL is disabled directly from closed loop mode\n");
		set_ol_config(cld);
		output_disable_ol_prepare(cld);
		set_mode(cld, TEGRA_CL_DVFS_DISABLED);
		output_disable_post_ol(cld);
		cl_dvfs_disable_clocks(cld);
		return;

	case TEGRA_CL_DVFS_OPEN_LOOP:
		set_mode(cld, TEGRA_CL_DVFS_DISABLED);
		cl_dvfs_disable_clocks(cld);
		return;

	default:
		BUG_ON(cld->mode > TEGRA_CL_DVFS_CLOSED_LOOP);
		return;
	}
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
	struct dfll_rate_req *req = &cld->last_req;

	switch (cld->mode) {
	case TEGRA_CL_DVFS_CLOSED_LOOP:
		return 0;

	case TEGRA_CL_DVFS_OPEN_LOOP:
		if (req->freq == 0) {
			pr_err("%s: Cannot lock DFLL at rate 0\n", __func__);
			return -EINVAL;
		}

		/*
		 * Update control logic setting with last rate request;
		 * sync output limits with current tuning and thermal state,
		 * enable output and switch to closed loop mode.
		 */
		set_cl_config(cld, req);
		output_enable(cld);
		set_mode(cld, TEGRA_CL_DVFS_CLOSED_LOOP);
		set_request(cld, req);
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
	int ret;

	switch (cld->mode) {
	case TEGRA_CL_DVFS_CLOSED_LOOP:
		set_ol_config(cld);
		ret = output_disable_ol_prepare(cld);
		set_mode(cld, TEGRA_CL_DVFS_OPEN_LOOP);
		if (!ret)
			ret = output_disable_post_ol(cld);
		return ret;

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

	/* Calibrate dfll minimum rate */
	cl_dvfs_calibrate(cld);

	/* Determine DFLL output scale */
	req.scale = SCALE_MAX - 1;
	if (rate < cld->dvco_rate_min) {
		req.scale = DIV_ROUND_UP((rate / 1000 * SCALE_MAX),
			(cld->dvco_rate_min / 1000));
		if (!req.scale) {
			pr_err("%s: Rate %lu is below scalable range\n",
			       __func__, rate);
			return -EINVAL;
		}
		req.scale--;
		rate = cld->dvco_rate_min;
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
	req.cap = req.output;

	/*
	 * Save validated request, and in CLOSED_LOOP mode actually update
	 * control logic settings; use request output to set maximum voltage
	 * limit, but keep one LUT step room above safe voltage
	 */
	cld->last_req = req;

	if (cld->mode == TEGRA_CL_DVFS_CLOSED_LOOP) {
		set_cl_config(cld, &cld->last_req);
		set_request(cld, &cld->last_req);
	}
	return 0;
}

unsigned long tegra_cl_dvfs_request_get(struct tegra_cl_dvfs *cld)
{
	struct dfll_rate_req *req = &cld->last_req;
	u32 rate = GET_REQUEST_RATE(req->freq, cld->ref_rate);
	if ((req->scale + 1) < SCALE_MAX) {
		rate = (rate / 1000 * (req->scale + 1)) / SCALE_MAX;
		rate *= 1000;
	}
	return rate;
}

#ifdef CONFIG_DEBUG_FS

static int lock_get(void *data, u64 *val)
{
	struct tegra_cl_dvfs *cld = ((struct clk *)data)->u.dfll.cl_dvfs;
	*val = cld->mode == TEGRA_CL_DVFS_CLOSED_LOOP;
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
	u32 v, s;
	unsigned long flags;
	struct clk *c = (struct clk *)data;
	struct tegra_cl_dvfs *cld = ((struct clk *)data)->u.dfll.cl_dvfs;

	clk_enable(cld->soc_clk);

	clk_lock_save(c, &flags);
	v = cl_dvfs_readl(cld, CL_DVFS_MONITOR_DATA) &
		CL_DVFS_MONITOR_DATA_MASK;

	if (cl_dvfs_readl(cld, CL_DVFS_MONITOR_CTRL) ==
	    CL_DVFS_MONITOR_CTRL_FREQ) {
		v = GET_MONITORED_RATE(v, cld->ref_rate);
		s = cl_dvfs_readl(cld, CL_DVFS_FREQ_REQ);
		s = (s & CL_DVFS_FREQ_REQ_SCALE_MASK) >>
			CL_DVFS_FREQ_REQ_SCALE_SHIFT;
		*val = (u64)v * (s + 1) / 256;

		clk_unlock_restore(c, &flags);
		clk_disable(cld->soc_clk);
		return 0;
	}
	*val = v;

	clk_unlock_restore(c, &flags);
	clk_disable(cld->soc_clk);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(monitor_fops, monitor_get, NULL, "%llu\n");

static int vmin_get(void *data, u64 *val)
{
	u32 v;
	struct tegra_cl_dvfs *cld = ((struct clk *)data)->u.dfll.cl_dvfs;

#if CL_DVFS_DYNAMIC_OUTPUT_CFG
	clk_enable(cld->soc_clk);
	v = cl_dvfs_readl(cld, CL_DVFS_OUTPUT_CFG);
	v = (v & CL_DVFS_OUTPUT_CFG_MIN_MASK) >> CL_DVFS_OUTPUT_CFG_MIN_SHIFT;
	clk_disable(cld->soc_clk);
#else
	v = cld->lut_min;
#endif
	*val = cld->out_map[v]->reg_uV / 1000;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(vmin_fops, vmin_get, NULL, "%llu\n");

static int tune_high_mv_get(void *data, u64 *val)
{
	struct tegra_cl_dvfs *cld = ((struct clk *)data)->u.dfll.cl_dvfs;
	*val = cld->safe_dvfs->dfll_data.tune_high_min_millivolts;
	return 0;
}
static int tune_high_mv_set(void *data, u64 val)
{
	unsigned long flags;
	struct clk *c = (struct clk *)data;
	struct tegra_cl_dvfs *cld = c->u.dfll.cl_dvfs;

	clk_lock_save(c, &flags);

	cld->safe_dvfs->dfll_data.tune_high_min_millivolts = val;
	cl_dvfs_init_output_thresholds(cld);
	if (cld->mode == TEGRA_CL_DVFS_CLOSED_LOOP) {
		set_cl_config(cld, &cld->last_req);
		set_request(cld, &cld->last_req);
	}

	clk_unlock_restore(c, &flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(tune_high_mv_fops, tune_high_mv_get, tune_high_mv_set,
			"%llu\n");
static int fmin_get(void *data, u64 *val)
{
	struct tegra_cl_dvfs *cld = ((struct clk *)data)->u.dfll.cl_dvfs;
	*val = cld->dvco_rate_min;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(dvco_rate_min_fops, fmin_get, NULL, "%llu\n");

static int cl_register_show(struct seq_file *s, void *data)
{
	u32 offs;
	struct clk *c = s->private;
	struct tegra_cl_dvfs *cld = c->u.dfll.cl_dvfs;

	clk_enable(cld->soc_clk);

	seq_printf(s, "CONTROL REGISTERS:\n");
	for (offs = 0; offs <= CL_DVFS_MONITOR_DATA; offs += 4)
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

	clk_disable(cld->soc_clk);
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

	clk_enable(cld->soc_clk);
	cl_dvfs_writel(cld, val, offs & (~0x3));
	clk_disable(cld->soc_clk);
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

	if (!debugfs_create_file("vmin_mv", S_IRUGO,
		cpu_cl_dvfs_dentry, dfll_cpu, &vmin_fops))
		goto err_out;

	if (!debugfs_create_file("tune_high_mv", S_IRUGO,
		cpu_cl_dvfs_dentry, dfll_cpu, &tune_high_mv_fops))
		goto err_out;

	if (!debugfs_create_file("dvco_min", S_IRUGO,
		cpu_cl_dvfs_dentry, dfll_cpu, &dvco_rate_min_fops))
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
