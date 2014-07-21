/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/irqchip/tegra-agic.h>

#include <mach/irqs.h>
#include "ape_actmon.h"
#include "dev.h"

#define ACTMON_DEV_CTRL				0x00
#define ACTMON_DEV_CTRL_ENB			(0x1 << 31)
#define ACTMON_DEV_CTRL_UP_WMARK_ENB		(0x1 << 19)
#define ACTMON_DEV_CTRL_DOWN_WMARK_ENB		(0x1 << 18)
#define ACTMON_DEV_CTRL_UP_WMARK_NUM_SHIFT	26
#define ACTMON_DEV_CTRL_UP_WMARK_NUM_MASK	(0x7 << 26)
#define ACTMON_DEV_CTRL_DOWN_WMARK_NUM_SHIFT	21
#define ACTMON_DEV_CTRL_DOWN_WMARK_NUM_MASK	(0x7 << 21)
#define ACTMON_DEV_CTRL_AVG_UP_WMARK_ENB	(0x1 << 17)
#define ACTMON_DEV_CTRL_AVG_DOWN_WMARK_ENB	(0x1 << 16)
#define ACTMON_DEV_CTRL_AT_END_ENB		(0x1 << 15)
#define ACTMON_DEV_CTRL_PERIODIC_ENB		(0x1 << 13)
#define ACTMON_DEV_CTRL_K_VAL_SHIFT		10
#define ACTMON_DEV_CTRL_K_VAL_MASK		(0x7 << 10)
#define ACTMON_DEV_CTRL_SAMPLE_PERIOD_VAL_SHIFT	(0)
#define ACTMON_DEV_CTRL_SAMPLE_PERIOD_MASK	(0xff << 0)

#define ACTMON_DEV_UP_WMARK			0x04
#define ACTMON_DEV_DOWN_WMARK			0x08
#define ACTMON_DEV_AVG_UP_WMARK			0x0c
#define ACTMON_DEV_AVG_DOWN_WMARK			0x10
#define ACTMON_DEV_INIT_AVG			0x14

#define ACTMON_DEV_COUNT			0x18
#define ACTMON_DEV_AVG_COUNT			0x1c

#define ACTMON_DEV_INTR_STATUS			0x20
#define ACTMON_DEV_INTR_UP_WMARK		(0x1 << 31)
#define ACTMON_DEV_INTR_DOWN_WMARK		(0x1 << 30)
#define ACTMON_DEV_INTR_AVG_DOWN_WMARK		(0x1 << 29)
#define ACTMON_DEV_INTR_AVG_UP_WMARK		(0x1 << 28)

#define ACTMON_DEV_COUNT_WEGHT			0x24
#define ACTMON_DEV_SAMPLE_CTRL		  0x28

#define ACTMON_DEFAULT_AVG_WINDOW_LOG2		1
#define ACTMON_DEFAULT_AVG_BAND			1	/* 1/10 of % */

/* TBD: These would come via dts file */
#define ACTMON_REG_OFFSET 0x800

/* Assuimg ADSP freq:600MHz and APE freq: 300MHz
 * ADSP freq <= (N_CLK+1) * APE freq
 * For N_CLK = 1, SAMPLE_PERIOD = 255
 * Here SAMPLE_PERIOD is set to usec mode.
 * SAMPLE_PERIOD = 255 * 256
 */
#define ACTMON_DEFAULT_SAMPLING_PERIOD		255
static unsigned long actmon_sampling_period;

static struct clk *actmon_clk;
static unsigned long actmon_clk_freq;

static void __iomem *actmon_base;

static inline u32 actmon_readl(u32 offset)
{
	return __raw_readl(actmon_base + offset);
}
static inline void actmon_writel(u32 val, u32 offset)
{
	__raw_writel(val, actmon_base + offset);
}
static inline void actmon_wmb(void)
{
	wmb();
}

#define offs(x)		(dev->reg + x)

static inline unsigned long do_percent(unsigned long val, unsigned int pct)
{
	return val * pct / 100;
}

void actmon_update_sample_period(unsigned long period)
{
	unsigned long clks_per_sample, freq_mhz;
	u32 val;

	actmon_clk_freq = clk_get_rate(actmon_clk) / 1000;
	/* Use usec and MHz mode */
	freq_mhz = actmon_clk_freq / 1000;

	actmon_sampling_period = period;
	clks_per_sample = freq_mhz * actmon_sampling_period;

	val = actmon_readl(ACTMON_DEV_CTRL);
	val |= (clks_per_sample << ACTMON_DEV_CTRL_SAMPLE_PERIOD_VAL_SHIFT)
			& ACTMON_DEV_CTRL_SAMPLE_PERIOD_MASK;
	actmon_writel(val, ACTMON_DEV_CTRL);
}

static inline void actmon_dev_up_wmark_set(struct actmon_dev *dev)
{
	u32 val;
	unsigned long freq = (dev->type == ACTMON_FREQ_SAMPLER) ?
					 dev->cur_freq : actmon_clk_freq;

	val = freq * actmon_sampling_period;
	actmon_writel(do_percent(val, dev->boost_up_threshold),
				  offs(ACTMON_DEV_UP_WMARK));
}

static inline void actmon_dev_down_wmark_set(struct actmon_dev *dev)
{
	u32 val;
	unsigned long freq = (dev->type == ACTMON_FREQ_SAMPLER) ?
				 dev->cur_freq : actmon_clk_freq;

	val = freq * actmon_sampling_period;
	actmon_writel(do_percent(val, dev->boost_down_threshold),
				  offs(ACTMON_DEV_DOWN_WMARK));
}

static inline void actmon_dev_wmark_set(struct actmon_dev *dev)
{
	u32 val;
	unsigned long freq = (dev->type == ACTMON_FREQ_SAMPLER) ?
				 dev->cur_freq : actmon_clk_freq;

	val = freq * actmon_sampling_period;
	actmon_writel(do_percent(val, dev->boost_up_threshold),
					  offs(ACTMON_DEV_UP_WMARK));
	actmon_writel(do_percent(val, dev->boost_down_threshold),
					  offs(ACTMON_DEV_DOWN_WMARK));
}

static inline void actmon_dev_avg_wmark_set(struct actmon_dev *dev)
{
	u32 avg = dev->avg_count;
	u32 band = dev->avg_band_freq * actmon_sampling_period;

	actmon_writel(avg + band, offs(ACTMON_DEV_AVG_UP_WMARK));
	avg = max(avg, band);
	actmon_writel(avg - band, offs(ACTMON_DEV_AVG_DOWN_WMARK));
}

static unsigned long actmon_dev_avg_freq_get(struct actmon_dev *dev)
{
	u64 val;

	if (dev->type == ACTMON_FREQ_SAMPLER)
		return dev->avg_count / actmon_sampling_period;

	val = (u64)dev->avg_count * dev->cur_freq;
	do_div(val, actmon_clk_freq * actmon_sampling_period);
	return (u32)val;
}

/* Activity monitor sampling operations */
irqreturn_t ape_actmon_dev_isr(int irq, void *dev_id)
{
	u32 val;
	unsigned long flags;
	struct actmon_dev *dev = (struct actmon_dev *)dev_id;

	spin_lock_irqsave(&dev->lock, flags);

	dev->avg_count = actmon_readl(offs(ACTMON_DEV_AVG_COUNT));
	actmon_dev_avg_wmark_set(dev);

	val = actmon_readl(offs(ACTMON_DEV_INTR_STATUS));
	if (val & ACTMON_DEV_INTR_UP_WMARK) {
		val = actmon_readl(offs(ACTMON_DEV_CTRL)) |
			  ACTMON_DEV_CTRL_UP_WMARK_ENB |
			  ACTMON_DEV_CTRL_DOWN_WMARK_ENB;

		dev->boost_freq = dev->boost_freq_step +
			do_percent(dev->boost_freq, dev->boost_up_coef);
		if (dev->boost_freq >= dev->max_freq) {
			dev->boost_freq = dev->max_freq;
			val &= ~ACTMON_DEV_CTRL_UP_WMARK_ENB;
		}
		actmon_writel(val, offs(ACTMON_DEV_CTRL));
	} else if (val & ACTMON_DEV_INTR_DOWN_WMARK) {
		val = actmon_readl(offs(ACTMON_DEV_CTRL)) |
				  ACTMON_DEV_CTRL_UP_WMARK_ENB |
				  ACTMON_DEV_CTRL_DOWN_WMARK_ENB;

		dev->boost_freq =
			do_percent(dev->boost_freq, dev->boost_down_coef);
		if (dev->boost_freq < (dev->boost_freq_step >> 1)) {
			dev->boost_freq = 0;
			val &= ~ACTMON_DEV_CTRL_DOWN_WMARK_ENB;
		}
		actmon_writel(val, offs(ACTMON_DEV_CTRL));
	}

	actmon_writel(0xFFFFFFFF, offs(ACTMON_DEV_INTR_STATUS)); /* clr all */
	actmon_wmb();

	spin_unlock_irqrestore(&dev->lock, flags);
	return IRQ_WAKE_THREAD;
}

irqreturn_t ape_actmon_dev_fn(int irq, void *dev_id)
{
	unsigned long flags, freq;
	struct actmon_dev *dev = (struct actmon_dev *)dev_id;

	spin_lock_irqsave(&dev->lock, flags);

	if (dev->state != ACTMON_ON) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return IRQ_HANDLED;
	}

	freq = actmon_dev_avg_freq_get(dev);
	dev->avg_actv_freq = freq;
	freq = do_percent(freq, dev->avg_sustain_coef);
	freq += dev->boost_freq;

	dev->target_freq = freq;

	spin_unlock_irqrestore(&dev->lock, flags);

	pr_info("%s(kHz): avg: %lu,  boost: %lu, target: %lu, current: %lu\n",
	dev->clk_name, dev->avg_actv_freq, dev->boost_freq, dev->target_freq,
	dev->cur_freq);

#if defined(CONFIG_TEGRA_ADSP_DFS)
	adsp_cpu_set_rate(freq);
#endif

	return IRQ_HANDLED;
}

void actmon_rate_change(unsigned long freq)
{
	unsigned long flags;
	struct actmon_dev *dev = container_of(&freq, struct actmon_dev,
		cur_freq);

	spin_lock_irqsave(&dev->lock, flags);

	dev->cur_freq = freq;
	if (dev->type == ACTMON_FREQ_SAMPLER) {
			actmon_dev_wmark_set(dev);
			actmon_wmb();
	}

	spin_unlock_irqrestore(&dev->lock, flags);
};

/* Activity monitor configuration and control */
static void actmon_dev_configure(struct actmon_dev *dev,
		unsigned long freq)
{
	u32 val;

	dev->cur_freq = freq;
	dev->target_freq = freq;
	dev->avg_actv_freq = freq;

	if (dev->type == ACTMON_FREQ_SAMPLER) {
		dev->avg_count = dev->cur_freq * actmon_sampling_period;
		dev->avg_band_freq = dev->max_freq *
						 ACTMON_DEFAULT_AVG_BAND / 1000;
	} else {
		dev->avg_count = actmon_clk_freq * actmon_sampling_period;
		dev->avg_band_freq = actmon_clk_freq *
					 ACTMON_DEFAULT_AVG_BAND / 1000;
	}
	actmon_writel(dev->avg_count, offs(ACTMON_DEV_INIT_AVG));

	BUG_ON(!dev->boost_up_threshold);
	dev->avg_sustain_coef = 100 * 100 / dev->boost_up_threshold;
	actmon_dev_avg_wmark_set(dev);
	actmon_dev_wmark_set(dev);

	actmon_writel(dev->count_weight, offs(ACTMON_DEV_COUNT_WEGHT));
	actmon_writel(0xffffffff, offs(ACTMON_DEV_INTR_STATUS)); /* clr all */

	val = ACTMON_DEV_CTRL_PERIODIC_ENB |
			ACTMON_DEV_CTRL_AVG_UP_WMARK_ENB |
		 ACTMON_DEV_CTRL_AVG_DOWN_WMARK_ENB;
	val |= ((dev->avg_window_log2 - 1) << ACTMON_DEV_CTRL_K_VAL_SHIFT) &
			ACTMON_DEV_CTRL_K_VAL_MASK;
	val |= ((dev->down_wmark_window - 1) <<
				ACTMON_DEV_CTRL_DOWN_WMARK_NUM_SHIFT) &
			   ACTMON_DEV_CTRL_DOWN_WMARK_NUM_MASK;
	val |=  ((dev->up_wmark_window - 1) <<
				 ACTMON_DEV_CTRL_UP_WMARK_NUM_SHIFT) &
				ACTMON_DEV_CTRL_UP_WMARK_NUM_MASK;
	val |= ACTMON_DEV_CTRL_DOWN_WMARK_ENB |
	ACTMON_DEV_CTRL_UP_WMARK_ENB;
	actmon_writel(val, offs(ACTMON_DEV_CTRL));
	actmon_wmb();
}

static void actmon_dev_enable(struct actmon_dev *dev)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	if (dev->state == ACTMON_OFF) {
		dev->state = ACTMON_ON;

		val = actmon_readl(offs(ACTMON_DEV_CTRL));
		val |= ACTMON_DEV_CTRL_ENB;
		actmon_writel(val, offs(ACTMON_DEV_CTRL));
		actmon_wmb();
	}
	spin_unlock_irqrestore(&dev->lock, flags);
}

static void actmon_dev_disable(struct actmon_dev *dev)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	if (dev->state == ACTMON_ON) {
		dev->state = ACTMON_OFF;

		val = actmon_readl(offs(ACTMON_DEV_CTRL));
		val &= ~ACTMON_DEV_CTRL_ENB;
			actmon_writel(val, offs(ACTMON_DEV_CTRL));
			actmon_writel(0xffffffff, offs(ACTMON_DEV_INTR_STATUS));
			actmon_wmb();
	}
	spin_unlock_irqrestore(&dev->lock, flags);
}

static int __init actmon_dev_init(struct actmon_dev *dev)
{
	int ret;
	unsigned long freq;
	int virq;

	spin_lock_init(&dev->lock);

	dev->clk = clk_get_sys(NULL, dev->clk_name);
	if (IS_ERR(dev->clk)) {
		pr_err("Failed to find %s.%s clock\n",
			dev->dev_id, dev->con_id);
		return -EINVAL;
	}
	dev->max_freq = clk_round_rate(dev->clk, ULONG_MAX);
	clk_set_rate(dev->clk, dev->max_freq);
	dev->max_freq /= 1000;
	freq = clk_get_rate(dev->clk) / 1000;
	actmon_dev_configure(dev, freq);

	dev->state = ACTMON_OFF;
	actmon_dev_enable(dev);
	clk_prepare_enable(dev->clk);
		virq = tegra_agic_irq_get_virq(INT_AMISC_ACTMON);

	ret = request_threaded_irq(virq, ape_actmon_dev_isr,
		ape_actmon_dev_fn, IRQF_SHARED, dev->dev_id, dev);
	if (ret) {
		pr_err("Failed irq %d request for %s.%s\n",
		virq, dev->dev_id, dev->con_id);
		return ret;
	}

	return 0;
}

/* APE activity monitor: Samples ADSP activity */
static struct actmon_dev actmon_dev_adsp = {
	.reg = 0x000,
	.clk_name = "adsp_cpu",

	/* ADSP/SCLK suspend activity floor */
	.suspend_freq = 40000,

	.boost_freq_step = 8000,
	.boost_up_coef = 200,
	.boost_down_coef = 50,
	.boost_up_threshold = 35,
	.boost_down_threshold = 25,

	.up_wmark_window = 0,
	.down_wmark_window = 0,
	.avg_window_log2 = ACTMON_DEFAULT_AVG_WINDOW_LOG2,
	/* Assuming ADSP is running at 600 MHz and APE is running at 300 MHz*/
	.count_weight = 0x1,

	.type = ACTMON_FREQ_SAMPLER,
	.state = ACTMON_UNINITIALIZED,
};

static struct actmon_dev *actmon_devices[] = {
	&actmon_dev_adsp,
};

#ifdef CONFIG_DEBUG_FS

#define RW_MODE (S_IWUSR | S_IRUGO)
#define RO_MODE S_IRUGO

static struct dentry *clk_debugfs_root;

static int type_show(struct seq_file *s, void *data)
{
	struct actmon_dev *dev = s->private;

	seq_printf(s, "%s\n", (dev->type == ACTMON_LOAD_SAMPLER) ?
			"Load Activity Monitor" : "Frequency Activity Monitor");
	return 0;
}
static int type_open(struct inode *inode, struct file *file)
{
	return single_open(file, type_show, inode->i_private);
}
static const struct file_operations type_fops = {
	.open		= type_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actv_get(void *data, u64 *val)
{
	unsigned long flags;
	struct actmon_dev *dev = data;

	spin_lock_irqsave(&dev->lock, flags);
	*val = actmon_dev_avg_freq_get(dev);
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(actv_fops, actv_get, NULL, "%llu\n");

static int step_get(void *data, u64 *val)
{
	struct actmon_dev *dev = data;
	*val = dev->boost_freq_step * 100 / dev->max_freq;
	return 0;
}
static int step_set(void *data, u64 val)
{
	unsigned long flags;
	struct actmon_dev *dev = data;

	if (val > 100)
		val = 100;

	spin_lock_irqsave(&dev->lock, flags);
	dev->boost_freq_step = do_percent(dev->max_freq, (unsigned int)val);
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(step_fops, step_get, step_set, "%llu\n");

static int up_threshold_get(void *data, u64 *val)
{
	struct actmon_dev *dev = data;
	*val = dev->boost_up_threshold;
	return 0;
}
static int up_threshold_set(void *data, u64 val)
{
	unsigned long flags;
	struct actmon_dev *dev = data;
	unsigned int up_threshold = (unsigned int)val;

	if (up_threshold > 100)
		up_threshold = 100;

	spin_lock_irqsave(&dev->lock, flags);

	if (up_threshold <= dev->boost_down_threshold)
		up_threshold = dev->boost_down_threshold;
	if (up_threshold)
		dev->avg_sustain_coef = 100 * 100 / up_threshold;
	dev->boost_up_threshold = up_threshold;

	actmon_dev_up_wmark_set(dev);
	actmon_wmb();

	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(up_threshold_fops, up_threshold_get,
						up_threshold_set, "%llu\n");

static int down_threshold_get(void *data, u64 *val)
{
	struct actmon_dev *dev = data;
	*val = dev->boost_down_threshold;
	return 0;
}
static int down_threshold_set(void *data, u64 val)
{
	unsigned long flags;
	struct actmon_dev *dev = data;
	unsigned int down_threshold = (unsigned int)val;

	spin_lock_irqsave(&dev->lock, flags);

	if (down_threshold >= dev->boost_up_threshold)
		down_threshold = dev->boost_up_threshold;
	dev->boost_down_threshold = down_threshold;

	actmon_dev_down_wmark_set(dev);
	actmon_wmb();

	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(down_threshold_fops, down_threshold_get,
					down_threshold_set, "%llu\n");

static int state_get(void *data, u64 *val)
{
	struct actmon_dev *dev = data;
	*val = dev->state;
	return 0;
}
static int state_set(void *data, u64 val)
{
	struct actmon_dev *dev = data;

	if (val)
		actmon_dev_enable(dev);
	else
		actmon_dev_disable(dev);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(state_fops, state_get, state_set, "%llu\n");

/* Get period in usec */
static int period_get(void *data, u64 *val)
{
	*val = actmon_sampling_period;
	return 0;
}
/* Set period in usec */
static int period_set(void *data, u64 val)
{
	int i;
	unsigned long flags;
	u8 period = (u8)val;

	if (period) {
		actmon_update_sample_period(period);

		for (i = 0; i < ARRAY_SIZE(actmon_devices); i++) {
			struct actmon_dev *dev = actmon_devices[i];
			spin_lock_irqsave(&dev->lock, flags);
			actmon_dev_wmark_set(dev);
			spin_unlock_irqrestore(&dev->lock, flags);
		}
		actmon_wmb();
		return 0;
	}
	return -EINVAL;
}
DEFINE_SIMPLE_ATTRIBUTE(period_fops, period_get, period_set, "%llu\n");


static int actmon_debugfs_create_dev(struct actmon_dev *dev)
{
	struct dentry *dir, *d;

	if (dev->state == ACTMON_UNINITIALIZED)
		return 0;

	dir = debugfs_create_dir(dev->clk_name, clk_debugfs_root);
	if (!dir)
		return -ENOMEM;

	d = debugfs_create_file(
			"actv_type", RO_MODE, dir, dev, &type_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_file(
			"avg_activity", RO_MODE, dir, dev, &actv_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_file(
			"boost_step", RW_MODE, dir, dev, &step_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_u32(
		"boost_rate_dec", RW_MODE, dir, (u32 *)&dev->boost_down_coef);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_u32(
		"boost_rate_inc", RW_MODE, dir, (u32 *)&dev->boost_up_coef);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_file(
		"boost_threshold_dn", RW_MODE, dir, dev, &down_threshold_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_file(
		"boost_threshold_up", RW_MODE, dir, dev, &up_threshold_fops);
	if (!d)
		return -ENOMEM;

	d = debugfs_create_file(
		"state", RW_MODE, dir, dev, &state_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}

static int __init actmon_debugfs_init(void)
{
	int i;
	int ret = -ENOMEM;
	struct dentry *d;

	d = debugfs_create_dir("adsp_actmon", NULL);
	if (!d)
		return ret;
	clk_debugfs_root = d;

	d = debugfs_create_file("period", RW_MODE, d, NULL, &period_fops);
	if (!d)
		goto err_out;

	for (i = 0; i < ARRAY_SIZE(actmon_devices); i++) {
		ret = actmon_debugfs_create_dev(actmon_devices[i]);
		if (ret)
			goto err_out;
	}
	return 0;

err_out:
	debugfs_remove_recursive(clk_debugfs_root);
	return ret;
}

#endif

int ape_actmon_init(struct platform_device *pdev)
{
	int i, ret;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);
	actmon_base = drv->base_regs[AMISC] + ACTMON_REG_OFFSET;

	actmon_clk = clk_get_sys(NULL, "ape");
	if (!actmon_clk) {
		pr_err("%s: Failed to find actmon clock\n", __func__);
		return -EINVAL;
	}

	ret = clk_prepare_enable(actmon_clk);
	if (ret) {
		pr_err("%s: Failed to enable actmon clock\n", __func__);
		return -EINVAL;
	}

	actmon_update_sample_period(ACTMON_DEFAULT_SAMPLING_PERIOD);
	for (i = 0; i < ARRAY_SIZE(actmon_devices); i++) {
		ret = actmon_dev_init(actmon_devices[i]);
		pr_info("%s : %s initialization (%d)\n",
		actmon_devices[i]->clk_name, ret ? "Failed" : "Completed", ret);
	}

#ifdef CONFIG_DEBUG_FS
	actmon_debugfs_init();
#endif
	return 0;
}
