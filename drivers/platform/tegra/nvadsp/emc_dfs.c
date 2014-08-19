/*
 * emc_dfs.c
 *
 * Emc dynamic frequency scaling due to APE
 *
 * Copyright (C) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/tegra_nvadsp.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/debugfs.h>

#include "dev.h"

/* Register offsets */
#define ABRIDGE_STATS_READ_0		0x04
#define ABRIDGE_STATS_WRITE_0		0x0c
#define ABRIDGE_STATS_CLEAR_0		0x1b
#define ABRIDGE_STATS_HI_0FFSET		0x04


#define DEFAULT_SAMPLE_PERIOD	500000
#define MIN_SCALE_DOWN_TIME		39995
#define INT_SHIFT		32
#define make64(hi, low) ((((u64)hi) << INT_SHIFT) | (low))

struct emc_dfs_info {
	void __iomem *abridge_base;
	struct timer_list cnt_timer;
	u64 rd_cnt;
	u64 wr_cnt;
	bool enable;
	u64 avg_cnt;
	unsigned long timer_rate;
	unsigned long scale_down_time;
	u8 boost_step;
	struct clk *emcclk;
};

static struct emc_dfs_info emcinfo;
struct emc_dfs_info *einfo;

u64 read64(u32 offset)
{
	u32 low;
	u32 hi;

	low = readl(einfo->abridge_base + offset);
	hi = readl(einfo->abridge_base + (offset + ABRIDGE_STATS_HI_0FFSET));
	return make64(hi, low);
}
unsigned long count_to_emcfreq(u64 avg_cnt)
{
	unsigned long freq;
	unsigned long final_freq;
	freq = clk_get_rate(einfo->emcclk) / 1000; /* emc freq in MHZ */

	final_freq = (freq * avg_cnt) / 100;

	pr_debug("%s: avg_cnt: %llu  current freq(kHz): %lu target freq(kHz): %lu\n",
		"ape.emc", avg_cnt, freq, final_freq);

	return final_freq;
}

static void emc_dfs_timer(unsigned long data)
{
	u64 rd_cnt;
	u64 wr_cnt;
	u64 total_cnt;
	unsigned long emc_freq;
	u64 prev_total_cnt = einfo->rd_cnt + einfo->wr_cnt;

	/* Return if emc dfs is disabled */
	if (!einfo->enable)
		return;

	rd_cnt = read64((u32)ABRIDGE_STATS_READ_0);
	wr_cnt = read64((u32)ABRIDGE_STATS_WRITE_0);

	einfo->rd_cnt = rd_cnt;
	einfo->wr_cnt = wr_cnt;

	total_cnt = rd_cnt + wr_cnt;

	einfo->avg_cnt = div64_u64(total_cnt * 100, prev_total_cnt);

	emc_freq = count_to_emcfreq(einfo->avg_cnt);

	clk_set_rate(einfo->emcclk, emc_freq * 1000);

	mod_timer(&einfo->cnt_timer,
			  jiffies + usecs_to_jiffies(einfo->timer_rate));
}

static void emc_dfs_enable(void)
{
	einfo->rd_cnt = read64((u32)ABRIDGE_STATS_READ_0);
	einfo->wr_cnt = read64((u32)ABRIDGE_STATS_WRITE_0);

	mod_timer(&einfo->cnt_timer, jiffies + 2);
}
static void emc_dfs_disable(void)
{
	einfo->rd_cnt = read64((u32)ABRIDGE_STATS_READ_0);
	einfo->wr_cnt = read64((u32)ABRIDGE_STATS_WRITE_0);

	del_timer_sync(&einfo->cnt_timer);
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *emc_dfs_root;

#define RW_MODE (S_IWUSR | S_IRUGO)
#define RO_MODE S_IRUGO

/* Get emc dfs staus: 0: disabled 1:enabled */
static int dfs_enable_get(void *data, u64 *val)
{
	*val = einfo->enable;
	return 0;
}
/* Enable/disable emc dfs */
static int dfs_enable_set(void *data, u64 val)
{
	einfo->enable = val;
		/*
		 * If enabling: activate a timer to execute in next 2 jiffies,
		 * so that emc scaled value takes effect immidiately.
		 */
	if (einfo->enable)
		emc_dfs_enable();
	else
		emc_dfs_disable();

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(enable_fops, dfs_enable_get,
	dfs_enable_set, "%llu\n");

/* scaling emc freq in multiple of boost factor */
static int boost_step_get(void *data, u64 *val)
{
	*val = einfo->boost_step;
	return 0;
}
/* Set period in usec */
static int boost_step_set(void *data, u64 val)
{
	if (val) {
		einfo->boost_step = val;
		return 0;
	}
	return -EINVAL;
}
DEFINE_SIMPLE_ATTRIBUTE(boost_fops, boost_step_get,
	boost_step_set, "%llu\n");

/* minimum time after that emc scaling down happens in usec */
static int scale_down_time_get(void *data, u64 *val)
{
	*val = einfo->scale_down_time;
	return 0;
}
/* Set period in usec */
static int scale_down_time_set(void *data, u64 val)
{
	if (val) {
		einfo->scale_down_time = val;
		return 0;
	}
	return -EINVAL;
}
DEFINE_SIMPLE_ATTRIBUTE(down_fops, scale_down_time_get,
	scale_down_time_set, "%llu\n");

static int period_get(void *data, u64 *val)
{
	*val = einfo->timer_rate;
	return 0;
}
/* Set period in usec */
static int period_set(void *data, u64 val)
{
	if (val) {
		einfo->timer_rate = (unsigned long)val;
		return 0;
	}
	return -EINVAL;
}
DEFINE_SIMPLE_ATTRIBUTE(period_fops, period_get, period_set, "%llu\n");


static int __init emc_dfs_debugfs_init(struct nvadsp_drv_data *drv)
{
	int ret = -ENOMEM;
	struct dentry *d;

	if (!drv->adsp_debugfs_root)
		return ret;

	emc_dfs_root  = debugfs_create_dir("emc_dfs", drv->adsp_debugfs_root);
	if (!emc_dfs_root)
		goto err_out;

	d = debugfs_create_file("enable", RW_MODE, emc_dfs_root, NULL,
		&enable_fops);
	if (!d)
		goto err_root;

	d = debugfs_create_file("boost_step", RW_MODE, emc_dfs_root, NULL,
		&boost_fops);
	if (!d)
		goto err_root;

	d = debugfs_create_file("min_scale_down_time", RW_MODE, emc_dfs_root,
		NULL, &down_fops);
	if (!d)
		goto err_root;

	d = debugfs_create_file("period", RW_MODE, emc_dfs_root, NULL,
		&period_fops);
	if (!d)
		goto err_root;

	return 0;

err_root:
	debugfs_remove_recursive(emc_dfs_root);

err_out:
	return ret;
}

#endif

status_t emc_dfs_init(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);
	int ret = 0;

	einfo = &emcinfo;
	einfo->abridge_base = drv->base_regs[ABRIDGE];
	einfo->emcclk = clk_get_sys("ape", "emc");

	einfo->timer_rate = DEFAULT_SAMPLE_PERIOD;
	einfo->scale_down_time = MIN_SCALE_DOWN_TIME;
	einfo->boost_step = 0;
	einfo->enable = 1;

	init_timer(&einfo->cnt_timer);
	einfo->cnt_timer.function = emc_dfs_timer;

	emc_dfs_enable();

	pr_info("EMC DFS is initialized\n");

#ifdef CONFIG_DEBUG_FS
	emc_dfs_debugfs_init(drv);
#endif

	return ret;
}
