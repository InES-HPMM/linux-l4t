/*
 * adsp_dfs.c
 *
 * adsp dynamic frequency scaling
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
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/clk/tegra.h>
#include <linux/seq_file.h>
#include <asm/cputime.h>
#include <linux/slab.h>

#include "dev.h"
#include "ape_actmon.h"

#ifndef CONFIG_TEGRA_ADSP_ACTMON
void actmon_rate_change(unsigned long freq)
{

}
#endif

#define MBOX_TIMEOUT 1000 /* in ms */
#define HOST_ADSP_DFS_MBOX_ID 3

enum adsp_dfs_reply {
	ACK,
	NACK,
};

/*
 * Freqency in Hz.The frequency always needs to be a multiple of 12.8 Mhz and
 * should be extended with a slab 51.2 Mhz.
 */
static unsigned long adsp_cpu_freq_table[] = {
	51200000,
	102400000,
	153600000,
	204800000,
	256000000,
	307200000,
	358400000,
	409600000,
	460800000,
	512000000,
	563200000,
	614400000,

};

struct adsp_dfs_policy {
	bool enable;
 /*
 * update_freq_flag = TRUE, ADSP ACKed the new freq
 *					= FALSE, ADSP NACKed the new freq
 */
	bool update_freq_flag;

	const char *clk_name;
	unsigned long min;    /* in kHz */
	unsigned long max;    /* in kHz */
	unsigned long cur;    /* in kHz */
	unsigned long cpu_min;    /* in kHz */
	unsigned long cpu_max;    /* in kHz */

	struct clk *adsp_clk;
	struct notifier_block rate_change_nb;
	struct nvadsp_mbox mbox;

#ifdef CONFIG_DEBUG_FS
	struct dentry *root;
#endif
};

struct adsp_freq_stats {
	struct device *dev;
	unsigned long long last_time;
	int last_index;
	u64 *time_in_state;
	int state_num;
};

struct adsp_dfs_policy *policy;
static struct adsp_freq_stats freq_stats;
static DEFINE_MUTEX(policy_mutex);

static unsigned long adsp_get_target_freq(unsigned long tfreq, int *index)
{
	int i;
	int size = sizeof(adsp_cpu_freq_table) / sizeof(adsp_cpu_freq_table[0]);

	if (tfreq <= adsp_cpu_freq_table[0]) {
		*index = 0;
		return adsp_cpu_freq_table[0];
	}

	if (tfreq >= adsp_cpu_freq_table[size - 1]) {
		*index = size - 1;
		return adsp_cpu_freq_table[size - 1];
	}

	for (i = 1; i < size; i++) {
		if ((tfreq <= adsp_cpu_freq_table[i]) &&
				(tfreq > adsp_cpu_freq_table[i - 1])) {
			*index = i;
			return adsp_cpu_freq_table[i];
		}
	}

	return 0;
}

static void adspfreq_stats_update(void)
{
	unsigned long long cur_time;

	cur_time = get_jiffies_64();
	freq_stats.time_in_state[freq_stats.last_index] += cur_time -
		freq_stats.last_time;
	freq_stats.last_time = cur_time;
}

/* adsp clock rate change notifier callback */
static int adsp_dfs_rc_callback(
	struct notifier_block *nb, unsigned long rate, void *v)
{
	unsigned long freq = rate / 1000;
	int old_index, new_index;

	actmon_rate_change(freq);

	mutex_lock(&policy_mutex);
	policy->cur = freq;

	/* update states */
	adspfreq_stats_update();
	old_index = freq_stats.last_index;
	adsp_get_target_freq(policy->cur * 1000, &new_index);
	if (old_index != new_index)
		freq_stats.last_index = new_index;

	mutex_unlock(&policy_mutex);

	pr_debug("policy->cur:%lu\n", policy->cur);
	/* TBD: Communicate ADSP about new freq */

	return NOTIFY_OK;
};

static struct adsp_dfs_policy dfs_policy =  {
	.enable = 1,
	.clk_name = "adsp_cpu",
	.rate_change_nb = {
		.notifier_call = adsp_dfs_rc_callback,
	},
};

/**
 * update_policy - update adsp freq and ask adsp to work post change
 *		in freq tasks
 * tfreq - target frequency
 * return - final freq got set.
 *		- 0, incase of error.
 *
 * Note - Policy->cur would be updated via rate
 * change notifier, when freq is changed in hw
 *
 */
static unsigned long update_policy(unsigned long tfreq)
{
	enum adsp_dfs_reply reply;
	struct nvadsp_mbox *mbx = &policy->mbox;
	unsigned long old_freq;
	int index;
	int ret;

	old_freq = policy->cur;

	tfreq = adsp_get_target_freq(tfreq * 1000, &index);
	if (!tfreq) {
		pr_info("unable set the target freq\n");
		return 0;
	}

	ret = clk_set_rate(policy->adsp_clk, tfreq);
	if (ret) {
		pr_err("failed to set adsp freq:%d\n", ret);
		policy->update_freq_flag = false;
		return 0;
	}

	tfreq = clk_get_rate(policy->adsp_clk) / 1000;

	mutex_lock(&policy_mutex);

	pr_debug("sending change in freq:%lu\n", tfreq);
	/*
	 * Ask adsp to do action upon change in freq. ADSP and Host need to
	 * maintain the same freq table.
	 */
	ret = nvadsp_mbox_send(mbx, index,
				NVADSP_MBOX_SMSG, true, 100);
	if (ret) {
		pr_err("%s:host to adsp, mbox_send failure ....\n", __func__);
		policy->update_freq_flag = false;
		goto fail;
	}

	ret = nvadsp_mbox_recv(&policy->mbox, &reply, true, MBOX_TIMEOUT);
	if (ret) {
		pr_err("%s:host to adsp,  mbox_receive failure ....\n",
		__func__);
		policy->update_freq_flag = false;
		goto fail;
	}

	switch (reply) {
	case ACK:
		/* Set Update freq flag */
		pr_debug("adsp freq change status:ACK\n");
		policy->update_freq_flag = true;
		break;
	case NACK:
		/* Set Update freq flag */
		pr_debug("adsp freq change status:NACK\n");
		policy->update_freq_flag = false;
		break;
	default:
		pr_err("Error: adsp freq change status\n");
	}

	pr_debug("%s:status received from adsp: %s, tfreq:%lu\n", __func__,
		(policy->update_freq_flag == true ? "ACK" : "NACK"), tfreq);

fail:
	mutex_unlock(&policy_mutex);

	if (!policy->update_freq_flag) {
		ret = clk_set_rate(policy->adsp_clk, old_freq * 1000);
		if (ret) {
			pr_err("failed to resume adsp freq:%lu\n", old_freq);
			policy->update_freq_flag = false;
		}

		tfreq = old_freq;
	}
	return tfreq;
}

#ifdef CONFIG_DEBUG_FS

#define RW_MODE (S_IWUSR | S_IRUGO)
#define RO_MODE S_IRUGO

/* Get adsp dfs staus: 0: disabled, 1: enabled */
static int dfs_enable_get(void *data, u64 *val)
{
	*val = policy->enable;
	return 0;
}

/* Enable/disable adsp dfs */
static int dfs_enable_set(void *data, u64 val)
{
	policy->enable = (bool) val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(enable_fops, dfs_enable_get,
	dfs_enable_set, "%llu\n");

/* Get adsp dfs policy min freq(KHz) */
static int policy_min_get(void *data, u64 *val)
{
	*val = policy->min;
	return 0;
}

/* Set adsp dfs policy min freq(Khz) */
static int policy_min_set(void *data, u64 val)
{
	unsigned long min = (unsigned long)val;

	if (!policy->enable) {
		pr_info("adsp dfs policy is not enabled\n");
		return -EINVAL;
	}

	if (!min || ((min < policy->cpu_min) || (min == policy->min)))
		return -EINVAL;
	else if (min >=  policy->cpu_max)
		min = policy->cpu_max;

	if (min > policy->cur)
			policy->min = update_policy(min);
	else
		policy->min = min;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(min_fops, policy_min_get,
	policy_min_set, "%llu\n");

/* Get adsp dfs policy max freq(KHz) */
static int policy_max_get(void *data, u64 *val)
{
	*val = policy->max;
	return 0;
}

/* Set adsp dfs policy max freq(KHz) */
static int policy_max_set(void *data, u64 val)
{
	unsigned long max = (unsigned long)val;

	if (!policy->enable) {
		pr_info("adsp dfs policy is not enabled\n");
		return -EINVAL;
	}

	if (!max || ((max > policy->cpu_max) || (max == policy->max)))
		return -EINVAL;
	else if (max <=  policy->cpu_min)
		max = policy->cpu_min;

	if (max < policy->cur)
			policy->max = update_policy(max);
	else
		policy->max = max;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(max_fops, policy_max_get,
	policy_max_set, "%llu\n");

/* Get adsp dfs policy's current freq */
static int policy_cur_get(void *data, u64 *val)
{
	*val = policy->cur;
	return 0;
}

/* Set adsp dfs policy cur freq(Khz) */
static int policy_cur_set(void *data, u64 val)
{
	unsigned long cur = (unsigned long)val;

	if (policy->enable) {
		pr_info("adsp dfs is enabled, should be disabled first\n");
		return -EINVAL;
	}

	if (!cur || cur == policy->cur)
		return -EINVAL;

	/* Check tfreq policy sanity */
	if (cur < policy->min)
		cur = policy->min;
	else if (cur > policy->max)
		cur = policy->max;

	cur = update_policy(cur);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cur_fops, policy_cur_get,
	policy_cur_set, "%llu\n");

/*
 * Print residency in each freq levels
 */
static void dump_stats_table(struct seq_file *s, struct adsp_freq_stats *fstats)
{
	int i;

	adspfreq_stats_update();

	seq_printf(s, "%-10s %-10s\n", "rate(kHz)", "time(ms)");
	for (i = 0; i < fstats->state_num; i++) {
		seq_printf(s, "%-10lu %-10llu\n",
			(long unsigned int)(adsp_cpu_freq_table[i] / 1000),
			cputime64_to_clock_t(fstats->time_in_state[i]));
	}
}

static int show_time_in_state(struct seq_file *s, void *data)
{
	struct adsp_freq_stats *fstats =
		(struct adsp_freq_stats *) (s->private);

	dump_stats_table(s, fstats);
	return 0;
}

static int stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_time_in_state, inode->i_private);
}

static const struct file_operations time_in_state_fops = {
	.open = stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int adsp_dfs_debugfs_init(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	struct dentry *d, *root;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	if (!drv->adsp_debugfs_root)
		return ret;

	root = debugfs_create_dir("adsp_dfs", drv->adsp_debugfs_root);
	if (!root)
		return ret;

	policy->root = root;

	d = debugfs_create_file("enable", RW_MODE, root, NULL,
		&enable_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("min_freq", RW_MODE, root, NULL,
		&min_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("max_freq", RW_MODE, root,
		NULL, &max_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("cur_freq", RW_MODE, root, NULL,
		&cur_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("time_in_state", S_IRUGO,
					root, &freq_stats,
					&time_in_state_fops);
	if (!d)
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(root);
	policy->root = NULL;
	dev_err(&pdev->dev,
	"unable to create adsp logger debug fs file\n");
	return ret;
}
#endif

void adsp_cpu_set_rate(unsigned long freq)
{
	if (!policy->enable) {
		pr_info("adsp dfs policy is not enabled\n");
		return;
	}

	if (!freq || freq == policy->cur)
		return;

	if (freq < policy->min)
		freq = policy->min;
	else if (freq > policy->max)
		freq = policy->max;

	if (freq)
		update_policy(freq);
}

/* Should be called after ADSP os is loaded */
int adsp_dfs_core_init(struct platform_device *pdev)
{
	int ret = 0;
	int size = sizeof(adsp_cpu_freq_table) / sizeof(adsp_cpu_freq_table[0]);
	uint16_t mid = HOST_ADSP_DFS_MBOX_ID;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	if (drv->dfs_initialized)
		return 0;

	policy = &dfs_policy;
	policy->adsp_clk = clk_get_sys(NULL, policy->clk_name);
	if (IS_ERR_OR_NULL(policy->adsp_clk)) {
		dev_err(&pdev->dev, "unable to find ahub clock\n");
		ret = PTR_ERR(policy->adsp_clk);
		goto end;
	}

	/* Clk_round_rate returns in Hz */
	policy->max = policy->cpu_max =
		(clk_round_rate(policy->adsp_clk, ULONG_MAX)) / 1000;
	policy->min = policy->cpu_min =
		(clk_round_rate(policy->adsp_clk, 0)) / 1000;
	policy->cur = clk_get_rate(policy->adsp_clk) / 1000;

	adsp_get_target_freq(policy->cur * 1000, &freq_stats.last_index);
	freq_stats.last_time = get_jiffies_64();
	freq_stats.state_num = size;
	freq_stats.time_in_state = kzalloc(size, GFP_KERNEL);
	if (!freq_stats.time_in_state) {
		ret = -ENOMEM;
		goto end;
	}
	ret = nvadsp_mbox_open(&policy->mbox, &mid, "dfs_comm", NULL, NULL);
	if (ret) {
		dev_info(&pdev->dev, "unable to open mailbox\n");
		kfree(freq_stats.time_in_state);
		goto end;
	}

	if (policy->rate_change_nb.notifier_call) {
		ret = tegra_register_clk_rate_notifier(policy->adsp_clk,
			&policy->rate_change_nb);
		if (ret) {
			dev_err(&pdev->dev, "rate change notifier err: %s\n",
			policy->clk_name);
			nvadsp_mbox_close(&policy->mbox);
			kfree(freq_stats.time_in_state);
			goto end;
		}
	}

#ifdef CONFIG_DEBUG_FS
	adsp_dfs_debugfs_init(pdev);
#endif
	drv->dfs_initialized = true;

	dev_info(&pdev->dev, "adsp dfs is initialized ....\n");
	return ret;
end:
	if (policy->adsp_clk)
		clk_put(policy->adsp_clk);
	return ret;
}

int adsp_dfs_core_exit(struct platform_device *pdev)
{
	status_t ret = 0;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	ret = nvadsp_mbox_close(&policy->mbox);
	if (ret)
		dev_info(&pdev->dev, "adsp dfs exit failed: mbox close error ....\n");

	kfree(freq_stats.time_in_state);
	tegra_unregister_clk_rate_notifier(policy->adsp_clk,
					   &policy->rate_change_nb);

	clk_put(policy->adsp_clk);
	drv->dfs_initialized = false;

	dev_info(&pdev->dev, "adsp dfs is exited ....\n");

	return ret;
}
