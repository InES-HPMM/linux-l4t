/*
 * drivers/net/wireless/bcmdhd/dhd_custom_pm_qos_tegra.c
 *
 * NVIDIA Tegra PM QOS for BCMDHD driver
 *
 * Copyright (C) 2015 NVIDIA Corporation. All rights reserved.
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

#include "dhd_custom_pm_qos_tegra.h"

#define CLOCK_SBUS_RATE_PATH "/d/clock/sbus/rate"
#define CPU0_SCALING_GOVERNOR_PATH "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
#define CPU1_SCALING_GOVERNOR_PATH "/sys/devices/system/cpu/cpu1/cpufreq/scaling_governor"
#define CPU2_SCALING_GOVERNOR_PATH "/sys/devices/system/cpu/cpu2/cpufreq/scaling_governor"
#define CPU3_SCALING_GOVERNOR_PATH "/sys/devices/system/cpu/cpu3/cpufreq/scaling_governor"
#define CPU0_SCALING_MIN_FREQ_PATH "/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq"
#define CPU1_SCALING_MIN_FREQ_PATH "/sys/devices/system/cpu/cpu1/cpufreq/scaling_min_freq"
#define CPU2_SCALING_MIN_FREQ_PATH "/sys/devices/system/cpu/cpu2/cpufreq/scaling_min_freq"
#define CPU3_SCALING_MIN_FREQ_PATH "/sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq"
#define GPU_OVERRIDE_STATE_PATH "/d/clock/override.gbus/state"
#define GPU_OVERRIDE_RATE_PATH "/d/clock/override.gbus/rate"
#define EMC_OVERRIDE_STATE_PATH "/d/clock/override.emc/state"
#define EMC_OVERRIDE_RATE_PATH "/d/clock/override.emc/rate"

#if 0
#undef pr_debug
#define pr_debug pr_info
#endif

DEFINE_SPINLOCK(net_rx_spinlock);
DEFINE_SPINLOCK(net_tx_spinlock);

static int cpu_freq_boost_enable = 1;

static unsigned long net_rx_jiffies0;
static unsigned long net_rx_jiffies1;
static unsigned long net_rx_jiffies_boost_timeout;
static unsigned long net_rx_bits;
static unsigned long net_rx_bits_per_sec;
static unsigned long net_rx_bits_per_sec_threshold
	= TEGRA_PM_QOS_NET_RX_THRESHOLD;
static int net_rx_bits_per_sec_threshold_exceeded_cpu_boost_freq
	= TEGRA_PM_QOS_NET_RX_CPU_FREQ;
static int net_rx_bits_per_sec_threshold_exceeded_cpu_boost_core
	= TEGRA_PM_QOS_NET_RX_CPU_CORE;
static int net_rx_bits_per_sec_threshold_exceeded_gpu_boost_freq
	= TEGRA_PM_QOS_NET_RX_GPU_FREQ;
static int net_rx_bits_per_sec_threshold_exceeded_emc_boost_freq
	= TEGRA_PM_QOS_NET_RX_EMC_FREQ;
static int net_rx_bits_per_sec_threshold_exceeded_sys_boost_freq
	= TEGRA_PM_QOS_NET_RX_SYS_FREQ;
static unsigned int net_rx_bits_per_sec_threshold_exceeded_cpu_boost_ms
	= TEGRA_PM_QOS_NET_RX_TIMEOUT;
static unsigned int net_rx_boost_period
	= 1000;

static unsigned long net_tx_jiffies0;
static unsigned long net_tx_jiffies1;
static unsigned long net_tx_jiffies_boost_timeout;
static unsigned long net_tx_bits;
static unsigned long net_tx_bits_per_sec;
static unsigned long net_tx_bits_per_sec_threshold
	= TEGRA_PM_QOS_NET_TX_THRESHOLD;
static int net_tx_bits_per_sec_threshold_exceeded_cpu_boost_freq
	= TEGRA_PM_QOS_NET_TX_CPU_FREQ;
static int net_tx_bits_per_sec_threshold_exceeded_cpu_boost_core
	= TEGRA_PM_QOS_NET_TX_CPU_CORE;
static int net_tx_bits_per_sec_threshold_exceeded_gpu_boost_freq
	= TEGRA_PM_QOS_NET_TX_GPU_FREQ;
static int net_tx_bits_per_sec_threshold_exceeded_emc_boost_freq
	= TEGRA_PM_QOS_NET_TX_EMC_FREQ;
static int net_tx_bits_per_sec_threshold_exceeded_sys_boost_freq
	= TEGRA_PM_QOS_NET_TX_SYS_FREQ;
static unsigned int net_tx_bits_per_sec_threshold_exceeded_cpu_boost_ms
	= TEGRA_PM_QOS_NET_TX_TIMEOUT;
static unsigned int net_tx_boost_period
	= 1000;

module_param(cpu_freq_boost_enable, int, 0644);

module_param(net_rx_jiffies0, ulong, 0444);
module_param(net_rx_jiffies1, ulong, 0444);
module_param(net_rx_jiffies_boost_timeout, ulong, 0444);
module_param(net_rx_bits, ulong, 0444);
module_param(net_rx_bits_per_sec, ulong, 0444);
module_param(net_rx_bits_per_sec_threshold, ulong, 0644);
module_param(net_rx_bits_per_sec_threshold_exceeded_cpu_boost_freq, int, 0644);
module_param(net_rx_bits_per_sec_threshold_exceeded_cpu_boost_core, int, 0644);
module_param(net_rx_bits_per_sec_threshold_exceeded_gpu_boost_freq, int, 0644);
module_param(net_rx_bits_per_sec_threshold_exceeded_emc_boost_freq, int, 0644);
module_param(net_rx_bits_per_sec_threshold_exceeded_sys_boost_freq, int, 0644);
module_param(net_rx_bits_per_sec_threshold_exceeded_cpu_boost_ms, uint, 0644);
module_param(net_rx_boost_period, uint, 0644);

module_param(net_tx_jiffies0, ulong, 0444);
module_param(net_tx_jiffies1, ulong, 0444);
module_param(net_tx_jiffies_boost_timeout, ulong, 0444);
module_param(net_tx_bits, ulong, 0444);
module_param(net_tx_bits_per_sec, ulong, 0444);
module_param(net_tx_bits_per_sec_threshold, ulong, 0644);
module_param(net_tx_bits_per_sec_threshold_exceeded_cpu_boost_freq, int, 0644);
module_param(net_tx_bits_per_sec_threshold_exceeded_cpu_boost_core, int, 0644);
module_param(net_tx_bits_per_sec_threshold_exceeded_gpu_boost_freq, int, 0644);
module_param(net_tx_bits_per_sec_threshold_exceeded_emc_boost_freq, int, 0644);
module_param(net_tx_bits_per_sec_threshold_exceeded_sys_boost_freq, int, 0644);
module_param(net_tx_bits_per_sec_threshold_exceeded_cpu_boost_ms, uint, 0644);
module_param(net_tx_boost_period, uint, 0644);

static unsigned long bcmdhd_net_rx_cpu_freq_boost_flag;
static unsigned long  bcmdhd_net_tx_cpu_freq_boost_flag;

static struct pm_qos_request bcmdhd_net_rx_cpu_freq_pm_qos;
static struct pm_qos_request bcmdhd_net_tx_cpu_freq_pm_qos;

static struct pm_qos_request bcmdhd_net_rx_cpu_core_pm_qos;
static struct pm_qos_request bcmdhd_net_tx_cpu_core_pm_qos;

static struct pm_qos_request bcmdhd_net_rx_gpu_freq_pm_qos;
static struct pm_qos_request bcmdhd_net_tx_gpu_freq_pm_qos;

static struct pm_qos_request bcmdhd_net_rx_emc_freq_pm_qos;
static struct pm_qos_request bcmdhd_net_tx_emc_freq_pm_qos;

static void bcmdhd_net_rx_cpu_freq_boost_worker(struct work_struct *work);
static void bcmdhd_net_tx_cpu_freq_boost_worker(struct work_struct *work);

static DECLARE_DELAYED_WORK(bcmdhd_net_rx_cpu_freq_boost_work,
	bcmdhd_net_rx_cpu_freq_boost_worker);

static DECLARE_DELAYED_WORK(bcmdhd_net_tx_cpu_freq_boost_work,
	bcmdhd_net_tx_cpu_freq_boost_worker);

static void bcmdhd_net_write_to_file(const char *filename, const char *buf,
	size_t size)
{
	static DEFINE_MUTEX(write_file_mutex);
	struct file *fp;
	mm_segment_t old_fs;
	loff_t pos = 0;

	pr_debug("%s: filename %s buf %*s size %u\n",
		__func__, filename, (int) size, buf, (unsigned) size);

	/* lock mutex */
	mutex_lock(&write_file_mutex);

	/* set kernel address limits */
	old_fs = get_fs();
	set_fs(get_ds());

	/* open file */
	fp = filp_open(filename, O_WRONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: cannot open %s\n", __func__, filename);
		goto exit;
	}

	/* write file */
	vfs_write(fp, buf, size, &pos);

	/* close file */
	filp_close(fp, current->files);

exit:
	/* restore old address limits */
	set_fs(old_fs);

	/* unlock mutex */
	mutex_unlock(&write_file_mutex);

}

static void bcmdhd_net_rx_cpu_freq_boost_worker(struct work_struct *work)
{
	unsigned long now = jiffies, timeout;
	unsigned long flags;
	char rate[32];

	pr_debug("%s {\n", __func__);

	/* get rx boost timeout */
	spin_lock_irqsave(&net_rx_spinlock, flags);
	timeout = net_rx_jiffies_boost_timeout;
	spin_unlock_irqrestore(&net_rx_spinlock, flags);

	/* check rx boost timeout */
	if (time_before(now, timeout)) {
		/* boost cpu freq / core, gpu freq, emc freq */
		if (!bcmdhd_net_rx_cpu_freq_boost_flag) {
			pr_debug("%s - pm qos CPU frequency boost\n",
				__func__);
			bcmdhd_net_rx_cpu_freq_boost_flag = 1;
			pm_qos_update_request(&bcmdhd_net_rx_cpu_freq_pm_qos,
				(s32)
				net_rx_bits_per_sec_threshold_exceeded_cpu_boost_freq);
			pm_qos_update_request(&bcmdhd_net_rx_cpu_core_pm_qos,
				(s32)
				net_rx_bits_per_sec_threshold_exceeded_cpu_boost_core);
			pm_qos_update_request(&bcmdhd_net_rx_gpu_freq_pm_qos,
				(s32)
				net_rx_bits_per_sec_threshold_exceeded_gpu_boost_freq);
			pm_qos_update_request(&bcmdhd_net_rx_emc_freq_pm_qos,
				(s32)
				net_rx_bits_per_sec_threshold_exceeded_emc_boost_freq);
		}
		/* boost sys clock */
		sprintf(rate, "%d000",
			net_rx_bits_per_sec_threshold_exceeded_sys_boost_freq);
		bcmdhd_net_write_to_file(CLOCK_SBUS_RATE_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU0_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		bcmdhd_net_write_to_file(CPU1_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		bcmdhd_net_write_to_file(CPU2_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		bcmdhd_net_write_to_file(CPU3_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		sprintf(rate, "%d",
			net_rx_bits_per_sec_threshold_exceeded_cpu_boost_freq);
		bcmdhd_net_write_to_file(CPU0_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU1_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU2_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU3_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(GPU_OVERRIDE_STATE_PATH,
			"1", 1);
		sprintf(rate, "%d000",
			net_rx_bits_per_sec_threshold_exceeded_gpu_boost_freq);
		bcmdhd_net_write_to_file(GPU_OVERRIDE_RATE_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(EMC_OVERRIDE_STATE_PATH,
			"1", 1);
		sprintf(rate, "%d000",
			net_rx_bits_per_sec_threshold_exceeded_emc_boost_freq);
		bcmdhd_net_write_to_file(EMC_OVERRIDE_RATE_PATH,
			rate, strlen(rate));
		/* reschedule later to either:
		 * - boost sys clock (again, in case another task lowered it)
		 * - restore cpu freq / core, gpu freq, emc freq
		 */
		if (timeout - now >=
			msecs_to_jiffies(net_rx_boost_period)) {
			schedule_delayed_work
				(&bcmdhd_net_rx_cpu_freq_boost_work,
					msecs_to_jiffies(net_rx_boost_period));
		} else {
			schedule_delayed_work
				(&bcmdhd_net_rx_cpu_freq_boost_work,
					timeout - now);
		}
	} else {
		/* restore cpu freq / core, gpu freq, emc freq */
		if (bcmdhd_net_rx_cpu_freq_boost_flag) {
			pr_debug("%s - pm qos CPU freq back to default\n",
				__func__);
			pm_qos_update_request(&bcmdhd_net_rx_cpu_freq_pm_qos,
				(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
			pm_qos_update_request(&bcmdhd_net_rx_cpu_core_pm_qos,
				(s32)PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE);
			pm_qos_update_request(&bcmdhd_net_rx_gpu_freq_pm_qos,
				(s32)PM_QOS_GPU_FREQ_MIN_DEFAULT_VALUE);
			pm_qos_update_request(&bcmdhd_net_rx_emc_freq_pm_qos,
				(s32)PM_QOS_EMC_FREQ_MIN_DEFAULT_VALUE);
			bcmdhd_net_rx_cpu_freq_boost_flag = 0;
		}
		bcmdhd_net_write_to_file(CPU0_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(CPU1_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(CPU2_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(CPU3_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(GPU_OVERRIDE_STATE_PATH,
			"0", 1);
		bcmdhd_net_write_to_file(EMC_OVERRIDE_STATE_PATH,
			"0", 1);
	}

	pr_debug("%s }\n", __func__);

}

static void bcmdhd_net_tx_cpu_freq_boost_worker(struct work_struct *work)
{
	unsigned long now = jiffies, timeout;
	unsigned long flags;
	char rate[32];

	pr_debug("%s {\n", __func__);

	/* get tx boost timeout */
	spin_lock_irqsave(&net_tx_spinlock, flags);
	timeout = net_tx_jiffies_boost_timeout;
	spin_unlock_irqrestore(&net_tx_spinlock, flags);

	/* check tx boost timeout */
	if (time_before(now, timeout)) {
		/* boost cpu freq / core, gpu freq, emc freq */
		if (!bcmdhd_net_tx_cpu_freq_boost_flag) {
			pr_debug("%s - pm qos CPU frequency boost\n",
				__func__);
			bcmdhd_net_tx_cpu_freq_boost_flag = 1;
			pm_qos_update_request(&bcmdhd_net_tx_cpu_freq_pm_qos,
				(s32)
				net_tx_bits_per_sec_threshold_exceeded_cpu_boost_freq);
			pm_qos_update_request(&bcmdhd_net_tx_cpu_core_pm_qos,
				(s32)
				net_tx_bits_per_sec_threshold_exceeded_cpu_boost_core);
			pm_qos_update_request(&bcmdhd_net_tx_gpu_freq_pm_qos,
				(s32)
				net_tx_bits_per_sec_threshold_exceeded_gpu_boost_freq);
			pm_qos_update_request(&bcmdhd_net_tx_emc_freq_pm_qos,
				(s32)
				net_tx_bits_per_sec_threshold_exceeded_emc_boost_freq);
		}
		/* boost sys clock */
		sprintf(rate, "%d000",
			net_tx_bits_per_sec_threshold_exceeded_sys_boost_freq);
		bcmdhd_net_write_to_file(CLOCK_SBUS_RATE_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU0_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		bcmdhd_net_write_to_file(CPU1_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		bcmdhd_net_write_to_file(CPU2_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		bcmdhd_net_write_to_file(CPU3_SCALING_GOVERNOR_PATH,
			"userspace", 9);
		sprintf(rate, "%d",
			net_tx_bits_per_sec_threshold_exceeded_cpu_boost_freq);
		bcmdhd_net_write_to_file(CPU0_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU1_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU2_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(CPU3_SCALING_MIN_FREQ_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(GPU_OVERRIDE_STATE_PATH,
			"1", 1);
		sprintf(rate, "%d000",
			net_tx_bits_per_sec_threshold_exceeded_gpu_boost_freq);
		bcmdhd_net_write_to_file(GPU_OVERRIDE_RATE_PATH,
			rate, strlen(rate));
		bcmdhd_net_write_to_file(EMC_OVERRIDE_STATE_PATH,
			"1", 1);
		sprintf(rate, "%d000",
			net_tx_bits_per_sec_threshold_exceeded_emc_boost_freq);
		bcmdhd_net_write_to_file(EMC_OVERRIDE_RATE_PATH,
			rate, strlen(rate));
		/* reschedule later to either:
		 * - boost sys clock (again, in case another task lowered it)
		 * - restore cpu freq / core, gpu freq, emc freq
		 */
		if (timeout - now >=
			msecs_to_jiffies(net_tx_boost_period)) {
			schedule_delayed_work
				(&bcmdhd_net_tx_cpu_freq_boost_work,
					msecs_to_jiffies(net_tx_boost_period));
		} else {
			schedule_delayed_work
				(&bcmdhd_net_tx_cpu_freq_boost_work,
					timeout - now);
		}
	} else {
		/* restore cpu freq / core, gpu freq, emc freq */
		if (bcmdhd_net_tx_cpu_freq_boost_flag) {
			pr_debug("%s - pm qos CPU freq back to default\n",
				__func__);
			pm_qos_update_request(&bcmdhd_net_tx_cpu_freq_pm_qos,
				(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
			pm_qos_update_request(&bcmdhd_net_tx_cpu_core_pm_qos,
				(s32)PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE);
			pm_qos_update_request(&bcmdhd_net_tx_gpu_freq_pm_qos,
				(s32)PM_QOS_GPU_FREQ_MIN_DEFAULT_VALUE);
			pm_qos_update_request(&bcmdhd_net_tx_emc_freq_pm_qos,
				(s32)PM_QOS_EMC_FREQ_MIN_DEFAULT_VALUE);
			bcmdhd_net_tx_cpu_freq_boost_flag = 0;
		}
		bcmdhd_net_write_to_file(CPU0_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(CPU1_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(CPU2_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(CPU3_SCALING_GOVERNOR_PATH,
			"interactive", 11);
		bcmdhd_net_write_to_file(GPU_OVERRIDE_STATE_PATH,
			"0", 1);
		bcmdhd_net_write_to_file(EMC_OVERRIDE_STATE_PATH,
			"0", 1);
	}

	pr_debug("%s }\n", __func__);

}

void
tegra_pm_qos_init(void)
{
	pr_debug("%s\n", __func__);

	pm_qos_add_request(&bcmdhd_net_rx_cpu_freq_pm_qos,
		PM_QOS_CPU_FREQ_MIN,
		(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
	pm_qos_add_request(&bcmdhd_net_tx_cpu_freq_pm_qos,
		PM_QOS_CPU_FREQ_MIN,
		(s32)PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);

	pm_qos_add_request(&bcmdhd_net_rx_cpu_core_pm_qos,
		PM_QOS_MIN_ONLINE_CPUS,
		(s32)PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE);
	pm_qos_add_request(&bcmdhd_net_tx_cpu_core_pm_qos,
		PM_QOS_MIN_ONLINE_CPUS,
		(s32)PM_QOS_MIN_ONLINE_CPUS_DEFAULT_VALUE);

	pm_qos_add_request(&bcmdhd_net_rx_gpu_freq_pm_qos,
		PM_QOS_GPU_FREQ_MIN,
		(s32)PM_QOS_GPU_FREQ_MIN_DEFAULT_VALUE);
	pm_qos_add_request(&bcmdhd_net_tx_gpu_freq_pm_qos,
		PM_QOS_GPU_FREQ_MIN,
		(s32)PM_QOS_GPU_FREQ_MIN_DEFAULT_VALUE);

	pm_qos_add_request(&bcmdhd_net_rx_emc_freq_pm_qos,
		PM_QOS_EMC_FREQ_MIN,
		(s32)PM_QOS_EMC_FREQ_MIN_DEFAULT_VALUE);
	pm_qos_add_request(&bcmdhd_net_tx_emc_freq_pm_qos,
		PM_QOS_EMC_FREQ_MIN,
		(s32)PM_QOS_EMC_FREQ_MIN_DEFAULT_VALUE);

}

void
tegra_pm_qos_exit(void)
{
	pr_debug("%s\n", __func__);

	pm_qos_remove_request(&bcmdhd_net_rx_cpu_freq_pm_qos);
	pm_qos_remove_request(&bcmdhd_net_tx_cpu_freq_pm_qos);

	pm_qos_remove_request(&bcmdhd_net_rx_cpu_core_pm_qos);
	pm_qos_remove_request(&bcmdhd_net_tx_cpu_core_pm_qos);

	pm_qos_remove_request(&bcmdhd_net_rx_gpu_freq_pm_qos);
	pm_qos_remove_request(&bcmdhd_net_tx_gpu_freq_pm_qos);

	pm_qos_remove_request(&bcmdhd_net_rx_emc_freq_pm_qos);
	pm_qos_remove_request(&bcmdhd_net_tx_emc_freq_pm_qos);

}

void
tegra_pm_qos_net_rx(struct sk_buff *skb)
{
	unsigned long flags;
	unsigned long delta;

	if (!cpu_freq_boost_enable)
		return;

	/* check if rx bps exceeds threshold for boosting cpu freq */
	spin_lock_irqsave(&net_rx_spinlock, flags);
	if (!net_rx_jiffies0)
		net_rx_jiffies0 = jiffies;
	net_rx_jiffies1 = jiffies;
	net_rx_bits += skb->len * 8;
	delta = net_rx_jiffies1 - net_rx_jiffies0;
	if (delta < 10) {
		spin_unlock_irqrestore(&net_rx_spinlock, flags);
		return;
	}
	if ((delta > msecs_to_jiffies(1000)) ||
		(net_rx_bits / (delta + 1) > ULONG_MAX / HZ)) {
		net_rx_jiffies0 = 0;
		net_rx_bits = 0;
		spin_unlock_irqrestore(&net_rx_spinlock, flags);
		return;
	}
	net_rx_bits_per_sec = net_rx_bits / (delta + 1) * HZ;
	if (net_rx_bits_per_sec < net_rx_bits_per_sec_threshold) {
		spin_unlock_irqrestore(&net_rx_spinlock, flags);
		return;
	}
	net_rx_jiffies_boost_timeout = net_rx_jiffies1 +
		msecs_to_jiffies
		(net_rx_bits_per_sec_threshold_exceeded_cpu_boost_ms);
	spin_unlock_irqrestore(&net_rx_spinlock, flags);

	/* boost cpu freq */
	if (!bcmdhd_net_rx_cpu_freq_boost_flag) {
		if (!delayed_work_pending
			(&bcmdhd_net_rx_cpu_freq_boost_work)) {
			schedule_delayed_work
				(&bcmdhd_net_rx_cpu_freq_boost_work,
				msecs_to_jiffies(1));
		}
	}

}

void
tegra_pm_qos_net_tx(struct sk_buff *skb)
{
	unsigned long flags;
	unsigned long delta;

	if (!cpu_freq_boost_enable)
		return;

	/* check if tx bps exceeds threshold for boosting cpu freq */
	spin_lock_irqsave(&net_tx_spinlock, flags);
	if (!net_tx_jiffies0)
		net_tx_jiffies0 = jiffies;
	net_tx_jiffies1 = jiffies;
	net_tx_bits += skb->len * 8;
	delta = net_tx_jiffies1 - net_tx_jiffies0;
	if (delta < 10) {
		spin_unlock_irqrestore(&net_tx_spinlock, flags);
		return;
	}
	if ((delta > msecs_to_jiffies(1000)) ||
		(net_tx_bits / (delta + 1) > ULONG_MAX / HZ)) {
		net_tx_jiffies0 = 0;
		net_tx_bits = 0;
		spin_unlock_irqrestore(&net_tx_spinlock, flags);
		return;
	}
	net_tx_bits_per_sec = net_tx_bits / (delta + 1) * HZ;
	if (net_tx_bits_per_sec < net_tx_bits_per_sec_threshold) {
		spin_unlock_irqrestore(&net_tx_spinlock, flags);
		return;
	}
	net_tx_jiffies_boost_timeout = net_tx_jiffies1 +
		msecs_to_jiffies
		(net_tx_bits_per_sec_threshold_exceeded_cpu_boost_ms);
	spin_unlock_irqrestore(&net_tx_spinlock, flags);

	/* boost cpu freq */
	if (!bcmdhd_net_tx_cpu_freq_boost_flag) {
		if (!delayed_work_pending
			(&bcmdhd_net_tx_cpu_freq_boost_work)) {
			schedule_delayed_work
				(&bcmdhd_net_tx_cpu_freq_boost_work,
				msecs_to_jiffies(1));
		}
	}

}
