/*
 * drivers/net/wireless/bcmdhd/include/dhd_custom_pm_qos_tegra.h
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

#ifndef _dhd_custom_pm_qos_tegra_h_
#define _dhd_custom_pm_qos_tegra_h_

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/pm_qos.h>
#include <linux/skbuff.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>

#ifndef TEGRA_PM_QOS_NET_RX_THRESHOLD
#define TEGRA_PM_QOS_NET_RX_THRESHOLD	100000000 /* bps */
#endif  /* TEGRA_PM_QOS_NET_RX_THRESHOLD */

#ifndef TEGRA_PM_QOS_NET_TX_THRESHOLD
#define TEGRA_PM_QOS_NET_TX_THRESHOLD	100000000 /* bps */
#endif  /* TEGRA_PM_QOS_NET_TX_THRESHOLD */

#ifndef TEGRA_PM_QOS_NET_RX_CPU_FREQ
#define TEGRA_PM_QOS_NET_RX_CPU_FREQ	2200000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_RX_CPU_FREQ */

#ifndef TEGRA_PM_QOS_NET_TX_CPU_FREQ
#define TEGRA_PM_QOS_NET_TX_CPU_FREQ	2200000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_TX_CPU_FREQ */

#ifndef TEGRA_PM_QOS_NET_RX_CPU_CORE
#define TEGRA_PM_QOS_NET_RX_CPU_CORE	4
#endif  /* TEGRA_PM_QOS_NET_RX_CPU_CORE */

#ifndef TEGRA_PM_QOS_NET_TX_CPU_CORE
#define TEGRA_PM_QOS_NET_TX_CPU_CORE	4
#endif  /* TEGRA_PM_QOS_NET_TX_CPU_CORE */

#ifndef TEGRA_PM_QOS_NET_RX_GPU_FREQ
#define TEGRA_PM_QOS_NET_RX_GPU_FREQ	1000000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_RX_GPU_FREQ */

#ifndef TEGRA_PM_QOS_NET_TX_GPU_FREQ
#define TEGRA_PM_QOS_NET_TX_GPU_FREQ	1000000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_TX_GPU_FREQ */

#ifndef TEGRA_PM_QOS_NET_RX_EMC_FREQ
#define TEGRA_PM_QOS_NET_RX_EMC_FREQ	1600000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_RX_EMC_FREQ */

#ifndef TEGRA_PM_QOS_NET_TX_EMC_FREQ
#define TEGRA_PM_QOS_NET_TX_EMC_FREQ	1600000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_TX_EMC_FREQ */

#ifndef TEGRA_PM_QOS_NET_RX_SYS_FREQ
#define TEGRA_PM_QOS_NET_RX_SYS_FREQ	408000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_RX_SYS_FREQ */

#ifndef TEGRA_PM_QOS_NET_TX_SYS_FREQ
#define TEGRA_PM_QOS_NET_TX_SYS_FREQ	408000 /* kHz */
#endif  /* TEGRA_PM_QOS_NET_TX_SYS_FREQ */

#ifndef TEGRA_PM_QOS_NET_RX_TIMEOUT
#define TEGRA_PM_QOS_NET_RX_TIMEOUT	2000 /* ms */
#endif  /* TEGRA_PM_QOS_NET_RX_TIMEOUT */

#ifndef TEGRA_PM_QOS_NET_TX_TIMEOUT
#define TEGRA_PM_QOS_NET_TX_TIMEOUT	2000 /* ms */
#endif  /* TEGRA_PM_QOS_NET_TX_TIMEOUT */

/* initialization */

void
tegra_pm_qos_init(void);

void
tegra_pm_qos_exit(void);

/* network packet rx/tx notification */

void
tegra_pm_qos_net_rx(struct sk_buff *skb);

void
tegra_pm_qos_net_tx(struct sk_buff *skb);

#endif  /* _dhd_custom_pm_qos_tegra_h_ */
