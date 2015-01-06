/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <soc/tegra/tegra_bpmp.h>
#include "bpmp.h"

/*
 * Coordinates the cluster idle entries (CC6 & deeper)
 *
 * When any CPU is ready to enter CC6 or deeper, it shall call this API
 * to request a GO from bpmp. A zero return value indicates that the
 * transition is granted and a non-zero return value means either an
 * error or that the transition is denied.
 *
 * If the CCx entry do happen, bpmp will execute the given SCx entry as
 * a side effect. The last CPU's scx value prevails over any previous
 * requests.
 *
 * Should be called from the cpuidle driver after disabling interrupts
 *
 * @cpu: CPU id
 * @ccxtl: CCx tolerance of the given CPU
 * @scx: SCx side-effect mode
 */
int tegra_bpmp_do_idle(int cpu, int ccxtl, int scx)
{
	int data[] = { cpu, ccxtl, scx };
	int tl;
	return bpmp_rpc(MRQ_DO_IDLE, data, sizeof(data),
			&tl, sizeof(tl)) ?: tl;
}

/*
 * Called by CPU to inform bpmp about its new tolerance level.
 * This is used when the CPU does not require permission from
 * bpmp for its next step - for e.g during offlining.
 *
 * Can be called from interrupt or thread context
 *
 * @cpu: CPU id
 * @ccxtl: CCx tolerance of the given CPU
 * @scxtl: SCX tolerance of the given CPU
 */
int tegra_bpmp_tolerate_idle(int cpu, int ccxtl, int scxtl)
{
	int data[] = { cpu, ccxtl, scxtl };
	return bpmp_post(MRQ_TOLERATE_IDLE, data, sizeof(data));
}

int tegra_bpmp_scx_enable(int scx)
{
	return bpmp_post(MRQ_SCX_ENABLE, &scx, sizeof(scx));
}

/*
 * Cluster switch coordinator.
 * Returns the online cpu mask in current cluster.
 * Should be called with interrupts disabled.
 *
 * @cpu: id of the cluster switch initiator
 */
int tegra_bpmp_switch_cluster(int cpu)
{
	int on_cpus;

	if (bpmp_rpc(MRQ_SWITCH_CLUSTER, &cpu, sizeof(cpu),
			&on_cpus, sizeof(on_cpus))) {
		WARN_ON(1);
		return -EFAULT;
	}

	return on_cpus;
}

void tegra_bpmp_sclk_skip_set_rate(unsigned long input_rate,
		unsigned long rate)
{
	uint32_t mb[] = { input_rate, rate };
	int r;
	r = __bpmp_rpc(MRQ_SCLK_SKIP_SET_RATE, &mb, sizeof(mb), NULL, 0);
	WARN_ON(r);
}

void tegra_bpmp_enable_suspend(int mode, int flags)
{
	int mb[] = { mode, flags };
	int r = bpmp_post(MRQ_ENABLE_SUSPEND, &mb, sizeof(mb));
	WARN_ON(r);
}
