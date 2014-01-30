/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/platform_data/tegra_bpmp.h>
#include "bpmp_abi.h"
#include "bpmp_private.h"

int bpmp_ping(void)
{
	unsigned long flags;
	ktime_t tm;
	int r;

	/* disable local irqs so we can call the atomic IPC api */
	local_irq_save(flags);
	tm = ktime_get();
	r = bpmp_rpc(MRQ_PING, NULL, 0, NULL, 0);
	tm = ktime_sub(ktime_get(), tm);
	local_irq_restore(flags);

	return r ?: ktime_to_us(tm);
}

/*
 * Coordinates the cluster idle entries (CC6 & deeper)
 *
 * When any CPU is ready to enter CC6 or deeper, it shall call this API
 * to request a GO from bpmp. A zero return value indicates that the
 * transition is granted and a non-zero return value means either an
 * error or that the transition is denied.
 *
 * Should be called from the cpuidle driver after disabling interrupts
 *
 * @cpu: CPU id
 * @tolerance: tolerance of the given CPU
 */
int tegra_bpmp_do_idle(int cpu, int tolerance)
{
	int data[] = { cpu, tolerance };
	int tl;

	if (bpmp_rpc(MRQ_DO_IDLE, data, sizeof(data), &tl, sizeof(tl))) {
		WARN_ON(1);
		return -EFAULT;
	}

	return tl;
}

/*
 * When any CPU is being hot plugged/unplugged, it shall call this API
 * to inform bpmp about its new tolerance level.
 *
 * Can be called from interrupt or thread context
 *
 * @cpu: CPU id
 * @tolerance: tolerance of the given CPU
 */
int tegra_bpmp_tolerate_idle(int cpu, int tolerance)
{
	unsigned long flags;
	int data[] = { cpu, tolerance };
	int r;

	local_irq_save(flags);
	r = bpmp_post(MRQ_TOLERATE_IDLE, data, sizeof(data));
	local_irq_restore(flags);

	return r;
}
