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

int tegra_bpmp_pm_target(int cpu, int tolerance)
{
	return min(TEGRA_PM_C7, tolerance);
}
