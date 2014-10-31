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

#include <linux/dma-mapping.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/platform_data/tegra_bpmp.h>
#include <linux/slab.h>
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

int bpmp_module_load(struct device *dev, const void *base, u32 size,
		u32 *handle)
{
	void *virt;
	dma_addr_t phys;
	struct { u32 phys; u32 size; } __packed msg;
	int r;

	virt = dma_alloc_coherent(dev, size, &phys, GFP_KERNEL);
	if (virt == NULL)
		return -ENOMEM;

	memcpy(virt, base, size);

	msg.phys = phys;
	msg.size = size;

	r = bpmp_threaded_rpc(MRQ_MODULE_LOAD, &msg, sizeof(msg),
			handle, sizeof(*handle));

	dma_free_coherent(dev, size, virt, phys);
	return r;
}

int bpmp_module_unload(struct device *dev, u32 handle)
{
	return bpmp_threaded_rpc(MRQ_MODULE_UNLOAD, &handle, sizeof(handle),
			NULL, 0);
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

int bpmp_cpuidle_usage(int state)
{
	struct { int usage; uint64_t time; } ib;
	return bpmp_threaded_rpc(MRQ_CPUIDLE_USAGE, &state, sizeof(state),
			&ib, sizeof(ib)) ?: ib.usage;
}

uint64_t bpmp_cpuidle_time(int state)
{
	struct { int usage; uint64_t time; } ib;
	return bpmp_threaded_rpc(MRQ_CPUIDLE_USAGE, &state, sizeof(state),
			&ib, sizeof(ib)) ?: ib.time;
}

int bpmp_write_trace(uint32_t phys, int size, int *eof)
{
	uint32_t ob[] = { phys, size };
	return __bpmp_rpc(MRQ_WRITE_TRACE, ob, sizeof(ob), eof, sizeof(*eof));
}

int bpmp_modify_trace_mask(uint32_t clr, uint32_t set)
{
	uint32_t ob[] = { clr, set };
	uint32_t new;
	return bpmp_threaded_rpc(MRQ_TRACE_MODIFY, ob, sizeof(ob),
			&new, sizeof(new)) ?: new;
}

int bpmp_init_cpus_present(int nr_cpus)
{
	return bpmp_post(MRQ_INIT_NR_CPUS, &nr_cpus, sizeof(nr_cpus));
}

void tegra_bpmp_sclk_skip_set_rate(unsigned long input_rate,
		unsigned long rate)
{
	uint32_t mb[] = { input_rate, rate };
	int r;
	r = __bpmp_rpc(MRQ_SCLK_SKIP_SET_RATE, &mb, sizeof(mb), NULL, 0);
	WARN_ON(r);
}

int bpmp_query_tag(uint32_t phys)
{
	return __bpmp_rpc(MRQ_QUERY_TAG, &phys, sizeof(phys), NULL, 0);
}
