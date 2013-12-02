/*
 *  Based on arch/arm/kernel/smp.c
 *
 *  Original Copyright (C) 2002 ARM Limited, All Rights Reserved.
 *  Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/irqchip/tegra.h>

#include "flowctrl.h"

extern void tegra210_hotplug_shutdown(void);

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void tegra_cpu_die(unsigned int cpu)
{
	unsigned long reg;

	tegra_gic_cpu_disable(false);

	reg = FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG |
			FLOW_CTRL_CSR_ENABLE;
	reg |= (FLOW_CTRL_WAIT_WFI_BITMAP << cpu);
	flowctrl_write_cpu_csr(cpu, reg);

	flowctrl_write_cpu_halt(cpu, FLOW_CTRL_WAITEVENT);

	flowctrl_write_cc4_ctrl(cpu, 0);

	tegra210_hotplug_shutdown();

	BUG();
}
