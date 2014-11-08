/*
 * drivers/platform/tegra/pm.c
 *
 * CPU complex suspend & resume functions for Tegra SoCs
 *
 * Copyright (c) 2009-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/io.h>
#include <linux/syscore_ops.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/serial_reg.h>
#include <linux/tegra-pm.h>
#include <linux/tegra_pm_domains.h>

#include "../../../arch/arm/mach-tegra/iomap.h"

struct tegra_pm_context {
	u8	uart[5];
};

static RAW_NOTIFIER_HEAD(tegra_pm_chain_head);

int tegra_register_pm_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&tegra_pm_chain_head, nb);
}
EXPORT_SYMBOL(tegra_register_pm_notifier);

int tegra_unregister_pm_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&tegra_pm_chain_head, nb);
}
EXPORT_SYMBOL(tegra_unregister_pm_notifier);

int tegra_pm_notifier_call_chain(unsigned int val)
{
	int ret = raw_notifier_call_chain(&tegra_pm_chain_head, val, NULL);

	return notifier_to_errno(ret);
}

static struct tegra_pm_context suspend_ctx;

unsigned long debug_uart_port_base = 0;
EXPORT_SYMBOL(debug_uart_port_base);

static int tegra_debug_uart_suspend(void)
{
	void __iomem *uart;
	u32 lcr;

	if (!debug_uart_port_base)
		return 0;

	uart = IO_ADDRESS(debug_uart_port_base);

	lcr = readb(uart + UART_LCR * 4);

	suspend_ctx.uart[0] = lcr;
	suspend_ctx.uart[1] = readb(uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	suspend_ctx.uart[2] = readb(uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	suspend_ctx.uart[3] = readb(uart + UART_DLL * 4);
	suspend_ctx.uart[4] = readb(uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);

	return 0;
}

static void tegra_debug_uart_resume(void)
{
	void __iomem *uart;
	u32 lcr;

	if (!debug_uart_port_base)
		return;

	uart = IO_ADDRESS(debug_uart_port_base);

	lcr = suspend_ctx.uart[0];

	writeb(suspend_ctx.uart[1], uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(UART_FCR_ENABLE_FIFO | UART_FCR_T_TRIG_01 | UART_FCR_R_TRIG_01,
			uart + UART_FCR * 4);

	writeb(suspend_ctx.uart[2], uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(suspend_ctx.uart[3], uart + UART_DLL * 4);
	writeb(suspend_ctx.uart[4], uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);
}

static struct syscore_ops tegra_debug_uart_syscore_ops = {
	.suspend = tegra_debug_uart_suspend,
	.resume = tegra_debug_uart_resume,
	.save = tegra_debug_uart_suspend,
	.restore = tegra_debug_uart_resume,
};

static int tegra_debug_uart_syscore_init(void)
{
	register_syscore_ops(&tegra_debug_uart_syscore_ops);
	return 0;
}
arch_initcall(tegra_debug_uart_syscore_init);
