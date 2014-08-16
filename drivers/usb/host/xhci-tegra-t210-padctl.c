/*
 * xhci-tegra-t210-padctl.c - Nvidia xHCI host padctl driver

 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <mach/tegra_usb_pad_ctrl.h>
#include "xhci-tegra.h"
#include "xhci-tegra-t210-padreg.h"

/* UPHY_USB3_PADx_ECTL_1 */
#define TX_TERM_CTRL(x)	(((x) & 0x3) << 16)

/* UPHY_USB3_PADx_ECTL_2 */
#define RX_CTLE(x)		(((x) & 0xffff) << 0)

/* UPHY_USB3_PADx_ECTL_3 */
#define RX_DFE(x)		(((x) & 0xffffffff) << 0)

/* UPHY_USB3_PADx_ECTL_6 */
#define RX_EQ_CTRL_H(x)	(((x) & 0xffffffff) << 0)

void t210_program_ss_pad(struct tegra_xhci_hcd *tegra, u8 port)
{
	u32 ctl1, ctl2, ctl3, ctl6, val;

	xusb_ss_pad_init(port, GET_SS_PORTMAP(tegra->bdata->ss_portmap, port)
			, XUSB_HOST_MODE);

	ctl1 = tegra->padregs->uphy_usb3_padX_ectlY_0[port][0];
	ctl2 = tegra->padregs->uphy_usb3_padX_ectlY_0[port][1];
	ctl3 = tegra->padregs->uphy_usb3_padX_ectlY_0[port][2];
	ctl6 = tegra->padregs->uphy_usb3_padX_ectlY_0[port][5];

	val = padctl_readl(tegra, ctl1);
	val &= ~TX_TERM_CTRL(~0);
	val |= TX_TERM_CTRL(tegra->soc_config->tx_term_ctrl);
	padctl_writel(tegra, val, ctl1);

	val = padctl_readl(tegra, ctl2);
	val &= ~RX_CTLE(~0);
	val |= RX_CTLE(tegra->soc_config->rx_ctle);
	padctl_writel(tegra, val, ctl2);

	val = padctl_readl(tegra, ctl3);
	val &= ~RX_DFE(~0);
	val |= RX_DFE(tegra->soc_config->rx_dfe);
	padctl_writel(tegra, val, ctl3);

	val = padctl_readl(tegra, ctl6);
	val &= ~RX_EQ_CTRL_H(~0);
	val |= RX_EQ_CTRL_H(tegra->soc_config->rx_eq_ctrl_h);
	padctl_writel(tegra, val, ctl6);
}
