/*
 * PCIe host controller driver for TEGRA SOCs
 *
 * Copyright (c) 2010, CompuLab, Ltd.
 * Author: Mike Rapoport <mike@compulab.co.il>
 *
 * Based on NVIDIA PCIe driver
 * Copyright (c) 2008-2014, NVIDIA Corporation. All rights reserved.
 *
 * Bits taken from arch/arm/mach-dove/pcie.c
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

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/clk/tegra.h>
#include <linux/msi.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/async.h>
#include <linux/vmalloc.h>
#include <linux/pm_runtime.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra-soc.h>
#include <linux/pci-tegra.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/tegra_pm_domains.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-tegra.h>

#include <asm/sizes.h>
#include <asm/mach/pci.h>
#include <asm/io.h>

#include <mach/tegra_usb_pad_ctrl.h>
#include <mach/io_dpd.h>

/* register definitions */
#define AFI_OFFSET							0x3800
#define PADS_OFFSET							0x3000
#define RP_OFFSET							0x1000

#define AFI_AXI_BAR0_SZ							0x00
#define AFI_AXI_BAR1_SZ							0x04
#define AFI_AXI_BAR2_SZ							0x08
#define AFI_AXI_BAR3_SZ							0x0c
#define AFI_AXI_BAR4_SZ							0x10
#define AFI_AXI_BAR5_SZ							0x14

#define AFI_AXI_BAR0_START						0x18
#define AFI_AXI_BAR1_START						0x1c
#define AFI_AXI_BAR2_START						0x20
#define AFI_AXI_BAR3_START						0x24
#define AFI_AXI_BAR4_START						0x28
#define AFI_AXI_BAR5_START						0x2c

#define AFI_FPCI_BAR0							0x30
#define AFI_FPCI_BAR1							0x34
#define AFI_FPCI_BAR2							0x38
#define AFI_FPCI_BAR3							0x3c
#define AFI_FPCI_BAR4							0x40
#define AFI_FPCI_BAR5							0x44

#define AFI_CACHE_BAR0_SZ						0x48
#define AFI_CACHE_BAR0_ST						0x4c
#define AFI_CACHE_BAR1_SZ						0x50
#define AFI_CACHE_BAR1_ST						0x54

#define AFI_MSI_BAR_SZ							0x60
#define AFI_MSI_FPCI_BAR_ST						0x64
#define AFI_MSI_AXI_BAR_ST						0x68

#define AFI_MSI_VEC0_0							0x6c
#define AFI_MSI_VEC1_0							0x70
#define AFI_MSI_VEC2_0							0x74
#define AFI_MSI_VEC3_0							0x78
#define AFI_MSI_VEC4_0							0x7c
#define AFI_MSI_VEC5_0							0x80
#define AFI_MSI_VEC6_0							0x84
#define AFI_MSI_VEC7_0							0x88

#define AFI_MSI_EN_VEC0_0						0x8c
#define AFI_MSI_EN_VEC1_0						0x90
#define AFI_MSI_EN_VEC2_0						0x94
#define AFI_MSI_EN_VEC3_0						0x98
#define AFI_MSI_EN_VEC4_0						0x9c
#define AFI_MSI_EN_VEC5_0						0xa0
#define AFI_MSI_EN_VEC6_0						0xa4
#define AFI_MSI_EN_VEC7_0						0xa8

#define AFI_CONFIGURATION						0xac
#define AFI_CONFIGURATION_EN_FPCI				(1 << 0)

#define AFI_FPCI_ERROR_MASKS						0xb0

#define AFI_INTR_MASK							0xb4
#define AFI_INTR_MASK_INT_MASK					(1 << 0)
#define AFI_INTR_MASK_MSI_MASK					(1 << 8)

#define AFI_INTR_CODE							0xb8
#define AFI_INTR_CODE_MASK						0x1f
#define AFI_INTR_MASTER_ABORT						4
#define AFI_INTR_LEGACY						6
#define AFI_INTR_PRSNT_SENSE						10

#define AFI_INTR_SIGNATURE						0xbc
#define AFI_SM_INTR_ENABLE						0xc4

#define AFI_AFI_INTR_ENABLE						0xc8
#define AFI_INTR_EN_INI_SLVERR						(1 << 0)
#define AFI_INTR_EN_INI_DECERR						(1 << 1)
#define AFI_INTR_EN_TGT_SLVERR						(1 << 2)
#define AFI_INTR_EN_TGT_DECERR						(1 << 3)
#define AFI_INTR_EN_TGT_WRERR						(1 << 4)
#define AFI_INTR_EN_DFPCI_DECERR					(1 << 5)
#define AFI_INTR_EN_AXI_DECERR						(1 << 6)
#define AFI_INTR_EN_FPCI_TIMEOUT					(1 << 7)
#define AFI_INTR_EN_PRSNT_SENSE					(1 << 8)

#define AFI_PCIE_PME						0x0f0
#define AFI_PCIE_PME_TURN_OFF					0x101
#define AFI_PCIE_PME_ACK					0x420

#define AFI_PCIE_CONFIG						0x0f8
#define AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE			(1 << 1)
#define AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE			(1 << 2)
#define AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_MASK		(0xf << 20)
#define AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X2_X1		(0x0 << 20)
#define AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X4_X1		(0x1 << 20)

#define AFI_FUSE							0x104
#define AFI_FUSE_PCIE_T0_GEN2_DIS				(1 << 2)

#define AFI_PEX0_CTRL							0x110
#define AFI_PEX1_CTRL							0x118
#define AFI_PEX_CTRL_RST					(1 << 0)
#define AFI_PEX_CTRL_CLKREQ_EN					(1 << 1)
#define AFI_PEX_CTRL_REFCLK_EN					(1 << 3)
#define AFI_PEX_CTRL_OVERRIDE_EN				(1 << 4)

#define AFI_PLLE_CONTROL					0x160
#define AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL		(1 << 9)
#define AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN			(1 << 1)

#define AFI_PEXBIAS_CTRL_0					0x168
#define AFI_WR_SCRATCH_0					0x120
#define AFI_WR_SCRATCH_0_RESET_VAL				0x00202020
#define AFI_WR_SCRATCH_0_DEFAULT_VAL				0x00000000

#define AFI_MSG_0						0x190
#define AFI_MSG_PM_PME_MASK					0x00100010
#define AFI_MSG_INTX_MASK					0x1f001f00
#define AFI_MSG_PM_PME0						(1 << 4)
#define AFI_MSG_RP_INT_MASK					0x10001000

#define RP_VEND_XP						0x00000F00
#define RP_VEND_XP_DL_UP					(1 << 30)

#define RP_LINK_CONTROL_STATUS					0x00000090

#define  PADS_REFCLK_CFG0					0x000000C8
#define  PADS_REFCLK_CFG1					0x000000CC
#define  PADS_REFCLK_BIAS					0x000000D0
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
#define REFCLK_POR_SETTINGS					0x409c409c
#else
#define REFCLK_POR_SETTINGS					0x44ac44ac
#endif

#define NV_PCIE2_RP_RSR					0x000000A0
#define NV_PCIE2_RP_RSR_PMESTAT				(1 << 16)

#define NV_PCIE2_RP_INTR_BCR					0x0000003C
#define NV_PCIE2_RP_INTR_BCR_INTR_LINE				(0xFF << 0)
#define NV_PCIE2_RP_TX_HDR_LIMIT				0x00000E08
#define PCIE2_RP_TX_HDR_LIMIT_NPT_0				32
#define PCIE2_RP_TX_HDR_LIMIT_NPT_1				4

#define NV_PCIE2_RP_TIMEOUT0					0x00000E24
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP_MASK			(0xFF)
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP				(0x7)
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM_MASK			(0xFFFF00)
#define PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM				(0x104 << 8)
#define PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2_MASK		(0xFF << 24)
#define PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2			(0x7 << 24)

#define NV_PCIE2_RP_TIMEOUT1					0x00000E28
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE_MASK		(0xFF << 16)
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE		(0xB << 16)
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE_MASK	(0xFF << 24)
#define PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE		(0x4E << 24)

#define NV_PCIE2_RP_XP_REF					0x00000F30
#define PCIE2_RP_XP_REF_MICROSECOND_LIMIT_MASK			(0xFF)
#define PCIE2_RP_XP_REF_MICROSECOND_LIMIT			(0xD)
#define PCIE2_RP_XP_REF_MICROSECOND_ENABLE			(1 << 8)
#define PCIE2_RP_XP_REF_CPL_TO_OVERRIDE			(1 << 13)
#define PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE_MASK		(0x1FFFF << 14)
#define PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE			(0xFDE << 14)

#define NV_PCIE2_RP_PRIV_MISC					0x00000FE0
#define PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_PRSNT			(0xE << 0)
#define PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT			(0xF << 0)
#define PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_THRESHOLD		(0xF << 16)
#define PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_ENABLE		(1 << 23)
#define PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_THRESHOLD		(0xF << 24)
#define PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_ENABLE		(1 << 31)

#define NV_PCIE2_RP_VEND_XP1					0x00000F04
#define NV_PCIE2_RP_VEND_XP1_LINK_PVT_CTL_L1_ASPM_SUPPORT	(1 << 21)

#define NV_PCIE2_RP_VEND_CTL0					0x00000F44
#define PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH_MASK		(0xF << 12)
#define PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH			(0x9 << 12)

#define NV_PCIE2_RP_VEND_CTL1					0x00000F48
#define PCIE2_RP_VEND_CTL1_ERPT				(1 << 13)

#define NV_PCIE2_RP_VEND_XP_BIST				0x00000F4C
#define PCIE2_RP_VEND_XP_BIST_GOTO_L1_L2_AFTER_DLLP_DONE	(1 << 28)

#define NV_PCIE2_RP_ECTL_1_R2					0x00000FD8
#define PCIE2_RP_ECTL_1_R2_TX_CMADJ_1C				(0xD << 8)
#define PCIE2_RP_ECTL_1_R2_TX_DRV_CNTL_1C			(0x3 << 28)

#define NV_PCIE2_RP_XP_CTL_1					0x00000FEC
#define PCIE2_RP_XP_CTL_1_SPARE_BIT29				(1 << 29)

#define NV_PCIE2_RP_L1_PM_SUBSTATES_CYA				0x00000C00
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_MASK		(0xFF << 8)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_SHIFT		(8)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_MASK		(0x3 << 16)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_SHIFT		(16)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_MASK		(0xF8 << 19)
#define PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_SHIFT		(19)

#define NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA			0x00000C04
#define PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY_MASK	(0x1FFF)
#define PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY		(0x19)
#define PCIE2_RP_L1_PM_SUBSTATES_1_CYA_CLKREQ_ASSERTED_DLY_MASK	(0x1FF << 13)
#define PCIE2_RP_L1_PM_SUBSTATES_1_CYA_CLKREQ_ASSERTED_DLY	(0x20 << 13)

#define NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA			0x00000C08
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY_MASK		(0x1FFF)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY		(0x33)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_MASK	(0xFF << 13)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND		(0xC << 13)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP_MASK	(0xF << 21)
#define PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP	(0x8 << 21)

#define TEGRA_PCIE_MSELECT_CLK_204				204000000
#define TEGRA_PCIE_MSELECT_CLK_408				408000000
#define TEGRA_PCIE_XCLK_500					500000000
#define TEGRA_PCIE_XCLK_250					250000000

/*
 * AXI address map for the PCIe aperture , defines 1GB in the AXI
 *  address map for PCIe.
 *
 *  That address space is split into different regions, with sizes and
 *  offsets as follows. Except for the Register space, SW is free to slice the
 *  regions as it chooces.
 *
 *  The split below seems to work fine for now.
 *
 *  0x0100_0000 to 0x01ff_ffff - Register space           16MB.
 *  0x0200_0000 to 0x11ff_ffff - Config space             256MB.
 *  0x1200_0000 to 0x1200_ffff - Downstream IO space
 *   ... Will be filled with other BARS like MSI/upstream IO etc.
 *  0x1210_0000 to 0x320f_ffff - Prefetchable memory aperture
 *  0x3210_0000 to 0x3fff_ffff - non-prefetchable memory aperture
 */
#define TEGRA_PCIE_BASE	0x01000000

#define PCIE_REGS_SZ		SZ_16M
#define PCIE_CFG_OFF		(TEGRA_PCIE_BASE + PCIE_REGS_SZ)
#define PCIE_CFG_SZ		SZ_256M
/* During the boot only registers/config and extended config apertures are
 * mapped. Rest are mapped on demand by the PCI device drivers.
 */
#define MMIO_BASE		(PCIE_CFG_OFF + PCIE_CFG_SZ)
#define MMIO_SIZE		SZ_64K
#define PREFETCH_MEM_BASE_0	(MMIO_BASE + SZ_1M)
#define PREFETCH_MEM_SIZE_0	SZ_512M
#define MEM_BASE_0		(PREFETCH_MEM_BASE_0 + PREFETCH_MEM_SIZE_0)
#define MEM_SIZE_0		(SZ_1G - MEM_BASE_0)


#define DEBUG 0
#if DEBUG
#define PR_FUNC_LINE	pr_info("PCIE: %s(%d)\n", __func__, __LINE__)
#else
#define PR_FUNC_LINE	do {} while (0)
#endif

/* Pinctrl configuration paramaters */
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
#define pinctrl_compatible	"nvidia,tegra210-pinmux"
#define pin_pex_l0_clkreq	"pex_l0_clkreq_n_pa1"
#define pin_pex_l1_clkreq	"pex_l1_clkreq_n_pa4"
#else
#define pinctrl_compatible	"nvidia,tegra124-pinmux"
#define pin_pex_l0_clkreq	"pex_l0_clkreq_n_pdd2"
#define pin_pex_l1_clkreq	"pex_l1_clkreq_n_pdd6"
#endif

struct tegra_pcie_chipdata {
	char			**pcie_regulator_names;
	int			num_pcie_regulators;
};

struct tegra_pcie_port {
	int			index;
	u8			root_bus_nr;
	void __iomem		*base;
	bool			link_up;
};

struct tegra_pcie_info {
	struct tegra_pcie_port	port[MAX_PCIE_SUPPORTED_PORTS];
	int			num_ports;
	void __iomem		*regs;
	int			power_rails_enabled;
	int			pcie_power_enabled;
	struct work_struct	hotplug_detect;

	struct regulator	**pcie_regulators;
	struct clk		*pcie_xclk;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	struct clk		*pex_uphy;
#endif
	struct clk		*pcie_mselect;
	struct device		*dev;
	struct tegra_pci_platform_data *plat_data;
	struct list_head busses;
	const struct tegra_pcie_chipdata *chipdata;
} tegra_pcie;

struct tegra_pcie_bus {
	struct vm_struct *area;
	struct list_head list;
	unsigned int nr;
};

static struct resource pcie_mem_space;
static struct resource pcie_prefetch_mem_space;

/* this flag enables features required either after boot or resume */
static bool resume_path;
/* used to avoid successive hotplug disconnect or connect */
static bool hotplug_event;
/* pcie mselect & xclk rate */
static unsigned long tegra_pcie_mselect_rate = TEGRA_PCIE_MSELECT_CLK_204;
static unsigned long tegra_pcie_xclk_rate = TEGRA_PCIE_XCLK_250;

static inline void afi_writel(u32 value, unsigned long offset)
{
	writel(value, offset + AFI_OFFSET + tegra_pcie.regs);
}

static inline u32 afi_readl(unsigned long offset)
{
	return readl(offset + AFI_OFFSET + tegra_pcie.regs);
}

/* Array of PCIe Controller Register offsets */
static u32 pex_controller_registers[] = {
	AFI_PEX0_CTRL,
	AFI_PEX1_CTRL,
};

static inline void pads_writel(u32 value, unsigned long offset)
{
	writel(value, offset + PADS_OFFSET + tegra_pcie.regs);
}

static inline u32 pads_readl(unsigned long offset)
{
	return readl(offset + PADS_OFFSET + tegra_pcie.regs);
}

static inline void rp_writel(u32 value, unsigned long offset, int rp)
{
	BUG_ON(rp != 0 && rp != 1 && rp != 2);
	offset += rp * (0x1UL << (rp - 1)) * RP_OFFSET;
	writel(value, offset + tegra_pcie.regs);
}

static inline unsigned int rp_readl(unsigned long offset, int rp)
{
	BUG_ON(rp != 0 && rp != 1 && rp != 2);
	offset += rp * (0x1UL << (rp - 1)) * RP_OFFSET;
	return readl(offset + tegra_pcie.regs);
}

static struct tegra_pcie_port *bus_to_port(int bus)
{
	int i;

	for (i = tegra_pcie.num_ports - 1; i >= 0; i--) {
		int rbus = tegra_pcie.port[i].root_bus_nr;
		if (rbus != -1 && rbus == bus)
			break;
	}

	return i >= 0 ? tegra_pcie.port + i : NULL;
}

/*
 * The configuration space mapping on Tegra is somewhat similar to the ECAM
 * defined by PCIe. However it deviates a bit in how the 4 bits for extended
 * register accesses are mapped:
 *
 *    [27:24] extended register number
 *    [23:16] bus number
 *    [15:11] device number
 *    [10: 8] function number
 *    [ 7: 0] register number
 *
 * Mapping the whole extended configuration space would required 256 MiB of
 * virtual address space, only a small part of which will actually be used.
 * To work around this, a 1 MiB of virtual addresses are allocated per bus
 * when the bus is first accessed. When the physical range is mapped, the
 * the bus number bits are hidden so that the extended register number bits
 * appear as bits [19:16]. Therefore the virtual mapping looks like this:
 *
 *    [19:16] extended register number
 *    [15:11] device number
 *    [10: 8] function number
 *    [ 7: 0] register number
 *
 * This is achieved by stitching together 16 chunks of 64 KiB of physical
 * address space via the MMU.
 */
static unsigned long tegra_pcie_conf_offset(unsigned int devfn, int where)
{

	return ((where & 0xf00) << 8) | (PCI_SLOT(devfn) << 11) |
	       (PCI_FUNC(devfn) << 8) | (where & 0xfc);
}

static struct tegra_pcie_bus *tegra_pcie_bus_alloc(unsigned int busnr)
{
	phys_addr_t cs = (phys_addr_t)PCIE_CFG_OFF;
	struct tegra_pcie_bus *bus;
	unsigned int i;
	int err;
#ifndef CONFIG_ARM64
	pgprot_t prot = L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_DIRTY | L_PTE_XN |
			L_PTE_MT_DEV_SHARED | L_PTE_SHARED;
#else
	pgprot_t prot = PTE_PRESENT | PTE_YOUNG | PTE_DIRTY | PTE_XN |
		PTE_SHARED | PTE_TYPE_PAGE;
	(void)pgprot_dmacoherent(prot); /* L_PTE_MT_DEV_SHARED */
#endif

	PR_FUNC_LINE;
	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&bus->list);
	bus->nr = busnr;

	/* allocate 1 MiB of virtual addresses */
	bus->area = get_vm_area(SZ_1M, VM_IOREMAP);
	if (!bus->area) {
		err = -ENOMEM;
		goto free;
	}

	/* map each of the 16 chunks of 64 KiB each.
	 *
	 * Note that each chunk still needs to increment by 16 MiB in
	 * physical space.
	 */
	for (i = 0; i < 16; i++) {
		unsigned long virt = (unsigned long)bus->area->addr +
				     i * SZ_64K;
		phys_addr_t phys = cs + i * SZ_16M + busnr * SZ_64K;

		err = ioremap_page_range(virt, virt + SZ_64K, phys, prot);
		if (err < 0) {
			dev_err(tegra_pcie.dev, "ioremap_page_range() failed: %d\n",
				err);
			goto unmap;
		}
	}

	return bus;
unmap:
	vunmap(bus->area->addr);
free:
	kfree(bus);
	return ERR_PTR(err);
}

/*
 * Look up a virtual address mapping for the specified bus number.
 * If no such mapping existis, try to create one.
 */
static void __iomem *tegra_pcie_bus_map(unsigned int busnr)
{
	struct tegra_pcie_bus *bus;

	list_for_each_entry(bus, &tegra_pcie.busses, list)
		if (bus->nr == busnr)
			return bus->area->addr;

	bus = tegra_pcie_bus_alloc(busnr);
	if (IS_ERR(bus))
		return NULL;

	list_add_tail(&bus->list, &tegra_pcie.busses);

	return bus->area->addr;
}

int tegra_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	struct tegra_pcie_port *pp = bus_to_port(bus->number);
	void __iomem *addr;

	if (pp) {
		if (devfn != 0) {
			*val = 0xffffffff;
			return PCIBIOS_DEVICE_NOT_FOUND;
		}
		addr = pp->base + (where & ~0x3);
	} else {
		addr = tegra_pcie_bus_map(bus->number);
		if (!addr) {
			dev_err(tegra_pcie.dev,
				"failed to map cfg. space for bus %u\n",
				bus->number);
			*val = 0xffffffff;
			return PCIBIOS_DEVICE_NOT_FOUND;
		}
		addr += tegra_pcie_conf_offset(devfn, where);
	}

	*val = readl(addr);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;

	return PCIBIOS_SUCCESSFUL;
}
EXPORT_SYMBOL(tegra_pcie_read_conf);

static int tegra_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	struct tegra_pcie_port *pp = bus_to_port(bus->number);
	void __iomem *addr;

	u32 mask;
	u32 tmp;

	/* pcie core is supposed to enable bus mastering and io/mem responses
	 * if its not setting then enable corresponding bits in pci_command
	 */
	if (where == PCI_COMMAND) {
		if (!(val & PCI_COMMAND_IO))
			val |= PCI_COMMAND_IO;
		if (!(val & PCI_COMMAND_MEMORY))
			val |= PCI_COMMAND_MEMORY;
		if (!(val & PCI_COMMAND_MASTER))
			val |= PCI_COMMAND_MASTER;
		if (!(val & PCI_COMMAND_SERR))
			val |= PCI_COMMAND_SERR;
	}

	if (pp) {
		if (devfn != 0)
			return PCIBIOS_DEVICE_NOT_FOUND;

		addr = pp->base + (where & ~0x3);
	} else {
		addr = tegra_pcie_bus_map(bus->number);
		if (!addr) {
			dev_err(tegra_pcie.dev,
				"failed to map cfg. space for bus %u\n",
				bus->number);
			return PCIBIOS_DEVICE_NOT_FOUND;
		}
		addr += tegra_pcie_conf_offset(devfn, where);
	}

	if (size == 4) {
		writel(val, addr);
		return PCIBIOS_SUCCESSFUL;
	}

	if (size == 2)
		mask = ~(0xffff << ((where & 0x3) * 8));
	else if (size == 1)
		mask = ~(0xff << ((where & 0x3) * 8));
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	tmp = readl(addr) & mask;
	tmp |= val << ((where & 0x3) * 8);
	writel(tmp, addr);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops tegra_pcie_ops = {
	.read	= tegra_pcie_read_conf,
	.write	= tegra_pcie_write_conf,
};

static void tegra_pcie_fixup_bridge(struct pci_dev *dev)
{
	u16 reg;

	if ((dev->class >> 16) == PCI_BASE_CLASS_BRIDGE) {
		pci_read_config_word(dev, PCI_COMMAND, &reg);
		reg |= (PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
			PCI_COMMAND_MASTER | PCI_COMMAND_SERR);
		pci_write_config_word(dev, PCI_COMMAND, reg);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, tegra_pcie_fixup_bridge);

/* Tegra PCIE root complex wrongly reports device class */
static void tegra_pcie_fixup_class(struct pci_dev *dev)
{
	dev->class = PCI_CLASS_BRIDGE_PCI << 8;
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_NVIDIA, 0x0e1c, tegra_pcie_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_NVIDIA, 0x0e1d, tegra_pcie_fixup_class);

/* Tegra PCIE requires relaxed ordering */
static void tegra_pcie_relax_enable(struct pci_dev *dev)
{
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, tegra_pcie_relax_enable);

static void tegra_pcie_preinit(void)
{
	PR_FUNC_LINE;
	pcie_mem_space.name = "PCIe MEM Space";
	pcie_mem_space.start = MEM_BASE_0;
	pcie_mem_space.end = MEM_BASE_0 + MEM_SIZE_0 - 1;
	pcie_mem_space.flags = IORESOURCE_MEM;
	if (request_resource(&iomem_resource, &pcie_mem_space))
		panic("can't allocate PCIe MEM space");

	pcie_prefetch_mem_space.name = "PCIe PREFETCH MEM Space";
	pcie_prefetch_mem_space.start = PREFETCH_MEM_BASE_0;
	pcie_prefetch_mem_space.end = (PREFETCH_MEM_BASE_0
			+ PREFETCH_MEM_SIZE_0 - 1);
	pcie_prefetch_mem_space.flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;
	if (request_resource(&iomem_resource, &pcie_prefetch_mem_space))
		panic("can't allocate PCIe PREFETCH MEM space");

}

static int tegra_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct tegra_pcie_port *pp;

	PR_FUNC_LINE;
	if (nr >= tegra_pcie.num_ports)
		return 0;

	pp = tegra_pcie.port + nr;
	pp->root_bus_nr = sys->busnr;

	pci_ioremap_io(nr * MMIO_SIZE, MMIO_BASE);
	pci_add_resource_offset(
		&sys->resources, &pcie_mem_space, sys->mem_offset);
	pci_add_resource_offset(
		&sys->resources, &pcie_prefetch_mem_space, sys->mem_offset);

	return 1;
}

static int tegra_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	return INT_PCIE_INTR;
}

static struct pci_bus *__init tegra_pcie_scan_bus(int nr,
						  struct pci_sys_data *sys)
{
	struct tegra_pcie_port *pp;

	PR_FUNC_LINE;
	if (nr >= tegra_pcie.num_ports)
		return NULL;

	pp = tegra_pcie.port + nr;
	pp->root_bus_nr = sys->busnr;

	return pci_scan_root_bus(NULL, sys->busnr, &tegra_pcie_ops, sys,
				 &sys->resources);
}

static struct hw_pci __initdata tegra_pcie_hw = {
	.nr_controllers	= MAX_PCIE_SUPPORTED_PORTS,
	.preinit	= tegra_pcie_preinit,
	.setup		= tegra_pcie_setup,
	.scan		= tegra_pcie_scan_bus,
	.map_irq	= tegra_pcie_map_irq,
};

#ifdef HOTPLUG_ON_SYSTEM_BOOT
/* It enumerates the devices when dock is connected after system boot */
/* this is similar to pcibios_init_hw in bios32.c */
static void __init tegra_pcie_hotplug_init(void)
{
	struct pci_sys_data *sys = NULL;
	int ret, nr;

	if (is_dock_conn_at_boot)
		return;

	PR_FUNC_LINE;
	tegra_pcie_preinit();
	for (nr = 0; nr < tegra_pcie_hw.nr_controllers; nr++) {
		sys = kzalloc(sizeof(struct pci_sys_data), GFP_KERNEL);
		if (!sys)
			panic("PCI: unable to allocate sys data!");

#ifdef CONFIG_PCI_DOMAINS
		sys->domain  = tegra_pcie_hw.domain;
#endif
		sys->busnr   = nr;
		sys->swizzle = tegra_pcie_hw.swizzle;
		sys->map_irq = tegra_pcie_hw.map_irq;
		INIT_LIST_HEAD(&sys->resources);

		ret = tegra_pcie_setup(nr, sys);
		if (ret > 0) {
			if (list_empty(&sys->resources)) {
				pci_add_resource_offset(&sys->resources,
					 &ioport_resource, sys->io_offset);
				pci_add_resource_offset(&sys->resources,
					 &iomem_resource, sys->mem_offset);
			}
			pci_create_root_bus(NULL, nr, &tegra_pcie_ops,
					sys, &sys->resources);
		}
	}
	is_dock_conn_at_boot = true;
}
#endif

static void tegra_pcie_enable_aer(int index, bool enable)
{
	unsigned int data;

	PR_FUNC_LINE;
	data = rp_readl(NV_PCIE2_RP_VEND_CTL1, index);
	if (enable)
		data |= PCIE2_RP_VEND_CTL1_ERPT;
	else
		data &= ~PCIE2_RP_VEND_CTL1_ERPT;
	rp_writel(data, NV_PCIE2_RP_VEND_CTL1, index);
}

static int tegra_pcie_attach(void)
{
	struct pci_bus *bus = NULL;

	PR_FUNC_LINE;
	if (!hotplug_event)
		return 0;

	/* rescan and recreate all pcie data structures */
	while ((bus = pci_find_next_bus(bus)) != NULL)
		pci_rescan_bus(bus);
	/* unhide AER capability */
	tegra_pcie_enable_aer(0, true);

	hotplug_event = false;
	return 0;
}

static int tegra_pcie_detach(void)
{
	struct pci_dev *pdev = NULL;

	PR_FUNC_LINE;
	if (hotplug_event)
		return 0;
	hotplug_event = true;

	/* hide AER capability to avoid log spew */
	tegra_pcie_enable_aer(0, false);
	/* remove all pcie data structures */
	for_each_pci_dev(pdev) {
		pci_stop_and_remove_bus_device(pdev);
		break;
	}
	return 0;
}

static void tegra_pcie_prsnt_map_override(int index, bool prsnt)
{
	unsigned int data;

	PR_FUNC_LINE;
	/* currently only hotplug on root port 0 supported */
	data = rp_readl(NV_PCIE2_RP_PRIV_MISC, index);
	data &= ~PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT;
	if (prsnt)
		data |= PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_PRSNT;
	else
		data |= PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT;
	rp_writel(data, NV_PCIE2_RP_PRIV_MISC, index);
}

static void work_hotplug_handler(struct work_struct *work)
{
	struct tegra_pcie_info *pcie_driver =
		container_of(work, struct tegra_pcie_info, hotplug_detect);
	int val;

	PR_FUNC_LINE;
	if (pcie_driver->plat_data->gpio_hot_plug == -1)
		return;
	val = gpio_get_value(pcie_driver->plat_data->gpio_hot_plug);
	if (val == 0) {
		pr_info("PCIE Hotplug: Connected\n");
		tegra_pcie_attach();
	} else {
		pr_info("PCIE Hotplug: DisConnected\n");
		tegra_pcie_detach();
	}
}

static irqreturn_t gpio_pcie_detect_isr(int irq, void *arg)
{
	PR_FUNC_LINE;
	schedule_work(&tegra_pcie.hotplug_detect);
	return IRQ_HANDLED;
}
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
static bool raise_emc_freq(void);
#endif
static void notify_device_isr(u32 mesg)
{
	pr_debug(KERN_INFO "Legacy INTx interrupt occurred %x\n", mesg);
	/* TODO: Need to call pcie device isr instead of ignoring interrupt */
	/* same comment applies to below handler also */
}

static void handle_sb_intr(void)
{
	u32 mesg;

	PR_FUNC_LINE;
	mesg = afi_readl(AFI_MSG_0);

	if (mesg & AFI_MSG_INTX_MASK)
		/* notify device isr for INTx messages from pcie devices */
		notify_device_isr(mesg);
	else if (mesg & AFI_MSG_PM_PME_MASK) {
		u32 idx;
		/* handle PME messages */
		idx = (mesg & AFI_MSG_PM_PME0) ? 0 : 1;
		mesg = rp_readl(NV_PCIE2_RP_RSR, idx);
		mesg |= NV_PCIE2_RP_RSR_PMESTAT;
		rp_writel(mesg, NV_PCIE2_RP_RSR, idx);
	} else
		afi_writel(mesg, AFI_MSG_0);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	if (mesg & AFI_MSG_RP_INT_MASK)
		raise_emc_freq();
#endif
}

static irqreturn_t tegra_pcie_isr(int irq, void *arg)
{
	const char *err_msg[] = {
		"Unknown",
		"AXI slave error",
		"AXI decode error",
		"Target abort",
		"Master abort",
		"Invalid write",
		"",
		"Response decoding error",
		"AXI response decoding error",
		"Transcation timeout",
		"",
		"Slot Clock request change",
		"TMS Clock clamp change",
		"TMS power down",
		"Peer to Peer error",
	};

	u32 code, signature;

	PR_FUNC_LINE;
	if (!tegra_pcie.regs) {
		pr_info("PCIE: PCI/AFI registers are unmapped\n");
		return IRQ_HANDLED;
	}
	code = afi_readl(AFI_INTR_CODE) & AFI_INTR_CODE_MASK;
	signature = afi_readl(AFI_INTR_SIGNATURE);

	if (code == AFI_INTR_LEGACY)
		handle_sb_intr();

	afi_writel(0, AFI_INTR_CODE);

	if (code >= ARRAY_SIZE(err_msg))
		code = 0;

	/*
	 * do not pollute kernel log with master abort reports since they
	 * happen a lot during enumeration
	 */
	if (code == AFI_INTR_MASTER_ABORT)
		pr_debug("PCIE: %s, signature: %08x\n",
				err_msg[code], signature);
	else if ((code != AFI_INTR_LEGACY) && (code != AFI_INTR_PRSNT_SENSE))
		pr_err("PCIE: %s, signature: %08x\n", err_msg[code], signature);

	return IRQ_HANDLED;
}

/*
 *  PCIe support functions
 */
static void tegra_pcie_setup_translations(void)
{
	u32 fpci_bar;
	u32 size;
	u32 axi_address;

	PR_FUNC_LINE;
	/* Bar 0: type 1 extended configuration space */
	fpci_bar = 0xfe100000;
	size = PCIE_CFG_SZ;
	axi_address = PCIE_CFG_OFF;
	afi_writel(axi_address, AFI_AXI_BAR0_START);
	afi_writel(size >> 12, AFI_AXI_BAR0_SZ);
	afi_writel(fpci_bar, AFI_FPCI_BAR0);

	/* Bar 1: downstream IO bar */
	fpci_bar = 0xfdfc0000;
	size = MMIO_SIZE;
	axi_address = MMIO_BASE;
	afi_writel(axi_address, AFI_AXI_BAR1_START);
	afi_writel(size >> 12, AFI_AXI_BAR1_SZ);
	afi_writel(fpci_bar, AFI_FPCI_BAR1);

	/* Bar 2: prefetchable memory BAR */
	fpci_bar = (((PREFETCH_MEM_BASE_0 >> 12) & 0xfffff) << 4) | 0x1;
	size =  PREFETCH_MEM_SIZE_0;
	axi_address = PREFETCH_MEM_BASE_0;
	afi_writel(axi_address, AFI_AXI_BAR2_START);
	afi_writel(size >> 12, AFI_AXI_BAR2_SZ);
	afi_writel(fpci_bar, AFI_FPCI_BAR2);

	/* Bar 3: non prefetchable memory BAR */
	fpci_bar = (((MEM_BASE_0 >> 12) & 0xfffff) << 4) | 0x1;
	size = MEM_SIZE_0;
	axi_address = MEM_BASE_0;
	afi_writel(axi_address, AFI_AXI_BAR3_START);
	afi_writel(size >> 12, AFI_AXI_BAR3_SZ);
	afi_writel(fpci_bar, AFI_FPCI_BAR3);

	/* NULL out the remaining BAR as it is not used */
	afi_writel(0, AFI_AXI_BAR4_START);
	afi_writel(0, AFI_AXI_BAR4_SZ);
	afi_writel(0, AFI_FPCI_BAR4);

	afi_writel(0, AFI_AXI_BAR5_START);
	afi_writel(0, AFI_AXI_BAR5_SZ);
	afi_writel(0, AFI_FPCI_BAR5);

	/* map all upstream transactions as uncached */
	afi_writel(PHYS_OFFSET, AFI_CACHE_BAR0_ST);
	afi_writel(0, AFI_CACHE_BAR0_SZ);
	afi_writel(0, AFI_CACHE_BAR1_ST);
	afi_writel(0, AFI_CACHE_BAR1_SZ);

	/* No MSI */
	afi_writel(0, AFI_MSI_FPCI_BAR_ST);
	afi_writel(0, AFI_MSI_BAR_SZ);
	afi_writel(0, AFI_MSI_AXI_BAR_ST);
	afi_writel(0, AFI_MSI_BAR_SZ);
}

static int tegra_pcie_enable_pads(bool enable)
{
	int err = 0;

	PR_FUNC_LINE;
	if (!tegra_platform_is_fpga()) {
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		if (!enable)
			tegra_periph_reset_assert(tegra_pcie.pex_uphy);
#endif
		/* WAR for Eye diagram failure */
		pads_writel(REFCLK_POR_SETTINGS, PADS_REFCLK_CFG0);
		pads_writel(0x00000028, PADS_REFCLK_BIAS);
		/* PCIe pad programming is moved to XUSB_PADCTL space */
		err = pcie_phy_pad_enable(enable,
				tegra_get_lane_owner_info() >> 1);
		if (err)
			pr_err("%s unable to initalize pads\n", __func__);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		if (enable)
			tegra_periph_reset_deassert(tegra_pcie.pex_uphy);
#endif
	}
	return err;
}

static int tegra_pcie_enable_controller(void)
{
	u32 val, reg;
	int i, ret = 0, lane_owner;

	PR_FUNC_LINE;
	/* Enable slot clock and ensure reset signal is assert */
	for (i = 0; i < ARRAY_SIZE(pex_controller_registers); i++) {
		reg = pex_controller_registers[i];
		val = afi_readl(reg) | AFI_PEX_CTRL_REFCLK_EN |
			AFI_PEX_CTRL_CLKREQ_EN;
		/* Since CLKREQ# pinmux pins may float in some platfoms */
		/* resulting in disappear of refclk specially at higher temp */
		/* overrided CLKREQ to always drive refclk */
		if (!tegra_pcie.plat_data->has_clkreq)
			val |= AFI_PEX_CTRL_OVERRIDE_EN;
		val &= ~AFI_PEX_CTRL_RST;
		afi_writel(val, reg);
	}

	/* Enable PLL power down */
	val = afi_readl(AFI_PLLE_CONTROL);
	val &= ~AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL;
	val |= AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN;
	afi_writel(val, AFI_PLLE_CONTROL);

	afi_writel(0, AFI_PEXBIAS_CTRL_0);

	lane_owner = 0;
	/* Enable all PCIE controller and */
	/* system management configuration of PCIE crossbar */
	val = afi_readl(AFI_PCIE_CONFIG);
	val &= ~(AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE |
		 AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_MASK);
	if (tegra_platform_is_fpga()) {
		/* FPGA supports only x2_x1 bar config */
		val |= AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X2_X1;
	} else {
		/* Extract 2 upper bits from odmdata[28:30] and configure */
		/* T124 pcie lanes in X2_X1/X4_X1 config based on them */
		lane_owner = tegra_get_lane_owner_info() >> 1;
		if (lane_owner == PCIE_LANES_X2_X1) {
			val |= AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X2_X1;
			if (tegra_pcie.plat_data->port_status[1])
				val &= ~AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE;
		} else {
			val |= AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X4_X1;
			if ((tegra_pcie.plat_data->port_status[1]) &&
				(lane_owner == PCIE_LANES_X4_X1))
				val &= ~AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE;
		}
	}
	afi_writel(val, AFI_PCIE_CONFIG);

	/* Enable Gen 2 capability of PCIE */
	val = afi_readl(AFI_FUSE) & ~AFI_FUSE_PCIE_T0_GEN2_DIS;
	afi_writel(val, AFI_FUSE);

	/* Finally enable PCIe */
	val = afi_readl(AFI_CONFIGURATION);
	val |=  AFI_CONFIGURATION_EN_FPCI;
	afi_writel(val, AFI_CONFIGURATION);

	val = (AFI_INTR_EN_INI_SLVERR | AFI_INTR_EN_INI_DECERR |
	       AFI_INTR_EN_TGT_SLVERR | AFI_INTR_EN_TGT_DECERR |
	       AFI_INTR_EN_TGT_WRERR | AFI_INTR_EN_DFPCI_DECERR |
	       AFI_INTR_EN_AXI_DECERR | AFI_INTR_EN_PRSNT_SENSE);
	afi_writel(val, AFI_AFI_INTR_ENABLE);
	afi_writel(0xffffffff, AFI_SM_INTR_ENABLE);

	/* FIXME: No MSI for now, only INT */
	afi_writel(AFI_INTR_MASK_INT_MASK, AFI_INTR_MASK);

	/* Disable all execptions */
	afi_writel(0, AFI_FPCI_ERROR_MASKS);

	/* Wait for clock to latch (min of 100us) */
	udelay(100);
	/* deassert PEX reset signal */
	for (i = 0; i < ARRAY_SIZE(pex_controller_registers); i++) {
		val = afi_readl(pex_controller_registers[i]);
		val |= AFI_PEX_CTRL_RST;
		afi_writel(val, pex_controller_registers[i]);
	}
	/* Take the PCIe interface module out of reset */
	tegra_periph_reset_deassert(tegra_pcie.pcie_xclk);

	return ret;
}

static int tegra_pcie_enable_regulators(void)
{
	int i;
	PR_FUNC_LINE;
	if (tegra_pcie.power_rails_enabled) {
		pr_debug("PCIE: Already power rails enabled\n");
		return 0;
	}
	tegra_pcie.power_rails_enabled = 1;
	pr_info("PCIE: Enable power rails\n");

	for (i = 0; i < tegra_pcie.chipdata->num_pcie_regulators; i++) {
		if (tegra_pcie.pcie_regulators[i])
			if (regulator_enable(tegra_pcie.pcie_regulators[i]))
				pr_err("%s: can't enable regulator %s\n",
				__func__,
				tegra_pcie.chipdata->pcie_regulator_names[i]);
	}

	return 0;
}

static int tegra_pcie_disable_regulators(void)
{
	int i;
	PR_FUNC_LINE;
	if (tegra_pcie.power_rails_enabled == 0) {
		pr_debug("PCIE: Already power rails disabled\n");
		return 0;
	}
	pr_info("PCIE: Disable power rails\n");

	for (i = 0; i < tegra_pcie.chipdata->num_pcie_regulators; i++) {
		if (tegra_pcie.pcie_regulators[i] != NULL)
			if (regulator_disable(tegra_pcie.pcie_regulators[i]))
				pr_err("%s: can't disable regulator %s\n",
				__func__,
				tegra_pcie.chipdata->pcie_regulator_names[i]);
	}

	tegra_pcie.power_rails_enabled = 0;
	return 0;
}

static int tegra_pcie_power_ungate(void)
{
	int err;

	PR_FUNC_LINE;
	err = tegra_unpowergate_partition_with_clk_on(TEGRA_POWERGATE_PCIE);
	if (err) {
		pr_err("PCIE: powerup sequence failed: %d\n", err);
		return err;
	}

	tegra_periph_reset_assert(tegra_pcie.pcie_xclk);
	err = clk_prepare_enable(tegra_pcie.pcie_mselect);
	if (err) {
		pr_err("PCIE: mselect clk enable failed: %d\n", err);
		return err;
	}
	err = clk_enable(tegra_pcie.pcie_xclk);
	if (err) {
		pr_err("PCIE: pciex clk enable failed: %d\n", err);
		return err;
	}

	return 0;
}

static int tegra_pcie_map_resources(void)
{
	PR_FUNC_LINE;
	/* Allocate config space virtual memory */
#ifdef CONFIG_ARM64
#define PROT_DEVICE_GRE (PROT_DEFAULT | PTE_PXN | PTE_UXN | PTE_ATTRINDX(MT_DEVICE_GRE))
	tegra_pcie.regs = __ioremap(TEGRA_PCIE_BASE, PCIE_REGS_SZ,
					__pgprot(PROT_DEVICE_GRE));
#else
	tegra_pcie.regs = ioremap_nocache(TEGRA_PCIE_BASE, PCIE_REGS_SZ);
#endif
	if (tegra_pcie.regs == NULL) {
		pr_err("PCIE: Failed to map PCI/AFI registers\n");
		return -ENOMEM;
	}
	return 0;
}

void tegra_pcie_unmap_resources(void)
{
	PR_FUNC_LINE;
	if (tegra_pcie.regs) {
		iounmap(tegra_pcie.regs);
		tegra_pcie.regs = 0;
	}
}

static bool tegra_pcie_is_fpga_pcie(void)
{
#define CLK_RST_BOND_OUT_REG		0x60006078
#define CLK_RST_BOND_OUT_REG_PCIE	(1 << 6)
	static int val;

	PR_FUNC_LINE;
	if (!val)
		val = readl(ioremap(CLK_RST_BOND_OUT_REG, 4));
	/* return if current netlist does not contain PCIE */
	if (val & CLK_RST_BOND_OUT_REG_PCIE)
		return false;
	return true;
}

static int tegra_pcie_fpga_phy_init(void)
{
#define FPGA_GEN2_SPEED_SUPPORT		0x90000001

	PR_FUNC_LINE;
	if (!tegra_pcie_is_fpga_pcie())
		return -ENODEV;

	/* Do reset for FPGA pcie phy */
	afi_writel(AFI_WR_SCRATCH_0_RESET_VAL, AFI_WR_SCRATCH_0);
	udelay(10);
	afi_writel(AFI_WR_SCRATCH_0_DEFAULT_VAL, AFI_WR_SCRATCH_0);
	udelay(10);
	afi_writel(AFI_WR_SCRATCH_0_RESET_VAL, AFI_WR_SCRATCH_0);

	/* required for gen2 speed support on FPGA */
	rp_writel(FPGA_GEN2_SPEED_SUPPORT, NV_PCIE2_RP_VEND_XP_BIST, 0);

	return 0;
}

static void tegra_pcie_pme_turnoff(void)
{
	unsigned int data;

	PR_FUNC_LINE;
	if (tegra_platform_is_fpga() && !tegra_pcie_is_fpga_pcie())
		return;
	data = afi_readl(AFI_PCIE_PME);
	data |= AFI_PCIE_PME_TURN_OFF;
	afi_writel(data, AFI_PCIE_PME);
	do {
		data = afi_readl(AFI_PCIE_PME);
	} while (!(data & AFI_PCIE_PME_ACK));

	/* Required for PLL power down */
	data = afi_readl(AFI_PLLE_CONTROL);
	data |= AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL;
	afi_writel(data, AFI_PLLE_CONTROL);
}

static struct tegra_io_dpd pexbias_io = {
	.name			= "PEX_BIAS",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 4,
};
static struct tegra_io_dpd pexclk1_io = {
	.name			= "PEX_CLK1",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 5,
};
static struct tegra_io_dpd pexclk2_io = {
	.name			= "PEX_CLK2",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 6,
};
static int tegra_pcie_power_on(void)
{
	int err = 0;

	PR_FUNC_LINE;
	if (tegra_pcie.pcie_power_enabled) {
		pr_debug("PCIE: Already powered on");
		goto err_exit;
	}
	tegra_pcie.pcie_power_enabled = 1;
	pm_runtime_get_sync(tegra_pcie.dev);

	if (!tegra_platform_is_fpga()) {
		/* disable PEX IOs DPD mode to turn on pcie */
		tegra_io_dpd_disable(&pexbias_io);
		tegra_io_dpd_disable(&pexclk1_io);
		tegra_io_dpd_disable(&pexclk2_io);
		err = tegra_pcie_enable_regulators();
		if (err) {
			pr_err("PCIE: Failed to enable regulators\n");
			goto err_exit;
		}
	}
	err = tegra_pcie_power_ungate();
	if (err) {
		pr_err("PCIE: Failed to power ungate\n");
		goto err_exit;
	}
	err = tegra_pcie_map_resources();
	if (err) {
		pr_err("PCIE: Failed to map resources\n");
		goto err_exit;
	}
	if (tegra_platform_is_fpga()) {
		err = tegra_pcie_fpga_phy_init();
		if (err)
			pr_err("PCIE: Failed to initialize FPGA Phy\n");
	}

err_exit:
	if (err)
		pm_runtime_put(tegra_pcie.dev);
	return err;
}

static int tegra_pcie_power_off(bool all)
{
	int err = 0;

	PR_FUNC_LINE;
	if (tegra_pcie.pcie_power_enabled == 0) {
		pr_debug("PCIE: Already powered off");
		goto err_exit;
	}
	if (all) {
		tegra_pcie_prsnt_map_override(0, false);
		tegra_pcie_pme_turnoff();
		tegra_pcie_enable_pads(false);
	}
	tegra_pcie_unmap_resources();
	if (tegra_pcie.pcie_mselect)
		clk_disable(tegra_pcie.pcie_mselect);
	if (tegra_pcie.pcie_xclk)
		clk_disable(tegra_pcie.pcie_xclk);
	err = tegra_powergate_partition_with_clk_off(TEGRA_POWERGATE_PCIE);
	if (err)
		goto err_exit;

	if (!tegra_platform_is_fpga()) {
		err = tegra_pcie_disable_regulators();
		if (err)
			goto err_exit;
		/* put PEX pads into DPD mode to save additional power */
		tegra_io_dpd_enable(&pexbias_io);
		tegra_io_dpd_enable(&pexclk1_io);
		tegra_io_dpd_enable(&pexclk2_io);
	}
	pm_runtime_put(tegra_pcie.dev);

	tegra_pcie.pcie_power_enabled = 0;
err_exit:
	return err;
}

static int tegra_pcie_clocks_get(void)
{
	PR_FUNC_LINE;
	/* get the PCIEXCLK */
	tegra_pcie.pcie_xclk = clk_get_sys("tegra_pcie", "pciex");
	if (IS_ERR_OR_NULL(tegra_pcie.pcie_xclk)) {
		pr_err("%s: unable to get PCIE Xclock\n", __func__);
		return -EINVAL;
	}
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	tegra_pcie.pex_uphy = clk_get_sys("tegra_pcie", "pex_uphy");
	if (IS_ERR_OR_NULL(tegra_pcie.pex_uphy)) {
		pr_err("%s: unable to get PCIE pex_uphy clock\n", __func__);
		return -EINVAL;
	}
#endif
	tegra_pcie.pcie_mselect = clk_get_sys("tegra_pcie", "mselect");
	if (IS_ERR_OR_NULL(tegra_pcie.pcie_mselect)) {
		pr_err("%s: unable to get PCIE mselect clock\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void tegra_pcie_clocks_put(void)
{
	PR_FUNC_LINE;
	if (tegra_pcie.pcie_xclk)
		clk_put(tegra_pcie.pcie_xclk);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	if (tegra_pcie.pex_uphy)
		clk_put(tegra_pcie.pex_uphy);
#endif
	if (tegra_pcie.pcie_mselect)
		clk_put(tegra_pcie.pcie_mselect);
}

static int tegra_pcie_get_resources(void)
{
	int err;

	PR_FUNC_LINE;
	tegra_pcie.power_rails_enabled = 0;
	tegra_pcie.pcie_power_enabled = 0;

	err = tegra_pcie_clocks_get();
	if (err) {
		pr_err("PCIE: failed to get clocks: %d\n", err);
		goto err_clk_get;
	}
	err = tegra_pcie_power_on();
	if (err) {
		pr_err("PCIE: Failed to power on: %d\n", err);
		goto err_pwr_on;
	}
	err = clk_set_rate(tegra_pcie.pcie_mselect, tegra_pcie_mselect_rate);
	if (err)
		return err;

	err = clk_set_rate(tegra_pcie.pcie_xclk, tegra_pcie_xclk_rate);
	if (err)
		return err;
	err = request_irq(INT_PCIE_INTR, tegra_pcie_isr,
			IRQF_SHARED, "PCIE", &tegra_pcie);
	if (err) {
		pr_err("PCIE: Failed to register IRQ: %d\n", err);
		goto err_pwr_on;
	}
	set_irq_flags(INT_PCIE_INTR, IRQF_VALID);
	return 0;

err_pwr_on:
	tegra_pcie_power_off(false);
err_clk_get:
	tegra_pcie_clocks_put();
	return err;
}

/*
 * FIXME: If there are no PCIe cards attached, then calling this function
 * can result in the increase of the bootup time as there are big timeout
 * loops.
 */
#define TEGRA_PCIE_LINKUP_TIMEOUT	200	/* up to 1.2 seconds */
static bool tegra_pcie_check_link(struct tegra_pcie_port *pp, int idx,
				  u32 reset_reg)
{
	u32 reg;
	int retries = 3;
	int timeout;

	PR_FUNC_LINE;
	do {
		timeout = TEGRA_PCIE_LINKUP_TIMEOUT;
		while (timeout) {
			reg = readl(pp->base + RP_VEND_XP);

			if (reg & RP_VEND_XP_DL_UP)
				break;

			mdelay(1);
			timeout--;
		}

		if (!timeout)  {
			pr_err("PCIE: port %d: link down, retrying\n", idx);
			goto retry;
		}

		timeout = TEGRA_PCIE_LINKUP_TIMEOUT;
		while (timeout) {
			reg = readl(pp->base + RP_LINK_CONTROL_STATUS);

			if (reg & 0x20000000)
				return true;

			mdelay(1);
			timeout--;
		}

retry:
		if (--retries) {
			/* Pulse the PEX reset */
			reg = afi_readl(reset_reg) & ~AFI_PEX_CTRL_RST;
			afi_writel(reg, reset_reg);
			reg = afi_readl(reset_reg) | AFI_PEX_CTRL_RST;
			afi_writel(reg, reset_reg);
		}

	} while (retries);

	return false;
}

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
static bool t210_war;
static bool is_all_gen2(void)
{
	struct pci_dev *pdev = NULL;
	u16 lnk_spd;

	PR_FUNC_LINE;
	for_each_pci_dev(pdev) {
		pcie_capability_read_word(pdev, PCI_EXP_LNKSTA, &lnk_spd);
		lnk_spd &= PCI_EXP_LNKSTA_CLS;
		if (lnk_spd != PCI_EXP_LNKSTA_CLS_5_0GB)
			return false;
	}
	return true;
}
static bool raise_emc_freq(void)
{
	/* raise emc freq to 508MHz to reach expected gen2 */
	/* bandwidth if all have gen2 enabled, bug#1452749 */
	if (t210_war && is_all_gen2()) {
		struct clk *emc_clk;
		emc_clk = clk_get_sys("tegra_pcie", "emc");
		if (IS_ERR_OR_NULL(emc_clk)) {
			pr_err("unable to get emc clk\n");
			goto fail;
		}
		if (clk_enable(emc_clk)) {
			pr_err("emc clk enable failed\n");
			goto fail;
		}
		clk_set_rate(emc_clk, 508000000);
	}
fail:
	return;
}
#endif
static void tegra_pcie_apply_sw_war(int index, bool enum_done)
{
	unsigned int data;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	int lane_owner;
#endif
	struct pci_dev *pdev = NULL;

	PR_FUNC_LINE;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	/* T210 WAR for perf bugs required when LPDDR4 */
	/* memory is used with both ctlrs in X4_X1 config */
	lane_owner = tegra_get_lane_owner_info() >> 1;
	t210_war = (tegra_pcie.plat_data->has_memtype_lpddr4 &&
		tegra_pcie.port[0].link_up && tegra_pcie.port[1].link_up &&
		(lane_owner == PCIE_LANES_X4_X1));
#endif
	if (enum_done) {
		/* disable msi for port driver to avoid panic */
		for_each_pci_dev(pdev)
			if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
				pdev->msi_enabled = 0;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		raise_emc_freq();
#endif
	} else {
		/* WAR for Eye diagram failure on lanes for T124 platforms */
		data = rp_readl(NV_PCIE2_RP_ECTL_1_R2, index);
		data |= PCIE2_RP_ECTL_1_R2_TX_CMADJ_1C;
		data |= PCIE2_RP_ECTL_1_R2_TX_DRV_CNTL_1C;
		rp_writel(data, NV_PCIE2_RP_ECTL_1_R2, index);
		/* Avoid warning during enumeration for invalid IRQ of RP */
		data = rp_readl(NV_PCIE2_RP_INTR_BCR, index);
		data |= NV_PCIE2_RP_INTR_BCR_INTR_LINE;
		rp_writel(data, NV_PCIE2_RP_INTR_BCR, index);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		/* resize buffers for better perf, bug#1447522 */
		if (t210_war) {
			data = rp_readl(NV_PCIE2_RP_XP_CTL_1, 0);
			data |= PCIE2_RP_XP_CTL_1_SPARE_BIT29;
			rp_writel(data, NV_PCIE2_RP_XP_CTL_1, 0);
			data = rp_readl(NV_PCIE2_RP_XP_CTL_1, 1);
			data |= PCIE2_RP_XP_CTL_1_SPARE_BIT29;
			rp_writel(data, NV_PCIE2_RP_XP_CTL_1, 1);

			data = rp_readl(NV_PCIE2_RP_TX_HDR_LIMIT, 0);
			data |= PCIE2_RP_TX_HDR_LIMIT_NPT_0;
			rp_writel(data, NV_PCIE2_RP_TX_HDR_LIMIT, 0);
			data = rp_readl(NV_PCIE2_RP_TX_HDR_LIMIT, 1);
			data |= PCIE2_RP_TX_HDR_LIMIT_NPT_1;
			rp_writel(data, NV_PCIE2_RP_TX_HDR_LIMIT, 1);
		}
		/* Bug#1461732 WAR, set clkreq asserted delay greater than */
		/* power off time (2us) to avoid RP wakeup in L1.2_ENTRY */
		data = rp_readl(NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA, index);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_1_CYA_CLKREQ_ASSERTED_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_1_CYA_CLKREQ_ASSERTED_DLY;
		rp_writel(data, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA, index);

		/* take care of link speed change error in corner cases */
		data = rp_readl(NV_PCIE2_RP_VEND_CTL0, index);
		data &= ~PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH_MASK;
		data |= PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH;
		rp_writel(data, NV_PCIE2_RP_VEND_CTL0, index);

		/* Avoid below register programming for >13 MHz clk_25m Freq */
		if (clk_get_rate(clk_get_sys(NULL, "clk_m")) > 13000000)
			return;
		data = rp_readl(NV_PCIE2_RP_TIMEOUT0, index);
		data &= ~PCIE2_RP_TIMEOUT0_PAD_PWRUP_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_PWRUP;
		data &= ~PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM;
		data &= ~PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2;
		rp_writel(data, NV_PCIE2_RP_TIMEOUT0, index);

		data = rp_readl(NV_PCIE2_RP_TIMEOUT1, index);
		data &= ~PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE_MASK;
		data |= PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE;
		data &= ~PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE_MASK;
		data |= PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE;
		rp_writel(data, NV_PCIE2_RP_TIMEOUT1, index);

		data = rp_readl(NV_PCIE2_RP_XP_REF, index);
		data &= ~PCIE2_RP_XP_REF_MICROSECOND_LIMIT_MASK;
		data |= PCIE2_RP_XP_REF_MICROSECOND_LIMIT;
		data |= PCIE2_RP_XP_REF_MICROSECOND_ENABLE;
		data |= PCIE2_RP_XP_REF_CPL_TO_OVERRIDE;
		data &= ~PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE_MASK;
		data |= PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE;
		rp_writel(data, NV_PCIE2_RP_XP_REF, index);

		data = rp_readl(NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA, index);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY;
		rp_writel(data, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA, index);

		data = rp_readl(NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA, index);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY;
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND;
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP;
		rp_writel(data, NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA, index);
#endif
	}
}

/* Enable various features of root port */
static void tegra_pcie_enable_rp_features(int index)
{
	unsigned int data;

	PR_FUNC_LINE;
	/* Power mangagement settings */
	/* Enable clock clamping by default and enable card detect */
	data = rp_readl(NV_PCIE2_RP_PRIV_MISC, index);
	data |= PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_THRESHOLD |
		PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_ENABLE |
		PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_THRESHOLD |
		PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_ENABLE;
	rp_writel(data, NV_PCIE2_RP_PRIV_MISC, index);

	/* Enable ASPM - L1 state support by default */
	data = rp_readl(NV_PCIE2_RP_VEND_XP1, index);
	data |= NV_PCIE2_RP_VEND_XP1_LINK_PVT_CTL_L1_ASPM_SUPPORT;
	rp_writel(data, NV_PCIE2_RP_VEND_XP1, index);

	/* LTSSM wait for DLLP to finish before entering L1 or L2/L3 */
	/* to avoid truncating of PM mesgs resulting in reciever errors */
	data = rp_readl(NV_PCIE2_RP_VEND_XP_BIST, index);
	data |= PCIE2_RP_VEND_XP_BIST_GOTO_L1_L2_AFTER_DLLP_DONE;
	rp_writel(data, NV_PCIE2_RP_VEND_XP_BIST, index);

	/* unhide AER capability */
	tegra_pcie_enable_aer(index, true);

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	/* program timers for L1 substate support */
	/* set cm_rtime = 100us and t_pwr_on = 70us as per HW team */
	data = rp_readl(NV_PCIE2_RP_L1_PM_SUBSTATES_CYA, index);
	data &= ~PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_MASK;
	data |= (0x64 << PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_SHIFT);
	rp_writel(data, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA, index);

	data = rp_readl(NV_PCIE2_RP_L1_PM_SUBSTATES_CYA, index);
	data &= ~(PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_MASK |
		PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_MASK);
	data |= (1 << PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_SHIFT) |
		(7 << PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_SHIFT);
	rp_writel(data, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA, index);
#endif
	tegra_pcie_apply_sw_war(index, false);
}

static void tegra_pcie_disable_ctlr(int index)
{
	u32 data;

	PR_FUNC_LINE;
	data = afi_readl(AFI_PCIE_CONFIG);
	if (index)
		data |= AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE;
	else
		data |= AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE;
	afi_writel(data, AFI_PCIE_CONFIG);
}

static void tegra_pcie_add_port(int index, u32 offset, u32 reset_reg)
{
	struct tegra_pcie_port *pp;

	PR_FUNC_LINE;
	tegra_pcie_prsnt_map_override(index, true);

	pp = tegra_pcie.port + tegra_pcie.num_ports;
	pp->index = -1;
	pp->base = tegra_pcie.regs + offset;
	pp->link_up = tegra_pcie_check_link(pp, index, reset_reg);

	if (!pp->link_up) {
		pp->base = NULL;
		pr_info("PCIE: port %d: link down, ignoring\n", index);
		tegra_pcie_disable_ctlr(index);
		return;
	}
	tegra_pcie_enable_rp_features(index);

	tegra_pcie.num_ports++;
	pp->index = index;
	/* initialize root bus in boot path only */
	if (!resume_path)
		pp->root_bus_nr = -1;
}

void tegra_pcie_check_ports(void)
{
	int port, rp_offset = 0;
	int ctrl_offset = AFI_PEX0_CTRL;

	PR_FUNC_LINE;
	/* reset number of ports */
	tegra_pcie.num_ports = 0;

	for (port = 0; port < MAX_PCIE_SUPPORTED_PORTS; port++) {
		ctrl_offset += (port * 8);
		rp_offset = (rp_offset + RP_OFFSET) * port;
		if (tegra_pcie.plat_data->port_status[port])
			tegra_pcie_add_port(port, rp_offset, ctrl_offset);
	}
}
EXPORT_SYMBOL(tegra_pcie_check_ports);

int tegra_pcie_get_test_info(void __iomem **regs)
{
	*regs = tegra_pcie.regs;
	return tegra_pcie.num_ports;
}
EXPORT_SYMBOL(tegra_pcie_get_test_info);

static int tegra_pcie_conf_gpios(void)
{
	int irq, err = 0;

	PR_FUNC_LINE;
	if (gpio_is_valid(tegra_pcie.plat_data->gpio_hot_plug)) {
		/* configure gpio for hotplug detection */
		dev_info(tegra_pcie.dev, "acquiring hotplug_detect = %d\n",
				tegra_pcie.plat_data->gpio_hot_plug);
		err = devm_gpio_request(tegra_pcie.dev,
				tegra_pcie.plat_data->gpio_hot_plug,
				"pcie_hotplug_detect");
		if (err < 0) {
			dev_err(tegra_pcie.dev, "%s: gpio_request failed %d\n",
					__func__, err);
			return err;
		}
		err = gpio_direction_input(
				tegra_pcie.plat_data->gpio_hot_plug);
		if (err < 0) {
			dev_err(tegra_pcie.dev,
				"%s: gpio_direction_input failed %d\n",
				__func__, err);
			return err;
		}
		irq = gpio_to_irq(tegra_pcie.plat_data->gpio_hot_plug);
		if (irq < 0) {
			dev_err(tegra_pcie.dev,
				"Unable to get irq for hotplug_detect\n");
			return err;
		}
		err = devm_request_irq(tegra_pcie.dev, (unsigned int)irq,
				gpio_pcie_detect_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"pcie_hotplug_detect",
				(void *)tegra_pcie.plat_data);
		if (err < 0) {
			dev_err(tegra_pcie.dev,
				"Unable to claim irq for hotplug_detect\n");
			return err;
		}
	}
	if (gpio_is_valid(tegra_pcie.plat_data->gpio_x1_slot)) {
		err = devm_gpio_request(tegra_pcie.dev,
			tegra_pcie.plat_data->gpio_x1_slot, "pcie_x1_slot");
		if (err < 0) {
			dev_err(tegra_pcie.dev,
				"%s: pcie_x1_slot gpio_request failed %d\n",
				__func__, err);
			return err;
		}
		err = gpio_direction_output(
			tegra_pcie.plat_data->gpio_x1_slot, 1);
		if (err < 0) {
			dev_err(tegra_pcie.dev,
				"%s: pcie_x1_slot gpio_direction_output failed %d\n",
					__func__, err);
			return err;
		}
		gpio_set_value_cansleep(
			tegra_pcie.plat_data->gpio_x1_slot, 1);
	}
	if (gpio_is_valid(tegra_pcie.plat_data->gpio_wake)) {
		err = devm_gpio_request(tegra_pcie.dev,
				tegra_pcie.plat_data->gpio_wake, "pcie_wake");
		if (err < 0) {
			dev_err(tegra_pcie.dev,
				"%s: pcie_wake gpio_request failed %d\n",
				__func__, err);
			return err;
		}
		err = gpio_direction_input(
				tegra_pcie.plat_data->gpio_wake);
		if (err < 0) {
			dev_err(tegra_pcie.dev,
				"%s: pcie_wake gpio_direction_input failed %d\n",
					__func__, err);
			return err;
		}
	}
	return 0;
}

static int tegra_pcie_scale_voltage(bool isGen2)
{
	int err = 0;

	PR_FUNC_LINE;
	if (isGen2) {
		if (tegra_pcie_xclk_rate == TEGRA_PCIE_XCLK_500 &&
			tegra_pcie_mselect_rate == TEGRA_PCIE_MSELECT_CLK_408)
			goto skip;
		/* Scale up voltage for Gen2 speed */
		tegra_pcie_xclk_rate = TEGRA_PCIE_XCLK_500;
		tegra_pcie_mselect_rate = TEGRA_PCIE_MSELECT_CLK_408;
	} else {
		if (tegra_pcie_xclk_rate == TEGRA_PCIE_XCLK_250 &&
			tegra_pcie_mselect_rate == TEGRA_PCIE_MSELECT_CLK_204)
			goto skip;
		/* Scale down voltage for Gen1 speed */
		tegra_pcie_xclk_rate = TEGRA_PCIE_XCLK_250;
		tegra_pcie_mselect_rate = TEGRA_PCIE_MSELECT_CLK_204;
	}
	err = clk_set_rate(tegra_pcie.pcie_xclk, tegra_pcie_xclk_rate);
	if (err)
		return err;
	err = clk_set_rate(tegra_pcie.pcie_mselect, tegra_pcie_mselect_rate);
skip:
	return err;

}

static bool tegra_pcie_change_link_speed(struct pci_dev *pdev, bool isGen2)
{
	u16 val, link_sts_up_spd, link_sts_dn_spd;
	u16 link_cap_up_spd, link_cap_dn_spd;
	struct pci_dev *up_dev, *dn_dev;

	PR_FUNC_LINE;
	/* skip if current device is not PCI express capable */
	/* or is either a root port or downstream port */
	if (!pci_is_pcie(pdev))
		goto skip;
	if ((pci_pcie_type(pdev) == PCI_EXP_TYPE_DOWNSTREAM) ||
		(pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT))
		goto skip;

	/* initialize upstream/endpoint and downstream/root port device ptr */
	up_dev = pdev;
	dn_dev = pdev->bus->self;

	/* read link status register to find current speed */
	pcie_capability_read_word(up_dev, PCI_EXP_LNKSTA, &link_sts_up_spd);
	link_sts_up_spd &= PCI_EXP_LNKSTA_CLS;
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKSTA, &link_sts_dn_spd);
	link_sts_dn_spd &= PCI_EXP_LNKSTA_CLS;
	/* read link capability register to find max speed supported */
	pcie_capability_read_word(up_dev, PCI_EXP_LNKCAP, &link_cap_up_spd);
	link_cap_up_spd &= PCI_EXP_LNKCAP_SLS;
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKCAP, &link_cap_dn_spd);
	link_cap_dn_spd &= PCI_EXP_LNKCAP_SLS;
	/* skip if both devices across the link are already trained to gen2 */
	if (isGen2) {
		if (((link_cap_up_spd >= PCI_EXP_LNKSTA_CLS_5_0GB) &&
			(link_cap_dn_spd >= PCI_EXP_LNKSTA_CLS_5_0GB)) &&
			((link_sts_up_spd != PCI_EXP_LNKSTA_CLS_5_0GB) ||
			 (link_sts_dn_spd != PCI_EXP_LNKSTA_CLS_5_0GB)))
			goto change;
		else
			goto skip;
	} else {
		/* gen1 should be supported by default by all pcie cards */
		if ((link_sts_up_spd != PCI_EXP_LNKSTA_CLS_2_5GB) ||
			 (link_sts_dn_spd != PCI_EXP_LNKSTA_CLS_2_5GB))
			goto change;
		else
			goto skip;
	}

change:
	if (tegra_pcie_scale_voltage(isGen2))
		goto skip;
	/* Set Link Speed */
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKCTL2, &val);
	val &= ~PCI_EXP_LNKSTA_CLS;
	if (isGen2)
		val |= PCI_EXP_LNKSTA_CLS_5_0GB;
	else
		val |= PCI_EXP_LNKSTA_CLS_2_5GB;
	pcie_capability_write_word(dn_dev, PCI_EXP_LNKCTL2, val);

	/* Retrain the link */
	pcie_capability_read_word(dn_dev, PCI_EXP_LNKCTL, &val);
	val |= PCI_EXP_LNKCTL_RL;
	pcie_capability_write_word(dn_dev, PCI_EXP_LNKCTL, val);

	return true;
skip:
	return false;
}

bool tegra_pcie_link_speed(bool isGen2)
{
	struct pci_dev *pdev = NULL;
	bool ret = false;

	PR_FUNC_LINE;
	/* Voltage scaling should happen before any device transition */
	/* to Gen2 or after all devices has transitioned to Gen1 */
	for_each_pci_dev(pdev) {
		if (tegra_pcie_change_link_speed(pdev, isGen2))
			ret = true;
	}
	return ret;
}
EXPORT_SYMBOL(tegra_pcie_link_speed);

/* support PLL power down in L1 dynamically based on platform */
static void tegra_pcie_pll_pdn(void)
{
	static struct pinctrl_dev *pctl_dev = NULL;
	struct pci_dev *pdev = NULL;

	if (!pctl_dev)
		pctl_dev = pinctrl_get_dev_from_of_compatible(
					pinctrl_compatible);
	if (!pctl_dev) {
		pr_err("%s(): tegra pincontrol does not found\n", __func__);
		return;
	}

	PR_FUNC_LINE;
	/* CLKREQ# to PD if device connected to RP doesn't have CLKREQ# */
	/* capability(no PLL power down in L1 here) and PU if they have */
	for_each_pci_dev(pdev) {
		if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
			continue;

		if ((pci_pcie_type(pdev->bus->self) ==
			PCI_EXP_TYPE_ROOT_PORT)) {
			u32 val = 0;
			unsigned long config;

			pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &val);
			if (val & PCI_EXP_LNKCAP_CLKPM) {
				config = TEGRA_PINCONF_PACK(
						TEGRA_PINCONF_PARAM_PULL,
						TEGRA_PIN_PULL_UP);
			} else {
				config = TEGRA_PINCONF_PACK(
						TEGRA_PINCONF_PARAM_PULL,
						TEGRA_PIN_PULL_DOWN);
			}
			pinctrl_set_config_for_group_name(pctl_dev,
					pin_pex_l0_clkreq, config);
			pinctrl_set_config_for_group_name(pctl_dev,
					pin_pex_l1_clkreq, config);
			break;
		}
	}
}

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
static void tegra_pcie_config_l1ss_tpwr_on(void)
{
	struct pci_dev *pdev = NULL;
	u32 data = 0, data1 = 0, data2 = 0, pos = 0;
	unsigned long max1 = 0, max2 = 0;

	PR_FUNC_LINE;
	/* find max T_POWER_ON reported by RP & EP capability regs */
	/* and program same in ctrl2 reg of both RP & EP */
	for_each_pci_dev(pdev) {
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT) {
			pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
			pci_read_config_dword(pdev->bus->self,
				pos + PCI_L1SS_CAP, &data1);
			max1 = (((data1 & PCI_L1SS_CAP_PWRN_SCL_MASK) >>
				PCI_L1SS_CAP_PWRN_SCL_SHIFT) *
				((data1 & PCI_L1SS_CAP_PWRN_VAL_MASK) >>
				PCI_L1SS_CAP_PWRN_VAL_SHIFT));
			pci_read_config_dword(pdev, pos + PCI_L1SS_CAP, &data2);
			max2 = (((data2 & PCI_L1SS_CAP_PWRN_SCL_MASK) >>
				PCI_L1SS_CAP_PWRN_SCL_SHIFT) *
				((data2 & PCI_L1SS_CAP_PWRN_VAL_MASK) >>
				PCI_L1SS_CAP_PWRN_VAL_SHIFT));
			if (max1 > max2)
				data = (data1 & PCI_L1SS_CAP_PWRN_VS_MASK) >>
					PCI_L1SS_CAP_PWRN_SCL_SHIFT;
			else
				data = (data2 & PCI_L1SS_CAP_PWRN_VS_MASK) >>
					PCI_L1SS_CAP_PWRN_SCL_SHIFT;

			pci_write_config_dword(pdev,
				pos + PCI_L1SS_CTRL2, data);
			pci_write_config_dword(pdev->bus->self,
				pos + PCI_L1SS_CTRL2, data);
		}
	}
}

static void tegra_pcie_config_l1ss_cm_rtime(void)
{
	struct pci_dev *pdev = NULL;
	u32 data = 0, data1 = 0, max[MAX_PCIE_SUPPORTED_PORTS] = {0};
	int i = -1, pos = 0;

	PR_FUNC_LINE;
	/* find max of common mode restore time reported by all */
	/* devices including RP in capability register, and set same */
	/* in control 1 register after substracting t_pwr_on for both RP & EP */
	for_each_pci_dev(pdev) {
		if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
			i++;
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CAP, &data);
		data &= PCI_L1SS_CAP_CM_RTM_MASK;
		if (max[i] < data)
			max[i] = data;
	}
	i = 0;
	for_each_pci_dev(pdev) {
		if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT) {
			pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
			pci_read_config_dword(pdev,
				pos + PCI_L1SS_CTRL1, &data);
			data &= ~PCI_L1SS_CAP_CM_RTM_MASK;

			pci_read_config_dword(pdev,
				pos + PCI_L1SS_CTRL2, &data1);
			data1 = (data1 & PCI_L1SS_CTRL2_PWRN_SCL_MASK) *
				((data1 & PCI_L1SS_CTRL2_PWRN_VAL_MASK) >>
				PCI_L1SS_CTRL2_PWRN_VAL_SHIFT);
			data |= max[i++] - (data1 << PCI_L1SS_CAP_CM_RTM_SHIFT);
			pci_write_config_dword(pdev,
				pos + PCI_L1SS_CTRL1, data);
		}
	}
}

static void tegra_pcie_config_l1ss_l12_thtime(void)
{
	struct pci_dev *pdev = NULL;
	u32 data = 0, pos = 0;

	PR_FUNC_LINE;
	/* program same LTR L1.2 threshold = 106us for all devices */
	for_each_pci_dev(pdev) {
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CTRL1, &data);
		data |= 0x6A << PCI_L1SS_CTRL1_L12TH_VAL_SHIFT;
		pci_write_config_dword(pdev, pos + PCI_L1SS_CTRL1, data);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CTRL1, &data);
		data |= 0x02 << PCI_L1SS_CTRL1_L12TH_SCALE_SHIFT;
		pci_write_config_dword(pdev, pos + PCI_L1SS_CTRL1, data);
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
}

static void tegra_pcie_enable_l1ss_support(void)
{
	struct pci_dev *pdev = NULL;
	u32 aspm = 0, data = 0, pos = 0;

	PR_FUNC_LINE;
	for_each_pci_dev(pdev) {
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		/* enable L1 substate as per device capability */
		pci_read_config_dword(pdev, pos + PCI_L1SS_CAP, &aspm);
		pci_read_config_dword(pdev, pos + PCI_L1SS_CTRL1, &data);
		data &= ~PCI_L1SS_CAP_L1PM_MASK;
		data |= (aspm & PCI_L1SS_CAP_L1PM_MASK);
		pci_write_config_dword(pdev, pos + PCI_L1SS_CTRL1, data);
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
}

static void tegra_pcie_enable_ltr_support(void)
{
	struct pci_dev *pdev = NULL;
	u16 val = 0;
	u32 data = 0;

	PR_FUNC_LINE;
	/* enable LTR mechanism for L1.2 support */
	for_each_pci_dev(pdev) {
		pcie_capability_read_dword(pdev, PCI_EXP_DEVCAP2, &data);
		if (data & PCI_EXP_DEVCAP2_LTR) {
			pcie_capability_read_word(pdev, PCI_EXP_DEVCTL2, &val);
			val |= PCI_EXP_LTR_EN;
			pcie_capability_write_word(pdev, PCI_EXP_DEVCTL2, val);
		}
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
}

static void tegra_pcie_config_clkreq(bool enable)
{
	static struct pinctrl_dev *pctl_dev = NULL;
	unsigned long od_conf, tr_conf;

	if (!pctl_dev)
		pctl_dev = pinctrl_get_dev_from_of_compatible(
				pinctrl_compatible);
	if (!pctl_dev) {
		pr_err("%s(): tegra pincontrol does not found\n", __func__);
		return;
	}
	if (enable) {
		od_conf = TEGRA_PINCONF_PACK(TEGRA_PINCONF_PARAM_OPEN_DRAIN,
					TEGRA_PIN_ENABLE);
		tr_conf = TEGRA_PINCONF_PACK(TEGRA_PINCONF_PARAM_TRISTATE,
					TEGRA_PIN_DISABLE);
	} else {
		od_conf = TEGRA_PINCONF_PACK(TEGRA_PINCONF_PARAM_OPEN_DRAIN,
					TEGRA_PIN_DISABLE);
		tr_conf = TEGRA_PINCONF_PACK(TEGRA_PINCONF_PARAM_TRISTATE,
					TEGRA_PIN_ENABLE);
	}
	if (enable) {
		/* Make CLKREQ# bi-directional if L1PM SS are enabled */
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l0_clkreq, tr_conf);
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l0_clkreq, od_conf);
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l1_clkreq, tr_conf);
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l1_clkreq, od_conf);
	} else {
		struct pci_dev *pdev = NULL;
		u16 val = 0;

		/* Make CLKREQ# input only if L1PM SS is disabled later */
		/* also disable ASPM L1 momentarily before doing this */
		for_each_pci_dev(pdev) {
			pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &val);
			val &= ~PCI_EXP_LNKCTL_ASPM_L1;
			pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, val);
		}
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l0_clkreq, tr_conf);
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l0_clkreq, od_conf);
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l1_clkreq, tr_conf);
		pinctrl_set_config_for_group_name(pctl_dev,
				pin_pex_l1_clkreq, od_conf);
		for_each_pci_dev(pdev) {
			pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &val);
			val |= PCI_EXP_LNKCTL_ASPM_L1;
			pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, val);
		}
	}
}

#endif

/* Enable ASPM support of all devices based on it's capability */
static void tegra_pcie_enable_aspm(void)
{
	struct pci_dev *pdev = NULL;
	u16 val = 0;
	u32 aspm = 0;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	int pos = 0;
	bool config_l1ss = true;
#endif

	PR_FUNC_LINE;
	if (!pcie_aspm_support_enabled()) {
		pr_info("PCIE: ASPM not enabled\n");
		return;
	}
	for_each_pci_dev(pdev) {
		/* Find ASPM capability */
		pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &aspm);
		aspm &= PCI_EXP_LNKCAP_ASPMS;

		/* Enable ASPM support as per capability */
		pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &val);
		val |= (u16)aspm >> 10;
		pcie_capability_write_word(pdev, PCI_EXP_LNKCTL, val);
#if defined CONFIG_ARCH_TEGRA_12x_SOC
		pcie_capability_clear_word(pdev, PCI_EXP_LNKCTL,
				PCI_EXP_LNKCTL_ASPM_L0S);
#endif
	}
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	/* L1SS configuration as per IAS */
	for_each_pci_dev(pdev) {
		/* check if L1SS capability is supported in current device */
		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_L1SS);
		if (!pos) {
			config_l1ss = false;
			break;
		}
		/* avoid L1SS config if no support of L1PM substate feature */
		pci_read_config_dword(pdev, pos + PCI_L1SS_CAP, &aspm);
		if (((aspm & PCI_L1SS_CAP_L1PMS) == 0) ||
			((aspm & PCI_L1SS_CAP_L1PM_MASK) == 0)) {
			config_l1ss = false;
			break;
		}
		if (pci_pcie_type(pdev) != PCI_EXP_TYPE_ROOT_PORT)
			break;
	}
	if (config_l1ss) {
		tegra_pcie_config_clkreq(true);
		tegra_pcie_config_l1ss_tpwr_on();
		tegra_pcie_config_l1ss_cm_rtime();
		tegra_pcie_config_l1ss_l12_thtime();
		tegra_pcie_enable_l1ss_support();
		tegra_pcie_enable_ltr_support();
	}
#endif
}

static void tegra_pcie_enable_features(void)
{
	PR_FUNC_LINE;

	/* configure all links to gen2 speed by default */
	if (!tegra_pcie_link_speed(true))
		pr_info("PCIE: No Link speed change happened\n");

	tegra_pcie_pll_pdn();
	tegra_pcie_enable_aspm();
	tegra_pcie_apply_sw_war(0, true);
}

static int __init tegra_pcie_init(void)
{
	int err = 0;

	pcibios_min_mem = 0x03000000ul;
	pcibios_min_io = 0x1000ul;

	PR_FUNC_LINE;
	INIT_LIST_HEAD(&tegra_pcie.busses);
	INIT_WORK(&tegra_pcie.hotplug_detect, work_hotplug_handler);
	err = tegra_pcie_get_resources();
	if (err) {
		pr_err("PCIE: get resources failed\n");
		return err;
	}
	err = tegra_pcie_enable_pads(true);
	if (err) {
		pr_err("PCIE: enable pads failed\n");
		tegra_pcie_power_off(false);
		return err;
	}
	err = tegra_pcie_enable_controller();
	if (err) {
		pr_err("PCIE: enable controller failed\n");
		goto fail;
	}
	err = tegra_pcie_conf_gpios();
	if (err) {
		pr_err("PCIE: configuring gpios failed\n");
		goto fail;
	}
	/* setup the AFI address translations */
	tegra_pcie_setup_translations();
	tegra_pcie_check_ports();

	if (tegra_pcie.num_ports)
		pci_common_init(&tegra_pcie_hw);
	else {
		pr_err("PCIE: no ports detected\n");
		goto fail;
	}
	tegra_pcie_enable_features();
	/* register pcie device as wakeup source */
	device_init_wakeup(tegra_pcie.dev, true);

	return 0;
fail:
	tegra_pcie_power_off(true);
	return err;
}

static void tegra_pcie_read_plat_data(void)
{
	struct device_node *node = tegra_pcie.dev->of_node;

	PR_FUNC_LINE;
	of_property_read_u32(node, "nvidia,port0_status",
			&tegra_pcie.plat_data->port_status[0]);
	of_property_read_u32(node, "nvidia,port1_status",
			&tegra_pcie.plat_data->port_status[1]);
	tegra_pcie.plat_data->gpio_hot_plug =
		of_get_named_gpio(node, "nvidia,hot-plug-gpio", 0);
	tegra_pcie.plat_data->gpio_wake =
		of_get_named_gpio(node, "nvidia,wake-gpio", 0);
	tegra_pcie.plat_data->gpio_x1_slot =
		of_get_named_gpio(node, "nvidia,x1-slot-gpio", 0);
	tegra_pcie.plat_data->has_clkreq =
		of_property_read_bool(node, "nvidia,has_clkreq");
	tegra_pcie.plat_data->has_memtype_lpddr4 =
		of_property_read_bool(node, "nvidia,has_memtype_lpddr4");
}

static char *t124_rail_names[] = {"hvdd-pex", "hvdd-pex-pll-e", "dvddio-pex",
				"avddio-pex", "avdd-pex-pll", "vddio-pex-ctl"};

static char *t210_rail_names[] = {"dvdd-pex-pll", "hvdd-pex-pll-e",
					"l0-hvddio-pex", "l0-dvddio-pex",
					"l1-hvddio-pex", "l1-dvddio-pex",
					"l2-hvddio-pex", "l2-dvddio-pex",
					"l3-hvddio-pex", "l3-dvddio-pex",
					"l4-hvddio-pex", "l4-dvddio-pex",
					"l5-hvddio-pex", "l5-dvddio-pex",
					"l6-hvddio-pex", "l6-dvddio-pex",
					"vddio-pex-ctl"};

static struct tegra_pcie_chipdata tegra124_pcie_chipdata = {
	.pcie_regulator_names = t124_rail_names,
	.num_pcie_regulators =
			sizeof(t124_rail_names) / sizeof(t124_rail_names[0]),
};

static struct tegra_pcie_chipdata tegra210_pcie_chipdata = {
	.pcie_regulator_names = t210_rail_names,
	.num_pcie_regulators =
			sizeof(t210_rail_names) / sizeof(t210_rail_names[0]),
};

static struct of_device_id tegra_pcie_of_match[] = {
	{ .compatible = "nvidia,tegra210-pcie",
		.data = &tegra210_pcie_chipdata, },
	{ .compatible = "nvidia,tegra124-pcie",
		.data = &tegra124_pcie_chipdata, },
	{ }
};

static int __init tegra_pcie_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	const struct of_device_id *match;
	const struct tegra_pcie_chipdata *chip_data = NULL;

	PR_FUNC_LINE;
	tegra_pcie.dev = &pdev->dev;
	if (tegra_pcie.dev->of_node) {
		match = of_match_device(of_match_ptr(tegra_pcie_of_match),
								&pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Device Not matching\n");
			return -ENODEV;
		}
		chip_data = match->data;
		tegra_pcie.chipdata = match->data;

		/* use DT way to init platform data */
		tegra_pcie.plat_data = devm_kzalloc(tegra_pcie.dev,
			sizeof(*tegra_pcie.plat_data), GFP_KERNEL);
		if (!tegra_pcie.plat_data) {
			dev_err(tegra_pcie.dev, "memory alloc failed\n");
			return -ENOMEM;
		}
		tegra_pcie_read_plat_data();

		tegra_pcie.pcie_regulators = devm_kzalloc(tegra_pcie.dev,
			chip_data->num_pcie_regulators
				* sizeof(struct regulator *), GFP_KERNEL);

		for (i = 0; i < tegra_pcie.chipdata->num_pcie_regulators; i++) {
			tegra_pcie.pcie_regulators[i] =
						regulator_get(tegra_pcie.dev,
				tegra_pcie.chipdata->pcie_regulator_names[i]);
			if (IS_ERR(tegra_pcie.pcie_regulators[i])) {
				pr_err("%s: unable to get regulator %s\n",
				__func__,
				tegra_pcie.chipdata->pcie_regulator_names[i]);
				tegra_pcie.pcie_regulators[i] = NULL;
			}
		}

	}
	dev_dbg(tegra_pcie.dev, "PCIE.C: %s : _port_status[0] %d\n",
		__func__, tegra_pcie.plat_data->port_status[0]);
	dev_dbg(tegra_pcie.dev, "PCIE.C: %s : _port_status[1] %d\n",
		__func__, tegra_pcie.plat_data->port_status[1]);

	/* Enable Runtime PM for PCIe, TODO: Need to add PCIe host device */
	pm_runtime_enable(tegra_pcie.dev);

	ret = tegra_pcie_init();
	if (ret)
		tegra_pd_remove_device(tegra_pcie.dev);

	return ret;
}

#ifdef CONFIG_PM
static int tegra_pcie_suspend_noirq(struct device *dev)
{
	int ret = 0;

	PR_FUNC_LINE;
	/* configure PE_WAKE signal as wake sources */
	if (gpio_is_valid(tegra_pcie.plat_data->gpio_wake) &&
			device_may_wakeup(dev)) {
		ret = enable_irq_wake(gpio_to_irq(
			tegra_pcie.plat_data->gpio_wake));
		if (ret < 0) {
			dev_err(dev,
				"ID wake-up event failed with error %d\n", ret);
			return ret;
		}
	}
	return tegra_pcie_power_off(true);
}

static bool tegra_pcie_enable_msi(bool);

static int tegra_pcie_resume_noirq(struct device *dev)
{
	int ret = 0;

	PR_FUNC_LINE;
	resume_path = true;

	if (gpio_is_valid(tegra_pcie.plat_data->gpio_wake) &&
			device_may_wakeup(dev)) {
		ret = disable_irq_wake(gpio_to_irq(
			tegra_pcie.plat_data->gpio_wake));
		if (ret < 0) {
			dev_err(dev,
				"ID wake-up event failed with error %d\n", ret);
			return ret;
		}
	}
	/* give 100ms for 1.05v to come up */
	msleep(100);
	ret = tegra_pcie_power_on();
	if (ret) {
		pr_err("PCIE: Failed to power on: %d\n", ret);
		return ret;
	}
	tegra_pcie_enable_pads(true);
	tegra_pcie_enable_controller();
	tegra_pcie_setup_translations();
	/* Set up MSI registers, if MSI have been enabled */
	tegra_pcie_enable_msi(true);

	tegra_pcie_check_ports();
	if (!tegra_pcie.num_ports) {
		tegra_pcie_power_off(true);
		goto exit;
	}
	resume_path = false;

exit:
	return 0;
}

static int tegra_pcie_resume(struct device *dev)
{
	PR_FUNC_LINE;
	tegra_pcie_enable_features();
	return 0;
}
#endif

static int tegra_pcie_remove(struct platform_device *pdev)
{
	struct tegra_pcie_bus *bus;

	PR_FUNC_LINE;
	list_for_each_entry(bus, &tegra_pcie.busses, list) {
		vunmap(bus->area->addr);
		kfree(bus);
	}
	tegra_pcie_detach();
	tegra_pd_remove_device(tegra_pcie.dev);

	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops tegra_pcie_pm_ops = {
	.suspend_noirq  = tegra_pcie_suspend_noirq,
	.resume_noirq = tegra_pcie_resume_noirq,
	.resume = tegra_pcie_resume,
	};
#endif

/* driver data is accessed after init, so use __refdata instead of __initdata */
static struct platform_driver __refdata tegra_pcie_driver = {
	.probe   = tegra_pcie_probe,
	.remove  = tegra_pcie_remove,
	.driver  = {
		.name  = "tegra-pcie",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &tegra_pcie_pm_ops,
#endif
		.of_match_table = tegra_pcie_of_match,
	},
};

static int __init tegra_pcie_init_driver(void)
{
	if (tegra_platform_is_linsim() || tegra_platform_is_qt())
		return 0;
	return platform_driver_register(&tegra_pcie_driver);
}

static void __exit_refok tegra_pcie_exit_driver(void)
{
	if (tegra_platform_is_linsim() || tegra_platform_is_qt())
		return;
	platform_driver_unregister(&tegra_pcie_driver);
}

module_init(tegra_pcie_init_driver);
module_exit(tegra_pcie_exit_driver);

static struct irq_chip tegra_irq_chip_msi_pcie = {
	.name = "PCIe-MSI",
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
};

/* 1:1 matching of these to the MSI vectors, 1 per bit */
/* and each mapping matches one of the available interrupts */
/*   irq should equal INT_PCI_MSI_BASE + index */
struct msi_map_entry {
	bool used;
	u8 index;
	int irq;
};

/* hardware supports 256 max*/
#if (INT_PCI_MSI_NR > 256)
#error "INT_PCI_MSI_NR too big"
#endif

#define MSI_MAP_SIZE  (INT_PCI_MSI_NR)
static struct msi_map_entry msi_map[MSI_MAP_SIZE];

static void msi_map_init(void)
{
	int i;

	for (i = 0; i < MSI_MAP_SIZE; i++) {
		msi_map[i].used = false;
		msi_map[i].index = i;
		msi_map[i].irq = 0;
	}
}

/* returns an index into the map*/
static struct msi_map_entry *msi_map_get(void)
{
	struct msi_map_entry *retval = NULL;
	int i;

	for (i = 0; i < MSI_MAP_SIZE; i++) {
		if (!msi_map[i].used) {
			retval = msi_map + i;
			retval->irq = INT_PCI_MSI_BASE + i;
			retval->used = true;
			break;
		}
	}

	return retval;
}

void msi_map_release(struct msi_map_entry *entry)
{
	if (entry) {
		entry->used = false;
		entry->irq = 0;
	}
}

static irqreturn_t tegra_pcie_msi_isr(int irq, void *arg)
{
	int i, offset, index;
	static int count;
	u32 reg;

	/* suppress print spews in debug mode */
	if (!count) {
		PR_FUNC_LINE;
		count = 10;
	}
	count--;
	for (i = 0; i < 8; i++) {
		reg = afi_readl(AFI_MSI_VEC0_0 + i * 4);
		while (reg != 0x00000000) {
			offset = find_first_bit((unsigned long int *)&reg, 32);
			index = i * 32 + offset;
			/* clear the interrupt */
			afi_writel(1ul << index, AFI_MSI_VEC0_0 + i * 4);
			if (index < MSI_MAP_SIZE) {
				if (msi_map[index].used)
					generic_handle_irq(msi_map[index].irq);
				else
					pr_info("unexpected MSI (1)\n");
			} else {
				/* that's weird who triggered this?*/
				/* just clear it*/
				pr_info("unexpected MSI (2)\n");
			}
			/* see if there's any more pending in this vector */
			reg = afi_readl(AFI_MSI_VEC0_0 + i * 4);
		}
	}

	return IRQ_HANDLED;
}

static bool tegra_pcie_enable_msi(bool no_init)
{
	u32 reg;
	static uintptr_t msi_base;

	PR_FUNC_LINE;
	if (!msi_base) {
		/* if not already initialized and no_init, nothing to do */
		if (no_init)
			return true;

		msi_map_init();

		/* enables MSI interrupts.  */
		if (request_irq(INT_PCIE_MSI, tegra_pcie_msi_isr,
			IRQF_SHARED, "PCIe-MSI", tegra_pcie_msi_isr)) {
			pr_err("%s: Cannot register IRQ %u\n",
					__func__, INT_PCIE_MSI);
			return false;
		}
		/* setup AFI/FPCI range */
		/* FIXME do this better! should be based on PAGE_SIZE */
		msi_base = __get_free_pages(GFP_KERNEL, 3);
		if (!msi_base) {
			pr_err("PCIE: Insufficient memory\n");
			return false;
		}
		msi_base = virt_to_phys((void *)msi_base);
	}

	afi_writel(msi_base>>8, AFI_MSI_FPCI_BAR_ST);
	afi_writel(msi_base, AFI_MSI_AXI_BAR_ST);
	/* this register is in 4K increments */
	afi_writel(1, AFI_MSI_BAR_SZ);

	/* enable all MSI vectors */
	afi_writel(0xffffffff, AFI_MSI_EN_VEC0_0);
	afi_writel(0xffffffff, AFI_MSI_EN_VEC1_0);
	afi_writel(0xffffffff, AFI_MSI_EN_VEC2_0);
	afi_writel(0xffffffff, AFI_MSI_EN_VEC3_0);
	afi_writel(0xffffffff, AFI_MSI_EN_VEC4_0);
	afi_writel(0xffffffff, AFI_MSI_EN_VEC5_0);
	afi_writel(0xffffffff, AFI_MSI_EN_VEC6_0);
	afi_writel(0xffffffff, AFI_MSI_EN_VEC7_0);

	/* and unmask the MSI interrupt */
	reg = 0;
	reg |= (AFI_INTR_MASK_INT_MASK | AFI_INTR_MASK_MSI_MASK);
	afi_writel(reg, AFI_INTR_MASK);

	set_irq_flags(INT_PCIE_MSI, IRQF_VALID);

	return true;
}


/* called by arch_setup_msi_irqs in drivers/pci/msi.c */
int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int retval = -EINVAL;
	struct msi_msg msg;
	struct msi_map_entry *map_entry = NULL;

	PR_FUNC_LINE;
	if (!tegra_pcie_enable_msi(false))
		goto exit;

	map_entry = msi_map_get();
	if (map_entry == NULL)
		goto exit;

	retval = irq_alloc_desc(0);
	if (retval < 0)
		goto exit;
	map_entry->irq = retval;

	irq_set_chip_and_handler(map_entry->irq,
				&tegra_irq_chip_msi_pcie,
				handle_simple_irq);

	retval = irq_set_msi_desc(map_entry->irq, desc);
	if (retval < 0)
		goto exit;
	set_irq_flags(map_entry->irq, IRQF_VALID);

	msg.address_lo = afi_readl(AFI_MSI_AXI_BAR_ST);
	/* 32 bit address only */
	msg.address_hi = 0;
	msg.data = map_entry->index;

	write_msi_msg(map_entry->irq, &msg);

	retval = 0;
exit:
	if (retval != 0) {
		if (map_entry) {
			irq_free_desc(map_entry->irq);
			msi_map_release(map_entry);
		}
	}

	return retval;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	int i;

	PR_FUNC_LINE;
	for (i = 0; i < MSI_MAP_SIZE; i++) {
		if ((msi_map[i].used) && (msi_map[i].irq == irq)) {
			irq_free_desc(msi_map[i].irq);
			msi_map_release(msi_map + i);
			break;
		}
	}
}
