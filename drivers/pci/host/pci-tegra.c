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
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
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
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
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
#define RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE	0x20000000
#define RP_LINK_CONTROL_STATUS_LINKSTAT_MASK	0x3fff0000

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
#define NV_PCIE2_RP_VEND_XP2					0x00000F08
#define NV_PCIE2_RP_VEND_XP_LINK_PVT_CTL_L1_ASPM_SUPPORT	(1 << 21)

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

struct tegra_pcie_soc_data {
	unsigned int	num_ports;
	char			**pcie_regulator_names;
	int				num_pcie_regulators;
};

struct tegra_msi {
	struct msi_chip chip;
	DECLARE_BITMAP(used, INT_PCI_MSI_NR);
	struct irq_domain *domain;
	unsigned long pages;
	struct mutex lock;
	int irq;
};

static inline struct tegra_msi *to_tegra_msi(struct msi_chip *chip)
{
	return container_of(chip, struct tegra_msi, chip);
}

struct tegra_pcie {
	struct device *dev;

	void __iomem *pads;
	void __iomem *afi;
	int irq;

	struct list_head buses;
	struct list_head sys;
	struct resource *cs;

	struct resource all;
	struct resource io;
	struct resource mem;
	struct resource prefetch;
	struct resource busn;

	struct tegra_msi msi;

	struct clk		*pcie_xclk;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	struct clk		*pex_uphy;
#endif
	struct clk		*pcie_mselect;

	struct list_head ports;
	int num_ports;

	int power_rails_enabled;
	int pcie_power_enabled;
	struct work_struct hotplug_detect;

	struct regulator	**pcie_regulators;

	struct tegra_pci_platform_data *plat_data;
	struct tegra_pcie_soc_data *soc_data;
};

struct tegra_pcie_port {
	struct tegra_pcie *pcie;
	struct list_head list;
	struct resource regs;
	void __iomem *base;
	unsigned int index;
	unsigned int lanes;
	bool has_clkreq;
	int status;
};

struct tegra_pcie_bus {
	struct vm_struct *area;
	struct list_head list;
	unsigned int nr;
};

/* used to avoid successive hotplug disconnect or connect */
static bool hotplug_event;
/* pcie mselect & xclk rate */
static unsigned long tegra_pcie_mselect_rate = TEGRA_PCIE_MSELECT_CLK_204;
static unsigned long tegra_pcie_xclk_rate = TEGRA_PCIE_XCLK_250;

static inline struct tegra_pcie *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static inline void afi_writel(struct tegra_pcie *pcie, u32 value,
							  unsigned long offset)
{
	writel(value, offset + pcie->afi);
}

static inline u32 afi_readl(struct tegra_pcie *pcie, unsigned long offset)
{
	return readl(offset + pcie->afi);
}

static inline void pads_writel(struct tegra_pcie *pcie, u32 value,
							   unsigned long offset)
{
	writel(value, offset + pcie->pads);
}

static inline u32 pads_readl(struct tegra_pcie *pcie, unsigned long offset)
{
	return readl(offset + pcie->pads);
}

static inline void rp_writel(struct tegra_pcie_port *port, u32 value,
							 unsigned long offset)
{
	writel(value, offset + port->base);
}

static inline unsigned int rp_readl(struct tegra_pcie_port *port,
							unsigned long offset)
{
	return readl(offset + port->base);
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

static struct tegra_pcie_bus *tegra_pcie_bus_alloc(struct tegra_pcie *pcie,
							unsigned int busnr)
{
	phys_addr_t cs = pcie->cs->start;
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
			dev_err(pcie->dev, "ioremap_page_range() failed: %d\n",
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
static void __iomem *tegra_pcie_bus_map(struct tegra_pcie *pcie,
							unsigned int busnr)
{
	struct tegra_pcie_bus *bus;

	PR_FUNC_LINE;

	list_for_each_entry(bus, &pcie->buses, list)
		if (bus->nr == busnr)
			return bus->area->addr;

	bus = tegra_pcie_bus_alloc(pcie, busnr);
	if (IS_ERR(bus))
		return NULL;

	list_add_tail(&bus->list, &pcie->buses);

	return bus->area->addr;
}


static void __iomem *tegra_pcie_conf_address(struct pci_bus *bus,
					     unsigned int devfn,
					     int where)
{
	struct tegra_pcie *pcie = sys_to_pcie(bus->sysdata);
	void __iomem *addr = NULL;

	PR_FUNC_LINE;

	if (bus->number == 0) {
		unsigned int slot = PCI_SLOT(devfn);
		struct tegra_pcie_port *port;

		list_for_each_entry(port, &pcie->ports, list) {
			if (port->index + 1 == slot) {
				addr = port->base + (where & ~3);
				break;
			}
		}
	} else {
		addr = tegra_pcie_bus_map(pcie, bus->number);
		if (!addr) {
			dev_err(pcie->dev,
				"failed to map cfg. space for bus %u\n",
				bus->number);
			return NULL;
		}

		addr += tegra_pcie_conf_offset(devfn, where);
	}

	return addr;
}

int tegra_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *value)
{
	void __iomem *addr;

	PR_FUNC_LINE;

	addr = tegra_pcie_conf_address(bus, devfn, where);
	if (!addr) {
		*value = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	*value = readl(addr);

	if (size == 1)
		*value = (*value >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*value = (*value >> (8 * (where & 3))) & 0xffff;

	return PCIBIOS_SUCCESSFUL;
}
EXPORT_SYMBOL(tegra_pcie_read_conf);

static int tegra_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 value)
{
	void __iomem *addr;
	u32 mask, tmp;

	PR_FUNC_LINE;

	addr = tegra_pcie_conf_address(bus, devfn, where);
	if (!addr)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (size == 4) {
		writel(value, addr);
		return PCIBIOS_SUCCESSFUL;
	}

	if (size == 2)
		mask = ~(0xffff << ((where & 0x3) * 8));
	else if (size == 1)
		mask = ~(0xff << ((where & 0x3) * 8));
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	tmp = readl(addr) & mask;
	tmp |= value << ((where & 0x3) * 8);
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

static int tegra_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct tegra_pcie *pcie = sys->private_data;

	PR_FUNC_LINE;
	if (nr >= pcie->num_ports)
		return 0;

	pci_ioremap_io(nr * MMIO_SIZE, MMIO_BASE);

	if (request_resource(&pcie->all, &pcie->mem))
		panic("can't allocate %s\n", pcie->mem.name);
	if (request_resource(&pcie->all, &pcie->prefetch))
		panic("can't allocate %s\n", pcie->prefetch.name);

	pci_add_resource_offset(
		&sys->resources, &pcie->mem, sys->mem_offset);
	pci_add_resource_offset(
		&sys->resources, &pcie->prefetch, sys->mem_offset);
	pci_add_resource(&sys->resources, &pcie->busn);
	return 1;
}

static int tegra_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	return INT_PCIE_INTR;
}

static void tegra_pcie_add_bus(struct pci_bus *bus)
{
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		struct tegra_pcie *pcie = sys_to_pcie(bus->sysdata);

		bus->msi = &pcie->msi.chip;
	}
}

static struct pci_bus *__init tegra_pcie_scan_bus(int nr,
						  struct pci_sys_data *sys)
{
	struct tegra_pcie *pcie = sys_to_pcie(sys);
	struct pci_bus *bus;

	PR_FUNC_LINE;

	bus = pci_create_root_bus(pcie->dev, sys->busnr, &tegra_pcie_ops, sys,
				  &sys->resources);
	if (!bus)
		return NULL;

	pci_scan_child_bus(bus);

	return bus;
}

static struct hw_pci __initdata tegra_pcie_hw = {
	.nr_controllers	= 1,
	.setup		= tegra_pcie_setup,
	.scan		= tegra_pcie_scan_bus,
	.map_irq	= tegra_pcie_map_irq,
	.add_bus	= tegra_pcie_add_bus,
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

static void tegra_pcie_enable_aer(struct tegra_pcie_port *port, bool enable)
{
	unsigned int data;

	PR_FUNC_LINE;
	data = rp_readl(port, NV_PCIE2_RP_VEND_CTL1);
	if (enable)
		data |= PCIE2_RP_VEND_CTL1_ERPT;
	else
		data &= ~PCIE2_RP_VEND_CTL1_ERPT;
	rp_writel(port, data, NV_PCIE2_RP_VEND_CTL1);
}

static int tegra_pcie_attach(struct tegra_pcie *pcie)
{
	struct pci_bus *bus = NULL;
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	if (!hotplug_event)
		return 0;

	/* rescan and recreate all pcie data structures */
	while ((bus = pci_find_next_bus(bus)) != NULL)
		pci_rescan_bus(bus);
	/* unhide AER capability */
	list_for_each_entry(port, &pcie->ports, list)
		tegra_pcie_enable_aer(port, true);

	hotplug_event = false;
	return 0;
}

static int tegra_pcie_detach(struct tegra_pcie *pcie)
{
	struct pci_dev *pdev = NULL;
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	if (hotplug_event)
		return 0;
	hotplug_event = true;

	/* hide AER capability to avoid log spew */
	list_for_each_entry(port, &pcie->ports, list)
		tegra_pcie_enable_aer(port, false);

	/* remove all pcie data structures */
	for_each_pci_dev(pdev) {
		pci_stop_and_remove_bus_device(pdev);
		break;
	}
	return 0;
}

static void tegra_pcie_prsnt_map_override(struct tegra_pcie_port *port,
					bool prsnt)
{
	unsigned int data;

	PR_FUNC_LINE;
	/* currently only hotplug on root port 0 supported */
	data = rp_readl(port, NV_PCIE2_RP_PRIV_MISC);
	data &= ~PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT;
	if (prsnt)
		data |= PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_PRSNT;
	else
		data |= PCIE2_RP_PRIV_MISC_PRSNT_MAP_EP_ABSNT;
	rp_writel(port, data, NV_PCIE2_RP_PRIV_MISC);
}

static void work_hotplug_handler(struct work_struct *work)
{
	struct tegra_pcie *pcie_driver =
		container_of(work, struct tegra_pcie, hotplug_detect);
	int val;

	PR_FUNC_LINE;
	if (pcie_driver->plat_data->gpio_hot_plug == -1)
		return;
	val = gpio_get_value(pcie_driver->plat_data->gpio_hot_plug);
	if (val == 0) {
		dev_info(pcie_driver->dev, "PCIE Hotplug: Connected\n");
		tegra_pcie_attach(pcie_driver);
	} else {
		dev_info(pcie_driver->dev, "PCIE Hotplug: DisConnected\n");
		tegra_pcie_detach(pcie_driver);
	}
}

static irqreturn_t gpio_pcie_detect_isr(int irq, void *arg)
{
	struct tegra_pcie *pcie = arg;
	PR_FUNC_LINE;
	schedule_work(&pcie->hotplug_detect);
	return IRQ_HANDLED;
}
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
static void raise_emc_freq(struct tegra_pcie *pcie);
#endif

static void handle_sb_intr(struct tegra_pcie *pcie)
{
	u32 mesg;

	PR_FUNC_LINE;
	mesg = afi_readl(pcie, AFI_MSG_0);
	if (mesg & AFI_MSG_INTX_MASK)
		/* notify device isr for INTx messages from pcie devices */
		dev_info(pcie->dev,
			"Legacy INTx interrupt occurred %x\n", mesg);
	else if (mesg & AFI_MSG_PM_PME_MASK) {
		struct tegra_pcie_port *port, *tmp;
		/* handle PME messages */
		list_for_each_entry_safe(port, tmp, &pcie->ports, list)
			if (port->index == (mesg & AFI_MSG_PM_PME0))
				break;
		mesg = rp_readl(port, NV_PCIE2_RP_RSR);
		mesg |= NV_PCIE2_RP_RSR_PMESTAT;
		rp_writel(port, mesg, NV_PCIE2_RP_RSR);
	} else
		afi_writel(pcie, mesg, AFI_MSG_0);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	if (mesg & AFI_MSG_RP_INT_MASK)
		raise_emc_freq(pcie);
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
	struct tegra_pcie *pcie = arg;
	u32 code, signature;

	PR_FUNC_LINE;
	code = afi_readl(pcie, AFI_INTR_CODE) & AFI_INTR_CODE_MASK;
	signature = afi_readl(pcie, AFI_INTR_SIGNATURE);

	if (code == AFI_INTR_LEGACY)
		handle_sb_intr(pcie);
	afi_writel(pcie, 0, AFI_INTR_CODE);

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
		dev_err(pcie->dev, "PCIE: %s, signature: %08x\n",
				err_msg[code], signature);

	return IRQ_HANDLED;
}

/*
 *  PCIe support functions
 */
static void tegra_pcie_setup_translations(struct tegra_pcie *pcie)
{
	u32 fpci_bar;
	u32 size;
	u32 axi_address;

	PR_FUNC_LINE;
	/* Bar 0: type 1 extended configuration space */
	fpci_bar = 0xfe100000;
	size = PCIE_CFG_SZ;
	axi_address = PCIE_CFG_OFF;
	afi_writel(pcie, axi_address, AFI_AXI_BAR0_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR0_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR0);

	/* Bar 1: downstream IO bar */
	fpci_bar = 0xfdfc0000;
	size = MMIO_SIZE;
	axi_address = MMIO_BASE;
	afi_writel(pcie, axi_address, AFI_AXI_BAR1_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR1_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR1);

	/* Bar 2: prefetchable memory BAR */
	fpci_bar = (((PREFETCH_MEM_BASE_0 >> 12) & 0xfffff) << 4) | 0x1;
	size =  PREFETCH_MEM_SIZE_0;
	axi_address = PREFETCH_MEM_BASE_0;
	afi_writel(pcie, axi_address, AFI_AXI_BAR2_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR2_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR2);

	/* Bar 3: non prefetchable memory BAR */
	fpci_bar = (((MEM_BASE_0 >> 12) & 0xfffff) << 4) | 0x1;
	size = MEM_SIZE_0;
	axi_address = MEM_BASE_0;
	afi_writel(pcie, axi_address, AFI_AXI_BAR3_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR3_SZ);
	afi_writel(pcie, fpci_bar, AFI_FPCI_BAR3);

	/* NULL out the remaining BAR as it is not used */
	afi_writel(pcie, 0, AFI_AXI_BAR4_START);
	afi_writel(pcie, 0, AFI_AXI_BAR4_SZ);
	afi_writel(pcie, 0, AFI_FPCI_BAR4);

	afi_writel(pcie, 0, AFI_AXI_BAR5_START);
	afi_writel(pcie, 0, AFI_AXI_BAR5_SZ);
	afi_writel(pcie, 0, AFI_FPCI_BAR5);

	/* map all upstream transactions as uncached */
	afi_writel(pcie, PHYS_OFFSET, AFI_CACHE_BAR0_ST);
	afi_writel(pcie, 0, AFI_CACHE_BAR0_SZ);
	afi_writel(pcie, 0, AFI_CACHE_BAR1_ST);
	afi_writel(pcie, 0, AFI_CACHE_BAR1_SZ);

	/* No MSI */
	afi_writel(pcie, 0, AFI_MSI_FPCI_BAR_ST);
	afi_writel(pcie, 0, AFI_MSI_BAR_SZ);
	afi_writel(pcie, 0, AFI_MSI_AXI_BAR_ST);
	afi_writel(pcie, 0, AFI_MSI_BAR_SZ);
}

static int tegra_pcie_enable_pads(struct tegra_pcie *pcie, bool enable)
{
	int err = 0;

	PR_FUNC_LINE;
	if (!tegra_platform_is_fpga()) {
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		if (!enable)
			tegra_periph_reset_assert(pcie->pex_uphy);
#endif
		/* WAR for Eye diagram failure */
		pads_writel(pcie, REFCLK_POR_SETTINGS, PADS_REFCLK_CFG0);
		pads_writel(pcie, 0x00000028, PADS_REFCLK_BIAS);
		/* PCIe pad programming is moved to XUSB_PADCTL space */
		err = pcie_phy_pad_enable(enable,
				tegra_get_lane_owner_info() >> 1);
		if (err)
			dev_err(pcie->dev,
				"%s unable to initalize pads\n", __func__);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		if (enable)
			tegra_periph_reset_deassert(pcie->pex_uphy);
#endif
	}
	return err;
}

static int tegra_pcie_enable_controller(struct tegra_pcie *pcie)
{
	u32 val;
	int ret = 0;

	PR_FUNC_LINE;
	/* Enable PLL power down */
	val = afi_readl(pcie, AFI_PLLE_CONTROL);
	val &= ~AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL;
	val |= AFI_PLLE_CONTROL_PADS2PLLE_CONTROL_EN;
	afi_writel(pcie, val, AFI_PLLE_CONTROL);

	afi_writel(pcie, 0, AFI_PEXBIAS_CTRL_0);

	/* Enable all PCIE controller and */
	/* system management configuration of PCIE crossbar */
	val = afi_readl(pcie, AFI_PCIE_CONFIG);
	val &= ~(AFI_PCIE_CONFIG_PCIEC0_DISABLE_DEVICE |
		 AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_MASK);
	if (tegra_platform_is_fpga()) {
		/* FPGA supports only x2_x1 bar config */
		val |= AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X2_X1;
	} else {
		struct tegra_pcie_port *port;
		int lane_owner = 0;
		lane_owner = tegra_get_lane_owner_info() >> 1;
		list_for_each_entry(port, &pcie->ports, list) {
			if (lane_owner == PCIE_LANES_X2_X1) {
				val |=
				 AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X2_X1;
				if (port->status && port->index == 1)
					val &=
					~AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE;
			} else {
				val |=
				 AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_X4_X1;
				if ((port->status && port->index == 1)
					&& (lane_owner == PCIE_LANES_X4_X1))
					val &=
					~AFI_PCIE_CONFIG_PCIEC1_DISABLE_DEVICE;
			}
		}
	}
	afi_writel(pcie, val, AFI_PCIE_CONFIG);

	/* Enable Gen 2 capability of PCIE */
	val = afi_readl(pcie, AFI_FUSE) & ~AFI_FUSE_PCIE_T0_GEN2_DIS;
	afi_writel(pcie, val, AFI_FUSE);

	/* Finally enable PCIe */
	val = afi_readl(pcie, AFI_CONFIGURATION);
	val |=  AFI_CONFIGURATION_EN_FPCI;
	afi_writel(pcie, val, AFI_CONFIGURATION);

	val = (AFI_INTR_EN_INI_SLVERR | AFI_INTR_EN_INI_DECERR |
	       AFI_INTR_EN_TGT_SLVERR | AFI_INTR_EN_TGT_DECERR |
	       AFI_INTR_EN_TGT_WRERR | AFI_INTR_EN_DFPCI_DECERR |
	       AFI_INTR_EN_AXI_DECERR | AFI_INTR_EN_PRSNT_SENSE);
	afi_writel(pcie, val, AFI_AFI_INTR_ENABLE);
	afi_writel(pcie, 0xffffffff, AFI_SM_INTR_ENABLE);

	/* FIXME: No MSI for now, only INT */
	afi_writel(pcie, AFI_INTR_MASK_INT_MASK, AFI_INTR_MASK);

	/* Disable all execptions */
	afi_writel(pcie, 0, AFI_FPCI_ERROR_MASKS);

	/* Wait for clock to latch (min of 100us) */
	udelay(100);
	tegra_periph_reset_deassert(pcie->pcie_xclk);

	return ret;
}

static int tegra_pcie_enable_regulators(struct tegra_pcie *pcie)
{
	int i;
	PR_FUNC_LINE;
	if (pcie->power_rails_enabled) {
		dev_info(pcie->dev, "PCIE: Already power rails enabled\n");
		return 0;
	}
	pcie->power_rails_enabled = 1;
	dev_info(pcie->dev, "PCIE: Enable power rails\n");

	for (i = 0; i < pcie->soc_data->num_pcie_regulators; i++) {
		if (pcie->pcie_regulators[i])
			if (regulator_enable(pcie->pcie_regulators[i]))
				dev_err(pcie->dev, "%s: can't enable regulator %s\n",
				__func__,
				pcie->soc_data->pcie_regulator_names[i]);
	}

	return 0;

}

static int tegra_pcie_disable_regulators(struct tegra_pcie *pcie)
{
	int i;
	PR_FUNC_LINE;
	if (pcie->power_rails_enabled == 0) {
		dev_info(pcie->dev, "PCIE: Already power rails disabled\n");
		return 0;
	}
	dev_info(pcie->dev, "PCIE: Disable power rails\n");

	for (i = 0; i < pcie->soc_data->num_pcie_regulators; i++) {
		if (pcie->pcie_regulators[i] != NULL)
			if (regulator_disable(pcie->pcie_regulators[i]))
				dev_err(pcie->dev, "%s: can't disable regulator %s\n",
				__func__,
				pcie->soc_data->pcie_regulator_names[i]);
	}

	pcie->power_rails_enabled = 0;
	return 0;

}

static int tegra_pcie_power_ungate(struct tegra_pcie *pcie)
{
	int err;

	PR_FUNC_LINE;
	err = tegra_unpowergate_partition_with_clk_on(TEGRA_POWERGATE_PCIE);
	if (err) {
		dev_err(pcie->dev, "PCIE: powerup sequence failed: %d\n", err);
		return err;
	}

	tegra_periph_reset_assert(pcie->pcie_xclk);
	err = clk_prepare_enable(pcie->pcie_mselect);
	if (err) {
		dev_err(pcie->dev,
			"PCIE: mselect clk enable failed: %d\n", err);
		return err;
	}
	err = clk_enable(pcie->pcie_xclk);
	if (err) {
		dev_err(pcie->dev, "PCIE: pciex clk enable failed: %d\n", err);
		return err;
	}

	return 0;
}

static int tegra_pcie_map_resources(struct tegra_pcie *pcie)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);
	struct resource *pads, *afi;

	PR_FUNC_LINE;
	pads = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pads");
	pcie->pads = devm_ioremap_nocache(&pdev->dev,
						pads->start,
						resource_size(pads));
	if (IS_ERR(pcie->pads)) {
		dev_err(pcie->dev, "PCIE: Failed to map PAD registers\n");
		return PTR_ERR(pcie->pads);
	}

	afi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "afi");
	pcie->afi = devm_ioremap_nocache(&pdev->dev,
						afi->start,
						resource_size(afi));
	if (IS_ERR(pcie->afi)) {
		dev_err(pcie->dev, "PCIE: Failed to map AFI registers\n");
		return PTR_ERR(pcie->afi);
	}
	return 0;
}

void tegra_pcie_unmap_resources(struct tegra_pcie *pcie)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);

	PR_FUNC_LINE;

	if (pcie->pads) {
		devm_iounmap(&pdev->dev, pcie->pads);
		pcie->pads = 0;
	}
	if (pcie->afi) {
		devm_iounmap(&pdev->dev, pcie->afi);
		pcie->afi = 0;
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

static int tegra_pcie_fpga_phy_init(struct tegra_pcie *pcie)
{
#define FPGA_GEN2_SPEED_SUPPORT		0x90000001
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	if (!tegra_pcie_is_fpga_pcie())
		return -ENODEV;

	/* Do reset for FPGA pcie phy */
	afi_writel(pcie, AFI_WR_SCRATCH_0_RESET_VAL, AFI_WR_SCRATCH_0);
	udelay(10);
	afi_writel(pcie, AFI_WR_SCRATCH_0_DEFAULT_VAL, AFI_WR_SCRATCH_0);
	udelay(10);
	afi_writel(pcie, AFI_WR_SCRATCH_0_RESET_VAL, AFI_WR_SCRATCH_0);

	/* required for gen2 speed support on FPGA */
	list_for_each_entry(port, &pcie->ports, list)
		rp_writel(port,
			FPGA_GEN2_SPEED_SUPPORT, NV_PCIE2_RP_VEND_XP_BIST);

	return 0;
}

static void tegra_pcie_pme_turnoff(struct tegra_pcie *pcie)
{
	unsigned int data;

	PR_FUNC_LINE;
	if (tegra_platform_is_fpga() && !tegra_pcie_is_fpga_pcie())
		return;
	data = afi_readl(pcie, AFI_PCIE_PME);
	data |= AFI_PCIE_PME_TURN_OFF;
	afi_writel(pcie, data, AFI_PCIE_PME);
	do {
		data = afi_readl(pcie, AFI_PCIE_PME);
	} while (!(data & AFI_PCIE_PME_ACK));

	/* Required for PLL power down */
	data = afi_readl(pcie, AFI_PLLE_CONTROL);
	data |= AFI_PLLE_CONTROL_BYPASS_PADS2PLLE_CONTROL;
	afi_writel(pcie, data, AFI_PLLE_CONTROL);
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
static int tegra_pcie_power_on(struct tegra_pcie *pcie)
{
	int err = 0;

	PR_FUNC_LINE;
	if (pcie->pcie_power_enabled) {
		dev_info(pcie->dev, "PCIE: Already powered on");
		goto err_exit;
	}
	pcie->pcie_power_enabled = 1;
	pm_runtime_get_sync(pcie->dev);

	if (!tegra_platform_is_fpga()) {
		/* disable PEX IOs DPD mode to turn on pcie */
		tegra_io_dpd_disable(&pexbias_io);
		tegra_io_dpd_disable(&pexclk1_io);
		tegra_io_dpd_disable(&pexclk2_io);
		err = tegra_pcie_enable_regulators(pcie);
		if (err) {
			dev_err(pcie->dev, "PCIE: Failed to enable regulators\n");
			goto err_exit;
		}
	}
	err = tegra_pcie_power_ungate(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: Failed to power ungate\n");
		goto err_exit;
	}
	err = tegra_pcie_map_resources(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: Failed to map resources\n");
		goto err_exit;
	}
	if (tegra_platform_is_fpga()) {
		err = tegra_pcie_fpga_phy_init(pcie);
		if (err)
			dev_err(pcie->dev, "PCIE: Failed to initialize FPGA Phy\n");
	}

err_exit:
	if (err)
		pm_runtime_put(pcie->dev);
	return err;
}

static int tegra_pcie_power_off(struct tegra_pcie *pcie, bool all)
{
	int err = 0;
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	if (pcie->pcie_power_enabled == 0) {
		dev_info(pcie->dev, "PCIE: Already powered off");
		goto err_exit;
	}
	if (all) {
		list_for_each_entry(port, &pcie->ports, list) {
			tegra_pcie_prsnt_map_override(port, false);
		}
		tegra_pcie_pme_turnoff(pcie);
		tegra_pcie_enable_pads(pcie, false);
	}
	tegra_pcie_unmap_resources(pcie);
	if (pcie->pcie_mselect)
		clk_disable(pcie->pcie_mselect);
	if (pcie->pcie_xclk)
		clk_disable(pcie->pcie_xclk);
	err = tegra_powergate_partition_with_clk_off(TEGRA_POWERGATE_PCIE);
	if (err)
		goto err_exit;

	if (!tegra_platform_is_fpga()) {
		err = tegra_pcie_disable_regulators(pcie);
		if (err)
			goto err_exit;
		/* put PEX pads into DPD mode to save additional power */
		tegra_io_dpd_enable(&pexbias_io);
		tegra_io_dpd_enable(&pexclk1_io);
		tegra_io_dpd_enable(&pexclk2_io);
	}
	pm_runtime_put(pcie->dev);

	pcie->pcie_power_enabled = 0;
err_exit:
	return err;
}

static int tegra_pcie_clocks_get(struct tegra_pcie *pcie)
{
	PR_FUNC_LINE;
	/* get the PCIEXCLK */
	pcie->pcie_xclk = clk_get_sys("tegra_pcie", "pciex");
	if (IS_ERR_OR_NULL(pcie->pcie_xclk)) {
		dev_err(pcie->dev, "%s: unable to get PCIE Xclock\n", __func__);
		return -EINVAL;
	}
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	pcie->pex_uphy = clk_get_sys("tegra_pcie", "pex_uphy");
	if (IS_ERR_OR_NULL(pcie->pex_uphy)) {
		dev_err(pcie->dev,
			"%s: unable to get PCIE pex_uphy clock\n", __func__);
		return -EINVAL;
	}
#endif
	pcie->pcie_mselect = clk_get_sys("tegra_pcie", "mselect");
	if (IS_ERR_OR_NULL(pcie->pcie_mselect)) {
		dev_err(pcie->dev,
			"%s: unable to get PCIE mselect clock\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void tegra_pcie_clocks_put(struct tegra_pcie *pcie)
{
	PR_FUNC_LINE;
	if (pcie->pcie_xclk)
		clk_put(pcie->pcie_xclk);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	if (pcie->pex_uphy)
		clk_put(pcie->pex_uphy);
#endif
	if (pcie->pcie_mselect)
		clk_put(pcie->pcie_mselect);
}

static int tegra_pcie_get_resources(struct tegra_pcie *pcie)
{
	int err;
	struct platform_device *pdev = to_platform_device(pcie->dev);
	struct resource *pads, *afi, *res;

	PR_FUNC_LINE;
	pcie->power_rails_enabled = 0;
	pcie->pcie_power_enabled = 0;

	err = tegra_pcie_clocks_get(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: failed to get clocks: %d\n", err);
		goto err_clk_get;
	}

	err = tegra_pcie_power_on(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: Failed to power on: %d\n", err);
		goto err_pwr_on;
	}

	err = clk_set_rate(pcie->pcie_mselect, tegra_pcie_mselect_rate);
	if (err)
		return err;

	err = clk_set_rate(pcie->pcie_xclk, tegra_pcie_xclk_rate);
	if (err)
		return err;

	pads = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pads");
	if (!pads) {
		dev_err(pcie->dev, "PCIE: Failed to get pad registers\n");
		goto err_pwr_on;
	}
	if (!devm_request_mem_region(&pdev->dev, pads->start,
			resource_size(pads),
			pads->name ?: dev_name(&pdev->dev))) {
		dev_err(pcie->dev,
			"PCIE: Failed to request region for pad registers\n");
		goto err_pwr_on;
	}

	afi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "afi");
	if (!afi) {
		dev_err(pcie->dev, "PCIE: Failed to get afi registers\n");
		goto err_pwr_on;
	}
	if (!devm_request_mem_region(&pdev->dev, afi->start, resource_size(afi),
			afi->name ?: dev_name(&pdev->dev))) {
		dev_err(pcie->dev, "PCIE: Failed to request region for afi registers\n");
		goto err_pwr_on;
	}

	/* request configuration space, but remap later, on demand */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cs");
	if (!res) {
		dev_err(pcie->dev, "PCIE: Failed to get CS registers\n");
		goto err_pwr_on;
	}
	pcie->cs = devm_request_mem_region(pcie->dev, res->start,
					   resource_size(res), res->name);
	if (!pcie->cs) {
		dev_err(pcie->dev, "PCIE: Failed to request region for CS registers\n");
		goto err_pwr_on;
	}

	err = devm_request_irq(pcie->dev, INT_PCIE_INTR, tegra_pcie_isr,
			IRQF_SHARED, "PCIE", pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: Failed to register IRQ: %d\n", err);
		goto err_pwr_on;
	}
	set_irq_flags(INT_PCIE_INTR, IRQF_VALID);

	return 0;

err_pwr_on:
	tegra_pcie_power_off(pcie, false);
err_clk_get:
	tegra_pcie_clocks_put(pcie);
	return err;
}

static unsigned long tegra_pcie_port_get_pex_ctrl(struct tegra_pcie_port *port)
{
	unsigned long ret = 0;

	switch (port->index) {
	case 0:
		ret = AFI_PEX0_CTRL;
		break;

	case 1:
		ret = AFI_PEX1_CTRL;
		break;
	}

	return ret;
}

static void tegra_pcie_port_reset(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	PR_FUNC_LINE;

	/* pulse reset signal */
	value = afi_readl(port->pcie, ctrl);
	value &= ~AFI_PEX_CTRL_RST;
	afi_writel(port->pcie, value, ctrl);

	usleep_range(1000, 2000);

	value = afi_readl(port->pcie, ctrl);
	value |= AFI_PEX_CTRL_RST;
	afi_writel(port->pcie, value, ctrl);
}

static void tegra_pcie_port_enable(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	PR_FUNC_LINE;

	/* enable reference clock */
	value = afi_readl(port->pcie, ctrl);
	value |= AFI_PEX_CTRL_REFCLK_EN | AFI_PEX_CTRL_CLKREQ_EN;

	/* Since CLKREQ# pinmux pins may float in some platfoms */
	/* resulting in disappear of refclk specially at higher temp */
	/* overrided CLKREQ to always drive refclk */
	if (!port->has_clkreq)
		value |= AFI_PEX_CTRL_OVERRIDE_EN;

	afi_writel(port->pcie, value, ctrl);

	tegra_pcie_port_reset(port);
}

static void tegra_pcie_port_disable(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	PR_FUNC_LINE;

	/* assert port reset */
	value = afi_readl(port->pcie, ctrl);
	value &= ~AFI_PEX_CTRL_RST;
	afi_writel(port->pcie, value, ctrl);

	/* disable reference clock */
	value = afi_readl(port->pcie, ctrl);
	value &= ~AFI_PEX_CTRL_CLKREQ_EN;
	value &= ~AFI_PEX_CTRL_REFCLK_EN;
	afi_writel(port->pcie, value, ctrl);
}

static void tegra_pcie_port_free(struct tegra_pcie_port *port)
{
	struct tegra_pcie *pcie = port->pcie;

	PR_FUNC_LINE;

	devm_iounmap(pcie->dev, port->base);
	devm_release_mem_region(pcie->dev, port->regs.start,
				resource_size(&port->regs));
	list_del(&port->list);
	devm_kfree(pcie->dev, port);
}

/*
 * FIXME: If there are no PCIe cards attached, then calling this function
 * can result in the increase of the bootup time as there are big timeout
 * loops.
 */
#define TEGRA_PCIE_LINKUP_TIMEOUT	200	/* up to 1.2 seconds */
static bool tegra_pcie_port_check_link(struct tegra_pcie_port *port)
{
	unsigned int retries = 3;
	unsigned long value;

	PR_FUNC_LINE;

	/* override presence detection */
	tegra_pcie_prsnt_map_override(port, true);
	do {
		unsigned int timeout = TEGRA_PCIE_LINKUP_TIMEOUT;

		do {
			value = readl(port->base + RP_VEND_XP);

			if (value & RP_VEND_XP_DL_UP)
				break;

			usleep_range(1000, 2000);
		} while (--timeout);

		if (!timeout) {
			dev_err(port->pcie->dev, "link %u down, retrying\n",
				port->index);
			goto retry;
		}

		timeout = TEGRA_PCIE_LINKUP_TIMEOUT;

		do {
			value = readl(port->base + RP_LINK_CONTROL_STATUS);

			if (value & RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE)
				return true;

			usleep_range(1000, 2000);
		} while (--timeout);

retry:
		tegra_pcie_port_reset(port);
	} while (--retries);

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

static void raise_emc_freq(struct tegra_pcie *pcie)
{
	PR_FUNC_LINE;

	/* raise emc freq to 508MHz to reach expected gen2 */
	/* bandwidth if all have gen2 enabled, bug#1452749 */
	if (t210_war && is_all_gen2()) {
		struct clk *emc_clk;
		emc_clk = clk_get_sys("tegra_pcie", "emc");
		if (IS_ERR_OR_NULL(emc_clk)) {
			dev_err(pcie->dev, "unable to get emc clk\n");
			goto fail;
		}
		if (clk_enable(emc_clk)) {
			dev_err(pcie->dev, "emc clk enable failed\n");
			goto fail;
		}
		clk_set_rate(emc_clk, 508000000);
	}
fail:
	return;
}
#endif
static void tegra_pcie_apply_sw_war(struct tegra_pcie_port *port,
				bool enum_done)
{
	unsigned int data;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	struct tegra_pcie *pcie = port->pcie;
	int lane_owner;
#endif
	struct pci_dev *pdev = NULL;

	PR_FUNC_LINE;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	/* T210 WAR for perf bugs required when LPDDR4 */
	/* memory is used with both ctlrs in X4_X1 config */
	lane_owner = tegra_get_lane_owner_info() >> 1;
	if (pcie->plat_data->has_memtype_lpddr4 &&
		(lane_owner == PCIE_LANES_X4_X1) &&
		(pcie->num_ports == pcie->soc_data->num_ports))
		t210_war = 1;
#endif
	if (enum_done) {
		/* disable msi for port driver to avoid panic */
		for_each_pci_dev(pdev)
			if (pci_pcie_type(pdev) == PCI_EXP_TYPE_ROOT_PORT)
				pdev->msi_enabled = 0;
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		raise_emc_freq(pcie);
#endif
	} else {
		/* WAR for Eye diagram failure on lanes for T124 platforms */
		data = rp_readl(port, NV_PCIE2_RP_ECTL_1_R2);
		data |= PCIE2_RP_ECTL_1_R2_TX_CMADJ_1C;
		data |= PCIE2_RP_ECTL_1_R2_TX_DRV_CNTL_1C;
		rp_writel(port, data, NV_PCIE2_RP_ECTL_1_R2);
		/* Avoid warning during enumeration for invalid IRQ of RP */
		data = rp_readl(port, NV_PCIE2_RP_INTR_BCR);
		data |= NV_PCIE2_RP_INTR_BCR_INTR_LINE;
		rp_writel(port, data, NV_PCIE2_RP_INTR_BCR);
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
		/* resize buffers for better perf, bug#1447522 */
		if (t210_war) {
			struct tegra_pcie_port *temp_port;
			list_for_each_entry(temp_port, &pcie->ports, list) {
				data = rp_readl(temp_port,
							NV_PCIE2_RP_XP_CTL_1);
				data |= PCIE2_RP_XP_CTL_1_SPARE_BIT29;
				rp_writel(temp_port, data,
					NV_PCIE2_RP_XP_CTL_1);

				data = rp_readl(temp_port,
						NV_PCIE2_RP_TX_HDR_LIMIT);
				if (temp_port->index)
					data |= PCIE2_RP_TX_HDR_LIMIT_NPT_1;
				else
					data |= PCIE2_RP_TX_HDR_LIMIT_NPT_0;
				rp_writel(temp_port, data,
					NV_PCIE2_RP_TX_HDR_LIMIT);
			}
		}
		/* Bug#1461732 WAR, set clkreq asserted delay greater than */
		/* power off time (2us) to avoid RP wakeup in L1.2_ENTRY */
		data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_1_CYA_CLKREQ_ASSERTED_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_1_CYA_CLKREQ_ASSERTED_DLY;
		rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);

		/* take care of link speed change error in corner cases */
		data = rp_readl(port, NV_PCIE2_RP_VEND_CTL0);
		data &= ~PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH_MASK;
		data |= PCIE2_RP_VEND_CTL0_DSK_RST_PULSE_WIDTH;
		rp_writel(port, data, NV_PCIE2_RP_VEND_CTL0);

		/* Avoid below register programming for >13 MHz clk_25m Freq */
		if (clk_get_rate(clk_get_sys(NULL, "clk_m")) > 13000000)
			return;
		data = rp_readl(port, NV_PCIE2_RP_TIMEOUT0);
		data &= ~PCIE2_RP_TIMEOUT0_PAD_PWRUP_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_PWRUP;
		data &= ~PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_PWRUP_CM;
		data &= ~PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2_MASK;
		data |= PCIE2_RP_TIMEOUT0_PAD_SPDCHNG_GEN2;
		rp_writel(port, data, NV_PCIE2_RP_TIMEOUT0);

		data = rp_readl(port, NV_PCIE2_RP_TIMEOUT1);
		data &= ~PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE_MASK;
		data |= PCIE2_RP_TIMEOUT1_RCVRY_SPD_SUCCESS_EIDLE;
		data &= ~PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE_MASK;
		data |= PCIE2_RP_TIMEOUT1_RCVRY_SPD_UNSUCCESS_EIDLE;
		rp_writel(port, data, NV_PCIE2_RP_TIMEOUT1);

		data = rp_readl(port, NV_PCIE2_RP_XP_REF);
		data &= ~PCIE2_RP_XP_REF_MICROSECOND_LIMIT_MASK;
		data |= PCIE2_RP_XP_REF_MICROSECOND_LIMIT;
		data |= PCIE2_RP_XP_REF_MICROSECOND_ENABLE;
		data |= PCIE2_RP_XP_REF_CPL_TO_OVERRIDE;
		data &= ~PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE_MASK;
		data |= PCIE2_RP_XP_REF_CPL_TO_CUSTOM_VALUE;
		rp_writel(port, data, NV_PCIE2_RP_XP_REF);

		data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_1_CYA_PWR_OFF_DLY;
		rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_1_CYA);

		data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA);
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_T_L1_2_DLY;
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND;
		data &= ~PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP_MASK;
		data |= PCIE2_RP_L1_PM_SUBSTATES_2_CYA_MICROSECOND_COMP;
		rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_2_CYA);
#endif
	}
}

/* Enable various features of root port */
static void tegra_pcie_enable_rp_features(struct tegra_pcie_port *port)
{
	unsigned int data;

	PR_FUNC_LINE;
	/* Power mangagement settings */
	/* Enable clock clamping by default and enable card detect */
	data = rp_readl(port, NV_PCIE2_RP_PRIV_MISC);
	data |= PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_THRESHOLD |
		PCIE2_RP_PRIV_MISC_CTLR_CLK_CLAMP_ENABLE |
		PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_THRESHOLD |
		PCIE2_RP_PRIV_MISC_TMS_CLK_CLAMP_ENABLE;
	rp_writel(port, data, NV_PCIE2_RP_PRIV_MISC);

	/* Enable ASPM - L1 state support by default */
	data = rp_readl(port, NV_PCIE2_RP_VEND_XP1);
	data |= NV_PCIE2_RP_VEND_XP_LINK_PVT_CTL_L1_ASPM_SUPPORT;
	rp_writel(port, data, NV_PCIE2_RP_VEND_XP1);

	/* LTSSM wait for DLLP to finish before entering L1 or L2/L3 */
	/* to avoid truncating of PM mesgs resulting in reciever errors */
	data = rp_readl(port, NV_PCIE2_RP_VEND_XP_BIST);
	data |= PCIE2_RP_VEND_XP_BIST_GOTO_L1_L2_AFTER_DLLP_DONE;
	rp_writel(port, data, NV_PCIE2_RP_VEND_XP_BIST);

	/* unhide AER capability */
	tegra_pcie_enable_aer(port, true);

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	/* program timers for L1 substate support */
	/* set cm_rtime = 100us and t_pwr_on = 70us as per HW team */
	data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);
	data &= ~PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_MASK;
	data |= (0x64 << PCIE2_RP_L1_PM_SUBSTATES_CYA_CM_RTIME_SHIFT);
	rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);

	data = rp_readl(port, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);
	data &= ~(PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_MASK |
		PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_MASK);
	data |= (1 << PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_SCL_SHIFT) |
		(7 << PCIE2_RP_L1_PM_SUBSTATES_CYA_T_PWRN_VAL_SHIFT);
	rp_writel(port, data, NV_PCIE2_RP_L1_PM_SUBSTATES_CYA);
#endif
	tegra_pcie_apply_sw_war(port, false);
}

void tegra_pcie_check_ports(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port, *tmp;

	PR_FUNC_LINE;
	pcie->num_ports = 0;

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		dev_info(pcie->dev, "probing port %u, using %u lanes\n",
			 port->index, port->lanes);

		tegra_pcie_port_enable(port);

		if (tegra_pcie_port_check_link(port)) {
			pcie->num_ports++;
			tegra_pcie_enable_rp_features(port);
			continue;
		}

		dev_info(pcie->dev, "link %u down, ignoring\n", port->index);

		tegra_pcie_port_disable(port);
		tegra_pcie_port_free(port);
	}
}
EXPORT_SYMBOL(tegra_pcie_check_ports);

int tegra_pcie_get_test_info(void __iomem **regs)
{
#if 0 /* FIX THIS */
	*regs = tegra_pcie.regs;
	return tegra_pcie.num_ports;
#else
	return 0;
#endif
}
EXPORT_SYMBOL(tegra_pcie_get_test_info);

static int tegra_pcie_conf_gpios(struct tegra_pcie *pcie)
{
	int irq, err = 0;

	PR_FUNC_LINE;
	if (gpio_is_valid(pcie->plat_data->gpio_hot_plug)) {
		/* configure gpio for hotplug detection */
		dev_info(pcie->dev, "acquiring hotplug_detect = %d\n",
				pcie->plat_data->gpio_hot_plug);
		err = devm_gpio_request(pcie->dev,
				pcie->plat_data->gpio_hot_plug,
				"pcie_hotplug_detect");
		if (err < 0) {
			dev_err(pcie->dev, "%s: gpio_request failed %d\n",
					__func__, err);
			return err;
		}
		err = gpio_direction_input(
				pcie->plat_data->gpio_hot_plug);
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: gpio_direction_input failed %d\n",
				__func__, err);
			return err;
		}
		irq = gpio_to_irq(pcie->plat_data->gpio_hot_plug);
		if (irq < 0) {
			dev_err(pcie->dev,
				"Unable to get irq for hotplug_detect\n");
			return err;
		}
		err = devm_request_irq(pcie->dev, (unsigned int)irq,
				gpio_pcie_detect_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"pcie_hotplug_detect",
				(void *)pcie);
		if (err < 0) {
			dev_err(pcie->dev,
				"Unable to claim irq for hotplug_detect\n");
			return err;
		}
	}
	if (gpio_is_valid(pcie->plat_data->gpio_x1_slot)) {
		err = devm_gpio_request(pcie->dev,
			pcie->plat_data->gpio_x1_slot, "pcie_x1_slot");
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_x1_slot gpio_request failed %d\n",
				__func__, err);
			return err;
		}
		err = gpio_direction_output(
			pcie->plat_data->gpio_x1_slot, 1);
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_x1_slot gpio_direction_output failed %d\n",
					__func__, err);
			return err;
		}
		gpio_set_value_cansleep(
			pcie->plat_data->gpio_x1_slot, 1);
	}
	if (gpio_is_valid(pcie->plat_data->gpio_wake)) {
		err = devm_gpio_request(pcie->dev,
				pcie->plat_data->gpio_wake, "pcie_wake");
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_wake gpio_request failed %d\n",
				__func__, err);
			return err;
		}
		err = gpio_direction_input(
				pcie->plat_data->gpio_wake);
		if (err < 0) {
			dev_err(pcie->dev,
				"%s: pcie_wake gpio_direction_input failed %d\n",
					__func__, err);
			return err;
		}
	}
	return 0;
}

static int tegra_pcie_scale_voltage(struct tegra_pcie *pcie, bool isGen2)
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
	err = clk_set_rate(pcie->pcie_xclk, tegra_pcie_xclk_rate);
	if (err)
		return err;
	err = clk_set_rate(pcie->pcie_mselect, tegra_pcie_mselect_rate);
skip:
	return err;

}

static bool tegra_pcie_change_link_speed(struct tegra_pcie *pcie,
				struct pci_dev *pdev, bool isGen2)
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
	if (tegra_pcie_scale_voltage(pcie, isGen2))
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

bool tegra_pcie_link_speed(struct tegra_pcie *pcie, bool isGen2)
{
	struct pci_dev *pdev = NULL;
	bool ret = false;

	PR_FUNC_LINE;
	/* Voltage scaling should happen before any device transition */
	/* to Gen2 or after all devices has transitioned to Gen1 */
	for_each_pci_dev(pdev) {
		if (tegra_pcie_change_link_speed(pcie, pdev, isGen2))
			ret = true;
	}
	return ret;
}
EXPORT_SYMBOL(tegra_pcie_link_speed);

/* support PLL power down in L1 dynamically based on platform */
static void tegra_pcie_pll_pdn(struct tegra_pcie *pcie)
{
	static struct pinctrl_dev *pctl_dev = NULL;
	struct pci_dev *pdev = NULL;

	if (!pctl_dev)
		pctl_dev = pinctrl_get_dev_from_of_compatible(
					pinctrl_compatible);
	if (!pctl_dev) {
		dev_err(pcie->dev,
			"%s(): tegra pincontrol does not found\n", __func__);
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

static void tegra_pcie_config_clkreq(struct tegra_pcie *pcie, bool enable)
{
	static struct pinctrl_dev *pctl_dev = NULL;
	unsigned long od_conf, tr_conf;

	PR_FUNC_LINE;

	if (!pctl_dev)
		pctl_dev = pinctrl_get_dev_from_of_compatible(
				pinctrl_compatible);
	if (!pctl_dev) {
		dev_err(pcie->dev,
			"%s(): tegra pincontrol does not found\n", __func__);
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
static void tegra_pcie_enable_aspm(struct tegra_pcie *pcie)
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
		dev_info(pcie->dev, "PCIE: ASPM not enabled\n");
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
		tegra_pcie_config_clkreq(pcie, true);
		tegra_pcie_config_l1ss_tpwr_on();
		tegra_pcie_config_l1ss_cm_rtime();
		tegra_pcie_config_l1ss_l12_thtime();
		tegra_pcie_enable_l1ss_support();
		tegra_pcie_enable_ltr_support();
	}
#endif
}

static void tegra_pcie_enable_features(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port;

	PR_FUNC_LINE;
	/* configure all links to gen2 speed by default */
	if (!tegra_pcie_link_speed(pcie, true))
		dev_info(pcie->dev, "PCIE: No Link speed change happened\n");

	tegra_pcie_pll_pdn(pcie);
	tegra_pcie_enable_aspm(pcie);
	list_for_each_entry(port, &pcie->ports, list) {
		tegra_pcie_apply_sw_war(port, true);
	}
}
static bool tegra_pcie_enable_msi(struct tegra_pcie *, bool);
static int tegra_pcie_disable_msi(struct tegra_pcie *pcie);

static int __init tegra_pcie_init(struct tegra_pcie *pcie)
{
	int err = 0;
	struct platform_device *pdev = to_platform_device(pcie->dev);

	pcibios_min_io = 0x1000ul;

	PR_FUNC_LINE;

	INIT_WORK(&pcie->hotplug_detect, work_hotplug_handler);
	err = tegra_pcie_get_resources(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: get resources failed\n");
		return err;
	}
	err = tegra_pcie_enable_pads(pcie, true);
	if (err) {
		dev_err(pcie->dev, "PCIE: enable pads failed\n");
		tegra_pcie_power_off(pcie, false);
		return err;
	}
	err = tegra_pcie_enable_controller(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: enable controller failed\n");
		goto fail;
	}
	err = tegra_pcie_conf_gpios(pcie);
	if (err) {
		dev_err(pcie->dev, "PCIE: configuring gpios failed\n");
		goto fail;
	}
	/* setup the AFI address translations */
	tegra_pcie_setup_translations(pcie);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		err = tegra_pcie_enable_msi(pcie, false);
		if (err < 0) {
			dev_err(&pdev->dev,
				"failed to enable MSI support: %d\n",
				err);
			goto fail;
		}
	}

	tegra_pcie_check_ports(pcie);

	if (pcie->num_ports) {
		tegra_pcie_hw.private_data = (void **)&pcie;
		pci_common_init(&tegra_pcie_hw);
	} else {
		dev_err(pcie->dev, "PCIE: no ports detected\n");
		goto disable_msi;
	}
	tegra_pcie_enable_features(pcie);
	/* register pcie device as wakeup source */
	device_init_wakeup(pcie->dev, true);

	return 0;

disable_msi:
	if (IS_ENABLED(CONFIG_PCI_MSI))
		tegra_pcie_disable_msi(pcie);
fail:
	tegra_pcie_power_off(pcie, true);
	return err;
}

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

void msi_map_release(struct msi_map_entry *entry)
{
	if (entry) {
		entry->used = false;
		entry->irq = 0;
	}
}

static int tegra_msi_alloc(struct tegra_msi *chip)
{
	int msi;

	PR_FUNC_LINE;

	mutex_lock(&chip->lock);

	msi = find_first_zero_bit(chip->used, INT_PCI_MSI_NR);
	if (msi < INT_PCI_MSI_NR)
		set_bit(msi, chip->used);
	else
		msi = -ENOSPC;

	mutex_unlock(&chip->lock);

	return msi;
}

static void tegra_msi_free(struct tegra_msi *chip, unsigned long irq)
{
	struct device *dev = chip->chip.dev;

	PR_FUNC_LINE;

	mutex_lock(&chip->lock);

	if (!test_bit(irq, chip->used))
		dev_err(dev, "trying to free unused MSI#%lu\n", irq);
	else
		clear_bit(irq, chip->used);

	mutex_unlock(&chip->lock);
}


static irqreturn_t tegra_pcie_msi_irq(int irq, void *data)
{
	struct tegra_pcie *pcie = data;
	struct tegra_msi *msi = &pcie->msi;
	unsigned int i, processed = 0;

	PR_FUNC_LINE;

	for (i = 0; i < 8; i++) {
		unsigned long reg = afi_readl(pcie, AFI_MSI_VEC0_0 + i * 4);

		while (reg) {
			unsigned int offset = find_first_bit(&reg, 32);
			unsigned int index = i * 32 + offset;
			unsigned int irq;

			/* clear the interrupt */
			afi_writel(pcie, 1 << offset, AFI_MSI_VEC0_0 + i * 4);

			irq = irq_find_mapping(msi->domain, index);
			if (irq) {
				if (test_bit(index, msi->used))
					generic_handle_irq(irq);
				else
					dev_info(pcie->dev, "unhandled MSI\n");
			} else {
				/*
				 * that's weird who triggered this?
				 * just clear it
				 */
				dev_info(pcie->dev, "unexpected MSI\n");
			}

			/* see if there's any more pending in this vector */
			reg = afi_readl(pcie, AFI_MSI_VEC0_0 + i * 4);

			processed++;
		}
	}

	return processed > 0 ? IRQ_HANDLED : IRQ_NONE;
}

static int tegra_msi_setup_irq(struct msi_chip *chip, struct pci_dev *pdev,
			       struct msi_desc *desc)
{
	struct tegra_msi *msi = to_tegra_msi(chip);
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;

	PR_FUNC_LINE;

	hwirq = tegra_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq)
		return -EINVAL;

	irq_set_msi_desc(irq, desc);

	msg.address_lo = virt_to_phys((void *)msi->pages);
	/* 32 bit address only */
	msg.address_hi = 0;
	msg.data = hwirq;

	write_msi_msg(irq, &msg);

	return 0;
}

static void tegra_msi_teardown_irq(struct msi_chip *chip, unsigned int irq)
{
	struct tegra_msi *msi = to_tegra_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);

	PR_FUNC_LINE;
	tegra_msi_free(msi, d->hwirq);
}

static struct irq_chip tegra_msi_irq_chip = {
	.name = "Tegra PCIe MSI",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

static int tegra_msi_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	PR_FUNC_LINE;
	irq_set_chip_and_handler(irq, &tegra_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);
	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = tegra_msi_map,
};

static bool tegra_pcie_enable_msi(struct tegra_pcie *pcie, bool no_init)
{
	struct platform_device *pdev = to_platform_device(pcie->dev);
	struct tegra_msi *msi = &pcie->msi;
	unsigned long base;
	int err;
	u32 reg;

	PR_FUNC_LINE;

	if (!msi->pages) {
		if (no_init)
			return true;

		mutex_init(&msi->lock);

		msi->chip.dev = pcie->dev;
		msi->chip.setup_irq = tegra_msi_setup_irq;
		msi->chip.teardown_irq = tegra_msi_teardown_irq;

		msi->domain = irq_domain_add_linear(pcie->dev->of_node,
			INT_PCI_MSI_NR, &msi_domain_ops, &msi->chip);
		if (!msi->domain) {
			dev_err(&pdev->dev, "failed to create IRQ domain\n");
			return -ENOMEM;
		}

		err = platform_get_irq_byname(pdev, "msi");
		if (err < 0) {
			dev_err(&pdev->dev, "failed to get IRQ: %d\n", err);
			goto err;
		}

		msi->irq = err;
		err = request_irq(msi->irq, tegra_pcie_msi_irq, 0,
				  tegra_msi_irq_chip.name, pcie);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to request IRQ: %d\n", err);
			goto err;
		}

		/* setup AFI/FPCI range */
		msi->pages = __get_free_pages(GFP_KERNEL, 0);
	}
	base = virt_to_phys((void *)msi->pages);

	afi_writel(pcie, base >> 8, AFI_MSI_FPCI_BAR_ST);
	afi_writel(pcie, base, AFI_MSI_AXI_BAR_ST);
	/* this register is in 4K increments */
	afi_writel(pcie, 1, AFI_MSI_BAR_SZ);

	/* enable all MSI vectors */
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC0_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC1_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC2_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC3_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC4_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC5_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC6_0);
	afi_writel(pcie, 0xffffffff, AFI_MSI_EN_VEC7_0);

	/* and unmask the MSI interrupt */
	reg = afi_readl(pcie, AFI_INTR_MASK);
	reg |= AFI_INTR_MASK_MSI_MASK;
	afi_writel(pcie, reg, AFI_INTR_MASK);

	return 0;

err:
	irq_domain_remove(msi->domain);
	return err;
}

static int tegra_pcie_disable_msi(struct tegra_pcie *pcie)
{
	struct tegra_msi *msi = &pcie->msi;
	unsigned int i, irq;
	u32 value;

	PR_FUNC_LINE;

	/* mask the MSI interrupt */
	value = afi_readl(pcie, AFI_INTR_MASK);
	value &= ~AFI_INTR_MASK_MSI_MASK;
	afi_writel(pcie, value, AFI_INTR_MASK);

	/* disable all MSI vectors */
	afi_writel(pcie, 0, AFI_MSI_EN_VEC0_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC1_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC2_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC3_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC4_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC5_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC6_0);
	afi_writel(pcie, 0, AFI_MSI_EN_VEC7_0);

	free_pages(msi->pages, 0);

	if (msi->irq > 0)
		free_irq(msi->irq, pcie);

	for (i = 0; i < INT_PCI_MSI_NR; i++) {
		irq = irq_find_mapping(msi->domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(msi->domain);

	return 0;
}

static void tegra_pcie_read_plat_data(struct tegra_pcie *pcie)
{
	struct device_node *node = pcie->dev->of_node;

	PR_FUNC_LINE;
	pcie->plat_data->gpio_hot_plug =
		of_get_named_gpio(node, "nvidia,hot-plug-gpio", 0);
	pcie->plat_data->gpio_wake =
		of_get_named_gpio(node, "nvidia,wake-gpio", 0);
	pcie->plat_data->gpio_x1_slot =
		of_get_named_gpio(node, "nvidia,x1-slot-gpio", 0);
	pcie->plat_data->has_memtype_lpddr4 =
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

static const struct tegra_pcie_soc_data tegra210_pcie_data = {
	.num_ports = 2,
	.pcie_regulator_names = t210_rail_names,
	.num_pcie_regulators =
			sizeof(t210_rail_names) / sizeof(t210_rail_names[0]),
};

static const struct tegra_pcie_soc_data tegra124_pcie_data = {
	.num_ports = 2,
	.pcie_regulator_names = t124_rail_names,
	.num_pcie_regulators =
			sizeof(t124_rail_names) / sizeof(t124_rail_names[0]),
};

static struct of_device_id tegra_pcie_of_match[] = {
	{ .compatible = "nvidia,tegra210-pcie", .data = &tegra210_pcie_data },
	{ .compatible = "nvidia,tegra124-pcie", .data = &tegra124_pcie_data },
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_pcie_of_match);

static int tegra_pcie_parse_dt(struct tegra_pcie *pcie)
{
	struct tegra_pcie_soc_data *soc = pcie->soc_data;
	struct device_node *np = pcie->dev->of_node, *port;
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	u32 lanes = 0, mask = 0;
	unsigned int lane = 0;
	struct resource res;
	int err;

	PR_FUNC_LINE;

	memset(&pcie->all, 0, sizeof(pcie->all));
	pcie->all.flags = IORESOURCE_MEM;
	pcie->all.name = np->full_name;
	pcie->all.start = ~0;
	pcie->all.end = 0;

	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(pcie->dev, "missing \"ranges\" property\n");
		return -EINVAL;
	}

	for_each_of_pci_range(&parser, &range) {
		of_pci_range_to_resource(&range, np, &res);
		switch (res.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			memcpy(&pcie->io, &res, sizeof(res));
			pcie->io.name = np->full_name;
			break;

		case IORESOURCE_MEM:
			if (res.flags & IORESOURCE_PREFETCH) {
				memcpy(&pcie->prefetch, &res, sizeof(res));
				pcie->prefetch.name = "prefetchable";
			} else {
				memcpy(&pcie->mem, &res, sizeof(res));
				pcie->mem.name = "non-prefetchable";
			}
			break;
		}

		if (res.start <= pcie->all.start)
			pcie->all.start = res.start;

		if (res.end >= pcie->all.end)
			pcie->all.end = res.end;
	}

	err = request_resource(&iomem_resource, &pcie->all);
	if (err < 0)
		return err;

	err = of_pci_parse_bus_range(np, &pcie->busn);
	if (err < 0) {
		dev_err(pcie->dev, "failed to parse ranges property: %d\n",
			err);
		pcie->busn.name = np->name;
		pcie->busn.start = 0;
		pcie->busn.end = 0xff;
		pcie->busn.flags = IORESOURCE_BUS;
	}

	/* parse root ports */
	for_each_child_of_node(np, port) {
		struct tegra_pcie_port *rp;
		unsigned int index;
		u32 value;

		err = of_pci_get_devfn(port);
		if (err < 0) {
			dev_err(pcie->dev, "failed to parse address: %d\n",
				err);
			return err;
		}

		index = PCI_SLOT(err);
		if (index < 1 || index > soc->num_ports) {
			dev_err(pcie->dev, "invalid port number: %d\n", index);
			return -EINVAL;
		}

		index--;

		err = of_property_read_u32(port, "nvidia,num-lanes", &value);
		if (err < 0) {
			dev_err(pcie->dev, "failed to parse # of lanes: %d\n",
				err);
			return err;
		}

		if (value > 16) {
			dev_err(pcie->dev, "invalid # of lanes: %u\n", value);
			return -EINVAL;
		}
		lanes |= value << (index << 3);

		if (!of_device_is_available(port)) {
			lane += value;
			continue;
		}

		mask |= ((1 << value) - 1) << lane;
		lane += value;

		rp = devm_kzalloc(pcie->dev, sizeof(*rp), GFP_KERNEL);
		if (!rp)
			return -ENOMEM;

		err = of_address_to_resource(port, 0, &rp->regs);
		if (err < 0) {
			dev_err(pcie->dev, "failed to parse address: %d\n",
				err);
			return err;
		}

		INIT_LIST_HEAD(&rp->list);
		rp->index = index;
		rp->lanes = value;
		rp->pcie = pcie;
		rp->base = devm_ioremap_resource(pcie->dev, &rp->regs);
		if (IS_ERR(rp->base))
			return PTR_ERR(rp->base);
		rp->has_clkreq = of_property_read_bool(port,
			"nvidia,has_clkreq");
		rp->status = of_device_is_available(port);

		list_add_tail(&rp->list, &pcie->ports);
	}

	return 0;
}

static int __init tegra_pcie_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	const struct of_device_id *match;
	struct tegra_pcie *pcie;

	PR_FUNC_LINE;

#ifdef CONFIG_ARCH_TEGRA_21x_SOC
	if (tegra_bonded_out_dev(BOND_OUT_PCIE)) {
		dev_err(&pdev->dev, "PCIE instance is not present\n");
		return -ENODEV;
	}
#endif

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = &pdev->dev;

	/* use DT way to init platform data */
	pcie->plat_data = devm_kzalloc(pcie->dev,
		sizeof(*(pcie->plat_data)), GFP_KERNEL);
	if (!(pcie->plat_data)) {
		dev_err(pcie->dev, "memory alloc failed\n");
		return -ENOMEM;
	}
	tegra_pcie_read_plat_data(pcie);

	match = of_match_device(tegra_pcie_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;
	pcie->soc_data = (struct tegra_pcie_soc_data *)match->data;

	pcie->pcie_regulators = devm_kzalloc(pcie->dev,
		pcie->soc_data->num_pcie_regulators
			* sizeof(struct regulator *), GFP_KERNEL);

	for (i = 0; i < pcie->soc_data->num_pcie_regulators; i++) {
		pcie->pcie_regulators[i] =
					devm_regulator_get(pcie->dev,
			pcie->soc_data->pcie_regulator_names[i]);
		if (IS_ERR(pcie->pcie_regulators[i])) {
			dev_err(pcie->dev, "%s: unable to get regulator %s\n",
			__func__,
			pcie->soc_data->pcie_regulator_names[i]);
			pcie->pcie_regulators[i] = NULL;
		}
	}

	INIT_LIST_HEAD(&pcie->buses);
	INIT_LIST_HEAD(&pcie->ports);
	INIT_LIST_HEAD(&pcie->sys);

	ret = tegra_pcie_parse_dt(pcie);
	if (ret < 0)
		return ret;

	/* Enable Runtime PM for PCIe, TODO: Need to add PCIe host device */
	pm_runtime_enable(pcie->dev);

	ret = tegra_pcie_init(pcie);
	if (ret) {
		tegra_pd_remove_device(pcie->dev);
		__pm_runtime_disable(pcie->dev, false);
	}

	platform_set_drvdata(pdev, pcie);

	return ret;
}

static int tegra_pcie_remove(struct platform_device *pdev)
{
	struct tegra_pcie *pcie = platform_get_drvdata(pdev);
	struct tegra_pcie_bus *bus;

	PR_FUNC_LINE;
	list_for_each_entry(bus, &pcie->buses, list) {
		vunmap(bus->area->addr);
		kfree(bus);
	}
	if (IS_ENABLED(CONFIG_PCI_MSI))
		tegra_pcie_disable_msi(pcie);
	tegra_pcie_detach(pcie);
	tegra_pd_remove_device(pcie->dev);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_pcie_suspend_noirq(struct device *dev)
{
	int ret = 0;
	struct tegra_pcie *pcie = dev_get_drvdata(dev);

	PR_FUNC_LINE;
	/* configure PE_WAKE signal as wake sources */
	if (gpio_is_valid(pcie->plat_data->gpio_wake) &&
			device_may_wakeup(dev)) {
		ret = enable_irq_wake(gpio_to_irq(
			pcie->plat_data->gpio_wake));
		if (ret < 0) {
			dev_err(dev,
				"ID wake-up event failed with error %d\n", ret);
			return ret;
		}
	}
	return tegra_pcie_power_off(pcie, true);
}

static bool tegra_pcie_enable_msi(struct tegra_pcie *, bool);

static int tegra_pcie_resume_noirq(struct device *dev)
{
	int ret = 0;
	struct tegra_pcie *pcie = dev_get_drvdata(dev);

	PR_FUNC_LINE;
	if (gpio_is_valid(pcie->plat_data->gpio_wake) &&
			device_may_wakeup(dev)) {
		ret = disable_irq_wake(gpio_to_irq(
			pcie->plat_data->gpio_wake));
		if (ret < 0) {
			dev_err(dev,
				"ID wake-up event failed with error %d\n", ret);
			return ret;
		}
	}
	/* give 100ms for 1.05v to come up */
	msleep(100);
	ret = tegra_pcie_power_on(pcie);
	if (ret) {
		dev_err(dev, "PCIE: Failed to power on: %d\n", ret);
		return ret;
	}
	tegra_pcie_enable_pads(pcie, true);
	tegra_pcie_enable_controller(pcie);
	tegra_pcie_setup_translations(pcie);
	/* Set up MSI registers, if MSI have been enabled */
	tegra_pcie_enable_msi(pcie, true);

	tegra_pcie_check_ports(pcie);
	if (!pcie->num_ports) {
		tegra_pcie_power_off(pcie, true);
		goto exit;
	}

exit:
	return 0;
}

static int tegra_pcie_resume(struct device *dev)
{
	struct tegra_pcie *pcie = dev_get_drvdata(dev);
	PR_FUNC_LINE;
	tegra_pcie_enable_features(pcie);
	return 0;
}

static const struct dev_pm_ops tegra_pcie_pm_ops = {
	.suspend_noirq  = tegra_pcie_suspend_noirq,
	.resume_noirq = tegra_pcie_resume_noirq,
	.resume = tegra_pcie_resume,
	};
#endif /* CONFIG_PM */

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

