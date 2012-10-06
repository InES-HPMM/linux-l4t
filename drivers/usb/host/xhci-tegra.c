/*
 * xhci-tegra.c - Nvidia xHCI host controller driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/tegra_xusb.h>
#include <linux/uaccess.h>
#include <mach/powergate.h>
#include <mach/clk.h>
#include "xhci-tegra.h"
#include "xhci.h"

/* macros */
#define PAGE_SELECT_MASK			0xFFFFFE00
#define PAGE_SELECT_SHIFT			9
#define PAGE_OFFSET_MASK			0x000001FF
#define CSB_PAGE_SELECT(_addr)						\
	({								\
		typecheck(u32, _addr);					\
		((_addr & PAGE_SELECT_MASK) >> PAGE_SELECT_SHIFT);	\
	})
#define CSB_PAGE_OFFSET(_addr)						\
	({								\
		typecheck(u32, _addr);					\
		(_addr & PAGE_OFFSET_MASK);				\
	})

/* PMC register definition */
#define PMC_PORT_UTMIP_P0		0
#define PMC_PORT_UTMIP_P1		1
#define PMC_PORT_UTMIP_P2		2
#define PMC_PORT_UHSIC_P0		3
#define PMC_PORT_NUM			4

#define PMC_USB_DEBOUNCE_DEL_0			0xec
#define   UTMIP_LINE_DEB_CNT(x)		(((x) & 0xf) << 16)
#define   UTMIP_LINE_DEB_CNT_MASK		(0xf << 16)

#define PMC_USB_AO_0				0xf0
#define   USBOP_VAL_PD(x)			(1 << (((x) << 2) + 0))
#define   USBON_VAL_PD(x)			(1 << (((x) << 2) + 1))

#define PMC_UTMIP_UHSIC_TRIGGERS_0		0x1ec
#define   UTMIP_CLR_WALK_PTR(x)		(1 << (0 + (x)))
#define   UTMIP_CAP_CFG(x)			(1 << (4 + (x)))
#define   UTMIP_CLR_WAKE_ALARM(x)		(1 << (12 + (x)))

#define PMC_UTMIP_UHSIC_SAVED_STATE_0		0x1f0
#define	UTMIP_WAKE_EX(x)		(1 << (((x) << 3) + 7))
#define	UTMIP_SPEED(x, v)		(((v) & 0x3) << (((x) << 3) + 0))
#define	UTMIP_SPEED_MASK(x)		((0x3) << (((x) << 3) + 0))
#define	UTMIP_SPEED_HS			0
#define	UTMIP_SPEED_FS			1
#define	UTMIP_SPEED_LS			2
#define	UTMIP_SPEED_RST			3

#define PMC_UTMIP_UHSIC_SLEEP_CFG_0		0x1fc
#define   UTMIP_UHSIC_MASTER_ENABLE(x)		(1 << (((x) << 3) + 0))
#define   UTMIP_UHSIC_FSLS_USE_PMC(x)		(1 << (((x) << 3) + 1))
#define   UTMIP_UHSIC_RCTRL_USE_PMC(x)		(1 << (((x) << 3) + 2))
#define   UTMIP_UHSIC_TCTRL_USE_PMC(x)		(1 << (((x) << 3) + 3))

#define   UTMIP_WAKE_VAL(x, v)			((v) << (((x) << 3) + 4))
#define   UTMIP_WAKE_VAL_MASK(x)		(0xf << (((x) << 3) + 4))

#define   WAKE_VAL_NONE			0xc
#define   WAKE_VAL_ANY				0xf
#define   WAKE_VAL_FSJ				0x2
#define   WAKE_VAL_FSK				0x1
#define   WAKE_VAL_SE0				0x0

#define PMC_UTMIP_UHSIC_SLEEPWALK_CFG_0	0x200
#define   UTMIP_GPIO_WALK_EN(x)		(1 << (((x) << 3) + 5))
#define   UTMIP_WAKE_WALK_EN(x)		(1 << (((x) << 3) + 6))
#define   UTMIP_LINEVAL_WALK_EN(x)		(1 << (((x) << 3) + 7))

#define PMC_UTMIP_SLEEPWALK_Px_0(x)		(0x204 + ((x) << 2))
#define   USBOP_RPD_A				(1 << 0)
#define   USBON_RPD_A				(1 << 1)
#define   USBOP_RPU_A				(1 << 2)
#define   USBON_RPU_A				(1 << 3)
#define   AP_A					(1 << 4)
#define   AN_A					(1 << 5)
#define   HIGHZ_A				(1 << 6)
#define   RESERVED_A				(1 << 7)
#define   USBOP_RPD_B				(1 << 8)
#define   USBON_RPD_B				(1 << 9)
#define   USBOP_RPU_B				(1 << 10)
#define   USBON_RPU_B				(1 << 11)
#define   AP_B					(1 << 12)
#define   AN_B					(1 << 13)
#define   HIGHZ_B				(1 << 14)
#define   RESERVED_B				(1 << 15)
#define   USBOP_RPD_C				(1 << 16)
#define   USBON_RPD_C				(1 << 17)
#define   USBOP_RPU_C				(1 << 18)
#define   USBON_RPU_C				(1 << 19)
#define   AP_C					(1 << 20)
#define   AN_C					(1 << 21)
#define   HIGHZ_C				(1 << 22)
#define   RESERVED_C				(1 << 23)
#define   USBOP_RPD_D				(1 << 24)
#define   USBON_RPD_D				(1 << 25)
#define   USBOP_RPU_D				(1 << 26)
#define   USBON_RPU_D				(1 << 27)
#define   AP_D					(1 << 28)
#define   AN_D					(1 << 29)
#define   HIGHZ_D				(1 << 30)
#define   RESERVED_D				(1 << 31)

#define PMC_UTMIP_UHSIC_FAKE_0			0x218
#define   UTMIP_FAKE_USBOP_VAL(x)		(1 << (((x) << 2) + 0))
#define   UTMIP_FAKE_USBON_VAL(x)		(1 << (((x) << 2) + 1))
#define   UTMIP_FAKE_USBOP_EN(x)		(1 << (((x) << 2) + 2))
#define   UTMIP_FAKE_USBON_EN(x)		(1 << (((x) << 2) + 3))

#define PMC_UTMIP_UHSIC_LINE_WAKEUP_0		0x26c
#define   UTMIP_LINE_WAKEUP_EN(x)		(1 << (x))

#define PMC_UTMIP_MASTER_CONFIG_0		0x274
#define   UTMIP_PWR(x)				(1 << (x))

#define PMC_UTMIP_UHSIC2_SLEEP_CFG_0		0x284

#define PMC_UTMIP_UHSIC2_LINE_WAKEUP_0		0x298
#define   UHSIC_LINE_WAKEUP_EN_P1		(1 << 0)

/* private data types */
/* command requests from the firmware */
enum MBOX_CMD_TYPE {
	MBOX_CMD_INC_FALC_CLOCK = 1,
	MBOX_CMD_DEC_FALC_CLOCK,
	MBOX_CMD_INC_SSPI_CLOCK,
	MBOX_CMD_DEC_SSPI_CLOCK,
	MBOX_CMD_SET_BW,
	MBOX_CMD_SET_SS_PWR_GATING,
	MBOX_CMD_SET_SS_PWR_UNGATING,

	/* needs to be the last cmd */
	MBOX_CMD_MAX,

	/* resp msg to ack above commands */
	MBOX_CMD_ACK,
	MBOX_CMD_NACK
};

/* Usb3 Firmware Cfg Table */
struct cfgtbl {
	u32 boot_loadaddr_in_imem;
	u32 boot_codedfi_offset;
	u32 boot_codetag;
	u32 boot_codesize;

	/* Physical memory reserved by Bootloader/BIOS */
	u32 phys_memaddr;
	u16 reqphys_memsize;
	u16 alloc_phys_memsize;

	/* .rodata section */
	u32 rodata_img_offset;
	u32 rodata_section_start;
	u32 rodata_section_end;
	u32 main_fnaddr;

	u32 fwimg_cksum;
	u32 fwimg_created_time;

	/* Fields that get filled by linker during linking phase
	 * or initialized in the FW code.
	 */
	u32 imem_resident_start;
	u32 imem_resident_end;
	u32 idirect_start;
	u32 idirect_end;
	u32 l2_imem_start;
	u32 l2_imem_end;
	u32 version_id;
	u8 init_ddirect;

	/*	Below two dummy variables are used to replace
	 *	L2IMemSymTabOffsetInDFI and L2IMemSymTabSize in order to
	 *	retain the size of struct _CFG_TBL used by other AP/Module.
	 */
	u32 dummy_var1;
	u32 dummy_var2;

	/* fwimg_len */
	u32 fwimg_len;
};

struct xusb_save_regs {
	u32 msi_bar_sz;
	u32 msi_axi_barst;
	u32 msi_fpci_barst;
	u32 msi_vec0;
	u32 msi_en_vec0;
	u32 fpci_error_masks;
	u32 intr_mask;
	u32 ipfs_intr_enable;
	u32 ufpci_config;
	u32 clkgate_hysteresis;
	u32 xusb_host_mccif_fifo_cntrl;

	/* PG does not mention below */
	u32 hs_pls;
	u32 fs_pls;
	u32 hs_fs_speed;
	u32 hs_fs_pp;
	u32 cfg_aru;
	u32 cfg_order;
	u32 cfg_fladj;
	u32 cfg_sid;
};

struct tegra_xhci_hcd {
	struct platform_device *pdev;
	struct xhci_hcd *xhci;

	spinlock_t lock;
	struct mutex sync_lock;

	int smi_irq;
	int padctl_irq;
	int usb3_irq;

	bool ss_wake_event;
	bool ss_pwr_gated;
	bool host_pwr_gated;
	bool hs_wake_event;
	bool host_resume_req;
	bool lp0_exit;
	unsigned long last_jiffies;
	unsigned long host_phy_base;

	void __iomem *pmc_base;
	void __iomem *padctl_base;
	void __iomem *fpci_base;
	void __iomem *ipfs_base;

	struct tegra_xusb_pad_data *xusb_padctl;

	/* mailbox variables */
	struct mutex mbox_lock;
	u32 mbox_owner;
	u32 cmd_type;
	u32 cmd_data;

	struct regulator *xusb_vbus_reg;
	struct regulator *xusb_avddio_usb3_reg;
	struct regulator *xusb_vddio_hsic_reg;
	struct regulator *xusb_hvdd_usb3_reg;
	struct regulator *xusb_avdd_usb3_pll_reg;

	struct work_struct mbox_work;
	struct work_struct ss_elpg_exit_work;
	struct work_struct host_elpg_exit_work;

	struct clk *host_partition_clk;
	struct clk *ss_partition_clk;
	struct clk *dev_partition_clk;

	/* XUSB Core Host Clock */
	struct clk *host_clk;

	/* XUSB Falcon SuperSpeed Clock */
	struct clk *falc_clk;

	/* EMC Clock */
	struct clk *emc_clk;

	/* XUSB SS PI Clock */
	struct clk *ss_clk;

	/* XUSB HS PI Clock */
	struct clk *hs_clk;

	/* PLLU Clock */
	struct clk *pllu_clk;

	/* PLLE Clock */
	struct clk *plle_clk;

	/* UTMIP Clock */
	struct clk *utmip_clk;

	/* FS Clock */
	struct clk *fs_clk;

	/* Dev Clock */
	struct clk *dev_clk;

	struct clk *pll_u_480M;
	struct clk *clk_m;

	struct clk *plle_re_vco_clk;

	struct clk *pll_u_60M;

	struct clk *pll_p_clk;

	struct clk *pll_u_48M;

	/*
	 * XUSB/IPFS specific registers these need to be saved/restored in
	 * addition to spec defined registers
	 */
	struct xusb_save_regs sregs;
	bool usb2_rh_suspend;
	bool usb3_rh_suspend;
	bool hc_in_elpg;

	/* firmware loading related */
	u32 fw_phys_base;
	u32 fw_virt_base;
	u32 fw_size;
};

/* functions */
static inline struct tegra_xhci_hcd *hcd_to_tegra_xhci(struct usb_hcd *hcd)
{
	return (struct tegra_xhci_hcd *) dev_get_drvdata(hcd->self.controller);
}

#if defined(CONFIG_DEBUG_MUTEXES) || defined(CONFIG_SMP)
static inline void must_have_sync_lock(struct tegra_xhci_hcd *tegra)
{
	WARN_ON(tegra->sync_lock.owner != current);
}
#else
static inline void must_have_sync_lock(struct tegra_xhci_hcd *tegra)
#endif

static void debug_print_portsc(struct xhci_hcd *xhci)
{
	__le32 __iomem *addr;
	int i;
	int ports;

	ports = HCS_MAX_PORTS(xhci->hcs_params1);
	addr = &xhci->op_regs->port_status_base;
	for (i = 0; i < ports; i++) {
		xhci_dbg(xhci, "%p port %d status reg = 0x%x\n",
				addr, i, (unsigned int) xhci_readl(xhci, addr));
		addr += NUM_PORT_REGS;
	}
}

static int tegra_xhci_pmc_usb2_wakenotif_init(struct tegra_xhci_hcd *tegra,
	int port, u32 *portsc)
{
	void __iomem *pmc_base = tegra->pmc_base;
	struct platform_device *pdev = tegra->pdev;
	struct xhci_hcd	*xhci = tegra->xhci;
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	int i;
	u32 val;

	/*
	 * MASTER_ENABLE_Px = 0.
	 * Also program UTMIP_PWR_P2 = 1 for power savings mode.
	 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
	if (val & UTMIP_UHSIC_MASTER_ENABLE(port)) {
		dev_err(&pdev->dev, "error: UTMIP_MASTER_ENABLE_P2 != 0\n");
		return -1;
	}

	/* PMC_UTMIP_MASTER_CONFIG_0 */
	val = readl(pmc_base + PMC_UTMIP_MASTER_CONFIG_0);
	val |= UTMIP_PWR(port);
	writel(val, pmc_base + PMC_UTMIP_MASTER_CONFIG_0);

	/*
	 * B. Program debouncer to some value
	 * (at least test 0 and non-zero value)
	 * UTMIP_LINE_DEB_CNT=0
	 */
	val = readl(pmc_base + PMC_USB_DEBOUNCE_DEL_0);
	val &= ~UTMIP_LINE_DEB_CNT_MASK;
	val |= UTMIP_LINE_DEB_CNT(1);
	writel(val, pmc_base + PMC_USB_DEBOUNCE_DEL_0);

	/*
	 * C. Make sure nothing is happening on the line with respect to PMC
	 * UTMIP_FAKE_USBON_EN_Px=0
	 * UTMIP_FAKE_USBOP_EN_Px=0
	 * UTMIP_FAKE_USBON_VAL_Px=0
	 * UTMIP_FAKE_USBOP_VAL_Px=0
	 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_FAKE_0);
	val &= ~(UTMIP_FAKE_USBOP_VAL(port) | UTMIP_FAKE_USBON_VAL(port) |
		 UTMIP_FAKE_USBOP_EN(port) | UTMIP_FAKE_USBON_EN(port));
	writel(val, pmc_base + PMC_UTMIP_UHSIC_FAKE_0);

	/*
	 * D. Make sure wake value for line is none, program the following in
	 * sequence:
	 * 1. UTMIP_LINE_WAKEUP_EN_Px=0
	 * 2. UTMIP_WAKE_VAL_Px=NONE
	 * 3. UTMIP_LINE_WAKEUP_EN_Px=1
	 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);
	val &= ~UTMIP_LINE_WAKEUP_EN(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);

	val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
	val &= ~UTMIP_WAKE_VAL_MASK(port);
	val |= UTMIP_WAKE_VAL(port, WAKE_VAL_NONE);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

	val = readl(pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);
	val |= UTMIP_LINE_WAKEUP_EN(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);

	/*
	 * E. Make sure pad detectors are off:
	 * USBOP_VAL_PD_Px=1
	 * USBON_VAL_PD_Px=1
	 */
	val = readl(pmc_base + PMC_USB_AO_0);
	val |= (USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	writel(val, pmc_base + PMC_USB_AO_0);

	/* FIXME: mapping between UTMIP to PORTSC should be corrected */
	*portsc = readl(hcd->regs +
		BAR0_XHCI_OP_PORTSC(port + BAR0_XHCI_OP_PORTSC_UTMIP_0));

	val = readl(pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);
	val &= ~UTMIP_SPEED_MASK(port);
	if (DEV_FULLSPEED(*portsc))
		val |= UTMIP_SPEED(port, UTMIP_SPEED_FS);
	else if (DEV_LOWSPEED(*portsc))
		val |= UTMIP_SPEED(port, UTMIP_SPEED_LS);
	else if (DEV_HIGHSPEED(*portsc))
		val |= UTMIP_SPEED(port, UTMIP_SPEED_HS);

	writel(val, pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);

	/* PMC_UTMIP_UHSIC_LINE_WAKEUP_0 */
	for (i = 0; i < PMC_PORT_NUM; i++) {
		val = readl(pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);
		val &= ~UTMIP_LINE_WAKEUP_EN(i);
		writel(val, pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);
	}

	/* UHSIC2_LINE_WAKEUP_EN_P1=0 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC2_LINE_WAKEUP_0);
	val &= ~UHSIC_LINE_WAKEUP_EN_P1;
	writel(val, pmc_base + PMC_UTMIP_UHSIC2_LINE_WAKEUP_0);

	/*
	 * G. Remove fake values and make synchronizers work a bit.
	 * (TODO: Duplicate with #C?)
	 * UTMIP_FAKE_USBOP_EN_P2=0
	 * UTMIP_FAKE_USBON_EN_P2=0
	 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_FAKE_0);
	val &= ~UTMIP_FAKE_USBOP_EN(port);
	val &= ~UTMIP_FAKE_USBON_EN(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_FAKE_0);

	/*
	 * H. Enable which type of event can trigger a walk to respond from a
	 * wakeup. GPIO N is connected to the baseband and USB Remote Wakeup
	 * are possible:
	 */

	/* UTMIP_GPIO_WALK_EN_P2=1 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEPWALK_CFG_0);
	val &= ~UTMIP_GPIO_WALK_EN(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEPWALK_CFG_0);

	/* UTMIP_WAKE_WALK_EN_P2=0 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEPWALK_CFG_0);
	val &= ~UTMIP_WAKE_WALK_EN(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEPWALK_CFG_0);

	/* UTMIP_LINEVAL_WALK_EN_P2=1 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEPWALK_CFG_0);
	val |= UTMIP_LINEVAL_WALK_EN(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEPWALK_CFG_0);

	/*
	 * I. Clear the Walk Pointers and Capture the FS/LS pad configuration.
	 *    TODO: Read captured config and at least verify LO_SPD is correct.
	 * UTMIP_CLR_WALK_PTR_P2=1
	 * UTMIP_CAP_CFG_P2=1
	 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_TRIGGERS_0);
	val |= (UTMIP_CLR_WALK_PTR(port) | UTMIP_CAP_CFG(port)
		| UTMIP_CLR_WAKE_ALARM(port));
	writel(val, pmc_base + PMC_UTMIP_UHSIC_TRIGGERS_0);

	/*
	 * J. Read the UB_RCTRL and UB_TCTRL field from UTMIP regs
	 * (see arutmip.spec) and program RCTRL_VAL and TCTRL_VAL.
	 * Ensure you program a thermally encoded value by using the most
	 * significant bit turned on in UB_RCTRL and UB_TCTRL to create
	 * RCTRL_VAL and TCTRL_VAL respectively.
	 * TODO: RCTRL_VAL=Thermal Encoding
	 * TODO: TCTRL_VAL=Thermal Encoding
	 * K. Program walk sequence
	 * For UTMIP, it is host pull downs to help the device maintain a J on
	 * the line. Followed by a driven FS K or LS K to signal a resume once
	 * an walk event is detected
	 * For UHSIC, it is host pull downs to maintain an idle on the line.
	 * Followed by a driven resume once an walk event is detected

	 * UTMIP_SLEEPWALK_P0:
	 * UTMIP_SLEEPWALK_P2:
	 */
	if (!(*portsc & PORT_CONNECT)) {
		/* Unconnected */
		val = USBOP_RPD_A | USBON_RPD_A | HIGHZ_A |
			USBOP_RPD_B | USBON_RPD_B | HIGHZ_B |
			USBOP_RPD_C | USBON_RPD_C | HIGHZ_C |
			USBOP_RPD_D | USBON_RPD_D | HIGHZ_D;
	} else if (DEV_LOWSPEED(*portsc)) {
		/* LowSpeed */
		val = USBOP_RPD_A | USBON_RPD_A | AP_A | HIGHZ_A |
			USBOP_RPD_B | USBON_RPD_B | AP_B |
			USBOP_RPD_C | USBON_RPD_C | AP_C |
			USBOP_RPD_D | USBON_RPD_D | AP_D;
	} else {
		/* FullSpeed or HighSpeed */
		val = USBOP_RPD_A | USBON_RPD_A | AN_A | HIGHZ_A |
			USBOP_RPD_B | USBON_RPD_B | AN_B |
			USBOP_RPD_C | USBON_RPD_C | AN_C |
			USBOP_RPD_D | USBON_RPD_D | AN_D;
	}
	writel(val, pmc_base + PMC_UTMIP_SLEEPWALK_Px_0(port));

	/*
	 * L. Place USB2 ports in suspend. There is now at least 5ms before a
	 * remote device could issue a wakeup and the time MASTER_ENABLE can
	 * be set to 1.
	 */

	return 0;
}

static void tegra_xhci_pmc_usb2_wakenotif_on(struct tegra_xhci_hcd *tegra,
	int port, u32 portsc)
{
	void __iomem *pmc_base = tegra->pmc_base;
	struct xhci_hcd *xhci = tegra->xhci;
	u32 val;

	/*
	 * M. Power up the value detectors:
	 * USBOP_VAL_PD_P2=0
	 * USBON_VAL_PD_P2=0
	 */
	val = readl(pmc_base + PMC_USB_AO_0);
	val &= ~(USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	writel(val, pmc_base + PMC_USB_AO_0);

	/*
	 * N. Turn over pad configuration to PMC and set the UTMIP_WAKE_VAL_P0
	 * to reflect a high speed resume signaling by a device:
	 * UTMIP_RCTRL_USE_PMC_P0=1
	 * UTMIP_TCTRL_USE_PMC_P0=1
	 * UTMIP_FSLS_USE_PMC_P0=1
	 */

	if (!(portsc & PORT_CONNECT)) {
		/*
		 * Unconnected with wake on connect enabled
		 * UTMIP_WAKE_EX_P2=ON
		 * UTMIP_WAKE_VAL_P2=SE0
		 * UTMIP_MASTER_ENABLE_P2=1
		 */

		if (portsc & PORT_WKCONN_E) {
			xhci_info(xhci,
				"Unconnected with wake on connect enabled\n");

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);
			val |= UTMIP_WAKE_EX(port);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val &= ~UTMIP_WAKE_VAL_MASK(port);
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_SE0);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val |= (UTMIP_UHSIC_MASTER_ENABLE(port)
				| UTMIP_UHSIC_FSLS_USE_PMC(port)
				| UTMIP_UHSIC_RCTRL_USE_PMC(port)
				| UTMIP_UHSIC_TCTRL_USE_PMC(port));
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
		}
	} else if (DEV_LOWSPEED(portsc)) {
		if (portsc & PORT_WKDISC_E) {
			/*
			 * LowSpeed with wake on disconnected enabled
			 * UTMIP_WAKE_EX_P2=ON
			 * UTMIP_WAKE_VAL_P2=FSK
			 * UTMIP_MASTER_ENABLE_P2=1
			 */

			xhci_info(xhci,
				"LS with wake on disconnected enabled\n");

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);
			val |= UTMIP_WAKE_EX(port);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val &= ~UTMIP_WAKE_VAL_MASK(port);
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_FSK);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val |= (UTMIP_UHSIC_MASTER_ENABLE(port)
				| UTMIP_UHSIC_FSLS_USE_PMC(port)
				| UTMIP_UHSIC_RCTRL_USE_PMC(port)
				| UTMIP_UHSIC_TCTRL_USE_PMC(port));
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
		} else {
			/*
			 * LowSpeed with wake on disconnected disabled
			 * UTMIP_WAKE_VAL_P2=FSJ
			 * UTMIP_MASTER_ENABLE_P2=1
			 */

			xhci_info(xhci,
				"LS with wake on disconnected disabled\n");

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val &= ~UTMIP_WAKE_VAL_MASK(port);
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_FSJ);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val |= (UTMIP_UHSIC_MASTER_ENABLE(port)
				| UTMIP_UHSIC_FSLS_USE_PMC(port)
				| UTMIP_UHSIC_RCTRL_USE_PMC(port)
				| UTMIP_UHSIC_TCTRL_USE_PMC(port));
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
		}
	} else {
		if (portsc & PORT_WKDISC_E) {
			/*
			 * Full or High Speed with wake on disconnected enabled
			 * UTMIP_WAKE_EX_P2=ON
			 * UTMIP_WAKE_VAL_P2=FSJ
			 * UTMIP_MASTER_ENABLE_P2=1
			 */
			xhci_info(xhci,
				"FS/HS with wake on disconnected enabled\n");

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);
			val |= UTMIP_WAKE_EX(port);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val &= ~UTMIP_WAKE_VAL_MASK(port);
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_FSJ);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val |= (UTMIP_UHSIC_MASTER_ENABLE(port)
				| UTMIP_UHSIC_FSLS_USE_PMC(port)
				| UTMIP_UHSIC_RCTRL_USE_PMC(port)
				| UTMIP_UHSIC_TCTRL_USE_PMC(port));
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

		} else {
			/*
			 * Full or High Speed with wake on disconnect disabled
			 * UTMIP_WAKE_VAL_P0=FSK
			 * UTMIP_MASTER_ENABLE_P0=1
			 */
			xhci_info(xhci,
				"FS/HS with wake on disconnected disabled\n");

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);
			val |= UTMIP_WAKE_EX(port);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val &= ~UTMIP_WAKE_VAL_MASK(port);
			val |= UTMIP_WAKE_VAL(port, WAKE_VAL_FSK);
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

			val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
			val |= (UTMIP_UHSIC_MASTER_ENABLE(port)
				| UTMIP_UHSIC_FSLS_USE_PMC(port)
				| UTMIP_UHSIC_RCTRL_USE_PMC(port)
				| UTMIP_UHSIC_TCTRL_USE_PMC(port));
			writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
		}
	}

	/* UTMIP_LINE_WAKEUP_EN_P2=1 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);
	val |= UTMIP_LINE_WAKEUP_EN(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_LINE_WAKEUP_0);

	/*
	 * O. Place XUSB in ELPG or keep it on.
	 * P. Wait for a wake or walk event
	 */
}

static void tegra_xhci_pmc_usb2_wakenotif_off(struct tegra_xhci_hcd *tegra,
	int port)
{
	void __iomem *pmc_base = tegra->pmc_base;
	u32 val;

	/*
	 * P. Exit from ELPG if necessary, reset XUSB controller and keep USB2.0
	 * ports in suspend. Note that this must be done all while an RESUME is
	 * driven on the line.
	 * Q. Let the XUSB PI take over the port from PMC when it is known the
	 * PI is drivng the RESUME signaling at the PHY and ready to release
	 * suspend:
	 */

	/* UTMIP_WAKE_EX_P2 = OFF */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);
	val &= ~UTMIP_WAKE_EX(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_SAVED_STATE_0);

	/*
	 * UTMIP_MASTER_ENABLE_P2 = 0
	 * UTMIP_FSLS_USE_PMC_P2 = 0
	 * UTMIP_RCTRL_USE_PMC_P2 = 0
	 * UTMIP_TCTRL_USE_PMC_P2 = 0
	 * UTMIP_WAKE_VAL_P2 = NONE
	 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
	val &= ~(UTMIP_UHSIC_MASTER_ENABLE(port) |
		UTMIP_UHSIC_FSLS_USE_PMC(port) |
		UTMIP_UHSIC_RCTRL_USE_PMC(port) |
		UTMIP_UHSIC_TCTRL_USE_PMC(port));
	val &= ~UTMIP_WAKE_VAL_MASK(port);
	val |= UTMIP_WAKE_VAL(port, WAKE_VAL_NONE);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

	/*
	 * USBOP_VAL_PD_P2 = 1
	 * USBON_VAL_PD_P2 = 1
	 */
	val = readl(pmc_base + PMC_USB_AO_0);
	val |= (USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	writel(val, pmc_base + PMC_USB_AO_0);

	/*
	 * UTMIP_FAKE_USBOP_EN_P2 = 1
	 * UTMIP_FAKE_USBON_EN_P2 = 1
	 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_FAKE_0);
	val |= (UTMIP_FAKE_USBOP_EN(port) | UTMIP_FAKE_USBON_EN(port));
	writel(val, pmc_base + PMC_UTMIP_UHSIC_FAKE_0);

	/* UTMIP_CLR_WAKE_ALARM_P2=1 */
	val = readl(pmc_base + PMC_UTMIP_UHSIC_TRIGGERS_0);
	val |= UTMIP_CLR_WAKE_ALARM(port);
	writel(val, pmc_base + PMC_UTMIP_UHSIC_TRIGGERS_0);
}

u32 csb_read(struct tegra_xhci_hcd *tegra, u32 addr)
{
	void __iomem *fpci_base = tegra->fpci_base;
	struct platform_device *pdev = tegra->pdev;
	u32 input_addr;
	u32 data;
	u32 csb_page_select;

	/* to select the appropriate CSB page to write to */
	csb_page_select = CSB_PAGE_SELECT(addr);

	dev_dbg(&pdev->dev, "csb_read: csb_page_select= 0x%08x\n",
			csb_page_select);

	iowrite32(csb_page_select, fpci_base + XUSB_CFG_ARU_C11_CSBRANGE);

	/* selects the appropriate offset in the page to read from */
	input_addr = CSB_PAGE_OFFSET(addr);
	data = ioread32(fpci_base + XUSB_CFG_CSB_BASE_ADDR + input_addr);

	dev_dbg(&pdev->dev, "csb_read: input_addr = 0x%08x data = 0x%08x\n",
			input_addr, data);
	return data;
}

void csb_write(struct tegra_xhci_hcd *tegra, u32 addr, u32 data)
{
	void __iomem *fpci_base = tegra->fpci_base;
	struct platform_device *pdev = tegra->pdev;
	u32 input_addr;
	u32 csb_page_select;

	/* to select the appropriate CSB page to write to */
	csb_page_select = CSB_PAGE_SELECT(addr);

	dev_dbg(&pdev->dev, "csb_write:csb_page_selectx = 0x%08x\n",
			csb_page_select);

	iowrite32(csb_page_select, fpci_base + XUSB_CFG_ARU_C11_CSBRANGE);

	/* selects the appropriate offset in the page to write to */
	input_addr = CSB_PAGE_OFFSET(addr);
	iowrite32(data, fpci_base + XUSB_CFG_CSB_BASE_ADDR + input_addr);

	dev_dbg(&pdev->dev, "csb_write: input_addr = 0x%08x data = %0x08x\n",
			input_addr, data);
}

static int tegra_xhci_load_firmware(struct tegra_xhci_hcd *tegra, bool reset)
{
	struct platform_device *pdev = tegra->pdev;
	u32 fw_virt_base = tegra->fw_virt_base;
	u32 fw_phys_base = tegra->fw_phys_base;
	u32 fw_size = tegra->fw_size;
	u64 phys_addr_lo, phys_addr_hi = 0;
	u32 HwReg, boot_start, imem_block_start;
	u32 boot_code_tag, boot_code_pc, imem_block_size;
	struct cfgtbl *cfg_tbl;
	u16 nblocks, rbytes;
	time_t fw_time;
	struct tm fw_tm;
	int ret = 0;

	/* First thing, reset the ARU. By the time we get to
	 * loading boot code below, reset would be complete.
	 * alternatively we can busy wait on rst pending bit.
	 */
	/* Don't reset during ELPG/LP0 exit path */
	if (reset) {
		iowrite32(0x1, tegra->fpci_base + XUSB_CFG_ARU_RST);
		usleep_range(1000, 2000);
	}

	if (csb_read(tegra, XUSB_CSB_MP_ILOAD_BASE_LO) != 0) {

		dev_dbg(&pdev->dev, "fw already loaded by BIOS\n");
		dev_dbg(&pdev->dev, "falcon state = %x\n",
				csb_read(tegra, XUSB_FALC_CPUCTL));
		return 0;
	}
	imem_block_size = IMEM_BLOCK_SIZE;
	/* TODO: use readl/writel to access mapped memory region */
	cfg_tbl = (struct cfgtbl *)fw_virt_base;

	fw_time = cfg_tbl->fwimg_created_time;
	time_to_tm(fw_time, 0, &fw_tm);
	dev_dbg(&pdev->dev, "%s: fwimg_created_time %ld-%02d-%02d "\
		"%02d:%02d:%02d UTC\n", __func__, fw_tm.tm_year + 1900,
		fw_tm.tm_mon + 1, fw_tm.tm_mday, fw_tm.tm_hour, fw_tm.tm_min,
		fw_tm.tm_sec);

	/* Offset from start of code region in DFI
	 * (does not include 256 byte cfgtbl)
	 */

	boot_start = (u32)((u8 *)cfg_tbl->boot_codedfi_offset  + 256);
	imem_block_start = cfg_tbl->boot_loadaddr_in_imem/IMEM_BLOCK_SIZE;
	boot_code_pc = cfg_tbl->boot_codetag;

	/* below to be programmed to BOOTVEC as start of boot code */
	boot_code_tag = cfg_tbl->boot_codetag/256;

	nblocks = cfg_tbl->boot_codesize/imem_block_size;
	rbytes = cfg_tbl->boot_codesize % imem_block_size;

	phys_addr_lo = (u32) ((char *)fw_phys_base + 256);

	/* Program the size of DFI into ILOAD_ATTR */
	csb_write(tegra, XUSB_CSB_MP_ILOAD_ATTR, fw_size);

	/* Boot code of the firmware reads the ILOAD_BASE_LO register
	 * to get to the start of the dfi in system memory.
	 */
	csb_write(tegra, XUSB_CSB_MP_ILOAD_BASE_LO, phys_addr_lo);

	/* Program the ILOAD_BASE_HI with a value of MSB 32 bits */
	csb_write(tegra, XUSB_CSB_MP_ILOAD_BASE_HI, phys_addr_hi);

	/* Set BOOTPATH to 1 in APMAP Register.Bit 31 is APMAP_BOOTMAP */
	HwReg = APMAP_BOOTPATH;
	csb_write(tegra, XUSB_CSB_MP_APMAP, HwReg);

	/* Invalidate l2imem. */
	HwReg = L2IMEM_INVALIDATE_ALL;
	csb_write(tegra, XUSB_CSB_MP_L2IMEMOP_TRIG, HwReg);

	/* Initiate Fetch of Bootcode from system memory into l2imem.
	 * a. Program BootCode location and size in system memory.
	 */

	HwReg = (((cfg_tbl->boot_codetag/imem_block_size)
			<< L2IMEMOP_SIZE_SRC_OFFSET_SHIFT)
			| ((cfg_tbl->boot_codesize/imem_block_size)
			<< L2IMEMOP_SIZE_SRC_COUNT_SHIFT)
			);

	csb_write(tegra, XUSB_CSB_MP_L2IMEMOP_SIZE, HwReg);

	/* b. Trigger Load operation. */
	HwReg = L2IMEM_LOAD_LOCKED_RESULT << L2IMEMOP_TRIG_LOAD_LOCKED_SHIFT;
	csb_write(tegra, XUSB_CSB_MP_L2IMEMOP_TRIG, HwReg);

	/* Program the size of autofill section */
	HwReg = cfg_tbl->boot_codesize/imem_block_size;
	csb_write(tegra, XUSB_FALC_IMFILLCTL, HwReg);


	/* Program TagLo/TagHi of autofill section. */
	HwReg = ((cfg_tbl->boot_codetag/imem_block_size)|
				(((cfg_tbl->boot_codetag+cfg_tbl->boot_codesize)
				/imem_block_size-1)<<IMFILLRNG1_TAG_HI_SHIFT));

	csb_write(tegra, XUSB_FALC_IMFILLRNG1, HwReg);

	/* Write 0x0 to DMACTL register. */
	csb_write(tegra, XUSB_FALC_DMACTL, 0x0);

	/* write to BOOTVEC register */
	csb_write(tegra, XUSB_FALC_BOOTVEC, boot_code_pc);

	/* Start Falcon CPU */
	csb_write(tegra, XUSB_FALC_CPUCTL, CPUCTL_STARTCPU);
	usleep_range(1000, 2000);

	dev_info(&pdev->dev,
		"load fw created at %ld-%02d-%02d %02d:%02d:%02d "\
		"UTC falcon state = %x\n", fw_tm.tm_year + 1900,
		fw_tm.tm_mon + 1, fw_tm.tm_mday, fw_tm.tm_hour,
		fw_tm.tm_min, fw_tm.tm_sec,
		csb_read(tegra, XUSB_FALC_CPUCTL));

	/* return fail if firmware status is not good */
	if (csb_read(tegra, XUSB_FALC_CPUCTL) == XUSB_FALC_STATE_HALTED)
		ret = -EFAULT;
	else
		ret = 0;

	return ret;
}

static void tegra_xhci_debug_read_pads(struct tegra_xhci_hcd *tegra)
{
	struct xhci_hcd *xhci = tegra->xhci;
	u32 reg;

	xhci_info(xhci, "============ PADCTL VALUES START =================\n");
	reg = readl(tegra->padctl_base + USB2_PAD_MUX_0);
	xhci_info(xhci, " PAD MUX = %x\n", reg);
	reg = readl(tegra->padctl_base + USB2_PORT_CAP_0);
	xhci_info(xhci, " PORT CAP = %x\n", reg);
	reg = readl(tegra->padctl_base + SNPS_OC_MAP_0);
	xhci_info(xhci, " SNPS OC MAP = %x\n", reg);
	reg = readl(tegra->padctl_base + USB2_OC_MAP_0);
	xhci_info(xhci, " USB2 OC MAP = %x\n", reg);
	reg = readl(tegra->padctl_base + SS_PORT_MAP_0);
	xhci_info(xhci, " SS PORT MAP = %x\n", reg);
	reg = readl(tegra->padctl_base + OC_DET_0);
	xhci_info(xhci, " OC DET 0= %x\n", reg);
	reg = readl(tegra->padctl_base + IOPHY_USB3_PAD0_CTL_2_0);
	xhci_info(xhci, " IOPHY_USB3_PAD0_CTL_2_0= %x\n", reg);
	reg = readl(tegra->padctl_base + IOPHY_USB3_PAD1_CTL_2_0);
	xhci_info(xhci, " IOPHY_USB3_PAD1_CTL_2_0= %x\n", reg);
	reg = readl(tegra->padctl_base + USB2_OTG_PAD0_CTL_0_0);
	xhci_info(xhci, " USB2_OTG_PAD0_CTL_0_0= %x\n", reg);
	reg = readl(tegra->padctl_base + USB2_OTG_PAD1_CTL_0_0);
	xhci_info(xhci, " USB2_OTG_PAD1_CTL_0_0= %x\n", reg);
	reg = readl(tegra->padctl_base + USB2_OTG_PAD0_CTL_1_0);
	xhci_info(xhci, " USB2_OTG_PAD0_CTL_1_0= %x\n", reg);
	reg = readl(tegra->padctl_base + USB2_OTG_PAD1_CTL_1_0);
	xhci_info(xhci, " USB2_OTG_PAD1_CTL_1_0= %x\n", reg);
	reg = readl(tegra->padctl_base + USB2_BIAS_PAD_CTL_0_0);
	xhci_info(xhci, " USB2_BIAS_PAD_CTL_0_0= %x\n", reg);
	reg = readl(tegra->padctl_base + HSIC_PAD0_CTL_0_0);
	xhci_info(xhci, " HSIC_PAD0_CTL_0_0= %x\n", reg);
	reg = readl(tegra->padctl_base + HSIC_PAD1_CTL_0_0);
	xhci_info(xhci, " HSIC_PAD1_CTL_0_0= %x\n", reg);
	xhci_info(xhci, "============ PADCTL VALUES END=================\n");
}

static void tegra_xhci_cfg(struct tegra_xhci_hcd *tegra)
{
	u32 reg;

	reg = readl(tegra->ipfs_base + IPFS_XUSB_HOST_CONFIGURATION_0);
	reg |= IPFS_EN_FPCI;
	writel(reg, tegra->ipfs_base + IPFS_XUSB_HOST_CONFIGURATION_0);
	udelay(10);

	/* Program Bar0 Space */
	reg = readl(tegra->fpci_base + XUSB_CFG_4);
	reg |= tegra->host_phy_base;
	writel(reg, tegra->fpci_base + XUSB_CFG_4);
	usleep_range(100, 200);

	/* Enable Bus Master */
	reg = readl(tegra->fpci_base + XUSB_CFG_1);
	reg |= 0x7;
	writel(reg, tegra->fpci_base + XUSB_CFG_1);

	/* Set intr mask to enable intr assertion */
	reg = readl(tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);
	reg |= IPFS_IP_INT_MASK;
	writel(reg, tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);

	/* Set hysteris to 0x80 */
	writel(0x80, tegra->ipfs_base + IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0);
}

static int tegra_xusb_regulator_init(struct tegra_xhci_hcd *tegra,
		struct platform_device *pdev)
{
	int err = 0;

	tegra->xusb_vbus_reg = devm_regulator_get(&pdev->dev, "usb_vbus");
	if (IS_ERR(tegra->xusb_vbus_reg)) {
		dev_err(&pdev->dev, "vbus regulator not found: %ld."
			, PTR_ERR(tegra->xusb_vbus_reg));
		err = PTR_ERR(tegra->xusb_vbus_reg);
		goto err_null_regulator;
	}
	err = regulator_enable(tegra->xusb_vbus_reg);
	if (err < 0) {
		dev_err(&pdev->dev, "vbus: regulator enable failed:%d\n", err);
		goto err_null_regulator;
	}

	tegra->xusb_avddio_usb3_reg =
			devm_regulator_get(&pdev->dev, "avddio_usb");
	if (IS_ERR(tegra->xusb_avddio_usb3_reg)) {
		dev_err(&pdev->dev, "avddio_usb3: regulator not found: %ld."
			, PTR_ERR(tegra->xusb_avddio_usb3_reg));
		err = PTR_ERR(tegra->xusb_avddio_usb3_reg);
		goto err_put_vbus;
	}
	err = regulator_enable(tegra->xusb_avddio_usb3_reg);
	if (err < 0) {
		dev_err(&pdev->dev,
			"avddio_usb3: regulator enable failed:%d\n", err);
		goto err_put_vbus;
	}

	tegra->xusb_vddio_hsic_reg =
			devm_regulator_get(&pdev->dev, "vddio_hsic");
	if (IS_ERR(tegra->xusb_vddio_hsic_reg)) {
		dev_err(&pdev->dev, "vddio_hsic: regulator not found: %ld."
			, PTR_ERR(tegra->xusb_vddio_hsic_reg));
		err = PTR_ERR(tegra->xusb_vddio_hsic_reg);
		goto err_put_avddio_usb3;
	}
	err = regulator_enable(tegra->xusb_vddio_hsic_reg);
	if (err < 0) {
		dev_err(&pdev->dev,
			"vddio_hsic: regulator enable failed:%d\n", err);
		goto err_put_avddio_usb3;
	}

	tegra->xusb_hvdd_usb3_reg =
			devm_regulator_get(&pdev->dev, "hvdd_usb");
	if (IS_ERR(tegra->xusb_vddio_hsic_reg)) {
		dev_dbg(&pdev->dev, "hvdd_usb: regulator not found: %ld."
			, PTR_ERR(tegra->xusb_hvdd_usb3_reg));
		err = PTR_ERR(tegra->xusb_hvdd_usb3_reg);
		goto err_put_vddio_hsic;
	}
	err = regulator_enable(tegra->xusb_hvdd_usb3_reg);
	if (err < 0) {
		dev_err(&pdev->dev,
			"hvdd_usb3: regulator enable failed:%d\n", err);
		goto err_put_vddio_hsic;
	}

	tegra->xusb_avdd_usb3_pll_reg =
		devm_regulator_get(&pdev->dev, "avdd_usb_pll");
	if (IS_ERR(tegra->xusb_avdd_usb3_pll_reg)) {
		dev_dbg(&pdev->dev, "regulator not found: %ld."
			, PTR_ERR(tegra->xusb_avdd_usb3_pll_reg));
		err = PTR_ERR(tegra->xusb_avdd_usb3_pll_reg);
		goto err_put_hvdd_usb3;
	}
	err = regulator_enable(tegra->xusb_avdd_usb3_pll_reg);
	if (err < 0) {
		dev_err(&pdev->dev,
			"avdd_usb3_pll: regulator enable failed:%d\n", err);
		goto err_put_hvdd_usb3;
	}

	return err;

err_put_hvdd_usb3:
	regulator_disable(tegra->xusb_hvdd_usb3_reg);
err_put_vddio_hsic:
	regulator_disable(tegra->xusb_vddio_hsic_reg);
err_put_avddio_usb3:
	regulator_disable(tegra->xusb_avddio_usb3_reg);
err_put_vbus:
	regulator_disable(tegra->xusb_vbus_reg);
err_null_regulator:
	tegra->xusb_vbus_reg = NULL;
	tegra->xusb_avddio_usb3_reg = NULL;
	tegra->xusb_vddio_hsic_reg = NULL;
	tegra->xusb_hvdd_usb3_reg = NULL;
	tegra->xusb_avdd_usb3_pll_reg = NULL;
	return err;
}

static void tegra_xusb_regulator_deinit(struct tegra_xhci_hcd *tegra)
{
	regulator_disable(tegra->xusb_avdd_usb3_pll_reg);
	regulator_disable(tegra->xusb_hvdd_usb3_reg);
	regulator_disable(tegra->xusb_vddio_hsic_reg);
	regulator_disable(tegra->xusb_vbus_reg);

	tegra->xusb_vbus_reg = NULL;
	tegra->xusb_vddio_hsic_reg = NULL;
	tegra->xusb_hvdd_usb3_reg = NULL;
	tegra->xusb_avdd_usb3_pll_reg = NULL;
}

static int tegra_usb2_clocks_init(struct tegra_xhci_hcd *tegra)
{
	struct platform_device *pdev = tegra->pdev;
	int err = 0;

	/* get clock handle for pllu clock. */
	tegra->pllu_clk = devm_clk_get(&pdev->dev, "pll_u");
	if (IS_ERR(tegra->pllu_clk)) {
		dev_err(&pdev->dev, "%s: Failed to get pllu clock\n", __func__);
		err = PTR_ERR(tegra->pllu_clk);
		goto err_clk;
	}

	/* enable pllu clock */
	err = clk_enable(tegra->pllu_clk);
	if (err) {
		dev_err(&pdev->dev, "%s: could not enable pllu clock\n",
			__func__);
		goto err_clk;
	}

	tegra->utmip_clk = devm_clk_get(&pdev->dev, "utmip-pad");
	if (IS_ERR(tegra->utmip_clk)) {
		dev_err(&pdev->dev, "%s: can't get utmip clock\n", __func__);
		err = PTR_ERR(tegra->utmip_clk);
		goto err_disable_pllu;
	}
	/* enable utmip clock */
	err = clk_enable(tegra->utmip_clk);
	if (err) {
		dev_err(&pdev->dev, "%s: could not enable utmip clock\n",
			__func__);
		goto err_disable_pllu;
	}

	tegra->plle_re_vco_clk = devm_clk_get(&pdev->dev, "pll_re_vco");
	if (IS_ERR(tegra->plle_re_vco_clk)) {
		dev_err(&pdev->dev, "%s: Failed to get plle_re_vco_clk\n",
			__func__);
		goto err_disable_pll_re_vco;
	}
	err = clk_enable(tegra->plle_re_vco_clk);
	if (err) {
		dev_err(&pdev->dev, "%s: could not enable plle_re_vco_clk\n",
			__func__);
		goto err_disable_pll_re_vco;
	}

	/* Enable PLLE as well */
	tegra->plle_clk = devm_clk_get(&pdev->dev, "pll_e");
	if (IS_ERR(tegra->plle_clk)) {
		dev_err(&pdev->dev, "%s: Failed to get plle clock\n", __func__);
		err = PTR_ERR(tegra->plle_clk);
		goto err_disable_utmi_pad;
	}
	/* enable plle clock */
	err = clk_enable(tegra->plle_clk);
	if (err) {
		dev_err(&pdev->dev, "%s: could not enable plle clock\n",
			__func__);
		goto err_disable_utmi_pad;
	}

	return err;

err_disable_utmi_pad:
	clk_disable(tegra->utmip_clk);
err_disable_pll_re_vco:
	clk_disable(tegra->plle_re_vco_clk);
err_disable_pllu:
	clk_disable(tegra->pllu_clk);
err_clk:
	return err;
}

static void tegra_usb2_clocks_deinit(struct tegra_xhci_hcd *tegra)
{
	clk_disable(tegra->plle_clk);
	clk_disable(tegra->utmip_clk);
	clk_disable(tegra->plle_re_vco_clk);
	clk_disable(tegra->pllu_clk);
	tegra->plle_clk = NULL;
	tegra->utmip_clk = NULL;
	tegra->plle_re_vco_clk = NULL;
	tegra->pllu_clk = NULL;
}

static int tegra_xusb_partitions_clk_init(struct tegra_xhci_hcd *tegra)
{
	struct platform_device *pdev = tegra->pdev;
	int err = 0;

	/* get the clock handle of 120MHz clock source */
	tegra->pll_u_480M = devm_clk_get(&pdev->dev, "pll_u_480M");
	if (IS_ERR(tegra->pll_u_480M)) {
		dev_err(&pdev->dev, "Failed to get pll_u_480M clk handle\n");
		return PTR_ERR(tegra->pll_u_480M);
	}

	/* get the clock handle of 12MHz clock source */
	tegra->clk_m = devm_clk_get(&pdev->dev, "clk_m");
	if (IS_ERR(tegra->clk_m)) {
		dev_err(&pdev->dev, "Failed to get clk_m clk handle\n");
		err = PTR_ERR(tegra->clk_m);
		goto clk_get_clk_m_failed;
	}

	tegra->ss_clk = devm_clk_get(&pdev->dev, "ss_src");
	if (IS_ERR(tegra->ss_clk)) {
		dev_err(&pdev->dev, "Failed to get ss partition clk\n");
		err = PTR_ERR(tegra->ss_clk);
		tegra->ss_clk = NULL;
		goto get_ss_clk_failed;
	}

	tegra->host_partition_clk = devm_clk_get(&pdev->dev, "host");
	if (IS_ERR(tegra->host_partition_clk)) {
		dev_err(&pdev->dev, "Failed to get host partition clk\n");
		err = PTR_ERR(tegra->host_partition_clk);
		tegra->host_partition_clk = NULL;
		goto get_host_partition_clk_failed;
	}

	tegra->ss_partition_clk = devm_clk_get(&pdev->dev, "ss");
	if (IS_ERR(tegra->ss_partition_clk)) {
		dev_err(&pdev->dev, "Failed to get ss partition clk\n");
		err = PTR_ERR(tegra->ss_partition_clk);
		tegra->ss_partition_clk = NULL;
		goto get_ss_partition_clk_failed;
	}

	tegra->dev_partition_clk = devm_clk_get(&pdev->dev, "dev");
	if (IS_ERR(tegra->dev_partition_clk)) {
		dev_err(&pdev->dev, "Failed to get dev partition clk\n");
		err = PTR_ERR(tegra->dev_partition_clk);
		tegra->dev_partition_clk = NULL;
		goto get_dev_partition_clk_failed;
	}

	/* enable ss clock */
	err = clk_enable(tegra->ss_clk);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable ss partition clk\n");
		goto enable_ss_clk_failed;
	}

	err = clk_enable(tegra->host_partition_clk);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable host partition clk\n");
		goto enable_host_partition_clk_failed;
	}

	err = clk_enable(tegra->ss_partition_clk);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable ss partition clk\n");
		goto eanble_ss_partition_clk_failed;
	}

	err = clk_enable(tegra->dev_partition_clk);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable dev partition clk\n");
		goto enable_dev_partition_clk_failed;
	}

	return 0;

enable_dev_partition_clk_failed:
	clk_disable(tegra->ss_partition_clk);

eanble_ss_partition_clk_failed:
	clk_disable(tegra->host_partition_clk);

enable_host_partition_clk_failed:
	clk_disable(tegra->ss_clk);

enable_ss_clk_failed:
	tegra->dev_partition_clk = NULL;

get_dev_partition_clk_failed:
	tegra->ss_partition_clk = NULL;

get_ss_partition_clk_failed:
	tegra->host_partition_clk = NULL;

get_host_partition_clk_failed:
	tegra->ss_clk = NULL;

get_ss_clk_failed:
	tegra->clk_m = NULL;

clk_get_clk_m_failed:
	tegra->pll_u_480M = NULL;

	return err;
}

static void tegra_xusb_partitions_clk_deinit(struct tegra_xhci_hcd *tegra)
{
	clk_disable(tegra->dev_partition_clk);
	clk_disable(tegra->ss_partition_clk);
	clk_disable(tegra->host_partition_clk);
	clk_disable(tegra->ss_clk);
	tegra->dev_partition_clk = NULL;
	tegra->ss_partition_clk = NULL;
	tegra->host_partition_clk = NULL;
	tegra->ss_clk = NULL;
	tegra->clk_m = NULL;
	tegra->pll_u_480M = NULL;
}

/* Enable ss clk, host clk, falcon clk,
 * fs clk, dev clk, plle and refplle
 */

static int
tegra_xusb_request_clk_rate(struct tegra_xhci_hcd *tegra,
		struct clk *clk_handle, u32 rate, u32 *sw_resp)
{
	int ret = 0;
	enum MBOX_CMD_TYPE cmd_ack = MBOX_CMD_ACK;
	int fw_req_rate = rate, cur_rate;

	/* frequency request from firmware is in KHz.
	 * Convert it to MHz
	 */

	/* get current rate of clock */
	cur_rate = clk_get_rate(clk_handle);
	cur_rate /= 1000;

	if (fw_req_rate == cur_rate) {
		cmd_ack = MBOX_CMD_ACK;
		*sw_resp = fw_req_rate;
	} else {

		if (clk_handle == tegra->ss_clk && fw_req_rate == 12000) {
			/* Change SS clock source to CLK_M at 12MHz */
			clk_set_parent(clk_handle, tegra->clk_m);
			clk_set_rate(clk_handle, fw_req_rate * 1000);
		} else if (clk_handle == tegra->ss_clk &&
				fw_req_rate == 120000) {
			/* Change SS clock source to HSIC_480 at 120MHz */
			clk_set_rate(clk_handle,  3000 * 1000);
			clk_set_parent(clk_handle, tegra->pll_u_480M);
		}

		*sw_resp = clk_get_rate(clk_handle);
		*sw_resp /= 1000;

		if (*sw_resp != fw_req_rate) {
			xhci_err(tegra->xhci, "cur_rate=%d, fw_req_rate=%d\n",
				cur_rate, fw_req_rate);
			cmd_ack = MBOX_CMD_NACK;
		}
	}
	*sw_resp |= (cmd_ack << MBOX_CMD_SHIFT);
	return ret;
}

static void
tegra_xhci_ss_wake_on_interrupts(struct tegra_xhci_hcd *tegra, bool enable)
{
	u32 elpg_program0;

	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);
	elpg_program0 |= (SS_PORT0_WAKEUP_EVENT | SS_PORT1_WAKEUP_EVENT);

	writel(elpg_program0, tegra->padctl_base + ELPG_PROGRAM_0);

	/* Enable ss wake interrupts */
	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);

	if (enable) {
		/* enable interrupts */
		elpg_program0 |= (SS_PORT0_WAKE_INTERRUPT_ENABLE |
				SS_PORT1_WAKE_INTERRUPT_ENABLE);
	} else {
		/* disable interrupts */
		elpg_program0 &= ~(SS_PORT0_WAKE_INTERRUPT_ENABLE |
				SS_PORT1_WAKE_INTERRUPT_ENABLE);
	}
	writel(elpg_program0, tegra->padctl_base + ELPG_PROGRAM_0);
}

static void
tegra_xhci_hs_wake_on_interrupts(struct tegra_xhci_hcd *tegra, bool enable)
{
	u32 elpg_program0;

	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);
	elpg_program0 |= (USB2_PORT0_WAKEUP_EVENT | USB2_PORT1_WAKEUP_EVENT |
			USB2_HSIC_PORT0_WAKEUP_EVENT |
			USB2_HSIC_PORT1_WAKEUP_EVENT);
	writel(elpg_program0, tegra->padctl_base + ELPG_PROGRAM_0);

	/* Enable the wake interrupts */
	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);
	if (enable)
		/* enable interrupts */
		elpg_program0 |= (USB2_PORT0_WAKE_INTERRUPT_ENABLE |
				USB2_PORT1_WAKE_INTERRUPT_ENABLE |
				USB2_HSIC_PORT0_WAKE_INTERRUPT_ENABLE |
				USB2_HSIC_PORT1_WAKE_INTERRUPT_ENABLE);
	else
		elpg_program0 &= ~(USB2_PORT0_WAKE_INTERRUPT_ENABLE |
				USB2_PORT1_WAKE_INTERRUPT_ENABLE |
				USB2_HSIC_PORT0_WAKE_INTERRUPT_ENABLE |
				USB2_HSIC_PORT1_WAKE_INTERRUPT_ENABLE);
	writel(elpg_program0, tegra->padctl_base + ELPG_PROGRAM_0);
}

static void
tegra_xhci_ss_wake_signal(struct tegra_xhci_hcd *tegra, bool enable)
{
	u32 elpg_program0;

	/* DO NOT COMBINE BELOW 2 WRITES */

	/* Assert/Deassert clamp_en_early signals to SSP0/1 */
	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);
	if (enable)
		elpg_program0 |= (SSP0_ELPG_CLAMP_EN_EARLY |
				SSP1_ELPG_CLAMP_EN_EARLY);
	else
		elpg_program0 &= ~(SSP0_ELPG_CLAMP_EN_EARLY |
				SSP1_ELPG_CLAMP_EN_EARLY);
	writel(elpg_program0, tegra->padctl_base + ELPG_PROGRAM_0);

	/*
	 * Check the LP0 figure and leave gap bw writes to
	 * clamp_en_early and clamp_en
	 */
	usleep_range(100, 200);

	/* Assert/Deassert clam_en signal */
	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);

	if (enable)
		elpg_program0 |= (SSP0_ELPG_CLAMP_EN | SSP1_ELPG_CLAMP_EN);
	else
		elpg_program0 &= ~(SSP0_ELPG_CLAMP_EN | SSP1_ELPG_CLAMP_EN);

	writel(elpg_program0, tegra->padctl_base + ELPG_PROGRAM_0);

	/* wait for 250us for the writes to propogate */
	if (enable)
		usleep_range(250, 300);
}

static void
tegra_xhci_ss_vcore(struct tegra_xhci_hcd *tegra, bool enable)
{
	u32 elpg_program0;


	/* Assert vcore_off signal */
	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);

	if (enable)
		elpg_program0 |= (SSP0_ELPG_VCORE_DOWN|SSP1_ELPG_VCORE_DOWN);
	else
		elpg_program0 &= ~(SSP0_ELPG_VCORE_DOWN|SSP1_ELPG_VCORE_DOWN);

	writel(elpg_program0, tegra->padctl_base + ELPG_PROGRAM_0);
}

static void
tegra_xhci_padctl_enable_usb_vbus(struct tegra_xhci_hcd *tegra)
{
	u32 reg;
	struct tegra_xusb_pad_data *xusb_padctl = tegra->xusb_padctl;
	struct xhci_hcd *xhci = tegra->xhci;

	/* Program the following XUSB PADCTL registers to
	 * 0x7 to disable the over current signal mapping
	 * for USB 2.0 ports owned by XUSB and USB2:
	 */
	reg = readl(tegra->padctl_base + SNPS_OC_MAP_0);
	reg |= xusb_padctl->snps_oc_map;
	writel(reg, tegra->padctl_base + SNPS_OC_MAP_0);
	reg = readl(tegra->padctl_base + SNPS_OC_MAP_0);

	reg = readl(tegra->padctl_base + OC_DET_0);
	reg = xusb_padctl->oc_det;
	writel(reg, tegra->padctl_base + OC_DET_0);

	/* check if over current seen. Clear if present */
	reg = readl(tegra->padctl_base + OC_DET_0);
	if (reg & (0x3 << 20)) {
		xhci_info(xhci, "Over current detected. Clearing...\n");
		writel(reg, tegra->padctl_base + OC_DET_0);

		usleep_range(100, 200);

		reg = readl(tegra->padctl_base + OC_DET_0);
		if (reg & (0x3 << 20))
			xhci_info(xhci, "Over current still present\n");
	}

	reg = readl(tegra->padctl_base + USB2_OC_MAP_0);
	reg |= xusb_padctl->usb2_oc_map;
	writel(reg, tegra->padctl_base + USB2_OC_MAP_0);
}

/* This function assigns the USB ports to the controllers,
 * then programs the port capabilities and pad parameters
 * of ports assigned to XUSB after booted to OS.
 */
void
tegra_xhci_padctl_portmap_and_caps(struct tegra_xhci_hcd *tegra)
{
	u32 reg;
	struct tegra_xusb_pad_data *xusb_padctl = tegra->xusb_padctl;

	/* Below line should be removed when we have clarity on why
	 * FUSE setting read 0x10 for hs_curr_level causing
	 * SS/HS/FS hub re-enumeration.
	 */
	xusb_padctl->hs_curr_level = 0x30;

	/* Program the following XUSB PADCTL registers to assign
	 * the USB2.0 ports to XUSB or USB2, according to the platform
	 * specific configuration
	 */
	reg = readl(tegra->padctl_base + USB2_PAD_MUX_0);
	reg |= xusb_padctl->pad_mux;
	writel(reg, tegra->padctl_base + USB2_PAD_MUX_0);

	/* Program the following XUSB PADCTL registers to assign
	 * the port capabilities for USB2.0 ports owned by XUSB,
	 * according to the platform specific configuration:
	 */
	reg = readl(tegra->padctl_base + USB2_PORT_CAP_0);
	reg |= xusb_padctl->port_cap;
	writel(reg, tegra->padctl_base + USB2_PORT_CAP_0);



	/* Program the following XUSB PADCTL registers to assign
	 * the SuperSpeed port mapping to USB2.0 ports owned by XUSB,
	 * where the SuperSpeed ports inherit their port capabilities
	 * from the USB2.0 ports they mapped to, according to the
	 * platform specific configuration
	 */
	reg = readl(tegra->padctl_base + SS_PORT_MAP_0);
	reg |= xusb_padctl->ss_port_map;
	writel(reg, tegra->padctl_base + SS_PORT_MAP_0);

	reg = readl(tegra->padctl_base + IOPHY_USB3_PAD0_CTL_2_0);
	reg &= ~0xfffffff0; /* clear needed field */
	reg |= xusb_padctl->rx_wander | xusb_padctl->rx_eq |
			xusb_padctl->cdr_cntl;
	writel(reg, tegra->padctl_base + IOPHY_USB3_PAD0_CTL_2_0);

	reg = readl(tegra->padctl_base + IOPHY_USB3_PAD1_CTL_2_0);
	reg &= ~0xfffffff0; /* clear needed field */
	reg |= xusb_padctl->rx_wander | xusb_padctl->rx_eq |
			xusb_padctl->cdr_cntl;
	writel(reg, tegra->padctl_base + IOPHY_USB3_PAD1_CTL_2_0);

	reg = readl(tegra->padctl_base + IOPHY_USB3_PAD0_CTL_4_0);
	reg = xusb_padctl->dfe_cntl;
	writel(reg, tegra->padctl_base + IOPHY_USB3_PAD0_CTL_4_0);

	reg = readl(tegra->padctl_base + IOPHY_USB3_PAD1_CTL_4_0);
	reg = xusb_padctl->dfe_cntl;
	writel(reg, tegra->padctl_base + IOPHY_USB3_PAD1_CTL_4_0);

	reg = readl(tegra->padctl_base + USB2_OTG_PAD0_CTL_0_0);
	reg &= xusb_padctl->otg_pad0_ctl0;
	reg |= xusb_padctl->hs_slew;
	reg &= ~(0x3f << 0);
	reg |= xusb_padctl->hs_curr_level;
	writel(reg, tegra->padctl_base + USB2_OTG_PAD0_CTL_0_0);

	reg = readl(tegra->padctl_base + USB2_OTG_PAD1_CTL_0_0);
	reg &= xusb_padctl->otg_pad1_ctl0;
	reg |= xusb_padctl->hs_slew;
	reg &= ~(0x3f << 0);
	reg |= xusb_padctl->hs_curr_level;
	writel(reg, tegra->padctl_base + USB2_OTG_PAD1_CTL_0_0);

	reg = readl(tegra->padctl_base + USB2_OTG_PAD0_CTL_1_0);
	reg ^= xusb_padctl->otg_pad0_ctl1;
	reg &= ~((0x3 << 9) | (0xf << 3));
	reg |= (xusb_padctl->hs_iref_cap << 9) |
					(xusb_padctl->hs_term_range_adj << 3);
	writel(reg, tegra->padctl_base + USB2_OTG_PAD0_CTL_1_0);

	reg = readl(tegra->padctl_base + USB2_OTG_PAD1_CTL_1_0);
	reg ^= xusb_padctl->otg_pad1_ctl1;
	reg &= ~((0x3 << 9) | (0xf << 3));
	reg |= (xusb_padctl->hs_iref_cap << 9) |
					(xusb_padctl->hs_term_range_adj << 3);
	writel(reg, tegra->padctl_base + USB2_OTG_PAD1_CTL_1_0);

	reg = readl(tegra->padctl_base + USB2_BIAS_PAD_CTL_0_0);
	reg &= xusb_padctl->bias_pad_ctl0;
	reg |= HS_DISCON_LEVEL(5);
	reg &= ~(0x3 << 0);
	reg |= xusb_padctl->hs_squelch_level;
	writel(reg, tegra->padctl_base + USB2_BIAS_PAD_CTL_0_0);

	reg = readl(tegra->padctl_base + HSIC_PAD0_CTL_0_0);
	reg &= xusb_padctl->hsic_pad0_ctl0;
	writel(reg, tegra->padctl_base + HSIC_PAD0_CTL_0_0);

	reg = readl(tegra->padctl_base + HSIC_PAD1_CTL_0_0);
	reg &= xusb_padctl->hsic_pad0_ctl1;
	writel(reg, tegra->padctl_base + HSIC_PAD1_CTL_0_0);
}

/* This function read XUSB registers and stores in device context */
static void
tegra_xhci_save_xusb_ctx(struct tegra_xhci_hcd *tegra)
{

	/* a. Save the IPFS registers */
	tegra->sregs.msi_bar_sz =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_BAR_SZ_0);

	tegra->sregs.msi_axi_barst =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_AXI_BAR_ST_0);

	tegra->sregs.msi_fpci_barst =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_BAR_ST_0);

	tegra->sregs.msi_vec0 =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_VEC0_0);

	tegra->sregs.msi_en_vec0 =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MSI_EN_VEC0_0);

	tegra->sregs.fpci_error_masks =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_ERROR_MASKS_0);

	tegra->sregs.intr_mask =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);

	tegra->sregs.ipfs_intr_enable =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_IPFS_INTR_ENABLE_0);

	tegra->sregs.ufpci_config =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_UFPCI_CONFIG_0);

	tegra->sregs.clkgate_hysteresis =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0);

	tegra->sregs.xusb_host_mccif_fifo_cntrl =
		readl(tegra->ipfs_base + IPFS_XUSB_HOST_MCCIF_FIFOCTRL_0);

	/* b. Save the CFG registers */

	tegra->sregs.hs_pls =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HS_PLS);

	tegra->sregs.fs_pls =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_FS_PLS);

	tegra->sregs.hs_fs_speed =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_SPEED);

	tegra->sregs.hs_fs_pp =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_PP);

	tegra->sregs.cfg_aru =
		readl(tegra->fpci_base + XUSB_CFG_ARU_CONTEXT);

	tegra->sregs.cfg_order =
		readl(tegra->fpci_base + XUSB_CFG_FPCICFG);

	tegra->sregs.cfg_fladj =
		readl(tegra->fpci_base + XUSB_CFG_24);

	tegra->sregs.cfg_sid =
		readl(tegra->fpci_base + XUSB_CFG_16);
}

/* This function restores XUSB registers from device context */
static void
tegra_xhci_restore_ctx(struct tegra_xhci_hcd *tegra)
{
	/* Restore Cfg registers */
	writel(tegra->sregs.hs_pls,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HS_PLS);

	writel(tegra->sregs.fs_pls,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_FS_PLS);

	writel(tegra->sregs.hs_fs_speed,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_SPEED);

	writel(tegra->sregs.hs_fs_pp,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT_HSFS_PP);

	writel(tegra->sregs.cfg_aru,
		tegra->fpci_base + XUSB_CFG_ARU_CONTEXT);

	writel(tegra->sregs.cfg_order,
		tegra->fpci_base + XUSB_CFG_FPCICFG);

	writel(tegra->sregs.cfg_fladj,
		tegra->fpci_base + XUSB_CFG_24);

	writel(tegra->sregs.cfg_sid,
		tegra->fpci_base + XUSB_CFG_16);

	/* Restore IPFS registers */

	writel(tegra->sregs.msi_bar_sz,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_BAR_SZ_0);

	writel(tegra->sregs.msi_axi_barst,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_AXI_BAR_ST_0);

	writel(tegra->sregs.msi_fpci_barst,
		tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_BAR_ST_0);

	writel(tegra->sregs.msi_vec0,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_VEC0_0);

	writel(tegra->sregs.msi_en_vec0,
		tegra->ipfs_base + IPFS_XUSB_HOST_MSI_EN_VEC0_0);

	writel(tegra->sregs.fpci_error_masks,
		tegra->ipfs_base + IPFS_XUSB_HOST_FPCI_ERROR_MASKS_0);

	writel(tegra->sregs.intr_mask,
		tegra->ipfs_base + IPFS_XUSB_HOST_INTR_MASK_0);

	writel(tegra->sregs.ipfs_intr_enable,
		tegra->ipfs_base + IPFS_XUSB_HOST_IPFS_INTR_ENABLE_0);

	writel(tegra->sregs.ufpci_config,
		tegra->fpci_base + IPFS_XUSB_HOST_UFPCI_CONFIG_0);

	writel(tegra->sregs.clkgate_hysteresis,
		tegra->ipfs_base + IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0);

	writel(tegra->sregs.xusb_host_mccif_fifo_cntrl,
		tegra->ipfs_base + IPFS_XUSB_HOST_MCCIF_FIFOCTRL_0);
}

static int
tegra_xhci_load_fw_from_pmc(struct tegra_xhci_hcd *tegra, bool reset)
{
	u32 cnr, count = 0xff;
	struct platform_device *pdev = tegra->pdev;

	if (tegra_xhci_load_firmware(tegra, reset)) {
		dev_err(&pdev->dev, "Firmware load unsuccessful\n");
		return -ENODEV;
	}

	/* wait for CNR to get set */
	do {
		cnr = readl(IO_ADDRESS(tegra->host_phy_base + 0x24));
	} while ((cnr & 0x800) && count--);

	if (!count && !(cnr & 0x800))
		dev_err(&pdev->dev, "CNR not set. Nothing would work now\n");

	return 0;
}

/* SS ELPG Entry initiated by fw */
static int tegra_xhci_ss_elpg_entry(struct tegra_xhci_hcd *tegra)
{
	struct xhci_hcd *xhci = tegra->xhci;
	u32 ret = 0;

	must_have_sync_lock(tegra);

	/* This is SS partition ELPG entry
	 * STEP 0: firmware will set WOC WOD bits in PVTPORTSC2 regs.
	 */

	/* Step 0: Acquire mbox and send PWRGATE msg to firmware
	 * only if it is sw initiated one
	 */

	/* STEP 1: xHCI firmware and xHCIPEP driver communicates
	 * SuperSpeed partition ELPG entry via mailbox protocol
	 */

	/* STEP 2: xHCI PEP driver and XUSB device mode driver
	 * enable the XUSB wakeup interrupts for the SuperSpeed
	 * and USB2.0 ports assigned to host.Section 4.1 Step 3
	 */
	tegra_xhci_ss_wake_on_interrupts(tegra, true);

	/* STEP 3: xHCI PEP driver initiates the signal sequence
	 * to enable the XUSB SSwake detection logic for the
	 * SuperSpeed ports assigned to host.Section 4.1 Step 4
	 */
	tegra_xhci_ss_wake_signal(tegra, true);

	/* STEP 4: System Power Management driver asserts reset
	 * to XUSB SuperSpeed partition then disables its clocks
	 */
	tegra_periph_reset_assert(tegra->ss_partition_clk);
	clk_disable(tegra->ss_partition_clk);

	usleep_range(100, 200);

	/* STEP 5: System Power Management driver disables the
	 * XUSB SuperSpeed partition power rails.
	 */
	debug_print_portsc(xhci);

	/* tegra_powergate_partition also does partition reset assert */
	ret = tegra_powergate_partition(TEGRA_POWERGATE_XUSBA);
	if (ret) {
		xhci_err(xhci, "%s: could not powergate xusba partition\n",
				__func__);
		/* TODO: error recovery? */
	}
	tegra->ss_pwr_gated = true;

	/* STEP 6: xHCI PEP driver initiates the signal sequence
	 * to enable the XUSB SSwake detection logic for the
	 * SuperSpeed ports assigned to host.Section 4.1 Step 7
	 */
	tegra_xhci_ss_vcore(tegra, true);

	return ret;
}

/* Host ELPG Entry */
static int tegra_xhci_host_elpg_entry(struct tegra_xhci_hcd *tegra)
{
	struct xhci_hcd *xhci = tegra->xhci;
	u32 ret;
	u32 portsc;

	must_have_sync_lock(tegra);

	/* If ss is already powergated skip ss ctx save stuff */
	if (tegra->ss_pwr_gated) {
		xhci_info(xhci, "%s: SS partition is already powergated\n",
			__func__);
	} else {
		ret = tegra_xhci_ss_elpg_entry(tegra);
		if (ret) {
			xhci_err(xhci, "%s: ss_elpg_entry failed %d\n",
				__func__, ret);
			return ret;
		}
	}

	/* 1. IS INTR PENDING INT_PENDING=1 ? */

	/* STEP 1.1: Do a context save of XUSB and IPFS registers */
	tegra_xhci_save_xusb_ctx(tegra);

	tegra_xhci_pmc_usb2_wakenotif_init(tegra, PMC_PORT_UTMIP_P2, &portsc);
	tegra_xhci_hs_wake_on_interrupts(tegra, true);
	xhci_dbg(xhci, "%s: PMC_UTMIP_UHSIC_SLEEP_CFG_0 = %x\n", __func__,
		readl(tegra->pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0));

	/* STEP 4: Assert reset to host clk and disable host clk */
	tegra_periph_reset_assert(tegra->host_partition_clk);

	clk_disable(tegra->host_partition_clk);

	/* wait 150us */
	usleep_range(150, 200);

	/* STEP 4: Powergate host partition */
	/* tegra_powergate_partition also does partition reset assert */
	ret = tegra_powergate_partition(TEGRA_POWERGATE_XUSBC);
	if (ret) {
		xhci_err(xhci, "%s: could not unpowergate xusbc partition %d\n",
			__func__, ret);
		/* TODO: error handling? */
		return ret;
	}
	tegra->host_pwr_gated = true;

	tegra_xhci_pmc_usb2_wakenotif_on(tegra, PMC_PORT_UTMIP_P2, portsc);
	xhci_dbg(xhci, "%s: PMC_UTMIP_UHSIC_SLEEP_CFG_0 = %x\n", __func__,
		readl(tegra->pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0));

	xhci_info(xhci, "%s: elpg_entry: completed\n", __func__);
	xhci_dbg(xhci, "%s: HOST POWER STATUS = %d\n",
		__func__, tegra_powergate_is_powered(TEGRA_POWERGATE_XUSBC));
	return ret;
}

/* SS ELPG Exit triggered by PADCTL irq */
/**
 * tegra_xhci_ss_partition_elpg_exit - bring XUSBA partition out from elpg
 *
 * This function must be called with tegra->sync_lock acquired.
 *
 * @tegra: xhci controller context
 * @return 0 for success, or error numbers
 */
static int tegra_xhci_ss_partition_elpg_exit(struct tegra_xhci_hcd *tegra)
{
	struct xhci_hcd *xhci = tegra->xhci;
	int ret = 0;

	must_have_sync_lock(tegra);

	if (tegra->ss_pwr_gated && (tegra->ss_wake_event ||
			tegra->hs_wake_event || tegra->host_resume_req)) {

		/*
		 * PWR_UNGATE SS partition. XUSBA
		 * tegra_unpowergate_partition also does partition reset
		 * deassert
		 */
		ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBA);
		if (ret) {
			xhci_err(xhci,
			"%s: could not unpowergate xusba partition %d\n",
			__func__, ret);
			goto out;
		}
		if (tegra->ss_wake_event)
			tegra->ss_wake_event = false;

	} else {
		xhci_info(xhci, "%s: ss already power gated\n",
			__func__);
		return ret;
	}

	/* Step 3: Enable clock to ss partition */
	clk_enable(tegra->ss_partition_clk);

	/* Step 4: Disable ss wake detection logic */
	tegra_xhci_ss_wake_on_interrupts(tegra, false);

	/* Step 4.1: Disable ss wake detection logic */
	tegra_xhci_ss_vcore(tegra, false);

	/* wait 150us */
	usleep_range(150, 200);

	/* Step 4.2: Disable ss wake detection logic */
	tegra_xhci_ss_wake_signal(tegra, false);

	/* Step 6 Deassert reset for ss clks */
	tegra_periph_reset_deassert(tegra->ss_partition_clk);

	xhci_dbg(xhci, "%s: SS ELPG EXIT. ALL DONE\n", __func__);
	tegra->ss_pwr_gated = false;
out:
	return ret;
}

static void ss_partition_elpg_exit_work(struct work_struct *work)
{
	struct tegra_xhci_hcd *tegra = container_of(work, struct tegra_xhci_hcd,
		ss_elpg_exit_work);

	mutex_lock(&tegra->sync_lock);
	tegra_xhci_ss_partition_elpg_exit(tegra);
	mutex_unlock(&tegra->sync_lock);
}

/* Host ELPG Exit triggered by PADCTL irq */
/**
 * tegra_xhci_host_partition_elpg_exit - bring XUSBC partition out from elpg
 *
 * This function must be called with tegra->sync_lock acquired.
 *
 * @tegra: xhci controller context
 * @return 0 for success, or error numbers
 */
static int
tegra_xhci_host_partition_elpg_exit(struct tegra_xhci_hcd *tegra)
{
	struct xhci_hcd *xhci = tegra->xhci;
	int ret = 0;

	must_have_sync_lock(tegra);

	if (!tegra->hc_in_elpg)
		return 0;

	/* Step 2: Enable clock to host partition */
	clk_enable(tegra->host_partition_clk);

	if (tegra->lp0_exit) {
		u32 reg;

		/* check if over current seen. Clear if present */
		reg = readl(tegra->padctl_base + OC_DET_0);
		xhci_dbg(xhci, "%s: OC_DET_0=0x%x\n", __func__, reg);
		if (reg & (0x3 << 20)) {
			xhci_info(xhci, "Over current detected. Clearing...\n");
			writel(reg, tegra->padctl_base + OC_DET_0);

			usleep_range(100, 200);

			reg = readl(tegra->padctl_base + OC_DET_0);
			if (reg & (0x3 << 20))
				xhci_info(xhci, "Over current still present\n");
		}
		tegra_xhci_padctl_portmap_and_caps(tegra);
	}

	/* release clamps post deassert */
	if (tegra->lp0_exit) {
		tegra_xhci_padctl_enable_usb_vbus(tegra);
		tegra->lp0_exit = false;
	}

	/*
	 * PWR_UNGATE Host partition. XUSBC
	 * tegra_unpowergate_partition also does partition reset deassert
	 */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBC);
	if (ret) {
		xhci_err(xhci, "%s: could not unpowergate xusbc partition %d\n",
			__func__, ret);
		goto out;
	}

	/* Step 4: Deassert reset to host partition clk */
	tegra_periph_reset_deassert(tegra->host_partition_clk);

	/* Step 6.1: IPFS and XUSB BAR initialization */
	tegra_xhci_cfg(tegra);

	/* Step 6.2: IPFS and XUSB related restore */
	tegra_xhci_restore_ctx(tegra);

	/* Step 8: xhci spec related ctx restore
	 * will be done in xhci_resume().Do it here.
	 */

	tegra_xhci_ss_partition_elpg_exit(tegra);

	/* Load firmware */
	xhci_dbg(xhci, "%s: elpg_exit: loading firmware from pmc.\n"
			"ss (p1=0x%x, p2=0x%x, p3=0x%x), "
			"hs (p1=0x%x, p2=0x%x, p3=0x%x),\n"
			"fs (p1=0x%x, p2=0x%x, p3=0x%x)\n",
			__func__,
			csb_read(tegra, XUSB_FALC_SS_PVTPORTSC1),
			csb_read(tegra, XUSB_FALC_SS_PVTPORTSC2),
			csb_read(tegra, XUSB_FALC_SS_PVTPORTSC3),
			csb_read(tegra, XUSB_FALC_HS_PVTPORTSC1),
			csb_read(tegra, XUSB_FALC_HS_PVTPORTSC2),
			csb_read(tegra, XUSB_FALC_HS_PVTPORTSC3),
			csb_read(tegra, XUSB_FALC_FS_PVTPORTSC1),
			csb_read(tegra, XUSB_FALC_FS_PVTPORTSC2),
			csb_read(tegra, XUSB_FALC_FS_PVTPORTSC3));
	debug_print_portsc(xhci);

	ret = tegra_xhci_load_fw_from_pmc(tegra, 0);
	if (ret < 0) {
		xhci_err(xhci, "%s: error loading fw from RAM %d\n",
			__func__, ret);
		goto out;
	}

	tegra->hc_in_elpg = false;
	ret = xhci_resume(tegra->xhci, 0);
	if (ret) {
		xhci_err(xhci, "%s: could not resume right %d\n",
				__func__, ret);
		goto out;
	}

	tegra_xhci_pmc_usb2_wakenotif_off(tegra, PMC_PORT_UTMIP_P2);

	if (tegra->hs_wake_event)
		tegra->hs_wake_event = false;

	if (tegra->host_resume_req)
		tegra->host_resume_req = false;

	xhci_info(xhci, "elpg_exit: completed: lp0/elpg time=%d msec\n",
		jiffies_to_msecs(jiffies - tegra->last_jiffies));

	tegra->host_pwr_gated = false;
out:
	return ret;
}

static void host_partition_elpg_exit_work(struct work_struct *work)
{
	struct tegra_xhci_hcd *tegra = container_of(work, struct tegra_xhci_hcd,
		host_elpg_exit_work);

	mutex_lock(&tegra->sync_lock);
	tegra_xhci_host_partition_elpg_exit(tegra);
	mutex_unlock(&tegra->sync_lock);
}

/* Mailbox handling function. This function handles requests
 * from firmware and communicates with clock and powergating
 * module to alter clock rates and to power gate/ungate xusb
 * partitions.
 *
 * Following is the structure of mailbox messages.
 * bit 31:28 - msg type
 * bits 27:0 - mbox data
 * FIXME:  Check if we can just call clock functions like below
 * or should we schedule it for calling later ?
 */

static void
tegra_xhci_process_mbox_message(struct work_struct *work)
{
	u32 sw_resp = 0;
	int ret = 0, fw_msg, data_in;
	struct tegra_xhci_hcd *tegra = container_of(work, struct tegra_xhci_hcd,
					mbox_work);
	struct xhci_hcd *xhci = tegra->xhci;

	mutex_lock(&tegra->mbox_lock);

	/* get the owner id */
	tegra->mbox_owner = readl(tegra->fpci_base + XUSB_CFG_ARU_MBOX_OWNER);
	tegra->mbox_owner &= MBOX_OWNER_ID_MASK;

	/* get the mbox message from firmware */
	fw_msg = readl(tegra->fpci_base + XUSB_CFG_ARU_MBOX_DATA_OUT);

	data_in = readl(tegra->fpci_base + XUSB_CFG_ARU_MBOX_DATA_IN);
	if (data_in) {
		mutex_unlock(&tegra->mbox_lock);
		return;
	}

	/* get cmd type and cmd data */
	tegra->cmd_type	= (fw_msg & MBOX_CMD_TYPE_MASK) >> MBOX_CMD_SHIFT;
	tegra->cmd_data	= (fw_msg & MBOX_CMD_DATA_MASK);

	/* decode the message and make appropriate requests to
	 * clock or powergating module.
	 */

	switch (tegra->cmd_type) {
	case MBOX_CMD_INC_FALC_CLOCK:
	case MBOX_CMD_DEC_FALC_CLOCK:
		ret = tegra_xusb_request_clk_rate(
				tegra,
				tegra->falc_clk,
				tegra->cmd_data,
				&sw_resp);
		if (ret)
			xhci_err(xhci, "%s: could not set required falc rate\n",
				__func__);
		goto send_sw_response;
	case MBOX_CMD_INC_SSPI_CLOCK:
	case MBOX_CMD_DEC_SSPI_CLOCK:
		ret = tegra_xusb_request_clk_rate(
				tegra,
				tegra->ss_clk,
				tegra->cmd_data,
				&sw_resp);
		if (ret)
			xhci_err(xhci, "%s: could not set required ss rate.\n",
				__func__);
		goto send_sw_response;
	case MBOX_CMD_SET_BW:
		/* make sure mem bandwidth
		 * is requested in MB/s
		 */
		ret = tegra_xusb_request_clk_rate(
				tegra,
				tegra->emc_clk,
				tegra->cmd_data,
				&sw_resp);
		if (ret)
			xhci_err(xhci, "%s: could not set required mem bw.\n",
				__func__);
		goto send_sw_response;
	case MBOX_CMD_ACK:
		writel(0, tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);
		writel(0, tegra->fpci_base + XUSB_CFG_ARU_MBOX_OWNER);
		break;
	case MBOX_CMD_NACK:
		writel(0, tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);
		writel(0, tegra->fpci_base + XUSB_CFG_ARU_MBOX_OWNER);
		break;
	default:
		xhci_err(xhci, "%s: invalid cmdtype %d\n",
				__func__, tegra->cmd_type);
	}
	mutex_unlock(&tegra->mbox_lock);
	return;

send_sw_response:
	writel(sw_resp,
		 tegra->fpci_base + XUSB_CFG_ARU_MBOX_DATA_IN);


	writel(MBOX_INT_EN|MBOX_FALC_INT_EN,
			tegra->fpci_base + XUSB_CFG_ARU_MBOX_CMD);

	mutex_unlock(&tegra->mbox_lock);
	usleep_range(100, 200);
}

static irqreturn_t tegra_xhci_xusb_host_irq(int irq, void *ptrdev)
{
	struct tegra_xhci_hcd *tegra = (struct tegra_xhci_hcd *) ptrdev;
	struct xhci_hcd *xhci = tegra->xhci;

	xhci_dbg(xhci, "%s", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t tegra_xhci_padctl_irq(int irq, void *ptrdev)
{
	struct tegra_xhci_hcd *tegra = (struct tegra_xhci_hcd *) ptrdev;
	struct xhci_hcd *xhci = tegra->xhci;
	u32 elpg_program0 = 0;

	spin_lock(&tegra->lock);

	tegra->last_jiffies = jiffies;

	/* Check the intr cause. Could be  USB2 or HSIC or SS wake events */
	elpg_program0 = readl(tegra->padctl_base + ELPG_PROGRAM_0);

	xhci_dbg(xhci, "%s: elpg_program0 = %x\n",
		__func__, elpg_program0);
	xhci_dbg(xhci, "%s: PMC REGISTER = %x\n",
		__func__, readl(tegra->pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0));
	xhci_dbg(xhci, "%s: PAD ELPG_PROGRAM_0 INTERRUPT REGISTER = %x\n",
		__func__, readl(tegra->padctl_base + ELPG_PROGRAM_0));
	xhci_dbg(xhci, "%s: OC_DET Register = %x\n",
		__func__, readl(tegra->padctl_base + OC_DET_0));
	xhci_dbg(xhci, "%s: USB2_BATTERY_CHRG_OTGPAD0_0 Register = %x\n",
		__func__,
		readl(tegra->padctl_base + USB2_BATTERY_CHRG_OTGPAD0_0));
	xhci_dbg(xhci, "%s: USB2_BATTERY_CHRG_OTGPAD1_0 Register = %x\n",
		__func__,
		readl(tegra->padctl_base + USB2_BATTERY_CHRG_OTGPAD1_0));
	xhci_dbg(xhci, "%s: USB2_BATTERY_CHRG_BIASPAD_0 Register = %x\n",
		__func__,
		readl(tegra->padctl_base + USB2_BATTERY_CHRG_BIASPAD_0));

	if (elpg_program0 & SS_PORT0_WAKEUP_EVENT) {
		elpg_program0 &= ~SS_PORT0_WAKE_INTERRUPT_ENABLE;
		writel(elpg_program0 | SS_PORT0_WAKEUP_EVENT,
				tegra->padctl_base + ELPG_PROGRAM_0);
		tegra->ss_wake_event = true;

	} else if (elpg_program0 & SS_PORT1_WAKEUP_EVENT) {
		elpg_program0 &= ~SS_PORT1_WAKE_INTERRUPT_ENABLE;
		writel(elpg_program0 | SS_PORT1_WAKEUP_EVENT,
				tegra->padctl_base + ELPG_PROGRAM_0);
		tegra->ss_wake_event = true;

	} else if (elpg_program0 &  USB2_PORT0_WAKEUP_EVENT) {
		elpg_program0 &= ~USB2_PORT0_WAKE_INTERRUPT_ENABLE;
		writel(elpg_program0 | USB2_PORT0_WAKEUP_EVENT,
				tegra->padctl_base + ELPG_PROGRAM_0);
		tegra->hs_wake_event = true;

	} else if (elpg_program0 & USB2_PORT1_WAKEUP_EVENT) {
		elpg_program0 &= ~USB2_PORT1_WAKE_INTERRUPT_ENABLE;
		writel(elpg_program0 | USB2_PORT1_WAKEUP_EVENT,
				tegra->padctl_base + ELPG_PROGRAM_0);
		tegra->hs_wake_event = true;

	} else if (elpg_program0 & USB2_HSIC_PORT0_WAKEUP_EVENT) {
		writel(elpg_program0 | USB2_HSIC_PORT0_WAKEUP_EVENT,
				tegra->padctl_base + ELPG_PROGRAM_0);

	} else if (elpg_program0 & USB2_HSIC_PORT1_WAKEUP_EVENT) {
		writel(elpg_program0 | USB2_HSIC_PORT1_WAKEUP_EVENT,
				tegra->padctl_base + ELPG_PROGRAM_0);
	}
	if (tegra->ss_wake_event) {
		if (tegra->ss_pwr_gated && !tegra->host_pwr_gated) {
			xhci_dbg(xhci, "[%s] schedule ss_elpg_exit_work\n",
				__func__);
			schedule_work(&tegra->ss_elpg_exit_work);
		} else if (tegra->ss_pwr_gated
				&& tegra->host_pwr_gated) {
			xhci_dbg(xhci, "[%s] schedule host_elpg_exit_work\n",
				__func__);
			schedule_work(&tegra->host_elpg_exit_work);
		}
	} else if (tegra->hs_wake_event) {
		xhci_dbg(xhci, "[%s] schedule host_elpg_exit_work\n",
			__func__);
		schedule_work(&tegra->host_elpg_exit_work);
	} else {
		xhci_err(xhci, "error: wake due to no hs/ss event\n");
		writel(0xffffffff, tegra->padctl_base + ELPG_PROGRAM_0);
	}
	spin_unlock(&tegra->lock);
	return IRQ_HANDLED;
}

static irqreturn_t tegra_xhci_smi_irq(int irq, void *ptrdev)
{
	struct tegra_xhci_hcd *tegra = (struct tegra_xhci_hcd *) ptrdev;
	u32 temp;

	spin_lock(&tegra->lock);

	/* clear the mbox intr status 1st thing. Other
	 * bits are W1C bits, so just write to SMI bit.
	 */

	temp = readl(tegra->fpci_base + XUSB_CFG_ARU_SMI_INTR);

	/* write 1 to clear SMI INTR en bit ( bit 3 ) */
	temp = MBOX_SMI_INTR_EN;
	writel(temp, tegra->fpci_base + XUSB_CFG_ARU_SMI_INTR);

	schedule_work(&tegra->mbox_work);

	spin_unlock(&tegra->lock);
	return IRQ_HANDLED;
}

static void tegra_xhci_plat_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= XHCI_BROKEN_MSI;
	xhci->quirks &= ~XHCI_SPURIOUS_REBOOT;
}

/* called during probe() after chip reset completes */
static int xhci_plat_setup(struct usb_hcd *hcd)
{
	return xhci_gen_setup(hcd, tegra_xhci_plat_quirks);
}

static int tegra_xhci_request_mem_region(struct platform_device *pdev,
	const char *name, void __iomem **region)
{
	struct resource	*res;
	void __iomem *mem;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(&pdev->dev, "memory resource %s doesn't exist\n", name);
		return -ENODEV;
	}

	mem = devm_request_and_ioremap(&pdev->dev, res);
	if (!mem) {
		dev_err(&pdev->dev, "failed to ioremap for %s\n", name);
		return -EFAULT;
	}
	*region = mem;

	return 0;
}

static int tegra_xhci_request_irq(struct platform_device *pdev,
	const char *rscname, irq_handler_t handler, unsigned long irqflags,
	const char *devname, int *irq_no)
{
	int ret;
	struct tegra_xhci_hcd *tegra = platform_get_drvdata(pdev);
	struct resource	*res;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, rscname);
	if (!res) {
		dev_err(&pdev->dev, "irq resource %s doesn't exist\n", rscname);
		return -ENODEV;
	}

	ret = devm_request_irq(&pdev->dev, res->start, handler, irqflags,
			devname, tegra);
	if (ret != 0) {
		dev_err(&pdev->dev,
			"failed to request_irq for %s (irq %d), error = %d\n",
			devname, res->start, ret);
		return ret;
	}
	*irq_no = res->start;

	return 0;
}

#ifdef CONFIG_PM

static int tegra_xhci_bus_suspend(struct usb_hcd *hcd)
{
	struct tegra_xhci_hcd *tegra = hcd_to_tegra_xhci(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int err = 0;
	unsigned long flags;

	mutex_lock(&tegra->sync_lock);

	if (xhci->shared_hcd == hcd) {
		tegra->usb3_rh_suspend = true;
		xhci_dbg(xhci, "%s: usb3 root hub\n", __func__);
	} else if (xhci->main_hcd == hcd) {
		tegra->usb2_rh_suspend = true;
		xhci_dbg(xhci, "%s: usb2 root hub\n", __func__);
	}

	WARN_ON(tegra->hc_in_elpg);

	/* suspend xhci bus. This will also set remote mask */
	err = xhci_bus_suspend(hcd);
	if (err) {
		xhci_err(xhci, "%s: xhci_bus_suspend failed %d\n",
				__func__, err);
		goto xhci_bus_suspend_failed;
	}

	if (!(tegra->usb2_rh_suspend && tegra->usb3_rh_suspend))
		goto done; /* one of the root hubs is still working */

	spin_lock_irqsave(&tegra->lock, flags);
	tegra->hc_in_elpg = true;
	spin_unlock_irqrestore(&tegra->lock, flags);

	WARN_ON(tegra->ss_pwr_gated && tegra->host_pwr_gated);

	/* save xhci spec ctx. Already done by xhci_suspend */
	err = xhci_suspend(tegra->xhci);
	if (err) {
		xhci_err(xhci, "%s: xhci_suspend failed %d\n", __func__, err);
		goto xhci_suspend_failed;
	}

	/* Powergate host. Include ss power gate if not already done */
	err = tegra_xhci_host_elpg_entry(tegra);
	if (err) {
		xhci_err(xhci, "%s: unable to perform elpg entry %d\n",
				__func__, err);
		goto tegra_xhci_host_elpg_entry_failed;
	}

done:
	mutex_unlock(&tegra->sync_lock);
	return 0;

tegra_xhci_host_elpg_entry_failed:

xhci_suspend_failed:
	tegra->hc_in_elpg = false;
xhci_bus_suspend_failed:
	if (xhci->shared_hcd == hcd)
		tegra->usb3_rh_suspend = false;
	else if (xhci->main_hcd == hcd)
		tegra->usb2_rh_suspend = false;

	mutex_unlock(&tegra->sync_lock);
	return err;
}

/* First, USB2HCD and then USB3HCD resume will be called */
static int tegra_xhci_bus_resume(struct usb_hcd *hcd)
{
	struct tegra_xhci_hcd *tegra = hcd_to_tegra_xhci(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int err = 0;

	mutex_lock(&tegra->sync_lock);

	tegra->host_resume_req = true;

	if (xhci->shared_hcd == hcd)
		xhci_dbg(xhci, "%s: usb3 root hub\n", __func__);
	else if (xhci->main_hcd == hcd)
		xhci_dbg(xhci, "%s: usb2 root hub\n", __func__);

	if (tegra->usb2_rh_suspend && tegra->usb3_rh_suspend) {
		if (tegra->ss_pwr_gated && tegra->host_pwr_gated)
			tegra_xhci_host_partition_elpg_exit(tegra);
	}

	err = xhci_bus_resume(hcd);
	if (err) {
		xhci_err(xhci, "%s: xhci_bus_resume failed %d\n",
				__func__, err);
		goto xhci_bus_resume_failed;
	}

	if (xhci->shared_hcd == hcd)
		tegra->usb3_rh_suspend = false;
	else if (xhci->main_hcd == hcd)
		tegra->usb2_rh_suspend = false;

	mutex_unlock(&tegra->sync_lock);
	return 0;

xhci_bus_resume_failed:
	/* TODO: reverse elpg? */
	mutex_unlock(&tegra->sync_lock);
	return err;
}
#endif

static irqreturn_t tegra_xhci_irq(struct usb_hcd *hcd)
{
	struct tegra_xhci_hcd *tegra = hcd_to_tegra_xhci(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	irqreturn_t iret = IRQ_HANDLED;
	u32 status;

	spin_lock(&tegra->lock);
	if (tegra->hc_in_elpg) {
		spin_lock(&xhci->lock);
		if (HCD_HW_ACCESSIBLE(hcd)) {
			status = xhci_readl(xhci, &xhci->op_regs->status);
			status |= STS_EINT;
			xhci_writel(xhci, status, &xhci->op_regs->status);
		}
		xhci_dbg(xhci, "%s: schedule host_elpg_exit_work\n",
				__func__);
		schedule_work(&tegra->host_elpg_exit_work);
		spin_unlock(&xhci->lock);
	} else
		iret = xhci_irq(hcd);
	spin_unlock(&tegra->lock);

	return iret;
}


static const struct hc_driver tegra_plat_xhci_driver = {
	.description =		"tegra-xhci",
	.product_desc =		"Nvidia xHCI Host Controller",
	.hcd_priv_size =	sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq =			tegra_xhci_irq,
	.flags =		HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =		xhci_plat_setup,
	.start =		xhci_run,
	.stop =			xhci_stop,
	.shutdown =		xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		xhci_urb_enqueue,
	.urb_dequeue =		xhci_urb_dequeue,
	.alloc_dev =		xhci_alloc_dev,
	.free_dev =		xhci_free_dev,
	.alloc_streams =	xhci_alloc_streams,
	.free_streams =		xhci_free_streams,
	.add_endpoint =		xhci_add_endpoint,
	.drop_endpoint =	xhci_drop_endpoint,
	.endpoint_reset =	xhci_endpoint_reset,
	.check_bandwidth =	xhci_check_bandwidth,
	.reset_bandwidth =	xhci_reset_bandwidth,
	.address_device =	xhci_address_device,
	.update_hub_device =	xhci_update_hub_device,
	.reset_device =		xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =	xhci_get_frame,

	/* Root hub support */
	.hub_control =		xhci_hub_control,
	.hub_status_data =	xhci_hub_status_data,

#ifdef CONFIG_PM
	.bus_suspend =		tegra_xhci_bus_suspend,
	.bus_resume =		tegra_xhci_bus_resume,
#endif
};

#ifdef CONFIG_PM
static int
tegra_xhci_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	struct tegra_xhci_hcd *tegra = platform_get_drvdata(pdev);
	struct xhci_hcd *xhci = tegra->xhci;

	int ret = 0;

	mutex_lock(&tegra->sync_lock);
	if (!tegra->hc_in_elpg) {
		xhci_warn(xhci, "%s: lp0 suspend entry while elpg not done\n",
				__func__);
		mutex_unlock(&tegra->sync_lock);
		return -EBUSY;
	}
	mutex_unlock(&tegra->sync_lock);

	tegra_xhci_ss_wake_on_interrupts(tegra, false);
	tegra_xhci_hs_wake_on_interrupts(tegra, false);

	/* enable_irq_wake for ss ports */
	ret = enable_irq_wake(tegra->padctl_irq);
	if (ret < 0) {
		xhci_err(xhci,
		"%s: Couldn't enable USB host mode wakeup, irq=%d, error=%d\n",
		__func__, tegra->padctl_irq, ret);
	}

	/* enable_irq_wake for hs/fs/ls ports */
	ret = enable_irq_wake(tegra->usb3_irq);
	if (ret < 0) {
		xhci_err(xhci,
		"%s: Couldn't enable USB host mode wakeup, irq=%d, error=%d\n",
		__func__, tegra->usb3_irq, ret);
	}
	clk_disable(tegra->plle_clk);

	return ret;
}

static int
tegra_xhci_resume(struct platform_device *pdev)
{
	struct tegra_xhci_hcd *tegra = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	tegra->last_jiffies = jiffies;

	disable_irq_wake(tegra->padctl_irq);
	disable_irq_wake(tegra->usb3_irq);
	tegra->lp0_exit = true;

	regulator_enable(tegra->xusb_vbus_reg);
	regulator_enable(tegra->xusb_avddio_usb3_reg);
	regulator_enable(tegra->xusb_vddio_hsic_reg);
	regulator_enable(tegra->xusb_hvdd_usb3_reg);
	regulator_enable(tegra->xusb_avdd_usb3_pll_reg);
	tegra_usb2_clocks_init(tegra);

	return 0;
}
#endif

/* TODO: we have to refine error handling in tegra_xhci_probe() */
static int tegra_xhci_probe(struct platform_device *pdev)
{
	const struct hc_driver *driver;
	struct xhci_hcd	*xhci;
	struct tegra_xhci_hcd *tegra;
	struct resource	*res;
	struct usb_hcd	*hcd;
	u32 pmc_reg;
	int ret;
	int irq;

	if (usb_disabled())
		return -ENODEV;

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra) {
		dev_err(&pdev->dev, "memory alloc failed\n");
		return -ENOMEM;
	}
	tegra->pdev = pdev;

	ret = tegra_xhci_request_mem_region(pdev, "pmc", &tegra->pmc_base);
	if (ret) {
		dev_err(&pdev->dev, "failed to map pmc\n");
		return ret;
	}

	ret = tegra_xhci_request_mem_region(pdev, "padctl",
			&tegra->padctl_base);
	if (ret) {
		dev_err(&pdev->dev, "failed to map padctl\n");
		return ret;
	}

	ret = tegra_xhci_request_mem_region(pdev, "fpci", &tegra->fpci_base);
	if (ret) {
		dev_err(&pdev->dev, "failed to map fpci\n");
		return ret;
	}

	ret = tegra_xhci_request_mem_region(pdev, "ipfs", &tegra->ipfs_base);
	if (ret) {
		dev_err(&pdev->dev, "failed to map ipfs\n");
		return ret;
	}

	ret = tegra_xusb_partitions_clk_init(tegra);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to initialize xusb partitions clocks\n");
		return ret;
	}

	/* Enable UTMIP, PLLU and PLLE */
	ret = tegra_usb2_clocks_init(tegra);
	if (ret) {
		dev_err(&pdev->dev, "error initializing usb2 clocks\n");
		goto err_deinit_xusb_partition_clk;
	}

	/* tegra_unpowergate_partition also does partition reset deassert */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBA);
	if (ret)
		dev_err(&pdev->dev, "could not unpowergate xusba partition\n");

	/* tegra_unpowergate_partition also does partition reset deassert */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_XUSBC);
	if (ret)
		dev_err(&pdev->dev, "could not unpowergate xusbc partition\n");

	/* Step 5: Enable power rails to the PAD,VBUS
	 * and pull-up voltage.Initialize the regulators
	 */
	ret = tegra_xusb_regulator_init(tegra, pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize xusb regulator\n");
		goto err_deinit_usb2_clocks;
	}

	tegra->xusb_padctl = dev_get_platdata(&pdev->dev);

	/* reset the pointer back to NULL. driver uses it */
	/* platform_set_drvdata(pdev, NULL); */

	/* Program the XUSB pads to take ownership of ports */
	tegra_xhci_padctl_portmap_and_caps(tegra);

	/* Enable Vbus of host ports */
	tegra_xhci_padctl_enable_usb_vbus(tegra);

	/* Release XUSB wake logic state latching */
	tegra_xhci_ss_wake_signal(tegra, false);
	tegra_xhci_ss_vcore(tegra, false);

	/* Disable clocks to get ref cnt to 0 */
	clk_disable(tegra->host_partition_clk);
	clk_disable(tegra->ss_partition_clk);

	/* Enable host, ss, dev clocks */
	clk_enable(tegra->host_partition_clk);
	clk_enable(tegra->ss_partition_clk);

	/* Deassert reset to XUSB host, ss, dev clocks */
	tegra_periph_reset_deassert(tegra->host_partition_clk);
	tegra_periph_reset_deassert(tegra->ss_partition_clk);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "host");
	if (!res) {
		dev_err(&pdev->dev, "mem resource host doesn't exist\n");
		ret = -ENODEV;
		goto err_deinit_tegra_xusb_regulator;
	}
	tegra->host_phy_base = res->start;

	/* Setup IPFS access and BAR0 space */
	tegra_xhci_cfg(tegra);

	/* Read fw base address from SCRATCH34 register  */
	tegra->fw_phys_base = readl(tegra->pmc_base + PMC_SCRATCH34);

	/* ioremap the fw physical base address */
	tegra->fw_virt_base = (u32) devm_ioremap(&pdev->dev,
				tegra->fw_phys_base, FW_SIZE_OFFSET);
	if (!tegra->fw_virt_base) {
		dev_err(&pdev->dev, "error mapping fw memory 0x%x\n",
				tegra->fw_phys_base);
		ret = -ENOMEM;
		goto err_deinit_tegra_xusb_regulator;
	}
	dev_info(&pdev->dev, "fw_phys_base=0x%x, fw_virt_base=0x%x\n",
				tegra->fw_phys_base, tegra->fw_virt_base);

	/* Read fw size at offset 0x100 in CFG table */
	tegra->fw_size = *((u32 *)((u8 *)tegra->fw_virt_base + FW_SIZE_OFFSET));

	/* Load firmware */
	ret = tegra_xhci_load_fw_from_pmc(tegra, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error loading fw from RAM\n");
		goto err_deinit_tegra_xusb_regulator;
	}

	driver = &tegra_plat_xhci_driver;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "failed to create usb2 hcd\n");
		ret = -ENOMEM;
		goto err_deinit_tegra_xusb_regulator;
	}

	ret = tegra_xhci_request_mem_region(pdev, "host", &hcd->regs);
	if (ret) {
		dev_err(&pdev->dev, "failed to map host\n");
		goto err_put_usb2_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "host");
	if (!res) {
		dev_err(&pdev->dev, "irq resource host doesn't exist\n");
		ret = -ENODEV;
		goto err_put_usb2_hcd;
	}
	irq = res->start;
	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret) {
		dev_err(&pdev->dev, "failed to add usb2hcd, error = %d\n", ret);
		goto err_put_usb2_hcd;
	}

	/* USB 2.0 roothub is stored in the platform_device now. */
	hcd = dev_get_drvdata(&pdev->dev);
	xhci = hcd_to_xhci(hcd);
	tegra->xhci = xhci;
	platform_set_drvdata(pdev, tegra);

	xhci->shared_hcd = usb_create_shared_hcd(driver, &pdev->dev,
						dev_name(&pdev->dev), hcd);
	if (!xhci->shared_hcd) {
		dev_err(&pdev->dev, "failed to create usb3 hcd\n");
		ret = -ENOMEM;
		goto err_remove_usb2_hcd;
	}

	/*
	 * Set the xHCI pointer before xhci_plat_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

	ret = usb_add_hcd(xhci->shared_hcd, irq, IRQF_SHARED);
	if (ret) {
		dev_err(&pdev->dev, "failed to add usb3hcd, error = %d\n", ret);
		goto err_put_usb3_hcd;
	}

	spin_lock_init(&tegra->lock);
	mutex_init(&tegra->sync_lock);
	mutex_init(&tegra->mbox_lock);

	/* do mailbox related initializations */
	tegra->mbox_owner = 0xffff;
	INIT_WORK(&tegra->mbox_work, tegra_xhci_process_mbox_message);

	/* do ss partition elpg exit related initialization */
	INIT_WORK(&tegra->ss_elpg_exit_work, ss_partition_elpg_exit_work);

	/* do host partition elpg exit related initialization */
	INIT_WORK(&tegra->host_elpg_exit_work, host_partition_elpg_exit_work);

	/* Register interrupt handler for SMI line to handle mailbox
	 * interrupt from firmware
	 */
	ret = tegra_xhci_request_irq(pdev, "host-smi", tegra_xhci_smi_irq,
			IRQF_SHARED, "tegra_xhci_mbox_irq", &tegra->smi_irq);
	if (ret != 0)
		goto err_remove_usb3_hcd;

	/* Register interrupt handler for PADCTRL line to
	 * handle wake on connect irqs interrupt from
	 * firmware
	 */
	ret = tegra_xhci_request_irq(pdev, "padctl", tegra_xhci_padctl_irq,
			IRQF_SHARED | IRQF_TRIGGER_HIGH,
			"tegra_xhci_padctl_irq", &tegra->padctl_irq);
	if (ret != 0)
		goto err_remove_usb3_hcd;

	ret = tegra_xhci_request_irq(pdev, "usb3", tegra_xhci_xusb_host_irq,
			IRQF_SHARED | IRQF_TRIGGER_HIGH, "xusb_host_irq",
			&tegra->usb3_irq);
	if (ret != 0)
		goto err_remove_usb3_hcd;

	device_init_wakeup(&pdev->dev, 1);

	tegra->ss_pwr_gated = false;
	tegra->host_pwr_gated = false;
	tegra->hc_in_elpg = false;
	tegra->hs_wake_event = false;
	tegra->host_resume_req = false;
	tegra->lp0_exit = false;

	/* reset wake event to NONE */
	pmc_reg = readl(tegra->pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);
	pmc_reg |= UTMIP_WAKE_VAL(0, WAKE_VAL_NONE);
	pmc_reg |= UTMIP_WAKE_VAL(1, WAKE_VAL_NONE);
	pmc_reg |= UTMIP_WAKE_VAL(2, WAKE_VAL_NONE);
	pmc_reg |= UTMIP_WAKE_VAL(3, WAKE_VAL_NONE);
	writel(pmc_reg, tegra->pmc_base + PMC_UTMIP_UHSIC_SLEEP_CFG_0);

	tegra_xhci_debug_read_pads(tegra);

	return 0;

err_remove_usb3_hcd:
	usb_remove_hcd(xhci->shared_hcd);
err_put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);
err_remove_usb2_hcd:
	kfree(tegra->xhci);
	usb_remove_hcd(hcd);
err_put_usb2_hcd:
	usb_put_hcd(hcd);
err_deinit_tegra_xusb_regulator:
	tegra_xusb_regulator_deinit(tegra);
err_deinit_usb2_clocks:
	tegra_usb2_clocks_deinit(tegra);
err_deinit_xusb_partition_clk:
	tegra_xusb_partitions_clk_deinit(tegra);

	return ret;
}

static int tegra_xhci_remove(struct platform_device *pdev)
{
	struct tegra_xhci_hcd *tegra = platform_get_drvdata(pdev);
	struct xhci_hcd	*xhci = NULL;
	struct usb_hcd *hcd = NULL;

	if (tegra == NULL)
		return -EINVAL;

	xhci = tegra->xhci;
	hcd = xhci_to_hcd(xhci);

	devm_free_irq(&pdev->dev, tegra->usb3_irq, tegra);
	devm_free_irq(&pdev->dev, tegra->padctl_irq, tegra);
	devm_free_irq(&pdev->dev, tegra->smi_irq, tegra);
	usb_remove_hcd(xhci->shared_hcd);
	usb_put_hcd(xhci->shared_hcd);
	kfree(xhci);
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	tegra_xusb_regulator_deinit(tegra);
	tegra_usb2_clocks_deinit(tegra);
	tegra_xusb_partitions_clk_deinit(tegra);

	return 0;
}

static void tegra_xhci_shutdown(struct platform_device *pdev)
{
	struct tegra_xhci_hcd *tegra = platform_get_drvdata(pdev);
	struct xhci_hcd	*xhci = NULL;
	struct usb_hcd *hcd = NULL;

	if (tegra == NULL)
		return;

	xhci = tegra->xhci;
	hcd = xhci_to_hcd(xhci);
	xhci_shutdown(hcd);
}

static struct platform_driver tegra_xhci_driver = {
	.probe	= tegra_xhci_probe,
	.remove	= tegra_xhci_remove,
	.shutdown = tegra_xhci_shutdown,
#ifdef CONFIG_PM
	.suspend = tegra_xhci_suspend,
	.resume  = tegra_xhci_resume,
#endif
	.driver	= {
		.name = "tegra-xhci",
	},
};
MODULE_ALIAS("platform:tegra-xhci");

int tegra_xhci_register_plat(void)
{
	return platform_driver_register(&tegra_xhci_driver);
}

void tegra_xhci_unregister_plat(void)
{
	platform_driver_unregister(&tegra_xhci_driver);
}
