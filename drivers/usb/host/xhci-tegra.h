/*
 * xhci-tegra.h - Nvidia xHCI host controller related data
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

#ifndef __XUSB_H
#define __XUSB_H

#define XUSB_CSB_MP_L2IMEMOP_TRIG				0x00101A14
#define XUSB_CSB_MP_APMAP					0x0010181C
#define XUSB_CSB_ARU_SCRATCH0				0x00100100

/* Nvidia Cfg Registers */

#define XUSB_CFG_0					0x00000000
#define XUSB_CFG_1					0x00000004
#define XUSB_CFG_4					0x00000010
#define XUSB_CFG_16					0x00000040
#define XUSB_CFG_24					0x00000060
#define XUSB_CFG_FPCICFG					0x000000F8
#define XUSB_CFG_ARU_C11_CSBRANGE				0x0000041C
#define XUSB_CFG_ARU_SMI_INTR				0x00000428
#define XUSB_CFG_ARU_RST					0x0000042C
#define XUSB_CFG_ARU_SMI_INTR				0x00000428
#define XUSB_CFG_ARU_CONTEXT				0x0000043C
#define XUSB_CFG_ARU_FW_SCRATCH				0x00000440
#define XUSB_CFG_CSB_BASE_ADDR				0x00000800
#define XUSB_CFG_ARU_CONTEXT_HSFS_SPEED			0x00000480
#define XUSB_CFG_ARU_CONTEXT_HS_PLS			0x00000478
#define XUSB_CFG_ARU_CONTEXT_FS_PLS			0x0000047C
#define XUSB_CFG_ARU_CONTEXT_HSFS_SPEED			0x00000480
#define XUSB_CFG_ARU_CONTEXT_HSFS_PP			0x00000484
#define XUSB_CFG_CSB_BASE_ADDR				0x00000800

/* DEVICE ID */
#define XUSB_DEVICE_ID_T114				0x0E16
#define XUSB_DEVICE_ID_T124				0x0FA3
#define XUSB_DEVICE_ID_T210				0x0FAC

/* TODO: Do not have the definitions of below
 * registers.
 */

/* BAR0 Registers */
#define BAR0_XHCI_OP_PORTSC(i)                         (0x00000440+(i)*16)

/* IPFS Registers to save and restore  */
#define	IPFS_XUSB_HOST_MSI_BAR_SZ_0			0xC0
#define	IPFS_XUSB_HOST_MSI_AXI_BAR_ST_0			0xC4
#define	IPFS_XUSB_HOST_FPCI_BAR_ST_0			0xC8
#define	IPFS_XUSB_HOST_MSI_VEC0_0			0x100
#define	IPFS_XUSB_HOST_MSI_EN_VEC0_0			0x140
#define	IPFS_XUSB_HOST_CONFIGURATION_0			0x180
#define	IPFS_XUSB_HOST_FPCI_ERROR_MASKS_0		0x184
#define	IPFS_XUSB_HOST_INTR_MASK_0			0x188
#define	IPFS_XUSB_HOST_IPFS_INTR_ENABLE_0		0x198
#define	IPFS_XUSB_HOST_UFPCI_CONFIG_0			0x19C
#define	IPFS_XUSB_HOST_CLKGATE_HYSTERESIS_0		0x1BC
#define IPFS_XUSB_HOST_MCCIF_FIFOCTRL_0			0x1DC

/* IPFS bit definitions */
#define IPFS_EN_FPCI					(1 << 0)
#define IPFS_IP_INT_MASK				(1 << 16)

/* Nvidia MailBox Registers */

#define XUSB_CFG_ARU_MBOX_CMD		0xE4
#define XUSB_CFG_ARU_MBOX_DATA_IN	0xE8
#define  CMD_DATA_SHIFT		(0)
#define  CMD_DATA_MASK			(0xFFFFFF)
#define  CMD_DATA(_x)			((_x & CMD_DATA_MASK) << CMD_DATA_SHIFT)
#define  CMD_TYPE_SHIFT		(24)
#define  CMD_TYPE_MASK			(0xFF)
#define  CMD_TYPE(_x)			((_x & CMD_TYPE_MASK) << CMD_TYPE_SHIFT)
#define XUSB_CFG_ARU_MBOX_DATA_OUT	0xEC
#define XUSB_CFG_ARU_MBOX_OWNER	0xF0

/* Nvidia Falcon Registers */
#define XUSB_FALC_CPUCTL					0x00000100
#define XUSB_FALC_BOOTVEC					0x00000104
#define XUSB_FALC_DMACTL					0x0000010C
#define XUSB_FALC_IMFILLRNG1					0x00000154
#define XUSB_FALC_IMFILLCTL					0x00000158
#define XUSB_FALC_CMEMBASE					0x00000160
#define XUSB_FALC_DMEMAPERT					0x00000164
#define XUSB_FALC_IMEMC_START					0x00000180
#define XUSB_FALC_IMEMD_START					0x00000184
#define XUSB_FALC_IMEMT_START					0x00000188
#define XUSB_FALC_ICD_CMD					0x00000200
#define XUSB_FALC_ICD_RDATA					0x0000020C
#define XUSB_FALC_SS_PVTPORTSC1					0x00116000
#define XUSB_FALC_SS_PVTPORTSC2					0x00116004
#define XUSB_FALC_SS_PVTPORTSC3					0x00116008
#define XUSB_FALC_HS_PVTPORTSC1					0x00116800
#define XUSB_FALC_HS_PVTPORTSC2					0x00116804
#define XUSB_FALC_HS_PVTPORTSC3					0x00116808
#define XUSB_FALC_FS_PVTPORTSC1					0x00117000
#define XUSB_FALC_FS_PVTPORTSC2					0x00117004
#define XUSB_FALC_FS_PVTPORTSC3					0x00117008

#define XUSB_FALC_STATE_HALTED					0x00000010
/* Nvidia mailbox constants */
#define MBOX_INT_EN			(1 << 31)
#define MBOX_XHCI_INT_EN	(1 << 30)
#define MBOX_SMI_INT_EN		(1 << 29)
#define MBOX_PME_INT_EN		(1 << 28)
#define MBOX_FALC_INT_EN	(1 << 27)

#define MBOX_OWNER_FW						1
#define MBOX_OWNER_SW						2
#define MBOX_OWNER_ID_MASK					0xFF

#define SMI_INTR_STATUS_MBOX				(1 << 3)
#define SMI_INTR_STATUS_FW_REINIT			(1 << 1)

/* PMC Register */
#define PMC_SCRATCH34						0x124

#define TEGRA_POWERGATE_XUSBA	20
#define TEGRA_POWERGATE_XUSBB	21
#define TEGRA_POWERGATE_XUSBC	22

/* Nvidia Constants */
#define IMEM_BLOCK_SIZE						256

#define MEMAPERT_ENABLE						0x00000010
#define DMEMAPERT_ENABLE_INIT				0x00000000
#define CPUCTL_STARTCPU						0x00000002
#define L2IMEMOP_SIZE_SRC_OFFSET_SHIFT		8
#define L2IMEMOP_SIZE_SRC_OFFSET_MASK		0x3ff
#define L2IMEMOP_SIZE_SRC_COUNT_SHIFT		24
#define L2IMEMOP_SIZE_SRC_COUNT_MASK		0xff
#define L2IMEMOP_TRIG_LOAD_LOCKED_SHIFT	24
#define IMFILLRNG_TAG_MASK			0xffff
#define IMFILLRNG1_TAG_HI_SHIFT		16
#define APMAP_BOOTPATH						(1 << 31)
#define L2IMEM_INVALIDATE_ALL				0x40000000
#define L2IMEM_LOAD_LOCKED_RESULT			(0x11 << 24)
#define FW_SIZE_OFFSET						0x64
#define HSIC_PORT1	0
#define HSIC_PORT0	1
#define ULPI_PORT	2
#define OTG_PORT1	3
#define OTG_PORT2	4

/* Nvidia Host Controller Device and Vendor ID */
#define XUSB_USB_DID		0xE16
#define XUSB_USB_VID		0x10DE

/* Nvidia CSB MP Registers */
#define XUSB_CSB_MP_ILOAD_ATTR		0x00101A00
#define XUSB_CSB_MP_ILOAD_BASE_LO		0x00101A04
#define XUSB_CSB_MP_ILOAD_BASE_HI		0x00101A08
#define XUSB_CSB_MP_L2IMEMOP_SIZE		0x00101A10

/*Nvidia CFG registers */
#define XUSB_CFG_ARU_CONTEXT_HSFS_SPEED	0x00000480
#define XUSB_CFG_ARU_CONTEXT_HS_PLS		0x00000478
#define XUSB_CFG_ARU_CONTEXT_FS_PLS		0x0000047C
#define ARU_CONTEXT_HSFS_PP				0x00000484
#define ARU_ULPI_REGACCESS				0x474
#define ARU_ULPI_REGACCESS_ADDR_MASK	0xff00
#define ARU_ULPI_REGACCESS_CMD_MASK		0x1
#define ARU_ULPI_REGACCESS_DATA_MASK	0xff0000

/*
 * FIXME: looks like no any .c requires below structure types
 * revisit and decide whether we can delete or not
 */
struct usb2_pad_port_map {
	u32 hsic_port0;
	u32 hsic_port1;
	u32 ulpi_port;
	u32 otg_port1;
	u32 otg_port0;
};

struct usb2_otg_caps {
	u32 port0_cap;
	u32 port0_internal;
	u32 port1_cap;
	u32 port1_internal;
};

struct usb2_ulpi_caps {
	u32	port_cap;
	u32 port_internal;
};

/* this is used to assign the SuperSpeed port mapping
 * to USB2.0 ports owned by XUSB, where the SuperSpeed ports inherit
 * their port capabilities from the USB2.0 ports they mapped to*/

struct usb2_ss_port_map {
	u32	port0;
	u32 port1;
};

struct hsic_pad0_ctl_0_vals {
	u32 tx_rtunep;
	u32 tx_rtunen;
	u32 tx_slewp;
	u32 tx_slewn;
	u32 hsic_opt;
};

struct hsic_pad0_ctl_1_vals {
	u32	tx_rtunep;
	u32 tx_rtunen;
	u32 tx_slewp;
	u32 tx_slewn;
	u32 hsic_opt;
};

struct snps_oc_map_0 {
	u32 controller1_oc;
	u32 controller2_oc;
	u32 controller3_oc;
};

struct usb2_oc_map_0 {
	u32 port0;
	u32 port1;
};

struct vbus_enable_oc_map {
	u32 vbus_en0;
	u32 vbus_en1;
};

/**
 * port_to_hsic_pad - given "port number", return with hsic pad number
 * @_port:	(zero-based) index to portsc registers array.
 */
#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
#define port_to_hsic_pad(_port) ({			\
	int _pad = -1;					\
	int __p = _port;				\
	if (__p == 5)					\
		_pad = 0;				\
	else if (__p == 6)				\
		_pad = 1;				\
	_pad; })
#define hsic_pad_to_port(_pad) ({			\
	int _port = -1;					\
	int __p = _pad;					\
	if (__p == 0)					\
		_port = 5;				\
	else if (__p == 1)				\
		_port = 6;				\
	_port; })
#elif defined(CONFIG_ARCH_TEGRA_12x_SOC) || defined(CONFIG_ARCH_TEGRA_21x_SOC)
#define port_to_hsic_pad(_port) ({			\
	int _pad = -1;					\
	int __p = _port;				\
	if (__p == 6)					\
		_pad = 0;				\
	else if (__p == 7)				\
		_pad = 1;				\
	_pad; })

#define hsic_pad_to_port(_pad) ({			\
	int _port = -1;					\
	int __p = _pad;					\
	if (__p == 0)					\
		_port = 6;				\
	else if (__p == 1)				\
		_port = 7;				\
	_port; })
#endif
#endif
