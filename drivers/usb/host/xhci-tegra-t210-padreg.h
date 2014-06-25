/*
 * xhci-tegra-T210-padreg.h - Nvidia xHCI host controller related data
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

/* xusb_padctl register offsets */

/* XUSB_PADCTL_USB2_PAD_MUX_0 0x4 */
#define USB2_OTG_PAD_PORT_MASK(x)		(0x3 << (2 * x))
#define USB2_OTG_PAD_PORT_OWNER_XUSB(x) (0x1 << (2 * x))
#define USB2_ULPI_PAD	0x0
#define USB2_ULPI_PAD_OWNER_XUSB	0x0
#define USB2_HSIC_PAD_PORT(p)	(1 << (20 + p))

/* XUSB_PADCTL_USB2_PORT_CAP_0 0x8 */
#define USB2_PORT_CAP_MASK(x)		(0x3 << (4 * x))
#define USB2_PORT_CAP_HOST(x)		(0x1 << (4 * x))
#define USB2_ULPI_PORT_CAP	0x0

/* XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_2_0 NOT_SUPPORT */
/* XUSB_PADCTL_IOPHY_USB3_PAD1_CTL_2_0 NOT_SUPPORT */
#define IOPHY_USB3_RXWANDER		0x0
#define IOPHY_USB3_RXEQ			0x0
#define IOPHY_USB3_CDRCNTL		0x0

/* XUSB_PADCTL_USB2_BIAS_PAD_CTL_0_0 0x284 */
#define USB2_BIAS_HS_SQUELCH_LEVEL (0x0)
#define USB2_BIAS_HS_DISCON_LEVEL (0x0)

/* XUSB_PADCTL_USB2_OC_MAP_0 0x10 */
#define OC_DETECTION_DISABLED 0xf
#define USB2_OC_MAP(x, val) ((val & 0xf) << (4 * x))
#define VBUS_ENABLE_OC_MAP(x, v)	\
	((v & 0xf) << (4 * x))

/* XUSB_PADCTL_USB2_OTG_PAD0_CTL_0_0 0x88*/
/* XUSB_PADCTL_USB2_OTG_PAD1_CTL_0_0 0xc8*/
/* XUSB_PADCTL_USB2_OTG_PAD2_CTL_0_0 0x108*/
/* XUSB_PADCTL_USB2_OTG_PAD3_CTL_0_0 0x148*/
#define USB2_OTG_HS_CURR_LVL (0x3F << 0)
#define USB2_OTG_HS_SLEW 0x0
#define USB2_OTG_FS_SLEW 0x0
#define USB2_OTG_LS_RSLEW 0x0
#define USB2_OTG_LS_FSLEW 0x0
#define USB2_OTG_PD (0x1 << 26)
#define USB2_OTG_PD2 (0x1 << 27)
#define USB2_OTG_PD_ZI (0x1 << 29)

/* XUSB_PADCTL_USB2_OTG_PAD0_CTL_1_0 0x8c*/
/* XUSB_PADCTL_USB2_OTG_PAD1_CTL_1_0 0xcc*/
/* XUSB_PADCTL_USB2_OTG_PAD2_CTL_1_0 0x10c*/
/* XUSB_PADCTL_USB2_OTG_PAD3_CTL_1_0 0x14c*/
#define USB2_OTG_PD_DR (0x1 << 2)
#define USB2_OTG_TERM_RANGE_AD (0xF << 3)
#define USB2_OTG_PD_CHRP_FORCE_POWERUP 0x0
#define USB2_OTG_PD_DISC_FORCE_POWERUP 0x0
#define USB2_OTG_HS_IREF_CAP 0x0

/* XUSB_PADCTL_SS_PORT_MAP_0 0x14 */
#define SS_PORT_MAP(_p, val) \
	((val & 0x7) << (_p * 5))

/* XUSB_PADCTL_OC_DET_0 0x1c */
#define OC_DET_OC_DETECTED_VBUS_PAD0 (1 << 12)
#define OC_DET_OC_DETECTED_VBUS_PAD1 (1 << 13)
#define OC_DET_OC_DETECTED_VBUS_PAD2 (1 << 14)
#define OC_DET_OC_DETECTED_VBUS_PAD3 (1 << 15)

/* DFE/CTLE No need for T210 */
#define tegra_xhci_restore_ctle_context(hcd, port)	\
	do {} while (0)
#define tegra_xhci_save_ctle_context(hcd, port)	\
	do {} while (0)

/* XUSB_PADCTL_IOPHY_MISC_PAD_P0_CTL_6_0 NOT_SUPPORT */
/* XUSB_PADCTL_IOPHY_MISC_PAD_P1_CTL_6_0 NOT_SUPPORT */
#define MISC_OUT_SEL(x) 0x0
#define MISC_OUT_TAP_VAL(reg) 0x0
#define MISC_OUT_AMP_VAL(reg) 0x0
#define MISC_OUT_G_Z_VAL(reg) 0x0

/* XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_4_0 NOT_SUPPORT */
/* XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_4_0 NOT_SUPPORT */
#define DFE_CNTL_TAP_VAL(x) 0x0
#define DFE_CNTL_AMP_VAL(x) 0x0

/* XUSB_PADCTL_IOPHY_USB3_PAD0_CTL_2_0 NOT_SUPPORT*/
/* XUSB_PADCTL_IOPHY_USB3_PAD1_CTL_2_0 NOT_SUPPORT*/
#define RX_EQ_G_VAL(x) 0x0
#define RX_EQ_Z_VAL(x) 0x0

/* XUSB_PADCTL_IOPHY_MISC_PAD_P0_CTL_2_0 NOT_SUPPORT*/
/* XUSB_PADCTL_IOPHY_MISC_PAD_P1_CTL_2_0 NOT_SUPPORT*/
#define SPARE_IN(x) 0x0

/* XUSB_PADCTL_IOPHY_MISC_PAD_P0_CTL_3_0 NOT_SUPPORT*/
/* XUSB_PADCTL_IOPHY_MISC_PAD_P0_CTL_3_0 NOT_SUPPORT*/
#define RX_IDLE_MODE			0x0
#define RX_IDLE_MODE_OVRD		0x0

/* XUSB_PADCTL_IOPHY_MISC_PAD_P0_CTL_5_0 NOT_SUPPORT*/
/* XUSB_PADCTL_IOPHY_MISC_PAD_P1_CTL_5_0 NOT_SUPPORT*/
#define RX_QEYE_EN				0x0

/* XUSB_PADCTL_USB2_BIAS_PAD_CTL_1_0 NOT_SUPPORT */
#define RCTRL(x)			0x0
#define TCTRL(x)			0x0

/* XUSB_PADCTL_HSIC_PAD0_CTL_2_0 0x308 */
/* XUSB_PADCTL_HSIC_PAD1_CTL_2_0 0x328 */
#define RX_DATA_TRIM(x)		0x0
#define RX_STROBE_TRIM(x)	0x0
#define CALIOUT(x)			0x0

/* XUSB_PADCTL_HSIC_PAD0_CTL_1_0 0x304 */
/* XUSB_PADCTL_HSIC_PAD1_CTL_1_0 0x324 */
#define TX_RTUNEP(x)	0x0
#define TX_RTUNEN(x)	0x0
#define TX_SLEWP(x)		0x0
#define TX_SLEWN(x)		0x0
#define HSIC_OPT(x)		0x0

/* XUSB_PADCTL_HSIC_PAD0_CTL_0_0 0x300 */
/* XUSB_PADCTL_HSIC_PAD1_CTL_0_0 0x320 */
#define GET_HSIC_REG_OFFSET()	(padregs->usb2_hsic_padX_ctlY_0[pad][0])
#define HSIC_IDDQ		(0x1 << 0)
#define PD_TX_DATA0		(0x1 << 1)
#define PD_TX_DATA1		(0x1 << 2)
#define PD_TX_STROBE	(0x1 << 3)
#define PD_TX		(PD_TX_DATA0 | PD_TX_DATA1 | PD_TX_STROBE)

#define PD_RX_DATA0		(0x1 << 4)
#define PD_RX_DATA1		(0x1 << 5)
#define PD_RX_STROBE	(0x1 << 6)
#define PD_RX		(PD_RX_DATA0 | PD_RX_DATA1 | PD_RX_STROBE)

#define PD_ZI_DATA0		(0x1 << 7)
#define PD_ZI_DATA1		(0x1 << 8)
#define PD_ZI_STROBE	(0x1 << 9)
#define HSIC_PD_ZI	(PD_ZI_DATA0 | PD_ZI_DATA1 | PD_ZI_STROBE)

#define LBPK_DATA0		(0x1 << 10)
#define LBPK_DATA1		(0x1 << 11)
#define LBPK_STROBE		(0x1 << 12)
#define RPD_DATA0		(0x1 << 13)
#define RPD_DATA1	(0x1 << 14)
#define RPD_DATA	(RPD_DATA0 | RPD_DATA1)
#define RPD_STROBE	(0x1 << 15)

#define RPU_DATA0	(0x1 << 16)
#define RPU_DATA1	(0x1 << 17)
#define RPU_DATA	(RPU_DATA0 | RPU_DATA1)
#define RPU_STROBE	(0x1 << 18)
#define PD_TRX		0x0

/* XUSB_PADCTL_HSIC_PAD_TRK_CTL_0 0x340*/
#define AUTO_TERM_EN	(0x1 << 24)

/* XUSB_PADCTL_HSIC_STRB_TRIM_CONTROL_0 0x344*/
#define STRB_TRIM_VAL(x)	0x0
