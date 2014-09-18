 /*
 * arch/arm64/mach-tegra/board-t210ref.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/i2c-hid.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/maxim_sti.h>
#include <linux/memblock.h>
#include <linux/spi/spi-tegra.h>
#include <linux/nfc/pn544.h>
#include <linux/rfkill-gpio.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/regulator/consumer.h>
#include <linux/smb349-charger.h>
#include <linux/max17048_battery.h>
#include <linux/leds.h>
#include <linux/i2c/at24.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/edp.h>
#include <linux/usb/tegra_usb_phy.h>
#include <linux/mfd/palmas.h>
#include <linux/clk/tegra.h>
#include <media/tegra_dtv.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/irqchip/tegra.h>
#include <linux/tegra-soc.h>
#include <linux/tegra_fiq_debugger.h>
#include <linux/platform_data/tegra_usb_modem_power.h>
#include <linux/platform_data/tegra_ahci.h>
#include <linux/irqchip/tegra.h>
#include <sound/max98090.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-tegra.h>
#include <linux/tegra_nvadsp.h>
#include <linux/tegra-pm.h>

#include <mach/irqs.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <linux/platform/tegra/isomgr.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/dc.h>
#include <mach/tegra_usb_pad_ctrl.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/xusb.h>

#include "board.h"
#include "board-panel.h"
#include "board-common.h"
#include "board-t210ref.h"
#include "board-touch-raydium.h"
#include "board-touch-maxim_sti.h"
#include <linux/platform/tegra/clock.h>
#include "common.h"
#include "devices.h"
#include "gpio-names.h"
#include "iomap.h"
#include "pm.h"
#include "tegra-board-id.h"
#include "tegra-of-dev-auxdata.h"
#include "tegra12_emc.h"
#include <linux/platform/tegra/dvfs.h>
#include "board-t210.h"

static struct tegra_usb_platform_data tegra_udc_pdata;
static struct tegra_usb_otg_data tegra_otg_pdata;

#if defined(CONFIG_TEGRA_NVADSP) && \
		!defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
static struct nvadsp_platform_data nvadsp_plat_data;
#endif

static __initdata struct tegra_clk_init_table t210ref_clk_init_table[] = {
	/* name		parent		rate		enabled */
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	{ "pll_m",	NULL,		0,		false},
#endif
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	48000000,	false},
	{ "i2s0",	"pll_a_out0",	12288000,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	12288000,	false},
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "sbc5",	"pll_p",	25000000,	false},
	{ "sbc6",	"pll_p",	25000000,	false},
#endif
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "vi_sensor2",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "extern3",	"pll_p",	41000000,	false},
	{ "clk_out_3",	"extern3",	0,		false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "vii2c",	"pll_p",	204000000,	false},
	{ "i2cslow",	"clk_m",	1000000,	false},
	{ "sbc1",	"pll_p",	25000000,	false},
	{ "sbc2",	"pll_p",	25000000,	false},
	{ "sbc3",	"pll_p",	25000000,	false},
	{ "sbc4",	"pll_p",	25000000,	false},
	{ "uarta",	"pll_p",	408000000,	true},
	{ "uartb",	"pll_p",	408000000,	false},
	{ "uartc",	"pll_p",	408000000,	false},
	{ "uartd",	"pll_p",	408000000,	false},
	{ "extern2",	"pll_p",	41000000,	false},
	{ "clk_out_2",	"extern2",	40800000,	false},
	{ "extern1",	"clk_m",	19200000,	true},
	{ "clk_out_1",	"extern1",	19200000,	true},
	{ "uart_mipi_cal",	"pll_p",	68000000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct platform_device *t210ref_devices[] __initdata = {
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	&tegra_pmu_device,
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_offload_device,
	&tegra30_avp_audio_device,
#endif
#ifdef CONFIG_TEGRA_CEC_SUPPORT
	&tegra_cec_device,
#endif
};

static void t210ref_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_pdata.is_xhci = false;
		tegra_udc_pdata.u_data.dev.is_xhci = false;
	} else {
		tegra_otg_pdata.is_xhci = true;
		tegra_udc_pdata.u_data.dev.is_xhci = true;
	}
}

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
struct of_dev_auxdata t210ref_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC1_BASE,
			"sdhci-tegra.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC2_BASE,
			"sdhci-tegra.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC3_BASE,
			"sdhci-tegra.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-sdhci", TEGRA_SDMMC4_BASE,
			"sdhci-tegra.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-udc", TEGRA_USB_BASE,
			"tegra-udc.0", &tegra_udc_pdata.u_data.dev),
	OF_DEV_AUXDATA("nvidia,tegra132-otg", TEGRA_USB_BASE,
			"tegra-otg", &tegra_otg_pdata),
	OF_DEV_AUXDATA("nvidia,tegra132-ehci", 0x7d004000, "tegra-ehci.1",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-host1x", TEGRA_HOST1X_BASE, "host1x",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-gm20b", TEGRA_GK20A_BAR0_BASE,
			"gpu.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvenc", TEGRA_NVENC_BASE, "msenc",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vi", TEGRA_VI_BASE, "vi",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vi-i2c", 0x546c0000, "vi-i2c",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-isp", TEGRA_ISP_BASE, "isp.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-isp", TEGRA_ISPB_BASE, "isp.1",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-tsec", TEGRA_TSEC_BASE, "tsec",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-tsec", TEGRA_TSECB_BASE, "tsecb",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vic", TEGRA_VIC_BASE, "vic03",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-se", 0x70012000, "tegra21-se",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dtv", 0x7000c300, "dtv", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvdec", TEGRA_NVDEC_BASE, "nvdec",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-nvjpg", TEGRA_NVJPG_BASE, "nvjpg",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-axbar",  TEGRA_AXBAR_BASE,
			"tegra210-axbar", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-adma",  TEGRA_ADMA_BASE,
			"tegra210-adma", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c000,
			"tegra21-i2c.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c400,
			"tegra21-i2c.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c500,
			"tegra21-i2c.2", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000c700,
			"tegra21-i2c.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000d000,
			"tegra21-i2c.4", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-i2c", 0x7000d100,
			"tegra21-i2c.5", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-vii2c", 0,
			"tegra21-i2c.6", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000d400, "spi-tegra114.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000d600, "spi-tegra114.1",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000d800, "spi-tegra114.2",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-spi", 0x7000da00, "spi-tegra114.3",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-qspi", 0x70410000, "qspi",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006000, "serial8250.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006040, "serial8250.1",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006200, "serial8250.2",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-uart", 0x70006300, "serial8250.3",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006000, "serial-tegra.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006040, "serial-tegra.1",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006200, "serial-tegra.2",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006300, "serial-tegra.3",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ahci-sata", 0x70027000, "tegra-sata.0",
		NULL),
#if defined(CONFIG_TEGRA_NVADSP) && \
		!defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	OF_DEV_AUXDATA("nvidia,tegra210-adsp", TEGRA_APE_AMC_BASE,
			"tegra210-adsp", &nvadsp_plat_data),
#endif
	OF_DEV_AUXDATA("nvidia,tegra210-adsp-audio", 0, "adsp_audio.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra-bluedroid_pm", 0, "bluedroid_pm",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra-bcmdhd-wlan", 0, "bcm4329_wlan",
			NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 0, "spdif-dit.0", NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 1, "spdif-dit.1", NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 2, "spdif-dit.2", NULL),
	OF_DEV_AUXDATA("linux,spdif-dit", 3, "spdif-dit.3", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dfll", 0x70110000, "tegra_cl_dvfs",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-xhci", 0x70090000, "tegra-xhci",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-xudc", 0x700D0000, "tegra-xudc",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-pwm", 0x7000a000, "tegra-pwm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-camera", 0, "pcl-generic",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra210-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
			NULL),
	OF_DEV_AUXDATA("pwm-backlight", 0, "pwm-backlight", NULL),
#ifdef CONFIG_TEGRA_CEC_SUPPORT
	OF_DEV_AUXDATA("nvidia,tegra210-cec", 0x70015000, "tegra_cec", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra-audio-rt5639", 0x0, "tegra-snd-rt5639",
			NULL),
	OF_DEV_AUXDATA("nvidia,icera-i500", 0, "tegra_usb_modem_power", NULL),
	OF_DEV_AUXDATA("raydium,rm_ts_spidev", 0, "rm_ts_spidev", NULL),
	OF_DEV_AUXDATA("nvidia,tegra30-hda", 0x70030000, "tegra30-hda", NULL),
	OF_DEV_AUXDATA("nvidia,ptm", 0x72010000, "ptm", NULL),
	{}
};
#else
static struct of_dev_auxdata t210ref_auxdata_lookup[] __initdata = {
	T124_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra-audio-rt5639", 0x0, "tegra-snd-rt5639",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-se", 0x70012000, "tegra12-se", NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-dtv", 0x7000c300, "dtv", NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-udc", 0x7d000000, "tegra-udc.0",
			&tegra_udc_pdata.u_data.dev),
	OF_DEV_AUXDATA("nvidia,tegra132-otg", 0x7d000000, "tegra-otg",
			&tegra_otg_pdata),
	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE,
			"gk20a.0", NULL),
#ifdef CONFIG_ARCH_TEGRA_VIC
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
			NULL),
#ifdef CONFIG_VI_ONE_DEVICE
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi", NULL),
#else
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	T124_UART_OF_DEV_AUXDATA,
	T124_I2C_OF_DEV_AUXDATA,
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	/* supports t210-interposer using t13x */
	T124_SDMMC_OF_DEV_AUXDATA,
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nvavp", 0x60001000, "nvavp",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-pwm", 0x7000a000, "tegra-pwm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dfll", 0x70110000, "tegra_cl_dvfs",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-dfll", 0x70040084, "tegra_cl_dvfs",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-camera", 0, "pcl-generic",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-ahci-sata", 0x70027000, "tegra-sata.0",
			NULL),
	OF_DEV_AUXDATA("nvidia,tegra-bluedroid_pm", 0, "bluedroid_pm",
			NULL),
	OF_DEV_AUXDATA("pwm-backlight", 0, "pwm-backlight", NULL),
	OF_DEV_AUXDATA("nvidia,icera-i500", 0, "tegra_usb_modem_power", NULL),
	OF_DEV_AUXDATA("raydium,rm_ts_spidev", 0, "rm_ts_spidev", NULL),
	OF_DEV_AUXDATA("nvidia,tegra30-hda", 0x70030000, "tegra30-hda", NULL),
	OF_DEV_AUXDATA("nvidia,ptm", 0x72010000, "ptm", NULL),
	{}
};
#endif

static void __init tegra_t210ref_early_init(void)
{
	if (!tegra_platform_is_fpga()) {
		tegra_clk_init_from_table(t210ref_clk_init_table);
		tegra_clk_verify_parents();
	}
	if (of_machine_is_compatible("nvidia,e2141"))
		tegra_soc_device_init("e2141");
	else if (of_machine_is_compatible("nvidia,e2220"))
		tegra_soc_device_init("e2220");
	else if (of_machine_is_compatible("nvidia,e2190"))
		tegra_soc_device_init("e2190");
	else if (of_machine_is_compatible("nvidia,grenada"))
		tegra_soc_device_init("grenada");
	else if (of_machine_is_compatible("nvidia,loki-e"))
		tegra_soc_device_init("loki_e");
	else if (of_machine_is_compatible("nvidia,foster-e"))
		tegra_soc_device_init("foster_e");
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

static struct tegra_suspend_platform_data t210ref_suspend_data = {
	.cpu_timer      = 500,
	.cpu_off_timer  = 300,
	.suspend_mode   = TEGRA_SUSPEND_LP0,
	.core_timer     = 0x157e,
	.core_off_timer = 10,
	.corereq_high   = true,
	.sysclkreq_high = true,
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	.cpu_lp2_min_residency = 1000,
	.min_residency_vmin_fmin = 1000,
	.min_residency_ncpu_fast = 8000,
	.min_residency_ncpu_slow = 5000,
	.min_residency_mclk_stop = 5000,
	.min_residency_crail = 20000,
#endif
};

#define T210REF_CPU_VDD_MIN_UV		703000
#define T210REF_CPU_VDD_STEP_UV		19200
#define E2190_CPU_VDD_MIN_UV		606250
#define E2190_CPU_VDD_STEP_UV		6250


static int __init t210ref_rail_alignment_init(void)
{
	int step_uv, offset_uv;

	if (of_machine_is_compatible("nvidia,e2190")) {
		step_uv = E2190_CPU_VDD_STEP_UV;
		offset_uv = E2190_CPU_VDD_MIN_UV;
	} else {
		step_uv = T210REF_CPU_VDD_STEP_UV;
		offset_uv = T210REF_CPU_VDD_MIN_UV;
	}

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	tegra21x_vdd_cpu_align(step_uv, offset_uv);
#else
	tegra13x_vdd_cpu_align(step_uv, offset_uv);
#endif
	return 0;
}

static int __init t210ref_suspend_init(void)
{
	tegra_init_suspend(&t210ref_suspend_data);
	return 0;
}

static void __init tegra_t210ref_late_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);

	t210ref_usb_init();
	platform_add_devices(t210ref_devices, ARRAY_SIZE(t210ref_devices));
	tegra_io_dpd_init();
	/* FIXME: Assumed all t210ref platforms have sdhci DT support */
	t210ref_suspend_init();

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	tegra21_emc_init();
#else
	tegra12_emc_init();
#endif
	t210ref_edp_init();
	isomgr_init();
	tegra_fb_copy_or_clear();

	/* put PEX pads into DPD mode to save additional power */
	tegra_io_dpd_enable(&pexbias_io);
	tegra_io_dpd_enable(&pexclk1_io);
	tegra_io_dpd_enable(&pexclk2_io);

	t210ref_camera_init();

	t210ref_thermal_sensors_init();
	t210ref_soctherm_init();
}

static void __init tegra_t210ref_init_early(void)
{
	t210ref_rail_alignment_init();
#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
	tegra21x_init_early();
#else
	tegra12x_init_early();
#endif
}

static int tegra_t210ref_notifier_call(struct notifier_block *nb,
				    unsigned long event, void *data)
{
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	struct device *dev = data;
#endif

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
		if (dev->of_node) {
			if (of_device_is_compatible(dev->of_node,
				"pwm-backlight")) {
				tegra_pwm_bl_ops_register(dev);
			}
		}
#endif
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block platform_nb = {
	.notifier_call = tegra_t210ref_notifier_call,
};
static void __init tegra_t210ref_dt_init(void)
{
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	/*
	 * In t210ref, zero display_board_id is considered to
	 * jdi 1440x810 5.8" one.
	 */
	tegra_set_fixed_panel_ops(true, &dsi_j_1440_810_5_8_ops,
		"j,1440-810-5-8");
	tegra_set_fixed_pwm_bl_ops(dsi_j_1440_810_5_8_ops.pwm_bl_ops);
#endif
	bus_register_notifier(&platform_bus_type, &platform_nb);
	tegra_t210ref_early_init();
	t210ref_camera_auxdata(t210ref_auxdata_lookup);
	of_platform_populate(NULL,
		of_default_bus_match_table, t210ref_auxdata_lookup,
		&platform_bus);

	tegra_t210ref_late_init();
}

static void __init tegra_t210ref_reserve(void)
{
#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	ulong tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM) || \
		defined(CONFIG_TEGRA_NO_CARVEOUT)
	ulong carveout_size = 0;
	ulong fb2_size = SZ_16M;
#else
	ulong carveout_size = SZ_1G;
	ulong fb2_size = SZ_4M;
#endif
	ulong fb1_size = SZ_16M + SZ_2M;
	ulong vpr_size = 186 * SZ_1M;

#if defined(CONFIG_TEGRA_NVADSP) && \
		!defined(CONFIG_TEGRA_NVADSP_ON_SMMU)
	nvadsp_plat_data.co_pa = tegra_reserve_adsp(SZ_32M);
	nvadsp_plat_data.co_size = SZ_32M;
#endif

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* support FBcon on 4K monitors */
	fb2_size = SZ_64M + SZ_8M;	/* 4096*2160*4*2 = 70778880 bytes */
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	tmp = fb1_size;
	fb1_size = fb2_size;
	fb2_size = tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

	tegra_reserve4(carveout_size, fb1_size, fb2_size, vpr_size);
}

static const char * const t210ref_dt_board_compat[] = {
	"nvidia,e2220",
	"nvidia,e2190",
	"nvidia,e2141",
	"nvidia,grenada",
	"nvidia,loki-e",
	"nvidia,foster-e",
	NULL
};

DT_MACHINE_START(T210REF, "t210ref")
	.atag_offset	= 0x100,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_t210ref_reserve,
	.init_early	= tegra_t210ref_init_early,
#if defined(CONFIG_ARCH_TEGRA_13x_SOC)
	.smp		= smp_ops(tegra_smp_ops),
	.init_irq	= irqchip_init,
#endif
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_t210ref_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= t210ref_dt_board_compat,
	.init_late      = tegra_init_late,
MACHINE_END

#if defined(CONFIG_ARCH_TEGRA_21x_SOC)
void tegra_pd_in_idle(bool enable) {}
#endif
