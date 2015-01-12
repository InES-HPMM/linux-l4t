/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>
#include <linux/nvhost.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/tegra_v4l2_camera.h>

#include <mach/clk.h>

#include "nvhost_syncpt.h"
#include "nvhost_acm.h"
#include "bus_client.h"
#include "common.h"

#define TEGRA_SYNCPT_RETRY_COUNT	10

#define TEGRA_SYNCPT_CSI_WAIT_TIMEOUT                   200

/* VI registers */
#define TEGRA_VI_CFG_VI_INCR_SYNCPT			0x000
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL		0x004
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR		0x008
#define TEGRA_VI_CFG_CTXSW				0x020
#define TEGRA_VI_CFG_INTSTATUS				0x024
#define TEGRA_VI_CFG_PWM_CONTROL			0x038
#define TEGRA_VI_CFG_PWM_HIGH_PULSE			0x03c
#define TEGRA_VI_CFG_PWM_LOW_PULSE			0x040
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_A			0x044
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_B			0x048
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_C			0x04c
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_D			0x050
#define TEGRA_VI_CFG_VGP1				0x064
#define TEGRA_VI_CFG_VGP2				0x068
#define TEGRA_VI_CFG_VGP3				0x06c
#define TEGRA_VI_CFG_VGP4				0x070
#define TEGRA_VI_CFG_VGP5				0x074
#define TEGRA_VI_CFG_VGP6				0x078
#define TEGRA_VI_CFG_INTERRUPT_MASK			0x08c
#define TEGRA_VI_CFG_INTERRUPT_TYPE_SELECT		0x090
#define TEGRA_VI_CFG_INTERRUPT_POLARITY_SELECT		0x094
#define TEGRA_VI_CFG_INTERRUPT_STATUS			0x098
#define TEGRA_VI_CFG_VGP_SYNCPT_CONFIG			0x0ac
#define TEGRA_VI_CFG_VI_SW_RESET			0x0b4
#define TEGRA_VI_CFG_CG_CTRL				0x0b8
#define TEGRA_VI_CFG_VI_MCCIF_FIFOCTRL			0x0e4
#define TEGRA_VI_CFG_TIMEOUT_WCOAL_VI			0x0e8
#define TEGRA_VI_CFG_DVFS				0x0f0
#define TEGRA_VI_CFG_RESERVE				0x0f4
#define TEGRA_VI_CFG_RESERVE_1				0x0f8

/* CSI registers */
#define TEGRA_VI_CSI_0_BASE				0x100
#define TEGRA_VI_CSI_1_BASE				0x200
#define TEGRA_VI_CSI_2_BASE				0x300
#define TEGRA_VI_CSI_3_BASE				0x400
#define TEGRA_VI_CSI_4_BASE				0x500
#define TEGRA_VI_CSI_5_BASE				0x600

#define TEGRA_VI_CSI_SW_RESET				0x000
#define TEGRA_VI_CSI_SINGLE_SHOT			0x004
#define TEGRA_VI_CSI_SINGLE_SHOT_STATE_UPDATE		0x008
#define TEGRA_VI_CSI_IMAGE_DEF				0x00c
#define TEGRA_VI_CSI_RGB2Y_CTRL				0x010
#define TEGRA_VI_CSI_MEM_TILING				0x014
#define TEGRA_VI_CSI_IMAGE_SIZE				0x018
#define TEGRA_VI_CSI_IMAGE_SIZE_WC			0x01c
#define TEGRA_VI_CSI_IMAGE_DT				0x020
#define TEGRA_VI_CSI_SURFACE0_OFFSET_MSB		0x024
#define TEGRA_VI_CSI_SURFACE0_OFFSET_LSB		0x028
#define TEGRA_VI_CSI_SURFACE1_OFFSET_MSB		0x02c
#define TEGRA_VI_CSI_SURFACE1_OFFSET_LSB		0x030
#define TEGRA_VI_CSI_SURFACE2_OFFSET_MSB		0x034
#define TEGRA_VI_CSI_SURFACE2_OFFSET_LSB		0x038
#define TEGRA_VI_CSI_SURFACE0_BF_OFFSET_MSB		0x03c
#define TEGRA_VI_CSI_SURFACE0_BF_OFFSET_LSB		0x040
#define TEGRA_VI_CSI_SURFACE1_BF_OFFSET_MSB		0x044
#define TEGRA_VI_CSI_SURFACE1_BF_OFFSET_LSB		0x048
#define TEGRA_VI_CSI_SURFACE2_BF_OFFSET_MSB		0x04c
#define TEGRA_VI_CSI_SURFACE2_BF_OFFSET_LSB		0x050
#define TEGRA_VI_CSI_SURFACE0_STRIDE			0x054
#define TEGRA_VI_CSI_SURFACE1_STRIDE			0x058
#define TEGRA_VI_CSI_SURFACE2_STRIDE			0x05c
#define TEGRA_VI_CSI_SURFACE_HEIGHT0			0x060
#define TEGRA_VI_CSI_ISPINTF_CONFIG			0x064
#define TEGRA_VI_CSI_ERROR_STATUS			0x084
#define TEGRA_VI_CSI_ERROR_INT_MASK			0x088
#define TEGRA_VI_CSI_WD_CTRL				0x08c
#define TEGRA_VI_CSI_WD_PERIOD				0x090

#define TEGRA_CSI_CSI_CAP_CIL				0x808
#define TEGRA_CSI_CSI_CAP_CSI				0x818
#define TEGRA_CSI_CSI_CAP_PP				0x828

/* CSI Pixel Parser registers */
#define TEGRA_CSI_PIXEL_PARSER_0_BASE			0x0838
#define TEGRA_CSI_PIXEL_PARSER_1_BASE			0x086c
#define TEGRA_CSI_PIXEL_PARSER_2_BASE			0x1038
#define TEGRA_CSI_PIXEL_PARSER_3_BASE			0x106c
#define TEGRA_CSI_PIXEL_PARSER_4_BASE			0x1838
#define TEGRA_CSI_PIXEL_PARSER_5_BASE			0x186c

#define TEGRA_CSI_INPUT_STREAM_CONTROL			0x000
#define TEGRA_CSI_PIXEL_STREAM_CONTROL0			0x004
#define TEGRA_CSI_PIXEL_STREAM_CONTROL1			0x008
#define TEGRA_CSI_PIXEL_STREAM_GAP			0x00c
#define TEGRA_CSI_PIXEL_STREAM_PP_COMMAND		0x010
#define TEGRA_CSI_PIXEL_STREAM_EXPECTED_FRAME		0x014
#define TEGRA_CSI_PIXEL_PARSER_INTERRUPT_MASK		0x018
#define TEGRA_CSI_PIXEL_PARSER_STATUS			0x01c
#define TEGRA_CSI_CSI_SW_SENSOR_RESET			0x020

/* CSI PHY registers */
#define TEGRA_CSI_CIL_PHY_0_BASE			0x0908
#define TEGRA_CSI_CIL_PHY_1_BASE			0x1108
#define TEGRA_CSI_CIL_PHY_2_BASE			0x1908
#define TEGRA_CSI_PHY_CIL_COMMAND			0x0908

/* CSI CIL registers */
#define TEGRA_CSI_CIL_0_BASE				0x092c
#define TEGRA_CSI_CIL_1_BASE				0x0960
#define TEGRA_CSI_CIL_2_BASE				0x112c
#define TEGRA_CSI_CIL_3_BASE				0x1160
#define TEGRA_CSI_CIL_4_BASE				0x192c
#define TEGRA_CSI_CIL_5_BASE				0x1960

#define TEGRA_CSI_CIL_PAD_CONFIG0			0x000
#define TEGRA_CSI_CIL_PAD_CONFIG1			0x004
#define TEGRA_CSI_CIL_PHY_CONTROL			0x008
#define TEGRA_CSI_CIL_INTERRUPT_MASK			0x00c
#define TEGRA_CSI_CIL_STATUS				0x010
#define TEGRA_CSI_CILX_STATUS				0x014
#define TEGRA_CSI_CIL_ESCAPE_MODE_COMMAND		0x018
#define TEGRA_CSI_CIL_ESCAPE_MODE_DATA			0x01c
#define TEGRA_CSI_CIL_SW_SENSOR_RESET			0x020

/* CSI Pattern Generator registers */
#if (IS_ENABLED(CONFIG_ARCH_TEGRA_12x_SOC) || \
	    IS_ENABLED(CONFIG_ARCH_TEGRA_13x_SOC))
#define TEGRA_CSI_PATTERN_GENERATOR_0_BASE		0xa68
#define TEGRA_CSI_PATTERN_GENERATOR_1_BASE		0xa9c
#else
#define TEGRA_CSI_PATTERN_GENERATOR_0_BASE		0x09c4
#define TEGRA_CSI_PATTERN_GENERATOR_1_BASE		0x09f8
#define TEGRA_CSI_PATTERN_GENERATOR_2_BASE		0x11c4
#define TEGRA_CSI_PATTERN_GENERATOR_3_BASE		0x11f8
#define TEGRA_CSI_PATTERN_GENERATOR_4_BASE		0x19c4
#define TEGRA_CSI_PATTERN_GENERATOR_5_BASE		0x19f8
#endif

#define TEGRA_CSI_PATTERN_GENERATOR_CTRL		0x000
#define TEGRA_CSI_PG_BLANK				0x004
#define TEGRA_CSI_PG_PHASE				0x008
#define TEGRA_CSI_PG_RED_FREQ				0x00c
#define TEGRA_CSI_PG_RED_FREQ_RATE			0x010
#define TEGRA_CSI_PG_GREEN_FREQ				0x014
#define TEGRA_CSI_PG_GREEN_FREQ_RATE			0x018
#define TEGRA_CSI_PG_BLUE_FREQ				0x01c
#define TEGRA_CSI_PG_BLUE_FREQ_RATE			0x020
#define TEGRA_CSI_PG_AOHDR				0x024

#define TEGRA_CSI_DPCM_CTRL_A				0xad0
#define TEGRA_CSI_DPCM_CTRL_B				0xad4
#define TEGRA_CSI_STALL_COUNTER				0xae8
#define TEGRA_CSI_CSI_READONLY_STATUS			0xaec
#define TEGRA_CSI_CSI_SW_STATUS_RESET			0xaf0
#define TEGRA_CSI_CLKEN_OVERRIDE			0xaf4
#define TEGRA_CSI_DEBUG_CONTROL				0xaf8
#define TEGRA_CSI_DEBUG_COUNTER_0			0xafc
#define TEGRA_CSI_DEBUG_COUNTER_1			0xb00
#define TEGRA_CSI_DEBUG_COUNTER_2			0xb04

/* These go into the TEGRA_VI_CSI_n_IMAGE_DEF registers bits 23:16 */
#define TEGRA_IMAGE_FORMAT_T_L8				16
#define TEGRA_IMAGE_FORMAT_T_R16_I			32
#define TEGRA_IMAGE_FORMAT_T_B5G6R5			33
#define TEGRA_IMAGE_FORMAT_T_R5G6B5			34
#define TEGRA_IMAGE_FORMAT_T_A1B5G5R5			35
#define TEGRA_IMAGE_FORMAT_T_A1R5G5B5			36
#define TEGRA_IMAGE_FORMAT_T_B5G5R5A1			37
#define TEGRA_IMAGE_FORMAT_T_R5G5B5A1			38
#define TEGRA_IMAGE_FORMAT_T_A4B4G4R4			39
#define TEGRA_IMAGE_FORMAT_T_A4R4G4B4			40
#define TEGRA_IMAGE_FORMAT_T_B4G4R4A4			41
#define TEGRA_IMAGE_FORMAT_T_R4G4B4A4			42
#define TEGRA_IMAGE_FORMAT_T_A8B8G8R8			64
#define TEGRA_IMAGE_FORMAT_T_A8R8G8B8			65
#define TEGRA_IMAGE_FORMAT_T_B8G8R8A8			66
#define TEGRA_IMAGE_FORMAT_T_R8G8B8A8			67
#define TEGRA_IMAGE_FORMAT_T_A2B10G10R10		68
#define TEGRA_IMAGE_FORMAT_T_A2R10G10B10		69
#define TEGRA_IMAGE_FORMAT_T_B10G10R10A2		70
#define TEGRA_IMAGE_FORMAT_T_R10G10B10A2		71
#define TEGRA_IMAGE_FORMAT_T_A8Y8U8V8			193
#define TEGRA_IMAGE_FORMAT_T_V8U8Y8A8			194
#define TEGRA_IMAGE_FORMAT_T_A2Y10U10V10		197
#define TEGRA_IMAGE_FORMAT_T_V10U10Y10A2		198
#define TEGRA_IMAGE_FORMAT_T_Y8_U8__Y8_V8		200
#define TEGRA_IMAGE_FORMAT_T_Y8_V8__Y8_U8		201
#define TEGRA_IMAGE_FORMAT_T_U8_Y8__V8_Y8		202
#define TEGRA_IMAGE_FORMAT_T_T_V8_Y8__U8_Y8		203
#define TEGRA_IMAGE_FORMAT_T_T_Y8__U8__V8_N444		224
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N444		225
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N444		226
#define TEGRA_IMAGE_FORMAT_T_Y8__U8__V8_N422		227
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N422		228
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N422		229
#define TEGRA_IMAGE_FORMAT_T_Y8__U8__V8_N420		230
#define TEGRA_IMAGE_FORMAT_T_Y8__U8V8_N420		231
#define TEGRA_IMAGE_FORMAT_T_Y8__V8U8_N420		232
#define TEGRA_IMAGE_FORMAT_T_X2Lc10Lb10La10		233
#define TEGRA_IMAGE_FORMAT_T_A2R6R6R6R6R6		234

/* These go into the TEGRA_VI_CSI_n_CSI_IMAGE_DT registers bits 7:0 */
#define TEGRA_IMAGE_DT_YUV420_8				24
#define TEGRA_IMAGE_DT_YUV420_10			25
#define TEGRA_IMAGE_DT_YUV420CSPS_8			28
#define TEGRA_IMAGE_DT_YUV420CSPS_10			29
#define TEGRA_IMAGE_DT_YUV422_8				30
#define TEGRA_IMAGE_DT_YUV422_10			31
#define TEGRA_IMAGE_DT_RGB444				32
#define TEGRA_IMAGE_DT_RGB555				33
#define TEGRA_IMAGE_DT_RGB565				34
#define TEGRA_IMAGE_DT_RGB666				35
#define TEGRA_IMAGE_DT_RGB888				36
#define TEGRA_IMAGE_DT_RAW6				40
#define TEGRA_IMAGE_DT_RAW7				41
#define TEGRA_IMAGE_DT_RAW8				42
#define TEGRA_IMAGE_DT_RAW10				43
#define TEGRA_IMAGE_DT_RAW12				44
#define TEGRA_IMAGE_DT_RAW14				45

static int vi2_port_is_valid(int port)
{
	return (((port) >= TEGRA_CAMERA_PORT_CSI_A) &&
		((port) <= TEGRA_CAMERA_PORT_CSI_B));
}

/* Clock settings for camera */
static struct tegra_camera_clk vi2_clks0[] = {
	{
		.name = "vi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "vi_sensor",
		.freq = 24000000,
	},
	{
		.name = "csi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "isp",
		.freq = 0,
	},
	{
		.name = "csus",
		.freq = 0,
		.use_devname = 1,
	},
	{
		.name = "sclk",
		.freq = 80000000,
	},
	{
		.name = "emc",
		.freq = 300000000,
	},
	{
		.name = "cilab",
		.freq = 102000000,
		.use_devname = 1,
	},
	/* Always put "p11_d" at the end */
	{
		.name = "pll_d",
		.freq = 927000000,
	},
};

static struct tegra_camera_clk vi2_clks1[] = {
	{
		.name = "vi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "vi_sensor2",
		.freq = 24000000,
	},
	{
		.name = "csi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "isp",
		.freq = 0,
	},
	{
		.name = "sclk",
		.freq = 80000000,
	},
	{
		.name = "emc",
		.freq = 300000000,
	},
	{
		.name = "cilcd",
		.freq = 102000000,
		.use_devname = 1,
	},
	{
		.name = "cile",
		.freq = 102000000,
		.use_devname = 1,
	},
	/* Always put "p11_d" at the end */
	{
		.name = "pll_d",
		.freq = 927000000,
	},
};

#define MAX_DEVID_LENGTH	16

static void vi2_init_syncpts(struct tegra_camera *cam)
{
	cam->syncpt_id = nvhost_get_syncpt_client_managed(
				dev_name(&cam->pdev->dev));
}

static void vi2_free_syncpts(struct tegra_camera *cam)
{
	nvhost_free_syncpt(cam->syncpt_id);
}

static void vi2_incr_syncpts(struct tegra_camera *cam)
{
	return;
}

static int vi2_clock_init(struct tegra_camera *cam, int port)
{
	struct platform_device *pdev = cam->pdev;
	struct tegra_camera_clk *clks;
	int i;

	switch (port) {
	case TEGRA_CAMERA_PORT_CSI_A:
		cam->num_clks = ARRAY_SIZE(vi2_clks0);
		cam->clks = vi2_clks0;
		break;
	case TEGRA_CAMERA_PORT_CSI_B:
		cam->num_clks = ARRAY_SIZE(vi2_clks1);
		cam->clks = vi2_clks1;
		break;
	default:
		dev_err(&pdev->dev, "Wrong port number %d\n", port);
		return -ENODEV;
	}

	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];

		if (clks->use_devname) {
			char devname[MAX_DEVID_LENGTH];
			snprintf(devname, MAX_DEVID_LENGTH,
				 "tegra_%s", dev_name(&pdev->dev));
			clks->clk = clk_get_sys(devname, clks->name);
		} else
			clks->clk = clk_get(&pdev->dev, clks->name);
		if (IS_ERR_OR_NULL(clks->clk)) {
			dev_err(&pdev->dev, "Failed to get clock %s.\n",
				clks->name);
			return PTR_ERR(clks->clk);
		}
	}

	return 0;
}

static void vi2_clock_deinit(struct tegra_camera *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_put(clks->clk);
	}
}

static void vi2_ops_deinit(struct tegra_camera *cam)
{
	struct platform_device *pdev = cam->pdev;

	nvhost_client_device_release(pdev);
	cam->ndata->aperture[0] = NULL;
	vi2_free_syncpts(cam);
}

static int vi2_ops_init(struct tegra_camera *cam)
{
	struct platform_device *pdev = cam->pdev;
	struct nvhost_device_data *ndata = cam->ndata;
	int err;

	/* Init syncpts */
	vi2_init_syncpts(cam);

	/* Init Regulator */
	cam->reg = devm_regulator_get(&pdev->dev, cam->regulator_name);
	if (IS_ERR_OR_NULL(cam->reg)) {
		dev_err(&pdev->dev, "%s: couldn't get regulator %s, err %ld\n",
			__func__, cam->regulator_name, PTR_ERR(cam->reg));
		cam->reg = NULL;
		goto exit;
	}

	mutex_init(&ndata->lock);
	platform_set_drvdata(pdev, ndata);
	err = nvhost_client_device_get_resources(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: nvhost get resources failed %d\n",
				__func__, err);
		goto exit;
	}

	if (!ndata->aperture[0]) {
		if (ndata->master) {
			struct nvhost_device_data *master_ndata =
				ndata->master->dev.platform_data;
			ndata->aperture[0] = master_ndata->aperture[0];
		} else {
			dev_err(&pdev->dev, "%s: failed to map register base\n",
				__func__);
			err = -ENXIO;
			goto exit;
		}
	}

	/* Match the nvhost_module_init VENC powergating */
	tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
	nvhost_module_init(pdev);

	err = nvhost_client_device_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: nvhost init failed %d\n",
				__func__, err);
		goto exit;
	}

	return 0;

exit:
	vi2_ops_deinit(cam);
	return 0;
}

static void vi2_clock_start(struct tegra_camera *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks - 1; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_prepare_enable(clks->clk);
		if (clks->freq > 0)
			clk_set_rate(clks->clk, clks->freq);
	}

	if (cam->tpg_mode) {
		clks = &cam->clks[i];
		if (clks->clk) {
			clk_prepare_enable(clks->clk);
			if (clks->freq > 0)
				clk_set_rate(clks->clk, clks->freq);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_DSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_MIPI_CSI_OUT_ENB, 0);
		}
	}
}

static void vi2_clock_stop(struct tegra_camera *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks - 1; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_disable_unprepare(clks->clk);
	}

	if (cam->tpg_mode) {
		clks = &cam->clks[i];
		if (clks->clk) {
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_MIPI_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_CSI_OUT_ENB, 0);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_DSI_OUT_ENB, 0);
			clk_disable_unprepare(clks->clk);
		}
	}
}

/* Reset VI2/CSI2 when activating, no sepecial ops for deactiving  */
static void vi2_sw_reset(struct tegra_camera *cam)
{
	/* T12_CG_2ND_LEVEL_EN */
	TC_VI_REG_WT(cam, TEGRA_VI_CFG_CG_CTRL, 1);

	TC_VI_REG_WT(cam, TEGRA_CSI_CLKEN_OVERRIDE, 0x0);

	udelay(10);
}

static void vi2_capture_clean(struct tegra_camera *cam)
{
	/* Clean up status */
	cil_regs_write(cam, TEGRA_CSI_CIL_STATUS, 0xFFFFFFFF);
	cil_regs_write(cam, TEGRA_CSI_CILX_STATUS, 0xFFFFFFFF);
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_PARSER_STATUS, 0xFFFFFFFF);
	csi_regs_write(cam, TEGRA_VI_CSI_ERROR_STATUS, 0xFFFFFFFF);
}

static int vi2_activate(struct tegra_camera *cam, int port)
{
	int ret = 0;

	/* Init Clocks */
	vi2_clock_init(cam, port);
	vi2_clock_start(cam);

	ret = nvhost_module_busy_ext(cam->pdev);
	if (ret) {
		dev_err(&cam->pdev->dev, "nvhost module is busy\n");
		goto exit;
	}

	/* Enable external power */
	if (cam->reg) {
		ret = regulator_enable(cam->reg);
		if (ret)
			dev_err(&cam->pdev->dev, "enabling regulator failed\n");
	}

	vi2_sw_reset(cam);

	/* Unpowergate VE */
	tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);

	vi2_capture_clean(cam);

	cam->sof = 1;

	return 0;

exit:
	vi2_clock_stop(cam);
	vi2_clock_deinit(cam);
	return ret;
}

static void vi2_deactivate(struct tegra_camera *cam)
{
	vi2_clock_stop(cam);
	vi2_clock_deinit(cam);

	/* Powergate VE */
	tegra_powergate_partition(TEGRA_POWERGATE_VENC);

	/* Disable external power */
	if (cam->reg)
		regulator_disable(cam->reg);

	nvhost_module_idle_ext(cam->pdev);

	cam->sof = 0;
}

#define TEGRA_CSI_CILA_PAD_CONFIG0	0x92c
#define TEGRA_CSI_CILB_PAD_CONFIG0	0x960
#define TEGRA_CSI_CILC_PAD_CONFIG0	0x994
#define TEGRA_CSI_CILD_PAD_CONFIG0	0x9c8
#define TEGRA_CSI_CILE_PAD_CONFIG0	0xa08

#define TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK	0x938
#define TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK	0x96c
#define TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK	0x9a0
#define TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK	0x9d4
#define TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK	0xa14

#define TEGRA_CSI_PHY_CILA_CONTROL0	0x934
#define TEGRA_CSI_PHY_CILB_CONTROL0	0x968
#define TEGRA_CSI_PHY_CILC_CONTROL0	0x99c
#define TEGRA_CSI_PHY_CILD_CONTROL0	0x9d0
#define TEGRA_CSI_PHY_CILE_CONTROL0	0xa10

static void vi2_capture_setup_cil_t124(struct tegra_camera *cam, int port)
{
	if (port == TEGRA_CAMERA_PORT_CSI_A) {
		/*
		 * PAD_CILA_PDVCLAMP 0, PAD_CILA_PDIO_CLK 0,
		 * PAD_CILA_PDIO 0, PAD_AB_BK_MODE 1
		 */
		TC_VI_REG_WT(cam, TEGRA_CSI_CILA_PAD_CONFIG0, 0x10000);

		/* PAD_CILB_PDVCLAMP 0, PAD_CILB_PDIO_CLK 0, PAD_CILB_PDIO 0 */
		TC_VI_REG_WT(cam, TEGRA_CSI_CILB_PAD_CONFIG0, 0x0);

		TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK, 0x0);

#ifdef DEBUG
		TC_VI_REG_WT(cam, TEGRA_CSI_DEBUG_CONTROL,
			     0x3 | (0x1 << 5) | (0x40 << 8));
#endif
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILA_CONTROL0, 0x9);
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILB_CONTROL0, 0x9);
	} else {
		/*
		 * PAD_CILC_PDVCLAMP 0, PAD_CILC_PDIO_CLK 0,
		 * PAD_CILC_PDIO 0, PAD_CD_BK_MODE 1
		 */
		TC_VI_REG_WT(cam, TEGRA_CSI_CILC_PAD_CONFIG0, 0x10000);

		/* PAD_CILD_PDVCLAMP 0, PAD_CILD_PDIO_CLK 0, PAD_CILD_PDIO 0 */
		TC_VI_REG_WT(cam, TEGRA_CSI_CILD_PAD_CONFIG0, 0x0);

		/* PAD_CILE_PDVCLAMP 0, PAD_CILE_PDIO_CLK 0, PAD_CILE_PDIO 0 */
		TC_VI_REG_WT(cam, TEGRA_CSI_CILE_PAD_CONFIG0, 0x0);

		TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK, 0x0);
#ifdef DEBUG
		TC_VI_REG_WT(cam, TEGRA_CSI_DEBUG_CONTROL,
				0x5 | (0x1 << 5) | (0x50 << 8));
#endif
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILC_CONTROL0, 0x9);
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILD_CONTROL0, 0x9);
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILE_CONTROL0, 0x9);
	}
}

static void vi2_capture_setup_cil_phy_t124(struct tegra_camera *cam,
					   int lanes, int port)
{
	u32 val;

	/* Shared register */
	val = TC_VI_REG_RD(cam, TEGRA_CSI_PHY_CIL_COMMAND);
	if (port == TEGRA_CAMERA_PORT_CSI_A) {
		if (lanes == 4)
			TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND,
					(val & 0xFFFF0000) | 0x0101);
		else
			TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND,
					(val & 0xFFFF0000) | 0x0201);
	} else {
		if (lanes == 4)
			TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND,
					(val & 0x0000FFFF) | 0x21010000);
		else if (lanes == 1)
			TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND,
					(val & 0x0000FFFF) | 0x12020000);
		else
			TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND,
					(val & 0x0000FFFF) | 0x22010000);
	}
}

static u32 vi2_cal_regs_base(u32 regs_base, int port)
{
	return regs_base + (port / 2 * 0x800) + (port & 1) * 0x34;
}

static int vi2_capture_setup(struct tegra_camera *cam)
{
	struct vb2_buffer *vb = cam->active;
	struct tegra_camera_buffer *buf = to_tegra_vb(vb);
	struct soc_camera_device *icd = buf->icd;
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	int port = pdata->port;
	struct cam_regs_config *regs = &cam->regs;
	int format = 0, data_type = 0, image_size = 0;
	u32 val;

	/* Skip VI2/CSI2 setup for second and later frame capture */
	if (!cam->sof)
		return 0;

	regs->csi_base = TEGRA_VI_CSI_0_BASE + port * 0x100;
	regs->csi_pp_base = vi2_cal_regs_base(TEGRA_CSI_PIXEL_PARSER_0_BASE,
			port);
	regs->cil_base = vi2_cal_regs_base(TEGRA_CSI_CIL_0_BASE, port);
	regs->cil_phy_base = TEGRA_CSI_CIL_PHY_0_BASE + port / 2 * 0x800;
	regs->tpg_base = vi2_cal_regs_base(TEGRA_CSI_PATTERN_GENERATOR_0_BASE,
			port);

	/* CIL PHY register setup */
	if (IS_ENABLED(CONFIG_ARCH_TEGRA_12x_SOC) ||
			IS_ENABLED(CONFIG_ARCH_TEGRA_13x_SOC))
		vi2_capture_setup_cil_t124(cam, pdata->port);
	else {
		cil_regs_write(cam, TEGRA_CSI_CIL_PAD_CONFIG0,
				(pdata->port & 0x1) ? 0x0 : 0x10000);
		cil_regs_write(cam, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);
		cil_regs_write(cam, TEGRA_CSI_CIL_PHY_CONTROL, 0xA);
		if (pdata->lanes == 4) {
			regs->cil_base = vi2_cal_regs_base(TEGRA_CSI_CIL_0_BASE,
							   port + 1);
			cil_regs_write(cam, TEGRA_CSI_CIL_PAD_CONFIG0, 0x0);
			cil_regs_write(cam, TEGRA_CSI_CIL_INTERRUPT_MASK, 0x0);
			cil_regs_write(cam, TEGRA_CSI_CIL_PHY_CONTROL, 0xA);
			regs->cil_base = vi2_cal_regs_base(TEGRA_CSI_CIL_0_BASE,
							   port);
		}
	}

	/* CSI pixel parser registers setup */
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND, 0xf007);
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_PARSER_INTERRUPT_MASK, 0x0);
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_CONTROL0,
			0x280301f0 | (port & 0x1));
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND, 0xf007);
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_CONTROL1, 0x11);
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_GAP, 0x140000);
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_EXPECTED_FRAME, 0x0);
	csi_pp_regs_write(cam, TEGRA_CSI_INPUT_STREAM_CONTROL,
			0x3f0000 | (pdata->lanes - 1));

	/* CIL PHY register setup */
	if (IS_ENABLED(CONFIG_ARCH_TEGRA_12x_SOC) ||
	    IS_ENABLED(CONFIG_ARCH_TEGRA_13x_SOC))
		vi2_capture_setup_cil_phy_t124(cam, pdata->lanes, pdata->port);
	else {
		if (pdata->lanes == 4)
			cil_phy_reg_write(cam, 0x0101);
		else
			cil_phy_reg_write(cam, 0x0201);
	}

	if (cam->tpg_mode) {
		tpg_regs_write(cam, TEGRA_CSI_PATTERN_GENERATOR_CTRL,
				((cam->tpg_mode - 1) << 2) | 0x1);
		tpg_regs_write(cam, TEGRA_CSI_PG_PHASE, 0x0);
		tpg_regs_write(cam, TEGRA_CSI_PG_RED_FREQ, 0x100010);
		tpg_regs_write(cam, TEGRA_CSI_PG_RED_FREQ_RATE, 0x0);
		tpg_regs_write(cam, TEGRA_CSI_PG_GREEN_FREQ, 0x100010);
		tpg_regs_write(cam, TEGRA_CSI_PG_GREEN_FREQ_RATE, 0x0);
		tpg_regs_write(cam, TEGRA_CSI_PG_BLUE_FREQ, 0x100010);
		tpg_regs_write(cam, TEGRA_CSI_PG_BLUE_FREQ_RATE, 0x0);
		if (IS_ENABLED(CONFIG_ARCH_TEGRA_12x_SOC) ||
		    IS_ENABLED(CONFIG_ARCH_TEGRA_13x_SOC))
			TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND,
				     0x22020202);
		else
			cil_phy_reg_write(cam, 0x0202);

		format = TEGRA_IMAGE_FORMAT_T_A8B8G8R8;
		data_type = TEGRA_IMAGE_DT_RGB888;
		image_size = icd->user_width * 3;
	} else if ((icd->current_fmt->code == V4L2_MBUS_FMT_UYVY8_2X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_VYUY8_2X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_YUYV8_2X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_YVYU8_2X8)) {
		/* TBD */
	} else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG8_1X8)) {
		format = TEGRA_IMAGE_FORMAT_T_L8;
		data_type = TEGRA_IMAGE_DT_RAW8;
		image_size = icd->user_width;
	} else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10) ||
		   (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10)) {
		format = TEGRA_IMAGE_FORMAT_T_R16_I;
		data_type = TEGRA_IMAGE_DT_RAW10;
		image_size = icd->user_width * 10 / 8;
	} else if (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB12_1X12) {
		format = TEGRA_IMAGE_FORMAT_T_R16_I;
		data_type = TEGRA_IMAGE_DT_RAW12;
		image_size = icd->user_width * 12 / 8;
	}

	csi_regs_write(cam, TEGRA_VI_CSI_IMAGE_DEF,
			(cam->tpg_mode ? 0 : 1 << 24) | (format << 16) | 0x1);
	csi_regs_write(cam, TEGRA_VI_CSI_IMAGE_DT, data_type);
	csi_regs_write(cam, TEGRA_VI_CSI_IMAGE_SIZE_WC, image_size);
	csi_regs_write(cam, TEGRA_VI_CSI_IMAGE_SIZE,
			(icd->user_height << 16) | icd->user_width);

	return 0;
}

static int vi2_capture_buffer_setup(struct tegra_camera *cam,
			struct tegra_camera_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);

	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		/* FIXME: Setup YUV buffer */

	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_RGB32:
		csi_regs_write(cam,
			       TEGRA_VI_CSI_SURFACE0_OFFSET_MSB +
			       buf->output_channel * 8,
			       0x0);
		csi_regs_write(cam,
			       TEGRA_VI_CSI_SURFACE0_OFFSET_LSB +
			       buf->output_channel * 8,
			       buf->buffer_addr);
		csi_regs_write(cam,
			       TEGRA_VI_CSI_SURFACE0_STRIDE +
			       buf->output_channel * 4,
			       bytes_per_line);
		break;
	default:
		dev_err(&cam->pdev->dev, "Wrong host format %d\n",
			icd->current_fmt->host_fmt->fourcc);
		return -EINVAL;
	}

	return 0;
}

static void vi2_capture_error_status(struct tegra_camera *cam,
				     int port, int err)
{
	u32 val;

	dev_err(&cam->pdev->dev,
		"CSI %d syncpt timeout, syncpt = %d, err = %d\n", port,
		cam->syncpt_id, err);

#ifdef DEBUG
	val = TC_VI_REG_RD(cam, TEGRA_CSI_DEBUG_COUNTER_0);
	pr_err("TEGRA_CSI_DEBUG_COUNTER_0 0x%08x\n", val);
#endif
	val = cil_regs_read(cam, TEGRA_CSI_CIL_STATUS);
	pr_err("TEGRA_CSI_CSI_CIL_STATUS 0x%08x\n", val);
	val = cil_regs_read(cam, TEGRA_CSI_CILX_STATUS);
	pr_err("TEGRA_CSI_CSI_CILX_STATUS 0x%08x\n", val);
	val = csi_pp_regs_read(cam, TEGRA_CSI_PIXEL_PARSER_STATUS);
	pr_err("TEGRA_CSI_PIXEL_PARSER_STATUS 0x%08x\n", val);
	val = csi_regs_read(cam, TEGRA_VI_CSI_ERROR_STATUS);
	pr_err("TEGRA_VI_CSI_ERROR_STATUS 0x%08x\n", val);
}

static int vi2_capture_start(struct tegra_camera *cam,
				      struct tegra_camera_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	int port = pdata->port;
	int err;
	u32 val;

	err = vi2_capture_buffer_setup(cam, buf);
	if (err < 0)
		return err;

	if (IS_ENABLED(CONFIG_ARCH_TEGRA_12x_SOC) ||
	    IS_ENABLED(CONFIG_ARCH_TEGRA_13x_SOC))
		TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT,
			((6 + port * 1) << 8) | cam->syncpt_id);
	else
		TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT,
			((7 + port * 4) << 8) | cam->syncpt_id);
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND, 0xf005);
	csi_regs_write(cam, TEGRA_VI_CSI_SINGLE_SHOT, 0x1);

	if (!nvhost_syncpt_read_ext_check(cam->pdev,
				cam->syncpt_id, &val))
		cam->syncpt_thresh = nvhost_syncpt_incr_max_ext(cam->pdev,
					cam->syncpt_id, 1);
	err = nvhost_syncpt_wait_timeout_ext(cam->pdev,
			cam->syncpt_id,	cam->syncpt_thresh,
			TEGRA_SYNCPT_CSI_WAIT_TIMEOUT,
			NULL,
			NULL);

	/* Mark SOF flag to Zero after we captured the FIRST frame */
	if (cam->sof)
		cam->sof = 0;

	/* Capture syncpt timeout err, then dump error status */
	if (err)
		vi2_capture_error_status(cam, port, err);

	return err;
}

static int vi2_capture_stop(struct tegra_camera *cam, int port)
{
	csi_pp_regs_write(cam, TEGRA_CSI_PIXEL_STREAM_PP_COMMAND, 0xf002);

	return 0;
}

static int vi2_capture_frame(struct tegra_camera *cam,
			     struct tegra_camera_buffer *buf)
{
	struct vb2_buffer *vb = cam->active;
	struct soc_camera_device *icd = buf->icd;
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	int port = pdata->port;
	int retry = TEGRA_SYNCPT_RETRY_COUNT;
	int err;

	/* Setup capture registers */
	vi2_capture_setup(cam);

	vi2_incr_syncpts(cam);

	while (retry) {
		err = vi2_capture_start(cam, buf);
		/* Capturing succeed, stop capturing */
		vi2_capture_stop(cam, port);
		if (err) {
			retry--;
			vi2_incr_syncpts(cam);
			continue;
		}
		break;
	}

	/* Reset hardware for too many errors */
	if (!retry) {
		vi2_deactivate(cam);
		mdelay(5);
		vi2_activate(cam, port);
	}

	spin_lock_irq(&cam->videobuf_queue_lock);

	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.field = cam->field;
	if (port == TEGRA_CAMERA_PORT_CSI_A)
		vb->v4l2_buf.sequence = cam->sequence_a++;
	else if (port == TEGRA_CAMERA_PORT_CSI_B)
		vb->v4l2_buf.sequence = cam->sequence_b++;

	vb2_buffer_done(vb, err < 0 ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

	cam->num_frames++;

	spin_unlock_irq(&cam->videobuf_queue_lock);

	return err;
}

struct tegra_camera_ops vi2_ops = {
	.init		= vi2_ops_init,
	.deinit		= vi2_ops_deinit,
	.activate	= vi2_activate,
	.deactivate	= vi2_deactivate,
	.port_is_valid = vi2_port_is_valid,
	.capture_frame	= vi2_capture_frame,
};

int vi2_register(struct tegra_camera *cam)
{
	/* Init regulator */
	cam->regulator_name = "avdd_dsi_csi";

	/* Init VI2/CSI2 ops */
	cam->ops = &vi2_ops;

	return 0;
}
