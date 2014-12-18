/*
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

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/tegra_v4l2_camera.h>

#include <mach/clk.h>

#include "nvhost_syncpt.h"
#include "bus_client.h"
#include "common.h"

static int bypass_port_is_valid(int port)
{
	return (((port) >= TEGRA_CAMERA_PORT_CSI_A) &&
		((port) <= TEGRA_CAMERA_PORT_CSI_C));
}

/* Clock settings for camera */
static struct tegra_camera_clk bypass_clks0[] = {
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

static struct tegra_camera_clk bypass_clks1[] = {
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

static int bypass_clock_init(struct tegra_camera *cam, int port)
{
	struct platform_device *pdev = cam->pdev;
	struct tegra_camera_clk *clks;
	int i;

	switch (port) {
	case TEGRA_CAMERA_PORT_CSI_A:
		cam->num_clks = ARRAY_SIZE(bypass_clks0);
		cam->clks = bypass_clks0;
		break;
	case TEGRA_CAMERA_PORT_CSI_B:
	case TEGRA_CAMERA_PORT_CSI_C:
		cam->num_clks = ARRAY_SIZE(bypass_clks1);
		cam->clks = bypass_clks1;
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

static void bypass_clock_deinit(struct tegra_camera *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_put(clks->clk);
	}
}

static int bypass_ops_init(struct tegra_camera *cam)
{
	return 0;
}

static void bypass_ops_deinit(struct tegra_camera *cam)
{
	return;
}

static void bypass_clock_start(struct tegra_camera *cam)
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

static void bypass_clock_stop(struct tegra_camera *cam)
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

static int bypass_activate(struct tegra_camera *cam,
				      int port)
{
	/* Init Clocks */
	bypass_clock_init(cam, port);
	bypass_clock_start(cam);

	return 0;
}

static void bypass_deactivate(struct tegra_camera *cam)
{
	bypass_clock_stop(cam);
	bypass_clock_deinit(cam);

	return;
}

static int bypass_capture_frame(struct tegra_camera *cam,
			     struct tegra_camera_buffer *buf)
{
	struct vb2_buffer *vb = cam->active;

	/*
	 * TBD: This dummy function just completes a request without actually
	 * doing anything.
	 */

	spin_lock_irq(&cam->videobuf_queue_lock);

	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.field = cam->field;

	vb2_buffer_done(vb, VB2_BUF_STATE_DONE);

	spin_unlock_irq(&cam->videobuf_queue_lock);

	return 0;
}

struct tegra_camera_ops bypass_ops = {
	.init		= bypass_ops_init,
	.deinit		= bypass_ops_deinit,
	.activate	= bypass_activate,
	.deactivate	= bypass_deactivate,
	.port_is_valid = bypass_port_is_valid,
	.capture_frame	= bypass_capture_frame,
};

int bypass_register(struct tegra_camera *cam)
{
	/* Init bypass/CSI2 ops */
	cam->ops = &bypass_ops;

	return 0;
}
