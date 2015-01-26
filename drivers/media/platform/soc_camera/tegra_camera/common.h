/*
 * Copyright (c) 2012-2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_CAMERA_COMMON_H_
#define _TEGRA_CAMERA_COMMON_H_

#include <linux/videodev2.h>
#include <linux/nvhost.h>
#include <mach/powergate.h>


#include <media/videobuf2-dma-contig.h>
#include <media/soc_camera.h>

/* Buffer for one video frame */
struct tegra_camera_buffer {
	struct vb2_buffer		vb; /* v4l buffer must be first */
	struct list_head		queue;
	struct soc_camera_device	*icd;
	int				output_channel;

	/*
	 * Various buffer addresses shadowed so we don't have to recalculate
	 * per frame. These are calculated during videobuf_prepare.
	 */
	dma_addr_t			buffer_addr;
	dma_addr_t			buffer_addr_u;
	dma_addr_t			buffer_addr_v;
	dma_addr_t			start_addr;
	dma_addr_t			start_addr_u;
	dma_addr_t			start_addr_v;
};

static inline  struct tegra_camera_buffer *to_tegra_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct tegra_camera_buffer, vb);
}

struct tegra_camera;

struct tegra_camera_ops {
	int (*activate)(struct tegra_camera *cam,
		 int port);
	void (*deactivate)(struct tegra_camera *cam);
	s32 (*bytes_per_line)(u32 width, const struct soc_mbus_pixelfmt *mf);

	/*
	 * If we want to ignore the subdev and have our camera host do its
	 * own thing (for example, test pattern), then have
	 * ignore_subdev_fmt() return true and make get_formats() return
	 * the format we want to support.
	 */
	bool (*ignore_subdev_fmt)(struct tegra_camera *cam);
	int (*get_formats)(struct soc_camera_device *icd, unsigned int idx,
			   struct soc_camera_format_xlate *xlate);
	int (*try_mbus_fmt)(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *mf);
	int (*s_mbus_fmt)(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf);
	bool (*port_is_valid)(int port);
	int (*capture_frame)(struct tegra_camera *cam,
			     struct tegra_camera_buffer *buf);
};

struct cam_regs_config {
	u32 csi_base;
	u32 csi_pp_base;
	u32 cil_base;
	u32 cil_phy_base;
	u32 tpg_base;
};

struct tegra_camera {
	struct soc_camera_host		ici;

	/* These should be set prior to calling tegra_camera_probe(). */
	struct platform_device		*pdev;
	char				card[32];
	u32				version;
	struct tegra_camera_ops		*ops;

	spinlock_t			videobuf_queue_lock;
	struct list_head		capture;
	struct vb2_buffer		*active;
	struct vb2_alloc_ctx		*alloc_ctx;
	enum v4l2_field			field;

	struct work_struct		work;
	struct mutex			work_mutex;

	/* Debug */
	int				num_frames;
	int				enable_refcnt;
};

#define TC_VI_REG_RD(dev, offset) readl(dev->reg_base + offset)
#define TC_VI_REG_WT(dev, offset, val) writel(val, dev->reg_base + offset)
#define csi_regs_write(cam, offset, val) \
		TC_VI_REG_WT(cam, cam->regs.csi_base + offset, val)
#define csi_regs_read(cam, offset) \
		TC_VI_REG_RD(cam, cam->regs.csi_base + offset)
#define csi_pp_regs_write(cam, offset, val) \
		TC_VI_REG_WT(cam, cam->regs.csi_pp_base + offset, val)
#define csi_pp_regs_read(cam, offset) \
		TC_VI_REG_RD(cam, cam->regs.csi_pp_base + offset)
#define cil_regs_write(cam, offset, val) \
		TC_VI_REG_WT(cam, cam->regs.cil_base + offset, val)
#define cil_regs_read(cam, offset) \
		TC_VI_REG_RD(cam, cam->regs.cil_base + offset)
#define cil_phy_reg_write(cam, val) \
		TC_VI_REG_WT(cam, cam->regs.cil_phy_base, val)
#define cil_phy_reg_read(cam) TC_VI_REG_RD(cam, cam->regs.cil_phy_base)
#define tpg_regs_write(cam, offset, val) \
		TC_VI_REG_WT(cam, cam->regs.tpg_base + offset, val)
#define tpg_regs_read(cam, offset) \
		TC_VI_REG_RD(cam, cam->regs.tpg_base + offset)

int vi2_register(struct tegra_camera *cam);

int tegra_camera_init(struct platform_device *pdev, struct tegra_camera *cam);
void tegra_camera_deinit(struct platform_device *pdev,
			 struct tegra_camera *cam);

#endif
