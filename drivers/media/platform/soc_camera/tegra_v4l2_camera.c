/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/nvhost.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/tegra-powergate.h>

#include <mach/pm_domains.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/videobuf2-dma-contig.h>
#include <media/tegra_v4l2_camera.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_syncpt.h"
#include "t20/t20.h"
#include "t30/t30.h"
#include "t114/t114.h"

#define TEGRA_CAM_DRV_NAME "vi"
#define TEGRA_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5)

#define TEGRA_SYNCPT_VI_WAIT_TIMEOUT                    200
#define TEGRA_SYNCPT_CSI_WAIT_TIMEOUT                   200

#define TEGRA_SYNCPT_RETRY_COUNT			10

#define TEGRA_VIP_H_ACTIVE_START			0x98
#define TEGRA_VIP_V_ACTIVE_START			0x10

/* SYNCPTs 12-17 are reserved for VI. */
#define TEGRA_VI_SYNCPT_VI                              NVSYNCPT_VI_ISP_2
#define TEGRA_VI_SYNCPT_CSI_A                           NVSYNCPT_VI_ISP_3
#define TEGRA_VI_SYNCPT_CSI_B                           NVSYNCPT_VI_ISP_4

/* Tegra CSI-MIPI registers. */
#define TEGRA_VI_OUT_1_INCR_SYNCPT			0x0000
#define TEGRA_VI_OUT_1_INCR_SYNCPT_CNTRL		0x0004
#define TEGRA_VI_OUT_1_INCR_SYNCPT_ERROR		0x0008
#define TEGRA_VI_OUT_2_INCR_SYNCPT			0x0020
#define TEGRA_VI_OUT_2_INCR_SYNCPT_CNTRL		0x0024
#define TEGRA_VI_OUT_2_INCR_SYNCPT_ERROR		0x0028
#define TEGRA_VI_MISC_INCR_SYNCPT			0x0040
#define TEGRA_VI_MISC_INCR_SYNCPT_CNTRL			0x0044
#define TEGRA_VI_MISC_INCR_SYNCPT_ERROR			0x0048
#define TEGRA_VI_CONT_SYNCPT_OUT_1			0x0060
#define TEGRA_VI_CONT_SYNCPT_OUT_2			0x0064
#define TEGRA_VI_CONT_SYNCPT_VIP_VSYNC			0x0068
#define TEGRA_VI_CONT_SYNCPT_VI2EPP			0x006c
#define TEGRA_VI_CONT_SYNCPT_CSI_PPA_FRAME_START	0x0070
#define TEGRA_VI_CONT_SYNCPT_CSI_PPA_FRAME_END		0x0074
#define TEGRA_VI_CONT_SYNCPT_CSI_PPB_FRAME_START	0x0078
#define TEGRA_VI_CONT_SYNCPT_CSI_PPB_FRAME_END		0x007c
#define TEGRA_VI_CTXSW					0x0080
#define TEGRA_VI_INTSTATUS				0x0084
#define TEGRA_VI_VI_INPUT_CONTROL			0x0088
#define TEGRA_VI_VI_CORE_CONTROL			0x008c
#define TEGRA_VI_VI_FIRST_OUTPUT_CONTROL		0x0090
#define TEGRA_VI_VI_SECOND_OUTPUT_CONTROL		0x0094
#define TEGRA_VI_HOST_INPUT_FRAME_SIZE			0x0098
#define TEGRA_VI_HOST_H_ACTIVE				0x009c
#define TEGRA_VI_HOST_V_ACTIVE				0x00a0
#define TEGRA_VI_VIP_H_ACTIVE				0x00a4
#define TEGRA_VI_VIP_V_ACTIVE				0x00a8
#define TEGRA_VI_VI_PEER_CONTROL			0x00ac
#define TEGRA_VI_VI_DMA_SELECT				0x00b0
#define TEGRA_VI_HOST_DMA_WRITE_BUFFER			0x00b4
#define TEGRA_VI_HOST_DMA_BASE_ADDRESS			0x00b8
#define TEGRA_VI_HOST_DMA_WRITE_BUFFER_STATUS		0x00bc
#define TEGRA_VI_HOST_DMA_WRITE_PEND_BUFCOUNT		0x00c0
#define TEGRA_VI_VB0_START_ADDRESS_FIRST		0x00c4
#define TEGRA_VI_VB0_BASE_ADDRESS_FIRST			0x00c8
#define TEGRA_VI_VB0_START_ADDRESS_U			0x00cc
#define TEGRA_VI_VB0_BASE_ADDRESS_U			0x00d0
#define TEGRA_VI_VB0_START_ADDRESS_V			0x00d4
#define TEGRA_VI_VB0_BASE_ADDRESS_V			0x00d8
#define TEGRA_VI_VB_SCRATCH_ADDRESS_UV			0x00dc
#define TEGRA_VI_FIRST_OUTPUT_FRAME_SIZE		0x00e0
#define TEGRA_VI_VB0_COUNT_FIRST			0x00e4
#define TEGRA_VI_VB0_SIZE_FIRST				0x00e8
#define TEGRA_VI_VB0_BUFFER_STRIDE_FIRST		0x00ec
#define TEGRA_VI_VB0_START_ADDRESS_SECOND		0x00f0
#define TEGRA_VI_VB0_BASE_ADDRESS_SECOND		0x00f4
#define TEGRA_VI_SECOND_OUTPUT_FRAME_SIZE		0x00f8
#define TEGRA_VI_VB0_COUNT_SECOND			0x00fc
#define TEGRA_VI_VB0_SIZE_SECOND			0x0100
#define TEGRA_VI_VB0_BUFFER_STRIDE_SECOND		0x0104
#define TEGRA_VI_H_LPF_CONTROL				0x0108
#define TEGRA_VI_H_DOWNSCALE_CONTROL			0x010c
#define TEGRA_VI_V_DOWNSCALE_CONTROL			0x0110
#define TEGRA_VI_CSC_Y					0x0114
#define TEGRA_VI_CSC_UV_R				0x0118
#define TEGRA_VI_CSC_UV_G				0x011c
#define TEGRA_VI_CSC_UV_B				0x0120
#define TEGRA_VI_CSC_ALPHA				0x0124
#define TEGRA_VI_HOST_VSYNC				0x0128
#define TEGRA_VI_COMMAND				0x012c
#define TEGRA_VI_HOST_FIFO_STATUS			0x0130
#define TEGRA_VI_INTERRUPT_MASK				0x0134
#define TEGRA_VI_INTERRUPT_TYPE_SELECT			0x0138
#define TEGRA_VI_INTERRUPT_POLARITY_SELECT		0x013c
#define TEGRA_VI_INTERRUPT_STATUS			0x0140
#define TEGRA_VI_VIP_INPUT_STATUS			0x0144
#define TEGRA_VI_VIDEO_BUFFER_STATUS			0x0148
#define TEGRA_VI_SYNC_OUTPUT				0x014c
#define TEGRA_VI_VVS_OUTPUT_DELAY			0x0150
#define TEGRA_VI_PWM_CONTROL				0x0154
#define TEGRA_VI_PWM_SELECT_PULSE_A			0x0158
#define TEGRA_VI_PWM_SELECT_PULSE_B			0x015c
#define TEGRA_VI_PWM_SELECT_PULSE_C			0x0160
#define TEGRA_VI_PWM_SELECT_PULSE_D			0x0164
#define TEGRA_VI_VI_DATA_INPUT_CONTROL			0x0168
#define TEGRA_VI_PIN_INPUT_ENABLE			0x016c
#define TEGRA_VI_PIN_OUTPUT_ENABLE			0x0170
#define TEGRA_VI_PIN_INVERSION				0x0174
#define TEGRA_VI_PIN_INPUT_DATA				0x0178
#define TEGRA_VI_PIN_OUTPUT_DATA			0x017c
#define TEGRA_VI_PIN_OUTPUT_SELECT			0x0180
#define TEGRA_VI_RAISE_VIP_BUFFER_FIRST_OUTPUT		0x0184
#define TEGRA_VI_RAISE_VIP_FRAME_FIRST_OUTPUT		0x0188
#define TEGRA_VI_RAISE_VIP_BUFFER_SECOND_OUTPUT		0x018c
#define TEGRA_VI_RAISE_VIP_FRAME_SECOND_OUTPUT		0x0190
#define TEGRA_VI_RAISE_HOST_FIRST_OUTPUT		0x0194
#define TEGRA_VI_RAISE_HOST_SECOND_OUTPUT		0x0198
#define TEGRA_VI_RAISE_EPP				0x019c
#define TEGRA_VI_CAMERA_CONTROL				0x01a0
#define TEGRA_VI_VI_ENABLE				0x01a4
#define TEGRA_VI_VI_ENABLE_2				0x01a8
#define TEGRA_VI_VI_RAISE				0x01ac
#define TEGRA_VI_Y_FIFO_WRITE				0x01b0
#define TEGRA_VI_U_FIFO_WRITE				0x01b4
#define TEGRA_VI_V_FIFO_WRITE				0x01b8
#define TEGRA_VI_VI_MCCIF_FIFOCTRL			0x01bc
#define TEGRA_VI_TIMEOUT_WCOAL_VI			0x01c0
#define TEGRA_VI_MCCIF_VIRUV_HP				0x01c4
#define TEGRA_VI_MCCIF_VIWSB_HP				0x01c8
#define TEGRA_VI_MCCIF_VIWU_HP				0x01cc
#define TEGRA_VI_MCCIF_VIWV_HP				0x01d0
#define TEGRA_VI_MCCIF_VIWY_HP				0x01d4
#define TEGRA_VI_CSI_PPA_RAISE_FRAME_START		0x01d8
#define TEGRA_VI_CSI_PPA_RAISE_FRAME_END		0x01dc
#define TEGRA_VI_CSI_PPB_RAISE_FRAME_START		0x01e0
#define TEGRA_VI_CSI_PBB_RAISE_FRAME_END		0x01e4
#define TEGRA_VI_CSI_PPA_H_ACTIVE			0x01e8
#define TEGRA_VI_CSI_PPA_V_ACTIVE			0x01ec
#define TEGRA_VI_CSI_PPB_H_ACTIVE			0x01f0
#define TEGRA_VI_CSI_PPB_V_ACTIVE			0x01f4
#define TEGRA_VI_ISP_H_ACTIVE				0x01f8
#define TEGRA_VI_ISP_V_ACTIVE				0x01fc
#define TEGRA_VI_STREAM_1_RESOURCE_DEFINE		0x0200
#define TEGRA_VI_STREAM_2_RESOURCE_DEFINE		0x0204
#define TEGRA_VI_RAISE_STREAM_1_DONE			0x0208
#define TEGRA_VI_RAISE_STREAM_2_DONE			0x020c
#define TEGRA_VI_TS_MODE				0x0210
#define TEGRA_VI_TS_CONTROL				0x0214
#define TEGRA_VI_TS_PACKET_COUNT			0x0218
#define TEGRA_VI_TS_ERROR_COUNT				0x021c
#define TEGRA_VI_TS_CPU_FLOW_CTL			0x0220
#define TEGRA_VI_VB0_CHROMA_BUFFER_STRIDE_FIRST		0x0224
#define TEGRA_VI_VB0_CHROMA_LINE_STRIDE_FIRST		0x0228
#define TEGRA_VI_EPP_LINES_PER_BUFFER			0x022c
#define TEGRA_VI_BUFFER_RELEASE_OUTPUT1			0x0230
#define TEGRA_VI_BUFFER_RELEASE_OUTPUT2			0x0234
#define TEGRA_VI_DEBUG_FLOW_CONTROL_COUNTER_OUTPUT1	0x0238
#define TEGRA_VI_DEBUG_FLOW_CONTROL_COUNTER_OUTPUT2	0x023c
#define TEGRA_VI_TERMINATE_BW_FIRST			0x0240
#define TEGRA_VI_TERMINATE_BW_SECOND			0x0244
#define TEGRA_VI_VB0_FIRST_BUFFER_ADDR_MODE		0x0248
#define TEGRA_VI_VB0_SECOND_BUFFER_ADDR_MODE		0x024c
#define TEGRA_VI_RESERVE_0				0x0250
#define TEGRA_VI_RESERVE_1				0x0254
#define TEGRA_VI_RESERVE_2				0x0258
#define TEGRA_VI_RESERVE_3				0x025c
#define TEGRA_VI_RESERVE_4				0x0260
#define TEGRA_VI_MCCIF_VIRUV_HYST			0x0264
#define TEGRA_VI_MCCIF_VIWSB_HYST			0x0268
#define TEGRA_VI_MCCIF_VIWU_HYST			0x026c
#define TEGRA_VI_MCCIF_VIWV_HYST			0x0270
#define TEGRA_VI_MCCIF_VIWY_HYST			0x0274

#define TEGRA_CSI_VI_INPUT_STREAM_CONTROL		0x0800
#define TEGRA_CSI_HOST_INPUT_STREAM_CONTROL		0x0808
#define TEGRA_CSI_INPUT_STREAM_A_CONTROL		0x0810
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL0		0x0818
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL1		0x081c
#define TEGRA_CSI_PIXEL_STREAM_A_WORD_COUNT		0x0820
#define TEGRA_CSI_PIXEL_STREAM_A_GAP			0x0824
#define TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND		0x0828
#define TEGRA_CSI_INPUT_STREAM_B_CONTROL		0x083c
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL0		0x0844
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL1		0x0848
#define TEGRA_CSI_PIXEL_STREAM_B_WORD_COUNT		0x084c
#define TEGRA_CSI_PIXEL_STREAM_B_GAP			0x0850
#define TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND		0x0854
#define TEGRA_CSI_PHY_CIL_COMMAND			0x0868
#define TEGRA_CSI_PHY_CILA_CONTROL0			0x086c
#define TEGRA_CSI_PHY_CILB_CONTROL0			0x0870
#define TEGRA_CSI_CSI_PIXEL_PARSER_STATUS		0x0878
#define TEGRA_CSI_CSI_CIL_STATUS			0x087c
#define TEGRA_CSI_CSI_PIXEL_PARSER_INTERRUPT_MASK	0x0880
#define TEGRA_CSI_CSI_CIL_INTERRUPT_MASK		0x0884
#define TEGRA_CSI_CSI_READONLY_STATUS			0x0888
#define TEGRA_CSI_ESCAPE_MODE_COMMAND			0x088c
#define TEGRA_CSI_ESCAPE_MODE_DATA			0x0890
#define TEGRA_CSI_CILA_PAD_CONFIG0			0x0894
#define TEGRA_CSI_CILA_PAD_CONFIG1			0x0898
#define TEGRA_CSI_CILB_PAD_CONFIG0			0x089c
#define TEGRA_CSI_CILB_PAD_CONFIG1			0x08a0
#define TEGRA_CSI_CIL_PAD_CONFIG			0x08a4
#define TEGRA_CSI_CILA_MIPI_CAL_CONFIG			0x08a8
#define TEGRA_CSI_CILB_MIPI_CAL_CONFIG			0x08ac
#define TEGRA_CSI_CIL_MIPI_CAL_STATUS			0x08b0
#define TEGRA_CSI_CLKEN_OVERRIDE			0x08b4
#define TEGRA_CSI_DEBUG_CONTROL				0x08b8
#define TEGRA_CSI_DEBUG_COUNTER_0			0x08bc
#define TEGRA_CSI_DEBUG_COUNTER_1			0x08c0
#define TEGRA_CSI_DEBUG_COUNTER_2			0x08c4
#define TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME		0x08c8
#define TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME		0x08cc
#define TEGRA_CSI_DSI_MIPI_CAL_CONFIG			0x08d0

#define TC_VI_REG_RD(DEV, REG) readl(DEV->vi_base + REG)
#define TC_VI_REG_WT(DEV, REG, VAL) writel(VAL, DEV->vi_base + REG)

#define tegra_camera_port_is_valid(port) \
	(((port) >= TEGRA_CAMERA_PORT_CSI_A) && \
	 ((port) <= TEGRA_CAMERA_PORT_VIP))

#define tegra_camera_port_is_csi(port) \
	(((port) == TEGRA_CAMERA_PORT_CSI_A) || \
	 ((port) == TEGRA_CAMERA_PORT_CSI_B))

/*
 * Structures
 */

/* buffer for one video frame */
struct tegra_buffer {
	struct vb2_buffer		vb; /* v4l buffer must be first */
	struct list_head		queue;
	struct soc_camera_device	*icd;
	int				output_channel;

	/*
	 * Various buffer addresses shadowed so we don't have to recalculate
	 * per frame.  These are calculated during videobuf_prepare.
	 */
	dma_addr_t			buffer_addr;
	dma_addr_t			buffer_addr_u;
	dma_addr_t			buffer_addr_v;
	dma_addr_t			start_addr;
	dma_addr_t			start_addr_u;
	dma_addr_t			start_addr_v;
};

struct tegra_camera_dev {
	struct soc_camera_host		ici;
	struct platform_device		*ndev;
	struct nvhost_device_data	*ndata;

	struct clk			*clk_vi;
	struct clk			*clk_vi_sensor;
	struct clk			*clk_csi;
	struct clk			*clk_isp;
	struct clk			*clk_csus;
	struct clk			*clk_sclk;
	struct clk			*clk_emc;

	struct regulator		*reg;

	void __iomem			*vi_base;
	spinlock_t			videobuf_queue_lock;
	struct list_head		capture;
	struct vb2_buffer		*active;
	struct vb2_alloc_ctx		*alloc_ctx;
	enum v4l2_field			field;
	int				sequence_a;
	int				sequence_b;

	struct work_struct		work;
	struct mutex			work_mutex;

	u32				syncpt_vi;
	u32				syncpt_csi_a;
	u32				syncpt_csi_b;

	/* Debug */
	int num_frames;
	int enable_refcnt;
};

static const struct soc_mbus_pixelfmt tegra_camera_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.name			= "YUV422 (UYVY) packed",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_VYUY,
		.name			= "YUV422 (VYUY) packed",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.name			= "YUV422 (YUYV) packed",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_YVYU,
		.name			= "YUV422 (YVYU) packed",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_YUV420,
		.name			= "YUV420 (YU12) planar",
		.bits_per_sample	= 12,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_YVU420,
		.name			= "YVU420 (YV12) planar",
		.bits_per_sample	= 12,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},

	/* For RAW8 and RAW10 output, we always output 16-bit (2 bytes). */
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR8,
		.name			= "Bayer 8 BGBG.. GRGR..",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_EXTEND16,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR10,
		.name			= "Bayer 10 BGBG.. GRGR..",
		.bits_per_sample	= 16,
		.packing		= SOC_MBUS_PACKING_EXTEND16,
		.order			= SOC_MBUS_ORDER_LE,
	},

};

static struct tegra_buffer *to_tegra_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct tegra_buffer, vb);
}

static void tegra_camera_save_syncpts(struct tegra_camera_dev *pcdev)
{
	pcdev->syncpt_csi_a =
		nvhost_syncpt_read_ext(pcdev->ndev,
				       TEGRA_VI_SYNCPT_CSI_A);

	pcdev->syncpt_csi_b =
		nvhost_syncpt_read_ext(pcdev->ndev,
				       TEGRA_VI_SYNCPT_CSI_B);

	pcdev->syncpt_vi =
		nvhost_syncpt_read_ext(pcdev->ndev,
				       TEGRA_VI_SYNCPT_VI);
}

static void tegra_camera_incr_syncpts(struct tegra_camera_dev *pcdev)
{
	nvhost_syncpt_cpu_incr_ext(pcdev->ndev,
				   TEGRA_VI_SYNCPT_CSI_A);

	nvhost_syncpt_cpu_incr_ext(pcdev->ndev,
				   TEGRA_VI_SYNCPT_CSI_B);

	nvhost_syncpt_cpu_incr_ext(pcdev->ndev,
				   TEGRA_VI_SYNCPT_VI);
}

static void tegra_camera_capture_clean(struct tegra_camera_dev *pcdev)
{
	TC_VI_REG_WT(pcdev, TEGRA_CSI_VI_INPUT_STREAM_CONTROL, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_HOST_INPUT_STREAM_CONTROL, 0x00000000);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_CSI_PIXEL_PARSER_STATUS, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CSI_CIL_STATUS, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CSI_PIXEL_PARSER_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CSI_CIL_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CSI_READONLY_STATUS, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_ESCAPE_MODE_COMMAND, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_ESCAPE_MODE_DATA, 0x0);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_CIL_PAD_CONFIG, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CIL_MIPI_CAL_STATUS, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CLKEN_OVERRIDE, 0x00000000);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_DEBUG_CONTROL, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_DEBUG_COUNTER_0, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_DEBUG_COUNTER_1, 0x0);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_DEBUG_COUNTER_2, 0x0);
}

static void tegra_camera_capture_setup_csi_a(struct tegra_camera_dev *pcdev,
					     struct soc_camera_device *icd,
					     u32 hdr)
{
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_INPUT_STREAM_A_CONTROL, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_CONTROL0, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_CONTROL1, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_WORD_COUNT, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_GAP, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PHY_CILA_CONTROL0, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CILA_PAD_CONFIG0, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CILA_PAD_CONFIG1, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CILA_MIPI_CAL_CONFIG, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME, 0x0);

	TC_VI_REG_WT(pcdev, TEGRA_VI_VI_CORE_CONTROL, 0x02000000);

	/* CSI-A H_ACTIVE and V_ACTIVE */
	TC_VI_REG_WT(pcdev, TEGRA_VI_CSI_PPA_H_ACTIVE,
		     (icd->user_width << 16));
	TC_VI_REG_WT(pcdev, TEGRA_VI_CSI_PPA_V_ACTIVE,
		     (icd->user_height << 16));

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_CONTROL1,
		0x1); /* Frame # for top field detect for interlaced */

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_WORD_COUNT,
		bytes_per_line);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_GAP, 0x00140000);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME,
		(icd->user_height << 16) |
		(0x100 << 4) | /* Wait 0x100 vi clks for timeout */
		0x1); /* Enable line timeout */

	/* pad 0s enabled, virtual channel ID 00 */
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_A_CONTROL0,
		(0x1 << 16) | /* Output 1 pixel per clock */
		(hdr << 8) | /* If hdr shows wrong fmt, use right value */
		(0x1 << 7) | /* Check header CRC */
		(0x1 << 6) | /* Use word count field in the header */
		(0x1 << 5) | /* Look at data identifier byte in hdr */
		(0x1 << 4));  /* Expect packet header */

	TC_VI_REG_WT(pcdev, TEGRA_CSI_INPUT_STREAM_A_CONTROL,
		     (0x3f << 16) | /* Skip packet threshold */
		     (pdata->lanes - 1));

	/* Use 0x00000022 for continuous clock mode. */
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PHY_CILA_CONTROL0,
		(pdata->continuous_clk << 5) |
		0x5); /* Clock settle time */

	TC_VI_REG_WT(pcdev, TEGRA_VI_CONT_SYNCPT_CSI_PPA_FRAME_END,
		(0x1 << 8) | /* Enable continuous syncpt */
		TEGRA_VI_SYNCPT_CSI_A);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PHY_CIL_COMMAND, 0x00020001);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0x0000f002);
}

static void tegra_camera_capture_setup_csi_b(struct tegra_camera_dev *pcdev,
					     struct soc_camera_device *icd,
					     u32 hdr)
{
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_INPUT_STREAM_B_CONTROL, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_CONTROL0, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_CONTROL1, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_WORD_COUNT, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_GAP, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PHY_CILB_CONTROL0, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CILB_PAD_CONFIG0, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CILB_PAD_CONFIG1, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_CILB_MIPI_CAL_CONFIG, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME, 0x0);

	TC_VI_REG_WT(pcdev, TEGRA_VI_VI_CORE_CONTROL, 0x04000000);

	/* CSI-B H_ACTIVE and V_ACTIVE */
	TC_VI_REG_WT(pcdev, TEGRA_VI_CSI_PPB_H_ACTIVE,
		(icd->user_width << 16));
	TC_VI_REG_WT(pcdev, TEGRA_VI_CSI_PPB_V_ACTIVE,
		(icd->user_height << 16));

	/* pad 0s enabled, virtual channel ID 00 */
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_CONTROL0,
		(0x1 << 16) | /* Output 1 pixel per clock */
		(hdr << 8) | /* If hdr shows wrong fmt, use right value */
		(0x1 << 7) | /* Check header CRC */
		(0x1 << 6) | /* Use word count field in the header */
		(0x1 << 5) | /* Look at data identifier byte in hdr */
		(0x1 << 4) | /* Expect packet header */
		0x1); /* Set PPB stream source to CSI B */

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_CONTROL1,
		0x1); /* Frame # for top field detect for interlaced */

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_WORD_COUNT,
		bytes_per_line);
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_GAP, 0x00140000);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME,
		(icd->user_height << 16) |
		(0x100 << 4) | /* Wait 0x100 vi clks for timeout */
		0x1); /* Enable line timeout */

	TC_VI_REG_WT(pcdev, TEGRA_CSI_INPUT_STREAM_B_CONTROL,
		     (0x3f << 16) | /* Skip packet threshold */
		     (pdata->lanes - 1));

	/* Use 0x00000022 for continuous clock mode. */
	TC_VI_REG_WT(pcdev, TEGRA_CSI_PHY_CILB_CONTROL0,
		(pdata->continuous_clk << 5) |
		0x5); /* Clock settle time */

	TC_VI_REG_WT(pcdev, TEGRA_VI_CONT_SYNCPT_CSI_PPB_FRAME_END,
		(0x1 << 8) | /* Enable continuous syncpt */
		TEGRA_VI_SYNCPT_CSI_B);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PHY_CIL_COMMAND, 0x00010002);

	TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0x0000f002);
}

static void tegra_camera_capture_setup_vip(struct tegra_camera_dev *pcdev,
					   struct soc_camera_device *icd,
					   u32 input_control)
{

	TC_VI_REG_WT(pcdev, TEGRA_VI_VI_CORE_CONTROL, 0x00000000);

	TC_VI_REG_WT(pcdev, TEGRA_VI_VI_INPUT_CONTROL,
		(1 << 27) | /* field detect */
		(1 << 25) | /* hsync/vsync decoded from data (BT.656) */
		(1 << 1) | /* VIP_INPUT_ENABLE */
		input_control);

	TC_VI_REG_WT(pcdev, TEGRA_VI_H_DOWNSCALE_CONTROL, 0x00000000);
	TC_VI_REG_WT(pcdev, TEGRA_VI_V_DOWNSCALE_CONTROL, 0x00000000);

	/* VIP H_ACTIVE and V_ACTIVE */
	TC_VI_REG_WT(pcdev, TEGRA_VI_VIP_H_ACTIVE,
		(icd->user_width << 16) |
		TEGRA_VIP_H_ACTIVE_START);
	TC_VI_REG_WT(pcdev, TEGRA_VI_VIP_V_ACTIVE,
		(icd->user_height << 16) |
		TEGRA_VIP_V_ACTIVE_START);

	/*
	 * For VIP, D9..D2 is mapped to the video decoder's P7..P0.
	 * Disable/mask out the other Dn wires.
	 */
	TC_VI_REG_WT(pcdev, TEGRA_VI_PIN_INPUT_ENABLE, 0x000003fc);
	TC_VI_REG_WT(pcdev, TEGRA_VI_VI_DATA_INPUT_CONTROL, 0x000003fc);
	TC_VI_REG_WT(pcdev, TEGRA_VI_PIN_INVERSION, 0x00000000);

	TC_VI_REG_WT(pcdev, TEGRA_VI_CONT_SYNCPT_VIP_VSYNC,
		(0x1 << 8) | /* Enable continuous syncpt */
		TEGRA_VI_SYNCPT_VI);

	TC_VI_REG_WT(pcdev, TEGRA_VI_CAMERA_CONTROL, 0x00000004);
}

static int tegra_camera_capture_output_channel_setup(
		struct tegra_camera_dev *pcdev,
		struct soc_camera_device *icd)
{
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	int port = pdata->port;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);
	const struct soc_camera_format_xlate *current_fmt = icd->current_fmt;
	u32 output_fourcc = current_fmt->host_fmt->fourcc;
	u32 output_format, output_control;
	struct tegra_buffer *buf = to_tegra_vb(pcdev->active);

	switch (output_fourcc) {
	case V4L2_PIX_FMT_UYVY:
		output_format = 0x3; /* Default to YUV422 */
		break;
	case V4L2_PIX_FMT_VYUY:
		output_format = (0x1 << 17) | 0x3;
		break;
	case V4L2_PIX_FMT_YUYV:
		output_format = (0x2 << 17) | 0x3;
		break;
	case V4L2_PIX_FMT_YVYU:
		output_format = (0x3 << 17) | 0x3;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		output_format = 0x6; /* YUV420 planar */
		break;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SBGGR10:
		/* Use second output channel for RAW8/RAW10 */
		buf->output_channel = 1;

		if (port == TEGRA_CAMERA_PORT_CSI_A)
			output_format = 0x7;
		else if (port == TEGRA_CAMERA_PORT_CSI_B)
			output_format = 0x8;
		else
			output_format = 0x9;
		break;
	default:
		dev_err(&pcdev->ndev->dev, "Wrong output format %d\n",
			output_fourcc);
		return -EINVAL;
	}

	output_control = (pdata->flip_v ? (0x1 << 20) : 0) |
			(pdata->flip_h ? (0x1 << 19) : 0) |
			output_format;

	if (buf->output_channel == 0) {
		TC_VI_REG_WT(pcdev, TEGRA_VI_VI_FIRST_OUTPUT_CONTROL,
				output_control);
		/*
		 * Set up frame size.  Bits 31:16 are the number of lines, and
		 * bits 15:0 are the number of pixels per line.
		 */
		TC_VI_REG_WT(pcdev, TEGRA_VI_FIRST_OUTPUT_FRAME_SIZE,
				(icd->user_height << 16) | icd->user_width);

		/* First output memory enabled */
		TC_VI_REG_WT(pcdev, TEGRA_VI_VI_ENABLE, 0x00000000);

		/* Set the number of frames in the buffer. */
		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_COUNT_FIRST, 0x00000001);

		/* Set up buffer frame size. */
		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_SIZE_FIRST,
				(icd->user_height << 16) | icd->user_width);

		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_BUFFER_STRIDE_FIRST,
				(icd->user_height * bytes_per_line));

		TC_VI_REG_WT(pcdev, TEGRA_VI_CONT_SYNCPT_OUT_1,
				(0x1 << 8) | /* Enable continuous syncpt */
				TEGRA_VI_SYNCPT_VI);

		TC_VI_REG_WT(pcdev, TEGRA_VI_VI_ENABLE, 0x00000000);
	} else if (buf->output_channel == 1) {
		TC_VI_REG_WT(pcdev, TEGRA_VI_VI_SECOND_OUTPUT_CONTROL,
				output_control);

		TC_VI_REG_WT(pcdev, TEGRA_VI_SECOND_OUTPUT_FRAME_SIZE,
				(icd->user_height << 16) | icd->user_width);

		TC_VI_REG_WT(pcdev, TEGRA_VI_VI_ENABLE_2, 0x00000000);

		/* Set the number of frames in the buffer. */
		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_COUNT_SECOND, 0x00000001);

		/* Set up buffer frame size. */
		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_SIZE_SECOND,
				(icd->user_height << 16) | icd->user_width);

		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_BUFFER_STRIDE_SECOND,
				(icd->user_height * bytes_per_line));

		TC_VI_REG_WT(pcdev, TEGRA_VI_CONT_SYNCPT_OUT_2,
				(0x1 << 8) | /* Enable continuous syncpt */
				TEGRA_VI_SYNCPT_VI);

		TC_VI_REG_WT(pcdev, TEGRA_VI_VI_ENABLE_2, 0x00000000);
	} else {
		dev_err(&pcdev->ndev->dev, "Wrong output channel %d\n",
			buf->output_channel);
		return -EINVAL;
	}

	return 0;
}

static int tegra_camera_capture_setup(struct tegra_camera_dev *pcdev)
{
	struct vb2_buffer *vb = pcdev->active;
	struct tegra_buffer *buf = to_tegra_vb(vb);
	struct soc_camera_device *icd = buf->icd;
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	int port = pdata->port;
	const struct soc_camera_format_xlate *current_fmt = icd->current_fmt;
	enum v4l2_mbus_pixelcode input_code = current_fmt->code;
	u32 hdr, input_control = 0x0;

	switch (input_code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		input_control |= 0x2 << 8;
		hdr = 30;
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8:
		input_control |= 0x3 << 8;
		hdr = 30;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		input_control |= 0x0;
		hdr = 30;
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8:
		input_control |= 0x1 << 8;
		hdr = 30;
		break;
	case V4L2_MBUS_FMT_SBGGR8_1X8:
		input_control |= 0x2 << 2;	/* Input Format = Bayer */
		hdr = 42;
		break;
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		input_control |= 0x2 << 2;	/* Input Format = Bayer */
		hdr = 43;
		break;
	default:
		dev_err(&pcdev->ndev->dev, "Input format %d is not supported\n",
			input_code);
		return -EINVAL;
	}

	/*
	 * Set up low pass filter.  Use 0x240 for chromaticity and 0x240
	 * for luminance, which is the default and means not to touch
	 * anything.
	 */
	TC_VI_REG_WT(pcdev, TEGRA_VI_H_LPF_CONTROL, 0x02400240);

	/* Set up raise-on-edge, so we get an interrupt on end of frame. */
	TC_VI_REG_WT(pcdev, TEGRA_VI_VI_RAISE, 0x00000001);

	/* Cleanup registers */
	tegra_camera_capture_clean(pcdev);

	/* Setup registers for CSI-A, CSI-B and VIP inputs */
	if (port == TEGRA_CAMERA_PORT_CSI_A)
		tegra_camera_capture_setup_csi_a(pcdev, icd, hdr);
	else if (port == TEGRA_CAMERA_PORT_CSI_B)
		tegra_camera_capture_setup_csi_b(pcdev, icd, hdr);
	else
		tegra_camera_capture_setup_vip(pcdev, icd, input_control);

	/* Setup registers for output channels */
	return tegra_camera_capture_output_channel_setup(pcdev, icd);
}

static int tegra_camera_capture_buffer_setup(struct tegra_camera_dev *pcdev,
			struct tegra_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;

	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_BASE_ADDRESS_U,
			     buf->buffer_addr_u);
		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_START_ADDRESS_U,
			     buf->start_addr_u);

		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_BASE_ADDRESS_V,
			     buf->buffer_addr_v);
		TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_START_ADDRESS_V,
			     buf->start_addr_v);

	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SBGGR10:
		/* output 1 */
		if (buf->output_channel == 0) {
			TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_BASE_ADDRESS_FIRST,
					buf->buffer_addr);
			TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_START_ADDRESS_FIRST,
					buf->start_addr);
		/* output 2 */
		} else if (buf->output_channel == 1) {
			TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_BASE_ADDRESS_SECOND,
					buf->buffer_addr);
			TC_VI_REG_WT(pcdev, TEGRA_VI_VB0_START_ADDRESS_SECOND,
					buf->start_addr);
		} else {
			dev_err(&pcdev->ndev->dev, "Wrong output channel %d\n",
				buf->output_channel);
			return -EINVAL;
		}
	break;

	default:
		dev_err(&pcdev->ndev->dev, "Wrong host format %d\n",
			icd->current_fmt->host_fmt->fourcc);
		return -EINVAL;
	}

	return 0;
}

static int tegra_camera_capture_start(struct tegra_camera_dev *pcdev,
				      struct tegra_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	int port = pdata->port;
	int err;

	err = tegra_camera_capture_buffer_setup(pcdev, buf);
	if (err < 0)
		return err;
	/*
	 * Only wait on CSI frame end syncpt if we're using CSI.  Otherwise,
	 * wait on VIP VSYNC syncpt.
	 */
	if (port == TEGRA_CAMERA_PORT_CSI_A) {
		pcdev->syncpt_csi_a++;
		TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND,
				0x0000f005);
		err = nvhost_syncpt_wait_timeout_ext(pcdev->ndev,
				TEGRA_VI_SYNCPT_CSI_A,
				pcdev->syncpt_csi_a,
				TEGRA_SYNCPT_CSI_WAIT_TIMEOUT,
				NULL,
				NULL);
	} else if (port == TEGRA_CAMERA_PORT_CSI_B) {
		pcdev->syncpt_csi_b++;
		TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND,
				0x0000f005);
		err = nvhost_syncpt_wait_timeout_ext(pcdev->ndev,
				TEGRA_VI_SYNCPT_CSI_B,
				pcdev->syncpt_csi_b,
				TEGRA_SYNCPT_CSI_WAIT_TIMEOUT,
				NULL,
				NULL);
	} else {
		pcdev->syncpt_vi++;
		TC_VI_REG_WT(pcdev, TEGRA_VI_CAMERA_CONTROL,
				0x00000001);
		err = nvhost_syncpt_wait_timeout_ext(pcdev->ndev,
				TEGRA_VI_SYNCPT_VI,
				pcdev->syncpt_csi_a,
				TEGRA_SYNCPT_VI_WAIT_TIMEOUT,
				NULL,
				NULL);
	}

	if (!err)
		return 0;

	if (tegra_camera_port_is_csi(port)) {
		u32 ppstatus;
		u32 cilstatus;
		u32 rostatus;

		dev_warn(&icd->vdev->dev, "Timeout on CSI syncpt\n");
		dev_warn(&icd->vdev->dev, "buffer_addr = 0x%08x\n",
			buf->buffer_addr);

		ppstatus = TC_VI_REG_RD(pcdev,
			TEGRA_CSI_CSI_PIXEL_PARSER_STATUS);
		cilstatus = TC_VI_REG_RD(pcdev,
			 TEGRA_CSI_CSI_CIL_STATUS);
		rostatus = TC_VI_REG_RD(pcdev,
			TEGRA_CSI_CSI_READONLY_STATUS);

		dev_warn(&icd->vdev->dev,
			"PPSTATUS = 0x%08x, "
			"CILSTATUS = 0x%08x, "
			"ROSTATUS = 0x%08x\n",
			ppstatus, cilstatus, rostatus);
	} else {
		u32 vip_input_status;

		dev_warn(&pcdev->ndev->dev, "Timeout on VI syncpt\n");
		dev_warn(&pcdev->ndev->dev, "buffer_addr = 0x%08x\n",
			buf->buffer_addr);

		vip_input_status = TC_VI_REG_RD(pcdev,
			TEGRA_VI_VIP_INPUT_STATUS);

		dev_warn(&pcdev->ndev->dev,
			"VIP_INPUT_STATUS = 0x%08x\n",
			vip_input_status);
	}

	return err;
}

static int tegra_camera_capture_stop(struct tegra_camera_dev *pcdev, int port)
{
	int err;
	struct tegra_buffer *buf = to_tegra_vb(pcdev->active);

	if (port == TEGRA_CAMERA_PORT_CSI_A)
		TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND,
			     0x0000f002);
	else if (port == TEGRA_CAMERA_PORT_CSI_B)
		TC_VI_REG_WT(pcdev, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND,
			     0x0000f002);
	else
		TC_VI_REG_WT(pcdev, TEGRA_VI_CAMERA_CONTROL,
			     0x00000005);

	if (tegra_camera_port_is_csi(port))
		err = nvhost_syncpt_wait_timeout_ext(pcdev->ndev,
			TEGRA_VI_SYNCPT_VI,
			pcdev->syncpt_vi,
			TEGRA_SYNCPT_VI_WAIT_TIMEOUT,
			NULL,
			NULL);
	else
		err = 0;

	if (err) {
		u32 buffer_addr;
		u32 ppstatus;
		u32 cilstatus;

		dev_warn(&pcdev->ndev->dev, "Timeout on VI syncpt\n");

		if (buf->output_channel == 0)
			buffer_addr = TC_VI_REG_RD(pcdev,
					   TEGRA_VI_VB0_BASE_ADDRESS_FIRST);
		else if (buf->output_channel == 1)
			buffer_addr = TC_VI_REG_RD(pcdev,
					   TEGRA_VI_VB0_BASE_ADDRESS_SECOND);
		else {
			dev_err(&pcdev->ndev->dev, "Wrong output channel %d\n",
				buf->output_channel);
			return -EINVAL;
		}

		dev_warn(&pcdev->ndev->dev, "buffer_addr = 0x%08x\n",
			buffer_addr);

		ppstatus = TC_VI_REG_RD(pcdev,
					TEGRA_CSI_CSI_PIXEL_PARSER_STATUS);
		cilstatus = TC_VI_REG_RD(pcdev,
					 TEGRA_CSI_CSI_CIL_STATUS);
		dev_warn(&pcdev->ndev->dev,
			"PPSTATUS = 0x%08x, CILSTATUS = 0x%08x\n",
			ppstatus, cilstatus);
	}

	return err;
}

static void tegra_camera_activate(struct tegra_camera_dev *pcdev)
{
		nvhost_module_busy_ext(pcdev->ndev);

	/* Enable external power */
	regulator_enable(pcdev->reg);

	/*
	 * Powergating DIS must powergate VE partition. Camera
	 * module needs to increase the ref-count of disa to
	 * avoid itself powergated by DIS inadvertently.
	 */
#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
	tegra_unpowergate_partition(TEGRA_POWERGATE_DISA);
#endif
	/* Unpowergate VE */
	tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);

	/* Turn on relevant clocks. */
	clk_set_rate(pcdev->clk_vi, 150000000);
	clk_prepare_enable(pcdev->clk_vi);
	clk_set_rate(pcdev->clk_vi_sensor, 24000000);
	clk_prepare_enable(pcdev->clk_vi_sensor);
	clk_prepare_enable(pcdev->clk_csi);
	clk_prepare_enable(pcdev->clk_isp);
	clk_prepare_enable(pcdev->clk_csus);
	clk_set_rate(pcdev->clk_sclk, 80000000);
	clk_prepare_enable(pcdev->clk_sclk);
	clk_set_rate(pcdev->clk_sclk, 375000000);
	clk_prepare_enable(pcdev->clk_emc);

	/* Save current syncpt values. */
	tegra_camera_save_syncpts(pcdev);
}

static void tegra_camera_deactivate(struct tegra_camera_dev *pcdev)
{
	/* Turn off relevant clocks. */
	clk_disable_unprepare(pcdev->clk_vi);
	clk_disable_unprepare(pcdev->clk_vi_sensor);
	clk_disable_unprepare(pcdev->clk_csi);
	clk_disable_unprepare(pcdev->clk_isp);
	clk_disable_unprepare(pcdev->clk_csus);
	clk_disable_unprepare(pcdev->clk_sclk);
	clk_disable_unprepare(pcdev->clk_emc);

	/* Powergate VE */
	tegra_powergate_partition(TEGRA_POWERGATE_VENC);
#if defined(CONFIG_ARCH_TEGRA_11x_SOC) || defined(CONFIG_ARCH_TEGRA_14x_SOC)
	tegra_powergate_partition(TEGRA_POWERGATE_DISA);
#endif

	/* Disable external power */
	regulator_disable(pcdev->reg);

	nvhost_module_idle_ext(pcdev->ndev);
}

static int tegra_camera_capture_frame(struct tegra_camera_dev *pcdev)
{
	struct vb2_buffer *vb = pcdev->active;
	struct tegra_buffer *buf = to_tegra_vb(vb);
	struct soc_camera_device *icd = buf->icd;
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	int port = pdata->port;
	int retry = TEGRA_SYNCPT_RETRY_COUNT;
	int err;

	while (retry) {
		err = tegra_camera_capture_start(pcdev, buf);
		/* Capturing succeed, stop capturing */
		if (!err)
			err = tegra_camera_capture_stop(pcdev, port);
		/* Capturing failed, stop and retry */
		else {
			retry--;

			/* Stop streaming. */
			if (port == TEGRA_CAMERA_PORT_CSI_A) {
				TC_VI_REG_WT(pcdev,
					     TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND,
					     0x0000f002);
				/* Clear status registers. */
				TC_VI_REG_WT(pcdev,
					     TEGRA_CSI_CSI_PIXEL_PARSER_STATUS,
					     0xffffffff);
				TC_VI_REG_WT(pcdev,
					     TEGRA_CSI_CSI_CIL_STATUS,
					     0xffffffff);
			} else if (port == TEGRA_CAMERA_PORT_CSI_B) {
				TC_VI_REG_WT(pcdev,
					     TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND,
					     0x0000f002);
				/* Clear status registers. */
				TC_VI_REG_WT(pcdev,
					     TEGRA_CSI_CSI_PIXEL_PARSER_STATUS,
					     0xffffffff);
				TC_VI_REG_WT(pcdev,
					     TEGRA_CSI_CSI_CIL_STATUS,
					     0xffffffff);
			} else {
				TC_VI_REG_WT(pcdev,
					     TEGRA_VI_CAMERA_CONTROL,
					     0x00000005);
			}

			tegra_camera_incr_syncpts(pcdev);
			tegra_camera_save_syncpts(pcdev);

			continue;
		}

		break;
	}

	/* Reset hardware for too many errors */
	if (!retry) {
		tegra_camera_deactivate(pcdev);
		mdelay(5);
		tegra_camera_activate(pcdev);
		if (pcdev->active)
			tegra_camera_capture_setup(pcdev);
	}

	spin_lock_irq(&pcdev->videobuf_queue_lock);

	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.field = pcdev->field;
	if (port == TEGRA_CAMERA_PORT_CSI_A)
		vb->v4l2_buf.sequence = pcdev->sequence_a++;
	else if (port == TEGRA_CAMERA_PORT_CSI_B)
		vb->v4l2_buf.sequence = pcdev->sequence_b++;

	vb2_buffer_done(vb, err < 0 ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	list_del_init(&buf->queue);

	pcdev->num_frames++;

	spin_unlock_irq(&pcdev->videobuf_queue_lock);

	return err;
}

static void tegra_camera_work(struct work_struct *work)
{
	struct tegra_camera_dev *pcdev =
		container_of(work, struct tegra_camera_dev, work);
	struct tegra_buffer *buf;

	while (1) {
		mutex_lock(&pcdev->work_mutex);

		spin_lock_irq(&pcdev->videobuf_queue_lock);
		if (list_empty(&pcdev->capture)) {
			pcdev->active = NULL;
			spin_unlock_irq(&pcdev->videobuf_queue_lock);
			mutex_unlock(&pcdev->work_mutex);
			return;
		}

		buf = list_entry(pcdev->capture.next, struct tegra_buffer,
				queue);
		pcdev->active = &buf->vb;
		spin_unlock_irq(&pcdev->videobuf_queue_lock);

		tegra_camera_capture_setup(pcdev);
		tegra_camera_capture_frame(pcdev);

		mutex_unlock(&pcdev->work_mutex);
	}
}

static int tegra_camera_init_buffer(struct tegra_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);
	struct tegra_camera_platform_data *pdata = icd->link->priv;

	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SBGGR10:
		buf->buffer_addr = vb2_dma_contig_plane_dma_addr(&buf->vb, 0);
		buf->start_addr = buf->buffer_addr;

		if (pdata->flip_v)
			buf->start_addr += bytes_per_line *
					   (icd->user_height-1);

		if (pdata->flip_h)
			buf->start_addr += bytes_per_line - 1;

		break;

	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		buf->buffer_addr = vb2_dma_contig_plane_dma_addr(&buf->vb, 0);
		buf->buffer_addr_u = buf->buffer_addr +
				     icd->user_width * icd->user_height;
		buf->buffer_addr_v = buf->buffer_addr_u +
				     (icd->user_width * icd->user_height) / 4;

		/* For YVU420, we swap the locations of the U and V planes. */
		if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_YVU420) {
			dma_addr_t temp = buf->buffer_addr_u;
			buf->buffer_addr_u = buf->buffer_addr_v;
			buf->buffer_addr_v = temp;
		}

		buf->start_addr = buf->buffer_addr;
		buf->start_addr_u = buf->buffer_addr_u;
		buf->start_addr_v = buf->buffer_addr_v;

		if (pdata->flip_v) {
			buf->start_addr += icd->user_width *
					   (icd->user_height - 1);

			buf->start_addr_u += ((icd->user_width/2) *
					      ((icd->user_height/2) - 1));

			buf->start_addr_v += ((icd->user_width/2) *
					      ((icd->user_height/2) - 1));
		}

		if (pdata->flip_h) {
			buf->start_addr += icd->user_width - 1;

			buf->start_addr_u += (icd->user_width/2) - 1;

			buf->start_addr_v += (icd->user_width/2) - 1;
		}

		break;

	default:
		dev_err(icd->parent, "Wrong host format %d\n",
			icd->current_fmt->host_fmt->fourcc);
		return -EINVAL;
	}

	return 0;
}

/*
 *  Videobuf operations
 */
static int tegra_camera_videobuf_setup(struct vb2_queue *vq,
				       const struct v4l2_format *fmt,
				       unsigned int *num_buffers,
				       unsigned int *num_planes,
				       unsigned int sizes[],
				       void *alloc_ctxs[])
{
	struct soc_camera_device *icd = container_of(vq,
						     struct soc_camera_device,
						     vb2_vidq);
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct tegra_camera_dev *pcdev = ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);
	if (bytes_per_line < 0)
		return bytes_per_line;

	*num_planes = 1;

	if (pdata->port == TEGRA_CAMERA_PORT_CSI_A)
		pcdev->sequence_a = 0;
	else if (pdata->port == TEGRA_CAMERA_PORT_CSI_B)
		pcdev->sequence_b = 0;
	sizes[0] = bytes_per_line * icd->user_height;
	alloc_ctxs[0] = pcdev->alloc_ctx;

	if (!*num_buffers)
		*num_buffers = 2;

	return 0;
}

static int tegra_camera_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
						     struct soc_camera_device,
						     vb2_vidq);
	struct tegra_buffer *buf = to_tegra_vb(vb);
	struct tegra_camera_platform_data *pdata = icd->link->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);
	unsigned long size;

	if (bytes_per_line < 0)
		return bytes_per_line;

	buf->icd = icd;

	if (!pdata) {
		dev_err(icd->parent, "No platform data for this device!\n");
		return -EINVAL;
	}

	if (!tegra_camera_port_is_valid(pdata->port)) {
		dev_err(icd->parent,
			"Invalid camera port %d in platform data\n",
			pdata->port);
		return -EINVAL;
	}

	dev_dbg(icd->parent, "%s (vb=0x%p) 0x%p %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_plane_size(vb, 0));

#ifdef PREFILL_BUFFER
	/*
	 * This can be useful if you want to see if we actually fill
	 * the buffer with something
	 */
	if (vb2_plane_vaddr(vb, 0))
		memset(vb2_plane_vaddr(vb, 0), 0xbd, vb2_plane_size(vb, 0));
#endif

	if (!icd->current_fmt) {
		dev_err(icd->parent, "%s NULL format point\n", __func__);
		return -EINVAL;
	}

	size = icd->user_height * bytes_per_line;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(icd->parent, "Buffer too small (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -ENOBUFS;
	}

	vb2_set_plane_payload(vb, 0, size);

	return tegra_camera_init_buffer(buf);
}

static void tegra_camera_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
						     struct soc_camera_device,
						     vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct tegra_camera_dev *pcdev = ici->priv;
	struct tegra_buffer *buf = to_tegra_vb(vb);

	dev_dbg(icd->parent, "%s (vb=0x%p) 0x%p %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	spin_lock_irq(&pcdev->videobuf_queue_lock);
	list_add_tail(&buf->queue, &pcdev->capture);
	schedule_work(&pcdev->work);
	spin_unlock_irq(&pcdev->videobuf_queue_lock);

	dev_dbg(icd->parent, "Finished tegra_camera_videobuf_queue()\n");
}

static void tegra_camera_videobuf_release(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
						     struct soc_camera_device,
						     vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct tegra_buffer *buf = to_tegra_vb(vb);
	struct tegra_camera_dev *pcdev = ici->priv;

	dev_dbg(icd->parent, "In tegra_camera_videobuf_release()\n");

	mutex_lock(&pcdev->work_mutex);

	spin_lock_irq(&pcdev->videobuf_queue_lock);

	if (pcdev->active == vb)
		pcdev->active = NULL;

	/*
	 * Doesn't hurt also if the list is empty, but it hurts, if queuing the
	 * buffer failed, and .buf_init() hasn't been called
	 */
	if (buf->queue.next)
		list_del_init(&buf->queue);

	spin_unlock_irq(&pcdev->videobuf_queue_lock);

	mutex_unlock(&pcdev->work_mutex);

	dev_dbg(icd->parent, "Finished tegra_camera_videobuf_release()\n");
}

static int tegra_camera_videobuf_init(struct vb2_buffer *vb)
{
	/* This is for locking debugging only */
	INIT_LIST_HEAD(&to_tegra_vb(vb)->queue);

	return 0;
}

static int tegra_camera_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = container_of(q,
						     struct soc_camera_device,
						     vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct tegra_camera_dev *pcdev = ici->priv;
	struct list_head *buf_head, *tmp;


	mutex_lock(&pcdev->work_mutex);

	spin_lock_irq(&pcdev->videobuf_queue_lock);
	list_for_each_safe(buf_head, tmp, &pcdev->capture) {
		struct tegra_buffer *buf = container_of(buf_head,
				struct tegra_buffer,
				queue);
		if (buf->icd == icd)
			list_del_init(buf_head);
	}
	spin_unlock_irq(&pcdev->videobuf_queue_lock);

	if (pcdev->active) {
		struct tegra_buffer *buf = to_tegra_vb(pcdev->active);
		if (buf->icd == icd)
			pcdev->active = NULL;
	}

	mutex_unlock(&pcdev->work_mutex);

	return 0;
}

static struct vb2_ops tegra_camera_videobuf_ops = {
	.queue_setup	= tegra_camera_videobuf_setup,
	.buf_prepare	= tegra_camera_videobuf_prepare,
	.buf_queue	= tegra_camera_videobuf_queue,
	.buf_cleanup	= tegra_camera_videobuf_release,
	.buf_init	= tegra_camera_videobuf_init,
	.wait_prepare	= soc_camera_unlock,
	.wait_finish	= soc_camera_lock,
	.stop_streaming	= tegra_camera_stop_streaming,
};

/*
 *  SOC camera host operations
 */
static int tegra_camera_init_videobuf(struct vb2_queue *q,
				      struct soc_camera_device *icd)
{
	dev_dbg(icd->parent, "In tegra_camera_init_videobuf()\n");

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = icd;
	q->ops = &tegra_camera_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct tegra_buffer);

	dev_dbg(icd->parent, "Finished tegra_camera_init_videobuf()\n");

	return vb2_queue_init(q);
}

/*
 * Called with .video_lock held
 */
static int tegra_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct tegra_camera_dev *pcdev = ici->priv;

	if (!pcdev->enable_refcnt) {
		pm_runtime_get_sync(ici->v4l2_dev.dev);
		tegra_camera_activate(pcdev);
		pcdev->num_frames = 0;
	}
	pcdev->enable_refcnt++;

	dev_dbg(icd->parent, "TEGRA Camera host attached to camera %d\n",
		icd->devnum);

	return 0;
}

/* Called with .video_lock held */
static void tegra_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct tegra_camera_dev *pcdev = ici->priv;

	pcdev->enable_refcnt--;
	if (!pcdev->enable_refcnt) {
		cancel_work_sync(&pcdev->work);
		tegra_camera_deactivate(pcdev);
		pm_runtime_put_sync(ici->v4l2_dev.dev);
	}

	dev_dbg(icd->parent, "Frames captured: %d\n", pcdev->num_frames);

	dev_dbg(icd->parent, "TEGRA camera host detached from camera %d\n",
		icd->devnum);
}

static int tegra_camera_set_bus_param(struct soc_camera_device *icd)
{
	return 0;
}

static int tegra_camera_get_formats(struct soc_camera_device *icd,
				    unsigned int idx,
				    struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	int formats = 0;
	int ret;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;
	int k;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret != 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_err(dev, "Invalid format code #%u: %d\n", idx, code);
		return 0;
	}

	switch (code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
	case V4L2_MBUS_FMT_SBGGR8_1X8:
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		formats += ARRAY_SIZE(tegra_camera_formats);
		for (k = 0;
		     xlate && (k < ARRAY_SIZE(tegra_camera_formats));
		     k++) {
			xlate->host_fmt	= &tegra_camera_formats[k];
			xlate->code	= code;
			xlate++;

			dev_info(dev, "Providing format %s using code %d\n",
				 tegra_camera_formats[k].name, code);
		}
		break;
	default:
		dev_info(dev, "Not supporting %s\n", fmt->name);
		return 0;
	}

	return formats;
}

static void tegra_camera_put_formats(struct soc_camera_device *icd)
{
	kfree(icd->host_priv);
	icd->host_priv = NULL;
}

static int tegra_camera_set_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct device *dev = icd->parent;
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct tegra_camera_dev *pcdev = ici->priv;

	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	dev_dbg(dev, "In tegra_camera_set_fmt()\n");

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(dev, "Format %x not found\n", pix->pixelformat);
		return -EINVAL;
	}

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (IS_ERR_VALUE(ret)) {
		dev_warn(dev, "Failed to configure for format %x\n",
			 pix->pixelformat);
		return ret;
	}

	if (mf.code != xlate->code) {
		dev_warn(dev, "mf.code = %d, xlate->code = %d, mismatch\n",
			mf.code, xlate->code);
		return -EINVAL;
	}

	icd->user_width		= mf.width;
	icd->user_height	= mf.height;
	icd->current_fmt	= xlate;

	pcdev->field = pix->field;

	dev_dbg(dev, "Finished tegra_camera_set_fmt(), returning %d\n", ret);

	return ret;
}

static int tegra_camera_try_fmt(struct soc_camera_device *icd,
				struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int ret;

	dev_dbg(icd->parent, "In tegra_camera_try_fmt()\n");

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(icd->parent, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
						    xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;
	pix->sizeimage = pix->height * pix->bytesperline;

	/* limit to sensor capabilities */
	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (IS_ERR_VALUE(ret))
		return ret;

	pix->width	= mf.width;
	pix->height	= mf.height;
	pix->colorspace	= mf.colorspace;
	/*
	 * width and height could have been changed, therefore update the
	 * bytesperline and sizeimage here.
	 */
	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
						    xlate->host_fmt);
	pix->sizeimage = pix->height * pix->bytesperline;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field	= V4L2_FIELD_NONE;
		break;
	default:
		/* TODO: support interlaced at least in pass-through mode */
		dev_err(icd->parent, "Field type %d unsupported.\n",
			mf.field);
		return -EINVAL;
	}

	dev_dbg(icd->parent,
		"Finished tegra_camera_try_fmt(), returning %d\n", ret);

	return ret;
}

static int tegra_camera_reqbufs(struct soc_camera_device *icd,
				struct v4l2_requestbuffers *p)
{
	return 0;
}

static unsigned int tegra_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int tegra_camera_querycap(struct soc_camera_host *ici,
				 struct v4l2_capability *cap)
{
	strlcpy(cap->card, TEGRA_CAM_DRV_NAME, sizeof(cap->card));
	cap->version = TEGRA_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static struct soc_camera_host_ops tegra_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.init_videobuf2	= tegra_camera_init_videobuf,
	.add		= tegra_camera_add_device,
	.remove		= tegra_camera_remove_device,
	.set_bus_param	= tegra_camera_set_bus_param,
	.get_formats	= tegra_camera_get_formats,
	.put_formats	= tegra_camera_put_formats,
	.set_fmt	= tegra_camera_set_fmt,
	.try_fmt	= tegra_camera_try_fmt,
	.reqbufs	= tegra_camera_reqbufs,
	.poll		= tegra_camera_poll,
	.querycap	= tegra_camera_querycap,
};

static struct of_device_id tegra_vi_of_match[] = {
#ifdef TEGRA_2X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra20-vi",
		.data = (struct nvhost_device_data *)&t20_vi_info },
#endif
#ifdef TEGRA_3X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra30-vi",
		.data = (struct nvhost_device_data *)&t30_vi_info },
#endif
#ifdef TEGRA_11X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra114-vi",
		.data = (struct nvhost_device_data *)&t11_vi_info },
#endif
#ifdef TEGRA_14X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra148-vi",
		.data = (struct nvhost_device_data *)&t14_vi_info },
#endif
	{ },
};

static int tegra_camera_probe(struct platform_device *pdev)
{
	struct tegra_camera_dev *pcdev;
	struct nvhost_device_data *ndata = NULL;
	int err = 0;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_vi_of_match, &pdev->dev);
		if (match)
			ndata = match->data;
	} else
		ndata = pdev->dev.platform_data;

	if (!ndata) {
		dev_err(&pdev->dev, "No nvhost device data!\n");
		err = -EINVAL;
		goto exit;
	}

	pcdev = kzalloc(sizeof(struct tegra_camera_dev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	pcdev->ndata = ndata;
	pcdev->ndev = pdev;

	pcdev->ici.priv = pcdev;
	pcdev->ici.v4l2_dev.dev = &pdev->dev;
	pcdev->ici.nr = pdev->id;
	pcdev->ici.drv_name = dev_name(&pdev->dev);
	pcdev->ici.ops = &tegra_soc_camera_host_ops;

	INIT_LIST_HEAD(&pcdev->capture);
	INIT_WORK(&pcdev->work, tegra_camera_work);
	spin_lock_init(&pcdev->videobuf_queue_lock);
	mutex_init(&pcdev->work_mutex);

	pcdev->clk_vi = clk_get(&pdev->dev, "vi");
	if (IS_ERR_OR_NULL(pcdev->clk_vi)) {
		dev_err(&pdev->dev, "Failed to get vi clock.\n");
		goto exit_free_pcdev;
	}

	pcdev->clk_vi_sensor = clk_get(&pdev->dev, "vi_sensor");
	if (IS_ERR_OR_NULL(pcdev->clk_vi_sensor)) {
		dev_err(&pdev->dev, "Failed to get vi_sensor clock.\n");
		goto exit_put_clk_vi;
	}

	pcdev->clk_csi = clk_get(&pdev->dev, "csi");
	if (IS_ERR_OR_NULL(pcdev->clk_csi)) {
		dev_err(&pdev->dev, "Failed to get csi clock.\n");
		goto exit_put_clk_vi_sensor;
	}

	pcdev->clk_isp = clk_get(&pdev->dev, "isp");
	if (IS_ERR_OR_NULL(pcdev->clk_isp)) {
		dev_err(&pdev->dev, "Failed to get isp clock.\n");
		goto exit_put_clk_csi;
	}

	pcdev->clk_csus = clk_get(&pdev->dev, "csus");
	if (IS_ERR_OR_NULL(pcdev->clk_csus)) {
		dev_err(&pdev->dev, "Failed to get csus clock.\n");
		goto exit_put_clk_isp;
	}

	pcdev->clk_sclk = clk_get(&pdev->dev, "sclk");
	if (IS_ERR_OR_NULL(pcdev->clk_sclk)) {
		dev_err(&pdev->dev, "Failed to get sclk clock.\n");
		goto exit_put_clk_csus;
	}

	pcdev->clk_emc = clk_get(&pdev->dev, "emc");
	if (IS_ERR_OR_NULL(pcdev->clk_emc)) {
		dev_err(&pdev->dev, "Failed to get emc clock.\n");
		goto exit_put_clk_sclk;
	}

	clk_set_rate(pcdev->clk_vi, 150000000);
	clk_set_rate(pcdev->clk_vi_sensor, 24000000);

	/* Get regulator pointer */
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	pcdev->reg = regulator_get(&pdev->dev, "vcsi");
#else
	pcdev->reg = regulator_get(&pdev->dev, "avdd_dsi_csi");
#endif
	if (IS_ERR_OR_NULL(pcdev->reg)) {
		dev_err(&pdev->dev, "%s: couldn't get regulator\n",
				__func__);
		goto exit_put_clk_emc;
	}

	platform_set_drvdata(pdev, ndata);
	err = nvhost_client_device_get_resources(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: nvhost get resources failed %d\n",
				__func__, err);
		goto exit_put_regulator;
	}

	err = nvhost_client_device_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: nvhost init failed %d\n",
				__func__, err);
		goto exit_put_regulator;
	}

	pcdev->vi_base = ndata->aperture[0];

	tegra_pd_add_device(&pdev->dev);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, ndata->clockgate_delay);
	pm_runtime_enable(&pdev->dev);

	pcdev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(pcdev->alloc_ctx)) {
		err = PTR_ERR(pcdev->alloc_ctx);
		goto exit_pm_disable;
	}

	platform_set_drvdata(pdev, pcdev);
	err = soc_camera_host_register(&pcdev->ici);
	if (IS_ERR_VALUE(err))
		goto exit_cleanup_alloc_ctx;

	dev_notice(&pdev->dev, "Tegra camera driver loaded.\n");

	return err;

exit_cleanup_alloc_ctx:
	platform_set_drvdata(pdev, pcdev->ndata);
	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);
exit_pm_disable:
	pm_runtime_disable(&pdev->dev);
exit_put_regulator:
	regulator_put(pcdev->reg);
exit_put_clk_emc:
	clk_put(pcdev->clk_emc);
exit_put_clk_sclk:
	clk_put(pcdev->clk_sclk);
exit_put_clk_csus:
	clk_put(pcdev->clk_csus);
exit_put_clk_isp:
	clk_put(pcdev->clk_isp);
exit_put_clk_csi:
	clk_put(pcdev->clk_csi);
exit_put_clk_vi_sensor:
	clk_put(pcdev->clk_vi_sensor);
exit_put_clk_vi:
	clk_put(pcdev->clk_vi);
exit_free_pcdev:
	kfree(pcdev);
exit:
	return err;
}

static int tegra_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *ici = to_soc_camera_host(&pdev->dev);
	struct tegra_camera_dev *pcdev = container_of(ici,
					struct tegra_camera_dev, ici);

	soc_camera_host_unregister(ici);

	platform_set_drvdata(pdev, pcdev->ndata);
	nvhost_client_device_release(pdev);

	vb2_dma_contig_cleanup_ctx(pcdev->alloc_ctx);

	pm_runtime_disable(&pdev->dev);

	regulator_put(pcdev->reg);

	clk_put(pcdev->clk_emc);
	clk_put(pcdev->clk_sclk);
	clk_put(pcdev->clk_csus);
	clk_put(pcdev->clk_isp);
	clk_put(pcdev->clk_csi);
	clk_put(pcdev->clk_vi_sensor);
	clk_put(pcdev->clk_vi);

	kfree(pcdev);

	dev_notice(&pdev->dev, "Tegra camera host driver unloaded\n");

	return 0;
}

#ifdef CONFIG_PM_FISH
static int tegra_camera_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct soc_camera_host *ici = to_soc_camera_host(&pdev->dev);
	struct tegra_camera_dev *pcdev = container_of(ici,
					struct tegra_camera_dev, ici);

	mutex_lock(&pcdev->work_mutex);

	/* We only need to do something if a camera sensor is attached. */
	if (pcdev->icd) {
		/* Suspend the camera sensor. */
		WARN_ON(!pcdev->icd->ops->suspend);
		pcdev->icd->ops->suspend(pcdev->icd, state);
	}

	return 0;
}

static int tegra_camera_resume(struct platform_device *pdev)
{
	struct soc_camera_host *ici = to_soc_camera_host(&pdev->dev);
	struct tegra_camera_dev *pcdev = container_of(ici,
					struct tegra_camera_dev, ici);

	/* We only need to do something if a camera sensor is attached. */
	if (pcdev->icd) {
		/* Resume the camera host. */
		tegra_camera_save_syncpts(pcdev);
		if (pcdev->active)
			tegra_camera_capture_setup(pcdev);

		/* Resume the camera sensor. */
		WARN_ON(!pcdev->icd->ops->resume);
		pcdev->icd->ops->resume(pcdev->icd);
	}

	mutex_unlock(&pcdev->work_mutex);

	return 0;
}
#endif

static struct platform_driver tegra_camera_driver = {
	.driver	= {
		.name	= TEGRA_CAM_DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = tegra_vi_of_match,
#endif
	},
	.probe		= tegra_camera_probe,
	.remove		= tegra_camera_remove,
#ifdef CONFIG_PM_FISH
	.suspend	= tegra_camera_suspend,
	.resume		= tegra_camera_resume,
#endif
};


static int __init tegra_camera_init(void)
{
	return platform_driver_register(&tegra_camera_driver);
}

static void __exit tegra_camera_exit(void)
{
	platform_driver_unregister(&tegra_camera_driver);
}

module_init(tegra_camera_init);
module_exit(tegra_camera_exit);

MODULE_DESCRIPTION("TEGRA SoC Camera Host driver");
MODULE_AUTHOR("Andrew Chew <achew@nvidia.com>");
MODULE_AUTHOR("Bryan Wu <pengw@nvidia.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("nvhost:" TEGRA_CAM_DRV_NAME);
