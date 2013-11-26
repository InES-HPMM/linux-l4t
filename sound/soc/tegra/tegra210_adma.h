/*
 * tegra210_adma.c - Tegra ADMA driver.
 *
 * Author: Dara Ramesh <dramesh@nvidia.com>
 *
 * Copyright (C) 2013, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __MACH_TEGRA_ADMA_H
#define __MACH_TEGRA_ADMA_H

#define TEGRA210_ADMA_MAX_CHANNELS 22
#define TEGRA210_ADMA_CHANNEL_OFFSET 0x80

/* Register offsets from TEGRA210_ADMA*_BASE */
#define TEGRA210_ADMA_CH_CMD										0x00
#define TEGRA210_ADMA_CH_SOFT_RESET								0x04
#define TEGRA210_ADMA_CH_STATUS									0x0c
#define TEGRA210_ADMA_CH_INT_STATUS								0x10
#define TEGRA210_ADMA_CH_INT_SET									0x18
#define TEGRA210_ADMA_CH_INT_CLEAR								0x1c
#define TEGRA210_ADMA_CH_CTRL									0x24
#define TEGRA210_ADMA_CH_CONFIG									0x28
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL							0x2c
#define TEGRA210_ADMA_CH_TC_STATUS								0x30
#define TEGRA210_ADMA_CH_LOWER_SOURCE_ADDR						0x34
#define TEGRA210_ADMA_CH_LOWER_TARGET_ADDR						0x3c
#define TEGRA210_ADMA_CH_TC										0x44
#define TEGRA210_ADMA_CH_LOWER_DESC_ADDR							0x48
#define TEGRA210_ADMA_CH_TRANSFER_STATUS							0x54
#define TEGRA210_ADMA_GLOBAL_CMD									0xc00
#define TEGRA210_ADMA_GLOBAL_INT_CLEAR							0xc20

#define TEGRA210_ADMA_LAST_REG							0xc44

/* Fields in ADMA_CH_STATUS */
#define TEGRA210_ADMA_CH_STATUS_CURRENT_SOURCE_MEMORY_BUFFER_SHIFT		20
#define TEGRA210_ADMA_CH_STATUS_CURRENT_TARGET_MEMORY_BUFFER_SHIFT		16
#define TEGRA210_ADMA_CH_STATUS_OUTSTANDING_TRANSFERS_SHIFT				02
#define TEGRA210_ADMA_CH_STATUS_TRANSFER_PAUSED_SHIFT					1

/* Fields in ADMA_CH_CTRL  */
#define TEGRA210_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT			28
#define TEGRA210_ADMA_CH_CTRL_TX_REQUEST_SELECT_MASK				(15 << TEGRA210_ADMA_CH_CTRL_TX_REQUEST_SELECT_SHIFT)
#define TEGRA210_ADMA_CH_CTRL_RX_REQUEST_SELECT_SHIFT			24
#define TEGRA210_ADMA_CH_CTRL_RX_REQUEST_SELECT_MASK				(15 << TEGRA210_ADMA_CH_CTRL_RX_REQUEST_SELECT_SHIFT)
#define TEGRA210_ADMA_CH_CTRL_TRIGGER_SELECT_SHIFT				16
#define TEGRA210_ADMA_CH_CTRL_TRANSFER_DIRECTION_SHIFT			12
#define TEGRA210_ADMA_CH_CTRL_TRANSFER_DIRECTION_MASK			(15 << TEGRA210_ADMA_CH_CTRL_TRANSFER_DIRECTION_SHIFT)
#define TEGRA210_ADMA_CH_CTRL_TRANSFER_MODE_SHIFT				8
#define TEGRA210_ADMA_CH_CTRL_TRIGGER_ENABLE_SHIFT				2
#define TEGRA210_ADMA_CH_CTRL_FLOWCTRL_ENABLE_SHIFT				1
#define TEGRA210_ADMA_CH_CTRL_TRANSFER_PAUSE_SHIFT				0

/* Fields in ADMA_CH_CONFIG  */
#define TEGRA210_ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_SHIFT		28
#define TEGRA210_ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_MASK		(7 << TEGRA210_ADMA_CH_CONFIG_SOURCE_MEMORY_BUFFER_SHIFT)
#define TEGRA210_ADMA_CH_CONFIG_TARGET_MEMORY_BUFFER_SHIFT		24
#define TEGRA210_ADMA_CH_CONFIG_TARGET_MEMORY_BUFFER_MASK		(7 << TEGRA210_ADMA_CH_CONFIG_TARGET_MEMORY_BUFFER_SHIFT)
#define TEGRA210_ADMA_CH_CONFIG_BURST_SIZE_SHIFT					20
#define TEGRA210_ADMA_CH_CONFIG_BURST_SIZE_MASK					(7 << TEGRA210_ADMA_CH_CONFIG_BURST_SIZE_SHIFT)
#define TEGRA210_ADMA_CH_CONFIG_SOURCE_ADDR_WRAP_SHIFT			16
#define TEGRA210_ADMA_CH_CONFIG_TARGET_ADDR_WRAP_SHIFT			12
#define TEGRA210_ADMA_CH_CONFIG_WEIGHT_FOR_WRR_SHIFT				3

/* Fields in ADMA_CH_AHUB_FIFO_CTRL  */
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_FETCHING_POLICY_SHIFT		31
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_OVERFLOW_THRESHOLD_SHIFT		24
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_STARVATION_THRESHOLD_SHIFT	16
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_SHIFT			8
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_MASK			(15 << TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_TX_FIFO_SIZE_SHIFT)
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_SHIFT			0
#define TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_MASK			(15 << TEGRA210_ADMA_CH_AHUB_FIFO_CTRL_RX_FIFO_SIZE_SHIFT)

enum tegra210_adma_fetching_policy {
	BURST_BASED = 0,
	THRESHOLD_BASED = 1,
};

enum tegra210_adma_burst_size {
	WORD_1 = 1,
	WORDS_2 = 2,
	WORDS_4 = 3,
	WORDS_8 = 4,
	WORDS_16 = 5,
};

struct tegra210_adma_req;
struct tegra210_adma_channel;

enum tegra210_adma_mode {
	ADMA_MODE_ONESHOT = 1,
	ADMA_MODE_CONTINUOUS = 2,
	ADMA_MODE_LINKED_LIST = 4,
};

enum tegra210_adma_transfer_direction {
	MEMORY_TO_MEMORY = 1,
	AHUB_TO_MEMORY = 2,
	MEMORY_TO_AHUB = 4,
	AHUB_TO_AHUB = 8,
};
/*
 * TEGRA_ADMA_req_status: Dma request status
 * TEGRA_ADMA_REQ_PENDING: Request is pending and not programmed in hw.
 * TEGRA_ADMA_REQ_SUCCESS: The request has been successfully completed.
 *		The byte_transferred tells number of bytes transferred.
 * TEGRA_ADMA_REQ_ERROR_ABORTED: The request is aborted by client after
 *		calling TEGRA_ADMA_dequeue_req.
 *		The byte_transferred tells number of bytes transferred
 *		which may be more than request size due to buffer
 *		wrap-up in continuous mode.
 * TEGRA_ADMA_REQ_ERROR_STOPPED: Applicable in continuous mode.
 *		The request is stopped forcefully. This may be becasue of
 *		- due to non-available of next request.
 *		- not able to serve current interrupt before next buffer
 *		  completed by dma. This can happen if buffer req size is
 *		  not enough and it transfer completes before system actually
 *		  serve the previous dma interrupts.
 *		The byte_transferred will not be accurate in this case. It will
 *		just give an idea that how much approximately have been
 *		transferred by dma.
 * TEGRA_ADMA_REQ_INFLIGHT: The request is configured in the dma register
 *		for transfer.
 */
enum tegra210_adma_req_status {
	ADMA_REQ_PENDING = 0,
	ADMA_REQ_SUCCESS,
	ADMA_REQ_ERROR_ABORTED,
	ADMA_REQ_ERROR_STOPPED,
	ADMA_REQ_INFLIGHT,
};

enum tegra210_adma_req_buff_status {
	ADMA_REQ_BUF_STATUS_EMPTY = 0,
	ADMA_REQ_BUF_STATUS_HALF_FULL,
	ADMA_REQ_BUF_STATUS_FULL,
};

typedef void (*tegra210_adma_callback)(struct tegra210_adma_req *req);

struct tegra210_adma_req {
	struct list_head node;
	unsigned int modid;
	int instance;

	/* Called when the req is complete and from the ADMA ISR context.
	 * When this is called the req structure is no longer queued by
	 * the DMA channel.
	 *
	 * State of the ADMA depends on the number of req it has. If there are
	 * no ADMA requests queued up, then it will STOP the ADMA. It there are
	 * more requests in the ADMA, then it will queue the next request.
	 */
	tegra210_adma_callback complete;

	/*  This is a called from the ADMA ISR context when the ADMA is still in
	 *  progress and is actively filling same buffer.
	 *
	 * In the case of continuous mode receive, if there is next req already
	 * queued, ADMA programs the HW to use that req when this req is
	 * completed. If there is no "next req" queued, then ADMA ISR doesn't do
	 * anything before calling this callback.
	 *
	 */
	tegra210_adma_callback threshold;

	int transfer_direction;

	void *virt_addr;
	unsigned long source_addr;
	unsigned long dest_addr;
	unsigned long dest_wrap;
	unsigned long source_wrap;
	unsigned long source_bus_width;
	unsigned long dest_bus_width;
	unsigned long req_sel;
	unsigned int size;
	bool use_smmu;

	int fixed_burst_size;

	/* Updated by the ADMA driver on the conpletion of the request. */
	int bytes_transferred;
	int status;

	/* ADMA completion tracking information */
	int buffer_status;

	/* Client specific data */
	void *dev;
};
/* TEGRA210 ADMA driver context */
struct tegra210_adma_ctx {
	struct device		*dev;
	struct clk		*clk;
	struct regmap		*regmap;
	void __iomem		*regs;
	bool adma_initialized;
};

void tegra210_adma_init(void);
int tegra210_adma_enqueue_req(struct tegra210_adma_channel *ch,
		struct tegra210_adma_req *req);
struct tegra210_adma_channel *tegra210_adma_allocate_channel(int mode,
		const char namefmt[], ...);
void tegra210_adma_free_channel(struct tegra210_adma_channel *ch);
void tegra210_adma_channel_enable(struct tegra210_adma_channel *ch, bool en);
void tegra210_adma_global_enable(bool en);
int tegra210_adma_get_channel_id(struct tegra210_adma_channel *ch);
int tegra210_adma_channel_status_read(struct tegra210_adma_channel *ch);
int tegra210_adma_channel_tc_status(struct tegra210_adma_channel *ch);
#endif
