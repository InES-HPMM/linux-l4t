/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef ___NVAVP_DEV_IVC_H___
#define ___NVAVP_DEV_IVC_H___

#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra_nvavp.h>

enum ivc_msg_t {
	NVAVP_CONNECT = 0,
	NVAVP_DISCONNECT,
	NVAVP_ABORT,
	NVAVP_PUSHBUFFER_WRITE,
	NVAVP_SET_CLOCK,
	NVAVP_GET_CLOCK,
	NVAVP_FORCE_CLOCK_STAY_ON,
	NVAVP_WAKE,
	NVAVP_SET_MIN_CPU_ONLINE,
	NVAVP_ENABLE_AUDIO_CLOCK,
	NVAVP_DISABLE_AUDIO_CLOCK,
	NVAVP_MAP_IOVA,
	NVAVP_UNMAP_IOVA,
	NVAVP_INVALID,
};

enum ivc_avp_err_t {
	NVAVP_ERR_OK,
	NVAVP_ERR_SERVER_STATE,
	NVAVP_ERR_ARGS,
	NVAVP_ERR_REQ,
	NVAVP_ERR_UNSUPPORTED_REQ
};

enum rx_state_t {
	RX_INIT,
	RX_PENDING,
	RX_AVAIL,
	RX_DONE,
};

struct tegra_nvavp_connect_params {
	struct platform_device *dev;
};

struct tegra_nvavp_pushbuffer_params {
	u32 addr;
	u32 count;
	u32 flags;
	int channel_id;
};

struct ivc_msg {
	u32			channel_id;
	struct nvavp_syncpt	syncpt;
	enum ivc_msg_t		msg;
	enum ivc_avp_err_t	err;
	union {
		struct tegra_nvavp_connect_params connect_params;
		struct tegra_nvavp_pushbuffer_params pushbuffer_params;
		struct nvavp_clock_args clock_params;
		struct nvavp_clock_stay_on_state_args state_params;
		struct nvavp_num_cpus_args numcpu_params;
		struct nvavp_map_args map_params;
	} params;
};

struct ivc_dev;

struct ivc_ctxt {
	struct tegra_hv_ivc_cookie	*ivck;
	wait_queue_head_t		wait;
	int				timeout;
	struct ivc_msg			rx_msg;
	volatile enum rx_state_t	rx_state;
	struct ivc_dev			*ivcdev;
	spinlock_t			lock;
};

struct ivc_dev {
	struct tegra_hv_ivc_cookie	*ivck;
	struct ivc_ctxt			*ictxt;
	struct device			*dev;
	spinlock_t			ivck_rx_lock;
	spinlock_t			ivck_tx_lock;
	spinlock_t			lock;
};

struct ivc_dev *nvavp_ivc_init(struct device *dev);
void nvavp_ivc_deinit(struct ivc_dev *idev);
struct ivc_ctxt *nvavp_ivc_alloc_ctxt(struct ivc_dev *ivcdev);
void nvavp_ivc_free_ctxt(struct ivc_ctxt *ictxt);
int nvavp_ivc(struct ivc_ctxt *ictxt, struct ivc_msg *msg);
#endif
