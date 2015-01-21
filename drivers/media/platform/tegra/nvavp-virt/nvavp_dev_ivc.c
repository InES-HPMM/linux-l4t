/*
 * IVC based librabry for AVP.
 *
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/tegra-ivc.h>
#include <linux/spinlock.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>

#include "nvavp_dev_ivc.h"

static struct ivc_dev *saved_ivc_dev;
static int ivc_poll = 1;

static int ivc_send(struct ivc_ctxt *ictxt,
		struct ivc_msg *msg, int size)
{
	int len = 0;
	unsigned long flags = 0;
	int err = 0;

	if (!ictxt || !ictxt->ivck || !msg || !size)
		return -EINVAL;

	spin_lock_irqsave(&saved_ivc_dev->ivck_tx_lock, flags);

	if (!tegra_hv_ivc_can_write(ictxt->ivck)) {
		err = -EBUSY;
		goto fail;
	}

	len = tegra_hv_ivc_write(ictxt->ivck, msg, size);
	if (len != size) {
		err = -EIO;
		goto fail;
	}

	err = len;

fail:
	spin_unlock_irqrestore(&saved_ivc_dev->ivck_tx_lock, flags);
	return err;
}

static void ivc_rx(struct tegra_hv_ivc_cookie *ivck)
{
	unsigned long flags;

	if (!ivck || !saved_ivc_dev)
		BUG();

	spin_lock_irqsave(&saved_ivc_dev->ivck_rx_lock, flags);

	if (tegra_hv_ivc_can_read(ivck)) {
		/* Message available */
		struct ivc_ctxt *ictxt = NULL;
		u32 len = 0;
		struct ivc_msg rx_msg;

		memset(&rx_msg, 0, sizeof(struct ivc_msg));
		len = tegra_hv_ivc_read(ivck, &rx_msg, sizeof(struct ivc_msg));
		if (len != sizeof(struct ivc_msg)) {
			dev_err(saved_ivc_dev->dev,
					"IVC read failure (msg size error)\n");
			goto fail;
		}

		ictxt = saved_ivc_dev->ictxt;
		if (ictxt->rx_state != RX_PENDING) {
			dev_err(saved_ivc_dev->dev,
					"Spurious message from server\n");
			goto fail;
		}

		/* Copy the message to consumer*/
		memcpy(&ictxt->rx_msg, &rx_msg, sizeof(struct ivc_msg));
		ictxt->rx_state = RX_AVAIL;
	}
fail:
	spin_unlock_irqrestore(&saved_ivc_dev->ivck_rx_lock, flags);
	return;
}

static int ivc_recv_sync(struct ivc_ctxt *ictxt,
		struct ivc_msg *msg, int size)
{
	int err = 0;

	if (!ictxt || !ictxt->ivck)
		return -EINVAL;
	if (!msg)
		return -EINVAL;
	if (!size)
		return -EINVAL;

	/* Make sure some response is pending */
	BUG_ON((ictxt->rx_state != RX_PENDING) &&
			(ictxt->rx_state != RX_AVAIL));

	/* Poll for response from server
	 * This is not the best way as reponse from server
	 * can get delayed and we are wasting cpu cycles.
	 *
	 * Linux drivers can call dma_map/unmap calls from
	 * atomic contexts and it's not possible to block
	 * from those contexts and reason for using polling
	 *
	 * This will change once hypervisor will support
	 * guest timestealing approach for IVC
	 */

	if (ivc_poll) {
		/* Loop for server response */
		while (ictxt->rx_state != RX_AVAIL)
			ivc_rx(ictxt->ivck);
	} else {
		/* Implementation not used */
		err = wait_event_timeout(ictxt->wait,
				ictxt->rx_state == RX_AVAIL,
				msecs_to_jiffies(ictxt->timeout));

		if (!err) {
			err = -ETIMEDOUT;
			goto fail;
		}

		ictxt->rx_state = RX_PENDING;
		ivc_rx(ictxt->ivck);
	}

	err = size;
	memcpy(msg, &ictxt->rx_msg, size);
fail:
	ictxt->rx_state = RX_INIT;
	return err;
}

/* Send request and wait for response */
static int ivc_send_recv(struct ivc_ctxt *ictxt, struct ivc_msg *tx_msg,
			struct ivc_msg *rx_msg)
{
	int err = -EINVAL;
	unsigned long flags;

	if (!ictxt || !tx_msg || !rx_msg)
		return err;

	if (!saved_ivc_dev)
		return err;

	if (tegra_hv_ivc_channel_notified(ictxt->ivck)) {
		pr_warn("%s: Skipping work since queue is not ready\n",
				__func__);
		return 0;
	}

	/* Serialize requests per ASID */
	spin_lock_irqsave(&ictxt->lock, flags);

	/* No outstanding for this ASID */
	BUG_ON(ictxt->rx_state != RX_INIT);

	ictxt->rx_state = RX_PENDING;
	err = ivc_send(ictxt, tx_msg, sizeof(struct ivc_msg));
	if (err < 0)
		goto fail;

	err = ivc_recv_sync(ictxt, rx_msg, sizeof(struct ivc_msg));
	if (err < 0)
		goto fail;

	/* Return the server error code to the caller
	 * Using positive error codes for server status
	 * Using negative error codes for IVC comm errors
	 */
	err = rx_msg->err;
fail:
	spin_unlock_irqrestore(&ictxt->lock, flags);
	return err;
}

/* Every communication with the server is identified
 * with this ivc context.
 * There can be one outstanding request to the server per
 * ivc context.
 *
 * Allocate datastructure to hold context information
 */
struct ivc_ctxt *nvavp_ivc_alloc_ctxt(struct ivc_dev *ivcdev)
{
	struct ivc_ctxt *ictxt = NULL;
	unsigned long flags = 0;

	if (!ivcdev || !ivcdev->ivck || ivcdev != saved_ivc_dev) {
		ictxt = ERR_PTR(-EINVAL);
		goto out;
	}

	spin_lock_irqsave(&ivcdev->lock, flags);

	/* Already provisioned */
	if (ivcdev->ictxt) {
		ictxt = ivcdev->ictxt;
		goto out;
	}

	ictxt = devm_kzalloc(ivcdev->dev, sizeof(struct ivc_ctxt),
			GFP_KERNEL);
	if (!ictxt) {
		dev_err(ivcdev->dev, "failed to allocate ivc context\n");
		ictxt = ERR_PTR(-ENOMEM);
		goto out;
	}

	ictxt->ivck = ivcdev->ivck;
	if (!ivc_poll)
		init_waitqueue_head(&ictxt->wait);
	ictxt->timeout = 250; /* Not used in polling */
	ictxt->rx_state = RX_INIT;
	ictxt->ivcdev = ivcdev;
	spin_lock_init(&ictxt->lock);

	ivcdev->ictxt = ictxt;

out:
	spin_unlock_irqrestore(&ivcdev->lock, flags);
	return ictxt;
}

/* hook to domain destroy */
void nvavp_ivc_free_ctxt(struct ivc_ctxt *ictxt)
{
	struct ivc_dev *ivcdev = NULL;

	if (!ictxt) {
		dev_err(saved_ivc_dev->dev,
				"trying to free null ivc context\n");
		return;
	}

	ivcdev = ictxt->ivcdev;

	spin_lock(&ivcdev->lock);
	/* Acquire the ivc context lock to make sure nothing is in flight */
	spin_lock(&ictxt->lock);
	ivcdev->ictxt = NULL;
	spin_unlock(&ictxt->lock);

	devm_kfree(ivcdev->dev, ictxt);
	spin_unlock(&ivcdev->lock);

}

void nvavp_ivc_deinit(struct ivc_dev *idev)
{
	if (!idev || idev != saved_ivc_dev) {
		dev_err(saved_ivc_dev->dev,
				"Illegal ivc device handle\n");
		return;
	}

	tegra_hv_ivc_unreserve(idev->ivck);

	devm_kfree(idev->dev, idev);
}

static irqreturn_t hv_nvavp_isr(int irq, void *dev_id)
{
	struct ivc_ctxt *ictxt = NULL;
	ictxt = saved_ivc_dev->ictxt;

	if (!ictxt)
		return IRQ_HANDLED;

	if (!ivc_poll)
		wake_up(&ictxt->wait);

	return IRQ_HANDLED;
}

struct ivc_dev *nvavp_ivc_init(struct device *dev)
{
	int err, ivc_queue;
	struct device_node *dn, *hv_dn;
	struct tegra_hv_ivc_cookie *ivck = NULL;
	struct ivc_dev *ivcdev;

	dn = dev->of_node;
	if (dn == NULL) {
		dev_err(dev, "No OF data\n");
		return ERR_PTR(-EINVAL);
	}

	hv_dn = of_parse_phandle(dn, "ivc_queue", 0);
	if (hv_dn == NULL) {
		dev_err(dev, "Failed to parse phandle of ivc prop\n");
		return ERR_PTR(-EINVAL);
	}

	err = of_property_read_u32_index(dn, "ivc_queue", 1, &ivc_queue);
	if (err != 0) {
		dev_err(dev, "Failed to read IVC property ID\n");
		of_node_put(hv_dn);
		return ERR_PTR(-EINVAL);
	}

	ivck = tegra_hv_ivc_reserve(hv_dn, ivc_queue, NULL);
	if (IS_ERR_OR_NULL(ivck)) {
		dev_err(dev, "Failed to reserve ivc queue %d\n", ivc_queue);
		if (ivck == ERR_PTR(-EPROBE_DEFER))
			return ERR_PTR(-EPROBE_DEFER);
		else
			return ERR_PTR(-EINVAL);
	}

	of_node_put(hv_dn);

	ivcdev = devm_kzalloc(dev, sizeof(struct ivc_dev), GFP_KERNEL);
	if (!ivcdev)
		return ERR_PTR(-ENOMEM);

	ivcdev->ivck = ivck;
	ivcdev->dev = dev;
	saved_ivc_dev = ivcdev;

	spin_lock_init(&ivcdev->ivck_rx_lock);
	spin_lock_init(&ivcdev->ivck_tx_lock);
	spin_lock_init(&ivcdev->lock);

	/* Our comm_dev is ready, so enable irq here. But channels are
	 * not yet allocated, we need to take care of that in the
	 * handler
	 */
	err = request_threaded_irq(ivck->irq, hv_nvavp_isr, NULL, 0,
			dev_name(dev), ivcdev);
	if (err) {
		devm_kfree(dev, ivcdev);
		tegra_hv_ivc_unreserve(ivck);
		return ERR_PTR(-ENOMEM);
	}

	/* set ivc channel to invalid state */
	tegra_hv_ivc_channel_reset(ivck);

	return ivcdev;
}

static void print_server_error(struct device *dev, int err)
{
	switch (err) {
	case NVAVP_ERR_SERVER_STATE:
		dev_err(dev, "invalid server state\n");
		break;
	case NVAVP_ERR_ARGS:
		dev_err(dev, "invalid args passed to server\n");
		break;
	case NVAVP_ERR_REQ:
		dev_err(dev, "invalid request to server\n");
		break;
	case NVAVP_ERR_UNSUPPORTED_REQ:
		dev_err(dev, "unsupported message to server\n");
		break;
	default:
		return;
	}
	return;
}

int nvavp_ivc(struct ivc_ctxt *ictxt, struct ivc_msg *msg)
{
	int err = 0;
	struct ivc_msg rx_msg;

	if (!ictxt || !msg || ictxt != saved_ivc_dev->ictxt) {
		dev_err(saved_ivc_dev->dev, "Invalid arguments\n");
		return -EINVAL;
	}

	memset(&rx_msg, 0, sizeof(struct ivc_msg));

	err = ivc_send_recv(ictxt, msg, &rx_msg);

	if (err < 0) {
		dev_err(ictxt->ivcdev->dev,
				"IVC comm error\n");
		return -EIO;
	} else if (err) {
		print_server_error(ictxt->ivcdev->dev, err);
		return -EIO;
	}

	msg->err = rx_msg.err;

	switch (msg->msg) {
	case NVAVP_CONNECT:
		msg->syncpt.id = rx_msg.syncpt.id;
		break;
	case NVAVP_PUSHBUFFER_WRITE:
		msg->syncpt.value = rx_msg.syncpt.value;
		msg->syncpt.id = rx_msg.syncpt.id;
		break;
	case NVAVP_SET_CLOCK:
		msg->params.clock_params.id = rx_msg.params.clock_params.id;
		msg->params.clock_params.rate = rx_msg.params.clock_params.rate;
		break;
	case NVAVP_GET_CLOCK:
		msg->params.clock_params.id = rx_msg.params.clock_params.id;
		msg->params.clock_params.rate = rx_msg.params.clock_params.rate;
		break;
	case NVAVP_MAP_IOVA:
		msg->params.map_params.addr = rx_msg.params.map_params.addr;
		break;
	default:
		break;
	};

	return err;
}
