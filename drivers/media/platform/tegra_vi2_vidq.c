/*
 * Copyright 2015 Alban Bedel <alban.bedel@avionic-design.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include <media/videobuf2-dma-contig.h>

#include "tegra_vi2.h"

#define TEGRA_SYNCPT_CSI_WAIT_TIMEOUT 200

#define ROUND_UP(v, a) (DIV_ROUND_UP(v, a) * (a))
#define ROUND_STRIDE(v) ROUND_UP(v, 64)

static int tegra_vi_channel_capture_thread(void *data);

static int show_statistics = 1;
module_param(show_statistics, int, 0644);

static int v4l2_pix_format_stride(const struct v4l2_pix_format *pf, int *stride)
{
	switch (pf->pixelformat) {
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		stride[0] = ROUND_STRIDE(pf->width);
		return 1;
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		stride[0] = ROUND_STRIDE(pf->width * 2);
		return 1;
	case V4L2_PIX_FMT_RGB32:
		stride[0] = ROUND_STRIDE(pf->width * 4);
		return 1;
	case V4L2_PIX_FMT_YUV422P:
		/* Make sure that all strides aligns properly while
		 * ensuring that stride[1] = stride[0] / 2.
		 */
		stride[1] = ROUND_STRIDE(pf->width / 2);
		stride[2] = stride[1];
		stride[0] = stride[1] * 2;
		return 3;
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		stride[0] = ROUND_STRIDE(pf->width);
		stride[1] = ROUND_STRIDE(pf->width);
		return 2;
	default:
		return -EINVAL;
	}
}

int v4l2_pix_format_set_sizeimage(struct v4l2_pix_format *pf)
{
	int planes, stride[3] = {};

	planes = v4l2_pix_format_stride(pf, stride);
	if (planes < 0)
		return planes;

	pf->sizeimage = (stride[0] + stride[1] + stride[2]) * pf->height;
	pf->bytesperline = stride[0];
	return 0;
}

static int tegra_vi_videobuf_queue_setup(
	struct vb2_queue *vq, const struct v4l2_format *fmt,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], void *alloc_ctxs[])
{
	struct tegra_vi_channel *chan =
		container_of(vq, struct tegra_vi_channel, vb);
	const struct v4l2_pix_format *pf = &fmt->fmt.pix;
	struct video_device *vdev = &chan->vdev;
	int planes, stride[3] = {};

	if (fmt) {
		if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		pf = &fmt->fmt.pix;
	} else {
		pf = &chan->pixfmt;
	}

	planes = v4l2_pix_format_stride(pf, stride);
	if (planes < 0) {
		dev_err(&vdev->dev, "Format unsupported by the queue\n");
		return planes;
	}

	if (*num_buffers < 4)
		*num_buffers = 4;

	*num_planes = 1; /* We don't support the planar API yet */
	sizes[0] = (stride[0] + stride[1] + stride[2]) * pf->height;
	alloc_ctxs[0] = chan->vb2_alloc_ctx;

	return 0;
}

static int tegra_vi_videobuf_init(struct vb2_buffer *vb)
{
	struct tegra_vi_buffer *buf =
		container_of(vb, struct tegra_vi_buffer, vb);

	INIT_LIST_HEAD(&buf->queue);

	return 0;
}

static int tegra_vi_videobuf_prepare(struct vb2_buffer *vb)
{
	struct tegra_vi_channel *chan =
		container_of(vb->vb2_queue, struct tegra_vi_channel, vb);
	struct tegra_vi_buffer *buf =
		container_of(vb, struct tegra_vi_buffer, vb);
	unsigned height = chan->pixfmt.height;

	if (vb->v4l2_buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, vb2_plane_size(vb, 0));

	buf->num_planes = v4l2_pix_format_stride(&chan->pixfmt, buf->stride);
	if (buf->num_planes < 0)
		return buf->num_planes;

	switch (chan->pixfmt.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		buf->addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->addr[1] = buf->addr[0] + buf->stride[0] * height;
		buf->addr[2] = buf->addr[1] + buf->stride[1] * height;
		break;

	case V4L2_FIELD_INTERLACED:
	case V4L2_FIELD_INTERLACED_TB:
		buf->addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->addr[1] = buf->addr[0] + buf->stride[0] * height;
		buf->addr[2] = buf->addr[1] + buf->stride[1] * height;

		buf->bf_addr[0] = buf->addr[0] + buf->stride[0];
		buf->bf_addr[1] = buf->addr[1] + buf->stride[1];
		buf->bf_addr[2] = buf->addr[2] + buf->stride[2];

		buf->stride[0] *= 2;
		buf->stride[1] *= 2;
		buf->stride[2] *= 2;
		break;
	case V4L2_FIELD_INTERLACED_BT:
		buf->bf_addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->bf_addr[1] = buf->bf_addr[0] + buf->stride[0] * height;
		buf->bf_addr[2] = buf->bf_addr[1] + buf->stride[1] * height;

		buf->addr[0] = buf->bf_addr[0] + buf->stride[0];
		buf->addr[1] = buf->bf_addr[1] + buf->stride[1];
		buf->addr[2] = buf->bf_addr[2] + buf->stride[2];

		buf->stride[0] *= 2;
		buf->stride[1] *= 2;
		buf->stride[2] *= 2;
		break;

	case V4L2_FIELD_SEQ_TB:
		buf->addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->addr[1] = buf->addr[0] + buf->stride[0] * height / 2;
		buf->addr[2] = buf->addr[1] + buf->stride[1] * height / 2;

		buf->bf_addr[0] = buf->addr[2] + buf->stride[2] * height / 2;
		buf->bf_addr[1] = buf->bf_addr[0] + buf->stride[0] * height / 2;
		buf->bf_addr[2] = buf->bf_addr[1] + buf->stride[1] * height / 2;
		break;

	case V4L2_FIELD_SEQ_BT:
		buf->bf_addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->bf_addr[1] = buf->bf_addr[0] + buf->stride[0] * height / 2;
		buf->bf_addr[2] = buf->bf_addr[1] + buf->stride[1] * height / 2;

		buf->addr[0] = buf->bf_addr[2] + buf->stride[2] * height / 2;
		buf->addr[1] = buf->addr[0] + buf->stride[0] * height / 2;
		buf->addr[2] = buf->addr[1] + buf->stride[1] * height / 2;
		break;
	}

	return 0;
}

static void tegra_vi_videobuf_queue(struct vb2_buffer *vb)
{
	struct tegra_vi_channel *chan =
		container_of(vb->vb2_queue, struct tegra_vi_channel, vb);
	struct tegra_vi_buffer *buf =
		container_of(vb, struct tegra_vi_buffer, vb);
	unsigned long flags;

	spin_lock_irqsave(&chan->vq_lock, flags);
	list_add_tail(&buf->queue, &chan->capture);
	spin_unlock_irqrestore(&chan->vq_lock, flags);
}

static void tegra_vi_channel_clear_queue(struct tegra_vi_channel *chan)
{
	struct tegra_vi_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&chan->vq_lock, flags);
	list_for_each_entry_safe(buf, tmp, &chan->capture, queue) {
		list_del_init(&buf->queue);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&chan->vq_lock, flags);
}

static int tegra_vi_videobuf_start_streaming(struct vb2_queue *q,
					unsigned int count)
{
	struct tegra_vi_channel *chan =
		container_of(q, struct tegra_vi_channel, vb);
	struct video_device *vdev = &chan->vdev;
	struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);
	int err;

	mutex_lock(&chan->lock);
	if (!chan->input) {
		mutex_unlock(&chan->lock);
		return -EINVAL;
	}

	mutex_lock(&chan->input->lock);
	if (chan->input->streaming_count == 0) {
		tegra_vi_input_start(vi2, chan->input);

		err = v4l2_subdev_call(chan->input->sensor, video, s_stream, 1);
		if (err && err != -ENOIOCTLCMD) {
			dev_err(&vdev->dev, "Failed to start sensor\n");
			goto input_error;
		}

		/* Calibrate the input */
		err = tegra_vi_calibrate_input(vi2, chan->input);
		if (err) {
			dev_err(&vdev->dev, "Failed to calibrate input\n");
			v4l2_subdev_call(chan->input->sensor, video, s_stream, 0);
			goto input_error;
		}
	}
	chan->input->streaming_count++;

	chan->sequence = 0;
	chan->work_th = kthread_run(tegra_vi_channel_capture_thread, chan,
				"tergra_vi.%d capture", chan->id);
	if (IS_ERR(chan->work_th)) {
		dev_err(&vdev->dev, "Failed to create capture thread\n");
		goto input_error;
	}

	mutex_unlock(&chan->input->lock);
	mutex_unlock(&chan->lock);

	return 0;

input_error:
	tegra_vi_input_stop(vi2, chan->input);
	mutex_unlock(&chan->input->lock);

	tegra_vi_channel_clear_queue(chan);
	mutex_unlock(&chan->lock);

	return err;
}

static int tegra_vi_videobuf_stop_streaming(struct vb2_queue *q)
{
	struct tegra_vi_channel *chan =
		container_of(q, struct tegra_vi_channel, vb);
	struct video_device *vdev = &chan->vdev;
	struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);
	int err;

	/* First wait for the capture thread to finish */
	kthread_stop(chan->work_th);
	chan->work_th = NULL;
	chan->should_stop = false;

	mutex_lock(&chan->lock);

	/* Stop the sensor */
	mutex_lock(&chan->input->lock);
	if (chan->input->streaming_count == 1) {
		err = v4l2_subdev_call(chan->input->sensor, video, s_stream, 0);
		if (err)
			dev_warn(&vdev->dev, "Failed to stop sensor!\n");

		tegra_vi_input_stop(vi2, chan->input);
	}
	chan->input->streaming_count--;
	mutex_unlock(&chan->input->lock);

	/* Clear the buffer queue */
	tegra_vi_channel_clear_queue(chan);
	/* Reset the channel logic */
	vi_writel(0x1F, &chan->vi_regs->sw_reset);
	vi_writel(0x00, &chan->vi_regs->sw_reset);

	mutex_unlock(&chan->lock);

	return 0;
}

const struct vb2_ops tegra_vi_qops = {
	.queue_setup = tegra_vi_videobuf_queue_setup,
	.buf_init = tegra_vi_videobuf_init,
	.buf_prepare = tegra_vi_videobuf_prepare,
	.buf_queue = tegra_vi_videobuf_queue,
	.start_streaming = tegra_vi_videobuf_start_streaming,
	.stop_streaming = tegra_vi_videobuf_stop_streaming,
};

static void tegra_vi_channel_queue_syncpt(
	const struct tegra_vi_channel *chan, unsigned cond, u32 *syncpt_val)
{
	const struct video_device *vdev = &chan->vdev;
	struct platform_device *pdev = to_platform_device(vdev->v4l2_dev->dev);
	const struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);

	/* Update the sync point value */
	if (!nvhost_syncpt_read_ext_check(pdev, chan->syncpt_id, syncpt_val))
		*syncpt_val = nvhost_syncpt_incr_max_ext(
			pdev, chan->syncpt_id, 1);

	/* Queue a sync point raise on the condition */
	vi_writel((cond << 8) | chan->syncpt_id, &vi2->vi_regs->vi_incr_syncpt);
}

/* Let a channel wait for an event */
static int tegra_vi_channel_wait_for_syncpt(
	struct tegra_vi_channel *chan, u32 syncpt_val)
{
	struct video_device *vdev = &chan->vdev;
	struct platform_device *pdev = to_platform_device(vdev->v4l2_dev->dev);
	int err;

	/* Wait for the sync point without lock */
	mutex_unlock(&chan->lock);
	err = nvhost_syncpt_wait_timeout_ext(
		pdev, chan->syncpt_id, syncpt_val,
		TEGRA_SYNCPT_CSI_WAIT_TIMEOUT, NULL, NULL);
	mutex_lock(&chan->lock);

	/* TODO: recover in case of error */
	return err;
}

/* Get a buffer form the capture queue and setup the DMA registers */
static struct tegra_vi_buffer * tegra_vi_channel_set_next_buffer(struct tegra_vi_channel *chan)
{
	struct tegra_vi_buffer *buf = NULL;
	unsigned long flags;
	bool missed = false;
	int i;

	/* Get the next buffer, if none is available sleep a bit */
	while (!buf) {
		if (chan->should_stop || kthread_should_stop())
			return NULL;

		spin_lock_irqsave(&chan->vq_lock, flags);

		if (!list_empty(&chan->capture)) {
			buf = list_first_entry(&chan->capture, struct tegra_vi_buffer, queue);
			list_del_init(&buf->queue);
		}

		spin_unlock_irqrestore(&chan->vq_lock, flags);

		if (!missed && !buf)
			missed = true;
	}

	/* Setup the DMA registers */
	for (i = 0; i < buf->num_planes; i++) {
		vi_writel(0, &chan->vi_regs->surface_offset[i].msb);
		vi_writel(buf->addr[i], &chan->vi_regs->surface_offset[i].lsb);
		vi_writel(buf->stride[i], &chan->vi_regs->surface_stride[i]);

		vi_writel(0, &chan->vi_regs->surface_bf_offset[i].msb);
		vi_writel(buf->bf_addr[i],
			&chan->vi_regs->surface_bf_offset[i].lsb);
	}

	if (missed)
		chan->missed_buffer++;

	return buf;
}

static void tegra_vi_channel_clear_errors(struct tegra_vi_channel *chan)
{
	vi_writel(0xffffffff, &chan->vi_regs->error_status);
	vi_writel(0xffffffff, &chan->mipi_regs->status);
	if (chan->input->cil_regs[0]) {
		vi_writel(0xffffffff, &chan->input->cil_regs[0]->status);
		vi_writel(0xffffffff, &chan->input->cil_regs[0]->cil_status);
	}
	if (chan->input->cil_regs[1]) {
		vi_writel(0xffffffff, &chan->input->cil_regs[1]->status);
		vi_writel(0xffffffff, &chan->input->cil_regs[1]->cil_status);
	}
	chan->missed_buffer = 0;
}

static void tegra_vi_channel_print_errors(struct tegra_vi_channel *chan)
{
	struct video_device *vdev = &chan->vdev;

	dev_err(&vdev->dev, "VI STATUS: 0x%08x\n",
		readl(&chan->vi_regs->error_status));
	dev_err(&vdev->dev, "PP STATUS: 0x%08x\n",
		readl(&chan->mipi_regs->status));
	if (chan->input->cil_regs[0]) {
		dev_err(&vdev->dev, "CIL%c STATUS: 0x%08x\n",
			'A' + chan->input->id * 2,
			readl(&chan->input->cil_regs[0]->status));
		dev_err(&vdev->dev, "CIL%c CIL STATUS: 0x%08x\n",
			'A' + chan->input->id * 2,
			readl(&chan->input->cil_regs[0]->cil_status));
	}
	if (chan->input->cil_regs[1]) {
		dev_err(&vdev->dev, "CIL%c STATUS: 0x%08x\n",
			'B' + chan->input->id * 2,
			readl(&chan->input->cil_regs[1]->status));
		dev_err(&vdev->dev, "CIL%c CIL STATUS: 0x%08x\n",
			'B' + chan->input->id * 2,
			readl(&chan->input->cil_regs[1]->cil_status));
	}
	tegra_vi_channel_clear_errors(chan);
}

/* Capture loop */
static int tegra_vi_channel_capture_thread(void *data)
{
	struct tegra_vi_channel *chan = data;
	struct video_device *vdev = &chan->vdev;
	ktime_t start_time;
	unsigned syncpt_cond = 0;
	u32 syncpt_val;
	int err;

	mutex_lock(&chan->lock);

	/* Don't capture in broadcast mode it is broken */
	if (chan->input->use_count > 1) {
		dev_err(&vdev->dev, "Broadcast capture is not working yet!\n");
		goto finish;
	}

	tegra_vi_channel_clear_errors(chan);

	/* Program DMA for first buffer */
	chan->active_buffer = tegra_vi_channel_set_next_buffer(chan);
	if (!chan->active_buffer)
		goto finish;

	switch (chan->id) {
	case 0: syncpt_cond = 5; /* VI_CSI_PPA_FRAME_START */
		break;
	case 1: syncpt_cond = 13; /* VI_CSI_PPC_FRAME_START */
		break;
	case 2: syncpt_cond = 21; /* VI_CSI_PPE_FRAME_START */
		break;
	default:
		pr_err("%s(): Error: undefined chan->id (%d)\n", __func__, chan->id);
	}

	tegra_vi_channel_queue_syncpt(chan, syncpt_cond, &syncpt_val);

	if (show_statistics)
		start_time = ktime_get();

	/* Start the Pixel Parser */
	vi_writel(0xf005, &chan->mipi_regs->pp_command);
	/* Queue a buffer update on the next shot */
	vi_writel(1, &chan->vi_regs->single_shot);

	/* Wait for SOF */
	err = tegra_vi_channel_wait_for_syncpt(chan, syncpt_val);
	if (err) {
		dev_err(&vdev->dev, "Initial wait for SOF failed!\n");
		tegra_vi_channel_print_errors(chan);
		goto stop_vi;
	}

	/* Program DMA for the second buffer (in the shadow registers) */
	chan->pending_buffer = tegra_vi_channel_set_next_buffer(chan);
	if (!chan->pending_buffer)
		goto stop_vi;

	/* FIXME: We should check for chan->active_buffer here, but the last
	 *        sync point wait fails. As a workaround we stop one frame
	 *        earlier. */
	while (chan->pending_buffer) {
		struct tegra_vi_buffer *done_buffer;

		/* Wait for the write ACK */
		err = tegra_vi_channel_wait_for_syncpt(chan, syncpt_val);
		if (err) {
			if (!chan->should_stop) {
				dev_err(&vdev->dev,
					"Failed to capture frame\n");
				tegra_vi_channel_print_errors(chan);
			}
			break;
		}

		/* The active buffer is done and then pending is now active */
		done_buffer = chan->active_buffer;
		chan->active_buffer = chan->pending_buffer;

		/* Setup the next pending buffer */
		chan->pending_buffer = tegra_vi_channel_set_next_buffer(chan);
		switch (chan->id) {
		case 0: syncpt_cond = 7; /* VI_MWA_ACK_DONE */
			break;
		case 1: syncpt_cond = 15; /* VI_MWC_ACK_DONE */
			break;
		case 2: syncpt_cond = 23; /* VI_MWE_ACK_DONE */
			break;
		default:
			pr_err("%s(): Error: undefined chan->id (%d)\n",
				__func__, chan->id);
			break;
		}
		tegra_vi_channel_queue_syncpt(chan, syncpt_cond, &syncpt_val);
		if (chan->pending_buffer) {
			/* Queue a buffer update on the next shot */
			vi_writel(1, &chan->vi_regs->single_shot);
		} else {
			vi_writel(0, &chan->vi_regs->single_shot);
			if (chan->active_buffer) {
				/* Last frame stop the pixel parser once done */
				vi_writel(0xf002, &chan->mipi_regs->pp_command);
			}
		}
		/* Return the last active buffer */
		do_gettimeofday(&done_buffer->vb.v4l2_buf.timestamp);
		done_buffer->vb.v4l2_buf.sequence = chan->sequence++;
		vb2_buffer_done(&done_buffer->vb, VB2_BUF_STATE_DONE);
	}

stop_vi:
	/* Stop capture shot */
	vi_writel(0, &chan->vi_regs->single_shot);
	/* Stop the Pixel Parser */
	vi_writel(0xf003, &chan->mipi_regs->pp_command);

	if (show_statistics) {
		u32 tdiff = ktime_to_ms(ktime_sub(ktime_get(), start_time));
		u64 cap_per_us = 1000000 * chan->sequence;
		u32 subfps;

		do_div(cap_per_us, tdiff);
		subfps = do_div(cap_per_us, 1000);
		dev_info(&vdev->dev,
			"Captured %lu frames in %u ms (%u.%u fps), %lu missed buffers\n",
			chan->sequence, tdiff, (u32)cap_per_us, subfps,
			chan->missed_buffer);
	}

	/* Release the active and pending buffer it not yet done */
	if (chan->active_buffer) {
		vb2_buffer_done(&chan->active_buffer->vb,
				VB2_BUF_STATE_ERROR);
		chan->active_buffer = NULL;
	}

	if (chan->pending_buffer) {
		vb2_buffer_done(&chan->pending_buffer->vb,
				VB2_BUF_STATE_ERROR);
		chan->pending_buffer = NULL;
	}

finish:
	mutex_unlock(&chan->lock);

	/* Wait for kthread_stop() to be called */
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}

	return 0;
}
