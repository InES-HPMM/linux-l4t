/*
 * Copyright 2015 Alban Bedel <alban.bedel@avionic-design.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 */

#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <linux/delay.h>
#include <linux/completion.h>

#include <media/videobuf2-dma-contig.h>

#include "tegra_vi2.h"

#define TEGRA_SYNCPT_CSI_WAIT_TIMEOUT 200

static int v4l2_pix_format_stride(const struct v4l2_pix_format *pf, int *stride)
{
	switch (pf->pixelformat) {
	case V4L2_PIX_FMT_GREY:
		stride[0] = pf->width;
		return 1;
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		stride[0] = pf->width * 2;
		return 1;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		stride[0] = pf->width * 4;
		return 1;
	case V4L2_PIX_FMT_YUV422P:
		stride[0] = pf->width;
		stride[1] = pf->width / 2;
		stride[2] = pf->width / 2;
		return 3;
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		stride[0] = pf->width;
		stride[1] = pf->width;
		return 2;
	default:
		return -EINVAL;
	}
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
	struct video_device *vdev = &chan->vdev;

	if (vb->v4l2_buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, vb2_plane_size(vb, 0));

	buf->num_planes = v4l2_pix_format_stride(&chan->pixfmt, buf->stride);
	if (buf->num_planes < 0)
		return buf->num_planes;

	buf->addr[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
	buf->addr[1] = buf->addr[0] + buf->stride[0] * chan->pixfmt.height;
	buf->addr[2] = buf->addr[1] + buf->stride[1] * chan->pixfmt.height;
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
		tegra_vi_input_enable(vi2, chan->input);

		err = v4l2_subdev_call(chan->input->sensor, video, s_stream, 1);
		if (err && err != -ENOIOCTLCMD) {
			dev_err(&vdev->dev, "Failed to start sensor\n");
			tegra_vi_input_disable(vi2, chan->input);
			return err;
		}

		/* Calibrate the input */
		err = tegra_vi_calibrate_input(vi2, chan->input);
		if (err) {
			dev_err(&vdev->dev, "Failed to calibrate input\n");
			v4l2_subdev_call(chan->input->sensor, video, s_stream, 0);
			tegra_vi_input_disable(vi2, chan->input);
			return err;
		}

	}
	chan->input->streaming_count++;
	mutex_unlock(&chan->input->lock);

	chan->sequence = 0;
	chan->streaming = true;
	INIT_COMPLETION(chan->streaming_completion);
	queue_work(vi2->wq, &chan->work);

	mutex_unlock(&chan->lock);

	return 0;
}

static int tegra_vi_videobuf_stop_streaming(struct vb2_queue *q)
{
	struct tegra_vi_channel *chan =
		container_of(q, struct tegra_vi_channel, vb);
	struct video_device *vdev = &chan->vdev;
	struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);
	struct tegra_vi_buffer *buf, *tmp;
	unsigned long flags;
	int err;

	/* First wait for the capture thread to finish */
	chan->streaming = false;
	wait_for_completion(&chan->streaming_completion);

	mutex_lock(&chan->lock);

	/* Stop the sensor */
	mutex_lock(&chan->input->lock);
	if (chan->input->streaming_count == 1) {
		err = v4l2_subdev_call(chan->input->sensor, video, s_stream, 0);
		if (err) {
			mutex_unlock(&chan->lock);
			return err;
		}

		tegra_vi_input_disable(vi2, chan->input);
	}
	chan->input->streaming_count--;
	mutex_unlock(&chan->input->lock);

	/* Clear the buffer queue */
	spin_lock_irqsave(&chan->vq_lock, flags);
	list_for_each_entry_safe(buf, tmp, &chan->capture, queue) {
		list_del_init(&buf->queue);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&chan->vq_lock, flags);

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

/* Let a channel wait for an event */
static int tegra_vi_channel_wait_for(struct tegra_vi_channel *chan, unsigned cond)
{
	struct video_device *vdev = &chan->vdev;
	struct platform_device *pdev = to_platform_device(vdev->v4l2_dev->dev);
	struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);
	u32 syncpt_val;
	int err;

	/* Update the sync point value */
	if (!nvhost_syncpt_read_ext_check(pdev, chan->syncpt_id, &syncpt_val))
		syncpt_val = nvhost_syncpt_incr_max_ext(
			pdev, chan->syncpt_id, 1);

	/* Queue a sync point raise on the condition */
	vi_writel((cond << 8) | chan->syncpt_id, &vi2->vi_regs->vi_incr_syncpt);

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
	int i;

	/* Get the next buffer, if none is available sleep a bit */
	while (!buf) {
		if (!chan->streaming)
			return NULL;

		spin_lock_irqsave(&chan->vq_lock, flags);

		if (!list_empty(&chan->capture)) {
			buf = list_first_entry(&chan->capture, struct tegra_vi_buffer, queue);
			list_del_init(&buf->queue);
		}

		spin_unlock_irqrestore(&chan->vq_lock, flags);

		if (!buf)
			usleep_range(100, 200);
	}

	/* Setup the DMA registers */
	for (i = 0; i < buf->num_planes; i++) {
		vi_writel(0, &chan->vi_regs->surface_offset[i].msb);
		vi_writel(buf->addr[i], &chan->vi_regs->surface_offset[i].lsb);
		vi_writel(buf->stride[i], &chan->vi_regs->surface_stride[i]);
	}

	return buf;
}

static void tegra_vi_channel_print_errors(struct tegra_vi_channel *chan)
{
	struct video_device *vdev = &chan->vdev;

	dev_err(&vdev->dev, "Failed to capture frame\n");
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
}

/* Capture loop */
void tegra_vi_channel_work(struct work_struct *work)
{
	struct tegra_vi_channel *chan =
		container_of(work, struct tegra_vi_channel, work);
	int err;


	mutex_lock(&chan->lock);

	/* Program DMA for first buffer */
	chan->active_buffer = tegra_vi_channel_set_next_buffer(chan);
	if (!chan->active_buffer)
		goto finish;

	/* Start the Pixel Parser */
	vi_writel(0xf005, &chan->mipi_regs->pp_command);
	/* Queue a buffer update on the next shot */
	vi_writel(1, &chan->vi_regs->single_shot);

	/* Wait for SOF */
	err = tegra_vi_channel_wait_for(chan, 9 + chan->id);
	if (err)
		goto stop_vi;

	/* Program DMA for the second buffer (in the shadow registers) */
	chan->pending_buffer = tegra_vi_channel_set_next_buffer(chan);
	if (!chan->pending_buffer)
		goto stop_vi;

	/* FIXME: IT should here check for chan->active_buffer but the last
	 *        sync point wait fails. As a workaround we stop one frame
	 *        earlier. */
	while (chan->pending_buffer) {
		struct tegra_vi_buffer *done_buffer;

		/* Wait for the write ACK */
		err = tegra_vi_channel_wait_for(chan, 6 + chan->id);
		if (err) {
			tegra_vi_channel_print_errors(chan);
			break;
		}

		/* The active buffer is done and then pending is now active */
		done_buffer = chan->active_buffer;
		chan->active_buffer = chan->pending_buffer;

		/* Setup the next pending buffer */
		chan->pending_buffer = tegra_vi_channel_set_next_buffer(chan);
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

	complete(&chan->streaming_completion);
}
