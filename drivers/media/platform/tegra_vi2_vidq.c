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
	printk("%s: entered\n", __func__);
	printk("%s: trying pf->width=%d, ROUND_STRIDE(pf->width)=%d, pixelformat=0x%x\n", __func__, 
		pf->width, ROUND_STRIDE(pf->width), pf->pixelformat);
	// FIXME: Need to modify for dual-link?
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
	// FIXME: Need to modify for dual-link?
	int planes, stride[3] = {};

	planes = v4l2_pix_format_stride(pf, stride);
	if (planes < 0)
		return planes;

	pf->sizeimage = (stride[0] + stride[1] + stride[2]) * pf->height;
	pf->bytesperline = stride[0];

	printk("%s: stride[0]=%d,[1]=%d,[2]=%d, pf->height=%d, pf->sizeimage=%d\n", __func__, stride[0], stride[1], stride[2], pf->height, pf->sizeimage);

	return 0;
}

/*
 * @queue_setup:	called from VIDIOC_REQBUFS and VIDIOC_CREATE_BUFS
 *			handlers before memory allocation, or, if
 *			*num_planes != 0, after the allocation to verify a
 *			smaller number of buffers. Driver should return
 *			the required number of buffers in *num_buffers, the
 *			required number of planes per buffer in *num_planes; the
 *			size of each plane should be set in the sizes[] array
 *			and optional per-plane allocator specific context in the
 *			alloc_ctxs[] array. When called from VIDIOC_REQBUFS,
 *			fmt == NULL, the driver has to use the currently
 *			configured format and *num_buffers is the total number
 *			of buffers, that are being allocated. When called from
 *			VIDIOC_CREATE_BUFS, fmt != NULL and it describes the
 *			target frame format. In this case *num_buffers are being
 *			allocated additionally to q->num_buffers.
 */
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

	printk("%s: entered\n", __func__);

	if (fmt) {
		if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		pf = &fmt->fmt.pix;
		printk("%s: got fmt arg\n", __func__);
	} else {
		pf = &chan->multifmt.composite_pf;
		printk("%s: got no fmt arg, using chan with width=%d, height=%d, pixelformat=%d, sizeimage=%d\n", __func__, chan->multifmt.composite_pf.width, chan->multifmt.composite_pf.height, chan->multifmt.composite_pf.pixelformat, chan->multifmt.composite_pf.sizeimage);
	}

	planes = v4l2_pix_format_stride(pf, stride);
	printk("%s: planes=%d\n", __func__, planes);
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

/*
 * @buf_init:		called once after allocating a buffer (in MMAP case)
 *			or after acquiring a new USERPTR buffer; drivers may
 *			perform additional buffer-related initialization;
 *			initialization failure (return != 0) will prevent
 *			queue setup from completing successfully; optional
 */
static int tegra_vi_videobuf_init(struct vb2_buffer *vb)
{
	struct tegra_vi_buffer *buf =
		container_of(vb, struct tegra_vi_buffer, vb);

	printk("%s: entered\n", __func__);

	INIT_LIST_HEAD(&buf->queue);

	return 0;
}

/*
 * @buf_prepare:	called every time the buffer is queued from userspace
 *			and from the VIDIOC_PREPARE_BUF ioctl; drivers may
 *			perform any initialization required before each hardware
 *			operation in this callback; if an error is returned, the
 *			buffer will not be queued in driver; optional
 */
static int tegra_vi_videobuf_prepare(struct vb2_buffer *vb)
{
	struct tegra_vi_channel *chan =
		container_of(vb->vb2_queue, struct tegra_vi_channel, vb);
	struct tegra_vi_buffer *buf =
		container_of(vb, struct tegra_vi_buffer, vb);
	unsigned height = chan->pixfmt.height;

	printk("%s: entered\n", __func__);

	if (vb->v4l2_buf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, vb2_plane_size(vb, 0));

	buf->num_planes = v4l2_pix_format_stride(&chan->multifmt.composite_pf, 
		buf->stride);
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

/*
 * @buf_queue:		passes buffer vb to the driver; driver may start
 *			hardware operation on this buffer; driver should give
 *			the buffer back by calling vb2_buffer_done() function;
 *			it is allways called after calling STREAMON ioctl;
 *			might be called before start_streaming callback if user
 *			pre-queued buffers before calling STREAMON
 */
static void tegra_vi_videobuf_queue(struct vb2_buffer *vb)
{
	struct tegra_vi_channel *chan =
		container_of(vb->vb2_queue, struct tegra_vi_channel, vb);
	struct tegra_vi_buffer *buf =
		container_of(vb, struct tegra_vi_buffer, vb);
	unsigned long flags;

	//printk("%s: entered\n", __func__);

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

/*
 * @start_streaming:	called once to enter 'streaming' state; the driver may
 *			receive buffers with @buf_queue callback before
 *			@start_streaming is called; the driver gets the number
 *			of already queued buffers in count parameter; driver
 *			can return an error if hardware fails or not enough
 *			buffers has been queued, in such case all buffers that
 *			have been already given by the @buf_queue callback are
 *			invalidated.
 */
static int tegra_vi_videobuf_start_streaming(struct vb2_queue *q,
					unsigned int count)
{
	struct tegra_vi_channel *chan =
		container_of(q, struct tegra_vi_channel, vb);
	struct video_device *vdev = &chan->vdev;
	struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);
	int err, i;

	printk("%s: entered\n", __func__);

	mutex_lock(&chan->lock);
	if (!chan->input) {
		mutex_unlock(&chan->lock);
		return -EINVAL;
	}

	// Start capturing on every input and connected sensor
	for (i = 0; i < chan->input->endpoint_count; i++) {
		mutex_lock(&chan->input->endpoint[i]->lock);

		if (chan->input->endpoint[i]->streaming_count == 0) {
			tegra_vi_input_start(vi2, chan->input->endpoint[i]);
			printk("%s: started input for ep %d\n", __func__, i);

			// Only call the subdev "start stream" once
			if (i == 0) {
				err = v4l2_subdev_call(chan->input->endpoint[i]->sensor, video, s_stream, 1);
				printk("%s: started stream on sensor for ep %d (err=%d)\n", __func__, i, err);
				if (err && err != -ENOIOCTLCMD) {
					dev_err(&vdev->dev, "Failed to start sensor attached to input %d\n", i);
					goto input_error;
				}
			}

			/* Calibrate the input */
			err = tegra_vi_calibrate_input(vi2, chan->input->endpoint[i]);
			printk("%s: calibrated input for ep %d (err=%d)\n", __func__, i, err);
			if (err) {
				dev_err(&vdev->dev, "Failed to calibrate input %d\n", i);
				v4l2_subdev_call(chan->input->endpoint[i]->sensor, video, s_stream, 0);
				goto input_error;
			}			
		}
		chan->input->endpoint[i]->streaming_count++;

		mutex_unlock(&chan->input->endpoint[i]->lock);
	}

	chan->sequence = 0;
	// FIXME: Can we start the same thread for 1 chan or do we need 1 per input?
	chan->work_th = kthread_run(tegra_vi_channel_capture_thread, chan,
				"tergra_vi.%d capture", chan->id);
	if (IS_ERR(chan->work_th)) {
		dev_err(&vdev->dev, "Failed to create capture thread\n");
		goto input_error;
	}

	mutex_unlock(&chan->lock);

	return 0;

input_error:	
	for (i = 0; i < chan->input->endpoint_count; i++) {
		tegra_vi_input_stop(vi2, chan->input->endpoint[i]);
		mutex_unlock(&chan->input->endpoint[i]->lock);
	}

	tegra_vi_channel_clear_queue(chan);
	mutex_unlock(&chan->lock);

	return err;
}

/*
 * @stop_streaming:	called when 'streaming' state must be disabled; driver
 *			should stop any DMA transactions or wait until they
 *			finish and give back all buffers it got from buf_queue()
 *			callback; may use vb2_wait_for_all_buffers() function
 */
static int tegra_vi_videobuf_stop_streaming(struct vb2_queue *q)
{
	struct tegra_vi_channel *chan =
		container_of(q, struct tegra_vi_channel, vb);
	struct video_device *vdev = &chan->vdev;
	struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);
	int err, i, p;

	printk("%s: entered.    STOPPING CAPTURE THREAD!!\n", __func__);

	/* First wait for the capture thread to finish */
	kthread_stop(chan->work_th);
	chan->work_th = NULL;
	chan->should_stop = false;

	mutex_lock(&chan->lock);

	/* Stop the sensor */
	for (i = 0; i < chan->input->endpoint_count; i++) {
		mutex_lock(&chan->input->endpoint[i]->lock);
		if (chan->input->endpoint[i]->streaming_count == 1) {
			err = v4l2_subdev_call(chan->input->endpoint[i]->sensor, video, s_stream, 0);
			//FIXME: Does it make sense to call the subdev_call twice on the same subdev?
			if (err)
				dev_warn(&vdev->dev, "Failed to stop sensor!\n");

			tegra_vi_input_stop(vi2, chan->input->endpoint[i]);
		}
		chan->input->endpoint[i]->streaming_count--;
		mutex_unlock(&chan->input->endpoint[i]->lock);

	}

	/* Clear the buffer queue */
	tegra_vi_channel_clear_queue(chan);
	/* Reset the channel logic */
	for (p = 0; p < chan->pp_count; p++) {
		vi_writel(0x1F, &chan->pp[p]->vi_regs->sw_reset);
		vi_writel(0x00, &chan->pp[p]->vi_regs->sw_reset);
	}

	mutex_unlock(&chan->lock);

	return 0;
}

/*
 * @wait_prepare:	release any locks taken while calling vb2 functions;
 *			it is called before an ioctl needs to wait for a new
 *			buffer to arrive; required to avoid a deadlock in
 *			blocking access type
 */
static void tegra_vi_videobuf_wait_prepare(struct vb2_queue *q)
{
	struct tegra_vi_channel *chan =
		container_of(q, struct tegra_vi_channel, vb);
	//unsigned long flags = chan->vq_flags;

	printk("%s: entered\n", __func__);

	// mutex_unlock(&chan->lock);
	// spin_unlock_irqrestore(&chan->vq_lock, flags);
}

/*
 * @wait_finish:	reacquire all locks released in the previous callback;
 *			required to continue operation after sleeping while
 *			waiting for a new buffer to arrive
 */
static void tegra_vi_videobuf_wait_finish(struct vb2_queue *q)
{
	struct tegra_vi_channel *chan =
		container_of(q, struct tegra_vi_channel, vb);
	//unsigned long flags = chan->vq_flags;

	printk("%s: entered\n", __func__);

	// mutex_lock(&chan->lock);
	// spin_lock_irqsave(&chan->vq_lock, flags);
}

const struct vb2_ops tegra_vi_qops = {
	.queue_setup = tegra_vi_videobuf_queue_setup,
	.buf_init = tegra_vi_videobuf_init,
	.buf_prepare = tegra_vi_videobuf_prepare,
	.buf_queue = tegra_vi_videobuf_queue,
	.start_streaming = tegra_vi_videobuf_start_streaming,
	.stop_streaming = tegra_vi_videobuf_stop_streaming,
	.wait_prepare = tegra_vi_videobuf_wait_prepare,
	.wait_finish = tegra_vi_videobuf_wait_finish,
};

// Called by capture_thread(): 
/* Queue a syncpt raise for a condition */
static void tegra_vi_channel_queue_syncpt(
	const struct tegra_vi_channel *chan, unsigned cond, u32 *syncpt_val, u32 incrs)
{
	const struct video_device *vdev = &chan->vdev;
	struct platform_device *pdev = to_platform_device(vdev->v4l2_dev->dev);
	const struct tegra_vi2 *vi2 =
		container_of(vdev->v4l2_dev, struct tegra_vi2, v4l2_dev);
	int err;

	printk("%s: entered.     cond=%d\n", __func__, cond);

	/* Update the sync point value */
	// FIXME: HOW TO SYNC??
	if (!(err = nvhost_syncpt_read_ext_check(pdev, chan->syncpt_id, syncpt_val)))
		*syncpt_val = nvhost_syncpt_incr_max_ext(
			pdev, chan->syncpt_id, incrs);

	printk("%s: err=%d\n", __func__, err);

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
	int i, p, dual_link_offset, wait_cnt = 0;

	/* Get the next buffer, if none is available sleep a bit */
	while (!buf) {
		if (chan->should_stop || kthread_should_stop()){
			printk("%s: chan or cap.thread should stop: chan->should_stop=%d. wait_cnt=%d\n", __func__, chan->should_stop, wait_cnt);
			return NULL;
		}

		spin_lock_irqsave(&chan->vq_lock, flags);

		if (!list_empty(&chan->capture)) {
			buf = list_first_entry(&chan->capture, struct tegra_vi_buffer, queue);
			list_del_init(&buf->queue);
		} else {
			//DBG
			//printk("%s: WAITING FOR A BUFFER...\n", __func__);
			wait_cnt++;
			//mdelay(1);
		}

		spin_unlock_irqrestore(&chan->vq_lock, flags);

		if (!missed && !buf)
			missed = true;
	}

	printk("%s: Got a buffer after waiting %d times\n", __func__, wait_cnt);

	/* Setup the DMA registers */
	for (i = 0; i < buf->num_planes; i++) {
		for (p = 0; p < chan->pp_count; p++) {
			//printk("%s: ------- DMA Config for PP %d ------- \n", __func__, p);
			vi_writel(0, &chan->pp[p]->vi_regs->surface_offset[i].msb);
			//printk("%s: SURFACE_OFFSET.MSB[%d]=%d\n", __func__, i, 0);
			dual_link_offset = p * (buf->stride[i] / 2);
			vi_writel(buf->addr[i] + dual_link_offset, 
				&chan->pp[p]->vi_regs->surface_offset[i].lsb);
			//printk("%s: SURFACE_OFFSET.LSB[%d]=%d\n", __func__, i, 
			//	buf->addr[i] + p * buf->stride[i]);
			vi_writel(buf->stride[i], &chan->pp[p]->vi_regs->surface_stride[i]);
			//printk("%s: SURFACE_STRIDE.[%d]=%d \n", __func__, i, buf->stride[i]);


			vi_writel(0, &chan->pp[p]->vi_regs->surface_bf_offset[i].msb);
			vi_writel(buf->bf_addr[i],
				&chan->pp[p]->vi_regs->surface_bf_offset[i].lsb);
		}
	}

	if (missed)
		chan->missed_buffer++;

	return buf;
}

static void tegra_vi_channel_clear_errors(struct tegra_vi_channel *chan)
{
	int i, p;

	for (p = 0; p < chan->pp_count; p++) {
		vi_writel(0xffffffff, &chan->pp[p]->vi_regs->error_status);
		vi_writel(0xffffffff, &chan->pp[p]->mipi_regs->status);
	}
	for (i = 0; i < chan->input->endpoint_count; i++) {
		if (chan->input->endpoint[i]->cil_regs[0]) {
			vi_writel(0xffffffff, &chan->input->endpoint[i]->cil_regs[0]->status);
			vi_writel(0xffffffff, &chan->input->endpoint[i]->cil_regs[0]->cil_status);
		}
		if (chan->input->endpoint[i]->cil_regs[1]) {
			vi_writel(0xffffffff, &chan->input->endpoint[i]->cil_regs[1]->status);
			vi_writel(0xffffffff, &chan->input->endpoint[i]->cil_regs[1]->cil_status);
		}
	}
	chan->missed_buffer = 0;
}

static void tegra_vi_channel_print_errors(struct tegra_vi_channel *chan)
{
	int i, p;
	struct video_device *vdev = &chan->vdev;

	/* PP Errors */
	for (p = 0; p < chan->pp_count; p++) {
		dev_err(&vdev->dev, "PP %d:\n", p);
		dev_err(&vdev->dev, "VI STATUS: 0x%08x\n",
			readl(&chan->pp[p]->vi_regs->error_status));
		dev_err(&vdev->dev, "PP STATUS: 0x%08x\n",
			readl(&chan->pp[p]->mipi_regs->status));
	}
	/* Endpoint Errors */
	for (i = 0; i < chan->input->endpoint_count; i++) {
		if (chan->input->endpoint[i]->cil_regs[0]) {
			dev_err(&vdev->dev, " Input %d:\n", i);
			dev_err(&vdev->dev, "CIL%c STATUS: 0x%08x\n",
				'A' + chan->input->endpoint[i]->id * 2,
				readl(&chan->input->endpoint[i]->cil_regs[0]->status));
			dev_err(&vdev->dev, "CIL%c CIL STATUS: 0x%08x\n",
				'A' + chan->input->endpoint[i]->id * 2,
				readl(&chan->input->endpoint[i]->cil_regs[0]->cil_status));
		}
		if (chan->input->endpoint[i]->cil_regs[1]) {
			dev_err(&vdev->dev, " Input %d:\n", i);
			dev_err(&vdev->dev, "CIL%c STATUS: 0x%08x\n",
				'B' + chan->input->endpoint[i]->id * 2,
				readl(&chan->input->endpoint[i]->cil_regs[1]->status));
			dev_err(&vdev->dev, "CIL%c CIL STATUS: 0x%08x\n",
				'B' + chan->input->endpoint[i]->id * 2,
				readl(&chan->input->endpoint[i]->cil_regs[1]->cil_status));
		}
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
	u32 syncpt_val[2];
	u32 incrs;
	int err0, err1, i, p;

	printk("%s: entered\n", __func__);
	printk("%s: endpoint_count=%d\n", __func__, chan->input->endpoint_count);
	//printk("%s: &chan->lock=0x%x\n", __func__, &chan->lock);

	mutex_lock(&chan->lock);
	printk("%s: acquired chan lock\n", __func__);

	/* Don't capture in broadcast mode it is broken */
	for (i = 0; i < chan->input->endpoint_count; i++) {
		if (chan->input->endpoint[i]->use_count > 1) {
			dev_err(&vdev->dev, "Broadcast capture is not working yet!\n");
			goto finish;
		}
	}

	incrs = chan->input->endpoint_count == 2 ? 2 : 1;
	printk("%s: incrs=%d\n", __func__, incrs);

	tegra_vi_channel_clear_errors(chan);

	// FIXME: HOW TO EXPAND FOR 2 CAPTURES??

	/* Program DMA for first buffer */
	chan->active_buffer = tegra_vi_channel_set_next_buffer(chan);
	if (!chan->active_buffer) {
		printk("%s: GOT NO ACTIVE BUFFER -> EXIT\n", __func__);
		goto finish;

	}
	printk("%s: got an active buffer from DMA\n", __func__);
/*
VI_CSI_PPA_FRAME_START Value=0x09 FIFO Depth=2
Valid SOF has been received by the camera (PPA) input. This
SOF is detected at the VI2 input.
*/
	// FIXME: Probably we expect only 1 SOF on PPA, nothing on PPB
	printk("%s: got syncpt before queueing SOF: %d\n", __func__, syncpt_val[0]);
	tegra_vi_channel_queue_syncpt(chan, 5, &syncpt_val[0], 1);
	printk("%s: got syncpt after queueing SOF: %d\n", __func__, syncpt_val[0]);

	printk("%s: queued syncpt for SOF on PPA (chan->pp[0]->id=%d)\n", __func__, chan->pp[0]->id);

	if (show_statistics)
		start_time = ktime_get();

	for (p=0; p < chan->pp_count; p++) {
		/* Start the Pixel Parser */
		vi_writel(0xf005, &chan->pp[p]->mipi_regs->pp_command);
		/* Queue a buffer update on the next shot */
		vi_writel(1, &chan->pp[p]->vi_regs->single_shot);
	}
	printk("%s: started PPs and queued a buffer update each\n", __func__);

	/* Wait for SOF */
	err0 = tegra_vi_channel_wait_for_syncpt(chan, syncpt_val[0]);
	printk("%s: wait for SOF: err=%d\n", __func__, err0);
	if (err0) {
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
		err0 = tegra_vi_channel_wait_for_syncpt(chan, syncpt_val[0]);
		printk("%s: wait for write ACK.0 err=%d\n", __func__, err0);
		if (err0) {
			if (!chan->should_stop) {
				dev_err(&vdev->dev,
					"Failed to capture half frame\n");
				tegra_vi_channel_print_errors(chan);
			}
			printk("%s: err0\n", __func__);
			break;
		}
		err1 = tegra_vi_channel_wait_for_syncpt(chan, syncpt_val[1]);
		printk("%s: wait for write ACK.1 err=%d\n", __func__, err1);
		if (err1) {
			if (!chan->should_stop) {
				dev_err(&vdev->dev,
					"Failed to capture half frame\n");
				tegra_vi_channel_print_errors(chan);
			}
			//DBG
			printk("%s: DBG: CONTINUE WITHOUT THIS HALF FRAME\n", __func__);
			printk("%s: err1\n", __func__);
			break;
		}
		if (err0 && err1) {
			if (!chan->should_stop) {
				dev_err(&vdev->dev,
					"Failed to capture frame\n");
				tegra_vi_channel_print_errors(chan);
			}
			printk("%s: Both captures failed, exiting..\n", __func__);
			break;
		}

		/* The active buffer is done and then pending is now active */
		done_buffer = chan->active_buffer;
		chan->active_buffer = chan->pending_buffer;

		/* Setup the next pending buffer */
		chan->pending_buffer = tegra_vi_channel_set_next_buffer(chan);
		if (!chan->pending_buffer)
			printk("%s: ERROR: Out of buffers! EXITING..\n", __func__);

/* 
VI_MWA_ACK_DONE Value=0x06 FIFO Depth=2
Predefined Operation Done for all WrAcks for a Frame condition.
The VI2 generates this syncpt when all WrAcks for a captured
frame from the camera (PPA) to memory are received.
*/
		for (p=0; p < chan->pp_count; p++) {
			printk("%s: got syncpt before queueing (p=%d): %d\n", __func__, p, syncpt_val[p]);
			tegra_vi_channel_queue_syncpt(chan, 7 + 8*chan->pp[p]->id, &syncpt_val[p], 1);//incrs);
			printk("%s: got syncpt after queueing (p=%d): %d\n", __func__, p, syncpt_val[p]);
		}
		for (p=0; p < chan->pp_count; p++) {
			if (chan->pending_buffer) {
				/* Queue a buffer update on the next shot */
				/* This triggers the shadow registers to be written into VI2 */
				vi_writel(1, &chan->pp[p]->vi_regs->single_shot);
			} else {
				vi_writel(0, &chan->pp[p]->vi_regs->single_shot);
				if (chan->active_buffer) {
					/* Last frame stop the pixel parser once done */
					vi_writel(0xf002, &chan->pp[p]->mipi_regs->pp_command);
				}
			}
		}
		/* Return the last active buffer */
		do_gettimeofday(&done_buffer->vb.v4l2_buf.timestamp);
		done_buffer->vb.v4l2_buf.sequence = chan->sequence++;
		vb2_buffer_done(&done_buffer->vb, VB2_BUF_STATE_DONE);
	}

stop_vi:
	for (p=0; p < chan->pp_count; p++) {
		/* Stop capture shot */
		vi_writel(0, &chan->pp[p]->vi_regs->single_shot);
		/* Stop the Pixel Parser */
		vi_writel(0xf003, &chan->pp[p]->mipi_regs->pp_command);
	}

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
	printk("%s: goto finish:\n", __func__);
	mutex_unlock(&chan->lock);

	/* Wait for kthread_stop() to be called */
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}

	return 0;
}
