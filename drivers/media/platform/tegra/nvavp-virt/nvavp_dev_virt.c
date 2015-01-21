/*
 * drivers/media/video/tegra/nvavp/nvavp_dev_hv.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <trace/events/nvavp.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "nvavp_dev_ivc.h"

#include "../nvavp/nvavp_os.h"

#define TEGRA_NVAVP_NAME "avp-virt"
#define SCLK_BOOST_RATE		40000000
#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
#define NVAVP_AUDIO_CHANNEL		1
#define IS_AUDIO_CHANNEL_ID(channel_id)	\
	(channel_id == NVAVP_AUDIO_CHANNEL ? 1 : 0)
#endif

#define NVAVP_NUM_CHANNELS		2

#define NVAVP_VIDEO_CHANNEL		0
#define IS_VIDEO_CHANNEL_ID(channel_id)	\
	(channel_id == NVAVP_VIDEO_CHANNEL ? 1 : 0)

static bool boost_sclk;

struct nvavp_info {
	u32				syncpt_id;
	struct miscdevice		video_misc_dev;
#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
	struct miscdevice		audio_misc_dev;
#endif
	/* used for dvfs */
	struct clk			*sclk;
	/* client to change number of min online cpus*/
	struct task_struct		*init_task;
	/*IVC */
	struct ivc_dev			*ivcdev;
	struct ivc_ctxt			*ivc_ctx;
	struct mutex			open_lock;
	struct platform_device		*nvavp_virt;
};

struct nvavp_clientctx {
	struct nvavp_reloc relocs[NVAVP_MAX_RELOCATION_COUNT];
	int num_relocs;
	struct nvavp_info *nvavp;
	int channel_id;
	struct mutex pushbuffer_lock;
};

static struct nvavp_info *nvavp_info_ctx;

static int tegra_nvavp_open(struct nvavp_info *nvavp,
		struct nvavp_clientctx **client, int channel_id)
{
	struct nvavp_clientctx *clientctx;
	int ret = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	dev_dbg(&nvavp->nvavp_virt->dev, "%s: ++\n", __func__);

	clientctx = kzalloc(sizeof(*clientctx), GFP_KERNEL);
	if (!clientctx)
		return -ENOMEM;

	pr_debug("tegra_nvavp_open channel_id (%d)\n", channel_id);

	clientctx->channel_id = channel_id;
	mutex_init(&clientctx->pushbuffer_lock);

	msg.msg = NVAVP_CONNECT;
	msg.channel_id = channel_id;

	ret = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (ret != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		ret = -EINVAL;
	}

	nvavp->syncpt_id = msg.syncpt.id;

	clientctx->nvavp = nvavp;
	*client = clientctx;

	return ret;
}

static int nvavp_set_clock_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int err = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_SET_CLOCK;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	if (copy_from_user(&(msg.params.clock_params), (void __user *)arg,
				sizeof(struct nvavp_clock_args)))
		return -EFAULT;

	dev_dbg(&nvavp->nvavp_virt->dev, "%s: clk_id=%d, clk_rate=%u\n",
			__func__, msg.params.clock_params.id,
			msg.params.clock_params.rate);
	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		return -EINVAL;
	}

	trace_nvavp_set_clock_ioctl(clientctx->channel_id,
			msg.params.clock_params.id,
			msg.params.clock_params.rate);

	if (copy_to_user((void __user *)arg, &(msg.params.clock_params),
				sizeof(struct nvavp_clock_args)))
		return -EFAULT;

	return 0;
}

static int nvavp_get_clock_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int err = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_GET_CLOCK;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	if (copy_from_user(&(msg.params.clock_params), (void __user *)arg,
				sizeof(struct nvavp_clock_args)))
		return -EFAULT;

	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		return -EINVAL;
	}

	trace_nvavp_get_clock_ioctl(clientctx->channel_id,
			msg.params.clock_params.id,
			msg.params.clock_params.rate);

	if (copy_to_user((void __user *)arg, &(msg.params.clock_params),
				sizeof(struct nvavp_clock_args)))
		return -EFAULT;

	return 0;
}

static int
nvavp_channel_open(struct file *filp, struct nvavp_channel_open_args *arg)
{
	int err = 0;

	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_CONNECT;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev,
				"cannot complete channel open\n");
		err = -EINVAL;
		nvavp->init_task = NULL;
	} else {
		nvavp->init_task = current;
		nvavp->syncpt_id = msg.syncpt.id;
	}

	return err;
}

static int nvavp_get_syncpointid_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	u32 id = nvavp->syncpt_id;

	if (_IOC_DIR(cmd) & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &id, sizeof(u32)))
			return -EFAULT;
		else
			return 0;
	}

	trace_nvavp_get_syncpointid_ioctl(clientctx->channel_id, id);

	return -EFAULT;
}
static int nvavp_pushbuffer_submit_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	struct nvavp_pushbuffer_submit_hdr hdr;
	u32 *cmdbuf_data;
	struct dma_buf *cmdbuf_dmabuf;
	struct dma_buf_attachment *cmdbuf_attach;
	struct sg_table *cmdbuf_sgt;
	int ret = 0, i;
	phys_addr_t phys_addr;
	unsigned long virt_addr;
	struct nvavp_pushbuffer_submit_hdr *user_hdr =
			(struct nvavp_pushbuffer_submit_hdr *) arg;
	struct nvavp_syncpt syncpt;
	struct ivc_msg msg;

	mutex_lock(&clientctx->pushbuffer_lock);

	syncpt.id = NVSYNCPT_INVALID;
	syncpt.value = 0;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(&hdr, (void __user *)arg,
			sizeof(struct nvavp_pushbuffer_submit_hdr))) {
			mutex_unlock(&clientctx->pushbuffer_lock);
			return -EFAULT;
		}
	}

	if (!hdr.cmdbuf.mem) {
		mutex_unlock(&clientctx->pushbuffer_lock);
		return 0;
	}

	if (copy_from_user(clientctx->relocs, (void __user *)hdr.relocs,
			sizeof(struct nvavp_reloc) * hdr.num_relocs)) {
		mutex_unlock(&clientctx->pushbuffer_lock);
		return -EFAULT;
	}

	cmdbuf_dmabuf = dma_buf_get(hdr.cmdbuf.mem);
	if (IS_ERR(cmdbuf_dmabuf)) {
		dev_err(&nvavp->nvavp_virt->dev,
			"invalid cmd buffer handle %08x\n", hdr.cmdbuf.mem);
		mutex_unlock(&clientctx->pushbuffer_lock);
		return PTR_ERR(cmdbuf_dmabuf);
	}

	cmdbuf_attach = dma_buf_attach(cmdbuf_dmabuf, &nvavp->nvavp_virt->dev);
	if (IS_ERR(cmdbuf_attach)) {
		dev_err(&nvavp->nvavp_virt->dev,
				"cannot attach cmdbuf_dmabuf\n");
		ret = PTR_ERR(cmdbuf_attach);
		goto err_dmabuf_attach;
	}

	cmdbuf_sgt = dma_buf_map_attachment(cmdbuf_attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(cmdbuf_sgt)) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot map cmdbuf_dmabuf\n");
		ret = PTR_ERR(cmdbuf_sgt);
		goto err_dmabuf_map;
	}

	phys_addr = sg_dma_address(cmdbuf_sgt->sgl);

	virt_addr = (unsigned long)dma_buf_vmap(cmdbuf_dmabuf);
	if (!virt_addr) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot vmap cmdbuf_dmabuf\n");
		ret = -ENOMEM;
		goto err_dmabuf_vmap;
	}

	cmdbuf_data = (u32 *)(virt_addr + hdr.cmdbuf.offset);

	for (i = 0; i < hdr.num_relocs; i++) {
		struct dma_buf *target_dmabuf;
		struct dma_buf_attachment *target_attach;
		struct sg_table *target_sgt;
		u32 *reloc_addr, target_phys_addr;

		if (clientctx->relocs[i].cmdbuf_mem != hdr.cmdbuf.mem) {
			dev_err(&nvavp->nvavp_virt->dev,
				"reloc info does not match target bufferID\n");
			ret = -EPERM;
			goto err_reloc_info;
		}

		reloc_addr = cmdbuf_data +
			     (clientctx->relocs[i].cmdbuf_offset >> 2);
#if  !defined(CONFIG_TEGRA_NVAVP_ENCRYPTED_STREAMS)
		/* Exist criteria for encrypted streams */
		/* NVE276_VIDEO_SET_AES_PARAMS correponds to 0xC0 + 30) */
		if (*(reloc_addr - 1) == 0xDE) {
			dev_err(&nvavp->nvavp_virt->dev,
				"encrypted bitstream is not supported\n");
			ret = -EINVAL;
			goto target_dmabuf_fail;
		}
#endif
		target_dmabuf = dma_buf_get(clientctx->relocs[i].target);
		if (IS_ERR(target_dmabuf)) {
			ret = PTR_ERR(target_dmabuf);
			goto target_dmabuf_fail;
		}
		target_attach = dma_buf_attach(target_dmabuf,
					       &nvavp->nvavp_virt->dev);
		if (IS_ERR(target_attach)) {
			ret = PTR_ERR(target_attach);
			goto target_attach_fail;
		}
		target_sgt = dma_buf_map_attachment(target_attach,
						    DMA_BIDIRECTIONAL);
		if (IS_ERR(target_sgt)) {
			ret = PTR_ERR(target_sgt);
			goto target_map_fail;
		}

		target_phys_addr = sg_dma_address(target_sgt->sgl);
		if (!target_phys_addr)
			target_phys_addr = sg_phys(target_sgt->sgl);
		target_phys_addr += clientctx->relocs[i].target_offset;
		writel(target_phys_addr, reloc_addr);
		dma_buf_unmap_attachment(target_attach, target_sgt,
					 DMA_BIDIRECTIONAL);
target_map_fail:
		dma_buf_detach(target_dmabuf, target_attach);
target_attach_fail:
		dma_buf_put(target_dmabuf);
target_dmabuf_fail:
		if (ret != 0)
			goto err_reloc_info;
	}

	trace_nvavp_pushbuffer_submit_ioctl(clientctx->channel_id,
				hdr.cmdbuf.mem, hdr.cmdbuf.offset,
				hdr.cmdbuf.words, hdr.num_relocs, hdr.flags);

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_PUSHBUFFER_WRITE;
	msg.channel_id = clientctx->channel_id;
	msg.params.pushbuffer_params.addr = phys_addr + hdr.cmdbuf.offset;
	msg.params.pushbuffer_params.count = hdr.cmdbuf.words;
	msg.params.pushbuffer_params.flags = hdr.flags & NVAVP_UCODE_EXT;

	ret = nvavp_ivc(nvavp->ivc_ctx, &msg);

	if (ret != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev,
				"cannot pass Pushbuffer arguments\n");
		ret = -EINVAL;
		goto err_reloc_info;
	}

	if (hdr.syncpt) {
		if (copy_to_user((void __user *)user_hdr->syncpt, &msg.syncpt,
				sizeof(struct nvavp_syncpt))) {
			ret = -EFAULT;
			goto err_reloc_info;
		}
	}

err_reloc_info:
	dma_buf_vunmap(cmdbuf_dmabuf, (void *)virt_addr);
err_dmabuf_vmap:
	dma_buf_unmap_attachment(cmdbuf_attach, cmdbuf_sgt, DMA_BIDIRECTIONAL);
err_dmabuf_map:
	dma_buf_detach(cmdbuf_dmabuf, cmdbuf_attach);
err_dmabuf_attach:
	dma_buf_put(cmdbuf_dmabuf);

	mutex_unlock(&clientctx->pushbuffer_lock);

	return ret;
}

static int nvavp_map_iova(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int err = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_MAP_IOVA;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	if (copy_from_user(&(msg.params.map_params), (void __user *)arg,
		sizeof(struct nvavp_map_args))) {
		dev_err(&nvavp->nvavp_virt->dev,
			"failed to copy memory handle\n");
		return -EFAULT;
	}
	if (!msg.params.map_params.fd) {
		dev_err(&nvavp->nvavp_virt->dev,
			"invalid memory handle %08x\n",
			msg.params.map_params.fd);
		return -EINVAL;
	}

	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		return -EINVAL;
	}

	trace_nvavp_map_iova(clientctx->channel_id,
			msg.params.map_params.fd,
			msg.params.map_params.addr);

	if (copy_to_user((void __user *)arg, &(msg.params.map_params),
		sizeof(struct nvavp_map_args))) {
		dev_err(&nvavp->nvavp_virt->dev,
			"failed to copy phys addr\n");
		return -EFAULT;
	}

	return err;
}

static int nvavp_unmap_iova(struct file *filp, unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int err = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_UNMAP_IOVA;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	if (copy_from_user(&(msg.params.map_params), (void __user *)arg,
		sizeof(struct nvavp_map_args))) {
		dev_err(&nvavp->nvavp_virt->dev,
			"failed to copy memory handle\n");
		return -EFAULT;
	}

	trace_nvavp_unmap_iova(clientctx->channel_id,
			msg.params.map_params.fd,
			msg.params.map_params.addr);

	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		return -EINVAL;
	}

	return err;
}

static int nvavp_set_min_online_cpus_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int err = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_SET_MIN_CPU_ONLINE;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	if (copy_from_user(&(msg.params.numcpu_params), (void __user *)arg,
					sizeof(struct nvavp_num_cpus_args)))
		return -EFAULT;

	dev_dbg(&nvavp->nvavp_virt->dev, "%s: min_online_cpus=%d\n",
			__func__, msg.params.numcpu_params.min_online_cpus);

	trace_nvavp_set_min_online_cpus_ioctl(clientctx->channel_id,
				msg.params.numcpu_params.min_online_cpus);

	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		return -EINVAL;
	}

	return err;
}

static int nvavp_force_clock_stay_on_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int err = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_FORCE_CLOCK_STAY_ON;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	if (copy_from_user(&(msg.params.state_params), (void __user *)arg,
			   sizeof(struct nvavp_clock_stay_on_state_args)))
		return -EFAULT;

	dev_dbg(&nvavp->nvavp_virt->dev, "%s: state=%d\n",
		__func__, msg.params.state_params.state);

	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		return -EINVAL;
	}

	return err;
}
static int nvavp_wake_avp_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int err = 0;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_WAKE;
	msg.channel_id = NVAVP_VIDEO_CHANNEL;

	err = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (err != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev, "cannot complete avp IVC\n");
		return -EINVAL;
	}

	return err;
}

static long tegra_nvavp_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	u8 buf[NVAVP_IOCTL_CHANNEL_MAX_ARG_SIZE];

	if (_IOC_TYPE(cmd) != NVAVP_IOCTL_MAGIC ||
			_IOC_NR(cmd) < NVAVP_IOCTL_MIN_NR ||
			_IOC_NR(cmd) > NVAVP_IOCTL_MAX_NR)
		return -EFAULT;

	switch (cmd) {
	case NVAVP_IOCTL_SET_NVMAP_FD:
		break;
	case NVAVP_IOCTL_GET_SYNCPOINT_ID:
		ret = nvavp_get_syncpointid_ioctl(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_SET_CLOCK:
		ret = nvavp_set_clock_ioctl(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_GET_CLOCK:
		ret = nvavp_get_clock_ioctl(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_WAKE_AVP:
		ret = nvavp_wake_avp_ioctl(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_FORCE_CLOCK_STAY_ON:
		ret = nvavp_force_clock_stay_on_ioctl(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_ENABLE_AUDIO_CLOCKS:
		ret = 0;
		break;
	case NVAVP_IOCTL_DISABLE_AUDIO_CLOCKS:
		ret = 0;
		break;
	case NVAVP_IOCTL_SET_MIN_ONLINE_CPUS:
		ret = nvavp_set_min_online_cpus_ioctl(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_MAP_IOVA:
		ret = nvavp_map_iova(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_UNMAP_IOVA:
		ret = nvavp_unmap_iova(filp, arg);
		break;
	case NVAVP_IOCTL_PUSH_BUFFER_SUBMIT:
		ret = nvavp_pushbuffer_submit_ioctl(filp, cmd, arg);
		break;
	case NVAVP_IOCTL_CHANNEL_OPEN:
		ret = nvavp_channel_open(filp, (void *)buf);
		if (ret == 0)
			ret = copy_to_user((void __user *)arg, buf,
					_IOC_SIZE(cmd));
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tegra_nvavp_release(struct nvavp_clientctx *clientctx,
		int channel_id)
{
	int ret = 0;
	struct nvavp_info *nvavp = clientctx->nvavp;
	struct ivc_msg msg;

	memset(&msg, 0, sizeof(struct ivc_msg));

	msg.msg = NVAVP_DISCONNECT;
	msg.channel_id = channel_id;

	ret = nvavp_ivc(nvavp->ivc_ctx, &msg);
	if (ret != NVAVP_ERR_OK) {
		dev_err(&nvavp->nvavp_virt->dev,
				"cannot disconnect succesfully avp IVC\n");
		ret = -EINVAL;
	}

	nvavp->init_task = NULL;

	mutex_destroy(&clientctx->pushbuffer_lock);
	kfree(clientctx);
	return ret;
}

static int tegra_nvavp_video_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct nvavp_info *nvavp = dev_get_drvdata(miscdev->parent);
	struct nvavp_clientctx *clientctx = NULL;
	int ret = 0;

	pr_debug("tegra_nvavp_video_open NVAVP_VIDEO_CHANNEL\n");

	nonseekable_open(inode, filp);

	mutex_lock(&nvavp->open_lock);
	ret = tegra_nvavp_open(nvavp, &clientctx, NVAVP_VIDEO_CHANNEL);
	filp->private_data = clientctx;
	mutex_unlock(&nvavp->open_lock);

	return ret;
}

#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
static int tegra_nvavp_audio_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct nvavp_info *nvavp = dev_get_drvdata(miscdev->parent);
	struct nvavp_clientctx *clientctx;
	int ret = 0;

	pr_debug("tegra_nvavp_audio_open NVAVP_AUDIO_CHANNEL\n");

	nonseekable_open(inode, filp);

	mutex_lock(&nvavp->open_lock);
	ret = tegra_nvavp_open(nvavp, &clientctx, NVAVP_AUDIO_CHANNEL);
	filp->private_data = clientctx;
	mutex_unlock(&nvavp->open_lock);

	return ret;
}

int tegra_nvavp_audio_client_open(nvavp_clientctx_t *clientctx)
{
	struct nvavp_info *nvavp = nvavp_info_ctx;
	int ret = 0;

	mutex_lock(&nvavp->open_lock);
	ret = tegra_nvavp_open(nvavp, (struct nvavp_clientctx **)clientctx,
			NVAVP_AUDIO_CHANNEL);
	mutex_unlock(&nvavp->open_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tegra_nvavp_audio_client_open);
#endif


static int tegra_nvavp_video_release(struct inode *inode, struct file *filp)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int ret = 0;

	mutex_lock(&nvavp->open_lock);
	filp->private_data = NULL;
	ret = tegra_nvavp_release(clientctx, NVAVP_VIDEO_CHANNEL);
	mutex_unlock(&nvavp->open_lock);

	return ret;
}

#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
static int tegra_nvavp_audio_release(struct inode *inode,
		struct file *filp)
{
	struct nvavp_clientctx *clientctx = filp->private_data;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int ret = 0;

	mutex_lock(&nvavp->open_lock);
	filp->private_data = NULL;
	ret = tegra_nvavp_release(clientctx, NVAVP_AUDIO_CHANNEL);
	mutex_unlock(&nvavp->open_lock);

	return ret;
}

int tegra_nvavp_audio_client_release(nvavp_clientctx_t client)
{
	struct nvavp_clientctx *clientctx = client;
	struct nvavp_info *nvavp = clientctx->nvavp;
	int ret = 0;

	mutex_lock(&nvavp->open_lock);
	ret = tegra_nvavp_release(clientctx, NVAVP_AUDIO_CHANNEL);
	mutex_unlock(&nvavp->open_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tegra_nvavp_audio_client_release);
#endif

static const struct file_operations tegra_video_nvavp_fops = {
	.owner		= THIS_MODULE,
	.open		= tegra_nvavp_video_open,
	.release	= tegra_nvavp_video_release,
	.unlocked_ioctl	= tegra_nvavp_ioctl,
};

#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
static const struct file_operations tegra_audio_nvavp_fops = {
	.owner          = THIS_MODULE,
	.open           = tegra_nvavp_audio_open,
	.release        = tegra_nvavp_audio_release,
	.unlocked_ioctl = tegra_nvavp_ioctl,
};
#endif

static struct of_device_id tegra_nvavp_of_match[] = {
	{ .compatible = "nvidia,tegra124-avp-virt", },
	{},
};

static ssize_t boost_sclk_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", boost_sclk);
}

static ssize_t boost_sclk_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct nvavp_info *nvavp = platform_get_drvdata(ndev);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val)
		clk_set_rate(nvavp->sclk, SCLK_BOOST_RATE);
	else if (!val)
		clk_set_rate(nvavp->sclk, 0);

	boost_sclk = val;

	return count;
}

static DEVICE_ATTR(boost_sclk, S_IRUGO | S_IWUSR, boost_sclk_show,
		boost_sclk_store);

static struct device_dma_parameters nvavp_dma_parameters = {
	.max_segment_size = UINT_MAX,
};

static int tegra_nvavp_probe(struct platform_device *ndev)
{
	struct nvavp_info *nvavp;
	int ret = 0;
	struct device_node *np;
	struct ivc_dev *ivcdev;
	struct ivc_ctxt *ctxt = NULL;
	const struct of_device_id *match;

	match = of_match_device(tegra_nvavp_of_match, &ndev->dev);
	if (!match) {
		dev_err(&ndev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	np = ndev->dev.of_node;

	nvavp = kzalloc(sizeof(struct nvavp_info), GFP_KERNEL);
	if (!nvavp) {
		dev_err(&ndev->dev, "cannot allocate avp_info\n");
		ret = -ENOMEM;
		goto err_alloc_fail;
	}

	memset(nvavp, 0, sizeof(*nvavp));

	ndev->dev.dma_parms = &nvavp_dma_parameters;

	ivcdev = nvavp_ivc_init(&ndev->dev);
	if (IS_ERR_OR_NULL(ivcdev)) {
		if (ivcdev == ERR_PTR(-EPROBE_DEFER))
			ret = -EPROBE_DEFER;
		else
			ret = -ENODEV;
		goto exit_ivc_fail;
	}

	ctxt = nvavp_ivc_alloc_ctxt(ivcdev);
	if (IS_ERR_OR_NULL(ctxt)) {
		ret = -ENOMEM;
		goto exit_ivc_alloc_fail;
	}

	nvavp->ivcdev = ivcdev;
	nvavp->ivc_ctx = ctxt;

	mutex_init(&nvavp->open_lock);

	nvavp->video_misc_dev.minor = MISC_DYNAMIC_MINOR;
	nvavp->video_misc_dev.name = "tegra_avpchannel";
	nvavp->video_misc_dev.fops = &tegra_video_nvavp_fops;
	nvavp->video_misc_dev.mode = S_IRWXUGO;
	nvavp->video_misc_dev.parent = &ndev->dev;

	ret = misc_register(&nvavp->video_misc_dev);
	if (ret) {
		dev_err(&ndev->dev, "unable to register misc device!\n");
		goto err_misc_reg;
	}

#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
	nvavp->audio_misc_dev.minor = MISC_DYNAMIC_MINOR;
	nvavp->audio_misc_dev.name = "tegra_audio_avpchannel";
	nvavp->audio_misc_dev.fops = &tegra_audio_nvavp_fops;
	nvavp->audio_misc_dev.mode = S_IRWXUGO;
	nvavp->audio_misc_dev.parent = &ndev->dev;

	ret = misc_register(&nvavp->audio_misc_dev);
	if (ret) {
		dev_err(&ndev->dev, "unable to register misc device!\n");
		goto err_audio_misc_reg;
	}
#endif

	nvavp->nvavp_virt = ndev;
	platform_set_drvdata(ndev, nvavp);

	ret = device_create_file(&ndev->dev, &dev_attr_boost_sclk);
	if (ret) {
		dev_err(&ndev->dev,
				"%s: device_create_file failed\n", __func__);
		goto err_dev_create_fail;
	}
	nvavp_info_ctx = nvavp;

	dev_err(&ndev->dev, "Node device %p\n", &ndev->dev);

	return 0;
err_dev_create_fail:
#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
	misc_deregister(&nvavp->audio_misc_dev);
err_audio_misc_reg:
#endif
	misc_deregister(&nvavp->video_misc_dev);
err_misc_reg:
	if (ctxt)
		nvavp_ivc_free_ctxt(ctxt);
exit_ivc_alloc_fail:
	nvavp_ivc_deinit(ivcdev);
exit_ivc_fail:
	kfree(nvavp);
err_alloc_fail:
	dev_err(&ndev->dev, "tegra nvavp probe failed, e=%d", ret);
	return ret;
}

static int tegra_nvavp_remove(struct platform_device *ndev)
{
	struct nvavp_info *nvavp = platform_get_drvdata(ndev);

	if (!nvavp)
		return 0;

	device_remove_file(&ndev->dev, &dev_attr_boost_sclk);

	misc_deregister(&nvavp->video_misc_dev);

#if defined(CONFIG_TEGRA_NVAVP_AUDIO)
	misc_deregister(&nvavp->audio_misc_dev);
#endif
	nvavp_ivc_free_ctxt(nvavp->ivc_ctx);
	nvavp_ivc_deinit(nvavp->ivcdev);
	kfree(nvavp);
	return 0;
}

#ifdef CONFIG_PM
static int tegra_nvavp_resume(struct device *dev)
{
	return 0;
}

static int tegra_nvavp_suspend(struct device *dev)
{
	return 0;
}

static int tegra_nvavp_runtime_resume(struct device *dev)
{
	return 0;
}

static int tegra_nvavp_runtime_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops nvavp_pm_ops = {
	.runtime_suspend = tegra_nvavp_runtime_suspend,
	.runtime_resume = tegra_nvavp_runtime_resume,
	.suspend = tegra_nvavp_suspend,
	.resume = tegra_nvavp_resume,
};

#define NVAVP_PM_OPS	(&nvavp_pm_ops)

#else /* CONFIG_PM */

#define NVAVP_PM_OPS	NULL

#endif /* CONFIG_PM */

static struct platform_driver tegra_nvavp_driver = {
	.driver	= {
		.name	= TEGRA_NVAVP_NAME,
		.owner	= THIS_MODULE,
		.pm	= NVAVP_PM_OPS,
		.of_match_table = of_match_ptr(tegra_nvavp_of_match),
	},
	.probe		= tegra_nvavp_probe,
	.remove		= tegra_nvavp_remove,
};

module_platform_driver(tegra_nvavp_driver);

MODULE_AUTHOR("Deepak Kamurthy");
MODULE_DESCRIPTION("Channel based AVP driver for Tegra");
MODULE_VERSION("1.0");
MODULE_LICENSE("Dual BSD/GPL");
