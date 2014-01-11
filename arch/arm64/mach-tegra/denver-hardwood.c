/*
 *  arch/arm64/mach-tegra/denver-hardwoord.c
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>

#include <asm/cacheflush.h>

#include "denver-knobs.h"

/* #define MY_DEBUG 1 */

#ifdef MY_DEBUG
#define DBG_PRINT(format, arg...) \
	pr_info("hardwood: %s: " format "\n", __func__, ##arg)
#else
#define DBG_PRINT(format, arg...)
#endif

#include "denver-hardwood.h"

#define TRACER_OSDUMP_BUFFER_CMD		100
#define TRACER_OSDUMP_BUFFER_CMD_DATA	101

#define CORE_GUARD_MASK \
	~(BIT(HARDWOOD_GET_IP_ADDRESS) | \
	BIT(HARDWOOD_ENABLE_TRACE_DUMP) | \
	BIT(HARDWOOD_DISABLE_TRACE_DUMP) | \
	BIT(HARDWOOD_SET_TRACER_MASK) | \
	BIT(HARDWOOD_CLR_TRACER_MASK) | \
	/* Below are called during cleanup */ \
	BIT(HARDWOOD_GET_STATUS) | \
	BIT(HARDWOOD_GET_BYTES_USED) | \
	BIT(HARDWOOD_GET_PHYS_ADDR) | \
	BIT(HARDWOOD_RELEASE_BUFFER) | \
	BIT(HARDWOOD_WAKEUP_READERS))

#define HW_CMD(c, b, m) ((c) | ((b) << 8) | ((m) << 16))

struct hardwood_buf {
	void *va;
	dma_addr_t pa;
};

struct hardwood_device {
	int cpu;
	char name[32];
	struct miscdevice dev;
	struct irqaction irq;
	wait_queue_head_t wait_q;
	int signaled;
	ulong buf_status;
	spinlock_t buf_status_lock;
	struct hardwood_buf bufs[N_BUFFER];
} hardwood_devs[N_CPU];

static int minor_map[N_CPU] = { -1 };

static int TRACER_IRQS[] = { 48, 54 };

static bool hardwood_init_done;
static DEFINE_SPINLOCK(hardwood_init_lock);

static bool hardwood_supported;

#define CPU_GUARD(target, error) \
{ \
	int core = smp_processor_id(); \
	BUG_ON(target >= N_CPU); \
	if (core != target) { \
		pr_err("%s: invalid target core: %d (cur = %d)\n", \
				__func__, target, core); \
		return error; \
	} \
}

static void hardwood_late_init(void);

static irqreturn_t hardwood_handler(int irq, void *dev_id)
{
	struct hardwood_device *dev = (struct hardwood_device *) dev_id;

	dev->signaled = 1;
	wake_up_interruptible(&dev->wait_q);

	DBG_PRINT("core%d is interrupted\n", dev->cpu);

	return IRQ_HANDLED;
}

static int hardwood_open(struct inode *inode, struct file *file)
{
	int cpu;
	int found = 0;
	int minor = iminor(inode);

	if (!hardwood_supported) {
		pr_warn("hardwood is not available.\n");
		return -ENOENT;
	}

	if (!hardwood_init_done)
		hardwood_late_init();

	for (cpu = 0; cpu < N_CPU; ++cpu)
		if (minor_map[cpu] == minor) {
			found = 1;
			break;
		}
	BUG_ON(!found);
	file->private_data = &hardwood_devs[cpu];
	return nonseekable_open(inode, file);
}

static void hw_run_cmd(u64 data)
{
	asm volatile (
	"	sys 0, c11, c0, 1, %0\n"
	"	sys 0, c11, c0, 0, %1\n"
	:
	: "r"(data), "r" (TRACER_OSDUMP_BUFFER_CMD)
	);
}

static void hw_set_data(u64 data)
{
	asm volatile (
	"	sys 0, c11, c0, 1, %0\n"
	"	sys 0, c11, c0, 0, %1\n"
	:
	: "r" (data), "r" (TRACER_OSDUMP_BUFFER_CMD_DATA)
	);
}

static void hw_get_data(u64 *data)
{
	asm volatile ("sysl %0, 0, c11, c0, 0" : "=r" (*data));
}

long hardwood_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 trace_cmd;
	struct hardwood_cmd op;

	if (copy_from_user(&op, (void __user *)arg, sizeof(op)))
		return -EFAULT;

	if (BIT(cmd) & CORE_GUARD_MASK)
		CPU_GUARD(op.core_id, -EINVAL);

	if (op.buffer_id >= N_BUFFER)
		return -EINVAL;

	trace_cmd = HW_CMD(op.core_id, op.buffer_id, cmd);

	switch (cmd) {
	/* CMD with input argument */
	case HARDWOOD_SET_PHYS_ADDR:
	case HARDWOOD_SET_BUFFER_SIZE:
		hw_set_data(op.data);
		hw_run_cmd(trace_cmd);
		break;

	/* CMD with output argument */
	case HARDWOOD_GET_IP_ADDRESS:
	case HARDWOOD_GET_STATUS:
	case HARDWOOD_GET_BYTES_USED:
	case HARDWOOD_RELEASE_BUFFER:
	case HARDWOOD_OVERFLOW_COUNT:
		hw_run_cmd(trace_cmd);
		hw_get_data(&op.data);
		if (copy_to_user((void __user *)arg, &op, sizeof(op)))
			return -EFAULT;
		break;

	/* CMD without input/output argument */
	case HARDWOOD_ENABLE_TRACE_DUMP:
	case HARDWOOD_DISABLE_TRACE_DUMP:
	case HARDWOOD_MARK_EMPTY:
		hw_run_cmd(trace_cmd);
		break;

	case HARDWOOD_SET_TRACER_MASK:
	case HARDWOOD_CLR_TRACER_MASK:
		/* lower 64bits */
		hw_set_data(op.data);
		hw_run_cmd(HW_CMD(0, 0, cmd));
		/* upper 64bits */
		hw_set_data(op.data2);
		hw_run_cmd(HW_CMD(0, 1, cmd));

	/* SW-only CMD */
	case HARDWOOD_GET_PHYS_ADDR:
		op.data = hardwood_devs[op.core_id].bufs[op.buffer_id].pa;
		if (copy_to_user((void __user *)arg, &op, sizeof(op)))
			return -EFAULT;
		break;

	case HARDWOOD_WAKEUP_READERS:
		DBG_PRINT("core%d is FORCED to wake up\n", op.core_id);
		hardwood_handler(TRACER_IRQS[op.core_id],
				&hardwood_devs[op.core_id]);
		break;

	default:
		return -ENOTTY;
	}
	return 0;
}

static bool is_buffer_ready(int cpu, int buf)
{
	u64 status;
	u64 bytes_used;

	hw_run_cmd(HW_CMD(cpu, buf, HARDWOOD_GET_BYTES_USED));
	hw_get_data(&bytes_used);
	bytes_used &= 0xffffffff;

	hw_run_cmd(HW_CMD(cpu, buf, HARDWOOD_GET_STATUS));
	hw_get_data(&status);
	status &= 0xff;

	if ((bytes_used > 0) && (status == 1))
		DBG_PRINT("buffer %d:%d is READY.", cpu, buf);
	else
		DBG_PRINT("buffer %d:%d is BUSY (used=%llu, status=%llu).",
			cpu, buf, bytes_used, status);

	return (bytes_used > 0) && (status == 1);
}

ssize_t hardwood_read(struct file *file, char __user *p, size_t s, loff_t *r)
{
	int i;
	int buf_id = -EAGAIN;
	struct hardwood_device *dev;

	dev = (struct hardwood_device *)file->private_data;

	CPU_GUARD(dev->cpu, 0);

	if (s != sizeof(u32)) {
		DBG_PRINT("must use u32 to read.\n");
		return 0;
	}

	spin_lock(&dev->buf_status_lock);
	if (dev->buf_status) {
		buf_id = find_first_bit(&dev->buf_status, sizeof(ulong));
		clear_bit(buf_id, &dev->buf_status);
	}
	spin_unlock(&dev->buf_status_lock);

	if (buf_id < 0) {
		/* No buffer available for readout */
		if (wait_event_interruptible(dev->wait_q, dev->signaled))
			return -ERESTARTSYS;

		/* Restore signal */
		dev->signaled = 0;

		DBG_PRINT("core%d is up\n", dev->cpu);

		spin_lock(&dev->buf_status_lock);

		/* Recheck */
		if (dev->buf_status) {
			buf_id = find_first_bit(&dev->buf_status,
					sizeof(ulong));
			clear_bit(buf_id, &dev->buf_status);
		} else {
			/* Poll each buffer */
			for (i = 0; i < N_BUFFER; ++i) {
				if (is_buffer_ready(dev->cpu, i)) {
					if (buf_id < 0) {
						buf_id = i;
						/* consider buf_id used */
						continue;
					}
					set_bit(i, &dev->buf_status);
				}
			}
		}

		DBG_PRINT("buf_status = %lx\n", dev->buf_status);

		spin_unlock(&dev->buf_status_lock);
	}

	if (buf_id >= 0) {
		/* Invalidate the cache */
		FLUSH_DCACHE_AREA(dev->bufs[buf_id].va, BUFFER_SIZE);

		if (copy_to_user((void __user *)p, &buf_id, sizeof(u32)))
			return -ENOMEM;

		return s;
	}

	return -EBUSY;
}

const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.open		= hardwood_open,
	.read		= hardwood_read,
	.unlocked_ioctl	= hardwood_ioctl,
#if CONFIG_COMPAT
	.compat_ioctl	= hardwood_ioctl,
#endif
};

static __init void init_one_buffer(int cpu, int buf_id)
{
	int i;
	u64 trace_cmd;
	struct hardwood_buf *buf;
	u32 *ptr;

	buf = &hardwood_devs[cpu].bufs[buf_id];
	buf->va = NULL;
	buf->pa = 0;

	buf->va = dma_alloc_coherent(NULL, BUFFER_SIZE, &buf->pa, GFP_KERNEL);
	BUG_ON(!buf->va || !buf->pa);

	/* Set buffer physical address */
	trace_cmd = HW_CMD(cpu, buf_id, HARDWOOD_SET_PHYS_ADDR);
	hw_set_data(buf->pa);
	hw_run_cmd(trace_cmd);

	/* Set buffer physical size */
	trace_cmd = HW_CMD(cpu, buf_id, HARDWOOD_SET_BUFFER_SIZE);
	hw_set_data(BUFFER_SIZE);
	hw_run_cmd(trace_cmd);

	ptr = (u32 *)buf->va;
	for (i = 0; i < BUFFER_SIZE; i += sizeof(u32))
		/* Fill in some poison data */
		*ptr++ = 0xdeadbeef;
}

static void init_one_cpu(int cpu)
{
	struct irqaction *irq;
	struct hardwood_device *hdev;

	hdev = &hardwood_devs[cpu];
	memset(hdev, 0, sizeof(*hdev));

	sprintf(hdev->name, "hardwood-%d", cpu);
	hdev->cpu = cpu;

	init_waitqueue_head(&hdev->wait_q);

	hdev->dev.minor = MISC_DYNAMIC_MINOR;
	hdev->dev.name = hdev->name;
	hdev->dev.fops = &fops;
	misc_register(&hdev->dev);
	minor_map[cpu] = hdev->dev.minor;
	DBG_PRINT("minor for cpu[%d] is %d", cpu, minor_map[cpu]);

	irq = &hdev->irq;
	irq->name = hdev->name;
	irq->flags = 0;
	irq->handler = hardwood_handler;
	irq->dev_id = (void *)hdev;
	irq->irq = TRACER_IRQS[cpu];
	BUG_ON(setup_irq(TRACER_IRQS[cpu], irq));

	hdev->buf_status = 0;
	hdev->signaled = 0;
	spin_lock_init(&hdev->buf_status_lock);
}

static void hardwood_late_init(void)
{
	int cpu;
	int i;
	spin_lock(&hardwood_init_lock);
	if (!hardwood_init_done) {
		for (cpu = 0; cpu < N_CPU; ++cpu)
			for (i = 0; i < N_BUFFER; i++)
				init_one_buffer(cpu, i);

		hardwood_init_done = 1;
	}
	spin_unlock(&hardwood_init_lock);
}

static __init int hardwood_init(void)
{
	int cpu;

	hardwood_supported = denver_backdoor_enabled();

	pr_info("Denver: hardwood is %ssupported.\n",
		hardwood_supported ? "" : "NOT ");

	if (hardwood_supported)
		for (cpu = 0; cpu < N_CPU; ++cpu)
			init_one_cpu(cpu);

	return 0;
}

late_initcall(hardwood_init);
