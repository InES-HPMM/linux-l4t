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
#include <linux/kthread.h>
#include <linux/cpu.h>

#include <asm/cacheflush.h>

#include "common.h" /* tegra_cpu_is_secure */

#include "denver-knobs.h" /* backdoor detection */

#include "denver-hardwood.h"

/* #define MY_DEBUG 1 */

#ifdef MY_DEBUG
#define DBG_PRINT(format, arg...) \
	pr_info("hardwood: %s: " format "\n", __func__, ##arg)
#else
#define DBG_PRINT(format, arg...)
#endif

#define TRACER_OSDUMP_BUFFER_CMD		100
#define TRACER_OSDUMP_BUFFER_CMD_DATA	101
#define HW_CMD(c, b, m) ((c) | ((b) << 8) | ((m) << 16))

#define PHYS_NS_BIT (1ULL << 40)

struct hardwood_buf {
	void *va;
	dma_addr_t pa;
};

struct hardwood_device {
	/* device info */
	int cpu;
	char name[32];
	struct miscdevice dev;
	struct irqaction irq;

	/* sleeping support */
	int signaled;
	wait_queue_head_t wait_q;

	/* bitmask for buffer availability */
	ulong buf_status;
	spinlock_t buf_status_lock;

	/* bitmask for buffer occupation by userspace */
	ulong buf_occupied;

	/* Per CPU buffers */
	struct hardwood_buf bufs[N_BUFFER];
} hardwood_devs[N_CPU];

static int minor_map[N_CPU] = { -1 };

static int TRACER_IRQS[] = { 48, 54 };

static bool hardwood_supported;
static bool hardwood_init_done;
static DEFINE_MUTEX(hardwood_init_lock);

/* Thread for probing buffers when some CPUs are hot-unplugged */
static struct task_struct *agent_thread;
static bool agent_stopped;

static void hardwood_init_agent(void);

static void hardwood_late_init(void);

static bool check_buffers(struct hardwood_device *dev, bool lock);

static irqreturn_t hardwood_handler(int irq, void *dev_id)
{
	struct hardwood_device *dev = (struct hardwood_device *) dev_id;

	DBG_PRINT("IRQ%d received\n", irq);

	if (num_online_cpus() == N_CPU) {
		/* All CPUs are online, waking up target CPU */
		dev->signaled = 1;
		wake_up_interruptible(&dev->wait_q);
		DBG_PRINT("CPU%d is interrupted\n", dev->cpu);
	} else {
		if (agent_thread->state != TASK_RUNNING) {
			/* Some CPUs are offline, waking up agent */
			wake_up_process(agent_thread);
			DBG_PRINT("Agent is interrupted\n");
		}
	}

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

	/* Lazy init */
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
	struct hardwood_device *dev;

	if (copy_from_user(&op, (void __user *)arg, sizeof(op)))
		return -EFAULT;

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
		dev = &hardwood_devs[op.core_id];
		if (cmd == HARDWOOD_MARK_EMPTY)
			/* Removing its occupying bit */
			clear_bit(op.buffer_id, &dev->buf_occupied);
		break;

	case HARDWOOD_SET_TRACER_MASK:
	case HARDWOOD_CLR_TRACER_MASK:
		/* lower 64bits */
		hw_set_data(op.data);
		hw_run_cmd(HW_CMD(0, 0, cmd));
		/* upper 64bits */
		hw_set_data(op.data2);
		hw_run_cmd(HW_CMD(0, 1, cmd));
		break;

	/* SW-only CMD */
	case HARDWOOD_GET_PHYS_ADDR:
		op.data = hardwood_devs[op.core_id].bufs[op.buffer_id].pa;
		op.data &= ~PHYS_NS_BIT;
		if (copy_to_user((void __user *)arg, &op, sizeof(op)))
			return -EFAULT;
		break;

	case HARDWOOD_WAKEUP_READERS:
		DBG_PRINT("core%d is FORCED to wake up\n", op.core_id);
		dev = &hardwood_devs[op.core_id];
		check_buffers(dev, true);
		dev->signaled = 1;
		wake_up_interruptible(&dev->wait_q);
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

	DBG_PRINT("Probing buffer %d:%d\n", cpu, buf);

	hw_run_cmd(HW_CMD(cpu, buf, HARDWOOD_GET_BYTES_USED));
	hw_get_data(&bytes_used);
	bytes_used &= 0xffffffff;

	hw_run_cmd(HW_CMD(cpu, buf, HARDWOOD_GET_STATUS));
	hw_get_data(&status);
	status &= 0xff;

	if ((bytes_used > 0) && (status == 1))
		DBG_PRINT("buffer %d:%d is READY (used=%llu, status=%llu)..",
			cpu, buf, bytes_used, status);
	else
		DBG_PRINT("buffer %d:%d is BUSY (used=%llu, status=%llu).",
			cpu, buf, bytes_used, status);

	return (bytes_used > 0) && (status == 1);
}

static bool check_buffers(struct hardwood_device *dev, bool lock)
{
	int i;

	if (lock)
		spin_lock(&dev->buf_status_lock);

	/* Poll each buffer */
	for (i = 0; i < N_BUFFER; ++i)
		if (is_buffer_ready(dev->cpu, i)) {
			/* Present buffer only if not occupied */
			if (!test_bit(i, &dev->buf_occupied))
				set_bit(i, &dev->buf_status);
		}

	if (lock)
		spin_unlock(&dev->buf_status_lock);

	return dev->buf_status != 0;
}

static int agent_thread_fn(void *data)
{
	int cpu;
	struct hardwood_device *dev;
	while (!agent_stopped) {
		DBG_PRINT("agent: start waiting\n");
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		/* woken up by interrupt */
		DBG_PRINT("agent: woken up\n");
		set_current_state(TASK_RUNNING);

		for (cpu = 0; cpu < N_CPU; ++cpu) {
			DBG_PRINT("agent: checking CPU %d\n", cpu);
			dev = &hardwood_devs[cpu];
			if (check_buffers(dev, true)) {
				DBG_PRINT("Waking up reader %d\n", cpu);
				dev->signaled = 1;
				wake_up_interruptible(&dev->wait_q);
			}
		}

		DBG_PRINT("agent: done one pass\n");
	}
	return 0;
}

ssize_t hardwood_read(struct file *file, char __user *p, size_t s, loff_t *r)
{
	int buf_id = -EAGAIN;
	struct hardwood_device *dev;
	bool found;

	dev = (struct hardwood_device *)file->private_data;

	if (s != sizeof(u32)) {
		DBG_PRINT("must use u32 to read.\n");
		return 0;
	}

	/* Check if any buffers is available NOW */
	spin_lock(&dev->buf_status_lock);
	if (dev->buf_status) {
		buf_id = find_first_bit(&dev->buf_status, sizeof(ulong));
		clear_bit(buf_id, &dev->buf_status);
		set_bit(buf_id, &dev->buf_occupied);
	}
	spin_unlock(&dev->buf_status_lock);

	if (buf_id < 0) {
		/* No buffer available for readout */
		DBG_PRINT("[HW] CPU%d is waiting on %p\n",
			dev == &hardwood_devs[0] ? 0 : 1, &dev->wait_q);

		/* Sleep until signaled by IRQ handler */
		if (wait_event_interruptible(dev->wait_q, dev->signaled))
			return -ERESTARTSYS;

		/* Restore signal */
		dev->signaled = 0;

		DBG_PRINT("CPU%d is woken up\n", dev->cpu);

		spin_lock(&dev->buf_status_lock);

		/* Re-probe */
		found = true;
		if (!dev->buf_status)
			found = check_buffers(dev, false);

		/* Some buffers are available NOW, we grab one */
		if (found) {
			buf_id = find_first_bit(&dev->buf_status,
					sizeof(ulong));
			clear_bit(buf_id, &dev->buf_status);
			set_bit(buf_id, &dev->buf_occupied);
		}

		DBG_PRINT("buf_status = %lx, buf_id = %d\n",
			dev->buf_status, buf_id);

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

	buf->va = kmalloc(BUFFER_SIZE, GFP_KERNEL);
	buf->pa = virt_to_phys(buf->va);
	BUG_ON(!buf->va || !buf->pa);

	/* Set NS bit if kernel is non-secure */
	if (tegra_cpu_is_secure())
		buf->pa |= PHYS_NS_BIT;

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

static inline void hardwood_late_init(void)
{
	int i, j;

	if (hardwood_init_done)
		return;

	/* use mutex b/c below code might sleep */
	mutex_lock(&hardwood_init_lock);
	if (!hardwood_init_done) {
		hardwood_init_agent();

		for (i = 0; i < N_CPU; i++)
			for (j = 0; j < N_BUFFER; j++)
				init_one_buffer(i, j);

		hardwood_init_done = 1;
	}
	mutex_unlock(&hardwood_init_lock);
}

static int hardwood_cpu_notify(struct notifier_block *self,
					 unsigned long action, void *hcpu)
{
	long cpu = (long) hcpu;
	long new_cpu;

	switch (action) {
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		new_cpu = cpu;
		break;

	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		new_cpu = cpu == 0 ? 1 : 0;
		break;

	default:
		return NOTIFY_OK;
	}

	hw_run_cmd(HW_CMD(cpu, new_cpu, HARDWOOD_SET_IRQ_TARGET));

	return NOTIFY_OK;
}

static struct notifier_block hardwood_cpu_notifier = {
	.notifier_call = hardwood_cpu_notify,
};

static void hardwood_init_agent(void)
{
	register_hotcpu_notifier(&hardwood_cpu_notifier);
	agent_thread = kthread_create(agent_thread_fn, 0, "hardwood-agent");
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
	hdev->buf_occupied = 0;
	hdev->signaled = 0;
	spin_lock_init(&hdev->buf_status_lock);
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
