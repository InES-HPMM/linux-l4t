/*
 * tegra_hv_comm.c: TTY over Tegra HV
 *
 * Copyright (c) 2014-2015 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-ivc.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>

/*
 * frame format is
 * 0000: <size>
 * 0004: <flags>
 * 0008: data
 */

#define TEGRA_HV_COMM_MAJOR    204
#define TEGRA_HV_COMM_MINOR    213
#define TEGRA_HV_COMM_MAXPORTS 1

#define HDR_SIZE 8
#define DRV_NAME "tegra_hv_comm"

#define to_tegra_hv_comm(_x) \
	container_of(_x, struct tegra_hv_comm, port)

/*
 * The below value is the retry timeout to use when incoming data cannot
 * be accommodated because the kernel tty buffers are full. We may want
 * to expose this for user configuration at some point, but for now it's
 * not expected for users to change this value given the planned set of
 * use cases we are initially supporting.
 */

#define RX_BUF_WAIT_JIFFIES ((HZ)/16)

struct tegra_hv_comm {
	struct uart_port port;
	struct platform_device *pdev;
	struct tegra_hv_ivc_cookie *ivck;
	struct workqueue_struct *work_queue;
	struct delayed_work work_task;
	unsigned int tx_en;
	unsigned int rx_en;
};

static int __push_ivc_buffer(struct tegra_hv_comm *pp)
{
	struct uart_port *port = &pp->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct device *dev = &pp->pdev->dev;
	unsigned int count;
	unsigned int i;
	void *buf;

	count = uart_circ_chars_pending(xmit);

	if (count > (pp->ivck->frame_size - HDR_SIZE))
		count = pp->ivck->frame_size - HDR_SIZE;

	if (count == 0)
		return -EAGAIN;

	buf = tegra_hv_ivc_write_get_next_frame(pp->ivck);
	if (IS_ERR(buf))
		return -ENOBUFS;

	((u32 *)buf)[0] = count;
	((u32 *)buf)[1] = 0;

	/*
	 * Dequeue data from our circular transmit buffer. We do this
	 * operation a byte at a time because the buffers are not
	 * necessarily contiguous (i.e., the buffer will wrap around.)
	 */

	for (i = 0; i < count; i++) {
		((char *)buf + HDR_SIZE)[i] = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
	}

	if (tegra_hv_ivc_write_advance(pp->ivck) != 0)
		dev_info(dev, "failed to advance write pos\n");

	port->icount.tx += count;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return count;
}

static int __pull_ivc_buffer(struct tegra_hv_comm *pp)
{
	struct uart_port *port = &pp->port;
	struct tty_port *tty = &port->state->port;
	struct device *dev = &pp->pdev->dev;
	unsigned int count;
	void *buf;
	int copied;
	int ret;

	buf = tegra_hv_ivc_read_get_next_frame(pp->ivck);
	if (IS_ERR(buf))
		return -ENOBUFS;

	count = *((u32 *)buf);

	if (count > pp->ivck->frame_size - HDR_SIZE) {
		dev_info(dev, "bad data size\n");
		if (tegra_hv_ivc_read_advance(pp->ivck) != 0)
			dev_info(dev, "failed to advance read pos\n");
		return count;
	}

	/*
	 * Check for available space in the kernel tty buffer. If there
	 * is not enough available space, check again "later". Ideally,
	 * the tty layer would notify us when a buffer becomes free, but
	 * since that is not supported now we will just have to rely on
	 * the passing of time as a hint to retry.
	 */

	ret = tty_buffer_request_room(tty, count);
	if (ret != count) {
		(void)queue_delayed_work(pp->work_queue,
			&pp->work_task, RX_BUF_WAIT_JIFFIES);
		return -EAGAIN;
	}

	copied = tty_insert_flip_string(tty, (char *)buf + HDR_SIZE, count);
	if (copied != count)
		dev_info(dev, "failed to insert %u\n", count);

	/*
	 * Insertion failures are not expected because we have explicitly
	 * checked for available room, but should they occur anyway,
	 * we act conservatively and throw away the entire buffer.
	 */

	if (tegra_hv_ivc_read_advance(pp->ivck) != 0)
		dev_info(dev, "failed to advance read pos\n");

	if (copied > 0) {
		port->icount.rx += copied;
		tty_flip_buffer_push(tty);
	}

	return count;
}

static void __work_handler(struct work_struct *ws)
{
	struct delayed_work *dw =
		container_of(ws, struct delayed_work, work);
	struct tegra_hv_comm *pp =
		container_of(dw, struct tegra_hv_comm, work_task);
	struct uart_port *port = &pp->port;
	struct device *dev = &pp->pdev->dev;
	unsigned long flags;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	/*
	 * For consistency, we simply assume all IVC calls require
	 * synchronization if there is chance of concurrent execution.
	 */

	spin_lock_irqsave(&port->lock, flags);
	ret = tegra_hv_ivc_channel_notified(pp->ivck);
	spin_unlock_irqrestore(&port->lock, flags);

	if (ret != 0)
		return;

	/*
	 * It is expected for serial drivers to interact with port->lock
	 * held and from atomic/uninterruptible context.
	 *
	 * The below functions break up work so that we operate on at most
	 * one IVC buffer at a time, so we can still have relatively short
	 * critical sections with relatively frequent preemption boundaries.
	 */

	do {
		spin_lock_irqsave(&port->lock, flags);
		ret = (pp->tx_en) ? __push_ivc_buffer(pp) : -EIO;
		spin_unlock_irqrestore(&port->lock, flags);
	} while (ret >= 0);

	do {
		spin_lock_irqsave(&port->lock, flags);
		ret = (pp->rx_en) ? __pull_ivc_buffer(pp) : -EIO;
		spin_unlock_irqrestore(&port->lock, flags);
	} while (ret >= 0);
}

static irqreturn_t __irq_handler(int irq, void *data)
{
	struct tegra_hv_comm *pp = (struct tegra_hv_comm *)data;
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	(void)queue_delayed_work(pp->work_queue, &pp->work_task, 0);

	return IRQ_HANDLED;
}

static unsigned int tegra_hv_comm_tx_empty(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;
	unsigned long flags;
	unsigned int ret;

	dev_dbg(dev, "%s\n", __func__);

	/*
	 * For consistency, we simply assume all IVC calls require
	 * synchronization if there is chance of concurrent execution.
	 */

	spin_lock_irqsave(&port->lock, flags);
	ret = tegra_hv_ivc_tx_empty(pp->ivck) ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

static unsigned int tegra_hv_comm_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void tegra_hv_comm_set_mctrl(struct uart_port *port, unsigned int sigs)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);
}

static void tegra_hv_comm_start_tx(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	/*
	 * This function is called with port->lock held during an
	 * uninterruptible context, so no additional synchronization
	 * is needed.
	 */

	pp->tx_en = 1;

	(void)queue_delayed_work(pp->work_queue, &pp->work_task, 0);
}

static void tegra_hv_comm_stop_tx(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	/*
	 * This function is called with port->lock held during an
	 * uninterruptible context, so no additional synchronization
	 * is needed.
	 */

	pp->tx_en = 0;
}

static void tegra_hv_comm_stop_rx(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	/*
	 * This function is called with port->lock held during an
	 * uninterruptible context, so no additional synchronization
	 * is needed.
	 */

	pp->rx_en = 0;
}

static void tegra_hv_comm_break_ctl(struct uart_port *port, int break_state)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);
}

static void tegra_hv_comm_enable_ms(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);
}

static void tegra_hv_comm_set_termios(struct uart_port *port,
	struct ktermios *termios, struct ktermios *old)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	/* Just copy the old termios settings back */
	if (old)
		tty_termios_copy_hw(termios, old);
}

static void tegra_hv_comm_config_port(struct uart_port *port, int flags)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	port->type = PORT_TEGRA_HV;
}

static int tegra_hv_comm_startup(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	pp->rx_en = 1;
	pp->tx_en = 0;

	ret = devm_request_irq(dev, pp->ivck->irq, __irq_handler, 0,
		dev_name(dev), pp);
	if (ret) {
		dev_err(dev, "failed to request irq=%d\n", pp->ivck->irq);
		return ret;
	}

	(void)queue_delayed_work(pp->work_queue, &pp->work_task, 0);
	return 0;
}

static void tegra_hv_comm_shutdown(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	devm_free_irq(&pp->pdev->dev, pp->ivck->irq, port);
	synchronize_irq(pp->ivck->irq);
	(void)cancel_delayed_work_sync(&pp->work_task);

	pp->rx_en = 0;
	pp->tx_en = 0;
}

static const char *tegra_hv_comm_type(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	return (port->type == PORT_TEGRA_HV) ? "Tegra HV" : NULL;
}

static int tegra_hv_comm_request_port(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	return 0;
}

static void tegra_hv_comm_release_port(struct uart_port *port)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);
}

static int tegra_hv_comm_verify_port(struct uart_port *port,
	struct serial_struct *ser)
{
	struct tegra_hv_comm *pp = to_tegra_hv_comm(port);
	struct device *dev = &pp->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_TEGRA_HV)
		return -EINVAL;

	return 0;
}

static struct uart_ops tegra_hv_comm_ops = {
	.tx_empty	= tegra_hv_comm_tx_empty,
	.get_mctrl	= tegra_hv_comm_get_mctrl,
	.set_mctrl	= tegra_hv_comm_set_mctrl,
	.start_tx	= tegra_hv_comm_start_tx,
	.stop_tx	= tegra_hv_comm_stop_tx,
	.stop_rx	= tegra_hv_comm_stop_rx,
	.enable_ms	= tegra_hv_comm_enable_ms,
	.break_ctl	= tegra_hv_comm_break_ctl,
	.startup	= tegra_hv_comm_startup,
	.shutdown	= tegra_hv_comm_shutdown,
	.set_termios	= tegra_hv_comm_set_termios,
	.type		= tegra_hv_comm_type,
	.request_port	= tegra_hv_comm_request_port,
	.release_port	= tegra_hv_comm_release_port,
	.config_port	= tegra_hv_comm_config_port,
	.verify_port	= tegra_hv_comm_verify_port,
};

static struct uart_driver tegra_hv_comm_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "tegra_hv_comm",
	.dev_name	= "ttyTGRHVC",
	.major		= TEGRA_HV_COMM_MAJOR,
	.minor		= TEGRA_HV_COMM_MINOR,
	.nr		= TEGRA_HV_COMM_MAXPORTS,
	.cons		= NULL,	/* no console supported yet */
};

static int tegra_hv_comm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn, *hv_dn;
	struct tegra_hv_comm *pp;
	struct uart_port *port;
	int idx = pdev->id;
	int ret;
	u32 val;

	if (!is_tegra_hypervisor_mode()) {
		dev_info(dev, "hypervisor is not present\n");
		return -ENODEV;
	}

	/*
	 * A id value of -1 emphasizes that the platform must have one port
	 * (i.e., no .N suffix).
	 */

	if (idx == -1)
		idx = 0;

	if (idx >= TEGRA_HV_COMM_MAXPORTS)
		return -EINVAL;

	dn = dev->of_node;
	if (dn == NULL) {
		dev_err(dev, "failed to find OF data\n");
		return -EINVAL;
	}

	hv_dn = of_parse_phandle(dn, "ivc", 0);
	if (hv_dn == NULL) {
		dev_err(dev, "failed to parse phandle\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_index(dn, "ivc", 1, &val);
	if (ret != 0) {
		dev_err(dev, "failed to read property id\n");
		of_node_put(hv_dn);
		return ret;
	}

	pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	if (pp == NULL) {
		dev_err(dev, "failed to allocate memory\n");
		of_node_put(hv_dn);
		return -ENOMEM;
	}

	pp->pdev = pdev;
	pp->ivck = tegra_hv_ivc_reserve(hv_dn, val, NULL);
	of_node_put(hv_dn);

	if (IS_ERR_OR_NULL(pp->ivck)) {
		devm_kfree(dev, pp);
		dev_err(dev, "failed to reserve ivc %d\n", val);
		return PTR_ERR(pp->ivck);
	}

	if (pp->ivck->frame_size <= HDR_SIZE) {
		dev_err(dev, "frame size is too small\n");
		ret = -EINVAL;
		goto out_failed;
	}

	/*
	 * The maximum sized buffer accepted by tty_insert_flip_string()
	 * without fragmentation is TTY_BUFFER_PAGE, so for simplicity we
	 * don't accept frame sizes that are larger.
	 */

	if ((pp->ivck->frame_size - HDR_SIZE) > TTY_BUFFER_PAGE) {
		dev_err(dev, "frame size is too big\n");
		ret = -EINVAL;
		goto out_failed;
	}

	INIT_DELAYED_WORK(&pp->work_task, __work_handler);

	pp->work_queue = create_singlethread_workqueue("tegra-hv-comm");
	if (pp->work_queue == NULL) {
		dev_err(dev, "failed to create workqueue\n");
		ret = -EINVAL;
		goto out_failed;
	}

	port = &pp->port;
	port->line = idx;
	port->type = PORT_TEGRA_HV;
	port->iotype = SERIAL_IO_MEM;
	port->ops = &tegra_hv_comm_ops;
	port->flags = UPF_BOOT_AUTOCONF;

	ret = uart_add_one_port(&tegra_hv_comm_driver, port);
	if (ret != 0) {
		destroy_workqueue(pp->work_queue);
		dev_err(dev, "failed to add uart port\n");
		goto out_failed;
	}

	/*
	 * Start the channel reset process asynchronously. Until the reset
	 * process completes, any attempt to use the ivc channel will return
	 * an error (e.g., all transmits will fail.)
	 */

	tegra_hv_ivc_channel_reset(pp->ivck);

	platform_set_drvdata(pdev, pp);
	dev_info(dev, "reserved ivc=%d framesize=%d\n",
		val, pp->ivck->frame_size);

	return 0;

out_failed:
	tegra_hv_ivc_unreserve(pp->ivck);
	devm_kfree(dev, pp);
	return ret;
}

static int tegra_hv_comm_remove(struct platform_device *pdev)
{
	struct tegra_hv_comm *pp = platform_get_drvdata(pdev);
	struct device *dev = &pp->pdev->dev;

	destroy_workqueue(pp->work_queue);
	uart_remove_one_port(&tegra_hv_comm_driver, &pp->port);
	tegra_hv_ivc_unreserve(pp->ivck);
	devm_kfree(dev, pp);
	platform_set_drvdata(pdev, NULL);
	dev_info(dev, "removed\n");

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id tegra_hv_comm_match[] = {
	{ .compatible = "nvidia,tegra-hv-comm", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_hv_comm_match);
#endif /* CONFIG_OF */

static struct platform_driver tegra_hv_comm_platform_driver = {
	.probe	= tegra_hv_comm_probe,
	.remove	= tegra_hv_comm_remove,
	.driver	= {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(tegra_hv_comm_match),
	},
};

static int tegra_hv_comm_init(void)
{
	int rc;

	rc = uart_register_driver(&tegra_hv_comm_driver);
	if (rc)
		return rc;

	rc = platform_driver_register(&tegra_hv_comm_platform_driver);
	if (rc)
		uart_unregister_driver(&tegra_hv_comm_driver);

	return rc;
}

static void tegra_hv_comm_exit(void)
{
	platform_driver_unregister(&tegra_hv_comm_platform_driver);
	uart_unregister_driver(&tegra_hv_comm_driver);
}

module_init(tegra_hv_comm_init);
module_exit(tegra_hv_comm_exit);

MODULE_AUTHOR("Pantelis Antoniou <pantoniou@nvidia.com>");
MODULE_DESCRIPTION("TTY over Tegra Hypervisor IVC channel");
MODULE_LICENSE("GPL");
