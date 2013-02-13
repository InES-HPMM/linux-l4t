/*
 * Copyright (C) 2012-2013 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <linux/skbuff.h>

#include "nvshm_types.h"
#include "nvshm_if.h"
#include "nvshm_priv.h"
#include "nvshm_iobuf.h"

/* NVSHM interface */

#define MAX_OUTPUT_SIZE 1500

/*
 * This structure hold per tty line information like
 * nvshm_iobuf queues and back reference to nvshm channel/driver
 */

struct nvshm_tty_line {
	int use;
	int nvshm_chan; /* nvshm channel */
	int throttled;
	/* iobuf queues for nvshm flow control support */
	struct nvshm_iobuf *io_queue_head;
	struct nvshm_iobuf *io_queue_tail;
	struct tty_struct *tty;
	struct nvshm_channel *pchan;
	int errno;
	spinlock_t lock;
};

struct nvshm_tty_device {
	spinlock_t lock;
	struct tty_driver *tty_driver;
	struct nvshm_handle *handle;
	int nlines;
	struct workqueue_struct *tty_wq;
	struct work_struct tty_worker;
	struct nvshm_tty_line line[NVSHM_MAX_CHANNELS];
};

static struct nvshm_tty_device tty_dev;

static void nvshm_tty_rx_rewrite_line(int l)
{
	struct nvshm_iobuf *list;
	struct tty_struct *tty = NULL;
	int len, nbuff = 0;

	if (!nvshm_interface_up())
		return;

	tty = tty_dev.line[l].tty;
	if (!tty)
		return;
	spin_lock(&tty_dev.line[l].lock);

	while (tty_dev.line[l].io_queue_head) {
		list = tty_dev.line[l].io_queue_head;
		spin_unlock(&tty_dev.line[l].lock);
		len = tty_insert_flip_string(tty,
					     NVSHM_B2A(tty_dev.handle,
						       list->npduData)
					     + list->dataOffset,
					     list->length);
		tty_flip_buffer_push(tty);
		spin_lock(&tty_dev.line[l].lock);
		if (len < list->length) {
			list->dataOffset += len;
			list->length -= len;
			spin_unlock(&tty_dev.line[l].lock);
			return;
		}
		if (list->sg_next) {
			/* Propagate ->next to the sg_next fragment
			   do not forget to move tail also */
			if (tty_dev.line[l].io_queue_head !=
			    tty_dev.line[l].io_queue_tail) {
				tty_dev.line[l].io_queue_head =
					NVSHM_B2A(tty_dev.handle,
						  list->sg_next);
				tty_dev.line[l].io_queue_head->next =
					list->next;
			} else {
				tty_dev.line[l].io_queue_head =
					NVSHM_B2A(tty_dev.handle,
						  list->sg_next);
				tty_dev.line[l].io_queue_tail =
					tty_dev.line[l].io_queue_head;
				if (list->next != NULL)
					pr_debug("%s:tail->next!=NULL\n",
						 __func__);
			}
		} else {
			if (list->next) {
				if (tty_dev.line[l].io_queue_head !=
				    tty_dev.line[l].io_queue_tail) {
					tty_dev.line[l].io_queue_head =
						NVSHM_B2A(tty_dev.handle,
							  list->next);
				} else {
					tty_dev.line[l].io_queue_head =
						NVSHM_B2A(tty_dev.handle,
							  list->next);
					tty_dev.line[l].io_queue_tail =
						tty_dev.line[l].io_queue_head;
				}
			} else {
				tty_dev.line[l].io_queue_tail = NULL;
				tty_dev.line[l].io_queue_head = NULL;
			}
		}
		nbuff++;
		nvshm_iobuf_free((struct nvshm_iobuf *)list);
	}
	if (!tty_dev.line[l].io_queue_head)
		tty_dev.line[l].throttled = 0;
	spin_unlock(&tty_dev.line[l].lock);
}

/* Called in a workqueue when unthrottle is called */
static void nvshm_tty_rx_rewrite(struct work_struct *work)
{
	int idx;

	for (idx = 0; idx < tty_dev.nlines; idx++)
		nvshm_tty_rx_rewrite_line(idx);
}

/*
 * nvshm_tty_rx_event()
 * NVSHM has received data insert them in tty flip buffer.
 * If no data is available, turn flow control on
 */
void nvshm_tty_rx_event(struct nvshm_channel *chan,
			struct nvshm_iobuf *iob)
{
	struct tty_struct *tty = (struct tty_struct *)chan->data;
	struct nvshm_iobuf *_phy_iob, *tmp;
	int len, idx;

	if (!nvshm_interface_up()) {
		return;
	}

	idx = tty->index;
	spin_lock(&tty_dev.line[idx].lock);

	if (!tty_dev.line[idx].throttled) {
		_phy_iob = iob;
		while (_phy_iob) {
			spin_unlock(&tty_dev.line[idx].lock);
			len = tty_insert_flip_string(tty,
						     NVSHM_B2A(tty_dev.handle,
							       iob->npduData)
						     + iob->dataOffset,
						     iob->length);
			tty_flip_buffer_push(tty);
			spin_lock(&tty_dev.line[idx].lock);
			if (len < iob->length) {
				tty_dev.line[idx].throttled = 1;
				iob->dataOffset += len;
				iob->length -= len;
				goto queue;
			}

			tmp = iob;
			/* Go next element: if ->sg_next follow it
			   and propagate ->next otherwise go next */
			if (iob->sg_next) {
				struct nvshm_iobuf *leaf;
				_phy_iob = iob->sg_next;
				if (_phy_iob) {
					leaf = NVSHM_B2A(tty_dev.handle,
							 _phy_iob);
					leaf->next = iob->next;
				}
			} else {
				_phy_iob = iob->next;
			}
			iob = NVSHM_B2A(tty_dev.handle, _phy_iob);
			nvshm_iobuf_free(tmp);
		}
		spin_unlock(&tty_dev.line[idx].lock);
		return;
	}
queue:
	/* Queue into FIFO */
	if (tty_dev.line[idx].io_queue_tail) {
		tty_dev.line[idx].io_queue_tail->next =
			NVSHM_A2B(tty_dev.handle, iob);
	} else {
		if (tty_dev.line[idx].io_queue_head) {
			tty_dev.line[idx].io_queue_head->next =
				NVSHM_A2B(tty_dev.handle, iob);
		} else {
			tty_dev.line[idx].io_queue_head = iob;
		}
	}
	tty_dev.line[idx].io_queue_tail = iob;
	spin_unlock(&tty_dev.line[idx].lock);
	queue_work(tty_dev.tty_wq, &tty_dev.tty_worker);
	return;
}

void nvshm_tty_error_event(struct nvshm_channel *chan,
			   enum nvshm_error_id error)
{
	struct tty_struct *tty = (struct tty_struct *)chan->data;
	pr_debug("%s\n", __func__);
	tty_dev.line[tty->index].errno = error;
	tty_hangup(tty);
}

void nvshm_tty_start_tx(struct nvshm_channel *chan)
{
	struct tty_struct *tty = (struct tty_struct *)chan->data;

	pr_debug("%s\n", __func__);
	tty_unthrottle(tty);
}

static struct nvshm_if_operations nvshm_tty_ops = {
	.rx_event = nvshm_tty_rx_event,
	.error_event = nvshm_tty_error_event,
	.start_tx = nvshm_tty_start_tx
};

/* TTY interface */

static int nvshm_tty_open(struct tty_struct *tty, struct file *f)
{
	int ret = 0;
	int idx = tty->index;

	pr_debug("%s\n", __func__);

	if (!nvshm_interface_up())
		return 0;

	/* Set TTY flags */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	tty->low_latency = 1;

	spin_lock(&tty_dev.line[idx].lock);
	if (!tty_dev.line[idx].use) {
		tty_dev.line[idx].throttled = 0;
		tty_dev.line[idx].pchan =
			nvshm_open_channel(tty_dev.line[idx].nvshm_chan,
					   &nvshm_tty_ops,
					   tty);
		if (tty_dev.line[idx].pchan)
			tty_dev.line[idx].tty = tty;
	}
	if (!ret)
		tty_dev.line[idx].use++;
	spin_unlock(&tty_dev.line[idx].lock);
	return ret;
}

static void nvshm_tty_close(struct tty_struct *tty, struct file *f)
{
	int use = 0;
	int idx = tty->index;

	if (!nvshm_interface_up())
		return;

	spin_lock(&tty_dev.line[idx].lock);

	if (tty_dev.line[idx].use > 0)
		use = --tty_dev.line[idx].use;

	pr_debug("%s %d\n", __func__, use);
	if (!use) {
		flush_workqueue(tty_dev.tty_wq);
		/* Cleanup if data are still present in io queue */
		if (tty_dev.line[idx].io_queue_head) {
			pr_debug("%s: still some data in queue!\n",
				 __func__);
				nvshm_iobuf_free_cluster(
					tty_dev.line[idx].io_queue_head);
				tty_dev.line[idx].io_queue_head =
					tty_dev.line[idx].io_queue_tail = NULL;
				tty_dev.line[idx].throttled = 0;
		}
		nvshm_close_channel(tty_dev.line[idx].pchan);
		tty_dev.line[idx].tty = NULL;
	}
	spin_unlock(&tty_dev.line[idx].lock);
}

static int nvshm_tty_write_room(struct tty_struct *tty)
{
	if (!nvshm_interface_up())
		return 0;

	return MAX_OUTPUT_SIZE;
}

static int nvshm_tty_write(struct tty_struct *tty, const unsigned char *buf,
			   int len)
{
	struct nvshm_iobuf *iob, *leaf = NULL, *list = NULL;
	int to_send = 0, remain, idx = tty->index, ret;

	if (!nvshm_interface_up()) {
		return 0;
	}

	remain = len;
	while (remain) {
		to_send = remain < MAX_OUTPUT_SIZE ? remain : MAX_OUTPUT_SIZE;
		iob = nvshm_iobuf_alloc(tty_dev.line[idx].pchan, to_send);
		if (!iob) {
			if (tty_dev.line[idx].errno) {
				pr_err("%s iobuf alloc failed\n", __func__);
				if (list)
					nvshm_iobuf_free_cluster(list);
				return -ENOMEM;
			} else {
				pr_err("%s: Xoff condition\n", __func__);
				return 0;
			}
		}

		iob->length = to_send;
		iob->chan = tty_dev.line[idx].pchan->index;
		remain -= to_send;
		memcpy(NVSHM_B2A(tty_dev.handle,
				 iob->npduData +
				 iob->dataOffset),
		       buf,
		       to_send);
		buf += to_send;

		if (!list) {
			leaf = list = iob;
		} else {
			leaf->sg_next = NVSHM_A2B(tty_dev.handle, iob);
			leaf = iob;
		}
	}
	ret = nvshm_write(tty_dev.line[idx].pchan, list);

	if (ret == 1) {
		pr_warn("%s rate limit hit on TTY %d\n", __func__, idx);
		tty_throttle(tty);
	}

	return len;
}

static void nvshm_tty_unthrottle(struct tty_struct *tty)
{
	int idx = tty->index;

	if (!nvshm_interface_up())
		return;

	spin_lock(&tty_dev.line[idx].lock);
	if (tty_dev.line[idx].throttled) {
		spin_unlock(&tty_dev.line[idx].lock);
		queue_work(tty_dev.tty_wq, &tty_dev.tty_worker);
		return;
	}
	spin_unlock(&tty_dev.line[idx].lock);
}

static const struct tty_operations nvshm_tty_ttyops = {
	.open = nvshm_tty_open,
	.close = nvshm_tty_close,
	.write = nvshm_tty_write,
	.write_room = nvshm_tty_write_room,
	.unthrottle = nvshm_tty_unthrottle,
};

int nvshm_tty_init(struct nvshm_handle *handle)
{
	int ret, chan;

	pr_debug("%s\n", __func__);

	memset(&tty_dev, 0, sizeof(tty_dev));
	tty_dev.handle = handle;

	for (chan = 0; chan < NVSHM_MAX_CHANNELS; chan++) {
		if ((handle->chan[chan].map.type == NVSHM_CHAN_TTY)
			|| (handle->chan[chan].map.type == NVSHM_CHAN_LOG)) {
			tty_dev.line[tty_dev.nlines].nvshm_chan = chan;
			spin_lock_init(&tty_dev.line[tty_dev.nlines].lock);
			tty_dev.nlines++;
		}
	}
	tty_dev.tty_wq = create_singlethread_workqueue("NVSHM_tty");
	INIT_WORK(&tty_dev.tty_worker, nvshm_tty_rx_rewrite);

	tty_dev.tty_driver = alloc_tty_driver(tty_dev.nlines);

	if (tty_dev.tty_driver == NULL)
		return -ENOMEM;

	tty_dev.tty_driver->owner = THIS_MODULE;
	tty_dev.tty_driver->driver_name = "nvshm_tty";
	tty_dev.tty_driver->name = "ttySHM";
	tty_dev.tty_driver->major = 0;
	tty_dev.tty_driver->minor_start = 0;
	tty_dev.tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	tty_dev.tty_driver->subtype = SERIAL_TYPE_NORMAL;
	tty_dev.tty_driver->init_termios = tty_std_termios;
	tty_dev.tty_driver->init_termios.c_iflag = 0;
	tty_dev.tty_driver->init_termios.c_oflag = 0;
	tty_dev.tty_driver->init_termios.c_cflag =
		B115200 | CS8 | CREAD | CLOCAL;
	tty_dev.tty_driver->init_termios.c_lflag = 0;
	tty_dev.tty_driver->flags =
		TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW |
		TTY_DRIVER_DYNAMIC_DEV;

	tty_set_operations(tty_dev.tty_driver, &nvshm_tty_ttyops);

	ret = tty_register_driver(tty_dev.tty_driver);

	if (ret)
		return 0;

	for (chan = 0; chan < tty_dev.nlines; chan++) {
		spin_lock_init(&tty_dev.line[chan].lock);
		tty_register_device(tty_dev.tty_driver, chan, 0);
	}

	return 0;
}

void nvshm_tty_cleanup(void)
{
	int chan;

	pr_debug("%s\n", __func__);
	if (tty_dev.tty_wq) {
		/* Wait workqueue end */
		destroy_workqueue(tty_dev.tty_wq);
		tty_dev.tty_wq = NULL;
	}
	for (chan = 0; chan < tty_dev.nlines; chan++) {
		/* No need to cleanup data as iobufs are invalid now */
		/* Next nvshm_tty_init will do it */
		spin_lock(&tty_dev.line[chan].lock);
		if (tty_dev.line[chan].use) {
			tty_dev.line[chan].use = 0;
			spin_unlock(&tty_dev.line[chan].lock);
			nvshm_close_channel(tty_dev.line[chan].pchan);
			pr_debug("%s hangup tty device %d\n", __func__,
				 tty_dev.line[chan].tty->index);
			tty_hangup(tty_dev.line[chan].tty);
		} else {
			spin_unlock(&tty_dev.line[chan].lock);
		}
		pr_debug("%s unregister tty device %d\n", __func__, chan);
		tty_unregister_device(tty_dev.tty_driver, chan);
	}
	pr_debug("%s unregister tty driver\n", __func__);
	tty_unregister_driver(tty_dev.tty_driver);
	memset(&tty_dev, 0, sizeof(tty_dev));
}
