/*
 * f_digitizer: Tegra digitizer driver.
 *
 * Copyright (c) 2012-2015, NVIDIA CORPORATION.  All rights reserved.
 * It's based on f_hid.c from Fabien Chouteau <fabien.chouteau@barco.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hid.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/usb/g_hid.h>

#define     REPORTID_PEN        0x01
#define     REPORTID_TOUCH      0x02
#define     REPORTID_MAX_COUNT  0x06
#define     MAX_PRESSURE        0xFF
#define     MAX_FINGER          6

#pragma pack(1)
struct report_pen {
	unsigned char		report_id;
	unsigned char		tip;
	unsigned short		x;
	unsigned short		y;
	unsigned short		pressure;
};
struct report_touch {
	unsigned char		report_id;
	unsigned char		tip;
	unsigned char		contact_id;
	unsigned short		x;
	unsigned short		y;
	unsigned short		pressure;
	unsigned short		scan_time;
	unsigned char		contact_count;
};
struct event_ts {
	unsigned char		id;
	unsigned char		tip;
	unsigned short		x;
	unsigned short		y;
	unsigned short		pressure;
};
#pragma pack()

static struct hidg_func_descriptor digitizer_report_desc = {
	.subclass			= 0,
	.protocol			= 0,
	.report_length		= 256,
	.report_desc_length = 190,
	.report_desc		= {
	0x05, 0x0d,                  /* USAGE_PAGE (Digitizers)          */
	0x09, 0x02,                  /* USAGE (Pen)                      */
	0xa1, 0x01,                  /* COLLECTION (Application)         */
	0x85, REPORTID_PEN,          /*   REPORT_ID (Pen)                */
	0x09, 0x20,                  /*   USAGE (Stylus)                 */
	0xa1, 0x02,                  /*   COLLECTION (Logical)           */
	0x09, 0x42,                  /*     USAGE (Tip Switch)           */
	0x15, 0x00,                  /*     LOGICAL_MINIMUM (0)          */
	0x25, 0x01,                  /*     LOGICAL_MAXIMUM (1)          */
	0x75, 0x01,                  /*     REPORT_SIZE (1)              */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x95, 0x07,                  /*     REPORT_COUNT (7)             */
	0x81, 0x03,                  /*     INPUT (Cnst,Ary,Abs)         */
	0x05, 0x01,                  /*     USAGE_PAGE (Generic Desk..)  */
	0x26, 0x7F, 0x07,            /*     LOGICAL_MAXIMUM (1920-1)     */
	0x75, 0x10,                  /*     REPORT_SIZE (16)             */
	0x55, 0x0e,                  /*     UNIT_EXPONENT (-2)           */
	0x65, 0x11,                  /*     Unit (SI Lin: Length (cm))   */
	0x09, 0x30,                  /*     USAGE (X)                    */
	0x35, 0x00,                  /*     PHYSICAL_MINIMUM (0)         */
	0x46, 0x72, 0x0D,            /*     PHYSICAL_MAXIMUM (3442)      */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x46, 0x8F, 0x07,            /*     PHYSICAL_MAXIMUM (1935)      */
	0x26, 0xAF, 0x04,            /*     LOGICAL_MAXIM (1200-1)       */
	0x09, 0x31,                  /*     USAGE (Y)                    */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x05, 0x0d,                  /*     USAGE_PAGE (Digitizers)      */
	0x09, 0x30,                  /*     USAGE (Tip Pressure)         */
	0x26, MAX_PRESSURE, 0x00,    /*     LOGICAL_MAXIMUM (255)        */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0xc0,                        /*   END_COLLECTION                 */
	0xc0,                        /* END_COLLECTION                   */
	0x09, 0x04,                  /* USAGE (Touch Screen)             */
	0xa1, 0x01,                  /* COLLECTION (Application)         */
	0x85, REPORTID_TOUCH,        /*   REPORT_ID (Touch)              */
	0x09, 0x22,                  /*   USAGE (Finger)                 */
	0xa1, 0x02,                  /*   COLLECTION (Logical)           */
	0x09, 0x42,                  /*     USAGE (Tip Switch)           */
	0x15, 0x00,                  /*     LOGICAL_MINIMUM (0)          */
	0x25, 0x01,                  /*     LOGICAL_MAXIMUM (1)          */
	0x75, 0x01,                  /*     REPORT_SIZE (1)              */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x95, 0x07,                  /*     REPORT_COUNT (7)             */
	0x81, 0x03,                  /*     INPUT (Cnst,Ary,Abs)         */
	0x75, 0x08,                  /*     REPORT_SIZE (8)              */
	0x09, 0x51,                  /*     USAGE (Contact Identifier)   */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x05, 0x01,                  /*     USAGE_PAGE (Generic Desk..)  */
	0x26, 0x7F, 0x07,            /*     LOGICAL_MAXIMUM (1920-1)     */
	0x75, 0x10,                  /*     REPORT_SIZE (16)             */
	0x55, 0x0e,                  /*     UNIT_EXPONENT (-2)           */
	0x65, 0x11,                  /*     Unit (SI Lin: Length (cm))   */
	0x09, 0x30,                  /*     USAGE (X)                    */
	0x35, 0x00,                  /*     PHYSICAL_MINIMUM (0)         */
	0x46, 0x72, 0x0D,            /*     PHYSICAL_MAXIMUM (3442)      */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x46, 0x8F, 0x07,            /*     PHYSICAL_MAXIMUM (1935)      */
	0x26, 0xAF, 0x04,            /*     LOGICAL_MAXIMUM (1200-1)     */
	0x09, 0x31,                  /*     USAGE (Y)                    */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x05, 0x0d,                  /*     USAGE_PAGE (Digitizers)      */
	0x09, 0x30,                  /*     USAGE (Tip Pressure)         */
	0x26, MAX_PRESSURE, 0x00,    /*     LOGICAL_MAXIMUM (255)        */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x55, 0x00,                  /*     UNIT_EXPONENT (0)            */
	0x65, 0x00,                  /*     UNIT (None)                  */
	0x47, 0xff, 0xff, 0x00, 0x00,/*     PHYSICAL_MAXIMUM (65535)     */
	0x27, 0xff, 0xff, 0x00, 0x00,/*     LOGICAL_MAXIMUM (65535)      */
	0x75, 0x10,                  /*     REPORT_SIZE (16)             */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x09, 0x56,                  /*     USAGE (Scan Time)            */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x09, 0x54,                  /*     USAGE (Contact count)        */
	0x25, 0x7f,                  /*     LOGICAL_MAXIMUM (127)        */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x75, 0x08,                  /*     REPORT_SIZE (8)              */
	0x81, 0x02,                  /*     INPUT (Data,Var,Abs)         */
	0x85, REPORTID_MAX_COUNT,    /*     REPORT_ID (Feature)          */
	0x09, 0x55,                  /*     USAGE(Contact Count Maximum) */
	0x95, 0x01,                  /*     REPORT_COUNT (1)             */
	0x25, 0x02,                  /*     LOGICAL_MAXIMUM (2)          */
	0xb1, 0x02,                  /*     FEATURE (Data,Var,Abs)       */
	0xc0,                        /*   END_COLLECTION                 */
	0xc0,                        /* END_COLLECTION                   */
	}
};

static int major, minors;
static struct class *digitizer_class;

static struct f_digitizer *g_digitizer;

static int report_out(const char *buffer, size_t count);
static void hid_work(struct work_struct *wk);
static int hid_event_pen(const struct event_ts *evt);
static int hid_event_touch(const unsigned char num,
				const struct event_ts *evt);

/*-------------------------------------------------------------------------*/
/*                            HID gadget struct                            */

struct f_digitizer_req_list {
	struct usb_request	*req;
	unsigned int		pos;
	struct list_head	list;
};

struct f_digitizer {
	/* configuration */
	unsigned char			subclass;
	unsigned char			protocol;
	unsigned short			report_desc_length;
	char				*report_desc;
	unsigned short			report_length;

	/* recv report */
	struct list_head		completed_out_req;
	spinlock_t			spinlock;
	wait_queue_head_t		read_queue;
	unsigned int			qlen;

	/* send report */
	struct mutex			lock;
	bool				write_pending;
	wait_queue_head_t		write_queue;
	struct usb_request		*req;

	int				minor;
	struct cdev			cdev;
	struct usb_function		func;

	struct usb_ep			*in_ep;
	struct usb_ep			*out_ep;

	struct workqueue_struct *hid_wq;
	struct list_head  list;
	unsigned int		l_len;
	spinlock_t			s_lock;
};


static inline struct f_digitizer *func_to_digitizer(struct usb_function *f)
{
	return container_of(f, struct f_digitizer, func);
}

/*-------------------------------------------------------------------------*/
/*                           Static descriptors                            */

static struct usb_interface_descriptor digitizer_interface_desc = {
	.bLength		= sizeof(digitizer_interface_desc),
	.bDescriptorType	= USB_DT_INTERFACE,
	/* .bInterfaceNumber	= DYNAMIC */
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_HID,
	/* .bInterfaceSubClass	= DYNAMIC */
	/* .bInterfaceProtocol	= DYNAMIC */
	/* .iInterface		= DYNAMIC */
};

static struct hid_descriptor digitizer_desc = {
	.bLength			= sizeof(digitizer_desc),
	.bDescriptorType		= HID_DT_HID,
	.bcdHID				= 0x0101,
	.bCountryCode			= 0x00,
	.bNumDescriptors		= 0x1,
	/*.desc[0].bDescriptorType	= DYNAMIC */
	/*.desc[0].wDescriptorLenght	= DYNAMIC */
};

/* High-Speed Support */

static struct usb_endpoint_descriptor digitizer_hs_in_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 4, /* FIXME: Add this field in the
				      * HID gadget configuration?
				      * (struct digitizer_func_descriptor)
				      */
};

static struct usb_endpoint_descriptor digitizer_hs_out_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 4, /* FIXME: Add this field in the
				      * HID gadget configuration?
				      * (struct digitizer_func_descriptor)
				      */
};

static struct usb_descriptor_header *digitizer_hs_descriptors[] = {
	(struct usb_descriptor_header *)&digitizer_interface_desc,
	(struct usb_descriptor_header *)&digitizer_desc,
	(struct usb_descriptor_header *)&digitizer_hs_in_ep_desc,
	NULL,
};

/* Full-Speed Support */

static struct usb_endpoint_descriptor digitizer_fs_in_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 10, /* FIXME: Add this field in the
				       * HID gadget configuration?
				       * (struct digitizer_func_descriptor)
				       */
};

static struct usb_endpoint_descriptor digitizer_fs_out_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 10, /* FIXME: Add this field in the
				       * HID gadget configuration?
				       * (struct digitizer_func_descriptor)
				       */
};

static struct usb_descriptor_header *digitizer_fs_descriptors[] = {
	(struct usb_descriptor_header *)&digitizer_interface_desc,
	(struct usb_descriptor_header *)&digitizer_desc,
	(struct usb_descriptor_header *)&digitizer_fs_in_ep_desc,
	NULL,
};

/*-------------------------------------------------------------------------*/
/*                              Char Device                                */

static ssize_t f_digitizer_read(struct file *file, char __user *buffer,
			size_t count, loff_t *ptr)
{
	struct f_digitizer *digitizer = file->private_data;
	struct f_digitizer_req_list *list;
	struct usb_request *req;
	unsigned long flags;
	int ret;

	if (!count)
		return 0;

	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;

	spin_lock_irqsave(&digitizer->spinlock, flags);

#define READ_COND (!list_empty(&digitizer->completed_out_req))

	/* wait for at least one buffer to complete */
	while (!READ_COND) {
		spin_unlock_irqrestore(&digitizer->spinlock, flags);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(digitizer->read_queue, READ_COND))
			return -ERESTARTSYS;

		spin_lock_irqsave(&digitizer->spinlock, flags);
	}

	/* pick the first one */
	list = list_first_entry(&digitizer->completed_out_req,
				struct f_digitizer_req_list, list);
	req = list->req;
	count = min_t(unsigned int, count, req->actual - list->pos);
	spin_unlock_irqrestore(&digitizer->spinlock, flags);

	/* copy to user outside spinlock */
	count -= copy_to_user(buffer, req->buf + list->pos, count);
	list->pos += count;

	/*
	 * if this request is completely handled and transfered to
	 * userspace, remove its entry from the list and requeue it
	 * again. Otherwise, we will revisit it again upon the next
	 * call, taking into account its current read position.
	 */
	if (list->pos == req->actual) {
		spin_lock_irqsave(&digitizer->spinlock, flags);
		list_del(&list->list);
		kfree(list);
		spin_unlock_irqrestore(&digitizer->spinlock, flags);

		req->length = digitizer->report_length;
		ret = usb_ep_queue(digitizer->out_ep, req, GFP_KERNEL);
		if (ret < 0)
			return ret;
	}

	return count;
}

static void f_digitizer_req_complete(struct usb_ep *ep,
				struct usb_request *req)
{
	struct f_digitizer *digitizer = (struct f_digitizer *)ep->driver_data;

	if (req->status != 0) {
		ERROR(digitizer->func.config->cdev,
			"End Point Request ERROR: %d\n", req->status);
	}

	digitizer->write_pending = 0;
	wake_up(&digitizer->write_queue);
}

static ssize_t f_digitizer_write(struct file *file, const char __user *buffer,
			    size_t count, loff_t *offp)
{
	struct f_digitizer *digitizer  = file->private_data;
	ssize_t status = -ENOMEM;
	unsigned char *p;

	if (!access_ok(VERIFY_READ, buffer, count))
		return -EFAULT;

	mutex_lock(&digitizer->lock);

#define WRITE_COND (!digitizer->write_pending)

	/* write queue */
	while (!WRITE_COND) {
		mutex_unlock(&digitizer->lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible_exclusive(
				digitizer->write_queue, WRITE_COND))
			return -ERESTARTSYS;

		mutex_lock(&digitizer->lock);
	}

	count  = min_t(unsigned, count, digitizer->report_length);
	status = copy_from_user(digitizer->req->buf, buffer, count);

	if (status != 0) {
		ERROR(digitizer->func.config->cdev,
			"copy_from_user error\n");
		mutex_unlock(&digitizer->lock);
		return -EINVAL;
	}

	p = digitizer->req->buf;
	if (p[0] == 0)
		hid_event_pen((struct event_ts *)&p[1]);
	else
		hid_event_touch(p[1], (struct event_ts *)&p[2]);
	status = count;

	mutex_unlock(&digitizer->lock);

	return status;
}

static unsigned int f_digitizer_poll(struct file *file, poll_table *wait)
{
	struct f_digitizer	*digitizer  = file->private_data;
	unsigned int	ret = 0;

	poll_wait(file, &digitizer->read_queue, wait);
	poll_wait(file, &digitizer->write_queue, wait);

	if (WRITE_COND)
		ret |= POLLOUT | POLLWRNORM;

	if (READ_COND)
		ret |= POLLIN | POLLRDNORM;

	return ret;
}

#undef WRITE_COND
#undef READ_COND

static int f_digitizer_release(struct inode *inode, struct file *fd)
{
	fd->private_data = NULL;
	return 0;
}

static int f_digitizer_open(struct inode *inode, struct file *fd)
{
	struct f_digitizer *digitizer =
		container_of(inode->i_cdev, struct f_digitizer, cdev);

	fd->private_data = digitizer;

	return 0;
}

/*                                usb_function                             */

static int digitizer_setup(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct f_digitizer		*digitizer = func_to_digitizer(f);
	struct usb_composite_dev	*cdev = f->config->cdev;
	struct usb_request		*req  = cdev->req;
	int status = 0;
	__u16 value, length;
	__u8 *byte;

	value	= __le16_to_cpu(ctrl->wValue);
	length	= __le16_to_cpu(ctrl->wLength);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_GET_REPORT):
		VDBG(cdev, "get_report\n");

		/* send an empty report */
		length = min_t(unsigned, length, digitizer->report_length);
		memset(req->buf, 0x0, length);

		if ((value & 0xff) == REPORTID_MAX_COUNT) {
			byte = (__u8 *) req->buf;
			*byte++ = REPORTID_MAX_COUNT;	/* REPORTID */
			*byte++ = MAX_FINGER;	/* Contact Count Maximum */
		}

		goto respond;
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_GET_PROTOCOL):
		VDBG(cdev, "get_protocol\n");
		goto stall;
		break;

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_SET_REPORT):
		VDBG(cdev, "set_report | wLenght=%d\n", ctrl->wLength);
		goto stall;
		break;

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_SET_PROTOCOL):
		VDBG(cdev, "set_protocol\n");
		goto stall;
		break;

	case ((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8
		  | USB_REQ_GET_DESCRIPTOR):
		switch (value >> 8) {
		case HID_DT_HID:
			VDBG(cdev, "USB_REQ_GET_DESCRIPTOR: HID\n");
			length = min_t(unsigned short, length,
						   digitizer_desc.bLength);
			memcpy(req->buf, &digitizer_desc, length);
			goto respond;
			break;
		case HID_DT_REPORT:
			VDBG(cdev, "USB_REQ_GET_DESCRIPTOR: REPORT\n");
			length = min_t(unsigned short, length,
					digitizer->report_desc_length);
			memcpy(req->buf, digitizer->report_desc, length);
			goto respond;
			break;

		default:
			VDBG(cdev, "Unknown descriptor request 0x%x\n",
				 value >> 8);
			goto stall;
			break;
		}
		break;

	default:
		VDBG(cdev, "Unknown request 0x%x\n",
			 ctrl->bRequest);
		goto stall;
		break;
	}

stall:
	return -EOPNOTSUPP;

respond:
	req->zero = 0;
	req->length = length;
	status = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (status < 0)
		ERROR(cdev, "usb_ep_queue error on ep0 %d\n", value);
	return status;
}

static void digitizer_disable(struct usb_function *f)
{
	struct f_digitizer *digitizer = func_to_digitizer(f);
	struct f_digitizer_req_list *list, *next;

	usb_ep_disable(digitizer->in_ep);
	digitizer->in_ep->driver_data = NULL;

	list_for_each_entry_safe(list, next, &digitizer->completed_out_req,
		list)
	{
		list_del(&list->list);
		kfree(list);
	}
}

static int digitizer_set_alt(struct usb_function *f,
				unsigned intf, unsigned alt)
{
	struct usb_composite_dev	*cdev = f->config->cdev;
	struct f_digitizer		*digitizer = func_to_digitizer(f);
	int status = 0;

	VDBG(cdev, "digitizer_set_alt intf:%d alt:%d\n", intf, alt);

	if (digitizer->in_ep != NULL) {
		/* restart endpoint */
		if (digitizer->in_ep->driver_data != NULL)
			usb_ep_disable(digitizer->in_ep);

		status = config_ep_by_speed(f->config->cdev->gadget, f,
					    digitizer->in_ep);
		if (status) {
			ERROR(cdev, "config_ep_by_speed FAILED!\n");
			goto fail;
		}
		status = usb_ep_enable(digitizer->in_ep);
		if (status < 0) {
			ERROR(cdev, "Enable IN endpoint FAILED!\n");
			goto fail;
		}
		digitizer->in_ep->driver_data = digitizer;
	}

fail:
	return status;
}

const struct file_operations f_digitizer_fops = {
	.owner		= THIS_MODULE,
	.open		= f_digitizer_open,
	.release	= f_digitizer_release,
	.write		= f_digitizer_write,
	.read		= f_digitizer_read,
	.poll		= f_digitizer_poll,
	.llseek		= noop_llseek,
};

static int digitizer_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_ep		*ep;
	struct f_digitizer		*digitizer = func_to_digitizer(f);
	int			status;
	dev_t			dev;

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	digitizer_interface_desc.bInterfaceNumber = status;

	/* allocate instance-specific endpoints */
	status = -ENODEV;
	ep = usb_ep_autoconfig(c->cdev->gadget, &digitizer_fs_in_ep_desc);
	if (!ep)
		goto fail;
	ep->driver_data = c->cdev;	/* claim */
	digitizer->in_ep = ep;

	/* preallocate request and buffer */
	status = -ENOMEM;
	digitizer->req = usb_ep_alloc_request(digitizer->in_ep, GFP_KERNEL);
	if (!digitizer->req)
		goto fail;

	digitizer->req->buf = kmalloc(digitizer->report_length, GFP_KERNEL);
	if (!digitizer->req->buf)
		goto fail;

	/* set descriptor dynamic values */
	digitizer_interface_desc.bInterfaceSubClass =
		digitizer->subclass;
	digitizer_interface_desc.bInterfaceProtocol =
		digitizer->protocol;
	digitizer_hs_in_ep_desc.wMaxPacketSize =
		cpu_to_le16(digitizer->report_length);
	digitizer_fs_in_ep_desc.wMaxPacketSize =
		cpu_to_le16(digitizer->report_length);
	digitizer_hs_out_ep_desc.wMaxPacketSize =
		cpu_to_le16(digitizer->report_length);
	digitizer_fs_out_ep_desc.wMaxPacketSize =
		cpu_to_le16(digitizer->report_length);
	digitizer_desc.desc[0].bDescriptorType = HID_DT_REPORT;
	digitizer_desc.desc[0].wDescriptorLength =
		cpu_to_le16(digitizer->report_desc_length);

	digitizer_hs_in_ep_desc.bEndpointAddress =
		digitizer_fs_in_ep_desc.bEndpointAddress;
	digitizer_hs_out_ep_desc.bEndpointAddress =
		digitizer_fs_out_ep_desc.bEndpointAddress;

	status = usb_assign_descriptors(f, digitizer_fs_descriptors,
			digitizer_hs_descriptors, NULL);
	if (status)
		goto fail;

	mutex_init(&digitizer->lock);
	spin_lock_init(&digitizer->spinlock);
	spin_lock_init(&digitizer->s_lock);

	init_waitqueue_head(&digitizer->write_queue);
	init_waitqueue_head(&digitizer->read_queue);
	INIT_LIST_HEAD(&digitizer->completed_out_req);

	/* create char device */
	cdev_init(&digitizer->cdev, &f_digitizer_fops);
	dev = MKDEV(major, digitizer->minor);
	status = cdev_add(&digitizer->cdev, dev, 1);
	if (status)
		goto fail_free_descs;

	device_create(digitizer_class, NULL, dev, NULL, "%s%d",
					"digitizer", digitizer->minor);

	return 0;

fail_free_descs:
	usb_free_all_descriptors(f);
fail:
	ERROR(f->config->cdev, "digitizer_bind FAILED\n");
	if (digitizer->req != NULL) {
		kfree(digitizer->req->buf);
		if (digitizer->in_ep != NULL)
			usb_ep_free_request(digitizer->in_ep, digitizer->req);
	}

	return status;
}

static void digitizer_unbind(struct usb_configuration *c,
						struct usb_function *f)
{
	struct f_digitizer *digitizer = func_to_digitizer(f);

	device_destroy(digitizer_class, MKDEV(major, digitizer->minor));
	cdev_del(&digitizer->cdev);

	/* disable/free request and end point */
	usb_ep_disable(digitizer->in_ep);
	usb_ep_dequeue(digitizer->in_ep, digitizer->req);
	kfree(digitizer->req->buf);
	usb_ep_free_request(digitizer->in_ep, digitizer->req);

	usb_free_all_descriptors(f);

	kfree(digitizer->report_desc);
	kfree(digitizer);
}

/*-------------------------------------------------------------------------*/
/*                                 Strings                                 */

#define CT_FUNC_HID_IDX	0

static struct usb_string ct_func_string_defs[] = {
	[CT_FUNC_HID_IDX].s	= "HID Interface",
	{},			/* end of list */
};

static struct usb_gadget_strings ct_func_string_table = {
	.language	= 0x0409,	/* en-US */
	.strings	= ct_func_string_defs,
};

static struct usb_gadget_strings *ct_func_strings[] = {
	&ct_func_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/
/*                             usb_configuration                           */

void tegra_digitizer_unbind_config(void)
{
	destroy_workqueue(g_digitizer->hid_wq);
	g_digitizer = NULL;
}

int tegra_digitizer_bind_config(struct usb_configuration *c,
			    struct hidg_func_descriptor *fdesc, int index)
{
	struct f_digitizer *digitizer;
	int status;

	if (index >= minors)
		return -ENOENT;

	/* maybe allocate device-global string IDs, and patch descriptors */
	if (ct_func_string_defs[CT_FUNC_HID_IDX].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		ct_func_string_defs[CT_FUNC_HID_IDX].id = status;
		digitizer_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	digitizer = kzalloc(sizeof(*digitizer), GFP_KERNEL);
	if (!digitizer)
		return -ENOMEM;

	digitizer->minor = index;
	digitizer->subclass = fdesc->subclass;
	digitizer->protocol = fdesc->protocol;
	digitizer->report_length = fdesc->report_length;
	digitizer->report_desc_length = fdesc->report_desc_length;
	digitizer->report_desc = kmemdup(fdesc->report_desc,
				    fdesc->report_desc_length,
				    GFP_KERNEL);
	if (!digitizer->report_desc) {
		kfree(digitizer);
		return -ENOMEM;
	}

	digitizer->func.name    = "digitizer";
	digitizer->func.strings = ct_func_strings;
	digitizer->func.bind    = digitizer_bind;
	digitizer->func.unbind  = digitizer_unbind;
	digitizer->func.set_alt = digitizer_set_alt;
	digitizer->func.disable = digitizer_disable;
	digitizer->func.setup   = digitizer_setup;

	/* this could me made configurable at some point */
	digitizer->qlen	   = 4;
	digitizer->hid_wq = create_singlethread_workqueue("digitizer_wq");
	INIT_LIST_HEAD(&digitizer->list);
	digitizer->l_len = 0;

	g_digitizer = digitizer;

	status = usb_add_function(c, &digitizer->func);
	if (status)
		kfree(digitizer);

	return status;
}

int tegra_digitizer_setup(struct usb_gadget *g, int count)
{
	int status;
	dev_t dev;

	digitizer_class = class_create(THIS_MODULE, "digitizer");

	status = alloc_chrdev_region(&dev, 0, count, "digitizer");
	if (!status) {
		major = MAJOR(dev);
		minors = count;
	}

	return status;
}

void tegra_digitizer_cleanup(void)
{
	if (major) {
		unregister_chrdev_region(MKDEV(major, 0), minors);
		major = minors = 0;
	}

	class_destroy(digitizer_class);
	digitizer_class = NULL;
}

static ssize_t usb_data_out(const char *buffer, size_t count)
{
	struct f_digitizer *digitizer  = g_digitizer;
	ssize_t status = -ENOMEM;

	mutex_lock(&digitizer->lock);

#define WRITE_COND (!digitizer->write_pending)

	/* write queue */
	while (!WRITE_COND) {
		mutex_unlock(&digitizer->lock);

		if (wait_event_interruptible_exclusive(
				digitizer->write_queue, WRITE_COND))
			return -ERESTARTSYS;

		mutex_lock(&digitizer->lock);
	}

	count  = min_t(unsigned, count, digitizer->report_length);
	memcpy(digitizer->req->buf, buffer, count);

	digitizer->req->status   = 0;
	digitizer->req->zero     = 0;
	digitizer->req->length   = count;
	digitizer->req->complete = f_digitizer_req_complete;
	digitizer->req->context  = digitizer;
	digitizer->write_pending = 1;

	status = usb_ep_queue(digitizer->in_ep, digitizer->req, GFP_ATOMIC);
	if (status < 0) {
		ERROR(digitizer->func.config->cdev,
			"usb_ep_queue error on int endpoint %zd\n", status);
		digitizer->write_pending = 0;
		wake_up(&digitizer->write_queue);
	} else {
		status = count;
	}

	mutex_unlock(&digitizer->lock);

	return status;
}

#define MAX_QUEUE			20

struct report_tx {
	struct list_head list;
	char *buffer;
	size_t count;
};

static int hid_event_touch(const unsigned char num, const struct event_ts *evt)
{
	struct report_touch report;
	static struct event_ts evt_last[MAX_FINGER];
	static unsigned char num_last;
	int i;

	if (g_digitizer == NULL) {
		pr_err("%s: hid inactive!", __func__);
		return -ENODEV;
	}

	if (num == num_last) {
		if (!memcmp(evt, evt_last, num*sizeof(struct event_ts)))
			return 0;
	}

	num_last = num;
	memcpy(evt_last, evt, num*sizeof(struct event_ts));

	for (i = 0; i < num; i++) {
		const struct event_ts *e = &evt[i];
		memset(&report, 0x0, sizeof(report));
		report.report_id = REPORTID_TOUCH;
		report.tip = e->tip;
		report.contact_id = e->id;
		if (report.tip) {
			report.x = e->x;
			report.y = e->y;
			report.pressure = e->pressure;
		}
		report.scan_time = 0;
		if (i == 0)
			report.contact_count = num;
		report_out((char *)&report, sizeof(report));
	}

	return 0;
}

static int hid_event_pen(const struct event_ts *evt)
{
	struct report_pen report;
	static struct event_ts evt_last;

	if (g_digitizer == NULL) {
		pr_err("%s: hid inactive!", __func__);
		return -ENODEV;
	}
	if (!memcmp(evt, &evt_last, sizeof(evt_last)))
		return 0;

	memcpy(&evt_last, evt, sizeof(evt_last));

	if (g_digitizer->l_len >= MAX_QUEUE) {
		pr_err("%s: hid queue full!", __func__);
		return 0;
	}

	memset(&report, 0x0, sizeof(report));
	report.report_id = REPORTID_PEN;
	report.tip = evt->tip;
	if (report.tip) {
		report.x = evt->x;
		report.y = evt->y;
		report.pressure = evt->pressure;
	}
	report_out((char *)&report, sizeof(report));

	return 0;
}

static int report_out(const char *buffer, size_t count)
{
	struct report_tx *p;
	struct work_struct *hid_wk;
	int ret;

	p = kmalloc(sizeof(struct report_tx), GFP_KERNEL);
	p->count = count;
	p->buffer = kmalloc(count, GFP_KERNEL);

	memcpy(p->buffer, buffer, count);

	INIT_LIST_HEAD(&p->list);
	spin_lock(&g_digitizer->s_lock);
	list_add_tail(&p->list, &g_digitizer->list);
	g_digitizer->l_len++;
	spin_unlock(&g_digitizer->s_lock);

	hid_wk = kmalloc(sizeof(struct work_struct), GFP_KERNEL);

	INIT_WORK(hid_wk, hid_work);
	ret = queue_work(g_digitizer->hid_wq, hid_wk);
	if (!ret)
		pr_err("%s: failed to queue work!", __func__);

	return ret;
}

static void hid_work(struct work_struct *wk)
{
	struct list_head *pos;
	struct report_tx *p;

	kfree(wk);

	while (!list_empty(&g_digitizer->list)) {
		pos = g_digitizer->list.next;
		p = list_entry(pos, struct report_tx, list);
		spin_lock(&g_digitizer->s_lock);

		list_del(pos);
		g_digitizer->l_len--;
		spin_unlock(&g_digitizer->s_lock);
		usb_data_out(p->buffer, p->count);
		kfree(p->buffer);
		kfree(p);
	}
}