/*
 * USB LED Driver for NVIDIA Shield
 *
 * Copyright (c) 2013, NVIDIA Corporation. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/slab.h>

#define DRIVER_AUTHOR "Jun Yan, juyan@nvidia.com"
#define DRIVER_DESC "NVIDIA Shield USB LED Driver"

#define INTF_CLASS 0xFF
#define INTF_SUBCLASS 0x01
#define INTF_PROTOCOL 0x01

#define LED_STATE_NORMAL	"normal"
#define LED_STATE_BLINK		"blink"
#define LED_STATE_BREATHE	"breathe"
#define LED_STATE_OFF		"off"
#define LED_STATE_UNKNOW	"unknow"

#define LED_STATE_MAXCHAR 8

#define UC_COMM_ON		0x10
#define UC_COMM_BLINK	0x11
#define UC_COMM_BREATHE	0x12
#define UC_COMM_OFF		0x13

#define VID 0x0955
#define PID 0x7205

static const struct usb_device_id nvshield_table[] = {
	{ USB_INTERFACE_INFO(INTF_CLASS, INTF_SUBCLASS, INTF_PROTOCOL) },
	{ USB_DEVICE(VID, PID)},
	{}
};
MODULE_DEVICE_TABLE(usb, nvshield_table);

enum led_state {
	LED_NORMAL,
	LED_BLINK,
	LED_BREATHE,
	LED_OFF,
};

struct nvshield_led {
	struct usb_device	*udev;
	unsigned char		brightness;
	enum led_state		state;
};

static void send_command(struct nvshield_led *led)
{
	int retval = 0;
	unsigned char state;

	if (led->state == LED_BREATHE)
		state = UC_COMM_BREATHE;
	else if (led->state == LED_BLINK)
		state = UC_COMM_BLINK;
	else if (led->state == LED_NORMAL)
		state = UC_COMM_ON;
	else if (led->state == LED_OFF)
		state = UC_COMM_OFF;
	else
		return;

	retval = usb_control_msg(led->udev,
			usb_sndctrlpipe(led->udev, 0),
			0x00,
			0x42,
			cpu_to_le16(led->brightness),
			cpu_to_le16(state),
			NULL,
			0,
			2000);
}

static ssize_t show_brightness(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);

	return sprintf(buf, "%d\n", led->brightness);
}

static ssize_t set_brightness(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);
	int brightness_val;

	if (!kstrtoul(buf, 10, &brightness_val)) {
		led->brightness = brightness_val;
		send_command(led);
	}
	return count;
}

static ssize_t show_ledstate(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);

	switch (led->state) {
	case LED_NORMAL:
		return sprintf(buf, "%s\n", LED_STATE_NORMAL);
	case LED_BLINK:
		return sprintf(buf, "%s\n", LED_STATE_BLINK);
	case LED_BREATHE:
		return sprintf(buf, "%s\n", LED_STATE_BREATHE);
	case LED_OFF:
		return sprintf(buf, "%s\n", LED_STATE_OFF);
	default:
		return sprintf(buf, LED_STATE_UNKNOW);
	}
}

static ssize_t set_ledstate(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);
	char ledstate_name[LED_STATE_MAXCHAR];
	size_t len;

	ledstate_name[sizeof(ledstate_name) - 1] = '\0';
	strncpy(ledstate_name, buf, sizeof(ledstate_name) - 1);
	len = strlen(ledstate_name);

	if (len && ledstate_name[len - 1] == '\n')
		ledstate_name[len - 1] = '\0';

	if (!strcmp(ledstate_name, LED_STATE_NORMAL))
		led->state = LED_NORMAL;
	else if (!strcmp(ledstate_name, LED_STATE_BLINK))
		led->state = LED_BLINK;
	else if (!strcmp(ledstate_name, LED_STATE_BREATHE))
		led->state = LED_BREATHE;
	else if (!strcmp(ledstate_name, LED_STATE_OFF))
		led->state = LED_OFF;
	else
		return count;

	send_command(led);
	return count;
}

static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR,
		show_brightness, set_brightness);
static DEVICE_ATTR(state, S_IRUGO | S_IWUSR,
		show_ledstate, set_ledstate);

static int nvshieldled_probe(struct usb_interface *interface,
		const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct nvshield_led *dev = NULL;
	int retval = -ENOMEM;

	dev = kzalloc(sizeof(struct nvshield_led), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&interface->dev, "out of memory\n");
		goto error_mem;
	}

	dev->udev = usb_get_dev(udev);
	dev->state = LED_NORMAL;
	dev->brightness = 255;

	usb_set_intfdata(interface, dev);

	retval = device_create_file(&interface->dev, &dev_attr_brightness);
	if (retval)
		goto error;
	retval = device_create_file(&interface->dev, &dev_attr_state);
	if (retval)
		goto error;
	dev_info(&interface->dev, "Nvidia Shield LED attached\n");
	return 0;

error:
	device_remove_file(&interface->dev, &dev_attr_brightness);
	device_remove_file(&interface->dev, &dev_attr_state);
	usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);
error_mem:
	return retval;
}

static void nvshieldled_disconnect(struct usb_interface *interface)
{
	struct nvshield_led *dev;

	dev = usb_get_intfdata(interface);

	device_remove_file(&interface->dev, &dev_attr_brightness);
	device_remove_file(&interface->dev, &dev_attr_state);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);
	dev_info(&interface->dev, "Nvidia Shield LED disconnected\n");
}

static struct usb_driver shieldled_driver = {
	.name =		"nvshieldled",
	.probe =	nvshieldled_probe,
	.disconnect = nvshieldled_disconnect,
	.id_table = nvshield_table,
};

module_usb_driver(shieldled_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
