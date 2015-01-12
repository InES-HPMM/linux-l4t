/*
 * USB LED Driver for NVIDIA Shield
 *
 * Copyright (c) 2013-2015, NVIDIA Corporation. All Rights Reserved.
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
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/sysedp.h>
#include <linux/kobject.h>

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
#define LED_NUM 2

#define UC_COMM_ON		0x10
#define UC_COMM_BLINK	0x11
#define UC_COMM_BREATHE	0x12
#define UC_COMM_OFF		0x13

#define UC_COMM_PWR_LED 0x00
#define UC_COMM_TCH_LED 0x80

#define VID 0x0955
#define PID 0x7205

#define LED_NVBUTTON 0
#define LED_TOUCH 1

#define LED_MIN_BRIGHTNESS	0
#define LED_MAX_BRIGHTNESS	255
#define MAX_EDP_BRIGHTNESS	511
#define NUM_EDP_STATES		11

struct nvshield_led *g_dev;
struct class_compat *nvshieldled_compat_class;

static const struct usb_device_id nvshield_table[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(VID, PID,
			INTF_CLASS, INTF_SUBCLASS, INTF_PROTOCOL) },
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
	bool			pwm_enabled;
	bool			nvshieldled_state;
	unsigned char		brightness[LED_NUM];
	enum led_state		state[LED_NUM];
	struct sysedp_consumer	*sysedpc;
};

static unsigned char brightness_table[256] = {
	0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
	3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 9,
	9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 15, 16, 16,
	17, 17, 18, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 24, 25, 26, 27,
	27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 38, 39, 39,
	40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72,
	73, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 90, 91, 92,
	93, 94, 96, 97, 98, 99, 100, 102, 103, 104, 106, 107, 108, 109, 111,
	112, 113, 115, 116, 117, 118, 120, 121, 122, 124, 125, 127, 128, 129,
	131, 132, 133, 135, 136, 138, 139, 140, 142, 143, 145, 146, 147, 149,
	150, 152, 153, 155, 156, 157, 159, 160, 162, 163, 165, 166, 168, 169,
	171, 172, 174, 175, 177, 178, 180, 181, 183, 184, 186, 187, 189, 190,
	192, 193, 195, 196, 198, 199, 201, 202, 204, 205, 207, 208, 210, 212,
	213, 215, 216, 218, 219, 221, 222, 224, 226, 227, 229, 230, 232, 233,
	235, 236, 238, 240, 241, 243, 244, 246, 247, 249, 251, 252, 254, 255
};

static void send_command(struct nvshield_led *led, int led_id)
{
	int retval = 0;
	unsigned char state;
	unsigned char brightness;

	if (led->state[led_id] == LED_BREATHE)
		state = UC_COMM_BREATHE;
	else if (led->state[led_id] == LED_BLINK)
		state = UC_COMM_BLINK;
	else if (led->state[led_id] == LED_NORMAL)
		state = UC_COMM_ON;
	else if (led->state[led_id] == LED_OFF)
		state = UC_COMM_OFF;
	else
		return;

	brightness = led->brightness[led_id];
	if (led_id == LED_TOUCH)
		state |= UC_COMM_TCH_LED;
	else
		brightness = brightness_table[(unsigned int)brightness];

	retval = usb_control_msg(led->udev,
			usb_sndctrlpipe(led->udev, 0),
			0x00,
			0x42,
			cpu_to_le16(brightness),
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

	return sprintf(buf, "%d\n", led->brightness[LED_NVBUTTON]);
}

static ssize_t show_brightness2(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);

	return sprintf(buf, "%d\n", led->brightness[LED_TOUCH]);
}

static unsigned int compute_edp_state(unsigned long brightness1,
				unsigned long brightness2)
{
	unsigned long effective_brightness = brightness1 + brightness2;
	unsigned int edp_state = (effective_brightness /
			((MAX_EDP_BRIGHTNESS - LED_MIN_BRIGHTNESS) /
			NUM_EDP_STATES)) - 1;
	return edp_state;
}

static ssize_t set_brightness(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);
	unsigned long brightness_val;
	unsigned int edp_state;

	if (!kstrtoul(buf, 10, &brightness_val)) {
		if (led->pwm_enabled)
			led->brightness[LED_NVBUTTON] =
					(unsigned char)brightness_val;
		else
			led->brightness[LED_NVBUTTON] =
					(unsigned char)LED_MAX_BRIGHTNESS;
		edp_state = compute_edp_state(brightness_val,
					led->brightness[LED_TOUCH]);
		if (edp_state > sysedp_get_state(led->sysedpc)) {
			sysedp_set_state(led->sysedpc, edp_state);
			send_command(led, LED_NVBUTTON);
		} else {
			send_command(led, LED_NVBUTTON);
			sysedp_set_state(led->sysedpc, edp_state);
		}
	}
	return count;
}

static ssize_t set_brightness2(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);
	unsigned long brightness_val;
	unsigned int edp_state;

	if (!kstrtoul(buf, 10, &brightness_val)) {
		if (led->pwm_enabled)
			led->brightness[LED_TOUCH] =
					(unsigned char)brightness_val;
		else {
			if (brightness_val <= 20)
				led->brightness[LED_TOUCH] =
					(unsigned char)LED_MIN_BRIGHTNESS;
			else
				led->brightness[LED_TOUCH] =
					(unsigned char)LED_MAX_BRIGHTNESS;
		}
		edp_state = compute_edp_state(brightness_val,
					led->brightness[LED_NVBUTTON]);
		if (edp_state > sysedp_get_state(led->sysedpc)) {
			sysedp_set_state(led->sysedpc, edp_state);
			send_command(led, LED_TOUCH);
		} else {
			send_command(led, LED_TOUCH);
			sysedp_set_state(led->sysedpc, edp_state);
		}
	}
	return count;
}

static ssize_t show_ledstate(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);

	switch (led->state[LED_NVBUTTON]) {
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
		led->state[LED_NVBUTTON] = LED_NORMAL;
	else if (!strcmp(ledstate_name, LED_STATE_BLINK))
		led->state[LED_NVBUTTON] = LED_BLINK;
	else if (!strcmp(ledstate_name, LED_STATE_BREATHE))
		led->state[LED_NVBUTTON] = LED_BREATHE;
	else if (!strcmp(ledstate_name, LED_STATE_OFF))
		led->state[LED_NVBUTTON] = LED_OFF;
	else
		return count;

	send_command(led, LED_NVBUTTON);
	return count;
}

static ssize_t show_ledstate2(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct nvshield_led *led = usb_get_intfdata(intf);

	switch (led->state[LED_TOUCH]) {
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

static ssize_t set_ledstate2(struct device *dev, struct device_attribute *attr,
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
		led->state[LED_TOUCH] = LED_NORMAL;
	else if (!strcmp(ledstate_name, LED_STATE_BLINK))
		led->state[LED_TOUCH] = LED_BLINK;
	else if (!strcmp(ledstate_name, LED_STATE_BREATHE))
		led->state[LED_TOUCH] = LED_BREATHE;
	else if (!strcmp(ledstate_name, LED_STATE_OFF))
		led->state[LED_TOUCH] = LED_OFF;
	else
		return count;

	send_command(led, LED_TOUCH);
	return count;
}

static DEVICE_ATTR(brightness, S_IRUGO,
		show_brightness, set_brightness);
static DEVICE_ATTR(state, S_IRUGO | S_IWUSR,
		show_ledstate, set_ledstate);
static DEVICE_ATTR(brightness2, S_IRUGO | S_IWUSR,
		show_brightness2, set_brightness2);
static DEVICE_ATTR(state2, S_IRUGO | S_IWUSR,
		show_ledstate2, set_ledstate2);

static int nvshieldled_reboot_callback(struct notifier_block *nb,
		unsigned long code,
		void *unused) {
	if (!g_dev)
		return NOTIFY_DONE;
	g_dev->state[LED_NVBUTTON] = LED_NORMAL;
	g_dev->state[LED_TOUCH] = LED_NORMAL;
	send_command(g_dev, LED_NVBUTTON);
	send_command(g_dev, LED_TOUCH);
	return NOTIFY_DONE;
}

static struct notifier_block nvshieldled_notifier = {
	.notifier_call = nvshieldled_reboot_callback,
};

static int nvshieldled_probe(struct usb_interface *interface,
		const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct nvshield_led *dev = NULL;
	int retval = -ENOMEM;
	struct device_node *np;
	char *edp_name;

	dev = kzalloc(sizeof(struct nvshield_led), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&interface->dev, "out of memory\n");
		goto error_mem;
	}

	np = of_find_node_by_name(NULL, "touch-leds-pwm-rework");
	if (np)
		dev->pwm_enabled = of_device_is_available(np);

	if (dev->pwm_enabled)
		dev_info(&interface->dev, "Enabling LED PWM\n");
	else
		dev_info(&interface->dev, "Disabling LED PWM\n");

	dev->udev = usb_get_dev(udev);
	np = of_find_node_by_name(NULL, "usb_nvshield_led");
	if (np) {
		dev->nvshieldled_state = of_device_is_available(np);
		if (dev->nvshieldled_state) {
			dev->state[LED_NVBUTTON] = LED_NORMAL;
			dev->brightness[LED_NVBUTTON] = 255;
			dev->state[LED_TOUCH] = LED_NORMAL;
			dev->brightness[LED_TOUCH] = 0;
		} else {
			dev->state[LED_NVBUTTON] = LED_OFF;
			dev->brightness[LED_NVBUTTON] = 255;
			dev->state[LED_TOUCH] = LED_OFF;
			dev->brightness[LED_TOUCH] = 0;
			send_command(dev, LED_TOUCH);
			send_command(dev, LED_NVBUTTON);
		}
	} else {
		dev->state[LED_NVBUTTON] = LED_NORMAL;
		dev->brightness[LED_NVBUTTON] = 255;
		dev->state[LED_TOUCH] = LED_NORMAL;
		dev->brightness[LED_TOUCH] = 255;
	}

	if (np) {
		if (of_property_read_string(np, "edp-consumer-name",
						&edp_name)) {
			dev_info(&interface->dev,
				"property 'edp-consumer-name' missing or \
				invalid. Setting LED EDP consumer name as \
				'shieldled' \n");
			dev->sysedpc = sysedp_create_consumer("shieldled",
								"shieldled");
		} else {
			dev->sysedpc = sysedp_create_consumer(edp_name,
								edp_name);
		}
	}

	usb_set_intfdata(interface, dev);
	g_dev = dev;

	retval = device_create_file(&interface->dev, &dev_attr_brightness);
	if (retval)
		goto error;
	retval = device_create_file(&interface->dev, &dev_attr_state);
	if (retval)
		goto error2;
	retval = device_create_file(&interface->dev, &dev_attr_brightness2);
	if (retval)
		goto error3;
	retval = device_create_file(&interface->dev, &dev_attr_state2);
	if (retval)
		goto error4;
	dev_info(&interface->dev, "Nvidia Shield LED attached\n");

	retval = kobject_uevent(&interface->dev.kobj, KOBJ_CHANGE);
	if (retval)
		dev_info(&interface->dev, "sent uevent successfully\n");
	else
		dev_info(&interface->dev, "failed to send uevent\n");

	retval = register_reboot_notifier(&nvshieldled_notifier);
	if (retval)
		goto error5;

	nvshieldled_compat_class = class_compat_register("nvshieldled");
	if (IS_ERR(nvshieldled_compat_class))
		goto error6;

	retval = class_compat_create_link(nvshieldled_compat_class,
					&interface->dev, NULL);
	if (retval)
		goto error7;

	retval = kobject_uevent(
			(*(struct kobject**)nvshieldled_compat_class),
								KOBJ_CHANGE);
	if (retval)
		dev_info(&interface->dev, "sent uevent successfully\n");
	else
		dev_info(&interface->dev, "failed to send uevent\n");

	return 0;

error7:
	class_compat_unregister(nvshieldled_compat_class);
error6:
	unregister_reboot_notifier(&nvshieldled_notifier);
error5:
	device_remove_file(&interface->dev, &dev_attr_state2);
error4:
	device_remove_file(&interface->dev, &dev_attr_brightness2);
error3:
	device_remove_file(&interface->dev, &dev_attr_brightness);
error2:
	device_remove_file(&interface->dev, &dev_attr_state);
error:
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

	unregister_reboot_notifier(&nvshieldled_notifier);
	device_remove_file(&interface->dev, &dev_attr_brightness);
	device_remove_file(&interface->dev, &dev_attr_state);
	device_remove_file(&interface->dev, &dev_attr_brightness2);
	device_remove_file(&interface->dev, &dev_attr_state2);
	sysedp_free_consumer(dev->sysedpc);

	class_compat_unregister(nvshieldled_compat_class);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);
	dev_info(&interface->dev, "Nvidia Shield LED disconnected\n");
}

static int nvshieldled_suspend(struct usb_interface *interface,
		pm_message_t notused)
{
	dev_info(&interface->dev, "Nvidia Shield LED suspended\n");
	return 0;
}

static int nvshieldled_resume(struct usb_interface *interface)
{
	dev_info(&interface->dev, "Nvidia Shield LED resumed\n");
	return 0;
}

static int nvshieldled_reset_resume(struct usb_interface *interface)
{
	dev_info(&interface->dev, "Nvidia Shield USB reset resume");
	return 0;
}

static struct usb_driver shieldled_driver = {
	.name =		"nvshieldled",
	.probe =	nvshieldled_probe,
	.disconnect = nvshieldled_disconnect,
	.suspend =	nvshieldled_suspend,
	.resume =	nvshieldled_resume,
	.reset_resume = nvshieldled_reset_resume,
	.id_table = nvshield_table,
};

module_usb_driver(shieldled_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
