/*
* tegra-xotg.c - Nvidia XOTG implementation
*
* Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/usb.h>
#include <linux/device.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/extcon.h>
#include <asm/unaligned.h>
#include <mach/tegra_usb_pad_ctrl.h>
#include "linux/usb/hcd.h"
#include "linux/usb/otg.h"
#include "tegra-xotg.h"

int xotg_debug_level = LEVEL_INFO;
module_param(xotg_debug_level, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(xotg_debug_level, "level 0~4");

static const char driver_name[] = "tegra-xotg";

static void xotg_notify_event(struct xotg *xotg, int event)
{
	spin_lock(&xotg->phy.sync_lock);

	xotg->phy.last_event = event;
	atomic_notifier_call_chain(&xotg->phy.notifier, event, NULL);

	spin_unlock(&xotg->phy.sync_lock);
}

static ssize_t debug_store(struct device *_dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct xotg *xotg = dev_get_drvdata(_dev);

	if (sysfs_streq(buf, "enable_host")) {
		xotg_info(xotg->dev, "AppleOTG: setting up host mode\n");
		xotg->id_grounded = true;
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
			USB2_VBUS_ID_0_VBUS_OVERRIDE, 0);
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
			USB2_VBUS_ID_0_ID_OVERRIDE,
			USB2_VBUS_ID_0_ID_OVERRIDE_RID_GND);
		/* pad protection for host mode */
		xusb_enable_pad_protection(0);
	}
	if (sysfs_streq(buf, "enable_device")) {
		xotg_info(xotg->dev, "AppleOTG: setting up device mode\n");
		xotg->id_grounded = false;
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
			USB2_VBUS_ID_0_ID_OVERRIDE,
			USB2_VBUS_ID_0_ID_OVERRIDE_RID_FLOAT);
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
			USB2_VBUS_ID_0_VBUS_OVERRIDE,
			USB2_VBUS_ID_0_VBUS_OVERRIDE);
		/* pad protection for device mode */
		xusb_enable_pad_protection(1);
	}
	return size;
}
static DEVICE_ATTR(debug, S_IWUSR, NULL, debug_store);

static int extcon_id_notifications(struct notifier_block *nb,
				   unsigned long event, void *unused)
{
	struct xotg *xotg =
			container_of(nb, struct xotg, id_extcon_nb);
	unsigned long flags;

	spin_lock_irqsave(&xotg->lock, flags);

	if (extcon_get_cable_state(xotg->id_extcon_dev, "USB-Host")) {
		xotg_info(xotg->dev, "USB_ID pin grounded\n");
		xotg->id_grounded = true;
		xotg->phy.state = OTG_STATE_A_IDLE;
	} else {
		xotg_info(xotg->dev, "USB_ID pin floating\n");
		xotg->id_grounded = false;
		xotg->phy.state = OTG_STATE_B_IDLE;
	}

	schedule_work(&xotg->vbus_work);
	spin_unlock_irqrestore(&xotg->lock, flags);
	return NOTIFY_DONE;
}

static int xotg_enable_vbus(struct xotg *xotg)
{
	int ret = 0;

	if (!IS_ERR_OR_NULL(xotg->usb_vbus_reg)) {
		xotg_info(xotg->dev, "enabling vbus\n");
		if (!xotg->vbus_enabled) {
			ret = regulator_enable(xotg->usb_vbus_reg);
			if (ret < 0) {
				xotg_err(xotg->dev,
				"vbus0 enable failed. ret=%d\n", ret);
			} else {
				mutex_lock(&xotg->vbus_lock);
				xotg->vbus_enabled = true;
				mutex_unlock(&xotg->vbus_lock);
			}
		}
	} else {
		ret = -ENODEV;
	}

	return ret;
}

static int xotg_disable_vbus(struct xotg *xotg)
{
	int ret = 0;

	if (!IS_ERR_OR_NULL(xotg->usb_vbus_reg)) {
		xotg_info(xotg->dev, "disabling vbus\n");
		if (xotg->vbus_enabled) {
			ret = regulator_disable(xotg->usb_vbus_reg);
			if (ret < 0) {
				xotg_err(xotg->dev,
				"vbus0 disable failed. ret=%d\n", ret);
			} else {
				mutex_lock(&xotg->vbus_lock);
				xotg->vbus_enabled = false;
				mutex_unlock(&xotg->vbus_lock);
			}
		}
	} else {
		ret = -ENODEV;
	}

	return ret;
}

static void nv_vbus_work(struct work_struct *work)
{
	struct xotg *xotg = container_of(work, struct xotg, vbus_work);

	if (xotg->id_grounded)
		xotg_notify_event(xotg, USB_EVENT_ID);
	else
		xotg_notify_event(xotg, USB_EVENT_NONE);
}

/* will be called from the HCD during its initialization */
static int xotg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct xotg *xotg;

	if (!otg)
		return -ENODEV;

	if (host) {
		xotg = container_of(otg->phy, struct xotg, phy);
		xotg_info(xotg->dev, "host = 0x%p\n", host);

		otg->host = host;
		/* set the hs_otg_port here for the usbcore/HCD to be able
		 * to access it by setting it in the usb_bus structure
		 */
		otg->host->otg_port = xotg->hs_otg_port + 1;

		extcon_id_notifications(&xotg->id_extcon_nb, 0, NULL);
	} else {
		otg->host = NULL;
		otg->host->otg_port = 0;
	}
	return 0;
}

/* will be called from the HCD during its USB3 shared_hcd initialization */
static int xotg_set_xhci_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct xotg *xotg;

	if (!otg)
		return -ENODEV;

	if (host) {
		xotg = container_of(otg->phy, struct xotg, phy);
		xotg_info(xotg->dev, "xhcihost = 0x%p\n", host);

		otg->xhcihost = host;
		/* set the otg_port here for the usbcore/HCD to be able
		 * to access it by setting it in the usb_bus structure
		 */
		otg->xhcihost->otg_port = xotg->ss_otg_port + 1;
	} else {
		otg->xhcihost = NULL;
		otg->xhcihost->otg_port = 0;
	}
	return 0;
}

static int xotg_notify_connect(struct usb_phy *phy, enum usb_device_speed speed)
{
	struct xotg *xotg = container_of(phy, struct xotg, phy);
	unsigned long flags;

	xotg_info(xotg->dev, "speed = 0x%d\n", speed);
	spin_lock_irqsave(&xotg->lock, flags);
	xotg->device_connected = true;
	xotg->phy.state = OTG_STATE_A_HOST;
	spin_unlock_irqrestore(&xotg->lock, flags);
	return 0;
}

static int xotg_set_vbus(struct usb_phy *phy, int on)
{
	struct xotg *xotg = container_of(phy, struct xotg, phy);
	unsigned long flags;

	if (on) {
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
			USB2_VBUS_ID_0_ID_OVERRIDE,
			USB2_VBUS_ID_0_ID_OVERRIDE_RID_GND);

		/* pad protection for host mode */
		xusb_enable_pad_protection(0);

		/* set PP */
		xotg_notify_event(xotg, USB_EVENT_HANDLE_OTG_PP);
		xotg_enable_vbus(xotg);
	} else {
		spin_lock_irqsave(&xotg->lock, flags);
		if (!xotg->device_connected) {
			spin_unlock_irqrestore(&xotg->lock, flags);
			xotg_disable_vbus(xotg);
			xotg_notify_event(xotg, USB_EVENT_HANDLE_OTG_PP);
			tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
				USB2_VBUS_ID_0_ID_OVERRIDE,
				USB2_VBUS_ID_0_ID_OVERRIDE_RID_FLOAT);

			/* pad protection for device mode */
			xusb_enable_pad_protection(1);
		} else {
			spin_unlock_irqrestore(&xotg->lock, flags);
		}
	}

	return 0;
}

static int xotg_notify_disconnect(struct usb_phy *phy,
		enum usb_device_speed speed)
{
	struct xotg *xotg = container_of(phy, struct xotg, phy);
	unsigned long flags;

	xotg_info(xotg->dev, "speed = 0x%d\n", speed);
	spin_lock_irqsave(&xotg->lock, flags);
	xotg->device_connected = false;
	xotg->phy.state = OTG_STATE_A_IDLE;
	spin_unlock_irqrestore(&xotg->lock, flags);

	if (!xotg->id_grounded)
		xotg_set_vbus(phy, 0);

	return 0;
}

/* will be called from the PCD */
static int xotg_set_peripheral(struct usb_otg *otg, struct usb_gadget *gadget)
{
	struct xotg *xotg;

	if (!otg || !gadget)
		return -1;

	xotg = container_of(otg->phy, struct xotg, phy);
	xotg_info(xotg->dev, "usb_gadget = 0x%p\n", gadget);

	otg->gadget = gadget;

	return 0;
}

static int xotg_set_power(struct usb_phy *phy, unsigned ma)
{
	return 0;
}

static void xotg_struct_init(struct xotg *xotg)
{
	/* initialize the spinlock */
	spin_lock_init(&xotg->lock);

	mutex_init(&xotg->vbus_lock);

	xotg->phy.label = "Nvidia USB XOTG PHY";
	xotg->phy.set_power = xotg_set_power;
	xotg->phy.set_vbus = xotg_set_vbus;
	xotg->phy.notify_connect = xotg_notify_connect;
	xotg->phy.notify_disconnect = xotg_notify_disconnect;

	xotg->phy.otg->phy = &xotg->phy;
	xotg->phy.otg->set_host = xotg_set_host;
	xotg->phy.otg->set_xhci_host = xotg_set_xhci_host;
	xotg->phy.otg->set_peripheral = xotg_set_peripheral;

	/* initial states */
	xotg_dbg(xotg->dev, "UNDEFINED\n");
	xotg->phy.state = OTG_STATE_UNDEFINED;

	/* hs_otg_port = 0 , ss_otg_port = 0 */
	xotg->hs_otg_port = 0;
	xotg->ss_otg_port = 0;
	xotg->device_connected = false;

	/* set default_a to 0 */
	xotg->phy.otg->default_a = 0;
}

/* start the OTG controller */
static int xotg_start(struct xotg *xotg)
{
	u32 reg;

	/* clear interrupts */
	reg = tegra_usb_pad_reg_read(XUSB_PADCTL_USB2_VBUS_ID_0);
	tegra_usb_pad_reg_write(XUSB_PADCTL_USB2_VBUS_ID_0, reg);

	/* TODO: vbus and ID interrupt can be enabled when needed */

	/* get the RID value from IDDIG/IDDIG_A/IDDIG_B/IDDIG_C bits */
	reg = tegra_usb_pad_reg_read(XUSB_PADCTL_USB2_VBUS_ID_0);
	reg &= USB2_VBUS_ID_0_RID_MASK;

	if (reg == USB2_VBUS_ID_0_RID_FLOAT
		|| reg == USB2_VBUS_ID_0_RID_B
		|| reg == USB2_VBUS_ID_0_RID_C) {
		xotg_info(xotg->dev, "ID=1\n");
		xotg->id = 1;
	} else if (reg == USB2_VBUS_ID_0_RID_A
		|| reg == USB2_VBUS_ID_0_RID_GND) {
		/* micro-A end attached */
		xotg_info(xotg->dev, "ID=0\n");
		xotg->id = 0;
	}
	xotg_info(xotg->dev, "UNDEFINED\n");
	xotg->phy.state = OTG_STATE_UNDEFINED;

	return 0;
}

static irqreturn_t xotg_phy_wake_isr(int irq, void *data)
{
	struct xotg *xotg = (struct xotg *)data;

	xotg_dbg(xotg->dev, "irq %d", irq);
	return IRQ_HANDLED;
}

/* xotg interrupt handler */
static irqreturn_t xotg_irq(int irq, void *data)
{
	struct xotg *xotg = (struct xotg *)data;
	unsigned long flags;
	u32 vbus_id_reg, rid_status;

	spin_lock_irqsave(&xotg->lock, flags);

	/* handle the ID CHANGE interrupt */
	vbus_id_reg = tegra_usb_pad_reg_read(XUSB_PADCTL_USB2_VBUS_ID_0);

	xotg_dbg(xotg->dev, "vbus_id_reg: 0x%x\n", vbus_id_reg);

	if (!(vbus_id_reg & USB2_VBUS_ID_0_INTR_STS_CHG_MASK)) {
		xotg_err(xotg->dev, "padctl interrupt is not for xotg %x\n",
			vbus_id_reg);
		spin_unlock_irqrestore(&xotg->lock, flags);
		return IRQ_NONE;
	}

	/* int generated only when id value changes */
	if (vbus_id_reg & USB2_VBUS_ID_0_IDDIG_STS_CHG) {
		xotg_dbg(xotg->dev, "IDDIG STS CHG\n");
		/* get the RID value from the ID bits */
		rid_status = vbus_id_reg & USB2_VBUS_ID_0_RID_MASK;

		if (rid_status == USB2_VBUS_ID_0_RID_GND ||
			rid_status == USB2_VBUS_ID_0_RID_A) {
			xotg_dbg(xotg->dev, "A_IDLE\n");
			xotg->phy.otg->default_a = 1;
			xotg->id = 0;
		} else if (rid_status == USB2_VBUS_ID_0_RID_B ||
			rid_status == USB2_VBUS_ID_0_RID_FLOAT) {
			xotg_dbg(xotg->dev, "B_IDLE\n");
			xotg->id = 1;
		}
	}

	/* write to clear the status */
	tegra_usb_pad_reg_write(XUSB_PADCTL_USB2_VBUS_ID_0, vbus_id_reg);

	spin_unlock_irqrestore(&xotg->lock, flags);
	return IRQ_HANDLED;
}

static int xotg_probe(struct platform_device *pdev)
{
	struct xotg *xotg;
	int status;
	struct resource *res;

	/* allocate space for nvidia XOTG device */
	xotg = kzalloc(sizeof(struct xotg), GFP_KERNEL);
	if (!xotg) {
		xotg_err(&pdev->dev, "memory alloc failed for xotg\n");
		return -ENOMEM;
	}

	/* allocate memory for usb_otg structure */
	xotg->phy.otg = kzalloc(sizeof(struct usb_otg), GFP_KERNEL);
	if (!xotg->phy.otg) {
		xotg_err(xotg->dev, "memory alloc failed for usb_otg\n");
		status = -ENOMEM;
		goto error1;
	}

	/* initialize the otg structure */
	xotg_struct_init(xotg);
	xotg->phy.dev = &pdev->dev;
	xotg->dev = &pdev->dev;
	xotg->pdev = pdev;
	platform_set_drvdata(pdev, xotg);
	xotg->phy.type = USB_PHY_TYPE_UNDEFINED;
	dev_set_drvdata(&pdev->dev, xotg);

	/* store the otg phy */
	status = usb_add_phy(&xotg->phy, USB_PHY_TYPE_USB3);
	if (status) {
		xotg_err(xotg->dev, "usb_add_phy failed\n");
		goto error2;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		xotg_err(xotg->dev, "irq resource 0 doesn't exist\n");
		goto error3;
	}
	xotg->nv_irq = res->start;

	/* register shared padctl IRQ for otg */
	status = devm_request_irq(&pdev->dev, xotg->nv_irq, xotg_irq,
			IRQF_SHARED | IRQF_TRIGGER_HIGH,
			"tegra_xotg_irq", xotg);
	if (status != 0) {
		xotg_err(xotg->dev, "padctl irq register fail %d\n", status);
		goto error3;
	}

	/* register phy wake isr */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		xotg_err(xotg->dev, "irq resource 1 doesn't exist\n");
		goto error4;
	}
	xotg->usb_irq = res->start;

	status = devm_request_irq(&pdev->dev, xotg->usb_irq, xotg_phy_wake_isr,
			IRQF_SHARED | IRQF_TRIGGER_HIGH,
			"xotg_phy_wake_isr", xotg);
	if (status != 0) {
		xotg_err(xotg->dev, "usb_irq register fail %d\n", status);
		goto error4;
	}

	xotg->vbus_enabled = false;
	INIT_WORK(&xotg->vbus_work, nv_vbus_work);

	/* regulator for usb_vbus, to be moved to OTG driver */
	xotg->usb_vbus_reg = devm_regulator_get(&pdev->dev, "usb_vbus");
	if (IS_ERR_OR_NULL(xotg->usb_vbus_reg)) {
		xotg_err(xotg->dev, "usb_vbus regulator not found: %ld\n",
			PTR_ERR(xotg->usb_vbus_reg));
		goto error5;
	}

	/* start OTG */
	status = xotg_start(xotg);
	if (status) {
		xotg_err(xotg->dev, "xotg_start failed\n");
		goto error6;
	}

	/* extcon for id pin */
	xotg->id_extcon_dev =
		extcon_get_extcon_dev_by_cable(&pdev->dev, "id");
	if (IS_ERR(xotg->id_extcon_dev))
		status = -ENODEV;

	xotg->id_extcon_nb.notifier_call = extcon_id_notifications;
	extcon_register_notifier(xotg->id_extcon_dev,
						&xotg->id_extcon_nb);

	status = device_create_file(xotg->dev, &dev_attr_debug);
	if (status)
		goto error5;

	return status;

error6:
	devm_regulator_put(xotg->usb_vbus_reg);
error5:
	devm_free_irq(&pdev->dev, xotg->usb_irq, xotg);
error4:
	devm_free_irq(&pdev->dev, xotg->nv_irq, xotg);
error3:
	usb_remove_phy(&xotg->phy);
error2:
	kfree(xotg->phy.otg);
error1:
	kfree(xotg);
	return status;
}

static int __exit xotg_remove(struct platform_device *pdev)
{
	struct xotg *xotg = platform_get_drvdata(pdev);

	device_remove_file(xotg->dev, &dev_attr_debug);
	extcon_unregister_notifier(xotg->id_extcon_dev,
		&xotg->id_extcon_nb);

	devm_regulator_put(xotg->usb_vbus_reg);
	devm_free_irq(&pdev->dev, xotg->nv_irq, xotg);
	devm_free_irq(&pdev->dev, xotg->usb_irq, xotg);
	usb_remove_phy(&xotg->phy);

	kfree(xotg->phy.otg);
	kfree(xotg);

	return 0;
}

static struct of_device_id tegra_xotg_of_match[] = {
	{.compatible = "nvidia,tegra210-xotg",},
	{},
};

static int xotg_resume_platform(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct xotg *xotg = platform_get_drvdata(pdev);

	xotg_info(xotg->dev, "%s\n", __func__);

	disable_irq_wake(xotg->usb_irq);
	/* restore USB2_ID register except when xudc driver updated it
	 * due to lp0 wake by connecting device mode cable
	 */
	if (xotg->phy.state != OTG_STATE_B_PERIPHERAL)
		tegra_usb_pad_reg_write(XUSB_PADCTL_USB2_VBUS_ID_0,
			xotg->usb2_id);
	return 0;
}

static int xotg_suspend_platform(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct xotg *xotg = platform_get_drvdata(pdev);
	int ret = 0;

	xotg_info(xotg->dev, "%s\n", __func__);

	/* save USB2_ID register */
	xotg->usb2_id = tegra_usb_pad_reg_read(XUSB_PADCTL_USB2_VBUS_ID_0);

	/* enable_irq_wake for otg port wakes */
	ret = enable_irq_wake(xotg->usb_irq);
	if (ret < 0) {
		xotg_err(xotg->dev,
		"Couldn't enable otg port wakeup, irq=%d, error=%d\n",
		xotg->usb_irq, ret);
	}

	return ret;
}

static const struct dev_pm_ops tegra_xotg_pm_ops = {
	.suspend = xotg_suspend_platform,
	.resume = xotg_resume_platform,
};

static struct platform_driver xotg_driver = {
	.probe = xotg_probe,
	.remove = __exit_p(xotg_remove),
	.driver = {
		.name = driver_name,
		.owner = THIS_MODULE,
		.of_match_table = tegra_xotg_of_match,
		.pm = &tegra_xotg_pm_ops,
	},
};

static int __init xotg_init(void)
{
	return platform_driver_register(&xotg_driver);
}
fs_initcall(xotg_init);

static void __exit xotg_exit(void)
{
	platform_driver_unregister(&xotg_driver);
}
module_exit(xotg_exit);

MODULE_DESCRIPTION("Nvidia USB XOTG Driver");
MODULE_VERSION("Rev. 1.00");
MODULE_AUTHOR("Ajay Gupta");
MODULE_LICENSE("GPL");
