/*
* nvxxx_udc.c - Nvidia device mode implementation
*
* Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#define DEBUG
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/errno.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/usb/otg.h>
#include <linux/usb/hcd.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/tegra-powergate.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/tegra-fuse.h>
#include <linux/extcon.h>
#include <mach/tegra_usb_pad_ctrl.h>
#include "nvxxx.h"
#include "../../../arch/arm/mach-tegra/iomap.h"

static const char const driver_name[] = "tegra-xudc";
static struct NV_UDC_S *the_controller;

static const char * const udc_regulator_names[] = {
	"hvdd_usb",		/* TODO 3.3V */
	"avddio_usb",		/* TODO 1.05V */
	"avdd_pll_utmip",	/* TODO 1.8V */
};

static const int udc_regulator_count = ARRAY_SIZE(udc_regulator_names);

static void nvudc_remove_pci(struct pci_dev *pdev);

static int nvudc_gadget_start(struct usb_gadget *,
		struct usb_gadget_driver *);
static int nvudc_gadget_stop(struct usb_gadget *,
		struct usb_gadget_driver *);

static void build_ep0_status(struct NV_UDC_EP *udc_ep_ptr, bool default_value,
		u32 status, struct NV_UDC_REQUEST *udc_req_ptr);

static int ep_halt(struct NV_UDC_EP *udc_ep_ptr, int halt);
static void setup_status_trb(struct NV_UDC_S *nvudc,
	struct TRANSFER_TRB_S *p_trb, struct usb_request *usb_req, u8 pcs);

static void nvudc_handle_setup_pkt(struct NV_UDC_S *nvudc,
		struct usb_ctrlrequest *setup_pkt, u16 seq_num);
#define NUM_EP_CX	32

#define TRB_MAX_BUFFER_SIZE		65536
#define NVUDC_CONTROL_EP_TD_RING_SIZE	1024
#define NVUDC_BULK_EP_TD_RING_SIZE	1024
#define NVUDC_ISOC_EP_TD_RING_SIZE	8
#define NVUDC_INT_EP_TD_RING_SIZE	8
#define EVENT_RING_SIZE			4096

#define IOCTL_HOST_FLAG_ENABLED		1
#define PORTSC_MASK     0xFF15FFFF
#define PORTREGSEL_MASK	0xFFFFFFFC;

static struct usb_endpoint_descriptor nvudc_ep0_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0,
	.bmAttributes = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize = cpu_to_le16(64),
};

int debug_level = LEVEL_WARNING;
module_param(debug_level, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug_level, "level 0~4");

static void usbep_struct_setup(struct NV_UDC_S *nvudc, u32 index, u8 *name);
static bool u1_u2_enable = true;
module_param(u1_u2_enable, bool, S_IRUGO|S_IWUSR);

/* T210 workaround. Disable LPM for HS and FS by default */
static bool disable_lpm = false;
module_param(disable_lpm, bool, S_IRUGO|S_IWUSR);

#ifdef PRIME_NOT_RCVD_WAR
/* With some hosts, after they reject the stream, they are not sending the
 * PRIME packet for the Device to know when to restart a stream. In that case
 * we retry the following count times before stopping and waiting for PRIME
 */
static unsigned int  max_stream_rejected_retry_count = 20;

/* When the stream is rejected by the Host, we wait for this much
 * time(in milliseconds) before we retry sending an ERDY. so in total
 * we retry for 400ms( 20 count x 20ms sleep each time) for the Host to
 * respond and then stop retrying and wait for the PRIME from Host
 */
static unsigned int stream_rejected_sleep_msecs = 20;
module_param(max_stream_rejected_retry_count, uint, S_IRUGO|S_IWUSR);
module_param(stream_rejected_sleep_msecs, uint, S_IRUGO|S_IWUSR);
#endif

#ifdef DEBUG
#define reg_dump(_dev, _base, _reg) \
	msg_dbg(_dev, "%s @%x = 0x%x\n", #_reg, _reg, ioread32(_base + _reg));
#else
#define reg_dump(_dev, _base, _reg)	do {} while (0)
#endif

struct xusb_usb2_otg_pad_config {
	u32 hs_curr_level;
	u32 hs_squelch_level;
	u32 term_range_adj;
	u32 hs_iref_cap;
	u32 hs_slew;
	u32 ls_rslew;
};

#define DEFAULT_MIN_IRQ_INTERVAL	(1000) /* 1 ms */
/* disable interrupt moderation to boost the performance on silicon bringup */
static unsigned int min_irq_interval_us;
module_param(min_irq_interval_us, uint, S_IRUGO);
MODULE_PARM_DESC(min_irq_interval_us, "minimum irq interval in microseconds");

static int nvudc_ep_disable(struct usb_ep *_ep);

static inline void xudc_set_port_power(struct NV_UDC_S *nvudc, bool on)
{
	u32 portsc;
	struct device *dev = nvudc->dev;

	if (!tegra_xhci_hcd) {
		msg_err(dev, "%s: host driver not loaded yet\n", __func__);
		return;
	}

	if (on) {
		if (tegra_xhci_hcd->driver &&
				tegra_xhci_hcd->driver->hub_control)
			tegra_xhci_hcd->driver->hub_control(tegra_xhci_hcd,
				ClearPortFeature, USB_PORT_FEAT_POWER, 1, 0, 0);

		if (tegra_xhci_hcd->driver &&
				tegra_xhci_hcd->driver->reset_sspi)
			tegra_xhci_hcd->driver->reset_sspi(tegra_xhci_hcd, 0);

		if (tegra_xhci_hcd->driver &&
				tegra_xhci_hcd->driver->hub_control)
			tegra_xhci_hcd->driver->hub_control(tegra_xhci_hcd,
				SetPortFeature, USB_PORT_FEAT_POWER, 1, 0, 0);
	} else {
		if (tegra_xhci_hcd->driver &&
				tegra_xhci_hcd->driver->hub_control)
			tegra_xhci_hcd->driver->hub_control(tegra_xhci_hcd,
				ClearPortFeature, USB_PORT_FEAT_POWER, 1, 0, 0);
	}
}

/* must hold nvudc->lock */
static inline void vbus_detected(struct NV_UDC_S *nvudc)
{
	/* set VBUS_OVERRIDE only in device mode when ID pin is floating */
	if (nvudc->vbus_detected || nvudc->id_grounded)
		return; /* nothing to do */

	tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
		USB2_VBUS_ID_0_VBUS_OVERRIDE, USB2_VBUS_ID_0_VBUS_OVERRIDE);

	nvudc->vbus_detected = true;
}

/* must hold nvudc->lock */
static inline void vbus_not_detected(struct NV_UDC_S *nvudc)
{
	if (!nvudc->vbus_detected)
		return; /* nothing to do */

	tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
		USB2_VBUS_ID_0_VBUS_OVERRIDE, 0);

	nvudc->vbus_detected = false;
}

static inline void xudc_enable_vbus(struct NV_UDC_S *nvudc)
{
	struct device *dev = nvudc->dev;
	int ret;

	if (!IS_ERR_OR_NULL(nvudc->usb_vbus0_reg)) {
		msg_info(dev, "Enabling Vbus\n");
		ret = regulator_enable(nvudc->usb_vbus0_reg);
		if (ret < 0) {
			msg_err(dev, "%s vbus0 enable failed. ret=%d\n",
				__func__, ret);
		}
	}
}

static inline void xudc_disable_vbus(struct NV_UDC_S *nvudc)
{
	struct device *dev = nvudc->dev;
	int ret;

	if (!IS_ERR_OR_NULL(nvudc->usb_vbus0_reg)) {
		msg_info(dev, "Disabling Vbus\n");
		ret = regulator_disable(nvudc->usb_vbus0_reg);
		if (ret < 0) {
			msg_err(dev, "%s vbus0 disable failed. ret = %d\n",
				__func__, ret);
		}
	}
}

static void irq_work(struct work_struct *work)
{
	struct NV_UDC_S *nvudc =
		container_of(work, struct NV_UDC_S, work);

	if (nvudc->id_grounded) {
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
			USB2_VBUS_ID_0_ID_OVERRIDE,
			USB2_VBUS_ID_0_ID_OVERRIDE_RID_GND);

		/* set PP */
		xudc_set_port_power(nvudc, true);
		xudc_enable_vbus(nvudc);
	} else {
		/* clear PP */
		xudc_disable_vbus(nvudc);
		xudc_set_port_power(nvudc, false);
		tegra_usb_pad_reg_update(XUSB_PADCTL_USB2_VBUS_ID_0,
			USB2_VBUS_ID_0_ID_OVERRIDE,
			USB2_VBUS_ID_0_ID_OVERRIDE_RID_FLOAT);
	}
}
static int extcon_id_notifications(struct notifier_block *nb,
				   unsigned long event, void *unused)
{
	struct NV_UDC_S *nvudc =
			container_of(nb, struct NV_UDC_S, id_extcon_nb);
	struct device *dev = nvudc->dev;
	unsigned long flags;

	spin_lock_irqsave(&nvudc->lock, flags);

	if (!nvudc->pullup) {
		msg_info(dev, "%s: gadget is not ready yet\n", __func__);
		goto done;
	}

	if (extcon_get_cable_state(nvudc->id_extcon_dev, "USB-Host")
			&& !nvudc->id_grounded) {
		msg_info(dev, "%s: USB_ID pin grounded\n", __func__);
		nvudc->id_grounded = true;
	} else if (nvudc->id_grounded) {
		msg_info(dev, "%s: USB_ID pin floating\n", __func__);
		nvudc->id_grounded = false;
	} else {
		msg_info(dev, "%s: USB_ID pin status unchanged\n", __func__);
		goto done;
	}

	schedule_work(&nvudc->work);

done:
	spin_unlock_irqrestore(&nvudc->lock, flags);
	return NOTIFY_DONE;
}

static int extcon_notifications(struct notifier_block *nb,
				   unsigned long event, void *unused)
{
	struct NV_UDC_S *nvudc =
			container_of(nb, struct NV_UDC_S, vbus_extcon_nb);
	struct device *dev = nvudc->dev;
	unsigned long flags;

	spin_lock_irqsave(&nvudc->lock, flags);

	if (!nvudc->pullup) {
		msg_info(dev, "%s: gadget is not ready yet\n", __func__);
		goto exit;
	}

	if (extcon_get_cable_state(nvudc->vbus_extcon_dev, "USB")) {
		msg_info(dev, "%s: vbus on detected\n", __func__);
		vbus_detected(nvudc);
	} else {
		msg_info(dev, "%s: vbus off detected\n", __func__);
		vbus_not_detected(nvudc);
	}

exit:
	spin_unlock_irqrestore(&nvudc->lock, flags);
	return NOTIFY_DONE;
}

static void set_interrupt_moderation(struct NV_UDC_S *nvudc, unsigned us)
{
	u32 reg;
	u32 imodi = us * 4; /* 1 us = 4 x 250 ns */

	if (imodi > 0xFFFF) {
		msg_warn(nvudc->dev, "invalid interrupt interval %u us\n", us);
		imodi = DEFAULT_MIN_IRQ_INTERVAL * 4;
	}

	reg = ioread32(nvudc->mmio_reg_base + RT_IMOD);
	reg &= ~RT_IMOD_IMODI(-1);
	reg |= RT_IMOD_IMODI(imodi);
	/* HW won't overwrite IMODC. Set IMODC to the new value also */
	reg &= ~RT_IMOD_IMODC(-1);
	reg |= RT_IMOD_IMODC(imodi);
	iowrite32(reg, nvudc->mmio_reg_base + RT_IMOD);
}

static void setup_link_trb(struct TRANSFER_TRB_S *link_trb,
					bool toggle, dma_addr_t next_trb)
{
	u32 dw = 0;

	link_trb->data_buf_ptr_lo = cpu_to_le32(lower_32_bits(next_trb));
	link_trb->data_buf_ptr_hi = cpu_to_le32(upper_32_bits(next_trb));

	link_trb->trb_dword2 = 0;

	XHCI_SETF_VAR(TRB_TYPE, dw, TRB_TYPE_LINK);
	if (toggle)
		XHCI_SETF_VAR(TRB_LINK_TOGGLE_CYCLE, dw, 1);
	else
		XHCI_SETF_VAR(TRB_LINK_TOGGLE_CYCLE, dw, 0);

	link_trb->trb_dword3 = cpu_to_le32(dw);
}

static dma_addr_t tran_trb_virt_to_dma(struct NV_UDC_EP *udc_ep,
	struct TRANSFER_TRB_S *trb)
{
	unsigned long offset;
	int trb_idx;
	dma_addr_t dma_addr = 0;

	trb_idx = trb - udc_ep->tran_ring_ptr;
	if (unlikely(trb_idx < 0))
		return 0;

	offset = trb_idx * sizeof(*trb);
	if (unlikely(offset > udc_ep->tran_ring_info.len))
		return 0;
	dma_addr = udc_ep->tran_ring_info.dma + offset;
	return dma_addr;
}

static struct TRANSFER_TRB_S *tran_trb_dma_to_virt(struct NV_UDC_EP *udc_ep,
	u64 address)
{
	unsigned long offset;
	struct TRANSFER_TRB_S *trb_virt;
	struct NV_UDC_S *nvudc = udc_ep->nvudc;

	if (lower_32_bits(address) & 0xf) {
		msg_dbg(nvudc->dev, "transfer ring dma address incorrect\n");
		return NULL;
	}

	offset = address - udc_ep->tran_ring_info.dma;
	if (unlikely(offset > udc_ep->tran_ring_info.len))
		return NULL;
	offset = offset / sizeof(struct TRANSFER_TRB_S);
	trb_virt = udc_ep->tran_ring_ptr + offset;
	return trb_virt;
}

static dma_addr_t event_trb_virt_to_dma(struct NV_UDC_S *nvudc,
		struct EVENT_TRB_S *event)
{
	dma_addr_t dma_addr = 0;
	unsigned long seg_offset;
	struct EVENT_TRB_S *evt_seg0_first_trb;
	struct EVENT_TRB_S *evt_seg1_first_trb;

	if (!nvudc || !event)
		return 0;

	evt_seg0_first_trb = (struct EVENT_TRB_S *)nvudc->event_ring0.vaddr;
	evt_seg1_first_trb = (struct EVENT_TRB_S *)nvudc->event_ring1.vaddr;

	if ((event >= evt_seg0_first_trb) &&
		(event <= nvudc->evt_seg0_last_trb)) {
		seg_offset = event - evt_seg0_first_trb;
		dma_addr = nvudc->event_ring0.dma + seg_offset * sizeof(*event);
	} else if ((event >= evt_seg1_first_trb)
			&& (event <= nvudc->evt_seg1_last_trb)) {
		seg_offset = event - evt_seg1_first_trb;
		dma_addr = nvudc->event_ring1.dma + seg_offset * sizeof(*event);
	}

	return dma_addr;
}

static void nvudc_epcx_setup(struct NV_UDC_EP *udc_ep)
{
	struct NV_UDC_S *nvudc = udc_ep->nvudc;
	struct device *dev = nvudc->dev;
	const struct usb_endpoint_descriptor *desc = udc_ep->desc;
	const struct usb_ss_ep_comp_descriptor *comp_desc = udc_ep->comp_desc;
	u8 DCI = udc_ep->DCI;
	struct EP_CX_S *epcx = (struct EP_CX_S *)nvudc->p_epcx + DCI;
	enum EP_TYPE_E ep_type;
	u16 esit_payload = 0;
	u16 maxburst = 0;
	u8 maxstreams = 0;
	u8 mult = 0;
	u16 maxsize;
	u32 dw;

	msg_dbg(dev, "nvudc->p_epcx %p, epcx %p\n", nvudc->p_epcx, epcx);
	msg_dbg(dev, "DCI %d, sizeof ep_cx %d\n", DCI, sizeof(struct EP_CX_S));

	if (usb_endpoint_dir_out(desc))
		ep_type = usb_endpoint_type(desc);
	else
		ep_type = usb_endpoint_type(desc) + EP_TYPE_CNTRL;

	maxsize = usb_endpoint_maxp(desc) & 0x07ff; /* D[0:10] */

	if (nvudc->gadget.speed == USB_SPEED_SUPER) {
		maxburst = comp_desc->bMaxBurst;

		if (usb_endpoint_xfer_bulk(udc_ep->desc))
			maxstreams = comp_desc->bmAttributes & 0x1f;
		else if (usb_endpoint_xfer_isoc(udc_ep->desc))
			mult = comp_desc->bmAttributes & 0x3;

		if (usb_endpoint_xfer_int(udc_ep->desc) ||
					usb_endpoint_xfer_isoc(udc_ep->desc)) {
			esit_payload =
				le16_to_cpu(comp_desc->wBytesPerInterval);
		}
	} else if ((nvudc->gadget.speed == USB_SPEED_HIGH ||
		nvudc->gadget.speed == USB_SPEED_FULL) &&
			(usb_endpoint_xfer_int(udc_ep->desc) ||
				usb_endpoint_xfer_isoc(udc_ep->desc))) {
		if (nvudc->gadget.speed == USB_SPEED_HIGH)
			maxburst = (usb_endpoint_maxp(desc) >> 11) & 0x3;
		if (maxburst == 0x3) {
			msg_warn(dev, "invalid maxburst\n");
			maxburst = 0x2; /* really need ? */
		}
		esit_payload = (maxsize * (maxburst + 1));
	}

	/* fill ep_dw0 */
	dw = 0;
	XHCI_SETF_VAR(EP_CX_EP_STATE, dw, EP_STATE_RUNNING);
	XHCI_SETF_VAR(EP_CX_INTERVAL, dw, desc->bInterval);
	XHCI_SETF_VAR(EP_CX_MULT, dw, mult);
	if (maxstreams) {
		XHCI_SETF_VAR(EP_CX_MAX_PSTREAMS, dw, maxstreams);
		XHCI_SETF_VAR(EP_LINEAR_STRM_ARR, dw, 1);
	}
	epcx->ep_dw0 = cpu_to_le32(dw);

	/* fill ep_dw1 */
	dw = 0;
	XHCI_SETF_VAR(EP_CX_EP_TYPE, dw, ep_type);
	XHCI_SETF_VAR(EP_CX_CERR, dw, 3);
	XHCI_SETF_VAR(EP_CX_MAX_PACKET_SIZE, dw, maxsize);
	XHCI_SETF_VAR(EP_CX_MAX_BURST_SIZE, dw, maxburst);
	epcx->ep_dw1 = cpu_to_le32(dw);

	/* fill ep_dw2 */
	dw = lower_32_bits(udc_ep->tran_ring_info.dma) & EP_CX_TR_DQPT_LO_MASK;
	XHCI_SETF_VAR(EP_CX_DEQ_CYC_STATE, dw, udc_ep->pcs);
	epcx->ep_dw2 = cpu_to_le32(dw);

	/* fill ep_dw3 */
	dw = upper_32_bits(udc_ep->tran_ring_info.dma);
	epcx->ep_dw3 = cpu_to_le32(dw);

	/* fill ep_dw4 */
	dw = 0;
	/* Reasonable initial values of Average TRB Length for
	 * Control endpoints would be 8B,
	 * Interrupt endpoints would be 1KB,
	 * Bulk and Isoch endpoints would be 3KB.
	 * */
	XHCI_SETF_VAR(EP_CX_AVE_TRB_LEN, dw, 3000);
	XHCI_SETF_VAR(EP_CX_MAX_ESIT_PAYLOAD, dw, esit_payload);
	epcx->ep_dw4 = cpu_to_le32(dw);

	/* fill ep_dw5 */
	epcx->ep_dw5 = 0;

	/* fill ep_dw6 */
	dw = 0;
	XHCI_SETF_VAR(EP_CX_CERRCNT, dw, 3);
	epcx->ep_dw6 = cpu_to_le32(dw);
}

static int nvudc_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct NV_UDC_EP *udc_ep;
	struct NV_UDC_S *nvudc;
	struct device *dev;
	bool is_isoc_ep = false;
	u32 u_temp = 0;
	unsigned long flags;
	struct EP_CX_S *p_ep_cx;

	if  (!ep || !desc || (desc->bDescriptorType != USB_DT_ENDPOINT))
		return -EINVAL;

	udc_ep = container_of(ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep->nvudc;

	if (!nvudc->driver)
		return -ESHUTDOWN;

	p_ep_cx = (struct EP_CX_S *) nvudc->p_epcx + udc_ep->DCI;
	if (XHCI_GETF(EP_CX_EP_STATE, p_ep_cx->ep_dw0) != EP_STATE_DISABLED)
		nvudc_ep_disable(ep);

	udc_ep->desc = desc;
	udc_ep->comp_desc = ep->comp_desc;
	dev = nvudc->dev;
	msg_entry(dev);

	/* setup endpoint context for regular endpoint
	   the endpoint context for control endpoint has been
	   setted up in probe function */
	if (udc_ep->DCI) {
		msg_dbg(dev, "ep_enable udc_ep->DCI = %d\n", udc_ep->DCI);
		if (usb_endpoint_xfer_isoc(desc)) {
			nvudc->g_isoc_eps++;
			if (nvudc->g_isoc_eps > 4) {
				msg_err(dev, "too many isoc eps %d\n",
						nvudc->g_isoc_eps);
				nvudc->g_isoc_eps--;
				return -EBUSY;
			}
			is_isoc_ep = true;
		}

		/* setup transfer ring */
		if (!udc_ep->tran_ring_info.vaddr) {
			dma_addr_t dma;
			u32 ring_size = 0;
			void *vaddr;
			size_t len;

			if (usb_endpoint_xfer_bulk(desc))
				ring_size =  NVUDC_BULK_EP_TD_RING_SIZE;
			else if (usb_endpoint_xfer_isoc(desc))
				ring_size = NVUDC_ISOC_EP_TD_RING_SIZE;
			else if (usb_endpoint_xfer_int(desc))
				ring_size = NVUDC_INT_EP_TD_RING_SIZE;

			len = ring_size * sizeof(struct TRANSFER_TRB_S);
			vaddr = dma_alloc_coherent(nvudc->dev, len,
					&dma, GFP_ATOMIC);
			if (!vaddr) {
				msg_err(dev, "failed to allocate trb ring\n");
				return -ENOMEM;
			}

			udc_ep->tran_ring_info.vaddr = vaddr;
			udc_ep->tran_ring_info.dma = dma;
			udc_ep->tran_ring_info.len = len;
			udc_ep->first_trb = vaddr;
			udc_ep->link_trb = udc_ep->first_trb + ring_size - 1;
		}
		memset(udc_ep->first_trb, 0, udc_ep->tran_ring_info.len);

		setup_link_trb(udc_ep->link_trb, true,
					udc_ep->tran_ring_info.dma);

		if (usb_endpoint_xfer_bulk(desc))
			nvudc->act_bulk_ep |= BIT(udc_ep->DCI);

		udc_ep->enq_pt = udc_ep->first_trb;
		udc_ep->pcs = 1;
		udc_ep->tran_ring_full = false;
		nvudc->num_enabled_eps++;
		nvudc_epcx_setup(udc_ep);
	}

	msg_dbg(dev, "num_enabled_eps = %d\n", nvudc->num_enabled_eps);
	spin_lock_irqsave(&nvudc->lock, flags);
	if (nvudc->device_state == USB_STATE_ADDRESS) {
		u32 reg;
		reg = ioread32(nvudc->mmio_reg_base + CTRL);
		reg |= CTRL_RUN;
		iowrite32(reg, nvudc->mmio_reg_base + CTRL);

		msg_dbg(dev, "set run bit\n");

		nvudc->device_state = USB_STATE_CONFIGURED;
	}

	if (is_isoc_ep) {
		u32 u_temp;
		msg_dbg(dev, "is_isoc_ep is TRUE\n");

		/* pause all the active bulk endpoint */
		u_temp = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
		u_temp |= nvudc->act_bulk_ep;

		iowrite32(u_temp, nvudc->mmio_reg_base + EP_PAUSE);

		poll_stchg(dev, "Pausing bulk eps", nvudc->act_bulk_ep);

		iowrite32(nvudc->act_bulk_ep, nvudc->mmio_reg_base + EP_STCHG);

		u_temp &= ~nvudc->act_bulk_ep;
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_PAUSE);
		poll_stchg(dev, "Pausing bulk eps", u_temp);

		iowrite32(u_temp, nvudc->mmio_reg_base + EP_STCHG);
	}

	iowrite32(NV_BIT(udc_ep->DCI), nvudc->mmio_reg_base + EP_RELOAD);
	msg_dbg(dev, "load endpoint context = 0x%x\n",
			ioread32(nvudc->mmio_reg_base + EP_RELOAD));

	poll_reload(dev, "ep_enable()", NV_BIT(udc_ep->DCI));

	/* unpause ep */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
	u_temp &= ~NV_BIT(udc_ep->DCI);
	iowrite32(u_temp, nvudc->mmio_reg_base + EP_PAUSE);

	poll_stchg(dev, "EP PAUSED", NV_BIT(udc_ep->DCI));

	iowrite32(NV_BIT(udc_ep->DCI), nvudc->mmio_reg_base + EP_STCHG);
	msg_dbg(dev, "Clear Pause bits set by HW when reload context=0x%x\n",
		ioread32(nvudc->mmio_reg_base + EP_PAUSE));

	/* if EP was halted, clear the halt bit */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_HALT);
	if (u_temp & NV_BIT(udc_ep->DCI)) {
		msg_dbg(dev, "EP was halted u_temp = 0x%x\n", u_temp);
		u_temp &= ~NV_BIT(udc_ep->DCI);
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_HALT);

		poll_stchg(nvudc->dev, "EP halted", NV_BIT(udc_ep->DCI));
		iowrite32(NV_BIT(udc_ep->DCI), nvudc->mmio_reg_base + EP_STCHG);

		msg_dbg(dev, "clear state change register\n");
	}

	spin_unlock_irqrestore(&nvudc->lock, flags);
	msg_exit(dev);
	return 0;
}

/* Completes request.  Calls gadget completion handler
 * caller must have acquired spin lock.
 */
static void req_done(struct NV_UDC_EP *udc_ep,
			struct NV_UDC_REQUEST *udc_req, int status)
{
	struct NV_UDC_S *nvudc = udc_ep->nvudc;

	if (likely(udc_req->usb_req.status == -EINPROGRESS))
		udc_req->usb_req.status = status;

	list_del_init(&udc_req->queue);
	if (udc_req->mapped) {
		dma_unmap_single(nvudc->dev,
				udc_req->usb_req.dma, udc_req->usb_req.length,
				(udc_ep->desc->bEndpointAddress & USB_DIR_IN)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
#define DMA_ADDR_INVALID    (~(dma_addr_t)0)
		udc_req->usb_req.dma = DMA_ADDR_INVALID;
		udc_req->mapped = 0;
	}

	if (udc_req->usb_req.complete) {
		spin_unlock(&nvudc->lock);
		udc_req->usb_req.complete(&udc_ep->usb_ep, &udc_req->usb_req);
		spin_lock(&nvudc->lock);
	}
	msg_exit(nvudc->dev);
}

static void nuke(struct NV_UDC_EP *udc_ep, int status)
{
	struct NV_UDC_REQUEST *req = NULL;
	while (!list_empty(&udc_ep->queue)) {

		req = list_entry(udc_ep->queue.next,
				struct NV_UDC_REQUEST,
				queue);

		req_done(udc_ep, req, status);
	}

}

static int nvudc_ep_disable(struct usb_ep *_ep)
{
	struct NV_UDC_EP *udc_ep;
	struct NV_UDC_S *nvudc;
	struct EP_CX_S *p_ep_cx;
	unsigned long flags;
	u32 u_temp, u_temp2;
	uint	ep_state;

	if (!_ep)
		return -EINVAL;

	udc_ep = container_of(_ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep->nvudc;
	msg_entry(nvudc->dev);

	p_ep_cx = (struct EP_CX_S *)nvudc->p_epcx + udc_ep->DCI;

	ep_state = XHCI_GETF(EP_CX_EP_STATE, p_ep_cx->ep_dw0);
	if (!ep_state) {
		/* get here if ep is already disabled */
		return -EINVAL;
	}

	spin_lock_irqsave(&nvudc->lock, flags);
	msg_dbg(nvudc->dev, "EPDCI = 0x%x\n", udc_ep->DCI);

	/* HW will pause the endpoint first while reload ep,
	 * do not need to write Pause register */
	XHCI_SETF_VAR(EP_CX_EP_STATE, p_ep_cx->ep_dw0, EP_STATE_DISABLED);

	msg_dbg(nvudc->dev, "reload eps\n");
	iowrite32(NV_BIT(udc_ep->DCI), nvudc->mmio_reg_base + EP_RELOAD);

	poll_reload(nvudc->dev, "ep_disable()", NV_BIT(udc_ep->DCI));

	/* clean up the request queue */
	nuke(udc_ep, -ESHUTDOWN);

	/* decrement ep counters */
	nvudc->num_enabled_eps--;
	if (usb_endpoint_xfer_isoc(udc_ep->desc))
		nvudc->g_isoc_eps--;

	/* release all the memory allocate for the endpoint */
	/* dma_free_coherent(nvudc->dev, nvudc->event_ring0.buff_len,
	 * nvudc->event_ring0.virt_addr, nvudc->event_ring0.dma_addr); */

	udc_ep->desc = NULL;

	/* clean up the endpoint context */
	memset(p_ep_cx, 0, sizeof(struct EP_CX_S));

	/* clean up the hw registers which used to track ep states*/
	/* EP_PAUSE register */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
	u_temp2 = 1 << udc_ep->DCI;
	if (u_temp & u_temp2) {
		u_temp &= ~u_temp2;
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_PAUSE);
		poll_stchg(nvudc->dev, "nvudc_ep_disable", u_temp2);
		iowrite32(u_temp2, nvudc->mmio_reg_base + EP_STCHG);
		msg_dbg(nvudc->dev, "clear EP_PAUSE register\n");
	}

	/* EP_HALT register */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_HALT);
	u_temp2 = 1 << udc_ep->DCI;
	if (u_temp & u_temp2) {
		u_temp &= ~u_temp2;
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_HALT);
		poll_stchg(nvudc->dev, "nvudc_ep_disable", u_temp2);
		iowrite32(u_temp2, nvudc->mmio_reg_base + EP_STCHG);
		msg_dbg(nvudc->dev, "clear EP_HALT register\n");
	}

	/* EP_STOPPED register */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_STOPPED);
	u_temp2 = 1 << udc_ep->DCI;
	if (u_temp & u_temp2) {
		iowrite32(u_temp2, nvudc->mmio_reg_base + EP_STOPPED);
		msg_dbg(nvudc->dev, "clear EP_STOPPED register\n");
	}

	msg_dbg(nvudc->dev, "num_enabled_eps = %d\n", nvudc->num_enabled_eps);

	/* If device state was changed to default by port
	reset, should not overwrite it again */
	if ((nvudc->num_enabled_eps == 0) &&
		(nvudc->device_state == USB_STATE_CONFIGURED)) {
		msg_dbg(nvudc->dev, "Device State USB_STATE_CONFIGURED\n");
		msg_dbg(nvudc->dev, "Set Device State to addressed\n");
		nvudc->device_state = USB_STATE_ADDRESS;
		u_temp = ioread32(nvudc->mmio_reg_base + CTRL);
		u_temp &= ~CTRL_RUN;
		iowrite32(u_temp, nvudc->mmio_reg_base + CTRL);

		msg_dbg(nvudc->dev, "clear RUN bit = 0x%x\n", u_temp);

		/*        nvudc_power_gate(); */
	}
	spin_unlock_irqrestore(&nvudc->lock, flags);
	msg_exit(nvudc->dev);
	return 0;
}

static struct usb_request *
nvudc_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct NV_UDC_EP *udc_ep;
	struct NV_UDC_S *nvudc;
	struct NV_UDC_REQUEST *udc_req_ptr = NULL;

	udc_ep = container_of(_ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep->nvudc;
	msg_entry(nvudc->dev);

	udc_req_ptr = kzalloc(sizeof(struct NV_UDC_REQUEST), gfp_flags);

	msg_dbg(nvudc->dev, "udc_req_ptr = 0x%p\n", udc_req_ptr);

	if (!udc_req_ptr)
		return NULL;

	udc_req_ptr->usb_req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&udc_req_ptr->queue);

	msg_exit(nvudc->dev);
	return &udc_req_ptr->usb_req;
}

static void nvudc_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct NV_UDC_EP *udc_ep;
	struct NV_UDC_S *nvudc;
	struct NV_UDC_REQUEST *udc_req_ptr = NULL;

	udc_ep = container_of(_ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep->nvudc;
	msg_entry(nvudc->dev);

	if (!_ep || !_req)
		return;

	udc_req_ptr = container_of(_req, struct NV_UDC_REQUEST, usb_req);
	kfree(udc_req_ptr);
	msg_exit(nvudc->dev);
}

/* num_trbs here is the size of the ring. */
u32 room_on_ring(struct NV_UDC_S *nvudc, u32 num_trbs,
		struct TRANSFER_TRB_S *p_ring, struct TRANSFER_TRB_S *enq_pt,
		struct TRANSFER_TRB_S *dq_pt)
{
	u32 i = 0;

	msg_dbg(nvudc->dev, "room_on_ring enq_pt = 0x%p, dq_pt = 0x%p",
			enq_pt, dq_pt);
	if (enq_pt == dq_pt) {
		/* ring is empty */
		return num_trbs - 1;
	}

	while (enq_pt != dq_pt) {
		i++;

		enq_pt++;

		if (XHCI_GETF(TRB_TYPE, enq_pt->trb_dword3) == TRB_TYPE_LINK)
			enq_pt = p_ring;

		if (i > num_trbs)
			break;
	}

	msg_dbg(nvudc->dev, "room_on_ring 0x%x\n", i);
	return i-1;
}

void setup_status_trb(struct NV_UDC_S *nvudc, struct TRANSFER_TRB_S *p_trb,
		struct usb_request *usb_req, u8 pcs)
{

	u32 u_temp, dir = 0;

	/* There are some cases where seutp_status_trb() is called with
	 * usb_req set to NULL.*/
	if (usb_req != NULL) {
		p_trb->data_buf_ptr_lo = lower_32_bits(usb_req->dma);
		p_trb->data_buf_ptr_hi = upper_32_bits(usb_req->dma);
	}

	msg_dbg(nvudc->dev, "data_buf_ptr_lo = 0x%x, data_buf_ptr_hi = 0x%x\n",
		p_trb->data_buf_ptr_lo, p_trb->data_buf_ptr_hi);

	p_trb->trb_dword2 = 0;

	u_temp = 0;
	XHCI_SETF_VAR(TRB_CYCLE_BIT, u_temp, pcs);
	XHCI_SETF_VAR(TRB_INTR_ON_COMPLETION, u_temp, 1);
	XHCI_SETF_VAR(TRB_TYPE, u_temp, TRB_TYPE_XFER_STATUS_STAGE);

	if (nvudc->setup_status == STATUS_STAGE_XFER)
		dir = 1;

	XHCI_SETF_VAR(DATA_STAGE_TRB_DIR, u_temp, dir);

	p_trb->trb_dword3 = u_temp;
	msg_dbg(nvudc->dev, "trb_dword2 = 0x%x, trb_dword3 = 0x%x\n",
			p_trb->trb_dword2, p_trb->trb_dword3);

}

void setup_datastage_trb(struct NV_UDC_S *nvudc, struct TRANSFER_TRB_S *p_trb,
		struct usb_request *usb_req, u8 pcs, u32 num_trb,
		u32 transfer_length, u32 td_size, u8 IOC, u8 dir)
{

	u32 u_temp;
	p_trb->data_buf_ptr_lo = lower_32_bits(usb_req->dma);
	p_trb->data_buf_ptr_hi = upper_32_bits(usb_req->dma);

	msg_dbg(nvudc->dev, "data_buf_ptr_lo = 0x%x, data_buf_ptr_hi = 0x%x\n",
		p_trb->data_buf_ptr_lo, p_trb->data_buf_ptr_hi);

	/* TRB_Transfer_Length
	For USB_DIR_OUT, this field is the number of data bytes expected from
	xhc. For USB_DIR_IN, this field is the number of data bytes the device
	will send. */
	u_temp = 0;
	XHCI_SETF_VAR(TRB_TRANSFER_LEN, u_temp, transfer_length);
	XHCI_SETF_VAR(TRB_TD_SIZE, u_temp, td_size);
	p_trb->trb_dword2 = u_temp;

	u_temp = 0;
	XHCI_SETF_VAR(TRB_CYCLE_BIT, u_temp, pcs);
	XHCI_SETF_VAR(TRB_INTR_ON_SHORT_PKT, u_temp, 1);
	XHCI_SETF_VAR(TRB_INTR_ON_COMPLETION, u_temp, IOC);
	XHCI_SETF_VAR(TRB_TYPE, u_temp, TRB_TYPE_XFER_DATA_STAGE);
	XHCI_SETF_VAR(DATA_STAGE_TRB_DIR, u_temp, dir);
	p_trb->trb_dword3 = u_temp;

	msg_dbg(nvudc->dev, "trb_dword2 = 0x%x, trb_dword3 = 0x%x\n",
			p_trb->trb_dword2, p_trb->trb_dword3);

}


void setup_trb(struct NV_UDC_S *nvudc, struct TRANSFER_TRB_S *p_trb,
		struct usb_request *usb_req, u32 xfer_len,
		dma_addr_t xfer_buf_addr, u8 td_size, u8 pcs,
		u8 trb_type, u8 short_pkt, u8 chain_bit,
		u8 intr_on_compl, bool b_setup_stage, u8 usb_dir,
		bool b_isoc, u8 tlb_pc, u16 frame_i_d, u8 SIA)
{
	u32 u_temp;
	p_trb->data_buf_ptr_lo = lower_32_bits(xfer_buf_addr);
	p_trb->data_buf_ptr_hi = upper_32_bits(xfer_buf_addr);

	msg_dbg(nvudc->dev, "data_buf_ptr_lo = 0x%x, data_buf_ptr_hi = 0x%x\n",
		p_trb->data_buf_ptr_lo, p_trb->data_buf_ptr_hi);

	u_temp = 0;
	XHCI_SETF_VAR(TRB_TRANSFER_LEN, u_temp, xfer_len);
	XHCI_SETF_VAR(TRB_TD_SIZE, u_temp, td_size);

	p_trb->trb_dword2 = u_temp;

	u_temp = 0;

	XHCI_SETF_VAR(TRB_CYCLE_BIT, u_temp, pcs);
	XHCI_SETF_VAR(TRB_INTR_ON_SHORT_PKT, u_temp, short_pkt);
	XHCI_SETF_VAR(TRB_CHAIN_BIT, u_temp, chain_bit);
	XHCI_SETF_VAR(TRB_INTR_ON_COMPLETION, u_temp, intr_on_compl);
	XHCI_SETF_VAR(TRB_TYPE, u_temp, trb_type);

	if (b_setup_stage)
		XHCI_SETF_VAR(DATA_STAGE_TRB_DIR, u_temp, usb_dir);

	if (usb_req->stream_id)
		XHCI_SETF_VAR(STREAM_TRB_STREAM_ID, u_temp, usb_req->stream_id);

	if (b_isoc) {
		XHCI_SETF_VAR(ISOC_TRB_TLBPC, u_temp, tlb_pc);
		XHCI_SETF_VAR(ISOC_TRB_FRAME_ID, u_temp, frame_i_d);
		XHCI_SETF_VAR(ISOC_TRB_SIA, u_temp, SIA);
	}

	p_trb->trb_dword3 = u_temp;
	msg_dbg(nvudc->dev, "trb_dword2 = 0x%.8x, trb_dword3 = 0x%.8x\n",
		p_trb->trb_dword2, p_trb->trb_dword3);
}

/*
 * TD and zlp for a transfer share the same usb_request. IOC is only set
 * once between them. The ways to set IOC for TD and zlp are different
 * between control and bulk/interrupt endpoint:
 * 1. Control endpoint: IOC of data stage TD is clear. IOC of zlp is set.
 *    Status stage TD is scheduled during IOC interrupt of zlp. This
 *    guarantees status stage TD scheduled after both of data stage
 *    TD and zlp.
 * 2. Bulk/interrupt endpoint: IOC of TD is set. IOC of zlp is clear.
 *    Actual transfer length is gotten and usb_request gets completed
 *    during IOC interrupt of TD. The completion of zlp is ignored.
 */
static void nvudc_queue_zlp_td(struct NV_UDC_S *nvudc, struct NV_UDC_EP *udc_ep)
{
	struct TRANSFER_TRB_S *enq_pt = udc_ep->enq_pt;
	u32 dw = 0;

	enq_pt->data_buf_ptr_lo = 0;
	enq_pt->data_buf_ptr_hi = 0;
	enq_pt->trb_dword2 = 0;

	XHCI_SETF_VAR(TRB_CYCLE_BIT, dw, udc_ep->pcs);
	if (usb_endpoint_xfer_control(udc_ep->desc)) {
		XHCI_SETF_VAR(TRB_TYPE, dw, TRB_TYPE_XFER_DATA_STAGE);
		XHCI_SETF_VAR(DATA_STAGE_TRB_DIR, dw, 1);
		XHCI_SETF_VAR(TRB_INTR_ON_COMPLETION, dw, 1);
	} else {
		XHCI_SETF_VAR(TRB_TYPE, dw, TRB_TYPE_XFER_NORMAL);
	}
	enq_pt->trb_dword3 = dw;
	msg_dbg(nvudc->dev, "trb_dword2 = 0x%x, trb_dword3 = 0x%x\n",
			enq_pt->trb_dword2, enq_pt->trb_dword3);

	enq_pt++;
	if (XHCI_GETF(TRB_TYPE, enq_pt->trb_dword3) ==
			TRB_TYPE_LINK) {
		if (XHCI_GETF(TRB_LINK_TOGGLE_CYCLE,
				enq_pt->trb_dword3)) {
			XHCI_SETF_VAR(TRB_CYCLE_BIT,
				enq_pt->trb_dword3,
				udc_ep->pcs);
			udc_ep->pcs ^= 0x1;
		}
		enq_pt = udc_ep->tran_ring_ptr;
	}
	udc_ep->enq_pt = enq_pt;
}

int nvudc_queue_trbs(struct NV_UDC_EP *udc_ep_ptr,
		struct NV_UDC_REQUEST *udc_req_ptr,  bool b_isoc,
		u32 xfer_ring_size,
		u32 num_trbs_needed, u64 buffer_length)
{
	struct NV_UDC_S *nvudc = udc_ep_ptr->nvudc;
	struct TRANSFER_TRB_S *p_xfer_ring = udc_ep_ptr->tran_ring_ptr;
	u32 num_trbs_ava = 0;
	struct usb_request *usb_req = &udc_req_ptr->usb_req;
	struct EP_CX_S *p_ep_cx = nvudc->p_epcx + udc_ep_ptr->DCI;
	u32 deq_pt_lo = p_ep_cx->ep_dw2 & EP_CX_TR_DQPT_LO_MASK;
	u32 deq_pt_hi = p_ep_cx->ep_dw3;
	u64 dq_pt_addr = (u64)deq_pt_lo + ((u64)deq_pt_hi << 32);
	struct TRANSFER_TRB_S *dq_pt;
	u64 buff_len_temp = 0;
	u32 i, j = 1;
	struct TRANSFER_TRB_S *enq_pt = udc_ep_ptr->enq_pt;
	u8 td_size;
	u8 chain_bit = 1;
	u8 short_pkt = 0;
	u8 intr_on_compl = 0;
	u32 count;
	bool full_td = true;
	u32 intr_rate;
	dma_addr_t trb_buf_addr;
	bool need_zlp = false;

	msg_dbg(nvudc->dev, "nvudc_queue_trbs\n");
	msg_dbg(nvudc->dev, "dq_pt_addr = 0x%x\n", (unsigned
				int)dq_pt_addr);
	dq_pt = tran_trb_dma_to_virt(udc_ep_ptr, dq_pt_addr);

	if (!b_isoc) {
		if (udc_req_ptr->usb_req.zero == 1 &&
			udc_req_ptr->usb_req.length != 0 &&
			((udc_req_ptr->usb_req.length %
			  udc_ep_ptr->usb_ep.maxpacket) == 0)) {
			need_zlp = true;
		}
	}

	td_size = num_trbs_needed;

	num_trbs_ava = room_on_ring(nvudc, xfer_ring_size, p_xfer_ring,
			udc_ep_ptr->enq_pt, (struct TRANSFER_TRB_S *)dq_pt);


	/* trb_buf_addr points to the addr of the buffer that we write in
	 * each TRB. If this function is called to complete the pending TRB
	 * transfers of a previous request, point it to the buffer that is
	 * not transferred, or else point it to the starting address of the
	 * buffer received in usb_request
	 */
	if (udc_req_ptr->trbs_needed) {
		/* Here udc_req_ptr->trbs_needed is used to indicate if we
		 * are completing a previous req
		 */
		trb_buf_addr = usb_req->dma +
			(usb_req->length - udc_req_ptr->buff_len_left);
	} else {
		trb_buf_addr = usb_req->dma;
	}


	if (num_trbs_ava >= num_trbs_needed + (need_zlp ? 1 : 0))
		count = num_trbs_needed;
	else {
		if (b_isoc) {
			struct NV_UDC_REQUEST *udc_req_ptr_temp;
			u8 temp = 0;
			list_for_each_entry(udc_req_ptr_temp,
					&udc_ep_ptr->queue, queue) {
				temp++;
			}

			if (temp >= 2) {
				/*  we already scheduled two mfi in advance. */
				return 0;
			}
		}

		count = num_trbs_ava;
		full_td = false;
		msg_dbg(nvudc->dev, "TRB Ring Full. Avail: 0x%x Req: 0x%x\n",
						num_trbs_ava, num_trbs_needed);
		udc_ep_ptr->tran_ring_full = true;
	}
	msg_dbg(nvudc->dev, "queue_trbs count = 0x%x\n", count);
	for (i = 0; i < count; i++) {
		if (buffer_length > TRB_MAX_BUFFER_SIZE)
			buff_len_temp = TRB_MAX_BUFFER_SIZE;
		else
			buff_len_temp = buffer_length;

		buffer_length -= buff_len_temp;

		if (usb_endpoint_dir_out(udc_ep_ptr->desc))
			short_pkt = 1;

		if (buffer_length == 0) {
			chain_bit = 0;
			intr_on_compl = 1;
			udc_req_ptr->all_trbs_queued = 1;
		}

#define BULK_EP_INTERRUPT_RATE      5
#define ISOC_EP_INTERRUPT_RATE      1
		if  (b_isoc)
			intr_rate = ISOC_EP_INTERRUPT_RATE;
		else
			intr_rate = BULK_EP_INTERRUPT_RATE;

		if  ((!full_td) && (j == intr_rate)) {
			intr_on_compl = 1;
			j = 0;
		}

		if (b_isoc) {
			setup_trb(nvudc, enq_pt, usb_req, buff_len_temp,
				trb_buf_addr, td_size-1, udc_ep_ptr->pcs,
				TRB_TYPE_XFER_DATA_ISOCH, short_pkt, chain_bit,
				intr_on_compl, 0, 0, 1, 0, 0, 1);
		} else {
			u8 pcs = udc_ep_ptr->pcs;
			/*
			   if ((!udc_ep_ptr->csb) &&
			   (usb_endpoint_dir_out(udc_ep_ptr->desc)))
			   msg_dbg(nvudc->dev, "regular bulk out\n");
			   if (!udc_ep_ptr->first_bulk_t_rBQueued)
			   udc_ep_ptr->first_bulk_t_rBQueued = enq_pt;
			   udc_ep_ptr->last_bulk_t_rBQueued = enq_pt;
			   udc_ep_ptr->num_of_trbs++;
			   chain_bit = 1;
			   pcs ^= 0x1;
			 */
#define TRB_TYPE_XFER_STREAM	48
		if (udc_ep_ptr->comp_desc
				&& usb_ss_max_streams(udc_ep_ptr->comp_desc)) {
			setup_trb(nvudc, enq_pt, usb_req, buff_len_temp,
					trb_buf_addr, td_size-1,
					pcs, TRB_TYPE_XFER_STREAM, short_pkt,
					chain_bit, intr_on_compl, 0, 0, 0, 0,
					udc_req_ptr->usb_req.stream_id, 0);
		} else {
			setup_trb(nvudc, enq_pt, usb_req, buff_len_temp,
					trb_buf_addr, td_size-1,
					pcs, TRB_TYPE_XFER_NORMAL, short_pkt,
					chain_bit, intr_on_compl, 0, 0, 0, 0, 0,
					0);
		}
		}
		trb_buf_addr += buff_len_temp;
		td_size--;
		enq_pt++;
		j++;
		if (XHCI_GETF(TRB_TYPE, enq_pt->trb_dword3) == TRB_TYPE_LINK) {
			if (XHCI_GETF(TRB_LINK_TOGGLE_CYCLE,
					enq_pt->trb_dword3)) {

				XHCI_SETF_VAR(TRB_CYCLE_BIT,
					enq_pt->trb_dword3, udc_ep_ptr->pcs);
				udc_ep_ptr->pcs ^= 0x1;

				enq_pt = udc_ep_ptr->tran_ring_ptr;
			}
		}
	}


	udc_req_ptr->td_start = udc_ep_ptr->enq_pt;
	udc_ep_ptr->enq_pt = enq_pt;
	udc_req_ptr->buff_len_left = buffer_length;
	udc_req_ptr->trbs_needed = td_size;

	if (need_zlp && udc_req_ptr->buff_len_left == 0 &&
			!udc_ep_ptr->tran_ring_full)
		nvudc_queue_zlp_td(nvudc, udc_ep_ptr);

	return 0;
}



int nvudc_queue_ctrl(struct NV_UDC_EP *udc_ep_ptr,
		struct NV_UDC_REQUEST *udc_req_ptr, u32 num_of_trbs_needed)
{
	struct NV_UDC_S *nvudc = udc_ep_ptr->nvudc;
	struct EP_CX_S *p_ep_cx = nvudc->p_epcx;
	u8 ep_state = XHCI_GETF(EP_CX_EP_STATE, p_ep_cx->ep_dw0);
	struct TRANSFER_TRB_S *enq_pt = udc_ep_ptr->enq_pt;
	u32 deq_pt_lo = p_ep_cx->ep_dw2 & EP_CX_TR_DQPT_LO_MASK;
	struct TRANSFER_TRB_S *dq_pt;
	u32 deq_pt_hi = p_ep_cx->ep_dw3;
	u64 dq_pt_addr = (u64)deq_pt_lo + ((u64)deq_pt_hi << 32);
	struct usb_request *usb_req = &udc_req_ptr->usb_req;
	struct TRANSFER_TRB_S *p_trb;
	u32 transfer_length;
	u32 td_size = 0;
	u8 IOC;
	u8 dir = 0;

	dq_pt = tran_trb_dma_to_virt(udc_ep_ptr, dq_pt_addr);

	msg_entry(nvudc->dev);
	msg_dbg(nvudc->dev, "num_of_trbs_needed = 0x%x\n", num_of_trbs_needed);

	/* Need to queue the request even ep is paused or halted */
	if (ep_state != EP_STATE_RUNNING) {
		msg_dbg(nvudc->dev, "EP State = 0x%x\n", ep_state);
		return -EINVAL;
	}

	if (list_empty(&udc_ep_ptr->queue)) {
		/* For control endpoint, we can handle one setup request at a
		 time. so if there are TD pending in the transfer ring.
		 wait for the sequence number error event. Then put the new
		 request to tranfer ring */
		if (enq_pt == dq_pt) {
			u32 u_temp = 0, i;
			bool need_zlp = false;

			msg_dbg(nvudc->dev, "Setup Data Stage TRBs\n");
			/* Transfer ring is empty
			   setup data stage TRBs */
			udc_req_ptr->td_start = udc_ep_ptr->enq_pt;

			if (nvudc->setup_status ==  DATA_STAGE_XFER)
				dir = 1;
			else if (nvudc->setup_status == DATA_STAGE_RECV)
				dir = 0;
			else
				msg_dbg(nvudc->dev, "unexpected setup_status!%d\n"
						, nvudc->setup_status);

			if (udc_req_ptr->usb_req.zero == 1 &&
				udc_req_ptr->usb_req.length != 0 &&
				((udc_req_ptr->usb_req.length %
				  udc_ep_ptr->usb_ep.maxpacket) == 0))
				need_zlp = true;

			msg_dbg(nvudc->dev, "dir=%d, enq_pt=0x%p\n",
					dir, enq_pt);

			for (i = 0; i < num_of_trbs_needed; i++) {
				p_trb = enq_pt;
				if (i < (num_of_trbs_needed - 1)) {
					transfer_length = TRB_MAX_BUFFER_SIZE;
					IOC = 0;
				} else {
					u_temp = TRB_MAX_BUFFER_SIZE * i;
					transfer_length = (u32)usb_req->length
						- u_temp;
					/* If zlp is needed, IOC is set in
					   zlp. */
					IOC = need_zlp ? 0 : 1;
				}

				msg_dbg(nvudc->dev,
					"tx_len = 0x%x, u_temp = 0x%x\n",
					transfer_length, u_temp);

				setup_datastage_trb(nvudc, p_trb, usb_req,
					udc_ep_ptr->pcs, i,
					transfer_length, td_size, IOC, dir);
				udc_req_ptr->all_trbs_queued = 1;
				enq_pt++;

				if (XHCI_GETF(TRB_TYPE, enq_pt->trb_dword3) ==
						TRB_TYPE_LINK) {
					if (XHCI_GETF(TRB_LINK_TOGGLE_CYCLE,
							enq_pt->trb_dword3)) {

						XHCI_SETF_VAR(TRB_CYCLE_BIT,
							enq_pt->trb_dword3,
							udc_ep_ptr->pcs);
						udc_ep_ptr->pcs ^= 0x1;
					}

					enq_pt = udc_ep_ptr->tran_ring_ptr;
				}
			}

			udc_ep_ptr->enq_pt = enq_pt;

			u_temp = 0;

			u_temp = DB_TARGET(0);


			u_temp |= DB_STREAMID(nvudc->ctrl_seq_num);

			msg_dbg(nvudc->dev, "DB register 0x%x\n",
					u_temp);

			iowrite32(u_temp, nvudc->mmio_reg_base + DB);

			if (need_zlp) {
				u32 dw;
				nvudc_queue_zlp_td(nvudc, udc_ep_ptr);

				dw = DB_TARGET(0);
				dw |= DB_STREAMID(nvudc->ctrl_seq_num);
				msg_dbg(nvudc->dev, "DB register 0x%x\n", dw);
				iowrite32(dw, nvudc->mmio_reg_base + DB);
			}
		} else {
			/* we process one setup request at a time, so ring
			 * should already be empty.*/
			msg_dbg(nvudc->dev, "Eq = 0x%p != Dq = 0x%p\n",
				 enq_pt, dq_pt);
			/* Assert() */
		}
	} else {
		msg_dbg(nvudc->dev, "udc_ep_ptr->queue not empty\n");
		/* New setup packet came
		   Drop the this req.. */
		return -ECANCELED;
	}

	return 0;
}

int nvudc_build_td(struct NV_UDC_EP *udc_ep_ptr,
		struct NV_UDC_REQUEST *udc_req_ptr)
{
	int status = 0;
	struct NV_UDC_S *nvudc = udc_ep_ptr->nvudc;
	u32 num_trbs_needed;
	u64 buffer_length;
	u32 u_temp;

	msg_entry(nvudc->dev);

	if (udc_req_ptr->trbs_needed) {
		/* If this is called to complete pending TRB transfers
		 * of previous Request
		 */
		buffer_length = udc_req_ptr->buff_len_left;
		num_trbs_needed = udc_req_ptr->trbs_needed;
	} else {
		buffer_length = (u64)udc_req_ptr->usb_req.length;
		num_trbs_needed = (u32)(buffer_length / TRB_MAX_BUFFER_SIZE);

		if ((buffer_length == 0) ||
			(buffer_length % TRB_MAX_BUFFER_SIZE))
			num_trbs_needed += 1;
	}
	msg_dbg(nvudc->dev, "buf_len = %ld, num_trb_needed = %d",
	(unsigned long)buffer_length, num_trbs_needed);

	if (usb_endpoint_xfer_control(udc_ep_ptr->desc))
		status = nvudc_queue_ctrl(udc_ep_ptr,
				 udc_req_ptr, num_trbs_needed);
	else if (usb_endpoint_xfer_isoc(udc_ep_ptr->desc)) {
		status = nvudc_queue_trbs(udc_ep_ptr, udc_req_ptr, 1,
				NVUDC_ISOC_EP_TD_RING_SIZE,
				num_trbs_needed, buffer_length);
		u_temp = udc_ep_ptr->DCI;
		u_temp = DB_TARGET(u_temp);
		msg_dbg(nvudc->dev, "DOORBELL = 0x%x\n", u_temp);
		iowrite32(u_temp, nvudc->mmio_reg_base + DB);

	} else if (usb_endpoint_xfer_bulk(udc_ep_ptr->desc)) {
		msg_dbg(nvudc->dev, "nvudc_queue_trbs\n");
		status = nvudc_queue_trbs(udc_ep_ptr, udc_req_ptr, 0,
				NVUDC_BULK_EP_TD_RING_SIZE,
				num_trbs_needed, buffer_length);
		u_temp = udc_ep_ptr->DCI;
		u_temp = DB_TARGET(u_temp);
		if (udc_ep_ptr->comp_desc &&
				usb_ss_max_streams(udc_ep_ptr->comp_desc)) {
			/* hold the doorbell if stream_rejected is set */
			if (nvudc->stream_rejected & NV_BIT(udc_ep_ptr->DCI))
				return status;
			u_temp |= DB_STREAMID(udc_req_ptr->usb_req.stream_id);
		}
		msg_dbg(nvudc->dev, "DOORBELL = 0x%x\n", u_temp);
		iowrite32(u_temp, nvudc->mmio_reg_base + DB);

	} else {
		status = nvudc_queue_trbs(udc_ep_ptr, udc_req_ptr, 0,
				NVUDC_INT_EP_TD_RING_SIZE,
				num_trbs_needed, buffer_length);
		u_temp = udc_ep_ptr->DCI;
		u_temp = DB_TARGET(u_temp);
		msg_dbg(nvudc->dev, "DOORBELL = 0x%x\n", u_temp);
		iowrite32(u_temp, nvudc->mmio_reg_base + DB);
	}

	return status;
}

static int
nvudc_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct NV_UDC_REQUEST *udc_req_ptr;
	struct NV_UDC_EP *udc_ep_ptr;
	struct NV_UDC_S *nvudc;
	int status;
	unsigned long flags;

	if (!_req || !_ep)
		return -EINVAL;

	udc_req_ptr = container_of(_req, struct NV_UDC_REQUEST, usb_req);
	udc_ep_ptr = container_of(_ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep_ptr->nvudc;
	msg_entry(nvudc->dev);

	if (!udc_ep_ptr->tran_ring_ptr ||
		!udc_req_ptr->usb_req.complete ||
		!udc_req_ptr->usb_req.buf ||
		!list_empty(&udc_req_ptr->queue))
		return -EINVAL;
	msg_dbg(nvudc->dev, "EPDCI = 0x%x\n", udc_ep_ptr->DCI);

	if (!udc_ep_ptr->desc) {
		msg_dbg(nvudc->dev, "udc_ep_ptr->Desc is null\n");
		return -EINVAL;
	}

	/* Clearing the Values of the NV_UDC_REQUEST container */
	udc_req_ptr->buff_len_left = 0;
	udc_req_ptr->trbs_needed = 0;
	udc_req_ptr->mapped = 0;
	udc_req_ptr->all_trbs_queued = 0;
	udc_req_ptr->td_start = NULL;
	udc_req_ptr->short_pkt = 0;

	spin_lock_irqsave(&nvudc->lock, flags);
	if (usb_endpoint_xfer_control(udc_ep_ptr->desc) &&
				(_req->length == 0)) {
		nvudc->setup_status = STATUS_STAGE_XFER;
		status = -EINPROGRESS;
		if (udc_req_ptr) {
			msg_dbg(nvudc->dev,
				"udc_req_ptr = 0x%p\n", udc_req_ptr);

			build_ep0_status(&nvudc->udc_ep[0], false, status,
					udc_req_ptr);
		} else {
			msg_dbg(nvudc->dev, "udc_req_ptr = NULL\n");
			build_ep0_status(&nvudc->udc_ep[0], true, status, NULL);
		}
		msg_dbg(nvudc->dev,
			"act status request for control endpoint\n");
		spin_unlock_irqrestore(&nvudc->lock, flags);
		return 0;
	}

	/* request length is possible to be 0. Like SCSI blank command */
	msg_dbg(nvudc->dev, "request length=%d\n", _req->length);

	if (udc_req_ptr->usb_req.dma == DMA_ADDR_INVALID) {
		udc_req_ptr->usb_req.dma =
			dma_map_single(nvudc->gadget.dev.parent,
					udc_req_ptr->usb_req.buf,
					udc_req_ptr->usb_req.length,
					usb_endpoint_dir_in(udc_ep_ptr->desc)
					? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		udc_req_ptr->mapped = 1;
	}

	udc_req_ptr->usb_req.status = -EINPROGRESS;
	udc_req_ptr->usb_req.actual = 0;

	/* If the transfer ring for this particular end point is full,
	 * then simply queue the request and return
	 */
	if (udc_ep_ptr->tran_ring_full == true) {
		status = 0;
	} else {
		/* push the request to the transfer ring if possible. */
		status = nvudc_build_td(udc_ep_ptr, udc_req_ptr);
	}
	if (!status) {

		if (udc_req_ptr)
			list_add_tail(&udc_req_ptr->queue, &udc_ep_ptr->queue);
	}
	spin_unlock_irqrestore(&nvudc->lock, flags);
	msg_exit(nvudc->dev);
	return status;
}

/* This function will go through the list of the USB requests for the
 * given endpoint and schedule any unscheduled trb's to the xfer ring
 */
void queue_pending_trbs(struct NV_UDC_EP *nvudc_ep_ptr)
{
	struct NV_UDC_S *nvudc = nvudc_ep_ptr->nvudc;
	struct NV_UDC_REQUEST *udc_req_ptr;
	msg_entry(nvudc->dev);
	/* schedule  trbs till there arent any pending unscheduled ones
	 * or the ring is full again
	 */
	list_for_each_entry(udc_req_ptr, &nvudc_ep_ptr->queue, queue) {
		if (udc_req_ptr->all_trbs_queued == 0)
			nvudc_build_td(nvudc_ep_ptr, udc_req_ptr);

		if (nvudc_ep_ptr->tran_ring_full == true)
			break;
		}
	msg_exit(nvudc->dev);
}

u32 actual_data_xfered(struct NV_UDC_EP *udc_ep, struct NV_UDC_REQUEST *udc_req)
{
	struct NV_UDC_S *nvudc = udc_ep->nvudc;
	struct EP_CX_S *p_ep_cx = nvudc->p_epcx + udc_ep->DCI;
	u16 data_offset = XHCI_GETF(EP_CX_DATA_OFFSET, p_ep_cx->ep_dw7);
	u8 num_trbs = XHCI_GETF(EP_CX_NUMTRBS, p_ep_cx->ep_dw7);
	u64 data_left = ((num_trbs+1) * TRB_MAX_BUFFER_SIZE) -
		data_offset;
	return udc_req->usb_req.length - data_left;
}

void flush_xfer_ring(struct NV_UDC_EP *udc_ep_ptr,
		struct NV_UDC_REQUEST *udc_req_ptr, u8 num_trbs)
{
	struct NV_UDC_S *nvudc = udc_ep_ptr->nvudc;
	struct TRANSFER_TRB_S *enq_pt = udc_req_ptr->td_start;
	struct TRANSFER_TRB_S *dq_pt;
	struct EP_CX_S *p_ep_cx = nvudc->p_epcx + udc_ep_ptr->DCI;
	u32 deq_pt_lo = p_ep_cx->ep_dw2 & EP_CX_TR_DQPT_LO_MASK;
	u32 deq_pt_hi = p_ep_cx->ep_dw3;
	u64   dq_pt_addr = (u64)deq_pt_lo + ((u64)deq_pt_hi << 32);

	dq_pt = tran_trb_dma_to_virt(udc_ep_ptr, dq_pt_addr);

	while ((enq_pt+1) != dq_pt) {
		enq_pt->data_buf_ptr_lo = 0;
		enq_pt->data_buf_ptr_hi = 0;
		enq_pt->trb_dword2 = 0;
		enq_pt->trb_dword3 = 0;

		enq_pt++;

		if (XHCI_GETF(TRB_TYPE, enq_pt->trb_dword3) == TRB_TYPE_LINK)
			enq_pt = udc_ep_ptr->tran_ring_ptr;
	}

}

	static int
nvudc_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct NV_UDC_EP *udc_ep;
	struct NV_UDC_S *nvudc;
	u32 u_temp, u_temp2 = 0;
	struct NV_UDC_REQUEST *udc_req;
	int ret = 0;
	struct EP_CX_S *p_ep_cx;
	u8 ep_state;
	struct NV_UDC_REQUEST *p_new_udc_req;
	u64 new_dq_pt;
	unsigned long flags;

	if (!_ep || !_req)
		return -EINVAL;

	udc_ep = container_of(_ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep->nvudc;
	msg_entry(nvudc->dev);

	p_ep_cx = nvudc->p_epcx + udc_ep->DCI;
	ep_state = XHCI_GETF(EP_CX_EP_STATE, p_ep_cx->ep_dw0);

	msg_dbg(nvudc->dev, "EPDCI = 0x%x\n", udc_ep->DCI);

	spin_lock_irqsave(&nvudc->lock, flags);
	if (ep_state == EP_STATE_RUNNING) {

		msg_dbg(nvudc->dev, "EP_STATE_RUNNING\n");
		/* stop the DMA from HW first */
		u_temp = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
		u_temp2 = NV_BIT(udc_ep->DCI);
		u_temp |= u_temp2;

		iowrite32(u_temp, nvudc->mmio_reg_base + EP_PAUSE);
		msg_dbg(nvudc->dev, "pause endpoints 0x%x", u_temp);

		poll_stchg(nvudc->dev, "ep_dequeue()", u_temp2);

		msg_dbg(nvudc->dev, "STCHG = 0x%x\n",
			ioread32(nvudc->mmio_reg_base + EP_STCHG));

		iowrite32(u_temp2, nvudc->mmio_reg_base + EP_STCHG);
		msg_dbg(nvudc->dev, "clear the state change register\n");
	}


	list_for_each_entry(udc_req, &udc_ep->queue, queue) {
		if (&udc_req->usb_req == _req)
			break;
	}

	if (&udc_req->usb_req != _req) {
		ret = -EINVAL;
		msg_dbg(nvudc->dev,
			" did not find the request in request queue\n");
		goto dqerr;
	}

	if (udc_ep->queue.next == &udc_req->queue) {
		msg_dbg(nvudc->dev, "head of the request queue\n");

		/* this request is handling by hw or is completed but haven't
		 * got dequeue yet
		 update dequeue pointer to next TD. */
		udc_req->usb_req.actual = actual_data_xfered(udc_ep, udc_req);
		p_new_udc_req = list_entry(udc_ep->queue.next,
						struct NV_UDC_REQUEST, queue);

		if (p_new_udc_req) {
			msg_dbg(nvudc->dev, " more requests pending\n");
			new_dq_pt =
			tran_trb_virt_to_dma(udc_ep, p_new_udc_req->td_start);

			p_ep_cx->ep_dw2 = lower_32_bits(new_dq_pt)
							& EP_CX_TR_DQPT_LO_MASK;
			XHCI_SETF_VAR(EP_CX_DEQ_CYC_STATE, p_ep_cx->ep_dw2,
						   udc_ep->pcs);
			p_ep_cx->ep_dw3 = upper_32_bits(new_dq_pt);

			XHCI_SETF_VAR(EP_CX_EDTLA, p_ep_cx->ep_dw5, 0);
			XHCI_SETF_VAR(EP_CX_PARTIALTD, p_ep_cx->ep_dw5, 0);
			XHCI_SETF_VAR(EP_CX_DATA_OFFSET, p_ep_cx->ep_dw7, 0);
		}

		msg_dbg(nvudc->dev, "complete requests\n");
		req_done(udc_ep, udc_req, -ECONNRESET);

		msg_dbg(nvudc->dev, "reload endpoint\n");
		iowrite32(NV_BIT(udc_ep->DCI),
			nvudc->mmio_reg_base + EP_RELOAD);
		poll_reload(nvudc->dev, "ep_dequeue()", NV_BIT(udc_ep->DCI));

		if (udc_ep->tran_ring_full == true) {
			udc_ep->tran_ring_full = false;
			queue_pending_trbs(udc_ep);
		}

	} else if (udc_req->td_start == NULL) {
		msg_dbg(nvudc->dev, "udc_req->td_start == NULL\n");
		/* this request is in the software queue, but haven't write to
		 * the transfer ring yet */
		req_done(udc_ep, udc_req, -ECONNRESET);
	} else {
		/* request was written to the transfer ring, but haven't
		 * processed by HW yet.*/
		u32 xfer_ring_size;
		struct NV_UDC_REQUEST *next_req;
		msg_dbg(nvudc->dev, "haven't process by HW yet.\n");

		if (usb_endpoint_xfer_control(udc_ep->desc))
			xfer_ring_size = NVUDC_CONTROL_EP_TD_RING_SIZE;
		else if (usb_endpoint_xfer_isoc(udc_ep->desc))
			xfer_ring_size = NVUDC_ISOC_EP_TD_RING_SIZE;
		else if (usb_endpoint_xfer_bulk(udc_ep->desc))
			xfer_ring_size = NVUDC_BULK_EP_TD_RING_SIZE;
		else
			xfer_ring_size = NVUDC_INT_EP_TD_RING_SIZE;

		flush_xfer_ring(udc_ep, udc_req, xfer_ring_size);
		/* Update the new enq_ptr starting from the deleted req */
		udc_ep->enq_pt = udc_req->td_start;

		if (udc_ep->tran_ring_full == true)
			udc_ep->tran_ring_full = false;

		next_req = list_entry(udc_req->queue.next,
					struct NV_UDC_REQUEST,	queue);

		list_for_each_entry_from(next_req, &udc_ep->queue, queue) {
			next_req->usb_req.status = -EINPROGRESS;
			next_req->usb_req.actual = 0;

			/* The follwing variable being non zero means that
			* some trbs's of a request are queued in the ring
			* and only the remaining have to be added when
			* build_TD is called. When we flush the transfer
			* ring before, these trb's are flushed. So we should
			* clear this variable so that all the trb's are again
			* added to the ring
			*/
			if (next_req->trbs_needed != 0)
				next_req->trbs_needed = 0;

			if (udc_ep->tran_ring_full == false) {
				/* push the request to the transfer ring */
				ret = nvudc_build_td(udc_ep, next_req);
			}
		}
		req_done(udc_ep, udc_req, -ECONNRESET);
	}

	/* clear PAUSE bit and reresume data transfer */
	if (ep_state == EP_STATE_RUNNING) {
		u_temp = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
		u_temp &= ~(1 << udc_ep->DCI);
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_PAUSE);

		poll_stchg(nvudc->dev, "EP PAUSED", (1 << udc_ep->DCI));

		iowrite32(NV_BIT(udc_ep->DCI), nvudc->mmio_reg_base + EP_STCHG);
	}

dqerr:
	spin_unlock_irqrestore(&nvudc->lock, flags);
	msg_exit(nvudc->dev);
	return ret;
}

/*
 * This function will set or clear ep halt state depending on value input.
 */
static int nvudc_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct NV_UDC_EP *udc_ep_ptr;
	struct NV_UDC_S *nvudc;
	int status;
	unsigned long flags;

	if (!_ep)
		return -EINVAL;

	udc_ep_ptr = container_of(_ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep_ptr->nvudc;
	msg_entry(nvudc->dev);

	if (value && usb_endpoint_dir_in(udc_ep_ptr->desc) &&
			!list_empty(&udc_ep_ptr->queue)) {
		msg_err(nvudc->dev, "list not empty\n");
		return -EAGAIN;
	}

	spin_lock_irqsave(&nvudc->lock, flags);
	status = ep_halt(udc_ep_ptr, value);
	spin_unlock_irqrestore(&nvudc->lock, flags);

	msg_exit(nvudc->dev);
	return status;
}

static int ep_halt(struct NV_UDC_EP *udc_ep_ptr, int halt)
{
	struct NV_UDC_S *nvudc;
	u32 u_temp, u_temp2, u_pause;
	struct EP_CX_S *p_ep_cx;
	bool reset_seq_only = false;

	nvudc = udc_ep_ptr->nvudc;
	msg_entry(nvudc->dev);

	u_temp = ioread32(nvudc->mmio_reg_base + EP_HALT);
	u_temp2 = NV_BIT(udc_ep_ptr->DCI);

	if (!udc_ep_ptr->desc) {
		msg_err(nvudc->dev, "NULL desc\n");
		return -EINVAL;
	}

	if (udc_ep_ptr->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		msg_err(nvudc->dev, "Isoc ep, halt not supported\n");
		return -EOPNOTSUPP;
	}

	p_ep_cx = nvudc->p_epcx + udc_ep_ptr->DCI;

	if (((u_temp >> udc_ep_ptr->DCI) & 1) == halt) {
		msg_dbg(nvudc->dev, "EP status already : %d, EP_STATE = %d\n",
		halt, XHCI_GETF(EP_CX_EP_STATE, p_ep_cx->ep_dw0));

		if (!halt) {
			/* For unhalt a not halted EP case, driver still need to
			clear sequecne number in EPCx. No unhalt EP again. */
			reset_seq_only = true;
		} else {
			/* Just return for halt a halted EP case.*/
			return 0;
		}
	}

	if (halt && !reset_seq_only) {
		/* setting ep to halt */
		u_temp |= u_temp2;

		msg_dbg(nvudc->dev, "HALT EP = 0x%x\n", u_temp2);
		msg_dbg(nvudc->dev, "XHCI_EP_HALT = 0x%x\n", u_temp);
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_HALT);
		poll_stchg(nvudc->dev, "ep_sethalt()", u_temp2);

		msg_dbg(nvudc->dev, "EP_STCHG = 0x%x, u_temp2 = 0x%x\n",
			ioread32(nvudc->mmio_reg_base + EP_STCHG), u_temp2);
		iowrite32(u_temp2, nvudc->mmio_reg_base + EP_STCHG);
	} else {
		/* clearing ep halt state */
		msg_dbg(nvudc->dev, "Clear EP HALT = 0x%x\n", u_temp2);
		/* disable ep and reset sequence number */
		XHCI_SETF_VAR(
			EP_CX_EP_STATE, p_ep_cx->ep_dw0, EP_STATE_DISABLED);

		iowrite32(u_temp2, nvudc->mmio_reg_base + EP_RELOAD);
		poll_reload(nvudc->dev, "ep_clearhalt()", u_temp2);

		msg_dbg(nvudc->dev, "EP_RELOAD = 0x%x, u_temp2 = 0x%x\n",
		ioread32(nvudc->mmio_reg_base + EP_RELOAD), u_temp2);

		if (!reset_seq_only) {
			/* Clear halt for a halted EP.*/
			u_temp &= ~u_temp2;
			iowrite32(u_temp, nvudc->mmio_reg_base + EP_HALT);
			poll_stchg(nvudc->dev, "ep_clearhalt()", u_temp2);

			msg_dbg(nvudc->dev, "EP_STCHG = 0x%x, u_temp2 = 0x%x\n",
			ioread32(nvudc->mmio_reg_base + EP_STCHG), u_temp2);

			iowrite32(u_temp2, nvudc->mmio_reg_base + EP_STCHG);
		}

		/* set endpoint to running state */
		XHCI_SETF_VAR(
			EP_CX_EP_STATE, p_ep_cx->ep_dw0, EP_STATE_RUNNING);
		XHCI_SETF_VAR(EP_CX_SEQ_NUM, p_ep_cx->ep_dw5, 0);

		iowrite32(u_temp2, nvudc->mmio_reg_base + EP_RELOAD);
		poll_reload(nvudc->dev, "ep_clearhalt()", u_temp2);

		msg_dbg(nvudc->dev, "EP_RELOAD = 0x%x, u_temp2 = 0x%x\n",
			ioread32(nvudc->mmio_reg_base + EP_RELOAD), u_temp2);

		/* clear pause for the endpoint */
		u_pause = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
		u_pause &= ~u_temp2;
		iowrite32(u_pause, nvudc->mmio_reg_base + EP_PAUSE);
		poll_stchg(nvudc->dev, "ep_clearhalt()", u_temp2);

		msg_dbg(nvudc->dev, "EP_STCHG = 0x%x, u_temp2 = 0x%x\n",
			ioread32(nvudc->mmio_reg_base + EP_STCHG), u_temp2);

		if (!list_empty(&udc_ep_ptr->queue)) {
			u_temp = udc_ep_ptr->DCI;
			u_temp = DB_TARGET(u_temp);
			iowrite32(u_temp, nvudc->mmio_reg_base + DB);
			msg_dbg(nvudc->dev, "DOORBELL = 0x%x\n", u_temp);
		}
	}

	msg_exit(nvudc->dev);
	return 0;
}

void doorbell_for_unpause(struct NV_UDC_S *nvudc, u32 paused_bits)
{
	int i;
	u32 u_temp;
	struct NV_UDC_EP *udc_ep_ptr;

	for (i = 0; i < NUM_EP_CX; i++) {
		if (!paused_bits)
			break;
		if (paused_bits & 1) {
			udc_ep_ptr = &nvudc->udc_ep[i];
			if (!list_empty(&udc_ep_ptr->queue)) {
				u_temp = udc_ep_ptr->DCI;
				u_temp = DB_TARGET(u_temp);
				iowrite32(u_temp, nvudc->mmio_reg_base + DB);
			}
		}
		paused_bits = paused_bits >> 1;
	}
}

static struct usb_ep_ops nvudc_ep_ops = {
	.enable = nvudc_ep_enable,
	.disable = nvudc_ep_disable,
	.alloc_request = nvudc_alloc_request,
	.free_request = nvudc_free_request,
	.queue = nvudc_ep_queue,
	.dequeue = nvudc_ep_dequeue,
	.set_halt = nvudc_ep_set_halt,
};

static int nvudc_gadget_get_frame(struct usb_gadget *gadget)
{
	u32 u_temp;
	struct NV_UDC_S *nvudc = container_of(gadget, struct NV_UDC_S, gadget);
	u_temp = ioread32(nvudc->mmio_reg_base + MFINDEX);
	u_temp = MFINDEX_FRAME(u_temp);
	return u_temp >> MFINDEX_FRAME_SHIFT;
}

static void nvudc_resume_state(struct NV_UDC_S *nvudc)
{
	u32 ep_paused, u_temp, u_temp2;

	/* Unpausing the Endpoints */
	ep_paused = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
	iowrite32(0, nvudc->mmio_reg_base + EP_PAUSE);
	poll_stchg(nvudc->dev, "Resume_state", ep_paused);
	iowrite32(ep_paused, nvudc->mmio_reg_base + EP_STCHG);

	u_temp = ioread32(nvudc->mmio_reg_base + PORTSC);
	u_temp2 = u_temp & PORTSC_MASK;

	if (nvudc->gadget.speed == USB_SPEED_SUPER) {
		msg_dbg(nvudc->dev, "Directing link to U0\n");
		/* don't clear (write to 1) bits which we are not handling */
		u_temp2 &= ~PORTSC_PLS(-1);
		u_temp2 |= PORTSC_PLS(0);
		u_temp2 |= PORTSC_LWS;
	}

	/* IF this is called because of Port LInk state change Event */
	if (PORTSC_PLC & u_temp)
		u_temp2 |= PORTSC_PLC;

	iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);

	nvudc->device_state = nvudc->resume_state;
	nvudc->resume_state = 0;

	/* ring door bell for the paused endpoints */
	doorbell_for_unpause(nvudc, ep_paused);
}

static int nvudc_gadget_wakeup(struct usb_gadget *gadget)
{
	struct NV_UDC_S *nvudc = container_of(gadget, struct NV_UDC_S, gadget);
	u32 utemp, u_temp2;
	unsigned long flags;

	msg_entry(nvudc->dev);
	spin_lock_irqsave(&nvudc->lock, flags);

	utemp = ioread32(nvudc->mmio_reg_base + PORTPM);

	msg_dbg(nvudc->dev, "PORTPM = %08x, speed = %d\n", utemp,
		nvudc->gadget.speed);

	if (((nvudc->gadget.speed == USB_SPEED_FULL ||
		nvudc->gadget.speed == USB_SPEED_HIGH) &&
		(PORTPM_RWE & utemp)) ||
		((nvudc->gadget.speed == USB_SPEED_SUPER) &&
		(PORTPM_FRWE & utemp))) {

		nvudc_resume_state(nvudc);

		/* write the function wake device notification register */
		if (nvudc->gadget.speed == USB_SPEED_SUPER) {
			u_temp2 = 0;
			u_temp2 |= DEVNOTIF_LO_TYPE(1);
			u_temp2 |= DEVNOTIF_LO_TRIG;
			iowrite32(0, nvudc->mmio_reg_base + DEVNOTIF_HI);
			iowrite32(u_temp2, nvudc->mmio_reg_base + DEVNOTIF_LO);
			msg_dbg(nvudc->dev, "Sent DEVNOTIF\n");
		}
	}
	spin_unlock_irqrestore(&nvudc->lock, flags);

	msg_exit(nvudc->dev);
	return 0;
}

static int nvudc_gadget_pullup(struct usb_gadget *gadget, int is_on)
{
	u32 temp;
	unsigned long flags;
	struct NV_UDC_S *nvudc = container_of(gadget, struct NV_UDC_S, gadget);
	msg_dbg(nvudc->dev, "pullup is_on = %x", is_on);

	spin_lock_irqsave(&nvudc->lock, flags);
	temp = ioread32(nvudc->mmio_reg_base + CTRL);
	if (is_on != nvudc->pullup) {
		if (is_on) {
			/* set ENABLE bit */
			temp |= CTRL_ENABLE;
		} else {
			/* clear ENABLE bit */
			temp &= ~CTRL_ENABLE;
		}
		nvudc->pullup = is_on;
		iowrite32(temp, nvudc->mmio_reg_base + CTRL);
		nvudc->device_state = USB_STATE_DEFAULT;
	}
	spin_unlock_irqrestore(&nvudc->lock, flags);

	/* update vbus status */
	extcon_notifications(&nvudc->vbus_extcon_nb, 0, NULL);

	/* update id status */
	extcon_id_notifications(&nvudc->id_extcon_nb, 0, NULL);

	return 0;
}

static int nvudc_vbus_draw(struct usb_gadget *gadget, unsigned m_a)
{
	struct NV_UDC_S *nvudc;

	nvudc = container_of(gadget, struct NV_UDC_S, gadget);
	msg_dbg(nvudc->dev, "nvudc_vbus_draw m_a= 0x%x\n", m_a);
	return -ENOTSUPP;
}

/* defined in gadget.h */
static struct usb_gadget_ops nvudc_gadget_ops = {
	.get_frame = nvudc_gadget_get_frame,
	.wakeup = nvudc_gadget_wakeup,
	.pullup = nvudc_gadget_pullup,
	/* TODO: remove when we don't support 3.8.2 anymore */
	.udc_start = nvudc_gadget_start,
	.udc_stop = nvudc_gadget_stop,
	.vbus_draw = nvudc_vbus_draw,
};

static void nvudc_gadget_release(struct device *dev)
{
	return;
}

static void update_ep0_maxpacketsize(struct NV_UDC_S *nvudc)
{
	u16 maxpacketsize = 0;
	struct EP_CX_S *p_epcx = nvudc->p_epcx;
	struct NV_UDC_EP *ep0 = &nvudc->udc_ep[0];

	if (nvudc->gadget.speed == USB_SPEED_SUPER)
		maxpacketsize = 512;
	else
		maxpacketsize = 64;

	XHCI_SETF_VAR(EP_CX_MAX_PACKET_SIZE, p_epcx->ep_dw1, maxpacketsize);
	nvudc_ep0_desc.wMaxPacketSize = cpu_to_le16(maxpacketsize);
	ep0->usb_ep.maxpacket = maxpacketsize;
}

int init_ep0(struct NV_UDC_S *nvudc)
{
	struct EP_CX_S *epcx = nvudc->p_epcx;
	struct NV_UDC_EP *udc_ep = &nvudc->udc_ep[0];
	struct device *dev = nvudc->dev;
	u32 dw;

	/* setup transfer ring */
	if (!udc_ep->tran_ring_info.vaddr) {
		dma_addr_t dma;
		u32 ring_size = NVUDC_CONTROL_EP_TD_RING_SIZE;
		void *vaddr;
		size_t len;

		len = ring_size * sizeof(struct TRANSFER_TRB_S);
		vaddr = dma_alloc_coherent(nvudc->dev, len, &dma, GFP_ATOMIC);
		if (!vaddr) {
			msg_err(dev, "failed to allocate trb ring\n");
			return -ENOMEM;
		}

		udc_ep->tran_ring_info.vaddr = vaddr;
		udc_ep->tran_ring_info.dma = dma;
		udc_ep->tran_ring_info.len = len;
		udc_ep->first_trb = vaddr;
		udc_ep->link_trb = udc_ep->first_trb + ring_size - 1;
	}

	memset(udc_ep->first_trb, 0, udc_ep->tran_ring_info.len);
	udc_ep->enq_pt = udc_ep->first_trb;
	udc_ep->tran_ring_full = false;

	dw = 0;
	XHCI_SETF_VAR(EP_CX_EP_STATE, dw, EP_STATE_RUNNING);
	epcx->ep_dw0 = cpu_to_le32(dw);

	dw = 0;
	XHCI_SETF_VAR(EP_CX_EP_TYPE, dw, EP_TYPE_CNTRL);
	XHCI_SETF_VAR(EP_CX_CERR, dw, 3);
	XHCI_SETF_VAR(EP_CX_MAX_BURST_SIZE, dw, 0);
	XHCI_SETF_VAR(EP_CX_MAX_PACKET_SIZE, dw, 512);
	epcx->ep_dw1 = cpu_to_le32(dw);

	dw = lower_32_bits(udc_ep->tran_ring_info.dma) & EP_CX_TR_DQPT_LO_MASK;
	udc_ep->pcs = 1;
	XHCI_SETF_VAR(EP_CX_DEQ_CYC_STATE, dw, udc_ep->pcs);
	epcx->ep_dw2 = cpu_to_le32(dw);

	dw = upper_32_bits(udc_ep->tran_ring_info.dma);
	epcx->ep_dw3 = cpu_to_le32(dw);
	msg_dbg(dev, "EPTr_dq_pt_hi = 0x%x\n", dw);

	setup_link_trb(udc_ep->link_trb, true, udc_ep->tran_ring_info.dma);
	msg_dbg(dev, "Link Trb address = 0x%p\n", udc_ep->link_trb);

	dw = 0;
	XHCI_SETF_VAR(EP_CX_AVE_TRB_LEN, dw, 8);
	epcx->ep_dw4 = cpu_to_le32(dw);

	epcx->ep_dw5 = 0;

	dw = 0;
	XHCI_SETF_VAR(EP_CX_CERRCNT, epcx->ep_dw6, 3);
	epcx->ep_dw6 = cpu_to_le32(dw);

	epcx->ep_dw7 = 0;
	epcx->ep_dw8 = 0;
	epcx->ep_dw9 = 0;
	epcx->ep_dw11 = 0;
	epcx->ep_dw12 = 0;
	epcx->ep_dw13 = 0;
	epcx->ep_dw15 = 0;

	return 0;
}

int EP0_Start(struct NV_UDC_S *nvudc)
{
	struct usb_ep *usb_ep;

	usbep_struct_setup(nvudc, 0, NULL);

	nvudc->udc_ep[0].desc = &nvudc_ep0_desc;

	usb_ep = &nvudc->udc_ep[0].usb_ep;

	usb_ep->ops = &nvudc_ep_ops;

	return 0;
}

void build_ep0_status(struct NV_UDC_EP *udc_ep_ptr, bool default_value, u32
		status, struct NV_UDC_REQUEST *udc_req_ptr)
{
	struct NV_UDC_S *nvudc = udc_ep_ptr->nvudc;
	struct TRANSFER_TRB_S *enq_pt = udc_ep_ptr->enq_pt;
	u32 u_temp;

	if (default_value) {
		udc_req_ptr = nvudc->status_req;
		udc_req_ptr->usb_req.length = 0;
		udc_req_ptr->usb_req.status = status;
		udc_req_ptr->usb_req.actual = 0;
		udc_req_ptr->usb_req.complete = NULL;
	} else {
		udc_req_ptr->usb_req.status = status;
		udc_req_ptr->usb_req.actual = 0;
	}

	msg_dbg(nvudc->dev, "udc_req_ptr=0x%p, enq_pt=0x%p\n",
			udc_req_ptr, enq_pt);

	setup_status_trb(nvudc, enq_pt, &udc_req_ptr->usb_req, udc_ep_ptr->pcs);

	enq_pt++;

	/* check if we are at end of trb segment.  If so, update
	 * pcs and enq for next segment
	 */
	if (XHCI_GETF(TRB_TYPE, enq_pt->trb_dword3) == TRB_TYPE_LINK) {
		if (XHCI_GETF(TRB_LINK_TOGGLE_CYCLE, enq_pt->trb_dword3)) {
			XHCI_SETF_VAR(
				TRB_CYCLE_BIT,
				enq_pt->trb_dword3,
				udc_ep_ptr->pcs);
			udc_ep_ptr->pcs ^= 0x1;
		}
		enq_pt = udc_ep_ptr->tran_ring_ptr;
	}
	udc_ep_ptr->enq_pt = enq_pt;

	/* Note: for ep0, streamid field is also used for seqnum.*/
	u_temp = 0;
	u_temp |= DB_STREAMID(nvudc->ctrl_seq_num);

	msg_dbg(nvudc->dev, "DB register 0x%x\n", u_temp);

	iowrite32(u_temp, nvudc->mmio_reg_base + DB);

	list_add_tail(&udc_req_ptr->queue, &udc_ep_ptr->queue);
}

void ep0_req_complete(struct NV_UDC_EP *udc_ep_ptr)
{
	struct NV_UDC_S *nvudc = udc_ep_ptr->nvudc;
	switch (nvudc->setup_status) {
	case DATA_STAGE_XFER:
		nvudc->setup_status = STATUS_STAGE_RECV;
		build_ep0_status(udc_ep_ptr, true, -EINPROGRESS, NULL);
		break;
	case DATA_STAGE_RECV:
		nvudc->setup_status = STATUS_STAGE_XFER;
		build_ep0_status(udc_ep_ptr, true, -EINPROGRESS, NULL);
		break;
	default:
		if (nvudc->setup_fn_call_back)
			nvudc->setup_fn_call_back(nvudc);
		nvudc->setup_status = WAIT_FOR_SETUP;
		break;
	}
}

#ifdef PRIME_NOT_RCVD_WAR
/* When the stream is rejected by Host, we call schedule_delayed_work
 * function from the IRQ handler which will make this function to be
 * executed by one of the kernel's already created worker threads. Since
 * this is executed in the Process context we can sleep here for some time
 * before retrying the stream
 */
static void retry_stream_rejected_work(struct work_struct *work)
{
	u32 u_temp;
	struct NV_UDC_EP *udc_ep_ptr = container_of(work, struct NV_UDC_EP,
								      work);
	struct NV_UDC_S *nvudc = udc_ep_ptr->nvudc;
	msleep(stream_rejected_sleep_msecs);

	u_temp = NV_BIT(udc_ep_ptr->DCI);
	poll_ep_stopped(nvudc->dev, "stream_rejected", u_temp);
	iowrite32(u_temp, nvudc->mmio_reg_base + EP_STOPPED);

	if (!list_empty(&udc_ep_ptr->queue)) {
		u_temp = udc_ep_ptr->DCI;
		u_temp = DB_TARGET(u_temp);
		iowrite32(u_temp, nvudc->mmio_reg_base + DB);
		msg_dbg(nvudc->dev, "DOORBELL STREAM = 0x%x\n", u_temp);
	}
}
#endif

void handle_cmpl_code_success(struct NV_UDC_S *nvudc, struct EVENT_TRB_S *event,
	struct NV_UDC_EP *udc_ep_ptr)
{
	u64 trb_pt;
	struct TRANSFER_TRB_S *p_trb;
	struct NV_UDC_REQUEST *udc_req_ptr;
	u32 trb_transfer_length;

	msg_entry(nvudc->dev);
	trb_pt = (u64)event->trb_pointer_lo +
			((u64)(event->trb_pointer_hi) << 32);
	p_trb = tran_trb_dma_to_virt(udc_ep_ptr, trb_pt);
	msg_dbg(nvudc->dev, "trb_pt = %lx, p_trb = %p\n",
			(unsigned long)trb_pt, p_trb);

	if (!XHCI_GETF(TRB_CHAIN_BIT, p_trb->trb_dword3)) {
		/* chain bit is not set, which means it
		* is the end of a TD */
		msg_dbg(nvudc->dev, "end of TD\n");
		udc_req_ptr = list_entry(udc_ep_ptr->queue.next,
					struct NV_UDC_REQUEST, queue);

		msg_dbg(nvudc->dev, "udc_req_ptr = 0x%p\n", udc_req_ptr);

		trb_transfer_length = XHCI_GETF(EVE_TRB_TRAN_LEN,
					event->eve_trb_dword2);
		udc_req_ptr->usb_req.actual = udc_req_ptr->usb_req.length -
					trb_transfer_length;
		msg_dbg(nvudc->dev, "Actual data xfer = 0x%x, tx_len = 0x%x\n",
			udc_req_ptr->usb_req.actual, trb_transfer_length);

		req_done(udc_ep_ptr, udc_req_ptr, 0);

		if (!udc_ep_ptr->desc) {
			msg_dbg(nvudc->dev, "udc_ep_ptr->desc is NULL\n");
		} else {
			if (usb_endpoint_xfer_control(udc_ep_ptr->desc))
				ep0_req_complete(udc_ep_ptr);
		}
	}
}

int nvudc_handle_exfer_event(struct NV_UDC_S *nvudc, struct EVENT_TRB_S *event)
{
	u8 ep_index = XHCI_GETF(EVE_TRB_ENDPOINT_ID, event->eve_trb_dword3);
	struct NV_UDC_EP *udc_ep_ptr = &nvudc->udc_ep[ep_index];
	struct EP_CX_S *p_ep_cx = nvudc->p_epcx + ep_index;
	u16 comp_code;
	struct NV_UDC_REQUEST *udc_req_ptr;
	bool trbs_dequeued = false;

	msg_entry(nvudc->dev);
	if (!udc_ep_ptr->tran_ring_ptr || XHCI_GETF(EP_CX_EP_STATE,
				p_ep_cx->ep_dw0) == EP_STATE_DISABLED)
		return -ENODEV;

	comp_code = XHCI_GETF(EVE_TRB_COMPL_CODE, event->eve_trb_dword2);


	switch (comp_code) {
	case CMPL_CODE_SUCCESS:
	{
#ifdef PRIME_NOT_RCVD_WAR
		u32 u_temp;
		u_temp = NV_BIT(ep_index);
		/* When the stream is being rejected by the Host and then
		 * a valid packet is exchanged on this pipe, we clear the
		 * stream rejected condition and cancel any pending work that
		 * was scheduled to retry the stream
		 */
		if (nvudc->stream_rejected & u_temp) {
			nvudc->stream_rejected &= ~u_temp;
			cancel_delayed_work(&udc_ep_ptr->work);
			udc_ep_ptr->stream_rejected_retry_count = 0;
		}
#endif
		handle_cmpl_code_success(nvudc, event, udc_ep_ptr);
		trbs_dequeued = true;
		break;
	}
	case CMPL_CODE_SHORT_PKT:
	{
		u32 trb_transfer_length;

#ifdef PRIME_NOT_RCVD_WAR
		u32 u_temp;
		u_temp = NV_BIT(ep_index);
		/* When the stream is being rejected by the Host and then
		 * a valid packet is exchanged on this pipe, we clear the
		 * stream rejected condition and cancel any pending work that
		 * was scheduled to retry the stream
		 */
		if (nvudc->stream_rejected & u_temp) {
			nvudc->stream_rejected &= ~u_temp;
			cancel_delayed_work(&udc_ep_ptr->work);
			udc_ep_ptr->stream_rejected_retry_count = 0;
		}
#endif
		msg_dbg(nvudc->dev, "handle_exfer_event CMPL_CODE_SHORT_PKT\n");
		if (usb_endpoint_dir_out(udc_ep_ptr->desc)) {
			msg_dbg(nvudc->dev, "dir out EPIdx = 0x%x\n", ep_index);

			trb_transfer_length = XHCI_GETF(EVE_TRB_TRAN_LEN,
						event->eve_trb_dword2);
			udc_req_ptr = list_entry(udc_ep_ptr->queue.next,
						struct NV_UDC_REQUEST, queue);

			udc_req_ptr->usb_req.actual =
				udc_req_ptr->usb_req.length -
				trb_transfer_length;
			msg_dbg(nvudc->dev, "Actual Data transfered = 0x%x\n",
					udc_req_ptr->usb_req.actual);
				req_done(udc_ep_ptr, udc_req_ptr, 0);
		} else
			msg_dbg(nvudc->dev, "ep dir in\n");
		trbs_dequeued = true;
		break;
	}
	case CMPL_CODE_HOST_REJECTED:
	{
		u32 u_temp;
		msg_dbg(nvudc->dev, "CMPL_CODE_HOST_REJECTED\n");
		/* bug 1285731, 1376479*/
		/* PCD driver needs to block the ERDY until Prime Pipe
		Received. But Since currently the Host is not sending
		a PRIME packet when the stream is rejected, we are retrying
		the ERDY till some time before stopping and waiting for the
		PRIME*/
		u_temp = NV_BIT(ep_index);

#ifdef PRIME_NOT_RCVD_WAR
		if ((nvudc->stream_rejected & u_temp) &&
			(udc_ep_ptr->stream_rejected_retry_count
				>= max_stream_rejected_retry_count)) {

			/* We have tried retrying the stream for some time
			 * but the Host is not responding. So we stop
			 * retrying now and wait for the PRIME
			 */
			udc_ep_ptr->stream_rejected_retry_count = 0;
			msg_dbg(nvudc->dev, "Stream retry Limit reached\n");
		} else {
			nvudc->stream_rejected |= u_temp;
			udc_ep_ptr->stream_rejected_retry_count++;
			if (!schedule_delayed_work(&udc_ep_ptr->work, 0))
				msg_dbg(nvudc->dev,
				"Error occured in retrying stream\n");
		}
#else
		nvudc->stream_rejected |= u_temp;
#endif
		break;
	}
	case CMPL_CODE_PRIME_PIPE_RECEIVED:
	{
		u32 u_temp;
		msg_dbg(nvudc->dev, "CMPL_CODE_PRIME_PIPE_RECEIVED\n");
		u_temp = NV_BIT(ep_index);

		if (nvudc->stream_rejected & u_temp) {
			nvudc->stream_rejected &= ~u_temp;
#ifdef PRIME_NOT_RCVD_WAR
			udc_ep_ptr->stream_rejected_retry_count = 0;
			cancel_delayed_work(&udc_ep_ptr->work);
#endif
			/* When a stream is rejected for particular ep, its
			 * corresponding bit is set in the EP_STOPPED register
			 * The bit has to be cleared by writing 1 to it before
			 * sending an ERDY on the endpoint.
			 */
			poll_ep_stopped(nvudc->dev, "stream_rejected", u_temp);
			iowrite32(u_temp, nvudc->mmio_reg_base + EP_STOPPED);
		}

		/* re-ring the door bell if ERDY is pending */
		if (!list_empty(&udc_ep_ptr->queue)) {
			u_temp = udc_ep_ptr->DCI;
			u_temp = DB_TARGET(u_temp);
			iowrite32(u_temp, nvudc->mmio_reg_base + DB);
			msg_dbg(nvudc->dev, "DOORBELL STREAM = 0x%x\n", u_temp);
		}
			break;
	}
	case CMPL_CODE_BABBLE_DETECTED_ERR:
	{
		/* Race condition
		* When HW detects babble condition it generates a babble event
		* and flow controls the request from host, if at that time SW
		* was in proess of adding a new td to the transfer ring-
		* The doorbell for the new td could clear the flow control
		* condition and HW will attempt to resume transfer
		* by sending ERDY which is not desire.
		* Wait for STOPPED bit to be set by HW, so HW will stop
		* processing doorbells
		*/
		poll_ep_stopped(nvudc->dev, "cmpl_babble", NV_BIT(ep_index));
		iowrite32(NV_BIT(ep_index), nvudc->mmio_reg_base + EP_STOPPED);

		msg_dbg(nvudc->dev, "comp_babble_err\n");
		/* FALL THROUGH to halt endpoint */
	}
	case CMPL_CODE_STREAM_NUMP_ERROR:
	case CMPL_CODE_CTRL_DIR_ERR:
	case CMPL_CODE_INVALID_STREAM_TYPE_ERR:
	case CMPL_CODE_RING_UNDERRUN:
	case CMPL_CODE_RING_OVERRUN:
	case CMPL_CODE_ISOCH_BUFFER_OVERRUN:
	case CMPL_CODE_USB_TRANS_ERR:
	case CMPL_CODE_TRB_ERR:
	{
		u32 utemp;

		msg_dbg(nvudc->dev, "comp_code = 0x%x\n", comp_code);
		/* halt the endpoint */
		utemp = ioread32(nvudc->mmio_reg_base + EP_HALT);
		utemp |= NV_BIT(ep_index);
		iowrite32(utemp, nvudc->mmio_reg_base + EP_HALT);

		poll_stchg(nvudc->dev, "cmpl_code_err", NV_BIT(ep_index));

		iowrite32(NV_BIT(ep_index), nvudc->mmio_reg_base + EP_STCHG);

		break;
	}
	case CMPL_CODE_CTRL_SEQNUM_ERR:
	{
		u32 enq_idx = nvudc->ctrl_req_enq_idx;
		struct usb_ctrlrequest *setup_pkt;
		struct nv_setup_packet *nv_setup_pkt;
		u16 seq_num;

		msg_dbg(nvudc->dev, "CMPL_CODE_SEQNUM_ERR\n");

		udc_req_ptr = list_entry(udc_ep_ptr->queue.next,
					struct NV_UDC_REQUEST, queue);
		req_done(udc_ep_ptr, udc_req_ptr, -EINVAL);

		/*drop all the queued setup packet, only
		* process the latest one.*/
		nvudc->setup_status = WAIT_FOR_SETUP;
		if (enq_idx) {
			nv_setup_pkt = &nvudc->ctrl_req_queue[enq_idx - 1];
			setup_pkt = &nv_setup_pkt->usbctrlreq;
			seq_num = nv_setup_pkt->seqnum;
			/* flash the queue after the latest
			 * setup pkt got handled.. */
			memset(nvudc->ctrl_req_queue, 0,
				sizeof(struct usb_ctrlrequest)
				* CTRL_REQ_QUEUE_DEPTH);
			nvudc->ctrl_req_enq_idx = 0;
			nvudc_handle_setup_pkt(nvudc, setup_pkt, seq_num);
		}
		break;
	}
	case CMPL_CODE_STOPPED:
		/* stop transfer event for disconnect. */
		msg_dbg(nvudc->dev, "CMPL_CODE_STOPPED\n");
		msg_dbg(nvudc->dev, "EPDCI = 0x%x\n", udc_ep_ptr->DCI);
		udc_req_ptr = list_entry(udc_ep_ptr->queue.next,
				struct NV_UDC_REQUEST,
				queue);
		if (udc_req_ptr)
			req_done(udc_ep_ptr, udc_req_ptr, -ECONNREFUSED);

		break;
	default:
		msg_dbg(nvudc->dev, "comp_code = 0x%x\n", comp_code);
		msg_dbg(nvudc->dev, "EPDCI = 0x%x\n", udc_ep_ptr->DCI);
		break;
	}

	/* If there are some trbs dequeued by HW and the ring
	 * was full before, then schedule any pending TRB's
	 */
	if ((trbs_dequeued == true) && (udc_ep_ptr->tran_ring_full == true)) {
		udc_ep_ptr->tran_ring_full = false;
		queue_pending_trbs(udc_ep_ptr);
	}
	msg_exit(nvudc->dev);
	return 0;
}

bool setfeaturesrequest(struct NV_UDC_S *nvudc, u8 RequestType, u8 bRequest, u16
		value, u16 index, u16 length)
{
	int status = -EINPROGRESS;
	u8  DCI;
	struct NV_UDC_EP *udc_ep_ptr;
	u32 u_temp;
	bool set_feat = 0;

	if (length != 0) {
		status = -EINVAL;
		goto set_feature_error;
	}

	if (nvudc->device_state == USB_STATE_DEFAULT) {
		status = -EINVAL;
		goto set_feature_error;
	}

	set_feat = (bRequest == USB_REQ_SET_FEATURE) ? 1 : 0;
	if ((RequestType & (USB_RECIP_MASK | USB_TYPE_MASK)) ==
			(USB_RECIP_ENDPOINT | USB_TYPE_STANDARD)) {
		msg_dbg(nvudc->dev, "Halt/Unhalt EP\n");
		if (nvudc->device_state == USB_STATE_ADDRESS) {
			if (index != 0) {
				status = -EINVAL;
				goto set_feature_error;
			}
		}

		DCI = (index & USB_ENDPOINT_NUMBER_MASK)*2 + ((index &
					USB_DIR_IN) ? 1 : 0);
		udc_ep_ptr = &nvudc->udc_ep[DCI];
		msg_dbg(nvudc->dev, "halt/Unhalt endpoint DCI = 0x%x\n", DCI);

		status = ep_halt(udc_ep_ptr,
				(bRequest == USB_REQ_SET_FEATURE) ? 1 : 0);
		if (status < 0)
			goto set_feature_error;
	} else if ((RequestType & (USB_RECIP_MASK | USB_TYPE_MASK)) ==
			(USB_RECIP_DEVICE | USB_TYPE_STANDARD)) {
		switch (value) {
		case USB_DEVICE_REMOTE_WAKEUP:
			/* REMOTE_WAKEUP selector is not used by USB3.0 */
			if ((nvudc->device_state < USB_STATE_DEFAULT) ||
				(nvudc->gadget.speed == USB_SPEED_SUPER)) {
				status = -EINVAL;
				goto set_feature_error;
			}
			msg_dbg(nvudc->dev, "%s_Feature RemoteWake\n",
				set_feat ? "Set" : "Clear");
			u_temp = ioread32(nvudc->mmio_reg_base + PORTPM);
			u_temp &= ~PORTPM_RWE;
			u_temp |= set_feat << PORTPM_RWE_SHIFT;
			iowrite32(u_temp, nvudc->mmio_reg_base + PORTPM);
			break;
		case USB_DEVICE_U1_ENABLE:
		case USB_DEVICE_U2_ENABLE:
		{
			if (nvudc->device_state != USB_STATE_CONFIGURED) {
				status = -EINVAL;
				goto set_feature_error;
			}

			if (index & 0xff) {
				status = -EINVAL;
				goto set_feature_error;
			}

			/* disable U1/U2 by default */
			if (u1_u2_enable) {
				u_temp = ioread32(nvudc->mmio_reg_base
						+ PORTPM);

				if (value == USB_DEVICE_U1_ENABLE) {
					u_temp &= ~PORTPM_U1E;
					u_temp |= set_feat << PORTPM_U1E_SHIFT;
				}
				if (value == USB_DEVICE_U2_ENABLE) {
					u_temp &= ~PORTPM_U2E;
					u_temp |= set_feat << PORTPM_U2E_SHIFT;
				}

				iowrite32(u_temp, nvudc->mmio_reg_base
						+ PORTPM);
				msg_dbg(nvudc->dev, "PORTPM = 0x%x", u_temp);
			}
			break;

		}
		case USB_DEVICE_TEST_MODE:
		{
			u32 u_pattern;
			if (nvudc->gadget.speed != USB_SPEED_HIGH)
				goto set_feature_error;

			if (nvudc->device_state < USB_STATE_DEFAULT)
				goto set_feature_error;

			u_pattern = index >> 8;
			iowrite32(u_pattern, nvudc->mmio_reg_base
				+ HSFSPI_TESTMODE_CTRL);
			break;
		}
#ifdef OTG
		case USB_DEVICE_B_HNP_ENABLE:
			nvudc->gadget.b_hnp_enable = 1;
			nv_notify_otg(B_HNP_ENABLE);
			break;
		case USB_DEVICE_A_HNP_SUPPORT:
			nvudc->gadget.a_hnp_support = 1;
			break;
		case USB_DEVICE_A_ALT_HNP_SUPPORT:
			nvudc->gadget.a_alt_hnp_support = 1;
			break;
#endif
		default:
			goto set_feature_error;
		}

	} else if ((RequestType & (USB_RECIP_MASK | USB_TYPE_MASK)) ==
			(USB_RECIP_INTERFACE | USB_TYPE_STANDARD)) {
		if (nvudc->device_state != USB_STATE_CONFIGURED) {
			status = -EINVAL;
			goto set_feature_error;
		}

		/* Suspend Option */
		if (value == USB_INTRF_FUNC_SUSPEND) {
			if (index & USB_INTR_FUNC_SUSPEND_OPT_MASK &
				USB_INTRF_FUNC_SUSPEND_LP) {
				u_temp = ioread32(nvudc->mmio_reg_base
						+ PORTPM);
				if (index & USB_INTRF_FUNC_SUSPEND_RW) {
					msg_dbg(nvudc->dev,
						"Enable Func Remote Wakeup\n");
					u_temp |= PORTPM_FRWE;
				} else {
					msg_dbg(nvudc->dev,
						"Disable Func RemoteWakeup\n");
					u_temp &= ~PORTPM_FRWE;
				}
				iowrite32(u_temp, nvudc->mmio_reg_base
						+ PORTPM);
				/* Do not need to return status stage here */
				/* Pass to composite gadget driver to process
				the request */
				return false;
			}
		}
	}

	nvudc->setup_status = STATUS_STAGE_XFER;
	build_ep0_status(&nvudc->udc_ep[0], true, status, NULL);
	return true;

set_feature_error:
	u_temp = ioread32(nvudc->mmio_reg_base + EP_HALT);
	u_temp |= 1;
	iowrite32(u_temp, nvudc->mmio_reg_base + EP_HALT);

	msg_dbg(nvudc->dev, "halt the control endpoint 0x%x\n",
			u_temp);
	poll_stchg(nvudc->dev, "set_feature():", 1);
				;
	iowrite32(1, nvudc->mmio_reg_base + EP_STCHG);

	nvudc->setup_status = WAIT_FOR_SETUP;
	return true;

}

void getstatusrequest(struct NV_UDC_S *nvudc, u8 RequestType, u16 value,
		u16 index, u16 length)
{
	u64 temp = 0;
	u32 temp2;
	u32 status = -EINPROGRESS;
	struct NV_UDC_REQUEST *udc_req_ptr = nvudc->status_req;
	struct NV_UDC_EP *udc_ep_ptr;

	if ((value) || (length > 2) || !length) {
		status = -EINVAL;
		goto get_status_error;
	}

	msg_dbg(nvudc->dev, "Get status request RequestType = 0x%x\n",
			RequestType);
	if ((RequestType & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		msg_dbg(nvudc->dev, "Get status request Device request\n");
		if (index == 0xF000) {
			if (IOCTL_HOST_FLAG_ENABLED) {
#define USB_HOST_REQUEST_FLAG   1
				temp = NV_BIT(USB_HOST_REQUEST_FLAG);
			}
		} else if (index) {
			status = -EINVAL;
			goto get_status_error;
		}

		temp2 = ioread32(nvudc->mmio_reg_base + PORTPM);

		if (nvudc->gadget.speed == USB_SPEED_HIGH ||
			nvudc->gadget.speed == USB_SPEED_FULL) {
			if (PORTPM_RWE & temp2)
				temp |= NV_BIT(USB_DEVICE_REMOTE_WAKEUP);
		}

		if (nvudc->gadget.speed == USB_SPEED_SUPER) {
			if (PORTPM_U1E & temp2)
				temp |= (u64)NV_BIT(USB_DEV_STAT_U1_ENABLED);
			if (PORTPM_U2E & temp2)
				temp |= (u64)NV_BIT(USB_DEV_STAT_U2_ENABLED);
		}
		msg_dbg(nvudc->dev, "Status = 0x%llx\n", temp);

	} else if ((RequestType & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {

		msg_dbg(nvudc->dev, "Get status request Interface request\n");
		if (nvudc->gadget.speed == USB_SPEED_SUPER) {
			temp2 = ioread32(nvudc->mmio_reg_base + PORTPM);
			temp = USB_INTRF_STAT_FUNC_RW_CAP;
			if (PORTPM_FRWE & temp2)
				temp |= USB_INTRF_STAT_FUNC_RW;
			msg_dbg(nvudc->dev, "Status = 0x%llx\n", temp);
		} else
			temp = 0;
	} else if ((RequestType & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {

		u32 DCI = (index & USB_ENDPOINT_NUMBER_MASK)*2 + ((index &
					USB_DIR_IN) ? 1 : 0);
		msg_dbg(nvudc->dev,
		"Get status request endpoint request DCI = 0x%x\n", DCI);
		/* if device state is address state, index should be 0
		if device state is configured state, index should be an
		endpoint configured.*/

		if ((nvudc->device_state == USB_STATE_ADDRESS) && (DCI != 0)) {
			status = -EINVAL;
			goto get_status_error;
		}

		if (nvudc->device_state == USB_STATE_CONFIGURED) {
			struct EP_CX_S *p_ep_cx =
				(struct EP_CX_S *)nvudc->p_epcx + DCI;
			msg_dbg(nvudc->dev, "p_ep_cx->EPDWord0 = 0x%x\n",
				p_ep_cx->ep_dw0);
			if (XHCI_GETF(EP_CX_EP_STATE, p_ep_cx->ep_dw0) ==
					EP_STATE_DISABLED) {
				status = -EINVAL;
				goto get_status_error;
			}

			/* EP_STATE is not synced with SW and HW. Use EP_HALT
			 * and EP_PAUSE register to check the EP state */
			if (ioread32(nvudc->mmio_reg_base + EP_HALT)
				& (1 << DCI)) {
				temp = NV_BIT(USB_ENDPOINT_HALT);
				msg_dbg(nvudc->dev,
					"endpoint was halted = 0x%lx\n",
					(long unsigned int)temp);
			}

		}
	}

get_status_error:
	if (status != -EINPROGRESS)
		udc_req_ptr->usb_req.length = 0;
	else {
		udc_req_ptr->usb_req.buf = &nvudc->statusbuf;
		*(u16 *)udc_req_ptr->usb_req.buf = cpu_to_le64(temp);
		msg_dbg(nvudc->dev, "usb_req.buf = 0x%x\n",
				*((u16 *)udc_req_ptr->usb_req.buf));
		udc_req_ptr->usb_req.length = 2;
	}
	udc_req_ptr->usb_req.status = status;
	udc_req_ptr->usb_req.actual = 0;
	udc_req_ptr->usb_req.complete = NULL;

	if (udc_req_ptr->usb_req.dma == DMA_ADDR_INVALID) {
		udc_req_ptr->usb_req.dma =
			dma_map_single(nvudc->gadget.dev.parent,
					udc_req_ptr->usb_req.buf,
					udc_req_ptr->usb_req.length,
					DMA_FROM_DEVICE);
		udc_req_ptr->mapped = 1;
	}


	udc_ep_ptr = &nvudc->udc_ep[0];

	nvudc->setup_status = DATA_STAGE_XFER;
	status = nvudc_build_td(udc_ep_ptr, udc_req_ptr);

	if (!status) {

		if (udc_req_ptr)
			list_add_tail(&udc_req_ptr->queue, &udc_ep_ptr->queue);
	}
}

void nvudc_set_sel_cmpl(struct usb_ep *ep, struct usb_request *req)
{
	struct NV_UDC_EP *udc_ep;
	struct NV_UDC_S	 *nvudc;

	udc_ep = container_of(ep, struct NV_UDC_EP, usb_ep);
	nvudc = udc_ep->nvudc;
	msg_entry(nvudc->dev);

	msg_dbg(nvudc->dev, "u1_sel_value = 0x%x, u2_sel_value = 0x%x\n",
			nvudc->sel_value.u1_sel_value,
			nvudc->sel_value.u2_sel_value);
	msg_exit(nvudc->dev);
}

void setselrequest(struct NV_UDC_S *nvudc, u16 value, u16 index, u16 length,
		u64 data)
{
	int status = -EINPROGRESS;
	struct NV_UDC_REQUEST *udc_req_ptr = nvudc->status_req;
	struct NV_UDC_EP *udc_ep_ptr = &nvudc->udc_ep[0];

	if (nvudc->device_state == USB_STATE_DEFAULT)
		status = -EINVAL;

	if ((index != 0) || (value != 0) || (length != 6))
		status = -EINVAL;

	if (status != -EINPROGRESS) {
		/* ??? send status stage?
		   udc_req_ptr->usb_req.length = 0; */
	} else {
		udc_req_ptr->usb_req.length = length;
		(nvudc->sel_value).u2_pel_value = 0;
		(nvudc->sel_value).u2_sel_value = 0;
		(nvudc->sel_value).u1_pel_value = 0;
		(nvudc->sel_value).u1_sel_value = 0;
		udc_req_ptr->usb_req.buf = &nvudc->sel_value;
	}

	udc_req_ptr->usb_req.status = -EINPROGRESS;
	udc_req_ptr->usb_req.actual = 0;
	udc_req_ptr->usb_req.complete = nvudc_set_sel_cmpl;

	if (udc_req_ptr->usb_req.dma == DMA_ADDR_INVALID) {
		udc_req_ptr->usb_req.dma =
			dma_map_single(nvudc->gadget.dev.parent,
					udc_req_ptr->usb_req.buf,
					udc_req_ptr->usb_req.length,
					DMA_TO_DEVICE);
		udc_req_ptr->mapped = 1;
	}

	status = nvudc_build_td(udc_ep_ptr, udc_req_ptr);

	if (!status) {

		if (udc_req_ptr)
			list_add_tail(&udc_req_ptr->queue, &udc_ep_ptr->queue);
	}
}

void set_isoch_delay(struct NV_UDC_S *nvudc, u16 value, u16 index, u16 length,
		u16 seq_num)
{
	int status = -EINPROGRESS;
	if ((value > 65535) || (index != 0) || (length != 0))
		status = -EINVAL;

	nvudc->iso_delay = value;
	nvudc->setup_status = STATUS_STAGE_XFER;
	build_ep0_status(&nvudc->udc_ep[0], true, status, NULL);
}

void set_address_cmpl(struct NV_UDC_S *nvudc)
{
	if ((nvudc->device_state == USB_STATE_DEFAULT) &&
				nvudc->dev_addr != 0) {
		nvudc->device_state = USB_STATE_ADDRESS;
		msg_dbg(nvudc->dev, "USB State Addressed\n");

	} else if (nvudc->device_state == USB_STATE_ADDRESS) {
		if (nvudc->dev_addr == 0)
			nvudc->device_state = USB_STATE_DEFAULT;
	}
}

void setaddressrequest(struct NV_UDC_S *nvudc, u16 value, u16 index, u16 length,
		u16 seq_num)
{
	int status = -EINPROGRESS;
	struct EP_CX_S *p_epcx = nvudc->p_epcx;
	u32 reg;

	if ((value > 127) || (index != 0) || (length != 0)) {
		status = -EINVAL;
		goto set_address_error;
	}

	if (((nvudc->device_state == USB_STATE_DEFAULT) && value != 0) ||
			(nvudc->device_state == USB_STATE_ADDRESS)) {
		nvudc->dev_addr = value;
		reg = ioread32(nvudc->mmio_reg_base + CTRL);
		reg &= ~CTRL_DEVADDR(-1);
		reg |= CTRL_DEVADDR(nvudc->dev_addr);
		iowrite32(reg, nvudc->mmio_reg_base + CTRL);

		XHCI_SETF_VAR(
			EP_CX_DEVADDR, p_epcx->ep_dw11, nvudc->dev_addr);
	} else
		status = -EINVAL;


set_address_error:
	msg_dbg(nvudc->dev, "build_ep0_status for Address Device\n");

	nvudc->setup_status = STATUS_STAGE_XFER;
	nvudc->setup_fn_call_back = &set_address_cmpl;
	build_ep0_status(&nvudc->udc_ep[0], true, status, NULL);
}


void nvudc_handle_setup_pkt(struct NV_UDC_S *nvudc,
		struct usb_ctrlrequest *setup_pkt, u16 seq_num)
{
	u16 wValue = setup_pkt->wValue;
	u16 wIndex = setup_pkt->wIndex;
	u16 wLength = setup_pkt->wLength;
	u64 wData = 0;
	u32 u_temp;

	nvudc->ctrl_seq_num = seq_num;

	/* clear halt for ep0 when new setup request coming. */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_HALT);
	if (u_temp & 1) {
		msg_dbg(nvudc->dev,
			"EP0 is halted when new setup request coming\n");
		u_temp &= ~1;
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_HALT);

		poll_stchg(nvudc->dev, "handle_setup_pkt():", 1);
		iowrite32(1, nvudc->mmio_reg_base + EP_STCHG);
	}

	msg_dbg(nvudc->dev,
		"bRequest=%d, wValue=0x%.4x, wIndex=%d, wLength=%d\n",
		setup_pkt->bRequest, wValue, wIndex, wLength);

	nvudc->setup_status = SETUP_PKT_PROCESS_IN_PROGRESS;
	nvudc->setup_fn_call_back = NULL;
	if ((setup_pkt->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (setup_pkt->bRequest) {
		case USB_REQ_GET_STATUS:
			msg_dbg(nvudc->dev, "USB_REQ_GET_STATUS\n");
			if ((setup_pkt->bRequestType & (USB_DIR_IN |
							USB_TYPE_MASK))
				!= (USB_DIR_IN | USB_TYPE_STANDARD)) {
				nvudc->setup_status = WAIT_FOR_SETUP;
				return;
			}

			getstatusrequest(nvudc, setup_pkt->bRequestType,
					wValue, wIndex, wLength);
			return;
		case USB_REQ_SET_ADDRESS:
			msg_dbg(nvudc->dev, "USB_REQ_SET_ADDRESS\n");
			if (setup_pkt->bRequestType != (USB_DIR_OUT |
						USB_RECIP_DEVICE |
						USB_TYPE_STANDARD)) {
				nvudc->setup_status = WAIT_FOR_SETUP;
				return;
			}

			setaddressrequest(nvudc, wValue, wIndex, wLength,
					seq_num);
			return;
		case USB_REQ_SET_SEL:
			msg_dbg(nvudc->dev, "USB_REQ_SET_SEL\n");

			if (setup_pkt->bRequestType != (USB_DIR_OUT |
						USB_RECIP_DEVICE |
						USB_TYPE_STANDARD)) {
				nvudc->setup_status = WAIT_FOR_SETUP;
				return;
			}

			nvudc->setup_status = DATA_STAGE_RECV;
			setselrequest(nvudc, wValue, wIndex, wLength, wData);
			return;
		case USB_REQ_SET_ISOCH_DELAY:
			if (setup_pkt->bRequestType != (USB_DIR_OUT |
						USB_RECIP_DEVICE |
						USB_TYPE_STANDARD))
				break;
			msg_dbg(nvudc->dev, "USB_REQ_SET_ISOCH_DELAY\n");
			set_isoch_delay(nvudc, wValue, wIndex, wLength,
					seq_num);
			return;

		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
			msg_dbg(nvudc->dev, "USB_REQ_CLEAR/SET_FEATURE\n");

			/* Need composite gadget driver
			to process the function remote wakeup request */
			if (setfeaturesrequest(nvudc, setup_pkt->bRequestType,
						setup_pkt->bRequest,
					wValue, wIndex, wLength)) {
				/* Get here if request has been processed. */
				return;
			}
			break;
		case USB_REQ_SET_CONFIGURATION:
			/* In theory we need to clear RUN bit before
			status stage of deconfig request sent.
			But seeing problem if we do it before all the
			endpoints belonging to the configuration
			disabled. */
			/* FALL THROUGH */
		default:
			msg_dbg(nvudc->dev,
			 "USB_REQ default bRequest=%d, bRequestType=%d\n",
			 setup_pkt->bRequest, setup_pkt->bRequestType);

#ifndef HUI_XXX
			if (setup_pkt->bRequest == 0xFF)
				nvudc->dbg_cnt1 = 1;
#endif
			break;
		}
	}


	if (wLength) {
		/* data phase from gadget like GET_CONFIGURATION
		   call the setup routine of gadget driver.
		   remember the request direction. */
		nvudc->setup_status =  (setup_pkt->bRequestType & USB_DIR_IN) ?
			DATA_STAGE_XFER :  DATA_STAGE_RECV;
	}

	spin_unlock(&nvudc->lock);
	if (nvudc->driver->setup(&nvudc->gadget, setup_pkt) < 0) {
		u32 u_temp;
		spin_lock(&nvudc->lock);
		msg_dbg(nvudc->dev, "setup request failed\n");
		u_temp = ioread32(nvudc->mmio_reg_base + EP_HALT);
		u_temp |= 1;
		iowrite32(u_temp, nvudc->mmio_reg_base + EP_HALT);

		msg_dbg(nvudc->dev, "halt the control endpoint 0x%x\n", u_temp);

		poll_stchg(nvudc->dev, "handle_setup_pkt():halting ep0", 1);
			iowrite32(1, nvudc->mmio_reg_base + EP_STCHG);
		msg_dbg(nvudc->dev, "cleaned EP STCHG for control ep 0x%x\n",
				ioread32(nvudc->mmio_reg_base + EP_STCHG));

		nvudc->setup_status = WAIT_FOR_SETUP;
		return;
	}
	spin_lock(&nvudc->lock);
}

void nvudc_reset(struct NV_UDC_S *nvudc)
{
	u32 i, u_temp;
	struct NV_UDC_EP *udc_ep_ptr;
	struct EP_CX_S *p_ep_cx;
	dma_addr_t dqptaddr;
	nvudc->setup_status = WAIT_FOR_SETUP;
	/* Base on Figure 9-1, default USB_STATE is attached */
	nvudc->device_state = USB_STATE_ATTACHED;

	msg_entry(nvudc->dev);

	/* clear pause for all the endpoints */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
	iowrite32(0, nvudc->mmio_reg_base + EP_PAUSE);
	poll_stchg(nvudc->dev, "nvudc_reset", u_temp);
	iowrite32(u_temp, nvudc->mmio_reg_base + EP_STCHG);

	msg_dbg(nvudc->dev, "EPSTCHG2 = 0x%x\n",
			ioread32(nvudc->mmio_reg_base + EP_STCHG));

	for (i = 2; i < 32; i++) {
		udc_ep_ptr = &nvudc->udc_ep[i];
		p_ep_cx = nvudc->p_epcx + i;

		if (udc_ep_ptr->desc)
			nuke(udc_ep_ptr, -ESHUTDOWN);
		udc_ep_ptr->tran_ring_full = false;
	}

	/* Complete any reqs on EP0 queue */
	udc_ep_ptr = &nvudc->udc_ep[0];
	nuke(udc_ep_ptr, -ESHUTDOWN);

	p_ep_cx = nvudc->p_epcx;

	/* Reset the sequence number and dequeue pointer
	   So HW can flush the transfer ring. */
	XHCI_SETF_VAR(EP_CX_SEQ_NUM, p_ep_cx->ep_dw5, 0);

	dqptaddr = tran_trb_virt_to_dma(udc_ep_ptr, udc_ep_ptr->enq_pt);

	p_ep_cx->ep_dw2 = lower_32_bits(dqptaddr);
	XHCI_SETF_VAR(
		EP_CX_DEQ_CYC_STATE, p_ep_cx->ep_dw2, udc_ep_ptr->pcs);
	p_ep_cx->ep_dw3 = upper_32_bits(dqptaddr);

	/* reload EP0 */
	iowrite32(1, nvudc->mmio_reg_base + EP_RELOAD);
	poll_reload(nvudc->dev, "nvudc_reset()", 1);

	/* clear the PAUSE bit */
	u_temp = ioread32(nvudc->mmio_reg_base + EP_PAUSE);
	u_temp &= ~1;
	iowrite32(u_temp, nvudc->mmio_reg_base + EP_PAUSE);

	poll_stchg(nvudc->dev, "nvudc_reset()", 1);

	iowrite32(1, nvudc->mmio_reg_base + EP_STCHG);

	nvudc->ctrl_req_enq_idx = 0;
	memset(nvudc->ctrl_req_queue, 0,
			sizeof(struct nv_setup_packet) * CTRL_REQ_QUEUE_DEPTH);
	if (nvudc->driver) {
		msg_dbg(nvudc->dev, "calling disconnect\n");
		spin_unlock(&nvudc->lock);
		nvudc->driver->disconnect(&nvudc->gadget);
		spin_lock(&nvudc->lock);
	}
	msg_exit(nvudc->dev);
}

void dbg_print_rings(struct NV_UDC_S *nvudc)
{
	u32 i;
	struct EVENT_TRB_S *temp_trb;
	msg_dbg(nvudc->dev, "Event Ring Segment 0\n");
	temp_trb = (struct EVENT_TRB_S *)nvudc->event_ring0.vaddr;
	for (i = 0; i < (EVENT_RING_SIZE + 5); i++) {
		msg_dbg(nvudc->dev, "0x%p: 0x%x, 0x%x, 0x%x, 0x%x\n",
				temp_trb, temp_trb->trb_pointer_lo,
				temp_trb->trb_pointer_hi,
				temp_trb->eve_trb_dword2,
				temp_trb->eve_trb_dword3);
		temp_trb = temp_trb + 1;
	}

	msg_dbg(nvudc->dev, "Event Ring Segment 1\n");
	temp_trb = (struct EVENT_TRB_S *)nvudc->event_ring1.vaddr;
	for (i = 0; i < (EVENT_RING_SIZE + 5); i++) {
		msg_dbg(nvudc->dev, "0x%p: 0x%x, 0x%x, 0x%x, 0x%x\n",
				temp_trb, temp_trb->trb_pointer_lo,
				temp_trb->trb_pointer_hi,
				temp_trb->eve_trb_dword2,
				temp_trb->eve_trb_dword3);
		temp_trb = temp_trb + 1;
	}
}

void dbg_print_ep_ctx(struct NV_UDC_S *nvudc)
{
	u32 i, j;
	struct EP_CX_S *p_epcx_temp = nvudc->p_epcx;
	struct TRANSFER_TRB_S *temp_trb1;

	msg_dbg(nvudc->dev, "ENDPOINT CONTEXT\n");
	for (i = 0; i < 32; i++) {
		if (p_epcx_temp->ep_dw0) {
			msg_dbg(nvudc->dev, "endpoint %d\n", i);
			msg_dbg(nvudc->dev, "0x%x 0x%x 0x%x 0x%x\n",
					p_epcx_temp->ep_dw0,
					p_epcx_temp->ep_dw1,
					p_epcx_temp->ep_dw2,
					p_epcx_temp->ep_dw3);
			msg_dbg(nvudc->dev, "0x%x 0x%x 0x%x 0x%x\n",
					p_epcx_temp->ep_dw4,
					p_epcx_temp->ep_dw5,
					p_epcx_temp->ep_dw6,
					p_epcx_temp->ep_dw7);
			msg_dbg(nvudc->dev, "0x%x 0x%x 0x%x 0x%x\n",
					p_epcx_temp->ep_dw8,
					p_epcx_temp->ep_dw9,
					p_epcx_temp->ep_dw10,
					p_epcx_temp->ep_dw11);
			msg_dbg(nvudc->dev, "0x%x 0x%x 0x%x 0x%x\n",
					p_epcx_temp->ep_dw12,
					p_epcx_temp->ep_dw13,
					p_epcx_temp->ep_dw14,
					p_epcx_temp->ep_dw15);

		}
		p_epcx_temp++;
	}

	msg_dbg(nvudc->dev, "Transfer ring for each endpoint\n");
	for (i = 0; i < 32; i++) {
		if (nvudc->udc_ep[i].tran_ring_ptr) {

			msg_dbg(nvudc->dev, "endpoint DCI = %d\n",
				nvudc->udc_ep[i].DCI);
			temp_trb1 =
			(struct TRANSFER_TRB_S *)nvudc->udc_ep[i].tran_ring_ptr;
			for (j = 0; j < NVUDC_BULK_EP_TD_RING_SIZE; j++) {
				msg_dbg(nvudc->dev,
					"0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
						(unsigned long)temp_trb1,
						temp_trb1->data_buf_ptr_lo,
						temp_trb1->data_buf_ptr_hi,
						temp_trb1->trb_dword2,
						temp_trb1->trb_dword3);
				temp_trb1++;

			}
		}
	}
}

static ssize_t debug_store(struct device *_dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct NV_UDC_S *nvudc = the_controller;

	if (sysfs_streq(buf, "show_evtr"))
		dbg_print_rings(nvudc);

	if (sysfs_streq(buf, "show_epc"))
		dbg_print_ep_ctx(nvudc);

	return size;
}
static DEVICE_ATTR(debug, S_IWUSR, NULL, debug_store);

static void portpm_config_war(struct NV_UDC_S *nvudc)
{
	u32 portsc_value = ioread32(nvudc->mmio_reg_base + PORTSC);
	u32 reg_portpm;

	if (!u1_u2_enable) {
		/* bug 1376335 */
		reg_portpm = ioread32(nvudc->mmio_reg_base + PORTPM);
		reg_portpm &= ~PORTPM_U1TIMEOUT(-1);
		reg_portpm &= ~PORTPM_U2TIMEOUT(-1);
		iowrite32(reg_portpm, nvudc->mmio_reg_base + PORTPM);
	}

	if (disable_lpm &&
		(((portsc_value >> PORTSC_PS_SHIFT) & PORTSC_PS_MASK) <= 3)) {
		reg_portpm = ioread32(nvudc->mmio_reg_base + PORTPM);
		reg_portpm &= ~PORTPM_L1S(-1);
		reg_portpm |= PORTPM_L1S(2);
		iowrite32(reg_portpm, nvudc->mmio_reg_base + PORTPM);
		msg_dbg(nvudc->dev, "XHCI_PORTPM = %x\n",
			ioread32(nvudc->mmio_reg_base + PORTPM));
	}
}

bool nvudc_handle_port_status(struct NV_UDC_S *nvudc)
{
	u32 u_temp, u_temp2;
	bool update_ptrs = true;

	u_temp = ioread32(nvudc->mmio_reg_base + PORTSC);
	msg_dbg(nvudc->dev, "PortSC= 0x%.8x\n", u_temp);

	if ((PORTSC_PRC & u_temp) &&
			(PORTSC_PR & u_temp)) {
		/* first port status change event for port reset*/
		msg_dbg(nvudc->dev, "PRC and PR both are set\n");
		/* clear PRC */
		u_temp2 = u_temp & PORTSC_MASK;
		u_temp2 |= PORTSC_PRC;
		u_temp2 |= PORTSC_PED;
		iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);
	}

	if ((PORTSC_PRC & u_temp) &&
			(!(PORTSC_PR & u_temp))) {
		msg_dbg(nvudc->dev, "PR completed. PRC is set, but PR is not\n");

		u_temp2 = (u_temp >> PORTSC_PS_SHIFT) & PORTSC_PS_MASK;
		if (u_temp2 <= 2)
			nvudc->gadget.speed = USB_SPEED_FULL;
		else if (u_temp2 == 3)
			nvudc->gadget.speed = USB_SPEED_HIGH;
		else
			nvudc->gadget.speed = USB_SPEED_SUPER;
		msg_dbg(nvudc->dev, "gadget speed = 0x%x\n",
				nvudc->gadget.speed);

		nvudc_reset(nvudc);
		u_temp2 = u_temp & PORTSC_MASK;
		u_temp2 |= PORTSC_PRC;
		u_temp2 |= PORTSC_PED;
		iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);
		nvudc->device_state = USB_STATE_DEFAULT;

		msg_dbg(nvudc->dev, "PORTSC = 0x%x\n",
				ioread32(nvudc->mmio_reg_base + PORTSC));

		portpm_config_war(nvudc);

		/* clear run change bit to enable door bell */
		u_temp2 = ioread32(nvudc->mmio_reg_base + ST);
		if (ST_RC & u_temp2)
			iowrite32(ST_RC, nvudc->mmio_reg_base + ST);
	}

	u_temp = ioread32(nvudc->mmio_reg_base + PORTHALT);
	if (PORTHALT_STCHG_REQ & u_temp) {

		msg_dbg(nvudc->dev, "STCHG_REQ is set, port_halt_reg=0x%.8x\n",
				u_temp);

#define PORTHALT_MASK   0x071F0003
		u_temp2 = u_temp & PORTHALT_MASK;
		u_temp2 |= PORTHALT_STCHG_REQ;

		if (PORTHALT_HALT_LTSSM & u_temp2) {
			u_temp2 &= ~PORTHALT_HALT_LTSSM;
			iowrite32(u_temp2, nvudc->mmio_reg_base + PORTHALT);
			msg_dbg(nvudc->dev,
				"STCHG_REQ_CLEAR, PORTSCHALT = 0x%x\n",
				(u32)ioread32(nvudc->mmio_reg_base + PORTHALT));
		}
	}

	u_temp = ioread32(nvudc->mmio_reg_base + PORTSC);
	if ((PORTSC_WRC & u_temp) && (PORTSC_WPR & u_temp)) {
		/* first port status change event for port reset*/
		msg_dbg(nvudc->dev, "WRC and WPR both are set\n");
		/* clear WRC */
		u_temp2 = u_temp & PORTSC_MASK;
		u_temp2 |= PORTSC_WRC;
		u_temp2 |= PORTSC_PED;
		iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);

	}

	if ((PORTSC_WRC & u_temp) &&
			(!(PORTSC_WPR & u_temp))) {

		msg_dbg(nvudc->dev, "WRC is set, but WPR is not\n");
		nvudc_reset(nvudc);
		u_temp2 = u_temp & PORTSC_MASK;
		u_temp2 |= PORTSC_WRC;
		u_temp2 |= PORTSC_PED;
		iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);
		nvudc->device_state = USB_STATE_DEFAULT;

		msg_dbg(nvudc->dev, "PORTSC = 0x%x\n",
				ioread32(nvudc->mmio_reg_base + PORTSC));

		portpm_config_war(nvudc);

		/* clear run change bit to enable door bell */
		u_temp2 = ioread32(nvudc->mmio_reg_base + ST);
		if (ST_RC & u_temp2) {
			msg_dbg(nvudc->dev, "clear RC\n");
			iowrite32(ST_RC, nvudc->mmio_reg_base + ST);
		}

	}

	if (PORTSC_CSC & u_temp) {

		msg_dbg(nvudc->dev, "CSC is set\n");
		if (PORTSC_CCS & u_temp) {

			msg_dbg(nvudc->dev, "CCS is set\n");
			u_temp2 = (u_temp >> PORTSC_PS_SHIFT) & PORTSC_PS_MASK;
			if (u_temp2 <= 2)
				nvudc->gadget.speed = USB_SPEED_FULL;
			else if (u_temp2 == 3)
				nvudc->gadget.speed = USB_SPEED_HIGH;
			else
				nvudc->gadget.speed = USB_SPEED_SUPER;
			msg_dbg(nvudc->dev, "gadget speed = 0x%x\n",
					nvudc->gadget.speed);
			nvudc->device_state = USB_STATE_DEFAULT;
			nvudc->setup_status = WAIT_FOR_SETUP;
			update_ep0_maxpacketsize(nvudc);

			portpm_config_war(nvudc);

#ifdef OTG_XXX
			nv_notify_otg(A_CONN);
#endif
		} else {
			msg_dbg(nvudc->dev, "disconnect\n");
			nvudc_reset(nvudc);

			/* clear run change bit to enable door bell */
			u_temp2 = ioread32(nvudc->mmio_reg_base + ST);
			if (ST_RC & u_temp2)
				iowrite32(ST_RC, nvudc->mmio_reg_base + ST);
		}

		u_temp2 = ioread32(nvudc->mmio_reg_base + PORTSC);
		u_temp2 = u_temp2 & PORTSC_MASK;
		u_temp2 |= PORTSC_CSC;
		iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);

	}

	if (PORTSC_PLC & u_temp) {

		msg_dbg(nvudc->dev, "PLC is set PORTSC= 0x%x\n",
				u_temp);

		if (((u_temp >> PORTSC_PLS_SHIFT) & PORTSC_PLS_MASK) == 3) {
			msg_dbg(nvudc->dev, "PLS Suspend (U3)\n");
			nvudc->resume_state = nvudc->device_state;
			nvudc->device_state = USB_STATE_SUSPENDED;
			if (nvudc->driver->suspend) {
				spin_unlock(&nvudc->lock);
				nvudc->driver->suspend(&nvudc->gadget);
				spin_lock(&nvudc->lock);
			}
			u_temp2 = ioread32(nvudc->mmio_reg_base + PORTSC);
			u_temp2 = u_temp2 & PORTSC_MASK;
			u_temp2 |= PORTSC_PLC;
			iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);

#ifdef OTG_XXX
			nv_notify_otg(A_BUS_SUSPEND);
#endif
		} else if ((((((u_temp >> PORTSC_PLS_SHIFT) & PORTSC_PLS_MASK)
			== 0xF) && (nvudc->gadget.speed == USB_SPEED_SUPER)) ||
			((((u_temp >> PORTSC_PLS_SHIFT) & PORTSC_PLS_MASK) == 0)
			&& (nvudc->gadget.speed < USB_SPEED_SUPER))) &&
			 (nvudc->resume_state != 0)) {

			msg_dbg(nvudc->dev, "PLS Resume\n");

			nvudc_resume_state(nvudc);

			if (nvudc->driver->resume) {
				spin_unlock(&nvudc->lock);
				nvudc->driver->resume(&nvudc->gadget);
				spin_lock(&nvudc->lock);
			}
#ifdef OTG_XXX
			nv_notify_otg(A_BUS_RESUME);
#endif
		} else {
			u_temp2 = ioread32(nvudc->mmio_reg_base + PORTSC);
			u_temp2 = u_temp2 & PORTSC_MASK;
			u_temp2 |= PORTSC_PLC;
			iowrite32(u_temp2, nvudc->mmio_reg_base + PORTSC);
		}
	}

	return update_ptrs;
}

void process_event_ring(struct NV_UDC_S *nvudc)
{
	struct EVENT_TRB_S *event;
	nvudc->num_evts_processed = 0;
	while (nvudc->evt_dq_pt) {
		event = (struct EVENT_TRB_S *)nvudc->evt_dq_pt;

		if (XHCI_GETF(EVE_TRB_CYCLE_BIT, event->eve_trb_dword3) !=
				nvudc->CCS)
			break;
		else {
			nvudc->num_evts_processed++;
			nvudc_handle_event(nvudc, event);

			if (event == nvudc->evt_seg0_last_trb)
				nvudc->evt_dq_pt = nvudc->event_ring1.vaddr;
			else if (event == nvudc->evt_seg1_last_trb) {
				msg_dbg(nvudc->dev,
					"evt_seg1_last_trb = 0x%p\n",
					nvudc->evt_seg1_last_trb);
				msg_dbg(nvudc->dev,
					"evt_dq_pt = 0x%p\n", nvudc->evt_dq_pt);
				nvudc->CCS = nvudc->CCS ? 0 : 1;
				nvudc->evt_dq_pt = nvudc->event_ring0.vaddr;
				msg_dbg(nvudc->dev,
				"wrap Event dq_pt to Event ring segment 0\n");
			} else
				nvudc->evt_dq_pt++;
		}
	}
}

void queue_setup_pkt(struct NV_UDC_S *nvudc, struct usb_ctrlrequest *setup_pkt,
			u16 seq_num)
{
	msg_entry(nvudc->dev);
	if (nvudc->ctrl_req_enq_idx == CTRL_REQ_QUEUE_DEPTH) {
		msg_dbg(nvudc->dev, "ctrl request queque is full\n");
		return;
	}

	memcpy(&(nvudc->ctrl_req_queue[nvudc->ctrl_req_enq_idx].usbctrlreq),
			setup_pkt, sizeof(struct usb_ctrlrequest));
	nvudc->ctrl_req_queue[nvudc->ctrl_req_enq_idx].seqnum = seq_num;
	nvudc->ctrl_req_enq_idx++;
	msg_exit(nvudc->dev);
}

void nvudc_handle_event(struct NV_UDC_S *nvudc, struct EVENT_TRB_S *event)
{

	msg_entry(nvudc->dev);

	switch (XHCI_GETF(EVE_TRB_TYPE, event->eve_trb_dword3)) {
	case TRB_TYPE_EVT_PORT_STATUS_CHANGE:
		nvudc_handle_port_status(nvudc);
		break;
	case TRB_TYPE_EVT_TRANSFER:
		nvudc_handle_exfer_event(nvudc, event);
		break;
	case TRB_TYPE_EVT_SETUP_PKT:
		{
			struct usb_ctrlrequest *setup_pkt;
			u16 seq_num;
			msg_dbg(nvudc->dev, "nvudc_handle_setup_pkt\n");

			setup_pkt = (struct usb_ctrlrequest *)
					&event->trb_pointer_lo;

			seq_num = XHCI_GETF(EVE_TRB_SEQ_NUM,
					event->eve_trb_dword2);
			msg_dbg(nvudc->dev, "setup_pkt = 0x%p, seq_num = %d\n",
				setup_pkt, seq_num);
			if (nvudc->setup_status != WAIT_FOR_SETUP) {
				/*previous setup packet hasn't
				completed yet.save the new one in a
				software queue.*/
				queue_setup_pkt(nvudc, setup_pkt, seq_num);
				break;
			}
			nvudc_handle_setup_pkt(nvudc, setup_pkt, seq_num);

			break;
		}
	default:
		msg_dbg(nvudc->dev, "TRB_TYPE = 0x%x",
			XHCI_GETF(EVE_TRB_TYPE, event->eve_trb_dword3));
		break;
	}

	msg_exit(nvudc->dev);
}

static irqreturn_t nvudc_irq(int irq, void *_udc)
{
	struct NV_UDC_S *nvudc = (struct NV_UDC_S *)_udc;
	u32 u_temp;
	unsigned long flags;
	dma_addr_t erdp;
	u32 reg;

	u_temp = ioread32(nvudc->mmio_reg_base + ST);
	if (!(ST_IP & u_temp))
		return IRQ_NONE;

	msg_entry(nvudc->dev);
	msg_dbg(nvudc->dev, "Download_events nvudc->evt_dq_pt = 0x%p\n",
				nvudc->evt_dq_pt);

	spin_lock_irqsave(&nvudc->lock, flags);

	u_temp = 0;
	u_temp |= ST_IP;
	iowrite32(u_temp, nvudc->mmio_reg_base + ST);

	process_event_ring(nvudc);

	/* update dequeue pointer */
	erdp = event_trb_virt_to_dma(nvudc, nvudc->evt_dq_pt);
	reg =  upper_32_bits(erdp);
	iowrite32(reg, nvudc->mmio_reg_base + ERDPHI);
	reg = lower_32_bits(erdp);
	reg |= ERDPLO_EHB;
	iowrite32(reg, nvudc->mmio_reg_base + ERDPLO);

	spin_unlock_irqrestore(&nvudc->lock, flags);

	msg_exit(nvudc->dev);
	return IRQ_HANDLED;
}


void usbep_struct_setup(struct NV_UDC_S *nvudc, u32 index, u8 *name)
{
	struct NV_UDC_EP *ep = &nvudc->udc_ep[index];
	ep->DCI = index;

	if (index) {
		strcpy(ep->name, name);
		ep->usb_ep.name = ep->name;
		ep->usb_ep.maxpacket = 1024;
		ep->usb_ep.max_streams = 16;
	} else {

		ep->usb_ep.name = "ep0";
		ep->usb_ep.maxpacket = 64;
	}
	msg_dbg(nvudc->dev, "ep = %p, ep name = %s maxpacket = 0x%x\n",
			ep, ep->name, ep->usb_ep.maxpacket);
	ep->usb_ep.ops = &nvudc_ep_ops;
	ep->nvudc = nvudc;
#ifdef PRIME_NOT_RCVD_WAR
	ep->stream_rejected_retry_count = 0;
	INIT_DELAYED_WORK(&ep->work, retry_stream_rejected_work);
#endif
	INIT_LIST_HEAD(&ep->queue);
	if (index)
		list_add_tail(&ep->usb_ep.ep_list, &nvudc->gadget.ep_list);
}

static int nvudc_gadget_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct NV_UDC_S *nvudc = the_controller;
	unsigned long flags;
	int retval = -ENODEV;
	u32 u_temp = 0;
	u32 i;

	if (!nvudc)
		return -ENODEV;

	msg_entry(nvudc->dev);
	if (!driver || (driver->max_speed < USB_SPEED_FULL)
			|| !driver->setup || !driver->disconnect)
		return -EINVAL;

	msg_dbg(nvudc->dev, "Sanity check. speed = 0x%x\n",
			driver->max_speed);

	msg_dbg(nvudc->dev, "nvudc->driver = %p, driver = %p\n",
			nvudc->driver, driver);
	if (nvudc->driver)
		return -EBUSY;
	msg_dbg(nvudc->dev, "nvudc->driver is not assgined.\n");

	spin_lock_irqsave(&nvudc->lock, flags);
	driver->driver.bus = NULL;
	nvudc->driver = driver;
	nvudc->gadget.dev.driver = &driver->driver;
	spin_unlock_irqrestore(&nvudc->lock, flags);
	msg_dbg(nvudc->dev, "bind\n");

	for (i = 1; i < NUM_EP_CX/2; i++) {
		u8 name[14];
		sprintf(name, "ep%dout", i);
		usbep_struct_setup(nvudc, i*2, name);
		sprintf(name, "ep%din", i);
		usbep_struct_setup(nvudc, i*2+1, name);
	}

	retval = device_create_file(nvudc->dev, &dev_attr_debug);
	if (retval)
		goto err_unbind;

	nvudc->device_state = USB_STATE_ATTACHED;
	nvudc->setup_status = WAIT_FOR_SETUP;

	set_interrupt_moderation(nvudc, min_irq_interval_us);

	u_temp = ioread32(nvudc->mmio_reg_base + CTRL);
	u_temp |= CTRL_IE;
	u_temp |= CTRL_LSE;
	iowrite32(u_temp, nvudc->mmio_reg_base + CTRL);

	u_temp = ioread32(nvudc->mmio_reg_base + PORTHALT);
	u_temp = u_temp & PORTHALT_MASK;
	u_temp |= PORTHALT_STCHG_INTR_EN;
	iowrite32(u_temp, nvudc->mmio_reg_base + PORTHALT);


	/* Enable clock gating */
	/* T210 WAR, Disable BLCG CORE FE */
	iowrite32(0xFFFB, nvudc->mmio_reg_base + BLCG);

	if (nvudc->pullup) {
		/* set ENABLE bit */
		u_temp = ioread32(nvudc->mmio_reg_base + CTRL);
		u_temp |= CTRL_ENABLE;
		iowrite32(u_temp, nvudc->mmio_reg_base + CTRL);
	}

	/* update vbus status */
	extcon_notifications(&nvudc->vbus_extcon_nb, 0, NULL);

	/* update id status */
	extcon_id_notifications(&nvudc->id_extcon_nb, 0, NULL);

#ifdef OTG
	if (nvudc->transceiver)
		retval = otg_set_peripheral(nvudc->transceiver, &nvudc->gadget);
#endif
	msg_exit(nvudc->dev);
	return 0;
err_unbind:
	nvudc->gadget.dev.driver = NULL;
	nvudc->driver = NULL;
	msg_exit(nvudc->dev);
	return retval;
}

static int nvudc_gadget_stop(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	u32 u_temp;
	struct NV_UDC_S *nvudc = the_controller;
	unsigned long flags;

	if (!nvudc)
		return -ENODEV;

	msg_entry(nvudc->dev);

	if (!driver || driver != nvudc->driver || !driver->unbind)
		return -EINVAL;

	spin_lock_irqsave(&nvudc->lock, flags);
	u_temp = ioread32(nvudc->mmio_reg_base + CTRL);
	u_temp &= ~CTRL_IE;
	u_temp &= ~CTRL_ENABLE;
	iowrite32(u_temp, nvudc->mmio_reg_base + CTRL);

	nvudc_reset(nvudc);

	/* clear ENABLE bit */
	u_temp = ioread32(nvudc->mmio_reg_base + CTRL);
	u_temp &= ~CTRL_ENABLE;
	iowrite32(u_temp, nvudc->mmio_reg_base + CTRL);

	reset_data_struct(nvudc);

	spin_unlock_irqrestore(&nvudc->lock, flags);
	driver->unbind(&nvudc->gadget);
	nvudc->gadget.dev.driver = NULL;
	nvudc->driver = NULL;
	nvudc->gadget.max_speed = USB_SPEED_UNKNOWN;

#ifdef OTG
	if (nvudc->transceiver)
		otg_set_peripheral(nvudc->transceiver, NULL);
#endif
	device_remove_file(nvudc->dev, &dev_attr_debug);
	return 0;
}

u32 init_hw_event_ring(struct NV_UDC_S *nvudc)
{
	u32 u_temp, buff_length;
	int retval = 0;
	dma_addr_t mapping;

	msg_entry(nvudc->dev);

#define EVENT_RING_SEGMENT_TABLE_SIZE   2
	/* One event ring segment won't work on T210 HW */
	u_temp = 0;
	u_temp |= SPARAM_ERSTMAX(EVENT_RING_SEGMENT_TABLE_SIZE);
	iowrite32(u_temp, nvudc->mmio_reg_base + SPARAM);

	u_temp = 0;
	u_temp |= ERSTSZ_ERST0SZ(EVENT_RING_SIZE);
	u_temp |= ERSTSZ_ERST1SZ(EVENT_RING_SIZE);
	iowrite32(u_temp, nvudc->mmio_reg_base + ERSTSZ);

	buff_length = EVENT_RING_SIZE * sizeof(struct EVENT_TRB_S);
	if (!(nvudc->event_ring0).vaddr) {
		(nvudc->event_ring0).vaddr =
			dma_alloc_coherent(nvudc->dev, buff_length,
						&mapping, GFP_ATOMIC);

		if ((nvudc->event_ring0).vaddr == NULL) {
			retval = -ENOMEM;
			return retval;
		}
	} else
		mapping = nvudc->event_ring0.dma;

	(nvudc->event_ring0).len = buff_length;
	(nvudc->event_ring0).dma = mapping;

	u_temp = 0;
	u_temp = lower_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ERST0BALO);

	u_temp = upper_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ERST0BAHI);

	buff_length = EVENT_RING_SIZE * sizeof(struct EVENT_TRB_S);
	if (!nvudc->event_ring1.vaddr) {
		(nvudc->event_ring1).vaddr =
			dma_alloc_coherent(nvudc->dev, buff_length,
						&mapping, GFP_ATOMIC);
		if ((nvudc->event_ring1).vaddr == NULL) {
			retval = -ENOMEM;
			return retval;
		}
	} else
		mapping = (nvudc->event_ring1).dma;

	(nvudc->event_ring1).len = buff_length;
	(nvudc->event_ring1).dma = mapping;

	u_temp = 0;
	u_temp = lower_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ERST1BALO);

	u_temp = upper_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ERST1BAHI);

	return 0;
}

static void
fpga_hack_setup_vbus_sense_and_termination(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	u32 reg;

	reg = 0x00211040;
	iowrite32(reg, IO_ADDRESS(0x7009f000 + XUSB_VBUS));

	reg = 0x00215040;
	iowrite32(reg, IO_ADDRESS(0x7009f000 + XUSB_VBUS));

	reg = 0x3080;
	iowrite32(reg, nvudc->base + TERMINATION_2);

	reg = 0x1012E;
	iowrite32(reg, nvudc->base + TERMINATION_1);

	reg = 0x1000;
	iowrite32(reg, nvudc->base + TERMINATION_1);
}

u32 reset_data_struct(struct NV_UDC_S *nvudc)
{
	int retval = 0;
	u32 u_temp = 0, buff_length = 0;
	dma_addr_t mapping;

	msg_entry(nvudc->dev);
	u_temp = ioread32(nvudc->mmio_reg_base + CTRL);

	retval = init_hw_event_ring(nvudc);
	if (retval)
		return retval;

	buff_length = EVENT_RING_SIZE * sizeof(struct EVENT_TRB_S);
	memset((void *)nvudc->event_ring0.vaddr, 0, buff_length);

	nvudc->evt_dq_pt = (nvudc->event_ring0).vaddr;
	nvudc->CCS = 1;

	mapping = nvudc->event_ring0.dma;
	u_temp = lower_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ERDPLO);

	u_temp |= EREPLO_ECS;
	iowrite32(u_temp, nvudc->mmio_reg_base + EREPLO);
	msg_dbg(nvudc->dev, "XHCI_EREPLO = 0x%x\n", u_temp);

	u_temp = upper_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ERDPHI);
	iowrite32(u_temp, nvudc->mmio_reg_base + EREPHI);
	msg_dbg(nvudc->dev, "XHCI_EREPHI = 0x%x\n", u_temp);

	memset((void *)(nvudc->event_ring1.vaddr), 0, buff_length);

	nvudc->evt_seg0_last_trb =
		(struct EVENT_TRB_S *)(nvudc->event_ring0.vaddr)
		+ (EVENT_RING_SIZE - 1);
	nvudc->evt_seg1_last_trb =
		(struct EVENT_TRB_S *)(nvudc->event_ring1.vaddr)
		+ (EVENT_RING_SIZE - 1);

	buff_length = NUM_EP_CX * sizeof(struct EP_CX_S);

	if (!nvudc->ep_cx.vaddr) {
		(nvudc->ep_cx).vaddr =
			dma_alloc_coherent(nvudc->dev, buff_length,
						&mapping, GFP_ATOMIC);
		if  ((nvudc->ep_cx).vaddr == NULL) {
			retval = -ENOMEM;
			return retval;
		}
	} else {
		mapping = nvudc->ep_cx.dma;
	}

	nvudc->p_epcx = (nvudc->ep_cx).vaddr;
	(nvudc->ep_cx).len = buff_length;
	(nvudc->ep_cx).dma = mapping;

	msg_dbg(nvudc->dev, "allocate ep_cx = %p\n", nvudc->p_epcx);
	u_temp = lower_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ECPLO);


	msg_dbg(nvudc->dev, "allocate ep_cx XHCI_ECPLO= 0x%x\n",
			u_temp);

	u_temp = upper_32_bits(mapping);
	iowrite32(u_temp, nvudc->mmio_reg_base + ECPHI);

	msg_dbg(nvudc->dev, "allocate ep_cx XHCI_EPHI= 0x%x\n", u_temp);

	retval = init_ep0(nvudc);
	if (retval)
		return retval;

	if (!nvudc->status_req) {
		nvudc->status_req =
		container_of(nvudc_alloc_request(&nvudc->udc_ep[0].usb_ep,
			GFP_ATOMIC), struct NV_UDC_REQUEST,
			usb_req);

		nvudc->status_req->usb_req.buf = kmalloc(8, GFP_ATOMIC);
	}

	nvudc->act_bulk_ep = 0;
	nvudc->g_isoc_eps = 0;
	nvudc->ctrl_seq_num = 0;
	nvudc->dev_addr = 0;

	if (tegra_platform_is_fpga())
		fpga_hack_setup_vbus_sense_and_termination(nvudc);

	/* select HS/FS PI */
	u_temp = ioread32(nvudc->mmio_reg_base + CFG_DEV_FE);
	u_temp &= PORTREGSEL_MASK;
	u_temp |= CFG_DEV_FE_PORTREGSEL(2);
	iowrite32(u_temp, nvudc->mmio_reg_base + CFG_DEV_FE);

	/* set LWS = 1 and PLS = rx_detect */
	u_temp = ioread32(nvudc->mmio_reg_base + PORTSC);
	u_temp &= PORTSC_MASK;
	u_temp &= ~PORTSC_PLS(~0);
	u_temp |= PORTSC_PLS(5);
	u_temp |= PORTSC_LWS;
	iowrite32(u_temp, nvudc->mmio_reg_base + PORTSC);

	/* select SS PI */
	u_temp = ioread32(nvudc->mmio_reg_base + CFG_DEV_FE);
	u_temp &= PORTREGSEL_MASK;
	u_temp |= CFG_DEV_FE_PORTREGSEL(1);
	iowrite32(u_temp, nvudc->mmio_reg_base + CFG_DEV_FE);

	/* set LWS = 1 and PLS = rx_detect */
	u_temp = ioread32(nvudc->mmio_reg_base + PORTSC);
	u_temp &= PORTSC_MASK;
	u_temp &= ~PORTSC_PLS(~0);
	u_temp |= PORTSC_PLS(5);
	u_temp |= PORTSC_LWS;
	iowrite32(u_temp, nvudc->mmio_reg_base + PORTSC);

	/* restore portregsel */
	u_temp = ioread32(nvudc->mmio_reg_base + CFG_DEV_FE);
	u_temp &= PORTREGSEL_MASK;
	u_temp |= CFG_DEV_FE_PORTREGSEL(0);
	iowrite32(u_temp, nvudc->mmio_reg_base + CFG_DEV_FE);
	return 0;
}

static int nvudc_pci_get_irq_resources(struct NV_UDC_S *nvudc)
{
	struct pci_dev *pdev = nvudc->pdev.pci;

	if (request_irq(pdev->irq, nvudc_irq, IRQF_SHARED,
				driver_name, nvudc) != 0) {
		msg_err(nvudc->dev, "Can't get irq resource\n");
		return -EBUSY;
	}

	nvudc->irq = nvudc->pdev.pci->irq;
	return 0;
}

static int nvudc_pci_get_memory_resources(struct NV_UDC_S *nvudc)
{
	struct pci_dev *pdev = nvudc->pdev.pci;
	int len;

	nvudc->mmio_phys_base = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);

	if (!request_mem_region(nvudc->mmio_phys_base, len, driver_name)) {
		msg_err(nvudc->dev, "Can't get memory resources\n");
		return -EBUSY;
	}

	nvudc->mmio_reg_base = ioremap_nocache(nvudc->mmio_phys_base, len);
	if (nvudc->mmio_reg_base == NULL) {
		msg_err(nvudc->dev, "memory mapping failed\n");
		return -EFAULT;
	}

	nvudc->mmio_phys_len = len;
	return 0;
}

static int nvudc_probe_pci(struct pci_dev *pdev, const struct pci_device_id
		*id)
{
	int retval;
	u32 i;
	struct NV_UDC_S *nvudc_dev;

	nvudc_dev = kzalloc(sizeof(struct NV_UDC_S), GFP_ATOMIC);
	if  (nvudc_dev == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	/* this needs to be done before using msg_ macros */
	nvudc_dev->dev = &pdev->dev;
	msg_entry(nvudc_dev->dev);

	spin_lock_init(&nvudc_dev->lock);
	pci_set_drvdata(pdev, nvudc_dev);
	msg_dbg(nvudc_dev->dev, "pci_set_drvdata\n");

	/* Initialize back pointer in udc_ep[i]. This is used for accessing
	 * nvudc from gadget api calls */
	for (i = 0; i < 32; i++)
		nvudc_dev->udc_ep[i].nvudc = nvudc_dev;

	nvudc_dev->pdev.pci = pdev;
	dev_set_name(&nvudc_dev->gadget.dev, "gadget");
	nvudc_dev->gadget.dev.parent = &pdev->dev;
	nvudc_dev->gadget.dev.release = nvudc_gadget_release;
	nvudc_dev->gadget.ops = &nvudc_gadget_ops;
	nvudc_dev->gadget.ep0 = &nvudc_dev->udc_ep[0].usb_ep;
	INIT_LIST_HEAD(&nvudc_dev->gadget.ep_list);
	nvudc_dev->gadget.max_speed = USB_SPEED_SUPER;
	nvudc_dev->gadget.name = driver_name;

	if (pci_enable_device(pdev) < 0) {
		retval = -ENODEV;
		goto error;
	}
	nvudc_dev->enabled = 1;
	/* Default value for pullup is 1*/
	nvudc_dev->pullup = 1;

	msg_dbg(nvudc_dev->dev, "pci device enabled\n");

	retval = nvudc_pci_get_memory_resources(nvudc_dev);
	if (retval < 0)
		goto error;

	msg_dbg(nvudc_dev->dev, "ioremap_nocache nvudc->mmio_reg_base = 0x%p\n",
			nvudc_dev->mmio_reg_base);

	retval = reset_data_struct(nvudc_dev);
	if (retval)
		goto error;

	retval = EP0_Start(nvudc_dev);
	if (retval)
		goto error;

	retval = nvudc_pci_get_irq_resources(nvudc_dev);
	if (retval < 0)
		goto error;

	msg_dbg(nvudc_dev->dev, "request_irq, irq = 0x%x\n", nvudc_dev->irq);
	/* Setting the BusMaster bit so that the Device be able to do DMA
	 * which is also required for the MSI interrupts to be generated
	 */
	pci_set_master(pdev);

	retval = device_register(&nvudc_dev->gadget.dev);
	if (retval) {
		msg_err(nvudc_dev->dev, "Can't register device\n");
		goto error;
	}

	msg_dbg(nvudc_dev->dev, "device registered\n");
	nvudc_dev->registered = true;

	retval = usb_add_gadget_udc(&pdev->dev, &nvudc_dev->gadget);
	if (retval)
		goto error;

	msg_dbg(nvudc_dev->dev, "usb_add_gadget_udc\n");

	the_controller = nvudc_dev;

	return 0;
error:
	nvudc_remove_pci(pdev);
	msg_exit(nvudc_dev->dev);
	return retval;
}

void free_data_struct(struct NV_UDC_S *nvudc)
{
	u32 i;
	struct NV_UDC_EP *udc_ep_ptr;
	struct EP_CX_S *p_ep_cx;
	if (nvudc->event_ring0.vaddr) {
		dma_free_coherent(nvudc->dev,
				nvudc->event_ring0.len,
				nvudc->event_ring0.vaddr,
				nvudc->event_ring0.dma);
		nvudc->event_ring0.vaddr = 0;
		nvudc->event_ring0.dma = 0;
		nvudc->event_ring0.len = 0;
	}

	if (nvudc->event_ring1.vaddr) {
		dma_free_coherent(nvudc->dev,
				nvudc->event_ring1.len,
				nvudc->event_ring1.vaddr,
				nvudc->event_ring1.dma);
		nvudc->event_ring1.vaddr = 0;
		nvudc->event_ring1.dma = 0;
		nvudc->event_ring1.len = 0;

	}

	if (nvudc->p_epcx) {
		dma_free_coherent(nvudc->dev, nvudc->ep_cx.len,
				nvudc->ep_cx.vaddr, nvudc->ep_cx.dma);
		nvudc->ep_cx.vaddr = 0;
		nvudc->ep_cx.dma = 0;
		nvudc->ep_cx.len = 0;
	}

	for (i = 2; i < 32; i++) {
		udc_ep_ptr = &nvudc->udc_ep[i];
		p_ep_cx = nvudc->p_epcx + i;
		if (udc_ep_ptr->tran_ring_info.vaddr) {
			dma_free_coherent(nvudc->dev,
					udc_ep_ptr->tran_ring_info.len,
					udc_ep_ptr->tran_ring_info.vaddr,
					udc_ep_ptr->tran_ring_info.dma);
			udc_ep_ptr->tran_ring_info.vaddr = 0;
			udc_ep_ptr->tran_ring_info.dma = 0;
			udc_ep_ptr->tran_ring_info.len = 0;
		}
	}

	if (nvudc->status_req) {
		kfree(nvudc->status_req->usb_req.buf);
		kfree(nvudc->status_req);
	}
	/*        otg_put_transceiver(nvudc->transceiver); */
}

static void nvudc_remove_pci(struct pci_dev *pdev)
{
	struct NV_UDC_S *nvudc;

	nvudc = pci_get_drvdata(pdev);
	msg_entry(nvudc->dev);

	usb_del_gadget_udc(&nvudc->gadget);
	free_data_struct(nvudc);
	if (nvudc) {
		if (nvudc->mmio_phys_base) {
			release_mem_region(nvudc->mmio_phys_base,
					nvudc->mmio_phys_len);
			if (nvudc->mmio_reg_base) {
				iounmap(nvudc->mmio_reg_base);
				nvudc->mmio_reg_base = NULL;
			}
		}
		if (nvudc->irq)
			free_irq(pdev->irq, nvudc);

		if (nvudc->enabled)
			pci_disable_device(pdev);

		if (nvudc->registered)
			device_unregister(&nvudc->gadget.dev);
		pci_set_drvdata(pdev, NULL);
	}
	msg_exit(nvudc->dev);
}

#ifdef ELPG
void save_mmio_reg(struct NV_UDC_S *nvudc)
{
	/* Save Device Address, U2 Timeout, Port Link State and Port State
	*Event Ring enqueue pointer and PCS
	*/
	nvudc->mmio_reg.device_address = nvudc->dev_addr;
	nvudc->mmio_reg.portsc = ioread32(nvudc->mmio_reg_base + PORTSC);
	nvudc->mmio_reg.portpm = ioread32(nvudc->mmio_reg_base + PORTPM);
	nvudc->mmio_reg.ereplo = ioread32(nvudc->mmio_reg_base + EREPLO);
	nvudc->mmio_reg.erephi = ioread32(nvudc->mmio_reg_base + EREPHI);
	nvudc->mmio_reg.ctrl = ioread32(nvudc->mmio_reg_base + CTRL);
}

void restore_mmio_reg(struct NV_UDC_S *nvudc)
{
	u32 u_temp;
	dma_addr_t dma;

	/* restore the event ring info */
	init_hw_event_ring(nvudc);

	iowrite32(nvudc->mmio_reg.ereplo, nvudc->mmio_reg_base + EREPLO);
	iowrite32(nvudc->mmio_reg.erephi, nvudc->mmio_reg_base + EREPHI);

	dma = event_trb_virt_to_dma(nvudc, nvudc->evt_dq_pt);
	iowrite32(lower_32_bits(dma), nvudc->mmio_reg_base + ERDPLO);
	iowrite32(upper_32_bits(dma), nvudc->mmio_reg_base + ERDPHI);

	/* restore endpoint contexts info */
	dma = nvudc->ep_cx.dma;
	u_temp = lower_32_bits(dma);
	iowrite32(u_temp, nvudc->mmio_reg_base + ECPLO);

	u_temp = upper_32_bits(dma);
	iowrite32(u_temp, nvudc->mmio_reg_base + ECPHI;

	/* restore port related info */
	iowrite32(nvudc->mmio_reg.portsc, nvudc->mmio_reg_base + PORTSC);

	iowrite32(nvudc->mmio_reg.portpm, nvudc->mmio_reg_base + PORTPM);

	/* restore the control register
	* this has to be done at last, because ENABLE bit should not be set
	until event ring .. has setup
	*/
	iowrite32(nvudc->mmio_reg.ctrl, nvudc->mmio_reg_base + CTRL);

	/* restore TIMER for U3 exit */
	reg = ioread32(nvudc->base + SSPX_CORE_PADCTL4);
	reg &= ~(RXDAT_VLD_TIMEOUT_U3_MASK);
	reg |= RXDAT_VLD_TIMEOUT_U3;
	iowrite32(reg, nvudc->base + SSPX_CORE_PADCTL4);
}

static int nvudc_suspend_platform(struct platform_device *pdev,
				pm_message_t state)
{
	u32 u_temp;
	struct NV_UDC_S *nvudc;

	nvudc = platform_get_drvdata(pdev);

	/* do not support suspend if link is connected */
	u_temp = ioread32(nvudc->mmio_reg_base + PORTSC);
	if (PORTSC_CCS(u_temp))
		return -EAGAIN;

	nvudc->resume_state = nvudc->device_state;
	nvudc->device_state = USB_STATE_SUSPENDED;

	save_mmio_reg(nvudc);

	return 0;
}

static int nvudc_suspend_pci(struct pci_dev *pdev)
{
	u32 u_temp;
	struct NV_UDC_S *nvudc;

	nvudc = pci_get_drvdata(pdev);

	/* do not support suspend if link is connected */
	u_temp = ioread32(nvudc->mmio_reg_base + PORTSC);
	if (PORTSC_CCS(u_temp))
		return -EAGAIN;

	nvudc->resume_state = nvudc->device_state;
	nvudc->device_state = USB_STATE_SUSPENDED;

	save_mmio_reg(nvudc);
	return 0;
}

static int nvudc_resume_pci(struct pci_dev *pdev)
{
	struct NV_UDC_S *nvudc;

	nvudc = pci_get_drvdata(pdev);
	restore_mmio_reg(nvudc);
	nvudc_resume_state(nvudc);
	return 0;
}

static int nvudc_resume_platform(struct platform_device *pdev)
{
	struct NV_UDC_S *nvudc;

	nvudc = platform_get_drvdata(pdev);
	restore_mmio_reg(nvudc);
	nvudc_resume_state(nvudc);
	return 0;
}
#endif

static void nvudc_plat_clocks_deinit(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;

	if (nvudc->pll_u_480M) {
		devm_clk_put(dev, nvudc->pll_u_480M);
		nvudc->pll_u_480M = NULL;
	}

	if (nvudc->dev_clk) {
		devm_clk_put(dev, nvudc->dev_clk);
		nvudc->dev_clk = NULL;
	}

	if (nvudc->ss_clk) {
		devm_clk_put(dev, nvudc->ss_clk);
		nvudc->ss_clk = NULL;
	}

	if (nvudc->pll_e) {
		devm_clk_put(dev, nvudc->pll_e);
		nvudc->pll_e = NULL;
	}
}

static int nvudc_plat_clocks_init(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	int err;

	nvudc->pll_u_480M = devm_clk_get(dev, "pll_u_480M");
	if (IS_ERR(nvudc->pll_u_480M)) {
		err = PTR_ERR(nvudc->pll_u_480M);
		msg_err(dev, "failed to get pll_u_480M %d\n", err);
		nvudc->pll_u_480M = NULL;
		return err;
	}

	nvudc->dev_clk = devm_clk_get(dev, "dev");
	if (IS_ERR(nvudc->dev_clk)) {
		err = PTR_ERR(nvudc->dev_clk);
		msg_err(dev, "failed to get dev_clk %d\n", err);
		nvudc->dev_clk = NULL;
		return err;
	}

	nvudc->ss_clk = devm_clk_get(dev, "ss");
	if (IS_ERR(nvudc->ss_clk)) {
		err = PTR_ERR(nvudc->ss_clk);
		msg_err(dev, "failed to get ss_clk %d\n", err);
		nvudc->ss_clk = NULL;
		return err;
	}

	nvudc->pll_e = devm_clk_get(dev, "pll_e");
	if (IS_ERR(nvudc->pll_e)) {
		err = PTR_ERR(nvudc->pll_e);
		msg_err(dev, "failed to get pll_e %d\n", err);
		nvudc->pll_e = NULL;
		return err;
	}

	return 0;
}

static int nvudc_plat_clocks_enable(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	int err;

	if (nvudc->pll_u_480M) {
		err = clk_prepare_enable(nvudc->pll_u_480M);
		if (err) {
			msg_err(dev, "failed to enable pll_u_480M %d\n", err);
			return err;
		}
	}

	if (nvudc->dev_clk) {
		err = clk_prepare_enable(nvudc->dev_clk);
		if (err) {
			msg_err(dev, "failed to enable dev_clk %d\n", err);
			goto err_disable_pll_u_480M;
		}
	}

	if (nvudc->ss_clk) {
		err = clk_prepare_enable(nvudc->ss_clk);
		if (err) {
			msg_err(dev, "failed to enable ss_clk %d\n", err);
			goto err_disable_dev_clk;
		}
	}

	if (nvudc->pll_e) {
		err = clk_prepare_enable(nvudc->pll_e);
		if (err) {
			msg_err(dev, "failed to enable pll_e %d\n", err);
			goto err_disable_ss_clk;
		}
	}
	return 0;

err_disable_ss_clk:
	clk_disable_unprepare(nvudc->ss_clk);
err_disable_dev_clk:
	clk_disable_unprepare(nvudc->dev_clk);
err_disable_pll_u_480M:
	clk_disable_unprepare(nvudc->pll_u_480M);
	return err;
}

static void nvudc_plat_clocks_disable(struct NV_UDC_S *nvudc)
{

	clk_disable_unprepare(nvudc->pll_e);
	clk_disable_unprepare(nvudc->ss_clk);
	clk_disable_unprepare(nvudc->dev_clk);
	clk_disable_unprepare(nvudc->pll_u_480M);
}

static int nvudc_plat_regulators_init(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	int err;
	int i;
	size_t size;

	size = udc_regulator_count * sizeof(struct regulator_bulk_data);
	nvudc->supplies = devm_kzalloc(dev, size, GFP_ATOMIC);
	if (!nvudc->supplies) {
		msg_err(dev, "failed to alloc memory for regulators\n");
		return -ENOMEM;
	}

	for (i = 0; i < udc_regulator_count; i++)
		nvudc->supplies[i].supply = udc_regulator_names[i];

	err = devm_regulator_bulk_get(dev, udc_regulator_count,
				nvudc->supplies);
	if (err) {
		msg_err(dev, "failed to request regulators %d\n", err);
		return err;
	}

	err = regulator_bulk_enable(udc_regulator_count, nvudc->supplies);
	if (err) {
		msg_err(dev, "failed to enable regulators %d\n", err);

		for (i = 0; i < udc_regulator_count; i++) {
			if (nvudc->supplies[i].consumer)
				devm_regulator_put(nvudc->supplies[i].consumer);
		}
		return err;
	}

	/* regulator for usb_vbus0, to be moved to OTG driver */
	nvudc->usb_vbus0_reg = regulator_get(dev, "usb_vbus0");
	if (IS_ERR_OR_NULL(nvudc->usb_vbus0_reg))
		msg_err(dev, "%s usb_vbus0 regulator not found\n", __func__);

	return 0;
}

static void nvudc_plat_regulator_deinit(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	int i;
	int err;

	err = regulator_bulk_disable(udc_regulator_count, nvudc->supplies);
	if (err)
		msg_err(dev, "failed to disable regulators %d\n", err);

	for (i = 0; i < udc_regulator_count; i++) {
		if (nvudc->supplies[i].consumer)
			devm_regulator_put(nvudc->supplies[i].consumer);
	}

	devm_kfree(dev, nvudc->supplies);
}

static int nvudc_plat_mmio_regs_init(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		msg_err(dev, "failed to get udc mmio resources\n");
		return -ENXIO;
	}
	msg_info(dev, "xudc mmio start %pa end %pa\n", &res->start, &res->end);
	nvudc->mmio_phys_base = res->start;
	msg_info(dev, "xudc mmio_phys_base %pa\n", &nvudc->mmio_phys_base);

	nvudc->base = devm_request_and_ioremap(dev, res);
	if (!nvudc->base) {
		msg_err(dev, "failed to request and map udc mmio\n");
		return -EFAULT;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		msg_err(dev, "failed to get fpci mmio resources\n");
		return -ENXIO;
	}
	msg_info(dev, "fpci mmio start %pa end %pa\n", &res->start, &res->end);

	nvudc->fpci = devm_request_and_ioremap(dev, res);
	if (!nvudc->fpci) {
		msg_err(dev, "failed to request and map fpci mmio\n");
		return -EFAULT;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		msg_err(dev, "failed to get ipfs mmio resources\n");
		return -ENXIO;
	}
	msg_info(dev, "ipfs mmio start %pa end %pa\n", &res->start, &res->end);

	nvudc->ipfs = devm_request_and_ioremap(dev, res);
	if (!nvudc->ipfs) {
		msg_err(dev, "failed to request and map ipfs mmio\n");
		return -EFAULT;
	}

	return 0;
}

static void nvudc_plat_fpci_ipfs_init(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	u32 reg, addr;

	reg = ioread32(nvudc->ipfs + XUSB_DEV_CONFIGURATION);
	reg |= EN_FPCI;
	iowrite32(reg, nvudc->ipfs + XUSB_DEV_CONFIGURATION);
	reg_dump(dev, nvudc->ipfs, XUSB_DEV_CONFIGURATION);
	usleep_range(10, 30);

	reg = (IO_SPACE_ENABLED | MEMORY_SPACE_ENABLED | BUS_MASTER_ENABLED);
	iowrite32(reg, nvudc->fpci + XUSB_DEV_CFG_1);
	reg_dump(dev, nvudc->fpci, XUSB_DEV_CFG_1);

	addr = lower_32_bits(nvudc->mmio_phys_base);
	reg = BASE_ADDRESS((addr >> 16));
	iowrite32(reg, nvudc->fpci + XUSB_DEV_CFG_4);
	reg_dump(dev, nvudc->fpci, XUSB_DEV_CFG_4);

	reg = upper_32_bits(nvudc->mmio_phys_base);
	iowrite32(reg, nvudc->fpci + XUSB_DEV_CFG_5);
	reg_dump(dev, nvudc->fpci, XUSB_DEV_CFG_5);

	usleep_range(100, 200);

	/* enable interrupt assertion */
	reg = ioread32(nvudc->ipfs + XUSB_DEV_INTR_MASK);
	reg |= IP_INT_MASK;
	writel(reg, nvudc->ipfs + XUSB_DEV_INTR_MASK);
	reg_dump(dev, nvudc->ipfs, XUSB_DEV_INTR_MASK);

	reg = ioread32(nvudc->base + SSPX_CORE_PADCTL4);
	reg &= ~(RXDAT_VLD_TIMEOUT_U3_MASK);
	reg |= RXDAT_VLD_TIMEOUT_U3;
	iowrite32(reg, nvudc->base + SSPX_CORE_PADCTL4);
	reg_dump(dev, nvudc->base, SSPX_CORE_PADCTL4);

}

static int nvudc_plat_irqs_init(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	int err;

	/* TODO try "interrupt-names" in dts */
	err = platform_get_irq(pdev, 0);
	if (err < 0) {
		msg_err(dev, "failed to get irq resource 0\n");
		return -ENXIO;
	}
	nvudc->irq = err;
	msg_info(dev, "irq %u\n", nvudc->irq);

	err = devm_request_irq(dev, nvudc->irq, nvudc_irq, 0,
				dev_name(dev), nvudc);
	if (err < 0) {
		msg_err(dev, "failed to claim irq %d\n", err);
		return err;
	}

	return 0;
}

static int nvudc_get_bdata(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *padctl;
	int ret;
	int portcap, ss_portmap, lane_owner;

	/* Get common setting for padctl */
	padctl = of_parse_phandle(node, "nvidia,common_padctl", 0);

	ret = of_property_read_u32(padctl, "nvidia,ss_portmap"
			, &ss_portmap);
	if (ret < 0)
		pr_err("Fail to get ss_portmap, ret (%d)\n", ret);
	nvudc->bdata.ss_portmap = ss_portmap;

	ret = of_property_read_u32(padctl, "nvidia,lane_owner"
			, &lane_owner);
	if (ret < 0)
		pr_err("Fail to get lane_owner, ret (%d)\n", ret);
	nvudc->bdata.lane_owner = lane_owner;

	return 0;
}

static void t210_program_ss_pad()
{
	tegra_usb_pad_reg_update(UPHY_USB3_PAD0_ECTL_1,
			TX_TERM_CTRL(~0), TX_TERM_CTRL(0x2));

	tegra_usb_pad_reg_update(UPHY_USB3_PAD0_ECTL_2,
			RX_CTLE(~0), RX_CTLE(0xfb));

	tegra_usb_pad_reg_update(UPHY_USB3_PAD0_ECTL_3,
			RX_DFE(~0), RX_DFE(0xc0077f1f));

	tegra_usb_pad_reg_update(UPHY_USB3_PAD0_ECTL_4,
			RX_CDR_CTRL(~0), RX_CDR_CTRL(0x1c7));

	tegra_usb_pad_reg_update(UPHY_USB3_PAD0_ECTL_6,
			RX_EQ_CTRL_H(~0), RX_EQ_CTRL_H(0xfcf01368));
}

static int nvudc_plat_pad_init(struct NV_UDC_S *nvudc)
{
	/* VBUS_ID init */
	usb2_vbus_id_init();

	/* utmi pad init for pad 0 */
	xusb_utmi_pad_init(0, PORT_CAP(0, PORT_CAP_OTG), false);
	utmi_phy_pad_enable();
	utmi_phy_iddq_override(false);

	/* ss pad init for pad 0 */
	xusb_ss_pad_init(0, (nvudc->bdata.ss_portmap & 0xf)
			, XUSB_OTG_MODE);

	t210_program_ss_pad();

	tegra_xhci_ss_wake_signal(TEGRA_XUSB_SS_P0, false);
	tegra_xhci_ss_vcore(TEGRA_XUSB_SS_P0, false);

	/* ss pad phy enable */
	usb3_phy_pad_enable(nvudc->bdata.lane_owner);

	return 0;
}

static void __iomem *car_base;
#define RST_DEVICES_U_0	(0xc)
#define HOST_RST	25
static void fpga_hack_init(struct platform_device *pdev)
{
	car_base = devm_ioremap(&pdev->dev, 0x60006000, 0x1000);
	if (!car_base)
		dev_err(&pdev->dev, "failed to map CAR mmio\n");
}

static void fpga_hack_setup_car(struct NV_UDC_S *nvudc)
{
	struct platform_device *pdev = nvudc->pdev.plat;
	struct device *dev = &pdev->dev;
	struct clock *c = clk_get_sys(NULL, "xusb_padctl");
	u32 val;

	if (!car_base) {
		dev_err(&pdev->dev, "not able to access CAR mmio\n");
		return;
	}

	tegra_periph_reset_deassert(c);

	reg_dump(dev, car_base, RST_DEVICES_U_0);
	val = ioread32(car_base + RST_DEVICES_U_0);
	val &= ~((1 << HOST_RST));
	iowrite32(val, car_base + RST_DEVICES_U_0);
	reg_dump(dev, car_base, RST_DEVICES_U_0);
}

static int tegra_xudc_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct NV_UDC_S *nvudc;
	int i;
	int err;

	if (!dev->dma_mask)
		dev->dma_mask = &dev->coherent_dma_mask;

	err = dma_set_mask(dev, DMA_BIT_MASK(64));
	if (err) {
		dev_warn(dev, "no suitable DMA available\n");
		return err;
	} else
		dma_set_coherent_mask(dev, DMA_BIT_MASK(64));

	nvudc = devm_kzalloc(dev, sizeof(*nvudc), GFP_ATOMIC);
	if (!nvudc) {
		dev_err(dev, "failed to allocate memory for nvudc\n");
		return -ENOMEM;
	}

	INIT_WORK(&nvudc->work, irq_work);

	nvudc->pdev.plat = pdev;
	nvudc->dev = dev;
	platform_set_drvdata(pdev, nvudc);

	nvudc_get_bdata(nvudc);

	if (tegra_platform_is_fpga()) {
		fpga_hack_init(pdev);
		fpga_hack_setup_car(nvudc);
	}

	err = nvudc_plat_regulators_init(nvudc);
	if (err) {
		dev_err(dev, "failed to init regulators\n");
		err = -EPROBE_DEFER;
		goto err_kfree_nvudc;
	}

	if (pex_usb_pad_pll_reset_deassert())
		dev_err(dev, "failed to deassert pex pll\n");

	err = nvudc_plat_clocks_init(nvudc);
	if (err) {
		dev_err(dev, "failed to init clocks\n");
		goto err_regulators_deinit;
	}

	err = nvudc_plat_mmio_regs_init(nvudc);
	if (err) {
		dev_err(dev, "failed to init mmio regions\n");
		goto err_clocks_deinit;
	}

	err = tegra_unpowergate_partition_with_clk_on(TEGRA_POWERGATE_XUSBA);
	if (err) {
		dev_err(dev, "failed to unpowergate XUSBA partition\n");
		goto err_clocks_deinit;
	}

	err = tegra_unpowergate_partition_with_clk_on(TEGRA_POWERGATE_XUSBB);
	if (err) {
		dev_err(dev, "failed to unpowergate XUSBB partition\n");
		goto err_powergate_xusba;
	}

	err = nvudc_plat_clocks_enable(nvudc);
	if (err) {
		dev_err(dev, "failed to enable clocks\n");
		goto err_powergate_xusbb;
	}

	err = nvudc_plat_pad_init(nvudc);
	if (err) {
		dev_err(dev, "failed to config pads\n");
		goto err_clocks_disable;
	}

	nvudc_plat_fpci_ipfs_init(nvudc);

	err = nvudc_plat_irqs_init(nvudc);
	if (err) {
		dev_err(dev, "failed to init irqs\n");
		goto err_clocks_disable;
	}

	spin_lock_init(&nvudc->lock);

	for (i = 0; i < 32; i++)
		nvudc->udc_ep[i].nvudc = nvudc;

	dev_set_name(&nvudc->gadget.dev, "gadget");
	nvudc->gadget.dev.parent = &pdev->dev;
	nvudc->gadget.dev.release = nvudc_gadget_release;
	nvudc->gadget.ops = &nvudc_gadget_ops;
	nvudc->gadget.ep0 = &nvudc->udc_ep[0].usb_ep;
	INIT_LIST_HEAD(&nvudc->gadget.ep_list);
	nvudc->gadget.name = driver_name;
	nvudc->gadget.max_speed = USB_SPEED_SUPER;

	nvudc->mmio_reg_base = nvudc->base; /* TODO support device context */

	err = reset_data_struct(nvudc);
	if (err) {
		dev_err(dev, "failed to reset_data_struct\n");
		goto err_clocks_disable;
	}

	err = EP0_Start(nvudc);
	if (err) {
		dev_err(dev, "failed to EP0_Start\n");
		goto err_clocks_disable;
	}

	err = usb_add_gadget_udc(&pdev->dev, &nvudc->gadget);
	if (err) {
		dev_err(dev, "failed to usb_add_gadget_udc\n");
		goto err_clocks_disable;
	}

	the_controller = nvudc; /* TODO support device context */

	/* TODO: support non-dt ?*/
	nvudc->vbus_extcon_dev =
		extcon_get_extcon_dev_by_cable(&pdev->dev, "vbus");
	if (IS_ERR(nvudc->vbus_extcon_dev))
		err = -ENODEV;

	nvudc->vbus_extcon_nb.notifier_call = extcon_notifications;
	extcon_register_notifier(nvudc->vbus_extcon_dev,
						&nvudc->vbus_extcon_nb);

	nvudc->id_extcon_dev =
		extcon_get_extcon_dev_by_cable(&pdev->dev, "id");
	if (IS_ERR(nvudc->id_extcon_dev))
		err = -ENODEV;

	nvudc->id_extcon_nb.notifier_call = extcon_id_notifications;
	extcon_register_notifier(nvudc->id_extcon_dev,
						&nvudc->id_extcon_nb);
	return 0;

err_clocks_disable:
	nvudc_plat_clocks_disable(nvudc);
err_powergate_xusbb:
	tegra_powergate_partition(TEGRA_POWERGATE_XUSBB);
err_powergate_xusba:
	tegra_powergate_partition(TEGRA_POWERGATE_XUSBA);
err_clocks_deinit:
	nvudc_plat_clocks_deinit(nvudc);
err_regulators_deinit:
	nvudc_plat_regulator_deinit(nvudc);
err_kfree_nvudc:
	devm_kfree(dev, nvudc);
	return err;
}

static int __exit tegra_xudc_plat_remove(struct platform_device *pdev)
{
	struct NV_UDC_S *nvudc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_info(dev, "%s nvudc %p\n", __func__, nvudc);

	/* TODO implement synchronization */
	if (nvudc) {
		usb_del_gadget_udc(&nvudc->gadget);
		free_data_struct(nvudc);
		nvudc_plat_clocks_disable(nvudc);
		tegra_powergate_partition(TEGRA_POWERGATE_XUSBB);
		tegra_powergate_partition(TEGRA_POWERGATE_XUSBA);
		nvudc_plat_clocks_deinit(nvudc);
		if (pex_usb_pad_pll_reset_assert())
			pr_err("Fail to assert pex pll\n");
		nvudc_plat_regulator_deinit(nvudc);
		extcon_unregister_notifier(nvudc->vbus_extcon_dev,
			&nvudc->vbus_extcon_nb);
		extcon_unregister_notifier(nvudc->id_extcon_dev,
			&nvudc->id_extcon_nb);
		devm_kfree(dev, nvudc);
		platform_set_drvdata(pdev, NULL);
	}
	return 0;
}

static void tegra_xudc_plat_shutdown(struct platform_device *pdev)
{
	struct NV_UDC_S *nvudc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_info(dev, "%s nvudc %p\n", __func__, nvudc);

	nvudc_gadget_pullup(&nvudc->gadget, 0);
}

static struct of_device_id tegra_xudc_of_match[] = {
	{.compatible = "nvidia,tegra210-xudc",},
	{},
};

static struct platform_driver tegra_xudc_driver = {
	.probe = tegra_xudc_plat_probe,
	.remove = __exit_p(tegra_xudc_plat_remove),
	.shutdown = tegra_xudc_plat_shutdown,
	.driver = {
		.name = driver_name,
		.of_match_table = tegra_xudc_of_match,
		.owner = THIS_MODULE,
	},
	/* TODO support PM */
};

static DEFINE_PCI_DEVICE_TABLE(nvpci_ids) = {
	{
		.class = ((PCI_CLASS_SERIAL_USB << 8) | 0xfe),
		.class_mask = ~0,
		.vendor = 0x10de,
		.device = 0x0fad,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
	},
	{
		/* end: all zeroes */
	}
};
MODULE_DEVICE_TABLE(pci, nvpci_ids);

static struct pci_driver nvudc_driver_pci = {
	.name = driver_name,
	.id_table = nvpci_ids,
	.probe = nvudc_probe_pci,
	.remove = nvudc_remove_pci,
	/*	.suspend = nvudc_suspend_pci,  */
	/*	.resume = nvudc_resume_pci,    */
	/*      .shutdown = nvudc_shutdown, */
};

static int __init udc_init(void)
{
	int ret;

	ret = platform_driver_register(&tegra_xudc_driver);
	if (ret) {
		pr_err("%s failed to register platform driver %d\n",
				__func__, ret);
		return ret;
	}

	ret = pci_register_driver(&nvudc_driver_pci);
	if (ret) {
		pr_err("%s failed to register pci driver %d\n",
				__func__, ret);
		platform_driver_unregister(&tegra_xudc_driver);
		return ret;
	}

	return 0;
}

static void __exit udc_exit(void)
{
	pci_unregister_driver(&nvudc_driver_pci);
	platform_driver_unregister(&tegra_xudc_driver);
}

module_init(udc_init);
module_exit(udc_exit);

MODULE_DESCRIPTION("NV Usb3 Peripheral Controller");
MODULE_VERSION("0.0.1");
MODULE_AUTHOR("Hui Fu");
MODULE_LICENSE("GPL v2");
