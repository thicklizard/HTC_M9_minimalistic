/**
 * gadget.c - DesignWare USB3 DRD Controller Gadget Framework Link
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/module.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/htc_info.h>
#include <linux/power/htc_charger.h>

#include "core.h"
#include "gadget.h"
#include "debug.h"
#include "io.h"

static int bulk_ep_xfer_timeout_ms;
module_param(bulk_ep_xfer_timeout_ms, int, S_IRUGO | S_IWUSR);

#define bulk_ep_xfer_timeout_ns (bulk_ep_xfer_timeout_ms * NSEC_PER_MSEC)
static void dwc3_gadget_wakeup_interrupt(struct dwc3 *dwc, bool remote_wakeup);
static int dwc3_gadget_wakeup_int(struct dwc3 *dwc);
static void dwc3_restart_hrtimer(struct dwc3 *dwc);

struct dwc3_usb_gadget {
	struct work_struct wakeup_work;
	bool disable_during_lpm;
	struct dwc3 *dwc;
};

static void dwc3_endpoint_transfer_complete(struct dwc3 *, struct dwc3_ep *,
	const struct dwc3_event_depevt *, int);
int dwc3_gadget_set_test_mode(struct dwc3 *dwc, int mode)
{
	u32		reg;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;

	switch (mode) {
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
	case TEST_PACKET:
	case TEST_FORCE_EN:
		reg |= mode << 1;
		break;
	default:
		return -EINVAL;
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	return 0;
}

int dwc3_gadget_set_link_state(struct dwc3 *dwc, enum dwc3_link_state state)
{
	int		retries = 10000;
	u32		reg;

	if (dwc->revision >= DWC3_REVISION_194A) {
		while (--retries) {
			reg = dwc3_readl(dwc->regs, DWC3_DSTS);
			if (reg & DWC3_DSTS_DCNRD)
				udelay(5);
			else
				break;
		}

		if (retries <= 0)
			return -ETIMEDOUT;
	}

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;

	
	reg |= DWC3_DCTL_ULSTCHNGREQ(state);
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	if (dwc->revision >= DWC3_REVISION_194A)
		return 0;

	
	retries = 10000;
	while (--retries) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);

		if (DWC3_DSTS_USBLNKST(reg) == state)
			return 0;

		udelay(5);
	}

	dev_vdbg(dwc->dev, "link state change request timed out\n");

	return -ETIMEDOUT;
}

int dwc3_gadget_resize_tx_fifos(struct dwc3 *dwc)
{
	int		last_fifo_depth = 0;
	int		ram1_depth;
	int		fifo_size;
	int		mdwidth;
	int		num;
	int		num_eps;
	struct usb_composite_dev *cdev = get_gadget_data(&dwc->gadget);

	if (!(cdev && cdev->config) || !dwc->needs_fifo_resize)
		return 0;

	num_eps = dwc->num_in_eps;
	ram1_depth = DWC3_RAM1_DEPTH(dwc->hwparams.hwparams7);
	mdwidth = DWC3_MDWIDTH(dwc->hwparams.hwparams0);

	
	mdwidth >>= 3;

	dev_dbg(dwc->dev, "%s: num eps: %d\n", __func__, num_eps);

	for (num = 0; num < num_eps; num++) {
		struct dwc3_ep	*dep = dwc->eps[(num << 1) | 1];
		int		mult = 1;
		int		tmp;
		int		max_packet = 1024;

		tmp = max_packet + mdwidth;
		if (dep->endpoint.ep_type == EP_TYPE_GSI)
			mult = 3;

		if (!(dep->flags & DWC3_EP_ENABLED)) {
			dev_dbg(dwc->dev, "ep%dIn not enabled", num);
			goto resize_fifo;
		}

		if (((dep->endpoint.maxburst > 1) &&
				usb_endpoint_xfer_bulk(dep->endpoint.desc))
				|| usb_endpoint_xfer_isoc(dep->endpoint.desc))
			mult = 3;

<<<<<<< HEAD
		tmp = mult * (dep->endpoint.maxpacket + mdwidth);

		if (dwc->tx_fifo_size &&
			(usb_endpoint_xfer_bulk(dep->endpoint.desc)
			|| usb_endpoint_xfer_isoc(dep->endpoint.desc))) {
			if (!dwc->tx_fifo_reduced)
				tmp = 3 * (1024 + mdwidth);
			else
				tmp = mult * (1024 + mdwidth);
		}

=======
>>>>>>> 0e91d2a... Nougat
resize_fifo:
		tmp *= mult;
		tmp += mdwidth;

		fifo_size = DIV_ROUND_UP(tmp, mdwidth);

		fifo_size |= (last_fifo_depth << 16);

		dev_vdbg(dwc->dev, "%s: Fifo Addr %04x Size %d\n",
				dep->name, last_fifo_depth, fifo_size & 0xffff);

		last_fifo_depth += (fifo_size & 0xffff);
		if (dwc->tx_fifo_size &&
				(last_fifo_depth >= dwc->tx_fifo_size)) {
			dev_err(dwc->dev, "Fifosize(%d) > available RAM(%d)\n",
					last_fifo_depth, dwc->tx_fifo_size);
			return -ENOMEM;
		}

		dwc3_writel(dwc->regs, DWC3_GTXFIFOSIZ(num), fifo_size);

	}

	return 0;
}

void dwc3_gadget_giveback(struct dwc3_ep *dep, struct dwc3_request *req,
		int status)
{
	struct dwc3			*dwc = dep->dwc;
	int				i;

	if (req->queued) {
		i = 0;
		do {
			dep->busy_slot++;
			if (((dep->busy_slot & DWC3_TRB_MASK) ==
				DWC3_TRB_NUM- 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				dep->busy_slot++;
		} while(++i < req->request.num_mapped_sgs);
		req->queued = false;

		if (req->request.zero && req->ztrb) {
			dep->busy_slot++;
			req->ztrb = NULL;
			if (((dep->busy_slot & DWC3_TRB_MASK) ==
				DWC3_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				dep->busy_slot++;
		}
	}
	list_del(&req->list);
	req->trb = NULL;

	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	if (dwc->ep0_bounced && dep->number == 0)
		dwc->ep0_bounced = false;
	else
		usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);

	dev_dbg(dwc->dev, "request %p from %s completed %d/%d ===> %d\n",
			req, dep->name, req->request.actual,
			req->request.length, status);

	dbg_done(dep->number, req->request.actual, req->request.status);
	spin_unlock(&dwc->lock);
	req->request.complete(&dep->endpoint, &req->request);
	spin_lock(&dwc->lock);
}

static const char *dwc3_gadget_ep_cmd_string(u8 cmd)
{
	switch (cmd) {
	case DWC3_DEPCMD_DEPSTARTCFG:
		return "Start New Configuration";
	case DWC3_DEPCMD_ENDTRANSFER:
		return "End Transfer";
	case DWC3_DEPCMD_UPDATETRANSFER:
		return "Update Transfer";
	case DWC3_DEPCMD_STARTTRANSFER:
		return "Start Transfer";
	case DWC3_DEPCMD_CLEARSTALL:
		return "Clear Stall";
	case DWC3_DEPCMD_SETSTALL:
		return "Set Stall";
	case DWC3_DEPCMD_GETEPSTATE:
		return "Get Endpoint State";
	case DWC3_DEPCMD_SETTRANSFRESOURCE:
		return "Set Endpoint Transfer Resource";
	case DWC3_DEPCMD_SETEPCONFIG:
		return "Set Endpoint Configuration";
	default:
		return "UNKNOWN command";
	}
}

int dwc3_send_gadget_generic_command(struct dwc3 *dwc, int cmd, u32 param)
{
	u32		timeout = 500;
	u32		reg;
	int ret;

	dwc3_writel(dwc->regs, DWC3_DGCMDPAR, param);
	dwc3_writel(dwc->regs, DWC3_DGCMD, cmd | DWC3_DGCMD_CMDACT);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DGCMD);
		if (!(reg & DWC3_DGCMD_CMDACT)) {
			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
					DWC3_DGCMD_STATUS(reg));
			ret = 0;
			break;
		}

		timeout--;
		if (!timeout) {
			ret = -ETIMEDOUT;
			break;
		}
		udelay(1);
	} while (1);

	return ret;
}

int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, struct dwc3_gadget_ep_cmd_params *params)
{
	struct dwc3_ep		*dep = dwc->eps[ep];
	u32			timeout = 1500;
	u32			reg;
	int ret;

	dev_vdbg(dwc->dev, "%s: cmd '%s' params %08x %08x %08x\n",
			dep->name,
			dwc3_gadget_ep_cmd_string(cmd), params->param0,
			params->param1, params->param2);

	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR0(ep), params->param0);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR1(ep), params->param1);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR2(ep), params->param2);

	dwc3_writel(dwc->regs, DWC3_DEPCMD(ep), cmd | DWC3_DEPCMD_CMDACT);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DEPCMD(ep));
		if (!(reg & DWC3_DEPCMD_CMDACT)) {
			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
					DWC3_DEPCMD_STATUS(reg));
			if (reg & 0x2000)
				ret = -EAGAIN;
			else
				ret = 0;
			break;
		}

		timeout--;
		if (!timeout) {
			dev_err(dwc->dev, "%s command timeout for %s\n",
				dwc3_gadget_ep_cmd_string(cmd),
				dep->name);
			ret = -ETIMEDOUT;
			break;
		}

		udelay(1);
	} while (1);

	return ret;
}

dma_addr_t dwc3_trb_dma_offset(struct dwc3_ep *dep,
		struct dwc3_trb *trb)
{
	u32		offset = (char *) trb - (char *) dep->trb_pool;

	return dep->trb_pool_dma + offset;
}

static int dwc3_alloc_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;
	u32			num_trbs = DWC3_TRB_NUM;

	if (dep->trb_pool)
		return 0;

	if (dep->number == 0 || dep->number == 1)
		return 0;

	dep->trb_pool = dma_zalloc_coherent(dwc->dev,
<<<<<<< HEAD
			sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			&dep->trb_pool_dma, GFP_KERNEL);
=======
			sizeof(struct dwc3_trb) * num_trbs,
			&dep->trb_pool_dma, GFP_ATOMIC);
>>>>>>> 0e91d2a... Nougat
	if (!dep->trb_pool) {
		dev_err(dep->dwc->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}
	dep->num_trbs = num_trbs;

	return 0;
}

static void dwc3_free_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;

	dma_free_coherent(dwc->dev, sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			dep->trb_pool, dep->trb_pool_dma);

	dep->trb_pool = NULL;
	dep->trb_pool_dma = 0;
}

static int dwc3_gadget_start_config(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;
	u32			cmd;

	memset(&params, 0x00, sizeof(params));

	if (dep->number != 1) {
		cmd = DWC3_DEPCMD_DEPSTARTCFG;
		
		if (dep->number > 1) {
			if (dwc->start_config_issued)
				return 0;
			dwc->start_config_issued = true;
			cmd |= DWC3_DEPCMD_PARAM(2);
		}

		return dwc3_send_gadget_ep_cmd(dwc, 0, cmd, &params);
	}

	return 0;
}

static int dwc3_gadget_set_ep_config(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPCFG_EP_TYPE(usb_endpoint_type(desc))
		| DWC3_DEPCFG_MAX_PACKET_SIZE(usb_endpoint_maxp(desc));

	
	if (dwc->gadget.speed == USB_SPEED_SUPER) {
		u32 burst = dep->endpoint.maxburst - 1;

		params.param0 |= DWC3_DEPCFG_BURST_SIZE(burst);
	}

	if (ignore)
		params.param0 |= DWC3_DEPCFG_IGN_SEQ_NUM;

	if (!dep->endpoint.endless) {
		pr_debug("%s(): enable xfer_complete_int for %s\n",
				__func__, dep->endpoint.name);
		params.param1 = DWC3_DEPCFG_XFER_COMPLETE_EN
				| DWC3_DEPCFG_XFER_NOT_READY_EN;
	} else {
		pr_debug("%s(): disable xfer_complete_int for %s\n",
				 __func__, dep->endpoint.name);
	}

	if (usb_ss_max_streams(comp_desc) && usb_endpoint_xfer_bulk(desc)) {
		params.param1 |= DWC3_DEPCFG_STREAM_CAPABLE
			| DWC3_DEPCFG_STREAM_EVENT_EN;
		dep->stream_capable = true;
	}
	if ((usb_endpoint_xfer_isoc(desc))
	    || ((dep->endpoint.is_ncm) && !usb_endpoint_xfer_control(desc)))
		params.param1 |= DWC3_DEPCFG_XFER_IN_PROGRESS_EN;
	params.param1 |= DWC3_DEPCFG_EP_NUMBER(dep->number);

	if (dep->direction)
		params.param0 |= DWC3_DEPCFG_FIFO_NUMBER(dep->number >> 1);

	if (desc->bInterval) {
		params.param1 |= DWC3_DEPCFG_BINTERVAL_M1(desc->bInterval - 1);
		dep->interval = 1 << (desc->bInterval - 1);
	}

	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETEPCONFIG, &params);
}

static int dwc3_gadget_set_xfer_resource(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPXFERCFG_NUM_XFER_RES(1);

	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETTRANSFRESOURCE, &params);
}

static int __dwc3_gadget_ep_enable(struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;
	int			ret = -ENOMEM;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		ret = dwc3_gadget_start_config(dwc, dep);
		if (ret) {
			dev_err(dwc->dev, "start_config() failed for %s\n",
								dep->name);
			return ret;
		}
	}

	ret = dwc3_gadget_set_ep_config(dwc, dep, desc, comp_desc, ignore);
	if (ret) {
		dev_err(dwc->dev, "set_ep_config() failed for %s\n",
							dep->name);
		return ret;
	}

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		struct dwc3_trb	*trb_st_hw;
		struct dwc3_trb	*trb_link;

		ret = dwc3_gadget_set_xfer_resource(dwc, dep);
		if (ret) {
			dev_err(dwc->dev, "set_xfer_resource() failed for %s\n",
								dep->name);
			return ret;
		}

		dep->endpoint.desc = desc;
		dep->comp_desc = comp_desc;
		dep->type = usb_endpoint_type(desc);
		dep->flags |= DWC3_EP_ENABLED;

		reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
		reg |= DWC3_DALEPENA_EP(dep->number);
		dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);
		if (dep->endpoint.is_ncm)
			dwc3_gadget_resize_tx_fifos(dwc);
		if (!usb_endpoint_xfer_isoc(desc))
			return 0;

		
		trb_st_hw = &dep->trb_pool[0];

		trb_link = &dep->trb_pool[DWC3_TRB_NUM - 1];
		memset(trb_link, 0, sizeof(*trb_link));

		trb_link->bpl = lower_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->bph = upper_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->ctrl |= DWC3_TRBCTL_LINK_TRB;
		trb_link->ctrl |= DWC3_TRB_CTRL_HWO;
	}

	return 0;
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum);
static void dwc3_remove_requests(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_request		*req;

	if (!list_empty(&dep->req_queued)) {
		dwc3_stop_active_transfer(dwc, dep->number);

		
		while (!list_empty(&dep->req_queued)) {
			req = next_request(&dep->req_queued);

			dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
		}
	}

	while (!list_empty(&dep->request_list)) {
		req = next_request(&dep->request_list);

		dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
	}
}

static int __dwc3_gadget_ep_disable(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;

	dwc3_remove_requests(dwc, dep);

	
	if (dep->flags & DWC3_EP_STALL)
		__dwc3_gadget_ep_set_halt(dep, 0, false);

	reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
	reg &= ~DWC3_DALEPENA_EP(dep->number);
	dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);

	dep->stream_capable = false;
	dep->endpoint.desc = NULL;
	dep->comp_desc = NULL;
	dep->type = 0;
	dep->flags = 0;

<<<<<<< HEAD
	memset(&dep->trb_pool[0], 0, sizeof(struct dwc3_trb) * DWC3_TRB_NUM);
	dbg_event(dep->number, "Clr_TRB", 0);
=======
	if (dep->number > 1 && dep->trb_pool) {
		memset(&dep->trb_pool[0], 0,
			sizeof(struct dwc3_trb) * dep->num_trbs);
		dbg_event(dep->number, "Clr_TRB", 0);
	}

>>>>>>> 0e91d2a... Nougat
	return 0;
}

static void dwc3_gadget_ep_nuke(struct usb_ep *ep)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long flags;
	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;
	spin_lock_irqsave(&dwc->lock, flags);
	dwc3_remove_requests(dwc, dep);
	spin_unlock_irqrestore(&dwc->lock, flags);

}


static int dwc3_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

static int dwc3_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}


static int dwc3_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("dwc3: invalid parameters. ep=%p, desc=%p, DT=%d\n",
			ep, desc, desc ? desc->bDescriptorType : 0);
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("dwc3: missing wMaxPacketSize\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (dep->flags & DWC3_EP_ENABLED) {
		dev_WARN_ONCE(dwc->dev, true, "%s is already enabled\n",
				dep->name);
		return 0;
	}

	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		strlcat(dep->name, "-control", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_ISOC:
		strlcat(dep->name, "-isoc", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_BULK:
		strlcat(dep->name, "-bulk", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_INT:
		strlcat(dep->name, "-int", sizeof(dep->name));
		break;
	default:
		dev_err(dwc->dev, "invalid endpoint transfer type\n");
	}

	dev_vdbg(dwc->dev, "Enabling %s\n", dep->name);

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_enable(dep, desc, ep->comp_desc, false);
	dbg_event(dep->number, "ENABLE", ret);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_disable(struct usb_ep *ep)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		dev_dbg(dwc->dev, "%s is already disabled\n", dep->name);
		dbg_event(dep->number, "ALRDY DISABLED", dep->flags);
		return 0;
	}

	
	if (!strnstr(dep->name, "gsi", 10)) {
		snprintf(dep->name, sizeof(dep->name), "ep%d%s",
			dep->number >> 1,
			(dep->number & 1) ? "in" : "out");
	}

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_disable(dep);
	dbg_event(dep->number, "DISABLE", ret);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static struct usb_request *dwc3_gadget_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req;
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req) {
		dev_err(dwc->dev, "not enough memory\n");
		return NULL;
	}

	req->epnum	= dep->number;
	req->dep	= dep;
	req->request.dma = DMA_ERROR_CODE;

	return &req->request;
}

static void dwc3_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);

	kfree(req);
}

static void dwc3_prepare_one_trb(struct dwc3_ep *dep,
		struct dwc3_request *req, dma_addr_t dma,
		unsigned length, unsigned last, unsigned chain, unsigned node,
		unsigned ioc)
{
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_trb		*trb;
	bool			zlp_appended = false;
	unsigned		rlen;

	dev_vdbg(dwc->dev, "%s: req %p dma %08llx length %d%s%s\n",
			dep->name, req, (unsigned long long) dma,
			length, last ? " last" : "",
			chain ? " chain" : "");


	trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];

	if (!req->trb) {
		dwc3_gadget_move_request_queued(req);
		req->trb = trb;
		req->trb_dma = dwc3_trb_dma_offset(dep, trb);
		req->start_slot = dep->free_slot & DWC3_TRB_MASK;
	}

	dep->free_slot++;
	
	if (((dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
			usb_endpoint_xfer_isoc(dep->endpoint.desc))
		dep->free_slot++;

update_trb:
	trb->size = DWC3_TRB_SIZE_LENGTH(length);
	trb->bpl = lower_32_bits(dma);
	trb->bph = upper_32_bits(dma);

	switch (usb_endpoint_type(dep->endpoint.desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		trb->ctrl = DWC3_TRBCTL_CONTROL_SETUP;
		break;

	case USB_ENDPOINT_XFER_ISOC:
		if (!node)
			trb->ctrl = DWC3_TRBCTL_ISOCHRONOUS_FIRST;
		else
			trb->ctrl = DWC3_TRBCTL_ISOCHRONOUS;

		if (!req->request.no_interrupt && !chain)
			trb->ctrl |= DWC3_TRB_CTRL_IOC;
		break;

	case USB_ENDPOINT_XFER_BULK:
		trb->ctrl = DWC3_TRBCTL_NORMAL;
		if (!last && bulk_ep_xfer_timeout_ms)
			trb->ctrl |= DWC3_TRB_CTRL_CSP;
		break;

	case USB_ENDPOINT_XFER_INT:
		trb->ctrl = DWC3_TRBCTL_NORMAL;
		if (ioc)
			trb->ctrl |= DWC3_TRB_CTRL_IOC;
		break;
	default:
		BUG();
	}

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		trb->ctrl |= DWC3_TRB_CTRL_ISP_IMI;
		trb->ctrl |= DWC3_TRB_CTRL_CSP;
	}

	if (chain)
		trb->ctrl |= DWC3_TRB_CTRL_CHN;
        if (dep->endpoint.is_ncm)
            trb->ctrl |= DWC3_TRB_CTRL_CSP;
	if (usb_endpoint_xfer_bulk(dep->endpoint.desc) && dep->stream_capable)
		trb->ctrl |= DWC3_TRB_CTRL_SID_SOFN(req->request.stream_id);

	trb->ctrl |= DWC3_TRB_CTRL_HWO;

	rlen = req->request.length;
	if (!zlp_appended && !chain &&
		req->request.zero && rlen &&
		(rlen % usb_endpoint_maxp(dep->endpoint.desc) == 0)) {

		zlp_appended = true;
		
		if (((dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
			usb_endpoint_xfer_isoc(dep->endpoint.desc))
			dep->free_slot++;

		trb->ctrl |= DWC3_TRB_CTRL_CHN;
		trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];
		dep->free_slot++;

		req->ztrb = trb;
		length = 0;

		goto update_trb;
	}
	if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		if (last)
		trb->ctrl |= DWC3_TRB_CTRL_LST;
		else if (dep->endpoint.is_ncm && (!req->request.no_interrupt) && (dep->direction != 1))
			trb->ctrl |= DWC3_TRB_CTRL_IOC;
	}
}

static void dwc3_prepare_trbs(struct dwc3_ep *dep, bool starting)
{
	struct dwc3_request	*req, *n;
	u32			trbs_left;
	u32			max;
	unsigned int		last_one = 0;
	int			maxpkt_size;
	bool			isoc;
	struct dwc3		*dwc = dep->dwc;

	maxpkt_size = usb_endpoint_maxp(dep->endpoint.desc);
	isoc = usb_endpoint_xfer_isoc(dep->endpoint.desc);

	BUILD_BUG_ON_NOT_POWER_OF_2(DWC3_TRB_NUM);

	
	trbs_left = (dep->busy_slot - dep->free_slot) & DWC3_TRB_MASK;

	
	if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		max = DWC3_TRB_NUM - (dep->free_slot & DWC3_TRB_MASK);
		if (trbs_left > max)
			trbs_left = max;
	}

	if (!trbs_left) {
		if (!starting)
			return;
		dep->busy_slot = 0;
		dep->free_slot = 0;
		
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc))
			trbs_left = DWC3_TRB_NUM-1;
		else
			trbs_left = DWC3_TRB_NUM;
	}

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
		(dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_MASK-1)
			trbs_left--;

	list_for_each_entry_safe(req, n, &dep->request_list, list) {
		unsigned	length;
		dma_addr_t	dma;
		int		num_trbs_required = 0;

		last_one = false;

		
		if (isoc)
			num_trbs_required++;

		if (req->request.num_mapped_sgs)
			num_trbs_required += req->request.num_mapped_sgs;
		else
			num_trbs_required++;

		if (req->request.zero && req->request.length &&
				(req->request.length % maxpkt_size == 0))
			num_trbs_required++;

		if (trbs_left < num_trbs_required)
			break;

		if (req->request.num_mapped_sgs > 0) {
			struct usb_request *request = &req->request;
			struct scatterlist *sg = request->sg;
			struct scatterlist *s;
			int		i;

			for_each_sg(sg, s, request->num_mapped_sgs, i) {
				unsigned chain = true;
				unsigned	ioc = 0;

				length = sg_dma_len(s);
				dma = sg_dma_address(s);

				if (i == (request->num_mapped_sgs - 1) ||
						sg_is_last(s)) {
					unsigned temp = 0;
					unsigned len;
					struct dwc3_request *nreq = n;
					struct usb_request *ureq;
					bool mpkt = false;

					chain = false;
					if (list_empty(&dep->request_list)) {
						last_one = true;
						goto start_trb_queuing;
					}

					ureq = &nreq->request;
					len  = ureq->length;

					if (len % maxpkt_size == 0)
						mpkt = true;

					if (ureq->zero && len && mpkt)
						temp++;

					if (ureq->num_mapped_sgs)
						temp +=
						ureq->num_mapped_sgs;
					else
						temp++;

					if (isoc)
						temp++;

					if (trbs_left <= temp)
						last_one = true;

				}

start_trb_queuing:
				trbs_left--;
				if (!trbs_left)
					last_one = true;

				if (last_one)
					chain = false;

				if (!last_one && !chain &&
					!request->no_interrupt)
					ioc = 1;

				dwc3_prepare_one_trb(dep, req, dma, length,
						last_one, chain, i, ioc);

				if (last_one)
					break;
			}
			dbg_queue(dep->number, &req->request, trbs_left);

			if (last_one)
				break;
		} else {
			struct dwc3_request	*req1;
			int maxpkt_size = usb_endpoint_maxp(dep->endpoint.desc);

			dma = req->request.dma;
			length = req->request.length;
			trbs_left--;

			if (req->request.zero && length &&
						(length % maxpkt_size == 0))
				trbs_left--;

			if (!trbs_left) {
				last_one = 1;
			} else if (dep->direction && (trbs_left <= 1)) {
				req1 = next_request(&req->list);
				if (req1->request.zero && req1->request.length
				 && (req1->request.length % maxpkt_size == 0))
					last_one = 1;
			}

			
			if (list_is_last(&req->list, &dep->request_list))
				last_one = 1;

			dwc3_prepare_one_trb(dep, req, dma, length,
					last_one, false, 0, 0);

			dbg_queue(dep->number, &req->request, 0);
			if (last_one)
				break;
		}
	}
}

static int __dwc3_gadget_kick_transfer(struct dwc3_ep *dep, u16 cmd_param,
		int start_new)
{
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_request		*req, *req1, *n;
	struct dwc3			*dwc = dep->dwc;
	int				ret;
	u32				cmd;

	if (start_new && (dep->flags & DWC3_EP_BUSY)) {
		dev_vdbg(dwc->dev, "%s: endpoint busy\n", dep->name);
		return -EBUSY;
	}
	dep->flags &= ~DWC3_EP_PENDING_REQUEST;

	if (start_new) {
		if (list_empty(&dep->req_queued))
			dwc3_prepare_trbs(dep, start_new);

		
		req = next_request(&dep->req_queued);
	} else {
		dwc3_prepare_trbs(dep, start_new);

		req = next_request(&dep->req_queued);
	}
	if (!req) {
		dep->flags |= DWC3_EP_PENDING_REQUEST;
		dbg_event(dep->number, "NO REQ", 0);
		return 0;
	}

	memset(&params, 0, sizeof(params));

	if (start_new) {
		params.param0 = upper_32_bits(req->trb_dma);
		params.param1 = lower_32_bits(req->trb_dma);
		cmd = DWC3_DEPCMD_STARTTRANSFER;
	} else {
		cmd = DWC3_DEPCMD_UPDATETRANSFER;
	}

	cmd |= DWC3_DEPCMD_PARAM(cmd_param);
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	if (ret < 0) {
		dev_dbg(dwc->dev, "failed to send STARTTRANSFER command\n");

		if ((ret == -EAGAIN) && start_new &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			if (!dep->resource_index) {
				dep->resource_index =
					 dwc3_gadget_ep_get_transfer_index(dwc,
								dep->number);
				WARN_ON_ONCE(!dep->resource_index);
			}
			dwc3_stop_active_transfer(dwc, dep->number);
			list_for_each_entry_safe_reverse(req1, n,
						 &dep->req_queued, list) {
				req1->trb = NULL;
				dwc3_gadget_move_request_list_front(req1);
				if (req->request.num_mapped_sgs)
					dep->busy_slot +=
						 req->request.num_mapped_sgs;
				else
					dep->busy_slot++;
				if ((dep->busy_slot & DWC3_TRB_MASK) ==
							DWC3_TRB_NUM - 1)
					dep->busy_slot++;
			}
			return ret;
		} else {
			usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);
			list_del(&req->list);
			return ret;
		}
	}

	dep->flags |= DWC3_EP_BUSY;

	if (start_new) {
		dep->resource_index = dwc3_gadget_ep_get_transfer_index(dwc,
				dep->number);
		WARN_ON_ONCE(!dep->resource_index);

		if (usb_endpoint_xfer_bulk(dep->endpoint.desc) &&
						bulk_ep_xfer_timeout_ms) {
			hrtimer_start(&dep->xfer_timer,
				ktime_set(0, bulk_ep_xfer_timeout_ns),
				HRTIMER_MODE_REL);
		}
	}

	return 0;
}

static void __dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, u32 cur_uf)
{
	u32 uf;
	int ret;

	dep->current_uf = cur_uf;

	if (list_empty(&dep->request_list)) {
		dev_vdbg(dwc->dev, "ISOC ep %s run out for requests.\n",
			dep->name);
		dep->flags |= DWC3_EP_PENDING_REQUEST;
		return;
	}

	if (dwc->revision <= DWC3_REVISION_250A) {
		u32	u1;
		u32	reg;

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		u1 = reg & (DWC3_DCTL_INITU1ENA | DWC3_DCTL_ACCEPTU1ENA);

		if (!dwc->u1)
			dwc->u1 = u1;

		reg &= ~u1;

		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	
	uf = cur_uf + dep->interval * 4;

	ret = __dwc3_gadget_kick_transfer(dep, uf, 1);
	if (ret < 0)
		dbg_event(dep->number, "ISOC QUEUE", ret);
}

static void dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event)
{
	u32 cur_uf, mask;

	mask = ~(dep->interval - 1);
	cur_uf = event->parameters & mask;

	__dwc3_gadget_start_isoc(dwc, dep, cur_uf);
}

static int __dwc3_gadget_ep_queue(struct dwc3_ep *dep, struct dwc3_request *req)
{
	struct dwc3		*dwc = dep->dwc;
	int			ret;

	if (req->request.status == -EINPROGRESS) {
		ret = -EBUSY;
		dev_err(dwc->dev, "%s: %p request already in queue",
					dep->name, req);
		return ret;
	}

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

	ret = usb_gadget_map_request(&dwc->gadget, &req->request,
			dep->direction);
	if (ret)
		return ret;

	list_add_tail(&req->list, &dep->request_list);

	if (dep->flags & DWC3_EP_PENDING_REQUEST) {
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			if (list_empty(&dep->req_queued) && dep->resource_index)
				dwc3_stop_active_transfer(dwc, dep->number);
			else
				__dwc3_gadget_start_isoc(dwc, dep,
							dep->current_uf);
			dep->flags &= ~DWC3_EP_PENDING_REQUEST;
			return 0;
		}

		ret = __dwc3_gadget_kick_transfer(dep, 0, true);
		if (ret && ret != -EBUSY) {
			dbg_event(dep->number, "XfNR QUEUE", ret);
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
		return ret;
	}

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			(dep->flags & DWC3_EP_BUSY) &&
			!(dep->flags & DWC3_EP_MISSED_ISOC)) {
		WARN_ON_ONCE(!dep->resource_index);
		ret = __dwc3_gadget_kick_transfer(dep, dep->resource_index,
				false);
		if (ret && ret != -EBUSY) {
			dbg_event(dep->number, "XfIP QUEUE", ret);
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
		return ret;
	}

	return 0;
}

static int dwc3_gadget_wakeup(struct usb_gadget *g)
{
	struct dwc3_usb_gadget *dwc3_gadget = g->private;

	schedule_work(&dwc3_gadget->wakeup_work);
	return 0;
}

static inline enum dwc3_link_state dwc3_get_link_state(struct dwc3 *dwc)
{
	u32 reg;
	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	return DWC3_DSTS_USBLNKST(reg);
}

static bool dwc3_gadget_is_suspended(struct dwc3 *dwc)
{
	if (atomic_read(&dwc->in_lpm) ||
		dwc3_get_link_state(dwc) == DWC3_LINK_STATE_U3)
		return true;
	return false;
}

static int dwc3_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;
	int				ret;

	spin_lock_irqsave(&dwc->lock, flags);

	if (!dep->endpoint.desc) {
		spin_unlock_irqrestore(&dwc->lock, flags);
		dev_dbg(dwc->dev, "trying to queue request %p to disabled %s\n",
				request, ep->name);
		return -ESHUTDOWN;
	}

	if (dep->endpoint.endless) {
		dev_dbg(dwc->dev, "trying to queue endless request %p to %s\n",
				request, ep->name);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EPERM;
	}

	if (dwc3_gadget_is_suspended(dwc)) {
		if (dwc->gadget.remote_wakeup)
			dwc3_gadget_wakeup(&dwc->gadget);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return dwc->gadget.remote_wakeup ? -EAGAIN : -ENOTSUPP;
	}

	dev_vdbg(dwc->dev, "queing request %p to %s length %d\n",
			request, ep->name, request->length);

	WARN(!dep->direction && (request->length % ep->desc->wMaxPacketSize),
		"trying to queue unaligned request (%d)\n", request->length);

	ret = __dwc3_gadget_ep_queue(dep, req);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_request		*r = NULL;

	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;
	int				ret = 0;

	if (atomic_read(&dwc->in_lpm)) {
		dev_err(dwc->dev, "%s: Unable to dequeue while in LPM\n",
				dep->name);
		return -EAGAIN;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	list_for_each_entry(r, &dep->request_list, list) {
		if (r == req)
			break;
	}

	if (r != req) {
		list_for_each_entry(r, &dep->req_queued, list) {
			if (r == req)
				break;
		}
		if (r == req) {
			
			dwc3_stop_active_transfer(dwc, dep->number);
			goto out1;
		}
		dev_err(dwc->dev, "request %p was not queued to %s\n",
				request, ep->name);
		ret = -EINVAL;
		goto out0;
	}

out1:
	dbg_event(dep->number, "DEQUEUE", 0);
	
	dwc3_gadget_giveback(dep, req, -ECONNRESET);

out0:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

int __dwc3_gadget_ep_set_halt(struct dwc3_ep *dep, int value, int protocol)
{
	struct dwc3_gadget_ep_cmd_params	params;
	struct dwc3				*dwc = dep->dwc;
	int					ret;

	memset(&params, 0x00, sizeof(params));

	if (value) {
		if (!protocol && ((dep->direction && dep->flags & DWC3_EP_BUSY) ||
				(!list_empty(&dep->req_queued) ||
				 !list_empty(&dep->request_list)))) {
			dev_dbg(dwc->dev, "%s: pending request, cannot halt\n",
					dep->name);
			return -EAGAIN;
		}

		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags |= DWC3_EP_STALL;
	} else {
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_CLEARSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags &= ~(DWC3_EP_STALL | DWC3_EP_WEDGE);
	}

	return ret;
}

static int dwc3_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&dwc->lock, flags);

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		dev_err(dwc->dev, "%s is of Isochronous type\n", dep->name);
		ret = -EINVAL;
		goto out;
	}

	dbg_event(dep->number, "HALT", value);
	ret = __dwc3_gadget_ep_set_halt(dep, value, false);
out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;
	unsigned long			flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dbg_event(dep->number, "WEDGE", 0);
	dep->flags |= DWC3_EP_WEDGE;
	spin_unlock_irqrestore(&dwc->lock, flags);

	if (dep->number == 0 || dep->number == 1)
		return dwc3_gadget_ep0_set_halt(ep, 1);
	else
		return __dwc3_gadget_ep_set_halt(dep, 1, false);
}


static struct usb_endpoint_descriptor dwc3_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
};

static const struct usb_ep_ops dwc3_gadget_ep0_ops = {
	.enable		= dwc3_gadget_ep0_enable,
	.disable	= dwc3_gadget_ep0_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep0_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep0_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
};

static const struct usb_ep_ops dwc3_gadget_ep_ops = {
	.enable		= dwc3_gadget_ep_enable,
	.disable	= dwc3_gadget_ep_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
	.nuke           = dwc3_gadget_ep_nuke,
};


static int dwc3_gadget_get_frame(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	return DWC3_DSTS_SOFFN(reg);
}
static void dwc3_gadget_wakeup_work(struct work_struct *w)
{
	struct dwc3_usb_gadget *dwc3_gadget;
	struct dwc3		*dwc;
	int			ret;

	dwc3_gadget = container_of(w, struct dwc3_usb_gadget, wakeup_work);
	dwc = dwc3_gadget->dwc;

	if (atomic_read(&dwc->in_lpm)) {
		dbg_event(0xFF, "Gdgwake gsyn", 0);
		pm_runtime_get_sync(dwc->dev);
	}

	ret = dwc3_gadget_wakeup_int(dwc);

	if (ret)
		pr_err("Remote wakeup failed. ret = %d.\n", ret);
	else
		pr_debug("Remote wakeup succeeded.\n");
}

static int dwc3_gadget_wakeup_int(struct dwc3 *dwc)
{
	bool			link_recover_only = false;

	u32			reg;
	int			ret = 0;
	u8			link_state;
	unsigned long		flags;

	pr_debug("%s(): Entry\n", __func__);
	disable_irq(dwc->irq);
	spin_lock_irqsave(&dwc->lock, flags);
	link_state = dwc3_get_link_state(dwc);

	switch (link_state) {
	case DWC3_LINK_STATE_RX_DET:	
	case DWC3_LINK_STATE_U3:	
		break;
	case DWC3_LINK_STATE_U1:
		if (dwc->gadget.speed != USB_SPEED_SUPER) {
			link_recover_only = true;
			break;
		}
		
	default:
		dev_dbg(dwc->dev, "can't wakeup from link state %d\n",
				link_state);
		dwc3_restart_hrtimer(dwc);
		goto out;
	}

	
	reg = dwc3_readl(dwc->regs, DWC3_DEVTEN);
	reg |= DWC3_DEVTEN_ULSTCNGEN;
	dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);
	mb();

	ret = dwc3_gadget_set_link_state(dwc, DWC3_LINK_STATE_RECOV);
	if (ret < 0) {
		dev_err(dwc->dev, "failed to put link in Recovery\n");
		
		reg = dwc3_readl(dwc->regs, DWC3_DEVTEN);
		reg &= ~DWC3_DEVTEN_ULSTCNGEN;
		dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);
		
		mb();
		goto out;
	}

	
	if (dwc->revision < DWC3_REVISION_194A) {
		
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
	enable_irq(dwc->irq);
	ret = wait_event_interruptible_timeout(dwc->wait_linkstate,
			(dwc->link_state < DWC3_LINK_STATE_U3) ||
			(dwc->link_state == DWC3_LINK_STATE_SS_DIS),
			msecs_to_jiffies(3000)); 

	spin_lock_irqsave(&dwc->lock, flags);
	
	reg = dwc3_readl(dwc->regs, DWC3_DEVTEN);
	reg &= ~DWC3_DEVTEN_ULSTCNGEN;
	dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);
	mb();

	if (!ret) {
		dev_dbg(dwc->dev, "Timeout moving into state(%d)\n",
							dwc->link_state);
		ret = -EINVAL;
		spin_unlock_irqrestore(&dwc->lock, flags);
		goto out1;
	} else {
		ret = 0;
		if (dwc->link_state == DWC3_LINK_STATE_SS_DIS ||
				dwc->gadget.state == USB_STATE_DEFAULT)
			link_recover_only = true;
	}

	if (!link_recover_only)
		dwc3_gadget_wakeup_interrupt(dwc, true);

	spin_unlock_irqrestore(&dwc->lock, flags);
	pr_debug("%s: Exit\n", __func__);
	return ret;

out:
	spin_unlock_irqrestore(&dwc->lock, flags);
	enable_irq(dwc->irq);

out1:
	return ret;
}

static int dwc_gadget_func_wakeup(struct usb_gadget *g, int interface_id)
{
	int ret = 0;
	struct dwc3 *dwc = gadget_to_dwc(g);

	if (!g || (g->speed != USB_SPEED_SUPER))
		return -ENOTSUPP;

	if (dwc3_gadget_is_suspended(dwc)) {
		pr_debug("USB bus is suspended. Scheduling wakeup and returning -EAGAIN.\n");
		dwc3_gadget_wakeup(&dwc->gadget);
		return -EAGAIN;
	}

	if (dwc->revision < DWC3_REVISION_220A) {
		ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_XMIT_FUNCTION, interface_id);
	} else {
		ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_XMIT_DEV, 0x1 | (interface_id << 4));
	}

	if (ret)
		pr_err("Function wakeup HW command failed.\n");
	else
		pr_debug("Function wakeup HW command succeeded.\n");

	return ret;
}

static int dwc3_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->is_selfpowered = !!is_selfpowered;
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

#define DWC3_SOFT_RESET_TIMEOUT	10  
static int dwc3_gadget_run_stop(struct dwc3 *dwc, int is_on)
{
	u32			reg;
	u32			timeout = 500;
	ktime_t start, diff;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	if (is_on) {
		if (dwc->revision <= DWC3_REVISION_187A) {
			reg &= ~DWC3_DCTL_TRGTULST_MASK;
			reg |= DWC3_DCTL_TRGTULST_RX_DET;
		}

		if (dwc->revision >= DWC3_REVISION_194A)
			reg &= ~DWC3_DCTL_KEEP_CONNECT;

		start = ktime_get();
		
		dwc3_writel(dwc->regs, DWC3_DCTL, reg | DWC3_DCTL_CSFTRST);
		do {
			reg = dwc3_readl(dwc->regs, DWC3_DCTL);
			if (!(reg & DWC3_DCTL_CSFTRST))
				break;

			diff = ktime_sub(ktime_get(), start);
			
			if (ktime_to_ms(diff) > DWC3_SOFT_RESET_TIMEOUT) {
				printk_ratelimited(KERN_ERR
					"%s:core Reset Timed Out\n", __func__);
				break;
			}
			cpu_relax();
		} while (true);


		dwc3_event_buffers_setup(dwc);
		dwc3_gadget_restart(dwc);
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= DWC3_DCTL_RUN_STOP;
		dwc->pullups_connected = true;
	} else {
		reg &= ~DWC3_DCTL_RUN_STOP;
		dwc->pullups_connected = false;
		usb_gadget_set_state(&dwc->gadget, USB_STATE_NOTATTACHED);
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);
		if (is_on) {
			if (!(reg & DWC3_DSTS_DEVCTRLHLT))
				break;
		} else {
			if (reg & DWC3_DSTS_DEVCTRLHLT)
				break;
		}
		timeout--;
		if (!timeout) {
			dev_err(dwc->dev, "failed to %s controller\n",
						is_on ? "start" : "stop");
			if (is_on)
				dbg_event(0xFF, "STARTTOUT", reg);
			else
				dbg_event(0xFF, "STOPTOUT", reg);
			return -ETIMEDOUT;
		}
		udelay(1);
	} while (1);

	dev_vdbg(dwc->dev, "gadget %s data soft-%s\n",
			dwc->gadget_driver
			? dwc->gadget_driver->function : "no-function",
			is_on ? "connect" : "disconnect");

	return 0;
}

static int dwc3_gadget_vbus_draw(struct usb_gadget *g, unsigned mA)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	struct dwc3_otg		*dotg = dwc->dotg;

<<<<<<< HEAD
	if (dotg && dotg->otg.phy)
		return usb_phy_set_power(dotg->otg.phy, mA);

	return -ENOTSUPP;
=======
	dwc->vbus_draw = mA;
	dev_dbg(dwc->dev, "Notify controller from %s. mA = %d\n", __func__, mA);
	dwc3_notify_event(dwc, DWC3_CONTROLLER_SET_CURRENT_DRAW_EVENT, 0);
	return 0;
>>>>>>> 0e91d2a... Nougat
}

static int dwc3_gadget_pullup(struct usb_gadget *g, int is_on)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	struct dwc3_usb_gadget *dwc3_gadget = g->private;
	unsigned long		flags;
	int			ret;
	unsigned long		timeout;

	is_on = !!is_on;

	spin_lock_irqsave(&dwc->lock, flags);

	
	if (dwc->enable_bus_suspend)
		usb_phy_set_suspend(dwc->dotg->otg.phy, 0);

	dwc->softconnect = is_on;

	if ((dwc->dotg && !dwc->vbus_active) ||
		!dwc->gadget_driver) {

		spin_unlock_irqrestore(&dwc->lock, flags);

		return 0;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
	if (atomic_read(&dwc->in_lpm) && !is_on) {
		dbg_event(0xFF, "Gdgpull gsyn", 0);
		pm_runtime_get_sync(dwc->dev);
		if (dwc3_gadget)
			dwc3_gadget->disable_during_lpm = true;
		goto set_run_stop;
	}

	if (is_on && dwc3_gadget && dwc3_gadget->disable_during_lpm) {
		dbg_event(0xFF, "Gdgpull psyn", 0);
		pm_runtime_put_sync(dwc->dev);
		if (dwc3_gadget)
			dwc3_gadget->disable_during_lpm = false;
		goto set_run_stop;
	}

<<<<<<< HEAD
	if (atomic_read(&dwc->in_lpm)) {
		pm_runtime_resume(dwc->dev);
		timeout = jiffies + msecs_to_jiffies(20);
		do {
			if (!atomic_read(&dwc->in_lpm))
				break;
=======
	dev_dbg(dwc->dev, "Notify OTG from %s\n", __func__);
	dwc->b_suspend = false;
	dwc3_notify_event(dwc, DWC3_CONTROLLER_NOTIFY_OTG_EVENT, 0);
>>>>>>> 0e91d2a... Nougat

			if (time_after(jiffies, timeout)) {
				pr_err("%s(): Err getting pullup\n", __func__);
				return -ETIMEDOUT;
			}
			usleep_range(2, 5);
		} while (true);
	}

set_run_stop:
	spin_lock_irqsave(&dwc->lock, flags);

	ret = dwc3_gadget_run_stop(dwc, is_on);

	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

void dwc3_gadget_enable_irq(struct dwc3 *dwc)
{
	u32			reg;

	
	reg = (DWC3_DEVTEN_EVNTOVERFLOWEN |
			DWC3_DEVTEN_CMDCMPLTEN |
			DWC3_DEVTEN_ERRTICERREN |
			DWC3_DEVTEN_WKUPEVTEN |
			DWC3_DEVTEN_CONNECTDONEEN |
			DWC3_DEVTEN_USBRSTEN |
			DWC3_DEVTEN_DISCONNEVTEN);

	if (dwc->revision < DWC3_REVISION_230A)
		reg |= DWC3_DEVTEN_ULSTCNGEN;
	else
		reg |= DWC3_DEVTEN_SUSPEND;

	dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);
}

void dwc3_gadget_disable_irq(struct dwc3 *dwc)
{
	
	dwc3_writel(dwc->regs, DWC3_DEVTEN, 0x00);
}

static irqreturn_t dwc3_thread_interrupt(int irq, void *_dwc);
static void dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc);

static int dwc3_gadget_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct dwc3 *dwc = gadget_to_dwc(_gadget);
	struct dwc3_otg		*dotg = dwc->dotg;
	unsigned long flags;

	if (!dwc->dotg)
		return -EPERM;

	is_active = !!is_active;

	spin_lock_irqsave(&dwc->lock, flags);

	
	dwc->vbus_active = is_active;

	if (dwc->gadget_driver && dwc->softconnect) {
		if (dwc->vbus_active) {
			dwc3_gadget_run_stop(dwc, 1);
		} else {
			cancel_delayed_work(&dotg->unknown_charger_notify_work);
			dwc3_gadget_run_stop(dwc, 0);
		}
	}

	if (!dwc->vbus_active) {
		dev_dbg(dwc->dev, "calling disconnect from %s\n", __func__);
		dwc3_gadget_disconnect_interrupt(dwc);
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
	return 0;
}

static int __dwc3_gadget_start(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret = 0;
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_SPEED_MASK);

	if (dwc->revision < DWC3_REVISION_220A) {
		reg |= DWC3_DCFG_SUPERSPEED;
	} else {
		switch (dwc->maximum_speed) {
		case USB_SPEED_LOW:
			reg |= DWC3_DSTS_LOWSPEED;
			break;
		case USB_SPEED_FULL:
			reg |= DWC3_DSTS_FULLSPEED2;
			break;
		case USB_SPEED_HIGH:
			reg |= DWC3_DSTS_HIGHSPEED;
			break;
		case USB_SPEED_SUPER:	
		case USB_SPEED_UNKNOWN:	
		default:
			reg |= DWC3_DSTS_SUPERSPEED;
		}
	}
<<<<<<< HEAD
=======

>>>>>>> 0e91d2a... Nougat
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);

	if (dwc->revision >= DWC3_REVISION_270A) {
		reg = dwc3_readl(dwc->regs, DWC3_GSBUSCFG1);
		reg &= ~DWC3_GSBUSCFG1_PIPETRANSLIMIT_MASK;
		reg |= DWC3_GSBUSCFG1_PIPETRANSLIMIT(0xe);
		dwc3_writel(dwc->regs, DWC3_GSBUSCFG1, reg);
	}

	dwc->start_config_issued = false;

	
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dwc->delayed_status = false;
	
	dep = dwc->eps[0];
	dep->flags = 0;
	dep->endpoint.maxburst = 1;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return ret;
	}

	dep = dwc->eps[1];
	dep->flags = 0;
	dep->endpoint.maxburst = 1;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		__dwc3_gadget_ep_disable(dwc->eps[0]);
		return ret;
	}

	
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);

	dwc3_gadget_enable_irq(dwc);

	return ret;
}

void dwc3_gadget_restart(struct dwc3 *dwc)
{
	__dwc3_gadget_start(dwc);
}

static int dwc3_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			ret = 0;
<<<<<<< HEAD
	int			irq;

	dbg_event(0xFF, "GdgStrt Begin", 0);
	pm_runtime_get_sync(dwc->dev);
	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	dwc->irq = irq;
	ret = request_irq(irq, dwc3_interrupt, IRQF_SHARED, "dwc3", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		goto err0;
	}

=======

	g->interrupt_num = dwc->irq;
>>>>>>> 0e91d2a... Nougat
	spin_lock_irqsave(&dwc->lock, flags);

	if (dwc->gadget_driver) {
		dev_err(dwc->dev, "%s is already bound to %s\n",
				dwc->gadget.name,
				dwc->gadget_driver->driver.name);
		ret = -EBUSY;
		goto err0;
	}

	dwc->gadget_driver	= driver;

	if (!dwc->dotg) {
		ret = __dwc3_gadget_start(dwc);
		if (ret)
			goto err1;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
	pm_runtime_put(dwc->dev);
	dbg_event(0xFF, "GdgStrt End", 0);

	return 0;

<<<<<<< HEAD
err1:
	spin_unlock_irqrestore(&dwc->lock, flags);
	pm_runtime_put(dwc->dev);

err0:
	free_irq(irq, dwc);

=======
err0:
	spin_unlock_irqrestore(&dwc->lock, flags);
>>>>>>> 0e91d2a... Nougat
	return ret;
}

static int dwc3_gadget_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			irq;

	dwc3_gadget_disable_irq(dwc);

	tasklet_kill(&dwc->bh);

	spin_lock_irqsave(&dwc->lock, flags);

	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	dwc->gadget_driver	= NULL;

	spin_unlock_irqrestore(&dwc->lock, flags);

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	free_irq(irq, dwc);

	return 0;
}

<<<<<<< HEAD
=======
static int dwc3_gadget_restart_usb_session(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);

	return dwc3_notify_event(dwc, DWC3_CONTROLLER_RESTART_USB_SESSION, 0);
}

>>>>>>> 0e91d2a... Nougat
static const struct usb_gadget_ops dwc3_gadget_ops = {
	.get_frame		= dwc3_gadget_get_frame,
	.wakeup			= dwc3_gadget_wakeup,
	.func_wakeup		= dwc_gadget_func_wakeup,
	.set_selfpowered	= dwc3_gadget_set_selfpowered,
	.vbus_session		= dwc3_gadget_vbus_session,
	.vbus_draw		= dwc3_gadget_vbus_draw,
	.pullup			= dwc3_gadget_pullup,
	.udc_start		= dwc3_gadget_start,
	.udc_stop		= dwc3_gadget_stop,
};


<<<<<<< HEAD
static enum hrtimer_restart dwc3_gadget_ep_timer(struct hrtimer *hrtimer)
{
	struct dwc3_ep *dep = container_of(hrtimer, struct dwc3_ep, xfer_timer);
	struct dwc3_event_depevt event;
	unsigned long flags;
	struct dwc3 *dwc;

	if (!dep) {
		pr_err("%s: NULL endpoint!\n", __func__);
		goto out;
	}
	if (!(dep->flags & DWC3_EP_ENABLED)) {
		pr_debug("%s: disabled endpoint!\n", __func__);
		goto out;
	}
	if (!dep->endpoint.desc) {
		pr_err("%s: NULL endpoint desc!\n", __func__);
		goto out;
	}
	if (!dep->dwc) {
		pr_err("%s: NULL dwc3 ptr!\n", __func__);
		goto out;
	}
	dwc = dep->dwc;

	event.status = 0;

	spin_lock_irqsave(&dwc->lock, flags);

	if (!(atomic_read(&dwc->in_lpm)))
		dwc3_endpoint_transfer_complete(dep->dwc, dep, &event, 1);

	spin_unlock_irqrestore(&dwc->lock, flags);

out:
	return HRTIMER_NORESTART;
}

static void dwc3_restart_hrtimer(struct dwc3 *dwc)
{
	int i;
	struct dwc3_ep	*dep;

	if (!bulk_ep_xfer_timeout_ms)
		return;

	for (i = 0; i < dwc->num_out_eps; i++) {
		dep = dwc->eps[i];

		
		if (dep && dep->endpoint.desc &&
			usb_endpoint_xfer_bulk(dep->endpoint.desc) &&
			!dep->direction && (dep->flags & DWC3_EP_ENABLED)) {

			if (!hrtimer_active(&dep->xfer_timer)) {
				pr_debug("%s(): restart hrtimer of (%s)\n",
						__func__, dep->name);
				hrtimer_start(&dep->xfer_timer,
					ktime_set(0, bulk_ep_xfer_timeout_ns),
					HRTIMER_MODE_REL);
			}
		}
	}
}
=======
#define NUM_GSI_OUT_EPS	1
#define NUM_GSI_IN_EPS	2
>>>>>>> 0e91d2a... Nougat

static int dwc3_gadget_init_hw_endpoints(struct dwc3 *dwc,
		u8 num, u32 direction)
{
	struct dwc3_ep			*dep;
	u8				i, gsi_ep_count, gsi_ep_index = 0;

	gsi_ep_count = dwc->num_gsi_event_buffers;
	
	if (gsi_ep_count && !direction)
		gsi_ep_count = NUM_GSI_OUT_EPS;
	
	else if (gsi_ep_count && direction)
		gsi_ep_count = NUM_GSI_IN_EPS;

	for (i = 0; i < num; i++) {
		u8 epnum = (i << 1) | (!!direction);

		dep = kzalloc(sizeof(*dep), GFP_KERNEL);
		if (!dep) {
			dev_err(dwc->dev, "can't allocate endpoint %d\n",
					epnum);
			return -ENOMEM;
		}

		dep->dwc = dwc;
		dep->number = epnum;
		dwc->eps[epnum] = dep;

		
		if ((gsi_ep_index < gsi_ep_count) &&
				(i > (num - 1 - gsi_ep_count))) {
			gsi_ep_index++;
			
			snprintf(dep->name, sizeof(dep->name), "%s",
				(epnum & 1) ? "gsi-epin" : "gsi-epout");
			
			dep->endpoint.ep_type = EP_TYPE_GSI;
		} else {
			snprintf(dep->name, sizeof(dep->name), "ep%d%s",
				epnum >> 1, (epnum & 1) ? "in" : "out");
		}

		dep->endpoint.ep_num = epnum >> 1;
		dep->endpoint.name = dep->name;
<<<<<<< HEAD
		dep->direction = (epnum & 1);
=======

		dev_vdbg(dwc->dev, "initializing %s %d\n",
				dep->name, epnum >> 1);
>>>>>>> 0e91d2a... Nougat

		if (epnum == 0 || epnum == 1) {
			dep->endpoint.maxpacket = 512;
			dep->endpoint.maxburst = 1;
			dep->endpoint.ops = &dwc3_gadget_ep0_ops;
			if (!epnum)
				dwc->gadget.ep0 = &dep->endpoint;
		} else {
			int		ret;

			dep->endpoint.maxpacket = 1024;
			dep->endpoint.max_streams = 15;
			dep->endpoint.ops = &dwc3_gadget_ep_ops;
			list_add_tail(&dep->endpoint.ep_list,
					&dwc->gadget.ep_list);

			ret = dwc3_alloc_trb_pool(dep);
			if (ret)
				return ret;
		}

		INIT_LIST_HEAD(&dep->request_list);
		INIT_LIST_HEAD(&dep->req_queued);

		hrtimer_init(&dep->xfer_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
		dep->xfer_timer.function = dwc3_gadget_ep_timer;
	}

	return 0;
}

static int dwc3_gadget_init_endpoints(struct dwc3 *dwc)
{
	int				ret;

	INIT_LIST_HEAD(&dwc->gadget.ep_list);

	ret = dwc3_gadget_init_hw_endpoints(dwc, dwc->num_out_eps, 0);
	if (ret < 0) {
		dev_vdbg(dwc->dev, "failed to allocate OUT endpoints\n");
		return ret;
	}

	ret = dwc3_gadget_init_hw_endpoints(dwc, dwc->num_in_eps, 1);
	if (ret < 0) {
		dev_vdbg(dwc->dev, "failed to allocate IN endpoints\n");
		return ret;
	}

	return 0;
}

static void dwc3_gadget_free_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	u8				epnum;

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		dep = dwc->eps[epnum];
		if (!dep)
			continue;
		if (epnum != 0 && epnum != 1) {
			dwc3_free_trb_pool(dep);
			list_del(&dep->endpoint.ep_list);
		}

		kfree(dep);
	}
}


static int __dwc3_cleanup_done_trbs(struct dwc3 *dwc, struct dwc3_ep *dep,
		struct dwc3_request *req, struct dwc3_trb *trb,
		const struct dwc3_event_depevt *event, int status)
{
	unsigned int		count;
	unsigned int		s_pkt = 0;
	unsigned int		trb_status;

	if ((trb->ctrl & DWC3_TRB_CTRL_HWO) && status != -ESHUTDOWN)
		/*
		 * We continue despite the error. There is not much we
		 * can do. If we don't clean it up we loop forever. If
		 * we skip the TRB then it gets overwritten after a
		 * while since we use them in a ring buffer. A BUG()
		 * would help. Lets hope that if this occurs, someone
		 * fixes the root cause instead of looking away :)
		 */
		dev_err(dwc->dev, "%s's TRB (%p) still owned by HW\n",
				dep->name, trb);
	count = trb->size & DWC3_TRB_SIZE_MASK;

	if (dep->direction) {
		if (count) {
			trb_status = DWC3_TRB_SIZE_TRBSTS(trb->size);
			if (trb_status == DWC3_TRBSTS_MISSED_ISOC) {
				dev_dbg(dwc->dev, "incomplete IN transfer %s\n",
						dep->name);
				dep->flags |= DWC3_EP_MISSED_ISOC;
				dbg_event(dep->number, "MISSED ISOC", status);
			} else {
				dev_err(dwc->dev, "incomplete IN transfer %s\n",
						dep->name);
				status = -ECONNRESET;
			}
		} else {
			dep->flags &= ~DWC3_EP_MISSED_ISOC;
		}
	} else {
		if (count && (event->status & DEPEVT_STATUS_SHORT) &&
			!(trb->ctrl & DWC3_TRB_CTRL_CSP))
			s_pkt = 1;
	}

	req->request.actual += req->request.length - count;
	if (s_pkt)
		return 1;
	if ((event->status & DEPEVT_STATUS_LST) &&
			(trb->ctrl & (DWC3_TRB_CTRL_LST |
				DWC3_TRB_CTRL_HWO)))
		return 1;
	if ((event->status & DEPEVT_STATUS_IOC) &&
			(trb->ctrl & DWC3_TRB_CTRL_IOC))
		return 1;
	return 0;
}

static int dwc3_cleanup_done_reqs(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct dwc3_event_depevt *event, int status)
{
	struct dwc3_request	*req;
	struct dwc3_trb		*trb;
	unsigned int		slot;
	unsigned int		i;
	int			ret;

	do {
		req = next_request(&dep->req_queued);
		if (!req) {
			if (event->status)
				dev_err(dwc->dev,
					"%s: evt sts %x for no req queued",
					dep->name, event->status);
			return 1;
		}

		if (usb_endpoint_xfer_bulk(dep->endpoint.desc) &&
			(req->trb->ctrl & DWC3_TRB_CTRL_HWO))
			return 0;

		i = 0;
		do {
			slot = req->start_slot + i;
			if ((slot == DWC3_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				slot++;
			slot %= DWC3_TRB_NUM;
			trb = &dep->trb_pool[slot];

			ret = __dwc3_cleanup_done_trbs(dwc, dep, req, trb,
					event, status);
			if (ret)
				break;
		}while (++i < req->request.num_mapped_sgs);

		if (req->ztrb) {
			trb = req->ztrb;
			if ((event->status & DEPEVT_STATUS_LST) &&
				(trb->ctrl & (DWC3_TRB_CTRL_LST |
					DWC3_TRB_CTRL_HWO)))
				ret = 1;

			if ((event->status & DEPEVT_STATUS_IOC) &&
					(trb->ctrl & DWC3_TRB_CTRL_IOC))
				ret = 1;
		}
		dwc3_gadget_giveback(dep, req, status);

		
		if (!(dep->flags & DWC3_EP_ENABLED)) {
			dev_dbg(dwc->dev, "%s disabled while handling ep event\n",
					dep->name);
			return 0;
		}

		if (ret)
			break;
	} while (1);

	dwc->gadget.xfer_isr_count++;

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			list_empty(&dep->req_queued)) {
		if (list_empty(&dep->request_list))
			dep->flags |= DWC3_EP_PENDING_REQUEST;
		else
			dwc3_stop_active_transfer(dwc, dep->number);
		dep->flags &= ~DWC3_EP_MISSED_ISOC;
		return 1;
	}

	if ((event->status & DEPEVT_STATUS_IOC) &&
			(trb->ctrl & DWC3_TRB_CTRL_IOC))
		return 0;
	return 1;
}

static void dwc3_endpoint_transfer_complete(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event,
		int start_new)
{
	unsigned		status = 0;
	int			clean_busy;

	if (event->status & DEPEVT_STATUS_BUSERR)
		status = -ECONNRESET;

	clean_busy = dwc3_cleanup_done_reqs(dwc, dep, event, status);
	if (clean_busy)
		dep->flags &= ~DWC3_EP_BUSY;

	if (bulk_ep_xfer_timeout_ms && dep->endpoint.desc &&
		usb_endpoint_xfer_bulk(dep->endpoint.desc)) {

		if (event->status & DEPEVT_STATUS_LST)
			hrtimer_try_to_cancel(&dep->xfer_timer);
		else if (!clean_busy && (dwc->link_state != DWC3_LINK_STATE_U3))
			hrtimer_start(&dep->xfer_timer,
				ktime_set(0, bulk_ep_xfer_timeout_ns),
				HRTIMER_MODE_REL);
	}

	if (dwc->revision < DWC3_REVISION_183A) {
		u32		reg;
		int		i;

		for (i = 0; i < DWC3_ENDPOINTS_NUM; i++) {
			dep = dwc->eps[i];

			if (!(dep->flags & DWC3_EP_ENABLED))
				continue;

			if (!list_empty(&dep->req_queued))
				goto isoc_workaround;
		}

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= (dwc->u1u2 & ~(dwc->u1));
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

		dwc->u1u2 = 0;
	}

isoc_workaround:
	if (dwc->revision <= DWC3_REVISION_250A) {
		u32		reg;
		int		i;

		for (i = 0; i < DWC3_ENDPOINTS_NUM; i++) {
			dep = dwc->eps[i];

			if (dep->endpoint.desc == NULL)
				continue;

			if (!(dep->flags & DWC3_EP_ENABLED) ||
				!usb_endpoint_xfer_isoc(dep->endpoint.desc))
				continue;

			if (!list_empty(&dep->req_queued))
				return;
		}

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= dwc->u1;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

		dwc->u1 = 0;
	}
}

static void dwc3_endpoint_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_depevt *event)
{
	struct dwc3_ep		*dep;
	u8			epnum = event->endpoint_number;

	dep = dwc->eps[epnum];

	if (!(dep->flags & DWC3_EP_ENABLED))
		return;

	dev_vdbg(dwc->dev, "%s: %s\n", dep->name,
			dwc3_ep_event_string(event->endpoint_event));

	if (epnum == 0 || epnum == 1) {
		dwc3_ep0_interrupt(dwc, event);
		return;
	}

	dep->dbg_ep_events.total++;

	switch (event->endpoint_event) {
	case DWC3_DEPEVT_XFERCOMPLETE:
		dep->resource_index = 0;
		dep->dbg_ep_events.xfercomplete++;

		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is an Isochronous endpoint\n",
					dep->name);
			return;
		}

		dwc3_endpoint_transfer_complete(dwc, dep, event, 1);
		break;
	case DWC3_DEPEVT_XFERINPROGRESS:
		if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is not an Isochronous endpoint\n",
					dep->name);
			if (!dep->endpoint.is_ncm)
				return;
		}
		dwc3_endpoint_transfer_complete(dwc, dep, event, 0);
		break;
	case DWC3_DEPEVT_XFERNOTREADY:
		dep->dbg_ep_events.xfernotready++;
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dwc3_gadget_start_isoc(dwc, dep, event);
		} else {
			int ret;

			dev_vdbg(dwc->dev, "%s: reason %s\n",
					dep->name, event->status &
					DEPEVT_STATUS_TRANSFER_ACTIVE
					? "Transfer Active"
					: "Transfer Not Active");

			ret = __dwc3_gadget_kick_transfer(dep, 0, 1);
			if (!ret || ret == -EBUSY)
				return;

			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}

		break;
	case DWC3_DEPEVT_STREAMEVT:
		dep->dbg_ep_events.streamevent++;
		if (!usb_endpoint_xfer_bulk(dep->endpoint.desc)) {
			dev_err(dwc->dev, "Stream event for non-Bulk %s\n",
					dep->name);
			return;
		}

		switch (event->status) {
		case DEPEVT_STREAMEVT_FOUND:
			dev_vdbg(dwc->dev, "Stream %d found and started\n",
					event->parameters);

			break;
		case DEPEVT_STREAMEVT_NOTFOUND:
			
		default:
			dev_dbg(dwc->dev, "Couldn't find suitable stream\n");
		}
		break;
	case DWC3_DEPEVT_RXTXFIFOEVT:
		dev_dbg(dwc->dev, "%s FIFO Overrun\n", dep->name);
		dep->dbg_ep_events.rxtxfifoevent++;
		break;
	case DWC3_DEPEVT_EPCMDCMPLT:
		dev_vdbg(dwc->dev, "Endpoint Command Complete\n");
		dep->dbg_ep_events.epcmdcomplete++;
		break;
	}
}
static void dwc3_disconnect_gadget(struct dwc3 *dwc, int mute)
{
	if (dwc->gadget_driver && dwc->gadget_driver->disconnect) {
		spin_unlock(&dwc->lock);
		if (mute)
			dwc->gadget_driver->mute_disconnect(&dwc->gadget);
		else
			dwc->gadget_driver->disconnect(&dwc->gadget);
		spin_lock(&dwc->lock);
	}
	dwc->gadget.xfer_isr_count = 0;
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum)
{
	struct dwc3_ep *dep;
	struct dwc3_gadget_ep_cmd_params params;
	u32 cmd;
	int ret;

	dep = dwc->eps[epnum];

	if (!dep->resource_index)
		return;

	if (dep->endpoint.endless)
		dwc3_notify_event(dwc, DWC3_CONTROLLER_NOTIFY_DISABLE_UPDXFER,
								dep->number);


	cmd = DWC3_DEPCMD_ENDTRANSFER;
	cmd |= DWC3_DEPCMD_HIPRI_FORCERM | DWC3_DEPCMD_CMDIOC;
	cmd |= DWC3_DEPCMD_PARAM(dep->resource_index);
	memset(&params, 0, sizeof(params));
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	dep->resource_index = 0;
	dep->flags &= ~DWC3_EP_BUSY;
	udelay(100);
}

static void dwc3_stop_active_transfers(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 2; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;

		dep = dwc->eps[epnum];
		if (!dep)
			continue;

		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		dwc3_remove_requests(dwc, dep);
	}
}

static void dwc3_clear_stall_all_ep(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 1; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;
		struct dwc3_gadget_ep_cmd_params params;
		int ret;

		dep = dwc->eps[epnum];
		if (!dep)
			continue;

		if (!(dep->flags & DWC3_EP_STALL))
			continue;

		dep->flags &= ~DWC3_EP_STALL;

		memset(&params, 0, sizeof(params));
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_CLEARSTALL, &params);
		if (ret) {
			dev_dbg(dwc->dev, "%s; send ep cmd CLEARSTALL failed",
				dep->name);
			dbg_event(dep->number, "ECLRSTALL", ret);
		}
	}
}

static void dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc)
{
	int			reg;

<<<<<<< HEAD
	dev_vdbg(dwc->dev, "%s\n", __func__);

	
	if (dwc->enable_bus_suspend)
		usb_phy_set_suspend(dwc->dotg->otg.phy, 0);
=======
	dev_dbg(dwc->dev, "Notify OTG from %s\n", __func__);
	dwc->b_suspend = false;
	dwc3_notify_event(dwc, DWC3_CONTROLLER_NOTIFY_OTG_EVENT, 0);
>>>>>>> 0e91d2a... Nougat

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_INITU1ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	reg &= ~DWC3_DCTL_INITU2ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	dbg_event(0xFF, "DISCONNECT", 0);
	dwc3_disconnect_gadget(dwc, 0);
	dwc->start_config_issued = false;

	dwc->gadget.speed = USB_SPEED_UNKNOWN;
	dwc->setup_packet_pending = false;
	dwc->link_state = DWC3_LINK_STATE_SS_DIS;
	wake_up_interruptible(&dwc->wait_linkstate);
}

void dwc3_gadget_usb3_phy_suspend(struct dwc3 *dwc, int suspend)
{
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	if (suspend)
		reg |= DWC3_GUSB3PIPECTL_SUSPHY;
	else
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
}

static void dwc3_gadget_reset_interrupt(struct dwc3 *dwc)
{
	u32			reg;
	struct dwc3_otg		*dotg = dwc->dotg;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	if (dwc->revision < DWC3_REVISION_188A) {
		if (dwc->setup_packet_pending)
			dwc3_gadget_disconnect_interrupt(dwc);
	}

<<<<<<< HEAD
	
	if (dwc->enable_bus_suspend)
		usb_phy_set_suspend(dwc->dotg->otg.phy, 0);
=======
	dev_dbg(dwc->dev, "Notify OTG from %s\n", __func__);
	dwc->b_suspend = false;
	dwc3_notify_event(dwc, DWC3_CONTROLLER_NOTIFY_OTG_EVENT, 0);
>>>>>>> 0e91d2a... Nougat

	dbg_event(0xFF, "BUS RST", 0);
	
	usb_gadget_set_state(&dwc->gadget, USB_STATE_DEFAULT);

	dwc3_gadget_usb3_phy_suspend(dwc, false);

	if (dotg && dotg->otg.phy)
		usb_phy_set_power(dotg->otg.phy, 0);

	if (dwc->gadget.speed != USB_SPEED_UNKNOWN)
		dwc3_disconnect_gadget(dwc, 1);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	dwc->test_mode = false;

	dwc3_stop_active_transfers(dwc);
	dwc3_clear_stall_all_ep(dwc);
	dwc->start_config_issued = false;

	
	dwc->resize_fifos = 0;

	
	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_DEVADDR_MASK);
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);
	dwc->gadget.speed = USB_SPEED_UNKNOWN;
	dwc->link_state = DWC3_LINK_STATE_U0;
	wake_up_interruptible(&dwc->wait_linkstate);
}

static void dwc3_update_ram_clk_sel(struct dwc3 *dwc, u32 speed)
{
	u32 reg;
	u32 usb30_clock = DWC3_GCTL_CLK_BUS;


	if (speed != DWC3_DSTS_SUPERSPEED)
		return;

	if (!usb30_clock)
		return;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg |= DWC3_GCTL_RAMCLKSEL(usb30_clock);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static const char *speed_to_string(enum usb_device_speed speed_type)
{
	switch (speed_type) {
	case USB_SPEED_SUPER:	return "SUPERSPEED";
	case USB_SPEED_HIGH:	return "HIGHSPEED";
	case USB_SPEED_FULL:	return "FULLSPEED";
	case USB_SPEED_LOW:		return "LOWSPEED";
	case USB_SPEED_UNKNOWN:
	default:				return "UNKNOWN SPEED";
	}
}

static void dwc3_gadget_conndone_interrupt(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret;
	u32			reg;
	u8			speed;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	speed = reg & DWC3_DSTS_CONNECTSPD;
	dwc->speed = speed;

	dwc3_update_ram_clk_sel(dwc, speed);

	switch (speed) {
	case DWC3_DCFG_SUPERSPEED:
		if (dwc->revision < DWC3_REVISION_190A)
			dwc3_gadget_reset_interrupt(dwc);

		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		dwc->gadget.ep0->maxpacket = 512;
		dwc->gadget.speed = USB_SPEED_SUPER;
		break;
	case DWC3_DCFG_HIGHSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_HIGH;
		break;
	case DWC3_DCFG_FULLSPEED2:
	case DWC3_DCFG_FULLSPEED1:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_FULL;
		break;
	case DWC3_DCFG_LOWSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(8);
		dwc->gadget.ep0->maxpacket = 8;
		dwc->gadget.speed = USB_SPEED_LOW;
		break;
	}

	USB_INFO("%s\n", speed_to_string(dwc->gadget.speed));

	

	if ((dwc->revision > DWC3_REVISION_194A)
			&& (speed != DWC3_DCFG_SUPERSPEED)) {
		reg = dwc3_readl(dwc->regs, DWC3_DCFG);
		reg |= DWC3_DCFG_LPM_CAP;
		dwc3_writel(dwc->regs, DWC3_DCFG, reg);

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~(DWC3_DCTL_HIRD_THRES_MASK | DWC3_DCTL_L1_HIBER_EN);
		reg |= DWC3_DCTL_HIRD_THRES(dwc->hird_thresh);

		if (dwc->revision >= DWC3_REVISION_240A &&
					dwc->lpm_nyet_thresh) {
			reg &= ~DWC3_DCTL_LPM_NYET_THRES_MASK;
			reg |= DWC3_DCTL_LPM_NYET_THRES(dwc->lpm_nyet_thresh);
		}
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	dwc3_gadget_usb3_phy_suspend(dwc, true);

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dwc3_notify_event(dwc, DWC3_CONTROLLER_CONNDONE_EVENT, 0);

}

static void dwc3_gadget_wakeup_interrupt(struct dwc3 *dwc, bool remote_wakeup)
{
	bool perform_resume = true;

	dev_dbg(dwc->dev, "%s\n", __func__);

	if (!remote_wakeup && dwc->link_state != DWC3_LINK_STATE_U3)
		perform_resume = false;

	
	if (perform_resume) {
		dbg_event(0xFF, "WAKEUP", 0);

<<<<<<< HEAD
		
		if (dwc->enable_bus_suspend)
			usb_phy_set_suspend(dwc->dotg->otg.phy, 0);

		
		dwc3_restart_hrtimer(dwc);
=======
		dev_dbg(dwc->dev, "Notify OTG from %s\n", __func__);
		dwc->b_suspend = false;
		dwc3_notify_event(dwc,
				DWC3_CONTROLLER_NOTIFY_OTG_EVENT, 0);
>>>>>>> 0e91d2a... Nougat

		dwc->link_state = DWC3_LINK_STATE_U0;

		spin_unlock(&dwc->lock);
		dwc->gadget_driver->resume(&dwc->gadget);
		spin_lock(&dwc->lock);
		return;
	}

	dwc->link_state = DWC3_LINK_STATE_U0;
}

static void dwc3_gadget_linksts_change_interrupt(struct dwc3 *dwc,
		unsigned int evtinfo)
{
	enum dwc3_link_state	next = evtinfo & DWC3_LINK_STATE_MASK;
	unsigned int		pwropt;

	pwropt = DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1);
	if ((dwc->revision <= DWC3_REVISION_250A) &&
			(pwropt != DWC3_GHWPARAMS1_EN_PWROPT_HIB)) {
		if ((dwc->link_state == DWC3_LINK_STATE_U3) &&
				(next == DWC3_LINK_STATE_RESUME)) {
			dev_vdbg(dwc->dev, "ignoring transition U3 -> Resume\n");
			return;
		}
	}

	if (dwc->revision < DWC3_REVISION_183A) {
		if (next == DWC3_LINK_STATE_U0) {
			u32	u1u2;
			u32	reg;

			switch (dwc->link_state) {
			case DWC3_LINK_STATE_U1:
			case DWC3_LINK_STATE_U2:
				reg = dwc3_readl(dwc->regs, DWC3_DCTL);
				u1u2 = reg & (DWC3_DCTL_INITU2ENA
						| DWC3_DCTL_ACCEPTU2ENA
						| DWC3_DCTL_INITU1ENA
						| DWC3_DCTL_ACCEPTU1ENA);

				if (!dwc->u1u2)
					dwc->u1u2 = reg & u1u2;

				reg &= ~u1u2;

				dwc3_writel(dwc->regs, DWC3_DCTL, reg);
				break;
			default:
				
				break;
			}
		}
	}

	dev_dbg(dwc->dev, "Going from (%d)--->(%d)\n", dwc->link_state, next);
	dwc->link_state = next;
	wake_up_interruptible(&dwc->wait_linkstate);
	dev_vdbg(dwc->dev, "%s link %d\n", __func__, dwc->link_state);
}

static void dwc3_gadget_suspend_interrupt(struct dwc3 *dwc,
			unsigned int evtinfo)
{
	enum dwc3_link_state    next = evtinfo & DWC3_LINK_STATE_MASK;

	dev_dbg(dwc->dev, "%s Entry\n", __func__);

	if (dwc->link_state != next && next == DWC3_LINK_STATE_U3) {
		dbg_event(0xFF, "SUSPEND", 0);
		if (dwc->gadget.state != USB_STATE_CONFIGURED) {
			pr_err("%s(): state:%d. Ignore SUSPEND.\n",
						__func__, dwc->gadget.state);
			return;
		}
		spin_unlock(&dwc->lock);
		dwc->gadget_driver->suspend(&dwc->gadget);
		spin_lock(&dwc->lock);

		if (dwc->enable_bus_suspend) {
			dev_dbg(dwc->dev, "%s: Triggering OTG suspend\n",
				__func__);

<<<<<<< HEAD
			usb_phy_set_suspend(dwc->dotg->otg.phy, 1);
		}
=======
		dev_dbg(dwc->dev, "Notify OTG from %s\n", __func__);
		dwc->b_suspend = true;
		dwc3_notify_event(dwc, DWC3_CONTROLLER_NOTIFY_OTG_EVENT, 0);
>>>>>>> 0e91d2a... Nougat
	}

	dwc->link_state = next;
	dev_vdbg(dwc->dev, "%s link %d\n", __func__, dwc->link_state);
}

static void dwc3_dump_reg_info(struct dwc3 *dwc)
{
	dbg_event(0xFF, "REGDUMP", 0);

	dbg_print_reg("GUSB3PIPCTL", dwc3_readl(dwc->regs,
							DWC3_GUSB3PIPECTL(0)));
	dbg_print_reg("GUSB2PHYCONFIG", dwc3_readl(dwc->regs,
							DWC3_GUSB2PHYCFG(0)));
	dbg_print_reg("GCTL", dwc3_readl(dwc->regs, DWC3_GCTL));
	dbg_print_reg("GUCTL", dwc3_readl(dwc->regs, DWC3_GUCTL));
	dbg_print_reg("GDBGLTSSM", dwc3_readl(dwc->regs, DWC3_GDBGLTSSM));
	dbg_print_reg("DCFG", dwc3_readl(dwc->regs, DWC3_DCFG));
	dbg_print_reg("DCTL", dwc3_readl(dwc->regs, DWC3_DCTL));
	dbg_print_reg("DEVTEN", dwc3_readl(dwc->regs, DWC3_DEVTEN));
	dbg_print_reg("DSTS", dwc3_readl(dwc->regs, DWC3_DSTS));
	dbg_print_reg("DALPENA", dwc3_readl(dwc->regs, DWC3_DALEPENA));
	dbg_print_reg("DGCMD", dwc3_readl(dwc->regs, DWC3_DGCMD));

	dbg_print_reg("OCFG", dwc3_readl(dwc->regs, DWC3_OCFG));
	dbg_print_reg("OCTL", dwc3_readl(dwc->regs, DWC3_OCTL));
	dbg_print_reg("OEVT", dwc3_readl(dwc->regs, DWC3_OEVT));
	dbg_print_reg("OSTS", dwc3_readl(dwc->regs, DWC3_OSTS));
}

extern void usb_set_connect_type(int);
void htc_dwc3_disable_usb(int disable_usb);
static void dwc3_gadget_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_devt *event)
{
	struct dwc3_otg *dotg = dwc->dotg;
	switch (event->type) {
	case DWC3_DEVICE_EVENT_DISCONNECT:
		dwc3_gadget_disconnect_interrupt(dwc);
		dwc->dbg_gadget_events.disconnect++;
		break;
	case DWC3_DEVICE_EVENT_RESET:
		USB_INFO("reset\n");
		dwc3_gadget_reset_interrupt(dwc);
		dwc->dbg_gadget_events.reset++;
		usb_set_connect_type(CONNECT_TYPE_USB);
		cancel_delayed_work(&dotg->unknown_charger_notify_work); 
		if (dotg->charger && dotg->charger->usb_disable) {
			dotg->notify_usb_disabled();
		}
		break;
	case DWC3_DEVICE_EVENT_CONNECT_DONE:
		dwc3_gadget_conndone_interrupt(dwc);
		dwc->dbg_gadget_events.connect++;
		break;
	case DWC3_DEVICE_EVENT_WAKEUP:
		dwc3_gadget_wakeup_interrupt(dwc, false);
		dwc->dbg_gadget_events.wakeup++;
		break;
	case DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE:
		dwc3_gadget_linksts_change_interrupt(dwc, event->event_info);
		dwc->dbg_gadget_events.link_status_change++;
		break;
	case DWC3_DEVICE_EVENT_SUSPEND:
		USB_INFO("suspend\n");
		cancel_delayed_work(&dotg->unknown_charger_notify_work);
		queue_delayed_work(system_nrt_wq, &dotg->unknown_charger_notify_work, HZ * 10);

		if (dwc->revision < DWC3_REVISION_230A) {
			dev_vdbg(dwc->dev, "End of Periodic Frame\n");
			dwc->dbg_gadget_events.eopf++;
		} else {
			dev_vdbg(dwc->dev, "U3/L1-L2 Suspend Event\n");
			dbg_event(0xFF, "GAD SUS", 0);
			dwc->dbg_gadget_events.suspend++;

			if (dwc->gadget.speed != USB_SPEED_UNKNOWN &&
				dwc->vbus_active)
					dwc3_gadget_suspend_interrupt(dwc,
							event->event_info);
		}
		break;
	case DWC3_DEVICE_EVENT_SOF:
		dev_vdbg(dwc->dev, "Start of Periodic Frame\n");
		dwc->dbg_gadget_events.sof++;
		break;
	case DWC3_DEVICE_EVENT_ERRATIC_ERROR:
		if (!dwc->err_evt_seen) {
			dbg_event(0xFF, "ERROR", 0);
			dev_vdbg(dwc->dev, "Erratic Error\n");
			dwc3_dump_reg_info(dwc);
		}
		dwc->dbg_gadget_events.erratic_error++;
		break;
	case DWC3_DEVICE_EVENT_CMD_CMPL:
		dev_vdbg(dwc->dev, "Command Complete\n");
		dwc->dbg_gadget_events.cmdcmplt++;
		break;
	case DWC3_DEVICE_EVENT_OVERFLOW:
		dbg_event(0xFF, "OVERFL", 0);
		dev_vdbg(dwc->dev, "Overflow\n");
		/*
		 * Controllers prior to 2.30a revision has a bug where
		 * Overflow Event may overwrite an unacknowledged event
		 * in the event buffer.  The severity of the issue depends
		 * on the overwritten event type.  Add a warning message
		 * saying that an event is overwritten.
		 *
		 * TODO: In future we may need to see if we can re-enumerate
		 * with host.
		 */
		if (dwc->revision < DWC3_REVISION_230A)
			dev_warn(dwc->dev, "Unacknowledged event overwritten\n");
		dwc->dbg_gadget_events.overflow++;
		break;
	case DWC3_DEVICE_EVENT_VENDOR_DEV_TEST_LMP:
		dbg_event(0xFF, "TSTLMP", 0);
		if (dwc->revision < DWC3_REVISION_230A)
			dev_warn(dwc->dev, "Vendor Device Test LMP Received\n");
		dwc->dbg_gadget_events.vendor_dev_test_lmp++;
		break;
	default:
		dev_dbg(dwc->dev, "UNKNOWN IRQ %d\n", event->type);
		dwc->dbg_gadget_events.unknown_event++;
	}

	dwc->err_evt_seen = (event->type == DWC3_DEVICE_EVENT_ERRATIC_ERROR);
}

static void dwc3_process_event_entry(struct dwc3 *dwc,
		const union dwc3_event *event)
{
	
	if (!dwc->vbus_active) {
		dbg_print_reg("SKIP EVT", event->raw);
		return;
	}

	
	if (event->type.is_devspec == 0) {
		
		return dwc3_endpoint_interrupt(dwc, &event->depevt);
	}

	switch (event->type.type) {
	case DWC3_EVENT_TYPE_DEV:
		dwc3_gadget_interrupt(dwc, &event->devt);
		break;
	
	default:
		dev_err(dwc->dev, "UNKNOWN IRQ type %d\n", event->raw);
	}
}

static irqreturn_t dwc3_thread_interrupt(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	int i;
	unsigned temp_cnt = 0, temp_time;
	ktime_t start_time;

<<<<<<< HEAD
	start_time = ktime_get();
=======
		if (dwc->err_evt_seen) {
			evt->lpos = (evt->lpos + left) %
					DWC3_EVENT_BUFFERS_SIZE;
			dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(buf), left);
			if (dwc3_notify_event(dwc,
						DWC3_CONTROLLER_ERROR_EVENT, 0))
				dwc->err_evt_seen = 0;
			break;
		}
>>>>>>> 0e91d2a... Nougat

	spin_lock_irqsave(&dwc->lock, flags);

	for (i = 0; i < dwc->num_event_buffers; i++) {
		struct dwc3_event_buffer *evt;
		int			left;

		evt = dwc->ev_buffs[i];
		left = evt->count;
		temp_cnt = temp_cnt + evt->count;

		if (!(evt->flags & DWC3_EVENT_PENDING))
			continue;

		while (left > 0) {
			union dwc3_event event;

			event.raw = *(u32 *) (evt->buf + evt->lpos);
#if defined(CONFIG_HTC_DEBUG_RTB)
			uncached_logk(LOGK_LOGBUF,(void *)((unsigned long int) event.raw));
#endif

			dwc3_process_event_entry(dwc, &event);

<<<<<<< HEAD
			if (dwc->err_evt_seen) {
				evt->lpos = (evt->lpos + left) %
						DWC3_EVENT_BUFFERS_SIZE;
				dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(i),
						left);
				if (dwc3_notify_event(dwc,
						DWC3_CONTROLLER_ERROR_EVENT))
					dwc->err_evt_seen = 0;
				break;
			}
=======
	dwc3_thread_interrupt(dwc->irq, dwc);
	enable_irq(dwc->irq);
}
>>>>>>> 0e91d2a... Nougat

			evt->lpos = (evt->lpos + 4) % DWC3_EVENT_BUFFERS_SIZE;
			left -= 4;

			dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(i), 4);
		}

		evt->count = 0;
		evt->flags &= ~DWC3_EVENT_PENDING;
		ret = IRQ_HANDLED;

		if (dwc->err_evt_seen)
			break;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	temp_time = ktime_to_us(ktime_sub(ktime_get(), start_time));
	temp_cnt = temp_cnt / 4;
	dwc->bh_handled_evt_cnt[dwc->bh_dbg_index] = temp_cnt;
	dwc->bh_completion_time[dwc->bh_dbg_index] = temp_time;
	dwc->bh_dbg_index = (dwc->bh_dbg_index + 1) % 10;

	return ret;
}

static irqreturn_t dwc3_process_event_buf(struct dwc3 *dwc, u32 buf)
{
	struct dwc3_event_buffer *evt;
	u32 count;

	evt = dwc->ev_buffs[buf];

	count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(buf));
	count &= DWC3_GEVNTCOUNT_MASK;
	if (!count)
		return IRQ_NONE;

	evt->count = count;
	evt->flags |= DWC3_EVENT_PENDING;

	return IRQ_WAKE_THREAD;
}

<<<<<<< HEAD
static void dwc3_interrupt_bh(unsigned long param)
{
	struct dwc3 *dwc = (struct dwc3 *) param;

	pm_runtime_get(dwc->dev);
	dwc3_thread_interrupt(dwc->irq, dwc);
	enable_irq(dwc->irq);
}

static irqreturn_t dwc3_interrupt(int irq, void *_dwc)
=======
irqreturn_t dwc3_interrupt(int irq, void *_dwc)
>>>>>>> 0e91d2a... Nougat
{
	struct dwc3			*dwc = _dwc;
	int				i;
	irqreturn_t			ret = IRQ_NONE;
	unsigned			temp_cnt = 0;
	ktime_t				start_time;

	start_time = ktime_get();
	spin_lock(&dwc->lock);

	dwc->irq_cnt++;

	if (dwc->err_evt_seen) {
		
		spin_unlock(&dwc->lock);
		return IRQ_HANDLED;
	}

	for (i = 0; i < dwc->num_event_buffers; i++) {
		irqreturn_t status;

		status = dwc3_process_event_buf(dwc, i);
		if (status == IRQ_WAKE_THREAD)
			ret = status;

		temp_cnt += dwc->ev_buffs[i]->count;
	}

	spin_unlock(&dwc->lock);

	dwc->irq_start_time[dwc->irq_dbg_index] = start_time;
	dwc->irq_completion_time[dwc->irq_dbg_index] =
		ktime_us_delta(ktime_get(), start_time);
	dwc->irq_event_count[dwc->irq_dbg_index] = temp_cnt / 4;
	dwc->irq_dbg_index = (dwc->irq_dbg_index + 1) % MAX_INTR_STATS;

	if (ret == IRQ_WAKE_THREAD) {
		disable_irq_nosync(irq);
		tasklet_schedule(&dwc->bh);
	}
	return IRQ_HANDLED;
}

int dwc3_gadget_init(struct dwc3 *dwc)
{
	u32					reg;
	int					ret;
	struct dwc3_usb_gadget *dwc3_gadget;

	dwc3_gadget = vzalloc(sizeof(*dwc3_gadget));
	if (!dwc3_gadget) {
		dev_err(dwc->dev, "failed to allocate dwc3_gadget\n");
		return -ENOMEM;
	}
	dwc3_gadget->dwc = dwc;
	dwc3_gadget->disable_during_lpm = false;
	INIT_WORK(&dwc3_gadget->wakeup_work, dwc3_gadget_wakeup_work);

	dwc->ctrl_req = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			&dwc->ctrl_req_addr, GFP_KERNEL);
	if (!dwc->ctrl_req) {
		dev_err(dwc->dev, "failed to allocate ctrl request\n");
		ret = -ENOMEM;
		goto err0;
	}

	dwc->ep0_trb = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			&dwc->ep0_trb_addr, GFP_KERNEL);
	if (!dwc->ep0_trb) {
		dev_err(dwc->dev, "failed to allocate ep0 trb\n");
		ret = -ENOMEM;
		goto err1;
	}

	dwc->setup_buf = kzalloc(DWC3_EP0_BOUNCE_SIZE, GFP_KERNEL);
	if (!dwc->setup_buf) {
		dev_err(dwc->dev, "failed to allocate setup buffer\n");
		ret = -ENOMEM;
		goto err2;
	}

	dwc->ep0_bounce = dma_alloc_coherent(dwc->dev,
			DWC3_EP0_BOUNCE_SIZE, &dwc->ep0_bounce_addr,
			GFP_KERNEL);
	if (!dwc->ep0_bounce) {
		dev_err(dwc->dev, "failed to allocate ep0 bounce buffer\n");
		ret = -ENOMEM;
		goto err3;
	}

	dwc->bh.func = dwc3_interrupt_bh;
	dwc->bh.data = (unsigned long)dwc;

	dwc->gadget.ops			= &dwc3_gadget_ops;
	if (dwc->maximum_speed == USB_SPEED_SUPER)
		dwc->gadget.max_speed		= USB_SPEED_SUPER;
	else
		dwc->gadget.max_speed		= USB_SPEED_HIGH;

	dwc->gadget.speed		= USB_SPEED_UNKNOWN;
	dwc->gadget.sg_supported	= true;
	dwc->gadget.name		= "dwc3-gadget";
	dwc->gadget.private		= dwc3_gadget;


	ret = dwc3_gadget_init_endpoints(dwc);
	if (ret)
		goto err4;

	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg |= DWC3_DCFG_LPM_CAP;
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);

	ret = usb_add_gadget_udc(dwc->dev, &dwc->gadget);
	if (ret) {
		dev_err(dwc->dev, "failed to register udc\n");
		goto err5;
	}

	if (dwc->dotg) {
		
		ret = otg_set_peripheral(&dwc->dotg->otg, &dwc->gadget);
		if (ret) {
			dev_err(dwc->dev, "failed to set peripheral to otg\n");
			goto err5;
		}
	} else {
		pm_runtime_no_callbacks(&dwc->gadget.dev);
		pm_runtime_set_active(&dwc->gadget.dev);
		pm_runtime_enable(&dwc->gadget.dev);
		pm_runtime_get(&dwc->gadget.dev);
	}

	return 0;

err5:
	dwc3_gadget_free_endpoints(dwc);

err4:
	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

err3:
	kfree(dwc->setup_buf);

err2:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

err1:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);

err0:
	vfree(dwc3_gadget);
	return ret;
}


void dwc3_gadget_exit(struct dwc3 *dwc)
{
	if (dwc->dotg) {
		pm_runtime_put(&dwc->gadget.dev);
		pm_runtime_disable(&dwc->gadget.dev);
	}

	usb_del_gadget_udc(&dwc->gadget);

	dwc3_gadget_free_endpoints(dwc);

	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

	kfree(dwc->setup_buf);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);
}

int dwc3_gadget_prepare(struct dwc3 *dwc)
{
	if (dwc->pullups_connected)
		dwc3_gadget_disable_irq(dwc);

	return 0;
}

void dwc3_gadget_complete(struct dwc3 *dwc)
{
	if (dwc->pullups_connected) {
		dwc3_gadget_enable_irq(dwc);
		dwc3_gadget_run_stop(dwc, true);
	}
}

int dwc3_gadget_suspend(struct dwc3 *dwc)
{
	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	dwc->dcfg = dwc3_readl(dwc->regs, DWC3_DCFG);

	return 0;
}

int dwc3_gadget_resume(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret;

	
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret)
		goto err0;

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret)
		goto err1;

	
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);

	dwc3_writel(dwc->regs, DWC3_DCFG, dwc->dcfg);

	return 0;

err1:
	__dwc3_gadget_ep_disable(dwc->eps[0]);

err0:
	return ret;
}
