/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/qpnp-misc.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/usb/msm_ext_chg.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/clk/msm-clk.h>
#include <linux/msm-bus.h>
#include <linux/irq.h>
#include <soc/qcom/scm.h>
#include <linux/usb/cable_detect.h>
#include <linux/usb/htc_info.h>
#include <linux/power/htc_charger.h>

#include "dwc3_otg.h"
#include "core.h"
#include "gadget.h"
#include "dbm.h"
#include "debug.h"
#include "xhci.h"

<<<<<<< HEAD
=======
#define DWC3_IDEV_CHG_MAX 1500
#define DWC3_HVDCP_CHG_MAX 1700

#define PERIPH_SS_AHB2PHY_TOP_CFG 0x10

#define ONE_READ_WRITE_WAIT 0x11

>>>>>>> 0e91d2a... Nougat
static int cpu_to_affin;
module_param(cpu_to_affin, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(cpu_to_affin, "affin usb irq to this cpu");

<<<<<<< HEAD
static int adc_low_threshold = 700;
module_param(adc_low_threshold, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(adc_low_threshold, "ADC ID Low voltage threshold");

static int adc_high_threshold = 950;
module_param(adc_high_threshold, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(adc_high_threshold, "ADC ID High voltage threshold");

static int adc_meas_interval = ADC_MEAS1_INTERVAL_1S;
module_param(adc_meas_interval, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(adc_meas_interval, "ADC ID polling period");
=======
static bool disable_host_mode;
module_param(disable_host_mode, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_host_mode, "To stop HOST mode detection");
>>>>>>> 0e91d2a... Nougat

static int override_phy_init;
module_param(override_phy_init, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(override_phy_init, "Override HSPHY Init Seq");

static bool prop_chg_detect;
module_param(prop_chg_detect, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(prop_chg_detect, "Enable Proprietary charger detection");

static bool usb_lpm_override;
module_param(usb_lpm_override, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(usb_lpm_override, "Override no_suspend_resume with USB");

static int hvdcp_max_current = DWC3_HVDCP_CHG_MAX;
module_param(hvdcp_max_current, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(hvdcp_max_current, "max current drawn for HVDCP charger");

int dcp_max_current = DWC3_IDEV_CHG_MAX;
module_param(dcp_max_current, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dcp_max_current, "max current drawn for DCP charger");

#define USB3_HCSPARAMS1		(0x4)
#define USB3_PORTSC		(0x420)

#define QSCRATCH_REG_OFFSET	(0x000F8800)
#define QSCRATCH_CTRL_REG      (QSCRATCH_REG_OFFSET + 0x04)
#define QSCRATCH_GENERAL_CFG	(QSCRATCH_REG_OFFSET + 0x08)
#define QSCRATCH_RAM1_REG	(QSCRATCH_REG_OFFSET + 0x0C)
#define HS_PHY_CTRL_REG		(QSCRATCH_REG_OFFSET + 0x10)
#define PARAMETER_OVERRIDE_X_REG (QSCRATCH_REG_OFFSET + 0x14)
#define CHARGING_DET_CTRL_REG	(QSCRATCH_REG_OFFSET + 0x18)
#define CHARGING_DET_OUTPUT_REG	(QSCRATCH_REG_OFFSET + 0x1C)
#define ALT_INTERRUPT_EN_REG	(QSCRATCH_REG_OFFSET + 0x20)
#define HS_PHY_IRQ_STAT_REG	(QSCRATCH_REG_OFFSET + 0x24)
#define CGCTL_REG		(QSCRATCH_REG_OFFSET + 0x28)
#define SS_PHY_CTRL_REG		(QSCRATCH_REG_OFFSET + 0x30)
#define SS_PHY_PARAM_CTRL_1	(QSCRATCH_REG_OFFSET + 0x34)
#define SS_PHY_PARAM_CTRL_2	(QSCRATCH_REG_OFFSET + 0x38)
#define SS_CR_PROTOCOL_DATA_IN_REG  (QSCRATCH_REG_OFFSET + 0x3C)
#define SS_CR_PROTOCOL_DATA_OUT_REG (QSCRATCH_REG_OFFSET + 0x40)
#define SS_CR_PROTOCOL_CAP_ADDR_REG (QSCRATCH_REG_OFFSET + 0x44)
#define SS_CR_PROTOCOL_CAP_DATA_REG (QSCRATCH_REG_OFFSET + 0x48)
#define SS_CR_PROTOCOL_READ_REG     (QSCRATCH_REG_OFFSET + 0x4C)
#define SS_CR_PROTOCOL_WRITE_REG    (QSCRATCH_REG_OFFSET + 0x50)
#define PWR_EVNT_IRQ_STAT_REG    (QSCRATCH_REG_OFFSET + 0x58)
#define PWR_EVNT_IRQ_MASK_REG    (QSCRATCH_REG_OFFSET + 0x5C)
<<<<<<< HEAD
#define HS_PHY_CTRL_COMMON_REG	(QSCRATCH_REG_OFFSET + 0xEC)
=======
#define QSCRATCH_USB30_STS_REG	(QSCRATCH_REG_OFFSET + 0xF8)

>>>>>>> 0e91d2a... Nougat

#define PWR_EVNT_POWERDOWN_IN_P3_MASK		BIT(2)
#define PWR_EVNT_POWERDOWN_OUT_P3_MASK		BIT(3)
#define PWR_EVNT_LPM_IN_L2_MASK			BIT(4)
#define PWR_EVNT_LPM_OUT_L2_MASK		BIT(5)
#define PWR_EVNT_LPM_OUT_L1_MASK		BIT(13)

#define DWC3_3P3_VOL_MIN		3075000 
#define DWC3_3P3_VOL_MAX		3200000 
#define DWC3_3P3_HPM_LOAD		30000	

#define DWC3_MSM_RESTORE_SCM_CFG_CMD 0x2
struct dwc3_msm_scm_cmd_buf {
	unsigned int device_id;
	unsigned int spare;
};

struct dwc3_msm_req_complete {
	struct list_head list_item;
	struct usb_request *req;
	void (*orig_complete)(struct usb_ep *ep,
			      struct usb_request *req);
};

struct dwc3_msm {
	struct device *dev;
	void __iomem *base;
	struct resource *io_res;
	struct platform_device	*dwc3;
	const struct usb_ep_ops *original_ep_ops[DWC3_ENDPOINTS_NUM];
	struct list_head req_complete_list;
	struct clk		*xo_clk;
	struct clk		*ref_clk;
	struct clk		*core_clk;
	struct clk		*iface_clk;
	struct clk		*sleep_clk;
	struct clk		*utmi_clk;
	struct clk		*phy_com_reset;
	unsigned int		utmi_clk_rate;
	struct clk		*utmi_clk_src;
	struct regulator	*dwc3_gdsc;
	struct wake_lock	cable_detect_wlock;

	struct usb_phy		*hs_phy, *ss_phy;

	struct dbm		*dbm;

	
	struct regulator	*vbus_otg;
	struct regulator	*vdda33;
	struct dwc3_ext_xceiv	ext_xceiv;
	bool			resume_pending;
	atomic_t                pm_suspended;
	int			hs_phy_irq;
	struct delayed_work	resume_work;
	struct work_struct	restart_usb_work;
	bool			in_restart;
<<<<<<< HEAD
	struct dwc3_charger	charger;
	struct usb_phy		*otg_xceiv;
	struct delayed_work	chg_work;
=======
	struct workqueue_struct *dwc3_wq;
	struct delayed_work	sm_work;
	unsigned long		inputs;
	enum dwc3_chg_type	chg_type;
	unsigned		max_power;
	bool			charging_disabled;
	bool			detect_dpdm_floating;
	enum usb_otg_state	otg_state;
>>>>>>> 0e91d2a... Nougat
	enum usb_chg_state	chg_state;
	int			pmic_id_irq;
	struct work_struct	id_work;
	struct qpnp_adc_tm_btm_param	adc_param;
	struct qpnp_adc_tm_chip *adc_tm_dev;
	struct delayed_work	init_adc_work;
	bool			id_adc_detect;
	struct qpnp_vadc_chip	*vadc_dev;
	u8			dcd_retries;
	u32			bus_perf_client;
	struct msm_bus_scale_pdata	*bus_scale_table;
	struct power_supply	usb_psy;
	struct power_supply	*ext_vbus_psy;
	unsigned int		online;
	unsigned int		scope;
	unsigned int		voltage_max;
	unsigned int		current_max;
	unsigned int		health_status;
	unsigned int		tx_fifo_size;
	unsigned int		qdss_tx_fifo_size;
	bool			vbus_active;
	bool			ext_inuse;
	bool			mhl_online;
	bool			rm_pulldown;
	enum dwc3_id_state	id_state;
	unsigned long		lpm_flags;
#define MDWC3_PHY_REF_CLK_OFF		BIT(0)
#define MDWC3_TCXO_SHUTDOWN		BIT(1)
#define MDWC3_ASYNC_IRQ_WAKE_CAPABILITY	BIT(2)
#define MDWC3_POWER_COLLAPSE		BIT(3)
#define MDWC3_CORECLK_OFF		BIT(4)
#define MDWC3_PHY_REF_AND_CORECLK_OFF	\
			(MDWC3_PHY_REF_CLK_OFF | MDWC3_CORECLK_OFF)

	u32 qscratch_ctl_val;
	dev_t ext_chg_dev;
	struct cdev ext_chg_cdev;
	struct class *ext_chg_class;
	struct device *ext_chg_device;
	bool ext_chg_opened;
	bool ext_chg_active;
	struct completion ext_chg_wait;
	unsigned int scm_dev_id;
	bool suspend_resume_no_support;

	bool power_collapse; 
	bool power_collapse_por; 

	unsigned int		irq_to_affin;
	struct notifier_block	dwc3_cpu_notifier;
	struct notifier_block	usbdev_nb;
	bool			hc_died;

	int  pwr_event_irq;
	atomic_t                in_p3;
	unsigned int		lpm_to_suspend_delay;
};

#define USB_HSPHY_3P3_VOL_MIN		3050000 
#define USB_HSPHY_3P3_VOL_MAX		3300000 
#define USB_HSPHY_3P3_HPM_LOAD		16000	

#define USB_HSPHY_1P8_VOL_MIN		1800000 
#define USB_HSPHY_1P8_VOL_MAX		1800000 
#define USB_HSPHY_1P8_HPM_LOAD		19000	

#define USB_SSPHY_1P8_VOL_MIN		1800000 
#define USB_SSPHY_1P8_VOL_MAX		1800000 
#define USB_SSPHY_1P8_HPM_LOAD		23000	

#define DSTS_CONNECTSPD_SS		0x4


static struct dwc3_msm *context = NULL;
static struct usb_ext_notification *usb_ext;

static int htc_id_backup;
static int htc_vbus_backup;

static void dwc3_pwr_event_handler(struct dwc3_msm *mdwc);

static inline u32 dwc3_msm_read_reg(void *base, u32 offset)
{
	u32 val = ioread32(base + offset);
	return val;
}

static inline u32 dwc3_msm_read_reg_field(void *base,
					  u32 offset,
					  const u32 mask)
{
	u32 shift = find_first_bit((void *)&mask, 32);
	u32 val = ioread32(base + offset);
	val &= mask;		
	val >>= shift;
	return val;
}

static inline void dwc3_msm_write_reg(void *base, u32 offset, u32 val)
{
	iowrite32(val, base + offset);
}

static inline void dwc3_msm_write_reg_field(void *base, u32 offset,
					    const u32 mask, u32 val)
{
	u32 shift = find_first_bit((void *)&mask, 32);
	u32 tmp = ioread32(base + offset);

	tmp &= ~mask;		/* clear written bits */
	val = tmp | (val << shift);
	iowrite32(val, base + offset);
}

/**
 * Write register and read back masked value to confirm it is written
 *
 * @base - DWC3 base virtual address.
 * @offset - register offset.
 * @mask - register bitmask specifying what should be updated
 * @val - value to write.
 *
 */
static inline void dwc3_msm_write_readback(void *base, u32 offset,
					    const u32 mask, u32 val)
{
	u32 write_val, tmp = ioread32(base + offset);

	tmp &= ~mask;		
	write_val = tmp | val;

	iowrite32(write_val, base + offset);

	/* Read back to see if val was written */
	tmp = ioread32(base + offset);
	tmp &= mask;		

	if (tmp != val)
		pr_err("%s: write: %x to QSCRATCH: %x FAILED\n",
			__func__, val, offset);
}

static bool dwc3_msm_is_host_superspeed(struct dwc3_msm *mdwc)
{
	int i, num_ports;
	u32 reg;

	reg = dwc3_msm_read_reg(mdwc->base, USB3_HCSPARAMS1);
	num_ports = HCS_MAX_PORTS(reg);

	for (i = 0; i < num_ports; i++) {
		reg = dwc3_msm_read_reg(mdwc->base, USB3_PORTSC + i*0x10);
		if ((reg & PORT_PE) && DEV_SUPERSPEED(reg))
			return true;
	}

	return false;
}

static inline bool dwc3_msm_is_dev_superspeed(struct dwc3_msm *mdwc)
{
	u8 speed;

	speed = dwc3_msm_read_reg(mdwc->base, DWC3_DSTS) & DWC3_DSTS_CONNECTSPD;
	return !!(speed & DSTS_CONNECTSPD_SS);
}

static inline bool dwc3_msm_is_superspeed(struct dwc3_msm *mdwc)
{
	if (mdwc->scope == POWER_SUPPLY_SCOPE_SYSTEM)
		return dwc3_msm_is_host_superspeed(mdwc);

	return dwc3_msm_is_dev_superspeed(mdwc);
}

<<<<<<< HEAD
static void dwc3_msm_dump_phy_info(struct dwc3_msm *mdwc)
{

	dbg_print_reg("SSPHY_CTRL_REG", dwc3_msm_read_reg(mdwc->base,
						SS_PHY_CTRL_REG));
	dbg_print_reg("HSPHY_CTRL_REG", dwc3_msm_read_reg(mdwc->base,
						HS_PHY_CTRL_REG));
	dbg_print_reg("QSCRATCH_CTRL_REG", dwc3_msm_read_reg(mdwc->base,
						QSCRATCH_CTRL_REG));
	dbg_print_reg("QSCRATCH_GENERAL_CFG", dwc3_msm_read_reg(mdwc->base,
						QSCRATCH_GENERAL_CFG));
	dbg_print_reg("PARAMETER_OVERRIDE_X_REG", dwc3_msm_read_reg(mdwc->base,
						PARAMETER_OVERRIDE_X_REG));
	dbg_print_reg("HS_PHY_IRQ_STAT_REG", dwc3_msm_read_reg(mdwc->base,
						HS_PHY_IRQ_STAT_REG));
	dbg_print_reg("SS_PHY_PARAM_CTRL_1", dwc3_msm_read_reg(mdwc->base,
						SS_PHY_PARAM_CTRL_1));
	dbg_print_reg("SS_PHY_PARAM_CTRL_2", dwc3_msm_read_reg(mdwc->base,
						SS_PHY_PARAM_CTRL_2));
	dbg_print_reg("QSCRATCH_RAM1_REG", dwc3_msm_read_reg(mdwc->base,
						QSCRATCH_RAM1_REG));
	dbg_print_reg("PWR_EVNT_IRQ_STAT_REG", dwc3_msm_read_reg(mdwc->base,
						PWR_EVNT_IRQ_STAT_REG));
	dbg_print_reg("PWR_EVNT_IRQ_MASK_REG", dwc3_msm_read_reg(mdwc->base,
						PWR_EVNT_IRQ_MASK_REG));
}

=======
int dwc3_msm_dbm_disable_updxfer(struct dwc3 *dwc, u8 usb_ep)
{
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	dev_dbg(mdwc->dev, "%s\n", __func__);
	dwc3_dbm_disable_update_xfer(mdwc->dbm, usb_ep);

	return 0;
}
>>>>>>> 0e91d2a... Nougat

int msm_data_fifo_config(struct usb_ep *ep, phys_addr_t addr,
			 u32 size, u8 dst_pipe_idx)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	dev_dbg(mdwc->dev, "%s\n", __func__);

	return	dbm_data_fifo_config(mdwc->dbm, dep->number, addr, size,
						dst_pipe_idx);
}


static void dwc3_msm_req_complete_func(struct usb_ep *ep,
				       struct usb_request *request)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	struct dwc3_msm_req_complete *req_complete = NULL;

	
	list_for_each_entry(req_complete, &mdwc->req_complete_list, list_item) {
		if (req_complete->req == request)
			break;
	}
	if (!req_complete || req_complete->req != request) {
		dev_err(dep->dwc->dev, "%s: could not find the request\n",
					__func__);
		return;
	}
	list_del(&req_complete->list_item);

	dep->busy_slot++;

	
	dbm_ep_unconfig(mdwc->dbm, dep->number);

	if (0 == dbm_get_num_of_eps_configured(mdwc->dbm) &&
		!dbm_reset_ep_after_lpm(mdwc->dbm))
			dbm_event_buffer_config(mdwc->dbm, 0, 0, 0);

	request->complete = req_complete->orig_complete;
	if (request->complete)
		request->complete(ep, request);

	kfree(req_complete);
}


static int __dwc3_msm_dbm_ep_reset(struct dwc3_msm *mdwc, struct dwc3_ep *dep)
{
	int ret;

	dev_dbg(mdwc->dev, "Resetting dbm endpoint %d\n", dep->number);

	
	ret = dbm_ep_soft_reset(mdwc->dbm, dep->number, true);
	if (ret) {
		dev_err(mdwc->dev, "%s: failed to assert dbm ep reset\n",
				__func__);
		return ret;
	}

	if (dbm_get_num_of_eps_configured(mdwc->dbm) > 1)
		usleep_range(1000, 1200);
	else
		udelay(10);
	ret = dbm_ep_soft_reset(mdwc->dbm, dep->number, false);
	if (ret) {
		dev_err(mdwc->dev, "%s: failed to deassert dbm ep reset\n",
				__func__);
		return ret;
	}

	return 0;
}


int msm_dwc3_reset_dbm_ep(struct usb_ep *ep)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	return __dwc3_msm_dbm_ep_reset(mdwc, dep);
}
EXPORT_SYMBOL(msm_dwc3_reset_dbm_ep);


static int __dwc3_msm_ep_queue(struct dwc3_ep *dep, struct dwc3_request *req)
{
	struct dwc3_trb *trb;
	struct dwc3_trb *trb_link;
	struct dwc3_gadget_ep_cmd_params params;
	u32 cmd;
	int ret = 0;

	req->queued = true;
	list_add_tail(&req->list, &dep->req_queued);

	
	trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];
	dep->free_slot++;
	memset(trb, 0, sizeof(*trb));

	req->trb = trb;
	trb->bph = DBM_TRB_BIT | DBM_TRB_DMA | DBM_TRB_EP_NUM(dep->number);
	trb->size = DWC3_TRB_SIZE_LENGTH(req->request.length);
	trb->ctrl = DWC3_TRBCTL_NORMAL | DWC3_TRB_CTRL_HWO |
		DWC3_TRB_CTRL_CHN | (req->direction ? 0 : DWC3_TRB_CTRL_CSP);
	req->trb_dma = dwc3_trb_dma_offset(dep, trb);

	
	trb_link = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];
	dep->free_slot++;
	memset(trb_link, 0, sizeof *trb_link);

	trb_link->bpl = lower_32_bits(req->trb_dma);
	trb_link->bph = DBM_TRB_BIT |
			DBM_TRB_DMA | DBM_TRB_EP_NUM(dep->number);
	trb_link->size = 0;
	trb_link->ctrl = DWC3_TRBCTL_LINK_TRB | DWC3_TRB_CTRL_HWO;

	memset(&params, 0, sizeof(params));
	params.param0 = 0; 
	params.param1 = lower_32_bits(req->trb_dma); 

	
	cmd = DWC3_DEPCMD_STARTTRANSFER | DWC3_DEPCMD_CMDIOC;
	ret = dwc3_send_gadget_ep_cmd(dep->dwc, dep->number, cmd, &params);
	if (ret < 0) {
		dev_dbg(dep->dwc->dev,
			"%s: failed to send STARTTRANSFER command\n",
			__func__);

		list_del(&req->list);
		return ret;
	}
	dep->flags |= DWC3_EP_BUSY;
	dep->resource_index = dwc3_gadget_ep_get_transfer_index(dep->dwc,
		dep->number);

	return ret;
}

static int dwc3_msm_ep_queue(struct usb_ep *ep,
			     struct usb_request *request, gfp_t gfp_flags)
{
	struct dwc3_request *req = to_dwc3_request(request);
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	struct dwc3_msm_req_complete *req_complete;
	unsigned long flags;
<<<<<<< HEAD
	int ret = 0;
	u8 bam_pipe;
	bool producer;
	bool disable_wb;
	bool internal_mem;
	bool ioc;
=======
	int ret = 0, size;
>>>>>>> 0e91d2a... Nougat
	bool superspeed;

	spin_lock_irqsave(&dwc->lock, flags);
	if (!dep->endpoint.desc) {
		dev_err(mdwc->dev,
			"%s: trying to queue request %p to disabled ep %s\n",
			__func__, request, ep->name);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EPERM;
	}

	if (!mdwc->original_ep_ops[dep->number]) {
		dev_err(mdwc->dev,
			"ep [%s,%d] was unconfigured as msm endpoint\n",
			ep->name, dep->number);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EINVAL;
	}

	if (!request) {
		dev_err(mdwc->dev, "%s: request is NULL\n", __func__);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EINVAL;
	}

	if (!(request->udc_priv & MSM_SPS_MODE)) {
		dev_err(mdwc->dev, "%s: sps mode is not set\n",
					__func__);

		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EINVAL;
	}

	
	if (req->request.length < 0x2000) {
		dev_err(mdwc->dev, "%s: Min TRB size is 8KB\n", __func__);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EINVAL;
	}

	if (dep->number == 0 || dep->number == 1) {
		dev_err(mdwc->dev,
			"%s: trying to queue dbm request %p to control ep %s\n",
			__func__, request, ep->name);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EPERM;
	}

	if (dep->busy_slot != dep->free_slot || !list_empty(&dep->request_list)
					 || !list_empty(&dep->req_queued)) {
		dev_err(mdwc->dev,
			"%s: trying to queue dbm request %p tp ep %s\n",
			__func__, request, ep->name);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EPERM;
	}
	dep->busy_slot = 0;
	dep->free_slot = 0;

	req_complete = kzalloc(sizeof(*req_complete), gfp_flags);
	if (!req_complete) {
		dev_err(mdwc->dev, "%s: not enough memory\n", __func__);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -ENOMEM;
	}
	req_complete->req = request;
	req_complete->orig_complete = request->complete;
	list_add_tail(&req_complete->list_item, &mdwc->req_complete_list);
	request->complete = dwc3_msm_req_complete_func;

	dev_vdbg(dwc->dev, "%s: queing request %p to ep %s length %d\n",
			__func__, request, ep->name, request->length);

	dbm_event_buffer_config(mdwc->dbm,
		dwc3_msm_read_reg(mdwc->base, DWC3_GEVNTADRLO(0)),
		dwc3_msm_read_reg(mdwc->base, DWC3_GEVNTADRHI(0)),
		dwc3_msm_read_reg(mdwc->base, DWC3_GEVNTSIZ(0)));

	ret = __dwc3_msm_ep_queue(dep, req);
	if (ret < 0) {
		dev_err(mdwc->dev,
			"error %d after calling __dwc3_msm_ep_queue\n", ret);
		goto err;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);
	superspeed = dwc3_msm_is_dev_superspeed(mdwc);
	dbm_set_speed(mdwc->dbm, (u8)superspeed);

	return 0;

err:
	list_del(&req_complete->list_item);
	spin_unlock_irqrestore(&dwc->lock, flags);
	kfree(req_complete);
	return ret;
}

<<<<<<< HEAD
=======
static inline int gsi_get_xfer_index(struct usb_ep *ep)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);

	return dep->resource_index;
}

static void gsi_get_channel_info(struct usb_ep *ep,
			struct gsi_channel_info *ch_info)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	int last_trb_index = 0;
	struct dwc3	*dwc = dep->dwc;
	struct usb_gsi_request *request = ch_info->ch_req;

	
	ch_info->depcmd_low_addr = (u32)(dwc->reg_phys +
						DWC3_DEPCMD(dep->number));
	ch_info->depcmd_hi_addr = 0;

	ch_info->xfer_ring_base_addr = dwc3_trb_dma_offset(dep,
							&dep->trb_pool[0]);
	
	ch_info->const_buffer_size = request->buf_len/1024;

	
	if (dep->direction) {
		ch_info->xfer_ring_len = (2 * request->num_bufs + 2) * 0x10;
		last_trb_index = 2 * request->num_bufs + 2;
	} else { 
		ch_info->xfer_ring_len = (request->num_bufs + 1) * 0x10;
		last_trb_index = request->num_bufs + 1;
	}

	
	ch_info->last_trb_addr = (dwc3_trb_dma_offset(dep,
			&dep->trb_pool[last_trb_index - 1]) & 0x0000FFFF);
	ch_info->gevntcount_low_addr = (u32)(dwc->reg_phys +
			DWC3_GEVNTCOUNT(ep->ep_intr_num));
	ch_info->gevntcount_hi_addr = 0;

	dev_dbg(dwc->dev,
	"depcmd_laddr=%x last_trb_addr=%x gevtcnt_laddr=%x gevtcnt_haddr=%x",
		ch_info->depcmd_low_addr, ch_info->last_trb_addr,
		ch_info->gevntcount_low_addr, ch_info->gevntcount_hi_addr);
}

static int gsi_startxfer_for_ep(struct usb_ep *ep)
{
	int ret;
	struct dwc3_gadget_ep_cmd_params params;
	u32				cmd;
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3	*dwc = dep->dwc;

	memset(&params, 0, sizeof(params));
	params.param0 = GSI_TRB_ADDR_BIT_53_MASK | GSI_TRB_ADDR_BIT_55_MASK;
	params.param0 |= (ep->ep_intr_num << 16);
	params.param1 = lower_32_bits(dwc3_trb_dma_offset(dep,
						&dep->trb_pool[0]));
	cmd = DWC3_DEPCMD_STARTTRANSFER;
	cmd |= DWC3_DEPCMD_PARAM(0);
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);

	if (ret < 0)
		dev_dbg(dwc->dev, "Fail StrtXfr on GSI EP#%d\n", dep->number);
	dep->resource_index = dwc3_gadget_ep_get_transfer_index(dwc,
								dep->number);
	dev_dbg(dwc->dev, "XferRsc = %x", dep->resource_index);
	return ret;
}

static void gsi_store_ringbase_dbl_info(struct usb_ep *ep, u32 dbl_addr)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3	*dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	int n = ep->ep_intr_num - 1;

	dwc3_msm_write_reg(mdwc->base, GSI_RING_BASE_ADDR_L(n),
			dwc3_trb_dma_offset(dep, &dep->trb_pool[0]));
	dwc3_msm_write_reg(mdwc->base, GSI_DBL_ADDR_L(n), dbl_addr);

	dev_dbg(mdwc->dev, "Ring Base Addr %d = %x", n,
			dwc3_msm_read_reg(mdwc->base, GSI_RING_BASE_ADDR_L(n)));
	dev_dbg(mdwc->dev, "GSI DB Addr %d = %x", n,
			dwc3_msm_read_reg(mdwc->base, GSI_DBL_ADDR_L(n)));
}

static void gsi_ring_in_db(struct usb_ep *ep, struct usb_gsi_request *request)
{
	void __iomem *gsi_dbl_address_lsb;
	void __iomem *gsi_dbl_address_msb;
	dma_addr_t offset;
	u64 dbl_addr = *((u64 *)request->buf_base_addr);
	u32 dbl_lo_addr = (dbl_addr & 0xFFFFFFFF);
	u32 dbl_hi_addr = (dbl_addr >> 32);
	u32 num_trbs = (request->num_bufs * 2 + 2);
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3	*dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	gsi_dbl_address_lsb = devm_ioremap_nocache(mdwc->dev,
					dbl_lo_addr, sizeof(u32));
	if (!gsi_dbl_address_lsb)
		dev_dbg(mdwc->dev, "Failed to get GSI DBL address LSB\n");

	gsi_dbl_address_msb = devm_ioremap_nocache(mdwc->dev,
					dbl_hi_addr, sizeof(u32));
	if (!gsi_dbl_address_msb)
		dev_dbg(mdwc->dev, "Failed to get GSI DBL address MSB\n");

	offset = dwc3_trb_dma_offset(dep, &dep->trb_pool[num_trbs-1]);
	dev_dbg(mdwc->dev, "Writing link TRB addr: %pa to %p (%x)\n",
	&offset, gsi_dbl_address_lsb, dbl_lo_addr);

	writel_relaxed(offset, gsi_dbl_address_lsb);
	writel_relaxed(0, gsi_dbl_address_msb);
}

static int gsi_updatexfer_for_ep(struct usb_ep *ep,
					struct usb_gsi_request *request)
{
	int i;
	int ret;
	u32				cmd;
	int num_trbs = request->num_bufs + 1;
	struct dwc3_trb *trb;
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;

	for (i = 0; i < num_trbs - 1; i++) {
		trb = &dep->trb_pool[i];
		trb->ctrl |= DWC3_TRB_CTRL_HWO;
	}

	memset(&params, 0, sizeof(params));
	cmd = DWC3_DEPCMD_UPDATETRANSFER;
	cmd |= DWC3_DEPCMD_PARAM(dep->resource_index);
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	dep->flags |= DWC3_EP_BUSY;
	if (ret < 0)
		dev_dbg(dwc->dev, "UpdateXfr fail on GSI EP#%d\n", dep->number);
	return ret;
}

static void gsi_endxfer_for_ep(struct usb_ep *ep)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3	*dwc = dep->dwc;

	dwc3_stop_active_transfer(dwc, dep->number, true);
}

static int gsi_prepare_trbs(struct usb_ep *ep, struct usb_gsi_request *req)
{
	int i = 0;
	dma_addr_t buffer_addr = req->dma;
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_trb *trb;
	int num_trbs = (dep->direction) ? (2 * (req->num_bufs) + 2)
					: (req->num_bufs + 1);

	dep->trb_dma_pool = dma_pool_create(ep->name, dwc->dev,
					num_trbs * sizeof(struct dwc3_trb),
					num_trbs * sizeof(struct dwc3_trb), 0);
	if (!dep->trb_dma_pool) {
		dev_err(dep->dwc->dev, "failed to alloc trb dma pool for %s\n",
				dep->name);
		return -ENOMEM;
	}

	dep->num_trbs = num_trbs;

	dep->trb_pool = dma_pool_alloc(dep->trb_dma_pool,
					   GFP_KERNEL, &dep->trb_pool_dma);
	if (!dep->trb_pool) {
		dev_err(dep->dwc->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}

	
	if (dep->direction) {
		for (i = 0; i < num_trbs ; i++) {
			trb = &dep->trb_pool[i];
			memset(trb, 0, sizeof(*trb));
			
			if (i < (req->num_bufs + 1)) {
				trb->bpl = 0;
				trb->bph = 0;
				trb->size = 0;
				trb->ctrl = DWC3_TRBCTL_NORMAL
						| DWC3_TRB_CTRL_IOC;
				continue;
			}

			
			trb->bpl = lower_32_bits(buffer_addr);
			trb->bph = 0;
			trb->size = 0;
			trb->ctrl = DWC3_TRBCTL_NORMAL
					| DWC3_TRB_CTRL_IOC;
			buffer_addr += req->buf_len;

			
			if (i == (num_trbs - 1)) {
				trb->bpl = dwc3_trb_dma_offset(dep,
							&dep->trb_pool[0]);
				trb->bph = (1 << 23) | (1 << 21)
						| (ep->ep_intr_num << 16);
				trb->size = 0;
				trb->ctrl = DWC3_TRBCTL_LINK_TRB
						| DWC3_TRB_CTRL_HWO;
			}
		}
	} else { 

		for (i = 0; i < num_trbs ; i++) {

			trb = &dep->trb_pool[i];
			memset(trb, 0, sizeof(*trb));
			trb->bpl = lower_32_bits(buffer_addr);
			trb->bph = 0;
			trb->size = req->buf_len;
			trb->ctrl = DWC3_TRBCTL_NORMAL | DWC3_TRB_CTRL_IOC
					| DWC3_TRB_CTRL_CSP
					| DWC3_TRB_CTRL_ISP_IMI;
			buffer_addr += req->buf_len;

			
			if (i == (num_trbs - 1)) {
				trb->bpl = dwc3_trb_dma_offset(dep,
							&dep->trb_pool[0]);
				trb->bph = (1 << 23) | (1 << 21)
						| (ep->ep_intr_num << 16);
				trb->size = 0;
				trb->ctrl = DWC3_TRBCTL_LINK_TRB
						| DWC3_TRB_CTRL_HWO;
			}
		 }
	}
	return 0;
}

static void gsi_free_trbs(struct usb_ep *ep)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);

	if (dep->endpoint.ep_type == EP_TYPE_NORMAL)
		return;

	
	if (dep->trb_dma_pool) {
		dma_pool_free(dep->trb_dma_pool, dep->trb_pool,
						dep->trb_pool_dma);
		dma_pool_destroy(dep->trb_dma_pool);
		dep->trb_pool = NULL;
		dep->trb_pool_dma = 0;
		dep->trb_dma_pool = NULL;
	}
}
static void gsi_configure_ep(struct usb_ep *ep, struct usb_gsi_request *request)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	struct dwc3_gadget_ep_cmd_params params;
	const struct usb_endpoint_descriptor *desc = ep->desc;
	const struct usb_ss_ep_comp_descriptor *comp_desc = ep->comp_desc;
	u32			reg;

	memset(&params, 0x00, sizeof(params));

	
	params.param0 = DWC3_DEPCFG_EP_TYPE(usb_endpoint_type(desc))
		| DWC3_DEPCFG_MAX_PACKET_SIZE(usb_endpoint_maxp(desc));

	
	if (dwc->gadget.speed == USB_SPEED_SUPER) {
		u32 burst = dep->endpoint.maxburst - 1;

		params.param0 |= DWC3_DEPCFG_BURST_SIZE(burst);
	}

	if (usb_ss_max_streams(comp_desc) && usb_endpoint_xfer_bulk(desc)) {
		params.param1 |= DWC3_DEPCFG_STREAM_CAPABLE
					| DWC3_DEPCFG_STREAM_EVENT_EN;
		dep->stream_capable = true;
	}

	
	params.param1 |= DWC3_DEPCFG_EP_NUMBER(dep->number);

	
	params.param1 |= DWC3_DEPCFG_INT_NUM(ep->ep_intr_num);

	
	params.param1 |= DWC3_DEPCFG_XFER_COMPLETE_EN;
	params.param1 |= DWC3_DEPCFG_XFER_IN_PROGRESS_EN;
	params.param1 |= DWC3_DEPCFG_FIFO_ERROR_EN;
	
	if (dep->direction)
		params.param0 |= DWC3_DEPCFG_FIFO_NUMBER(dep->number >> 1);

	params.param0 |= DWC3_DEPCFG_ACTION_INIT;

	dev_dbg(mdwc->dev, "Set EP config to params = %x %x %x, for %s\n",
	params.param0, params.param1, params.param2, dep->name);

	dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_SETEPCONFIG, &params);

	
	if (!(dep->flags & DWC3_EP_ENABLED)) {
		memset(&params, 0x00, sizeof(params));
		params.param0 = DWC3_DEPXFERCFG_NUM_XFER_RES(1);
		dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_SETTRANSFRESOURCE, &params);

		dep->endpoint.desc = desc;
		dep->comp_desc = comp_desc;
		dep->type = usb_endpoint_type(desc);
		dep->flags |= DWC3_EP_ENABLED;
		reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
		reg |= DWC3_DALEPENA_EP(dep->number);
		dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);
	}

}

static void gsi_enable(struct usb_ep *ep)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	dwc3_msm_write_reg_field(mdwc->base,
			GSI_GENERAL_CFG_REG, GSI_CLK_EN_MASK, 1);
	dwc3_msm_write_reg_field(mdwc->base,
			GSI_GENERAL_CFG_REG, GSI_RESTART_DBL_PNTR_MASK, 1);
	dwc3_msm_write_reg_field(mdwc->base,
			GSI_GENERAL_CFG_REG, GSI_RESTART_DBL_PNTR_MASK, 0);
	dev_dbg(mdwc->dev, "%s: Enable GSI\n", __func__);
	dwc3_msm_write_reg_field(mdwc->base,
			GSI_GENERAL_CFG_REG, GSI_EN_MASK, 1);
}

static void gsi_set_clear_dbell(struct usb_ep *ep,
					bool block_db)
{

	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	dwc3_msm_write_reg_field(mdwc->base,
		GSI_GENERAL_CFG_REG, BLOCK_GSI_WR_GO_MASK, block_db);
}

static bool gsi_check_ready_to_suspend(struct usb_ep *ep, bool f_suspend)
{
	u32	timeout = 1500;
	u32	reg = 0;
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	while (dwc3_msm_read_reg_field(mdwc->base,
		GSI_IF_STS, GSI_WR_CTRL_STATE_MASK)) {
		if (!timeout--) {
			dev_err(mdwc->dev,
			"Unable to suspend GSI ch. WR_CTRL_STATE != 0\n");
			return false;
		}
	}
	
	if (!f_suspend) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);
		if (DWC3_DSTS_USBLNKST(reg) != DWC3_LINK_STATE_U3) {
			dev_err(mdwc->dev, "Unable to suspend GSI ch\n");
			return false;
		}
	}

	return true;
}


static int dwc3_msm_gsi_ep_op(struct usb_ep *ep,
		void *op_data, enum gsi_ep_op op)
{
	u32 ret = 0;
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	struct usb_gsi_request *request;
	struct gsi_channel_info *ch_info;
	bool block_db, f_suspend;

	switch (op) {
	case GSI_EP_OP_PREPARE_TRBS:
		request = (struct usb_gsi_request *)op_data;
		dev_dbg(mdwc->dev, "EP_OP_PREPARE_TRBS for %s\n", ep->name);
		ret = gsi_prepare_trbs(ep, request);
		break;
	case GSI_EP_OP_FREE_TRBS:
		dev_dbg(mdwc->dev, "EP_OP_FREE_TRBS for %s\n", ep->name);
		gsi_free_trbs(ep);
		break;
	case GSI_EP_OP_CONFIG:
		request = (struct usb_gsi_request *)op_data;
		dev_dbg(mdwc->dev, "EP_OP_CONFIG for %s\n", ep->name);
		gsi_configure_ep(ep, request);
		break;
	case GSI_EP_OP_STARTXFER:
		dev_dbg(mdwc->dev, "EP_OP_STARTXFER for %s\n", ep->name);
		ret = gsi_startxfer_for_ep(ep);
		break;
	case GSI_EP_OP_GET_XFER_IDX:
		dev_dbg(mdwc->dev, "EP_OP_GET_XFER_IDX for %s\n", ep->name);
		ret = gsi_get_xfer_index(ep);
		break;
	case GSI_EP_OP_STORE_DBL_INFO:
		dev_dbg(mdwc->dev, "EP_OP_STORE_DBL_INFO\n");
		gsi_store_ringbase_dbl_info(ep, *((u32 *)op_data));
		break;
	case GSI_EP_OP_ENABLE_GSI:
		dev_dbg(mdwc->dev, "EP_OP_ENABLE_GSI\n");
		gsi_enable(ep);
		break;
	case GSI_EP_OP_GET_CH_INFO:
		ch_info = (struct gsi_channel_info *)op_data;
		gsi_get_channel_info(ep, ch_info);
		break;
	case GSI_EP_OP_RING_IN_DB:
		request = (struct usb_gsi_request *)op_data;
		dev_dbg(mdwc->dev, "RING IN EP DB\n");
		gsi_ring_in_db(ep, request);
		break;
	case GSI_EP_OP_UPDATEXFER:
		request = (struct usb_gsi_request *)op_data;
		dev_dbg(mdwc->dev, "EP_OP_UPDATEXFER\n");
		ret = gsi_updatexfer_for_ep(ep, request);
		break;
	case GSI_EP_OP_ENDXFER:
		request = (struct usb_gsi_request *)op_data;
		dev_dbg(mdwc->dev, "EP_OP_ENDXFER for %s\n", ep->name);
		gsi_endxfer_for_ep(ep);
		break;
	case GSI_EP_OP_SET_CLR_BLOCK_DBL:
		block_db = *((bool *)op_data);
		dev_dbg(mdwc->dev, "EP_OP_SET_CLR_BLOCK_DBL %d\n",
						block_db);
		gsi_set_clear_dbell(ep, block_db);
		break;
	case GSI_EP_OP_CHECK_FOR_SUSPEND:
		dev_dbg(mdwc->dev, "EP_OP_CHECK_FOR_SUSPEND\n");
		f_suspend = *((bool *)op_data);
		ret = gsi_check_ready_to_suspend(ep, f_suspend);
		break;
	default:
		dev_err(mdwc->dev, "%s: Invalid opcode GSI EP\n", __func__);
	}

	return ret;
}
>>>>>>> 0e91d2a... Nougat

int msm_ep_config(struct usb_ep *ep, struct usb_request *request)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	struct usb_ep_ops *new_ep_ops;
	int ret = 0;
	u8 bam_pipe;
	bool producer;
	bool disable_wb;
	bool internal_mem;
	bool ioc;
	unsigned long flags;


	spin_lock_irqsave(&dwc->lock, flags);
	
	if (mdwc->original_ep_ops[dep->number]) {
		dev_err(mdwc->dev,
			"ep [%s,%d] already configured as msm endpoint\n",
			ep->name, dep->number);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EPERM;
	}
	mdwc->original_ep_ops[dep->number] = ep->ops;

	
	new_ep_ops = kzalloc(sizeof(struct usb_ep_ops), GFP_ATOMIC);
	if (!new_ep_ops) {
		dev_err(mdwc->dev,
			"%s: unable to allocate mem for new usb ep ops\n",
			__func__);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -ENOMEM;
	}
	(*new_ep_ops) = (*ep->ops);
	new_ep_ops->queue = dwc3_msm_ep_queue;
	ep->ops = new_ep_ops;

	if (!mdwc->dbm || !request || (dep->endpoint.ep_type == EP_TYPE_GSI)) {
		spin_unlock_irqrestore(&dwc->lock, flags);
		return 0;
	}

	bam_pipe = request->udc_priv & MSM_PIPE_ID_MASK;
	producer = ((request->udc_priv & MSM_PRODUCER) ? true : false);
	disable_wb = ((request->udc_priv & MSM_DISABLE_WB) ? true : false);
	internal_mem = ((request->udc_priv & MSM_INTERNAL_MEM) ? true : false);
	ioc = ((request->udc_priv & MSM_ETD_IOC) ? true : false);

	ret = dbm_ep_config(mdwc->dbm, dep->number, bam_pipe, producer,
					disable_wb, internal_mem, ioc);
	if (ret < 0) {
		dev_err(mdwc->dev,
			"error %d after calling dbm_ep_config\n", ret);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return ret;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}
EXPORT_SYMBOL(msm_ep_config);

int msm_ep_unconfig(struct usb_ep *ep)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	struct usb_ep_ops *old_ep_ops;
	unsigned long flags;

	spin_lock_irqsave(&dwc->lock, flags);
	
	if (!mdwc->original_ep_ops[dep->number]) {
		dev_dbg(mdwc->dev,
			"ep [%s,%d] was not configured as msm endpoint\n",
			ep->name, dep->number);
		spin_unlock_irqrestore(&dwc->lock, flags);
		return -EINVAL;
	}
	old_ep_ops = (struct usb_ep_ops	*)ep->ops;
	ep->ops = mdwc->original_ep_ops[dep->number];
	mdwc->original_ep_ops[dep->number] = NULL;
	kfree(old_ep_ops);

	if (!mdwc->dbm || (dep->endpoint.ep_type == EP_TYPE_GSI)) {
		spin_unlock_irqrestore(&dwc->lock, flags);
		return 0;
	}

	if (dep->busy_slot == dep->free_slot && list_empty(&dep->request_list)
					 && list_empty(&dep->req_queued)) {
		dev_dbg(mdwc->dev,
			"%s: request is not queued, disable DBM ep for ep %s\n",
			__func__, ep->name);
		
		dbm_ep_unconfig(mdwc->dbm, dep->number);

		if (0 == dbm_get_num_of_eps_configured(mdwc->dbm) &&
				!dbm_reset_ep_after_lpm(mdwc->dbm))
			dbm_event_buffer_config(mdwc->dbm, 0, 0, 0);
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}
EXPORT_SYMBOL(msm_ep_unconfig);

void dwc3_tx_fifo_resize_request(struct usb_ep *ep, bool qdss_enabled)
{
	struct dwc3_ep *dep = to_dwc3_ep(ep);
	struct dwc3 *dwc = dep->dwc;
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	if (qdss_enabled) {
		dwc->tx_fifo_reduced = true;
		dwc->tx_fifo_size = mdwc->qdss_tx_fifo_size;
	} else {
		dwc->tx_fifo_reduced = false;
		dwc->tx_fifo_size = mdwc->tx_fifo_size;
	}
}
EXPORT_SYMBOL(dwc3_tx_fifo_resize_request);

static void dwc3_resume_work(struct work_struct *w);

static void dwc3_restart_usb_work(struct work_struct *w)
{
	struct dwc3_msm *mdwc = container_of(w, struct dwc3_msm,
						restart_usb_work);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	enum dwc3_chg_type chg_type;
	unsigned timeout = 50;

	dev_dbg(mdwc->dev, "%s\n", __func__);

	if (atomic_read(&dwc->in_lpm) || !mdwc->otg_xceiv) {
		dev_err(mdwc->dev, "%s failed!!!\n", __func__);
		return;
	}

	
	mdwc->in_restart = true;

	if (!mdwc->ext_xceiv.bsv) {
		dev_dbg(mdwc->dev, "%s bailing out in disconnect\n", __func__);
		dwc->err_evt_seen = false;
		mdwc->in_restart = false;
		return;
	}

	dbg_event(0xFF, "RestartUSB", 0);
	chg_type = mdwc->charger.chg_type;

	
	mdwc->ext_xceiv.bsv = false;
	dwc3_resume_work(&mdwc->resume_work.work);

	
	while (--timeout && !pm_runtime_suspended(mdwc->dev))
		msleep(20);

	if (!timeout) {
		dev_warn(mdwc->dev, "Not in LPM after disconnect, forcing suspend...\n");
		pm_runtime_suspend(mdwc->dev);
	}

	
	if (mdwc->vbus_active) {
		mdwc->ext_xceiv.bsv = true;
		mdwc->charger.chg_type = chg_type;
		dwc3_resume_work(&mdwc->resume_work.work);
	}

	dwc->err_evt_seen = false;
	mdwc->in_restart = false;
}

static int msm_dwc3_usbdev_notify(struct notifier_block *self,
			unsigned long action, void *priv)
{
	struct dwc3_msm *mdwc = container_of(self, struct dwc3_msm, usbdev_nb);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	struct usb_bus *bus = priv;

	
	if (action != USB_BUS_DIED)
		return 0;

<<<<<<< HEAD
	dev_dbg(mdwc->dev, "%s\n", __func__);
	queue_work(system_nrt_wq, &mdwc->restart_usb_work);
=======
	dev_dbg(mdwc->dev, "%s initiate recovery from hc_died\n", __func__);
	
	if (mdwc->hc_died)
		return 0;

	if (bus->controller != &dwc->xhci->dev) {
		dev_dbg(mdwc->dev, "%s event for diff HCD\n", __func__);
		return 0;
	}

	mdwc->hc_died = true;
	schedule_delayed_work(&mdwc->sm_work, 0);
	return 0;
>>>>>>> 0e91d2a... Nougat
}


int msm_register_usb_ext_notification(struct usb_ext_notification *info)
{
	pr_debug("%s usb_ext: %p\n", __func__, info);

	if (info) {
		if (usb_ext) {
			pr_err("%s: already registered\n", __func__);
			return -EEXIST;
		}

		if (!info->notify) {
			pr_err("%s: notify is NULL\n", __func__);
			return -EINVAL;
		}
	}

	usb_ext = info;
	return 0;
}
EXPORT_SYMBOL(msm_register_usb_ext_notification);

bool msm_dwc3_reset_ep_after_lpm(struct usb_gadget *gadget)
{
	struct dwc3 *dwc = container_of(gadget, struct dwc3, gadget);
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);

	return dbm_reset_ep_after_lpm(mdwc->dbm);
}
EXPORT_SYMBOL(msm_dwc3_reset_ep_after_lpm);

static int dwc3_msm_config_gdsc(struct dwc3_msm *mdwc, int on)
{
	int ret = 0;

	if (IS_ERR(mdwc->dwc3_gdsc))
		return 0;

	if (!mdwc->dwc3_gdsc) {
		mdwc->dwc3_gdsc = devm_regulator_get(mdwc->dev,
			"USB3_GDSC");
		if (IS_ERR(mdwc->dwc3_gdsc))
			return 0;
	}

	if (on) {
		ret = regulator_enable(mdwc->dwc3_gdsc);
		if (ret) {
			dev_err(mdwc->dev, "unable to enable usb3 gdsc\n");
			return ret;
		}
	} else {
		regulator_disable(mdwc->dwc3_gdsc);
	}

	return 0;
}

static int dwc3_msm_restore_sec_config(unsigned int device_id)
{
	struct dwc3_msm_scm_cmd_buf cbuf;
	int ret, scm_ret = 0;

	if (!device_id)
		return 0;

	cbuf.device_id = device_id;

	ret = scm_call(SCM_SVC_MP, DWC3_MSM_RESTORE_SCM_CFG_CMD, &cbuf,
			sizeof(cbuf), &scm_ret, sizeof(scm_ret));
	if (ret || scm_ret) {
		pr_err("%s: failed(%d) to restore sec config, scm_ret=%d\n",
			__func__, ret, scm_ret);
		return ret;
	}

	return 0;
}

static int dwc3_msm_link_clk_reset(struct dwc3_msm *mdwc, bool assert)
{
	int ret = 0;

	if (assert) {
		if (mdwc->pwr_event_irq)
			disable_irq(mdwc->pwr_event_irq);
		
		dev_dbg(mdwc->dev, "block_reset ASSERT\n");
		clk_disable_unprepare(mdwc->ref_clk);
		clk_disable_unprepare(mdwc->iface_clk);
		clk_disable_unprepare(mdwc->utmi_clk);
		clk_disable_unprepare(mdwc->sleep_clk);
		clk_disable_unprepare(mdwc->core_clk);
		ret = clk_reset(mdwc->core_clk, CLK_RESET_ASSERT);
		if (ret)
			dev_err(mdwc->dev, "dwc3 core_clk assert failed\n");
	} else {
		dev_dbg(mdwc->dev, "block_reset DEASSERT\n");
		ret = clk_reset(mdwc->core_clk, CLK_RESET_DEASSERT);
		ndelay(200);
		clk_prepare_enable(mdwc->core_clk);
		clk_prepare_enable(mdwc->sleep_clk);
		clk_prepare_enable(mdwc->utmi_clk);
		clk_prepare_enable(mdwc->ref_clk);
		clk_prepare_enable(mdwc->iface_clk);
		if (ret)
			dev_err(mdwc->dev, "dwc3 core_clk deassert failed\n");
		if (mdwc->pwr_event_irq)
			enable_irq(mdwc->pwr_event_irq);
	}

	return ret;
}

static void dwc3_msm_update_ref_clk(struct dwc3_msm *mdwc)
{
	u32 guctl, gfladj = 0;

	guctl = dwc3_msm_read_reg(mdwc->base, DWC3_GUCTL);
	guctl &= ~DWC3_GUCTL_REFCLKPER;

	
	if (dwc3_msm_read_reg(mdwc->base, DWC3_GSNPSID) >= DWC3_REVISION_250A) {
		gfladj = dwc3_msm_read_reg(mdwc->base, DWC3_GFLADJ);
		gfladj &= ~DWC3_GFLADJ_REFCLK_240MHZDECR_PLS1;
		gfladj &= ~DWC3_GFLADJ_REFCLK_240MHZ_DECR;
		gfladj &= ~DWC3_GFLADJ_REFCLK_LPM_SEL;
		gfladj &= ~DWC3_GFLADJ_REFCLK_FLADJ;
	}

	
	switch (mdwc->utmi_clk_rate) {
	case 19200000:
		guctl |= 52 << __ffs(DWC3_GUCTL_REFCLKPER);
		gfladj |= 12 << __ffs(DWC3_GFLADJ_REFCLK_240MHZ_DECR);
		gfladj |= DWC3_GFLADJ_REFCLK_240MHZDECR_PLS1;
		gfladj |= DWC3_GFLADJ_REFCLK_LPM_SEL;
		gfladj |= 200 << __ffs(DWC3_GFLADJ_REFCLK_FLADJ);
		break;
	case 24000000:
		guctl |= 41 << __ffs(DWC3_GUCTL_REFCLKPER);
		gfladj |= 10 << __ffs(DWC3_GFLADJ_REFCLK_240MHZ_DECR);
		gfladj |= DWC3_GFLADJ_REFCLK_LPM_SEL;
		gfladj |= 2032 << __ffs(DWC3_GFLADJ_REFCLK_FLADJ);
		break;
	default:
		dev_warn(mdwc->dev, "Unsupported utmi_clk_rate: %u\n",
				mdwc->utmi_clk_rate);
		break;
	}

	dwc3_msm_write_reg(mdwc->base, DWC3_GUCTL, guctl);
	if (gfladj)
		dwc3_msm_write_reg(mdwc->base, DWC3_GFLADJ, gfladj);
}

static void dwc3_msm_qscratch_reg_init(struct dwc3_msm *mdwc)
{
	if (dwc3_msm_read_reg(mdwc->base, DWC3_GSNPSID) < DWC3_REVISION_250A)
		
		dwc3_msm_write_reg_field(mdwc->base, QSCRATCH_GENERAL_CFG,
					 BIT(2), 1);

	dwc3_msm_write_reg(mdwc->base, CGCTL_REG,
		dwc3_msm_read_reg(mdwc->base, CGCTL_REG) | 0x18);

	mdwc->qscratch_ctl_val =
		dwc3_msm_read_reg(mdwc->base, QSCRATCH_CTRL_REG);
}

static void dwc3_msm_notify_event(struct dwc3 *dwc, unsigned event,
							unsigned value)
{
	struct dwc3_msm *mdwc = dev_get_drvdata(dwc->dev->parent);
	u32 reg;

	if (dwc->revision < DWC3_REVISION_230A)
		return;

	switch (event) {
	case DWC3_CONTROLLER_ERROR_EVENT:
		dev_info(mdwc->dev,
			"DWC3_CONTROLLER_ERROR_EVENT received, irq cnt %lu\n",
			dwc->irq_cnt);

		dwc3_msm_dump_phy_info(mdwc);
		dwc3_gadget_disable_irq(dwc);

		
		reg = dwc3_msm_read_reg(mdwc->base, DWC3_GCTL);
		reg |= DWC3_GCTL_CORESOFTRESET;
		dwc3_msm_write_reg(mdwc->base, DWC3_GCTL, reg);

		
		queue_work(system_nrt_wq, &mdwc->restart_usb_work);
		break;
	case DWC3_CONTROLLER_RESET_EVENT:
		dev_dbg(mdwc->dev, "DWC3_CONTROLLER_RESET_EVENT received\n");
		
		dwc3_msm_qscratch_reg_init(mdwc);
		break;
	case DWC3_CONTROLLER_POST_RESET_EVENT:
		dev_dbg(mdwc->dev,
				"DWC3_CONTROLLER_POST_RESET_EVENT received\n");
		
		usb_phy_set_params(mdwc->ss_phy);
		dwc3_msm_update_ref_clk(mdwc);
		dwc3_msm_restore_sec_config(mdwc->scm_dev_id);
		dwc->tx_fifo_size = mdwc->tx_fifo_size;
		break;
	case DWC3_CONTROLLER_POST_INITIALIZATION_EVENT:
		dwc3_msm_write_reg_field(mdwc->base, DWC3_GCTL,
					DWC3_GCTL_DSBLCLKGTNG, 1);
		usb_phy_post_init(mdwc->ss_phy);
		break;
	case DWC3_CONTROLLER_CONNDONE_EVENT:
		dev_dbg(mdwc->dev, "DWC3_CONTROLLER_CONNDONE_EVENT received\n");
		if (mdwc->dbm && dbm_l1_lpm_interrupt(mdwc->dbm))
			dwc3_msm_write_reg_field(mdwc->base,
					PWR_EVNT_IRQ_MASK_REG,
					PWR_EVNT_LPM_OUT_L1_MASK, 1);

		atomic_set(&dwc->in_lpm, 0);
		break;
<<<<<<< HEAD
=======
	case DWC3_CONTROLLER_NOTIFY_OTG_EVENT:
		dev_dbg(mdwc->dev, "DWC3_CONTROLLER_NOTIFY_OTG_EVENT received\n");
		if (dwc->enable_bus_suspend) {
			mdwc->suspend = dwc->b_suspend;
			queue_delayed_work(mdwc->dwc3_wq,
					&mdwc->resume_work, 0);
		}
		break;
	case DWC3_CONTROLLER_SET_CURRENT_DRAW_EVENT:
		dev_dbg(mdwc->dev, "DWC3_CONTROLLER_SET_CURRENT_DRAW_EVENT received\n");
		dwc3_msm_gadget_vbus_draw(mdwc, dwc->vbus_draw);
		break;
	case DWC3_CONTROLLER_RESTART_USB_SESSION:
		dev_dbg(mdwc->dev, "DWC3_CONTROLLER_RESTART_USB_SESSION received\n");
		dwc3_restart_usb_work(&mdwc->restart_usb_work);
		break;
	case DWC3_CONTROLLER_NOTIFY_DISABLE_UPDXFER:
		dwc3_msm_dbm_disable_updxfer(dwc, value);
		break;
>>>>>>> 0e91d2a... Nougat
	default:
		dev_dbg(mdwc->dev, "unknown dwc3 event\n");
		break;
	}
}

static void dwc3_msm_block_reset(struct dwc3_ext_xceiv *xceiv, bool core_reset)
{
	struct dwc3_msm *mdwc = container_of(xceiv, struct dwc3_msm, ext_xceiv);
	int ret  = 0;

	if (core_reset) {
		ret = dwc3_msm_link_clk_reset(mdwc, 1);
		if (ret)
			return;

		usleep_range(1000, 1200);
		ret = dwc3_msm_link_clk_reset(mdwc, 0);
		if (ret)
			return;

		usleep_range(10000, 12000);
	}

	
	dbm_soft_reset(mdwc->dbm, 1);
	usleep_range(1000, 1200);
	dbm_soft_reset(mdwc->dbm, 0);


	
	dwc3_msm_write_reg_field(mdwc->base, QSCRATCH_GENERAL_CFG,
		DBM_EN_MASK, 0x1);
	dbm_enable(mdwc->dbm);

}

static void dwc3_chg_enable_secondary_det(struct dwc3_msm *mdwc)
{
	u32 chg_ctrl;

	
	dwc3_msm_write_reg(mdwc->base, CHARGING_DET_CTRL_REG, 0x0);
	msleep(20);

	
	chg_ctrl = dwc3_msm_read_reg(mdwc->base, CHARGING_DET_CTRL_REG);
	if (chg_ctrl & 0x3F)
		dev_err(mdwc->dev, "%s Unable to reset chg_det block: %x\n",
						 __func__, chg_ctrl);
	dwc3_msm_write_readback(mdwc->base, CHARGING_DET_CTRL_REG, 0x3F, 0x34);
}

static bool dwc3_chg_det_check_linestate(struct dwc3_msm *mdwc)
{
	u32 chg_det;

	if (!prop_chg_detect)
		return false;

	chg_det = dwc3_msm_read_reg(mdwc->base, CHARGING_DET_OUTPUT_REG);
	return chg_det & (3 << 8);
}

int htc_dwc3_chg_det_check_linestate(void)
{
	u32 chg_det;
	struct dwc3_msm *mdwc = context;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	if (!atomic_read(&dwc->in_lpm)) {
		chg_det = dwc3_msm_read_reg(mdwc->base, CHARGING_DET_OUTPUT_REG);
	} else {
		pr_info("%s: Already suspended, can't get linestate\n", __func__);
		chg_det = 0;
	}
	return chg_det & (3 << 8);
}

static bool dwc3_chg_det_check_output(struct dwc3_msm *mdwc)
{
	u32 chg_det;
	bool ret = false;

	chg_det = dwc3_msm_read_reg(mdwc->base, CHARGING_DET_OUTPUT_REG);
	ret = chg_det & 1;

	return ret;
}

static void dwc3_chg_enable_primary_det(struct dwc3_msm *mdwc)
{
	dwc3_msm_write_readback(mdwc->base, CHARGING_DET_CTRL_REG, 0x3F, 0x30);
}

static inline bool dwc3_chg_check_dcd(struct dwc3_msm *mdwc)
{
	u32 chg_state;
	bool ret = false;

	chg_state = dwc3_msm_read_reg(mdwc->base, CHARGING_DET_OUTPUT_REG);
	ret = chg_state & 2;

	return ret;
}

static inline void dwc3_chg_disable_dcd(struct dwc3_msm *mdwc)
{
	dwc3_msm_write_readback(mdwc->base, CHARGING_DET_CTRL_REG, 0x3F, 0x0);
}

static inline void dwc3_chg_enable_dcd(struct dwc3_msm *mdwc)
{
	
	dwc3_msm_write_readback(mdwc->base, CHARGING_DET_CTRL_REG, 0x3F, 0x2);
}

static void dwc3_chg_block_reset(struct dwc3_msm *mdwc)
{
	u32 chg_ctrl;

	dwc3_msm_write_reg(mdwc->base, QSCRATCH_CTRL_REG,
			mdwc->qscratch_ctl_val);
	
	dwc3_msm_write_reg(mdwc->base, CHARGING_DET_CTRL_REG, 0x0);

	
	dwc3_msm_write_reg(mdwc->base, HS_PHY_IRQ_STAT_REG, 0xFFF);
	dwc3_msm_write_reg(mdwc->base, ALT_INTERRUPT_EN_REG, 0x0);

	udelay(100);

	
	chg_ctrl = dwc3_msm_read_reg(mdwc->base, CHARGING_DET_CTRL_REG);
	if (chg_ctrl & 0x3F)
		dev_err(mdwc->dev, "%s Unable to reset chg_det block: %x\n",
						 __func__, chg_ctrl);
}

static const char *chg_to_string(enum dwc3_chg_type chg_type)
{
	switch (chg_type) {
	case DWC3_SDP_CHARGER:		return "USB_SDP_CHARGER";
	case DWC3_DCP_CHARGER:		return "USB_DCP_CHARGER";
	case DWC3_CDP_CHARGER:		return "USB_CDP_CHARGER";
	case DWC3_PROPRIETARY_CHARGER:	return "USB_PROPRIETARY_CHARGER";
	case DWC3_FLOATED_CHARGER:	return "USB_FLOATED_CHARGER";
	default:			return "UNKNOWN_CHARGER";
	}
}

#define DWC3_CHG_DCD_POLL_TIME		(100 * HZ/1000) 
#define DWC3_CHG_DCD_MAX_RETRIES	6 
#define DWC3_CHG_PRIMARY_DET_TIME	(50 * HZ/1000) 
#define DWC3_CHG_SECONDARY_DET_TIME	(50 * HZ/1000) 

extern void usb_set_connect_type(int);
static void dwc3_chg_detect_work(struct work_struct *w)
{
	struct dwc3_msm *mdwc = container_of(w, struct dwc3_msm, chg_work.work);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3); 
	bool is_dcd = false, tmout, vout;
	static bool dcd;
	unsigned long delay;

	dev_dbg(mdwc->dev, "chg detection work\n");
	switch (mdwc->chg_state) {
	case USB_CHG_STATE_UNDEFINED:
		dwc3_chg_block_reset(mdwc);
		dwc3_chg_enable_dcd(mdwc);
		mdwc->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
		mdwc->dcd_retries = 0;
		delay = DWC3_CHG_DCD_POLL_TIME;
		break;
	case USB_CHG_STATE_WAIT_FOR_DCD:
		is_dcd = dwc3_chg_check_dcd(mdwc);
		tmout = ++mdwc->dcd_retries == DWC3_CHG_DCD_MAX_RETRIES;
		if (is_dcd || tmout) {
			if (is_dcd)
				dcd = true;
			else
				dcd = false;
			dwc3_chg_disable_dcd(mdwc);
			usleep_range(1000, 1200);
			if (dwc3_chg_det_check_linestate(mdwc)) {
				mdwc->charger.chg_type =
						DWC3_PROPRIETARY_CHARGER;
				mdwc->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
				break;
			}
			dwc3_chg_enable_primary_det(mdwc);
			delay = DWC3_CHG_PRIMARY_DET_TIME;
			mdwc->chg_state = USB_CHG_STATE_DCD_DONE;
		} else {
			delay = DWC3_CHG_DCD_POLL_TIME;
		}
		break;
	case USB_CHG_STATE_DCD_DONE:
		vout = dwc3_chg_det_check_output(mdwc);
		if (vout) {
			dwc3_chg_enable_secondary_det(mdwc);
			delay = DWC3_CHG_SECONDARY_DET_TIME;
			mdwc->chg_state = USB_CHG_STATE_PRIMARY_DONE;
		} else {
			if (!dcd && prop_chg_detect)
				mdwc->charger.chg_type =
						DWC3_FLOATED_CHARGER;
			else
				mdwc->charger.chg_type = DWC3_SDP_CHARGER;
			mdwc->chg_state = USB_CHG_STATE_DETECTED;
			delay = 0;
		}
		break;
	case USB_CHG_STATE_PRIMARY_DONE:
		vout = dwc3_chg_det_check_output(mdwc);
		if (vout)
			mdwc->charger.chg_type = DWC3_DCP_CHARGER;
		else
			mdwc->charger.chg_type = DWC3_CDP_CHARGER;
		mdwc->chg_state = USB_CHG_STATE_SECONDARY_DONE;
		usb_set_connect_type(CONNECT_TYPE_AC);
		
	case USB_CHG_STATE_SECONDARY_DONE:
		mdwc->chg_state = USB_CHG_STATE_DETECTED;
		
	case USB_CHG_STATE_DETECTED:
		dwc3_chg_block_reset(mdwc);
		
		if (mdwc->charger.chg_type == DWC3_DCP_CHARGER) {
			dwc3_msm_write_readback(mdwc->base,
					CHARGING_DET_CTRL_REG, 0x1F, 0x10);
			if (mdwc->ext_chg_opened) {
				init_completion(&mdwc->ext_chg_wait);
				mdwc->ext_chg_active = true;
			}
		}
		if (mdwc->charger.chg_type != DWC3_SDP_CHARGER)
			cancel_delayed_work(&dwc->dotg->unknown_charger_notify_work);

		USB_INFO("%s : charger type: %s\n", __func__, chg_to_string(mdwc->charger.chg_type));
		dev_dbg(mdwc->dev, "chg_type = %s\n",
			chg_to_string(mdwc->charger.chg_type));
		mdwc->charger.notify_detection_complete(mdwc->otg_xceiv->otg,
								&mdwc->charger);
		return;
	default:
		return;
	}

	queue_delayed_work(system_nrt_wq, &mdwc->chg_work, delay);
}

static void dwc3_start_chg_det(struct dwc3_charger *charger, bool start)
{
	struct dwc3_msm *mdwc = container_of(charger, struct dwc3_msm, charger);

	if (start == false) {
		dev_dbg(mdwc->dev, "canceling charging detection work\n");
		cancel_delayed_work_sync(&mdwc->chg_work);
		mdwc->chg_state = USB_CHG_STATE_UNDEFINED;
		charger->chg_type = DWC3_INVALID_CHARGER;
		return;
	}

	
	if (mdwc->chg_state == USB_CHG_STATE_DETECTED &&
		charger->chg_type != DWC3_INVALID_CHARGER)
		return;

	mdwc->chg_state = USB_CHG_STATE_UNDEFINED;
	charger->chg_type = DWC3_INVALID_CHARGER;
	queue_delayed_work(system_nrt_wq, &mdwc->chg_work, 0);
}

static void dwc3_msm_power_collapse_por(struct dwc3_msm *mdwc)
{
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	dwc3_core_init(dwc);
	
	dwc3_event_buffers_setup(dwc);
	dwc3_msm_notify_event(dwc,
				DWC3_CONTROLLER_POST_INITIALIZATION_EVENT, 0);
}

static int dwc3_msm_prepare_suspend(struct dwc3_msm *mdwc)
{
	unsigned long timeout;
	u32 reg = 0;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	if (dwc3_msm_is_superspeed(mdwc)) {
		if (!atomic_read(&mdwc->in_p3)) {
			dev_err(mdwc->dev,
				"Not in P3, stoping LPM sequence\n");
			return -EBUSY;
		}
	} else {
		
		dwc3_msm_write_reg(mdwc->base, PWR_EVNT_IRQ_STAT_REG,
			PWR_EVNT_LPM_IN_L2_MASK |
			PWR_EVNT_LPM_OUT_L2_MASK);

		
		dwc3_msm_write_reg(mdwc->base,
			DWC3_GUSB2PHYCFG(0),
			dwc3_msm_read_reg(mdwc->base,
			DWC3_GUSB2PHYCFG(0)) |
			DWC3_GUSB2PHYCFG_ENBLSLPM |
			DWC3_GUSB2PHYCFG_SUSPHY);

		
		timeout = jiffies + msecs_to_jiffies(5);
		while (!time_after(jiffies, timeout)) {
			reg = dwc3_msm_read_reg(mdwc->base,
				PWR_EVNT_IRQ_STAT_REG);
			if (reg & PWR_EVNT_LPM_IN_L2_MASK)
				break;
		}
		if (!(reg & PWR_EVNT_LPM_IN_L2_MASK)) {
			dev_err(mdwc->dev,
				"could not transition HS PHY to L2\n");
			return -EBUSY;
		}

		
		dwc3_msm_write_reg(mdwc->base, PWR_EVNT_IRQ_STAT_REG,
			PWR_EVNT_LPM_IN_L2_MASK);
	}
<<<<<<< HEAD
=======

	if (!(reg & PWR_EVNT_LPM_IN_L2_MASK)) {
		dev_err(mdwc->dev, "could not transition HS PHY to L2\n");
		dbg_event(0xFF, "PWR_EVNT_LPM",
			dwc3_msm_read_reg(mdwc->base, PWR_EVNT_IRQ_STAT_REG));
		dbg_event(0xFF, "QUSB_STS",
			dwc3_msm_read_reg(mdwc->base, QSCRATCH_USB30_STS_REG));
		
		if (mdwc->in_host_mode || (mdwc->vbus_active
			&& mdwc->otg_state == OTG_STATE_B_SUSPEND)) {
			queue_delayed_work(mdwc->dwc3_wq,
					&mdwc->resume_work, 0);
			return -EBUSY;
		}
	}

	
	dwc3_msm_write_reg(mdwc->base, PWR_EVNT_IRQ_STAT_REG,
		PWR_EVNT_LPM_IN_L2_MASK);
>>>>>>> 0e91d2a... Nougat

	return 0;
}

static void dwc3_msm_wake_interrupt_enable(struct dwc3_msm *mdwc, bool on)
{
	u32 irq_mask;

	if (on) {
		
		irq_mask = dwc3_msm_read_reg(mdwc->base, PWR_EVNT_IRQ_MASK_REG);
		irq_mask |= PWR_EVNT_LPM_OUT_L2_MASK |
				PWR_EVNT_POWERDOWN_OUT_P3_MASK;
		dwc3_msm_write_reg(mdwc->base, PWR_EVNT_IRQ_MASK_REG, irq_mask);
	} else {
		static const u32 pwr_events = PWR_EVNT_POWERDOWN_OUT_P3_MASK |
					      PWR_EVNT_LPM_OUT_L2_MASK;

		
		irq_mask = dwc3_msm_read_reg(mdwc->base, PWR_EVNT_IRQ_MASK_REG);
		irq_mask &= ~pwr_events;
		dwc3_msm_write_reg(mdwc->base, PWR_EVNT_IRQ_MASK_REG, irq_mask);

		
		dwc3_msm_write_reg(mdwc->base, PWR_EVNT_IRQ_STAT_REG,
				   pwr_events);
	}
}

static int dwc3_msm_suspend(struct dwc3_msm *mdwc)
{
	int ret, i;
	bool dcp;
	bool host_bus_suspend;
	bool host_ss_active;
	bool can_suspend_ssphy;
	bool device_bus_suspend = false;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(mdwc->dev, "%s: entering lpm. usb_lpm_override:%d\n",
					 __func__, usb_lpm_override);
	dbg_event(0xFF, "Ctl Sus", atomic_read(&dwc->in_lpm));

	if (!usb_lpm_override && mdwc->suspend_resume_no_support) {
		dev_dbg(mdwc->dev, "%s no support for suspend\n", __func__);
		return -EPERM;
	}

	if (atomic_read(&dwc->in_lpm)) {
		dev_dbg(mdwc->dev, "%s: Already suspended\n", __func__);
		return 0;
	}

	if (mdwc->otg_xceiv && mdwc->otg_xceiv->state == OTG_STATE_B_PERIPHERAL)
		device_bus_suspend = true;

	host_bus_suspend = (mdwc->scope == POWER_SUPPLY_SCOPE_SYSTEM);

	if (!host_bus_suspend) {
		
		for (i = 0; i < dwc->num_event_buffers; i++) {
			struct dwc3_event_buffer *evt = dwc->ev_buffs[i];
			if ((evt->flags & DWC3_EVENT_PENDING)) {
				dev_dbg(mdwc->dev,
				"%s: %d device events pending, abort suspend\n",
				__func__, evt->count / 4);
				dbg_print_reg("PENDING DEVICE EVENT",
						*(u32 *)(evt->buf + evt->lpos));
				return -EBUSY;
			}
		}

		if (!msm_bam_usb_lpm_ok(DWC3_CTRL)) {
			dev_dbg(mdwc->dev, "%s: IPA handshake not finished, will suspend when done\n",
					__func__);
			pm_schedule_suspend(mdwc->dev, 1000);
			return -EBUSY;
		}
	}

	if (!mdwc->vbus_active && mdwc->otg_xceiv &&
		mdwc->otg_xceiv->state == OTG_STATE_B_PERIPHERAL) {
		dev_dbg(mdwc->dev,
			"%s: cable disconnected while not in idle otg state\n",
			__func__);
		return -EBUSY;
	}

	host_ss_active = dwc3_msm_is_host_superspeed(mdwc);

	if (cancel_delayed_work_sync(&mdwc->chg_work))
		dev_dbg(mdwc->dev, "%s: chg_work was pending\n", __func__);
	if (mdwc->chg_state != USB_CHG_STATE_DETECTED) {
		
		mdwc->chg_state = USB_CHG_STATE_UNDEFINED;
		mdwc->charger.chg_type = DWC3_INVALID_CHARGER;
		dwc3_msm_write_readback(mdwc->base, CHARGING_DET_CTRL_REG,
								0x37, 0x0);
	}

	dcp = ((mdwc->charger.chg_type == DWC3_DCP_CHARGER) ||
	      (mdwc->charger.chg_type == DWC3_PROPRIETARY_CHARGER) ||
	      (mdwc->charger.chg_type == DWC3_FLOATED_CHARGER));
	if (dcp) {
		mdwc->hs_phy->flags |= PHY_CHARGER_CONNECTED;
		mdwc->ss_phy->flags |= PHY_CHARGER_CONNECTED;
	} else {
		mdwc->hs_phy->flags &= ~PHY_CHARGER_CONNECTED;
		mdwc->ss_phy->flags &= ~PHY_CHARGER_CONNECTED;
	}

	can_suspend_ssphy = !(host_bus_suspend && host_ss_active);

	if (host_bus_suspend || device_bus_suspend) {

		if (device_bus_suspend &&
			(dwc->gadget.state != USB_STATE_CONFIGURED)) {
			pr_err("%s(): Trying to go in LPM with state:%d\n",
						__func__, dwc->gadget.state);
			pr_err("%s(): LPM is not performed.\n", __func__);
			return -EBUSY;
		}

		ret = dwc3_msm_prepare_suspend(mdwc);
		if (ret)
			return ret;
	}

	tasklet_kill(&dwc->bh);
	
	if (dwc->irq)
		disable_irq(dwc->irq);

	if (!dcp && !host_bus_suspend)
		dwc3_msm_write_reg(mdwc->base, QSCRATCH_CTRL_REG,
			mdwc->qscratch_ctl_val);

	
	if (mdwc->pwr_event_irq) {
		dwc3_msm_wake_interrupt_enable(mdwc, true);
		enable_irq_wake(mdwc->pwr_event_irq);
	}

	atomic_set(&dwc->in_lpm, 1);

	usb_phy_set_suspend(mdwc->hs_phy, 1);

	if (can_suspend_ssphy) {
		usb_phy_set_suspend(mdwc->ss_phy, 1);
		usleep_range(1000, 1200);
		clk_disable_unprepare(mdwc->ref_clk);
		mdwc->lpm_flags |= MDWC3_PHY_REF_CLK_OFF;
	}

	
	wmb();

	
	if (!host_bus_suspend && !device_bus_suspend && mdwc->power_collapse) {
		mdwc->lpm_flags |= MDWC3_POWER_COLLAPSE;
		dev_dbg(mdwc->dev, "%s: power collapse\n", __func__);
		dwc3_msm_config_gdsc(mdwc, 0);
		clk_disable_unprepare(mdwc->sleep_clk);
	}

	clk_disable_unprepare(mdwc->iface_clk);
	clk_disable_unprepare(mdwc->utmi_clk);

	if (can_suspend_ssphy) {
		clk_set_rate(mdwc->core_clk, 19200000);
		clk_disable_unprepare(mdwc->core_clk);
		mdwc->lpm_flags |= MDWC3_CORECLK_OFF;
		
		clk_disable_unprepare(mdwc->xo_clk);
		mdwc->lpm_flags |= MDWC3_TCXO_SHUTDOWN;
	}

	if (mdwc->bus_perf_client) {
		ret = msm_bus_scale_client_update_request(
						mdwc->bus_perf_client, 0);
		if (ret)
			dev_err(mdwc->dev, "Failed to reset bus bw vote\n");
	}

	if (mdwc->lpm_to_suspend_delay) {
		dev_dbg(mdwc->dev, "defer suspend with %d(msecs)\n",
					mdwc->lpm_to_suspend_delay);
		pm_wakeup_event(mdwc->dev, mdwc->lpm_to_suspend_delay);
	} else {
		pm_relax(mdwc->dev);
	}

	if (mdwc->hs_phy_irq) {
		if (host_bus_suspend || device_bus_suspend) {
			enable_irq_wake(mdwc->hs_phy_irq);
			mdwc->lpm_flags |= MDWC3_ASYNC_IRQ_WAKE_CAPABILITY;
		}
	}

	dev_info(mdwc->dev, "DWC3 in low power mode\n");
	return 0;
}

static int dwc3_msm_resume(struct dwc3_msm *mdwc)
{
	int ret;
	bool dcp;
	bool host_bus_suspend;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(mdwc->dev, "%s: exiting lpm\n", __func__);
	dbg_event(0xFF, "Ctl Res", atomic_read(&dwc->in_lpm));

	if (!atomic_read(&dwc->in_lpm)) {
		dev_dbg(mdwc->dev, "%s: Already resumed\n", __func__);
		return 0;
	}

	pm_stay_awake(mdwc->dev);

	if (mdwc->bus_perf_client) {
		ret = msm_bus_scale_client_update_request(
						mdwc->bus_perf_client, 1);
		if (ret)
			dev_err(mdwc->dev, "Failed to vote for bus scaling\n");
	}

	dcp = ((mdwc->charger.chg_type == DWC3_DCP_CHARGER) ||
	      (mdwc->charger.chg_type == DWC3_PROPRIETARY_CHARGER) ||
	      (mdwc->charger.chg_type == DWC3_FLOATED_CHARGER));
	if (dcp) {
		mdwc->hs_phy->flags |= PHY_CHARGER_CONNECTED;
		mdwc->ss_phy->flags |= PHY_CHARGER_CONNECTED;
	} else {
		mdwc->hs_phy->flags &= ~PHY_CHARGER_CONNECTED;
		mdwc->ss_phy->flags &= ~PHY_CHARGER_CONNECTED;
	}

	host_bus_suspend = (mdwc->scope == POWER_SUPPLY_SCOPE_SYSTEM);

	if (mdwc->lpm_flags & MDWC3_TCXO_SHUTDOWN) {
		
		ret = clk_prepare_enable(mdwc->xo_clk);
		if (ret)
			dev_err(mdwc->dev, "%s failed to vote TCXO buffer%d\n",
						__func__, ret);
		mdwc->lpm_flags &= ~MDWC3_TCXO_SHUTDOWN;
	}

	
	if (mdwc->lpm_flags & MDWC3_POWER_COLLAPSE) {
		dev_dbg(mdwc->dev, "%s: exit power collapse\n", __func__);
		dwc3_msm_config_gdsc(mdwc, 1);

		if (mdwc->phy_com_reset)
			clk_reset(mdwc->phy_com_reset, CLK_RESET_ASSERT);
		clk_reset(mdwc->core_clk, CLK_RESET_ASSERT);
		
		usleep_range(1000, 1200);
		clk_reset(mdwc->core_clk, CLK_RESET_DEASSERT);
		if (mdwc->phy_com_reset)
			clk_reset(mdwc->phy_com_reset, CLK_RESET_DEASSERT);
	}

<<<<<<< HEAD

	if (mdwc->lpm_flags & MDWC3_PHY_REF_CLK_OFF) {
		clk_prepare_enable(mdwc->ref_clk);
=======
	clk_prepare_enable(mdwc->iface_clk);
	clk_set_rate(mdwc->core_clk, mdwc->core_clk_rate);
	clk_prepare_enable(mdwc->core_clk);
	clk_prepare_enable(mdwc->utmi_clk);
	if (mdwc->bus_aggr_clk)
		clk_prepare_enable(mdwc->bus_aggr_clk);

	
	if (mdwc->lpm_flags & MDWC3_SS_PHY_SUSPEND) {
>>>>>>> 0e91d2a... Nougat
		usb_phy_set_suspend(mdwc->ss_phy, 0);
		mdwc->lpm_flags &= ~MDWC3_PHY_REF_CLK_OFF;
	}
	usleep_range(1000, 1200);

<<<<<<< HEAD
	clk_prepare_enable(mdwc->iface_clk);
	if (mdwc->lpm_flags & MDWC3_CORECLK_OFF) {
		clk_set_rate(mdwc->core_clk, 125000000);
		clk_prepare_enable(mdwc->core_clk);
		mdwc->lpm_flags &= ~MDWC3_CORECLK_OFF;
	}

	clk_prepare_enable(mdwc->utmi_clk);

	if (mdwc->lpm_flags & MDWC3_POWER_COLLAPSE)
		clk_prepare_enable(mdwc->sleep_clk);

	usb_phy_set_suspend(mdwc->hs_phy, 0);

=======
>>>>>>> 0e91d2a... Nougat
	
	if (mdwc->lpm_flags & MDWC3_POWER_COLLAPSE) {
		dev_dbg(mdwc->dev, "%s: exit power collapse (POR=%d)\n",
			__func__, mdwc->power_collapse_por);

		
		ret = usb_phy_reset(mdwc->hs_phy);
		if (ret) {
			dev_err(mdwc->dev, "hsphy reset failed\n");
			return ret;
		}

		
		ret = usb_phy_reset(mdwc->ss_phy);
		if (ret) {
			dev_err(mdwc->dev, "ssphy reset failed\n");
			return ret;
		}

		ret = dwc3_msm_restore_sec_config(mdwc->scm_dev_id);
		if (ret)
			return ret;

		if (mdwc->power_collapse_por)
			dwc3_msm_power_collapse_por(mdwc);

		
		dwc3_msm_write_reg_field(mdwc->base, PWR_EVNT_IRQ_MASK_REG,
					PWR_EVNT_POWERDOWN_IN_P3_MASK, 1);

		mdwc->lpm_flags &= ~MDWC3_POWER_COLLAPSE;
	}

	atomic_set(&dwc->in_lpm, 0);

	msm_bam_notify_lpm_resume(DWC3_CTRL);
	
	if (mdwc->pwr_event_irq) {
		disable_irq_wake(mdwc->pwr_event_irq);
		dwc3_msm_wake_interrupt_enable(mdwc, false);
	}

	
	dwc3_msm_write_reg(mdwc->base, DWC3_GUSB2PHYCFG(0),
		dwc3_msm_read_reg(mdwc->base, DWC3_GUSB2PHYCFG(0)) &
				~(DWC3_GUSB2PHYCFG_ENBLSLPM |
					DWC3_GUSB2PHYCFG_SUSPHY));

	
	if (mdwc->hs_phy_irq &&
			(mdwc->lpm_flags & MDWC3_ASYNC_IRQ_WAKE_CAPABILITY)) {
			disable_irq_wake(mdwc->hs_phy_irq);
			mdwc->lpm_flags &= ~MDWC3_ASYNC_IRQ_WAKE_CAPABILITY;
	}

	dev_info(mdwc->dev, "DWC3 exited from low power mode\n");

	
	if (dwc->irq)
		enable_irq(dwc->irq);

	dwc3_pwr_event_handler(mdwc);

	return 0;
}

static void dwc3_wait_for_ext_chg_done(struct dwc3_msm *mdwc)
{
<<<<<<< HEAD
	unsigned long t;
=======
	if (mdwc->init)
		flush_delayed_work(&mdwc->sm_work);

	if (mdwc->id_state == DWC3_ID_FLOAT) {
		dev_dbg(mdwc->dev, "XCVR: ID set\n");
		set_bit(ID, &mdwc->inputs);
	} else {
		dev_dbg(mdwc->dev, "XCVR: ID clear\n");
		clear_bit(ID, &mdwc->inputs);
	}
>>>>>>> 0e91d2a... Nougat


	if (mdwc->ext_chg_active && (mdwc->ext_xceiv.bsv ||
				!mdwc->ext_xceiv.id)) {

<<<<<<< HEAD
		dev_dbg(mdwc->dev, "before ext chg wait\n");

		t = wait_for_completion_timeout(&mdwc->ext_chg_wait,
				msecs_to_jiffies(3000));
		if (!t)
			dev_err(mdwc->dev, "ext chg wait timeout\n");
		else
			dev_dbg(mdwc->dev, "ext chg wait done\n");
	}

=======
	if (!mdwc->init) {
		mdwc->init = true;
		pm_runtime_set_autosuspend_delay(mdwc->dev, 1000);
		pm_runtime_use_autosuspend(mdwc->dev);
		if (!work_busy(&mdwc->sm_work.work))
			schedule_delayed_work(&mdwc->sm_work, 0);
		return;
	}

	schedule_delayed_work(&mdwc->sm_work, 0);
>>>>>>> 0e91d2a... Nougat
}

static void dwc3_resume_work(struct work_struct *w)
{
	struct dwc3_msm *mdwc = container_of(w, struct dwc3_msm,
							resume_work.work);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	USB_INFO("%s: mhl_online=%d, vbus_active=%d\n",
		__func__, mdwc->mhl_online, mdwc->vbus_active);

	dev_dbg(mdwc->dev, "%s: dwc3 resume work\n", __func__);
	
	if (!atomic_read(&dwc->in_lpm)) {
		dev_dbg(mdwc->dev, "%s: notifying xceiv event\n", __func__);
		dbg_event(0xFF, "RWrk !lpm", 0);
		if (mdwc->otg_xceiv) {
			dwc3_wait_for_ext_chg_done(mdwc);
			mdwc->ext_xceiv.notify_ext_events(mdwc->otg_xceiv->otg,
							DWC3_EVENT_XCEIV_STATE);
		}
		return;
	}

	
	if (atomic_read(&mdwc->pm_suspended)) {
		dbg_event(0xFF, "RWrk PMSus", 0);
		mdwc->resume_pending = true;
	} else {
		dbg_event(0xFF, "RWrk !PMSus", mdwc->otg_xceiv ? 1 : 0);
		pm_runtime_get_sync(mdwc->dev);
		if (mdwc->otg_xceiv) {
			mdwc->ext_xceiv.notify_ext_events(mdwc->otg_xceiv->otg,
							DWC3_EVENT_PHY_RESUME);
		} else if (mdwc->scope == POWER_SUPPLY_SCOPE_SYSTEM) {
			struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
			pm_runtime_resume(&dwc->xhci->dev);
		}

		pm_runtime_put_noidle(mdwc->dev);
		if (mdwc->otg_xceiv) {
			dwc3_wait_for_ext_chg_done(mdwc);
			mdwc->ext_xceiv.notify_ext_events(mdwc->otg_xceiv->otg,
							DWC3_EVENT_XCEIV_STATE);
		}
	}
}

static void dwc3_pwr_event_handler(struct dwc3_msm *mdwc)
{
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	u32 irq_stat, irq_clear = 0;

	pm_runtime_get_noresume(mdwc->dev);

	irq_stat = dwc3_msm_read_reg(mdwc->base, PWR_EVNT_IRQ_STAT_REG);
	dev_dbg(mdwc->dev, "%s irq_stat=%X\n", __func__, irq_stat);

	
	if ((irq_stat & PWR_EVNT_POWERDOWN_OUT_P3_MASK) &&
			(irq_stat & PWR_EVNT_POWERDOWN_IN_P3_MASK)) {
		
		u32 ls = dwc3_msm_read_reg_field(mdwc->base,
				DWC3_GDBGLTSSM, DWC3_GDBGLTSSM_LINKSTATE_MASK);
		dev_dbg(mdwc->dev, "%s link state = 0x%04x\n", __func__, ls);
		atomic_set(&mdwc->in_p3, ls == DWC3_LINK_STATE_U3);

		irq_stat &= ~(PWR_EVNT_POWERDOWN_OUT_P3_MASK |
				PWR_EVNT_POWERDOWN_IN_P3_MASK);
		irq_clear |= (PWR_EVNT_POWERDOWN_OUT_P3_MASK |
				PWR_EVNT_POWERDOWN_IN_P3_MASK);
	} else if (irq_stat & PWR_EVNT_POWERDOWN_OUT_P3_MASK) {
		atomic_set(&mdwc->in_p3, 0);
		irq_stat &= ~PWR_EVNT_POWERDOWN_OUT_P3_MASK;
		irq_clear |= PWR_EVNT_POWERDOWN_OUT_P3_MASK;
	} else if (irq_stat & PWR_EVNT_POWERDOWN_IN_P3_MASK) {
		atomic_set(&mdwc->in_p3, 1);
		irq_stat &= ~PWR_EVNT_POWERDOWN_IN_P3_MASK;
		irq_clear |= PWR_EVNT_POWERDOWN_IN_P3_MASK;
	}

	
	if (irq_stat & PWR_EVNT_LPM_OUT_L2_MASK) {
		irq_stat &= ~PWR_EVNT_LPM_OUT_L2_MASK;
		irq_stat |= PWR_EVNT_LPM_OUT_L2_MASK;
	}

	
	if (irq_stat & PWR_EVNT_LPM_OUT_L1_MASK) {
		dev_dbg(mdwc->dev, "%s: handling PWR_EVNT_LPM_OUT_L1_MASK\n",
				__func__);
		if (usb_gadget_wakeup(&dwc->gadget))
			dev_err(mdwc->dev, "%s failed to take dwc out of L1\n",
					__func__);
		irq_stat &= ~PWR_EVNT_LPM_OUT_L1_MASK;
		irq_clear |= PWR_EVNT_LPM_OUT_L1_MASK;
	}

	
	if (irq_stat)
		dev_dbg(mdwc->dev, "%s: unexpected PWR_EVNT, irq_stat=%X\n",
			__func__, irq_stat);

	dwc3_msm_write_reg(mdwc->base, PWR_EVNT_IRQ_STAT_REG, irq_clear);
	pm_runtime_put(mdwc->dev);
}

static irqreturn_t msm_dwc3_pwr_irq_thread(int irq, void *_mdwc)
{
	struct dwc3_msm *mdwc = _mdwc;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(mdwc->dev, "%s\n", __func__);
	dbg_event(0xFF, "PWR IRQ", atomic_read(&dwc->in_lpm));

	if (atomic_read(&dwc->in_lpm))
		dwc3_resume_work(&mdwc->resume_work.work);
	else
		dwc3_pwr_event_handler(mdwc);
	return IRQ_HANDLED;
}

static u32 debug_id = true, debug_bsv, debug_connect;

static int dwc3_connect_show(struct seq_file *s, void *unused)
{
	if (debug_connect)
		seq_printf(s, "true\n");
	else
		seq_printf(s, "false\n");

	return 0;
}

static int dwc3_connect_open(struct inode *inode, struct file *file)
{
	return single_open(file, dwc3_connect_show, inode->i_private);
}

static ssize_t dwc3_connect_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct dwc3_msm *mdwc = s->private;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	char buf[8];

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6) || !strncmp(buf, "true", 4)) {
		debug_connect = true;
	} else {
		debug_connect = debug_bsv = false;
		debug_id = true;
	}

	mdwc->ext_xceiv.bsv = debug_bsv;
	mdwc->ext_xceiv.id = debug_id ? DWC3_ID_FLOAT : DWC3_ID_GROUND;

	if (atomic_read(&dwc->in_lpm)) {
		dev_dbg(mdwc->dev, "%s: calling resume_work\n", __func__);
		dwc3_resume_work(&mdwc->resume_work.work);
	} else {
		dev_dbg(mdwc->dev, "%s: notifying xceiv event\n", __func__);
		if (mdwc->otg_xceiv)
			mdwc->ext_xceiv.notify_ext_events(mdwc->otg_xceiv->otg,
							DWC3_EVENT_XCEIV_STATE);
	}

	return count;
}

const struct file_operations dwc3_connect_fops = {
	.open = dwc3_connect_open,
	.read = seq_read,
	.write = dwc3_connect_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *dwc3_debugfs_root;

static void dwc3_msm_debugfs_init(struct dwc3_msm *mdwc)
{
	dwc3_debugfs_root = debugfs_create_dir("msm_dwc3", NULL);

	if (!dwc3_debugfs_root || IS_ERR(dwc3_debugfs_root))
		return;

	if (!debugfs_create_bool("id", S_IRUGO | S_IWUSR, dwc3_debugfs_root,
				 &debug_id))
		goto error;

	if (!debugfs_create_bool("bsv", S_IRUGO | S_IWUSR, dwc3_debugfs_root,
				 &debug_bsv))
		goto error;

	if (!debugfs_create_file("connect", S_IRUGO | S_IWUSR,
				dwc3_debugfs_root, mdwc, &dwc3_connect_fops))
		goto error;

	return;

error:
	debugfs_remove_recursive(dwc3_debugfs_root);
}

static irqreturn_t msm_dwc3_hs_phy_irq(int irq, void *data)
{
	struct dwc3_msm *mdwc = data;

	dev_dbg(mdwc->dev, "%s HS PHY IRQ handled\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t msm_dwc3_pwr_irq(int irq, void *data)
{
	struct dwc3_msm *mdwc = data;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(mdwc->dev, "%s received\n", __func__);
	if (atomic_read(&dwc->in_lpm)) {
		
		if (!atomic_read(&mdwc->pm_suspended))
				return IRQ_WAKE_THREAD;

		mdwc->resume_pending = true;

		return IRQ_HANDLED;
	}

	dwc3_pwr_event_handler(mdwc);
	return IRQ_HANDLED;
}

void htc_dwc3_disable_usb(int state)
{
	struct dwc3_msm *mdwc = context;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	printk(KERN_INFO "[USB] %s state : %d\n", __func__, state);

	if (state == 1) {
		mdwc->charger.usb_disable = 1;
		flush_delayed_work(&mdwc->resume_work);
		if (!atomic_read(&dwc->in_lpm)) {
			mdwc->ext_xceiv.bsv = 0;
			mdwc->ext_xceiv.id = DWC3_ID_FLOAT;
			if (mdwc->otg_xceiv)
				mdwc->ext_xceiv.notify_ext_events(mdwc->otg_xceiv->otg, DWC3_EVENT_XCEIV_STATE);
		}
	} else if (state == 2) {
		mdwc->charger.usb_disable = 2;
	} else {
		mdwc->charger.usb_disable = 0;
		mdwc->ext_xceiv.bsv = htc_vbus_backup;
		mdwc->ext_xceiv.id = htc_id_backup;
		queue_delayed_work(system_nrt_wq, &mdwc->resume_work, 20);
	}
}

static int dwc3_msm_remove_pulldown(struct dwc3_msm *mdwc, bool rm_pulldown)
{
	int ret = 0;

	if (!rm_pulldown)
		goto disable_vdda33;

	ret = regulator_set_optimum_mode(mdwc->vdda33, DWC3_3P3_HPM_LOAD);
	if (ret < 0) {
		dev_err(mdwc->dev, "Unable to set HPM of vdda33:%d\n", ret);
		return ret;
	}

	ret = regulator_set_voltage(mdwc->vdda33, DWC3_3P3_VOL_MIN,
						DWC3_3P3_VOL_MAX);
	if (ret) {
		dev_err(mdwc->dev,
				"Unable to set voltage for vdda33:%d\n", ret);
		goto put_vdda33_lpm;
	}

	ret = regulator_enable(mdwc->vdda33);
	if (ret) {
		dev_err(mdwc->dev, "Unable to enable vdda33:%d\n", ret);
		goto unset_vdd33;
	}

	return 0;

disable_vdda33:
	ret = regulator_disable(mdwc->vdda33);
	if (ret)
		dev_err(mdwc->dev, "Unable to disable vdda33:%d\n", ret);

unset_vdd33:
	ret = regulator_set_voltage(mdwc->vdda33, 0, DWC3_3P3_VOL_MAX);
	if (ret)
		dev_err(mdwc->dev,
			"Unable to set (0) voltage for vdda33:%d\n", ret);

put_vdda33_lpm:
	ret = regulator_set_optimum_mode(mdwc->vdda33, 0);
	if (ret < 0)
		dev_err(mdwc->dev, "Unable to set (0) HPM of vdda33\n");

	return ret;
}

static int
get_prop_usbin_voltage_now(struct dwc3_msm *mdwc)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (IS_ERR_OR_NULL(mdwc->vadc_dev)) {
		mdwc->vadc_dev = qpnp_get_vadc(mdwc->dev, "usbin");
		if (IS_ERR(mdwc->vadc_dev))
			return PTR_ERR(mdwc->vadc_dev);
	}

	rc = qpnp_vadc_read(mdwc->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read usbin rc=%d\n", rc);
		return 0;
	} else {
		return results.physical;
	}
}

static int dwc3_msm_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct dwc3_msm *mdwc = container_of(psy, struct dwc3_msm,
								usb_psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = mdwc->scope;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = mdwc->voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = mdwc->current_max;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = mdwc->vbus_active;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mdwc->online;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = psy->type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_usbin_voltage_now(mdwc);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = mdwc->health_status;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

extern int vbus;
static int dwc3_msm_power_set_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct dwc3_msm *mdwc = container_of(psy, struct dwc3_msm,
								usb_psy);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	struct dwc3_otg *dotg = dwc->dotg;
	struct usb_phy *phy = dotg->otg.phy;

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_OTG:
<<<<<<< HEAD
=======
		id = val->intval ? DWC3_ID_GROUND : DWC3_ID_FLOAT;
		if (mdwc->id_state == id)
			break;

		if (disable_host_mode && !id) {
			dev_dbg(mdwc->dev, "%s: Ignoring ID change event :%d\n",
						__func__, mdwc->id_state);
			break;
		}

>>>>>>> 0e91d2a... Nougat
		
		mdwc->id_state = val->intval ? DWC3_ID_GROUND : DWC3_ID_FLOAT;
		if (mdwc->otg_xceiv)
			queue_work(system_nrt_wq, &mdwc->id_work);

		break;
	case POWER_SUPPLY_PROP_SCOPE:
		mdwc->scope = val->intval;
		if (mdwc->scope == POWER_SUPPLY_SCOPE_SYSTEM) {
			mdwc->hs_phy->flags |= PHY_HOST_MODE;
			mdwc->ss_phy->flags |= PHY_HOST_MODE;
		} else {
			mdwc->hs_phy->flags &= ~PHY_HOST_MODE;
			mdwc->ss_phy->flags &= ~PHY_HOST_MODE;
		}
		break;
	
	case POWER_SUPPLY_PROP_ALLOW_DETECTION:
		if (mdwc->rm_pulldown == val->intval)
			break;

		mdwc->rm_pulldown = val->intval;
		dbg_event(0xFF, "RM PuDwn", val->intval);
		dwc3_msm_remove_pulldown(mdwc, mdwc->rm_pulldown);
		break;
	
	case POWER_SUPPLY_PROP_PRESENT:
		dev_dbg(mdwc->dev, "%s: notify xceiv event\n", __func__);
		USB_INFO("check_vbus_in: %d -> %d\n", mdwc->vbus_active, val->intval);
		htc_vbus_backup = val->intval;
		vbus = val->intval;
		if (mdwc->otg_xceiv && !mdwc->ext_inuse && !mdwc->in_restart) {
			if (mdwc->ext_xceiv.bsv == val->intval)
				break;

			mdwc->ext_xceiv.bsv = val->intval;
			dbg_event(0xFF, "Q RW (vbus)", val->intval);
			queue_delayed_work(system_nrt_wq,
							&mdwc->resume_work, 12);
		}

		if (mdwc->vbus_active == 1 && val->intval == 0)
			cancel_delayed_work(&dwc->dotg->unknown_charger_notify_work);
		else if (mdwc->vbus_active == 0 && val->intval == 1) {
			if (mdwc->chg_state != USB_CHG_STATE_DETECTED
					&& (mdwc->charger.chg_type == DWC3_INVALID_CHARGER
						|| mdwc->charger.chg_type == DWC3_SDP_CHARGER)) {
				cancel_delayed_work(&dwc->dotg->unknown_charger_notify_work);
				queue_delayed_work(system_nrt_wq, &dwc->dotg->unknown_charger_notify_work, HZ * 10);
			}
		}
		mdwc->vbus_active = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		USB_INFO("%s: online %d -> %d\n", __func__, mdwc->online, val->intval);
		mdwc->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		mdwc->voltage_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		mdwc->current_max = val->intval;
		break;
<<<<<<< HEAD
=======
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		mdwc->typec_current_max = val->intval;

		dbg_event(0xFF, "TYPE C CURMAX)", val->intval);
		
		if (mdwc->chg_type != DWC3_INVALID_CHARGER) {
			dev_dbg(mdwc->dev, "update type-c charger\n");
			dwc3_msm_gadget_vbus_draw(mdwc,
						mdwc->bc1p2_current_max);
		}
		break;
>>>>>>> 0e91d2a... Nougat
	case POWER_SUPPLY_PROP_TYPE:
		psy->type = val->intval;

		switch (psy->type) {
		case POWER_SUPPLY_TYPE_USB:
			mdwc->charger.chg_type = DWC3_SDP_CHARGER;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
			mdwc->charger.chg_type = DWC3_DCP_CHARGER;
			break;
		case POWER_SUPPLY_TYPE_USB_HVDCP:
			mdwc->charger.chg_type = DWC3_DCP_CHARGER;
			usb_phy_set_power(phy, hvdcp_max_current);
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			
			
			mdwc->charger.chg_type = DWC3_SDP_CHARGER;
			break;
		case POWER_SUPPLY_TYPE_USB_ACA:
			mdwc->charger.chg_type = DWC3_PROPRIETARY_CHARGER;
			break;
		default:
			mdwc->charger.chg_type = DWC3_INVALID_CHARGER;
			break;
		}

		if (mdwc->charger.chg_type != DWC3_INVALID_CHARGER) {
			cancel_delayed_work(&dwc->dotg->unknown_charger_notify_work); 
			mdwc->chg_state = USB_CHG_STATE_DETECTED;
		}

		USB_INFO("%s : charger type: %s\n", __func__, chg_to_string(mdwc->charger.chg_type));
		dev_dbg(mdwc->dev, "%s: charger type: %s\n", __func__,
				chg_to_string(mdwc->charger.chg_type));

		break;
	case POWER_SUPPLY_PROP_HEALTH:
		mdwc->health_status = val->intval;
		break;
	default:
		return -EINVAL;
	}

	power_supply_changed(&mdwc->usb_psy);
	return 0;
}

static void dwc3_msm_external_power_changed(struct power_supply *psy)
{
	struct dwc3_msm *mdwc = container_of(psy, struct dwc3_msm, usb_psy);
	union power_supply_propval ret = {0,};

	if (!mdwc->ext_vbus_psy)
		mdwc->ext_vbus_psy = power_supply_get_by_name("ext-vbus");

	if (!mdwc->ext_vbus_psy) {
		pr_err("%s: Unable to get ext_vbus power_supply\n", __func__);
		return;
	}

	mdwc->ext_vbus_psy->get_property(mdwc->ext_vbus_psy,
					POWER_SUPPLY_PROP_ONLINE, &ret);
	if (ret.intval) {
		dwc3_start_chg_det(&mdwc->charger, false);
		mdwc->ext_vbus_psy->get_property(mdwc->ext_vbus_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		power_supply_set_current_limit(&mdwc->usb_psy, ret.intval);
	}

	power_supply_set_online(&mdwc->usb_psy, ret.intval);
	power_supply_changed(&mdwc->usb_psy);
}

static int
dwc3_msm_property_is_writeable(struct power_supply *psy,
				enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}


static char *dwc3_msm_pm_power_supplied_to[] = {
	"battery",
	"bms",
};

static enum power_supply_property dwc3_msm_pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
};

static void dwc3_init_adc_work(struct work_struct *w);

static void dwc3_ext_notify_online(void *ctx, int on)
{
	struct dwc3_msm *mdwc = ctx;
	bool notify_otg = false;

	if (!mdwc) {
		pr_err("%s: DWC3 driver already removed\n", __func__);
		return;
	}

	dev_dbg(mdwc->dev, "notify %s%s\n", on ? "" : "dis", "connected");

	if (!mdwc->ext_vbus_psy)
		mdwc->ext_vbus_psy = power_supply_get_by_name("ext-vbus");

	mdwc->ext_inuse = on;
	if (on) {
		
		mdwc->ext_xceiv.bsv = false;
		notify_otg = true;
		dwc3_start_chg_det(&mdwc->charger, false);
	} else {
		
		if (mdwc->ext_xceiv.id != mdwc->id_state) {
			mdwc->ext_xceiv.id = mdwc->id_state;
			notify_otg = true;
		}

		mdwc->ext_xceiv.bsv = mdwc->vbus_active;
		notify_otg |= mdwc->vbus_active;
	}

	if (mdwc->ext_vbus_psy)
		power_supply_set_present(mdwc->ext_vbus_psy, on);

	if (notify_otg)
		queue_delayed_work(system_nrt_wq, &mdwc->resume_work, 0);
}

static void dwc3_id_work(struct work_struct *w)
{
	struct dwc3_msm *mdwc = container_of(w, struct dwc3_msm, id_work);
	int ret;

	
	if (!mdwc->ext_inuse && usb_ext) {
		if (mdwc->pmic_id_irq)
			disable_irq(mdwc->pmic_id_irq);

		ret = usb_ext->notify(usb_ext->ctxt, mdwc->id_state,
				      dwc3_ext_notify_online, mdwc);
		dev_dbg(mdwc->dev, "%s: external handler returned %d\n",
			__func__, ret);

		if (mdwc->pmic_id_irq) {
			unsigned long flags;
			local_irq_save(flags);
			
			mdwc->id_state = !!irq_read_line(mdwc->pmic_id_irq);
			local_irq_restore(flags);
			enable_irq(mdwc->pmic_id_irq);
		}

		mdwc->ext_inuse = (ret == 0);
	}
	if (mdwc->ext_inuse || mdwc->mhl_online) {
		
		mdwc->ext_xceiv.id = DWC3_ID_FLOAT;
		mdwc->ext_xceiv.bsv = false;
	} else {
		
		mdwc->ext_xceiv.id = mdwc->id_state;
	}

	dbg_event(0xFF, "RW (id)", 0);
	
	dwc3_resume_work(&mdwc->resume_work.work);
}

static irqreturn_t dwc3_pmic_id_irq(int irq, void *data)
{
	struct dwc3_msm *mdwc = data;
	enum dwc3_id_state id;

	
	id = !!irq_read_line(irq);
	if (mdwc->id_state != id) {
		mdwc->id_state = id;
		queue_work(system_nrt_wq, &mdwc->id_work);
	}

	return IRQ_HANDLED;
}

static int dwc3_cpu_notifier_cb(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	uint32_t cpu = (uintptr_t)hcpu;
	struct dwc3_msm *mdwc =
			container_of(nfb, struct dwc3_msm, dwc3_cpu_notifier);

	if (cpu == cpu_to_affin && action == CPU_ONLINE) {
		pr_debug("%s: cpu online:%u irq:%d\n", __func__,
				cpu_to_affin, mdwc->irq_to_affin);
		irq_set_affinity(mdwc->irq_to_affin, get_cpu_mask(cpu));
	}

	return NOTIFY_OK;
}

static void dwc3_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct dwc3_msm *mdwc = ctx;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("%s: invalid notification %d\n", __func__, state);
		return;
	}

	dev_dbg(mdwc->dev, "%s: state = %s\n", __func__,
			state == ADC_TM_HIGH_STATE ? "high" : "low");

	
	if (state == ADC_TM_HIGH_STATE) {
		mdwc->id_state = DWC3_ID_FLOAT;
		mdwc->adc_param.state_request = ADC_TM_LOW_THR_ENABLE;
	} else {
		mdwc->id_state = DWC3_ID_GROUND;
		mdwc->adc_param.state_request = ADC_TM_HIGH_THR_ENABLE;
	}

	dwc3_id_work(&mdwc->id_work);

	
	qpnp_adc_tm_usbid_configure(mdwc->adc_tm_dev, &mdwc->adc_param);
}

static void dwc3_init_adc_work(struct work_struct *w)
{
	struct dwc3_msm *mdwc = container_of(w, struct dwc3_msm,
							init_adc_work.work);
	int ret;

	mdwc->adc_tm_dev = qpnp_get_adc_tm(mdwc->dev, "dwc_usb3-adc_tm");
	if (IS_ERR(mdwc->adc_tm_dev)) {
		if (PTR_ERR(mdwc->adc_tm_dev) == -EPROBE_DEFER)
			queue_delayed_work(system_nrt_wq, to_delayed_work(w),
					msecs_to_jiffies(100));
		else
			mdwc->adc_tm_dev = NULL;

		return;
	}

	mdwc->adc_param.low_thr = adc_low_threshold;
	mdwc->adc_param.high_thr = adc_high_threshold;
	mdwc->adc_param.timer_interval = adc_meas_interval;
	mdwc->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	mdwc->adc_param.btm_ctx = mdwc;
	mdwc->adc_param.threshold_notification = dwc3_adc_notification;

	ret = qpnp_adc_tm_usbid_configure(mdwc->adc_tm_dev, &mdwc->adc_param);
	if (ret) {
		dev_err(mdwc->dev, "%s: request ADC error %d\n", __func__, ret);
		return;
	}

<<<<<<< HEAD
	mdwc->id_adc_detect = true;
}
=======
	if (!of_property_read_u32(mdwc->dev->of_node, "qcom,core-clk-rate",
				(u32 *)&mdwc->core_clk_rate)) {
		mdwc->core_clk_rate = clk_round_rate(mdwc->core_clk,
							mdwc->core_clk_rate);
	} else {
		mdwc->core_clk_rate = clk_round_rate(mdwc->core_clk, LONG_MAX);
	}

	if (IS_ERR_VALUE(mdwc->core_clk_rate)) {
		dev_err(mdwc->dev, "fail to get core clk max freq.\n");
	} else {
		dev_dbg(mdwc->dev, "USB core frequency = %ld\n",
							mdwc->core_clk_rate);
		ret = clk_set_rate(mdwc->core_clk, mdwc->core_clk_rate);
		if (ret)
			dev_err(mdwc->dev, "fail to set core_clk freq:%d\n",
									ret);
	}
>>>>>>> 0e91d2a... Nougat

static ssize_t adc_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct dwc3_msm *mdwc = dev_get_drvdata(dev);

	if (!mdwc)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "%s\n", mdwc->id_adc_detect ?
						"enabled" : "disabled");
}

static ssize_t adc_enable_store(struct device *dev,
		struct device_attribute *attr, const char
		*buf, size_t size)
{
	struct dwc3_msm *mdwc = dev_get_drvdata(dev);

	if (!mdwc)
		return -EINVAL;

	if (!strnicmp(buf, "enable", 6)) {
		if (!mdwc->id_adc_detect)
			dwc3_init_adc_work(&mdwc->init_adc_work.work);
		return size;
	} else if (!strnicmp(buf, "disable", 7)) {
		qpnp_adc_tm_usbid_end(mdwc->adc_tm_dev);
		mdwc->id_adc_detect = false;
		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(adc_enable, S_IRUGO | S_IWUSR, adc_enable_show,
		adc_enable_store);

void dwc3_otg_set_id_state(int id, int mhl)
{
	struct dwc3_msm *dwc3_msm = context;

	if (id == 0 && mhl == 0) {
		printk("[USB] PMIC: ID set\n");
		dwc3_msm->id_state = DWC3_ID_GROUND;
		dwc3_msm->hs_phy->is_in_host = true;
		htc_id_backup = DWC3_ID_GROUND;
	} else if (id == 1) {
		printk("[USB] PMIC: ID clear\n");
		dwc3_msm->id_state = DWC3_ID_FLOAT;
		dwc3_msm->hs_phy->is_in_host = false;
		htc_id_backup = DWC3_ID_FLOAT;
		dwc3_msm->mhl_online = 0;
	} else if (id == 0 && mhl == 1) {
		printk("[USB] PMIC: MHL case\n");
		dwc3_msm->mhl_online = 1;
	}

	wake_lock_timeout(&dwc3_msm->cable_detect_wlock, 3*HZ);
	queue_work(system_nrt_wq, &dwc3_msm->id_work);
}

static void usb_host_mhl_cable_detect(enum usb_host_mhl_type cable_in)
{
	if (cable_in == CABLE_TYPE_HOST)
		dwc3_otg_set_id_state(0, 0);
	else if (cable_in == CABLE_TYPE_MHL)
		dwc3_otg_set_id_state(0, 1);
	else if (cable_in == CABLE_TYPE_NONE)
		dwc3_otg_set_id_state(1, 0);
}

static struct t_usb_host_mhl_status_notifier usb_host_mhl_status_notifier = {
	.name = "usb_host_mhl",
	.func = usb_host_mhl_cable_detect,
};

static int dwc3_msm_ext_chg_open(struct inode *inode, struct file *file)
{
	struct dwc3_msm *mdwc =
		container_of(inode->i_cdev, struct dwc3_msm, ext_chg_cdev);

	pr_debug("dwc3-msm ext chg open\n");
	file->private_data = mdwc;
	mdwc->ext_chg_opened = true;

	return 0;
}

static long
dwc3_msm_ext_chg_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dwc3_msm *mdwc = file->private_data;
	struct msm_usb_chg_info info = {0};
	int ret = 0, val;

	switch (cmd) {
	case MSM_USB_EXT_CHG_INFO:
		info.chg_block_type = USB_CHG_BLOCK_QSCRATCH;
		info.page_offset = (mdwc->io_res->start +
				QSCRATCH_REG_OFFSET) & ~PAGE_MASK;
		info.length = PAGE_SIZE;

		if (copy_to_user((void __user *)arg, &info, sizeof(info))) {
			pr_err("%s: copy to user failed\n\n", __func__);
			ret = -EFAULT;
		}
		break;
	case MSM_USB_EXT_CHG_BLOCK_LPM:
		if (get_user(val, (int __user *)arg)) {
			pr_err("%s: get_user failed\n\n", __func__);
			ret = -EFAULT;
			break;
		}
		pr_debug("%s: LPM block request %d\n", __func__, val);
		if (val) { 
			if (mdwc->charger.chg_type == DWC3_DCP_CHARGER) {
				dbg_event(0xFF, "CHGioct gsyn", 0);
				pm_runtime_get_sync(mdwc->dev);
			} else {
				mdwc->ext_chg_active = false;
				complete(&mdwc->ext_chg_wait);
				ret = -ENODEV;
			}
		} else {
			mdwc->ext_chg_active = false;
			complete(&mdwc->ext_chg_wait);
			dbg_event(0xFF, "CHGioct put", 0);
			pm_runtime_put(mdwc->dev);
		}
		break;
	case MSM_USB_EXT_CHG_VOLTAGE_INFO:
		if (get_user(val, (int __user *)arg)) {
			pr_err("%s: get_user failed\n\n", __func__);
			ret = -EFAULT;
			break;
		}

		if (val == USB_REQUEST_5V)
			pr_debug("%s:voting 5V voltage request\n", __func__);
		else if (val == USB_REQUEST_9V)
			pr_debug("%s:voting 9V voltage request\n", __func__);
		break;
	case MSM_USB_EXT_CHG_RESULT:
		if (get_user(val, (int __user *)arg)) {
			pr_err("%s: get_user failed\n\n", __func__);
			ret = -EFAULT;
			break;
		}

		if (!val)
			pr_debug("%s:voltage request successful\n", __func__);
		else
			pr_debug("%s:voltage request failed\n", __func__);
		break;
	case MSM_USB_EXT_CHG_TYPE:
		if (get_user(val, (int __user *)arg)) {
			pr_err("%s: get_user failed\n\n", __func__);
			ret = -EFAULT;
			break;
		}

		if (val)
			pr_debug("%s:charger is external charger\n", __func__);
		else
			pr_debug("%s:charger is not ext charger\n", __func__);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int dwc3_msm_ext_chg_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct dwc3_msm *mdwc = file->private_data;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	int ret;

	if (vma->vm_pgoff != 0 || vsize > PAGE_SIZE)
		return -EINVAL;

	vma->vm_pgoff = __phys_to_pfn(mdwc->io_res->start +
				QSCRATCH_REG_OFFSET);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = io_remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				 vsize, vma->vm_page_prot);
	if (ret < 0)
		pr_err("%s: failed with return val %d\n", __func__, ret);

	return ret;
}

static int dwc3_msm_ext_chg_release(struct inode *inode, struct file *file)
{
	struct dwc3_msm *mdwc = file->private_data;

	pr_debug("dwc3-msm ext chg release\n");

	mdwc->ext_chg_opened = false;

	return 0;
}

static const struct file_operations dwc3_msm_ext_chg_fops = {
	.owner = THIS_MODULE,
	.open = dwc3_msm_ext_chg_open,
	.unlocked_ioctl = dwc3_msm_ext_chg_ioctl,
	.mmap = dwc3_msm_ext_chg_mmap,
	.release = dwc3_msm_ext_chg_release,
};

static int dwc3_msm_setup_cdev(struct dwc3_msm *mdwc)
{
	int ret;

	ret = alloc_chrdev_region(&mdwc->ext_chg_dev, 0, 1, "usb_ext_chg");
	if (ret < 0) {
		pr_err("Fail to allocate usb ext char dev region\n");
		return ret;
	}
	mdwc->ext_chg_class = class_create(THIS_MODULE, "dwc_ext_chg");
	if (ret < 0) {
		pr_err("Fail to create usb ext chg class\n");
		goto unreg_chrdev;
	}
	cdev_init(&mdwc->ext_chg_cdev, &dwc3_msm_ext_chg_fops);
	mdwc->ext_chg_cdev.owner = THIS_MODULE;

	ret = cdev_add(&mdwc->ext_chg_cdev, mdwc->ext_chg_dev, 1);
	if (ret < 0) {
		pr_err("Fail to add usb ext chg cdev\n");
		goto destroy_class;
	}
	mdwc->ext_chg_device = device_create(mdwc->ext_chg_class,
					NULL, mdwc->ext_chg_dev, NULL,
					"usb_ext_chg");
	if (IS_ERR(mdwc->ext_chg_device)) {
		pr_err("Fail to create usb ext chg device\n");
		ret = PTR_ERR(mdwc->ext_chg_device);
		mdwc->ext_chg_device = NULL;
		goto del_cdev;
	}

	pr_debug("dwc3 msm ext chg cdev setup success\n");
	return 0;

del_cdev:
	cdev_del(&mdwc->ext_chg_cdev);
destroy_class:
	class_destroy(mdwc->ext_chg_class);
unreg_chrdev:
	unregister_chrdev_region(mdwc->ext_chg_dev, 1);

	return ret;
}

static int dwc3_msm_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node, *dwc3_node;
	struct device	*dev = &pdev->dev;
	struct dwc3_msm *mdwc;
	struct dwc3	*dwc;
	struct resource *res;
	void __iomem *tcsr;
	unsigned long flags;
	u32 tmp;
	bool host_mode;
	int ret = 0;

	htc_vbus_backup = 0;
	htc_id_backup = 1;
	mdwc = devm_kzalloc(&pdev->dev, sizeof(*mdwc), GFP_KERNEL);
	if (!mdwc) {
		dev_err(&pdev->dev, "not enough memory\n");
		return -ENOMEM;
	}

	if (!dev->dma_mask)
		dev->dma_mask = &dev->coherent_dma_mask;
	if (!dev->coherent_dma_mask)
		dev->coherent_dma_mask = DMA_BIT_MASK(64);

	platform_set_drvdata(pdev, mdwc);
	context = mdwc;
	mdwc->dev = &pdev->dev;

	wake_lock_init(&mdwc->cable_detect_wlock, WAKE_LOCK_SUSPEND, "msm_usb_cable");
	INIT_LIST_HEAD(&mdwc->req_complete_list);
	INIT_DELAYED_WORK(&mdwc->chg_work, dwc3_chg_detect_work);
	INIT_DELAYED_WORK(&mdwc->resume_work, dwc3_resume_work);
	INIT_WORK(&mdwc->restart_usb_work, dwc3_restart_usb_work);
<<<<<<< HEAD
	INIT_WORK(&mdwc->id_work, dwc3_id_work);
	INIT_DELAYED_WORK(&mdwc->init_adc_work, dwc3_init_adc_work);
	init_completion(&mdwc->ext_chg_wait);
=======
	INIT_WORK(&mdwc->bus_vote_w, dwc3_msm_bus_vote_w);
	INIT_WORK(&mdwc->disable_work, usb_disable_work); 	
	INIT_DELAYED_WORK(&mdwc->sm_work, dwc3_otg_sm_work);
#ifdef CONFIG_ANALOGIX_OHIO
	INIT_DELAYED_WORK(&mdwc->vbus_notify_work, dwc3_notify_vbus_work);
#endif
>>>>>>> 0e91d2a... Nougat

	mdwc->vdda33 = devm_regulator_get(dev, "vdda33");
	if (IS_ERR(mdwc->vdda33)) {
		dev_err(&pdev->dev, "unable to get vdda33 supply\n");
		return PTR_ERR(mdwc->vdda33);
	}

	ret = dwc3_msm_config_gdsc(mdwc, 1);
	if (ret) {
		dev_err(&pdev->dev, "unable to configure usb3 gdsc\n");
		goto destroy_wlock;
	}

	mdwc->xo_clk = clk_get(&pdev->dev, "xo");
	if (IS_ERR(mdwc->xo_clk)) {
		dev_err(&pdev->dev, "%s unable to get TCXO buffer handle\n",
								__func__);
		ret = PTR_ERR(mdwc->xo_clk);
		goto disable_dwc3_gdsc;
	}

	clk_set_rate(mdwc->xo_clk, 19200000);
	ret = clk_prepare_enable(mdwc->xo_clk);
	if (ret) {
		dev_err(&pdev->dev, "%s failed to vote for TCXO buffer%d\n",
						__func__, ret);
		goto put_xo;
	}

	mdwc->core_clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(mdwc->core_clk)) {
		dev_err(&pdev->dev, "failed to get core_clk\n");
		ret = PTR_ERR(mdwc->core_clk);
		goto disable_xo;
	}
	clk_set_rate(mdwc->core_clk, 125000000);
	clk_prepare_enable(mdwc->core_clk);

	mdwc->iface_clk = devm_clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(mdwc->iface_clk)) {
		dev_err(&pdev->dev, "failed to get iface_clk\n");
		ret = PTR_ERR(mdwc->iface_clk);
		goto disable_core_clk;
	}
	clk_set_rate(mdwc->iface_clk, 125000000);
	clk_prepare_enable(mdwc->iface_clk);

	mdwc->sleep_clk = devm_clk_get(&pdev->dev, "sleep_clk");
	if (IS_ERR(mdwc->sleep_clk)) {
		dev_err(&pdev->dev, "failed to get sleep_clk\n");
		ret = PTR_ERR(mdwc->sleep_clk);
		goto disable_iface_clk;
	}
	clk_set_rate(mdwc->sleep_clk, 32000);
	clk_prepare_enable(mdwc->sleep_clk);

	ret = of_property_read_u32(node, "qcom,utmi-clk-rate",
				   (u32 *)&mdwc->utmi_clk_rate);
	if (ret)
		mdwc->utmi_clk_rate = 19200000;

	mdwc->utmi_clk = devm_clk_get(&pdev->dev, "utmi_clk");
	if (IS_ERR(mdwc->utmi_clk)) {
		dev_err(&pdev->dev, "failed to get utmi_clk\n");
		ret = PTR_ERR(mdwc->utmi_clk);
		goto disable_sleep_clk;
	}

<<<<<<< HEAD
	if (mdwc->utmi_clk_rate == 24000000) {
		mdwc->utmi_clk_src = devm_clk_get(&pdev->dev, "utmi_clk_src");
		if (IS_ERR(mdwc->utmi_clk_src)) {
			dev_err(&pdev->dev, "failed to get utmi_clk_src\n");
			ret = PTR_ERR(mdwc->utmi_clk_src);
			goto disable_sleep_clk;
		}
		clk_set_rate(mdwc->utmi_clk_src, 48000000);
		
		clk_set_rate(mdwc->utmi_clk, 1);
	} else {
		clk_set_rate(mdwc->utmi_clk, mdwc->utmi_clk_rate);
	}
	clk_prepare_enable(mdwc->utmi_clk);

	mdwc->ref_clk = devm_clk_get(&pdev->dev, "ref_clk");
	if (IS_ERR(mdwc->ref_clk)) {
		dev_err(&pdev->dev, "failed to get ref_clk\n");
		ret = PTR_ERR(mdwc->ref_clk);
		goto disable_utmi_clk;
	}
	clk_set_rate(mdwc->ref_clk, 19200000);
	clk_prepare_enable(mdwc->ref_clk);

	mdwc->id_state = mdwc->ext_xceiv.id = DWC3_ID_FLOAT;
	mdwc->charger.charging_disabled = of_property_read_bool(node,
=======
	mdwc->id_state = DWC3_ID_FLOAT;
	set_bit(ID, &mdwc->inputs);

	mdwc->charging_disabled = of_property_read_bool(node,
>>>>>>> 0e91d2a... Nougat
				"qcom,charging-disabled");

	mdwc->charger.skip_chg_detect = of_property_read_bool(node,
				"qcom,skip-charger-detection");

	mdwc->suspend_resume_no_support = of_property_read_bool(node,
				"qcom,no-suspend-resume");

	mdwc->power_collapse_por = of_property_read_bool(node,
		"qcom,por-after-power-collapse");

	mdwc->power_collapse = of_property_read_bool(node,
		"qcom,power-collapse-on-cable-disconnect");

	dev_dbg(&pdev->dev, "power collapse=%d, POR=%d\n",
		mdwc->power_collapse, mdwc->power_collapse_por);

	ret = of_property_read_u32(node, "qcom,lpm-to-suspend-delay-ms",
				&mdwc->lpm_to_suspend_delay);
	if (ret) {
		dev_dbg(&pdev->dev, "setting lpm_to_suspend_delay to zero.\n");
		mdwc->lpm_to_suspend_delay = 0;
	}

	if (mdwc->power_collapse) {
		mdwc->phy_com_reset = devm_clk_get(&pdev->dev, "phy_com_reset");
		if (IS_ERR(mdwc->phy_com_reset)) {
			dev_dbg(&pdev->dev,
				"%s unable to get phy_com_reset clock, ret=%ld\n",
				__func__, PTR_ERR(mdwc->phy_com_reset));
			mdwc->phy_com_reset = NULL;
		}
	}

	mdwc->hs_phy_irq = platform_get_irq_byname(pdev, "hs_phy_irq");
	if (mdwc->hs_phy_irq < 0) {
		dev_err(&pdev->dev, "pget_irq for hs_phy_irq failed\n");
		goto disable_ref_clk;
	} else {
		irq_set_status_flags(mdwc->hs_phy_irq, IRQ_NOAUTOEN);
		ret = devm_request_irq(&pdev->dev, mdwc->hs_phy_irq,
				msm_dwc3_hs_phy_irq, IRQF_TRIGGER_RISING,
			       "msm_hs_phy_irq", mdwc);
		if (ret) {
			dev_err(&pdev->dev, "irqreq HSPHYINT failed\n");
			goto disable_ref_clk;
		}
	}

	mdwc->pwr_event_irq = platform_get_irq_byname(pdev, "pwr_event_irq");
	if (mdwc->pwr_event_irq < 0) {
		dev_err(&pdev->dev, "pget_irq for pwr_event_irq failed\n");
		goto disable_ref_clk;
	} else {
		ret = devm_request_threaded_irq(&pdev->dev, mdwc->pwr_event_irq,
					msm_dwc3_pwr_irq,
					msm_dwc3_pwr_irq_thread,
					IRQF_TRIGGER_RISING,
					"msm_dwc3", mdwc);
		if (ret) {
			dev_err(&pdev->dev, "irqreq pwr_event_irq failed: %d\n",
					ret);
			goto disable_ref_clk;
		}
	}

	mdwc->pmic_id_irq = platform_get_irq_byname(pdev, "pmic_id_irq");
	if (mdwc->pmic_id_irq > 0) {
		
		if (of_get_property(pdev->dev.of_node, "qcom,misc-ref", NULL))
			
			ret = qpnp_misc_irqs_available(&pdev->dev);
		else
			ret = mdwc->pmic_id_irq;

		if (ret == -EPROBE_DEFER) {
			
			goto disable_ref_clk;
		} else if (ret == 0) {
			mdwc->pmic_id_irq = 0;
		} else {
			irq_set_status_flags(mdwc->pmic_id_irq,
					IRQ_NOAUTOEN);
			ret = devm_request_irq(&pdev->dev,
					       mdwc->pmic_id_irq,
					       dwc3_pmic_id_irq,
					       IRQF_TRIGGER_RISING |
					       IRQF_TRIGGER_FALLING,
					       "dwc3_msm_pmic_id",
					       mdwc);
			if (ret) {
				dev_err(&pdev->dev, "irqreq IDINT failed\n");
				goto disable_ref_clk;
			}
		}
	}

	if (mdwc->pmic_id_irq <= 0) {
		
		queue_work(system_nrt_wq, &mdwc->init_adc_work.work);
		device_create_file(&pdev->dev, &dev_attr_adc_enable);
		mdwc->pmic_id_irq = 0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_dbg(&pdev->dev, "missing TCSR memory resource\n");
	} else {
		tcsr = devm_ioremap_nocache(&pdev->dev, res->start,
			resource_size(res));
		if (!tcsr) {
			dev_dbg(&pdev->dev, "tcsr ioremap failed\n");
		} else {
			
			writel_relaxed(0x1, tcsr);
			mb();
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "missing memory base resource\n");
		ret = -ENODEV;
		goto disable_ref_clk;
	}

	mdwc->base = devm_ioremap_nocache(&pdev->dev, res->start,
		resource_size(res));
	if (!mdwc->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENODEV;
		goto disable_ref_clk;
	}

<<<<<<< HEAD
	mdwc->io_res = res; 
=======
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"ahb2phy_base");
	if (res) {
		mdwc->ahb2phy_base = devm_ioremap_nocache(&pdev->dev,
					res->start, resource_size(res));
		if (IS_ERR_OR_NULL(mdwc->ahb2phy_base)) {
			dev_err(dev, "couldn't find ahb2phy_base addr.\n");
			mdwc->ahb2phy_base = NULL;
		} else {
			dwc3_msm_config_gdsc(mdwc, 1);
			clk_prepare_enable(mdwc->iface_clk);
			clk_prepare_enable(mdwc->cfg_ahb_clk);
			
			val = readl_relaxed(mdwc->ahb2phy_base +
					PERIPH_SS_AHB2PHY_TOP_CFG);
			if (val != ONE_READ_WRITE_WAIT) {
				writel_relaxed(ONE_READ_WRITE_WAIT,
					mdwc->ahb2phy_base +
					PERIPH_SS_AHB2PHY_TOP_CFG);
				
				mb();
			}
			clk_disable_unprepare(mdwc->cfg_ahb_clk);
			clk_disable_unprepare(mdwc->iface_clk);
			dwc3_msm_config_gdsc(mdwc, 0);
		}
	}
>>>>>>> 0e91d2a... Nougat

	if (of_get_property(pdev->dev.of_node, "qcom,usb-dbm", NULL)) {
		mdwc->dbm = usb_get_dbm_by_phandle(&pdev->dev, "qcom,usb-dbm");
		if (IS_ERR(mdwc->dbm)) {
			dev_err(&pdev->dev, "unable to get dbm device\n");
			ret = -EPROBE_DEFER;
			goto disable_ref_clk;
		}
		if (dbm_l1_lpm_interrupt(mdwc->dbm)) {
			if (!mdwc->pwr_event_irq) {
				dev_err(&pdev->dev,
					"need pwr_event_irq exiting L1\n");
				ret = -EINVAL;
				goto disable_ref_clk;
			}
		}
	}

	ret = of_property_read_u32(node, "qcom,restore-sec-cfg-for-scm-dev-id",
					&mdwc->scm_dev_id);
	if (ret)
		dev_dbg(&pdev->dev, "unable to read scm device id (%d)\n", ret);

	if (of_property_read_u32(node, "qcom,dwc-usb3-msm-tx-fifo-size",
				 &mdwc->tx_fifo_size))
		dev_err(&pdev->dev,
			"unable to read platform data tx fifo size\n");

	if (of_property_read_u32(node, "qcom,dwc-usb3-msm-qdss-tx-fifo-size",
				 &mdwc->qdss_tx_fifo_size))
		dev_err(&pdev->dev,
			"unable to read platform data qdss tx fifo size\n");

	dwc3_set_notifier(&dwc3_msm_notify_event);

	
	dwc3_node = of_get_next_available_child(node, NULL);
	if (!dwc3_node) {
		dev_err(&pdev->dev, "failed to find dwc3 child\n");
		goto put_psupply;
	}

	host_mode = of_property_read_bool(dwc3_node, "snps,host-only-mode");
	if (host_mode && of_get_property(pdev->dev.of_node, "vbus_dwc3-supply",
									NULL)) {
		mdwc->vbus_otg = devm_regulator_get(&pdev->dev, "vbus_dwc3");
		if (IS_ERR(mdwc->vbus_otg)) {
			dev_err(&pdev->dev, "Failed to get vbus regulator\n");
			ret = PTR_ERR(mdwc->vbus_otg);
			of_node_put(dwc3_node);
			goto put_psupply;
		}
		ret = regulator_enable(mdwc->vbus_otg);
		if (ret) {
			mdwc->vbus_otg = 0;
			dev_err(&pdev->dev, "Failed to enable vbus_otg\n");
			of_node_put(dwc3_node);
			goto put_psupply;
		}
	}

	
	if (!host_mode) {
		mdwc->usb_psy.name = "usb";
		mdwc->usb_psy.type = POWER_SUPPLY_TYPE_USB;
		mdwc->usb_psy.supplied_to = dwc3_msm_pm_power_supplied_to;
		mdwc->usb_psy.num_supplicants = ARRAY_SIZE(
						dwc3_msm_pm_power_supplied_to);
		mdwc->usb_psy.properties = dwc3_msm_pm_power_props_usb;
		mdwc->usb_psy.num_properties =
					ARRAY_SIZE(dwc3_msm_pm_power_props_usb);
		mdwc->usb_psy.get_property = dwc3_msm_power_get_property_usb;
		mdwc->usb_psy.set_property = dwc3_msm_power_set_property_usb;
<<<<<<< HEAD
		mdwc->usb_psy.external_power_changed =
					dwc3_msm_external_power_changed;
		mdwc->usb_psy.property_is_writeable =
				dwc3_msm_property_is_writeable;
=======
		mdwc->usb_psy.property_is_writeable =
				dwc3_msm_property_is_writeable;

		ret = power_supply_register(&pdev->dev, &mdwc->usb_psy);
		if (ret < 0) {
			dev_err(&pdev->dev,
					"%s:power_supply_register usb failed\n",
						__func__);
			goto err;
		}
	}

	ret = of_platform_populate(node, NULL, NULL, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to add create dwc3 core\n");
		of_node_put(dwc3_node);
		goto put_psupply;
	}

	mdwc->dwc3 = of_find_device_by_node(dwc3_node);
	of_node_put(dwc3_node);
	if (!mdwc->dwc3) {
		dev_err(&pdev->dev, "failed to get dwc3 platform device\n");
		goto put_dwc3;
	}

	mdwc->hs_phy = devm_usb_get_phy_by_phandle(&mdwc->dwc3->dev,
							"usb-phy", 0);
	if (IS_ERR(mdwc->hs_phy)) {
		dev_err(&pdev->dev, "unable to get hsphy device\n");
		ret = PTR_ERR(mdwc->hs_phy);
		goto put_dwc3;
	}
	mdwc->ss_phy = devm_usb_get_phy_by_phandle(&mdwc->dwc3->dev,
							"usb-phy", 1);
	if (IS_ERR(mdwc->ss_phy)) {
		dev_err(&pdev->dev, "unable to get ssphy device\n");
		ret = PTR_ERR(mdwc->ss_phy);
		goto put_dwc3;
	}

	mdwc->bus_scale_table = msm_bus_cl_get_pdata(pdev);
	if (!mdwc->bus_scale_table) {
		dev_err(&pdev->dev, "bus scaling is disabled\n");
	} else {
		mdwc->bus_perf_client =
			msm_bus_scale_register_client(mdwc->bus_scale_table);
		ret = msm_bus_scale_client_update_request(
						mdwc->bus_perf_client, 1);
		if (ret)
			dev_err(&pdev->dev, "Failed to vote for bus scaling\n");
	}

	dwc = platform_get_drvdata(mdwc->dwc3);
	if (!dwc) {
		dev_err(&pdev->dev, "Failed to get dwc3 device\n");
		goto put_dwc3;
	}

	mdwc->irq_to_affin = platform_get_irq(mdwc->dwc3, 0);
	mdwc->dwc3_cpu_notifier.notifier_call = dwc3_cpu_notifier_cb;

	if (cpu_to_affin)
		register_cpu_notifier(&mdwc->dwc3_cpu_notifier);

	device_init_wakeup(mdwc->dev, 1);
	pm_stay_awake(mdwc->dev);

	if (of_property_read_bool(node, "qcom,disable-dev-mode-pm"))
		pm_runtime_get_noresume(mdwc->dev);

	
	if (mdwc->pmic_id_irq) {
		enable_irq(mdwc->pmic_id_irq);
		local_irq_save(flags);
		mdwc->id_state = !!irq_read_line(mdwc->pmic_id_irq);
		if (mdwc->id_state == DWC3_ID_GROUND)
			dwc3_ext_event_notify(mdwc);
		local_irq_restore(flags);
		enable_irq_wake(mdwc->pmic_id_irq);
	}

	dwc->notify_usb_disabled = dwc3_msm_notify_usb_disabled; 

	if (!dwc->is_drd && host_mode) {
		dev_dbg(&pdev->dev, "DWC3 in host only mode\n");
		mdwc->id_state = DWC3_ID_GROUND;
		dwc3_ext_event_notify(mdwc);
	}

	return 0;

put_dwc3:
	platform_device_put(mdwc->dwc3);
	if (mdwc->bus_perf_client)
		msm_bus_scale_unregister_client(mdwc->bus_perf_client);
put_psupply:
	if (mdwc->usb_psy.dev)
		power_supply_unregister(&mdwc->usb_psy);
err:
	return ret;
}

static int dwc3_msm_remove_children(struct device *dev, void *data)
{
	device_unregister(dev);
	return 0;
}

static int dwc3_msm_remove(struct platform_device *pdev)
{
	struct dwc3_msm	*mdwc = platform_get_drvdata(pdev);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	int ret_pm;

	if (cpu_to_affin)
		unregister_cpu_notifier(&mdwc->dwc3_cpu_notifier);

	ret_pm = pm_runtime_get_sync(mdwc->dev);
	dbg_event(0xFF, "Remov gsyn", ret_pm);
	if (ret_pm < 0) {
		dev_err(mdwc->dev,
			"pm_runtime_get_sync failed with %d\n", ret_pm);
		clk_prepare_enable(mdwc->utmi_clk);
		clk_prepare_enable(mdwc->core_clk);
		clk_prepare_enable(mdwc->iface_clk);
		clk_prepare_enable(mdwc->sleep_clk);
		if (mdwc->bus_aggr_clk)
			clk_prepare_enable(mdwc->bus_aggr_clk);
		clk_prepare_enable(mdwc->xo_clk);
	}

	cancel_delayed_work_sync(&mdwc->sm_work);
	cancel_work_sync(&mdwc->disable_work); 

	if (mdwc->usb_psy.dev)
		power_supply_unregister(&mdwc->usb_psy);
	if (mdwc->hs_phy)
		mdwc->hs_phy->flags &= ~PHY_HOST_MODE;

	dbg_event(0xFF, "Remov put", 0);
	platform_device_put(mdwc->dwc3);
	device_for_each_child(&pdev->dev, NULL, dwc3_msm_remove_children);

	pm_runtime_disable(mdwc->dev);
	pm_runtime_barrier(mdwc->dev);
	pm_runtime_put_sync(mdwc->dev);
	pm_runtime_set_suspended(mdwc->dev);
	device_wakeup_disable(mdwc->dev);

	if (mdwc->bus_perf_client)
		msm_bus_scale_unregister_client(mdwc->bus_perf_client);

	if (!IS_ERR_OR_NULL(mdwc->vbus_reg))
		regulator_disable(mdwc->vbus_reg);

	disable_irq(mdwc->hs_phy_irq);
	if (mdwc->ss_phy_irq)
		disable_irq(mdwc->ss_phy_irq);
	disable_irq(mdwc->pwr_event_irq);

	clk_disable_unprepare(mdwc->utmi_clk);
	clk_set_rate(mdwc->core_clk, 19200000);
	clk_disable_unprepare(mdwc->core_clk);
	clk_disable_unprepare(mdwc->iface_clk);
	clk_disable_unprepare(mdwc->sleep_clk);
	clk_disable_unprepare(mdwc->xo_clk);
	clk_put(mdwc->xo_clk);
	mdwc->xo_vote_for_charger = false;

	dwc3_msm_config_gdsc(mdwc, 0);

	return 0;
}

#define VBUS_REG_CHECK_DELAY	(msecs_to_jiffies(1000))

#ifdef CONFIG_ANALOGIX_OHIO
int dwc3_pd_vbus_ctrl(int on)
{
	struct dwc3_msm *mdwc = context;
	struct dwc3 *dwc = NULL;
	int ret = 0;

	if (IS_ERR_OR_NULL(mdwc))
		return -EFAULT;
	dwc = platform_get_drvdata(mdwc->dwc3);

	pr_debug("%s: on = %d\n", __func__, on);
	if (on == -1) { 
		mdwc->pd_vbus_change = 0;
		mdwc->power_role = UNKNOWN_POWER_ROLE;
		mdwc->data_role = UNKNOWN_DATA_ROLE;
	}
	else {
		mdwc->pd_vbus_change = 1;
		pr_info("%s: set pd vbus flag to 1\n", __func__);
	}

	if (!mdwc->vbus_reg) {
		mdwc->vbus_reg = devm_regulator_get_optional(mdwc->dev,
					"vbus_dwc3");
		if (IS_ERR(mdwc->vbus_reg) &&
				PTR_ERR(mdwc->vbus_reg) == -EPROBE_DEFER) {
			
			mdwc->vbus_reg = NULL;
			pr_err("%s: regulator not ready, rc = %d\n", __func__, -EPROBE_DEFER);
			return -EPROBE_DEFER;
		}
	}

	if (on == 1) {
		dev_dbg(mdwc->dev, "%s: turn on regulator\n", __func__);

		if (!IS_ERR(mdwc->vbus_reg)) {
			if (!regulator_is_enabled(mdwc->vbus_reg)) {
				ret = regulator_enable(mdwc->vbus_reg);
				pr_debug("%s: regulator_enable, ret = %d\n",__func__, ret);
				if (ret) {
					dev_err(dwc->dev, "unable to enable vbus_reg\n");
					pr_err("%s: unable to enable vbus_reg\n", __func__);
					return ret;
				}
			}
			else
				pr_info("%s: regulator already enabled\n", __func__);
		}
		else {
			pr_err("%s: ERR: vbus_reg, err_code = %ld\n", __func__, PTR_ERR(mdwc->vbus_reg));
			return -1;
		}
		mdwc->power_role = POWER_SOURCE;
		pr_debug("%s: turn on Vbus\n", __func__);
		if (ohio_get_data_value(3) == 2 ) {
			pr_info("during try_source, Vbus output should be ready 40ms later\n");
			pr_swap_rsp.done = 1;
			complete(&pr_swap_rsp);
		}
	}
	else { 
		dev_dbg(dwc->dev, "%s: turn off regulator\n", __func__);

		if (!IS_ERR(mdwc->vbus_reg) && regulator_is_enabled(mdwc->vbus_reg))
			ret = regulator_disable(mdwc->vbus_reg);
		if (ret) {
			dev_err(dwc->dev, "unable to disable vbus_reg\n");
			return ret;
		}
		if (on == 0)
			mdwc->power_role = POWER_SINK;
		pr_debug("%s: turn off Vbus\n", __func__);
	}
	return ret;
}

int dwc3_pd_drswap(int new_role)
{
	struct dwc3_msm *mdwc = context;
	int ret = 0;
	if (IS_ERR_OR_NULL(mdwc))
		return -EFAULT;

	pr_info("%s: 0:HOST 1:DEVICE; new_role = %d\n", __func__, new_role);

	if (new_role) { 
		set_bit(B_SESS_VLD, &mdwc->inputs);
		mdwc->chg_type = DWC3_SDP_CHARGER;
		mdwc->data_role = DATA_DEVICE;
	}
	else { 
		mdwc->data_role = DATA_HOST;
	}
	return ret;
}
EXPORT_SYMBOL(dwc3_pd_vbus_ctrl);
EXPORT_SYMBOL(dwc3_pd_drswap);
#endif

static int dwc3_otg_start_host(struct dwc3_msm *mdwc, int on)
{
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
	int ret = 0;

	if (!dwc->xhci)
		return -EINVAL;

	if (!mdwc->vbus_reg) {
		mdwc->vbus_reg = devm_regulator_get_optional(mdwc->dev,
					"vbus_dwc3");
		if (IS_ERR(mdwc->vbus_reg) &&
				PTR_ERR(mdwc->vbus_reg) == -EPROBE_DEFER) {
			
			mdwc->vbus_reg = NULL;
			return -EPROBE_DEFER;
		}
	}

	if (on) {
		dev_dbg(mdwc->dev, "%s: turn on host\n", __func__);

		pm_runtime_get_sync(mdwc->dev);
		dbg_event(0xFF, "StrtHost gync",
			atomic_read(&mdwc->dev->power.usage_count));
		mdwc->hs_phy->flags |= PHY_HOST_MODE;
		mdwc->ss_phy->flags |= PHY_HOST_MODE;
		usb_phy_notify_connect(mdwc->hs_phy, USB_SPEED_HIGH);
#ifndef CONFIG_ANALOGIX_OHIO
		if (!IS_ERR(mdwc->vbus_reg))
			ret = regulator_enable(mdwc->vbus_reg);
		if (ret) {
			dev_err(mdwc->dev, "unable to enable vbus_reg\n");
			mdwc->hs_phy->flags &= ~PHY_HOST_MODE;
			mdwc->ss_phy->flags &= ~PHY_HOST_MODE;
			pm_runtime_put_sync(mdwc->dev);
			dbg_event(0xFF, "vregerr psync",
				atomic_read(&mdwc->dev->power.usage_count));
			return ret;
		}
#endif

		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);

		mdwc->usbdev_nb.notifier_call = msm_dwc3_usbdev_notify;
		usb_register_atomic_notify(&mdwc->usbdev_nb);
		pm_runtime_init(&dwc->xhci->dev);
		ret = platform_device_add(dwc->xhci);
		if (ret) {
			dev_err(mdwc->dev,
				"%s: failed to add XHCI pdev ret=%d\n",
				__func__, ret);
			if (!IS_ERR(mdwc->vbus_reg))
				regulator_disable(mdwc->vbus_reg);
			mdwc->hs_phy->flags &= ~PHY_HOST_MODE;
			mdwc->ss_phy->flags &= ~PHY_HOST_MODE;
			pm_runtime_put_sync(mdwc->dev);
			dbg_event(0xFF, "pdeverr psync",
				atomic_read(&mdwc->dev->power.usage_count));
			return ret;
		}

		if (mdwc->disable_host_mode_pm)
			pm_runtime_disable(&dwc->xhci->dev);

		mdwc->in_host_mode = true;
		dwc3_gadget_usb3_phy_suspend(dwc, true);

		
		dbg_event(0xFF, "StrtHost psync",
			atomic_read(&mdwc->dev->power.usage_count));
		pm_runtime_mark_last_busy(mdwc->dev);
		pm_runtime_put_sync_autosuspend(mdwc->dev);
	} else {
		dev_dbg(mdwc->dev, "%s: turn off host\n", __func__);

		usb_unregister_atomic_notify(&mdwc->usbdev_nb);
#ifndef CONFIG_ANALOGIX_OHIO
		if (!IS_ERR(mdwc->vbus_reg))
			ret = regulator_disable(mdwc->vbus_reg);
		if (ret) {
			dev_err(mdwc->dev, "unable to disable vbus_reg\n");
			return ret;
		}
#endif

		pm_runtime_get_sync(mdwc->dev);
		dbg_event(0xFF, "StopHost gsync",
			atomic_read(&mdwc->dev->power.usage_count));
		usb_phy_notify_disconnect(mdwc->hs_phy, USB_SPEED_HIGH);
		mdwc->hs_phy->flags &= ~PHY_HOST_MODE;
		mdwc->ss_phy->flags &= ~PHY_HOST_MODE;
		platform_device_del(dwc->xhci);

		dwc3_msm_block_reset(mdwc, true);

		dwc3_gadget_usb3_phy_suspend(dwc, false);
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);

		mdwc->in_host_mode = false;

		
		dwc3_post_host_reset_core_init(dwc);
		pm_runtime_mark_last_busy(mdwc->dev);
		pm_runtime_put_sync_autosuspend(mdwc->dev);
		dbg_event(0xFF, "StopHost psync",
			atomic_read(&mdwc->dev->power.usage_count));
	}

	return 0;
}

static int dwc3_otg_start_peripheral(struct dwc3_msm *mdwc, int on)
{
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	pm_runtime_get_sync(mdwc->dev);
	dbg_event(0xFF, "StrtGdgt gsync",
		atomic_read(&mdwc->dev->power.usage_count));

	if (on) {
		dev_dbg(mdwc->dev, "%s: turn on gadget %s\n",
					__func__, dwc->gadget.name);

		usb_phy_notify_connect(mdwc->hs_phy, USB_SPEED_HIGH);
		usb_phy_notify_connect(mdwc->ss_phy, USB_SPEED_SUPER);

		dwc3_msm_block_reset(mdwc, false);

		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		usb_gadget_vbus_connect(&dwc->gadget);
	} else {
		dev_dbg(mdwc->dev, "%s: turn off gadget %s\n",
					__func__, dwc->gadget.name);
		usb_gadget_vbus_disconnect(&dwc->gadget);
		usb_phy_notify_disconnect(mdwc->hs_phy, USB_SPEED_HIGH);
		usb_phy_notify_disconnect(mdwc->ss_phy, USB_SPEED_SUPER);
		dwc3_gadget_usb3_phy_suspend(dwc, false);
	}

	pm_runtime_put_sync(mdwc->dev);
	dbg_event(0xFF, "StopGdgt psync",
		atomic_read(&mdwc->dev->power.usage_count));

	return 0;
}

static int dwc3_msm_gadget_vbus_draw(struct dwc3_msm *mdwc, unsigned mA)
{
	enum power_supply_type power_supply_type;

	if (mdwc->charging_disabled)
		return 0;

	if (mdwc->chg_type != DWC3_INVALID_CHARGER) {
		dev_dbg(mdwc->dev,
			"SKIP setting power supply type again,chg_type = %d\n",
			mdwc->chg_type);
		goto skip_psy_type;
	}

	dev_dbg(mdwc->dev, "Requested curr from USB = %u, max-type-c:%u\n",
					mA, mdwc->typec_current_max);

	if (mdwc->chg_type == DWC3_SDP_CHARGER)
		power_supply_type = POWER_SUPPLY_TYPE_USB;
	else if (mdwc->chg_type == DWC3_CDP_CHARGER)
		power_supply_type = POWER_SUPPLY_TYPE_USB_CDP;
	else if (mdwc->chg_type == DWC3_DCP_CHARGER ||
			mdwc->chg_type == DWC3_PROPRIETARY_CHARGER)
		power_supply_type = POWER_SUPPLY_TYPE_USB_DCP;
	else
		power_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;

	power_supply_set_supply_type(&mdwc->usb_psy, power_supply_type);

skip_psy_type:
>>>>>>> 0e91d2a... Nougat

		ret = power_supply_register(&pdev->dev, &mdwc->usb_psy);
		if (ret < 0) {
			dev_err(&pdev->dev,
					"%s:power_supply_register usb failed\n",
						__func__);
			goto disable_ref_clk;
		}
	}

	
	dwc3_msm_link_clk_reset(mdwc, 1);
	msleep(20);
	dwc3_msm_link_clk_reset(mdwc, 0);

	ret = of_platform_populate(node, NULL, NULL, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to add create dwc3 core\n");
		of_node_put(dwc3_node);
		goto disable_vbus;
	}

	mdwc->dwc3 = of_find_device_by_node(dwc3_node);
	of_node_put(dwc3_node);
	if (!mdwc->dwc3) {
		dev_err(&pdev->dev, "failed to get dwc3 platform device\n");
		goto put_dwc3;
	}

	mdwc->hs_phy = devm_usb_get_phy_by_phandle(&mdwc->dwc3->dev,
							"usb-phy", 0);
	if (IS_ERR(mdwc->hs_phy)) {
		dev_err(&pdev->dev, "unable to get hsphy device\n");
		ret = PTR_ERR(mdwc->hs_phy);
		goto put_dwc3;
	}

	mdwc->ss_phy = devm_usb_get_phy_by_phandle(&mdwc->dwc3->dev,
							"usb-phy", 1);
	if (IS_ERR(mdwc->ss_phy)) {
		dev_err(&pdev->dev, "unable to get ssphy device\n");
		ret = PTR_ERR(mdwc->ss_phy);
		goto put_dwc3;
	}

	mdwc->bus_scale_table = msm_bus_cl_get_pdata(pdev);
	if (!mdwc->bus_scale_table) {
		dev_err(&pdev->dev, "bus scaling is disabled\n");
	} else {
		mdwc->bus_perf_client =
			msm_bus_scale_register_client(mdwc->bus_scale_table);
		ret = msm_bus_scale_client_update_request(
						mdwc->bus_perf_client, 1);
		if (ret)
			dev_err(&pdev->dev, "Failed to vote for bus scaling\n");
	}

	dwc = platform_get_drvdata(mdwc->dwc3);
	if (!dwc) {
		dev_err(&pdev->dev, "Failed to get dwc3 device\n");
		goto put_dwc3;
	}

	dwc->vbus_active = of_property_read_bool(node, "qcom,vbus-present");

	if (dwc->dotg)
		mdwc->otg_xceiv = dwc->dotg->otg.phy;
	
	if (mdwc->otg_xceiv) {
		
		if (!mdwc->charger.skip_chg_detect) {
			mdwc->charger.start_detection = dwc3_start_chg_det;
			ret = dwc3_set_charger(mdwc->otg_xceiv->otg,
					&mdwc->charger);
			if (ret || !mdwc->charger.notify_detection_complete) {
				dev_err(&pdev->dev,
					"failed to register charger: %d\n",
					ret);
				goto put_dwc3;
			}
		}

		mdwc->ext_xceiv.ext_block_reset = dwc3_msm_block_reset;
		ret = dwc3_set_ext_xceiv(mdwc->otg_xceiv->otg,
						&mdwc->ext_xceiv);
		if (ret || !mdwc->ext_xceiv.notify_ext_events) {
			dev_err(&pdev->dev, "failed to register xceiver: %d\n",
									ret);
			goto put_dwc3;
		}
	} else if (host_mode) {
		dev_dbg(&pdev->dev, "No OTG, DWC3 running in host only mode\n");
		mdwc->scope = POWER_SUPPLY_SCOPE_SYSTEM;
		mdwc->hs_phy->flags |= PHY_HOST_MODE;
		mdwc->ss_phy->flags |= PHY_HOST_MODE;
	} else {
		dev_err(&pdev->dev, "DWC3 device-only mode not supported\n");
		ret = -ENODEV;
		goto put_dwc3;
	}

<<<<<<< HEAD
	if (mdwc->charger.start_detection) {
		ret = dwc3_msm_setup_cdev(mdwc);
		if (ret)
			dev_err(&pdev->dev, "Fail to setup dwc3 setup cdev\n");
=======
	return 0;

psy_error:
	dev_dbg(mdwc->dev, "power supply error when setting property\n");
	return -ENXIO;
}

static void dwc3_check_float_lines(struct dwc3_msm *mdwc)
{
	int dpdm;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(mdwc->dev, "%s: Check linestate\n", __func__);
	dwc3_msm_gadget_vbus_draw(mdwc, 0);

	
	dpdm = usb_phy_dpdm_with_idp_src(mdwc->hs_phy);
	if (dpdm == 0x2) {
		
		mdwc->chg_type = DWC3_PROPRIETARY_CHARGER;
		mdwc->otg_state = OTG_STATE_B_IDLE;
		pm_runtime_put_sync(mdwc->dev);
		dbg_event(0xFF, "FLT psync",
				atomic_read(&mdwc->dev->power.usage_count));
	} else if (dpdm) {
		dev_dbg(mdwc->dev, "%s:invalid linestate:%x\n", __func__, dpdm);
>>>>>>> 0e91d2a... Nougat
	}

<<<<<<< HEAD
	mdwc->irq_to_affin = platform_get_irq(mdwc->dwc3, 0);
	mdwc->dwc3_cpu_notifier.notifier_call = dwc3_cpu_notifier_cb;

	if (cpu_to_affin)
		register_cpu_notifier(&mdwc->dwc3_cpu_notifier);

	device_init_wakeup(mdwc->dev, 1);
	pm_stay_awake(mdwc->dev);
	dwc3_msm_debugfs_init(mdwc);

	pm_runtime_set_active(mdwc->dev);
	pm_runtime_enable(mdwc->dev);

	
	tmp = dwc3_msm_read_reg_field(mdwc->base,
			DWC3_GDBGLTSSM, DWC3_GDBGLTSSM_LINKSTATE_MASK);
	atomic_set(&mdwc->in_p3, tmp == DWC3_LINK_STATE_U3);
	dwc3_msm_write_reg_field(mdwc->base, PWR_EVNT_IRQ_MASK_REG,
				PWR_EVNT_POWERDOWN_IN_P3_MASK, 1);
=======
static void dwc3_initialize(struct dwc3_msm *mdwc)
{
	u32 tmp;
	int ret;
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
>>>>>>> 0e91d2a... Nougat

	enable_irq(mdwc->hs_phy_irq);

	
	if (mdwc->pmic_id_irq) {
		enable_irq(mdwc->pmic_id_irq);
		local_irq_save(flags);
		mdwc->id_state = !!irq_read_line(mdwc->pmic_id_irq);
		if (mdwc->id_state == DWC3_ID_GROUND)
			queue_work(system_nrt_wq, &mdwc->id_work);
		local_irq_restore(flags);
		enable_irq_wake(mdwc->pmic_id_irq);
	}
	usb_host_mhl_detect_register_notifier(&usb_host_mhl_status_notifier);
	msm_bam_set_usb_dev(mdwc->dev);

	mdwc->charger.usb_disable = 0;

	return 0;

put_dwc3:
	platform_device_put(mdwc->dwc3);
	if (mdwc->bus_perf_client)
		msm_bus_scale_unregister_client(mdwc->bus_perf_client);
disable_vbus:
	if (!IS_ERR_OR_NULL(mdwc->vbus_otg))
		regulator_disable(mdwc->vbus_otg);
put_psupply:
	if (mdwc->usb_psy.dev)
		power_supply_unregister(&mdwc->usb_psy);
disable_ref_clk:
	clk_disable_unprepare(mdwc->ref_clk);
disable_utmi_clk:
	clk_disable_unprepare(mdwc->utmi_clk);
disable_sleep_clk:
	clk_disable_unprepare(mdwc->sleep_clk);
disable_iface_clk:
	clk_disable_unprepare(mdwc->iface_clk);
disable_core_clk:
	clk_set_rate(mdwc->core_clk, 19200000);
	clk_disable_unprepare(mdwc->core_clk);
disable_xo:
	clk_disable_unprepare(mdwc->xo_clk);
put_xo:
	clk_put(mdwc->xo_clk);
disable_dwc3_gdsc:
	dwc3_msm_config_gdsc(mdwc, 0);
destroy_wlock:
	wake_lock_destroy(&mdwc->cable_detect_wlock);

	return ret;
}

static int dwc3_msm_remove_children(struct device *dev, void *data)
{
	device_unregister(dev);
	return 0;
}

static int dwc3_msm_remove(struct platform_device *pdev)
{
	struct dwc3_msm	*mdwc = platform_get_drvdata(pdev);
	int ret_pm;

	if (cpu_to_affin)
		unregister_cpu_notifier(&mdwc->dwc3_cpu_notifier);

	ret_pm = pm_runtime_get_sync(mdwc->dev);
	dbg_event(0xFF, "Remov gsyn", ret_pm);
	if (ret_pm < 0) {
		dev_err(mdwc->dev,
			"pm_runtime_get_sync failed with %d\n", ret_pm);
		clk_prepare_enable(mdwc->utmi_clk);
		clk_prepare_enable(mdwc->core_clk);
		clk_prepare_enable(mdwc->iface_clk);
		clk_prepare_enable(mdwc->sleep_clk);
		clk_prepare_enable(mdwc->ref_clk);
		clk_prepare_enable(mdwc->xo_clk);
	}

<<<<<<< HEAD
	if (mdwc->ext_chg_device) {
		device_destroy(mdwc->ext_chg_class, mdwc->ext_chg_dev);
		cdev_del(&mdwc->ext_chg_cdev);
		class_destroy(mdwc->ext_chg_class);
		unregister_chrdev_region(mdwc->ext_chg_dev, 1);
	}

	if (mdwc->id_adc_detect)
		qpnp_adc_tm_usbid_end(mdwc->adc_tm_dev);
	if (dwc3_debugfs_root)
		debugfs_remove_recursive(dwc3_debugfs_root);
	if (mdwc->otg_xceiv)
		dwc3_start_chg_det(&mdwc->charger, false);
	if (mdwc->usb_psy.dev)
		power_supply_unregister(&mdwc->usb_psy);
	if (mdwc->hs_phy)
		mdwc->hs_phy->flags &= ~PHY_HOST_MODE;
	platform_device_put(mdwc->dwc3);
	device_for_each_child(&pdev->dev, NULL, dwc3_msm_remove_children);
=======
	state = usb_otg_state_string(mdwc->otg_state);
	dev_dbg(mdwc->dev, "%s state\n", state);
	pr_info("%s: %s state\n", __func__, state);
	dbg_event(0xFF, state, 0);

	
	switch (mdwc->otg_state) {
	case OTG_STATE_UNDEFINED:
		if (!test_bit(ID, &mdwc->inputs)) {
			dbg_event(0xFF, "undef_host", 0);
			atomic_set(&dwc->in_lpm, 0);
			pm_runtime_set_active(mdwc->dev);
			pm_runtime_enable(mdwc->dev);
			pm_runtime_get_noresume(mdwc->dev);
			dwc3_initialize(mdwc);
			mdwc->otg_state = OTG_STATE_A_HOST;
			ret = dwc3_otg_start_host(mdwc, 1);
			pm_runtime_put_noidle(mdwc->dev);
			if (ret != -EPROBE_DEFER)
				return;
			mdwc->otg_state = OTG_STATE_A_IDLE;
			work = 1;
			break;
		}

		if (test_bit(B_SESS_VLD, &mdwc->inputs)) {
			dev_dbg(mdwc->dev, "b_sess_vld\n");
			dbg_event(0xFF, "undef_b_sess_vld", 0);
			switch (mdwc->chg_type) {
			case DWC3_DCP_CHARGER:
			case DWC3_PROPRIETARY_CHARGER:
				dev_dbg(mdwc->dev, "DCP charger\n");
				dwc3_msm_gadget_vbus_draw(mdwc,
						dcp_max_current);
				atomic_set(&dwc->in_lpm, 1);
				pm_relax(mdwc->dev);
				break;
			case DWC3_CDP_CHARGER:
			case DWC3_SDP_CHARGER:
				atomic_set(&dwc->in_lpm, 0);
				pm_runtime_set_active(mdwc->dev);
				pm_runtime_enable(mdwc->dev);
				pm_runtime_get_noresume(mdwc->dev);
				dwc3_initialize(mdwc);
				
				if (mdwc->detect_dpdm_floating) {
					dwc3_check_float_lines(mdwc);
					if (mdwc->chg_type != DWC3_SDP_CHARGER)
						break;
				}
				dwc3_otg_start_peripheral(mdwc, 1);
				mdwc->otg_state = OTG_STATE_B_PERIPHERAL;
				dbg_event(0xFF, "Undef SDP",
					atomic_read(
					&mdwc->dev->power.usage_count));
				break;
			default:
				WARN_ON(1);
				break;
			}
		}

		if (!test_bit(B_SESS_VLD, &mdwc->inputs)) {
			dbg_event(0xFF, "undef_!b_sess_vld", 0);
			atomic_set(&dwc->in_lpm, 0);
			pm_runtime_set_active(mdwc->dev);
			pm_runtime_enable(mdwc->dev);
			pm_runtime_get_noresume(mdwc->dev);
			dwc3_initialize(mdwc);
			pm_runtime_put_sync(mdwc->dev);
			dbg_event(0xFF, "Undef NoUSB",
				atomic_read(&mdwc->dev->power.usage_count));
			mdwc->otg_state = OTG_STATE_B_IDLE;
		}
		break;

	case OTG_STATE_B_IDLE:
		if (!test_bit(ID, &mdwc->inputs)) {
			dev_dbg(mdwc->dev, "!id\n");
			mdwc->otg_state = OTG_STATE_A_IDLE;
			work = 1;
			mdwc->chg_type = DWC3_INVALID_CHARGER;
		} else if (test_bit(B_SESS_VLD, &mdwc->inputs)) {
			dev_dbg(mdwc->dev, "b_sess_vld\n");
			switch (mdwc->chg_type) {
			case DWC3_DCP_CHARGER:
			case DWC3_PROPRIETARY_CHARGER:
				dev_dbg(mdwc->dev, "lpm, DCP charger\n");
				dwc3_msm_gadget_vbus_draw(mdwc,
						dcp_max_current);
				break;
			case DWC3_CDP_CHARGER:
				dwc3_msm_gadget_vbus_draw(mdwc,
						DWC3_IDEV_CHG_MAX);
				
			case DWC3_SDP_CHARGER:
				pm_runtime_get_sync(mdwc->dev);
				dbg_event(0xFF, "CHG gsync",
					atomic_read(
						&mdwc->dev->power.usage_count));
				
				if (mdwc->detect_dpdm_floating &&
				    mdwc->chg_type == DWC3_SDP_CHARGER) {
					dwc3_check_float_lines(mdwc);
					if (mdwc->chg_type != DWC3_SDP_CHARGER)
						break;
				}
				dwc3_otg_start_peripheral(mdwc, 1);
				mdwc->otg_state = OTG_STATE_B_PERIPHERAL;
				work = 1;
				break;
			
			default:
				break;
			}
		} else {
			mdwc->typec_current_max = 0;
			dwc3_msm_gadget_vbus_draw(mdwc, 0);
			dev_dbg(mdwc->dev, "No device, allowing suspend\n");
		}
		break;
>>>>>>> 0e91d2a... Nougat

	dbg_event(0xFF, "Remov put", 0);
	pm_runtime_disable(mdwc->dev);
	pm_runtime_barrier(mdwc->dev);
	pm_runtime_put_sync(mdwc->dev);
	pm_runtime_set_suspended(mdwc->dev);
	device_wakeup_disable(mdwc->dev);
	wake_lock_destroy(&mdwc->cable_detect_wlock);

	if (mdwc->bus_perf_client)
		msm_bus_scale_unregister_client(mdwc->bus_perf_client);

	if (!IS_ERR_OR_NULL(mdwc->vbus_otg))
		regulator_disable(mdwc->vbus_otg);

<<<<<<< HEAD
	if (mdwc->hs_phy_irq)
		disable_irq(mdwc->hs_phy_irq);
	if (mdwc->pwr_event_irq)
		disable_irq(mdwc->pwr_event_irq);
=======
	case OTG_STATE_A_HOST:
		if (test_bit(ID, &mdwc->inputs) || mdwc->hc_died) {
			dev_dbg(mdwc->dev, "id || hc_died\n");
			dwc3_otg_start_host(mdwc, 0);
			mdwc->otg_state = OTG_STATE_B_IDLE;
			mdwc->vbus_retry_count = 0;
			mdwc->hc_died = false;
			work = 1;
		} else {
			dev_dbg(mdwc->dev, "still in a_host state. Resuming root hub.\n");
			dbg_event(0xFF, "XHCIResume", 0);
			if (dwc)
				pm_runtime_resume(&dwc->xhci->dev);
		}
		break;
	default:
		dev_err(mdwc->dev, "%s: invalid otg-state\n", __func__);
>>>>>>> 0e91d2a... Nougat

	clk_disable_unprepare(mdwc->utmi_clk);
	clk_set_rate(mdwc->core_clk, 19200000);
	clk_disable_unprepare(mdwc->core_clk);
	clk_disable_unprepare(mdwc->iface_clk);
	clk_disable_unprepare(mdwc->sleep_clk);
	clk_disable_unprepare(mdwc->ref_clk);
	clk_disable_unprepare(mdwc->xo_clk);
	clk_put(mdwc->xo_clk);

	dwc3_msm_config_gdsc(mdwc, 0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dwc3_msm_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct dwc3_msm *mdwc = dev_get_drvdata(dev);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(dev, "dwc3-msm PM suspend\n");
	dbg_event(0xFF, "PM Sus", 0);

	flush_delayed_work(&mdwc->resume_work);
	if (!atomic_read(&dwc->in_lpm)) {
		dev_err(mdwc->dev, "Abort PM suspend!! (USB is outside LPM)\n");
		return -EBUSY;
	}

	ret = dwc3_msm_suspend(mdwc);
	if (!ret)
		atomic_set(&mdwc->pm_suspended, 1);

	return ret;
}

static int dwc3_msm_pm_resume(struct device *dev)
{
	int ret = 0;
	struct dwc3_msm *mdwc = dev_get_drvdata(dev);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(dev, "dwc3-msm PM resume\n");

	atomic_set(&mdwc->pm_suspended, 0);
	dbg_event(0xFF, "PM Res", mdwc->resume_pending);
	if (mdwc->resume_pending) {
		mdwc->resume_pending = false;

		ret = dwc3_msm_resume(mdwc);

		
		pm_runtime_disable(dev);
		pm_runtime_set_active(dev);
		pm_runtime_enable(dev);

		
		if (mdwc->otg_xceiv) {
			mdwc->ext_xceiv.notify_ext_events(mdwc->otg_xceiv->otg,
							DWC3_EVENT_PHY_RESUME);
			mdwc->ext_xceiv.notify_ext_events(mdwc->otg_xceiv->otg,
							DWC3_EVENT_XCEIV_STATE);

		} else if (mdwc->scope == POWER_SUPPLY_SCOPE_SYSTEM) {
			pm_runtime_resume(&dwc->xhci->dev);
		}
	}

	return ret;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int dwc3_msm_runtime_idle(struct device *dev)
{
	struct dwc3_msm *mdwc = dev_get_drvdata(dev);
<<<<<<< HEAD
=======
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);
>>>>>>> 0e91d2a... Nougat

	dev_dbg(dev, "DWC3-msm runtime idle\n");

	if (mdwc->ext_chg_active) {
		dev_dbg(dev, "Deferring LPM\n");
		pm_schedule_suspend(dev, 3000);
		return -EAGAIN;
	}

	return 0;
}

static int dwc3_msm_runtime_suspend(struct device *dev)
{
	struct dwc3_msm *mdwc = dev_get_drvdata(dev);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(dev, "DWC3-msm runtime suspend\n");
	dbg_event(0xFF, "RT Sus", 0);

	return dwc3_msm_suspend(mdwc);
}

static int dwc3_msm_runtime_resume(struct device *dev)
{
	struct dwc3_msm *mdwc = dev_get_drvdata(dev);
	struct dwc3 *dwc = platform_get_drvdata(mdwc->dwc3);

	dev_dbg(dev, "DWC3-msm runtime resume\n");
	dbg_event(0xFF, "RT Res", 0);

	return dwc3_msm_resume(mdwc);
}
#endif

static const struct dev_pm_ops dwc3_msm_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_msm_pm_suspend, dwc3_msm_pm_resume)
	SET_RUNTIME_PM_OPS(dwc3_msm_runtime_suspend, dwc3_msm_runtime_resume,
				dwc3_msm_runtime_idle)
};

static const struct of_device_id of_dwc3_matach[] = {
	{
		.compatible = "qcom,dwc-usb3-msm",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_dwc3_matach);

static struct platform_driver dwc3_msm_driver = {
	.probe		= dwc3_msm_probe,
	.remove		= dwc3_msm_remove,
	.driver		= {
		.name	= "msm-dwc3",
		.pm	= &dwc3_msm_dev_pm_ops,
		.of_match_table	= of_dwc3_matach,
	},
};

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 MSM Glue Layer");

static int dwc3_msm_init(void)
{
	return platform_driver_register(&dwc3_msm_driver);
}
module_init(dwc3_msm_init);

static void __exit dwc3_msm_exit(void)
{
	platform_driver_unregister(&dwc3_msm_driver);
}
module_exit(dwc3_msm_exit);
