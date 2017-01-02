<<<<<<< HEAD
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2013-2016, The Linux Foundation. All rights reserved.
>>>>>>> 0e91d2a... Nougat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/qcom_iommu.h>
#include <linux/ratelimit.h>

<<<<<<< HEAD
#include "msm_isp47.h"
=======

>>>>>>> 0e91d2a... Nougat
#include "msm_isp_util.h"
#include "msm_isp_axi_util.h"
#include "msm_isp_stats_util.h"
#include "msm_isp.h"
#include "msm.h"
#include "msm_camera_io_util.h"
<<<<<<< HEAD
=======
#include "cam_hw_ops.h"
#include "msm_isp47.h"
#include "cam_soc_api.h"
#include "msm_isp48.h"
>>>>>>> 0e91d2a... Nougat

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

#define STATS_IDX_BF_SCALE  0
#define STATS_IDX_HDR_BE    1
#define STATS_IDX_BG        2
#define STATS_IDX_BF        3
#define STATS_IDX_HDR_BHIST 4
#define STATS_IDX_RS        5
#define STATS_IDX_CS        6
#define STATS_IDX_IHIST     7
#define STATS_IDX_BHIST     8

#define VFE47_8996V1_VERSION   0x60000000

#define VFE47_BURST_LEN 3
#define VFE47_FETCH_BURST_LEN 3
#define VFE47_STATS_BURST_LEN 3
#define VFE47_UB_SIZE_VFE0 2048
#define VFE47_UB_SIZE_VFE1 1536
#define VFE47_UB_STATS_SIZE 144
#define MSM_ISP47_TOTAL_IMAGE_UB_VFE0 (VFE47_UB_SIZE_VFE0 - VFE47_UB_STATS_SIZE)
#define MSM_ISP47_TOTAL_IMAGE_UB_VFE1 (VFE47_UB_SIZE_VFE1 - VFE47_UB_STATS_SIZE)
#define VFE47_WM_BASE(idx) (0xA0 + 0x24 * idx)
#define VFE47_RDI_BASE(idx) (0x39C + 0x4 * idx)
#define VFE47_XBAR_BASE(idx) (0x90 + 0x4 * (idx / 2))
#define VFE47_XBAR_SHIFT(idx) ((idx%2) ? 16 : 0)
#define VFE47_PING_PONG_BASE(wm, ping_pong) \
	(VFE47_WM_BASE(wm) + 0x4 * (1 + (~(ping_pong >> wm) & 0x1)))
#define SHIFT_BF_SCALE_BIT 1

<<<<<<< HEAD
static uint32_t stats_base_addr[] = {
	0x1E4, 
	0x19C, 
	0x1F0, 
	0x1CC, 
	0x1B4, 
	0x220, 
	0x238, 
	0x250, 
	0x208, 
};

static uint8_t stats_pingpong_offset_map[] = {
	11, 
=======
#define VFE47_BUS_RD_CGC_OVERRIDE_BIT 16

#define VFE47_VBIF_CLK_OFFSET    0x4

#ifdef CONFIG_MSM_CAMERA_AUTOMOTIVE
#define UB_CFG_POLICY MSM_WM_UB_EQUAL_SLICING
#else
#define UB_CFG_POLICY MSM_WM_UB_CFG_DEFAULT
#endif

static uint32_t stats_base_addr[] = {
	0x1D4, 
	0x254, 
	0x214, 
	0x1F4, 
	0x294, 
	0x2B4, 
	0x2D4, 
	0x274, 
	0x234, 
};

static uint8_t stats_pingpong_offset_map[] = {
>>>>>>> 0e91d2a... Nougat
	 8, 
	12, 
	10, 
	 9, 
	14, 
	15, 
	16, 
	13, 
<<<<<<< HEAD
};

#define VFE47_NUM_STATS_TYPE 9
=======
	11, 
};

static uint8_t stats_irq_map_comp_mask[] = {
	16, 
	17, 
	18, 
	19, 
	20, 
	21, 
	22, 
	23, 
	15, 
};
>>>>>>> 0e91d2a... Nougat
#define VFE47_STATS_BASE(idx) (stats_base_addr[idx])
#define VFE47_STATS_PING_PONG_BASE(idx, ping_pong) \
	(VFE47_STATS_BASE(idx) + 0x4 * \
	(~(ping_pong >> (stats_pingpong_offset_map[idx])) & 0x1))

<<<<<<< HEAD
#define VFE47_VBIF_ROUND_ROBIN_QOS_ARB   0x124
#define VFE47_BUS_BDG_QOS_CFG_BASE       0x378
#define VFE47_BUS_BDG_QOS_CFG_NUM            8
#define VFE47_BUS_BDG_DS_CFG_BASE        0xBD8
#define VFE47_BUS_BDG_DS_CFG_NUM            17

#define VFE47_CLK_IDX 2
static struct msm_cam_clk_info msm_vfe47_clk_info[VFE_CLK_INFO_MAX];

static uint32_t vfe47_qos_settings_8996_v1[] = {
	0xAAA9AAA9, 
	0xAAA9AAA9, 
	0xAAA9AAA9, 
	0xAAA9AAA9, 
	0xAAA9AAA9, 
	0xAAA9AAA9, 
	0xAAA9AAA9, 
	0x0001AAA9, 
};

static uint32_t vfe47_ds_settings_8996_v1[] = {
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x44441111, 
	0x00000103, 
};

static void msm_vfe47_init_qos_parms(struct vfe_device *vfe_dev)
=======
#define VFE47_SRC_CLK_DTSI_IDX 5

static struct msm_bus_vectors msm_isp_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors msm_isp_ping_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = MSM_ISP_MIN_AB,
		.ib  = MSM_ISP_MIN_IB,
	},
};

static struct msm_bus_vectors msm_isp_pong_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_paths msm_isp_bus_client_config[] = {
	{
		ARRAY_SIZE(msm_isp_init_vectors),
		msm_isp_init_vectors,
	},
	{
		ARRAY_SIZE(msm_isp_ping_vectors),
		msm_isp_ping_vectors,
	},
	{
		ARRAY_SIZE(msm_isp_pong_vectors),
		msm_isp_pong_vectors,
	},
};

static struct msm_bus_scale_pdata msm_isp_bus_client_pdata = {
	msm_isp_bus_client_config,
	ARRAY_SIZE(msm_isp_bus_client_config),
	.name = "msm_camera_isp",
};

void msm_vfe47_config_irq(struct vfe_device *vfe_dev,
		uint32_t irq0_mask, uint32_t irq1_mask,
		enum msm_isp_irq_operation oper)
{
	switch (oper) {
	case MSM_ISP_IRQ_ENABLE:
		vfe_dev->irq0_mask |= irq0_mask;
		vfe_dev->irq1_mask |= irq1_mask;
		break;
	case MSM_ISP_IRQ_DISABLE:
		vfe_dev->irq0_mask &= ~irq0_mask;
		vfe_dev->irq1_mask &= ~irq1_mask;
		break;
	case MSM_ISP_IRQ_SET:
		vfe_dev->irq0_mask = irq0_mask;
		vfe_dev->irq1_mask = irq1_mask;
		break;
	}
	msm_camera_io_w_mb(vfe_dev->irq0_mask,
				vfe_dev->vfe_base + 0x5C);
	msm_camera_io_w_mb(vfe_dev->irq1_mask,
				vfe_dev->vfe_base + 0x60);
}

static int32_t msm_vfe47_init_dt_parms(struct vfe_device *vfe_dev,
	struct msm_vfe_hw_init_parms *dt_parms, void __iomem *dev_mem_base)
>>>>>>> 0e91d2a... Nougat
{
	void __iomem *vfebase = vfe_dev->vfe_base;
	uint32_t *qos_settings = NULL;

	if (vfe_dev->vfe_hw_version >= VFE47_8996V1_VERSION)
		qos_settings = vfe47_qos_settings_8996_v1;

	if (qos_settings == NULL) {
		pr_err("%s: QOS is NOT configured for HW Version %x\n",
			__func__, vfe_dev->vfe_hw_version);
		BUG();
	} else {
		uint32_t i;
		for (i = 0; i < VFE47_BUS_BDG_QOS_CFG_NUM; i++)
			msm_camera_io_w(qos_settings[i],
				vfebase + VFE47_BUS_BDG_QOS_CFG_BASE + i * 4);
	}
}

<<<<<<< HEAD
static void msm_vfe47_init_vbif_parms(struct vfe_device *vfe_dev)
{
	void __iomem *vfe_vbif_base = vfe_dev->vfe_vbif_base;

	if (vfe_dev->vfe_hw_version >= VFE47_8996V1_VERSION) {
		msm_camera_io_w(0x3,
			vfe_vbif_base + VFE47_VBIF_ROUND_ROBIN_QOS_ARB);
	} else {
		pr_err("%s: VBIF is NOT configured for HW Version %x\n",
			__func__, vfe_dev->vfe_hw_version);
		BUG();
	}
}

static void msm_vfe47_init_danger_safe_parms(
	struct vfe_device *vfe_dev)
{
	void __iomem *vfebase = vfe_dev->vfe_base;
	uint32_t *ds_settings = NULL;

	if (vfe_dev->vfe_hw_version >= VFE47_8996V1_VERSION)
		ds_settings = vfe47_ds_settings_8996_v1;

	if (ds_settings == NULL) {
		pr_err("%s: DS is NOT configured for HW Version %x\n",
			__func__, vfe_dev->vfe_hw_version);
		BUG();
	} else {
		uint32_t i;
		for (i = 0; i < VFE47_BUS_BDG_DS_CFG_NUM; i++)
			msm_camera_io_w(ds_settings[i],
				vfebase + VFE47_BUS_BDG_DS_CFG_BASE + i * 4);
	}
}

static int msm_vfe47_init_hardware(struct vfe_device *vfe_dev)
{
	int rc = -1;

	rc = msm_isp_init_bandwidth_mgr(ISP_VFE0 + vfe_dev->pdev->id);
	if (rc < 0) {
		pr_err("%s: Bandwidth registration Failed!\n", __func__);
		goto bus_scale_register_failed;
	}

	if (vfe_dev->fs_vfe) {
		rc = regulator_enable(vfe_dev->fs_vfe);
		if (rc) {
			pr_err("%s: Regulator enable failed\n", __func__);
			goto fs_failed;
		}
	}
=======
static enum cam_ahb_clk_vote msm_isp47_get_cam_clk_vote(
	 enum msm_vfe_ahb_clk_vote vote)
{
	switch (vote) {
	case MSM_ISP_CAMERA_AHB_SVS_VOTE:
		return CAM_AHB_SVS_VOTE;
	case MSM_ISP_CAMERA_AHB_TURBO_VOTE:
		return CAM_AHB_TURBO_VOTE;
	case MSM_ISP_CAMERA_AHB_NOMINAL_VOTE:
		return CAM_AHB_NOMINAL_VOTE;
	case MSM_ISP_CAMERA_AHB_SUSPEND_VOTE:
		return CAM_AHB_SUSPEND_VOTE;
	}
	return 0;
}

static int msm_isp47_ahb_clk_cfg(struct vfe_device *vfe_dev,
			struct msm_isp_ahb_clk_cfg *ahb_cfg)
{
	int rc = 0;
	enum cam_ahb_clk_vote vote;

	vote = msm_isp47_get_cam_clk_vote(ahb_cfg->vote);

	if (vote && vfe_dev->ahb_vote != vote) {
		rc = cam_config_ahb_clk(NULL, 0,
			(ISP_VFE0 == vfe_dev->pdev->id ?
			CAM_AHB_CLIENT_VFE0 : CAM_AHB_CLIENT_VFE1), vote);
		if (rc)
			pr_err("%s: failed to set ahb vote to %x\n",
				__func__, vote);
		else
			vfe_dev->ahb_vote = vote;
	}
	return rc;
}

int msm_vfe47_init_hardware(struct vfe_device *vfe_dev)
{
	int rc = -1;
	enum cam_ahb_clk_client id;

	if (vfe_dev->pdev->id == 0)
		id = CAM_AHB_CLIENT_VFE0;
	else
		id = CAM_AHB_CLIENT_VFE1;

	rc = vfe_dev->hw_info->vfe_ops.platform_ops.enable_regulators(
								vfe_dev, 1);
	if (rc)
		goto enable_regulators_failed;

	rc = vfe_dev->hw_info->vfe_ops.platform_ops.enable_clks(
							vfe_dev, 1);
	if (rc)
		goto clk_enable_failed;
>>>>>>> 0e91d2a... Nougat

	rc = cam_config_ahb_clk(NULL, 0, id, CAM_AHB_SVS_VOTE);
	if (rc < 0) {
<<<<<<< HEAD
		pr_err("msm_isp_get_clk_info() failed\n");
		goto fs_failed;
	}
	if (vfe_dev->num_clk <= 0) {
		pr_err("%s: Invalid num of clock\n", __func__);
		goto fs_failed;
	} else {
		vfe_dev->vfe_clk =
			kzalloc(sizeof(struct clk *) * vfe_dev->num_clk,
			GFP_KERNEL);
		if (!vfe_dev->vfe_clk) {
			pr_err("%s:%d No memory\n", __func__, __LINE__);
			return -ENOMEM;
		}
=======
		pr_err("%s: failed to vote for AHB\n", __func__);
		goto ahb_vote_fail;
>>>>>>> 0e91d2a... Nougat
	}
	vfe_dev->ahb_vote = CAM_AHB_SVS_VOTE;

<<<<<<< HEAD
	vfe_dev->vfe_base = ioremap(vfe_dev->vfe_mem->start,
		resource_size(vfe_dev->vfe_mem));
	if (!vfe_dev->vfe_base) {
		rc = -ENOMEM;
		pr_err("%s: vfe ioremap failed\n", __func__);
		goto vfe_remap_failed;
	}
=======
	vfe_dev->common_data->dual_vfe_res->vfe_base[vfe_dev->pdev->id] =
		vfe_dev->vfe_base;
>>>>>>> 0e91d2a... Nougat

	rc = msm_isp_update_bandwidth(ISP_VFE0 + vfe_dev->pdev->id,
					MSM_ISP_MIN_AB, MSM_ISP_MIN_IB);
	if (rc)
		goto bw_enable_fail;

	rc = msm_camera_enable_irq(vfe_dev->vfe_irq, 1);
	if (rc < 0)
		goto irq_enable_fail;

	return rc;
<<<<<<< HEAD
irq_req_failed:
	iounmap(vfe_dev->vfe_vbif_base);
	vfe_dev->vfe_vbif_base = NULL;
vbif_remap_failed:
	iounmap(vfe_dev->vfe_base);
	vfe_dev->vfe_base = NULL;
vfe_remap_failed:
	msm_cam_clk_enable(&vfe_dev->pdev->dev, msm_vfe47_clk_info,
		vfe_dev->vfe_clk, vfe_dev->num_clk, 0);
clk_enable_failed:
	if (vfe_dev->fs_vfe)
		regulator_disable(vfe_dev->fs_vfe);
	kfree(vfe_dev->vfe_clk);
fs_failed:
	msm_isp_deinit_bandwidth_mgr(ISP_VFE0 + vfe_dev->pdev->id);
bus_scale_register_failed:
=======
irq_enable_fail:
	msm_isp_update_bandwidth(ISP_VFE0 + vfe_dev->pdev->id, 0, 0);
bw_enable_fail:
	vfe_dev->common_data->dual_vfe_res->vfe_base[vfe_dev->pdev->id] = NULL;
	if (cam_config_ahb_clk(NULL, 0, id, CAM_AHB_SUSPEND_VOTE) < 0)
		pr_err("%s: failed to remove vote for AHB\n", __func__);
	vfe_dev->ahb_vote = CAM_AHB_SUSPEND_VOTE;
ahb_vote_fail:
	vfe_dev->hw_info->vfe_ops.platform_ops.enable_clks(vfe_dev, 0);
clk_enable_failed:
	vfe_dev->hw_info->vfe_ops.platform_ops.enable_regulators(vfe_dev, 0);
enable_regulators_failed:
>>>>>>> 0e91d2a... Nougat
	return rc;
}

void msm_vfe47_release_hardware(struct vfe_device *vfe_dev)
{
<<<<<<< HEAD
	free_irq(vfe_dev->vfe_irq->start, vfe_dev);
	tasklet_kill(&vfe_dev->vfe_tasklet);
	iounmap(vfe_dev->vfe_vbif_base);
	vfe_dev->vfe_vbif_base = NULL;
	iounmap(vfe_dev->vfe_base);
	vfe_dev->vfe_base = NULL;
	msm_cam_clk_enable(&vfe_dev->pdev->dev, msm_vfe47_clk_info,
		vfe_dev->vfe_clk, vfe_dev->num_clk, 0);
	kfree(vfe_dev->vfe_clk);
	regulator_disable(vfe_dev->fs_vfe);
	msm_isp_deinit_bandwidth_mgr(ISP_VFE0 + vfe_dev->pdev->id);
=======
	enum cam_ahb_clk_client id;

	
	vfe_dev->irq0_mask = 0;
	vfe_dev->irq1_mask = 0;
	vfe_dev->hw_info->vfe_ops.irq_ops.config_irq(vfe_dev,
				vfe_dev->irq0_mask, vfe_dev->irq1_mask,
				MSM_ISP_IRQ_SET);
	msm_camera_enable_irq(vfe_dev->vfe_irq, 0);
	tasklet_kill(&vfe_dev->vfe_tasklet);
	msm_isp_flush_tasklet(vfe_dev);

	vfe_dev->common_data->dual_vfe_res->vfe_base[vfe_dev->pdev->id] = NULL;

	msm_isp_update_bandwidth(ISP_VFE0 + vfe_dev->pdev->id, 0, 0);

	if (vfe_dev->pdev->id == 0)
		id = CAM_AHB_CLIENT_VFE0;
	else
		id = CAM_AHB_CLIENT_VFE1;

	if (cam_config_ahb_clk(NULL, 0, id, CAM_AHB_SUSPEND_VOTE) < 0)
		pr_err("%s: failed to vote for AHB\n", __func__);

	vfe_dev->ahb_vote = CAM_AHB_SUSPEND_VOTE;

	vfe_dev->hw_info->vfe_ops.platform_ops.enable_clks(
							vfe_dev, 0);
	vfe_dev->hw_info->vfe_ops.platform_ops.enable_regulators(vfe_dev, 0);
>>>>>>> 0e91d2a... Nougat
}

void msm_vfe47_init_hardware_reg(struct vfe_device *vfe_dev)
{
<<<<<<< HEAD
	msm_vfe47_init_qos_parms(vfe_dev);
	msm_vfe47_init_vbif_parms(vfe_dev);
	msm_vfe47_init_danger_safe_parms(vfe_dev);

	
	msm_camera_io_w(0x00000001, vfe_dev->vfe_base + 0x84);
	
	msm_camera_io_w(0xE00000F1, vfe_dev->vfe_base + 0x5C);
	msm_camera_io_w_mb(0xE1FFFFFF, vfe_dev->vfe_base + 0x60);
=======
	struct msm_vfe_hw_init_parms qos_parms;
	struct msm_vfe_hw_init_parms vbif_parms;
	struct msm_vfe_hw_init_parms ds_parms;

	memset(&qos_parms, 0, sizeof(struct msm_vfe_hw_init_parms));
	memset(&vbif_parms, 0, sizeof(struct msm_vfe_hw_init_parms));
	memset(&ds_parms, 0, sizeof(struct msm_vfe_hw_init_parms));

	qos_parms.entries = "qos-entries";
	qos_parms.regs = "qos-regs";
	qos_parms.settings = "qos-settings";
	vbif_parms.entries = "vbif-entries";
	vbif_parms.regs = "vbif-regs";
	vbif_parms.settings = "vbif-settings";
	ds_parms.entries = "ds-entries";
	ds_parms.regs = "ds-regs";
	ds_parms.settings = "ds-settings";

	msm_vfe47_init_dt_parms(vfe_dev, &qos_parms, vfe_dev->vfe_base);
	msm_vfe47_init_dt_parms(vfe_dev, &ds_parms, vfe_dev->vfe_base);
	msm_vfe47_init_dt_parms(vfe_dev, &vbif_parms, vfe_dev->vfe_vbif_base);


	
	msm_camera_io_w(0x00000101, vfe_dev->vfe_base + 0x84);
	
	msm_vfe47_config_irq(vfe_dev, 0x810000E0, 0xFFFFFF7E,
				MSM_ISP_IRQ_ENABLE);
>>>>>>> 0e91d2a... Nougat
	msm_camera_io_w(0xFFFFFFFF, vfe_dev->vfe_base + 0x64);
	msm_camera_io_w_mb(0xFFFFFFFF, vfe_dev->vfe_base + 0x68);
	msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x58);
}

void msm_vfe47_clear_status_reg(struct vfe_device *vfe_dev)
{
	msm_vfe47_config_irq(vfe_dev, 0x80000000, 0x0,
				MSM_ISP_IRQ_SET);
	msm_camera_io_w(0xFFFFFFFF, vfe_dev->vfe_base + 0x64);
	msm_camera_io_w_mb(0xFFFFFFFF, vfe_dev->vfe_base + 0x68);
	msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x58);
}

void msm_vfe47_process_reset_irq(struct vfe_device *vfe_dev,
	uint32_t irq_status0, uint32_t irq_status1)
{
	if (irq_status0 & (1 << 31)) {
		complete(&vfe_dev->reset_complete);
		vfe_dev->reset_pending = 0;
	}
}

void msm_vfe47_process_halt_irq(struct vfe_device *vfe_dev,
	uint32_t irq_status0, uint32_t irq_status1)
{
	if (irq_status1 & (1 << 8)) {
		complete(&vfe_dev->halt_complete);
		msm_camera_io_w(0x0, vfe_dev->vfe_base + 0x374);
	}
}

void msm_vfe47_process_input_irq(struct vfe_device *vfe_dev,
	uint32_t irq_status0, uint32_t irq_status1,
	struct msm_isp_timestamp *ts)
{
	if (!(irq_status0 & 0x1000003))
		return;

<<<<<<< HEAD
=======
	if (irq_status0 & (1 << 0)) {
		ISP_DBG("vfe %d: SOF IRQ, frame id %d\n",
			vfe_dev->pdev->id,
			vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id);
		msm_isp_increment_frame_id(vfe_dev, VFE_PIX_0, ts);
	}

>>>>>>> 0e91d2a... Nougat
	if (irq_status0 & (1 << 24)) {
		ISP_DBG("%s: Fetch Engine Read IRQ\n", __func__);
		msm_isp_fetch_engine_done_notify(vfe_dev,
			&vfe_dev->fetch_engine_info);
	}

	if (irq_status0 & (1 << 0)) {
		if (vfe_dev->axi_data.src_info[VFE_PIX_0].raw_stream_count > 0
			&& vfe_dev->axi_data.src_info[VFE_PIX_0].
			pix_stream_count == 0) {
			ISP_DBG("%s: SOF IRQ\n", __func__);
			msm_isp_notify(vfe_dev, ISP_EVENT_SOF, VFE_PIX_0, ts);
			if (vfe_dev->axi_data.stream_update[VFE_PIX_0])
				msm_isp_axi_stream_update(vfe_dev, VFE_PIX_0);
			msm_isp_update_framedrop_reg(vfe_dev, VFE_PIX_0);
		}
	}
	if (irq_status0 & (1 << 1))
		ISP_DBG("%s: EOF IRQ\n", __func__);
}

void msm_vfe47_process_violation_status(
	struct vfe_device *vfe_dev)
{
	uint32_t violation_status = vfe_dev->error_info.violation_status;

	if (violation_status > 39) {
		pr_err("%s: invalid violation status %d\n",
			__func__, violation_status);
		return;
	}

	pr_err("%s: VFE pipeline violation status %d\n", __func__,
		violation_status);
}

void msm_vfe47_process_error_status(struct vfe_device *vfe_dev)
{
	uint32_t error_status1 = vfe_dev->error_info.error_mask1;

	if (error_status1 & (1 << 0))
		pr_err("%s: camif error status: 0x%x\n",
			__func__, vfe_dev->error_info.camif_status);
<<<<<<< HEAD
=======
		
		msm_camera_io_dump(vfe_dev->vfe_base + 0x478, 0x3C, 1);
		
		if (TESTGEN == vfe_dev->axi_data.src_info[VFE_PIX_0].input_mux)
			msm_camera_io_dump(vfe_dev->vfe_base + 0xC58, 0x28, 1);
	}
>>>>>>> 0e91d2a... Nougat
	if (error_status1 & (1 << 1))
		pr_err("%s: stats bhist overwrite\n", __func__);
	if (error_status1 & (1 << 2))
		pr_err("%s: stats cs overwrite\n", __func__);
	if (error_status1 & (1 << 3))
		pr_err("%s: stats ihist overwrite\n", __func__);
	if (error_status1 & (1 << 4))
		pr_err("%s: realign buf y overflow\n", __func__);
	if (error_status1 & (1 << 5))
		pr_err("%s: realign buf cb overflow\n", __func__);
	if (error_status1 & (1 << 6))
		pr_err("%s: realign buf cr overflow\n", __func__);
	if (error_status1 & (1 << 7)) {
		pr_err("%s: violation\n", __func__);
		msm_vfe47_process_violation_status(vfe_dev);
	}
	if (error_status1 & (1 << 9))
		pr_err("%s: image master 0 bus overflow\n", __func__);
	if (error_status1 & (1 << 10))
		pr_err("%s: image master 1 bus overflow\n", __func__);
	if (error_status1 & (1 << 11))
		pr_err("%s: image master 2 bus overflow\n", __func__);
	if (error_status1 & (1 << 12))
		pr_err("%s: image master 3 bus overflow\n", __func__);
	if (error_status1 & (1 << 13))
		pr_err("%s: image master 4 bus overflow\n", __func__);
	if (error_status1 & (1 << 14))
		pr_err("%s: image master 5 bus overflow\n", __func__);
	if (error_status1 & (1 << 15))
		pr_err("%s: image master 6 bus overflow\n", __func__);
	if (error_status1 & (1 << 16))
		pr_err("%s: status hdr be bus overflow\n", __func__);
	if (error_status1 & (1 << 17))
		pr_err("%s: status bg bus overflow\n", __func__);
	if (error_status1 & (1 << 18))
		pr_err("%s: status bf bus overflow\n", __func__);
	if (error_status1 & (1 << 19))
		pr_err("%s: status hdr bhist bus overflow\n", __func__);
	if (error_status1 & (1 << 20))
		pr_err("%s: status rs bus overflow\n", __func__);
	if (error_status1 & (1 << 21))
		pr_err("%s: status cs bus overflow\n", __func__);
	if (error_status1 & (1 << 22))
		pr_err("%s: status ihist bus overflow\n", __func__);
	if (error_status1 & (1 << 23))
		pr_err("%s: status skin bhist bus overflow\n", __func__);
	if (error_status1 & (1 << 24))
<<<<<<< HEAD
		pr_err("%s: status bf scale bus overflow\n", __func__);
}

static void msm_vfe47_read_irq_status(struct vfe_device *vfe_dev,
=======
		pr_err("%s: status aec bg bus overflow\n", __func__);
	if (error_status1 & (1 << 25))
		pr_err("%s: status dsp error\n", __func__);
}

void msm_vfe47_read_irq_status(struct vfe_device *vfe_dev,
>>>>>>> 0e91d2a... Nougat
	uint32_t *irq_status0, uint32_t *irq_status1)
{
	*irq_status0 = msm_camera_io_r(vfe_dev->vfe_base + 0x6C);
	*irq_status1 = msm_camera_io_r(vfe_dev->vfe_base + 0x70);
<<<<<<< HEAD
	msm_camera_io_w(*irq_status0, vfe_dev->vfe_base + 0x64);
	msm_camera_io_w(*irq_status1, vfe_dev->vfe_base + 0x68);
	msm_camera_io_w_mb(1, vfe_dev->vfe_base + 0x58);
=======
	
	msm_camera_io_w(*irq_status0, vfe_dev->vfe_base + 0x64);
	msm_camera_io_w(*irq_status1, vfe_dev->vfe_base + 0x68);
	msm_camera_io_w_mb(1, vfe_dev->vfe_base + 0x58);
	*irq_status0 &= vfe_dev->irq0_mask;
	*irq_status1 &= vfe_dev->irq1_mask;
>>>>>>> 0e91d2a... Nougat

	if (*irq_status1 & (1 << 0))
		vfe_dev->error_info.camif_status =
<<<<<<< HEAD
		msm_camera_io_r(vfe_dev->vfe_base + 0x3D0);
=======
		msm_camera_io_r(vfe_dev->vfe_base + 0x4A4);
		
		msm_vfe47_config_irq(vfe_dev, 0, (1 << 0), MSM_ISP_IRQ_DISABLE);
	}
>>>>>>> 0e91d2a... Nougat

	if (*irq_status1 & (1 << 7))
		vfe_dev->error_info.violation_status =
		msm_camera_io_r(vfe_dev->vfe_base + 0x7C);

}

void msm_vfe47_process_reg_update(struct vfe_device *vfe_dev,
	uint32_t irq_status0, uint32_t irq_status1,
	struct msm_isp_timestamp *ts)
{
	enum msm_vfe_input_src i;
	uint32_t shift_irq;

	if (!(irq_status0 & 0xF0))
		return;
	
	shift_irq = ((irq_status0 & 0xF0) >> 4);

	for (i = VFE_PIX_0; i <= VFE_RAW_2; i++) {
		if (shift_irq & BIT(i)) {
			vfe_dev->axi_data.reg_update_requested &= ~BIT(i);
			ISP_DBG("%s update_mask %x\n", __func__,
				(uint32_t)BIT(i));
			switch (i) {
			case VFE_PIX_0:
				msm_isp_notify(vfe_dev, ISP_EVENT_REG_UPDATE,
					VFE_PIX_0, ts);
				if (atomic_read(
					&vfe_dev->stats_data.stats_update))
					msm_isp_stats_stream_update(vfe_dev);
				break;
			case VFE_RAW_0:
			case VFE_RAW_1:
			case VFE_RAW_2:
<<<<<<< HEAD
=======
				msm_isp_increment_frame_id(vfe_dev, i, ts);
>>>>>>> 0e91d2a... Nougat
				msm_isp_notify(vfe_dev, ISP_EVENT_SOF, i, ts);
				msm_isp_update_framedrop_reg(vfe_dev, i);
				vfe_dev->hw_info->vfe_ops.core_ops.
					reg_update(vfe_dev, i);
				break;
			default:
				pr_err("%s: Error case\n", __func__);
				return;
			}
			if (vfe_dev->axi_data.stream_update[i])
				msm_isp_axi_stream_update(vfe_dev, i);
			msm_isp_save_framedrop_values(vfe_dev, i);
			if (atomic_read(&vfe_dev->axi_data.axi_cfg_update[i])) {
				msm_isp_axi_cfg_update(vfe_dev, i);
				if (atomic_read(
					&vfe_dev->axi_data.axi_cfg_update[i]) ==
					0)
					msm_isp_notify(vfe_dev,
						ISP_EVENT_STREAM_UPDATE_DONE,
						i, ts);
			}
		}
	}
}

void msm_vfe47_process_epoch_irq(struct vfe_device *vfe_dev,
	uint32_t irq_status0, uint32_t irq_status1,
	struct msm_isp_timestamp *ts)
{
	if (!(irq_status0 & 0xc))
		return;

	if (irq_status0 & BIT(2)) {
		ISP_DBG("%s: EPOCH0 IRQ\n", __func__);
		msm_isp_update_framedrop_reg(vfe_dev, VFE_PIX_0);
		msm_isp_update_stats_framedrop_reg(vfe_dev);
		msm_isp_update_error_frame_count(vfe_dev);
<<<<<<< HEAD
=======
		msm_isp_notify(vfe_dev, ISP_EVENT_SOF, VFE_PIX_0, ts);
		if (vfe_dev->axi_data.src_info[VFE_PIX_0].raw_stream_count > 0
			&& vfe_dev->axi_data.src_info[VFE_PIX_0].
			pix_stream_count == 0) {
			if (vfe_dev->axi_data.stream_update[VFE_PIX_0])
				msm_isp_axi_stream_update(vfe_dev, VFE_PIX_0);
			vfe_dev->hw_info->vfe_ops.core_ops.reg_update(
				vfe_dev, VFE_PIX_0);
		}
>>>>>>> 0e91d2a... Nougat
	}
}

void msm_vfe47_reg_update(struct vfe_device *vfe_dev,
	enum msm_vfe_input_src frame_src)
{
	uint32_t update_mask = 0;
<<<<<<< HEAD
=======
	unsigned long flags;

>>>>>>> 0e91d2a... Nougat
	
	if (frame_src > VFE_RAW_2 && frame_src != VFE_SRC_MAX) {
		pr_err("%s Error case\n", __func__);
		return;
	}

	if (frame_src == VFE_SRC_MAX)
		update_mask = 0xF;
	else
		update_mask = BIT((uint32_t)frame_src);
	ISP_DBG("%s update_mask %x\n", __func__, update_mask);
<<<<<<< HEAD
	vfe_dev->axi_data.reg_update_requested |= update_mask;
	msm_camera_io_w_mb(vfe_dev->axi_data.reg_update_requested,
		vfe_dev->vfe_base + 0x3D8);
=======

	spin_lock_irqsave(&vfe_dev->reg_update_lock, flags);
	vfe_dev->axi_data.src_info[VFE_PIX_0].reg_update_frame_id =
		vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id;
	vfe_dev->reg_update_requested |= update_mask;
	vfe_dev->common_data->dual_vfe_res->reg_update_mask[vfe_dev->pdev->id] =
		vfe_dev->reg_update_requested;
	if ((vfe_dev->is_split && vfe_dev->pdev->id == ISP_VFE1) &&
		((frame_src == VFE_PIX_0) || (frame_src == VFE_SRC_MAX))) {
		msm_camera_io_w_mb(update_mask,
			vfe_dev->common_data->dual_vfe_res->
			vfe_base[ISP_VFE0] + 0x4AC);
		msm_camera_io_w_mb(update_mask,
			vfe_dev->vfe_base + 0x4AC);
	} else if (!vfe_dev->is_split ||
		((frame_src == VFE_PIX_0) &&
		(vfe_dev->axi_data.camif_state == CAMIF_STOPPING)) ||
		(frame_src >= VFE_RAW_0 && frame_src <= VFE_SRC_MAX)) {
		msm_camera_io_w_mb(update_mask,
			vfe_dev->vfe_base + 0x4AC);
	}
	spin_unlock_irqrestore(&vfe_dev->reg_update_lock, flags);
>>>>>>> 0e91d2a... Nougat
}

long msm_vfe47_reset_hardware(struct vfe_device *vfe_dev,
	uint32_t first_start, uint32_t blocking_call)
{
	long rc = 0;
<<<<<<< HEAD
=======
	uint32_t reset;

>>>>>>> 0e91d2a... Nougat
	init_completion(&vfe_dev->reset_complete);

	if (blocking_call)
		vfe_dev->reset_pending = 1;

	if (first_start) {
<<<<<<< HEAD
		msm_camera_io_w_mb(0x1FF, vfe_dev->vfe_base + 0x18);
	} else {
		msm_camera_io_w_mb(0x1EF, vfe_dev->vfe_base + 0x18);
=======
		if (msm_vfe_is_vfe48(vfe_dev))
			reset = 0x3F7;
		else
			reset = 0x3FF;
		msm_camera_io_w_mb(reset, vfe_dev->vfe_base + 0x18);
	} else {
		if (msm_vfe_is_vfe48(vfe_dev))
			reset = 0x3E7;
		else
			reset = 0x3EF;
		msm_camera_io_w_mb(reset, vfe_dev->vfe_base + 0x18);
>>>>>>> 0e91d2a... Nougat
		msm_camera_io_w(0x7FFFFFFF, vfe_dev->vfe_base + 0x64);
		msm_camera_io_w(0xFFFFFEFF, vfe_dev->vfe_base + 0x68);
		msm_camera_io_w(0x1, vfe_dev->vfe_base + 0x58);
		vfe_dev->hw_info->vfe_ops.axi_ops.
			reload_wm(vfe_dev, 0x0001FFFF);
	}

	if (blocking_call) {
		rc = wait_for_completion_timeout(
			&vfe_dev->reset_complete, msecs_to_jiffies(50));
		if (rc <= 0) {
			pr_err("%s:%d failed: reset timeout\n", __func__,
				__LINE__);
			vfe_dev->reset_pending = 0;
		}
	}

	return rc;
}

<<<<<<< HEAD
static void msm_vfe47_axi_reload_wm(
	struct vfe_device *vfe_dev, uint32_t reload_mask)
=======
void msm_vfe47_axi_reload_wm(struct vfe_device *vfe_dev,
	void __iomem *vfe_base, uint32_t reload_mask)
>>>>>>> 0e91d2a... Nougat
{
	msm_camera_io_w_mb(reload_mask, vfe_dev->vfe_base + 0x80);
}

void msm_vfe47_axi_update_cgc_override(struct vfe_device *vfe_dev,
	uint8_t wm_idx, uint8_t enable)
{
	uint32_t val;

	
	val = msm_camera_io_r(vfe_dev->vfe_base + 0x3C);
	if (enable)
		val |= (1 << wm_idx);
	else
		val &= ~(1 << wm_idx);
	msm_camera_io_w_mb(val, vfe_dev->vfe_base + 0x3C);
}

static void msm_vfe47_axi_enable_wm(struct vfe_device *vfe_dev,
	uint8_t wm_idx, uint8_t enable)
{
	uint32_t val;

	val = msm_camera_io_r(vfe_dev->vfe_base + VFE47_WM_BASE(wm_idx));
	if (enable)
		val |= 0x1;
	else
		val &= ~0x1;
	msm_camera_io_w_mb(val,
		vfe_dev->vfe_base + VFE47_WM_BASE(wm_idx));
}

void msm_vfe47_axi_cfg_comp_mask(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	uint32_t comp_mask, comp_mask_index =
		stream_info->comp_mask_index;

	comp_mask = msm_camera_io_r(vfe_dev->vfe_base + 0x74);
	comp_mask &= ~(0x7F << (comp_mask_index * 8));
	comp_mask |= (axi_data->composite_info[comp_mask_index].
		stream_composite_mask << (comp_mask_index * 8));
	msm_camera_io_w(comp_mask, vfe_dev->vfe_base + 0x74);

	msm_vfe47_config_irq(vfe_dev, 1 << (comp_mask_index + 25), 0,
				MSM_ISP_IRQ_ENABLE);
}

void msm_vfe47_axi_clear_comp_mask(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	uint32_t comp_mask, comp_mask_index = stream_info->comp_mask_index;

	comp_mask = msm_camera_io_r(vfe_dev->vfe_base + 0x74);
	comp_mask &= ~(0x7F << (comp_mask_index * 8));
	msm_camera_io_w(comp_mask, vfe_dev->vfe_base + 0x74);

	msm_vfe47_config_irq(vfe_dev, (1 << (comp_mask_index + 25)), 0,
				MSM_ISP_IRQ_DISABLE);
}

void msm_vfe47_axi_cfg_wm_irq_mask(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	msm_vfe47_config_irq(vfe_dev, 1 << (stream_info->wm[0] + 8), 0,
				MSM_ISP_IRQ_ENABLE);
}

void msm_vfe47_axi_clear_wm_irq_mask(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	msm_vfe47_config_irq(vfe_dev, (1 << (stream_info->wm[0] + 8)), 0,
				MSM_ISP_IRQ_DISABLE);
}

<<<<<<< HEAD
static void msm_vfe47_cfg_framedrop(struct vfe_device *vfe_dev,
=======
void msm_vfe47_cfg_framedrop(void __iomem *vfe_base,
>>>>>>> 0e91d2a... Nougat
	struct msm_vfe_axi_stream *stream_info, uint32_t framedrop_pattern,
	uint32_t framedrop_period)
{
	uint32_t i, temp;

	for (i = 0; i < stream_info->num_planes; i++) {
		msm_camera_io_w(framedrop_pattern, vfe_dev->vfe_base +
			VFE47_WM_BASE(stream_info->wm[i]) + 0x1C);
		temp = msm_camera_io_r(vfe_dev->vfe_base +
			VFE47_WM_BASE(stream_info->wm[i]) + 0xC);
		temp &= 0xFFFFFF83;
		msm_camera_io_w(temp | framedrop_period << 2,
		vfe_dev->vfe_base + VFE47_WM_BASE(stream_info->wm[i]) + 0xC);
	}
	msm_camera_io_w_mb(0x1,vfe_base + 0x4AC); 
}

void msm_vfe47_clear_framedrop(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	uint32_t i;

	for (i = 0; i < stream_info->num_planes; i++)
		msm_camera_io_w(0, vfe_dev->vfe_base +
			VFE47_WM_BASE(stream_info->wm[i]) + 0x1C);
}

static int32_t msm_vfe47_convert_bpp_to_reg(int32_t bpp, uint32_t *bpp_reg)
{
	int rc = 0;
	switch (bpp) {
	case 8:
		*bpp_reg = 0;
		break;
	case 10:
		*bpp_reg = 1 << 0;
		break;
	case 12:
		*bpp_reg = 1 << 1;
		break;
	default:
		pr_err("%s:%d invalid bpp %d", __func__, __LINE__, bpp);
		return -EINVAL;
	}

	return rc;
}

static int32_t msm_vfe47_convert_io_fmt_to_reg(
	enum msm_isp_pack_fmt pack_format, uint32_t *pack_reg)
{
	int rc = 0;

	switch (pack_format) {
	case QCOM:
		*pack_reg = 0x0;
		break;
	case MIPI:
		*pack_reg = 0x1;
		break;
	case DPCM6:
		*pack_reg = 0x2;
		break;
	case DPCM8:
		*pack_reg = 0x3;
		break;
	case PLAIN8:
		*pack_reg = 0x4;
		break;
	case PLAIN16:
		*pack_reg = 0x5;
		break;
	default:
		pr_err("%s: invalid pack fmt %d!\n", __func__, pack_format);
		return -EINVAL;
	}

	return rc;
}

int32_t msm_vfe47_cfg_io_format(struct vfe_device *vfe_dev,
	enum msm_vfe_axi_stream_src stream_src, uint32_t io_format)
{
	int rc = 0;
	int bpp = 0, read_bpp = 0;
	enum msm_isp_pack_fmt pack_fmt = 0, read_pack_fmt = 0;
	uint32_t bpp_reg = 0, pack_reg = 0;
	uint32_t read_bpp_reg = 0, read_pack_reg = 0;
	uint32_t io_format_reg = 0; 

	io_format_reg = msm_camera_io_r(vfe_dev->vfe_base + 0x88);

	
	if ((stream_src < RDI_INTF_0) &&
		(vfe_dev->axi_data.src_info[VFE_PIX_0].input_mux ==
		EXTERNAL_READ)) {
		read_bpp = msm_isp_get_bit_per_pixel(
			vfe_dev->axi_data.src_info[VFE_PIX_0].input_format);
		rc = msm_vfe47_convert_bpp_to_reg(read_bpp, &read_bpp_reg);
		if (rc < 0) {
			pr_err("%s: convert_bpp_to_reg err! in_bpp %d rc %d\n",
				__func__, read_bpp, rc);
			return rc;
		}

		read_pack_fmt = msm_isp_get_pack_format(
			vfe_dev->axi_data.src_info[VFE_PIX_0].input_format);
		rc = msm_vfe47_convert_io_fmt_to_reg(
			read_pack_fmt, &read_pack_reg);
		if (rc < 0) {
			pr_err("%s: convert_io_fmt_to_reg err! rc = %d\n",
				__func__, rc);
			return rc;
		}
		
		io_format_reg &= 0xFFC8FFFF;
		io_format_reg |= (read_bpp_reg << 20 | read_pack_reg << 16);
	}

	bpp = msm_isp_get_bit_per_pixel(io_format);
	rc = msm_vfe47_convert_bpp_to_reg(bpp, &bpp_reg);
	if (rc < 0) {
		pr_err("%s: convert_bpp_to_reg err! bpp %d rc = %d\n",
			__func__, bpp, rc);
		return rc;
	}

	switch (stream_src) {
	case PIX_VIDEO:
	case PIX_ENCODER:
	case PIX_VIEWFINDER:
	case CAMIF_RAW:
		io_format_reg &= 0xFFFFCFFF;
		io_format_reg |= bpp_reg << 12;
		break;
	case IDEAL_RAW:
		
		pack_fmt = msm_isp_get_pack_format(io_format);
		rc = msm_vfe47_convert_io_fmt_to_reg(pack_fmt, &pack_reg);
		if (rc < 0) {
			pr_err("%s: convert_io_fmt_to_reg err! rc = %d\n",
				__func__, rc);
			return rc;
		}
		io_format_reg &= 0xFFFFFFC8;
		io_format_reg |= bpp_reg << 4 | pack_reg;
		break;
	case RDI_INTF_0:
	case RDI_INTF_1:
	case RDI_INTF_2:
	default:
		pr_err("%s: Invalid stream source\n", __func__);
		return -EINVAL;
	}
	msm_camera_io_w(io_format_reg, vfe_dev->vfe_base + 0x88);
	return 0;
}

int msm_vfe47_start_fetch_engine(struct vfe_device *vfe_dev,
	void *arg)
{
	int rc = 0;
	uint32_t bufq_handle;
	struct msm_isp_buffer *buf = NULL;
	struct msm_vfe_fetch_eng_start *fe_cfg = arg;

	if (vfe_dev->fetch_engine_info.is_busy == 1) {
		pr_err("%s: fetch engine busy\n", __func__);
		return -EINVAL;
	}

<<<<<<< HEAD
	bufq_handle = vfe_dev->buf_mgr->ops->get_bufq_handle(
		vfe_dev->buf_mgr, fe_cfg->session_id, fe_cfg->stream_id);
	vfe_dev->fetch_engine_info.bufq_handle = bufq_handle;
	vfe_dev->fetch_engine_info.session_id = fe_cfg->session_id;
	vfe_dev->fetch_engine_info.stream_id = fe_cfg->stream_id;
=======
	memset(&mapped_info, 0, sizeof(struct msm_isp_buffer_mapped_info));

	vfe_dev->fetch_engine_info.session_id = fe_cfg->session_id;
	vfe_dev->fetch_engine_info.stream_id = fe_cfg->stream_id;
	vfe_dev->fetch_engine_info.offline_mode = fe_cfg->offline_mode;
	vfe_dev->fetch_engine_info.fd = fe_cfg->fd;

	if (!fe_cfg->offline_mode) {
		bufq_handle = vfe_dev->buf_mgr->ops->get_bufq_handle(
			vfe_dev->buf_mgr, fe_cfg->session_id,
			fe_cfg->stream_id);
		vfe_dev->fetch_engine_info.bufq_handle = bufq_handle;

		rc = vfe_dev->buf_mgr->ops->get_buf_by_index(
			vfe_dev->buf_mgr, bufq_handle, fe_cfg->buf_idx, &buf);
		if (rc < 0 || !buf) {
			pr_err("%s: No fetch buffer rc= %d buf= %pK\n",
				__func__, rc, buf);
			return -EINVAL;
		}
		mapped_info = buf->mapped_info[0];
		buf->state = MSM_ISP_BUFFER_STATE_DISPATCHED;
	} else {
		rc = vfe_dev->buf_mgr->ops->map_buf(vfe_dev->buf_mgr,
			&mapped_info, fe_cfg->fd);
		if (rc < 0) {
			pr_err("%s: can not map buffer\n", __func__);
			return -EINVAL;
		}
	}
>>>>>>> 0e91d2a... Nougat

	rc = vfe_dev->buf_mgr->ops->get_buf_by_index(
		vfe_dev->buf_mgr, bufq_handle, fe_cfg->buf_idx, &buf);
	if (rc < 0) {
		pr_err("%s: No fetch buffer\n", __func__);
		return -EINVAL;
	}
	vfe_dev->fetch_engine_info.buf_idx = fe_cfg->buf_idx;
	vfe_dev->fetch_engine_info.is_busy = 1;

	msm_camera_io_w(buf->mapped_info[0].paddr, vfe_dev->vfe_base + 0x268);

	msm_camera_io_w_mb(0x100000, vfe_dev->vfe_base + 0x80);
	msm_camera_io_w_mb(0x200000, vfe_dev->vfe_base + 0x80);

	ISP_DBG("%s:VFE%d Fetch Engine ready\n", __func__, vfe_dev->pdev->id);
	buf->state = MSM_ISP_BUFFER_STATE_DISPATCHED;

	return 0;
}

int msm_vfe47_start_fetch_engine_multi_pass(struct vfe_device *vfe_dev,
	void *arg)
{
	int rc = 0;
	uint32_t bufq_handle = 0;
	struct msm_isp_buffer *buf = NULL;
	struct msm_vfe_fetch_eng_multi_pass_start *fe_cfg = arg;
	struct msm_isp_buffer_mapped_info mapped_info;

	if (vfe_dev->fetch_engine_info.is_busy == 1) {
		pr_err("%s: fetch engine busy\n", __func__);
		return -EINVAL;
	}

	memset(&mapped_info, 0, sizeof(struct msm_isp_buffer_mapped_info));

	vfe_dev->fetch_engine_info.session_id = fe_cfg->session_id;
	vfe_dev->fetch_engine_info.stream_id = fe_cfg->stream_id;
	vfe_dev->fetch_engine_info.offline_mode = fe_cfg->offline_mode;
	vfe_dev->fetch_engine_info.fd = fe_cfg->fd;

	if (!fe_cfg->offline_mode) {
		bufq_handle = vfe_dev->buf_mgr->ops->get_bufq_handle(
			vfe_dev->buf_mgr, fe_cfg->session_id,
			fe_cfg->stream_id);
		vfe_dev->fetch_engine_info.bufq_handle = bufq_handle;

		rc = vfe_dev->buf_mgr->ops->get_buf_by_index(
			vfe_dev->buf_mgr, bufq_handle, fe_cfg->buf_idx, &buf);
		if (rc < 0 || !buf) {
			pr_err("%s: No fetch buffer rc= %d buf= %pK\n",
				__func__, rc, buf);
			return -EINVAL;
		}
		mapped_info = buf->mapped_info[0];
		buf->state = MSM_ISP_BUFFER_STATE_DISPATCHED;
	} else {
		rc = vfe_dev->buf_mgr->ops->map_buf(vfe_dev->buf_mgr,
			&mapped_info, fe_cfg->fd);
		if (rc < 0) {
			pr_err("%s: can not map buffer\n", __func__);
			return -EINVAL;
		}
	}

	vfe_dev->fetch_engine_info.buf_idx = fe_cfg->buf_idx;
	vfe_dev->fetch_engine_info.is_busy = 1;

	msm_camera_io_w(mapped_info.paddr + fe_cfg->input_buf_offset,
		vfe_dev->vfe_base + 0x2F4);
	msm_camera_io_w_mb(0x100000, vfe_dev->vfe_base + 0x80);
	msm_camera_io_w_mb(0x200000, vfe_dev->vfe_base + 0x80);

	ISP_DBG("%s:VFE%d Fetch Engine ready\n", __func__, vfe_dev->pdev->id);

	return 0;
}
void msm_vfe47_cfg_fetch_engine(struct vfe_device *vfe_dev,
	struct msm_vfe_pix_cfg *pix_cfg)
{
	uint32_t x_size_word, temp;
	struct msm_vfe_fetch_engine_cfg *fe_cfg = NULL;

	if (pix_cfg->input_mux == EXTERNAL_READ) {
		fe_cfg = &pix_cfg->fetch_engine_cfg;
		pr_debug("%s:VFE%d wd x ht buf = %d x %d, fe = %d x %d\n",
			__func__, vfe_dev->pdev->id, fe_cfg->buf_width,
			fe_cfg->buf_height,
			fe_cfg->fetch_width, fe_cfg->fetch_height);

		temp = msm_camera_io_r(vfe_dev->vfe_base + 0x84);
		temp &= 0xFFFFFFFD;
		temp |= (1 << 1);
		msm_camera_io_w(temp, vfe_dev->vfe_base + 0x84);

		msm_vfe47_config_irq(vfe_dev, (1 << 24), 0,
				MSM_ISP_IRQ_ENABLE);

		temp = fe_cfg->fetch_height - 1;
		msm_camera_io_w(temp & 0x3FFF, vfe_dev->vfe_base + 0x278);

		x_size_word = msm_isp_cal_word_per_line(
			vfe_dev->axi_data.src_info[VFE_PIX_0].input_format,
			fe_cfg->buf_width);
		msm_camera_io_w((x_size_word - 1) << 16,
			vfe_dev->vfe_base + 0x27C);

		x_size_word = msm_isp_cal_word_per_line(
			vfe_dev->axi_data.src_info[VFE_PIX_0].input_format,
			fe_cfg->fetch_width);
		msm_camera_io_w(x_size_word << 16 |
			(temp & 0x3FFF) << 2 | VFE47_FETCH_BURST_LEN,
			vfe_dev->vfe_base + 0x280);

		temp = ((fe_cfg->buf_width - 1) & 0x3FFF) << 16 |
			((fe_cfg->buf_height - 1) & 0x3FFF);
		msm_camera_io_w(temp, vfe_dev->vfe_base + 0x284);

		
<<<<<<< HEAD
		msm_camera_io_w(0xF6543210, vfe_dev->vfe_base + 0x288);
		msm_camera_io_w(0xF, vfe_dev->vfe_base + 0x2A4);
=======
		msm_camera_io_w(0xF6543210, vfe_dev->vfe_base + 0x318);
		msm_camera_io_w(0xF, vfe_dev->vfe_base + 0x334);
>>>>>>> 0e91d2a... Nougat

		temp = msm_camera_io_r(vfe_dev->vfe_base + 0x50);
		temp |= 2 << 5;
		temp |= 128 << 8;
		temp |= pix_cfg->pixel_pattern;
		msm_camera_io_w(temp, vfe_dev->vfe_base + 0x50);

	} else {
		pr_err("%s: Invalid mux configuration - mux: %d", __func__,
			pix_cfg->input_mux);
	}
}

<<<<<<< HEAD
static void msm_vfe47_cfg_camif(struct vfe_device *vfe_dev,
=======
void msm_vfe47_cfg_testgen(struct vfe_device *vfe_dev,
	struct msm_vfe_testgen_cfg *testgen_cfg)
{
	uint32_t temp;
	uint32_t bit_per_pixel = 0;
	uint32_t bpp_reg = 0;
	uint32_t bayer_pix_pattern_reg = 0;
	uint32_t unicolorbar_reg = 0;
	uint32_t unicolor_enb = 0;

	bit_per_pixel = msm_isp_get_bit_per_pixel(
		vfe_dev->axi_data.src_info[VFE_PIX_0].input_format);

	switch (bit_per_pixel) {
	case 8:
		bpp_reg = 0x0;
		break;
	case 10:
		bpp_reg = 0x1;
		break;
	case 12:
		bpp_reg = 0x10;
		break;
	case 14:
		bpp_reg = 0x11;
		break;
	default:
		pr_err("%s: invalid bpp %d\n", __func__, bit_per_pixel);
		break;
	}

	msm_camera_io_w(bpp_reg << 16 | testgen_cfg->burst_num_frame,
		vfe_dev->vfe_base + 0xC5C);

	msm_camera_io_w(((testgen_cfg->lines_per_frame - 1) << 16) |
		(testgen_cfg->pixels_per_line - 1), vfe_dev->vfe_base + 0xC60);

	temp = msm_camera_io_r(vfe_dev->vfe_base + 0x50);
	temp |= (((testgen_cfg->h_blank) & 0x3FFF) << 8);
	temp |= (1 << 22);
	msm_camera_io_w(temp, vfe_dev->vfe_base + 0x50);

	msm_camera_io_w((1 << 16) | testgen_cfg->v_blank,
		vfe_dev->vfe_base + 0xC70);

	switch (testgen_cfg->pixel_bayer_pattern) {
	case ISP_BAYER_RGRGRG:
		bayer_pix_pattern_reg = 0x0;
		break;
	case ISP_BAYER_GRGRGR:
		bayer_pix_pattern_reg = 0x1;
		break;
	case ISP_BAYER_BGBGBG:
		bayer_pix_pattern_reg = 0x10;
		break;
	case ISP_BAYER_GBGBGB:
		bayer_pix_pattern_reg = 0x11;
		break;
	default:
		pr_err("%s: invalid pix pattern %d\n",
			__func__, bit_per_pixel);
		break;
	}

	if (testgen_cfg->color_bar_pattern == COLOR_BAR_8_COLOR) {
		unicolor_enb = 0x0;
	} else {
		unicolor_enb = 0x1;
		switch (testgen_cfg->color_bar_pattern) {
		case UNICOLOR_WHITE:
			unicolorbar_reg = 0x0;
			break;
		case UNICOLOR_YELLOW:
			unicolorbar_reg = 0x1;
			break;
		case UNICOLOR_CYAN:
			unicolorbar_reg = 0x10;
			break;
		case UNICOLOR_GREEN:
			unicolorbar_reg = 0x11;
			break;
		case UNICOLOR_MAGENTA:
			unicolorbar_reg = 0x100;
			break;
		case UNICOLOR_RED:
			unicolorbar_reg = 0x101;
			break;
		case UNICOLOR_BLUE:
			unicolorbar_reg = 0x110;
			break;
		case UNICOLOR_BLACK:
			unicolorbar_reg = 0x111;
			break;
		default:
			pr_err("%s: invalid colorbar %d\n",
				__func__, testgen_cfg->color_bar_pattern);
			break;
		}
	}

	msm_camera_io_w((testgen_cfg->rotate_period << 8) |
		(bayer_pix_pattern_reg << 6) | (unicolor_enb << 4) |
		(unicolorbar_reg), vfe_dev->vfe_base + 0xC78);
	return;
}

void msm_vfe47_cfg_camif(struct vfe_device *vfe_dev,
>>>>>>> 0e91d2a... Nougat
	struct msm_vfe_pix_cfg *pix_cfg)
{
	uint16_t first_pixel, last_pixel, first_line, last_line;
	struct msm_vfe_camif_cfg *camif_cfg = &pix_cfg->camif_cfg;
	uint32_t val;

	msm_camera_io_w(pix_cfg->input_mux << 5 | pix_cfg->pixel_pattern,
		vfe_dev->vfe_base + 0x50);

	first_pixel = camif_cfg->first_pixel;
	last_pixel = camif_cfg->last_pixel;
	first_line = camif_cfg->first_line;
	last_line = camif_cfg->last_line;

	msm_camera_io_w(camif_cfg->lines_per_frame << 16 |
		camif_cfg->pixels_per_line, vfe_dev->vfe_base + 0x3B4);

	msm_camera_io_w(first_pixel << 16 | last_pixel,
	vfe_dev->vfe_base + 0x3B8);

	msm_camera_io_w(first_line << 16 | last_line,
	vfe_dev->vfe_base + 0x3BC);

<<<<<<< HEAD
	msm_camera_io_w(0xFFFFFFFF, vfe_dev->vfe_base + 0x3C8);

	val = msm_camera_io_r(vfe_dev->vfe_base + 0x39C);
=======
	if (subsample_cfg->first_pixel ||
		subsample_cfg->last_pixel ||
		subsample_cfg->first_line ||
		subsample_cfg->last_line) {
		msm_camera_io_w(
		subsample_cfg->first_pixel << 16 |
			subsample_cfg->last_pixel,
			vfe_dev->vfe_base + 0xCE4);
		msm_camera_io_w(
		subsample_cfg->first_line << 16 |
			subsample_cfg->last_line,
			vfe_dev->vfe_base + 0xCE4);
		val = msm_camera_io_r(
			vfe_dev->vfe_base + 0x47C);
		ISP_DBG("%s: camif raw crop enabled\n", __func__);
		val |= 1 << 22;
		msm_camera_io_w(val,
			vfe_dev->vfe_base + 0x47C);
	}

	ISP_DBG("%s: camif raw op fmt %d\n",
		__func__, subsample_cfg->output_format);
	
	val = msm_camera_io_r(vfe_dev->vfe_base + 0x88);
	switch (subsample_cfg->output_format) {
	case CAMIF_PLAIN_8:
		val |= PLAIN8 << 9;
		break;
	case CAMIF_PLAIN_16:
		val |= PLAIN16 << 9;
		break;
	case CAMIF_MIPI_RAW:
		val |= MIPI << 9;
		break;
	case CAMIF_QCOM_RAW:
		val |= QCOM << 9;
		break;
	default:
		break;
	}
	msm_camera_io_w(val, vfe_dev->vfe_base + 0x88);

	val = msm_camera_io_r(vfe_dev->vfe_base + 0x46C);
>>>>>>> 0e91d2a... Nougat
	val |= camif_cfg->camif_input;
	msm_camera_io_w(val, vfe_dev->vfe_base + 0x39C);
}

void msm_vfe47_cfg_input_mux(struct vfe_device *vfe_dev,
	struct msm_vfe_pix_cfg *pix_cfg)
{
	uint32_t core_cfg = 0;
	uint32_t val = 0;

	core_cfg =  msm_camera_io_r(vfe_dev->vfe_base + 0x50);
	core_cfg &= 0xFFFFFF9F;

	switch (pix_cfg->input_mux) {
	case CAMIF:
		core_cfg |= 0x0 << 5;
		msm_camera_io_w_mb(core_cfg, vfe_dev->vfe_base + 0x50);
		msm_vfe47_cfg_camif(vfe_dev, pix_cfg);
		break;
	case TESTGEN:
		
		val = msm_camera_io_r(vfe_dev->vfe_base + 0x3C);
		val |= (1 << 31);
		msm_camera_io_w(val, vfe_dev->vfe_base + 0x3C);

		
		core_cfg |= 0x1 << 5;
		msm_camera_io_w_mb(core_cfg, vfe_dev->vfe_base + 0x50);
		msm_vfe47_cfg_camif(vfe_dev, pix_cfg);
		break;
	case EXTERNAL_READ:
		core_cfg |= 0x2 << 5;
		msm_camera_io_w_mb(core_cfg, vfe_dev->vfe_base + 0x50);
		msm_vfe47_cfg_fetch_engine(vfe_dev, pix_cfg);
		break;
	default:
		pr_err("%s: Unsupported input mux %d\n",
			__func__, pix_cfg->input_mux);
		break;
	}
	return;
}

<<<<<<< HEAD
static void msm_vfe47_update_camif_state(struct vfe_device *vfe_dev,
=======
void msm_vfe47_configure_hvx(struct vfe_device *vfe_dev,
	uint8_t is_stream_on)
{
	uint32_t val;
	if (is_stream_on == 1) {
		
		val = msm_camera_io_r(vfe_dev->vfe_base + 0x50);
		val |= (1 << 3);
		msm_camera_io_w_mb(val, vfe_dev->vfe_base + 0x50);
		val &= 0xFF7FFFFF;
		if (vfe_dev->hvx_cmd == HVX_ROUND_TRIP)
			val |= (1 << 23);
		msm_camera_io_w_mb(val, vfe_dev->vfe_base + 0x50);
	} else {
		
		val = msm_camera_io_r(vfe_dev->vfe_base + 0x50);
		val &= 0xFFFFFFF7;
		msm_camera_io_w_mb(val, vfe_dev->vfe_base + 0x50);
	}
}

void msm_vfe47_update_camif_state(struct vfe_device *vfe_dev,
>>>>>>> 0e91d2a... Nougat
	enum msm_isp_camif_update_state update_state)
{
	uint32_t val;
	bool bus_en, vfe_en;

	if (update_state == NO_UPDATE)
		return;

	if (update_state == ENABLE_CAMIF) {
		msm_camera_io_w(0x0, vfe_dev->vfe_base + 0x64);
		msm_camera_io_w(0x81, vfe_dev->vfe_base + 0x68);
		msm_camera_io_w(0x1, vfe_dev->vfe_base + 0x58);
		msm_vfe47_config_irq(vfe_dev, 0x15, 0x81,
					MSM_ISP_IRQ_ENABLE);

<<<<<<< HEAD
=======
		if ((vfe_dev->hvx_cmd > HVX_DISABLE) &&
			(vfe_dev->hvx_cmd <= HVX_ROUND_TRIP))
			msm_vfe47_configure_hvx(vfe_dev, 1);
		else
			msm_vfe47_configure_hvx(vfe_dev, 0);

>>>>>>> 0e91d2a... Nougat
		bus_en =
			((vfe_dev->axi_data.
			src_info[VFE_PIX_0].raw_stream_count > 0) ? 1 : 0);
		vfe_en =
			((vfe_dev->axi_data.
			src_info[VFE_PIX_0].pix_stream_count > 0) ? 1 : 0);
		val = msm_camera_io_r(vfe_dev->vfe_base + 0x3AC);
		val &= 0xFFFFFF3F;
		val = val | bus_en << 7 | vfe_en << 6;
<<<<<<< HEAD
		msm_camera_io_w(val, vfe_dev->vfe_base + 0x3AC);
		msm_camera_io_w_mb(0x4, vfe_dev->vfe_base + 0x3A8);
		msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x3A8);
		msm_camera_io_w_mb(0x200, vfe_dev->vfe_base + 0x4A0);
		vfe_dev->axi_data.src_info[VFE_PIX_0].active = 1;
		
		if (vfe_dev->axi_data.src_info[VFE_PIX_0].input_mux == TESTGEN)
			msm_camera_io_w(1, vfe_dev->vfe_base + 0xAF4);
	} else if (update_state == DISABLE_CAMIF) {
		msm_camera_io_w_mb(0x0, vfe_dev->vfe_base + 0x3A8);
		vfe_dev->axi_data.src_info[VFE_PIX_0].active = 0;
		
		if (vfe_dev->axi_data.src_info[VFE_PIX_0].input_mux == TESTGEN)
			msm_camera_io_w(1 << 1, vfe_dev->vfe_base + 0xAF4);
	} else if (update_state == DISABLE_CAMIF_IMMEDIATELY) {
		msm_camera_io_w_mb(0x6, vfe_dev->vfe_base + 0x3A8);
=======
		msm_camera_io_w(val, vfe_dev->vfe_base + 0x47C);
		msm_camera_io_w_mb(0x4, vfe_dev->vfe_base + 0x478);
		msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x478);
		
		msm_camera_io_w_mb(0x140000, vfe_dev->vfe_base + 0x4A0);
		vfe_dev->axi_data.src_info[VFE_PIX_0].active = 1;
		
		if (vfe_dev->axi_data.src_info[VFE_PIX_0].input_mux == TESTGEN)
			msm_camera_io_w(1, vfe_dev->vfe_base + 0xC58);
	} else if (update_state == DISABLE_CAMIF ||
		update_state == DISABLE_CAMIF_IMMEDIATELY) {
		
		msm_vfe47_config_irq(vfe_dev, 0, 0x81,
					MSM_ISP_IRQ_DISABLE);
		val = msm_camera_io_r(vfe_dev->vfe_base + 0x464);
		
		msm_camera_io_w_mb(val & ~(1 << 8), vfe_dev->vfe_base + 0x464);
		msm_camera_io_w_mb((update_state == DISABLE_CAMIF ? 0x0 : 0x6),
				vfe_dev->vfe_base + 0x478);
>>>>>>> 0e91d2a... Nougat
		vfe_dev->axi_data.src_info[VFE_PIX_0].active = 0;
		
		if (vfe_dev->axi_data.src_info[VFE_PIX_0].input_mux == TESTGEN)
<<<<<<< HEAD
			msm_camera_io_w(1 << 1, vfe_dev->vfe_base + 0xAF4);
=======
			msm_camera_io_w(1 << 1, vfe_dev->vfe_base + 0xC58);

		if ((vfe_dev->hvx_cmd > HVX_DISABLE) &&
			(vfe_dev->hvx_cmd <= HVX_ROUND_TRIP))
			msm_vfe47_configure_hvx(vfe_dev, 0);
>>>>>>> 0e91d2a... Nougat
	}
}

void msm_vfe47_cfg_rdi_reg(
	struct vfe_device *vfe_dev, struct msm_vfe_rdi_cfg *rdi_cfg,
	enum msm_vfe_input_src input_src)
{
	uint8_t rdi = input_src - VFE_RAW_0;
	uint32_t rdi_reg_cfg;

	rdi_reg_cfg = msm_camera_io_r(
		vfe_dev->vfe_base + VFE47_RDI_BASE(rdi));
	rdi_reg_cfg &= 0x03;
	rdi_reg_cfg |= (rdi * 3) << 28 | rdi_cfg->cid << 4 | 0x4;
	msm_camera_io_w(
		rdi_reg_cfg, vfe_dev->vfe_base + VFE47_RDI_BASE(rdi));
}

void msm_vfe47_axi_cfg_wm_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info,
	uint8_t plane_idx)
{
	uint32_t val;
	uint32_t wm_base = VFE47_WM_BASE(stream_info->wm[plane_idx]);

	val = msm_camera_io_r(vfe_dev->vfe_base + wm_base + 0xC);
	val &= ~0x2;
	if (stream_info->frame_based)
		val |= 0x2;
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0xC);
	if (!stream_info->frame_based) {
		
		val = ((msm_isp_cal_word_per_line(
			stream_info->output_format,
			stream_info->plane_cfg[plane_idx].
			output_width)+3)/4 - 1) << 16 |
			(stream_info->plane_cfg[plane_idx].
			output_height - 1);
<<<<<<< HEAD
		msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x14);
=======
		msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x1C);
>>>>>>> 0e91d2a... Nougat
		
		val = VFE47_BURST_LEN |
			(stream_info->plane_cfg[plane_idx].output_height - 1) <<
			2 |
			((msm_isp_cal_word_per_line(stream_info->output_format,
			stream_info->plane_cfg[plane_idx].
			output_stride)+1)/2) << 16;
	}
<<<<<<< HEAD
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x18);
=======
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x20);
>>>>>>> 0e91d2a... Nougat
	
	msm_camera_io_w(0xFFFFFFFF,
		vfe_dev->vfe_base + wm_base + 0x20);
	
}

void msm_vfe47_axi_clear_wm_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint8_t plane_idx)
{
	uint32_t val = 0;
	uint32_t wm_base = VFE47_WM_BASE(stream_info->wm[plane_idx]);

	
<<<<<<< HEAD
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0xC);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x14);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x18);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x20);
=======
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x14);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x1C);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x20);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + wm_base + 0x28);
>>>>>>> 0e91d2a... Nougat
}

void msm_vfe47_axi_cfg_wm_xbar_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info,
	uint8_t plane_idx)
{
	struct msm_vfe_axi_plane_cfg *plane_cfg =
		&stream_info->plane_cfg[plane_idx];
	uint8_t wm = stream_info->wm[plane_idx];
	uint32_t xbar_cfg = 0;
	uint32_t xbar_reg_cfg = 0;

	switch (stream_info->stream_src) {
	case PIX_VIDEO:
	case PIX_ENCODER:
	case PIX_VIEWFINDER: {
		if (plane_cfg->output_plane_format != CRCB_PLANE &&
			plane_cfg->output_plane_format != CBCR_PLANE) {
			
			xbar_cfg |= plane_cfg->output_plane_format << 8;
		} else {
			switch (stream_info->output_format) {
			case V4L2_PIX_FMT_NV12:
			case V4L2_PIX_FMT_NV14:
			case V4L2_PIX_FMT_NV16:
<<<<<<< HEAD
=======
			case V4L2_PIX_FMT_NV24:
>>>>>>> 0e91d2a... Nougat
				
				xbar_cfg |= 0x3 << 4;
				break;
			}
			xbar_cfg |= 0x1 << 2; 
		}
		if (stream_info->stream_src == PIX_VIEWFINDER)
			xbar_cfg |= 0x1; 
		else if (stream_info->stream_src == PIX_VIDEO)
			xbar_cfg |= 0x2;
		break;
	}
	case CAMIF_RAW:
		xbar_cfg = 0x300;
		break;
	case IDEAL_RAW:
		xbar_cfg = 0x400;
		break;
	case RDI_INTF_0:
		xbar_cfg = 0xC00;
		break;
	case RDI_INTF_1:
		xbar_cfg = 0xD00;
		break;
	case RDI_INTF_2:
		xbar_cfg = 0xE00;
		break;
	default:
		pr_err("%s: Invalid stream src\n", __func__);
		break;
	}
	xbar_reg_cfg =
		msm_camera_io_r(vfe_dev->vfe_base + VFE47_XBAR_BASE(wm));
	xbar_reg_cfg &= ~(0xFFFF << VFE47_XBAR_SHIFT(wm));
	xbar_reg_cfg |= (xbar_cfg << VFE47_XBAR_SHIFT(wm));
	msm_camera_io_w(xbar_reg_cfg,
		vfe_dev->vfe_base + VFE47_XBAR_BASE(wm));
}

void msm_vfe47_axi_clear_wm_xbar_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint8_t plane_idx)
{
	uint8_t wm = stream_info->wm[plane_idx];
	uint32_t xbar_reg_cfg = 0;

	xbar_reg_cfg =
		msm_camera_io_r(vfe_dev->vfe_base + VFE47_XBAR_BASE(wm));
	xbar_reg_cfg &= ~(0xFFFF << VFE47_XBAR_SHIFT(wm));
	msm_camera_io_w(xbar_reg_cfg,
		vfe_dev->vfe_base + VFE47_XBAR_BASE(wm));
}


void msm_vfe47_cfg_axi_ub_equal_default(
	struct vfe_device *vfe_dev)
{
	int i;
	uint32_t ub_offset = 0;
	struct msm_vfe_axi_shared_data *axi_data =
		&vfe_dev->axi_data;
	uint32_t total_image_size = 0;
	uint8_t num_used_wms = 0;
	uint32_t prop_size = 0;
	uint32_t wm_ub_size;
	uint64_t delta;

	for (i = 0; i < axi_data->hw_info->num_wm; i++) {
		if (axi_data->free_wm[i] > 0) {
			num_used_wms++;
			total_image_size += axi_data->wm_image_size[i];
		}
	}
	if (vfe_dev->pdev->id == ISP_VFE0) {
		prop_size = MSM_ISP47_TOTAL_IMAGE_UB_VFE0 -
		axi_data->hw_info->min_wm_ub * num_used_wms;
	} else if (vfe_dev->pdev->id == ISP_VFE1) {
		prop_size = MSM_ISP47_TOTAL_IMAGE_UB_VFE1 -
		axi_data->hw_info->min_wm_ub * num_used_wms;
	} else {
		pr_err("%s: incorrect VFE device\n", __func__);
	}
	for (i = 0; i < axi_data->hw_info->num_wm; i++) {
		if (axi_data->free_wm[i]) {
			delta = (uint64_t)axi_data->wm_image_size[i] *
					(uint64_t)prop_size;
			do_div(delta, total_image_size);
			wm_ub_size = axi_data->hw_info->min_wm_ub +
					(uint32_t)delta;
			msm_camera_io_w(ub_offset << 16 | (wm_ub_size - 1),
				vfe_dev->vfe_base + VFE47_WM_BASE(i) + 0x10);
			ub_offset += wm_ub_size;
		} else
			msm_camera_io_w(0,
				vfe_dev->vfe_base + VFE47_WM_BASE(i) + 0x10);
	}
}

void msm_vfe47_cfg_axi_ub_equal_slicing(
	struct vfe_device *vfe_dev)
{
	int i;
	uint32_t ub_offset = 0;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	uint32_t ub_equal_slice = 0;
	if (vfe_dev->pdev->id == ISP_VFE0) {
		ub_equal_slice = MSM_ISP47_TOTAL_IMAGE_UB_VFE0 /
		axi_data->hw_info->num_wm;
	} else if (vfe_dev->pdev->id == ISP_VFE1) {
		ub_equal_slice = MSM_ISP47_TOTAL_IMAGE_UB_VFE1 /
		axi_data->hw_info->num_wm;
	} else {
		pr_err("%s: incorrect VFE device\n ", __func__);
	}
	for (i = 0; i < axi_data->hw_info->num_wm; i++) {
		msm_camera_io_w(ub_offset << 16 | (ub_equal_slice - 1),
			vfe_dev->vfe_base + VFE47_WM_BASE(i) + 0x10);
		ub_offset += ub_equal_slice;
	}
}

void msm_vfe47_cfg_axi_ub(struct vfe_device *vfe_dev)
{
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;

	axi_data->wm_ub_cfg_policy = UB_CFG_POLICY;
	if (axi_data->wm_ub_cfg_policy == MSM_WM_UB_EQUAL_SLICING)
		msm_vfe47_cfg_axi_ub_equal_slicing(vfe_dev);
	else
		msm_vfe47_cfg_axi_ub_equal_default(vfe_dev);
}

<<<<<<< HEAD
static void msm_vfe47_update_ping_pong_addr(
	struct vfe_device *vfe_dev,
	uint8_t wm_idx, uint32_t pingpong_status, dma_addr_t paddr)
=======
void msm_vfe47_read_wm_ping_pong_addr(
	struct vfe_device *vfe_dev)
{
	msm_camera_io_dump(vfe_dev->vfe_base +
		(VFE47_WM_BASE(0) & 0xFFFFFFF0), 0x200, 1);
}

void msm_vfe47_update_ping_pong_addr(
	void __iomem *vfe_base,
	uint8_t wm_idx, uint32_t pingpong_bit, dma_addr_t paddr,
	int32_t buf_size)
>>>>>>> 0e91d2a... Nougat
{
	uint32_t paddr32 = (paddr & 0xFFFFFFFF);

	msm_camera_io_w(paddr32, vfe_dev->vfe_base +
		VFE47_PING_PONG_BASE(wm_idx, pingpong_status));
}

static void msm_vfe47_set_halt_restart_mask(struct vfe_device *vfe_dev)
{
	msm_vfe47_config_irq(vfe_dev, BIT(31), BIT(8), MSM_ISP_IRQ_SET);
}

int msm_vfe47_axi_halt(struct vfe_device *vfe_dev,
	uint32_t blocking)
{
	int rc = 0;
	enum msm_vfe_input_src i;

<<<<<<< HEAD
	
	msm_camera_io_w(BIT(31), vfe_dev->vfe_base + 0x5C);
	msm_camera_io_w(BIT(8), vfe_dev->vfe_base + 0x60);
=======
	val = msm_camera_io_r(vfe_dev->vfe_vbif_base + VFE47_VBIF_CLK_OFFSET);
	val |= 0x1;
	msm_camera_io_w(val, vfe_dev->vfe_vbif_base + VFE47_VBIF_CLK_OFFSET);

	
	msm_vfe47_set_halt_restart_mask(vfe_dev);
>>>>>>> 0e91d2a... Nougat

	
	msm_camera_io_w(0x7FFFFFFF, vfe_dev->vfe_base + 0x64);

	
	msm_camera_io_w(0xFFFFFEFF, vfe_dev->vfe_base + 0x68);

	
	msm_camera_io_w(0x1, vfe_dev->vfe_base + 0x58);

	if (blocking) {
		init_completion(&vfe_dev->halt_complete);
		
<<<<<<< HEAD
		msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x374);
=======
		msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x400);
>>>>>>> 0e91d2a... Nougat
		rc = wait_for_completion_timeout(
			&vfe_dev->halt_complete, msecs_to_jiffies(500));
	} else {
		
<<<<<<< HEAD
		msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x374);
=======
		msm_camera_io_w_mb(0x1, vfe_dev->vfe_base + 0x400);
>>>>>>> 0e91d2a... Nougat
	}

	for (i = VFE_PIX_0; i <= VFE_RAW_2; i++) {
		
		if (vfe_dev->axi_data.stream_update[i]) {
			ISP_DBG("%s: complete stream update\n", __func__);
			msm_isp_axi_stream_update(vfe_dev, i);
			if (vfe_dev->axi_data.stream_update[i])
				msm_isp_axi_stream_update(vfe_dev, i);
		}
		if (atomic_read(&vfe_dev->axi_data.axi_cfg_update[i])) {
			ISP_DBG("%s: complete on axi config update\n",
				__func__);
			msm_isp_axi_cfg_update(vfe_dev, i);
			if (atomic_read(&vfe_dev->axi_data.axi_cfg_update[i]))
				msm_isp_axi_cfg_update(vfe_dev, i);
		}
	}

	if (atomic_read(&vfe_dev->stats_data.stats_update)) {
		ISP_DBG("%s: complete on stats update\n", __func__);
		msm_isp_stats_stream_update(vfe_dev);
		if (atomic_read(&vfe_dev->stats_data.stats_update))
			msm_isp_stats_stream_update(vfe_dev);
	}

	return rc;
}

int msm_vfe47_axi_restart(struct vfe_device *vfe_dev,
	uint32_t blocking, uint32_t enable_camif)
{
	msm_camera_io_w(0x7FFFFFFF, vfe_dev->vfe_base + 0x64);
	msm_camera_io_w(0xFFFFFEFF, vfe_dev->vfe_base + 0x68);
	msm_camera_io_w(0x1, vfe_dev->vfe_base + 0x58);

	
<<<<<<< HEAD
	msm_camera_io_w(0x0, vfe_dev->vfe_base + 0x374);
=======
	msm_camera_io_w(0x0, vfe_dev->vfe_base + 0x400);
>>>>>>> 0e91d2a... Nougat

	memset(&vfe_dev->error_info, 0, sizeof(vfe_dev->error_info));
	atomic_set(&vfe_dev->error_info.overflow_state, NO_OVERFLOW);

	
	msm_vfe47_config_irq(vfe_dev, vfe_dev->recovery_irq0_mask,
		vfe_dev->recovery_irq1_mask, MSM_ISP_IRQ_SET);

	vfe_dev->hw_info->vfe_ops.core_ops.reg_update(vfe_dev, VFE_SRC_MAX);

	if (enable_camif) {
		vfe_dev->hw_info->vfe_ops.core_ops.
		update_camif_state(vfe_dev, ENABLE_CAMIF);
	}

	return 0;
}

uint32_t msm_vfe47_get_wm_mask(
	uint32_t irq_status0, uint32_t irq_status1)
{
	return (irq_status0 >> 8) & 0x7F;
}

uint32_t msm_vfe47_get_comp_mask(
	uint32_t irq_status0, uint32_t irq_status1)
{
	return (irq_status0 >> 25) & 0xF;
}

uint32_t msm_vfe47_get_pingpong_status(
	struct vfe_device *vfe_dev)
{
	return msm_camera_io_r(vfe_dev->vfe_base + 0x2A8);
}

int msm_vfe47_get_stats_idx(enum msm_isp_stats_type stats_type)
{
<<<<<<< HEAD
=======
	
>>>>>>> 0e91d2a... Nougat
	switch (stats_type) {
	case MSM_ISP_STATS_HDR_BE:
		return STATS_IDX_HDR_BE;
	case MSM_ISP_STATS_BG:
		return STATS_IDX_BG;
	case MSM_ISP_STATS_BF:
		return STATS_IDX_BF;
	case MSM_ISP_STATS_HDR_BHIST:
		return STATS_IDX_HDR_BHIST;
	case MSM_ISP_STATS_RS:
		return STATS_IDX_RS;
	case MSM_ISP_STATS_CS:
		return STATS_IDX_CS;
	case MSM_ISP_STATS_IHIST:
		return STATS_IDX_IHIST;
	case MSM_ISP_STATS_BHIST:
		return STATS_IDX_BHIST;
	case MSM_ISP_STATS_BF_SCALE:
		return STATS_IDX_BF_SCALE;
	default:
		pr_err("%s: Invalid stats type\n", __func__);
		return -EINVAL;
	}
}

int msm_vfe47_stats_check_streams(
	struct msm_vfe_stats_stream *stream_info)
{
	if (stream_info[STATS_IDX_BF].state ==
		STATS_AVALIABLE &&
		stream_info[STATS_IDX_BF_SCALE].state !=
		STATS_AVALIABLE) {
		pr_err("%s: does not support BF_SCALE while BF is disabled\n",
			__func__);
		return -EINVAL;
	}
	if (stream_info[STATS_IDX_BF].state != STATS_AVALIABLE &&
		stream_info[STATS_IDX_BF_SCALE].state != STATS_AVALIABLE &&
		stream_info[STATS_IDX_BF].composite_flag !=
		stream_info[STATS_IDX_BF_SCALE].composite_flag) {
		pr_err("%s: Different composite flag for BF and BF_SCALE\n",
			__func__);
		return -EINVAL;
	}
	return 0;
}

<<<<<<< HEAD
static void msm_vfe47_stats_cfg_comp_mask(
	struct vfe_device *vfe_dev,
	uint32_t stats_mask, uint8_t enable)
=======
void msm_vfe47_stats_cfg_comp_mask(
	struct vfe_device *vfe_dev, uint32_t stats_mask,
	uint8_t request_comp_index, uint8_t enable)
>>>>>>> 0e91d2a... Nougat
{
	uint32_t reg_mask, comp_stats_mask, mask_bf_scale;
	uint32_t i = 0;
	atomic_t *stats_comp;
	struct msm_vfe_stats_shared_data *stats_data = &vfe_dev->stats_data;


	if (vfe_dev->hw_info->stats_hw_info->num_stats_comp_mask < 1)
		
		return;

	if (vfe_dev->hw_info->stats_hw_info->num_stats_comp_mask >
			MAX_NUM_STATS_COMP_MASK) {
		pr_err("%s: num of comp masks %d exceed max %d\n",
			__func__,
			vfe_dev->hw_info->stats_hw_info->num_stats_comp_mask,
			MAX_NUM_STATS_COMP_MASK);
		return;
	}

	
	stats_mask = stats_mask & 0x1FF;
	mask_bf_scale = stats_mask >> SHIFT_BF_SCALE_BIT;

	for (i = 0;
		i < vfe_dev->hw_info->stats_hw_info->num_stats_comp_mask; i++) {
		stats_comp = &stats_data->stats_comp_mask[i];
		reg_mask = msm_camera_io_r(vfe_dev->vfe_base + 0x78);
		comp_stats_mask = reg_mask & (STATS_COMP_BIT_MASK << (i*8));

<<<<<<< HEAD
		if (enable) {
			if (comp_stats_mask)
				continue;

			reg_mask |= (mask_bf_scale << (16 + i*8));
			atomic_set(stats_comp, stats_mask |
					atomic_read(stats_comp));
			break;
=======
	if (enable) {
		comp_mask_reg |= stats_mask << (request_comp_index * 16);
		atomic_set(stats_comp_mask, stats_mask |
				atomic_read(stats_comp_mask));
		msm_vfe47_config_irq(vfe_dev, 1 << (29 + request_comp_index),
				0, MSM_ISP_IRQ_ENABLE);
	} else {
		if (!(atomic_read(stats_comp_mask) & stats_mask))
			return;

		atomic_set(stats_comp_mask,
				~stats_mask & atomic_read(stats_comp_mask));
		comp_mask_reg &= ~(stats_mask << (request_comp_index * 16));
		msm_vfe47_config_irq(vfe_dev, 1 << (29 + request_comp_index),
				0, MSM_ISP_IRQ_DISABLE);
	}
>>>>>>> 0e91d2a... Nougat

		} else {

			if (!(atomic_read(stats_comp) & stats_mask))
				continue;
			if (stats_mask & (1 << STATS_IDX_BF_SCALE) &&
				atomic_read(stats_comp) &
					(1 << STATS_IDX_BF_SCALE))
				atomic_set(stats_comp,
						~(1 << STATS_IDX_BF_SCALE) &
						atomic_read(stats_comp));

			atomic_set(stats_comp,
					~stats_mask & atomic_read(stats_comp));
			reg_mask &= ~(mask_bf_scale << (16 + i*8));
			break;
		}
	}

	ISP_DBG("%s: comp_mask: %x atomic stats[0]: %x %x\n",
		__func__, reg_mask,
		atomic_read(&stats_data->stats_comp_mask[0]),
		atomic_read(&stats_data->stats_comp_mask[1]));

	msm_camera_io_w(reg_mask, vfe_dev->vfe_base + 0x78);
	return;
}

void msm_vfe47_stats_cfg_wm_irq_mask(
	struct vfe_device *vfe_dev,
	struct msm_vfe_stats_stream *stream_info)
{
<<<<<<< HEAD
	uint32_t irq_mask;

	irq_mask = msm_camera_io_r(vfe_dev->vfe_base + 0x5C);
	irq_mask |= 1 << (STATS_IDX(stream_info->stream_handle) + 15);
	msm_camera_io_w(irq_mask, vfe_dev->vfe_base + 0x5C);
=======
	switch (STATS_IDX(stream_info->stream_handle)) {
	case STATS_COMP_IDX_AEC_BG:
		msm_vfe47_config_irq(vfe_dev, 1 << 15, 0, MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_HDR_BE:
		msm_vfe47_config_irq(vfe_dev, 1 << 16, 0, MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_BG:
		msm_vfe47_config_irq(vfe_dev, 1 << 17, 0, MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_BF:
		msm_vfe47_config_irq(vfe_dev, 1 << 18, 1 << 26,
				MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_HDR_BHIST:
		msm_vfe47_config_irq(vfe_dev, 1 << 19, 0, MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_RS:
		msm_vfe47_config_irq(vfe_dev, 1 << 20, 0, MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_CS:
		msm_vfe47_config_irq(vfe_dev, 1 << 21, 0, MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_IHIST:
		msm_vfe47_config_irq(vfe_dev, 1 << 22, 0, MSM_ISP_IRQ_ENABLE);
		break;
	case STATS_COMP_IDX_BHIST:
		msm_vfe47_config_irq(vfe_dev, 1 << 23, 0, MSM_ISP_IRQ_ENABLE);
		break;
	default:
		pr_err("%s: Invalid stats idx %d\n", __func__,
			STATS_IDX(stream_info->stream_handle));
	}
>>>>>>> 0e91d2a... Nougat
}

void msm_vfe47_stats_clear_wm_irq_mask(
	struct vfe_device *vfe_dev,
	struct msm_vfe_stats_stream *stream_info)
{
	uint32_t irq_mask;

<<<<<<< HEAD
	irq_mask = msm_camera_io_r(vfe_dev->vfe_base + 0x5C);
	irq_mask &= ~(1 << (STATS_IDX(stream_info->stream_handle) + 15));
	msm_camera_io_w(irq_mask, vfe_dev->vfe_base + 0x5C);
=======
	irq_mask = vfe_dev->irq0_mask;
	irq_mask_1 = vfe_dev->irq1_mask;

	switch (STATS_IDX(stream_info->stream_handle)) {
	case STATS_COMP_IDX_AEC_BG:
		msm_vfe47_config_irq(vfe_dev, 1 << 15, 0, MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_HDR_BE:
		msm_vfe47_config_irq(vfe_dev, 1 << 16, 0, MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_BG:
		msm_vfe47_config_irq(vfe_dev, 1 << 17, 0, MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_BF:
		msm_vfe47_config_irq(vfe_dev, 1 << 18, 1 << 26,
				MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_HDR_BHIST:
		msm_vfe47_config_irq(vfe_dev, 1 << 19, 0, MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_RS:
		msm_vfe47_config_irq(vfe_dev, 1 << 20, 0, MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_CS:
		msm_vfe47_config_irq(vfe_dev, 1 << 21, 0, MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_IHIST:
		msm_vfe47_config_irq(vfe_dev, 1 << 22, 0, MSM_ISP_IRQ_DISABLE);
		break;
	case STATS_COMP_IDX_BHIST:
		msm_vfe47_config_irq(vfe_dev, 1 << 23, 0, MSM_ISP_IRQ_DISABLE);
		break;
	default:
		pr_err("%s: Invalid stats idx %d\n", __func__,
			STATS_IDX(stream_info->stream_handle));
	}
>>>>>>> 0e91d2a... Nougat
}

void msm_vfe47_stats_cfg_wm_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_stats_stream *stream_info)
{
	int stats_idx = STATS_IDX(stream_info->stream_handle);
	uint32_t stats_base = VFE47_STATS_BASE(stats_idx);

<<<<<<< HEAD
	if (stats_idx == STATS_IDX_BF_SCALE)
		return;

	
	msm_camera_io_w(stream_info->framedrop_period << 2,
		vfe_dev->vfe_base + stats_base + 0x8);
	
	msm_camera_io_w(stream_info->framedrop_pattern,
		vfe_dev->vfe_base + stats_base + 0x10);
=======
	
	msm_camera_io_w(stream_info->framedrop_period << 2,
		vfe_dev->vfe_base + stats_base + 0x10);
	
	msm_camera_io_w(stream_info->framedrop_pattern,
		vfe_dev->vfe_base + stats_base + 0x18);
>>>>>>> 0e91d2a... Nougat
	
	msm_camera_io_w(0xFFFFFFFF,
		vfe_dev->vfe_base + stats_base + 0x14);
}

void msm_vfe47_stats_clear_wm_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_stats_stream *stream_info)
{
	uint32_t val = 0;
	int stats_idx = STATS_IDX(stream_info->stream_handle);
	uint32_t stats_base = VFE47_STATS_BASE(stats_idx);

<<<<<<< HEAD
	if (stats_idx == STATS_IDX_BF_SCALE)
		return;

	
	msm_camera_io_w(val, vfe_dev->vfe_base + stats_base + 0x8);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + stats_base + 0x10);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + stats_base + 0x14);
=======
	
	msm_camera_io_w(val, vfe_dev->vfe_base + stats_base + 0x10);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + stats_base + 0x18);
	
	msm_camera_io_w(val, vfe_dev->vfe_base + stats_base + 0x1C);
>>>>>>> 0e91d2a... Nougat
}

void msm_vfe47_stats_cfg_ub(struct vfe_device *vfe_dev)
{
	int i;
	uint32_t ub_offset = 0;
	uint32_t ub_size[VFE47_NUM_STATS_TYPE] = {
		16, 
		16, 
		16, 
		16, 
		16, 
		16, 
		16, 
		16, 
		16, 
	};
	if (vfe_dev->pdev->id == ISP_VFE1) {
		ub_offset = VFE47_UB_SIZE_VFE1;
	} else if (vfe_dev->pdev->id == ISP_VFE0) {
		ub_offset = VFE47_UB_SIZE_VFE0;
	} else {
		pr_err("%s: incorrect VFE device\n", __func__);
	}

	for (i = 0; i < VFE47_NUM_STATS_TYPE; i++) {
		ub_offset -= ub_size[i];
		msm_camera_io_w(VFE47_STATS_BURST_LEN << 30 |
			ub_offset << 16 | (ub_size[i] - 1),
			vfe_dev->vfe_base + VFE47_STATS_BASE(i) +
			((i == STATS_IDX_BF_SCALE) ? 0x8 : 0xC));
	}
}

void msm_vfe47_stats_update_cgc_override(struct vfe_device *vfe_dev,
	uint32_t stats_mask, uint8_t enable)
{
	int i;
	uint32_t module_cfg, cgc_mask = 0;

	for (i = 0; i < VFE47_NUM_STATS_TYPE; i++) {
		if ((stats_mask >> i) & 0x1) {
			switch (i) {
			case STATS_IDX_HDR_BE:
				cgc_mask |= 1;
				break;
			case STATS_IDX_HDR_BHIST:
				cgc_mask |= (1 << 1);
				break;
			case STATS_IDX_BF:
				cgc_mask |= (1 << 2);
				break;
			case STATS_IDX_BG:
				cgc_mask |= (1 << 3);
				break;
			case STATS_IDX_BHIST:
				cgc_mask |= (1 << 4);
				break;
			case STATS_IDX_RS:
				cgc_mask |= (1 << 5);
				break;
			case STATS_IDX_CS:
				cgc_mask |= (1 << 6);
				break;
			case STATS_IDX_IHIST:
				cgc_mask |= (1 << 7);
				break;
			case STATS_IDX_BF_SCALE:
				cgc_mask |= (1 << 2);
				break;
			default:
				pr_err("%s: Invalid stats mask\n", __func__);
				return;
			}
		}
	}

	
	module_cfg = msm_camera_io_r(vfe_dev->vfe_base + 0x30);
	if (enable)
		module_cfg |= cgc_mask;
	else
		module_cfg &= ~cgc_mask;
	msm_camera_io_w(module_cfg, vfe_dev->vfe_base + 0x30);
}

<<<<<<< HEAD
static void msm_vfe47_stats_enable_module(struct vfe_device *vfe_dev,
=======
bool msm_vfe47_is_module_cfg_lock_needed(
	uint32_t reg_offset)
{
	return false;
}

void msm_vfe47_stats_enable_module(struct vfe_device *vfe_dev,
>>>>>>> 0e91d2a... Nougat
	uint32_t stats_mask, uint8_t enable)
{
	int i;
	uint32_t module_cfg, module_cfg_mask = 0;
	uint32_t stats_cfg, stats_cfg_mask = 0;

<<<<<<< HEAD
=======
	
>>>>>>> 0e91d2a... Nougat
	for (i = 0; i < VFE47_NUM_STATS_TYPE; i++) {
		if ((stats_mask >> i) & 0x1) {
			switch (i) {
			case STATS_IDX_HDR_BE:
				module_cfg_mask |= 1;
				break;
			case STATS_IDX_HDR_BHIST:
				module_cfg_mask |= 1 << 1;
				break;
			case STATS_IDX_BF:
				module_cfg_mask |= 1 << 2;
				break;
			case STATS_IDX_BG:
				module_cfg_mask |= 1 << 3;
				break;
			case STATS_IDX_BHIST:
				module_cfg_mask |= 1 << 4;
				break;
			case STATS_IDX_RS:
				module_cfg_mask |= 1 << 5;
				break;
			case STATS_IDX_CS:
				module_cfg_mask |= 1 << 6;
				break;
			case STATS_IDX_IHIST:
				module_cfg_mask |= 1 << 7;
				break;
			case STATS_IDX_BF_SCALE:
				stats_cfg_mask |= 1 << 5;
				break;
			default:
				pr_err("%s: Invalid stats mask\n", __func__);
				return;
			}
		}
	}

	module_cfg = msm_camera_io_r(vfe_dev->vfe_base + 0x44);
	if (enable)
		module_cfg |= module_cfg_mask;
	else
		module_cfg &= ~module_cfg_mask;
	msm_camera_io_w(module_cfg, vfe_dev->vfe_base + 0x44);
<<<<<<< HEAD

	stats_cfg = msm_camera_io_r(vfe_dev->vfe_base + 0x9B8);
	if (enable)
		stats_cfg |= stats_cfg_mask;
	else
		stats_cfg &= ~stats_cfg_mask;
	msm_camera_io_w(stats_cfg, vfe_dev->vfe_base + 0x9B8);
}

static void msm_vfe47_stats_update_ping_pong_addr(
	struct vfe_device *vfe_dev, struct msm_vfe_stats_stream *stream_info,
=======
	
	if (vfe_dev->hw_info->vfe_ops.stats_ops.enable_stats_wm)
		vfe_dev->hw_info->vfe_ops.stats_ops.enable_stats_wm(vfe_dev,
						stats_mask, enable);
}

void msm_vfe47_stats_update_ping_pong_addr(
	void __iomem *vfe_base, struct msm_vfe_stats_stream *stream_info,
>>>>>>> 0e91d2a... Nougat
	uint32_t pingpong_status, dma_addr_t paddr)
{
	uint32_t paddr32 = (paddr & 0xFFFFFFFF);
	int stats_idx = STATS_IDX(stream_info->stream_handle);

	msm_camera_io_w(paddr32, vfe_dev->vfe_base +
		VFE47_STATS_PING_PONG_BASE(stats_idx, pingpong_status));
}

uint32_t msm_vfe47_stats_get_wm_mask(
	uint32_t irq_status0, uint32_t irq_status1)
{
<<<<<<< HEAD
	return (irq_status0 >> 15) & 0x1FF;
=======
	uint32_t comp_mapped_irq_mask = 0;
	int i = 0;

	irq_status0 &= ~(1 << 18);

	for (i = 0; i < VFE47_NUM_STATS_TYPE; i++)
		if ((irq_status0 >> stats_irq_map_comp_mask[i]) & 0x1)
			comp_mapped_irq_mask |= (1 << i);
	if ((irq_status1 >> 26) & 0x1)
		comp_mapped_irq_mask |= (1 << STATS_COMP_IDX_BF);

	return comp_mapped_irq_mask;
>>>>>>> 0e91d2a... Nougat
}

uint32_t msm_vfe47_stats_get_comp_mask(
	uint32_t irq_status0, uint32_t irq_status1)
{
	return (irq_status0 >> 29) & 0x3;
}

uint32_t msm_vfe47_stats_get_frame_id(
	struct vfe_device *vfe_dev)
{
	return vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id;
}

void msm_vfe47_deinit_bandwidth_mgr(
		struct msm_isp_bandwidth_mgr *isp_bandwidth_mgr)
{
	msm_bus_scale_client_update_request(
	   isp_bandwidth_mgr->bus_client, 0);
	msm_bus_scale_unregister_client(isp_bandwidth_mgr->bus_client);
	isp_bandwidth_mgr->bus_client = 0;
}

int msm_vfe47_init_bandwidth_mgr(struct vfe_device *vfe_dev,
	struct msm_isp_bandwidth_mgr *isp_bandwidth_mgr)
{
	isp_bandwidth_mgr->bus_client =
		msm_bus_scale_register_client(&msm_isp_bus_client_pdata);
	if (!isp_bandwidth_mgr->bus_client) {
		pr_err("%s: client register failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

int msm_vfe47_update_bandwidth(
		struct msm_isp_bandwidth_mgr *isp_bandwidth_mgr)
{
	int i;
	uint64_t ab = 0;
	uint64_t ib = 0;
	struct msm_bus_paths *path;

	if (!isp_bandwidth_mgr->bus_vector_active_idx)
		isp_bandwidth_mgr->bus_vector_active_idx = 1;
	else
		ALT_VECTOR_IDX(isp_bandwidth_mgr->bus_vector_active_idx);

	path = &(msm_isp_bus_client_pdata.usecase[
			isp_bandwidth_mgr->bus_vector_active_idx]);
	path->vectors[0].ab = 0;
	path->vectors[0].ib = 0;
	for (i = 0; i < MAX_ISP_CLIENT; i++) {
		if (isp_bandwidth_mgr->client_info[i].active) {
			path->vectors[0].ab +=
				isp_bandwidth_mgr->client_info[i].ab;
			path->vectors[0].ib +=
				isp_bandwidth_mgr->client_info[i].ib;
			ab += isp_bandwidth_mgr->client_info[i].ab;
			ib += isp_bandwidth_mgr->client_info[i].ib;
		}
	}
	msm_bus_scale_client_update_request(isp_bandwidth_mgr->bus_client,
		isp_bandwidth_mgr->bus_vector_active_idx);
	
	msm_isp_update_req_history(isp_bandwidth_mgr->bus_client,
		ab, ib,
		isp_bandwidth_mgr->client_info,
		sched_clock());
	return 0;
}

int msm_vfe47_get_clks(struct vfe_device *vfe_dev)
{
	int i, rc;

	rc = msm_camera_get_clk_info(vfe_dev->pdev, &vfe_dev->vfe_clk_info,
			&vfe_dev->vfe_clk, &vfe_dev->num_clk);
	if (rc)
		return rc;

	for (i = 0; i < vfe_dev->num_clk; i++) {
		if (0 == strcmp(vfe_dev->vfe_clk_info[i].clk_name,
					"vfe_clk_src"))
			vfe_dev->hw_info->vfe_clk_idx = i;
	}
	return 0;
}

void msm_vfe47_put_clks(struct vfe_device *vfe_dev)
{
	msm_camera_put_clk_info(vfe_dev->pdev, &vfe_dev->vfe_clk_info,
			&vfe_dev->vfe_clk, vfe_dev->num_clk);

	vfe_dev->num_clk = 0;
}

int msm_vfe47_enable_clks(struct vfe_device *vfe_dev, int enable)
{
	return msm_camera_clk_enable(&vfe_dev->pdev->dev,
			vfe_dev->vfe_clk_info,
			vfe_dev->vfe_clk, vfe_dev->num_clk, enable);
}

int msm_vfe47_set_clk_rate(struct vfe_device *vfe_dev, long *rate)
{
	int rc = 0;
	int clk_idx = vfe_dev->hw_info->vfe_clk_idx;

	rc = msm_camera_clk_set_rate(&vfe_dev->pdev->dev,
				vfe_dev->vfe_clk[clk_idx], *rate);
	if (rc < 0)
		return rc;
	*rate = clk_round_rate(vfe_dev->vfe_clk[clk_idx], *rate);
	vfe_dev->msm_isp_vfe_clk_rate = *rate;
	return 0;
}

int msm_vfe47_get_max_clk_rate(struct vfe_device *vfe_dev, long *rate)
{
	int	   clk_idx = 0;
	unsigned long max_value = ~0;
	long	  round_rate = 0;

	if (!vfe_dev || !rate) {
		pr_err("%s:%d failed: vfe_dev %pK rate %pK\n",
			__func__, __LINE__, vfe_dev, rate);
		return -EINVAL;
	}

	*rate = 0;
	if (!vfe_dev->hw_info) {
		pr_err("%s:%d failed: vfe_dev->hw_info %pK\n", __func__,
			__LINE__, vfe_dev->hw_info);
		return -EINVAL;
	}

	clk_idx = vfe_dev->hw_info->vfe_clk_idx;
	if (clk_idx >= vfe_dev->num_clk) {
		pr_err("%s:%d failed: clk_idx %d max array size %zd\n",
			__func__, __LINE__, clk_idx,
			vfe_dev->num_clk);
		return -EINVAL;
	}

	round_rate = clk_round_rate(vfe_dev->vfe_clk[clk_idx], max_value);
	if (round_rate < 0) {
		pr_err("%s: Invalid vfe clock rate\n", __func__);
		return -EINVAL;
	}

	*rate = round_rate;
	return 0;
}

int msm_vfe47_get_clk_rates(struct vfe_device *vfe_dev,
	struct msm_isp_clk_rates *rates)
{
	struct device_node *of_node;
	int32_t  rc = 0;
	uint32_t svs = 0, nominal = 0, turbo = 0;

	if (!vfe_dev || !rates) {
		pr_err("%s:%d failed: vfe_dev %pK rates %pK\n", __func__,
			__LINE__, vfe_dev, rates);
		return -EINVAL;
	}

	if (!vfe_dev->pdev) {
		pr_err("%s:%d failed: vfe_dev->pdev %pK\n", __func__,
			__LINE__, vfe_dev->pdev);
		return -EINVAL;
	}

	of_node = vfe_dev->pdev->dev.of_node;

	if (!of_node) {
		pr_err("%s %d failed: of_node = %pK\n", __func__,
		 __LINE__, of_node);
		return -EINVAL;
	}

	rc = of_property_read_u32(of_node, "max-clk-svs",
		&svs);
	if (rc < 0)
		svs = 0;

	rc = of_property_read_u32(of_node, "max-clk-nominal",
		&nominal);
	if (rc < 0 || !nominal) {
		pr_err("%s: nominal rate error\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(of_node, "max-clk-turbo",
		&turbo);
	if (rc < 0 || !turbo) {
		pr_err("%s: turbo rate error\n", __func__);
		return -EINVAL;
	}
	rates->svs_rate = svs;
	rates->nominal_rate = nominal;
	rates->high_rate = turbo;
	return 0;
}

void msm_vfe47_put_regulators(struct vfe_device *vfe_dev)
{
	int i;

	for (i = 0; i < vfe_dev->vfe_num_regulators; i++)
		regulator_put(vfe_dev->regulator_info[i].vdd);

	vfe_dev->vfe_num_regulators = 0;
	kfree(vfe_dev->regulator_info);
	vfe_dev->regulator_info = NULL;
}

int msm_vfe47_get_regulators(struct vfe_device *vfe_dev)
{
	int rc = 0;
	int i;

	vfe_dev->vfe_num_regulators =
		sizeof(*vfe_dev->hw_info->regulator_names) / sizeof(char *);

	vfe_dev->regulator_info = kzalloc(sizeof(struct msm_cam_regulator) *
				vfe_dev->vfe_num_regulators, GFP_KERNEL);
	if (!vfe_dev->regulator_info)
		return -ENOMEM;

	for (i = 0; i < vfe_dev->vfe_num_regulators; i++) {
		vfe_dev->regulator_info[i].vdd = regulator_get(
					&vfe_dev->pdev->dev,
					vfe_dev->hw_info->regulator_names[i]);
		if (IS_ERR(vfe_dev->regulator_info[i].vdd)) {
			pr_err("%s: Regulator vfe get failed %ld\n", __func__,
			PTR_ERR(vfe_dev->regulator_info[i].vdd));
			rc = -ENODEV;
			goto reg_get_fail;
		}
	}
	return 0;

reg_get_fail:
	for (i--; i >= 0; i--)
		regulator_put(vfe_dev->regulator_info[i].vdd);
	kfree(vfe_dev->regulator_info);
	vfe_dev->regulator_info = NULL;
	return rc;
}

int msm_vfe47_enable_regulators(struct vfe_device *vfe_dev, int enable)
{
	return msm_camera_regulator_enable(vfe_dev->regulator_info,
					vfe_dev->vfe_num_regulators, enable);
}

int msm_vfe47_get_platform_data(struct vfe_device *vfe_dev)
{
	int rc = 0;

	vfe_dev->vfe_base = msm_camera_get_reg_base(vfe_dev->pdev, "vfe", 0);
	if (!vfe_dev->vfe_base)
		return -ENOMEM;
	vfe_dev->vfe_vbif_base = msm_camera_get_reg_base(vfe_dev->pdev,
					"vfe_vbif", 0);
	if (!vfe_dev->vfe_vbif_base) {
		rc = -ENOMEM;
		goto vbif_base_fail;
	}

	vfe_dev->vfe_irq = msm_camera_get_irq(vfe_dev->pdev, "vfe");
	if (!vfe_dev->vfe_irq) {
		rc = -ENODEV;
		goto vfe_irq_fail;
	}

<<<<<<< HEAD
	vfe_dev->fs_vfe = regulator_get(&vfe_dev->pdev->dev, "vdd");
	if (IS_ERR(vfe_dev->fs_vfe)) {
		pr_err("%s: Regulator get failed %ld\n", __func__,
		PTR_ERR(vfe_dev->fs_vfe));
		vfe_dev->fs_vfe = NULL;
		rc = -ENODEV;
		goto vfe_no_resource;
	}

	vfe_dev->iommu_ctx[0] = msm_iommu_get_ctx("vfe");
	if (!vfe_dev->iommu_ctx[0]) {
		pr_err("%s: cannot get iommu_ctx\n", __func__);
		rc = -ENODEV;
		goto vfe_no_resource;
	}
	if (vfe_dev->hw_info->num_iommu_secure_ctx) {
		vfe_dev->iommu_secure_ctx[0] = msm_iommu_get_ctx("vfe_secure");
		if (!vfe_dev->iommu_secure_ctx[0]) {
			pr_err("%s: cannot get secure iommu_ctx\n", __func__);
			rc = -ENODEV;
			goto vfe_no_resource;
		}
	}

vfe_no_resource:
=======
	vfe_dev->vfe_base_size = msm_camera_get_res_size(vfe_dev->pdev, "vfe");
	vfe_dev->vfe_vbif_base_size = msm_camera_get_res_size(vfe_dev->pdev,
						"vfe_vbif");
	if (!vfe_dev->vfe_base_size || !vfe_dev->vfe_vbif_base_size) {
		rc = -ENOMEM;
		goto get_res_fail;
	}

	rc = vfe_dev->hw_info->vfe_ops.platform_ops.get_regulators(vfe_dev);
	if (rc)
		goto get_regulator_fail;

	rc = vfe_dev->hw_info->vfe_ops.platform_ops.get_clks(vfe_dev);
	if (rc)
		goto get_clkcs_fail;

	rc = msm_camera_register_irq(vfe_dev->pdev, vfe_dev->vfe_irq,
		msm_isp_process_irq,
		IRQF_TRIGGER_RISING, "vfe", vfe_dev);
	if (rc < 0)
		goto irq_register_fail;

	msm_camera_enable_irq(vfe_dev->vfe_irq, 0);

	rc = msm_isp_init_bandwidth_mgr(vfe_dev, ISP_VFE0 + vfe_dev->pdev->id);
	if (rc)
		goto init_bw_fail;

	return 0;

init_bw_fail:
	msm_camera_unregister_irq(vfe_dev->pdev, vfe_dev->vfe_irq, "vfe");
irq_register_fail:
	vfe_dev->hw_info->vfe_ops.platform_ops.put_clks(vfe_dev);
get_clkcs_fail:
	vfe_dev->hw_info->vfe_ops.platform_ops.put_regulators(vfe_dev);
get_regulator_fail:
get_res_fail:
	vfe_dev->vfe_vbif_base_size = 0;
	vfe_dev->vfe_base_size = 0;
vfe_irq_fail:
	msm_camera_put_reg_base(vfe_dev->pdev, vfe_dev->vfe_base,
					"vfe_vbif", 0);
vbif_base_fail:
	msm_camera_put_reg_base(vfe_dev->pdev, vfe_dev->vfe_base, "vfe", 0);
>>>>>>> 0e91d2a... Nougat
	return rc;
}

void msm_vfe47_get_error_mask(
	uint32_t *error_mask0, uint32_t *error_mask1)
{
	*error_mask0 = 0x00000000;
	*error_mask1 = 0x01FFFEFF;
}

void msm_vfe47_get_overflow_mask(uint32_t *overflow_mask)
{
	*overflow_mask = 0x01FFFE7E;
}

void msm_vfe47_get_rdi_wm_mask(struct vfe_device *vfe_dev,
	uint32_t *rdi_wm_mask)
{
	*rdi_wm_mask = vfe_dev->axi_data.rdi_wm_mask;
}

void msm_vfe47_get_irq_mask(struct vfe_device *vfe_dev,
	uint32_t *irq0_mask, uint32_t *irq1_mask)
{
	*irq0_mask = vfe_dev->irq0_mask;
	*irq1_mask = vfe_dev->irq1_mask;
}

void msm_vfe47_get_halt_restart_mask(uint32_t *irq0_mask,
	uint32_t *irq1_mask)
{
	*irq0_mask = BIT(31);
	*irq1_mask = BIT(8);
}

static struct msm_vfe_axi_hardware_info msm_vfe47_axi_hw_info = {
	.num_wm = 7,
	.num_comp_mask = 3,
	.num_rdi = 3,
	.num_rdi_master = 3,
	.min_wm_ub = 96,
<<<<<<< HEAD
=======
	.scratch_buf_range = SZ_32M + SZ_4M,
>>>>>>> 0e91d2a... Nougat
};

static struct msm_vfe_stats_hardware_info msm_vfe47_stats_hw_info = {
	.stats_capability_mask =
		1 << MSM_ISP_STATS_HDR_BE    | 1 << MSM_ISP_STATS_BF    |
		1 << MSM_ISP_STATS_BG        | 1 << MSM_ISP_STATS_BHIST |
		1 << MSM_ISP_STATS_HDR_BHIST | 1 << MSM_ISP_STATS_IHIST |
		1 << MSM_ISP_STATS_RS        | 1 << MSM_ISP_STATS_CS    |
		1 << MSM_ISP_STATS_BF_SCALE,
	.stats_ping_pong_offset = stats_pingpong_offset_map,
	.num_stats_type = VFE47_NUM_STATS_TYPE,
	.num_stats_comp_mask = VFE47_NUM_STATS_COMP,
};

static struct v4l2_subdev_core_ops msm_vfe47_subdev_core_ops = {
	.ioctl = msm_isp_ioctl,
	.subscribe_event = msm_isp_subscribe_event,
	.unsubscribe_event = msm_isp_unsubscribe_event,
};

static struct v4l2_subdev_ops msm_vfe47_subdev_ops = {
	.core = &msm_vfe47_subdev_core_ops,
};

static struct v4l2_subdev_internal_ops msm_vfe47_internal_ops = {
	.open = msm_isp_open_node,
	.close = msm_isp_close_node,
};

struct msm_vfe_hardware_info vfe47_hw_info = {
	.num_iommu_ctx = 1,
<<<<<<< HEAD
	.num_iommu_secure_ctx = 1,
	.vfe_clk_idx = VFE47_CLK_IDX,
=======
	.num_iommu_secure_ctx = 0,
	.vfe_clk_idx = VFE47_SRC_CLK_DTSI_IDX,
	.runtime_axi_update = 1,
	.min_ib = 100000000,
	.min_ab = 100000000,
>>>>>>> 0e91d2a... Nougat
	.vfe_ops = {
		.irq_ops = {
			.read_irq_status = msm_vfe47_read_irq_status,
			.process_camif_irq = msm_vfe47_process_input_irq,
			.process_reset_irq = msm_vfe47_process_reset_irq,
			.process_halt_irq = msm_vfe47_process_halt_irq,
			.process_reset_irq = msm_vfe47_process_reset_irq,
			.process_reg_update = msm_vfe47_process_reg_update,
			.process_axi_irq = msm_isp_process_axi_irq,
			.process_stats_irq = msm_isp_process_stats_irq,
			.process_epoch_irq = msm_vfe47_process_epoch_irq,
<<<<<<< HEAD
=======
			.config_irq = msm_vfe47_config_irq,
>>>>>>> 0e91d2a... Nougat
		},
		.axi_ops = {
			.reload_wm = msm_vfe47_axi_reload_wm,
			.enable_wm = msm_vfe47_axi_enable_wm,
			.cfg_io_format = msm_vfe47_cfg_io_format,
			.cfg_comp_mask = msm_vfe47_axi_cfg_comp_mask,
			.clear_comp_mask = msm_vfe47_axi_clear_comp_mask,
			.cfg_wm_irq_mask = msm_vfe47_axi_cfg_wm_irq_mask,
			.clear_wm_irq_mask = msm_vfe47_axi_clear_wm_irq_mask,
			.cfg_framedrop = msm_vfe47_cfg_framedrop,
			.clear_framedrop = msm_vfe47_clear_framedrop,
			.cfg_wm_reg = msm_vfe47_axi_cfg_wm_reg,
			.clear_wm_reg = msm_vfe47_axi_clear_wm_reg,
			.cfg_wm_xbar_reg = msm_vfe47_axi_cfg_wm_xbar_reg,
			.clear_wm_xbar_reg = msm_vfe47_axi_clear_wm_xbar_reg,
			.cfg_ub = msm_vfe47_cfg_axi_ub,
			.update_ping_pong_addr =
				msm_vfe47_update_ping_pong_addr,
			.get_comp_mask = msm_vfe47_get_comp_mask,
			.get_wm_mask = msm_vfe47_get_wm_mask,
			.get_pingpong_status = msm_vfe47_get_pingpong_status,
			.halt = msm_vfe47_axi_halt,
			.restart = msm_vfe47_axi_restart,
			.update_cgc_override =
				msm_vfe47_axi_update_cgc_override,
		},
		.core_ops = {
			.reg_update = msm_vfe47_reg_update,
			.cfg_input_mux = msm_vfe47_cfg_input_mux,
			.update_camif_state = msm_vfe47_update_camif_state,
			.start_fetch_eng = msm_vfe47_start_fetch_engine,
			.cfg_rdi_reg = msm_vfe47_cfg_rdi_reg,
			.reset_hw = msm_vfe47_reset_hardware,
			.init_hw = msm_vfe47_init_hardware,
			.init_hw_reg = msm_vfe47_init_hardware_reg,
			.clear_status_reg = msm_vfe47_clear_status_reg,
			.release_hw = msm_vfe47_release_hardware,
			.get_error_mask = msm_vfe47_get_error_mask,
			.get_overflow_mask = msm_vfe47_get_overflow_mask,
			.get_rdi_wm_mask = msm_vfe47_get_rdi_wm_mask,
			.get_irq_mask = msm_vfe47_get_irq_mask,
			.get_halt_restart_mask =
				msm_vfe47_get_halt_restart_mask,
			.process_error_status = msm_vfe47_process_error_status,
<<<<<<< HEAD
=======
			.is_module_cfg_lock_needed =
				msm_vfe47_is_module_cfg_lock_needed,
			.ahb_clk_cfg = msm_isp47_ahb_clk_cfg,
			.set_halt_restart_mask =
				msm_vfe47_set_halt_restart_mask,
			.start_fetch_eng_multi_pass =
				msm_vfe47_start_fetch_engine_multi_pass,
>>>>>>> 0e91d2a... Nougat
		},
		.stats_ops = {
			.get_stats_idx = msm_vfe47_get_stats_idx,
			.check_streams = msm_vfe47_stats_check_streams,
			.cfg_comp_mask = msm_vfe47_stats_cfg_comp_mask,
			.cfg_wm_irq_mask = msm_vfe47_stats_cfg_wm_irq_mask,
			.clear_wm_irq_mask = msm_vfe47_stats_clear_wm_irq_mask,
			.cfg_wm_reg = msm_vfe47_stats_cfg_wm_reg,
			.clear_wm_reg = msm_vfe47_stats_clear_wm_reg,
			.cfg_ub = msm_vfe47_stats_cfg_ub,
			.enable_module = msm_vfe47_stats_enable_module,
			.update_ping_pong_addr =
				msm_vfe47_stats_update_ping_pong_addr,
			.get_comp_mask = msm_vfe47_stats_get_comp_mask,
			.get_wm_mask = msm_vfe47_stats_get_wm_mask,
			.get_frame_id = msm_vfe47_stats_get_frame_id,
			.get_pingpong_status = msm_vfe47_get_pingpong_status,
			.update_cgc_override =
				msm_vfe47_stats_update_cgc_override,
			.enable_stats_wm = NULL,
		},
		.platform_ops = {
			.get_platform_data = msm_vfe47_get_platform_data,
			.enable_regulators = msm_vfe47_enable_regulators,
			.get_regulators = msm_vfe47_get_regulators,
			.put_regulators = msm_vfe47_put_regulators,
			.enable_clks = msm_vfe47_enable_clks,
			.get_clks = msm_vfe47_get_clks,
			.put_clks = msm_vfe47_put_clks,
			.get_clk_rates = msm_vfe47_get_clk_rates,
			.get_max_clk_rate = msm_vfe47_get_max_clk_rate,
			.set_clk_rate = msm_vfe47_set_clk_rate,
			.init_bw_mgr = msm_vfe47_init_bandwidth_mgr,
			.deinit_bw_mgr = msm_vfe47_deinit_bandwidth_mgr,
			.update_bw = msm_vfe47_update_bandwidth,
		}
	},
	.dmi_reg_offset = 0xACC,
	.axi_hw_info = &msm_vfe47_axi_hw_info,
	.stats_hw_info = &msm_vfe47_stats_hw_info,
<<<<<<< HEAD
	.subdev_ops = &msm_vfe47_subdev_ops,
	.subdev_internal_ops = &msm_vfe47_internal_ops,
=======
	.regulator_names = {"vdd", "camss-vdd", "mmagic-vdd"},
>>>>>>> 0e91d2a... Nougat
};
EXPORT_SYMBOL(vfe47_hw_info);
