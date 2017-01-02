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
#include <linux/io.h>
#include <media/v4l2-subdev.h>
#include <asm/div64.h>
#include "msm_isp_util.h"
#include "msm_isp_axi_util.h"
#include <linux/time.h>
#define SENSOR_ISP_MAX 2
int g_subcam_SOF = 0;
extern int g_subcam_vfe_intf;
extern int g_subcam_no_ack;

<<<<<<< HEAD
#define SRC_TO_INTF(src) \
	((src < RDI_INTF_0 || src == VFE_AXI_SRC_MAX) ? VFE_PIX_0 : \
	(VFE_RAW_0 + src - RDI_INTF_0))

#define HANDLE_TO_IDX(handle) (handle & 0xFF)
=======
#define HANDLE_TO_IDX(handle) (handle & 0xFF)
#define ISP_SOF_DEBUG_COUNT 0

static int msm_isp_update_dual_HW_ms_info_at_start(
	struct vfe_device *vfe_dev,
	enum msm_vfe_input_src stream_src);

static int msm_isp_update_dual_HW_axi(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info);

#define DUAL_VFE_AND_VFE1(s, v) ((s->stream_src < RDI_INTF_0) && \
			v->is_split && vfe_dev->pdev->id == ISP_VFE1)

#define RDI_OR_NOT_DUAL_VFE(v, s) (!v->is_split || \
			((s->stream_src >= RDI_INTF_0) && \
			(stream_info->stream_src <= RDI_INTF_2)))

static inline struct msm_vfe_axi_stream *msm_isp_vfe_get_stream(
			struct dual_vfe_resource *dual_vfe_res,
			int vfe_id, uint32_t index)
{
	struct msm_vfe_axi_shared_data *axi_data =
				dual_vfe_res->axi_data[vfe_id];
	return &axi_data->stream_info[index];
}

static inline struct msm_vfe_axi_stream *msm_isp_get_controllable_stream(
		struct vfe_device *vfe_dev,
		struct msm_vfe_axi_stream *stream_info)
{
	if (vfe_dev->is_split && stream_info->stream_src < RDI_INTF_0 &&
		stream_info->controllable_output)
		return msm_isp_vfe_get_stream(
					vfe_dev->common_data->dual_vfe_res,
					ISP_VFE1,
					HANDLE_TO_IDX(
					stream_info->stream_handle));
	return stream_info;
}
>>>>>>> 0e91d2a... Nougat

int msm_isp_axi_create_stream(
	struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream_request_cmd *stream_cfg_cmd)
{
<<<<<<< HEAD
	int i, rc = -1;
	for (i = 0; i < MAX_NUM_STREAM; i++) {
		if (axi_data->stream_info[i].state == AVALIABLE)
			break;
=======
	uint32_t i = stream_cfg_cmd->stream_src;

	if (i >= VFE_AXI_SRC_MAX) {
		pr_err("%s:%d invalid stream_src %d\n", __func__, __LINE__,
			stream_cfg_cmd->stream_src);
		return -EINVAL;
>>>>>>> 0e91d2a... Nougat
	}

	if (i == MAX_NUM_STREAM) {
		pr_err("%s: No free stream\n", __func__);
		return rc;
	}

	if ((axi_data->stream_handle_cnt << 8) == 0)
		axi_data->stream_handle_cnt++;

	stream_cfg_cmd->axi_stream_handle =
		(++axi_data->stream_handle_cnt) << 8 | i;

<<<<<<< HEAD
=======
	ISP_DBG("%s: vfe %d handle %x\n", __func__, vfe_dev->pdev->id,
		stream_cfg_cmd->axi_stream_handle);

>>>>>>> 0e91d2a... Nougat
	memset(&axi_data->stream_info[i], 0,
		   sizeof(struct msm_vfe_axi_stream));
	spin_lock_init(&axi_data->stream_info[i].lock);
	axi_data->stream_info[i].session_id = stream_cfg_cmd->session_id;
	axi_data->stream_info[i].stream_id = stream_cfg_cmd->stream_id;
	axi_data->stream_info[i].buf_divert = stream_cfg_cmd->buf_divert;
	axi_data->stream_info[i].state = INACTIVE;
	axi_data->stream_info[i].stream_handle =
		stream_cfg_cmd->axi_stream_handle;
	axi_data->stream_info[i].controllable_output =
		stream_cfg_cmd->controllable_output;
<<<<<<< HEAD
=======
	axi_data->stream_info[i].activated_framedrop_period =
		MSM_VFE_STREAM_STOP_PERIOD;
>>>>>>> 0e91d2a... Nougat
	if (stream_cfg_cmd->controllable_output)
		stream_cfg_cmd->frame_skip_pattern = SKIP_ALL;
	INIT_LIST_HEAD(&axi_data->stream_info[i].request_q);
	return 0;
}

void msm_isp_axi_destroy_stream(
	struct msm_vfe_axi_shared_data *axi_data, int stream_idx)
{
	if (axi_data->stream_info[stream_idx].state != AVALIABLE) {
		axi_data->stream_info[stream_idx].state = AVALIABLE;
		axi_data->stream_info[stream_idx].stream_handle = 0;
	} else {
		pr_err("%s: stream does not exist\n", __func__);
	}
}

int msm_isp_validate_axi_request(struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream_request_cmd *stream_cfg_cmd)
{
	int rc = -1, i;
	struct msm_vfe_axi_stream *stream_info = NULL;
	if (HANDLE_TO_IDX(stream_cfg_cmd->axi_stream_handle) < MAX_NUM_STREAM) {
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->axi_stream_handle)];
	} else {
		pr_err("%s: Invalid axi_stream_handle\n", __func__);
		return rc;
	}

	if (!stream_info) {
		pr_err("%s: Stream info is NULL\n", __func__);
		return -EINVAL;
	}

	switch (stream_cfg_cmd->output_format) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR10DPCM6:
	case V4L2_PIX_FMT_SGBRG10DPCM6:
	case V4L2_PIX_FMT_SGRBG10DPCM6:
	case V4L2_PIX_FMT_SRGGB10DPCM6:
	case V4L2_PIX_FMT_SBGGR10DPCM8:
	case V4L2_PIX_FMT_SGBRG10DPCM8:
	case V4L2_PIX_FMT_SGRBG10DPCM8:
	case V4L2_PIX_FMT_SRGGB10DPCM8:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SBGGR14:
	case V4L2_PIX_FMT_SGBRG14:
	case V4L2_PIX_FMT_SGRBG14:
	case V4L2_PIX_FMT_SRGGB14:
	case V4L2_PIX_FMT_QBGGR8:
	case V4L2_PIX_FMT_QGBRG8:
	case V4L2_PIX_FMT_QGRBG8:
	case V4L2_PIX_FMT_QRGGB8:
	case V4L2_PIX_FMT_QBGGR10:
	case V4L2_PIX_FMT_QGBRG10:
	case V4L2_PIX_FMT_QGRBG10:
	case V4L2_PIX_FMT_QRGGB10:
	case V4L2_PIX_FMT_QBGGR12:
	case V4L2_PIX_FMT_QGBRG12:
	case V4L2_PIX_FMT_QGRBG12:
	case V4L2_PIX_FMT_QRGGB12:
	case V4L2_PIX_FMT_QBGGR14:
	case V4L2_PIX_FMT_QGBRG14:
	case V4L2_PIX_FMT_QGRBG14:
	case V4L2_PIX_FMT_QRGGB14:
	case V4L2_PIX_FMT_P16BGGR10:
	case V4L2_PIX_FMT_P16GBRG10:
	case V4L2_PIX_FMT_P16GRBG10:
	case V4L2_PIX_FMT_P16RGGB10:
	case V4L2_PIX_FMT_JPEG:
	case V4L2_PIX_FMT_META:
<<<<<<< HEAD
=======
	case V4L2_PIX_FMT_META10:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
>>>>>>> 0e91d2a... Nougat
		stream_info->num_planes = 1;
		stream_info->format_factor = ISP_Q2;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV14:
	case V4L2_PIX_FMT_NV41:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		stream_info->num_planes = 2;
		stream_info->format_factor = 1.5 * ISP_Q2;
		break;
	
	default:
		msm_isp_print_fourcc_error(__func__,
				stream_cfg_cmd->output_format);
		return rc;
	}

	if (axi_data->hw_info->num_wm - axi_data->num_used_wm <
		stream_info->num_planes) {
		pr_err("%s: No free write masters\n", __func__);
		return rc;
	}

	if ((stream_info->num_planes > 1) &&
			(axi_data->hw_info->num_comp_mask -
			axi_data->num_used_composite_mask < 1)) {
		pr_err("%s: No free composite mask\n", __func__);
		return rc;
	}

	if (stream_cfg_cmd->init_frame_drop >= MAX_INIT_FRAME_DROP) {
		pr_err("%s: Invalid skip pattern\n", __func__);
		return rc;
	}

	if (stream_cfg_cmd->frame_skip_pattern >= MAX_SKIP) {
		pr_err("%s: Invalid skip pattern\n", __func__);
		return rc;
	}

	for (i = 0; i < stream_info->num_planes; i++) {
		stream_info->plane_cfg[i] = stream_cfg_cmd->plane_cfg[i];
		stream_info->max_width = max(stream_info->max_width,
			stream_cfg_cmd->plane_cfg[i].output_width);
	}

	stream_info->output_format = stream_cfg_cmd->output_format;
	stream_info->runtime_output_format = stream_info->output_format;
	stream_info->stream_src = stream_cfg_cmd->stream_src;
	stream_info->frame_based = stream_cfg_cmd->frame_base;
	return 0;
}

static uint32_t msm_isp_axi_get_plane_size(
	struct msm_vfe_axi_stream *stream_info, int plane_idx)
{
	uint32_t size = 0;
	struct msm_vfe_axi_plane_cfg *plane_cfg = stream_info->plane_cfg;
	switch (stream_info->output_format) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_QBGGR8:
	case V4L2_PIX_FMT_QGBRG8:
	case V4L2_PIX_FMT_QGRBG8:
	case V4L2_PIX_FMT_QRGGB8:
	case V4L2_PIX_FMT_JPEG:
	case V4L2_PIX_FMT_META:
<<<<<<< HEAD
=======
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
>>>>>>> 0e91d2a... Nougat
		size = plane_cfg[plane_idx].output_height *
		plane_cfg[plane_idx].output_width;
		break;
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR10DPCM6:
	case V4L2_PIX_FMT_SGBRG10DPCM6:
	case V4L2_PIX_FMT_SGRBG10DPCM6:
	case V4L2_PIX_FMT_SRGGB10DPCM6:
	case V4L2_PIX_FMT_SBGGR10DPCM8:
	case V4L2_PIX_FMT_SGBRG10DPCM8:
	case V4L2_PIX_FMT_SGRBG10DPCM8:
	case V4L2_PIX_FMT_SRGGB10DPCM8:
	case V4L2_PIX_FMT_QBGGR10:
	case V4L2_PIX_FMT_QGBRG10:
	case V4L2_PIX_FMT_QGRBG10:
	case V4L2_PIX_FMT_QRGGB10:
	case V4L2_PIX_FMT_META10:
		
		size = plane_cfg[plane_idx].output_height *
		plane_cfg[plane_idx].output_width;
		break;
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_QBGGR12:
	case V4L2_PIX_FMT_QGBRG12:
	case V4L2_PIX_FMT_QGRBG12:
	case V4L2_PIX_FMT_QRGGB12:
	case V4L2_PIX_FMT_SBGGR14:
	case V4L2_PIX_FMT_SGBRG14:
	case V4L2_PIX_FMT_SGRBG14:
	case V4L2_PIX_FMT_SRGGB14:
	case V4L2_PIX_FMT_QBGGR14:
	case V4L2_PIX_FMT_QGBRG14:
	case V4L2_PIX_FMT_QGRBG14:
	case V4L2_PIX_FMT_QRGGB14:
		
		size = plane_cfg[plane_idx].output_height *
		plane_cfg[plane_idx].output_width;
		break;
	case V4L2_PIX_FMT_P16BGGR10:
	case V4L2_PIX_FMT_P16GBRG10:
	case V4L2_PIX_FMT_P16GRBG10:
	case V4L2_PIX_FMT_P16RGGB10:
		size = plane_cfg[plane_idx].output_height *
		plane_cfg[plane_idx].output_width;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		if (plane_cfg[plane_idx].output_plane_format == Y_PLANE)
			size = plane_cfg[plane_idx].output_height *
				plane_cfg[plane_idx].output_width;
		else
			size = plane_cfg[plane_idx].output_height *
				plane_cfg[plane_idx].output_width;
		break;
	case V4L2_PIX_FMT_NV14:
	case V4L2_PIX_FMT_NV41:
		if (plane_cfg[plane_idx].output_plane_format == Y_PLANE)
			size = plane_cfg[plane_idx].output_height *
				plane_cfg[plane_idx].output_width;
		else
			size = plane_cfg[plane_idx].output_height *
				plane_cfg[plane_idx].output_width;
		break;
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		size = plane_cfg[plane_idx].output_height *
			plane_cfg[plane_idx].output_width;
		break;
	
	default:
		msm_isp_print_fourcc_error(__func__,
				stream_info->output_format);
		break;
	}
	return size;
}

void msm_isp_axi_reserve_wm(struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream *stream_info)
{
	int i, j;
	for (i = 0; i < stream_info->num_planes; i++) {
		for (j = 0; j < axi_data->hw_info->num_wm; j++) {
			if (!axi_data->free_wm[j]) {
				axi_data->free_wm[j] =
					stream_info->stream_handle;
				axi_data->wm_image_size[j] =
					msm_isp_axi_get_plane_size(
						stream_info, i);
				axi_data->num_used_wm++;
				break;
			}
		}
		stream_info->wm[i] = j;
	}
}

void msm_isp_axi_free_wm(struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream *stream_info)
{
	int i;

	for (i = 0; i < stream_info->num_planes; i++) {
		axi_data->free_wm[stream_info->wm[i]] = 0;
		axi_data->num_used_wm--;
	}
	if (stream_info->stream_src <= IDEAL_RAW) {
		axi_data->num_pix_stream++;
	} else if (stream_info->stream_src < VFE_AXI_SRC_MAX) {
		axi_data->num_rdi_stream++;
	}
}

void msm_isp_axi_reserve_comp_mask(
	struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream *stream_info)
{
	int i;
	uint8_t comp_mask = 0;
	for (i = 0; i < stream_info->num_planes; i++)
		comp_mask |= 1 << stream_info->wm[i];

	for (i = 0; i < axi_data->hw_info->num_comp_mask; i++) {
		if (!axi_data->composite_info[i].stream_handle) {
			axi_data->composite_info[i].stream_handle =
				stream_info->stream_handle;
			axi_data->composite_info[i].
				stream_composite_mask = comp_mask;
			axi_data->num_used_composite_mask++;
			break;
		}
	}
	stream_info->comp_mask_index = i;
	return;
}

void msm_isp_axi_free_comp_mask(struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream *stream_info)
{
	axi_data->composite_info[stream_info->comp_mask_index].
		stream_composite_mask = 0;
	axi_data->composite_info[stream_info->comp_mask_index].
		stream_handle = 0;
	axi_data->num_used_composite_mask--;
}

int msm_isp_axi_check_stream_state(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd)
{
	int rc = 0, i;
	unsigned long flags;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	struct msm_vfe_axi_stream *stream_info;
	enum msm_vfe_axi_state valid_state =
		(stream_cfg_cmd->cmd == START_STREAM) ? INACTIVE : ACTIVE;

	if (stream_cfg_cmd->num_streams > MAX_NUM_STREAM)
		return -EINVAL;

	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		if (HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])
		> MAX_NUM_STREAM) {
			return -EINVAL;
		}
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
		spin_lock_irqsave(&stream_info->lock, flags);
		if (stream_info->state != valid_state) {
			if ((stream_info->state == PAUSING ||
				stream_info->state == PAUSED ||
				stream_info->state == RESUME_PENDING ||
				stream_info->state == RESUMING) &&
				(stream_cfg_cmd->cmd == STOP_STREAM ||
				stream_cfg_cmd->cmd == STOP_IMMEDIATELY)) {
				stream_info->state = ACTIVE;
			} else {
				pr_err("%s: Invalid stream state: %d\n",
					__func__, stream_info->state);
				spin_unlock_irqrestore(
					&stream_info->lock, flags);
				rc = -EINVAL;
				break;
			}
		}
		spin_unlock_irqrestore(&stream_info->lock, flags);
	}
	return rc;
}

void msm_isp_cfg_framedrop_reg(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	uint32_t framedrop_pattern = 0, framedrop_period = 0;
	if (stream_info->runtime_init_frame_drop == 0) {
		framedrop_pattern = stream_info->framedrop_pattern;
		framedrop_period = stream_info->framedrop_period;
	}

	if (stream_info->stream_type == BURST_STREAM &&
			stream_info->runtime_burst_frame_count == 0) {
		framedrop_pattern = 0;
		framedrop_period = 0;
	}

	ISP_DBG("%s: stream %x framedrop pattern %x period %u\n", __func__,
<<<<<<< HEAD
		stream_info->stream_id, framedrop_pattern, framedrop_period);

	vfe_dev->hw_info->vfe_ops.axi_ops.cfg_framedrop(vfe_dev, stream_info,
		framedrop_pattern, framedrop_period);
=======
		stream_info->stream_handle, framedrop_pattern,
		framedrop_period);

	BUG_ON(0 == framedrop_period);
	if (DUAL_VFE_AND_VFE1(stream_info, vfe_dev)) {
		vfe0_stream_info = msm_isp_vfe_get_stream(
					vfe_dev->common_data->dual_vfe_res,
					ISP_VFE0,
					HANDLE_TO_IDX(
					stream_info->stream_handle));
		vfe_dev->hw_info->vfe_ops.axi_ops.cfg_framedrop(
			vfe_dev->common_data->dual_vfe_res->
			vfe_base[ISP_VFE0],
			vfe0_stream_info, framedrop_pattern,
			framedrop_period);
		vfe_dev->hw_info->vfe_ops.axi_ops.cfg_framedrop(
			vfe_dev->vfe_base, stream_info,
			framedrop_pattern,
			framedrop_period);

			stream_info->requested_framedrop_period =
				framedrop_period;
			vfe0_stream_info->requested_framedrop_period =
				framedrop_period;

	} else if (RDI_OR_NOT_DUAL_VFE(vfe_dev, stream_info)) {
		vfe_dev->hw_info->vfe_ops.axi_ops.cfg_framedrop(
			vfe_dev->vfe_base, stream_info, framedrop_pattern,
			framedrop_period);
		stream_info->requested_framedrop_period = framedrop_period;
	}
>>>>>>> 0e91d2a... Nougat
}

static int msm_isp_check_epoch_status(struct vfe_device **vfe_dev,
	enum msm_vfe_input_src frame_src)
{
	struct vfe_device *vfe_dev_cur = *vfe_dev;
	struct vfe_device *vfe_dev_other = NULL;
	uint32_t vfe_id_other = 0;
	uint32_t vfe_id_cur = 0;
	uint32_t epoch_mask = 0;
	unsigned long flags;
	int completed = 0;

	spin_lock_irqsave(
		&vfe_dev_cur->common_data->common_dev_data_lock, flags);

	if (vfe_dev_cur->is_split &&
		frame_src == VFE_PIX_0) {
		if (vfe_dev_cur->pdev->id == ISP_VFE0) {
			vfe_id_cur = ISP_VFE0;
			vfe_id_other = ISP_VFE1;
		} else {
			vfe_id_cur = ISP_VFE1;
			vfe_id_other = ISP_VFE0;
		}
		vfe_dev_other = vfe_dev_cur->common_data->dual_vfe_res->
			vfe_dev[vfe_id_other];

		if (vfe_dev_cur->common_data->dual_vfe_res->
			epoch_sync_mask & (1 << vfe_id_cur)) {
			
			pr_err("Missing epoch: vfe %d, epoch mask 0x%x\n",
				vfe_dev_cur->pdev->id,
				vfe_dev_cur->common_data->dual_vfe_res->
					epoch_sync_mask);
			goto fatal;
		}

		vfe_dev_cur->common_data->dual_vfe_res->
			epoch_sync_mask |= (1 << vfe_id_cur);

		epoch_mask = (1 << vfe_id_cur) | (1 << vfe_id_other);
		if ((vfe_dev_cur->common_data->dual_vfe_res->
			epoch_sync_mask & epoch_mask) == epoch_mask) {

			if (vfe_id_other == ISP_VFE0)
				*vfe_dev = vfe_dev_cur;
			else
				*vfe_dev = vfe_dev_other;

			vfe_dev_cur->common_data->dual_vfe_res->
				epoch_sync_mask &= ~epoch_mask;
			completed = 1;
		}
	} else
		completed = 1;

	spin_unlock_irqrestore(
		&vfe_dev_cur->common_data->common_dev_data_lock, flags);

	return completed;
fatal:
	spin_unlock_irqrestore(
		&vfe_dev_cur->common_data->common_dev_data_lock, flags);
	
	msm_isp_halt_send_error(vfe_dev_cur, ISP_EVENT_PING_PONG_MISMATCH);
	return 0;
}


void msm_isp_update_framedrop_reg(struct vfe_device *vfe_dev,
	enum msm_vfe_input_src frame_src)
{
	int i;
	struct msm_vfe_axi_shared_data *axi_data = NULL;
	struct msm_vfe_axi_stream *stream_info;
<<<<<<< HEAD
	for (i = 0; i < MAX_NUM_STREAM; i++) {
=======
	unsigned long flags;

	if (msm_isp_check_epoch_status(&vfe_dev, frame_src) != 1)
		return;

	axi_data = &vfe_dev->axi_data;

	for (i = 0; i < VFE_AXI_SRC_MAX; i++) {
>>>>>>> 0e91d2a... Nougat
		if (SRC_TO_INTF(axi_data->stream_info[i].stream_src) !=
			frame_src) {
			continue;
		}
		stream_info = &axi_data->stream_info[i];
		if (stream_info->state != ACTIVE)
			continue;

<<<<<<< HEAD
		if (stream_info->runtime_framedrop_update &&
			vfe_dev->axi_data.src_info[frame_src].frame_id > 0) {
			stream_info->runtime_init_frame_drop--;
			if (stream_info->runtime_init_frame_drop == 0) {
				stream_info->runtime_framedrop_update = 0;
				msm_isp_cfg_framedrop_reg(vfe_dev, stream_info);
			}
		}
		if (stream_info->stream_type == BURST_STREAM &&
			(SRC_TO_INTF(stream_info->stream_src) == frame_src)) {
			if (stream_info->runtime_framedrop_update_burst) {
				stream_info->runtime_framedrop_update_burst = 0;
				stream_info->runtime_burst_frame_count =
					stream_info->runtime_init_frame_drop +
					(stream_info->runtime_num_burst_capture
					- 1) *
					(stream_info->framedrop_period + 1) + 1;
				msm_isp_cfg_framedrop_reg(vfe_dev, stream_info);
			} else {
				stream_info->runtime_burst_frame_count--;
				if (stream_info->runtime_burst_frame_count == 0)
					msm_isp_cfg_framedrop_reg(vfe_dev,
						stream_info);
			}
=======
		spin_lock_irqsave(&stream_info->lock, flags);

		if (BURST_STREAM == stream_info->stream_type) {
			if (0 == stream_info->runtime_num_burst_capture ||
				(stream_info->runtime_num_burst_capture == 1 &&
				stream_info->activated_framedrop_period == 1))
				stream_info->current_framedrop_period =
					MSM_VFE_STREAM_STOP_PERIOD;
		}

		if (stream_info->undelivered_request_cnt > 0)
			stream_info->current_framedrop_period =
				MSM_VFE_STREAM_STOP_PERIOD;

		if (stream_info->current_framedrop_period !=
			stream_info->requested_framedrop_period) {
			msm_isp_cfg_framedrop_reg(vfe_dev, stream_info);
>>>>>>> 0e91d2a... Nougat
		}
	}
}

void msm_isp_reset_framedrop(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	stream_info->runtime_init_frame_drop = stream_info->init_frame_drop;
	stream_info->runtime_burst_frame_count =
		stream_info->burst_frame_count;
	stream_info->runtime_num_burst_capture =
		stream_info->num_burst_capture;
	stream_info->runtime_framedrop_update = stream_info->framedrop_update;
	msm_isp_cfg_framedrop_reg(vfe_dev, stream_info);
	ISP_DBG("%s: init frame drop: %d\n", __func__,
		stream_info->runtime_init_frame_drop);
	ISP_DBG("%s: burst_frame_count: %d\n", __func__,
		stream_info->runtime_burst_frame_count);
	ISP_DBG("%s: num_burst_capture: %d\n", __func__,
		stream_info->runtime_num_burst_capture);
}

<<<<<<< HEAD
=======
void msm_isp_check_for_output_error(struct vfe_device *vfe_dev,
	struct msm_isp_timestamp *ts, struct msm_isp_sof_info *sof_info)
{
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data;
	int i;
	uint32_t stream_idx;

	if (!vfe_dev || !sof_info) {
		pr_err("%s %d failed: vfe_dev %pK sof_info %pK\n", __func__,
			__LINE__, vfe_dev, sof_info);
		return;
	}
	sof_info->regs_not_updated = 0;
	sof_info->reg_update_fail_mask = 0;
	sof_info->stream_get_buf_fail_mask = 0;

	axi_data = &vfe_dev->axi_data;
	if (!vfe_dev->reg_updated) {
		sof_info->regs_not_updated =
			vfe_dev->reg_update_requested;
	}
	for (i = 0; i < VFE_AXI_SRC_MAX; i++) {
		struct msm_vfe_axi_stream *temp_stream_info;

		stream_info = &axi_data->stream_info[i];
		stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);

		if (!((stream_info->state == ACTIVE  &&
			stream_info->controllable_output &&
			(SRC_TO_INTF(stream_info->stream_src) ==
			VFE_PIX_0)) ||
			stream_info->state == RESUMING))
				continue;

		if (stream_info->controllable_output &&
			!vfe_dev->reg_updated) {
			temp_stream_info =
				msm_isp_get_controllable_stream(vfe_dev,
				stream_info);
			if (temp_stream_info->undelivered_request_cnt) {
				if (msm_isp_drop_frame(vfe_dev, stream_info, ts,
					sof_info)) {
					pr_err("drop frame failed\n");
				}
			}
		}

		if (stream_info->state == RESUMING &&
			!stream_info->controllable_output) {
			ISP_DBG("%s: axi_updating_mask stream_id %x frame_id %d\n",
				__func__, stream_idx, vfe_dev->axi_data.
				src_info[SRC_TO_INTF(stream_info->stream_src)]
				.frame_id);
			sof_info->axi_updating_mask |=
				1 << stream_idx;
		}
	}

	vfe_dev->reg_updated = 0;

	
	if (vfe_dev->error_info.framedrop_flag) {
		for (i = 0; i < BUF_MGR_NUM_BUF_Q; i++) {
			if (vfe_dev->error_info.stream_framedrop_count[i]) {
				ISP_DBG("%s: get buf failed i %d\n", __func__,
					i);
				sof_info->stream_get_buf_fail_mask |= (1 << i);
				vfe_dev->error_info.
					stream_framedrop_count[i] = 0;
			}
		}
		vfe_dev->error_info.framedrop_flag = 0;
	}
}

void msm_isp_increment_frame_id(struct vfe_device *vfe_dev,
	enum msm_vfe_input_src frame_src, struct msm_isp_timestamp *ts)
{
	struct msm_vfe_src_info *src_info = NULL;
	struct msm_vfe_sof_info *sof_info = NULL;
	enum msm_vfe_dual_hw_type dual_hw_type;
	enum msm_vfe_dual_hw_ms_type ms_type;
	struct msm_vfe_sof_info *master_sof_info = NULL;
	int32_t time, master_time, delta;
	uint32_t sof_incr = 0;
	unsigned long flags;

	if (vfe_dev->axi_data.src_info[frame_src].frame_id == 0)
		msm_isp_update_dual_HW_ms_info_at_start(vfe_dev, frame_src);

	spin_lock_irqsave(&vfe_dev->common_data->common_dev_data_lock, flags);
	dual_hw_type =
		vfe_dev->axi_data.src_info[frame_src].dual_hw_type;
	ms_type =
		vfe_dev->axi_data.src_info[frame_src].
		dual_hw_ms_info.dual_hw_ms_type;
	if ((dual_hw_type == DUAL_HW_MASTER_SLAVE) &&
		(ms_type == MS_TYPE_SLAVE) &&
		(vfe_dev->common_data->ms_resource.master_active == 1)) {
		
		time = ts->buf_time.tv_sec * 1000 +
			ts->buf_time.tv_usec / 1000;
		master_sof_info = &vfe_dev->common_data->ms_resource.
			master_sof_info;
		master_time = master_sof_info->mono_timestamp_ms;
		delta = vfe_dev->common_data->ms_resource.sof_delta_threshold;
		ISP_DBG("%s: vfe %d frame_src %d frame %d Slave time %d Master time %d delta %d\n",
			__func__, vfe_dev->pdev->id, frame_src,
			vfe_dev->axi_data.src_info[frame_src].frame_id,
			time, master_time, time - master_time);

		if (time - master_time > delta)
			sof_incr = 1;

		vfe_dev->axi_data.src_info[frame_src].frame_id =
			master_sof_info->frame_id + sof_incr;
	} else {
		if (frame_src == VFE_PIX_0) {
			vfe_dev->axi_data.src_info[frame_src].frame_id +=
				vfe_dev->axi_data.src_info[frame_src].
				sof_counter_step;
			ISP_DBG("%s: vfe %d sof_step %d\n", __func__,
			vfe_dev->pdev->id,
			vfe_dev->axi_data.src_info[frame_src].
				sof_counter_step);
			src_info = &vfe_dev->axi_data.src_info[frame_src];

			if (!src_info->frame_id &&
				!src_info->reg_update_frame_id &&
				((src_info->frame_id -
				src_info->reg_update_frame_id) >
				(MAX_REG_UPDATE_THRESHOLD *
				src_info->sof_counter_step))) {
				pr_err("%s:%d reg_update not received for %d frames\n",
					__func__, __LINE__,
					src_info->frame_id -
					src_info->reg_update_frame_id);

				msm_isp_halt_send_error(vfe_dev,
					ISP_EVENT_REG_UPDATE_MISSING);
			}

		} else
			vfe_dev->axi_data.src_info[frame_src].frame_id++;
	}

	sof_info = vfe_dev->axi_data.src_info[frame_src].
		dual_hw_ms_info.sof_info;
	if (dual_hw_type == DUAL_HW_MASTER_SLAVE &&
		sof_info != NULL) {
		sof_info->frame_id = vfe_dev->axi_data.src_info[frame_src].
			frame_id;
		sof_info->timestamp_ms = ts->event_time.tv_sec * 1000 +
			ts->event_time.tv_usec / 1000;
		sof_info->mono_timestamp_ms = ts->buf_time.tv_sec * 1000 +
			ts->buf_time.tv_usec / 1000;
	}
	spin_unlock_irqrestore(&vfe_dev->common_data->common_dev_data_lock,
		flags);
}

>>>>>>> 0e91d2a... Nougat
void msm_isp_notify(struct vfe_device *vfe_dev, uint32_t event_type,
	enum msm_vfe_input_src frame_src, struct msm_isp_timestamp *ts)
{
	struct msm_isp_event_data event_data;
	
	static struct timeval m_last[SENSOR_ISP_MAX];
	struct timeval m_curr;
	static uint32_t last_frame_id[SENSOR_ISP_MAX];
	uint32_t time_diff = 0;
	

	switch (event_type) {
	case ISP_EVENT_SOF:
		
		if (frame_src == VFE_PIX_0) {
			if ((vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id < 3)||
				(vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id == 10))
			{
				pr_info("[CAM]%s:(%d)PIX0 frame id: %u\n", __func__, vfe_dev->pdev->id,
					vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id);
			}

			if(vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id == 0)
			{
				do_gettimeofday(&m_last[vfe_dev->pdev->id]);
				last_frame_id[vfe_dev->pdev->id] = 0;
			}
			else
			{
				do_gettimeofday(&m_curr);
				time_diff = (m_curr.tv_sec-m_last[vfe_dev->pdev->id].tv_sec)*1000000 + (m_curr.tv_usec-m_last[vfe_dev->pdev->id].tv_usec);
				if(time_diff >= 1000000)   
				{
					pr_info("[CAM]%s:(%d)PIX0 frame id: %u, fps: %d\n", __func__, vfe_dev->pdev->id, vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id, (vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id-last_frame_id[vfe_dev->pdev->id]));
					m_last[vfe_dev->pdev->id] = m_curr;
					last_frame_id[vfe_dev->pdev->id] = vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id;
				}
			}
		}
		
		
		if(vfe_dev->pdev->id == g_subcam_vfe_intf && g_subcam_SOF < 100)
			g_subcam_SOF ++;
		
		vfe_dev->axi_data.src_info[frame_src].frame_id++;
		if (vfe_dev->axi_data.src_info[frame_src].frame_id == 0)
			vfe_dev->axi_data.src_info[frame_src].frame_id = 1;
		ISP_DBG("%s: frame_src %d frame id: %u\n", __func__,
			frame_src,
			vfe_dev->axi_data.src_info[frame_src].frame_id);
		break;

	default:
		break;
	}

	event_data.input_intf = frame_src;
	event_data.frame_id = vfe_dev->axi_data.src_info[frame_src].frame_id;
	event_data.timestamp = ts->event_time;
	event_data.mono_timestamp = ts->buf_time;
	msm_isp_send_event(vfe_dev, event_type | frame_src, &event_data);
}

void msm_isp_calculate_framedrop(
	struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream_request_cmd *stream_cfg_cmd)
{
	uint32_t framedrop_period = 0;
	struct msm_vfe_axi_stream *stream_info = NULL;
	if (HANDLE_TO_IDX(stream_cfg_cmd->axi_stream_handle) < MAX_NUM_STREAM) {
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->axi_stream_handle)];
	} else {
		pr_err("%s: Invalid stream handle", __func__);
		return;
	}
	if (!stream_info) {
		pr_err("%s: Stream info is NULL\n", __func__);
		return;
	}

	framedrop_period = msm_isp_get_framedrop_period(
	   stream_cfg_cmd->frame_skip_pattern);
	stream_info->frame_skip_pattern =
		stream_cfg_cmd->frame_skip_pattern;
	if (stream_cfg_cmd->frame_skip_pattern == SKIP_ALL)
		stream_info->framedrop_pattern = 0x0;
	else
		stream_info->framedrop_pattern = 0x1;
	stream_info->framedrop_period = framedrop_period - 1;

	if (stream_cfg_cmd->init_frame_drop < framedrop_period) {
		stream_info->framedrop_pattern <<=
			stream_cfg_cmd->init_frame_drop;
		stream_info->init_frame_drop = 0;
		stream_info->framedrop_update = 0;
	} else {
		stream_info->init_frame_drop = stream_cfg_cmd->init_frame_drop;
		stream_info->framedrop_update = 1;
	}

	if (stream_cfg_cmd->burst_count > 0) {
		stream_info->stream_type = BURST_STREAM;
		stream_info->num_burst_capture =
			stream_cfg_cmd->burst_count;
		stream_info->burst_frame_count =
		stream_cfg_cmd->init_frame_drop +
			(stream_cfg_cmd->burst_count - 1) *
			framedrop_period + 1;
	} else {
		stream_info->stream_type = CONTINUOUS_STREAM;
		stream_info->burst_frame_count = 0;
		stream_info->num_burst_capture = 0;
	}
}

void msm_isp_calculate_bandwidth(
	struct msm_vfe_axi_shared_data *axi_data,
	struct msm_vfe_axi_stream *stream_info)
{
	if (stream_info->stream_src < RDI_INTF_0) {
		stream_info->bandwidth =
			(axi_data->src_info[VFE_PIX_0].pixel_clock /
			axi_data->src_info[VFE_PIX_0].width) *
			stream_info->max_width;
		stream_info->bandwidth = (unsigned long)stream_info->bandwidth *
			stream_info->format_factor / ISP_Q2;
	} else {
		int rdi = SRC_TO_INTF(stream_info->stream_src);
		if (rdi < VFE_SRC_MAX)
			stream_info->bandwidth =
				axi_data->src_info[rdi].pixel_clock;
		else
			pr_err("%s: Invalid rdi interface\n", __func__);
	}
}

#ifdef CONFIG_MSM_AVTIMER
void msm_isp_start_avtimer(void)
{
	avcs_core_open();
	avcs_core_disable_power_collapse(1);
}

void msm_isp_get_avtimer_ts(
		struct msm_isp_timestamp *time_stamp)
{
	int rc = 0;
	uint32_t avtimer_usec = 0;
	uint64_t avtimer_tick = 0;

	rc = avcs_core_query_timer(&avtimer_tick);
	if (rc < 0) {
		pr_err("%s: Error: Invalid AVTimer Tick, rc=%d\n",
			   __func__, rc);
		
		time_stamp->vt_time.tv_sec = 0;
		time_stamp->vt_time.tv_usec = 0;
	} else {
		avtimer_usec = do_div(avtimer_tick, USEC_PER_SEC);
		time_stamp->vt_time.tv_sec = (uint32_t)(avtimer_tick);
		time_stamp->vt_time.tv_usec = avtimer_usec;
		pr_debug("%s: AVTimer TS = %u:%u\n", __func__,
			(uint32_t)(avtimer_tick), avtimer_usec);
	}
}
#else
void msm_isp_start_avtimer(void)
{
	pr_err("AV Timer is not supported\n");
}

void msm_isp_get_avtimer_ts(
		struct msm_isp_timestamp *time_stamp)
{
	pr_err_ratelimited("%s: Error: AVTimer driver not available\n",
		__func__);
	time_stamp->vt_time.tv_sec = 0;
	time_stamp->vt_time.tv_usec = 0;
}
#endif

int msm_isp_request_axi_stream(struct vfe_device *vfe_dev, void *arg)
{
	int rc = 0, i;
	uint32_t io_format = 0;
	struct msm_vfe_axi_stream_request_cmd *stream_cfg_cmd = arg;
	struct msm_vfe_axi_stream *stream_info;

	rc = msm_isp_axi_create_stream(
		&vfe_dev->axi_data, stream_cfg_cmd);
	if (rc) {
		pr_err("%s: create stream failed\n", __func__);
		return rc;
	}

	rc = msm_isp_validate_axi_request(
		&vfe_dev->axi_data, stream_cfg_cmd);
	if (rc) {
		pr_err("%s: Request validation failed\n", __func__);
		msm_isp_axi_destroy_stream(&vfe_dev->axi_data,
			HANDLE_TO_IDX(stream_cfg_cmd->axi_stream_handle));
		return rc;
	}
	stream_info = &vfe_dev->axi_data.
		stream_info[HANDLE_TO_IDX(stream_cfg_cmd->axi_stream_handle)];
	if (!stream_info) {
		pr_err("%s: can not find stream handle %x\n", __func__,
			stream_cfg_cmd->axi_stream_handle);
		return -EINVAL;
	}

	stream_info->memory_input = stream_cfg_cmd->memory_input;

	msm_isp_axi_reserve_wm(&vfe_dev->axi_data, stream_info);

	if (stream_info->stream_src < RDI_INTF_0) {
		io_format = vfe_dev->axi_data.src_info[VFE_PIX_0].input_format;
		if (stream_info->stream_src == CAMIF_RAW ||
			stream_info->stream_src == IDEAL_RAW) {
			if (stream_info->stream_src == CAMIF_RAW &&
				io_format != stream_info->output_format)
				pr_debug("%s: Overriding input format\n",
					__func__);

			io_format = stream_info->output_format;
		}
		rc = vfe_dev->hw_info->vfe_ops.axi_ops.cfg_io_format(
			vfe_dev, stream_info->stream_src, io_format);
		if (rc) {
			pr_err("%s: cfg io format failed\n", __func__);
			msm_isp_axi_free_wm(&vfe_dev->axi_data,
				stream_info);
			msm_isp_axi_destroy_stream(&vfe_dev->axi_data,
				HANDLE_TO_IDX(
				stream_cfg_cmd->axi_stream_handle));
			return rc;
		}
	}

	msm_isp_calculate_framedrop(&vfe_dev->axi_data, stream_cfg_cmd);
	if (stream_cfg_cmd->vt_enable && !vfe_dev->vt_enable) {
		vfe_dev->vt_enable = stream_cfg_cmd->vt_enable;
		msm_isp_start_avtimer();
	}
	if (stream_info->num_planes > 1)
		msm_isp_axi_reserve_comp_mask(
			&vfe_dev->axi_data, stream_info);

	for (i = 0; i < stream_info->num_planes; i++) {
		vfe_dev->hw_info->vfe_ops.axi_ops.
			cfg_wm_reg(vfe_dev, stream_info, i);

		vfe_dev->hw_info->vfe_ops.axi_ops.
			cfg_wm_xbar_reg(vfe_dev, stream_info, i);
	}
	return rc;
}

int msm_isp_release_axi_stream(struct vfe_device *vfe_dev, void *arg)
{
	int rc = 0, i;
	struct msm_vfe_axi_stream_release_cmd *stream_release_cmd = arg;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	struct msm_vfe_axi_stream *stream_info =
		&axi_data->stream_info[
		HANDLE_TO_IDX(stream_release_cmd->stream_handle)];
	struct msm_vfe_axi_stream_cfg_cmd stream_cfg;

	if (stream_info->state == AVALIABLE) {
		pr_err("%s: Stream already released\n", __func__);
		return -EINVAL;
	} else if (stream_info->state != INACTIVE) {
		stream_cfg.cmd = STOP_STREAM;
		stream_cfg.num_streams = 1;
		stream_cfg.stream_handle[0] = stream_release_cmd->stream_handle;
		msm_isp_cfg_axi_stream(vfe_dev, (void *) &stream_cfg);
	}

	for (i = 0; i < stream_info->num_planes; i++) {
		vfe_dev->hw_info->vfe_ops.axi_ops.
			clear_wm_reg(vfe_dev, stream_info, i);

		vfe_dev->hw_info->vfe_ops.axi_ops.
		clear_wm_xbar_reg(vfe_dev, stream_info, i);
	}

	if (stream_info->num_planes > 1)
		msm_isp_axi_free_comp_mask(&vfe_dev->axi_data, stream_info);

	vfe_dev->hw_info->vfe_ops.axi_ops.clear_framedrop(vfe_dev, stream_info);
	msm_isp_axi_free_wm(axi_data, stream_info);

	msm_isp_axi_destroy_stream(&vfe_dev->axi_data,
		HANDLE_TO_IDX(stream_release_cmd->stream_handle));

	return rc;
}

static void msm_isp_axi_stream_enable_cfg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	int i;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
<<<<<<< HEAD
=======
	uint32_t stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);
	struct dual_vfe_resource *dual_vfe_res = NULL;

	if (stream_idx >= VFE_AXI_SRC_MAX) {
		pr_err("%s: Invalid stream_idx", __func__);
		goto error;
	}

>>>>>>> 0e91d2a... Nougat
	if (stream_info->state == INACTIVE)
		return;
	for (i = 0; i < stream_info->num_planes; i++) {
<<<<<<< HEAD
		if (stream_info->state == START_PENDING ||
			stream_info->state == RESUME_PENDING) {
=======
		if ((stream_info->stream_src < RDI_INTF_0) &&
			vfe_dev->is_split && vfe_dev->pdev->id == ISP_VFE1 &&
			dual_vfe_sync) {
			dual_vfe_res = vfe_dev->common_data->dual_vfe_res;
			if (!dual_vfe_res->vfe_base[ISP_VFE0] ||
				!dual_vfe_res->axi_data[ISP_VFE0] ||
				!dual_vfe_res->vfe_base[ISP_VFE1] ||
				!dual_vfe_res->axi_data[ISP_VFE1]) {
				pr_err("%s:%d failed vfe0 %pK %pK vfe %pK %pK\n",
					__func__, __LINE__,
					dual_vfe_res->vfe_base[ISP_VFE0],
					dual_vfe_res->axi_data[ISP_VFE0],
					dual_vfe_res->vfe_base[ISP_VFE1],
					dual_vfe_res->axi_data[ISP_VFE1]);
				goto error;
			}
			for (vfe_id = 0; vfe_id < MAX_VFE; vfe_id++) {
				vfe_dev->hw_info->vfe_ops.axi_ops.
				enable_wm(dual_vfe_res->vfe_base[vfe_id],
					dual_vfe_res->axi_data[vfe_id]->
					stream_info[stream_idx].wm[i],
					enable_wm);
			}
		} else if (!vfe_dev->is_split ||
			(stream_info->stream_src >= RDI_INTF_0 &&
			stream_info->stream_src <= RDI_INTF_2) ||
			!dual_vfe_sync) {
>>>>>>> 0e91d2a... Nougat
			vfe_dev->hw_info->vfe_ops.axi_ops.
				enable_wm(vfe_dev, stream_info->wm[i], 1);
		} else {
			vfe_dev->hw_info->vfe_ops.axi_ops.
				enable_wm(vfe_dev, stream_info->wm[i], 0);
			if (vfe_dev->axi_data.src_info[VFE_PIX_0].
				raw_stream_count > 0
				&& vfe_dev->axi_data.src_info[VFE_PIX_0].
				pix_stream_count == 0) {
				if (stream_info->stream_src == CAMIF_RAW ||
					stream_info->stream_src == IDEAL_RAW) {
					vfe_dev->hw_info->vfe_ops.core_ops.
						reg_update(vfe_dev,
						VFE_PIX_0);
				}
			}
		}
	}

	if (stream_info->state == START_PENDING)
		axi_data->num_active_stream++;
	else if (stream_info->state == STOP_PENDING)
		axi_data->num_active_stream--;
}

void msm_isp_axi_stream_update(struct vfe_device *vfe_dev,
	enum msm_vfe_input_src frame_src)
{
	int i;
	unsigned long flags;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;

	spin_lock_irqsave(&vfe_dev->shared_data_lock, flags);
	for (i = 0; i < MAX_NUM_STREAM; i++) {
		if (SRC_TO_INTF(axi_data->stream_info[i].stream_src) !=
			frame_src) {
			ISP_DBG("%s stream_src %d frame_src %d\n", __func__,
				SRC_TO_INTF(
				axi_data->stream_info[i].stream_src),
				frame_src);
			continue;
		}
		if (axi_data->stream_info[i].state == UPDATING)
			axi_data->stream_info[i].state = ACTIVE;
		else if (axi_data->stream_info[i].state == START_PENDING ||
			axi_data->stream_info[i].state == STOP_PENDING) {
			msm_isp_axi_stream_enable_cfg(
<<<<<<< HEAD
				vfe_dev, &axi_data->stream_info[i]);
=======
				vfe_dev, &axi_data->stream_info[i],
				axi_data->stream_info[i].state ==
				START_PENDING ? 1 : 0);
>>>>>>> 0e91d2a... Nougat
			axi_data->stream_info[i].state =
				axi_data->stream_info[i].state ==
				START_PENDING ? STARTING : STOPPING;
		} else if (axi_data->stream_info[i].state == STARTING ||
			axi_data->stream_info[i].state == STOPPING) {
			axi_data->stream_info[i].state =
				axi_data->stream_info[i].state == STARTING ?
				ACTIVE : INACTIVE;
		}
	}

	if (vfe_dev->axi_data.stream_update[frame_src]) {
		vfe_dev->axi_data.stream_update[frame_src]--;
	}

	if (vfe_dev->axi_data.pipeline_update == DISABLE_CAMIF ||
		(vfe_dev->axi_data.pipeline_update ==
		DISABLE_CAMIF_IMMEDIATELY)) {
		vfe_dev->hw_info->vfe_ops.stats_ops.
			enable_module(vfe_dev, 0xFF, 0);
		vfe_dev->axi_data.pipeline_update = NO_UPDATE;
	}

	if (vfe_dev->axi_data.stream_update[frame_src] == 0)
		complete(&vfe_dev->stream_config_complete);

	spin_unlock_irqrestore(&vfe_dev->shared_data_lock, flags);
}

static void msm_isp_reload_ping_pong_offset(struct vfe_device *vfe_dev,
		struct msm_vfe_axi_stream *stream_info)
{
	int i, j;
	uint32_t flag;
	struct msm_isp_buffer *buf;
	for (i = 0; i < 2; i++) {
		buf = stream_info->buf[i];
		if (!buf)
			continue;
		flag = i ? VFE_PONG_FLAG : VFE_PING_FLAG;
		for (j = 0; j < stream_info->num_planes; j++) {
			vfe_dev->hw_info->vfe_ops.axi_ops.update_ping_pong_addr(
				vfe_dev, stream_info->wm[j], flag,
				buf->mapped_info[j].paddr +
				stream_info->plane_cfg[j].plane_addr_offset);
		}
	}
}

void msm_isp_axi_cfg_update(struct vfe_device *vfe_dev,
	enum msm_vfe_input_src frame_src)
{
	int i, j;
	uint32_t update_state;
	unsigned long flags, flags1;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	struct msm_vfe_axi_stream *stream_info;
	int num_stream = 0;

<<<<<<< HEAD
	for (i = 0; i < MAX_NUM_STREAM; i++) {
=======
	spin_lock_irqsave(&vfe_dev->common_data->common_dev_data_lock, flags);
	for (i = 0; i < VFE_AXI_SRC_MAX; i++) {
>>>>>>> 0e91d2a... Nougat
		if (SRC_TO_INTF(axi_data->stream_info[i].stream_src) !=
			frame_src) {
			continue;
		}
		num_stream++;
		stream_info = &axi_data->stream_info[i];
<<<<<<< HEAD
		if (stream_info->stream_type == BURST_STREAM ||
			stream_info->state == AVALIABLE)
=======
		if (stream_info->state == INACTIVE)
			continue;
		if ((stream_info->stream_type == BURST_STREAM &&
			!stream_info->controllable_output) ||
			stream_info->state == AVAILABLE)
>>>>>>> 0e91d2a... Nougat
			continue;
		spin_lock_irqsave(&stream_info->lock, flags1);
		if (stream_info->state == PAUSING) {
			
			stream_info->state = PAUSED;
			msm_isp_reload_ping_pong_offset(vfe_dev, stream_info);
			for (j = 0; j < stream_info->num_planes; j++)
				vfe_dev->hw_info->vfe_ops.axi_ops.
					cfg_wm_reg(vfe_dev, stream_info, j);
			
			stream_info->state = RESUME_PENDING;
<<<<<<< HEAD
			msm_isp_axi_stream_enable_cfg(
				vfe_dev, &axi_data->stream_info[i]);
			stream_info->state = RESUMING;
=======
			msm_isp_update_dual_HW_axi(vfe_dev, stream_info);
>>>>>>> 0e91d2a... Nougat
		} else if (stream_info->state == RESUMING) {
			msm_isp_update_dual_HW_axi(vfe_dev, stream_info);
		}
		spin_unlock_irqrestore(&stream_info->lock, flags1);
	}
	spin_unlock_irqrestore(&vfe_dev->common_data->common_dev_data_lock,
		flags);
	if (num_stream)
		update_state = atomic_dec_return(
			&axi_data->axi_cfg_update[frame_src]);
}

static int msm_isp_cfg_pong_address(struct vfe_device *vfe_dev,
		struct msm_vfe_axi_stream *stream_info)
{
<<<<<<< HEAD
	int i, rc = -1;
	struct msm_isp_buffer *buf = stream_info->buf[0];
	if (!buf)
		return rc;
	for (i = 0; i < stream_info->num_planes; i++)
		vfe_dev->hw_info->vfe_ops.axi_ops.update_ping_pong_addr(
			vfe_dev, stream_info->wm[i],
			VFE_PONG_FLAG, buf->mapped_info[i].paddr +
			stream_info->plane_cfg[i].plane_addr_offset);
	stream_info->buf[1] = buf;
	return 0;
=======
	uint32_t i = 0;
	struct msm_isp_event_data error_event;
	struct msm_vfe_axi_halt_cmd halt_cmd;
	uint32_t irq_status0, irq_status1;

	if (ISP_EVENT_PING_PONG_MISMATCH == event &&
		vfe_dev->axi_data.recovery_count < MAX_RECOVERY_THRESHOLD) {
		vfe_dev->hw_info->vfe_ops.irq_ops.
			read_irq_status(vfe_dev, &irq_status0, &irq_status1);
		pr_err("%s:pingpong mismatch from vfe%d, core%d, recovery_count %d\n",
			__func__, vfe_dev->pdev->id, smp_processor_id(),
			vfe_dev->axi_data.recovery_count);

		vfe_dev->axi_data.recovery_count++;

		msm_isp_process_overflow_irq(vfe_dev,
			&irq_status0, &irq_status1, 1);
		return;
	}

	memset(&halt_cmd, 0, sizeof(struct msm_vfe_axi_halt_cmd));
	memset(&error_event, 0, sizeof(struct msm_isp_event_data));
	halt_cmd.stop_camif = 1;
	halt_cmd.overflow_detected = 0;
	halt_cmd.blocking_halt = 0;

	pr_err("%s: vfe%d  exiting camera!\n", __func__, vfe_dev->pdev->id);

	atomic_set(&vfe_dev->error_info.overflow_state,
		HALT_ENFORCED);

	
	msm_isp_axi_halt(vfe_dev, &halt_cmd);

	for (i = 0; i < VFE_AXI_SRC_MAX; i++)
		vfe_dev->axi_data.stream_info[i].state =
			INACTIVE;

	error_event.frame_id =
		vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id;

	msm_isp_send_event(vfe_dev, event, &error_event);
>>>>>>> 0e91d2a... Nougat
}

static void msm_isp_get_done_buf(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t pingpong_status,
	struct msm_isp_buffer **done_buf)
{
	uint32_t pingpong_bit = 0, i;
	pingpong_bit = (~(pingpong_status >> stream_info->wm[0]) & 0x1);
	for (i = 0; i < stream_info->num_planes; i++) {
		if (pingpong_bit !=
			(~(pingpong_status >> stream_info->wm[i]) & 0x1)) {
			pr_debug("%s: Write master ping pong mismatch. Status: 0x%x\n",
				__func__, pingpong_status);
		}
	}

	*done_buf = stream_info->buf[pingpong_bit];

	if (stream_info->controllable_output) {
		stream_info->buf[pingpong_bit] = NULL;
		if (!stream_info->undelivered_request_cnt) {
			pr_err("%s:%d error undelivered_request_cnt 0\n",
				__func__, __LINE__);
		} else {
		   stream_info->undelivered_request_cnt--;
		}
	}
}

static int msm_isp_cfg_ping_pong_address(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t pingpong_status)
{
	int i, rc = -1;
	struct msm_isp_buffer *buf = NULL;
	uint32_t bufq_handle = 0;
	struct msm_vfe_frame_request_queue *queue_req;
	uint32_t pingpong_bit;
	uint32_t stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);

	if (stream_idx >= MAX_NUM_STREAM) {
		pr_err("%s: Invalid stream_idx", __func__);
		return rc;
	}

	if (!stream_info->controllable_output) {
		bufq_handle = stream_info->bufq_handle[VFE_BUF_QUEUE_DEFAULT];
	} else {
		queue_req = list_first_entry_or_null(&stream_info->request_q,
			struct msm_vfe_frame_request_queue, list);
		if (!queue_req)
			return 0;

		queue_req->cmd_used = 0;
		bufq_handle = stream_info->
			bufq_handle[queue_req->buff_queue_id];
		list_del(&queue_req->list);
		stream_info->request_q_cnt--;

		if (!bufq_handle) {
			pr_err("%s: Drop request. Shared stream is stopped.\n",
				__func__);
			return -EINVAL;
		}
	}

	rc = vfe_dev->buf_mgr->ops->get_buf(vfe_dev->buf_mgr,
			vfe_dev->pdev->id, bufq_handle, &buf);

<<<<<<< HEAD
	if (rc < 0) {
		vfe_dev->error_info.stream_framedrop_count[stream_idx]++;
		return rc;
	}
=======
	if (rc == -EFAULT) {
		msm_isp_halt_send_error(vfe_dev,
			ISP_EVENT_BUF_FATAL_ERROR);
		return buf;
	}
	if (rc < 0)
		return buf;
>>>>>>> 0e91d2a... Nougat

	if (buf->num_planes != stream_info->num_planes) {
		pr_err("%s: Invalid buffer\n", __func__);
		rc = -EINVAL;
		goto buf_error;
	}

<<<<<<< HEAD
	for (i = 0; i < stream_info->num_planes; i++)
		vfe_dev->hw_info->vfe_ops.axi_ops.update_ping_pong_addr(
			vfe_dev, stream_info->wm[i],
			pingpong_status, buf->mapped_info[i].paddr +
			stream_info->plane_cfg[i].plane_addr_offset);

	pingpong_bit = (~(pingpong_status >> stream_info->wm[0]) & 0x1);
	stream_info->buf[pingpong_bit] = buf;

	return 0;
buf_error:
	vfe_dev->buf_mgr->ops->put_buf(vfe_dev->buf_mgr,
		buf->bufq_handle, buf->buf_idx);
	return rc;
}

static void msm_isp_process_done_buf(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, struct msm_isp_buffer *buf,
	struct msm_isp_timestamp *ts)
=======
	return buf;
}

int msm_isp_cfg_offline_ping_pong_address(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t pingpong_status,
	uint32_t buf_idx)
{
	int i, rc = 0;
	struct msm_isp_buffer *buf = NULL;
	uint32_t pingpong_bit;
	uint32_t stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);
	uint32_t buffer_size_byte = 0;
	int32_t word_per_line = 0;
	dma_addr_t paddr;
	uint32_t bufq_handle = 0;

	if (stream_idx >= VFE_AXI_SRC_MAX) {
		pr_err("%s: Invalid stream_idx", __func__);
		return -EINVAL;
	}

	bufq_handle = stream_info->bufq_handle[VFE_BUF_QUEUE_DEFAULT];

	if (!vfe_dev->is_split) {
		rc = vfe_dev->buf_mgr->ops->get_buf_by_index(
			vfe_dev->buf_mgr, bufq_handle, buf_idx, &buf);
		if (rc < 0 || !buf) {
			pr_err("%s: No fetch buffer rc= %d buf= %p\n",
				__func__, rc, buf);
			return -EINVAL;
		}

		if (buf->num_planes != stream_info->num_planes) {
			pr_err("%s: Invalid buffer\n", __func__);
			vfe_dev->buf_mgr->ops->put_buf(vfe_dev->buf_mgr,
				bufq_handle, buf->buf_idx);
			return -EINVAL;
		}

		pingpong_bit = ((pingpong_status >>
			stream_info->wm[0]) & 0x1);

		for (i = 0; i < stream_info->num_planes; i++) {
			word_per_line = msm_isp_cal_word_per_line(
				stream_info->output_format,
				stream_info->plane_cfg[i].
				output_stride);
			if (word_per_line < 0) {
				
				word_per_line = 0;
				buffer_size_byte = 0;
			} else {
				buffer_size_byte = (word_per_line * 8 *
					stream_info->plane_cfg[i].
					output_scan_lines) -
					stream_info->
					plane_cfg[i].plane_addr_offset;
			}
			paddr = buf->mapped_info[i].paddr;

			vfe_dev->hw_info->vfe_ops.axi_ops.
				update_ping_pong_addr(
				vfe_dev->vfe_base, stream_info->wm[i],
				pingpong_bit, paddr +
				stream_info->
				plane_cfg[i].plane_addr_offset,
				buffer_size_byte);

			if (0 == i) {
				stream_info->buf[!pingpong_bit] = buf;
				buf->pingpong_bit = !pingpong_bit;
			}
			buf->state = MSM_ISP_BUFFER_STATE_DEQUEUED;
		}
	}
	return rc;

}

static int msm_isp_cfg_ping_pong_address(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t pingpong_status,
	int scratch)
>>>>>>> 0e91d2a... Nougat
{
	int rc;
	struct msm_isp_event_data buf_event;
	struct timeval *time_stamp;
	uint32_t stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);
<<<<<<< HEAD
	uint32_t frame_id;
	uint32_t buf_src;
	memset(&buf_event, 0, sizeof(buf_event));
=======
	uint32_t buffer_size_byte = 0;
	int32_t word_per_line = 0;
	dma_addr_t paddr;
	struct dual_vfe_resource *dual_vfe_res = NULL;
	uint32_t vfe_id = 0;
	unsigned long flags;

	if (stream_idx >= VFE_AXI_SRC_MAX) {
		pr_err("%s: Invalid stream_idx", __func__);
		return -EINVAL;
	}
	
	if ((stream_info->stream_src < RDI_INTF_0) &&
		vfe_dev->is_split) {
		dual_vfe_res = vfe_dev->common_data->dual_vfe_res;
		if (!dual_vfe_res->vfe_base[ISP_VFE0] ||
			!dual_vfe_res->axi_data[ISP_VFE0] ||
			!dual_vfe_res->vfe_base[ISP_VFE1] ||
			!dual_vfe_res->axi_data[ISP_VFE1]) {
			pr_err("%s:%d failed vfe0 %pK %pK vfe %pK %pK\n",
				__func__, __LINE__,
				dual_vfe_res->vfe_base[ISP_VFE0],
				dual_vfe_res->axi_data[ISP_VFE0],
				dual_vfe_res->vfe_base[ISP_VFE1],
				dual_vfe_res->axi_data[ISP_VFE1]);
			return -EINVAL;
		}
	} else if (!vfe_dev->is_split ||
			(stream_info->stream_src >= RDI_INTF_0 &&
			stream_info->stream_src <= RDI_INTF_2)) {
		dual_vfe_res = NULL;
	} else {
		pr_err("%s: Error! Should not reach this case is_split %d stream_src %d\n",
			__func__, vfe_dev->is_split, stream_info->stream_src);
		msm_isp_halt_send_error(vfe_dev, ISP_EVENT_BUF_FATAL_ERROR);
		return 0;
	}

	if (!scratch)
		buf = msm_isp_get_stream_buffer(vfe_dev, stream_info);

	
	pingpong_bit = ((pingpong_status >>
		stream_info->wm[0]) & 0x1);

	for (i = 0; i < stream_info->num_planes; i++) {
		if (buf) {
			word_per_line = msm_isp_cal_word_per_line(
				stream_info->output_format, stream_info->
				plane_cfg[i].output_stride);
			if (word_per_line < 0) {
				
				word_per_line = 0;
				buffer_size_byte = 0;
			} else {
				buffer_size_byte = (word_per_line * 8 *
					stream_info->plane_cfg[i].
					output_scan_lines) - stream_info->
					plane_cfg[i].plane_addr_offset;
			}

			paddr = buf->mapped_info[i].paddr;
			ISP_DBG(
			"%s: vfe %d config buf %d to pingpong %d stream %x\n",
				__func__, vfe_dev->pdev->id,
				buf->buf_idx, !pingpong_bit,
				stream_info->stream_id);
		}

		if (dual_vfe_res) {
			for (vfe_id = 0; vfe_id < MAX_VFE; vfe_id++) {
				if (vfe_id != vfe_dev->pdev->id)
					spin_lock_irqsave(
						&dual_vfe_res->
						axi_data[vfe_id]->
						stream_info[stream_idx].
						lock, flags);

				if (buf)
					vfe_dev->hw_info->vfe_ops.axi_ops.
						update_ping_pong_addr(
						dual_vfe_res->vfe_base[vfe_id],
						dual_vfe_res->axi_data[vfe_id]->
						stream_info[stream_idx].wm[i],
						pingpong_bit, paddr +
						dual_vfe_res->axi_data[vfe_id]->
						stream_info[stream_idx].
						plane_cfg[i].plane_addr_offset,
						buffer_size_byte);
				else
					msm_isp_cfg_stream_scratch(
						dual_vfe_res->vfe_dev[vfe_id],
						&(dual_vfe_res->axi_data
						[vfe_id]->
						stream_info[stream_idx]),
						pingpong_status);

				if (i == 0) {
					dual_vfe_res->axi_data[vfe_id]->
						stream_info[stream_idx].
						buf[!pingpong_bit] =
						buf;
				}
				if (vfe_id != vfe_dev->pdev->id)
					spin_unlock_irqrestore(
						&dual_vfe_res->
						axi_data[vfe_id]->
						stream_info[stream_idx].
						lock, flags);
			}
		} else {
			if (buf)
				vfe_dev->hw_info->vfe_ops.axi_ops.
					update_ping_pong_addr(
					vfe_dev->vfe_base, stream_info->wm[i],
					pingpong_bit, paddr +
					stream_info->plane_cfg[i].
						plane_addr_offset,
					buffer_size_byte);
			else
				msm_isp_cfg_stream_scratch(vfe_dev,
					stream_info, pingpong_status);
			if (0 == i)
				stream_info->buf[!pingpong_bit] = buf;
		}
		if (0 == i && buf)
			buf->pingpong_bit = !pingpong_bit;
	}

	return 0;
}

static void msm_isp_handle_done_buf_frame_id_mismatch(
	struct vfe_device *vfe_dev, struct msm_vfe_axi_stream *stream_info,
	struct msm_isp_buffer *buf, struct timeval *time_stamp,
	uint32_t frame_id)
{
	struct msm_isp_event_data error_event;
	int ret = 0;

	memset(&error_event, 0, sizeof(error_event));
	error_event.frame_id =
		vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id;
	error_event.u.error_info.err_type =
		ISP_ERROR_FRAME_ID_MISMATCH;
	ret = vfe_dev->buf_mgr->ops->buf_done(vfe_dev->buf_mgr,
		buf->bufq_handle, buf->buf_idx, time_stamp,
		frame_id,
		stream_info->runtime_output_format);
	if (ret == -EFAULT) {
		msm_isp_halt_send_error(vfe_dev, ISP_EVENT_BUF_FATAL_ERROR);
		return;
	}
	msm_isp_send_event(vfe_dev, ISP_EVENT_ERROR,
		&error_event);
	pr_err("%s: Error! frame id mismatch!! 1st buf frame %d,curr frame %d\n",
		__func__, buf->frame_id, frame_id);
	vfe_dev->buf_mgr->frameId_mismatch_recovery = 1;
}

static int msm_isp_process_done_buf(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, struct msm_isp_buffer *buf,
	struct timeval *time_stamp, uint32_t frame_id)
{
	int rc;
	unsigned long flags;
	struct msm_isp_event_data buf_event;
	uint32_t stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);
	uint32_t buf_src;
	uint8_t drop_frame = 0;
	struct msm_isp_bufq *bufq = NULL;
	memset(&buf_event, 0, sizeof(buf_event));

	if (stream_idx >= VFE_AXI_SRC_MAX) {
		pr_err_ratelimited("%s: Invalid stream_idx", __func__);
		return -EINVAL;
	}
>>>>>>> 0e91d2a... Nougat

	if (SRC_TO_INTF(stream_info->stream_src) >= VFE_SRC_MAX) {
		pr_err("%s: Invalid stream index, put buf back to vb2 queue\n",
			__func__);
		vfe_dev->buf_mgr->ops->put_buf(vfe_dev->buf_mgr,
			buf->bufq_handle, buf->buf_idx);
		return;
	}

	if (!buf || !ts)
		return;

	frame_id = vfe_dev->axi_data.
		src_info[SRC_TO_INTF(stream_info->stream_src)].frame_id;

	if (vfe_dev->vt_enable) {
		msm_isp_get_avtimer_ts(ts);
		time_stamp = &ts->vt_time;
	} else {
		time_stamp = &ts->buf_time;
	}

	rc = vfe_dev->buf_mgr->ops->get_buf_src(vfe_dev->buf_mgr,
					buf->bufq_handle, &buf_src);
	if (stream_info->buf_divert && rc == 0 &&
			buf_src != MSM_ISP_BUFFER_SRC_SCRATCH) {
		rc = vfe_dev->buf_mgr->ops->buf_divert(vfe_dev->buf_mgr,
			buf->bufq_handle, buf->buf_idx,
<<<<<<< HEAD
			time_stamp, frame_id);
		if (rc)
			return;

		buf_event.input_intf = SRC_TO_INTF(stream_info->stream_src);
		buf_event.frame_id = frame_id;
		buf_event.timestamp = *time_stamp;
		buf_event.u.buf_done.session_id = stream_info->session_id;
		buf_event.u.buf_done.stream_id = stream_info->stream_id;
		buf_event.u.buf_done.handle = buf->bufq_handle;
		buf_event.u.buf_done.buf_idx = buf->buf_idx;
		buf_event.u.buf_done.output_format =
			stream_info->runtime_output_format;

		
		if (buf_event.frame_id < 10 || !(buf_event.frame_id%30))
			pr_info("%s: ISP_EVENT_BUF_DIVERT Session id %d, Stream id %d, Frame id %d\n",
				__func__, buf_event.u.buf_done.session_id, buf_event.u.buf_done.stream_id, buf_event.frame_id);
		
		msm_isp_send_event(vfe_dev, ISP_EVENT_BUF_DIVERT + stream_idx,
			&buf_event);
	} else {
		buf_event.input_intf = SRC_TO_INTF(stream_info->stream_src);
		buf_event.frame_id = frame_id;
		buf_event.timestamp = ts->buf_time;
		buf_event.u.buf_done.session_id = stream_info->session_id;
		buf_event.u.buf_done.stream_id = stream_info->stream_id;
		buf_event.u.buf_done.output_format =
			stream_info->runtime_output_format;

		
		if ((buf_event.u.buf_done.stream_id != 3 && buf_event.u.buf_done.stream_id <= 6) ||
			buf_event.frame_id < 10 || !(buf_event.frame_id%30))
			pr_info("%s: ISP_EVENT_BUF_DONE Session id %d, Stream id %d, Frame id %d\n",
				__func__, buf_event.u.buf_done.session_id, buf_event.u.buf_done.stream_id, buf_event.frame_id);
		
		msm_isp_send_event(vfe_dev, ISP_EVENT_BUF_DONE, &buf_event);
		vfe_dev->buf_mgr->ops->buf_done(vfe_dev->buf_mgr,
			buf->bufq_handle, buf->buf_idx, time_stamp, frame_id,
			stream_info->runtime_output_format);
=======
			time_stamp, frame_id,
			stream_info->runtime_output_format);

		if (rc == -EFAULT) {
			msm_isp_halt_send_error(vfe_dev,
					ISP_EVENT_BUF_FATAL_ERROR);
			return rc;
		}
		if (!rc) {
			ISP_DBG("%s:%d vfe_id %d Buffer dropped %d\n",
				__func__, __LINE__, vfe_dev->pdev->id,
				frame_id);
			return rc;
		}
	}

	buf_event.frame_id = frame_id;
	buf_event.timestamp = *time_stamp;
	buf_event.u.buf_done.session_id = stream_info->session_id;
	buf_event.u.buf_done.stream_id = stream_info->stream_id;
	buf_event.u.buf_done.handle = buf->bufq_handle;
	buf_event.u.buf_done.buf_idx = buf->buf_idx;
	buf_event.u.buf_done.output_format =
		stream_info->runtime_output_format;
	if (vfe_dev->fetch_engine_info.is_busy &&
		SRC_TO_INTF(stream_info->stream_src) == VFE_PIX_0) {
		vfe_dev->fetch_engine_info.is_busy = 0;
	}

	if (stream_info->buf_divert &&
		buf_src != MSM_ISP_BUFFER_SRC_SCRATCH) {
		bufq = vfe_dev->buf_mgr->ops->get_bufq(vfe_dev->buf_mgr,
			buf->bufq_handle);
		if (!bufq) {
			pr_err("%s: Invalid bufq buf_handle %x\n",
				__func__, buf->bufq_handle);
			return -EINVAL;
		}
		if ((bufq != NULL) && bufq->buf_type == ISP_SHARE_BUF)
			msm_isp_send_event(vfe_dev->common_data->
				dual_vfe_res->vfe_dev[ISP_VFE1],
				ISP_EVENT_BUF_DIVERT, &buf_event);
		else
			msm_isp_send_event(vfe_dev,
			ISP_EVENT_BUF_DIVERT, &buf_event);
	} else {
		ISP_DBG("%s: vfe_id %d send buf done buf-id %d bufq %x\n",
			__func__, vfe_dev->pdev->id, buf->buf_idx,
			buf->bufq_handle);
		msm_isp_send_event(vfe_dev, ISP_EVENT_BUF_DONE,
			&buf_event);
		buf->buf_debug.put_state[
			buf->buf_debug.put_state_last] =
			MSM_ISP_BUFFER_STATE_PUT_BUF;
		buf->buf_debug.put_state_last ^= 1;
		rc = vfe_dev->buf_mgr->ops->buf_done(vfe_dev->buf_mgr,
			buf->bufq_handle, buf->buf_idx, time_stamp,
			frame_id, stream_info->runtime_output_format);
		if (rc == -EFAULT) {
			msm_isp_halt_send_error(vfe_dev,
					ISP_EVENT_BUF_FATAL_ERROR);
			return rc;
		}
	}

	return 0;
}

int msm_isp_drop_frame(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, struct msm_isp_timestamp *ts,
	struct msm_isp_sof_info *sof_info)
{
	struct msm_isp_buffer *done_buf = NULL;
	uint32_t pingpong_status;
	unsigned long flags;
	struct msm_isp_bufq *bufq = NULL;
	uint32_t pingpong_bit;

	if (!vfe_dev || !stream_info || !ts || !sof_info) {
		pr_err("%s %d vfe_dev %pK stream_info %pK ts %pK op_info %pK\n",
			 __func__, __LINE__, vfe_dev, stream_info, ts,
			sof_info);
		return -EINVAL;
	}
	pingpong_status =
		~vfe_dev->hw_info->vfe_ops.axi_ops.get_pingpong_status(vfe_dev);

	spin_lock_irqsave(&stream_info->lock, flags);
	pingpong_bit = (~(pingpong_status >> stream_info->wm[0]) & 0x1);
	done_buf = stream_info->buf[pingpong_bit];
	if (done_buf) {
		bufq = vfe_dev->buf_mgr->ops->get_bufq(vfe_dev->buf_mgr,
			done_buf->bufq_handle);
		if (!bufq) {
			spin_unlock_irqrestore(&stream_info->lock, flags);
			pr_err("%s: Invalid bufq buf_handle %x\n",
				__func__, done_buf->bufq_handle);
			return -EINVAL;
		}
		sof_info->reg_update_fail_mask_ext |=
			(bufq->bufq_handle & 0xFF);
	}
	spin_unlock_irqrestore(&stream_info->lock, flags);

	
	if (stream_info->activated_framedrop_period ==
		MSM_VFE_STREAM_STOP_PERIOD) {
		
		msm_isp_process_axi_irq_stream(vfe_dev, stream_info,
			pingpong_status, ts);
>>>>>>> 0e91d2a... Nougat
	}
}

static void msm_isp_get_camif_update_state_and_halt(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd,
	enum msm_isp_camif_update_state *camif_update,
	int *halt)
{
	int i;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	uint8_t pix_stream_cnt = 0, cur_pix_stream_cnt;
	cur_pix_stream_cnt =
		axi_data->src_info[VFE_PIX_0].pix_stream_count +
		axi_data->src_info[VFE_PIX_0].raw_stream_count;
	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		stream_info =
			&axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
		if (stream_info->stream_src  < RDI_INTF_0)
			pix_stream_cnt++;
	}

	if (vfe_dev->axi_data.num_active_stream == stream_cfg_cmd->num_streams
		&& (stream_cfg_cmd->cmd == STOP_STREAM ||
		stream_cfg_cmd->cmd == STOP_IMMEDIATELY))
		*halt = 1;
	else
		*halt = 0;

	if ((pix_stream_cnt) &&
		(axi_data->src_info[VFE_PIX_0].input_mux != EXTERNAL_READ)) {
		if (cur_pix_stream_cnt == 0 && pix_stream_cnt &&
			stream_cfg_cmd->cmd == START_STREAM)
			*camif_update = ENABLE_CAMIF;
		else if (cur_pix_stream_cnt &&
			(cur_pix_stream_cnt - pix_stream_cnt) == 0 &&
			(stream_cfg_cmd->cmd == STOP_STREAM ||
			stream_cfg_cmd->cmd == STOP_IMMEDIATELY)) {
			if (*halt)
				*camif_update = DISABLE_CAMIF_IMMEDIATELY;
			else
				*camif_update = DISABLE_CAMIF;
		}
		else
			*camif_update = NO_UPDATE;
	} else
		*camif_update = NO_UPDATE;
}

static void msm_isp_update_camif_output_count(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd)
{
	int i;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;

	if (stream_cfg_cmd->num_streams > MAX_NUM_STREAM)
		return;

	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		if (HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])
		> MAX_NUM_STREAM) {
			return;
		}
		stream_info =
			&axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
		if (stream_info->stream_src >= RDI_INTF_0)
			continue;
		if (stream_info->stream_src == PIX_ENCODER ||
			stream_info->stream_src == PIX_VIEWFINDER ||
			stream_info->stream_src == PIX_VIDEO ||
			stream_info->stream_src == IDEAL_RAW) {
			if (stream_cfg_cmd->cmd == START_STREAM)
				vfe_dev->axi_data.src_info[VFE_PIX_0].
					pix_stream_count++;
			else
				vfe_dev->axi_data.src_info[VFE_PIX_0].
					pix_stream_count--;
		} else if (stream_info->stream_src == CAMIF_RAW) {
			if (stream_cfg_cmd->cmd == START_STREAM)
				vfe_dev->axi_data.src_info[VFE_PIX_0].
					raw_stream_count++;
			else
				vfe_dev->axi_data.src_info[VFE_PIX_0].
					raw_stream_count--;
		}
	}
}

#define ISP_DEFAULT_FORMAT_FACTOR 6
#define ISP_BUS_UTILIZATION_FACTOR 6
static int msm_isp_update_stream_bandwidth(struct vfe_device *vfe_dev)
{
	int i, rc = 0;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	uint64_t total_pix_bandwidth = 0, total_rdi_bandwidth = 0;
	uint32_t num_pix_streams = 0;
	uint64_t total_bandwidth = 0;

	for (i = 0; i < MAX_NUM_STREAM; i++) {
		stream_info = &axi_data->stream_info[i];
		if (stream_info->state == ACTIVE ||
			stream_info->state == START_PENDING) {
			if (stream_info->stream_src < RDI_INTF_0) {
				total_pix_bandwidth += stream_info->bandwidth;
				num_pix_streams++;
			} else {
				total_rdi_bandwidth += stream_info->bandwidth;
			}
		}
	}
	total_bandwidth = total_pix_bandwidth + total_rdi_bandwidth;
	rc = msm_isp_update_bandwidth(ISP_VFE0 + vfe_dev->pdev->id,
<<<<<<< HEAD
		total_bandwidth,  total_bandwidth *
		ISP_BUS_UTILIZATION_FACTOR / ISP_Q2);
=======
		(total_bandwidth + vfe_dev->hw_info->min_ab),
		(total_bandwidth + vfe_dev->hw_info->min_ib));

>>>>>>> 0e91d2a... Nougat
	if (rc < 0)
		pr_err("%s: update failed\n", __func__);

	return rc;
}

static int msm_isp_axi_wait_for_cfg_done(struct vfe_device *vfe_dev,
	enum msm_isp_camif_update_state camif_update, uint32_t src_mask)
{
	int rc;
	unsigned long flags;
	enum msm_vfe_input_src i = 0;
	spin_lock_irqsave(&vfe_dev->shared_data_lock, flags);

	for (i = 0; i < VFE_SRC_MAX; i++) {
		if (src_mask & (1 << i))
			vfe_dev->axi_data.stream_update[i] = 2;
	}
	if (src_mask) {
		init_completion(&vfe_dev->stream_config_complete);
		vfe_dev->axi_data.pipeline_update = camif_update;
	}
	spin_unlock_irqrestore(&vfe_dev->shared_data_lock, flags);
	
	if(g_subcam_no_ack == 1 && vfe_dev->pdev->id == g_subcam_vfe_intf)
	{
		rc = wait_for_completion_interruptible_timeout(
			&vfe_dev->stream_config_complete,
			msecs_to_jiffies(100));
	}
	else
	
	rc = wait_for_completion_timeout(
		&vfe_dev->stream_config_complete,
		msecs_to_jiffies(VFE_MAX_CFG_TIMEOUT));
	if (rc == 0) {
		for (i = 0; i < VFE_SRC_MAX; i++) {
			if (src_mask & (1 << i))
				vfe_dev->axi_data.stream_update[i] = 0;
		}
		pr_err("%s: wait timeout\n", __func__);
		rc = -EBUSY;
	} else {
		rc = 0;
	}
	return rc;
}

static int msm_isp_init_stream_ping_pong_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	int rc = 0;
	
	rc = msm_isp_cfg_ping_pong_address(vfe_dev,
		stream_info, VFE_PING_FLAG);
	if (rc < 0) {
		pr_err("%s: No free buffer for ping\n",
			   __func__);
		return rc;
	}

<<<<<<< HEAD
	if (stream_info->stream_type == BURST_STREAM &&
		stream_info->runtime_num_burst_capture <= 1) {
		rc = msm_isp_cfg_pong_address(vfe_dev, stream_info);
		if (rc < 0) {
			pr_err("%s: No free buffer for pong\n",
				   __func__);
			return rc;
		}
	} else {
		rc = msm_isp_cfg_ping_pong_address(vfe_dev,
			stream_info, VFE_PONG_FLAG);
		if (rc < 0) {
			pr_err("%s: No free buffer for pong\n",
				   __func__);
			return rc;
=======
	if ((vfe_dev->is_split && vfe_dev->pdev->id == 1 &&
		stream_info->stream_src < RDI_INTF_0) ||
		!vfe_dev->is_split || stream_info->stream_src >= RDI_INTF_0) {
		if (stream_info->stream_type == BURST_STREAM) {
			
			rc = msm_isp_cfg_ping_pong_address(vfe_dev,
				stream_info, VFE_PING_FLAG, 0);
			if (rc < 0) {
				pr_err("%s: No free buffer for ping\n",
					   __func__);
				return rc;
			}

			if (stream_info->runtime_num_burst_capture > 1)
				rc = msm_isp_cfg_ping_pong_address(vfe_dev,
					stream_info, VFE_PONG_FLAG, 0);

			if (rc < 0) {
				pr_err("%s: No free buffer for pong\n",
					__func__);
				return rc;
			}
		} else if (stream_info->stream_type == CONTINUOUS_STREAM) {
			rc = msm_isp_cfg_ping_pong_address(vfe_dev,
				stream_info, VFE_PING_FLAG, 0);
			if (rc < 0 && rc != ENOMEM) {
				pr_err("%s: config error for ping\n",
				__func__);
				return rc;
			}
			rc = msm_isp_cfg_ping_pong_address(vfe_dev,
				stream_info, VFE_PONG_FLAG, 0);
			if (rc < 0 && rc != ENOMEM) {
				pr_err("%s: config error for pong\n",
				__func__);
				return rc;
			}
		} else {
			rc = -1;
			pr_err("%s:%d failed invalid stream type %d", __func__,
				__LINE__, stream_info->stream_type);
>>>>>>> 0e91d2a... Nougat
		}
	}
	return rc;
}

static void msm_isp_deinit_stream_ping_pong_reg(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info)
{
	int i;
	for (i = 0; i < 2; i++) {
		struct msm_isp_buffer *buf;
		buf = stream_info->buf[i];
		if (buf) {
			vfe_dev->buf_mgr->ops->put_buf(vfe_dev->buf_mgr,
				buf->bufq_handle, buf->buf_idx);
		}
	}
}

static void msm_isp_get_stream_wm_mask(
	struct msm_vfe_axi_stream *stream_info,
	uint32_t *wm_reload_mask)
{
	int i;
	for (i = 0; i < stream_info->num_planes; i++)
		*wm_reload_mask |= (1 << stream_info->wm[i]);
}

int msm_isp_axi_halt(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_halt_cmd *halt_cmd)
{
	int rc = 0;

<<<<<<< HEAD
	if (halt_cmd->overflow_detected) {
		
		if (vfe_dev->error_info.overflow_recover_irq_mask0 == 0) {
			vfe_dev->hw_info->vfe_ops.core_ops.get_irq_mask(vfe_dev,
			&vfe_dev->error_info.overflow_recover_irq_mask0,
			&vfe_dev->error_info.overflow_recover_irq_mask1);
		}
		atomic_set(&vfe_dev->error_info.overflow_state,
			OVERFLOW_DETECTED);
=======
	if (halt_cmd->stop_camif) {
		vfe_dev->hw_info->vfe_ops.core_ops.
		update_camif_state(vfe_dev, DISABLE_CAMIF_IMMEDIATELY);
>>>>>>> 0e91d2a... Nougat
	}

	rc = vfe_dev->hw_info->vfe_ops.axi_ops.halt(vfe_dev, 1);

	return rc;
}

int msm_isp_axi_reset(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_reset_cmd *reset_cmd)
{
	int rc = 0, i, j;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	struct msm_isp_bufq *bufq = NULL;
	uint32_t bufq_handle = 0, bufq_id = 0;

	if (!reset_cmd) {
		pr_err("%s: NULL pointer reset cmd %pK\n", __func__, reset_cmd);
		rc = -1;
		return rc;
	}

	
	msm_isp_flush_tasklet(vfe_dev);

	rc = vfe_dev->hw_info->vfe_ops.core_ops.reset_hw(vfe_dev,
		0, reset_cmd->blocking);

<<<<<<< HEAD
=======
	msm_isp_get_timestamp(&timestamp, vfe_dev);

>>>>>>> 0e91d2a... Nougat
	for (i = 0, j = 0; j < axi_data->num_active_stream &&
		i < MAX_NUM_STREAM; i++, j++) {
		stream_info = &axi_data->stream_info[i];
		if (stream_info->stream_src >= VFE_AXI_SRC_MAX) {
			rc = -1;
			pr_err("%s invalid  stream src = %d\n", __func__,
				stream_info->stream_src);
			break;
		}
		if (stream_info->state != ACTIVE) {
			j--;
			continue;
		}

		for (bufq_id = 0; bufq_id < VFE_BUF_QUEUE_MAX; bufq_id++) {
			bufq_handle = stream_info->bufq_handle[bufq_id];
			if (!bufq_handle)
				continue;

			bufq = vfe_dev->buf_mgr->ops->get_bufq(vfe_dev->buf_mgr,
				bufq_handle);
			if (!bufq) {
				pr_err("%s: bufq null %p by handle %x\n",
					__func__, bufq, bufq_handle);
				continue;
			}

			if (bufq->buf_type != ISP_SHARE_BUF) {
				msm_isp_deinit_stream_ping_pong_reg(vfe_dev,
					stream_info);
			} else {
				vfe_dev->buf_mgr->ops->flush_buf(
					vfe_dev->buf_mgr, bufq_handle,
					MSM_ISP_BUFFER_FLUSH_ALL);
			}
			axi_data->src_info[SRC_TO_INTF(stream_info->
				stream_src)]. frame_id = reset_cmd->frame_id;
			msm_isp_reset_burst_count_and_frame_drop(vfe_dev,
				stream_info);
		}
	}

	if (rc < 0)
		pr_err("%s Error! reset hw Timed out\n", __func__);

	return rc;
}

int msm_isp_axi_restart(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_restart_cmd *restart_cmd)
{
	int rc = 0, i, j;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	uint32_t wm_reload_mask = 0x0;

<<<<<<< HEAD
=======
	
	spin_lock_irqsave(
		&vfe_dev->common_data->common_dev_data_lock, flags);
	vfe_dev->common_data->dual_vfe_res->epoch_sync_mask = 0;
	spin_unlock_irqrestore(
		&vfe_dev->common_data->common_dev_data_lock, flags);

	vfe_dev->buf_mgr->frameId_mismatch_recovery = 0;


>>>>>>> 0e91d2a... Nougat
	for (i = 0, j = 0; j < axi_data->num_active_stream &&
		i < MAX_NUM_STREAM; i++, j++) {
		stream_info = &axi_data->stream_info[i];
		if (stream_info->state != ACTIVE) {
			j--;
			continue;
		}
		msm_isp_get_stream_wm_mask(stream_info, &wm_reload_mask);
		msm_isp_init_stream_ping_pong_reg(vfe_dev, stream_info);
	}

	vfe_dev->hw_info->vfe_ops.axi_ops.reload_wm(vfe_dev, wm_reload_mask);
	rc = vfe_dev->hw_info->vfe_ops.axi_ops.restart(vfe_dev, 0,
		restart_cmd->enable_camif);
	if (rc < 0)
		pr_err("%s Error restarting vfe %d HW\n",
			__func__, vfe_dev->pdev->id);

	return rc;
}

static int msm_isp_axi_update_cgc_override(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd,
	uint8_t cgc_override)
{
	int i = 0, j = 0;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;

	if (stream_cfg_cmd->num_streams > MAX_NUM_STREAM)
		return -EINVAL;

	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		if (HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])
		> MAX_NUM_STREAM) {
			return -EINVAL;
		}
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
		for (j = 0; j < stream_info->num_planes; j++) {
			if (vfe_dev->hw_info->vfe_ops.axi_ops.
				update_cgc_override)
				vfe_dev->hw_info->vfe_ops.axi_ops.
					update_cgc_override(vfe_dev,
					stream_info->wm[j], cgc_override);
		}
	}
	return 0;
}

<<<<<<< HEAD
=======
static int msm_isp_update_dual_HW_ms_info_at_start(
	struct vfe_device *vfe_dev,
	enum msm_vfe_input_src stream_src)
{
	int rc = 0;
	uint32_t j, k, max_sof = 0;
	uint8_t slave_id;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	struct msm_vfe_src_info *src_info = NULL;
	uint32_t vfe_id = 0;
	unsigned long flags;

	if (stream_src >= VFE_SRC_MAX) {
		pr_err("%s: Error! Invalid src %u\n", __func__, stream_src);
		return -EINVAL;
	}

	src_info = &axi_data->src_info[stream_src];
	if (src_info->dual_hw_type != DUAL_HW_MASTER_SLAVE)
		return rc;

	spin_lock_irqsave(&vfe_dev->common_data->common_dev_data_lock, flags);
	if (src_info->dual_hw_ms_info.dual_hw_ms_type ==
		MS_TYPE_MASTER) {
		if (vfe_dev->common_data->ms_resource.master_active == 1) {
			spin_unlock_irqrestore(&vfe_dev->common_data->
				common_dev_data_lock, flags);
			return rc;
		}

		vfe_dev->common_data->ms_resource.master_active = 1;

		if (!vfe_dev->common_data->ms_resource.slave_active_mask) {
			spin_unlock_irqrestore(&vfe_dev->common_data->
				common_dev_data_lock, flags);
			return rc;
		}

		for (j = 0, k = 0; k < MS_NUM_SLAVE_MAX; k++) {
			if (!(vfe_dev->common_data->ms_resource.
				reserved_slave_mask & (1 << k)))
				continue;

			if (vfe_dev->common_data->ms_resource.slave_active_mask
				& (1 << k) &&
				(vfe_dev->common_data->ms_resource.
					slave_sof_info[k].frame_id > max_sof)) {
				max_sof = vfe_dev->common_data->ms_resource.
					slave_sof_info[k].frame_id;
			}
			j++;
			if (j == vfe_dev->common_data->ms_resource.num_slave)
				break;
		}
		vfe_dev->axi_data.src_info[stream_src].frame_id =
			max_sof + 1;
		if (vfe_dev->is_split) {
			vfe_id = vfe_dev->pdev->id;
			vfe_id = (vfe_id == 0) ? 1 : 0;
			vfe_dev->common_data->dual_vfe_res->axi_data[vfe_id]->
				src_info[stream_src].frame_id = max_sof + 1;
		}

		ISP_DBG("%s: Setting Master frame_id to %u\n", __func__,
			max_sof + 1);
	} else {
		if (src_info->dual_hw_ms_info.sof_info != NULL) {
			slave_id = src_info->dual_hw_ms_info.slave_id;
			vfe_dev->common_data->ms_resource.slave_active_mask |=
				(1 << slave_id);
		}
	}
	spin_unlock_irqrestore(&vfe_dev->common_data->common_dev_data_lock,
		flags);

	return rc;
}

static int msm_isp_update_dual_HW_ms_info_at_stop(
	struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd,
	enum msm_isp_camif_update_state camif_update)
{
	int i, rc = 0;
	uint8_t slave_id;
	struct msm_vfe_axi_stream *stream_info = NULL;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	enum msm_vfe_input_src stream_src = VFE_SRC_MAX;
	struct msm_vfe_src_info *src_info = NULL;
	unsigned long flags;

	if (stream_cfg_cmd->num_streams > MAX_NUM_STREAM ||
		stream_cfg_cmd->num_streams == 0)
		return -EINVAL;

	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		if (HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i]) >=
			VFE_AXI_SRC_MAX) {
			return -EINVAL;
		}
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
		stream_src = SRC_TO_INTF(stream_info->stream_src);

		
		if (stream_src == VFE_PIX_0 && !((camif_update == DISABLE_CAMIF)
			|| (camif_update == DISABLE_CAMIF_IMMEDIATELY)))
			continue;

		src_info = &axi_data->src_info[stream_src];
		if (src_info->dual_hw_type != DUAL_HW_MASTER_SLAVE)
			continue;

		spin_lock_irqsave(
			&vfe_dev->common_data->common_dev_data_lock,
			flags);
		if (src_info->dual_hw_ms_info.dual_hw_ms_type ==
			MS_TYPE_MASTER) {
			vfe_dev->common_data->ms_resource.master_active = 0;
		} else {
			slave_id = src_info->dual_hw_ms_info.slave_id;
			vfe_dev->common_data->ms_resource.reserved_slave_mask &=
				~(1 << slave_id);
			vfe_dev->common_data->ms_resource.slave_active_mask &=
				~(1 << slave_id);
			vfe_dev->common_data->ms_resource.num_slave--;
		}
		src_info->dual_hw_ms_info.sof_info = NULL;
		spin_unlock_irqrestore(
			&vfe_dev->common_data->common_dev_data_lock,
			flags);
		vfe_dev->vfe_ub_policy = 0;
	}

	return rc;
}

static int msm_isp_update_dual_HW_axi(struct vfe_device *vfe_dev,
			struct msm_vfe_axi_stream *stream_info)
{
	int rc = 0;
	int vfe_id;
	uint32_t stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);
	struct dual_vfe_resource *dual_vfe_res = NULL;

	if (stream_idx >= VFE_AXI_SRC_MAX) {
		pr_err("%s: Invalid stream idx %d\n", __func__, stream_idx);
		return -EINVAL;
	}

	dual_vfe_res = vfe_dev->common_data->dual_vfe_res;
	if (vfe_dev->is_split) {
		if (!dual_vfe_res->vfe_dev[ISP_VFE0] ||
			!dual_vfe_res->vfe_dev[ISP_VFE1] ||
			!dual_vfe_res->axi_data[ISP_VFE0] ||
			!dual_vfe_res->axi_data[ISP_VFE1]) {
			pr_err("%s: Error in dual vfe resource\n", __func__);
			rc = -EINVAL;
		} else {
			if (stream_info->state == RESUME_PENDING &&
				(dual_vfe_res->axi_data[!vfe_dev->pdev->id]->
				stream_info[stream_idx].state ==
				RESUME_PENDING)) {
				for (vfe_id = 0; vfe_id < MAX_VFE; vfe_id++) {
					rc = msm_isp_axi_stream_enable_cfg(
						dual_vfe_res->vfe_dev[vfe_id],
						&dual_vfe_res->
						axi_data[vfe_id]->
						stream_info[stream_idx], 1);
					dual_vfe_res->axi_data[vfe_id]->
						stream_info[stream_idx].state =
						RESUMING;
				}
			} else if (stream_info->state == RESUMING &&
				(dual_vfe_res->axi_data[!vfe_dev->pdev->id]->
				 stream_info[stream_idx].state == RESUMING)) {
				for (vfe_id = 0; vfe_id < MAX_VFE; vfe_id++) {
					dual_vfe_res->axi_data[vfe_id]->
						stream_info[stream_idx].
						runtime_output_format =
						stream_info->output_format;
					dual_vfe_res->axi_data[vfe_id]->
						stream_info[stream_idx].state =
						ACTIVE;
				}
			}

		}
	} else {
		if (stream_info->state == RESUME_PENDING) {
			msm_isp_axi_stream_enable_cfg(
				vfe_dev, stream_info, 0);
			stream_info->state = RESUMING;
		} else if (stream_info->state == RESUMING) {
			stream_info->runtime_output_format =
				stream_info->output_format;
			stream_info->state = ACTIVE;
		}
	}
	return rc;
}

>>>>>>> 0e91d2a... Nougat
static int msm_isp_start_axi_stream(struct vfe_device *vfe_dev,
			struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd,
			enum msm_isp_camif_update_state camif_update)
{
	int i, rc = 0;
	uint8_t src_state, wait_for_complete = 0;
	uint32_t wm_reload_mask = 0x0;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	uint32_t src_mask = 0;

	if (stream_cfg_cmd->num_streams > MAX_NUM_STREAM)
		return -EINVAL;

<<<<<<< HEAD
=======
	if (camif_update == ENABLE_CAMIF) {
		ISP_DBG("%s: vfe %d camif enable\n", __func__,
			vfe_dev->pdev->id);
		vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id = 0;
	}

>>>>>>> 0e91d2a... Nougat
	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		if (HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])
		> MAX_NUM_STREAM) {
			return -EINVAL;
		}
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
		if (SRC_TO_INTF(stream_info->stream_src) < VFE_SRC_MAX)
			src_state = axi_data->src_info[
				SRC_TO_INTF(stream_info->stream_src)].active;
		else {
			ISP_DBG("%s: invalid src info index\n", __func__);
			return -EINVAL;
		}

		msm_isp_calculate_bandwidth(axi_data, stream_info);
		msm_isp_reset_framedrop(vfe_dev, stream_info);
		msm_isp_get_stream_wm_mask(stream_info, &wm_reload_mask);
		rc = msm_isp_init_stream_ping_pong_reg(vfe_dev, stream_info);
		if (rc < 0) {
			pr_err("%s: No buffer for stream%d\n", __func__,
				HANDLE_TO_IDX(
				stream_cfg_cmd->stream_handle[i]));
			return rc;
		}
<<<<<<< HEAD

		stream_info->state = START_PENDING;
=======
		spin_unlock_irqrestore(&stream_info->lock, flags);
		if (stream_info->num_planes > 1) {
			vfe_dev->hw_info->vfe_ops.axi_ops.
				cfg_comp_mask(vfe_dev, stream_info);
		} else {
			vfe_dev->hw_info->vfe_ops.axi_ops.
				cfg_wm_irq_mask(vfe_dev, stream_info);
		}

		stream_info->state = START_PENDING;

		ISP_DBG("%s, Stream 0x%x src %d src_state %d on vfe %d\n",
			__func__, stream_info->stream_id,
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i]),
			src_state, vfe_dev->pdev->id);

>>>>>>> 0e91d2a... Nougat
		if (src_state) {
			src_mask |= (1 << SRC_TO_INTF(stream_info->stream_src));
			wait_for_complete = 1;
		} else {
			if (vfe_dev->dump_reg)
<<<<<<< HEAD
				msm_camera_io_dump_2(vfe_dev->vfe_base, 0x900);
=======
				msm_camera_io_dump(vfe_dev->vfe_base,
					0x1000, 1);
>>>>>>> 0e91d2a... Nougat

			
			msm_isp_axi_stream_enable_cfg(vfe_dev, stream_info);
			stream_info->state = ACTIVE;
			vfe_dev->hw_info->vfe_ops.core_ops.reg_update(vfe_dev,
				SRC_TO_INTF(stream_info->stream_src));
		}
		if (SRC_TO_INTF(stream_info->stream_src) >= VFE_RAW_0 &&
			SRC_TO_INTF(stream_info->stream_src) < VFE_SRC_MAX) {
			vfe_dev->axi_data.src_info[SRC_TO_INTF(
				stream_info->stream_src)].frame_id = 0;
		}
	}
	msm_isp_update_stream_bandwidth(vfe_dev);
	vfe_dev->hw_info->vfe_ops.axi_ops.reload_wm(vfe_dev, wm_reload_mask);
	msm_isp_update_camif_output_count(vfe_dev, stream_cfg_cmd);

	if (camif_update == ENABLE_CAMIF) {
		atomic_set(&vfe_dev->error_info.overflow_state,
				NO_OVERFLOW);
		vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id = 0;
		vfe_dev->hw_info->vfe_ops.core_ops.
			update_camif_state(vfe_dev, camif_update);
<<<<<<< HEAD
=======
		vfe_dev->axi_data.camif_state = CAMIF_ENABLE;
		vfe_dev->common_data->dual_vfe_res->epoch_sync_mask = 0;
>>>>>>> 0e91d2a... Nougat
	}

	if (wait_for_complete) {
		rc = msm_isp_axi_wait_for_cfg_done(vfe_dev, camif_update,
<<<<<<< HEAD
			src_mask);
		if (rc < 0)
=======
			src_mask, 2, 0);
#else
		rc = msm_isp_axi_wait_for_cfg_done(vfe_dev, camif_update,
			src_mask, 2);
#endif
		if (rc < 0) {
>>>>>>> 0e91d2a... Nougat
			pr_err("%s: wait for config done failed\n", __func__);
			for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
				stream_info = &axi_data->stream_info[
					HANDLE_TO_IDX(
					stream_cfg_cmd->stream_handle[i])];
				stream_info->state = STOPPING;
				msm_isp_axi_stream_enable_cfg(
					vfe_dev, stream_info, 0);
				stream_cfg_cmd->cmd = STOP_IMMEDIATELY;
				msm_isp_update_camif_output_count(vfe_dev,
					stream_cfg_cmd);
			}
		}
	}

	return rc;
}

static int msm_isp_stop_axi_stream(struct vfe_device *vfe_dev,
			struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd,
			enum msm_isp_camif_update_state camif_update,
			int halt)
{
	int i, rc = 0;
	uint8_t wait_for_complete_for_this_stream = 0;
	struct msm_vfe_axi_stream *stream_info = NULL;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	int ext_read =
		(axi_data->src_info[VFE_PIX_0].input_mux == EXTERNAL_READ);
	uint32_t src_mask = 0;

	if (stream_cfg_cmd->num_streams > MAX_NUM_STREAM ||
		stream_cfg_cmd->num_streams == 0)
		return -EINVAL;

<<<<<<< HEAD
=======
	msm_isp_get_timestamp(&timestamp, vfe_dev);

>>>>>>> 0e91d2a... Nougat
	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		if (HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])
		> MAX_NUM_STREAM) {
			return -EINVAL;
		}
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
<<<<<<< HEAD

=======
		
		spin_lock_irqsave(&stream_info->lock, flags);
		msm_isp_cfg_stream_scratch(vfe_dev, stream_info, VFE_PING_FLAG);
		msm_isp_cfg_stream_scratch(vfe_dev, stream_info, VFE_PONG_FLAG);
		spin_unlock_irqrestore(&stream_info->lock, flags);
>>>>>>> 0e91d2a... Nougat
		wait_for_complete_for_this_stream = 0;

		if (stream_info->num_planes > 1)
			vfe_dev->hw_info->vfe_ops.axi_ops.
				clear_comp_mask(vfe_dev, stream_info);
		else
			vfe_dev->hw_info->vfe_ops.axi_ops.
				clear_wm_irq_mask(vfe_dev, stream_info);

		stream_info->state = STOP_PENDING;
<<<<<<< HEAD
		if (stream_info->stream_src == CAMIF_RAW ||
			stream_info->stream_src == IDEAL_RAW) {
			if ((camif_update != DISABLE_CAMIF_IMMEDIATELY) &&
				(!ext_read))
				wait_for_complete_for_this_stream = 1;

		} else if (stream_info->stream_type == BURST_STREAM &&
				stream_info->runtime_num_burst_capture == 0) {
			if (stream_info->stream_src == RDI_INTF_0 ||
				stream_info->stream_src == RDI_INTF_1 ||
				stream_info->stream_src == RDI_INTF_2)
				wait_for_complete_for_this_stream = 1;
		} else {
			if  ((camif_update != DISABLE_CAMIF_IMMEDIATELY) &&
				(!ext_read))
				wait_for_complete_for_this_stream = 1;
		}
		if (!wait_for_complete_for_this_stream) {
			msm_isp_axi_stream_enable_cfg(vfe_dev, stream_info);
=======

		if (!halt && !ext_read &&
			!(stream_info->stream_type == BURST_STREAM &&
			stream_info->runtime_num_burst_capture == 0))
			wait_for_complete_for_this_stream = 1;

		ISP_DBG("%s: stream 0x%x, vfe %d camif %d halt %d wait %d\n",
			__func__,
			stream_info->stream_id,
			vfe_dev->pdev->id,
			camif_update,
			halt,
			wait_for_complete_for_this_stream);

		intf = SRC_TO_INTF(stream_info->stream_src);
		if (!wait_for_complete_for_this_stream ||
			stream_info->state == INACTIVE ||
			!vfe_dev->axi_data.src_info[intf].active) {
			msm_isp_axi_stream_enable_cfg(vfe_dev, stream_info, 0);
>>>>>>> 0e91d2a... Nougat
			stream_info->state = INACTIVE;
			vfe_dev->hw_info->vfe_ops.core_ops.reg_update(vfe_dev,
				SRC_TO_INTF(stream_info->stream_src));
		} else
			src_mask |= (1 << SRC_TO_INTF(stream_info->stream_src));

	}

	if (src_mask) {
		rc = msm_isp_axi_wait_for_cfg_done(vfe_dev, camif_update,
			src_mask);
		if (rc < 0) {
			pr_err("%s: wait for config done failed\n", __func__);
			for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
				stream_info = &axi_data->stream_info[
				HANDLE_TO_IDX(
					stream_cfg_cmd->stream_handle[i])];
				stream_info->state = STOP_PENDING;
				msm_isp_axi_stream_enable_cfg(
					vfe_dev, stream_info);
				vfe_dev->hw_info->vfe_ops.core_ops.reg_update(
					vfe_dev,
					SRC_TO_INTF(stream_info->stream_src));
<<<<<<< HEAD
				stream_info->state = INACTIVE;
=======
#if 1
				rc = msm_isp_axi_wait_for_cfg_done(vfe_dev,
					camif_update, src_mask, 1, reduce_timeout);
#else
				rc = msm_isp_axi_wait_for_cfg_done(vfe_dev,
					camif_update, src_mask, 1);
#endif
				if (rc < 0) {
					pr_err("%s: vfe%d cfg done failed\n",
						__func__, vfe_dev->pdev->id);
					stream_info->state = INACTIVE;
				} else
					pr_err("%s: vfe%d retry success! report err!\n",
						__func__, vfe_dev->pdev->id);

				rc = -EBUSY;
			}
		}

		for (i = VFE_RAW_0; i < VFE_SRC_MAX; i++) {
			if (src_mask & (1 << i)) {
				vfe_dev->axi_data.src_info[i].active = 0;
>>>>>>> 0e91d2a... Nougat
			}
		}
	}

	if (camif_update == DISABLE_CAMIF) {
		vfe_dev->hw_info->vfe_ops.core_ops.
			update_camif_state(vfe_dev, DISABLE_CAMIF);
	} else if ((camif_update == DISABLE_CAMIF_IMMEDIATELY) ||
					(ext_read)) {
<<<<<<< HEAD
		
		vfe_dev->ignore_error = 1;
		vfe_dev->hw_info->vfe_ops.axi_ops.halt(vfe_dev, 1);
=======
>>>>>>> 0e91d2a... Nougat
		if (!ext_read)
			vfe_dev->hw_info->vfe_ops.core_ops.
				update_camif_state(vfe_dev,
						DISABLE_CAMIF_IMMEDIATELY);
<<<<<<< HEAD
		vfe_dev->hw_info->vfe_ops.core_ops.reset_hw(vfe_dev, 0, 1);
		vfe_dev->hw_info->vfe_ops.core_ops.init_hw_reg(vfe_dev);
		vfe_dev->ignore_error = 0;
=======
		vfe_dev->axi_data.camif_state = CAMIF_STOPPED;
	}
	if (halt) {
		
		vfe_dev->hw_info->vfe_ops.axi_ops.halt(vfe_dev, 1);
		vfe_dev->hw_info->vfe_ops.core_ops.reset_hw(vfe_dev, 0, 1);
		vfe_dev->hw_info->vfe_ops.core_ops.init_hw_reg(vfe_dev);
>>>>>>> 0e91d2a... Nougat
	}

	msm_isp_update_camif_output_count(vfe_dev, stream_cfg_cmd);
	msm_isp_update_stream_bandwidth(vfe_dev);

	for (i = 0; i < stream_cfg_cmd->num_streams; i++) {
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(stream_cfg_cmd->stream_handle[i])];
		msm_isp_deinit_stream_ping_pong_reg(vfe_dev, stream_info);
	}

	return rc;
}


int msm_isp_cfg_axi_stream(struct vfe_device *vfe_dev, void *arg)
{
	int rc = 0;
	struct msm_vfe_axi_stream_cfg_cmd *stream_cfg_cmd = arg;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	enum msm_isp_camif_update_state camif_update;
	int halt = 0;

	rc = msm_isp_axi_check_stream_state(vfe_dev, stream_cfg_cmd);
	if (rc < 0) {
		pr_err("%s: Invalid stream state\n", __func__);
		return rc;
	}

	if (axi_data->num_active_stream == 0) {
		
		vfe_dev->hw_info->vfe_ops.axi_ops.cfg_ub(vfe_dev);
	}
<<<<<<< HEAD
	camif_update = msm_isp_get_camif_update_state(vfe_dev, stream_cfg_cmd);

=======
	msm_isp_get_camif_update_state_and_halt(vfe_dev, stream_cfg_cmd,
		&camif_update, &halt);
	if (camif_update == DISABLE_CAMIF)
		vfe_dev->axi_data.camif_state = CAMIF_STOPPING;
>>>>>>> 0e91d2a... Nougat
	if (stream_cfg_cmd->cmd == START_STREAM) {
		msm_isp_axi_update_cgc_override(vfe_dev, stream_cfg_cmd, 1);

		rc = msm_isp_start_axi_stream(
		   vfe_dev, stream_cfg_cmd, camif_update);
	} else {
		rc = msm_isp_stop_axi_stream(
<<<<<<< HEAD
		   vfe_dev, stream_cfg_cmd, camif_update);
=======
			vfe_dev, stream_cfg_cmd, camif_update, halt);
>>>>>>> 0e91d2a... Nougat

		msm_isp_axi_update_cgc_override(vfe_dev, stream_cfg_cmd, 0);
	}

	if (rc < 0)
		pr_err("%s: start/stop %d stream failed\n", __func__,
			stream_cfg_cmd->cmd);
	return rc;
}

<<<<<<< HEAD
=======
static int msm_isp_return_empty_buffer(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t user_stream_id,
	uint32_t frame_id, enum msm_vfe_input_src frame_src)
{
	int rc = -1;
	struct msm_isp_buffer *buf = NULL;
	uint32_t bufq_handle = 0;
	uint32_t stream_idx;
	struct msm_isp_event_data error_event;
	struct msm_isp_timestamp timestamp;

	if (!vfe_dev || !stream_info) {
		pr_err("%s %d failed: vfe_dev %pK stream_info %pK\n", __func__,
			__LINE__, vfe_dev, stream_info);
		return -EINVAL;
	}

	stream_idx = HANDLE_TO_IDX(stream_info->stream_handle);
	if (!stream_info->controllable_output)
		return -EINVAL;

	if (frame_src >= VFE_SRC_MAX) {
		pr_err("%s: Invalid frame_src %d", __func__, frame_src);
		return -EINVAL;
	}

	if (stream_idx >= VFE_AXI_SRC_MAX) {
		pr_err("%s: Invalid stream_idx", __func__);
		return rc;
	}

	if (user_stream_id == stream_info->stream_id)
		bufq_handle = stream_info->bufq_handle[VFE_BUF_QUEUE_DEFAULT];
	else
		bufq_handle = stream_info->bufq_handle[VFE_BUF_QUEUE_SHARED];


	rc = vfe_dev->buf_mgr->ops->get_buf(vfe_dev->buf_mgr,
		vfe_dev->pdev->id, bufq_handle, &buf);
	if (rc == -EFAULT) {
		msm_isp_halt_send_error(vfe_dev, ISP_EVENT_BUF_FATAL_ERROR);
		return rc;
	}

	if (rc < 0 || buf == NULL) {
		pr_err("Skip framedrop report due to no buffer\n");
		return rc;
	}

	msm_isp_get_timestamp(&timestamp, vfe_dev);
	buf->buf_debug.put_state[buf->buf_debug.put_state_last] =
		MSM_ISP_BUFFER_STATE_DROP_REG;
	buf->buf_debug.put_state_last ^= 1;
	rc = vfe_dev->buf_mgr->ops->buf_done(vfe_dev->buf_mgr,
		buf->bufq_handle, buf->buf_idx,
		&timestamp.buf_time, frame_id,
		stream_info->runtime_output_format);
	if (rc == -EFAULT) {
		msm_isp_halt_send_error(vfe_dev,
			ISP_EVENT_BUF_FATAL_ERROR);
		return rc;
	}

	memset(&error_event, 0, sizeof(error_event));
	error_event.frame_id = frame_id;
	error_event.u.error_info.err_type = ISP_ERROR_RETURN_EMPTY_BUFFER;
	error_event.u.error_info.session_id = stream_info->session_id;
	error_event.u.error_info.stream_id_mask =
		1 << (bufq_handle & 0xFF);
	msm_isp_send_event(vfe_dev, ISP_EVENT_ERROR, &error_event);

	return 0;
}

>>>>>>> 0e91d2a... Nougat
static int msm_isp_request_frame(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t user_stream_id)
{
	struct msm_vfe_axi_stream_request_cmd stream_cfg_cmd;
	struct msm_vfe_frame_request_queue *queue_req;
	uint32_t pingpong_status, wm_reload_mask = 0x0;
	unsigned long flags;
	int rc = 0;
<<<<<<< HEAD

	if (!stream_info->controllable_output)
=======
	enum msm_vfe_input_src frame_src = 0;
	struct dual_vfe_resource *dual_vfe_res =
			vfe_dev->common_data->dual_vfe_res;
	uint32_t vfe_id = 0;
	bool dual_vfe = false;

	if (!vfe_dev || !stream_info) {
		pr_err("%s %d failed: vfe_dev %pK stream_info %pK\n", __func__,
			__LINE__, vfe_dev, stream_info);
		return -EINVAL;
	}

	if (vfe_dev->is_split) {
		if (stream_info->stream_src < RDI_INTF_0) {
			if (vfe_dev->pdev->id == ISP_VFE1) {
				dual_vfe = true;
			} else {
				
				return 0;
			}
		}
	}

	if (stream_info->stream_src >= VFE_AXI_SRC_MAX) {
		pr_err("%s:%d invalid stream src %d\n", __func__, __LINE__,
			stream_info->stream_src);
		return -EINVAL;
	}

	frame_src = SRC_TO_INTF(stream_info->stream_src);
	if (((vfe_dev->axi_data.src_info[VFE_PIX_0].active) && (frame_id <=
		vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id)) ||
		((!vfe_dev->axi_data.src_info[VFE_PIX_0].active) && (frame_id <=
		vfe_dev->axi_data.src_info[frame_src].frame_id)) ||
		stream_info->undelivered_request_cnt >= MAX_BUFFERS_IN_HW) {
#if 1
                pr_err("[CAM]%s:%d invalid request_frame %d cur frame id %d pix %d\n",
                        __func__, __LINE__, frame_id,
                        vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id,
                        vfe_dev->axi_data.src_info[VFE_PIX_0].active);
#else
		pr_debug("%s:%d invalid request_frame %d cur frame id %d pix %d\n",
			__func__, __LINE__, frame_id,
			vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id,
			vfe_dev->axi_data.src_info[VFE_PIX_0].active);
#endif

		rc = msm_isp_return_empty_buffer(vfe_dev, stream_info,
			user_stream_id, frame_id, frame_src);
		if (rc < 0)
			pr_err("%s:%d failed: return_empty_buffer src %d\n",
				__func__, __LINE__, frame_src);
		return 0;
	}
	if ((frame_src == VFE_PIX_0) && !stream_info->undelivered_request_cnt &&
		MSM_VFE_STREAM_STOP_PERIOD !=
		stream_info->activated_framedrop_period) {
#if 1
                pr_err("[CAM]%s:%d vfe %d frame_id %d prev_pattern %x stream_id %x\n",
                        __func__, __LINE__, vfe_dev->pdev->id, frame_id,
                        stream_info->activated_framedrop_period,
                        stream_info->stream_id);
#else
		pr_debug("%s:%d vfe %d frame_id %d prev_pattern %x stream_id %x\n",
			__func__, __LINE__, vfe_dev->pdev->id, frame_id,
			stream_info->activated_framedrop_period,
			stream_info->stream_id);
#endif

		rc = msm_isp_return_empty_buffer(vfe_dev, stream_info,
			user_stream_id, frame_id, frame_src);
		if (rc < 0)
			pr_err("%s:%d failed: return_empty_buffer src %d\n",
				__func__, __LINE__, frame_src);
		stream_info->current_framedrop_period =
			MSM_VFE_STREAM_STOP_PERIOD;
		msm_isp_cfg_framedrop_reg(vfe_dev, stream_info);
>>>>>>> 0e91d2a... Nougat
		return 0;

	spin_lock_irqsave(&stream_info->lock, flags);
	queue_req = &stream_info->request_queue_cmd[stream_info->request_q_idx];
	if (queue_req->cmd_used) {
		spin_unlock_irqrestore(&stream_info->lock, flags);
		pr_err_ratelimited("%s: Request queue overflow.\n", __func__);
		return -EINVAL;
	}

	if (user_stream_id == stream_info->stream_id)
		queue_req->buff_queue_id = VFE_BUF_QUEUE_DEFAULT;
	else
		queue_req->buff_queue_id = VFE_BUF_QUEUE_SHARED;

	if (!stream_info->bufq_handle[queue_req->buff_queue_id]) {
		spin_unlock_irqrestore(&stream_info->lock, flags);
		pr_err("%s:%d stream is stoped\n", __func__, __LINE__);
		return 0;
	}
	queue_req->cmd_used = 1;

	stream_info->request_q_idx =
		(stream_info->request_q_idx + 1) % MSM_VFE_REQUESTQ_SIZE;
	list_add_tail(&queue_req->list, &stream_info->request_q);
	stream_info->request_q_cnt++;

	stream_info->undelivered_request_cnt++;

	stream_cfg_cmd.axi_stream_handle = stream_info->stream_handle;
	stream_cfg_cmd.frame_skip_pattern = NO_SKIP;
	stream_cfg_cmd.init_frame_drop = 0;
	stream_cfg_cmd.burst_count = stream_info->request_q_cnt;
<<<<<<< HEAD
	msm_isp_calculate_framedrop(&vfe_dev->axi_data, &stream_cfg_cmd);
	msm_isp_reset_framedrop(vfe_dev, stream_info);

=======
>>>>>>> 0e91d2a... Nougat
	if (stream_info->undelivered_request_cnt == 1) {
		rc = msm_isp_cfg_ping_pong_address(vfe_dev, stream_info,
			VFE_PING_FLAG);
		if (rc) {
			spin_unlock_irqrestore(&stream_info->lock, flags);
			pr_err("%s:%d fail to set ping pong address\n",
				__func__, __LINE__);
			return rc;
		}

		msm_isp_get_stream_wm_mask(stream_info, &wm_reload_mask);
		vfe_dev->hw_info->vfe_ops.axi_ops.reload_wm(vfe_dev,
			wm_reload_mask);
	} else if (stream_info->undelivered_request_cnt == 2) {
		pingpong_status =
			vfe_dev->hw_info->vfe_ops.axi_ops.get_pingpong_status(
				vfe_dev);

		rc = msm_isp_cfg_ping_pong_address(vfe_dev, stream_info,
			pingpong_status);
		if (rc) {
			spin_unlock_irqrestore(&stream_info->lock, flags);
			pr_err("%s:%d fail to set ping pong address\n",
				__func__, __LINE__);
			return rc;
		}
	}

	spin_unlock_irqrestore(&stream_info->lock, flags);

	return rc;
}

static int msm_isp_add_buf_queue(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t stream_id)
{
	int rc = 0;
	uint32_t bufq_id = 0;

	if (stream_id == stream_info->stream_id)
		bufq_id = VFE_BUF_QUEUE_DEFAULT;
	else
		bufq_id = VFE_BUF_QUEUE_SHARED;

	stream_info->bufq_handle[bufq_id] =
		vfe_dev->buf_mgr->ops->get_bufq_handle(vfe_dev->buf_mgr,
			stream_info->session_id, stream_id);
	if (stream_info->bufq_handle[bufq_id] == 0) {
		pr_err("%s: failed: No valid buffer queue for stream: 0x%x\n",
			__func__, stream_id);
		rc = -EINVAL;
	}

	return rc;
}

static void msm_isp_remove_buf_queue(struct vfe_device *vfe_dev,
	struct msm_vfe_axi_stream *stream_info, uint32_t stream_id)
{
	uint32_t bufq_id = 0;
	unsigned long flags;

	if (stream_id == stream_info->stream_id)
		bufq_id = VFE_BUF_QUEUE_DEFAULT;
	else
		bufq_id = VFE_BUF_QUEUE_SHARED;

	spin_lock_irqsave(&stream_info->lock, flags);
	stream_info->bufq_handle[bufq_id] = 0;
	spin_unlock_irqrestore(&stream_info->lock, flags);

}

int msm_isp_update_axi_stream(struct vfe_device *vfe_dev, void *arg)
{
	int rc = 0, i, j;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	struct msm_vfe_axi_stream_update_cmd *update_cmd = arg;
<<<<<<< HEAD
	struct msm_vfe_axi_stream_cfg_update_info *update_info;
=======
	struct msm_vfe_axi_stream_cfg_update_info *update_info = NULL;
	struct msm_isp_sw_framskip *sw_skip_info = NULL;
	unsigned long flags;
	struct msm_isp_timestamp timestamp;
	uint32_t frame_id;
>>>>>>> 0e91d2a... Nougat

	
	if (update_cmd->num_streams > MAX_NUM_STREAM)
		return -EINVAL;

	for (i = 0; i < update_cmd->num_streams; i++) {
		update_info = (struct msm_vfe_axi_stream_cfg_update_info *)
			&update_cmd->update_info[i];
		
		if (HANDLE_TO_IDX(update_info->stream_handle)
		 > MAX_NUM_STREAM) {
			return -EINVAL;
		}
		stream_info = &axi_data->stream_info[
			HANDLE_TO_IDX(update_info->stream_handle)];
		if (SRC_TO_INTF(stream_info->stream_src) >= VFE_SRC_MAX)
			continue;
		if (stream_info->state != ACTIVE &&
			stream_info->state != INACTIVE) {
			pr_err("%s: Invalid stream state\n", __func__);
			return -EINVAL;
		}
		if (update_cmd->update_type == UPDATE_STREAM_AXI_CONFIG &&
			atomic_read(&axi_data->axi_cfg_update[
				SRC_TO_INTF(stream_info->stream_src)])) {
			pr_err("%s: AXI stream config updating\n", __func__);
			return -EBUSY;
		}
	}

	switch (update_cmd->update_type) {
	case ENABLE_STREAM_BUF_DIVERT:
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			stream_info->buf_divert = 1;
		}
		break;
	case DISABLE_STREAM_BUF_DIVERT:
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			stream_info->buf_divert = 0;
<<<<<<< HEAD
			vfe_dev->buf_mgr->ops->flush_buf(vfe_dev->buf_mgr,
				stream_info->bufq_handle[VFE_BUF_QUEUE_DEFAULT],
				MSM_ISP_BUFFER_FLUSH_DIVERTED);
			break;
		case UPDATE_STREAM_FRAMEDROP_PATTERN: {
			uint32_t framedrop_period =
				msm_isp_get_framedrop_period(
				   update_info->skip_pattern);
=======
			msm_isp_get_timestamp(&timestamp, vfe_dev);
			frame_id = vfe_dev->axi_data.src_info[
				SRC_TO_INTF(stream_info->stream_src)].frame_id;
			
			spin_lock_irqsave(&stream_info->lock, flags);
			msm_isp_cfg_stream_scratch(vfe_dev, stream_info,
					VFE_PING_FLAG);
			msm_isp_cfg_stream_scratch(vfe_dev, stream_info,
					VFE_PONG_FLAG);
			spin_unlock_irqrestore(&stream_info->lock, flags);
			rc = vfe_dev->buf_mgr->ops->flush_buf(vfe_dev->buf_mgr,
				vfe_dev->pdev->id,
				stream_info->bufq_handle[VFE_BUF_QUEUE_DEFAULT],
				MSM_ISP_BUFFER_FLUSH_DIVERTED,
				&timestamp.buf_time, frame_id);
			if (rc == -EFAULT) {
				msm_isp_halt_send_error(vfe_dev,
					ISP_EVENT_BUF_FATAL_ERROR);
				return rc;
			}
		}
		break;
	case UPDATE_STREAM_FRAMEDROP_PATTERN: {
		for (i = 0; i < update_cmd->num_streams; i++) {
			uint32_t framedrop_period =
				msm_isp_get_framedrop_period(
					update_info->skip_pattern);
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			spin_lock_irqsave(&stream_info->lock, flags);
			
			if (stream_info->current_framedrop_period ==
				framedrop_period) {
				spin_unlock_irqrestore(&stream_info->lock,
					flags);
				break;
			}
>>>>>>> 0e91d2a... Nougat
			if (stream_info->controllable_output) {
				pr_err("Controllable output streams does not support custom frame skip pattern\n");
				return -EINVAL;
			}
			if (update_info->skip_pattern == SKIP_ALL)
				stream_info->framedrop_pattern = 0x0;
			else
				stream_info->framedrop_pattern = 0x1;
			stream_info->framedrop_period = framedrop_period - 1;
			if (stream_info->stream_type == BURST_STREAM) {
				stream_info->runtime_framedrop_update_burst = 1;
			} else {
				stream_info->runtime_init_frame_drop = 0;
				msm_isp_cfg_framedrop_reg(vfe_dev, stream_info);
<<<<<<< HEAD
=======
			spin_unlock_irqrestore(&stream_info->lock, flags);
		}
		break;
	}
	case UPDATE_STREAM_SW_FRAME_DROP: {
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			sw_skip_info = &update_info->sw_skip_info;
			if (sw_skip_info->stream_src_mask != 0) {
				
				pr_debug("%s:%x sw skip type %x mode %d min %d max %d\n",
					__func__, stream_info->stream_id,
					sw_skip_info->stats_type_mask,
					sw_skip_info->skip_mode,
					sw_skip_info->min_frame_id,
					sw_skip_info->max_frame_id);
				spin_lock_irqsave(&stream_info->lock, flags);
				stream_info->sw_skip = *sw_skip_info;
				spin_unlock_irqrestore(&stream_info->lock,
					flags);
>>>>>>> 0e91d2a... Nougat
			}
		}
		break;
	}
	case UPDATE_STREAM_AXI_CONFIG: {
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			for (j = 0; j < stream_info->num_planes; j++) {
				stream_info->plane_cfg[j] =
					update_info->plane_cfg[j];
			}
<<<<<<< HEAD
			stream_info->output_format = update_info->output_format;
			if (stream_info->state == ACTIVE) {
=======
			stream_info->output_format =
				update_info->output_format;
			if ((stream_info->state == ACTIVE) &&
				((vfe_dev->hw_info->runtime_axi_update == 0) ||
				(vfe_dev->dual_vfe_enable == 1)))  {
				spin_lock_irqsave(&stream_info->lock, flags);
>>>>>>> 0e91d2a... Nougat
				stream_info->state = PAUSE_PENDING;
				msm_isp_axi_stream_enable_cfg(
					vfe_dev, stream_info);
				stream_info->state = PAUSING;
				atomic_set(&axi_data->
					axi_cfg_update[SRC_TO_INTF(
					stream_info->stream_src)],
					UPDATE_REQUESTED);
			} else {
				for (j = 0; j < stream_info->num_planes; j++)
					vfe_dev->hw_info->vfe_ops.axi_ops.
					cfg_wm_reg(vfe_dev, stream_info, j);
				stream_info->runtime_output_format =
					stream_info->output_format;
			}
		}
		break;
	}
	case UPDATE_STREAM_REQUEST_FRAMES: {
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			rc = msm_isp_request_frame(vfe_dev, stream_info,
				update_info->user_stream_id);
			if (rc)
				pr_err("%s failed to request frame!\n",
					__func__);
		}
		break;
	}
	case UPDATE_STREAM_ADD_BUFQ: {
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			rc = msm_isp_add_buf_queue(vfe_dev, stream_info,
				update_info->user_stream_id);
			if (rc)
				pr_err("%s failed to add bufq!\n", __func__);
		}
		break;
	}
	case UPDATE_STREAM_REMOVE_BUFQ: {
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			msm_isp_remove_buf_queue(vfe_dev, stream_info,
				update_info->user_stream_id);

			if (stream_info->state == ACTIVE) {
				stream_info->state = UPDATING;
				rc = msm_isp_axi_wait_for_cfg_done(vfe_dev,
					NO_UPDATE, (1 << SRC_TO_INTF(
					stream_info->stream_src)));
				if (rc < 0)
					pr_err("%s: wait for update failed\n",
						__func__);
			}
		}
		break;
	}
	case UPDATE_STREAM_OFFLINE_AXI_CONFIG: {
		for (i = 0; i < update_cmd->num_streams; i++) {
			update_info =
				(struct msm_vfe_axi_stream_cfg_update_info *)
				&update_cmd->update_info[i];
			stream_info = &axi_data->stream_info[HANDLE_TO_IDX(
				update_info->stream_handle)];
			for (j = 0; j < stream_info->num_planes; j++) {
				stream_info->plane_cfg[j] =
					update_info->plane_cfg[j];
			}
			for (j = 0; j < stream_info->num_planes; j++) {
				vfe_dev->hw_info->vfe_ops.axi_ops.
					cfg_wm_reg(vfe_dev, stream_info, j);
			}
		}
		break;
	}
	default:
		pr_err("%s: Invalid update type\n", __func__);
		return -EINVAL;
	}

	return rc;
}

<<<<<<< HEAD
=======
void msm_isp_process_axi_irq_stream(struct vfe_device *vfe_dev,
		struct msm_vfe_axi_stream *stream_info,
		uint32_t pingpong_status,
		struct msm_isp_timestamp *ts)
{
	int rc = -1;
	uint32_t pingpong_bit = 0, i;
	struct msm_isp_buffer *done_buf = NULL;
	unsigned long flags;
	struct timeval *time_stamp;
	uint32_t frame_id, buf_index = -1;
	struct msm_vfe_axi_stream *temp_stream;

	if (!ts) {
		pr_err("%s: Error! Invalid argument\n", __func__);
		return;
	}

	if (vfe_dev->vt_enable) {
		msm_isp_get_avtimer_ts(ts);
		time_stamp = &ts->vt_time;
	} else {
		time_stamp = &ts->buf_time;
	}

	frame_id = vfe_dev->axi_data.
		src_info[SRC_TO_INTF(stream_info->stream_src)].frame_id;

	spin_lock_irqsave(&stream_info->lock, flags);
	pingpong_bit = (~(pingpong_status >> stream_info->wm[0]) & 0x1);
	for (i = 0; i < stream_info->num_planes; i++) {
		if (pingpong_bit !=
			(~(pingpong_status >> stream_info->wm[i]) & 0x1)) {
			spin_unlock_irqrestore(&stream_info->lock, flags);
			pr_err("%s: Write master ping pong mismatch. Status: 0x%x\n",
				__func__, pingpong_status);
			msm_isp_halt_send_error(vfe_dev,
					ISP_EVENT_PING_PONG_MISMATCH);
			return;
		}
	}

	if (stream_info->state == INACTIVE) {
		msm_isp_cfg_stream_scratch(vfe_dev, stream_info,
					pingpong_status);
		spin_unlock_irqrestore(&stream_info->lock, flags);
		pr_err_ratelimited("%s: Warning! Stream already inactive. Drop irq handling\n",
			__func__);
		return;
	}
	done_buf = stream_info->buf[pingpong_bit];

	if (vfe_dev->buf_mgr->frameId_mismatch_recovery == 1) {
		pr_err_ratelimited("%s: Mismatch Recovery in progress, drop frame!\n",
			__func__);
		spin_unlock_irqrestore(&stream_info->lock, flags);
		return;
	}

	stream_info->frame_id++;
	if (done_buf)
		buf_index = done_buf->buf_idx;

	ISP_DBG("%s: vfe %d: stream 0x%x, frame id %d, pingpong bit %d\n",
		__func__,
		vfe_dev->pdev->id,
		stream_info->stream_id,
		frame_id,
		pingpong_bit);

	rc = vfe_dev->buf_mgr->ops->update_put_buf_cnt(vfe_dev->buf_mgr,
		vfe_dev->pdev->id,
		done_buf ? done_buf->bufq_handle :
		stream_info->bufq_handle[VFE_BUF_QUEUE_DEFAULT], buf_index,
		time_stamp, frame_id, pingpong_bit);

	if (rc < 0) {
		spin_unlock_irqrestore(&stream_info->lock, flags);
		
		msm_isp_halt_send_error(vfe_dev, ISP_EVENT_PING_PONG_MISMATCH);
		return;
	}
	if (rc != 0) {
		spin_unlock_irqrestore(&stream_info->lock, flags);
		return;
	}

	if (stream_info->stream_type == CONTINUOUS_STREAM ||
		stream_info->runtime_num_burst_capture > 1) {
		rc = msm_isp_cfg_ping_pong_address(vfe_dev,
			stream_info, pingpong_status, 0);
		if (rc < 0)
			ISP_DBG("%s: Error configuring ping_pong\n",
				__func__);
	} else if (done_buf) {
		rc = msm_isp_cfg_ping_pong_address(vfe_dev,
			stream_info, pingpong_status, 1);
		if (rc < 0)
			ISP_DBG("%s: Error configuring ping_pong\n",
				__func__);
	}

	if (!done_buf) {
		if (stream_info->buf_divert) {
			vfe_dev->error_info.stream_framedrop_count[
				stream_info->bufq_handle[
				VFE_BUF_QUEUE_DEFAULT] & 0xFF]++;
			vfe_dev->error_info.framedrop_flag = 1;
		}
		spin_unlock_irqrestore(&stream_info->lock, flags);
		return;
	}

	temp_stream = msm_isp_get_controllable_stream(vfe_dev,
					stream_info);
	if (temp_stream->stream_type == BURST_STREAM &&
		temp_stream->runtime_num_burst_capture) {
		ISP_DBG("%s: burst_frame_count: %d\n",
			__func__,
			temp_stream->runtime_num_burst_capture);
		temp_stream->runtime_num_burst_capture--;
		if (!stream_info->controllable_output && vfe_dev->is_split &&
			RDI_INTF_0 > stream_info->stream_src) {
			temp_stream = msm_isp_vfe_get_stream(
					vfe_dev->common_data->dual_vfe_res,
					((vfe_dev->pdev->id == ISP_VFE0) ?
					ISP_VFE1 : ISP_VFE0),
					HANDLE_TO_IDX(
					stream_info->stream_handle));
			temp_stream->runtime_num_burst_capture--;
		}
	}

	rc = msm_isp_update_deliver_count(vfe_dev, stream_info,
					pingpong_bit);
	if (rc) {
		spin_unlock_irqrestore(&stream_info->lock, flags);
		pr_err_ratelimited("%s:VFE%d get done buf fail\n",
			__func__, vfe_dev->pdev->id);
		msm_isp_halt_send_error(vfe_dev,
			ISP_EVENT_PING_PONG_MISMATCH);
		return;
	}

	spin_unlock_irqrestore(&stream_info->lock, flags);

	if ((done_buf->frame_id != frame_id) &&
		vfe_dev->axi_data.enable_frameid_recovery) {
		msm_isp_handle_done_buf_frame_id_mismatch(vfe_dev,
			stream_info, done_buf, time_stamp, frame_id);
		return;
	}

	msm_isp_process_done_buf(vfe_dev, stream_info,
			done_buf, time_stamp, frame_id);
}

>>>>>>> 0e91d2a... Nougat
void msm_isp_process_axi_irq(struct vfe_device *vfe_dev,
	uint32_t irq_status0, uint32_t irq_status1,
	uint32_t pingpong_status, struct msm_isp_timestamp *ts)
{
	int i, rc = 0;
	struct msm_isp_buffer *done_buf = NULL;
	uint32_t comp_mask = 0, wm_mask = 0;
	uint32_t stream_idx;
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_composite_info *comp_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	unsigned long flags;

	comp_mask = vfe_dev->hw_info->vfe_ops.axi_ops.
		get_comp_mask(irq_status0, irq_status1);
	wm_mask = vfe_dev->hw_info->vfe_ops.axi_ops.
		get_wm_mask(irq_status0, irq_status1);
	if (!(comp_mask || wm_mask))
		return;

	ISP_DBG("%s: status: 0x%x\n", __func__, irq_status0);

	for (i = 0; i < axi_data->hw_info->num_comp_mask; i++) {
		rc = 0;
		comp_info = &axi_data->composite_info[i];
		wm_mask &= ~(comp_info->stream_composite_mask);
		if (comp_mask & (1 << i)) {
			if (!comp_info->stream_handle) {
				pr_err("%s: Invalid handle for composite irq\n",
					__func__);
				continue;
			}
			stream_idx = HANDLE_TO_IDX(comp_info->stream_handle);
			stream_info = &axi_data->stream_info[stream_idx];
<<<<<<< HEAD
			ISP_DBG("%s: stream id %x frame id: 0x%x\n", __func__,
				stream_info->stream_id, stream_info->frame_id);
			stream_info->frame_id++;

			if (stream_info->stream_type == BURST_STREAM) {
				ISP_DBG("%s: burst_frame_count: %d\n",
					__func__,
					stream_info->runtime_num_burst_capture);
				stream_info->runtime_num_burst_capture--;
			}

			spin_lock_irqsave(&stream_info->lock, flags);
			msm_isp_get_done_buf(vfe_dev, stream_info,
				pingpong_status, &done_buf);
			if (stream_info->stream_type == CONTINUOUS_STREAM ||
				stream_info->runtime_num_burst_capture > 1) {
				rc = msm_isp_cfg_ping_pong_address(vfe_dev,
					stream_info, pingpong_status);
			}
			spin_unlock_irqrestore(&stream_info->lock, flags);
=======
			msm_isp_process_axi_irq_stream(vfe_dev, stream_info,
						pingpong_status, ts);
>>>>>>> 0e91d2a... Nougat

			if (done_buf && !rc)
				msm_isp_process_done_buf(vfe_dev, stream_info,
					done_buf, ts);
		}
	}

	for (i = 0; i < axi_data->hw_info->num_wm; i++) {
		if (wm_mask & (1 << i)) {
			if (!axi_data->free_wm[i]) {
				pr_err("%s: Invalid handle for wm irq\n",
					__func__);
				continue;
			}
			stream_idx = HANDLE_TO_IDX(axi_data->free_wm[i]);
			stream_info = &axi_data->stream_info[stream_idx];
			ISP_DBG("%s: stream id %x frame id: 0x%x\n", __func__,
				stream_info->stream_id, stream_info->frame_id);
			stream_info->frame_id++;

<<<<<<< HEAD
			if (stream_info->stream_type == BURST_STREAM) {
				ISP_DBG("%s: burst_frame_count: %d\n",
					__func__,
					stream_info->runtime_num_burst_capture);
				stream_info->runtime_num_burst_capture--;
			}
=======
			msm_isp_process_axi_irq_stream(vfe_dev, stream_info,
						pingpong_status, ts);
		}
	}
	return;
}

void msm_isp_axi_disable_all_wm(struct vfe_device *vfe_dev)
{
	struct msm_vfe_axi_stream *stream_info;
	struct msm_vfe_axi_shared_data *axi_data = &vfe_dev->axi_data;
	int i, j;

	if (!vfe_dev || !axi_data) {
		pr_err("%s: error  %pK %pK\n", __func__, vfe_dev, axi_data);
		return;
	}

	for (i = 0; i < VFE_AXI_SRC_MAX; i++) {
		stream_info = &axi_data->stream_info[i];
>>>>>>> 0e91d2a... Nougat

			spin_lock_irqsave(&stream_info->lock, flags);
			msm_isp_get_done_buf(vfe_dev, stream_info,
						pingpong_status, &done_buf);
			if (stream_info->stream_type == CONTINUOUS_STREAM ||
				stream_info->runtime_num_burst_capture > 1) {
				rc = msm_isp_cfg_ping_pong_address(vfe_dev,
					stream_info, pingpong_status);
			}
			spin_unlock_irqrestore(&stream_info->lock, flags);

			if (done_buf && !rc)
				msm_isp_process_done_buf(vfe_dev,
				stream_info, done_buf, ts);
		}
	}
	return;
}
