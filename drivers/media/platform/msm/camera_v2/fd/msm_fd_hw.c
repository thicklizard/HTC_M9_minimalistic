<<<<<<< HEAD
/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/msm_iommu_domains.h>
#include <linux/spinlock.h>
#include <linux/iommu.h>
#include <linux/qcom_iommu.h>
#include <linux/msm_ion.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <media/videobuf2-core.h>

#include "msm_fd_dev.h"
#include "msm_fd_hw.h"
#include "msm_fd_regs.h"
<<<<<<< HEAD

#define MSM_FD_PROCESSING_TIMEOUT_MS 500

static struct msm_iova_partition msm_fd_fw_partition = {
	.start = SZ_128K,
	.size = SZ_2G - SZ_128K,
};

static struct msm_iova_layout msm_fd_fw_layout = {
	.partitions = &msm_fd_fw_partition,
	.npartitions = 1,
	.client_name = "fd_iommu",
	.domain_flags = 0,
};

static struct msm_bus_vectors msm_fd_bandwidth_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VPU,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 450000000,
		.ib  = 900000000,
	},
};

static struct msm_bus_paths msm_fd_bus_client_config[] = {
	{
		ARRAY_SIZE(msm_fd_bandwidth_vectors),
		msm_fd_bandwidth_vectors,
	},
};

static struct msm_bus_scale_pdata msm_fd_bus_scale_data = {
	msm_fd_bus_client_config,
	ARRAY_SIZE(msm_fd_bus_client_config),
	.name = "msm_face_detect",
=======
#include "cam_smmu_api.h"
#include "msm_camera_io_util.h"

/* After which revision misc irq for engine is needed */
#define MSM_FD_MISC_IRQ_FROM_REV 0x10010000
/* Face detection workqueue name */
#define MSM_FD_WORQUEUE_NAME "face-detection"
/* Face detection bus client name */
#define MSM_FD_BUS_CLIENT_NAME "msm_face_detect"
/* Face detection processing timeout in ms */
#define MSM_FD_PROCESSING_TIMEOUT_MS 150
/* Face detection halt timeout in ms */
#define MSM_FD_HALT_TIMEOUT_MS 100
/* Smmu callback name */
#define MSM_FD_SMMU_CB_NAME "camera_fd"
/*
 * enum msm_fd_reg_setting_entries - FD register setting entries in DT.
 * @MSM_FD_REG_ADDR_OFFSET_IDX: Register address offset index.
 * @MSM_FD_REG_VALUE_IDX: Register value index.
 * @MSM_FD_REG_MASK_IDX: Regester mask index.
 * @MSM_FD_REG_LAST_IDX: Index count.
 */
enum msm_fd_dt_reg_setting_index {
	MSM_FD_REG_ADDR_OFFSET_IDX,
	MSM_FD_REG_VALUE_IDX,
	MSM_FD_REG_MASK_IDX,
	MSM_FD_REG_LAST_IDX
>>>>>>> 0e91d2a... Nougat
};

static inline u32 msm_fd_hw_read_reg(struct msm_fd_device *fd,
	enum msm_fd_mem_resources base_idx, u32 reg)
{
	return readl_relaxed(fd->iomem_base[base_idx] + reg);
}

/*
 * msm_fd_hw_read_reg - Fd write to register.
 * @fd: Pointer to fd device.
 * @base_idx: Fd memory resource index.
 * @reg: Register addr need to be read from.
 e @value: Value to be written.
 */
static inline void msm_fd_hw_write_reg(struct msm_fd_device *fd,
	enum msm_fd_mem_resources base_idx, u32 reg, u32 value)
{
	writel_relaxed(value, fd->iomem_base[base_idx] + reg);
}

static inline void msm_fd_hw_reg_clr(struct msm_fd_device *fd,
	enum msm_fd_mem_resources mmio_range, u32 reg, u32 clr_bits)
{
	u32 bits = msm_fd_hw_read_reg(fd, mmio_range, reg);

	msm_fd_hw_write_reg(fd, mmio_range, reg, (bits & ~clr_bits));
}

static inline void msm_fd_hw_reg_set(struct msm_fd_device *fd,
	enum msm_fd_mem_resources mmio_range, u32 reg, u32 set_bits)
{
	u32 bits = msm_fd_hw_read_reg(fd, mmio_range, reg);

	msm_fd_hw_write_reg(fd, mmio_range, reg, (bits | set_bits));
}

static inline void msm_fd_hw_set_size_mode(struct msm_fd_device *fd, u32 mode)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_IMAGE_SIZE, mode);
}

static inline void msm_fd_hw_set_crop(struct msm_fd_device *fd,
	struct v4l2_rect *crop)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_START_X,
		(crop->top & MSM_FD_START_X_MASK));

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_START_Y,
		(crop->left & MSM_FD_START_Y_MASK));

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_SIZE_X,
		(crop->width & MSM_FD_SIZE_X_MASK));

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_SIZE_Y,
		(crop->height & MSM_FD_SIZE_Y_MASK));
}

static inline void msm_fd_hw_set_bytesperline(struct msm_fd_device *fd, u32 b)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_LINE_BYTES,
		(b & MSM_FD_LINE_BYTES_MASK));
}

static inline void msm_fd_hw_set_image_addr(struct msm_fd_device *fd, u32 addr)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_IMAGE_ADDR, addr);
}

static inline void msm_fd_hw_set_work_addr(struct msm_fd_device *fd, u32 addr)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_WORK_ADDR, addr);
}

static inline void msm_fd_hw_set_direction_angle(struct msm_fd_device *fd,
	u32 direction, u32 angle)
{
	u32 reg;
	u32 value;

	value = direction | (angle ? 1 << (angle + 1) : 0);
	if (value > MSM_FD_CONDT_DIR_MAX)
		value = MSM_FD_CONDT_DIR_MAX;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONDT);

	reg &= ~MSM_FD_CONDT_DIR_MASK;
	reg |= (value << MSM_FD_CONDT_DIR_SHIFT);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONDT, reg);
}

static inline void msm_fd_hw_set_min_face(struct msm_fd_device *fd, u32 size)
{
	u32 reg;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONDT);

	reg &= ~MSM_FD_CONDT_MIN_MASK;
	reg |= (size << MSM_FD_CONDT_MIN_SHIFT);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONDT, reg);
}

static inline void msm_fd_hw_set_threshold(struct msm_fd_device *fd, u32 thr)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_DHINT,
		(thr & MSM_FD_DHINT_MASK));
}

static inline void msm_fd_hw_srst(struct msm_fd_device *fd)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONTROL,
		MSM_FD_CONTROL_SRST);
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONTROL, 0);
}

int msm_fd_hw_get_face_count(struct msm_fd_device *fd)
{
	u32 reg;
	u32 value;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_RESULT_CNT);

	value = reg & MSM_FD_RESULT_CNT_MASK;
	if (value > MSM_FD_MAX_FACES_DETECTED) {
		dev_warn(fd->dev, "Face count %d out of limit\n", value);
		value = MSM_FD_MAX_FACES_DETECTED;
	}

	return value;
}

static inline void msm_fd_hw_run(struct msm_fd_device *fd)
{
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONTROL,
		MSM_FD_CONTROL_RUN);
}

int msm_fd_hw_is_finished(struct msm_fd_device *fd)
{
	u32 reg;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONTROL);

	return reg & MSM_FD_CONTROL_FINISH;
}

int msm_fd_hw_is_runnig(struct msm_fd_device *fd)
{
	u32 reg;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONTROL);

	return reg & MSM_FD_CONTROL_RUN;
}

int msm_fd_hw_get_result_x(struct msm_fd_device *fd, int idx)
{
	u32 reg;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE,
		MSM_FD_RESULT_CENTER_X(idx));

	return reg;
}

int msm_fd_hw_get_result_y(struct msm_fd_device *fd, int idx)
{
	u32 reg;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE,
		MSM_FD_RESULT_CENTER_Y(idx));

	return reg;
}

void msm_fd_hw_get_result_conf_size(struct msm_fd_device *fd,
	int idx, u32 *conf, u32 *size)
{
	u32 reg;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE,
		MSM_FD_RESULT_CONF_SIZE(idx));

	*conf = (reg >> MSM_FD_RESULT_CONF_SHIFT) & MSM_FD_RESULT_CONF_MASK;
	*size = (reg >> MSM_FD_RESULT_SIZE_SHIFT) & MSM_FD_RESULT_SIZE_MASK;
}

void msm_fd_hw_get_result_angle_pose(struct msm_fd_device *fd, int idx,
	u32 *angle, u32 *pose)
{
	u32 reg;
	u32 pose_reg;

	reg = msm_fd_hw_read_reg(fd, MSM_FD_IOMEM_CORE,
		MSM_FD_RESULT_ANGLE_POSE(idx));
	*angle = (reg >> MSM_FD_RESULT_ANGLE_SHIFT) & MSM_FD_RESULT_ANGLE_MASK;
	pose_reg = (reg >> MSM_FD_RESULT_POSE_SHIFT) & MSM_FD_RESULT_POSE_MASK;

	switch (pose_reg) {
	case MSM_FD_RESULT_POSE_FRONT:
		*pose = MSM_FD_POSE_FRONT;
		break;
	case MSM_FD_RESULT_POSE_RIGHT_DIAGONAL:
		*pose = MSM_FD_POSE_RIGHT_DIAGONAL;
		break;
	case MSM_FD_RESULT_POSE_RIGHT:
		*pose = MSM_FD_POSE_RIGHT;
		break;
	case MSM_FD_RESULT_POSE_LEFT_DIAGONAL:
		*pose = MSM_FD_POSE_LEFT_DIAGONAL;
		break;
	case MSM_FD_RESULT_POSE_LEFT:
		*pose = MSM_FD_POSE_LEFT;
		break;
	default:
		dev_err(fd->dev, "Invalid pose from the engine\n");
		*pose = MSM_FD_POSE_FRONT;
		break;
	}
}

void msm_fd_hw_vbif_register(struct msm_fd_device *fd)
{

	msm_fd_hw_reg_set(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_CLKON, 0x1);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_QOS_OVERRIDE_EN, 0x10001);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_QOS_OVERRIDE_REQPRI, 0x1);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_QOS_OVERRIDE_PRILVL, 0x1);

<<<<<<< HEAD
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_IN_RD_LIM_CONF0, 0x10);
=======
		/* Reset sequence after halt */
		msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_MISC, MSM_FD_MISC_SW_RESET,
			MSM_FD_MISC_SW_RESET_SET);
		msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONTROL,
			MSM_FD_CONTROL_SRST);
		msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_MISC,
			MSM_FD_MISC_SW_RESET, 0);
		msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_CORE, MSM_FD_CONTROL, 0);
	}
}
>>>>>>> 0e91d2a... Nougat

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_IN_WR_LIM_CONF0, 0x10);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_OUT_RD_LIM_CONF0, 0x10);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_OUT_WR_LIM_CONF0, 0x10);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_DDR_OUT_MAX_BURST, 0xF0F);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_ARB_CTL, 0x30);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_OUT_AXI_AMEMTYPE_CONF0, 0x02);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_OUT_AXI_AOOO_EN, 0x10001);

	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_ROUND_ROBIN_QOS_ARB, 0x03);
}

void msm_fd_hw_vbif_unregister(struct msm_fd_device *fd)
{
<<<<<<< HEAD
	msm_fd_hw_write_reg(fd, MSM_FD_IOMEM_VBIF,
		MSM_FD_VBIF_CLKON, 0x0);
}

void msm_fd_hw_release_mem_resources(struct msm_fd_device *fd)
{
	int i;

	
	for (i = 0; i < MSM_FD_IOMEM_LAST; i++) {
		if (fd->iomem_base[i]) {
			iounmap(fd->iomem_base[i]);
			fd->iomem_base[i] = NULL;
		}
		if (fd->ioarea[i]) {
			release_mem_region(fd->res_mem[i]->start,
				resource_size(fd->res_mem[i]));
			fd->ioarea[i] = NULL;
		}
		fd->res_mem[i] = NULL;
	}
=======
	int ret;

	fd->irq = msm_camera_get_irq(pdev, "fd");
	if (fd->irq_num < 0) {
		dev_err(fd->dev, "Can not get fd core irq resource\n");
		ret = -ENODEV;
		goto error_irq;
	}

	/* If vbif is shared we will need wrapper irq for releasing vbif */
	if (msm_fd_hw_misc_irq_supported(fd)) {
		ret = msm_camera_register_irq(pdev,
				fd->irq, msm_fd_hw_misc_irq,
				IRQF_TRIGGER_RISING, "fd", fd);
		if (ret) {
			dev_err(fd->dev, "Can not claim wrapper IRQ\n");
			goto error_irq;
		}
	} else {
		ret = msm_camera_register_irq(pdev,
				fd->irq, msm_fd_hw_core_irq,
				IRQF_TRIGGER_RISING, "fd", fd);
		if (ret) {
			dev_err(&pdev->dev, "Can not claim core IRQ\n");
			goto error_irq;
		}

	}

	fd->work_queue = alloc_workqueue(MSM_FD_WORQUEUE_NAME,
		WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!fd->work_queue) {
		dev_err(fd->dev, "Can not register workqueue\n");
		ret = -ENOMEM;
		goto error_alloc_workqueue;
	}
	INIT_WORK(&fd->work, work_func);

	return 0;

error_alloc_workqueue:
	msm_camera_unregister_irq(pdev, fd->irq, fd);
error_irq:
	return ret;
}

/*
 * msm_fd_hw_release_irq - Free core and wrap irq.
 * @fd: Pointer to fd device.
 */
void msm_fd_hw_release_irq(struct msm_fd_device *fd)
{
	if (fd->irq)
		msm_camera_unregister_irq(fd->pdev, fd->irq, fd);

	if (fd->work_queue) {
		destroy_workqueue(fd->work_queue);
		fd->work_queue = NULL;
	}
}

/*
 * msm_fd_hw_set_dt_parms_by_name() - read DT params and write to registers.
 * @fd: Pointer to fd device.
 * @dt_prop_name: Name of the device tree property to read.
 * @base_idx: Fd memory resource index.
 *
 * This function reads register offset and value pairs from dtsi based on
 * device tree property name and writes to FD registers.
 *
 * Return: 0 on success and negative error on failure.
 */
int32_t msm_fd_hw_set_dt_parms_by_name(struct msm_fd_device *fd,
			const char *dt_prop_name,
			enum msm_fd_mem_resources base_idx)
{
	struct device_node *of_node;
	int32_t i = 0 , rc = 0;
	uint32_t *dt_reg_settings = NULL;
	uint32_t dt_count = 0;

	of_node = fd->dev->of_node;
	pr_debug("%s:%d E\n", __func__, __LINE__);

	if (!of_get_property(of_node, dt_prop_name, &dt_count)) {
		pr_err("%s: Error property does not exist\n", __func__);
		return -ENOENT;
	}
	if (dt_count % (sizeof(int32_t) * MSM_FD_REG_LAST_IDX)) {
		pr_err("%s: Error invalid entries\n", __func__);
		return -EINVAL;
	}
	dt_count /= sizeof(int32_t);
	if (dt_count != 0) {
		dt_reg_settings = kcalloc(dt_count,
			sizeof(uint32_t),
			GFP_KERNEL);

		if (!dt_reg_settings)
			return -ENOMEM;

		rc = of_property_read_u32_array(of_node,
				dt_prop_name,
				dt_reg_settings,
				dt_count);
		if (rc < 0) {
			pr_err("%s: No reg info\n", __func__);
			kfree(dt_reg_settings);
			return -EINVAL;
		}

		for (i = 0; i < dt_count; i = i + MSM_FD_REG_LAST_IDX) {
			msm_fd_hw_reg_clr(fd, base_idx,
				dt_reg_settings[i + MSM_FD_REG_ADDR_OFFSET_IDX],
				dt_reg_settings[i + MSM_FD_REG_MASK_IDX]);
			msm_fd_hw_reg_set(fd, base_idx,
				dt_reg_settings[i + MSM_FD_REG_ADDR_OFFSET_IDX],
				dt_reg_settings[i + MSM_FD_REG_VALUE_IDX] &
				dt_reg_settings[i + MSM_FD_REG_MASK_IDX]);
			pr_debug("%s:%d] %pK %08x\n", __func__, __LINE__,
				fd->iomem_base[base_idx] +
				dt_reg_settings[i + MSM_FD_REG_ADDR_OFFSET_IDX],
				dt_reg_settings[i + MSM_FD_REG_VALUE_IDX] &
				dt_reg_settings[i + MSM_FD_REG_MASK_IDX]);
		}
		kfree(dt_reg_settings);
	}
	return 0;
}

/*
 * msm_fd_hw_set_dt_parms() - set FD device tree configuration.
 * @fd: Pointer to fd device.
 *
 * This function holds an array of device tree property names and calls
 * msm_fd_hw_set_dt_parms_by_name() for each property.
 *
 * Return: 0 on success and negative error on failure.
 */
int msm_fd_hw_set_dt_parms(struct msm_fd_device *fd)
{
	int rc = 0;
	uint8_t dt_prop_cnt = MSM_FD_IOMEM_LAST;
	char *dt_prop_name[MSM_FD_IOMEM_LAST] = {"qcom,fd-core-reg-settings",
		"qcom,fd-misc-reg-settings", "qcom,fd-vbif-reg-settings"};

	while (dt_prop_cnt) {
		dt_prop_cnt--;
		rc = msm_fd_hw_set_dt_parms_by_name(fd,
			dt_prop_name[dt_prop_cnt],
			dt_prop_cnt);
		if (rc == -ENOENT) {
			pr_debug("%s: No %s property\n", __func__,
				dt_prop_name[dt_prop_cnt]);
			rc = 0;
		} else if (rc < 0) {
			pr_err("%s: %s params set fail\n", __func__,
				dt_prop_name[dt_prop_cnt]);
			return rc;
		}
	}
	return rc;
}

/*
 * msm_fd_hw_release_mem_resources - Releases memory resources.
 * @fd: Pointer to fd device.
 */
void msm_fd_hw_release_mem_resources(struct msm_fd_device *fd)
{
	msm_camera_put_reg_base(fd->pdev,
		fd->iomem_base[MSM_FD_IOMEM_MISC], "fd_misc", true);
	msm_camera_put_reg_base(fd->pdev,
		fd->iomem_base[MSM_FD_IOMEM_CORE], "fd_core", true);
	msm_camera_put_reg_base(fd->pdev,
		fd->iomem_base[MSM_FD_IOMEM_VBIF], "fd_vbif", false);
>>>>>>> 0e91d2a... Nougat
}

int msm_fd_hw_get_mem_resources(struct platform_device *pdev,
	struct msm_fd_device *fd)
{
	int ret = 0;

<<<<<<< HEAD
	
	for (i = 0; i < MSM_FD_IOMEM_LAST; i++) {
		
		fd->res_mem[i] = platform_get_resource(pdev,
			IORESOURCE_MEM, i);
		if (!fd->res_mem[i]) {
			dev_err(fd->dev, "Fail get resource idx %d\n",
				i);
			ret = -ENODEV;
			break;
		}

		fd->ioarea[i] = request_mem_region(fd->res_mem[i]->start,
			resource_size(fd->res_mem[i]), fd->res_mem[i]->name);
		if (!fd->ioarea[i]) {
			dev_err(fd->dev, "%s can not request mem\n",
				fd->res_mem[i]->name);
			ret = -ENODEV;
			break;
		}

		fd->iomem_base[i] = ioremap(fd->res_mem[i]->start,
			resource_size(fd->res_mem[i]));
		if (!fd->iomem_base[i]) {
			dev_err(fd->dev, "%s can not remap region\n",
				fd->res_mem[i]->name);
			ret = -ENODEV;
			break;
		}
	}

	if (ret < 0)
		msm_fd_hw_release_mem_resources(fd);

	return ret;
}

int msm_fd_hw_get_iommu(struct msm_fd_device *fd)
{
	int ret;

	fd->iommu_domain_num = msm_register_domain(&msm_fd_fw_layout);
	if (fd->iommu_domain_num < 0) {
		dev_err(fd->dev, "Can not register iommu domain\n");
		ret = -ENODEV;
		goto error;
	}

	fd->iommu_domain = msm_get_iommu_domain(fd->iommu_domain_num);
	if (!fd->iommu_domain) {
		dev_err(fd->dev, "Can not get iommu domain\n");
		ret = -ENODEV;
		goto error;
	}

	fd->iommu_dev = msm_iommu_get_ctx("camera_fd");
	if (IS_ERR(fd->iommu_dev)) {
		dev_err(fd->dev, "Can not get iommu device\n");
		ret = -EPROBE_DEFER;
		goto error_iommu_get_dev;
	}

	return 0;

error_iommu_get_dev:
	msm_unregister_domain(fd->iommu_domain);
	fd->iommu_domain = NULL;
	fd->iommu_domain_num = -1;
error:
	return ret;
}

void msm_fd_hw_put_iommu(struct msm_fd_device *fd)
{
	msm_unregister_domain(fd->iommu_domain);
	fd->iommu_domain = NULL;
}

int msm_fd_hw_get_clocks(struct msm_fd_device *fd)
=======
	/* Prepare memory resources */
	fd->iomem_base[MSM_FD_IOMEM_CORE] =
		msm_camera_get_reg_base(pdev, "fd_core", true);
	if (!fd->iomem_base[MSM_FD_IOMEM_CORE]) {
		dev_err(fd->dev, "%s can not map fd_core region\n", __func__);
		ret = -ENODEV;
		goto fd_core_base_failed;
	}

	fd->iomem_base[MSM_FD_IOMEM_MISC] =
		msm_camera_get_reg_base(pdev, "fd_misc", true);
	if (!fd->iomem_base[MSM_FD_IOMEM_MISC]) {
		dev_err(fd->dev, "%s can not map fd_misc region\n", __func__);
		ret = -ENODEV;
		goto fd_misc_base_failed;
	}

	fd->iomem_base[MSM_FD_IOMEM_VBIF] =
		msm_camera_get_reg_base(pdev, "fd_vbif", false);
	if (!fd->iomem_base[MSM_FD_IOMEM_VBIF]) {
		dev_err(fd->dev, "%s can not map fd_vbif region\n", __func__);
		ret = -ENODEV;
		goto fd_vbif_base_failed;
	}

	return ret;
fd_vbif_base_failed:
	msm_camera_put_reg_base(pdev,
		fd->iomem_base[MSM_FD_IOMEM_MISC], "fd_misc", true);
fd_misc_base_failed:
	msm_camera_put_reg_base(pdev,
		fd->iomem_base[MSM_FD_IOMEM_CORE], "fd_core", true);
fd_core_base_failed:
	return ret;
}

/*
 * msm_fd_hw_bus_request - Request bus for memory access.
 * @fd: Pointer to fd device.
 * @idx: Bus bandwidth array index described in device tree.
 */
static int msm_fd_hw_bus_request(struct msm_fd_device *fd, unsigned int idx)
>>>>>>> 0e91d2a... Nougat
{
	int ret;

	ret = msm_camera_update_bus_vector(CAM_BUS_CLIENT_FD, idx);
	if (ret < 0) {
		dev_err(fd->dev, "Fail bus scale update %d\n", ret);
		return -EINVAL;
	}

<<<<<<< HEAD
	clk_rates = 0;
	for (i = 0; i < cnt; i++) {
		ret = of_property_read_string_index(fd->dev->of_node,
			"clock-names", i, &clk_name);
		if (ret < 0) {
			dev_err(fd->dev, "Can not read clock name %d\n", i);
			goto error;
		}

		fd->clk[i] = clk_get(fd->dev, clk_name);
		if (IS_ERR(fd->clk[i])) {
			ret = -ENOENT;
			dev_err(fd->dev, "Error clock get %s\n", clk_name);
			goto error;
		}
		dev_dbg(fd->dev, "Clock name idx %d %s\n", i, clk_name);
	}
	fd->clk_num = cnt;

	cnt = 0;
	for (clk_rates = 0; clk_rates < MSM_FD_MAX_CLK_RATES; clk_rates++) {
		for (i = 0; i < fd->clk_num; i++) {
			ret = of_property_read_u32_index(fd->dev->of_node,
				"clock-rates", cnt++,
				&fd->clk_rates[clk_rates][i]);
			if (ret < 0)
				break;
			dev_dbg(fd->dev, "Clock rate idx %d idx %d value %d\n",
				clk_rates, i, fd->clk_rates[clk_rates][i]);

		}
		if (ret < 0)
			break;
	}
	fd->clk_rates_num = clk_rates;
	if (fd->clk_rates_num == 0) {
		ret = -ENOENT;
		dev_err(fd->dev, "Can not get clock rates\n");
		goto error;
	}

	return 0;
error:
	for (; i > 0; i--)
		clk_put(fd->clk[i - 1]);

	return ret;
}

int msm_fd_hw_put_clocks(struct msm_fd_device *fd)
{
	int i;

	for (i = 0; i < fd->clk_num; i++) {
		if (!IS_ERR_OR_NULL(fd->clk[i]))
			clk_put(fd->clk[i]);
		fd->clk_num = 0;
	}
=======
>>>>>>> 0e91d2a... Nougat
	return 0;
}

static int msm_fd_hw_set_clock_rate_idx(struct msm_fd_device *fd,
		unsigned int idx)
{
	int ret;
	int i;

	if (idx >= fd->clk_rates_num)
		idx = fd->clk_rates_num - 1;

	for (i = 0; i < fd->clk_num; i++) {
		ret = msm_camera_clk_set_rate(&fd->pdev->dev,
			fd->clk[i], fd->clk_rates[idx][i]);
		if (ret < 0) {
			dev_err(fd->dev, "fail set rate on idx[%u][%u]\n",
				idx, i);
			return -EINVAL;
		}
<<<<<<< HEAD
		dev_dbg(fd->dev, "Clk rate %d-%ld idx %d\n", i, clk_rate, idx);
=======
>>>>>>> 0e91d2a... Nougat
	}

	return 0;
}
<<<<<<< HEAD
static int msm_fd_hw_enable_clocks(struct msm_fd_device *fd)
{
	int i;
	int ret;

	for (i = 0; i < fd->clk_num; i++) {
		ret = clk_prepare(fd->clk[i]);
		if (ret < 0) {
			dev_err(fd->dev, "clock prepare failed %d\n", i);
			goto error;
		}

		ret = clk_enable(fd->clk[i]);
		if (ret < 0) {
			dev_err(fd->dev, "clock enable %d\n", i);
			clk_unprepare(fd->clk[i]);
			goto error;
		}
	}

	return 0;
error:
	for (; i > 0; i--) {
		clk_disable(fd->clk[i - 1]);
		clk_unprepare(fd->clk[i - 1]);
	}
	return ret;
}
static void msm_fd_hw_disable_clocks(struct msm_fd_device *fd)
{
	int i;

	for (i = 0; i < fd->clk_num; i++) {
		clk_disable(fd->clk[i]);
		clk_unprepare(fd->clk[i]);
	}
}

static int msm_fd_hw_bus_request(struct msm_fd_device *fd)
{
	int ret;

	fd->bus_client = msm_bus_scale_register_client(&msm_fd_bus_scale_data);
	if (!fd->bus_client) {
		dev_err(fd->dev, "Fail to register bus client\n");
		return -ENOENT;
	}

	ret = msm_bus_scale_client_update_request(fd->bus_client, 0);
=======

/**
 * msm_fd_hw_update_settings() - API to set clock rate and bus settings
 * @fd: Pointer to fd device.
 * @buf: fd buffer
 */
static int msm_fd_hw_update_settings(struct msm_fd_device *fd,
				struct msm_fd_buffer *buf)
{
	int ret = 0;
	uint32_t clk_rate_idx;

	if (!buf)
		return 0;

	clk_rate_idx = buf->settings.speed;
	if (fd->clk_rate_idx == clk_rate_idx)
		return 0;

	if (fd->bus_client) {
		ret = msm_fd_hw_bus_request(fd, clk_rate_idx);
		if (ret < 0) {
			dev_err(fd->dev, "Fail bus scale update %d\n", ret);
			return -EINVAL;
		}
	}

	ret = msm_fd_hw_set_clock_rate_idx(fd, clk_rate_idx);
>>>>>>> 0e91d2a... Nougat
	if (ret < 0) {
		dev_err(fd->dev, "Fail to set clock rate idx\n");
		goto end;
	}
	dev_dbg(fd->dev, "set clk %d %d", fd->clk_rate_idx, clk_rate_idx);
	fd->clk_rate_idx = clk_rate_idx;

<<<<<<< HEAD
	return 0;
}

static void msm_fd_hw_bus_release(struct msm_fd_device *fd)
{
	if (fd->bus_client) {
		msm_bus_scale_unregister_client(fd->bus_client);
		fd->bus_client = 0;
	}
=======
end:
	return ret;
>>>>>>> 0e91d2a... Nougat
}

int msm_fd_hw_get(struct msm_fd_device *fd, unsigned int clock_rate_idx)
{
	int ret;

	mutex_lock(&fd->lock);

	if (fd->ref_count == 0) {
<<<<<<< HEAD
		ret = msm_fd_hw_set_clock_rate_idx(fd, clock_rate_idx);
=======
		ret =
			msm_camera_regulator_enable(fd->vdd_info,
				fd->num_reg, true);
>>>>>>> 0e91d2a... Nougat
		if (ret < 0) {
			dev_err(fd->dev, "Fail to enable vdd\n");
			goto error;
		}

		ret = regulator_enable(fd->vdd);
		if (ret < 0) {
<<<<<<< HEAD
			dev_err(fd->dev, "Fail to enable vdd\n");
			goto error;
=======
			dev_err(fd->dev, "Fail bus request\n");
			goto error_bus_request;
		}
		ret = msm_fd_hw_set_clock_rate_idx(fd, clock_rate_idx);
		if (ret < 0) {
			dev_err(fd->dev, "Fail to set clock rate idx\n");
			goto error_clocks;
>>>>>>> 0e91d2a... Nougat
		}
		ret = msm_camera_clk_enable(&fd->pdev->dev, fd->clk_info,
				fd->clk, fd->clk_num, true);
		if (ret < 0) {
			dev_err(fd->dev, "Fail clk enable request\n");
			goto error_clocks;
		}

<<<<<<< HEAD
		ret = msm_fd_hw_bus_request(fd);
		if (ret < 0) {
			dev_err(fd->dev, "Fail bus request\n");
			goto error_bus_request;
		}
		msm_fd_hw_vbif_register(fd);
=======
		if (msm_fd_hw_misc_irq_supported(fd))
			msm_fd_hw_misc_irq_enable(fd);

		ret = msm_fd_hw_set_dt_parms(fd);
		if (ret < 0)
			goto error_set_dt;

		fd->clk_rate_idx = clock_rate_idx;
>>>>>>> 0e91d2a... Nougat
	}

	fd->ref_count++;
	mutex_unlock(&fd->lock);

	return 0;

<<<<<<< HEAD
error_bus_request:
	msm_fd_hw_disable_clocks(fd);
error_clocks:
	regulator_disable(fd->vdd);
=======
error_set_dt:
	if (msm_fd_hw_misc_irq_supported(fd))
		msm_fd_hw_misc_irq_disable(fd);
	msm_camera_clk_enable(&fd->pdev->dev, fd->clk_info,
		fd->clk, fd->clk_num, false);
error_clocks:
error_bus_request:
	msm_camera_regulator_enable(fd->vdd_info, fd->num_reg, false);
>>>>>>> 0e91d2a... Nougat
error:
	mutex_unlock(&fd->lock);
	return ret;
}

void msm_fd_hw_put(struct msm_fd_device *fd)
{
	mutex_lock(&fd->lock);
	BUG_ON(fd->ref_count == 0);

	if (--fd->ref_count == 0) {
<<<<<<< HEAD
		msm_fd_hw_vbif_unregister(fd);
		msm_fd_hw_bus_release(fd);
		msm_fd_hw_disable_clocks(fd);
		regulator_disable(fd->vdd);
=======
		msm_fd_hw_halt(fd);

		if (msm_fd_hw_misc_irq_supported(fd))
			msm_fd_hw_misc_irq_disable(fd);

		/* vector index 0 is 0 ab and 0 ib */
		msm_fd_hw_bus_request(fd, 0);
		msm_camera_clk_enable(&fd->pdev->dev, fd->clk_info,
				fd->clk, fd->clk_num, false);
		msm_camera_regulator_enable(fd->vdd_info, fd->num_reg, false);
>>>>>>> 0e91d2a... Nougat
	}
	mutex_unlock(&fd->lock);
}

static int msm_fd_hw_attach_iommu(struct msm_fd_device *fd)
{
	int ret = -EINVAL;

	mutex_lock(&fd->lock);

	if (fd->iommu_attached_cnt == UINT_MAX) {
		dev_err(fd->dev, "Max count reached! can not attach iommu\n");
		goto error;
	}

	if (fd->iommu_attached_cnt == 0) {
		ret = iommu_attach_device(fd->iommu_domain, fd->iommu_dev);
		if (ret < 0) {
			dev_err(fd->dev, "Can not attach iommu domain\n");
			goto error;
		}
	}
	fd->iommu_attached_cnt++;
	mutex_unlock(&fd->lock);

	return 0;

error:
	mutex_unlock(&fd->lock);
	return ret;
}

static void msm_fd_hw_detach_iommu(struct msm_fd_device *fd)
{
	mutex_lock(&fd->lock);
	if (fd->iommu_attached_cnt == 0) {
		dev_err(fd->dev, "There is no attached device\n");
		mutex_unlock(&fd->lock);
		return;
	}

	if (--fd->iommu_attached_cnt == 0)
		iommu_detach_device(fd->iommu_domain, fd->iommu_dev);

	mutex_unlock(&fd->lock);
}

int msm_fd_hw_map_buffer(struct msm_fd_mem_pool *pool, int fd,
	struct msm_fd_buf_handle *buf)
{
	int ret;

	if (!pool || fd < 0)
		return -EINVAL;

	ret = msm_fd_hw_attach_iommu(pool->fd_device);
	if (ret < 0)
		goto error;

	buf->pool = pool;
	buf->fd = fd;

	buf->handle = ion_import_dma_buf(pool->client, buf->fd);
	if (IS_ERR_OR_NULL(buf->handle))
		goto error_import_dma;

	ret = ion_map_iommu(pool->client, buf->handle, pool->domain_num,
		0, SZ_4K, 0, &buf->addr, &buf->size, 0, 0);
	if (ret < 0)
		goto error_map_iommu;

	return buf->size;

error_map_iommu:
	ion_free(pool->client, buf->handle);
error_import_dma:
	msm_fd_hw_detach_iommu(pool->fd_device);
error:
	return -ENOMEM;
}

void msm_fd_hw_unmap_buffer(struct msm_fd_buf_handle *buf)
{
	if (buf->size) {
		ion_unmap_iommu(buf->pool->client, buf->handle,
			buf->pool->domain_num, 0);
		msm_fd_hw_detach_iommu(buf->pool->fd_device);
	}

	if (!IS_ERR_OR_NULL(buf->handle))
		ion_free(buf->pool->client, buf->handle);

	buf->fd = -1;
	buf->pool = NULL;
	buf->handle = NULL;
}

static int msm_fd_hw_enable(struct msm_fd_device *fd,
	struct msm_fd_buffer *buffer)
{
	struct msm_fd_buf_handle *buf_handle =
		buffer->vb.planes[0].mem_priv;

	if (msm_fd_hw_is_runnig(fd)) {
		dev_err(fd->dev, "Device is busy we can not enable\n");
		return 0;
	}

	msm_fd_hw_srst(fd);
	msm_fd_hw_set_size_mode(fd, buffer->format.size->reg_val);
	msm_fd_hw_set_crop(fd, &buffer->format.crop);
	msm_fd_hw_set_bytesperline(fd, buffer->format.bytesperline);
	msm_fd_hw_set_image_addr(fd, buf_handle->addr);
	msm_fd_hw_set_work_addr(fd, buffer->work_addr);
	msm_fd_hw_set_min_face(fd, buffer->settings.min_size_index);
	msm_fd_hw_set_threshold(fd, buffer->settings.threshold);
	msm_fd_hw_set_direction_angle(fd, buffer->settings.direction_index,
		buffer->settings.angle_index);
	msm_fd_hw_run(fd);
	if (fd->recovery_mode)
		dev_err(fd->dev, "Scheduled buffer in recovery mode\n");
	return 1;
}

static int msm_fd_hw_try_enable(struct msm_fd_device *fd,
	struct msm_fd_buffer *buffer, enum msm_fd_device_state state)
{
	int enabled = 0;

	if (state == fd->state) {

		fd->state = MSM_FD_DEVICE_RUNNING;
		atomic_set(&buffer->active, 1);

		msm_fd_hw_enable(fd, buffer);
		enabled = 1;
	}
	return enabled;
}

static int msm_fd_hw_remove_active_buffer(struct msm_fd_device *fd)
{
	struct msm_fd_buffer *buffer;
	int active_removed = 0;

	if (!list_empty(&fd->buf_queue)) {
		buffer = list_first_entry(&fd->buf_queue,
			struct msm_fd_buffer, list);
		list_del(&buffer->list);
		active_removed = 1;
	}

	return active_removed;
}

static struct msm_fd_buffer *msm_fd_hw_next_buffer(struct msm_fd_device *fd)
{
	struct msm_fd_buffer *buffer = NULL;

	if (!list_empty(&fd->buf_queue))
		buffer = list_first_entry(&fd->buf_queue,
			struct msm_fd_buffer, list);

	return buffer;
}

void msm_fd_hw_add_buffer(struct msm_fd_device *fd,
	struct msm_fd_buffer *buffer)
{
	spin_lock(&fd->slock);

	atomic_set(&buffer->active, 0);
	init_completion(&buffer->completion);

	INIT_LIST_HEAD(&buffer->list);
	list_add_tail(&buffer->list, &fd->buf_queue);
	spin_unlock(&fd->slock);
}

void msm_fd_hw_remove_buffers_from_queue(struct msm_fd_device *fd,
	struct vb2_queue *vb2_q)
{
	struct msm_fd_buffer *curr_buff;
	struct msm_fd_buffer *temp;
	struct msm_fd_buffer *active_buffer;
	unsigned long time;

	spin_lock(&fd->slock);

	active_buffer = NULL;
	list_for_each_entry_safe(curr_buff, temp, &fd->buf_queue, list) {
		if (curr_buff->vb.vb2_queue == vb2_q) {

			if (atomic_read(&curr_buff->active))
				active_buffer = curr_buff;
			else {
				/* Do a Buffer done on all the other buffers */
				vb2_buffer_done(&curr_buff->vb,
					VB2_BUF_STATE_DONE);
				list_del(&curr_buff->list);
			}
		}
	}
	spin_unlock(&fd->slock);

	
	if (active_buffer) {
		time = wait_for_completion_timeout(&active_buffer->completion,
			msecs_to_jiffies(MSM_FD_PROCESSING_TIMEOUT_MS));
		if (!time) {
<<<<<<< HEAD
			
=======
			/* Do a vb2 buffer done since it timed out */
			vb2_buffer_done(&active_buffer->vb, VB2_BUF_STATE_DONE);
			/* Remove active buffer */
			msm_fd_hw_get_active_buffer(fd);
			/* Schedule if other buffers are present in device */
>>>>>>> 0e91d2a... Nougat
			msm_fd_hw_schedule_next_buffer(fd);
		}
	}

	return;
}

int msm_fd_hw_buffer_done(struct msm_fd_device *fd,
	struct msm_fd_buffer *buffer)
{
	int ret = 0;

	spin_lock(&fd->slock);

	if (atomic_read(&buffer->active)) {
		atomic_set(&buffer->active, 0);
		complete_all(&buffer->completion);
	} else {
		dev_err(fd->dev, "Buffer is not active\n");
		ret = -1;
	}

	spin_unlock(&fd->slock);

	return ret;
}

struct msm_fd_buffer *msm_fd_hw_get_active_buffer(struct msm_fd_device *fd)
{
	struct msm_fd_buffer *buffer = NULL;

	spin_lock(&fd->slock);
	if (!list_empty(&fd->buf_queue)) {
		buffer = list_first_entry(&fd->buf_queue,
			struct msm_fd_buffer, list);
	}
	spin_unlock(&fd->slock);

	return buffer;
}

int msm_fd_hw_schedule_and_start(struct msm_fd_device *fd)
{
	struct msm_fd_buffer *buf;

	spin_lock(&fd->slock);
	buf = msm_fd_hw_next_buffer(fd);
	if (buf)
		msm_fd_hw_try_enable(fd, buf, MSM_FD_DEVICE_IDLE);

	spin_unlock(&fd->slock);

	msm_fd_hw_update_settings(fd, buf);

	return 0;
}

int msm_fd_hw_schedule_next_buffer(struct msm_fd_device *fd)
{
	struct msm_fd_buffer *buf;
	int ret;

	spin_lock(&fd->slock);

	
	if (fd->state != MSM_FD_DEVICE_RUNNING) {
		dev_err(fd->dev, "Can not schedule next buffer\n");
		spin_unlock(&fd->slock);
		return -EBUSY;
	}

	ret = msm_fd_hw_remove_active_buffer(fd);
	if (ret == 0) {
		dev_err(fd->dev, "Active buffer is missing\n");
		spin_unlock(&fd->slock);
		return -EBUSY;
	}

	buf = msm_fd_hw_next_buffer(fd);
	if (buf) {
		ret = msm_fd_hw_try_enable(fd, buf, MSM_FD_DEVICE_RUNNING);
		if (0 == ret) {
			dev_err(fd->dev, "Ouch can not process next buffer\n");
			spin_unlock(&fd->slock);
			return -EBUSY;
		}
	} else {
		fd->state = MSM_FD_DEVICE_IDLE;
		if (fd->recovery_mode)
			dev_err(fd->dev, "No Buffer in recovery mode.Device Idle\n");
	}
	spin_unlock(&fd->slock);

	msm_fd_hw_update_settings(fd, buf);

	return 0;
}
