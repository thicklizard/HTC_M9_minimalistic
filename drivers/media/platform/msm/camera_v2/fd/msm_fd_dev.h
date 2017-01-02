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

#ifndef __MSM_FD_DEV_H__
#define __MSM_FD_DEV_H__

#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ctrls.h>
#include <media/msm_fd.h>
<<<<<<< HEAD

=======
#include <linux/dma-buf.h>
#include <linux/msm_ion.h>
#include "cam_soc_api.h"
#include "cam_hw_ops.h"
#include "msm_cpp.h"

/* Maximum number of result buffers */
>>>>>>> 0e91d2a... Nougat
#define MSM_FD_MAX_RESULT_BUFS 5
#define MSM_FD_MAX_CLK_NUM 10
#define MSM_FD_MAX_CLK_RATES 5
#define MSM_FD_MAX_FACES_DETECTED 32

struct msm_fd_size {
	int width;
	int height;
	u32 reg_val;
	int work_size;
};

struct msm_fd_setings {
	unsigned int min_size_index;
	unsigned int angle_index;
	unsigned int direction_index;
	unsigned int threshold;
	unsigned int speed;
};

struct msm_fd_format {
	struct msm_fd_size *size;
	struct v4l2_rect crop;
	int bytesperline;
	int sizeimage;
	u32 pixelformat;
};

struct msm_fd_mem_pool {
	struct msm_fd_device *fd_device;
	struct ion_client *client;
	int domain_num;
};

struct msm_fd_buf_handle {
	int fd;
	struct msm_fd_mem_pool *pool;
	void *handle;
	unsigned long size;
	ion_phys_addr_t addr;
};

struct msm_fd_buffer {
	struct vb2_buffer vb;
	atomic_t active;
	struct completion completion;
	struct msm_fd_format format;
	struct msm_fd_setings settings;
	ion_phys_addr_t work_addr;
	struct list_head list;
};

struct msm_fd_stats {
	atomic_t frame_id;
	u32 face_cnt;
	struct msm_fd_face_data face_data[MSM_FD_MAX_FACES_DETECTED];
};

struct fd_ctx {
	struct msm_fd_device *fd_device;
	struct v4l2_fh fh;
	struct vb2_queue vb2_q;
	unsigned int sequence;
	atomic_t subscribed_for_event;
	struct msm_fd_format format;
	struct msm_fd_setings settings;
	struct msm_fd_mem_pool mem_pool;
	struct msm_fd_stats *stats;
	struct msm_fd_buf_handle work_buf;
	struct completion *wait_stop_stream;
};

enum msm_fd_device_state {
	MSM_FD_DEVICE_IDLE,
	MSM_FD_DEVICE_RUNNING,
};

enum msm_fd_mem_resources {
	MSM_FD_IOMEM_CORE,
	MSM_FD_IOMEM_MISC,
	MSM_FD_IOMEM_VBIF,
	MSM_FD_IOMEM_LAST
};

<<<<<<< HEAD
=======
/*
 * struct msm_fd_device - FD device structure.
 * @hw_revision: Face detection hw revision.
 * @lock: Lock used for reference count.
 * @slock: Spinlock used to protect FD device struct.
 * @irq_num: Face detection irq number.
 * @ref_count: Device reference count.
 * @res_mem: Array of memory resources used by FD device.
 * @iomem_base: Array of register mappings used by FD device.
 * @vdd: Pointer to vdd regulator.
 * @clk_num: Number of clocks attached to the device.
 * @clk: Array of clock resources used by fd device.
 * @clk_rates: Array of clock rates set.
 * @bus_vectors: Pointer to bus vectors array.
 * @bus_paths: Pointer to bus paths array.
 * @bus_scale_data: Memory access bus scale data.
 * @bus_client: Memory access bus client.
 * @iommu_attached_cnt: Iommu attached devices reference count.
 * @iommu_hdl: reference for iommu context.
 * @dev: Pointer to device struct.
 * @v4l2_dev: V4l2 device.
 * @video: Video device.
 * @state: FD device state.
 * @buf_queue: FD device processing queue.
 * @work_queue: Pointer to FD device IRQ bottom half workqueue.
 * @work: IRQ bottom half work struct.
 * @hw_halt_completion: Completes when face detection hw halt completes.
 * @recovery_mode: Indicates if FD is in recovery mode
 */
>>>>>>> 0e91d2a... Nougat
struct msm_fd_device {
	struct mutex lock;
	spinlock_t slock;
	struct mutex recovery_lock;
	int ref_count;

	int irq_num;
	void __iomem *iomem_base[MSM_FD_IOMEM_LAST];
<<<<<<< HEAD
	struct resource *ioarea[MSM_FD_IOMEM_LAST];
	struct regulator *vdd;

	unsigned int clk_num;
	struct clk *clk[MSM_FD_MAX_CLK_NUM];
	unsigned int clk_rates_num;
	unsigned int clk_rates[MSM_FD_MAX_CLK_RATES][MSM_FD_MAX_CLK_NUM];

=======
	struct msm_cam_clk_info *clk_info;
	struct msm_cam_regulator *vdd_info;
	int num_reg;
	struct resource *irq;

	size_t clk_num;
	size_t clk_rates_num;
	struct clk **clk;
	uint32_t **clk_rates;
>>>>>>> 0e91d2a... Nougat
	uint32_t bus_client;

	struct iommu_domain *iommu_domain;
	int iommu_domain_num;
	unsigned int iommu_attached_cnt;

	struct device *iommu_dev;
	struct device *dev;
	struct platform_device *pdev;
	struct v4l2_device v4l2_dev;
	struct video_device video;

	enum msm_fd_device_state state;
	struct list_head buf_queue;
	struct workqueue_struct *work_queue;
	struct work_struct work;
<<<<<<< HEAD
=======
	struct completion hw_halt_completion;
	int recovery_mode;
	uint32_t clk_rate_idx;
>>>>>>> 0e91d2a... Nougat
};

#endif 
