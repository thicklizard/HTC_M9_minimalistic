<<<<<<< HEAD
/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2010-2013, 2015-2016, The Linux Foundation. All rights reserved.
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

#ifndef __MDSS_HDMI_CEC_H__
#define __MDSS_HDMI_CEC_H__

#include "mdss_hdmi_util.h"

struct hdmi_cec_init_data {
	struct workqueue_struct *workq;
	struct kobject *sysfs_kobj;
	struct dss_io_data *io;
};

int hdmi_cec_deconfig(void *cec_ctrl);
int hdmi_cec_config(void *cec_ctrl);
int hdmi_cec_isr(void *cec_ctrl);
void hdmi_cec_deinit(void *cec_ctrl);
void *hdmi_cec_init(struct hdmi_cec_init_data *init_data);
<<<<<<< HEAD
=======

/**
 * hdmi_cec_deinit() - de-initialize CEC HW module
 * @data: CEC HW module data
 *
 * This API release all resources allocated.
 */
void hdmi_cec_deinit(void *data);

/**
 * hdmi_cec_is_wakeup_en() - checks cec wakeup state
 * @cec_ctrl: pointer to cec hw module's data
 *
 * Return: cec wakeup state
 *
 * This API is used to query whether the cec wakeup functionality is
 * enabled or not.
 */
bool hdmi_cec_is_wakeup_en(void *cec_ctrl);

/**
 * hdmi_cec_device_suspend() - updates cec with device suspend state
 * @cec_ctrl: pointer to cec hw module's data
 * @suspend: device suspend state
 *
 * This API is used to update the CEC HW module of the device's suspend
 * state.
 */
void hdmi_cec_device_suspend(void *cec_ctrl, bool suspend);
>>>>>>> 0e91d2a... Nougat
#endif /* __MDSS_HDMI_CEC_H__ */
