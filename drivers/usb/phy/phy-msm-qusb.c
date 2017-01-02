/*
 * Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk/msm-clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/phy.h>
#include <linux/usb/msm_hsusb.h>

#define QUSB2PHY_PORT_POWERDOWN		0xB4
#define CLAMP_N_EN			BIT(5)
#define FREEZIO_N			BIT(1)
#define POWER_DOWN			BIT(0)
#define QUSB2PHY_PORT_UTMI_CTRL2	0xC4
#define QUSB2PHY_PORT_TUNE1             0x80
#define QUSB2PHY_PORT_TUNE2             0x84
#define QUSB2PHY_PORT_TUNE3             0x88
#define QUSB2PHY_PORT_TUNE4             0x8C

#define QUSB2PHY_PORT_INTR_CTRL         0xBC
#define CHG_DET_INTR_EN                 BIT(4)
#define DMSE_INTR_HIGH_SEL              BIT(3)
#define DMSE_INTR_EN                    BIT(2)
#define DPSE_INTR_HIGH_SEL              BIT(1)
#define DPSE_INTR_EN                    BIT(0)

#define QUSB2PHY_PORT_UTMI_STATUS	0xF4
#define LINESTATE_DP			BIT(0)
#define LINESTATE_DM			BIT(1)

#define UTMI_OTG_VBUS_VALID             BIT(20)
#define SW_SESSVLD_SEL                  BIT(28)

#define QRBTC_USB2_PLL		0x404
#define QRBTC_USB2_PLLCTRL2	0x414
#define QRBTC_USB2_PLLCTRL1	0x410
#define QRBTC_USB2_PLLCTRL3	0x418
#define QRBTC_USB2_PLLTEST1	0x408
#define RUMI_RESET_ADDRESS	0x6500
#define RUMI_RESET_VALUE_1	0x80000000
#define RUMI_RESET_VALUE_2	0x000201e0

#define PORT_OFFSET(i) ((i == 0) ? 0x0 : ((i == 1) ? 0x6c : 0x88))
#define HS_PHY_CTRL_REG(i)              (0x10 + PORT_OFFSET(i))

#define QUSB2PHY_1P8_VOL_MIN           1800000 /* uV */
#define QUSB2PHY_1P8_VOL_MAX           1800000 /* uV */
#define QUSB2PHY_1P8_HPM_LOAD          30000   /* uA */

#define QUSB2PHY_3P3_VOL_MIN		3075000 /* uV */
#define QUSB2PHY_3P3_VOL_MAX		3200000 /* uV */
#define QUSB2PHY_3P3_HPM_LOAD		30000	/* uA */

struct qusb_phy {
	struct usb_phy		phy;
	void __iomem		*base;
	void __iomem		*qscratch_base;

	struct clk		*ref_clk;
	struct clk		*cfg_ahb_clk;
	struct clk		*phy_reset;
	struct clk		*iface_clk;
	struct clk		*core_clk;

	struct regulator	*gdsc;
	struct regulator	*vdd;
	struct regulator	*vdda33;
	struct regulator	*vdda18;
<<<<<<< HEAD
	int			vdd_levels[3]; /* none, low, high */
	u32			qusb_tune;
=======
	int			vdd_levels[3]; 
	int			init_seq_len;
	int			*qusb_phy_init_seq;

	u32			tune2_val;
	int			tune2_efuse_bit_pos;
	int			tune2_efuse_num_of_bits;
	int			tune2_efuse_correction;
>>>>>>> 0e91d2a... Nougat

	bool			vdd_enabled;
	bool			power_enabled;
	bool			clocks_enabled;
	bool			cable_connected;
	bool			suspended;
	bool			emulation;
<<<<<<< HEAD
	bool			ulpi_mode;
=======
	int			*emu_init_seq;
	int			emu_init_seq_len;
	int			*phy_pll_reset_seq;
	int			phy_pll_reset_seq_len;
	int			*emu_dcm_reset_seq;
	int			emu_dcm_reset_seq_len;
	spinlock_t		pulse_lock;
	int			usb_phy_always_on;
	bool			put_into_high_z_state;
>>>>>>> 0e91d2a... Nougat
};

static int qusb_phy_reset(struct usb_phy *phy)
{
	struct qusb_phy *qphy = container_of(phy, struct qusb_phy, phy);

<<<<<<< HEAD
	dev_dbg(phy->dev, "%s\n", __func__);

	clk_reset(qphy->phy_reset, CLK_RESET_ASSERT);
	usleep(100);
	clk_reset(qphy->phy_reset, CLK_RESET_DEASSERT);
=======
	if (!qphy->clocks_enabled && on) {
		clk_prepare_enable(qphy->ref_clk_src);
		clk_prepare_enable(qphy->ref_clk);
		clk_prepare_enable(qphy->iface_clk);
		clk_prepare_enable(qphy->core_clk);
		clk_prepare_enable(qphy->cfg_ahb_clk);
		qphy->clocks_enabled = true;
	}

	if (qphy->clocks_enabled && !on) {
		clk_disable_unprepare(qphy->cfg_ahb_clk);
		clk_disable_unprepare(qphy->core_clk);
		clk_disable_unprepare(qphy->iface_clk);
		clk_disable_unprepare(qphy->ref_clk);
		clk_disable_unprepare(qphy->ref_clk_src);
		qphy->clocks_enabled = false;
	}
>>>>>>> 0e91d2a... Nougat

	return 0;
}

static int qusb_phy_gdsc(struct qusb_phy *qphy, bool on)
{
	int ret;

	if (IS_ERR_OR_NULL(qphy->gdsc))
		return -EPERM;

	if (on) {
		dev_dbg(qphy->phy.dev, "TURNING ON GDSC\n");
		ret = regulator_enable(qphy->gdsc);
		if (ret) {
			dev_err(qphy->phy.dev, "unable to enable gdsc\n");
			return ret;
		}
	} else {
		dev_dbg(qphy->phy.dev, "TURNING OFF GDSC\n");
		ret = regulator_disable(qphy->gdsc);
		if (ret) {
			dev_err(qphy->phy.dev, "unable to disable gdsc\n");
			return ret;
		}
	}

	return ret;
}

static int qusb_phy_config_vdd(struct qusb_phy *qphy, int high)
{
	int min, ret;

	min = high ? 1 : 0; /* low or none? */
	ret = regulator_set_voltage(qphy->vdd, qphy->vdd_levels[min],
						qphy->vdd_levels[2]);
	if (ret) {
		dev_err(qphy->phy.dev, "unable to set voltage for qusb vdd\n");
		return ret;
	}

	dev_dbg(qphy->phy.dev, "min_vol:%d max_vol:%d\n",
			qphy->vdd_levels[min], qphy->vdd_levels[2]);
	return ret;
}

<<<<<<< HEAD
static int qusb_phy_enable_power(struct qusb_phy *qphy, bool on)
{
	int ret = 0;

	dev_dbg(qphy->phy.dev, "%s turn %s regulators. power_enabled:%d\n",
			__func__, on ? "on" : "off", qphy->power_enabled);

	if (qphy->power_enabled == on) {
		dev_dbg(qphy->phy.dev, "PHYs' regulators are already ON.\n");
		return 0;
	}

	if (!on)
		goto disable_vdda33;

	ret = qusb_phy_config_vdd(qphy, true);
	if (ret) {
		dev_err(qphy->phy.dev, "Unable to config VDD:%d\n", ret);
		goto err_vdd;
	}

	ret = regulator_enable(qphy->vdd);
	if (ret) {
		dev_err(qphy->phy.dev, "Unable to enable VDD\n");
		goto unconfig_vdd;
=======
static int qusb_phy_vdd(struct qusb_phy *qphy, bool on)
{
	int ret = 0;

	if (!qphy->vdd_enabled && on) {
		dev_dbg(qphy->phy.dev, "TURNING ON VDD\n");
		ret = qusb_phy_config_vdd(qphy, true);
		if (ret) {
			dev_err(qphy->phy.dev, "Unable to config VDD:%d\n",
								ret);
			goto err;
		}

		ret = regulator_enable(qphy->vdd);
		if (ret) {
			dev_err(qphy->phy.dev, "Unable to enable VDD\n");
			goto err;
		}
		qphy->vdd_enabled = true;
	}

	if (qphy->vdd_enabled && !on) {
		dev_dbg(qphy->phy.dev, "TURNING OFF VDD\n");
		ret = regulator_disable(qphy->vdd);
		if (ret) {
			dev_err(qphy->phy.dev, "Unable to disable vdd:%d\n",
									ret);
			goto err;
		}

		ret = qusb_phy_config_vdd(qphy, false);
		if (ret) {
			dev_err(qphy->phy.dev, "Unable unconfig VDD:%d\n", ret);
			goto err;
		}
		qphy->vdd_enabled = false;
>>>>>>> 0e91d2a... Nougat
	}
err:
	return ret;
}

static int qusb_phy_enable_power(struct qusb_phy *qphy, bool on)
{
	int ret = 0;
	static bool L24_keep = false; 

	dev_dbg(qphy->phy.dev, "%s turn %s regulators. power_enabled:%d\n",
			__func__, on ? "on" : "off", qphy->power_enabled);

	if (qphy->power_enabled == on) {
		dev_dbg(qphy->phy.dev, "PHYs' regulators are already ON.\n");
		return 0;
	}

	if (!on)
		goto disable_vdda33;

	ret = qusb_phy_vdd(qphy, true);
	if (ret < 0)
		goto err_vdd;

	ret = regulator_set_optimum_mode(qphy->vdda18, QUSB2PHY_1P8_HPM_LOAD);
	if (ret < 0) {
		dev_err(qphy->phy.dev, "Unable to set HPM of vdda18:%d\n", ret);
		goto disable_vdd;
	}

	ret = regulator_set_voltage(qphy->vdda18, QUSB2PHY_1P8_VOL_MIN,
						QUSB2PHY_1P8_VOL_MAX);
	if (ret) {
		dev_err(qphy->phy.dev,
				"Unable to set voltage for vdda18:%d\n", ret);
		goto put_vdda18_lpm;
	}

<<<<<<< HEAD
	ret = regulator_enable(qphy->vdda18);
	if (ret) {
		dev_err(qphy->phy.dev, "Unable to enable vdda18:%d\n", ret);
		goto unset_vdda18;
=======
	if (qphy->usb_phy_always_on) {
		if (!L24_keep) {
			ret = regulator_enable(qphy->vdda18);
			if (ret) {
				dev_err(qphy->phy.dev, "Unable to enable vdda18:%d\n", ret);
				goto unset_vdda18;
			}
		}
	} else { 
		ret = regulator_enable(qphy->vdda18);
		if (ret) {
			dev_err(qphy->phy.dev, "Unable to enable vdda18:%d\n", ret);
			goto unset_vdda18;
		}
>>>>>>> 0e91d2a... Nougat
	}

	ret = regulator_set_optimum_mode(qphy->vdda33, QUSB2PHY_3P3_HPM_LOAD);
	if (ret < 0) {
		dev_err(qphy->phy.dev, "Unable to set HPM of vdda33:%d\n", ret);
		goto disable_vdda18;
	}

	ret = regulator_set_voltage(qphy->vdda33, QUSB2PHY_3P3_VOL_MIN,
						QUSB2PHY_3P3_VOL_MAX);
	if (ret) {
		dev_err(qphy->phy.dev,
				"Unable to set voltage for vdda33:%d\n", ret);
		goto put_vdda33_lpm;
	}
<<<<<<< HEAD

	ret = regulator_enable(qphy->vdda33);
	if (ret) {
		dev_err(qphy->phy.dev, "Unable to enable vdda33:%d\n", ret);
		goto unset_vdd33;
	}
=======
	if (qphy->usb_phy_always_on) {
		if (!L24_keep) {
			ret = regulator_enable(qphy->vdda33);
			if (ret) {
				dev_err(qphy->phy.dev, "Unable to enable vdda33:%d\n", ret);
				goto unset_vdd33;
			}
			L24_keep = true;
		}
	} else { 
		ret = regulator_enable(qphy->vdda33);
		if (ret) {
			dev_err(qphy->phy.dev, "Unable to enable vdda33:%d\n", ret);
			goto unset_vdd33;
		}
		L24_keep = true;
	}

	qphy->power_enabled = true;
>>>>>>> 0e91d2a... Nougat

	qphy->power_enabled = true;
	pr_debug("%s(): QUSB PHY's regulators are turned ON.\n", __func__);
	return 0;

disable_vdda33:
<<<<<<< HEAD
	ret = regulator_disable(qphy->vdda33);
	if (ret)
		dev_err(qphy->phy.dev, "Unable to disable vdda33:%d\n", ret);

unset_vdd33:
	ret = regulator_set_voltage(qphy->vdda33, 0, QUSB2PHY_3P3_VOL_MAX);
	if (ret)
		dev_err(qphy->phy.dev,
			"Unable to set (0) voltage for vdda33:%d\n", ret);
=======
	if (!qphy->usb_phy_always_on) { 
		ret = regulator_disable(qphy->vdda33);
		if (ret)
			dev_err(qphy->phy.dev, "Unable to disable vdda33:%d\n", ret);
	}

unset_vdd33:
	if (!qphy->usb_phy_always_on) { 
		ret = regulator_set_voltage(qphy->vdda33, 0, QUSB2PHY_3P3_VOL_MAX);
		if (ret)
			dev_err(qphy->phy.dev,
				"Unable to set (0) voltage for vdda33:%d\n", ret);
	}
>>>>>>> 0e91d2a... Nougat

put_vdda33_lpm:
	ret = regulator_set_optimum_mode(qphy->vdda33, 0);
	if (ret < 0)
		dev_err(qphy->phy.dev, "Unable to set (0) HPM of vdda33\n");

disable_vdda18:
<<<<<<< HEAD
	ret = regulator_disable(qphy->vdda18);
	if (ret)
		dev_err(qphy->phy.dev, "Unable to disable vdda18:%d\n", ret);

unset_vdda18:
	ret = regulator_set_voltage(qphy->vdda18, 0, QUSB2PHY_1P8_VOL_MAX);
	if (ret)
		dev_err(qphy->phy.dev,
			"Unable to set (0) voltage for vdda18:%d\n", ret);

=======
	if (!qphy->usb_phy_always_on) { 
		ret = regulator_disable(qphy->vdda18);
		if (ret)
			dev_err(qphy->phy.dev, "Unable to disable vdda18:%d\n", ret);
	}
unset_vdda18:
	if (!qphy->usb_phy_always_on) { 
		ret = regulator_set_voltage(qphy->vdda18, 0, QUSB2PHY_1P8_VOL_MAX);
		if (ret)
			dev_err(qphy->phy.dev,
				"Unable to set (0) voltage for vdda18:%d\n", ret);
	}
>>>>>>> 0e91d2a... Nougat
put_vdda18_lpm:
	ret = regulator_set_optimum_mode(qphy->vdda18, 0);
	if (ret < 0)
		dev_err(qphy->phy.dev, "Unable to set LPM of vdda18\n");

disable_vdd:
<<<<<<< HEAD
	ret = regulator_disable(qphy->vdd);
	if (ret)
		dev_err(qphy->phy.dev, "Unable to disable vdd:%d\n", ret);

unconfig_vdd:
	ret = qusb_phy_config_vdd(qphy, false);
	if (ret)
		dev_err(qphy->phy.dev, "Unable unconfig VDD:%d\n", ret);

=======
	ret = qusb_phy_vdd(qphy, false);
>>>>>>> 0e91d2a... Nougat
err_vdd:
	qphy->power_enabled = false;
	dev_dbg(qphy->phy.dev, "QUSB PHY's regulators are turned OFF.\n");
	return ret;
}

<<<<<<< HEAD
=======
#define PHY_PULSE_TIME_USEC		250
static int qusb_phy_update_dpdm(struct usb_phy *phy, int value)
{
	struct qusb_phy *qphy = container_of(phy, struct qusb_phy, phy);
	unsigned long flags;
	int ret = 0;
	u32 reg;

	dev_dbg(phy->dev, "%s value:%d rm_pulldown:%d pulsing enabled %d\n",
			__func__, value, qphy->rm_pulldown,
			qphy->dpdm_pulsing_enabled);

	switch (value) {
	case POWER_SUPPLY_DP_DM_DPF_DMF:
		dev_dbg(phy->dev, "POWER_SUPPLY_DP_DM_DPF_DMF\n");
		if (!qphy->rm_pulldown) {

			if (qphy->put_into_high_z_state) {

				
				ret = qusb_phy_vdd(qphy, true);
				if (ret < 0)
					goto clk_error;
				qusb_phy_gdsc(qphy, true);
				qusb_phy_enable_clocks(qphy, true);

				dev_dbg(phy->dev, "RESET QUSB PHY\n");
				clk_reset(qphy->phy_reset, CLK_RESET_ASSERT);
				usleep_range(100, 150);
				clk_reset(qphy->phy_reset, CLK_RESET_DEASSERT);

				writel_relaxed(TERM_SELECT | XCVR_SELECT_FS |
					OP_MODE_NON_DRIVE,
					qphy->base + QUSB2PHY_PORT_UTMI_CTRL1);
				writel_relaxed(UTMI_ULPI_SEL |
					UTMI_TEST_MUX_SEL,
					qphy->base + QUSB2PHY_PORT_UTMI_CTRL2);

				
				writel_relaxed(CLAMP_N_EN | FREEZIO_N |
					POWER_DOWN,
					qphy->base + QUSB2PHY_PORT_POWERDOWN);
				
				wmb();
			}

			ret = qusb_phy_enable_power(qphy, true);
			if (ret >= 0) {
				qphy->rm_pulldown = true;
				dev_dbg(phy->dev, "DP_DM_F: rm_pulldown:%d\n",
						qphy->rm_pulldown);
			}

			if (qphy->put_into_high_z_state) {
				qusb_phy_enable_clocks(qphy, false);
				qusb_phy_gdsc(qphy, false);
			}
		}

		
		if (qphy->dpdm_pulsing_enabled && qphy->rm_pulldown) {
			dev_dbg(phy->dev, "clearing qc1 and qc2 registers.\n");
			ret = clk_prepare_enable(qphy->cfg_ahb_clk);
			if (ret)
				goto clk_error;

			
			writel_relaxed(0x00, qphy->base + QUSB2PHY_PORT_QC1);
			writel_relaxed(0x00, qphy->base + QUSB2PHY_PORT_QC2);
			
			mb();
			clk_disable_unprepare(qphy->cfg_ahb_clk);
		}
		break;

	case POWER_SUPPLY_DP_DM_DPR_DMR:
		dev_dbg(phy->dev, "POWER_SUPPLY_DP_DM_DPR_DMR\n");
		if (qphy->rm_pulldown) {
			dev_dbg(phy->dev, "clearing qc1 and qc2 registers.\n");
			if (qphy->dpdm_pulsing_enabled) {
				ret = clk_prepare_enable(qphy->cfg_ahb_clk);
				if (ret)
					goto clk_error;

				
				writel_relaxed(0x00,
						qphy->base + QUSB2PHY_PORT_QC1);
				writel_relaxed(0x00,
						qphy->base + QUSB2PHY_PORT_QC2);
				
				mb();
				clk_disable_unprepare(qphy->cfg_ahb_clk);
			}

			if (!qphy->cable_connected) {
				dev_dbg(phy->dev, "turn off for HVDCP case\n");
				ret = qusb_phy_enable_power(qphy, false);
			}
			if (ret >= 0) {
				qphy->rm_pulldown = false;
				dev_dbg(phy->dev, "DP_DM_R: rm_pulldown:%d\n",
						qphy->rm_pulldown);
			}
		}
		break;

	case POWER_SUPPLY_DP_DM_DP0P6_DMF:
		if (!qphy->dpdm_pulsing_enabled)
			break;

		dev_dbg(phy->dev, "POWER_SUPPLY_DP_DM_DP0P6_DMF\n");
		ret = clk_prepare_enable(qphy->cfg_ahb_clk);
		if (ret)
			goto clk_error;

		
		writel_relaxed(VDP_SRC_EN, qphy->base + QUSB2PHY_PORT_QC1);
		
		mb();
		clk_disable_unprepare(qphy->cfg_ahb_clk);
		break;

	case POWER_SUPPLY_DP_DM_DP0P6_DM3P3:
		if (!qphy->dpdm_pulsing_enabled)
			break;

		dev_dbg(phy->dev, "POWER_SUPPLY_DP_DM_DP0PHVDCP_36_DM3P3\n");
		ret = clk_prepare_enable(qphy->cfg_ahb_clk);
		if (ret)
			goto clk_error;

		
		writel_relaxed(VDP_SRC_EN, qphy->base + QUSB2PHY_PORT_QC1);
		
		writel_relaxed(RPUM_LOW_EN | RDM_UP_EN,
				qphy->base + QUSB2PHY_PORT_QC2);
		
		mb();
		clk_disable_unprepare(qphy->cfg_ahb_clk);
		break;

	case POWER_SUPPLY_DP_DM_DP_PULSE:
		if (!qphy->dpdm_pulsing_enabled)
			break;

		dev_dbg(phy->dev, "POWER_SUPPLY_DP_DM_DP_PULSE\n");
		ret = clk_prepare_enable(qphy->cfg_ahb_clk);
		if (ret)
			goto clk_error;

		spin_lock_irqsave(&qphy->pulse_lock, flags);
		
		reg = readl_relaxed(qphy->base + QUSB2PHY_PORT_QC2);
		reg |= (RDP_UP_EN | RPUP_LOW_EN);
		writel_relaxed(reg, qphy->base + QUSB2PHY_PORT_QC2);

		
		mb();

		udelay(PHY_PULSE_TIME_USEC);

		 
		reg = readl_relaxed(qphy->base + QUSB2PHY_PORT_QC1);
		reg |= VDP_SRC_EN;
		writel_relaxed(reg, qphy->base + QUSB2PHY_PORT_QC1);

		reg = readl_relaxed(qphy->base + QUSB2PHY_PORT_QC2);
		reg &= ~(RDP_UP_EN | RPUP_LOW_EN);
		writel_relaxed(reg, qphy->base + QUSB2PHY_PORT_QC2);
		
		mb();
		spin_unlock_irqrestore(&qphy->pulse_lock, flags);
		usleep_range(2000, 3000);
		clk_disable_unprepare(qphy->cfg_ahb_clk);
		break;

	case POWER_SUPPLY_DP_DM_DM_PULSE:
		if (!qphy->dpdm_pulsing_enabled)
			break;

		dev_dbg(phy->dev, "POWER_SUPPLY_DP_DM_DM_PULSE\n");
		ret = clk_prepare_enable(qphy->cfg_ahb_clk);
		if (ret)
			goto clk_error;

		spin_lock_irqsave(&qphy->pulse_lock, flags);
		
		reg = readl_relaxed(qphy->base + QUSB2PHY_PORT_QC1);
		reg |= VDM_SRC_EN;
		writel_relaxed(reg, qphy->base + QUSB2PHY_PORT_QC1);

		reg = readl_relaxed(qphy->base + QUSB2PHY_PORT_QC2);
		reg &= ~(RDM_UP_EN | RPUM_LOW_EN);
		writel_relaxed(reg, qphy->base + QUSB2PHY_PORT_QC2);

		
		mb();

		udelay(PHY_PULSE_TIME_USEC);

		
		reg = readl_relaxed(qphy->base + QUSB2PHY_PORT_QC2);
		reg |= (RPUM_LOW_EN | RDM_UP_EN);
		writel_relaxed(reg, qphy->base + QUSB2PHY_PORT_QC2);

		reg = readl_relaxed(qphy->base + QUSB2PHY_PORT_QC1);
		reg &= ~VDM_SRC_EN;
		writel_relaxed(reg, qphy->base + QUSB2PHY_PORT_QC1);

		
		mb();
		spin_unlock_irqrestore(&qphy->pulse_lock, flags);

		usleep_range(2000, 3000);
		clk_disable_unprepare(qphy->cfg_ahb_clk);
		break;
	default:
		ret = -EINVAL;
		dev_err(phy->dev, "Invalid power supply property(%d)\n", value);
		break;
	}

clk_error:
	return ret;
}

static void __maybe_unused qusb_phy_get_tune2_param(struct qusb_phy *qphy)
{
	u8 num_of_bits;
	u32 bit_mask = 1;

	pr_debug("%s(): num_of_bit s:%d bit_pos:%d\n", __func__,
				qphy->tune2_efuse_num_of_bits,
				qphy->tune2_efuse_bit_pos);

	
	if (qphy->tune2_efuse_num_of_bits) {
		num_of_bits = qphy->tune2_efuse_num_of_bits;
		bit_mask = (bit_mask << num_of_bits) - 1;
	}

	qphy->tune2_val = readl_relaxed(qphy->tune2_efuse_reg);
	pr_debug("%s(): bit_mask:%d efuse based tune2 value:%d\n",
				__func__, bit_mask, qphy->tune2_val);

	qphy->tune2_val = TUNE2_HIGH_NIBBLE_VAL(qphy->tune2_val,
				qphy->tune2_efuse_bit_pos, bit_mask);

	
	if (qphy->tune2_efuse_correction) {
		if (qphy->tune2_efuse_correction > 5 ||
				qphy->tune2_efuse_correction < -10)
			pr_warn("Correction value is out of range : %d\n",
					qphy->tune2_efuse_correction);
		else
			qphy->tune2_val = qphy->tune2_val +
						qphy->tune2_efuse_correction;
	}

	if (!qphy->tune2_val)
		qphy->tune2_val = TUNE2_DEFAULT_HIGH_NIBBLE;

	
	qphy->tune2_val = ((qphy->tune2_val << 0x4) |
					TUNE2_DEFAULT_LOW_NIBBLE);
}

static void qusb_phy_write_seq(void __iomem *base, u32 *seq, int cnt,
		unsigned long delay)
{
	int i;

	pr_debug("Seq count:%d\n", cnt);
	for (i = 0; i < cnt; i = i+2) {
		pr_debug("write 0x%02x to 0x%02x\n", seq[i], seq[i+1]);
		writel_relaxed(seq[i], base + seq[i+1]);
		if (delay)
			usleep_range(delay, (delay + 2000));
	}
}

>>>>>>> 0e91d2a... Nougat
static int qusb_phy_init(struct usb_phy *phy)
{
	struct qusb_phy *qphy = container_of(phy, struct qusb_phy, phy);
	int ret;
	u32 t1, t2, t3, t4;

	dev_dbg(phy->dev, "%s\n", __func__);

	ret = qusb_phy_enable_power(qphy, true);
	if (ret)
		return ret;

	if (!qphy->clocks_enabled) {
		clk_prepare_enable(qphy->ref_clk);
		clk_prepare_enable(qphy->cfg_ahb_clk);
		qphy->clocks_enabled = true;
	}

	if (qphy->emulation) {
		/* Configure QUSB2 PLLs for RUMI */
		writel_relaxed(0x19, qphy->base + QRBTC_USB2_PLL);
		writel_relaxed(0x20, qphy->base + QRBTC_USB2_PLLCTRL2);
		writel_relaxed(0x79, qphy->base + QRBTC_USB2_PLLCTRL1);
		writel_relaxed(0x00, qphy->base + QRBTC_USB2_PLLCTRL3);
		writel_relaxed(0x99, qphy->base + QRBTC_USB2_PLL);
		writel_relaxed(0x04, qphy->base + QRBTC_USB2_PLLTEST1);
		writel_relaxed(0xD9, qphy->base + QRBTC_USB2_PLL);

		/* Wait for 5ms as per QUSB2 RUMI sequence from VI */
		usleep(5000);

		/* Perform the RUMI PLL Reset */
		writel_relaxed((int)RUMI_RESET_VALUE_1,
					qphy->base + RUMI_RESET_ADDRESS);
		/* Wait for 10ms as per QUSB2 RUMI sequence from VI */
		usleep(10000);
		writel_relaxed(0x0, qphy->base + RUMI_RESET_ADDRESS);
		/* Wait for 10ms as per QUSB2 RUMI sequence from VI */
		usleep(10000);
		writel_relaxed((int)RUMI_RESET_VALUE_2,
					qphy->base + RUMI_RESET_ADDRESS);
		/* Wait for 10ms as per QUSB2 RUMI sequence from VI */
		usleep(10000);
		writel_relaxed(0x0, qphy->base + RUMI_RESET_ADDRESS);
	} else {
		/* Disable the PHY */
		writel_relaxed(CLAMP_N_EN | FREEZIO_N | POWER_DOWN,
				qphy->base + QUSB2PHY_PORT_POWERDOWN);

		/* configure for ULPI mode if requested */
		if (qphy->ulpi_mode)
			writel_relaxed(0x0,
					qphy->base + QUSB2PHY_PORT_UTMI_CTRL2);
		if (qphy->qusb_tune) {

			t1 = qphy->qusb_tune >> 24;
			t2 = (qphy->qusb_tune) >> 16 & 0xFF;
			t3 = (qphy->qusb_tune) >> 8 & 0xFF;
			t4 = (qphy->qusb_tune) & 0xFF;

			/* Program tuning parameters for PHY */
			writel_relaxed(t1, qphy->base + QUSB2PHY_PORT_TUNE1);
			writel_relaxed(t2, qphy->base + QUSB2PHY_PORT_TUNE2);
			writel_relaxed(t3, qphy->base + QUSB2PHY_PORT_TUNE3);
			writel_relaxed(t4, qphy->base + QUSB2PHY_PORT_TUNE4);

		}
		/* ensure above writes are completed before re-enabling PHY */
		wmb();

		/* Enable the PHY */
		writel_relaxed(CLAMP_N_EN | FREEZIO_N,
				qphy->base + QUSB2PHY_PORT_POWERDOWN);
	}

	return 0;
}

static void qusb_phy_shutdown(struct usb_phy *phy)
{
	struct qusb_phy *qphy = container_of(phy, struct qusb_phy, phy);

	dev_dbg(phy->dev, "%s\n", __func__);

	/* clocks need to be on to access register */
	if (!qphy->clocks_enabled) {
		clk_prepare_enable(qphy->ref_clk);
		clk_prepare_enable(qphy->cfg_ahb_clk);
		qphy->clocks_enabled = true;
	}

	/* Disable the PHY */
	writel_relaxed(CLAMP_N_EN | FREEZIO_N | POWER_DOWN,
			qphy->base + QUSB2PHY_PORT_POWERDOWN);
	wmb();

	clk_disable_unprepare(qphy->cfg_ahb_clk);
	clk_disable_unprepare(qphy->ref_clk);
	qphy->clocks_enabled = false;
}

static void qusb_write_readback(void *base, u32 offset,
					const u32 mask, u32 val)
{
	u32 write_val, tmp = readl_relaxed(base + offset);
	tmp &= ~mask; /* retain other bits */
	write_val = tmp | val;

	writel_relaxed(write_val, base + offset);

	/* Read back to see if val was written */
	tmp = readl_relaxed(base + offset);
	tmp &= mask; /* clear other bits */

<<<<<<< HEAD
	if (tmp != val)
		pr_err("%s: write: %x to QSCRATCH: %x FAILED\n",
			__func__, val, offset);
=======
	
	qusb_phy_init(phy);

	return ret;
>>>>>>> 0e91d2a... Nougat
}

/**
 * Performs QUSB2 PHY suspend/resume functionality.
 *
 * @uphy - usb phy pointer.
 * @suspend - to enable suspend or not. 1 - suspend, 0 - resume
 *
 */
static int qusb_phy_set_suspend(struct usb_phy *phy, int suspend)
{
	struct qusb_phy *qphy = container_of(phy, struct qusb_phy, phy);
	u32 linestate = 0, intr_mask = 0;

	if (!qphy->clocks_enabled) {
		dev_dbg(phy->dev, "clocks not enabled yet\n");
		return -EAGAIN;
	}

	if (qphy->suspended && suspend) {
		dev_dbg(phy->dev, "%s: USB PHY is already suspended\n",
			__func__);
		return 0;
	}

	if (suspend) {
		/* Bus suspend case */
		if (qphy->cable_connected ||
			(qphy->phy.flags & PHY_HOST_MODE)) {
			/* Clear all interrupts */
			writel_relaxed(0x00,
				qphy->base + QUSB2PHY_PORT_INTR_CTRL);

			linestate = readl_relaxed(qphy->base +
					QUSB2PHY_PORT_UTMI_STATUS);

			/*
			 * D+/D- interrupts are level-triggered, but we are
			 * only interested if the line state changes, so enable
			 * the high/low trigger based on current state. In
			 * other words, enable the triggers _opposite_ of what
			 * the current D+/D- levels are.
			 * e.g. if currently D+ high, D- low (HS 'J'/Suspend),
			 * configure the mask to trigger on D+ low OR D- high
			 */
			intr_mask = DPSE_INTR_EN | DMSE_INTR_EN;
			if (!(linestate & LINESTATE_DP)) /* D+ low */
				intr_mask |= DPSE_INTR_HIGH_SEL;
			if (!(linestate & LINESTATE_DM)) /* D- low */
				intr_mask |= DMSE_INTR_HIGH_SEL;

			writel_relaxed(intr_mask,
				qphy->base + QUSB2PHY_PORT_INTR_CTRL);

<<<<<<< HEAD
			clk_disable_unprepare(qphy->cfg_ahb_clk);
			clk_disable_unprepare(qphy->ref_clk);
		} else { /* Disconnect case */
			clk_disable_unprepare(qphy->cfg_ahb_clk);
			clk_disable_unprepare(qphy->ref_clk);
			qusb_phy_enable_power(qphy, false);
=======
			qusb_phy_enable_clocks(qphy, false);
		} else { 
			
			writel_relaxed(0x00,
				qphy->base + QUSB2PHY_PORT_INTR_CTRL);

			
			wmb();

			qusb_phy_enable_clocks(qphy, false);
			if (!qphy->usb_phy_always_on)
				qusb_phy_enable_power(qphy, false);
			qphy->put_into_high_z_state = true;
>>>>>>> 0e91d2a... Nougat
		}
		qphy->suspended = true;
	} else {
		/* Bus suspend case */
		if (qphy->cable_connected ||
			(qphy->phy.flags & PHY_HOST_MODE)) {
			clk_prepare_enable(qphy->ref_clk);
			clk_prepare_enable(qphy->cfg_ahb_clk);
			/* Clear all interrupts on resume */
			writel_relaxed(0x00,
				qphy->base + QUSB2PHY_PORT_INTR_CTRL);
		} else {
			qusb_phy_enable_power(qphy, true);
<<<<<<< HEAD
			clk_prepare_enable(qphy->ref_clk);
			clk_prepare_enable(qphy->cfg_ahb_clk);
=======
			qusb_phy_enable_clocks(qphy, true);
>>>>>>> 0e91d2a... Nougat
		}
		qphy->suspended = false;
	}

	return 0;
}

static int qusb_phy_notify_connect(struct usb_phy *phy,
					enum usb_device_speed speed)
{
	struct qusb_phy *qphy = container_of(phy, struct qusb_phy, phy);

	qphy->cable_connected = true;

	dev_dbg(phy->dev, " cable_connected=%d\n", qphy->cable_connected);

	/* Set OTG VBUS Valid from HSPHY to controller */
	qusb_write_readback(qphy->qscratch_base, HS_PHY_CTRL_REG(0),
				UTMI_OTG_VBUS_VALID,
				UTMI_OTG_VBUS_VALID);

	/* Indicate value is driven by UTMI_OTG_VBUS_VALID bit */
	qusb_write_readback(qphy->qscratch_base, HS_PHY_CTRL_REG(0),
				SW_SESSVLD_SEL, SW_SESSVLD_SEL);

	dev_dbg(phy->dev, "QUSB2 phy connect notification\n");
	return 0;
}

static int qusb_phy_notify_disconnect(struct usb_phy *phy,
					enum usb_device_speed speed)
{
	struct qusb_phy *qphy = container_of(phy, struct qusb_phy, phy);

	qphy->cable_connected = false;

	dev_dbg(phy->dev, " cable_connected=%d\n", qphy->cable_connected);

	/* Set OTG VBUS Valid from HSPHY to controller */
	qusb_write_readback(qphy->qscratch_base, HS_PHY_CTRL_REG(0),
				UTMI_OTG_VBUS_VALID, 0);

	/* Indicate value is driven by UTMI_OTG_VBUS_VALID bit */
	qusb_write_readback(qphy->qscratch_base, HS_PHY_CTRL_REG(0),
				SW_SESSVLD_SEL, 0);

	dev_dbg(phy->dev, "QUSB2 phy disconnect notification\n");
	return 0;
}

static int qusb_phy_probe(struct platform_device *pdev)
{
	struct qusb_phy *qphy;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret = 0;
	const char *phy_type;
<<<<<<< HEAD
=======
	bool hold_phy_reset;
	struct property *prop = NULL;
	u32 buf;
>>>>>>> 0e91d2a... Nougat

	qphy = devm_kzalloc(dev, sizeof(*qphy), GFP_KERNEL);
	if (!qphy)
		return -ENOMEM;

	qphy->phy.dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"qusb_phy_base");
	qphy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(qphy->base))
		return PTR_ERR(qphy->base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"qscratch_base");
<<<<<<< HEAD
	qphy->qscratch_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(qphy->qscratch_base))
		qphy->qscratch_base = NULL;
=======
	if (res) {
		qphy->qscratch_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(qphy->qscratch_base)) {
			dev_dbg(dev, "couldn't ioremap qscratch_base\n");
			qphy->qscratch_base = NULL;
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"emu_phy_base");
	if (res) {
		qphy->emu_phy_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(qphy->emu_phy_base)) {
			dev_dbg(dev, "couldn't ioremap emu_phy_base\n");
			qphy->emu_phy_base = NULL;
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"tune2_efuse_addr");
	if (res) {
		qphy->tune2_efuse_reg = devm_ioremap_nocache(dev, res->start,
							resource_size(res));
		if (!IS_ERR_OR_NULL(qphy->tune2_efuse_reg)) {
			ret = of_property_read_u32(dev->of_node,
					"qcom,tune2-efuse-bit-pos",
					&qphy->tune2_efuse_bit_pos);
			if (!ret) {
				ret = of_property_read_u32(dev->of_node,
						"qcom,tune2-efuse-num-bits",
						&qphy->tune2_efuse_num_of_bits);
			}
			of_property_read_u32(dev->of_node,
						"qcom,tune2-efuse-correction",
						&qphy->tune2_efuse_correction);

			if (ret) {
				dev_err(dev, "DT Value for tune2 efuse is invalid.\n");
				return -EINVAL;
			}
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"ref_clk_addr");
	if (res) {
		qphy->ref_clk_base = devm_ioremap_nocache(dev,
				res->start, resource_size(res));
		if (IS_ERR(qphy->ref_clk_base))
			dev_dbg(dev, "ref_clk_address is not available.\n");
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"tcsr_phy_clk_scheme_sel");
	if (res) {
		qphy->tcsr_phy_clk_scheme_sel = devm_ioremap_nocache(dev,
				res->start, resource_size(res));
		if (IS_ERR(qphy->tcsr_phy_clk_scheme_sel))
			dev_dbg(dev, "err reading tcsr_phy_clk_scheme_sel\n");
	}

	qphy->dpdm_pulsing_enabled = of_property_read_bool(dev->of_node,
					"qcom,enable-dpdm-pulsing");

	qphy->ref_clk_src = devm_clk_get(dev, "ref_clk_src");
	if (IS_ERR(qphy->ref_clk_src))
		dev_dbg(dev, "clk get failed for ref_clk_src\n");
>>>>>>> 0e91d2a... Nougat

	qphy->ref_clk = devm_clk_get(dev, "ref_clk");
	if (IS_ERR(qphy->ref_clk))
		return PTR_ERR(qphy->ref_clk);
	clk_set_rate(qphy->ref_clk, 19200000);

	qphy->cfg_ahb_clk = devm_clk_get(dev, "cfg_ahb_clk");
	if (IS_ERR(qphy->cfg_ahb_clk))
		return PTR_ERR(qphy->cfg_ahb_clk);

	qphy->phy_reset = devm_clk_get(dev, "phy_reset");
	if (IS_ERR(qphy->phy_reset))
		return PTR_ERR(qphy->phy_reset);

	if (of_property_match_string(dev->of_node,
		"clock-names", "iface_clk") >= 0) {
		qphy->iface_clk = devm_clk_get(dev, "iface_clk");
		if (IS_ERR(qphy->iface_clk)) {
			ret = PTR_ERR(qphy->iface_clk);
			qphy->iface_clk = NULL;
			if (ret == -EPROBE_DEFER)
				return ret;
			dev_err(dev, "couldn't get iface_clk(%d)\n", ret);
		}
	}

	if (of_property_match_string(dev->of_node,
		"clock-names", "core_clk") >= 0) {
		qphy->core_clk = devm_clk_get(dev, "core_clk");
		if (IS_ERR(qphy->core_clk)) {
			ret = PTR_ERR(qphy->core_clk);
			qphy->core_clk = NULL;
			if (ret == -EPROBE_DEFER)
				return ret;
			dev_err(dev, "couldn't get core_clk(%d)\n", ret);
		}
	}

	qphy->gdsc = devm_regulator_get(dev, "USB3_GDSC");
	if (IS_ERR(qphy->gdsc))
		qphy->gdsc = NULL;

	qphy->emulation = of_property_read_bool(dev->of_node,
<<<<<<< HEAD
						"qcom,emulation");

	of_property_read_u32(dev->of_node, "qcom,qusb-tune", &qphy->qusb_tune);
=======
					"qcom,emulation");

	of_get_property(dev->of_node, "qcom,emu-init-seq", &size);
	if (size) {
		qphy->emu_init_seq = devm_kzalloc(dev,
						size, GFP_KERNEL);
		if (qphy->emu_init_seq) {
			qphy->emu_init_seq_len =
				(size / sizeof(*qphy->emu_init_seq));
			if (qphy->emu_init_seq_len % 2) {
				dev_err(dev, "invalid emu_init_seq_len\n");
				return -EINVAL;
			}

			of_property_read_u32_array(dev->of_node,
				"qcom,emu-init-seq",
				qphy->emu_init_seq,
				qphy->emu_init_seq_len);
		} else {
			dev_dbg(dev, "error allocating memory for emu_init_seq\n");
		}
	}

	of_get_property(dev->of_node, "qcom,phy-pll-reset-seq", &size);
	if (size) {
		qphy->phy_pll_reset_seq = devm_kzalloc(dev,
						size, GFP_KERNEL);
		if (qphy->phy_pll_reset_seq) {
			qphy->phy_pll_reset_seq_len =
				(size / sizeof(*qphy->phy_pll_reset_seq));
			if (qphy->phy_pll_reset_seq_len % 2) {
				dev_err(dev, "invalid phy_pll_reset_seq_len\n");
				return -EINVAL;
			}

			of_property_read_u32_array(dev->of_node,
				"qcom,phy-pll-reset-seq",
				qphy->phy_pll_reset_seq,
				qphy->phy_pll_reset_seq_len);
		} else {
			dev_dbg(dev, "error allocating memory for phy_pll_reset_seq\n");
		}
	}

	of_get_property(dev->of_node, "qcom,emu-dcm-reset-seq", &size);
	if (size) {
		qphy->emu_dcm_reset_seq = devm_kzalloc(dev,
						size, GFP_KERNEL);
		if (qphy->emu_dcm_reset_seq) {
			qphy->emu_dcm_reset_seq_len =
				(size / sizeof(*qphy->emu_dcm_reset_seq));
			if (qphy->emu_dcm_reset_seq_len % 2) {
				dev_err(dev, "invalid emu_dcm_reset_seq_len\n");
				return -EINVAL;
			}

			of_property_read_u32_array(dev->of_node,
				"qcom,emu-dcm-reset-seq",
				qphy->emu_dcm_reset_seq,
				qphy->emu_dcm_reset_seq_len);
		} else {
			dev_dbg(dev, "error allocating memory for emu_dcm_reset_seq\n");
		}
	}

	prop = of_find_property(dev->of_node, "htc,usb-phy-always-on", NULL);
	if (prop) {
		of_property_read_u32(dev->of_node, "htc,usb-phy-always-on", &buf);
		qphy->usb_phy_always_on = buf;
		dev_dbg(dev, "usb-phy-always-on = %d\n", qphy->usb_phy_always_on);
	} else {
		dev_err(dev, "usb-phy-always-on was not found\n");
	}

	of_get_property(dev->of_node, "qcom,qusb-phy-init-seq", &size);
	if (size) {
		qphy->qusb_phy_init_seq = devm_kzalloc(dev,
						size, GFP_KERNEL);
		if (qphy->qusb_phy_init_seq) {
			qphy->init_seq_len =
				(size / sizeof(*qphy->qusb_phy_init_seq));
			if (qphy->init_seq_len % 2) {
				dev_err(dev, "invalid init_seq_len\n");
				return -EINVAL;
			}

			of_property_read_u32_array(dev->of_node,
				"qcom,qusb-phy-init-seq",
				qphy->qusb_phy_init_seq,
				qphy->init_seq_len);
		} else {
			dev_err(dev, "error allocating memory for phy_init_seq\n");
		}
	}
>>>>>>> 0e91d2a... Nougat

	qphy->ulpi_mode = false;
	ret = of_property_read_string(dev->of_node, "phy_type", &phy_type);

	if (!ret) {
		if (!strcasecmp(phy_type, "ulpi"))
			qphy->ulpi_mode = true;
	} else {
		dev_err(dev, "error reading phy_type property\n");
		return ret;
	}

	ret = of_property_read_u32_array(dev->of_node, "qcom,vdd-voltage-level",
					 (u32 *) qphy->vdd_levels,
					 ARRAY_SIZE(qphy->vdd_levels));
	if (ret) {
		dev_err(dev, "error reading qcom,vdd-voltage-level property\n");
		return ret;
	}

	qphy->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(qphy->vdd)) {
		dev_err(dev, "unable to get vdd supply\n");
		return PTR_ERR(qphy->vdd);
	}

	qphy->vdda33 = devm_regulator_get(dev, "vdda33");
	if (IS_ERR(qphy->vdda33)) {
		dev_err(dev, "unable to get vdda33 supply\n");
		return PTR_ERR(qphy->vdda33);
	}

	qphy->vdda18 = devm_regulator_get(dev, "vdda18");
	if (IS_ERR(qphy->vdda18)) {
		dev_err(dev, "unable to get vdda18 supply\n");
		return PTR_ERR(qphy->vdda18);
	}

	platform_set_drvdata(pdev, qphy);

	qphy->phy.label			= "msm-qusb-phy";
	qphy->phy.init			= qusb_phy_init;
	qphy->phy.set_suspend           = qusb_phy_set_suspend;
	qphy->phy.shutdown		= qusb_phy_shutdown;
	qphy->phy.reset			= qusb_phy_reset;
	qphy->phy.type			= USB_PHY_TYPE_USB2;

	if (qphy->qscratch_base) {
		qphy->phy.notify_connect        = qusb_phy_notify_connect;
		qphy->phy.notify_disconnect     = qusb_phy_notify_disconnect;
	}

	qusb_phy_reset(&qphy->phy);
	ret = usb_add_phy_dev(&qphy->phy);

	return ret;
}

static int qusb_phy_remove(struct platform_device *pdev)
{
	struct qusb_phy *qphy = platform_get_drvdata(pdev);

	usb_remove_phy(&qphy->phy);

	if (qphy->clocks_enabled) {
		clk_disable_unprepare(qphy->cfg_ahb_clk);
		clk_disable_unprepare(qphy->ref_clk);
		qphy->clocks_enabled = false;
	}

	qusb_phy_enable_power(qphy, false);

	return 0;
}

static const struct of_device_id qusb_phy_id_table[] = {
	{ .compatible = "qcom,qusb2phy", },
	{ },
};
MODULE_DEVICE_TABLE(of, qusb_phy_id_table);

static struct platform_driver qusb_phy_driver = {
	.probe		= qusb_phy_probe,
	.remove		= qusb_phy_remove,
	.driver = {
		.name	= "msm-qusb-phy",
		.of_match_table = of_match_ptr(qusb_phy_id_table),
	},
};

module_platform_driver(qusb_phy_driver);

MODULE_DESCRIPTION("MSM QUSB2 PHY driver");
MODULE_LICENSE("GPL v2");
