/*
 * Copyright (c) 2012-2016 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include "wil6210.h"

static int use_msi = 1;
module_param(use_msi, int, S_IRUGO);
MODULE_PARM_DESC(use_msi,
		 " Use MSI interrupt: "
		 "0 - don't, 1 - (default) - single, or 3");

static bool debug_fw; /* = false; */
module_param(debug_fw, bool, S_IRUGO);
MODULE_PARM_DESC(debug_fw, " load driver if FW not ready. For FW debug");

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
static int wil6210_pm_notify(struct notifier_block *notify_block,
			     unsigned long mode, void *unused);
#endif /* CONFIG_PM_SLEEP */
#endif /* CONFIG_PM */

static
void wil_set_capabilities(struct wil6210_priv *wil)
{
	u32 rev_id = ioread32(wil->csr + HOSTADDR(RGF_USER_JTAG_DEV_ID));

	bitmap_zero(wil->hw_capabilities, hw_capability_last);

	switch (rev_id) {
	case JTAG_DEV_ID_MARLON_B0:
		wil->hw_name = "Marlon B0";
		wil->hw_version = HW_VER_MARLON_B0;
		break;
	case JTAG_DEV_ID_SPARROW_A0:
		wil->hw_name = "Sparrow A0";
		wil->hw_version = HW_VER_SPARROW_A0;
		break;
	case JTAG_DEV_ID_SPARROW_A1:
		wil->hw_name = "Sparrow A1";
		wil->hw_version = HW_VER_SPARROW_A1;
		break;
	case JTAG_DEV_ID_SPARROW_B0:
		wil->hw_name = "Sparrow B0";
		wil->hw_version = HW_VER_SPARROW_B0;
		break;
	default:
		wil_err(wil, "Unknown board hardware 0x%08x\n", rev_id);
		wil->hw_name = "Unknown";
		wil->hw_version = HW_VER_UNKNOWN;
	}

	wil_info(wil, "Board hardware is %s\n", wil->hw_name);

	if (wil->hw_version >= HW_VER_SPARROW_A0)
		set_bit(hw_capability_reset_v2, wil->hw_capabilities);

	if (wil->hw_version >= HW_VER_SPARROW_B0)
		set_bit(hw_capability_advanced_itr_moderation,
			wil->hw_capabilities);
}

void wil_disable_irq(struct wil6210_priv *wil)
{
	int irq = wil->pdev->irq;

	disable_irq(irq);
	if (wil->n_msi == 3) {
		disable_irq(irq + 1);
		disable_irq(irq + 2);
	}
}

void wil_enable_irq(struct wil6210_priv *wil)
{
	int irq = wil->pdev->irq;

	enable_irq(irq);
	if (wil->n_msi == 3) {
		enable_irq(irq + 1);
		enable_irq(irq + 2);
	}
}

/* Bus ops */
static int wil_if_pcie_enable(struct wil6210_priv *wil)
{
	struct pci_dev *pdev = wil->pdev;
	int rc;
	/* on platforms with buggy ACPI, pdev->msi_enabled may be set to
	 * allow pci_enable_device to work. This indicates INTx was not routed
	 * and only MSI should be used
	 */
	int msi_only = pdev->msi_enabled;

	wil_dbg_misc(wil, "%s()\n", __func__);

	pdev->msi_enabled = 0;

	pci_set_master(pdev);

	/*
	 * how many MSI interrupts to request?
	 */
	switch (use_msi) {
	case 3:
	case 1:
	case 0:
		break;
	default:
		wil_err(wil, "Invalid use_msi=%d, default to 1\n",
			use_msi);
		use_msi = 1;
	}
	wil->n_msi = use_msi;
	if (wil->n_msi) {
		wil_dbg_misc(wil, "Setup %d MSI interrupts\n", use_msi);
		rc = pci_enable_msi_block(pdev, wil->n_msi);
		if (rc && (wil->n_msi == 3)) {
			wil_err(wil, "3 MSI mode failed, try 1 MSI\n");
			wil->n_msi = 1;
			rc = pci_enable_msi_block(pdev, wil->n_msi);
		}
		if (rc) {
			wil_err(wil, "pci_enable_msi failed, use INTx\n");
			wil->n_msi = 0;
		}
	} else {
		wil_dbg_misc(wil, "MSI interrupts disabled, use INTx\n");
	}

	if ((wil->n_msi == 0) && msi_only) {
		wil_err(wil, "Interrupt pin not routed, unable to use INTx\n");
		rc = -ENODEV;
		goto stop_master;
	}

	rc = wil6210_init_irq(wil, pdev->irq);
	if (rc)
		goto stop_master;

	/* need reset here to obtain MAC */
	mutex_lock(&wil->mutex);
	rc = wil_reset(wil);
	mutex_unlock(&wil->mutex);
	if (debug_fw)
		rc = 0;
	if (rc)
		goto release_irq;

	return 0;

 release_irq:
	wil6210_fini_irq(wil, pdev->irq);
	/* safe to call if no MSI */
	pci_disable_msi(pdev);
 stop_master:
	pci_clear_master(pdev);
	return rc;
}

static int wil_if_pcie_disable(struct wil6210_priv *wil)
{
	struct pci_dev *pdev = wil->pdev;

	wil_dbg_misc(wil, "%s()\n", __func__);

	pci_clear_master(pdev);
	/* disable and release IRQ */
	wil6210_fini_irq(wil, pdev->irq);
	/* safe to call if no MSI */
	pci_disable_msi(pdev);
	/* TODO: disable HW */

	return 0;
}

static int wil_platform_rop_ramdump(void *wil_handle, void *buf, uint32_t size)
{
	struct wil6210_priv *wil = wil_handle;

	if (!wil)
		return -EINVAL;

	return wil_fw_copy_crash_dump(wil, buf, size);
}

static int wil_platform_rop_fw_recovery(void *wil_handle)
{
	struct wil6210_priv *wil = wil_handle;

	if (!wil)
		return -EINVAL;

	wil_fw_error_recovery(wil);

	return 0;
}

static int wil_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct wil6210_priv *wil;
	struct device *dev = &pdev->dev;
	void __iomem *csr;
	int rc;
	const struct wil_platform_rops rops = {
		.ramdump = wil_platform_rop_ramdump,
		.fw_recovery = wil_platform_rop_fw_recovery,
	};

	/* check HW */
	dev_info(&pdev->dev, WIL_NAME
		 " device found [%04x:%04x] (rev %x)\n",
		 (int)pdev->vendor, (int)pdev->device, (int)pdev->revision);

	if (pci_resource_len(pdev, 0) != WIL6210_MEM_SIZE) {
		dev_err(&pdev->dev, "Not " WIL_NAME "? "
			"BAR0 size is %lu while expecting %lu\n",
			(ulong)pci_resource_len(pdev, 0), WIL6210_MEM_SIZE);
		return -ENODEV;
	}

<<<<<<< HEAD
=======
	wil = wil_if_alloc(dev);
	if (IS_ERR(wil)) {
		rc = (int)PTR_ERR(wil);
		dev_err(dev, "wil_if_alloc failed: %d\n", rc);
		return rc;
	}
	wil->pdev = pdev;
	pci_set_drvdata(pdev, wil);
	/* rollback to if_free */

	wil->platform_handle =
		wil_platform_init(&pdev->dev, &wil->platform_ops, &rops, wil);
	if (!wil->platform_handle) {
		rc = -ENODEV;
		wil_err(wil, "wil_platform_init failed\n");
		goto if_free;
	}
	/* rollback to err_plat */

>>>>>>> 0e91d2a... Nougat
	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev,
			"pci_enable_device failed, retry with MSI only\n");
		/* Work around for platforms that can't allocate IRQ:
		 * retry with MSI only
		 */
		pdev->msi_enabled = 1;
		rc = pci_enable_device(pdev);
	}
	if (rc)
		return -ENODEV;
	/* rollback to err_disable_pdev */

	rc = pci_request_region(pdev, 0, WIL_NAME);
	if (rc) {
		dev_err(&pdev->dev, "pci_request_region failed\n");
		goto err_disable_pdev;
	}
	/* rollback to err_release_reg */

	csr = pci_ioremap_bar(pdev, 0);
	if (!csr) {
		dev_err(&pdev->dev, "pci_ioremap_bar failed\n");
		rc = -ENODEV;
		goto err_release_reg;
	}
	/* rollback to err_iounmap */
	dev_info(&pdev->dev, "CSR at %pR -> 0x%p\n", &pdev->resource[0], csr);

	wil = wil_if_alloc(dev, csr);
	if (IS_ERR(wil)) {
		rc = (int)PTR_ERR(wil);
		dev_err(dev, "wil_if_alloc failed: %d\n", rc);
		goto err_iounmap;
	}
	/* rollback to if_free */

	pci_set_drvdata(pdev, wil);
	wil->pdev = pdev;
	wil_set_capabilities(wil);
	wil6210_clear_irq(wil);

	wil->platform_handle =
			wil_platform_init(&pdev->dev, &wil->platform_ops);

	/* FW should raise IRQ when ready */
	rc = wil_if_pcie_enable(wil);
	if (rc) {
		wil_err(wil, "Enable device failed\n");
		goto if_free;
	}
	/* rollback to bus_disable */

	rc = wil_if_add(wil);
	if (rc) {
		wil_err(wil, "wil_if_add failed: %d\n", rc);
		goto bus_disable;
	}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
	wil->pm_notify.notifier_call = wil6210_pm_notify;
	rc = register_pm_notifier(&wil->pm_notify);
	if (rc)
		/* Do not fail the driver initialization, as suspend can
		 * be prevented in a later phase if needed
		 */
		wil_err(wil, "register_pm_notifier failed: %d\n", rc);
#endif /* CONFIG_PM_SLEEP */
#endif /* CONFIG_PM */

	wil6210_debugfs_init(wil);

	/* check FW is alive */
	wmi_echo(wil);

	return 0;

 bus_disable:
	wil_if_pcie_disable(wil);
 if_free:
	if (wil->platform_ops.uninit)
		wil->platform_ops.uninit(wil->platform_handle);
	wil_if_free(wil);
 err_iounmap:
	pci_iounmap(pdev, csr);
 err_release_reg:
	pci_release_region(pdev, 0);
 err_disable_pdev:
	pci_disable_device(pdev);

	return rc;
}

static void wil_pcie_remove(struct pci_dev *pdev)
{
	struct wil6210_priv *wil = pci_get_drvdata(pdev);
	void __iomem *csr = wil->csr;

	wil_dbg_misc(wil, "%s()\n", __func__);

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
	unregister_pm_notifier(&wil->pm_notify);
#endif /* CONFIG_PM_SLEEP */
#endif /* CONFIG_PM */

	wil6210_debugfs_remove(wil);
	wil_if_remove(wil);
	wil_if_pcie_disable(wil);
	if (wil->platform_ops.uninit)
		wil->platform_ops.uninit(wil->platform_handle);
	wil_p2p_wdev_free(wil);
	wil_if_free(wil);
	pci_iounmap(pdev, csr);
	pci_release_region(pdev, 0);
	pci_disable_device(pdev);
}

static const struct pci_device_id wil6210_pcie_ids[] = {
	{ PCI_DEVICE(0x1ae9, 0x0301) },
	{ PCI_DEVICE(0x1ae9, 0x0310) },
	{ PCI_DEVICE(0x1ae9, 0x0302) }, /* same as above, firmware broken */
	{ /* end: all zeroes */	},
};
MODULE_DEVICE_TABLE(pci, wil6210_pcie_ids);

<<<<<<< HEAD
=======
#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP

static int wil6210_suspend(struct device *dev, bool is_runtime)
{
	int rc = 0;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wil6210_priv *wil = pci_get_drvdata(pdev);

	wil_dbg_pm(wil, "%s(%s)\n", __func__,
		   is_runtime ? "runtime" : "system");

	rc = wil_can_suspend(wil, is_runtime);
	if (rc)
		goto out;

	rc = wil_suspend(wil, is_runtime);
	if (rc)
		goto out;

	/* TODO: how do I bring card in low power state? */

	/* disable bus mastering */
	pci_clear_master(pdev);
	/* PCI will call pci_save_state(pdev) and pci_prepare_to_sleep(pdev) */

out:
	return rc;
}

static int wil6210_resume(struct device *dev, bool is_runtime)
{
	int rc = 0;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct wil6210_priv *wil = pci_get_drvdata(pdev);

	wil_dbg_pm(wil, "%s(%s)\n", __func__,
		   is_runtime ? "runtime" : "system");

	/* allow master */
	pci_set_master(pdev);

	rc = wil_resume(wil, is_runtime);
	if (rc)
		pci_clear_master(pdev);

	return rc;
}

static int wil6210_pm_notify(struct notifier_block *notify_block,
			     unsigned long mode, void *unused)
{
	struct wil6210_priv *wil = container_of(
		notify_block, struct wil6210_priv, pm_notify);
	int rc = 0;
	enum wil_platform_event evt;

	wil_dbg_pm(wil, "%s: mode (%ld)\n", __func__, mode);

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
	case PM_RESTORE_PREPARE:
		rc = wil_can_suspend(wil, false);
		if (rc)
			break;
		evt = WIL_PLATFORM_EVT_PRE_SUSPEND;
		if (wil->platform_ops.notify)
			rc = wil->platform_ops.notify(wil->platform_handle,
						      evt);
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		evt = WIL_PLATFORM_EVT_POST_SUSPEND;
		if (wil->platform_ops.notify)
			rc = wil->platform_ops.notify(wil->platform_handle,
						      evt);
		break;
	default:
		wil_dbg_pm(wil, "unhandled notify mode %ld\n", mode);
		break;
	}

	wil_dbg_pm(wil, "notification mode %ld: rc (%d)\n", mode, rc);
	return rc;
}

static int wil6210_pm_suspend(struct device *dev)
{
	return wil6210_suspend(dev, false);
}

static int wil6210_pm_resume(struct device *dev)
{
	return wil6210_resume(dev, false);
}
#endif /* CONFIG_PM_SLEEP */

#endif /* CONFIG_PM */

static const struct dev_pm_ops wil6210_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wil6210_pm_suspend, wil6210_pm_resume)
};

>>>>>>> 0e91d2a... Nougat
static struct pci_driver wil6210_driver = {
	.probe		= wil_pcie_probe,
	.remove		= wil_pcie_remove,
	.id_table	= wil6210_pcie_ids,
	.name		= WIL_NAME,
};

module_pci_driver(wil6210_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Qualcomm Atheros <wil6210@qca.qualcomm.com>");
MODULE_DESCRIPTION("Driver for 60g WiFi WIL6210 card");
