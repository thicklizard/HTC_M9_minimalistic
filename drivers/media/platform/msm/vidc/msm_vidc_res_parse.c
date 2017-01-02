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

#include <linux/of.h>
#include <linux/slab.h>
#include "msm_vidc_resources.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_res_parse.h"

enum clock_properties {
	CLOCK_PROP_HAS_SCALING = 1 << 0,
	CLOCK_PROP_HAS_GATING = 1 << 1,
};

static size_t get_u32_array_num_elements(struct platform_device *pdev,
					char *name)
{
	struct device_node *np = pdev->dev.of_node;
	int len;
	size_t num_elements = 0;
	if (!of_get_property(np, name, &len)) {
		dprintk(VIDC_ERR, "Failed to read %s from device tree\n",
			name);
		goto fail_read;
	}

	num_elements = len / sizeof(u32);
	if (num_elements <= 0) {
		dprintk(VIDC_ERR, "%s not specified in device tree\n",
			name);
		goto fail_read;
	}
	return num_elements;

fail_read:
	return 0;
}

int read_hfi_type(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int rc = 0;
	const char *hfi_name = NULL;

	if (np) {
		rc = of_property_read_string(np, "qcom,hfi", &hfi_name);
		if (rc) {
			dprintk(VIDC_ERR,
				"Failed to read hfi from device tree\n");
			goto err_hfi_read;
		}
		if (!strcmp(hfi_name, "venus"))
			rc = VIDC_HFI_VENUS;
		else if (!strcmp(hfi_name, "q6"))
			rc = VIDC_HFI_Q6;
		else
			rc = -EINVAL;
	} else
		rc = VIDC_HFI_Q6;

err_hfi_read:
	return rc;
}

static inline void msm_vidc_free_platform_version_table(
		struct msm_vidc_platform_resources *res)
{
	res->pf_ver_tbl = NULL;
}

static inline void msm_vidc_free_freq_table(
		struct msm_vidc_platform_resources *res)
{
	res->load_freq_tbl = NULL;
}

static inline void msm_vidc_free_reg_table(
			struct msm_vidc_platform_resources *res)
{
	res->reg_set.reg_tbl = NULL;
}

static inline void msm_vidc_free_qdss_addr_table(
			struct msm_vidc_platform_resources *res)
{
	res->qdss_addr_set.addr_tbl = NULL;
}

static inline void msm_vidc_free_bus_vectors(
			struct msm_vidc_platform_resources *res)
{
	int i = 0;
	for (i = 0; i < res->bus_set.count; i++) {
		if (res->bus_set.bus_tbl[i].pdata)
			msm_bus_cl_clear_pdata(res->bus_set.bus_tbl[i].pdata);
	}
}

static inline void msm_vidc_free_iommu_groups(
			struct msm_vidc_platform_resources *res)
{
	res->iommu_group_set.iommu_maps = NULL;
}

static inline void msm_vidc_free_regulator_table(
			struct msm_vidc_platform_resources *res)
{
	int c = 0;
	for (c = 0; c < res->regulator_set.count; ++c) {
		struct regulator_info *rinfo =
			&res->regulator_set.regulator_tbl[c];

		rinfo->name = NULL;
	}

	res->regulator_set.regulator_tbl = NULL;
	res->regulator_set.count = 0;
}

static inline void msm_vidc_free_clock_table(
			struct msm_vidc_platform_resources *res)
{
	res->clock_set.clock_tbl = NULL;
	res->clock_set.count = 0;
}

void msm_vidc_free_platform_resources(
			struct msm_vidc_platform_resources *res)
{
	msm_vidc_free_clock_table(res);
	msm_vidc_free_regulator_table(res);
	msm_vidc_free_freq_table(res);
<<<<<<< HEAD
=======
	msm_vidc_free_platform_version_table(res);
	msm_vidc_free_dcvs_table(res);
	msm_vidc_free_dcvs_limit(res);
	msm_vidc_free_cycles_per_mb_table(res);
	msm_vidc_free_allowed_clocks_table(res);
>>>>>>> 0e91d2a... Nougat
	msm_vidc_free_reg_table(res);
	msm_vidc_free_qdss_addr_table(res);
	msm_vidc_free_bus_vectors(res);
	msm_vidc_free_iommu_groups(res);
}

static int msm_vidc_load_reg_table(struct msm_vidc_platform_resources *res)
{
	struct reg_set *reg_set;
	struct platform_device *pdev = res->pdev;
	int i;
	int rc = 0;

	if (!of_find_property(pdev->dev.of_node, "qcom,reg-presets", NULL)) {
		/* qcom,reg-presets is an optional property.  It likely won't be
		 * present if we don't have any register settings to program */
		dprintk(VIDC_DBG, "qcom,reg-presets not found\n");
		return 0;
	}

	reg_set = &res->reg_set;
	reg_set->count = get_u32_array_num_elements(pdev, "qcom,reg-presets");
	reg_set->count /=  sizeof(*reg_set->reg_tbl) / sizeof(u32);

	if (reg_set->count == 0) {
		dprintk(VIDC_DBG, "no elements in reg set\n");
		return rc;
	}

	reg_set->reg_tbl = devm_kzalloc(&pdev->dev, reg_set->count *
			sizeof(*(reg_set->reg_tbl)), GFP_KERNEL);
	if (!reg_set->reg_tbl) {
		dprintk(VIDC_ERR, "%s Failed to alloc register table\n",
			__func__);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(pdev->dev.of_node, "qcom,reg-presets",
		(u32 *)reg_set->reg_tbl, reg_set->count * 2)) {
		dprintk(VIDC_ERR, "Failed to read register table\n");
		msm_vidc_free_reg_table(res);
		return -EINVAL;
	}
	for (i = 0; i < reg_set->count; i++) {
		dprintk(VIDC_DBG,
			"reg = %x, value = %x\n",
			reg_set->reg_tbl[i].reg,
			reg_set->reg_tbl[i].value
		);
	}
	return rc;
}
static int msm_vidc_load_qdss_table(struct msm_vidc_platform_resources *res)
{
	struct addr_set *qdss_addr_set;
	struct platform_device *pdev = res->pdev;
	int i;
	int rc = 0;

	if (!of_find_property(pdev->dev.of_node, "qcom,qdss-presets", NULL)) {
		/* qcom,qdss-presets is an optional property. It likely won't be
		 * present if we don't have any register settings to program */
		dprintk(VIDC_DBG, "qcom,qdss-presets not found\n");
		return rc;
	}

	qdss_addr_set = &res->qdss_addr_set;
	qdss_addr_set->count = get_u32_array_num_elements(pdev,
					"qcom,qdss-presets");
	qdss_addr_set->count /= sizeof(*qdss_addr_set->addr_tbl) / sizeof(u32);

	if (qdss_addr_set->count == 0) {
		dprintk(VIDC_DBG, "no elements in qdss reg set\n");
		return rc;
	}

	qdss_addr_set->addr_tbl = devm_kzalloc(&pdev->dev,
			qdss_addr_set->count * sizeof(*qdss_addr_set->addr_tbl),
			GFP_KERNEL);
	if (!qdss_addr_set->addr_tbl) {
		dprintk(VIDC_ERR, "%s Failed to alloc register table\n",
			__func__);
		rc = -ENOMEM;
		goto err_qdss_addr_tbl;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node, "qcom,qdss-presets",
		(u32 *)qdss_addr_set->addr_tbl, qdss_addr_set->count * 2);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read qdss address table\n");
		msm_vidc_free_qdss_addr_table(res);
		rc = -EINVAL;
		goto err_qdss_addr_tbl;
	}

	for (i = 0; i < qdss_addr_set->count; i++) {
		dprintk(VIDC_DBG, "qdss addr = %x, value = %x\n",
				qdss_addr_set->addr_tbl[i].start,
				qdss_addr_set->addr_tbl[i].size);
	}
err_qdss_addr_tbl:
	return rc;
}

<<<<<<< HEAD
=======
static int msm_vidc_load_imem_ab_table(struct msm_vidc_platform_resources *res)
{
	int num_elements = 0;
	struct platform_device *pdev = res->pdev;

	if (!of_find_property(pdev->dev.of_node, "qcom,imem-ab-tbl", NULL)) {
		/* optional property */
		dprintk(VIDC_DBG, "qcom,imem-freq-tbl not found\n");
		return 0;
	}

	num_elements = get_u32_array_num_elements(pdev->dev.of_node,
			"qcom,imem-ab-tbl");
	num_elements /= (sizeof(*res->imem_ab_tbl) / sizeof(u32));
	if (!num_elements) {
		dprintk(VIDC_ERR, "no elements in imem ab table\n");
		return -EINVAL;
	}

	res->imem_ab_tbl = devm_kzalloc(&pdev->dev, num_elements *
			sizeof(*res->imem_ab_tbl), GFP_KERNEL);
	if (!res->imem_ab_tbl) {
		dprintk(VIDC_ERR, "Failed to alloc imem_ab_tbl\n");
		return -ENOMEM;
	}

	if (of_property_read_u32_array(pdev->dev.of_node,
		"qcom,imem-ab-tbl", (u32 *)res->imem_ab_tbl,
		num_elements * sizeof(*res->imem_ab_tbl) / sizeof(u32))) {
		dprintk(VIDC_ERR, "Failed to read imem_ab_tbl\n");
		msm_vidc_free_imem_ab_table(res);
		return -EINVAL;
	}

	res->imem_ab_tbl_size = num_elements;

	return 0;
}

/**
 * msm_vidc_load_u32_table() - load dtsi table entries
 * @pdev: A pointer to the platform device.
 * @of_node:      A pointer to the device node.
 * @table_name:   A pointer to the dtsi table entry name.
 * @struct_size:  The size of the structure which is nothing but
 *                a single entry in the dtsi table.
 * @table:        A pointer to the table pointer which needs to be
 *                filled by the dtsi table entries.
 * @num_elements: Number of elements pointer which needs to be filled
 *                with the number of elements in the table.
 *
 * This is a generic implementation to load single or multiple array
 * table from dtsi. The array elements should be of size equal to u32.
 *
 * Return:        Return '0' for success else appropriate error value.
 */
int msm_vidc_load_u32_table(struct platform_device *pdev,
		struct device_node *of_node, char *table_name, int struct_size,
		u32 **table, u32 *num_elements)
{
	int rc = 0, num_elemts = 0;
	u32 *ptbl = NULL;

	if (!of_find_property(of_node, table_name, NULL)) {
		dprintk(VIDC_DBG, "%s not found\n", table_name);
		return 0;
	}

	num_elemts = get_u32_array_num_elements(of_node, table_name);
	if (!num_elemts) {
		dprintk(VIDC_ERR, "no elements in %s\n", table_name);
		return 0;
	}
	num_elemts /= struct_size / sizeof(u32);

	ptbl = devm_kzalloc(&pdev->dev, num_elemts * struct_size, GFP_KERNEL);
	if (!ptbl) {
		dprintk(VIDC_ERR, "Failed to alloc table %s\n", table_name);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(of_node, table_name, ptbl,
			num_elemts * struct_size / sizeof(u32))) {
		dprintk(VIDC_ERR, "Failed to read %s\n", table_name);
		return -EINVAL;
	}

	*table = ptbl;
	if (num_elements)
		*num_elements = num_elemts;

	return rc;
}

static int msm_vidc_load_platform_version_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct platform_device *pdev = res->pdev;

	if (!of_find_property(pdev->dev.of_node,
			"qcom,platform-version", NULL)) {
		dprintk(VIDC_DBG, "qcom,platform-version not found\n");
		return 0;
	}

	rc = msm_vidc_load_u32_table(pdev, pdev->dev.of_node,
			"qcom,platform-version",
			sizeof(*res->pf_ver_tbl),
			(u32 **)&res->pf_ver_tbl,
			NULL);
	if (rc) {
		dprintk(VIDC_ERR,
			"%s: failed to read platform version table\n",
			__func__);
		return rc;
	}

	return 0;
}

static int msm_vidc_load_allowed_clocks_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct platform_device *pdev = res->pdev;

	if (!of_find_property(pdev->dev.of_node,
			"qcom,allowed-clock-rates", NULL)) {
		dprintk(VIDC_DBG, "qcom,allowed-clock-rates not found\n");
		return 0;
	}

	rc = msm_vidc_load_u32_table(pdev, pdev->dev.of_node,
				"qcom,allowed-clock-rates",
				sizeof(*res->allowed_clks_tbl),
				(u32 **)&res->allowed_clks_tbl,
				&res->allowed_clks_tbl_size);
	if (rc) {
		dprintk(VIDC_ERR,
			"%s: failed to read allowed clocks table\n", __func__);
		return rc;
	}

	return 0;
}

static int msm_vidc_load_cycles_per_mb_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0, i = 0;
	struct clock_freq_table *clock_freq_tbl = &res->clock_freq_tbl;
	struct clock_profile_entry *entry = NULL;
	struct device_node *parent_node = NULL;
	struct device_node *child_node = NULL;
	struct platform_device *pdev = res->pdev;

	parent_node = of_find_node_by_name(pdev->dev.of_node,
			"qcom,clock-freq-tbl");
	if (!parent_node) {
		dprintk(VIDC_DBG, "Node qcom,clock-freq-tbl not found.\n");
		return 0;
	}

	clock_freq_tbl->count = 0;
	for_each_child_of_node(parent_node, child_node)
		clock_freq_tbl->count++;

	if (!clock_freq_tbl->count) {
		dprintk(VIDC_DBG, "No child nodes in qcom,clock-freq-tbl\n");
		return 0;
	}

	clock_freq_tbl->clk_prof_entries = devm_kzalloc(&pdev->dev,
		sizeof(*clock_freq_tbl->clk_prof_entries) *
		clock_freq_tbl->count, GFP_KERNEL);
	if (!clock_freq_tbl->clk_prof_entries) {
		dprintk(VIDC_DBG, "no memory to allocate clk_prof_entries\n");
		return -ENOMEM;
	}

	for_each_child_of_node(parent_node, child_node) {

		if (i >= clock_freq_tbl->count) {
			dprintk(VIDC_ERR,
				"qcom,clock-freq-tbl: invalid child node %d, max is %d\n",
				i, clock_freq_tbl->count);
			break;
		}

		entry = &clock_freq_tbl->clk_prof_entries[i];
		dprintk(VIDC_DBG, "qcom,clock-freq-tbl: profile[%d]\n", i);

		if (of_find_property(child_node, "qcom,codec-mask", NULL)) {
			rc = of_property_read_u32(child_node,
					"qcom,codec-mask", &entry->codec_mask);
			if (rc) {
				dprintk(VIDC_ERR,
					"qcom,codec-mask not found\n");
				goto error;
			}
		} else {
			entry->codec_mask = 0;
		}
		dprintk(VIDC_DBG, "codec_mask %#x\n", entry->codec_mask);

		if (of_find_property(child_node, "qcom,cycles-per-mb", NULL)) {
			rc = of_property_read_u32(child_node,
					"qcom,cycles-per-mb", &entry->cycles);
			if (rc) {
				dprintk(VIDC_ERR,
					"qcom,cycles-per-mb not found\n");
				goto error;
			}
		} else {
			entry->cycles = 0;
		}
		dprintk(VIDC_DBG, "cycles_per_mb %d\n", entry->cycles);

		if (of_find_property(child_node,
				"qcom,low-power-mode-factor", NULL)) {
			rc = of_property_read_u32(child_node,
					"qcom,low-power-mode-factor",
					&entry->low_power_factor);
			if (rc) {
				dprintk(VIDC_ERR,
					"qcom,low-power-mode-factor not found\n");
				goto error;
			}
		} else {
			entry->low_power_factor = 0;
		}
		dprintk(VIDC_DBG, "low_power_factor %d\n",
				entry->low_power_factor);

		i++;
	}

error:
	return rc;
}

>>>>>>> 0e91d2a... Nougat
static int msm_vidc_load_freq_table(struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	int num_elements = 0;
	struct platform_device *pdev = res->pdev;

	if (!of_find_property(pdev->dev.of_node, "qcom,load-freq-tbl", NULL)) {
		/* qcom,load-freq-tbl is an optional property.  It likely won't
		 * be present on cores that we can't clock scale on. */
		dprintk(VIDC_DBG, "qcom,load-freq-tbl not found\n");
		return 0;
	}

	num_elements = get_u32_array_num_elements(pdev, "qcom,load-freq-tbl");
	num_elements /= sizeof(*res->load_freq_tbl) / sizeof(u32);
	if (num_elements == 0) {
		dprintk(VIDC_ERR, "no elements in frequency table\n");
		return rc;
	}

	res->load_freq_tbl = devm_kzalloc(&pdev->dev, num_elements *
			sizeof(*res->load_freq_tbl), GFP_KERNEL);
	if (!res->load_freq_tbl) {
		dprintk(VIDC_ERR,
				"%s Failed to alloc load_freq_tbl\n",
				__func__);
		return -ENOMEM;
	}

	if (of_property_read_u32_array(pdev->dev.of_node,
		"qcom,load-freq-tbl", (u32 *)res->load_freq_tbl,
		num_elements * sizeof(*res->load_freq_tbl) / sizeof(u32))) {
		dprintk(VIDC_ERR, "Failed to read frequency table\n");
		msm_vidc_free_freq_table(res);
		return -EINVAL;
	}

	res->load_freq_tbl_size = num_elements;
	return rc;
}

static int msm_vidc_load_bus_vectors(struct msm_vidc_platform_resources *res)
{
	struct platform_device *pdev = res->pdev;
	struct device_node *child_node, *bus_node;
	struct bus_set *buses = &res->bus_set;
	int rc = 0, c = 0;
	u32 num_buses = 0;

	bus_node = of_find_node_by_name(pdev->dev.of_node,
			"qcom,msm-bus-clients");
	if (!bus_node) {
		/* Not a required property */
		dprintk(VIDC_DBG, "qcom,msm-bus-clients not found\n");
		rc = 0;
		goto err_bad_node;
	}

	for_each_child_of_node(bus_node, child_node)
		++num_buses;

	buses->bus_tbl = devm_kzalloc(&pdev->dev, sizeof(*buses->bus_tbl) *
			num_buses, GFP_KERNEL);
	if (!buses->bus_tbl) {
		dprintk(VIDC_ERR, "%s: Failed to allocate memory\n", __func__);
		rc = -ENOMEM;
		goto err_bad_node;
	}

	buses->count = num_buses;
	c = 0;

	for_each_child_of_node(bus_node, child_node) {
		bool passive = false;
		bool low_power = false;
		u32 configs = 0;
		struct bus_info *bus = &buses->bus_tbl[c];

		passive = of_property_read_bool(child_node, "qcom,bus-passive");
		low_power = of_property_read_bool(child_node,
			"qcom,bus-low-power");
		rc = of_property_read_u32(child_node, "qcom,bus-configs",
				&configs);
		if (rc) {
			dprintk(VIDC_ERR,
					"Failed to read qcom,bus-configs in %s: %d\n",
					child_node->name, rc);
			break;
		}

		bus->passive = passive;
		bus->low_power = low_power;
		bus->sessions_supported = configs;
		bus->pdata = msm_bus_pdata_from_node(pdev, child_node);
		if (IS_ERR_OR_NULL(bus->pdata)) {
			rc = PTR_ERR(bus->pdata) ?: -EBADHANDLE;
			dprintk(VIDC_ERR, "Failed to get bus pdata: %d\n", rc);
			break;
		}

		dprintk(VIDC_DBG, "Bus %s supports: %x, passive: %d\n",
				bus->pdata->name, bus->sessions_supported,
				passive);
		++c;
	}

	if (c < num_buses) {
		for (c--; c >= 0; c--)
			msm_bus_cl_clear_pdata(buses->bus_tbl[c].pdata);

		goto err_bad_node;
	}

err_bad_node:
	return rc;
}

static int msm_vidc_load_iommu_groups(struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct platform_device *pdev = res->pdev;
	struct device_node *domains_parent_node = NULL;
	struct device_node *domains_child_node = NULL;
	struct iommu_set *iommu_group_set = &res->iommu_group_set;
	int domain_idx = 0;
	struct iommu_info *iommu_map;
	int array_size = 0;

	domains_parent_node = of_find_node_by_name(pdev->dev.of_node,
				"qcom,vidc-iommu-domains");
	if (!domains_parent_node) {
		dprintk(VIDC_DBG, "Node qcom,vidc-iommu-domains not found.\n");
		return 0;
	}

	iommu_group_set->count = 0;
	for_each_child_of_node(domains_parent_node, domains_child_node) {
		iommu_group_set->count++;
	}

	if (iommu_group_set->count == 0) {
		dprintk(VIDC_ERR, "No group present in iommu_domains\n");
		rc = -ENOENT;
		goto err_no_of_node;
	}
	iommu_group_set->iommu_maps = devm_kzalloc(&pdev->dev,
			iommu_group_set->count *
			sizeof(*iommu_group_set->iommu_maps), GFP_KERNEL);

	if (!iommu_group_set->iommu_maps) {
		dprintk(VIDC_ERR, "Cannot allocate iommu_maps\n");
		rc = -ENOMEM;
		goto err_no_of_node;
	}

	/* set up each context bank */
	for_each_child_of_node(domains_parent_node, domains_child_node) {
		struct device_node *ctx_node = of_parse_phandle(
						domains_child_node,
						"qcom,vidc-domain-phandle",
						0);
		if (domain_idx >= iommu_group_set->count)
			break;

		iommu_map = &iommu_group_set->iommu_maps[domain_idx];
		if (!ctx_node) {
			dprintk(VIDC_ERR, "Unable to parse pHandle\n");
			rc = -EBADHANDLE;
			goto err_load_groups;
		}

		/* domain info from domains.dtsi */
		rc = of_property_read_string(ctx_node, "label",
				&(iommu_map->name));
		if (rc) {
			dprintk(VIDC_ERR, "Could not find label property\n");
			goto err_load_groups;
		}

		dprintk(VIDC_DBG,
				"domain %d has name %s\n",
				domain_idx,
				iommu_map->name);

		if (!of_get_property(ctx_node, "qcom,virtual-addr-pool",
				&array_size)) {
			dprintk(VIDC_ERR,
				"Could not find any addr pool for group : %s\n",
				iommu_map->name);
			rc = -EBADHANDLE;
			goto err_load_groups;
		}

		iommu_map->npartitions = array_size / sizeof(u32) / 2;

		dprintk(VIDC_DBG,
				"%d partitions in domain %d",
				iommu_map->npartitions,
				domain_idx);

		rc = of_property_read_u32_array(ctx_node,
				"qcom,virtual-addr-pool",
				(u32 *)iommu_map->addr_range,
				iommu_map->npartitions * 2);
		if (rc) {
			dprintk(VIDC_ERR,
				"Could not read addr pool for group : %s (%d)\n",
				iommu_map->name,
				rc);
			goto err_load_groups;
		}

		iommu_map->is_secure =
			of_property_read_bool(ctx_node,	"qcom,secure-domain");

		dprintk(VIDC_DBG,
				"domain %s : secure = %d\n",
				iommu_map->name,
				iommu_map->is_secure);

		/* setup partitions and buffer type per partition */
		rc = of_property_read_u32_array(domains_child_node,
				"qcom,vidc-partition-buffer-types",
				iommu_map->buffer_type,
				iommu_map->npartitions);

		if (rc) {
			dprintk(VIDC_ERR,
					"cannot load partition buffertype information (%d)\n",
					rc);
			rc = -ENOENT;
			goto err_load_groups;
		}
		domain_idx++;
	}
	return rc;
err_load_groups:
	msm_vidc_free_iommu_groups(res);
err_no_of_node:
	return rc;
}

static int msm_vidc_load_regulator_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct platform_device *pdev = res->pdev;
	struct regulator_set *regulators = &res->regulator_set;
	struct device_node *domains_parent_node = NULL;
	struct property *domains_property = NULL;
	int reg_count = 0;

	regulators->count = 0;
	regulators->regulator_tbl = NULL;

	domains_parent_node = pdev->dev.of_node;
	for_each_property_of_node(domains_parent_node, domains_property) {
		const char *search_string = "-supply";
		char *supply;
		bool matched = false;

		/* check if current property is possibly a regulator */
		supply = strnstr(domains_property->name, search_string,
				strlen(domains_property->name) + 1);
		matched = supply && (*(supply + strlen(search_string)) == '\0');
		if (!matched)
			continue;

		reg_count++;
	}

	regulators->regulator_tbl = devm_kzalloc(&pdev->dev,
			sizeof(*regulators->regulator_tbl) *
			reg_count, GFP_KERNEL);

	if (!regulators->regulator_tbl) {
		rc = -ENOMEM;
		dprintk(VIDC_ERR,
			"Failed to alloc memory for regulator table\n");
		goto err_reg_tbl_alloc;
	}

	for_each_property_of_node(domains_parent_node, domains_property) {
		const char *search_string = "-supply";
		char *supply;
		bool matched = false;
		struct device_node *regulator_node = NULL;
		struct regulator_info *rinfo = NULL;

		/* check if current property is possibly a regulator */
		supply = strnstr(domains_property->name, search_string,
				strlen(domains_property->name) + 1);
		matched = supply && (supply[strlen(search_string)] == '\0');
		if (!matched)
			continue;

		/* make sure prop isn't being misused */
		regulator_node = of_parse_phandle(domains_parent_node,
				domains_property->name, 0);
		if (IS_ERR(regulator_node)) {
			dprintk(VIDC_WARN, "%s is not a phandle\n",
					domains_property->name);
			continue;
		}
		regulators->count++;

		/* populate regulator info */
		rinfo = &regulators->regulator_tbl[regulators->count - 1];
		rinfo->name = devm_kzalloc(&pdev->dev,
			(supply - domains_property->name) + 1, GFP_KERNEL);
		if (!rinfo->name) {
			rc = -ENOMEM;
			dprintk(VIDC_ERR,
					"Failed to alloc memory for regulator name\n");
			goto err_reg_name_alloc;
		}
		strlcpy(rinfo->name, domains_property->name,
			(supply - domains_property->name) + 1);

		rinfo->has_hw_power_collapse = of_property_read_bool(
			regulator_node, "qcom,support-hw-trigger");

		dprintk(VIDC_DBG, "Found regulator %s: h/w collapse = %s\n",
				rinfo->name,
				rinfo->has_hw_power_collapse ? "yes" : "no");
	}

	if (!regulators->count)
		dprintk(VIDC_DBG, "No regulators found");

	return 0;

err_reg_name_alloc:
err_reg_tbl_alloc:
	msm_vidc_free_regulator_table(res);
	return rc;
}

static int msm_vidc_load_clock_table(
		struct msm_vidc_platform_resources *res)
{
	int rc = 0, num_clocks = 0, c = 0;
	struct platform_device *pdev = res->pdev;
	int *clock_props = NULL;
	struct clock_set *clocks = &res->clock_set;

	num_clocks = of_property_count_strings(pdev->dev.of_node,
				"clock-names");
	if (num_clocks <= 0) {
		/* Devices such as Q6 might not have any control over clocks
		 * hence have none specified, which is ok. */
		dprintk(VIDC_DBG, "No clocks found\n");
		clocks->count = 0;
		rc = 0;
		goto err_load_clk_table_fail;
	}

	clock_props = devm_kzalloc(&pdev->dev, num_clocks *
			sizeof(*clock_props), GFP_KERNEL);
	if (!clock_props) {
		dprintk(VIDC_ERR, "No memory to read clock properties\n");
		rc = -ENOMEM;
		goto err_load_clk_table_fail;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node,
				"qcom,clock-configs", clock_props,
				num_clocks);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read clock properties: %d\n", rc);
		goto err_load_clk_prop_fail;
	}

	clocks->clock_tbl = devm_kzalloc(&pdev->dev, sizeof(*clocks->clock_tbl)
			* num_clocks, GFP_KERNEL);
	if (!clocks->clock_tbl) {
		dprintk(VIDC_ERR, "Failed to allocate memory for clock tbl\n");
		rc = -ENOMEM;
		goto err_load_clk_prop_fail;
	}

	clocks->count = num_clocks;
	dprintk(VIDC_DBG, "Found %d clocks\n", num_clocks);

	for (c = 0; c < num_clocks; ++c) {
		struct clock_info *vc = &res->clock_set.clock_tbl[c];

		of_property_read_string_index(pdev->dev.of_node,
				"clock-names", c, &vc->name);

		if (clock_props[c] & CLOCK_PROP_HAS_SCALING) {
			vc->count = res->load_freq_tbl_size;
			vc->load_freq_tbl = res->load_freq_tbl;
		} else {
			vc->count = 0;
			vc->load_freq_tbl = NULL;
		}

		vc->has_gating = !!(clock_props[c] & CLOCK_PROP_HAS_GATING);

		dprintk(VIDC_DBG,
			"Found clock %s: scale-able = %s, gate-able = %s\n",
			vc->name, vc->count ? "yes" : "no",
			vc->has_gating ? "yes" : "no");
	}

	res->sw_power_collapsible = of_property_read_bool(pdev->dev.of_node,
					"qcom,sw-power-collapse");
	dprintk(VIDC_DBG, "Power collapse supported = %s\n",
		res->sw_power_collapsible ? "yes" : "no");

	res->early_fw_load = of_property_read_bool(pdev->dev.of_node,
				"qcom,early-fw-load");
	dprintk(VIDC_DBG, "Early fw load = %s\n",
				res->early_fw_load ? "yes" : "no");

	return 0;

err_load_clk_prop_fail:
err_load_clk_table_fail:
	return rc;
}

int read_platform_resources_from_dt(
		struct msm_vidc_platform_resources *res)
{
	struct platform_device *pdev = res->pdev;
	struct resource *kres = NULL;
	int rc = 0;
	uint32_t firmware_base = 0;

	if (!pdev->dev.of_node) {
		dprintk(VIDC_ERR, "DT node not found\n");
		return -ENOENT;
	}

	res->firmware_base = (phys_addr_t)firmware_base;

	kres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res->register_base = kres ? kres->start : -1;
	res->register_size = kres ? (kres->end + 1 - kres->start) : -1;

	kres = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	res->irq = kres ? kres->start : -1;

	of_property_read_u32(pdev->dev.of_node,
			"qcom,ocmem-size", &res->ocmem_size);

	res->dynamic_bw_update = of_property_read_bool(pdev->dev.of_node,
			"qcom,use-dynamic-bw-update");
	res->sys_idle_indicator = of_property_read_bool(pdev->dev.of_node,
			"qcom,enable-idle-indicator");

	res->thermal_mitigable =
			of_property_read_bool(pdev->dev.of_node,
			"qcom,enable-thermal-mitigation");

<<<<<<< HEAD
=======
	rc = of_property_read_string(pdev->dev.of_node, "qcom,firmware-name",
			&res->fw_name);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read firmware name: %d\n", rc);
		goto err_load_freq_table;
	}
	dprintk(VIDC_DBG, "Firmware filename: %s\n", res->fw_name);

	rc = of_property_read_string(pdev->dev.of_node, "qcom,hfi-version",
			&res->hfi_version);
	if (rc)
		dprintk(VIDC_DBG, "HFI packetization will default to legacy\n");

	rc = msm_vidc_load_platform_version_table(res);
	if (rc)
		dprintk(VIDC_ERR, "Failed to load pf version table: %d\n", rc);

>>>>>>> 0e91d2a... Nougat
	rc = msm_vidc_load_freq_table(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load freq table: %d\n", rc);
		goto err_load_freq_table;
	}

	rc = msm_vidc_load_qdss_table(res);
	if (rc)
		dprintk(VIDC_WARN, "Failed to load qdss reg table: %d\n", rc);

	rc = msm_vidc_load_reg_table(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load reg table: %d\n", rc);
		goto err_load_reg_table;
	}

	rc = msm_vidc_load_bus_vectors(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load bus vectors: %d\n", rc);
		goto err_load_bus_vectors;
	}
	rc = msm_vidc_load_iommu_groups(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load iommu groups: %d\n", rc);
		goto err_load_iommu_groups;
	}

	rc = msm_vidc_load_regulator_table(res);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to load list of regulators %d\n", rc);
		goto err_load_regulator_table;
	}

	rc = msm_vidc_load_clock_table(res);
	if (rc) {
		dprintk(VIDC_ERR,
			"Failed to load clock table: %d\n", rc);
		goto err_load_clock_table;
	}

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,max-hw-load",
			&res->max_load);
	if (rc) {
		dprintk(VIDC_ERR,
			"Failed to determine max load supported: %d\n", rc);
		goto err_load_max_hw_load;
	}

	res->use_non_secure_pil = of_property_read_bool(pdev->dev.of_node,
			"qcom,use-non-secure-pil");

	if (res->use_non_secure_pil || !is_iommu_present(res)) {
		of_property_read_u32(pdev->dev.of_node, "qcom,fw-bias",
				&firmware_base);
		res->firmware_base = (phys_addr_t)firmware_base;
		dprintk(VIDC_DBG,
				"Using fw-bias : %pa", &res->firmware_base);
	}
	return rc;
err_load_max_hw_load:
	msm_vidc_free_clock_table(res);
err_load_clock_table:
	msm_vidc_free_regulator_table(res);
err_load_regulator_table:
	msm_vidc_free_iommu_groups(res);
err_load_iommu_groups:
	msm_vidc_free_bus_vectors(res);
err_load_bus_vectors:
	msm_vidc_free_reg_table(res);
err_load_reg_table:
	msm_vidc_free_freq_table(res);
err_load_freq_table:
	return rc;
}
<<<<<<< HEAD
=======

static int get_secure_vmid(struct context_bank_info *cb)
{
	if (!strcasecmp(cb->name, "venus_sec_bitstream"))
		return VMID_CP_BITSTREAM;
	else if (!strcasecmp(cb->name, "venus_sec_pixel"))
		return VMID_CP_PIXEL;
	else if (!strcasecmp(cb->name, "venus_sec_non_pixel"))
		return VMID_CP_NON_PIXEL;
	else {
		WARN(1, "No matching secure vmid for cb name: %s\n",
			cb->name);
		return VMID_INVAL;
	}
}

static int msm_vidc_setup_context_bank(struct context_bank_info *cb,
		struct device *dev)
{
	int rc = 0;
	int disable_htw = 1;
	int secure_vmid = VMID_INVAL;
	struct bus_type *bus;

	if (!dev || !cb) {
		dprintk(VIDC_ERR,
			"%s: Invalid Input params\n", __func__);
		return -EINVAL;
	}
	cb->dev = dev;

	bus = msm_iommu_get_bus(cb->dev);
	if (IS_ERR_OR_NULL(bus)) {
		dprintk(VIDC_ERR, "%s - failed to get bus type\n", __func__);
		rc = PTR_ERR(bus) ?: -ENODEV;
		goto remove_cb;
	}

	cb->mapping = arm_iommu_create_mapping(bus, cb->addr_range.start,
					cb->addr_range.size);
	if (IS_ERR_OR_NULL(cb->mapping)) {
		dprintk(VIDC_ERR, "%s - failed to create mapping\n", __func__);
		rc = PTR_ERR(cb->mapping) ?: -ENODEV;
		goto remove_cb;
	}

	rc = iommu_domain_set_attr(cb->mapping->domain,
			DOMAIN_ATTR_COHERENT_HTW_DISABLE, &disable_htw);
	if (rc) {
		dprintk(VIDC_ERR, "%s - disable coherent HTW failed: %s %d\n",
				__func__, dev_name(dev), rc);
		goto release_mapping;
	}

	if (cb->is_secure) {
		secure_vmid = get_secure_vmid(cb);
		rc = iommu_domain_set_attr(cb->mapping->domain,
				DOMAIN_ATTR_SECURE_VMID, &secure_vmid);
		if (rc) {
			dprintk(VIDC_ERR,
					"%s - programming secure vmid failed: %s %d\n",
					__func__, dev_name(dev), rc);
			goto release_mapping;
		}
	}

	rc = arm_iommu_attach_device(cb->dev, cb->mapping);
	if (rc) {
		dprintk(VIDC_ERR, "%s - Couldn't arm_iommu_attach_device\n",
			__func__);
		goto release_mapping;
	}

	dprintk(VIDC_DBG, "Attached %s and created mapping\n", dev_name(dev));
	dprintk(VIDC_DBG,
		"Context bank name:%s, buffer_type: %#x, is_secure: %d, address range start: %#x, size: %#x, dev: %pK, mapping: %pK",
		cb->name, cb->buffer_type, cb->is_secure, cb->addr_range.start,
		cb->addr_range.size, cb->dev, cb->mapping);

	return rc;

release_mapping:
	arm_iommu_release_mapping(cb->mapping);
remove_cb:
	return rc;
}

int msm_vidc_smmu_fault_handler(struct iommu_domain *domain,
		struct device *dev, unsigned long iova, int flags, void *token)
{
	struct msm_vidc_core *core = token;
	struct msm_vidc_inst *inst;
	struct buffer_info *temp;
	struct internal_buf *buf;
	int i = 0;
	bool is_decode = false;
	enum vidc_ports port;

	if (!domain || !core) {
		dprintk(VIDC_ERR, "%s - invalid param %pK %pK\n",
			__func__, domain, core);
		return -EINVAL;
	}

	if (core->smmu_fault_handled)
		return -ENOSYS;

	dprintk(VIDC_ERR, "%s - faulting address: %lx\n", __func__, iova);

	mutex_lock(&core->lock);
	list_for_each_entry(inst, &core->instances, list) {
		is_decode = inst->session_type == MSM_VIDC_DECODER;
		port = is_decode ? OUTPUT_PORT : CAPTURE_PORT;
		dprintk(VIDC_ERR,
			"%s session, Codec type: %s HxW: %d x %d fps: %d bitrate: %d bit-depth: %s\n",
			is_decode ? "Decode" : "Encode", inst->fmts[port]->name,
			inst->prop.height[port], inst->prop.width[port],
			inst->prop.fps, inst->prop.bitrate,
			!inst->bit_depth ? "8" : "10");

		dprintk(VIDC_ERR,
			"---Buffer details for inst: %pK of type: %d---\n",
			inst, inst->session_type);
		mutex_lock(&inst->registeredbufs.lock);
		dprintk(VIDC_ERR, "registered buffer list:\n");
		list_for_each_entry(temp, &inst->registeredbufs.list, list)
			for (i = 0; i < temp->num_planes; i++)
				dprintk(VIDC_ERR,
					"type: %d plane: %d addr: %pa size: %d\n",
					temp->type, i, &temp->device_addr[i],
					temp->size[i]);

		mutex_unlock(&inst->registeredbufs.lock);

		mutex_lock(&inst->scratchbufs.lock);
		dprintk(VIDC_ERR, "scratch buffer list:\n");
		list_for_each_entry(buf, &inst->scratchbufs.list, list)
			dprintk(VIDC_ERR, "type: %d addr: %pa size: %zu\n",
				buf->buffer_type, &buf->handle->device_addr,
				buf->handle->size);
		mutex_unlock(&inst->scratchbufs.lock);

		mutex_lock(&inst->persistbufs.lock);
		dprintk(VIDC_ERR, "persist buffer list:\n");
		list_for_each_entry(buf, &inst->persistbufs.list, list)
			dprintk(VIDC_ERR, "type: %d addr: %pa size: %zu\n",
				buf->buffer_type, &buf->handle->device_addr,
				buf->handle->size);
		mutex_unlock(&inst->persistbufs.lock);

		mutex_lock(&inst->outputbufs.lock);
		dprintk(VIDC_ERR, "dpb buffer list:\n");
		list_for_each_entry(buf, &inst->outputbufs.list, list)
			dprintk(VIDC_ERR, "type: %d addr: %pa size: %zu\n",
				buf->buffer_type, &buf->handle->device_addr,
				buf->handle->size);
		mutex_unlock(&inst->outputbufs.lock);
	}
	core->smmu_fault_handled = true;
	mutex_unlock(&core->lock);
	/*
	 * Return -ENOSYS to elicit the default behaviour of smmu driver.
	 * If we return -ENOSYS, then smmu driver assumes page fault handler
	 * is not installed and prints a list of useful debug information like
	 * FAR, SID etc. This information is not printed if we return 0.
	 */
	return -ENOSYS;
}

static int msm_vidc_populate_context_bank(struct device *dev,
		struct msm_vidc_core *core)
{
	int rc = 0;
	struct context_bank_info *cb = NULL;
	struct device_node *np = NULL;

	if (!dev || !core) {
		dprintk(VIDC_ERR, "%s - invalid inputs\n", __func__);
		return -EINVAL;
	}

	np = dev->of_node;
	cb = devm_kzalloc(dev, sizeof(*cb), GFP_KERNEL);
	if (!cb) {
		dprintk(VIDC_ERR, "%s - Failed to allocate cb\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&cb->list);
	list_add_tail(&cb->list, &core->resources.context_banks);

	rc = of_property_read_string(np, "label", &cb->name);
	if (rc) {
		dprintk(VIDC_DBG,
			"Failed to read cb label from device tree\n");
		rc = 0;
	}

	dprintk(VIDC_DBG, "%s: context bank has name %s\n", __func__, cb->name);
	rc = of_property_read_u32_array(np, "virtual-addr-pool",
			(u32 *)&cb->addr_range, 2);
	if (rc) {
		dprintk(VIDC_ERR,
			"Could not read addr pool for context bank : %s %d\n",
			cb->name, rc);
		goto err_setup_cb;
	}

	cb->is_secure = of_property_read_bool(np, "qcom,secure-context-bank");
	dprintk(VIDC_DBG, "context bank %s : secure = %d\n",
			cb->name, cb->is_secure);

	/* setup buffer type for each sub device*/
	rc = of_property_read_u32(np, "buffer-types", &cb->buffer_type);
	if (rc) {
		dprintk(VIDC_ERR, "failed to load buffer_type info %d\n", rc);
		rc = -ENOENT;
		goto err_setup_cb;
	}
	dprintk(VIDC_DBG,
		"context bank %s address start = %x address size = %x buffer_type = %x\n",
		cb->name, cb->addr_range.start,
		cb->addr_range.size, cb->buffer_type);

	rc = msm_vidc_setup_context_bank(cb, dev);
	if (rc) {
		dprintk(VIDC_ERR, "Cannot setup context bank %d\n", rc);
		goto err_setup_cb;
	}

	iommu_set_fault_handler(cb->mapping->domain,
		msm_vidc_smmu_fault_handler, (void *)core);

	return 0;

err_setup_cb:
	list_del(&cb->list);
	return rc;
}

static int msm_vidc_populate_legacy_context_bank(
			struct msm_vidc_platform_resources *res)
{
	int rc = 0;
	struct platform_device *pdev = NULL;
	struct device_node *domains_parent_node = NULL;
	struct device_node *domains_child_node = NULL;
	struct device_node *ctx_node = NULL;
	struct context_bank_info *cb;

	if (!res || !res->pdev) {
		dprintk(VIDC_ERR, "%s - invalid inputs\n", __func__);
		return -EINVAL;
	}
	pdev = res->pdev;

	domains_parent_node = of_find_node_by_name(pdev->dev.of_node,
			"qcom,vidc-iommu-domains");
	if (!domains_parent_node) {
		dprintk(VIDC_DBG,
			"%s legacy iommu domains not present\n", __func__);
		return 0;
	}

	/* set up each context bank for legacy DT bindings*/
	for_each_child_of_node(domains_parent_node,
		domains_child_node) {
		cb = devm_kzalloc(&pdev->dev, sizeof(*cb), GFP_KERNEL);
		if (!cb) {
			dprintk(VIDC_ERR,
				"%s - Failed to allocate cb\n", __func__);
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&cb->list);
		list_add_tail(&cb->list, &res->context_banks);

		ctx_node = of_parse_phandle(domains_child_node,
				"qcom,vidc-domain-phandle", 0);
		if (!ctx_node) {
			dprintk(VIDC_ERR,
				"%s Unable to parse pHandle\n", __func__);
			rc = -EBADHANDLE;
			goto err_setup_cb;
		}

		rc = of_property_read_string(ctx_node, "label", &(cb->name));
		if (rc) {
			dprintk(VIDC_ERR,
				"%s Could not find label\n", __func__);
			goto err_setup_cb;
		}

		rc = of_property_read_u32_array(ctx_node,
			"qcom,virtual-addr-pool", (u32 *)&cb->addr_range, 2);
		if (rc) {
			dprintk(VIDC_ERR,
				"%s Could not read addr pool for group : %s (%d)\n",
				__func__, cb->name, rc);
			goto err_setup_cb;
		}

		cb->is_secure =
			of_property_read_bool(ctx_node, "qcom,secure-domain");

		rc = of_property_read_u32(domains_child_node,
				"qcom,vidc-buffer-types", &cb->buffer_type);
		if (rc) {
			dprintk(VIDC_ERR,
				"%s Could not read buffer type (%d)\n",
				__func__, rc);
			goto err_setup_cb;
		}

		cb->dev = msm_iommu_get_ctx(cb->name);
		if (IS_ERR_OR_NULL(cb->dev)) {
			dprintk(VIDC_ERR, "%s could not get device for cb %s\n",
					__func__, cb->name);
			rc = -ENOENT;
			goto err_setup_cb;
		}

		rc = msm_vidc_setup_context_bank(cb, cb->dev);
		if (rc) {
			dprintk(VIDC_ERR, "Cannot setup context bank %d\n", rc);
			goto err_setup_cb;
		}
		dprintk(VIDC_DBG,
			"%s: context bank %s secure %d addr start = %#x addr size = %#x buffer_type = %#x\n",
			__func__, cb->name, cb->is_secure, cb->addr_range.start,
			cb->addr_range.size, cb->buffer_type);
	}
	return rc;

err_setup_cb:
	list_del(&cb->list);
	return rc;
}

int read_context_bank_resources_from_dt(struct platform_device *pdev)
{
	struct msm_vidc_core *core;
	int rc = 0;

	if (!pdev) {
		dprintk(VIDC_ERR, "Invalid platform device\n");
		return -EINVAL;
	} else if (!pdev->dev.parent) {
		dprintk(VIDC_ERR, "Failed to find a parent for %s\n",
				dev_name(&pdev->dev));
		return -ENODEV;
	}

	core = dev_get_drvdata(pdev->dev.parent);
	if (!core) {
		dprintk(VIDC_ERR, "Failed to find cookie in parent device %s",
				dev_name(pdev->dev.parent));
		return -EINVAL;
	}

	if (of_property_read_bool(pdev->dev.of_node, "qcom,fw-context-bank")) {
		if (core->resources.use_non_secure_pil) {
			struct context_bank_info *cb;

			cb = devm_kzalloc(&pdev->dev, sizeof(*cb), GFP_KERNEL);
			if (!cb) {
				dprintk(VIDC_ERR, "alloc venus cb failed\n");
				return -ENOMEM;
			}

			cb->dev = &pdev->dev;
			rc = venus_boot_init(&core->resources, cb);
			if (rc) {
				dprintk(VIDC_ERR,
				"Failed to init non-secure PIL %d\n", rc);
			}
		}
	} else {
		rc = msm_vidc_populate_context_bank(&pdev->dev, core);
		if (rc)
			dprintk(VIDC_ERR, "Failed to probe context bank\n");
		else
			dprintk(VIDC_DBG, "Successfully probed context bank\n");
	}
	return rc;
}

int read_bus_resources_from_dt(struct platform_device *pdev)
{
	struct msm_vidc_core *core;

	if (!pdev) {
		dprintk(VIDC_ERR, "Invalid platform device\n");
		return -EINVAL;
	} else if (!pdev->dev.parent) {
		dprintk(VIDC_ERR, "Failed to find a parent for %s\n",
				dev_name(&pdev->dev));
		return -ENODEV;
	}

	core = dev_get_drvdata(pdev->dev.parent);
	if (!core) {
		dprintk(VIDC_ERR, "Failed to find cookie in parent device %s",
				dev_name(pdev->dev.parent));
		return -EINVAL;
	}

	return msm_vidc_populate_bus(&pdev->dev, &core->resources);
}
>>>>>>> 0e91d2a... Nougat
