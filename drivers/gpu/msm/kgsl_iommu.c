/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
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
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/msm_kgsl.h>
<<<<<<< HEAD
#include <linux/msm_iommu_domains.h>
=======
#include <linux/ratelimit.h>
#include <linux/of_platform.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/secure_buffer.h>
>>>>>>> 0e91d2a... Nougat
#include <stddef.h>

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_mmu.h"
#include "kgsl_sharedmem.h"
#include "kgsl_iommu.h"
#include "adreno_pm4types.h"
#include "adreno.h"
#include "kgsl_trace.h"
#include "kgsl_cffdump.h"
#include "kgsl_pwrctrl.h"

<<<<<<< HEAD
static struct kgsl_iommu_register_list kgsl_iommuv0_reg[KGSL_IOMMU_REG_MAX] = {
	{ 0, 0 },			
	{ 0x0, 1 },			
	{ 0x10, 1 },			
	{ 0x14, 1 },			
	{ 0x20, 1 },			
	{ 0x28, 1 },			
	{ 0x800, 1 },			
	{ 0x820, 1 },			
	{ 0x03C, 1 },			
	{ 0x818, 1 },			
	{ 0x2C, 1 },			
	{ 0x30, 1 },			
	{ 0, 0 },			
	{ 0, 0 },			
	{ 0, 0 }			
};

static struct kgsl_iommu_register_list kgsl_iommuv1_reg[KGSL_IOMMU_REG_MAX] = {
	{ 0, 0 },			
	{ 0x0, 1 },			
	{ 0x20, 1 },			
	{ 0x28, 1 },			
	{ 0x58, 1 },			
	{ 0x60, 1 },			
	{ 0x618, 1 },			
	{ 0x008, 1 },			
	{ 0, 0 },			
	{ 0, 0 },			
	{ 0x68, 1 },			
	{ 0x6C, 1 },			
	{ 0x7F0, 1 },			
	{ 0x7F4, 1 },			
	{ 0x2000, 0 }			
};

static struct iommu_access_ops *iommu_access_ops;
=======
#define _IOMMU_PRIV(_mmu) (&((_mmu)->priv.iommu))

static struct kgsl_mmu_pt_ops iommu_pt_ops;
static bool need_iommu_sync;

const unsigned int kgsl_iommu_reg_list[KGSL_IOMMU_REG_MAX] = {
	0x0,
	0x20,
	0x34,
	0x58,
	0x60,
	0x618,
	0x008,
	0x68,
	0x6C,
	0x7F0,
	0x7F4,
};

struct kgsl_iommu_addr_entry {
	uint64_t base;
	uint64_t size;
	struct rb_node node;
};

static struct kmem_cache *addr_entry_cache;


#define GLOBAL_PT_ENTRIES 32

static struct kgsl_memdesc *global_pt_entries[GLOBAL_PT_ENTRIES];
static struct kgsl_memdesc *kgsl_global_secure_pt_entry;
static int global_pt_count;
uint64_t global_pt_alloc;
static struct kgsl_memdesc gpu_qdss_desc;

static void kgsl_iommu_unmap_globals(struct kgsl_pagetable *pagetable)
{
	unsigned int i;

	for (i = 0; i < global_pt_count; i++) {
		if (global_pt_entries[i] != NULL)
			kgsl_mmu_unmap(pagetable, global_pt_entries[i]);
	}
}

static void kgsl_iommu_map_globals(struct kgsl_pagetable *pagetable)
{
	unsigned int i;

	for (i = 0; i < global_pt_count; i++) {
		if (global_pt_entries[i] != NULL) {
			int ret = kgsl_mmu_map(pagetable, global_pt_entries[i]);

			BUG_ON(ret);
		}
	}
}

static void kgsl_iommu_unmap_global_secure_pt_entry(struct kgsl_pagetable
								*pagetable)
{
	struct kgsl_memdesc *entry = kgsl_global_secure_pt_entry;

	if (entry != NULL)
		kgsl_mmu_unmap(pagetable, entry);

}

static void kgsl_map_global_secure_pt_entry(struct kgsl_pagetable *pagetable)
{
	int ret;
	struct kgsl_memdesc *entry = kgsl_global_secure_pt_entry;

	if (entry != NULL) {
		entry->pagetable = pagetable;
		ret = kgsl_mmu_map(pagetable, entry);
		BUG_ON(ret);
	}
}

static void kgsl_iommu_remove_global(struct kgsl_mmu *mmu,
		struct kgsl_memdesc *memdesc)
{
	int i;

	if (memdesc->gpuaddr == 0 || !(memdesc->priv & KGSL_MEMDESC_GLOBAL))
		return;

	for (i = 0; i < global_pt_count; i++) {
		if (global_pt_entries[i] == memdesc) {
			memdesc->gpuaddr = 0;
			memdesc->priv &= ~KGSL_MEMDESC_GLOBAL;
			global_pt_entries[i] = NULL;
			return;
		}
	}
}

static void kgsl_iommu_add_global(struct kgsl_mmu *mmu,
		struct kgsl_memdesc *memdesc)
{
	if (memdesc->gpuaddr != 0)
		return;

	BUG_ON(global_pt_count >= GLOBAL_PT_ENTRIES);
	BUG_ON((global_pt_alloc + memdesc->size) >= KGSL_IOMMU_GLOBAL_MEM_SIZE);

	memdesc->gpuaddr = KGSL_IOMMU_GLOBAL_MEM_BASE + global_pt_alloc;
	memdesc->priv |= KGSL_MEMDESC_GLOBAL;
	global_pt_alloc += memdesc->size;

	global_pt_entries[global_pt_count++] = memdesc;
}

void kgsl_add_global_secure_entry(struct kgsl_device *device,
					struct kgsl_memdesc *memdesc)
{
	memdesc->gpuaddr = KGSL_IOMMU_SECURE_BASE;
	kgsl_global_secure_pt_entry = memdesc;
}

struct kgsl_memdesc *kgsl_iommu_get_qdss_global_entry(void)
{
	return &gpu_qdss_desc;
}

static void kgsl_setup_qdss_desc(struct kgsl_device *device)
{
	int result = 0;
	uint32_t gpu_qdss_entry[2];

	if (!of_find_property(device->pdev->dev.of_node,
		"qcom,gpu-qdss-stm", NULL))
		return;

	if (of_property_read_u32_array(device->pdev->dev.of_node,
				"qcom,gpu-qdss-stm", gpu_qdss_entry, 2)) {
		KGSL_CORE_ERR("Failed to read gpu qdss dts entry\n");
		return;
	}

	gpu_qdss_desc.flags = 0;
	gpu_qdss_desc.priv = 0;
	gpu_qdss_desc.physaddr = gpu_qdss_entry[0];
	gpu_qdss_desc.size = gpu_qdss_entry[1];
	gpu_qdss_desc.pagetable = NULL;
	gpu_qdss_desc.ops = NULL;
	gpu_qdss_desc.dev = device->dev->parent;
	gpu_qdss_desc.hostptr = NULL;

	result = memdesc_sg_dma(&gpu_qdss_desc, gpu_qdss_desc.physaddr,
			gpu_qdss_desc.size);
	if (result) {
		KGSL_CORE_ERR("memdesc_sg_dma failed: %d\n", result);
		return;
	}

	kgsl_mmu_add_global(device, &gpu_qdss_desc);
}

static inline void kgsl_cleanup_qdss_desc(struct kgsl_mmu *mmu)
{
	kgsl_iommu_remove_global(mmu, &gpu_qdss_desc);
	kgsl_sharedmem_free(&gpu_qdss_desc);
}


static inline void _iommu_sync_mmu_pc(bool lock)
{
	if (need_iommu_sync == false)
		return;
>>>>>>> 0e91d2a... Nougat

static int kgsl_iommu_flush_pt(struct kgsl_mmu *mmu);
static phys_addr_t
kgsl_iommu_get_current_ptbase(struct kgsl_mmu *mmu);

static void _iommu_lock(struct kgsl_iommu const *iommu)
{
	if (iommu_access_ops && iommu_access_ops->iommu_lock_acquire)
		iommu_access_ops->iommu_lock_acquire(
						iommu->sync_lock_initialized);
}

static void _iommu_unlock(struct kgsl_iommu const *iommu)
{
<<<<<<< HEAD
	if (iommu_access_ops && iommu_access_ops->iommu_lock_release)
		iommu_access_ops->iommu_lock_release(
						iommu->sync_lock_initialized);
}

struct remote_iommu_petersons_spinlock kgsl_iommu_sync_lock_vars;

=======
	int ret;

	if (iommu_pt->attached)
		return 0;

	_iommu_sync_mmu_pc(true);
	ret = iommu_attach_device(iommu_pt->domain, ctx->dev);
	_iommu_sync_mmu_pc(false);

	if (ret == 0)
		iommu_pt->attached = true;

	return ret;
}

static int _lock_if_secure_mmu(struct kgsl_memdesc *memdesc,
		struct kgsl_mmu *mmu)
{
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);

	if (!kgsl_memdesc_is_secured(memdesc))
		return 0;

	if (!kgsl_mmu_is_secured(mmu))
		return -EINVAL;

	mutex_lock(&device->mutex);
	if (kgsl_active_count_get(device)) {
		mutex_unlock(&device->mutex);
		return -EINVAL;
	}

	return 0;
}

static void _unlock_if_secure_mmu(struct kgsl_memdesc *memdesc,
		struct kgsl_mmu *mmu)
{
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);

	if (!kgsl_memdesc_is_secured(memdesc) || !kgsl_mmu_is_secured(mmu))
		return;
>>>>>>> 0e91d2a... Nougat

static struct page *kgsl_guard_page;
static struct kgsl_memdesc kgsl_secure_guard_page_memdesc;

static int get_iommu_unit(struct device *dev, struct kgsl_mmu **mmu_out,
			struct kgsl_iommu_unit **iommu_unit_out)
{
<<<<<<< HEAD
	int i, j, k;

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		struct kgsl_mmu *mmu;
		struct kgsl_iommu *iommu;
=======
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;

	_iommu_sync_mmu_pc(true);

	ret = iommu_map(iommu_pt->domain, gpuaddr, physaddr, size, flags);
>>>>>>> 0e91d2a... Nougat

		if (kgsl_driver.devp[i] == NULL)
			continue;

<<<<<<< HEAD
		mmu = kgsl_get_mmu(kgsl_driver.devp[i]);
		if (mmu == NULL || mmu->priv == NULL)
			continue;

		iommu = mmu->priv;

		for (j = 0; j < iommu->unit_count; j++) {
			struct kgsl_iommu_unit *iommu_unit =
				&iommu->iommu_units[j];
			for (k = 0; k < iommu_unit->dev_count; k++) {
				if (iommu_unit->dev[k].dev == dev) {
					*mmu_out = mmu;
					*iommu_unit_out = iommu_unit;
					return 0;
				}
			}
		}
=======
	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (ret) {
		KGSL_CORE_ERR("map err: 0x%016llX, 0x%llx, 0x%x, %d\n",
			gpuaddr, size, flags, ret);
		return -ENODEV;
>>>>>>> 0e91d2a... Nougat
	}

	return -EINVAL;
}

static struct kgsl_iommu_device *get_iommu_device(struct kgsl_iommu_unit *unit,
		struct device *dev)
{
<<<<<<< HEAD
	int k;

	for (k = 0; unit && k < unit->dev_count; k++) {
		if (unit->dev[k].dev == dev)
			return &(unit->dev[k]);
	}
=======
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	size_t unmapped = 0;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;

	_iommu_sync_mmu_pc(true);
>>>>>>> 0e91d2a... Nougat

	return NULL;
}


<<<<<<< HEAD

struct _mem_entry {
	unsigned int gpuaddr;
	unsigned int size;
	unsigned int flags;
	unsigned int priv;
	int pending_free;
	pid_t pid;
};
=======
	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (unmapped != size) {
		KGSL_CORE_ERR("unmap err: 0x%016llx, 0x%llx, %zd\n",
			addr, size, unmapped);
		return -ENODEV;
	}
>>>>>>> 0e91d2a... Nougat


<<<<<<< HEAD
static void _prev_entry(struct kgsl_process_private *priv,
	unsigned int faultaddr, struct _mem_entry *ret)
{
	struct rb_node *node;
	struct kgsl_mem_entry *entry;
=======
static int _iommu_map_sg_offset_sync_pc(struct kgsl_pagetable *pt,
		uint64_t addr, struct kgsl_memdesc *memdesc,
		struct scatterlist *sg, int nents,
		uint64_t offset, uint64_t size, unsigned int flags)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	uint64_t offset_tmp = offset;
	uint64_t size_tmp = size;
	size_t mapped = 0;
	unsigned int i;
	struct scatterlist *s;
	phys_addr_t physaddr;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;

	_iommu_sync_mmu_pc(true);

	for_each_sg(sg, s, nents, i) {
		
		if (offset_tmp >= s->length) {
			offset_tmp -= s->length;
			continue;
		}

		
		if (size < s->length - offset_tmp)
			size_tmp = size;
		else
			size_tmp = s->length - offset_tmp;

		
		if (offset_tmp != 0) {
			physaddr = page_to_phys(nth_page(sg_page(s),
					offset_tmp >> PAGE_SHIFT));
			
			offset_tmp = 0;
		} else
			physaddr = page_to_phys(sg_page(s));

		
		ret = iommu_map(iommu_pt->domain, addr + mapped,
				physaddr, size_tmp, flags);
		if (ret)
			break;

		mapped += size_tmp;
		size -= size_tmp;

		if (size == 0)
			break;
	}

	_iommu_sync_mmu_pc(false);

	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (size != 0) {
		
		_iommu_unmap_sync_pc(pt, memdesc, addr, mapped);
		KGSL_CORE_ERR("map err: 0x%016llX, %d, %x, %zd\n",
			addr, nents, flags, mapped);
		return  -ENODEV;
	}

	return 0;
}

static int _iommu_map_sg_sync_pc(struct kgsl_pagetable *pt,
		uint64_t addr, struct kgsl_memdesc *memdesc,
		struct scatterlist *sg, int nents,
		unsigned int flags)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	size_t mapped;
	int ret;

	ret = _lock_if_secure_mmu(memdesc, pt->mmu);
	if (ret)
		return ret;
>>>>>>> 0e91d2a... Nougat

	for (node = rb_first(&priv->mem_rb); node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);

<<<<<<< HEAD
		if (entry->memdesc.gpuaddr > faultaddr)
			break;
=======
	mapped = iommu_map_sg(iommu_pt->domain, addr, sg, nents, flags);
>>>>>>> 0e91d2a... Nougat


<<<<<<< HEAD
		if (entry->memdesc.gpuaddr > ret->gpuaddr) {
			ret->gpuaddr = entry->memdesc.gpuaddr;
			ret->size = entry->memdesc.size;
			ret->flags = entry->memdesc.flags;
			ret->priv = entry->memdesc.priv;
			ret->pending_free = entry->pending_free;
			ret->pid = priv->pid;
		}

		node = rb_next(&entry->node);
=======
	_unlock_if_secure_mmu(memdesc, pt->mmu);

	if (mapped == 0) {
		KGSL_CORE_ERR("map err: 0x%016llX, %d, %x, %zd\n",
			addr, nents, flags, mapped);
		return  -ENODEV;
>>>>>>> 0e91d2a... Nougat
	}
}

<<<<<<< HEAD
=======

static struct page *kgsl_guard_page;
static struct kgsl_memdesc kgsl_secure_guard_page_memdesc;



struct _mem_entry {
	uint64_t gpuaddr;
	uint64_t size;
	uint64_t flags;
	unsigned int priv;
	int pending_free;
	pid_t pid;
};
>>>>>>> 0e91d2a... Nougat

static void _next_entry(struct kgsl_process_private *priv,
	unsigned int faultaddr, struct _mem_entry *ret)
{
	struct rb_node *node;
	struct kgsl_mem_entry *entry;

	for (node = rb_last(&priv->mem_rb); node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);

		if (entry->memdesc.gpuaddr < faultaddr)
			break;


		if (entry->memdesc.gpuaddr < ret->gpuaddr) {
			ret->gpuaddr = entry->memdesc.gpuaddr;
			ret->size = entry->memdesc.size;
			ret->flags = entry->memdesc.flags;
			ret->priv = entry->memdesc.priv;
			ret->pending_free = entry->pending_free;
			ret->pid = priv->pid;
		}

		node = rb_prev(&entry->node);
	}
}

<<<<<<< HEAD
static void _find_mem_entries(struct kgsl_mmu *mmu, unsigned int faultaddr,
	unsigned int ptbase, struct _mem_entry *preventry,
	struct _mem_entry *nextentry)
{
	struct kgsl_process_private *private;
	int id = kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase);
=======
static void _find_mem_entries(struct kgsl_mmu *mmu, uint64_t faultaddr,
		struct _mem_entry *preventry, struct _mem_entry *nextentry,
		struct kgsl_context *context)
{
	struct kgsl_process_private *private;
>>>>>>> 0e91d2a... Nougat

	memset(preventry, 0, sizeof(*preventry));
	memset(nextentry, 0, sizeof(*nextentry));

	
<<<<<<< HEAD
	nextentry->gpuaddr = 0xFFFFFFFF;

	mutex_lock(&kgsl_driver.process_mutex);

	list_for_each_entry(private, &kgsl_driver.process_list, list) {

		if (private->pagetable && (private->pagetable->name != id))
			continue;

=======
	nextentry->gpuaddr = (uint64_t) -1;

	if (context) {
		private = context->proc_priv;
>>>>>>> 0e91d2a... Nougat
		spin_lock(&private->mem_lock);
		_prev_entry(private, faultaddr, preventry);
		_next_entry(private, faultaddr, nextentry);
		spin_unlock(&private->mem_lock);
	}

	mutex_unlock(&kgsl_driver.process_mutex);
}

static void _print_entry(struct kgsl_device *device, struct _mem_entry *entry)
{
	char name[32];
	memset(name, 0, sizeof(name));

	kgsl_get_memory_usage(name, sizeof(name) - 1, entry->flags);

	KGSL_LOG_DUMP(device,
		"[%8.8X - %8.8X] %s %s (pid = %d) (%s)\n",
		entry->gpuaddr,
		entry->gpuaddr + entry->size,
		entry->priv & KGSL_MEMDESC_GUARD_PAGE ? "(+guard)" : "",
		entry->pending_free ? "(pending free)" : "",
		entry->pid, name);
}

static void _check_if_freed(struct kgsl_iommu_device *iommu_dev,
	unsigned long addr, unsigned int pid)
{
	unsigned long gpuaddr = addr;
	unsigned long size = 0;
	unsigned int flags = 0;

	char name[32];
	memset(name, 0, sizeof(name));

	if (kgsl_memfree_find_entry(pid, &gpuaddr, &size, &flags)) {
		kgsl_get_memory_usage(name, sizeof(name) - 1, flags);
		KGSL_LOG_DUMP(iommu_dev->kgsldev, "---- premature free ----\n");
		KGSL_LOG_DUMP(iommu_dev->kgsldev,
			"[%8.8lX-%8.8lX] (%s) was already freed by pid %d\n",
			gpuaddr, gpuaddr + size, name, pid);
	}
}

static bool
kgsl_iommu_uche_overfetch(struct kgsl_process_private *private,
		uint64_t faultaddr)
{
	int id;
	struct kgsl_mem_entry *entry = NULL;

	spin_lock(&private->mem_lock);
	idr_for_each_entry(&private->mem_idr, entry, id) {
		struct kgsl_memdesc *m = &entry->memdesc;

		if ((faultaddr >= (m->gpuaddr + m->size))
				&& (faultaddr < (m->gpuaddr + m->size + 64))) {
			spin_unlock(&private->mem_lock);
			return true;
		}
	}
	spin_unlock(&private->mem_lock);
	return false;
}


static bool kgsl_iommu_suppress_pagefault(uint64_t faultaddr, int write,
					struct kgsl_context *context)
{
	if (!context || write)
		return false;

	return kgsl_iommu_uche_overfetch(context->proc_priv, faultaddr);
}

static int kgsl_iommu_fault_handler(struct iommu_domain *domain,
	struct device *dev, unsigned long addr, int flags, void *token)
{
	int ret = 0;
	struct kgsl_mmu *mmu;
	struct kgsl_iommu *iommu;
<<<<<<< HEAD
	struct kgsl_iommu_unit *iommu_unit;
	struct kgsl_iommu_device *iommu_dev;
	unsigned int ptbase, fsr;
	unsigned int pid;
=======
	struct kgsl_iommu_context *ctx;
	u64 ptbase;
	u32 contextidr;
	pid_t tid = 0;
	pid_t ptname;
>>>>>>> 0e91d2a... Nougat
	struct _mem_entry prev, next;
	unsigned int fsynr0, fsynr1;
	int write;
	struct kgsl_device *device;
	struct adreno_device *adreno_dev;
	unsigned int no_page_fault_log = 0;
	unsigned int curr_context_id = 0;
	struct kgsl_context *context;

<<<<<<< HEAD
	ret = get_iommu_unit(dev, &mmu, &iommu_unit);
	if (ret)
		goto done;

	device = mmu->device;
=======
	static DEFINE_RATELIMIT_STATE(_rs,
					DEFAULT_RATELIMIT_INTERVAL,
					DEFAULT_RATELIMIT_BURST);

	if (mmu == NULL)
		return ret;

	iommu = _IOMMU_PRIV(mmu);
	ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	device = KGSL_MMU_DEVICE(mmu);
>>>>>>> 0e91d2a... Nougat
	adreno_dev = ADRENO_DEVICE(device);
	if (1 == atomic_cmpxchg(&mmu->fault, 0, 1))
		goto done;

	iommu_dev = get_iommu_device(iommu_unit, dev);
	if (!iommu_dev) {
		KGSL_CORE_ERR("Invalid IOMMU device %p\n", dev);
		ret = -ENOSYS;
		goto done;
	}
	iommu = mmu->priv;

<<<<<<< HEAD
	fsr = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
		iommu_dev->ctx_id, FSR);
	if (!fsr) {
		atomic_set(&mmu->fault, 0);
		goto done;
	}
=======
>>>>>>> 0e91d2a... Nougat
	kgsl_sharedmem_readl(&device->memstore, &curr_context_id,
		KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL, current_context));

	context = kgsl_context_get(device, curr_context_id);

<<<<<<< HEAD
	if (context != NULL) {
		
		set_bit(KGSL_CONTEXT_PRIV_PAGEFAULT, &context->priv);
=======
	write = (flags & IOMMU_FAULT_WRITE) ? 1 : 0;
	if (flags & IOMMU_FAULT_TRANSLATION)
		fault_type = "translation";
	else if (flags & IOMMU_FAULT_PERMISSION)
		fault_type = "permission";
>>>>>>> 0e91d2a... Nougat

	if (kgsl_iommu_suppress_pagefault(addr, write, context)) {
		iommu->pagefault_suppression_count++;
		kgsl_context_put(context);
		return ret;
	}

	if (context != NULL) {
		
		set_bit(KGSL_CONTEXT_PRIV_PAGEFAULT, &context->priv);
		tid = context->tid;
	}

	iommu_dev->fault = 1;

<<<<<<< HEAD
	if (adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE) {
		adreno_set_gpu_fault(adreno_dev, ADRENO_IOMMU_PAGE_FAULT);
		
=======
	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
		&adreno_dev->ft_pf_policy) &&
		(flags & IOMMU_FAULT_TRANSACTION_STALLED)) {
		mutex_lock(&device->mutex);
>>>>>>> 0e91d2a... Nougat
		kgsl_pwrctrl_change_state(device, KGSL_STATE_AWARE);
		adreno_dispatcher_schedule(device);
	}

<<<<<<< HEAD
	ptbase = KGSL_IOMMU_GET_CTX_REG_Q(iommu, iommu_unit,
				iommu_dev->ctx_id, TTBR0);

	fsynr0 = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
		iommu_dev->ctx_id, FSYNR0);
	fsynr1 = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
		iommu_dev->ctx_id, FSYNR1);

	if (msm_soc_version_supports_iommu_v0())
		write = ((fsynr1 & (KGSL_IOMMU_FSYNR1_AWRITE_MASK <<
			KGSL_IOMMU_FSYNR1_AWRITE_SHIFT)) ? 1 : 0);
	else
		write = ((fsynr0 & (KGSL_IOMMU_V1_FSYNR0_WNR_MASK <<
			KGSL_IOMMU_V1_FSYNR0_WNR_SHIFT)) ? 1 : 0);

	pid = kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase);
=======
	ptbase = KGSL_IOMMU_GET_CTX_REG_Q(ctx, TTBR0);
	contextidr = KGSL_IOMMU_GET_CTX_REG(ctx, CONTEXTIDR);

	ptname = MMU_FEATURE(mmu, KGSL_MMU_GLOBAL_PAGETABLE) ?
		KGSL_MMU_GLOBAL_PT : tid;
>>>>>>> 0e91d2a... Nougat

	if (adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_LOG_ONE_PER_PAGE)
		no_page_fault_log = kgsl_mmu_log_fault_addr(mmu, ptbase, addr);

	if (!no_page_fault_log) {
		KGSL_MEM_CRIT(iommu_dev->kgsldev,
			"GPU PAGE FAULT: addr = %lX pid = %d\n", addr, pid);
		KGSL_MEM_CRIT(iommu_dev->kgsldev,
		 "context = %d TTBR0 = %X FSR = %X FSYNR0 = %X FSYNR1 = %X(%s fault)\n",
			iommu_dev->ctx_id, ptbase, fsr, fsynr0, fsynr1,
			write ? "write" : "read");

<<<<<<< HEAD
		_check_if_freed(iommu_dev, addr, pid);
=======
		
		if (!(flags & IOMMU_FAULT_PERMISSION)) {
			_check_if_freed(ctx, addr, ptname);
>>>>>>> 0e91d2a... Nougat

		KGSL_LOG_DUMP(iommu_dev->kgsldev, "---- nearby memory ----\n");

<<<<<<< HEAD
		_find_mem_entries(mmu, addr, ptbase, &prev, &next);

		if (prev.gpuaddr)
			_print_entry(iommu_dev->kgsldev, &prev);
		else
			KGSL_LOG_DUMP(iommu_dev->kgsldev, "*EMPTY*\n");
=======
			_find_mem_entries(mmu, addr, &prev, &next, context);
			if (prev.gpuaddr)
				_print_entry(ctx->kgsldev, &prev);
			else
				KGSL_LOG_DUMP(ctx->kgsldev, "*EMPTY*\n");
>>>>>>> 0e91d2a... Nougat

		KGSL_LOG_DUMP(iommu_dev->kgsldev, " <- fault @ %8.8lX\n", addr);

<<<<<<< HEAD
		if (next.gpuaddr != 0xFFFFFFFF)
			_print_entry(iommu_dev->kgsldev, &next);
		else
			KGSL_LOG_DUMP(iommu_dev->kgsldev, "*EMPTY*\n");

	}

	trace_kgsl_mmu_pagefault(iommu_dev->kgsldev, addr,
			kgsl_mmu_get_ptname_from_ptbase(mmu, ptbase),
			write ? "write" : "read");

	if (adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)
		ret = -EBUSY;
done:
	return ret;
}

static void kgsl_iommu_disable_clk(struct kgsl_mmu *mmu, int unit)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;
=======
			if (next.gpuaddr != (uint64_t) -1)
				_print_entry(ctx->kgsldev, &next);
			else
				KGSL_LOG_DUMP(ctx->kgsldev, "*EMPTY*\n");
		}
	}

	trace_kgsl_mmu_pagefault(ctx->kgsldev, addr,
			ptname, write ? "write" : "read");

	if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE,
		&adreno_dev->ft_pf_policy) &&
		(flags & IOMMU_FAULT_TRANSACTION_STALLED)) {
		uint32_t sctlr_val;
		ret = -EBUSY;
		sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);
		sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_CFIE_SHIFT);
		KGSL_IOMMU_SET_CTX_REG(ctx, SCTLR, sctlr_val);

		adreno_set_gpu_fault(adreno_dev, ADRENO_IOMMU_PAGE_FAULT);
		
		adreno_dispatcher_schedule(device);
	}

	kgsl_context_put(context);
	return ret;
}

static void kgsl_iommu_disable_clk(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	int j;
>>>>>>> 0e91d2a... Nougat

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];

		
		if ((unit != i) && (unit != KGSL_IOMMU_MAX_UNITS))
			continue;

		atomic_dec(&iommu_unit->clk_enable_count);
		BUG_ON(atomic_read(&iommu_unit->clk_enable_count) < 0);

		for (j = (KGSL_IOMMU_MAX_CLKS - 1); j >= 0; j--)
			if (iommu_unit->clks[j])
				clk_disable_unprepare(iommu_unit->clks[j]);
	}
}

<<<<<<< HEAD

static int kgsl_iommu_clk_prepare_enable(struct clk *clk)
=======
static void kgsl_iommu_clk_prepare_enable(struct clk *clk)
>>>>>>> 0e91d2a... Nougat
{
	int num_retries = 4;

	while (num_retries--) {
		if (!clk_prepare_enable(clk))
			return 0;
	}

<<<<<<< HEAD
	return 1;
}

static void kgsl_iommu_enable_clk(struct kgsl_mmu *mmu,
				int unit)
{
	int i, j;
	struct kgsl_iommu *iommu = mmu->priv;
=======
	
	KGSL_CORE_ERR("IOMMU clock enable failed\n");
	BUG();
}

static void kgsl_iommu_enable_clk(struct kgsl_mmu *mmu)
{
	int j;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
>>>>>>> 0e91d2a... Nougat

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];

		
		if ((unit != i) && (unit != KGSL_IOMMU_MAX_UNITS))
			continue;

		for (j = 0; j < KGSL_IOMMU_MAX_CLKS; j++) {
			if (iommu_unit->clks[j])
				if (kgsl_iommu_clk_prepare_enable(
						iommu_unit->clks[j]))
						goto done;
		}
		atomic_inc(&iommu_unit->clk_enable_count);
	}
	return;
done:
	KGSL_CORE_ERR("IOMMU clk enable failed\n");
	BUG();
}

<<<<<<< HEAD
static int kgsl_iommu_pt_equal(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pt,
				phys_addr_t pt_base)
=======
static u64 kgsl_iommu_get_ttbr0(struct kgsl_pagetable *pt)
>>>>>>> 0e91d2a... Nougat
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;
	phys_addr_t domain_ptbase;

	if (iommu_pt == NULL)
		return 0;

	domain_ptbase = iommu_get_pt_base_addr(iommu_pt->domain)
			& KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;

	pt_base &= KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;

	return (domain_ptbase == pt_base);

}

<<<<<<< HEAD
static phys_addr_t kgsl_iommu_get_ptbase(struct kgsl_pagetable *pt)
=======
static bool kgsl_iommu_pt_equal(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pt,
				u64 ttbr0)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;
	u64 domain_ttbr0;

	if (iommu_pt == NULL)
		return 0;

	domain_ttbr0 = kgsl_iommu_get_ttbr0(pt);

	return (domain_ttbr0 == ttbr0);
}

static u32 kgsl_iommu_get_contextidr(struct kgsl_pagetable *pt)
>>>>>>> 0e91d2a... Nougat
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;
	phys_addr_t domain_ptbase;

	if (iommu_pt == NULL)
		return 0;

	domain_ptbase = iommu_get_pt_base_addr(iommu_pt->domain);

	return domain_ptbase & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
}

static void kgsl_iommu_destroy_pagetable(struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
<<<<<<< HEAD
=======
	struct kgsl_mmu *mmu = pt->mmu;
	struct kgsl_iommu *iommu;
	struct kgsl_iommu_context  *ctx;

	BUG_ON(!list_empty(&pt->list));

	iommu = _IOMMU_PRIV(mmu);

	if (KGSL_MMU_SECURE_PT == pt->name) {
		ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];
		kgsl_iommu_unmap_global_secure_pt_entry(pt);
	}
	else {
		ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
		kgsl_iommu_unmap_globals(pt);
	}
>>>>>>> 0e91d2a... Nougat

	if (iommu_pt->domain) {
		phys_addr_t domain_ptbase =
			iommu_get_pt_base_addr(iommu_pt->domain);
		trace_kgsl_pagetable_destroy(domain_ptbase, pt->name);
		msm_unregister_domain(iommu_pt->domain);
	}

	kfree(iommu_pt);
	iommu_pt = NULL;
}

<<<<<<< HEAD
static void *kgsl_iommu_create_pagetable(void)
=======
static void setup_64bit_pagetable(struct kgsl_mmu *mmu,
		struct kgsl_pagetable *pagetable,
		struct kgsl_iommu_pt *pt)
{
	unsigned int secure_global_size = kgsl_global_secure_pt_entry != NULL ?
					kgsl_global_secure_pt_entry->size : 0;
	if (mmu->secured && pagetable->name == KGSL_MMU_SECURE_PT) {
		pt->compat_va_start = KGSL_IOMMU_SECURE_BASE +
						secure_global_size;
		pt->compat_va_end = KGSL_IOMMU_SECURE_END;
		pt->va_start = KGSL_IOMMU_SECURE_BASE + secure_global_size;
		pt->va_end = KGSL_IOMMU_SECURE_END;
	} else {
		pt->compat_va_start = KGSL_IOMMU_SVM_BASE32;
		pt->compat_va_end = KGSL_IOMMU_SVM_END32;
		pt->va_start = KGSL_IOMMU_VA_BASE64;
		pt->va_end = KGSL_IOMMU_VA_END64;
	}

	if (pagetable->name != KGSL_MMU_GLOBAL_PT &&
		pagetable->name != KGSL_MMU_SECURE_PT) {
		if ((BITS_PER_LONG == 32) || is_compat_task()) {
			pt->svm_start = KGSL_IOMMU_SVM_BASE32;
			pt->svm_end = KGSL_IOMMU_SVM_END32;
		} else {
			pt->svm_start = KGSL_IOMMU_SVM_BASE64;
			pt->svm_end = KGSL_IOMMU_SVM_END64;
		}
	}
}

static void setup_32bit_pagetable(struct kgsl_mmu *mmu,
		struct kgsl_pagetable *pagetable,
		struct kgsl_iommu_pt *pt)
{
	unsigned int secure_global_size = kgsl_global_secure_pt_entry != NULL ?
					kgsl_global_secure_pt_entry->size : 0;
	if (mmu->secured) {
		if (pagetable->name == KGSL_MMU_SECURE_PT) {
			pt->compat_va_start = KGSL_IOMMU_SECURE_BASE +
						secure_global_size;
			pt->compat_va_end = KGSL_IOMMU_SECURE_END;
			pt->va_start = KGSL_IOMMU_SECURE_BASE +
						secure_global_size;
			pt->va_end = KGSL_IOMMU_SECURE_END;
		} else {
			pt->va_start = KGSL_IOMMU_SVM_BASE32;
			pt->va_end = KGSL_IOMMU_SECURE_BASE +
						secure_global_size;
			pt->compat_va_start = pt->va_start;
			pt->compat_va_end = pt->va_end;
		}
	} else {
		pt->va_start = KGSL_IOMMU_SVM_BASE32;
		pt->va_end = KGSL_IOMMU_GLOBAL_MEM_BASE;
		pt->compat_va_start = pt->va_start;
		pt->compat_va_end = pt->va_end;
	}

	if (pagetable->name != KGSL_MMU_GLOBAL_PT &&
		pagetable->name != KGSL_MMU_SECURE_PT) {
		pt->svm_start = KGSL_IOMMU_SVM_BASE32;
		pt->svm_end = KGSL_IOMMU_SVM_END32;
	}
}


static struct kgsl_iommu_pt *
_alloc_pt(struct device *dev, struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
>>>>>>> 0e91d2a... Nougat
{
	int domain_num;
	struct kgsl_iommu_pt *iommu_pt;

	struct msm_iova_layout kgsl_layout = {
		
		.partitions = NULL,
		.npartitions = 0,
		.client_name = "kgsl",
		.domain_flags = 0,
	};

	iommu_pt = kzalloc(sizeof(struct kgsl_iommu_pt), GFP_KERNEL);
	if (!iommu_pt)
		return NULL;

	
	if (msm_soc_version_supports_iommu_v0())
		kgsl_layout.domain_flags = MSM_IOMMU_DOMAIN_PT_CACHEABLE;

<<<<<<< HEAD
	domain_num = msm_register_domain(&kgsl_layout);
	if (domain_num >= 0) {
		iommu_pt->domain = msm_get_iommu_domain(domain_num);
=======
	pt->pt_ops = &iommu_pt_ops;
	pt->priv = iommu_pt;
	pt->fault_addr = ~0ULL;
	iommu_pt->rbtree = RB_ROOT;
>>>>>>> 0e91d2a... Nougat

		if (iommu_pt->domain) {
			iommu_set_fault_handler(iommu_pt->domain,
				kgsl_iommu_fault_handler, NULL);

			return iommu_pt;
		}
	}

	KGSL_CORE_ERR("Failed to create iommu domain\n");
	kfree(iommu_pt);
	return NULL;
}

static void *kgsl_iommu_create_secure_pagetable(void)
{
	int domain_num;
	struct kgsl_iommu_pt *iommu_pt;

	struct msm_iova_layout kgsl_secure_layout = {
		
		.partitions = NULL,
		.npartitions = 0,
		.client_name = "kgsl_secure",
		.domain_flags = 0,
		.is_secure = 1,
	};

	iommu_pt = kzalloc(sizeof(struct kgsl_iommu_pt), GFP_KERNEL);
	if (!iommu_pt)
		return NULL;

	domain_num = msm_register_domain(&kgsl_secure_layout);
	if (domain_num >= 0) {
		iommu_pt->domain = msm_get_iommu_domain(domain_num);

		if (iommu_pt->domain)
			return iommu_pt;
	}

	KGSL_CORE_ERR("Failed to create secure iommu domain\n");
	kfree(iommu_pt);
	return NULL;
}

static void kgsl_detach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		iommu_pt = mmu->defaultpagetable->priv;
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (mmu->priv_bank_table &&
				(KGSL_IOMMU_CONTEXT_PRIV == j))
				iommu_pt = mmu->priv_bank_table->priv;
			if (mmu->securepagetable &&
				(KGSL_IOMMU_CONTEXT_SECURE == j))
				iommu_pt = mmu->securepagetable->priv;
			if (iommu_unit->dev[j].attached) {
				iommu_detach_device(iommu_pt->domain,
						iommu_unit->dev[j].dev);
				iommu_unit->dev[j].attached = false;
				KGSL_MEM_INFO(mmu->device, "iommu %p detached "
					"from user dev of MMU: %p\n",
					iommu_pt->domain, mmu);
			}
		}
	}
}
=======
	int ret = 0;
	struct kgsl_iommu_pt *iommu_pt = NULL;
	int disable_htw = !MMU_FEATURE(mmu, KGSL_MMU_COHERENT_HTW);
	unsigned int cb_num;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];

	iommu_pt = _alloc_pt(ctx->dev, mmu, pt);
>>>>>>> 0e91d2a... Nougat

static int kgsl_attach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	struct msm_iommu_drvdata *drvdata = 0;
	int i, j, ret = 0;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		iommu_pt = mmu->defaultpagetable->priv;
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (KGSL_IOMMU_CONTEXT_PRIV == j) {
				if (mmu->priv_bank_table)
					iommu_pt = mmu->priv_bank_table->priv;
				else
					continue;
			}

			if (KGSL_IOMMU_CONTEXT_SECURE == j) {
				if (mmu->securepagetable)
					iommu_pt = mmu->securepagetable->priv;
				else
					continue;
			}

			if (!iommu_unit->dev[j].attached) {
				ret = iommu_attach_device(iommu_pt->domain,
							iommu_unit->dev[j].dev);
				if (ret) {
					KGSL_MEM_ERR(mmu->device,
						"Failed to attach device, err %d\n",
						ret);
					goto done;
				}
				iommu_unit->dev[j].attached = true;
				KGSL_MEM_INFO(mmu->device,
				"iommu pt %p attached to dev %p, ctx_id %d\n",
				iommu_pt->domain, iommu_unit->dev[j].dev,
				iommu_unit->dev[j].ctx_id);
				
				if (!drvdata) {
					drvdata = dev_get_drvdata(
					iommu_unit->dev[j].dev->parent);
					iommu_unit->clks[0] = drvdata->pclk;
					iommu_unit->clks[1] = drvdata->clk;
					iommu_unit->clks[2] = drvdata->aclk;
					iommu_unit->clks[3] =
							iommu->gtcu_iface_clk;
				}
			}
		}
	}
done:
	return ret;
}

static int _get_iommu_ctxs(struct kgsl_mmu *mmu,
	struct kgsl_device_iommu_data *data, unsigned int unit_id)
{
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[unit_id];
	int i, j;
	int found_ctx;
	int ret = 0;

	for (j = 0; j < KGSL_IOMMU_CONTEXT_MAX; j++) {
		found_ctx = 0;
		for (i = 0; i < data->iommu_ctx_count; i++) {
			if (j == data->iommu_ctxs[i].ctx_id) {
				found_ctx = 1;
				break;
			}
		}
		if (!found_ctx)
			break;
		if (!data->iommu_ctxs[i].iommu_ctx_name) {
			KGSL_CORE_ERR("Context name invalid\n");
			ret = -EINVAL;
			goto done;
		}
		atomic_set(&(iommu_unit->clk_enable_count), 0);

		iommu_unit->dev[iommu_unit->dev_count].dev =
			msm_iommu_get_ctx(data->iommu_ctxs[i].iommu_ctx_name);
		if (NULL == iommu_unit->dev[iommu_unit->dev_count].dev)
			ret = -EINVAL;
		if (IS_ERR(iommu_unit->dev[iommu_unit->dev_count].dev)) {
			ret = PTR_ERR(
				iommu_unit->dev[iommu_unit->dev_count].dev);
			iommu_unit->dev[iommu_unit->dev_count].dev = NULL;
		}
		if (ret)
			goto done;
		iommu_unit->dev[iommu_unit->dev_count].ctx_id =
						data->iommu_ctxs[i].ctx_id;
		iommu_unit->dev[iommu_unit->dev_count].kgsldev = mmu->device;

		KGSL_DRV_INFO(mmu->device,
				"Obtained dev handle %p for iommu context %s\n",
				iommu_unit->dev[iommu_unit->dev_count].dev,
				data->iommu_ctxs[i].iommu_ctx_name);

		iommu_unit->dev_count++;
	}
done:
	if (!iommu_unit->dev_count && !ret)
		ret = -EINVAL;
	if (ret) {
		if (!msm_soc_version_supports_iommu_v0() &&
			iommu_unit->dev_count)
			ret = 0;
		else
			KGSL_CORE_ERR(
			"Failed to initialize iommu contexts, err: %d\n", ret);
	}

<<<<<<< HEAD
=======
	kgsl_iommu_map_globals(pt);

done:
	if (ret)
		_free_pt(ctx, pt);

>>>>>>> 0e91d2a... Nougat
	return ret;
}

static int kgsl_iommu_start_sync_lock(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = mmu->priv;
	uint32_t lock_gpu_addr = 0;
=======
	int ret = 0;
	struct kgsl_iommu_pt *iommu_pt = NULL;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	int disable_htw = !MMU_FEATURE(mmu, KGSL_MMU_COHERENT_HTW);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];
	int secure_vmid = VMID_CP_PIXEL;
	unsigned int cb_num;
>>>>>>> 0e91d2a... Nougat

	if (KGSL_DEVICE_3D0 != mmu->device->id ||
		!msm_soc_version_supports_iommu_v0() ||
		!kgsl_mmu_is_perprocess(mmu) ||
		iommu->sync_lock_vars)
		return 0;

	if (!(mmu->flags & KGSL_MMU_FLAGS_IOMMU_SYNC)) {
		KGSL_DRV_ERR(mmu->device,
		"The GPU microcode does not support IOMMUv1 sync opcodes\n");
		return -ENXIO;
	}
	
	lock_gpu_addr = (iommu->sync_lock_desc.gpuaddr +
			iommu->sync_lock_offset);

	kgsl_iommu_sync_lock_vars.flag[PROC_APPS] = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock,
			flag[PROC_APPS])));
	kgsl_iommu_sync_lock_vars.flag[PROC_GPU] = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock,
			flag[PROC_GPU])));
	kgsl_iommu_sync_lock_vars.turn = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock, turn)));

	iommu->sync_lock_vars = &kgsl_iommu_sync_lock_vars;

	return 0;
}

#ifdef CONFIG_MSM_IOMMU_GPU_SYNC

static int kgsl_iommu_init_sync_lock(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int status = 0;
	uint32_t lock_phy_addr = 0;
	uint32_t page_offset = 0;

	if (!msm_soc_version_supports_iommu_v0() ||
		!kgsl_mmu_is_perprocess(mmu))
		return status;

	
	if (iommu->sync_lock_initialized)
		return status;

	iommu_access_ops = msm_get_iommu_access_ops();

	if (iommu_access_ops && iommu_access_ops->iommu_lock_initialize) {
		lock_phy_addr = (uint32_t)
				iommu_access_ops->iommu_lock_initialize();
		if (!lock_phy_addr) {
			iommu_access_ops = NULL;
			return status;
		}
		lock_phy_addr = lock_phy_addr - (uint32_t)MSM_SHARED_RAM_BASE +
				(uint32_t)msm_shared_ram_phys;
	}

	
	page_offset = (lock_phy_addr & (PAGE_SIZE - 1));
	lock_phy_addr = (lock_phy_addr & ~(PAGE_SIZE - 1));
	iommu->sync_lock_desc.physaddr = (unsigned int)lock_phy_addr;
	iommu->sync_lock_offset = page_offset;

<<<<<<< HEAD
	iommu->sync_lock_desc.size =
				PAGE_ALIGN(sizeof(kgsl_iommu_sync_lock_vars));
	status =  memdesc_sg_phys(&iommu->sync_lock_desc,
				 iommu->sync_lock_desc.physaddr,
				 iommu->sync_lock_desc.size);

	if (status) {
		iommu_access_ops = NULL;
		return status;
	}

	
	iommu->sync_lock_desc.priv |= KGSL_MEMDESC_PRIVATE;
	status = kgsl_add_global_pt_entry(mmu->device, &iommu->sync_lock_desc);
	if (status) {
		kgsl_sg_free(iommu->sync_lock_desc.sg,
			iommu->sync_lock_desc.sglen);
		iommu_access_ops = NULL;
		return status;
	}

	
	iommu->sync_lock_initialized = 1;

	return status;
=======
	kgsl_map_global_secure_pt_entry(pt);

done:
	if (ret)
		_free_pt(ctx, pt);
	return ret;
>>>>>>> 0e91d2a... Nougat
}
#else
static int kgsl_iommu_init_sync_lock(struct kgsl_mmu *mmu)
{
	return 0;
}
#endif

static unsigned int kgsl_iommu_sync_lock(struct kgsl_mmu *mmu,
						unsigned int *cmds)
{
<<<<<<< HEAD
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct remote_iommu_petersons_spinlock *lock_vars =
					iommu->sync_lock_vars;
	unsigned int *start = cmds;
=======
	int ret = 0;
	struct kgsl_iommu_pt *iommu_pt = NULL;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	int dynamic = 1;
	unsigned int cb_num = ctx->cb_num;
	int disable_htw = !MMU_FEATURE(mmu, KGSL_MMU_COHERENT_HTW);
>>>>>>> 0e91d2a... Nougat

	if (!iommu->sync_lock_initialized)
		return 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 1;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x1;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->turn;
	*cmds++ = 0;

<<<<<<< HEAD
	cmds += adreno_add_idle_cmds(adreno_dev, cmds);
=======
	
	ret = iommu_domain_get_attr(iommu_pt->domain,
				DOMAIN_ATTR_TTBR0, &iommu_pt->ttbr0);
	if (ret) {
		KGSL_CORE_ERR("get DOMAIN_ATTR_TTBR0 failed: %d\n", ret);
		goto done;
	}
>>>>>>> 0e91d2a... Nougat

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x1;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_TEST_TWO_MEMS, 3);
	*cmds++ = lock_vars->flag[PROC_APPS];
	*cmds++ = lock_vars->turn;
	*cmds++ = 0;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - start;
}

static unsigned int kgsl_iommu_sync_unlock(struct kgsl_mmu *mmu,
					unsigned int *cmds)
{
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct remote_iommu_petersons_spinlock *lock_vars =
						iommu->sync_lock_vars;
	unsigned int *start = cmds;

	if (!iommu->sync_lock_initialized)
		return 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x0;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	
	*cmds++ = cp_type3_packet(CP_SET_PROTECTED_MODE, 1);
	*cmds++ = 0;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - start;
}

static int kgsl_get_iommu_ctxt(struct kgsl_mmu *mmu)
{
	struct kgsl_device_platform_data *pdata =
		dev_get_platdata(&mmu->device->pdev->dev);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	int i, ret = 0;

	
	if (KGSL_IOMMU_MAX_UNITS < pdata->iommu_count) {
		KGSL_CORE_ERR("Too many IOMMU units defined\n");
		ret = -EINVAL;
		goto  done;
	}

<<<<<<< HEAD
	for (i = 0; i < pdata->iommu_count; i++) {
		ret = _get_iommu_ctxs(mmu, &pdata->iommu_data[i], i);
		if (ret)
			break;
	}
	iommu->unit_count = pdata->iommu_count;
=======
	kgsl_iommu_map_globals(pt);

>>>>>>> 0e91d2a... Nougat
done:
	return ret;
}

<<<<<<< HEAD
static int kgsl_set_register_map(struct kgsl_mmu *mmu)
=======
static int kgsl_iommu_init_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
>>>>>>> 0e91d2a... Nougat
{
	struct kgsl_device_platform_data *pdata =
		dev_get_platdata(&mmu->device->pdev->dev);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct kgsl_iommu_unit *iommu_unit;
	int i = 0, ret = 0;
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	for (; i < pdata->iommu_count; i++) {
		struct kgsl_device_iommu_data data = pdata->iommu_data[i];
		iommu_unit = &iommu->iommu_units[i];
		
		if (!data.physstart || !data.physend) {
			KGSL_CORE_ERR("The register range for IOMMU unit not"
					" specified\n");
			ret = -EINVAL;
			goto err;
		}
		
		iommu_unit->reg_map.hostptr = ioremap(data.physstart,
					data.physend - data.physstart + 1);
		if (!iommu_unit->reg_map.hostptr) {
			KGSL_CORE_ERR("Failed to map SMMU register address "
				"space from %x to %x\n", data.physstart,
				data.physend - data.physstart + 1);
			ret = -ENOMEM;
			i--;
			goto err;
		}
		iommu_unit->reg_map.size = data.physend - data.physstart + 1;
		iommu_unit->reg_map.physaddr = data.physstart;
		ret = memdesc_sg_phys(&iommu_unit->reg_map, data.physstart,
				iommu_unit->reg_map.size);
		if (ret)
			goto err;

		if (msm_soc_version_supports_iommu_v0()) {
			iommu_unit->reg_map.priv |= KGSL_MEMDESC_PRIVATE;
			kgsl_add_global_pt_entry(mmu->device,
					&iommu_unit->reg_map);

		}

		if (!msm_soc_version_supports_iommu_v0())
			iommu_unit->iommu_halt_enable = 1;

		if (kgsl_msm_supports_iommu_v2())
			if (adreno_is_a405(adreno_dev)) {
				iommu_unit->ahb_base =
					KGSL_IOMMU_V2_AHB_BASE_A405;
			} else
				iommu_unit->ahb_base = KGSL_IOMMU_V2_AHB_BASE;
		else
			iommu_unit->ahb_base =
				data.physstart - mmu->device->reg_phys;
	}
	iommu->unit_count = pdata->iommu_count;
	return ret;
err:
	
	for (; i >= 0; i--) {
		iommu_unit = &iommu->iommu_units[i];

		kgsl_remove_global_pt_entry(&iommu_unit->reg_map);
		iounmap(iommu_unit->reg_map.hostptr);
		iommu_unit->reg_map.size = 0;
		iommu_unit->reg_map.physaddr = 0;
	}
	return ret;
}

static phys_addr_t kgsl_iommu_get_pt_base_addr(struct kgsl_mmu *mmu,
						struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	return iommu_get_pt_base_addr(iommu_pt->domain) &
			KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
}

static uint64_t kgsl_iommu_get_default_ttbr0(struct kgsl_mmu *mmu,
				unsigned int unit_id,
				enum kgsl_iommu_context_id ctx_id)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;
	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++)
			if (unit_id == i &&
				ctx_id == iommu_unit->dev[j].ctx_id)
				return iommu_unit->dev[j].default_ttbr0;
	}
	return 0;
}

<<<<<<< HEAD
static unsigned int kgsl_iommu_get_reg_ahbaddr(struct kgsl_mmu *mmu,
					int iommu_unit, int ctx_id,
					enum kgsl_iommu_reg_map reg)
{
	struct kgsl_iommu *iommu = mmu->priv;

	if (iommu->iommu_reg_list[reg].ctx_reg)
		return iommu->iommu_units[iommu_unit].ahb_base +
			iommu->iommu_reg_list[reg].reg_offset +
			(ctx_id << KGSL_IOMMU_CTX_SHIFT) +
			iommu->ctx_ahb_offset;
	else
		return iommu->iommu_units[iommu_unit].ahb_base +
			iommu->iommu_reg_list[reg].reg_offset;
=======
static struct kgsl_pagetable *kgsl_iommu_getpagetable(struct kgsl_mmu *mmu,
		unsigned long name)
{
	struct kgsl_pagetable *pt;

	if (!kgsl_mmu_is_perprocess(mmu) && (name != KGSL_MMU_SECURE_PT)) {
		name = KGSL_MMU_GLOBAL_PT;
		if (mmu->defaultpagetable != NULL)
			return mmu->defaultpagetable;
	}

	pt = kgsl_get_pagetable(name);
	if (pt == NULL)
		pt = kgsl_mmu_createpagetableobject(mmu, name);

	return pt;
}

static unsigned int kgsl_iommu_get_reg_ahbaddr(struct kgsl_mmu *mmu,
		int id, unsigned int reg)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[id];

	return ctx->gpu_offset + kgsl_iommu_reg_list[reg];
}

static void _detach_context(struct kgsl_iommu_context *ctx)
{
	struct kgsl_iommu_pt *iommu_pt;

	if (ctx->default_pt == NULL)
		return;

	iommu_pt = ctx->default_pt->priv;

	_detach_pt(iommu_pt, ctx);

	ctx->default_pt = NULL;
}

static void kgsl_iommu_close(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	int i;

	for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++)
		_detach_context(&iommu->ctx[i]);

	kgsl_mmu_putpagetable(mmu->defaultpagetable);
	mmu->defaultpagetable = NULL;

	kgsl_mmu_putpagetable(mmu->securepagetable);
	mmu->securepagetable = NULL;

	if (iommu->regbase != NULL)
		iounmap(iommu->regbase);

	kgsl_sharedmem_free(&kgsl_secure_guard_page_memdesc);

	if (kgsl_guard_page != NULL) {
		__free_page(kgsl_guard_page);
		kgsl_guard_page = NULL;
	}

	kgsl_iommu_remove_global(mmu, &iommu->setstate);
	kgsl_sharedmem_free(&iommu->setstate);
	kgsl_cleanup_qdss_desc(mmu);
}

static int _setstate_alloc(struct kgsl_device *device,
		struct kgsl_iommu *iommu)
{
	int ret;

	ret = kgsl_sharedmem_alloc_contig(device, &iommu->setstate, NULL,
		PAGE_SIZE);
	if (ret)
		return ret;

	
	iommu->setstate.flags |= KGSL_MEMFLAGS_GPUREADONLY;

	kgsl_sharedmem_set(device, &iommu->setstate, 0, 0, PAGE_SIZE);

	return 0;
>>>>>>> 0e91d2a... Nougat
}

static int kgsl_iommu_init(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	int status = 0;
	struct kgsl_iommu *iommu;
	struct platform_device *pdev = mmu->device->pdev;
	size_t secured_pool_sz = 0;
=======
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	int status;

	mmu->features |= KGSL_MMU_PAGED;
>>>>>>> 0e91d2a... Nougat

	atomic_set(&mmu->fault, 0);
	iommu = kzalloc(sizeof(struct kgsl_iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

<<<<<<< HEAD
	mmu->priv = iommu;
	status = kgsl_get_iommu_ctxt(mmu);
	if (status)
		goto done;
	status = kgsl_set_register_map(mmu);
	if (status)
		goto done;
=======
	status = _setstate_alloc(device, iommu);
	if (status)
		return status;

	
	if (ctx->gpu_offset == UINT_MAX) {
		KGSL_CORE_ERR("missing qcom,gpu-offset forces global pt\n");
		mmu->features |= KGSL_MMU_GLOBAL_PAGETABLE;
	}
>>>>>>> 0e91d2a... Nougat

	if (mmu->secured)
		secured_pool_sz = KGSL_IOMMU_SECURE_MEM_SIZE;

<<<<<<< HEAD
	if (KGSL_MMU_USE_PER_PROCESS_PT &&
		of_property_match_string(pdev->dev.of_node, "clock-names",
						"gtcu_iface_clk") >= 0)
		iommu->gtcu_iface_clk = clk_get(&pdev->dev, "gtcu_iface_clk");

	mmu->pt_base = KGSL_MMU_MAPPED_MEM_BASE;
	mmu->pt_size = (KGSL_MMU_MAPPED_MEM_SIZE - secured_pool_sz);
=======
	
	need_iommu_sync = of_property_read_bool(device->pdev->dev.of_node,
		"qcom,gpu-quirk-iommu-sync");

	iommu->regbase = ioremap(iommu->regstart, iommu->regsize);
	if (iommu->regbase == NULL) {
		KGSL_CORE_ERR("Could not map IOMMU registers 0x%lx:0x%x\n",
			iommu->regstart, iommu->regsize);
		status = -ENOMEM;
		goto done;
	}
>>>>>>> 0e91d2a... Nougat

	status = kgsl_iommu_init_sync_lock(mmu);
	if (status)
		goto done;

	if (kgsl_msm_supports_iommu_v2()) {
		iommu->iommu_reg_list = kgsl_iommuv1_reg;
		iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V2;
		iommu->ctx_ahb_offset = KGSL_IOMMU_CTX_AHB_OFFSET_V2;
	} else if (msm_soc_version_supports_iommu_v0()) {
		iommu->iommu_reg_list = kgsl_iommuv0_reg;
		iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V0;
		iommu->ctx_ahb_offset = KGSL_IOMMU_CTX_OFFSET_V0;
	} else {
		iommu->iommu_reg_list = kgsl_iommuv1_reg;
		iommu->ctx_offset = KGSL_IOMMU_CTX_OFFSET_V1;
		iommu->ctx_ahb_offset = KGSL_IOMMU_CTX_OFFSET_V1;
	}

<<<<<<< HEAD
	kgsl_sharedmem_writel(mmu->device, &mmu->setstate_memory,
				KGSL_IOMMU_SETSTATE_NOP_OFFSET,
				cp_nop_packet(1));

	if (kgsl_guard_page == NULL) {
		kgsl_guard_page = alloc_page(GFP_KERNEL | __GFP_ZERO |
				__GFP_HIGHMEM);
		if (kgsl_guard_page == NULL) {
			status = -ENOMEM;
			goto done;
		}
	}

done:
	if (status) {
		kfree(iommu);
		mmu->priv = NULL;
	}
	return status;
}

static int kgsl_iommu_setup_defaultpagetable(struct kgsl_mmu *mmu)
{
	int status = 0;

	if (msm_soc_version_supports_iommu_v0()) {
		mmu->priv_bank_table =
			kgsl_mmu_getpagetable(mmu, KGSL_MMU_PRIV_PT);
		if (mmu->priv_bank_table == NULL) {
			status = -ENOMEM;
			goto err;
		}
	}
	mmu->defaultpagetable = kgsl_mmu_getpagetable(mmu, KGSL_MMU_GLOBAL_PT);
	
	if (mmu->defaultpagetable == NULL) {
		status = -ENOMEM;
		goto err;
	}

	if (mmu->secured) {
		mmu->securepagetable = kgsl_mmu_getpagetable(mmu,
				KGSL_MMU_SECURE_PT);
		
		if (mmu->securepagetable == NULL) {
			KGSL_DRV_ERR(mmu->device,
			"Unable to create secure pagetable, disable content protection\n");
			status = -ENOMEM;
			goto err;
		}
	}
	return status;
err:
	if (mmu->priv_bank_table) {
		kgsl_mmu_putpagetable(mmu->priv_bank_table);
		mmu->priv_bank_table = NULL;
	}
	if (mmu->defaultpagetable) {
		kgsl_mmu_putpagetable(mmu->defaultpagetable);
		mmu->defaultpagetable = NULL;
	}
	return status;
}

static void kgsl_iommu_lock_rb_in_tlb(struct kgsl_mmu *mmu)
{
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb;
	struct kgsl_iommu *iommu = mmu->priv;
	unsigned int num_tlb_entries;
	unsigned int tlblkcr = 0;
	unsigned int v2pxx = 0;
	unsigned int vaddr = 0;
	int i, j, k, l;

	if (!iommu->sync_lock_initialized)
		return;

	rb = ADRENO_CURRENT_RINGBUFFER(adreno_dev);
	num_tlb_entries = rb->buffer_desc.size / PAGE_SIZE;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (!iommu_unit->dev[j].attached)
				continue;
			tlblkcr = 0;
			tlblkcr |= (((num_tlb_entries *
				iommu_unit->dev_count) &
				KGSL_IOMMU_TLBLKCR_FLOOR_MASK) <<
				KGSL_IOMMU_TLBLKCR_FLOOR_SHIFT);
			
			tlblkcr	|= ((1 & KGSL_IOMMU_TLBLKCR_TLBIALLCFG_MASK)
				<< KGSL_IOMMU_TLBLKCR_TLBIALLCFG_SHIFT);
			tlblkcr	|= ((1 & KGSL_IOMMU_TLBLKCR_TLBIASIDCFG_MASK)
				<< KGSL_IOMMU_TLBLKCR_TLBIASIDCFG_SHIFT);
			tlblkcr	|= ((1 & KGSL_IOMMU_TLBLKCR_TLBIVAACFG_MASK)
				<< KGSL_IOMMU_TLBLKCR_TLBIVAACFG_SHIFT);
			
			tlblkcr |= ((1 & KGSL_IOMMU_TLBLKCR_LKE_MASK)
				<< KGSL_IOMMU_TLBLKCR_LKE_SHIFT);
			KGSL_IOMMU_SET_CTX_REG(iommu, iommu_unit,
					iommu_unit->dev[j].ctx_id,
					TLBLKCR, tlblkcr);
		}
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (!iommu_unit->dev[j].attached)
				continue;
			
			vaddr = rb->buffer_desc.gpuaddr;
			for (k = 0; k < num_tlb_entries; k++) {
				v2pxx = 0;
				v2pxx |= (((k + j * num_tlb_entries) &
					KGSL_IOMMU_V2PXX_INDEX_MASK)
					<< KGSL_IOMMU_V2PXX_INDEX_SHIFT);
				v2pxx |= vaddr & (KGSL_IOMMU_V2PXX_VA_MASK <<
						KGSL_IOMMU_V2PXX_VA_SHIFT);

				KGSL_IOMMU_SET_CTX_REG(iommu, iommu_unit,
						iommu_unit->dev[j].ctx_id,
						V2PUR, v2pxx);
				mb();
				vaddr += PAGE_SIZE;
				for (l = 0; l < iommu_unit->dev_count; l++) {
					tlblkcr = KGSL_IOMMU_GET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[l].ctx_id,
						TLBLKCR);
					mb();
					tlblkcr &=
					~(KGSL_IOMMU_TLBLKCR_VICTIM_MASK
					<< KGSL_IOMMU_TLBLKCR_VICTIM_SHIFT);
					tlblkcr |= (((k + 1 +
					(j * num_tlb_entries)) &
					KGSL_IOMMU_TLBLKCR_VICTIM_MASK) <<
					KGSL_IOMMU_TLBLKCR_VICTIM_SHIFT);
					KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[l].ctx_id,
						TLBLKCR, tlblkcr);
				}
			}
		}
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (!iommu_unit->dev[j].attached)
				continue;
			tlblkcr = KGSL_IOMMU_GET_CTX_REG(iommu, iommu_unit,
						iommu_unit->dev[j].ctx_id,
						TLBLKCR);
			mb();
			
			tlblkcr &= ~(KGSL_IOMMU_TLBLKCR_LKE_MASK
				<< KGSL_IOMMU_TLBLKCR_LKE_SHIFT);
			KGSL_IOMMU_SET_CTX_REG(iommu, iommu_unit,
				iommu_unit->dev[j].ctx_id, TLBLKCR, tlblkcr);
		}
	}
=======
	kgsl_iommu_add_global(mmu, &iommu->setstate);
	kgsl_setup_qdss_desc(device);

done:
	if (status)
		kgsl_iommu_close(mmu);

	return status;
>>>>>>> 0e91d2a... Nougat
}

static int kgsl_iommu_start(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	int status;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;
	int sctlr_val = 0;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(mmu->device);

	if (mmu->defaultpagetable == NULL) {
		status = kgsl_iommu_setup_defaultpagetable(mmu);
		if (status)
			return -ENOMEM;
=======
	int ret = 0;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu_pt *iommu_pt = NULL;
	unsigned int  sctlr_val;

	if (mmu->defaultpagetable == NULL) {
		mmu->defaultpagetable = kgsl_mmu_getpagetable(mmu,
				KGSL_MMU_GLOBAL_PT);
		
		if (IS_ERR(mmu->defaultpagetable)) {
			ret = PTR_ERR(mmu->defaultpagetable);
			mmu->defaultpagetable = NULL;
			return ret;
		}
>>>>>>> 0e91d2a... Nougat
	}
	status = kgsl_iommu_start_sync_lock(mmu);
	if (status)
		return status;

<<<<<<< HEAD
	status = kgsl_attach_pagetable_iommu_domain(mmu);
	if (status)
		goto done;
=======
	iommu_pt = mmu->defaultpagetable->priv;
	if (iommu_pt == NULL)
		return -ENODEV;
>>>>>>> 0e91d2a... Nougat

	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

	_iommu_lock(iommu);
	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {

			if ((!iommu_unit->dev[j].attached) ||
				(KGSL_IOMMU_CONTEXT_SECURE == j))
				continue;

			if ((!msm_soc_version_supports_iommu_v0()) &&
				(!(adreno_dev->ft_pf_policy &
				   KGSL_FT_PAGEFAULT_GPUHALT_ENABLE))) {
				sctlr_val = KGSL_IOMMU_GET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						SCTLR);
				sctlr_val |= (0x1 <<
						KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
				KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						SCTLR, sctlr_val);
			}
			iommu_unit->dev[j].default_ttbr0 =
				KGSL_IOMMU_GET_CTX_REG_Q(iommu,
					iommu_unit,
					iommu_unit->dev[j].ctx_id, TTBR0);
		}
	}
	kgsl_iommu_lock_rb_in_tlb(mmu);
	_iommu_unlock(iommu);

	
	kgsl_cffdump_write(mmu->device, mmu->setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET,
				cp_nop_packet(1));

	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

done:
	return status;
}

<<<<<<< HEAD
static void kgsl_iommu_flush_tlb_pt_current(struct kgsl_pagetable *pt,
				struct kgsl_memdesc *memdesc)
{
	struct kgsl_iommu *iommu = pt->mmu->priv;
=======
>>>>>>> 0e91d2a... Nougat

	if (kgsl_memdesc_is_secured(memdesc))
		return;

	mutex_lock(&pt->mmu->device->mutex);
	if (kgsl_mmu_is_perprocess(pt->mmu) &&
		iommu->iommu_units[0].dev[KGSL_IOMMU_CONTEXT_USER].attached &&
		kgsl_iommu_pt_equal(pt->mmu, pt,
		kgsl_iommu_get_current_ptbase(pt->mmu)))
		kgsl_iommu_flush_pt(pt->mmu);
	mutex_unlock(&pt->mmu->device->mutex);
}

static int
kgsl_iommu_unmap(struct kgsl_pagetable *pt,
		struct kgsl_memdesc *memdesc)
{
<<<<<<< HEAD
	struct kgsl_device *device = pt->mmu->device;
	int ret = 0;
	unsigned int range = memdesc->size;
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
=======
	int ret;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_SECURE];
	unsigned int cb_num;
>>>>>>> 0e91d2a... Nougat


	unsigned int gpuaddr = PAGE_ALIGN(memdesc->gpuaddr);

	if (range == 0 || gpuaddr == 0)
		return 0;

	if (kgsl_memdesc_has_guard_page(memdesc))
		range += kgsl_memdesc_guard_page_size(memdesc);

	if (kgsl_memdesc_is_secured(memdesc)) {

		if (!kgsl_mmu_is_secured(pt->mmu))
			return -EINVAL;

		mutex_lock(&device->mutex);
		ret = kgsl_active_count_get(device);
		if (!ret) {
			ret = iommu_unmap_range(iommu_pt->domain,
						gpuaddr, range);
			kgsl_active_count_put(device);
		}
		mutex_unlock(&device->mutex);
	} else
		ret = iommu_unmap_range(iommu_pt->domain, gpuaddr, range);
	if (ret) {
		KGSL_CORE_ERR("iommu_unmap_range(%p, %x, %d) failed "
			"with err: %d\n", iommu_pt->domain, gpuaddr,
			range, ret);
		return ret;
	}

	if (!kgsl_memdesc_is_global(memdesc))
		kgsl_iommu_flush_tlb_pt_current(pt, memdesc);

	return ret;
}

<<<<<<< HEAD
struct scatterlist *_create_sg_no_large_pages(struct kgsl_memdesc *memdesc)
{
	struct page *page;
	struct scatterlist *s, *s_temp, *sg_temp;
	int sglen_alloc = 0;
	uint64_t offset, pg_size;
	int i;

	for_each_sg(memdesc->sg, s, memdesc->sglen, i) {
		if (SZ_1M <= s->length) {
			sglen_alloc += s->length >> 16;
			sglen_alloc += ((s->length & 0xF000) >> 12);
		} else {
			sglen_alloc++;
		}
	}
	
	if (sglen_alloc == memdesc->sglen)
		return NULL;
=======
static int kgsl_iommu_set_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt);

static int kgsl_iommu_start(struct kgsl_mmu *mmu)
{
	int status;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);

	status = _setup_user_context(mmu);
	if (status)
		return status;

	status = _setup_secure_context(mmu);
	if (status) {
		_detach_context(&iommu->ctx[KGSL_IOMMU_CONTEXT_USER]);
		return status;
	}

	
	return kgsl_iommu_set_pt(mmu, mmu->defaultpagetable);
}

static int
kgsl_iommu_unmap_offset(struct kgsl_pagetable *pt,
		struct kgsl_memdesc *memdesc, uint64_t addr,
		uint64_t offset, uint64_t size)
{

	addr = PAGE_ALIGN(addr);

	if (size == 0 || addr == 0)
		return 0;

	return _iommu_unmap_sync_pc(pt, memdesc, addr + offset, size);
}

static int
kgsl_iommu_unmap(struct kgsl_pagetable *pt, struct kgsl_memdesc *memdesc)
{
	uint64_t size = memdesc->size;
>>>>>>> 0e91d2a... Nougat

	sg_temp = kgsl_malloc(sglen_alloc * sizeof(struct scatterlist));
	if (NULL == sg_temp)
		return ERR_PTR(-ENOMEM);

<<<<<<< HEAD
	sg_init_table(sg_temp, sglen_alloc);
	s_temp = sg_temp;

	for_each_sg(memdesc->sg, s, memdesc->sglen, i) {
		page = sg_page(s);
		if (SZ_1M <= s->length) {
			for (offset = 0; offset < s->length; s_temp++) {
				pg_size = ((s->length - offset) >= SZ_64K) ?
						SZ_64K : SZ_4K;
				sg_set_page(s_temp, page, pg_size, offset);
				offset += pg_size;
			}
		} else {
			sg_set_page(s_temp, page, s->length, 0);
			s_temp++;
		}
	}
	return sg_temp;
}

int _iommu_add_guard_page(struct kgsl_pagetable *pt,
						   struct kgsl_memdesc *memdesc,
						   unsigned int gpuaddr,
						   unsigned int protflags)
=======
	return kgsl_iommu_unmap_offset(pt, memdesc, memdesc->gpuaddr, 0,
			size);
}

static int _iommu_map_guard_page(struct kgsl_pagetable *pt,
				   struct kgsl_memdesc *memdesc,
				   uint64_t gpuaddr,
				   unsigned int protflags)
>>>>>>> 0e91d2a... Nougat
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	phys_addr_t physaddr = page_to_phys(kgsl_guard_page);
	int ret;

	if (kgsl_memdesc_has_guard_page(memdesc)) {

<<<<<<< HEAD
		if (kgsl_memdesc_is_secured(memdesc)) {
			if (!kgsl_secure_guard_page_memdesc.physaddr) {
				if (kgsl_cma_alloc_secure(pt->mmu->device,
					&kgsl_secure_guard_page_memdesc,
					SZ_1M)) {
					KGSL_CORE_ERR(
=======
	if (kgsl_memdesc_is_secured(memdesc)) {
		struct scatterlist *sg;
		unsigned int sgp_size = pt->mmu->secure_align_mask + 1;

		if (!kgsl_secure_guard_page_memdesc.sgt) {
			if (kgsl_allocate_user(KGSL_MMU_DEVICE(pt->mmu),
					&kgsl_secure_guard_page_memdesc, pt,
					sgp_size, KGSL_MEMFLAGS_SECURE)) {
				KGSL_CORE_ERR(
>>>>>>> 0e91d2a... Nougat
					"Secure guard page alloc failed\n");
					return -ENOMEM;
				}
			}

			physaddr = kgsl_secure_guard_page_memdesc.physaddr;
		}

<<<<<<< HEAD
		ret = iommu_map(iommu_pt->domain, gpuaddr, physaddr,
				kgsl_memdesc_guard_page_size(memdesc),
				protflags & ~IOMMU_WRITE);
		if (ret) {
			KGSL_CORE_ERR(
			"iommu_map(%p, addr %x, flags %x) err: %d\n",
			iommu_pt->domain, gpuaddr, protflags & ~IOMMU_WRITE,
			ret);
			return ret;
		}
=======
		sg = kgsl_secure_guard_page_memdesc.sgt->sgl;
		physaddr = page_to_phys(sg_page(sg));
	} else {
		if (kgsl_guard_page == NULL) {
			kgsl_guard_page = alloc_page(GFP_KERNEL | __GFP_ZERO |
					__GFP_HIGHMEM);
			if (kgsl_guard_page == NULL)
				return -ENOMEM;
		}

		physaddr = page_to_phys(kgsl_guard_page);
>>>>>>> 0e91d2a... Nougat
	}

	return 0;
}

static unsigned int _get_protection_flags(struct kgsl_memdesc *memdesc)
{
	unsigned int flags = IOMMU_READ | IOMMU_WRITE | IOMMU_NOEXEC;

	if (memdesc->flags & KGSL_MEMFLAGS_GPUREADONLY)
		flags &= ~IOMMU_WRITE;

	if (memdesc->priv & KGSL_MEMDESC_PRIVILEGED)
		flags |= IOMMU_PRIV;

	return flags;
}

static int
kgsl_iommu_map(struct kgsl_pagetable *pt,
			struct kgsl_memdesc *memdesc)
{
<<<<<<< HEAD
	int ret = 0;
	unsigned int iommu_virt_addr;
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	size_t size = memdesc->size;
	unsigned int protflags;
	struct kgsl_device *device = pt->mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct scatterlist *sg_temp = NULL;

	BUG_ON(NULL == iommu_pt);

	iommu_virt_addr = memdesc->gpuaddr;

	
	protflags = IOMMU_READ;
	if (!(memdesc->flags & KGSL_MEMFLAGS_GPUREADONLY))
		protflags |= IOMMU_WRITE;

	if (memdesc->priv & KGSL_MEMDESC_PRIVILEGED)
		protflags |= IOMMU_PRIV;

	if (kgsl_memdesc_is_secured(memdesc)) {

		if (!kgsl_mmu_is_secured(pt->mmu))
			return -EINVAL;

		mutex_lock(&device->mutex);
		ret = kgsl_active_count_get(device);
		if (!ret) {
			ret = iommu_map_range(iommu_pt->domain, iommu_virt_addr,
				memdesc->sg, size, protflags);
			kgsl_active_count_put(device);
		}
		mutex_unlock(&device->mutex);
	} else {
		sg_temp = _create_sg_no_large_pages(memdesc);

		if (IS_ERR(sg_temp))
			return PTR_ERR(sg_temp);

		ret = iommu_map_range(iommu_pt->domain, iommu_virt_addr,
				sg_temp ? sg_temp : memdesc->sg,
				size, protflags);
	}

	if (ret)
		KGSL_CORE_ERR("iommu_map_range(%p, %x, %p, %zd, %x) err: %d\n",
			iommu_pt->domain, iommu_virt_addr,
			sg_temp != NULL ? sg_temp : memdesc->sg, size,
			protflags, ret);

	kgsl_free(sg_temp);

=======
	int ret;
	uint64_t addr = memdesc->gpuaddr;
	uint64_t size = memdesc->size;
	unsigned int flags = _get_protection_flags(memdesc);
	struct sg_table *sgt = NULL;

	if (memdesc->pages != NULL)
		sgt = kgsl_alloc_sgt_from_pages(memdesc);
	else
		sgt = memdesc->sgt;

	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	ret = _iommu_map_sg_sync_pc(pt, addr, memdesc, sgt->sgl,
				sgt->nents, flags);
>>>>>>> 0e91d2a... Nougat
	if (ret)
		goto done;

	ret = _iommu_add_guard_page(pt, memdesc, iommu_virt_addr + size,
								protflags);
	if (ret)
		
		iommu_unmap_range(iommu_pt->domain, iommu_virt_addr, size);

<<<<<<< HEAD

	if (ADRENO_FEATURE(adreno_dev, IOMMU_FLUSH_TLB_ON_MAP)
		&& !kgsl_memdesc_is_global(memdesc))
		kgsl_iommu_flush_tlb_pt_current(pt, memdesc);

	return ret;
=======
done:
	if (memdesc->pages != NULL)
		kgsl_free_sgt(sgt);

	return ret;
}

static int kgsl_iommu_map_offset(struct kgsl_pagetable *pt,
		uint64_t virtaddr, uint64_t virtoffset,
		struct kgsl_memdesc *memdesc, uint64_t physoffset,
		uint64_t size, uint64_t feature_flag)
{
	int pg_sz;
	unsigned int protflags = _get_protection_flags(memdesc);
	int ret;
	struct sg_table *sgt = NULL;

	pg_sz = (1 << kgsl_memdesc_get_align(memdesc));
	if (!IS_ALIGNED(virtaddr | virtoffset | physoffset | size, pg_sz))
		return -EINVAL;

	if (size == 0)
		return -EINVAL;

	if (memdesc->pages != NULL)
		sgt = kgsl_alloc_sgt_from_pages(memdesc);
	else
		sgt = memdesc->sgt;

	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	ret = _iommu_map_sg_offset_sync_pc(pt, virtaddr + virtoffset,
		memdesc, sgt->sgl, sgt->nents,
		physoffset, size, protflags);

	if (memdesc->pages != NULL)
		kgsl_free_sgt(sgt);

	return ret;
}

static void kgsl_iommu_clear_fsr(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context  *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	unsigned int sctlr_val;

	if (ctx->default_pt != NULL) {
		kgsl_iommu_enable_clk(mmu);
		KGSL_IOMMU_SET_CTX_REG(ctx, FSR, 0xffffffff);
		sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);
		sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_CFIE_SHIFT);
		KGSL_IOMMU_SET_CTX_REG(ctx, SCTLR, sctlr_val);
		wmb();
		kgsl_iommu_disable_clk(mmu);
	}
>>>>>>> 0e91d2a... Nougat
}

static void kgsl_iommu_pagefault_resume(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;

	if (atomic_read(&mmu->fault)) {
		kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_MAX_UNITS);
		for (i = 0; i < iommu->unit_count; i++) {
			struct kgsl_iommu_unit *iommu_unit =
						&iommu->iommu_units[i];
			for (j = 0; j < iommu_unit->dev_count; j++) {

				if ((!iommu_unit->dev[j].attached) ||
					(KGSL_IOMMU_CONTEXT_SECURE == j))
					continue;

				if (iommu_unit->dev[j].fault) {
					_iommu_lock(iommu);
					KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						RESUME, 1);
					KGSL_IOMMU_SET_CTX_REG(iommu,
						iommu_unit,
						iommu_unit->dev[j].ctx_id,
						FSR, 0);
					_iommu_unlock(iommu);
					iommu_unit->dev[j].fault = 0;
				}
			}
		}
		kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);
		atomic_set(&mmu->fault, 0);
=======
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];

	if (ctx->default_pt != NULL && ctx->fault) {
		KGSL_IOMMU_SET_CTX_REG(ctx, RESUME, 1);
		wmb();
		ctx->fault = 0;
>>>>>>> 0e91d2a... Nougat
	}
}


static void kgsl_iommu_stop(struct kgsl_mmu *mmu)
{
<<<<<<< HEAD
	
	kgsl_detach_pagetable_iommu_domain(mmu);

	kgsl_iommu_pagefault_resume(mmu);
}

static int kgsl_iommu_close(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i;

	if (mmu->priv_bank_table != NULL)
		kgsl_mmu_putpagetable(mmu->priv_bank_table);

	if (mmu->defaultpagetable != NULL)
		kgsl_mmu_putpagetable(mmu->defaultpagetable);

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_memdesc *reg_map = &iommu->iommu_units[i].reg_map;

		
		kgsl_remove_global_pt_entry(reg_map);

		if (reg_map->hostptr)
			iounmap(reg_map->hostptr);
		kgsl_free(reg_map->sg);
	}
	

	kgsl_remove_global_pt_entry(&iommu->sync_lock_desc);
	kgsl_free(iommu->sync_lock_desc.sg);
	memset(&iommu->sync_lock_desc, 0, sizeof(iommu->sync_lock_desc));
	iommu->sync_lock_vars = NULL;

	kfree(iommu);

	if (kgsl_guard_page != NULL) {
		__free_page(kgsl_guard_page);
		kgsl_guard_page = NULL;
	}

	kgsl_sharedmem_free(&kgsl_secure_guard_page_memdesc);

	return 0;
}

static phys_addr_t
kgsl_iommu_get_current_ptbase(struct kgsl_mmu *mmu)
{
	phys_addr_t pt_base;
	struct kgsl_iommu *iommu = mmu->priv;
=======
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	int i;

	if (!MMU_FEATURE(mmu, KGSL_MMU_RETENTION)) {
		for (i = 0; i < KGSL_IOMMU_CONTEXT_MAX; i++)
			_detach_context(&iommu->ctx[i]);
	}
}

static u64
kgsl_iommu_get_current_ttbr0(struct kgsl_mmu *mmu)
{
	u64 val;
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
>>>>>>> 0e91d2a... Nougat
	if (in_interrupt())
		return 0;
	
	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_MAX_UNITS);
	pt_base = KGSL_IOMMU_GET_CTX_REG_Q(iommu,
				(&iommu->iommu_units[0]),
				KGSL_IOMMU_CONTEXT_USER, TTBR0);
	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);
	return pt_base & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
}

<<<<<<< HEAD
static int kgsl_iommu_flush_pt(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	unsigned long wait_for_flush;
	unsigned int tlbflush_ctxt = KGSL_IOMMU_CONTEXT_USER;
	int i;
	int ret = 0;

	
	if (msm_soc_version_supports_iommu_v0()) {
		ret = adreno_spin_idle(mmu->device);
		if (ret)
			return ret;
	}
	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

	
	_iommu_lock(iommu);

	for (i = 0; i < iommu->unit_count; i++) {
		KGSL_IOMMU_SET_CTX_REG(iommu, (&iommu->iommu_units[i]),
			tlbflush_ctxt, TLBIALL, 1);
		mb();
		if (!msm_soc_version_supports_iommu_v0()) {
			wait_for_flush = jiffies + msecs_to_jiffies(2000);
			KGSL_IOMMU_SET_CTX_REG(iommu,
				(&iommu->iommu_units[i]),
				tlbflush_ctxt, TLBSYNC, 0);
			while (KGSL_IOMMU_GET_CTX_REG(iommu,
				(&iommu->iommu_units[i]),
				tlbflush_ctxt, TLBSTATUS) &
				(KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE)) {
				if (time_after(jiffies,
					wait_for_flush)) {
					KGSL_DRV_WARN(mmu->device,
					"Wait limit reached for IOMMU tlb flush\n");
					break;
				}
				cpu_relax();
			}
=======
static int kgsl_iommu_set_pt(struct kgsl_mmu *mmu, struct kgsl_pagetable *pt)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	uint64_t ttbr0, temp;
	unsigned int contextidr;
	unsigned long wait_for_flush;

	if ((pt != mmu->defaultpagetable) && !kgsl_mmu_is_perprocess(mmu))
		return 0;

	kgsl_iommu_enable_clk(mmu);

	ttbr0 = kgsl_mmu_pagetable_get_ttbr0(pt);
	contextidr = kgsl_mmu_pagetable_get_contextidr(pt);

	KGSL_IOMMU_SET_CTX_REG_Q(ctx, TTBR0, ttbr0);
	KGSL_IOMMU_SET_CTX_REG(ctx, CONTEXTIDR, contextidr);

	mb();
	temp = KGSL_IOMMU_GET_CTX_REG_Q(ctx, TTBR0);

	KGSL_IOMMU_SET_CTX_REG(ctx, TLBIALL, 1);
	
	mb();
	wait_for_flush = jiffies + msecs_to_jiffies(2000);
	KGSL_IOMMU_SET_CTX_REG(ctx, TLBSYNC, 0);
	while (KGSL_IOMMU_GET_CTX_REG(ctx, TLBSTATUS) &
		(KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE)) {
		if (time_after(jiffies, wait_for_flush)) {
			KGSL_DRV_WARN(KGSL_MMU_DEVICE(mmu),
			"Wait limit reached for IOMMU tlb flush\n");
			break;
		}
		cpu_relax();
	}

	kgsl_iommu_disable_clk(mmu);
	return 0;
}

static int kgsl_iommu_set_pf_policy(struct kgsl_mmu *mmu,
				unsigned long pf_policy)
{
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
	struct kgsl_iommu_context *ctx = &iommu->ctx[KGSL_IOMMU_CONTEXT_USER];
	struct kgsl_device *device = KGSL_MMU_DEVICE(mmu);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if ((adreno_dev->ft_pf_policy &
		BIT(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)) ==
		(pf_policy & BIT(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)))
		return 0;

	
	if (ctx->default_pt != NULL) {
		unsigned int sctlr_val;

		kgsl_iommu_enable_clk(mmu);

		sctlr_val = KGSL_IOMMU_GET_CTX_REG(ctx, SCTLR);

		if (test_bit(KGSL_FT_PAGEFAULT_GPUHALT_ENABLE, &pf_policy)) {
			sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_CFCFG_SHIFT);
			sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
		} else {
			sctlr_val &= ~(0x1 << KGSL_IOMMU_SCTLR_CFCFG_SHIFT);
			sctlr_val |= (0x1 << KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
>>>>>>> 0e91d2a... Nougat
		}
	}
	
	_iommu_unlock(iommu);

	
	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

	return 0;
}

static int kgsl_iommu_set_pt(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pt)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = mmu->priv;
	int temp;
	int i;
	int ret = 0;
	phys_addr_t pt_base;
	uint64_t pt_val;
=======
	struct kgsl_iommu *iommu = _IOMMU_PRIV(mmu);
>>>>>>> 0e91d2a... Nougat

	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

	pt_base = kgsl_iommu_get_pt_base_addr(mmu, pt);

	ret = adreno_spin_idle(mmu->device);
	if (ret)
		return ret;

	
	_iommu_lock(iommu);

	for (i = 0; i < iommu->unit_count; i++) {
		pt_val = kgsl_iommu_get_default_ttbr0(mmu, i,
					KGSL_IOMMU_CONTEXT_USER);

		pt_base &= KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		pt_val |= pt_base;
		KGSL_IOMMU_SET_CTX_REG_Q(iommu,
				(&iommu->iommu_units[i]),
				KGSL_IOMMU_CONTEXT_USER, TTBR0, pt_val);

		mb();
		temp = KGSL_IOMMU_GET_CTX_REG_Q(iommu,
			(&iommu->iommu_units[i]),
			KGSL_IOMMU_CONTEXT_USER, TTBR0);
	}
	
	_iommu_unlock(iommu);

<<<<<<< HEAD
	kgsl_iommu_flush_pt(mmu);

	
	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

	return ret;
=======
	rb_link_node(&new->node, parent, node);
	rb_insert_color(&new->node, &pt->rbtree);

	return 0;
}

static uint64_t _get_unmapped_area(struct kgsl_pagetable *pagetable,
		uint64_t bottom, uint64_t top, uint64_t size,
		uint64_t align)
{
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node *node = rb_first(&pt->rbtree);
	uint64_t start;

	bottom = ALIGN(bottom, align);
	start = bottom;

	while (node != NULL) {
		uint64_t gap;
		struct kgsl_iommu_addr_entry *entry = rb_entry(node,
			struct kgsl_iommu_addr_entry, node);

		if (entry->base < bottom) {
			if (entry->base + entry->size > bottom)
				start = ALIGN(entry->base + entry->size, align);
			node = rb_next(node);
			continue;
		}

		
		if (entry->base >= top)
			break;

		
		if (start < entry->base) {
			gap = entry->base - start;

			if (gap >= size)
				return start;
		}

		
		if (entry->base + entry->size >= top)
			return (uint64_t) -ENOMEM;

		
		start = ALIGN(entry->base + entry->size, align);
		node = rb_next(node);
	}

	if (start + size <= top)
		return start;

	return (uint64_t) -ENOMEM;
>>>>>>> 0e91d2a... Nougat
}

static unsigned int kgsl_iommu_get_reg_gpuaddr(struct kgsl_mmu *mmu,
					int iommu_unit, int ctx_id, int reg)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = mmu->priv;

	if (KGSL_IOMMU_GLOBAL_BASE == reg)
		return iommu->iommu_units[iommu_unit].reg_map.gpuaddr;
=======
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node *node = rb_last(&pt->rbtree);
	uint64_t end = top;
	uint64_t mask = ~(align - 1);
	struct kgsl_iommu_addr_entry *entry;

	
	bottom = ALIGN(bottom, align);

	
	if (size > (top - bottom))
		return -ENOMEM;

	
	for (node = rb_last(&pt->rbtree); node != NULL; node = rb_prev(node)) {
		entry = rb_entry(node, struct kgsl_iommu_addr_entry, node);
		if (entry->base < top)
			break;
	}

	while (node != NULL) {
		uint64_t offset;

		entry = rb_entry(node, struct kgsl_iommu_addr_entry, node);

		
		if ((entry->base + entry->size) < bottom)
			break;

		
		offset = ALIGN(entry->base + entry->size, align);


		if ((end > size) && (offset < end)) {
			uint64_t chunk = (end - size) & mask;

			if (chunk >= offset)
				return chunk;
		}


		if (entry->base < bottom)
			return (uint64_t) -ENOMEM;

		
		end = entry->base;

		
		node = rb_prev(node);
	}

	
	if ((end > size) && (((end - size) & mask) >= bottom))
		return (end - size) & mask;
>>>>>>> 0e91d2a... Nougat

	if (iommu->iommu_reg_list[reg].ctx_reg)
		return iommu->iommu_units[iommu_unit].reg_map.gpuaddr +
			iommu->iommu_reg_list[reg].reg_offset +
			(ctx_id << KGSL_IOMMU_CTX_SHIFT) + iommu->ctx_offset;
	else
		return iommu->iommu_units[iommu_unit].reg_map.gpuaddr +
			iommu->iommu_reg_list[reg].reg_offset;
}
static int kgsl_iommu_hw_halt_supported(struct kgsl_mmu *mmu, int iommu_unit)
{
<<<<<<< HEAD
	struct kgsl_iommu *iommu = mmu->priv;
	return iommu->iommu_units[iommu_unit].iommu_halt_enable;
}

static int kgsl_iommu_get_num_iommu_units(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	return iommu->unit_count;
=======
	uint64_t addr;

	
	BUG_ON(end <= start);

	spin_lock(&pagetable->lock);
	addr = _get_unmapped_area_topdown(pagetable,
			start, end, size, alignment);
	spin_unlock(&pagetable->lock);
	return addr;
}

#define ADDR_IN_GLOBAL(_a) \
	(((_a) >= KGSL_IOMMU_GLOBAL_MEM_BASE) && \
	 ((_a) < (KGSL_IOMMU_GLOBAL_MEM_BASE + KGSL_IOMMU_GLOBAL_MEM_SIZE)))

static int kgsl_iommu_set_svm_region(struct kgsl_pagetable *pagetable,
		uint64_t gpuaddr, uint64_t size)
{
	int ret = -ENOMEM;
	struct kgsl_iommu_pt *pt = pagetable->priv;
	struct rb_node *node;

	
	if (ADDR_IN_GLOBAL(gpuaddr) || ADDR_IN_GLOBAL(gpuaddr + size))
		return -ENOMEM;

	spin_lock(&pagetable->lock);
	node = pt->rbtree.rb_node;

	while (node != NULL) {
		uint64_t start, end;
		struct kgsl_iommu_addr_entry *entry = rb_entry(node,
			struct kgsl_iommu_addr_entry, node);

		start = entry->base;
		end = entry->base + entry->size;

		if (gpuaddr  + size <= start)
			node = node->rb_left;
		else if (end <= gpuaddr)
			node = node->rb_right;
		else
			goto out;
	}

	ret = _insert_gpuaddr(pagetable, gpuaddr, size);
out:
	spin_unlock(&pagetable->lock);
	return ret;
>>>>>>> 0e91d2a... Nougat
}

static int kgsl_iommu_set_pf_policy(struct kgsl_mmu *mmu,
				unsigned int pf_policy)
{
	int i, j;
	struct kgsl_iommu *iommu = mmu->priv;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(mmu->device);
	int ret = 0;
	unsigned int sctlr_val;

	if ((adreno_dev->ft_pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE) ==
		(pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE))
		return ret;
	if (msm_soc_version_supports_iommu_v0())
		return ret;

	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

	
	ret = mmu->device->ftbl->idle(mmu->device);
	if (ret) {
		kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);
		return ret;
	}

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {

			if ((!iommu_unit->dev[j].attached) ||
				(KGSL_IOMMU_CONTEXT_SECURE == j))
				continue;

			sctlr_val = KGSL_IOMMU_GET_CTX_REG(iommu,
					iommu_unit,
					iommu_unit->dev[j].ctx_id,
					SCTLR);
			if (pf_policy & KGSL_FT_PAGEFAULT_GPUHALT_ENABLE)
				sctlr_val &= ~(0x1 <<
					KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
			else
				sctlr_val |= (0x1 <<
					KGSL_IOMMU_SCTLR_HUPCF_SHIFT);
			KGSL_IOMMU_SET_CTX_REG(iommu,
					iommu_unit,
					iommu_unit->dev[j].ctx_id,
					SCTLR, sctlr_val);
		}
	}

	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);
	return ret;
}

static void kgsl_iommu_set_pagefault(struct kgsl_mmu *mmu)
{
	int i, j;
	struct kgsl_iommu *iommu = mmu->priv;
	unsigned int fsr;

	
	if (atomic_read(&mmu->fault))
		return;

	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_MAX_UNITS);

	
	for (i = 0; i < iommu->unit_count; i++) {
		for (j = 0; j < iommu->iommu_units[i].dev_count; j++) {

			if ((!iommu->iommu_units[i].dev[j].attached) ||
				(KGSL_IOMMU_CONTEXT_SECURE == j))
				continue;

			fsr = KGSL_IOMMU_GET_CTX_REG(iommu,
				(&(iommu->iommu_units[i])),
				iommu->iommu_units[i].dev[j].ctx_id, FSR);
			if (fsr) {
				uint64_t far =
					KGSL_IOMMU_GET_CTX_REG_Q(iommu,
					(&(iommu->iommu_units[i])),
					iommu->iommu_units[i].dev[j].ctx_id,
					FAR);
				kgsl_iommu_fault_handler(NULL,
				iommu->iommu_units[i].dev[j].dev, far, 0, NULL);
				break;
			}
		}
	}

	kgsl_iommu_disable_clk(mmu, KGSL_IOMMU_MAX_UNITS);
}

struct kgsl_protected_registers *kgsl_iommu_get_prot_regs(struct kgsl_mmu *mmu)
{
	static struct kgsl_protected_registers iommuv1_regs = { 0x4000, 14 };
	static struct kgsl_protected_registers iommuv2_regs;

	if (msm_soc_version_supports_iommu_v0())
		return NULL;
	if (kgsl_msm_supports_iommu_v2()) {

		struct kgsl_iommu *iommu = mmu->priv;

		
		iommuv2_regs.base = iommu->iommu_units[0].ahb_base >> 2;
		iommuv2_regs.range = 10;
		return &iommuv2_regs;
	}
	else
		return &iommuv1_regs;
}

static const struct {
	int id;
	char *name;
} kgsl_iommu_cbs[] = {
	{ KGSL_IOMMU_CONTEXT_USER, "gfx3d_user", },
	{ KGSL_IOMMU_CONTEXT_SECURE, "gfx3d_secure" },
};

static int _kgsl_iommu_cb_probe(struct kgsl_device *device,
		struct kgsl_iommu *iommu, struct device_node *node)
{
	struct platform_device *pdev = of_find_device_by_node(node);
	struct kgsl_iommu_context *ctx = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(kgsl_iommu_cbs); i++) {
		if (!strcmp(node->name, kgsl_iommu_cbs[i].name)) {
			int id = kgsl_iommu_cbs[i].id;

			ctx = &iommu->ctx[id];
			ctx->id = id;
			ctx->cb_num = -1;
			ctx->name = kgsl_iommu_cbs[i].name;

			break;
		}
	}

	if (ctx == NULL) {
		KGSL_CORE_ERR("dt: Unknown context label %s\n", node->name);
		return -EINVAL;
	}

	if (ctx->id == KGSL_IOMMU_CONTEXT_SECURE)
		device->mmu.secured = true;

	
	if (of_property_read_u32(node, "qcom,gpu-offset", &ctx->gpu_offset))
		ctx->gpu_offset = UINT_MAX;

	ctx->kgsldev = device;

	
	if (of_find_property(node, "iommus", NULL)) {
		ctx->dev = &pdev->dev;
	} else {
		ctx->dev = kgsl_mmu_get_ctx(ctx->name);

		if (IS_ERR(ctx->dev))
			return PTR_ERR(ctx->dev);
	}

	return 0;
}

static const struct {
	char *feature;
	int bit;
} kgsl_iommu_features[] = {
	{ "qcom,retention", KGSL_MMU_RETENTION },
	{ "qcom,global_pt", KGSL_MMU_GLOBAL_PAGETABLE },
	{ "qcom,hyp_secure_alloc", KGSL_MMU_HYP_SECURE_ALLOC },
	{ "qcom,force-32bit", KGSL_MMU_FORCE_32BIT },
	{ "qcom,coherent-htw", KGSL_MMU_COHERENT_HTW },
};

static int _kgsl_iommu_probe(struct kgsl_device *device,
		struct device_node *node)
{
	const char *cname;
	struct property *prop;
	u32 reg_val[2];
	int i = 0;
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	struct device_node *child;
	struct platform_device *pdev = of_find_device_by_node(node);

	memset(iommu, 0, sizeof(*iommu));

	if (of_device_is_compatible(node, "qcom,kgsl-smmu-v1"))
		iommu->version = 1;
	else
		iommu->version = 2;

	if (of_property_read_u32_array(node, "reg", reg_val, 2)) {
		KGSL_CORE_ERR("dt: Unable to read KGSL IOMMU register range\n");
		return -EINVAL;
	}
	iommu->regstart = reg_val[0];
	iommu->regsize = reg_val[1];

	
	if (of_property_read_u32_array(node, "qcom,protect", reg_val, 2)) {
		KGSL_CORE_ERR("dt: no iommu protection range specified\n");
		return -EINVAL;
	}
	iommu->protect.base = reg_val[0] / sizeof(u32);
	iommu->protect.range = ilog2(reg_val[1] / sizeof(u32));

	of_property_for_each_string(node, "clock-names", prop, cname) {
		struct clk *c = devm_clk_get(&pdev->dev, cname);

		if (IS_ERR(c)) {
			KGSL_CORE_ERR("dt: Couldn't get clock: %s\n", cname);
			return -ENODEV;
		}
		if (i >= KGSL_IOMMU_MAX_CLKS) {
			KGSL_CORE_ERR("dt: too many clocks defined.\n");
			return -EINVAL;
		}

		iommu->clks[i] = c;
		++i;
	}

	for (i = 0; i < ARRAY_SIZE(kgsl_iommu_features); i++) {
		if (of_property_read_bool(node, kgsl_iommu_features[i].feature))
			device->mmu.features |= kgsl_iommu_features[i].bit;
	}

	if (of_property_read_u32(node, "qcom,micro-mmu-control",
		&iommu->micro_mmu_ctrl))
		iommu->micro_mmu_ctrl = UINT_MAX;

	if (of_property_read_u32(node, "qcom,secure_align_mask",
		&device->mmu.secure_align_mask))
		device->mmu.secure_align_mask = 0xfff;

	
	of_platform_populate(node, NULL, NULL, &pdev->dev);

	for_each_child_of_node(node, child) {
		int ret;

		if (!of_device_is_compatible(child, "qcom,smmu-kgsl-cb"))
			continue;

		ret = _kgsl_iommu_cb_probe(device, iommu, child);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct {
	char *compat;
	int (*probe)(struct kgsl_device *device, struct device_node *node);
} kgsl_dt_devices[] = {
	{ "qcom,kgsl-smmu-v1", _kgsl_iommu_probe },
	{ "qcom,kgsl-smmu-v2", _kgsl_iommu_probe },
};

static int kgsl_iommu_probe(struct kgsl_device *device)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(kgsl_dt_devices); i++) {
		struct device_node *node;

		node = of_find_compatible_node(device->pdev->dev.of_node,
			NULL, kgsl_dt_devices[i].compat);

		if (node != NULL)
			return kgsl_dt_devices[i].probe(device, node);
	}

	return -ENODEV;
}

struct kgsl_mmu_ops kgsl_iommu_ops = {
	.mmu_init = kgsl_iommu_init,
	.mmu_close = kgsl_iommu_close,
	.mmu_start = kgsl_iommu_start,
	.mmu_stop = kgsl_iommu_stop,
	.mmu_set_pt = kgsl_iommu_set_pt,
	.mmu_pagefault_resume = kgsl_iommu_pagefault_resume,
	.mmu_get_current_ptbase = kgsl_iommu_get_current_ptbase,
	.mmu_enable_clk = kgsl_iommu_enable_clk,
	.mmu_disable_clk = kgsl_iommu_disable_clk,
	.mmu_get_default_ttbr0 = kgsl_iommu_get_default_ttbr0,
	.mmu_get_reg_gpuaddr = kgsl_iommu_get_reg_gpuaddr,
	.mmu_get_reg_ahbaddr = kgsl_iommu_get_reg_ahbaddr,
<<<<<<< HEAD
	.mmu_get_num_iommu_units = kgsl_iommu_get_num_iommu_units,
	.mmu_pt_equal = kgsl_iommu_pt_equal,
	.mmu_get_pt_base_addr = kgsl_iommu_get_pt_base_addr,
	.mmu_hw_halt_supported = kgsl_iommu_hw_halt_supported,
	
	.mmu_sync_lock = kgsl_iommu_sync_lock,
	.mmu_sync_unlock = kgsl_iommu_sync_unlock,
	.mmu_set_pf_policy = kgsl_iommu_set_pf_policy,
	.mmu_set_pagefault = kgsl_iommu_set_pagefault,
	.mmu_get_prot_regs = kgsl_iommu_get_prot_regs
=======
	.mmu_pt_equal = kgsl_iommu_pt_equal,
	.mmu_set_pf_policy = kgsl_iommu_set_pf_policy,
	.mmu_pagefault_resume = kgsl_iommu_pagefault_resume,
	.mmu_get_prot_regs = kgsl_iommu_get_prot_regs,
	.mmu_init_pt = kgsl_iommu_init_pt,
	.mmu_add_global = kgsl_iommu_add_global,
	.mmu_remove_global = kgsl_iommu_remove_global,
	.mmu_getpagetable = kgsl_iommu_getpagetable,
	.mmu_get_qdss_global_entry = kgsl_iommu_get_qdss_global_entry,
	.probe = kgsl_iommu_probe,
>>>>>>> 0e91d2a... Nougat
};

struct kgsl_mmu_pt_ops iommu_pt_ops = {
	.mmu_map = kgsl_iommu_map,
	.mmu_unmap = kgsl_iommu_unmap,
	.mmu_create_pagetable = kgsl_iommu_create_pagetable,
	.mmu_create_secure_pagetable = kgsl_iommu_create_secure_pagetable,
	.mmu_destroy_pagetable = kgsl_iommu_destroy_pagetable,
<<<<<<< HEAD
	.get_ptbase = kgsl_iommu_get_ptbase,
=======
	.get_ttbr0 = kgsl_iommu_get_ttbr0,
	.get_contextidr = kgsl_iommu_get_contextidr,
	.get_gpuaddr = kgsl_iommu_get_gpuaddr,
	.put_gpuaddr = kgsl_iommu_put_gpuaddr,
	.set_svm_region = kgsl_iommu_set_svm_region,
	.find_svm_region = kgsl_iommu_find_svm_region,
	.svm_range = kgsl_iommu_svm_range,
	.addr_in_range = kgsl_iommu_addr_in_range,
	.mmu_map_offset = kgsl_iommu_map_offset,
	.mmu_unmap_offset = kgsl_iommu_unmap_offset,
>>>>>>> 0e91d2a... Nougat
};
