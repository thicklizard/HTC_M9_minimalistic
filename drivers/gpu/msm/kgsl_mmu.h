/* Copyright (c) 2002,2007-2016, The Linux Foundation. All rights reserved.
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
#ifndef __KGSL_MMU_H
#define __KGSL_MMU_H

#include <linux/qcom_iommu.h>
#include "kgsl_iommu.h"
<<<<<<< HEAD
/*
 * These defines control the address range for allocations that
 * are mapped into all pagetables.
 */
#define KGSL_GLOBAL_PT_SIZE	SZ_4M
#define KGSL_MMU_GLOBAL_MEM_BASE	0xf8000000

/* Virtual memory range to map non-kgsl allocations */
#define KGSL_MMU_MAPPED_MEM_BASE	KGSL_SVM_UPPER_BOUND
#define KGSL_MMU_MAPPED_MEM_SIZE	(KGSL_MMU_GLOBAL_MEM_BASE -	\
					KGSL_MMU_MAPPED_MEM_BASE -	\
					SZ_1M)

/*
 * These defines control the address range for allocations that
 * are mapped into secure pagetable.
 */
#define KGSL_IOMMU_SECURE_MEM_BASE     0xe8000000
#define KGSL_IOMMU_SECURE_MEM_SIZE     SZ_256M

/* defconfig option for disabling per process pagetables */
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
#define KGSL_MMU_USE_PER_PROCESS_PT true
#else
#define KGSL_MMU_USE_PER_PROCESS_PT false
#endif

/* Identifier for the global page table */
/* Per process page tables will probably pass in the thread group
   as an identifier */
=======

>>>>>>> 0e91d2a... Nougat
#define KGSL_MMU_GLOBAL_PT 0
#define KGSL_MMU_SECURE_PT 1
#define KGSL_MMU_PRIV_PT   0xFFFFFFFF

#define MMU_DEFAULT_TTBR0(_d) \
	(kgsl_mmu_pagetable_get_ttbr0((_d)->mmu.defaultpagetable))

#define MMU_DEFAULT_CONTEXTIDR(_d) \
	(kgsl_mmu_pagetable_get_contextidr((_d)->mmu.defaultpagetable))

struct kgsl_device;

enum kgsl_mmutype {
	KGSL_MMU_TYPE_IOMMU = 0,
	KGSL_MMU_TYPE_NONE
};

struct kgsl_pagetable {
	spinlock_t lock;
	struct kref refcount;
	struct gen_pool *pool;
	struct list_head list;
	unsigned int name;
	struct kobject *kobj;

	struct {
		unsigned int entries;
		unsigned int mapped;
		unsigned int max_mapped;
	} stats;
	const struct kgsl_mmu_pt_ops *pt_ops;
	uint64_t fault_addr;
	void *priv;
	struct kgsl_mmu *mmu;
<<<<<<< HEAD
	unsigned long *mem_bitmap;
=======
>>>>>>> 0e91d2a... Nougat
};

struct kgsl_mmu;

struct kgsl_mmu_ops {
	int (*probe)(struct kgsl_device *device);
	int (*mmu_init) (struct kgsl_mmu *mmu);
	void (*mmu_close)(struct kgsl_mmu *mmu);
	int (*mmu_start) (struct kgsl_mmu *mmu);
	void (*mmu_stop) (struct kgsl_mmu *mmu);
	int (*mmu_set_pt) (struct kgsl_mmu *mmu, struct kgsl_pagetable *pt);
	phys_addr_t (*mmu_get_current_ptbase)
			(struct kgsl_mmu *mmu);
	void (*mmu_pagefault_resume)
			(struct kgsl_mmu *mmu);
	void (*mmu_enable_clk)
		(struct kgsl_mmu *mmu, int unit);
	void (*mmu_disable_clk)
		(struct kgsl_mmu *mmu, int unit);
	uint64_t (*mmu_get_default_ttbr0)(struct kgsl_mmu *mmu,
				unsigned int unit_id,
				enum kgsl_iommu_context_id ctx_id);
	unsigned int (*mmu_get_reg_gpuaddr)(struct kgsl_mmu *mmu,
			int iommu_unit_num, int ctx_id, int reg);
	unsigned int (*mmu_get_reg_ahbaddr)(struct kgsl_mmu *mmu,
<<<<<<< HEAD
			int iommu_unit_num, int ctx_id,
			enum kgsl_iommu_reg_map reg);
	int (*mmu_get_num_iommu_units)(struct kgsl_mmu *mmu);
	int (*mmu_pt_equal) (struct kgsl_mmu *mmu,
			struct kgsl_pagetable *pt,
			phys_addr_t pt_base);
	phys_addr_t (*mmu_get_pt_base_addr)
			(struct kgsl_mmu *mmu,
			struct kgsl_pagetable *pt);
	unsigned int (*mmu_sync_lock)
			(struct kgsl_mmu *mmu, unsigned int *cmds);
	unsigned int (*mmu_sync_unlock)
			(struct kgsl_mmu *mmu, unsigned int *cmds);
	int (*mmu_hw_halt_supported)(struct kgsl_mmu *mmu, int iommu_unit_num);
	int (*mmu_set_pf_policy)(struct kgsl_mmu *mmu, unsigned int pf_policy);
	void (*mmu_set_pagefault)(struct kgsl_mmu *mmu);
	struct kgsl_protected_registers *(*mmu_get_prot_regs)
			(struct kgsl_mmu *mmu);
=======
			int ctx_id, unsigned int reg);
	bool (*mmu_pt_equal)(struct kgsl_mmu *mmu,
			struct kgsl_pagetable *pt, u64 ttbr0);
	int (*mmu_set_pf_policy)(struct kgsl_mmu *mmu, unsigned long pf_policy);
	struct kgsl_protected_registers *(*mmu_get_prot_regs)
			(struct kgsl_mmu *mmu);
	int (*mmu_init_pt)(struct kgsl_mmu *mmu, struct kgsl_pagetable *);
	void (*mmu_add_global)(struct kgsl_mmu *mmu,
			struct kgsl_memdesc *memdesc);
	void (*mmu_remove_global)(struct kgsl_mmu *mmu,
			struct kgsl_memdesc *memdesc);
	struct kgsl_pagetable * (*mmu_getpagetable)(struct kgsl_mmu *mmu,
			unsigned long name);
	struct kgsl_memdesc* (*mmu_get_qdss_global_entry)(void);
>>>>>>> 0e91d2a... Nougat
};

struct kgsl_mmu_pt_ops {
	int (*mmu_map) (struct kgsl_pagetable *pt,
			struct kgsl_memdesc *memdesc);
	int (*mmu_unmap) (struct kgsl_pagetable *pt,
			struct kgsl_memdesc *memdesc);
	void *(*mmu_create_pagetable) (void);
	void *(*mmu_create_secure_pagetable) (void);
	void (*mmu_destroy_pagetable) (struct kgsl_pagetable *);
<<<<<<< HEAD
	phys_addr_t (*get_ptbase) (struct kgsl_pagetable *);
};

#define KGSL_MMU_FLAGS_IOMMU_SYNC BIT(31)

struct kgsl_mmu {
	uint32_t      flags;
	struct kgsl_device     *device;
	struct kgsl_memdesc    setstate_memory;
	/* current page table object being used by device mmu */
	struct kgsl_pagetable  *defaultpagetable;
	/* secure global pagetable device mmu */
	struct kgsl_pagetable  *securepagetable;
	/* pagetable object used for priv bank of IOMMU */
	struct kgsl_pagetable  *priv_bank_table;
	const struct kgsl_mmu_ops *mmu_ops;
	void *priv;
	atomic_t fault;
	unsigned long pt_base;
	unsigned long pt_size;
	bool secured;
};

extern struct kgsl_mmu_ops kgsl_iommu_ops;
extern struct kgsl_mmu_pt_ops iommu_pt_ops;
=======
	u64 (*get_ttbr0)(struct kgsl_pagetable *);
	u32 (*get_contextidr)(struct kgsl_pagetable *);
	int (*get_gpuaddr)(struct kgsl_pagetable *, struct kgsl_memdesc *);
	void (*put_gpuaddr)(struct kgsl_pagetable *, struct kgsl_memdesc *);
	uint64_t (*find_svm_region)(struct kgsl_pagetable *, uint64_t, uint64_t,
		uint64_t, uint64_t);
	int (*set_svm_region)(struct kgsl_pagetable *, uint64_t, uint64_t);
	int (*svm_range)(struct kgsl_pagetable *, uint64_t *, uint64_t *,
			uint64_t);
	bool (*addr_in_range)(struct kgsl_pagetable *pagetable, uint64_t);
	int (*mmu_map_offset)(struct kgsl_pagetable *pt,
			uint64_t virtaddr, uint64_t virtoffset,
			struct kgsl_memdesc *memdesc, uint64_t physoffset,
			uint64_t size, uint64_t flags);
	int (*mmu_unmap_offset)(struct kgsl_pagetable *pt,
			struct kgsl_memdesc *memdesc, uint64_t addr,
			uint64_t offset, uint64_t size);
};

#define MMU_FEATURE(_mmu, _bit) \
	((_mmu)->features & (_bit))

#define KGSL_MMU_RETENTION  BIT(1)
#define KGSL_MMU_FLUSH_TLB_ON_MAP BIT(2)
#define KGSL_MMU_GLOBAL_PAGETABLE BIT(3)
#define KGSL_MMU_HYP_SECURE_ALLOC BIT(4)
#define KGSL_MMU_FORCE_32BIT BIT(5)
#define KGSL_MMU_64BIT BIT(6)
#define KGSL_MMU_COHERENT_HTW BIT(7)
#define KGSL_MMU_PAGED BIT(8)
#define KGSL_MMU_NEED_GUARD_PAGE BIT(9)

struct kgsl_mmu {
	unsigned long flags;
	enum kgsl_mmutype type;
	struct kgsl_pagetable *defaultpagetable;
	struct kgsl_pagetable *securepagetable;
	const struct kgsl_mmu_ops *mmu_ops;
	bool secured;
	unsigned long features;
	unsigned int secure_align_mask;
	union {
		struct kgsl_iommu iommu;
	} priv;
};

#define KGSL_IOMMU_PRIV(_device) (&((_device)->mmu.priv.iommu))
>>>>>>> 0e91d2a... Nougat

extern struct kgsl_mmu_ops kgsl_iommu_ops;

int kgsl_mmu_probe(struct kgsl_device *device, char *name);
int kgsl_mmu_start(struct kgsl_device *device);
struct kgsl_pagetable *kgsl_mmu_getpagetable_ptbase(struct kgsl_mmu *,
						phys_addr_t ptbase);

void kgsl_add_global_secure_entry(struct kgsl_device *device,
					struct kgsl_memdesc *memdesc);
void kgsl_mmu_putpagetable(struct kgsl_pagetable *pagetable);
<<<<<<< HEAD
int kgsl_mmu_init(struct kgsl_device *device);
int kgsl_mmu_start(struct kgsl_device *device);
int kgsl_mmu_close(struct kgsl_device *device);
int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 struct kgsl_memdesc *memdesc);
=======

>>>>>>> 0e91d2a... Nougat
int kgsl_mmu_get_gpuaddr(struct kgsl_pagetable *pagetable,
		 struct kgsl_memdesc *memdesc);
int kgsl_mmu_map(struct kgsl_pagetable *pagetable,
		 struct kgsl_memdesc *memdesc);
int kgsl_mmu_unmap(struct kgsl_pagetable *pagetable,
		    struct kgsl_memdesc *memdesc);
void kgsl_mmu_put_gpuaddr(struct kgsl_pagetable *pagetable,
		 struct kgsl_memdesc *memdesc);
unsigned int kgsl_virtaddr_to_physaddr(void *virtaddr);
<<<<<<< HEAD
int kgsl_mmu_get_ptname_from_ptbase(struct kgsl_mmu *mmu,
					phys_addr_t pt_base);
unsigned int kgsl_mmu_log_fault_addr(struct kgsl_mmu *mmu,
			phys_addr_t pt_base, unsigned int addr);
int kgsl_mmu_pt_get_flags(struct kgsl_pagetable *pt,
			enum kgsl_deviceid id);
int kgsl_mmu_enabled(void);
void kgsl_mmu_set_mmutype(char *mmutype);
enum kgsl_mmutype kgsl_mmu_get_mmutype(void);
int kgsl_mmu_gpuaddr_in_range(struct kgsl_pagetable *pt, unsigned int gpuaddr);
=======
unsigned int kgsl_mmu_log_fault_addr(struct kgsl_mmu *mmu,
		u64 ttbr0, uint64_t addr);
enum kgsl_mmutype kgsl_mmu_get_mmutype(struct kgsl_device *device);
bool kgsl_mmu_gpuaddr_in_range(struct kgsl_pagetable *pt, uint64_t gpuaddr);

int kgsl_mmu_get_region(struct kgsl_pagetable *pagetable,
		uint64_t gpuaddr, uint64_t size);

int kgsl_mmu_find_region(struct kgsl_pagetable *pagetable,
		uint64_t region_start, uint64_t region_end,
		uint64_t *gpuaddr, uint64_t size, unsigned int align);
>>>>>>> 0e91d2a... Nougat

void kgsl_mmu_add_global(struct kgsl_device *device,
	struct kgsl_memdesc *memdesc);
<<<<<<< HEAD
void kgsl_remove_global_pt_entry(struct kgsl_memdesc *memdesc);

struct kgsl_memdesc *kgsl_search_global_pt_entries(unsigned int gpuaddr,
		unsigned int size);

void kgsl_mmu_detach_pagetable(struct kgsl_pagetable *pagetable);
/*
 * Static inline functions of MMU that simply call the SMMU specific
 * function using a function pointer. These functions can be thought
 * of as wrappers around the actual function
 */
=======
void kgsl_mmu_remove_global(struct kgsl_device *device,
		struct kgsl_memdesc *memdesc);

struct kgsl_pagetable *kgsl_mmu_get_pt_from_ptname(struct kgsl_mmu *mmu,
							int ptname);
void kgsl_mmu_close(struct kgsl_device *device);

uint64_t kgsl_mmu_find_svm_region(struct kgsl_pagetable *pagetable,
		uint64_t start, uint64_t end, uint64_t size,
		uint64_t alignment);

int kgsl_mmu_set_svm_region(struct kgsl_pagetable *pagetable, uint64_t gpuaddr,
		uint64_t size);

void kgsl_mmu_detach_pagetable(struct kgsl_pagetable *pagetable);

int kgsl_mmu_svm_range(struct kgsl_pagetable *pagetable,
		uint64_t *lo, uint64_t *hi, uint64_t memflags);

struct kgsl_pagetable *kgsl_get_pagetable(unsigned long name);

struct kgsl_pagetable *
kgsl_mmu_createpagetableobject(struct kgsl_mmu *mmu, unsigned int name);

int kgsl_mmu_map_offset(struct kgsl_pagetable *pagetable,
		uint64_t virtaddr, uint64_t virtoffset,
		struct kgsl_memdesc *memdesc, uint64_t physoffset,
		uint64_t size, uint64_t flags);
int kgsl_mmu_unmap_offset(struct kgsl_pagetable *pagetable,
		struct kgsl_memdesc *memdesc, uint64_t addr, uint64_t offset,
		uint64_t size);

struct kgsl_memdesc *kgsl_mmu_get_qdss_global_entry(struct kgsl_device *device);

>>>>>>> 0e91d2a... Nougat

static inline phys_addr_t kgsl_mmu_get_current_ptbase(struct kgsl_mmu *mmu)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_get_current_ptbase)
		return mmu->mmu_ops->mmu_get_current_ptbase(mmu);
	else
		return 0;
}

static inline struct kgsl_pagetable *kgsl_mmu_getpagetable(struct kgsl_mmu *mmu,
		unsigned long name)
{
	if (MMU_OP_VALID(mmu, mmu_getpagetable))
		return mmu->mmu_ops->mmu_getpagetable(mmu, name);

	return NULL;
}

static inline int kgsl_mmu_set_pt(struct kgsl_mmu *mmu,
					struct kgsl_pagetable *pagetable)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_set_pt)
		return mmu->mmu_ops->mmu_set_pt(mmu, pagetable);

	return 0;
}

static inline void kgsl_mmu_stop(struct kgsl_mmu *mmu)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_stop)
		mmu->mmu_ops->mmu_stop(mmu);
}

<<<<<<< HEAD
static inline int kgsl_mmu_pt_equal(struct kgsl_mmu *mmu,
			struct kgsl_pagetable *pt,
			phys_addr_t pt_base)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_pt_equal)
		return mmu->mmu_ops->mmu_pt_equal(mmu, pt, pt_base);
	else
		return 1;
}

static inline phys_addr_t kgsl_mmu_get_pt_base_addr(struct kgsl_mmu *mmu,
						struct kgsl_pagetable *pt)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_get_pt_base_addr)
		return mmu->mmu_ops->mmu_get_pt_base_addr(mmu, pt);
	else
		return 0;
}

static inline phys_addr_t kgsl_mmu_get_default_ttbr0(struct kgsl_mmu *mmu,
					unsigned int unit_id,
					enum kgsl_iommu_context_id ctx_id)
=======
static inline bool kgsl_mmu_pt_equal(struct kgsl_mmu *mmu,
			struct kgsl_pagetable *pt, u64 ttbr0)
{
	if (MMU_OP_VALID(mmu, mmu_pt_equal))
		return mmu->mmu_ops->mmu_pt_equal(mmu, pt, ttbr0);

	return false;
}

static inline void kgsl_mmu_enable_clk(struct kgsl_mmu *mmu)
>>>>>>> 0e91d2a... Nougat
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_get_default_ttbr0)
		return mmu->mmu_ops->mmu_get_default_ttbr0(mmu, unit_id,
							ctx_id);
	else
		return 0;
}

static inline void kgsl_mmu_enable_clk(struct kgsl_mmu *mmu, int unit)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_enable_clk)
		mmu->mmu_ops->mmu_enable_clk(mmu, unit);
	else
		return;
}

static inline void kgsl_mmu_disable_clk(struct kgsl_mmu *mmu, int unit)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_disable_clk)
		mmu->mmu_ops->mmu_disable_clk(mmu, unit);
}

static inline unsigned int kgsl_mmu_get_reg_gpuaddr(struct kgsl_mmu *mmu,
							int iommu_unit_num,
							int ctx_id, int reg)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_get_reg_gpuaddr)
		return mmu->mmu_ops->mmu_get_reg_gpuaddr(mmu, iommu_unit_num,
							ctx_id, reg);
	else
		return 0;
}

<<<<<<< HEAD
/*
 * kgsl_mmu_get_reg_ahbaddr() - Calls the mmu specific function pointer to
 * return the address that GPU can use to access register
 * @mmu:		Pointer to the device mmu
 * @iommu_unit_num:	There can be multiple iommu units used for graphics.
 *			This parameter is an index to the iommu unit being used
 * @ctx_id:		The context id within the iommu unit
 * @reg:		Register whose address is to be returned
 *
 * Returns the ahb address of reg else 0
 */
static inline unsigned int kgsl_mmu_get_reg_ahbaddr(struct kgsl_mmu *mmu,
						int iommu_unit_num,
						int ctx_id,
						enum kgsl_iommu_reg_map reg)
=======
static inline unsigned int kgsl_mmu_get_reg_ahbaddr(struct kgsl_mmu *mmu,
				int ctx_id, unsigned int reg)
>>>>>>> 0e91d2a... Nougat
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_get_reg_ahbaddr)
		return mmu->mmu_ops->mmu_get_reg_ahbaddr(mmu, iommu_unit_num,
							ctx_id, reg);
	else
		return 0;
}

static inline int kgsl_mmu_get_num_iommu_units(struct kgsl_mmu *mmu)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_get_num_iommu_units)
		return mmu->mmu_ops->mmu_get_num_iommu_units(mmu);
	else
		return 0;
}

/*
 * kgsl_mmu_hw_halt_supported() - Runtime check for iommu hw halt
 * @mmu: the mmu
 *
 * Returns non-zero if the iommu supports hw halt,
 * 0 if not.
 */
static inline int kgsl_mmu_hw_halt_supported(struct kgsl_mmu *mmu,
						int iommu_unit_num)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_hw_halt_supported)
		return mmu->mmu_ops->mmu_hw_halt_supported(mmu, iommu_unit_num);
	else
		return 0;
}

<<<<<<< HEAD
/*
 * kgsl_mmu_is_perprocess() - Runtime check for per-process
 * pagetables.
 * @mmu: the mmu
 *
 * Returns true if per-process pagetables are enabled,
 * false if not.
 */
static inline int kgsl_mmu_is_perprocess(struct kgsl_mmu *mmu)
{
	return KGSL_MMU_USE_PER_PROCESS_PT;
}

/*
 * kgsl_mmu_use_cpu_map() - Runtime check for matching the CPU
 * address space on the GPU.
 * @mmu: the mmu
 *
 * Returns true if supported false if not.
 */
static inline int kgsl_mmu_use_cpu_map(struct kgsl_mmu *mmu)
{
	return kgsl_mmu_is_perprocess(mmu);
}

static inline int kgsl_mmu_sync_lock(struct kgsl_mmu *mmu,
				unsigned int *cmds)
=======
static inline int kgsl_mmu_set_pagefault_policy(struct kgsl_mmu *mmu,
						unsigned long pf_policy)
>>>>>>> 0e91d2a... Nougat
{
	if ((mmu->flags & KGSL_MMU_FLAGS_IOMMU_SYNC) &&
		mmu->mmu_ops && mmu->mmu_ops->mmu_sync_lock)
		return mmu->mmu_ops->mmu_sync_lock(mmu, cmds);
	else
		return 0;
}

static inline int kgsl_mmu_sync_unlock(struct kgsl_mmu *mmu,
				unsigned int *cmds)
{
	if ((mmu->flags & KGSL_MMU_FLAGS_IOMMU_SYNC) &&
		mmu->mmu_ops && mmu->mmu_ops->mmu_sync_unlock)
		return mmu->mmu_ops->mmu_sync_unlock(mmu, cmds);
	else
		return 0;
}

static inline int kgsl_mmu_set_pagefault_policy(struct kgsl_mmu *mmu,
						unsigned int pf_policy)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_set_pf_policy)
		return mmu->mmu_ops->mmu_set_pf_policy(mmu, pf_policy);
	else
		return 0;
}

static inline void kgsl_mmu_set_pagefault(struct kgsl_mmu *mmu)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_set_pagefault)
		return mmu->mmu_ops->mmu_set_pagefault(mmu);
}

static inline struct kgsl_protected_registers *kgsl_mmu_get_prot_regs
						(struct kgsl_mmu *mmu)
{
	if (mmu->mmu_ops && mmu->mmu_ops->mmu_get_prot_regs)
		return mmu->mmu_ops->mmu_get_prot_regs(mmu);
	else
		return NULL;
}

static inline int kgsl_mmu_is_perprocess(struct kgsl_mmu *mmu)
{
	return MMU_FEATURE(mmu, KGSL_MMU_GLOBAL_PAGETABLE) ? 0 : 1;
}

static inline int kgsl_mmu_use_cpu_map(struct kgsl_mmu *mmu)
{
	return kgsl_mmu_is_perprocess(mmu);
}

static inline int kgsl_mmu_is_secured(struct kgsl_mmu *mmu)
{
	return mmu && (mmu->secured) && (mmu->securepagetable);
}

static inline phys_addr_t
kgsl_mmu_pagetable_get_ptbase(struct kgsl_pagetable *pagetable)
{
	if (pagetable && pagetable->pt_ops->get_ptbase)
		return pagetable->pt_ops->get_ptbase(pagetable);
	return 0;
}


<<<<<<< HEAD
=======
	return 0;
}

#ifdef CONFIG_MSM_IOMMU
#include <linux/qcom_iommu.h>
#ifndef CONFIG_ARM_SMMU
static inline bool kgsl_mmu_bus_secured(struct device *dev)
{
	struct bus_type *bus = msm_iommu_get_bus(dev);

	return (bus == &msm_iommu_sec_bus_type) ? true : false;
}
#else
static inline bool kgsl_mmu_bus_secured(struct device *dev)
{
	
	return true;
}
#endif 
static inline struct bus_type *kgsl_mmu_get_bus(struct device *dev)
{
	return msm_iommu_get_bus(dev);
}
static inline struct device *kgsl_mmu_get_ctx(const char *name)
{
	return msm_iommu_get_ctx(name);
}
#else
static inline bool kgsl_mmu_bus_secured(struct device *dev)
{
	
	return true;
}

static inline struct bus_type *kgsl_mmu_get_bus(struct device *dev)
{
	return &platform_bus_type;
}
static inline struct device *kgsl_mmu_get_ctx(const char *name)
{
	return ERR_PTR(-ENODEV);
}
#endif
>>>>>>> 0e91d2a... Nougat

#endif 
