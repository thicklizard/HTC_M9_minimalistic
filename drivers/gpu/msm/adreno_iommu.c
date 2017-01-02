<<<<<<< HEAD
/* Copyright (c) 2002,2007-2014, The Linux Foundation. All rights reserved.
=======
/* Copyright (c) 2002,2007-2016, The Linux Foundation. All rights reserved.
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
 *
 */
#include "adreno.h"
#include "kgsl_sharedmem.h"
<<<<<<< HEAD
=======
#include "a3xx_reg.h"
#include "adreno_pm4types.h"

#define A5XX_PFP_PER_PROCESS_UCODE_VER 0x5FF064
#define A5XX_PM4_PER_PROCESS_UCODE_VER 0x5FF052

static unsigned int _wait_reg(struct adreno_device *adreno_dev,
			unsigned int *cmds, unsigned int addr,
			unsigned int val, unsigned int mask,
			unsigned int interval)
{
	unsigned int *start = cmds;

	if (adreno_is_a3xx(adreno_dev)) {
		*cmds++ = cp_packet(adreno_dev, CP_WAIT_REG_EQ, 4);
		*cmds++ = addr;
		*cmds++ = val;
		*cmds++ = mask;
		*cmds++ = interval;
	} else {
		*cmds++ = cp_mem_packet(adreno_dev, CP_WAIT_REG_MEM, 5, 1);
		*cmds++ = 0x3; 
		cmds += cp_gpuaddr(adreno_dev, cmds, addr); 
		*cmds++ = val; 
		*cmds++ = mask;
		*cmds++ = interval;

		
		*cmds++ = cp_packet(adreno_dev, CP_SET_PROTECTED_MODE, 1);
		*cmds++ = 0;
	}

	return cmds - start;
}

#define KGSL_MMU(_dev) \
	((struct kgsl_mmu *) (&(KGSL_DEVICE((_dev))->mmu)))

static unsigned int  _iommu_lock(struct adreno_device *adreno_dev,
				 unsigned int *cmds)
{
	unsigned int *start = cmds;
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);

	BUG_ON(iommu->micro_mmu_ctrl == UINT_MAX);

	cmds += _wait_reg(adreno_dev, cmds,
			adreno_getreg(adreno_dev, ADRENO_REG_CP_WFI_PEND_CTR),
			1, 0xFFFFFFFF, 0xF);

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = iommu->micro_mmu_ctrl >> 2;
	
	*cmds++ = ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
	
	*cmds++ = KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT;

	
	cmds += _wait_reg(adreno_dev, cmds, iommu->micro_mmu_ctrl >> 2,
			KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
			KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE, 0xF);

	return cmds - start;
}

static unsigned int _iommu_unlock(struct adreno_device *adreno_dev,
				  unsigned int *cmds)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	unsigned int *start = cmds;

	BUG_ON(iommu->micro_mmu_ctrl == UINT_MAX);

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = iommu->micro_mmu_ctrl >> 2;
	
	*cmds++ = ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
	
	*cmds++ = 0;

	
	cmds += cp_wait_for_me(adreno_dev, cmds);

	return cmds - start;
}

static unsigned int _vbif_lock(struct adreno_device *adreno_dev,
			unsigned int *cmds)
{
	unsigned int *start = cmds;
	cmds += _wait_reg(adreno_dev, cmds,
			adreno_getreg(adreno_dev, ADRENO_REG_CP_WFI_PEND_CTR),
			1, 0xFFFFFFFF, 0xF);

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
	
	*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
	
	*cmds++ = 0x1;

	
	cmds += _wait_reg(adreno_dev, cmds,
			A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL1,
			1, 0xFFFFFFFF, 0xF);

	return cmds - start;
}

static unsigned int _vbif_unlock(struct adreno_device *adreno_dev,
				unsigned int *cmds)
{
	unsigned int *start = cmds;

	
	*cmds++ = cp_packet(adreno_dev, CP_REG_RMW, 3);
	*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
	
	*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
	
	*cmds++ = 0;

	
	cmds += cp_wait_for_me(adreno_dev, cmds);
	return cmds - start;
}

static unsigned int _cp_smmu_reg(struct adreno_device *adreno_dev,
				unsigned int *cmds,
				enum kgsl_iommu_reg_map reg,
				unsigned int num)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	unsigned int *start = cmds;
	unsigned int offset;

	offset = kgsl_mmu_get_reg_ahbaddr(KGSL_MMU(adreno_dev),
					  KGSL_IOMMU_CONTEXT_USER, reg) >> 2;

	if (adreno_is_a5xx(adreno_dev) || iommu->version == 1) {
		*cmds++ = cp_register(adreno_dev, offset, num);
	} else if (adreno_is_a3xx(adreno_dev)) {
		*cmds++ = cp_packet(adreno_dev, CP_REG_WR_NO_CTXT, num + 1);
		*cmds++ = offset;
	} else if (adreno_is_a4xx(adreno_dev)) {
		*cmds++ = cp_packet(adreno_dev, CP_WIDE_REG_WRITE, num + 1);
		*cmds++ = offset;
	} else  {
		BUG();
	}
	return cmds - start;
}

static unsigned int _tlbiall(struct adreno_device *adreno_dev,
				unsigned int *cmds)
{
	unsigned int *start = cmds;
	unsigned int tlbstatus;

	tlbstatus = kgsl_mmu_get_reg_ahbaddr(KGSL_MMU(adreno_dev),
			KGSL_IOMMU_CONTEXT_USER,
			KGSL_IOMMU_CTX_TLBSTATUS) >> 2;

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TLBIALL, 1);
	*cmds++ = 1;

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TLBSYNC, 1);
	*cmds++ = 0;

	cmds += _wait_reg(adreno_dev, cmds, tlbstatus, 0,
			KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);

	return cmds - start;
}


static inline int _adreno_iommu_add_idle_cmds(struct adreno_device *adreno_dev,
							unsigned int *cmds)
{
	unsigned int *start = cmds;

	cmds += cp_wait_for_idle(adreno_dev, cmds);

	if (adreno_is_a3xx(adreno_dev))
		cmds += cp_wait_for_me(adreno_dev, cmds);

	return cmds - start;
}

static void _invalidate_uche_cpu(struct adreno_device *adreno_dev)
{
	
	if (adreno_is_a5xx(adreno_dev))
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0, 0x12);
	else if (adreno_is_a4xx(adreno_dev)) {
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0, 0);
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE1, 0x12);
	} else if (adreno_is_a3xx(adreno_dev)) {
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0, 0);
		adreno_writereg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE1,
			0x90000000);
	} else {
		BUG();
	}
}

static bool _ctx_switch_use_cpu_path(
				struct adreno_device *adreno_dev,
				struct kgsl_pagetable *new_pt,
				struct adreno_ringbuffer *rb)
{
	struct kgsl_mmu *mmu = KGSL_MMU(adreno_dev);

	if (adreno_dev->cur_rb == rb)
		return adreno_isidle(KGSL_DEVICE(adreno_dev)) &&
			(new_pt == mmu->defaultpagetable);
	else if (adreno_rb_empty(rb) &&
			(new_pt == mmu->defaultpagetable))
		return true;

	return false;
}

static unsigned int adreno_iommu_set_apriv(struct adreno_device *adreno_dev,
				unsigned int *cmds, int set)
{
	unsigned int *cmds_orig = cmds;

	
	if (adreno_is_a3xx(adreno_dev))
		return 0;

	cmds += cp_wait_for_idle(adreno_dev, cmds);
	cmds += cp_wait_for_me(adreno_dev, cmds);
	*cmds++ = cp_register(adreno_dev, adreno_getreg(adreno_dev,
				ADRENO_REG_CP_CNTL), 1);
	if (set)
		*cmds++ = 1;
	else
		*cmds++ = 0;

	return cmds - cmds_orig;
}

static inline int _adreno_iommu_add_idle_indirect_cmds(
			struct adreno_device *adreno_dev,
			unsigned int *cmds, uint64_t nop_gpuaddr)
{
	unsigned int *start = cmds;
	cmds += cp_wait_for_me(adreno_dev, cmds);
	*cmds++ = cp_mem_packet(adreno_dev, CP_INDIRECT_BUFFER_PFE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, nop_gpuaddr);
	*cmds++ = 2;
	cmds += cp_wait_for_idle(adreno_dev, cmds);
	return cmds - start;
}
>>>>>>> 0e91d2a... Nougat

static unsigned int _adreno_mmu_set_pt_update_condition(
			struct adreno_ringbuffer *rb,
			unsigned int *cmds, unsigned int ptname)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int *cmds_orig = cmds;
<<<<<<< HEAD
	/*
	 * write 1 to switch pt flag indicating that we need to execute the
	 * pt switch commands
	 */
	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
		switch_pt_enable);
=======
	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
>>>>>>> 0e91d2a... Nougat
	*cmds++ = 1;
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
<<<<<<< HEAD
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
=======
	cmds += cp_wait_for_me(adreno_dev, cmds);
	*cmds++ = cp_mem_packet(adreno_dev, CP_COND_WRITE, 6, 2);
	
	*cmds++ = (1 << 8) | (1 << 4) | 3;
	cmds += cp_gpuaddr(adreno_dev, cmds,
	   (adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(current_global_ptname)));
	*cmds++ = ptname;
	*cmds++ = 0xFFFFFFFF;
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
>>>>>>> 0e91d2a... Nougat
	*cmds++ = 0;
	if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
		/* copy current ptid value to register SCRATCH_REG7 */
		*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
		*cmds++ = adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG7);
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
		/* copy the incoming ptid to SCRATCH_REG6 */
		*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
		*cmds++ = adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				incoming_ptname);
		/*
		 * compare the incoming ptid to current ptid and make the
		 * the pt switch commands optional based on condition
		 * that current_global_ptname(SCRATCH_REG7) ==
		 * incoming_ptid(SCRATCH_REG6)
		 */
		*cmds++ = cp_type3_packet(CP_COND_REG_EXEC, 3);
		*cmds++ = (2 << 28) | adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG7);
		*cmds++ = 7;
		/*
		 * if the incoming and current pt are equal then set the pt
		 * switch flag to 0 so that the pt switch commands will be
		 * skipped
		 */
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
		*cmds++ = 0;
	} else {
		/*
		 * same as if operation above except the current ptname is
		 * directly compared to the incoming pt id
		 */
		*cmds++ = cp_type3_packet(CP_COND_WRITE, 6);
		/* write to mem space, when a mem space is equal to ref val */
		*cmds++ = (1 << 8) | (1 << 4) | 3;
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
		*cmds++ = ptname;
		*cmds++ = 0xFFFFFFFF;
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
		*cmds++ = 0;
	}
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
	*cmds++ = 0;

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_pt_update_pid_to_mem(
				struct adreno_ringbuffer *rb,
				unsigned int *cmds, int ptname)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int *cmds_orig = cmds;

<<<<<<< HEAD
	if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
		/* copy the incoming pt in memory to  SCRATCH_REG6 */
		*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
		*cmds++ = adreno_getreg(adreno_dev,
			ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				incoming_ptname);
		/* copy the value in SCRATCH_REG6 to the current pt field */
		*cmds++ = cp_type3_packet(CP_REG_TO_MEM, 2);
		*cmds++ = adreno_getreg(adreno_dev,
			ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_rb_ptname);
		*cmds++ = cp_type3_packet(CP_REG_TO_MEM, 2);
		*cmds++ = adreno_getreg(adreno_dev,
			ADRENO_REG_CP_SCRATCH_REG6);
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
	} else {
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = rb->pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_rb_ptname);
		*cmds++ = ptname;

		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_global_ptname);
		*cmds++ = ptname;
	}
	/* pagetable switch done, Housekeeping: set the switch_pt_enable to 0 */
	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
		switch_pt_enable);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
=======
	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(current_rb_ptname)));
	*cmds++ = ptname;
	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds,
		(adreno_dev->ringbuffers[0].pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(current_global_ptname)));
	*cmds++ = ptname;
	
	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
>>>>>>> 0e91d2a... Nougat
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
	*cmds++ = 0;

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v0(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units)
{
	phys_addr_t reg_pt_val;
	unsigned int *cmds = cmds_orig;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;

	cmds += adreno_add_bank_change_cmds(cmds,
				KGSL_IOMMU_CONTEXT_USER,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	/* Acquire GPU-CPU sync Lock here */
	cmds += kgsl_mmu_sync_lock(&device->mmu, cmds);

	/*
	 * We need to perfrom the following operations for all
	 * IOMMU units
	 */
	for (i = 0; i < num_iommu_units; i++) {
		reg_pt_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
					i, KGSL_IOMMU_CONTEXT_USER);
		reg_pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		reg_pt_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		/*
		 * Set address of the new pagetable by writng to IOMMU
		 * TTBR0 register
		 */
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
			KGSL_IOMMU_CONTEXT_USER, KGSL_IOMMU_CTX_TTBR0);
		*cmds++ = reg_pt_val;
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
		*cmds++ = 0x00000000;

		/*
		 * Read back the ttbr0 register as a barrier to ensure
		 * above writes have completed
		 */
		cmds += adreno_add_read_cmds(cmds,
			kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER, KGSL_IOMMU_CTX_TTBR0),
				reg_pt_val,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);
	}
	/*
	 * tlb flush
	 */
	for (i = 0; i < num_iommu_units; i++) {
		reg_pt_val = (pt_val + kgsl_mmu_get_default_ttbr0(
					&device->mmu,
					i, KGSL_IOMMU_CONTEXT_USER));
		reg_pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		reg_pt_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
		*cmds++ = kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
			KGSL_IOMMU_CONTEXT_USER,
			KGSL_IOMMU_CTX_TLBIALL);
		*cmds++ = 1;

		cmds += __adreno_add_idle_indirect_cmds(cmds,
			device->mmu.setstate_memory.gpuaddr +
			KGSL_IOMMU_SETSTATE_NOP_OFFSET);

		cmds += adreno_add_read_cmds(cmds,
			kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_CTX_TTBR0),
				reg_pt_val,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);
	}

	/* Release GPU-CPU sync Lock here */
	cmds += kgsl_mmu_sync_unlock(&device->mmu, cmds);

	cmds += adreno_add_bank_change_cmds(cmds,
		KGSL_IOMMU_CONTEXT_PRIV,
		device->mmu.setstate_memory.gpuaddr +
		KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v1(struct adreno_ringbuffer *rb,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					unsigned int ptname,
					int num_iommu_units)
{
<<<<<<< HEAD
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	uint64_t ttbr0_val = 0;
	unsigned int reg_pt_val;
=======
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
>>>>>>> 0e91d2a... Nougat
	unsigned int *cmds = cmds_orig;
	unsigned int *cond_exec_ptr;
	int i;
	unsigned int ttbr0, tlbiall, tlbstatus, tlbsync, mmu_ctrl;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	
	cmds += _adreno_mmu_set_pt_update_condition(rb, cmds, ptname);
<<<<<<< HEAD
	*cmds++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
			switch_pt_enable);
=======
	*cmds++ = cp_mem_packet(adreno_dev, CP_COND_EXEC, 4, 2);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(switch_pt_enable)));
>>>>>>> 0e91d2a... Nougat
	*cmds++ = 1;
	
	cond_exec_ptr = cmds;
	cmds++;
	for (i = 0; i < num_iommu_units; i++) {
		if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
			int count = 1;

			if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
				0xFFFFFFFF00000000ULL)
				count = 2;
			/* transfer the ttbr0 value to ME_SCRATCH */
			*cmds++ = cp_type3_packet(CP_MEM_TO_REG, 2);
			*cmds++ = count << 16 | adreno_getreg(adreno_dev,
					ADRENO_REG_CP_SCRATCH_REG6);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t);
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
			*cmds++ = 0;
			*cmds++ = cp_type3_packet(CP_REG_TO_SCRATCH, 1);
			*cmds++ = (count << 24) | (6 << 16) |
				adreno_getreg(adreno_dev,
						ADRENO_REG_CP_SCRATCH_REG6);
		} else {
			ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
			ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
			ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		}
		mmu_ctrl = kgsl_mmu_get_reg_ahbaddr(
			&device->mmu, i,
			KGSL_IOMMU_CONTEXT_USER,
			KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL) >> 2;

		ttbr0 = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TTBR0) >> 2;

		if (kgsl_mmu_hw_halt_supported(&device->mmu, i)) {
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
			*cmds++ = 0;
			/*
			 * glue commands together until next
			 * WAIT_FOR_ME
			 */
			if (adreno_is_a4xx(adreno_dev))
				cmds += adreno_wait_reg_mem(cmds,
				adreno_getreg(adreno_dev,
					ADRENO_REG_CP_WFI_PEND_CTR),
					1, 0xFFFFFFFF, 0xF);
			else
				cmds += adreno_wait_reg_eq(cmds,
				adreno_getreg(adreno_dev,
					ADRENO_REG_CP_WFI_PEND_CTR),
					1, 0xFFFFFFFF, 0xF);

			/* set the iommu lock bit */
			*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
			*cmds++ = mmu_ctrl;
			/* AND to unmask the lock bit */
			*cmds++ =
				 ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
				/* OR to set the IOMMU lock bit */
			*cmds++ =
				   KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT;
			/* wait for smmu to lock */
			if (adreno_is_a4xx(adreno_dev))
				cmds += adreno_wait_reg_mem(cmds,
					mmu_ctrl,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					0xF);
			else
				cmds += adreno_wait_reg_eq(cmds,
					mmu_ctrl,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
					0xF);
		}
		if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS)) {
			/* ME_SCRATCH_REG to REG copy */
			*cmds++ = cp_type3_packet(CP_SCRATCH_TO_REG, 1);
			if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
				0xFFFFFFFF00000000ULL)
				*cmds++ = (2 << 24) | (6 << 16) | ttbr0;
			else
				*cmds++ = (1 << 24) | (6 << 16) | ttbr0;
		} else {
			/*
			 * set ttbr0, only need to set the higer bits if the
			 * address bits lie in the higher bits
			 */
			if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
				0xFFFFFFFF00000000ULL) {
				reg_pt_val = (unsigned int)ttbr0_val &
						0xFFFFFFFF;
				*cmds++ = cp_type0_packet(ttbr0, 1);
				*cmds++ = reg_pt_val;
				reg_pt_val = (unsigned int)
				((ttbr0_val & 0xFFFFFFFF00000000ULL) >> 32);
				*cmds++ = cp_type0_packet(ttbr0 + 1, 1);
				*cmds++ = reg_pt_val;
			} else {
				reg_pt_val = ttbr0_val;
				*cmds++ = cp_type0_packet(ttbr0, 1);
				*cmds++ = reg_pt_val;
			}
		}

<<<<<<< HEAD
		if (kgsl_mmu_hw_halt_supported(&device->mmu, i) &&
			adreno_is_a3xx(adreno_dev)) {
			/* unlock the IOMMU lock */
			*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
			*cmds++ = mmu_ctrl;
			/* AND to unmask the lock bit */
			*cmds++ =
			   ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
			/* OR with 0 so lock bit is unset */
			*cmds++ = 0;
			/* release all commands with wait_for_me */
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
			*cmds++ = 0;
		}
		tlbiall = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBIALL) >> 2;
		*cmds++ = cp_type0_packet(tlbiall, 1);
		*cmds++ = 1;

		tlbsync = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSYNC) >> 2;
		*cmds++ = cp_type0_packet(tlbsync, 1);
		*cmds++ = 0;

		tlbstatus = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_CTX_TLBSTATUS) >> 2;
		if (adreno_is_a4xx(adreno_dev))
			cmds += adreno_wait_reg_mem(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
		else
			cmds += adreno_wait_reg_eq(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);

		if (kgsl_mmu_hw_halt_supported(&device->mmu, i) &&
			!adreno_is_a3xx(adreno_dev)) {
			/* unlock the IOMMU lock */
			*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
			*cmds++ = mmu_ctrl;
			/* AND to unmask the lock bit */
			*cmds++ =
			   ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
			/* OR with 0 so lock bit is unset */
			*cmds++ = 0;
		}
		/* release all commands with wait_for_me */
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;
	}
	/* Exec count ordinal of CP_COND_EXEC packet */
=======
	cmds += cp_wait_for_idle(adreno_dev, cmds);

	cmds += _iommu_lock(adreno_dev, cmds);

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TTBR0, 2);
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	cmds += _cp_smmu_reg(adreno_dev, cmds,
			KGSL_IOMMU_CTX_CONTEXTIDR, 1);
	*cmds++ = contextidr;

	
	if (adreno_is_a3xx(adreno_dev))
		cmds += _iommu_unlock(adreno_dev, cmds);

	cmds += _tlbiall(adreno_dev, cmds);

	
	if (!adreno_is_a3xx(adreno_dev))
		cmds += _iommu_unlock(adreno_dev, cmds);
	else
		cmds += cp_wait_for_me(adreno_dev, cmds);

	
>>>>>>> 0e91d2a... Nougat
	*cond_exec_ptr = (cmds - cond_exec_ptr - 1);
	cmds += adreno_add_idle_cmds(adreno_dev, cmds);
	cmds += _adreno_iommu_pt_update_pid_to_mem(rb, cmds, ptname);

	return cmds - cmds_orig;
}


static unsigned int _adreno_iommu_set_pt_v2_a3xx(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	uint64_t ttbr0_val;
	unsigned int reg_pt_val;
	unsigned int *cmds = cmds_orig;
	int i;
	unsigned int ttbr0, tlbiall, tlbstatus, tlbsync;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	for (i = 0; i < num_iommu_units; i++) {
		ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
		ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		ttbr0 = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TTBR0) >> 2;

		/*
		 * glue commands together until next
		 * WAIT_FOR_ME
		 */
		cmds += adreno_wait_reg_eq(cmds,
			adreno_getreg(adreno_dev,
				ADRENO_REG_CP_WFI_PEND_CTR),
				1, 0xFFFFFFFF, 0xF);

		/* MMU-500 VBIF stall */
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		/* AND to unmask the HALT bit */
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		/* OR to set the HALT bit */
		*cmds++ = 0x1;

		/* Wait for acknowledgement */
		cmds += adreno_wait_reg_eq(cmds,
			A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL1,
				1, 0xFFFFFFFF, 0xF);

		/* set ttbr0 */
		if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK &
			0xFFFFFFFF00000000ULL) {
			reg_pt_val = (unsigned int)ttbr0_val &
					0xFFFFFFFF;
			*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;
			reg_pt_val = (unsigned int)
			((ttbr0_val & 0xFFFFFFFF00000000ULL) >> 32);
			*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
			*cmds++ = ttbr0 + 1;
			*cmds++ = reg_pt_val;
		} else {
			reg_pt_val = ttbr0_val;
			*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;
		}

		/* MMU-500 VBIF unstall */
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		/* AND to unmask the HALT bit */
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		/* OR to reset the HALT bit */
		*cmds++ = 0;

		/* release all commands with wait_for_me */
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;

		tlbiall = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBIALL) >> 2;
		*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
		*cmds++ = tlbiall;
		*cmds++ = 1;

		tlbsync = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSYNC) >> 2;
		*cmds++ = cp_type3_packet(CP_REG_WR_NO_CTXT, 2);
		*cmds++ = tlbsync;
		*cmds++ = 0;

<<<<<<< HEAD
		tlbstatus = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSTATUS) >> 2;
		cmds += adreno_wait_reg_eq(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
			/* release all commands with wait_for_me */
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;
	}
=======
	
	cmds += cp_wait_for_me(adreno_dev, cmds);
>>>>>>> 0e91d2a... Nougat

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_set_pt_v2_a4xx(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	uint64_t ttbr0_val;
	unsigned int reg_pt_val;
	unsigned int *cmds = cmds_orig;
	int i;
	unsigned int ttbr0, tlbiall, tlbstatus, tlbsync;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	for (i = 0; i < num_iommu_units; i++) {
		ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
		ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		ttbr0 = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TTBR0) >> 2;

		/*
		 * glue commands together until next
		 * WAIT_FOR_ME
		 */
		cmds += adreno_wait_reg_mem(cmds,
				adreno_getreg(adreno_dev,
					ADRENO_REG_CP_WFI_PEND_CTR),
					1, 0xFFFFFFFF, 0xF);

		/* MMU-500 VBIF stall */
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		/* AND to unmask the HALT bit */
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		/* OR to set the HALT bit */
		*cmds++ = 0x1;

		/* Wait for acknowledgement */
		cmds += adreno_wait_reg_mem(cmds,
			A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL1,
				1, 0xFFFFFFFF, 0xF);

		/* set ttbr0 */
		if (sizeof(phys_addr_t) > sizeof(unsigned int)) {

			reg_pt_val = ttbr0_val & 0xFFFFFFFF;
			*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;

			reg_pt_val = (unsigned int)((ttbr0_val &
				0xFFFFFFFF00000000ULL) >> 32);
			*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
			*cmds++ = ttbr0+1;
			*cmds++ = reg_pt_val;
		} else {
			reg_pt_val = ttbr0_val;
			*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
			*cmds++ = ttbr0;
			*cmds++ = reg_pt_val;
		}

<<<<<<< HEAD
		/* MMU-500 VBIF unstall */
		*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
		*cmds++ = A3XX_VBIF_DDR_OUTPUT_RECOVERABLE_HALT_CTRL0;
		/* AND to unmask the HALT bit */
		*cmds++ = ~(VBIF_RECOVERABLE_HALT_CTRL);
		/* OR to reset the HALT bit */
		*cmds++ = 0;
=======
	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);

	cmds += _vbif_lock(adreno_dev, cmds);

	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_TTBR0, 2);
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	cmds += _cp_smmu_reg(adreno_dev, cmds, KGSL_IOMMU_CTX_CONTEXTIDR, 1);
	*cmds++ = contextidr;

	cmds += _vbif_unlock(adreno_dev, cmds);

	cmds += _tlbiall(adreno_dev, cmds);

	
	cmds += cp_wait_for_me(adreno_dev, cmds);

	cmds += _adreno_iommu_add_idle_cmds(adreno_dev, cmds);
>>>>>>> 0e91d2a... Nougat

		/* release all commands with wait_for_me */
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;

		tlbiall = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBIALL) >> 2;

		*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
		*cmds++ = tlbiall;
		*cmds++ = 1;

<<<<<<< HEAD
		tlbsync = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSYNC) >> 2;

		*cmds++ = cp_type3_packet(CP_WIDE_REG_WRITE, 2);
		*cmds++ = tlbsync;
		*cmds++ = 0;

		tlbstatus = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_CTX_TLBSTATUS) >> 2;
		cmds += adreno_wait_reg_mem(cmds, tlbstatus, 0,
				KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
		/* release all commands with wait_for_me */
		*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
		*cmds++ = 0;
	}
=======
	
	*cmds++ = cp_packet(adreno_dev, CP_SMMU_TABLE_UPDATE, 3);
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	*cmds++ = contextidr;

	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 4, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds, (rb->pagetable_desc.gpuaddr +
		PT_INFO_OFFSET(ttbr0)));
	*cmds++ = lower_32_bits(ttbr0);
	*cmds++ = upper_32_bits(ttbr0);
	*cmds++ = contextidr;

	
	cmds += cp_wait_for_me(adreno_dev, cmds);
>>>>>>> 0e91d2a... Nougat

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

<<<<<<< HEAD
/**
 * adreno_iommu_set_pt_generate_cmds() - Generate commands to change pagetable
 * @rb: The RB pointer in which these commaands are to be submitted
 * @cmds: The pointer where the commands are placed
 * @pt: The pagetable to switch to
 */
static unsigned int adreno_iommu_set_pt_generate_cmds(
=======
unsigned int adreno_iommu_set_pt_generate_cmds(
>>>>>>> 0e91d2a... Nougat
					struct adreno_ringbuffer *rb,
					unsigned int *cmds,
					struct kgsl_pagetable *pt)
{
<<<<<<< HEAD
	struct kgsl_device *device = rb->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	phys_addr_t pt_val;
	int num_iommu_units;
=======
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);
	u64 ttbr0;
	u32 contextidr;
>>>>>>> 0e91d2a... Nougat
	unsigned int *cmds_orig = cmds;

	/* If we are in a fault the MMU will be reset soon */
	if (test_bit(ADRENO_DEVICE_FAULT, &adreno_dev->priv))
		return 0;

	num_iommu_units = kgsl_mmu_get_num_iommu_units(&device->mmu);

	pt_val = kgsl_mmu_get_pt_base_addr(&device->mmu, pt);

<<<<<<< HEAD
	cmds += __adreno_add_idle_indirect_cmds(cmds,
		device->mmu.setstate_memory.gpuaddr +
		KGSL_IOMMU_SETSTATE_NOP_OFFSET);
=======
	cmds += _adreno_iommu_add_idle_indirect_cmds(adreno_dev, cmds,
		iommu->setstate.gpuaddr + KGSL_IOMMU_SETSTATE_NOP_OFFSET);
>>>>>>> 0e91d2a... Nougat

	if (kgsl_msm_supports_iommu_v2())
		if (adreno_is_a4xx(adreno_dev))
			cmds += _adreno_iommu_set_pt_v2_a4xx(device, cmds,
					pt_val, num_iommu_units);
		else
<<<<<<< HEAD
			cmds += _adreno_iommu_set_pt_v2_a3xx(device, cmds,
					pt_val, num_iommu_units);
	else if (msm_soc_version_supports_iommu_v0())
		cmds += _adreno_iommu_set_pt_v0(device, cmds, pt_val,
						num_iommu_units);
	else
		cmds += _adreno_iommu_set_pt_v1(rb, cmds, pt_val,
						pt->name,
						num_iommu_units);

	/* invalidate all base pointers */
	*cmds++ = cp_type3_packet(CP_INVALIDATE_STATE, 1);
	*cmds++ = 0x7fff;
=======
			BUG(); 
	} else {
		cmds += _adreno_iommu_set_pt_v1(rb, cmds, ttbr0, contextidr,
						pt->name);
	}

	
	cmds += cp_invalidate_state(adreno_dev, cmds);

	cmds += adreno_iommu_set_apriv(adreno_dev, cmds, 0);
>>>>>>> 0e91d2a... Nougat

	return cmds - cmds_orig;
}

<<<<<<< HEAD
/**
 * adreno_iommu_set_pt_ib() - Generate commands to swicth pagetable. The
 * commands generated use an IB
 * @rb: The RB in which the commands will be executed
 * @cmds: Memory pointer where commands are generated
 * @pt: The pagetable to switch to
 */
static unsigned int adreno_iommu_set_pt_ib(struct adreno_ringbuffer *rb,
				unsigned int *cmds,
				struct kgsl_pagetable *pt)
{
	struct kgsl_device *device = rb->device;
	unsigned int *cmds_orig = cmds;
	phys_addr_t pt_val;
	int i;
	uint64_t ttbr0_val;
	int num_iommu_units = kgsl_mmu_get_num_iommu_units(&device->mmu);

	pt_val = kgsl_mmu_get_pt_base_addr(&(rb->device->mmu), pt);

	/* put the ptname in pagetable desc */
	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = rb->pagetable_desc.gpuaddr +
		offsetof(struct adreno_ringbuffer_pagetable_info,
			incoming_ptname);
	*cmds++ = pt->name;

	/* Write the ttbr0 value to pagetable desc memory */
	for (i = 0; i < num_iommu_units; i++) {
		ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
		ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);

		if (KGSL_IOMMU_CTX_TTBR0_ADDR_MASK & 0xFFFFFFFF00000000ULL) {
			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t);
			*cmds++ = ttbr0_val & 0xFFFFFFFF;
			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t) +
				sizeof(unsigned int);
			*cmds++ = ttbr0_val >> 32;
		} else {
			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = rb->pagetable_desc.gpuaddr +
				offsetof(
				struct adreno_ringbuffer_pagetable_info,
				ttbr0) + i * sizeof(uint64_t);
			*cmds = ttbr0_val & 0xFFFFFFFF;
		}
	}
	*cmds++ = cp_type3_packet(CP_WAIT_MEM_WRITES, 1);
	*cmds++ = 0;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
	*cmds++ = 0;
	*cmds++ = CP_HDR_INDIRECT_BUFFER_PFE;
	*cmds++ = rb->pt_update_desc.gpuaddr;
	*cmds++ = rb->pt_update_desc.size / sizeof(unsigned int);
=======
static unsigned int __add_curr_ctxt_cmds(struct adreno_ringbuffer *rb,
			unsigned int *cmds,
			struct adreno_context *drawctxt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	unsigned int *cmds_orig = cmds;

	
	*cmds++ = cp_packet(adreno_dev, CP_NOP, 1);
	*cmds++ = KGSL_CONTEXT_TO_MEM_IDENTIFIER;

	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds,
			MEMSTORE_RB_GPU_ADDR(device, rb, current_context));
	*cmds++ = (drawctxt ? drawctxt->base.id : 0);

	*cmds++ = cp_mem_packet(adreno_dev, CP_MEM_WRITE, 2, 1);
	cmds += cp_gpuaddr(adreno_dev, cmds,
			MEMSTORE_ID_GPU_ADDR(device,
				KGSL_MEMSTORE_GLOBAL, current_context));
	*cmds++ = (drawctxt ? drawctxt->base.id : 0);

	
	if (adreno_is_a5xx(adreno_dev)) {
		*cmds++ = cp_register(adreno_dev,
			adreno_getreg(adreno_dev,
		ADRENO_REG_UCHE_INVALIDATE0), 1);
		*cmds++ = 0x12;
	} else if (adreno_is_a4xx(adreno_dev)) {
		*cmds++ = cp_register(adreno_dev,
			adreno_getreg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0), 2);
		*cmds++ = 0;
		*cmds++ = 0x12;
	} else if (adreno_is_a3xx(adreno_dev)) {
		*cmds++ = cp_register(adreno_dev,
			adreno_getreg(adreno_dev,
			ADRENO_REG_UCHE_INVALIDATE0), 2);
		*cmds++ = 0;
		*cmds++ = 0x90000000;
	} else
		BUG();
>>>>>>> 0e91d2a... Nougat

	return cmds - cmds_orig;
}

<<<<<<< HEAD
/**
 * _set_pagetable_gpu() - Use GPU to switch the pagetable
 * @rb: The ringbuffer in which commands to switch pagetable are to be submitted
 * @new_pt: The pagetable to switch to
 */
int _set_pagetable_gpu(struct adreno_ringbuffer *rb,
=======
static void _set_ctxt_cpu(struct adreno_ringbuffer *rb,
			struct adreno_context *drawctxt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);

	if (rb == adreno_dev->cur_rb) {
		_invalidate_uche_cpu(adreno_dev);
		
		kgsl_sharedmem_writel(device, &device->memstore,
			KGSL_MEMSTORE_OFFSET(KGSL_MEMSTORE_GLOBAL,
						current_context),
			drawctxt ? drawctxt->base.id : 0);
	}
	
	kgsl_sharedmem_writel(device, &device->memstore,
		MEMSTORE_RB_OFFSET(rb, current_context),
		drawctxt ? drawctxt->base.id : 0);
}

static int _set_ctxt_gpu(struct adreno_ringbuffer *rb,
			struct adreno_context *drawctxt)
{
	unsigned int link[15], *cmds;
	int result;

	cmds = &link[0];
	cmds += __add_curr_ctxt_cmds(rb, cmds, drawctxt);
	result = adreno_ringbuffer_issuecmds(rb, 0, link,
			(unsigned int)(cmds - link));
	return result;
}

static int _set_pagetable_cpu(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	int result;

	
	if (adreno_dev->cur_rb == rb) {
		result = kgsl_mmu_set_pt(&device->mmu, new_pt);
		if (result)
			return result;
		
		adreno_ringbuffer_set_global(adreno_dev, new_pt->name);
	}

	
	adreno_ringbuffer_set_pagetable(rb, new_pt);

	return 0;
}

static int _set_pagetable_gpu(struct adreno_ringbuffer *rb,
>>>>>>> 0e91d2a... Nougat
			struct kgsl_pagetable *new_pt)
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	unsigned int *link = NULL, *cmds;
	int result;

	link = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (link == NULL)
		return -ENOMEM;

	cmds = link;

<<<<<<< HEAD
	kgsl_mmu_enable_clk(&device->mmu, KGSL_IOMMU_MAX_UNITS);

	/* pt switch may use privileged memory */
	if (adreno_is_a4xx(adreno_dev))
		cmds += adreno_set_apriv(adreno_dev, cmds, 1);
=======
	
	if (test_bit(ADRENO_DEVICE_FAULT, &adreno_dev->priv)) {
		kfree(link);
		return 0;
	}
>>>>>>> 0e91d2a... Nougat

	if (ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS))
		cmds += adreno_iommu_set_pt_ib(rb, cmds, new_pt);
	else
		cmds += adreno_iommu_set_pt_generate_cmds(rb, cmds, new_pt);

	if (adreno_is_a4xx(adreno_dev))
		cmds += adreno_set_apriv(adreno_dev, cmds, 0);

	if ((unsigned int) (cmds - link) > (PAGE_SIZE / sizeof(unsigned int))) {
		KGSL_DRV_ERR(KGSL_DEVICE(adreno_dev),
			"Temp command buffer overflow\n");
		BUG();
	}
	result = adreno_ringbuffer_issuecmds(rb,
			KGSL_CMD_FLAGS_PMODE, link,
			(unsigned int)(cmds - link));

<<<<<<< HEAD
	/*
	 * On error disable the IOMMU clock right away otherwise turn it off
	 * after the command has been retired
	 */
	if (result)
		kgsl_mmu_disable_clk(&device->mmu, KGSL_IOMMU_MAX_UNITS);
	else
		adreno_ringbuffer_mmu_disable_clk_on_ts(device, rb,
						rb->timestamp,
						KGSL_IOMMU_MAX_UNITS);

done:
=======
>>>>>>> 0e91d2a... Nougat
	kfree(link);
	return result;
}

<<<<<<< HEAD
/**
 * adreno_mmu_set_pt() - Change the pagetable of the current RB
 * @device: Pointer to device to which the rb belongs
 * @rb: The RB pointer on which pagetable is to be changed
 * @new_pt: The new pt the device will change to
 * @adreno_ctx: The context whose pagetable the ringbuffer should switch to,
 * NULL means default
 *
 * Returns 0 on success else error code.
 */
int adreno_iommu_set_pt(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt)
=======
int adreno_iommu_init(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_iommu *iommu = KGSL_IOMMU_PRIV(device);

	if (kgsl_mmu_get_mmutype(device) == KGSL_MMU_TYPE_NONE)
		return 0;


	kgsl_sharedmem_writel(device, &iommu->setstate,
				KGSL_IOMMU_SETSTATE_NOP_OFFSET,
				cp_packet(adreno_dev, CP_NOP, 1));

	
	if (adreno_is_a420(adreno_dev))
		device->mmu.features |= KGSL_MMU_FLUSH_TLB_ON_MAP;

	if (adreno_is_a5xx(adreno_dev) &&
		!MMU_FEATURE(&device->mmu, KGSL_MMU_GLOBAL_PAGETABLE)) {
		if ((adreno_compare_pfp_version(adreno_dev,
				A5XX_PFP_PER_PROCESS_UCODE_VER) < 0) ||
		    (adreno_compare_pm4_version(adreno_dev,
				A5XX_PM4_PER_PROCESS_UCODE_VER) < 0)) {
			KGSL_DRV_ERR(device,
				"Invalid ucode for per process pagetables\n");
			return -ENODEV;
		}
	}

	
	if (adreno_is_a3xx(adreno_dev) || adreno_is_a4xx(adreno_dev))
		device->mmu.features |= KGSL_MMU_NEED_GUARD_PAGE;

	return 0;
}

int adreno_iommu_set_pt_ctx(struct adreno_ringbuffer *rb,
			struct kgsl_pagetable *new_pt,
			struct adreno_context *drawctxt,
			unsigned long flags)
>>>>>>> 0e91d2a... Nougat
{
	struct adreno_device *adreno_dev = ADRENO_RB_DEVICE(rb);
	struct kgsl_device *device = KGSL_DEVICE(adreno_dev);
	struct kgsl_pagetable *cur_pt = device->mmu.defaultpagetable;
	int result = 0;
	int cpu_path = 0;

	if (rb->drawctxt_active)
		cur_pt = rb->drawctxt_active->base.proc_priv->pagetable;

<<<<<<< HEAD
	if (new_pt == cur_pt)
		return 0;

	/*
	 * For current rb cpu path can be used if device state is null or
	 * for non current rb if we are switching to default then all that
	 * we need to do is set the rb's current pagetable register to
	 * point to the default one
	 */
	if (adreno_dev->cur_rb == rb) {
		if (adreno_use_cpu_path(adreno_dev))
			cpu_path = 1;
	} else if ((rb->wptr == rb->rptr &&
			new_pt == device->mmu.defaultpagetable)) {
		cpu_path = 1;
	}
	if (cpu_path) {
		if (rb == adreno_dev->cur_rb) {
			result = kgsl_mmu_set_pt(&device->mmu, new_pt);
			if (result)
				return result;
			/* write the new pt set to the memory variables */
			kgsl_sharedmem_writel(device,
			&adreno_dev->ringbuffers[0].pagetable_desc,
				offsetof(
					struct adreno_ringbuffer_pagetable_info,
					current_global_ptname),
				new_pt->name);
		}
		kgsl_sharedmem_writel(device, &rb->pagetable_desc,
			offsetof(struct adreno_ringbuffer_pagetable_info,
				current_rb_ptname),
			new_pt->name);
	} else {
		result = _set_pagetable_gpu(rb, new_pt);
	}
	return result;
}
/**
 * adreno_iommu_set_pt_generate_rb_cmds() - Generate commands to switch pt
 * in a ringbuffer descriptor
 * @rb: The RB whose descriptor is used
 * @pt: The pt to switch to
 */
void adreno_iommu_set_pt_generate_rb_cmds(struct adreno_ringbuffer *rb,
						struct kgsl_pagetable *pt)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(rb->device);

	if (!ADRENO_FEATURE(adreno_dev, ADRENO_HAS_REG_TO_REG_CMDS) ||
		rb->pt_update_desc.hostptr)
		return;

	rb->pt_update_desc.hostptr = rb->pagetable_desc.hostptr +
			sizeof(struct adreno_ringbuffer_pagetable_info);
	rb->pt_update_desc.size =
		adreno_iommu_set_pt_generate_cmds(rb,
				rb->pt_update_desc.hostptr, pt) *
				sizeof(unsigned int);
	rb->pt_update_desc.gpuaddr = rb->pagetable_desc.gpuaddr +
			sizeof(struct adreno_ringbuffer_pagetable_info);
}
=======
	cpu_path = !(flags & ADRENO_CONTEXT_SWITCH_FORCE_GPU) &&
		_ctx_switch_use_cpu_path(adreno_dev, new_pt, rb);

	
	if (new_pt != cur_pt) {
		if (cpu_path)
			result = _set_pagetable_cpu(rb, new_pt);
		else
			result = _set_pagetable_gpu(rb, new_pt);
	}

	if (result)
		return result;

	
	if (cpu_path)
		_set_ctxt_cpu(rb, drawctxt);
	else
		result = _set_ctxt_gpu(rb, drawctxt);

	return result;
}
>>>>>>> 0e91d2a... Nougat
