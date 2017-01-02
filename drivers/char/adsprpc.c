/*
 * Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
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
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/hash.h>
#include <linux/msm_ion.h>
#include <soc/qcom/secure_buffer.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/glink.h>
#include <soc/qcom/subsystem_notif.h>
#include <linux/msm_iommu_domains.h>
#include <linux/scatterlist.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/dma-contiguous.h>
#include <linux/dma-buf.h>
#include <linux/iommu.h>
#include <linux/kref.h>
#include <linux/sort.h>
#include "adsprpc_compat.h"
#include "adsprpc_shared.h"
<<<<<<< HEAD

#ifndef ION_ADSPRPC_HEAP_ID
#define ION_ADSPRPC_HEAP_ID ION_AUDIO_HEAP_ID
#endif /*ION_ADSPRPC_HEAP_ID*/
=======
#include <soc/qcom/ramdump.h>
#include <linux/workqueue.h>
#include <linux/wait.h>


#define TZ_PIL_PROTECT_MEM_SUBSYS_ID 0x0C
#define TZ_PIL_CLEAR_PROTECT_MEM_SUBSYS_ID 0x0D
#define TZ_PIL_AUTH_QDSP6_PROC 1
#define ADSP_MMAP_HEAP_ADDR 4
#define FASTRPC_ENOSUCH 39
#define VMID_SSC_Q6     5
>>>>>>> 0e91d2a... Nougat

#define RPC_TIMEOUT	(5 * HZ)
#define RPC_HASH_BITS	5
#define RPC_HASH_SZ	(1 << RPC_HASH_BITS)
#define BALIGN		32
#define NUM_CHANNELS    2

#define LOCK_MMAP(kernel)\
		do {\
			if (!kernel)\
				down_read(&current->mm->mmap_sem);\
		} while (0)

#define UNLOCK_MMAP(kernel)\
		do {\
			if (!kernel)\
				up_read(&current->mm->mmap_sem);\
		} while (0)


#define IS_CACHE_ALIGNED(x) (((x) & ((L1_CACHE_BYTES)-1)) == 0)

<<<<<<< HEAD
static inline uintptr_t buf_page_start(void *buf)
=======
static void file_free_work_handler(struct work_struct *w);

static DECLARE_WAIT_QUEUE_HEAD(wait_queue);

static inline uint64_t buf_page_start(uint64_t buf)
>>>>>>> 0e91d2a... Nougat
{
	uintptr_t start = (uintptr_t) buf & PAGE_MASK;
	return start;
}

static inline uintptr_t buf_page_offset(void *buf)
{
	uintptr_t offset = (uintptr_t) buf & (PAGE_SIZE - 1);
	return offset;
}

static inline int buf_num_pages(void *buf, ssize_t len)
{
	uintptr_t start = buf_page_start(buf) >> PAGE_SHIFT;
	uintptr_t end = (((uintptr_t) buf + len - 1) & PAGE_MASK) >> PAGE_SHIFT;
	int nPages = end - start + 1;
	return nPages;
}

static inline uint32_t buf_page_size(uint32_t size)
{
	uint32_t sz = (size + (PAGE_SIZE - 1)) & PAGE_MASK;
	return sz > PAGE_SIZE ? sz : PAGE_SIZE;
}

static inline int buf_get_pages(void *addr, ssize_t sz, int nr_pages,
				int access, struct smq_phy_page *pages,
				int nr_elems, struct smq_phy_page *range)
{
	struct vm_area_struct *vma, *vmaend;
	uintptr_t start = buf_page_start(addr);
	uintptr_t end = buf_page_start((void *)((uintptr_t)addr + sz - 1));
	uint32_t len = nr_pages << PAGE_SHIFT;
	unsigned long pfn, pfnend, paddr;
	int n = -1, err = 0;

	VERIFY(err, 0 != access_ok(access ? VERIFY_WRITE : VERIFY_READ,
					(void __user *)start, len));
	if (err)
		goto bail;
	VERIFY(err, 0 != (vma = find_vma(current->mm, start)));
	if (err)
		goto bail;
	VERIFY(err, 0 != (vmaend = find_vma(current->mm, end)));
	if (err)
		goto bail;
	n = 0;
	if (follow_pfn(vma, start, &pfn))
		goto bail;
	if (follow_pfn(vmaend, end, &pfnend))
		goto bail;
	VERIFY(err, (pfn + nr_pages - 1) == pfnend);
	if (err)
		goto bail;
	VERIFY(err, nr_elems > 0);
	if (err)
		goto bail;
	VERIFY(err, __pfn_to_phys(pfnend) <= UINT_MAX);
	if (err)
		goto bail;
	paddr = __pfn_to_phys(pfn);
	if (range->size && (paddr < range->addr))
		goto bail;
	if (range->size && ((paddr - range->addr + len) > range->size))
		goto bail;
	pages->addr = paddr;
	pages->size = len;
	n++;
 bail:
	return n;
}

<<<<<<< HEAD
=======
struct fastrpc_file;

struct fastrpc_fl {
	struct fastrpc_file *fl;
	struct hlist_node hn;
};

>>>>>>> 0e91d2a... Nougat
struct fastrpc_buf {
	struct ion_handle *handle;
	void *virt;
	ion_phys_addr_t phys;
	ssize_t size;
	int used;
};

struct smq_context_list;

struct overlap {
	uintptr_t start;
	uintptr_t end;
	int raix;
	uintptr_t mstart;
	uintptr_t mend;
	uintptr_t offset;
};


struct smq_invoke_ctx {
	struct hlist_node hn;
	struct completion work;
	int retval;
	int pid;
	int tgid;
	remote_arg_t *pra;
	remote_arg_t *rpra;
	struct fastrpc_buf obuf;
	struct fastrpc_buf *abufs;
	struct fastrpc_device *dev;
	struct fastrpc_apps *apps;
	struct file_data *fdata;
	int *fds;
	struct ion_handle **handles;
	int nbufs;
	bool smmu;
	uint32_t sc;
	struct overlap *overs;
	struct overlap **overps;
	struct smq_msg msg;
};

struct smq_context_list {
	struct hlist_head pending;
	struct hlist_head interrupted;
	spinlock_t hlock;
};

struct fastrpc_smmu {
<<<<<<< HEAD
	struct iommu_group *group;
	struct iommu_domain *domain;
	int domain_id;
	bool enabled;
};

struct fastrpc_channel_context {
	smd_channel_t *chan;
=======
	struct dma_iommu_mapping *mapping;
	int cb;
	int enabled;
	int faults;
};

struct fastrpc_session_ctx {
	struct device *dev;
	struct fastrpc_smmu smmu;
};

struct fastrpc_channel_ctx {
	char *name;
	char *subsys;
	void *chan;
>>>>>>> 0e91d2a... Nougat
	struct device *dev;
	struct completion work;
	struct fastrpc_smmu smmu;
	struct kref kref;
	struct notifier_block nb;
	int ssrcount;
<<<<<<< HEAD
=======
	void *handle;
	int prevssrcount;
	int vmid;
	int ramdumpenabled;
	void *remoteheap_ramdump_dev;
	struct glink_link_info link_info;
	void *link_notify_handle;
	struct glink_open_config cfg;
	char *edge;
>>>>>>> 0e91d2a... Nougat
};

struct fastrpc_apps {
	struct fastrpc_channel_context channel[NUM_CHANNELS];
	struct smq_context_list clst;
	struct ion_client *iclient;
	struct cdev cdev;
	struct class *class;
	struct mutex smd_mutex;
	struct smq_phy_page range;
<<<<<<< HEAD
=======
	struct hlist_head maps;
	struct hlist_head fls;
	struct workqueue_struct *wq;
	int pending_free;
	struct work_struct free_work;
	struct mutex flfree_mutex;
>>>>>>> 0e91d2a... Nougat
	dev_t dev_no;
	int compat;
	spinlock_t wrlock;
	spinlock_t hlock;
<<<<<<< HEAD
	struct hlist_head htbl[RPC_HASH_SZ];
=======
	struct ion_client *client;
	struct device *dev;
	bool glink;
>>>>>>> 0e91d2a... Nougat
};

struct fastrpc_mmap {
	struct hlist_node hn;
	struct ion_handle *handle;
	void *virt;
	ion_phys_addr_t phys;
	uintptr_t *vaddrin;
	uintptr_t vaddrout;
	ssize_t size;
	int refs;
};

struct file_data {
	spinlock_t hlock;
	struct hlist_head hlst;
	uint32_t mode;
	int cid;
	int tgid;
	int ssrcount;
};

struct fastrpc_device {
	uint32_t tgid;
	struct hlist_node hn;
	struct fastrpc_buf buf;
};

struct fastrpc_channel_info {
	char *name;
	char *node;
	char *group;
	char *subsys;
	int channel;
};

static struct fastrpc_apps gfa;

static const struct fastrpc_channel_info gcinfo[NUM_CHANNELS] = {
	{
		.name = "adsprpc-smd",
		.node = "qcom,msm-audio-ion",
		.group = "lpass_audio",
		.subsys = "adsp",
		.channel = SMD_APPS_QDSP,
		.edge = "lpass",
	},
	{
		.name = "sdsprpc-smd",
		.subsys = "dsps",
		.channel = SMD_APPS_DSPS,
		.edge = "dsps",
		.vmid = VMID_SSC_Q6,
	},
	{
		.name = "mdsprpc-smd",
		.subsys = "modem",
		.channel = SMD_APPS_MODEM,
	},
};

static int map_iommu_mem(struct ion_handle *handle, struct file_data *fdata,
			ion_phys_addr_t *iova, unsigned long size)
{
<<<<<<< HEAD
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_mmap *map = 0, *mapmatch = 0;
	struct hlist_node *n;
	unsigned long len = size;
	int cid = fdata->cid;
	int err = 0;
=======
	struct fastrpc_file *fl = buf == 0 ? 0 : buf->fl;
	int vmid;

	if (!fl)
		return;
	if (cache) {
		spin_lock(&fl->hlock);
		hlist_add_head(&buf->hn, &fl->bufs);
		spin_unlock(&fl->hlock);
		return;
	}
	if (!IS_ERR_OR_NULL(buf->virt)) {
		int destVM[1] = {VMID_HLOS};
		int destVMperm[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};

		if (fl->sctx->smmu.cb)
			buf->phys &= ~((uint64_t)fl->sctx->smmu.cb << 32);
		vmid = fl->apps->channel[fl->cid].vmid;
		if (vmid) {
			int srcVM[2] = {VMID_HLOS, vmid};

			hyp_assign_phys(buf->phys, buf_page_size(buf->size),
				srcVM, 2, destVM, destVMperm, 1);
		}
		dma_free_coherent(fl->sctx->dev, buf->size, buf->virt,
					buf->phys);
	}
	kfree(buf);
}
>>>>>>> 0e91d2a... Nougat

	spin_lock(&fdata->hlock);
	hlist_for_each_entry_safe(map, n, &fdata->hlst, hn) {
		if (handle == map->handle) {
			mapmatch = map;
			break;
		}
	}
	spin_unlock(&fdata->hlock);

	if (mapmatch) {
		*iova = mapmatch->phys;
		return 0;
	}

	mutex_lock(&me->smd_mutex);
	VERIFY(err, fdata->ssrcount == me->channel[cid].ssrcount);
	if (!err)
		VERIFY(err, 0 == ion_map_iommu(me->iclient, handle,
				me->channel[cid].smmu.domain_id, 0,
				SZ_4K, 0, iova, &len, 0, 0));
	mutex_unlock(&me->smd_mutex);
	return err;
}

static void unmap_iommu_mem(struct ion_handle *handle, struct file_data *fdata,
				int cached)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_mmap *map = 0, *mapmatch = 0;
	struct hlist_node *n;
	int cid = fdata->cid;

	if (cached) {
		spin_lock(&fdata->hlock);
		hlist_for_each_entry_safe(map, n, &fdata->hlst, hn) {
			if (handle == map->handle) {
				mapmatch = map;
				break;
			}
		}
		spin_unlock(&fdata->hlock);
	}

<<<<<<< HEAD
	if (!mapmatch) {
		mutex_lock(&me->smd_mutex);
		if (fdata->ssrcount == me->channel[cid].ssrcount)
			ion_unmap_iommu(me->iclient, handle,
					me->channel[cid].smmu.domain_id, 0);
		mutex_unlock(&me->smd_mutex);
=======
	if (me->dev == NULL) {
		pr_err("device adsprpc-mem is not initialized\n");
		return -ENODEV;
	}
	dma_set_attr(DMA_ATTR_SKIP_ZEROING, &attrs);
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
	vaddr = dma_alloc_attrs(me->dev, size, region_start, GFP_KERNEL,
						&attrs);
	if (!vaddr) {
		pr_err("ADSPRPC: Failed to allocate %x remote heap memory\n",
						(unsigned int)size);
		return -ENOMEM;
>>>>>>> 0e91d2a... Nougat
	}
}

static void free_mem(struct fastrpc_buf *buf, struct file_data *fd)
{
	struct fastrpc_apps *me = &gfa;

	if (!IS_ERR_OR_NULL(buf->handle)) {
		if (me->channel[fd->cid].smmu.enabled && buf->phys) {
			unmap_iommu_mem(buf->handle, fd, 0);
			buf->phys = 0;
		}
		if (!IS_ERR_OR_NULL(buf->virt)) {
			ion_unmap_kernel(me->iclient, buf->handle);
			buf->virt = 0;
		}
		ion_free(me->iclient, buf->handle);
		buf->handle = 0;
	}
}

static void free_map(struct fastrpc_mmap *map, struct file_data *fdata)
{
	struct fastrpc_apps *me = &gfa;
<<<<<<< HEAD
	int cid = fdata->cid;

	if (!IS_ERR_OR_NULL(map->handle)) {
		if (me->channel[cid].smmu.enabled && map->phys) {
			unmap_iommu_mem(map->handle, fdata, 0);
			map->phys = 0;
		}
		if (!IS_ERR_OR_NULL(map->virt)) {
			ion_unmap_kernel(me->iclient, map->handle);
			map->virt = 0;
		}
		ion_free(me->iclient, map->handle);
=======
	struct fastrpc_file *fl;
	int vmid;

	if (!map)
		return;
	fl = map->fl;
	if (map->flags == ADSP_MMAP_HEAP_ADDR) {
		spin_lock(&me->hlock);
		map->refs--;
		if (!map->refs)
			hlist_del_init(&map->hn);
		spin_unlock(&me->hlock);
	} else {
		spin_lock(&fl->hlock);
		map->refs--;
		if (!map->refs)
			hlist_del_init(&map->hn);
		spin_unlock(&fl->hlock);
	}
	if (map->refs > 0)
		return;
	if (map->flags == ADSP_MMAP_HEAP_ADDR) {
		DEFINE_DMA_ATTRS(attrs);

		if (me->dev == NULL) {
			pr_err("failed to free remote heap allocation\n");
			return;
		}
		if (map->phys) {
			dma_set_attr(DMA_ATTR_SKIP_ZEROING, &attrs);
			dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &attrs);
			dma_free_attrs(me->dev, map->size,
					&(map->va), map->phys,	&attrs);
		}
	} else {
		int destVM[1] = {VMID_HLOS};
		int destVMperm[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};

		if (!IS_ERR_OR_NULL(map->handle))
			ion_free(fl->apps->client, map->handle);
		if (fl->sctx->smmu.enabled) {
			if (map->size || map->phys)
				msm_dma_unmap_sg(fl->sctx->dev,
					map->table->sgl,
					map->table->nents, DMA_BIDIRECTIONAL,
					map->buf);
		}
		vmid = fl->apps->channel[fl->cid].vmid;
		if (vmid && map->phys) {
			int srcVM[2] = {VMID_HLOS, vmid};

			hyp_assign_phys(map->phys, buf_page_size(map->size),
				srcVM, 2, destVM, destVMperm, 1);
		}

		if (!IS_ERR_OR_NULL(map->table))
			dma_buf_unmap_attachment(map->attach, map->table,
					DMA_BIDIRECTIONAL);
		if (!IS_ERR_OR_NULL(map->attach))
			dma_buf_detach(map->buf, map->attach);
		if (!IS_ERR_OR_NULL(map->buf))
			dma_buf_put(map->buf);
>>>>>>> 0e91d2a... Nougat
	}
	map->handle = 0;
}

static int alloc_mem(struct fastrpc_buf *buf, struct file_data *fdata)
{
	struct fastrpc_apps *me = &gfa;
<<<<<<< HEAD
	struct ion_client *clnt = gfa.iclient;
	struct sg_table *sg;
	int err = 0;
	int cid = fdata->cid;
	unsigned int heap;
	buf->handle = 0;
	buf->virt = 0;
	buf->phys = 0;
	heap = me->channel[cid].smmu.enabled ? ION_HEAP(ION_IOMMU_HEAP_ID) :
		ION_HEAP(ION_ADSP_HEAP_ID);
	buf->handle = ion_alloc(clnt, buf->size, SZ_4K, heap, ION_FLAG_CACHED);
	VERIFY(err, 0 == IS_ERR_OR_NULL(buf->handle));
=======
	struct fastrpc_session_ctx *sess = fl->sctx;
	struct fastrpc_mmap *map = 0;
	struct dma_attrs attrs;
	phys_addr_t region_start = 0;
	unsigned long flags;
	int err = 0, vmid;

	if (!fastrpc_mmap_find(fl, fd, va, len, mflags, ppmap))
		return 0;
	map = kzalloc(sizeof(*map), GFP_KERNEL);
	VERIFY(err, !IS_ERR_OR_NULL(map));
>>>>>>> 0e91d2a... Nougat
	if (err)
		goto bail;
	buf->virt = ion_map_kernel(clnt, buf->handle);
	VERIFY(err, 0 == IS_ERR_OR_NULL(buf->virt));
	if (err)
		goto bail;
	if (me->channel[cid].smmu.enabled) {
		VERIFY(err, 0 == map_iommu_mem(buf->handle, fdata,
						&buf->phys, buf->size));
		if (err)
			goto bail;
	} else {
<<<<<<< HEAD
		VERIFY(err, 0 != (sg = ion_sg_table(clnt, buf->handle)));
		if (err)
			goto bail;
		buf->phys = sg_dma_address(sg->sgl);
	}
=======
		VERIFY(err, !IS_ERR_OR_NULL(map->buf = dma_buf_get(fd)));
		if (err)
			goto bail;
		VERIFY(err, !IS_ERR_OR_NULL(map->attach =
				dma_buf_attach(map->buf, sess->dev)));
		if (err)
			goto bail;
		VERIFY(err, !IS_ERR_OR_NULL(map->table =
				dma_buf_map_attachment(map->attach,
					DMA_BIDIRECTIONAL)));
		if (err)
			goto bail;
		if (sess->smmu.enabled) {
			init_dma_attrs(&attrs);
			dma_set_attr(DMA_ATTR_EXEC_MAPPING, &attrs);
			VERIFY(err, map->table->nents ==
					msm_dma_map_sg_attrs(sess->dev,
					map->table->sgl, map->table->nents,
					DMA_BIDIRECTIONAL, map->buf, &attrs));
			if (err)
				goto bail;
		} else {
			VERIFY(err, map->table->nents == 1);
			if (err)
				goto bail;
		}
		VERIFY(err, !IS_ERR_OR_NULL(map->handle =
				ion_import_dma_buf(fl->apps->client, fd)));
		if (err)
			goto bail;
		VERIFY(err, !ion_handle_get_flags(fl->apps->client,
							map->handle, &flags));
		if (err)
			goto bail;
		map->uncached = !ION_IS_CACHED(flags);
		map->phys = sg_dma_address(map->table->sgl);
		if (sess->smmu.cb) {
			map->phys += ((uint64_t)sess->smmu.cb << 32);
			map->size = sg_dma_len(map->table->sgl);
		} else {
			map->size = buf_page_size(len);
		}
		vmid = fl->apps->channel[fl->cid].vmid;
		if (vmid) {
			int srcVM[1] = {VMID_HLOS};
			int destVM[2] = {VMID_HLOS, vmid};
			int destVMperm[2] = {PERM_READ | PERM_WRITE,
						PERM_READ | PERM_WRITE };

			VERIFY(err, !hyp_assign_phys(map->phys,
					buf_page_size(map->size),
					srcVM, 1, destVM, destVMperm, 2));
			if (err)
				goto bail;
		}
	}
	map->va = va;
	map->len = len;

	fastrpc_mmap_add(map);
	*ppmap = map;

bail:
	if (err && map)
		fastrpc_mmap_free(map);
	return err;
}

static int fastrpc_buf_alloc(struct fastrpc_file *fl, ssize_t size,
			     struct fastrpc_buf **obuf)
{
	int err = 0, vmid;
	struct fastrpc_buf *buf = 0, *fr = 0;
	struct hlist_node *n;

	VERIFY(err, size > 0);
	if (err)
		goto bail;

	/* find the smallest buffer that fits in the cache */
	spin_lock(&fl->hlock);
	hlist_for_each_entry_safe(buf, n, &fl->bufs, hn) {
		if (buf->size >= size && (!fr || fr->size > buf->size))
			fr = buf;
	}
	if (fr)
		hlist_del_init(&fr->hn);
	spin_unlock(&fl->hlock);
	if (fr) {
		*obuf = fr;
		return 0;
	}
	buf = 0;
	VERIFY(err, buf = kzalloc(sizeof(*buf), GFP_KERNEL));
	if (err)
		goto bail;
	INIT_HLIST_NODE(&buf->hn);
	buf->fl = fl;
	buf->virt = 0;
	buf->phys = 0;
	buf->size = size;
	buf->virt = dma_alloc_coherent(fl->sctx->dev, buf->size,
				       (void *)&buf->phys, GFP_KERNEL);
	if (IS_ERR_OR_NULL(buf->virt)) {
		/* free cache and retry */
		fastrpc_buf_list_free(fl);
		buf->virt = dma_alloc_coherent(fl->sctx->dev, buf->size,
					       (void *)&buf->phys, GFP_KERNEL);
		VERIFY(err, !IS_ERR_OR_NULL(buf->virt));
	}
	if (err)
		goto bail;
	if (fl->sctx->smmu.cb)
		buf->phys += ((uint64_t)fl->sctx->smmu.cb << 32);
	vmid = fl->apps->channel[fl->cid].vmid;
	if (vmid) {
		int srcVM[1] = {VMID_HLOS};
		int destVM[2] = {VMID_HLOS, vmid};
		int destVMperm[2] = {PERM_READ | PERM_WRITE,
					PERM_READ | PERM_WRITE};

		VERIFY(err, !hyp_assign_phys(buf->phys, buf_page_size(size),
			srcVM, 1, destVM, destVMperm, 2));
		if (err)
			goto bail;
	}

	*obuf = buf;
>>>>>>> 0e91d2a... Nougat
 bail:
	if (err && !IS_ERR_OR_NULL(buf->handle))
		free_mem(buf, fdata);
	return err;
}

static int context_restore_interrupted(struct fastrpc_apps *me,
				struct fastrpc_ioctl_invoke_fd *invokefd,
				struct file_data *fdata,
				struct smq_invoke_ctx **po)
{
	int err = 0;
	struct smq_invoke_ctx *ctx = 0, *ictx = 0;
	struct hlist_node *n;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;
	spin_lock(&me->clst.hlock);
	hlist_for_each_entry_safe(ictx, n, &me->clst.interrupted, hn) {
		if (ictx->pid == current->pid) {
			if (invoke->sc != ictx->sc || ictx->fdata != fdata)
				err = -1;
			else {
				ctx = ictx;
				hlist_del(&ctx->hn);
				hlist_add_head(&ctx->hn, &me->clst.pending);
			}
			break;
		}
	}
	spin_unlock(&me->clst.hlock);
	if (ctx)
		*po = ctx;
	return err;
}

#define CMP(aa, bb) ((aa) == (bb) ? 0 : (aa) < (bb) ? -1 : 1)
static int overlap_ptr_cmp(const void *a, const void *b)
{
	struct overlap *pa = *((struct overlap **)a);
	struct overlap *pb = *((struct overlap **)b);
	/* sort with lowest starting buffer first */
	int st = CMP(pa->start, pb->start);
	/* sort with highest ending buffer first */
	int ed = CMP(pb->end, pa->end);
	return st == 0 ? ed : st;
}

static int context_build_overlap(struct smq_invoke_ctx *ctx)
{
	int err = 0, i;
	remote_arg_t *pra = ctx->pra;
	int inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);
	int nbufs = inbufs + outbufs;
	struct overlap max;
	ctx->overs = kzalloc(sizeof(*ctx->overs) * (nbufs), GFP_KERNEL);
	VERIFY(err, !IS_ERR_OR_NULL(ctx->overs));
	if (err)
		goto bail;
	ctx->overps = kzalloc(sizeof(*ctx->overps) * (nbufs), GFP_KERNEL);
	VERIFY(err, !IS_ERR_OR_NULL(ctx->overps));
	if (err)
		goto bail;
	for (i = 0; i < nbufs; ++i) {
		ctx->overs[i].start = (uintptr_t)pra[i].buf.pv;
		ctx->overs[i].end = ctx->overs[i].start + pra[i].buf.len;
		ctx->overs[i].raix = i;
		ctx->overps[i] = &ctx->overs[i];
	}
	sort(ctx->overps, nbufs, sizeof(*ctx->overps), overlap_ptr_cmp, 0);
	max.start = 0;
	max.end = 0;
	for (i = 0; i < nbufs; ++i) {
		if (ctx->overps[i]->start < max.end) {
			ctx->overps[i]->mstart = max.end;
			ctx->overps[i]->mend = ctx->overps[i]->end;
			ctx->overps[i]->offset = max.end -
				ctx->overps[i]->start;
			if (ctx->overps[i]->end > max.end) {
				max.end = ctx->overps[i]->end;
			} else {
				ctx->overps[i]->mend = 0;
				ctx->overps[i]->mstart = 0;
			}
		} else  {
			ctx->overps[i]->mend = ctx->overps[i]->end;
			ctx->overps[i]->mstart = ctx->overps[i]->start;
			ctx->overps[i]->offset = 0;
			max = *ctx->overps[i];
		}
	}
bail:
	return err;
}


static void context_free(struct smq_invoke_ctx *ctx, int remove);

static int context_alloc(struct fastrpc_apps *me, uint32_t kernel,
				struct fastrpc_ioctl_invoke_fd *invokefd,
				struct file_data *fdata,
				struct smq_invoke_ctx **po)
{
	int err = 0, bufs, size = 0;
	struct smq_invoke_ctx *ctx = 0;
	struct smq_context_list *clst = &me->clst;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;

	bufs = REMOTE_SCALARS_INBUFS(invoke->sc) +
			REMOTE_SCALARS_OUTBUFS(invoke->sc);
	if (bufs) {
		size = bufs * sizeof(*ctx->pra);
		if (invokefd->fds)
			size = size + bufs * sizeof(*ctx->fds) +
				bufs * sizeof(*ctx->handles);
	}

	VERIFY(err, 0 != (ctx = kzalloc(sizeof(*ctx) + size, GFP_KERNEL)));
	if (err)
		goto bail;

	INIT_HLIST_NODE(&ctx->hn);
	hlist_add_fake(&ctx->hn);
	ctx->apps = me;
	ctx->fdata = fdata;
	ctx->pra = (remote_arg_t *)(&ctx[1]);
	ctx->fds = invokefd->fds == 0 ? 0 : (int *)(&ctx->pra[bufs]);
	ctx->handles = invokefd->fds == 0 ? 0 :
					(struct ion_handle **)(&ctx->fds[bufs]);
	if (!kernel) {
		VERIFY(err, 0 == copy_from_user(ctx->pra, invoke->pra,
					bufs * sizeof(*ctx->pra)));
		if (err)
			goto bail;
	} else {
		memmove(ctx->pra, invoke->pra, bufs * sizeof(*ctx->pra));
	}

	if (invokefd->fds) {
		if (!kernel) {
			VERIFY(err, 0 == copy_from_user(ctx->fds, invokefd->fds,
						bufs * sizeof(*ctx->fds)));
			if (err)
				goto bail;
		} else {
			memmove(ctx->fds, invokefd->fds,
						bufs * sizeof(*ctx->fds));
		}
	}
	ctx->sc = invoke->sc;
	if (REMOTE_SCALARS_INBUFS(ctx->sc) + REMOTE_SCALARS_OUTBUFS(ctx->sc)) {
		VERIFY(err, 0 == context_build_overlap(ctx));
		if (err)
			goto bail;
	}
	ctx->retval = -1;
	ctx->pid = current->pid;
	ctx->tgid = current->tgid;
	init_completion(&ctx->work);
	spin_lock(&clst->hlock);
	hlist_add_head(&ctx->hn, &clst->pending);
	spin_unlock(&clst->hlock);

	*po = ctx;
bail:
	if (ctx && err)
		context_free(ctx, 1);
	return err;
}

static void context_save_interrupted(struct smq_invoke_ctx *ctx)
{
	struct smq_context_list *clst = &ctx->apps->clst;
	spin_lock(&clst->hlock);
	hlist_del(&ctx->hn);
	hlist_add_head(&ctx->hn, &clst->interrupted);
	spin_unlock(&clst->hlock);
}

static void add_dev(struct fastrpc_apps *me, struct fastrpc_device *dev);

static void context_free(struct smq_invoke_ctx *ctx, int remove)
{
	struct smq_context_list *clst = &ctx->apps->clst;
	struct fastrpc_apps *apps = ctx->apps;
	struct ion_client *clnt = apps->iclient;
	int cid = ctx->fdata->cid;
	int ssrcount = ctx->fdata->ssrcount;
	struct fastrpc_smmu *smmu = &apps->channel[cid].smmu;
	struct fastrpc_buf *b;
	int i, bufs;
	if (ctx->smmu) {
		bufs = REMOTE_SCALARS_INBUFS(ctx->sc) +
			REMOTE_SCALARS_OUTBUFS(ctx->sc);
		if (ctx->fds) {
			for (i = 0; i < bufs; i++) {
				if (IS_ERR_OR_NULL(ctx->handles[i]))
					continue;
				unmap_iommu_mem(ctx->handles[i], ctx->fdata, 1);
				ion_free(clnt, ctx->handles[i]);
			}
		}
		mutex_lock(&apps->smd_mutex);
		if (ssrcount == apps->channel[cid].ssrcount)
			iommu_detach_group(smmu->domain, smmu->group);
		mutex_unlock(&apps->smd_mutex);
	}
	for (i = 0, b = ctx->abufs; i < ctx->nbufs; ++i, ++b)
		free_mem(b, ctx->fdata);

	kfree(ctx->abufs);
	if (ctx->dev) {
		add_dev(apps, ctx->dev);
		if (ctx->obuf.handle != ctx->dev->buf.handle)
			free_mem(&ctx->obuf, ctx->fdata);
	}
	if (remove) {
		spin_lock(&clst->hlock);
		hlist_del(&ctx->hn);
		spin_unlock(&clst->hlock);
	}
	kfree(ctx->overps);
	kfree(ctx->overs);
	kfree(ctx);
}

static void context_notify_user(struct smq_invoke_ctx *ctx, int retval)
{
	ctx->retval = retval;
	complete(&ctx->work);
}

static void context_notify_all_users(struct smq_context_list *me, int cid)
{
	struct smq_invoke_ctx *ictx = 0;
	struct hlist_node *n;
	spin_lock(&me->hlock);
	hlist_for_each_entry_safe(ictx, n, &me->pending, hn) {
		if (ictx->fdata->cid == cid)
			complete(&ictx->work);
	}
	hlist_for_each_entry_safe(ictx, n, &me->interrupted, hn) {
		if (ictx->fdata->cid == cid)
			complete(&ictx->work);
	}
	spin_unlock(&me->hlock);

}

static void context_list_ctor(struct smq_context_list *me)
{
	INIT_HLIST_HEAD(&me->interrupted);
	INIT_HLIST_HEAD(&me->pending);
	spin_lock_init(&me->hlock);
}

static void context_list_dtor(struct fastrpc_apps *me,
				struct smq_context_list *clst)
{
	struct smq_invoke_ctx *ictx = 0, *ctxfree;
	struct hlist_node *n;
	do {
		ctxfree = 0;
		spin_lock(&clst->hlock);
		hlist_for_each_entry_safe(ictx, n, &clst->interrupted, hn) {
			hlist_del(&ictx->hn);
			ctxfree = ictx;
			break;
		}
		spin_unlock(&clst->hlock);
		if (ctxfree)
			context_free(ctxfree, 0);
	} while (ctxfree);
	do {
		ctxfree = 0;
		spin_lock(&clst->hlock);
		hlist_for_each_entry_safe(ictx, n, &clst->pending, hn) {
			hlist_del(&ictx->hn);
			ctxfree = ictx;
			break;
		}
		spin_unlock(&clst->hlock);
		if (ctxfree)
			context_free(ctxfree, 0);
	} while (ctxfree);
}

static int get_page_list(uint32_t kernel, struct smq_invoke_ctx *ctx)
{
	struct fastrpc_apps *me = &gfa;
	struct smq_phy_page *pgstart, *pages;
	struct smq_invoke_buf *list;
	struct fastrpc_buf *ibuf = &ctx->dev->buf;
	struct fastrpc_buf *obuf = &ctx->obuf;
	remote_arg_t *pra = ctx->pra;
	ssize_t rlen;
	uint32_t sc = ctx->sc;
	int cid = ctx->fdata->cid;
	int i, err = 0;
	int inbufs = REMOTE_SCALARS_INBUFS(sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(sc);

	LOCK_MMAP(kernel);
	*obuf = *ibuf;
 retry:
	list = smq_invoke_buf_start((remote_arg_t *)obuf->virt, sc);
	pgstart = smq_phy_page_start(sc, list);
	pages = pgstart + 1;
	rlen = obuf->size - ((uintptr_t)pages - (uintptr_t)obuf->virt);
	if (rlen < 0) {
		rlen = ((uintptr_t)pages - (uintptr_t)obuf->virt) - obuf->size;
		obuf->size += buf_page_size(rlen);
		VERIFY(err, 0 == alloc_mem(obuf, ctx->fdata));
		if (err)
			goto bail;
		goto retry;
	}
	pgstart->addr = obuf->phys;
	pgstart->size = obuf->size;
	for (i = 0; i < inbufs + outbufs; ++i) {
		void *buf;
		int len, num;

		list[i].num = 0;
		list[i].pgidx = 0;
		len = pra[i].buf.len;
		VERIFY(err, len >= 0);
		if (err)
			goto bail;
		if (!len)
			continue;
		buf = pra[i].buf.pv;
		num = buf_num_pages(buf, len);
		if (!kernel) {
			if (me->channel[cid].smmu.enabled) {
				VERIFY(err, 0 != access_ok(i >= inbufs ?
					VERIFY_WRITE : VERIFY_READ,
					(void __user *)buf, len));
				if (err)
					goto bail;
				if (ctx->fds && (ctx->fds[i] >= 0))
					list[i].num = 1;
			} else {
				list[i].num = buf_get_pages(buf, len, num,
					i >= inbufs, pages,
					rlen / sizeof(*pages), &me->range);
			}
		}
		VERIFY(err, list[i].num >= 0);
		if (err)
			goto bail;
		if (list[i].num) {
			list[i].pgidx = pages - pgstart;
			pages = pages + list[i].num;
		} else if (rlen > sizeof(*pages)) {
			list[i].pgidx = pages - pgstart;
			pages = pages + 1;
		} else {
			if (obuf->handle != ibuf->handle)
				free_mem(obuf, ctx->fdata);
			obuf->size += buf_page_size(sizeof(*pages));
			VERIFY(err, 0 == alloc_mem(obuf, ctx->fdata));
			if (err)
				goto bail;
			goto retry;
		}
		rlen = obuf->size - ((uintptr_t)pages - (uintptr_t)obuf->virt);
	}
	obuf->used = obuf->size - rlen;
 bail:
	if (err && (obuf->handle != ibuf->handle))
		free_mem(obuf, ctx->fdata);
	UNLOCK_MMAP(kernel);
	return err;
}

static inline int is_overlapped_outbuf(struct smq_invoke_ctx *ctx, int oix)
{
	int inbufs, outbufs;

	inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);
	if (!ctx->overps[oix]->mstart)
		return 1;
	oix = oix + 1;
	if ((oix < inbufs + outbufs) && !ctx->overps[oix]->mstart &&
			ctx->overps[oix]->raix < inbufs)
		return 1;
	return 0;
}

static int clear_user_outbufs(struct smq_invoke_ctx *ctx)
{
	remote_arg_t *pra = ctx->pra;
	remote_arg_t *rpra = ctx->rpra;
	uintptr_t ptr, end;
	int oix, err = 0;
	int inbufs = REMOTE_SCALARS_INBUFS(ctx->sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(ctx->sc);

	for (oix = 0; oix < inbufs + outbufs; ++oix) {
		int i = ctx->overps[oix]->raix;
		if ((i < inbufs) || (pra[i].buf.pv != rpra[i].buf.pv) ||
			is_overlapped_outbuf(ctx, oix))
			continue;
		VERIFY(err, 0 == clear_user(rpra[i].buf.pv,
				(rpra[i].buf.len < 8) ? rpra[i].buf.len : 8));
		if (err)
			goto bail;
		ptr = buf_page_start(rpra[i].buf.pv) + PAGE_SIZE;
		end = (uintptr_t)rpra[i].buf.pv + rpra[i].buf.len;
		for (; ptr < end; ptr += PAGE_SIZE) {
			VERIFY(err, 0 == clear_user((void *)ptr,
					((end - ptr) < 8) ? end - ptr : 8));
			if (err)
				goto bail;
		}
	}

 bail:
	return err;
}

static int get_args(uint32_t kernel, struct smq_invoke_ctx *ctx,
			remote_arg_t *upra)
{
	struct fastrpc_apps *me = &gfa;
	struct smq_invoke_buf *list;
	struct fastrpc_buf *pbuf = &ctx->obuf, *obufs = 0;
	struct smq_phy_page *pages;
	struct vm_area_struct *vma;
	struct ion_handle **handles = ctx->handles;
	void *args;
	remote_arg_t *pra = ctx->pra;
	remote_arg_t *rpra = ctx->rpra;
	ssize_t rlen, used, size;
	uint32_t sc = ctx->sc, start;
	int i, inh, bufs = 0, err = 0, oix, copylen = 0;
	int inbufs = REMOTE_SCALARS_INBUFS(sc);
	int outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	int cid = ctx->fdata->cid;
	int *fds = ctx->fds, idx, num;
	ion_phys_addr_t iova;

	list = smq_invoke_buf_start(rpra, sc);
	pages = smq_phy_page_start(sc, list);
	used = ALIGN(pbuf->used, BALIGN);
	args = (void *)((char *)pbuf->virt + used);
	rlen = pbuf->size - used;

	/* map ion buffers */
	for (i = 0; i < inbufs + outbufs; ++i) {
		rpra[i].buf.len = pra[i].buf.len;
		if (!pra[i].buf.len)
			continue;
		if (me->channel[cid].smmu.enabled &&
					fds && (fds[i] >= 0)) {
			unsigned long len;
			start = buf_page_start(pra[i].buf.pv);
			len = buf_page_size(pra[i].buf.len);
			num = buf_num_pages(pra[i].buf.pv, pra[i].buf.len);
			idx = list[i].pgidx;
			handles[i] = ion_import_dma_buf(me->iclient, fds[i]);
			VERIFY(err, 0 == IS_ERR_OR_NULL(handles[i]));
			if (err)
				goto bail;
			VERIFY(err, 0 == map_iommu_mem(handles[i],
						ctx->fdata, &iova, len));
			if (err)
				goto bail;
			VERIFY(err, 0 != (vma = find_vma(current->mm, start)));
			if (err)
				goto bail;
			rpra[i].buf.pv = pra[i].buf.pv;
			pages[idx].addr = iova + (start - vma->vm_start);
			pages[idx].size = num << PAGE_SHIFT;
			continue;
		} else if (list[i].num) {
			rpra[i].buf.pv = pra[i].buf.pv;
			continue;
		}
	}

	/* calculate len requreed for copying */
	for (oix = 0; oix < inbufs + outbufs; ++oix) {
		int i = ctx->overps[oix]->raix;
		if (!pra[i].buf.len)
			continue;
		if (list[i].num)
			continue;
		if (ctx->overps[oix]->offset == 0)
			copylen = ALIGN(copylen, BALIGN);
		copylen += ctx->overps[oix]->mend - ctx->overps[oix]->mstart;
	}

	/* alocate new buffer */
	if (copylen > rlen) {
		struct fastrpc_buf *b;
		pbuf->used = pbuf->size - rlen;
		VERIFY(err, 0 != (b = krealloc(obufs,
			 (bufs + 1) * sizeof(*obufs), GFP_KERNEL)));
		if (err)
			goto bail;
		obufs = b;
		pbuf = obufs + bufs;
		pbuf->size = buf_num_pages(0, copylen) * PAGE_SIZE;
		VERIFY(err, 0 == alloc_mem(pbuf, ctx->fdata));
		if (err)
			goto bail;
		bufs++;
		args = pbuf->virt;
		rlen = pbuf->size;

	}

	/* copy non ion buffers */
	for (oix = 0; oix < inbufs + outbufs; ++oix) {
		int i = ctx->overps[oix]->raix;
		int mlen = ctx->overps[oix]->mend - ctx->overps[oix]->mstart;
		if (!pra[i].buf.len)
			continue;
		if (list[i].num)
			continue;

		if (ctx->overps[oix]->offset == 0) {
			rlen -= ALIGN((uintptr_t)args, BALIGN) -
				(uintptr_t)args;
			args = (void *)ALIGN((uintptr_t)args, BALIGN);
		}
		VERIFY(err, rlen >= mlen);
		if (err)
			goto bail;
		list[i].num = 1;
		rpra[i].buf.pv = args - ctx->overps[oix]->offset;
		pages[list[i].pgidx].addr =
			buf_page_start((void *)((uintptr_t)pbuf->phys -
						ctx->overps[oix]->offset +
						 (pbuf->size - rlen)));
		pages[list[i].pgidx].size = buf_num_pages(rpra[i].buf.pv,
						rpra[i].buf.len) * PAGE_SIZE;
		if (i < inbufs) {
			if (!kernel) {
				VERIFY(err, 0 == copy_from_user(rpra[i].buf.pv,
					pra[i].buf.pv, pra[i].buf.len));
				if (err)
					goto bail;
			} else {
				memmove(rpra[i].buf.pv, pra[i].buf.pv,
					pra[i].buf.len);
			}
		}
		args = (void *)((uintptr_t)args + mlen);
		rlen -= mlen;
	}

	if (!kernel) {
		VERIFY(err, 0 == clear_user_outbufs(ctx));
		if (err)
			goto bail;
	}
	for (oix = 0; oix < inbufs + outbufs; ++oix) {
		int i = ctx->overps[oix]->raix;
		if (rpra[i].buf.len && ctx->overps[oix]->mstart)
			dmac_flush_range(rpra[i].buf.pv,
				  (char *)rpra[i].buf.pv + rpra[i].buf.len);
	}
	pbuf->used = pbuf->size - rlen;
	size = sizeof(*rpra) * REMOTE_SCALARS_INHANDLES(sc);
	if (size) {
		inh = inbufs + outbufs;
		if (!kernel) {
			VERIFY(err, 0 == copy_from_user(&rpra[inh], &upra[inh],
							size));
			if (err)
				goto bail;
		} else {
			memmove(&rpra[inh], &upra[inh], size);
		}
	}
	dmac_flush_range(rpra, (char *)rpra + used);
 bail:
	ctx->abufs = obufs;
	ctx->nbufs = bufs;
	return err;
}

static int put_args(uint32_t kernel, uint32_t sc, remote_arg_t *pra,
			remote_arg_t *rpra, remote_arg_t *upra)
{
	int i, inbufs, outbufs, outh, size;
	int err = 0;

	inbufs = REMOTE_SCALARS_INBUFS(sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	for (i = inbufs; i < inbufs + outbufs; ++i) {
		if (rpra[i].buf.pv != pra[i].buf.pv) {
			if (!kernel) {
				VERIFY(err, 0 == copy_to_user(pra[i].buf.pv,
					rpra[i].buf.pv, rpra[i].buf.len));
				if (err)
					goto bail;
			} else {
				memmove(pra[i].buf.pv, rpra[i].buf.pv,
							rpra[i].buf.len);
			}
		}
	}
	size = sizeof(*rpra) * REMOTE_SCALARS_OUTHANDLES(sc);
	if (size) {
		outh = inbufs + outbufs + REMOTE_SCALARS_INHANDLES(sc);
		if (!kernel) {
			VERIFY(err, 0 == copy_to_user(&upra[outh], &rpra[outh],
						size));
			if (err)
				goto bail;
		} else {
			memmove(&upra[outh], &rpra[outh], size);
		}
	}
 bail:
	return err;
}

static void inv_args_pre(uint32_t sc, remote_arg_t *rpra)
{
	int i, inbufs, outbufs;
	uintptr_t end;

	inbufs = REMOTE_SCALARS_INBUFS(sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	for (i = inbufs; i < inbufs + outbufs; ++i) {
		if (!rpra[i].buf.len)
			continue;
		if (buf_page_start(rpra) == buf_page_start(rpra[i].buf.pv))
			continue;
		if (!IS_CACHE_ALIGNED((uintptr_t)rpra[i].buf.pv))
			dmac_flush_range(rpra[i].buf.pv,
				(char *)rpra[i].buf.pv + 1);
		end = (uintptr_t)rpra[i].buf.pv + rpra[i].buf.len;
		if (!IS_CACHE_ALIGNED(end))
			dmac_flush_range((char *)end,
				(char *)end + 1);
	}
}

static void inv_args(struct smq_invoke_ctx *ctx)
{
	uint32_t sc = ctx->sc;
	remote_arg_t *rpra = ctx->rpra;
	int used = ctx->obuf.used;
	int i, inbufs, outbufs;
	int inv = 0;

	inbufs = REMOTE_SCALARS_INBUFS(sc);
	outbufs = REMOTE_SCALARS_OUTBUFS(sc);
	for (i = inbufs; i < inbufs + outbufs; ++i) {
		if (buf_page_start(rpra) == buf_page_start(rpra[i].buf.pv))
			inv = 1;
		else if (rpra[i].buf.len)
			dmac_inv_range(rpra[i].buf.pv,
				(char *)rpra[i].buf.pv + rpra[i].buf.len);
	}

	if (inv || REMOTE_SCALARS_OUTHANDLES(sc))
		dmac_inv_range(rpra, (char *)rpra + used);
}

static int fastrpc_invoke_send(struct fastrpc_apps *me,
				 uint32_t kernel, uint32_t handle,
				 uint32_t sc, struct smq_invoke_ctx *ctx,
				 struct fastrpc_buf *buf)
{
<<<<<<< HEAD
	struct smq_msg msg;
	int err = 0, len;
	msg.pid = current->tgid;
	msg.tid = current->pid;
	if (kernel)
		msg.pid = 0;
	msg.invoke.header.ctx = ctx;
	msg.invoke.header.handle = handle;
	msg.invoke.header.sc = sc;
	msg.invoke.page.addr = buf->phys;
	msg.invoke.page.size = buf_page_size(buf->used);
	spin_lock(&me->wrlock);
	len = smd_write(me->channel[ctx->fdata->cid].chan, &msg, sizeof(msg));
	spin_unlock(&me->wrlock);
	VERIFY(err, len == sizeof(msg));
	return err;
}

static void fastrpc_deinit(void)
{
	struct fastrpc_apps *me = &gfa;
	int i;

	for (i = 0; i < NUM_CHANNELS; i++) {
		if (me->channel[i].chan) {
			(void)smd_close(me->channel[i].chan);
			me->channel[i].chan = 0;
		}
	}
	ion_client_destroy(me->iclient);
	me->iclient = 0;
}

static void fastrpc_read_handler(int cid)
=======
	struct smq_msg *msg = &ctx->msg;
	struct fastrpc_file *fl = ctx->fl;
	int err = 0, len;

	VERIFY(err, 0 != fl->apps->channel[fl->cid].chan);
	if (err)
		goto bail;
	msg->pid = current->tgid;
	msg->tid = current->pid;
	if (kernel)
		msg->pid = 0;
	msg->invoke.header.ctx = ptr_to_uint64(ctx);
	msg->invoke.header.handle = handle;
	msg->invoke.header.sc = ctx->sc;
	msg->invoke.page.addr = ctx->buf ? ctx->buf->phys : 0;
	msg->invoke.page.size = buf_page_size(ctx->used);

	if (fl->apps->glink) {
		err = glink_tx(fl->apps->channel[fl->cid].chan,
			(void *)&fl->apps->channel[fl->cid], msg, sizeof(*msg),
			GLINK_TX_REQ_INTENT);
	} else {
		spin_lock(&fl->apps->hlock);
		len = smd_write((smd_channel_t *)
				fl->apps->channel[fl->cid].chan,
				msg, sizeof(*msg));
		spin_unlock(&fl->apps->hlock);
		VERIFY(err, len == sizeof(*msg));
	}
 bail:
	return err;
}

static void fastrpc_smd_read_handler(int cid)
>>>>>>> 0e91d2a... Nougat
{
	struct fastrpc_apps *me = &gfa;
	struct smq_invoke_rsp rsp;
	int ret = 0;

	do {
		ret = smd_read_from_cb(me->channel[cid].chan, &rsp,
					sizeof(rsp));
		if (ret != sizeof(rsp))
			break;
		context_notify_user(rsp.ctx, rsp.retval);
	} while (ret == sizeof(rsp));
}

static void smd_event_handler(void *priv, unsigned event)
{
	struct fastrpc_apps *me = &gfa;
	int cid = (int)(uintptr_t)priv;

	switch (event) {
	case SMD_EVENT_OPEN:
		complete(&me->channel[cid].work);
		break;
	case SMD_EVENT_CLOSE:
		context_notify_all_users(&me->clst, cid);
		break;
	case SMD_EVENT_DATA:
		fastrpc_smd_read_handler(cid);
		break;
	}
}

static int fastrpc_init(void)
{
<<<<<<< HEAD
	int i, err = 0;
	struct fastrpc_apps *me = &gfa;
	struct device_node *node;
	struct fastrpc_smmu *smmu;
	bool enabled = 0;

=======
	int i;
	INIT_HLIST_HEAD(&me->drivers);
	INIT_HLIST_HEAD(&me->fls);
>>>>>>> 0e91d2a... Nougat
	spin_lock_init(&me->hlock);
	spin_lock_init(&me->wrlock);
	mutex_init(&me->smd_mutex);
	context_list_ctor(&me->clst);
	for (i = 0; i < RPC_HASH_SZ; ++i)
		INIT_HLIST_HEAD(&me->htbl[i]);
	me->iclient = msm_ion_client_create(DEVICE_NAME);
	VERIFY(err, 0 == IS_ERR_OR_NULL(me->iclient));
	if (err)
		goto bail;
	for (i = 0; i < NUM_CHANNELS; i++) {
		init_completion(&me->channel[i].work);
		if (!gcinfo[i].node)
			continue;
		smmu = &me->channel[i].smmu;
		node = of_find_compatible_node(NULL, NULL, gcinfo[i].node);
		if (node)
			enabled = of_property_read_bool(node,
						"qcom,smmu-enabled");
		if (enabled)
			smmu->group = iommu_group_find(gcinfo[i].group);
		if (smmu->group)
			smmu->domain = iommu_group_get_iommudata(smmu->group);
		if (!IS_ERR_OR_NULL(smmu->domain)) {
			smmu->domain_id = msm_find_domain_no(smmu->domain);
			if (smmu->domain_id >= 0)
				smmu->enabled = enabled;
		}
	}
	return 0;

bail:
	return err;
}

static void free_dev(struct fastrpc_device *dev, struct file_data *fdata)
{
	if (dev) {
		free_mem(&dev->buf, fdata);
		kfree(dev);
		module_put(THIS_MODULE);
	}
}

static int alloc_dev(struct fastrpc_device **dev, struct file_data *fdata)
{
	int err = 0;
	struct fastrpc_device *fd = 0;

	VERIFY(err, 0 != try_module_get(THIS_MODULE));
	if (err)
		goto bail;
	VERIFY(err, 0 != (fd = kzalloc(sizeof(*fd), GFP_KERNEL)));
	if (err)
		goto bail;

	INIT_HLIST_NODE(&fd->hn);

	fd->buf.size = PAGE_SIZE;
	VERIFY(err, 0 == alloc_mem(&fd->buf, fdata));
	if (err)
		goto bail;
	fd->tgid = current->tgid;

	*dev = fd;
 bail:
	if (err)
		free_dev(fd, fdata);
	return err;
}

static int get_dev(struct fastrpc_apps *me, struct file_data *fdata,
			struct fastrpc_device **rdev)
{
	struct hlist_head *head;
	struct fastrpc_device *dev = 0, *devfree = 0;
	struct hlist_node *n;
	uint32_t h = hash_32(current->tgid, RPC_HASH_BITS);
	int err = 0;

	spin_lock(&me->hlock);
	head = &me->htbl[h];
	hlist_for_each_entry_safe(dev, n, head, hn) {
		if (dev->tgid == current->tgid) {
			hlist_del(&dev->hn);
			devfree = dev;
			break;
		}
	}
	spin_unlock(&me->hlock);
	VERIFY(err, devfree != 0);
	if (err)
		goto bail;
	*rdev = devfree;
 bail:
	if (err) {
		free_dev(devfree, fdata);
		err = alloc_dev(rdev, fdata);
	}
	return err;
}

static void add_dev(struct fastrpc_apps *me, struct fastrpc_device *dev)
{
	struct hlist_head *head;
	uint32_t h = hash_32(current->tgid, RPC_HASH_BITS);

	spin_lock(&me->hlock);
	head = &me->htbl[h];
	hlist_add_head(&dev->hn, head);
	spin_unlock(&me->hlock);
	return;
}

static int fastrpc_release_current_dsp_process(struct file_data *fdata);

static int fastrpc_internal_invoke(struct fastrpc_apps *me, uint32_t mode,
			uint32_t kernel,
			struct fastrpc_ioctl_invoke_fd *invokefd,
			struct file_data *fdata)
{
	struct smq_invoke_ctx *ctx = 0;
	struct fastrpc_ioctl_invoke *invoke = &invokefd->inv;
	int cid = fdata->cid;
	int interrupted = 0;
	int err = 0;

	if (!kernel) {
		VERIFY(err, 0 == context_restore_interrupted(me, invokefd,
								fdata, &ctx));
		if (err)
			goto bail;
		if (ctx)
			goto wait;
	}

	VERIFY(err, 0 == context_alloc(me, kernel, invokefd, fdata, &ctx));
	if (err)
		goto bail;

	if (me->channel[cid].smmu.enabled) {
		mutex_lock(&me->smd_mutex);
		VERIFY(err, fdata->ssrcount == me->channel[cid].ssrcount);
		if (!err)
			VERIFY(err, 0 == iommu_attach_group(
						me->channel[cid].smmu.domain,
						me->channel[cid].smmu.group));
		mutex_unlock(&me->smd_mutex);
		if (err)
			goto bail;
		ctx->smmu = 1;
	}
	if (REMOTE_SCALARS_LENGTH(ctx->sc)) {
		VERIFY(err, 0 == get_dev(me, fdata, &ctx->dev));
		if (err)
			goto bail;
		VERIFY(err, 0 == get_page_list(kernel, ctx));
		if (err)
			goto bail;
		ctx->rpra = (remote_arg_t *)ctx->obuf.virt;
		VERIFY(err, 0 == get_args(kernel, ctx, invoke->pra));
		if (err)
			goto bail;
	}

	inv_args_pre(ctx->sc, ctx->rpra);
	if (FASTRPC_MODE_SERIAL == mode)
		inv_args(ctx);
	VERIFY(err, 0 == fastrpc_invoke_send(me, kernel, invoke->handle,
						ctx->sc, ctx, &ctx->obuf));
	if (err)
		goto bail;
	if (FASTRPC_MODE_PARALLEL == mode)
		inv_args(ctx);
 wait:
	if (kernel)
		wait_for_completion(&ctx->work);
	else {
		interrupted = wait_for_completion_interruptible(&ctx->work);
		VERIFY(err, 0 == (err = interrupted));
		if (err)
			goto bail;
	}
	VERIFY(err, 0 == (err = ctx->retval));
	if (err)
		goto bail;
	VERIFY(err, 0 == put_args(kernel, ctx->sc, ctx->pra, ctx->rpra,
					invoke->pra));
	if (err)
		goto bail;
 bail:
	if (ctx && interrupted == -ERESTARTSYS)
		context_save_interrupted(ctx);
	else if (ctx)
		context_free(ctx, 1);
	if (fdata->ssrcount != me->channel[cid].ssrcount)
		err = ECONNRESET;
	return err;
}

static int map_buffer(struct fastrpc_apps *me, struct file_data *fdata,
			int fd, char *buf, unsigned long len,
			struct fastrpc_mmap **ppmap,
			struct smq_phy_page **ppages, int *pnpages);

static int fastrpc_init_process(struct file_data *fdata,
				struct fastrpc_ioctl_init *init)
{
	int err = 0;
	struct fastrpc_ioctl_invoke_fd ioctl;
	struct smq_phy_page *pages = 0;
	struct fastrpc_mmap *map = 0;
	int npages = 0;
	struct fastrpc_apps *me = &gfa;
	if (init->flags == FASTRPC_INIT_ATTACH) {
		remote_arg_t ra[1];
		int tgid = current->tgid;
		ra[0].buf.pv = &tgid;
		ra[0].buf.len = sizeof(tgid);
		ioctl.inv.handle = 1;
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(0, 1, 0);
		ioctl.inv.pra = ra;
		ioctl.fds = 0;
		VERIFY(err, 0 == (err = fastrpc_internal_invoke(me,
			FASTRPC_MODE_PARALLEL, 1, &ioctl, fdata)));
		if (err)
			goto bail;
	} else if (init->flags == FASTRPC_INIT_CREATE) {
		remote_arg_t ra[4];
		int fds[4];
		struct {
			int pgid;
			int namelen;
			int filelen;
			int pageslen;
		} inbuf;
		inbuf.pgid = current->tgid;
		inbuf.namelen = strlen(current->comm) + 1;
		inbuf.filelen = init->filelen;
		VERIFY(err, 0 == map_buffer(me, fdata, init->memfd,
					(char *)init->mem, init->memlen,
					&map, &pages, &npages));
		if (err)
			goto bail;
		inbuf.pageslen = npages;
		ra[0].buf.pv = &inbuf;
		ra[0].buf.len = sizeof(inbuf);
		fds[0] = 0;

		ra[1].buf.pv = current->comm;
		ra[1].buf.len = inbuf.namelen;
		fds[1] = 0;

		ra[2].buf.pv = (void *)init->file;
		ra[2].buf.len = inbuf.filelen;
		fds[2] = init->filefd;

		ra[3].buf.pv = pages;
		ra[3].buf.len = npages * sizeof(*pages);
		fds[3] = 0;

		ioctl.inv.handle = 1;
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(6, 4, 0);
		ioctl.inv.pra = ra;
		ioctl.fds = fds;
		VERIFY(err, 0 == (err = fastrpc_internal_invoke(me,
			FASTRPC_MODE_PARALLEL, 1, &ioctl, fdata)));
		if (err)
			goto bail;
		spin_lock(&fdata->hlock);
		map->vaddrout = 0;
		hlist_add_head(&map->hn, &fdata->hlst);
		spin_unlock(&fdata->hlock);
	} else {
		err = -ENOTTY;
	}
bail:
	kfree(pages);
	if (err && map)
		free_map(map, fdata);
	return err;
}

static int fastrpc_release_current_dsp_process(struct file_data *fdata)
{
	int err = 0;
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_ioctl_invoke_fd ioctl;
	remote_arg_t ra[1];
	int tgid = 0;

	tgid = fdata->tgid;
	ra[0].buf.pv = &tgid;
	ra[0].buf.len = sizeof(tgid);
	ioctl.inv.handle = 1;
	ioctl.inv.sc = REMOTE_SCALARS_MAKE(1, 1, 0);
	ioctl.inv.pra = ra;
	ioctl.fds = 0;
	VERIFY(err, 0 == (err = fastrpc_internal_invoke(me,
		FASTRPC_MODE_PARALLEL, 1, &ioctl, fdata)));
	return err;
}

static int fastrpc_mmap_on_dsp(struct fastrpc_apps *me,
					 struct fastrpc_ioctl_mmap *mmap,
					 struct smq_phy_page *pages,
					 struct file_data *fdata, int num)
{
	struct fastrpc_ioctl_invoke_fd ioctl;
	remote_arg_t ra[3];
	int err = 0;
	struct {
		int pid;
		uint32_t flags;
		uintptr_t vaddrin;
		int num;
	} inargs;

	struct {
		uintptr_t vaddrout;
	} routargs;
	inargs.pid = current->tgid;
	inargs.vaddrin = (uintptr_t)mmap->vaddrin;
	inargs.flags = mmap->flags;
	inargs.num = me->compat ? num * sizeof(*pages) : num;
	ra[0].buf.pv = &inargs;
	ra[0].buf.len = sizeof(inargs);

	ra[1].buf.pv = pages;
	ra[1].buf.len = num * sizeof(*pages);

	ra[2].buf.pv = &routargs;
	ra[2].buf.len = sizeof(routargs);

	ioctl.inv.handle = 1;
	if (me->compat)
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(4, 2, 1);
	else
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(2, 2, 1);
	ioctl.inv.pra = ra;
	ioctl.fds = 0;
	VERIFY(err, 0 == (err = fastrpc_internal_invoke(me,
		FASTRPC_MODE_PARALLEL, 1, &ioctl, fdata)));
	mmap->vaddrout = (uintptr_t)routargs.vaddrout;
	if (err)
		goto bail;
bail:
	return err;
}

static int fastrpc_munmap_on_dsp(struct fastrpc_apps *me,
				 struct fastrpc_ioctl_munmap *munmap,
				struct file_data *fdata)
{
	struct fastrpc_ioctl_invoke_fd ioctl;
	remote_arg_t ra[1];
	int err = 0;
	struct {
		int pid;
		uintptr_t vaddrout;
		ssize_t size;
	} inargs;

	inargs.pid = current->tgid;
	inargs.size = munmap->size;
	inargs.vaddrout = munmap->vaddrout;
	ra[0].buf.pv = &inargs;
	ra[0].buf.len = sizeof(inargs);

	ioctl.inv.handle = 1;
	if (me->compat)
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(5, 1, 0);
	else
		ioctl.inv.sc = REMOTE_SCALARS_MAKE(3, 1, 0);
	ioctl.inv.pra = ra;
	ioctl.fds = 0;
	VERIFY(err, 0 == (err = fastrpc_internal_invoke(me,
		FASTRPC_MODE_PARALLEL, 1, &ioctl, fdata)));
	return err;
}

static int fastrpc_internal_munmap(struct fastrpc_apps *me,
				   struct file_data *fdata,
				   struct fastrpc_ioctl_munmap *munmap)
{
	int err = 0;
	struct fastrpc_mmap *map = 0, *mapfree = 0;
	struct hlist_node *n;

	spin_lock(&fdata->hlock);
	hlist_for_each_entry_safe(map, n, &fdata->hlst, hn) {
		if (map->vaddrout == munmap->vaddrout &&
		    map->size == munmap->size && --map->refs == 0) {
			hlist_del(&map->hn);
			mapfree = map;
			break;
		}
	}
	spin_unlock(&fdata->hlock);
	if (mapfree) {
		VERIFY(err, 0 == (err = fastrpc_munmap_on_dsp(me, munmap,
								fdata)));
		free_map(mapfree, fdata);
		kfree(mapfree);
	}
	return err;
}

static int map_buffer(struct fastrpc_apps *me, struct file_data *fdata,
			int fd, char *buf, unsigned long len,
			struct fastrpc_mmap **ppmap,
			struct smq_phy_page **ppages, int *pnpages)
{
	struct ion_client *clnt = gfa.iclient;
	struct ion_handle *handle = 0;
	struct fastrpc_mmap *map = 0, *mapmatch = 0;
	struct smq_phy_page *pages = 0;
	struct hlist_node *n;
	uintptr_t vaddrout = 0;
	int num;
	int err = 0;

	handle = ion_import_dma_buf(clnt, fd);
	VERIFY(err, 0 == IS_ERR_OR_NULL(handle));
	if (err)
		goto bail;
	spin_lock(&fdata->hlock);
	hlist_for_each_entry_safe(map, n, &fdata->hlst, hn) {
		if (map->handle == handle) {
			map->refs++;
			mapmatch = map;
			break;
		}
	}
	spin_unlock(&fdata->hlock);
	if (mapmatch) {
		vaddrout = mapmatch->vaddrout;
		return 0;
	}
	VERIFY(err, 0 != (map = kzalloc(sizeof(*map), GFP_KERNEL)));
	if (err)
		goto bail;
	map->handle = handle;
	handle = 0;
	map->virt = ion_map_kernel(clnt, map->handle);
	VERIFY(err, 0 == IS_ERR_OR_NULL(map->virt));
	if (err)
		goto bail;
	num = buf_num_pages(buf, len);
	VERIFY(err, 0 != (pages = kzalloc(num * sizeof(*pages), GFP_KERNEL)));
	if (err)
		goto bail;

	if (me->channel[fdata->cid].smmu.enabled) {
		VERIFY(err, 0 == map_iommu_mem(map->handle, fdata,
						&map->phys, len));
		if (err)
			goto bail;
		pages->addr = map->phys;
		pages->size = len;
		num = 1;
	} else {
		VERIFY(err, 0 < (num = buf_get_pages(buf, len, num, 1,
						pages, num, &me->range)));
		if (err)
			goto bail;
	}
	map->refs = 1;
	INIT_HLIST_NODE(&map->hn);
	map->vaddrin = (uintptr_t *)buf;
	map->vaddrout = vaddrout;
	map->size = len;
	if (ppages)
		*ppages = pages;
	pages = 0;
	if (pnpages)
		*pnpages = num;
	if (ppmap)
		*ppmap = map;
	map = 0;
 bail:
	if (map)
		free_map(map, fdata);
	kfree(pages);
	return err;

}

static int fastrpc_internal_mmap(struct fastrpc_apps *me,
				 struct file_data *fdata,
				 struct fastrpc_ioctl_mmap *mmap)
{

	struct fastrpc_mmap *map = 0;
	struct smq_phy_page *pages = 0;
	int num = 0;
	int err = 0;
	VERIFY(err, 0 == map_buffer(me, fdata, mmap->fd, (char *)mmap->vaddrin,
					mmap->size, &map, &pages, &num));
	VERIFY(err, 0 == fastrpc_mmap_on_dsp(me, mmap, pages, fdata, num));
	if (err)
		goto bail;
	map->vaddrout = mmap->vaddrout;
	spin_lock(&fdata->hlock);
	hlist_add_head(&map->hn, &fdata->hlst);
	spin_unlock(&fdata->hlock);
 bail:
	if (err && map) {
		free_map(map, fdata);
		kfree(map);
	}
	kfree(pages);
	return err;
}

static void cleanup_current_dev(struct file_data *fdata)
{
	struct fastrpc_apps *me = &gfa;
	uint32_t h = hash_32(current->tgid, RPC_HASH_BITS);
	struct hlist_head *head;
	struct hlist_node *n;
	struct fastrpc_device *dev, *devfree;

 rnext:
	devfree = dev = 0;
	spin_lock(&me->hlock);
	head = &me->htbl[h];
	hlist_for_each_entry_safe(dev, n, head, hn) {
		if (dev->tgid == current->tgid) {
			hlist_del(&dev->hn);
			devfree = dev;
			break;
		}
	}
	spin_unlock(&me->hlock);
	if (devfree) {
		free_dev(devfree, fdata);
		goto rnext;
	}
	return;
}

static void fastrpc_channel_close(struct kref *kref)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_channel_context *ctx;
	int cid;

<<<<<<< HEAD
	ctx = container_of(kref, struct fastrpc_channel_context, kref);
	smd_close(ctx->chan);
=======
	ctx = container_of(kref, struct fastrpc_channel_ctx, kref);
	if (!me->glink) {
		smd_close(ctx->chan);
	} else {
		glink_unregister_link_state_cb(ctx->link_notify_handle);
		glink_close(ctx->chan);
	}
>>>>>>> 0e91d2a... Nougat
	ctx->chan = 0;
	mutex_unlock(&me->smd_mutex);
	cid = ctx - &me->channel[0];
	pr_info("'closed /dev/%s c %d %d'\n", gcinfo[cid].name,
						MAJOR(me->dev_no), cid);
}

static int fastrpc_device_release(struct inode *inode, struct file *file)
{
	struct file_data *fdata = (struct file_data *)file->private_data;
	struct fastrpc_apps *me = &gfa;
	struct smq_context_list *clst = &me->clst;
	struct smq_invoke_ctx *ictx = 0, *ctxfree;
	struct hlist_node *n;
	struct fastrpc_mmap *map = 0;
	int cid = MINOR(inode->i_rdev);

	if (!fdata)
		return 0;

<<<<<<< HEAD
	(void)fastrpc_release_current_dsp_process(fdata);
	do {
		ctxfree = 0;
		spin_lock(&clst->hlock);
		hlist_for_each_entry_safe(ictx, n, &clst->interrupted, hn) {
			if ((ictx->tgid == current->tgid) &&
				(ictx->fdata->cid == cid)) {
				hlist_del(&ictx->hn);
				ctxfree = ictx;
				break;
			}
		}
		spin_unlock(&clst->hlock);
		if (ctxfree)
			context_free(ctxfree, 0);
	} while (ctxfree);

	cleanup_current_dev(fdata);
	file->private_data = 0;
	hlist_for_each_entry_safe(map, n, &fdata->hlst, hn) {
		hlist_del(&map->hn);
		free_map(map, fdata);
		kfree(map);
	}
	if (fdata->ssrcount == me->channel[cid].ssrcount)
		kref_put_mutex(&me->channel[cid].kref,
				fastrpc_channel_close, &me->smd_mutex);
	kfree(fdata);
	return 0;
=======
	(void)fastrpc_release_current_dsp_process(fl);
	fastrpc_context_list_dtor(fl);
	fastrpc_buf_list_free(fl);
	hlist_for_each_entry_safe(map, n, &fl->maps, hn) {
		fastrpc_mmap_free(map);
	}
	if (fl->ssrcount == fl->apps->channel[cid].ssrcount)
		kref_put_mutex(&fl->apps->channel[cid].kref,
				fastrpc_channel_close, &fl->apps->smd_mutex);
	kfree(fl);
	return 0;
}

static int fastrpc_session_alloc(struct fastrpc_channel_ctx *chan, int *session)
{
	struct fastrpc_apps *me = &gfa;
	int idx = 0, err = 0;

	if (chan->sesscount) {
		idx = ffz(chan->bitmap);
		VERIFY(err, idx < chan->sesscount);
		if (err)
			goto bail;
		set_bit(idx, &chan->bitmap);
	} else {
		VERIFY(err, me->dev != NULL);
		if (err)
			goto bail;
		chan->session[0].dev = me->dev;
	}

	chan->session[idx].smmu.faults = 0;
	*session = idx;
 bail:
	return err;
}

static int fastrpc_session_free(struct fastrpc_channel_ctx *chan, int session)
{
	int err = 0;

	if (chan->sesscount) {
		VERIFY(err, session < chan->sesscount);
		if (err)
			goto bail;
		clear_bit(session, &chan->bitmap);
	}
 bail:
	return err;
}

bool fastrpc_glink_notify_rx_intent_req(void *h, const void *priv, size_t size)
{
	if (0 != glink_queue_rx_intent(h, NULL, size))
		return false;
	return true;
}

void fastrpc_glink_notify_tx_done(void *handle, const void *priv,
		const void *pkt_priv, const void *ptr)
{
}

void fastrpc_glink_notify_rx(void *handle, const void *priv,
	const void *pkt_priv, const void *ptr, size_t size)
{
	struct smq_invoke_rsp *rsp = (struct smq_invoke_rsp *)ptr;
	int len = size;

	while (len >= sizeof(*rsp) && rsp) {
		context_notify_user(uint64_to_ptr(rsp->ctx), rsp->retval);
		rsp++;
		len = len - sizeof(*rsp);
	}
	glink_rx_done(handle, ptr, true);
}

void fastrpc_glink_notify_state(void *handle, const void *priv, unsigned event)
{
	struct fastrpc_apps *me = &gfa;
	int cid = (int)(uintptr_t)priv;

	switch (event) {
	case GLINK_CONNECTED:
		complete(&me->channel[cid].work);
		break;
	case GLINK_LOCAL_DISCONNECTED:
		break;
	case GLINK_REMOTE_DISCONNECTED:
		fastrpc_notify_drivers(me, cid);
		break;
	}
}

static int fastrpc_device_release(struct inode *inode, struct file *file)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_file *fl = (struct fastrpc_file *)file->private_data;
	struct fastrpc_fl *pfl = 0;
	int session, cid;
	int err = 0;

	VERIFY(err, pfl = kzalloc(sizeof(*pfl), GFP_KERNEL));
	if (err)
		goto bail;
	INIT_HLIST_NODE(&pfl->hn);
	if (fl) {
		cid = fl->cid;
		if (fl->sctx) {
			session = fl->sctx - &me->channel[cid].session[0];
			fastrpc_session_free(&me->channel[cid], session);
		}
		mutex_lock(&me->flfree_mutex);
		pfl->fl = fl;
		hlist_add_head(&pfl->hn, &me->fls);
		VERIFY(err, me->wq);
		if (err) {
			hlist_del_init(&pfl->hn);
			mutex_unlock(&me->flfree_mutex);
			goto bail;
		}
		if (!work_busy(&me->free_work)) {
			INIT_WORK(&me->free_work, file_free_work_handler);
			VERIFY(err, queue_work(me->wq, &me->free_work));
			if (err) {
				hlist_del_init(&pfl->hn);
				mutex_unlock(&me->flfree_mutex);
				goto bail;
			}
		}
		me->pending_free++;
		mutex_unlock(&me->flfree_mutex);
		file->private_data = 0;
	}
bail:
	if (err)
		kfree(pfl);
	return err;
}


static void file_free_work_handler(struct work_struct *w)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_fl *fl = 0, *freefl = 0;
	struct hlist_node *n = 0;

	while (1) {
		mutex_lock(&me->flfree_mutex);
		hlist_for_each_entry_safe(fl, n, &me->fls, hn) {
			hlist_del_init(&fl->hn);
			freefl = fl;
			break;
		}
		mutex_unlock(&me->flfree_mutex);
		fastrpc_file_free(freefl->fl);
		mutex_lock(&me->flfree_mutex);

		if (hlist_empty(&me->fls)) {
			me->pending_free = 0;
			wake_up_interruptible_all(&wait_queue);
			mutex_unlock(&me->flfree_mutex);
			break;
		}
		mutex_unlock(&me->flfree_mutex);
		kfree(freefl);
	}
	return;
}

static void fastrpc_glink_register_cb(struct glink_link_state_cb_info *cb_info,
					 void *priv)
{
	switch (cb_info->link_state) {
	case GLINK_LINK_STATE_UP:
		if (priv)
			complete(priv);
		break;
	case GLINK_LINK_STATE_DOWN:
		break;
	default:
		pr_err("adsprpc: unknown glnk state %d\n", cb_info->link_state);
		break;
	}
}

static int fastrpc_glink_open(int cid, struct fastrpc_apps *me)
{
	int err = 0;
	struct glink_open_config *cfg = &me->channel[cid].cfg;
	struct glink_link_info *link_info = &me->channel[cid].link_info;

	link_info->edge = gcinfo[cid].edge;
	link_info->transport = "smem";
	link_info->glink_link_state_notif_cb = fastrpc_glink_register_cb;
	me->channel[cid].link_notify_handle = glink_register_link_state_cb(
					&me->channel[cid].link_info,
					(void *)(&me->channel[cid].work));
	VERIFY(err, !IS_ERR_OR_NULL(me->channel[cid].link_notify_handle));
	if (err)
		goto bail;

	VERIFY(err, wait_for_completion_timeout(&me->channel[cid].work,
						RPC_TIMEOUT));
	if (err)
		goto bail;

	cfg->priv = (void *)(uintptr_t)cid;
	cfg->edge = gcinfo[cid].edge;
	cfg->name = FASTRPC_GLINK_GUID;
	cfg->notify_rx = fastrpc_glink_notify_rx;
	cfg->notify_tx_done = fastrpc_glink_notify_tx_done;
	cfg->notify_state = fastrpc_glink_notify_state;
	cfg->notify_rx_intent_req = fastrpc_glink_notify_rx_intent_req;
	VERIFY(err, 0 != (me->channel[cid].chan = glink_open(cfg)));
bail:
	return err;
>>>>>>> 0e91d2a... Nougat
}

static int fastrpc_device_open(struct inode *inode, struct file *filp)
{
	int cid = MINOR(inode->i_rdev);
<<<<<<< HEAD
	int err = 0, ssrcount;
	struct fastrpc_apps *me = &gfa;

	mutex_lock(&me->smd_mutex);
	ssrcount = me->channel[cid].ssrcount;
	if ((kref_get_unless_zero(&me->channel[cid].kref) == 0) ||
		(me->channel[cid].chan == 0)) {
		VERIFY(err, 0 == smd_named_open_on_edge(
					FASTRPC_SMD_GUID,
					gcinfo[cid].channel,
					&me->channel[cid].chan,
					(void *)(uintptr_t)cid,
					smd_event_handler));
		if (err)
			goto smd_bail;
		VERIFY(err, 0 != wait_for_completion_timeout(
							&me->channel[cid].work,
							RPC_TIMEOUT));
		if (err)
			goto completion_bail;
=======
	int err = 0, session;
	int event;
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_file *fl = 0;

	if (me->pending_free) {
		event = wait_event_interruptible_timeout(wait_queue,
						me->pending_free, RPC_TIMEOUT);
		if (event == 0)
			pr_err("timed out..list is still not empty\n");
	}

	VERIFY(err, fl = kzalloc(sizeof(*fl), GFP_KERNEL));
	if (err)
		return err;

	filp->private_data = fl;

	mutex_lock(&me->smd_mutex);

	context_list_ctor(&fl->clst);
	spin_lock_init(&fl->hlock);
	INIT_HLIST_HEAD(&fl->maps);
	INIT_HLIST_HEAD(&fl->bufs);
	INIT_HLIST_NODE(&fl->hn);
	fl->tgid = current->tgid;
	fl->apps = me;
	fl->cid = cid;
	VERIFY(err, !fastrpc_session_alloc(&me->channel[cid], &session));
	if (err)
		goto bail;
	fl->sctx = &me->channel[cid].session[session];

	fl->ssrcount = me->channel[cid].ssrcount;
	if ((kref_get_unless_zero(&me->channel[cid].kref) == 0) ||
	    (me->channel[cid].chan == 0)) {
		if (me->glink) {
			VERIFY(err, 0 == fastrpc_glink_open(cid, me));
		} else {
			VERIFY(err, !smd_named_open_on_edge(FASTRPC_SMD_GUID,
				    gcinfo[cid].channel,
				    (smd_channel_t **)&me->channel[cid].chan,
				    (void *)(uintptr_t)cid,
				    smd_event_handler));
		}
		if (err)
			goto bail;

		VERIFY(err, wait_for_completion_timeout(&me->channel[cid].work,
							RPC_TIMEOUT));
		if (err) {
			me->channel[cid].chan = 0;
			goto bail;
		}
>>>>>>> 0e91d2a... Nougat
		kref_init(&me->channel[cid].kref);
		pr_info("'opened /dev/%s c %d %d'\n", gcinfo[cid].name,
						MAJOR(me->dev_no), cid);
	}
	mutex_unlock(&me->smd_mutex);

	filp->private_data = 0;
	if (0 != try_module_get(THIS_MODULE)) {
		struct file_data *fdata = 0;
		/* This call will cause a dev to be created
		 * which will addref this module
		 */
		VERIFY(err, 0 != (fdata = kzalloc(sizeof(*fdata), GFP_KERNEL)));
		if (err)
			goto bail;

		spin_lock_init(&fdata->hlock);
		INIT_HLIST_HEAD(&fdata->hlst);
		fdata->cid = cid;
		fdata->tgid = current->tgid;
		fdata->ssrcount = ssrcount;

		filp->private_data = fdata;
bail:
		if (err) {
			if (fdata) {
				cleanup_current_dev(fdata);
				kfree(fdata);
			}
			kref_put_mutex(&me->channel[cid].kref,
					fastrpc_channel_close, &me->smd_mutex);
		}
		module_put(THIS_MODULE);
	}
	return err;

completion_bail:
	smd_close(me->channel[cid].chan);
	me->channel[cid].chan = 0;
smd_bail:
	mutex_unlock(&me->smd_mutex);
	return err;
}


static long fastrpc_device_ioctl(struct file *file, unsigned int ioctl_num,
				 unsigned long ioctl_param)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_ioctl_invoke_fd invokefd;
	struct fastrpc_ioctl_mmap mmap;
	struct fastrpc_ioctl_munmap munmap;
	struct fastrpc_ioctl_init init;
	void *param = (char *)ioctl_param;
	struct file_data *fdata = (struct file_data *)file->private_data;
	int size = 0, err = 0;

	switch (ioctl_num) {
	case FASTRPC_IOCTL_INVOKE_FD:
	case FASTRPC_IOCTL_INVOKE:
		invokefd.fds = 0;
		size = (ioctl_num == FASTRPC_IOCTL_INVOKE) ?
				sizeof(invokefd.inv) : sizeof(invokefd);
		VERIFY(err, 0 == copy_from_user(&invokefd, param, size));
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_internal_invoke(me, fdata->mode,
						0, &invokefd, fdata)));
		if (err)
			goto bail;
		break;
	case FASTRPC_IOCTL_MMAP:
		VERIFY(err, 0 == copy_from_user(&mmap, param,
						sizeof(mmap)));
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_internal_mmap(me, fdata,
							      &mmap)));
		if (err)
			goto bail;
		VERIFY(err, 0 == copy_to_user(param, &mmap, sizeof(mmap)));
		if (err)
			goto bail;
		break;
	case FASTRPC_IOCTL_MUNMAP:
		VERIFY(err, 0 == copy_from_user(&munmap, param,
						sizeof(munmap)));
		if (err)
			goto bail;
		VERIFY(err, 0 == (err = fastrpc_internal_munmap(me, fdata,
								&munmap)));
		if (err)
			goto bail;
		break;
	case FASTRPC_IOCTL_SETMODE:
		switch ((uint32_t)ioctl_param) {
		case FASTRPC_MODE_PARALLEL:
		case FASTRPC_MODE_SERIAL:
			fdata->mode = (uint32_t)ioctl_param;
			break;
		default:
			err = -ENOTTY;
			break;
		}
		break;
	case FASTRPC_IOCTL_INIT:
		VERIFY(err, 0 == copy_from_user(&init, param,
						sizeof(init)));
		if (err)
			goto bail;
		VERIFY(err, 0 == fastrpc_init_process(fdata, &init));
		if (err)
			goto bail;
		break;

	default:
		err = -ENOTTY;
		break;
	}
 bail:
	return err;
}

static int fastrpc_restart_notifier_cb(struct notifier_block *nb,
					unsigned long code,
					void *data)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_channel_context *ctx;
	int cid;

	ctx = container_of(nb, struct fastrpc_channel_context, nb);
	cid = ctx - &me->channel[0];
	if (code == SUBSYS_BEFORE_SHUTDOWN) {
		mutex_lock(&me->smd_mutex);
		ctx->ssrcount++;
		if (ctx->chan) {
			if (me->glink) {
				glink_unregister_link_state_cb(
					ctx->link_notify_handle);
				glink_close(ctx->chan);
			} else {
				smd_close(ctx->chan);
			}
			ctx->chan = 0;
			pr_info("'restart notifier: closed /dev/%s c %d %d'\n",
				 gcinfo[cid].name, MAJOR(me->dev_no), cid);
		}
		mutex_unlock(&me->smd_mutex);
		context_notify_all_users(&me->clst, cid);
	}

	return NOTIFY_DONE;
}

static const struct file_operations fops = {
	.open = fastrpc_device_open,
	.release = fastrpc_device_release,
	.unlocked_ioctl = fastrpc_device_ioctl,
	.compat_ioctl = compat_fastrpc_device_ioctl,
};

<<<<<<< HEAD
=======
static struct of_device_id fastrpc_match_table[] = {
	{ .compatible = "qcom,msm-fastrpc-adsp", },
	{ .compatible = "qcom,msm-fastrpc-compute-cb", },
	{ .compatible = "qcom,msm-fastrpc-legacy-compute-cb", },
	{ .compatible = "qcom,msm-adsprpc-mem-region", },
	{}
};

static int fastrpc_cb_probe(struct device *dev)
{
	struct fastrpc_channel_ctx *chan;
	struct fastrpc_session_ctx *sess;
	struct of_phandle_args iommuspec;
	const char *name;
	int err = 0, i;
	int disable_htw = 1;

	VERIFY(err, 0 != (name = of_get_property(dev->of_node, "label", NULL)));
	if (err)
		goto bail;
	for (i = 0; i < NUM_CHANNELS; i++) {
		if (!gcinfo[i].name)
			continue;
		if (!strcmp(name, gcinfo[i].name))
			break;
	}
	VERIFY(err, i < NUM_CHANNELS);
	if (err)
		goto bail;
	chan = &gcinfo[i];
	VERIFY(err, chan->sesscount < NUM_SESSIONS);
	if (err)
		goto bail;

	VERIFY(err, !of_parse_phandle_with_args(dev->of_node, "iommus",
						"#iommu-cells", 0, &iommuspec));
	if (err)
		goto bail;
	sess = &chan->session[chan->sesscount];
	sess->smmu.cb = iommuspec.args[0];
	VERIFY(err, !IS_ERR_OR_NULL(sess->smmu.mapping =
				arm_iommu_create_mapping(&platform_bus_type,
						0x80000000, 0x7fffffff)));
	if (err)
		goto bail;
	iommu_domain_set_attr(sess->smmu.mapping->domain,
				DOMAIN_ATTR_COHERENT_HTW_DISABLE,
				&disable_htw);
	iommu_set_fault_handler(sess->smmu.mapping->domain,
				fastrpc_smmu_fault_handler, sess);
	VERIFY(err, !arm_iommu_attach_device(dev, sess->smmu.mapping));
	if (err)
		goto bail;
	sess->dev = dev;
	sess->smmu.enabled = 1;
	chan->sesscount++;
bail:
	return err;
}

static int fastrpc_cb_legacy_probe(struct device *dev)
{
	struct device_node *domains_child_node = NULL;
	struct device_node *ctx_node = NULL;
	struct fastrpc_channel_ctx *chan;
	struct fastrpc_session_ctx *first_sess, *sess;
	const char *name;
	unsigned int *range = 0, range_size = 0;
	unsigned int *sids = 0, sids_size = 0;
	int err = 0, ret = 0, i;
	int disable_htw = 1;

	VERIFY(err, 0 != (domains_child_node = of_get_child_by_name(
			dev->of_node,
			"qcom,msm_fastrpc_compute_cb")));
	if (err)
		goto bail;
	VERIFY(err, 0 != (ctx_node = of_parse_phandle(
			domains_child_node,
			"qcom,adsp-shared-phandle", 0)));
	if (err)
		goto bail;
	VERIFY(err, 0 != of_get_property(domains_child_node,
				"qcom,adsp-shared-sids", &sids_size));
	if (err)
		goto bail;
	VERIFY(err, sids = kzalloc(sids_size, GFP_KERNEL));
	if (err)
		goto bail;
	ret = of_property_read_u32_array(domains_child_node,
					"qcom,adsp-shared-sids",
					sids,
					sids_size/sizeof(unsigned int));
	if (ret)
		goto bail;
	VERIFY(err, 0 != (name = of_get_property(ctx_node, "label", NULL)));
	if (err)
		goto bail;
	VERIFY(err, 0 != of_get_property(domains_child_node,
					"qcom,virtual-addr-pool", &range_size));
	if (err)
		goto bail;
	VERIFY(err, range = kzalloc(range_size, GFP_KERNEL));
	if (err)
		goto bail;
	ret = of_property_read_u32_array(domains_child_node,
					"qcom,virtual-addr-pool",
					range,
					range_size/sizeof(unsigned int));
	if (ret)
		goto bail;

	chan = &gcinfo[0];
	VERIFY(err, chan->sesscount < NUM_SESSIONS);
	if (err)
		goto bail;
	first_sess = &chan->session[chan->sesscount];
	first_sess->dev = msm_iommu_get_ctx(name);
	VERIFY(err, !IS_ERR_OR_NULL(first_sess->smmu.mapping =
			arm_iommu_create_mapping(
				msm_iommu_get_bus(first_sess->dev),
				range[0], range[1])));
	if (err)
		goto bail;
	iommu_domain_set_attr(first_sess->smmu.mapping->domain,
			DOMAIN_ATTR_COHERENT_HTW_DISABLE,
			&disable_htw);
	VERIFY(err, !arm_iommu_attach_device(first_sess->dev,
					first_sess->smmu.mapping));
	if (err)
		goto bail;
	VERIFY(err, (sids_size/sizeof(unsigned int)) <= NUM_SESSIONS);
	if (err)
		goto bail;
	for (i = 0; i < sids_size/sizeof(unsigned int); i++) {
		VERIFY(err, chan->sesscount < NUM_SESSIONS);
		if (err)
			goto bail;
		sess = &chan->session[chan->sesscount];
		sess->smmu.cb = sids[i];
		sess->dev = first_sess->dev;
		sess->smmu.enabled = 1;
		sess->smmu.mapping = first_sess->smmu.mapping;
		chan->sesscount++;
	}
bail:
	kfree(sids);
	kfree(range);
	return err;
}

static int fastrpc_probe(struct platform_device *pdev)
{
	int err = 0;
	struct fastrpc_apps *me = &gfa;
	struct device *dev = &pdev->dev;

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-fastrpc-compute-cb"))
		return fastrpc_cb_probe(dev);

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-fastrpc-legacy-compute-cb"))
		return fastrpc_cb_legacy_probe(dev);

	if (of_device_is_compatible(dev->of_node,
					"qcom,msm-adsprpc-mem-region")) {
		me->dev = dev;
		me->channel[0].remoteheap_ramdump_dev =
				create_ramdump_device("adsp_rh", dev);
		if (IS_ERR_OR_NULL(me->channel[0].remoteheap_ramdump_dev)) {
			pr_err("ADSPRPC: Unable to create adsp-remoteheap ramdump device.\n");
			me->channel[0].remoteheap_ramdump_dev = NULL;
		}
		return 0;
	}

	me->glink = of_property_read_bool(dev->of_node, "qcom,fastrpc-glink");
	pr_info("adsprpc: channel link type: %d\n", me->glink);

	VERIFY(err, !of_platform_populate(pdev->dev.of_node,
					  fastrpc_match_table,
					  NULL, &pdev->dev));
	if (err)
		goto bail;
bail:
	return err;
}

static void fastrpc_deinit(void)
{
	struct fastrpc_apps *me = &gfa;
	struct fastrpc_channel_ctx *chan = gcinfo;
	int i, j;

	for (i = 0; i < NUM_CHANNELS; i++, chan++) {
		if (chan->chan) {
			kref_put_mutex(&chan->kref,
				fastrpc_channel_close, &me->smd_mutex);
			chan->chan = 0;
		}
		for (j = 0; j < NUM_SESSIONS; j++) {
			struct fastrpc_session_ctx *sess = &chan->session[j];
			if (sess->smmu.enabled) {
				arm_iommu_detach_device(sess->dev);
				sess->dev = 0;
			}
			if (sess->smmu.mapping) {
				arm_iommu_release_mapping(sess->smmu.mapping);
				sess->smmu.mapping = 0;
			}
		}
	}
}

static struct platform_driver fastrpc_driver = {
	.probe = fastrpc_probe,
	.driver = {
		.name = "fastrpc",
		.owner = THIS_MODULE,
		.of_match_table = fastrpc_match_table,
	},
};

>>>>>>> 0e91d2a... Nougat
static int __init fastrpc_device_init(void)
{
	struct fastrpc_apps *me = &gfa;
	struct device_node *ion_node, *node, *pnode;
	struct platform_device *pdev;
	const u32 *addr;
	uint64_t size;
	uint32_t val;
	int i, err = 0;

	memset(me, 0, sizeof(*me));
<<<<<<< HEAD
	VERIFY(err, 0 == fastrpc_init());
=======

	fastrpc_init(me);
	mutex_init(&me->flfree_mutex);
	me->wq = create_singlethread_workqueue("FILE_FREE");
	INIT_WORK(&me->free_work, file_free_work_handler);

	me->dev = NULL;
	VERIFY(err, 0 == platform_driver_register(&fastrpc_driver));
>>>>>>> 0e91d2a... Nougat
	if (err)
		goto fastrpc_bail;
	VERIFY(err, 0 == alloc_chrdev_region(&me->dev_no, 0, NUM_CHANNELS,
					DEVICE_NAME));
	if (err)
		goto alloc_chrdev_bail;
	cdev_init(&me->cdev, &fops);
	me->cdev.owner = THIS_MODULE;
	VERIFY(err, 0 == cdev_add(&me->cdev, MKDEV(MAJOR(me->dev_no), 0),
				NUM_CHANNELS));
	if (err)
		goto cdev_init_bail;
	me->class = class_create(THIS_MODULE, "fastrpc");
	VERIFY(err, !IS_ERR(me->class));
	if (err)
		goto class_create_bail;
	me->compat = (NULL == fops.compat_ioctl) ? 0 : 1;
	ion_node = of_find_compatible_node(NULL, NULL, "qcom,msm-ion");
	if (ion_node) {
		for_each_available_child_of_node(ion_node, node) {
			if (of_property_read_u32(node, "reg", &val))
				continue;
			if (val != ION_ADSP_HEAP_ID)
				continue;
			pdev = of_find_device_by_node(node);
			if (!pdev)
				break;
			pnode = of_parse_phandle(node,
					"linux,contiguous-region", 0);
			if (!pnode)
				break;
			addr = of_get_address(pnode, 0, &size, NULL);
			of_node_put(pnode);
			if (!addr)
				break;
			me->range.addr = cma_get_base(&pdev->dev);
			me->range.size = (size_t)size;
			break;
		}
	}
	for (i = 0; i < NUM_CHANNELS; i++) {
		me->channel[i].dev = device_create(me->class, NULL,
					MKDEV(MAJOR(me->dev_no), i),
					NULL, gcinfo[i].name);
		VERIFY(err, !IS_ERR(me->channel[i].dev));
		if (err)
			goto device_create_bail;
		me->channel[i].ssrcount = 0;
		me->channel[i].nb.notifier_call = fastrpc_restart_notifier_cb,
		(void)subsys_notif_register_notifier(gcinfo[i].subsys,
							&me->channel[i].nb);
	}

	return 0;

device_create_bail:
	class_destroy(me->class);
class_create_bail:
	cdev_del(&me->cdev);
cdev_init_bail:
	unregister_chrdev_region(me->dev_no, NUM_CHANNELS);
alloc_chrdev_bail:
	fastrpc_deinit();
fastrpc_bail:
	return err;
}

static void __exit fastrpc_device_exit(void)
{
	struct fastrpc_apps *me = &gfa;
	int i;

	context_list_dtor(me, &me->clst);
	fastrpc_deinit();
	if (me->wq) {
		flush_workqueue(me->wq);
		destroy_workqueue(me->wq);
	}
	for (i = 0; i < NUM_CHANNELS; i++) {
		device_destroy(me->class, MKDEV(MAJOR(me->dev_no), i));
		subsys_notif_unregister_notifier(gcinfo[i].subsys,
						&me->channel[i].nb);
	}
	class_destroy(me->class);
	cdev_del(&me->cdev);
	unregister_chrdev_region(me->dev_no, NUM_CHANNELS);
}

late_initcall(fastrpc_device_init);
module_exit(fastrpc_device_exit);

MODULE_LICENSE("GPL v2");
