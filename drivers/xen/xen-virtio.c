// SPDX-License-Identifier: GPL-2.0-only
/******************************************************************************
 * Xen virtio driver - enables using virtio devices in Xen guests.
 *
 * Copyright (c) 2021, Juergen Gross <jgross@suse.com>
 */

#include <linux/module.h>
#include <linux/dma-map-ops.h>
#include <linux/pci.h>
#include <linux/pfn.h>
#include <linux/virtio_config.h>
#include <xen/xen.h>
#include <xen/grant_table.h>

#define XEN_GRANT_ADDR_OFF	0x8000000000000000ULL

static inline dma_addr_t grant_to_dma(grant_ref_t grant)
{
	return XEN_GRANT_ADDR_OFF | ((dma_addr_t)grant << PAGE_SHIFT);
}

static inline grant_ref_t dma_to_grant(dma_addr_t dma)
{
	return (grant_ref_t)((dma & ~XEN_GRANT_ADDR_OFF) >> PAGE_SHIFT);
}

/*
 * DMA ops for Xen virtio frontends.
 *
 * Used to act as a kind of software IOMMU for Xen guests by using grants as
 * DMA addresses.
 * Such a DMA address is formed by using the grant reference as a frame
 * number and setting the highest address bit (this bit is for the backend
 * to be able to distinguish it from e.g. a mmio address).
 *
 * Note that for now we hard wire dom0 to be the backend domain. In order to
 * support any domain as backend we'd need to add a way to communicate the
 * domid of this backend, e.g. via Xenstore or via the PCI-device's config
 * space.
 */
static void *xen_virtio_dma_alloc(struct device *dev, size_t size,
				  dma_addr_t *dma_handle, gfp_t gfp,
				  unsigned long attrs)
{
	unsigned int n_pages = PFN_UP(size);
	unsigned int i;
	unsigned long pfn;
	grant_ref_t grant;
	void *ret;

	ret = (void *)__get_free_pages(gfp, get_order(size));
	if (!ret)
		return NULL;

	pfn = virt_to_pfn(ret);

	if (gnttab_alloc_grant_reference_seq(n_pages, &grant)) {
		free_pages((unsigned long)ret, get_order(size));
		return NULL;
	}

	for (i = 0; i < n_pages; i++) {
		gnttab_grant_foreign_access_ref(grant + i, 0,
						pfn_to_gfn(pfn + i), 0);
	}

	*dma_handle = grant_to_dma(grant);

	return ret;
}

static void xen_virtio_dma_free(struct device *dev, size_t size, void *vaddr,
				dma_addr_t dma_handle, unsigned long attrs)
{
	unsigned int n_pages = PFN_UP(size);
	unsigned int i;
	grant_ref_t grant;

	grant = dma_to_grant(dma_handle);

	for (i = 0; i < n_pages; i++)
		gnttab_end_foreign_access_ref(grant + i, 0);

	gnttab_free_grant_reference_seq(grant, n_pages);

	free_pages((unsigned long)vaddr, get_order(size));
}

static struct page *xen_virtio_dma_alloc_pages(struct device *dev, size_t size,
					       dma_addr_t *dma_handle,
					       enum dma_data_direction dir,
					       gfp_t gfp)
{
	WARN_ONCE(1, "xen_virtio_dma_alloc_pages size %ld\n", size);
	return NULL;
}

static void xen_virtio_dma_free_pages(struct device *dev, size_t size,
				      struct page *vaddr, dma_addr_t dma_handle,
				      enum dma_data_direction dir)
{
	WARN_ONCE(1, "xen_virtio_dma_free_pages size %ld\n", size);
}

static dma_addr_t xen_virtio_dma_map_page(struct device *dev, struct page *page,
					  unsigned long offset, size_t size,
					  enum dma_data_direction dir,
					  unsigned long attrs)
{
	grant_ref_t grant;

	if (gnttab_alloc_grant_references(1, &grant))
		return 0;

	gnttab_grant_foreign_access_ref(grant, 0, xen_page_to_gfn(page),
					dir == DMA_TO_DEVICE);

	return grant_to_dma(grant) + offset;
}

static void xen_virtio_dma_unmap_page(struct device *dev, dma_addr_t dma_handle,
				      size_t size, enum dma_data_direction dir,
				      unsigned long attrs)
{
	grant_ref_t grant;

	grant = dma_to_grant(dma_handle);

	gnttab_end_foreign_access_ref(grant, dir == DMA_TO_DEVICE);

	gnttab_free_grant_reference(grant);
}

static int xen_virtio_dma_map_sg(struct device *dev, struct scatterlist *sg,
				 int nents, enum dma_data_direction dir,
				 unsigned long attrs)
{
	WARN_ONCE(1, "xen_virtio_dma_map_sg nents %d\n", nents);
	return -EINVAL;
}

static void xen_virtio_dma_unmap_sg(struct device *dev, struct scatterlist *sg,
				    int nents, enum dma_data_direction dir,
				    unsigned long attrs)
{
	WARN_ONCE(1, "xen_virtio_dma_unmap_sg nents %d\n", nents);
}

static int xen_virtio_dma_dma_supported(struct device *dev, u64 mask)
{
	return 1;
}

static const struct dma_map_ops xen_virtio_dma_ops = {
	.alloc = xen_virtio_dma_alloc,
	.free = xen_virtio_dma_free,
	.alloc_pages = xen_virtio_dma_alloc_pages,
	.free_pages = xen_virtio_dma_free_pages,
	.mmap = dma_common_mmap,
	.get_sgtable = dma_common_get_sgtable,
	.map_page = xen_virtio_dma_map_page,
	.unmap_page = xen_virtio_dma_unmap_page,
	.map_sg = xen_virtio_dma_map_sg,
	.unmap_sg = xen_virtio_dma_unmap_sg,
	.dma_supported = xen_virtio_dma_dma_supported,
};

void xen_virtio_setup_dma_ops(struct device *dev)
{
	dev->dma_ops = &xen_virtio_dma_ops;
}
EXPORT_SYMBOL_GPL(xen_virtio_setup_dma_ops);

MODULE_DESCRIPTION("Xen virtio support driver");
MODULE_AUTHOR("Juergen Gross <jgross@suse.com>");
MODULE_LICENSE("GPL");
