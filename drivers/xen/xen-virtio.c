// SPDX-License-Identifier: GPL-2.0-only
/******************************************************************************
 * Xen virtio driver - enables using virtio devices in Xen guests.
 *
 * Copyright (c) 2021, Juergen Gross <jgross@suse.com>
 */

#include <linux/module.h>
#include <linux/dma-map-ops.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/pfn.h>
#include <linux/virtio_config.h>
#include <xen/xen.h>
#include <xen/grant_table.h>

struct xen_virtio_data {
	/* The ID of backend domain */
	domid_t dev_domid;
	struct device *dev;
	struct list_head list;
	spinlock_t lock;
	/* Is device behaving sane? */
	bool broken;
};

static LIST_HEAD(xen_virtio_devices);
static DEFINE_SPINLOCK(xen_virtio_lock);

#define XEN_GRANT_ADDR_OFF	0x8000000000000000ULL

static inline dma_addr_t grant_to_dma(grant_ref_t grant)
{
	return XEN_GRANT_ADDR_OFF | ((dma_addr_t)grant << PAGE_SHIFT);
}

static inline grant_ref_t dma_to_grant(dma_addr_t dma)
{
	return (grant_ref_t)((dma & ~XEN_GRANT_ADDR_OFF) >> PAGE_SHIFT);
}

static struct xen_virtio_data *find_xen_virtio_data(struct device *dev)
{
	struct xen_virtio_data *data = NULL;
	bool found = false;

	spin_lock(&xen_virtio_lock);

	list_for_each_entry( data, &xen_virtio_devices, list) {
		if (data->dev == dev) {
			found = true;
			break;
		}
	}

	spin_unlock(&xen_virtio_lock);

	return found ? data : NULL;
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
	struct xen_virtio_data *data;
	unsigned int i, n_pages = PFN_UP(size);
	unsigned long pfn;
	grant_ref_t grant;
	void *ret = NULL;

	data = find_xen_virtio_data(dev);
	if (!data)
		return NULL;

	spin_lock(&data->lock);

	if (unlikely(data->broken))
		goto out;

	ret = alloc_pages_exact(n_pages * PAGE_SIZE, gfp);
	if (!ret)
		goto out;

	pfn = virt_to_pfn(ret);

	if (gnttab_alloc_grant_reference_seq(n_pages, &grant)) {
		free_pages_exact(ret, n_pages * PAGE_SIZE);
		ret = NULL;
		goto out;
	}

	for (i = 0; i < n_pages; i++) {
		gnttab_grant_foreign_access_ref(grant + i, data->dev_domid,
						pfn_to_gfn(pfn + i), 0);
	}

	*dma_handle = grant_to_dma(grant);

out:
	spin_unlock(&data->lock);

	return ret;
}

static void xen_virtio_dma_free(struct device *dev, size_t size, void *vaddr,
				dma_addr_t dma_handle, unsigned long attrs)
{
	struct xen_virtio_data *data;
	unsigned int i, n_pages = PFN_UP(size);
	grant_ref_t grant;

	data = find_xen_virtio_data(dev);
	if (!data)
		return;

	spin_lock(&data->lock);

	if (unlikely(data->broken))
		goto out;

	grant = dma_to_grant(dma_handle);

	for (i = 0; i < n_pages; i++) {
		if (unlikely(!gnttab_end_foreign_access_ref(grant + i))) {
			dev_alert(dev, "Grant still in use by backend domain, disabled for further use\n");
			data->broken = true;
			goto out;
		}
	}

	gnttab_free_grant_reference_seq(grant, n_pages);

	free_pages_exact(vaddr, n_pages * PAGE_SIZE);

out:
	spin_unlock(&data->lock);
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
	struct xen_virtio_data *data;
	unsigned int i, n_pages = PFN_UP(size);
	grant_ref_t grant;
	dma_addr_t dma_handle = DMA_MAPPING_ERROR;

	BUG_ON(dir == DMA_NONE);

	data = find_xen_virtio_data(dev);
	if (!data)
		return DMA_MAPPING_ERROR;

	spin_lock(&data->lock);

	if (unlikely(data->broken))
		goto out;

	if (gnttab_alloc_grant_reference_seq(n_pages, &grant))
		goto out;

	for (i = 0; i < n_pages; i++) {
		gnttab_grant_foreign_access_ref(grant + i, data->dev_domid,
				xen_page_to_gfn(page) + i, dir == DMA_TO_DEVICE);
	}

	dma_handle = grant_to_dma(grant) + offset;

out:
	spin_unlock(&data->lock);

	return dma_handle;
}

static void xen_virtio_dma_unmap_page(struct device *dev, dma_addr_t dma_handle,
				      size_t size, enum dma_data_direction dir,
				      unsigned long attrs)
{
	struct xen_virtio_data *data;
	unsigned int i, n_pages = PFN_UP(size);
	grant_ref_t grant;

	BUG_ON(dir == DMA_NONE);

	data = find_xen_virtio_data(dev);
	if (!data)
		return;

	spin_lock(&data->lock);

	if (unlikely(data->broken))
		goto out;

	grant = dma_to_grant(dma_handle);

	for (i = 0; i < n_pages; i++) {
		if (unlikely(!gnttab_end_foreign_access_ref(grant + i))) {
			dev_alert(dev, "Grant still in use by backend domain, disabled for further use\n");
			data->broken = true;
			goto out;
		}
	}

	gnttab_free_grant_reference_seq(grant, n_pages);

out:
	spin_unlock(&data->lock);
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
	return mask == DMA_BIT_MASK(64);
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

bool xen_is_virtio_device(struct device *dev)
{
	/* XXX Handle only DT devices for now */
	if (!dev->of_node)
		return false;

	if (!of_device_is_compatible(dev->of_node, "virtio,mmio"))
		return false;

	return of_property_read_bool(dev->of_node, "xen,dev-domid");
}
EXPORT_SYMBOL_GPL(xen_is_virtio_device);

void xen_virtio_setup_dma_ops(struct device *dev)
{
	struct xen_virtio_data *data;
	uint32_t dev_domid;

	data = find_xen_virtio_data(dev);
	if (data) {
		dev_err(dev, "xen_virtio data is already created\n");
		return;
	}

	if (dev_is_pci(dev)) {
		/* XXX Leave it hard wired to dom0 for now */
		dev_domid = 0;
	} else if (dev->of_node) {
		if (of_property_read_u32(dev->of_node, "xen,dev-domid", &dev_domid)) {
			dev_err(dev, "xen,dev-domid property is not present\n");
			goto err;
		}
	} else
		/* The ACPI case is not supported */
		goto err;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Сannot allocate xen_virtio data\n");
		goto err;
	}
	data->dev_domid = dev_domid;
	data->dev = dev;
	spin_lock_init(&data->lock);

	spin_lock(&xen_virtio_lock);
	list_add(&data->list, &xen_virtio_devices);
	spin_unlock(&xen_virtio_lock);

	dev->dma_ops = &xen_virtio_dma_ops;

	return;

err:
	dev_err(dev, "Сannot set up xen_virtio DMA ops, retain platform DMA ops\n");
}
EXPORT_SYMBOL_GPL(xen_virtio_setup_dma_ops);

MODULE_DESCRIPTION("Xen virtio support driver");
MODULE_AUTHOR("Juergen Gross <jgross@suse.com>");
MODULE_LICENSE("GPL");
