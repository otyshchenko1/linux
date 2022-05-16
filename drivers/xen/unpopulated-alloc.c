// SPDX-License-Identifier: GPL-2.0
#include <linux/errno.h>
#include <linux/genalloc.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/memremap.h>
#include <linux/slab.h>

#include <asm/page.h>

#include <xen/page.h>
#include <xen/xen.h>

static DEFINE_MUTEX(list_lock);
static struct page *page_list;
static unsigned int list_count;

static struct gen_pool *dma_pool;

static struct resource *target_resource;

/*
 * If arch is not happy with system "iomem_resource" being used for
 * the region allocation it can provide it's own view by creating specific
 * Xen resource with unused regions of guest physical address space provided
 * by the hypervisor.
 */
int __weak __init arch_xen_unpopulated_init(struct resource **res)
{
	*res = &iomem_resource;

	return 0;
}

static int fill_list(unsigned int nr_pages)
{
	struct dev_pagemap *pgmap;
	struct resource *res, *tmp_res = NULL;
	void *vaddr;
	unsigned int i, alloc_pages = round_up(nr_pages, PAGES_PER_SECTION);
	struct range mhp_range;
	int ret;

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;

	res->name = "Xen scratch";
	res->flags = IORESOURCE_MEM | IORESOURCE_BUSY;

	mhp_range = mhp_get_pluggable_range(true);

	ret = allocate_resource(target_resource, res,
				alloc_pages * PAGE_SIZE, mhp_range.start, mhp_range.end,
				PAGES_PER_SECTION * PAGE_SIZE, NULL, NULL);
	if (ret < 0) {
		pr_err("Cannot allocate new IOMEM resource\n");
		goto err_resource;
	}

	/*
	 * Reserve the region previously allocated from Xen resource to avoid
	 * re-using it by someone else.
	 */
	if (target_resource != &iomem_resource) {
		pr_err("XEN RESOURCE\n");
		tmp_res = kzalloc(sizeof(*tmp_res), GFP_KERNEL);
		if (!res) {
			ret = -ENOMEM;
			goto err_insert;
		}

		tmp_res->name = res->name;
		tmp_res->start = res->start;
		tmp_res->end = res->end;
		tmp_res->flags = res->flags;

		ret = request_resource(&iomem_resource, tmp_res);
		if (ret < 0) {
			pr_err("Cannot request resource %pR (%d)\n", tmp_res, ret);
			kfree(tmp_res);
			goto err_insert;
		}
	} else
		pr_err("IOMEM RESOURCE\n");

	pr_err(">>> NON-RAM Allocate 0x%llx - 0x%llx (size %lu)\n", res->start, res->end, alloc_pages * PAGE_SIZE);

	pgmap = kzalloc(sizeof(*pgmap), GFP_KERNEL);
	if (!pgmap) {
		ret = -ENOMEM;
		goto err_pgmap;
	}

	pgmap->type = MEMORY_DEVICE_GENERIC;
	pgmap->range = (struct range) {
		.start = res->start,
		.end = res->end,
	};
	pgmap->nr_range = 1;
	pgmap->owner = res;

#ifdef CONFIG_XEN_HAVE_PVMMU
        /*
         * memremap will build page tables for the new memory so
         * the p2m must contain invalid entries so the correct
         * non-present PTEs will be written.
         *
         * If a failure occurs, the original (identity) p2m entries
         * are not restored since this region is now known not to
         * conflict with any devices.
         */
	if (!xen_feature(XENFEAT_auto_translated_physmap)) {
		xen_pfn_t pfn = PFN_DOWN(res->start);

		for (i = 0; i < alloc_pages; i++) {
			if (!set_phys_to_machine(pfn + i, INVALID_P2M_ENTRY)) {
				pr_warn("set_phys_to_machine() failed, no memory added\n");
				ret = -ENOMEM;
				goto err_memremap;
			}
                }
	}
#endif

	vaddr = memremap_pages(pgmap, NUMA_NO_NODE);
	if (IS_ERR(vaddr)) {
		pr_err("Cannot remap memory range\n");
		ret = PTR_ERR(vaddr);
		goto err_memremap;
	}

	for (i = 0; i < alloc_pages; i++) {
		struct page *pg = virt_to_page(vaddr + PAGE_SIZE * i);

		pg->zone_device_data = page_list;
		page_list = pg;
		list_count++;
	}

	return 0;

err_memremap:
	kfree(pgmap);
err_pgmap:
	if (tmp_res) {
		release_resource(tmp_res);
		kfree(tmp_res);
	}
err_insert:
	release_resource(res);
err_resource:
	kfree(res);
	return ret;
}

/**
 * xen_alloc_unpopulated_pages - alloc unpopulated pages
 * @nr_pages: Number of pages
 * @pages: pages returned
 * @return 0 on success, error otherwise
 */
int xen_alloc_unpopulated_pages(unsigned int nr_pages, struct page **pages)
{
	unsigned int i;
	int ret = 0;

	/*
	 * Bail out early if we do not have any suitable resource to allocate
	 * required region from and as the result we won't be able to construct
	 * pages.
	 */
	if (!target_resource)
		return -ENODEV;

	mutex_lock(&list_lock);
	if (list_count < nr_pages) {
		ret = fill_list(nr_pages - list_count);
		if (ret)
			goto out;
	}

	for (i = 0; i < nr_pages; i++) {
		struct page *pg = page_list;

		BUG_ON(!pg);
		page_list = pg->zone_device_data;
		list_count--;
		pages[i] = pg;

#ifdef CONFIG_XEN_HAVE_PVMMU
		if (!xen_feature(XENFEAT_auto_translated_physmap)) {
			ret = xen_alloc_p2m_entry(page_to_pfn(pg));
			if (ret < 0) {
				unsigned int j;

				for (j = 0; j <= i; j++) {
					pages[j]->zone_device_data = page_list;
					page_list = pages[j];
					list_count++;
				}
				goto out;
			}
		}
#endif
	}

out:
	mutex_unlock(&list_lock);
	return ret;
}
EXPORT_SYMBOL(xen_alloc_unpopulated_pages);

/**
 * xen_free_unpopulated_pages - return unpopulated pages
 * @nr_pages: Number of pages
 * @pages: pages to return
 */
void xen_free_unpopulated_pages(unsigned int nr_pages, struct page **pages)
{
	unsigned int i;

	if (!target_resource)
		return;

	mutex_lock(&list_lock);
	for (i = 0; i < nr_pages; i++) {
		pages[i]->zone_device_data = page_list;
		page_list = pages[i];
		list_count++;
	}
	mutex_unlock(&list_lock);
}
EXPORT_SYMBOL(xen_free_unpopulated_pages);

static int fill_dma_pool(unsigned int nr_pages, u64 dma_mask)
{
	struct dev_pagemap *pgmap;
	struct resource *res, *tmp_res = NULL;
	void *vaddr;
	unsigned int alloc_pages = round_up(nr_pages, PAGES_PER_SECTION);
	struct range mhp_range;
	resource_size_t end;
	int ret;

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;

	res->name = "Xen DMA pool";
	res->flags = IORESOURCE_MEM | IORESOURCE_BUSY;

	mhp_range = mhp_get_pluggable_range(true);
	end = min(dma_mask, mhp_range.end);

	ret = allocate_resource(target_resource, res,
				alloc_pages * PAGE_SIZE, mhp_range.start, end,
				PAGES_PER_SECTION * PAGE_SIZE, NULL, NULL);
	if (ret < 0) {
		pr_err("Cannot allocate new IOMEM resource\n");
		goto err_resource;
	}

	/*
	 * Reserve the region previously allocated from Xen resource to avoid
	 * re-using it by someone else.
	 */
	if (target_resource != &iomem_resource) {
		pr_err("XEN RESOURCE\n");
		tmp_res = kzalloc(sizeof(*tmp_res), GFP_KERNEL);
		if (!res) {
			ret = -ENOMEM;
			goto err_insert;
		}

		tmp_res->name = res->name;
		tmp_res->start = res->start;
		tmp_res->end = res->end;
		tmp_res->flags = res->flags;

		ret = request_resource(&iomem_resource, tmp_res);
		if (ret < 0) {
			pr_err("Cannot request resource %pR (%d)\n", tmp_res, ret);
			kfree(tmp_res);
			goto err_insert;
		}
	} else
		pr_err("IOMEM RESOURCE\n");

	pr_err(">>> NON-RAM (CONTIG) Allocate 0x%llx - 0x%llx (size %lu)\n", res->start, res->end, alloc_pages * PAGE_SIZE);

	pgmap = kzalloc(sizeof(*pgmap), GFP_KERNEL);
	if (!pgmap) {
		ret = -ENOMEM;
		goto err_pgmap;
	}

	pgmap->type = MEMORY_DEVICE_GENERIC;
	pgmap->range = (struct range) {
		.start = res->start,
		.end = res->end,
	};
	pgmap->nr_range = 1;
	pgmap->owner = res;

	vaddr = memremap_pages(pgmap, NUMA_NO_NODE);
	if (IS_ERR(vaddr)) {
		pr_err("Cannot remap memory range\n");
		ret = PTR_ERR(vaddr);
		goto err_memremap;
	}

	ret = gen_pool_add_virt(dma_pool, (unsigned long)vaddr, res->start,
			alloc_pages * PAGE_SIZE, NUMA_NO_NODE);
	if (ret)
		goto err_pool;

	printk("%s[%d] %u: 0x%p -> 0x%llx\n", __func__, __LINE__, nr_pages, vaddr, virt_to_phys(vaddr));

	return 0;

err_pool:
	memunmap_pages(pgmap);
err_memremap:
	kfree(pgmap);
err_pgmap:
	if (tmp_res) {
		release_resource(tmp_res);
		kfree(tmp_res);
	}
err_insert:
	release_resource(res);
err_resource:
	kfree(res);
	return ret;
}

/**
 * xen_alloc_unpopulated_dma_pages - alloc unpopulated DMAable pages
 * @dev: valid struct device pointer
 * @nr_pages: Number of pages
 * @pages: pages returned
 * @return 0 on success, error otherwise
 */
int xen_alloc_unpopulated_dma_pages(struct device *dev, unsigned int nr_pages,
		struct page **pages)
{
	void *vaddr;
	bool filled = false;
	unsigned int i;
	int ret;

	if (!dma_pool)
		return -ENODEV;

	while (!(vaddr = (void *)gen_pool_alloc(dma_pool, nr_pages * PAGE_SIZE))) {
		if (filled)
			return -ENOMEM;
		else {
			ret = fill_dma_pool(nr_pages, dma_get_mask(dev));
			if (ret)
				return ret;

			filled = true;
		}
	}

	for (i = 0; i < nr_pages; i++)
		pages[i] = virt_to_page(vaddr + PAGE_SIZE * i);

	printk("%s[%d] %u: 0x%p -> 0x%llx/ 0x%p -> 0x%llx\n", __func__, __LINE__,
			nr_pages, page_to_virt(pages[0]), page_to_phys(pages[0]), vaddr,
			gen_pool_virt_to_phys(dma_pool, (unsigned long)vaddr));

	return 0;
}
EXPORT_SYMBOL(xen_alloc_unpopulated_dma_pages);

/**
 * xen_free_unpopulated_dma_pages - return unpopulated DMAable pages
 * @dev: valid struct device pointer
 * @nr_pages: Number of pages
 * @pages: pages to return
 */
void xen_free_unpopulated_dma_pages(struct device *dev, unsigned int nr_pages,
		struct page **pages)
{
	void *vaddr;

	if (!dma_pool)
		return;

	vaddr = page_to_virt(pages[0]);

	printk("%s[%d] %u: 0x%p -> 0x%llx\n", __func__, __LINE__,
			nr_pages, page_to_virt(pages[0]), page_to_phys(pages[0]));

	gen_pool_free(dma_pool, (unsigned long)vaddr, nr_pages * PAGE_SIZE);
}
EXPORT_SYMBOL(xen_free_unpopulated_dma_pages);

static int __init init(void)
{
	int ret;

#ifdef CONFIG_XEN_PV
	unsigned int i;

	if (!xen_domain())
		return -ENODEV;

	if (!xen_pv_domain())
		return 0;

	/*
	 * Initialize with pages from the extra memory regions (see
	 * arch/x86/xen/setup.c).
	 */
	for (i = 0; i < XEN_EXTRA_MEM_MAX_REGIONS; i++) {
		unsigned int j;

		for (j = 0; j < xen_extra_mem[i].n_pfns; j++) {
			struct page *pg =
				pfn_to_page(xen_extra_mem[i].start_pfn + j);

			pg->zone_device_data = page_list;
			page_list = pg;
			list_count++;
		}
	}
#endif

	ret = arch_xen_unpopulated_init(&target_resource);
	if (ret) {
		pr_err("xen:unpopulated: Cannot initialize target resource\n");
		target_resource = NULL;
		return ret;
	}

	dma_pool = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
	if (!dma_pool) {
		pr_err("xen:unpopulated: Cannot create DMA pool\n");
		return -ENOMEM;
	}

	gen_pool_set_algo(dma_pool, gen_pool_best_fit, NULL);

	return ret;
}
subsys_initcall(init);
