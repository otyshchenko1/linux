/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_ARM_XEN_OPS_H
#define _ASM_ARM_XEN_OPS_H

#include <linux/virtio_config.h>
#include <xen/swiotlb-xen.h>
#include <xen/xen-ops.h>

static inline void xen_setup_dma_ops(struct device *dev)
{
	if ((IS_ENABLED(CONFIG_ARM64) && xen_swiotlb_detect()) ||
	    (IS_ENABLED(CONFIG_ARM) && xen_initial_domain()))
		dev->dma_ops = &xen_swiotlb_dma_ops;

#ifdef CONFIG_XEN_VIRTIO
	if (arch_has_restricted_virtio_memory_access() && xen_is_virtio_device(dev))
		xen_virtio_setup_dma_ops(dev);
#endif
}

#endif /* _ASM_ARM_XEN_OPS_H */
