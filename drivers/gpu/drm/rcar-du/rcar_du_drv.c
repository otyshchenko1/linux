/*
 * rcar_du_drv.c  --  R-Car Display Unit DRM driver
 *
 * Copyright (C) 2013-2014 Renesas Electronics Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_encoder_slave.h>
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
#include <drm/rcar_du_drm.h>
#endif /* CONFIG_DRM_RCAR_DU_CONNECT_VSP */

#include "rcar_du_crtc.h"
#include "rcar_du_drv.h"
#include "rcar_du_encoder.h"
#include "rcar_du_kms.h"
#include "rcar_du_regs.h"
#include "rcar_du_lvdsenc.h"
#include "rcar_lvds_regs.h"

#if defined(R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND) || \
	defined(R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND) || \
	defined(R8A7790_ES1_DU_ALIGN_128B_WORKAROUND)
#define PRODUCT_REGISTER	0xFF000044
#define PRODUCT_CUT_MASK	(0x00007FF0)
#define PRODUCT_H2_BIT		(0x45 << 8)
#define PRODUCT_M2_BIT		(0x47 << 8)
#define CUT_ES2X_BIT		(0x00000010)
#endif
/* -----------------------------------------------------------------------------
 * DRM operations
 */

static int rcar_du_unload(struct drm_device *dev)
{
	struct rcar_du_device *rcdu = dev->dev_private;

	if (rcdu->fbdev)
		drm_fbdev_cma_fini(rcdu->fbdev);

	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
	drm_vblank_cleanup(dev);

	dev->irq_enabled = 0;
	dev->dev_private = NULL;

	if (rcdu->pdata->backlight_off)
		rcdu->pdata->backlight_off();

	return 0;
}

static int rcar_du_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct rcar_du_platform_data *pdata = pdev->dev.platform_data;
	struct rcar_du_device *rcdu;
	struct resource *mem;
#if defined(R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND) || \
	defined(R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND) || \
	defined(R8A7790_ES1_DU_ALIGN_128B_WORKAROUND)
	void __iomem *product_reg;
#endif
	int ret;

	if (pdata == NULL) {
		dev_err(dev->dev, "no platform data\n");
		return -ENODEV;
	}

	rcdu = devm_kzalloc(&pdev->dev, sizeof(*rcdu), GFP_KERNEL);
	if (rcdu == NULL) {
		dev_err(dev->dev, "failed to allocate private data\n");
		return -ENOMEM;
	}

	rcdu->dev = &pdev->dev;
	rcdu->pdata = pdata;
	rcdu->info = (struct rcar_du_device_info *)pdev->id_entry->driver_data;
	rcdu->ddev = dev;
	dev->dev_private = rcdu;
	rcdu->dpad0_source = rcdu->info->drgbs_bit;
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	rcdu->vsp_reserve = rcdu->info->vspd_crtc;
#endif

#if defined(R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND) || \
	defined(R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND) || \
	defined(R8A7790_ES1_DU_ALIGN_128B_WORKAROUND)

	product_reg = ioremap_nocache(PRODUCT_REGISTER, 0x04);
	if (!product_reg)
		return -ENOMEM;

#ifdef R8A7790_ES1_DU_ALIGN_128B_WORKAROUND
	/* Add the workaround of 128 byte align in R-Car H2 ES1.0. */
	if ((readl(product_reg) & PRODUCT_CUT_MASK) == PRODUCT_H2_BIT)
		rcdu->info->quirks = rcdu->info->quirks |
					 RCAR_DU_QUIRK_ALIGN_128B;
#endif
#ifdef R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND
	/* Add the workaround of LVDS lane mis-connection in R-Car H2 ES1.x. */
	if ((readl(product_reg) & PRODUCT_CUT_MASK) == PRODUCT_H2_BIT)
		rcdu->info->quirks = rcdu->info->quirks |
					 RCAR_DU_QUIRK_LVDS_LANES;
#endif
#ifdef R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND
	/* Add the workaround of LVDS CH data gap
		in R-Car H2 ES2.x. and R-Car M2 ES2.x. */
	if ((readl(product_reg) & PRODUCT_CUT_MASK)
		 == (PRODUCT_H2_BIT | CUT_ES2X_BIT)) {
		rcdu->info->cpu_clk_time_ps = 1000000000000 / 1400000000;
		rcdu->info->quirks = rcdu->info->quirks |
					 RCAR_DU_QUIRK_LVDS_CH_DATA_GAP;

	} else if ((readl(product_reg) & PRODUCT_CUT_MASK)
		 == (PRODUCT_M2_BIT | CUT_ES2X_BIT)) {
		rcdu->info->cpu_clk_time_ps = 1000000000000 / 1500000000;
		rcdu->info->quirks = rcdu->info->quirks |
					 RCAR_DU_QUIRK_LVDS_CH_DATA_GAP;
	} else
		rcdu->info->cpu_clk_time_ps = 0;
#endif
	iounmap(product_reg);
#endif

	/* I/O resources */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rcdu->mmio = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(rcdu->mmio))
		return PTR_ERR(rcdu->mmio);

	/* DRM/KMS objects */
	ret = rcar_du_modeset_init(rcdu);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialize DRM/KMS\n");
		goto done;
	}

	/* vblank handling */
	ret = drm_vblank_init(dev, (1 << rcdu->num_crtcs) - 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialize vblank\n");
		goto done;
	}

	dev->irq_enabled = 1;

	platform_set_drvdata(pdev, rcdu);

done:
	if (ret)
		rcar_du_unload(dev);

	return ret;
}

static void rcar_du_preclose(struct drm_device *dev, struct drm_file *file)
{
	struct rcar_du_device *rcdu = dev->dev_private;
	unsigned int i;

	for (i = 0; i < rcdu->num_crtcs; ++i)
		rcar_du_crtc_cancel_page_flip(&rcdu->crtcs[i], file);
}

static void rcar_du_lastclose(struct drm_device *dev)
{
	struct rcar_du_device *rcdu = dev->dev_private;

	drm_fbdev_cma_restore_mode(rcdu->fbdev);
}

static int rcar_du_enable_vblank(struct drm_device *dev, int crtc)
{
	struct rcar_du_device *rcdu = dev->dev_private;

	rcar_du_crtc_enable_vblank(&rcdu->crtcs[crtc], true);

	return 0;
}

static void rcar_du_disable_vblank(struct drm_device *dev, int crtc)
{
	struct rcar_du_device *rcdu = dev->dev_private;

	rcar_du_crtc_enable_vblank(&rcdu->crtcs[crtc], false);
}


#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP

int rcar_du_debug_reg(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int i;
	int crtc_id = *(int *)data;
	struct drm_mode_object *obj;
	struct drm_crtc *crtc;
	struct rcar_du_crtc *rcrtc;

	obj = drm_mode_object_find(dev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj)
		return -EINVAL;
	crtc = obj_to_crtc(obj);

	rcrtc = to_rcar_crtc(crtc);

#define rcar_dbg_print(fmt, ...) \
	pr_info(KERN_INFO fmt, ##__VA_ARGS__)

#define group_dbg_print(fmt, rcrtc, reg) \
	rcar_dbg_print(fmt, rcar_du_group_read(rcrtc->group, reg));

#define crtc_dbg_print(fmt, rcrtc, reg) \
do { \
	struct rcar_du_device *rcdu = rcrtc->group->dev; \
	rcar_dbg_print(fmt, rcar_du_read(rcdu, rcrtc->mmio_offset + reg)); \
} while (0)


	rcar_dbg_print("Control Register\n");
	group_dbg_print(" DS1PR    0x%08x\n", rcrtc, DS1PR);
	group_dbg_print(" DS2PR    0x%08x\n", rcrtc, DS2PR);
	group_dbg_print(" DORCR    0x%08x\n", rcrtc, DORCR);
	group_dbg_print(" DPTSR    0x%08x\n", rcrtc, DPTSR);
	group_dbg_print(" DSYSR    0x%08x\n", rcrtc, DSYSR);
	group_dbg_print(" DORCR    0x%08x\n", rcrtc, DORCR);
	group_dbg_print(" ESCR     0x%08x\n", rcrtc, ESCR);
	group_dbg_print(" OTAR     0x%08x\n", rcrtc, OTAR);
	rcar_dbg_print("\n");

	crtc_dbg_print(" DOOR     0x%08x\n", rcrtc, DOOR);
	crtc_dbg_print(" BPOR     0x%08x\n", rcrtc, BPOR);
	crtc_dbg_print(" DSMR     0x%08x\n", rcrtc, DSMR);
	crtc_dbg_print(" HDSR     0x%08x\n", rcrtc, HDSR);
	crtc_dbg_print(" HDER     0x%08x\n", rcrtc, HDER);
	crtc_dbg_print(" HSWR     0x%08x\n", rcrtc, HSWR);
	crtc_dbg_print(" HCR      0x%08x\n", rcrtc, HCR);
	crtc_dbg_print(" VDSR     0x%08x\n", rcrtc, VDSR);
	crtc_dbg_print(" VDER     0x%08x\n", rcrtc, VDER);
	crtc_dbg_print(" VSPR     0x%08x\n", rcrtc, VSPR);
	crtc_dbg_print(" VCR      0x%08x\n", rcrtc, VCR);
	crtc_dbg_print(" DESR     0x%08x\n", rcrtc, DESR);
	crtc_dbg_print(" DEWR     0x%08x\n", rcrtc, DEWR);
	rcar_dbg_print("\n");

#define plane_read(rcrtc, index, reg) \
	rcar_du_read((rcrtc->group)->dev, \
		(rcrtc->group)->mmio_offset + index * PLANE_OFF + reg)

#define plane_dbg_print(fmt, rcrtc, index, reg) \
		rcar_dbg_print(fmt, index + 1, plane_read(rcrtc, index, reg))

	rcar_dbg_print("Plnae Register\n");
	for (i = 0; i < 8; i++) {
		plane_dbg_print(" P%dMR     0x%08x\n", rcrtc, i, PnMR);
		plane_dbg_print(" P%dMWR    0x%08x\n", rcrtc, i, PnMWR);
		plane_dbg_print(" P%dALPHAR 0x%08x\n", rcrtc, i, PnALPHAR);
		plane_dbg_print(" P%dDSXR   0x%08x\n", rcrtc, i, PnDSXR);
		plane_dbg_print(" P%dDSYR   0x%08x\n", rcrtc, i, PnDSYR);
		plane_dbg_print(" P%dDPXR   0x%08x\n", rcrtc, i, PnDPXR);
		plane_dbg_print(" P%dDPYR   0x%08x\n", rcrtc, i, PnDPYR);
		plane_dbg_print(" P%dDSA0R  0x%08x\n", rcrtc, i, PnDSA0R);
		plane_dbg_print(" P%dDSA1R  0x%08x\n", rcrtc, i, PnDSA1R);
		plane_dbg_print(" P%dDSA2R  0x%08x\n", rcrtc, i, PnDSA2R);
		plane_dbg_print(" P%dSPXR   0x%08x\n", rcrtc, i, PnSPXR);
		plane_dbg_print(" P%dSPYR   0x%08x\n", rcrtc, i, PnSPYR);
		plane_dbg_print(" P%dWASPR  0x%08x\n", rcrtc, i, PnWASPR);
		plane_dbg_print(" P%dWAMWR  0x%08x\n", rcrtc, i, PnWAMWR);
		plane_dbg_print(" P%dBTR    0x%08x\n", rcrtc, i, PnBTR);
		plane_dbg_print(" P%dTC1R   0x%08x\n", rcrtc, i, PnTC1R);
		plane_dbg_print(" P%dTC2R   0x%08x\n", rcrtc, i, PnTC2R);
		plane_dbg_print(" P%dTC3R   0x%08x\n", rcrtc, i, PnTC3R);
		plane_dbg_print(" P%dMLR    0x%08x\n", rcrtc, i, PnMLR);
		plane_dbg_print(" P%dSWAPR  0x%08x\n", rcrtc, i, PnSWAPR);
		plane_dbg_print(" P%dDDCR   0x%08x\n", rcrtc, i, PnDDCR);
		plane_dbg_print(" P%dDDC2R  0x%08x\n", rcrtc, i, PnDDCR2);
		plane_dbg_print(" P%dDDC4R  0x%08x\n", rcrtc, i, PnDDCR4);
		rcar_dbg_print("\n");
	}

	rcar_dbg_print("group Register\n");
	group_dbg_print(" DEFR1     0x%08x\n", rcrtc, DEFR);
	group_dbg_print(" DEFR2     0x%08x\n", rcrtc, DEFR2);
	group_dbg_print(" DEFR3     0x%08x\n", rcrtc, DEFR3);
	group_dbg_print(" DEFR4     0x%08x\n", rcrtc, DEFR4);
	group_dbg_print(" DEFR5     0x%08x\n", rcrtc, DEFR5);
	group_dbg_print(" DEFR8     0x%08x\n", rcrtc, DEFR8);

	return 0;
}

int rcar_du_debug_reg_vsp(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	int crtc_id = *(int *)data;
	struct drm_mode_object *obj;
	struct drm_crtc *crtc;
	struct rcar_du_crtc *rcrtc;

	obj = drm_mode_object_find(dev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj)
		return -EINVAL;
	crtc = obj_to_crtc(obj);

	rcrtc = to_rcar_crtc(crtc);

	if (rcrtc->vpsd_handle == NULL)
		return -EINVAL;

	vsp_du_if_reg_debug(rcrtc->vpsd_handle);

	return 0;
}

int rcar_du_screen_shot(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct rcar_du_screen_shot *sh =
		(struct rcar_du_screen_shot *)data;
	struct drm_mode_object *obj;
	struct drm_crtc *crtc;
	struct rcar_du_crtc *rcrtc;

	obj = drm_mode_object_find(dev, sh->crtc_id,
				DRM_MODE_OBJECT_CRTC);
	if (!obj)
		return -EINVAL;
	crtc = obj_to_crtc(obj);

	rcrtc = to_rcar_crtc(crtc);

	if (rcrtc->vpsd_handle == NULL)
		return -EINVAL;

	if (!rcrtc->lif_enable)
		return -EAGAIN;

	return vsp_du_if_write_back(rcrtc->vpsd_handle, sh);
}

static const struct drm_ioctl_desc rcar_du_ioctls[] = {
	DRM_IOCTL_DEF_DRV(RCAR_DU_SET_PLANE_FENCE, drm_noop,
		DRM_UNLOCKED | DRM_CONTROL_ALLOW),

	/* Debug gunction */
	DRM_IOCTL_DEF_DRV(RCAR_DU_DBG, rcar_du_debug_reg,
		DRM_UNLOCKED | DRM_CONTROL_ALLOW),
	DRM_IOCTL_DEF_DRV(RCAR_DU_DBG_VSP, rcar_du_debug_reg_vsp,
		DRM_UNLOCKED | DRM_CONTROL_ALLOW),

	DRM_IOCTL_DEF_DRV(RCAR_DU_SET_DESKTOP, drm_noop,
		DRM_UNLOCKED | DRM_CONTROL_ALLOW),

	DRM_IOCTL_DEF_DRV(RCAR_DU_SET_SCRSHOT, rcar_du_screen_shot,
		DRM_UNLOCKED | DRM_CONTROL_ALLOW),
};
#endif /* CONFIG_DRM_RCAR_DU_CONNECT_VSP */


static const struct file_operations rcar_du_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= no_llseek,
	.mmap		= drm_gem_cma_mmap,
};

static struct drm_driver rcar_du_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME,
	.load			= rcar_du_load,
	.unload			= rcar_du_unload,
	.preclose		= rcar_du_preclose,
	.lastclose		= rcar_du_lastclose,
	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= rcar_du_enable_vblank,
	.disable_vblank		= rcar_du_disable_vblank,
	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,
	.dumb_create		= rcar_du_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.fops			= &rcar_du_fops,
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	.ioctls			= rcar_du_ioctls,
	.num_ioctls		= ARRAY_SIZE(rcar_du_ioctls),
#endif
	.name			= "rcar-du",
	.desc			= "Renesas R-Car Display Unit",
	.date			= "20130110",
	.major			= 1,
	.minor			= 0,
};

/* -----------------------------------------------------------------------------
 * Power management
 */

#ifdef CONFIG_PM_SLEEP
static int rcar_du_pm_suspend(struct device *dev)
{
	struct rcar_du_device *rcdu = dev_get_drvdata(dev);
	struct drm_encoder *encoder;
	int i;

	encoder = NULL;

	drm_kms_helper_poll_disable(rcdu->ddev);

#if defined(CONFIG_DRM_ADV7511) || defined(CONFIG_DRM_ADV7511_MODULE)
	list_for_each_entry(encoder,
			 &rcdu->ddev->mode_config.encoder_list, head) {
		if ((encoder->encoder_type == DRM_MODE_ENCODER_TMDS) &&
			(get_rcar_slave_funcs(encoder)->dpms))
			get_rcar_slave_funcs(encoder)->dpms(encoder,
						 DRM_MODE_DPMS_OFF);
	}
#endif
#ifdef CONFIG_DRM_RCAR_LVDS
	for (i = 0; i < rcdu->info->num_lvds; ++i) {
		if (rcdu->lvds[i])
			rcar_du_lvdsenc_stop_suspend(rcdu->lvds[i]);
	}
#endif
	for (i = 0; i < rcdu->pdata->num_crtcs; ++i)
		rcar_du_crtc_suspend(&rcdu->crtcs[i]);

	return 0;
}

static int rcar_du_pm_resume(struct device *dev)
{
	struct rcar_du_device *rcdu = dev_get_drvdata(dev);
	struct drm_encoder *encoder;
	int i;

	encoder = NULL;

	for (i = 0; i < rcdu->pdata->num_crtcs; ++i)
		rcar_du_crtc_resume(&rcdu->crtcs[i]);

#ifdef CONFIG_DRM_RCAR_LVDS
	for (i = 0; i < rcdu->pdata->num_crtcs; ++i) {
		if (rcdu->crtcs[i].lvds_ch >= 0)
			rcar_du_lvdsenc_start(
					rcdu->lvds[rcdu->crtcs[i].lvds_ch],
					&rcdu->crtcs[i]);
	}
#endif

#if defined(CONFIG_DRM_ADV7511) || defined(CONFIG_DRM_ADV7511_MODULE)
	list_for_each_entry(encoder,
			 &rcdu->ddev->mode_config.encoder_list, head) {
		if ((encoder->encoder_type == DRM_MODE_ENCODER_TMDS) &&
			(get_rcar_slave_funcs(encoder)->dpms))
			get_rcar_slave_funcs(encoder)->dpms(encoder,
						 DRM_MODE_DPMS_ON);
	}
#endif
	drm_kms_helper_poll_enable(rcdu->ddev);

	return 0;
}
#endif

static const struct dev_pm_ops rcar_du_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rcar_du_pm_suspend, rcar_du_pm_resume)
};

/* -----------------------------------------------------------------------------
 * Platform driver
 */

static int rcar_du_probe(struct platform_device *pdev)
{
	return drm_platform_init(&rcar_du_driver, pdev);
}

static int rcar_du_remove(struct platform_device *pdev)
{
	drm_platform_exit(&rcar_du_driver, pdev);

	return 0;
}

#if defined(R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND) || \
	defined(R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND)
static struct rcar_du_device_info rcar_du_r8a7779_info = {
#else
static const struct rcar_du_device_info rcar_du_r8a7779_info = {
#endif
	.features = 0,
	.num_crtcs = 2,
	.routes = {
		/* R8A7779 has two RGB outputs and one (currently unsupported)
		 * TCON output.
		 */
		[RCAR_DU_OUTPUT_DPAD0] = {
			.possible_crtcs = BIT(0),
			.encoder_type = DRM_MODE_ENCODER_NONE,
		},
		[RCAR_DU_OUTPUT_DPAD1] = {
			.possible_crtcs = BIT(1) | BIT(0),
			.encoder_type = DRM_MODE_ENCODER_NONE,
		},
	},
	.num_lvds = 0,
	.chip = RCAR_H1,
};

#if defined(R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND) || \
	defined(R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND) || \
	defined(R8A7790_ES1_DU_ALIGN_128B_WORKAROUND)
static struct rcar_du_device_info rcar_du_r8a7790_info = {
#else
static const struct rcar_du_device_info rcar_du_r8a7790_info = {
#endif
	.features = RCAR_DU_FEATURE_CRTC_IRQ_CLOCK | RCAR_DU_FEATURE_DEFR8 |
		    RCAR_DU_FEATURE_VSP1_SOURCE,
	.num_crtcs = 3,
	.routes = {
		/* R8A7790 has one RGB output, two LVDS outputs and one
		 * (currently unsupported) TCON output.
		 */
		[RCAR_DU_OUTPUT_DPAD0] = {
			.possible_crtcs = BIT(2) | BIT(1) | BIT(0),
#if (defined(CONFIG_DRM_ADV7511) || defined(CONFIG_DRM_ADV7511_MODULE)) \
	&& defined(CONFIG_DRM_RCAR_LVDS)
			.possible_clones = BIT(2) | BIT(0),
#elif defined(CONFIG_DRM_RCAR_LVDS)
			.possible_clones = BIT(1),
#else
			.possible_clones = 0,
#endif
			.encoder_type = DRM_MODE_ENCODER_NONE,
		},
		[RCAR_DU_OUTPUT_LVDS0] = {
			.possible_crtcs = BIT(0),
			.possible_clones = 0,
			.encoder_type = DRM_MODE_ENCODER_LVDS,
		},
		[RCAR_DU_OUTPUT_LVDS1] = {
			.possible_crtcs = BIT(2) | BIT(1),
#if defined(CONFIG_DRM_ADV7511) || defined(CONFIG_DRM_ADV7511_MODULE)
			.possible_clones = BIT(1),
#else
			.possible_clones = 0,
#endif
			.encoder_type = DRM_MODE_ENCODER_LVDS,
		},
	},
	.num_lvds = 2,
	.drgbs_bit = 0,
	.max_xres = 1920,
	.max_yres = 1080,
	.interlace = false,
	.lvds0_crtc = BIT(0),
	.lvds1_crtc = BIT(1) | BIT(2),
	.vspd_crtc = BIT(0) | BIT(1),
	.chip = RCAR_H2,
	.drgbs_use = true,
	.vscs_use = true,
};

#if defined(R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND) || \
	defined(R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND)
static struct rcar_du_device_info rcar_du_r8a7791_info = {
#else
static const struct rcar_du_device_info rcar_du_r8a7791_info = {
#endif
	.features = RCAR_DU_FEATURE_CRTC_IRQ_CLOCK | RCAR_DU_FEATURE_DEFR8 |
		    RCAR_DU_FEATURE_VSP1_SOURCE,
	.num_crtcs = 2,
	.routes = {
		/* R8A7791 has one RGB output, one LVDS output and one
		 * (currently unsupported) TCON output.
		 */
		[RCAR_DU_OUTPUT_LVDS0] = {
			.possible_crtcs = BIT(0),
			.possible_clones = 0,
			.encoder_type = DRM_MODE_ENCODER_LVDS,
		},
		[RCAR_DU_OUTPUT_DPAD0] = {
			.possible_crtcs = BIT(1),
			.possible_clones = 0,
			.encoder_type = DRM_MODE_ENCODER_NONE,
		},
	},
	.num_lvds = 1,
	.drgbs_bit = 1,
	.max_xres = 1920,
	.max_yres = 1080,
	.interlace = true,
	.lvds0_crtc = BIT(0),
	.lvds1_crtc = 0,
	.vspd_crtc = BIT(0) | BIT(1),
	.chip = RCAR_M2,
	.drgbs_use = true,
	.vscs_use = false,
};

#if defined(R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND) || \
	defined(R8A779X_ES2_DU_LVDS_CH_DATA_GAP_WORKAROUND)
static struct rcar_du_device_info rcar_du_r8a7793_info = {
#else
static const struct rcar_du_device_info rcar_du_r8a7793_info = {
#endif
	.features = RCAR_DU_FEATURE_CRTC_IRQ_CLOCK | RCAR_DU_FEATURE_DEFR8 |
		    RCAR_DU_FEATURE_VSP1_SOURCE,
	.num_crtcs = 2,
	.routes = {
		/* R8A7793 has one RGB output, one LVDS output and one
		 * (currently unsupported) TCON output.
		 */
		[RCAR_DU_OUTPUT_LVDS0] = {
			.possible_crtcs = BIT(0),
			.possible_clones = 0,
			.encoder_type = DRM_MODE_ENCODER_LVDS,
		},
		[RCAR_DU_OUTPUT_DPAD0] = {
			.possible_crtcs = BIT(1),
			.possible_clones = 0,
			.encoder_type = DRM_MODE_ENCODER_NONE,
		},
	},
	.num_lvds = 1,
	.drgbs_bit = 1,
	.max_xres = 1920,
	.max_yres = 1080,
	.interlace = true,
	.lvds0_crtc = BIT(0),
	.lvds1_crtc = 0,
	.vspd_crtc = BIT(0) | BIT(1),
	.chip = RCAR_M2N,
	.drgbs_use = true,
	.vscs_use = false,
};

#ifdef R8A7790_ES1_DU_LVDS_LANE_MISCONNECTION_WORKAROUND
static struct rcar_du_device_info rcar_du_r8a7794_info = {
#else
static const struct rcar_du_device_info rcar_du_r8a7794_info = {
#endif
	.features = RCAR_DU_FEATURE_CRTC_IRQ_CLOCK | RCAR_DU_FEATURE_DEFR8 |
		    RCAR_DU_FEATURE_VSP1_SOURCE,
	.num_crtcs = 2,
	.routes = {
		[RCAR_DU_OUTPUT_DPAD0] = {
			.possible_crtcs = BIT(0),
#ifdef CONFIG_DRM_RCAR_LVDS
			.possible_clones = BIT(1),
#else
			.possible_clones = 0,
#endif
			.encoder_type = RCAR_DU_ENCODER_HDMI,
		},
		[RCAR_DU_OUTPUT_LVDS0] = {
			.possible_crtcs = BIT(0),
#if defined(CONFIG_DRM_ADV7511) || defined(CONFIG_DRM_ADV7511_MODULE)
			.possible_clones = BIT(0),
#else
			.possible_clones = 0,
#endif
			.encoder_type = DRM_MODE_ENCODER_LVDS,
		},
		[RCAR_DU_OUTPUT_DPAD1] = {
			.possible_crtcs = BIT(1),
			.possible_clones = 0,
			.encoder_type = DRM_MODE_ENCODER_DAC,
		},
	},
	.num_lvds = 0,
	.drgbs_bit = 0,
	.max_xres = 1920,
	.max_yres = 1080,
	.interlace = true,
	.lvds0_crtc = 0,
	.lvds1_crtc = 0,
	.vspd_crtc = BIT(0),
	.chip = RCAR_E2,
	.drgbs_use = false,
	.vscs_use = false,
};

static const struct platform_device_id rcar_du_id_table[] = {
	{ "rcar-du-r8a7779", (kernel_ulong_t)&rcar_du_r8a7779_info },
	{ "rcar-du-r8a7790", (kernel_ulong_t)&rcar_du_r8a7790_info },
	{ "rcar-du-r8a7791", (kernel_ulong_t)&rcar_du_r8a7791_info },
	{ "rcar-du-r8a7793", (kernel_ulong_t)&rcar_du_r8a7793_info },
	{ "rcar-du-r8a7794", (kernel_ulong_t)&rcar_du_r8a7794_info },
	{ }
};

MODULE_DEVICE_TABLE(platform, rcar_du_id_table);

static struct platform_driver rcar_du_platform_driver = {
	.probe		= rcar_du_probe,
	.remove		= rcar_du_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "rcar-du",
		.pm	= &rcar_du_pm_ops,
	},
	.id_table	= rcar_du_id_table,
};

module_platform_driver(rcar_du_platform_driver);

MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_DESCRIPTION("Renesas R-Car Display Unit DRM Driver");
MODULE_LICENSE("GPL");
