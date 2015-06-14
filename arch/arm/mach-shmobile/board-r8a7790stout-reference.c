/*
 * R8A7790-stout board support - Reference DT implementation
 *
 * Copyright (C) 2015 Renesas Electronics Europe GmbH
 * Copyright (C) 2015 Renesas Electronics Corporation
 * Copyright (C) 2015 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/tmio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sh_mobile_sdhi.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_data/camera-rcar.h>
#include <linux/platform_data/rcar-du.h>
#include <linux/platform_data/usb-rcar-gen2-phy.h>
#include <linux/serial_sci.h>
#include <linux/sh_dma.h>
#include <linux/spi/flash.h>
#include <linux/spi/sh_msiof.h>
#include <linux/spi/spi.h>
#include <linux/usb/phy.h>
#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
#include <linux/platform_data/vsp1.h>
#endif
#include <media/soc_camera.h>
#include <asm/mach/arch.h>
#include <sound/rcar_snd.h>
#include <sound/simple_card.h>

#include "clock.h"
#include "common.h"
#include "dma-register.h"
#include "irqs.h"
#include "r8a7790.h"
#include "rcar-gen2.h"

/* DU */
static struct rcar_du_encoder_data r8a7790stout_du_encoders[] = {
	{
		.type = RCAR_DU_ENCODER_HDMI,
		.output = RCAR_DU_OUTPUT_DPAD0,
	},
};

static struct rcar_du_crtc_data r8a7790stout_du_crtcs[] = {
	{
		/* LVDS0 */
		.exclk = 0, /* 148500000 */
		.init_conn_type = DRM_MODE_CONNECTOR_LVDS,
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
		.vsp = CONFIG_DRM_RCAR_DU0_USE_VSPDU_CH,
#endif
	}, {
		/* RGB */
		.exclk =  0, /* 148500000 */
		.init_conn_type = DRM_MODE_CONNECTOR_HDMIA,
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
		.vsp = CONFIG_DRM_RCAR_DU1_USE_VSPDU_CH,
#endif
	}, {
		/* LVDS1 */
		.exclk = 0, /* 148500000 */
		.init_conn_type = DRM_MODE_CONNECTOR_LVDS,
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
		.vsp = CONFIG_DRM_RCAR_DU2_USE_VSPDU_CH,
#endif
	},
};

static struct rcar_du_platform_data r8a7790stout_du_pdata = {
	.encoders = r8a7790stout_du_encoders,
	.num_encoders = ARRAY_SIZE(r8a7790stout_du_encoders),
	.crtcs = r8a7790stout_du_crtcs,
	.num_crtcs = ARRAY_SIZE(r8a7790stout_du_crtcs),
#ifdef CONFIG_DRM_FBDEV_CRTC
	.fbdev_crtc = 1,
#endif
	.i2c_ch = 2,
};

static const struct resource du_resources[] __initconst = {
	DEFINE_RES_MEM(0xfeb00000, 0x70000),
	DEFINE_RES_MEM_NAMED(0xfeb90000, 0x1c, "lvds.0"),
	DEFINE_RES_MEM_NAMED(0xfeb94000, 0x1c, "lvds.1"),
	DEFINE_RES_IRQ(gic_spi(256)),
	DEFINE_RES_IRQ(gic_spi(268)),
	DEFINE_RES_IRQ(gic_spi(269)),
};

static void __init r8a7790stout_add_du_device(void)
{
	struct platform_device_info info = {
		.name = "rcar-du-r8a7790",
		.id = -1,
		.res = du_resources,
		.num_res = ARRAY_SIZE(du_resources),
		.data = &r8a7790stout_du_pdata,
		.size_data = sizeof(r8a7790stout_du_pdata),
		.dma_mask = DMA_BIT_MASK(32),
	};

	platform_device_register_full(&info);
}

/*
 * This is a really crude hack to provide clkdev support to platform
 * devices until they get moved to DT.
 */
static const struct clk_name clk_names[] __initconst = {
	{ "cmt0", NULL, "sh_cmt.0" },
	{ "du0", "du.0", "rcar-du-r8a7790" },
	{ "du1", "du.1", "rcar-du-r8a7790" },
	{ "du2", "du.2", "rcar-du-r8a7790" },
	{ "lvds0", "lvds.0", "rcar-du-r8a7790" },
	{ "lvds1", "lvds.1", "rcar-du-r8a7790" },
	{ "hsusb", NULL, "usb_phy_rcar_gen2" },
	{ "vin0", NULL, "r8a7790-vin.0" },
	{ "vspr", NULL, NULL },
	{ "vsps", NULL, NULL },
#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
	{ "vsp1-du0", NULL, "vsp1.2" },
	{ "vsp1-du1", NULL, "vsp1.3" },
#else
	{ "vsp1-du0", NULL, NULL },
	{ "vsp1-du1", NULL, NULL },
#endif
	{ "vcp1", NULL, NULL },
	{ "vcp0", NULL, NULL },
	{ "vpc1", NULL, NULL },
	{ "vpc0", NULL, NULL },
	{ "tddmac", NULL, NULL },
	{ "fdp2", NULL, NULL },
	{ "fdp1", NULL, NULL },
	{ "fdp0", NULL, NULL },
	{ "pvrsrvkm", NULL, "pvrsrvkm" },
	{ "ehci", NULL, "pci-rcar-gen2.0" },
};

/*
 * This is a really crude hack to work around core platform clock issues
 */
static const struct clk_name clk_enables[] __initconst = {
	{ "ehci", NULL, "pci-rcar-gen2.1" },
	{ "dmal", NULL, "sh-dma-engine.0" },
	{ "dmah", NULL, "sh-dma-engine.1" },
	{ "sys-dmac1", NULL, "sh-dma-engine.2" },
	{ "sys-dmac0", NULL, "sh-dma-engine.3" },
	{ "ssp_dev", NULL, "ssp_dev" },
};

#define DMAE_CHANNEL(a, b)			\
{						\
	.offset		= (a) - 0x20,		\
	.dmars		= (a) - 0x20 + 0x40,	\
	.chclr_bit	= (b),			\
	.chclr_offset	= 0x80 - 0x20,		\
}

/* Sys-DMAC */
#define SYS_DMAC_SLAVE(_id, _bit, _addr, toffset, roffset, t, r)	\
{								\
	.slave_id	= SYS_DMAC_SLAVE_## _id ##_TX,		\
	.addr		= _addr + toffset,			\
	.chcr		= CHCR_TX(XMIT_SZ_## _bit ##BIT),	\
	.mid_rid	= t,					\
}, {								\
	.slave_id	= SYS_DMAC_SLAVE_## _id ##_RX,		\
	.addr		= _addr + roffset,			\
	.chcr		= CHCR_RX(XMIT_SZ_## _bit ##BIT),	\
	.mid_rid	= r,					\
}

#define SYS_DMAC_SLAVE_TX(_id, _bit, _addr, toffset, roffset, t, r)	\
{								\
	.slave_id	= SYS_DMAC_SLAVE_## _id ##_TX,		\
	.addr		= _addr + toffset,			\
	.chcr		= CHCR_TX(XMIT_SZ_## _bit ##BIT),	\
	.mid_rid	= t,					\
}

static const struct sh_dmae_slave_config r8a7790_sys_dmac_slaves[] = {
	SYS_DMAC_SLAVE(SDHI0, 256, 0xee100000, 0x60, 0x2060, 0xcd, 0xce),
	SYS_DMAC_SLAVE(SCIFA0, 8, 0xe6c40000, 0x20, 0x24, 0x21, 0x22),
};

static const struct sh_dmae_channel r8a7790_sys_dmac_channels[] = {
	DMAE_CHANNEL(0x8000, 0),
	DMAE_CHANNEL(0x8080, 1),
	DMAE_CHANNEL(0x8100, 2),
	DMAE_CHANNEL(0x8180, 3),
	DMAE_CHANNEL(0x8200, 4),
	DMAE_CHANNEL(0x8280, 5),
	DMAE_CHANNEL(0x8300, 6),
	DMAE_CHANNEL(0x8380, 7),
	DMAE_CHANNEL(0x8400, 8),
	DMAE_CHANNEL(0x8480, 9),
	DMAE_CHANNEL(0x8500, 10),
	DMAE_CHANNEL(0x8580, 11),
	DMAE_CHANNEL(0x8600, 12),
	DMAE_CHANNEL(0x8680, 13),
	DMAE_CHANNEL(0x8700, 14),
};

static struct sh_dmae_pdata r8a7790_sys_dmac_platform_data = {
	.slave		= r8a7790_sys_dmac_slaves,
	.slave_num	= ARRAY_SIZE(r8a7790_sys_dmac_slaves),
	.channel	= r8a7790_sys_dmac_channels,
	.channel_num	= ARRAY_SIZE(r8a7790_sys_dmac_channels),
	.ts_low_shift	= TS_LOW_SHIFT,
	.ts_low_mask	= TS_LOW_BIT << TS_LOW_SHIFT,
	.ts_high_shift	= TS_HI_SHIFT,
	.ts_high_mask	= TS_HI_BIT << TS_HI_SHIFT,
	.ts_shift	= dma_ts_shift,
	.ts_shift_num	= ARRAY_SIZE(dma_ts_shift),
	.dmaor_init	= DMAOR_DME,
	.chclr_present	= 1,
	.chclr_bitwise	= 1,
	.fourty_bits_addr = 1,
};

static struct resource r8a7790_sys_dmac_resources[] = {
	/* Channel registers and DMAOR for low */
	DEFINE_RES_MEM(0xe6700020, 0x8763 - 0x20),
	DEFINE_RES_IRQ(gic_spi(197)),
	DEFINE_RES_NAMED(gic_spi(200), 15, NULL, IORESOURCE_IRQ),

	/* Channel registers and DMAOR for high */
	DEFINE_RES_MEM(0xe6720020, 0x8763 - 0x20),
	DEFINE_RES_IRQ(gic_spi(220)),
	DEFINE_RES_NAMED(gic_spi(216), 4, NULL, IORESOURCE_IRQ),
	DEFINE_RES_NAMED(gic_spi(308), 11, NULL, IORESOURCE_IRQ),
};

#define r8a7790_register_sys_dmac(id)				\
	platform_device_register_resndata(			\
		&platform_bus, "sh-dma-engine", 2 + id,		\
		&r8a7790_sys_dmac_resources[id * 3],	id * 1 + 3,	\
		&r8a7790_sys_dmac_platform_data,		\
		sizeof(r8a7790_sys_dmac_platform_data))

static void __init r8a7790stout_add_dmac_prototype(void)
{
	r8a7790_register_sys_dmac(0);
	r8a7790_register_sys_dmac(1);
}

static struct sh_mobile_sdhi_info sdhi0_info = {
	.dma_slave_tx   = SYS_DMAC_SLAVE_SDHI0_TX,
	.dma_slave_rx   = SYS_DMAC_SLAVE_SDHI0_RX,
	.dma_rx_offset  = 0x2000,

	.tmio_caps	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ |
			  MMC_CAP_POWER_OFF_CARD,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT |
			  TMIO_MMC_WRPROTECT_DISABLE,
	.tmio_ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
};

/* SCIF */
#define SCIF_PD(scif_type, index, scif_index)				\
static struct plat_sci_port scif##index##_platform_data = {	\
	.type		= PORT_##scif_type,			\
	.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,	\
	.scscr		= SCSCR_RE | SCSCR_TE,			\
	.dma_slave_tx	= SYS_DMAC_SLAVE_##scif_type##scif_index##_TX,	\
	.dma_slave_rx	= SYS_DMAC_SLAVE_##scif_type##scif_index##_RX,	\
}

#define PDATA_SCIFA(index, baseaddr, irq, i) SCIF_PD(SCIFA, index, i)

PDATA_SCIFA(0, 0xe6c40000, gic_spi(144), 0); /* SCIFA0 */

#define SCIF_AD(scif_type, index, baseaddr)		\
	OF_DEV_AUXDATA("renesas," scif_type "-r8a7790", baseaddr, \
			"sh-sci." # index, &scif##index##_platform_data)

#define AUXDATA_SCIFA(index, baseaddr, irq) SCIF_AD("scifa", index, baseaddr)

/* Internal PCI0 */
static const struct resource pci0_resources[] __initconst = {
	DEFINE_RES_MEM(0xee090000, 0x10000),	/* CFG */
	DEFINE_RES_MEM(0xee080000, 0x10000),	/* MEM */
	DEFINE_RES_IRQ(gic_spi(108)),
};

static const struct platform_device_info pci0_info __initconst = {
	.parent		= &platform_bus,
	.name		= "pci-rcar-gen2",
	.id		= 0,
	.res		= pci0_resources,
	.num_res	= ARRAY_SIZE(pci0_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

static void __init r8a7790stout_add_usb0_device(void)
{
	usb_bind_phy("pci-rcar-gen2.0", 0, "usb_phy_rcar_gen2");
	platform_device_register_full(&pci0_info);
}

/* USBHS PHY */
static const struct rcar_gen2_phy_platform_data usbhs_phy_pdata __initconst = {
	.chan0_pci = 1,	/* Channel 0 is PCI USB */
	.gpio_vbus = -1,
	.wakeup = false,
};

static const struct resource usbhs_phy_resources[] __initconst = {
	DEFINE_RES_MEM(0xe6590100, 0x100),
};

/* VIN */
static const struct resource vin_resources[] __initconst = {
	/* VIN0 */
	DEFINE_RES_MEM(0xe6ef0000, 0x1000),
	DEFINE_RES_IRQ(gic_spi(188)),
};

static void __init r8a7790stout_add_vin_device(unsigned idx,
					struct rcar_vin_platform_data *pdata)
{
	struct platform_device_info vin_info = {
		.parent		= &platform_bus,
		.name		= "r8a7790-vin",
		.id		= idx,
		.res		= &vin_resources[idx * 2],
		.num_res	= 2,
		.dma_mask	= DMA_BIT_MASK(32),
		.data		= pdata,
		.size_data	= sizeof(*pdata),
	};

	BUG_ON(idx > 3);

	platform_device_register_full(&vin_info);
}

static int r8a7790stout_camera_power(struct device *dev, int on)
{
	return 0;
}

#define R8A7790stout_CAMERA(idx, name, bus, addr, pdata, flag)		\
static struct i2c_board_info i2c_cam##idx##_device = {			\
	I2C_BOARD_INFO(name, addr),					\
};									\
									\
static struct rcar_vin_platform_data vin##idx##_pdata = {		\
	.flags = flag,							\
};									\
									\
static struct soc_camera_link cam##idx##_link = {			\
	.bus_id = idx,							\
	.board_info = &i2c_cam##idx##_device,				\
	.i2c_adapter_id = bus,						\
	.module_name = name,						\
	.priv = pdata,							\
	.power = r8a7790stout_camera_power,				\
}

R8A7790stout_CAMERA(0, "ov10635", 2, 0x30, NULL, RCAR_VIN_BT656);

static void __init r8a7790stout_add_camera0_device(void)
{
	platform_device_register_data(&platform_bus, "soc-camera-pdrv", 0,
				      &cam0_link, sizeof(cam0_link));
	r8a7790stout_add_vin_device(0, &vin0_pdata);
}

/* VSP1 */
#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
static const struct vsp1_platform_data r8a7790stout_vspr_pdata __initconst = {
	.features = VSP1_HAS_LUT | VSP1_HAS_SRU,
	.rpf_count = 5,
	.uds_count = 1,
	.wpf_count = 4,
};

static const struct vsp1_platform_data r8a7790stout_vsps_pdata __initconst = {
	.features = VSP1_HAS_SRU,
	.rpf_count = 5,
	.uds_count = 3,
	.wpf_count = 4,
};

static const struct vsp1_platform_data r8a7790stout_vspd0_pdata __initconst = {
	.features = VSP1_HAS_LIF | VSP1_HAS_LUT,
	.rpf_count = 4,
	.uds_count = 1,
	.wpf_count = 4,
};

static const struct vsp1_platform_data r8a7790stout_vspd1_pdata __initconst = {
	.features = VSP1_HAS_LIF | VSP1_HAS_LUT,
	.rpf_count = 4,
	.uds_count = 1,
	.wpf_count = 4,
};

static const struct vsp1_platform_data * const r8a7790stout_vsp1_pdata[4] __initconst = {
	&r8a7790stout_vspr_pdata,
	&r8a7790stout_vsps_pdata,
	&r8a7790stout_vspd0_pdata,
	&r8a7790stout_vspd1_pdata,
};

static const struct resource vspr_resources[] __initconst = {
	DEFINE_RES_MEM(0xfe920000, 0x8000),
	DEFINE_RES_IRQ(gic_spi(266)),
};

static const struct resource vsps_resources[] __initconst = {
	DEFINE_RES_MEM(0xfe928000, 0x8000),
	DEFINE_RES_IRQ(gic_spi(267)),
};

static const struct resource vspd0_resources[] __initconst = {
	DEFINE_RES_MEM(0xfe930000, 0x8000),
	DEFINE_RES_IRQ(gic_spi(246)),
};

static const struct resource vspd1_resources[] __initconst = {
	DEFINE_RES_MEM(0xfe938000, 0x8000),
	DEFINE_RES_IRQ(gic_spi(247)),
};

static const struct resource * const vsp1_resources[4] __initconst = {
	vspr_resources,
	vsps_resources,
	vspd0_resources,
	vspd1_resources,
};

static void __init r8a7790stout_add_vsp1_devices(void)
{
	struct platform_device_info info = {
		.name = "vsp1",
		.size_data = sizeof(*r8a7790stout_vsp1_pdata[0]),
		.num_res = 2,
		.dma_mask = DMA_BIT_MASK(32),
	};
	unsigned int i;

	for (i = 2; i < ARRAY_SIZE(vsp1_resources); ++i) {
		info.id = i;
		info.data = r8a7790stout_vsp1_pdata[i];
		info.res = vsp1_resources[i];

		platform_device_register_full(&info);
	}
}
#endif

static struct of_dev_auxdata r8a7790stout_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("renesas,sdhi-r8a7790", 0xee100000, "sdhi0",
			&sdhi0_info),
	AUXDATA_SCIFA(0, 0xe6c40000, gic_spi(144)), /* SCIFA0 */
	{},
};

static void __init r8a7790stout_add_standard_devices(void)
{
	shmobile_clk_workaround(clk_names, ARRAY_SIZE(clk_names), false);
	shmobile_clk_workaround(clk_enables, ARRAY_SIZE(clk_enables), true);
	r8a7790_add_dt_devices();

	r8a7790stout_add_dmac_prototype();
	of_platform_populate(NULL, of_default_bus_match_table,
			     r8a7790stout_auxdata_lookup, NULL);

	r8a7790stout_add_du_device();
	platform_device_register_resndata(&platform_bus, "usb_phy_rcar_gen2",
					  -1, usbhs_phy_resources,
					  ARRAY_SIZE(usbhs_phy_resources),
					  &usbhs_phy_pdata,
					  sizeof(usbhs_phy_pdata));
	r8a7790stout_add_usb0_device();

	r8a7790stout_add_camera0_device();

#if IS_ENABLED(CONFIG_VIDEO_RENESAS_VSP1) && \
!defined(CONFIG_DRM_RCAR_DU_CONNECT_VSP)
	r8a7790stout_add_vsp1_devices();
#endif
}

static const char *r8a7790stout_boards_compat_dt[] __initdata = {
	"renesas,r8a7790stout",
	"renesas,r8a7790stout-reference",
	NULL,
};

DT_MACHINE_START(R8A7790stout_DT, "r8a7790stout")
	.smp		= smp_ops(r8a7790_smp_ops),
	.init_early	= shmobile_init_delay,
	.init_time	= rcar_gen2_timer_init,
	.init_machine	= r8a7790stout_add_standard_devices,
	.init_late	= shmobile_init_late,
	.reserve	= rcar_gen2_reserve,
	.dt_compat	= r8a7790stout_boards_compat_dt,
MACHINE_END
