/*
 * Lager board support - Reference DT implementation
 *
 * Copyright (C) 2014  Renesas Electronics Corporation
 * Copyright (C) 2013  Renesas Solutions Corp.
 * Copyright (C) 2013  Simon Horman
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
#include <linux/mfd/tmio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sh_mmcif.h>
#include <linux/mmc/sh_mobile_sdhi.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_data/camera-rcar.h>
#include <linux/platform_data/rcar-du.h>
#include <linux/platform_data/usb-rcar-gen2-phy.h>
#include <linux/serial_sci.h>
#include <linux/sh_dma.h>
#include <linux/spi/flash.h>
#include <linux/spi/sh_msiof.h>
#include <linux/spi/spi.h>
#include <linux/usb/phy.h>
#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
#include <linux/usb/renesas_usbhs.h>
#endif
#if defined(CONFIG_VIDEO_RENESAS_VSP1)
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
static struct rcar_du_encoder_data lager_du_encoders[] = {
#if defined(CONFIG_DRM_ADV7511) || defined(CONFIG_DRM_ADV7511_MODULE)
	{
		.type = RCAR_DU_ENCODER_HDMI,
		.output = RCAR_DU_OUTPUT_LVDS0,
	}, {
		.type = RCAR_DU_ENCODER_VGA,
		.output = RCAR_DU_OUTPUT_DPAD0,
	}, {
		.type = RCAR_DU_ENCODER_NONE,
		.output = RCAR_DU_OUTPUT_LVDS1,
		.connector.lvds.panel = {
			.width_mm = 210,
			.height_mm = 158,
			.mode = {
				.clock = 65000,
				.hdisplay = 1024,
				.hsync_start = 1048,
				.hsync_end = 1184,
				.htotal = 1344,
				.vdisplay = 768,
				.vsync_start = 771,
				.vsync_end = 777,
				.vtotal = 806,
				.flags = 0,
			},
		},
	},
#else
	{
		.type = RCAR_DU_ENCODER_VGA,
		.output = RCAR_DU_OUTPUT_DPAD0,
	}, {
		.type = RCAR_DU_ENCODER_NONE,
		.output = RCAR_DU_OUTPUT_LVDS1,
		.connector.lvds.panel = {
			.width_mm = 210,
			.height_mm = 158,
			.mode = {
				.clock = 65000,
				.hdisplay = 1024,
				.hsync_start = 1048,
				.hsync_end = 1184,
				.htotal = 1344,
				.vdisplay = 768,
				.vsync_start = 771,
				.vsync_end = 777,
				.vtotal = 806,
				.flags = 0,
			},
		},
	},
#endif
};

static struct rcar_du_crtc_data lager_du_crtcs[] = {
#if defined(CONFIG_DRM_ADV7511) || defined(CONFIG_DRM_ADV7511_MODULE)
	{
		.exclk = 148500000,
		.init_conn_type = DRM_MODE_CONNECTOR_HDMIA,
	},
	{
		.exclk = 148500000,
		.init_conn_type = DRM_MODE_CONNECTOR_LVDS,
	},
	{
		.exclk = 0,
		.init_conn_type = DRM_MODE_CONNECTOR_VGA,
	},
#else
	{
		.exclk = 148500000,
		.init_conn_type = DRM_MODE_CONNECTOR_VGA,
	},
	{
		.exclk = 148500000,
		.init_conn_type = DRM_MODE_CONNECTOR_LVDS,
	},
#endif
};

static struct rcar_du_platform_data lager_du_pdata = {
	.encoders = lager_du_encoders,
	.num_encoders = ARRAY_SIZE(lager_du_encoders),
	.crtcs = lager_du_crtcs,
	.num_crtcs = ARRAY_SIZE(lager_du_crtcs),
#ifdef CONFIG_DRM_FBDEV_CRTC
	.fbdev_crtc = 0,
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

static void __init lager_add_du_device(void)
{
	struct platform_device_info info = {
		.name = "rcar-du-r8a7790",
		.id = -1,
		.res = du_resources,
		.num_res = ARRAY_SIZE(du_resources),
		.data = &lager_du_pdata,
		.size_data = sizeof(lager_du_pdata),
		.dma_mask = DMA_BIT_MASK(32),
	};

	platform_device_register_full(&info);
}

/* Sound */
static struct rsnd_ssi_platform_info rsnd_ssi[] = {
	RSND_SSI(AUDIOPP_DMAC_SLAVE_CMD0_TO_SSI0, gic_spi(370), 0),
	RSND_SSI(AUDIOPP_DMAC_SLAVE_SSI1_TO_SCU1, gic_spi(371), RSND_SSI_CLK_PIN_SHARE),
};

static struct rsnd_src_platform_info rsnd_src[2] = {
	RSND_SRC(0, AUDIO_DMAC_SLAVE_SCU0_TX),
	RSND_SRC(0, AUDIO_DMAC_SLAVE_CMD1_TO_MEM),
};

static struct rsnd_dvc_platform_info rsnd_dvc[2] = {
};

static struct rsnd_dai_platform_info rsnd_dai = {
	.playback = { .ssi = &rsnd_ssi[0], .src = &rsnd_src[0], .dvc = &rsnd_dvc[0], },
	.capture  = { .ssi = &rsnd_ssi[1], .src = &rsnd_src[1], .dvc = &rsnd_dvc[1], },
};

static struct rcar_snd_info rsnd_info = {
	.flags		= RSND_GEN2,
	.ssi_info	= rsnd_ssi,
	.ssi_info_nr	= ARRAY_SIZE(rsnd_ssi),
	.src_info	= rsnd_src,
	.src_info_nr	= ARRAY_SIZE(rsnd_src),
	.dvc_info	= rsnd_dvc,
	.dvc_info_nr	= ARRAY_SIZE(rsnd_dvc),
	.dai_info	= &rsnd_dai,
	.dai_info_nr	= 1,
};

static struct asoc_simple_card_info rsnd_card_info = {
	.name		= "SSI01-AK4643",
	.codec		= "ak4642-codec.2-0012",
	.platform	= "rcar_sound",
	.daifmt		= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS,
	.cpu_dai = {
		.name	= "rcar_sound",
	},
	.codec_dai = {
		.name	= "ak4642-hifi",
		.sysclk	= 11289600,
	},
};

static void __init lager_add_rsnd_device(void)
{
	struct resource rsnd_resources[] = {
		[RSND_GEN2_SCU]  = DEFINE_RES_MEM(0xec500000, 0x1000),
		[RSND_GEN2_ADG]  = DEFINE_RES_MEM(0xec5a0000, 0x100),
		[RSND_GEN2_SSIU] = DEFINE_RES_MEM(0xec540000, 0x1000),
		[RSND_GEN2_SSI]  = DEFINE_RES_MEM(0xec541000, 0x1280),
	};

	struct platform_device_info cardinfo = {
		.parent         = &platform_bus,
		.name           = "asoc-simple-card",
		.id             = -1,
		.data           = &rsnd_card_info,
		.size_data      = sizeof(struct asoc_simple_card_info),
		.dma_mask       = DMA_BIT_MASK(32),
	};

	platform_device_register_resndata(
		&platform_bus, "rcar_sound", -1,
		rsnd_resources, ARRAY_SIZE(rsnd_resources),
		&rsnd_info, sizeof(rsnd_info));

	platform_device_register_full(&cardinfo);
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
	{ "ssi0", "ssi.0", "rcar_sound" },
	{ "ssi1", "ssi.1", "rcar_sound" },
	{ "src0", "src.0", "rcar_sound" },
	{ "src1", "src.1", "rcar_sound" },
	{ "dvc0", "dvc.0", "rcar_sound" },
	{ "dvc1", "dvc.1", "rcar_sound" },
	{ "vin0", NULL, "r8a7790-vin.0" },
	{ "vin1", NULL, "r8a7790-vin.1" },
	{ "vspr", NULL, NULL },
	{ "vsps", NULL, NULL },
#if defined(CONFIG_VIDEO_RENESAS_VSP1)
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
};

/*
 * This is a really crude hack to work around core platform clock issues
 */
static const struct clk_name clk_enables[] __initconst = {
#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
	{ "hsusb", NULL, "renesas_usbhs" },
#else
	{ "ehci", NULL, "pci-rcar-gen2.0" },
#endif
	{ "ehci", NULL, "pci-rcar-gen2.1" },
	{ "ehci", NULL, "pci-rcar-gen2.2" },
	{ "ssi", NULL, "rcar_sound" },
	{ "scu", NULL, "rcar_sound" },
	{ "dmal", NULL, "sh-dma-engine.0" },
	{ "dmah", NULL, "sh-dma-engine.1" },
#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
	{ "usbdmac0", NULL, "sh-dma-engine.4" },
#endif
	{ "pvrsrvkm", NULL, "pvrsrvkm" },
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
	SYS_DMAC_SLAVE(MMCIF0, 32, 0xee200000, 0x34, 0x34, 0xd1, 0xd2),
	SYS_DMAC_SLAVE(MMCIF1, 32, 0xee220000, 0x34, 0x34, 0xe1, 0xe2),
	SYS_DMAC_SLAVE(SDHI0, 256, 0xee100000, 0x60, 0x2060, 0xcd, 0xce),
	SYS_DMAC_SLAVE(SDHI2, 256, 0xee140000, 0x30, 0x2030, 0xc1, 0xc2),
	SYS_DMAC_SLAVE(SCIF0, 8, 0xe6e60000, 0xc, 0x14, 0x29, 0x2a),
	SYS_DMAC_SLAVE(SCIF1, 8, 0xe6e68000, 0xc, 0x14, 0x2d, 0x2e),
	SYS_DMAC_SLAVE(SCIFA0, 8, 0xe6c40000, 0x20, 0x24, 0x21, 0x22),
	SYS_DMAC_SLAVE(SCIFA1, 8, 0xe6c50000, 0x20, 0x24, 0x25, 0x26),
	SYS_DMAC_SLAVE(SCIFA2, 8, 0xe6c60000, 0x20, 0x24, 0x27, 0x28),
	SYS_DMAC_SLAVE(SCIFB0, 8, 0xe6c20000, 0x40, 0x60, 0x3d, 0x3e),
	SYS_DMAC_SLAVE(SCIFB1, 8, 0xe6c30000, 0x40, 0x60, 0x19, 0x1a),
	SYS_DMAC_SLAVE(SCIFB2, 8, 0xe6ce0000, 0x40, 0x60, 0x1d, 0x1e),
	SYS_DMAC_SLAVE(HSCIF0, 8, 0xe62c0000, 0xc, 0x14, 0x39, 0x3a),
	SYS_DMAC_SLAVE(HSCIF1, 8, 0xe62c8000, 0xc, 0x14, 0x4d, 0x4e),
	SYS_DMAC_SLAVE(MSIOF0, 32, 0xe7e20000, 0x50, 0x60, 0x51, 0x52),
	SYS_DMAC_SLAVE(MSIOF1, 32, 0xe7e10000, 0x50, 0x60, 0x55, 0x56),
	SYS_DMAC_SLAVE(MSIOF2, 32, 0xe7e00000, 0x50, 0x60, 0x41, 0x42),
	SYS_DMAC_SLAVE(MSIOF3, 32, 0xec900000, 0x50, 0x60, 0x45, 0x46),
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

static void __init lager_add_dmac_prototype(void)
{
	r8a7790_register_sys_dmac(0);
	r8a7790_register_sys_dmac(1);
}

static struct sh_mmcif_plat_data mmcif1_pdata = {
	.caps		= MMC_CAP_4_BIT_DATA |
			  MMC_CAP_8_BIT_DATA |
			  MMC_CAP_NONREMOVABLE,
	.ccs_unsupported = true,
	.clk_ctrl2_present = true,
	.slave_id_tx	= SYS_DMAC_SLAVE_MMCIF1_TX,
	.slave_id_rx	= SYS_DMAC_SLAVE_MMCIF1_RX,
};

static struct sh_mobile_sdhi_info sdhi0_info __initdata = {
	.dma_slave_tx   = SYS_DMAC_SLAVE_SDHI0_TX,
	.dma_slave_rx   = SYS_DMAC_SLAVE_SDHI0_RX,
	.dma_rx_offset  = 0x2000,

	.tmio_caps	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ |
			  MMC_CAP_POWER_OFF_CARD,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT |
			  TMIO_MMC_WRPROTECT_DISABLE,
};

static struct sh_mobile_sdhi_info sdhi2_info __initdata = {
	.dma_slave_tx   = SYS_DMAC_SLAVE_SDHI2_TX,
	.dma_slave_rx   = SYS_DMAC_SLAVE_SDHI2_RX,
	.dma_rx_offset  = 0x2000,

	.tmio_caps	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ |
			  MMC_CAP_POWER_OFF_CARD,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT |
			  TMIO_MMC_WRPROTECT_DISABLE,
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

#define PDATA_SCIF(index, baseaddr, irq, i) SCIF_PD(SCIF, index, i)
#define PDATA_SCIFA(index, baseaddr, irq, i) SCIF_PD(SCIFA, index, i)
#define PDATA_SCIFB(index, baseaddr, irq, i) SCIF_PD(SCIFB, index, i)
#define PDATA_HSCIF(index, baseaddr, irq, i) SCIF_PD(HSCIF, index, i)

PDATA_SCIFA(0, 0xe6c40000, gic_spi(144), 0); /* SCIFA0 */
PDATA_SCIFA(1, 0xe6c50000, gic_spi(145), 1); /* SCIFA1 */
PDATA_SCIFB(2, 0xe6c20000, gic_spi(148), 0); /* SCIFB0 */
PDATA_SCIFB(3, 0xe6c30000, gic_spi(149), 1); /* SCIFB1 */
PDATA_SCIFB(4, 0xe6ce0000, gic_spi(150), 2); /* SCIFB2 */
PDATA_SCIFA(5, 0xe6c60000, gic_spi(151), 2); /* SCIFA2 */
PDATA_SCIF(6,  0xe6e60000, gic_spi(152), 0); /* SCIF0 */
PDATA_SCIF(7,  0xe6e68000, gic_spi(153), 1); /* SCIF1 */
PDATA_HSCIF(8, 0xe62c0000, gic_spi(154), 0); /* HSCIF0 */
PDATA_HSCIF(9, 0xe62c8000, gic_spi(155), 1); /* HSCIF1 */

#define SCIF_AD(scif_type, index, baseaddr)		\
	OF_DEV_AUXDATA("renesas," scif_type "-r8a7790", baseaddr, \
			"sh-sci." # index, &scif##index##_platform_data)

#define AUXDATA_SCIF(index, baseaddr, irq) SCIF_AD("scif", index, baseaddr)
#define AUXDATA_SCIFA(index, baseaddr, irq) SCIF_AD("scifa", index, baseaddr)
#define AUXDATA_SCIFB(index, baseaddr, irq) SCIF_AD("scifb", index, baseaddr)
#define AUXDATA_HSCIF(index, baseaddr, irq) SCIF_AD("hscif", index, baseaddr)

#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
/* USB-DMAC */
static const struct sh_dmae_channel usb_dmac_channels[] = {
	{
		.offset = 0,
	}, {
		.offset = 0x20,
	},
};

static const struct sh_dmae_slave_config usb_dmac_slaves[] = {
	{
		.slave_id	= USB_DMAC_SLAVE_USBHS_TX,
		.chcr		= USBTS_INDEX2VAL(USBTS_XMIT_SZ_32BYTE),
	}, {
		.slave_id	= USB_DMAC_SLAVE_USBHS_RX,
		.chcr		= USBTS_INDEX2VAL(USBTS_XMIT_SZ_32BYTE),
	},
};

static struct sh_dmae_pdata usb_dmac_platform_data = {
	.slave		= usb_dmac_slaves,
	.slave_num	= ARRAY_SIZE(usb_dmac_slaves),
	.channel	= usb_dmac_channels,
	.channel_num	= ARRAY_SIZE(usb_dmac_channels),
	.ts_low_shift	= USBTS_LOW_SHIFT,
	.ts_low_mask	= USBTS_LOW_BIT << USBTS_LOW_SHIFT,
	.ts_high_shift	= USBTS_HI_SHIFT,
	.ts_high_mask	= USBTS_HI_BIT << USBTS_HI_SHIFT,
	.ts_shift	= dma_usbts_shift,
	.ts_shift_num	= ARRAY_SIZE(dma_usbts_shift),
	.dmaor_init	= DMAOR_DME,
	.chcr_offset	= 0x14,
	.chcr_ie_bit	= 1 << 5,
	.dmaor_is_32bit	= 1,
	.needs_tend_set	= 1,
	.no_dmars	= 1,
	.slave_only	= 1,
};

static struct resource usb_dmac_resources[] = {
	DEFINE_RES_MEM(0xe65a0020, 0x44), /* Channel registers and DMAOR */
	DEFINE_RES_MEM(0xe65a0000, 0x14), /* VCR/SWR/DMICR */
	DEFINE_RES_IRQ(gic_spi(109)),
};

static void __init lager_add_usb_dmac_prototype(void)
{
	platform_device_register_resndata(&platform_bus, "sh-dma-engine",
					  4,
					  usb_dmac_resources,
					  ARRAY_SIZE(usb_dmac_resources),
					  &usb_dmac_platform_data,
					  sizeof(usb_dmac_platform_data));
}

/* USBHS */
static const struct resource usbhs_resources[] __initconst = {
	DEFINE_RES_MEM(0xe6590000, 0x100),
	DEFINE_RES_IRQ(gic_spi(107)),
};

struct usbhs_private {
	struct renesas_usbhs_platform_info info;
	struct usb_phy *phy;
	int pwen_gpio;
};

#define usbhs_get_priv(pdev) \
	container_of(renesas_usbhs_get_info(pdev), struct usbhs_private, info)

static int usbhs_power_ctrl(struct platform_device *pdev,
				void __iomem *base, int enable)
{
	struct usbhs_private *priv = usbhs_get_priv(pdev);

	if (!priv->phy)
		return -ENODEV;

	if (enable) {
		int retval = usb_phy_init(priv->phy);

		if (!retval)
			retval = usb_phy_set_suspend(priv->phy, 0);
		return retval;
	}

	usb_phy_set_suspend(priv->phy, 1);
	usb_phy_shutdown(priv->phy);
	return 0;
}

static int usbhs_hardware_init(struct platform_device *pdev)
{
	struct usbhs_private *priv = usbhs_get_priv(pdev);
	struct usb_phy *phy;
	int ret;
	struct device_node *np;

	np = of_find_node_by_path("/gpio@e6055000");
	if (np) {
		priv->pwen_gpio = of_get_gpio(np, 18);
		of_node_put(np);
	} else {
		pr_warn("Error: Unable to get PWEN GPIO line\n");
		ret = -ENOTSUPP;
		goto error2;
	}

	/* USB0 Function - use PWEN as GPIO input to detect DIP Switch SW5
	 * setting to avoid VBUS short circuit due to wrong cable.
	 * PWEN should be pulled up high if USB Function is selected by SW5
	 */
	gpio_request_one(priv->pwen_gpio, GPIOF_IN, NULL); /* USB0_PWEN */
	if (!gpio_get_value(priv->pwen_gpio)) {
		pr_warn("Error: USB Function not selected - check SW5 + SW6\n");
		ret = -ENOTSUPP;
		goto error;
	}

	phy = usb_get_phy_dev(&pdev->dev, 0);
	if (IS_ERR(phy)) {
		ret = PTR_ERR(phy);
		goto error;
	}

	priv->phy = phy;
	return 0;
 error:
	gpio_free(priv->pwen_gpio);
 error2:
	return ret;
}

static int usbhs_hardware_exit(struct platform_device *pdev)
{
	struct usbhs_private *priv = usbhs_get_priv(pdev);

	if (!priv->phy)
		return 0;

	usb_put_phy(priv->phy);
	priv->phy = NULL;
	gpio_free(priv->pwen_gpio);
	return 0;
}

static int usbhs_get_id(struct platform_device *pdev)
{
	return USBHS_GADGET;
}

static u32 lager_usbhs_pipe_type[] = {
	USB_ENDPOINT_XFER_CONTROL,
	USB_ENDPOINT_XFER_ISOC,
	USB_ENDPOINT_XFER_ISOC,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_INT,
	USB_ENDPOINT_XFER_INT,
	USB_ENDPOINT_XFER_INT,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
	USB_ENDPOINT_XFER_BULK,
};

static struct usbhs_private usbhs_priv __initdata = {
	.info = {
		.platform_callback = {
			.power_ctrl	= usbhs_power_ctrl,
			.hardware_init	= usbhs_hardware_init,
			.hardware_exit	= usbhs_hardware_exit,
			.get_id		= usbhs_get_id,
		},
		.driver_param = {
			.buswait_bwait	= 4,
			.pipe_type = lager_usbhs_pipe_type,
			.pipe_size = ARRAY_SIZE(lager_usbhs_pipe_type),
			.d0_rx_id	= USB_DMAC_SLAVE_USBHS_RX,
			.d1_tx_id	= USB_DMAC_SLAVE_USBHS_TX,
			.usb_dmac_xfer_size = 32,
		},
	}
};

static void __init lager_register_usbhs(void)
{
	usb_bind_phy("renesas_usbhs", 0, "usb_phy_rcar_gen2");
	platform_device_register_resndata(&platform_bus,
					  "renesas_usbhs", -1,
					  usbhs_resources,
					  ARRAY_SIZE(usbhs_resources),
					  &usbhs_priv.info,
					  sizeof(usbhs_priv.info));
}
#else
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

static void __init lager_add_usb0_device(void)
{
	usb_bind_phy("pci-rcar-gen2.0", 0, "usb_phy_rcar_gen2");
	platform_device_register_full(&pci0_info);
}
#endif

/* USBHS PHY */
static const struct rcar_gen2_phy_platform_data usbhs_phy_pdata __initconst = {
#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
	.chan0_pci = 0,	/* Channel 0 is USBHS */
#else
	.chan0_pci = 1,	/* Channel 0 is PCI USB */
#endif
#ifndef CONFIG_USB_XHCI_HCD
	.chan2_pci = 1,	/* Channel 2 is PCI USB */
#else
	.chan2_pci = 0,	/* Channel 2 is USB3.0 */
#endif
};

static const struct resource usbhs_phy_resources[] __initconst = {
	DEFINE_RES_MEM(0xe6590100, 0x100),
};

/* Internal PCI1 */
static const struct resource pci1_resources[] __initconst = {
	DEFINE_RES_MEM(0xee0b0000, 0x10000),	/* CFG */
	DEFINE_RES_MEM(0xee0a0000, 0x10000),	/* MEM */
	DEFINE_RES_IRQ(gic_spi(112)),
};

static const struct platform_device_info pci1_info __initconst = {
	.parent		= &platform_bus,
	.name		= "pci-rcar-gen2",
	.id		= 1,
	.res		= pci1_resources,
	.num_res	= ARRAY_SIZE(pci1_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

static void __init lager_add_usb1_device(void)
{
	usb_bind_phy("pci-rcar-gen2.1", 0, "usb_phy_rcar_gen2");
	platform_device_register_full(&pci1_info);
}

/* Internal PCI2 */
static const struct resource pci2_resources[] __initconst = {
	DEFINE_RES_MEM(0xee0d0000, 0x10000),	/* CFG */
	DEFINE_RES_MEM(0xee0c0000, 0x10000),	/* MEM */
	DEFINE_RES_IRQ(gic_spi(113)),
};

static const struct platform_device_info pci2_info __initconst = {
	.parent		= &platform_bus,
	.name		= "pci-rcar-gen2",
	.id		= 2,
	.res		= pci2_resources,
	.num_res	= ARRAY_SIZE(pci2_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

#ifndef CONFIG_USB_XHCI_HCD
static void __init lager_add_usb2_device(void)
{
	usb_bind_phy("pci-rcar-gen2.2", 0, "usb_phy_rcar_gen2");
	platform_device_register_full(&pci2_info);
};
#endif

/* VIN */
static const struct resource vin_resources[] __initconst = {
	/* VIN0 */
	DEFINE_RES_MEM(0xe6ef0000, 0x1000),
	DEFINE_RES_IRQ(gic_spi(188)),
	/* VIN1 */
	DEFINE_RES_MEM(0xe6ef1000, 0x1000),
	DEFINE_RES_IRQ(gic_spi(189)),
};

static void __init lager_add_vin_device(unsigned idx,
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

	BUG_ON(idx > 1);

	platform_device_register_full(&vin_info);
}

#define LAGER_CAMERA(idx, name, addr, pdata, flag)			\
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
	.i2c_adapter_id = 2,						\
	.module_name = name,						\
	.priv = pdata,							\
}

LAGER_CAMERA(0, "adv7612", 0x4c, NULL, RCAR_VIN_BT709);
LAGER_CAMERA(1, "adv7180", 0x20, NULL, RCAR_VIN_BT656);

static void __init lager_add_camera0_device(void)
{
	platform_device_register_data(&platform_bus, "soc-camera-pdrv", 0,
				      &cam0_link, sizeof(cam0_link));
	lager_add_vin_device(0, &vin0_pdata);
}

static void __init lager_add_camera1_device(void)
{
	platform_device_register_data(&platform_bus, "soc-camera-pdrv", 1,
				      &cam1_link, sizeof(cam1_link));
	lager_add_vin_device(1, &vin1_pdata);
}

/* VSP1 */
#if defined(CONFIG_VIDEO_RENESAS_VSP1)
static const struct vsp1_platform_data lager_vspr_pdata __initconst = {
	.features = VSP1_HAS_LUT | VSP1_HAS_SRU,
	.rpf_count = 5,
	.uds_count = 1,
	.wpf_count = 4,
};

static const struct vsp1_platform_data lager_vsps_pdata __initconst = {
	.features = VSP1_HAS_SRU,
	.rpf_count = 5,
	.uds_count = 3,
	.wpf_count = 4,
};

static const struct vsp1_platform_data lager_vspd0_pdata __initconst = {
	.features = VSP1_HAS_LIF | VSP1_HAS_LUT,
	.rpf_count = 4,
	.uds_count = 1,
	.wpf_count = 4,
};

static const struct vsp1_platform_data lager_vspd1_pdata __initconst = {
	.features = VSP1_HAS_LIF | VSP1_HAS_LUT,
	.rpf_count = 4,
	.uds_count = 1,
	.wpf_count = 4,
};

static const struct vsp1_platform_data * const lager_vsp1_pdata[4] __initconst = {
	&lager_vspr_pdata,
	&lager_vsps_pdata,
	&lager_vspd0_pdata,
	&lager_vspd1_pdata,
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

static void __init lager_add_vsp1_devices(void)
{
	struct platform_device_info info = {
		.name = "vsp1",
		.size_data = sizeof(*lager_vsp1_pdata[0]),
		.num_res = 2,
		.dma_mask = DMA_BIT_MASK(32),
	};
	unsigned int i;

	for (i = 2; i < ARRAY_SIZE(vsp1_resources); ++i) {
		info.id = i;
		info.data = lager_vsp1_pdata[i];
		info.res = vsp1_resources[i];

		platform_device_register_full(&info);
	}
}
#endif

/* MSIOF */
static struct sh_msiof_spi_info msiof1_info = {
	.rx_fifo_override       = 256,
	.num_chipselect         = 1,
	.dma_tx_id              = SYS_DMAC_SLAVE_MSIOF1_TX,
	.dma_rx_id              = SYS_DMAC_SLAVE_MSIOF1_RX,
};

/* MSIOF spidev */
static const struct spi_board_info spi_bus[] __initconst = {
	{
		.modalias	= "spidev",
		.max_speed_hz	= 6000000,
		.mode		= SPI_MODE_3,
		.bus_num	= 2,
		.chip_select	= 0,
	},
};

#define lager_add_msiof_device spi_register_board_info

/* POWER IC */
static struct i2c_board_info poweric_i2c[] = {
	{ I2C_BOARD_INFO("da9063", 0x58), },
};

static void lager_restart(char mode, const char *cmd)
{
	struct i2c_adapter *adap;
	struct i2c_client *client;
	u8 val;
	int busnum = 3;

	adap = i2c_get_adapter(busnum);
	if (!adap) {
		pr_err("failed to get adapter i2c%d\n", busnum);
		return;
	}

	client = i2c_new_device(adap, &poweric_i2c[0]);
	if (!client)
		pr_err("failed to register %s to i2c%d\n",
		       poweric_i2c[0].type, busnum);

	i2c_put_adapter(adap);

	val = i2c_smbus_read_byte_data(client, 0x13);

	if (val < 0)
		pr_err("couldn't access da9063\n");

	val |= 0x02;

	i2c_smbus_write_byte_data(client, 0x13, val);
}

static struct of_dev_auxdata lager_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("renesas,mmcif-r8a7790", 0xee220000, "sh_mmcif",
			&mmcif1_pdata),
	OF_DEV_AUXDATA("renesas,sdhi-r8a7790", 0xee100000, "sdhi0",
			&sdhi0_info),
	OF_DEV_AUXDATA("renesas,sdhi-r8a7790", 0xee140000, "sdhi2",
			&sdhi2_info),
	AUXDATA_SCIFA(0, 0xe6c40000, gic_spi(144)), /* SCIFA0 */
	AUXDATA_SCIFA(1, 0xe6c50000, gic_spi(145)), /* SCIFA1 */
	AUXDATA_SCIFB(2, 0xe6c20000, gic_spi(148)), /* SCIFB0 */
	AUXDATA_SCIFB(3, 0xe6c30000, gic_spi(149)), /* SCIFB1 */
	AUXDATA_SCIFB(4, 0xe6ce0000, gic_spi(150)), /* SCIFB2 */
	AUXDATA_SCIFA(5, 0xe6c60000, gic_spi(151)), /* SCIFA2 */
	AUXDATA_SCIF(6,  0xe6e60000, gic_spi(152)), /* SCIF0 */
	AUXDATA_SCIF(7,  0xe6e68000, gic_spi(153)), /* SCIF1 */
	AUXDATA_HSCIF(8, 0xe62c0000, gic_spi(154)), /* HSCIF0 */
	AUXDATA_HSCIF(9, 0xe62c8000, gic_spi(155)), /* HSCIF1 */
	OF_DEV_AUXDATA("renesas,msiof-r8a7790", 0xe6e10000,
		       "spi_r8a7790_msiof.0", &msiof1_info),
	{},
};

static void __init lager_add_standard_devices(void)
{
	shmobile_clk_workaround(clk_names, ARRAY_SIZE(clk_names), false);
	shmobile_clk_workaround(clk_enables, ARRAY_SIZE(clk_enables), true);
	r8a7790_add_dt_devices();
	lager_add_dmac_prototype();
#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
	lager_add_usb_dmac_prototype();
#endif
	of_platform_populate(NULL, of_default_bus_match_table,
			     lager_auxdata_lookup, NULL);

	lager_add_du_device();
	platform_device_register_resndata(&platform_bus, "usb_phy_rcar_gen2",
					  -1, usbhs_phy_resources,
					  ARRAY_SIZE(usbhs_phy_resources),
					  &usbhs_phy_pdata,
					  sizeof(usbhs_phy_pdata));
#if IS_ENABLED(CONFIG_USB_RENESAS_USBHS_UDC)
	lager_register_usbhs();
#else
	lager_add_usb0_device();
#endif
	lager_add_usb1_device();
#ifndef CONFIG_USB_XHCI_HCD
	lager_add_usb2_device();
#endif
	lager_add_rsnd_device();
	lager_add_camera0_device();
	lager_add_camera1_device();
#if defined(CONFIG_VIDEO_RENESAS_VSP1)
	lager_add_vsp1_devices();
#endif
	lager_add_msiof_device(spi_bus, ARRAY_SIZE(spi_bus));
}

static const char *lager_boards_compat_dt[] __initdata = {
	"renesas,lager",
	"renesas,lager-reference",
	NULL,
};

DT_MACHINE_START(LAGER_DT, "lager")
	.smp		= smp_ops(r8a7790_smp_ops),
	.init_early	= shmobile_init_delay,
	.init_time	= rcar_gen2_timer_init,
	.init_machine	= lager_add_standard_devices,
	.init_late	= shmobile_init_late,
	.reserve	= rcar_gen2_reserve,
	.restart	= lager_restart,
	.dt_compat	= lager_boards_compat_dt,
MACHINE_END
