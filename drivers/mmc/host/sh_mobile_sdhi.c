/*
 * SuperH Mobile SDHI
 *
 * Copyright (C) 2014 Renesas Electronics Corporation
 * Copyright (C) 2009 Magnus Damm
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on "Compaq ASIC3 support":
 *
 * Copyright 2001 Compaq Computer Corporation.
 * Copyright 2004-2005 Phil Blundell
 * Copyright 2007-2008 OpenedHand Ltd.
 *
 * Authors: Phil Blundell <pb@handhelds.org>,
 *	    Samuel Ortiz <sameo@openedhand.com>
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sh_mobile_sdhi.h>
#include <linux/mfd/tmio.h>
#include <linux/sh_dma.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include "tmio_mmc.h"

#define R8A7790_ES1_SDHI_WORKAROUND

/* Product device */
#define PRODUCT_REGISTER	0xFF000044
#define PRODUCT_CUT_MASK	(0x00007FF0)
#define PRODUCT_H2_BIT		(0x45 << 8)

/* SDHI host controller version */
#define SDHI_VERSION_CB0D	0xCB0D
#define SDHI_VERSION_490C	0x490C

#define EXT_ACC           0xe4
#define SD_DMACR(x)       ((x) ? 0x192 : 0xe6)

/* Maximum number DMA transfer size */
#define SH_MOBILE_SDHI_DMA_XMIT_SZ_MAX	6

/* SDHI host controller type */
enum {
	SH_MOBILE_SDHI_VER_490C = 0,
	SH_MOBILE_SDHI_VER_CB0D,
	SH_MOBILE_SDHI_VER_MAX, /* SDHI max */
};

/* SD buffer access size */
enum {
	SH_MOBILE_SDHI_EXT_ACC_16BIT = 0,
	SH_MOBILE_SDHI_EXT_ACC_32BIT,
	SH_MOBILE_SDHI_EXT_ACC_MAX, /* EXT_ACC access size max */
};

/* SD buffer access size for EXT_ACC */
static unsigned short sh_acc_size[][SH_MOBILE_SDHI_EXT_ACC_MAX] = {
	/* { 16bit, 32bit, }, */
	{ 0x0000, 0x0001, },	/* SH_MOBILE_SDHI_VER_490C */
	{ 0x0001, 0x0000, },	/* SH_MOBILE_SDHI_VER_CB0D */
};

struct sh_mobile_sdhi_scc {
	unsigned long clk;	/* clock for SDR104 */
	u32 tap;		/* sampling clock position for SDR104 */
};

struct sh_mobile_sdhi_of_data {
	unsigned long tmio_flags;
	unsigned long capabilities;
	unsigned long capabilities2;
	dma_addr_t dma_rx_offset;
	struct sh_mobile_sdhi_scc *taps;
	int taps_num;
};

static const struct sh_mobile_sdhi_of_data sh_mobile_sdhi_of_cfg[] = {
	{
		.tmio_flags = TMIO_MMC_HAS_IDLE_WAIT,
	},
};

static const struct sh_mobile_sdhi_of_data of_rcar_gen1_compatible = {
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_WRPROTECT_DISABLE,
	.capabilities	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ,
};

/* Definitions for sampling clocks */
static struct sh_mobile_sdhi_scc rcar_gen2_scc_taps[] = {
	{
		.clk = 195000000,
		.tap = 0x00000300,
	},
	{
		.clk = 156000000,
		.tap = 0x00000703,
	},
};

static const struct sh_mobile_sdhi_of_data of_rcar_gen2_compatible = {
	.tmio_flags	= TMIO_MMC_CLK_ACTUAL | TMIO_MMC_CLK_NO_SLEEP |
			  TMIO_MMC_HAS_IDLE_WAIT |
			  TMIO_MMC_SDIO_STATUS_QUIRK,
	.capabilities	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ,
	.capabilities2	= MMC_CAP2_NO_2BLKS_READ,
	.dma_rx_offset	= 0x2000,
	.taps = rcar_gen2_scc_taps,
	.taps_num = ARRAY_SIZE(rcar_gen2_scc_taps),
};

static const struct of_device_id sh_mobile_sdhi_of_match[] = {
	{ .compatible = "renesas,sdhi-shmobile" },
	{ .compatible = "renesas,sdhi-sh7372" },
	{ .compatible = "renesas,sdhi-sh73a0", .data = &sh_mobile_sdhi_of_cfg[0], },
	{ .compatible = "renesas,sdhi-r8a73a4", .data = &sh_mobile_sdhi_of_cfg[0], },
	{ .compatible = "renesas,sdhi-r8a7740", .data = &sh_mobile_sdhi_of_cfg[0], },
	{ .compatible = "renesas,sdhi-r8a7778", .data = &of_rcar_gen1_compatible, },
	{ .compatible = "renesas,sdhi-r8a7779", .data = &of_rcar_gen1_compatible, },
	{ .compatible = "renesas,sdhi-r8a7790", .data = &of_rcar_gen2_compatible, },
	{ .compatible = "renesas,sdhi-r8a7791", .data = &of_rcar_gen2_compatible, },
	{ .compatible = "renesas,sdhi-r8a7793", .data = &of_rcar_gen2_compatible, },
	{ .compatible = "renesas,sdhi-r8a7794", .data = &of_rcar_gen2_compatible, },
	{},
};
MODULE_DEVICE_TABLE(of, sh_mobile_sdhi_of_match);

/* transfer size for SD_DMACR */
static int sh_dma_size[][SH_MOBILE_SDHI_DMA_XMIT_SZ_MAX] = {
	/* { 1byte, 2byte, 4byte, 8byte, 16byte, 32byte, }, */
	{ -EINVAL, 0x0000, 0x0000, -EINVAL, 0x5000, 0xa000, },	/* VER_490C */
	{ -EINVAL, 0x0000, 0x0000, -EINVAL, 0x0001, 0x0004, },	/* VER_CB0D */
};

struct sh_mobile_sdhi_vlt {
	u32 base;		/* base address for IO voltage */
	u32 offset;		/* offset value for IO voltage */
	u32 mask;		/* bit mask position for IO voltage */
};

struct sh_mobile_sdhi {
	struct clk *clk;
	struct tmio_mmc_data mmc_data;
	struct tmio_mmc_dma dma_priv;
	unsigned int type;
	struct sh_mobile_sdhi_vlt vlt;
};

static int sh_mobile_sdhi_clk_enable(struct platform_device *pdev, unsigned int *f)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct tmio_mmc_host *host = mmc_priv(mmc);
	struct sh_mobile_sdhi *priv = container_of(host->pdata, struct sh_mobile_sdhi, mmc_data);
	int ret = clk_prepare_enable(priv->clk);
	if (ret < 0)
		return ret;

	*f = clk_get_rate(priv->clk);
	return 0;
}

static void sh_mobile_sdhi_clk_disable(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct tmio_mmc_host *host = mmc_priv(mmc);
	struct sh_mobile_sdhi *priv = container_of(host->pdata, struct sh_mobile_sdhi, mmc_data);
	clk_disable_unprepare(priv->clk);
}

#define SH_MOBILE_SDHI_SIGNAL_180V	0
#define SH_MOBILE_SDHI_SIGNAL_330V	1

static void sh_mobile_sdhi_set_ioctrl(struct tmio_mmc_host *host, int state)
{
	struct platform_device *pdev = host->pdev;
	void __iomem *pmmr, *ioctrl;
	unsigned int ctrl, mask;
	struct sh_mobile_sdhi *priv = container_of(host->pdata, struct sh_mobile_sdhi, mmc_data);
	struct sh_mobile_sdhi_vlt *vlt = &priv->vlt;

	if (!vlt)
		return;

	pmmr = ioremap(vlt->base, 0x04);
	ioctrl = ioremap(vlt->base + vlt->offset, 0x04);

	ctrl = ioread32(ioctrl);
	/* Set 1.8V/3.3V */
	mask = 0xff << (24 - vlt->mask * 8);

	if (state == SH_MOBILE_SDHI_SIGNAL_330V)
		ctrl |= mask;
	else if (state == SH_MOBILE_SDHI_SIGNAL_180V)
		ctrl &= ~mask;
	else {
		dev_err(&pdev->dev, "update_ioctrl: unknown state\n");
		goto err;
	}

	iowrite32(~ctrl, pmmr);
	iowrite32(ctrl, ioctrl);
err:
	iounmap(pmmr);
	iounmap(ioctrl);
}

static int sh_mobile_sdhi_start_signal_voltage_switch(
	struct tmio_mmc_host *host, unsigned char signal_voltage)
{
	struct mmc_host *mmc = host->mmc;
	int ret;

	if (signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		/* Enable 3.3V Signal */
		if (mmc->supply.vqmmc) {
			/* ioctrl */
			sh_mobile_sdhi_set_ioctrl(host,
						  SH_MOBILE_SDHI_SIGNAL_330V);
			ret = regulator_set_voltage(mmc->supply.vqmmc,
						    3300000, 3300000);
			if (ret) {
				dev_warn(&host->pdev->dev,
					 "3.3V signalling voltage failed\n");
				return -EIO;
			}
		}
		usleep_range(5000, 10000);
	} else if (signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		/* Enable 1.8V Signal */
		if (mmc->supply.vqmmc) {
			ret = regulator_set_voltage(mmc->supply.vqmmc,
						    1800000, 1800000);
			if (ret) {
				dev_warn(&host->pdev->dev,
					 "1.8V signalling voltage failed\n");
				return -EIO;
			}
			/* ioctrl */
			sh_mobile_sdhi_set_ioctrl(host,
						  SH_MOBILE_SDHI_SIGNAL_180V);
		}
		/* Wait for 5ms */
		usleep_range(5000, 10000);
	} else {
		/* No signal voltage switch required */
	}

	return 0;
}

#define SH_MOBILE_SDHI_DAT0	0x0080
static int sh_mobile_sdhi_card_busy(struct tmio_mmc_host *host)
{
	u16 dat0;

	/* check to see DAT[3:0] */
	dat0 = sd_ctrl_read16(host, CTL_STATUS2);
	return !(dat0 & SH_MOBILE_SDHI_DAT0);
}

static bool sh_mobile_sdhi_inquiry_tuning(struct tmio_mmc_host *host)
{
	/* SDHI should be tuning only SDR104 */
	if (host->mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		return true;
	else
		return false;
}

/* SCC registers */
#define SH_MOBILE_SDHI_SCC_DTCNTL	0x300
#define SH_MOBILE_SDHI_SCC_TAPSET	0x304
#define SH_MOBILE_SDHI_SCC_DT2FF	0x308
#define SH_MOBILE_SDHI_SCC_CKSEL	0x30C
#define SH_MOBILE_SDHI_SCC_RVSCNTL	0x310
#define SH_MOBILE_SDHI_SCC_RVSREQ	0x314

/* Definitions for values the SH_MOBILE_SDHI_SCC_DTCNTL register */
#define SH_MOBILE_SDHI_SCC_DTCNTL_TAPEN		(1 << 0)
/* Definitions for values the SH_MOBILE_SDHI_SCC_CKSEL register */
#define SH_MOBILE_SDHI_SCC_CKSEL_DTSEL		(1 << 0)
/* Definitions for values the SH_MOBILE_SDHI_SCC_RVSCNTL register */
#define SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN	(1 << 0)
/* Definitions for values the SH_MOBILE_SDHI_SCC_RVSREQ register */
#define SH_MOBILE_SDHI_SCC_RVSREQ_RVSERR	(1 << 2)

static void sh_mobile_sdhi_init_tuning(struct tmio_mmc_host *host,
							unsigned long *num)
{
	struct platform_device *pdev = host->pdev;
	struct device_node *np = pdev->dev.of_node;
	u32 scc_tapnum;
	struct mmc_host *mmc = dev_get_drvdata(&host->pdev->dev);
	const struct of_device_id *of_id =
		of_match_device(sh_mobile_sdhi_of_match, &pdev->dev);
	struct sh_mobile_sdhi_scc *taps;
	int i;
#ifdef R8A7790_ES1_SDHI_WORKAROUND
	void __iomem *product_reg;
#endif

	/* set sampling clock selection range */
	if (np && !of_property_read_u32(np, "renesas,mmc-scc-tapnum",
							&scc_tapnum)) {
		if (scc_tapnum)
#ifdef R8A7790_ES1_SDHI_WORKAROUND
			product_reg = ioremap_nocache(PRODUCT_REGISTER, 0x04);
			if (!product_reg) {
				dev_err(&pdev->dev,
					"Cannot ioremap_nocache\n");
				return;
			}

			if ((ioread32(product_reg) & PRODUCT_CUT_MASK) ==
								PRODUCT_H2_BIT)
				scc_tapnum = 10;
			iounmap(product_reg);
#endif
			writel(scc_tapnum << 16,
				host->ctl + SH_MOBILE_SDHI_SCC_DTCNTL);
	}

	/* Initialize SCC */
	sd_ctrl_write32(host, CTL_STATUS, 0x00000000);

	writel(SH_MOBILE_SDHI_SCC_DTCNTL_TAPEN |
		readl(host->ctl + SH_MOBILE_SDHI_SCC_DTCNTL),
		host->ctl + SH_MOBILE_SDHI_SCC_DTCNTL);

	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, ~0x0100 &
		sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));

	writel(SH_MOBILE_SDHI_SCC_CKSEL_DTSEL |
		readl(host->ctl + SH_MOBILE_SDHI_SCC_CKSEL),
		host->ctl + SH_MOBILE_SDHI_SCC_CKSEL);

	sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, 0x0100 |
		sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));

	writel(~SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN &
		readl(host->ctl + SH_MOBILE_SDHI_SCC_RVSCNTL),
		host->ctl + SH_MOBILE_SDHI_SCC_RVSCNTL);

	if (of_id && of_id->data) {
		const struct sh_mobile_sdhi_of_data *of_data = of_id->data;
		for (i = 0, taps = of_data->taps;
			i < of_data->taps_num; i++, taps++) {
			if (taps->clk == mmc->f_max) {
				writel(taps->tap,
					host->ctl + SH_MOBILE_SDHI_SCC_DT2FF);
				break;
			}
		}
	}
	if (taps->clk != mmc->f_max)
		dev_warn(&host->pdev->dev, "Unknown f_max for SDR104\n");

	/* Read TAPNUM */
	*num = (readl(host->ctl + SH_MOBILE_SDHI_SCC_DTCNTL) >> 16) & 0xf;

	return;
}

static int sh_mobile_sdhi_prepare_tuning(struct tmio_mmc_host *host,
							unsigned long tap)
{
	/* Set sampling clock position */
	writel(tap, host->ctl + SH_MOBILE_SDHI_SCC_TAPSET);

	return 0;
}

#define SH_MOBILE_SDHI_MAX_TAP	3
static int sh_mobile_sdhi_select_tuning(struct tmio_mmc_host *host,
							unsigned long *tap)
{
	unsigned long tap_num;	/* total number of taps */
	unsigned long tap_cnt;	/* counter of tuning success */
	unsigned long tap_set;	/* tap position */
	unsigned long tap_start;	/* start position of tuning success */
	unsigned long tap_end;	/* end position of tuning success */
	unsigned long ntap;	/* temporary counter of tuning success */
	unsigned long i;

	/* Clear SCC_RVSREQ */
	writel(0x00000000, host->ctl + SH_MOBILE_SDHI_SCC_RVSREQ);

	/* Select SCC */
	tap_num = (readl(host->ctl + SH_MOBILE_SDHI_SCC_DTCNTL) >> 16) & 0xf;

	tap_cnt = 0;
	ntap = 0;
	tap_start = 0;
	tap_end = 0;
	for (i = 0; i < tap_num * 2; i++) {
		if (tap[i] == 0)
			ntap++;
		else {
			if (ntap > tap_cnt) {
				tap_start = i - ntap;
				tap_end = i - 1;
				tap_cnt = ntap;
			}
			ntap = 0;
		}
	}

	if (ntap > tap_cnt) {
		tap_start = i - ntap;
		tap_end = i - 1;
		tap_cnt = ntap;
	}

	if (tap_cnt >= SH_MOBILE_SDHI_MAX_TAP)
		tap_set = (tap_start + tap_end) / 2 % tap_num;
	else
		return -EIO;

	/* Set SCC */
	writel(tap_set, host->ctl + SH_MOBILE_SDHI_SCC_TAPSET);

	/* Enable auto re-tuning */
	writel(SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN |
		readl(host->ctl + SH_MOBILE_SDHI_SCC_RVSCNTL),
		host->ctl + SH_MOBILE_SDHI_SCC_RVSCNTL);
	return 0;
}

static bool sh_mobile_sdhi_retuning(struct tmio_mmc_host *host)
{
	/* Check SCC error */
	if (readl(host->ctl + SH_MOBILE_SDHI_SCC_RVSCNTL) &
	    SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN &&
	    readl(host->ctl + SH_MOBILE_SDHI_SCC_RVSREQ) &
	    SH_MOBILE_SDHI_SCC_RVSREQ_RVSERR) {
		/* Clear SCC error */
		writel(0x00000000,
			host->ctl + SH_MOBILE_SDHI_SCC_RVSREQ);
		return true;
	}
	return false;
}

static void sh_mobile_sdhi_hw_reset(struct tmio_mmc_host *host)
{
	struct tmio_mmc_data *pdata = host->pdata;

	if (pdata->flags & TMIO_MMC_HAS_UHS_SCC) {
		/* Reset SCC */
		sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, ~0x0100 &
			sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));

		writel(~SH_MOBILE_SDHI_SCC_CKSEL_DTSEL &
			readl(host->ctl + SH_MOBILE_SDHI_SCC_CKSEL),
			host->ctl + SH_MOBILE_SDHI_SCC_CKSEL);

		sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, 0x0100 |
			sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));

		writel(~SH_MOBILE_SDHI_SCC_DTCNTL_TAPEN &
			readl(host->ctl + SH_MOBILE_SDHI_SCC_DTCNTL),
			host->ctl + SH_MOBILE_SDHI_SCC_DTCNTL);

		writel(~SH_MOBILE_SDHI_SCC_RVSCNTL_RVSEN &
			readl(host->ctl + SH_MOBILE_SDHI_SCC_RVSCNTL),
			host->ctl + SH_MOBILE_SDHI_SCC_RVSCNTL);
	}
}

static int sh_mobile_sdhi_wait_idle(struct tmio_mmc_host *host)
{
	int timeout = 1000;

	while (--timeout && !(sd_ctrl_read16(host, CTL_STATUS2) & (1 << 13)))
		udelay(1);

	if (!timeout) {
		dev_warn(host->pdata->dev, "timeout waiting for SD bus idle\n");
		return -EBUSY;
	}

	return 0;
}

static int sh_mobile_sdhi_write16_hook(struct tmio_mmc_host *host, int addr)
{
	switch (addr)
	{
	case CTL_SD_CMD:
	case CTL_STOP_INTERNAL_ACTION:
	case CTL_XFER_BLK_COUNT:
	case CTL_SD_CARD_CLK_CTL:
	case CTL_SD_XFER_LEN:
	case CTL_SD_MEM_CARD_OPT:
	case CTL_TRANSACTION_CTL:
	case CTL_DMA_ENABLE:
	case EXT_ACC:
		return sh_mobile_sdhi_wait_idle(host);
	}

	return 0;
}

#define SH_MOBILE_SDHI_DISABLE_AUTO_CMD12	0x4000

static void sh_mobile_sdhi_disable_auto_cmd12(int *val)
{
	*val |= SH_MOBILE_SDHI_DISABLE_AUTO_CMD12;

	return;
}

static void sh_mobile_sdhi_cd_wakeup(const struct platform_device *pdev)
{
	mmc_detect_change(platform_get_drvdata(pdev), msecs_to_jiffies(100));
}

static void sh_mobile_sdhi_set_clk_div(struct platform_device *pdev, int clk)
{
	struct mmc_host *mmc = dev_get_drvdata(&pdev->dev);
	struct tmio_mmc_host *host = mmc_priv(mmc);

	if (clk) {
		sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, ~0x0100 &
				sd_ctrl_read16(host, CTL_SD_CARD_CLK_CTL));
		sd_ctrl_write16(host, CTL_SD_CARD_CLK_CTL, 0x00ff);
	}
}

static int sh_mobile_sdhi_get_xmit_size(unsigned int type, int shift)
{
	int dma_size;

	if (type >= SH_MOBILE_SDHI_VER_MAX)
		return -EINVAL;
	if (shift >= SH_MOBILE_SDHI_DMA_XMIT_SZ_MAX)
		return -EINVAL;

	dma_size = sh_dma_size[type][shift];
	return dma_size;
}

static const struct sh_mobile_sdhi_ops sdhi_ops = {
	.cd_wakeup = sh_mobile_sdhi_cd_wakeup,
};

static void sh_mobile_sdhi_enable_sdbuf_acc32(struct tmio_mmc_host *host,
								int enable)
{
	struct sh_mobile_sdhi *priv = container_of(host->pdata,
						   struct sh_mobile_sdhi,
						   mmc_data);
	unsigned short acc_size;

	acc_size = enable ?
		sh_acc_size[priv->type][SH_MOBILE_SDHI_EXT_ACC_32BIT] :
		sh_acc_size[priv->type][SH_MOBILE_SDHI_EXT_ACC_16BIT];
	sd_ctrl_write16(host, EXT_ACC, acc_size);

}

static int sh_mobile_sdhi_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
		of_match_device(sh_mobile_sdhi_of_match, &pdev->dev);
	struct sh_mobile_sdhi *priv;
	struct tmio_mmc_data *mmc_data;
	struct sh_mobile_sdhi_info *p = pdev->dev.platform_data;
	const struct device_node *np = pdev->dev.of_node;
	struct tmio_mmc_host *host;
	struct resource *res;
	int irq, ret, i = 0;
	bool multiplexed_isr = true;
	struct tmio_mmc_dma *dma_priv;
	u16 ver;
	int dma_xmit_sz;
	int base, dma_size;
	int shift = 1; /* 2byte alignment */
	int clk_rate;
#ifdef R8A7790_ES1_SDHI_WORKAROUND
	void __iomem *product_reg;
#endif
	struct sh_mobile_sdhi_vlt *vlt;
	u32 pfcs[2], mask;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct sh_mobile_sdhi), GFP_KERNEL);
	if (priv == NULL) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	mmc_data = &priv->mmc_data;
	dma_priv = &priv->dma_priv;
	vlt = &priv->vlt;

	if (p) {
		if (p->init) {
			ret = p->init(pdev, &sdhi_ops);
			if (ret)
				return ret;
		}
	}

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		dev_err(&pdev->dev, "cannot get clock: %d\n", ret);
		goto eclkget;
	}

	if (np && !of_property_read_u32(np, "renesas,clk-rate", &clk_rate)) {
		if (clk_rate) {
#ifdef R8A7790_ES1_SDHI_WORKAROUND
			product_reg = ioremap_nocache(PRODUCT_REGISTER, 0x04);
			if (!product_reg) {
				dev_err(&pdev->dev,
					"Cannot ioremap_nocache\n");
				goto eclkget;
			}

			if ((ioread32(product_reg) & PRODUCT_CUT_MASK) ==
								PRODUCT_H2_BIT)
				if (clk_rate > 156000000)
					clk_rate = 156000000;
			iounmap(product_reg);
#endif
			ret = clk_set_rate(priv->clk, clk_rate);
			if (ret < 0)
				dev_err(&pdev->dev,
					"cannot set clock rate: %d\n", ret);
		}
	}

	if (np && !of_property_read_u32_array(np, "renesas,pfcs", pfcs, 2)) {
		if (pfcs[0]) {
			vlt->base = pfcs[0];
			vlt->offset = pfcs[1];
		}
	}

	if (np && !of_property_read_u32(np, "renesas,id", &mask))
		vlt->mask = mask;

	mmc_data->clk_enable = sh_mobile_sdhi_clk_enable;
	mmc_data->clk_disable = sh_mobile_sdhi_clk_disable;
	mmc_data->capabilities = MMC_CAP_MMC_HIGHSPEED;
	mmc_data->write16_hook = sh_mobile_sdhi_write16_hook;
	mmc_data->disable_auto_cmd12 = sh_mobile_sdhi_disable_auto_cmd12;
	mmc_data->set_clk_div = sh_mobile_sdhi_set_clk_div;
	mmc_data->start_signal_voltage_switch =
			sh_mobile_sdhi_start_signal_voltage_switch;
	mmc_data->card_busy = sh_mobile_sdhi_card_busy;
	mmc_data->inquiry_tuning = sh_mobile_sdhi_inquiry_tuning;
	mmc_data->init_tuning = sh_mobile_sdhi_init_tuning;
	mmc_data->prepare_tuning = sh_mobile_sdhi_prepare_tuning;
	mmc_data->select_tuning = sh_mobile_sdhi_select_tuning;
	mmc_data->retuning = sh_mobile_sdhi_retuning;
	mmc_data->hw_reset = sh_mobile_sdhi_hw_reset;
	if (p) {
		mmc_data->flags = p->tmio_flags;
		mmc_data->ocr_mask = p->tmio_ocr_mask;
		mmc_data->capabilities |= p->tmio_caps;
		mmc_data->capabilities2 |= p->tmio_caps2;
		mmc_data->cd_gpio = p->cd_gpio;
		dma_priv->dma_rx_offset = p->dma_rx_offset;

		if (p->dma_slave_tx > 0 && p->dma_slave_rx > 0) {
			/*
			 * Yes, we have to provide slave IDs twice to TMIO:
			 * once as a filter parameter and once for channel
			 * configuration as an explicit slave ID
			 */
			dma_priv->chan_priv_tx = (void *)p->dma_slave_tx;
			dma_priv->chan_priv_rx = (void *)p->dma_slave_rx;
			dma_priv->slave_id_tx = p->dma_slave_tx;
			dma_priv->slave_id_rx = p->dma_slave_rx;
			mmc_data->enable_sdbuf_acc32 =
				sh_mobile_sdhi_enable_sdbuf_acc32;
		}
	}

	if (np && !of_property_read_u32(np, "dma-xmit-sz", &dma_xmit_sz)) {
		if (dma_xmit_sz) {
			base = dma_xmit_sz;
			for (shift = 0; base > 1; shift++)
				base >>= 1;
		}
	}

	dma_priv->alignment_shift = shift;
	dma_priv->filter = shdma_chan_filter;

	mmc_data->dma = dma_priv;

	/*
	 * All SDHI blocks support 2-byte and larger block sizes in 4-bit
	 * bus width mode.
	 */
	mmc_data->flags |= TMIO_MMC_BLKSZ_2BYTES;

	/*
	 * All SDHI blocks support SDIO IRQ signalling.
	 */
	mmc_data->flags |= TMIO_MMC_SDIO_IRQ;

	if (of_id && of_id->data) {
		const struct sh_mobile_sdhi_of_data *of_data = of_id->data;
		mmc_data->flags |= of_data->tmio_flags;
		mmc_data->capabilities |= of_data->capabilities;
		mmc_data->capabilities2 |= of_data->capabilities2;
		dma_priv->dma_rx_offset = of_data->dma_rx_offset;
	}

	if (of_find_property(np, "cap-uhs-sdr50", NULL))
		mmc_data->capabilities |= MMC_CAP_UHS_SDR50;
	if (of_find_property(np, "cap-uhs-sdr104", NULL))
		mmc_data->capabilities |= MMC_CAP_UHS_SDR104;

	if (mmc_data->capabilities & MMC_CAP_UHS_SDR104) {
		mmc_data->capabilities |= MMC_CAP_CMD23 |
					  MMC_CAP_HW_RESET;
		mmc_data->flags |= TMIO_MMC_HAS_UHS_SCC;
	}

	/* SD control register space size is 0x100, 0x200 for bus_shift=1 */
	mmc_data->bus_shift = resource_size(res) >> 9;

	ret = tmio_mmc_host_probe(&host, pdev, mmc_data);
	if (ret < 0)
		goto eprobe;

	pm_runtime_get_sync(&pdev->dev);

	ver = sd_ctrl_read16(host, CTL_VERSION);
	if (ver == SDHI_VERSION_CB0D)
		priv->type = SH_MOBILE_SDHI_VER_CB0D;
	else if (ver == SDHI_VERSION_490C)
		priv->type = SH_MOBILE_SDHI_VER_490C;
	else {
		dev_err(host->pdata->dev, "Unknown SDHI version\n");
		goto eirq;
	}

	/*
	 * FIXME:
	 * this Workaround can be more clever method
	 */
	sh_mobile_sdhi_enable_sdbuf_acc32(host, false);

	/* Some controllers check the ILL_FUNC bit. */
	if (priv->type == SH_MOBILE_SDHI_VER_490C)
		mmc_data->flags |= TMIO_MMC_CHECK_ILL_FUNC;

	/* Set DMA xmit size */
	if (p && p->dma_slave_tx > 0 && p->dma_slave_rx > 0) {
		dma_size = sh_mobile_sdhi_get_xmit_size(priv->type,
					priv->dma_priv.alignment_shift);
		if (dma_size < 0) {
			ret = dma_size;
			goto eirq;
		}
		sd_ctrl_write16(host, SD_DMACR(priv->type), dma_size);
	}

	/*
	 * Allow one or more specific (named) ISRs or
	 * one or more multiplexed (un-named) ISRs.
	 */

	irq = platform_get_irq_byname(pdev, SH_MOBILE_SDHI_IRQ_CARD_DETECT);
	if (irq >= 0) {
		multiplexed_isr = false;
		ret = devm_request_irq(&pdev->dev, irq, tmio_mmc_card_detect_irq, 0,
				  dev_name(&pdev->dev), host);
		if (ret)
			goto eirq;
	}

	irq = platform_get_irq_byname(pdev, SH_MOBILE_SDHI_IRQ_SDIO);
	if (irq >= 0) {
		multiplexed_isr = false;
		ret = devm_request_irq(&pdev->dev, irq, tmio_mmc_sdio_irq, 0,
				  dev_name(&pdev->dev), host);
		if (ret)
			goto eirq;
	}

	irq = platform_get_irq_byname(pdev, SH_MOBILE_SDHI_IRQ_SDCARD);
	if (irq >= 0) {
		multiplexed_isr = false;
		ret = devm_request_irq(&pdev->dev, irq, tmio_mmc_sdcard_irq, 0,
				  dev_name(&pdev->dev), host);
		if (ret)
			goto eirq;
	} else if (!multiplexed_isr) {
		dev_err(&pdev->dev,
			"Principal SD-card IRQ is missing among named interrupts\n");
		ret = irq;
		goto eirq;
	}

	if (multiplexed_isr) {
		while (1) {
			irq = platform_get_irq(pdev, i);
			if (irq < 0)
				break;
			i++;
			ret = devm_request_irq(&pdev->dev, irq, tmio_mmc_irq, 0,
					  dev_name(&pdev->dev), host);
			if (ret)
				goto eirq;
		}

		/* There must be at least one IRQ source */
		if (!i) {
			ret = irq;
			goto eirq;
		}
	}

	dev_info(&pdev->dev, "%s base at 0x%08lx clock rate %u MHz\n",
		 mmc_hostname(host->mmc), (unsigned long)
		 (platform_get_resource(pdev, IORESOURCE_MEM, 0)->start),
		 host->mmc->f_max / 1000000);

	pm_runtime_put(&pdev->dev);

	return ret;

eirq:
	pm_runtime_put(&pdev->dev);
	tmio_mmc_host_remove(host);
eprobe:
eclkget:
	if (p && p->cleanup)
		p->cleanup(pdev);
	return ret;
}

static int sh_mobile_sdhi_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct tmio_mmc_host *host = mmc_priv(mmc);
	struct sh_mobile_sdhi_info *p = pdev->dev.platform_data;

	tmio_mmc_host_remove(host);

	if (p && p->cleanup)
		p->cleanup(pdev);

	return 0;
}

static const struct dev_pm_ops tmio_mmc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tmio_mmc_host_suspend, tmio_mmc_host_resume)
	SET_RUNTIME_PM_OPS(tmio_mmc_host_runtime_suspend,
			tmio_mmc_host_runtime_resume,
			NULL)
};

static struct platform_driver sh_mobile_sdhi_driver = {
	.driver		= {
		.name	= "sh_mobile_sdhi",
		.owner	= THIS_MODULE,
		.pm	= &tmio_mmc_dev_pm_ops,
		.of_match_table = sh_mobile_sdhi_of_match,
	},
	.probe		= sh_mobile_sdhi_probe,
	.remove		= sh_mobile_sdhi_remove,
};

module_platform_driver(sh_mobile_sdhi_driver);

MODULE_DESCRIPTION("SuperH Mobile SDHI driver");
MODULE_AUTHOR("Magnus Damm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sh_mobile_sdhi");
