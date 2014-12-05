/*  Renesas Ethernet AVB device driver
 *
 *  Copyright (C) 2014  Renesas Electronics Corporation
 *  Copyright (C) 2006-2012 Nobuhiro Iwamatsu
 *  Copyright (C) 2008-2014 Renesas Solutions Corp.
 *  Copyright (C) 2013-2014 Cogent Embedded, Inc.
 *  Copyright (C) 2014 Codethink Limited
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms and conditions of the GNU General Public License,
 *  version 2, as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 *  The full GNU General Public License is included in this distribution in
 *  the file called "COPYING".
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mdio-bitbang.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/cache.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/clk.h>
#include <linux/sh_eth.h>
#include <linux/of_mdio.h>
#include <linux/net_tstamp.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "ravb.h"

#define RAVB_DEF_MSG_ENABLE \
		(NETIF_MSG_LINK	| \
		NETIF_MSG_TIMER	| \
		NETIF_MSG_RX_ERR| \
		NETIF_MSG_TX_ERR)

static const u16 ravb_offset_rcar_gen2[RAVB_MAX_REGISTER_OFFSET] = {
	/* AVB-DMAC registers */
	[CCC] = 0x0000,
	[DBAT] = 0x0004,
	[DLR] = 0x0008,
	[CSR] = 0x000C,
	[CDAR0] = 0x0010,
	[CDAR1] = 0x0014,
	[CDAR2] = 0x0018,
	[CDAR3] = 0x001C,
	[CDAR4] = 0x0020,
	[CDAR5] = 0x0024,
	[CDAR6] = 0x0028,
	[CDAR7] = 0x002C,
	[CDAR8] = 0x0030,
	[CDAR9] = 0x0034,
	[CDAR10] = 0x0038,
	[CDAR11] = 0x003C,
	[CDAR12] = 0x0040,
	[CDAR13] = 0x0044,
	[CDAR14] = 0x0048,
	[CDAR15] = 0x004C,
	[CDAR16] = 0x0050,
	[CDAR17] = 0x0054,
	[CDAR18] = 0x0058,
	[CDAR19] = 0x005C,
	[CDAR20] = 0x0060,
	[CDAR21] = 0x0064,
	[ESR] = 0x0088,
	[PSR] = 0x008C,
	[RCR] = 0x0090,
	[RQC] = 0x0094,
	[RPC] = 0x00B0,
	[UFCW] = 0x00BC,
	[UFCS] = 0x00C0,
	[UFCV0] = 0x00C4,
	[UFCV1] = 0x00C8,
	[UFCV2] = 0x00CC,
	[UFCV3] = 0x00D0,
	[UFCV4] = 0x00D4,
	[UFCD0] = 0x00E0,
	[UFCD1] = 0x00E4,
	[UFCD2] = 0x00E8,
	[UFCD3] = 0x00EC,
	[UFCD4] = 0x00F0,
	[SFO] = 0x00FC,
	[SFP0] = 0x0100,
	[SFP1] = 0x0104,
	[SFP2] = 0x0108,
	[SFP3] = 0x010c,
	[SFP4] = 0x0110,
	[SFP5] = 0x0114,
	[SFP6] = 0x0118,
	[SFP7] = 0x011c,
	[SFP8] = 0x0120,
	[SFP9] = 0x0124,
	[SFP10] = 0x0128,
	[SFP11] = 0x012c,
	[SFP12] = 0x0130,
	[SFP13] = 0x0134,
	[SFP14] = 0x0138,
	[SFP15] = 0x013c,
	[SFP16] = 0x0140,
	[SFP17] = 0x0144,
	[SFP18] = 0x0148,
	[SFP19] = 0x014c,
	[SFP20] = 0x0150,
	[SFP21] = 0x0154,
	[SFP22] = 0x0158,
	[SFP23] = 0x015c,
	[SFP24] = 0x0160,
	[SFP25] = 0x0164,
	[SFP26] = 0x0168,
	[SFP27] = 0x016c,
	[SFP28] = 0x0170,
	[SFP29] = 0x0174,
	[SFP30] = 0x0178,
	[SFP31] = 0x017c,
	[SFM0] = 0x01C0,
	[SFM1] = 0x01C4,
	[RTSR] = 0x01D0,
	[CIAR] = 0x0200,
	[LIAR] = 0x0280,
	[TGC] = 0x0300,
	[TCCR] = 0x0304,
	[TSR] = 0x0308,
	[MFA] = 0x030C,
	[TFA0] = 0x0310,
	[TFA1] = 0x0314,
	[TFA2] = 0x0318,
	[CIVR0] = 0x0320,
	[CIVR1] = 0x0324,
	[CDVR0] = 0x0328,
	[CDVR1] = 0x032C,
	[CUL0] = 0x0330,
	[CUL1] = 0x0334,
	[CLL0] = 0x0338,
	[CLL1] = 0x033C,
	[DIC] = 0x0350,
	[DIS] = 0x0354,
	[EIC] = 0x0358,
	[EIS] = 0x035C,
	[RIC0] = 0x0360,
	[RIS0] = 0x0364,
	[RIC1] = 0x0368,
	[RIS1] = 0x036C,
	[RIC2] = 0x0370,
	[RIS2] = 0x0374,
	[TIC] = 0x0378,
	[TIS] = 0x037C,
	[ISS] = 0x0380,
	[GCCR] = 0x0390,
	[GMTT] 0x0394,
	[GPTC] = 0x0398,
	[GTI] = 0x039C,
	[GTO0] = 0x03A0,
	[GTO1] = 0x03A4,
	[GTO2] = 0x03A8,
	[GIC] = 0x03AC,
	[GIS] = 0x03B0,
	[GCPT] = 0x03B4,
	[GCT0] = 0x03B8,
	[GCT1] = 0x03BC,
	[GCT2] = 0x03C0,

	[ECMR]	= 0x0500,
	[ECSR]	= 0x0510,
	[ECSIPR]	= 0x0518,
	[PIR]	= 0x0520,
	[PSR]	= 0x0528,
	[PIPR]	= 0x052c,
	[RFLR]	= 0x0508,
	[APR]	= 0x0554,
	[MPR]	= 0x0558,
	[PFTCR]	= 0x055c,
	[PFRCR]	= 0x0560,
	[TPAUSER]	= 0x0564,
	[GECMR]	= 0x05b0,
	[BCULR]	= 0x05b4,
	[MAHR]	= 0x05c0,
	[MALR]	= 0x05c8,
	[TROCR]	= 0x0700,
	[CDCR]	= 0x0708,
	[LCCR]	= 0x0710,
	[CEFCR]	= 0x0740,
	[FRECR]	= 0x0748,
	[TSFRCR]	= 0x0750,
	[TLFRCR]	= 0x0758,
	[RFCR]	= 0x0760,
	[CERCR]	= 0x0768,
	[CEECR]	= 0x0770,
	[MAFCR]	= 0x0778,
};

static void ravb_set_duplex(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	if (mdp->duplex) /* Full */
		ravb_write(ndev, ravb_read(ndev, ECMR) | ECMR_DM, ECMR);
	else		/* Half */
		ravb_write(ndev, ravb_read(ndev, ECMR) & ~ECMR_DM, ECMR);
}

/* There is CPU dependent code */
static int ravb_wait_status(struct net_device *ndev, u32 reg, u32 status)
{
	int i, ret = 0;

	for (i = 0; i < 100; i++) {
		if (!(ravb_read(ndev, reg) & status))
			break;
		mdelay(1);
	}
	if (i >= 100)
		return -ETIMEDOUT;
	return ret;
}

static int ravb_check_reset(struct net_device *ndev)
{
	int ret = 0;
	int cnt = 100;

	while (cnt > 0) {
		if (ravb_read(ndev, CSR) & 0x2)
			break;
		mdelay(1);
		cnt--;
	}
	if (cnt <= 0) {
		netdev_err(ndev, "Device reset failed\n");
		ret = -ETIMEDOUT;
	}
	return ret;
}

static int ravb_reset(struct net_device *ndev)
{
	int ret = 0;

	ravb_write(ndev,
		(ravb_read(ndev, CCC) & ~CCC_OPC) | 0x1, CCC);
	ret = ravb_check_reset(ndev);

	return ret;
}

static void ravb_set_rate(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	switch (mdp->speed) {
	case 100:/* 100BASE */
		ravb_write(ndev, 0x00, GECMR);
		break;
	case 1000: /* 1000BASE */
		ravb_write(ndev, 0x01, GECMR);
		break;
	default:
		break;
	}
}

/* R8A779x */
static struct ravb_cpu_data r8a779x_data_giga = {
	.set_duplex	= ravb_set_duplex,
	.set_rate	= ravb_set_rate,

	.register_type	= RAVB_REG_RCAR_GEN2,

	.csel_value = 0x00010000,
	.gti_value = ((1000*(1<<20)) / 130),

	.ecsr_value	= ECSR_ICD | ECSR_MPD,
	.ecsipr_value	= ECSIPR_LCHNGIP | ECSIPR_ICDIP | ECSIPR_MPDIP,

	.irq_flags	= IRQF_SHARED,
	.apr		= 0,
	.mpr		= 1,
	.tpauser	= 0,
	.hw_swap	= 1,
	.need_txalign = 1,
};

static void ravb_set_default_cpu_data(struct ravb_cpu_data *cd)
{
	if (!cd->ecsr_value)
		cd->ecsr_value = DEFAULT_ECSR_INIT;

	if (!cd->ecsipr_value)
		cd->ecsipr_value = DEFAULT_ECSIPR_INIT;
}

static void ravb_set_buff_align(struct sk_buff *skb)
{
	u32 reserve = (u32)skb->data & (RAVB_ALIGN - 1);
	if (reserve)
		skb_reserve(skb, RAVB_ALIGN - reserve);
}


/* CPU <-> EDMAC endian convert */
static inline __u32 cpu_to_edmac(struct ravb_private *mdp, u32 x)
{
	switch (mdp->edmac_endian) {
	case EDMAC_LITTLE_ENDIAN:
		return cpu_to_le32(x);
	case EDMAC_BIG_ENDIAN:
		return cpu_to_be32(x);
	}
	return x;
}

static inline __u32 edmac_to_cpu(struct ravb_private *mdp, u32 x)
{
	switch (mdp->edmac_endian) {
	case EDMAC_LITTLE_ENDIAN:
		return le32_to_cpu(x);
	case EDMAC_BIG_ENDIAN:
		return be32_to_cpu(x);
	}
	return x;
}

/* Program the hardware MAC address from dev->dev_addr. */
static void update_mac_address(struct net_device *ndev)
{
	ravb_write(ndev,
		     (ndev->dev_addr[0] << 24) | (ndev->dev_addr[1] << 16) |
		     (ndev->dev_addr[2] << 8) | (ndev->dev_addr[3]), MAHR);
	ravb_write(ndev,
		     (ndev->dev_addr[4] << 8) | (ndev->dev_addr[5]), MALR);
}

/* Get MAC address from SuperH MAC address register
 *
 * SuperH's Ethernet device doesn't have 'ROM' to MAC address.
 * This driver get MAC address that use by bootloader(U-boot or sh-ipl+g).
 * When you want use this device, you must set MAC address in bootloader.
 *
 */
static void read_mac_address(struct net_device *ndev, unsigned char *mac)
{
	if (mac[0] || mac[1] || mac[2] || mac[3] || mac[4] || mac[5]) {
		memcpy(ndev->dev_addr, mac, 6);
	} else {
		ndev->dev_addr[0] = (ravb_read(ndev, MAHR) >> 24);
		ndev->dev_addr[1] = (ravb_read(ndev, MAHR) >> 16) & 0xFF;
		ndev->dev_addr[2] = (ravb_read(ndev, MAHR) >> 8) & 0xFF;
		ndev->dev_addr[3] = (ravb_read(ndev, MAHR) & 0xFF);
		ndev->dev_addr[4] = (ravb_read(ndev, MALR) >> 8) & 0xFF;
		ndev->dev_addr[5] = (ravb_read(ndev, MALR) & 0xFF);
	}
}

struct bb_info {
	void (*set_gate)(void *addr);
	struct mdiobb_ctrl ctrl;
	void *addr;
	u32 mmd_msk;/* MMD */
	u32 mdo_msk;
	u32 mdi_msk;
	u32 mdc_msk;
};

/* PHY bit set */
static void bb_set(void *addr, u32 msk)
{
	iowrite32(ioread32(addr) | msk, addr);
}

/* PHY bit clear */
static void bb_clr(void *addr, u32 msk)
{
	iowrite32((ioread32(addr) & ~msk), addr);
}

/* PHY bit read */
static int bb_read(void *addr, u32 msk)
{
	return (ioread32(addr) & msk) != 0;
}

/* Data I/O pin control */
static void sh_mmd_ctrl(struct mdiobb_ctrl *ctrl, int bit)
{
	struct bb_info *bitbang = container_of(ctrl, struct bb_info, ctrl);

	if (bitbang->set_gate)
		bitbang->set_gate(bitbang->addr);

	if (bit)
		bb_set(bitbang->addr, bitbang->mmd_msk);
	else
		bb_clr(bitbang->addr, bitbang->mmd_msk);
}

/* Set bit data*/
static void sh_set_mdio(struct mdiobb_ctrl *ctrl, int bit)
{
	struct bb_info *bitbang = container_of(ctrl, struct bb_info, ctrl);

	if (bitbang->set_gate)
		bitbang->set_gate(bitbang->addr);

	if (bit)
		bb_set(bitbang->addr, bitbang->mdo_msk);
	else
		bb_clr(bitbang->addr, bitbang->mdo_msk);
}

/* Get bit data*/
static int sh_get_mdio(struct mdiobb_ctrl *ctrl)
{
	struct bb_info *bitbang = container_of(ctrl, struct bb_info, ctrl);

	if (bitbang->set_gate)
		bitbang->set_gate(bitbang->addr);

	return bb_read(bitbang->addr, bitbang->mdi_msk);
}

/* MDC pin control */
static void sh_mdc_ctrl(struct mdiobb_ctrl *ctrl, int bit)
{
	struct bb_info *bitbang = container_of(ctrl, struct bb_info, ctrl);

	if (bitbang->set_gate)
		bitbang->set_gate(bitbang->addr);

	if (bit)
		bb_set(bitbang->addr, bitbang->mdc_msk);
	else
		bb_clr(bitbang->addr, bitbang->mdc_msk);
}

/* mdio bus control struct */
static struct mdiobb_ops bb_ops = {
	.owner = THIS_MODULE,
	.set_mdc = sh_mdc_ctrl,
	.set_mdio_dir = sh_mmd_ctrl,
	.set_mdio_data = sh_set_mdio,
	.get_mdio_data = sh_get_mdio,
};

/* free skb and descriptor buffer for Ethernet AVB */
static void ravb_ring_free(struct net_device *ndev, int q)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int i;

	/* Free Rx skb ringbuffer */
	if (mdp->rx_skbuff[q]) {
		for (i = 0; i < mdp->num_rx_ring[q]; i++) {
			if (mdp->rx_skbuff[q][i])
				dev_kfree_skb(mdp->rx_skbuff[q][i]);
		}
	}
	kfree(mdp->rx_skbuff[q]);
	mdp->rx_skbuff[q] = NULL;

	/* Free Tx skb ringbuffer */
	if (mdp->tx_skbuff[q]) {
		for (i = 0; i < mdp->num_tx_ring[q]; i++) {
			if (mdp->tx_skbuff[q][i])
				dev_kfree_skb(mdp->tx_skbuff[q][i]);
		}
	}
	kfree(mdp->tx_skbuff[q]);
	mdp->tx_skbuff[q] = NULL;

	/* Free aligned Tx skb ringbuffer */
	if (mdp->tx_skbuff_aligned[q]) {
		for (i = 0; i < mdp->num_tx_ring[q]; i++) {
			if (mdp->tx_skbuff_aligned[q][i])
				dev_kfree_skb(mdp->tx_skbuff_aligned[q][i]);
		}
	}
	kfree(mdp->tx_skbuff_aligned[q]);
	mdp->tx_skbuff_aligned[q] = NULL;
}

/* format skb and descriptor buffer for Ethernet AVB */
static void ravb_ring_format(struct net_device *ndev, int q)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int i;
	struct sk_buff *skb;
	struct sk_buff *tx_skb;
	struct ravb_desc *desc = NULL;
	struct ravb_txdesc *txdesc = NULL;
	struct ravb_ex_rxdesc *rxdesc = NULL;
	int rx_ringsize = sizeof(*rxdesc) * mdp->num_rx_ring[q];
	int tx_ringsize = sizeof(*txdesc) * mdp->num_tx_ring[q];
	int skbuff_size = mdp->rx_buf_sz + RAVB_ALIGN - 1;

	mdp->cur_rx[q] = mdp->cur_tx[q] = 0;
	mdp->dirty_rx[q] = mdp->dirty_tx[q] = 0;
	memset(mdp->rx_ring[q], 0, rx_ringsize);
	/* build Rx ring buffer */
	for (i = 0; i < mdp->num_rx_ring[q]; i++) {
		/* skb */
		mdp->rx_skbuff[q][i] = NULL;
		skb = netdev_alloc_skb(ndev, skbuff_size);
		mdp->rx_skbuff[q][i] = skb;
		if (skb == NULL)
			break;
		ravb_set_buff_align(skb);
		/* RX descriptor */
		rxdesc = &mdp->rx_ring[q][i];
		/* The size of the buffer is 16 byte boundary. */
		rxdesc->ds = ALIGN(mdp->rx_buf_sz, 16);
		dma_map_single(&ndev->dev, skb->data, rxdesc->ds,
				DMA_FROM_DEVICE);
		rxdesc->dptr = virt_to_phys(skb->data);
		if (dma_mapping_error(&ndev->dev, rxdesc->dptr)) {
			dev_kfree_skb(mdp->rx_skbuff[q][i]);
			mdp->rx_skbuff[q][i] = NULL;
			break;
		}
		rxdesc->dt = DT_FEMPTY;
	}
	rxdesc = &mdp->rx_ring[q][i];
	rxdesc->dptr = (u32)mdp->rx_desc_dma[q];
	rxdesc->dt = DT_LINKFIX; /* type */
	mdp->dirty_rx[q] = (u32) (i - mdp->num_rx_ring[q]);

	memset(mdp->tx_ring[q], 0, tx_ringsize);
	/* build Tx ring buffer */
	if (mdp->cd->need_txalign) {
		for (i = 0; i < mdp->num_tx_ring[q]; i++) {
			/* skb buffer for alignment */
			mdp->tx_skbuff[q][i] = NULL;
			mdp->tx_skbuff_aligned[q][i] = NULL;
			tx_skb = netdev_alloc_skb(ndev, skbuff_size);
			if (tx_skb == NULL)
				break;
			ravb_set_buff_align(tx_skb);

			/* skb buffer for alignment */
			mdp->tx_skbuff_aligned[q][i] = tx_skb;
			txdesc = &mdp->tx_ring[q][i];
			txdesc->dptr = virt_to_phys(tx_skb->data);
			txdesc->dt = DT_EEMPTY;
		}
		txdesc = &mdp->tx_ring[q][i];
		txdesc->dptr = (u32)mdp->tx_desc_dma[q];
		txdesc->dt = DT_LINKFIX; /* type */
	}

	/* rx descriptor base address for best effort */
	desc = &mdp->desc_bat[RX_QUEUE_OFFSET + q];
	desc->dt = DT_LINKFIX; /* type */
	desc->dptr = (u32)mdp->rx_desc_dma[q];

	/* tx descriptor base address for best effort */
	desc = &mdp->desc_bat[q];
	desc->dt = DT_LINKFIX; /* type */
	desc->dptr = (u32)mdp->tx_desc_dma[q];
}

/* Get skb and descriptor buffer for Ethernet AVB */
static int ravb_ring_init(struct net_device *ndev, int q)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int rx_ringsize = 0, tx_ringsize, ret = 0;

	/* Allocate RX and TX skb rings */
	mdp->rx_skbuff[q] = kmalloc_array(
				mdp->num_rx_ring[q],
				sizeof(*mdp->rx_skbuff[q]),
				GFP_KERNEL);
	if (!mdp->rx_skbuff[q]) {
		ret = -ENOMEM;
		return ret;
	}

	mdp->tx_skbuff[q] = kmalloc_array(
				mdp->num_tx_ring[q],
				sizeof(*mdp->tx_skbuff[q]),
				GFP_KERNEL);
	if (!mdp->tx_skbuff[q]) {
		ret = -ENOMEM;
		goto skb_ring_free;
	}

	/* Allocate skb rings for buffer align */
	if (mdp->cd->need_txalign) {
		mdp->tx_skbuff_aligned[q] = kmalloc_array(
				mdp->num_tx_ring[q],
				sizeof(*mdp->tx_skbuff_aligned[q]),
				GFP_KERNEL);
		if (!mdp->tx_skbuff_aligned[q]) {
			ret = -ENOMEM;
			goto skb_ring_free;
		}
	}

	/* Allocate all Rx descriptors. */
	rx_ringsize = sizeof(struct ravb_ex_rxdesc);
	rx_ringsize *= mdp->num_rx_ring[q] + 1;
	mdp->rx_ring[q] = dma_alloc_coherent(NULL,
			rx_ringsize, &mdp->rx_desc_dma[q],
			GFP_KERNEL);
	if (!mdp->rx_ring[q]) {
		ret = -ENOMEM;
		goto skb_ring_free;
	}

	mdp->dirty_rx[q] = 0;

	/* Allocate all Tx descriptors. */
	tx_ringsize = sizeof(struct ravb_txdesc);
	tx_ringsize *= mdp->num_tx_ring[q] + 1;
	mdp->tx_ring[q] = dma_alloc_coherent(NULL,
			tx_ringsize, &mdp->tx_desc_dma[q],
			GFP_KERNEL);
	if (!mdp->tx_ring[q]) {
		ret = -ENOMEM;
		goto desc_ring_free;
	}

	return ret;

desc_ring_free:
	/* free DMA buffer */
	dma_free_coherent(NULL, rx_ringsize,
		mdp->rx_ring[q], mdp->rx_desc_dma[q]);

skb_ring_free:
	/* Free Rx and Tx skb ring buffer */
	ravb_ring_free(ndev, q);
	mdp->tx_ring[q] = NULL;
	mdp->rx_ring[q] = NULL;

	return ret;
}

static void ravb_free_dma_buffer(struct ravb_private *mdp)
{
	int ringsize;
	int q;

	for (q = RAVB_BE; q < NUM_RX_QUEUE; q++) {
		if (mdp->rx_ring[q]) {
			ringsize = sizeof(struct ravb_ex_rxdesc);
			ringsize *= mdp->num_rx_ring[q] + 1;
			dma_free_coherent(NULL,
					ringsize, mdp->rx_ring[q],
					mdp->rx_desc_dma[q]);
			mdp->rx_ring[q] = NULL;
		}
	}

	for (q = RAVB_BE; q < NUM_TX_QUEUE; q++) {
		if (mdp->tx_ring[q]) {
			ringsize = sizeof(struct ravb_txdesc);
			ringsize *= mdp->num_tx_ring[q] + 1;
			dma_free_coherent(NULL,
					ringsize, mdp->tx_ring[q],
					mdp->tx_desc_dma[q]);
			mdp->tx_ring[q] = NULL;
		}
	}
}

/* E-MAC init function */
static int ravb_mac_init(struct net_device *ndev, bool start)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	u32 val;

	/* Recv frame limit set register */
	ravb_write(ndev, ndev->mtu + ETH_HLEN + VLAN_HLEN + ETH_FCS_LEN,
		     RFLR);

	/* PAUSE Prohibition */
	val = (ravb_read(ndev, ECMR) & ECMR_DM) |
		ECMR_ZPF | (mdp->duplex ? ECMR_DM : 0) | ECMR_TE | ECMR_RE;

	ravb_write(ndev, val, ECMR);

	if (mdp->cd->set_rate)
		mdp->cd->set_rate(ndev);

	/* Set MAC address */
	update_mac_address(ndev);

	/* mask reset */
	if (mdp->cd->apr)
		ravb_write(ndev, APR_AP, APR);
	if (mdp->cd->mpr)
		ravb_write(ndev, MPR_MP, MPR);
	if (mdp->cd->tpauser)
		ravb_write(ndev, TPAUSER_UNLIMITED, TPAUSER);

	/* E-MAC Status Register clear */
	ravb_write(ndev, mdp->cd->ecsr_value, ECSR);

	/* E-MAC Interrupt Enable register */
	if (start) {
		ravb_write(ndev, mdp->cd->ecsipr_value, ECSIPR);
		netif_start_queue(ndev);
	}

	return 0;
}

/* Device init function for Ethernet AVB */
static int ravb_dmac_init(struct net_device *ndev, bool start)
{
	int ret = 0;
	struct ravb_private *mdp = netdev_priv(ndev);

	/* Set CONFIG mode */
	ret = ravb_reset(ndev);
	if (ret)
		return ret;

	/* Descriptor format */
	ravb_ring_format(ndev, RAVB_BE);
	ravb_ring_format(ndev, RAVB_NC);

	/* all ravb int mask disable*/
	ravb_write(ndev, 0, RIC0);
	ravb_write(ndev, 0, RIC1);
	ravb_write(ndev, 0, RIC2);
	ravb_write(ndev, 0, TIC);

#if defined(__LITTLE_ENDIAN)
	if (mdp->cd->hw_swap)
		ravb_write(ndev,
			ravb_read(ndev, CCC) & ~CCC_BOC, CCC);
	else
#endif
		ravb_write(ndev,
			ravb_read(ndev, CCC) | CCC_BOC, CCC);

	/* AVB rx set */
	ravb_write(ndev, 0x18000013, RCR);

	/* FIFO size set */
	ravb_write(ndev, 0x00222210, TGC);

	/* Timestamp Enable */
	ravb_write(ndev, 0x00000100, TCCR);

	/* Interrupt Enable */
	if (start) {
		/* Frame Receive */
		ravb_write(ndev, 0x00000003, RIC0);
		/* Receive FIFO full warning */
		ravb_write(ndev, 0x80000000, RIC1);
		/* Receive FIFO full error, Descriptor Empty */
		ravb_write(ndev, 0x80000003, RIC2);
		/* Frame Transmited, Timestamp FIFO updated */
		ravb_write(ndev, 0x00000103, TIC);
	}

	if (start) {
		/* Setting the control will start the AVB-DMAC process. */
		ravb_write(ndev,
			(ravb_read(ndev, CCC) & ~CCC_OPC) | 0x02, CCC);
		netif_start_queue(ndev);
	}

	return ret;
}

/* free Tx skb function for AVB-IP */
static int ravb_txfree(struct net_device *ndev, int q)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_txdesc *desc;
	int free_num = 0;
	int entry = 0;
	struct net_device_stats *stats = &mdp->stats[q];

	for (; mdp->cur_tx[q] - mdp->dirty_tx[q] > 0; mdp->dirty_tx[q]++) {
		entry = mdp->dirty_tx[q] % mdp->num_tx_ring[q];
		desc = &mdp->tx_ring[q][entry];
		if (desc->dt != DT_FEMPTY)
			break;

		/* Free the original skb. */
		if (mdp->tx_skbuff[q][entry]) {
			dma_unmap_single(&ndev->dev,
				desc->dptr, desc->ds, DMA_TO_DEVICE);
			dev_kfree_skb_any(mdp->tx_skbuff[q][entry]);
			mdp->tx_skbuff[q][entry] = NULL;
			free_num++;
		}
		stats->tx_packets++;
		stats->tx_bytes += desc->ds;
		desc->dt = DT_EEMPTY;
	}
	return free_num;
}

static int ravb_get_txtstamp(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct sk_buff *skb;
	struct ravb_tstamp_skb *ts_skb, *ts_skb1;
	struct skb_shared_hwtstamps shhwtstamps;
	u16 tag, tfa_tag;
	u32 tfa2;
	struct timespec ts;
	int ret = 0;
	int count = 0;

	count = (ravb_read(ndev, TSR) & 0x00000700) >> 8;
	while (count--) {
		tfa2 = ravb_read(ndev, TFA2);
		tfa_tag = ((tfa2 & 0x03ff0000)>>16);
		ts.tv_nsec = (u64)ravb_read(ndev, TFA0);
		ts.tv_sec = (((u64)(tfa2 & 0x0000ffff)<<32))
						| ravb_read(ndev, TFA1);
		memset(&shhwtstamps, 0, sizeof(shhwtstamps));
		shhwtstamps.hwtstamp = timespec_to_ktime(ts);
		list_for_each_entry_safe(ts_skb,
					 ts_skb1, &mdp->ts_skb_head, list) {
			skb = ts_skb->skb;
			tag = ts_skb->tag;
			list_del(&ts_skb->list);
			kfree(ts_skb);
			if (tag == tfa_tag) {
				skb_tstamp_tx(skb, &shhwtstamps);
				break;
			}
		}
		ravb_write(ndev, ravb_read(ndev, TCCR) | TCCR_TFR, TCCR);
	}
	return ret;
}

/* Packet receive function for Ethernet AVB */
static int ravb_rx(struct net_device *ndev, u32 ris0, int *quota, int q)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_ex_rxdesc *desc;
	struct timespec ts;

	int entry = mdp->cur_rx[q] % mdp->num_rx_ring[q];
	int boguscnt = (mdp->dirty_rx[q] + mdp->num_rx_ring[q])
			- mdp->cur_rx[q];
	int limit = boguscnt;
	struct sk_buff *skb;
	u16 pkt_len = 0;
	u32 desc_status;
	struct net_device_stats *stats = &mdp->stats[q];
	int skbuff_size = mdp->rx_buf_sz + RAVB_ALIGN - 1;

	if (quota)
		limit = boguscnt = min(boguscnt, *quota);
	desc = &mdp->rx_ring[q][entry];
	while (desc->dt != DT_FEMPTY) {
		desc_status = desc->msc;
		pkt_len = desc->ds;

		if (--boguscnt < 0)
			break;

		if (desc_status & MSC_MC)
			stats->multicast++;

		if (desc_status & (MSC_CRC | MSC_RFE | MSC_RTLF |
				   MSC_RTSF | MSC_CEEF)) {
			stats->rx_errors++;
			if (desc_status & MSC_CRC)
				stats->rx_crc_errors++;
			if (desc_status & MSC_RFE)
				stats->rx_frame_errors++;
			if (desc_status & MSC_RTLF)
				stats->rx_length_errors++;
			if (desc_status & MSC_RTSF)
				stats->rx_length_errors++;
			if (desc_status & MSC_CEEF)
				stats->rx_missed_errors++;
		} else {
			u32 get_ts = mdp->tstamp_rx_ctrl &
					RAVB_RXTSTAMP_TYPE_MASK;
			if (!mdp->cd->hw_swap)
				ravb_soft_swap(
					phys_to_virt(ALIGN(desc->dptr, 4)),
					pkt_len + 2);
			skb = mdp->rx_skbuff[q][entry];
			mdp->rx_skbuff[q][entry] = NULL;
			dma_sync_single_for_cpu(&ndev->dev, desc->dptr,
						ALIGN(mdp->rx_buf_sz, 16),
						DMA_FROM_DEVICE);
			get_ts &= (q == RAVB_NC)
					? RAVB_RXTSTAMP_TYPE_V2_L2_EVENT
					: ~RAVB_RXTSTAMP_TYPE_V2_L2_EVENT;
			if (get_ts) {
				struct skb_shared_hwtstamps *shhwtstamps;
				shhwtstamps = skb_hwtstamps(skb);
				memset(shhwtstamps, 0, sizeof(*shhwtstamps));
				ts.tv_sec = ((u64)desc->ts_sh << 32) |
					    desc->ts_sl;
				ts.tv_nsec = (u64)desc->ts_n;
				shhwtstamps->hwtstamp = timespec_to_ktime(ts);
			}
			skb_put(skb, pkt_len);
			skb->protocol = eth_type_trans(skb, ndev);
			if (q == RAVB_NC)
				netif_rx(skb);
			else
				netif_receive_skb(skb);
			stats->rx_packets++;
			stats->rx_bytes += pkt_len;
		}

		entry = (++mdp->cur_rx[q]) % mdp->num_rx_ring[q];
		desc = &mdp->rx_ring[q][entry];
	}

	/* Refill the Rx ring buffers. */
	for (; mdp->cur_rx[q] - mdp->dirty_rx[q] > 0;
	     mdp->dirty_rx[q]++) {
		entry = mdp->dirty_rx[q] % mdp->num_rx_ring[q];
		desc = &mdp->rx_ring[q][entry];
		/* The size of the buffer is 16 byte boundary. */
		desc->ds = ALIGN(mdp->rx_buf_sz, 16);

		if (mdp->rx_skbuff[q][entry] == NULL) {
			skb = netdev_alloc_skb(ndev, skbuff_size);
			mdp->rx_skbuff[q][entry] = skb;
			if (skb == NULL)
				break;	/* Better luck next round. */
			ravb_set_buff_align(skb);
			dma_unmap_single(&ndev->dev, desc->dptr,
					 desc->ds, DMA_FROM_DEVICE);
			dma_map_single(&ndev->dev, skb->data, desc->ds,
				       DMA_FROM_DEVICE);

			skb_checksum_none_assert(skb);
			desc->dptr = virt_to_phys(skb->data);
			if (dma_mapping_error(&ndev->dev, desc->dptr)) {
				dev_kfree_skb_any(mdp->rx_skbuff[q][entry]);
				mdp->rx_skbuff[q][entry] = NULL;
				break;
			}
		}
		desc->dt = DT_FEMPTY;
	}

	if (quota)
		*quota -= limit - (++boguscnt);

	return (boguscnt <= 0);
}

static void ravb_rcv_snd_disable(struct net_device *ndev)
{
	/* disable tx and rx */
	ravb_write(ndev, ravb_read(ndev, ECMR) &
		~(ECMR_RE | ECMR_TE), ECMR);
}

static void ravb_rcv_snd_enable(struct net_device *ndev)
{
	/* enable tx and rx */
	ravb_write(ndev, ravb_read(ndev, ECMR) |
		(ECMR_RE | ECMR_TE), ECMR);
}

/* error control function */
static void ravb_error(struct net_device *ndev, int intr_status)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	u32 gelic_stat;
	u32 link_stat;
	u32 eis, ris2;

	if (intr_status & ISS_MS) {
		gelic_stat = ravb_read(ndev, ECSR);
		ravb_write(ndev, gelic_stat, ECSR);	/* clear int */
		if (gelic_stat & ECSR_ICD)
			ndev->stats.tx_carrier_errors++;
		if (gelic_stat & ECSR_LCHNG) {
			/* Link Changed */
			if (mdp->cd->no_psr || mdp->no_ether_link) {
				goto ignore_link;
			} else {
				link_stat = (ravb_read(ndev, PSR));
				if (mdp->ether_link_active_low)
					link_stat = ~link_stat;
			}
			if (!(link_stat & PHY_ST_LINK)) {
				ravb_rcv_snd_disable(ndev);
			} else {
				/* clear int */
				ravb_write(ndev, ravb_read(ndev, ECSR),
					  ECSR);
				/* enable tx and rx */
				ravb_rcv_snd_enable(ndev);
			}
		}
	}

ignore_link:
	if (intr_status & ISS_ES) {
		eis = ravb_read(ndev, EIS);
		ravb_write(ndev, ~EIS_QFS, EIS);
		if (eis & EIS_QFS) {
			ris2 = ravb_read(ndev, RIS2);
			ravb_write(ndev, ~RIS2_CHECK, RIS2);

			/* Receive Descriptor Empty int */
			if (ris2 & RIS2_QFF0)
				mdp->stats[RAVB_BE].rx_over_errors++;

			/* Receive Descriptor Empty int */
			if (ris2 & RIS2_QFF1)
				mdp->stats[RAVB_NC].rx_over_errors++;

			/* Receive FIFO Overflow int */
			if (ris2 & RIS2_RFFF)
				mdp->rx_fifo_errors++;
		}
	}
}

static irqreturn_t ravb_interrupt(int irq, void *netdev)
{
	struct net_device *ndev = netdev;
	struct ravb_private *mdp = netdev_priv(ndev);
	irqreturn_t ret = IRQ_NONE;
	unsigned long intr_status;

	spin_lock(&mdp->lock);
	/* Get interrupt status */
	intr_status = ravb_read(ndev, ISS);
	if (!(intr_status & (ISS_FRS | ISS_FTS | ISS_ES |
			ISS_MS | ISS_TFUS)))
		goto other_irq;

	/* Received and Transmited interrupts */
	if (intr_status & (ISS_FRS
			| ISS_TFUS | ISS_FTS)) {
		unsigned long ris0, ric0, tic, tis;
		ris0 = ravb_read(ndev, RIS0);
		ric0 = ravb_read(ndev, RIC0);
		tis = ravb_read(ndev, TIS);
		tic = ravb_read(ndev, TIC);
		ravb_write(ndev,
				~(TIS_TFUF | TIS_FTF1), TIS);

		/* Received Network Control Queue */
		if (ris0 & RIS0_FRF1) {
			ravb_write(ndev, ~RIS0_FRF1, RIS0);
			/* Timestamp of Network Control packets, that is based
			 * on IEEE802.1AS, is used time synchronization of PTP.
			 * it should not be handled by napi scheduling, because
			 * it needs to be received as soon as possible.
			 */
			ravb_rx(ndev, ris0, NULL, RAVB_NC);
			ret = IRQ_HANDLED;
		}

		/* Timestamp updated */
		if (tis & TIS_TFUF) {
			ravb_get_txtstamp(ndev);
			ret = IRQ_HANDLED;
		}

		/* Transmited Network Control Queue */
		if (tis & TIS_FTF1) {
			ravb_txfree(ndev, RAVB_NC);
			netif_wake_queue(ndev);
			ret = IRQ_HANDLED;
		}

		/* Received and Transmited Best Effort Queue */
		if (((ris0 & ric0) & RIS0_FRF0) ||
			((tis & tic) & TIS_FTF0)) {
			if (napi_schedule_prep(&mdp->napi)) {
				/* Mask Rx and Tx interrupts */
				ravb_write(ndev, ric0 & ~RIC0_FRE0,
					RIC0);
				ravb_write(ndev, tic & ~TIC_FTE0,
					TIC);
				__napi_schedule(&mdp->napi);
			} else {
				netdev_warn(ndev,
					"ignoring interrupt, rx status 0x%08lx,"
					" rx mask 0x%08lx,\n"
					"                    tx status 0x%08lx,"
					" tx mask 0x%08lx.\n",
					ris0, ric0, tis, tic);
			}
			ret = IRQ_HANDLED;
		}
	}

	/* Error & MAC status Summary */
	if (intr_status & (ISS_ES | ISS_MS)) {
		ravb_error(ndev, intr_status);
		ret = IRQ_HANDLED;
	}

other_irq:
	spin_unlock(&mdp->lock);

	return ret;
}

static int ravb_poll(struct napi_struct *napi, int budget)
{
	struct ravb_private *mdp = container_of(napi, struct ravb_private,
						  napi);
	struct net_device *ndev = napi->dev;
	int quota = budget;
	unsigned long ris0, ric0, tis, tic;
	unsigned long flags;

	for (;;) {
		tis = ravb_read(ndev, TIS);
		ris0 = ravb_read(ndev, RIS0);
		if (!((ris0 & RIS0_FRF0) || (tis & TIS_FTF0)))
			break;

		/* Processing Rx Descriptor Ring */
		if ((ris0 & RIS0_FRF0)) {
			/* Clear Rx interrupt */
			ravb_write(ndev, ~RIS0_FRF0, RIS0);
			if (ravb_rx(ndev, ris0, &quota, RAVB_BE))
				goto out;
		}
		/* Processing Tx Descriptor Ring */
		if (tis & TIS_FTF0) {
			/* Clear Tx interrupt */
			ravb_write(ndev, ~TIS_FTF0, TIS);
			spin_lock_irqsave(&mdp->lock, flags);
			ravb_txfree(ndev, RAVB_BE);
			if (netif_queue_stopped(ndev))
				netif_wake_queue(ndev);
			spin_unlock_irqrestore(&mdp->lock, flags);
		}
	}

	napi_complete(napi);

	/* Reenable Rx interrupts */
	spin_lock_irqsave(&mdp->lock, flags);
	ric0 = ravb_read(ndev, RIC0);
	ravb_write(ndev, ric0 | RIC0_FRE0, RIC0);
	tic = ravb_read(ndev, TIC);
	ravb_write(ndev, tic | TIC_FTE0, TIC);
	spin_unlock_irqrestore(&mdp->lock, flags);

	/* Receive error message handling */
	mdp->rx_over_errors = mdp->stats[RAVB_BE].rx_over_errors;
	mdp->rx_over_errors += mdp->stats[RAVB_NC].rx_over_errors;
	if (mdp->rx_over_errors != ndev->stats.rx_over_errors) {
		ndev->stats.rx_over_errors = mdp->rx_over_errors;
		netif_err(mdp, rx_err, ndev, "Receive Descriptor Empty\n");
	}
	if (mdp->rx_fifo_errors != ndev->stats.rx_fifo_errors) {
		ndev->stats.rx_fifo_errors = mdp->rx_fifo_errors;
		netif_err(mdp, rx_err, ndev, "Receive FIFO Overflow\n");
	}
out:
	return budget - quota;
}

/* PHY state control function */
static void ravb_adjust_link(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct phy_device *phydev = mdp->phydev;
	int new_state = 0;

	if (phydev->irq > 0)
		phy_read_status(phydev);
	if (phydev->link) {
		if (phydev->duplex != mdp->duplex) {
			new_state = 1;
			mdp->duplex = phydev->duplex;
			if (mdp->cd->set_duplex)
				mdp->cd->set_duplex(ndev);
		}

		if (phydev->speed != mdp->speed) {
			new_state = 1;
			mdp->speed = phydev->speed;
			if (mdp->cd->set_rate)
				mdp->cd->set_rate(ndev);
		}
		if (!mdp->link) {
			ravb_write(ndev,
				     ravb_read(ndev, ECMR) & ~ECMR_TXF,
				     ECMR);
			new_state = 1;
			mdp->link = phydev->link;
			if (mdp->cd->no_psr || mdp->no_ether_link)
				ravb_rcv_snd_enable(ndev);
		}
	} else if (mdp->link) {
		new_state = 1;
		mdp->link = 0;
		mdp->speed = 0;
		mdp->duplex = -1;
		if (mdp->cd->no_psr || mdp->no_ether_link)
			ravb_rcv_snd_disable(ndev);
	}

	if (new_state && netif_msg_link(mdp))
		phy_print_status(phydev);
}

/* PHY configuration for EEB-xxPHY and TSE-xxPHY board */
#define PHY_ID_MASK		0xfffffff0
#define PHY_ID_VSC8211		0x000fc4b0
#define PHY_ID_BCM54810		0x03625d00
#define PHY_ID_BCM89810		0x03625cc0
static void ravb_phy_hack(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	unsigned long phy_id;
	int ret, i, gpio;

	phy_id = (phy_read(mdp->phydev, 0x2) << 16) |
		 phy_read(mdp->phydev, 0x3);
	switch (phy_id & PHY_ID_MASK) {
	case PHY_ID_VSC8211:
		/* Modify CMODE */
		ret = phy_read(mdp->phydev, 0x17);
		ret &= ~0xf006;
		ret |= 0x2002;
		phy_write(mdp->phydev, 0x17, ret);
		ret = phy_read(mdp->phydev, 0x0);
		ret |= 0x8000;
		phy_write(mdp->phydev, 0x0, ret);
		ret = phy_read(mdp->phydev, 0x0);
		ret &= ~0x8000;
		phy_write(mdp->phydev, 0x0, ret);
		break;
	case PHY_ID_BCM54810:
	case PHY_ID_BCM89810:
		/* Unused pins must be GPIO Input */
		for (i = 0; i < mdp->num_phy_ignore_pins; i++) {
			gpio = mdp->phy_ignore_pins[i];
			if (!gpio_is_valid(gpio))
				continue;
			gpio_request_one(gpio,
					 GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED,
					 "AVB_mii-lite-ignore");
		}
		/* {TSE,EEB}-BRPHY board's PHY_IRQ is fixed ACTIVE_HIGH */
		if (mdp->phy_irq)
			irq_set_irq_type(mdp->phy_irq, IRQ_TYPE_LEVEL_HIGH);
		break;
	default:
		break;
	}
}

/* PHY init function */
static int ravb_phy_init(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct phy_device *phydev = NULL;
	char phy_id[MII_BUS_ID_SIZE + 3];

	mdp->link = 0;
	mdp->speed = 0;
	mdp->duplex = -1;

	/* Try connect to PHY */

	snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT,
		 mdp->mii_bus->id, mdp->phy_id);

	phydev = phy_connect(ndev, phy_id, ravb_adjust_link,
			     mdp->phy_interface);

	if (IS_ERR(phydev)) {
		netdev_err(ndev, "failed to connect PHY\n");
		return PTR_ERR(phydev);
	}

	netdev_info(ndev, "attached PHY %d (IRQ %d) to driver %s\n",
		    phydev->addr, phydev->irq, phydev->drv->name);

	mdp->phydev = phydev;

	return 0;
}

/* PHY control start function */
static int ravb_phy_start(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int ret;

	ret = ravb_phy_init(ndev);
	if (ret)
		return ret;

	ravb_phy_hack(ndev);

	phy_start(mdp->phydev);

	return 0;
}

static int ravb_get_settings(struct net_device *ndev,
			       struct ethtool_cmd *ecmd)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mdp->lock, flags);
	ret = phy_ethtool_gset(mdp->phydev, ecmd);
	spin_unlock_irqrestore(&mdp->lock, flags);

	return ret;
}

static int ravb_set_settings(struct net_device *ndev,
			       struct ethtool_cmd *ecmd)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mdp->lock, flags);

	/* disable tx and rx */
	ravb_rcv_snd_disable(ndev);

	ret = phy_ethtool_sset(mdp->phydev, ecmd);
	if (ret)
		goto error_exit;

	if (ecmd->duplex == DUPLEX_FULL)
		mdp->duplex = 1;
	else
		mdp->duplex = 0;

	if (mdp->cd->set_duplex)
		mdp->cd->set_duplex(ndev);

error_exit:
	mdelay(1);

	/* enable tx and rx */
	ravb_rcv_snd_enable(ndev);

	spin_unlock_irqrestore(&mdp->lock, flags);

	return ret;
}

static int ravb_nway_reset(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mdp->lock, flags);
	ret = phy_start_aneg(mdp->phydev);
	spin_unlock_irqrestore(&mdp->lock, flags);

	return ret;
}

static u32 ravb_get_msglevel(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	return mdp->msg_enable;
}

static void ravb_set_msglevel(struct net_device *ndev, u32 value)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	mdp->msg_enable = value;
}

static const char ravb_gstrings_stats[][ETH_GSTRING_LEN] = {
	"rx_queue_0_current",
	"tx_queue_0_current",
	"rx_queue_0_dirty",
	"tx_queue_0_dirty",
	"rx_queue_0_packets",
	"tx_queue_0_packets",
	"rx_queue_0_bytes",
	"tx_queue_0_bytes",
	"rx_queue_0_mcast_packets",
	"rx_queue_0_errors",
	"rx_queue_0_crc_errors",
	"rx_queue_0_frame_errors",
	"rx_queue_0_length_errors",
	"rx_queue_0_missed_errors",
	"rx_queue_0_over_errors",

	"rx_queue_1_current",
	"tx_queue_1_current",
	"rx_queue_1_dirty",
	"tx_queue_1_dirty",
	"rx_queue_1_packets",
	"tx_queue_1_packets",
	"rx_queue_1_bytes",
	"tx_queue_1_bytes",
	"rx_queue_1_mcast_packets",
	"rx_queue_1_errors",
	"rx_queue_1_crc_errors",
	"rx_queue_1_frame_errors_",
	"rx_queue_1_length_errors",
	"rx_queue_1_missed_errors",
	"rx_queue_1_over_errors",
};
#define RAVB_STATS_LEN  ARRAY_SIZE(ravb_gstrings_stats)

static int ravb_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return RAVB_STATS_LEN;
	default:
		return -EOPNOTSUPP;
	}
}

static void ravb_get_ethtool_stats(struct net_device *ndev,
			struct ethtool_stats *stats, u64 *data)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int i = 0;
	int q;

	/* device-specific stats */
	for (q = RAVB_BE; q < NUM_RX_QUEUE; q++) {
		struct net_device_stats *stats = &mdp->stats[q];
		data[i++] = mdp->cur_rx[q];
		data[i++] = mdp->cur_tx[q];
		data[i++] = mdp->dirty_rx[q];
		data[i++] = mdp->dirty_tx[q];
		data[i++] = stats->rx_packets;
		data[i++] = stats->tx_packets;
		data[i++] = stats->rx_bytes;
		data[i++] = stats->tx_bytes;
		data[i++] = stats->multicast;
		data[i++] = stats->rx_errors;
		data[i++] = stats->rx_crc_errors;
		data[i++] = stats->rx_frame_errors;
		data[i++] = stats->rx_length_errors;
		data[i++] = stats->rx_missed_errors;
		data[i++] = stats->rx_over_errors;
	}
}

static void ravb_get_strings(struct net_device *ndev, u32 stringset,
				   u8 *data)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(data, *ravb_gstrings_stats,
					sizeof(ravb_gstrings_stats));
		break;
	}
}

static void ravb_get_ringparam(struct net_device *ndev,
				 struct ethtool_ringparam *ring)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	ring->rx_max_pending = BE_RX_RING_MAX;
	ring->tx_max_pending = BE_TX_RING_MAX;
	ring->rx_pending = mdp->num_rx_ring[RAVB_BE];
	ring->tx_pending = mdp->num_tx_ring[RAVB_BE];
}

static int ravb_set_ringparam(struct net_device *ndev,
				struct ethtool_ringparam *ring)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int ret;

	if (ring->tx_pending > BE_TX_RING_MAX ||
	    ring->rx_pending > BE_RX_RING_MAX ||
	    ring->tx_pending < BE_TX_RING_MIN ||
	    ring->rx_pending < BE_RX_RING_MIN)
		return -EINVAL;
	if (ring->rx_mini_pending || ring->rx_jumbo_pending)
		return -EINVAL;

	if (netif_running(ndev)) {
		netif_tx_disable(ndev);
		ravb_wait_status(ndev, TCCR,
				TCCR_TSRQ0 | TCCR_TSRQ1 |
				TCCR_TSRQ2 | TCCR_TSRQ3);

		ravb_wait_status(ndev, CSR,
				CSR_TPO0 | CSR_TPO1 |
				CSR_TPO2 | CSR_TPO3);

		ravb_write(ndev,
			ravb_read(ndev, ECMR) & ~ECMR_RE, ECMR);

		ravb_wait_status(ndev, CSR, CSR_RPO);

		ravb_write(ndev,
			(ravb_read(ndev, CCC) & ~CCC_OPC) | 0x1, CCC);

		ret = ravb_check_reset(ndev);
		if (ret < 0) {
				netdev_err(ndev,
					   "Cannot reset ringparam! any AVB"
					   " processes are still running.\n");
			return ret;
		}
		synchronize_irq(ndev->irq);
	}

	/* Free all the skbuffs in the Rx queue. */
	ravb_ring_free(ndev, RAVB_BE);
	ravb_ring_free(ndev, RAVB_NC);
	/* Free DMA buffer */
	ravb_free_dma_buffer(mdp);

	/* Set new parameters */
	mdp->num_rx_ring[RAVB_BE] = ring->rx_pending;
	mdp->num_tx_ring[RAVB_BE] = ring->tx_pending;
	mdp->num_rx_ring[RAVB_NC] = NC_RX_RING_SIZE;
	mdp->num_tx_ring[RAVB_NC] = NC_TX_RING_SIZE;

	ret = ravb_ring_init(ndev, RAVB_BE);
	if (ret < 0) {
		netdev_err(ndev, "%s: ravb_ring_init(AVB_BE) failed.\n",
			   __func__);
		return ret;
	}

	ret = ravb_ring_init(ndev, RAVB_NC);
	if (ret < 0) {
		netdev_err(ndev, "%s: ravb_ring_init(AVB_NC) failed.\n",
			   __func__);
		return ret;
	}

	ret = ravb_dmac_init(ndev, false);
	if (ret < 0) {
		netdev_err(ndev, "%s: ravb_dmac_init failed.\n",
			   __func__);
		return ret;
	}
	ravb_mac_init(ndev, false);

	if (netif_running(ndev)) {
		ravb_write(ndev,
			(ravb_read(ndev, CCC) & ~CCC_OPC) | 0x2, CCC);
		netif_wake_queue(ndev);
	}

	return 0;
}

static int ravb_get_ts_info(struct net_device *ndev,
			    struct ethtool_ts_info *info)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_ptp *ptp = mdp->ptp;

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON);
	info->rx_filters =
		(1 << HWTSTAMP_FILTER_NONE) |
		(1 << HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
		(1 << HWTSTAMP_FILTER_ALL);
	if (ptp)
		info->phc_index = ptp_clock_index(ptp->clock);
	else
		info->phc_index = -1;

	return 0;
}

static const struct ethtool_ops ravb_ethtool_ops = {
	.get_settings	= ravb_get_settings,
	.set_settings	= ravb_set_settings,
	.nway_reset	= ravb_nway_reset,
	.get_msglevel	= ravb_get_msglevel,
	.set_msglevel	= ravb_set_msglevel,
	.get_link	= ethtool_op_get_link,
	.get_strings	= ravb_get_strings,
	.get_ethtool_stats  = ravb_get_ethtool_stats,
	.get_sset_count     = ravb_get_sset_count,
	.get_ringparam	= ravb_get_ringparam,
	.set_ringparam	= ravb_set_ringparam,
	.get_ts_info	= ravb_get_ts_info,
};


/* network device open function for Ethernet AVB */
static int ravb_open(struct net_device *ndev)
{
	int ret = 0;
	struct ravb_private *mdp = netdev_priv(ndev);

	napi_enable(&mdp->napi);

	ret = request_irq(ndev->irq, ravb_interrupt,
			mdp->cd->irq_flags, ndev->name, ndev);
	if (ret) {
		netdev_err(ndev, "Can not assign IRQ number\n");
		goto out_napi_off;
	}

	/* Descriptor set */
	/* +26 gets the maximum ethernet encapsulation, +7 & ~7 because the
	 * card needs room to do 8 byte alignment, +2 so we can reserve
	 * the first 2 bytes, and +16 gets room for the status word from the
	 * card.
	 */
	mdp->rx_buf_sz = (ndev->mtu <= 1492 ? PKT_BUF_SZ :
			  (((ndev->mtu + 26 + 7) & ~7) + 2 + 16));

	ret = ravb_ring_init(ndev, RAVB_BE);
	if (ret)
		goto out_free_irq;
	ret = ravb_ring_init(ndev, RAVB_NC);
	if (ret)
		goto out_free_irq;

	/* device init */
	ret = ravb_dmac_init(ndev, true);
	if (ret)
		goto out_free_irq;
	ret = ravb_mac_init(ndev, true);
	if (ret)
		goto out_free_irq;

	/* PHY control start*/
	ret = ravb_phy_start(ndev);
	if (ret)
		goto out_free_irq;

	return ret;

out_free_irq:
	free_irq(ndev->irq, ndev);
out_napi_off:
	napi_disable(&mdp->napi);
	return ret;
}

/* Timeout function for Ethernet AVB */
static void ravb_tx_timeout(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int i, q;

	netif_stop_queue(ndev);

	netif_err(mdp, tx_err, ndev,
		  "transmit timed out, status %8.8x, resetting...\n",
		  (int)ravb_read(ndev, ISS));

	/* tx_errors count up */
	ndev->stats.tx_errors++;

	/* Free all the skbuffs */
	for (q = RAVB_BE; q < NUM_RX_QUEUE; q++) {
		for (i = 0; i < mdp->num_rx_ring[q]; i++) {
			if (mdp->rx_skbuff[q][i])
				dev_kfree_skb(mdp->rx_skbuff[q][i]);
			mdp->rx_skbuff[q][i] = NULL;
		}
	}
	for (q = RAVB_BE; q < NUM_TX_QUEUE; q++) {
		for (i = 0; i < mdp->num_tx_ring[q]; i++) {
			if (mdp->tx_skbuff[q][i])
				dev_kfree_skb(mdp->tx_skbuff[q][i]);
			mdp->tx_skbuff[q][i] = NULL;
			if (mdp->tx_skbuff_aligned[q][i])
				dev_kfree_skb(mdp->tx_skbuff_aligned[q][i]);
			mdp->tx_skbuff_aligned[q][i] = NULL;
		}
	}

	/* device init */
	ravb_dmac_init(ndev, true);
	ravb_mac_init(ndev, true);
}

/* Packet transmit function for Ethernet AVB */
static int ravb_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_txdesc *desc;
	struct ravb_tstamp_skb *ts_skb = NULL;
	u32 entry;
	unsigned long flags;
	int q;

	/* if skb needs tx timestamp, it is handled in Network Control Queue */
	q = (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)
		? RAVB_NC
		: RAVB_BE;

	spin_lock_irqsave(&mdp->lock, flags);
	if ((mdp->cur_tx[q] - mdp->dirty_tx[q]) >= (mdp->num_tx_ring[q] - 4)) {
		if (!ravb_txfree(ndev, q)) {
			netif_warn(mdp, tx_queued, ndev, "TxFD exhausted.\n");
			netif_stop_queue(ndev);
			spin_unlock_irqrestore(&mdp->lock, flags);
			return NETDEV_TX_BUSY;
		}
	}
	entry = mdp->cur_tx[q] % mdp->num_tx_ring[q];
	mdp->cur_tx[q]++;
	spin_unlock_irqrestore(&mdp->lock, flags);

	mdp->tx_skbuff[q][entry] = skb;
	desc = &mdp->tx_ring[q][entry];
	/* soft swap. */
	if (!mdp->cd->hw_swap)
		ravb_soft_swap(phys_to_virt(ALIGN(desc->dptr, 4)),
				 skb->len + 2);
	if (skb->len < ETH_ZLEN)
		desc->ds = ETH_ZLEN;
	else
		desc->ds = skb->len;
	if (mdp->cd->need_txalign) {
		struct sk_buff *tx_skb = mdp->tx_skbuff_aligned[q][entry];
		memcpy(tx_skb->data, skb->data, skb->len);
		desc->dptr = dma_map_single(&ndev->dev, tx_skb->data,
					    desc->ds, DMA_TO_DEVICE);
	} else {
		desc->dptr = dma_map_single(&ndev->dev, skb->data,
					    desc->ds, DMA_TO_DEVICE);
	}
	if (dma_mapping_error(&ndev->dev, desc->dptr)) {
		dev_kfree_skb_any(mdp->tx_skbuff[q][entry]);
		mdp->tx_skbuff[q][entry] = NULL;
		goto out;
	}

	/* Tx timestamp required */
	if (q == RAVB_NC) {
		ts_skb = kmalloc(sizeof(struct ravb_tstamp_skb), GFP_ATOMIC);
		if (!ts_skb) {
			netdev_err(ndev,
			 "Cannot allocate skb list element for HW timestamp\n");
			return -ENOMEM;
		}
		ts_skb->skb = skb;
		ts_skb->tag = mdp->ts_skb_tag++;
		mdp->ts_skb_tag %= 0x400;
		list_add_tail(&ts_skb->list, &mdp->ts_skb_head);

		/* TAG and timestamp required flag */
		skb_tx_timestamp(skb);
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		desc->tsr = 1;
		desc->tag = ts_skb->tag;
	}

	/* descriptor type */
	desc->dt = DT_FSINGLE;

	if (!(ravb_read(ndev, TCCR) & (TCCR_TSRQ0 << q))) {
		spin_lock_irqsave(&mdp->lock, flags);
		ravb_write(ndev,
			ravb_read(ndev, TCCR) | (TCCR_TSRQ0 << q), TCCR);
		spin_unlock_irqrestore(&mdp->lock, flags);
	}
out:
	return NETDEV_TX_OK;
}

static struct net_device_stats *ravb_get_stats(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct net_device_stats *nstats, *stats0, *stats1;

	nstats = &ndev->stats;
	stats0 = &mdp->stats[RAVB_BE];
	stats1 = &mdp->stats[RAVB_NC];

	nstats->tx_dropped += ravb_read(ndev, TROCR);
	ravb_write(ndev, 0, TROCR);	/* (write clear) */
	nstats->collisions += ravb_read(ndev, CDCR);
	ravb_write(ndev, 0, CDCR);	/* (write clear) */
	nstats->tx_carrier_errors += ravb_read(ndev, LCCR);
	ravb_write(ndev, 0, LCCR);	/* (write clear) */

	nstats->tx_carrier_errors += ravb_read(ndev, CERCR);
	ravb_write(ndev, 0, CERCR);	/* (write clear) */
	nstats->tx_carrier_errors += ravb_read(ndev, CEECR);
	ravb_write(ndev, 0, CEECR);	/* (write clear) */

	nstats->rx_packets =
		stats0->rx_packets + stats1->rx_packets;
	nstats->tx_packets =
		stats0->tx_packets + stats1->tx_packets;
	nstats->rx_bytes =
		stats0->rx_bytes + stats1->rx_bytes;
	nstats->tx_bytes =
		stats0->tx_bytes + stats1->tx_bytes;
	nstats->multicast =
		stats0->multicast + stats1->multicast;
	nstats->rx_errors =
		stats0->rx_errors + stats1->rx_errors;
	nstats->rx_crc_errors =
		stats0->rx_crc_errors + stats1->rx_crc_errors;
	nstats->rx_frame_errors =
		stats0->rx_frame_errors + stats1->rx_frame_errors;
	nstats->rx_length_errors =
		stats0->rx_length_errors + stats1->rx_length_errors;
	nstats->rx_missed_errors =
		stats0->rx_missed_errors + stats1->rx_missed_errors;
	nstats->rx_over_errors =
		stats0->rx_over_errors + stats1->rx_over_errors;

	return nstats;
}

/* device close function for Ethernet AVB */
static int ravb_close(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_tstamp_skb *ts_skb, *ts_skb1;

	netif_stop_queue(ndev);

	/* Disable interrupts by clearing the interrupt mask. */
	ravb_write(ndev, 0, RIC0);
	ravb_write(ndev, 0, RIC1);
	ravb_write(ndev, 0, RIC2);
	ravb_write(ndev, 0, TIC);

	/* wait stopping the hardware tx process */
	ravb_wait_status(ndev, TCCR,
			TCCR_TSRQ0 | TCCR_TSRQ1 |
			TCCR_TSRQ2 | TCCR_TSRQ3);

	ravb_wait_status(ndev, CSR,
			CSR_TPO0 | CSR_TPO1 |
			CSR_TPO2 | CSR_TPO3);

	/* Stop the E-MAC's Rx processes. */
	ravb_write(ndev, ravb_read(ndev, ECMR) & ~ECMR_RE, ECMR);

	/* wait stopping the rx dma process */
	ravb_wait_status(ndev, CSR, CSR_RPO);

	/* Stop the AVB-DMAC's processes. */
	ravb_write(ndev,
		(ravb_read(ndev, CCC) & ~CCC_OPC) | 0x1, CCC);

	if (ravb_check_reset(ndev) < 0)
		netdev_err(ndev,
		      "Device will be stopped, after HW processes are done.\n");

	/* clear timestamp list element */
	list_for_each_entry_safe(ts_skb, ts_skb1, &mdp->ts_skb_head, list) {
		list_del(&ts_skb->list);
		kfree(ts_skb);
	}

	/* PHY Disconnect */
	if (mdp->phydev) {
		phy_stop(mdp->phydev);
		phy_disconnect(mdp->phydev);
	}

	free_irq(ndev->irq, ndev);

	napi_disable(&mdp->napi);

	/* Free all the skbuffs in the Rx queue. */
	ravb_ring_free(ndev, RAVB_BE);
	ravb_ring_free(ndev, RAVB_NC);

	/* free DMA buffer */
	ravb_free_dma_buffer(mdp);

	return 0;
}

/* control hardware time stamping */
static int ravb_hwtstamp_ioctl(struct net_device *ndev,
				struct ifreq *ifr, int cmd)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct hwtstamp_config config;
	u32 tstamp_tx_ctrl = RAVB_TXTSTAMP_ENABLED;
	u32 tstamp_rx_ctrl = RAVB_RXTSTAMP_ENABLED;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		tstamp_tx_ctrl = 0;
	case HWTSTAMP_TX_ON:
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		tstamp_rx_ctrl = 0;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		tstamp_rx_ctrl |= RAVB_RXTSTAMP_TYPE_V2_L2_EVENT;
		break;
	default:
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		tstamp_rx_ctrl |= RAVB_RXTSTAMP_TYPE_ALL;
	}

	mdp->tstamp_tx_ctrl = tstamp_tx_ctrl;
	mdp->tstamp_rx_ctrl = tstamp_rx_ctrl;

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

/* ioctl to device function */
static int ravb_do_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct phy_device *phydev = mdp->phydev;

	if (!netif_running(ndev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;

	if (cmd == SIOCSHWTSTAMP)
		return ravb_hwtstamp_ioctl(ndev, rq, cmd);

	return phy_mii_ioctl(phydev, rq, cmd);
}

/* MDIO bus release function */
static int sh_mdio_release(struct ravb_private *mdp)
{
	/* unregister mdio bus */
	mdiobus_unregister(mdp->mii_bus);

	/* free bitbang info */
	free_mdio_bitbang(mdp->mii_bus);

	return 0;
}

/* MDIO bus init function */
static int sh_mdio_init(struct ravb_private *mdp,
			struct ravb_plat_data *pd)
{
	int ret, i;
	struct bb_info *bitbang;
	struct platform_device *pdev = mdp->pdev;
	struct device *dev = &mdp->pdev->dev;

	/* create bit control struct for PHY */
	bitbang = devm_kzalloc(dev, sizeof(struct bb_info), GFP_KERNEL);
	if (!bitbang)
		return -ENOMEM;

	/* bitbang init */
	bitbang->addr = mdp->addr + mdp->reg_offset[PIR];
	bitbang->set_gate = pd->set_mdio_gate;
	bitbang->mdi_msk = PIR_MDI;
	bitbang->mdo_msk = PIR_MDO;
	bitbang->mmd_msk = PIR_MMD;
	bitbang->mdc_msk = PIR_MDC;
	bitbang->ctrl.ops = &bb_ops;

	/* MII controller setting */
	mdp->mii_bus = alloc_mdio_bitbang(&bitbang->ctrl);
	if (!mdp->mii_bus)
		return -ENOMEM;

	/* Hook up MII support for ethtool */
	mdp->mii_bus->name = "sh_mii";
	mdp->mii_bus->parent = dev;
	snprintf(mdp->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 pdev->name, pdev->id);

	/* PHY IRQ */
	mdp->mii_bus->irq = devm_kmalloc_array(dev, PHY_MAX_ADDR, sizeof(int),
					       GFP_KERNEL);
	if (!mdp->mii_bus->irq) {
		ret = -ENOMEM;
		goto out_free_bus;
	}

	/* register MDIO bus */
	for (i = 0; i < PHY_MAX_ADDR; i++)
		mdp->mii_bus->irq[i] = PHY_POLL;
	if (pd->phy_irq > 0)
		mdp->mii_bus->irq[pd->phy] = pd->phy_irq;

	ret = mdiobus_register(mdp->mii_bus);

	if (ret)
		goto out_free_bus;

	return 0;

out_free_bus:
	free_mdio_bitbang(mdp->mii_bus);
	return ret;
}

static const u16 *ravb_get_register_offset(int register_type)
{
	const u16 *reg_offset = NULL;

	switch (register_type) {
	case RAVB_REG_RCAR_GEN2:
		reg_offset = ravb_offset_rcar_gen2;
		break;
	default:
		break;
	}

	return reg_offset;
}

static const struct net_device_ops ravb_netdev_ops = {
	.ndo_open		= ravb_open,
	.ndo_stop		= ravb_close,
	.ndo_start_xmit		= ravb_start_xmit,
	.ndo_get_stats		= ravb_get_stats,
	.ndo_tx_timeout		= ravb_tx_timeout,
	.ndo_do_ioctl		= ravb_do_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_change_mtu		= eth_change_mtu,
};

#ifdef CONFIG_OF
static struct ravb_plat_data *ravb_parse_dt(struct device *dev,
					    struct ravb_private *mdp)
{
	struct device_node *np = dev->of_node;
	struct ravb_plat_data *pdata;
	const char *mac_addr;
	int gpio, count, i;
	enum of_gpio_flags flags;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->phy_interface = of_get_phy_mode(np);

	mac_addr = of_get_mac_address(np);
	if (mac_addr)
		memcpy(pdata->mac_addr, mac_addr, ETH_ALEN);

	pdata->no_ether_link =
		of_property_read_bool(np, "renesas,no-ether-link");
	pdata->ether_link_active_low =
		of_property_read_bool(np, "renesas,ether-link-active-low");
	of_property_read_u32(np, "renesas,phy", &pdata->phy);
	of_property_read_u32(np, "renesas,phy_irq", &pdata->phy_irq);
	gpio = of_get_named_gpio_flags(np, "phy-int-gpio", 0, &flags);
	if (gpio_is_valid(gpio)) {
		gpio_request_one(gpio,
				 GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED,
				 "AVB_phy_int");
		pdata->phy_irq = gpio_to_irq(gpio);
		if (flags & OF_GPIO_ACTIVE_LOW)
			irq_set_irq_type(pdata->phy_irq, IRQ_TYPE_LEVEL_LOW);
		else
			irq_set_irq_type(pdata->phy_irq, IRQ_TYPE_LEVEL_HIGH);
	}
	gpio = of_get_named_gpio(np, "phy-reset-gpio", 0);
	if (gpio_is_valid(gpio))
		gpio_request_one(gpio,
				 GPIOF_OUT_INIT_HIGH | GPIOF_EXPORT_DIR_FIXED,
				 "AVB_phy_reset");
	count = of_gpio_named_count(np,
				    "renesas,mii-lite-ignore-pins");
	mdp->num_phy_ignore_pins = count;
	mdp->phy_ignore_pins = devm_kmalloc_array(dev, count, sizeof(int),
						  GFP_KERNEL);
	for (i = 0; i < count; i++) {
		mdp->phy_ignore_pins[i] = of_get_named_gpio(np,
					"renesas,mii-lite-ignore-pins", i);
	}

	return pdata;
}

static const struct of_device_id ravb_match_table[] = {
	{ .compatible = "renesas,gether-r8a7790", .data = &r8a779x_data_giga },
	{ .compatible = "renesas,gether-r8a7794", .data = &r8a779x_data_giga },
	{ }
};
MODULE_DEVICE_TABLE(of, ravb_match_table);
#else
static inline struct ravb_plat_data *ravb_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int ravb_drv_probe(struct platform_device *pdev)
{
	int ret, devno = 0, q;
	struct resource *res;
	struct net_device *ndev = NULL;
	struct ravb_private *mdp = NULL;
	struct ravb_plat_data *pd = pdev->dev.platform_data;
	const struct platform_device_id *id = platform_get_device_id(pdev);

	/* get base addr */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(res == NULL)) {
		dev_err(&pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	ndev = alloc_etherdev(sizeof(struct ravb_private));
	if (!ndev)
		return -ENOMEM;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* The sh Ether-specific entries in the device structure. */
	ndev->base_addr = res->start;
	devno = pdev->id;
	if (devno < 0)
		devno = 0;

	ndev->dma = -1;
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		ret = -ENODEV;
		goto out_release;
	}
	ndev->irq = ret;

	SET_NETDEV_DEV(ndev, &pdev->dev);

	mdp = netdev_priv(ndev);
	mdp->ndev = ndev;
	mdp->num_tx_ring[RAVB_BE] = BE_TX_RING_SIZE;
	mdp->num_rx_ring[RAVB_BE] = BE_RX_RING_SIZE;
	mdp->num_tx_ring[RAVB_NC] = NC_TX_RING_SIZE;
	mdp->num_rx_ring[RAVB_NC] = NC_RX_RING_SIZE;
	mdp->addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mdp->addr)) {
		ret = PTR_ERR(mdp->addr);
		goto out_release;
	}

	spin_lock_init(&mdp->lock);
	mdp->pdev = pdev;

	if (pdev->dev.of_node)
		pd = ravb_parse_dt(&pdev->dev, mdp);
	if (!pd) {
		dev_err(&pdev->dev, "no platform data\n");
		ret = -EINVAL;
		goto out_release;
	}

	/* get PHY ID */
	mdp->phy_id = pd->phy;
	mdp->phy_irq = pd->phy_irq;
	mdp->phy_interface = pd->phy_interface;
	/* EDMAC endian */
	mdp->edmac_endian = pd->edmac_endian;
	mdp->no_ether_link = pd->no_ether_link;
	mdp->ether_link_active_low = pd->ether_link_active_low;

	/* set cpu data */
	if (id) {
		mdp->cd = (struct ravb_cpu_data *)id->driver_data;
	} else	{
		const struct of_device_id *match;

		match = of_match_device(of_match_ptr(ravb_match_table),
					&pdev->dev);
		mdp->cd = (struct ravb_cpu_data *)match->data;
	}
	mdp->reg_offset = ravb_get_register_offset(mdp->cd->register_type);
	if (!mdp->reg_offset) {
		dev_err(&pdev->dev, "Unknown register type (%d)\n",
			mdp->cd->register_type);
		ret = -EINVAL;
		goto out_release;
	}
	ravb_set_default_cpu_data(mdp->cd);

	ndev->netdev_ops = &ravb_netdev_ops;

	mdp->rx_over_errors = 0;
	mdp->rx_fifo_errors = 0;
	for (q = RAVB_BE; q < NUM_RX_QUEUE; q++) {
		struct net_device_stats *stats = &mdp->stats[q];
		stats->rx_packets = 0;
		stats->tx_packets = 0;
		stats->rx_bytes = 0;
		stats->tx_bytes = 0;
		stats->multicast = 0;
		stats->rx_errors = 0;
		stats->rx_crc_errors = 0;
		stats->rx_frame_errors = 0;
		stats->rx_length_errors = 0;
		stats->rx_missed_errors = 0;
		stats->rx_over_errors = 0;
	}

	/* set function */
	ndev->netdev_ops = &ravb_netdev_ops;
	SET_ETHTOOL_OPS(ndev, &ravb_ethtool_ops);

	/* set AVB config mode */
	ravb_write(ndev,
		(ravb_read(ndev, CCC) & ~CCC_OPC) | 0x1, CCC);

	/* set CSEL value*/
	ravb_write(ndev,
		(ravb_read(ndev, CCC) & ~CCC_CSEL) |
		mdp->cd->csel_value, CCC);

	/* set GTI value */
	ravb_write(ndev, mdp->cd->gti_value & 0x0fffffff, GTI);

	/* request GTI loading */
	ravb_write(ndev, ravb_read(ndev, GCCR) | GCCR_LTI, GCCR);

	/* Allocate Descriptor Base Address Table */
	mdp->desc_bat_sz =
		sizeof(struct ravb_desc) * DBAT_ENTRY_NUM;
	mdp->desc_bat = dma_alloc_coherent(NULL, mdp->desc_bat_sz,
			&mdp->desc_bat_dma, GFP_KERNEL);
	if (!mdp->desc_bat) {
		dev_err(&ndev->dev,
			"Cannot allocate Desc Base Address Table"
			" (size %d bytes)\n",
			mdp->desc_bat_sz);
		ret = -ENOMEM;
		goto out_release;
	}
	for (q = RAVB_BE; q < DBAT_ENTRY_NUM; q++)
		mdp->desc_bat[q].dt = DT_EOS;
	ravb_write(ndev, mdp->desc_bat_dma, DBAT);

	/* initialise HW timestamp list */
	INIT_LIST_HEAD(&mdp->ts_skb_head);

	/* initialise PTP Clock driver */
	ravb_ptp_init(ndev, pdev);

	/* debug message level */
	mdp->msg_enable = RAVB_DEF_MSG_ENABLE;

	/* read and set MAC address */
	read_mac_address(ndev, pd->mac_addr);
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		dev_warn(&pdev->dev,
			"no valid MAC address supplied, using a random one.\n");
		eth_hw_addr_random(ndev);
	}

	/* MDIO bus init */
	ret = sh_mdio_init(mdp, pd);
	if (ret) {
		dev_err(&ndev->dev, "failed to initialise MDIO\n");
		goto out_release;
	}

	netif_napi_add(ndev, &mdp->napi, ravb_poll, 64);

	/* network device register */
	ret = register_netdev(ndev);
	if (ret)
		goto out_napi_del;

	/* print device information */
	netdev_info(ndev, "Base address at 0x%x, %pM, IRQ %d.\n",
		    (u32)ndev->base_addr, ndev->dev_addr, ndev->irq);

	platform_set_drvdata(pdev, ndev);

	return ret;

out_napi_del:
	netif_napi_del(&mdp->napi);
	sh_mdio_release(mdp);
	dma_free_coherent(NULL,
			  mdp->desc_bat_sz, mdp->desc_bat, mdp->desc_bat_dma);

out_release:
	/* stop PTP Clock driver */
	ravb_ptp_stop(ndev, pdev);
	/* net_dev free */
	if (ndev)
		free_netdev(ndev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int ravb_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ravb_private *mdp = netdev_priv(ndev);

	/* stop PTP Clock driver */
	ravb_ptp_stop(ndev, pdev);

	dma_free_coherent(NULL, mdp->desc_bat_sz, mdp->desc_bat,
			  mdp->desc_bat_dma);
	/* set RESET mode */
	ravb_write(ndev, 0x0, CCC);
	pm_runtime_put_sync(&pdev->dev);
	unregister_netdev(ndev);
	netif_napi_del(&mdp->napi);
	sh_mdio_release(mdp);
	pm_runtime_disable(&pdev->dev);
	free_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int ravb_runtime_nop(struct device *dev)
{
	/* Runtime PM callback shared between ->runtime_suspend()
	 * and ->runtime_resume(). Simply returns success.
	 *
	 * This driver re-initializes all registers after
	 * pm_runtime_get_sync() anyway so there is no need
	 * to save and restore registers here.
	 */
	return 0;
}

static const struct dev_pm_ops ravb_dev_pm_ops = {
	.runtime_suspend = ravb_runtime_nop,
	.runtime_resume = ravb_runtime_nop,
};
#define RAVB_PM_OPS (&ravb_dev_pm_ops)
#else
#define RAVB_PM_OPS NULL
#endif

static struct platform_device_id ravb_id_table[] = {
	{ "r8a7790-gether", (kernel_ulong_t)&r8a779x_data_giga },
	{ "r8a7794-gether", (kernel_ulong_t)&r8a779x_data_giga },
	{ }
};
MODULE_DEVICE_TABLE(platform, ravb_id_table);

static struct platform_driver ravb_driver = {
	.probe = ravb_drv_probe,
	.remove = ravb_drv_remove,
	.id_table = ravb_id_table,
	.driver = {
		   .name = CARDNAME,
		   .pm = RAVB_PM_OPS,
		   .of_match_table = of_match_ptr(ravb_match_table),
	},
};

module_platform_driver(ravb_driver);

MODULE_AUTHOR("Nobuhiro Iwamatsu, Yoshihiro Shimoda, Mitsuhiro Kimura");
MODULE_DESCRIPTION("Renesas Ethernet AVB driver");
MODULE_LICENSE("GPL v2");
