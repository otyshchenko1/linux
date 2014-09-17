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

#include "ravb.h"

#define RAVB_DEF_MSG_ENABLE \
		(NETIF_MSG_LINK	| \
		NETIF_MSG_TIMER	| \
		NETIF_MSG_RX_ERR| \
		NETIF_MSG_TX_ERR)

static const u16 ravb_offset_fast_rcar[RAVB_MAX_REGISTER_OFFSET] = {
	[ECMR]		= 0x0300,
	[RFLR]		= 0x0308,
	[ECSR]		= 0x0310,
	[ECSIPR]	= 0x0318,
	[PIR]		= 0x0320,
	[PSR]		= 0x0328,
	[RDMLR]		= 0x0340,
	[IPGR]		= 0x0350,
	[APR]		= 0x0354,
	[MPR]		= 0x0358,
	[RFCF]		= 0x0360,
	[TPAUSER]	= 0x0364,
	[TPAUSECR]	= 0x0368,
	[MAHR]		= 0x03c0,
	[MALR]		= 0x03c8,
	[TROCR]		= 0x03d0,
	[CDCR]		= 0x03d4,
	[LCCR]		= 0x03d8,
	[CNDCR]		= 0x03dc,
	[CEFCR]		= 0x03e4,
	[FRECR]		= 0x03e8,
	[TSFRCR]	= 0x03ec,
	[TLFRCR]	= 0x03f0,
	[RFCR]		= 0x03f4,
	[MAFCR]		= 0x03f8,

	[EDMR]		= 0x0200,
	[EDTRR]		= 0x0208,
	[EDRRR]		= 0x0210,
	[TDLAR]		= 0x0218,
	[RDLAR]		= 0x0220,
	[EESR]		= 0x0228,
	[EESIPR]	= 0x0230,
	[TRSCER]	= 0x0238,
	[RMFCR]		= 0x0240,
	[TFTR]		= 0x0248,
	[FDR]		= 0x0250,
	[RMCR]		= 0x0258,
	[TFUCR]		= 0x0264,
	[RFOCR]		= 0x0268,
	[RMIIMODE]      = 0x026c,
	[FCFTR]		= 0x0270,
	[TRIMD]		= 0x027c,
};

static void ravb_select_mii(struct net_device *ndev)
{
	u32 value = 0x0;
	struct ravb_private *mdp = netdev_priv(ndev);

	switch (mdp->phy_interface) {
	case PHY_INTERFACE_MODE_GMII:
		value = 0x2;
		break;
	case PHY_INTERFACE_MODE_MII:
		value = 0x1;
		break;
	case PHY_INTERFACE_MODE_RMII:
		value = 0x0;
		break;
	default:
		netdev_warn(ndev,
			    "PHY interface mode was not setup. Set to MII.\n");
		value = 0x1;
		break;
	}

	ravb_write(ndev, value, RMII_MII);
}

static void ravb_set_duplex(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	if (mdp->duplex) /* Full */
		ravb_write(ndev, ravb_read(ndev, ECMR) | ECMR_DM, ECMR);
	else		/* Half */
		ravb_write(ndev, ravb_read(ndev, ECMR) & ~ECMR_DM, ECMR);
}

/* There is CPU dependent code */
static void ravb_set_rate_r8a777x(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	switch (mdp->speed) {
	case 10: /* 10BASE */
		ravb_write(ndev, ravb_read(ndev, ECMR) & ~ECMR_ELB, ECMR);
		break;
	case 100:/* 100BASE */
		ravb_write(ndev, ravb_read(ndev, ECMR) | ECMR_ELB, ECMR);
		break;
	default:
		break;
	}
}

/* R8A7790/1 */
static struct ravb_cpu_data r8a779x_data = {
	.set_duplex	= ravb_set_duplex,
	.set_rate	= ravb_set_rate_r8a777x,

	.register_type	= RAVB_REG_FAST_RCAR,

	.ecsr_value	= ECSR_PSRTO | ECSR_LCHNG | ECSR_ICD,
	.ecsipr_value	= ECSIPR_PSRTOIP | ECSIPR_LCHNGIP | ECSIPR_ICDIP,
	.eesipr_value	= 0x01ff009f,

	.tx_check	= EESR_FTC | EESR_CND | EESR_DLC | EESR_CD | EESR_RTO,
	.eesr_err_check	= EESR_TWB | EESR_TABT | EESR_RABT | EESR_RFE |
			  EESR_RDE | EESR_RFRMER | EESR_TFE | EESR_TDE |
			  EESR_ECI,

	.apr		= 1,
	.mpr		= 1,
	.tpauser	= 1,
	.hw_swap	= 1,
	.rmiimode	= 1,
	.shift_rd0	= 1,
};

static void ravb_set_default_cpu_data(struct ravb_cpu_data *cd)
{
	if (!cd->ecsr_value)
		cd->ecsr_value = DEFAULT_ECSR_INIT;

	if (!cd->ecsipr_value)
		cd->ecsipr_value = DEFAULT_ECSIPR_INIT;

	if (!cd->fcftr_value)
		cd->fcftr_value = DEFAULT_FIFO_F_D_RFF |
				  DEFAULT_FIFO_F_D_RFD;

	if (!cd->fdr_value)
		cd->fdr_value = DEFAULT_FDR_INIT;

	if (!cd->tx_check)
		cd->tx_check = DEFAULT_TX_CHECK;

	if (!cd->eesr_err_check)
		cd->eesr_err_check = DEFAULT_EESR_ERR_CHECK;
}

static int ravb_check_reset(struct net_device *ndev)
{
	int ret = 0;
	int cnt = 100;

	while (cnt > 0) {
		if (!(ravb_read(ndev, EDMR) & 0x3))
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
	struct ravb_private *mdp = netdev_priv(ndev);
	int ret = 0;

	ravb_write(ndev, EDSR_ENALL, EDSR);
	ravb_write(ndev, ravb_read(ndev, EDMR) | EDMR_SRST_GETHER,
		     EDMR);

	ret = ravb_check_reset(ndev);
	if (ret)
		return ret;

	/* Table Init */
	ravb_write(ndev, 0x0, TDLAR);
	ravb_write(ndev, 0x0, TDFAR);
	ravb_write(ndev, 0x0, TDFXR);
	ravb_write(ndev, 0x0, TDFFR);
	ravb_write(ndev, 0x0, RDLAR);
	ravb_write(ndev, 0x0, RDFAR);
	ravb_write(ndev, 0x0, RDFXR);
	ravb_write(ndev, 0x0, RDFFR);

	/* Reset HW CRC register */
	if (mdp->cd->hw_crc)
		ravb_write(ndev, 0x0, CSMR);

	/* Select MII mode */
	if (mdp->cd->select_mii)
		ravb_select_mii(ndev);

	return ret;
}

static void ravb_set_receive_align(struct sk_buff *skb)
{
	u32 reserve = (u32)skb->data & (RAVB_RX_ALIGN - 1);
	if (reserve)
		skb_reserve(skb, RAVB_RX_ALIGN - reserve);
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

static unsigned long ravb_get_edtrr_trns(struct ravb_private *mdp)
{
	return EDTRR_TRNS_ETHER;
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

/* free skb and descriptor buffer */
static void ravb_ring_free(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int i;

	/* Free Rx skb ringbuffer */
	if (mdp->rx_skbuff) {
		for (i = 0; i < mdp->num_rx_ring; i++) {
			if (mdp->rx_skbuff[i])
				dev_kfree_skb(mdp->rx_skbuff[i]);
		}
	}
	kfree(mdp->rx_skbuff);
	mdp->rx_skbuff = NULL;

	/* Free Tx skb ringbuffer */
	if (mdp->tx_skbuff) {
		for (i = 0; i < mdp->num_tx_ring; i++) {
			if (mdp->tx_skbuff[i])
				dev_kfree_skb(mdp->tx_skbuff[i]);
		}
	}
	kfree(mdp->tx_skbuff);
	mdp->tx_skbuff = NULL;
}

/* format skb and descriptor buffer */
static void ravb_ring_format(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int i;
	struct sk_buff *skb;
	struct ravb_rxdesc *rxdesc = NULL;
	struct ravb_txdesc *txdesc = NULL;
	int rx_ringsize = sizeof(*rxdesc) * mdp->num_rx_ring;
	int tx_ringsize = sizeof(*txdesc) * mdp->num_tx_ring;
	int skbuff_size = mdp->rx_buf_sz + RAVB_RX_ALIGN - 1;

	mdp->cur_rx = 0;
	mdp->cur_tx = 0;
	mdp->dirty_rx = 0;
	mdp->dirty_tx = 0;

	memset(mdp->rx_ring, 0, rx_ringsize);

	/* build Rx ring buffer */
	for (i = 0; i < mdp->num_rx_ring; i++) {
		/* skb */
		mdp->rx_skbuff[i] = NULL;
		skb = netdev_alloc_skb(ndev, skbuff_size);
		mdp->rx_skbuff[i] = skb;
		if (skb == NULL)
			break;
		ravb_set_receive_align(skb);

		/* RX descriptor */
		rxdesc = &mdp->rx_ring[i];
		/* The size of the buffer is 16 byte boundary. */
		rxdesc->buffer_length = ALIGN(mdp->rx_buf_sz, 16);
		dma_map_single(&ndev->dev, skb->data, rxdesc->buffer_length,
			       DMA_FROM_DEVICE);
		rxdesc->addr = virt_to_phys(skb->data);
		if (dma_mapping_error(&ndev->dev, rxdesc->addr)) {
			dev_kfree_skb(mdp->rx_skbuff[i]);
			mdp->rx_skbuff[i] = NULL;
			break;
		}
		rxdesc->status = cpu_to_edmac(mdp, RD_RACT | RD_RFP);

		/* Rx descriptor address set */
		if (i == 0)
			ravb_write(ndev, mdp->rx_desc_dma, RDLAR);
	}

	mdp->dirty_rx = (u32) (i - mdp->num_rx_ring);

	/* Mark the last entry as wrapping the ring. */
	rxdesc->status |= cpu_to_edmac(mdp, RD_RDEL);

	memset(mdp->tx_ring, 0, tx_ringsize);

	/* build Tx ring buffer */
	for (i = 0; i < mdp->num_tx_ring; i++) {
		mdp->tx_skbuff[i] = NULL;
		txdesc = &mdp->tx_ring[i];
		txdesc->status = cpu_to_edmac(mdp, TD_TFP);
		txdesc->buffer_length = 0;
		if (i == 0)
			/* Tx descriptor address set */
			ravb_write(ndev, mdp->tx_desc_dma, TDLAR);
	}

	txdesc->status |= cpu_to_edmac(mdp, TD_TDLE);
}

/* Get skb and descriptor buffer */
static int ravb_ring_init(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int rx_ringsize, tx_ringsize, ret = 0;

	/* +26 gets the maximum ethernet encapsulation, +7 & ~7 because the
	 * card needs room to do 8 byte alignment, +2 so we can reserve
	 * the first 2 bytes, and +16 gets room for the status word from the
	 * card.
	 */
	mdp->rx_buf_sz = (ndev->mtu <= 1492 ? PKT_BUF_SZ :
			  (((ndev->mtu + 26 + 7) & ~7) + 2 + 16));
	if (mdp->cd->rpadir)
		mdp->rx_buf_sz += NET_IP_ALIGN;

	/* Allocate RX and TX skb rings */
	mdp->rx_skbuff = kmalloc_array(mdp->num_rx_ring,
				       sizeof(*mdp->rx_skbuff), GFP_KERNEL);
	if (!mdp->rx_skbuff) {
		ret = -ENOMEM;
		return ret;
	}

	mdp->tx_skbuff = kmalloc_array(mdp->num_tx_ring,
				       sizeof(*mdp->tx_skbuff), GFP_KERNEL);
	if (!mdp->tx_skbuff) {
		ret = -ENOMEM;
		goto skb_ring_free;
	}

	/* Allocate all Rx descriptors. */
	rx_ringsize = sizeof(struct ravb_rxdesc) * mdp->num_rx_ring;
	mdp->rx_ring = dma_alloc_coherent(NULL, rx_ringsize, &mdp->rx_desc_dma,
					  GFP_KERNEL);
	if (!mdp->rx_ring) {
		ret = -ENOMEM;
		goto desc_ring_free;
	}

	mdp->dirty_rx = 0;

	/* Allocate all Tx descriptors. */
	tx_ringsize = sizeof(struct ravb_txdesc) * mdp->num_tx_ring;
	mdp->tx_ring = dma_alloc_coherent(NULL, tx_ringsize, &mdp->tx_desc_dma,
					  GFP_KERNEL);
	if (!mdp->tx_ring) {
		ret = -ENOMEM;
		goto desc_ring_free;
	}
	return ret;

desc_ring_free:
	/* free DMA buffer */
	dma_free_coherent(NULL, rx_ringsize, mdp->rx_ring, mdp->rx_desc_dma);

skb_ring_free:
	/* Free Rx and Tx skb ring buffer */
	ravb_ring_free(ndev);
	mdp->tx_ring = NULL;
	mdp->rx_ring = NULL;

	return ret;
}

static void ravb_free_dma_buffer(struct ravb_private *mdp)
{
	int ringsize;

	if (mdp->rx_ring) {
		ringsize = sizeof(struct ravb_rxdesc) * mdp->num_rx_ring;
		dma_free_coherent(NULL, ringsize, mdp->rx_ring,
				  mdp->rx_desc_dma);
		mdp->rx_ring = NULL;
	}

	if (mdp->tx_ring) {
		ringsize = sizeof(struct ravb_txdesc) * mdp->num_tx_ring;
		dma_free_coherent(NULL, ringsize, mdp->tx_ring,
				  mdp->tx_desc_dma);
		mdp->tx_ring = NULL;
	}
}

static int ravb_dev_init(struct net_device *ndev, bool start)
{
	int ret = 0;
	struct ravb_private *mdp = netdev_priv(ndev);
	u32 val;

	/* Soft Reset */
	ret = ravb_reset(ndev);
	if (ret)
		return ret;

	if (mdp->cd->rmiimode)
		ravb_write(ndev, 0x1, RMIIMODE);

	/* Descriptor format */
	ravb_ring_format(ndev);
	if (mdp->cd->rpadir)
		ravb_write(ndev, mdp->cd->rpadir_value, RPADIR);

	/* all ravb int mask */
	ravb_write(ndev, 0, EESIPR);

#if defined(__LITTLE_ENDIAN)
	if (mdp->cd->hw_swap)
		ravb_write(ndev, EDMR_EL, EDMR);
	else
#endif
		ravb_write(ndev, 0, EDMR);

	/* FIFO size set */
	ravb_write(ndev, mdp->cd->fdr_value, FDR);
	ravb_write(ndev, 0, TFTR);

	/* Frame recv control (enable multiple-packets per rx irq) */
	ravb_write(ndev, RMCR_RNC, RMCR);

	ravb_write(ndev, DESC_I_RINT8 | DESC_I_RINT5 | DESC_I_TINT2, TRSCER);

	if (mdp->cd->bculr)
		ravb_write(ndev, 0x800, BCULR);	/* Burst sycle set */

	ravb_write(ndev, mdp->cd->fcftr_value, FCFTR);

	if (!mdp->cd->no_trimd)
		ravb_write(ndev, 0, TRIMD);

	/* Recv frame limit set register */
	ravb_write(ndev, ndev->mtu + ETH_HLEN + VLAN_HLEN + ETH_FCS_LEN,
		     RFLR);

	ravb_write(ndev, ravb_read(ndev, EESR), EESR);
	if (start)
		ravb_write(ndev, mdp->cd->eesipr_value, EESIPR);

	/* PAUSE Prohibition */
	val = (ravb_read(ndev, ECMR) & ECMR_DM) |
		ECMR_ZPF | (mdp->duplex ? ECMR_DM : 0) | ECMR_TE | ECMR_RE;

	ravb_write(ndev, val, ECMR);

	if (mdp->cd->set_rate)
		mdp->cd->set_rate(ndev);

	/* E-MAC Status Register clear */
	ravb_write(ndev, mdp->cd->ecsr_value, ECSR);

	/* E-MAC Interrupt Enable register */
	if (start)
		ravb_write(ndev, mdp->cd->ecsipr_value, ECSIPR);

	/* Set MAC address */
	update_mac_address(ndev);

	/* mask reset */
	if (mdp->cd->apr)
		ravb_write(ndev, APR_AP, APR);
	if (mdp->cd->mpr)
		ravb_write(ndev, MPR_MP, MPR);
	if (mdp->cd->tpauser)
		ravb_write(ndev, TPAUSER_UNLIMITED, TPAUSER);

	if (start) {
		/* Setting the Rx mode will start the Rx process. */
		ravb_write(ndev, EDRRR_R, EDRRR);

		netif_start_queue(ndev);
	}

	return ret;
}

/* free Tx skb function */
static int ravb_txfree(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_txdesc *txdesc;
	int free_num = 0;
	int entry = 0;

	for (; mdp->cur_tx - mdp->dirty_tx > 0; mdp->dirty_tx++) {
		entry = mdp->dirty_tx % mdp->num_tx_ring;
		txdesc = &mdp->tx_ring[entry];
		if (txdesc->status & cpu_to_edmac(mdp, TD_TACT))
			break;
		/* Free the original skb. */
		if (mdp->tx_skbuff[entry]) {
			dma_unmap_single(&ndev->dev, txdesc->addr,
					 txdesc->buffer_length, DMA_TO_DEVICE);
			dev_kfree_skb_any(mdp->tx_skbuff[entry]);
			mdp->tx_skbuff[entry] = NULL;
			free_num++;
		}
		txdesc->status = cpu_to_edmac(mdp, TD_TFP);
		if (entry >= mdp->num_tx_ring - 1)
			txdesc->status |= cpu_to_edmac(mdp, TD_TDLE);

		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += txdesc->buffer_length;
	}
	return free_num;
}

/* Packet receive function */
static int ravb_rx(struct net_device *ndev, u32 intr_status, int *quota)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_rxdesc *rxdesc;

	int entry = mdp->cur_rx % mdp->num_rx_ring;
	int boguscnt = (mdp->dirty_rx + mdp->num_rx_ring) - mdp->cur_rx;
	int limit = boguscnt;
	struct sk_buff *skb;
	u16 pkt_len = 0;
	u32 desc_status;
	int skbuff_size = mdp->rx_buf_sz + RAVB_RX_ALIGN - 1;

	if (quota)
		limit = boguscnt = min(boguscnt, *quota);
	rxdesc = &mdp->rx_ring[entry];
	while (!(rxdesc->status & cpu_to_edmac(mdp, RD_RACT))) {
		desc_status = edmac_to_cpu(mdp, rxdesc->status);
		pkt_len = rxdesc->frame_length;

		if (--boguscnt < 0)
			break;

		if (!(desc_status & RDFEND))
			ndev->stats.rx_length_errors++;

		/* In case of almost all GETHER/ETHERs, the Receive Frame State
		 * (RFS) bits in the Receive Descriptor 0 are from bit 9 to
		 * bit 0. However, in case of the R8A7740, R8A779x, and
		 * R7S72100 the RFS bits are from bit 25 to bit 16. So, the
		 * driver needs right shifting by 16.
		 */
		if (mdp->cd->shift_rd0)
			desc_status >>= 16;

		if (desc_status & (RD_RFS1 | RD_RFS2 | RD_RFS3 | RD_RFS4 |
				   RD_RFS5 | RD_RFS6 | RD_RFS10)) {
			ndev->stats.rx_errors++;
			if (desc_status & RD_RFS1)
				ndev->stats.rx_crc_errors++;
			if (desc_status & RD_RFS2)
				ndev->stats.rx_frame_errors++;
			if (desc_status & RD_RFS3)
				ndev->stats.rx_length_errors++;
			if (desc_status & RD_RFS4)
				ndev->stats.rx_length_errors++;
			if (desc_status & RD_RFS6)
				ndev->stats.rx_missed_errors++;
			if (desc_status & RD_RFS10)
				ndev->stats.rx_over_errors++;
		} else {
			if (!mdp->cd->hw_swap)
				ravb_soft_swap(
					phys_to_virt(ALIGN(rxdesc->addr, 4)),
					pkt_len + 2);
			skb = mdp->rx_skbuff[entry];
			mdp->rx_skbuff[entry] = NULL;
			if (mdp->cd->rpadir)
				skb_reserve(skb, NET_IP_ALIGN);
			dma_sync_single_for_cpu(&ndev->dev, rxdesc->addr,
					ALIGN(mdp->rx_buf_sz, 16),
					DMA_FROM_DEVICE);
			skb_put(skb, pkt_len);
			skb->protocol = eth_type_trans(skb, ndev);
			netif_receive_skb(skb);
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += pkt_len;
		}
		entry = (++mdp->cur_rx) % mdp->num_rx_ring;
		rxdesc = &mdp->rx_ring[entry];
	}

	/* Refill the Rx ring buffers. */
	for (; mdp->cur_rx - mdp->dirty_rx > 0; mdp->dirty_rx++) {
		entry = mdp->dirty_rx % mdp->num_rx_ring;
		rxdesc = &mdp->rx_ring[entry];
		/* The size of the buffer is 16 byte boundary. */
		rxdesc->buffer_length = ALIGN(mdp->rx_buf_sz, 16);

		if (mdp->rx_skbuff[entry] == NULL) {
			skb = netdev_alloc_skb(ndev, skbuff_size);
			mdp->rx_skbuff[entry] = skb;
			if (skb == NULL)
				break;	/* Better luck next round. */
			ravb_set_receive_align(skb);
			dma_unmap_single(&ndev->dev, rxdesc->addr,
					 rxdesc->buffer_length,
					 DMA_FROM_DEVICE);
			dma_map_single(&ndev->dev, skb->data,
				       rxdesc->buffer_length, DMA_FROM_DEVICE);

			skb_checksum_none_assert(skb);
			rxdesc->addr = virt_to_phys(skb->data);
			if (dma_mapping_error(&ndev->dev, rxdesc->addr)) {
				dev_kfree_skb_any(mdp->rx_skbuff[entry]);
				mdp->rx_skbuff[entry] = NULL;
				break;
			}
		}
		if (entry >= mdp->num_rx_ring - 1)
			rxdesc->status |=
				cpu_to_edmac(mdp, RD_RACT | RD_RFP | RD_RDEL);
		else
			rxdesc->status |=
				cpu_to_edmac(mdp, RD_RACT | RD_RFP);
	}

	/* Restart Rx engine if stopped. */
	/* If we don't need to check status, don't. -KDU */
	if (!(ravb_read(ndev, EDRRR) & EDRRR_R)) {
		/* fix the values for the next receiving if RDE is set */
		if (intr_status & EESR_RDE) {
			u32 count = (ravb_read(ndev, RDFAR) -
				     ravb_read(ndev, RDLAR)) >> 4;

			mdp->cur_rx = count;
			mdp->dirty_rx = count;
		}
		ravb_write(ndev, EDRRR_R, EDRRR);
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
	u32 felic_stat;
	u32 link_stat;
	u32 mask;

	if (intr_status & EESR_ECI) {
		felic_stat = ravb_read(ndev, ECSR);
		ravb_write(ndev, felic_stat, ECSR);	/* clear int */
		if (felic_stat & ECSR_ICD)
			ndev->stats.tx_carrier_errors++;
		if (felic_stat & ECSR_LCHNG) {
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
				/* Link Up */
				ravb_write(ndev, ravb_read(ndev, EESIPR) &
						   ~DMAC_M_ECI, EESIPR);
				/* clear int */
				ravb_write(ndev, ravb_read(ndev, ECSR),
					     ECSR);
				ravb_write(ndev, ravb_read(ndev, EESIPR) |
						   DMAC_M_ECI, EESIPR);
				/* enable tx and rx */
				ravb_rcv_snd_enable(ndev);
			}
		}
	}

ignore_link:
	if (intr_status & EESR_TWB) {
		/* Unused write back interrupt */
		if (intr_status & EESR_TABT) {	/* Transmit Abort int */
			ndev->stats.tx_aborted_errors++;
			netif_err(mdp, tx_err, ndev, "Transmit Abort\n");
		}
	}

	if (intr_status & EESR_RABT) {
		/* Receive Abort int */
		if (intr_status & EESR_RFRMER) {
			/* Receive Frame Overflow int */
			ndev->stats.rx_frame_errors++;
			netif_err(mdp, rx_err, ndev, "Receive Abort\n");
		}
	}

	if (intr_status & EESR_TDE) {
		/* Transmit Descriptor Empty int */
		ndev->stats.tx_fifo_errors++;
		netif_err(mdp, tx_err, ndev, "Transmit Descriptor Empty\n");
	}

	if (intr_status & EESR_TFE) {
		/* FIFO under flow */
		ndev->stats.tx_fifo_errors++;
		netif_err(mdp, tx_err, ndev, "Transmit FIFO Under flow\n");
	}

	if (intr_status & EESR_RDE) {
		/* Receive Descriptor Empty int */
		ndev->stats.rx_over_errors++;
		netif_err(mdp, rx_err, ndev, "Receive Descriptor Empty\n");
	}

	if (intr_status & EESR_RFE) {
		/* Receive FIFO Overflow int */
		ndev->stats.rx_fifo_errors++;
		netif_err(mdp, rx_err, ndev, "Receive FIFO Overflow\n");
	}

	if (!mdp->cd->no_ade && (intr_status & EESR_ADE)) {
		/* Address Error */
		ndev->stats.tx_fifo_errors++;
		netif_err(mdp, tx_err, ndev, "Address Error\n");
	}

	mask = EESR_TWB | EESR_TABT | EESR_ADE | EESR_TDE | EESR_TFE;
	if (mdp->cd->no_ade)
		mask &= ~EESR_ADE;
	if (intr_status & mask) {
		/* Tx error */
		u32 edtrr = ravb_read(ndev, EDTRR);

		/* dmesg */
		netdev_err(ndev, "TX error. status=%8.8x cur_tx=%8.8x dirty_tx=%8.8x state=%8.8x EDTRR=%8.8x.\n",
			   intr_status, mdp->cur_tx, mdp->dirty_tx,
			   (u32)ndev->state, edtrr);
		/* dirty buffer free */
		ravb_txfree(ndev);

		/* wakeup */
		netif_wake_queue(ndev);
	}
}

static irqreturn_t ravb_interrupt(int irq, void *netdev)
{
	struct net_device *ndev = netdev;
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_cpu_data *cd = mdp->cd;
	irqreturn_t ret = IRQ_NONE;
	unsigned long intr_status, intr_enable;

	spin_lock(&mdp->lock);

	/* Get interrupt status */
	intr_status = ravb_read(ndev, EESR);
	/* Mask it with the interrupt mask, forcing ECI interrupt to be always
	 * enabled since it's the one that  comes thru regardless of the mask,
	 * and we need to fully handle it in ravb_error() in order to quench
	 * it as it doesn't get cleared by just writing 1 to the ECI bit...
	 */
	intr_enable = ravb_read(ndev, EESIPR);
	intr_status &= intr_enable | DMAC_M_ECI;
	if (intr_status & (EESR_RX_CHECK | cd->tx_check | cd->eesr_err_check))
		ret = IRQ_HANDLED;
	else
		goto other_irq;

	if (intr_status & EESR_RX_CHECK) {
		if (napi_schedule_prep(&mdp->napi)) {
			/* Mask Rx interrupts */
			ravb_write(ndev, intr_enable & ~EESR_RX_CHECK,
				     EESIPR);
			__napi_schedule(&mdp->napi);
		} else {
			netdev_warn(ndev,
				    "ignoring interrupt, status 0x%08lx, mask 0x%08lx.\n",
				    intr_status, intr_enable);
		}
	}

	/* Tx Check */
	if (intr_status & cd->tx_check) {
		/* Clear Tx interrupts */
		ravb_write(ndev, intr_status & cd->tx_check, EESR);

		ravb_txfree(ndev);
		netif_wake_queue(ndev);
	}

	if (intr_status & cd->eesr_err_check) {
		/* Clear error interrupts */
		ravb_write(ndev, intr_status & cd->eesr_err_check, EESR);

		ravb_error(ndev, intr_status);
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
	unsigned long intr_status;

	for (;;) {
		intr_status = ravb_read(ndev, EESR);
		if (!(intr_status & EESR_RX_CHECK))
			break;
		/* Clear Rx interrupts */
		ravb_write(ndev, intr_status & EESR_RX_CHECK, EESR);

		if (ravb_rx(ndev, intr_status, &quota))
			goto out;
	}

	napi_complete(napi);

	/* Reenable Rx interrupts */
	ravb_write(ndev, mdp->cd->eesipr_value, EESIPR);
out:
	return budget - quota;
}

/* PHY state control function */
static void ravb_adjust_link(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct phy_device *phydev = mdp->phydev;
	int new_state = 0;

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

/* PHY init function */
static int ravb_phy_init(struct net_device *ndev)
{
	struct device_node *np = ndev->dev.parent->of_node;
	struct ravb_private *mdp = netdev_priv(ndev);
	struct phy_device *phydev = NULL;

	mdp->link = 0;
	mdp->speed = 0;
	mdp->duplex = -1;

	/* Try connect to PHY */
	if (np) {
		struct device_node *pn;

		pn = of_parse_phandle(np, "phy-handle", 0);
		phydev = of_phy_connect(ndev, pn,
					ravb_adjust_link, 0,
					mdp->phy_interface);

		if (!phydev)
			phydev = ERR_PTR(-ENOENT);
	} else {
		char phy_id[MII_BUS_ID_SIZE + 3];

		snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT,
			 mdp->mii_bus->id, mdp->phy_id);

		phydev = phy_connect(ndev, phy_id, ravb_adjust_link,
				     mdp->phy_interface);
	}

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

	if (!mdp->is_opened)
		return -EAGAIN;

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
	"rx_current", "tx_current",
	"rx_dirty", "tx_dirty",
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

	/* device-specific stats */
	data[i++] = mdp->cur_rx;
	data[i++] = mdp->cur_tx;
	data[i++] = mdp->dirty_rx;
	data[i++] = mdp->dirty_tx;
}

static void ravb_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
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

	ring->rx_max_pending = RX_RING_MAX;
	ring->tx_max_pending = TX_RING_MAX;
	ring->rx_pending = mdp->num_rx_ring;
	ring->tx_pending = mdp->num_tx_ring;
}

static int ravb_set_ringparam(struct net_device *ndev,
				struct ethtool_ringparam *ring)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	int ret;

	if (ring->tx_pending > TX_RING_MAX ||
	    ring->rx_pending > RX_RING_MAX ||
	    ring->tx_pending < TX_RING_MIN ||
	    ring->rx_pending < RX_RING_MIN)
		return -EINVAL;
	if (ring->rx_mini_pending || ring->rx_jumbo_pending)
		return -EINVAL;

	if (netif_running(ndev)) {
		netif_tx_disable(ndev);
		/* Disable interrupts by clearing the interrupt mask. */
		ravb_write(ndev, 0x0000, EESIPR);
		/* Stop the chip's Tx and Rx processes. */
		ravb_write(ndev, 0, EDTRR);
		ravb_write(ndev, 0, EDRRR);
		synchronize_irq(ndev->irq);
	}

	/* Free all the skbuffs in the Rx queue. */
	ravb_ring_free(ndev);
	/* Free DMA buffer */
	ravb_free_dma_buffer(mdp);

	/* Set new parameters */
	mdp->num_rx_ring = ring->rx_pending;
	mdp->num_tx_ring = ring->tx_pending;

	if (!mdp->is_opened)
		return 0;

	ret = ravb_ring_init(ndev);
	if (ret < 0) {
		netdev_err(ndev, "%s: ravb_ring_init failed.\n", __func__);
		return ret;
	}
	ret = ravb_dev_init(ndev, false);
	if (ret < 0) {
		netdev_err(ndev, "%s: ravb_dev_init failed.\n", __func__);
		return ret;
	}

	if (netif_running(ndev)) {
		ravb_write(ndev, mdp->cd->eesipr_value, EESIPR);
		/* Setting the Rx mode will start the Rx process. */
		ravb_write(ndev, EDRRR_R, EDRRR);
		netif_wake_queue(ndev);
	}

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
};

/* network device open function */
static int ravb_open(struct net_device *ndev)
{
	int ret = 0;
	struct ravb_private *mdp = netdev_priv(ndev);

	pm_runtime_get_sync(&mdp->pdev->dev);

	napi_enable(&mdp->napi);

	ret = request_irq(ndev->irq, ravb_interrupt,
			  mdp->cd->irq_flags, ndev->name, ndev);
	if (ret) {
		netdev_err(ndev, "Can not assign IRQ number\n");
		goto out_napi_off;
	}

	/* Descriptor set */
	ret = ravb_ring_init(ndev);
	if (ret)
		goto out_free_irq;

	/* device init */
	ret = ravb_dev_init(ndev, true);
	if (ret)
		goto out_free_irq;

	/* PHY control start*/
	ret = ravb_phy_start(ndev);
	if (ret)
		goto out_free_irq;

	mdp->is_opened = 1;

	return ret;

out_free_irq:
	free_irq(ndev->irq, ndev);
out_napi_off:
	napi_disable(&mdp->napi);
	pm_runtime_put_sync(&mdp->pdev->dev);
	return ret;
}

/* Timeout function */
static void ravb_tx_timeout(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_rxdesc *rxdesc;
	int i;

	netif_stop_queue(ndev);

	netif_err(mdp, timer, ndev,
		  "transmit timed out, status %8.8x, resetting...\n",
		  (int)ravb_read(ndev, EESR));

	/* tx_errors count up */
	ndev->stats.tx_errors++;

	/* Free all the skbuffs in the Rx queue. */
	for (i = 0; i < mdp->num_rx_ring; i++) {
		rxdesc = &mdp->rx_ring[i];
		rxdesc->status = 0;
		rxdesc->addr = 0xBADF00D0;
		if (mdp->rx_skbuff[i])
			dev_kfree_skb(mdp->rx_skbuff[i]);
		mdp->rx_skbuff[i] = NULL;
	}
	for (i = 0; i < mdp->num_tx_ring; i++) {
		if (mdp->tx_skbuff[i])
			dev_kfree_skb(mdp->tx_skbuff[i]);
		mdp->tx_skbuff[i] = NULL;
	}

	/* device init */
	ravb_dev_init(ndev, true);
}

/* Packet transmit function */
static int ravb_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_txdesc *txdesc;
	u32 entry;
	unsigned long flags;

	spin_lock_irqsave(&mdp->lock, flags);
	if ((mdp->cur_tx - mdp->dirty_tx) >= (mdp->num_tx_ring - 4)) {
		if (!ravb_txfree(ndev)) {
			netif_warn(mdp, tx_queued, ndev, "TxFD exhausted.\n");
			netif_stop_queue(ndev);
			spin_unlock_irqrestore(&mdp->lock, flags);
			return NETDEV_TX_BUSY;
		}
	}
	spin_unlock_irqrestore(&mdp->lock, flags);

	entry = mdp->cur_tx % mdp->num_tx_ring;
	mdp->tx_skbuff[entry] = skb;
	txdesc = &mdp->tx_ring[entry];
	/* soft swap. */
	if (!mdp->cd->hw_swap)
		ravb_soft_swap(phys_to_virt(ALIGN(txdesc->addr, 4)),
				 skb->len + 2);
	if (skb->len < ETH_ZLEN)
		txdesc->buffer_length = ETH_ZLEN;
	else
		txdesc->buffer_length = skb->len;
	txdesc->addr = dma_map_single(&ndev->dev, skb->data,
				      txdesc->buffer_length,
				      DMA_TO_DEVICE);
	if (dma_mapping_error(&ndev->dev, txdesc->addr)) {
		dev_kfree_skb_any(mdp->tx_skbuff[entry]);
		mdp->tx_skbuff[entry] = NULL;
		goto out;
	}

	if (entry >= mdp->num_tx_ring - 1)
		txdesc->status |= cpu_to_edmac(mdp, TD_TACT | TD_TDLE);
	else
		txdesc->status |= cpu_to_edmac(mdp, TD_TACT);

	mdp->cur_tx++;

	if (!(ravb_read(ndev, EDTRR) & ravb_get_edtrr_trns(mdp)))
		ravb_write(ndev, ravb_get_edtrr_trns(mdp), EDTRR);

out:
	return NETDEV_TX_OK;
}

static struct net_device_stats *ravb_get_stats(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	if (!mdp->is_opened)
		return &ndev->stats;

	ndev->stats.tx_dropped += ravb_read(ndev, TROCR);
	ravb_write(ndev, 0, TROCR);	/* (write clear) */
	ndev->stats.collisions += ravb_read(ndev, CDCR);
	ravb_write(ndev, 0, CDCR);	/* (write clear) */
	ndev->stats.tx_carrier_errors += ravb_read(ndev, LCCR);
	ravb_write(ndev, 0, LCCR);	/* (write clear) */

	ndev->stats.tx_carrier_errors += ravb_read(ndev, CERCR);
	ravb_write(ndev, 0, CERCR);	/* (write clear) */
	ndev->stats.tx_carrier_errors += ravb_read(ndev, CEECR);
	ravb_write(ndev, 0, CEECR);	/* (write clear) */

	return &ndev->stats;
}

/* device close function */
static int ravb_close(struct net_device *ndev)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	netif_stop_queue(ndev);

	/* Disable interrupts by clearing the interrupt mask. */
	ravb_write(ndev, 0x0000, EESIPR);

	/* Stop the chip's Tx and Rx processes. */
	ravb_write(ndev, 0, EDTRR);
	ravb_write(ndev, 0, EDRRR);

	ravb_get_stats(ndev);
	/* PHY Disconnect */
	if (mdp->phydev) {
		phy_stop(mdp->phydev);
		phy_disconnect(mdp->phydev);
	}

	free_irq(ndev->irq, ndev);

	napi_disable(&mdp->napi);

	/* Free all the skbuffs in the Rx queue. */
	ravb_ring_free(ndev);

	/* free DMA buffer */
	ravb_free_dma_buffer(mdp);

	pm_runtime_put_sync(&mdp->pdev->dev);

	mdp->is_opened = 0;

	return 0;
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
	if (dev->of_node) {
		ret = of_mdiobus_register(mdp->mii_bus, dev->of_node);
	} else {
		for (i = 0; i < PHY_MAX_ADDR; i++)
			mdp->mii_bus->irq[i] = PHY_POLL;
		if (pd->phy_irq > 0)
			mdp->mii_bus->irq[pd->phy] = pd->phy_irq;

		ret = mdiobus_register(mdp->mii_bus);
	}

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
	case RAVB_REG_FAST_RCAR:
		reg_offset = ravb_offset_fast_rcar;
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
static struct ravb_plat_data *ravb_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ravb_plat_data *pdata;
	const char *mac_addr;

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

	return pdata;
}

static const struct of_device_id ravb_match_table[] = {
	{ .compatible = "renesas,ether-r8a7790", .data = &r8a779x_data },
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
	int ret, devno = 0;
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
	mdp->num_tx_ring = TX_RING_SIZE;
	mdp->num_rx_ring = RX_RING_SIZE;
	mdp->addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mdp->addr)) {
		ret = PTR_ERR(mdp->addr);
		goto out_release;
	}

	spin_lock_init(&mdp->lock);
	mdp->pdev = pdev;

	if (pdev->dev.of_node)
		pd = ravb_parse_dt(&pdev->dev);
	if (!pd) {
		dev_err(&pdev->dev, "no platform data\n");
		ret = -EINVAL;
		goto out_release;
	}

	/* get PHY ID */
	mdp->phy_id = pd->phy;
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

	/* set function */
	ndev->netdev_ops = &ravb_netdev_ops;
	SET_ETHTOOL_OPS(ndev, &ravb_ethtool_ops);
	ndev->watchdog_timeo = TX_TIMEOUT;

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

	pm_runtime_put(&pdev->dev);
	platform_set_drvdata(pdev, ndev);

	return ret;

out_napi_del:
	netif_napi_del(&mdp->napi);
	sh_mdio_release(mdp);

out_release:
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
	{ "r8a7790-ether", (kernel_ulong_t)&r8a779x_data },
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
