/*
 * PTP 1588 clock using the Renesas Ethernet AVB
 *
 * Copyright (C) 2010 OMICRON electronics GmbH
 * Copyright (C) 2013-2014 Renesas Electronics Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include <linux/ptp_clock_kernel.h>

#include "ravb.h"

/* ravb ptp registers */
/* Bit definitions for the CCC register */
#define CSEL_SHIFT	(16)
#define CSEL_MASK	(0x3)
#define CSEL_DISABLE	(0) /* clock disable */
#define CSEL_PBUSCLK	(1) /* peripheral bus clock */
#define CSEL_TXCLK	(2) /* ethernet Tx clock */
#define CSEL_EXCLK	(3) /* external provided clock */

/* Bit definitions for the CSR register */
#define OPS_MASK	(0xf)
#define OPS_RESET	(1<<0)
#define OPS_CONFIG	(1<<1)
#define OPS_OPERATION	(1<<2)
#define OPS_STANDBY	(1<<3)

/* Bit definitions for the ISS register */
#define CGIS		(1<<13)	/* Combined Gptp Interrupt Summary */

/* Bit definitions for the GCCR register */
#define TCSS_SHIFT	(8)
#define TCSS_MASK	(0x3)
#define TCSS_GPTP	(0) /* gPTP timer value */
#define TCSS_CORR_GPTP	(1) /* Corrected gPTP timer value */
#define TCSS_AVTP	(2) /* AVTP presentation time value */
#define LMTT		(1<<5) /* Load Max Transit Time */
#define LPTC		(1<<4) /* Load Presentation Time Comparison */
#define LTI		(1<<3) /* Load Timer Increement */
#define LTO		(1<<2) /* Load Timer Offset */
#define TCR_MASK	(0x3)
#define TCR_NOREQ	(0) /* No request */
#define TCR_RESET	(1) /* Rset gPTP and AVTP presentation timer */
#define TCR_CAPTURE	(3) /* Capture values as selected
			     * by GCCR.TCSS to GCTt.CTV
			     */

/* Bit definitions for the GTI register */
#define TIV_MASK	(0x0fffffff)

/* Bit definitions for the GTO2 register */
#define GTO2_TOV_MASK	(0x0000ffff)

/* Bit definitions for the GIC register */
#define PTME		(1<<2)
#define PTOE		(1<<1)
#define PTCE		(1<<0)

/* Bit definitions for the GIS register */
#define PTMF		(1<<2)
#define PTOF		(1<<1)
#define PTCF		(1<<0)

/* Bit definitions for the GCT2 register */
#define GCT2_CTV_MASK	(0x0000ffff)

#define DRIVER		"ravb_ptp"
#if defined(CONFIG_ARCH_R8A7790)
#define N_EXT_TS	0
#else
#define N_EXT_TS	1
#endif

static inline void ravb_ptp_tcr_request(struct ravb_ptp *ravb_ptp,
		int request)
{
	struct net_device *ndev = ravb_ptp->ndev;

	if ((ravb_read(ndev, CSR) & OPS_MASK) & OPS_OPERATION) {
		while ((ravb_read(ndev, GCCR) & TCR_MASK) != TCR_NOREQ)
			;
		ravb_write(ndev,
			ravb_read(ndev, GCCR) | request, GCCR);
		while ((ravb_read(ndev, GCCR) & TCR_MASK) != TCR_NOREQ)
			;
	}
}

static inline bool ravb_ptp_is_config(struct ravb_ptp *ravb_ptp)
{
	struct net_device *ndev = ravb_ptp->ndev;

	if ((ravb_read(ndev, CSR) & OPS_MASK) & OPS_CONFIG)
		return true;
	else
		return false;
}

/* Caller must hold lock */
static void ravb_ptp_time_read(struct ravb_ptp *ravb_ptp,
		struct timespec *ts)
{
	struct net_device *ndev = ravb_ptp->ndev;

	ravb_ptp_tcr_request(ravb_ptp, TCR_CAPTURE);

	ts->tv_nsec = ravb_read(ndev, GCT0);
	ts->tv_sec = ravb_read(ndev, GCT1);
}

/* Caller must hold lock */
static u64 ravb_ptp_cnt_read(struct ravb_ptp *ravb_ptp)
{
	struct timespec ts;
	ktime_t kt;

	ravb_ptp_time_read(ravb_ptp, &ts);
	kt = timespec_to_ktime(ts);

	return ktime_to_ns(kt);
}

/* Caller must hold lock */
static void ravb_ptp_time_write(struct ravb_ptp *ravb_ptp,
		const struct timespec *ts)
{
	struct net_device *ndev = ravb_ptp->ndev;

	ravb_ptp_tcr_request(ravb_ptp, TCR_RESET);

	ravb_write(ndev, ts->tv_nsec, GTO0);
	ravb_write(ndev, ts->tv_sec, GTO1);
	ravb_write(ndev,
			ravb_read(ndev, GCCR) | LTO, GCCR);
	if ((ravb_read(ndev, CSR) & OPS_MASK) & OPS_OPERATION)
		while (ravb_read(ndev, GCCR) & LTO)
			;
}

/* Caller must hold lock */
static void ravb_ptp_cnt_write(struct ravb_ptp *ravb_ptp, u64 ns)
{
	struct timespec ts;

	ts = ns_to_timespec(ns);

	ravb_ptp_time_write(ravb_ptp, &ts);
}

/* Caller must hold lock */
static void ravb_ptp_select_counter(struct ravb_ptp *ravb_ptp, u16 sel)
{
	struct net_device *ndev = ravb_ptp->ndev;
	u32 val;

	while ((ravb_read(ndev, GCCR) & TCR_MASK) != TCR_NOREQ)
		;
	val = ravb_read(ndev, GCCR) & ~(TCSS_MASK<<TCSS_SHIFT);
	ravb_write(ndev, val | (sel<<TCSS_SHIFT), GCCR);
}

/* Caller must hold lock */
static void ravb_ptp_update_addend(struct ravb_ptp *ravb_ptp, u32 addend)
{
	struct net_device *ndev = ravb_ptp->ndev;
	ravb_write(ndev, addend & TIV_MASK, GTI);
	ravb_write(ndev,
			ravb_read(ndev, GCCR) | LTI, GCCR);
	if ((ravb_read(ndev, CSR) & OPS_MASK) & OPS_OPERATION)
		while (ravb_read(ndev, GCCR) & LTI)
			;
}

/* Interrupt service routine */
#if N_EXT_TS
static irqreturn_t isr(int irq, void *priv)
{
	struct ravb_ptp *ravb_ptp = priv;
	struct net_device *ndev = ravb_ptp->ndev;
	struct ptp_clock_event event;
	u32 ack = 0, val;

	val = ravb_read(ndev, ISS);
	if (val & CGIS) {
		val = ravb_read(ndev, GIS);
		if (val & PTCF) {
			ack |= PTCF;
			event.type = PTP_CLOCK_EXTTS;
			event.index = 0;
			event.timestamp = ravb_ptp_cnt_read(ravb_ptp);
			ptp_clock_event(ravb_ptp->clock, &event);
		}
	}

	if (ack) {
		ravb_write(ndev, ack, GIS);
		return IRQ_HANDLED;
	} else
		return IRQ_NONE;
}
#endif

/* PTP clock operations */
static int ravb_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, addend;
	int neg_adj = 0;
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}
	addend = ravb_ptp->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);

	addend = neg_adj ? addend - diff : addend + diff;

	spin_lock_irqsave(&ravb_ptp->lock, flags);
	ravb_ptp_update_addend(ravb_ptp, addend);
	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	s64 now;
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	if (ravb_ptp_is_config(ravb_ptp))
		return -EBUSY;

	spin_lock_irqsave(&ravb_ptp->lock, flags);

	now = ravb_ptp_cnt_read(ravb_ptp);
	now += delta;
	ravb_ptp_cnt_write(ravb_ptp, now);

	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_gettime(struct ptp_clock_info *ptp, struct timespec *ts)
{
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	if (ravb_ptp_is_config(ravb_ptp))
		return -EBUSY;

	spin_lock_irqsave(&ravb_ptp->lock, flags);

	ravb_ptp_time_read(ravb_ptp, ts);

	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_settime(struct ptp_clock_info *ptp,
		const struct timespec *ts)
{
	unsigned long flags;
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);

	spin_lock_irqsave(&ravb_ptp->lock, flags);

	ravb_ptp_time_write(ravb_ptp, ts);

	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	return 0;
}

static int ravb_ptp_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
#if N_EXT_TS
	struct ravb_ptp *ravb_ptp = container_of(ptp,
			struct ravb_ptp, caps);
	struct net_device *ndev = ravb_ptp->ndev;
	unsigned long flags;
	u32 bit, mask;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		switch (rq->extts.index) {
		case 0:
			bit = PTCE;
			break;
		default:
			return -EINVAL;
		}

		spin_lock_irqsave(&ravb_ptp->lock, flags);
		mask = ravb_ptp_read(ravb_ptp, GIC);
		if (on)
			mask |= bit;
		else
			mask &= ~bit;
		ravb_write(ndev, mask, GIC);
		spin_unlock_irqrestore(&ravb_ptp->lock, flags);
		return 0;

	default:
		break;
	}
#endif

	return -EOPNOTSUPP;
}

static struct ptp_clock_info ravb_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "ravb clock",
	.max_adj	= 50000000,
	.n_ext_ts	= N_EXT_TS,
	.adjfreq	= ravb_ptp_adjfreq,
	.adjtime	= ravb_ptp_adjtime,
	.gettime	= ravb_ptp_gettime,
	.settime	= ravb_ptp_settime,
	.enable		= ravb_ptp_enable,
};

int ravb_ptp_init(struct net_device *ndev,
			   struct platform_device *pdev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_ptp *ravb_ptp;
	int err = -ENOMEM;
	unsigned long flags;

	ravb_ptp = kzalloc(sizeof(*ravb_ptp), GFP_KERNEL);
	if (!ravb_ptp)
		goto no_memory;

	err = -ENODEV;

	ravb_ptp->ndev = ndev;
	ravb_ptp->caps = ravb_ptp_caps;

#if N_EXT_TS
	ravb_ptp->irq = platform_get_irq(pdev, 0);

	if (ravb_ptp->irq == NO_IRQ) {
		pr_err("irq not in platfrom\n");
		goto no_node;
	}
	if (request_irq(ravb_ptp->irq, isr, IRQF_SHARED,
				DRIVER, ravb_ptp)) {
		pr_err("request_irq failed\n");
		goto no_node;
	}
#endif
	ravb_ptp->default_addend = ravb_read(ndev, GTI);

	spin_lock_init(&ravb_ptp->lock);

	spin_lock_irqsave(&ravb_ptp->lock, flags);
	ravb_ptp_select_counter(ravb_ptp, TCSS_CORR_GPTP);
	spin_unlock_irqrestore(&ravb_ptp->lock, flags);

	ravb_ptp->clock = ptp_clock_register(&ravb_ptp->caps, &pdev->dev);
	if (IS_ERR(ravb_ptp->clock)) {
		err = PTR_ERR(ravb_ptp->clock);
		goto no_clock;
	}

	mdp->ptp = ravb_ptp;

	return 0;

no_clock:
#if N_EXT_TS
	free_irq(ravb_ptp->irq, ravb_ptp);
no_node:
#endif
	kfree(ravb_ptp);
no_memory:
	return err;
}

int ravb_ptp_stop(struct net_device *ndev,
			   struct platform_device *pdev)
{
	struct ravb_private *mdp = netdev_priv(ndev);
	struct ravb_ptp *ravb_ptp = mdp->ptp;

	ravb_write(ndev, 0, GIC);
	ravb_write(ndev, 0, GIS);

	ptp_clock_unregister(ravb_ptp->clock);
#if N_EXT_TS
	free_irq(ravb_ptp->irq, ravb_ptp);
#endif
	kfree(ravb_ptp);
	return 0;
}
