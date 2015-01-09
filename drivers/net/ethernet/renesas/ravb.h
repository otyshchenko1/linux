/*  SuperH Ethernet device driver
 *
 *  Copyright (C) 2014-2015 Renesas Electronics Corporation
 *  Copyright (C) 2006-2012 Nobuhiro Iwamatsu
 *  Copyright (C) 2008-2012 Renesas Solutions Corp.
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

#ifndef __RAVB_H__
#define __RAVB_H__

#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/ptp_clock_kernel.h>

#define CARDNAME	"ravb"
#define TX_TIMEOUT	(5*HZ)
#define BE_TX_RING_SIZE	64	/* Tx ring size for Best Effort */
#define BE_RX_RING_SIZE	1024	/* Rx ring size for Best Effort */
#define NC_TX_RING_SIZE	64	/* Tx ring size for Network Control */
#define NC_RX_RING_SIZE	64	/* Rx ring size for Network Control */
#define BE_TX_RING_MIN	64
#define BE_RX_RING_MIN	64
#define NC_TX_RING_MIN	64
#define NC_RX_RING_MIN	64
#define BE_TX_RING_MAX	1024
#define BE_RX_RING_MAX	2048
#define NC_TX_RING_MAX	128
#define NC_RX_RING_MAX	128
#define PKT_BUF_SZ	1538

enum {
	/* AVB-DMAC specific registers */
	CCC,
	DBAT,
	DLR,
	CSR,
	CDAR0,
	CDAR1,
	CDAR2,
	CDAR3,
	CDAR4,
	CDAR5,
	CDAR6,
	CDAR7,
	CDAR8,
	CDAR9,
	CDAR10,
	CDAR11,
	CDAR12,
	CDAR13,
	CDAR14,
	CDAR15,
	CDAR16,
	CDAR17,
	CDAR18,
	CDAR19,
	CDAR20,
	CDAR21,
	ESR,
	RCR,
	RQC,
	RPC,
	UFCW,
	UFCS,
	UFCV0,
	UFCV1,
	UFCV2,
	UFCV3,
	UFCV4,
	UFCD0,
	UFCD1,
	UFCD2,
	UFCD3,
	UFCD4,
	SFO,
	SFP0,
	SFP1,
	SFP2,
	SFP3,
	SFP4,
	SFP5,
	SFP6,
	SFP7,
	SFP8,
	SFP9,
	SFP10,
	SFP11,
	SFP12,
	SFP13,
	SFP14,
	SFP15,
	SFP16,
	SFP17,
	SFP18,
	SFP19,
	SFP20,
	SFP21,
	SFP22,
	SFP23,
	SFP24,
	SFP25,
	SFP26,
	SFP27,
	SFP28,
	SFP29,
	SFP30,
	SFP31,
	SFM0,
	SFM1,
	RTSR,
	CIAR,
	LIAR,
	TGC,
	TCCR,
	TSR,
	MFA,
	TFA0,
	TFA1,
	TFA2,
	CIVR0,
	CIVR1,
	CDVR0,
	CDVR1,
	CUL0,
	CUL1,
	CLL0,
	CLL1,
	DIC,
	DIS,
	EIC,
	EIS,
	RIC0,
	RIS0,
	RIC1,
	RIS1,
	RIC2,
	RIS2,
	TIC,
	TIS,
	ISS,
	GCCR,
	GMTT,
	GPTC,
	GTI,
	GTO0,
	GTO1,
	GTO2,
	GIC,
	GIS,
	GCPT,
	GCT0,
	GCT1,
	GCT2,

	/* Ether registers */
	ECMR,
	ECSR,
	ECSIPR,
	PIR,
	PSR,
	RDMLR,
	PIPR,
	RFLR,
	IPGR,
	APR,
	MPR,
	PFTCR,
	PFRCR,
	RFCR,
	RFCF,
	TPAUSER,
	TPAUSECR,
	BCFR,
	BCFRR,
	GECMR,
	BCULR,
	MAHR,
	MALR,
	TROCR,
	CDCR,
	LCCR,
	CNDCR,
	CEFCR,
	FRECR,
	TSFRCR,
	TLFRCR,
	CERCR,
	CEECR,
	MAFCR,
	RTRATE,
	CSMR,

	/* This value must be written at last. */
	RAVB_MAX_REGISTER_OFFSET,
};

enum {
	RAVB_REG_RCAR_GEN2,
};

/* Driver's parameters */
#define RAVB_ALIGN	128

/* Hwtstamp*/
#define RAVB_TXTSTAMP_VALID	0x00000001	/* tx timestamp valid */
#define RAVB_TXTSTAMP_ENABLED	0x00000010	/* enable tx timestampping */
#define RAVB_RXTSTAMP_VALID	0x00000001	/* rx timestamp valid */
#define RAVB_RXTSTAMP_TYPE_MASK	0x00000006	/* rx type mask */
#define RAVB_RXTSTAMP_TYPE_V2_L2_EVENT	0x02
#define RAVB_RXTSTAMP_TYPE_ALL		0x06
#define RAVB_RXTSTAMP_ENABLED	0x00000010	/* enable rx timestampping */

/* Register's bits of Ethernet AVB */
enum CCC_BIT {
	CCC_OPC	= 0x00000003,
	CCC_DTSR	= 0x00000040,
	CCC_CSEL	= 0x00030000,
	CCC_BOC	= 0x00100000,
	CCC_LBME	= 0x01000000,
};

enum CSR_BIT {
	CSR_OPS	= 0x0000000f,
	CSR_DTS	= 0x00000080,
	CSR_TPO0	= 0x00010000,
	CSR_TPO1	= 0x00020000,
	CSR_TPO2	= 0x00040000,
	CSR_TPO3	= 0x00080000,
	CSR_RPO	= 0x00100000,
};

enum ESR_BIT {
	ESR_EQN	= 0x0000000f,
	ESR_ET	= 0x00000f00,
	ESR_EIL	= 0x00001000,
};

enum RCR_BIT {
	RCR_EFFS	= 0x00000001,
	RCR_ENCF	= 0x00000002,
	RCR_ESF	= 0x0000000c,
	RCR_ETS0	= 0x00000010,
	RCR_ETS2	= 0x00000020,
	RCR_RFCL	= 0x1fff0000,
};

enum RQC_BIT {
	RQC_RSM0	= 0x00000003,
	TSEL0	= 0x00000004,
	UFCC0	= 0x00000030,
	RQC_RSM1	= 0x00000300,
	TSEL1	= 0x00000400,
	UFCC1	= 0x00003000,
	RQC_RSM2	= 0x00030000,
	TSEL2	= 0x00040000,
	UFCC2	= 0x00300000,
	RQC_RSM3	= 0x03000000,
	TSEL3	= 0x04000000,
	UFCC3	= 0x30000000,
};

enum RPC_BIT {
	RPC_PCNT	= 0x00000700,
	RPC_DCNT	= 0x00ff0000,
};

enum RTC_BIT {
	RTC_MFL0	= 0x00000fff,
	RTC_MFL1	= 0x0fff0000,
};

enum UFCW_BIT {
	UFCW_WL0	= 0x0000003f,
	UFCW_WL1	= 0x00003f00,
	UFCW_WL2	= 0x003f0000,
	UFCW_WL3	= 0x3f000000,
};

enum UFCS_BIT {
	UFCS_SL0	= 0x0000003f,
	UFCS_SL1	= 0x00003f00,
	UFCS_SL2	= 0x003f0000,
	UFCS_SL3	= 0x3f000000,
};

enum UFCV_BIT {
	UFCV_CV0	= 0x0000003f,
	UFCV_CV1	= 0x00003f00,
	UFCV_CV2	= 0x003f0000,
	UFCV_CV3	= 0x3f000000,
};

enum UFCD_BIT {
	UFCD_DV0	= 0x0000003f,
	UFCD_DV1	= 0x00003f00,
	UFCD_DV2	= 0x003f0000,
	UFCD_DV3	= 0x3f000000,
};

enum SFO_BIT {
	SFO_FPB	= 0x0000003f,
};

enum TGC_BIT {
	TGC_TSM0	= 0x00000001,
	TGC_TSM1	= 0x00000002,
	TGC_TSM2	= 0x00000004,
	TGC_TSM3	= 0x00000008,
	TGC_TQP	= 0x00000030,
	TGC_TBD0	= 0x00000300,
	TGC_TBD1	= 0x00003000,
	TGC_TBD2	= 0x00030000,
	TGC_TBD3	= 0x00300000,
};

enum TCCR_BIT {
	TCCR_TSRQ0	= 0x00000001,
	TCCR_TSRQ1	= 0x00000002,
	TCCR_TSRQ2	= 0x00000004,
	TCCR_TSRQ3	= 0x00000008,
	TCCR_TFEN	= 0x00000100,
	TCCR_TFR	= 0x00000200,
	TCCR_MFEN	= 0x00010000,
	TCCR_MFR	= 0x00020000,
};

enum TSR_BIT {
	TSR_CCS0	= 0x00000003,
	TSR_CCS1	= 0x0000000c,
	TSR_TFFL	= 0x00000700,
	TSR_MFFL	= 0x001f0000,
};

enum MFA_BIT {
	MFA_MSV	= 0x0000000f,
	MFA_MST	= 0x03ff0000,
};

enum GCCR_BIT {
	GCCR_TCR	= 0x00000003,
	GCCR_LTO	= 0x00000004,
	GCCR_LTI	= 0x00000008,
	GCCR_LPTC	= 0x00000010,
	GCCR_LMTT	= 0x00000020,
	GCCR_TCSS	= 0x00000300,
};

enum DIC_BIT {
	DIC_DPE1	= 0x00000002,
	DIC_DPE2	= 0x00000004,
	DIC_DPE3	= 0x00000008,
	DIC_DPE4	= 0x00000010,
	DIC_DPE5	= 0x00000020,
	DIC_DPE6	= 0x00000040,
	DIC_DPE7	= 0x00000080,
	DIC_DPE8	= 0x00000100,
	DIC_DPE9	= 0x00000200,
	DIC_DPE10	= 0x00000400,
	DIC_DPE11	= 0x00000800,
	DIC_DPE12	= 0x00001000,
	DIC_DPE13	= 0x00002000,
	DIC_DPE14	= 0x00004000,
	DIC_DPE15	= 0x00008000,
};

enum DIS_BIT {
	DIS_DPF1	= 0x00000002,
	DIS_DPF2	= 0x00000004,
	DIS_DPF3	= 0x00000008,
	DIS_DPF4	= 0x00000010,
	DIS_DPF5	= 0x00000020,
	DIS_DPF6	= 0x00000040,
	DIS_DPF7	= 0x00000080,
	DIS_DPF8	= 0x00000100,
	DIS_DPF9	= 0x00000200,
	DIS_DPF10	= 0x00000400,
	DIS_DPF11	= 0x00000800,
	DIS_DPF12	= 0x00001000,
	DIS_DPF13	= 0x00002000,
	DIS_DPF14	= 0x00004000,
	DIS_DPF15	= 0x00008000,
};

enum EIC_BIT {
	EIC_MREE	= 0x00000001,
	EIC_MTEE	= 0x00000002,
	EIC_QEE	= 0x00000004,
	EIC_SEE	= 0x00000008,
	EIC_CLLE0	= 0x00000010,
	EIC_CLLE1	= 0x00000020,
	EIC_CULE0	= 0x00000040,
	EIC_CULE1	= 0x00000080,
	EIC_TFFE	= 0x00000100,
	EIC_MFFE	= 0x00000200,
};

enum EIS_BIT {
	EIS_MREF	= 0x00000001,
	EIS_MTEF	= 0x00000002,
	EIS_QEF	= 0x00000004,
	EIS_SEF	= 0x00000008,
	EIS_CLLF0	= 0x00000010,
	EIS_CLLF1	= 0x00000020,
	EIS_CULF0	= 0x00000040,
	EIS_CULF1	= 0x00000080,
	EIS_TFFF	= 0x00000100,
	EIS_MFFF	= 0x00000200,
	EIS_QFS	= 0x00010000,
};

enum RIC0_BIT {
	RIC0_FRE0	= 0x00000001,
	RIC0_FRE1	= 0x00000002,
	RIC0_FRE2	= 0x00000004,
	RIC0_FRE3	= 0x00000008,
	RIC0_FRE4	= 0x00000010,
	RIC0_FRE5	= 0x00000020,
	RIC0_FRE6	= 0x00000040,
	RIC0_FRE7	= 0x00000080,
	RIC0_FRE8	= 0x00000100,
	RIC0_FRE9	= 0x00000200,
	RIC0_FRE10	= 0x00000400,
	RIC0_FRE11	= 0x00000800,
	RIC0_FRE12	= 0x00001000,
	RIC0_FRE13	= 0x00002000,
	RIC0_FRE14	= 0x00004000,
	RIC0_FRE15	= 0x00008000,
	RIC0_FRE16	= 0x00010000,
	RIC0_FRE17	= 0x00020000,
};

enum RIS0_BIT {
	RIS0_FRF0	= 0x00000001,
	RIS0_FRF1	= 0x00000002,
	RIS0_FRF2	= 0x00000004,
	RIS0_FRF3	= 0x00000008,
	RIS0_FRF4	= 0x00000010,
	RIS0_FRF5	= 0x00000020,
	RIS0_FRF6	= 0x00000040,
	RIS0_FRF7	= 0x00000080,
	RIS0_FRF8	= 0x00000100,
	RIS0_FRF9	= 0x00000200,
	RIS0_FRF10	= 0x00000400,
	RIS0_FRF11	= 0x00000800,
	RIS0_FRF12	= 0x00001000,
	RIS0_FRF13	= 0x00002000,
	RIS0_FRF14	= 0x00004000,
	RIS0_FRF15	= 0x00008000,
	RIS0_FRF16	= 0x00010000,
	RIS0_FRF17	= 0x00020000,
};

enum RIC1_BIT {
	RIC1_RWE0	= 0x00000001,
	RIC1_RWE1	= 0x00000002,
	RIC1_RWE2	= 0x00000004,
	RIC1_RWE3	= 0x00000008,
	RIC1_RWE4	= 0x00000010,
	RIC1_RWE5	= 0x00000020,
	RIC1_RWE6	= 0x00000040,
	RIC1_RWE7	= 0x00000080,
	RIC1_RWE8	= 0x00000100,
	RIC1_RWE9	= 0x00000200,
	RIC1_RWE10	= 0x00000400,
	RIC1_RWE11	= 0x00000800,
	RIC1_RWE12	= 0x00001000,
	RIC1_RWE13	= 0x00002000,
	RIC1_RWE14	= 0x00004000,
	RIC1_RWE15	= 0x00008000,
	RIC1_RWE16	= 0x00010000,
	RIC1_RWE17	= 0x00020000,
	RIC1_RFWE	= 0x80000000,
};

enum RIS1_BIT {
	RIS1_RWF0	= 0x00000001,
	RIS1_RWF1	= 0x00000002,
	RIS1_RWF2	= 0x00000004,
	RIS1_RWF3	= 0x00000008,
	RIS1_RWF4	= 0x00000010,
	RIS1_RWF5	= 0x00000020,
	RIS1_RWF6	= 0x00000040,
	RIS1_RWF7	= 0x00000080,
	RIS1_RWF8	= 0x00000100,
	RIS1_RWF9	= 0x00000200,
	RIS1_RWF10	= 0x00000400,
	RIS1_RWF11	= 0x00000800,
	RIS1_RWF12	= 0x00001000,
	RIS1_RWF13	= 0x00002000,
	RIS1_RWF14	= 0x00004000,
	RIS1_RWF15	= 0x00008000,
	RIS1_RWF16	= 0x00010000,
	RIS1_RWF17	= 0x00020000,
	RIS1_RFWF	= 0x80000000,
};

enum RIC2_BIT {
	RIC2_QFE0	= 0x00000001,
	RIC2_QFE1	= 0x00000002,
	RIC2_QFE2	= 0x00000004,
	RIC2_QFE3	= 0x00000008,
	RIC2_QFE4	= 0x00000010,
	RIC2_QFE5	= 0x00000020,
	RIC2_QFE6	= 0x00000040,
	RIC2_QFE7	= 0x00000080,
	RIC2_QFE8	= 0x00000100,
	RIC2_QFE9	= 0x00000200,
	RIC2_QFE10	= 0x00000400,
	RIC2_QFE11	= 0x00000800,
	RIC2_QFE12	= 0x00001000,
	RIC2_QFE13	= 0x00002000,
	RIC2_QFE14	= 0x00004000,
	RIC2_QFE15	= 0x00008000,
	RIC2_QFE16	= 0x00010000,
	RIC2_QFE17	= 0x00020000,
	RIC2_RFFE	= 0x80000000,
};

enum RIS2_BIT {
	RIS2_QFF0	= 0x00000001,
	RIS2_QFF1	= 0x00000002,
	RIS2_QFF2	= 0x00000004,
	RIS2_QFF3	= 0x00000008,
	RIS2_QFF4	= 0x00000010,
	RIS2_QFF5	= 0x00000020,
	RIS2_QFF6	= 0x00000040,
	RIS2_QFF7	= 0x00000080,
	RIS2_QFF8	= 0x00000100,
	RIS2_QFF9	= 0x00000200,
	RIS2_QFF10	= 0x00000400,
	RIS2_QFF11	= 0x00000800,
	RIS2_QFF12	= 0x00001000,
	RIS2_QFF13	= 0x00002000,
	RIS2_QFF14	= 0x00004000,
	RIS2_QFF15	= 0x00008000,
	RIS2_QFF16	= 0x00010000,
	RIS2_QFF17	= 0x00020000,
	RIS2_RFFF	= 0x80000000,
};

#define RIS2_CHECK	(RIS2_QFF0 | RIS2_RFFF)


enum TIC_BIT {
	TIC_FTE0	= 0x00000001,
	TIC_FTE1	= 0x00000002,
	TIC_FTE2	= 0x00000004,
	TIC_FTE3	= 0x00000008,
	TIC_TFUE	= 0x00000100,
	TIC_TFWE	= 0x00000200,
	TIC_MFUE	= 0x00000400,
	TIC_MFWE	= 0x00000800,
};

enum TIS_BIT {
	TIS_FTF0	= 0x00000001,
	TIS_FTF1	= 0x00000002,
	TIS_FTF2	= 0x00000004,
	TIS_FTF3	= 0x00000008,
	TIS_TFUF	= 0x00000100,
	TIS_TFWF	= 0x00000200,
	TIS_MFUF	= 0x00000400,
	TIS_MFWF	= 0x00000800,
};

enum GIC_BIT {
	GIC_PTCE	= 0x00000001,
	GIC_PTOE	= 0x00000002,
	GIC_PTME	= 0x00000004,
};

enum GIS_BIT {
	GIS_PTCF	= 0x00000001,
	GIS_PTOF	= 0x00000002,
	GIS_PTMF	= 0x00000004,
};

enum ISS_BIT {
	ISS_FRS	= 0x00000001,
	ISS_RWS	= 0x00000002,
	ISS_FTS	= 0x00000004,
	ISS_ES	= 0x00000040,
	ISS_MS	= 0x00000080,
	ISS_TFUS	= 0x00000100,
	ISS_TFWS	= 0x00000200,
	ISS_MFUS	= 0x00000400,
	ISS_MFWA	= 0x00000800,
	ISS_RFWS	= 0x00001000,
	ISS_CGIS	= 0x00002000,
	ISS_DPS1	= 0x00020000,
	ISS_DPS2	= 0x00040000,
	ISS_DPS3	= 0x00080000,
	ISS_DPS4	= 0x00100000,
	ISS_DPS5	= 0x00200000,
	ISS_DPS6	= 0x00400000,
	ISS_DPS7	= 0x00800000,
	ISS_DPS8	= 0x01000000,
	ISS_DPS9	= 0x02000000,
	ISS_DPS10	= 0x04000000,
	ISS_DPS11	= 0x08000000,
	ISS_DPS12	= 0x10000000,
	ISS_DPS13	= 0x20000000,
	ISS_DPS14	= 0x40000000,
	ISS_DPS15	= 0x80000000,
};

/* TPAUSER */
enum TPAUSER_BIT {
	TPAUSER_TPAUSE = 0x0000ffff,
	TPAUSER_UNLIMITED = 0,
};

/* BCFR */
enum BCFR_BIT {
	BCFR_RPAUSE = 0x0000ffff,
	BCFR_UNLIMITED = 0,
};

/* PIR */
enum PIR_BIT {
	PIR_MDI = 0x08, PIR_MDO = 0x04, PIR_MMD = 0x02, PIR_MDC = 0x01,
};

/* PSR */
enum PHY_STATUS_BIT { PHY_ST_LINK = 0x01, };

/* Receive descriptor bit */
/* ECMR */
enum FELIC_MODE_BIT {
	ECMR_TRCCM = 0x04000000, ECMR_RCSC = 0x00800000,
	ECMR_DPAD = 0x00200000, ECMR_RZPF = 0x00100000,
	ECMR_ZPF = 0x00080000, ECMR_PFR = 0x00040000, ECMR_RXF = 0x00020000,
	ECMR_TXF = 0x00010000, ECMR_MCT = 0x00002000, ECMR_PRCEF = 0x00001000,
	ECMR_PMDE = 0x00000200, ECMR_RE = 0x00000040, ECMR_TE = 0x00000020,
	ECMR_RTM = 0x00000010, ECMR_ILB = 0x00000008, ECMR_ELB = 0x00000004,
	ECMR_DM = 0x00000002, ECMR_PRM = 0x00000001,
};

/* ECSR */
enum ECSR_STATUS_BIT {
	ECSR_BRCRX = 0x20, ECSR_PSRTO = 0x10,
	ECSR_LCHNG = 0x04,
	ECSR_MPD = 0x02, ECSR_ICD = 0x01,
};

#define DEFAULT_ECSR_INIT	(ECSR_BRCRX | ECSR_PSRTO | ECSR_LCHNG | \
				 ECSR_ICD | ECSIPR_MPDIP)

/* ECSIPR */
enum ECSIPR_STATUS_MASK_BIT {
	ECSIPR_BRCRXIP = 0x20, ECSIPR_PSRTOIP = 0x10,
	ECSIPR_LCHNGIP = 0x04,
	ECSIPR_MPDIP = 0x02, ECSIPR_ICDIP = 0x01,
};

#define DEFAULT_ECSIPR_INIT	(ECSIPR_BRCRXIP | ECSIPR_PSRTOIP | \
				 ECSIPR_LCHNGIP | ECSIPR_ICDIP | ECSIPR_MPDIP)

/* APR */
enum APR_BIT {
	APR_AP = 0x00000001,
};

/* MPR */
enum MPR_BIT {
	MPR_MP = 0x00000001,
};

/* The ethernet avb descriptor definitions. */
enum DT {
	/* frame data */
	DT_FSTART    = 5,
	DT_FMID      = 4,
	DT_FEND      = 6,
	DT_FSINGLE   = 7,
	/* chain control */
	DT_LINK      = 8,
	DT_LINKFIX   = 9,
	DT_EOS       = 10,
	/* HW/SW arbitration */
	DT_FEMPTY    = 12,
	DT_FEMPTY_IS = 13,
	DT_FEMPTY_IC = 14,
	DT_FEMPTY_ND = 15,
	DT_LEMPTY    = 2,
	DT_EEMPTY    = 3,
	/* 0,1,11 is reserved */
};

struct ravb_desc {
#if defined(__LITTLE_ENDIAN)
	u32 ds:12;	/* descriptor size */
	u32 cc:12;	/* content control */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 dt:4;	/* dscriotor type */
#else
	u32 dt:4;	/* dscriotor type */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 cc:12;	/* content control */
	u32 ds:12;	/* descriptor size */
#endif
	u32 dptr;	/* descpriptor pointer */
};

struct ravb_ex_desc {
#if defined(__LITTLE_ENDIAN)
	u32 ds:12;	/* descriptor size */
	u32 cc:12;	/* content control */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 dt:4;	/* dscriotor type */
#else
	u32 dt:4;	/* dscriotor type */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 cc:12;	/* content control */
	u32 ds:12;	/* descriptor size */
#endif
	u32 dptr;	/* descpriptor pointer */
	u32 ts_n;	/* timestampe nsec */
	u32 ts_sl;	/* timestamp low */
#if defined(__LITTLE_ENDIAN)
	u32 res:16;	/* reserved bit */
	u32 ts_sh:16;	/* timestamp high */
#else
	u32 ts_sh:16;	/* timestamp high */
	u32 res:16;	/* reserved bit */
#endif
};

/* MAC reception status */
enum MSC {
	MSC_MC   = 1<<7, /* [7] Multicast frame reception */
	MSC_CEEF = 1<<6, /* [6] Carrier extension error */
	MSC_CRL  = 1<<5, /* [5] Carrier lost */
	MSC_FRE  = 1<<4, /* [4] Fraction error (isn't a multiple of 8bits) */
	MSC_RTLF = 1<<3, /* [3] Frame length error (frame too long) */
	MSC_RTSF = 1<<2, /* [2] Frame length error (frame too short) */
	MSC_RFE  = 1<<1, /* [1] Frame reception error (flagged by PHY) */
	MSC_CRC  = 1<<0, /* [0] Frame CRC error */
};

struct ravb_rxdesc {
#if defined(__LITTLE_ENDIAN)
	u32 ds:12;	/* descriptor size */
	u32 ei:1;	/* error indication */
	u32 ps:2;	/* padding selection */
	u32 tr:1;	/* truncation indication */
	u32 msc:8;	/* mac status code */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 dt:4;	/* dscriotor type */
#else
	u32 dt:4;	/* dscriotor type */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 msc:8;	/* mac status code */
	u32 ps:2;	/* padding selection */
	u32 ei:1;	/* error indication */
	u32 tr:1;	/* truncation indication */
	u32 ds:12;	/* descriptor size */
#endif
	u32 dptr;	/* descpriptor pointer */
};

struct ravb_ex_rxdesc {
#if defined(__LITTLE_ENDIAN)
	u32 ds:12;	/* descriptor size */
	u32 ei:1;	/* error indication */
	u32 ps:2;	/* padding selection */
	u32 tr:1;	/* truncation indication */
	u32 msc:8;	/* mac status code */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 dt:4;	/* dscriotor type */
#else
	u32 dt:4;	/* dscriotor type */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 msc:8;	/* mac status code */
	u32 ps:2;	/* padding selection */
	u32 ei:1;	/* error indication */
	u32 tr:1;	/* truncation indication */
	u32 ds:12;	/* descriptor size */
#endif
	u32 dptr;	/* descpriptor pointer */
	u32 ts_n;	/* timestampe nsec */
	u32 ts_sl;	/* timestamp low */
#if defined(__LITTLE_ENDIAN)
	u32 res:16;	/* reserved bit */
	u32 ts_sh:16;	/* timestamp high */
#else
	u32 ts_sh:16;	/* timestamp high */
	u32 res:16;	/* reserved bit */
#endif
};

struct ravb_txdesc {
#if defined(__LITTLE_ENDIAN)
	u32 ds:12;	/* descriptor size */
	u32 tag:10;	/* frame tag */
	u32 tsr:1;	/* timestamp storeage request */
	u32 msc:1;	/* mac status storeage request */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 dt:4;	/* dscriotor type */
#else
	u32 dt:4;	/* dscriotor type */
	u32 die:4;	/* descriptor interrupt enable */
			/* 0:disable, other:enable */
	u32 msc:1;	/* mac status storeage request */
	u32 tsr:1;	/* timestamp storeage request */
	u32 tag:10;	/* frame tag */
	u32 ds:12;	/* descriptor size */
#endif
	u32 dptr;	/* descpriptor pointer */
};

#define DBAT_ENTRY_NUM	(22)
#define RX_QUEUE_OFFSET	(4)
#define NUM_RX_QUEUE	(2)
#define NUM_TX_QUEUE	(2)
enum RAVB_QUEUE {
	RAVB_BE = 0,	/* Best Effort Queue */
	RAVB_NC,	/* Network Control Queue */
};

struct ravb_tstamp_skb {
	struct sk_buff *skb;
	u16 tag;
	struct list_head list;
};

struct ravb_ptp_perout {
	u32 target;
	u32 period;
};

struct ravb_ptp {
	struct net_device *ndev;
	spinlock_t lock; /* protects regs */
	struct ptp_clock *clock;
	struct ptp_clock_info caps;
	u32 default_addend;
	u32 current_addend;
	int irq;
	int extts[1];
	struct ravb_ptp_perout perout[1];
	int avtp_capture_gpio;
};

/* This structure is used by each CPU dependency handling. */
struct ravb_cpu_data {
	/* optional functions */
	void (*set_duplex)(struct net_device *ndev);
	void (*set_rate)(struct net_device *ndev);

	/* mandatory initialize value */
	int register_type;

	/* optional initialize value */
	unsigned long ecsr_value;
	unsigned long ecsipr_value;

	/* clock initialise value */
	unsigned long csel_value;
	unsigned long gti_value;

	/* hardware features */
	unsigned long irq_flags; /* IRQ configuration flags */
	unsigned no_psr:1;	/* EtherC DO NOT have PSR */
	unsigned apr:1;		/* EtherC have APR */
	unsigned mpr:1;		/* EtherC have MPR */
	unsigned tpauser:1;	/* EtherC have TPAUSER */
	unsigned bculr:1;	/* EtherC have BCULR */
	unsigned hw_swap:1;	/* AVB-DMAC have BOC bit in CCC */
	unsigned need_txalign:1;	/* tx skb needs particular alignment */
};

struct ravb_private {
	struct net_device *ndev;
	struct platform_device *pdev;
	struct ravb_cpu_data *cd;
	const u16 *reg_offset;
	void __iomem *addr;
	u32 num_rx_ring[NUM_RX_QUEUE];
	u32 num_tx_ring[NUM_TX_QUEUE];
	u32 desc_bat_sz;
	dma_addr_t rx_desc_dma[NUM_RX_QUEUE];
	dma_addr_t tx_desc_dma[NUM_TX_QUEUE];
	dma_addr_t desc_bat_dma;
	struct ravb_ex_rxdesc *rx_ring[NUM_RX_QUEUE];
	struct ravb_txdesc *tx_ring[NUM_TX_QUEUE];
	struct ravb_desc *desc_bat;
	struct sk_buff **rx_skbuff[NUM_RX_QUEUE];
	struct sk_buff **tx_skbuff[NUM_TX_QUEUE];
	struct sk_buff **tx_skbuff_aligned[NUM_TX_QUEUE];
	u32 rx_over_errors;
	u32 rx_fifo_errors;
	struct net_device_stats stats[NUM_RX_QUEUE];
	u32 tstamp_tx_ctrl;
	u32 tstamp_rx_ctrl;
	struct list_head ts_skb_head;
	u32 ts_skb_tag;
	struct ravb_ptp *ptp;
	spinlock_t lock;		/* Register access lock */
	u32 cur_rx[NUM_RX_QUEUE];	/* Consumer ring indices */
	u32 dirty_rx[NUM_RX_QUEUE];	/* Producer ring indices */
	u32 cur_tx[NUM_TX_QUEUE];
	u32 dirty_tx[NUM_TX_QUEUE];
	u32 rx_buf_sz;			/* Based on MTU+slack. */
	int edmac_endian;
	struct napi_struct napi;
	/* MII transceiver section. */
	u32 phy_id;			/* PHY ID */
	int phy_irq;			/* PHY irq number */
	int *phy_ignore_pins;		/* Unused pins */
	int num_phy_ignore_pins;	/* Number of unused pins */
	struct mii_bus *mii_bus;	/* MDIO bus control */
	struct phy_device *phydev;	/* PHY device control */
	int link;
	phy_interface_t phy_interface;
	int msg_enable;
	int speed;
	int duplex;

	unsigned no_ether_link:1;
	unsigned ether_link_active_low:1;
};

#define ravb_plat_data	sh_eth_plat_data

static inline void ravb_soft_swap(char *src, int len)
{
#ifdef __LITTLE_ENDIAN__
	u32 *p = (u32 *)src;
	u32 *maxp;
	maxp = p + ((len + sizeof(u32) - 1) / sizeof(u32));

	for (; p < maxp; p++)
		*p = swab32(*p);
#endif
}

static inline void ravb_write(struct net_device *ndev, unsigned long data,
				int enum_index)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	iowrite32(data, mdp->addr + mdp->reg_offset[enum_index]);
}

static inline unsigned long ravb_read(struct net_device *ndev,
					int enum_index)
{
	struct ravb_private *mdp = netdev_priv(ndev);

	return ioread32(mdp->addr + mdp->reg_offset[enum_index]);
}

#if defined(CONFIG_RAVB_PTP_1588_CLOCK)
extern int ravb_ptp_init(struct net_device *ndev,
			 struct platform_device *pdev);
extern int ravb_ptp_stop(struct net_device *ndev,
			 struct platform_device *pdev);
#else
static inline int ravb_ptp_init(struct net_device *ndev,
			 struct platform_device *pdev)
{
	return 0;
}
static inline int ravb_ptp_stop(struct net_device *ndev,
			 struct platform_device *pdev)
{
	return 0;
}
#endif

#endif	/* #ifndef __RAVB_H__ */
