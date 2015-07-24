/*
 * drivers/gpu/drm/rcar-du/vspd_drv_private.h
 *     This header file is R-Car VSPD register.
 *
 * Copyright (C) 2015 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _VSPD_PRIVATE_H_
#define _VSPD_PRIVATE_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/sched.h>

/* deveice/driver name */
#define DEVNAME "vspd"
#define DRVNAME DEVNAME
#define DEVNUM DEVNAME

/* General Control Registers */
#define VI6_CMDn(index) (0x0000 + (index * 0x04))
#define VI6_CLK_DCSWT 0x0018
#define VI6_SRESET 0x0028
#define VI6_STATUS 0x0038
 #define VI6_STATUS_SYSn_ACT(index) (1 << (8 + index))
#define VI6_WPFn_IRQ_ENB(index) (0x0048 + (index * 0x0c))
#define VI6_WPFn_IRQ_STA(index) (0x004c + (index * 0x0c))
 #define VI6_WPFn_IRQ_FRE		(1 << 0)
 #define VI6_WPFn_IRQ_DFE		(1 << 1)

#define VI6_DISP_IRQ_ENB		0x0078
#define VI6_DISP_IRQ_STA		0x007C
 #define VI6_DISP_IRQ_STA_DST		(1 << 8)
 #define VI6_DISP_IRQ_STA_MAE		(1 << 5)
 #define VI6_DISP_IRQ_STA_LNE(index)	(1 << index)


/* Display List Control */
#define VI6_DL_CTRL 0x0100
#define VI6_DL_HDR_ADDRn(index) (0x0104 + (index * 0x04))
 #define VI6_DL_CTRL_AR_WAIT (256 << 16)
 #define VI6_DL_CTRL_DL_ENABLE (1 << 12 | 1 << 8 | 1 << 4 | 1 << 0)
 #define VI6_DL_CTRL_CFM0 (1 << 2)
 #define VI6_DL_CTRL_NH0 (1 << 1)
#define VI6_DL_SWAP 0x0114
 #define VI6_DL_SWAP_LWS (1 << 2) /* LWORD swap */
 #define VI6_DL_SWAP_WDS (1 << 1) /* WORD swap */
 #define VI6_DL_SWAP_BTS (1 << 0) /* BYTE swap */
#define VI6_DL_BODY_SIZE 0x0120
 #define VI6_DL_BODY_SIZE_UPD0 (1 << 24)


/* Read Pixel Formatter */
#define VI6_RPFn_SRC_BSIZE(index)	(0x0300 + (0x100 * index))
#define VI6_RPFn_SRC_ESIZE(index)	(0x0304 + (0x100 * index))
#define VI6_RPFn_INFMT(index)		(0x0308 + (0x100 * index))
 #define VI6_RPFn_INFMT_CSC		(1 << 8)
 #define VI6_RPFn_INFMT_BT601		(0 << 9)
 #define VI6_RPFn_INFMT_BT709		(2 << 9)
#define VI6_RPFn_DSWAP(index)		(0x030c + (0x100 * index))
#define VI6_RPFn_LOC(index)		(0x0310 + (0x100 * index))
#define VI6_RPFn_ALPH_SEL(index)	(0x0314 + (0x100 * index))
#define VI6_RPFn_VRTCOL_SET(index)	(0x0318 + (0x100 * index))
#define VI6_RPFn_MSKCTRL(index)		(0x031c + (0x100 * index))
#define VI6_RPFn_MSKSET0(index)		(0x0320 + (0x100 * index))
#define VI6_RPFn_MSKSET1(index)		(0x0324 + (0x100 * index))
#define VI6_RPFn_CKEY_CTRL(index)	(0x0328 + (0x100 * index))
#define VI6_RPFn_CKEY_SET0(index)	(0x032c + (0x100 * index))
#define VI6_RPFn_CKEY_SET1(index)	(0x0330 + (0x100 * index))
#define VI6_RPFn_SRCM_PSTRIDE(index)	(0x0334 + (0x100 * index))
#define VI6_RPFn_SRCM_ASTRIDE(index)	(0x0338 + (0x100 * index))
#define VI6_RPFn_SRCM_ADDR_Y(index)	(0x033c + (0x100 * index))
#define VI6_RPFn_SRCM_ADDR_C0(index)	(0x0340 + (0x100 * index))
#define VI6_RPFn_SRCM_ADDR_C1(index)	(0x0344 + (0x100 * index))
#define VI6_RPFn_SRCM_ADDR_AI(index)	(0x0348 + (0x100 * index))

/* Write Pixel Formatter */
#define VI6_WPFn_SRCRPF(index)		(0x1000 + (0x100 * index))
#define VI6_WPFn_HSZCLIP(index)		(0x1004 + (0x100 * index))
#define VI6_WPFn_VSZCLIP(index)		(0x1008 + (0x100 * index))
#define VI6_WPFn_OUTFMT(index)		(0x100c + (0x100 * index))
 #define VI6_WPFn_OUTFMT_PDV(val)	(val << 24)
 #define VI6_WPFn_OUTFMT_PXA		(1 << 23)
 #define VI6_WPFn_OUTFMT_FLP		(1 << 16)
 #define VI6_WPFn_OUTFMT_SPYCS		(1 << 15)
 #define VI6_WPFn_OUTFMT_SPUVS		(1 << 14)
 #define VI6_WPFn_OUTFMT_DITH		(3 << 12)
 #define VI6_WPFn_OUTFMT_WRTM_BT601	(0 << 9)
 #define VI6_WPFn_OUTFMT_WRTM_BT709	(2 << 9)
 #define VI6_WPFn_OUTFMT_CSC		(1 << 8)
#define VI6_WPFn_DSWAP(index)		(0x1010 + (0x100 * index))
#define VI6_WPFn_RNDCTRL(index)		(0x1014 + (0x100 * index))
 #define VI6_WPFn_RNDCTRL_CBRM		(1 << 28)
 #define VI6_WPFn_RNDCTRL_ABRM_LOW	(0 << 24)
 #define VI6_WPFn_RNDCTRL_ABRM_ROUND	(1 << 24)
 #define VI6_WPFn_RNDCTRL_ABRM_CMP	(2 << 24)
 #define VI6_WPFn_RNDCTRL_ATHRESH(val)	(1 << 16)
 #define VI6_WPFn_RNDCTRL_CLMD_NO	(0 << 12)
 #define VI6_WPFn_RNDCTRL_CLMD_YCBCR1	(1 << 12)
 #define VI6_WPFn_RNDCTRL_CLMD_YCBCR2	(2 << 12)
#define VI6_WPFn_DSTM_STRIDE_Y(index)	(0x101c + (0x100 * index))
#define VI6_WPFn_DSTM_STRIDE_C(index)	(0x1020 + (0x100 * index))
#define VI6_WPFn_DSTM_ADDR_Y(index)	(0x1024 + (0x100 * index))
#define VI6_WPFn_DSTM_ADDR_C0(index)	(0x1028 + (0x100 * index))
#define VI6_WPFn_DSTM_ADDR_C1(index)	(0x102c + (0x100 * index))

/* WPF0 control register */
#define VI6_WPF0_WRBCK_CTRL		0x1034

/* Data Path Router */
#define VI6_DPR_RPFn_ROUTE(index)	(0x2000 + (index * 0x04))
#define VI6_DPR_WPFn_FPORCH(index)	(0x2014 + (index * 0x04))
#define VI6_DPR_SRU_ROUTE		0x2024
#define VI6_DPR_UDSn_ROUTE(index)	(0x2028 + (index * 0x04))
#define VI6_DPR_LUT_ROUTE		0x203c
#define VI6_DPR_CLU_ROUTE		0x2040
#define VI6_DPR_HST_ROUTE		0x2044
#define VI6_DPR_HSI_ROUTE		0x2048
#define VI6_DPR_BRU_ROUTE		0x204c
 #define VI6_DPR_ROUTE_TO_SRU		16
 #define VI6_DPR_ROUTE_TO_UDSn(index)	(17 + index)
 #define VI6_DPR_ROUTE_TO_LUT		22
 #define VI6_DPR_ROUTE_TO_CLU		29
 #define VI6_DPR_ROUTE_TO_HST		30
 #define VI6_DPR_ROUTE_TO_HSI		31
 #define VI6_DPR_ROUTE_TO_BRU(index)	(23 + index)
 #define VI6_DPR_ROUTE_TO_WPF(index)	(56 + index)
 #define VI6_DPR_ROUTE_DIS_CONN		63

/* Blend ROP Unit */
#define VI6_BRU_INCTRL			0x2c00
#define VI6_BRU_VIRRPF_SIZE		0x2c04
#define VI6_BRU_VIRRPF_LOC		0x2c08
#define VI6_BRU_VIRRPF_COL		0x2c0c
/* index:0-3(a-d) */
#define VI6_BRUm_CTRL(index)		(0x2c10 + (index * 0x08))

 #define VI6_BRUm_CTRL_RBC		(1 << 31)
 /* index:0-4, 0-3:BRU, 4:Vir RPF */
 /* only Blend/ROP Unit-A */
 #define VI6_BRUm_CTRL_DSTSEL(index)	(index << 20)
 /* index:0-4, 0-3:BRU, 4:Vir RPF */
 /* whth out Blend/ROP Unit-B */
 #define VI6_BRUm_CTRL_SRCSEL(index)	(index << 16)

/* index:0-3(a-d) */
#define VI6_BRUm_BLD(index)		(0x2c14 + (index * 0x08))
 #define VI6_BRUm_BLD_CBES		(1 << 31)
 #define VI6_BRUm_BLD_CCMDX_DSTA	(0 << 28)
 #define VI6_BRUm_BLD_CCMDX_255_DSTA	(1 << 28)
 #define VI6_BRUm_BLD_CCMDX_SRCA	(2 << 28)
 #define VI6_BRUm_BLD_CCMDX_255_SRCA	(3 << 28)
 #define VI6_BRUm_BLD_CCMDX_COEFX	(4 << 28)
 #define VI6_BRUm_BLD_CCMDY_DSTA	(0 << 24)
 #define VI6_BRUm_BLD_CCMDY_255_DSTA	(1 << 24)
 #define VI6_BRUm_BLD_CCMDY_SRCA	(2 << 24)
 #define VI6_BRUm_BLD_CCMDY_255_SRCA	(3 << 24)
 #define VI6_BRUm_BLD_CCMDY_COEFY	(4 << 24)
 #define VI6_BRUm_BLD_ABES		(1 << 23)
 #define VI6_BRUm_BLD_ACMDX_DSTA	(0 << 20)
 #define VI6_BRUm_BLD_ACMDX_255_DSTA	(1 << 20)
 #define VI6_BRUm_BLD_ACMDX_SRCA	(2 << 20)
 #define VI6_BRUm_BLD_ACMDX_255_SRCA	(3 << 20)
 #define VI6_BRUm_BLD_ACMDX_COEFX	(4 << 20)
 #define VI6_BRUm_BLD_ACMDY_DSTA	(0 << 16)
 #define VI6_BRUm_BLD_ACMDY_255_DSTA	(1 << 16)
 #define VI6_BRUm_BLD_ACMDY_SRCA	(2 << 16)
 #define VI6_BRUm_BLD_ACMDY_255_SRCA	(3 << 16)
 #define VI6_BRUm_BLD_ACMDY_COEFY	(4 << 16)
 #define VI6_BRUm_BLD_COEFX(a)		((a & 0xff) << 8)
 #define VI6_BRUm_BLD_COEFY(a)		((a & 0xff) << 0)
#define VI6_BRU_ROP		0x2c30

/* Up Down Scaler */
/* index:0-2 */
#define VI6_UDSn_CTRL(index)		(0x2300 + (index * 0x100))
 #define VI6_UDSn_CTRL_AMD		(1 << 30)
 #define VI6_UDSn_CTRL_AON		(1 << 25)
 #define VI6_UDSn_CTRL_BC		(1 << 20)
 #define VI6_UDSn_CTRL_TDIPC		(1 << 1)
#define VI6_UDSn_SCALE(index)		(0x2304 + (index * 0x100))
#define VI6_UDSn_ALPTH(index)		(0x2308 + (index * 0x100))
#define VI6_UDSn_ALPVAL(index)		(0x230c + (index * 0x100))
#define VI6_UDSn_PASS_BWIDTH(index)	(0x2310 + (index * 0x100))
#define VI6_UDSn_IPC(index)		(0x2318 + (index * 0x100))
 #define VI6_UDSn_IPC_FIELD_TOP		(0 << 27)
 #define VI6_UDSn_IPC_FIELD_BOTTOM	(1 << 27)
#define VI6_UDSn_CLIP_SIZE(index)	(0x2324 + (index * 0x100))
#define VI6_UDSn_FILL_COLOR(index)	(0x2328 + (index * 0x100))


#define VI6_UDSn_VSZCLIP(index)		(0x2320 + (index * 0x100))
 #define VI6_UDSn_VSZCLIP_VCEN		(1 << 28)
 #define VI6_UDSn_VSZCLIP_OFST(val)	((val) << 16)
 #define VI6_UDSn_VSZCLIP_SIZE(val)	((val) << 0)

/* LIF control registers */
#define VI6_LIF_CTRL	0x3b00
#define VI6_LIF_CSBTH	0x3b04

/* DL mode */
enum {
	DL_MODE_SINGLE = 0,
	DL_MODE_MANUAL_REPEAT,
	DL_MODE_AUTO_REPEAT,
	DL_MODE_HEADER_LESS_MANUAL_REPEAT,
	DL_MODE_HEADER_LESS_AUTO_REPEAT,
};

/* DL frame stat */
#define DL_IRQ_FRAME_END	0x01
#define DL_IRQ_UPDATE_FRAME	0x02


/* write back stat */
enum {
	WB_STAT_NONE = 0,
	WB_STAT_CATP_ENABLE,
	WB_STAT_CATP_START,
	WB_STAT_CATP_DONE,
};

#define USE_WPF 0

#define DISPLAY_LIST_NUM 8
#define DISPLAY_LIST_BODY_NUM 8


/* display list header format */
struct display_header {
	/* zero_bits:29 + num_list_minus1:3 */
	unsigned long num_list_minus1;
	struct {
		/* zero_bits:15 + num_bytes:17 */
		unsigned long num_bytes;
		unsigned long plist;
	} display_list[DISPLAY_LIST_BODY_NUM];
	unsigned long pnext_header;
	/* zero_bits:30 + current_frame_int_enable:1 + */
	/* next_frame_auto_start:1 */
	unsigned long int_auto;


	/* if (VI6_DL_EXT_CTRL.EXT) */
	unsigned long zero_bits;
	/* zero_bits:6 + pre_ext_dl_exec:1 + */
	/* post_ext_dl_exec:1 + zero_bits:8 + pre_ext_dl_num_cmd:16 */
	unsigned long pre_post_num;
	unsigned long pre_ext_dl_plist;
	/* zero_bits:16 + post_ext_dl_num_cmd:16 */
	unsigned long post_ext_dl_num_cmd;
	unsigned long post_ext_dl_p_list;
};

/* display list body format */
struct display_list { /* 8byte align */
	unsigned long set_address; /* resistor address */
	unsigned long set_data; /* resistor data */
};

enum {
	DL_LIST_OFFSET_RPF0 = 0,
	DL_LIST_OFFSET_RPF1,
	DL_LIST_OFFSET_RPF2,
	DL_LIST_OFFSET_RPF3,
	DL_LIST_OFFSET_RPF4,
	DL_LIST_OFFSET_CTRL,
	DL_LIST_OFFSET_LUT,
	DL_LIST_OFFSET_CLUT,
};

struct dl_head;
struct dl_body;

struct dl_body {
	int size;
	int use;
	int reg_count;
	dma_addr_t paddr;
	struct display_list *dlist;
	unsigned long dlist_offset;
	int flag;
	struct dl_body *next;
};

struct dl_head {
	int size;
	int use;
	dma_addr_t paddr;
	struct display_header *dheader;
	unsigned long dheader_offset;
	struct dl_body *dl_body_list[DISPLAY_LIST_BODY_NUM];
	int flag;
	struct dl_head *next;
};

struct dl_memory {
	int size;
	dma_addr_t paddr;
	void *vaddr;
	int flag;
	spinlock_t lock;

	/* header mode */
	struct dl_head head[DISPLAY_LIST_NUM];
	struct dl_body body[DISPLAY_LIST_NUM][DISPLAY_LIST_BODY_NUM];
	struct dl_head *active_header;
	struct dl_head *next_header;

	/* header less mode */
	struct dl_body single_body[DISPLAY_LIST_NUM];
	struct dl_body *active_body;
	struct dl_body *active_body_now;
	struct dl_body *active_body_next_set;
	struct dl_body *next_body;
	struct dl_body *pending_body;
};

struct vspd_resource {
	char *name;
	unsigned long addr;
	unsigned long size;
	void __iomem *base;
	int irq;
	char *clk_name;
	struct clk *clk;
};

struct write_back {
	int stat;
	wait_queue_head_t wb_wait;
	int count;
};

struct vspd_private_data {
	struct device *dev;
	struct vspd_resource res;

	bool active;
	bool lif;
	wait_queue_head_t event_wait;
	spinlock_t lock;
	struct mutex mutex_lock;

	struct dl_memory *dlmemory;

	void (*callback)(void *data);
	void *callback_data;

	struct write_back wb;
};

int vspd_dl_create(struct vspd_private_data *vdata, struct device *dev);
void vspd_dl_destroy(struct vspd_private_data *vdata);
void vspd_dl_reset(struct vspd_private_data *vdata);

int vspd_dl_start(struct vspd_private_data *vdata,
			void *dl, int num, int dl_mode);
int vspd_dl_swap(struct vspd_private_data *vdata, void *dl, int num);

int vspd_dl_irq_frame_end(struct vspd_private_data *vdata);
int vspd_dl_irq_dl_frame_end(struct vspd_private_data *vdata);
int vspd_dl_irq_display_start(struct vspd_private_data *vdata);

struct dl_head *vspd_dl_get_header(struct dl_memory *dlmemory);
struct dl_body *vspd_dl_get_body(struct dl_memory *dlmemory, int module);
struct dl_body *vspd_dl_get_single_body(struct dl_memory *dlmemory);
int vspd_dl_set_body(struct dl_head *head, struct dl_body *body, int module);
void vspd_dl_free_header(struct dl_memory *dlmemory, struct dl_head *head);
void vspd_dl_free_body(struct dl_memory *dlmemory, struct dl_body *body);

int vsp_du_if_du_info(void *callback_data);

#define vspd_err(vdata, fmt, ...) \
	dev_err(vdata->dev, "[Error] %s : " fmt, __func__, ##__VA_ARGS__)
#define vspd_warn(vdata, fmt, ...) \
	dev_warn(vdata->dev, "[Warn] %s : " fmt, __func__, ##__VA_ARGS__)
#define vspd_info(vdata, fmt, ...) \
	dev_info(vdata->dev, "[Info] %s : " fmt, __func__, ##__VA_ARGS__)

static inline unsigned long vspd_read(struct vspd_private_data *vdata,
				unsigned long reg)
{
	return ioread32(vdata->res.base + reg);
}

static inline void vspd_write(struct vspd_private_data *vdata,
			      unsigned long reg, unsigned long data)
{
	iowrite32(data, vdata->res.base + reg);
}


#endif /* _VSPD_PRIVATE_H_ */
