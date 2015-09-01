/*
 * drivers/gpu/drm/rcar-du/vspd_drv_main.c
 *     This file is R-Car VSPD main driver.
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

#include "vspd_ioctl.h"
#include "vspd_drv_private.h"
#include "vspd_drv.h"

/* VSP SYS */
#define VSPS_MEM_ADDR	0xfe928000
#define VSPS_MEM_SIZE	0x00008000
#define VSPS_IRQ_NUM	(267 + 32)
#define VSPS_CLK_NAME	"vsps"

/* VSP DU0 */
#define VSPD0_NAME	"vspd0"
#define VSPD0_MEM_ADDR	0xFE930000
#define VSPD0_MEM_SIZE	0x00008000
#define VSPD0_IRQ_NUM	(246 + 32)
#define VSPD0_CLK_NAME	"vsp1-du0"

/* VSP DU1 */
#define VSPD1_NAME	"vspd1"
#define VSPD1_MEM_ADDR	0xFE938000
#define VSPD1_MEM_SIZE	0x00008000
#define VSPD1_IRQ_NUM	(247 + 32)
#define VSPD1_CLK_NAME	"vsp1-du1"

/* structure */
struct vspd_drvdata {
	/* for sysfs */
	struct cdev   cdev;
	struct class  *class;
	dev_t         devt;
	struct device *dev;
};
struct vspd_drvdata *p_vdrv;

enum {
	UDS_IPCONV_NONE,	/* not scaling */
	UDS_IPCONV_1,	/* 3 <= scale */
	UDS_IPCONV_2,	/* 1 < scale < 3 */
	UDS_IPCONV_3,	/* scale = 1 */
	UDS_IPCONV_4,	/* 1/2 < scale < 1 */
};

struct vspd_pre_check {
	struct vspd_image *master_layer;
	int scaling;
};

static void vspd_write_back_done(struct vspd_private_data *vdata);

#define ALIGN_ROUND_DOWN(X, Y)  ((X) & ~((Y)-1))

static inline int is_scaling(struct vspd_image *image)
{
	if ((image->crop.width != image->dist.width) ||
	    (image->crop.height != image->dist.height))
		return 1;

	return 0;
}


static unsigned long vspd_get_bwidth(unsigned long ratio)
{
	unsigned long bwidth;
	unsigned long mant, frac;

	mant = (ratio & 0xF000) >> 12;
	frac = ratio & 0x0FFF;
	if (mant)
		bwidth = 64 * 4096 * mant / (4096 * mant + frac);
	else
		bwidth = 64;

	return bwidth;
}

static unsigned long vspd_compute_ratio(unsigned int input, unsigned int output)
{
#ifdef DISABLE_UDS_CTRL_AMD
	return (input - 1) * 4096 / (output - 1);
#else
	if (output > input)
		return input * 4096 / output;
	else
		return (input - 1) * 4096 / (output - 1);
#endif
}

static int is_yuv_fmt(int fmt)
{
	switch (fmt) {
	case VSPD_FMT_YUV422I_UYVY:
	case VSPD_FMT_YUV422I_YUYV:
	case VSPD_FMT_YUV420SP_NV12:
	case VSPD_FMT_YUV420SP_NV21:
	case VSPD_FMT_YUV422SP_NV16:
	case VSPD_FMT_YUV422SP_NV61:
	case VSPD_FMT_YUV420P_YU12:
		return 1;
	}

	return 0;
}

static int get_fmt_val(struct vspd_image *image, unsigned long *fmt)
{
	unsigned long _fmt = 0;

	*fmt = 0;

	switch (image->format) {
	case VSPD_FMT_XRGB8888:
	case VSPD_FMT_ARGB8888:
		_fmt = 0x13;
		break;

	case VSPD_FMT_RGB888:
		_fmt = 0x15;
		break;

	case VSPD_FMT_ARGB1555:
	case VSPD_FMT_XRGB1555:
		_fmt = 0x1B;
		break;

	case VSPD_FMT_RGB565:
		_fmt = 0x06;
		break;

	case VSPD_FMT_YUV422I_YUYV:
		_fmt = 1 << 15; /* SPYCS */
	case VSPD_FMT_YUV422I_UYVY:
		_fmt |= 0x47;
		break;

	case VSPD_FMT_YUV422SP_NV61:
		_fmt = 1 << 14; /* SPUVS */
	case VSPD_FMT_YUV422SP_NV16:
		_fmt |= 0x41;
		break;

	case VSPD_FMT_YUV420SP_NV21:
		_fmt = 1 << 14; /* SPUVS */
	case VSPD_FMT_YUV420SP_NV12:
		_fmt |= 0x42;
		break;

	case VSPD_FMT_YUV420P_YU12:
		_fmt = 0x4c;
		break;

	default:
		return -EINVAL;
	}

	*fmt = _fmt;
	return 0;
}

void vspd_set_callback(struct vspd_private_data *vdata,
				void (*callback)(void *data),
				void *callback_data)
{
	vdata->callback = callback;
	vdata->callback_data = callback_data;
}

static irqreturn_t vspd_irq_handler(int irq, void *data)
{
	struct vspd_private_data *vdata = (struct vspd_private_data *)data;
	unsigned long stat;
	int frame_stat = 0;

	stat = vspd_read(vdata, VI6_WPFn_IRQ_STA(USE_WPF)) &
		(VI6_WPFn_IRQ_DFE | VI6_WPFn_IRQ_FRE);
	if (stat) {
		vspd_write(vdata, VI6_WPFn_IRQ_STA(USE_WPF), ~stat);

		if (stat & VI6_WPFn_IRQ_DFE)
			frame_stat = vspd_dl_irq_dl_frame_end(vdata);
		else
			frame_stat = vspd_dl_irq_frame_end(vdata);
	}

	stat = vspd_read(vdata, VI6_DISP_IRQ_STA) & VI6_DISP_IRQ_STA_DST;
	if (stat) {
		vspd_write(vdata, VI6_DISP_IRQ_STA, ~stat);
		vspd_dl_irq_display_start(vdata);
	}

	if (frame_stat & DL_IRQ_UPDATE_FRAME)
		wake_up_interruptible(&vdata->event_wait);

	if (frame_stat & DL_IRQ_FRAME_END) {
		vspd_write_back_done(vdata);

		if (vdata->callback)
			vdata->callback(vdata->callback_data);
	}

	return IRQ_HANDLED;
}


int vspd_wpf_reset(struct vspd_private_data *vdata)
{
	int time;
	unsigned long status;

	vdata->active = false;
	vdata->lif = false;

	vspd_write(vdata, VI6_WPFn_IRQ_ENB(USE_WPF), 0);
	vspd_write(vdata, VI6_DISP_IRQ_ENB, 0);

	/* reset WFP */
	status = vspd_read(vdata, VI6_STATUS);
	if (status & VI6_STATUS_SYSn_ACT(USE_WPF)) {
		vspd_write(vdata, VI6_SRESET, 1 << USE_WPF);
		for (time = 0; time < 10; time++) {
			status = vspd_read(vdata, VI6_STATUS);
			if (!(status & VI6_STATUS_SYSn_ACT(USE_WPF)))
				break;

			usleep_range(1000, 2000);
		}

		if (time >= 10)
			vspd_warn(vdata, "WPF reset timeout\n");
	}

	vspd_write(vdata, VI6_WPFn_IRQ_STA(USE_WPF), 0);
	vspd_write(vdata, VI6_DISP_IRQ_STA, 0);

	vspd_dl_reset(vdata);

	vdata->wb.stat = WB_STAT_NONE;
	wake_up_interruptible(&vdata->wb.wb_wait);

	return 0;
}

int vspd_device_init(struct vspd_private_data *vdata)
{
	int i;

	vspd_write(vdata, VI6_DL_CTRL, 0);

	vspd_wpf_reset(vdata);

	/* modules disconnect */
	for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++)
		vspd_write(vdata, VI6_DPR_RPFn_ROUTE(i),
				VI6_DPR_ROUTE_DIS_CONN);

	for (i = 0; i < VSPD_SCALING_IMAGE_NUM; i++)
		vspd_write(vdata, VI6_DPR_UDSn_ROUTE(i),
				VI6_DPR_ROUTE_DIS_CONN);

	vspd_write(vdata, VI6_DPR_SRU_ROUTE, VI6_DPR_ROUTE_DIS_CONN);
	vspd_write(vdata, VI6_DPR_LUT_ROUTE, VI6_DPR_ROUTE_DIS_CONN);
	vspd_write(vdata, VI6_DPR_CLU_ROUTE, VI6_DPR_ROUTE_DIS_CONN);
	vspd_write(vdata, VI6_DPR_HST_ROUTE, VI6_DPR_ROUTE_DIS_CONN);
	vspd_write(vdata, VI6_DPR_HSI_ROUTE, VI6_DPR_ROUTE_DIS_CONN);
	vspd_write(vdata, VI6_DPR_BRU_ROUTE, VI6_DPR_ROUTE_DIS_CONN);

	vspd_write(vdata, VI6_LIF_CTRL, 0);

	return 0;
}

/* use Display List */

static inline void vspd_set_dl(struct vspd_private_data *vdata,
			  unsigned long reg, unsigned long data,
			  struct dl_body *body)
{
	body->dlist[body->reg_count].set_address = reg;
	body->dlist[body->reg_count].set_data = data;
	body->reg_count++;
}

struct vpsd_rpf_regs {
	unsigned long size;
	unsigned long infmt;
	unsigned long loc;
	unsigned long alph_sel;
	unsigned long laya;
	unsigned long pstride;
	unsigned long addr_y;
	unsigned long addr_c0;
	unsigned long addr_c1;
};

static int check_rpf_param(struct vspd_image *in, struct vspd_image *out,
			   struct vpsd_rpf_regs *regs,
			   struct vspd_pre_check *pre)
{
	unsigned long crop_width, crop_height, crop_x, crop_y;
	unsigned long addr_y, addr_c0, addr_c1;
	unsigned long stride_y, stride_c;
	unsigned long dist_x, dist_y;
	unsigned long ip_flag = in->flag & VSPD_FLAG_IP_MASK;

	if (get_fmt_val(in, &regs->infmt))
		return -EINVAL;

	if (is_yuv_fmt(in->format) != is_yuv_fmt(out->format)) {
		/* Color Space Conversion enable */
		regs->infmt |= VI6_RPFn_INFMT_CSC;
		if (in->flag & VSPD_FLAG_COLOR_CONV_MASK) {
			/* BT.709 YCbCr [16,235/240] <-> RGB [0,255] */
			regs->infmt |= VI6_RPFn_INFMT_BT709;
		} else {
			/* BT.601 YCbCr [16,235/240] <-> RGB [0,255] */
			regs->infmt |= VI6_RPFn_INFMT_BT601;
		}
	}

	regs->addr_y = 0;
	regs->addr_c0 = 0;
	regs->addr_c1 = 0;

	crop_width = in->crop.width;
	crop_x = in->crop.x;
	dist_x = in->dist.x;

	/* If you want to scaling & IP conversion, it is done by the UDS. */
	/* In this case, RPF will read progressive image. */
	switch (in->flag & VSPD_FLAG_IP_MASK) {
	case VSPD_FLAG_INTERLACE_TOP:
		if (!is_scaling(in)) {
			addr_y = in->addr_y;
			addr_c0 = in->addr_c0;
			addr_c1 = in->addr_c1;
			stride_y = in->stride_y * 2;
			stride_c = in->stride_c * 2;
			crop_height = in->crop.height / 2;
			crop_y = in->crop.y / 2;
			dist_y = in->dist.y / 2;
		} else {
			addr_y = in->addr_y;
			addr_c0 = in->addr_c0;
			addr_c1 = in->addr_c1;
			stride_y = in->stride_y;
			stride_c = in->stride_c;
			crop_height = in->crop.height;
			crop_y = in->crop.y;
			dist_y = in->dist.y / 2;
		}
		break;

	case VSPD_FLAG_INTERLACE_BOTTOM:
		if (!is_scaling(in)) {
			addr_y = in->addr_y + in->stride_y;
			addr_c0 = in->addr_c0 + in->stride_c;
			addr_c1 = in->addr_c1 + in->stride_c;
			stride_y = in->stride_y * 2;
			stride_c = in->stride_c * 2;
			crop_height = in->crop.height / 2;
			crop_y = in->crop.y / 2;
			dist_y = in->dist.y / 2;
		} else {
			addr_y = in->addr_y;
			addr_c0 = in->addr_c0;
			addr_c1 = in->addr_c1;
			stride_y = in->stride_y;
			stride_c = in->stride_c;
			crop_height = in->crop.height;
			crop_y = in->crop.y;
			dist_y = in->dist.y / 2;
		}
		break;

	case VSPD_FLAG_PROGRESSIVE:
		addr_y = in->addr_y;
		addr_c0 = in->addr_c0;
		addr_c1 = in->addr_c1;
		stride_y = in->stride_y;
		stride_c = in->stride_c;
		crop_height = in->crop.height;
		crop_y = in->crop.y;
		dist_y = in->dist.y;
		break;
	}

	switch (in->format) {
	case VSPD_FMT_XRGB8888:
	case VSPD_FMT_ARGB8888:
		regs->addr_y = addr_y + 4 * crop_x + stride_y * crop_y;
		break;

	case VSPD_FMT_RGB888:
		regs->addr_y = addr_y + 3 * crop_x + stride_y * crop_y;
		break;

	case VSPD_FMT_XRGB1555:
	case VSPD_FMT_ARGB1555:
	case VSPD_FMT_RGB565:
		regs->addr_y = addr_y + 2 * crop_x + stride_y * crop_y;
		break;

	case VSPD_FMT_YUV422I_UYVY:
	case VSPD_FMT_YUV422I_YUYV:
		crop_width = ALIGN_ROUND_DOWN(crop_width, 2);
		crop_x = ALIGN_ROUND_DOWN(crop_x, 2);
		regs->addr_y = addr_y + 2 * crop_x + stride_y * crop_y;
		break;

	case VSPD_FMT_YUV420SP_NV12:
	case VSPD_FMT_YUV420SP_NV21:
		crop_width = ALIGN_ROUND_DOWN(crop_width, 2);
		crop_height = ALIGN_ROUND_DOWN(crop_height, 2);
		crop_x = ALIGN_ROUND_DOWN(in->crop.x, 2);
		crop_y = ALIGN_ROUND_DOWN(in->crop.y, 2);
		regs->addr_y = addr_y + crop_x + stride_y * crop_y;
		regs->addr_c0 = addr_c0 + crop_x + stride_c * crop_y / 2;
		break;

	case VSPD_FMT_YUV422SP_NV61:
	case VSPD_FMT_YUV422SP_NV16:
		crop_width = ALIGN_ROUND_DOWN(crop_width, 2);
		crop_x = ALIGN_ROUND_DOWN(in->crop.x, 2);
		regs->addr_y = addr_y + crop_x + stride_y * crop_y;
		regs->addr_c0 = addr_c0 + crop_x + stride_c * crop_y;
		break;

	case VSPD_FMT_YUV420P_YU12:
		crop_width = ALIGN_ROUND_DOWN(crop_width, 2);
		crop_height = ALIGN_ROUND_DOWN(crop_height, 2);
		regs->addr_y = addr_y + crop_x + stride_y * crop_y;
		regs->addr_c0 = addr_c0 + crop_x / 2 + stride_c * crop_y / 2;
		regs->addr_c1 = addr_c1 + crop_x / 2 + stride_c * crop_y / 2;
		break;
	default:
		return -EINVAL;
	}

	regs->size = (crop_width << 16) | crop_height;
	if (pre->master_layer == in) {
		regs->loc = 0;
	} else {
		if ((pre->scaling == UDS_IPCONV_2) &&
		    (ip_flag == VSPD_FLAG_INTERLACE_BOTTOM))
			regs->loc = (dist_x << 16) | (dist_y + 1);
		else
			regs->loc = (dist_x << 16) | dist_y;
	}
	regs->pstride = (stride_y << 16) | stride_c;

	switch (in->format) {
	case VSPD_FMT_ARGB1555:
		if (CONFIG_DRM_RCAR_VSP_ALPHA_BIT_ARGB1555 == 1)
			regs->alph_sel = (2 << 28) | (1 << 18) |
					 (0xFF << 8) | (in->alpha & 0xFF);
		else
			regs->alph_sel = (2 << 28) | (1 << 18) |
					 ((in->alpha & 0xFF) << 8) | 0xFF;
		regs->laya = 0;
		break;
	case VSPD_FMT_ARGB8888:
		regs->alph_sel = (1 << 18);
		regs->laya = 0;
		break;
	default:
		regs->alph_sel = (4 << 28) | (1 << 18);
		regs->laya = (in->alpha & 0xFF) << 24;
		break;
	}

	return 0;
}

static int vspd_rpf_to_dl_core(struct vspd_private_data *vdata,
			struct vspd_image *in, struct vspd_image *out,
			int rpf_index, struct dl_body *body,
			struct vspd_pre_check *pre)
{
	struct vpsd_rpf_regs regs;

	if (!in->enable)
		return -1;

	if (check_rpf_param(in, out, &regs, pre)) {
		vspd_err(vdata, "rpf %d parameter error\n", rpf_index);
		return -1;
	}

	/* input image size (width/height) */
	vspd_set_dl(vdata, VI6_RPFn_SRC_BSIZE(rpf_index), regs.size, body);
	vspd_set_dl(vdata, VI6_RPFn_SRC_ESIZE(rpf_index), regs.size, body);

	/* input image format */
	vspd_set_dl(vdata, VI6_RPFn_INFMT(rpf_index), regs.infmt, body);

	/* input data swap */
	vspd_set_dl(vdata, VI6_RPFn_DSWAP(rpf_index), in->swap, body);

	/* input image position (master layer = 0.0) */
	vspd_set_dl(vdata, VI6_RPFn_LOC(rpf_index), regs.loc, body);

	/* alpha plane */
	vspd_set_dl(vdata, VI6_RPFn_ALPH_SEL(rpf_index), regs.alph_sel, body);
	vspd_set_dl(vdata, VI6_RPFn_VRTCOL_SET(rpf_index), regs.laya, body);

	vspd_set_dl(vdata, VI6_RPFn_MSKCTRL(rpf_index), 0, body);
	vspd_set_dl(vdata, VI6_RPFn_MSKSET0(rpf_index), 0, body);
	vspd_set_dl(vdata, VI6_RPFn_MSKSET1(rpf_index), 0, body);

	/* key color */
	vspd_set_dl(vdata, VI6_RPFn_CKEY_CTRL(rpf_index), 0, body);
	vspd_set_dl(vdata, VI6_RPFn_CKEY_SET0(rpf_index), 0, body);
	vspd_set_dl(vdata, VI6_RPFn_CKEY_SET1(rpf_index), 0, body);

	/* input image stride */
	vspd_set_dl(vdata, VI6_RPFn_SRCM_PSTRIDE(rpf_index),
			regs.pstride, body);

	/* input image alpha plane stride */
	vspd_set_dl(vdata, VI6_RPFn_SRCM_ASTRIDE(rpf_index), 0, body);

	/* input image address */
	vspd_set_dl(vdata, VI6_RPFn_SRCM_ADDR_Y(rpf_index),
			regs.addr_y, body);
	vspd_set_dl(vdata, VI6_RPFn_SRCM_ADDR_C0(rpf_index),
			regs.addr_c0, body);
	vspd_set_dl(vdata, VI6_RPFn_SRCM_ADDR_C1(rpf_index),
			regs.addr_c1, body);

	/* input image alpha plane address */
	vspd_set_dl(vdata, VI6_RPFn_SRCM_ADDR_AI(rpf_index), 0, body);

	return 0;
}

int vspd_rpfs_to_dl(struct vspd_private_data *vdata, struct vspd_blend *blend,
		    struct dl_head *head, struct vspd_pre_check *pre)
{
	int i;
	int rpf_count = 0;
	struct dl_body *body;

	for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];
		struct vspd_image *out = &blend->out;
		body = vspd_dl_get_body(vdata->dlmemory,
				DL_LIST_OFFSET_RPF0 + rpf_count);

		if (vspd_rpf_to_dl_core(vdata, in, out, rpf_count,
					body, pre)) {
			vspd_dl_free_body(vdata->dlmemory, body);
			body = NULL;
			continue;
		}

		vspd_dl_set_body(head, body, DL_LIST_OFFSET_RPF0 + rpf_count);

		rpf_count++;
	}

	return 0;
}

struct vpsd_wpf_regs {
	unsigned long outfmt;
	unsigned long srcrpf;
	unsigned long h_crop;
	unsigned long v_crop;
};

static int check_wpf_param(struct vspd_blend *blend, struct vpsd_wpf_regs *regs,
			   struct vspd_pre_check *pre)
{
	struct vspd_image *out = &blend->out;
	struct vspd_image *master_layer = NULL;
	unsigned long rpf_count = 0;
	int i;
	unsigned long ip_flag = out->flag & VSPD_FLAG_IP_MASK;

	if (get_fmt_val(out, &regs->outfmt))
		return -EINVAL;

	/* WPF alpha use VI6_WPFn_OUTFMT.PDV */
	regs->outfmt |= VI6_WPFn_OUTFMT_PDV((out->alpha & 0xFF));

	if ((255 < out->crop.x) || (255 < out->crop.y))
		return -EINVAL;

	regs->srcrpf = 0;
	for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];

		if (in->enable) {
			if (master_layer == NULL) {
				/* rpf master layer */
				regs->srcrpf |= 2 << (2 * rpf_count);
				master_layer = in;
			} else {
				/* rpf sub layer */
				regs->srcrpf |= 1 << (2 * rpf_count);
			}

			rpf_count++;
		}
	}

	if (((out->crop.x + out->crop.width) > master_layer->dist.width) ||
	    ((out->crop.y + out->crop.height) > master_layer->dist.height))
		return -EINVAL;

	if ((out->crop.x + out->crop.width) != master_layer->dist.width)
		regs->h_crop = (1 << 28) | (out->crop.x << 16) |
				out->crop.width;
	else
		regs->h_crop = 0;


	if (ip_flag) {
		regs->v_crop = (1 << 28) | (out->crop.height / 2);
		if ((pre->scaling == UDS_IPCONV_2) &&
		    (ip_flag == VSPD_FLAG_INTERLACE_BOTTOM))
			regs->v_crop |= ((out->crop.y + 1) << 16);
		else
			regs->v_crop |= (out->crop.y << 16);
	} else if ((out->crop.y + out->crop.height) !=
		    master_layer->dist.height)
		regs->v_crop = (1 << 28) | (out->crop.y << 16) |
				out->crop.height;
	else
		regs->v_crop = 0;

	return 0;
}

int vspd_wpf_to_dl(struct vspd_private_data *vdata,
			  struct vspd_blend *blend,
			  struct dl_body *body,
			  struct vspd_pre_check *pre)
{
	struct vspd_image *out = &blend->out;
	struct vpsd_wpf_regs regs;

	if (check_wpf_param(blend, &regs, pre)) {
			vspd_err(vdata, "parameter error\n");
		return -EINVAL;
	}

	/* select input rpf */
	vspd_set_dl(vdata, VI6_WPFn_SRCRPF(USE_WPF), regs.srcrpf, body);

	/* crop horizontal input image */
	vspd_set_dl(vdata, VI6_WPFn_HSZCLIP(USE_WPF), regs.h_crop, body);

	/* crop vertical input image */
	vspd_set_dl(vdata, VI6_WPFn_VSZCLIP(USE_WPF), regs.v_crop, body);

	/* output image format */
	vspd_set_dl(vdata, VI6_WPFn_OUTFMT(USE_WPF), regs.outfmt, body);

	/* output data swap */
	vspd_set_dl(vdata, VI6_WPFn_DSWAP(USE_WPF), out->swap, body);

	/* rounding control */
	/* lower-order bits are truncated */
	vspd_set_dl(vdata, VI6_WPFn_RNDCTRL(USE_WPF),
			VI6_WPFn_RNDCTRL_ABRM_LOW, body);

	/* output image stride */
	vspd_set_dl(vdata, VI6_WPFn_DSTM_STRIDE_Y(USE_WPF),
			out->stride_y, body);
	vspd_set_dl(vdata, VI6_WPFn_DSTM_STRIDE_C(USE_WPF),
			out->stride_c, body);

	/* output image address */
	vspd_set_dl(vdata, VI6_WPFn_DSTM_ADDR_Y(USE_WPF),
		    out->addr_y, body);
	vspd_set_dl(vdata, VI6_WPFn_DSTM_ADDR_C0(USE_WPF),
		    out->addr_c0, body);
	vspd_set_dl(vdata, VI6_WPFn_DSTM_ADDR_C1(USE_WPF),
		    out->addr_c1, body);

	if (vdata->wb.stat == WB_STAT_CATP_ENABLE)
		vspd_set_dl(vdata, VI6_WPF0_WRBCK_CTRL, 1, body);
	else
		vspd_set_dl(vdata, VI6_WPF0_WRBCK_CTRL, 0, body);

	return 0;
}


int vspd_bru_to_dl(struct vspd_private_data *vdata,
			  struct vspd_blend *blend,
			  struct dl_body *body)
{
	int i;
	int rpf_count = 0;

	for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];

		if (in->enable)
			rpf_count++;
	}

	if (rpf_count == 1)
		return 0;

	vspd_set_dl(vdata, VI6_BRU_INCTRL, 0, body);

	/* vspd_set_dl(vdata, VI6_BRU_VIRRPF_SIZE, 0, body); */
	/* vspd_set_dl(vdata, VI6_BRU_VIRRPF_LOC, 0, body); */
	/* vspd_set_dl(vdata, VI6_BRU_VIRRPF_COL, 0, body); */

	rpf_count = 0;
	for (i = 0; i < VSPD_BLEND_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];
		unsigned long bru_ctrl;
		if (in->enable) {
			bru_ctrl = VI6_BRUm_CTRL_RBC; /* Blend operation */

			if (rpf_count == 0) {
				/* Blend/ROP UNIT A DST is Virtual RPF */
				bru_ctrl |= VI6_BRUm_CTRL_DSTSEL(4);
			}
			if (rpf_count != 1) {
				/* Blend/ROP UNIT B SRC is ROP unit */
				bru_ctrl |= VI6_BRUm_CTRL_SRCSEL(rpf_count);
			}
			vspd_set_dl(vdata, VI6_BRUm_CTRL(rpf_count),
					bru_ctrl, body);
			if (++rpf_count > VSPD_BLEND_IMAGE_NUM)
				break;
		}
	}
	/* ROP UNIT SRC is BRUnit1 */
	vspd_set_dl(vdata, VI6_BRU_ROP, 1 << 20, body);

	for (i = rpf_count; i < VSPD_BLEND_IMAGE_NUM; i++)
		vspd_set_dl(vdata, VI6_BRUm_CTRL(i), 0, body); /* disable */

	for (i = 0; i < VSPD_BLEND_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];
		unsigned long bld = VI6_BRUm_BLD_CCMDX_255_SRCA |
				    VI6_BRUm_BLD_ACMDX_255_SRCA |
				    VI6_BRUm_BLD_ACMDY_COEFY |
				    VI6_BRUm_BLD_COEFY(0xff);

		bld |= (in->flag & VSPD_FLAG_PREMUL_ALPH) ?
			VI6_BRUm_BLD_CCMDY_COEFY : VI6_BRUm_BLD_CCMDY_SRCA;

		vspd_set_dl(vdata, VI6_BRUm_BLD(i), bld, body);
	}

	return 0;
}

struct vpsd_uds_regs {
	unsigned long ctrl;
	unsigned long scale;
	unsigned long pass_bwidth;
	unsigned long clip_size;
	unsigned long vphase;
	unsigned long vszclip;
};

static int check_uds_param(struct vspd_image *in, struct vpsd_uds_regs *regs)
{
	unsigned long ratio_h, ratio_v;
	unsigned long bwidth_h, bwidth_v;
	unsigned long ctrl;
	unsigned long vphase = 0;
	unsigned long clip_size;
	unsigned long vszclip = 0;
	unsigned long ip_flag = in->flag & VSPD_FLAG_IP_MASK;

	ratio_h = vspd_compute_ratio(in->crop.width, in->dist.width);
	ratio_v = vspd_compute_ratio(in->crop.height, in->dist.height);

	bwidth_h = vspd_get_bwidth(ratio_h);
	bwidth_v = vspd_get_bwidth(ratio_v);

	ctrl = (in->alpha ? VI6_UDSn_CTRL_AON : 0) | VI6_UDSn_CTRL_AMD;

	/* scaling & progressive -> interlace */
	if (ip_flag != VSPD_FLAG_PROGRESSIVE) {
		unsigned long vfrac = ratio_v & 0x0FFF;

		ctrl |= VI6_UDSn_CTRL_TDIPC;

		if (ratio_v <= 1365) {
			/* 3 <= scale */
			vszclip = 0;

			if (ip_flag == VSPD_FLAG_INTERLACE_TOP) {
				vphase = (((4096 - vfrac) / 2) & 0xFFF) << 16;
			} else {
				vphase = (((4096 - (vfrac * 3)) / 2) & 0xFFF)
						<< 16;
			}
			vphase |= ((4096 - vfrac) / 2) & 0xFFF;

			clip_size = (in->dist.height / 2);
		} else if (ratio_v < 4096) {
			/* 1 < scale < 3 */
			vszclip = 0;

			if (ip_flag == VSPD_FLAG_INTERLACE_TOP) {
				clip_size = (in->dist.height / 2);
				vphase = (((4096 - vfrac) / 2) & 0xFFF) << 16;
			} else {
				clip_size = (in->dist.height / 2 + 1);
				vphase = (((4096 + vfrac) / 2) & 0xFFF) << 16;
			}
			vphase |= ((4096 - vfrac) / 2) & 0xFFF;
		} else if (ratio_v == 4096) {
			if (ip_flag == VSPD_FLAG_INTERLACE_TOP) {
				vszclip = 0;
			} else {
				vszclip = VI6_UDSn_VSZCLIP_VCEN |
				  VI6_UDSn_VSZCLIP_OFST(1) |
				  VI6_UDSn_VSZCLIP_SIZE(in->crop.height - 1);
			}
			vphase = 0;
			clip_size = (in->dist.height / 2);
		} else if (ratio_v < 8192) {
			/* 1/2 < scale < 1 */

			if (ip_flag == VSPD_FLAG_INTERLACE_TOP) {
				vszclip = 0;
				vphase = 0;
			} else {
				vszclip = VI6_UDSn_VSZCLIP_VCEN |
				  VI6_UDSn_VSZCLIP_OFST(2) |
				  VI6_UDSn_VSZCLIP_SIZE(in->crop.height - 2);
				vphase = (4096 - vfrac) << 16;
			}
			clip_size = (in->dist.height / 2);

			/* If you want to reducing in the */
			/* IP conversion, set MultiTap.   */
			/* Can not be reducing if you use */
			/* Bilinear/NearestNeighbor.      */
			ctrl |= VI6_UDSn_CTRL_BC;
		} else {
			return -EINVAL;
		}

		clip_size |= (in->dist.width << 16);
		ratio_v *= 2;
	} else {
		clip_size = (in->dist.width << 16) | in->dist.height;
	}

	if ((in->alpha < 0xff) && (ratio_h >= 8192 || ratio_v >= 8192))
		;
	else
		ctrl |= VI6_UDSn_CTRL_BC;


	regs->ctrl = ctrl;
	regs->scale = (ratio_h << 16) | ratio_v;
	regs->pass_bwidth = (bwidth_h << 16) | bwidth_v;
	regs->clip_size = clip_size;
	regs->vphase = vphase;
	regs->vszclip = vszclip;

	return 0;
}

int vspd_uds_to_dl(struct vspd_private_data *vdata,
			  struct vspd_blend *blend,
			  struct dl_body *body)
{
	int i;
	int scal_count = 0;
	struct vpsd_uds_regs regs;

	for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];

		if (!in->enable)
			continue;

		if (is_scaling(in)) {
			if (scal_count >= VSPD_SCALING_IMAGE_NUM) {
				vspd_err(vdata, "Scaling layer too many\n");
				return -EBUSY;
			}
		} else {
			continue;
		}

		if (check_uds_param(in, &regs)) {
				vspd_err(vdata, "parameter error\n");
			return -EINVAL;
		}

		vspd_set_dl(vdata, VI6_UDSn_CTRL(scal_count),
				regs.ctrl, body);
		vspd_set_dl(vdata, VI6_UDSn_SCALE(scal_count),
				regs.scale, body);
		vspd_set_dl(vdata, VI6_UDSn_PASS_BWIDTH(scal_count),
				regs.pass_bwidth, body);
		vspd_set_dl(vdata, VI6_UDSn_CLIP_SIZE(scal_count),
				regs.clip_size, body);
		vspd_set_dl(vdata, VI6_UDSn_IPC(scal_count),
				regs.vphase, body);
		vspd_set_dl(vdata, VI6_UDSn_VSZCLIP(scal_count),
				regs.vszclip, body);

		scal_count++;
	}

	return 0;
}


/* set dpr */
int vspd_dpr_to_dl(struct vspd_private_data *vdata,
			  struct vspd_blend *blend,
			  struct dl_body *body)
{
	int i;
	int rpf_count = 0;
	int scal_count = 0;
	unsigned long bru_connect_to = VI6_DPR_ROUTE_DIS_CONN;

	for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];
		if (in->enable) {
			if (is_scaling(in))
				scal_count++;

			rpf_count++;
		}
	}

	if (scal_count > VSPD_SCALING_IMAGE_NUM)
		return -EINVAL;

	if (rpf_count == 0) {
		vspd_err(vdata, "no input layer\n");
		return -EINVAL;
	} else if (rpf_count == 1) {
		if (scal_count) {
			/* rpf0 -> uds -> wpf0 */
			vspd_set_dl(vdata, VI6_DPR_RPFn_ROUTE(0),
					VI6_DPR_ROUTE_TO_UDSn(0), body);
			vspd_set_dl(vdata, VI6_DPR_UDSn_ROUTE(0),
					VI6_DPR_ROUTE_TO_WPF(USE_WPF), body);
		} else {
			/* rpf0 -> wpf0 */
			vspd_set_dl(vdata, VI6_DPR_RPFn_ROUTE(0),
					VI6_DPR_ROUTE_TO_WPF(USE_WPF), body);
		}
	} else if (rpf_count <= VSPD_INPUT_IMAGE_NUM) {
		scal_count = 0;
		rpf_count = 0;

		/* rpfn -> bru */
		for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++) {
			struct vspd_image *in = &blend->in[i];

			if (!in->enable)
				continue;

			if (is_scaling(in) &&
			    (scal_count < VSPD_SCALING_IMAGE_NUM)) {
				/* RPFn -> UDSn -> BRU */
				vspd_set_dl(vdata,
					VI6_DPR_RPFn_ROUTE(rpf_count),
					VI6_DPR_ROUTE_TO_UDSn(scal_count),
					body);
				vspd_set_dl(vdata,
					VI6_DPR_UDSn_ROUTE(scal_count),
					VI6_DPR_ROUTE_TO_BRU(rpf_count),
					body);
				scal_count++;
			} else {
				/* RPFn -> BRUn */
				vspd_set_dl(vdata,
					VI6_DPR_RPFn_ROUTE(rpf_count),
					VI6_DPR_ROUTE_TO_BRU(rpf_count),
					body);
			}
			rpf_count++;
		}
		/* BRU -> WPF0 */
		bru_connect_to = VI6_DPR_ROUTE_TO_WPF(USE_WPF);
	} else {
		vspd_err(vdata, "too many input layer\n");
		return -EINVAL;
	}

	/* set BRU connect to ... */
	vspd_set_dl(vdata, VI6_DPR_BRU_ROUTE, bru_connect_to, body);

	/* disable */
	for (i = rpf_count; i < VSPD_INPUT_IMAGE_NUM; i++)
		vspd_set_dl(vdata, VI6_DPR_RPFn_ROUTE(i),
			VI6_DPR_ROUTE_DIS_CONN, body);
	for (i = scal_count; i < VSPD_SCALING_IMAGE_NUM; i++)
		vspd_set_dl(vdata, VI6_DPR_UDSn_ROUTE(i),
			VI6_DPR_ROUTE_DIS_CONN, body);

	vspd_set_dl(vdata, VI6_DPR_WPFn_FPORCH(USE_WPF), 5 << 8, body);

	return 0;
}

/* connect rcar-du */
int vspd_lif_set(struct vspd_private_data *vdata, struct vspd_blend *blend)
{
	struct vspd_image *out = &blend->out;
	unsigned long obth;

	if (vdata->active || !vdata->lif)
		return 0;

#define OBTH 128
#define HBTH 1536
#define LBTH 1520

	if (out->flag & VSPD_FLAG_IP_MASK)
		obth = ((out->width + 1) / 2) * (out->height / 2) - 4;
	else
		obth = ((out->width + 1) / 2) * out->height - 4;
	if (obth >= OBTH)
		obth = OBTH;

	vspd_write(vdata, VI6_LIF_CTRL,
		(obth << 16) | (1 << 1) | (1 << 0));
	vspd_write(vdata, VI6_LIF_CSBTH,
		(HBTH << 16) | (LBTH << 0));

	return 0;
}

int vspd_run_dl(struct vspd_private_data *vdata,
		void *dl, int num, int dl_mode, int use_sync)
{
	unsigned long flags;
	int ret = 0;

	if (vdata->active) {
		spin_lock_irqsave(&vdata->lock, flags);
		vspd_dl_swap(vdata, dl, num);
		spin_unlock_irqrestore(&vdata->lock, flags);
	} else {
		vspd_lif_set(vdata, (struct vspd_blend *)dl);

		vspd_write(vdata, VI6_WPFn_IRQ_STA(USE_WPF), 0);
		vspd_write(vdata, VI6_DISP_IRQ_STA, 0);

		spin_lock_irqsave(&vdata->lock, flags);
		vdata->active = true;
		vspd_dl_start(vdata, dl, num, dl_mode);
		spin_unlock_irqrestore(&vdata->lock, flags);
	}

	return ret;
}


static int pre_check(struct vspd_blend *blend, struct vspd_pre_check *pre)
{
	int i;

	pre->master_layer = NULL;
	pre->scaling = UDS_IPCONV_NONE;

	for (i = 0; i < VSPD_INPUT_IMAGE_NUM; i++) {
		struct vspd_image *in = &blend->in[i];
		unsigned long ratio_v;

		if (!in->enable)
			continue;

		/* check scaling param. */
		if (!(in->flag & VSPD_FLAG_VIRTUAL) && is_scaling(in) &&
		    (in->flag & VSPD_FLAG_IP_MASK)) {

			ratio_v = vspd_compute_ratio(in->crop.height,
						     in->dist.height);

			if (ratio_v <= 1365) { /* 3 <= scale */
				pre->scaling = UDS_IPCONV_1;
			} else if (ratio_v < 4096) { /* 1 < scale < 3 */
				/* In this case, the up/down layer must be */
				/* a master layer. */
				if (pre->master_layer != NULL)
					return -EINVAL;
				pre->scaling = UDS_IPCONV_2;
			} else if (ratio_v == 4096) { /* scale = 1 */
				pre->scaling = UDS_IPCONV_3;
			} else if (ratio_v < 8192) { /* 1/2 < scale < 1 */
				pre->scaling = UDS_IPCONV_4;
			} else {
				return -EINVAL;
			}
		}

		if (pre->master_layer == NULL)
			pre->master_layer = in;
	}

	return 0;
}


int vspd_dl_mem_copy(struct vspd_private_data *vdata, struct vspd_blend *blend)
{
	struct dl_head *head = vspd_dl_get_header(vdata->dlmemory);
	struct dl_body *body = vspd_dl_get_body(vdata->dlmemory,
					DL_LIST_OFFSET_CTRL);
	struct dl_head *heads[1];
	struct vspd_pre_check pre;

	if (head == NULL) {
		vspd_err(vdata, "Display List header busy\n");
		return -ENOMEM;
	}

	if (body == NULL) {
		vspd_err(vdata, "Display List body busy\n");
		return -ENOMEM;
	}

	pre_check(blend, &pre);

	vspd_rpfs_to_dl(vdata, blend, head, &pre);

	vspd_wpf_to_dl(vdata, blend, body, &pre);
	vspd_bru_to_dl(vdata, blend, body);
	vspd_uds_to_dl(vdata, blend, body);
	vspd_dpr_to_dl(vdata, blend, body);
	vspd_dl_set_body(head, body, DL_LIST_OFFSET_CTRL);

	heads[0] = head;
	vspd_run_dl(vdata, heads, 1, DL_MODE_SINGLE, VSPD_FENCE_NONE);

	return 0;
}


static int vspd_dl_output_du_head_mode(struct vspd_private_data *vdata,
		struct vspd_blend blends[], int num, int use_sync)
{
	int i;
	struct dl_body *body;
	struct dl_head *heads[DISPLAY_LIST_NUM];
	struct vspd_pre_check pre;

	for (i = 0; i < num; i++) {
		if (pre_check(&blends[i], &pre))
			goto error_get_dl_body;

		body = vspd_dl_get_body(vdata->dlmemory, DL_LIST_OFFSET_CTRL);
		if (body == NULL) {
			vspd_err(vdata, "Display List body busy\n");
			goto error_get_dl_body;
		}

		heads[i] = vspd_dl_get_header(vdata->dlmemory);
		if (heads[i] == NULL) {
			vspd_err(vdata, "Display List header busy\n");
			goto error_get_dl_head;
		}

		if (vspd_rpfs_to_dl(vdata, &blends[i], heads[i], &pre))
			goto error_set_dl_param;

		if (vspd_wpf_to_dl(vdata, &blends[i], body, &pre))
			goto error_set_dl_param;

		if (vspd_bru_to_dl(vdata, &blends[i], body))
			goto error_set_dl_param;

		if (vspd_uds_to_dl(vdata, &blends[i], body))
			goto error_set_dl_param;

		if (vspd_dpr_to_dl(vdata, &blends[i], body))
			goto error_set_dl_param;

		vspd_dl_set_body(heads[i], body, DL_LIST_OFFSET_CTRL);
	}

	return vspd_run_dl(vdata, heads, num,
				DL_MODE_MANUAL_REPEAT, use_sync);


error_set_dl_param:
	vspd_dl_free_header(vdata->dlmemory, heads[i]);
error_get_dl_head:
	vspd_dl_free_body(vdata->dlmemory, body);
error_get_dl_body:
	i--;
	for (; i >= 0; i--)
		vspd_dl_free_header(vdata->dlmemory, heads[i]);

	return -ENOMEM;
}

static int vspd_dl_output_du_head_less(struct vspd_private_data *vdata,
		struct vspd_blend blends[], int num, int use_sync)
{
	int i, j;
	int ret;
	int rpf_count;
	struct dl_body *bodies[DISPLAY_LIST_NUM];
	struct vspd_pre_check pre;

	for (i = 0; i < num; i++) {
		ret = pre_check(&blends[i], &pre);
		if (ret)
			goto error_get_dl_body;

		bodies[i] = vspd_dl_get_single_body(vdata->dlmemory);
		if (bodies[i] == NULL) {
			vspd_err(vdata, "Display List body busy\n");
			ret = -ENOMEM;
			goto error_get_dl_body;
		}

		rpf_count = 0;
		for (j = 0; j < VSPD_INPUT_IMAGE_NUM; j++) {
			struct vspd_image *in = &blends[i].in[j];
			struct vspd_image *out = &blends[i].out;
			if (vspd_rpf_to_dl_core(vdata, in, out, rpf_count,
				  bodies[i], &pre)) {
				continue;
			}
			rpf_count++;
		}

		ret = vspd_wpf_to_dl(vdata, &blends[i], bodies[i], &pre);
		if (ret)
			goto error_set_dl_param;

		ret = vspd_bru_to_dl(vdata, &blends[i], bodies[i]);
		if (ret)
			goto error_set_dl_param;

		ret = vspd_uds_to_dl(vdata, &blends[i], bodies[i]);
		if (ret)
			goto error_set_dl_param;

		ret = vspd_dpr_to_dl(vdata, &blends[i], bodies[i]);
		if (ret)
			goto error_set_dl_param;
	}

	return vspd_run_dl(vdata, bodies, num,
		DL_MODE_HEADER_LESS_AUTO_REPEAT, use_sync);

error_set_dl_param:
	vspd_dl_free_body(vdata->dlmemory, bodies[i]);
error_get_dl_body:
	i--;
	for (; i >= 0; i--)
		vspd_dl_free_body(vdata->dlmemory, bodies[i]);

	return ret;
}

int vspd_dl_output_du(struct vspd_private_data *vdata,
		struct vspd_blend blends[], int num, int use_sync)
{
	int ret;
	int dl_mode = DL_MODE_HEADER_LESS_AUTO_REPEAT;

	mutex_lock(&vdata->mutex_lock);

	if (!vdata->lif)
		vdata->lif = true;

	if (dl_mode == DL_MODE_HEADER_LESS_AUTO_REPEAT)
		ret = vspd_dl_output_du_head_less(vdata,
					blends, num, use_sync);
	else
		ret = vspd_dl_output_du_head_mode(vdata,
					blends, num, use_sync);

	mutex_unlock(&vdata->mutex_lock);

	return ret;
}

/* Please use only in DL headless mode. */
int vspd_dl_output_mute(struct vspd_private_data *vdata,
			unsigned long width, unsigned long height,
			int num, int use_sync)
{
	int i, j;
	struct dl_body *bodies[DISPLAY_LIST_NUM];

	for (i = 0; i < num; i++) {
		bodies[i] = vspd_dl_get_single_body(vdata->dlmemory);
		if (bodies[i] == NULL) {
			vspd_err(vdata, "Display List body busy\n");
			goto error_get_dl_body;
		}

		/* set wpf */
		vspd_set_dl(vdata, VI6_WPFn_SRCRPF(USE_WPF),
			    2 << 28, bodies[i]);
		vspd_set_dl(vdata, VI6_WPFn_HSZCLIP(USE_WPF), 0, bodies[i]);
		vspd_set_dl(vdata, VI6_WPFn_VSZCLIP(USE_WPF), 0, bodies[i]);

		/* set bru */
		vspd_set_dl(vdata, VI6_BRU_INCTRL, 0, bodies[i]);
		vspd_set_dl(vdata, VI6_BRU_VIRRPF_SIZE,
				(width << 16) | (height << 0), bodies[i]);
		vspd_set_dl(vdata, VI6_BRU_VIRRPF_LOC,
				(0 << 16) | (0 << 0), bodies[i]); /* black */
		vspd_set_dl(vdata, VI6_BRU_VIRRPF_COL,
				(0xFF << 24), bodies[i]);
		vspd_set_dl(vdata, VI6_BRUm_CTRL(0),
				VI6_BRUm_CTRL_DSTSEL(4), bodies[i]);
		vspd_set_dl(vdata, VI6_BRUm_BLD(0),
				VI6_BRUm_BLD_CCMDX_255_SRCA |
				VI6_BRUm_BLD_ACMDX_255_SRCA |
				VI6_BRUm_BLD_CCMDY_SRCA |
				VI6_BRUm_BLD_ACMDY_COEFY |
				VI6_BRUm_BLD_COEFY(0xff),
				bodies[i]);
		for (j = 1; j < VSPD_BLEND_IMAGE_NUM; j++)
			vspd_set_dl(vdata, VI6_BRUm_CTRL(j), 0, bodies[i]);

		/* set dpr */
		for (j = 0; j < VSPD_INPUT_IMAGE_NUM; j++)
			vspd_set_dl(vdata, VI6_DPR_RPFn_ROUTE(j),
				    VI6_DPR_ROUTE_DIS_CONN, bodies[i]);
		for (j = 0; j < VSPD_SCALING_IMAGE_NUM; j++)
			vspd_set_dl(vdata, VI6_DPR_UDSn_ROUTE(i),
				    VI6_DPR_ROUTE_DIS_CONN, bodies[i]);
		vspd_set_dl(vdata, VI6_DPR_BRU_ROUTE, 56 + USE_WPF, bodies[i]);
		vspd_set_dl(vdata, VI6_DPR_WPFn_FPORCH(USE_WPF),
			    5 << 8, bodies[i]);
	}

	return vspd_run_dl(vdata, bodies, num,
		DL_MODE_HEADER_LESS_AUTO_REPEAT, use_sync);

error_get_dl_body:
	i--;
	for (; i >= 0; i--)
		vspd_dl_free_body(vdata->dlmemory, bodies[i]);

	return -ENOMEM;
}


static void vspd_write_back_done(struct vspd_private_data *vdata)
{
	/* wait 2vsync */
	switch (vdata->wb.stat) {
	case WB_STAT_CATP_ENABLE:
		/* capture start */
		vdata->wb.stat = WB_STAT_CATP_START;
		wake_up_interruptible(&vdata->wb.wb_wait);
		return;

	case WB_STAT_CATP_START:
		/* capture done */
		vdata->wb.stat = WB_STAT_CATP_DONE;
		wake_up_interruptible(&vdata->wb.wb_wait);
		return;
	}
}

int vspd_dl_write_back_start(struct vspd_private_data *vdata,
		struct vspd_blend blends[], int num)
{
	int ret;

	mutex_lock(&vdata->mutex_lock);

	if (vdata->wb.stat != WB_STAT_NONE) {
		ret = -EBUSY;
		goto end;
	}

	if (!vdata->lif) {
		ret = -ECANCELED;
		goto end;
	}

	if (!vdata->active) {
		ret = -ECANCELED;
		goto end;
	}

	vdata->wb.count = 0;
	vdata->wb.stat = WB_STAT_CATP_ENABLE;

	ret = vspd_dl_output_du_head_less(vdata, blends, num,
						VSPD_FENCE_NONE);

end:
	mutex_unlock(&vdata->mutex_lock);

	return ret;
}


/* Wait write back start or done.                               */
/* Return 0 write back done, 1 write back start, <0 error.      */
/* Write back does not stop even call this function.            */
/* In order to stop the write back, please update display       */
/* (call vspd_dl_output_du) after this function returns 1 or 0. */
int vspd_dl_write_back_wait(struct vspd_private_data *vdata, int done)
{
	int ret;

	if (vdata->wb.stat == WB_STAT_NONE)
		return -EBUSY;

	if (done) {
		ret = wait_event_interruptible((vdata->wb.wb_wait),
				((vdata->wb.stat == WB_STAT_CATP_DONE) ||
				  !vdata->active));
	} else {
		ret = wait_event_interruptible((vdata->wb.wb_wait),
				((vdata->wb.stat >= WB_STAT_CATP_START) ||
				  !vdata->active));
	}

	if (!vdata->active)
		return -ECANCELED;

	if (ret) {
		vdata->wb.stat = WB_STAT_NONE;
	} else if (vdata->wb.stat == WB_STAT_CATP_START) {
		ret = 1;
	} else {
		vdata->wb.stat = WB_STAT_NONE;
		ret = 0;
	}

	return ret;
}

int vspd_check_reg(struct vspd_private_data *vdata, unsigned long arg)
{
	void __iomem *vsp1_base;
	struct clk *vsp1_clk;
	unsigned long addr, size;
	char *clk_name;
	char *vsp_name;
	int i;
	int ret = -EINVAL;

#define  vsp1_read(reg) ioread32(vsp1_base + reg)
#define  vsp1_write(data, reg) iowrite32(data, vsp1_base + reg)

	if (arg == USE_VSPD0) {
		addr = VSPD0_MEM_ADDR;
		size = VSPD0_MEM_SIZE;
		clk_name = VSPD0_CLK_NAME;
		vsp_name = "vspd0";
	} else if (arg == USE_VSPD1) {
		addr = VSPD1_MEM_ADDR;
		size = VSPD1_MEM_SIZE;
		clk_name = VSPD1_CLK_NAME;
		vsp_name = "vspd1";
	} else if (arg == USE_VSPS) {
		addr = VSPS_MEM_ADDR;
		size = VSPS_MEM_SIZE;
		clk_name = VSPS_CLK_NAME;
		vsp_name = "vsps";
	} else {
		return -EINVAL;
	}

	/* get vsp1 clock and enable */
	vsp1_clk = clk_get(NULL, clk_name);
	if (IS_ERR(vsp1_clk)) {
		vspd_err(vdata, "clk_get(%s)\n", clk_name);
		goto error_clk_get;
	} else {
		if (clk_prepare_enable(vsp1_clk) < 0) {
			vspd_err(vdata, "clk_prepare_enable(%s)\n", clk_name);
			goto error_clk_ena;
		}
	}
	vsp1_base = ioremap_nocache(addr, size);
	if (vsp1_base == NULL) {
		vspd_err(vdata,
			"ioremap_nocache(0x%08lx, 0x%08lx)\n", addr, size);
		goto error_ioremap;
	}

#define rcar_dbg_print(fmt, ...) \
	pr_info(fmt, ##__VA_ARGS__)

	rcar_dbg_print("[Info] %s reg\n", vsp_name);
	rcar_dbg_print(" RPF reg\n");
	for (i = 0; i < 5; i++) {
		rcar_dbg_print("  VI6_RPF%d_SRC_BSIZE     : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRC_BSIZE(i)));
		rcar_dbg_print("  VI6_RPF%d_SRC_ESIZE     : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRC_ESIZE(i)));
		rcar_dbg_print("  VI6_RPF%d_INFMT         : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_INFMT(i)));
		rcar_dbg_print("  VI6_RPF%d_DSWAP         : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_DSWAP(i)));
		rcar_dbg_print("  VI6_RPF%d_LOC           : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_LOC(i)));
		rcar_dbg_print("  VI6_RPF%d_ALPH_SEL      : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_ALPH_SEL(i)));
		rcar_dbg_print("  VI6_RPF%d_VRTCOL_SET    : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_VRTCOL_SET(i)));
		rcar_dbg_print("  VI6_RPF%d_MSKCTRL       : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_MSKCTRL(i)));
		rcar_dbg_print("  VI6_RPF%d_MSKSET0       : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_MSKSET0(i)));
		rcar_dbg_print("  VI6_RPF%d_MSKSET1       : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_MSKSET1(i)));
		rcar_dbg_print("  VI6_RPF%d_CKEY_CTRL     : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_CKEY_CTRL(i)));
		rcar_dbg_print("  VI6_RPF%d_CKEY_SET0     : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_CKEY_SET0(i)));
		rcar_dbg_print("  VI6_RPF%d_CKEY_SET1     : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_CKEY_SET1(i)));
		rcar_dbg_print("  VI6_RPF%d_SRCM_PSTRIDE  : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRCM_PSTRIDE(i)));
		rcar_dbg_print("  VI6_RPF%d_SRCM_ASTRIDE  : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRCM_ASTRIDE(i)));
		rcar_dbg_print("  VI6_RPF%d_SRCM_ADDR_Y   : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRCM_ADDR_Y(i)));
		rcar_dbg_print("  VI6_RPF%d_SRCM_ADDR_C0  : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRCM_ADDR_C0(i)));
		rcar_dbg_print("  VI6_RPF%d_SRCM_ADDR_C1  : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRCM_ADDR_C1(i)));
		rcar_dbg_print("  VI6_RPF%d_SRCM_ADDR_AI  : 0x%08x\n",
			i, vsp1_read(VI6_RPFn_SRCM_ADDR_AI(i)));
		rcar_dbg_print("\n");
	}
	rcar_dbg_print("\n");

	rcar_dbg_print(" WPF reg\n");
	for (i = 0; i < 4; i++) {
		rcar_dbg_print("  VI6_WPF%d_SRCRPF        : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_SRCRPF(i)));
		rcar_dbg_print("  VI6_WPF%d_HSZCLIP       : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_HSZCLIP(i)));
		rcar_dbg_print("  VI6_WPF%d_VSZCLIP       : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_VSZCLIP(i)));
		rcar_dbg_print("  VI6_WPF%d_OUTFMT        : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_OUTFMT(i)));
		rcar_dbg_print("  VI6_WPF%d_DSWAP         : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_DSWAP(i)));
		rcar_dbg_print("  VI6_WPF%d_RNDCTRL       : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_RNDCTRL(i)));
		rcar_dbg_print("  VI6_WPF%d_DSTM_STRIDE_Y : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_DSTM_STRIDE_Y(i)));
		rcar_dbg_print("  VI6_WPF%d_DSTM_STRIDE_C : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_DSTM_STRIDE_C(i)));
		rcar_dbg_print("  VI6_WPF%d_DSTM_ADDR_Y   : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_DSTM_ADDR_Y(i)));
		rcar_dbg_print("  VI6_WPF%d_DSTM_ADDR_C0  : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_DSTM_ADDR_C0(i)));
		rcar_dbg_print("  VI6_WPF%d_DSTM_ADDR_C1  : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_DSTM_ADDR_C1(i)));
		rcar_dbg_print("  VI6_WPF%d_IRQ_STA       : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_IRQ_STA(i)));
		rcar_dbg_print("  VI6_WPF%d_IRQ_ENB       : 0x%08x\n",
			i, vsp1_read(VI6_WPFn_IRQ_ENB(i)));
		rcar_dbg_print("\n");
	}
	rcar_dbg_print("  VI6_WPF0_WRBCK_CTRL    : 0x%08x\n",
			vsp1_read(VI6_WPF0_WRBCK_CTRL));
	rcar_dbg_print("\n");

	rcar_dbg_print(" UDS reg\n");
	for (i = 0; i < VSPD_SCALING_IMAGE_NUM; i++) {
		rcar_dbg_print("  VI6_UDS%d_CTRL        : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_CTRL(i)));
		rcar_dbg_print("  VI6_UDS%d_SCALE       : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_SCALE(i)));
		rcar_dbg_print("  VI6_UDS%d_ALPTH       : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_ALPTH(i)));
		rcar_dbg_print("  VI6_UDS%d_ALPVAL      : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_ALPVAL(i)));
		rcar_dbg_print("  VI6_UDS%d_PASS_BWIDTH : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_PASS_BWIDTH(i)));
		rcar_dbg_print("  VI6_UDS%d_IPC         : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_IPC(i)));
		rcar_dbg_print("  VI6_UDS%d_CLIP_SIZE   : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_CLIP_SIZE(i)));
		rcar_dbg_print("  VI6_UDS%d_FILL_COLOR  : 0x%08x\n",
			i, vsp1_read(VI6_UDSn_FILL_COLOR(i)));
		rcar_dbg_print("\n");
	}
	rcar_dbg_print("\n");


	rcar_dbg_print(" DPR reg\n");
	for (i = 0; i < 5; i++)
		rcar_dbg_print("  VI6_DPR_RPF%d_ROUTE  : 0x%08x\n",
			i, vsp1_read(VI6_DPR_RPFn_ROUTE(i)));

	rcar_dbg_print("\n");

	for (i = 0; i < 4; i++)
		rcar_dbg_print("  VI6_DPR_WPF%d_FPORCH  : 0x%08x\n",
			i, vsp1_read(VI6_DPR_WPFn_FPORCH(i)));

	rcar_dbg_print("\n");

	for (i = 0; i < VSPD_SCALING_IMAGE_NUM; i++)
		rcar_dbg_print("  VI6_DPR_UDS%d_ROUTE  : 0x%08x\n",
			i, vsp1_read(VI6_DPR_UDSn_ROUTE(i)));

	rcar_dbg_print("\n");

	rcar_dbg_print("  VI6_DPR_SRU_ROUTE : 0x%08x\n",
		vsp1_read(VI6_DPR_SRU_ROUTE));
	rcar_dbg_print("  VI6_DPR_LUT_ROUTE : 0x%08x\n",
		vsp1_read(VI6_DPR_LUT_ROUTE));
	rcar_dbg_print("  VI6_DPR_CLU_ROUTE : 0x%08x\n",
		vsp1_read(VI6_DPR_CLU_ROUTE));
	rcar_dbg_print("  VI6_DPR_HST_ROUTE : 0x%08x\n",
		vsp1_read(VI6_DPR_HST_ROUTE));
	rcar_dbg_print("  VI6_DPR_HSI_ROUTE : 0x%08x\n",
		vsp1_read(VI6_DPR_HSI_ROUTE));
	rcar_dbg_print("  VI6_DPR_BRU_ROUTE : 0x%08x\n",
		vsp1_read(VI6_DPR_BRU_ROUTE));

	rcar_dbg_print("\n");

	rcar_dbg_print("  VI6_BRU_INCTRL      : 0x%08x\n",
		vsp1_read(VI6_BRU_INCTRL));
	rcar_dbg_print("  VI6_BRU_VIRRPF_SIZE : 0x%08x\n",
		vsp1_read(VI6_BRU_VIRRPF_SIZE));
	rcar_dbg_print("  VI6_BRU_VIRRPF_LOC  : 0x%08x\n",
		vsp1_read(VI6_BRU_VIRRPF_LOC));
	rcar_dbg_print("  VI6_BRU_VIRRPF_COL  : 0x%08x\n",
		vsp1_read(VI6_BRU_VIRRPF_COL));
	for (i = 0; i < 4; i++) {
		rcar_dbg_print("  VI6_BRU%c_CTRL       : 0x%08x\n",
			'A' + i, vsp1_read(VI6_BRUm_CTRL(i)));
		rcar_dbg_print("  VI6_BRU%c_BLD        : 0x%08x\n",
			'A' + i, vsp1_read(VI6_BRUm_BLD(i)));
	}
	rcar_dbg_print("  VI6_BRU_ROP         : 0x%08x\n",
		vsp1_read(VI6_BRU_ROP));

	rcar_dbg_print("\n");

	rcar_dbg_print("  VI6_DL_CTRL         : 0x%08x\n",
		vsp1_read(VI6_DL_CTRL));
	rcar_dbg_print("  VI6_DL_HDR_ADDR0    : 0x%08x\n",
		vsp1_read(VI6_DL_HDR_ADDRn(0)));
	rcar_dbg_print("  VI6_DL_BODY_SIZE    : 0x%08x\n",
		vsp1_read(VI6_DL_BODY_SIZE));
	rcar_dbg_print("  VI6_DL_SWAP         : 0x%08x\n",
		vsp1_read(VI6_DL_SWAP));

	rcar_dbg_print("\n");

	rcar_dbg_print("  VI6_LIF_CTRL     : 0x%08x\n",
		vsp1_read(VI6_LIF_CTRL));
	rcar_dbg_print("  VI6_DISP_IRQ_STA : 0x%08x\n",
		vsp1_read(VI6_DISP_IRQ_STA));
	rcar_dbg_print("  VI6_STATUS       : 0x%08x\n",
		vsp1_read(VI6_STATUS));

	ret = 0;

	iounmap(vsp1_base);
error_ioremap:
	clk_disable_unprepare(vsp1_clk);
error_clk_ena:
	clk_put(vsp1_clk);
error_clk_get:
	return ret;
}

static int get_vspd_resource(struct vspd_private_data *vdata, int use_vsp)
{
	int ret;

	switch (use_vsp) {
	case USE_VSPD0:
		vdata->res.name = VSPD0_NAME;
		vdata->res.addr = VSPD0_MEM_ADDR;
		vdata->res.size = VSPD0_MEM_SIZE;
		vdata->res.irq = VSPD0_IRQ_NUM;
		vdata->res.clk_name = VSPD0_CLK_NAME;
		break;
	case USE_VSPD1:
		vdata->res.name = VSPD1_NAME;
		vdata->res.addr = VSPD1_MEM_ADDR;
		vdata->res.size = VSPD1_MEM_SIZE;
		vdata->res.irq = VSPD1_IRQ_NUM;
		vdata->res.clk_name = VSPD1_CLK_NAME;
		break;
	case USE_VSPS:
		vdata->res.name = "dbg-vsps";
		vdata->res.addr = VSPS_MEM_ADDR;
		vdata->res.size = VSPS_MEM_SIZE;
		vdata->res.irq = VSPS_IRQ_NUM;
		vdata->res.clk_name = VSPS_CLK_NAME;
		break;
	default:
		return -EINVAL;
	}

	/* mapping VSPD */
	vdata->res.base = ioremap_nocache(vdata->res.addr, vdata->res.size);
	if (!vdata->res.base) {
		vspd_err(vdata, "ioremap_nocache(VSPD%d)\n", use_vsp);
		ret = -ENOMEM;
		goto error_ioremap;
	}

	/* get VSPD clock and enable */
	vdata->res.clk = clk_get(NULL, vdata->res.clk_name);
	if (IS_ERR(vdata->res.clk)) {
		vspd_err(vdata, "clk_get(VSPD%d)\n", use_vsp);
		ret = IS_ERR(vdata->res.clk);
		goto error_clk_get;
	}

	ret = clk_prepare_enable(vdata->res.clk);
	if (ret < 0) {
		vspd_err(vdata, "clk_prepare_enable(VSPD%d)\n", use_vsp);
		goto error_clk_enable;
	}

	/* register IRQ */
	ret = request_irq(vdata->res.irq, vspd_irq_handler, IRQF_SHARED,
			vdata->res.name, vdata);
	if (ret < 0) {
		vspd_err(vdata, "request_irq(VSPD%d)\n", use_vsp);
		goto error_request_irq;
	}

	return 0;

error_request_irq:
	clk_disable_unprepare(vdata->res.clk);
error_clk_enable:
	clk_put(vdata->res.clk);
error_clk_get:
	iounmap(vdata->res.base);
error_ioremap:
	return ret;
}

static void free_vspd_resource(struct vspd_private_data *vdata)
{
	free_irq(vdata->res.irq, vdata);
	clk_disable_unprepare(vdata->res.clk);
	clk_put(vdata->res.clk);
	iounmap(vdata->res.base);
}

static int vspd_write_back_init(struct vspd_private_data *vdata)
{
	init_waitqueue_head(&vdata->wb.wb_wait);
	vdata->wb.stat = WB_STAT_NONE;

	return 0;
}

static void vspd_write_back_deinit(struct vspd_private_data *vdata)
{
	/* none */
}


struct vspd_private_data *vspd_init(struct device *dev, int use_vsp)
{
	struct vspd_private_data *vdata;
	int ret;

	vdata = kzalloc(sizeof(*vdata), GFP_KERNEL);
	if (vdata == NULL)
		goto error_kalloc;

	ret = get_vspd_resource(vdata, use_vsp);
	if (ret)
		goto error_get_vspd_resource;

	ret = vspd_dl_create(vdata, dev);
	if (ret)
		goto error_vspd_dl_create;


	ret = vspd_write_back_init(vdata);
	if (ret)
		goto error_vspd_write_back_init;

	init_waitqueue_head(&vdata->event_wait);
	spin_lock_init(&vdata->lock);
	mutex_init(&vdata->mutex_lock);
	vdata->callback = NULL;
	vdata->callback_data = NULL;

	return vdata;

error_vspd_write_back_init:
	vspd_dl_destroy(vdata);
error_vspd_dl_create:
	free_vspd_resource(vdata);
error_get_vspd_resource:
	kfree(vdata);
error_kalloc:
	return NULL;
}

void vspd_deinit(struct vspd_private_data *vdata)
{
	vspd_wpf_reset(vdata);

	vspd_write_back_deinit(vdata);

	vspd_dl_destroy(vdata);

	free_vspd_resource(vdata);

	kfree(vdata);
}

#if 0 /* debug code */
static int vspd_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int vspd_init_ioctl(struct file *file, unsigned long arg)
{
	if (file->private_data != NULL)
		return -EFAULT;

	file->private_data = vspd_init(p_vdrv->dev, arg);

	if (file->private_data == NULL)
		return -EFAULT;

	return 0;
}

static int vspd_deinit_ioctl(struct file *file)
{
	if (file->private_data == NULL)
		return -EFAULT;

	vspd_deinit(file->private_data);
	file->private_data = NULL;

	return 0;
}

static int vspd_release(struct inode *inode, struct file *file)
{
	struct vspd_private_data *vdata = file->private_data;

	if (vdata == NULL)
		return 0;

	vspd_deinit(vdata);
	file->private_data = NULL;

	return 0;
}

static int vspd_dl_mem_copy_ioctl(struct vspd_private_data *vdata,
				unsigned long arg)
{
	int ret;
	struct vspd_blend blend;

	if (copy_from_user(&blend, (void __user *)arg, sizeof(blend))) {
		vspd_err(vdata, "copy_from_user\n");
		ret = -EFAULT;
		return ret;
	}

	return vspd_dl_mem_copy(vdata, &blend);
}

static int vspd_dl_output_du_ioctl(struct vspd_private_data *vdata,
				unsigned long arg)
{
	struct vspd_blend blend;

	if (copy_from_user(&blend, (void __user *)arg, sizeof(blend))) {
		vspd_err(vdata, "copy_from_user\n");
		return -EFAULT;
	}

	return vspd_dl_output_du(vdata, &blend, 1, VSPD_FENCE_NONE);
}

static long vspd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vspd_private_data *vdata = file->private_data;
	int ret;

	switch (cmd) {

	case VSPD_CHECK_REG:
		ret = vspd_check_reg(vdata, arg);
		break;

	case VSPD_MEM_COPY_DL:
		ret = vspd_dl_mem_copy_ioctl(vdata, arg);
		break;

	case VSPD_DISPLAY_DU_DL:
		ret = vspd_dl_output_du_ioctl(vdata, arg);
		break;

	case VSPD_STOP_DL:
		ret = vspd_wpf_reset(vdata);
		break;

	case VSPD_DEV_INIT:
		ret = vspd_device_init(vdata);
		break;

	case VSPD_RESOURCE_INIT:
		ret = vspd_init_ioctl(file, arg);
		break;

	case VSPD_RESOURCE_DEINIT:
		ret = vspd_deinit_ioctl(file);
		break;

	case VSPD_MEM_COPY:
		vspd_err(vdata, "[Error] VSPD_MEM_COPY not suport...\n");
		break;
	case VSPD_DISPLAY_DU:
		vspd_err(vdata, "[Error] VSPD_DISPLAY_DU not suport...\n");
		ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}


static unsigned int vspd_poll(struct file *file,
			struct poll_table_struct *wait)
{
	struct vspd_private_data *vdata = file->private_data;

	poll_wait(file, &vdata->event_wait, wait);

	return POLLIN;
}

static const struct file_operations vspd_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= vspd_ioctl,
	.open		= vspd_open,
	.release	= vspd_release,
	.poll		= vspd_poll,
};

static int vspd_probe(struct platform_device *pdev)
{
	int ret;
	int major;
	static struct vspd_drvdata *vdrv;

	vdrv = kzalloc(sizeof(*vdrv), GFP_KERNEL);
	if (vdrv == NULL) {
		pr_err("[Error] %s : kzalloc(vspd_drvdata)\n", __func__);
		ret = -ENOMEM;
		goto error1;
	}

	/* create sysfs */
	ret = alloc_chrdev_region(&vdrv->devt, 0, 1, DRVNAME);
	if (ret) {
		pr_err("[Error] %s : alloc_chrdev_region\n", __func__);
		goto error2;
	}

	major = MAJOR(vdrv->devt);
	cdev_init(&vdrv->cdev, &vspd_fops);
	vdrv->cdev.owner = THIS_MODULE;

	ret = cdev_add(&vdrv->cdev, vdrv->devt, 1);
	if (ret) {
		pr_err("[Error] %s : cdev_add\n", __func__);
		goto error3;
	}

	vdrv->class = class_create(THIS_MODULE, DRVNAME);
	if (IS_ERR(vdrv->class)) {
		pr_err("[Error] %s : class_create\n", __func__);
		ret = PTR_ERR(vdrv->class);
		goto error4;
	}

	vdrv->dev = device_create(vdrv->class, &pdev->dev,
				vdrv->devt, NULL, DEVNAME);
	if (IS_ERR(vdrv->dev)) {
		pr_err("[Error] %s : device_create\n", __func__);
		ret = PTR_ERR(vdrv->dev);
		goto error5;
	}

	platform_set_drvdata(pdev, vdrv);
	vdrv->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	p_vdrv = vdrv;

	return 0;

error5:
	class_destroy(vdrv->class);
error4:
	cdev_del(&vdrv->cdev);
error3:
	unregister_chrdev_region(vdrv->devt, 1);
error2:
	kfree(vdrv);
error1:
	return ret;
}

static int vspd_remove(struct platform_device *pdev)
{
	struct vspd_drvdata *vdrv = platform_get_drvdata(pdev);

	device_destroy(vdrv->class, vdrv->devt);
	class_destroy(vdrv->class);
	cdev_del(&vdrv->cdev);
	unregister_chrdev_region(vdrv->devt, 1);

	kfree(vdrv);

	return 0;
}

/* vspd device */
static void vspd_dev_release(struct device *dev)
{
	return;
}
static struct platform_device vspd_device = {
	.name		= DEVNAME,
	.id		= -1,
	.dev		= {
				.release = vspd_dev_release,
			},
};

/* vspd driver */
static struct platform_driver vspd_driver = {
	.driver		= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
	},
	.probe		= vspd_probe,
	.remove		= vspd_remove,
};

static int __init vspd_module_init(void)
{
	int ret;

	/* add a platform-level device */
	ret = platform_device_register(&vspd_device);
	if (ret) {
		pr_err("[Error] %s : platform_device_register\n", __func__);
		goto error1;
	}

	/* register a driver for platform-level devices */
	ret = platform_driver_register(&vspd_driver);
	if (ret) {
		pr_err("[Error] %s : platform_driver_register\n", __func__);
		goto error2;
	}

	return 0;

error2:
	platform_device_unregister(&vspd_device);
error1:
	return ret;
}
static void __exit vspd_module_exit(void)
{
	platform_driver_unregister(&vspd_driver);
	platform_device_unregister(&vspd_device);
}

module_init(vspd_module_init);
module_exit(vspd_module_exit);
#endif /* debug code */

MODULE_ALIAS("vspd");
MODULE_LICENSE("GPL");


