/*
 * rcar_imr.c  --  R-Car IMR-X2(4) Driver
 *
 * Copyright (C) 2015  Cogent Embedded, Inc.  <source@cogentembedded.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define DRV_NAME                        "rcar_imr"

/*******************************************************************************
 * Local types definitions
 ******************************************************************************/

struct imr_buffer {
	/* ...anything besides that? */
	struct v4l2_m2m_buffer      buf;
};

struct imr_q_data {
	/* ...anything we need to include into queue? */
	struct v4l2_pix_format      fmt;

	/* ...current format flags */
	u32                         flags;
};

struct imr_format_info {
	char       *name;
	u32         fourcc;
	u32         flags;
};

/* ...per-device data */
struct imr_device {
	struct device          *dev;
	struct clk             *clock;
	void __iomem           *mmio;
	int                     irq;
	struct mutex            mutex;
	spinlock_t              lock;

	struct v4l2_device      v4l2_dev;
	struct video_device     video_dev;
	struct v4l2_m2m_dev    *m2m_dev;
	void                   *alloc_ctx;

	/* ...should we include media-dev? likely, yes - tbd */

	/* ...do we need that counter really? framework counts fh structures for us - tbd */
	int                     refcount;
};

/* ...per file-handle context */
struct imr_ctx {
	struct v4l2_fh          fh;
	struct imr_device      *imr;
	struct v4l2_m2m_ctx    *m2m_ctx;
	struct imr_q_data       queue[2];
	int                     aborting;

	/* ...display-list main program data */
	void                   *dl_vaddr;
	dma_addr_t              dl_dma_addr;
	u32                     dl_size;
	u32                     dl_start_offset;

	/* ...pointers to the source/destination planes */
	u32                    *src_pa_ptr[2];
	u32                    *dst_pa_ptr[2];

#if 0
	/* ...current triangles list */
	void                   *tri_vaddr;
	dma_addr_t              tri_dma_addr;
	u32                     tri_size;
#endif

	/* ...cropping parameters */
	u16                     crop[4];
};

/*******************************************************************************
 * IMR registers
 ******************************************************************************/

#define IMR_IMRCR                       0x08
#define IMR_IMRCR_RS                    (1 << 0)
#define IMR_IMRCS_SWRST                 (1 << 15)

#define IMR_IMRSR                       0x0C
#define IMR_SRCR                        0x10
#define IMR_IMRSR_TRA                   (1 << 0)
#define IMR_IMRSR_IER                   (1 << 1)
#define IMR_IMRSR_INT                   (1 << 2)

#define IMR_ICR                         0x14
#define IMR_IMR                         0x18
#define IMR_ICR_TRAEN                   (1 << 0)
#define IMR_ICR_IEREN                   (1 << 1)
#define IMR_ICR_INTEN                   (1 << 2)

#define IMR_DLSP                        0x1C
#define IMR_DLSR                        0x20
#define IMR_DLSAR                       0x30

#define IMR_DSAR                        0x34
#define IMR_SSAR                        0x38
#define IMR_DSTR                        0x3C
#define IMR_SSTR                        0x40

#define IMR_CMRCR                       0x54
#define IMR_CMRCSR                      0x58
#define IMR_CMRCCR                      0x5C
#define IMR_CMR_RGBS                    (1 << 6)
#define IMR_CMR_DY10                    (1 << 8)
#define IMR_CMR_SY10                    (1 << 11)
#define IMR_CMR_UVS                     (1 << 13)
#define IMR_CMR_YCM                     (1 << 14)
#define IMR_CMR_CFS                     (1 << 15)

#define IMR_TRIMR                       0x60
#define IMR_TRIMSR                      0x64
#define IMR_TRIMCR                      0x68
#define IMR_TRIM_TME                    (1 << 0)
#define IMR_TRIM_BFE                    (1 << 1)
#define IMR_TRIM_AUTODG                 (1 << 2)
#define IMR_TRIM_AUTOSG                 (1 << 3)
#define IMR_TRIM_DYDXM                  (1 << 4)
#define IMR_TRIM_DUDVM                  (1 << 5)
#define IMR_TRIM_TCM                    (1 << 6)
#define IMR_TRICR                       0x6C

#define IMR_UVDPOR                      0x70
#define IMR_SUSR                        0x74
#define IMR_SVSR                        0x78

#define IMR_XMINR                       0x80
#define IMR_YMINR                       0x84
#define IMR_XMAXR                       0x88
#define IMR_YMAXR                       0x8C

#define IMR_AMXSR                       0x90
#define IMR_AMYSR                       0x94
#define IMR_AMXOR                       0x98
#define IMR_AMYOR                       0x9C

#define IMR_LINEMR                      0xA0
#define IMR_LINEMSR                     0xA4
#define IMR_LINEMCR                     0xA8

/*******************************************************************************
 * Auxiliary helpers
 ******************************************************************************/

static inline struct imr_ctx * fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct imr_ctx, fh);
}

/*******************************************************************************
 * Local constants definition
 ******************************************************************************/

#define IMR_F_Y                         (1 << 0)
#define IMR_F_UV420                     (1 << 1)
#define IMR_F_UV422                     (1 << 2)
#define IMR_F_RGB565                    (1 << 3)
#define IMR_F_ARGB555                   (1 << 4)
#define IMR_F_PLANES_MASK               ((1 << 5) - 1)
#define IMR_F_Y10                       (1 << 5)

/* ...get common planes bits */
static inline u32 __imr_flags_common(u32 iflags, u32 oflags)
{
	return (iflags & oflags) & IMR_F_PLANES_MASK;
}

static const struct imr_format_info imr_formats[] = {
	{
		.name = "YUV 4:2:0 semiplanar",
		.fourcc = V4L2_PIX_FMT_NV12,
		.flags = IMR_F_Y | IMR_F_UV420,
	},
	{
		.name = "YUV 4:2:2 semiplanar",
		.fourcc = V4L2_PIX_FMT_NV16,
		.flags = IMR_F_Y | IMR_F_UV422,
	},
	{
		.name = "ARGB1555",
		.fourcc = V4L2_PIX_FMT_ARGB555,
		.flags = IMR_F_ARGB555,
	},
	{
		.name = "RGB565",
		.fourcc = V4L2_PIX_FMT_RGB565,
		.flags = IMR_F_RGB565,
	},
	{
		.name = "Greyscale 8-bit",
		.fourcc = V4L2_PIX_FMT_GREY,
		.flags = IMR_F_Y,
	},
	{
		.name = "Greyscale 10-bit",
		.fourcc = V4L2_PIX_FMT_Y10,
		.flags = IMR_F_Y | IMR_F_Y10,
	},
};

/* ...test for a format supported */
static int __imr_try_fmt(struct imr_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	u32     fourcc = pix->pixelformat;
	int     i;

	/* ...both output and capture interface have the same set of supported formats */
	for (i = 0; i < ARRAY_SIZE(imr_formats); i++) {
		if (fourcc == imr_formats[i].fourcc) {
			/* ...fix-up format specification as needed */
			pix->field = V4L2_FIELD_NONE;

			v4l2_info(&ctx->imr->v4l2_dev, "format request: '%c%c%c%c', %d*%d\n",
				(fourcc >> 24) & 0xff, (fourcc >> 16) & 0xff,
				(fourcc >> 8) & 0xff, (fourcc >> 0) & 0xff,
				pix->width, pix->height);

			/* ...verify source/destination image dimensions */
			if (V4L2_TYPE_IS_OUTPUT(f->type))
				v4l_bound_align_image(&pix->width, 64, 2048, 6, &pix->height, 16, 2048, 1, 0);
			else
				v4l_bound_align_image(&pix->width, 64, 1024, 6, &pix->height, 16, 1024, 1, 0);

			return i;
		}
	}

	v4l2_err(&ctx->imr->v4l2_dev, "unsupported format request: '%c%c%c%c'\n",
		(fourcc >> 24) & 0xff, (fourcc >> 16) & 0xff,
		(fourcc >> 8) & 0xff, (fourcc >> 0) & 0xff);

	return -EINVAL;
}

/*******************************************************************************
 * V4L2 I/O controls
 ******************************************************************************/

static int imr_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strlcpy(cap->driver, DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->card, DRV_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, DRV_NAME, sizeof(cap->bus_info));

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT |
		V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;

	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

/* ...enumerate supported formats */
static int imr_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct imr_ctx *ctx = fh_to_ctx(priv);

	/* ...report the "input" formats */
	return -EINVAL;
}

/* ...retrieve current queue format; operation is locked ? */
static int imr_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct imr_ctx     *ctx = fh_to_ctx(priv);
	struct vb2_queue   *vq;
	struct imr_q_data  *q_data;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = &ctx->queue[V4L2_TYPE_IS_OUTPUT(f->type) ? 0 : 1];

	/* ...processing is locked? tbd */
	f->fmt.pix = q_data->fmt;

	return 0;
}

/* ...test particular format; operation is not locked */
static int imr_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct imr_ctx     *ctx = fh_to_ctx(priv);
	struct vb2_queue   *vq;

	/* ...make sure we have a queue of particular type */
	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	/* ...test if format is supported (adjust as appropriate) */
	return (__imr_try_fmt(ctx, f) >= 0 ? 0 : -EINVAL);
}

/* ...apply queue format; operation is locked ? */
static int imr_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct imr_ctx     *ctx = fh_to_ctx(priv);
	struct vb2_queue   *vq;
	struct imr_q_data  *q_data;
	int                 i;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	/* ...check if queue is busy */
	if (vb2_is_busy(vq))
		return -EBUSY;

	/* ...test if format is supported (adjust as appropriate) */
	i = __imr_try_fmt(ctx, f);
	if (i < 0)
		return -EINVAL;

	/* ...format is supported; save current format in a queue-specific data */
	q_data = &ctx->queue[V4L2_TYPE_IS_OUTPUT(f->type) ? 0 : 1];

	/* ...processing is locked? tbd */
	q_data->fmt = f->fmt.pix;
	q_data->flags = imr_formats[i].flags;

	return 0;
}

static int imr_reqbufs(struct file *file, void *priv, struct v4l2_requestbuffers *reqbufs)
{
	struct imr_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int imr_querybuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct imr_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int imr_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct imr_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int imr_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct imr_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int imr_expbuf(struct file *file, void *priv, struct v4l2_exportbuffer *eb)
{
	struct imr_ctx *ctx = fh_to_ctx(priv);

	return v4l2_m2m_expbuf(file, ctx->m2m_ctx, eb);
}

static int imr_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct imr_ctx         *ctx = fh_to_ctx(priv);

	/* ...verify the configuration is complete */
	if (!V4L2_TYPE_IS_OUTPUT(type) && !ctx->dl_vaddr) {
		v4l2_err(&ctx->imr->v4l2_dev, "stream configuration is not complete\n");
		return -EINVAL;
	}

	/* ...context is prepared for a streaming */
	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int imr_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct imr_ctx  *ctx = fh_to_ctx(priv);

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int imr_g_crop(struct file *file, void *priv, struct v4l2_crop *cr)
{
	struct imr_ctx  *ctx = fh_to_ctx(priv);

	cr->c.left = ctx->crop[0];
	cr->c.top = ctx->crop[2];
	cr->c.width = ctx->crop[1] - ctx->crop[0];
	cr->c.height = ctx->crop[3] - ctx->crop[2];

	return 0;
}

static int imr_s_crop(struct file *file, void *priv, const struct v4l2_crop *cr)
{
	struct imr_ctx *ctx = fh_to_ctx(priv);
	int             x0 = cr->c.left;
	int             y0 = cr->c.top;
	int             x1 = x0 + cr->c.width;
	int             y1 = y0 + cr->c.height;

	if (x0 < 0 || x1 >= 1024 || y0 < 0 || y1 >= 1024) {
		v4l2_err(&ctx->imr->v4l2_dev, "invalid cropping: %d/%d/%d/%d\n", x0, x1, y0, y1);
		return -EINVAL;
	}

	ctx->crop[0] = x0;
	ctx->crop[1] = x1;
	ctx->crop[2] = y0;
	ctx->crop[3] = y1;

	return 0;
}

static const struct v4l2_ioctl_ops imr_ioctl_ops = {
	.vidioc_querycap            = imr_querycap,

	.vidioc_enum_fmt_vid_cap    = imr_enum_fmt,
	.vidioc_enum_fmt_vid_out    = imr_enum_fmt,
	.vidioc_g_fmt_vid_cap       = imr_g_fmt,
	.vidioc_g_fmt_vid_out       = imr_g_fmt,
	.vidioc_try_fmt_vid_cap     = imr_try_fmt,
	.vidioc_try_fmt_vid_out     = imr_try_fmt,
	.vidioc_s_fmt_vid_cap       = imr_s_fmt,
	.vidioc_s_fmt_vid_out       = imr_s_fmt,

	.vidioc_reqbufs             = imr_reqbufs,
	.vidioc_querybuf            = imr_querybuf,
	.vidioc_qbuf                = imr_qbuf,
	.vidioc_dqbuf               = imr_dqbuf,
	.vidioc_expbuf              = imr_expbuf,
	.vidioc_streamon            = imr_streamon,
	.vidioc_streamoff           = imr_streamoff,

	.vidioc_g_crop              = imr_g_crop,
	.vidioc_s_crop              = imr_s_crop,
};

/*******************************************************************************
 * V4L2 control interface
 ******************************************************************************/

/* ...I guess we don't need that thing */
static inline int imr_controls_create(struct imr_ctx *ctx)
{
	return -EINVAL;
}

/*******************************************************************************
 * Context processing queue
 ******************************************************************************/

static int imr_queue_setup(struct vb2_queue *vq,
			const struct v4l2_format *fmt,
			unsigned int *nbuffers, unsigned int *nplanes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	struct imr_ctx     *ctx = vb2_get_drv_priv(vq);
	struct imr_q_data  *q_data = &ctx->queue[V4L2_TYPE_IS_OUTPUT(vq->type) ? 0 : 1];
	int                 w = q_data->fmt.width;
	int                 h = q_data->fmt.height;

	/* ...we use only single-plane formats */
	*nplanes = 1;

	/* ...specify plane size */
	switch (q_data->fmt.pixelformat) {
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_ARGB555:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_Y10:
		sizes[0] = w * h * 2;
		break;

	case V4L2_PIX_FMT_NV12:
		sizes[0] = w * h * 3 / 2;
		break;

	case V4L2_PIX_FMT_GREY:
		sizes[0] = w * h;
		break;

	default:
		return -EINVAL;
	}

	/* ...specify default allocator */
	alloc_ctxs[0] = ctx->imr->alloc_ctx;

	return 0;
}

/* ...do we need a buffer prepare flag? */
static int imr_buf_prepare(struct vb2_buffer *vb)
{
	struct imr_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	/* ...unclear yet if we want to prepare a buffer somehow */
	return 0;
}

static void imr_buf_queue(struct vb2_buffer *vb)
{
	struct imr_ctx     *ctx = vb2_get_drv_priv(vb->vb2_queue);

	/* ...should we take a lock? - tbd */
	v4l2_m2m_buf_queue(ctx->m2m_ctx, vb);
}

static int imr_buf_finish(struct vb2_buffer *vb)
{
	struct imr_ctx     *ctx = vb2_get_drv_priv(vb->vb2_queue);

	/* ...any special processing of completed buffer? - tbd */
	return 0;
}

static int imr_stop_streaming(struct vb2_queue *vq)
{
	struct imr_ctx     *ctx = vb2_get_drv_priv(vq);
	struct vb2_buffer  *vb;
	unsigned long       flags;

	/* ...protect against interrupt handler? - tbd */
	spin_lock_irqsave(&ctx->imr->lock, flags);

	/* ...purge all buffers from a queue - tbd - shall be available from framework */
	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vb = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
		else
			vb = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
		if (vb == NULL)
			break;

		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&ctx->imr->lock, flags);

	return 0;
}

/* ...buffer queue operations */
static struct vb2_ops imr_qops = {
	.queue_setup        = imr_queue_setup,
	.buf_prepare        = imr_buf_prepare,
	.buf_queue          = imr_buf_queue,
	.buf_finish         = imr_buf_finish,
	.stop_streaming     = imr_stop_streaming,
	.wait_prepare       = vb2_ops_wait_prepare,
	.wait_finish        = vb2_ops_wait_finish,
};

/* ...M2M device processing queue initialization */
static int imr_queue_init(void *priv, struct vb2_queue *src_vq,
			struct vb2_queue *dst_vq)
{
	struct imr_ctx *ctx = priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct imr_buffer);
	src_vq->ops = &imr_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->imr->mutex;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct imr_buffer);
	dst_vq->ops = &imr_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->imr->mutex;
	ret = vb2_queue_init(dst_vq);
	if (ret)
		return ret;

	return 0;
}

/*******************************************************************************
 * Generic device file operations
 ******************************************************************************/

static int imr_open(struct file *file)
{
	struct imr_device      *imr = video_drvdata(file);
	struct video_device    *vfd = video_devdata(file);
	struct imr_ctx         *ctx;
	int                     ret;

	/* ...allocate processing context associated with given instance */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	/* ...initialize per-file-handle structure */
	v4l2_fh_init(&ctx->fh, vfd);
	//ctx->fh.ctrl_handler = &ctx->ctrl_handler;
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	/* ...set default source / destination formats - need that? */
	ctx->imr = imr;
	ctx->queue[0].fmt.pixelformat = 0;
	ctx->queue[1].fmt.pixelformat = 0;

	/* ...set default cropping parameters */
	ctx->crop[1] = ctx->crop[3] = 0x3FF;

	/* ...initialize M2M processing context */
	ctx->m2m_ctx = v4l2_m2m_ctx_init(imr->m2m_dev, ctx, imr_queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		ret = PTR_ERR(ctx->m2m_ctx);
		goto v4l_prepare_rollback;
	}

#if 0
	/* ...initialize controls and stuff */
	ret = imr_controls_create(ctx);
	if (ret < 0)
		goto v4l_prepare_rollback;
#endif

	/* ...lock access to global device data */
	if (mutex_lock_interruptible(&imr->mutex)) {
		ret = -ERESTARTSYS;
		goto v4l_prepare_rollback;
	}

	/* ...bring-up device as needed */
	if (imr->refcount == 0) {
		ret = clk_prepare_enable(imr->clock);
		if (ret < 0)
			goto device_prepare_rollback;
	}

	imr->refcount++;

	mutex_unlock(&imr->mutex);

	v4l2_info(&imr->v4l2_dev, "IMR device opened (refcount=%u)\n", imr->refcount);

	return 0;

device_prepare_rollback:
	/* ...unlock global device data */
	mutex_unlock(&imr->mutex);

v4l_prepare_rollback:
	/* ...destroy context */
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);

	return ret;
}

static int imr_release(struct file *file)
{
	struct imr_device  *imr = video_drvdata(file);
	struct imr_ctx     *ctx = fh_to_ctx(file->private_data);

	/* ...destroy M2M device processing context */
	mutex_lock(&imr->mutex);
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mutex_unlock(&imr->mutex);

	//v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	/* ...destroy display-list if defined */
	if (ctx->dl_vaddr)
		dma_free_coherent(imr->dev, ctx->dl_size, ctx->dl_vaddr, ctx->dl_dma_addr);

	kfree(ctx);

	/* ...disable hardware operation */
	mutex_lock(&imr->mutex);
	if (--imr->refcount == 0)
		clk_disable_unprepare(imr->clock);
	mutex_unlock(&imr->mutex);

	v4l2_info(&imr->v4l2_dev, "closed device instance\n");

	return 0;
}

/*******************************************************************************
 * Display list commands
 ******************************************************************************/

/* ...display list opcodes */
#define IMR_OP_TRI(n)                   ((0x8A << 24) | ((n) & 0xFFFF))
#define IMR_OP_LINE(n)                  ((0x8B << 24) | ((n) & 0xFFFF))
#define IMR_OP_NOP(n)                   ((0x80 << 24) | ((n) & 0xFFFF))
#define IMR_OP_TRAP                     ((0x8F << 24))
#define IMR_OP_WTL(add, n)              ((0x81 << 24) | (((add) / 4) << 16) | ((n) & 0xFFFF))
#define IMR_OP_WTS(add, data)           ((0x82 << 24) | (((add) / 4) << 16) | ((data) & 0xFFFF))
#define IMR_OP_INT                      ((0x88 << 24))
#define IMR_OP_SYNCM                    ((0x86 << 24))
#define IMR_OP_GOSUB                    ((0x8C << 24))
#define IMR_OP_RET                      ((0x8D << 24))

/* ...mapping specification descriptor */
struct imr_map_desc {
	/* ...mapping types */
	u32             type;

	/* ...total size of the mesh structure */
	u32             size;

}   __attribute__((packed));

/* ...regular mesh specification */
#define IMR_MAP_MESH                    (1 << 0)

/* ...auto-generated source coordinates */
#define IMR_MAP_AUTODG                  (1 << 1)

/* ...auto-generated destination coordinates */
#define IMR_MAP_AUTOSG                  (1 << 2)

/* ...relative source coordinates specification mode */
#define IMR_MAP_DYDX                    (1 << 3)

/* ...relative destination coordinates specification mode */
#define IMR_MAP_DUDV                    (1 << 4)

/* ...vertex clockwise-mode order */
#define IMR_MAP_TCM                     (1 << 5)

/* ...source coordinate decimal point position bit index */
#define __IMR_MAP_UVDPOR_SHIFT          8
#define __IMR_MAP_UVDPOR_MASK           (0x7 << __IMR_MAP_UVDPOR_SHIFT)
#define IMR_MAP_UVDPOR(n)               ((n & 0x7) << __IMR_MAP_UVDPOR_SHIFT)

/* ...regular mesh specification */
struct imr_mesh {
	/* ...rectangular mesh size */
	u16             rows, columns;

	/* ...mesh parameters */
	u16             x0, y0, dx, dy;

}   __attribute__((packed));

/*******************************************************************************
 * Type A (absolute coordinates of source/destination) mapping
 ******************************************************************************/

/* ...return size of the subroutine for type "a" mapping */
static inline u32 imr_tri_type_a_get_length(int rows, int columns)
{
	return ((columns * 4 + 1) * (rows - 1) + 1) * sizeof(u32);
}

/* ...set a mesh rows * columns using absolute coordinates */
static inline u32 * imr_tri_set_type_a(u32 *dl, void *map, int rows, int columns)
{
	int     i, j;

	/* ...convert lattice into set of stripes */
	for (i = 0; i < rows - 1; i++) {
		*dl++ = IMR_OP_TRI(columns * 2);
		for (j = 0; j < columns; j++) {
			memcpy(dl, map, 8);
			memcpy(dl + 2, map + 8 * columns, 8);
			dl += 4;
			map += 8;
		}
	}

	*dl++ = IMR_OP_RET;
	return dl;
}

/* ...calculate length of a type "b" mapping */
static inline u32 imr_tri_type_b_get_length(int rows, int columns)
{
	return ((columns * 2 + 2) * (rows - 1) + 4) * sizeof(u32);
}

/* ...set an auto-generated mesh n * m for a source/destination */
static inline u32 * imr_tri_set_type_b(u32 *dl, void *map, int rows, int columns, int x0, int y0, int dx, int dy)
{
	int         i, j;

	/* ...set mesh configuration */
	*dl++ = IMR_OP_WTS(IMR_AMXSR, dx);
	*dl++ = IMR_OP_WTS(IMR_AMYSR, dy);

	/* ...origin by "x" coordinate is the same across all rows */
	*dl++ = IMR_OP_WTS(IMR_AMXOR, x0);

	/* ...convert lattice into set of stripes */
	for (i = 0; i < rows - 1; i++, y0 += dy) {
		/* ...set origin by "y" coordinate for a current row */
		*dl++ = IMR_OP_WTS(IMR_AMYOR, y0);
		*dl++ = IMR_OP_TRI(2 * columns);

		/* ...fill single row */
		for (j = 0; j < columns; j++) {
			memcpy(dl, map, 4);
			memcpy(dl + 1, map + 4 * columns, 4);
			dl += 2;
			map += 4;
		}
	}

	*dl++ = IMR_OP_RET;
	return dl;
}


/* ...setup DL for Y/RGB/YUV420/422 processing */
static inline void imr_dl_program_setup(struct imr_ctx *ctx, u32 type)
{
	u32    *dl = ctx->dl_vaddr + ctx->dl_start_offset;
	u32     subaddr = ctx->dl_dma_addr;
	u32     iflags = ctx->queue[0].flags;
	u32     oflags = ctx->queue[1].flags;
	u32     cflags = __imr_flags_common(iflags, oflags);
	int     w = ctx->queue[0].fmt.width;
	int     h = ctx->queue[0].fmt.height;
	int     W = ctx->queue[1].fmt.width;
	int     H = ctx->queue[1].fmt.height;

	v4l2_info(&ctx->imr->v4l2_dev, "setup %u*%u -> %u*%u mapping (type=%x)\n", w, h, W, H, type);

	/* ...set triangle mode register from user-supplied descriptor */
	*dl++ = IMR_OP_WTS(IMR_TRIMCR, 0xFFFF);
	*dl++ = IMR_OP_WTS(IMR_TRIMSR, ((type & 0x3E) << 1) | IMR_TRIM_BFE | IMR_TRIM_TME);

	/* ...set source coordinate precision */
	*dl++ = IMR_OP_WTS(IMR_UVDPOR, (type >> __IMR_MAP_UVDPOR_SHIFT) & 0x7);

	/* ...select Y/RGB-plane processing mode */
	*dl++ = IMR_OP_WTS(IMR_CMRCCR, 0xFFFF);

	/* ...set source/destination addresses of Y/RGB plane */
	*dl++ = IMR_OP_WTL(IMR_DSAR, 2);
	ctx->dst_pa_ptr[0] = dl++;
	ctx->src_pa_ptr[0] = dl++;

	/* ...select RGB operation mode if required */
	if (cflags & (IMR_F_RGB565 | IMR_F_ARGB555)) {
		/* ...select RGB processing mode */
		*dl++ = IMR_OP_WTS(IMR_CMRCSR, IMR_CMR_CFS | (cflags & IMR_F_RGB565 ? IMR_CMR_RGBS : 0));

		/* ...set source/destination strides (2 bytes/pixel) */
		*dl++ = IMR_OP_WTS(IMR_DSTR, W << 1);
		*dl++ = IMR_OP_WTS(IMR_SSTR, w << 1);
	} else {
		/* ...select Y plane processing */ 
		*dl++ = IMR_OP_WTS(IMR_CMRCSR, (iflags & IMR_F_Y10 ? IMR_CMR_SY10 : 0) | (oflags & IMR_F_Y10 ? IMR_CMR_DY10 : 0));

		/* ...set source/destination strides (1 byte/pixel) */
		*dl++ = IMR_OP_WTS(IMR_DSTR, W);
		*dl++ = IMR_OP_WTS(IMR_SSTR, w);
	}

	/* ...set source width/height of Y/RGB plane */
	*dl++ = IMR_OP_WTL(IMR_SUSR, 2);
	*dl++ = w - 1;
	*dl++ = h - 1;

	/* ...invoke subroutine for triangles drawing */
	*dl++ = IMR_OP_GOSUB;
	*dl++ = subaddr;

	/* ...add UV-plane processing as needed */
	if (cflags & (IMR_F_UV420 | IMR_F_UV422)) {
		/* ...select UV-plane processing mode; put sync before switching */
		*dl++ = IMR_OP_SYNCM;

		/* ...setup UV-plane source/destination addresses */
		*dl++ = IMR_OP_WTL(IMR_DSAR, 2);
		ctx->dst_pa_ptr[1] = dl++;
		ctx->src_pa_ptr[1] = dl++;

		if (cflags & IMR_F_UV420) {
			/* ...select chromacity plane processing */
			*dl++ = IMR_OP_WTS(IMR_CMRCSR, IMR_CMR_YCM | IMR_CMR_UVS);

			/* ...set source width/height of UV-plane */
			*dl++ = IMR_OP_WTL(IMR_SUSR, 2);
			*dl++ = ((w - 2) << 16) | (w - 1);
			*dl++ = (h >> 1) - 1;
		} else {
			/* ...select chromacity plane processing */
			*dl++ = IMR_OP_WTS(IMR_CMRCSR, IMR_CMR_YCM);

			/* ...set source width/height of UV-plane */
			*dl++ = IMR_OP_WTL(IMR_SUSR, 2);
			*dl++ = ((w - 2) << 16) | (w - 1);
			*dl++ = h - 1;
		}

		/* ...draw triangles */
		*dl++ = IMR_OP_GOSUB;
		*dl++ = subaddr;
	} else {
		/* ...clear pointers to the source/destination UV-planes addresses */
		ctx->src_pa_ptr[1] = ctx->dst_pa_ptr[1] = NULL;
	}

	/* ...signal completion of the operation */
	*dl++ = IMR_OP_SYNCM;
	*dl++ = IMR_OP_TRAP;
}

/* ...return length of a DL main program - should correspond the functions defined above */
static inline u32 imr_dl_program_length(struct imr_ctx *ctx)
{
	u32     iflags = ctx->queue[0].flags;
	u32     oflags = ctx->queue[1].flags;
	u32     cflags = __imr_flags_common(iflags, oflags);

	/* ...check if formats are compatible */
	if (cflags == 0) {
		v4l2_err(&ctx->imr->v4l2_dev, "formats are incompatible\n");
		return 0;
	}

	/* ...select program length basing on UV-processing status */
	return (cflags & (IMR_F_UV420 | IMR_F_UV422) ? 27 : 17) << 2;
}

/*******************************************************************************
 * Mapping specification processing
 ******************************************************************************/

/* ...set mapping data */
static int imr_ioctl_map(struct imr_ctx *ctx, unsigned int cmd, unsigned long arg)
{
	struct imr_device      *imr = ctx->imr;
	struct imr_map_desc     desc;
	struct imr_mesh        *mesh;
	void                   *buf, *map;
	u32                     length;
	u32                     tri_length;
	u32                     rows, columns;
	u32                     dl_size;
	u32                     dl_start_offset;
	int                     ret;

	/* ...we support only writing command */
	if (_IOC_DIR(cmd) != _IOC_WRITE)
		return -EINVAL;

	/* ...verify command size is valid (contains a descriptor) */
	if (_IOC_SIZE(cmd) != sizeof(desc))
		return -EINVAL;

	/* ...copy command descriptor */
	if (copy_from_user(&desc, (void __user *)arg, sizeof(desc)))
		return -EFAULT;

	/* ...read remainder of data into temporary buffer */
	length = desc.size;
	buf = kmalloc(length, GFP_KERNEL);
	if (!buf) {
		v4l2_err(&imr->v4l2_dev, "failed to allocate %u bytes for mapping reading\n", length);
		return -ENOMEM;
	}

	/* ...copy mesh data */
	if (copy_from_user(buf, (void __user *)(arg + sizeof(desc)), length)) {
		v4l2_err(&imr->v4l2_dev, "failed to read %u bytes of mapping specification\n", length);
		ret = -EFAULT;
		goto out;
	}

	/* ...for now we deal only with regular rectangular mesh */
	if ((desc.type & IMR_MAP_MESH) == 0) {
		v4l2_err(&imr->v4l2_dev, "irregular mesh is not supported\n");
		ret = -EINVAL;
		goto out;
	}

	/* ...assure we have proper mesh descriptor */
	if (length < sizeof(struct imr_mesh)) {
		v4l2_err(&imr->v4l2_dev, "invalid mesh specification size: %u\n", length);
		ret = -EINVAL;
		goto out;
	}

	mesh = (struct imr_mesh *)buf;
	rows = mesh->rows, columns = mesh->columns;
	length -= sizeof(struct imr_mesh);
	map = buf + sizeof(struct imr_mesh);

	/* ...allocate DMA-coherent (necessary? - tbd) display-list */
	if (desc.type & (IMR_MAP_AUTODG | IMR_MAP_AUTOSG)) {
		/* ...mapping is given using automatic generation pattern; check size */
		if (rows * columns * 4 != length) {
			v4l2_err(&imr->v4l2_dev, "invalid mesh size: %u*%u*4 != %u\n", rows, columns, length);
			ret = -EINVAL;
			goto out;
		}

		/* ...calculate size of triangles drawing subroutine */
		tri_length = imr_tri_type_b_get_length(rows, columns);
	} else {
		/* ...mapping is done with absolute coordinates */
		if (rows * columns * 8 != length) {
			v4l2_err(&imr->v4l2_dev, "invalid mesh size: %u*%u*8 != %u\n", rows, columns, length);
			ret = -EINVAL;
			goto out;
		}

		/* ...calculate size of triangles drawing subroutine */
		tri_length = imr_tri_type_a_get_length(rows, columns);
	}

	/* ...DL main program shall start with 8-byte aligned address */
	dl_start_offset = (tri_length + 7) & ~7;

	/* ...calculate main routine length */
	dl_size = imr_dl_program_length(ctx);
	if (!dl_size) {
		v4l2_err(&imr->v4l2_dev, "format configuration error\n");
		ret = -EINVAL;
		goto out;
	}

	/* ...for now we use a single display list, with TRI subroutine prepending MAIN */
	dl_size += dl_start_offset;

	/* ...release current DL program if exists */
	if (ctx->dl_vaddr)
		dma_free_coherent(imr->dev, ctx->dl_size, ctx->dl_vaddr, ctx->dl_dma_addr);

	/* ...allocate DMA-coherent memory for a display list */
	ctx->dl_vaddr = dma_alloc_coherent(imr->dev, dl_size, &ctx->dl_dma_addr, GFP_KERNEL);
	if (!ctx->dl_vaddr) {
		v4l2_err(&imr->v4l2_dev, "failed to allocate %u bytes from DMA-coherent pool\n", dl_size);
		goto out;
	} else {
		/* ...update display list DMA buffer parameters */
		ctx->dl_size = dl_size;
		ctx->dl_start_offset = dl_start_offset;
	}

	/* ...prepare a triangles drawing subroutine */
	if (desc.type & (IMR_MAP_AUTOSG | IMR_MAP_AUTODG)) {
		imr_tri_set_type_b(ctx->dl_vaddr, map, rows, columns, mesh->x0, mesh->y0, mesh->dx, mesh->dy);
	} else {
		imr_tri_set_type_a(ctx->dl_vaddr, map, rows, columns);
	}

	/* ...prepare main DL-program */
	imr_dl_program_setup(ctx, desc.type);

	/* ...display list updated successfully */
	v4l2_info(&ctx->imr->v4l2_dev, "display list updated\n");
	ret = 0;

	print_hex_dump_bytes("DL-", DUMP_PREFIX_OFFSET, (void *)ctx->dl_vaddr + ctx->dl_start_offset, ctx->dl_size - ctx->dl_start_offset);

out:
	/* ...release interim buffer */
	kfree(buf);

	return ret;
}


/* ...customized I/O control processing */
static long imr_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* ...intercept custom I/O control */
	if (_IOC_TYPE(cmd) == 'I') {
		switch (_IOC_NR(cmd)) {
		case 0:
			/* ...set mapping data */
			return imr_ioctl_map(file->private_data, cmd, arg);

		default:
			return -ENOIOCTLCMD;
		}
	}

	return video_ioctl2(file, cmd, arg);
}

static unsigned int imr_poll(struct file *file, struct poll_table_struct *wait)
{
	struct imr_device  *imr = video_drvdata(file);
	struct imr_ctx     *ctx = fh_to_ctx(file->private_data);
	unsigned int        res;

	if (mutex_lock_interruptible(&imr->mutex))
		return -ERESTARTSYS;

	res = v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
	mutex_unlock(&imr->mutex);

	return res;
}

static int imr_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct imr_device  *imr = video_drvdata(file);
	struct imr_ctx     *ctx = fh_to_ctx(file->private_data);
	int                 ret;

	/* ...should we protect all M2M operations with mutex? - tbd */
	if (mutex_lock_interruptible(&imr->mutex))
		return -ERESTARTSYS;

	ret = v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);

	mutex_unlock(&imr->mutex);

	return ret;
}

static const struct v4l2_file_operations imr_fops = {
	.owner          = THIS_MODULE,
	.open           = imr_open,
	.release        = imr_release,
	.unlocked_ioctl	= imr_ioctl,
	.poll           = imr_poll,
	.mmap           = imr_mmap,
};

/*******************************************************************************
 * M2M device interface
 ******************************************************************************/

/* ...job cleanup function */
static void imr_cleanup(struct imr_ctx *ctx)
{
	struct imr_device  *imr = ctx->imr;
	struct vb2_buffer  *src_buf, *dst_buf;
	unsigned long       flags;

	/* ...lock access to the context data */
	spin_lock_irqsave(&imr->lock, flags);

	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);

	v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
	v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);

	v4l2_m2m_job_finish(imr->m2m_dev, ctx->m2m_ctx);

	spin_unlock_irqrestore(&imr->lock, flags);
}

/* ...job execution function */
static void imr_device_run(void *priv)
{
	struct imr_ctx     *ctx = priv;
	struct imr_device  *imr = ctx->imr;
	struct vb2_buffer  *src_buf, *dst_buf;
	unsigned long       src_addr, dst_addr;
	unsigned long       flags;

	v4l2_info(&imr->v4l2_dev, "run next job...\n");

	/* ...protect access to internal device state */
	spin_lock_irqsave(&imr->lock, flags);

	/* ...retrieve input/output buffers */
	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);

	/* ...set cropping data */
	iowrite32(ctx->crop[0], imr->mmio + IMR_XMINR);
	iowrite32(ctx->crop[1], imr->mmio + IMR_XMAXR);
	iowrite32(ctx->crop[2], imr->mmio + IMR_YMINR);
	iowrite32(ctx->crop[3], imr->mmio + IMR_YMAXR);

	/* ...adjust source/destination parameters of the program (Y / RGB16) */
	*ctx->src_pa_ptr[0] = src_addr = vb2_dma_contig_plane_dma_addr(src_buf, 0);
	*ctx->dst_pa_ptr[0] = dst_addr = vb2_dma_contig_plane_dma_addr(dst_buf, 0);

	/* ...adjust source/destination parameters of the UV-plane as needed */
	if (ctx->src_pa_ptr[1] && ctx->dst_pa_ptr[1]) {
		*ctx->src_pa_ptr[1] = src_addr + ctx->queue[0].fmt.width * ctx->queue[0].fmt.height;
		*ctx->dst_pa_ptr[1] = dst_addr + ctx->queue[1].fmt.width * ctx->queue[1].fmt.height;
	}

	/* ...set display list address */
	iowrite32(ctx->dl_dma_addr + ctx->dl_start_offset, imr->mmio + IMR_DLSAR);

	/* ...force clearing of status register bits */
	iowrite32(0x7, imr->mmio + IMR_SRCR);

	/* ...unmask/enable interrupts */
	iowrite32((1 << 4), imr->mmio + IMR_IMR);
	iowrite32(IMR_ICR_TRAEN | IMR_ICR_IEREN | IMR_ICR_INTEN, imr->mmio + IMR_ICR);

	/* ...start rendering operation */
	iowrite32(0x1, imr->mmio + IMR_IMRCR);

	/* ...unlock device access */
	spin_unlock_irqrestore(&imr->lock, flags);

	v4l2_info(&imr->v4l2_dev, "process buffer-pair 0x%08x:0x%08x\n", *ctx->src_pa_ptr[0], *ctx->dst_pa_ptr[0]);
}

/* ...check whether a job is ready for execution */
static int imr_job_ready(void *priv)
{
	struct imr_ctx *ctx = priv;

	if (v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) > 0 &&
		v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx) > 0 && !ctx->aborting) {
		return 1;
	}

	return 0;
}

/* ...abort currently processed job */
static void imr_job_abort(void *priv)
{
	struct imr_ctx *ctx = priv;

	/* ...put job abortion flag (is that a sticky thing? - tbd) */
	ctx->aborting = 1;

	/* ...finish current job */
	imr_cleanup(ctx);
}

/* ...M2M interface definition */
static struct v4l2_m2m_ops imr_m2m_ops = {
	.device_run	= imr_device_run,
	.job_ready	= imr_job_ready,
	.job_abort	= imr_job_abort,
};

/*******************************************************************************
 * Interrupt handling
 ******************************************************************************/

static irqreturn_t imr_irq_handler(int irq, void *data)
{
	struct imr_device  *imr = data;
	struct imr_ctx     *ctx;
	struct vb2_buffer  *src_buf, *dst_buf;
	u32                 status;
	irqreturn_t         ret = IRQ_NONE;

	/* ...check and ack interrupt status */
	status = ioread32(imr->mmio + IMR_IMRSR);
	if (!(status & (IMR_IMRSR_INT | IMR_IMRSR_IER | IMR_IMRSR_TRA))) {
		v4l2_err(&imr->v4l2_dev, "spurious interrupt\n");
		return ret;
	}

	/* ...acknowledge interrupt */
	iowrite32(status, imr->mmio + IMR_SRCR);

	/* ...protect access to current context */
	spin_lock(&imr->lock);

	/* ...get current job context */
	ctx = v4l2_m2m_get_curr_priv(imr->m2m_dev);
	if (!ctx) {
		v4l2_err(&imr->v4l2_dev, "no active job\n");
		goto handled;
	}

	/* ...remove buffers */
	src_buf = v4l2_m2m_src_buf_remove(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(ctx->m2m_ctx);
	if (!src_buf || !dst_buf) {
		v4l2_err(&imr->v4l2_dev, "no buffers associated with current context\n");
		goto handled;
	}

	/* ...check for a TRAP interrupt indicating completion of current DL */
	if (status & IMR_IMRSR_TRA) {
		/* ...operation completed normally; data is written into destination buffer */
		dst_buf->v4l2_buf.timecode = src_buf->v4l2_buf.timecode;
		dst_buf->v4l2_buf.timestamp = src_buf->v4l2_buf.timestamp;
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

		v4l2_info(&imr->v4l2_dev, "ctx:%p - buffers <%p,%p> done\n", ctx, src_buf, dst_buf);
	} else {
		/* ...operation completed in error; no way to understand what exactly went wrong */
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);

		v4l2_info(&imr->v4l2_dev, "ctx:%p - buffers <%p,%p> done in error\n", ctx, src_buf, dst_buf);
	}

	/* ...what exactly I am trying to protect here? - tbd */
	spin_unlock(&imr->lock);

	/* ...finish current job (attempt to restart immediately) */
	v4l2_m2m_job_finish(imr->m2m_dev, ctx->m2m_ctx);

	return IRQ_HANDLED;

handled:
	/* ...again, what exactly is to be protected? */
	spin_unlock(&imr->lock);

	return IRQ_HANDLED;
}

/*******************************************************************************
 * Device probing / removal interface
 ******************************************************************************/

static int imr_probe(struct platform_device *pdev)
{
	struct imr_device *imr;
	struct resource *res;
	int ret;

	imr = devm_kzalloc(&pdev->dev, sizeof(*imr), GFP_KERNEL);
	if (!imr)
		return -ENOMEM;

	//init_waitqueue_head(&imr->irq_queue);
	mutex_init(&imr->mutex);
	spin_lock_init(&imr->lock);
	imr->dev = &pdev->dev;

	/* ...memory-mapped registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	imr->mmio = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(imr->mmio))
		return PTR_ERR(imr->mmio);

	/* ...interrupt service routine registration */
	imr->irq = ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, imr->irq, imr_irq_handler, 0, dev_name(&pdev->dev), imr);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", imr->irq);
		return ret;
	}

	imr->clock = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(imr->clock)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(imr->clock);
	}

	/* ...create v4l2 device */
	ret = v4l2_device_register(&pdev->dev, &imr->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register v4l2 device\n");
		return ret;
	}

	/* ...create mem2mem device handle */
	imr->m2m_dev = v4l2_m2m_init(&imr_m2m_ops);
	if (IS_ERR(imr->m2m_dev)) {
		v4l2_err(&imr->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(imr->m2m_dev);
		goto device_register_rollback;
	}

	imr->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(imr->alloc_ctx)) {
		v4l2_err(&imr->v4l2_dev, "Failed to init memory allocator\n");
		ret = PTR_ERR(imr->alloc_ctx);
		goto m2m_init_rollback;
	}

	strlcpy(imr->video_dev.name, DRV_NAME, sizeof(imr->video_dev.name));
	imr->video_dev.fops         = &imr_fops;
	imr->video_dev.ioctl_ops    = &imr_ioctl_ops;
	imr->video_dev.minor        = -1;
	imr->video_dev.release      = video_device_release_empty;
	imr->video_dev.lock         = &imr->mutex;
	imr->video_dev.v4l2_dev     = &imr->v4l2_dev;
	imr->video_dev.vfl_dir      = VFL_DIR_M2M;

	ret = video_register_device(&imr->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&imr->v4l2_dev, "Failed to register video device\n");
		goto vb2_allocator_rollback;
	}

	video_set_drvdata(&imr->video_dev, imr);
	platform_set_drvdata(pdev, imr);

	v4l2_info(&imr->v4l2_dev, "IMR device (pdev: %d) registered as /dev/video%d\n", pdev->id, imr->video_dev.num);

	return 0;

vb2_allocator_rollback:
	vb2_dma_contig_cleanup_ctx(imr->alloc_ctx);

m2m_init_rollback:
	v4l2_m2m_release(imr->m2m_dev);

device_register_rollback:
	v4l2_device_unregister(&imr->v4l2_dev);

	return ret;
}

static int imr_remove(struct platform_device *pdev)
{
	struct imr_device *imr = platform_get_drvdata(pdev);

	video_unregister_device(&imr->video_dev);
	vb2_dma_contig_cleanup_ctx(imr->alloc_ctx);
	v4l2_m2m_release(imr->m2m_dev);
	v4l2_device_unregister(&imr->v4l2_dev);

	return 0;
}

/*******************************************************************************
 * Power management
 ******************************************************************************/

#ifdef CONFIG_PM_SLEEP

/* ...device suspend hook; clock control only - tbd */
static int imr_pm_suspend(struct device *dev)
{
	struct imr_device *imr = dev_get_drvdata(dev);

	WARN_ON(mutex_is_locked(&imr->mutex));

	if (imr->refcount == 0)
		return 0;

	clk_disable_unprepare(imr->clock);

	return 0;
}

/* ...device resume hook; clock control only */
static int imr_pm_resume(struct device *dev)
{
	struct imr_device *imr = dev_get_drvdata(dev);

	WARN_ON(mutex_is_locked(&imr->mutex));

	if (imr->refcount == 0)
		return 0;

	clk_prepare_enable(imr->clock);

	return 0;
}

#endif  /* CONFIG_PM_SLEEP */

/* ...power management callbacks */
static const struct dev_pm_ops imr_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imr_pm_suspend, imr_pm_resume)
};

/* ...device table */
static const struct of_device_id imr_of_match[] = {
	{ .compatible = "renesas,imr-x2" },
	{ },
};

/* ...platform driver interface */
static struct platform_driver imr_platform_driver = {
	.probe		= imr_probe,
	.remove		= imr_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "imr",
		.pm	= &imr_pm_ops,
		.of_match_table = imr_of_match,
	},
};

module_platform_driver(imr_platform_driver);

MODULE_ALIAS("imr");
MODULE_AUTHOR("Cogent Embedded Inc. <sources@cogentembedded.com>");
MODULE_DESCRIPTION("Renesas IMR-(L)X2(3/4) Driver");
MODULE_LICENSE("GPL");
