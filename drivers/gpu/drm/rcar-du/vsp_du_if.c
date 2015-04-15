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

#include <uapi/linux/rcar-du-frm-interface.h>

#include "rcar_du_crtc.h"

#include "vspd_ioctl.h"
#include "vspd_drv.h"
#include "vsp_du_if.h"

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
struct vsp_du_if {
	bool active;
	bool mute;
	struct vspd_private_data *pdata;
	struct vspd_blend blend[2];
	struct mutex lock;
	int use_vsp;
	int interlace;
};



static int set_plane_param(struct vsp_du_if *du_if, int vsp_plane,
			   struct rcar_du_plane *rplane)
{
	int i;
	struct vspd_image *image;
	unsigned long in_flag;

	if (rplane == NULL) {
		for (i = 0; i < du_if->interlace; i++) {
			image = &du_if->blend[i].in[vsp_plane];
			image->enable = 0;
		}
		return 0;
	}

	in_flag = rplane->premultiplied ? VSPD_FLAG_PREMUL_ALPH : 0;
	in_flag |= VSPD_FLAG_COLOR_CONV_BT601;

	for (i = 0; i < du_if->interlace; i++) {
		image = &du_if->blend[i].in[vsp_plane];

		switch (rplane->format->fourcc) {
		case DRM_FORMAT_RGB565:
			image->format = VSPD_FMT_RGB565;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP;
			image->addr_c0 = 0;
			image->addr_c1 = 0;
			image->stride_c = 0;
			break;

		case DRM_FORMAT_ARGB1555:
			image->format = VSPD_FMT_ARGB1555;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP;
			image->addr_c0 = 0;
			image->addr_c1 = 0;
			image->stride_c = 0;
			break;

		case DRM_FORMAT_XRGB1555:
			image->format = VSPD_FMT_XRGB1555;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP;
			image->addr_c0 = 0;
			image->addr_c1 = 0;
			image->stride_c = 0;
			break;

		case DRM_FORMAT_ARGB8888:
			image->format = VSPD_FMT_ARGB8888;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP;
			image->addr_c0 = 0;
			image->addr_c1 = 0;
			image->stride_c = 0;
			break;

		case DRM_FORMAT_XRGB8888:
			image->format = VSPD_FMT_XRGB8888;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP;
			image->addr_c0 = 0;
			image->addr_c1 = 0;
			image->stride_c = 0;
			break;

		case DRM_FORMAT_UYVY:
			image->format = VSPD_FMT_YUV422I_UYVY;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP | VSPD_BYTE_SWAP;
			image->addr_c0 = 0;
			image->addr_c1 = 0;
			image->stride_c = 0;
			break;

		case DRM_FORMAT_YUYV:
			image->format = VSPD_FMT_YUV422I_YUYV;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP | VSPD_BYTE_SWAP;
			image->addr_c0 = 0;
			image->addr_c1 = 0;
			image->stride_c = 0;
			break;

		case DRM_FORMAT_NV12:
			image->format = VSPD_FMT_YUV420SP_NV12;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->addr_c0 = rplane->dma[1] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->stride_c = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP | VSPD_BYTE_SWAP;
			image->addr_c1 = 0;
			break;

		case DRM_FORMAT_NV21:
			image->format = VSPD_FMT_YUV420SP_NV21;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->addr_c0 = rplane->dma[1] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->stride_c = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP | VSPD_BYTE_SWAP;
			image->addr_c1 = 0;
			break;

		case DRM_FORMAT_NV16:
			image->format = VSPD_FMT_YUV422SP_NV16;
			image->addr_y = rplane->dma[0] + (rplane->pitch * i);
			image->addr_c0 = rplane->dma[1] + (rplane->pitch * i);
			image->stride_y = rplane->pitch * du_if->interlace;
			image->stride_c = rplane->pitch * du_if->interlace;
			image->swap = VSPD_LONG_LWORD_SWAP | VSPD_LWORD_SWAP |
					VSPD_WORD_SWAP | VSPD_BYTE_SWAP;
			image->addr_c1 = 0;
			break;

		default:
			pr_err("[Error] %s : no support format\n", __func__);
			return -EINVAL;
		}

		image->enable		= 1;
		image->width		= rplane->width;
		image->height		= rplane->height / du_if->interlace;
		image->crop.x		= rplane->src_x;
		image->crop.y		= rplane->src_y / du_if->interlace;
		image->crop.width	= rplane->width;
		image->crop.height	= rplane->height / du_if->interlace;
		image->dist.x		= rplane->dst_x;
		image->dist.y		= rplane->dst_y / du_if->interlace;
		image->dist.width	= rplane->d_width;
		image->dist.height	= rplane->d_height / du_if->interlace;
		image->alpha		= rplane->alpha;
		image->flag		= in_flag;
	}

	return 0;
}

static int update_image(struct vsp_du_if *du_if)
{
	if (du_if->active) {
		if (du_if->mute) {
			return vspd_dl_output_mute(du_if->pdata,
						du_if->blend[0].out.width,
						du_if->blend[0].out.height,
						du_if->interlace,
						VSPD_FENCE_NONE);
		} else  {
			return vspd_dl_output_du(du_if->pdata, du_if->blend,
						du_if->interlace,
						VSPD_FENCE_NONE);
		}
	}

	return 0;
}

int vsp_du_if_update_plane(void *handle, int vsp_plane,
			   struct rcar_du_plane *rplane, bool blend)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;
	int ret;

	if (!handle) {
		pr_err("[Error] %s : handle is NULL\n", __func__);
		return -EINVAL;
	}

	if ((vsp_plane < 1) || (VSPD_INPUT_IMAGE_NUM < vsp_plane)) {
		pr_err("[Error] %s : vspd plane invalid(%d)\n",
				__func__, vsp_plane);
		return -ENOSPC;
	}

	mutex_lock(&du_if->lock);

	ret = set_plane_param(du_if, vsp_plane, rplane);
	if (!ret && blend)
		ret = update_image(du_if);

	mutex_unlock(&du_if->lock);

	return ret;
}

int vsp_du_if_update_planes(void *handle,
			   struct rcar_du_plane rplanes[], int num)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;
	int ret;
	int i;

	if (!handle) {
		pr_err("[Error] %s : handle is NULL\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < num; i++) {
		if ((rplanes[i].hwindex < 1) ||
			(VSPD_INPUT_IMAGE_NUM < rplanes[i].hwindex)) {
			pr_err("[Error] %s : vspd plane invalid\n",
					__func__);
			return -ENOSPC;
		}
	}

	mutex_lock(&du_if->lock);

	for (i = 0; i < num; i++) {
		int vsp_plane = rplanes[i].hwindex;
		struct rcar_du_plane *rplane =
			rplanes[i].enabled ? &rplanes[i] : NULL;

		ret = set_plane_param(du_if, vsp_plane, rplane);
		if (ret)
			goto end;
	}

	ret = update_image(du_if);

end:
	mutex_unlock(&du_if->lock);

	return ret;
}

int vsp_du_if_setup_base(void *handle, struct rcar_du_plane *rplane,
				bool interlace)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;
	struct vspd_image *out = &du_if->blend[0].out;
	int i;

	mutex_lock(&du_if->lock);

	du_if->interlace = interlace ? 2 : 1;

	/* set output param */
	out->enable		= 0; /* not effect for out param */
	out->width		= rplane->width;
	out->height		= rplane->height;
	out->crop.x		= 0;
	out->crop.y		= 0;
	out->crop.width		= rplane->width;
	out->crop.height	= rplane->height;
	out->dist.x		= 0;
	out->dist.y		= 0;
	out->dist.width		= rplane->width;
	out->dist.height	= rplane->height;
	out->alpha		= 0; /* not effect for out param in LIF */
	out->format		= VSPD_FMT_XRGB8888;
	out->addr_y		= 0; /* not effect for out param in LIF */
	out->addr_c0		= 0;
	out->addr_c1		= 0;
	out->stride_y		= rplane->d_width * 4;
	out->stride_c		= 0;
	out->swap		= 0; /* not effect for out param in LIF */

	set_plane_param(du_if, 0, rplane);

	if (du_if->interlace == 2) {
		du_if->blend[1].out = du_if->blend[0].out;

		for (i = 0; i < du_if->interlace; i++) {
			out = &du_if->blend[i].out;
			out->crop.height	/= 2;
			out->dist.height	/= 2;
		}
	}

	mutex_unlock(&du_if->lock);

	return 0;
}

int vsp_du_if_update_base(void *handle, struct rcar_du_plane *rplane)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;
	int ret;

	mutex_lock(&du_if->lock);
	set_plane_param(du_if, 0, rplane);
	ret = update_image(du_if);
	mutex_unlock(&du_if->lock);

	return ret;
}

int vsp_du_if_start(void *handle)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;
	int ret;

	mutex_lock(&du_if->lock);
	du_if->active = true;
	ret = update_image(du_if);
	mutex_unlock(&du_if->lock);
	udelay(8); /* wait for vspd data output */
	return ret;
}


void vsp_du_if_stop(void *handle)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;

	mutex_lock(&du_if->lock);
	du_if->active = false;
	vspd_wpf_reset(du_if->pdata);
	mutex_unlock(&du_if->lock);
}

void vsp_du_if_reg_debug(void *handle)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;

	switch (du_if->use_vsp) {
	case USE_VSPD0:
	case USE_VSPD1:
		vspd_check_reg(du_if->pdata, du_if->use_vsp);
		break;
	}
}


void *vsp_du_if_init(struct device *dev, int use_vsp)
{
	struct vsp_du_if *du_if;

	du_if = kzalloc(sizeof(*du_if), GFP_KERNEL);

	if (du_if == NULL)
		goto error1;

	switch (use_vsp) {
	case RCAR_DU_VSPD_0:
		du_if->pdata = vspd_init(dev, USE_VSPD0);
		du_if->use_vsp = USE_VSPD0;
		break;
	case RCAR_DU_VSPD_1:
		du_if->pdata = vspd_init(dev, USE_VSPD1);
		du_if->use_vsp = USE_VSPD1;
		break;
	default:
		goto error2;
	}

	if (du_if->pdata == NULL)
		goto error2;

	du_if->active = false;
	du_if->mute = false;
	du_if->interlace = 0;
	mutex_init(&du_if->lock);

	vspd_device_init(du_if->pdata);

	return du_if;

error2:
	kfree(du_if);
error1:
	return NULL;
}

void vsp_du_if_deinit(void *handle)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;

	vspd_deinit(du_if->pdata);
	kfree(du_if);
}

void vsp_du_if_set_callback(void *handle,
			    void (*callback)(void *data),
			    void *callback_data)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;

	vspd_set_callback(du_if->pdata, callback, callback_data);
}


int vsp_du_if_du_info(void *callback_data)
{
	struct rcar_du_crtc *rcrtc = (struct rcar_du_crtc *)callback_data;
	return rcar_du_get_frmend(rcrtc->index);
}

void vsp_du_if_set_mute(void *handle, bool on)
{
	struct vsp_du_if *du_if = (struct vsp_du_if *)handle;

	mutex_lock(&du_if->lock);

	du_if->mute = on;

	update_image(du_if);

	mutex_unlock(&du_if->lock);
}

MODULE_ALIAS("vsp_du_if");
MODULE_LICENSE("GPL");

#endif /* CONFIG_DRM_RCAR_DU_CONNECT_VSP */

