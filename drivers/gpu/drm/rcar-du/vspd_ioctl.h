#ifndef VSPD_IOCTL_H
#define VSPD_IOCTL_H


#include <linux/types.h>
#include <asm/ioctl.h>

/* format */
enum {
	/* 16 bpp RGB */
	VSPD_FMT_RGB565,
	VSPD_FMT_ARGB1555,
	VSPD_FMT_XRGB1555,

	/* 24 bpp RGB */
	VSPD_FMT_RGB888,

	/* 32 bpp RGB */
	VSPD_FMT_ARGB8888,
	VSPD_FMT_XRGB8888,

	/* packed YCbCr */
	VSPD_FMT_YUV422I_UYVY, /* YUV422 Interleave */
	VSPD_FMT_YUV422I_YUYV, /* YUV422 Interleave */

	/* 2 plane YCbCr */
	VSPD_FMT_YUV420SP_NV12, /* YUV420 Semi-Planar */
	VSPD_FMT_YUV420SP_NV21, /* YUV420 Semi-Planar */
	VSPD_FMT_YUV422SP_NV16, /* YUV422 Semi-Planar */
	VSPD_FMT_YUV422SP_NV61, /* YUV422 Semi-Planar */

	/* 3 plane YCbCr */
	VSPD_FMT_YUV420P_YU12, /* YUV420 Planar */
};

enum {
	USE_VSPD0,
	USE_VSPD1,
	USE_VSPS,
};

/* module num */
#define VSPD_INPUT_IMAGE_NUM 4
#define VSPD_BLEND_IMAGE_NUM 4
#define VSPD_SCALING_IMAGE_NUM 1

/* swap */
#define VSPD_LONG_LWORD_SWAP	(1 << 3)
#define VSPD_LWORD_SWAP		(1 << 2)
#define VSPD_WORD_SWAP		(1 << 1)
#define VSPD_BYTE_SWAP		(1 << 0)

/* flags */
#define VSPD_FLAG_PREMUL_ALPH		(1 << 0)
#define VSPD_FLAG_VIRTUAL		(1 << 1)
#define VSPD_FLAG_COLOR_CONV_BT601	(0 << 2) /* for RGB <-> YUV */
#define VSPD_FLAG_COLOR_CONV_BT709	(1 << 2) /* for RGB <-> YUV */
 #define VSPD_FLAG_COLOR_CONV_MASK	(1 << 2)
#define  VSPD_FLAG_PROGRESSIVE		(0 << 3)
#define  VSPD_FLAG_INTERLACE_TOP	(1 << 3)
#define  VSPD_FLAG_INTERLACE_BOTTOM	(2 << 3)
 #define VSPD_FLAG_IP_MASK		(3 << 3)

struct vspd_rect {
	unsigned long x;
	unsigned long y;
	unsigned long width;
	unsigned long height;
};

struct vspd_image {
	int enable;
	unsigned long addr_y;
	unsigned long addr_c0;
	unsigned long addr_c1;
	unsigned long width;
	unsigned long height;
	unsigned long stride_y;
	unsigned long stride_c;
	unsigned long format;
	struct vspd_rect crop;
	struct vspd_rect dist;
	unsigned long swap;
	unsigned long alpha;
	unsigned long flag;
};

struct vspd_blend {
	struct vspd_image in[VSPD_INPUT_IMAGE_NUM];
	struct vspd_image out;
};

#define VSPD_MEM_COPY		_IOW('d', 0, struct vspd_image)
#define VSPD_CHECK_REG		_IOW('d', 1, int)
#define VSPD_MEM_COPY_DL	_IOW('d', 2, struct vspd_image)
#define VSPD_DISPLAY_DU		_IOW('d', 3, struct vspd_image)
#define VSPD_DISPLAY_DU_DL	_IOW('d', 4, struct vspd_image)
#define VSPD_STOP_DL		_IO('d', 5)
#define VSPD_DEV_INIT		_IO('d', 6)
#define VSPD_RESOURCE_INIT	_IOW('d', 7, int)
#define VSPD_RESOURCE_DEINIT	_IO('d', 8)

#endif /* VSPD_IOCTL_H */
