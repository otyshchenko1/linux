/*
 * rcar_du_drm.h  --  R-Car Display Unit DRM driver
 *
 * Copyright (C) 2015 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __RCAR_DU_DRM_H__
#define __RCAR_DU_DRM_H__

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
#include <drm/drm_mode.h>

/* DRM_IOCTL_RCAR_DU_SET_DESKTOP:Desktop Plane ON/OFF */
struct rcar_du_desktop_plane {
	int crtc_id;
	int on;
};

/* DRM_IOCTL_RCAR_DU_SET_SCRSHOT:VSPD screen shot */
struct rcar_du_screen_shot {
	unsigned int	buff;
	unsigned int	buff_len;
	unsigned int	crtc_id;
	unsigned int	fmt;
	unsigned int	width;
	unsigned int	height;
};


/* rcar-du + vspd specific ioctls */
#define DRM_RCAR_DU_SET_PLANE_FENCE	0
#define DRM_RCAR_DU_DBG			1
#define DRM_RCAR_DU_DBG_VSP		2
#define DRM_RCAR_DU_SET_DESKTOP		3
#define DRM_RCAR_DU_SET_SCRSHOT		4

#define DRM_IOCTL_RCAR_DU_SET_PLANE_FENCE \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_SET_PLANE_FENCE, \
		struct drm_mode_set_plane)
#define DRM_IOCTL_RCAR_DU_DBG \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_DBG, int)
#define DRM_IOCTL_RCAR_DU_DBG_VSP \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_DBG_VSP, int)
#define DRM_IOCTL_RCAR_DU_SET_DESKTOP \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_SET_DESKTOP, \
		struct rcar_du_desktop_plane)
#define DRM_IOCTL_RCAR_DU_SET_SCRSHOT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_SET_SCRSHOT, \
		struct rcar_du_screen_shot)

#endif /* CONFIG_DRM_RCAR_DU_CONNECT_VSP */

#endif /* __RCAR_DU_DRM_H__ */
