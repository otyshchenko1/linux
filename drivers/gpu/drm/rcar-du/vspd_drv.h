/*
 * drivers/gpu/drm/rcar-du/vspd_drv.h
 *     This header file is R-Car VSPD function.
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

#ifndef VSPD_DRV_H
#define VSPD_DRV_H


struct vspd_private_data;

enum {
	VSPD_FENCE_NONE,
	VSPD_FENCE_NEXT_PLANE,
	VSPD_FENCE_PLANE_END,
};

int vspd_wpf_reset(struct vspd_private_data *vdata);
int vspd_device_init(struct vspd_private_data *vdata);
int vspd_dl_output_du(struct vspd_private_data *vdata,
		struct vspd_blend blends[], int num, int use_sync);
int vspd_check_reg(struct vspd_private_data *vdata,
			 unsigned long arg);
struct vspd_private_data *vspd_init(struct device *dev, int use_vsp);
void vspd_deinit(struct vspd_private_data *vdata);
void vspd_set_callback(struct vspd_private_data *vdata,
				void (*callback)(void *data),
				void *callback_data);
int vspd_dl_output_mute(struct vspd_private_data *vdata,
			unsigned long width, unsigned long height,
			int num, int use_sync);
int vspd_dl_write_back_start(struct vspd_private_data *vdata,
		struct vspd_blend blends[], int num);
int vspd_dl_write_back_wait(struct vspd_private_data *vdata, int done);

#endif /* VSPD_DRV_H */
