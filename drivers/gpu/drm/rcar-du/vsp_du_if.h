/*
 * drivers/gpu/drm/rcar-du/vsp_du_if.h
 *     This header file is R-Car VSPD interface.
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

#ifndef __VSP_DU_DU_H___
#define __VSP_DU_DU_H___

#include <uapi/drm/rcar_du_drm.h>

#include "rcar_du_kms.h"
#include "rcar_du_plane.h"

int vsp_du_if_update_plane(void *handle, int vsp_plane,
			   struct rcar_du_plane *rplane, bool blend);
int vsp_du_if_update_planes(void *handle,
			   struct rcar_du_plane rplanes[], int num);
int vsp_du_if_setup_base(void *handle, struct rcar_du_plane *rplane,
				bool interlace);
int vsp_du_if_update_base(void *handle, struct rcar_du_plane *rplane);
int vsp_du_if_start(void *handle);
void vsp_du_if_stop(void *handle);
void *vsp_du_if_init(struct device *dev, int use_vsp);
void vsp_du_if_deinit(void *handle);
void vsp_du_if_set_callback(void *handle,
			    void (*callback)(void *data),
			    void *callback_data);
void vsp_du_if_set_mute(void *handle, bool on);
int vsp_du_if_write_back(void *handle, struct rcar_du_screen_shot *sh);
void vsp_du_if_set_desktop(void *handle, bool on);

void vsp_du_if_reg_debug(void *handle);

#endif /* __VSP_DU_DU_H___ */
