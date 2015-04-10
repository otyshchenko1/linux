#ifndef __VSP_DU_DU_H___
#define __VSP_DU_DU_H___

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

void vsp_du_if_reg_debug(void *handle);

#endif /* __VSP_DU_DU_H___ */
