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

#endif /* VSPD_DRV_H */
