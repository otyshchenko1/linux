/*
 * drivers/gpu/drm/rcar-du/vspd_dl.c
 *     This file is R-Car VSPD display List.
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

#include "vspd_drv_private.h"

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
#define CONNECT_DU_MODE 1
#else
#define CONNECT_DU_MODE 0
#endif

/* a DislpayList total size (header + body) */
#define DL_MEM_SIZE 2048

enum {
	DL_MEM_NO_USE,
	DL_MEM_USE,
};

#define	DL_FLAG_NONE		0x00000000
#define	DL_FLAG_INTERRUPT	0x00000001
#define	DL_FLAG_AUTO_REPEAT	0x00000002
#define	DL_FLAG_MANUAL_REPEAT	0x00000004
#define	DL_FLAG_HEADER_LESS	0x00000008
#define	DL_FLAG_BODY_WRITEBLE	0x00000010
#define	DL_FLAG_BODY_DL_TALL	0x00000020

#define	DL_FLAG_REPEAT_MASK	(DL_FLAG_MANUAL_REPEAT | DL_FLAG_AUTO_REPEAT)

inline void vspd_dl_free_header_core(struct dl_head *head)
{
	int i;
	struct dl_head *free_head = head;
	struct dl_head *tmp_head;

	if (head == NULL)
		return;

	free_head = head;

	do {
		/* in header mode, dl_head->next is NULL */
		for (i = 0; i < DISPLAY_LIST_BODY_NUM; i++)
			if (free_head->dl_body_list[i] != NULL)
				free_head->dl_body_list[i]->use = DL_MEM_NO_USE;

		free_head->use = DL_MEM_NO_USE;
		tmp_head = free_head->next;
		free_head->next = NULL;

		free_head = tmp_head;

	} while ((free_head != NULL)  && (free_head != head));
}

static void vspd_dl_free_multi_body(struct dl_body *body)
{
	struct dl_body *free_body = body;
	struct dl_body *tmp_body;

	if (body == NULL)
		return;

	free_body = body;

	do {
		free_body->use = DL_MEM_NO_USE;
		tmp_body = free_body->next;
		free_body->next = NULL;
		free_body = tmp_body;
	} while ((free_body != NULL)  && (free_body != body));
}

void vspd_dl_reset(struct vspd_private_data *vdata)
{
	struct dl_memory *dlmemory = vdata->dlmemory;
	int i, j;
	unsigned long flags;

	spin_lock_irqsave(&dlmemory->lock, flags);

	dlmemory->flag = 0;
	dlmemory->active_header = NULL;
	dlmemory->next_header = NULL;
	dlmemory->active_body = NULL;
	dlmemory->next_body = NULL;
	dlmemory->pending_body = NULL;
	dlmemory->active_body_now = NULL;
	dlmemory->active_body_next_set = NULL;

	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		vspd_dl_free_header_core(&dlmemory->head[i]);

		for (j = 0; j < DISPLAY_LIST_BODY_NUM; j++) {
			dlmemory->body[i][j].use = DL_MEM_NO_USE;
			dlmemory->body[i][j].reg_count = 0;
		}

		vspd_dl_free_multi_body(&dlmemory->single_body[i]);
	}

	spin_unlock_irqrestore(&dlmemory->lock, flags);
}

/* This function is called when the frame end interrupt occurs */
int vspd_dl_irq_frame_end(struct vspd_private_data *vdata)
{
	struct dl_memory *dlmemory = vdata->dlmemory;

	if (dlmemory->flag & DL_FLAG_HEADER_LESS)
		return 0;

	/* Header mode simply command start. */
	/* Display List update when Display List frame end interrupt */
	if (dlmemory->flag & DL_FLAG_MANUAL_REPEAT)
		vspd_write(vdata, VI6_CMDn(USE_WPF), 1);

	return 0;
}


static int vspd_dl_irq_dl_frame_end_header_mode(struct vspd_private_data *vdata)
{
	int frame_stat = DL_IRQ_FRAME_END;
	struct dl_memory *dlmemory = vdata->dlmemory;
	struct dl_head *free_head = NULL;

	spin_lock(&dlmemory->lock);

	if (dlmemory->flag & DL_FLAG_REPEAT_MASK) {
		if (dlmemory->next_header != NULL) {
			/* update next frame & free old Display List */
			free_head = dlmemory->active_header;
			dlmemory->active_header = dlmemory->next_header;
			dlmemory->next_header = NULL;
			vspd_write(vdata, VI6_DL_HDR_ADDRn(USE_WPF),
					dlmemory->active_header->paddr);
			frame_stat |= DL_IRQ_UPDATE_FRAME;
		}
	} else {
		free_head = dlmemory->active_header;
		dlmemory->active_header = NULL;
		frame_stat |= DL_IRQ_UPDATE_FRAME;
	}

	spin_unlock(&dlmemory->lock);

	if (dlmemory->flag & DL_FLAG_MANUAL_REPEAT)
		vspd_write(vdata, VI6_CMDn(USE_WPF), 1);

	vspd_dl_free_header_core(free_head);

	return frame_stat; /* Dislpay List was swap ? */
}

static int vspd_dl_irq_dl_frame_end_header_less(struct vspd_private_data *vdata)
{
	int frame_stat = 0;
	struct dl_memory *dlmemory = vdata->dlmemory;
	struct dl_body *free_body = NULL;

	spin_lock(&dlmemory->lock);

	dlmemory->flag &= ~DL_FLAG_BODY_WRITEBLE;

	if (dlmemory->flag & DL_FLAG_REPEAT_MASK) {
		if (dlmemory->active_body_now->flag & DL_FLAG_BODY_DL_TALL) {
			frame_stat = DL_IRQ_FRAME_END;

			if ((dlmemory->next_body != NULL)) {
				/* free old Display List */
				free_body = dlmemory->active_body;
				dlmemory->active_body = dlmemory->next_body;
				dlmemory->next_body = NULL;
				frame_stat |= DL_IRQ_UPDATE_FRAME;
			}
		}

		if (dlmemory->active_body_next_set) {
			dlmemory->active_body_now =
				dlmemory->active_body_next_set;
			dlmemory->active_body_next_set = NULL;
		}
	} else {
		free_body = dlmemory->active_body;
		dlmemory->active_body = NULL;
		frame_stat = DL_IRQ_UPDATE_FRAME | DL_IRQ_FRAME_END;
	}

	spin_unlock(&dlmemory->lock);

	if (dlmemory->flag & DL_FLAG_MANUAL_REPEAT)
		vspd_write(vdata, VI6_CMDn(USE_WPF), 1);

	vspd_dl_free_multi_body(free_body);

	return frame_stat; /* Dislpay List was swap ? */
}

/* This function is called when the Display List frame end interrupt occurs */
int vspd_dl_irq_dl_frame_end(struct vspd_private_data *vdata)
{
	int frame_stat;
	struct dl_memory *dlmemory = vdata->dlmemory;

	/* update next Display List & free old Display List & */
	/* VSP Start Command                                  */

	if (dlmemory->flag & DL_FLAG_HEADER_LESS)
		frame_stat = vspd_dl_irq_dl_frame_end_header_less(vdata);
	else
		frame_stat = vspd_dl_irq_dl_frame_end_header_mode(vdata);

	return frame_stat; /* Dislpay List was swap ? */
}

/* This function is called when the display start interrupt occurs. */
int vspd_dl_irq_display_start(struct vspd_private_data *vdata)
{
	struct dl_memory *dlmemory = vdata->dlmemory;
	struct dl_body *next_body = NULL;
	struct dl_body *free_body = NULL;

	if (!(dlmemory->flag & DL_FLAG_HEADER_LESS))
		return 0;

	spin_lock(&dlmemory->lock);

#if CONNECT_DU_MODE
	if (dlmemory->active_body->flag & DL_FLAG_BODY_DL_TALL) {
		/* DU progressive mode */
		dlmemory->flag |= DL_FLAG_BODY_WRITEBLE;
		if (dlmemory->pending_body) {
			/* update next frame for pending. */
			free_body = dlmemory->next_body;
			dlmemory->next_body = dlmemory->pending_body;
			next_body = dlmemory->next_body;
			dlmemory->pending_body = NULL;
			dlmemory->active_body_next_set = next_body;
		}
	} else {
		/* DU interlace mode */
		if (vsp_du_if_du_info(vdata->callback_data)) {
			/* DU started odd field, vspd set even field. */
			next_body = dlmemory->active_body->next;
			dlmemory->active_body_next_set = next_body;
		} else {
			/* DU started even field, vspd set even odd. */
			dlmemory->flag |= DL_FLAG_BODY_WRITEBLE;
			if (dlmemory->pending_body) {
				/* update next frame for pending. */
				free_body = dlmemory->next_body;
				dlmemory->next_body = dlmemory->pending_body;
				next_body = dlmemory->next_body;
				dlmemory->pending_body = NULL;
				dlmemory->active_body_next_set = next_body;
			} else {
				next_body = dlmemory->active_body;
				dlmemory->active_body_next_set = next_body;
			}
		}
	}
#else
	if (dlmemory->active_body_now->flag & DL_FLAG_BODY_DL_TALL) {
		dlmemory->flag |= DL_FLAG_BODY_WRITEBLE;
		if (dlmemory->pending_body) {
			/* update next frame for pending. */
			free_body = dlmemory->next_body;
			next_body = dlmemory->next_body =
				dlmemory->pending_body;
			dlmemory->pending_body = NULL;
			dlmemory->active_body_next_set = next_body;
		} else if (dlmemory->active_body !=
				dlmemory->active_body->next) {
			next_body = dlmemory->active_body_now->next;
			dlmemory->active_body_next_set = next_body;
		}
	} else {
		next_body = dlmemory->active_body_now->next;
		dlmemory->active_body_next_set = next_body;
	}
#endif

	spin_unlock(&dlmemory->lock);

	if (next_body) {
		vspd_write(vdata, VI6_DL_HDR_ADDRn(USE_WPF),
				next_body->paddr);
		vspd_write(vdata, VI6_DL_BODY_SIZE,
			(next_body->reg_count * 8) |
				VI6_DL_BODY_SIZE_UPD0);

		if (dlmemory->flag & DL_FLAG_MANUAL_REPEAT)
			vspd_write(vdata, VI6_CMDn(USE_WPF), 1);
	}

	vspd_dl_free_multi_body(free_body);

	return 0;
}

static int vspd_dl_header_setup(struct dl_memory *dlmemory,
				struct dl_head *head)
{
	int i;
	struct display_header *dheader = head->dheader;
	int dl_count = 0;

	memset(head->dheader, 0, sizeof(*head->dheader));
	for (i = 0; i < DISPLAY_LIST_BODY_NUM; i++) {
		struct dl_body *dl_body_list = head->dl_body_list[i];
		if (dl_body_list != NULL) {
			dheader->display_list[dl_count].num_bytes =
				(dl_body_list->reg_count * 8) << 0;
			dheader->display_list[dl_count].plist =
				dl_body_list->paddr;
			dl_count++;
		}
	}
	for (i = dl_count; i < DISPLAY_LIST_BODY_NUM; i++) {
		dheader->display_list[i].num_bytes = 0;
		dheader->display_list[i].plist = 0;
	}

	/* display list body num */
	dheader->num_list_minus1 = (dl_count - 1) << 0;
	dheader->pnext_header = (unsigned long)head->next->paddr;

	dheader->int_auto = 0;

	if (head->flag & DL_FLAG_INTERRUPT)
		dheader->int_auto |= 1 << 1;

	if (head->flag & DL_FLAG_AUTO_REPEAT)
		dheader->int_auto |= 1 << 0;

	return 0;
}



static int vspd_dl_start_header_mode(struct vspd_private_data *vdata,
			struct dl_head *head[], int num)
{
	struct dl_memory *dlmemory = vdata->dlmemory;
	int i;
	unsigned long flags;

	for (i = 0; i < (num - 1); i++) {
		head[i]->flag = 0;
		head[i]->next = head[i + 1];
		vspd_dl_header_setup(dlmemory, head[i]);
	}

	head[num - 1]->next = head[0];
	head[num - 1]->flag = dlmemory->flag;
	vspd_dl_header_setup(dlmemory, head[num - 1]);

	spin_lock_irqsave(&dlmemory->lock, flags);
	dlmemory->active_header = head[0];
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	/* DL LWORD swap */
	vspd_write(vdata, VI6_DL_SWAP, VI6_DL_SWAP_LWS);
	vspd_write(vdata, VI6_DL_CTRL,
		VI6_DL_CTRL_AR_WAIT | VI6_DL_CTRL_DL_ENABLE);
	vspd_write(vdata, VI6_DL_HDR_ADDRn(USE_WPF), head[0]->paddr);

	vspd_write(vdata, VI6_WPFn_IRQ_ENB(USE_WPF),
			VI6_WPFn_IRQ_FRE | VI6_WPFn_IRQ_DFE);
	vspd_write(vdata, VI6_DISP_IRQ_ENB, 0);

	vspd_write(vdata, VI6_CMDn(USE_WPF), 1);

	return 0;
}


static int vspd_dl_start_header_less(struct vspd_private_data *vdata,
				  struct dl_body *bodies[], int num)
{
	int i;
	struct dl_memory *dlmemory = vdata->dlmemory;
	unsigned long flags;

	for (i = 0; i < (num - 1); i++) {
		bodies[i]->next = bodies[i + 1];
		bodies[i]->flag = 0;
	}
	bodies[num - 1]->next = bodies[0];
	bodies[num - 1]->flag = DL_FLAG_BODY_DL_TALL;

	spin_lock_irqsave(&dlmemory->lock, flags);
	dlmemory->active_body = bodies[0];
	dlmemory->active_body_now = bodies[0];
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	vspd_write(vdata, VI6_DL_CTRL,
		VI6_DL_CTRL_AR_WAIT | VI6_DL_CTRL_DL_ENABLE |
		VI6_DL_CTRL_CFM0 | VI6_DL_CTRL_NH0);

	/* DL LWORD swap */
	vspd_write(vdata, VI6_DL_SWAP, VI6_DL_SWAP_LWS);

	vspd_write(vdata, VI6_DL_HDR_ADDRn(USE_WPF), bodies[0]->paddr);
	vspd_write(vdata, VI6_DL_BODY_SIZE,
			(bodies[0]->reg_count * 8) | VI6_DL_BODY_SIZE_UPD0);

	vspd_write(vdata, VI6_WPFn_IRQ_ENB(USE_WPF), VI6_WPFn_IRQ_DFE);
	vspd_write(vdata, VI6_DISP_IRQ_ENB, VI6_DISP_IRQ_STA_DST);

	vspd_write(vdata, VI6_CMDn(USE_WPF), 1);

	return 0;
}

int vspd_dl_start(struct vspd_private_data *vdata,
			void *dl, int num, int dl_mode)
{
	struct dl_memory *dlmemory = vdata->dlmemory;

	if ((dlmemory->active_header != NULL) ||
	    (dlmemory->active_body != NULL)) {
		pr_err("[Error] %s : vspd active\n",
			__func__);
		return -EPERM;
	}

	switch (dl_mode) {
	case DL_MODE_SINGLE:
		dlmemory->flag = DL_FLAG_INTERRUPT;
		break;
	case DL_MODE_MANUAL_REPEAT:
		dlmemory->flag = DL_FLAG_INTERRUPT | DL_FLAG_MANUAL_REPEAT;
		break;
	case DL_MODE_HEADER_LESS_AUTO_REPEAT:
		dlmemory->flag = DL_FLAG_HEADER_LESS | DL_FLAG_AUTO_REPEAT;
		break;

	case DL_MODE_HEADER_LESS_MANUAL_REPEAT: /* TODO */
	case DL_MODE_AUTO_REPEAT: /* TODO */
	default:
		pr_err("[Error] %s : not support mode\n",
			__func__);
		return -EINVAL;
	}

	if (dlmemory->flag & DL_FLAG_HEADER_LESS)
		return vspd_dl_start_header_less(vdata, dl, num);
	else
		return vspd_dl_start_header_mode(vdata, dl, num);

}


static int vspd_dl_swap_header_mode(struct vspd_private_data *vdata,
			struct dl_head *head[], int num)
{
	struct dl_memory *dlmemory = vdata->dlmemory;
	int i;
	unsigned long flags;
	struct dl_head *free_head;

	for (i = 0; i < (num - 1); i++) {
		head[i]->flag = 0;
		head[i]->next = head[i + 1];
		vspd_dl_header_setup(dlmemory, head[i]);
	}

	head[num - 1]->flag = dlmemory->flag;
	head[num - 1]->next = head[0];
	vspd_dl_header_setup(dlmemory, head[num - 1]);

	spin_lock_irqsave(&dlmemory->lock, flags);
	free_head = dlmemory->next_header;
	dlmemory->next_header = head[0];
	spin_unlock_irqrestore(&dlmemory->lock, flags);
	vspd_dl_free_header_core(free_head);

	return 0;
}

static int vspd_dl_swap_header_less(struct vspd_private_data *vdata,
				  struct dl_body *bodies[], int num)
{
	int i;
	struct dl_memory *dlmemory = vdata->dlmemory;
	unsigned long flags;
	unsigned long stat;
	int write_enable = 1;

	for (i = 0; i < (num - 1); i++) {
		bodies[i]->next = bodies[i + 1];
		bodies[i]->flag = 0;
	}
	bodies[num - 1]->next = bodies[0];
	bodies[num - 1]->flag = DL_FLAG_BODY_DL_TALL;

	if (dlmemory->flag & DL_FLAG_BODY_WRITEBLE) {
		stat = vspd_read(vdata, VI6_DL_BODY_SIZE);
		if (!(stat & VI6_DL_BODY_SIZE_UPD0) &&
		     (dlmemory->next_body != NULL)) {
			write_enable = 0;
		}
	} else {
		write_enable = 0;
	}

	if (!write_enable) {
		spin_lock_irqsave(&dlmemory->lock, flags);
		vspd_dl_free_multi_body(dlmemory->pending_body);
		dlmemory->pending_body = bodies[0];
		spin_unlock_irqrestore(&dlmemory->lock, flags);
		return 0;
	}

	vspd_write(vdata, VI6_DL_BODY_SIZE,
		(bodies[0]->reg_count * 8) | VI6_DL_BODY_SIZE_UPD0);
	vspd_write(vdata, VI6_DL_HDR_ADDRn(USE_WPF), bodies[0]->paddr);

	spin_lock_irqsave(&dlmemory->lock, flags);
	vspd_dl_free_multi_body(dlmemory->next_body);
	dlmemory->next_body = bodies[0];
	dlmemory->active_body_next_set = bodies[0];
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	return 0;
}

int vspd_dl_swap(struct vspd_private_data *vdata, void *dl, int num)
{
	struct dl_memory *dlmemory = vdata->dlmemory;

	if (dlmemory->flag & DL_FLAG_HEADER_LESS) {
		if (dlmemory->active_body == NULL) {
			pr_err("[Error] %s : vspd not active\n", __func__);
			return -EPERM;
		}
		return vspd_dl_swap_header_less(vdata, dl, num);
	} else {
		if (dlmemory->active_header == NULL) {
			pr_err("[Error] %s : vspd not active\n", __func__);
			return -EPERM;
		}
		return vspd_dl_swap_header_mode(vdata, dl, num);
	}
}

struct dl_head *vspd_dl_get_header(struct dl_memory *dlmemory)
{
	struct dl_head *head = NULL;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&dlmemory->lock, flags);
	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		if (dlmemory->head[i].use == DL_MEM_NO_USE) {
			head = &dlmemory->head[i];
			head->use = DL_MEM_USE;
			break;
		}
	}
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	if (head == NULL)
		return NULL;

	for (i = 0; i < DISPLAY_LIST_BODY_NUM; i++)
		head->dl_body_list[i] = NULL;

	return head;
}

void vspd_dl_free_header(struct dl_memory *dlmemory, struct dl_head *head)
{
	unsigned long flags;

	spin_lock_irqsave(&dlmemory->lock, flags);
	vspd_dl_free_header_core(head);
	spin_unlock_irqrestore(&dlmemory->lock, flags);
}

struct dl_body *vspd_dl_get_body(struct dl_memory *dlmemory, int module)
{
	struct dl_body *body = NULL;
	int i;
	unsigned long flags;

	switch (module) {
	case DL_LIST_OFFSET_RPF0:
	case DL_LIST_OFFSET_RPF1:
	case DL_LIST_OFFSET_RPF2:
	case DL_LIST_OFFSET_RPF3:
	case DL_LIST_OFFSET_RPF4:
	case DL_LIST_OFFSET_CTRL:
	case DL_LIST_OFFSET_LUT:
	case DL_LIST_OFFSET_CLUT:
		break;
	default:
		return NULL;
	}

	spin_lock_irqsave(&dlmemory->lock, flags);
	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		if (dlmemory->body[i][module].use == DL_MEM_NO_USE) {
			body = &dlmemory->body[i][module];
			body->use = DL_MEM_USE;
			break;
		}
	}
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	if (body == NULL)
		return NULL;

	body->reg_count = 0;
	body->next = NULL;

	return body;
}

struct dl_body *vspd_dl_get_single_body(struct dl_memory *dlmemory)
{
	struct dl_body *body = NULL;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&dlmemory->lock, flags);
	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		if (dlmemory->single_body[i].use == DL_MEM_NO_USE) {
			body = &dlmemory->single_body[i];
			body->use = DL_MEM_USE;
			break;
		}
	}
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	if (body == NULL)
		return NULL;

	body->reg_count = 0;
	body->next = NULL;

	return body;
}

void vspd_dl_free_body(struct dl_memory *dlmemory, struct dl_body *body)
{
	unsigned long flags;

	spin_lock_irqsave(&dlmemory->lock, flags);
	body->use = DL_MEM_NO_USE;
	spin_unlock_irqrestore(&dlmemory->lock, flags);
}

int vspd_dl_set_body(struct dl_head *head, struct dl_body *body, int module)
{
	switch (module) {
	case DL_LIST_OFFSET_RPF0:
	case DL_LIST_OFFSET_RPF1:
	case DL_LIST_OFFSET_RPF2:
	case DL_LIST_OFFSET_RPF3:
	case DL_LIST_OFFSET_RPF4:
	case DL_LIST_OFFSET_CTRL:
	case DL_LIST_OFFSET_LUT:
	case DL_LIST_OFFSET_CLUT:
		head->dl_body_list[module] = body;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}



static void vspd_dl_config(struct dl_memory *dlmemory)
{
	unsigned long offset = 0;
	int i, j;
	int dl_header_size = 76;
	int dl_body_size[DISPLAY_LIST_BODY_NUM] = {
		256, /* rpf0 */
		256, /* rpf1 */
		256, /* rpf2 */
		256, /* rpf3 */
		256, /* rpf4 */
		512,  /* wpf0 + bru + uds + dpr + lif*/
		8, /* TODO:LUT */
		8, /* TODO:CLUT */
	};

	offset = (((dlmemory->paddr + 15) / 16)) * 16 - dlmemory->paddr;

	/* header config */
	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		struct dl_head *head = &dlmemory->head[i];

		head->size = dl_header_size;
		head->use = DL_MEM_NO_USE;
		head->paddr = dlmemory->paddr + offset;
		head->dheader = dlmemory->vaddr + offset;
		head->dheader_offset = offset;
		head->next = NULL;
		for (j = 0; j < DISPLAY_LIST_BODY_NUM; j++)
			head->dl_body_list[j] = NULL;

		offset += ((((head->size + 15) / 16)) * 16);
	}

	/* body config */
	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		for (j = 0; j < DISPLAY_LIST_BODY_NUM; j++) {
			struct dl_body *body = &(dlmemory->body[i][j]);

			body->size = dl_body_size[j];
			body->reg_count = 0;
			body->use = DL_MEM_NO_USE;
			body->paddr = dlmemory->paddr + offset;
			body->dlist = dlmemory->vaddr + offset;
			body->dlist_offset = offset;

			offset += ((((body->size + 7) / 8)) * 8);
		}
	}

	if (dlmemory->size < offset)
		pr_warn("[Warning] display list size over\n");


	/* header less body config */
	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		struct dl_body *single_body = &dlmemory->single_body[i];

		single_body->size = DL_MEM_SIZE;
		single_body->reg_count = 0;
		single_body->use = DL_MEM_NO_USE;
		single_body->paddr = dlmemory->paddr + DL_MEM_SIZE * i;
		single_body->dlist = dlmemory->vaddr + DL_MEM_SIZE * i;
		single_body->dlist_offset = DL_MEM_SIZE * i;
	}

}

int vspd_dl_create(struct vspd_private_data *vdata, struct device *dev)
{
	struct dl_memory *_dlmemory;
	int ret;

	_dlmemory = kzalloc(sizeof(*_dlmemory), GFP_KERNEL);
	if (_dlmemory == NULL) {
		pr_err("[Error] %s : struct dl_mem allocate\n",
			__func__);
		ret = -ENOMEM;
		goto error1;
	}

	_dlmemory->vaddr =
		dma_alloc_writecombine(dev, DL_MEM_SIZE * DISPLAY_LIST_NUM,
			&_dlmemory->paddr, GFP_KERNEL);
	if (!_dlmemory->vaddr) {
		pr_err("[Error] %s : dma_alloc_writecombine\n",
			 __func__);
		ret = -ENOMEM;
		goto error2;
	}

	_dlmemory->size = DL_MEM_SIZE * DISPLAY_LIST_NUM;
	_dlmemory->active_header = NULL;
	_dlmemory->next_header = NULL;
	_dlmemory->flag = 0;
	_dlmemory->active_body = NULL;
	_dlmemory->next_body = NULL;
	_dlmemory->pending_body = NULL;
	spin_lock_init(&_dlmemory->lock);
	vspd_dl_config(_dlmemory);

	vdata->dlmemory = _dlmemory;

	vdata->dev = dev;

	return 0;

error2:
	dma_free_writecombine(dev, DL_MEM_SIZE,
			_dlmemory->vaddr, _dlmemory->paddr);

	kfree(_dlmemory);
error1:
	return ret;
}


void vspd_dl_destroy(struct vspd_private_data *vdata)
{
	dma_free_writecombine(vdata->dev, DL_MEM_SIZE,
			vdata->dlmemory->vaddr, vdata->dlmemory->paddr);
	kfree(vdata->dlmemory);

}

MODULE_LICENSE("GPL");


