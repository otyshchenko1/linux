
#include "vspd_drv_private.h"

#define DL_MEM_SIZE 2048

enum {
	DL_MEM_NO_USE,
	DL_MEM_SET,
	DL_MEM_ACTIVE,
};


#define	DL_FLAG_NONE		0x00000000
#define	DL_FLAG_INTERRUPT	0x00000001
#define	DL_FLAG_AUTO_REPEAT	0x00000002
#define	DL_FLAG_MANUAL_REPEAT	0x00000004

inline void vspd_dl_free_header_core(struct dl_head *head)
{
	int i;
	struct dl_head *free_head = head;
	struct dl_head *tmp_head;

	if (head == NULL)
		return;

	free_head = head;

	do {
		for (i = 0; i < DISPLAY_LIST_BODY_NUM; i++)
			if (free_head->dl_body_list[i] != NULL)
				free_head->dl_body_list[i]->use = DL_MEM_NO_USE;

		free_head->use = DL_MEM_NO_USE;
		tmp_head = free_head->next;
		free_head->next = NULL;

		free_head = tmp_head;

	} while ((free_head != NULL)  && (free_head != head));
}

void vspd_dl_reset(struct vspd_private_data *vdata)
{
	struct dl_memory *dlmemory = vdata->dlmemory;
	int i, j;
	unsigned long flags;

	spin_lock_irqsave(&dlmemory->lock, flags);

	dlmemory->active_header = NULL;
	dlmemory->next_header = NULL;

	for (i = 0; i < DISPLAY_LIST_NUM; i++) {
		vspd_dl_free_header_core(&dlmemory->head[i]);

		for (j = 0; j < DISPLAY_LIST_BODY_NUM; j++) {
			dlmemory->body[i][j].use = DL_MEM_NO_USE;
			dlmemory->body[i][j].reg_count = 0;
		}
	}

	spin_unlock_irqrestore(&dlmemory->lock, flags);
}

int vspd_dl_need_cmd(struct vspd_private_data *vdata)
{
	struct dl_memory *dlmemory = vdata->dlmemory;

	if (vdata->dlmemory == NULL)
		return 0;

	return !!(dlmemory->flag & DL_FLAG_MANUAL_REPEAT);
}

int vspd_dl_next_dl(struct vspd_private_data *vdata, unsigned long *addr)
{
	struct dl_memory *dlmemory = vdata->dlmemory;

	*addr = 0;

	spin_lock(&dlmemory->lock);

	if (dlmemory->flag & DL_FLAG_MANUAL_REPEAT) {
		if (dlmemory->next_header != NULL) {
			*addr = dlmemory->next_header->paddr;
			vspd_dl_free_header_core(dlmemory->active_header);
			dlmemory->next_header->use = DL_MEM_ACTIVE;
			dlmemory->active_header = dlmemory->next_header;
			dlmemory->next_header = NULL;
		}
	} else {
		vspd_dl_free_header_core(dlmemory->active_header);
		dlmemory->active_header = NULL;
	}

	spin_unlock(&dlmemory->lock);

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

unsigned long vspd_dl_start_setup(struct dl_memory *dlmemory,
			struct dl_head *head[], int num, int auto_flag)
{
	int i;
	unsigned long flags;

	if (dlmemory->active_header != NULL) {
		pr_err("[Error] %s : vspd active\n",
			__func__);
		return 0;
	}


	dlmemory->flag = 0;
	for (i = 0; i < (num - 1); i++) {
		head[i]->flag = 0;
		head[i]->next = head[i + 1];
		if (vspd_dl_header_setup(dlmemory, head[i]))
			return 0;
		head[i]->use = DL_MEM_ACTIVE;
	}

	switch (auto_flag) {
	case 0:
		head[num - 1]->flag = DL_FLAG_INTERRUPT;
		break;
	case 1:
		head[num - 1]->flag = DL_FLAG_INTERRUPT | DL_FLAG_AUTO_REPEAT;
		break;
	case 2:
		head[num - 1]->flag = DL_FLAG_INTERRUPT | DL_FLAG_MANUAL_REPEAT;
		break;
	}
	head[num - 1]->next = head[0];

	if (vspd_dl_header_setup(dlmemory, head[num - 1]))
		return 0;
	head[num - 1]->use = DL_MEM_ACTIVE;

	dlmemory->flag = head[num - 1]->flag;

	spin_lock_irqsave(&dlmemory->lock, flags);
	dlmemory->active_header = head[0];
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	return head[0]->paddr;
}


int vspd_dl_next_setup(struct dl_memory *dlmemory,
			struct dl_head *head[], int num)
{
	int i;
	unsigned long flags;

	if (dlmemory->active_header == NULL) {
		pr_err("[Error] %s : vspd not active\n",
			__func__);
		return -1;
	}

	if (!(dlmemory->flag & (DL_FLAG_AUTO_REPEAT | DL_FLAG_MANUAL_REPEAT))) {
		pr_err("[Error] %s : vspd busy(no repeat mode)\n", __func__);
		return -1;
	}

	for (i = 0; i < (num - 1); i++) {
		head[i]->flag = 0;
		head[i]->next = head[i + 1];
		if (vspd_dl_header_setup(dlmemory, head[i]))
			return 0;
		head[i]->use = DL_MEM_ACTIVE;
	}

	head[num - 1]->flag = dlmemory->flag;
	head[num - 1]->next = head[0];
	if (vspd_dl_header_setup(dlmemory, head[num - 1]))
		return 0;

	head[num - 1]->use = DL_MEM_ACTIVE;

	spin_lock_irqsave(&dlmemory->lock, flags);
	vspd_dl_free_header_core(dlmemory->next_header);
	dlmemory->next_header = head[0];
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	return 0;
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
			head->use = DL_MEM_SET;
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
			body->use = DL_MEM_SET;
			break;
		}
	}
	spin_unlock_irqrestore(&dlmemory->lock, flags);

	if (body == NULL)
		return NULL;

	body->reg_count = 0;

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
		return -1;
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


