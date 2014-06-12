/*
 * linux/drivers/mmc/core/lock.c
 *
 * Copyright 2006 Instituto Nokia de Tecnologia (INdT), All Rights Reserved.
 * Copyright 2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * MMC password key handling.
 */

#include <linux/device.h>
#include <linux/key-type.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>

#include "mmc_ops.h"
#include "lock.h"

#define MMC_KEYLEN_MAXBYTES 32

static int mmc_key_instantiate(struct key *key, struct key_preparsed_payload *prep)
{
	struct mmc_key_payload *mpayload;
	int ret;

	ret = -EINVAL;
	if (prep->datalen <= 0 ||
	    prep->datalen > MMC_KEYLEN_MAXBYTES || !prep->data) {
		pr_debug("Invalid data\n");
		goto error;
	}

	ret = key_payload_reserve(key, prep->datalen);
	if (ret < 0) {
		pr_debug("ret = %d\n", ret);
		goto error;
	}

	ret = -ENOMEM;
	mpayload = kmalloc(sizeof(*mpayload) + prep->datalen, GFP_KERNEL);
	if (!mpayload) {
		pr_debug("Unable to allocate mpayload structure\n");
		goto error;
	}
	mpayload->datalen = prep->datalen;
	memcpy(mpayload->data, prep->data, prep->datalen);

	rcu_assign_pointer(key->payload.data, mpayload);

	/* ret = 0 if there is no error */
	ret = 0;

error:
	return ret;
}

static int mmc_key_match(const struct key *key, const void *description)
{
	return strcmp(key->description, description) == 0;
}

/*
 * dispose of the data dangling from the corpse of a mmc key
 */
static void mmc_key_destroy(struct key *key)
{
	struct mmc_key_payload *mpayload = key->payload.data;

	kfree(mpayload);
}

static struct key_type mmc_key_type = {
	.name = "mmc",
	.def_datalen = MMC_KEYLEN_MAXBYTES,
	.instantiate = mmc_key_instantiate,
	.match = mmc_key_match,
	.destroy = mmc_key_destroy,
};

int mmc_register_key_type(void)
{
	return register_key_type(&mmc_key_type);
}

void mmc_unregister_key_type(void)
{
	unregister_key_type(&mmc_key_type);
}

static ssize_t
mmc_lockable_show(struct device *dev, struct device_attribute *att, char *buf)
{
	struct mmc_card *card = dev_to_mmc_card(dev);

	return sprintf(buf, "%slocked\n", mmc_card_locked(card) ? "" : "un");
}

/*
 * implement MMC password functions: force erase, remove password, change
 * password, unlock card and assign password.
 */
static ssize_t
mmc_lockable_store(struct device *dev, struct device_attribute *att,
 const char *data, size_t len)
{
	struct mmc_card *card = dev_to_mmc_card(dev);
	int ret;
	struct key *mmc_key;

	WARN_ON(!(mmc_card_mmc(card) || mmc_card_sd(card)));
	WARN_ON(!(card->csd.cmdclass & CCC_LOCK_CARD));

	if(!data)
		return -EINVAL;
	if((card->type != MMC_TYPE_MMC) && (card->type != MMC_TYPE_SD))
		return -EINVAL;
	if(!(card->csd.cmdclass & CCC_LOCK_CARD))
		return -EINVAL;

	mmc_claim_host(card->host);

	ret = -EINVAL;
	if (mmc_card_locked(card) && !strncmp(data, "erase", 5)) {
		/* forced erase only works while card is locked */
		mmc_lock_unlock(card, NULL, MMC_LOCK_MODE_ERASE);
		ret = len;
	} else if (!mmc_card_locked(card) && !strncmp(data, "remove", 6)) {
		/* remove password only works while card is unlocked */
		mmc_key = request_key(&mmc_key_type, "mmc:key", "remove");

		if (!IS_ERR(mmc_key)) {
			ret = mmc_lock_unlock(card, mmc_key, MMC_LOCK_MODE_CLR_PWD);
			if (!ret)
				ret = len;
		} else
			dev_dbg(&card->dev,
				"request_key returned error %ld\n",
				PTR_ERR(mmc_key));
	} else if (!mmc_card_locked(card) && ((!strncmp(data, "assign", 6)) ||
				(!strncmp(data, "change", 6)))) {
		/* assign or change */
		if(!(strncmp(data, "assign", 6)))
			mmc_key = request_key(&mmc_key_type, "mmc:key", "assign");
		else
			mmc_key = request_key(&mmc_key_type, "mmc:key", "change");

		if (!IS_ERR(mmc_key)) {
			ret = mmc_lock_unlock(card, mmc_key, MMC_LOCK_MODE_SET_PWD);
			if (!ret)
				ret = len;
		} else
			dev_dbg(&card->dev,
				"request_key returned error %ld\n",
				PTR_ERR(mmc_key));
	} else if (mmc_card_locked(card) && !strncmp(data, "unlock", 6)) {
		/* unlock */
		mmc_key = request_key(&mmc_key_type, "mmc:key", "unlock");
		if (!IS_ERR(mmc_key)) {
			ret = mmc_lock_unlock(card, mmc_key, MMC_LOCK_MODE_UNLOCK);
			if (ret) {
				dev_dbg(&card->dev, "Wrong password\n");
				ret = -EINVAL;
			}
			else {
				mmc_release_host(card->host);
				device_release_driver(dev);
				ret = device_attach(dev);
				if(!ret)
					return -EINVAL;
				else
					return len;
			}
		} else
			dev_dbg(&card->dev,
				"request_key returned error %ld\n",
				PTR_ERR(mmc_key));
	}

	mmc_release_host(card->host);
	return ret;
}


DEVICE_ATTR(lockable, S_IWUSR | S_IRUGO, mmc_lockable_show, mmc_lockable_store);
