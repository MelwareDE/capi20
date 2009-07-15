/*
 * dsp_pipeline.c: pipelined audio processing
 *
 * Copyright (C) 2007, Nadi Sarrar
 *
 * Nadi Sarrar <nadi@beronet.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 */

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/mISDNif.h>
#include <linux/mISDNdsp.h>
#include "layer1.h"
#include "dsp.h"

/* uncomment for debugging */
/*#define PIPELINE_DEBUG*/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
extern struct mISDN_dsp_element *dsp_hwec;
#else
extern mISDN_dsp_element_t *dsp_hwec;
#endif

extern void dsp_hwec_enable          (dsp_t *dsp, const char *arg);
extern void dsp_hwec_disable         (dsp_t *dsp);
extern int  dsp_hwec_init            (void);
extern void dsp_hwec_exit            (void);

typedef struct _dsp_pipeline_entry {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	struct mISDN_dsp_element *elem;
#else
	mISDN_dsp_element_t *elem;
#endif
	void                *p;
	struct list_head     list;
} dsp_pipeline_entry_t;

typedef struct _dsp_element_entry {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	struct mISDN_dsp_element *elem;
	struct device  dev;

#elif LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 26)
	mISDN_dsp_element_t *elem;
	struct device dev;

#else
	mISDN_dsp_element_t *elem;
	struct class_device dev;
#endif
	struct list_head     list;
} dsp_element_entry_t;

static rwlock_t dsp_elements_lock;
static LIST_HEAD(dsp_elements);

/* sysfs */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
static void elements_release(struct device *dev) {}
#else
static void elements_class_release (struct class_device *dev) {}
#endif

static struct class elements_class = {
	.name = "mISDN-dsp-elements",
#ifndef CLASS_WITHOUT_OWNER
	.owner = THIS_MODULE,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	.dev_release = &elements_release,
#else
	.release = &elements_class_release,
#endif
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
static ssize_t attr_show_args (struct device *dev, struct device_attribute *attr, char *buf) {
	struct mISDN_dsp_element *elem = dev_get_drvdata(dev);

#elif LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 26)
static ssize_t attr_show_args (struct device *dev, struct device_attribute *attr, char *buf) {
        mISDN_dsp_element_t *elem = dev_get_drvdata(dev);

#else
static ssize_t attr_show_args (struct class_device *dev, char *buf) {
	mISDN_dsp_element_t *elem = class_get_devdata(dev);
#endif
	ssize_t len = 0;
	int i = 0;

#if LINUX_VERSION_CODE != KERNEL_VERSION(2, 6, 26)
	*buf = 0;
#endif
	for (i = 0; i < elem->num_args; ++i) {
		len = sprintf(buf, "%sName:        %s\n%s%s%sDescription: %s\n\n", buf,
					  elem->args[i].name,
					  elem->args[i].def ? "Default:     " : "",
					  elem->args[i].def ? elem->args[i].def : "",
					  elem->args[i].def ? "\n" : "",
					  elem->args[i].desc);
	}

	return(len);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
static struct device_attribute element_attributes[] = {
        __ATTR(args, 0444, attr_show_args, NULL),
};

int mISDN_dsp_element_register (struct mISDN_dsp_element *elem) {

#elif LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 26)
static struct device_attribute element_attributes[] = {
        __ATTR(args, 0444, attr_show_args, NULL),
};

int mISDN_dsp_element_register (mISDN_dsp_element_t *elem) {

#else
static struct class_device_attribute element_attributes[] = {
	__ATTR(args, 0444, attr_show_args, NULL),
};

int mISDN_dsp_element_register (mISDN_dsp_element_t *elem) {
#endif
	dsp_element_entry_t *entry;
	u_long flags;
	int re, i;

	if (!elem)
		return -EINVAL;

	entry = kzalloc(sizeof(dsp_element_entry_t), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->elem = elem;

	entry->dev.class = &elements_class;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	dev_set_drvdata(&entry->dev, elem);
	
	snprintf(entry->dev.bus_id, BUS_ID_SIZE, elem->name);
        if ((re = device_register(&entry->dev)))
                goto err1;

        for (i = 0; i < (sizeof(element_attributes) / sizeof(struct device_attribute)); ++i)
                if ((re = device_create_file(&entry->dev, &element_attributes[i])))
			goto err2;
#else
	class_set_devdata(&entry->dev, elem);

	snprintf(entry->dev.class_id, BUS_ID_SIZE, elem->name);
	if ((re = class_device_register(&entry->dev)))
		goto err1;

	for (i = 0; i < (sizeof(element_attributes) / sizeof(struct class_device_attribute)); ++i)
		if ((re = class_device_create_file(&entry->dev, &element_attributes[i])))
			goto err2;
#endif

	write_lock_irqsave(&dsp_elements_lock, flags);
	list_add_tail(&entry->list, &dsp_elements);
	write_unlock_irqrestore(&dsp_elements_lock, flags);
	
	printk(KERN_DEBUG "%s: %s registered\n", __FUNCTION__, elem->name);

	return 0;

err2:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	device_unregister(&entry->dev);
#else
	class_device_unregister(&entry->dev);
#endif
err1:
	kfree(entry);
	return re;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
void mISDN_dsp_element_unregister (struct mISDN_dsp_element *elem) {

#else
void mISDN_dsp_element_unregister (mISDN_dsp_element_t *elem) {
#endif
	dsp_element_entry_t *entry, *n;
	u_long flags;

	if (!elem)
		return;

	write_lock_irqsave(&dsp_elements_lock, flags);
	
	list_for_each_entry_safe(entry, n, &dsp_elements, list)
		if (entry->elem == elem) {
			list_del(&entry->list);
			write_unlock_irqrestore(&dsp_elements_lock, flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
			device_unregister(&entry->dev);
#else
			class_device_unregister(&entry->dev);
#endif
			kfree(entry);
			printk(KERN_DEBUG "%s: %s unregistered\n", __FUNCTION__, elem->name);
			return;
		}
	
	write_unlock_irqrestore(&dsp_elements_lock, flags);
}

int dsp_pipeline_module_init (void)
{
	int re;

	rwlock_init(&dsp_elements_lock);

	if ((re = class_register(&elements_class)))
		return re;

	printk(KERN_DEBUG "%s: dsp pipeline module initialized\n", __FUNCTION__);

	dsp_hwec_init();

	return 0;
}

void dsp_pipeline_module_exit (void)
{
	dsp_element_entry_t *entry, *n;
	u_long flags;

	dsp_hwec_exit();

	class_unregister(&elements_class);

	write_lock_irqsave(&dsp_elements_lock, flags);
	list_for_each_entry_safe(entry, n, &dsp_elements, list) {
		list_del(&entry->list);
		printk(KERN_DEBUG "%s: element was still registered: %s\n", __FUNCTION__, entry->elem->name);
		kfree(entry);
	}
	write_unlock_irqrestore(&dsp_elements_lock, flags);
	
	printk(KERN_DEBUG "%s: dsp pipeline module exited\n", __FUNCTION__);
}

int dsp_pipeline_init (dsp_pipeline_t *pipeline)
{
	if (!pipeline)
		return -EINVAL;

	INIT_LIST_HEAD(&pipeline->list);
	rwlock_init(&pipeline->lock);

#ifdef PIPELINE_DEBUG
	printk(KERN_DEBUG "%s: dsp pipeline ready\n", __FUNCTION__);
#endif

	return 0;
}

static inline void _dsp_pipeline_destroy (dsp_pipeline_t *pipeline)
{
	dsp_pipeline_entry_t *entry, *n;

	list_for_each_entry_safe(entry, n, &pipeline->list, list) {
		list_del(&entry->list);
		if (entry->elem == dsp_hwec)
			dsp_hwec_disable(container_of(pipeline, dsp_t, pipeline));
		else
			entry->elem->free(entry->p);
		kfree(entry);
	}
}

void dsp_pipeline_destroy (dsp_pipeline_t *pipeline)
{
	u_long flags;

	if (!pipeline)
		return;

	write_lock_irqsave(&pipeline->lock, flags);
	_dsp_pipeline_destroy(pipeline);
	write_unlock_irqrestore(&pipeline->lock, flags);

#ifdef PIPELINE_DEBUG
	printk(KERN_DEBUG "%s: dsp pipeline destroyed\n", __FUNCTION__);
#endif
}

int dsp_pipeline_build (dsp_pipeline_t *pipeline, const char *cfg)
{
	int len, incomplete = 0, found = 0;
	char *dup, *tok, *name, *args;
	dsp_element_entry_t *entry, *n;
	dsp_pipeline_entry_t *pipeline_entry;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	struct mISDN_dsp_element *elem;
#else
	mISDN_dsp_element_t *elem;
#endif
	u_long elements_flags, pipeline_flags;

	if (!pipeline)
		return -EINVAL;

	write_lock_irqsave(&pipeline->lock, pipeline_flags);
	if (!list_empty(&pipeline->list))
		_dsp_pipeline_destroy(pipeline);

	if (!cfg) {
		write_unlock_irqrestore(&pipeline->lock, pipeline_flags);
		return 0;
	}

	len = strlen(cfg);
	if (!len) {
		write_unlock_irqrestore(&pipeline->lock, pipeline_flags);
		return 0;
	}

	dup = kmalloc(len + 1, GFP_KERNEL);
	if (!dup) {
		write_unlock_irqrestore(&pipeline->lock, pipeline_flags);
		return 0;
	}
	strcpy(dup, cfg);
	while ((tok = strsep(&dup, "|"))) {
		if (!strlen(tok))
			continue;
		name = strsep(&tok, "(");
		args = strsep(&tok, ")");
		if (args && !*args)
			args = 0;

		read_lock_irqsave(&dsp_elements_lock, elements_flags);
		list_for_each_entry_safe(entry, n, &dsp_elements, list)
			if (!strcmp(entry->elem->name, name)) {
				elem = entry->elem;
				read_unlock_irqrestore(&dsp_elements_lock, elements_flags);

				pipeline_entry = kmalloc(sizeof(dsp_pipeline_entry_t), GFP_KERNEL);
				if (!pipeline_entry) {
					printk(KERN_DEBUG "%s: failed to add entry to pipeline: %s (out of memory)\n", __FUNCTION__, elem->name);
					incomplete = 1;
					goto _out;
				}
				pipeline_entry->elem = elem;

				if (elem == dsp_hwec) {
					/* This is a hack to make the hwec available as a pipeline module */
					dsp_hwec_enable(container_of(pipeline, dsp_t, pipeline), args);
					list_add_tail(&pipeline_entry->list, &pipeline->list);
				} else {
					pipeline_entry->p = elem->new(args);
					if (pipeline_entry->p) {
						list_add_tail(&pipeline_entry->list, &pipeline->list);
#ifdef PIPELINE_DEBUG
						printk(KERN_DEBUG "%s: created instance of %s%s%s\n", __FUNCTION__, name, args ? " with args " : "", args ? args : "");
#endif
					} else {
						printk(KERN_DEBUG "%s: failed to add entry to pipeline: %s (new() returned NULL)\n", __FUNCTION__, elem->name);
						kfree(pipeline_entry);
						incomplete = 1;
					}
				}
				found = 1;
				break;
			}

		if (found)
			found = 0;
		else {
			read_unlock_irqrestore(&dsp_elements_lock, elements_flags);
			printk(KERN_DEBUG "%s: element not found, skipping: %s\n", __FUNCTION__, name);
			incomplete = 1;
		}
	}

_out:
	if (!list_empty(&pipeline->list)) 
		pipeline->inuse=1;
	else
		pipeline->inuse=0;

	write_unlock_irqrestore(&pipeline->lock, pipeline_flags);
#ifdef PIPELINE_DEBUG
	printk(KERN_DEBUG "%s: dsp pipeline built%s: %s\n", __FUNCTION__, incomplete ? " incomplete" : "", cfg);
#endif
	kfree(dup);
	return 0;
}

void dsp_pipeline_process_tx (dsp_pipeline_t *pipeline, u8 *data, int len)
{
	dsp_pipeline_entry_t *entry;

	if (!pipeline)
		return;

	if (!read_trylock(&pipeline->lock)) {
		printk(KERN_DEBUG "%s: bypassing pipeline because it is locked (TX)\n", __FUNCTION__);
		return;
	}
	list_for_each_entry(entry, &pipeline->list, list)
		if (entry->elem->process_tx)
			entry->elem->process_tx(entry->p, data, len);
	read_unlock(&pipeline->lock);
}

void dsp_pipeline_process_rx (dsp_pipeline_t *pipeline, u8 *data, int len)
{
	dsp_pipeline_entry_t *entry;

	if (!pipeline)
		return;

	if (!read_trylock(&pipeline->lock)) {
		printk(KERN_DEBUG "%s: bypassing pipeline because it is locked (RX)\n", __FUNCTION__);
		return;
	}
	list_for_each_entry_reverse(entry, &pipeline->list, list)
		if (entry->elem->process_rx)
			entry->elem->process_rx(entry->p, data, len);
	read_unlock(&pipeline->lock);
}

EXPORT_SYMBOL(mISDN_dsp_element_register);
EXPORT_SYMBOL(mISDN_dsp_element_unregister);

