// SPDX-License-Identifier: GPL-2.0
/*
 * Longcheer/Xiaomi audio info for Redmi Note 13 4G (sapphire).
 *
 * Exposes /sys/android_audio/{audio_codec,audio_pa} so userspace (factory
 * tools, audio HAL) can tell which codec and which of the dual-sourced
 * speaker amplifiers (FourSemi fs1815 / Awinic aw87xxx) is fitted.
 *
 * Based on the Longcheer driver published by Xiaomi (Copyright (c) 2022
 * XiaoMi Inc.), trimmed to match the stock sapphire lct_audio_info_dlkm.ko
 * (no bootargs/hwname helpers, no "already exists" check).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/string.h>

#define AUDIO_INFO_NAME_MAX	(64)

struct lct_audio_info {
	char codec_name[AUDIO_INFO_NAME_MAX];
	char pa_name[AUDIO_INFO_NAME_MAX];
};

static DEFINE_MUTEX(g_audio_info_mutex_lock);
static struct lct_audio_info g_audio_info;
static struct kobject *android_audio;

static ssize_t audio_codec_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	if (strlen(g_audio_info.codec_name) == 0)
		return sprintf(buf, "%s\n", "WCD9370");
	return sprintf(buf, "%s\n", g_audio_info.codec_name);
}

static ssize_t audio_pa_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	if (strlen(g_audio_info.pa_name) == 0)
		return sprintf(buf, "%s\n", "unkonw_pa");
	return sprintf(buf, "%s\n", g_audio_info.pa_name);
}

static struct kobj_attribute dev_attr_audio_codec =
	__ATTR(audio_codec, 0644, audio_codec_show, NULL);
static struct kobj_attribute dev_attr_audio_pa =
	__ATTR(audio_pa, 0644, audio_pa_show, NULL);

int lct_audio_info_create_sysfs(void)
{
	int ret;

	android_audio = kobject_create_and_add("android_audio", NULL);
	if (android_audio == NULL) {
		pr_err(" android_audio_create_sysfs_ failed\n");
		return -ENOMEM;
	}

	ret = sysfs_create_file(android_audio, &dev_attr_audio_codec.attr);
	if (ret) {
		pr_err("%s audio_codec failed \n", __func__);
		kobject_del(android_audio);
		return ret;
	}

	ret = sysfs_create_file(android_audio, &dev_attr_audio_pa.attr);
	if (ret) {
		pr_err("%s audio_pa failed \n", __func__);
		kobject_del(android_audio);
		return ret;
	}

	g_audio_info.codec_name[AUDIO_INFO_NAME_MAX - 1] = '\0';
	g_audio_info.pa_name[AUDIO_INFO_NAME_MAX - 1] = '\0';

	return 0;
}
EXPORT_SYMBOL(lct_audio_info_create_sysfs);

int lct_audio_info_set_codec_name(const char *codec_name, int count)
{
	int max_count = count > AUDIO_INFO_NAME_MAX ? AUDIO_INFO_NAME_MAX : count;

	mutex_lock(&g_audio_info_mutex_lock);
	strncpy(g_audio_info.codec_name, codec_name, max_count);
	mutex_unlock(&g_audio_info_mutex_lock);
	return strlen(g_audio_info.codec_name);
}
EXPORT_SYMBOL(lct_audio_info_set_codec_name);

/* appends: both amplifiers may report themselves */
int lct_audio_info_set_pa_name(const char *pa_name, int count)
{
	int max_count = count > AUDIO_INFO_NAME_MAX ? AUDIO_INFO_NAME_MAX : count;

	mutex_lock(&g_audio_info_mutex_lock);
	strncat(g_audio_info.pa_name, pa_name, max_count);
	g_audio_info.pa_name[AUDIO_INFO_NAME_MAX - 1] = '\0';
	mutex_unlock(&g_audio_info_mutex_lock);
	return strlen(g_audio_info.pa_name);
}
EXPORT_SYMBOL(lct_audio_info_set_pa_name);

MODULE_DESCRIPTION("lct audio pa info driver");
MODULE_LICENSE("GPL v2");
