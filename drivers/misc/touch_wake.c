/* drivers/misc/touch_wake.c
 *
 * Copyright 2011  Ezekeel
 * Modified by Renaud Allard (2012)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/touch_wake.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>

extern void touchscreen_enable(void);
extern void touchscreen_disable(void);

static bool touchwake_enabled = true;
static bool touch_disabled = false;
static bool device_suspended = false;
static bool timed_out = true;
static bool always_wake_enabled = false;
static unsigned int touchoff_delay = (30 * 1000);
static const unsigned int presspower_delay = 100;
static void touchwake_touchoff(struct work_struct * touchoff_work);
static DECLARE_DELAYED_WORK(touchoff_work, touchwake_touchoff);
static void press_powerkey(struct work_struct * presspower_work);
static DECLARE_WORK(presspower_work, press_powerkey);
static DEFINE_MUTEX(lock);
static struct input_dev * powerkey_device;
static struct timeval last_powerkeypress;

#define TOUCHWAKE_VERSION 2
#define TIME_LONGPRESS 400

static void touchwake_disable_touch(void)
{
    pr_info("disable touch controls\n");
    touchscreen_disable();
    touch_disabled = true;
    return;
}

static void touchwake_enable_touch(void)
{
    pr_info("enable touch controls\n");
    touchscreen_enable();
    touch_disabled = false;
    return;
}

static void touchwake_early_suspend(struct early_suspend * h)
{
    if (touchwake_enabled)
	{
	if (!always_wake_enabled) {
		    if (touchoff_delay > 0)
			{
			    if (timed_out)
				    schedule_delayed_work(&touchoff_work, msecs_to_jiffies(touchoff_delay));
			    else
				    touchwake_disable_touch();
			}
		}
	}
    else
	{
	    touchwake_disable_touch();
	}

    device_suspended = true;
    return;
}

static void touchwake_late_resume(struct early_suspend * h)
{
    cancel_delayed_work(&touchoff_work);
    flush_scheduled_work();

    if (touch_disabled)
	{
	    touchwake_enable_touch();
	} 

    timed_out = true;
    device_suspended = false;
    return;
}

static struct early_suspend touchwake_suspend_data = 
    {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = touchwake_early_suspend,
	.resume = touchwake_late_resume,
    };

static void touchwake_touchoff(struct work_struct * touchoff_work)
{
    touchwake_disable_touch();
    return;
}

static void press_powerkey(struct work_struct * presspower_work)
{
    input_event(powerkey_device, EV_KEY, KEY_POWER, 1);
    input_event(powerkey_device, EV_SYN, 0, 0);
    msleep(presspower_delay);

    input_event(powerkey_device, EV_KEY, KEY_POWER, 0);
    input_event(powerkey_device, EV_SYN, 0, 0);
    msleep(presspower_delay);

    mutex_unlock(&lock);

    return;
}

static ssize_t always_wake_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", (always_wake_enabled ? 1 : 0));
}

static ssize_t always_wake_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%u\n", &data) == 1) 
	{
	    pr_devel("%s: %u \n", __FUNCTION__, data);
	    
	    if (data == 1) 
		{
		    pr_info("%s: Always wake enabled\n", __FUNCTION__);
		    always_wake_enabled = true;
		} 
	    else if (data == 0) 
		{
		    pr_info("%s: Always wake disabled\n", __FUNCTION__);
		    always_wake_enabled = false;
		} 
	    else 
		{
		    pr_info("%s: invalid input range %u\n", __FUNCTION__, data);
		}
	} 
    else 
	{
	    pr_info("%s: invalid input\n", __FUNCTION__);
	}

    return size;
}

static ssize_t touchwake_status_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", (touchwake_enabled ? 1 : 0));
}

static ssize_t touchwake_status_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%u\n", &data) == 1) 
	{
	    pr_devel("%s: %u \n", __FUNCTION__, data);
	    
	    if (data == 1) 
		{
		    pr_info("%s: TOUCHWAKE function enabled\n", __FUNCTION__);
		    touchwake_enabled = true;
		} 
	    else if (data == 0) 
		{
		    pr_info("%s: TOUCHWAKE function disabled\n", __FUNCTION__);
		    touchwake_enabled = false;
		} 
	    else 
		{
		    pr_info("%s: invalid input range %u\n", __FUNCTION__, data);
		}
	} 
    else 
	{
	    pr_info("%s: invalid input\n", __FUNCTION__);
	}

    return size;
}

static ssize_t touchwake_delay_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", touchoff_delay);
}

static ssize_t touchwake_delay_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%u\n", &data) == 1) 
	{
	    touchoff_delay = data;
	    pr_info("TOUCHWAKE delay set to %u\n", touchoff_delay); 
	} 
    else 
	{
	    pr_info("%s: invalid input\n", __FUNCTION__);
	}

    return size;
}

static ssize_t touchwake_version(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", TOUCHWAKE_VERSION);
}

static DEVICE_ATTR(enabled, S_IRUGO | S_IWUGO, touchwake_status_read, touchwake_status_write);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUGO, touchwake_delay_read, touchwake_delay_write);
static DEVICE_ATTR(always, S_IRUGO | S_IWUGO, always_wake_read, always_wake_write);
static DEVICE_ATTR(version, S_IRUGO , touchwake_version, NULL);

static struct attribute *touchwake_notification_attributes[] = 
    {
	&dev_attr_enabled.attr,
	&dev_attr_delay.attr,
	&dev_attr_version.attr,
	&dev_attr_always.attr,
	NULL
    };

static struct attribute_group touchwake_notification_group = 
    {
	.attrs  = touchwake_notification_attributes,
    };

static struct miscdevice touchwake_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touchwake",
    };

void proximity_detected(void)
{   
    timed_out = false;

    return;
}
EXPORT_SYMBOL(proximity_detected);

void powerkey_pressed(void)
{   
    do_gettimeofday(&last_powerkeypress);

    return;
}
EXPORT_SYMBOL(powerkey_pressed);

void powerkey_released(void)
{   
    struct timeval now;
    int time_pressed;

    do_gettimeofday(&now);
    time_pressed = (now.tv_sec - last_powerkeypress.tv_sec) * MSEC_PER_SEC +
	(now.tv_usec - last_powerkeypress.tv_usec) / USEC_PER_MSEC;

    if (time_pressed < TIME_LONGPRESS)
	{
	    timed_out = false;
	}

    return;
}
EXPORT_SYMBOL(powerkey_released);

void touch_press(void)
{   
    if (device_suspended && touchwake_enabled && mutex_trylock(&lock))
	{
	    schedule_work(&presspower_work);
	}

    return;
}
EXPORT_SYMBOL(touch_press);

void set_powerkeydev(struct input_dev * input_device)
{   
    powerkey_device = input_device;
    return;
}
EXPORT_SYMBOL(set_powerkeydev);

bool device_is_suspended(void)
{   
    return device_suspended;
}
EXPORT_SYMBOL(device_is_suspended);

static int __init touchwake_control_init(void)
{
    int ret;
    pr_info("%s misc_register(%s)\n", __FUNCTION__, touchwake_device.name);
    ret = misc_register(&touchwake_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, touchwake_device.name);
	    return 1;
	}

    if (sysfs_create_group(&touchwake_device.this_device->kobj, &touchwake_notification_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", touchwake_device.name);
	}

    register_early_suspend(&touchwake_suspend_data);
    do_gettimeofday(&last_powerkeypress);
    return 0;
}

device_initcall(touchwake_control_init);
