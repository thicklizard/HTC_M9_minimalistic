/*
 * Driver model for leds and led triggers
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_LEDS_H_INCLUDED
#define __LINUX_LEDS_H_INCLUDED

#include <linux/list.h>
<<<<<<< HEAD
=======
#include <linux/mutex.h>
#include <linux/rwsem.h>
>>>>>>> 0e91d2a... Nougat
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

struct device;

enum led_brightness {
	LED_OFF		= 0,
	LED_HALF	= 127,
	LED_FULL	= 255,
};

struct led_classdev {
	const char		*name;
	int			 brightness;
	int			 max_brightness;
	int			 usr_brightness_req;
	int			 flags;

	
#define LED_SUSPENDED		(1 << 0)
	
#define LED_CORE_SUSPENDRESUME	(1 << 16)
#define LED_BLINK_ONESHOT	(1 << 17)
#define LED_BLINK_ONESHOT_STOP	(1 << 18)
#define LED_BLINK_INVERT	(1 << 19)
#define LED_SYSFS_DISABLE	(1 << 20)

	
	
	void		(*brightness_set)(struct led_classdev *led_cdev,
					  enum led_brightness brightness);
	
	enum led_brightness (*brightness_get)(struct led_classdev *led_cdev);

	int		(*blink_set)(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off);

	struct device		*dev;
	struct list_head	 node;			
	const char		*default_trigger;	

	unsigned long		 blink_delay_on, blink_delay_off;
	struct timer_list	 blink_timer;
	int			 blink_brightness;

	struct work_struct	set_brightness_work;
	int			delayed_set_value;

#ifdef CONFIG_LEDS_TRIGGERS
	
	struct rw_semaphore	 trigger_lock;

	struct led_trigger	*trigger;
	struct list_head	 trig_list;
	void			*trigger_data;
	
	bool			activated;
#endif

	/* Ensures consistent access to the LED Flash Class device */
	struct mutex		led_access;
};

extern int led_classdev_register(struct device *parent,
				 struct led_classdev *led_cdev);
extern void led_classdev_unregister(struct led_classdev *led_cdev);
extern void led_classdev_suspend(struct led_classdev *led_cdev);
extern void led_classdev_resume(struct led_classdev *led_cdev);

extern void led_blink_set(struct led_classdev *led_cdev,
			  unsigned long *delay_on,
			  unsigned long *delay_off);
extern void led_blink_set_oneshot(struct led_classdev *led_cdev,
				  unsigned long *delay_on,
				  unsigned long *delay_off,
				  int invert);
extern void led_set_brightness(struct led_classdev *led_cdev,
			       enum led_brightness brightness);

<<<<<<< HEAD
=======
/**
 * led_sysfs_disable - disable LED sysfs interface
 * @led_cdev: the LED to set
 *
 * Disable the led_cdev's sysfs interface.
 */
extern void led_sysfs_disable(struct led_classdev *led_cdev);

/**
 * led_sysfs_enable - enable LED sysfs interface
 * @led_cdev: the LED to set
 *
 * Enable the led_cdev's sysfs interface.
 */
extern void led_sysfs_enable(struct led_classdev *led_cdev);

/**
 * led_sysfs_is_disabled - check if LED sysfs interface is disabled
 * @led_cdev: the LED to query
 *
 * Returns: true if the led_cdev's sysfs interface is disabled.
 */
static inline bool led_sysfs_is_disabled(struct led_classdev *led_cdev)
{
	return led_cdev->flags & LED_SYSFS_DISABLE;
}

/*
 * LED Triggers
 */
/* Registration functions for simple triggers */
>>>>>>> 0e91d2a... Nougat
#define DEFINE_LED_TRIGGER(x)		static struct led_trigger *x;
#define DEFINE_LED_TRIGGER_GLOBAL(x)	struct led_trigger *x;

#ifdef CONFIG_LEDS_TRIGGERS

#define TRIG_NAME_MAX 50

struct led_trigger {
	
	const char	 *name;
	void		(*activate)(struct led_classdev *led_cdev);
	void		(*deactivate)(struct led_classdev *led_cdev);

	
	rwlock_t	  leddev_list_lock;
	struct list_head  led_cdevs;

	
	struct list_head  next_trig;
};

extern int led_trigger_register(struct led_trigger *trigger);
extern void led_trigger_unregister(struct led_trigger *trigger);

extern void led_trigger_register_simple(const char *name,
				struct led_trigger **trigger);
extern void led_trigger_unregister_simple(struct led_trigger *trigger);
extern void led_trigger_event(struct led_trigger *trigger,
				enum led_brightness event);
extern void led_trigger_blink(struct led_trigger *trigger,
			      unsigned long *delay_on,
			      unsigned long *delay_off);
extern void led_trigger_blink_oneshot(struct led_trigger *trigger,
				      unsigned long *delay_on,
				      unsigned long *delay_off,
				      int invert);
extern void led_trigger_rename_static(const char *name,
				      struct led_trigger *trig);

#else

struct led_trigger {};

static inline void led_trigger_register_simple(const char *name,
					struct led_trigger **trigger) {}
static inline void led_trigger_unregister_simple(struct led_trigger *trigger) {}
static inline void led_trigger_event(struct led_trigger *trigger,
				enum led_brightness event) {}
#endif 

#ifdef CONFIG_LEDS_TRIGGER_IDE_DISK
extern void ledtrig_ide_activity(void);
#else
static inline void ledtrig_ide_activity(void) {}
#endif

#if defined(CONFIG_LEDS_TRIGGER_CAMERA) || defined(CONFIG_LEDS_TRIGGER_CAMERA_MODULE)
extern void ledtrig_flash_ctrl(bool on);
extern void ledtrig_torch_ctrl(bool on);
#else
static inline void ledtrig_flash_ctrl(bool on) {}
static inline void ledtrig_torch_ctrl(bool on) {}
#endif

struct led_info {
	const char	*name;
	const char	*default_trigger;
	int		flags;
};

struct led_platform_data {
	int		num_leds;
	struct led_info	*leds;
};

struct gpio_led {
	const char *name;
	const char *default_trigger;
	unsigned 	gpio;
	unsigned	active_low : 1;
	unsigned	retain_state_suspended : 1;
	unsigned	default_state : 2;
	
};
#define LEDS_GPIO_DEFSTATE_OFF		0
#define LEDS_GPIO_DEFSTATE_ON		1
#define LEDS_GPIO_DEFSTATE_KEEP		2

struct gpio_led_platform_data {
	int 		num_leds;
	const struct gpio_led *leds;

#define GPIO_LED_NO_BLINK_LOW	0	
#define GPIO_LED_NO_BLINK_HIGH	1	
#define GPIO_LED_BLINK		2	
	int		(*gpio_blink_set)(unsigned gpio, int state,
					unsigned long *delay_on,
					unsigned long *delay_off);
};

struct platform_device *gpio_led_register_device(
		int id, const struct gpio_led_platform_data *pdata);

enum cpu_led_event {
	CPU_LED_IDLE_START,	
	CPU_LED_IDLE_END,	
	CPU_LED_START,		
	CPU_LED_STOP,		
	CPU_LED_HALTED,		
};
#ifdef CONFIG_LEDS_TRIGGER_CPU
extern void ledtrig_cpu(enum cpu_led_event evt);
#else
static inline void ledtrig_cpu(enum cpu_led_event evt)
{
	return;
}
#endif

#endif		
