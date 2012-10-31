#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/switch.h>
#include <linux/freezer.h>
#include <linux/utsname.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>


#define DRIVER_NAME             "usb_mass_storage"

static const char shortname[] = DRIVER_NAME;
struct dummy_usb_device {
	int config;
	struct platform_device *pdev;
	struct class *cdev;
	struct device   *dev;
	struct switch_dev sdev;
};
static struct dummy_usb_device dummy_dev;

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	printk("enter:print_switch_name:%s\n", buf);
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	printk("enter:print_switch_state:%s\n", buf);
	struct dummy_usb_device *devp = container_of(sdev, struct dummy_usb_device, sdev);
	return sprintf(buf, "%s\n", (devp->config ? "online" : "offline"));
}

#define ONLINE	1
#define OFFLINE	0
#define SCONFIG(conf)	(conf) ? "online" : "offline"

static int do_set_config(struct dummy_usb_device *usbdev, u8 new_config)
{
        int     rc = 0;

	printk("now config=%s new config=%s\n", SCONFIG(usbdev->config), SCONFIG(new_config));
        if (new_config == usbdev->config) {
		return rc;
	}
	/* Disable the single interface */
	if (usbdev->config) {
		printk("reset config\n");
		usbdev->config = OFFLINE;
	}

        /* Enable the interface */
        if (new_config) {
                usbdev->config = ONLINE;
	}

        switch_set_state(&usbdev->sdev, new_config);
        return rc;
}

static ssize_t show_file(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	printk("dummy_device:show_file:dev=%p attr=%p buf=%p\n", dev, attr, buf);
	//sprintf(buf, "test data by tmori");
	memcpy(buf, SCONFIG(dummy_dev.config), strlen(SCONFIG(dummy_dev.config)));
	printk("show:bufp=%s\n", buf);
	return (strlen(buf) + 1);
}
static ssize_t store_file(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u8 new_config;
	printk("store:bufp=%s\n", buf);
	if (!strncmp(buf, "online", strlen("online"))) {
		new_config = ONLINE;
	} else {
		new_config = OFFLINE;
	}
	do_set_config(&dummy_dev, new_config);
	return count;
}

static DEVICE_ATTR(file, 0777, show_file, store_file);

static int __init init(void)
{
	int rc;

	dummy_dev.config = OFFLINE;
	dummy_dev.sdev.name = DRIVER_NAME;
	dummy_dev.sdev.print_name = print_switch_name;
	dummy_dev.sdev.print_state = print_switch_state;
	dummy_dev.cdev = NULL;
	dummy_dev.dev = NULL;
	dummy_dev.cdev = class_create(THIS_MODULE, "dummy_device");

	if (!IS_ERR(dummy_dev.cdev)) {
		dummy_dev.dev = device_create(dummy_dev.cdev,
					NULL, 0, NULL, "%s",
					"usb_gadget");
		if (dummy_dev.dev) {
			rc = device_create_file(dummy_dev.dev, &dev_attr_file);
			printk("dummy_usb_gadget:device_create_file error. rc=%d\n", rc);
		} else {
			printk("dummy_usb_gadget:device_create error.\n");
		}
	} else {
		printk("dummy_usb_gadget:class_create error.\n");
	}
	printk("dummy_usb_gadget:cdev=%p dev=%p\n", dummy_dev.cdev, dummy_dev.dev);
	rc = switch_dev_register(&dummy_dev.sdev);
	if (rc != 0) {
		printk("dummy_usb_gadget:switch_dev_register:err=%d\n", rc);
	} else {
	}
	return rc;
}

//module_init(init);

