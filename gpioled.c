#include <linux/types.h>      // 基本数据类型定义（如 u8, u16, u32, u64 等）
#include <linux/kernel.h>     // 内核核心功能（printk, container_of 等宏）
#include <linux/delay.h>      // 延时函数（mdelay, udelay, msleep 等）
#include <linux/ide.h>        // IDE/ATA 硬盘驱动接口
#include <linux/init.h>       // 模块初始化和清理宏（__init, __exit 等）
#include <linux/module.h>     // 内核模块支持（MODULE_LICENSE, module_init 等）
#include <linux/errno.h>      // 错误码定义（EINVAL, ENOMEM 等）
#include <linux/gpio.h>       // GPIO 引脚控制接口
#include <linux/cdev.h>       // 字符设备驱动框架
#include <linux/device.h>     // 设备模型和设备类管理
#include <linux/of.h>         // 设备树（Device Tree）解析
#include <linux/of_address.h> // 设备树地址解析
#include <linux/of_gpio.h>    // 设备树GPIO解析
#include <asm/uaccess.h>      // 用户空间访问函数（copy_to_user, copy_from_user）
#include <asm/io.h>           // 硬件I/O访问函数（ioremap, readl, writel 等）

#define gpio_led_CNT     1       /* 设备号个数 */
#define gpio_led_NAME    "gpio_led"
#define LEDOFF          0
#define LEDON           1

/* gpio_led 设备结构体 */
struct gpio_led_dev {
    dev_t devid;                /* 设备号 */
    struct cdev cdev;           /* cdev */
    struct class *class;        /* 类 */
    struct device *device;      /* 设备 */
    int major;                  /* 主设备号 */
    int minor;                  /* 次设备号 */
    struct device_node *nd;     /* 设备节点 */
    int led_gpio;               /* led 所使用的 GPIO 编号 */
};

struct gpio_led_dev gpio_led;

/* 打开设备 */
static int led_open(struct inode *inode, struct file *filp) {
    filp->private_data = &gpio_led; // 将设备结构体指针保存到文件操作结构体中
    return 0;
}

/* 读取设备数据 */
static ssize_t led_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt) {
    return 0; 
}

/* 写设备数据 */
static ssize_t led_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt) {
    int retvalue;
    unsigned char databuf[1];
    unsigned char ledstat;
    struct gpio_led_dev *dev = filp->private_data;

    /* 从用户空间复制数据 */
    retvalue = copy_from_user(databuf, buf, cnt);
    if (retvalue < 0) {
        printk("kernel write failed!\r\n");
        return -EFAULT; // 复制失败返回错误
    }

    ledstat = databuf[0];

    /* 根据数据控制LED开关 */
    if (ledstat == LEDON) {
        gpio_set_value(dev->led_gpio, 1); // 点亮LED
    } else if (ledstat == LEDOFF) {
        gpio_set_value(dev->led_gpio, 0); // 熄灭LED
    }

    return 0;
}

/* 释放设备 */
static int led_release(struct inode *inode, struct file *filp) {
    return 0;
}

/* 文件操作结构体 */
static struct file_operations gpio_led_fops = {
    .owner = THIS_MODULE,
    .open = led_open,
    .read = led_read,
    .write = led_write,
    .release = led_release, 
};

/* 初始化函数 */
static int __init led_init(void) {
    int ret = 0;
    const char *str;

    /* 查找设备树中的 gpio_led 节点 */
    gpio_led.nd = of_find_node_by_path("/gpio_led");
    if (gpio_led.nd == NULL) {
        printk("gpio_led node not found!\r\n");
        return -EINVAL;
    }

    /* 读取设备树中的状态属性 */
    ret = of_property_read_string(gpio_led.nd, "status", &str);
    if (ret < 0) {
        return -EINVAL;
    }

    /* 检查设备树中的状态是否为“okay” */
    if (strcmp(str, "okay")) {
        return -EINVAL;
    }

    /* 读取兼容性属性 */
    ret = of_property_read_string(gpio_led.nd, "compatible", &str);
    if (ret < 0) {
        printk("gpio_led: Failed to get compatible property\r\n");
        return -EINVAL;
    }

    if (strcmp(str, "kehan,led")) {
        printk("gpio_led: Compatible match failed\r\n");
        return -EINVAL;
    }

    /* 获取LED的GPIO编号 */
    gpio_led.led_gpio = of_get_named_gpio(gpio_led.nd, "led-gpio", 0);
    if (gpio_led.led_gpio < 0) {
        printk("Cannot get led-gpio");
        return -EINVAL;
    }
    printk("led-gpio num = %d\r\n", gpio_led.led_gpio);

    /* 请求GPIO资源 */
    ret = gpio_request(gpio_led.led_gpio, "LED-GPIO");
    if (ret) {
        printk(KERN_ERR "gpio_led: Failed to request led-gpio\n");
        return ret;
    }

    /* 设置GPIO为输出模式，初始值为0 */
    ret = gpio_direction_output(gpio_led.led_gpio, 0);
    if (ret < 0) {
        printk("Cannot set gpio\r\n");
    }

    /* 注册字符设备 */
    if (gpio_led.major) {
        gpio_led.devid = MKDEV(gpio_led.major, 0);
        ret = register_chrdev_region(gpio_led.devid, gpio_led_CNT, gpio_led_NAME);
        if (ret < 0) {
            pr_err("Cannot register %s char driver [ret=%d]\n", gpio_led_NAME, gpio_led_CNT);
            goto free_gpio;
        }
    } else {
        ret = alloc_chrdev_region(&gpio_led.devid, 0, gpio_led_CNT, gpio_led_NAME);
        if (ret < 0) {
            pr_err("%s Couldnot alloc_chrdev_region, ret=%d\r\n", gpio_led_NAME, ret);
            goto free_gpio;
        }
        gpio_led.major = MAJOR(gpio_led.devid);
        gpio_led.minor = MINOR(gpio_led.devid);
    }
    printk("gpio_led major=%d, minor=%d\r\n", gpio_led.major, gpio_led.minor);

    gpio_led.cdev.owner = THIS_MODULE;
    cdev_init(&gpio_led.cdev, &gpio_led_fops); // 需要传入文件操作结构体

    /* 添加字符设备 */
    ret = cdev_add(&gpio_led.cdev, gpio_led.devid, gpio_led_CNT);
    if (ret < 0) {
        goto del_unregister;
    }

    /* 创建类和设备 */
    gpio_led.class = class_create(THIS_MODULE, gpio_led_NAME);
    if (IS_ERR(gpio_led.class)) {
        goto del_cdev;
    }

    gpio_led.device = device_create(gpio_led.class, NULL, gpio_led.devid, NULL, gpio_led_NAME);
    if (IS_ERR(gpio_led.device)) {
        goto destroy_class;
    }

    return 0;

destroy_class:
    class_destroy(gpio_led.class);
del_cdev:
    cdev_del(&gpio_led.cdev);
del_unregister:
    unregister_chrdev_region(gpio_led.devid, gpio_led_CNT);
free_gpio:
    gpio_free(gpio_led.led_gpio);
    return -EIO;
}

/* 清理函数 */
static void __exit led_exit(void) {
    cdev_del(&gpio_led.cdev);
    unregister_chrdev_region(gpio_led.devid, gpio_led_CNT);
    device_destroy(gpio_led.class, gpio_led.devid);
    class_destroy(gpio_led.class);
    gpio_free(gpio_led.led_gpio);
}

module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("KEHAN");
MODULE_INFO(intree, "Y");
