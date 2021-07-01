/**
 * @file    hw_serial.c
 * @author  Sonu Verma
 * @date    21 April 2021
 * @version 0.1
 * @brief  An introductory UART driver for AM35xx Cortex controller used i Beaglebone
*/

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/serial_reg.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/irqreturn.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#define BUFF_SIZE 512
//#define UART_LSR_THRE		0x20 /* Transmit-hold-register empty */
//#define UART_LSR_DR 		UART LSR value, contains the bit which define, Data Ready

//Circular buffer struct
static struct circ_buff
{
    char buff[BUFF_SIZE];
    int read_pos;
    int write_pos;
    int length;
};
typedef struct circ_buff circular_type;

//Serial device struct
static struct hw_serial_dev
{
    void __iomem *regs;
    struct miscdevice mDev;
    int irq;
    circular_type buf;
    wait_queue_head_t waitQ;
    unsigned long irqFlags;
    spinlock_t lock;
};
typedef struct hw_serial_dev serial_dev_type;

//Driver probe routine
static int dev_probe_init(struct platform_device *pdev);

//Driver remove routine
static int dev_remove_exit(struct platform_device *pdev);

//FOPS open
static int dev_open(struct inode *inode, struct file *file);

//FOPS close
static int dev_close(struct inode *inodep, struct file *filp);

//FOPS read
static ssize_t dev_read(struct file *file, char __user *buf, size_t size, loff_t *ppos);

//FOPS write
static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos);

//Routine to read from serial device registers
static unsigned int dev_read_reg(serial_dev_type *dev, int offset);

//Routine to write to serial device registers
static void dev_write_reg(serial_dev_type *dev, int val, int offset);

//Routine to write a character to the seriald device
static void write_char(serial_dev_type *dev, char test);

//Interrupt handler
static irqreturn_t irqRxHandler(int irq, void *devid);

//Utility method to write to circular buffer
static void write_circ_buff(char c, serial_dev_type *dev);

//Utility method to read from circular buff
static char read_circ_buff(serial_dev_type *dev);

//Device id struct
static struct of_device_id hw_match_table[] =
 {
        {
            .compatible = "serial",
        },
};

//File operations struct
static const struct file_operations hw_fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
    .write = dev_write,
    .open = dev_open,
    .release = dev_close,
    .llseek = no_llseek,
};

//Platform driver structure
static struct platform_driver hw_plat_driver = {
    .driver = {
        .name = "serial",
        .owner = THIS_MODULE,
        .of_match_table = hw_match_table},
    .probe = dev_probe_init,
    .remove = dev_remove_exit
};

/*********************************************************/
static int dev_open(struct inode *inode, struct file *file)
{
    return 0;
}
/*********************************************************/
static int dev_close(struct inode *inodep, struct file *filp)
{
    return 0;
}
/*********************************************************/
static ssize_t dev_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    struct miscdevice *mdev = (struct miscdevice *)file->private_data;
    serial_dev_type *dev = container_of(mdev, serial_dev_type, mDev);
    wait_event_interruptible(dev->waitQ, dev->buf.length > 0);

    char ret = read_circ_buff(dev);
    copy_to_user(buf, &ret, 1);
    return 1;
}
/*********************************************************/
static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos)
{
    struct miscdevice *mdev = (struct miscdevice *)file->private_data;
    serial_dev_type *dev = container_of(mdev, serial_dev_type, mDev);

    char kmem[len + 1];
    copy_from_user(kmem, buf, len);
    int i;
    for (i = 0; i < len; i++)
    {
        if (kmem[i] == '\n')
        {
            write_char(dev, '\n');
            write_char(dev, '\r');
        }
        else
        {
            write_char(dev, kmem[i]);
        }
    }   
    return len;
}

/*********************************************************/
static unsigned int dev_read_reg(serial_dev_type *dev, int offset)
{
    spin_lock_irqsave(&dev->lock, dev->irqFlags);
    unsigned int ret = ioread32(dev->regs + (4 * offset));
    spin_unlock_irqrestore(&dev->lock, dev->irqFlags);
    return ret;
}

/*********************************************************/
static void dev_write_reg(serial_dev_type *dev, int val, int offset)
{
    spin_lock_irqsave(&dev->lock, dev->irqFlags);
    iowrite32(val, dev->regs + (4 * offset));
    spin_unlock_irqrestore(&dev->lock, dev->irqFlags);
    return;
}

/*********************************************************/
static void write_char(serial_dev_type *dev, char c)
{
    unsigned int lsr = dev_read_reg(dev, UART_LSR);
    while (1)
    {
        if (lsr & UART_LSR_THRE)
        {
            break;
        }
        lsr = dev_read_reg(dev, UART_LSR);
    }
    dev_write_reg(dev, c, UART_TX);
}

/*********************************************************/
static irqreturn_t irqRxHandler(int irq, void *d)
{
    serial_dev_type *dev = d;
    do 
    {
        char recv = dev_read_reg(dev, UART_RX);
        write_circ_buff(recv, dev);
        wake_up(&dev->waitQ);
    }
    while (dev_read_reg(dev, UART_LSR) & UART_LSR_DR);
    return IRQ_HANDLED;
}

/*********************************************************/
static void write_circ_buff(char c, serial_dev_type *dev)
{
    spin_lock_irqsave(&dev->lock, dev->irqFlags);
    if(dev->buf.length < BUFF_SIZE)
    {
        dev->buf.buff[dev->buf.write_pos] = c;
        dev->buf.write_pos = ((dev->buf.write_pos + 1) % BUFF_SIZE);
        dev->buf.length++;
    }
    spin_unlock_irqrestore(&dev->lock, dev->irqFlags);
}

/*********************************************************/
static char read_circ_buff(serial_dev_type *dev)
{
    spin_lock_irqsave(&dev->lock, dev->irqFlags);
    char c = dev->buf.buff[dev->buf.read_pos];
    dev->buf.buff[dev-> buf.read_pos] = '\0';
    if(dev->buf.length > 0)
    {
        dev->buf.buff[dev->buf.read_pos] = '\0';
        dev->buf.read_pos = ((dev->buf.read_pos + 1 ) % BUFF_SIZE);
        dev->buf.length--;
    }
    spin_unlock_irqrestore(&dev->lock, dev->irqFlags);
    return c;
}


/*********************************************************/
static int dev_probe_init(struct platform_device *pdev)
{
    struct resource *res;
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    if (!res)
    {
        pr_err("%s: platform_get_resource returned NULL\n", __func__);
        return -EINVAL;
    }

    serial_dev_type *dev = devm_kzalloc(&pdev->dev, sizeof(serial_dev_type), GFP_KERNEL);
    if (!dev)
    {
        pr_err("%s: devm_kzalloc returned NULL\n", __func__);
        return -ENOMEM;
    }
    dev->regs = devm_ioremap_resource(&pdev->dev, res);

    if (IS_ERR(dev->regs))
    {
        dev_err(&pdev->dev, "%s: Can not remap registers\n", __func__);
        return PTR_ERR(dev->regs);
    }
    
    //Configure interrupts
    dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "%s: unable to get IRQ\n", __func__);
		return dev->irq;
	}
	int ret = devm_request_irq(&pdev->dev, dev->irq, irqRxHandler, 0, "hw_serial", dev);
	if (ret < 0) 
    {
		dev_err(&pdev->dev, "%s: unable to request IRQ %d (%d)\n", __func__, dev->irq, ret);
		return ret;
	}
    dev->buf.read_pos = 0;
    dev->buf.write_pos = 0;
    dev->buf.buff[0] = '\0';
    dev->buf.length = 0;
    init_waitqueue_head(&dev->waitQ);

    //Enable power management runtime
    pm_runtime_enable(&pdev->dev);
    pm_runtime_get_sync(&pdev->dev);

    //Configure the UART device
    unsigned int baud_divisor;
    unsigned int uartclk;

    of_property_read_u32(pdev->dev.of_node, "clock-frequency", &uartclk);

    baud_divisor = uartclk / 16 / 115200;

    dev_write_reg(dev, UART_OMAP_MDR1_DISABLE, UART_OMAP_MDR1);
    dev_write_reg(dev, 0x00, UART_LCR);
    dev_write_reg(dev, UART_LCR_DLAB, UART_LCR);
    dev_write_reg(dev, baud_divisor & 0xff, UART_DLL);
    dev_write_reg(dev, (baud_divisor >> 8) & 0xff, UART_DLM);
    dev_write_reg(dev, UART_LCR_WLEN8, UART_LCR);
    dev_write_reg(dev, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT | UART_FCR_ENABLE_FIFO, UART_FCR);

    dev_write_reg(dev, UART_OMAP_MDR1_16X_MODE, UART_OMAP_MDR1);

    //Initialize and register a misc device
    dev->mDev.minor = MISC_DYNAMIC_MINOR;
    dev->mDev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "hw_serial-%x", res->start);
    dev->mDev.fops = &hw_fops;

    int error = misc_register(&dev->mDev);
    if (error)
    {
        pr_err("%s: misc register failed.", __func__);
        return error;
    }

    dev_set_drvdata(&pdev->dev, dev);

    //Enable RX interrupt
    dev_write_reg(dev, UART_IER_RDI, UART_IER);

    return 0;
}

/*********************************************************/
static int dev_remove_exit(struct platform_device *pdev)
{
    pm_runtime_disable(&pdev->dev);
    serial_dev_type *dev = dev_get_drvdata(&pdev->dev);
    misc_deregister(&dev->mDev);
    return 0;
}

MODULE_AUTHOR("Sonu Verma");
MODULE_LICENSE("GPL");

//Register the platform driver
module_platform_driver(hw_plat_driver);
