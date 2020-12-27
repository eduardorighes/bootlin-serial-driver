// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <uapi/linux/serial_reg.h>
#include <linux/debugfs.h>

#define SERIAL_RESET_COUNTER  0
#define SERIAL_GET_TX_COUNTER 1
#define SERIAL_GET_RX_COUNTER 2

#define SERIAL_BUFSIZE 64

struct serial_dev {
	struct platform_device *pdev;
	struct miscdevice miscdev;
	void __iomem *regs;
	unsigned int tx_counter; /*TODO: this should be 64-bit */
	unsigned int rx_counter; /*TODO: this should be 64-bit */
	int irq;
	char buffer[SERIAL_BUFSIZE];
	int buf_rd;
	int buf_wr;
	wait_queue_head_t serial_wait;
	spinlock_t lock;
	struct dentry *debugfs_dir;
};

static struct serial_dev *file_to_serial(struct file *filp)
{
	return container_of(filp->private_data, struct serial_dev, miscdev);
}

static u32 reg_read(struct serial_dev *dev, off_t offset)
{
	return readl(dev->regs + (offset << 2));
}

static void reg_write(struct serial_dev *dev, u32 val, off_t offset)
{
	writel(val, dev->regs + (offset << 2));
}

static int serial_configure_baud_rate(
	struct platform_device *pdev, 
	struct serial_dev *dev)
{
	u32 uartclk, baud_divisor;
	int ret;

	/* configure baud rate to 115200 */

	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
		&uartclk);
	if (ret < 0) {
		dev_err(&pdev->dev, 
			"failed to get clock-frequency property (%d)\n", ret);
		return ret;
	}
	baud_divisor = uartclk / 16 / 115200;
	reg_write(dev, 0x07, UART_OMAP_MDR1);
	reg_write(dev, 0x00, UART_LCR);
	reg_write(dev, UART_LCR_DLAB, UART_LCR);
	reg_write(dev, baud_divisor & 0xff, UART_DLL);
	reg_write(dev, (baud_divisor >> 8) & 0xff, UART_DLM);
	reg_write(dev, UART_LCR_WLEN8, UART_LCR);

	return 0;
}

static void serial_write_char(struct serial_dev *dev, u8 val)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	while ((reg_read(dev, UART_LSR) & UART_LSR_THRE) == 0)
		cpu_relax();

	reg_write(dev, val, UART_TX);

	spin_unlock_irqrestore(&dev->lock, flags);
}

static ssize_t serial_read( struct file *filp, char __user *buf, 
	size_t bufsize, loff_t *ppos)
{
	int ret;
	struct serial_dev *dev = file_to_serial(filp);
	char ch;
	unsigned long flags;

retry:
	ret = wait_event_interruptible(dev->serial_wait, dev->buf_wr != dev->buf_rd);
	if (ret)
		return ret;

	spin_lock_irqsave(&dev->lock, flags);

	if (dev->buf_wr == dev->buf_rd) {
		/* if we get the lock and the buffer is empty, release lock */
		spin_unlock_irqrestore(&dev->lock, flags);
		goto retry;
	}

	ch = dev->buffer[dev->buf_rd];
	dev->buf_rd = (dev->buf_rd + 1) % SERIAL_BUFSIZE;
	++dev->rx_counter;
	
	spin_unlock_irqrestore(&dev->lock, flags);

	if (put_user(ch, buf))
		return -EFAULT;
	*ppos += 1;


	return 1;
}

static ssize_t serial_write( struct file *filp, const char __user *buf, 
	size_t bufsize, loff_t *ppos)
{
	struct serial_dev *dev = file_to_serial(filp);
	size_t i;
	u8 c;

	for (i = 0; i < bufsize; ++i) {
		if (get_user(c, buf + i))
			return -EFAULT;
		serial_write_char(dev, c);
		++dev->tx_counter;
		if (c == '\n') 
			serial_write_char(dev, '\r');
	}

	*ppos += bufsize;

	return bufsize;
}

static long serial_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct serial_dev *dev = file_to_serial(filp);
	unsigned int __user *argp = (unsigned int __user *) arg;

	switch (cmd) {
		case SERIAL_RESET_COUNTER:
			dev->tx_counter = 0;
			dev->rx_counter = 0;
			break;
		case SERIAL_GET_TX_COUNTER:
			if (put_user(dev->tx_counter, argp))
				return -EFAULT;
			break;
		case SERIAL_GET_RX_COUNTER:
			if (put_user(dev->rx_counter, argp))
				return -EFAULT;
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}

static irqreturn_t serial_interrupt(int irq, void *dev_id)
{
	struct serial_dev *dev = (struct serial_dev *) dev_id;

	spin_lock(&dev->lock);

	dev->buffer[dev->buf_wr] = reg_read(dev, UART_RX) & 0xff;
	dev->buf_wr = (dev->buf_wr + 1) % SERIAL_BUFSIZE;

	spin_unlock(&dev->lock);

	wake_up(&dev->serial_wait);

	return IRQ_HANDLED;
}

static const struct file_operations serial_fops = {
	.owner = THIS_MODULE,
	.read = serial_read,
	.write = serial_write,
	.unlocked_ioctl = serial_ioctl,
};

static int serial_probe(struct platform_device *pdev)
{
	struct serial_dev *dev;
	struct resource *res;
	int ret;

	dev_info(&pdev->dev, "probe start\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get resource\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "got address 0x%x\n", res->start);

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	
	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);

	init_waitqueue_head(&dev->serial_wait);
	spin_lock_init(&dev->lock);

	dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!dev->regs) {
		dev_err(&pdev->dev, "cannot remap registers\n");
		return -ENOMEM;
	}


	/* enable device */

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* baud rate */
	ret = serial_configure_baud_rate(pdev, dev);
	if (ret < 0) {
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	/* soft reset */

	reg_write(dev, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);
	reg_write(dev, 0x00, UART_OMAP_MDR1);

	/* interrupt handler */

	dev->irq = platform_get_irq(pdev, 0);
	ret =devm_request_irq(&pdev->dev, dev->irq, serial_interrupt, 0, 
		              "serial", dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register interrupt handler (%d)\n", ret);
		return ret;
	}

	reg_write(dev, UART_IER_RDI, UART_IER);

	/* register with misc subsystem */

	dev->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, 
		"serial-%x", res->start);
	dev->miscdev.fops = &serial_fops;
	dev->miscdev.parent = &pdev->dev;
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;

	misc_register(&dev->miscdev);

	/* debugfs */

	dev->debugfs_dir = debugfs_create_dir(dev->miscdev.name, NULL);
	debugfs_create_u32("tx_counter", S_IRUGO, dev->debugfs_dir, &dev->tx_counter);
	debugfs_create_u32("rx_counter", S_IRUGO, dev->debugfs_dir, &dev->rx_counter);

	dev_info(&pdev->dev, "probe complete\n");

	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *dev = platform_get_drvdata(pdev);

	debugfs_remove(dev->debugfs_dir);
	misc_deregister(&dev->miscdev);
	pm_runtime_disable(&pdev->dev);

	dev_info(&pdev->dev, "remove complete\n");
        
	return 0;
}

static const struct of_device_id serial_of_match[] = {
	{ .compatible = "bootlin,serial" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, serial_of_match);

static struct platform_driver serial_driver = {
        .driver = {
                .name = "bootlin-serial",
                .owner = THIS_MODULE,
		.of_match_table = serial_of_match,
        },
        .probe = serial_probe,
        .remove = serial_remove,
};

module_platform_driver(serial_driver);
MODULE_LICENSE("GPL");
