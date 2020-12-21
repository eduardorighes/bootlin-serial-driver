// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <uapi/linux/serial_reg.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

struct serial_dev {
	struct miscdevice miscdev;
	void __iomem *regs;
};

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
	while ((reg_read(dev, UART_LSR) & UART_LSR_THRE) == 0)
		cpu_relax();

	reg_write(dev, val, UART_TX);
}

static ssize_t serial_read(
	struct file *filp, 
	char __user *buf, 
	size_t bufsize, 
	loff_t *ppos)
{
	return -EINVAL;
}

static ssize_t serial_write(
	struct file *filp, 
	const char __user *buf, 
	size_t bufsize, 
	loff_t *ppos)
{
	return -EINVAL;
}

static const struct file_operations serial_fops = {
	.read = serial_read,
	.write = serial_write,
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
	
	platform_set_drvdata(pdev, dev);

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


	/* register with misc subsystem */

	dev->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, 
		"serial-%x", res->start);
	dev->miscdev.fops = &serial_fops;
	dev->miscdev.parent = &pdev->dev;
	dev->miscdev.minor = MISC_DYNAMIC_MINOR;

	misc_register(&dev->miscdev);

	dev_info(&pdev->dev, "probe complete\n");

	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *dev = platform_get_drvdata(pdev);

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
