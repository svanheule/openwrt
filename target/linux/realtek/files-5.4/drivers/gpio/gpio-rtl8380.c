// SPDX-License-Identifier: GPL-2.0-only

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/swab.h>
#include <linux/irq.h>
#include <asm/mach-rtl838x/mach-rtl83xx.h>

//extern struct rtl83xx_soc_info soc_info;

/* SDK definitions
 *  GPIO_BASE	0xB8003500
 *  P_ABCD_CNR	(GPIO_BASE + 0x00)
 *  P_ABCD_DIR	(GPIO_BASE + 0x08)
 *  P_ABCD_DAT	(GPIO_BASE + 0x0C)
 *  P_ABCD_ISR	(GPIO_BASE + 0x10)
 *  P_AB_IMR	(GPIO_BASE + 0x14)
 *  P_CD_IMR	(GPIO_BASE + 0x18)
 * 
 *  P_EFG_CNR	(GPIO_BASE + 0x1C)
 *  P_EFG_DIR	(GPIO_BASE + 0x24)
 *  P_EFG_DAT	(GPIO_BASE + 0x28)
 *  P_EFG_ISR	(GPIO_BASE + 0x2C)
 *  P_EF_IMR	(GPIO_BASE + 0x30)
 *  P_G_IMR	(GPIO_BASE + 0x34)
 */

// Pin mux? 0: "normal", 1: "dedicate peripheral"
#define RTL8380_PINMUX_REG_CNR		0x00
#define RTL8380_PINMUX_REG_PTYPE	0x04
// Set bit to 0 for input, 1 for output (output enable)
#define RTL8380_GPIO_REG_DIR		0x00
#define RTL8380_GPIO_REG_DATA		0x04
// Read bit for IRQ status, write 1 to clear IRQ
#define RTL8380_GPIO_REG_ISR		0x08
// Two bits per pin, 0: disable, 1: falling, 2: rising, 3: both
// use swahw32 to manipulate these registers
#define RTL8380_GPIO_REG_IMR_LOW	0x0C
#define RTL8380_GPIO_REG_IMR_HI		0x10

#define RTL8380_GPIO_IRQ_EDGE_FALLING	1
#define RTL8380_GPIO_IRQ_EDGE_RISING	2

struct rtl8380_gpio_ctrl {
	struct gpio_chip gc;
	void __iomem *base;
	raw_spinlock_t lock;
	// TODO gpio irq
};

static u32 rtl8380_gpio_read(struct rtl8380_gpio_ctrl *ctrl, unsigned int reg)
{
	return swab32(ioread32(ctrl->base + reg));
}

static void rtl8380_gpio_write(struct rtl8380_gpio_ctrl *ctrl,
			unsigned int reg, u32 val)
{
	return iowrite32(swab32(val), ctrl->base + reg);
}

static void rtl8380_gpio_update_bits(struct rtl8380_gpio_ctrl *ctrl,
			unsigned int reg, u32 mask, u32 bits)
{
	unsigned long flags;
	u32 v;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	v = rtl8380_gpio_read(ctrl, reg);
	rtl8380_gpio_write(ctrl, reg, (v & ~mask) | (bits & mask));
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

static void rtl8380_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct rtl8380_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	u32 mask = BIT(offset);

	rtl8380_gpio_update_bits(ctrl, RTL8380_GPIO_REG_DATA, mask, value << offset);
}

static int rtl8380_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8380_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	return !!(rtl8380_gpio_read(ctrl, RTL8380_GPIO_REG_DATA) & BIT(offset));
}

static int rtl8380_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8380_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	rtl8380_gpio_update_bits(ctrl, RTL8380_GPIO_REG_DIR, BIT(offset), 0);

	return 0;
}

static int rtl8380_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	unsigned long flags;
	struct rtl8380_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	u32 mask = BIT(offset);

	rtl8380_gpio_update_bits(ctrl, RTL8380_GPIO_REG_DIR, mask, mask);
	rtl8380_gpio_set(gc, offset, value);

	return 0;
}

static int rtl8380_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8380_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	return !(rtl8380_gpio_read(ctrl, RTL8380_GPIO_REG_DIR) & BIT(offset));
}

static int rtl8380_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rtl8380_gpio_ctrl *ctrl;
	u32 ngpios;
	int err;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	platform_set_drvdata(pdev, ctrl);

	if (!np) {
		dev_err(&pdev->dev, "no DT node found\n");
		return -EINVAL;
	}

	err = of_property_read_u32(np, "ngpios", &ngpios);
	if (err) {
		dev_err(&pdev->dev, "invalid ngpios property\n");
		return err;
	}

	if (ngpios > 32) {
		dev_err(&pdev->dev, "ngpios must be smaller than or equal to 32\n");
		return -EINVAL;
	}

	ctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->base))
		return PTR_ERR(ctrl->base);

	raw_spin_lock_init(&ctrl->lock);
	// Let the GPIO subsystem assign the base number
	ctrl->gc.base = -1;
	ctrl->gc.ngpio = ngpios;
	ctrl->gc.label = "rtl8380-gpio";
	ctrl->gc.parent = dev;
	ctrl->gc.owner = THIS_MODULE;
	ctrl->gc.can_sleep = false;

	ctrl->gc.set = rtl8380_gpio_set;
	ctrl->gc.get = rtl8380_gpio_get;
	ctrl->gc.direction_input = rtl8380_direction_input;
	ctrl->gc.direction_output = rtl8380_direction_output;
	ctrl->gc.get_direction = rtl8380_get_direction;

	err = gpiochip_add_data(&ctrl->gc, ctrl);
	return err;
}

static const struct of_device_id rtl8380_gpio_of_match[] = {
	{ .compatible = "realtek,rtl8380-gpio" },
	{},
};

MODULE_DEVICE_TABLE(of, rtl8380_gpio_of_match);

static struct platform_driver rtl8380_gpio_driver = {
	.driver = {
		.name = "rtl8380-gpio",
		.of_match_table	= rtl8380_gpio_of_match,
	},
	.probe = rtl8380_gpio_probe,
};

module_platform_driver(rtl8380_gpio_driver);

MODULE_DESCRIPTION("Realtek RTL8380 GPIO support");
MODULE_LICENSE("GPL v2");
