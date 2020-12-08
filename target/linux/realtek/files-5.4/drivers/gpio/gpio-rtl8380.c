// SPDX-License-Identifier: GPL-2.0-only

#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/swab.h>

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

/* Total register block size is 0x1C for ports A, B, C, and D.
 * same layout repeats at GPIO_CTRL_REG_BASE + 0x1C for ports E, F, G, (and H).
 */
#define RTL8380_GPIO_PORTA_REG_BASE 0
#define RTL8380_GPIO_PORTE_REG_BASE 0x1C

// Pin mux? 0: "normal", 1: "dedicate peripheral"
#define RTL8380_GPIO_REG_CNR		0x00
// Set bit to 0 for input, 1 for output (output enable)
#define RTL8380_GPIO_REG_DIR		0x08
#define RTL8380_GPIO_REG_DATA		0x0C
// Read bit for IRQ status, write 1 to clear IRQ
#define RTL8380_GPIO_REG_ISR		0x10
// Two bits per pin, 0: disable, 1: falling, 2: rising, 3: both
// use swahw32 to manipulate these registers
#define RTL8380_GPIO_REG_IMR_LOW	0x14
#define RTL8380_GPIO_REG_IMR_HI		0x18

#define RTL8380_GPIO_IRQ_EDGE_FALLING	1
#define RTL8380_GPIO_IRQ_EDGE_RISING	2

struct rtl8380_gpio_ctrl {
	struct gpio_chip gc;
	void __iomem *base;
	raw_spinlock_t lock;
	// TODO gpio irq
};

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

	if (of_property_read_u32(np, "ngpios", &ngpios) != 0)
		ngpios = 32;

	if (ngpios > 32) {
		dev_err(&pdev->dev, "ngpios must be smaller than or equal to 32\n");
		return -EINVAL;
	}

	ctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->base))
		return PTR_ERR(ctrl->base);

	raw_spin_lock_init(&ctrl->lock);
	bgpio_init(&ctrl->gc, dev, 4,
		ctrl->base + RTL8380_GPIO_REG_DATA, NULL, NULL,
		ctrl->base + RTL8380_GPIO_REG_DIR, NULL,
		BGPIOF_BIG_ENDIAN_BYTE_ORDER);

	ctrl->gc.ngpio = ngpios;

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
