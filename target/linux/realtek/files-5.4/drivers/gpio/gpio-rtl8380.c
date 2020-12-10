// SPDX-License-Identifier: GPL-2.0-only

#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_irq.h>
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
#define RTL8380_GPIO_REG_IMR(pin)	(0x14 + sizeof(u32)*((pin)/16))

#define RTL8380_GPIO_IRQ_EDGE_FALLING	1
#define RTL8380_GPIO_IRQ_EDGE_RISING	2

struct rtl8380_gpio_ctrl {
	struct device *dev;
	struct gpio_chip gc;
	void __iomem *base;
	raw_spinlock_t lock;
	/* keep split type/mask values to calculate
	 * the effective value for the IMR registers
	 */
	u32 intr_mask[2];
	u32 intr_type[2];
};

static struct rtl8380_gpio_ctrl *irq_data_to_ctrl(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);

	return container_of(gc, struct rtl8380_gpio_ctrl, gc);
}

static inline void rtl8380_gpio_isr_clear(struct rtl8380_gpio_ctrl *ctrl,
	unsigned int pin)
{
	iowrite32(swab32(BIT(pin)), ctrl->base + RTL8380_GPIO_REG_ISR);
}

static void rtl8380_gpio_irq_ack(struct irq_data *data)
{
	struct rtl8380_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	u32 pin = irqd_to_hwirq(data);

	rtl8380_gpio_isr_clear(ctrl, pin);
}

static inline u32 rtl8380_gpio_imr_bits(unsigned int pin, u32 value)
{
	return (value << 2*(pin % 16));
}

static inline void rtl8380_update_imr(struct rtl8380_gpio_ctrl *ctrl,
	unsigned int pin, u32 type, u32 mask)
{
	iowrite32(swahw32(type & mask),
		ctrl->base + RTL8380_GPIO_REG_IMR(pin));
}

static void rtl8380_gpio_irq_unmask(struct irq_data *data)
{
	struct rtl8380_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int offset = pin/16;
	unsigned long flags;
	u32 m;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	m = ctrl->intr_mask[offset];
	m |= rtl8380_gpio_imr_bits(pin, 3);
	ctrl->intr_mask[offset] = m;
	rtl8380_update_imr(ctrl, pin, ctrl->intr_type[offset], m);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

static void rtl8380_gpio_irq_mask(struct irq_data *data)
{
	struct rtl8380_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int offset = pin/16;
	unsigned long flags;
	u32 m;

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	m = ctrl->intr_mask[offset];
	m &= ~rtl8380_gpio_imr_bits(pin, 3);
	ctrl->intr_mask[offset] = m;
	rtl8380_update_imr(ctrl, pin, ctrl->intr_type[offset], m);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);
}

static int rtl8380_gpio_irq_set_type(struct irq_data *data,
	unsigned int flow_type)
{
	struct rtl8380_gpio_ctrl *ctrl = irq_data_to_ctrl(data);
	irq_flow_handler_t handler;
	unsigned int pin = irqd_to_hwirq(data);
	unsigned int offset = pin/16;
	unsigned long flags;
	u32 type, t;

	switch (flow_type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		type = 0;
		handler = handle_bad_irq;
		break;
	case IRQ_TYPE_EDGE_RISING:
		type = RTL8380_GPIO_IRQ_EDGE_RISING;
		handler = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		type = RTL8380_GPIO_IRQ_EDGE_RISING;
		fallthrough;
	case IRQ_TYPE_EDGE_FALLING:
		type |= RTL8380_GPIO_IRQ_EDGE_FALLING;
		handler = handle_edge_irq;
		break;
	default:
		return -EINVAL;
	}

	irq_set_handler_locked(data, handler);

	raw_spin_lock_irqsave(&ctrl->lock, flags);
	t = ctrl->intr_type[offset];
	t &= ~rtl8380_gpio_imr_bits(pin, 3);
	t |= rtl8380_gpio_imr_bits(pin, type);
	ctrl->intr_type[offset] = t;
	rtl8380_update_imr(ctrl, pin, t, ctrl->intr_mask[offset]);
	raw_spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static void rtl8380_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct rtl8380_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	int offset;
	unsigned long status;

	chained_irq_enter(irq_chip, desc);

	status = swab32(ioread32(ctrl->base + RTL8380_GPIO_REG_ISR));
	if (status)
		for_each_set_bit(offset, &status, gc->ngpio) {
			dev_dbg(ctrl->dev, "gpio irq %d\n", offset);
			generic_handle_irq(irq_find_mapping(gc->irq.domain,
							offset));
			rtl8380_gpio_isr_clear(ctrl, offset);
		}

	chained_irq_exit(irq_chip, desc);
}

static struct irq_chip rtl8380_gpio_irq_chip = {
	.name = "rtl8380-gpio",
	.irq_ack = rtl8380_gpio_irq_ack,
	.irq_mask = rtl8380_gpio_irq_mask,
	.irq_unmask = rtl8380_gpio_irq_unmask,
	.irq_set_type = rtl8380_gpio_irq_set_type
};

static int rtl8380_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gpio_irq_chip *girq;
	struct rtl8380_gpio_ctrl *ctrl;
	u32 ngpios;
	int err, irq;

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	platform_set_drvdata(pdev, ctrl);

	ctrl->dev = dev;

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

	irq = of_irq_get(np, 0);
	if (irq < 0) {
	    dev_err(dev, "failed to find irq number");
	    return irq;
	}

	ctrl->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctrl->base))
		return PTR_ERR(ctrl->base);

	raw_spin_lock_init(&ctrl->lock);

	err = bgpio_init(&ctrl->gc, dev, 4,
		ctrl->base + RTL8380_GPIO_REG_DATA, NULL, NULL,
		ctrl->base + RTL8380_GPIO_REG_DIR, NULL,
		BGPIOF_BIG_ENDIAN_BYTE_ORDER);
	if (err) {
	    dev_err(dev, "unable to init generic GPIO");
	    return err;
	}

	ctrl->gc.ngpio = ngpios;
	ctrl->gc.owner = THIS_MODULE;

	girq = &ctrl->gc.irq;
	girq->chip = &rtl8380_gpio_irq_chip;
	girq->parent_handler = rtl8380_gpio_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(dev, 1, sizeof(*girq->parents),
				GFP_KERNEL);
	if (!girq->parents)
	    return -ENOMEM;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;
	girq->parents[0] = irq;

	/* Disable and clear all interrupts */
	iowrite32(0, ctrl->base + RTL8380_GPIO_REG_IMR(0));
	iowrite32(0, ctrl->base + RTL8380_GPIO_REG_IMR(16));
	iowrite32(swab32(BIT(ngpios)-1), ctrl->base + RTL8380_GPIO_REG_ISR);

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
