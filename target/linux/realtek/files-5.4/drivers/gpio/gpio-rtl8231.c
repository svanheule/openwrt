// SPDX-License-Identifier: GPL-2.0-only

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <asm/mach-rtl838x/mach-rtl83xx.h>

/* RTL8231 registers for LED control */
#define RTL8231_GPIO_FUNC0			0x0000
#define RTL8231_GPIO_FUNC1			0x0001
#define RTL8231_GPIO_PIN_SEL(gpio)		(0x0002 + ((gpio) >> 4))
#define RTL8231_GPIO_HI_PIN_CFG			0x0004
#define RTL8231_GPIO_HI_PIN_DIR_SHIFT		5
#define RTL8231_SW_RESET			BIT(15)
#define RTL8231_GPIO_DIR(gpio)			(0x0005 + ((gpio) >> 4))
#define RTL8231_GPIO_DIR_INPUT			1
#define RTL8231_GPIO_DIR_OUTPUT			0
#define RTL8231_GPIO_DATA(gpio)			(0x001C + ((gpio) >> 4))

/* RTL8380 registers for access to one RTL8231's GPIOs
 * use sw_w32 macros
 */
#define RTL8380_IO_DRIVE_CTRL			0x1010

#define RTL8380_RTL8231_GPIO_CTRL		0xA0E0
#define RTL8380_RTL8231_GPIO_CLOCK_OFFSET	8
#define RTL8380_RTL8231_GPIO_CLOCK_MASK		(0x3 << 8)
#define RTL8380_RTL8231_GPIO_SYNC		BIT(7)
#define RTL8380_RTL8231_GPIO_READY		BIT(6)
#define RTL8380_RTL8231_GPIO_ADDR_OFFSET	1
#define RTL8380_RTL8231_GPIO_ADDR_MASK		(0x1F << 1)
#define RTL8380_RTL8231_GPIO_ENABLE		BIT(0)

/*
#define RTL8380_RTL8231_GPIO_LOW_DIR		0xA0E4
#define RTL8380_RTL8231_GPIO_LOW_HIGH		0xA0E8
#define RTL8380_RTL8231_GPIO_DATA_LOW		0xA0EC
#define RTL8380_RTL8231_GPIO_DATA_HIGH		0xA0F0

#define RTL8380_LED_CTRL			0xA000
#define RTL8380_RTL2831_GPIO_LOW_IMR		0x113C
#define RTL8380_RTL2831_GPIO_HIGH_IMR		0x1140
#define RTL8380_RTL2831_GPIO_LOW_ISR		0x1188
#define RTL8380_RTL2831_GPIO_HIGH_ISR		0x118C
*/

enum rtl8231_bus_type {
	RTL8231_BUS_MII,
	RTL8231_BUS_I2C
};

/**
 * struct rtl8231_gpio_ctrl - Control data for an RTL8231 chip
 *
 * @gc: Associated gpio_chip instance
 * @dev
 * @lock: Mutex to ensure only exclusive access
 * @read: Perform a register read from the chip
 * @write: Perform a register write on the chip
 */
struct rtl8231_gpio_ctrl {
	struct gpio_chip gc;
	struct device *dev;
	struct mutex lock;
	int (*read)(struct rtl8231_gpio_ctrl *ctrl, int reg);
	int (*write)(struct rtl8231_gpio_ctrl *ctrl, int reg, u16 value);

	/**
	 * @reg_width: Amount of bits used to represent a register.
	 * Must be 8 or 16 on and I2C bus, or 5 on an MIIM bus
	 */
	//u32 reg_width;

	/** @bus_type: Serial bus type */
	enum rtl8231_bus_type bus_type;
	/** @bus_addr: Address of the chip */
	unsigned int bus_addr;
	union {
		struct mii_bus *mii_bus;
		u32 ext_gpio_indrt_access;
	};

	/** @reg_shadow: Shadowed chip registers for faster updates */
	u16 reg_shadow[0x20];
	/** @reg_cached: Bitmap to indicate which shadowed registers are valid */
	u32 reg_cached;
};

#define RTL8231_INDIRECT_CMD_EXEC		BIT(0)
#define RTL8231_INDIRECT_CMD_READ		0
#define RTL8231_INDIRECT_CMD_WRITE		BIT(1)
#define RTL8231_INDIRECT_RTL8380_DATA_SHIFT	0x10
#define RTL8231_INDIRECT_RTL9300_DATA_SHIFT	0xC
#define RTL8231_INDIRECT_DATA_MASK		0xFFFF
#define RTL8231_INDIRECT_RTL9300_CMD_FAIL	BIT(28)

static int rtl8231_rtl8380_indirect_read(struct rtl8231_gpio_ctrl *ctrl, u32 reg)
{
	/* SKD MDC read
	 * Set BIT(0) in RTL838X_EXTRA_GPIO_CTRL
	 * Set (RTL838X_DMY_REG5 & ~3) | (1)
	 * Write command to RTL838X_EXT_GPIO_INDRT_ACCESS
	 * Wait for EXEC bit to clear
	 */
	u32 t;
	u8 addr = ctrl->bus_addr;

	/* Calculate read register address */
	t = (addr & 0x1f) << 2;
	t |= (reg & 0x1f) << 7;
	t |= RTL8231_INDIRECT_CMD_EXEC;

	sw_w32(t, ctrl->ext_gpio_indrt_access);
	do {
		t = sw_r32(ctrl->ext_gpio_indrt_access);
	} while (t & RTL8231_INDIRECT_CMD_EXEC);

	t = t >> RTL8231_INDIRECT_RTL8380_DATA_SHIFT;

	return t & RTL8231_INDIRECT_DATA_MASK;
}

static int rtl8231_rtl8380_indirect_write(struct rtl8231_gpio_ctrl *ctrl, u32 reg, u16 data)
{
	u32 t;
	u8 addr = ctrl->bus_addr;

	t = (addr & 0x1f) << 2;
	t |= (reg & 0x1f) << 7;
	t |= data << RTL8231_INDIRECT_RTL8380_DATA_SHIFT;
	t |= RTL8231_INDIRECT_CMD_WRITE | RTL8231_INDIRECT_CMD_EXEC;

	sw_w32(t, ctrl->ext_gpio_indrt_access);
	do {
		t = sw_r32(ctrl->ext_gpio_indrt_access);
	} while (t & RTL8231_INDIRECT_CMD_EXEC);

	ctrl->reg_shadow[reg] = data;
	ctrl->reg_cached |= BIT(reg);

	return 0;
}

static int rtl8231_mii_read(struct rtl8231_gpio_ctrl *ctrl, int reg)
{
	int err = ctrl->mii_bus->read(ctrl->mii_bus, ctrl->bus_addr, reg);
	dev_dbg(ctrl->dev, "MIIM READ [%x] A%02x R%02x : %04x\n", err,
		ctrl->bus_addr, reg, err & 0xffff);
	return err;
}

static int rtl8231_mii_write(struct rtl8231_gpio_ctrl *ctrl, int reg, u16 data)
{
	int err = ctrl->mii_bus->write(ctrl->mii_bus, ctrl->bus_addr, reg, data);
	dev_dbg(ctrl->dev, "MIIM WRITE [%x] A%02x R%02x : %04x\n", err,
		ctrl->bus_addr, reg, data);
	return err;
}

static int rtl8231_read_cached(struct rtl8231_gpio_ctrl *ctrl, u32 reg)
{
	if (ctrl->reg_cached & BIT(reg))
		return ctrl->reg_shadow[reg & 0x1f];

	return ctrl->read(ctrl, reg);
}

static int rtl8231_update_bit(struct rtl8231_gpio_ctrl *ctrl, int reg, int bit, u32 value)
{
	int v;
	int mask = BIT(bit);
	int bit_value = value << bit;

	mutex_lock(&ctrl->lock);
	v = ctrl->read(ctrl, reg);
	if (v >= 0 && (v & mask) != bit_value) {
		v = (v & ~mask) | bit_value;
		v = ctrl->write(ctrl, reg, v);
	}
	mutex_unlock(&ctrl->lock);

	if (v < 0)
		pr_err("error writing register bit\n");

	return v;
}

/* Set Direction of the RTL8231 pin:
 * dir 1: input
 * dir 0: output
 */
static int rtl8231_pin_dir_write(struct rtl8231_gpio_ctrl *ctrl, u32 gpio, u32 dir)
{
	int pin_dir_addr = RTL8231_GPIO_DIR(gpio);
	int bit = gpio % 16;

	if (gpio > 31) {
		pin_dir_addr = RTL8231_GPIO_HI_PIN_CFG;
		bit += RTL8231_GPIO_HI_PIN_DIR_SHIFT;
	}

	return rtl8231_update_bit(ctrl, pin_dir_addr, bit, dir);
}

static int rtl8231_pin_write(struct rtl8231_gpio_ctrl *ctrl, u32 gpio, u32 data)
{
	unsigned bit = gpio % 16;

	return rtl8231_update_bit(ctrl, RTL8231_GPIO_DATA(gpio), bit, data);
}

static int rtl8231_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	return rtl8231_pin_dir_write(ctrl, offset, 1);
}

static int rtl8231_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	int err;

	err = rtl8231_pin_dir_write(ctrl, offset, 0);
	if (!err)
		err = rtl8231_pin_write(ctrl, offset, value);
	return err;
}

static int rtl8231_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	int dir_addr = RTL8231_GPIO_DIR(offset);
	int bit = offset % 16;
	int v = 0;

	if (offset > 31) {
		dir_addr = RTL8231_GPIO_HI_PIN_CFG;
		bit += RTL8231_GPIO_HI_PIN_DIR_SHIFT;
	}

	pr_debug("%s: %d\n", __func__, offset);

	v = ctrl->read(ctrl, dir_addr);

	if (v < 0)
		return v;
	else
		return !!(v & BIT(bit));
}

static int rtl8231_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);
	int v;

	v = ctrl->read(ctrl, RTL8231_GPIO_DATA(offset));

	if (v < 0)
		return v;
	else
		return !!(v & BIT(offset % 16));
}

void rtl8231_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct rtl8231_gpio_ctrl *ctrl = gpiochip_get_data(gc);

	rtl8231_pin_write(ctrl, offset, value);
}

int rtl8231_init(struct rtl8231_gpio_ctrl *ctrl)
{
	u32 v;

	// FIXME Hacks
	/* set drive bit to high current (?) */
	sw_w32_mask(0, BIT(1), RTL8380_IO_DRIVE_CTRL);

	/* clock cannot be adjusted while enable bit is set
	 * first disable the HW peripheral, then set clock to 0
	 */
	sw_w32_mask(RTL8380_RTL8231_GPIO_ENABLE, 0, RTL8380_RTL8231_GPIO_CTRL);
	sw_w32_mask(3 << RTL8380_RTL8231_GPIO_CLOCK_OFFSET, 0, RTL8380_RTL8231_GPIO_CTRL);

	/*if (soc_info.family == RTL8390_FAMILY_ID) {
		sw_w32_mask(0x7 << 18, 0x4 << 18, RTL839X_LED_GLB_CTRL);
		return 0;
	}*/

	/* Enable RTL8231 indirect access mode
	 * FIXME This needs to go into a separate access mode init
	 */
	//sw_w32_mask(0, 1, RTL838X_EXTRA_GPIO_CTRL);
	//sw_w32_mask(3, 1, RTL838X_DMY_REG5);

	ctrl->write(ctrl, RTL8231_GPIO_HI_PIN_CFG, RTL8231_SW_RESET);

	ctrl->reg_cached = 0;

	/* if device is present and ready, FUNC1[9:4] should be 0x37 */
	v = ctrl->read(ctrl, RTL8231_GPIO_FUNC1);
	if (v < 0 || ((v >> 4) & 0x3f) != 0x37) {
		dev_err(ctrl->dev,
			"RTL8231 at address %d not present or ready (READY=%02x)\n",
			ctrl->bus_addr, (v >> 4) & 0x3f);
		return -1;
	}

	dev_info(ctrl->dev, "RTL8231 found at address %d\n", ctrl->bus_addr);

	/* Set device enable bit */
	v = ctrl->read(ctrl, RTL8231_GPIO_FUNC0);
	ctrl->write(ctrl, RTL8231_GPIO_FUNC0, v | BIT(1));

	/* Select GPIO functionality for all pins and set to input */
	ctrl->write(ctrl, RTL8231_GPIO_PIN_SEL(0), 0xffff);
	ctrl->write(ctrl, RTL8231_GPIO_DIR(0), 0xffff);
	ctrl->write(ctrl, RTL8231_GPIO_PIN_SEL(16), 0xffff);
	ctrl->write(ctrl, RTL8231_GPIO_DIR(16), 0xffff);

	v = ctrl->read(ctrl, RTL8231_GPIO_HI_PIN_CFG);
	v |= (0x1f << RTL8231_GPIO_HI_PIN_DIR_SHIFT) | (0x1f);
	ctrl->write(ctrl, RTL8231_GPIO_HI_PIN_CFG, v);

	return 0;
}

static const struct of_device_id rtl8231_gpio_of_match[] = {
	{ .compatible = "realtek,rtl8231-gpio" },
	{},
};

MODULE_DEVICE_TABLE(of, rtl8231_gpio_of_match);

static int rtl8231_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *expander_np = NULL;
	struct device_node *bus_np = NULL;
	struct rtl8231_gpio_ctrl *ctrl;
	int err;
	u32 bus_addr;
	u32 ngpios;

	if (!np) {
		dev_err(dev, "no DT node found\n");
		return -EINVAL;
	}

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	err = of_property_read_u32(np, "ngpios", &ngpios);
	if (err) {
		dev_err(dev, "invalid ngpios property\n");
		return err;
	}

	if (ngpios > 37) {
		dev_err(dev, "ngpios must be smaller than or equal to 37\n");
		return -EINVAL;
	}

	expander_np = of_parse_phandle(np, "dev-handle", 0);
	if (!expander_np) {
		dev_err(dev, "missing dev-handle node\n");
		return -EINVAL;
	}

	bus_np = of_get_parent(expander_np);
	if (!bus_np) {
		dev_err(dev, "cannot find parent bus\n");
		err = -EINVAL;
		goto err_expander_invalid;
	}

	if (of_node_name_prefix(bus_np, "mdio")) {
		ctrl->bus_type = RTL8231_BUS_MII;
	}
	else if (of_node_name_prefix(bus_np, "i2c")) {
		ctrl->bus_type = RTL8231_BUS_I2C;
	}
	else {
		dev_err(dev, "invalid bus type\n");
		return -EINVAL;
	}

	switch (ctrl->bus_type) {
	case RTL8231_BUS_MII:
		ctrl->mii_bus = of_mdio_find_bus(bus_np);
		if (!ctrl->mii_bus) {
			dev_err(dev, "bus is not a valid mii bus\n");
			err = -EPROBE_DEFER;
			goto err_bus_invalid;
		}
		ctrl->read = rtl8231_mii_read;
		ctrl->write = rtl8231_mii_write;
		break;
	case RTL8231_BUS_I2C:
	default:
		dev_err(dev, "bus type not supported\n");
		err = -EINVAL;
		goto err_bus_invalid;
	}

	if (of_property_read_u32(expander_np, "reg", &bus_addr) != 0) {
		dev_err(dev, "missing DT reg node");
		err = -EINVAL;
		goto err_bus_invalid;
	}

	if (ctrl->bus_type == RTL8231_BUS_I2C) {
		if (bus_addr > 0x7) {
			dev_err(dev, "address for I2C must be smaller than 8\n");
			err = -EINVAL;
			goto err_bus_invalid;
		}
		/* Complete 7-bit I2C address is [1 0 1 0 A2 A1 A0] */
		bus_addr |= 0x50;
	}
	else if (bus_addr > 0x1f) {
		dev_err(dev, "address for MII must be smaller than 32\n");
		err = -EINVAL;
		goto err_bus_invalid;
	}

	ctrl->bus_addr = bus_addr;

	// TODO optional reset-gpios node, although SW reset is already used

	ctrl->dev = dev;
	mutex_init(&ctrl->lock);
	err = rtl8231_init(ctrl);
	if (err < 0)
		goto err_bus_invalid;

	ctrl->gc.base = -1;
	ctrl->gc.ngpio = ngpios;
	ctrl->gc.label = "rtl8231-gpio";
	ctrl->gc.parent = dev;
	ctrl->gc.owner = THIS_MODULE;
	ctrl->gc.can_sleep = true;

	ctrl->gc.set = rtl8231_gpio_set;
	ctrl->gc.get = rtl8231_gpio_get;
	ctrl->gc.direction_input = rtl8231_direction_input;
	ctrl->gc.direction_output = rtl8231_direction_output;
	ctrl->gc.get_direction = rtl8231_get_direction;

	err = devm_gpiochip_add_data(dev, &ctrl->gc, ctrl);

err_bus_invalid:
	of_node_put(bus_np);

err_expander_invalid:
	of_node_put(expander_np);

	return err;
}

static struct platform_driver rtl8231_gpio_driver = {
	.driver = {
		.name = "rtl8231-gpio",
		.of_match_table	= rtl8231_gpio_of_match,
	},
	.probe = rtl8231_gpio_probe,
};

module_platform_driver(rtl8231_gpio_driver);

MODULE_DESCRIPTION("Realtek RTL8231 GPIO expansion chip support");
MODULE_LICENSE("GPL v2");
