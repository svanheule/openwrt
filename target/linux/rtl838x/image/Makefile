# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

KERNEL_LOADADDR = 0x80000000
KERNEL_ENTRY = 0x80000400

DEVICE_VARS += UIMAGE_MAGIC

define Build/realtek-uImage
	$(call Build/uImage,$(1) $(if $(UIMAGE_MAGIC),-M $(UIMAGE_MAGIC),))
endef

define Device/Default
  PROFILES = Default
  KERNEL := kernel-bin | append-dtb | gzip | realtek-uImage gzip
  KERNEL_INITRAMFS := $$(KERNEL)
  DEVICE_DTS_DIR := ../dts
  DEVICE_DTS = $$(SOC)_$(1)
  SUPPORTED_DEVICES := $(subst _,$(comma),$(1))
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := append-kernel | append-rootfs | \
      append-metadata | check-size
endef

define Device/allnet_all-sg8208m
  $(Device/Default)
  SOC := rtl8382
  IMAGE_SIZE := 7168k
  DEVICE_VENDOR := ALLNET
  DEVICE_MODEL := ALL-SG8208M
  UIMAGE_MAGIC := 0x00000006
  UIMAGE_NAME := 2.2.2.0
  DEVICE_PACKAGES := ip-full ip-bridge kmod-gpio-button-hotplug tc
endef
TARGET_DEVICES += allnet_all-sg8208m

$(eval $(call BuildImage))
