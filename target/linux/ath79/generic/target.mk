BOARDNAME:=Generic
FEATURES += squashfs

DEFAULT_PACKAGES += wpad-basic-wolfssl
KERNELNAME := vmlinux vmlinuz
# make Kernel/CopyImage use $LINUX_DIR/vmlinuz
IMAGES_DIR := ../../..

define Target/Description
	Build firmware images for generic Atheros AR71xx/AR913x/AR934x based boards.
endef
