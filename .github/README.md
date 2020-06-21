# OpenWrt for TP-Link EAP245 v1

## Hardware description

TP-Link EAP245 v1 is AC1750 Wireless Dual Band Gigabit Ceiling Mount
Access Point:
- CPU: QCA9563-AL3A MIPS 74kc v5.0 @775MHz with 2.4GHz wifi
- RAM: 128MiB M14D1G1664A-2.5B @650MHz
- Flash: 16MiB 25Q128CSIG SPI NOR
- Wifi 5Ghz: QCA9880-BR4A 5 GHz wifi ath10k
- Gigabit Eth: AR8033-AL1A 1 gigabit lan port (802.3at PoE)

## Platforms

### ar71xx

Release 18.06 works with original TP-ink broken U-Boot. In later releases
the kernel is too big, I switch to uImage firmware format.

As in TP-Link Kernel GPL sources, code from U-Boot is bundled in Linux kernel
to fix initialization of ethernet port. This code is enabled with
EAP245\_V1\_FACTORY\_UBOOT.

### ath79

Modern Linux Kernel with device tree. It requires a U-Boot upgrade to
support uImage and fix ethernet initialization.
[https://github.com/j-d-r/u-boot-QCA956x/releases]

## Installation

Factory firmwares are signed, signature is verified when upgrading by
software. The only way to update is to solder a serial port, U-Boot is
opened.

### Soldering serial port

Serial port can be soldered on PCB J3, pins are TXD, RXD, GND, VCC.
PCB line close to header are missing resistors R225 (TXD), R237 (RXD),
R230 (RXD voltage divider). R225 and R237 can be replaced by wires, and
R230 (voltage divider) must be let opended.

WITHOUT RESISTOR, THERE IS NO PROTECTION. USE 3.3V MAX, AND PLUG TXD TO
SERIAL PORT RX FIRST.

### Das U-Boot upgrade (mandatory for new ar71xs and th79 release)

Original U-Boot is broken, environment variables are not corresponding
to board, kernel parameters must be ignored, it miss ethernet SGMII port
initialization on normal boot. It boot on a ELF kernel with seprate
kernel and root partitions.

Tested release [https://github.com/j-d-r/u-boot-QCA956x/releases]

IT'S A RISKY PROCEDURE IF SOMETHING GOES WRONG YOU WILL HAVE TO UNSOLDIER
AND REPROGRAM FLASH CHIP.

    ath> setenv ipaddr XXX.XXX.XXX.XXX (optional 192.168.1.1 by default)
    ath> setenv serverip XXX.XXX.XXX.XXX (optional 192.168.1.10 by default)
    ath> tftp 0x80800000 u-boot.bin
    ath> erase 0x9f000000 +0x2000
    ath> cp.b $fileaddr 0x9f000000 $filesize
    ath> reset

### OpenWrt

#### Accessing U-Boot

U-Boot is opened, just send C-B to break.

TAKE CARE ALL U-BOOT ENVIRONMENT VARAIABLE ARE NOT CORRESPONDING TO THE
BOARD. Partitions in bootargs and tftp flash command are BROKEN.

#### ar71xx releases

    ath> setenv ipaddr XXX.XXX.XXX.XXX (optional 192.168.1.1 by default)
    ath> setenv serverip XXX.XXX.XXX.XXX (optional 192.168.1.10 by default)
    ath> tftp 0x80800000 openwrt-ar71xx-generic-eap245-v1-squashfs-sysupgrade.bin
    ath> erase 0x9f040000 +0xd80000
    ath> cp.b $fileaddr 0x9f040000 $filesize

#### ath79 releases

    u-boot> tftpboot 0x80800000 openwrt-ath79-generic-tplink-eap245-v1-squashfs-sysupgrade.bin
    u-boot> erase 0x9f040000 +$filesize
    u-boot> cp.b $fileaddr 0x9f040000 $filesize
    u-boot> setenv bootcmd bootm 0x9f040000
    u-boot> saveenv

## Source Code

This port has been refused upstream due to soldering and broken U-Boot.

