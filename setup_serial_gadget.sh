#!/bin/sh
modprobe libcomposite
mount -t configfs configfs /sys/kernel/config
cd /sys/kernel/config/usb_gadget
mkdir g1 && cd g1
mkdir functions/acm.test
mkdir configs/c.1 && cd configs/c.1
ln -s ../../functions/acm.test ./
cd ../..
ls /sys/class/udc > UDC
