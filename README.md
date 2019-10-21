## Issue Description

Repeatedly connecting and disconnecting a USB host to the USB device port (UDC)
on the sama5d2 while the UDC is in use causes both the UDC and the EMAC on the
chip to stop working. The observable behaviour w.r.t. the EMAC on the sama5d2 is
that packets start backing up in the Linux qdisc and don't get dequeued onto the
network interface. We stop getting transmit complete interrupts (this can be
observed by looking at the output of `/proc/interrupts`). Equally we no longer get
incoming packets.

The observable behaviour w.r.t. USB is that the device enumerates on the host,
but the USB device is otherwise not functional.

## Steps to reproduce

I've reproduced this issue using the buildroot-based Linux image
available from the linux4sam webpage for the SAMA5D2 Xplained devboard
(https://www.at91.com/linux4sam/bin/view/Linux4SAM/Sama5d2XplainedMainPage).

Below are the steps taken (from an Ubuntu 18.04 host) to flash this image to the
SD for the device.

```
# first, verify the image has the correct md5
$ md5sum linux4sam-buildroot-sama5d2_xplained-6.1.img.bz2
a325b40c9a6274d68230bbcbabecbd61  linux4sam-buildroot-sama5d2_xplained-6.1.img.bz2

# extract the image
$ bzip2 -d linux4sam-buildroot-sama5d2_xplained-6.1.img.bz2

# for sanity, check the md5 of the decompressed output
$ md5sum linux4sam-buildroot-sama5d2_xplained-6.1.img
b1e19809a7440757defeb6c00c46778e  linux4sam-buildroot-sama5d2_xplained-6.1.img

# flash the image to the SD card reader on /dev/sdb
$ sudo dd if=linux4sam-buildroot-sama5d2_xplained-6.1.img of=/dev/sdb bs=4M status=progress
```

Insert the SD card to the devboard's SD card slot. Attach an ethernet cable to
the devboard and connect the other end to the local network. Connect a USB cable
from the host machine to the micro-USB port labelled J14 on the board.  Wait for
Linux to boot on the board. On the host, start a serial session in order to run
commands on the devboard. On the devboard, via this serial session, run the
following commands:

```
## check that we're running the expected kernel version
# uname -a
Linux sama5 4.19.56-linux4sam-6.1 #1 Tue Jul 16 16:30:58 CEST 2019 armv7l GNU/Linux

## bring up the network interface, get ip on lan using dhcp
# ifup eth0
udhcpc: started, v1.30.1
udhcpc: sending discover
udhcpc: sending discover
udhcpc: sending select for 10.8.0.166
udhcpc: lease of 10.8.0.166 obtained, lease time 10800
deleting routers
adding dns 10.8.0.1

## if interface already configured, check ip with:
ip a show dev eth0 | grep inet
```

On the host, start pinging the device over the network:
```
$ ping -D 10.8.0.166
[1571242600.058226] 64 bytes from 10.8.0.166: icmp_seq=1 ttl=64 time=0.439 ms
...
```

Now, on the devboard, we need to set up a simple USB device. For our purposes,
let's set up a serial device (this script is available in the repository
as `setup_serial_gadget.sh`):

```
modprobe libcomposite
mount -t configfs configfs /sys/kernel/config
cd /sys/kernel/config/usb_gadget
mkdir g1 && cd g1
mkdir functions/acm.test
mkdir configs/c.1 && cd configs/c.1
ln -s ../../functions/acm.test ./
cd ../..
ls /sys/class/udc > UDC
```

Now connect the host to the micro-USB port labelled J23 on the devboard. A new
serial device should appear on the host. In my case, the device is called
`/dev/ttyACM1` (the name it gets can be observed in the output of `dmesg`). Note
that on Ubuntu 18.04, a program called ModemManager may try to claim the device.
To prevent this, either disable or uninstall ModemManager (to stop, run
`sudo systemctl stop modemmanager`; to uninstall, use `sudo apt purge modemmanager`).

Now we need to run the test program on both the host and the device in order to
exercise the USB device. To compile the test program for both the host and the
device, run:

```
make # compile for host
make CC=/path/to/atmel/gcc # compile for device
```

The output programs should end up in different subdirectories under the
(newly-created) build directory. Alternatively, pre-compiled binaries for Linux
x86-64 and ARM are available under the releases tab of this repository.

Now, on the host, run the test program, providing the serial device to use:

```
build/x86_64-linux-gnu/serial_stress /dev/ttyACM1
```

On the device:
```
./serial_stress /dev/ttyGS0
```

You should observe TX and RX output lines on both sides of the connection.

Now, once serial output can be observed on the device, simply disconnect and
re-connect the USB device, verifying that the connection is re-established each
time and that the previously-setup ping continues to work.

Eventually, USB and networking will stop working on the devboard (the host
`serial_stress` will no longer be able to connect and the pings will start getting
dropped). Serial access (over the J14 port) should continue to work.
