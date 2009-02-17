atuwi driver for FreeBSD 7.1
============================

Back in 2004, [Daan Vreeken](http://vitsch.net/bsd/atuwi/) created a driver for Atmel-based USB WLAN adapters. This includes the Linksys WUSB11v2.8 adapter that I happen to possess and need to use on one of my projects. Unfortunately, much has changed in FreeBSD driver structure since FreeBSD 5.3beta7, which was when the last update was made to this driver. It has since been cleaned up and imported into NetBSD and OpenBSD, but sadly no one in the FreeBSD world took this on.

Enter me. Now, I'm no C programmer, and am certainly not a kernel hacker by any means. In fact, I'm not even really a procedural programmer. But with a bit of work and examination of other current drivers for FreeBSD, I've been able to bring this driver to state where it compiles and loads. So far, I haven't been able to actually connect to my wifi network, but one step at a time. I'm pretty impressed that I've made it this far.

Note that I have included Daan's entire source tree which also includes various patches to the FreeBSD source tree. These patches no longer cleanly apply, and they are really to work-around some deficiency that I don't fully understand. Once the driver uploads the firmware to the USB WLAN adapter, it needs to "reset" the device. Daan has provided a work-around so that you don't have to patch your source tree, which is to unplug the USB cable part way so that power is still retained but so that the system sees the device disconnected. When you plug it back in, the uploaded firmware is retained on the device, and the atuwi driver can finish loading properly.

If you're interested in building this driver on FreeBSD 7.1, take the following steps:

1. Copy the .h and .c files dev/usb/ to /usr/src/sys/dev/usb.
2. Copy the modules/atuwi folder into /usr/src/sys/modules.
3. Go into /usr/src/sys/modules and type: make && make install
