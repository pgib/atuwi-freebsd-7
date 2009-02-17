		ATUWI - The FreeBSD USB WLAN driver project


**** What it is

A FreeBSD driver for Atmel based USB WLAN adapters. The aim of the project is
to get all Atmel at76c503a compatible USB WLAN adapters to work on FreeBSD.



**** Current driver features

- Support for Intersil, RFMD, RFMD2958 radio parts.
- Supports boards based on both the AT76c503 and the AT76c505 chipset.
- Support for the SMC 2662W v.4 adapter.
- Support for Ad-hoc networks.
- Support for Infrastructure mode (currently in infrastructure mode only 'open
  system' authentication is implemented).
- Setting the channel / SSID / WEP mode / WEP key / mac-addr via ifconfig.
- Automatic scanning and associating with a wireless network.
- bpf support for both DLT_EN10MB, DLT_IEEE802_11 and DLT_AIRONET_HEADER.
- Also compiles on FreeBSD 5.2 and later now.



**** Currently supported devices

At this moment the following USB devices are supported by the driver :
Vendor ID		   Product ID		Radio type  Firmware file
0x03eb (Atmel)		   0x7605 (BW002)	RFMD	    rfmd
0x0d5c (SMC)		   0xa002 (2662W-AR)	RFMD	    rfmd
0x077b (Linksys)	   0x2219 (WUSB11)	RFMD	    rfmd
0x04a5 (Acer Peripherals)  0x9001 (AWL400)	RFMD	    rfmd
0x03eb (Atmel)		   0x7613 (WL-1330)	RFMD2958    rfmd2958
0x1915 (Linksys)	   0x2233 (WUSB11-V28)	RFMD2958    rfmd2958
0x12fd (Aincomm)	   0x1001 (AWU2000B)	RFMD2958    rfmd2958
0x03eb (Atmel)		   0x7614 (2662W-V4)	RFMD2958    rfmd2958-smc
0x03eb (Atmel)		   0x7603 (DWL-120)	Intersil    intersil
0x04a5 (AcerP)		   0x9000 (AWL-300)	Intersil    intersi

Other devices might also work, please let me know if you find a device with
different USB ID's.



**** Known BUGS and limitations

- 'shared key' authentication is not yet implemented.
- There is no timeout when we're associated and connected to a network. Once
  the driver is associated with an AP it assumes the connection will stay
  forever.
- Nothing is done with the reg-domain of the device. Right now setting an
  unsupported channel will not return any error, but the device will simply
  refuse to work.
- Only 'auto rate fallback' is supported at this moment. There is no way to
  force the driver to use a specified transmit-rate.
- The firmware doesn't have a real promiscuous mode. We only receive things
  addressed to us and to the broadcast address.



**** Compiling the driver

To use this driver you need FreeBSD 5.1-RELEASE or later with the kernel
sources installed.

First, unpack the driver into your source tree. (You may want to create backups
of your source tree before you begin.)

# cd /usr/src/sys
# tar -xvzf /the/path/to/atuwi-0.6.tar.gz

This should extract the following files :
conf/files-atuwi-2004-01-11.diff
dev/usb/atuwi_intersil_fw.h
dev/usb/atuwi_rfmd2958-smc_fw.h
dev/usb/atuwi_rfmd2958_fw.h
dev/usb/atuwi_rfmd_fw.h
dev/usb/if_atuwi.c
dev/usb/if_atuwireg.h
dev/usb/uhci-waitintr-2004-10-22.diff
dev/usb-reset-2004-01-08.diff
modules/atuwi/CHANGELOG
modules/atuwi/Makefile
modules/atuwi/atuwi-compile
modules/atuwi/atuwi-README.txt

Since version 0.5 of the atuwi driver, the driver comes with a script that
automatically compiles the driver. To start the compile-script, go into the
directory where your kernel sources are installed (normally /usr/src/sys ) and
simply type the following :

# ./modules/atuwi/atuwi-compile

If the compile script finished successfully you can skip the next chapter and
continue with the chapter "Testing the driver".



**** Compiling the driver manually

If the atuwi-compile script failed, or if you like to compile the driver step
by step by hand, please follow the following steps :

The atuwi-driver uses a system called 'DFU' to upload firmware into the USB
adapter. After the firmware has been uploaded, the adapter has to be resetted.
The current USB stack doesn't have the ability to do this in a nice way, so I
have added this functionality and provided a patch.

Patching your USB code and recompiling it can be done as follows:
# cd /usr/src/sys/dev
# patch <usb-reset-2004-01-08.diff

FreeBSD's UHCI driver has an error in it that causes a system to hang when
booted with a wireless USB adapter plugged in. To prevent this I have submitted
a PR with a patch, so new versions of FreeBSD should already have this fixed.
For all versions that don't have this patched yet you should patch it with the
patch file that comes with the driver. If you're not sure if your system
already has the patch or not, just try to patch it and "patch" will complain if
the patch is already there. Type the following to apply the patch:
# cd /usr/src/sys/dev/usb
# patch <uhci-waitintr-2004-10-22.diff

Now update the usbdevs list with the ID's of the WLAN adapters and rebuild it:

(this can also be done using cvs, but for ease of use we'll just get the
usbdevs list from the cvsweb interface using fetch)
# cd /usr/src/sys/dev/usb
# fetch http://www.freebsd.org/cgi/cvsweb.cgi/~checkout~/src/sys/dev/usb/usbdevs
# make -f Makefile.usbdevs

After that it's time to compile the driver itself :
# cd /usr/src/sys/modules/atuwi
# make



**** Testing the driver

Note that the driver comes with patches to the usb subsystem. For these patches
to take effect you'll have to recompile your kernel and reboot now. Otherwise
the driver will work, but you will have to use a trick to get your wireless
device to work. (Which is explained later on)

If you've made it up to here you should now be the happy owner of a file called
'if_atuwi.ko'. Let's try to load it and see if it works...
# kldload /usr/src/sys/modules/atuwi/if_atuwi.ko

Now plug in the WLAN adapter and watch the debug output. If all goes well the
debug messages will stop with something like :
'atuwi0: Atmel Wireless adapter, rev 1.10/1.00, addr 3 attached successfully'

Congratulations, you can now begin configuring the interface (and skip the next
chapter). 

Please read the next chapter if the debug messages stop with something like :
'device_probe_and_attach: atuwi0 attach returned 35'

If you don't see any debug messages that start with 'atuwi0', you must have a
WLAN adapter that's either not supported, or not yet listed in the driver.
Please email me ( Danovitsch @ Vitsch . net ) the output of 'usbdevs -v' so I
can add the USB ID's to the driver.



**** How to reset the adapter without the reset code in the kernel?

If for some reasone you are using a kernel without my reset modifications, you
will have to use a little trick to reset the adapter. If you remove a USB plug
from a hub, you first disconnect the two data pins and then the two pins
supplying the power. (This is because the data pins are shorter in length.) So
if you gently unplug the USB connector only half way out of the hub (about 2
milimeters) the data pins are disconnected from the hub, but the device is
still powered. This means the hub will detect the device is gone, but the
firmware will stay in the RAM of the device. When you plug the device back in
now, the hub will detect the device again. Doing this is fairly easy, but it
won't always work on the first attempt. Just try a couple of times.



**** Examples of joining a network in Ad-Hoc and Infrastructure mode

The following are a couple of self-explenatory examples that show how the
interface can be configured :

# ifconfig atuwi0 192.168.1.10 ssid "SomeName"
This command let the driver scan all channels and try to join an Access Point
with ssid 'SomeName'.

# ifconfig atuwi0 192.168.1.10 ssid "Some Other Name" mediaopt adhoc channel 3
After this command the driver will scan all channels for an adhoc network with
ssid 'Some Other Name'. It will connect to the first match it finds, regardless
of the given channel. If no network is found, the driver will start it's own
network on channel 3.

Switching back from Ad-Hoc mode to Infrastructure mode can be done with this
command :
# ifconfig atuwi0 -mediaopt adhoc

And switching from Infrastructure to Ad-Hoc can be done like this:
# ifconfig atuwi0 mediaopt adhoc



**** Configuring the interface to use WEP encryption

The following command will set a 64-bit WEP key :
# ifconfig atuwi0 wepmode on weptxkey 1 wepkey 1:0x1122334455

And setting a 128-bit WEP key :
# ifconfig atuwi0 wepmode on weptxkey 1 wepkey 1:0x11223344556677889900112233

Turning WEP off again :
# ifconfig atuwi0 wepmode off

Possible wepmode's are :
off    no encryption is done when transmitting.
mixed  transmitted packets are encrypted, but both encrypted and non-encrypted
       packets are received.
on     transmitted packets are encrypted. only WEP encrypted packets are
       received.

WEP encrypted packets will ONLY be received correctly if the transmitter and
the receiver are using the exact same WEP key (wether we're in mixed mode or
not).

When setting WEP keys, make sure the length is exactly 10 hex-digits for 64-bit
keys or 23 hex-digits for 128-bits.



**** Setting up your system to automagically configure the interface

If you are going to plug/unplug the adapter, you probably want the thing to be
automatically configured when it's plugged in. USBD will do this for us after
some minor modifications.

# edit /etc/usbd.conf

Somewhere in the file add the next couple of lines :
device "USB WLAN"
                 devname "atuwi[0-9]+" 
                 attach  "/etc/pccard_ether ${DEVNAME} start"
                 detach  "/etc/pccard_ether ${DEVNAME} stop"

Save the file and restart usbd :
# killall -TERM usbd
# usbd

Now add your settings to /etc/rc.conf . The syntax should look like this :
ifconfig_atuwi0="192.168.3.20 ssid TheNetworkName wepmode on weptxkey 1 etc.."

To let the system know we want to automatically load the settings add :
removable_interfaces="atuwi0"

And to make 'usbd' start during boot you should have :
usbd_enable="YES"

After plugging the device into the usb hub it should now be automatically
detected and configured.



**** Compiling the driver into the kernel

Although it's possible to load the driver at boot time it might be easier to
compile the driver into the kernel sometimes. To do this we have to add the
driver to the list of options that can be choosen in a kernel-config :
# cd /usr/src/sys/conf
# patch files files-atuwi-2004-01-11.diff

Now edit your kernel config file and simply add a line with 'device atuwi' to
it. Re-config, compile, install, reboot and the adapter should initialise at
boot time :)



**** Using the driver during 'sysinstall' from a floppy

What if you're with your laptop in the middle of nowhere, far from your cat-5
hub, but just in range of a wireless network and you suddenly feel the urge to
install FreeBSD? Well... I found myself in that situation yesterday and I found
out that using the driver from sysinstall is very easy and completely painless
:)

First, make your kernel & mfsroot floppies. Then take a third floppy and put
the driver on it :

Format the floppy
# fdformat /dev/fd0
# disklabel -w -r /dev/fd0 fd1440
# newfs /dev/fd0

Mount it, copy the driver and unmount it again.
# mount -t ufs /dev/fd0 /mnt
# cp /usr/src/sys/modules/atuwi/if_atuwi.ko /mnt
# umount /mnt

Now boot your system with the kernel & mfsroot floppies. When sysinstall
appears simply insert the driver floppy and follow the menu to load it. During
install things like ssid and WEP-settings can be filled in in the box 'extra
options to ifconfig'.

For those that are having a little chicken-and-egg problem with the 'compile
the driver, put it on a floppy, and then install FreeBSD' order and for the
impatient, the floppy image can also be downloaded from the internet from :

For FreeBSD 5.1-RELEASE :
http://vitsch.net/bsd/atuwi/floppies/atuwi-0.6_5.1-RELEASE.flp.tar.gz

For FreeBSD 5.2.1-RELEASE :
http://vitsch.net/bsd/atuwi/floppies/atuwi-0.6_5.2.1-RELEASE.flp.tar.gz

For FreeBSD 5.3-BETA7 :
http://vitsch.net/bsd/atuwi/floppies/atuwi-0.6_5.3-BETA7.flp.tar.gz

For FreeBSD current (as of 2004-10-18) :
http://vitsch.net/bsd/atuwi/floppies/atuwi-0.6_current.flp.tar.gz

After downloading, extract the image and burn it onto a floppy. The driver will
support all devices currently supported by the driver.

For information on how to burn floppy images onto a floppy please read the
floppy-readme :
http://vitsch.net/bsd/atuwi/floppies/floppy-readme.txt



**** Some final tips

By default the driver tends to be a bit too verbose. The debug flags can be
altered on-the-fly with sysctl variable 'hw.atuwi.debug' .

This will silence the driver:
sysctl -w hw.atuwi.debug=0x00

For an explanation of possible debug flags, settings and optimizations of the
driver please have a look at /usr/src/sys/dev/usb/if_atuwireg.h



**** Driver history

Releaese    Release date
2004-10-23  Version 0.6 released. Added a bug fix to the FreeBSD UHCI driver
2004-10-18  Version 0.5 released. Added support for DLT_AIRONET_HEADER
2004-05-22  Version 0.4 released. Now also supports at76c505 based boards
2004-05-04  Version 0.3 released. Fixed compile errors on FreeBSD 5.2.1-RELEASE
2004-01-12  Version 0.2 released. All basic functionality works
2003-09-02  First public release - 0.1 - adhoc only.
2003-06-30  First successfull firmware upload & ping via ugen interface

$ATUWI: $Id: atuwi-README.daan,v 1.12 2004/10/23 18:45:35 daan Exp $

