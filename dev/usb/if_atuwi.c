/*
 * Copyright (c) 2003, 2004
 *	Daan Vreeken <Danovitsch@Vitsch.net>.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software       
 *    must display the following acknowledgement:
 *	This product includes software developed by Daan Vreeken.
 * 4. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Daan Vreeken AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Daan Vreeken OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Atmel AT76c503 / AT76c503a / AT76c505 / AT76c505a  USB WLAN driver
 * version 0.5 - 2004-08-03
 *
 * Written by Daan Vreeken <Danovitsch @ Vitsch . net>
 *  http://vitsch.net/bsd/atuwi
 *
 * Contributed to by :
 *  Chris Whitehouse, Alistair Phillips, Peter Pilka, Martijn van Buul,
 *  Suihong Liang, Arjan van Leeuwen, Stuart Walsh
 *
 */

#include <sys/types.h>
#include <sys/cdefs.h>

__FBSDID("$ATUWI: $Id: if_atuwi.c,v 1.27 2004/10/25 11:31:26 daan Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_media.h>

#include <net/bpf.h>

#include <sys/bus.h>
#include <sys/kthread.h>
#include <sys/queue.h>
#include <machine/bus.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_ethersubr.h>

/* usbdevs.h locations change of 2004-06-27 */
#if __FreeBSD_version < 502119
#include <dev/usb/usbdevs.h>
#else
#include <modules/usb/usbdevs.h>
#endif

#include <dev/wi/if_wavelan_ieee.h>

#if __FreeBSD_version < 502000
#include <net/if_ieee80211.h>
#else
#include <net80211/ieee80211.h>
#include <net80211/ieee80211_ioctl.h>
#endif

#include <dev/an/if_aironet_ieee.h>

#include <dev/usb/if_atuwireg.h>
#ifndef ATUWI_NO_RFMD
#include <dev/usb/atuwi_rfmd_fw.h>
#endif
#ifndef ATUWI_NO_RFMD2958
#include <dev/usb/atuwi_rfmd2958_fw.h>
#endif
#ifndef ATUWI_NO_RFMD2958_SMC
#include <dev/usb/atuwi_rfmd2958-smc_fw.h>
#endif
#ifndef ATUWI_NO_INTERSIL
#include <dev/usb/atuwi_intersil_fw.h>
#endif

#include <sys/sysctl.h>


#define FLAG_ALWAYS		0x80000

/*
 * non-safe way of printf-ing...
 *
#define DEBUG(flags, msg)	if ((atuwi_debug | FLAG_ALWAYS) & flags) { \
					printf msg;			   \
				}
#define ERROR(msg)		printf msg;
*/

/*
 * safe printf macro's
 * if (curthread == mgmt_thread) lock Giant while calling printf.
 * this will keep SMP machine's from spitting garbage on the console.
 * is there a better way to do this? (is there a special console-lock or
 * something?)
 */
#define DEBUG(flags, msg)						\
	if ((atuwi_debug | FLAG_ALWAYS) & flags) {			\
		if ((sc->atuwi_mgmt_flags & ATUWI_TASK_RUNNING) &&	\
		    (sc->atuwi_mgmt_thread->p_pid ==			\
		    curthread->td_proc->p_pid)) {			\
			mtx_lock(&Giant);				\
			printf msg;					\
			mtx_unlock(&Giant);				\
		} else 							\
			printf msg;					\
	}
#define ERROR(msg)							\
	if ((sc->atuwi_mgmt_flags & ATUWI_TASK_RUNNING) &&		\
	    (sc->atuwi_mgmt_thread->p_pid == curthread->td_proc->p_pid)) { \
	    	mtx_lock(&Giant);					\
		printf msg;						\
		mtx_unlock(&Giant);					\
	} else 								\
		printf msg;						\



int	atuwi_debug = ATUWI_INITIAL_DEBUG_LEVEL;



SYSCTL_NODE(_hw, OID_AUTO, atuwi, CTLFLAG_RW, 0, "Atmel USB WLAN adapter");
SYSCTL_INT(_hw_atuwi, OID_AUTO, debug, CTLFLAG_RW, &atuwi_debug, 0,
   "debug flags");

#ifdef ATUWI_DEBUG_USB
int	atuwi_debug_usb = 1;

SYSCTL_INT(_hw_atuwi, OID_AUTO, debug_usb, CTLFLAG_RW,
        &atuwi_debug_usb, 0, "usb requests debugging");
#endif /* ATUWI_DEBUG_USB */


#ifdef ATUWI_PROFILING
SYSCTL_NODE(_hw_atuwi, OID_AUTO, profiling, CTLFLAG_RW, 0, "Atmel USB WLAN adapter");

int	atuwi_profiling_init = 0;
int	atuwi_profiling_rxstart = 0;
int	atuwi_profiling_rxstart_noxfer = 0;
int	atuwi_profiling_rxstart_nobuf = 0;
int	atuwi_profiling_rxeof = 0;
int	atuwi_profiling_packets_in = 0;
int	atuwi_profiling_bytes_in = 0;
int	atuwi_profiling_start = 0;
int	atuwi_profiling_start_noxfer = 0;
int	atuwi_profiling_start_nopackets = 0;
int	atuwi_profiling_txeof = 0;
int	atuwi_profiling_packets_out = 0;
int	atuwi_profiling_bytes_out = 0;
int	atuwi_profiling_netisrs_scheduled = 0;
int	atuwi_profiling_netisrs_saved = 0;
int	atuwi_profiling_stop = 0;

SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, init_calls, CTLFLAG_RW,
        &atuwi_profiling_init, 0, "nr of atuwi_init calls");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, rxstart_calls, CTLFLAG_RW,
        &atuwi_profiling_rxstart, 0, "nr of atuwi_rxstart calls");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, rxstart_calls_noxfer, CTLFLAG_RW,
        &atuwi_profiling_rxstart_noxfer, 0, "nr of atuwi_rxstart calls with "
        "no xfer available");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, rxstart_calls_nobuf, CTLFLAG_RW,
        &atuwi_profiling_rxstart_nobuf, 0, "nr of atuwi_rxstart calls with "
        "no buffer available");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, rxeof_calls, CTLFLAG_RW,
        &atuwi_profiling_rxeof, 0, "nr of atuwi_rxeof calls");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, packets_in, CTLFLAG_RW,
        &atuwi_profiling_packets_in, 0, "nr of usb packets in");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, bytes_in, CTLFLAG_RW,
        &atuwi_profiling_bytes_in, 0, "nr of usb bytes in");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, start_calls, CTLFLAG_RW,
        &atuwi_profiling_start, 0, "nr of atuwi_start calls");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, start_calls_noxfer, CTLFLAG_RW,
        &atuwi_profiling_start_noxfer, 0, "nr of atuwi_start calls with no "
        "xfer available");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, start_calls_nopackets, CTLFLAG_RW,
        &atuwi_profiling_start_nopackets, 0, "nr of atuwi_start calls with "
        "no packets on TX queue");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, txeof_calls, CTLFLAG_RW,
        &atuwi_profiling_txeof, 0, "nr of atuwi_txeof calls");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, packets_out, CTLFLAG_RW,
        &atuwi_profiling_packets_out, 0, "nr of usb packets out");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, bytes_out, CTLFLAG_RW,
        &atuwi_profiling_bytes_out, 0, "nr of usb bytes out");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, netisrs_scheduled, CTLFLAG_RW,
        &atuwi_profiling_netisrs_scheduled, 0, "the number of scheduled "
        "netisr's from tx interrupt");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, netisrs_saved, CTLFLAG_RW,
        &atuwi_profiling_netisrs_saved, 0, "the number of netisr callbacks "
        "we could save by being lazy");
SYSCTL_INT(_hw_atuwi_profiling, OID_AUTO, stop_calls, CTLFLAG_RW,
        &atuwi_profiling_stop, 0, "nr of atuwi_stop calls");
        
#endif /* ATUWI_PROFILING */



/*
 * Various supported device vendors/products/radio type.
 */
Static struct atuwi_type atuwi_devs[] = {
#ifndef ATUWI_NO_RFMD
	/* 0x03eb		0x7605 */
	{ USB_VENDOR_ATMEL,	USB_PRODUCT_ATMEL_BW002,
	  RadioRFMD,		ATUWI_NO_QUIRK },
	/* 0x0d5c		0xa002 */
	{ USB_VENDOR_SMC3,	USB_PRODUCT_SMC3_2662WUSB,
	  RadioRFMD,		ATUWI_NO_QUIRK },
	/* 0x077b		0x2219 */
	{ USB_VENDOR_LINKSYS2,	USB_PRODUCT_LINKSYS2_WUSB11,
	  RadioRFMD,		ATUWI_NO_QUIRK },
	/* 0x04a5		0x9001 */
	{ USB_VENDOR_ACERP,	USB_PRODUCT_ACERP_AWL400,
	  RadioRFMD,		ATUWI_NO_QUIRK },
#endif
#ifndef ATUWI_NO_RFMD2958
	/* 0x03eb		0x7613 */
	{ USB_VENDOR_ATMEL,	USB_PRODUCT_ATMEL_WL1130USB,
	  RadioRFMD2958,	ATUWI_NO_QUIRK },
	/* 0x1915		0x2233 */
	{ USB_VENDOR_LINKSYS3,	USB_PRODUCT_LINKSYS3_WUSB11v28,
	  RadioRFMD2958,	ATUWI_NO_QUIRK },
	/* 0x12fd		0x1001 */
	{ USB_VENDOR_AINCOMM,	USB_PRODUCT_AINCOMM_AWU2000B,
	  RadioRFMD2958,	ATUWI_NO_QUIRK },
#endif
#ifndef ATUWI_NO_RFMD2958_SMC
	/* SMC2662 V.4 */
	/* 0x03eb		0x7614 */
	{ USB_VENDOR_ATMEL,	USB_PRODUCT_ATMEL_AT76C505A,
	  RadioRFMD2958_SMC,	ATUWI_QUIRK_NO_REMAP | ATUWI_QUIRK_FW_DELAY },
#endif
#ifndef ATUWI_NO_INTERSIL
	/* 0x03eb		0x7603 */
	{ USB_VENDOR_ATMEL,	USB_PRODUCT_ATMEL_DWL120,
	  RadioIntersil,	ATUWI_NO_QUIRK },
	/* 0x04a5		0x9000 */
	{ USB_VENDOR_ACERP,	USB_PRODUCT_ACERP_AWL300,
	  RadioIntersil,	ATUWI_NO_QUIRK },
#endif
	{ 0, 0, 0, 0 }
};

Static int atuwi_match(device_ptr_t);
Static int atuwi_attach(device_ptr_t);
Static int atuwi_detach(device_ptr_t);
Static void atuwi_shutdown(device_ptr_t);
Static int atuwi_newbuf(struct atuwi_softc *, struct atuwi_chain *, struct mbuf *);
Static int atuwi_encap(struct atuwi_softc *sc, struct mbuf *m, struct atuwi_chain *c);
Static void atuwi_rxeof(usbd_xfer_handle, usbd_private_handle, usbd_status);
Static void atuwi_txeof(usbd_xfer_handle, usbd_private_handle, usbd_status);
Static void atuwi_start(struct ifnet *);
Static void atuwi_rxstart(struct ifnet *);
Static void atuwi_mgmt_loop(void *arg);
Static int atuwi_ioctl(struct ifnet *, u_long, caddr_t);
Static void atuwi_init(void *);
Static void atuwi_stop(struct atuwi_softc *);
Static void atuwi_watchdog(struct ifnet *);


Static device_method_t atuwi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, atuwi_match),
	DEVMETHOD(device_attach, atuwi_attach),
	DEVMETHOD(device_detach, atuwi_detach),
	DEVMETHOD(device_shutdown, atuwi_shutdown),
	{ 0, 0 }
};

Static driver_t atuwi_driver = {
	"atuwi",
	atuwi_methods,
	sizeof(struct atuwi_softc)
};

Static devclass_t atuwi_devclass;

DRIVER_MODULE(atuwi, uhub, atuwi_driver, atuwi_devclass, usbd_driver_load, 0);
MODULE_DEPEND(atuwi, usb, 1, 1, 1);
MODULE_DEPEND(atuwi, ether, 1, 1, 1);



Static void atuwi_msleep(struct atuwi_softc *sc, int ms)
{
	u_int8_t	dummy;
	int		ticks;

	usbd_delay_ms(sc->atuwi_udev, ms);
	return;

	ticks = ms * hz / 1000;
	if (ticks == 0)
		ticks = 1;

	tsleep(&dummy, PZERO | PCATCH, "atuwis", ms * hz / 1000);
}


static usbd_status atuwi_reset(struct atuwi_softc *sc)
{
	/* We don't need to actually send the device a reset... */
	/*
	usb_port_status_t	stat;

	usbd_reset_port(sc->atuwi_udev->myhub,
	    sc->atuwi_udev->powersrc->portno, &stat);
	*/

	sc->atuwi_udev->address = USB_START_ADDR;
	
	return(0);
}


Static usbd_status atuwi_usb_request(struct atuwi_softc *sc, u_int8_t type,
	 u_int8_t request, u_int16_t value, u_int16_t index, u_int16_t length,
	 u_int8_t *data)
{
	usb_device_request_t	req;
	usbd_xfer_handle        xfer;
	usbd_status             err;
	int			total_len = 0;

	req.bmRequestType = type;
	req.bRequest = request;
	USETW(req.wValue, value);
	USETW(req.wIndex, index);
	USETW(req.wLength, length);

#ifdef ATUWI_DEBUG_USB
	if (atuwi_debug_usb) {
		if ((data == NULL) || (type & UT_READ)) {
			DEBUG(FLAG_ALWAYS, ("atuwi%d: req=%02x val=%02x "
			    "ind=%02x len=%02x\n", sc->atuwi_unit, request,
			    value, index, length));
		} else {
			DEBUG(FLAG_ALWAYS, ("atuwi%d: req=%02x val=%02x "
			    "ind=%02x len=%02x [%8D]\n", sc->atuwi_unit,
			    request, value, index, length, data, " "));
		}
	}
#endif /* ATUWI_DEBUG_USB */

	ATUWI_LOCK(sc);

	xfer = usbd_alloc_xfer(sc->atuwi_udev);
	usbd_setup_default_xfer(xfer, sc->atuwi_udev, 0, 500000, &req, data,
	    length, USBD_SHORT_XFER_OK, 0);

	err = usbd_sync_transfer(xfer);

	usbd_get_xfer_status(xfer, NULL, NULL, &total_len, NULL);

#ifdef ATUWI_DEBUG_USB
	if (atuwi_debug_usb) {
		if (type & UT_READ) {
			DEBUG(FLAG_ALWAYS, ("atuwi%d: transfered 0x%x bytes"
			    " in\n", sc->atuwi_unit, total_len));
			DEBUG(FLAG_ALWAYS, ("atuwi%d: dump [%10D]\n",
			    sc->atuwi_unit, data, " "));
		} else {
			if (total_len != length)
				DEBUG(FLAG_ALWAYS, ("atuwi%d: ARG! wrote "
				    "only %x bytes\n", sc->atuwi_unit,
				    total_len));
		}
	}
#endif /* ATUWI_DEBUG_USB */

	usbd_free_xfer(xfer);
	
	ATUWI_UNLOCK(sc);
	
	return(err);
}


Static int atuwi_send_command(struct atuwi_softc *sc, u_int8_t *command,
	int size) {

	return atuwi_usb_request(sc, UT_WRITE_VENDOR_DEVICE, 0x0e, 0x0000,
	    0x0000, size, command);
}


Static int atuwi_get_cmd_status(struct atuwi_softc *sc, u_int8_t cmd,
	 u_int8_t *status) {

	/*
	 * all other drivers (including Windoze) request 40 bytes of status
	 * and get a short-xfer of just 6 bytes. we can save 34 bytes of
	 * buffer if we just request those 6 bytes in the first place :)
	 */
	/*
	return atuwi_usb_request(sc, UT_READ_VENDOR_INTERFACE, 0x22, cmd,
	    0x0000, 40, status);
	*/
	return atuwi_usb_request(sc, UT_READ_VENDOR_INTERFACE, 0x22, cmd,
	    0x0000, 6, status);
}


Static int atuwi_wait_completion(struct atuwi_softc *sc, u_int8_t cmd,
	u_int8_t *status) {

	int			err;
	u_int8_t		statusreq[6];
	int			idle_count = 0;
	

	DEBUG(FLAG_COMMANDS, ("atuwi%d: wait-completion: cmd=%02x\n",
	    sc->atuwi_unit, cmd));

	while (1) {
	
		err = atuwi_get_cmd_status(sc, cmd, statusreq);
		if (err)
			return err;

#ifdef ATUWI_DEBUG_USB
		if (atuwi_debug_usb) {
			DEBUG(FLAG_ALWAYS, ("atuwi%d: status=%6D cmd=%02x\n",
			    sc->atuwi_unit, statusreq, " ", cmd));
		}
#endif /* ATUWI_DEBUG_USB */

		/*
		 * during normal operations waiting on STATUS_IDLE
		 * will never happen more than once
		 */
		if ((statusreq[5] == STATUS_IDLE) && (idle_count++ > 20)) {
			ERROR(("atuwi%d: AAARRGGG!!! FIX ME!\n",
			    sc->atuwi_unit));
			return 0;
		}
		
		if ((statusreq[5] != STATUS_IN_PROGRESS) &&
		    (statusreq[5] != STATUS_IDLE)) {
		    	if (status != NULL)
		    		*status = statusreq[5];
		    	return 0;
		}
		
		atuwi_msleep(sc, 25);
	}
}


Static int atuwi_send_mib(struct atuwi_softc *sc, u_int8_t type,
	u_int8_t size, u_int8_t index, void *data)
{

	int				err;
	struct atuwi_cmd_set_mib	request;

	/*
	 * We don't construct a MIB packet first and then memcpy it into an
	 * Atmel-command-packet, we just construct it the right way at once :)
	 */

	request.AtCmd = CMD_SET_MIB;
	request.AtReserved = 0;
	request.AtSize = size + 4;

	request.MIBType = type;
	request.MIBSize = size;
	request.MIBIndex = index;
	request.MIBReserved = 0;
	
	/*
	 * For 1 and 2 byte requests we assume a direct value,
	 * everything bigger than 2 bytes we assume a pointer to the data
	 */
	switch (size) {	
	case 0:
		break;
	case 1:
		request.data[0]=(u_int32_t)data & 0x000000ff;
		break;
	case 2:
		request.data[0]=(u_int32_t)data & 0x000000ff;
		request.data[1]=(u_int32_t)data >> 8;
		break;
	default:
		memcpy(request.data, data, size);
	}
	
	err = atuwi_usb_request(sc, UT_WRITE_VENDOR_DEVICE, 0x0e, 0x0000,
		0x0000, request.AtSize+4, (u_int8_t *)&request);
	if (err)
		return err;

	DEBUG(FLAG_COMMANDS, ("atuwi%d: sendmib : waitcompletion...\n",
	    sc->atuwi_unit));

	return atuwi_wait_completion(sc, CMD_SET_MIB, NULL);
}


Static int atuwi_get_mib(struct atuwi_softc *sc, u_int8_t type,
	u_int8_t size, u_int8_t index, u_int8_t *buf)
{

	/* linux/at76c503.c - 478 */
	return atuwi_usb_request(sc, UT_READ_VENDOR_INTERFACE, 0x033,
		type << 8, index, size, buf);
}


Static int atuwi_start_ibss(struct atuwi_softc *sc)
{
	int				err;
	struct atuwi_cmd_start_ibss	Request;

	Request.Cmd = CMD_START_IBSS;
	Request.Reserved = 0;
	Request.Size = sizeof(Request) - 4;

	memset(Request.BSSID, 0x00, sizeof(Request.BSSID));
	memset(Request.SSID, 0x00, sizeof(Request.SSID));
	memcpy(Request.SSID, sc->atuwi_ssid, sc->atuwi_ssidlen);
	Request.SSIDSize = sc->atuwi_ssidlen;
	Request.Channel = sc->atuwi_channel;
	Request.BSSType = AD_HOC_MODE;
	memset(Request.Res, 0x00, sizeof(Request.Res));

	/* Write config to adapter */
	err = atuwi_send_command(sc, (u_int8_t *)&Request, sizeof(Request));
	if (err) {
		ERROR(("atuwi%d: start ibss failed!\n", sc->atuwi_unit));
		return err;
	}
	
	/* Wait for the adapter to do it's thing */
	err = atuwi_wait_completion(sc, CMD_START_IBSS, NULL);
	if (err) {
		ERROR(("atuwi%d: error waiting for start_ibss\n",
		    sc->atuwi_unit));
		return err;
	}

	/* Get the current BSSID */
	err = atuwi_get_mib(sc, MIB_MAC_MGMT__CURRENT_BSSID, sc->atuwi_bssid);
	if (err) {
		ERROR(("atuwi%d: could not get BSSID!\n", sc->atuwi_unit));
		return err;
	}
	
	DEBUG(FLAG_STATE, ("atuwi%d: started a new IBSS (BSSID=%6D)\n",
		sc->atuwi_unit, sc->atuwi_bssid, ":"));
	
	return 0;
}


Static int atuwi_start_scan(struct atuwi_softc *sc)
{
	struct atuwi_cmd_do_scan	Scan;
	usbd_status			err;
	int				Cnt;
	
	Scan.Cmd = CMD_START_SCAN;
	Scan.Reserved = 0;
	Scan.Size = sizeof(Scan) - 4;
	
	/* use the broadcast BSSID (in active scan) */
	for (Cnt=0; Cnt<6; Cnt++)
		Scan.BSSID[Cnt] = 0xff;
	
	memset(Scan.SSID, 0x00, sizeof(Scan.SSID));
	memcpy(Scan.SSID, sc->atuwi_ssid, sc->atuwi_ssidlen);
	Scan.SSID_Len = sc->atuwi_ssidlen;
	Scan.SSID_Len = 0;

	/* default values for scan */	
	Scan.ScanType = ATUWI_SCAN_ACTIVE;
	Scan.Channel = sc->atuwi_channel;
	Scan.ProbeDelay = 3550;
	Scan.MinChannelTime = 250;
	Scan.MaxChannelTime = 3550;
	
	/* we like scans to be quick :) */
	/* the time we wait before sending probe's */
	Scan.ProbeDelay = 0;
	/* the time we stay on one channel */
	Scan.MinChannelTime = 100;
	Scan.MaxChannelTime = 200;
	/* wether or not we scan all channels */
	Scan.InternationalScan = 0xc1;

#ifdef ATUWI_DEBUG_USB
	if (atuwi_debug_usb) {
		DEBUG(FLAG_ALWAYS, ("atuwi%d: scan cmd len=%02x\n",
		    sc->atuwi_unit, sizeof(Scan)));
		DEBUG(FLAG_ALWAYS, ("atuwi%d: scan cmd: %52D\n",
		    sc->atuwi_unit, (u_int8_t *)&Scan, " "));
	}
#endif /* ATUWI_DEBUG_USB */

	/* Write config to adapter */
	err = atuwi_send_command(sc, (u_int8_t *)&Scan, sizeof(Scan));
	if (err)
		return err;
	
	/*
	 * We don't wait for the command to finish... the mgmt-thread will do
	 * that for us
	 */
	/*
	err = atuwi_wait_completion(sc, CMD_START_SCAN, NULL);
	if (err)
		return err;
	*/
	
	return 0;	
}


Static int atuwi_switch_radio(struct atuwi_softc *sc, int state)
{
	usbd_status		err;
	struct atuwi_cmd	CmdRadio = {CMD_RADIO_ON, 0, 0};
	
	if (sc->atuwi_radio == RadioIntersil) {
		/*
		 * Intersil doesn't seem to need/support switching the radio
		 * on/off
		 */
		return 0;
	}
	
	if (sc->atuwi_radio_on != state) {
		
		if (state == 0)
			CmdRadio.Cmd = CMD_RADIO_OFF;
		
		err = atuwi_send_command(sc, (u_int8_t *)&CmdRadio,
			sizeof(CmdRadio));
		if (err)
			return err;
		
		err = atuwi_wait_completion(sc, CmdRadio.Cmd, NULL);
		if (err)
			return err;
		
		DEBUG(FLAG_COMMANDS, ("atuwi%d: radio turned %s\n",
		    sc->atuwi_unit, state ? "on" : "off"));
		sc->atuwi_radio_on = state;
	}
	return 0;
}


Static int atuwi_initial_config(struct atuwi_softc *sc)
{
	usbd_status			err;
/*	u_int8_t			rates[4] = {0x82, 0x84, 0x8B, 0x96};*/
	u_int8_t			rates[4] = {0x82, 0x04, 0x0B, 0x16};
	struct atuwi_cmd_card_config	cmd;
	u_int8_t			reg_domain;


	DEBUG(FLAG_ALWAYS, ("atuwi%d: sending mac-addr\n", sc->atuwi_unit));
	err = atuwi_send_mib(sc, MIB_MAC_ADDR__ADDR, sc->atuwi_mac_addr);
	if (err) {
		ERROR(("atuwi%d: error setting mac-addr\n", sc->atuwi_unit));
		return err;
	}
	
	/*
	DEBUG(FLAG_ALWAYS, ("atuwi%d: sending reg-domain\n", sc->atuwi_unit));
	err = atuwi_send_mib(sc, MIB_PHY__REG_DOMAIN, NR(0x30));
	if (err) {
		ERROR(("atuwi%d: error setting mac-addr\n", sc->atuwi_unit));
		return err;
	}
	*/
	
	memset(&cmd, 0, sizeof(cmd));
	cmd.Cmd = CMD_STARTUP;
	cmd.Reserved = 0;
	cmd.Size = sizeof(cmd) - 4;

	cmd.Channel = sc->atuwi_channel;
	cmd.AutoRateFallback = 1;
	memcpy(cmd.BasicRateSet, rates, 4);

	/* ShortRetryLimit should be 7 according to 802.11 spec */
	cmd.ShortRetryLimit = 7;
	cmd.RTS_Threshold = 2347;
	cmd.FragThreshold = 2346;

	/* Doesn't seem to work, but we'll set it to 1 anyway */
	cmd.PromiscuousMode = 1;

	/* this goes into the beacon we transmit */
	if (sc->atuwi_encrypt == ATUWI_WEP_OFF)
		cmd.PrivacyInvoked = 0;
	else
		cmd.PrivacyInvoked = 1;
		
	cmd.ExcludeUnencrypted = 0;
	cmd.EncryptionType = sc->atuwi_wepkeylen;
	
	/* Setting the SSID here doesn't seem to do anything */
	memset(cmd.SSID, 0, sizeof(cmd.SSID));
	memcpy(cmd.SSID, sc->atuwi_ssid, sc->atuwi_ssidlen);
	cmd.SSID_Len = sc->atuwi_ssidlen;
	
	cmd.WEP_DefaultKeyID = sc->atuwi_wepkey;
	memcpy(cmd.WEP_DefaultKey, sc->atuwi_wepkeys,
	    sizeof(cmd.WEP_DefaultKey));
	
	cmd.ShortPreamble = 1;
	cmd.ShortPreamble = 0;
	cmd.BeaconPeriod = 100;
	/* cmd.BeaconPeriod = 65535; */

	/*
	 * TODO:
	 * read reg domain MIB_PHY @ 0x17 (1 byte), (reply = 0x30)
	 * we should do something usefull with this info. right now it's just
	 * ignored
	 */
	err = atuwi_get_mib(sc, MIB_PHY__REG_DOMAIN, &reg_domain);
	if (err) {
		DEBUG(FLAG_INIT, ("atuwi%d: could not get regdomain!\n",
		    sc->atuwi_unit));
	}
	
#ifdef ATUWI_DEBUG_USB
	if (atuwi_debug_usb) {
		DEBUG(FLAG_COMMANDS, ("atuwi%d: configlen=%02x\n",
		    sc->atuwi_unit, sizeof(cmd)));
		DEBUG(FLAG_COMMANDS, ("atuwi%d: configdata= %108D\n",
		    sc->atuwi_unit, (u_int8_t *)&cmd, " "));
	}
#endif /* ATUWI_DEBUG_USB */

	/* Windoze : driver says exclude-unencrypted=1 & encr-type=1 */
	
	err = atuwi_send_command(sc, (u_int8_t *)&cmd, sizeof(cmd));
	if (err)
		return err;
	err = atuwi_wait_completion(sc, CMD_STARTUP, NULL);
	if (err)
		return err;

	/* Turn on radio now */
	err = atuwi_switch_radio(sc, 1);
	if (err)
		return err;
	
	/* preamble type = short */
	err = atuwi_send_mib(sc, MIB_LOCAL__PREAMBLE, NR(PREAMBLE_SHORT));
	if (err)
		return err;

	/* frag = 1536 */	
	err = atuwi_send_mib(sc, MIB_MAC__FRAG, NR(2346));
	if (err)
		return err;
	
	/* rts = 1536 */
	err = atuwi_send_mib(sc, MIB_MAC__RTS, NR(2347));
	if (err)
		return err;

	/* auto rate fallback = 1 */
	err = atuwi_send_mib(sc, MIB_LOCAL__AUTO_RATE_FALLBACK, NR(1));
	if (err)
		return err;

	/* power mode = full on, no power saving */
	err = atuwi_send_mib(sc, MIB_MAC_MGMT__POWER_MODE,
	    NR(POWER_MODE_ACTIVE));
	if (err)
		return err;

	DEBUG(FLAG_INIT, ("atuwi%d: completed initial config\n",
	    sc->atuwi_unit));
	
	return 0;
}


Static int atuwi_join(struct atuwi_softc *sc)
{
	struct atuwi_cmd_join		join;
	u_int8_t			status;
	usbd_status			err;
	
	
	join.Cmd = CMD_JOIN;
	join.Reserved = 0x00;
	join.Size = sizeof(join) - 4;	
	
	DEBUG(FLAG_MGMT, ("atuwi%d: pre-join sc->atuwi_bssid=%6D\n",
	    sc->atuwi_unit, sc->atuwi_bssid, ":"));
	DEBUG(FLAG_MGMT, ("atuwi%d: mode=%d\n", sc->atuwi_unit,
	    sc->atuwi_mode));
	memcpy(join.bssid, sc->atuwi_bssid, ETHER_ADDR_LEN);	
	memset(join.essid, 0x00, 32);
	memcpy(join.essid, sc->atuwi_ssid, sc->atuwi_ssidlen);
	join.essid_size = sc->atuwi_ssidlen;
	join.bss_type = sc->atuwi_mode;
	join.channel = sc->atuwi_channel;
	
	join.timeout = ATUWI_JOIN_TIMEOUT;
	join.reserved = 0x00;

	DEBUG(FLAG_MGMT, ("atuwi%d: trying to join BSSID=%6D\n",
	    sc->atuwi_unit, join.bssid, ":"));
	err = atuwi_send_command(sc, (u_int8_t *)&join, sizeof(join));
	if (err) {
		DEBUG(FLAG_MGMT, ("atuwi%d: ERROR trying to join IBSS\n",
		    sc->atuwi_unit));
		return err;
	}
	err = atuwi_wait_completion(sc, CMD_JOIN, &status);
	if (err) {
		DEBUG(FLAG_MGMT, ("atuwi%d: error joining BSS!\n",
		    sc->atuwi_unit));
		return err;
	}
	if (status != STATUS_COMPLETE) {
		DEBUG(FLAG_MGMT, ("atuwi%d: error joining... [status=%02x]\n",
		    sc->atuwi_unit, status));
		return status;
	} else {
		DEBUG(FLAG_MGMT, ("atuwi%d: joined BSS\n", sc->atuwi_unit));
	}

	return err;
}


Static void
atuwi_airo_tap(struct atuwi_softc *sc, u_int8_t *pkt, u_int length,
    struct at76c503_rx_buffer *at_hdr)
{
	struct mbuf		*airo_pkt;
	struct an_rxframe	*airo_hdr;
	struct wi_80211_hdr	*wi_hdr;
	struct wi_80211_hdr	*wi_hdr_dst;
	int			wi_hdr_len;
	u_int8_t		*payload_ptr;

	
	MGETHDR(airo_pkt, M_DONTWAIT, MT_DATA);
	if (airo_pkt == NULL) {
		ERROR(("atuwi%d: airo_tap: could not get an mbuf!\n",
		    sc->atuwi_unit));
		return;
	}
	
	MCLGET(airo_pkt, M_DONTWAIT);
	if (!(airo_pkt->m_flags & M_EXT)) {
		ERROR(("atuwi%d: airo_tap: could not get an mbuf!\n",
		    sc->atuwi_unit));
		m_freem(airo_pkt);
		return;
	}

	airo_pkt->m_len = airo_pkt->m_pkthdr.len =
	    MCLBYTES;
	
	airo_hdr = mtod(airo_pkt, struct an_rxframe *);
	wi_hdr_dst = (struct wi_80211_hdr *)&airo_hdr->an_frame_ctl;
	wi_hdr = (struct wi_80211_hdr *)pkt;
	payload_ptr = mtod(airo_pkt, u_int8_t *) + sizeof(struct an_rxframe);
	
	bzero(airo_hdr, sizeof(struct an_rxframe));

	if (at_hdr != NULL) {
		/* It's a received packet : fill in receiver info */
		airo_hdr->an_rx_time = at_hdr->rx_time;
		airo_hdr->an_rx_signal_strength = at_hdr->rssi;
		airo_hdr->an_rx_rate = at_hdr->rx_rate;
		
		/* this is inaccurate when scanning! */
		airo_hdr->an_rx_chan = sc->atuwi_channel;
		
	} else {
		/* It's a transmitted packet : make up as much as we can */
		airo_hdr->an_rx_time = 0;
		airo_hdr->an_rx_signal_strength = 0xff;
		airo_hdr->an_rx_rate = 4;
		
		airo_hdr->an_rx_chan = sc->atuwi_channel;
	}
	
	if ((wi_hdr->frame_ctl & WI_FCTL_FTYPE) == WI_FTYPE_DATA) {
		wi_hdr_len = sizeof(struct wi_80211_hdr);
	} else {
		wi_hdr_len = sizeof(struct wi_mgmt_hdr);
	}
	
	airo_hdr->an_rx_payload_len = length - wi_hdr_len;
	
	bcopy(wi_hdr, wi_hdr_dst, wi_hdr_len);
	airo_hdr->an_gaplen = 0;

	m_copyback(airo_pkt, sizeof(struct an_rxframe), length - wi_hdr_len,
	    pkt + wi_hdr_len);

	airo_pkt->m_pkthdr.rcvif = (struct ifnet *)&sc->atuwi_qdat;
	airo_pkt->m_pkthdr.len = airo_pkt->m_len = length - wi_hdr_len +
	    sizeof(struct an_rxframe);
	
	bpf_mtap(sc->atuwi_airobpf, airo_pkt);
	m_free(airo_pkt);
}


Static int
atuwi_send_packet(struct atuwi_softc *sc, struct atuwi_chain *c)
{
	usbd_status		err;
	struct atuwi_txpkt	*pkt;
	
	/* Don't try to send when we're shutting down the driver */
	if (sc->atuwi_dying)
		return(EIO);

	/* drop the raw 802.11b packets to bpf */
	if (sc->atuwi_rawbpf) {
		DEBUG(FLAG_BPF, ("atuwi%d: raw bpf tap on TX :)\n",
		    sc->atuwi_unit));
		bpf_tap(sc->atuwi_rawbpf, c->atuwi_buf, c->atuwi_length);
	}
	
	/* drop the raw 802.11b packets to bpf with aironet header */
	if (sc->atuwi_airobpf) {
	
		DEBUG(FLAG_BPF, ("atuwi%d: aironet bpf tap on TX :)\n",
		    sc->atuwi_unit));

		pkt = (struct atuwi_txpkt *)c->atuwi_buf;

		atuwi_airo_tap(sc, (u_int8_t *)&pkt->WiHeader,
		    c->atuwi_length - sizeof(struct at76c503_tx_buffer),
		    NULL);
	}

#ifdef ATUWI_PROFILING
	atuwi_profiling_packets_out++;
	atuwi_profiling_bytes_out += c->atuwi_length;
#endif /* ATUWI_PROFILING */

#ifdef ATUWI_NO_COPY_TX
	usbd_setup_xfer(c->atuwi_xfer, sc->atuwi_ep[ATUWI_ENDPT_TX],
	    c, c->atuwi_buf, c->atuwi_length, USBD_NO_COPY, ATUWI_TX_TIMEOUT,
	    atuwi_txeof);
#else /* ATUWI_NO_COPY_TX */
	usbd_setup_xfer(c->atuwi_xfer, sc->atuwi_ep[ATUWI_ENDPT_TX],
	    c, c->atuwi_buf, c->atuwi_length, 0, ATUWI_TX_TIMEOUT, atuwi_txeof);
#endif /* ATUWI_NO_COPY_TX */

#ifdef ATUWI_LAZY_TX_NETISR
	c->atuwi_not_freed = 0;
#endif /* ATUWI_LAZY_TX_NETISR */

	/* Let's get this thing into the air! */
	c->atuwi_in_xfer = 1;
	err = usbd_transfer(c->atuwi_xfer);
	if (err != USBD_IN_PROGRESS) {
		atuwi_stop(sc);
		return(EIO);
	}
	
	DEBUG(FLAG_TX, ("atuwi%d: tx packet...\n", sc->atuwi_unit));
	
	return 0;
}


Static int atuwi_send_mgmt_packet(struct atuwi_softc *sc,
    struct atuwi_chain *c, u_int16_t length)
{
	struct atuwi_mgmt_packet	*packet;
	
	packet = (struct atuwi_mgmt_packet *)c->atuwi_buf;
	
	packet->athdr.wlength = length - sizeof(packet->athdr);
	packet->athdr.tx_rate = 4;
	packet->athdr.padding = 0;
	memset(packet->athdr.reserved, 0x00, 4);
	
	packet->mgmt_hdr.duration = 0x8000;
	memcpy(packet->mgmt_hdr.dst_addr, sc->atuwi_bssid, ETHER_ADDR_LEN);
	memcpy(packet->mgmt_hdr.src_addr, sc->atuwi_mac_addr, ETHER_ADDR_LEN);
	memcpy(packet->mgmt_hdr.bssid, sc->atuwi_bssid, ETHER_ADDR_LEN);
	packet->mgmt_hdr.seq_ctl = 0;
	
	c->atuwi_length = length;
	
	return atuwi_send_packet(sc, c);
}


Static int atuwi_authenticate(struct atuwi_softc *sc)
{
	usbd_status			err;
	struct atuwi_chain		*ch;
	struct atuwi_auth_packet	*packet;
	
	/*
	 * now we should authenticate :
	 *  7.2.3.10 - page 64 of 802.11b spec
	 *  8.1 - page 74 of 802.11b spec
	 *  see 7.3.1.9 - page 69 for status codes
	 *
	 * open systems :
	 *  send: seq_nr=1	auth req
	 *  recv: seq_nr=2	auth resp. (with status code)
	 *
	 * shared key systems :
	 *  send: seq_nr=1	auth req
	 *  recv: seq_nr=2	auth challenge (with status code & challenge
	 *                        text)
	 *  send: seq_nr=3	auth reponse (wep encr challenge text)
	 *  recv: seq_nr=4	auth result
	 *
	 * algorithm number :
	 *  0 = open
	 *  1 = shared
	 */
	
	ch = SLIST_FIRST(&sc->atuwi_cdata.atuwi_mgmt_free);
	if (ch == NULL) {
		ERROR(("atuwi%d: authenticate: no mgmt transfers available\n",
		    sc->atuwi_unit));
		return ENOMEM;
	}
	SLIST_REMOVE_HEAD(&sc->atuwi_cdata.atuwi_mgmt_free, atuwi_list);

	packet = (struct atuwi_auth_packet *)ch->atuwi_buf;

	packet->mgmt_hdr.frame_ctl = WI_FTYPE_MGMT |
	    IEEE80211_FC0_SUBTYPE_AUTH;

	packet->auth_hdr.wi_algo = 0;
	packet->auth_hdr.wi_seq = 1;
	packet->auth_hdr.wi_status = 0;

	
	DEBUG(FLAG_MGMTFULL, ("atuwi%d: auth packet: %30D\n", sc->atuwi_unit,
		((u_int8_t *)packet)+8, " "));
	
	err = atuwi_send_mgmt_packet(sc, ch, sizeof(*packet));
	if (err) {
		ERROR(("atuwi%d: could not send auth packet\n",
		    sc->atuwi_unit));
	}

	/*
	 * TODO: implement shared key auth
	 */
	/*
	packet->algoritm = 1;
	packet->sequence = 3;
	packet->status = 0;

	memcpy(packet->challenge, the_challenge_text, the_challenge_length);

	DEBUG(FLAG_MGMTFULL, ("atuwi%d: auth packet: %30D\n", sc->atuwi_unit,
		((u_int8_t *)packet)+8, " "));

	if (sc->atuwi_encrypt & ATUWI_WEP_TX) {
		packet->mgmt_hdr.frame_ctl |= WI_FCTL_WEP;
		DEBUG(FLAG_TX, ("atuwi%d: ==> WEP on please\n",
		    sc->atuwi_unit));
	}
	
	err = atuwi_send_mgmt_packet(sc, ch, sizeof(*packet) + challenge_len);
	if (err) {
		ERROR(("atuwi%d: could not send auth packet 2\n",
		    sc->atuwi_unit));
	}
	*/
	
	return 0;
}
	

Static int atuwi_associate(struct atuwi_softc *sc)
{
	usbd_status			err;
	struct atuwi_chain		*ch;
	u_int8_t			*ptr;
	struct atuwi_assoc_packet	*packet;
	
	/*
	 * associate :
	 *  7.2.3.4 - page 62 of 802.11b spec
	 *
	 */
	
	ch = SLIST_FIRST(&sc->atuwi_cdata.atuwi_mgmt_free);
	if (ch == NULL) {
		ERROR(("atuwi%d: associate: no mgmt transfers left\n",
		    sc->atuwi_unit));
		return ENOMEM;
	}
	SLIST_REMOVE_HEAD(&sc->atuwi_cdata.atuwi_mgmt_free, atuwi_list);

	packet = (struct atuwi_assoc_packet *)ch->atuwi_buf;

	packet->mgmt_hdr.frame_ctl = WI_FTYPE_MGMT |
	    IEEE80211_FC0_SUBTYPE_ASSOC_REQ;
	
	packet->capability = 1 + 32;	/* ess & short preamble */
	packet->capability = 1;
	packet->listen_interval = 100;	/* beacon interval */
	
	ptr = packet->data;
	*ptr++ = WI_VAR_SSID;		/* SSID */
	*ptr++ = sc->atuwi_ssidlen;
	memcpy(ptr, sc->atuwi_ssid, sc->atuwi_ssidlen);
	ptr += sc->atuwi_ssidlen;
	
	*ptr++ = WI_VAR_SRATES;		/* supported rates */
	*ptr++ = 0x04;
	*ptr++ = 0x82;
	*ptr++ = 0x84;
	*ptr++ = 0x8b;
	*ptr++ = 0x96;
	
	DEBUG(FLAG_MGMTFULL, ("atuwi%d: associate packet: %50D\n",
	    sc->atuwi_unit, (u_int8_t *)packet, " "));
	
	err = atuwi_send_mgmt_packet(sc, ch, sizeof(*packet) + 2 +
	    sc->atuwi_ssidlen + 6);
	if (err) {
		ERROR(("atuwi%d: could not send associate packet\n",
		    sc->atuwi_unit));
	}
	
	return 0;
}


/*
 * Get the state of the DFU unit
 */
Static int8_t atuwi_get_dfu_state(struct atuwi_softc *sc)
{
	u_int8_t	state;
	
	if (atuwi_usb_request(sc, DFU_GETSTATE, 0, 0, 1, &state))
		return -1;

	return state;
}


/*
 * Get MAC opmode
 */
Static u_int8_t atuwi_get_opmode(struct atuwi_softc *sc, u_int8_t *mode)
{

	return atuwi_usb_request(sc, UT_READ_VENDOR_INTERFACE, 0x33, 0x0001,
	    0x0000, 1, mode);
}


/*
 * Upload the internal firmware into the device
 */
Static int atuwi_upload_internal_firmware(struct atuwi_softc *sc)
{
	int8_t			state;
	int			bytes_left = 0;
	u_int8_t		*ptr = NULL;
	int			block_size;
	int			block = 0;
	u_int8_t		status[6];
	int			err;

	/*
	 * Uploading firmware is done with the DFU (Device Firmware Upgrade)
	 * interface. See "Universal Serial Bus - Device Class Specification
	 * for Device Firmware Upgrade" pdf for details of the protocol.
	 * Maybe this could be moved to a seperate 'firmware driver' once more
	 * device drivers need it... For now we'll just do it here.
	 *
	 * Just for your information, the Atmel's DFU descriptor looks like
	 * this:
	 *
	 * 07		size
	 * 21		type
	 * 01		capabilities : only firmware download, need reset
	 *		  after download
	 * 13 05	detach timeout : max 1299ms between DFU_DETACH and
	 *		  reset
	 * 00 04	max bytes of firmware per transaction : 1024
	 */

	/* Choose the right firmware for the device */	
	switch (sc->atuwi_radio) {
#ifndef ATUWI_NO_RFMD
	case RadioRFMD:
		ptr = atuwi_fw_rfmd_int;
		bytes_left = sizeof(atuwi_fw_rfmd_int);
		DEBUG(FLAG_FW, ("atuwi%d: loading RFMD firmware...\n",
		    sc->atuwi_unit));
		break;
#endif
#ifndef ATUWI_NO_RFMD2958
	case RadioRFMD2958:
		ptr = atuwi_fw_rfmd2958_int;
		bytes_left = sizeof(atuwi_fw_rfmd2958_int);
		DEBUG(FLAG_FW, ("atuwi%d: loading RFMD2958 firmware...\n",
		    sc->atuwi_unit));
		break;
#endif
#ifndef ATUWI_NO_RFMD2958_SMC
	case RadioRFMD2958_SMC:
		ptr = atuwi_fw_rfmd2958_smc_int;
		bytes_left = sizeof(atuwi_fw_rfmd2958_smc_int);
		DEBUG(FLAG_FW, ("atuwi%d: loading RFMD2958-smc firmware...\n",
		    sc->atuwi_unit));
		break;
#endif
#ifndef ATUWI_NO_INTERSIL
	case RadioIntersil:
		ptr = atuwi_fw_intersil_int;
		bytes_left = sizeof(atuwi_fw_intersil_int);
		DEBUG(FLAG_FW, ("atuwi%d: loading Intersil firmware...\n",
		    sc->atuwi_unit));
		break;
#endif
	default:
		ERROR(("atuwi%d: unknown device type?\n", sc->atuwi_unit));
		bytes_left = 0;
		break;
	}

	state = atuwi_get_dfu_state(sc);
	
	while (bytes_left >= 0 && state > 0) {
	
		switch (state) {
		case DFUState_DnLoadSync:
			/* get DFU status */
			err = atuwi_usb_request(sc, DFU_GETSTATUS, 0, 0 , 6,
			    status);
			if (err) {
				ERROR(("atuwi%d: dfu_getstatus failed!\n",
				    sc->atuwi_unit));
				return err;
			}
			/* success means state => DnLoadIdle */
			state = DFUState_DnLoadIdle;
			continue;
			break;
		
		case DFUState_DFUIdle:
		case DFUState_DnLoadIdle:
			if (bytes_left>=DFU_MaxBlockSize)
				block_size = DFU_MaxBlockSize;
			else
				block_size = bytes_left;
			DEBUG(FLAG_FWFULL, ("atuwi%d: firmware block %d\n",
				sc->atuwi_unit, block));
		
			err = atuwi_usb_request(sc, DFU_DNLOAD, block++, 0,
			    block_size, ptr);
			if (err) {
				ERROR(("atuwi%d: dfu_dnload failed\n",
				    sc->atuwi_unit));
				return err;
			}
			
			ptr += block_size;
			bytes_left -= block_size;
			if (block_size == 0)
				bytes_left = -1;
			break;
			
		default:
			atuwi_msleep(sc, 100);
			DEBUG(FLAG_FWFULL, ("atuwi%d: sleeping for a while\n",
				sc->atuwi_unit));
		}
		
		state = atuwi_get_dfu_state(sc);
	}

	if (state != DFUState_ManifestSync) {
		ERROR(("atuwi%d: state != manifestsync... eek!\n",
		    sc->atuwi_unit));
	}

	err = atuwi_usb_request(sc, DFU_GETSTATUS, 0, 0, 6, status);
	if (err) {
		ERROR(("atuwi%d: dfu_getstatus failed!\n", sc->atuwi_unit));
		return err;
	}

	DEBUG(FLAG_FWFULL, ("atuwi%d: sending remap\n", sc->atuwi_unit));
	err = atuwi_usb_request(sc, DFU_REMAP, 0, 0, 0, NULL);
	if ((err) && (! sc->atuwi_quirk & ATUWI_QUIRK_NO_REMAP)) {
		ERROR(("atuwi%d: remap failed!\n", sc->atuwi_unit));
		return err;
	}
	
	/* after a lot of trying and measuring I found out the device needs
	 * about 56 miliseconds after sending the remap command before
	 * it's ready to communicate again. So we'll wait just a little bit
	 * longer than that to be sure...
	 */
	atuwi_msleep(sc, 56+100);

	/* reset the device to get the firmware to boot */
	DEBUG(FLAG_FW, ("atuwi%d: trying to reset device...\n",
	    sc->atuwi_unit));
	err = atuwi_reset(sc);
	if (err) {
		ERROR(("atuwi%d: reset failed...\n", sc->atuwi_unit));
		return err;
	}
	
	DEBUG(FLAG_FW, ("atuwi%d: internal firmware upload done\n",
	    sc->atuwi_unit));
	
	return 0;
}


Static int atuwi_upload_external_firmware(struct atuwi_softc *sc) {
	u_int8_t		*ptr = NULL;
	int			bytes_left = 0;
	int			block_size;
	int			block = 0;
	u_int8_t		mode;
	u_int8_t		channel;
	int			err;
		

	err = atuwi_get_opmode(sc, &mode);
	if (err) {
		ERROR(("atuwi%d: could not get opmode\n", sc->atuwi_unit));
		return err;
	}
	DEBUG(FLAG_FWFULL, ("atuwi%d: opmode: %d\n", sc->atuwi_unit, mode));


	if (mode == MODE_NETCARD) {
		DEBUG(FLAG_FW, ("atuwi%d: device doesn't need external "
		    "firmware\n", sc->atuwi_unit));
		return 0;
	}
	if (mode != MODE_NOFLASHNETCARD) {
		ERROR(("atuwi%d: EEK! unexpected opmode=%d\n", sc->atuwi_unit,
		    mode));
	}

	/*
	 * There is no difference in opmode before and after external firmware
	 * upload with the SMC2662 V.4 . So instead we'll try to read the
	 * channel number. If we succeed, external firmware must have been
	 * already uploaded...
	 */
	err = atuwi_get_mib(sc, MIB_PHY__CHANNEL, &channel);
	if (! err) {
		DEBUG(FLAG_FW, ("atuwi%d: external firmware has already been "
		    "downloaded\n", sc->atuwi_unit));
		return 0;
	}

	switch (sc->atuwi_radio) {
#ifndef ATUWI_NO_RFMD
	case RadioRFMD:
		ptr = atuwi_fw_rfmd_ext;
		bytes_left = sizeof(atuwi_fw_rfmd_ext);
		DEBUG(FLAG_FW, ("atuwi%d: loading external RFMD firmware\n",
		    sc->atuwi_unit));
		break;
#endif
#ifndef ATUWI_NO_RFMD2958
	case RadioRFMD2958:
		ptr = atuwi_fw_rfmd2958_ext;
		bytes_left = sizeof(atuwi_fw_rfmd2958_ext);
		DEBUG(FLAG_FW, ("atuwi%d: loading external RFMD2958 "
		    "firmware\n", sc->atuwi_unit));
		break;
#endif
#ifndef ATUWI_NO_RFMD2958_SMC
	case RadioRFMD2958_SMC:
		ptr = atuwi_fw_rfmd2958_smc_ext;
		bytes_left = sizeof(atuwi_fw_rfmd2958_smc_ext);
		DEBUG(FLAG_FW, ("atuwi%d: loading external RFMD2958-smc "
		    "firmware\n", sc->atuwi_unit));
		break;
#endif
#ifndef ATUWI_NO_INTERSIL
	case RadioIntersil:
		ptr = atuwi_fw_intersil_ext;
		bytes_left = sizeof(atuwi_fw_intersil_ext);
		DEBUG(FLAG_FW, ("atuwi%d: loading external Intersil "
		    "firmware\n", sc->atuwi_unit));
		break;
#endif
	default:
		ERROR(("atuwi%d: unknown device type?\n", sc->atuwi_unit));
		bytes_left = 0;
		break;
	}

	while (bytes_left) {
		if (bytes_left > 1024)
			block_size = 1024;
		else
			block_size = bytes_left;
		
		DEBUG(FLAG_FWFULL, ("atuwi%d: block:%d size:%d\n",
		    sc->atuwi_unit, block, block_size));
		err = atuwi_usb_request(sc, UT_WRITE_VENDOR_DEVICE, 0x0e,
		    0x0802, block, block_size, ptr);
		if (err) {
			ERROR(("atuwi%d: could not load external firmware "
			    "block\n", sc->atuwi_unit));
			return err;
		}
		
		ptr += block_size;
		block++;
		bytes_left -= block_size;
	}
	
	err = atuwi_usb_request(sc, UT_WRITE_VENDOR_DEVICE, 0x0e, 0x0802,
	    block, 0, NULL);
	if (err) {
		ERROR(("atuwi%d: could not load last zero-length firmware block\n",
		    sc->atuwi_unit));
		return err;
	}
	
	/*
	 * The SMC2662w V.4 seems to require some time to do it's thing with
	 * the external firmware... 20 ms isn't enough, but 21 ms works 100
	 * times out of 100 tries. We'll wait a bit longer just to be sure
	 */
	if (sc->atuwi_quirk & ATUWI_QUIRK_FW_DELAY) {
		atuwi_msleep(sc, 21 + 100);
	}
	
	DEBUG(FLAG_FW, ("atuwi%d: external firmware upload done\n",
	    sc->atuwi_unit));
	
	return 0;
}


Static int atuwi_get_card_config(struct atuwi_softc *sc)
{
	struct atuwi_rfmd_conf		rfmd_conf;
	struct atuwi_intersil_conf	intersil_conf;
	int				err;
	
	
	switch (sc->atuwi_radio) {
	
	case RadioRFMD:
	case RadioRFMD2958:
	case RadioRFMD2958_SMC:
		DEBUG(FLAG_INIT, ("atuwi%d: trying to get rfmd config\n",
		    sc->atuwi_unit)); 
		err = atuwi_usb_request(sc, UT_READ_VENDOR_INTERFACE, 0x33,
		    0x0a02, 0x0000, sizeof(rfmd_conf),
		    (u_int8_t *)&rfmd_conf);
		if (err) {
			ERROR(("atuwi%d: could not get rfmd config!\n",
			    sc->atuwi_unit));
			return err;
		}
		memcpy(sc->atuwi_mac_addr, rfmd_conf.MACAddr, ETHER_ADDR_LEN);
		break;
	
	case RadioIntersil:
		DEBUG(FLAG_INIT, ("atuwi%d: trying to get intersil config\n",
		    sc->atuwi_unit));
		err = atuwi_usb_request(sc, UT_READ_VENDOR_INTERFACE, 0x33,
		    0x0902, 0x0000, sizeof(intersil_conf),
		    (u_int8_t *)&intersil_conf);
		if (err) {
			ERROR(("atuwi%d: could not get intersil config!\n",
			    sc->atuwi_unit));
			return err;
		}
		memcpy(sc->atuwi_mac_addr, intersil_conf.MACAddr,
		    ETHER_ADDR_LEN);
		break;
	}
	
	return 0;
}


/*
 * Probe for an AT76c503 chip.
 */
USB_MATCH(atuwi)
{
	USB_MATCH_START(atuwi, uaa);
	struct atuwi_type		*t;

	if (!uaa->iface)
		return(UMATCH_NONE);

	t = atuwi_devs;
	while(t->atuwi_vid) {
		if (uaa->vendor == t->atuwi_vid &&
		    uaa->product == t->atuwi_pid) {
			return(UMATCH_VENDOR_PRODUCT);
		}
		t++;
	}

	return(UMATCH_NONE);
}


/*
 * this routine gets called from the mgmt thread at least once a second.
 * if we return 0 the thread will go to sleep, if we return 1 we will be
 * called again immediately (like 'continue' does in a while-loop)
 */
Static int atuwi_mgmt_state_machine(struct atuwi_softc *sc)
{
	struct atuwi_mgmt	*vars = &sc->atuwi_mgmt_vars;
	usbd_status		err;
	u_int8_t		statusreq[6];
	
	if (sc->atuwi_mgmt_flags & ATUWI_CHANGED_SETTINGS) {
		sc->atuwi_mgmt_flags &= ~ATUWI_CHANGED_SETTINGS;
		sc->atuwi_mgmt_flags &= ~ATUWI_FOUND_BSSID;
		sc->atuwi_mgmt_flags &= ~ATUWI_AUTH_OK;
		sc->atuwi_mgmt_flags &= ~ATUWI_RE_AUTH;
		sc->atuwi_mgmt_flags &= ~ATUWI_ASSOC_OK;
		sc->atuwi_mgmt_flags |= ATUWI_SEARCHING;
		vars->state = STATE_LISTENING;
		vars->retry = 0;
	}
	
	DEBUG(FLAG_STATE, ("atuwi%d: [state=%s, retry=%d, chan=%d "
	    "enc=%d]\n", sc->atuwi_unit, atuwi_mgmt_statename[vars->state],
	    vars->retry, sc->atuwi_channel, sc->atuwi_encrypt));

	/* Fall back to authentication if needed */
	/* TODO: should we only allow this when in infra-mode? */
	if ((sc->atuwi_mgmt_flags & ATUWI_RE_AUTH) &&
	    (vars->state >= STATE_AUTHENTICATING)) {
	    
		vars->state = STATE_AUTHENTICATING;
		sc->atuwi_mgmt_flags &= ~(ATUWI_RE_AUTH | ATUWI_AUTH_OK |
		    ATUWI_ASSOC_OK);
		vars->retry = 0;
	}
	
	/* Fall back to association if needed */
	/* TODO: should we only allow this when in infra-mode? */
	if ((sc->atuwi_mgmt_flags & ATUWI_RE_ASSOC) &&
	    (vars->state >= STATE_ASSOCIATING)) {

		vars->state = STATE_ASSOCIATING;
		sc->atuwi_mgmt_flags &= ~(ATUWI_RE_ASSOC | ATUWI_AUTH_OK |
		    ATUWI_ASSOC_OK);
		vars->retry = 0;
	}


	switch (vars->state) {
	
	case STATE_NONE:
		/* awaiting orders */
		break;
	
	case STATE_LISTENING:
		/* do some nifty scanning here */

		if (sc->atuwi_mgmt_flags & ATUWI_FOUND_BSSID) {
			vars->state = STATE_JOINING;
			vars->retry = 0;
			return 1;
		}

		/*
		 * We NEED to have Giant locked when calling usb
		 * routines from here, otherwise usbd_transfer will
		 * panic when calling tsleep without a mutex.
		 */
		mtx_lock(&Giant);
		err = atuwi_get_cmd_status(sc, CMD_JOIN, statusreq);
		mtx_unlock(&Giant);
		if (err) {
			ERROR(("atuwi%d: get_cmd_status failed in "
			    "mgmt_loop\n", sc->atuwi_unit));
			vars->state = STATE_GIVEN_UP;
			vars->retry = 0;
		}
		if (statusreq[5]==STATUS_IN_PROGRESS) {
			DEBUG(FLAG_STATE, ("atuwi%d: scanning in "
			    "progress...\n", sc->atuwi_unit));
		} else {
			mtx_lock(&Giant);
			err = atuwi_start_scan(sc);
			mtx_unlock(&Giant);
			if (vars->retry++ > ATUWI_SCAN_RETRIES &&
			    sc->atuwi_mode == AD_HOC_MODE) {
				ERROR(("atuwi%d: scanned long enough\n",
				    sc->atuwi_unit));
				
				sc->atuwi_mgmt_flags &= ~ATUWI_SEARCHING;
				
				vars->state = STATE_CREATING_IBSS;
				vars->retry = 0;
			}
			if (err) {
				ERROR(("atuwi%d: get_cmd_failed in "
				    "mgmt_loop\n", sc->atuwi_unit));
				vars->state = STATE_GIVEN_UP;
				vars->retry = 0;
			}
		}
		break;
	
	case STATE_JOINING:
		DEBUG(FLAG_STATE, ("atuwi%d: going to join\n",
		    sc->atuwi_unit));
		mtx_lock(&Giant);
		err = atuwi_join(sc);
		mtx_unlock(&Giant);
		if (err) {
			if (vars->retry++ > ATUWI_JOIN_RETRIES) {
				if (sc->atuwi_mode == AD_HOC_MODE)
					vars->state = STATE_CREATING_IBSS;
				else
					vars->state = STATE_GIVEN_UP;
				vars->retry = 0;
			}
			ERROR(("atuwi%d: error joining\n",
			    sc->atuwi_unit));
		} else {
			if (sc->atuwi_mode == AD_HOC_MODE)
				vars->state = STATE_HAPPY_NETWORKING;
			else
				vars->state = STATE_AUTHENTICATING;
			vars->retry = 0;
		}
		break;
		
	case STATE_AUTHENTICATING:
		if (sc->atuwi_mgmt_flags & ATUWI_AUTH_OK) {
			vars->state = STATE_ASSOCIATING;
			vars->retry = 0;
			return 1;
		}

		DEBUG(FLAG_STATE, ("atuwi%d: trying authentication\n",
		    sc->atuwi_unit));
		mtx_lock(&Giant);
		atuwi_authenticate(sc);
		mtx_unlock(&Giant);
		if (vars->retry++ > ATUWI_AUTH_RETRIES) {
			vars->state = STATE_GIVEN_UP;
			ERROR(("atuwi%d: error authenticating...\n",
			    sc->atuwi_unit));
		}
		break;
		
	case STATE_ASSOCIATING:
		if (sc->atuwi_mgmt_flags & ATUWI_ASSOC_OK) {
			vars->state = STATE_HAPPY_NETWORKING;
			vars->retry = 0;
			return 1;
		}
		
		DEBUG(FLAG_STATE, ("atuwi%d: trying to associate\n",
		    sc->atuwi_unit));
		mtx_lock(&Giant);
		atuwi_associate(sc);
		mtx_unlock(&Giant);
		if (vars->retry++ > ATUWI_ASSOC_RETRIES) {
			vars->state = STATE_GIVEN_UP;
			ERROR(("atuwi%d: error associating...\n",
			    sc->atuwi_unit));
		}
		break;
	
	case STATE_CREATING_IBSS:
		DEBUG(FLAG_STATE, ("atuwi%d: trying to create IBSS\n",
		    sc->atuwi_unit));
		mtx_lock(&Giant);
		err = atuwi_start_ibss(sc);
		mtx_unlock(&Giant);
		if (err) {
			if (vars->retry++ > ATUWI_IBSS_RETRIES)
				vars->state = STATE_GIVEN_UP;
			ERROR(("atuwi:%d: error creating IBSS...\n",
			    sc->atuwi_unit));
		} else {
			vars->state = STATE_HAPPY_NETWORKING;
			vars->retry = 0;
		}
		break;
		
	
	case STATE_HAPPY_NETWORKING:
		/* happy networking
		 *
		 * TODO:
		 * we should bounce back to previous states from here
		 * on beacon timeout, disassociate or deauthenticate
		 *
		 * (but none of that has been implemented at this time)
		 */			
		break;
		
	case STATE_GIVEN_UP:
		/*
		 * can only leave this state if someone changes the
		 * config
		 */
		break;
	}

	if (vars->state == STATE_HAPPY_NETWORKING)
		sc->atuwi_mgmt_flags |= ATUWI_NETWORK_OK;
	else
		sc->atuwi_mgmt_flags &= ~ATUWI_NETWORK_OK;
	
	return 0;
}


Static void atuwi_mgmt_loop(void *arg)
{
	struct atuwi_softc	*sc = arg;
	int			again;


	DEBUG(FLAG_INIT, ("atuwi%d: mgmt task initialised\n", sc->atuwi_unit));
	
	sc->atuwi_mgmt_vars.state = STATE_NONE;
	sc->atuwi_mgmt_vars.retry = 0;
	
	while (!sc->atuwi_dying) {

		ATUWI_LOCK(sc);

		again = atuwi_mgmt_state_machine(sc);
		while (again)
			again = atuwi_mgmt_state_machine(sc);

		ATUWI_UNLOCK(sc);


		/*
		 * wait for something to happen (but not too long :)
		 * if someone changes the config or a mgmt packet is received
		 * we will be waken up
		 */
		tsleep(sc, PZERO | PCATCH, "atuwim", ATUWI_MGMT_INTERVAL);
	}
	
	DEBUG(FLAG_INIT, ("atuwi%d: mgmt thread stops now...\n",
	    sc->atuwi_unit));
	sc->atuwi_dying++;

	sc->atuwi_mgmt_flags &= ~ATUWI_TASK_RUNNING;

	mtx_lock(&Giant);
	kthread_exit(0);
}


Static int
atuwi_media_change(struct ifnet *ifp)
{
	struct atuwi_softc	*sc;
	struct ifmedia_entry	*ime;
	
	sc = ifp->if_softc;
	ime = sc->atuwi_media.ifm_cur;
	
	/* TODO: fully implement - see if_wi.c @ 1189 */
	
	DEBUG(FLAG_ALWAYS, ("atuwi%d: subtype=%d %d\n", sc->atuwi_unit,
	    IFM_SUBTYPE(ime->ifm_media), ime->ifm_media));
	
	if ((ime->ifm_media & IFM_IEEE80211_ADHOC) &&
	    (sc->atuwi_mode != AD_HOC_MODE)) {
		DEBUG(FLAG_ALWAYS, ("atuwi%d: mode changed to adhoc\n",
		    sc->atuwi_unit));
		
		sc->atuwi_mode = AD_HOC_MODE;
		sc->atuwi_mgmt_flags |= ATUWI_CHANGED_SETTINGS;
		wakeup(sc);
	}

	if ((!(ime->ifm_media & IFM_IEEE80211_ADHOC)) &&
	    (sc->atuwi_mode != INFRASTRUCTURE_MODE)) {
		DEBUG(FLAG_ALWAYS, ("atuwi%d: mode changed to infra\n",
		    sc->atuwi_unit));
		
		sc->atuwi_mode = INFRASTRUCTURE_MODE;
		sc->atuwi_mgmt_flags |= ATUWI_CHANGED_SETTINGS;
		wakeup(sc);
	}
	
	DEBUG(FLAG_ALWAYS, ("atuwi%d: media_change...\n", sc->atuwi_unit));
	
	return 0;
}


Static void
atuwi_media_status(struct ifnet *ifp, struct ifmediareq *req)
{
	struct atuwi_softc	*sc;
	
	sc = ifp->if_softc;
	
	/* TODO: fully implement */
	
	req->ifm_status = IFM_AVALID;
	req->ifm_active = IFM_IEEE80211;

	if (sc->atuwi_mgmt_flags & ATUWI_NETWORK_OK)
		req->ifm_status |= IFM_ACTIVE;
	
	/* req->ifm_active |= ieee80211_rate2media(2*11, IEEE80211_T_DS); */
	
	if (sc->atuwi_mode == AD_HOC_MODE) {
		req->ifm_active |= IFM_IEEE80211_ADHOC;
	}

	DEBUG(FLAG_IOCTL, ("atuwi%d: atuwi_media_status\n", sc->atuwi_unit));
}


/*
 * Attach the interface. Allocate softc structures, do
 * setup and ethernet/BPF attach.
 */
USB_ATTACH(atuwi)
{
	USB_ATTACH_START(atuwi, sc, uaa);
	char				devinfo[1024];
	struct ifnet			*ifp;
	usbd_status			err; 
	usb_interface_descriptor_t	*id;
	usb_endpoint_descriptor_t	*ed;
	int				i;
	u_int8_t			mode;
	struct atuwi_type		*t;
	/*
	struct atuwi_fw			fw;
	*/


	bzero(sc, sizeof(struct atuwi_softc));
	sc->atuwi_iface = uaa->iface;
	sc->atuwi_udev = uaa->device;
	sc->atuwi_unit = device_get_unit(self);

	id = usbd_get_interface_descriptor(uaa->iface);

	usbd_devinfo(uaa->device, 0, devinfo);
	device_set_desc_copy(self, devinfo);

	DEBUG(FLAG_ALWAYS, ("atuwi%d: %s\n", sc->atuwi_unit, devinfo));

	/*
	 * look up the radio_type for the device
	 * basically does the same as USB_MATCH
	 */
	t = atuwi_devs;
	while(t->atuwi_vid) {
		if (uaa->vendor == t->atuwi_vid &&
		    uaa->product == t->atuwi_pid) {
			sc->atuwi_radio = t->atuwi_radio;
			sc->atuwi_quirk = t->atuwi_quirk;
		}
		t++;
	}

	mtx_init(&sc->atuwi_mtx, device_get_nameunit(self), MTX_NETWORK_LOCK,
	    MTX_DEF | MTX_RECURSE);
	
	ATUWI_LOCK(sc);

	/*
	 * Check in the interface descriptor if we're in DFU mode
	 * If we're in DFU mode, we upload the external firmware
	 * If we're not, the PC must have rebooted without power-cycling
	 * the device.. I've tried this out, a reboot only requeres the
	 * external firmware to be reloaded :)
	 *
	 * Hmm. The at76c505a doesn't report a DFU descriptor when it's
	 * in DFU mode... Let's just try to get the opmode
	 */

	err = atuwi_get_opmode(sc, &mode);

	if (err || (mode != MODE_NETCARD &&
	    mode != MODE_NOFLASHNETCARD)) {

		DEBUG(FLAG_INIT, ("atuwi%d: starting internal firmware "
		    "download\n", sc->atuwi_unit));

		/* upload internal firmware */
		err = atuwi_upload_internal_firmware(sc);
		if (err) {
			ATUWI_UNLOCK(sc);
			mtx_destroy(&sc->atuwi_mtx);
			USB_ATTACH_ERROR_RETURN;
		}
		
		DEBUG(FLAG_INIT, ("atuwi%d: done...\n",
		    sc->atuwi_unit));
		
		ATUWI_UNLOCK(sc);
		mtx_destroy(&sc->atuwi_mtx);
		USB_ATTACH_NEED_RESET;
	} 	

	uaa->iface = sc->atuwi_iface;
	
	/* upload external firmware */
	DEBUG(FLAG_INIT, ("atuwi%d: starting external firmware download\n",
	    sc->atuwi_unit));
	err = atuwi_upload_external_firmware(sc);
	if (err) {
		ATUWI_UNLOCK(sc);
		mtx_destroy(&sc->atuwi_mtx);
		USB_ATTACH_ERROR_RETURN;
	}
	
	DEBUG(FLAG_INIT, ("atuwi%d: done...\n", sc->atuwi_unit));

	/* Find endpoints. */
	for (i = 0; i < id->bNumEndpoints; i++) {
		ed = usbd_interface2endpoint_descriptor(uaa->iface, i);
		if (!ed) {
			ERROR(("atuwi%d: num_endp:%d\n", sc->atuwi_unit,
			    uaa->iface->idesc->bNumEndpoints));
			ERROR(("atuwi%d: couldn't get ep %d\n",
			    sc->atuwi_unit, i));

			ATUWI_UNLOCK(sc);
			mtx_destroy(&sc->atuwi_mtx);
			USB_ATTACH_ERROR_RETURN;
		}
		if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_IN &&
		    UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			sc->atuwi_ed[ATUWI_ENDPT_RX] = ed->bEndpointAddress;
		} else if (UE_GET_DIR(ed->bEndpointAddress) == UE_DIR_OUT &&
			   UE_GET_XFERTYPE(ed->bmAttributes) == UE_BULK) {
			sc->atuwi_ed[ATUWI_ENDPT_TX] = ed->bEndpointAddress;
		}
	}
	

	/* read device config & get MAC address */
	err = atuwi_get_card_config(sc);
	if (err) {
		ERROR(("atuwi%d: could not get card cfg!\n", sc->atuwi_unit));
		
		ATUWI_UNLOCK(sc);
		mtx_destroy(&sc->atuwi_mtx);
		USB_ATTACH_ERROR_RETURN;
	}


	/* DEBUG : try to get firmware version */
	/*
	err = atuwi_get_mib(sc, MIB_FW_VERSION, sizeof(fw), 0,
	    (u_int8_t *)&fw);
	if (!err) {
		DEBUG(FLAG_FW, ("atuwi%d: firmware: maj:%d min:%d patch:%d "
		    "build:%d\n", sc->atuwi_unit, fw.major, fw.minor,
		    fw.patch, fw.build));
	} else {
		ERROR(("atuwi%d: get firmware version failed\n",
		    sc->atuwi_unit));
	}
	*/

	/* Show the world our MAC address */
	DEBUG(FLAG_INIT, ("atuwi%d: Ethernet address: %6D\n", sc->atuwi_unit,
		sc->atuwi_mac_addr, ":"));

	bcopy(sc->atuwi_mac_addr,
		(char *)&sc->arpcom.ac_enaddr, ETHER_ADDR_LEN);


	for (i=0; i<ATUWI_AVG_TIME; i++)
		sc->atuwi_signalarr[i] = 0;
	sc->atuwi_signaltotal = 0;
	sc->atuwi_signalptr = 0;

	sc->atuwi_cdata.atuwi_tx_inuse = 0;


	ifp = &sc->arpcom.ac_if;
	ifp->if_softc = sc;
#if __FreeBSD_version >= 501113
	if_initname(ifp, "atuwi", sc->atuwi_unit);
#else
	ifp->if_unit = sc->atuwi_unit;
	ifp->if_name = "atuwi";
	#endif
#ifdef IFF_NEEDSGIANT
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST |
	    IFF_NEEDSGIANT;
#else
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
#endif
	ifp->if_ioctl = atuwi_ioctl;
	ifp->if_output = ether_output;
	ifp->if_start = atuwi_start;
	ifp->if_watchdog = atuwi_watchdog;
	ifp->if_init = atuwi_init;
	ifp->if_baudrate = 10000000;
	ifp->if_snd.ifq_maxlen = IFQ_MAXLEN;
	
	sc->atuwi_encrypt = ATUWI_WEP_OFF;
	sc->atuwi_wepkeylen = ATUWI_WEP_104BITS;
	sc->atuwi_wepkey = 0;

	sc->atuwi_mgmt_flags = 0;
	
	bzero(sc->atuwi_bssid, ETHER_ADDR_LEN);
	sc->atuwi_ssidlen = strlen(ATUWI_DEFAULT_SSID);
	memcpy(sc->atuwi_ssid, ATUWI_DEFAULT_SSID, sc->atuwi_ssidlen);
	sc->atuwi_channel = 10;
	sc->atuwi_mode = INFRASTRUCTURE_MODE;
	sc->atuwi_encrypt = ATUWI_WEP_OFF;
	
	sc->atuwi_qdat.ifp = ifp;
	sc->atuwi_qdat.if_rxstart = atuwi_rxstart;
	
	/* Initialise transfer lists */
	SLIST_INIT(&sc->atuwi_cdata.atuwi_rx_free);
	SLIST_INIT(&sc->atuwi_cdata.atuwi_tx_free);
	SLIST_INIT(&sc->atuwi_cdata.atuwi_mgmt_free);

	/*
	 * Call MI attach routine.
	 */
	ether_ifattach(ifp, sc->atuwi_mac_addr);
	usb_register_netisr();
	sc->atuwi_dying = 0;

	ifp->if_mtu = ATUWI_DEFAULT_MTU;


	/* setup ifmedia interface */
	ifmedia_init(&sc->atuwi_media, 0, atuwi_media_change,
	    atuwi_media_status);

#define ADD(s, o)       ifmedia_add(&sc->atuwi_media, \
	IFM_MAKEWORD(IFM_IEEE80211, (s), (o), 0), 0, NULL)

	ADD(IFM_AUTO, 0);
	ADD(IFM_AUTO, IFM_IEEE80211_ADHOC);
	
	/*
	 * TODO:
	 * add a list of supported rates here.
	 * (can't do that as long as we only support 'auto fallback'
	 *
	for (i = 0; i < nrate; i++) {
		r = ic->ic_sup_rates[i];
		mword = ieee80211_rate2media(r, IEEE80211_T_DS);
		if (mword == 0)
			continue;
		printf("%s%d%sMbps", (i != 0 ? " " : ""),
			(r & IEEE80211_RATE_VAL) / 2, ((r & 0x1) != 0 ? ".5"
			    : ""));
		ADD(mword, 0);
		if (ic->ic_flags & IEEE80211_F_HASHOSTAP)
			ADD(mword, IFM_IEEE80211_HOSTAP);
		if (ic->ic_flags & IEEE80211_F_HASIBSS)  
			ADD(mword, IFM_IEEE80211_ADHOC); 
		ADD(mword, IFM_IEEE80211_ADHOC | IFM_FLAG0);
	}
	printf("\n");
	*/
	
	ADD(11, 0);
	ADD(11, IFM_IEEE80211_ADHOC);
	
	ifmedia_set(&sc->atuwi_media, IFM_MAKEWORD(IFM_IEEE80211, IFM_AUTO, 0,
	    0));
#undef ADD


	/* attach to bpf for raw 802.11 packets */
/* #if NBPFILTER > 0  */
	bpfattach2(ifp, DLT_IEEE802_11, sizeof(struct ieee80211_frame_addr4),
	    &sc->atuwi_rawbpf);
	bpfattach2(ifp, DLT_AIRONET_HEADER, sizeof(struct an_rxframe),
	    &sc->atuwi_airobpf);
/* #endif */

	ATUWI_UNLOCK(sc);

	DEBUG(FLAG_ALWAYS, ("atuwi%d: %s attached successfully\n",
	    sc->atuwi_unit, devinfo));
	
	USB_ATTACH_SUCCESS_RETURN;
}


Static int
atuwi_detach(device_ptr_t dev)
{
	struct atuwi_softc	*sc;
	struct ifnet		*ifp;
	
	sc = device_get_softc(dev);
	ATUWI_LOCK(sc);
	ifp = &sc->arpcom.ac_if;

	printf("atuwi%d: detach\n", sc->atuwi_unit);

	atuwi_stop(sc);

/* #if NBPFILTER > 0  */
	bpfdetach(ifp);
	bpfdetach(ifp);
/* #endif */

	ifmedia_removeall(&sc->atuwi_media);
	
	if (ifp != NULL)
		ether_ifdetach(ifp);

	if (sc->atuwi_ep[ATUWI_ENDPT_TX] != NULL)
		usbd_abort_pipe(sc->atuwi_ep[ATUWI_ENDPT_TX]);
	if (sc->atuwi_ep[ATUWI_ENDPT_RX] != NULL)
		usbd_abort_pipe(sc->atuwi_ep[ATUWI_ENDPT_RX]);
	ATUWI_UNLOCK(sc);
	mtx_destroy(&sc->atuwi_mtx);

	return(0);
}


/*
 * Initialize an RX descriptor and attach an MBUF cluster.
 */
Static int
atuwi_newbuf(struct atuwi_softc *sc, struct atuwi_chain *c, struct mbuf *m)
{
	struct mbuf		*m_new = NULL;

	if (m == NULL) {
		MGETHDR(m_new, M_DONTWAIT, MT_DATA);
		if (m_new == NULL) {
			ERROR(("atuwi%d: no memory for rx list "
			    "-- packet dropped!\n", sc->atuwi_unit));
			return(ENOBUFS);
		}

		MCLGET(m_new, M_DONTWAIT);
		if (!(m_new->m_flags & M_EXT)) {
			ERROR(("atuwi%d: no memory for rx list "
			    "-- packet dropped!\n", sc->atuwi_unit));
			m_freem(m_new);
			return(ENOBUFS);
		}
		m_new->m_len = m_new->m_pkthdr.len = MCLBYTES;
	} else {
		m_new = m;
		m_new->m_len = m_new->m_pkthdr.len = MCLBYTES;
		m_new->m_data = m_new->m_ext.ext_buf;
	}

	c->atuwi_mbuf = m_new;

	return(0);
}

Static int
atuwi_xfer_list_init(struct atuwi_softc *sc, struct atuwi_chain *ch,
    int listlen, int need_mbuf, int bufsize, struct atuwi_list_head *list)
{
	struct atuwi_cdata	*cd;
	int			i;

	cd = &sc->atuwi_cdata;
	
	DEBUG(FLAG_INIT, ("atuwi%d: list init (%d entries of %d bytes)\n",
	    sc->atuwi_unit, listlen, bufsize));
	
	for (i = 0; i < listlen; i++) {
		ch->atuwi_sc = sc;
		ch->atuwi_idx = i;
		if (ch->atuwi_xfer == NULL) {
			ch->atuwi_xfer = usbd_alloc_xfer(sc->atuwi_udev);
			if (ch->atuwi_xfer == NULL)
				return(ENOBUFS);
		}


		if (need_mbuf) {
			if (atuwi_newbuf(sc, ch, NULL) == ENOBUFS)
				return(ENOBUFS);
		} else {
			ch->atuwi_mbuf = NULL;
		}


		if ((bufsize > 0) && (ch->atuwi_buf == NULL)) {
#ifdef ATUWI_NO_COPY_TX
			ch->atuwi_buf = usbd_alloc_buffer(ch->atuwi_xfer,
			    bufsize);
			if (ch->atuwi_buf == NULL)
				return(ENOBUFS);
#else /* ATUWI_NO_COPY_TX */
			ch->atuwi_buf = malloc(bufsize, M_USBDEV,
			    M_NOWAIT);
			if (ch->atuwi_buf == NULL)
				return(ENOBUFS);
#endif /* ATUWI_NO_COPY_TX */
		}
		
		if (list != NULL) {
			SLIST_INSERT_HEAD(list, ch, atuwi_list);
		}
		
		ch++;
	}

	return(0);
}


Static void
atuwi_xfer_list_free(struct atuwi_softc *sc, struct atuwi_chain *ch,
    int listlen)
{
	int			i;
	
	
	/* Free resources. */
	for (i = 0; i < listlen; i++) {
		if (ch[i].atuwi_buf != NULL) {
#ifdef ATUWI_NO_COPY_TX
			/*
			 * usbdi.c cleans up for us
			 */
#else /* ATUWI_NO_COPY_TX */
			free(ch[i].atuwi_buf, M_USBDEV);
#endif /* ATUWI_NO_COPY_TX */
			ch[i].atuwi_buf = NULL;
		}
		if (ch[i].atuwi_mbuf != NULL) {
			m_freem(ch[i].atuwi_mbuf);
			ch[i].atuwi_mbuf = NULL;
		}
		if (ch[i].atuwi_xfer != NULL) {
			usbd_free_xfer(ch[i].atuwi_xfer);
			ch[i].atuwi_xfer = NULL;
		}
	}
}


Static void
atuwi_rxstart(struct ifnet *ifp)
{
	struct atuwi_softc	*sc;
	struct atuwi_chain	*c;

	sc = ifp->if_softc;
	ATUWI_LOCK(sc);

#ifdef ATUWI_PROFILING
	atuwi_profiling_rxstart++;
#endif /* ATUWI_PROFILING */
	
	c = SLIST_FIRST(&sc->atuwi_cdata.atuwi_rx_free);
	if (c == NULL) {
#ifdef ATUWI_PROFILING
		atuwi_profiling_rxstart_noxfer++;
#endif /* ATUWI_PROFILING */
		ifp->if_ierrors++;
		return;
	}
	
	if (atuwi_newbuf(sc, c, NULL) == ENOBUFS) {
#ifdef ATUWI_PROFILING
		atuwi_profiling_rxstart_nobuf++;
#endif /* ATUWI_PROFILING */
		ifp->if_ierrors++;
		return;
	}

	SLIST_REMOVE_HEAD(&sc->atuwi_cdata.atuwi_rx_free, atuwi_list);

	/* Setup new transfer. */
	usbd_setup_xfer(c->atuwi_xfer, sc->atuwi_ep[ATUWI_ENDPT_RX],
	    c, mtod(c->atuwi_mbuf, char *), ATUWI_RX_BUFSZ,
	    USBD_SHORT_XFER_OK, USBD_NO_TIMEOUT, atuwi_rxeof);
	usbd_transfer(c->atuwi_xfer);

	ATUWI_UNLOCK(sc);

	return;
}


Static void
atuwi_print_beacon(struct atuwi_softc *sc, struct atuwi_rxpkt *pkt)
{
	u_int8_t		*ptr;
	struct tlv		*tlv;
	u_int8_t		*end;
	u_int8_t		tmp;
	u_int8_t		rate;

	/* Let's have a closer look at this beacon... */
	ptr = (u_int8_t *)pkt->WiHeader.addr4 + 12;
	end = ptr + pkt->AtHeader.wlength - 24 - 12 - 4;
	tlv = (struct tlv *)ptr;

	while ((ptr<end) && (ptr + 2 + tlv->length <= end)) {
		switch (tlv->type) {
		case WI_VAR_SSID: /* SSID */
			/* sanity check */
			if (tlv->length > 32)
				break;
			
			tmp = tlv->value[tlv->length];
			tlv->value[tlv->length] = 0;
			DEBUG(FLAG_ALWAYS, ("atuwi%d:  ssid=[%s]\n",
			    sc->atuwi_unit, tlv->value));
			tlv->value[tlv->length] = tmp;
			break;
		case WI_VAR_SRATES: /* Supported rates */
			for (rate=0; rate<tlv->length; rate++) {
				tmp = tlv->value[rate] & (~0x80);
				DEBUG(FLAG_ALWAYS, ("atuwi%d:  rate: %d kbps "
				    "(%02x)\n", sc->atuwi_unit, 500 * tmp,
				    tlv->value[rate]));
			}
			break;
		case WI_VAR_DS: /* DS (channel) */
			DEBUG(FLAG_ALWAYS, ("atuwi%d:  channel=%d\n",
			    sc->atuwi_unit, *tlv->value));
			break;
		default :
			DEBUG(FLAG_ALWAYS, ("atuwi%d:  tlv: t=%02x l=%02x "
			    "v[0]=%02x\n", sc->atuwi_unit, tlv->type,
			    tlv->length, tlv->value[0]));
		}
				
		ptr += 2 + tlv->length;
		tlv = (struct tlv *)ptr;
	}
}


Static void
atuwi_handle_mgmt_packet(struct atuwi_softc *sc, struct atuwi_rxpkt *pkt)
{
	u_int8_t			*ptr;
	struct tlv			*tlv;
	u_int8_t			*end;
	u_int8_t			tmp;
	int				match;
	int				match_channel = 1;
	struct wi_80211_beacon		*beacon;
	struct wi_mgmt_auth_hdr		*auth;
	struct wi_mgmt_deauth_hdr	*deauth;
	struct wi_mgmt_disas_hdr	*deassoc;
	struct wi_mgmt_asresp_hdr	*assoc;
	int				src_ok = 0;
	int				dst_ok = 0;
	
	if (! memcmp(pkt->WiHeader.addr1, sc->atuwi_mac_addr, ETHER_ADDR_LEN))
		dst_ok = 1;
	if ((sc->atuwi_mode == INFRASTRUCTURE_MODE) &&
	    (! memcmp(pkt->WiHeader.addr2, sc->atuwi_bssid, ETHER_ADDR_LEN)))
		src_ok = 1;
	
	switch (pkt->WiHeader.frame_ctl & WI_FCTL_STYPE) {
	case WI_STYPE_MGMT_AUTH:
		if ((!src_ok) || (!dst_ok)) {
			DEBUG(FLAG_MGMT, ("atuwi%d: auth response not for "
			    "us... (src_ok=%d dst_ok=%d)\n", sc->atuwi_unit,
			    src_ok, dst_ok));
			break;
		}
	
		DEBUG(FLAG_STATE, ("atuwi%d: received auth response...\n",
		    sc->atuwi_unit));
		
		auth = (struct wi_mgmt_auth_hdr *)pkt->WiHeader.addr4;
		
		if (auth->wi_seq != 2) {
			DEBUG(FLAG_MGMT, ("atuwi%d: auth wrong seq.nr (%x)\n",
			    sc->atuwi_unit, auth->wi_seq));
			break;
		}
		if (auth->wi_status != 0) {
			DEBUG(FLAG_MGMT, ("atuwi%d: auth status error (%x)\n",
			    sc->atuwi_unit, auth->wi_status));
			break;
		}
		
		sc->atuwi_mgmt_flags |= ATUWI_AUTH_OK;
		wakeup(sc);
		
		/*
		 * TODO: DAAN Daan daan - Add challenge text blah blah
		 * (for shared-key systems)
		 */
		/*
		memcpy(the_challenge_text, ((u_int8_t *)&pkt->WiHeader) + 30,
		    the_challenge_length);
		DEBUG(FLAG_MGMT, ("atuwi%d: challenge= %100D\n",
		    sc->atuwi_unit, the_challende_text, " "));
		*/
		break;
	
	case WI_STYPE_MGMT_DEAUTH:
		if ((!src_ok) || (!dst_ok)) {
			DEBUG(FLAG_MGMT, ("atuwi%d: de-auth response not "
			    "for us... (src_ok=%d dst_ok=%d)\n",
			    sc->atuwi_unit, src_ok, dst_ok));
			break;
		}
		DEBUG(FLAG_STATE, ("atuwi%d: the AP has de-authenticated "
		    "us\n", sc->atuwi_unit));
		
		deauth = (struct wi_mgmt_deauth_hdr *)pkt->WiHeader.addr4;
		
		DEBUG(FLAG_STATE, ("autiw%d: de-authentication reason: %04x\n",
		    sc->atuwi_unit, deauth->wi_reason));
		
		/* wake up the state machine to get us re-authenticated */
		sc->atuwi_mgmt_flags |= ATUWI_RE_AUTH;
		wakeup(sc);
		break;
	
	case WI_STYPE_MGMT_ASRESP:
		if ((!src_ok) || (!dst_ok)) {
			DEBUG(FLAG_MGMT, ("atuwi%d: assoc response not "
			    "for us... (src_ok=%d dst_ok=%d)\n",
			    sc->atuwi_unit, src_ok, dst_ok));
			break;
		}
		DEBUG(FLAG_STATE, ("atuwi%d: received assoc response...\n",
		    sc->atuwi_unit));
		
		assoc = (struct wi_mgmt_asresp_hdr *)pkt->WiHeader.addr4;
		
		if (assoc->wi_status == 0) {
			sc->atuwi_mgmt_flags |= ATUWI_ASSOC_OK;
			wakeup(sc);
		} else {
			ERROR(("atuwi%d: assoc status error (%x)\n",
			    sc->atuwi_unit, assoc->wi_status));
			break;
		}
		
		wakeup(sc);
		break;

	case WI_STYPE_MGMT_DISAS:
		if ((!src_ok) || (!dst_ok)) {
			DEBUG(FLAG_MGMT, ("atuwi%d: dis-assoc response not "
			    "for us... (src_ok=%d dst_ok=%d)\n",
			    sc->atuwi_unit, src_ok, dst_ok));
			break;
		}
		DEBUG(FLAG_STATE, ("atuwi%d: the AP has de-associated us",
		    sc->atuwi_unit));

		deassoc = (struct wi_mgmt_disas_hdr *)pkt->WiHeader.addr4;

		DEBUG(FLAG_STATE, ("autiw%d: de-association reason: %04x\n",
		    sc->atuwi_unit, deassoc->wi_reason));

		/* wake up the state machine to get us re-authenticated */
		sc->atuwi_mgmt_flags |= ATUWI_RE_ASSOC;
		wakeup(sc);
		break;
	
	case WI_STYPE_MGMT_PROBERESP:
		DEBUG(FLAG_MGMT, ("atuwi%d: PROBE RESPONSE\n",
		    sc->atuwi_unit));
		/* FALLTHROUGH */
	case WI_STYPE_MGMT_BEACON:

		beacon = (struct wi_80211_beacon *)&pkt->WiHeader.addr4;

		/* Show beacon src MAC & signal strength */
		DEBUG(FLAG_BEACONS, ("atuwi%d: mgmt bssid=%6D mac=%6D "
		    "signal=%d\n", sc->atuwi_unit, pkt->WiHeader.addr3, ":",
		    pkt->WiHeader.addr2, ":", pkt->AtHeader.rssi));
	
		if (atuwi_debug & FLAG_SIGNAL) {
			/*
			 * calculate average signal strength, can be very
			 *  usefull when precisely aiming antenna's
			 * NOTE: this is done on ALL beacons, so multiple
			 * stations can end up in the average. this only
			 * works well if we're only receiving one station.
			 */
			sc->atuwi_signaltotal += pkt->AtHeader.rssi;
			sc->atuwi_signaltotal -=
			    sc->atuwi_signalarr[sc->atuwi_signalptr];
			sc->atuwi_signalarr[sc->atuwi_signalptr] =
			    pkt->AtHeader.rssi;
			sc->atuwi_signalptr=(sc->atuwi_signalptr+1) %
			    ATUWI_AVG_TIME;
			DEBUG(FLAG_ALWAYS, ("atuwi%d: mgmt mac=%6D signal=%02d"
			    " ptr=%02d avg=%02d.%02d\n", sc->atuwi_unit,
			    pkt->WiHeader.addr2, ":", pkt->AtHeader.rssi,
			    sc->atuwi_signalptr,
			    sc->atuwi_signaltotal / ATUWI_AVG_TIME,
			    (sc->atuwi_signaltotal * 100 / ATUWI_AVG_TIME) %
			    100));
		}


		DEBUG(FLAG_BEACONS, ("atuwi%d: mgmt capabilities=%04x "
		    "(mode=%s, wep=%s, short-preamble=%s)\n", sc->atuwi_unit,
		    beacon->flags,
		    (beacon->flags & IEEE80211_CAPINFO_ESS) ?
		    "infra" : "ad-hoc",
		    (beacon->flags & IEEE80211_CAPINFO_PRIVACY) ?
		    "on" : "off",
		    (beacon->flags & IEEE80211_CAPINFO_SHORT_PREAMBLE) ?
		    "yes" : "no"));
		

		if (atuwi_debug & FLAG_BEACONSFULL)
			atuwi_print_beacon(sc, pkt);

		if (!(sc->atuwi_mgmt_flags & ATUWI_SEARCHING))
			break;

		/* Let's have a closer look at this beacon... */
		ptr = (u_int8_t *)pkt->WiHeader.addr4 + 12;
		end = ptr + pkt->AtHeader.wlength - 24 - 12 - 4;
		tlv = (struct tlv *)ptr;
		match = 0;
		while ((ptr<end) && (ptr + 2 + tlv->length <= end)) {
			switch (tlv->type) {
			case WI_VAR_SSID: /* SSID */
				/* sanity check */
				if (tlv->length > 32)
					break;
				
				tmp = tlv->value[tlv->length];
				tlv->value[tlv->length] = 0;
				sc->atuwi_ssid[sc->atuwi_ssidlen] = 0;
				if (!strcmp(tlv->value, sc->atuwi_ssid)) {
					match = 1;
				}
				tlv->value[tlv->length] = tmp;
				break;
			case WI_VAR_SRATES: /* Supported rates */
				/*
				 * TODO: should check if we support all
				 *  mandatory rates
				 */
				break;
			case WI_VAR_DS: /* DS (channel) */
				if (match)
					match_channel = *tlv->value;
				break;
			}
					
			ptr += 2 + tlv->length;
			tlv = (struct tlv *)ptr;
		}
		
		/* check mode... */
		beacon = (struct wi_80211_beacon *)&pkt->WiHeader.addr4;
		if (match) {
			if ((sc->atuwi_mode == AD_HOC_MODE) &&
			    (beacon->flags & IEEE80211_CAPINFO_ESS))
				match = 0;
			if ((sc->atuwi_mode == INFRASTRUCTURE_MODE) &&
			    (!(beacon->flags & IEEE80211_CAPINFO_ESS)))
				match = 0;

			if (!match) {
				DEBUG(FLAG_STATE, ("atuwi%d: SSID matches, "
				    "but we're in %s mode instead of %s\n",
				    sc->atuwi_unit,
				    (sc->atuwi_mode == AD_HOC_MODE) ?
				    "adhoc" : "infra",
				    (sc->atuwi_mode == AD_HOC_MODE) ?
				    "infra" : "adhoc"));
				break;
			}
		}
		
		if (match) {
			DEBUG(FLAG_ALWAYS, ("atuwi%d: ==> MATCH! (BSSID=%6D, "
			    "ch=%d)\n", sc->atuwi_unit,
			    pkt->WiHeader.addr3, ":", match_channel));
			
			/*
			 * TODO: should do some channel-checking here instead
			 * of just ignoring the channel the user sets
			 */
			
			memcpy(sc->atuwi_bssid, pkt->WiHeader.addr3,
			    ETHER_ADDR_LEN);
			sc->atuwi_channel = match_channel;
			
			sc->atuwi_mgmt_flags &= ~ATUWI_SEARCHING;
			sc->atuwi_mgmt_flags |= ATUWI_FOUND_BSSID;
		}

	
		break;

	default:
		DEBUG(FLAG_ALWAYS, ("atuwi%d: FIXME: unhandled mgmt type! "
		    "(stype=%x)\n", sc->atuwi_unit,
		    pkt->WiHeader.frame_ctl & WI_FCTL_STYPE));
	}
}


/*
 * A frame has been uploaded: pass the resulting mbuf chain up to
 * the higher level protocols.
 */
Static void atuwi_rxeof(usbd_xfer_handle xfer, usbd_private_handle priv,
    usbd_status status)
{
	struct atuwi_softc	*sc;
	struct atuwi_chain	*c;
        struct mbuf		*m;
        struct ifnet		*ifp;
	int			total_len = 0;
	
	struct atuwi_rxpkt	*pkt;
	struct ether_header	*eth_hdr;
	int			offset;
	

	c = priv;
	sc = c->atuwi_sc;
	ATUWI_LOCK(sc);
	ifp = &sc->arpcom.ac_if;

#ifdef ATUWI_PROFILING
	atuwi_profiling_rxeof++;
#endif /* ATUWI_PROFILING */

	if (status != USBD_NORMAL_COMPLETION) {
		if (status == USBD_NOT_STARTED || status == USBD_CANCELLED) {
			ERROR(("atuwi%d: rxeof notstarted / cancelled\n",
			    sc->atuwi_unit));
			ATUWI_UNLOCK(sc);
			return;
		}
		if (status == USBD_IOERROR) {
			ERROR(("atuwi%d: rx: EEK! lost device?\n",
			    sc->atuwi_unit));
			
			/*
			 * My experience with USBD_IOERROR is that trying to
			 * restart the transfer will always fail and we'll
			 * keep on looping restarting transfers untill someone
			 * pulls the plug of the device.
			 * So we don't restart the transfer, but just let it
			 * die... If someone knows of a situation where we can
			 * recover from USBD_IOERROR, let me know.
			 */
			ATUWI_UNLOCK(sc);
			return;
		}
		
		if (usbd_ratecheck(&sc->atuwi_rx_notice)) {
			ERROR(("atuwi%d: usb error on rx: %s\n",
			    sc->atuwi_unit, usbd_errstr(status)));
		}
		if (status == USBD_STALLED)
			usbd_clear_endpoint_stall(
			    sc->atuwi_ep[ATUWI_ENDPT_RX]);
		goto done;
	}

	usbd_get_xfer_status(xfer, NULL, NULL, &total_len, NULL);
	if (total_len <= 1)
		goto done;

#ifdef ATUWI_PROFILING
	atuwi_profiling_packets_in++;
	atuwi_profiling_bytes_in += total_len;
#endif /* ATUWI_PROFILING */


	m = c->atuwi_mbuf;
	pkt = mtod(m, struct atuwi_rxpkt *);
	
	/* drop the raw 802.11b packets to bpf (including 4-byte FCS) */
	if (sc->atuwi_rawbpf) {
		/*
		 * TODO: could drop mbuf's to bpf
		 */
		DEBUG(FLAG_ALWAYS, ("atuwi%d: bpf raw tap on RX :)\n",
		    sc->atuwi_unit));

		bpf_tap(sc->atuwi_rawbpf, (u_int8_t *)&pkt->WiHeader,
		     pkt->AtHeader.wlength);
	}

	/* drop the raw 802.11b packets to bpf (with aironet header) */
	if (sc->atuwi_airobpf) {
		DEBUG(FLAG_BPF, ("atuwi%d: bpf aironet tap on RX\n",
		    sc->atuwi_unit));
		
		atuwi_airo_tap(sc, (u_int8_t *)&pkt->WiHeader,
		    pkt->AtHeader.wlength - 4, &pkt->AtHeader);
	}

	DEBUG(FLAG_RX, ("atuwi%d: -- RX (rate=%d enc=%d) mac=%6D bssid=%6D\n",
	    sc->atuwi_unit, pkt->AtHeader.rx_rate,
	    (pkt->WiHeader.frame_ctl & WI_FCTL_WEP) != 0,
	    pkt->WiHeader.addr3, ":", pkt->WiHeader.addr2, ":"));

	if (pkt->WiHeader.frame_ctl & WI_FCTL_WEP) {
		DEBUG(FLAG_RX, ("atuwi%d: WEP enabled on RX\n",
		    sc->atuwi_unit));
	}

	/* Is it a managment packet? */
	if ((pkt->WiHeader.frame_ctl & WI_FCTL_FTYPE) == WI_FTYPE_MGMT) {

		atuwi_handle_mgmt_packet(sc, pkt);
		goto done;
	}
	
	/* Everything but data packets we just ignore from here */
	if ((pkt->WiHeader.frame_ctl & WI_FCTL_FTYPE) != WI_FTYPE_DATA) {
		DEBUG(FLAG_RX, ("atuwi%d: ---- not a data packet? ---\n",
		    sc->atuwi_unit));
		goto done;
	}
	
	/* Woohaa! It's an ethernet packet! */

	DEBUG(FLAG_RX, ("atuwi%d: received a packet! rx-rate:%d\n",
	    sc->atuwi_unit, pkt->AtHeader.rx_rate));

	/* drop non-encrypted packets if wep-mode=on */
	if ((!(pkt->WiHeader.frame_ctl & WI_FCTL_WEP)) &&
	    (sc->atuwi_encrypt & ATUWI_WEP_RX)) {
		DEBUG(FLAG_RX, ("atuwi%d: dropping RX packet. (wep=off)\n",
		    sc->atuwi_unit));
		goto done;
	}

	DEBUG(FLAG_RX, ("atuwi%d: rx frag:%02x rssi:%02x q:%02x nl:%02x "
	    "time:%d\n", sc->atuwi_unit, pkt->AtHeader.fragmentation,
	    pkt->AtHeader.rssi, pkt->AtHeader.link_quality,
	    pkt->AtHeader.noise_level, pkt->AtHeader.rx_time));

	/* Do some sanity checking... */
	if (total_len < sizeof(struct ether_header)) {
		ERROR(("atuwi%d: Packet too small?? (size:%d)\n",
		    sc->atuwi_unit, total_len));
		ifp->if_ierrors++;
		goto done;
	}
	/* if (total_len > 1514) { */
	if (total_len > 1548) {
		ERROR(("atuwi%d: AAARRRGGGHHH!! Invalid packet size? (%d)\n",
		    sc->atuwi_unit, total_len));
		ifp->if_ierrors++;
		goto done;
	}
	
	/*
	if (!(ifp->if_flags & IFF_RUNNING)) {
		ATUWI_UNLOCK(sc);
		return;
	}
	*/

	/*
	 * Copy src & dest mac to the right place (overwriting part of the
	 * 802.11 header)
	 */
	eth_hdr = (struct ether_header *)(pkt->Packet - 2 * ETHER_ADDR_LEN);

	switch (pkt->WiHeader.frame_ctl & (WI_FCTL_TODS | WI_FCTL_FROMDS)) {
	case 0:
		/* ad-hoc: copy order doesn't matter here */
		memcpy(eth_hdr->ether_shost, pkt->WiHeader.addr2,
		    ETHER_ADDR_LEN);
		memcpy(eth_hdr->ether_dhost, pkt->WiHeader.addr1,
		    ETHER_ADDR_LEN);
		break;
	
	case WI_FCTL_FROMDS:
		/* infra mode: MUST be done in this order! */
		memcpy(eth_hdr->ether_shost, pkt->WiHeader.addr3,
		    ETHER_ADDR_LEN);
		memcpy(eth_hdr->ether_dhost, pkt->WiHeader.addr1,
		    ETHER_ADDR_LEN);
		
		DEBUG(FLAG_RX, ("atuwi%d: infra decap (%d bytes)\n",
		    sc->atuwi_unit, pkt->AtHeader.wlength));
		DEBUG(FLAG_RXFULL, ("atuwi%d: RX: %50D\n", sc->atuwi_unit,
		    (u_int8_t *)&pkt->WiHeader, " "));
		break;
		
	default:
		ERROR(("atuwi%d: we shouldn't receive this (f_cntl=%02x)\n",
		    sc->atuwi_unit, pkt->WiHeader.frame_ctl));
	}

	/* calculate 802.3 packet length (= packet - 802.11 hdr - fcs) */
	total_len = pkt->AtHeader.wlength - sizeof(struct wi_80211_hdr) +
	    2 * ETHER_ADDR_LEN - 4;

	ifp->if_ipackets++;
	m->m_pkthdr.rcvif = (struct ifnet *)&sc->atuwi_qdat;

	/* Adjust mbuf for headers */
	offset = sizeof(struct at76c503_rx_buffer) +
	    sizeof(struct wi_80211_hdr) - 12;
	m->m_pkthdr.len = m->m_len = total_len + offset;
	m_adj(m, offset);

	/* Put the packet on the special USB input queue. */
	usb_ether_input(m);
	ATUWI_UNLOCK(sc);

	/* Put transfer back into rx-chain */
	SLIST_INSERT_HEAD(&sc->atuwi_cdata.atuwi_rx_free, c, atuwi_list);

	return;
done:
	/* Setup new transfer. */
	usbd_setup_xfer(c->atuwi_xfer, sc->atuwi_ep[ATUWI_ENDPT_RX],
	    c, mtod(c->atuwi_mbuf, char *), ATUWI_RX_BUFSZ,
	    USBD_SHORT_XFER_OK, USBD_NO_TIMEOUT, atuwi_rxeof);
	usbd_transfer(c->atuwi_xfer);
	ATUWI_UNLOCK(sc);

	return;
}


/*
 * A frame was downloaded to the chip. It's safe for us to clean up
 * the list buffers.
 */
Static void
atuwi_txeof(usbd_xfer_handle xfer, usbd_private_handle priv, usbd_status status)
{
	struct atuwi_softc	*sc;
	struct atuwi_chain	*c;
	struct ifnet		*ifp;
	usbd_status		err;
#ifdef ATUWI_LAZY_TX_NETISR
	int			cnt;
	struct atuwi_chain	*chain;
#endif /* ATUWI_LAZY_TX_NETISR */

	c = priv;
	sc = c->atuwi_sc;
	ATUWI_LOCK(sc);

#ifdef ATUWI_PROFILING
	atuwi_profiling_txeof++;
#endif /* ATUWI_PROFILING */

	ifp = &sc->arpcom.ac_if;
	ifp->if_timer = 0;

	c->atuwi_in_xfer = 0;
	
	if (c->atuwi_mbuf != NULL) {
		/* put it back on the tx_list */
		sc->atuwi_cdata.atuwi_tx_inuse--;
		SLIST_INSERT_HEAD(&sc->atuwi_cdata.atuwi_tx_free, c,
		    atuwi_list);
	} else {
		/* put it back on the mgmt_list */
		SLIST_INSERT_HEAD(&sc->atuwi_cdata.atuwi_mgmt_free, c,
		    atuwi_list);
	}
	/*
	 * turn off active flag if we're done transmitting.
	 * we don't depend on the active flag anywhere. do we still need to
	 * set it then?
	 */
	if (sc->atuwi_cdata.atuwi_tx_inuse == 0) {
		ifp->if_flags &= ~IFF_OACTIVE;
	}
	DEBUG(FLAG_TX, ("atuwi%d: txeof me=%d  status=%d\n", sc->atuwi_unit,
		c->atuwi_idx, status));

	if (status != USBD_NORMAL_COMPLETION) {
		if (status == USBD_NOT_STARTED || status == USBD_CANCELLED) {
			ATUWI_UNLOCK(sc);
			return;
		}
		
		ERROR(("atuwi%d: usb error on tx: %s\n", sc->atuwi_unit,
		    usbd_errstr(status)));
		if (status == USBD_STALLED)
			usbd_clear_endpoint_stall(
			    sc->atuwi_ep[ATUWI_ENDPT_TX]);
		ATUWI_UNLOCK(sc);
		return;
	}

	usbd_get_xfer_status(c->atuwi_xfer, NULL, NULL, NULL, &err);

	if (c->atuwi_mbuf != NULL) {
		c->atuwi_mbuf->m_pkthdr.rcvif = ifp;
#ifdef ATUWI_LAZY_TX_NETISR
		c->atuwi_not_freed = 1;
		sc->atuwi_lazy_tx_cnt++;
		
		if ((sc->atuwi_lazy_tx_cnt == ATUWI_LAZY_TX_MAX) ||
		    (sc->atuwi_cdata.atuwi_tx_inuse < ATUWI_LAZY_TX_MIN)) {

			chain = sc->atuwi_cdata.atuwi_tx_chain;
#ifdef ATUWI_PROFILING
			atuwi_profiling_netisrs_saved +=
			    sc->atuwi_lazy_tx_cnt - 1;
			atuwi_profiling_netisrs_scheduled++;
#endif /* ATUWI_PROFILING */
			for (cnt=0; cnt<ATUWI_TX_LIST_CNT; cnt++)
				if (chain[cnt].atuwi_not_freed) {
					usb_tx_done(chain[cnt].atuwi_mbuf);
					chain[cnt].atuwi_mbuf = NULL;
					chain[cnt].atuwi_not_freed = 0;
				}

			sc->atuwi_lazy_tx_cnt = 0;
		}
#else /* ATUWI_LAZY_TX_NETISR */
		usb_tx_done(c->atuwi_mbuf);
		c->atuwi_mbuf = NULL;
#endif /* ATUWI_LAZY_TX_NETISR */
	}

	if (err)
		ifp->if_oerrors++;
	else
		ifp->if_opackets++;

	ATUWI_UNLOCK(sc);

	return;
}


#ifdef ATUWI_TX_PADDING
Static u_int8_t
atuwi_calculate_padding(int size)
{
	size %= 64;

	if (size < 50)
		return 50 - size;
	if (size >=61)
		return 64 + 50 - size;

	return 0;
}
#endif /* ATUWI_TX_PADDING */


Static int
atuwi_encap(struct atuwi_softc *sc, struct mbuf *m, struct atuwi_chain *c)
{
	int			total_len;
	struct atuwi_txpkt	*pkt;
	struct ether_header	*eth_hdr;
#ifdef ATUWI_TX_PADDING
	u_int8_t		padding;
#endif /* ATUWI_TX_PADDING */

	/*
	 * Copy the mbuf data into a contiguous buffer, leaving
	 * enough room for the atmel & 802.11 headers
	 */
	total_len = m->m_pkthdr.len;
	
	m_copydata(m, 0, m->m_pkthdr.len, c->atuwi_buf +
		sizeof(pkt->AtHeader) + sizeof(struct wi_80211_hdr) -
		2 * ETHER_ADDR_LEN);

	total_len += sizeof(struct wi_80211_hdr) - 2 * ETHER_ADDR_LEN;
	
	pkt = (struct atuwi_txpkt *)c->atuwi_buf;
	pkt->AtHeader.wlength = total_len;
	pkt->AtHeader.tx_rate = 4;			 /* rate = auto */
	pkt->AtHeader.padding = 0;
	memset(pkt->AtHeader.reserved, 0x00, 4);

	pkt->WiHeader.dur_id = 0x0000;			/* ? */
	pkt->WiHeader.frame_ctl = WI_FTYPE_DATA;

	eth_hdr = (struct ether_header *)(pkt->Packet - 2 * ETHER_ADDR_LEN);

	switch(sc->atuwi_mode) {
	case AD_HOC_MODE:
		/* dest */
		memcpy(pkt->WiHeader.addr1, eth_hdr->ether_dhost,
		    ETHER_ADDR_LEN);
		/* src */
		memcpy(pkt->WiHeader.addr2, eth_hdr->ether_shost,
		    ETHER_ADDR_LEN);
		/* bssid */
		memcpy(pkt->WiHeader.addr3, sc->atuwi_bssid, ETHER_ADDR_LEN);
		DEBUG(FLAG_TX, ("atuwi%d: adhoc encap (bssid=%6D)\n",
		    sc->atuwi_unit, sc->atuwi_bssid, ":"));
		break;
		
	case INFRASTRUCTURE_MODE:
		pkt->WiHeader.frame_ctl|=WI_FCTL_TODS;
		/* bssid */
		memcpy(pkt->WiHeader.addr1, sc->atuwi_bssid, ETHER_ADDR_LEN);
		/* src */
		memcpy(pkt->WiHeader.addr2, eth_hdr->ether_shost,
		    ETHER_ADDR_LEN);
		/* dst */
		memcpy(pkt->WiHeader.addr3, eth_hdr->ether_dhost,
		    ETHER_ADDR_LEN);
		
		DEBUG(FLAG_TX, ("atuwi%d: infra encap (bssid=%6D)\n",
		    sc->atuwi_unit, sc->atuwi_bssid, ":"));
	}
	memset(pkt->WiHeader.addr4, 0x00, ETHER_ADDR_LEN);
	pkt->WiHeader.seq_ctl = 0;
	
	if (sc->atuwi_encrypt & ATUWI_WEP_TX) {
		pkt->WiHeader.frame_ctl |= WI_FCTL_WEP;
		DEBUG(FLAG_TX, ("atuwi%d: turning WEP on on packet\n",
		    sc->atuwi_unit));
	}

	total_len += sizeof(pkt->AtHeader);
#ifdef ATUWI_TX_PADDING
	padding = atuwi_calculate_padding(total_len % 64);
	total_len += padding;
	pkt->AtHeader.padding = padding;
#endif /* ATUWI_TX_PADDING */
	c->atuwi_length = total_len;
	c->atuwi_mbuf = m;
	
	return(0);
}


Static void
atuwi_start(struct ifnet *ifp)
{
	struct atuwi_softc	*sc;
	struct mbuf		*m_head = NULL;
	struct atuwi_cdata	*cd;
	struct atuwi_chain	*entry;
	usbd_status		err;
#ifdef ATUWI_LAZY_TX_NETISR
	struct atuwi_chain	*chain;
	int			cnt;
#endif

	sc = ifp->if_softc;
	ATUWI_LOCK(sc);

#ifdef ATUWI_PROFILING
	atuwi_profiling_start++;
#endif /* ATUWI_PROFILING */

	/*
	if (ifp->if_flags & IFF_OACTIVE) {
		ATUWI_UNLOCK(sc);
		return;
	}
	*/

	/*
	 * TODO:
	 * should we check for IFF_RUNNING here?
	 */
	
	entry = SLIST_FIRST(&sc->atuwi_cdata.atuwi_tx_free);
	while (entry) {

		if (entry == NULL) {
			/* all transfers are in use at this moment */
#ifdef ATUWI_PROFILING
			atuwi_profiling_start_noxfer++;
#endif /* ATUWI_PROFILING */
			ATUWI_UNLOCK(sc);
			return;
		}
	
		if (sc->atuwi_mgmt_vars.state != STATE_HAPPY_NETWORKING) {
			/* don't try to send if we're not associated */
#ifdef ATUWI_PROFILING
			atuwi_profiling_start_noxfer++;
#endif /* ATUWI_PROFILING */
			ATUWI_UNLOCK(sc);
			return;
		}
	
		cd = &sc->atuwi_cdata;

		IF_DEQUEUE(&ifp->if_snd, m_head);
		if (m_head == NULL) {
			/* no packets on queue */
#ifdef ATUWI_PROFILING
			atuwi_profiling_start_nopackets++;
#endif /* ATUWI_PROFILING */
			ATUWI_UNLOCK(sc);
			return;
		}

		SLIST_REMOVE_HEAD(&sc->atuwi_cdata.atuwi_tx_free, atuwi_list);
	
		ifp->if_flags |= IFF_OACTIVE;
		cd->atuwi_tx_inuse++;
	
		DEBUG(FLAG_TX, ("atuwi%d: index:%d (inuse=%d)\n",
		    sc->atuwi_unit, entry->atuwi_idx, cd->atuwi_tx_inuse));

#ifdef ATUWI_LAZY_TX_NETISR
		chain = sc->atuwi_cdata.atuwi_tx_chain;

		/* mtx_lock(&Giant); */

		for (cnt=0; cnt<ATUWI_TX_LIST_CNT; cnt++)
			if (chain[cnt].atuwi_not_freed) {
				chain[cnt].atuwi_not_freed = 0;
				sc->atuwi_lazy_tx_cnt--;
			
				/*
				 * In an interrupt we can have freed the mbuf
				 * between checking atuwi_not_freed and here,
				 * so check if it's still here...
				 */
				if (chain[cnt].atuwi_mbuf != NULL) {
					m_freem(chain[cnt].atuwi_mbuf);
					chain[cnt].atuwi_mbuf = NULL;
				}
#ifdef ATUWI_PROFILING
				atuwi_profiling_netisrs_saved++;
#endif /* ATUWI_PROFILING */
			}

		/* mtx_unlock(&Giant); */

#endif /* ATUWI_LAZY_TX_NETISR */

		err = atuwi_encap(sc, m_head, entry);
		if (err) {
			ERROR(("atuwi%d: error encapsulating packet!\n",
			    sc->atuwi_unit));
			IF_PREPEND(&ifp->if_snd, m_head);
			if (--cd->atuwi_tx_inuse == 0)
				ifp->if_flags &= ~IFF_OACTIVE;
			ATUWI_UNLOCK(sc);
			return;
		}
		err = atuwi_send_packet(sc, entry);
		if (err) {
			ERROR(("atuwi%d: error sending packet!\n",
			    sc->atuwi_unit));
			ATUWI_UNLOCK(sc);
			return;
		}
	
		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
		BPF_MTAP(ifp, m_head);

		/*
		 * Set a timeout in case the chip goes out to lunch.
		 */
		ifp->if_timer = 5;

		entry = SLIST_FIRST(&sc->atuwi_cdata.atuwi_tx_free);
	}
	
	ATUWI_UNLOCK(sc);

	return;
}


Static void
atuwi_init(void *xsc)
{
	struct atuwi_softc	*sc = xsc;
	struct ifnet		*ifp = &sc->arpcom.ac_if;
	struct atuwi_chain	*c;
	struct atuwi_cdata	*cd = &sc->atuwi_cdata;
	usbd_status		err;
	int			i;

	ATUWI_LOCK(sc);

#ifdef ATUWI_PROFILING
		atuwi_profiling_init++;
#endif /* ATUWI_PROFILING */

	DEBUG(FLAG_INIT, ("atuwi%d: atuwi_init\n", sc->atuwi_unit));

	if (ifp->if_flags & IFF_RUNNING) {
		ATUWI_UNLOCK(sc);
		return;
	}

	/* Init TX ring */
	if (atuwi_xfer_list_init(sc, cd->atuwi_tx_chain, ATUWI_TX_LIST_CNT, 0,
	    ATUWI_TX_BUFSZ, &cd->atuwi_tx_free)) {
		ERROR(("atuwi%d: tx list init failed\n", sc->atuwi_unit));
	}

	/* Init RX ring */
	if (atuwi_xfer_list_init(sc, cd->atuwi_rx_chain, ATUWI_RX_LIST_CNT, 1,
	    0, &cd->atuwi_rx_free)) {
		ERROR(("atuwi%d: rx list init failed\n", sc->atuwi_unit));
	}

	/* Init mgmt ring */
	if (atuwi_xfer_list_init(sc, cd->atuwi_mgmt_chain,
	    ATUWI_MGMT_LIST_CNT, 0, ATUWI_MGMT_BUFSZ, &cd->atuwi_mgmt_free)) {
		ERROR(("atuwi%d: rx list init failed\n", sc->atuwi_unit));
	}

	/* Load the multicast filter. */
	/*atuwi_setmulti(sc); */

	/* Open RX and TX pipes. */
	err = usbd_open_pipe(sc->atuwi_iface, sc->atuwi_ed[ATUWI_ENDPT_RX],
	    USBD_EXCLUSIVE_USE, &sc->atuwi_ep[ATUWI_ENDPT_RX]);
	if (err) {
		ERROR(("atuwi%d: open rx pipe failed: %s\n", sc->atuwi_unit,
		    usbd_errstr(err)));
		ATUWI_UNLOCK(sc);
		return;
	}

	err = usbd_open_pipe(sc->atuwi_iface, sc->atuwi_ed[ATUWI_ENDPT_TX],
	    USBD_EXCLUSIVE_USE, &sc->atuwi_ep[ATUWI_ENDPT_TX]);
	if (err) {
		ERROR(("atuwi%d: open tx pipe failed: %s\n", sc->atuwi_unit,
		    usbd_errstr(err)));
		ATUWI_UNLOCK(sc);
		return;
	}

	/* Start up the receive pipe. */
	for (i = 0; i < ATUWI_RX_LIST_CNT; i++) {
		c = &sc->atuwi_cdata.atuwi_rx_chain[i];

		usbd_setup_xfer(c->atuwi_xfer, sc->atuwi_ep[ATUWI_ENDPT_RX],
		    c, mtod(c->atuwi_mbuf, char *), ATUWI_RX_BUFSZ,
		    USBD_SHORT_XFER_OK, USBD_NO_TIMEOUT, atuwi_rxeof);
		usbd_transfer(c->atuwi_xfer);
	}

	bcopy((char *)&sc->arpcom.ac_enaddr, sc->atuwi_mac_addr,
	    ETHER_ADDR_LEN);
	DEBUG(FLAG_INIT, ("atuwi%d: starting up using MAC=%6D\n",
	    sc->atuwi_unit, sc->atuwi_mac_addr, ":"));

	/* Do initial setup */
	err = atuwi_initial_config(sc);
	if (err) {
		DEBUG(FLAG_INIT, ("atuwi%d: initial config failed!\n",
			sc->atuwi_unit));
		ATUWI_UNLOCK(sc);
		return;
	}
	DEBUG(FLAG_INIT, ("atuwi%d: initialised transceiver\n",
	    sc->atuwi_unit));
	
	
	/* Fire up managment task */
	DEBUG(FLAG_INIT, ("atuwi%d: trying to start mgmt task...\n",
	    sc->atuwi_unit));
	if (!(sc->atuwi_mgmt_flags & ATUWI_TASK_RUNNING)) {
		sc->atuwi_dying = 0;
		err = kthread_create(atuwi_mgmt_loop, sc,
		    &sc->atuwi_mgmt_thread, 0, 0, "atuwi%d", sc->atuwi_unit);
		if (err) {
			ERROR(("atuwi%d: failed to create kthread\n",
			    sc->atuwi_unit));
		}

		sc->atuwi_mgmt_flags |= ATUWI_TASK_RUNNING;
	}

	
	/* sc->atuwi_rxfilt = ATUWI_RXFILT_UNICAST|ATUWI_RXFILT_BROADCAST; */

	/* If we want promiscuous mode, set the allframes bit. */
	/*
	if (ifp->if_flags & IFF_PROMISC)
		sc->atuwi_rxfilt |= ATUWI_RXFILT_PROMISC;
	*/

	sc->atuwi_mgmt_flags |= ATUWI_CHANGED_SETTINGS;
	wakeup(sc);

	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

	ATUWI_UNLOCK(sc);

	return;
}


Static void
atuwi_print_a_bunch_of_debug_things(struct atuwi_softc *sc)
{
	usbd_status		err;
	u_int8_t		tmp[32];
	
	
	/* DEBUG */
	err = atuwi_get_mib(sc, MIB_MAC_MGMT__CURRENT_BSSID, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: current BSSID=%6D\n",
	    sc->atuwi_unit, tmp, ":"));

	err = atuwi_get_mib(sc, MIB_MAC_MGMT__BEACON_PERIOD, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: beacon period=%2D\n",
	    sc->atuwi_unit, tmp, ":"));

	err = atuwi_get_mib(sc, MIB_MAC_WEP__PRIVACY_INVOKED, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: privacy invoked=%1D\n",
	    sc->atuwi_unit, tmp, ":"));
	
	err = atuwi_get_mib(sc, MIB_MAC_WEP__ENCR_LEVEL, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: encr_level=%d\n", sc->atuwi_unit,
	    tmp[0]));
	
	err = atuwi_get_mib(sc, MIB_MAC_WEP__ICV_ERROR_COUNT, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: icv error count=%4D\n",
	     sc->atuwi_unit, tmp, ":"));
	
	err = atuwi_get_mib(sc, MIB_MAC_WEP__EXCLUDED_COUNT, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: wep excluded count=%4D\n",
	    sc->atuwi_unit, tmp, ":"));
	
	err = atuwi_get_mib(sc, MIB_MAC_MGMT__POWER_MODE, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: power mode=%d\n", sc->atuwi_unit,
	    tmp[0]));
	
	err = atuwi_get_mib(sc, MIB_PHY__CHANNEL, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("autwi%d: DEBUG: channel=%d\n", sc->atuwi_unit,
	    tmp[0]));

	err = atuwi_get_mib(sc, MIB_PHY__REG_DOMAIN, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("autwi%d: DEBUG: reg domain=%d\n", sc->atuwi_unit,
	    tmp[0]));

	err = atuwi_get_mib(sc, MIB_LOCAL__SSID_SIZE, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("autwi%d: DEBUG: ssid size=%d\n", sc->atuwi_unit,
	    tmp[0]));

	err = atuwi_get_mib(sc, MIB_LOCAL__BEACON_ENABLE, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("autwi%d: DEBUG: beacon enable=%d\n",
	    sc->atuwi_unit, tmp[0]));

	err = atuwi_get_mib(sc, MIB_LOCAL__AUTO_RATE_FALLBACK, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("autwi%d: DEBUG: auto rate fallback=%d\n",
	    sc->atuwi_unit, tmp[0]));
	
	err = atuwi_get_mib(sc, MIB_MAC_ADDR__ADDR, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("autwi%d: DEBUG: mac addr=%6D\n", sc->atuwi_unit,
	    tmp, ":"));

	err = atuwi_get_mib(sc, MIB_MAC__DESIRED_SSID, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("autwi%d: DEBUG: desired ssid=%32D\n",
	    sc->atuwi_unit, tmp, ":"));
	
	err = atuwi_get_mib(sc, MIB_MAC_MGMT__CURRENT_ESSID, tmp);
	if (err) return;
	DEBUG(FLAG_IOCTL, ("atuwi%d: DEBUG: current ESSID=%32D\n",
	    sc->atuwi_unit, tmp, ":"));

}


Static int
atuwi_set_wepkey(struct atuwi_softc *sc, int nr, u_int8_t *key, int len)
{
	if ((len != 5) && (len != 13))
		return EINVAL;

	DEBUG(FLAG_ALWAYS, ("atuwi%d: changed wepkey %d (len=%d)\n",
	    sc->atuwi_unit, nr, len));
	
	memcpy(sc->atuwi_wepkeys[nr], key, len);
	if (len == 13)
		sc->atuwi_wepkeylen = ATUWI_WEP_104BITS;
	else
		sc->atuwi_wepkeylen = ATUWI_WEP_40BITS;
		
	atuwi_send_mib(sc, MIB_MAC_WEP__ENCR_LEVEL, NR(sc->atuwi_wepkeylen));
	
	return atuwi_send_mib(sc, MIB_MAC_WEP__KEYS(nr), key);
}


Static int
atuwi_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct atuwi_softc	*sc = ifp->if_softc;
	int			err = 0;
	
	struct ifreq		*ifr;
	struct ieee80211req	*ireq;
	struct thread		*td = curthread;
	u_int8_t		tmp[32];
	int			len;
	struct wi_req		wreq;
	
	int			change;


	ATUWI_LOCK(sc);

	ifr = (struct ifreq *)data;
	ireq = (struct ieee80211req *)data;
	
	change = ifp->if_flags ^ sc->atuwi_if_flags;

	switch(command) {
	case SIOCSIFMTU:
		if (ifr->ifr_mtu > ATUWI_MAX_MTU) {
			err = EINVAL;
			break;
		}
		ifp->if_mtu = ifr->ifr_mtu;
		break;
		
	case SIOCSIFFLAGS:
		err = suser(td);
		if (err)
			break;
			
		if (change & IFF_UP) {
		
			if (!(ifp->if_flags & IFF_RUNNING)) {
				DEBUG(FLAG_INIT, ("atuwi%d: calling "
				    "atuwi_init\n", sc->atuwi_unit));
				atuwi_init(sc);
			} else {
				DEBUG(FLAG_INIT, ("atuwi%d: calling "
				    "atuwi_stop\n", sc->atuwi_unit));
				atuwi_stop(sc);
			}
		
			if (ifp->if_flags & IFF_UP) {
				err = atuwi_switch_radio(sc, 1);
				break;
			} else {
				err = atuwi_switch_radio(sc, 0);
				break;
			}
		}
		
		/*
		 * TODO: should we do anything special if PROMISC is set?
		 *
		if (change & IFF_PROMISC) {
			if (ifp->if_flags & IFF_PROMISC)
				sc->atuwi_promisc = 1;
			else
				sc->atuwi_promisc = 0;
		}
		*/
		
		/*
		 * the following spaghetti has been replaced by what you see
		 * up here ^ ^
		 *
		if (ifp->if_flags & IFF_UP) {
	
			if (ifp->if_flags & IFF_RUNNING &&
			    ifp->if_flags & IFF_PROMISC &&
			    !(sc->atuwi_if_flags & IFF_PROMISC)) {
				sc->atuwi_rxfilt |= ATUWI_RXFILT_PROMISC;
				atuwi_setword(sc, ATUWI_CMD_SET_PKT_FILTER,
				    sc->atuwi_rxfilt);
			} else if (ifp->if_flags & IFF_RUNNING &&
			    !(ifp->if_flags & IFF_PROMISC) &&
			    sc->atuwi_if_flags & IFF_PROMISC) {
				sc->atuwi_rxfilt &= ~ATUWI_RXFILT_PROMISC;
				atuwi_setword(sc, ATUWI_CMD_SET_PKT_FILTER,
				    sc->atuwi_rxfilt);
			} else if (!(ifp->if_flags & IFF_RUNNING))
				atuwi_init(sc);
			
			DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl calling "
			    "atuwi_init()\n", sc->atuwi_unit));
			atuwi_init(sc);
			
		} else {
			if (ifp->if_flags & IFF_RUNNING)
				atuwi_stop(sc);
		}
		*/

		err = 0;
		break;
	case SIOCADDMULTI:
		err = 0;
		/* TODO: implement */
		DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl: add multi\n",
		    sc->atuwi_unit));
		break;
	case SIOCDELMULTI:
		/* TODO: implement */
		/* atuwi_setmulti(sc); */
		err = 0;
		DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl: del multi\n",
		     sc->atuwi_unit));
		break;
		
	case SIOCG80211:
		switch(ireq->i_type) {  
		case IEEE80211_IOC_SSID:
			err = copyout(sc->atuwi_ssid, ireq->i_data,
			    sc->atuwi_ssidlen);
			ireq->i_len = sc->atuwi_ssidlen;
			break;
			
		case IEEE80211_IOC_NUMSSIDS:
			ireq->i_val = 1;
			break;
		
		case IEEE80211_IOC_CHANNEL:
			ireq->i_val = sc->atuwi_channel;

			/*
			 * every time the channel is requested, we errr...
			 * print a bunch of debug things :)
			 */
			atuwi_print_a_bunch_of_debug_things(sc);
			break;
		
		case IEEE80211_IOC_AUTHMODE:
			/* TODO: change this when shared-key is implemented */
			ireq->i_val = IEEE80211_AUTH_OPEN;
			break;
		
		case IEEE80211_IOC_WEP:
			switch (sc->atuwi_encrypt) {
			case ATUWI_WEP_TX:
				ireq->i_val = IEEE80211_WEP_MIXED;
				break;
			case ATUWI_WEP_TXRX:
				ireq->i_val = IEEE80211_WEP_ON;
				break;
			default:
				ireq->i_val = IEEE80211_WEP_OFF;
			}
			break;
		
		case IEEE80211_IOC_NUMWEPKEYS:
			ireq->i_val = 4;
			break;
			
		case IEEE80211_IOC_WEPKEY:
			err = suser(td);
			if (err)
				break;

			if((ireq->i_val < 0) || (ireq->i_val > 3)) {
				err = EINVAL;
				break;
			}
			
			if (sc->atuwi_encrypt == ATUWI_WEP_40BITS)
				len = 5;
			else
				len = 13;
			
			err = copyout(sc->atuwi_wepkeys[ireq->i_val],
			    ireq->i_data, len);
			break;
		
		case IEEE80211_IOC_WEPTXKEY:
			ireq->i_val = sc->atuwi_wepkey;
			break;

		default:
			DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl:  unknown 80211: "
			    "%04x %d\n", sc->atuwi_unit, ireq->i_type,
			    ireq->i_type));
			err = EINVAL;
		}
		break;
	
	case SIOCS80211:
		err = suser(td);
		if (err)
			break;
	
		switch(ireq->i_type) {
		case IEEE80211_IOC_SSID:
			if (ireq->i_len < 0 || ireq->i_len > 32) {
				err = EINVAL;
				break;
			}
			
			err = copyin(ireq->i_data, tmp, ireq->i_len);
			if (err)
				break;
			
			sc->atuwi_ssidlen = ireq->i_len;
			memcpy(sc->atuwi_ssid, tmp, ireq->i_len);
			
			sc->atuwi_mgmt_flags |= ATUWI_CHANGED_SETTINGS;
			wakeup(sc);
			break;
			
		case IEEE80211_IOC_CHANNEL:
			if (ireq->i_val < 1 || ireq->i_val > 14) {
				err = EINVAL;
				break;
			}
			
			sc->atuwi_channel = ireq->i_val;

			/* restart scan / join / etc now */
			sc->atuwi_mgmt_flags |= ATUWI_CHANGED_SETTINGS;
			wakeup(sc);
			break;
		
		case IEEE80211_IOC_WEP:
			switch (ireq->i_val) {
			case IEEE80211_WEP_OFF:
				sc->atuwi_encrypt = ATUWI_WEP_OFF;
				break;
			case IEEE80211_WEP_MIXED:
				sc->atuwi_encrypt = ATUWI_WEP_TX;
				break;
			case IEEE80211_WEP_ON:
				sc->atuwi_encrypt = ATUWI_WEP_TXRX;
				break;
			default:
				err = EINVAL;
			}
			if (err)
				break;
			
			/*
			 * to change the wep-bit in our beacon we HAVE to send
			 * CMD_STARTUP again
			 */
			err = atuwi_initial_config(sc);
			/*
			 * after that we have to send CMD_JOIN again to get
			 * the receiver running again. so we'll just
			 * restart the entire join/assoc/auth state-machine.
			 */
			sc->atuwi_mgmt_flags |= ATUWI_CHANGED_SETTINGS;
			wakeup(sc);
			break;
			
		case IEEE80211_IOC_WEPKEY:
			if ((ireq->i_val < 0) || (ireq->i_val > 3) ||
			    (ireq->i_len > 13)) {
				err = EINVAL;
				break;
			}
			err = copyin(ireq->i_data, tmp, ireq->i_len);
			if (err)
				break;
			err = atuwi_set_wepkey(sc, ireq->i_val, tmp,
			    ireq->i_len);
			break;
			
		case IEEE80211_IOC_WEPTXKEY:
			if ((ireq->i_val < 0) || (ireq->i_val > 3)) {
				err = EINVAL;
				break;
			}
			sc->atuwi_wepkey = ireq->i_val;
			err = atuwi_send_mib(sc, MIB_MAC_WEP__KEY_ID,
			    NR(sc->atuwi_wepkey));

			break;
		
		case IEEE80211_IOC_AUTHMODE:
			/* TODO: change when shared-key is implemented */
			if (ireq->i_val != IEEE80211_AUTH_OPEN)
				err = EINVAL;
			break;
		
		default:
			err = EINVAL;
		}
		break;
	
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		err = ifmedia_ioctl(ifp, ifr, &sc->atuwi_media, command);
		break;
		
	case SIOCGWAVELAN:
		DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl: get wavelan\n",
		    sc->atuwi_unit));
		/*
		err = ether_ioctl(ifp, command, data);
		break;
		*/
		
		/* TODO: implement */

		err = copyin(ifr->ifr_data, &wreq, sizeof(wreq));
		if (err)
			break;
		
		DEBUG(FLAG_IOCTL, ("atuwi%d: SIOCGWAVELAN\n", sc->atuwi_unit));
		if (wreq.wi_len > WI_MAX_DATALEN) {
			err = EINVAL;
			break;
		}
		
		DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl: wi_type=%04x %d\n",
		    sc->atuwi_unit, wreq.wi_type, wreq.wi_type));
		err = 0;
		/* err = EINVAL; */
		break;
	
	case SIOCSWAVELAN:
		DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl: wavset type=%x\n",
		    sc->atuwi_unit, 0));
		err = 0;
		break;

	default:
		DEBUG(FLAG_IOCTL, ("atuwi%d: ioctl: default\n",
		    sc->atuwi_unit));
		err = ether_ioctl(ifp, command, data);
		break;
	}

	sc->atuwi_if_flags = ifp->if_flags;
	
	ATUWI_UNLOCK(sc);

	return(err);
}


Static void
atuwi_watchdog(struct ifnet *ifp)
{
	struct atuwi_softc	*sc;
	struct atuwi_chain	*c;
	usbd_status		stat;
	int			cnt;

	sc = ifp->if_softc;
	ATUWI_LOCK(sc);
	ifp->if_oerrors++;
	ERROR(("atuwi%d: watchdog timeout\n", sc->atuwi_unit));

	/*
	 * TODO:
	 * we should change this since we have multiple TX tranfers...
	 */
	for (cnt = 0; cnt < ATUWI_TX_LIST_CNT; cnt++) {
		c = &sc->atuwi_cdata.atuwi_tx_chain[cnt];
		if (c->atuwi_in_xfer) {
			usbd_get_xfer_status(c->atuwi_xfer, NULL, NULL, NULL,
			    &stat);
			atuwi_txeof(c->atuwi_xfer, c, stat);
		}
	}
	for (cnt = 0; cnt < ATUWI_MGMT_LIST_CNT; cnt++) {
		c = &sc->atuwi_cdata.atuwi_mgmt_chain[cnt];
		if (c->atuwi_in_xfer) {
			usbd_get_xfer_status(c->atuwi_xfer, NULL, NULL, NULL,
			    &stat);
			atuwi_txeof(c->atuwi_xfer, c, stat);
		}
	}

	if (ifp->if_snd.ifq_head != NULL)
		atuwi_start(ifp);
	ATUWI_UNLOCK(sc);

	return;
}


/*
 * Stop the adapter and free any mbufs allocated to the
 * RX and TX lists.
 */
Static void
atuwi_stop(struct atuwi_softc *sc)
{
	usbd_status		err;
	struct ifnet		*ifp;
	struct atuwi_cdata	*cd;


	ATUWI_LOCK(sc);

#ifdef ATUWI_PROFILING
		atuwi_profiling_stop++;
#endif /* ATUWI_PROFILING */

	ifp = &sc->arpcom.ac_if;
	ifp->if_timer = 0;

	DEBUG(FLAG_INIT, ("atuwi%d: atuwi_stop\n", sc->atuwi_unit));

	/* there must be a better way to clean up the mgmt task... */
	sc->atuwi_dying = 1;
	ATUWI_UNLOCK(sc);
	if (sc->atuwi_mgmt_flags & ATUWI_TASK_RUNNING) {
		DEBUG(FLAG_INIT, ("atuwi%d: waiting for mgmt task to die\n",
		    sc->atuwi_unit));
		wakeup(sc);
		while (sc->atuwi_dying == 1) {
			atuwi_msleep(sc, 100);
		}
	}
	ATUWI_LOCK(sc);
	DEBUG(FLAG_INIT, ("atuwi%d: stopped managment thread\n",
	    sc->atuwi_unit));


	/* Stop transfers. */
	if (sc->atuwi_ep[ATUWI_ENDPT_RX] != NULL) {
		err = usbd_abort_pipe(sc->atuwi_ep[ATUWI_ENDPT_RX]);
		if (err) {
			ERROR(("atuwi%d: abort rx pipe failed: %s\n",
			    sc->atuwi_unit, usbd_errstr(err)));
		}
		err = usbd_close_pipe(sc->atuwi_ep[ATUWI_ENDPT_RX]);
		if (err) {
			ERROR(("atuwi%d: close rx pipe failed: %s\n",
			    sc->atuwi_unit, usbd_errstr(err)));
		}
		sc->atuwi_ep[ATUWI_ENDPT_RX] = NULL;
	}

	if (sc->atuwi_ep[ATUWI_ENDPT_TX] != NULL) {
		err = usbd_abort_pipe(sc->atuwi_ep[ATUWI_ENDPT_TX]);
		if (err) {
			ERROR(("atuwi%d: abort tx pipe failed: %s\n",
			    sc->atuwi_unit, usbd_errstr(err)));
		}
		err = usbd_close_pipe(sc->atuwi_ep[ATUWI_ENDPT_TX]);
		if (err) {
			ERROR(("atuwi%d: close tx pipe failed: %s\n",
			    sc->atuwi_unit, usbd_errstr(err)));
		}
		sc->atuwi_ep[ATUWI_ENDPT_TX] = NULL;
	}

	/* Free RX/TX/MGMT list resources. */
	cd = &sc->atuwi_cdata;
	atuwi_xfer_list_free(sc, cd->atuwi_rx_chain, ATUWI_RX_LIST_CNT);
	atuwi_xfer_list_free(sc, cd->atuwi_tx_chain, ATUWI_TX_LIST_CNT);
	atuwi_xfer_list_free(sc, cd->atuwi_mgmt_chain, ATUWI_MGMT_LIST_CNT);

	/* Let's be nice and turn off the radio before we leave */
	atuwi_switch_radio(sc, 0);

	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	ATUWI_UNLOCK(sc);

	return;
}


/*
 * Stop all chip I/O so that the kernel's probe routines don't
 * get confused by errant DMAs when rebooting.
 */
Static void
atuwi_shutdown(device_ptr_t dev)
{
	struct atuwi_softc	*sc;

	sc = device_get_softc(dev);

	atuwi_stop(sc);

	return;
}



