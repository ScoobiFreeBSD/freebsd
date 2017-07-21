/*-
 * Copyright 2010, Gleb Smirnoff <glebius@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

/*
 *  http://www.eeti.com.tw/pdf/Software%20Programming%20Guide_v2.0.pdf
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/conf.h>
#include <sys/filio.h>	/* FIOSETOWN, FIOGETOWN */
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sigio.h>
#include <sys/signalvar.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbhid.h>
#include "usbdevs.h"

#include <sys/ioccom.h>
#include <sys/fcntl.h>
#include <sys/tty.h>

#ifdef EVDEV_SUPPORT
#include <dev/evdev/evdev.h>
#include <dev/evdev/input.h>
#endif

#define USB_DEBUG_VAR uep_debug
#include <dev/usb/usb_debug.h>

#ifdef USB_DEBUG
static int uep_debug = 0;

static SYSCTL_NODE(_hw_usb, OID_AUTO, uep, CTLFLAG_RW, 0, "USB uep");
SYSCTL_INT(_hw_usb_uep, OID_AUTO, debug, CTLFLAG_RWTUN,
    &uep_debug, 0, "Debug level");
#endif

#define UEP_MAX_X		2047
#define UEP_MAX_Y		2047

#define UEP_DOWN		0x01
#define UEP_PACKET_LEN_MAX	64
#define UEP_PACKET_LEN_REPORT	5
#define UEP_PACKET_LEN_REPORT2	6
#define UEP_PACKET_DIAG		0x0a
#define UEP_PACKET_TYPE_MASK		0xfe
#define UEP_PACKET_REPORT_MASK		0xe0
#define UEP_PACKET_REPORT		0x80
#define UEP_PACKET_MULTITOUCH		0x06
#define UEP_PACKET_LEN_MULTITOUCH	64
#define UEP_PACKET_REPORT_PRESSURE	0xc0
#define UEP_PACKET_REPORT_PLAYER	0xa0
#define	UEP_PACKET_LEN_MASK	

#define UEP_FIFO_BUF_SIZE	UEP_PACKET_LEN_MAX	/* bytes */
#define UEP_FIFO_QUEUE_MAXLEN	50	/* units */

enum {
	UEP_INIT_DT,
	UEP_INTR_DT,
	UEP_N_TRANSFER,
};

struct touch_event {
	int32_t x;
	int32_t y;
	int32_t width;	//!< Contact width
	int down;	//!< 1 = touch initiated; 0 = touch removed
	int timestamp;	//!< Relative event timestamp since touch initiated
};

struct uep_softc {
	struct mtx mtx;

	struct usb_xfer *xfer[UEP_N_TRANSFER];
	struct usb_fifo_sc fifo;

	u_int		pollrate;
	u_int		state;
#define UEP_ENABLED	0x01

	/* Reassembling buffer. */
	u_char		buf[UEP_PACKET_LEN_MAX];
	uint8_t		buf_len;
	struct sigio	*async;		/* Processes waiting for SIGIO */
#ifdef EVDEV_SUPPORT
	struct evdev_dev *evdev_a;	/* Absolute reporting device */
	struct evdev_dev *evdev_r;	/* Relative reporting device */
#endif
};

static usb_callback_t uep_init_callback;
static usb_callback_t uep_intr_callback;

static device_probe_t	uep_probe;
static device_attach_t	uep_attach;
static device_detach_t	uep_detach;

static usb_fifo_cmd_t	uep_start_read;
static usb_fifo_cmd_t	uep_stop_read;
static usb_fifo_open_t	uep_open;
static usb_fifo_close_t	uep_close;
static usb_fifo_ioctl_t uep_ioctl;

static void uep_put_queue(struct uep_softc *, u_char *, int len);

static struct usb_fifo_methods uep_fifo_methods = {
	.f_open = &uep_open,
	.f_close = &uep_close,
	.f_ioctl = &uep_ioctl,
	.f_start_read = &uep_start_read,
	.f_stop_read = &uep_stop_read,
	.basename[0] = "uep",
};

static int
get_pkt_len(u_char *buf)
{
	int len = 0;

	switch (buf[0] & UEP_PACKET_TYPE_MASK) {
		case UEP_PACKET_DIAG:
			len = buf[1] + 2;
			if (len > UEP_PACKET_LEN_MAX) {
				DPRINTF("bad packet len %u\n", len);
				return (UEP_PACKET_LEN_MAX);
			}
			break;

		case UEP_PACKET_REPORT:
			len = UEP_PACKET_LEN_REPORT;
			break;

		case UEP_PACKET_REPORT_PRESSURE:
		case UEP_PACKET_REPORT_PLAYER:
		case UEP_PACKET_REPORT_PRESSURE | UEP_PACKET_REPORT_PLAYER:
			len = UEP_PACKET_LEN_REPORT2;
			break;

		case UEP_PACKET_MULTITOUCH:
			len = UEP_PACKET_LEN_MULTITOUCH;
			break;

		default:
			break;
	}
	return len;
}

static void
uep_print_events(const struct touch_event *events, int num_events)
{
	int i;

	printf("%2d event%s:", num_events, num_events == 1 ? " " : "s");
	for (i = 0; i < num_events; ++i) {
		printf(" {x:%03x y:%03x w:%03x}",
				events[i].x,
				events[i].y,
				events[i].width);
		if (i == 0) {
			printf("(x:%03x y:%03x)", (events[i].x >> 1) & 0x7ff, (events[i].y >> 1) & 0x7ff);
		}
	}
	printf("\n");
}

static void
uep_process_pkt(struct uep_softc *sc, u_char *buf, int len)
{
	struct touch_event events[16];
	int num_events = 0;
	int i;

	switch((buf[0] & 0xFE)) {
	case  UEP_PACKET_REPORT:
		/*
		 * Packet format is 5 bytes:
		 *
		 * 1000000T
		 * 0000AAAA
		 * 0AAAAAAA
		 * 0000BBBB
		 * 0BBBBBBB
		 *
		 * T: 1=touched 0=not touched
		 * A: bits of axis A position, MSB to LSB
		 * B: bits of axis B position, MSB to LSB
		 *
		 * For the unit I have, which is CTF1020-S from CarTFT.com,
		 * A = X and B = Y. But in NetBSD uep(4) it is other way round :)
		 *
		 * The controller sends a stream of T=1 events while the
		 * panel is touched, followed by a single T=0 event.
		 *
		 */

		events[0].x = (buf[1] << 7) | buf[2];
		events[0].y = (buf[3] << 7) | buf[4];
		events[0].down = buf[0] & 0x01;
		uep_print_events(events, 1);
		break;

	case UEP_PACKET_MULTITOUCH:
		/*          BIT7 BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0
		 * BYTE 0:   0    0    0    0    0    1    1    0
		 * BYTE 1:  [        Number of fingers down       ]
		 *
		 * BYTE 2:   0    0    0    0    0    0    0   DOWN
		 * BYTE 3:  [           Touch ID                  ]
		 * BYTE 4:  X07  X06  X05  X04  X03  X02  X01  X00
		 * BYTE 5:   0    0    0    0   X11  X10  X09  X08
		 * BYTE 6:  Y07  Y06  Y05  Y04  Y03  Y02  Y01  Y00
		 * BYTE 7:   0    0    0    0   Y11  Y10  Y09  Y08
		 * BYTE 8:  W07  W06  W05  W04  W03  W02  W01  W00
		 * BYTE 9:   0    0    0    0   W11  W10  W09  W08
		 * BYTE10:   ?    ?    ?    ?    ?    ?    ?    ?
		 * BYTE11:   ?    ?    ?    ?    ?    ?    ?    ?
		 */
		num_events = buf[1];
		for (i = 0; i < num_events; ++i) {
			int offset = 2 + 10 * i;
			events[i].x = buf[offset + 2] + ((buf[offset + 3] & 0x0f) << 8);
			events[i].y = 0xfff - (buf[offset + 4] + ((buf[offset + 5] & 0x0f) << 8));
			events[i].width = buf[offset + 6] + ((buf[offset + 7] & 0x0f) << 8);
			events[i].down = buf[offset] & 0x01;

			/* Reformat packet to old protocol */
			if (i == 0) {
				buf[0] = 80 | events[i].down;
				buf[1] = (events[i].x >> 8) & 0x0f;
				buf[2] = (events[i].x & 0xfe) >> 1;
				buf[3] = (events[i].y >> 8) & 0x0f;
				buf[4] = (events[i].y & 0xfe) >> 1;
				len = 5;
			}
		}
		uep_print_events(events, num_events);

		break;
	default:
		DPRINTF("bad input packet format 0x%.2x\n", buf[0]);
		return;
	}
	uep_put_queue(sc, buf, len);
	if (sc->async != NULL) {
		pgsigio(&sc->async, SIGIO, 0);
	}
}

static void
uep_init_callback(struct usb_xfer *xfer, usb_error_t error)
{
	int len;
	usb_device_request_t req;
	struct usb_page_cache *pc;
	unsigned char buf[3];

	usbd_xfer_status(xfer, &len, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
		DPRINTF("Setting up init...\n");
		req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
		req.bRequest = 0;
		USETW(req.wValue, 0);
		USETW(req.wIndex, 0);
		USETW(req.wLength, sizeof(req));
	/*
	 * An eGalax diagnostic packet kicks the device into using the right
	 * protocol.  We send a "check active" packet.  The response will be
	 * read later and ignored.
	 */

#define EGALAX_PKT_TYPE_DIAG 0x0A
		buf[0] = EGALAX_PKT_TYPE_DIAG;
		buf[1] = 1;	/* length */
		buf[2] = 'A';	/* command - check active */

		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_in(pc, 0, &req, sizeof(req));
		pc = usbd_xfer_get_frame(xfer, 1);
		usbd_copy_in(pc, 0, buf, 3);

		usbd_xfer_set_frames(xfer, 2);
		usbd_xfer_set_frame_len(xfer, 0, sizeof(req));
		usbd_xfer_set_frame_len(xfer, 1, 3);
		usbd_transfer_submit(xfer);
		break;

	case USB_ST_TRANSFERRED:
	default:
		DPRINTF("Got state USB_ST_%s\n", USB_GET_STATE(xfer) == USB_ST_TRANSFERRED ? "TRANSFERRED" : "ERROR");
		break;
	}
}

static void
uep_intr_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct uep_softc *sc = usbd_xfer_softc(xfer);
	int len;

	usbd_xfer_status(xfer, &len, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
	    {
		struct usb_page_cache *pc;
		u_char buf[64], *p;
		int pkt_len;

		if (len > (int)sizeof(buf)) {
			DPRINTF("bad input length %d\n", len);
			goto tr_setup;
		}

		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, buf, len);

		/*
		 * The below code mimics Linux a lot. I don't know
		 * why NetBSD reads complete packets, but we need
		 * to reassamble 'em like Linux does (tries?).
		 */
		if (sc->buf_len > 0) {
			int res;

			if (sc->buf_len == 1)
				sc->buf[1] = buf[0];

			if ((pkt_len = get_pkt_len(sc->buf)) == 0)
				goto tr_setup;

			res = pkt_len - sc->buf_len;
			memcpy(sc->buf + sc->buf_len, buf, res);
			uep_process_pkt(sc, sc->buf, pkt_len);
			sc->buf_len = 0;

			p = buf + res;
			len -= res;
		} else
			p = buf;

		if (len == 1) {
			sc->buf[0] = buf[0];
			sc->buf_len = 1;

			goto tr_setup;
		}

		while (len > 0) {
			if ((pkt_len = get_pkt_len(p)) == 0)
				goto tr_setup;

			/* full packet: process */
			if (pkt_len <= len) {
				uep_process_pkt(sc, p, pkt_len);
			} else {
				/* incomplete packet: save in buffer */
				memcpy(sc->buf, p, len);
				sc->buf_len = len;
			}
			p += pkt_len;
			len -= pkt_len;
		}
	    }
	case USB_ST_SETUP:
	tr_setup:
		/* check if we can put more data into the FIFO */
		if (usb_fifo_put_bytes_max(sc->fifo.fp[USB_FIFO_RX]) != 0) {
			usbd_xfer_set_frame_len(xfer, 0,
			    usbd_xfer_max_len(xfer));
			usbd_transfer_submit(xfer);
                }
		break;

	default:
		DPRINTF("State=%s\n",
				USB_GET_STATE(xfer) == USB_ST_SETUP ? "USB_ST_SETUP" :
				USB_GET_STATE(xfer) == USB_ST_TRANSFERRED ? "USB_ST_TRANSFERRED" :
				"USB_ST_ERROR");
		if (error != USB_ERR_CANCELLED) {
			/* try clear stall first */
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		break;
	}
}


static const struct usb_config uep_config[UEP_N_TRANSFER] = {
	[UEP_INIT_DT] = {
		.type      = UE_CONTROL,
		.endpoint  = 0,
		.direction = UE_DIR_OUT,
		.bufsize   = 8,
		.callback  = &uep_init_callback,
		.interval  = 0,  /* no pre-delay */
	},
	[UEP_INTR_DT] = {
		.type      = UE_INTERRUPT,
		.endpoint  = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.flags = {
			.pipe_bof = 1,
			.short_xfer_ok = 1,
		},
		.bufsize   = 0,   /* use wMaxPacketSize */
		.callback  = &uep_intr_callback,
		.interval  = 0  /* no pre-delay */
	}
};

static const STRUCT_USB_HOST_ID uep_devs[] = {
	{USB_VPI(USB_VENDOR_EGALAX, USB_PRODUCT_EGALAX_TPANEL, 0)},
	{USB_VPI(USB_VENDOR_EGALAX, USB_PRODUCT_EGALAX_TPANEL2, 0)},
	{USB_VPI(USB_VENDOR_EGALAX, USB_PRODUCT_EGALAX_TPANEL3, 0)},
	{USB_VPI(USB_VENDOR_EGALAX2, USB_PRODUCT_EGALAX2_TPANEL, 0)},
};

static int
uep_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);
	if (uaa->info.bConfigIndex != 0)
		return (ENXIO);
	if (uaa->info.bIfaceIndex != 0)
		return (ENXIO);

	return (usbd_lookup_id_by_uaa(uep_devs, sizeof(uep_devs), uaa));
}

static int
uep_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uep_softc *sc = device_get_softc(dev);
	int error;

	device_set_usb_desc(dev);

	mtx_init(&sc->mtx, "uep lock", NULL, MTX_DEF);

	error = usbd_transfer_setup(uaa->device, &uaa->info.bIfaceIndex,
	    sc->xfer, uep_config, UEP_N_TRANSFER, sc, &sc->mtx);

	if (error) {
		DPRINTF("usbd_transfer_setup error=%s\n", usbd_errstr(error));
		goto detach;
	}

	error = usb_fifo_attach(uaa->device, sc, &sc->mtx, &uep_fifo_methods,
	    &sc->fifo, device_get_unit(dev), -1, uaa->info.bIfaceIndex,
	    UID_ROOT, GID_OPERATOR, 0644);

        if (error) {
		DPRINTF("usb_fifo_attach error=%s\n", usbd_errstr(error));
                goto detach;
        }

	sc->buf_len = 0;

	DPRINTF("Starting init control transfer.\n");
	usbd_transfer_start(sc->xfer[UEP_INIT_DT]);

	return (0);

detach:
	uep_detach(dev);

	return (ENOMEM); /* XXX */
}

static int
uep_detach(device_t dev)
{
	struct uep_softc *sc = device_get_softc(dev);

	usb_fifo_detach(&sc->fifo);

	usbd_transfer_unsetup(sc->xfer, UEP_N_TRANSFER);

	mtx_destroy(&sc->mtx);

	return (0);
}

static void
uep_start_read(struct usb_fifo *fifo)
{
	struct uep_softc *sc = usb_fifo_softc(fifo);
	u_int rate;

	if ((rate = sc->pollrate) > 1000)
		rate = 1000;

	if (rate > 0 && sc->xfer[UEP_INTR_DT] != NULL && !sc->async) {
		usbd_transfer_stop(sc->xfer[UEP_INTR_DT]);
		usbd_xfer_set_interval(sc->xfer[UEP_INTR_DT], 1000 / rate);
		sc->pollrate = 0;
	}

	if (!sc->async) {
		DPRINTF("Starting read transfer.\n");
		usbd_transfer_start(sc->xfer[UEP_INTR_DT]);
	}
}

static void
uep_stop_read(struct usb_fifo *fifo)
{
	struct uep_softc *sc = usb_fifo_softc(fifo);

	DPRINTF("Stopping read transfer.\n");
	usbd_transfer_stop(sc->xfer[UEP_INTR_DT]);
}

static void
uep_put_queue(struct uep_softc *sc, u_char *buf, int len)
{
	usb_fifo_put_data_linear(sc->fifo.fp[USB_FIFO_RX], buf, len, 1);
}

static int
uep_open(struct usb_fifo *fifo, int fflags)
{
	if (fflags & FREAD) {
		struct uep_softc *sc = usb_fifo_softc(fifo);

		if (sc->state & UEP_ENABLED)
			return (EBUSY);
		if (usb_fifo_alloc_buffer(fifo, UEP_FIFO_BUF_SIZE,
		    UEP_FIFO_QUEUE_MAXLEN))
			return (ENOMEM);

		sc->state |= UEP_ENABLED;
		sc->async = NULL;
	}

	return (0);
}

static void
uep_close(struct usb_fifo *fifo, int fflags)
{
	if (fflags & FREAD) {
		struct uep_softc *sc = usb_fifo_softc(fifo);

		sc->state &= ~(UEP_ENABLED);
		usb_fifo_free_buffer(fifo);
		if (sc->async != NULL) {
			funsetown(&sc->async);
			sc->async = NULL;
		}
	}
}

static int
uep_ioctl(struct usb_fifo *fifo, u_long cmd, void *addr, int fflags)
{
	struct uep_softc *sc = usb_fifo_softc(fifo);
	int error = 0;
	u_int rate;

	switch (cmd) {
	case FIOSETOWN:
		error = fsetown(*(int *)addr, &sc->async);
		if ((rate = sc->pollrate) > 1000)
			rate = 1000;
		if (rate > 0 && sc->xfer[UEP_INTR_DT] != NULL) {
			usbd_transfer_stop(sc->xfer[UEP_INTR_DT]);
			usbd_xfer_set_interval(sc->xfer[UEP_INTR_DT], 1000 / rate);
			sc->pollrate = 0;
		}
		usbd_transfer_start(sc->xfer[UEP_INTR_DT]);
		break;
	case FIOGETOWN:
		*(int *) addr = fgetown(&sc->async);
		break;
	default:
		break;
	}
	return error;
}


static devclass_t uep_devclass;

static device_method_t uep_methods[] = {
	DEVMETHOD(device_probe, uep_probe),
       	DEVMETHOD(device_attach, uep_attach),
	DEVMETHOD(device_detach, uep_detach),
	{ 0, 0 },
};

static driver_t uep_driver = {
	.name = "uep",
	.methods = uep_methods,
	.size = sizeof(struct uep_softc),
};

DRIVER_MODULE(uep, uhub, uep_driver, uep_devclass, NULL, NULL);
MODULE_DEPEND(uep, usb, 1, 1, 1);
MODULE_VERSION(uep, 1);
USB_PNP_HOST_INFO(uep_devs);
