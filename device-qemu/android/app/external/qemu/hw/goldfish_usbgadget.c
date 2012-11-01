/* Copyright (C) 2007-2008 The Android Open Source Project
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*/
#include "qemu-timer.h"
#include "cpu.h"
#include "arm_pic.h"
#include "goldfish_device.h"
#include "ch9.h"

#if 1 /* for next step */
static void deviceWrite(target_phys_addr_t regAddr, uint32_t value);
static uint32_t deviceRead(target_phys_addr_t regAddr);
#define REG(regAddr)		deviceRead(regAddr)
#define REGSET(regAddr, value)	deviceWrite(regAddr, value)
static void dbg(const char*  format, ... );
//#define dbg printf

/* register type definition */
#define UDC_REGMASK_ACCESS	0x0F
#define UDC_REGMASK_TYPE	0xF0
#define UDC_REGAC_READ		0x01
#define UDC_REGAC_WRITE		0x02
#define UDC_REGAC_RW		0x04
#define UDC_REGTYPE_NORMAL	0x10
#define UDC_REGTYPE_SFIFO	0x20	/* single fifo */
#define UDC_REGTYPE_DFIFO	0x40	/* double fifo */

/* register type accesor  */
#define UDC_REGAC(type)		((type)&UDC_REGMASK_ACCESS)
#define UDC_REGTYPE(type)	((type)&UDC_REGMASK_TYPE)

/* register definition */
#define MAX_REGISTERS			1024
#if 0
#define UDC_REGSIZE		(sizeof(target_phys_addr_t))
#define UDC_REGSIZE_BIT		(UDC_REGSIZE >> 2)
#else
#define UDC_REGSIZE_BIT		(0)
#endif
#define UDC_REG2OFF(regAddr)	(regAddr >> UDC_REGSIZE_BIT)
#define UDC_OFF2REG(off)	(off << UDC_REGSIZE_BIT)

struct udc_deviceRegister {
	int	type;	/* register type */
	void	*value; /* register value */
};
#define UDC_FIFO_SIZE	10240
struct udc_fifoValue {
	int count;	  /* remain size */
	int countRegAddr;
	int readOff;
	int writeOff;
	char values[UDC_FIFO_SIZE];
};
/* for cast access */
struct udc_doubleFifoValue {
	struct udc_fifoValue read;
	struct udc_fifoValue write;
};

#define OP_READ		0
#define OP_WRITE	1
#define FIFOSET(f, reg, op) do {					\
	if (UDC_REGTYPE((reg)->type) == UDC_REGTYPE_SFIFO) {		\
		(f) = (struct udc_fifoValue*)((reg)->value);		\
	} else {							\
		struct udc_doubleFifoValue *_p;				\
		_p = (struct udc_doubleFifoValue*)((reg)->value);	\
		if ((op) == OP_READ) {					\
			(f) = (struct udc_fifoValue*)(&_p->read);	\
		} else {						\
			(f) = (struct udc_fifoValue*)(&_p->write);	\
		}							\
	}								\
} while (0)


static char fifoRead(struct udc_fifoValue *fifo)
{
	char ret;
	if (fifo->count <= 0) {
		return 0;
	}

	ret = fifo->values[fifo->readOff];
	fifo->count--;
	if (fifo->countRegAddr) {
		REGSET(fifo->countRegAddr, fifo->count);
	}

	/* ringbuffer */
	if (fifo->readOff == (UDC_FIFO_SIZE - 1)) {
		fifo->readOff = 0;
	} else {
		fifo->readOff++;
	}
	return ret;
}
static void fifoWrite(struct udc_fifoValue *fifo, char value)
{
	dbg("fifoWrite:count=0x%x\n", fifo->count);
	if (fifo->count >= UDC_FIFO_SIZE) {
		return;
	}
	fifo->values[fifo->writeOff] = value;
	fifo->count++;
	dbg("fifoWrite:countRegAddr=0x%x\n", fifo->countRegAddr);
	if (fifo->countRegAddr) {
		REGSET(fifo->countRegAddr, fifo->count);
	}
	dbg("fifoWrite:countReg=0x%x\n", REG(fifo->countRegAddr));

	/* ringbuffer */
	if (fifo->writeOff == (UDC_FIFO_SIZE - 1)) {
		fifo->writeOff = 0;
	} else {
		fifo->writeOff++;
	}
}

static struct udc_deviceRegister udc_deviceRegister[MAX_REGISTERS];

/* register operations */

#define CHECK_OP_READ	0
#define CHECK_OP_WRITE	1
/*
 * ok: 1
 * ng: 0
 */
static int isAccessOK(int type, int op)
{
	int ret = 1;
	/* access check */
	switch (type) {
	case UDC_REGAC_RW:
		break;
	case UDC_REGAC_READ:
		if (op == CHECK_OP_WRITE) {
			ret = 0;
		}
		break;
	case UDC_REGAC_WRITE:
		if (op == CHECK_OP_READ) {
			ret = 0;
		}
		break;
	default:
		ret = 0; /* BUG */
	}
	return ret;
}
#if 1
/*
 * read/write　は、ドライバ側で設定するっぽい。不要なチェックか？
 */
#define GUESTOS_ACCESS_OK(type, op)	(1)
#else
#define GUESTOS_ACCESS_OK(type, op)	(isAccessOK(type, op))
#endif
static uint32_t guestOSRead(target_phys_addr_t regAddr)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	//dbg("guestOSRead:1:off=%d\n", off);
	if (!GUESTOS_ACCESS_OK(UDC_REGAC(udc_deviceRegister[off].type), CHECK_OP_READ)) {
		return 0;
	}
	//dbg("guestOSRead:2\n");
	if (UDC_REGTYPE(udc_deviceRegister[off].type) == UDC_REGTYPE_NORMAL) {
		return (uint32_t)udc_deviceRegister[off].value;
	}
	//dbg("guestOSRead:3\n");
	/* for fifo read */
	if (udc_deviceRegister[off].value == NULL) { /* sanity check */
		return 0;
	}
	//dbg("guestOSRead:4\n");
	FIFOSET(fifo, &udc_deviceRegister[off], OP_READ);
	return fifoRead(fifo);
}

static void guestOSWrite(target_phys_addr_t regAddr, uint32_t value)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	//dbg("guestOSWrite:regAddr=0x%x value=0x%x\n", regAddr, value);
	if (!GUESTOS_ACCESS_OK(UDC_REGAC(udc_deviceRegister[off].type), CHECK_OP_WRITE)) {
		return;
	}
	if (UDC_REGTYPE(udc_deviceRegister[off].type) == UDC_REGTYPE_NORMAL) {
		udc_deviceRegister[off].value = (void*)value;
		return;
	}
	/* for fifo write */
	if (udc_deviceRegister[off].value == NULL) { /* sanity check */
		return;
	}
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifoWrite(fifo, value);
	return;
}

static uint32_t deviceRead(target_phys_addr_t regAddr)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	if (UDC_REGTYPE(udc_deviceRegister[off].type) == UDC_REGTYPE_NORMAL) {
		return (uint32_t)udc_deviceRegister[off].value;
	}
	/* for fifo read */
	if (udc_deviceRegister[off].value == NULL) { /* sanity check */
		return 0;
	}
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	return fifoRead(fifo);
}


static void deviceWrite(target_phys_addr_t regAddr, uint32_t value)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	//dbg("deviceWrite:1\n");
	if (UDC_REGTYPE(udc_deviceRegister[off].type) == UDC_REGTYPE_NORMAL) {
		udc_deviceRegister[off].value = (void*)value;
		return;
	}
	//dbg("deviceWrite:2\n");
	/* for fifo write */
	if (udc_deviceRegister[off].value == NULL) { /* sanity check */
		return;
	}
	//dbg("deviceWrite:3\n");
	FIFOSET(fifo, &udc_deviceRegister[off], OP_READ);
	fifoWrite(fifo, value);
	return;
}

#endif

struct usbgadget_state {
    struct goldfish_device dev;
    QEMUTimer *timer;
};

static struct usbgadget_state usbgadget_state = {
    .dev = {
        .name = "goldfish_usbgadget",
        .id = -1,
        .size = 0x1000,
        .irq_count = 1,
    }
};

/****************** ホスト通信クラス ********************/
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "sockets.h"
static int sock_fd = -1;
static void hostEventHandler(void *opaque);

static int recvUsbData(char *bufp, int len)
{
	int ret;
	ret = socket_recv(sock_fd, bufp, len);
	if (ret <= 0) {
        	qemu_set_fd_handler(sock_fd, NULL, NULL, NULL);
		close(sock_fd);
		sock_fd = -1;
	}
	return ret;
}

static int sendUsbData(char *bufp, int len)
{
	int ret;
	ret = socket_send(sock_fd, bufp, len);
	return ret;
}

static int connectToHost(const char *path)
{
	int sock;
	struct sockaddr_un un;

	if (sock_fd != -1) {
		return;
	}

	dbg("connectToHost:%s\n", path);
	sock = socket(PF_UNIX, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("socket(unix)");
		return -1;
	}
	memset(&un, 0, sizeof(un));
	un.sun_family = AF_UNIX;
	snprintf(un.sun_path, sizeof(un.sun_path), "%s", path);
	if (connect(sock, (struct sockaddr*) &un, sizeof(un)) < 0) {
		dbg("connect(unix:%s): %s\n", path, strerror(errno));
		return -1;
	}
	sock_fd = sock;

	socket_set_nonblock(sock_fd);
        qemu_set_fd_handler(sock_fd, hostEventHandler, NULL, NULL);
	return 0;
}
static void disconnectToHost(void)
{
	if (sock_fd != -1) {
		dbg("close:sock_fd = %d\n", sock_fd);
		close(sock_fd);
		sock_fd = -1;
	}
}
/********************************************************/


/*********** (USB PROTCOL HEADER) ************************/
#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C

/*
 * USB directions
 *
 * This bit flag is used in endpoint descriptors' bEndpointAddress field.
 * It's also one of three fields in control requests bRequestType.
 */
#define USB_DIR_OUT                     0               /* to device */
#define USB_DIR_IN                      0x80            /* to host */
/*
 * USB types, the second of three bRequestType fields
 */
#define USB_TYPE_MASK                   (0x03 << 5)
#define USB_TYPE_STANDARD               (0x00 << 5)
#define USB_TYPE_CLASS                  (0x01 << 5)
#define USB_TYPE_VENDOR                 (0x02 << 5)
#define USB_TYPE_RESERVED               (0x03 << 5)

/*
 * USB recipients, the third of three bRequestType fields
 */
#define USB_RECIP_MASK                  0x1f
#define USB_RECIP_DEVICE                0x00
#define USB_RECIP_INTERFACE             0x01
#define USB_RECIP_ENDPOINT              0x02
#define USB_RECIP_OTHER                 0x03

/*
 * Descriptor types ... USB 2.0 spec table 9.5
 */
#define USB_DT_DEVICE                   0x01
#define USB_DT_CONFIG                   0x02
#define USB_DT_STRING                   0x03
#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05
#define USB_DT_DEVICE_QUALIFIER         0x06
#define USB_DT_OTHER_SPEED_CONFIG       0x07
#define USB_DT_INTERFACE_POWER          0x08
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DT_OTG                      0x09
#define USB_DT_DEBUG                    0x0a
#define USB_DT_INTERFACE_ASSOCIATION    0x0b
/* these are from the Wireless USB spec */
#define USB_DT_SECURITY                 0x0c
#define USB_DT_KEY                      0x0d
#define USB_DT_ENCRYPTION_TYPE          0x0e
#define USB_DT_BOS                      0x0f
#define USB_DT_DEVICE_CAPABILITY        0x10
#define USB_DT_WIRELESS_ENDPOINT_COMP   0x11
#define USB_DT_WIRE_ADAPTER             0x21
#define USB_DT_RPIPE                    0x22
#define USB_DT_CS_RADIO_CONTROL         0x23

typedef unsigned char __u8;
typedef unsigned short __le16;
#if 0
struct usb_ctrlrequest {
	__u8 bRequestType;
	__u8 bRequest;
	__le16 wValue;
	__le16 wIndex;
	__le16 wLength;
} __attribute__ ((packed));
#endif

/* token PID */
#define USB_PID_OUT		0x1
#define USB_PID_IN		0x9
#define USB_PID_SOF		0x5
#define USB_PID_SETUP		0xD

/* data PID */
#define USB_PID_DATA_0		0x3
#define USB_PID_DATA_1		0xB
#define USB_PID_DATA_2		0x7
#define USB_PID_MDATA		0xF

/* handshake PID */
#define USB_PID_ACK		0x2
#define USB_PID_NAK		0xA
#define USB_PID_STALL		0xC
#define USB_PID_NYET		0x6

struct usb_packet {
	__u8 pid;
	__u8 padding[3];
};
struct usb_packet_token {
	__u8 pid_token;
	__u8 endpoint;
	__u8 padding[2];
};
struct usb_packet_data {
	__u8 pid_token;
	__u8 endpoint;
	__u8 pid_data;
	__u8 padding;
	__le16 datalen;
	__u8 data[2];
};

struct usb_packet_handshake {
	__u8 pid_handshake;
	__u8 padding[3];
};
struct usb_packet_setup {
	__u8 pid_token;
	__u8 endpoint;
	__u8 pid_data;
	__u8 padding[1];
	struct usb_ctrlrequest req;	/* 8bytes */
};
struct usb_packet_setup_DescriptorRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
        __u8 data[1];
};
/*
 * buffer format
 * --------------------
 * 0-3  | bufferLen     <-- usb_packet_bufLen(自分自身のサイズは含まない)
 * --------------------
 * 4-4K | packetData    <-- usb_packet_bufp
 * --------------------
 */
static char usb_packet_buf[4096]; 
static int *usb_packet_bufLen = usb_packet_buf; /* 4bytes */
static struct usb_packet_setup_DescriptorRes *usb_packet_bufp = (struct usb_packet_setup_DescriptorRes *)&usb_packet_buf[4];
static struct usb_packet_data *usb_packet_datap = (struct usb_packet_data *)&usb_packet_buf[4];

struct usb_packet_setup_DescriptorDeviceRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
        struct usb_device_descriptor desc;
};

struct usb_packet_setup_DescriptorStringRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
        struct usb_string_descriptor desc;
};

/****************** USBプロトコル層 ********************/

static void putSetupData(struct usb_ctrlrequest *req);
static void putEndpData(int pid, int endpoint, const char* bufp, int len);
#if 1
struct usb_goldfish_cbw {
    uint32_t sig;
    uint32_t tag;
    uint32_t data_len;
    uint8_t flags;
    uint8_t lun;
    uint8_t cmd_len;
    uint8_t cmd[16];
};

#endif
/*
 * ホストPCからのUSBデータを受信する。
 */
static void hostEventHandler(void *opaque)
{
	int body_size;
	struct usb_packet *packet;
	char *bufp;
	static char buf[4096];
	int ret;

	dbg("hostEventHandler:enter\n");
	bufp = buf;
	/* read header */
	ret = recvUsbData(bufp, 1);
	if (ret <= 0) {
		dbg("hostEventHandler:head read err=%d\n", errno);
		return;
	}
	packet = (struct usb_packet*)bufp;
	switch (packet->pid) {
	case USB_PID_SETUP:
	{
		struct usb_packet_setup *packet_setup;
		struct usb_packet_handshake handshake;
		packet_setup = (struct usb_packet_setup *)bufp;
		body_size = sizeof(struct usb_packet_setup) - 1;
		bufp++;
		ret = recvUsbData(bufp, body_size);
		if (ret == body_size) {
			putSetupData(&packet_setup->req);
		} else {
			handshake.pid_handshake = USB_PID_NAK;
			dbg("hostEventHandler:body read ret = %d err=%d\n", ret, errno);
			ret = sendUsbData((char*)&handshake, sizeof(struct usb_packet_handshake));
			dbg("hostEventHandler:sendUsbData ret = %d err=%d\n", ret, errno);
		}
		break;
	}
	case USB_PID_OUT:
	case USB_PID_IN:
	{
		struct usb_packet_data *packet_data;
		struct usb_packet_handshake handshake;
		packet_data = (struct usb_packet_data *)bufp;
		bufp++;
		ret = recvUsbData(bufp, 4096);
		dbg("hostEventHandler:pid=0x%x ret = %d err=%d\n", packet->pid, ret, errno);
		if (ret <= 0) {
			handshake.pid_handshake = USB_PID_NAK;
			dbg("hostEventHandler:body read ret = %d err=%d\n", ret, errno);
			ret = sendUsbData((char*)&handshake, sizeof(struct usb_packet_handshake));
			dbg("hostEventHandler:sendUsbData ret = %d err=%d\n", ret, errno);
		} else {
#if 1
{
			struct usb_goldfish_cbw *cbw;
			cbw = (struct usb_goldfish_cbw*)packet_data->data;
			dbg("usb_goldfish_handle_data:datalen=%d\n", packet_data->datalen);
			if (packet_data->datalen) {
				dbg("usb_goldfish_handle_data:sig=0x%x\n", cbw->sig);
				dbg("usb_goldfish_handle_data:tag=0x%x\n", cbw->tag);
				dbg("usb_goldfish_handle_data:datalen=0x%x\n", cbw->data_len);
				dbg("usb_goldfish_handle_data:flags=0x%x\n", cbw->flags);
				dbg("usb_goldfish_handle_data:lun=0x%x\n", cbw->lun);
				dbg("usb_goldfish_handle_data:cmdlen=0x%x\n", cbw->cmd_len);
			}
}
#endif
			putEndpData(packet->pid, packet_data->endpoint, 
					packet_data->data, packet_data->datalen);
		}
		break;
	}
	case USB_PID_SOF:
		// TODO
		break;
	default:
		break;
	}
	return;
}

/********************************************************/

#if 1 /* TEST */
static void *console_client = NULL;
extern void  control_control_write(void* client, const char*  buff, int  len );

#if 0
static void dbg(const char*  format, ... )
{
    static char  temp[1024];
    va_list      args;

    va_start(args, format);
    vsnprintf( temp, sizeof(temp), format, args );
    va_end(args);

    temp[ sizeof(temp)-1 ] = 0;

    if (console_client)
    	control_control_write(console_client, temp, -1 );
}
#else
static void dbg(const char*  format, ... )
{
}
#endif

#define  GOLDFISH_TIMER_SAVE_VERSION  1


static void createGET_DESCRIPTOR(struct usb_ctrlrequest *req, __le16 wValue)
{
	req->bRequestType = 0;
	req->bRequestType |= USB_DIR_IN;
	req->bRequestType |= USB_TYPE_STANDARD;
	req->bRequestType |= USB_RECIP_DEVICE;
	req->bRequest = USB_REQ_GET_DESCRIPTOR;
	req->wValue = wValue;
	req->wIndex = 0;
	req->wLength = 0x12;
}
static void createSET_CONFIGURATION(struct usb_ctrlrequest *req, __le16 wValue)
{
	req->bRequestType = 0;
	req->bRequestType |= USB_DIR_IN;
	req->bRequestType |= USB_TYPE_STANDARD;
	req->bRequestType |= USB_RECIP_DEVICE;
	req->bRequest = USB_REQ_SET_CONFIGURATION;
	req->wValue = wValue;
	req->wIndex = 0;
	req->wLength = 0;
}

/*********** HERE(USB DEVICE REGISTER) ************************/
#define GOLDFISH_UDC_USB_INT_REG	0x0000 /* 0 */
#define GOLDFISH_UDC_EP_INT_REG         0x0001 /* 4 */

#define GOLDFISH_UDC_EP0_FIFO_CNT_REG	0x0010 /* 64  */
#define GOLDFISH_UDC_EP1_FIFO_CNT_REG	0x0011
#define GOLDFISH_UDC_EP2_FIFO_CNT_REG	0x0012
#define GOLDFISH_UDC_EP3_FIFO_CNT_REG	0x0013
#define GOLDFISH_UDC_EP4_FIFO_CNT_REG	0x0014
#define GOLDFISH_UDC_EP5_FIFO_CNT_REG	0x0015
#define GOLDFISH_UDC_EP6_FIFO_CNT_REG	0x0016
#define GOLDFISH_UDC_EP7_FIFO_CNT_REG	0x0017

#define GOLDFISH_UDC_EP0_FIFO_REG	0x0020
#define GOLDFISH_UDC_EP1_FIFO_REG	0x0021
#define GOLDFISH_UDC_EP2_FIFO_REG	0x0022
#define GOLDFISH_UDC_EP3_FIFO_REG	0x0023
#define GOLDFISH_UDC_EP4_FIFO_REG	0x0024
#define GOLDFISH_UDC_EP5_FIFO_REG	0x0025
#define GOLDFISH_UDC_EP6_FIFO_REG	0x0026
#define GOLDFISH_UDC_EP7_FIFO_REG	0x0027

#define GOLDFISH_UDC_EP0_CSR_REG	0x0030
#define GOLDFISH_UDC_EP1_CSR_REG	0x0031
#define GOLDFISH_UDC_EP2_CSR_REG	0x0032
#define GOLDFISH_UDC_EP3_CSR_REG	0x0033
#define GOLDFISH_UDC_EP4_CSR_REG	0x0034
#define GOLDFISH_UDC_EP5_CSR_REG	0x0035
#define GOLDFISH_UDC_EP6_CSR_REG	0x0036
#define GOLDFISH_UDC_EP7_CSR_REG	0x0037

#define GOLDFISH_UDC_INDEX_REG          0x0040
#define GOLDFISH_UDC_IN_CSR1_REG        0x0050
#define GOLDFISH_UDC_OUT_CSR1_REG       0x0060

#define GOLDFISH_UDC_ICSR1_PKTRDY       (1<<0)
#define GOLDFISH_UDC_OCSR1_PKTRDY       (1<<1)

/* 
 * GOLDFISH_UDC_USB_INT_REG 
 */
#define GOLDFISH_UDC_USB_INT_VBUS_ON    0x01
#define GOLDFISH_UDC_USB_INT_VBUS_OFF   0x02

/* 
 * GOLDFISH_UDC_EP_INT_REG 
 */
#define GOLDFISH_UDC_INT_EP0            (1<<0)
#define GOLDFISH_UDC_INT_EP1            (1<<1)
#define GOLDFISH_UDC_INT_EP2            (1<<2)
#define GOLDFISH_UDC_INT_EP3            (1<<3)
#define GOLDFISH_UDC_INT_EP4            (1<<4)

#define GOLDFISH_FIFOSIZE		1024
#define GOLDFISH_FIFO_NUM		16


#define GOLDFISH_UDC_EP0_CSR_OPKRDY      (1<<0)
#define GOLDFISH_UDC_EP0_CSR_IPKRDY      (1<<1)
#define GOLDFISH_UDC_EP0_CSR_SENTSTL     (1<<2)
#define GOLDFISH_UDC_EP0_CSR_DE          (1<<3)
#define GOLDFISH_UDC_EP0_CSR_SE          (1<<4)
#define GOLDFISH_UDC_EP0_CSR_SENDSTL     (1<<5)
#define GOLDFISH_UDC_EP0_CSR_SOPKTRDY    (1<<6)
#define GOLDFISH_UDC_EP0_CSR_SSE (1<<7)

/*
 * GOLDFISH_UDC_ADD_REG
 */
#define GOLDFISH_UDC_FUNC_ADDR_REG	0x100

/* flags */
#define GOLDFISH_UDC_FUNCADDR_UPDATE	0x10000000

/*********** HERE(USB DEVICE REGISTER) ************************/

static void sendData(target_phys_addr_t regAddr, char *p, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		//dbg("goldfish_usbgadget_test:i=%d\n", i);
		REGSET(regAddr, p[i]);
	}
}
static int recvFifoData(target_phys_addr_t regAddr, char *p)
{
	int i;
	int len;
	struct udc_fifoValue *fifo;
	FIFOSET(fifo, &udc_deviceRegister[regAddr], OP_WRITE);
	len = fifo->count;
	dbg("BEFORE:regAddr = %p len=%d\n", regAddr, len);
	for (i = 0; i < len; i++) {
		p[i] = REG(regAddr);
	}
	dbg("AFTER:len=%d\n", fifo->count);
	return len;
}

static void putSetupData(struct usb_ctrlrequest *req)
{
	dbg("putSetupData:req=0x%x\n", req);
	sendData(GOLDFISH_UDC_EP0_FIFO_REG, (char*)req, sizeof(*req));
	REGSET(GOLDFISH_UDC_EP_INT_REG, 
		REG(GOLDFISH_UDC_EP_INT_REG)|GOLDFISH_UDC_INT_EP0);
	goldfish_device_set_irq(&usbgadget_state.dev, 0, 1);
}
static void putEndpData(int pid, int endpoint, const char* bufp, int len)
{
	int fifo_reg = -1;
	int int_flag = -1;
	dbg("putEndpData:pid=0x%x endpoint=%d, bufp=0x%x, len=%d\n", pid, endpoint, bufp, len);
	dbg("putEndpData:GOLDFISH_UDC_EP_INT_REG:0x%x\n", REG(GOLDFISH_UDC_EP_INT_REG));
	switch (endpoint) {
		case 1:
			fifo_reg = GOLDFISH_UDC_EP1_FIFO_REG;
			int_flag = GOLDFISH_UDC_INT_EP1;
			break;
		case 2:
			fifo_reg = GOLDFISH_UDC_EP2_FIFO_REG;
			int_flag = GOLDFISH_UDC_INT_EP2;
			break;
		case 3:
			fifo_reg = GOLDFISH_UDC_EP3_FIFO_REG;
			int_flag = GOLDFISH_UDC_INT_EP3;
			break;
		case 4:
			fifo_reg = GOLDFISH_UDC_EP4_FIFO_REG;
			int_flag = GOLDFISH_UDC_INT_EP4;
			break;
		default:
			dbg("ERROR:invalid endpoint..\n");
			break;
	}
	if (fifo_reg != -1) {
		int baseAddr;
		if (pid == USB_PID_OUT) {
			sendData(fifo_reg, bufp, len);
			baseAddr = GOLDFISH_UDC_OUT_CSR1_REG + endpoint;
			REGSET(baseAddr, GOLDFISH_UDC_OCSR1_PKTRDY);
		} else {
			baseAddr = GOLDFISH_UDC_IN_CSR1_REG + endpoint;
			REGSET(baseAddr, GOLDFISH_UDC_OCSR1_PKTRDY);
			/* TODO: ここでfifoデータをチェックして、データがあるならそのまま返送する？*/
			/* でも、CBWをここで見るのか？？それに、常にCBWがあるとは限らない(DATA_INかも)のでは？？ */
		}
		REGSET(GOLDFISH_UDC_EP_INT_REG, 
			REG(GOLDFISH_UDC_EP_INT_REG)|int_flag);
		goldfish_device_set_irq(&usbgadget_state.dev, 0, 1);
		dbg("putEndpData:kick kernel:GOLDFISH_UDC_EP_INT_REG:0x%x\n", REG(GOLDFISH_UDC_EP_INT_REG));
	}
}


void goldfish_usbgadget_test(void *p, char *arg)
{
	struct usb_ctrlrequest reqData;
	struct usb_ctrlrequest *req;
	if (p) {
		console_client = p;
	}
	dbg("goldfish_usbgadget_test:irq=%d\n", usbgadget_state.dev.irq);

	req = &reqData;
	if (arg) {
		if (!strcmp(arg, "setconfig")) {
			createSET_CONFIGURATION(req, 1);
			sendData(GOLDFISH_UDC_EP0_FIFO_REG, (char*)req, sizeof(*req));
			REGSET(GOLDFISH_UDC_EP_INT_REG, 
				REG(GOLDFISH_UDC_EP_INT_REG)|GOLDFISH_UDC_INT_EP0);
		} else if (!strcmp(arg, "getdev")) {
			createGET_DESCRIPTOR(req, (USB_DT_DEVICE << 8));
			sendData(GOLDFISH_UDC_EP0_FIFO_REG, (char*)req, sizeof(*req));
			REGSET(GOLDFISH_UDC_EP_INT_REG, 
				REG(GOLDFISH_UDC_EP_INT_REG)|GOLDFISH_UDC_INT_EP0);
		} else if (!strcmp(arg, "vbuson")) {
			connectToHost("/tmp/vusb.sock");
			REGSET(GOLDFISH_UDC_USB_INT_REG, 
				REG(GOLDFISH_UDC_USB_INT_REG)|GOLDFISH_UDC_USB_INT_VBUS_ON);
			dbg("vbuson:reg=0x%x\n", REG(GOLDFISH_UDC_EP_INT_REG));
		} else if (!strcmp(arg, "vbusoff")) {
			disconnectToHost();
			REGSET(GOLDFISH_UDC_USB_INT_REG, 
				REG(GOLDFISH_UDC_USB_INT_REG)|GOLDFISH_UDC_USB_INT_VBUS_OFF);
			dbg("vbusoff:reg=0x%x\n", REG(GOLDFISH_UDC_EP_INT_REG));
		} else {
			dbg("not supported command\n");
		}
		goldfish_device_set_irq(&usbgadget_state.dev, 0, 1);
	}
}
#endif /* TEST */

static uint32_t goldfish_usbgadget_read(void *opaque, target_phys_addr_t offset)
{
	int ep_nr;
	struct usbgadget_state *s = (struct usbgadget_state *)opaque;

	dbg("goldfish_usbgadget_read:offset=0x%x\n", offset);
	if (offset == GOLDFISH_UDC_IN_CSR1_REG) { /* write fifo */
		ep_nr = REG(GOLDFISH_UDC_INDEX_REG);
		offset = offset + ep_nr;
	} else if (offset == GOLDFISH_UDC_OUT_CSR1_REG) { /* read fifo */
		ep_nr = REG(GOLDFISH_UDC_INDEX_REG);
		offset = offset + ep_nr;
	}
	return guestOSRead(offset);
}

static uint32_t getFifoAddr(int ep_nr)
{
	int addr = 0;
	switch (ep_nr) {
	case 1:
		addr = GOLDFISH_UDC_EP1_FIFO_REG;
		break;
	case 2:
		addr = GOLDFISH_UDC_EP2_FIFO_REG;
		break;
	case 3:
		addr = GOLDFISH_UDC_EP3_FIFO_REG;
		break;
	case 4:
		addr = GOLDFISH_UDC_EP4_FIFO_REG;
		break;
	case 5:
		addr = GOLDFISH_UDC_EP5_FIFO_REG;
		break;
	case 6:
		addr = GOLDFISH_UDC_EP6_FIFO_REG;
		break;
	case 7:
		addr = GOLDFISH_UDC_EP7_FIFO_REG;
		break;
	default:
		addr = GOLDFISH_UDC_EP0_FIFO_REG;
		break;
	}
	return addr;
}

static void goldfish_usbgadget_write(void *opaque, target_phys_addr_t offset, uint32_t value)
{
	int status;
	int int_status;
	int ep_status;
	struct usbgadget_state *s = (struct usbgadget_state *)opaque;
	dbg("goldfish_usbgadget_write:offset=0x%x value=0x%x\n", offset, value);

	if (offset == GOLDFISH_UDC_USB_INT_REG || 
		offset ==GOLDFISH_UDC_EP_INT_REG) {
		status = REG(offset);
		status &= ~value;
		dbg("goldfish_usbgadget_write:status=0x%x\n", status);
		guestOSWrite(offset, status);
		if (!REG(GOLDFISH_UDC_USB_INT_REG)
			&& !REG(GOLDFISH_UDC_EP_INT_REG)) {
			dbg("goldfish_usbgadget_write:irq clear\n");
			goldfish_device_set_irq(&usbgadget_state.dev, 0, 0);
		}
	} else if (offset == GOLDFISH_UDC_EP0_CSR_REG) {
		if (value & GOLDFISH_UDC_EP0_CSR_DE) {
			int len;
			int ret;
			value &= ~GOLDFISH_UDC_EP0_CSR_DE;
			usb_packet_bufp->pid_handshake = USB_PID_ACK;
			len = recvFifoData(GOLDFISH_UDC_EP0_FIFO_REG, &usb_packet_bufp->data[0]);
			*usb_packet_bufLen = sizeof(struct usb_packet_handshake) + len;
			ret = sendUsbData(usb_packet_buf, *usb_packet_bufLen + 4);
			dbg("sendUsbData ret = %d err=%d\n", ret, errno);
		}
		guestOSWrite(offset, value);
	} else if (offset == GOLDFISH_UDC_IN_CSR1_REG) { /* write fifo */
		int baseAddr;
		int regVal;
		int ep_nr;
		ep_nr = REG(GOLDFISH_UDC_INDEX_REG);
		baseAddr = offset + ep_nr;
		REGSET(baseAddr, value);
		if ((value & GOLDFISH_UDC_ICSR1_PKTRDY)) {
			int len;
			int ret;
			int fifoAddr;
			value &= ~(GOLDFISH_UDC_ICSR1_PKTRDY|GOLDFISH_UDC_OCSR1_PKTRDY);

			fifoAddr = getFifoAddr(ep_nr);
			usb_packet_datap->pid_token = USB_PID_ACK;
			len = recvFifoData(fifoAddr, &usb_packet_datap->data[0]);
			usb_packet_datap->datalen = len;
			*usb_packet_bufLen = (sizeof(struct usb_packet_data) - 2) + len;
			ret = sendUsbData(usb_packet_buf, *usb_packet_bufLen + 4);
			dbg("sendUsbData ret = %d err=%d\n", ret, errno);
		}
		guestOSWrite(baseAddr, value);
	} else if (offset == GOLDFISH_UDC_OUT_CSR1_REG) { /* read fifo */
		int baseAddr;
		int regVal;
		int ret;
		baseAddr = offset + REG(GOLDFISH_UDC_INDEX_REG);
		if (!(value & GOLDFISH_UDC_OCSR1_PKTRDY)) {
			usb_packet_bufp->pid_handshake = USB_PID_ACK;
			*usb_packet_bufLen = sizeof(struct usb_packet_handshake);
			ret = sendUsbData(usb_packet_buf, *usb_packet_bufLen + 4);
			dbg("sendUsbData ret = %d err=%d\n", ret, errno);
		}
		guestOSWrite(baseAddr, value);
	} else if (offset == GOLDFISH_UDC_FUNC_ADDR_REG) {
		if (value & GOLDFISH_UDC_FUNCADDR_UPDATE) {
			value &= ~GOLDFISH_UDC_FUNCADDR_UPDATE;
			dbg("setAddress:value=%d\n", value);
			guestOSWrite(offset, value);
		}
	} else {
		guestOSWrite(offset, value);
	}
}

static void  goldfish_usbgadget_save(QEMUFile*  f, void*  opaque)
{
	struct usbgadget_state*  s   = opaque;
	dbg("goldfish_usbgadget_save:enter\n");
}

static int  goldfish_usbgadget_load(QEMUFile*  f, void*  opaque, int  version_id)
{
	struct usbgadget_state*  s   = opaque;
	dbg("goldfish_usbgadget_load:version_id=%d\n", version_id);
	return 0;
}

static CPUReadMemoryFunc *goldfish_usbgadget_readfn[] = {
    goldfish_usbgadget_read,
    goldfish_usbgadget_read,
    goldfish_usbgadget_read
};

static CPUWriteMemoryFunc *goldfish_usbgadget_writefn[] = {
    goldfish_usbgadget_write,
    goldfish_usbgadget_write,
    goldfish_usbgadget_write
};

#define UDC_FIFO_NUM	10
static struct udc_fifoValue  udc_fifo[UDC_FIFO_NUM];
static void init_udc_deviceRegister(void)
{
	int i;
	int off;
	struct udc_fifoValue *fifo;
	for (i = 0; i < MAX_REGISTERS; i++) {
		udc_deviceRegister[i].type = UDC_REGAC_RW|UDC_REGTYPE_NORMAL;
		udc_deviceRegister[i].value = NULL;
	}
	/* EP0 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP0_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_DFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[0];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_READ);
	fifo->countRegAddr = GOLDFISH_UDC_EP0_FIFO_CNT_REG;
	/* EP1 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP1_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_SFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[2];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifo->countRegAddr = GOLDFISH_UDC_EP1_FIFO_CNT_REG;
	/* EP2 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP2_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_SFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[3];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifo->countRegAddr = GOLDFISH_UDC_EP2_FIFO_CNT_REG;
	/* EP3 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP3_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_SFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[4];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifo->countRegAddr = GOLDFISH_UDC_EP3_FIFO_CNT_REG;
	/* EP4 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP4_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_SFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[5];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifo->countRegAddr = GOLDFISH_UDC_EP4_FIFO_CNT_REG;
	/* EP5 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP5_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_SFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[6];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifo->countRegAddr = GOLDFISH_UDC_EP5_FIFO_CNT_REG;
	/* EP6 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP6_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_SFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[7];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifo->countRegAddr = GOLDFISH_UDC_EP6_FIFO_CNT_REG;
	/* EP7 */
	off = UDC_REG2OFF(GOLDFISH_UDC_EP7_FIFO_REG);
	udc_deviceRegister[off].type = UDC_REGAC_RW|UDC_REGTYPE_SFIFO;
	udc_deviceRegister[off].value = (void*)&udc_fifo[8];
	FIFOSET(fifo, &udc_deviceRegister[off], OP_WRITE);
	fifo->countRegAddr = GOLDFISH_UDC_EP7_FIFO_CNT_REG;

}
void goldfish_usbgadget_init(uint32_t usbgadgetbase, int usbgadgetirq)
{
	init_udc_deviceRegister();
	usbgadget_state.dev.base = usbgadgetbase;
	usbgadget_state.dev.irq = usbgadgetirq;
	goldfish_device_add(&usbgadget_state.dev, 
				goldfish_usbgadget_readfn, 
				goldfish_usbgadget_writefn, 
				&usbgadget_state);
	register_savevm( "goldfish_usbgadget", 0, 
			GOLDFISH_TIMER_SAVE_VERSION,
			goldfish_usbgadget_save, 
			goldfish_usbgadget_load, 
			&usbgadget_state);
}

