/*
 * USB Mass Storage Device emulation
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the LGPL.
 */

#include "qemu-common.h"
#include "qemu-option.h"
#include "qemu-config.h"
#include "hw/usb.h"
#include "hw/usb/desc.h"
#include "hw/scsi.h"
#include "console.h"
#include "monitor.h"
#include "sysemu.h"
#include "blockdev.h"

//#define DEBUG_MSD

#include "ch9.h"

#ifdef DEBUG_MSD
#define DPRINTF(fmt, ...) \
do { printf("usb-msd: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

/* USB requests.  */
#define MassStorageReset  0xff
#define GetMaxLun         0xfe

enum USBMSDMode {
    USB_MSDM_CBW, /* Command Block.  */
    USB_MSDM_DATAOUT, /* Transfer data to device.  */
    USB_MSDM_DATAIN, /* Transfer data from device.  */
    USB_MSDM_CSW /* Command Status.  */
};

struct usb_goldfish_csw {
    uint32_t sig;
    uint32_t tag;
    uint32_t residue;
    uint8_t status;
};

typedef struct {
    USBDevice dev;
    enum USBMSDMode mode;
    uint32_t scsi_off;
    uint32_t scsi_len;
    uint32_t data_len;
    struct usb_goldfish_csw csw;
    SCSIRequest *req;
    SCSIBus bus;
    BlockConf conf;
    char *serial;
    SCSIDevice *scsi_dev;
    uint32_t removable;
    /* For async completion.  */
    USBPacket *packet;
} MSDState;

struct usb_goldfish_cbw {
    uint32_t sig;
    uint32_t tag;
    uint32_t data_len;
    uint8_t flags;
    uint8_t lun;
    uint8_t cmd_len;
    uint8_t cmd[16];
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
    STR_CONFIG_FULL,
    STR_CONFIG_HIGH,
};

static const USBDescStrings desc_strings = {
    [STR_MANUFACTURER] = "QEMU",
    [STR_PRODUCT]      = "QEMU USB GOLDFISH",
    [STR_SERIALNUMBER] = "1",
    [STR_CONFIG_FULL]  = "Full speed config (usb 1.1)",
    [STR_CONFIG_HIGH]  = "High speed config (usb 2.0)",
};

static const USBDescIface desc_iface_full = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_MASS_STORAGE,
    .bInterfaceSubClass            = 0x06, /* SCSI */
    .bInterfaceProtocol            = 0x50, /* Bulk */
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | 0x01,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 64,
        },{
            .bEndpointAddress      = USB_DIR_OUT | 0x02,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 64,
        },
    }
};

static const USBDescDevice desc_device_full = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 8,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .iConfiguration        = STR_CONFIG_FULL,
            .bmAttributes          = 0xc0,
            .nif = 1,
            .ifs = &desc_iface_full,
        },
    },
};

static const USBDescIface desc_iface_high = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_MASS_STORAGE,
    .bInterfaceSubClass            = 0x06, /* SCSI */
    .bInterfaceProtocol            = 0x50, /* Bulk */
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | 0x01,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        },{
            .bEndpointAddress      = USB_DIR_OUT | 0x02,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        },
    }
};

static const USBDescDevice desc_device_high = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .iConfiguration        = STR_CONFIG_HIGH,
            .bmAttributes          = 0xc0,
            .nif = 1,
            .ifs = &desc_iface_high,
        },
    },
};

static const USBDesc desc = {
    .id = {
        .idVendor          = 0x46f4, /* CRC16() of "QEMU" */
        .idProduct         = 0x0001,
        .bcdDevice         = 0,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full = &desc_device_full,
    .high = &desc_device_high,
    .str  = desc_strings,
};

static void usb_goldfish_copy_data(MSDState *s, USBPacket *p)
{
    uint32_t len;
    len = p->iov.size - p->result;
DPRINTF("usb_goldfish_copy_data:enter\n");
    if (len > s->scsi_len)
        len = s->scsi_len;
    usb_packet_copy(p, scsi_req_get_buf(s->req) + s->scsi_off, len);
    s->scsi_len -= len;
    s->scsi_off += len;
    s->data_len -= len;
    if (s->scsi_len == 0 || s->data_len == 0) {
        scsi_req_continue(s->req);
    }
DPRINTF("usb_goldfish_copy_data:exit\n");
}

static void usb_goldfish_send_status(MSDState *s, USBPacket *p)
{
    int len;

    DPRINTF("Command status %d tag 0x%x, len %zd\n",
            s->csw.status, le32_to_cpu(s->csw.tag), p->iov.size);

    assert(s->csw.sig == cpu_to_le32(0x53425355));
    len = MIN(sizeof(s->csw), p->iov.size);
    usb_packet_copy(p, &s->csw, len);
    memset(&s->csw, 0, sizeof(s->csw));
    DPRINTF("Command status:exit\n");
}

static void usb_goldfish_packet_complete(MSDState *s)
{
    USBPacket *p = s->packet;

DPRINTF("usb_goldfish_packet_complete:enter\n");
    /* Set s->packet to NULL before calling usb_packet_complete
       because another request may be issued before
       usb_packet_complete returns.  */
    DPRINTF("Packet complete %p\n", p);
    s->packet = NULL;
    usb_packet_complete(&s->dev, p);
DPRINTF("usb_goldfish_packet_complete:exit\n");
}

static void usb_goldfish_transfer_data(SCSIRequest *req, uint32_t len)
{
    MSDState *s = DO_UPCAST(MSDState, dev.qdev, req->bus->qbus.parent);
    USBPacket *p = s->packet;
DPRINTF("usb_goldfish_transfer_data:enter\n");

    assert((s->mode == USB_MSDM_DATAOUT) == (req->cmd.mode == SCSI_XFER_TO_DEV));
    s->scsi_len = len;
    s->scsi_off = 0;
    if (p) {
        usb_goldfish_copy_data(s, p);
        p = s->packet;
        if (p && p->result == p->iov.size) {
            usb_goldfish_packet_complete(s);
        }
    }
DPRINTF("usb_goldfish_transfer_data:exit\n");
}

static void usb_goldfish_command_complete(SCSIRequest *req, uint32_t status, size_t resid)
{
    MSDState *s = DO_UPCAST(MSDState, dev.qdev, req->bus->qbus.parent);
    USBPacket *p = s->packet;

    DPRINTF("Command complete %d tag 0x%x\n", status, req->tag);

    s->csw.sig = cpu_to_le32(0x53425355);
    s->csw.tag = cpu_to_le32(req->tag);
    s->csw.residue = cpu_to_le32(s->data_len);
    s->csw.status = status != 0;

    if (s->packet) {
        if (s->data_len == 0 && s->mode == USB_MSDM_DATAOUT) {
            /* A deferred packet with no write data remaining must be
               the status read packet.  */
            usb_goldfish_send_status(s, p);
            s->mode = USB_MSDM_CBW;
        } else if (s->mode == USB_MSDM_CSW) {
            usb_goldfish_send_status(s, p);
            s->mode = USB_MSDM_CBW;
        } else {
            if (s->data_len) {
                int len = (p->iov.size - p->result);
                usb_packet_skip(p, len);
                s->data_len -= len;
            }
            if (s->data_len == 0) {
                s->mode = USB_MSDM_CSW;
            }
        }
        usb_goldfish_packet_complete(s);
    } else if (s->data_len == 0) {
        s->mode = USB_MSDM_CSW;
    }
    scsi_req_unref(req);
    s->req = NULL;
    DPRINTF("Command complete:exit\n");
}

static void usb_goldfish_request_cancelled(SCSIRequest *req)
{
    MSDState *s = DO_UPCAST(MSDState, dev.qdev, req->bus->qbus.parent);

DPRINTF("usb_goldfish_request_cancelled:enter\n");
    if (req == s->req) {
        scsi_req_unref(s->req);
        s->req = NULL;
        s->scsi_len = 0;
    }
DPRINTF("usb_goldfish_request_cancelled:exit\n");
}

static void usb_goldfish_handle_reset(USBDevice *dev)
{
    MSDState *s = (MSDState *)dev;

    DPRINTF("Reset\n");
    if (s->req) {
        scsi_req_cancel(s->req);
    }
    assert(s->req == NULL);

    if (s->packet) {
        s->packet->result = USB_RET_STALL;
        usb_goldfish_packet_complete(s);
    }

    s->mode = USB_MSDM_CBW;
    DPRINTF("usb_goldfish_handle_reset:exit\n");
}

#if 1
/* token PID */
#define USB_PID_OUT             0x1
#define USB_PID_IN              0x9
#define USB_PID_SOF             0x5
#define USB_PID_SETUP           0xD

/* data PID */
#define USB_PID_DATA_0          0x3
#define USB_PID_DATA_1          0xB
#define USB_PID_DATA_2          0x7
#define USB_PID_MDATA           0xF

/* handshake PID */
#define USB_PID_ACK             0x2
#define USB_PID_NAK             0xA
#define USB_PID_STALL           0xC
#define USB_PID_NYET            0x6
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
        struct usb_ctrlrequest req;     /* 8bytes */
};

struct usb_packet_setup_DescriptorRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
        __u8 data[1];
};
struct usb_packet_setup_DescriptorDeviceRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
	struct usb_device_descriptor desc;
};
static void printDevDesc(struct usb_device_descriptor *dp)
{
	DPRINTF("*****printDevDesc*******\n");
	DPRINTF("bLength=%d\n", dp->bLength);
	DPRINTF("bDescriptorType=0x%x\n", dp->bDescriptorType);
	DPRINTF("bcdUSB=0x%x\n", dp->bcdUSB);
	DPRINTF("bDeviceClass=0x%x\n", dp->bDeviceClass);
	DPRINTF("bDeviceSubClass=0x%x\n", dp->bDeviceSubClass);
	DPRINTF("bDeviceProtocol=0x%x\n", dp->bDeviceProtocol);
	DPRINTF("bMaxPacketSize0=0x%x\n", dp->bMaxPacketSize0);
	DPRINTF("idVendor=0x%x\n", dp->idVendor);
	DPRINTF("idProduct=0x%x\n", dp->idProduct);
	DPRINTF("bcdDevice=0x%x\n", dp->bcdDevice);
	DPRINTF("iManufacturer=0x%x\n", dp->iManufacturer);
	DPRINTF("iProduct=0x%x\n", dp->iProduct);
	DPRINTF("iSerialNumber=0x%x\n", dp->iSerialNumber);
	DPRINTF("bNumConfigurations=0x%x\n", dp->bNumConfigurations);
}

#define PDESC(str, dp, member) 				\
	DPRINTF("%s: %s : 0x%x\n", str, #member, dp->member)

static void printConfigDesc(struct usb_config_descriptor *dp)
{
	int i_inf;
	int i_ep;
	struct usb_interface_descriptor *idp = NULL;
	struct usb_endpoint_descriptor *ep = NULL;

	PDESC("Config", dp, bLength);
	PDESC("Config", dp, bDescriptorType);
        PDESC("Config", dp, wTotalLength);
        PDESC("Config", dp, bNumInterfaces);
        PDESC("Config", dp, bConfigurationValue);
        PDESC("Config", dp, iConfiguration);
        PDESC("Config", dp, bmAttributes);
        PDESC("Config", dp,bMaxPower);
	idp = ++dp; /* TODO */
	for (i_inf = 0; i_inf < dp->bNumInterfaces; i_inf++) {
		PDESC("Inf", idp, bLength);
		PDESC("Inf", idp, bDescriptorType);
		PDESC("Inf", idp, bInterfaceNumber);
		PDESC("Inf", idp, bAlternateSetting);
		PDESC("Inf", idp, bNumEndpoints);
		PDESC("Inf", idp, bInterfaceClass);
		PDESC("Inf", idp, bInterfaceSubClass);
		PDESC("Inf", idp, bInterfaceProtocol);
		PDESC("Inf", idp, iInterface);
		ep = (idp + 1); /* TODO */
		for (i_ep = 0; i_ep < idp->bNumEndpoints; i_ep++) {
			PDESC("Endp", ep, bLength);
			PDESC("Endp", ep, bDescriptorType);
			PDESC("Endp", ep, bEndpointAddress);
			PDESC("Endp", ep, bmAttributes);
			PDESC("Endp", ep, wMaxPacketSize);
			PDESC("Endp", ep, bInterval);
			ep = (((char*)ep) + ep->bLength);
		}
		idp = ep;
	}
}

static void printStringDesc(struct usb_string_descriptor *dp)
{
	int i;
	PDESC("String", dp, bLength);
	PDESC("String", dp, bDescriptorType);
	for (i = 0; i < dp->bLength - 2; i ++) {
		PDESC("String", dp, wData[i]);
	}
}
struct usb_packet_function {
	struct usb_interface_descriptor inf_desc;
	/* ここから可変 */
	struct usb_endpoint_descriptor ep_desc[1];
};
struct usb_packet_setup_DescriptorConfigRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
	struct usb_config_descriptor config_desc;
	/* ここから可変 */
	struct usb_packet_function functions[1];
};
struct usb_packet_setup_DescriptorStringRes {
        __u8 pid_handshake;
        __u8 pid_data;
	struct usb_string_descriptor desc;
};

static void createSET_INTERFACE(struct usb_ctrlrequest *req, 
	__le16 wValue, __le16 wIndex, __le16 wLength)
{
        req->bRequestType = USB_RECIP_INTERFACE;
        req->bRequest = USB_REQ_SET_INTERFACE;
        req->wValue = wValue;
        req->wIndex = wIndex;
        req->wLength = wLength;
}

static void createSET_ADDRESS(struct usb_ctrlrequest *req,
	__le16 wValue, __le16 wIndex, __le16 wLength)
{
        req->bRequestType = 0;
        req->bRequest = USB_REQ_SET_ADDRESS;
        req->wValue = wValue;
        req->wIndex = wIndex;
        req->wLength = wLength;
}

static void createSET_CONFIGURATION(struct usb_ctrlrequest *req, 
	__le16 wValue, __le16 wIndex, __le16 wLength)
{
        req->bRequestType = 0;
        req->bRequest = USB_REQ_SET_CONFIGURATION;
        req->wValue = wValue;
        req->wIndex = wIndex;
        req->wLength = wLength;
}
static void createGET_DESCRIPTOR(struct usb_ctrlrequest *req, 
	__le16 wValue, __le16 wIndex, __le16 wLength)
{
        req->bRequestType = 0;
        req->bRequestType |= USB_DIR_IN;
        req->bRequestType |= USB_TYPE_STANDARD;
        req->bRequestType |= USB_RECIP_DEVICE;
        req->bRequest = USB_REQ_GET_DESCRIPTOR;
        req->wValue = wValue;
        req->wIndex = wIndex;
        req->wLength = wLength;
}

static int client_fd = -1;
#if 1
static int sendRecvPacket(const char *sendBufp, int sendLen, 
				char *recvBufp, int recvLen)
{
	int ret = 0;
	int len = 0;
	ret = qemu_send_full(client_fd, sendBufp, sendLen, 0);
	DPRINTF("qemu_send_full():ret=%d\n", ret);


	if (ret == sendLen) {
		ret = qemu_recv_full(client_fd, &len, 4, 0);
		DPRINTF("sendRecvPacket:ret =%d len = %d errno=%d\n", ret, len, errno);
		if (ret != 4) {
			ret = -1;
		} else {
			ret = qemu_recv_full(client_fd, 
				recvBufp, len, 0);
			if (ret == len) {
				ret = 0;
			} else {
				ret = -1;
			}
		}
	} else {
		ret = -1;
	}
	return ret;
}
static int sendRecvDataPacket(int pid, int endpoint, 
			const char* sendBufp, int sendLen,
				char *recvBufp, int recvLen)
{
	int ret;
	static char sendBuf[4096];
	struct usb_packet_data *packetp;
	struct usb_packet_handshake *resp;

	packetp = (struct usb_packet_data *)sendBuf;
	resp = (struct usb_packet_handshake *)recvBufp;

	packetp->pid_token = pid;
	packetp->endpoint = endpoint;
	packetp->datalen = sendLen;
	memcpy(packetp->data, sendBufp, sendLen);
	DPRINTF("sendRecvDataPacket:pid=%d\n", packetp->pid_token);
	DPRINTF("sendRecvDataPacket:endp=%d\n", packetp->endpoint);
	DPRINTF("sendRecvDataPacket:datalen=%d\n", packetp->datalen);

	ret = sendRecvPacket((const char*)packetp, (6 + sendLen),
			recvBufp, recvLen);
	// need to check caller IN/OUT
	switch (pid) {
	case USB_PID_IN:
		// check ack
		// get data
		break;
	case USB_PID_OUT:
		// only ack...
		break;
	defaults:
		break;
	}
	DPRINTF("sendRecvDataPacket():ret=%d pid=0x%x\n", 
			ret, resp->pid_handshake);
	return 0;
}
static void sendRecvSetupPacket(int request,
	__le16 wValue, __le16 wIndex, __le16 wLength, 
	struct usb_packet_setup_DescriptorDeviceRes **data)
{
	static char recvBuf[4096];
	struct usb_packet_setup packet;
	struct usb_packet_handshake *resp;
	int ret;

	DPRINTF("sendRecvSetupPacket:request=0x%x\n", request);
	packet.pid_token = USB_PID_SETUP;
	switch (request) {
	case USB_REQ_SET_CONFIGURATION:
	{
		struct usb_packet_handshake res;
		resp = (struct usb_packet_handshake *)&res;
		createSET_CONFIGURATION(&packet.req, wValue, wIndex, wLength);
		ret = sendRecvPacket((const char*)&packet, sizeof(packet),
				(char*)&res, sizeof(res));
		break;
	}
	case USB_REQ_SET_ADDRESS:
	{
		struct usb_packet_handshake res;
		resp = (struct usb_packet_handshake *)&res;
		createSET_ADDRESS(&packet.req, wValue, wIndex, wLength);
		ret = sendRecvPacket((const char*)&packet, sizeof(packet),
				(char*)&res, sizeof(res));
		break;
	}
	case USB_REQ_SET_INTERFACE:
	{
		struct usb_packet_handshake res;
		resp = (struct usb_packet_handshake *)&res;
		createSET_INTERFACE(&packet.req, wValue, wIndex, wLength);
		ret = sendRecvPacket((const char*)&packet, sizeof(packet),
				(char*)&res, sizeof(res));
		break;
	}
	case USB_REQ_GET_DESCRIPTOR:
	{
		static struct usb_packet_setup_DescriptorDeviceRes *res;
		
		resp = (struct usb_packet_handshake *)recvBuf;
		res = (struct usb_packet_setup_DescriptorDeviceRes *)recvBuf;
		if (wValue ==  (USB_DT_CONFIG << 8)) {
			createGET_DESCRIPTOR(&packet.req, wValue, wIndex, wLength);
		} else if (wValue ==  (USB_DT_STRING << 8)) {
			createGET_DESCRIPTOR(&packet.req, wValue, wIndex, wLength);
		} else if (wValue ==  (USB_DT_DEVICE << 8)) {
			//createGET_DESCRIPTOR(&packet.req, wValue, 0x12);
			createGET_DESCRIPTOR(&packet.req, wValue, wIndex, wLength);
		}
		ret = sendRecvPacket((const char*)&packet, sizeof(packet),
				recvBuf, 4096);
		if (wValue ==  (USB_DT_CONFIG << 8)) {
			printConfigDesc(&res->desc);
		} else if (wValue ==  (USB_DT_STRING << 8)) {
			printStringDesc(&res->desc);
		} else if (wValue ==  (USB_DT_DEVICE << 8)) {
			printDevDesc(&res->desc);
		}
		*data = (struct usb_packet_setup_DescriptorDeviceRes *)recvBuf;
		break;
	}
	defaults:
		break;
	}
	DPRINTF("qemu_recv_full():ret=%d pid=0x%x\n", 
			ret, resp->pid_handshake);
}
#endif

#endif

/*
 * static void sendRecvSetupPacket(int request,
 *	__le16 wValue, __le16 wIndex, __le16 wLength)
 */
static int usb_goldfish_handle_control(USBDevice *dev, USBPacket *p,
               int request, int value, int index, int length, uint8_t *data)
{
    MSDState *s = (MSDState *)dev;
    int ret;

DPRINTF("usb_goldfish_handle_control:enter:request=0x%x\n", request);
    
    switch (request & 0xFF) {
    case USB_REQ_SET_ADDRESS:
    case USB_REQ_SET_INTERFACE:
    case USB_REQ_SET_CONFIGURATION:
    	sendRecvSetupPacket((request & 0xFF), value, index, length, NULL);
	break;
    defaults:
	break;
    }
    ret = usb_desc_handle_control(dev, p, request, value, index, length, data);
    if (ret >= 0) {
        return ret;
    }

    ret = 0;
    switch (request) {
    case EndpointOutRequest | USB_REQ_CLEAR_FEATURE:
        ret = 0;
        break;
        /* Class specific requests.  */
    case ClassInterfaceOutRequest | MassStorageReset:
        /* Reset state ready for the next CBW.  */
        s->mode = USB_MSDM_CBW;
        ret = 0;
        break;
    case ClassInterfaceRequest | GetMaxLun:
        data[0] = 0;
        ret = 1;
        break;
    default:
        ret = USB_RET_STALL;
        break;
    }
DPRINTF("usb_goldfish_handle_control:exit:ret=0x%x\n", ret);
    return ret;
}

static void usb_goldfish_cancel_io(USBDevice *dev, USBPacket *p)
{
    MSDState *s = DO_UPCAST(MSDState, dev, dev);

DPRINTF("usb_goldfish_handle_cancel_io:enter:p=0x%x\n", p);
    assert(s->packet == p);
    s->packet = NULL;

    if (s->req) {
        scsi_req_cancel(s->req);
    }
DPRINTF("usb_goldfish_cancel_io:exit\n");
}

static int handshake2ret(pid)
{
	int ret = USB_RET_STALL;
	switch (pid) {
	case USB_PID_ACK:
		ret = 0;
		break;
	case USB_PID_NAK:
		ret = USB_RET_NAK;
		break;
	case USB_PID_STALL:
		ret = USB_RET_STALL;
		break;
	case USB_PID_NYET:
		ret = USB_RET_NAK;
		break;
	defaults:
		break;
	}
	return ret;
}

static int usb_goldfish_handle_data(USBDevice *dev, USBPacket *p)
{
	int ret = 0;
	int data_len;
	uint8_t devep = p->ep->nr;
	static char sendBuf[10240];
	static char recvBuf[10240];
	struct usb_packet_handshake res;

	DPRINTF("usb_goldfish_handle_data:enter:pid=0x%x ep_nr=%d\n", p->pid, p->ep->nr);

	switch (p->pid) {
	case USB_TOKEN_OUT:
		data_len = iov_to_buf(p->iov.iov, p->iov.niov, p->result, sendBuf, 4096);
		p->result += data_len; 
		DPRINTF("usb_goldfish_handle_data:OUT:iov_size=%d data_len=%d\n", p->iov.size, data_len);
#if 1
{
	struct usb_goldfish_cbw *cbw;
	cbw = (struct usb_goldfish_cbw*)sendBuf;
	DPRINTF("usb_goldfish_handle_data:sig=0x%x\n", cbw->sig);
	DPRINTF("usb_goldfish_handle_data:tag=0x%x\n", cbw->tag);
	DPRINTF("usb_goldfish_handle_data:datalen=0x%x\n", cbw->data_len);
	DPRINTF("usb_goldfish_handle_data:flags=0x%x\n", cbw->flags);
	DPRINTF("usb_goldfish_handle_data:lun=0x%x\n", cbw->lun);
	DPRINTF("usb_goldfish_handle_data:cmdlen=0x%x\n", cbw->cmd_len);
}
#endif
		ret = sendRecvDataPacket(USB_PID_OUT, devep, 
			sendBuf, data_len, (char*)&res, sizeof(res));
		if (ret != 0) {
			ret = USB_RET_STALL;
		} else {
			ret = handshake2ret(res.pid_handshake);
		}
		break;
	case USB_TOKEN_IN:
{
		struct usb_packet_data *resp;
		resp = (struct usb_packet_data *)recvBuf;
		ret = sendRecvDataPacket(USB_PID_IN, devep, 
			NULL, 0, (char*)resp, 4096);
		if (ret != 0) {
			ret = USB_RET_STALL;
		} else {
			ret = handshake2ret(resp->pid_token);
			if (!ret) {
            			usb_packet_copy(p, &resp->data[0], resp->datalen);
				ret = p->result;
			}
		}
}
		break;
	defaults:
		break;
	}
#ifdef TODO
    MSDState *s = (MSDState *)dev;
    uint32_t tag;
    int ret = 0;
    struct usb_goldfish_cbw cbw;
    uint8_t devep = p->ep->nr;

    switch (p->pid) {
    case USB_TOKEN_OUT:
        if (devep != 2)
            goto fail;

        switch (s->mode) {
        case USB_MSDM_CBW:
            if (p->iov.size != 31) {
                fDPRINTF(stderr, "usb-msd: Bad CBW size");
                goto fail;
            }
            usb_packet_copy(p, &cbw, 31);
            if (le32_to_cpu(cbw.sig) != 0x43425355) {
                fDPRINTF(stderr, "usb-msd: Bad signature %08x\n",
                        le32_to_cpu(cbw.sig));
                goto fail;
            }
            DPRINTF("Command on LUN %d\n", cbw.lun);
            if (cbw.lun != 0) {
                fDPRINTF(stderr, "usb-msd: Bad LUN %d\n", cbw.lun);
                goto fail;
            }
            tag = le32_to_cpu(cbw.tag);
            s->data_len = le32_to_cpu(cbw.data_len);
            if (s->data_len == 0) {
                s->mode = USB_MSDM_CSW;
            } else if (cbw.flags & 0x80) {
                s->mode = USB_MSDM_DATAIN;
            } else {
                s->mode = USB_MSDM_DATAOUT;
            }
            DPRINTF("Command tag 0x%x flags %08x len %d data %d\n",
                    tag, cbw.flags, cbw.cmd_len, s->data_len);
            assert(le32_to_cpu(s->csw.residue) == 0);
            s->scsi_len = 0;
            s->req = scsi_req_new(s->scsi_dev, tag, 0, cbw.cmd, NULL);
#ifdef DEBUG_MSD
            scsi_req_print(s->req);
#endif
            scsi_req_enqueue(s->req);
            if (s->req && s->req->cmd.xfer != SCSI_XFER_NONE) {
                scsi_req_continue(s->req);
            }
            ret = p->result;
            break;

        case USB_MSDM_DATAOUT:
            DPRINTF("Data out %zd/%d\n", p->iov.size, s->data_len);
            if (p->iov.size > s->data_len) {
                goto fail;
            }

            if (s->scsi_len) {
                usb_goldfish_copy_data(s, p);
            }
            if (le32_to_cpu(s->csw.residue)) {
                int len = p->iov.size - p->result;
                if (len) {
                    usb_packet_skip(p, len);
                    s->data_len -= len;
                    if (s->data_len == 0) {
                        s->mode = USB_MSDM_CSW;
                    }
                }
            }
            if (p->result < p->iov.size) {
                DPRINTF("Deferring packet %p [wait data-out]\n", p);
                s->packet = p;
                ret = USB_RET_ASYNC;
            } else {
                ret = p->result;
            }
            break;

        default:
            DPRINTF("Unexpected write (len %zd)\n", p->iov.size);
            goto fail;
        }
        break;

    case USB_TOKEN_IN:
        if (devep != 1)
            goto fail;

        switch (s->mode) {
        case USB_MSDM_DATAOUT:
            if (s->data_len != 0 || p->iov.size < 13) {
                goto fail;
            }
            /* Waiting for SCSI write to complete.  */
            s->packet = p;
            ret = USB_RET_ASYNC;
            break;

        case USB_MSDM_CSW:
            if (p->iov.size < 13) {
                goto fail;
            }

            if (s->req) {
                /* still in flight */
                DPRINTF("Deferring packet %p [wait status]\n", p);
                s->packet = p;
                ret = USB_RET_ASYNC;
            } else {
                usb_goldfish_send_status(s, p);
                s->mode = USB_MSDM_CBW;
                ret = 13;
            }
            break;

        case USB_MSDM_DATAIN:
            DPRINTF("Data in %zd/%d, scsi_len %d\n",
                    p->iov.size, s->data_len, s->scsi_len);
            if (s->scsi_len) {
                usb_goldfish_copy_data(s, p);
            }
            if (le32_to_cpu(s->csw.residue)) {
                int len = p->iov.size - p->result;
                if (len) {
                    usb_packet_skip(p, len);
                    s->data_len -= len;
                    if (s->data_len == 0) {
                        s->mode = USB_MSDM_CSW;
                    }
                }
            }
            if (p->result < p->iov.size) {
                DPRINTF("Deferring packet %p [wait data-in]\n", p);
                s->packet = p;
                ret = USB_RET_ASYNC;
            } else {
                ret = p->result;
            }
            break;

        default:
            DPRINTF("Unexpected read (len %zd)\n", p->iov.size);
            goto fail;
        }
        break;

    default:
        DPRINTF("Bad token\n");
    fail:
        ret = USB_RET_STALL;
        break;
    }
#endif
DPRINTF("usb_goldfish_handle_data:exit:ret=0x%x\n", ret);

    return ret;
}

static void usb_goldfish_password_cb(void *opaque, int err)
{
    MSDState *s = opaque;

    if (!err)
        err = usb_device_attach(&s->dev);

    if (err)
        qdev_unplug(&s->dev.qdev, NULL);
}

static void *usb_goldfish_load_request(QEMUFile *f, SCSIRequest *req)
{
    MSDState *s = DO_UPCAST(MSDState, dev.qdev, req->bus->qbus.parent);

DPRINTF("usb_goldfish_load_request:enter\n");
    /* nothing to load, just store req in our state struct */
    assert(s->req == NULL);
    scsi_req_ref(req);
    s->req = req;
DPRINTF("usb_goldfish_load_request:exit\n");
    return NULL;
}

static const struct SCSIBusInfo usb_goldfish_scsi_info = {
    .tcq = false,
    .max_target = 0,
    .max_lun = 0,

    .transfer_data = usb_goldfish_transfer_data,
    .complete = usb_goldfish_command_complete,
    .cancel = usb_goldfish_request_cancelled,
    .load_request = usb_goldfish_load_request,
};

static int usb_goldfish_initfn(USBDevice *dev)
{
    MSDState *s = DO_UPCAST(MSDState, dev, dev);
    BlockDriverState *bs = s->conf.bs;
DPRINTF("usb_goldfish_initfn()\n");

#ifdef TODO
    if (!bs) {
        error_report("drive property not set");
        return -1;
    }

    blkconf_serial(&s->conf, &s->serial);

    /*
     * Hack alert: this pretends to be a block device, but it's really
     * a SCSI bus that can serve only a single device, which it
     * creates automatically.  But first it needs to detach from its
     * blockdev, or else scsi_bus_legacy_add_drive() dies when it
     * attaches again.
     *
     * The hack is probably a bad idea.
     */
    bdrv_detach_dev(bs, &s->dev.qdev);
    s->conf.bs = NULL;

    if (s->serial) {
        usb_desc_set_string(dev, STR_SERIALNUMBER, s->serial);
    } else {
        usb_desc_create_serial(dev);
    }

    usb_desc_init(dev);
    scsi_bus_new(&s->bus, &s->dev.qdev, &usb_goldfish_scsi_info);
    s->scsi_dev = scsi_bus_legacy_add_drive(&s->bus, bs, 0, !!s->removable,
                                            s->conf.bootindex);
    if (!s->scsi_dev) {
        return -1;
    }
    s->bus.qbus.allow_hotplug = 0;
    usb_goldfish_handle_reset(dev);

    if (bdrv_key_required(bs)) {
        if (cur_mon) {
            monitor_read_bdrv_key_start(cur_mon, bs, usb_goldfish_password_cb, s);
            s->dev.auto_attach = 0;
        } else {
            autostart = 0;
        }
    }
#endif
    return 0;
}

#if 1
static int usb_device_init(USBDevice *dev)
{
    USBDeviceClass *klass = USB_DEVICE_GET_CLASS(dev);
    if (klass->init) {
        return klass->init(dev);
    }
    return 0;
}

static USBDescIface	goldfish_desc_iface_full;
static USBDescDevice	goldfish_desc_device_full;
static USBDescIface	goldfish_desc_iface_high[2];/* ADB, MASS */
static USBDescEndpoint	goldfish_desc_endpoint[4];/* ADB, MASS */
static USBDescDevice	goldfish_desc_device_high;
static USBDesc		goldfish_desc;
static USBDescConfig	goldfish_desc_config;

static int usb_goldfish_init2(USBDevice *dev)
{
    int rc;
    
    goldfish_desc.high = &goldfish_desc_device_high;
    goldfish_desc.full = &goldfish_desc_device_high;
    usb_desc_init(dev);

    pstrcpy(dev->product_desc, sizeof(dev->product_desc),
            usb_device_get_product_desc(dev));
    dev->auto_attach = 1;
    QLIST_INIT(&dev->strings);
    usb_ep_init(dev);
    rc = usb_claim_port(dev);
    if (rc != 0) {
        return rc;
    }
#if 0
    rc = usb_device_init(dev);
    if (rc != 0) {
        usb_release_port(dev);
        return rc;
    }
#endif
#if 1
    if (dev->auto_attach) {
        rc = usb_device_attach(dev);
        if (rc != 0) {
            return rc;
        }
    }
#endif
    return 0;
}
#endif

#include "qemu_socket.h"

static int sock_fd = -1;
static char *sock_vusb_path = "/tmp/vusb.sock";
static void vusb_client_read(void *opaque)
{
	int ret;
	static char buf[255];
	DPRINTF("vusb_client_read:enter\n");
	ret = qemu_recv(client_fd, buf, 255, 0);
	if (ret <= 0) {
		DPRINTF("vusb_client_read:ret = %d errno=%d\n", ret, errno);
		qemu_set_fd_handler(client_fd, NULL, NULL, NULL);
		close(client_fd);
		client_fd = -1;
	}
	DPRINTF("vusb_client_read:exit\n");
}
static void vusb_attach(void *opaque)
{
	struct sockaddr_un addr;
	socklen_t addrlen = sizeof(addr);
	DPRINTF("vusb_attach:enter\n");
	client_fd = qemu_accept(sock_fd,
			(struct sockaddr *)&addr, &addrlen);
	if (client_fd == -1) {
		DPRINTF("error accept: %s", strerror(errno));
		return;
	}
	qemu_set_fd_handler(client_fd, vusb_client_read, NULL, NULL);
	usb_device_add("goldfish");
	DPRINTF("vusb_attach:exit\n");
	return;
}

/***************** USB プロトコル層 ******************/
/****************************************************/
static USBDevice *usb_goldfish_init(USBBus *bus, const char *filename)
{
    static int nr=0;
    char id[8];
    QemuOpts *opts;
    DriveInfo *dinfo;
    USBDevice *dev;
    const char *p1;
    char fmt[32];
    DPRINTF("usb_goldfish_init:bus=0x%x\n", bus);


    /* parse -usbdevice disk: syntax into drive opts */
#ifdef TODO
    snDPRINTF(id, sizeof(id), "usb%d", nr++);
    opts = qemu_opts_create(qemu_find_opts("drive"), id, 0, NULL);

    p1 = strchr(filename, ':');
    if (p1++) {
        const char *p2;

        if (strstart(filename, "format=", &p2)) {
            int len = MIN(p1 - p2, sizeof(fmt));
            pstrcpy(fmt, len, p2);
            qemu_opt_set(opts, "format", fmt);
        } else if (*filename != ':') {
            DPRINTF("unrecognized USB mass-storage option %s\n", filename);
            return NULL;
        }
        filename = p1;
    }
    if (!*filename) {
        DPRINTF("block device specification needed\n");
        return NULL;
    }
    qemu_opt_set(opts, "file", filename);
    DPRINTF("usb_goldfish_init:1\n");
    qemu_opt_set(opts, "if", "none");

    /* create host drive */
    dinfo = drive_init(opts, 0);
    if (!dinfo) {
        qemu_opts_del(opts);
        return NULL;
    }
#endif
    DPRINTF("usb_goldfish_init:2\n");

    /* create guest device */
    dev = usb_create(bus, "usb-goldfish");
    if (!dev) {
        return NULL;
    }
    DPRINTF("usb_goldfish_init:3\n");
#if 1
{
	int r;
	r = usb_goldfish_init2(dev);
	DPRINTF("usb_qdev_init():r=%d\n", r);
}
#endif
#ifdef TODO
    if (qdev_prop_set_drive(&dev->qdev, "drive", dinfo->bdrv) < 0) {
        qdev_free(&dev->qdev);
        return NULL;
    }
#endif
    DPRINTF("usb_goldfish_init:4\n");
#ifdef TODO
    if (qdev_init(&dev->qdev) < 0)
        return NULL;
#endif

    DPRINTF("usb_goldfish_init:5\n");
    return dev;
}

static const VMStateDescription vmstate_usb_msd = {
    .name = "usb-goldfish",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField []) {
        VMSTATE_USB_DEVICE(dev, MSDState),
        VMSTATE_UINT32(mode, MSDState),
        VMSTATE_UINT32(scsi_len, MSDState),
        VMSTATE_UINT32(scsi_off, MSDState),
        VMSTATE_UINT32(data_len, MSDState),
        VMSTATE_UINT32(csw.sig, MSDState),
        VMSTATE_UINT32(csw.tag, MSDState),
        VMSTATE_UINT32(csw.residue, MSDState),
        VMSTATE_UINT8(csw.status, MSDState),
        VMSTATE_END_OF_LIST()
    }
};

static Property msd_properties[] = {
    DEFINE_BLOCK_PROPERTIES(MSDState, conf),
    DEFINE_PROP_STRING("serial", MSDState, serial),
    DEFINE_PROP_BIT("removable", MSDState, removable, 0, false),
    DEFINE_PROP_END_OF_LIST(),
};

#if 1
/* TODO: ここでデバイス情報を取得するか。 */

static const USBDescStrings goldfish_desc_strings = {
    [STR_MANUFACTURER] = "QEMU",
    [STR_PRODUCT]      = "QEMU USB GOLDFISH",
    [STR_SERIALNUMBER] = "1",
    [STR_CONFIG_FULL]  = "Full speed config (usb 1.1)",
    [STR_CONFIG_HIGH]  = "High speed config (usb 2.0)",
};

static void setConfigDesc(struct usb_config_descriptor *dp)
{
	USBDescConfig *dest = &goldfish_desc_config;
	int i_inf;
	int i_ep;
	int nep = 0;
	struct usb_interface_descriptor *idp = NULL;
	struct usb_endpoint_descriptor *ep = NULL;

	dest->bNumInterfaces = dp->bNumInterfaces;
	dest->bConfigurationValue = dp->bConfigurationValue;
	dest->iConfiguration = dp->iConfiguration;
	dest->bmAttributes = dp->bmAttributes;
	dest->bMaxPower = dp->bMaxPower;
	dest->nif_groups = 0;
	dest->if_groups = NULL;
	dest->nif = dp->bNumInterfaces;
	dest->ifs = goldfish_desc_iface_high;

	idp = ++dp;
	for (i_inf = 0; i_inf < dp->bNumInterfaces; i_inf++) {
		dest->ifs[i_inf].bInterfaceNumber = idp->bInterfaceNumber;
		dest->ifs[i_inf].bAlternateSetting = idp->bAlternateSetting;
		dest->ifs[i_inf].bNumEndpoints = idp->bNumEndpoints;
		dest->ifs[i_inf].bInterfaceClass = idp->bInterfaceClass;
		dest->ifs[i_inf].bInterfaceSubClass = idp->bInterfaceSubClass;
		dest->ifs[i_inf].bInterfaceProtocol = idp->bInterfaceProtocol;
		dest->ifs[i_inf].iInterface = idp->iInterface;
		
		dest->ifs[i_inf].ndesc = 0;
		dest->ifs[i_inf].descs = NULL;
		dest->ifs[i_inf].eps = goldfish_desc_endpoint;
		ep = (idp + 1); 
		for (i_ep = 0; i_ep < idp->bNumEndpoints; i_ep++) {
			goldfish_desc_endpoint[nep].bEndpointAddress = ep->bEndpointAddress;
			goldfish_desc_endpoint[nep].bmAttributes = ep->bmAttributes;
			goldfish_desc_endpoint[nep].wMaxPacketSize = ep->wMaxPacketSize;
			goldfish_desc_endpoint[nep].bInterval = ep->bInterval;
			goldfish_desc_endpoint[nep].is_audio = 0;
			goldfish_desc_endpoint[nep].extra = NULL;
			ep = (((char*)ep) + ep->bLength);
			nep++;
		}
		idp = ep;
	}
}
#endif
void goldfish_usb_desc_attach(USBDevice *dev)
{
	struct usb_packet_setup_DescriptorDeviceRes *devp;
	struct usb_packet_setup_DescriptorConfigRes *confp;
	DPRINTF("goldfish_usb_desc_attach:enter\n");
#if 1
	//DEVICE
	sendRecvSetupPacket(USB_REQ_GET_DESCRIPTOR, 
			(USB_DT_DEVICE << 8), 0, 0x12, &devp);
	goldfish_desc.id.idVendor = devp->desc.idVendor;
	goldfish_desc.id.idProduct = devp->desc.idProduct;
	goldfish_desc.id.bcdDevice = devp->desc.bcdDevice;
	goldfish_desc.id.iManufacturer = devp->desc.iManufacturer;
	goldfish_desc.id.iProduct = devp->desc.iProduct;
	goldfish_desc.id.iSerialNumber = devp->desc.iSerialNumber;
	goldfish_desc.id.idVendor = devp->desc.idVendor;

	// STRING
	//sendRecvSetupPacket(USB_REQ_GET_DESCRIPTOR, 
	//		(USB_DT_STRING << 8), 1, 512, &devp);
	goldfish_desc.str = goldfish_desc_strings;

	goldfish_desc_device_high.bcdUSB = devp->desc.bcdUSB; 
	goldfish_desc_device_high.bMaxPacketSize0 = devp->desc.bMaxPacketSize0; 
	goldfish_desc_device_high.bNumConfigurations = devp->desc.bNumConfigurations; 

	// CONFIG
	sendRecvSetupPacket(USB_REQ_GET_DESCRIPTOR, 
			(USB_DT_CONFIG << 8), 0, 512, &confp);
	// 1個限定
	goldfish_desc_device_high.confs = &goldfish_desc_config;
	setConfigDesc(&confp->config_desc);
#else
	usb_desc_attach(dev);
#endif
	DPRINTF("goldfish_usb_desc_attach:exit\n");
}

static void usb_goldfish_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

DPRINTF("usb_goldfish_class_initfn()\n");

    uc->init           = usb_goldfish_initfn;
    uc->product_desc   = "QEMU USB GOLDFISH";
    uc->usb_desc       = &goldfish_desc;
    uc->cancel_packet  = usb_goldfish_cancel_io;
    uc->handle_attach  = goldfish_usb_desc_attach;
    uc->handle_reset   = usb_goldfish_handle_reset;
    uc->handle_control = usb_goldfish_handle_control;
    uc->handle_data    = usb_goldfish_handle_data;
    dc->fw_name = "goldfish";
    dc->vmsd = &vmstate_usb_msd;
    dc->props = msd_properties;
}

static TypeInfo msd_info = {
    .name          = "usb-goldfish",
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(MSDState),
    .class_init    = usb_goldfish_class_initfn,
};

static void usb_goldfish_register_types(void)
{
    DPRINTF("usb_goldfish_register_types:enter\n");
#if 1
    (void)unlink(sock_vusb_path);
    sock_fd = unix_listen(sock_vusb_path, NULL, strlen(sock_vusb_path));
    if (sock_fd >= 0) {
	DPRINTF("sock_fd=%d\n", sock_fd);
	socket_set_block(sock_fd);
    	qemu_set_fd_handler(sock_fd, vusb_attach, NULL, NULL);
    }
#endif
    type_register_static(&msd_info);
    usb_legacy_register("usb-goldfish", "goldfish", usb_goldfish_init);
}

type_init(usb_goldfish_register_types)
