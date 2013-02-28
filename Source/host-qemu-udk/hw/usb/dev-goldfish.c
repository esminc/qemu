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
#include "goldfish_usbgadget_prot.h"
#include "iov.h"
#include "qemu-timer.h"

#define DEBUG_MSD
#ifdef DEBUG_MSD
#define DPRINTF(fmt, ...) \
do { printf("usb-goldfish: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif
/******************************************************************************
 * 通信処理
 *
 */
#include "qemu_socket.h"
#include <netinet/in.h>
#include <netinet/tcp.h>

static int sock_fd = -1;
static int connectToDevice(void);
static void disconnectToDevice(void);
static void retry_connect_stop(void);
/**************************************************
 * コネクション切断時の再接続処理
 * 再接続処理は以下の契機で起動する
 * 1) 起動時
 * 2) 通信異常発生時(send, recv)
 **************************************************/

/**
 * コネクション切れたときに再接続要求するためのタイマーオブジェクト
 */
static QEMUTimer *goldfish_timer = NULL;

/**
 * コネクション再接続リトライ時間(0.1sec)
 */
static int64_t retry_time;

/**
 * タイマーをアップデートする
 */
static void update_timer(void)
{
        int64_t current_time;
        retry_time = 1000;
        current_time = qemu_get_clock_ms(vm_clock);
        printf("current_time=%lld retry_time=%lld\n", current_time, retry_time);
        qemu_mod_timer(goldfish_timer, current_time + retry_time);
}
/**
 * タイマー処理
 */
static void doRetryConnect(void *opaque)
{
        int ret = connectToDevice();
        if (ret < 0) {
                printf("doRetryConnect:failed\n");
                update_timer();
        } else {
                printf("doRetryConnect:Success\n");
                retry_connect_stop();
        }
        return;
}
/**
 * タイマースタートする
 */
static void retry_connect_start(void)
{
        if (goldfish_timer == NULL) {
                printf("retry_connect_start vm_clock=%p\n", vm_clock);
                goldfish_timer = qemu_new_timer_ms(vm_clock, doRetryConnect, NULL);
                assert(goldfish_timer != NULL);
                update_timer();
        }
}
/**
 * タイマー停止する
 */
static void retry_connect_stop(void)
{
        if (goldfish_timer != NULL) {
                printf("retry_connect_stop\n");
                qemu_del_timer(goldfish_timer);
        }
        goldfish_timer = NULL;
}
static int recvUsbData(char *bufp, int len)
{
        int ret;
        ret = qemu_recv_full(sock_fd, bufp, len, 0);
        if (ret <= 0) {
        	disconnectToDevice();
        }
        return ret;
}

static int sendUsbData(char *bufp, int len)
{
        int ret;
        //ret = socket_send(sock_fd, bufp, len);
        printf("before send\n");
        ret = qemu_send_full(sock_fd, bufp, len, 0);
        if (ret <= 0) {
                disconnectToDevice();
        }
        printf("ret=%d err=%d\n", ret, errno);
        //fsync(sock_fd);
        return ret;
}

static void declareHost(void)
{
        int ret;
        struct usb_packet_token token;
        printf("declareHost\n");
        token.pid_token = USB_PID_HOST;
        token.endpoint = 0;
        token.dir = DIR_HOST_TO_SLAVE;
        ret = sendUsbData((char*)&token, sizeof(token));
        assert(ret > 0);
        printf("***** declareHost:ret=%d\n", ret);
}
static void deviceEventHandler(void *opaque);
static int connectToDevice(void)
{
        int flag = 1;
        int sock;
        struct sockaddr_in in;

        if (sock_fd != -1) {
                return 0;
        }

        DPRINTF("connectToDevice\n");
        sock = socket(PF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
                perror("socket(inet)");
                return -1;
        }
        memset(&in, 0, sizeof(in));
        in.sin_family = PF_INET;
        in.sin_port = htons(10000);
        inet_aton("127.0.0.1", &(in.sin_addr));
        printf("before socket_set_nonblock\n");
        socket_set_nonblock(sock);
        fcntl(sock, F_SETFL, O_NONBLOCK);
        printf("before setsockopt\n");
        (void)setsockopt(sock, SOL_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag) );
        printf("before connect\n");
        if (connect(sock, (struct sockaddr*) &in, sizeof(in)) < 0) {
                int ret = errno;
                printf("err=%d\n", errno);
                if (errno == EINPROGRESS) {
                        char byte;
                        ret = recv(sock, &byte, sizeof(byte), MSG_PEEK);
                        if (ret == 0 || (ret < 0 && errno == EAGAIN)) {
                                printf("EINPROGRESS bug OK connect\n");
                        } else {
                                close(sock);
                                printf("err=%d\n", errno);
                                return -1;
                        }
                } else {
                        close(sock);
                        return -1;
                }
         }
        sock_fd = sock;
        declareHost();
        qemu_set_fd_handler(sock_fd, deviceEventHandler, NULL, NULL);
        return 0;
}
static void disconnectToDevice(void)
{
        if (sock_fd != -1) {
        		  DPRINTF("close:sock_fd = %d\n", sock_fd);
                qemu_set_fd_handler(sock_fd, NULL, NULL, NULL);
                close(sock_fd);
                sock_fd = -1;
                retry_connect_start();
        }
}
extern int usb_device_add(const char *devname);
extern int usb_device_del(const char *devname);
static void vusb_attach(void)
{
	int ret;
	struct usb_packet_token token;

	token.pid_token = USB_PID_RESET;
	token.endpoint = 0;
	token.dir = DIR_HOST_TO_SLAVE;

	// SEND RESET
    ret = sendUsbData((char*)&token, sizeof(token));
    assert(ret > 0);

    // RECV RESET
    ret = recvUsbData((char*)&token, sizeof(token));
    assert(ret > 0);
    assert(token.pid_token == USB_PID_RESET);

	usb_device_add("goldfish");
	return;
}
static void vusb_dettach(void)
{
	usb_device_del("goldfish");
	return;
}
static void deviceEventHandler(void *opaque)
{
	int ret;
	struct usb_packet_token token;

	DPRINTF("deviceEventHandler:enter\n");
	ret = recvUsbData((char*)&token, sizeof(token));
	assert(ret == sizeof(token));

	if (token.pid_token == USB_PID_ATTACH) {
		vusb_attach();
	} else if (token.pid_token == USB_PID_DETTACH) {
		vusb_dettach();
	} else {
		printf("ERROR!! Invalid token=%d\n", token.pid_token);
		assert(0);
	}
	DPRINTF("deviceEventHandler:exit\n");
}
/**
 * send (SETUP/OUT) TOKEN
 * send DATA
 * recv ACK
 */
static void sendOUT(struct usb_packet_token *tp, struct usb_packet_data *dp)
{
	int ret;
	struct usb_packet_handshake hs;

	tp->dir = DIR_HOST_TO_SLAVE;
	dp->dir = DIR_HOST_TO_SLAVE;
	// send TOKEN
	ret = sendUsbData((char*)tp, sizeof(*tp));
	assert(ret > 0);
	DPRINTF("sendOUT:TOKEN:ret=%d\n", ret);
	// send DATA
	ret = sendUsbData((char*)dp, sizeof(struct usb_packet_data_head) + dp->datalen);
	assert(ret > 0);
	DPRINTF("sendOUT:DATA:ret=%d\n", ret);
	// recv ACK
	ret = sendUsbData((char*)&hs, sizeof(hs));
	assert(ret > 0);
	DPRINTF("sendOUT:hand_shake:ret=%d pid=0x%x\n", ret, hs.pid_handshake);
}

/**
 * send IN TOKEN
 * recv DATA
 * send ACK
 */
static void sendIN(struct usb_packet_token *tp, struct usb_packet_data *dp)
{
	int ret;
	struct usb_packet_handshake hs;
	tp->dir = DIR_HOST_TO_SLAVE;
	// send IN TOKEN
	ret = sendUsbData((char*)tp, sizeof(*tp));
	assert(ret > 0);
	DPRINTF("sendIN:TOKEN:ret=%d\n", ret);
	// recv DATA
	ret = recvUsbData((char*)dp, sizeof(struct usb_packet_data_head));
	assert(ret > 0);// TODO
	DPRINTF("sendIN:DATA(1/2):ret=%d\n", ret);
	if (dp->datalen > 0) {
		ret += recvUsbData((char*)dp->data, dp->datalen); // TODO
		assert(ret > 0);
	}
	DPRINTF("sendIN:DATA(2/2):ret=%d\n", ret);
	// send ACK
	hs.pid_handshake = USB_PID_ACK;
	hs.endpoint = tp->endpoint;
	hs.dir = DIR_HOST_TO_SLAVE;
	ret = sendUsbData((char*)&hs, sizeof(hs));
	assert(ret > 0);
	DPRINTF("sendOUT:hand_shake:ret=%d pid=0x%x\n", ret, hs.pid_handshake);
}

/******************************************************************************/


struct usb_packet_setup_DescriptorDeviceRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
	struct usb_device_descriptor desc;
        __u8 padding1[4];
};
//#define DEBUG_MSD


static void sendNoDataSETUP(struct usb_packet_token *packetp, struct usb_ctrlrequest *req)
{
	struct usb_packet_token setup, in;
	struct usb_packet_setupdata setupdata;
	struct usb_packet_data s_data;
	memcpy(&setup, packetp, sizeof(setup));
	memcpy(&setupdata.req, req, sizeof(*req));
	setupdata.pid_token = USB_PID_DATA_0;
	setupdata.endpoint = setup.endpoint;
	setupdata.dir = DIR_HOST_TO_SLAVE;
	setupdata.datalen = sizeof(setupdata.req);
	// SETUP ステージ
	setup.dir = DIR_HOST_TO_SLAVE;
	sendOUT(&setup, (struct usb_packet_data*)&setupdata);

	// STATUS ステージ
	in.pid_token = USB_PID_IN;
	in.dir = DIR_HOST_TO_SLAVE;
	in.endpoint = 0;
	s_data.pid_token = USB_PID_DATA_0;
	s_data.endpoint = 0;
	s_data.dir = DIR_HOST_TO_SLAVE;
	s_data.datalen = 0;
	sendIN(&in, &s_data);
}
static void sendDataSETUP(struct usb_packet_token *packetp, struct usb_ctrlrequest *req, char *datap)
{
	struct usb_packet_token setup, out, in;
	struct usb_packet_setupdata setupdata;
	struct usb_packet_data s_data;
	struct usb_packet_data *in_data;
	char buf[4096*3];
	memcpy(&setup, packetp, sizeof(setup));
	memcpy(&setupdata.req, req, sizeof(*req));
	setupdata.pid_token = USB_PID_DATA_0;
	setupdata.endpoint = setup.endpoint;
	setupdata.dir = DIR_HOST_TO_SLAVE;
	setupdata.datalen = sizeof(setupdata.req);
	// SETUP ステージ
	setup.dir = DIR_HOST_TO_SLAVE;
	sendOUT(&setup, (struct usb_packet_data*)&setupdata);

	// DATA ステージ
	in.pid_token = USB_PID_IN;
	in.endpoint = 0;
	in_data = (struct usb_packet_data*)buf;
	sendIN(&in, in_data);
	memcpy(datap, in_data->data, in_data->datalen);

	// STATUS ステージ
	out.pid_token = USB_PID_OUT;
	out.endpoint = 0;
	s_data.pid_token = USB_PID_DATA_0;
	s_data.endpoint = 0;
	s_data.dir = DIR_HOST_TO_SLAVE;
	s_data.datalen = 0;
	sendOUT(&out, &s_data);
}


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
        idp = (struct usb_interface_descriptor *)++dp; /* TODO */
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
		ep = (struct usb_endpoint_descriptor *)(idp + 1); /* TODO */
		for (i_ep = 0; i_ep < idp->bNumEndpoints; i_ep++) {
			PDESC("Endp", ep, bLength);
			PDESC("Endp", ep, bDescriptorType);
			PDESC("Endp", ep, bEndpointAddress);
			PDESC("Endp", ep, bmAttributes);
			PDESC("Endp", ep, wMaxPacketSize);
			PDESC("Endp", ep, bInterval);
			ep = (struct usb_endpoint_descriptor *)(((char*)ep) + ep->bLength);
		}
		idp = (struct usb_interface_descriptor *)ep;
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

static int sendRecvDataPacket(int pid, int endpoint, 
			const char* sendBufp, int sendLen,
				char *recvBufp, int recvLen)
{
	struct usb_packet_handshake *resp;
	struct usb_packet_token token;
	struct usb_packet_data *datap;
	static char sendBuf[4096*3];
	struct usb_packet_data *res;

	datap = (struct usb_packet_data*)sendBuf;
	resp = (struct usb_packet_handshake *)recvBufp;
	resp->pid_handshake = USB_PID_ACK;
	token.pid_token = pid;
	token.endpoint = endpoint;
	datap->endpoint = endpoint;
	switch (pid) {
	case USB_PID_IN:
		sendIN(&token, datap);
		res = (struct usb_packet_data*)recvBufp;
		DPRINTF("sendRecvDataPacket:IN:datalen=%d\n", datap->datalen);
		res->datalen = datap->datalen;
		memcpy(res->data, datap->data, datap->datalen);
		break;
	case USB_PID_OUT:
		datap->pid_token = USB_PID_DATA_0;
		datap->datalen = sendLen;
		memcpy(datap->data, sendBufp, sendLen);
		sendOUT(&token, datap);
		break;
	default:
		break;
	}
	//DPRINTF("sendRecvDataPacket():ret=%d pid=0x%x\n", 
	//		ret, resp->pid_handshake);
	return 0;
}
static void sendRecvSetupPacket(int request,
	__le16 wValue, __le16 wIndex, __le16 wLength, 
	struct usb_packet_setup_DescriptorDeviceRes **data)
{
	static char recvBuf[4096*3];
	struct usb_packet_setup packet;
	struct usb_packet_handshake *resp;

	DPRINTF("sendRecvSetupPacket:request=0x%x\n", request);
	packet.pid_token = USB_PID_SETUP;
	packet.endpoint = 0;
	switch (request) {
	case USB_REQ_SET_CONFIGURATION:
	{
		struct usb_packet_handshake res;
		resp = (struct usb_packet_handshake *)&res;
		createSET_CONFIGURATION(&packet.req, wValue, wIndex, wLength);
		sendNoDataSETUP((struct usb_packet_token*)&packet, &packet.req);
		resp->pid_handshake = USB_PID_ACK;
		break;
	}
	case USB_REQ_SET_ADDRESS:
	{
		struct usb_packet_handshake res;
		resp = (struct usb_packet_handshake *)&res;
		createSET_ADDRESS(&packet.req, wValue, wIndex, wLength);
		sendNoDataSETUP((struct usb_packet_token*)&packet, &packet.req);
		resp->pid_handshake = USB_PID_ACK;
		break;
	}
	case USB_REQ_SET_INTERFACE:
	{
		struct usb_packet_handshake res;
		resp = (struct usb_packet_handshake *)&res;
		createSET_INTERFACE(&packet.req, wValue, wIndex, wLength);
		sendNoDataSETUP((struct usb_packet_token*)&packet, &packet.req);
		resp->pid_handshake = USB_PID_ACK;
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
		sendDataSETUP((struct usb_packet_token*)&packet, &packet.req, (char*)&res->desc);
		resp->pid_handshake = USB_PID_ACK;
		if (wValue ==  (USB_DT_CONFIG << 8)) {
			printConfigDesc((struct usb_config_descriptor *)&res->desc);
		} else if (wValue ==  (USB_DT_STRING << 8)) {
			printStringDesc((struct usb_string_descriptor *)&res->desc);
		} else if (wValue ==  (USB_DT_DEVICE << 8)) {
			printDevDesc(&res->desc);
		}
		*data = (struct usb_packet_setup_DescriptorDeviceRes *)recvBuf;
		break;
	}
	default:
		break;
	}

	//DPRINTF("qemu_recv_full():ret=%d pid=0x%x\n",
	//		ret, resp->pid_handshake);
}

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
    default:
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

DPRINTF("usb_goldfish_handle_cancel_io:enter:p=%p\n", p);
    assert(s->packet == p);
    s->packet = NULL;

    if (s->req) {
        scsi_req_cancel(s->req);
    }
DPRINTF("usb_goldfish_cancel_io:exit\n");
}

static int handshake2ret(int pid)
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
	default:
		break;
	}
	return ret;
}

static int usb_goldfish_handle_data(USBDevice *dev, USBPacket *p)
{
	int ret = 0;
	int data_len;
	uint8_t devep = p->ep->nr;
	static char sendBuf[4096*3];
	static char recvBuf[4096*3];
	struct usb_packet_handshake res;

	DPRINTF("usb_goldfish_handle_data:enter:pid=0x%x ep_nr=%d\n", p->pid, p->ep->nr);

	switch (p->pid) {
	case USB_TOKEN_OUT:
		data_len = iov_to_buf(p->iov.iov, p->iov.niov, p->result, sendBuf, 4096*3);
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
		assert(data_len <= p->ep->max_packet_size);
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
		int off = 0;
		resp = (struct usb_packet_data *)recvBuf;
		while (1) {
			ret = sendRecvDataPacket(USB_PID_IN, devep,
				NULL, 0, (char*)resp, 4096*3);
			assert(ret == 0);
			ret = handshake2ret(resp->pid_token);
			assert(ret == 0);
			DPRINTF("usb_goldfish_handle_data:IN:iov_size=%d data_len=%d p->result=%d\n", p->iov.size, resp->datalen, p->result);
			usb_packet_copy(p, &resp->data[off], resp->datalen);
			if (resp->datalen < p->ep->max_packet_size ||
					p->result >= p->iov.size) {
				ret = p->result;
				break;
			}
			off += resp->datalen;
		}
}
		break;
	default:
		break;
	}
DPRINTF("usb_goldfish_handle_data:exit:ret=0x%x\n", ret);

    return ret;
}

static int usb_goldfish_initfn(USBDevice *dev)
{
DPRINTF("usb_goldfish_initfn()\n");

    return 0;
}

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
    if (dev->auto_attach) {
        rc = usb_device_attach(dev);
        if (rc != 0) {
            return rc;
        }
    }
    return 0;
}



/***************** USB プロトコル層 ******************/
/****************************************************/
static USBDevice *usb_goldfish_init(USBBus *bus, const char *filename)
{
    USBDevice *dev;
    DPRINTF("usb_goldfish_init:bus=%p\n", bus);


    /* parse -usbdevice disk: syntax into drive opts */
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
    DPRINTF("usb_goldfish_init:4\n");

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

	idp = (struct usb_interface_descriptor *)++dp;
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
		dest->ifs[i_inf].eps = (USBDescEndpoint *)goldfish_desc_endpoint;
		ep = (struct usb_endpoint_descriptor *)(idp + 1);
		for (i_ep = 0; i_ep < idp->bNumEndpoints; i_ep++) {
			goldfish_desc_endpoint[nep].bEndpointAddress = ep->bEndpointAddress;
			goldfish_desc_endpoint[nep].bmAttributes = ep->bmAttributes;
			goldfish_desc_endpoint[nep].wMaxPacketSize = ep->wMaxPacketSize;
			goldfish_desc_endpoint[nep].bInterval = ep->bInterval;
			goldfish_desc_endpoint[nep].is_audio = 0;
			goldfish_desc_endpoint[nep].extra = NULL;
			ep = (struct usb_endpoint_descriptor *)(((char*)ep) + ep->bLength);
			nep++;
		}
		idp = (struct usb_interface_descriptor *)ep;
	}
}
static void goldfish_usb_desc_attach(USBDevice *dev)
{
	struct usb_packet_setup_DescriptorDeviceRes *devp;
	struct usb_packet_setup_DescriptorConfigRes *confp;
	static int singletone = 0;

	DPRINTF("goldfish_usb_desc_attach:enter\n");
	if (singletone) {
		DPRINTF("goldfish_usb_desc_attach:already init:exit\n");
		return;
	}
	singletone = 1;

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
			(USB_DT_CONFIG << 8), 0, 512, (struct usb_packet_setup_DescriptorDeviceRes **)&confp);
	// 1個限定
	goldfish_desc_device_high.confs = &goldfish_desc_config;
	setConfigDesc(&confp->config_desc);
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
    type_register_static(&msd_info);
    usb_legacy_register("usb-goldfish", "goldfish", usb_goldfish_init);
    doRetryConnect(NULL);
}

type_init(usb_goldfish_register_types)
