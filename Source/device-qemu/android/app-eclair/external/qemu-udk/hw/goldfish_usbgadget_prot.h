#ifndef _GOLDFISH_USBGADGET_PROT_H
#define _GOLDFISH_USBGADGET_PROT_H

/*********** (USB PROTCOL HEADER) ************************/
#include "ch9.h"
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

/* token PID */
#define USB_PID_OUT		0x1
#define USB_PID_IN		0x9
#define USB_PID_SOF		0x5
#define USB_PID_SETUP		0xD
#define USB_PID_PING		0x4

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

/* BUS PID */
#define USB_PID_HOST		0x10
#define USB_PID_DEVICE		0x20
#define USB_PID_ATTACH		0x30
#define USB_PID_DETTACH		0x40
#define USB_PID_RESET		0x50
#define USB_PID_INT		0x60

struct usb_packet {
	__u8 pid;
	__u8 padding[3];
	__u8 padding1[4];
};
struct usb_packet_token {
	__u8 pid_token;
	__u8 endpoint;
	__u8 dir; /* 0:HOST->SLAVE, 1:SLAVE->HOST */
	__u8 padding[1];
	__u8 padding1[4];
};
struct usb_packet_data_head {
	__u8 pid_token;
	__u8 endpoint;
	__u8 dir; /* 0:HOST->SLAVE, 1:SLAVE->HOST */
	__u8 padding[1];
	__le16 datalen;
	__u8 padding1[2];
};
struct usb_packet_data {
	__u8 pid_token;
	__u8 endpoint;
	__u8 dir; /* 0:HOST->SLAVE, 1:SLAVE->HOST */
	__u8 padding[1];
	__le16 datalen;
	__u8 padding1[2];
	__u8 data[4];
};
struct usb_packet_setupdata {
	__u8 pid_token;
	__u8 endpoint;
	__u8 dir; /* 0:HOST->SLAVE, 1:SLAVE->HOST */
	__u8 padding1[1];
	__le16 datalen;
	__u8 padding2[2];
	struct usb_ctrlrequest req;     /* 8bytes */
};
struct usb_packet_handshake {
	__u8 pid_handshake;
	__u8 endpoint;
	__u8 dir; /* 0:HOST->SLAVE, 1:SLAVE->HOST */
	__u8 padding[1];
	__u8 padding1[4];
};

struct usb_packet_setup {
	__u8 pid_token;
	__u8 endpoint;
	__u8 pid_data;
	__u8 padding[1];
	struct usb_ctrlrequest req;	/* 8bytes */
	__u8 padding1[4];
};
struct usb_packet_setup_DescriptorRes {
        __u8 pid_handshake;
        __u8 pid_data;
        __u8 padding[2];
        __u8 data[1];
	__u8 padding1[4];
};

#define DIR_HOST_TO_SLAVE       0x0
#define DIR_SLAVE_TO_HOST       0x1
extern void hostEventHandler(void *opaque);

#endif
