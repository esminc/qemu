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
#include "goldfish_usbgadget.h"
#include "goldfish_reg.h"
#include "goldfish_usbgadget_reg.h"
#include "goldfish_usbgadget_prot.h"
#define  GOLDFISH_TIMER_SAVE_VERSION  1

#define ECLIPSE_HOST

static void recvToken(struct usb_packet_token *tp);
static void recvDATA(struct usb_packet_data *dp);
static void sendDATA(struct usb_packet_data *dp);
static void sendACK(int endpoint)
{
	int ret;
	struct usb_packet_handshake token;
	dbg("sendACK:ep_nr=%d\n", endpoint);
	token.pid_handshake = USB_PID_ACK;
	token.endpoint = endpoint;
	token.dir = DIR_SLAVE_TO_HOST;
	ret = sendUsbData((char*)&token, sizeof(token));
	dbg("***** sendACK:ret=%d\n", ret);
}
#define dbg printf

static uint32_t goldfish_usbgadget_read(void *opaque, target_phys_addr_t offset);

struct usbgadget_state {
    struct goldfish_device dev;
    QEMUTimer *timer;
};

struct usbgadget_state usbgadget_state = {
    .dev = {
        .name = "goldfish_usbgadget",
        .id = -1,
        .size = 0x1000,
        .irq_count = 1,
    }
};
static void recvToken(struct usb_packet_token *tp)
{
	int ret;
	ret = recvUsbData((char*)tp, sizeof(*tp));
	dbg("***** recvToken:ret=%d\n", ret);
}
void hostEventHandler(void *opaque) 
{
	struct usb_packet_token token;
	dbg("hostEventHandler:enter\n");
	recvToken(&token);
	dbg("hostEventHandler:pid=%d\n", token.pid_token);
	goldfish_device_set_irq(&usbgadget_state.dev, 0, 1);
	return;
}

#define  GOLDFISH_TIMER_SAVE_VERSION  1

/****************** ホスト通信クラス ********************/
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include "sockets.h"
static int sock_fd = -1;

int recvUsbData(char *bufp, int len)
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

int sendUsbData(char *bufp, int len)
{
	int ret;
	//ret = socket_send(sock_fd, bufp, len);
	printf("before send\n");
	ret = send(sock_fd, bufp, len, MSG_DONTWAIT);
	printf("ret=%d err=%d\n", ret, errno);
	//fsync(sock_fd);
	return ret;
}
void declareDevice(void)
{
	int ret;
	struct usb_packet_token token;
	printf("declareDevice\n");
	token.pid_token = USB_PID_DEVICE;
	token.endpoint = 0;
	token.dir = DIR_SLAVE_TO_HOST;
	ret = sendUsbData((char*)&token, sizeof(token));
	printf("***** declareDevice:ret=%d\n", ret);
}

static int connectToHost(const char *path)
{
	int flag = 1;
	int sock;
	struct sockaddr_in in;

	if (sock_fd != -1) {
		return;
	}

	dbg("connectToHost:%s\n", path);
	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("socket(inet)");
		return -1;
	}
	memset(&in, 0, sizeof(in));
	in.sin_family = PF_INET;
	in.sin_port = htons(10000);
	inet_aton("127.0.0.1", &(in.sin_addr));
	sock_fd = sock;
	printf("before socket_set_nonblock\n");
	socket_set_nonblock(sock_fd);
	fcntl(sock_fd, F_SETFL, O_NONBLOCK);
	printf("before setsockopt\n");
	(void)setsockopt(sock_fd, SOL_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag) );
	printf("before connect\n");
	if (connect(sock, (struct sockaddr*) &in, sizeof(in)) < 0) {
		int ret = errno;
		printf("err=%d\n", errno);
		if (errno == EINPROGRESS) {
			char byte;
			ret = recv(sock_fd, &byte, sizeof(byte), MSG_PEEK);
			if (ret == 0 || (ret < 0 && errno == EAGAIN)) {
				printf("EINPROGRESS bug OK connect\n");
			} else {
				printf("err=%d\n", errno);
				return -1;
			}
		} else {
			return -1;
		}
	}

	declareDevice();
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

static uint32_t goldfish_usbgadget_read(void *opaque, target_phys_addr_t offset)
{
	uint32_t val;
	int ep_nr;
	struct usbgadget_state *s = (struct usbgadget_state *)opaque;

	if (offset == GOLDFISH_UDC_IN_CSR1_REG) { /* write fifo */
		ep_nr = REG(GOLDFISH_UDC_INDEX_REG);
		offset = offset + ep_nr;
	} else if (offset == GOLDFISH_UDC_OUT_CSR1_REG) { /* read fifo */
		ep_nr = REG(GOLDFISH_UDC_INDEX_REG);
		offset = offset + ep_nr;
	}
	val = guestOSRead(offset);
	dbg("goldfish_usbgadget_read:offset=0x%x val=0x%x\n", offset, val);
	return val;
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
		if (offset == GOLDFISH_UDC_USB_INT_REG) {
			if (value == GOLDFISH_UDC_USB_INT_VBUS_ON || value == GOLDFISH_UDC_USB_INT_VBUS_OFF) {
				sendACK(0);
			}
		}
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
			value &= ~GOLDFISH_UDC_EP0_CSR_DE;
			//ep0_done(GOLDFISH_UDC_EP0_FIFO_REG);
			guestOSWrite(offset, value);
			sendACK(0);
		} else {
			guestOSWrite(offset, value);
		}
	} else if (offset == GOLDFISH_UDC_IN_CSR1_REG) { /* write fifo */
		int baseAddr;
		int regVal;
		int ep_nr;
		ep_nr = REG(GOLDFISH_UDC_INDEX_REG);
		baseAddr = offset + ep_nr;
		REGSET(baseAddr, value);
		if ((value & GOLDFISH_UDC_ICSR1_PKTRDY)) {
			uint32_t fifoAddr = getFifoAddr(ep_nr);
			value &= ~(GOLDFISH_UDC_ICSR1_PKTRDY|GOLDFISH_UDC_OCSR1_PKTRDY);
			//ep_write_done(fifoAddr, ep_nr);
			guestOSWrite(baseAddr, value);
			sendACK(ep_nr);
		} else {
			guestOSWrite(baseAddr, value);
		}
	} else if (offset == GOLDFISH_UDC_OUT_CSR1_REG) { /* read fifo */
		int baseAddr;
		int regVal;
		int ret;
		int ep_nr;
		ep_nr = REG(GOLDFISH_UDC_INDEX_REG);
		baseAddr = offset + REG(GOLDFISH_UDC_INDEX_REG);
		if (!(value & GOLDFISH_UDC_OCSR1_PKTRDY)) {
			//ep_read_done(ep_nr);
			guestOSWrite(baseAddr, value);
			sendACK(ep_nr);
		} else {
			guestOSWrite(baseAddr, value);
		}
	} else if (offset == GOLDFISH_UDC_FUNC_ADDR_REG) {
		if (value & GOLDFISH_UDC_FUNCADDR_UPDATE) {
			value &= ~GOLDFISH_UDC_FUNCADDR_UPDATE;
			dbg("setAddress:value=%d\n", value);
			guestOSWrite(offset, value);
			sendACK(0);
		} else {
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
	connectToHost(NULL);
}
