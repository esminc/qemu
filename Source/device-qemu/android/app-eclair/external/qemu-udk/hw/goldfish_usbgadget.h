/* 
*/

#ifndef _GOLDFISH_USBGADGET_H
#define _GOLDFISH_USBGADGET_H
#include "qemu-timer.h"
#include "cpu.h"
#include "arm_pic.h"
#include "goldfish_device.h"

typedef unsigned char __u8;
typedef unsigned short __le16;
#if 0
#define dbg printf
#else
#define dbg
#endif
extern int recvUsbData(char *bufp, int len);
extern int sendUsbData(char *bufp, int len);
struct usb_ctrlrequest;
extern void ep0_done(uint32_t regAddr);
extern void ep_read_done(int ep_nr);
extern void ep_write_done(uint32_t fifoAddr, int ep_nr);
extern void putSetupData(struct usb_ctrlrequest *req);
extern void putEndpData(int pid, int endpoint, const char* bufp, int len);

#endif /* _GOLDFISH_USBGADGET_H */
