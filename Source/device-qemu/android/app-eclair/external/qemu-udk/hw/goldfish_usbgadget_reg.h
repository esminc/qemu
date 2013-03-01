#ifndef _GOLDFISH_USBGADGET_REG_H
#define _GOLDFISH_USBGADGET_REG_H
#include "qemu-timer.h"
#include "cpu.h"
#include "arm_pic.h"
#include "goldfish_device.h"
extern uint32_t guestOSRead(target_phys_addr_t regAddr);
extern void guestOSWrite(target_phys_addr_t regAddr, uint32_t value);
extern void deviceWrite(target_phys_addr_t regAddr, uint32_t value);
extern uint32_t deviceRead(target_phys_addr_t regAddr);
extern int recvFifoData(target_phys_addr_t regAddr, char *p);
extern void sendData(target_phys_addr_t regAddr, char *p, int len);
extern void init_udc_deviceRegister(void);
extern uint32_t getFifoAddr(int ep_nr);
extern uint32_t getFifoCount(int ep_nr);
#define REG(regAddr)		deviceRead(regAddr)
#define REGSET(regAddr, value)	deviceWrite(regAddr, value)
#define UDC_FIFO_NUM	9

#endif
