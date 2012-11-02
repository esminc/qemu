/*
 * linux/drivers/usb/gadget/goldfish_udc.c
 *
 * Samsung S3C24xx series on-chip full speed USB device controllers
 *
 * Copyright (C) 2004-2007 Herbert Pötzl - Arnaud Patard
 *	Additional cleanups by Ben Dooks <ben-linux@fluff.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/usb.h>
#include <linux/usb/gadget.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/irqs.h>

#include <mach/hardware.h>
#ifdef DEBUG_PRINT
#define ddprintk(fmt, ...) \
do { printk(fmt , ## __VA_ARGS__); } while (0)
#else
#define ddprintk(fmt, ...) do {} while(0)
#endif

#ifdef TODO
#include <plat/regs-udc.h>
#include <plat/udc.h>
#else
enum goldfish_udc_cmd_e {
        GOLDFISH_UDC_P_ENABLE    = 1,    /* Pull-up enable        */
        GOLDFISH_UDC_P_DISABLE   = 2,    /* Pull-up disable       */
        GOLDFISH_UDC_P_RESET     = 3,    /* UDC reset, in case of */
};

struct goldfish_udc_mach_info {
	uint32_t base;
	uint32_t irq;
        void    (*udc_command)(enum goldfish_udc_cmd_e);
        void    (*vbus_draw)(unsigned int ma);
        unsigned int vbus_pin;
        unsigned char vbus_pin_inverted;
};

// TODO
#define GOLDFISH_UDC_USB_INT_REG	0x00
#define GOLDFISH_UDC_EP_INT_REG		0x01
#define GOLDFISH_UDC_EP0_FIFO_CNT_REG   0x0010 /* 64  */
#define GOLDFISH_UDC_EP1_FIFO_CNT_REG   0x0011 
#define GOLDFISH_UDC_EP2_FIFO_CNT_REG   0x0012
#define GOLDFISH_UDC_EP3_FIFO_CNT_REG   0x0013
#define GOLDFISH_UDC_EP4_FIFO_CNT_REG   0x0014
#define GOLDFISH_UDC_EP5_FIFO_CNT_REG   0x0015
#define GOLDFISH_UDC_EP6_FIFO_CNT_REG   0x0016
#define GOLDFISH_UDC_EP7_FIFO_CNT_REG   0x0017

#define GOLDFISH_UDC_EP0_FIFO_REG       0x0020 
#define GOLDFISH_UDC_EP1_FIFO_REG       0x0021
#define GOLDFISH_UDC_EP2_FIFO_REG       0x0022
#define GOLDFISH_UDC_EP3_FIFO_REG       0x0023
#define GOLDFISH_UDC_EP4_FIFO_REG       0x0024
#define GOLDFISH_UDC_EP5_FIFO_REG       0x0025
#define GOLDFISH_UDC_EP6_FIFO_REG       0x0026
#define GOLDFISH_UDC_EP7_FIFO_REG       0x0027

#define GOLDFISH_UDC_EP0_CSR_REG        0x0030
#define GOLDFISH_UDC_EP1_CSR_REG        0x0031
#define GOLDFISH_UDC_EP2_CSR_REG        0x0032
#define GOLDFISH_UDC_EP3_CSR_REG        0x0033
#define GOLDFISH_UDC_EP4_CSR_REG        0x0034
#define GOLDFISH_UDC_EP5_CSR_REG        0x0035
#define GOLDFISH_UDC_EP6_CSR_REG        0x0036
#define GOLDFISH_UDC_EP7_CSR_REG        0x0037

#define GOLDFISH_UDC_INDEX_REG		0x0040
#define GOLDFISH_UDC_IN_CSR1_REG	0x0050
#define GOLDFISH_UDC_OUT_CSR1_REG	0x0060

#define GOLDFISH_UDC_ICSR1_PKTRDY	(1<<0)
#define GOLDFISH_UDC_OCSR1_PKTRDY	(1<<1)

#define GOLDFISH_UDC_EP0_CSR_OPKRDY     (1<<0)
#define GOLDFISH_UDC_EP0_CSR_IPKRDY     (1<<1)
#define GOLDFISH_UDC_EP0_CSR_SENTST     (1<<2)
#define GOLDFISH_UDC_EP0_CSR_DE         (1<<3)
#define GOLDFISH_UDC_EP0_CSR_SE         (1<<4)
#define GOLDFISH_UDC_EP0_CSR_SENDSTL    (1<<5)
#define GOLDFISH_UDC_EP0_CSR_SOPKTRDY   (1<<6)
#define GOLDFISH_UDC_EP0_CSR_SSE        (1<<7)

/*
 * GOLDFISH_UDC_USB_INT_REG
 */
#define GOLDFISH_UDC_USB_INT_VBUS_ON    0x01
#define GOLDFISH_UDC_USB_INT_VBUS_OFF   0x02

#define GOLDFISH_UDC_INT_EP0            (1<<0)
#define GOLDFISH_UDC_INT_EP1            (1<<1)
#define GOLDFISH_UDC_INT_EP2            (1<<2)
#define GOLDFISH_UDC_INT_EP3            (1<<3)
#define GOLDFISH_UDC_INT_EP4            (1<<4)

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
#define GOLDFISH_UDC_FUNC_ADDR_REG      0x100

/* flags */
#define GOLDFISH_UDC_FUNCADDR_UPDATE    0x10000000

#endif

#include "goldfish_udc.h"

#define DRIVER_DESC	"GOLDFISH USB Device Controller Gadget"
#define DRIVER_VERSION	"22 Dec 2012"
#define DRIVER_AUTHOR	"Takashi Mori <kanetugu2001@gmail.com> "

static const char		gadget_name[] = "goldfish_usbgadget";
static const char		driver_desc[] = DRIVER_DESC;

static struct goldfish_udc	*the_controller;
static struct clk		*udc_clock;
static struct clk		*usb_bus_clock;
static void __iomem		*base_addr;
static u64			rsrc_start;
static u64			rsrc_len;
static struct dentry		*goldfish_udc_debugfs_root;

static inline u32 udc_read(u32 reg)
{
	return readb(base_addr + reg);
}

static inline void udc_write(u32 value, u32 reg)
{
	writeb(value, base_addr + reg);
}

static inline void udc_writeb(void __iomem *base, u32 value, u32 reg)
{
	writeb(value, base + reg);
}

static struct goldfish_udc_mach_info *udc_info;

/*************************** DEBUG FUNCTION ***************************/
#define DEBUG_NORMAL	1
#define DEBUG_VERBOSE	2

#ifdef CONFIG_USB_GOLDFISH_DEBUG
#define USB_GOLDFISH_DEBUG_LEVEL 0

static uint32_t goldfish_ticks = 0;

static int dprintk(int level, const char *fmt, ...)
{
	static char printk_buf[1024];
	static long prevticks;
	static int invocation;
	va_list args;
	int len;

	if (level > USB_GOLDFISH_DEBUG_LEVEL)
		return 0;

	if (goldfish_ticks != prevticks) {
		prevticks = goldfish_ticks;
		invocation = 0;
	}

	len = scnprintf(printk_buf,
			sizeof(printk_buf), "%1lu.%02d USB: ",
			prevticks, invocation++);

	va_start(args, fmt);
	len = vscnprintf(printk_buf+len,
			sizeof(printk_buf)-len, fmt, args);
	va_end(args);

	return printk(KERN_DEBUG "%s", printk_buf);
}
#else
static int dprintk(int level, const char *fmt, ...)
{
	return 0;
}
#endif
static int goldfish_udc_debugfs_seq_show(struct seq_file *m, void *p)
{
#ifdef TODO
	u32 addr_reg,pwr_reg,ep_int_reg,usb_int_reg;
	u32 ep_int_en_reg, usb_int_en_reg, ep0_csr;
	u32 ep1_i_csr1,ep1_i_csr2,ep1_o_csr1,ep1_o_csr2;
	u32 ep2_i_csr1,ep2_i_csr2,ep2_o_csr1,ep2_o_csr2;

	addr_reg       = udc_read(GOLDFISH_UDC_FUNC_ADDR_REG);
	pwr_reg        = udc_read(GOLDFISH_UDC_PWR_REG);
	ep_int_reg     = udc_read(GOLDFISH_UDC_EP_INT_REG);
	usb_int_reg    = udc_read(GOLDFISH_UDC_USB_INT_REG);
	ep_int_en_reg  = udc_read(GOLDFISH_UDC_EP_INT_EN_REG);
	usb_int_en_reg = udc_read(GOLDFISH_UDC_USB_INT_EN_REG);
	udc_write(0, GOLDFISH_UDC_INDEX_REG);
	ep0_csr        = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
	udc_write(1, GOLDFISH_UDC_INDEX_REG);
	ep1_i_csr1     = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
	ep1_i_csr2     = udc_read(GOLDFISH_UDC_IN_CSR2_REG);
	ep1_o_csr1     = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
	ep1_o_csr2     = udc_read(GOLDFISH_UDC_IN_CSR2_REG);
	udc_write(2, GOLDFISH_UDC_INDEX_REG);
	ep2_i_csr1     = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
	ep2_i_csr2     = udc_read(GOLDFISH_UDC_IN_CSR2_REG);
	ep2_o_csr1     = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
	ep2_o_csr2     = udc_read(GOLDFISH_UDC_IN_CSR2_REG);

	seq_printf(m, "FUNC_ADDR_REG  : 0x%04X\n"
		 "PWR_REG        : 0x%04X\n"
		 "EP_INT_REG     : 0x%04X\n"
		 "USB_INT_REG    : 0x%04X\n"
		 "EP_INT_EN_REG  : 0x%04X\n"
		 "USB_INT_EN_REG : 0x%04X\n"
		 "EP0_CSR        : 0x%04X\n"
		 "EP1_I_CSR1     : 0x%04X\n"
		 "EP1_I_CSR2     : 0x%04X\n"
		 "EP1_O_CSR1     : 0x%04X\n"
		 "EP1_O_CSR2     : 0x%04X\n"
		 "EP2_I_CSR1     : 0x%04X\n"
		 "EP2_I_CSR2     : 0x%04X\n"
		 "EP2_O_CSR1     : 0x%04X\n"
		 "EP2_O_CSR2     : 0x%04X\n",
			addr_reg,pwr_reg,ep_int_reg,usb_int_reg,
			ep_int_en_reg, usb_int_en_reg, ep0_csr,
			ep1_i_csr1,ep1_i_csr2,ep1_o_csr1,ep1_o_csr2,
			ep2_i_csr1,ep2_i_csr2,ep2_o_csr1,ep2_o_csr2
		);
#endif

	return 0;
}

static int goldfish_udc_debugfs_fops_open(struct inode *inode,
					 struct file *file)
{
	return single_open(file, goldfish_udc_debugfs_seq_show, NULL);
}

static const struct file_operations goldfish_udc_debugfs_fops = {
	.open		= goldfish_udc_debugfs_fops_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.owner		= THIS_MODULE,
};

/* io macros */

static inline void goldfish_udc_clear_ep0_opr(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	udc_writeb(base, GOLDFISH_UDC_EP0_CSR_SOPKTRDY,
			GOLDFISH_UDC_EP0_CSR_REG);
#endif
}

static inline void goldfish_udc_clear_ep0_sst(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	writeb(0x00, base + GOLDFISH_UDC_EP0_CSR_REG);
#endif
}

static inline void goldfish_udc_clear_ep0_se(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	udc_writeb(base, GOLDFISH_UDC_EP0_CSR_SSE, GOLDFISH_UDC_EP0_CSR_REG);
#endif
}

static inline void goldfish_udc_set_ep0_ipr(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
#endif
	udc_writeb(base, GOLDFISH_UDC_EP0_CSR_IPKRDY, GOLDFISH_UDC_EP0_CSR_REG);
}

static inline void goldfish_udc_set_ep0_de(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	udc_writeb(base, GOLDFISH_UDC_EP0_CSR_DE, GOLDFISH_UDC_EP0_CSR_REG);
#endif
}

inline void goldfish_udc_set_ep0_ss(void __iomem *b)
{
#ifdef TODO
	udc_writeb(b, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	udc_writeb(b, GOLDFISH_UDC_EP0_CSR_SENDSTL, GOLDFISH_UDC_EP0_CSR_REG);
#endif
}

static inline void goldfish_udc_set_ep0_de_out(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
#endif

	udc_writeb(base,(GOLDFISH_UDC_EP0_CSR_SOPKTRDY
				| GOLDFISH_UDC_EP0_CSR_DE),
			GOLDFISH_UDC_EP0_CSR_REG);
}

static inline void goldfish_udc_set_ep0_sse_out(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	udc_writeb(base, (GOLDFISH_UDC_EP0_CSR_SOPKTRDY
				| GOLDFISH_UDC_EP0_CSR_SSE),
			GOLDFISH_UDC_EP0_CSR_REG);
#endif
}

static inline void goldfish_udc_set_ep0_de_in(void __iomem *base)
{
#ifdef TODO
	udc_writeb(base, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	udc_writeb(base, (GOLDFISH_UDC_EP0_CSR_IPKRDY
			| GOLDFISH_UDC_EP0_CSR_DE),
		GOLDFISH_UDC_EP0_CSR_REG);
#else
	udc_writeb(base, 
		(GOLDFISH_UDC_EP0_CSR_IPKRDY | GOLDFISH_UDC_EP0_CSR_DE),
		GOLDFISH_UDC_EP0_CSR_REG);
#endif
}

/*------------------------- I/O ----------------------------------*/

/*
 *	goldfish_udc_done
 */
static void goldfish_udc_done(struct goldfish_ep *ep,
		struct goldfish_request *req, int status)
{
	unsigned halted = ep->halted;

	list_del_init(&req->queue);

	if (likely (req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	ep->halted = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->halted = halted;
}

static void goldfish_udc_nuke(struct goldfish_udc *udc,
		struct goldfish_ep *ep, int status)
{
	/* Sanity check */
	if (&ep->queue == NULL)
		return;

	while (!list_empty (&ep->queue)) {
		struct goldfish_request *req;
		req = list_entry (ep->queue.next, struct goldfish_request,
				queue);
		goldfish_udc_done(ep, req, status);
	}
}

static inline void goldfish_udc_clear_ep_state(struct goldfish_udc *dev)
{
	unsigned i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */

	for (i = 1; i < GOLDFISH_ENDPOINTS; i++)
		goldfish_udc_nuke(dev, &dev->ep[i], -ECONNABORTED);
}

static inline int goldfish_udc_fifo_count_out(int countReg)
{
#ifdef TODO
	int tmp;

	tmp = udc_read(GOLDFISH_UDC_OUT_FIFO_CNT2_REG) << 8;
	tmp |= udc_read(GOLDFISH_UDC_OUT_FIFO_CNT1_REG);
	return tmp;
#else
	//return udc_read(GOLDFISH_UDC_EP0_FIFO_CNT_REG);
	return udc_read(countReg);
#endif
}

/*
 *	goldfish_udc_write_packet
 */
static inline int goldfish_udc_write_packet(int fifo,
		struct goldfish_request *req,
		unsigned max)
{
	unsigned len = min(req->req.length - req->req.actual, max);
	u8 *buf = req->req.buf + req->req.actual;

	prefetch(buf);

	ddprintk("%s %d %d %d %d\n", __func__,
		req->req.actual, req->req.length, len, req->req.actual + len);
	ddprintk("max=%d\n", max);

	req->req.actual += len;

	udelay(5);
	writesb(base_addr + fifo, buf, len);
	return len;
}

/*
 *	goldfish_udc_write_fifo
 *
 * return:  0 = still running, 1 = completed, negative = errno
 */
static int goldfish_udc_write_fifo(struct goldfish_ep *ep,
		struct goldfish_request *req)
{
	unsigned	count;
	int		is_last;
	u32		idx;
	int		fifo_reg;
	u32		ep_csr;
	ddprintk("goldfish_udc_write_fifo:enter\n");

	idx = ep->bEndpointAddress & 0x7F;
	switch (idx) {
	default:
		idx = 0;
	case 0:
		fifo_reg = GOLDFISH_UDC_EP0_FIFO_REG;
		break;
	case 1:
		fifo_reg = GOLDFISH_UDC_EP1_FIFO_REG;
		break;
	case 2:
		fifo_reg = GOLDFISH_UDC_EP2_FIFO_REG;
		break;
	case 3:
		fifo_reg = GOLDFISH_UDC_EP3_FIFO_REG;
		break;
	case 4:
		fifo_reg = GOLDFISH_UDC_EP4_FIFO_REG;
		break;
	}
	count = goldfish_udc_write_packet(fifo_reg, req, ep->ep.maxpacket);

	/* last packet is often short (sometimes a zlp) */
	if (count != ep->ep.maxpacket)
		is_last = 1;
	else if (req->req.length != req->req.actual || req->req.zero)
		is_last = 0;
	else
		is_last = 2;

	/* Only ep0 debug messages are interesting */
#ifdef TODO
	if (idx == 0)
#endif
		ddprintk(
			"Written ep%d %d.%d of %d b [last %d,z %d]\n",
			idx, count, req->req.actual, req->req.length,
			is_last, req->req.zero);

	if (is_last) {
ddprintk("is_last!=0\n");
		/* The order is important. It prevents sending 2 packets
		 * at the same time */

		if (idx == 0) {
#ifdef TODO
			/* Reset signal => no need to say 'data sent' */
			if (! (udc_read(GOLDFISH_UDC_USB_INT_REG)
					& GOLDFISH_UDC_USBINT_RESET))
				goldfish_udc_set_ep0_de_in(base_addr);
#else
			goldfish_udc_set_ep0_de_in(base_addr);
#endif
			ep->dev->ep0state=EP0_IDLE;
		} else {
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			ep_csr = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			udc_write(ep_csr | GOLDFISH_UDC_ICSR1_PKTRDY,
					GOLDFISH_UDC_IN_CSR1_REG);
		}

		goldfish_udc_done(ep, req, 0);
		is_last = 1;
	} else {
		if (idx == 0) {
			/* Reset signal => no need to say 'data sent' */
#ifdef TODO
			if (! (udc_read(GOLDFISH_UDC_USB_INT_REG)
					& GOLDFISH_UDC_USBINT_RESET))
				goldfish_udc_set_ep0_ipr(base_addr);
#else
			goldfish_udc_set_ep0_ipr(base_addr);
#endif
		} else {
ddprintk("is_last=0\n");
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			ep_csr = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			udc_write(ep_csr | GOLDFISH_UDC_ICSR1_PKTRDY,
					GOLDFISH_UDC_IN_CSR1_REG);
		}
	}
	return is_last;
}

static inline int goldfish_udc_read_packet(int fifo, u8 *buf,
		struct goldfish_request *req, unsigned avail)
{
	unsigned len;

	len = min(req->req.length - req->req.actual, avail);
	req->req.actual += len;

	readsb(fifo + base_addr, buf, len);
	return len;
}

/*
 * return:  0 = still running, 1 = queue empty, negative = errno
 */
static int goldfish_udc_read_fifo(struct goldfish_ep *ep,
				 struct goldfish_request *req)
{
	u8		*buf;
	u32		ep_csr;
	unsigned	bufferspace;
	int		is_last=1;
	unsigned	avail;
	int		fifo_count = 0;
	u32		idx;
	int		fifo_reg;
	int		count_reg;

	idx = ep->bEndpointAddress & 0x7F;

	switch (idx) {
	default:
		idx = 0;
	case 0:
		fifo_reg = GOLDFISH_UDC_EP0_FIFO_REG;
		count_reg = GOLDFISH_UDC_EP0_FIFO_CNT_REG;
		break;
	case 1:
		fifo_reg = GOLDFISH_UDC_EP1_FIFO_REG;
		count_reg = GOLDFISH_UDC_EP1_FIFO_CNT_REG;
		break;
	case 2:
		fifo_reg = GOLDFISH_UDC_EP2_FIFO_REG;
		count_reg = GOLDFISH_UDC_EP2_FIFO_CNT_REG;
		break;
	case 3:
		fifo_reg = GOLDFISH_UDC_EP3_FIFO_REG;
		count_reg = GOLDFISH_UDC_EP3_FIFO_CNT_REG;
		break;
	case 4:
		fifo_reg = GOLDFISH_UDC_EP4_FIFO_REG;
		count_reg = GOLDFISH_UDC_EP4_FIFO_CNT_REG;
		break;
	}
	if (!req->req.length)
		return 1;

	buf = req->req.buf + req->req.actual;
	bufferspace = req->req.length - req->req.actual;
	if (!bufferspace) {
		dprintk(DEBUG_NORMAL, "%s: buffer full!\n", __func__);
		return -1;
	}

#ifdef TODO
	udc_write(idx, GOLDFISH_UDC_INDEX_REG);
#endif

	fifo_count = goldfish_udc_fifo_count_out(count_reg);
	ddprintk("%s fifo count : %d\n", __func__, fifo_count);

	if (fifo_count > ep->ep.maxpacket)
		avail = ep->ep.maxpacket;
	else
		avail = fifo_count;

	fifo_count = goldfish_udc_read_packet(fifo_reg, buf, req, avail);

	/* checking this with ep0 is not accurate as we already
	 * read a control request
	 **/
	if (idx != 0 && fifo_count < ep->ep.maxpacket) {
		is_last = 1;
		/* overflowed this request?  flush extra data */
		if (fifo_count != avail)
			req->req.status = -EOVERFLOW;
	} else {
		is_last = (req->req.length <= req->req.actual) ? 1 : 0;
	}

#ifdef TODO
	udc_write(idx, GOLDFISH_UDC_INDEX_REG);
#endif
	//fifo_count = goldfish_udc_fifo_count_out();
	fifo_count = goldfish_udc_fifo_count_out(count_reg);

	/* Only ep0 debug messages are interesting */
	if (idx == 0)
		ddprintk("%s fifo count : %d [last %d]\n",
			__func__, fifo_count,is_last);

	if (is_last) {
		if (idx == 0) {
#ifdef TODO
			goldfish_udc_set_ep0_de_out(base_addr);
			ep->dev->ep0state = EP0_IDLE;
#endif
		} else {
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			ep_csr = udc_read(GOLDFISH_UDC_OUT_CSR1_REG);
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			udc_write(ep_csr & ~GOLDFISH_UDC_OCSR1_PKTRDY,
					GOLDFISH_UDC_OUT_CSR1_REG);
		}

		goldfish_udc_done(ep, req, 0);
	} else {
		if (idx == 0) {
#ifdef TODO
			goldfish_udc_clear_ep0_opr(base_addr);
#endif
		} else {
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			ep_csr = udc_read(GOLDFISH_UDC_OUT_CSR1_REG);
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			udc_write(ep_csr & ~GOLDFISH_UDC_OCSR1_PKTRDY,
					GOLDFISH_UDC_OUT_CSR1_REG);
		}
	}

	return is_last;
}

static int goldfish_udc_read_fifo_crq(struct usb_ctrlrequest *crq)
{
	unsigned char *outbuf = (unsigned char*)crq;
	int bytes_read = 0;

#ifdef TODO
	udc_write(0, GOLDFISH_UDC_INDEX_REG);
#endif

	bytes_read = goldfish_udc_fifo_count_out(GOLDFISH_UDC_EP0_FIFO_CNT_REG);

	ddprintk("%s: fifo_count=%d\n", __func__, bytes_read);

	if (bytes_read > sizeof(struct usb_ctrlrequest))
		bytes_read = sizeof(struct usb_ctrlrequest);

	readsb(GOLDFISH_UDC_EP0_FIFO_REG + base_addr, outbuf, bytes_read);
	ddprintk("%s: len=%d %02x:%02x {%x,%x,%x}\n", __func__,
		bytes_read, crq->bRequest, crq->bRequestType,
		crq->wValue, crq->wIndex, crq->wLength);

	return bytes_read;
}

static int goldfish_udc_get_status(struct goldfish_udc *dev,
		struct usb_ctrlrequest *crq)
{
	u16 status = 0;
	u8 ep_num = crq->wIndex & 0x7F;
	u8 is_in = crq->wIndex & USB_DIR_IN;

	switch (crq->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_INTERFACE:
		break;

	case USB_RECIP_DEVICE:
		status = dev->devstatus;
		break;

	case USB_RECIP_ENDPOINT:
		if (ep_num > 4 || crq->wLength > 2)
			return 1;

#ifdef TODO
		if (ep_num == 0) {
			udc_write(0, GOLDFISH_UDC_INDEX_REG);
			status = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
			status = status & GOLDFISH_UDC_EP0_CSR_SENDSTL;
		} else {
			udc_write(ep_num, GOLDFISH_UDC_INDEX_REG);
			if (is_in) {
				status = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
				status = status & GOLDFISH_UDC_ICSR1_SENDSTL;
			} else {
				status = udc_read(GOLDFISH_UDC_OUT_CSR1_REG);
				status = status & GOLDFISH_UDC_OCSR1_SENDSTL;
			}
		}
#endif

		status = status ? 1 : 0;
		break;

	default:
		return 1;
	}

	/* Seems to be needed to get it working. ouch :( */
	udelay(5);
#ifdef TODO
	udc_write(status & 0xFF, GOLDFISH_UDC_EP0_FIFO_REG);
	udc_write(status >> 8, GOLDFISH_UDC_EP0_FIFO_REG);
#endif
	goldfish_udc_set_ep0_de_in(base_addr);

	return 0;
}
/*------------------------- usb state machine -------------------------------*/
static int goldfish_udc_set_halt(struct usb_ep *_ep, int value);

static void goldfish_udc_handle_ep0_idle(struct goldfish_udc *dev,
					struct goldfish_ep *ep,
					struct usb_ctrlrequest *crq,
					u32 ep0csr)
{
	int len, ret, tmp;

	ddprintk("goldfish_udc_handle_ep0_idle:enter\n");
	/* start control request? */
#ifdef TODO
	if (!(ep0csr & GOLDFISH_UDC_EP0_CSR_OPKRDY))
		return;

	goldfish_udc_nuke(dev, ep, -EPROTO);
#endif

	len = goldfish_udc_read_fifo_crq(crq);
	if (len != sizeof(*crq)) {
		ddprintk("setup begin: fifo READ ERROR"
			" wanted %d bytes got %d. Stalling out...\n",
			sizeof(*crq), len);
#ifdef TODO
		goldfish_udc_set_ep0_ss(base_addr);
#endif
		return;
	}

	ddprintk("bRequest = %d bRequestType %d wLength = %d\n",
		crq->bRequest, crq->bRequestType, crq->wLength);
	/* cope with automagic for some standard requests. */
	dev->req_std = (crq->bRequestType & USB_TYPE_MASK)
		== USB_TYPE_STANDARD;
	dev->req_config = 0;
	dev->req_pending = 1;

	switch (crq->bRequest) {
	case USB_REQ_SET_CONFIGURATION:
		ddprintk("USB_REQ_SET_CONFIGURATION ... \n");

		if (crq->bRequestType == USB_RECIP_DEVICE) {
			dev->req_config = 1;
			goldfish_udc_set_ep0_de_out(base_addr);
		}
		break;

	case USB_REQ_SET_INTERFACE:
		ddprintk("USB_REQ_SET_INTERFACE ... \n");

		if (crq->bRequestType == USB_RECIP_INTERFACE) {
			dev->req_config = 1;
			goldfish_udc_set_ep0_de_out(base_addr);
		}
		break;

	case USB_REQ_SET_ADDRESS:
		ddprintk("USB_REQ_SET_ADDRESS ... \n");

		if (crq->bRequestType == USB_RECIP_DEVICE) {
			tmp = crq->wValue & 0x7F;
			dev->address = tmp;
			udc_write((tmp | GOLDFISH_UDC_FUNCADDR_UPDATE),
					GOLDFISH_UDC_FUNC_ADDR_REG);
			goldfish_udc_set_ep0_de_out(base_addr);
			return;
		}
		break;

	case USB_REQ_GET_STATUS:
		ddprintk("USB_REQ_GET_STATUS ... \n");
		goldfish_udc_clear_ep0_opr(base_addr);

		if (dev->req_std) {
			if (!goldfish_udc_get_status(dev, crq)) {
				return;
			}
		}
		break;

	case USB_REQ_CLEAR_FEATURE:
		goldfish_udc_clear_ep0_opr(base_addr);

		if (crq->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq->wValue != USB_ENDPOINT_HALT || crq->wLength != 0)
			break;

		goldfish_udc_set_halt(&dev->ep[crq->wIndex & 0x7f].ep, 0);
		goldfish_udc_set_ep0_de_out(base_addr);
		return;

	case USB_REQ_SET_FEATURE:
		goldfish_udc_clear_ep0_opr(base_addr);

		if (crq->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq->wValue != USB_ENDPOINT_HALT || crq->wLength != 0)
			break;

		goldfish_udc_set_halt(&dev->ep[crq->wIndex & 0x7f].ep, 1);
		goldfish_udc_set_ep0_de_out(base_addr);
		return;

	default:
		goldfish_udc_clear_ep0_opr(base_addr);
		break;
	}
	if (crq->bRequestType & USB_DIR_IN)
		dev->ep0state = EP0_IN_DATA_PHASE;
	else
		dev->ep0state = EP0_OUT_DATA_PHASE;
	
	if (dev->driver) {
		ddprintk("dev=0x%x\n", dev);
		ddprintk("driver=0x%x\n", dev->driver);
		ddprintk("setup=%p\n", dev->driver->setup);
	}
	ddprintk("gadget=%p\n", dev->gadget);
	ret = dev->driver->setup(&dev->gadget, crq);
#ifdef TODO
	if (ret < 0) {
		if (dev->req_config) {
			ddprintk("config change %02x fail %d?\n",
				crq->bRequest, ret);
			return;
		}

		if (ret == -EOPNOTSUPP)
			ddprintk("Operation not supported\n");
		else
			ddprintk(
				"dev->driver->setup failed. (%d)\n", ret);

		udelay(5);
		goldfish_udc_set_ep0_ss(base_addr);
		goldfish_udc_set_ep0_de_out(base_addr);
		dev->ep0state = EP0_IDLE;
		/* deferred i/o == no response yet */
	} else if (dev->req_pending) {
		ddprintk("dev->req_pending... what now?\n");
		dev->req_pending=0;
	}
#endif
	ddprintk("ep0state %s ret=%d\n", ep0states[dev->ep0state], ret);
}

static void goldfish_udc_handle_ep0(struct goldfish_udc *dev)
{
	u32			ep0csr;
	struct goldfish_ep	*ep = &dev->ep[0];
	struct goldfish_request	*req;
	struct usb_ctrlrequest	crq;

	if (list_empty(&ep->queue))
		req = NULL;
	else
		req = list_entry(ep->queue.next, struct goldfish_request, queue);

	/* We make the assumption that GOLDFISH_UDC_IN_CSR1_REG equal to
	 * GOLDFISH_UDC_EP0_CSR_REG when index is zero */

#ifdef TODO
	udc_write(0, GOLDFISH_UDC_INDEX_REG);
	ep0csr = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
#endif

	ddprintk("ep0csr %x ep0state %s\n",
		ep0csr, ep0states[dev->ep0state]);

	/* clear stall status */
#ifdef TODO
	if (ep0csr & GOLDFISH_UDC_EP0_CSR_SENTSTL) {
		goldfish_udc_nuke(dev, ep, -EPIPE);
		dprintk(DEBUG_NORMAL, "... clear SENT_STALL ...\n");
		goldfish_udc_clear_ep0_sst(base_addr);
		dev->ep0state = EP0_IDLE;
		return;
	}

	/* clear setup end */
	if (ep0csr & GOLDFISH_UDC_EP0_CSR_SE) {
		dprintk(DEBUG_NORMAL, "... serviced SETUP_END ...\n");
		goldfish_udc_nuke(dev, ep, 0);
		goldfish_udc_clear_ep0_se(base_addr);
		dev->ep0state = EP0_IDLE;
	}
#endif

	switch (dev->ep0state) {
	case EP0_IDLE:
		ddprintk("EP0_IDLE\n");
		goldfish_udc_handle_ep0_idle(dev, ep, &crq, ep0csr);
		break;

#ifdef TODO
	case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
		dprintk(DEBUG_NORMAL, "EP0_IN_DATA_PHASE ... what now?\n");
		if (!(ep0csr & GOLDFISH_UDC_EP0_CSR_IPKRDY) && req) {
			goldfish_udc_write_fifo(ep, req);
		}
		break;

	case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
		dprintk(DEBUG_NORMAL, "EP0_OUT_DATA_PHASE ... what now?\n");
		if ((ep0csr & GOLDFISH_UDC_EP0_CSR_OPKRDY) && req ) {
			goldfish_udc_read_fifo(ep,req);
		}
		break;

	case EP0_END_XFER:
		dprintk(DEBUG_NORMAL, "EP0_END_XFER ... what now?\n");
		dev->ep0state = EP0_IDLE;
		break;

	case EP0_STALL:
		dprintk(DEBUG_NORMAL, "EP0_STALL ... what now?\n");
		dev->ep0state = EP0_IDLE;
		break;
#endif
	}
}

/*
 *	handle_ep - Manage I/O endpoints
 */

static void goldfish_udc_handle_ep(struct goldfish_ep *ep)
{
	struct goldfish_request	*req;
	int			is_in = ep->bEndpointAddress & USB_DIR_IN;
#ifdef TODO
	u32			ep_csr1;
#endif
	u32			idx;

	if (likely (!list_empty(&ep->queue)))
		req = list_entry(ep->queue.next,
				struct goldfish_request, queue);
	else
		req = NULL;

	idx = ep->bEndpointAddress & 0x7F;

	ddprintk("goldfish_udc_handle_ep:idx=%d is_in=%d\n", idx, is_in);
	if (is_in) {
#ifdef TODO
		udc_write(idx, GOLDFISH_UDC_INDEX_REG);
		ep_csr1 = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
		ddprintk("ep%01d write csr:%02x %d\n",
			idx, ep_csr1, req ? 1 : 0);

		if (ep_csr1 & GOLDFISH_UDC_ICSR1_SENTSTL) {
			ddprintk("st\n");
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			udc_write(ep_csr1 & ~GOLDFISH_UDC_ICSR1_SENTSTL,
					GOLDFISH_UDC_IN_CSR1_REG);
			return;
		}

		if (!(ep_csr1 & GOLDFISH_UDC_ICSR1_PKTRDY) && req) {
			goldfish_udc_write_fifo(ep,req);
		}
#else
		if (req) {
			goldfish_udc_write_fifo(ep,req);
		}
#endif
	} else {
#ifdef TODO
		udc_write(idx, GOLDFISH_UDC_INDEX_REG);
		ep_csr1 = udc_read(GOLDFISH_UDC_OUT_CSR1_REG);
		ddprintk("ep%01d rd csr:%02x\n", idx, ep_csr1);

		if (ep_csr1 & GOLDFISH_UDC_OCSR1_SENTSTL) {
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			udc_write(ep_csr1 & ~GOLDFISH_UDC_OCSR1_SENTSTL,
					GOLDFISH_UDC_OUT_CSR1_REG);
			return;
		}

		if ((ep_csr1 & GOLDFISH_UDC_OCSR1_PKTRDY) && req) {
			goldfish_udc_read_fifo(ep,req);
		}
#else
		if (req) {
			goldfish_udc_read_fifo(ep,req);
		}
#endif
	}
}

#ifdef TODO
#include <mach/regs-irq.h>
#endif

static int goldfish_udc_pullup(struct usb_gadget *gadget, int is_on);
/*
 *	goldfish_udc_irq - interrupt handler
 */
static irqreturn_t goldfish_udc_irq(int dummy, void *_dev)
{
	struct goldfish_udc *dev = _dev;
	int usb_status;
	int usbd_status;
	int pwr_reg;
	int ep0csr;
	int i;
	u32 idx;
	unsigned long flags;

	ddprintk("goldfish_udc_irq:dev=0x%x\n", dev);
	spin_lock_irqsave(&dev->lock, flags);
	/* Driver connected ? */
	if (!dev->driver) {
		/* Clear interrupts */
		udc_write(udc_read(GOLDFISH_UDC_USB_INT_REG),
				GOLDFISH_UDC_USB_INT_REG);
		udc_write(udc_read(GOLDFISH_UDC_EP_INT_REG),
				GOLDFISH_UDC_EP_INT_REG);
	}

#ifdef TODO
	/* Save index */
	idx = udc_read(GOLDFISH_UDC_INDEX_REG);
#endif

	/* Read status registers */
	usb_status = udc_read(GOLDFISH_UDC_USB_INT_REG);
	usbd_status = udc_read(GOLDFISH_UDC_EP_INT_REG);
	if (usb_status & GOLDFISH_UDC_USB_INT_VBUS_ON) {
		udc_write(GOLDFISH_UDC_USB_INT_VBUS_ON,
				GOLDFISH_UDC_USB_INT_REG);
		goldfish_udc_pullup(&dev->gadget, 0);
	} else if (usb_status & GOLDFISH_UDC_USB_INT_VBUS_OFF) {
		udc_write(GOLDFISH_UDC_USB_INT_VBUS_OFF,
				GOLDFISH_UDC_USB_INT_REG);
		goldfish_udc_pullup(&dev->gadget, 1);
	}
	ddprintk("enter:usb_status=0x%x usbd_status=0x%x\n", usb_status, usbd_status);
#ifdef TODO
	pwr_reg = udc_read(GOLDFISH_UDC_PWR_REG);

	udc_writeb(base_addr, GOLDFISH_UDC_INDEX_EP0, GOLDFISH_UDC_INDEX_REG);
	ep0csr = udc_read(GOLDFISH_UDC_IN_CSR1_REG);

	dprintk(DEBUG_NORMAL, "usbs=%02x, usbds=%02x, pwr=%02x ep0csr=%02x\n",
		usb_status, usbd_status, pwr_reg, ep0csr);

	/*
	 * Now, handle interrupts. There's two types :
	 * - Reset, Resume, Suspend coming -> usb_int_reg
	 * - EP -> ep_int_reg
	 */

	/* RESET */
	if (usb_status & GOLDFISH_UDC_USBINT_RESET) {
		/* two kind of reset :
		 * - reset start -> pwr reg = 8
		 * - reset end   -> pwr reg = 0
		 **/
		dprintk(DEBUG_NORMAL, "USB reset csr %x pwr %x\n",
			ep0csr, pwr_reg);

		dev->gadget.speed = USB_SPEED_UNKNOWN;
		udc_write(0x00, GOLDFISH_UDC_INDEX_REG);
		udc_write((dev->ep[0].ep.maxpacket & 0x7ff) >> 3,
				GOLDFISH_UDC_MAXP_REG);
		dev->address = 0;

		dev->ep0state = EP0_IDLE;
		dev->gadget.speed = USB_SPEED_FULL;

		/* clear interrupt */
		udc_write(GOLDFISH_UDC_USBINT_RESET,
				GOLDFISH_UDC_USB_INT_REG);

		udc_write(idx, GOLDFISH_UDC_INDEX_REG);
		spin_unlock_irqrestore(&dev->lock, flags);
		return IRQ_HANDLED;
	}

	/* RESUME */
	if (usb_status & GOLDFISH_UDC_USBINT_RESUME) {
		dprintk(DEBUG_NORMAL, "USB resume\n");

		/* clear interrupt */
		udc_write(GOLDFISH_UDC_USBINT_RESUME,
				GOLDFISH_UDC_USB_INT_REG);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
				&& dev->driver
				&& dev->driver->resume)
			dev->driver->resume(&dev->gadget);
	}

	/* SUSPEND */
	if (usb_status & GOLDFISH_UDC_USBINT_SUSPEND) {
		dprintk(DEBUG_NORMAL, "USB suspend\n");

		/* clear interrupt */
		udc_write(GOLDFISH_UDC_USBINT_SUSPEND,
				GOLDFISH_UDC_USB_INT_REG);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN
				&& dev->driver
				&& dev->driver->suspend)
			dev->driver->suspend(&dev->gadget);

		dev->ep0state = EP0_IDLE;
	}
#endif

	/* EP */
	/* control traffic */
	/* check on ep0csr != 0 is not a good idea as clearing in_pkt_ready
	 * generate an interrupt
	 */
	if (usbd_status & GOLDFISH_UDC_INT_EP0) {
		ddprintk("USB ep0 irq\n");
		/* Clear the interrupt bit by setting it to 1 */
		udc_write(GOLDFISH_UDC_INT_EP0, GOLDFISH_UDC_EP_INT_REG);
		goldfish_udc_handle_ep0(dev);
	}

	/* endpoint data transfers */
	for (i = 1; i < GOLDFISH_ENDPOINTS; i++) {
		u32 tmp = 1 << i;
		if (usbd_status & tmp) {
			ddprintk("USB ep%d irq\n", i);

			/* Clear the interrupt bit by setting it to 1 */
			udc_write(tmp, GOLDFISH_UDC_EP_INT_REG);
			goldfish_udc_handle_ep(&dev->ep[i]);
		}
	}

#ifdef TODO
	ddprintk("irq: %d goldfish_udc_done.\n", IRQ_USBD);
	/* Restore old index */
	udc_write(idx, GOLDFISH_UDC_INDEX_REG);
#endif
	usbd_status = udc_read(GOLDFISH_UDC_EP_INT_REG);
	ddprintk("exit:usb_status=0x%x usbd_status=0x%x\n", usb_status, usbd_status);

	spin_unlock_irqrestore(&dev->lock, flags);

	return IRQ_HANDLED;
}
/*------------------------- goldfish_ep_ops ----------------------------------*/

static inline struct goldfish_ep *to_goldfish_ep(struct usb_ep *ep)
{
	return container_of(ep, struct goldfish_ep, ep);
}

static inline struct goldfish_udc *to_goldfish_udc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct goldfish_udc, gadget);
}

static inline struct goldfish_request *to_goldfish_req(struct usb_request *req)
{
	return container_of(req, struct goldfish_request, req);
}

/*
 *	goldfish_udc_ep_enable
 */
static int goldfish_udc_ep_enable(struct usb_ep *_ep,
				 const struct usb_endpoint_descriptor *desc)
{
	struct goldfish_udc	*dev;
	struct goldfish_ep	*ep;
	u32			max, tmp;
	unsigned long		flags;
	u32			csr1,csr2;
	u32			int_en_reg;

	ep = to_goldfish_ep(_ep);

	if (!_ep || !desc || ep->desc
			|| _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize) & 0x1fff;

	local_irq_save (flags);
	_ep->maxpacket = max & 0x7ff;
	ep->desc = desc;
	ep->halted = 0;
	ep->bEndpointAddress = desc->bEndpointAddress;

	/* set max packet */
#ifdef TODO
	udc_write(ep->num, GOLDFISH_UDC_INDEX_REG);
	udc_write(max >> 3, GOLDFISH_UDC_MAXP_REG);

	/* set type, direction, address; reset fifo counters */
	if (desc->bEndpointAddress & USB_DIR_IN) {
		csr1 = GOLDFISH_UDC_ICSR1_FFLUSH|GOLDFISH_UDC_ICSR1_CLRDT;
		csr2 = GOLDFISH_UDC_ICSR2_MODEIN|GOLDFISH_UDC_ICSR2_DMAIEN;

		udc_write(ep->num, GOLDFISH_UDC_INDEX_REG);
		udc_write(csr1, GOLDFISH_UDC_IN_CSR1_REG);
		udc_write(ep->num, GOLDFISH_UDC_INDEX_REG);
		udc_write(csr2, GOLDFISH_UDC_IN_CSR2_REG);
	} else {
		/* don't flush in fifo or it will cause endpoint interrupt */
		csr1 = GOLDFISH_UDC_ICSR1_CLRDT;
		csr2 = GOLDFISH_UDC_ICSR2_DMAIEN;

		udc_write(ep->num, GOLDFISH_UDC_INDEX_REG);
		udc_write(csr1, GOLDFISH_UDC_IN_CSR1_REG);
		udc_write(ep->num, GOLDFISH_UDC_INDEX_REG);
		udc_write(csr2, GOLDFISH_UDC_IN_CSR2_REG);

		csr1 = GOLDFISH_UDC_OCSR1_FFLUSH | GOLDFISH_UDC_OCSR1_CLRDT;
		csr2 = GOLDFISH_UDC_OCSR2_DMAIEN;

		udc_write(ep->num, GOLDFISH_UDC_INDEX_REG);
		udc_write(csr1, GOLDFISH_UDC_OUT_CSR1_REG);
		udc_write(ep->num, GOLDFISH_UDC_INDEX_REG);
		udc_write(csr2, GOLDFISH_UDC_OUT_CSR2_REG);
	}

	/* enable irqs */
	int_en_reg = udc_read(GOLDFISH_UDC_EP_INT_EN_REG);
	udc_write(int_en_reg | (1 << ep->num), GOLDFISH_UDC_EP_INT_EN_REG);
#endif

	/* print some debug message */
	tmp = desc->bEndpointAddress;
	ddprintk ("enable %s(%d) ep%x%s-blk max %02x\n",
		 _ep->name,ep->num, tmp,
		 desc->bEndpointAddress & USB_DIR_IN ? "in" : "out", max);

	local_irq_restore (flags);
	goldfish_udc_set_halt(_ep, 0);

	return 0;
}

/*
 * goldfish_udc_ep_disable
 */
static int goldfish_udc_ep_disable(struct usb_ep *_ep)
{
	struct goldfish_ep *ep = to_goldfish_ep(_ep);
	unsigned long flags;
	u32 int_en_reg;

	if (!_ep || !ep->desc) {
		dprintk(DEBUG_NORMAL, "%s not enabled\n",
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	local_irq_save(flags);

	dprintk(DEBUG_NORMAL, "ep_disable: %s\n", _ep->name);

	ep->desc = NULL;
	ep->halted = 1;

	goldfish_udc_nuke (ep->dev, ep, -ESHUTDOWN);

	/* disable irqs */
#ifdef TODO
	int_en_reg = udc_read(GOLDFISH_UDC_EP_INT_EN_REG);
	udc_write(int_en_reg & ~(1<<ep->num), GOLDFISH_UDC_EP_INT_EN_REG);
#endif

	local_irq_restore(flags);

	dprintk(DEBUG_NORMAL, "%s disabled\n", _ep->name);

	return 0;
}

/*
 * goldfish_udc_alloc_request
 */
static struct usb_request *
goldfish_udc_alloc_request(struct usb_ep *_ep, gfp_t mem_flags)
{
	struct goldfish_request *req;

	ddprintk("%s(%p,%d)\n", __func__, _ep, mem_flags);

	if (!_ep)
		return NULL;

	req = kzalloc (sizeof(struct goldfish_request), mem_flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD (&req->queue);
	return &req->req;
}

/*
 * goldfish_udc_free_request
 */
static void
goldfish_udc_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct goldfish_ep	*ep = to_goldfish_ep(_ep);
	struct goldfish_request	*req = to_goldfish_req(_req);

	dprintk(DEBUG_VERBOSE, "%s(%p,%p)\n", __func__, _ep, _req);

	if (!ep || !_req || (!ep->desc && _ep->name != ep0name))
		return;

	WARN_ON (!list_empty (&req->queue));
	kfree(req);
}

/*
 *	goldfish_udc_queue
 */
static int goldfish_udc_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t gfp_flags)
{
	struct goldfish_request	*req = to_goldfish_req(_req);
	struct goldfish_ep	*ep = to_goldfish_ep(_ep);
	struct goldfish_udc	*dev;
	u32			ep_csr = 0;
	int			fifo_count = 0;
	unsigned long		flags;
	int			idx;
	u32			fifo_reg;

	if (unlikely (!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		ddprintk("%s: invalid args\n", __func__);
		return -EINVAL;
	}

	dev = ep->dev;
	if (unlikely (!dev->driver
			|| dev->gadget.speed == USB_SPEED_UNKNOWN)) {
		return -ESHUTDOWN;
	}

	local_irq_save (flags);

	if (unlikely(!_req || !_req->complete
			|| !_req->buf || !list_empty(&req->queue))) {
		if (!_req)
			ddprintk("%s: 1 X X X\n", __func__);
		else {
			ddprintk("%s: 0 %01d %01d %01d\n",
				__func__, !_req->complete,!_req->buf,
				!list_empty(&req->queue));
		}

		local_irq_restore(flags);
		return -EINVAL;
	}

	idx = ep->bEndpointAddress & 0x7F;
	switch (idx) {
	default:
		idx = 0;
	case 0:
		fifo_reg = GOLDFISH_UDC_EP0_FIFO_CNT_REG;
		break;
	case 1:
		fifo_reg = GOLDFISH_UDC_EP1_FIFO_CNT_REG;
		break;
	case 2:
		fifo_reg = GOLDFISH_UDC_EP2_FIFO_CNT_REG;
		break;
	case 3:
		fifo_reg = GOLDFISH_UDC_EP3_FIFO_CNT_REG;
		break;
	case 4:
		fifo_reg = GOLDFISH_UDC_EP4_FIFO_CNT_REG;
		break;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	ddprintk("%s: ep%x len %d\n",
		 __func__, ep->bEndpointAddress, _req->length);

	if (ep->bEndpointAddress) {
		udc_write(ep->bEndpointAddress & 0x7F, GOLDFISH_UDC_INDEX_REG);

		ep_csr = udc_read((ep->bEndpointAddress & USB_DIR_IN)
				? GOLDFISH_UDC_IN_CSR1_REG
				: GOLDFISH_UDC_OUT_CSR1_REG);
		fifo_count = goldfish_udc_fifo_count_out(fifo_reg);
	} else {
#ifdef TODO
		udc_write(0, GOLDFISH_UDC_INDEX_REG);
		ep_csr = udc_read(GOLDFISH_UDC_IN_CSR1_REG);
		fifo_count = goldfish_udc_fifo_count_out();
#endif
	}

	/* kickstart this i/o queue? */
	ddprintk("%s list_empty(ep->queue)=%d endp=0x%x ep_csr=0x%x fifo_count=%d\n", 
		__func__, list_empty(&ep->queue),
		ep->bEndpointAddress, ep_csr, fifo_count);
	if (list_empty(&ep->queue) && !ep->halted) {
		if (ep->bEndpointAddress == 0 /* ep0 */) {
			switch (dev->ep0state) {
			case EP0_IN_DATA_PHASE:
#ifdef TODO
				if (!(ep_csr&GOLDFISH_UDC_EP0_CSR_IPKRDY)
						&& goldfish_udc_write_fifo(ep,
							req)) {
					dev->ep0state = EP0_IDLE;
					req = NULL;
				}
#else
				goldfish_udc_write_fifo(ep, req);
				dev->ep0state = EP0_IDLE;
				req = NULL;
#endif
				break;

			case EP0_OUT_DATA_PHASE:
#ifdef TODO
				if ((!_req->length)
					|| ((ep_csr & GOLDFISH_UDC_OCSR1_PKTRDY)
						&& goldfish_udc_read_fifo(ep,
							req))) {
					dev->ep0state = EP0_IDLE;
					req = NULL;
				}
#else
				dev->ep0state = EP0_IDLE;
				req = NULL;
#endif
				break;

			default:
				local_irq_restore(flags);
				return -EL2HLT;
			}
		} else if ((ep->bEndpointAddress & USB_DIR_IN) != 0
#ifdef TODO
				&& (!(ep_csr&GOLDFISH_UDC_OCSR1_PKTRDY))
#else
				&& ((ep_csr&GOLDFISH_UDC_OCSR1_PKTRDY))
#endif
				&& goldfish_udc_write_fifo(ep, req)) {
			ddprintk("%s write ok\n", __func__);
			req = NULL;
		} else if ((ep_csr & GOLDFISH_UDC_OCSR1_PKTRDY)
				&& fifo_count
				&& goldfish_udc_read_fifo(ep, req)) {
			req = NULL;
			ddprintk("%s read ok\n", __func__);
		}
	}

	/* pio or dma irq handler advances the queue. */
	if (likely (req != 0))
		list_add_tail(&req->queue, &ep->queue);

	local_irq_restore(flags);

	ddprintk("%s ok\n", __func__);
	return 0;
}

/*
 *	goldfish_udc_dequeue
 */
static int goldfish_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct goldfish_ep	*ep = to_goldfish_ep(_ep);
	struct goldfish_udc	*udc;
	int			retval = -EINVAL;
	unsigned long		flags;
	struct goldfish_request	*req = NULL;

	ddprintk("%s(%p,%p)\n", __func__, _ep, _req);

	if (!the_controller->driver)
		return -ESHUTDOWN;

	if (!_ep || !_req)
		return retval;

	udc = to_goldfish_udc(ep->gadget);

	local_irq_save (flags);

	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init (&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}

	if (retval == 0) {
		ddprintk(
			"dequeued req %p from %s, len %d buf %p\n",
			req, _ep->name, _req->length, _req->buf);

		goldfish_udc_done(ep, req, -ECONNRESET);
	}

	local_irq_restore (flags);
	return retval;
}

/*
 * goldfish_udc_set_halt
 */
static int goldfish_udc_set_halt(struct usb_ep *_ep, int value)
{
	struct goldfish_ep	*ep = to_goldfish_ep(_ep);
	u32			ep_csr = 0;
	unsigned long		flags;
	u32			idx;

	if (unlikely (!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		ddprintk("%s: inval 2\n", __func__);
		return -EINVAL;
	}

	local_irq_save (flags);

	idx = ep->bEndpointAddress & 0x7F;

	if (idx == 0) {
		goldfish_udc_set_ep0_ss(base_addr);
		goldfish_udc_set_ep0_de_out(base_addr);
	} else {
#ifdef TODO
		udc_write(idx, GOLDFISH_UDC_INDEX_REG);
		ep_csr = udc_read((ep->bEndpointAddress &USB_DIR_IN)
				? GOLDFISH_UDC_IN_CSR1_REG
				: GOLDFISH_UDC_OUT_CSR1_REG);

		if ((ep->bEndpointAddress & USB_DIR_IN) != 0) {
			if (value)
				udc_write(ep_csr | GOLDFISH_UDC_ICSR1_SENDSTL,
					GOLDFISH_UDC_IN_CSR1_REG);
			else {
				ep_csr &= ~GOLDFISH_UDC_ICSR1_SENDSTL;
				udc_write(ep_csr, GOLDFISH_UDC_IN_CSR1_REG);
				ep_csr |= GOLDFISH_UDC_ICSR1_CLRDT;
				udc_write(ep_csr, GOLDFISH_UDC_IN_CSR1_REG);
			}
		} else {
			if (value)
				udc_write(ep_csr | GOLDFISH_UDC_OCSR1_SENDSTL,
					GOLDFISH_UDC_OUT_CSR1_REG);
			else {
				ep_csr &= ~GOLDFISH_UDC_OCSR1_SENDSTL;
				udc_write(ep_csr, GOLDFISH_UDC_OUT_CSR1_REG);
				ep_csr |= GOLDFISH_UDC_OCSR1_CLRDT;
				udc_write(ep_csr, GOLDFISH_UDC_OUT_CSR1_REG);
			}
		}
#endif
	}

	ep->halted = value ? 1 : 0;
	local_irq_restore (flags);

	return 0;
}

static const struct usb_ep_ops goldfish_ep_ops = {
	.enable		= goldfish_udc_ep_enable,
	.disable	= goldfish_udc_ep_disable,

	.alloc_request	= goldfish_udc_alloc_request,
	.free_request	= goldfish_udc_free_request,

	.queue		= goldfish_udc_queue,
	.dequeue	= goldfish_udc_dequeue,

	.set_halt	= goldfish_udc_set_halt,
};

/*------------------------- usb_gadget_ops ----------------------------------*/

/*
 *	goldfish_udc_get_frame
 */
static int goldfish_udc_get_frame(struct usb_gadget *_gadget)
{
	int tmp;

	ddprintk("%s()\n", __func__);

#ifdef TODO
	tmp = udc_read(GOLDFISH_UDC_FRAME_NUM2_REG) << 8;
	tmp |= udc_read(GOLDFISH_UDC_FRAME_NUM1_REG);
#endif
	return tmp;
}

/*
 *	goldfish_udc_wakeup
 */
static int goldfish_udc_wakeup(struct usb_gadget *_gadget)
{
	ddprintk("%s()\n", __func__);
	return 0;
}

/*
 *	goldfish_udc_set_selfpowered
 */
static int goldfish_udc_set_selfpowered(struct usb_gadget *gadget, int value)
{
	struct goldfish_udc *udc = to_goldfish_udc(gadget);

	ddprintk("%s()\n", __func__);

	if (value)
		udc->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		udc->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);

	return 0;
}

static void goldfish_udc_disable(struct goldfish_udc *dev);
static void goldfish_udc_enable(struct goldfish_udc *dev);

static int goldfish_udc_set_pullup(struct goldfish_udc *udc, int is_on)
{
	ddprintk("%s()\n", __func__);

#ifdef TODO
	if (udc_info && udc_info->udc_command) {
		if (is_on)
			goldfish_udc_enable(udc);
		else {
			if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
				if (udc->driver && udc->driver->disconnect)
					udc->driver->disconnect(&udc->gadget);

			}
			goldfish_udc_disable(udc);
		}
	}
	else
		return -EOPNOTSUPP;
#else
	if (is_on) {
		goldfish_udc_enable(udc);
	} else {
		if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
			if (udc->driver && udc->driver->disconnect) {
				udc->driver->disconnect(&udc->gadget);
			}
		}
		goldfish_udc_disable(udc);
	}
#endif

	return 0;
}

static int goldfish_udc_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct goldfish_udc *udc = to_goldfish_udc(gadget);

	ddprintk("%s()\n", __func__);

	udc->vbus = (is_active != 0);
	goldfish_udc_set_pullup(udc, is_active);
	return 0;
}

static int goldfish_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct goldfish_udc *udc = to_goldfish_udc(gadget);

	ddprintk("%s()\n", __func__);

	goldfish_udc_set_pullup(udc, is_on ? 0 : 1);
	return 0;
}

static irqreturn_t goldfish_udc_vbus_irq(int irq, void *_dev)
{
	struct goldfish_udc	*dev = _dev;
	unsigned int		value;

	dprintk(DEBUG_NORMAL, "%s()\n", __func__);

	value = gpio_get_value(udc_info->vbus_pin) ? 1 : 0;
	if (udc_info->vbus_pin_inverted)
		value = !value;

	if (value != dev->vbus)
		goldfish_udc_vbus_session(&dev->gadget, value);

	return IRQ_HANDLED;
}

static int goldfish_vbus_draw(struct usb_gadget *_gadget, unsigned ma)
{
	dprintk(DEBUG_NORMAL, "%s()\n", __func__);

	if (udc_info && udc_info->vbus_draw) {
		udc_info->vbus_draw(ma);
		return 0;
	}

	return -ENOTSUPP;
}

static const struct usb_gadget_ops goldfish_ops = {
	.get_frame		= goldfish_udc_get_frame,
	.wakeup			= goldfish_udc_wakeup,
	.set_selfpowered	= goldfish_udc_set_selfpowered,
	.pullup			= goldfish_udc_pullup,
	.vbus_session		= goldfish_udc_vbus_session,
	.vbus_draw		= goldfish_vbus_draw,
};

/*------------------------- gadget driver handling---------------------------*/
/*
 * goldfish_udc_disable
 */
static void goldfish_udc_disable(struct goldfish_udc *dev)
{
	ddprintk("%s()\n", __func__);

	/* Disable all interrupts */
#ifdef TODO
	udc_write(0x00, GOLDFISH_UDC_USB_INT_EN_REG);
	udc_write(0x00, GOLDFISH_UDC_EP_INT_EN_REG);

	/* Clear the interrupt registers */
	udc_write(GOLDFISH_UDC_USBINT_RESET
				| GOLDFISH_UDC_USBINT_RESUME
				| GOLDFISH_UDC_USBINT_SUSPEND,
			GOLDFISH_UDC_USB_INT_REG);

	udc_write(0x1F, GOLDFISH_UDC_EP_INT_REG);

	/* Good bye, cruel world */
	if (udc_info && udc_info->udc_command)
		udc_info->udc_command(GOLDFISH_UDC_P_DISABLE);

	/* Set speed to unknown */
	dev->gadget.speed = USB_SPEED_UNKNOWN;
#endif
}

/*
 * goldfish_udc_reinit
 */
static void goldfish_udc_reinit(struct goldfish_udc *dev)
{
	u32 i;

	/* device/ep0 records init */
	INIT_LIST_HEAD (&dev->gadget.ep_list);
	INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);
	dev->ep0state = EP0_IDLE;

	for (i = 0; i < GOLDFISH_ENDPOINTS; i++) {
		struct goldfish_ep *ep = &dev->ep[i];

		if (i != 0)
			list_add_tail (&ep->ep.ep_list, &dev->gadget.ep_list);

		ep->dev = dev;
		ep->desc = NULL;
		ep->halted = 0;
		INIT_LIST_HEAD (&ep->queue);
	}
}

/*
 * goldfish_udc_enable
 */
static void goldfish_udc_enable(struct goldfish_udc *dev)
{
	int i;

	ddprintk("goldfish_udc_enable called\n");

	/* dev->gadget.speed = USB_SPEED_UNKNOWN; */
	dev->gadget.speed = USB_SPEED_FULL;

#ifdef TODO
	/* Set MAXP for all endpoints */
	for (i = 0; i < GOLDFISH_ENDPOINTS; i++) {
		udc_write(i, GOLDFISH_UDC_INDEX_REG);
		udc_write((dev->ep[i].ep.maxpacket & 0x7ff) >> 3,
				GOLDFISH_UDC_MAXP_REG);
	}

	/* Set default power state */
	udc_write(DEFAULT_POWER_STATE, GOLDFISH_UDC_PWR_REG);

	/* Enable reset and suspend interrupt interrupts */
	udc_write(GOLDFISH_UDC_USBINT_RESET | GOLDFISH_UDC_USBINT_SUSPEND,
			GOLDFISH_UDC_USB_INT_EN_REG);

	/* Enable ep0 interrupt */
	udc_write(GOLDFISH_UDC_INT_EP0, GOLDFISH_UDC_EP_INT_EN_REG);

	/* time to say "hello, world" */
	if (udc_info && udc_info->udc_command)
		udc_info->udc_command(GOLDFISH_UDC_P_ENABLE);
#endif
}

/*
 *	usb_gadget_register_driver
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct goldfish_udc *udc = the_controller;
	int		retval;

	ddprintk("usb_gadget_register_driver() '%s'\n",
		driver->driver.name);

	/* Sanity checks */
	if (!udc)
		return -ENODEV;

	if (udc->driver)
		return -EBUSY;

	if (!driver->bind || !driver->setup
			|| driver->speed < USB_SPEED_FULL) {
		printk(KERN_ERR "Invalid driver: bind %p setup %p speed %d\n",
			driver->bind, driver->setup, driver->speed);
		return -EINVAL;
	}
#if defined(MODULE)
	if (!driver->unbind) {
		printk(KERN_ERR "Invalid driver: no unbind method\n");
		return -EINVAL;
	}
#endif

	/* Hook the driver */
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	ddprintk("udc dev=0x%x\n", udc);
	ddprintk("udc driver=0x%x\n", udc->driver);

	/* Bind the driver */
	if ((retval = device_add(&udc->gadget.dev)) != 0) {
		printk(KERN_ERR "Error in device_add() : %d\n",retval);
		goto register_error;
	}

	ddprintk("binding gadget driver '%s'\n",
		driver->driver.name);

	if ((retval = driver->bind (&udc->gadget)) != 0) {
		device_del(&udc->gadget.dev);
		goto register_error;
	}

	/* Enable udc */
	goldfish_udc_enable(udc);

	return 0;

register_error:
	udc->driver = NULL;
	udc->gadget.dev.driver = NULL;
	return retval;
}

/*
 *	usb_gadget_unregister_driver
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct goldfish_udc *udc = the_controller;

	if (!udc)
		return -ENODEV;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	dprintk(DEBUG_NORMAL,"usb_gadget_register_driver() '%s'\n",
		driver->driver.name);

	if (driver->disconnect)
		driver->disconnect(&udc->gadget);

	device_del(&udc->gadget.dev);
	udc->driver = NULL;

	/* Disable udc */
	goldfish_udc_disable(udc);

	return 0;
}
#if 1
#undef EP0_FIFO_SIZE
#undef EP_FIFO_SIZE
#define EP0_FIFO_SIZE	64
#define EP_FIFO_SIZE	1024
#endif

/*---------------------------------------------------------------------------*/
static struct goldfish_udc memory = {
	.gadget = {
		.ops		= &goldfish_ops,
		.ep0		= &memory.ep[0].ep,
		.name		= gadget_name,
		.dev = {
			.init_name	= "gadget",
		},
	},

	/* control endpoint */
	.ep[0] = {
		.num		= 0,
		.ep = {
			.name		= ep0name,
			.ops		= &goldfish_ep_ops,
			.maxpacket	= EP0_FIFO_SIZE,
		},
		.dev		= &memory,
	},

	/* first group of endpoints */
	.ep[1] = {
		.num		= 1,
		.ep = {
			.name		= "ep1-bulk",
			.ops		= &goldfish_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 1,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	},
	.ep[2] = {
		.num		= 2,
		.ep = {
			.name		= "ep2-bulk",
			.ops		= &goldfish_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 2,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	},
	.ep[3] = {
		.num		= 3,
		.ep = {
			.name		= "ep3-bulk",
			.ops		= &goldfish_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 3,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	},
	.ep[4] = {
		.num		= 4,
		.ep = {
			.name		= "ep4-bulk",
			.ops		= &goldfish_ep_ops,
			.maxpacket	= EP_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= EP_FIFO_SIZE,
		.bEndpointAddress = 4,
		.bmAttributes	= USB_ENDPOINT_XFER_BULK,
	}

};

/*
 *	probe - binds to the platform device
 *
 * struct goldfish_udc_mach_info {
 *       void    (*udc_command)(enum goldfish_udc_cmd_e);
 *       void    (*vbus_draw)(unsigned int ma);
 *       unsigned int vbus_pin;
 *       unsigned char vbus_pin_inverted;
 * };
 */
static int goldfish_udc_probe(struct platform_device *pdev)
{
	struct goldfish_udc *udc = &memory;
	struct device *dev = &pdev->dev;
	struct resource *r;
	int retval;
	int irq;

	ddprintk("%s()\n", __func__);
	spin_lock_init (&udc->lock);

	udc_info = kzalloc(sizeof(*udc_info), GFP_KERNEL);
	if(udc_info == NULL) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, udc_info);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(r == NULL) {
		//TODO
		return -ENODEV;
	}
	udc_info->base = IO_ADDRESS(r->start - IO_START);
	udc_info->irq = platform_get_irq(pdev, 0);
	if(udc_info->irq < 0) {
		//TODO
		return -ENODEV;
	}
	rsrc_start = r->start;
	rsrc_len   = r->end - r->start + 1;
	if (!request_mem_region(rsrc_start, rsrc_len, gadget_name))
		return -EBUSY;

	base_addr = ioremap(rsrc_start, rsrc_len);
	if (!base_addr) {
		retval = -ENOMEM;
		goto err_mem;
	}

	device_initialize(&udc->gadget.dev);
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.dev.dma_mask = pdev->dev.dma_mask;

	the_controller = udc;
	//platform_set_drvdata(pdev, udc);

	goldfish_udc_disable(udc);
	goldfish_udc_reinit(udc);

	/* irq setup after old hardware state is cleaned up */
	ddprintk("%s() request_irq:irq=%d %d\n", __func__, 
				udc_info->irq, goldfish_udc_irq);
#if 1
	retval = request_irq(udc_info->irq, goldfish_udc_irq, 
				0, pdev->name, udc);
#else
	retval = request_irq(udc_info->irq, goldfish_udc_irq, 
				0, pdev->name, udc_info);
#endif

	if (retval != 0) {
		retval = -EBUSY;
		goto err_map;
	}

#if 0
	if (udc_info && udc_info->vbus_pin > 0) {
		retval = gpio_request(udc_info->vbus_pin, "udc vbus");
		if (retval < 0) {
			dev_err(dev, "cannot claim vbus pin\n");
			goto err_int;
		}

		irq = gpio_to_irq(udc_info->vbus_pin);
		if (irq < 0) {
			dev_err(dev, "no irq for gpio vbus pin\n");
			goto err_gpio_claim;
		}

		retval = request_irq(irq, goldfish_udc_vbus_irq,
				     IRQF_DISABLED | IRQF_TRIGGER_RISING
				     | IRQF_TRIGGER_FALLING | IRQF_SHARED,
				     gadget_name, udc);

		if (retval != 0) {
			dev_err(dev, "can't get vbus irq %d, err %d\n",
				irq, retval);
			retval = -EBUSY;
			goto err_gpio_claim;
		}

		dev_dbg(dev, "got irq %i\n", irq);
	} else {
		udc->vbus = 1;
	}
#endif

	if (goldfish_udc_debugfs_root) {
		udc->regs_info = debugfs_create_file("registers", S_IRUGO,
				goldfish_udc_debugfs_root,
				udc, &goldfish_udc_debugfs_fops);
		if (!udc->regs_info)
			dev_warn(dev, "debugfs file creation failed\n");
	}
	ddprintk("goldfish_usbgadget:probe ok\n");

	return 0;

err_gpio_claim:
	if (udc_info && udc_info->vbus_pin > 0)
		gpio_free(udc_info->vbus_pin);
err_int:
#ifdef TODO
	free_irq(IRQ_USBD, udc);
#endif
err_map:
	iounmap(base_addr);
err_mem:
	release_mem_region(rsrc_start, rsrc_len);

	return retval;
}

/*
 *	goldfish_udc_remove
 */
static int goldfish_udc_remove(struct platform_device *pdev)
{
	// TODO
	return 0;
}

#ifdef CONFIG_PM
static int goldfish_udc_suspend(struct platform_device *pdev, pm_message_t message)
{
#ifdef TODO
	if (udc_info && udc_info->udc_command)
		udc_info->udc_command(GOLDFISH_UDC_P_DISABLE);
#endif

	return 0;
}

static int goldfish_udc_resume(struct platform_device *pdev)
{
#ifdef TODO
	if (udc_info && udc_info->udc_command)
		udc_info->udc_command(GOLDFISH_UDC_P_ENABLE);
#endif

	return 0;
}
#else
#define goldfish_udc_suspend	NULL
#define goldfish_udc_resume	NULL
#endif

static struct platform_driver udc_driver_goldfish = {
	.driver		= {
		.name	= "goldfish_usbgadget",
		.owner	= THIS_MODULE,
	},
	.probe		= goldfish_udc_probe,
	.remove		= goldfish_udc_remove,
	.suspend	= goldfish_udc_suspend,
	.resume		= goldfish_udc_resume,
};


static int __init udc_init(void)
{
	int retval;

	dprintk(DEBUG_NORMAL, "%s: version %s\n", gadget_name, DRIVER_VERSION);

	goldfish_udc_debugfs_root = debugfs_create_dir(gadget_name, NULL);
	if (IS_ERR(goldfish_udc_debugfs_root)) {
		printk(KERN_ERR "%s: debugfs dir creation failed %ld\n",
			gadget_name, PTR_ERR(goldfish_udc_debugfs_root));
		goldfish_udc_debugfs_root = NULL;
	}

	retval = platform_driver_register(&udc_driver_goldfish);
	ddprintk("udc_init: platform_dirver_register:retval=%d\n", retval);
	if (retval)
		goto err;

	return 0;

err:
	debugfs_remove(goldfish_udc_debugfs_root);
	return retval;
}

static void __exit udc_exit(void)
{
	platform_driver_unregister(&udc_driver_goldfish);
	debugfs_remove(goldfish_udc_debugfs_root);
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);
EXPORT_SYMBOL(usb_gadget_register_driver);

module_init(udc_init);
module_exit(udc_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:goldfish-usbgadget");
