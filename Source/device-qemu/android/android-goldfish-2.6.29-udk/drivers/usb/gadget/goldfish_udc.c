/*
 * linux/drivers/usb/gadget/goldfish_udc.c
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
#include "goldfish_udc.h"
#include "goldfish_reg.h"
#define DEBUG_PRINT
#ifdef DEBUG_PRINT
#define ddprintk(fmt, ...) \
do { printk(fmt , ## __VA_ARGS__); } while (0)
#else
#define ddprintk(fmt, ...) do {} while(0)
#endif

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

#define DRIVER_DESC	"GOLDFISH USB Device Controller Gadget"
#define DRIVER_VERSION	"22 Dec 2012"
#define DRIVER_AUTHOR	"Takashi Mori <kanetugu2001@gmail.com> "

static const char		gadget_name[] = "goldfish_usbgadget";

static struct goldfish_udc	*the_controller;
static void __iomem		*base_addr;
static u64			rsrc_start;
static u64			rsrc_len;
static struct dentry		*goldfish_udc_debugfs_root;

static inline u32 udc_read(u32 reg)
{
	return readb(base_addr + reg);
}

static inline u32 udc_readl(u32 reg)
{
	return readw(base_addr + reg);
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

/* io macros */


static inline void goldfish_udc_set_ep0_ipr(void __iomem *base)
{
	udc_writeb(base, GOLDFISH_UDC_EP0_CSR_IPKRDY, GOLDFISH_UDC_EP0_CSR_REG);
}


static inline void goldfish_udc_set_ep0_de_out(void __iomem *base)
{
	udc_writeb(base,(GOLDFISH_UDC_EP0_CSR_SOPKTRDY
				| GOLDFISH_UDC_EP0_CSR_DE),
			GOLDFISH_UDC_EP0_CSR_REG);
}


static inline void goldfish_udc_set_ep0_de_in(void __iomem *base)
{
	udc_writeb(base, 
		(GOLDFISH_UDC_EP0_CSR_IPKRDY | GOLDFISH_UDC_EP0_CSR_DE),
		GOLDFISH_UDC_EP0_CSR_REG);
}

/*------------------------- I/O ----------------------------------*/

/*
 *	goldfish_udc_done
 */
static void goldfish_udc_done(struct goldfish_ep *ep,
		struct goldfish_request *req, int status)
{
	unsigned halted = ep->halted;

	ddprintk("%s enter\n", __func__);
	list_del_init(&req->queue);

	if (likely (req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	ep->halted = 1;
	req->req.complete(&ep->ep, &req->req);
	ep->halted = halted;
	ddprintk("%s exit\n", __func__);
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


static inline int goldfish_udc_fifo_count_out(int countReg)
{
	return udc_readl(countReg);
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
	ddprintk(
			"Written ep%d %d.%d of %d b [last %d,z %d]\n",
			idx, count, req->req.actual, req->req.length,
			is_last, req->req.zero);

	if (is_last) {
ddprintk("is_last!=0\n");
		/* The order is important. It prevents sending 2 packets
		 * at the same time */

		if (idx == 0) {
			goldfish_udc_set_ep0_de_in(base_addr);
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
			goldfish_udc_set_ep0_ipr(base_addr);
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

	udc_write(idx, GOLDFISH_UDC_INDEX_REG);

	fifo_count = goldfish_udc_fifo_count_out(count_reg);
	ddprintk("%s fifo count : %d maxpacket=%d\n", __func__, fifo_count, ep->ep.maxpacket);

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
		ddprintk("%s req.length=%d req.actual=%d\n", __func__, req->req.length, req->req.actual);
		is_last = (req->req.length <= req->req.actual) ? 1 : 0;
	}

	udc_write(idx, GOLDFISH_UDC_INDEX_REG);
	fifo_count = goldfish_udc_fifo_count_out(count_reg);

	/* Only ep0 debug messages are interesting */
	ddprintk("%s fifo count : %d [last %d]\n",
			__func__, fifo_count,is_last);

	if (is_last) {
		if (idx == 0) {
		} else {
			ddprintk("%s PKTRDY1\n",__func__);
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			ep_csr = udc_read(GOLDFISH_UDC_OUT_CSR1_REG);
			udc_write(idx, GOLDFISH_UDC_INDEX_REG);
			udc_write(ep_csr & ~GOLDFISH_UDC_OCSR1_PKTRDY,
					GOLDFISH_UDC_OUT_CSR1_REG);
		}

		goldfish_udc_done(ep, req, 0);
	} else {
		if (idx == 0) {
		} else {
			ddprintk("%s PKTRDY2\n",__func__);
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

	udc_write(0, GOLDFISH_UDC_INDEX_REG);

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

	switch (crq->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_INTERFACE:
		break;

	case USB_RECIP_DEVICE:
		status = dev->devstatus;
		break;

	case USB_RECIP_ENDPOINT:
		if (ep_num > 4 || crq->wLength > 2)
			return 1;
		status = status ? 1 : 0;
		break;

	default:
		return 1;
	}

	/* Seems to be needed to get it working. ouch :( */
	udelay(5);
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

	len = goldfish_udc_read_fifo_crq(crq);
	if (len != sizeof(*crq)) {
		ddprintk("setup begin: fifo READ ERROR"
			" wanted %d bytes got %d. Stalling out...\n",
			sizeof(*crq), len);
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

		if (dev->req_std) {
			if (!goldfish_udc_get_status(dev, crq)) {
				return;
			}
		}
		break;

	case USB_REQ_CLEAR_FEATURE:

		if (crq->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq->wValue != USB_ENDPOINT_HALT || crq->wLength != 0)
			break;

		goldfish_udc_set_halt(&dev->ep[crq->wIndex & 0x7f].ep, 0);
		goldfish_udc_set_ep0_de_out(base_addr);
		return;

	case USB_REQ_SET_FEATURE:

		if (crq->bRequestType != USB_RECIP_ENDPOINT)
			break;

		if (crq->wValue != USB_ENDPOINT_HALT || crq->wLength != 0)
			break;

		goldfish_udc_set_halt(&dev->ep[crq->wIndex & 0x7f].ep, 1);
		goldfish_udc_set_ep0_de_out(base_addr);
		return;

	default:
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
	//ddprintk("ep0state %s ret=%d\n", ep0states[dev->ep0state], ret);
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

	//ddprintk("ep0csr %x ep0state %s\n",
	//	ep0csr, ep0states[dev->ep0state]);

	/* clear stall status */
	switch (dev->ep0state) {
	case EP0_IDLE:
		ddprintk("EP0_IDLE\n");
		goldfish_udc_handle_ep0_idle(dev, ep, &crq, ep0csr);
		break;

	}
}

/*
 *	handle_ep - Manage I/O endpoints
 */

static void goldfish_udc_handle_ep(struct goldfish_ep *ep)
{
	struct goldfish_request	*req;
	int			is_in = ep->bEndpointAddress & USB_DIR_IN;
	u32			idx;

	if (likely (!list_empty(&ep->queue)))
		req = list_entry(ep->queue.next,
				struct goldfish_request, queue);
	else
		req = NULL;

	idx = ep->bEndpointAddress & 0x7F;

	ddprintk("goldfish_udc_handle_ep:idx=%d is_in=%d req=%p\n", idx, is_in, req);
	if (is_in) {
		if (req) {
			goldfish_udc_write_fifo(ep,req);
		}
	} else {
		if (req) {
			goldfish_udc_read_fifo(ep,req);
		}
	}
}

static int goldfish_udc_pullup(struct usb_gadget *gadget, int is_on);
/*
 *	goldfish_udc_irq - interrupt handler
 */
static irqreturn_t goldfish_udc_irq(int dummy, void *_dev)
{
	struct goldfish_udc *dev = _dev;
	int usb_status;
	int usbd_status;
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

	/* Save index */
	idx = udc_read(GOLDFISH_UDC_INDEX_REG);

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

	usb_status = udc_read(GOLDFISH_UDC_USB_INT_REG);
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

	ep = to_goldfish_ep(_ep);

	if (!_ep || !desc || ep->desc
			|| _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize);

	local_irq_save (flags);
	_ep->maxpacket = max;
	ep->desc = desc;
	ep->halted = 0;
	ep->bEndpointAddress = desc->bEndpointAddress;

	/* set max packet */

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
	}

	/* kickstart this i/o queue? */
	ddprintk("%s list_empty(ep->queue)=%d endp=0x%x ep_csr=0x%x fifo_count=%d\n", 
		__func__, list_empty(&ep->queue),
		ep->bEndpointAddress, ep_csr, fifo_count);
	if (list_empty(&ep->queue) && !ep->halted) {
		if (ep->bEndpointAddress == 0 /* ep0 */) {
			switch (dev->ep0state) {
			case EP0_IN_DATA_PHASE:
				goldfish_udc_write_fifo(ep, req);
				dev->ep0state = EP0_IDLE;
				req = NULL;
				break;

			case EP0_OUT_DATA_PHASE:
				dev->ep0state = EP0_IDLE;
				req = NULL;
				break;

			default:
				local_irq_restore(flags);
				return -EL2HLT;
			}
		} else if ((ep->bEndpointAddress & USB_DIR_IN) != 0
				&& ((ep_csr&GOLDFISH_UDC_OCSR1_PKTRDY))
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
	unsigned long		flags;
	u32			idx;

	if (unlikely (!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		ddprintk("%s: inval 2\n", __func__);
		return -EINVAL;
	}

	local_irq_save (flags);

	idx = ep->bEndpointAddress & 0x7F;

	if (idx == 0) {
		goldfish_udc_set_ep0_de_out(base_addr);
	} else {
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
	int tmp = 0;

	ddprintk("%s()\n", __func__);

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
	ddprintk("goldfish_udc_enable called\n");

	/* dev->gadget.speed = USB_SPEED_UNKNOWN; */
	dev->gadget.speed = USB_SPEED_FULL;

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
#undef EP0_FIFO_SIZE
#undef EP_FIFO_SIZE
#define EP0_FIFO_SIZE	64
#define EP_FIFO_SIZE	512

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
	struct resource *r;
	int retval;

	ddprintk("%s()\n", __func__);
	spin_lock_init (&udc->lock);

	udc_info = kzalloc(sizeof(*udc_info), GFP_KERNEL);
	if(udc_info == NULL) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, udc_info);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(r == NULL) {
		return -ENODEV;
	}
	udc_info->base = IO_ADDRESS(r->start - IO_START);
	udc_info->irq = platform_get_irq(pdev, 0);
	if(udc_info->irq < 0) {
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
	retval = request_irq(udc_info->irq, goldfish_udc_irq, 
				0, pdev->name, udc);

	if (retval != 0) {
		retval = -EBUSY;
		goto err_map;
	}

	ddprintk("goldfish_usbgadget:probe ok\n");

	return 0;

	if (udc_info && udc_info->vbus_pin > 0)
		gpio_free(udc_info->vbus_pin);
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
	return 0;
}

#ifdef CONFIG_PM
static int goldfish_udc_suspend(struct platform_device *pdev, pm_message_t message)
{
	return 0;
}

static int goldfish_udc_resume(struct platform_device *pdev)
{
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
