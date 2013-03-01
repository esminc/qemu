#include "goldfish_usbgadget.h"
#include "goldfish_usbgadget_reg.h"
#include "goldfish_reg.h"

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
#define UDC_FIFO_SIZE			10240
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

static char *baseAddr = NULL;
char *v2a(void *v) {
	return ((unsigned int)baseAddr + (char*)v);
}

#define OP_READ		0
#define OP_WRITE	1
#define FIFOSET(f, reg, op) do {					\
	if (UDC_REGTYPE((reg)->type) == UDC_REGTYPE_SFIFO) {		\
		(f) = (struct udc_fifoValue*)v2a((reg)->value);		\
	} else {							\
		struct udc_doubleFifoValue *_p;				\
		_p = (struct udc_doubleFifoValue*)v2a((reg)->value);	\
		if ((op) == OP_READ) {					\
			(f) = (struct udc_fifoValue*)(&_p->read);	\
		} else {						\
			(f) = (struct udc_fifoValue*)(&_p->write);	\
		}							\
	}								\
} while (0)


//static struct udc_deviceRegister udc_deviceRegister[MAX_REGISTERS];
//static struct udc_fifoValue  udc_fifo[UDC_FIFO_NUM];
static struct udc_deviceRegister *udc_deviceRegisterp;
static struct udc_fifoValue  *udc_fifop;

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/file.h>
static int map_fd = -1;
static void initMmap(void)
{
	int len1;
	int len2;
	const char *pathname = "/tmp/register.img";
	map_fd = open(pathname, O_RDWR, 0644);
	assert(map_fd > 0);
	len1 = sizeof(struct udc_deviceRegister)*MAX_REGISTERS;
	printf("len1=%d\n", len1);
	//ret = write(map_fd, (char*)udc_deviceRegister, len1);
	len2 = sizeof(struct udc_fifoValue)*UDC_FIFO_NUM;
	printf("len2=%d\n", len2);
	//ret = write(map_fd, (char*)udc_fifo, len2);
	udc_deviceRegisterp = (struct udc_deviceRegister *)mmap(NULL, len1+len2, PROT_READ|PROT_WRITE, MAP_SHARED, map_fd, 0);
	baseAddr = (char*)udc_deviceRegisterp;
	assert(udc_deviceRegisterp != MAP_FAILED);
	udc_fifop = ((char*)udc_deviceRegisterp) + len1;
	printf("regp=%p fifop=%p\n", udc_deviceRegisterp, udc_fifop);
	printf("mmap size=%d\n", len1+len2);
{
	int a, b;
	a =*(int*)(baseAddr + 0);
	b =*(int*)(baseAddr + 4);
	printf("a=0x%x b=0x%x\n", a, b);
}
}

static uint32_t getFifoCountAddr(int ep_nr)
{
	int addr = 0;
	switch (ep_nr) {
	case 1:
		addr = GOLDFISH_UDC_EP1_FIFO_CNT_REG;
		break;
	case 2:
		addr = GOLDFISH_UDC_EP2_FIFO_CNT_REG;
		break;
	case 3:
		addr = GOLDFISH_UDC_EP3_FIFO_CNT_REG;
		break;
	case 4:
		addr = GOLDFISH_UDC_EP4_FIFO_CNT_REG;
		break;
	case 5:
		addr = GOLDFISH_UDC_EP5_FIFO_CNT_REG;
		break;
	case 6:
		addr = GOLDFISH_UDC_EP6_FIFO_CNT_REG;
		break;
	case 7:
		addr = GOLDFISH_UDC_EP7_FIFO_CNT_REG;
		break;
	default:
		addr = GOLDFISH_UDC_EP0_FIFO_CNT_REG;
		break;
	}
	return addr;
}
#define dbg printf

uint32_t getFifoCount(int ep_nr)
{
	int regAddr;
	int count;
	regAddr = getFifoCountAddr(ep_nr);
	count = REG(regAddr);
	dbg("getFifoCount=%d\n", count);
	return count;
}

uint32_t getFifoAddr(int ep_nr)
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

void sendData(target_phys_addr_t regAddr, char *p, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		//dbg("goldfish_usbgadget_test:i=%d\n", i);
		REGSET(regAddr, p[i]);
	}
}
int recvFifoData(target_phys_addr_t regAddr, char *p)
{
	int i;
	int len;
	struct udc_fifoValue *fifo;
	FIFOSET(fifo, &udc_deviceRegisterp[regAddr], OP_WRITE);
	len = fifo->count;
	dbg("BEFORE:regAddr = %p len=%d\n", regAddr, len);
	for (i = 0; i < len; i++) {
		p[i] = REG(regAddr);
	}
	dbg("AFTER:len=%d\n", fifo->count);
	return len;
}


static char fifoRead(struct udc_fifoValue *fifo)
{
	char ret;
	if (fifo->count <= 0) {
		return 0;
	}
	ret = fifo->values[fifo->readOff];
	fifo->count--;
	printf("fifoRead:count=%d val[%d]=%d\n", fifo->count, fifo->readOff, ret);
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
	//dbg("fifoWrite:count=0x%x\n", fifo->count);
	if (fifo->count >= UDC_FIFO_SIZE) {
		return;
	}
	fifo->values[fifo->writeOff] = value;
	fifo->count++;
	dbg("fifoWrite:countRegAddr=0x%x\n", fifo->countRegAddr);
	if (fifo->countRegAddr) {
		REGSET(fifo->countRegAddr, fifo->count);
	}

	/* ringbuffer */
	if (fifo->writeOff == (UDC_FIFO_SIZE - 1)) {
		fifo->writeOff = 0;
	} else {
		fifo->writeOff++;
	}
}


/* register operations */

#define CHECK_OP_READ	0
#define CHECK_OP_WRITE	1
uint32_t guestOSRead(target_phys_addr_t regAddr)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	//dbg("guestOSRead:2\n");
	if (UDC_REGTYPE(udc_deviceRegisterp[off].type) == UDC_REGTYPE_NORMAL) {
		return (uint32_t)udc_deviceRegisterp[off].value;
	}
	//dbg("guestOSRead:3\n");
	/* for fifo read */
	if (udc_deviceRegisterp[off].value == NULL) { /* sanity check */
		return 0;
	}
	//dbg("guestOSRead:4\n");
	FIFOSET(fifo, &udc_deviceRegisterp[off], OP_READ);
	return fifoRead(fifo);
}

void guestOSWrite(target_phys_addr_t regAddr, uint32_t value)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	if (UDC_REGTYPE(udc_deviceRegisterp[off].type) == UDC_REGTYPE_NORMAL) {
		udc_deviceRegisterp[off].value = (void*)value;
		return;
	}
	/* for fifo write */
	if (udc_deviceRegisterp[off].value == NULL) { /* sanity check */
		return;
	}
	FIFOSET(fifo, &udc_deviceRegisterp[off], OP_WRITE);
	fifoWrite(fifo, value);
	return;
}

uint32_t deviceRead(target_phys_addr_t regAddr)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	if (UDC_REGTYPE(udc_deviceRegisterp[off].type) == UDC_REGTYPE_NORMAL) {
		return (uint32_t)udc_deviceRegisterp[off].value;
	}
	/* for fifo read */
	if (udc_deviceRegisterp[off].value == NULL) { /* sanity check */
		return 0;
	}
	FIFOSET(fifo, &udc_deviceRegisterp[off], OP_WRITE);
	return fifoRead(fifo);
}

void deviceWrite(target_phys_addr_t regAddr, uint32_t value)
{
	int off = UDC_REG2OFF(regAddr);
	struct udc_fifoValue *fifo;

	//dbg("deviceWrite:1\n");
	if (UDC_REGTYPE(udc_deviceRegisterp[off].type) == UDC_REGTYPE_NORMAL) {
		udc_deviceRegisterp[off].value = (void*)value;
		return;
	}
	//dbg("deviceWrite:2\n");
	/* for fifo write */
	if (udc_deviceRegisterp[off].value == NULL) { /* sanity check */
		return;
	}
	//dbg("deviceWrite:3\n");
	FIFOSET(fifo, &udc_deviceRegisterp[off], OP_READ);
	fifoWrite(fifo, value);
	return;
}

void init_udc_deviceRegister(void)
{
	initMmap();
}
