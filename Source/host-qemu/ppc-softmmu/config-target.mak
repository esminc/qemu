# Automatically generated by configure - do not modify
CONFIG_TCG_PASS_AREG0=y
TARGET_SHORT_ALIGNMENT=2
TARGET_INT_ALIGNMENT=4
TARGET_LONG_ALIGNMENT=4
TARGET_LLONG_ALIGNMENT=8
TARGET_ARCH=ppc
TARGET_PPC=y
TARGET_ARCH2=ppc
TARGET_TYPE=TARGET_TYPE_PPC
TARGET_BASE_ARCH=ppc
TARGET_ABI_DIR=ppc
CONFIG_NO_XEN=y
TARGET_WORDS_BIGENDIAN=y
TARGET_PHYS_ADDR_BITS=64
CONFIG_SOFTMMU=y
LIBS+=-lutil -lncurses  -luuid -L/usr/lib/i386-linux-gnu -lpng12   -lSDL   -lX11  
HWDIR=../libhw64
TARGET_XML_FILES= /mnt/external2/qemu/qemu-1.2.0/gdb-xml/power-core.xml /mnt/external2/qemu/qemu-1.2.0/gdb-xml/power-fpu.xml /mnt/external2/qemu/qemu-1.2.0/gdb-xml/power-altivec.xml /mnt/external2/qemu/qemu-1.2.0/gdb-xml/power-spe.xml
CONFIG_PCSPK=y
CONFIG_I386_DIS=y
CONFIG_PPC_DIS=y
LDFLAGS+=
QEMU_CFLAGS+=-DHAS_AUDIO -DHAS_AUDIO_CHOICE 
QEMU_INCLUDES+=-I$(SRC_PATH)/linux-headers -I$(SRC_PATH)/tcg -I$(SRC_PATH)/tcg/$(ARCH) 
