#
# Makefile for USB Host Controller Drivers
#

# tell define_trace.h where to find the xhci trace header
CFLAGS_xhci-trace.o := -I$(src)
#ccflags-$(CONFIG_USB_DEBUG) := -DDEBUG

xhci-verify-y := xhci.o xhci-mem.o
xhci-verify-y += xhci-ring.o xhci-hub.o xhci-dbg.o
xhci-verify-y += xhci-trace.o
xhci-verify-y += xhci_cmd_ring.o xhci_ctler_common.o xhci_ctler_verify.o 

#obj-$(CONFIG_USB_XHCI_VERIFY)	+= xhci-verify.o
obj-m	+= xhci-verify.o

KVERSION := $(shell uname -r)
all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(PWD) clean


