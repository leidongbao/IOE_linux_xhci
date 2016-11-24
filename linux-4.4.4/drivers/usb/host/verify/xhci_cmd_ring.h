#ifndef __VERIFY_CMD_RING_H
#define __VERIFY_CMD_RING_H

#include "xhci_ctler_verify.h"
unsigned int check_route_string_valid(int xhci_cmd);
unsigned int check_endpoint_valid(int xhci_cmd);

int cmd_not_support(struct xhci_ctler *ctler, int xhci_cmd);

void cmd_no_op(unsigned *field1, unsigned *field2, unsigned *field3, unsigned *field4);

void cmd_reset_endpoint(struct xhci_hcd *xhci,
	struct usb_host_endpoint *ep, struct usb_device *udev,
	unsigned *field1, unsigned *field2, unsigned *field3, unsigned *field4);

void cmd_force_header(struct usb_device *udev,unsigned *field1, 
	unsigned *field2, unsigned *field3, unsigned *field4);

#endif
