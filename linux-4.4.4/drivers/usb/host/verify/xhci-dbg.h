#ifndef __XHCI_DBG_H
#define __XHCI_DBG_H

void xhci_print_command_reg(struct xhci_hcd *xhci);
void xhci_print_ports(struct xhci_hcd *xhci);

void xhci_print_registers(struct xhci_hcd *xhci);

#endif
