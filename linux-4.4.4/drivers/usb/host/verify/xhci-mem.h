#ifndef __XHCI_MEM_H
#define __XHCI_MEM_H

// Used in xhci_ctler_verify.c in set_event_seg_num
struct xhci_segment *xhci_segment_alloc(struct xhci_hcd *xhci, 
			unsigned int cycle_state, gfp_t flags);

void xhci_segment_free(struct xhci_hcd *xhci, struct xhci_segment *seg);

void xhci_link_segments(struct xhci_hcd *xhci, struct xhci_segment *prev,
			struct xhci_segment *next, enum xhci_ring_type type);

u32 xhci_get_max_esit_payload(struct xhci_hcd *xhci, struct usb_device *udev,
			struct usb_host_endpoint *ep);

struct xhci_ring *xhci_ring_alloc(struct xhci_hcd *xhci, unsigned int num_segs,
			unsigned int cycle_state, enum xhci_ring_type type, gfp_t flags);

int xhci_check_trb_in_td_math(struct xhci_hcd *xhci, gfp_t mem_flags);

#endif
