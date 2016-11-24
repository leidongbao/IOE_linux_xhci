#ifndef __XHCI_RING_H
#define __XHCI_RING_H

int xhci_queue_bulk_tx_event_data_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index);
int xhci_queue_bulk_tx_zero_len_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index);
int xhci_queue_bulk_tx_with_little_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index);

int queue_bulk_sg_tx_zero_len_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index);
int queue_bulk_sg_tx_with_little_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index);

int xhci_verify_queue_bulk_sg_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
	int slot_id, unsigned int ep_index);


/*----------------------------------Used in Other files---------------------------------*/
int xhci_queue_ctrl_sg_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
	int slot_id, unsigned int ep_index);
int xhci_verify_queue_ctrl_sg_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
	int slot_id, unsigned int ep_index);
int xhci_verify_queue_isoc_tx_prepare(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index);
int prepare_ring(struct xhci_hcd *xhci, struct xhci_ring *ep_ring, u32 ep_state,
	unsigned int num_trbs, gfp_t mem_flags);

//used in xhci_queue_stor_cmd_trb
void queue_trb(struct xhci_hcd *xhci, struct xhci_ring *ring, bool more_trbs_coming,
	u32 field1, u32 field2, u32 field3, u32 field4);

//used in xhci_verify_stro_transport_with_setdeq
int queue_command(struct xhci_hcd *xhci, struct xhci_command *cmd, u32 field1,
	u32 field2, u32 field3, u32 field4, bool command_must_succeed);
#endif
