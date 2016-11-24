/*
 * xHCI host controller driver
 *
 * Copyright (C) 2008 Intel Corp.
 *
 * Author: Sarah Sharp
 * Some code borrowed from the Linux EHCI driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Ring initialization rules:
 * 1. Each segment is initialized to zero, except for link TRBs.
 * 2. Ring cycle state = 0.  This represents Producer Cycle State (PCS) or
 *    Consumer Cycle State (CCS), depending on ring function.
 * 3. Enqueue pointer = dequeue pointer = address of first TRB in the segment.
 *
 * Ring behavior rules:
 * 1. A ring is empty if enqueue == dequeue.  This means there will always be at
 *    least one free TRB in the ring.  This is useful if you want to turn that
 *    into a link TRB and expand the ring.
 * 2. When incrementing an enqueue or dequeue pointer, if the next TRB is a
 *    link TRB, then load the pointer with the address in the link TRB.  If the
 *    link TRB had its toggle bit set, you may need to update the ring cycle
 *    state (see cycle bit rules).  You may have to do this multiple times
 *    until you reach a non-link TRB.
 * 3. A ring is full if enqueue++ (for the definition of increment above)
 *    equals the dequeue pointer.
 *
 * Cycle bit rules:
 * 1. When a consumer increments a dequeue pointer and encounters a toggle bit
 *    in a link TRB, it must toggle the ring cycle state.
 * 2. When a producer increments an enqueue pointer and encounters a toggle bit
 *    in a link TRB, it must toggle the ring cycle state.
 *
 * Producer rules:
 * 1. Check if ring is full before you enqueue.
 * 2. Write the ring cycle state to the cycle bit in the TRB you're enqueuing.
 *    Update enqueue pointer between each write (which may update the ring
 *    cycle state).
 * 3. Notify consumer.  If SW is producer, it rings the doorbell for command
 *    and endpoint rings.  If HC is the producer for the event ring,
 *    and it generates an interrupt according to interrupt modulation rules.
 *
 * Consumer rules:
 * 1. Check if TRB belongs to you.  If the cycle bit == your ring cycle state,
 *    the TRB is owned by the consumer.
 * 2. Update dequeue pointer (which may update the ring cycle state) and
 *    continue processing TRBs until you reach a TRB which is not owned by you.
 * 3. Notify the producer.  SW is the consumer for the event ring, and it
 *   updates event ring dequeue pointer.  HC is the consumer for the command and
 *   endpoint rings; it generates events on the event ring for these.
 */

#include <linux/scatterlist.h>
#include <linux/slab.h>
#include "xhci.h"
#include "xhci-trace.h"
#include "xhci_ctler_verify.h"

/*
 * Returns zero if the TRB isn't in this segment, otherwise it returns the DMA
 * address of the TRB.
 */
dma_addr_t xhci_trb_virt_to_dma(struct xhci_segment *seg,
		union xhci_trb *trb)
{
	unsigned long segment_offset;

	if (!seg || !trb || trb < seg->trbs)
		return 0;
	/* offset in TRBs */
	segment_offset = trb - seg->trbs;
	if (segment_offset >= TRBS_PER_SEGMENT)
		return 0;
	return seg->dma + (segment_offset * sizeof(*trb));
}

/* Does this link TRB point to the first segment in a ring,
 * or was the previous TRB the last TRB on the last segment in the ERST?
 */
static bool last_trb_on_last_seg(struct xhci_hcd *xhci, struct xhci_ring *ring,
		struct xhci_segment *seg, union xhci_trb *trb)
{
	if (ring == xhci->event_ring)
		return (trb == &seg->trbs[TRBS_PER_SEGMENT]) &&
			(seg->next == xhci->event_ring->first_seg);
	else
		return le32_to_cpu(trb->link.control) & LINK_TOGGLE;
}

/* Is this TRB a link TRB or was the last TRB the last TRB in this event ring
 * segment?  I.e. would the updated event TRB pointer step off the end of the
 * event seg?
 */
static int last_trb(struct xhci_hcd *xhci, struct xhci_ring *ring,
		struct xhci_segment *seg, union xhci_trb *trb)
{
	if (ring == xhci->event_ring)
		return trb == &seg->trbs[TRBS_PER_SEGMENT];
	else
		return TRB_TYPE_LINK_LE32(trb->link.control);
}

static int enqueue_is_link_trb(struct xhci_ring *ring)
{
	struct xhci_link_trb *link = &ring->enqueue->link;
	return TRB_TYPE_LINK_LE32(link->control);
}

/* Updates trb to point to the next TRB in the ring, and updates seg if the next
 * TRB is in a new segment.  This does not skip over link TRBs, and it does not
 * effect the ring dequeue or enqueue pointers.
 */
static void next_trb(struct xhci_hcd *xhci,
		struct xhci_ring *ring,
		struct xhci_segment **seg,
		union xhci_trb **trb)
{
	if (last_trb(xhci, ring, *seg, *trb)) {
		*seg = (*seg)->next;
		*trb = ((*seg)->trbs);
	} else {
		(*trb)++;
	}
}


/*
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * If we've just enqueued a TRB that is in the middle of a TD (meaning the
 * chain bit is set), then set the chain bit in all the following link TRBs.
 * If we've enqueued the last TRB in a TD, make sure the following link TRBs
 * have their chain bit cleared (so that each Link TRB is a separate TD).
 *
 * Section 6.4.4.1 of the 0.95 spec says link TRBs cannot have the chain bit
 * set, but other sections talk about dealing with the chain bit set.  This was
 * fixed in the 0.96 specification errata, but we have to assume that all 0.95
 * xHCI hardware can't handle the chain bit being cleared on a link TRB.
 *
 * @more_trbs_coming:	Will you enqueue more TRBs before calling
 *			prepare_transfer()?
 */
static void inc_enq(struct xhci_hcd *xhci, struct xhci_ring *ring,
			bool more_trbs_coming)
{
	u32 chain;
	union xhci_trb *next;

	chain = le32_to_cpu(ring->enqueue->generic.field[3]) & TRB_CHAIN;
	/* If this is not event ring, there is one less usable TRB */
	if (ring->type != TYPE_EVENT &&
			!last_trb(xhci, ring, ring->enq_seg, ring->enqueue))
		ring->num_trbs_free--;
	next = ++(ring->enqueue);

	ring->enq_updates++;
	/* Update the dequeue pointer further if that was a link TRB or we're at
	 * the end of an event ring segment (which doesn't have link TRBS)
	 */
	while (last_trb(xhci, ring, ring->enq_seg, next)) {
		if (ring->type != TYPE_EVENT) {
			/*
			 * If the caller doesn't plan on enqueueing more
			 * TDs before ringing the doorbell, then we
			 * don't want to give the link TRB to the
			 * hardware just yet.  We'll give the link TRB
			 * back in prepare_ring() just before we enqueue
			 * the TD at the top of the ring.
			 */
			if (!chain && !more_trbs_coming)
				break;

			/* If we're not dealing with 0.95 hardware or
			 * isoc rings on AMD 0.96 host,
			 * carry over the chain bit of the previous TRB
			 * (which may mean the chain bit is cleared).
			 */
			if (!(ring->type == TYPE_ISOC &&
					(xhci->quirks & XHCI_AMD_0x96_HOST))
						&& !xhci_link_trb_quirk(xhci)) {
				next->link.control &=
					cpu_to_le32(~TRB_CHAIN);
				next->link.control |=
					cpu_to_le32(chain);
			}
			/* Give this link TRB to the hardware */
			wmb();
			next->link.control ^= cpu_to_le32(TRB_CYCLE);

			/* Toggle the cycle bit after the last ring segment. */
			if (last_trb_on_last_seg(xhci, ring, ring->enq_seg, next)) {
				ring->cycle_state ^= 1;
			}
		}
		ring->enq_seg = ring->enq_seg->next;
		ring->enqueue = ring->enq_seg->trbs;
		next = ring->enqueue;
	}
}

/*
 * Check to see if there's room to enqueue num_trbs on the ring and make sure
 * enqueue pointer will not advance into dequeue segment. See rules above.
 */
static inline int room_on_ring(struct xhci_hcd *xhci, struct xhci_ring *ring,
		unsigned int num_trbs)
{
	int num_trbs_in_deq_seg;

	if (ring->num_trbs_free < num_trbs)
		return 0;

	if (ring->type != TYPE_COMMAND && ring->type != TYPE_EVENT) {
		num_trbs_in_deq_seg = ring->dequeue - ring->deq_seg->trbs;
		if (ring->num_trbs_free < num_trbs + num_trbs_in_deq_seg)
			return 0;
	}

	return 1;
}

/* Ring the host controller doorbell after placing a command on the ring */
void xhci_ring_cmd_db(struct xhci_hcd *xhci)
{
	if (!(xhci->cmd_ring_state & CMD_RING_STATE_RUNNING))
		return;

	xhci_dbg(xhci, "// Ding dong!\n");
	writel(DB_VALUE_HOST, &xhci->dba->doorbell[0]);
	/* Flush PCI posted writes */
	readl(&xhci->dba->doorbell[0]);
}

static int xhci_abort_cmd_ring(struct xhci_hcd *xhci)
{
	u64 temp_64;
	int ret;

	xhci_dbg(xhci, "Abort command ring\n");

	temp_64 = xhci_read_64(xhci, &xhci->op_regs->cmd_ring);
	xhci->cmd_ring_state = CMD_RING_STATE_ABORTED;
	xhci_write_64(xhci, temp_64 | CMD_RING_ABORT,
			&xhci->op_regs->cmd_ring);

	/* Section 4.6.1.2 of xHCI 1.0 spec says software should
	 * time the completion od all xHCI commands, including
	 * the Command Abort operation. If software doesn't see
	 * CRR negated in a timely manner (e.g. longer than 5
	 * seconds), then it should assume that the there are
	 * larger problems with the xHC and assert HCRST.
	 */
	ret = xhci_handshake(&xhci->op_regs->cmd_ring,
			CMD_RING_RUNNING, 0, 5 * 1000 * 1000);
	if (ret < 0) {
		/* we are about to kill xhci, give it one more chance */
		xhci_write_64(xhci, temp_64 | CMD_RING_ABORT,
			      &xhci->op_regs->cmd_ring);
		udelay(1000);
		ret = xhci_handshake(&xhci->op_regs->cmd_ring,
				     CMD_RING_RUNNING, 0, 3 * 1000 * 1000);
		if (ret == 0)
			return 0;

		xhci_err(xhci, "Stopped the command ring failed, "
				"maybe the host is dead\n");
		xhci->xhc_state |= XHCI_STATE_DYING;
		xhci_quiesce(xhci);
		xhci_halt(xhci);
		return -ESHUTDOWN;
	}

	return 0;
}

void xhci_ring_ep_doorbell(struct xhci_hcd *xhci,
		unsigned int slot_id,
		unsigned int ep_index,
		unsigned int stream_id)
{
	__le32 __iomem *db_addr = &xhci->dba->doorbell[slot_id];
	struct xhci_virt_ep *ep = &xhci->devs[slot_id]->eps[ep_index];
	unsigned int ep_state = ep->ep_state;

	/* Don't ring the doorbell for this endpoint if there are pending
	 * cancellations because we don't want to interrupt processing.
	 * We don't want to restart any stream rings if there's a set dequeue
	 * pointer command pending because the device can choose to start any
	 * stream once the endpoint is on the HW schedule.
	 */
	if ((ep_state & EP_HALT_PENDING) || (ep_state & SET_DEQ_PENDING) ||
	    (ep_state & EP_HALTED))
		return;
	writel(DB_VALUE(ep_index, stream_id), db_addr);
	/* The CPU has better things to do at this point than wait for a
	 * write-posting flush.  It'll get there soon enough.
	 */
}

static struct xhci_ring *xhci_triad_to_transfer_ring(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		unsigned int stream_id)
{
	struct xhci_virt_ep *ep;

	ep = &xhci->devs[slot_id]->eps[ep_index];
	/* Common case: no streams */
	if (!(ep->ep_state & EP_HAS_STREAMS))
		return ep->ring;

	if (stream_id == 0) {
		xhci_warn(xhci,
				"WARN: Slot ID %u, ep index %u has streams, "
				"but URB has no stream ID.\n",
				slot_id, ep_index);
		return NULL;
	}

	if (stream_id < ep->stream_info->num_streams)
		return ep->stream_info->stream_rings[stream_id];

	xhci_warn(xhci,
			"WARN: Slot ID %u, ep index %u has "
			"stream IDs 1 to %u allocated, "
			"but stream ID %u is requested.\n",
			slot_id, ep_index,
			ep->stream_info->num_streams - 1,
			stream_id);
	return NULL;
}

/* Get the right ring for the given URB.
 * If the endpoint supports streams, boundary check the URB's stream ID.
 * If the endpoint doesn't support streams, return the singular endpoint ring.
 */
static struct xhci_ring *xhci_urb_to_transfer_ring(struct xhci_hcd *xhci,
		struct urb *urb)
{
	return xhci_triad_to_transfer_ring(xhci, urb->dev->slot_id,
		xhci_get_endpoint_index(&urb->ep->desc), urb->stream_id);
}

/*
 * Move the xHC's endpoint ring dequeue pointer past cur_td.
 * Record the new state of the xHC's endpoint ring dequeue segment,
 * dequeue pointer, and new consumer cycle state in state.
 * Update our internal representation of the ring's dequeue pointer.
 *
 * We do this in three jumps:
 *  - First we update our new ring state to be the same as when the xHC stopped.
 *  - Then we traverse the ring to find the segment that contains
 *    the last TRB in the TD.  We toggle the xHC's new cycle state when we pass
 *    any link TRBs with the toggle cycle bit set.
 *  - Finally we move the dequeue state one TRB further, toggling the cycle bit
 *    if we've moved it past a link TRB with the toggle cycle bit set.
 *
 * Some of the uses of xhci_generic_trb are grotty, but if they're done
 * with correct __le32 accesses they should work fine.  Only users of this are
 * in here.
 */
void xhci_find_new_dequeue_state(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		unsigned int stream_id, struct xhci_td *cur_td,
		struct xhci_dequeue_state *state)
{
	struct xhci_virt_device *dev = xhci->devs[slot_id];
	struct xhci_virt_ep *ep = &dev->eps[ep_index];
	struct xhci_ring *ep_ring;
	struct xhci_segment *new_seg;
	union xhci_trb *new_deq;
	dma_addr_t addr;
	u64 hw_dequeue;
	bool cycle_found = false;
	bool td_last_trb_found = false;

	ep_ring = xhci_triad_to_transfer_ring(xhci, slot_id,
			ep_index, stream_id);
	if (!ep_ring) {
		xhci_warn(xhci, "WARN can't find new dequeue state "
				"for invalid stream ID %u.\n",
				stream_id);
		return;
	}

	/* Dig out the cycle state saved by the xHC during the stop ep cmd */
	xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
			"Finding endpoint context");
	/* 4.6.9 the css flag is written to the stream context for streams */
	if (ep->ep_state & EP_HAS_STREAMS) {
		struct xhci_stream_ctx *ctx =
			&ep->stream_info->stream_ctx_array[stream_id];
		hw_dequeue = le64_to_cpu(ctx->stream_ring);
	} else {
		struct xhci_ep_ctx *ep_ctx
			= xhci_get_ep_ctx(xhci, dev->out_ctx, ep_index);
		hw_dequeue = le64_to_cpu(ep_ctx->deq);
	}

	new_seg = ep_ring->deq_seg;
	new_deq = ep_ring->dequeue;
	state->new_cycle_state = hw_dequeue & 0x1;

	/*
	 * We want to find the pointer, segment and cycle state of the new trb
	 * (the one after current TD's last_trb). We know the cycle state at
	 * hw_dequeue, so walk the ring until both hw_dequeue and last_trb are
	 * found.
	 */
	do {
		if (!cycle_found && xhci_trb_virt_to_dma(new_seg, new_deq)
		    == (dma_addr_t)(hw_dequeue & ~0xf)) {
			cycle_found = true;
			if (td_last_trb_found)
				break;
		}
		if (new_deq == cur_td->last_trb)
			td_last_trb_found = true;

		if (cycle_found &&
		    TRB_TYPE_LINK_LE32(new_deq->generic.field[3]) &&
		    new_deq->generic.field[3] & cpu_to_le32(LINK_TOGGLE))
			state->new_cycle_state ^= 0x1;

		next_trb(xhci, ep_ring, &new_seg, &new_deq);

		/* Search wrapped around, bail out */
		if (new_deq == ep->ring->dequeue) {
			xhci_err(xhci, "Error: Failed finding new dequeue state\n");
			state->new_deq_seg = NULL;
			state->new_deq_ptr = NULL;
			return;
		}

	} while (!cycle_found || !td_last_trb_found);

	state->new_deq_seg = new_seg;
	state->new_deq_ptr = new_deq;

	/* Don't update the ring cycle state for the producer (us). */
	xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
			"Cycle state = 0x%x", state->new_cycle_state);

	xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
			"New dequeue segment = %p (virtual)",
			state->new_deq_seg);
	addr = xhci_trb_virt_to_dma(state->new_deq_seg, state->new_deq_ptr);
	xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
			"New dequeue pointer = 0x%llx (DMA)",
			(unsigned long long) addr);
}

/* flip_cycle means flip the cycle bit of all but the first and last TRB.
 * (The last TRB actually points to the ring enqueue pointer, which is not part
 * of this TD.)  This is used to remove partially enqueued isoc TDs from a ring.
 */
static void td_to_noop(struct xhci_hcd *xhci, struct xhci_ring *ep_ring,
		struct xhci_td *cur_td, bool flip_cycle)
{
	struct xhci_segment *cur_seg;
	union xhci_trb *cur_trb;

	for (cur_seg = cur_td->start_seg, cur_trb = cur_td->first_trb;
			true;
			next_trb(xhci, ep_ring, &cur_seg, &cur_trb)) {
		if (TRB_TYPE_LINK_LE32(cur_trb->generic.field[3])) {
			/* Unchain any chained Link TRBs, but
			 * leave the pointers intact.
			 */
			cur_trb->generic.field[3] &= cpu_to_le32(~TRB_CHAIN);
			/* Flip the cycle bit (link TRBs can't be the first
			 * or last TRB).
			 */
			if (flip_cycle)
				cur_trb->generic.field[3] ^=
					cpu_to_le32(TRB_CYCLE);
			xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
					"Cancel (unchain) link TRB");
			xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
					"Address = %p (0x%llx dma); "
					"in seg %p (0x%llx dma)",
					cur_trb,
					(unsigned long long)xhci_trb_virt_to_dma(cur_seg, cur_trb),
					cur_seg,
					(unsigned long long)cur_seg->dma);
		} else {
			cur_trb->generic.field[0] = 0;
			cur_trb->generic.field[1] = 0;
			cur_trb->generic.field[2] = 0;
			/* Preserve only the cycle bit of this TRB */
			cur_trb->generic.field[3] &= cpu_to_le32(TRB_CYCLE);
			/* Flip the cycle bit except on the first or last TRB */
			if (flip_cycle && cur_trb != cur_td->first_trb &&
					cur_trb != cur_td->last_trb)
				cur_trb->generic.field[3] ^=
					cpu_to_le32(TRB_CYCLE);
			cur_trb->generic.field[3] |= cpu_to_le32(
				TRB_TYPE(TRB_TR_NOOP));
			xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
					"TRB to noop at offset 0x%llx",
					(unsigned long long)
					xhci_trb_virt_to_dma(cur_seg, cur_trb));
		}
		if (cur_trb == cur_td->last_trb)
			break;
	}
}


/* Must be called with xhci->lock held in interrupt context */
static void xhci_giveback_urb_in_irq(struct xhci_hcd *xhci,
		struct xhci_td *cur_td, int status)
{
	struct usb_hcd *hcd;
	struct urb	*urb;
	struct urb_priv	*urb_priv;

	urb = cur_td->urb;
	urb_priv = urb->hcpriv;
	urb_priv->td_cnt++;
	hcd = bus_to_hcd(urb->dev->bus);

	/* Only giveback urb when this is the last td in urb */
	if (urb_priv->td_cnt == urb_priv->length) {
		if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
			xhci_to_hcd(xhci)->self.bandwidth_isoc_reqs--;
			if (xhci_to_hcd(xhci)->self.bandwidth_isoc_reqs	== 0) {
				if (xhci->quirks & XHCI_AMD_PLL_FIX)
					usb_amd_quirk_pll_enable();
			}
		}
		usb_hcd_unlink_urb_from_ep(hcd, urb);

		spin_unlock(&xhci->lock);
		usb_hcd_giveback_urb(hcd, urb, status);
		xhci_urb_free_priv(xhci,urb_priv);
		spin_lock(&xhci->lock);
	}
}

static void xhci_kill_ring_urbs(struct xhci_hcd *xhci, struct xhci_ring *ring)
{
	struct xhci_td *cur_td;

	while (!list_empty(&ring->td_list)) {
		cur_td = list_first_entry(&ring->td_list,
				struct xhci_td, td_list);
		list_del_init(&cur_td->td_list);
		if (!list_empty(&cur_td->cancelled_td_list))
			list_del_init(&cur_td->cancelled_td_list);
		xhci_giveback_urb_in_irq(xhci, cur_td, -ESHUTDOWN);
	}
}

static void xhci_kill_endpoint_urbs(struct xhci_hcd *xhci,
		int slot_id, int ep_index)
{
	struct xhci_td *cur_td;
	struct xhci_virt_ep *ep;
	struct xhci_ring *ring;

	ep = &xhci->devs[slot_id]->eps[ep_index];
	if ((ep->ep_state & EP_HAS_STREAMS) ||
			(ep->ep_state & EP_GETTING_NO_STREAMS)) {
		int stream_id;

		for (stream_id = 0; stream_id < ep->stream_info->num_streams;
				stream_id++) {
			xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
					"Killing URBs for slot ID %u, ep index %u, stream %u",
					slot_id, ep_index, stream_id + 1);
			xhci_kill_ring_urbs(xhci,
					ep->stream_info->stream_rings[stream_id]);
		}
	} else {
		ring = ep->ring;
		if (!ring)
			return;
		xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
				"Killing URBs for slot ID %u, ep index %u",
				slot_id, ep_index);
		xhci_kill_ring_urbs(xhci, ring);
	}
	while (!list_empty(&ep->cancelled_td_list)) {
		cur_td = list_first_entry(&ep->cancelled_td_list,
				struct xhci_td, cancelled_td_list);
		list_del_init(&cur_td->cancelled_td_list);
		xhci_giveback_urb_in_irq(xhci, cur_td, -ESHUTDOWN);
	}
}

/* Watchdog timer function for when a stop endpoint command fails to complete.
 * In this case, we assume the host controller is broken or dying or dead.  The
 * host may still be completing some other events, so we have to be careful to
 * let the event ring handler and the URB dequeueing/enqueueing functions know
 * through xhci->state.
 *
 * The timer may also fire if the host takes a very long time to respond to the
 * command, and the stop endpoint command completion handler cannot delete the
 * timer before the timer function is called.  Another endpoint cancellation may
 * sneak in before the timer function can grab the lock, and that may queue
 * another stop endpoint command and add the timer back.  So we cannot use a
 * simple flag to say whether there is a pending stop endpoint command for a
 * particular endpoint.
 *
 * Instead we use a combination of that flag and a counter for the number of
 * pending stop endpoint commands.  If the timer is the tail end of the last
 * stop endpoint command, and the endpoint's command is still pending, we assume
 * the host is dying.
 */
void xhci_stop_endpoint_command_watchdog(unsigned long arg)
{
	struct xhci_hcd *xhci;
	struct xhci_virt_ep *ep;
	int ret, i, j;
	unsigned long flags;

	ep = (struct xhci_virt_ep *) arg;
	xhci = ep->xhci;

	spin_lock_irqsave(&xhci->lock, flags);

	ep->stop_cmds_pending--;
	if (xhci->xhc_state & XHCI_STATE_DYING) {
		xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
				"Stop EP timer ran, but another timer marked "
				"xHCI as DYING, exiting.");
		spin_unlock_irqrestore(&xhci->lock, flags);
		return;
	}
	if (!(ep->stop_cmds_pending == 0 && (ep->ep_state & EP_HALT_PENDING))) {
		xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
				"Stop EP timer ran, but no command pending, "
				"exiting.");
		spin_unlock_irqrestore(&xhci->lock, flags);
		return;
	}

	xhci_warn(xhci, "xHCI host not responding to stop endpoint command.\n");
	xhci_warn(xhci, "Assuming host is dying, halting host.\n");
	/* Oops, HC is dead or dying or at least not responding to the stop
	 * endpoint command.
	 */
	xhci->xhc_state |= XHCI_STATE_DYING;
	/* Disable interrupts from the host controller and start halting it */
	xhci_quiesce(xhci);
	spin_unlock_irqrestore(&xhci->lock, flags);

	ret = xhci_halt(xhci);

	spin_lock_irqsave(&xhci->lock, flags);
	if (ret < 0) {
		/* This is bad; the host is not responding to commands and it's
		 * not allowing itself to be halted.  At least interrupts are
		 * disabled. If we call usb_hc_died(), it will attempt to
		 * disconnect all device drivers under this host.  Those
		 * disconnect() methods will wait for all URBs to be unlinked,
		 * so we must complete them.
		 */
		xhci_warn(xhci, "Non-responsive xHCI host is not halting.\n");
		xhci_warn(xhci, "Completing active URBs anyway.\n");
		/* We could turn all TDs on the rings to no-ops.  This won't
		 * help if the host has cached part of the ring, and is slow if
		 * we want to preserve the cycle bit.  Skip it and hope the host
		 * doesn't touch the memory.
		 */
	}
	for (i = 0; i < MAX_HC_SLOTS; i++) {
		if (!xhci->devs[i])
			continue;
		for (j = 0; j < 31; j++)
			xhci_kill_endpoint_urbs(xhci, i, j);
	}
	spin_unlock_irqrestore(&xhci->lock, flags);
	xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
			"Calling usb_hc_died()");
	usb_hc_died(xhci_to_hcd(xhci)->primary_hcd);
	xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
			"xHCI host controller is dead.");
}

static void xhci_complete_del_and_free_cmd(struct xhci_command *cmd, u32 status)
{
	list_del(&cmd->cmd_list);

	if (cmd->completion) {
		cmd->status = status;
		complete(cmd->completion);
	} else {
		kfree(cmd);
	}
}

void xhci_cleanup_command_queue(struct xhci_hcd *xhci)
{
	struct xhci_command *cur_cmd, *tmp_cmd;
	list_for_each_entry_safe(cur_cmd, tmp_cmd, &xhci->cmd_list, cmd_list)
		xhci_complete_del_and_free_cmd(cur_cmd, COMP_CMD_ABORT);
}

/*
 * Turn all commands on command ring with status set to "aborted" to no-op trbs.
 * If there are other commands waiting then restart the ring and kick the timer.
 * This must be called with command ring stopped and xhci->lock held.
 */
static void xhci_handle_stopped_cmd_ring(struct xhci_hcd *xhci,
					 struct xhci_command *cur_cmd)
{
	struct xhci_command *i_cmd, *tmp_cmd;
	u32 cycle_state;

	/* Turn all aborted commands in list to no-ops, then restart */
	list_for_each_entry_safe(i_cmd, tmp_cmd, &xhci->cmd_list,
				 cmd_list) {

		if (i_cmd->status != COMP_CMD_ABORT)
			continue;

		i_cmd->status = COMP_CMD_STOP;

		xhci_dbg(xhci, "Turn aborted command %p to no-op\n",
			 i_cmd->command_trb);
		/* get cycle state from the original cmd trb */
		cycle_state = le32_to_cpu(
			i_cmd->command_trb->generic.field[3]) &	TRB_CYCLE;
		/* modify the command trb to no-op command */
		i_cmd->command_trb->generic.field[0] = 0;
		i_cmd->command_trb->generic.field[1] = 0;
		i_cmd->command_trb->generic.field[2] = 0;
		i_cmd->command_trb->generic.field[3] = cpu_to_le32(
			TRB_TYPE(TRB_CMD_NOOP) | cycle_state);

		/*
		 * caller waiting for completion is called when command
		 *  completion event is received for these no-op commands
		 */
	}

	xhci->cmd_ring_state = CMD_RING_STATE_RUNNING;

	/* ring command ring doorbell to restart the command ring */
	if ((xhci->cmd_ring->dequeue != xhci->cmd_ring->enqueue) &&
	    !(xhci->xhc_state & XHCI_STATE_DYING)) {
		xhci->current_cmd = cur_cmd;
		mod_timer(&xhci->cmd_timer, jiffies + XHCI_CMD_DEFAULT_TIMEOUT);
		xhci_ring_cmd_db(xhci);
	}
	return;
}


void xhci_handle_command_timeout(unsigned long data)
{
	struct xhci_hcd *xhci;
	int ret;
	unsigned long flags;
	u64 hw_ring_state;
	struct xhci_command *cur_cmd = NULL;
	xhci = (struct xhci_hcd *) data;

	/* mark this command to be cancelled */
	spin_lock_irqsave(&xhci->lock, flags);
	if (xhci->current_cmd) {
		cur_cmd = xhci->current_cmd;
		cur_cmd->status = COMP_CMD_ABORT;
	}


	/* Make sure command ring is running before aborting it */
	hw_ring_state = xhci_read_64(xhci, &xhci->op_regs->cmd_ring);
	if ((xhci->cmd_ring_state & CMD_RING_STATE_RUNNING) &&
	    (hw_ring_state & CMD_RING_RUNNING))  {

		spin_unlock_irqrestore(&xhci->lock, flags);
		xhci_dbg(xhci, "Command timeout\n");
		ret = xhci_abort_cmd_ring(xhci);
		if (unlikely(ret == -ESHUTDOWN)) {
			xhci_err(xhci, "Abort command ring failed\n");
			xhci_cleanup_command_queue(xhci);
			usb_hc_died(xhci_to_hcd(xhci)->primary_hcd);
			xhci_dbg(xhci, "xHCI host controller is dead.\n");
		}
		return;
	}
	/* command timeout on stopped ring, ring can't be aborted */
	xhci_dbg(xhci, "Command timeout on stopped ring\n");
	xhci_handle_stopped_cmd_ring(xhci, xhci->current_cmd);
	spin_unlock_irqrestore(&xhci->lock, flags);
	return;
}

/*
 * This TD is defined by the TRBs starting at start_trb in start_seg and ending
 * at end_trb, which may be in another segment.  If the suspect DMA address is a
 * TRB in this TD, this function returns that TRB's segment.  Otherwise it
 * returns 0.
 */
struct xhci_segment *trb_in_td(struct xhci_hcd *xhci,
		struct xhci_segment *start_seg,
		union xhci_trb	*start_trb,
		union xhci_trb	*end_trb,
		dma_addr_t	suspect_dma,
		bool		debug)
{
	dma_addr_t start_dma;
	dma_addr_t end_seg_dma;
	dma_addr_t end_trb_dma;
	struct xhci_segment *cur_seg;

	start_dma = xhci_trb_virt_to_dma(start_seg, start_trb);
	cur_seg = start_seg;

	do {
		if (start_dma == 0)
			return NULL;
		/* We may get an event for a Link TRB in the middle of a TD */
		end_seg_dma = xhci_trb_virt_to_dma(cur_seg,
				&cur_seg->trbs[TRBS_PER_SEGMENT - 1]);
		/* If the end TRB isn't in this segment, this is set to 0 */
		end_trb_dma = xhci_trb_virt_to_dma(cur_seg, end_trb);

		if (debug)
			xhci_warn(xhci,
				"Looking for event-dma %016llx trb-start %016llx trb-end %016llx seg-start %016llx seg-end %016llx\n",
				(unsigned long long)suspect_dma,
				(unsigned long long)start_dma,
				(unsigned long long)end_trb_dma,
				(unsigned long long)cur_seg->dma,
				(unsigned long long)end_seg_dma);

		if (end_trb_dma > 0) {
			/* The end TRB is in this segment, so suspect should be here */
			if (start_dma <= end_trb_dma) {
				if (suspect_dma >= start_dma && suspect_dma <= end_trb_dma)
					return cur_seg;
			} else {
				/* Case for one segment with
				 * a TD wrapped around to the top
				 */
				if ((suspect_dma >= start_dma &&
							suspect_dma <= end_seg_dma) ||
						(suspect_dma >= cur_seg->dma &&
						 suspect_dma <= end_trb_dma))
					return cur_seg;
			}
			return NULL;
		} else {
			/* Might still be somewhere in this segment */
			if (suspect_dma >= start_dma && suspect_dma <= end_seg_dma)
				return cur_seg;
		}
		cur_seg = cur_seg->next;
		start_dma = xhci_trb_virt_to_dma(cur_seg, &cur_seg->trbs[0]);
	} while (cur_seg != start_seg);

	return NULL;
}

int xhci_is_vendor_info_code(struct xhci_hcd *xhci, unsigned int trb_comp_code)
{
	if (trb_comp_code >= 224 && trb_comp_code <= 255) {
		/* Vendor defined "informational" completion code,
		 * treat as not-an-error.
		 */
		xhci_dbg(xhci, "Vendor defined info completion code %u\n",
				trb_comp_code);
		xhci_dbg(xhci, "Treating code as success.\n");
		return 1;
	}
	return 0;
}

/****		Endpoint Ring Operations	****/

/*
 * Generic function for queueing a TRB on a ring.
 * The caller must have checked to make sure there's room on the ring.
 *
 * @more_trbs_coming:	Will you enqueue more TRBs before calling
 *			prepare_transfer()?
 */
 void queue_trb(struct xhci_hcd *xhci, struct xhci_ring *ring,
		bool more_trbs_coming,
		u32 field1, u32 field2, u32 field3, u32 field4)
{
	struct xhci_generic_trb *trb;

	trb = &ring->enqueue->generic;
	trb->field[0] = cpu_to_le32(field1);
	trb->field[1] = cpu_to_le32(field2);
	trb->field[2] = cpu_to_le32(field3);
	trb->field[3] = cpu_to_le32(field4);
	inc_enq(xhci, ring, more_trbs_coming);
}

/*
 * Does various checks on the endpoint ring, and makes it ready to queue num_trbs.
 * FIXME allocate segments if the ring is full.
 */
 int prepare_ring(struct xhci_hcd *xhci, struct xhci_ring *ep_ring,
		u32 ep_state, unsigned int num_trbs, gfp_t mem_flags)
{
	unsigned int num_trbs_needed;

	/* Make sure the endpoint has been added to xHC schedule */
	switch (ep_state) {
	case EP_STATE_DISABLED:
		/*
		 * USB core changed config/interfaces without notifying us,
		 * or hardware is reporting the wrong state.
		 */
		xhci_warn(xhci, "WARN urb submitted to disabled ep\n");
		return -ENOENT;
	case EP_STATE_ERROR:
		xhci_warn(xhci, "WARN waiting for error on ep to be cleared\n");
		/* FIXME event handling code for error needs to clear it */
		/* XXX not sure if this should be -ENOENT or not */
		return -EINVAL;
	case EP_STATE_HALTED:
		xhci_dbg(xhci, "WARN halted endpoint, queueing URB anyway.\n");
	case EP_STATE_STOPPED:
	case EP_STATE_RUNNING:
		break;
	default:
		xhci_err(xhci, "ERROR unknown endpoint state for ep\n");
		/*
		 * FIXME issue Configure Endpoint command to try to get the HC
		 * back into a known state.
		 */
		return -EINVAL;
	}

	while (1) {
		if (room_on_ring(xhci, ep_ring, num_trbs))
			break;

		if (ep_ring == xhci->cmd_ring) {
			xhci_err(xhci, "Do not support expand command ring\n");
			return -ENOMEM;
		}

		xhci_dbg_trace(xhci, trace_xhci_dbg_ring_expansion,
				"ERROR no room on ep ring, try ring expansion");
		num_trbs_needed = num_trbs - ep_ring->num_trbs_free;
		if (xhci_ring_expansion(xhci, ep_ring, num_trbs_needed,
					mem_flags)) {
			xhci_err(xhci, "Ring expansion failed\n");
			return -ENOMEM;
		}
	}

	if (enqueue_is_link_trb(ep_ring)) {
		struct xhci_ring *ring = ep_ring;
		union xhci_trb *next;

		next = ring->enqueue;

		while (last_trb(xhci, ring, ring->enq_seg, next)) {
			/* If we're not dealing with 0.95 hardware or isoc rings
			 * on AMD 0.96 host, clear the chain bit.
			 */
			if (!xhci_link_trb_quirk(xhci) &&
					!(ring->type == TYPE_ISOC &&
					 (xhci->quirks & XHCI_AMD_0x96_HOST)))
				next->link.control &= cpu_to_le32(~TRB_CHAIN);
			else
				next->link.control |= cpu_to_le32(TRB_CHAIN);

			wmb();
			next->link.control ^= cpu_to_le32(TRB_CYCLE);

			/* Toggle the cycle bit after the last ring segment. */
			if (last_trb_on_last_seg(xhci, ring, ring->enq_seg, next)) {
				ring->cycle_state ^= 1;
			}
			ring->enq_seg = ring->enq_seg->next;
			ring->enqueue = ring->enq_seg->trbs;
			next = ring->enqueue;
		}
	}

	return 0;
}

static int prepare_transfer(struct xhci_hcd *xhci,
		struct xhci_virt_device *xdev,
		unsigned int ep_index,
		unsigned int stream_id,
		unsigned int num_trbs,
		struct urb *urb,
		unsigned int td_index,
		gfp_t mem_flags)
{
	int ret;
	struct urb_priv *urb_priv;
	struct xhci_td	*td;
	struct xhci_ring *ep_ring;
	struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);

	ep_ring = xhci_stream_id_to_ring(xdev, ep_index, stream_id);
	if (!ep_ring) {
		xhci_dbg(xhci, "Can't prepare ring for bad stream ID %u\n",
				stream_id);
		return -EINVAL;
	}

	ret = prepare_ring(xhci, ep_ring,
			   le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK,
			   num_trbs, mem_flags);
	if (ret)
		return ret;

	urb_priv = urb->hcpriv;
	td = urb_priv->td[td_index];

	INIT_LIST_HEAD(&td->td_list);
	INIT_LIST_HEAD(&td->cancelled_td_list);

	if (td_index == 0) {
		ret = usb_hcd_link_urb_to_ep(bus_to_hcd(urb->dev->bus), urb);
		if (unlikely(ret))
			return ret;
	}

	td->urb = urb;
	/* Add this TD to the tail of the endpoint ring's TD list */
	list_add_tail(&td->td_list, &ep_ring->td_list);
	td->start_seg = ep_ring->enq_seg;
	td->first_trb = ep_ring->enqueue;

	urb_priv->td[td_index] = td;

	return 0;
}

static unsigned int count_sg_trbs_needed(struct xhci_hcd *xhci, struct urb *urb)
{
	int num_sgs, num_trbs, running_total, temp, i;
	struct scatterlist *sg;

	sg = NULL;
	num_sgs = urb->num_mapped_sgs;
	temp = urb->transfer_buffer_length;

	num_trbs = 0;
	for_each_sg(urb->sg, sg, num_sgs, i) {
		unsigned int len = sg_dma_len(sg);

		/* Scatter gather list entries may cross 64KB boundaries */
		running_total = TRB_MAX_BUFF_SIZE -
			(sg_dma_address(sg) & (TRB_MAX_BUFF_SIZE - 1));
		running_total &= TRB_MAX_BUFF_SIZE - 1;
		if (running_total != 0)
			num_trbs++;

		/* How many more 64KB chunks to transfer, how many more TRBs? */
		while (running_total < sg_dma_len(sg) && running_total < temp) {
			num_trbs++;
			running_total += TRB_MAX_BUFF_SIZE;
		}
		len = min_t(int, len, temp);
		temp -= len;
		if (temp == 0)
			break;
	}
	return num_trbs;
}

static int count_isoc_trbs_needed(struct xhci_hcd *xhci,
		struct urb *urb, int i)
{
	int num_trbs = 0;
	u64 addr, td_len;

	addr = (u64) (urb->transfer_dma + urb->iso_frame_desc[i].offset);
	td_len = urb->iso_frame_desc[i].length;

	num_trbs = DIV_ROUND_UP(td_len + (addr & (TRB_MAX_BUFF_SIZE - 1)),
			TRB_MAX_BUFF_SIZE);
	if (num_trbs == 0)
		num_trbs++;

	return num_trbs;
}
static int ep_ring_is_processing(struct xhci_hcd *xhci,
		int slot_id, unsigned int ep_index)
{
	struct xhci_virt_device *xdev;
	struct xhci_ring *ep_ring;
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_virt_ep *xep;
	dma_addr_t hw_deq;

	xdev = xhci->devs[slot_id];
	xep = &xhci->devs[slot_id]->eps[ep_index];
	ep_ring = xep->ring;
	ep_ctx = xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);

	if ((le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK) != EP_STATE_RUNNING)
		return 0;

	hw_deq = le64_to_cpu(ep_ctx->deq) & ~EP_CTX_CYCLE_MASK;
	return (hw_deq !=
		xhci_trb_virt_to_dma(ep_ring->enq_seg, ep_ring->enqueue));
}

static void check_trb_math(struct urb *urb, int num_trbs, int running_total)
{
	if (num_trbs != 0)
		dev_err(&urb->dev->dev, "%s - ep %#x - Miscalculated number of "
				"TRBs, %d left\n", __func__,
				urb->ep->desc.bEndpointAddress, num_trbs);
	if (running_total != urb->transfer_buffer_length)
		dev_err(&urb->dev->dev, "%s - ep %#x - Miscalculated tx length, "
				"queued %#x (%d), asked for %#x (%d)\n",
				__func__,
				urb->ep->desc.bEndpointAddress,
				running_total, running_total,
				urb->transfer_buffer_length,
				urb->transfer_buffer_length);
}

static void giveback_first_trb(struct xhci_hcd *xhci, int slot_id,
		unsigned int ep_index, unsigned int stream_id, int start_cycle,
		struct xhci_generic_trb *start_trb)
{
	/*
	 * Pass all the TRBs to the hardware at once and make sure this write
	 * isn't reordered.
	 */
	wmb();
	if (start_cycle)
		start_trb->field[3] |= cpu_to_le32(start_cycle);
	else
		start_trb->field[3] &= cpu_to_le32(~TRB_CYCLE);
	xhci_ring_ep_doorbell(xhci, slot_id, ep_index, stream_id);
}

/*
 * xHCI uses normal TRBs for both bulk and interrupt.  When the interrupt
 * endpoint is to be serviced, the xHC will consume (at most) one TD.  A TD
 * (comprised of sg list entries) can take several service intervals to
 * transmit.
 */
int xhci_queue_intr_tx(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(xhci,
			xhci->devs[slot_id]->out_ctx, ep_index);
	int xhci_interval;
	int ep_interval;

	xhci_interval = EP_INTERVAL_TO_UFRAMES(le32_to_cpu(ep_ctx->ep_info));
	ep_interval = urb->interval;
	/* Convert to microframes */
	if (urb->dev->speed == USB_SPEED_LOW ||
			urb->dev->speed == USB_SPEED_FULL)
		ep_interval *= 8;
	/* FIXME change this to a warning and a suggestion to use the new API
	 * to set the polling interval (once the API is added).
	 */
	if (xhci_interval != ep_interval) {
		dev_dbg_ratelimited(&urb->dev->dev,
				"Driver uses different interval (%d microframe%s) than xHCI (%d microframe%s)\n",
				ep_interval, ep_interval == 1 ? "" : "s",
				xhci_interval, xhci_interval == 1 ? "" : "s");
		urb->interval = xhci_interval;
		/* Convert back to frames for LS/FS devices */
		if (urb->dev->speed == USB_SPEED_LOW ||
				urb->dev->speed == USB_SPEED_FULL)
			urb->interval /= 8;
	}
	return xhci_queue_bulk_tx(xhci, mem_flags, urb, slot_id, ep_index);
}

/*
 * For xHCI 1.0 host controllers, TD size is the number of max packet sized
 * packets remaining in the TD (*not* including this TRB).
 *
 * Total TD packet count = total_packet_count =
 *     DIV_ROUND_UP(TD size in bytes / wMaxPacketSize)
 *
 * Packets transferred up to and including this TRB = packets_transferred =
 *     rounddown(total bytes transferred including this TRB / wMaxPacketSize)
 *
 * TD size = total_packet_count - packets_transferred
 *
 * For xHCI 0.96 and older, TD size field should be the remaining bytes
 * including this TRB, right shifted by 10
 *
 * For all hosts it must fit in bits 21:17, so it can't be bigger than 31.
 * This is taken care of in the TRB_TD_SIZE() macro
 *
 * The last TRB in a TD must have the TD size set to zero.
 */
static u32 xhci_td_remainder(struct xhci_hcd *xhci, int transferred,
			      int trb_buff_len, unsigned int td_total_len,
			      struct urb *urb, unsigned int num_trbs_left)
{
	u32 maxp, total_packet_count;

	if (xhci->hci_version < 0x100)
		return ((td_total_len - transferred) >> 10);

	maxp = GET_MAX_PACKET(usb_endpoint_maxp(&urb->ep->desc));
	total_packet_count = DIV_ROUND_UP(td_total_len, maxp);

	/* One TRB with a zero-length data packet. */
	if (num_trbs_left == 0 || (transferred == 0 && trb_buff_len == 0) ||
	    trb_buff_len == td_total_len)
		return 0;

	/* Queueing functions don't count the current TRB into transferred */
	return (total_packet_count - ((transferred + trb_buff_len) / maxp));
}

/*weitaowang add:xhci_verify_mark start*/
/*Caller must have locked xhci->lock */
int xhci_queue_ctrl_tx(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	int num_trbs;
	int ret;
	struct usb_ctrlrequest *setup;
	struct xhci_generic_trb *start_trb;
	int start_cycle;
	u32 field, length_field, remainder;
	struct urb_priv *urb_priv;
	struct xhci_td *td;

	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if (!ep_ring)
		return -EINVAL;

	/*
	 * Need to copy setup packet into setup TRB, so we can't use the setup
	 * DMA address.
	 */
	if (!urb->setup_packet)
		return -EINVAL;

	/* 1 TRB for setup, 1 for status */
	num_trbs = 2;
	/*
	 * Don't need to check if we need additional event data and normal TRBs,
	 * since data in control transfers will never get bigger than 16MB
	 * XXX: can we get a buffer that crosses 64KB boundaries?
	 */
	if (urb->transfer_buffer_length > 0)
		num_trbs++;
	ret = prepare_transfer(xhci, xhci->devs[slot_id],
			ep_index, urb->stream_id,
			num_trbs, urb, 0, mem_flags);
	if (ret < 0)
		return ret;

	urb_priv = urb->hcpriv;
	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	/* Queue setup TRB - see section 6.4.1.2.1 */
	/* FIXME better way to translate setup_packet into two u32 fields? */
	setup = (struct usb_ctrlrequest *) urb->setup_packet;
	field = 0;
	field |= TRB_IDT | TRB_TYPE(TRB_SETUP);
	if (start_cycle == 0)
		field |= 0x1;

	/* xHCI 1.0/1.1 6.4.1.2.1: Transfer Type field */
	if (xhci->hci_version >= 0x100) {
		if (urb->transfer_buffer_length > 0) {
			if (setup->bRequestType & USB_DIR_IN)
				field |= TRB_TX_TYPE(TRB_DATA_IN);
			else
				field |= TRB_TX_TYPE(TRB_DATA_OUT);
		}
	}

	queue_trb(xhci, ep_ring, true,
		  setup->bRequestType | setup->bRequest << 8 | le16_to_cpu(setup->wValue) << 16,
		  le16_to_cpu(setup->wIndex) | le16_to_cpu(setup->wLength) << 16,
		  TRB_LEN(8) | TRB_INTR_TARGET(0),
		  /* Immediate data in pointer */
		  field);

	/* If there's data, queue data TRBs */
	/* Only set interrupt on short packet for IN endpoints */
	if (usb_urb_dir_in(urb))
		field = TRB_ISP | TRB_TYPE(TRB_DATA);
	else
		field = TRB_TYPE(TRB_DATA);

	remainder = xhci_td_remainder(xhci, 0,
				   urb->transfer_buffer_length,
				   urb->transfer_buffer_length,
				   urb, 1);

	length_field = TRB_LEN(urb->transfer_buffer_length) |
		TRB_TD_SIZE(remainder) |
		TRB_INTR_TARGET(0);

	if (urb->transfer_buffer_length > 0) {
		if (setup->bRequestType & USB_DIR_IN)
			field |= TRB_DIR_IN;
		queue_trb(xhci, ep_ring, true,
				lower_32_bits(urb->transfer_dma),
				upper_32_bits(urb->transfer_dma),
				length_field,
				field | ep_ring->cycle_state);
	}

	/* Save the DMA address of the last TRB in the TD */
	td->last_trb = ep_ring->enqueue;

	/* Queue status TRB - see Table 7 and sections 4.11.2.2 and 6.4.1.2.3 */
	/* If the device sent data, the status stage is an OUT transfer */
	if (urb->transfer_buffer_length > 0 && setup->bRequestType & USB_DIR_IN)
		field = 0;
	else
		field = TRB_DIR_IN;
	queue_trb(xhci, ep_ring, false,
			0,
			0,
			TRB_INTR_TARGET(0),
			/* Event on completion */
			field | TRB_IOC | TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state);

	giveback_first_trb(xhci, slot_id, ep_index, 0,
			start_cycle, start_trb);
	return 0;
}

/* Refactored by Jeff for Control sg Transfer */
/* Copied from xhci_queue_ctrl_sg_tx */
/* Caller must have locked xhci->lock */
/* copy from the standard routine xhci_queue_ctrl_tx and modify it with
 * sg support based on the sg support in xhci_queue_bulk_tx routine
 */
 /*weitaowang: port to linux_4.4 done*/
int xhci_queue_ctrl_sg_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
	int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	int num_trbs;
	int ret;
	struct usb_ctrlrequest *setup;
	struct xhci_generic_trb *start_trb;
	int start_cycle;
	u32 field, length_field,remainder;
	struct urb_priv *urb_priv;
	struct xhci_td *td;

	int trb_buff_len, this_sg_len, running_total;
	unsigned int total_packet_count;
	u64 addr;
	bool first_trb;
	bool more_trbs_coming;
	int data_num_trbs = 0;
	int num_sgs = 0;
	struct scatterlist *sg = NULL;

	pr_info("enter into xhci_verify_queue_ctrl_sg_tx\n");
	/* Get right transfer for this urb */
	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if(!ep_ring)
		return -EINVAL;

	/*
	 * Need to copy setup packet into setup TRB, so we can't use the setup
	 * DMA address.
	 */
	if(!urb->setup_packet)
		return -EINVAL;

	/* 1 TRB for setup, 1 for status */
	num_trbs = 2;
	/*
	 * Don't need to check if we need additional event data and normal TRBs,
	 * since data in control transfers will never get bigger than 16MB
	 * XXX: can we get a buffer that crosses 64KB boundaries?
	 */
	//if(urb->transfer_buffer_length > 0)
		//num_trbs++;  /* data stage */
	if(urb->transfer_buffer_length > 0)
	{
		if(urb->num_sgs)
		{
			data_num_trbs = count_sg_trbs_needed(xhci, urb);
			num_trbs += data_num_trbs;
			pr_info("the sg control transfer need %d TRBs\n", data_num_trbs);
		}
		else
			num_trbs++;
	}

	pr_info("So, In this control sg transfer, we need %d TRB\n", num_trbs);
	ret = prepare_transfer(xhci, xhci->devs[slot_id], 
			ep_index, urb->stream_id, num_trbs, urb, 0, mem_flags);
	if(ret < 0)
		return ret;

	urb_priv = urb->hcpriv;
	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	/* Queue setup TRB - see section 6.4.1.2.1 */
	/* FIXME better way to translate setup_packet into two u32 fields? */
	setup = (struct usb_ctrlrequest *)urb->setup_packet;
	field = 0;
	field |= TRB_IDT | TRB_TYPE(TRB_SETUP);
	if(start_cycle == 0)
		field |= 0x1;

	/* xHCI 1.0 6.4.1.2.1: Transfer Type field */
	if(xhci->hci_version == 0x100)
	{
		if(urb->transfer_buffer_length > 0)
		{
			if(setup->bRequestType & USB_DIR_IN)
				field |= TRB_TX_TYPE(TRB_DATA_IN);
			else
				field |= TRB_TX_TYPE(TRB_DATA_OUT);
		}
	}
	/* Queue setup TRB */
	queue_trb(xhci, 
			  ep_ring, 
			  true,
			  setup->bRequestType | setup->bRequest << 8 | le16_to_cpu(setup->wValue) << 16,
		      le16_to_cpu(setup->wIndex) | le16_to_cpu(setup->wLength) << 16,
		      TRB_LEN(8) | TRB_INTR_TARGET(0),
		      field);

	/* Data Stage */
	if(!urb->num_sgs)/* original control transfer data stage */
	{
		/* If there's data, queue data TRBs */
		/* Only set interrupt on short packet for IN endpoints */
		if(usb_urb_dir_in(urb)) /* Control Read */
			field = TRB_ISP | TRB_TYPE(TRB_DATA);
		else /* Control Write */
			field = TRB_TYPE(TRB_DATA);

		remainder = xhci_td_remainder(xhci, 0,
				   urb->transfer_buffer_length,
				   urb->transfer_buffer_length,
				   urb, 1);

	    length_field = TRB_LEN(urb->transfer_buffer_length) |
			  TRB_TD_SIZE(remainder) | 
			  TRB_INTR_TARGET(0);
		
		if(urb->transfer_buffer_length > 0)
		{
			if(setup->bRequestType & USB_DIR_IN)
				field |= TRB_DIR_IN;
			/* Queue Data TRB */
			queue_trb(xhci, ep_ring, true, lower_32_bits(urb->transfer_dma),
				upper_32_bits(urb->transfer_dma), length_field, field | ep_ring->cycle_state);
		}
	}
	else /* bulk control tranfer data stage */
	{
		pr_info(" Bulk control transfer branch\n");
		running_total = 0;
		/*
		 * How much data is in the first TRB?
		 *
		 * There are three forces at work for TRB buffer pointers and lengths:
		 * 1. We don't want to walk off the end of this sg-list entry buffer.
		 * 2. The transfer length that the driver requested may be smaller than
		 *    the amount of memory allocated for this scatter-gather list.
		 * 3. TRBs buffers can't cross 64KB boundaries.
		 */
		num_sgs = urb->num_sgs;
		sg = urb->sg;
		total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length, usb_endpoint_maxp(&urb->ep->desc));
			
		addr = (u64)sg_dma_address(sg);
		this_sg_len = sg_dma_len(sg);
		trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
		trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
		if(trb_buff_len > urb->transfer_buffer_length)
			trb_buff_len = urb->transfer_buffer_length;

		first_trb = true;
		/* Queue the first TRB, even if it's zero-length */
		do
		{
			u32 field = 0;
			u32 length_field = 0;
			//u32 remainder = 0;

			if(usb_urb_dir_in(urb))/* Control Read */
				field = TRB_ISP | TRB_TYPE(TRB_DATA);
			else /* Control Write */
				field = TRB_TYPE(TRB_DATA);
			
			if(setup->bRequestType & USB_DIR_IN)
				field |= TRB_DIR_IN;

			/* Don't change the cycle bit of the first TRB until later */
			//if(first_trb)
			//{
				//first_trb = false;
				//if(start_cycle == 0)
					//field |= 0x1;
			//}
			//else
				field |= ep_ring->cycle_state;

			/* Chain all the TRBs together; clear the chain bit in the last
			 * TRB to indicate it's the last TRB in the chain.
			 */
			if(num_trbs > 1)
			{
				field |= TRB_CHAIN;
			}
			else
			{
				/* FIXME - add check for ZERO_PACKET flag before this */
				td->last_trb = ep_ring->enqueue;
				field |= TRB_IOC;
			}

			/* Only set interrupt on short packet for IN endpoints */
			if(usb_urb_dir_in(urb))
				field |= TRB_ISP;

			if(TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1)) < trb_buff_len)
			{
				xhci_warn(xhci, "WARN: sg dma xfer crosses 64KB boundaries!\n");
				xhci_dbg(xhci, "Next boundary at %#x, end dma = %#x\n",
					(unsigned int) (addr + TRB_MAX_BUFF_SIZE) & ~(TRB_MAX_BUFF_SIZE - 1),
					(unsigned int) addr + trb_buff_len);
			}

			/* Set the TRB length, TD size, and interrupter fields. */
		   remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

	       length_field = TRB_LEN(trb_buff_len) |
			  TRB_TD_SIZE(remainder) |
			  TRB_INTR_TARGET(0);

			if(num_trbs > 1)
				more_trbs_coming = true;
			else
				more_trbs_coming = false;
			/* Queue data TRBs */
			/*if(first_trb)
			{
				if(setup->bRequestType & USB_DIR_IN)
					field |= TRB_DIR_IN;
				queue_trb(xhci, ep_ring, true, lower_32_bits(addr),
					upper_32_bits(addr), length_field,
					field | TRB_ISP | TRB_TYPE(TRB_DATA));
				pr_info("queue a control trb with field: 0x%x, 0x%x, 0x%x, 0x%x\n",
					lower_32_bits(addr), upper_32_bits(addr), length_field,
					field | TRB_ISP | TRB_TYPE(TRB_DATA));
			}
			else
			{*/
			//	if(setup->bRequestType & USB_DIR_IN)  weitaowang modify
			//		field |= TRB_DIR_IN;
				queue_trb(xhci, ep_ring, true, lower_32_bits(addr),
					upper_32_bits(addr), length_field,
					field  | TRB_TYPE(TRB_NORMAL));
				pr_info("queue a control trb with field: 0x%x, 0x%x, 0x%x, 0x%x\n",
					lower_32_bits(addr), upper_32_bits(addr), length_field,
					field | TRB_ISP | TRB_TYPE(TRB_NORMAL));
			//}
			//queue_trb(xhci, ep_ring, more_trbs_coming, lower_32_bits(addr),
				//upper_32_bits(addr), length_field, field | TRB_TYPE(TRB_NORMAL));
			--num_trbs;
			running_total += trb_buff_len;

			/* Calculate length for next transfer --
			 * Are we done queueing all the TRBs for this sg entry?
			 */
			this_sg_len -= trb_buff_len;
			if(this_sg_len == 0)
			{
				--num_sgs;
				if(num_sgs == 0)
					break;
				sg = sg_next(sg);
				addr = (u64) sg_dma_address(sg);
				this_sg_len = sg_dma_len(sg);
			}
			else
			{
				addr += trb_buff_len;
			}

			trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
			trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
			if(running_total + trb_buff_len > urb->transfer_buffer_length)
				trb_buff_len = urb->transfer_buffer_length - running_total;

			if(first_trb)
				first_trb = false;
				
		}while(running_total < urb->transfer_buffer_length);
		pr_info("Queue Data TRB done\n");
	}

	
	/* Save the DMA address of the last TRB in the TD */
	td->last_trb = ep_ring->enqueue;

	/* Queue status TRB - see Table 7 and sections 4.11.2.2 and 6.4.1.2.3 */
	/* If the device sent data, the status stage is an OUT transfer */
	if(urb->transfer_buffer_length > 0 && setup->bRequestType & USB_DIR_IN)
		field = 0;
	else
		field = TRB_DIR_IN;
	/* Queue status TRB */
	queue_trb(xhci, ep_ring, false, 0, 0, TRB_INTR_TARGET(0),
		/* Event on completion */
		field | TRB_IOC | TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state);

	giveback_first_trb(xhci, slot_id, ep_index, 0, start_cycle, start_trb);
	return 0;
}

/* copy from the standard routine xhci_queue_ctrl_tx and modify it with
  * sg support based on the sg support in xhci_queue_bulk_tx routine
  */
 /*weitaowang: port to linux_4.4 done*/
int xhci_verify_queue_ctrl_sg_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
	int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	int num_trbs;
	int data_num_trbs = 0;
	int ret;
	struct usb_ctrlrequest *setup;
	struct xhci_generic_trb *start_trb;
	int start_cycle;
	u32 field, length_field;
	struct urb_priv *urb_priv;
	struct xhci_td *td;

	int trb_buff_len, this_sg_len, running_total;
	u64 addr;
	struct scatterlist *sg;
	int num_sgs = 0;
	bool first_trb;
	unsigned int total_packet_count;

	pr_info("enter into xhci_verify_queue_ctrl_sg_tx\n");

	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if(!ep_ring)
		return -EINVAL;

	/*
	 * Need to copy setup packet into setup TRB, so we can't use the setup
	 * DMA address.
	 */
	if(!urb->setup_packet)
		return -EINVAL;

	if(!in_interrupt())
		xhci_dbg(xhci, "Queueing ctrl tx for slot id %d, ep %d\n", slot_id, ep_index);
	/* 1 TRB for setup, 1 for status */
	num_trbs = 2;
	/*
	 * Don't need to check if we need additional event data and normal TRBs,
	 * since data in control transfers will never get bigger than 16MB
	 * XXX: can we get a buffer that crosses 64KB boundaries?
	 */
	if(urb->transfer_buffer_length > 0)
	{
		if(urb->num_sgs)
		{
			data_num_trbs = count_sg_trbs_needed(xhci, urb);
			num_trbs += data_num_trbs;
			pr_info("the sg control transfer need %d TRBs\n",data_num_trbs);
				
		}
		else
			num_trbs++;
	}
	ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index, urb->stream_id, num_trbs,
		urb, 0, mem_flags);
	if(ret < 0)
		return ret;

	urb_priv = urb->hcpriv;
	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	/* Queue setup TRB - see section 6.4.1.2.1 */
	/* FIXME better way to translate setup_packet into two u32 fields? */
	setup = (struct usb_ctrlrequest *) urb->setup_packet;
	field = 0;
	field |= TRB_IDT | TRB_TYPE(TRB_SETUP);
	if(start_cycle == 0)
		field |= 0x1;

	/* xhci_verify_mark : support Spec 1.0 for control setup TRB TRT */
	if(xhci->hci_version > 0x96 && setup->wLength != 0)
	{
		if(setup->bRequestType & 0x80)
			field |= CTRL_TRB_TRT(3);
		else
			field |= CTRL_TRB_TRT(2);
	}

	queue_trb(xhci, ep_ring, true,
		/* FIXME endianness is probably going to bite my ass here. */
		setup->bRequestType | setup->bRequest << 8 | setup->wValue << 16,
		setup->wIndex | setup->wLength << 16, TRB_LEN(8) | TRB_INTR_TARGET(0),
		/* Immediate data in pointer */
		field);

	/* Data stage */
	if(!urb->num_sgs)
	{
		/* If there's data, queue data TRBs */
		pr_info("queue normal control trb with length 0x%x\n", urb->transfer_buffer_length);
		field = 0;
		length_field = TRB_LEN(urb->transfer_buffer_length) |
			/*support Spec 1.0 */
			//xhci_td_remainder(urb->transfer_buffer_length, urb->transfer_buffer_length,
			//urb->transfer_buffer_length, xhci, urb) |
			TRB_INTR_TARGET(0);
		if(urb->transfer_buffer_length > 0)
		{
			if(setup->bRequestType & USB_DIR_IN)
				field |= TRB_DIR_IN;
			queue_trb(xhci, ep_ring, true, lower_32_bits(urb->transfer_dma),
				upper_32_bits(urb->transfer_dma), length_field,
				/* Event on short tx */
				field | TRB_ISP | TRB_TYPE(TRB_DATA) | ep_ring->cycle_state);
		}
	}
	else
	{

		running_total = 0;
		total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length, usb_endpoint_maxp(&urb->ep->desc));
		num_sgs = urb->num_sgs;
		sg = urb->sg;
		addr = (u64) sg_dma_address(sg);
		this_sg_len = sg_dma_len(sg);
		pr_info("for the first sg, the addr: 0x%x, 0x%x,the len :0x%x\n",
			upper_32_bits(addr), lower_32_bits(addr), this_sg_len);
		trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
		trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
		if(trb_buff_len > urb->transfer_buffer_length)
			trb_buff_len = urb->transfer_buffer_length;
		first_trb = true;

		pr_info("queue sg control trb with length %d\n",
			urb->transfer_buffer_length);
		do
		{
			u32 field = 0;
			u32 length_field = 0;
			u32 remainder = 0;

			field |= ep_ring->cycle_state;

			/* Chain all the TRBs together; clear the chain bit in the last
			 * TRB to indicate it's the last TRB in the chain.
			 */

			if(data_num_trbs > 1)
			{
				field |= TRB_CHAIN;
			}

			/* Set the TRB length, TD size, and interrupter fields. */
		    remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		   length_field = TRB_LEN(trb_buff_len) |
		    	TRB_TD_SIZE(remainder) |
		    	TRB_INTR_TARGET(0);
			if(first_trb)
			{
				if(setup->bRequestType & USB_DIR_IN)
					field |= TRB_DIR_IN;
				queue_trb(xhci, ep_ring, true, lower_32_bits(addr),
					upper_32_bits(addr), length_field,
					field | TRB_ISP | TRB_TYPE(TRB_DATA));
				pr_info("queue a control trb with field: 0x%x, 0x%x, 0x%x, 0x%x\n",
					lower_32_bits(addr), upper_32_bits(addr), length_field,
					field | TRB_ISP | TRB_TYPE(TRB_DATA));
			}
			else
			{
				queue_trb(xhci, ep_ring, true, lower_32_bits(addr),
					upper_32_bits(addr), length_field,
					field | TRB_ISP | TRB_TYPE(TRB_NORMAL));
				pr_info("queue a control trb with field: 0x%x, 0x%x, 0x%x, 0x%x\n",
					lower_32_bits(addr), upper_32_bits(addr), length_field,
					field | TRB_ISP | TRB_TYPE(TRB_NORMAL));
			}

			--data_num_trbs;
			running_total += trb_buff_len;

			pr_info("left TRB num: 0x%x, running total %x\n",
				data_num_trbs, running_total);

			/* Calculate length for next transfer --
			 * Are we done queueing all the TRBs for this sg entry?
			 */

			this_sg_len -= trb_buff_len;
			if(this_sg_len == 0)
			{
				--num_sgs;
				if(num_sgs == 0)
					break;
				sg = sg_next(sg);
				addr = (u64) sg_dma_address(sg);
				this_sg_len = sg_dma_len(sg);
				pr_info("left sg num %d, for the cur sg, the addr: 0x%x, 0x%x,the len :0x%x\n",
					num_sgs, upper_32_bits(addr), lower_32_bits(addr), this_sg_len);
			}
			else
			{
				addr += trb_buff_len;
			}

			trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
			trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
			if(running_total + trb_buff_len > urb->transfer_buffer_length)
				trb_buff_len = urb->transfer_buffer_length - running_total;

			if(first_trb)
				first_trb = false;
		}while(running_total < urb->transfer_buffer_length);

	}

	/* Save the DMA address of the last TRB in the TD */
	td->last_trb = ep_ring->enqueue;

	/* Queue status TRB - see Table 7 and sections 4.11.2.2 and 6.4.1.2.3 */
	/* If the device sent data, the status stage is an OUT transfer */
	if(urb->transfer_buffer_length > 0 && setup->bRequestType & USB_DIR_IN)
		field = 0;
	else
		field = TRB_DIR_IN;
	queue_trb(xhci, ep_ring, false, 0, 0, TRB_INTR_TARGET(0),
		/* Event on completion */
		field | TRB_IOC | TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state);

	giveback_first_trb(xhci, slot_id, ep_index, 0, start_cycle, start_trb);
	pr_info("leave xhci_verify_queue_ctrl_sg_tx, return 0\n");
	return 0;
}

void isoc_giveback_first_trb(struct xhci_hcd *xhci, int slot_id, unsigned int ep_index,
	unsigned int stream_id, int start_cycle, struct xhci_generic_trb *start_trb,
	bool doorbell)
{
	/*
	 * Pass all the TRBs to the hardware at once and make sure this write
	 * isn't reordered.
	 */
	wmb();
	if(start_cycle)
		start_trb->field[3] |= start_cycle;
	else
		start_trb->field[3] &= ~0x1;

	if(doorbell)
		xhci_ring_ep_doorbell(xhci, slot_id, ep_index, stream_id);
}
/* copy from xhci_queue_isoc_tx in xhci-ring.c and modify it to support
  * underrun/overrun verify and sg transfer.
  * This is for isoc transfer */
 /*weitaowang: port to linux_4.4 done*/
static int xhci_verify_queue_isoc_tx(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	int num_tds, trbs_per_td;
	struct xhci_generic_trb *start_trb;
	bool first_trb;
	int start_cycle;
	u32 field, length_field;
	int running_total, trb_buff_len, td_len, td_remain_len, ret;
	u64 start_addr, addr;
	int i, j;
	bool more_trbs_coming;
	struct xhci_virt_ep *xep;

	/*xhci_verify_mark : support Spec 1.0 for isoch TBC and TLPBC */
	unsigned int tdpc;
	unsigned int max_burst_size;
	unsigned int max_packet_size;
	unsigned int isochBurstResiduePackets;
	unsigned int tbc, tlbpc;

	pr_info("enter into xhci_verify_queue_iso_tx\n");
	switch (urb->dev->speed)
	{
	case USB_SPEED_SUPER:
		max_packet_size = (urb->ep->desc.wMaxPacketSize & 0xFFFF);
		max_burst_size = (urb->ep->ss_ep_comp.bMaxBurst & 0xFF) + 1;
		break;
	case USB_SPEED_HIGH:
		/* bits 11:12 specify the number of additional transaction
		 * opportunities per microframe (USB 2.0, section 9.6.6)
		 */
		max_burst_size = (urb->ep->desc.wMaxPacketSize & 0x1800) >> 11;
		max_packet_size = GET_MAX_PACKET(urb->ep->desc.wMaxPacketSize);
		break;
	case USB_SPEED_FULL:
	case USB_SPEED_LOW:
		max_packet_size = GET_MAX_PACKET(urb->ep->desc.wMaxPacketSize);
		max_burst_size = 1;;
		break;
	default:
		xhci_warn(xhci, "the speed is not supported\n");
		max_packet_size = 1024;
		max_burst_size = 1;
		break;
	}
	
    xep = &xhci->devs[slot_id]->eps[ep_index];
	ep_ring = xhci->devs[slot_id]->eps[ep_index].ring;

	num_tds = urb->number_of_packets;
	if(num_tds < 1)
	{
		xhci_dbg(xhci, "Isoc URB with zero packets?\n");
		return -EINVAL;
	}

	if(!in_interrupt())
		xhci_dbg(xhci, "ep %#x - urb len = %#x (%d),"
			" addr = %#llx, num_tds = %d\n", urb->ep->desc.bEndpointAddress,
			urb->transfer_buffer_length, urb->transfer_buffer_length,
			(unsigned long long) urb->transfer_dma, num_tds);

	start_addr = (u64) urb->transfer_dma;
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	/* Queue the first TRB, even if it's zero-length */
	for(i = 0; i < num_tds; i++)
	{
		first_trb = true;

		running_total = 0;
		addr = start_addr + urb->iso_frame_desc[i].offset;
		td_len = urb->iso_frame_desc[i].length;
		td_remain_len = td_len;

		trbs_per_td = count_isoc_trbs_needed(xhci, urb, i);

		ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index, urb->stream_id,
			trbs_per_td, urb, i, mem_flags);
		if(ret < 0)
			return ret;

		urb_priv = urb->hcpriv;
		td = urb_priv->td[i];

		for(j = 0; j < trbs_per_td; j++)
		{
			u32 remainder = 0;
			field = 0;

			if(first_trb)
			{
				/* Queue the isoc TRB */
				field |= TRB_TYPE(TRB_ISOC);
				/* Assume URB_ISO_ASAP is set */
				field |= TRB_SIA;
				if(i == 0)
				{
					if(start_cycle == 0)
						field |= 0x1;
				}
				else
					field |= ep_ring->cycle_state;
				first_trb = false;

				/* xhci_verify_mark : support Spec 1.0 for isoch TBC and TLPBC */
				tdpc = (td_len + max_packet_size - 1) / max_packet_size;
				tbc = (tdpc + max_burst_size - 1) / max_burst_size;
				tbc -= 1;
				isochBurstResiduePackets = tdpc % max_packet_size;
				if(isochBurstResiduePackets == 0)
					tlbpc = max_burst_size - 1;
				else
					tlbpc = isochBurstResiduePackets - 1;
				if(xhci->hci_version > 0x96)
					field |= (ISOC_TRB_TBC(tbc) | ISOC_TRB_TLBPC(tlbpc));
			}
			else
			{
				/* Queue other normal TRBs */
				field |= TRB_TYPE(TRB_NORMAL);
				field |= ep_ring->cycle_state;
			}

			/* Chain all the TRBs together; clear the chain bit in
			 * the last TRB to indicate it's the last TRB in the
			 * chain.
			 */
			if(j < trbs_per_td - 1)
			{
				field |= TRB_CHAIN;
				more_trbs_coming = true;
			}
			else
			{
				td->last_trb = ep_ring->enqueue;
				field |= TRB_IOC;
				more_trbs_coming = false;
			}

			/* Calculate TRB length */
			trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & ((1 << TRB_MAX_BUFF_SHIFT) - 1));
			if(trb_buff_len > td_remain_len)
				trb_buff_len = td_remain_len;

			//remainder = xhci_td_remainder(td_len - running_total, trb_buff_len,
			//td_len, xhci, urb);
			length_field = TRB_LEN(trb_buff_len) | remainder | TRB_INTR_TARGET(0);
			queue_trb(xhci, ep_ring, more_trbs_coming, lower_32_bits(addr),
				upper_32_bits(addr), length_field,
				/* We always want to know if the TRB was short,
				 * or we won't get an event when it completes.
				 * (Unless we use event data TRBs, which are a
				 * waste of space and HC resources.)
				 */
				field | TRB_ISP);
			running_total += trb_buff_len;

			addr += trb_buff_len;
			td_remain_len -= trb_buff_len;
		}

		/* Check TD length */
		if(running_total != td_len)
		{
			xhci_err(xhci, "ISOC TD length unmatch\n");
			return -EINVAL;
		}
	}

	if(urb->num_sgs > 0)
		isoc_giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id, start_cycle,
			start_trb, true);
	else
		isoc_giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id, start_cycle,
			start_trb, false);
	return 0;
}

/*
 * copy from xhci-ring.c and modify it to support underrun/overrun verify and sg transfer
 * Check transfer ring to guarantee there is enough room for the urb.
 * Update ISO URB start_frame and interval.
 * Update interval as xhci_queue_intr_tx does. Just use xhci frame_index to
 * update the urb->start_frame by now.
 * Always assume URB_ISO_ASAP set, and NEVER use urb->start_frame as input.
 */
 /*weitaowang: port to linux_4.4 done NOTICE: the frame issues.*/
int xhci_verify_queue_isoc_tx_prepare(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)

{
	struct xhci_virt_device *xdev;
	struct xhci_ring *ep_ring;
	struct xhci_ep_ctx *ep_ctx;
	int start_frame;
	int xhci_interval;
	int ep_interval;
	int num_tds, num_trbs, i;
	int ret;
	struct xhci_virt_ep *xep;
	int ist;

	xdev = xhci->devs[slot_id];
	xep = &xhci->devs[slot_id]->eps[ep_index];
	ep_ring = xdev->eps[ep_index].ring;
	ep_ctx = xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);

	num_trbs = 0;
	num_tds = urb->number_of_packets;
	for (i = 0; i < num_tds; i++)
		num_trbs += count_isoc_trbs_needed(xhci, urb, i);

	/* Check the ring to guarantee there is enough room for the whole urb.
	 * Do not insert any td of the urb to the ring if the check failed.
	 */
	ret = prepare_ring(xhci, ep_ring, le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK,
			   num_trbs, mem_flags);
	if (ret)
		return ret;

	/*
	 * Check interval value. This should be done before we start to
	 * calculate the start frame value.
	 */
	xhci_interval = EP_INTERVAL_TO_UFRAMES(le32_to_cpu(ep_ctx->ep_info));
	ep_interval = urb->interval;
	/* Convert to microframes */
	if (urb->dev->speed == USB_SPEED_LOW ||
			urb->dev->speed == USB_SPEED_FULL)
		ep_interval *= 8;
	/* FIXME change this to a warning and a suggestion to use the new API
	 * to set the polling interval (once the API is added).
	 */
	if (xhci_interval != ep_interval) {
		dev_dbg_ratelimited(&urb->dev->dev,
				"Driver uses different interval (%d microframe%s) than xHCI (%d microframe%s)\n",
				ep_interval, ep_interval == 1 ? "" : "s",
				xhci_interval, xhci_interval == 1 ? "" : "s");
		urb->interval = xhci_interval;
		/* Convert back to frames for LS/FS devices */
		if (urb->dev->speed == USB_SPEED_LOW ||
				urb->dev->speed == USB_SPEED_FULL)
			urb->interval /= 8;
	}

	/* Calculate the start frame and put it in urb->start_frame. */
	if (HCC_CFC(xhci->hcc_params) &&
			ep_ring_is_processing(xhci, slot_id, ep_index)) {
		urb->start_frame = xep->next_frame_id;
		goto skip_start_over;
	}

	start_frame = readl(&xhci->run_regs->microframe_index);
	start_frame &= 0x3fff;
	/*
	 * Round up to the next frame and consider the time before trb really
	 * gets scheduled by hardare.
	 */
	ist = HCS_IST(xhci->hcs_params2) & 0x7;
	if (HCS_IST(xhci->hcs_params2) & (1 << 3))
		ist <<= 3;
	start_frame += ist + XHCI_CFC_DELAY;
	start_frame = roundup(start_frame, 8);

	/*
	 * Round up to the next ESIT (Endpoint Service Interval Time) if ESIT
	 * is greate than 8 microframes.
	 */
	if (urb->dev->speed == USB_SPEED_LOW ||
			urb->dev->speed == USB_SPEED_FULL) {
		start_frame = roundup(start_frame, urb->interval << 3);
		urb->start_frame = start_frame >> 3;
	} else {
		start_frame = roundup(start_frame, urb->interval);
		urb->start_frame = start_frame;
	}

skip_start_over:
	return xhci_verify_queue_isoc_tx(xhci, GFP_ATOMIC, urb, slot_id, ep_index);
	
}
/*weitaowang add:xhci_verify_mark end*/

static int queue_bulk_sg_tx(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	unsigned int num_trbs;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	struct scatterlist *sg;
	int num_sgs;
	int trb_buff_len, this_sg_len, running_total, ret;
	unsigned int total_packet_count;
	bool zero_length_needed;
	bool first_trb;
	int last_trb_num;
	u64 addr;
	bool more_trbs_coming;

	struct xhci_generic_trb *start_trb;
	int start_cycle;

	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if (!ep_ring)
		return -EINVAL;

	num_trbs = count_sg_trbs_needed(xhci, urb);
	num_sgs = urb->num_mapped_sgs;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length,
			usb_endpoint_maxp(&urb->ep->desc));

	ret = prepare_transfer(xhci, xhci->devs[slot_id],
			ep_index, urb->stream_id,
			num_trbs, urb, 0, mem_flags);
	if (ret < 0)
		return ret;

	urb_priv = urb->hcpriv;

	/* Deal with URB_ZERO_PACKET - need one more td/trb */
	zero_length_needed = urb->transfer_flags & URB_ZERO_PACKET &&
		urb_priv->length == 2;
	if (zero_length_needed) {
		num_trbs++;
		xhci_dbg(xhci, "Creating zero length td.\n");
		ret = prepare_transfer(xhci, xhci->devs[slot_id],
				ep_index, urb->stream_id,
				1, urb, 1, mem_flags);
		if (ret < 0)
			return ret;
	}

	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	running_total = 0;
	/*
	 * How much data is in the first TRB?
	 *
	 * There are three forces at work for TRB buffer pointers and lengths:
	 * 1. We don't want to walk off the end of this sg-list entry buffer.
	 * 2. The transfer length that the driver requested may be smaller than
	 *    the amount of memory allocated for this scatter-gather list.
	 * 3. TRBs buffers can't cross 64KB boundaries.
	 */
	sg = urb->sg;
	addr = (u64) sg_dma_address(sg);
	this_sg_len = sg_dma_len(sg);
	trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
	trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
	if (trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;

	first_trb = true;
	last_trb_num = zero_length_needed ? 2 : 1;
	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 field = 0;
		u32 length_field = 0;
		u32 remainder = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= 0x1;
		} else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > last_trb_num) {
			field |= TRB_CHAIN;
		} else if (num_trbs == last_trb_num) {
			td->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		} else if (zero_length_needed && num_trbs == 1) {
			trb_buff_len = 0;
			urb_priv->td[1]->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_urb_dir_in(urb))
			field |= TRB_ISP;

		if (TRB_MAX_BUFF_SIZE -
				(addr & (TRB_MAX_BUFF_SIZE - 1)) < trb_buff_len) {
			xhci_warn(xhci, "WARN: sg dma xfer crosses 64KB boundaries!\n");
			xhci_dbg(xhci, "Next boundary at %#x, end dma = %#x\n",
					(unsigned int) (addr + TRB_MAX_BUFF_SIZE) & ~(TRB_MAX_BUFF_SIZE - 1),
					(unsigned int) addr + trb_buff_len);
		}

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if (num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;
		queue_trb(xhci, ep_ring, more_trbs_coming,
				lower_32_bits(addr),
				upper_32_bits(addr),
				length_field,
				field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer --
		 * Are we done queueing all the TRBs for this sg entry?
		 */
		this_sg_len -= trb_buff_len;
		if (this_sg_len == 0) {
			--num_sgs;
			if (num_sgs == 0)
				break;
			sg = sg_next(sg);
			addr = (u64) sg_dma_address(sg);
			this_sg_len = sg_dma_len(sg);
		} else {
			addr += trb_buff_len;
		}

		trb_buff_len = TRB_MAX_BUFF_SIZE -
			(addr & (TRB_MAX_BUFF_SIZE - 1));
		trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
		if (running_total + trb_buff_len > urb->transfer_buffer_length)
			trb_buff_len =
				urb->transfer_buffer_length - running_total;
	} while (num_trbs > 0);

	check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id,
			start_cycle, start_trb);
	return 0;
}

/*weitaowang add:xhci_verify_mark start*/
/*weitaowang: port to linux_4.4 done*/
int queue_bulk_sg_tx_with_little_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	unsigned int num_trbs;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	struct scatterlist *sg;
	int num_sgs;
	int trb_buff_len, this_sg_len, running_total, ret;
	unsigned int total_packet_count;
	bool zero_length_needed;
	bool first_trb;
	int last_trb_num;
	u64 addr;
	bool more_trbs_coming;

	struct xhci_generic_trb *start_trb;
	int start_cycle;

	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if (!ep_ring)
		return -EINVAL;

	//num_trbs = count_sg_trbs_needed(xhci, urb);
	// for every sglist, the buffer must be 4K align, and we will
	// queue the TRB with 1K length
	num_trbs = (urb->transfer_buffer_length + 1024 - 1) / 1024;
	pr_info("the num_trbs needed is %d\n", num_trbs);
		
	num_sgs = urb->num_mapped_sgs;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length,
			usb_endpoint_maxp(&urb->ep->desc));

	ret = prepare_transfer(xhci, xhci->devs[slot_id],
			ep_index, urb->stream_id,
			num_trbs, urb, 0, mem_flags);
	if (ret < 0)
		return ret;

	urb_priv = urb->hcpriv;

	/* Deal with URB_ZERO_PACKET - need one more td/trb */
	zero_length_needed = urb->transfer_flags & URB_ZERO_PACKET &&
		urb_priv->length == 2;
	if (zero_length_needed) {
		num_trbs++;
		xhci_dbg(xhci, "Creating zero length td.\n");
		ret = prepare_transfer(xhci, xhci->devs[slot_id],
				ep_index, urb->stream_id,
				1, urb, 1, mem_flags);
		if (ret < 0)
			return ret;
	}

	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	running_total = 0;
	/*
	 * How much data is in the first TRB?
	 *
	 * There are three forces at work for TRB buffer pointers and lengths:
	 * 1. We don't want to walk off the end of this sg-list entry buffer.
	 * 2. The transfer length that the driver requested may be smaller than
	 *    the amount of memory allocated for this scatter-gather list.
	 * 3. TRBs buffers can't cross 64KB boundaries.
	 */
	sg = urb->sg;
	addr = (u64) sg_dma_address(sg);
	this_sg_len = sg_dma_len(sg);
	trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
	trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
	if (trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;

	// reset trb_buff_len
	if(trb_buff_len > 1024)
		trb_buff_len = 1024;
	
	first_trb = true;
	last_trb_num = zero_length_needed ? 2 : 1;
	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 field = 0;
		u32 length_field = 0;
		u32 remainder = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= 0x1;
		} else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > last_trb_num) {
			field |= TRB_CHAIN;
			//set field of all to TRB IOC and Block Event Interrupt 
			field |= (TRB_IOC | TRB_BEI);
		} else if (num_trbs == last_trb_num) {
			td->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		} else if (zero_length_needed && num_trbs == 1) {
			trb_buff_len = 0;
			urb_priv->td[1]->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_urb_dir_in(urb))
			field |= TRB_ISP;

		if (TRB_MAX_BUFF_SIZE -
				(addr & (TRB_MAX_BUFF_SIZE - 1)) < trb_buff_len) {
			xhci_warn(xhci, "WARN: sg dma xfer crosses 64KB boundaries!\n");
			xhci_dbg(xhci, "Next boundary at %#x, end dma = %#x\n",
					(unsigned int) (addr + TRB_MAX_BUFF_SIZE) & ~(TRB_MAX_BUFF_SIZE - 1),
					(unsigned int) addr + trb_buff_len);
		}

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if (num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;
		queue_trb(xhci, ep_ring, more_trbs_coming,
				lower_32_bits(addr),
				upper_32_bits(addr),
				length_field,
				field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer --
		 * Are we done queueing all the TRBs for this sg entry?
		 */
		this_sg_len -= trb_buff_len;
		if (this_sg_len == 0) {
			--num_sgs;
			if (num_sgs == 0)
				break;
			sg = sg_next(sg);
			addr = (u64) sg_dma_address(sg);
			this_sg_len = sg_dma_len(sg);
		} else {
			addr += trb_buff_len;
		}

		trb_buff_len = TRB_MAX_BUFF_SIZE -
			(addr & (TRB_MAX_BUFF_SIZE - 1));
		trb_buff_len = min_t(int, trb_buff_len, this_sg_len);

		//reset trb_buff_len
		if(trb_buff_len > 1024)
			trb_buff_len = 1024;
		
		if (running_total + trb_buff_len > urb->transfer_buffer_length)
			trb_buff_len =
				urb->transfer_buffer_length - running_total;
	} while (num_trbs > 0);

	//check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id,
			start_cycle, start_trb);
	return 0;
}

/*
 * xHCI 1.0 Spec -- 4.9.1
 * To generate a "zero-length"USB transaction, software shall explicitly define 
 * a TD with a single Transfer TRB, and its TRB Transfer Length field shall equal 0.
 * Note that this TD may include non-Transfer TRBs, e.g. an Event Data or Link TRB.
 *
 * Jeff: So here we contruct a TD with one zero-length TRB.
 */
 /*weitaowang: port to linux_4.4 done need to remodify*/
int queue_bulk_sg_tx_zero_len_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	unsigned int num_trbs;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	struct scatterlist *sg;
	int num_sgs;
	int trb_buff_len, this_sg_len, running_total, ret;
	unsigned int total_packet_count;
	bool zero_length_needed;
	bool first_trb;
	int last_trb_num;
	u64 addr;
	bool more_trbs_coming;

		//zero_len_trb
	u32 field1 = 0;
		
	struct xhci_generic_trb *start_trb;
	int start_cycle;
	
	pr_info("enter into queue_bulk_sg_tx_zero_len_trb\n");
	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if (!ep_ring)
		return -EINVAL;

	//num_trbs = count_sg_trbs_needed(xhci, urb);
	//one trb for zero length
	num_trbs = 1;
	
	num_sgs = urb->num_mapped_sgs;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length,
			usb_endpoint_maxp(&urb->ep->desc));
	pr_info("total packet count is %d\n", total_packet_count);
	total_packet_count = 0;
	
	ret = prepare_transfer(xhci, xhci->devs[slot_id],
			ep_index, urb->stream_id,
			num_trbs, urb, 0, mem_flags);
	if (ret < 0)
		return ret;

	urb_priv = urb->hcpriv;

	/* Deal with URB_ZERO_PACKET - need one more td/trb */
	zero_length_needed = urb->transfer_flags & URB_ZERO_PACKET &&
		urb_priv->length == 2;
	if (zero_length_needed) {
		num_trbs++;
		xhci_dbg(xhci, "Creating zero length td.\n");
		ret = prepare_transfer(xhci, xhci->devs[slot_id],
				ep_index, urb->stream_id,
				1, urb, 1, mem_flags);
		if (ret < 0)
			return ret;
	}

	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	if(start_cycle == 0)
		field1 |= 0x1;

	field1 |= TRB_IOC | TRB_ISP | TRB_TYPE(TRB_NORMAL);
	td->last_trb = ep_ring->enqueue;
	pr_info("queue zero_length_trb\n");
	queue_trb(xhci, ep_ring, false, 0, 0, 0, field1);
	goto giveback;
	
	running_total = 0;
	/*
	 * How much data is in the first TRB?
	 *
	 * There are three forces at work for TRB buffer pointers and lengths:
	 * 1. We don't want to walk off the end of this sg-list entry buffer.
	 * 2. The transfer length that the driver requested may be smaller than
	 *    the amount of memory allocated for this scatter-gather list.
	 * 3. TRBs buffers can't cross 64KB boundaries.
	 */
	sg = urb->sg;
	addr = (u64) sg_dma_address(sg);
	this_sg_len = sg_dma_len(sg);
	trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
	trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
	if (trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;

	first_trb = true;
	last_trb_num = zero_length_needed ? 2 : 1;
	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 field = 0;
		u32 length_field = 0;
		u32 remainder = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= 0x1;
		} else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > last_trb_num) {
			field |= TRB_CHAIN;
		} else if (num_trbs == last_trb_num) {
			//td->last_trb = ep_ring->enqueue;
			//field |= TRB_IOC;
		} else if (zero_length_needed && num_trbs == 1) {
			trb_buff_len = 0;
			urb_priv->td[1]->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_urb_dir_in(urb))
			field |= TRB_ISP;

		if (TRB_MAX_BUFF_SIZE -
				(addr & (TRB_MAX_BUFF_SIZE - 1)) < trb_buff_len) {
			xhci_warn(xhci, "WARN: sg dma xfer crosses 64KB boundaries!\n");
			xhci_dbg(xhci, "Next boundary at %#x, end dma = %#x\n",
					(unsigned int) (addr + TRB_MAX_BUFF_SIZE) & ~(TRB_MAX_BUFF_SIZE - 1),
					(unsigned int) addr + trb_buff_len);
		}

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if (num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;
		queue_trb(xhci, ep_ring, more_trbs_coming,
				lower_32_bits(addr),
				upper_32_bits(addr),
				length_field,
				field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer --
		 * Are we done queueing all the TRBs for this sg entry?
		 */
		this_sg_len -= trb_buff_len;
		if (this_sg_len == 0) {
			--num_sgs;
			if (num_sgs == 0)
				break;
			sg = sg_next(sg);
			addr = (u64) sg_dma_address(sg);
			this_sg_len = sg_dma_len(sg);
		} else {
			addr += trb_buff_len;
		}

		trb_buff_len = TRB_MAX_BUFF_SIZE -
			(addr & (TRB_MAX_BUFF_SIZE - 1));
		trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
		if (running_total + trb_buff_len > urb->transfer_buffer_length)
			trb_buff_len =
				urb->transfer_buffer_length - running_total;
	} while(running_total < urb->transfer_buffer_length);
	
	
	// queue the zero length TRB now
		td->last_trb = ep_ring->enqueue;
		field1 = 0;
		field1 |= TRB_IOC;
		field1 |= ep_ring->cycle_state;
		queue_trb(xhci, ep_ring, false, 0, 0, 0, field1 | TRB_ISP | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		pr_info("queue_zero_len_trb submitted\n");

giveback:
	//check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id,
			start_cycle, start_trb);
	return 0;
}
/*weitaowang add:xhci_verify_mark end*/

/* This is very similar to what ehci-q.c qtd_fill() does */
int xhci_queue_bulk_tx(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	int num_trbs;
	struct xhci_generic_trb *start_trb;
	bool first_trb;
	int last_trb_num;
	bool more_trbs_coming;
	bool zero_length_needed;
	int start_cycle;
	u32 field, length_field;

	int running_total, trb_buff_len, ret;
	unsigned int total_packet_count;
	u64 addr;

	if (urb->num_sgs)
		return queue_bulk_sg_tx(xhci, mem_flags, urb, slot_id, ep_index);

	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if (!ep_ring)
		return -EINVAL;

	num_trbs = 0;
	/* How much data is (potentially) left before the 64KB boundary? */
	running_total = TRB_MAX_BUFF_SIZE -
		(urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	/* If there's some data on this 64KB chunk, or we have to send a
	 * zero-length transfer, we need at least one TRB
	 */
	if (running_total != 0 || urb->transfer_buffer_length == 0)
		num_trbs++;
	/* How many more 64KB chunks to transfer, how many more TRBs? */
	while (running_total < urb->transfer_buffer_length) {
		num_trbs++;
		running_total += TRB_MAX_BUFF_SIZE;
	}

	ret = prepare_transfer(xhci, xhci->devs[slot_id],
			ep_index, urb->stream_id,
			num_trbs, urb, 0, mem_flags);
	if (ret < 0)
		return ret;

	urb_priv = urb->hcpriv;

	/* Deal with URB_ZERO_PACKET - need one more td/trb */
	zero_length_needed = urb->transfer_flags & URB_ZERO_PACKET &&
		urb_priv->length == 2;
	if (zero_length_needed) {
		num_trbs++;
		xhci_dbg(xhci, "Creating zero length td.\n");
		ret = prepare_transfer(xhci, xhci->devs[slot_id],
				ep_index, urb->stream_id,
				1, urb, 1, mem_flags);
		if (ret < 0)
			return ret;
	}

	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	running_total = 0;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length,
			usb_endpoint_maxp(&urb->ep->desc));
	/* How much data is in the first TRB? */
	addr = (u64) urb->transfer_dma;
	trb_buff_len = TRB_MAX_BUFF_SIZE -
		(urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	if (trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;

	first_trb = true;
	last_trb_num = zero_length_needed ? 2 : 1;
	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 remainder = 0;
		field = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= 0x1;
		} else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > last_trb_num) {
			field |= TRB_CHAIN;
		} else if (num_trbs == last_trb_num) {
			td->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		} else if (zero_length_needed && num_trbs == 1) {
			trb_buff_len = 0;
			urb_priv->td[1]->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_urb_dir_in(urb))
			field |= TRB_ISP;

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if (num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;
		queue_trb(xhci, ep_ring, more_trbs_coming,
				lower_32_bits(addr),
				upper_32_bits(addr),
				length_field,
				field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = urb->transfer_buffer_length - running_total;
		if (trb_buff_len > TRB_MAX_BUFF_SIZE)
			trb_buff_len = TRB_MAX_BUFF_SIZE;
	} while (num_trbs > 0);

	check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id,
			start_cycle, start_trb);
	return 0;
}

/*weitaowang add:xhci_verify_mark start*/
/*weitaowang: port to linux_4.4 done some redundant items need to modify*/
int xhci_verify_queue_bulk_sg_tx(struct xhci_hcd *xhci, gfp_t mem_flags, struct urb *urb,
	int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	unsigned int num_trbs, total_num_trbs;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	struct scatterlist *sg;
	int num_sgs;
	int trb_buff_len, this_sg_len, running_total;
	unsigned int total_packet_count;
	bool first_trb;
	u64 addr;
	u64 event_trb_dma_addr;
	bool more_trbs_coming;

	int left;

	struct xhci_generic_trb *start_trb;
	int start_cycle;

	u32 field = 0;

	/* Get Right Ring */
	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if(!ep_ring)
		return -EINVAL;

	//num_trbs = count_sg_trbs_needed(xhci, urb);
	/* there is only one sglist and the buffer must be 4K align, and we will	  
	* queue the TRB with 1K length	  
	*/	
	num_trbs = (urb->transfer_buffer_length+1024-1)/1024;	
	total_num_trbs = num_trbs;	
	num_trbs += 2;
	
	num_sgs = urb->num_mapped_sgs;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length, usb_endpoint_maxp(&urb->ep->desc));

	trb_buff_len = prepare_transfer(xhci, xhci->devs[slot_id], ep_index, 
						urb->stream_id, num_trbs, urb, 0, mem_flags);
		
	if(trb_buff_len < 0)
		return trb_buff_len;

	urb_priv = urb->hcpriv;
	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	running_total = 0;
	/*
	 * How much data is in the first TRB?
	 *
	 * There are three forces at work for TRB buffer pointers and lengths:
	 * 1. We don't want to walk off the end of this sg-list entry buffer.
	 * 2. The transfer length that the driver requested may be smaller than
	 *    the amount of memory allocated for this scatter-gather list.
	 * 3. TRBs buffers can't cross 64KB boundaries.
	 */
	 sg = urb->sg;
	addr = (u64)sg_dma_address(sg);
	this_sg_len = sg_dma_len(sg);
	trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
	trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
	if(trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;
	
	//reset trb_buff_len
	trb_buff_len = 1024;
	if (trb_buff_len > urb->transfer_buffer_length)		
		trb_buff_len = urb->transfer_buffer_length;	
	
	pr_info("First length to xfer from 1st sglist entry = %u\n", trb_buff_len);

	first_trb = true;
	/* Queue the first TRB, even if it's zero-length */
	do
	{
		u32 field = 0;
		u32 length_field = 0;
		u32 remainder = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if(first_trb)
		{
			first_trb = false;
			if(start_cycle == 0)
				field |= 0x1;
		}
		else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if(num_trbs > 1)
		{
			field |= TRB_CHAIN;
		}
		else
		{
			/* FIXME - add check for ZERO_PACKET flag before this */
			//td->last_trb = ep_ring->enqueue;
			//field |= TRB_IOC;
		}
		pr_info(" sg entry: dma = %#x, len = %#x (%d), "				
			"64KB boundary at %#x, end dma = %#x\n",				
			(unsigned int) addr, trb_buff_len, trb_buff_len,				
			(unsigned int) (addr + TRB_MAX_BUFF_SIZE) & ~(TRB_MAX_BUFF_SIZE - 1),				
			(unsigned int) addr + trb_buff_len);

		/* Only set interrupt on short packet for IN endpoints */
		//if(usb_urb_dir_in(urb))
			//field |= TRB_ISP;

		if(TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1)) < trb_buff_len)
		{
			xhci_warn(xhci, "WARN: sg dma xfer crosses 64KB boundaries!\n");
			xhci_dbg(xhci, "Next boundary at %#x, end dma = %#x\n",
				(unsigned int) (addr + TRB_MAX_BUFF_SIZE) & ~(TRB_MAX_BUFF_SIZE - 1),
				(unsigned int) addr + trb_buff_len);
		}

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if(num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;
		queue_trb(xhci, ep_ring, more_trbs_coming, lower_32_bits(addr),
			upper_32_bits(addr), length_field, field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer --
		 * Are we done queueing all the TRBs for this sg entry?
		 */
		this_sg_len -= trb_buff_len;
		if(this_sg_len == 0)
		{
			--num_sgs;
			if(num_sgs == 0)
				break;
			sg = sg_next(sg);
			addr = (u64) sg_dma_address(sg);
			this_sg_len = sg_dma_len(sg);
		}
		else
		{
			addr += trb_buff_len;
		}

		trb_buff_len = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
		trb_buff_len = min_t(int, trb_buff_len, this_sg_len);
		//reset trb_buff_len
		trb_buff_len = 1024;
		if(running_total + trb_buff_len > urb->transfer_buffer_length)
			trb_buff_len = urb->transfer_buffer_length - running_total;
		
		if (num_trbs == (total_num_trbs >> 1)) 
		{		
			field = 0;			
			field |= ep_ring->cycle_state;			
			field |= TRB_IOC | TRB_CHAIN;			
			event_trb_dma_addr = ep_ring->enq_seg->dma +				
				(sizeof(struct xhci_generic_trb) * (ep_ring->enqueue - ep_ring->enq_seg->trbs));
			
			queue_trb(xhci, ep_ring, true,				
				lower_32_bits(event_trb_dma_addr),				
				upper_32_bits(event_trb_dma_addr),				
				TRB_INTR_TARGET(0),				
				field | TRB_TYPE(TRB_EVENT_DATA));		
			
			--num_trbs;			
			left = urb->transfer_buffer_length - running_total;		
		}
	}while(running_total < urb->transfer_buffer_length);

	td->last_trb = ep_ring->enqueue;	
	--num_trbs;	
	event_trb_dma_addr = ep_ring->enq_seg->dma +		
		(sizeof(struct xhci_generic_trb) * (ep_ring->enqueue - ep_ring->enq_seg->trbs));	
	field = 0;	
	field |= ep_ring->cycle_state;	
	field |= TRB_IOC;		
	queue_trb(xhci, ep_ring, false,		
		lower_32_bits(event_trb_dma_addr),		
		upper_32_bits(event_trb_dma_addr),		
		TRB_INTR_TARGET(0),		
		field | TRB_TYPE(TRB_EVENT_DATA));

	//check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id, start_cycle, start_trb);
	return 0;
}

/* This is very similar to what ehci-q.c qtd_fill() does */
/*weitaowang: port to linux_4.4 done some redundant items need to modify*/
int xhci_queue_bulk_tx_with_little_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	int num_trbs;
	struct xhci_generic_trb *start_trb;
	bool first_trb;
	int last_trb_num;
	bool more_trbs_coming;
	bool zero_length_needed;
	int start_cycle;
	u32 field, length_field;

	int running_total, trb_buff_len, ret;
	unsigned int total_packet_count;
	u64 addr;

	pr_info("enter into xhci_queue_bulk_tx_with_little_trb\n");
	if(urb->num_sgs)
		return queue_bulk_sg_tx_with_little_trb(xhci, mem_flags, urb, slot_id, ep_index);

	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if (!ep_ring)
		return -EINVAL;

	num_trbs = 0;
	/* How much data is (potentially) left before the 64KB boundary? */
	running_total = TRB_MAX_BUFF_SIZE -
		(urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	/* If there's some data on this 64KB chunk, or we have to send a
	 * zero-length transfer, we need at least one TRB
	 */
	if (running_total != 0 || urb->transfer_buffer_length == 0)
		num_trbs++;
	/* How many more 64KB chunks to transfer, how many more TRBs? */
	while (running_total < urb->transfer_buffer_length) {
		num_trbs++;
		running_total += TRB_MAX_BUFF_SIZE;
	}

	ret = prepare_transfer(xhci, xhci->devs[slot_id],
			ep_index, urb->stream_id,
			num_trbs, urb, 0, mem_flags);
	if (ret < 0)
		return ret;

	urb_priv = urb->hcpriv;

	/* Deal with URB_ZERO_PACKET - need one more td/trb */
	zero_length_needed = urb->transfer_flags & URB_ZERO_PACKET &&
		urb_priv->length == 2;
	if (zero_length_needed) {
		num_trbs++;
		xhci_dbg(xhci, "Creating zero length td.\n");
		ret = prepare_transfer(xhci, xhci->devs[slot_id],
				ep_index, urb->stream_id,
				1, urb, 1, mem_flags);
		if (ret < 0)
			return ret;
	}

	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	running_total = 0;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length,
			usb_endpoint_maxp(&urb->ep->desc));
	/* How much data is in the first TRB? */
	addr = (u64) urb->transfer_dma;
	trb_buff_len = TRB_MAX_BUFF_SIZE -
		(urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	if (trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;

	first_trb = true;
	last_trb_num = zero_length_needed ? 2 : 1;
	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 remainder = 0;
		field = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= 0x1;
		} else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > last_trb_num) {
			field |= TRB_CHAIN;
		} else if (num_trbs == last_trb_num) {
			td->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		} else if (zero_length_needed && num_trbs == 1) {
			trb_buff_len = 0;
			urb_priv->td[1]->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_urb_dir_in(urb))
			field |= TRB_ISP;

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if (num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;
		queue_trb(xhci, ep_ring, more_trbs_coming,
				lower_32_bits(addr),
				upper_32_bits(addr),
				length_field,
				field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = urb->transfer_buffer_length - running_total;
		if (trb_buff_len > TRB_MAX_BUFF_SIZE)
			trb_buff_len = TRB_MAX_BUFF_SIZE;
	} while (num_trbs > 0);

	//check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id,
			start_cycle, start_trb);
	return 0;
}

/* This is very similar to what ehci-q.c qtd_fill() does */
/*weitaowang: port to linux_4.4 done some redundant items need to modify*/
int xhci_queue_bulk_tx_zero_len_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	int num_trbs;
	struct xhci_generic_trb *start_trb;
	bool first_trb;
	int last_trb_num;
	bool more_trbs_coming;
	bool zero_length_needed;
	int start_cycle;
	u32 field, length_field;

	int running_total, trb_buff_len, ret;
	unsigned int total_packet_count;
	u64 addr;

	pr_info("enter into xhci_queue_bulk_tx_zero_len_trb\n");
	if(urb->num_sgs)
		return queue_bulk_sg_tx_zero_len_trb(xhci, mem_flags, urb, slot_id, ep_index);

	pr_info("no sg tables\n");

	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if (!ep_ring)
		return -EINVAL;

	num_trbs = 0;
	/* How much data is (potentially) left before the 64KB boundary? */
	running_total = TRB_MAX_BUFF_SIZE -
		(urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	/* If there's some data on this 64KB chunk, or we have to send a
	 * zero-length transfer, we need at least one TRB
	 */
	if (running_total != 0 || urb->transfer_buffer_length == 0)
		num_trbs++;
	/* How many more 64KB chunks to transfer, how many more TRBs? */
	while (running_total < urb->transfer_buffer_length) {
		num_trbs++;
		running_total += TRB_MAX_BUFF_SIZE;
	}

	ret = prepare_transfer(xhci, xhci->devs[slot_id],
			ep_index, urb->stream_id,
			num_trbs, urb, 0, mem_flags);
	if (ret < 0)
		return ret;

	urb_priv = urb->hcpriv;

	/* Deal with URB_ZERO_PACKET - need one more td/trb */
	zero_length_needed = urb->transfer_flags & URB_ZERO_PACKET &&
		urb_priv->length == 2;
	if (zero_length_needed) {
		num_trbs++;
		xhci_dbg(xhci, "Creating zero length td.\n");
		ret = prepare_transfer(xhci, xhci->devs[slot_id],
				ep_index, urb->stream_id,
				1, urb, 1, mem_flags);
		if (ret < 0)
			return ret;
	}

	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	running_total = 0;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length,
			usb_endpoint_maxp(&urb->ep->desc));
	/* How much data is in the first TRB? */
	addr = (u64) urb->transfer_dma;
	trb_buff_len = TRB_MAX_BUFF_SIZE -
		(urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	if (trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;

	first_trb = true;
	last_trb_num = zero_length_needed ? 2 : 1;
	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 remainder = 0;
		field = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= 0x1;
		} else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > last_trb_num) {
			field |= TRB_CHAIN;
		} else if (num_trbs == last_trb_num) {
			td->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		} else if (zero_length_needed && num_trbs == 1) {
			trb_buff_len = 0;
			urb_priv->td[1]->last_trb = ep_ring->enqueue;
			field |= TRB_IOC;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_urb_dir_in(urb))
			field |= TRB_ISP;

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if (num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;
		queue_trb(xhci, ep_ring, more_trbs_coming,
				lower_32_bits(addr),
				upper_32_bits(addr),
				length_field,
				field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = urb->transfer_buffer_length - running_total;
		if (trb_buff_len > TRB_MAX_BUFF_SIZE)
			trb_buff_len = TRB_MAX_BUFF_SIZE;
	} while (num_trbs > 0);

	//check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id,
			start_cycle, start_trb);
	return 0;
}

/* This is very similar to what ehci-q.c qtd_fill() does */
/*weitaowang: port to linux_4.4 done base on the linux316.*/
int xhci_queue_bulk_tx_event_data_trb(struct xhci_hcd *xhci, gfp_t mem_flags,
	struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	int num_trbs;
	struct xhci_generic_trb *start_trb;
	bool first_trb;
	bool more_trbs_coming;
	int start_cycle;
	u32 field, length_field;

	int running_total, trb_buff_len, ret;
	unsigned int total_packet_count;
	u64 addr;
	u64 event_trb_dma_addr;

	pr_info("enter into xhci_queue_bulk_tx_event_data_trb\n");
	if(urb->num_sgs)
		//return queue_bulk_sg_tx(xhci, mem_flags, urb, slot_id, ep_index);
		return xhci_verify_queue_bulk_sg_tx(xhci, mem_flags, urb, slot_id, ep_index);
		

	pr_info("no sg tables\n");
	ep_ring = xhci_urb_to_transfer_ring(xhci, urb);
	if(!ep_ring)
		return -EINVAL;

	num_trbs = 0;
	/* How much data is (potentially) left before the 64KB boundary? */
	running_total = TRB_MAX_BUFF_SIZE - (urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	/* If there's some data on this 64KB chunk, or we have to send a
	 * zero-length transfer, we need at least one TRB
	 */
	if(running_total != 0 || urb->transfer_buffer_length == 0)
		num_trbs++;
	/* How many more 64KB chunks to transfer, how many more TRBs? */
	while(running_total < urb->transfer_buffer_length)
	{
		num_trbs++;
		running_total += TRB_MAX_BUFF_SIZE;
	}
	/* FIXME: this doesn't deal with URB_ZERO_PACKET - need one more */

	//add one event data TRB as the last TRB
	num_trbs++;

	ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index, urb->stream_id, num_trbs,
		urb, 0, mem_flags);
	if(ret < 0)
		return ret;

	urb_priv = urb->hcpriv;
	td = urb_priv->td[0];

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	running_total = 0;
	total_packet_count = DIV_ROUND_UP(urb->transfer_buffer_length, usb_endpoint_maxp(&urb->ep->desc));
	/* How much data is in the first TRB? */
	addr = (u64) urb->transfer_dma;
	trb_buff_len = TRB_MAX_BUFF_SIZE - (urb->transfer_dma & (TRB_MAX_BUFF_SIZE - 1));
	if(trb_buff_len > urb->transfer_buffer_length)
		trb_buff_len = urb->transfer_buffer_length;

	first_trb = true;

	/* Queue the first TRB, even if it's zero-length */
	do
	{
		u32 remainder = 0;
		field = 0;

		/* Don't change the cycle bit of the first TRB until later */
		if(first_trb)
		{
			first_trb = false;
			if(start_cycle == 0)
				field |= 0x1;
		}
		else
			field |= ep_ring->cycle_state;

		/* Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if(num_trbs > 1)
		{
			field |= TRB_CHAIN;
		}
		else
		{
			/* FIXME - add check for ZERO_PACKET flag before this */
			//td->last_trb = ep_ring->enqueue;
			//field |= TRB_IOC;
		}

		/* Only set interrupt on short packet for IN endpoints */
		//if(usb_urb_dir_in(urb))
			//field |= TRB_ISP;

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(xhci, running_total, trb_buff_len,
					   urb->transfer_buffer_length,
					   urb, num_trbs - 1);

		length_field = TRB_LEN(trb_buff_len) |
			TRB_TD_SIZE(remainder) |
			TRB_INTR_TARGET(0);

		if(num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;

		//queue_trb(xhci, ep_ring, more_trbs_coming, lower_32_bits(addr),
			//upper_32_bits(addr), length_field, field | TRB_ISP | TRB_TYPE(TRB_NORMAL));
		queue_trb(xhci, ep_ring, more_trbs_coming, lower_32_bits(addr),
			upper_32_bits(addr), length_field, field | TRB_TYPE(TRB_NORMAL));
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = urb->transfer_buffer_length - running_total;
		if(trb_buff_len > TRB_MAX_BUFF_SIZE)
			trb_buff_len = TRB_MAX_BUFF_SIZE;
	}while(running_total < urb->transfer_buffer_length);

	//queue the last event data TRB
	td->last_trb = ep_ring->enqueue;
	--num_trbs;
	event_trb_dma_addr = ep_ring->enq_seg->dma +
		  (sizeof(struct xhci_generic_trb) * (ep_ring->enqueue - ep_ring->enq_seg->trbs));
			

	field = 0;
	field |= ep_ring->cycle_state;
	field |= TRB_IOC;
	queue_trb(xhci, ep_ring, false, lower_32_bits(event_trb_dma_addr),
		upper_32_bits(event_trb_dma_addr), TRB_INTR_TARGET(0),
		field | TRB_TYPE(TRB_EVENT_DATA));
	pr_info("queue event trb");

	//check_trb_math(urb, num_trbs, running_total);
	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id, start_cycle, start_trb);
	return 0;
}

/*weitaowang add:xhci_verify_mark end*/

/*
 * The transfer burst count field of the isochronous TRB defines the number of
 * bursts that are required to move all packets in this TD.  Only SuperSpeed
 * devices can burst up to bMaxBurst number of packets per service interval.
 * This field is zero based, meaning a value of zero in the field means one
 * burst.  Basically, for everything but SuperSpeed devices, this field will be
 * zero.  Only xHCI 1.0 host controllers support this field.
 */
static unsigned int xhci_get_burst_count(struct xhci_hcd *xhci,
		struct usb_device *udev,
		struct urb *urb, unsigned int total_packet_count)
{
	unsigned int max_burst;

	if (xhci->hci_version < 0x100 || udev->speed != USB_SPEED_SUPER)
		return 0;

	max_burst = urb->ep->ss_ep_comp.bMaxBurst;
	return DIV_ROUND_UP(total_packet_count, max_burst + 1) - 1;
}

/*
 * Returns the number of packets in the last "burst" of packets.  This field is
 * valid for all speeds of devices.  USB 2.0 devices can only do one "burst", so
 * the last burst packet count is equal to the total number of packets in the
 * TD.  SuperSpeed endpoints can have up to 3 bursts.  All but the last burst
 * must contain (bMaxBurst + 1) number of packets, but the last burst can
 * contain 1 to (bMaxBurst + 1) packets.
 */
static unsigned int xhci_get_last_burst_packet_count(struct xhci_hcd *xhci,
		struct usb_device *udev,
		struct urb *urb, unsigned int total_packet_count)
{
	unsigned int max_burst;
	unsigned int residue;

	if (xhci->hci_version < 0x100)
		return 0;

	switch (udev->speed) {
	case USB_SPEED_SUPER:
		/* bMaxBurst is zero based: 0 means 1 packet per burst */
		max_burst = urb->ep->ss_ep_comp.bMaxBurst;
		residue = total_packet_count % (max_burst + 1);
		/* If residue is zero, the last burst contains (max_burst + 1)
		 * number of packets, but the TLBPC field is zero-based.
		 */
		if (residue == 0)
			return max_burst;
		return residue - 1;
	default:
		if (total_packet_count == 0)
			return 0;
		return total_packet_count - 1;
	}
}

/*
 * Calculates Frame ID field of the isochronous TRB identifies the
 * target frame that the Interval associated with this Isochronous
 * Transfer Descriptor will start on. Refer to 4.11.2.5 in 1.1 spec.
 *
 * Returns actual frame id on success, negative value on error.
 */
static int xhci_get_isoc_frame_id(struct xhci_hcd *xhci,
		struct urb *urb, int index)
{
	int start_frame, ist, ret = 0;
	int start_frame_id, end_frame_id, current_frame_id;

	if (urb->dev->speed == USB_SPEED_LOW ||
			urb->dev->speed == USB_SPEED_FULL)
		start_frame = urb->start_frame + index * urb->interval;
	else
		start_frame = (urb->start_frame + index * urb->interval) >> 3;

	/* Isochronous Scheduling Threshold (IST, bits 0~3 in HCSPARAMS2):
	 *
	 * If bit [3] of IST is cleared to '0', software can add a TRB no
	 * later than IST[2:0] Microframes before that TRB is scheduled to
	 * be executed.
	 * If bit [3] of IST is set to '1', software can add a TRB no later
	 * than IST[2:0] Frames before that TRB is scheduled to be executed.
	 */
	ist = HCS_IST(xhci->hcs_params2) & 0x7;
	if (HCS_IST(xhci->hcs_params2) & (1 << 3))
		ist <<= 3;

	/* Software shall not schedule an Isoch TD with a Frame ID value that
	 * is less than the Start Frame ID or greater than the End Frame ID,
	 * where:
	 *
	 * End Frame ID = (Current MFINDEX register value + 895 ms.) MOD 2048
	 * Start Frame ID = (Current MFINDEX register value + IST + 1) MOD 2048
	 *
	 * Both the End Frame ID and Start Frame ID values are calculated
	 * in microframes. When software determines the valid Frame ID value;
	 * The End Frame ID value should be rounded down to the nearest Frame
	 * boundary, and the Start Frame ID value should be rounded up to the
	 * nearest Frame boundary.
	 */
	current_frame_id = readl(&xhci->run_regs->microframe_index);
	start_frame_id = roundup(current_frame_id + ist + 1, 8);
	end_frame_id = rounddown(current_frame_id + 895 * 8, 8);

	start_frame &= 0x7ff;
	start_frame_id = (start_frame_id >> 3) & 0x7ff;
	end_frame_id = (end_frame_id >> 3) & 0x7ff;

	xhci_dbg(xhci, "%s: index %d, reg 0x%x start_frame_id 0x%x, end_frame_id 0x%x, start_frame 0x%x\n",
		 __func__, index, readl(&xhci->run_regs->microframe_index),
		 start_frame_id, end_frame_id, start_frame);

	if (start_frame_id < end_frame_id) {
		if (start_frame > end_frame_id ||
				start_frame < start_frame_id)
			ret = -EINVAL;
	} else if (start_frame_id > end_frame_id) {
		if ((start_frame > end_frame_id &&
				start_frame < start_frame_id))
			ret = -EINVAL;
	} else {
			ret = -EINVAL;
	}

	if (index == 0) {
		if (ret == -EINVAL || start_frame == start_frame_id) {
			start_frame = start_frame_id + 1;
			if (urb->dev->speed == USB_SPEED_LOW ||
					urb->dev->speed == USB_SPEED_FULL)
				urb->start_frame = start_frame;
			else
				urb->start_frame = start_frame << 3;
			ret = 0;
		}
	}

	if (ret) {
		xhci_warn(xhci, "Frame ID %d (reg %d, index %d) beyond range (%d, %d)\n",
				start_frame, current_frame_id, index,
				start_frame_id, end_frame_id);
		xhci_warn(xhci, "Ignore frame ID field, use SIA bit instead\n");
		return ret;
	}

	return start_frame;
}

/* This is for isoc transfer */
static int xhci_queue_isoc_tx(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_ring *ep_ring;
	struct urb_priv *urb_priv;
	struct xhci_td *td;
	int num_tds, trbs_per_td;
	struct xhci_generic_trb *start_trb;
	bool first_trb;
	int start_cycle;
	u32 field, length_field;
	int running_total, trb_buff_len, td_len, td_remain_len, ret;
	u64 start_addr, addr;
	int i, j;
	bool more_trbs_coming;
	struct xhci_virt_ep *xep;

	xep = &xhci->devs[slot_id]->eps[ep_index];
	ep_ring = xhci->devs[slot_id]->eps[ep_index].ring;

	num_tds = urb->number_of_packets;
	if (num_tds < 1) {
		xhci_dbg(xhci, "Isoc URB with zero packets?\n");
		return -EINVAL;
	}

	start_addr = (u64) urb->transfer_dma;
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;

	urb_priv = urb->hcpriv;
	/* Queue the first TRB, even if it's zero-length */
	for (i = 0; i < num_tds; i++) {
		unsigned int total_packet_count;
		unsigned int burst_count;
		unsigned int residue;

		first_trb = true;
		running_total = 0;
		addr = start_addr + urb->iso_frame_desc[i].offset;
		td_len = urb->iso_frame_desc[i].length;
		td_remain_len = td_len;
		total_packet_count = DIV_ROUND_UP(td_len,
				GET_MAX_PACKET(
					usb_endpoint_maxp(&urb->ep->desc)));
		/* A zero-length transfer still involves at least one packet. */
		if (total_packet_count == 0)
			total_packet_count++;
		burst_count = xhci_get_burst_count(xhci, urb->dev, urb,
				total_packet_count);
		residue = xhci_get_last_burst_packet_count(xhci,
				urb->dev, urb, total_packet_count);

		trbs_per_td = count_isoc_trbs_needed(xhci, urb, i);

		ret = prepare_transfer(xhci, xhci->devs[slot_id], ep_index,
				urb->stream_id, trbs_per_td, urb, i, mem_flags);
		if (ret < 0) {
			if (i == 0)
				return ret;
			goto cleanup;
		}

		td = urb_priv->td[i];
		for (j = 0; j < trbs_per_td; j++) {
			int frame_id = 0;
			u32 remainder = 0;
			field = 0;

			if (first_trb) {
				field = TRB_TBC(burst_count) |
					TRB_TLBPC(residue);
				/* Queue the isoc TRB */
				field |= TRB_TYPE(TRB_ISOC);

				/* Calculate Frame ID and SIA fields */
				if (!(urb->transfer_flags & URB_ISO_ASAP) &&
						HCC_CFC(xhci->hcc_params)) {
					frame_id = xhci_get_isoc_frame_id(xhci,
									  urb,
									  i);
					if (frame_id >= 0)
						field |= TRB_FRAME_ID(frame_id);
					else
						field |= TRB_SIA;
				} else
					field |= TRB_SIA;

				if (i == 0) {
					if (start_cycle == 0)
						field |= 0x1;
				} else
					field |= ep_ring->cycle_state;
				first_trb = false;
			} else {
				/* Queue other normal TRBs */
				field |= TRB_TYPE(TRB_NORMAL);
				field |= ep_ring->cycle_state;
			}

			/* Only set interrupt on short packet for IN EPs */
			if (usb_urb_dir_in(urb))
				field |= TRB_ISP;

			/* Chain all the TRBs together; clear the chain bit in
			 * the last TRB to indicate it's the last TRB in the
			 * chain.
			 */
			if (j < trbs_per_td - 1) {
				field |= TRB_CHAIN;
				more_trbs_coming = true;
			} else {
				td->last_trb = ep_ring->enqueue;
				field |= TRB_IOC;
				if (xhci->hci_version == 0x100 &&
						!(xhci->quirks &
							XHCI_AVOID_BEI)) {
					/* Set BEI bit except for the last td */
					if (i < num_tds - 1)
						field |= TRB_BEI;
				}
				more_trbs_coming = false;
			}

			/* Calculate TRB length */
			trb_buff_len = TRB_MAX_BUFF_SIZE -
				(addr & ((1 << TRB_MAX_BUFF_SHIFT) - 1));
			if (trb_buff_len > td_remain_len)
				trb_buff_len = td_remain_len;

			/* Set the TRB length, TD size, & interrupter fields. */
			remainder = xhci_td_remainder(xhci, running_total,
						   trb_buff_len, td_len,
						   urb, trbs_per_td - j - 1);

			length_field = TRB_LEN(trb_buff_len) |
				TRB_TD_SIZE(remainder) |
				TRB_INTR_TARGET(0);

			queue_trb(xhci, ep_ring, more_trbs_coming,
				lower_32_bits(addr),
				upper_32_bits(addr),
				length_field,
				field);
			running_total += trb_buff_len;

			addr += trb_buff_len;
			td_remain_len -= trb_buff_len;
		}

		/* Check TD length */
		if (running_total != td_len) {
			xhci_err(xhci, "ISOC TD length unmatch\n");
			ret = -EINVAL;
			goto cleanup;
		}
	}

	/* store the next frame id */
	if (HCC_CFC(xhci->hcc_params))
		xep->next_frame_id = urb->start_frame + num_tds * urb->interval;

	if (xhci_to_hcd(xhci)->self.bandwidth_isoc_reqs == 0) {
		if (xhci->quirks & XHCI_AMD_PLL_FIX)
			usb_amd_quirk_pll_disable();
	}
	xhci_to_hcd(xhci)->self.bandwidth_isoc_reqs++;

	giveback_first_trb(xhci, slot_id, ep_index, urb->stream_id,
			start_cycle, start_trb);
	return 0;
cleanup:
	/* Clean up a partially enqueued isoc transfer. */

	for (i--; i >= 0; i--)
		list_del_init(&urb_priv->td[i]->td_list);

	/* Use the first TD as a temporary variable to turn the TDs we've queued
	 * into No-ops with a software-owned cycle bit. That way the hardware
	 * won't accidentally start executing bogus TDs when we partially
	 * overwrite them.  td->first_trb and td->start_seg are already set.
	 */
	urb_priv->td[0]->last_trb = ep_ring->enqueue;
	/* Every TRB except the first & last will have its cycle bit flipped. */
	td_to_noop(xhci, ep_ring, urb_priv->td[0], true);

	/* Reset the ring enqueue back to the first TRB and its cycle bit. */
	ep_ring->enqueue = urb_priv->td[0]->first_trb;
	ep_ring->enq_seg = urb_priv->td[0]->start_seg;
	ep_ring->cycle_state = start_cycle;
	ep_ring->num_trbs_free = ep_ring->num_trbs_free_temp;
	usb_hcd_unlink_urb_from_ep(bus_to_hcd(urb->dev->bus), urb);
	return ret;
}

/*
 * Check transfer ring to guarantee there is enough room for the urb.
 * Update ISO URB start_frame and interval.
 * Update interval as xhci_queue_intr_tx does. Use xhci frame_index to
 * update urb->start_frame if URB_ISO_ASAP is set in transfer_flags or
 * Contiguous Frame ID is not supported by HC.
 */
int xhci_queue_isoc_tx_prepare(struct xhci_hcd *xhci, gfp_t mem_flags,
		struct urb *urb, int slot_id, unsigned int ep_index)
{
	struct xhci_virt_device *xdev;
	struct xhci_ring *ep_ring;
	struct xhci_ep_ctx *ep_ctx;
	int start_frame;
	int xhci_interval;
	int ep_interval;
	int num_tds, num_trbs, i;
	int ret;
	struct xhci_virt_ep *xep;
	int ist;

	xdev = xhci->devs[slot_id];
	xep = &xhci->devs[slot_id]->eps[ep_index];
	ep_ring = xdev->eps[ep_index].ring;
	ep_ctx = xhci_get_ep_ctx(xhci, xdev->out_ctx, ep_index);

	num_trbs = 0;
	num_tds = urb->number_of_packets;
	for (i = 0; i < num_tds; i++)
		num_trbs += count_isoc_trbs_needed(xhci, urb, i);

	/* Check the ring to guarantee there is enough room for the whole urb.
	 * Do not insert any td of the urb to the ring if the check failed.
	 */
	ret = prepare_ring(xhci, ep_ring, le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK,
			   num_trbs, mem_flags);
	if (ret)
		return ret;

	/*
	 * Check interval value. This should be done before we start to
	 * calculate the start frame value.
	 */
	xhci_interval = EP_INTERVAL_TO_UFRAMES(le32_to_cpu(ep_ctx->ep_info));
	ep_interval = urb->interval;
	/* Convert to microframes */
	if (urb->dev->speed == USB_SPEED_LOW ||
			urb->dev->speed == USB_SPEED_FULL)
		ep_interval *= 8;
	/* FIXME change this to a warning and a suggestion to use the new API
	 * to set the polling interval (once the API is added).
	 */
	if (xhci_interval != ep_interval) {
		dev_dbg_ratelimited(&urb->dev->dev,
				"Driver uses different interval (%d microframe%s) than xHCI (%d microframe%s)\n",
				ep_interval, ep_interval == 1 ? "" : "s",
				xhci_interval, xhci_interval == 1 ? "" : "s");
		urb->interval = xhci_interval;
		/* Convert back to frames for LS/FS devices */
		if (urb->dev->speed == USB_SPEED_LOW ||
				urb->dev->speed == USB_SPEED_FULL)
			urb->interval /= 8;
	}

	/* Calculate the start frame and put it in urb->start_frame. */
        if (HCC_CFC(xhci->hcc_params) && !list_empty(&ep_ring->td_list)) {
                if ((le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK) ==
                                EP_STATE_RUNNING) {
                        urb->start_frame = xep->next_frame_id;
                        goto skip_start_over;
                }
        }
	start_frame = readl(&xhci->run_regs->microframe_index);
	start_frame &= 0x3fff;
	/*
	 * Round up to the next frame and consider the time before trb really
	 * gets scheduled by hardare.
	 */
	ist = HCS_IST(xhci->hcs_params2) & 0x7;
	if (HCS_IST(xhci->hcs_params2) & (1 << 3))
		ist <<= 3;
	start_frame += ist + XHCI_CFC_DELAY;
	start_frame = roundup(start_frame, 8);

	/*
	 * Round up to the next ESIT (Endpoint Service Interval Time) if ESIT
	 * is greate than 8 microframes.
	 */
	if (urb->dev->speed == USB_SPEED_LOW ||
			urb->dev->speed == USB_SPEED_FULL) {
		start_frame = roundup(start_frame, urb->interval << 3);
		urb->start_frame = start_frame >> 3;
	} else {
		start_frame = roundup(start_frame, urb->interval);
		urb->start_frame = start_frame;
	}

skip_start_over:
	ep_ring->num_trbs_free_temp = ep_ring->num_trbs_free;

	return xhci_queue_isoc_tx(xhci, mem_flags, urb, slot_id, ep_index);
}

/****		Command Ring Operations		****/

/* Generic function for queueing a command TRB on the command ring.
 * Check to make sure there's room on the command ring for one command TRB.
 * Also check that there's room reserved for commands that must not fail.
 * If this is a command that must not fail, meaning command_must_succeed = TRUE,
 * then only check for the number of reserved spots.
 * Don't decrement xhci->cmd_ring_reserved_trbs after we've queued the TRB
 * because the command event handler may want to resubmit a failed command.
 */
 int queue_command(struct xhci_hcd *xhci, struct xhci_command *cmd,
			 u32 field1, u32 field2,
			 u32 field3, u32 field4, bool command_must_succeed)
{
	int reserved_trbs = xhci->cmd_ring_reserved_trbs;
	int ret;

	if (xhci->xhc_state) {
		xhci_dbg(xhci, "xHCI dying or halted, can't queue_command\n");
		return -ESHUTDOWN;
	}

	if (!command_must_succeed)
		reserved_trbs++;

	ret = prepare_ring(xhci, xhci->cmd_ring, EP_STATE_RUNNING,
			reserved_trbs, GFP_ATOMIC);
	if (ret < 0) {
		xhci_err(xhci, "ERR: No room for command on command ring\n");
		if (command_must_succeed)
			xhci_err(xhci, "ERR: Reserved TRB counting for "
					"unfailable commands failed.\n");
		return ret;
	}

	cmd->command_trb = xhci->cmd_ring->enqueue;
	list_add_tail(&cmd->cmd_list, &xhci->cmd_list);

	/* if there are no other commands queued we start the timeout timer */
	if (xhci->cmd_list.next == &cmd->cmd_list &&
	    !timer_pending(&xhci->cmd_timer)) {
		xhci->current_cmd = cmd;
		mod_timer(&xhci->cmd_timer, jiffies + XHCI_CMD_DEFAULT_TIMEOUT);
	}

	queue_trb(xhci, xhci->cmd_ring, false, field1, field2, field3,
			field4 | xhci->cmd_ring->cycle_state);
	return 0;
}

/* Queue a slot enable or disable request on the command ring */
int xhci_queue_slot_control(struct xhci_hcd *xhci, struct xhci_command *cmd,
		u32 trb_type, u32 slot_id)
{
	return queue_command(xhci, cmd, 0, 0, 0,
			TRB_TYPE(trb_type) | SLOT_ID_FOR_TRB(slot_id), false);
}

/* Queue an address device command TRB */
int xhci_queue_address_device(struct xhci_hcd *xhci, struct xhci_command *cmd,
		dma_addr_t in_ctx_ptr, u32 slot_id, enum xhci_setup_dev setup)
{
	return queue_command(xhci, cmd, lower_32_bits(in_ctx_ptr),
			upper_32_bits(in_ctx_ptr), 0,
			TRB_TYPE(TRB_ADDR_DEV) | SLOT_ID_FOR_TRB(slot_id)
			| (setup == SETUP_CONTEXT_ONLY ? TRB_BSR : 0), false);
}

int xhci_queue_vendor_command(struct xhci_hcd *xhci, struct xhci_command *cmd,
		u32 field1, u32 field2, u32 field3, u32 field4)
{
	return queue_command(xhci, cmd, field1, field2, field3, field4, false);
}

/* Queue a reset device command TRB */
int xhci_queue_reset_device(struct xhci_hcd *xhci, struct xhci_command *cmd,
		u32 slot_id)
{
	return queue_command(xhci, cmd, 0, 0, 0,
			TRB_TYPE(TRB_RESET_DEV) | SLOT_ID_FOR_TRB(slot_id),
			false);
}

/* Queue a configure endpoint command TRB */
int xhci_queue_configure_endpoint(struct xhci_hcd *xhci,
		struct xhci_command *cmd, dma_addr_t in_ctx_ptr,
		u32 slot_id, bool command_must_succeed)
{
	return queue_command(xhci, cmd, lower_32_bits(in_ctx_ptr),
			upper_32_bits(in_ctx_ptr), 0,
			TRB_TYPE(TRB_CONFIG_EP) | SLOT_ID_FOR_TRB(slot_id),
			command_must_succeed);
}

/* Queue an evaluate context command TRB */
int xhci_queue_evaluate_context(struct xhci_hcd *xhci, struct xhci_command *cmd,
		dma_addr_t in_ctx_ptr, u32 slot_id, bool command_must_succeed)
{
	return queue_command(xhci, cmd, lower_32_bits(in_ctx_ptr),
			upper_32_bits(in_ctx_ptr), 0,
			TRB_TYPE(TRB_EVAL_CONTEXT) | SLOT_ID_FOR_TRB(slot_id),
			command_must_succeed);
}

/*
 * Suspend is set to indicate "Stop Endpoint Command" is being issued to stop
 * activity on an endpoint that is about to be suspended.
 */
int xhci_queue_stop_endpoint(struct xhci_hcd *xhci, struct xhci_command *cmd,
			     int slot_id, unsigned int ep_index, int suspend)
{
	u32 trb_slot_id = SLOT_ID_FOR_TRB(slot_id);
	u32 trb_ep_index = EP_ID_FOR_TRB(ep_index);
	u32 type = TRB_TYPE(TRB_STOP_RING);
	u32 trb_suspend = SUSPEND_PORT_FOR_TRB(suspend);

	return queue_command(xhci, cmd, 0, 0, 0,
			trb_slot_id | trb_ep_index | type | trb_suspend, false);
}

/* Set Transfer Ring Dequeue Pointer command */
void xhci_queue_new_dequeue_state(struct xhci_hcd *xhci,
		unsigned int slot_id, unsigned int ep_index,
		unsigned int stream_id,
		struct xhci_dequeue_state *deq_state)
{
	dma_addr_t addr;
	u32 trb_slot_id = SLOT_ID_FOR_TRB(slot_id);
	u32 trb_ep_index = EP_ID_FOR_TRB(ep_index);
	u32 trb_stream_id = STREAM_ID_FOR_TRB(stream_id);
	u32 trb_sct = 0;
	u32 type = TRB_TYPE(TRB_SET_DEQ);
	struct xhci_virt_ep *ep;
	struct xhci_command *cmd;
	int ret;

	xhci_dbg_trace(xhci, trace_xhci_dbg_cancel_urb,
		"Set TR Deq Ptr cmd, new deq seg = %p (0x%llx dma), new deq ptr = %p (0x%llx dma), new cycle = %u",
		deq_state->new_deq_seg,
		(unsigned long long)deq_state->new_deq_seg->dma,
		deq_state->new_deq_ptr,
		(unsigned long long)xhci_trb_virt_to_dma(
			deq_state->new_deq_seg, deq_state->new_deq_ptr),
		deq_state->new_cycle_state);

	addr = xhci_trb_virt_to_dma(deq_state->new_deq_seg,
				    deq_state->new_deq_ptr);
	if (addr == 0) {
		xhci_warn(xhci, "WARN Cannot submit Set TR Deq Ptr\n");
		xhci_warn(xhci, "WARN deq seg = %p, deq pt = %p\n",
			  deq_state->new_deq_seg, deq_state->new_deq_ptr);
		return;
	}
	ep = &xhci->devs[slot_id]->eps[ep_index];
	if ((ep->ep_state & SET_DEQ_PENDING)) {
		xhci_warn(xhci, "WARN Cannot submit Set TR Deq Ptr\n");
		xhci_warn(xhci, "A Set TR Deq Ptr command is pending.\n");
		return;
	}

	/* This function gets called from contexts where it cannot sleep */
	cmd = xhci_alloc_command(xhci, false, false, GFP_ATOMIC);
	if (!cmd) {
		xhci_warn(xhci, "WARN Cannot submit Set TR Deq Ptr: ENOMEM\n");
		return;
	}

	ep->queued_deq_seg = deq_state->new_deq_seg;
	ep->queued_deq_ptr = deq_state->new_deq_ptr;
	if (stream_id)
		trb_sct = SCT_FOR_TRB(SCT_PRI_TR);
	ret = queue_command(xhci, cmd,
		lower_32_bits(addr) | trb_sct | deq_state->new_cycle_state,
		upper_32_bits(addr), trb_stream_id,
		trb_slot_id | trb_ep_index | type, false);
	if (ret < 0) {
		xhci_free_command(xhci, cmd);
		return;
	}

	/* Stop the TD queueing code from ringing the doorbell until
	 * this command completes.  The HC won't set the dequeue pointer
	 * if the ring is running, and ringing the doorbell starts the
	 * ring running.
	 */
	ep->ep_state |= SET_DEQ_PENDING;
}

int xhci_queue_reset_ep(struct xhci_hcd *xhci, struct xhci_command *cmd,
			int slot_id, unsigned int ep_index)
{
	u32 trb_slot_id = SLOT_ID_FOR_TRB(slot_id);
	u32 trb_ep_index = EP_ID_FOR_TRB(ep_index);
	u32 type = TRB_TYPE(TRB_RESET_EP);

	return queue_command(xhci, cmd, 0, 0, 0,
			trb_slot_id | trb_ep_index | type, false);
}
