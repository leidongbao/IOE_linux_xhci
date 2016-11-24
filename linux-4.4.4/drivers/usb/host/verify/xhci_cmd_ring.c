/*
 * Verify Command Ring
 * Jeff @ VIA 2015-1-29
 *
 */

#include <linux/usb.h>
#include "xhci.h"
#include "xhci_ctler_common.h"
#include "xhci_cmd_ring.h"
#include "xhci_ctler_verify.h"

unsigned int check_route_string_valid(int xhci_cmd)
{
	switch (xhci_cmd)
	{
		case TRB_NEG_BANDWIDTH:
		case TRB_GET_BW:
		case TRB_FORCE_HEADER:
		case TRB_RESET_DEV:
		case TRB_STOP_RING:
		case TRB_SET_DEQ:
		case TRB_RESET_EP:
		case TRB_EVAL_CONTEXT:
			return 1;

		default:
			break;
	}
	return 0;
}

unsigned int check_endpoint_valid(int xhci_cmd)
{
	switch (xhci_cmd)
	{
		case TRB_RESET_EP:
		case TRB_EVAL_CONTEXT:
			return 1;

		default:
			break;
	}
	return 0;
}

int cmd_not_support(struct xhci_ctler *ctler, int xhci_cmd)
{
	switch (xhci_cmd)
	{
	case TRB_ENABLE_SLOT:
	case TRB_DISABLE_SLOT:
	case TRB_ADDR_DEV:
	case TRB_CONFIG_EP:
		write_log(ctler,
			"Oops: this xhci command is not support here."
			"this command has been executed successfully when device emulated\n");
		break;
	case TRB_FORCE_EVENT:/* optional support */
		write_log(ctler, "Fail: this xhci command is not support here\n");
		break;
	case TRB_EVAL_CONTEXT:
		write_log(ctler,
			"Fail: this xhci command is not support here."
			" this command has been executed successfully when full speed device emunated\n");
		break;
	case TRB_SET_LT: /* optional support */
		write_log(ctler,
			"Fail: this xhci command is not support here."
			" this command has been executed in enable_ltm function\n");
		break;
		/*command not found, 0 means support. */
	default:
		return 0;
	}
	return -EINVAL;
}

void cmd_no_op(unsigned *field1, unsigned *field2, unsigned *field3, unsigned *field4)
{
	pr_info("COMMAND: no op\n");
	*field1 = 0;
	*field2 = 0;
	*field3 = 0;
	*field4 = TRB_TYPE(TRB_CMD_NOOP);
}

void cmd_reset_endpoint(struct xhci_hcd *xhci,
	struct usb_host_endpoint *ep, struct usb_device *udev,
	unsigned *field1, unsigned *field2, unsigned *field3, unsigned *field4)
{
	int ep_index = 0;
	struct xhci_container_ctx *out_ctx = NULL;
	struct xhci_ep_ctx *ep_ctx = NULL;

	ep_index = xhci_get_endpoint_index(&ep->desc);
	out_ctx = xhci->devs[udev->slot_id]->out_ctx;
	ep_ctx = xhci_get_ep_ctx(xhci, out_ctx, ep_index);

	pr_info("COMMAND: reset endpoint\n");
	if((ep_ctx->ep_info & EP_STATE_MASK) != EP_STATE_HALTED)
		pr_info("the target EP is not in halt state, reset ep maybe fail\n");
	
	*field1 = 0;
	*field2 = 0;
	*field3 = 0;
	*field4 = TRB_TYPE(TRB_RESET_EP) | SLOT_ID_FOR_TRB(udev->slot_id) | EP_ID_FOR_TRB(ep_index) | (1 << 9);
		
}

void cmd_force_header(struct usb_device *udev,unsigned *field1, 
	unsigned *field2, unsigned *field3, unsigned *field4)
{
	pr_info("COMMAND: force header\n");
	*field1 = 0x420;
	*field2 = 0;
	*field3 = 0;
	*field4 = TRB_TYPE(TRB_FORCE_HEADER);
	pr_info("udev->portnum: %d\n", udev->portnum);
	//*field4 |= (udev->portnum << 24);
	*field4 |= (2 << 24);
	dump_field(*field1, *field2, *field3, *field4);
}
		