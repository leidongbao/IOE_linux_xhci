#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <linux/random.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/ch11.h>
#include <linux/usb_usual.h>
#include <linux/usb/hcd.h>

#include <linux/dmapool.h>
#include <linux/string.h>

#include "xhci.h"
#include "xhci_ctler_common.h"
#include "xhci_ctler_verify.h"
#include "xhci-ring.h"

/*-----------------------------xhci_verify_mark : add ---------------------------------*/
/*xhci_verify_mark : support Spec 1.0 for isoch TBC and TLBPC */
#define ISOC_TRB_TBC(p)		(((p) & 0x3) << 7)
#define ISOC_TRB_TLBPC(p)	(((p) & 0xF) << 16)
#define CTRL_TRB_TRT(p)		(((p) & 0x3) << 16)
/*-------------------------------------------------------------------------------------*/


/* Root Port: 1 byte */
struct usb_device *xhci_verify_get_dev_by_route_string(struct xhci_ctler *ctler,
	unsigned int route_string)
{
	struct xhci_hcd *xhci = ctler->xhci;	
	struct xhci_virt_device *virt_dev;		
	struct xhci_slot_ctx *slot_ctx;
	struct usb_device *udev;
	int max_slots;
	int slot_id;
	unsigned int temp;
	unsigned int root_port;

	slot_id = 0;
	virt_dev = NULL;
	udev = NULL;
	max_slots = HCS_MAX_SLOTS(xhci_readl(xhci, &xhci->op_regs->config_reg));

	root_port = route_string & 0x00FF;
	temp = (route_string >> 8) & 0xFFFF;	//temp is the real route_string

	// poll all devs to compare the route string,and root_port
	for(slot_id = 1; slot_id < max_slots; slot_id++)
	{
		virt_dev = xhci->devs[slot_id];
		if(virt_dev == NULL)
			continue;
		udev = virt_dev->udev;
		slot_ctx = xhci_get_slot_ctx(xhci, virt_dev->out_ctx);

		//if it's on root hub
		if((temp == 0) && (((slot_ctx->dev_info2 >> 16) & 0xFF) == root_port))
			break;
		//not on root hub
		if((temp == udev->route) && (((slot_ctx->dev_info2 >> 16) & 0xFF) == root_port))
			break;
	}
	return udev;	//if false, udev == NULL;
}

struct usb_host_endpoint *xhci_verify_get_ep_by_address(struct usb_device *udev,
	unsigned char direction, unsigned char ep_address)
{
	int i, j;
	struct usb_interface *intf;
	struct usb_host_endpoint *ep = NULL;
	struct usb_host_endpoint *ep_tmp = NULL;

	i = j = 0;
	if(ep_address == 0)
		return (&udev->ep0);

	pr_info("udev is %s\n", udev->manufacturer);

	if(direction == 1)
		ep_tmp = udev->ep_in[ep_address];
	else
		ep_tmp = udev->ep_out[ep_address];

	if(ep_tmp == NULL)
	{
		pr_info("ep_tmp is NULL\n");
		//return NULL;
	}

	pr_info("udev->actconfig->desc.bNumInterfaces is %d\n",
		udev->actconfig->desc.bNumInterfaces);

	//return NULL;

	for(i = 0; i < udev->actconfig->desc.bNumInterfaces; i++)
	{
		intf = udev->actconfig->interface[i];
		if(intf == NULL)
			continue;
		pr_info("    interface %d: interface index 0x%x, intf alter setting num 0x%x, "
			"active intf setting index 0x%x, endpoint num 0x%x, intf class 0x%x, "
			"intf subclass 0x%x, intf protocol 0x%x, string descp: %s\n",
			i,
			intf->cur_altsetting->desc.bInterfaceNumber,
			intf->num_altsetting,
			intf->cur_altsetting->desc.bAlternateSetting,
			intf->cur_altsetting->desc.bNumEndpoints,
			intf->cur_altsetting->desc.bInterfaceClass,
			intf->cur_altsetting->desc.bInterfaceSubClass,
			intf->cur_altsetting->desc.bInterfaceProtocol,
			intf->cur_altsetting->string ? intf->cur_altsetting->string : "NULL");
		for(j = 0; j < intf->cur_altsetting->desc.bNumEndpoints; j++)
		{
			ep = &(intf->cur_altsetting->endpoint[j]);
			if(ep == NULL)
				continue;
			pr_info("ep num 0x%x\n", usb_endpoint_num(&ep->desc));
			pr_info("ep address 0x%x\n", ep_address);
			if(usb_endpoint_num(&ep->desc) == ep_address)
			{
				//pr_info("ep != ep_tmp \n");
				//pr_info("ep address 0x%x\n",ep->desc.bEndpointAddress);
				return ep;
			}
		}
	}
	return ep;
}

int xhci_verify_get_mass_storage_info(struct usb_device *udev,
	struct mass_storage_dev_info *info)
{
	int i, j;
	struct usb_host_endpoint *bulk_in, *bulk_out;
	struct usb_host_endpoint *int_in;
	struct usb_interface *intf;
	struct usb_host_endpoint *ep;

	for(i = 0; i < udev->actconfig->desc.bNumInterfaces; i++)
	{
		intf = udev->actconfig->interface[i];
		if(intf == NULL)
			continue;
		if(intf->cur_altsetting->desc.bInterfaceClass != USB_CLASS_MASS_STORAGE)
			continue;
		bulk_in = bulk_out = int_in = NULL;
		for(j = 0; j < intf->cur_altsetting->desc.bNumEndpoints; j++)
		{
			ep = &(intf->cur_altsetting->endpoint[j]);
			if(ep == NULL)
				continue;

			switch (ep->desc.bmAttributes)
			{
			case USB_ENDPOINT_XFER_BULK:
				if(usb_endpoint_dir_in(&ep->desc))
				{
					if(!bulk_in)
						bulk_in = ep;
				}
				else
				{
					if(!bulk_out)
						bulk_out = ep;
				}
				break;
			case USB_ENDPOINT_XFER_INT:
				if(usb_endpoint_dir_in(&ep->desc))
				{
					if(!int_in)
						int_in = ep;
				}
				break;
			default:
				break;
			}
		}
		/* the bulk in endpoint and bulk out endpoint must be exist */
		if(!(bulk_in && bulk_out))
			continue;

		if(intf->cur_altsetting->desc.bInterfaceProtocol == USB_PR_CBI)
		{
			if(!int_in)
				continue;
		}
		goto found;
	}

	return -EINVAL;

  found:
	info->control = &udev->ep0;
	info->rcv_ctl_pipe = usb_rcvctrlpipe(udev, 0);
	info->snd_ctl_pipe = usb_sndctrlpipe(udev, 0);
	info->bulk_in = bulk_in;
	info->bulk_in_pipe = usb_rcvbulkpipe(udev, usb_endpoint_num(&bulk_in->desc));
	info->bulk_out = bulk_out;
	info->bulk_out_pipe = usb_sndbulkpipe(udev, usb_endpoint_num(&bulk_out->desc));
	info->int_in = int_in;
	if(int_in)
		info->int_in_pipe = usb_rcvintpipe(udev, usb_endpoint_num(&int_in->desc));
	info->active_intf = intf;
	info->protocol = intf->cur_altsetting->desc.bInterfaceProtocol;

	return 0;
}

int read_cmd_str(struct xhci_ctler *ctler, unsigned int *cmd_code,
	unsigned int *route_string, unsigned int *request_code)
{
	return sscanf(ctler->test.cmd_str, "%d:0x%x:%d", cmd_code, route_string,
		request_code);
}

int check_num_parameter(struct xhci_ctler *ctler, int ret, int num_of_parameter)
{
	if(ret != num_of_parameter)
	{
		write_log(ctler,
			"Fail: Missing arguments, %d arguments required, but only %d received\n",
			num_of_parameter, ret);
		return -EINVAL;
	}
	return 0;
}

int check_cmd_code(struct xhci_ctler *ctler, unsigned cmd_code, unsigned CMD_CODE)
{
	if(cmd_code != CMD_CODE)
	{
		write_log(ctler,
			"Fail: Command code mismatch, expect cmd[%d], but receive cmd[%d]\n",
			CMD_CODE, cmd_code);
		return -EINVAL;
	}
	return 0;
}

int check_request_code(struct xhci_ctler *ctler, unsigned request_code,
	unsigned REQUEST_CODE)
{
	if(REQUEST_CODE != request_code)
	{
		write_log(ctler, "Fail: request code not support, receive request code[%d]\n",
			request_code);
		return -EINVAL;
	}
	return 0;
}

/*
 * This Route String is not the same as defination of route string in usb3.0 spec
 * the LSB and LSB+1 is root port index , the rest is route string.
 */
int check_route_string(struct xhci_ctler *ctler, unsigned int route_string)
{
	unsigned port_index;
	//unsigned temp;
	int ret = 0;
	port_index = route_string & 0xFF;
	if(route_string == 0 || port_index == 0)
	{
		write_log(ctler, "In check_route_string, Fail: route string[0x%x] invalid\n",
			route_string);
		ret = -EINVAL;
	}
	//temp = route_string;
	/*while(temp != 0)
	{
		port_index = temp & 0xFF;
		temp >>= 4;
		if(port_index == 0 && temp != 0)
		{
			write_log(ctler, "In check_route_string, Fail: route string[0x%x] invalid\n",
				route_string);
			ret = -EINVAL;
		}
	}*/
	return 0;
}

int check_endpoint(struct xhci_ctler *ctler, struct usb_host_endpoint *endpoint)
{
	if(endpoint == NULL)
	{
		write_log(ctler,
			"In check_endpoint,Fail: the target endpoint doesn't exist or enabled\n");
		return -EINVAL;
	}
	return 0;
}

int check_udev(struct xhci_ctler *ctler, struct usb_device *udev)
{
	if(udev == NULL)
	{
		write_log(ctler, "In check_udev,Fail: the target USB device doesn't exist\n");
		return -EINVAL;
	}
	return 0;
}

int check_hub_device(struct xhci_ctler *ctler, struct usb_device *udev)
{
	struct usb_interface *intf = NULL;

	intf = udev->actconfig->interface[0];

	if((udev->descriptor.bNumConfigurations != 1)
		|| (udev->actconfig->desc.bNumInterfaces != 1) || (intf == NULL))
	{
		write_log(ctler, "Fail: the target USB device is not Hub\n");
		return -EINVAL;
	}
	if(((udev->descriptor.bDeviceClass != USB_CLASS_HUB)
			&& (intf->cur_altsetting->desc.bInterfaceClass != USB_CLASS_HUB))
		|| (intf->cur_altsetting->desc.bNumEndpoints != 1))
	{
		write_log(ctler, "Fail: the target USB device is not Hub\n");
		return -EINVAL;
	}

	return 0;
}

int check_urb(struct xhci_ctler *ctler, struct urb *urb)
{
	if(urb == NULL)
	{
		write_log(ctler, "In check_urb,Fail: allocate urb failed\n");
		return -ENOMEM;
	}
	return 0;
}

int check_normal_table(struct xhci_ctler *ctler, struct xhci_mem_table *normal_table)
{
	if(normal_table == NULL)
	{
		write_log(ctler, "in check_normal_table,Fail: alloc normal mem_table failed\n");
		return -ENOMEM;
	}
	return 0;
}

int check_special_table(struct xhci_ctler *ctler, struct xhci_mem_table *special_table)
{
	if(special_table == NULL)
	{
		write_log(ctler, "in check_normal_table,Fail: alloc normal mem_table failed\n");
		return -ENOMEM;
	}
	return 0;
}

int check_null(void *p, char *s)
{
	if(p == NULL)
	{
		pr_debug("error: %s is null, check_null fail.\n", s);
		return -1;
	}
	else
		return 0;
}

char *err_str(int ret)
{
	switch (ret)
	{
	case -32:
		return "Broken Pipe";
	default:
		return "Unknow ret_val";
	}
}

void dump_field(unsigned field1, unsigned field2, unsigned field3, unsigned field4)
{
	pr_info("field1: 0x%x\n", field1);
	pr_info("field2: 0x%x\n", field2);
	pr_info("field3: 0x%x\n", field3);
	pr_info("field4: 0x%x\n", field4);
}



//adapted from hub.c: usb_hub_to_struct_hub 
struct usb_hub *hdev_to_struct_hub(struct usb_device *hdev)
{
	if(!hdev || !hdev->actconfig || !hdev->maxchild)
		return NULL;
	return usb_get_intfdata(hdev->actconfig->interface[0]);
}

int xhci_queue_stor_cmd_trb(struct xhci_ctler *ctler, struct usb_device *udev,
	struct usb_host_endpoint *ep, unsigned char *buffer, int length)
{
	int ep_index;
	struct xhci_ring *ep_ring;
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_hcd *xhci = ctler->xhci;

	int ret;

	u32 field, length_field;
	u64 addr;

	pr_info("enter into xhci_queue_stor_cmd_trb\n");
	ep_index = xhci_get_endpoint_index(&ep->desc);
	ep_ring = xhci->devs[udev->slot_id]->eps[ep_index].ring;
	ep_ctx = xhci_get_ep_ctx(xhci, xhci->devs[udev->slot_id]->out_ctx, ep_index);
	ret = prepare_ring(xhci, ep_ring, ep_ctx->ep_info & EP_STATE_MASK, 1, GFP_KERNEL);
	if(ret)
		return ret;

	/* How much data is in the first TRB? */
	addr =
		(u64) dma_map_single(ctler->hcd->self.controller, buffer, length, DMA_TO_DEVICE);
	field = 0;
	field |= ep_ring->cycle_state;
	length_field = TRB_LEN(length) | TRB_INTR_TARGET(0);
	queue_trb(xhci, ep_ring, false, lower_32_bits(addr), upper_32_bits(addr),
		length_field, field | TRB_ISP | TRB_TYPE(TRB_NORMAL) | TRB_IOC);

	return 0;
}

/*---------------Function about Mem--------------------*/
struct xhci_mem_table *xhci_verify_alloc_mem_table(unsigned int nents)
{
	struct xhci_mem_table *mem_tbl;

	mem_tbl = kzalloc(sizeof(struct xhci_mem_table) +
		(sizeof(struct xhci_mem_entry) * nents), GFP_KERNEL);
	if(mem_tbl)
		mem_tbl->nents = nents;

	return mem_tbl;
}

struct xhci_mem_table *xhci_verify_create_normal_mem_table(struct xhci_ctler *ctler,
	unsigned long normal_len)
{
	int i, j;
	unsigned long total_len = normal_len;
	unsigned char entry_num = 0;
	struct xhci_mem_table *table = NULL;
	struct xhci_mem_entry *entry = NULL;
	struct xhci_mem_pool *pool = NULL;

	/* if transfer length is 0, still construct the mem table, but entry number is zero */
	/*
	   if (total_len == 0)
	   return NULL;
	 */

	i = j = 0;
	while((i < MAX_POOL_ENTRY) && (total_len > 0))
	{
		pool = &(ctler->mem_pool[i]);
		j = 0;
		while((j < pool->actual_num) && (total_len > 0))
		{
			entry_num += 1;
			total_len -= (min(total_len, pool->length_in_page * PAGE_SIZE));
			j++;
		}
		i++;
	}
	if(total_len != 0)
		return NULL;

	table = xhci_verify_alloc_mem_table(entry_num);
	if(table == NULL)
		return NULL;

	if(normal_len == 0)
		return table;

	i = j = 0;
	entry = &(table->entry[0]);
	total_len = normal_len;
	while((i < MAX_POOL_ENTRY) && (total_len > 0))
	{
		pool = &(ctler->mem_pool[i]);
		j = 0;
		while((j < pool->actual_num) && (total_len > 0))
		{
			entry->page = pool->page_table[j];
			entry->high_mem = 0;
			entry->offset = 0;
			entry->length = (min(total_len, pool->length_in_page * PAGE_SIZE));
			entry->data_gen_type = get_random_int() % 3;
			entry->data = get_random_int();
			entry++;

			total_len -= (min(total_len, pool->length_in_page * PAGE_SIZE));
			j++;
		}
		i++;
	}
	return table;
}

struct xhci_mem_table *xhci_verify_create_special_mem_table(unsigned long special_len)
{
	unsigned int total_len = (unsigned int) special_len;
	unsigned char entry_num = 0;
	struct xhci_mem_table *table = NULL;
	struct xhci_mem_entry *entry;
	int i = ONE_MAX_PACKET_MEM_ENTRY_CNT - 1;

	while((total_len > 0) && (i >= 0))
	{
		total_len -= (min(one_max_packet_mem_entries[i].length, total_len));
		entry_num++;
		i--;
	}

	if(total_len != 0)
		return NULL;
	/*
	   if (entry_num == 0)
	   return NULL;
	 */

	table = xhci_verify_alloc_mem_table(entry_num);
	if(table == NULL)
		return NULL;
	if(special_len == 0)
		return table;

	i = ONE_MAX_PACKET_MEM_ENTRY_CNT - 1;
	total_len = special_len;
	entry = &(table->entry[entry_num - 1]);

	while((total_len > 0) && (i >= 0))
	{
		memcpy(entry, &one_max_packet_mem_entries[i],
			sizeof(one_max_packet_mem_entries[i]));
		entry->length = min(one_max_packet_mem_entries[i].length, total_len);
		total_len -= (min(one_max_packet_mem_entries[i].length, total_len));
		entry--;
		i--;
	}

	return table;
}

int xhci_verify_init_normal_mem_entry(struct xhci_mem_entry *mem_entry)
{
	unsigned char *page_addr;
	unsigned int offset;
	unsigned char data;
	unsigned int i;

	if(!mem_entry->page)
	{
		pr_info("!!!Error: the nomal table entry is NULL\n");
		return -ENOMEM;
	}

	page_addr = (unsigned char *) page_address(mem_entry->page);

	offset = mem_entry->offset;
	data = (unsigned char) mem_entry->data;

	switch (mem_entry->data_gen_type)
	{
	case 0:					/* fix */
		for(i = 0; i < mem_entry->length; i++)
			page_addr[offset++] = data;

		break;

	case 1:					/* increase */
		for(i = 0; i < mem_entry->length; i++)
			page_addr[offset++] = data++;

		break;

	case 2:					/* decrease */
		for(i = 0; i < mem_entry->length; i++)
			page_addr[offset++] = data--;

		break;

	default:
		return -EINVAL;
		break;
	}

	return 0;
}

int xhci_verify_init_special_mem_entry(struct xhci_mem_entry *mem_entry)
{
	unsigned char *page_addr;
	unsigned int offset;
	unsigned char data;
	unsigned int i;

	if(!mem_entry->page)
	{
		if(mem_entry->high_mem)
			mem_entry->page = alloc_page(GFP_KERNEL | __GFP_HIGHMEM);
		else
			mem_entry->page = alloc_page(GFP_KERNEL);
	}

	if(!mem_entry->page)
	{
		pr_info("Error: fail to allocate the special mem table entry\n");
		return -ENOMEM;
	}

	page_addr = kmap(mem_entry->page);

	offset = mem_entry->offset;
	data = (unsigned char) mem_entry->data;

	switch (mem_entry->data_gen_type)
	{
	case 0:					/* fix */
		for(i = 0; i < mem_entry->length; i++)
			page_addr[offset++] = data;

		break;

	case 1:					/* increase */
		for(i = 0; i < mem_entry->length; i++)
			page_addr[offset++] = data++;

		break;

	case 2:					/* decrease */
		for(i = 0; i < mem_entry->length; i++)
			page_addr[offset++] = data--;

		break;

	default:
		return -EINVAL;
		break;
	}

	kunmap(mem_entry->page);

	return 0;
}

int xhci_verify_reset_special_mem_entry_data(struct xhci_mem_entry *mem_entry)
{
	unsigned char *page_addr;
	unsigned int offset;

	if(!mem_entry->page)
	{
		pr_info("Error: special mem table entry has no buffer\n");
		return -ENOMEM;
	}

	page_addr = kmap(mem_entry->page);
	offset = mem_entry->offset;
	memset(page_addr + offset, 0, mem_entry->length);
	kunmap(mem_entry->page);

	return 0;
}

int xhci_verify_init_normal_mem_table(struct xhci_mem_table *mem_tbl)
{
	int ret;
	int i;

	for(i = 0; i < mem_tbl->nents; i++)
	{
		ret = xhci_verify_init_normal_mem_entry(&mem_tbl->entry[i]);
		if(ret)
			return ret;
	}

	return 0;
}

int xhci_verify_init_special_mem_table(struct xhci_mem_table *mem_tbl)
{
	int ret;
	int i;

	for(i = 0; i < mem_tbl->nents; i++)
	{
		ret = xhci_verify_init_special_mem_entry(&mem_tbl->entry[i]);
		if(ret)
			return ret;
	}

	return 0;
}

int xhci_verify_reset_special_mem_table_data(struct xhci_mem_table *mem_tbl)
{
	int ret;
	int i;

	for(i = 0; i < mem_tbl->nents; i++)
	{
		ret = xhci_verify_reset_special_mem_entry_data(&mem_tbl->entry[i]);
		if(ret)
			return ret;
	}

	return 0;
}

inline void xhci_verify_dump_normal_mem_entry(struct xhci_mem_entry *mem_entry)
{
	//unsigned char *page_addr;
	//unsigned int   offset;
	//int i;

	pr_info("page=0x%p\n", mem_entry->page);

	pr_info("high_mem=%s\n", mem_entry->high_mem ? "yes" : "no");

	pr_info("offset=%d\n", mem_entry->offset);

	pr_info("length=%d\n", mem_entry->length);

	pr_info("data=%d\n", (unsigned char) mem_entry->data);

	switch (mem_entry->data_gen_type)
	{
	case 0:
		pr_info("data_gen_type=fixed\n");
		break;
	case 1:
		pr_info("data_gen_type=increase\n");
		break;
	case 2:
		pr_info("data_gen_type=decrease\n");
		break;
	default:
		pr_info("data_gen_type=unknown\n");
		break;
	}

	if(!mem_entry->page)
		return;

	/*

	   pr_info( "contained data:\n");

	   page_addr = (unsigned char *)page_address(mem_entry->page);

	   offset = mem_entry->offset;

	   for (i = 0; i < mem_entry->length; i++)
	   pr_info( "%4d    ", page_addr[offset++]);

	   pr_info( "\n");
	 */

}

inline void xhci_verify_dump_special_mem_entry(struct xhci_mem_entry *mem_entry)
{
	//unsigned char *page_addr;
	//unsigned int   offset;
	//int i;

	pr_info("page=0x%p\n", mem_entry->page);

	pr_info("high_mem=%s\n", mem_entry->high_mem ? "yes" : "no");

	pr_info("offset=%d\n", mem_entry->offset);

	pr_info("length=%d\n", mem_entry->length);

	pr_info("data=%d\n", (unsigned char) mem_entry->data);

	switch (mem_entry->data_gen_type)
	{
	case 0:
		pr_info("data_gen_type=fixed\n");
		break;
	case 1:
		pr_info("data_gen_type=increase\n");
		break;
	case 2:
		pr_info("data_gen_type=decrease\n");
		break;
	default:
		pr_info("data_gen_type=unknown\n");
		break;
	}

	if(!mem_entry->page)
		return;

	/*

	   pr_info( "contained data:\n");

	   page_addr = kmap(mem_entry->page);

	   offset = mem_entry->offset;

	   for (i = 0; i < mem_entry->length; i++)
	   pr_info( "%4d    ", page_addr[offset++]);

	   pr_info( "\n");

	   kunmap(mem_entry->page);
	 */
}

inline void xhci_verify_dump_normal_mem_table(struct xhci_mem_table *mem_tbl)
{
	int i;

	pr_info("Normal Table Total %d mem entries\n", mem_tbl->nents);

	for(i = 0; i < mem_tbl->nents; i++)
	{
		pr_info("\nInfo for entry[%d]:\n", i);
		xhci_verify_dump_normal_mem_entry(&mem_tbl->entry[i]);
	}
}

inline void xhci_verify_dump_special_mem_table(struct xhci_mem_table *mem_tbl)
{
	int i;

	pr_info("--------Special Table Total %d mem entries--------\n", mem_tbl->nents);

	for(i = 0; i < mem_tbl->nents; i++)
	{
		pr_info("\nInfo for entry[%d]:\n", i);
		xhci_verify_dump_special_mem_entry(&mem_tbl->entry[i]);
	}
}

int xhci_verify_normal_mem_entry(struct xhci_mem_entry *mem_entry)
{
	unsigned char *page_addr;
	unsigned int offset;
	unsigned char data;
	unsigned int i;

	if(!mem_entry->page)
		return -ENOMEM;

	page_addr = (unsigned char *) page_address(mem_entry->page);

	offset = mem_entry->offset;
	data = (unsigned char) mem_entry->data;

	switch (mem_entry->data_gen_type)
	{
	case 0:					/* fix */
		for(i = 0; i < mem_entry->length; i++)
			if(page_addr[offset++] != data)
				return -ENOTSYNC;

		break;

	case 1:					/* increase */
		for(i = 0; i < mem_entry->length; i++)
			if(page_addr[offset++] != (data++))
				return -ENOTSYNC;

		break;

	case 2:					/* decrease */
		for(i = 0; i < mem_entry->length; i++)
			if(page_addr[offset++] != (data--))
				return -ENOTSYNC;

		break;

	default:
		return -EINVAL;
		break;
	}

	return 0;
}

int xhci_verify_special_mem_entry(struct xhci_mem_entry *mem_entry)
{
	unsigned char *page_addr;
	unsigned int offset;
	unsigned char data;
	unsigned int i;

	if(!mem_entry->page)
		return -ENOMEM;

	page_addr = kmap(mem_entry->page);

	offset = mem_entry->offset;
	data = (unsigned char) mem_entry->data;

	switch (mem_entry->data_gen_type)
	{
	case 0:					/* fix */
		for(i = 0; i < mem_entry->length; i++)
			if(page_addr[offset++] != data)
				return -ENOTSYNC;

		break;

	case 1:					/* increase */
		for(i = 0; i < mem_entry->length; i++)
			if(page_addr[offset++] != (data++))
				return -ENOTSYNC;

		break;

	case 2:					/* decrease */
		for(i = 0; i < mem_entry->length; i++)
			if(page_addr[offset++] != (data--))
				return -ENOTSYNC;

		break;

	default:
		return -EINVAL;
		break;
	}

	kunmap(mem_entry->page);

	return 0;
}

int xhci_verify_normal_mem_table(struct xhci_mem_table *mem_tbl)
{
	int ret;
	int i;

	for(i = 0; i < mem_tbl->nents; i++)
	{
		ret = xhci_verify_normal_mem_entry(&mem_tbl->entry[i]);
		if(ret)
			return ret;
	}

	return 0;
}

int xhci_verify_special_mem_table(struct xhci_mem_table *mem_tbl)
{
	int ret;
	int i;

	for(i = 0; i < mem_tbl->nents; i++)
	{
		ret = xhci_verify_special_mem_entry(&mem_tbl->entry[i]);
		if(ret)
			return ret;
	}

	return 0;
}

int xhci_verify_copy_special_mem_table(struct xhci_mem_table *mem_tbl,
	unsigned char *dst, unsigned int length)
{
	unsigned char *page_addr;
	unsigned int offset;
	int i = 0;
	unsigned int total_len = length;
	unsigned int tmp_len;
	unsigned char *tmp_addr = dst;

	while(total_len > 0 && i < mem_tbl->nents)
	{
		if(!mem_tbl->entry[i].page)
		{
			pr_info("Error: special mem table entry has no buffer\n");
			return -ENOMEM;
		}

		page_addr = kmap(mem_tbl->entry[i].page);
		offset = mem_tbl->entry[i].offset;
		tmp_len = min(total_len, mem_tbl->entry[i].length);
		total_len -= tmp_len;
		memcpy(tmp_addr, page_addr + offset, tmp_len);
		tmp_addr += tmp_len;
		kunmap(mem_tbl->entry[i].page);
		i++;
	}

	if(total_len > 0)
	{
		pr_info("Error: special mem table has no enough buffer\n");
		return -ENOTSYNC;
	}

	return 0;
}

void xhci_verify_free_normal_mem_table(struct xhci_mem_table *mem_tbl)
{
	kfree(mem_tbl);
}

void xhci_verify_free_special_mem_table(struct xhci_mem_table *mem_tbl)
{
	int i;

	for(i = 0; i < mem_tbl->nents; i++)
		if(mem_tbl->entry[i].page)
			__free_page(mem_tbl->entry[i].page);

	kfree(mem_tbl);
}

int xhci_verify_init_mem_pool(struct xhci_ctler *ctler)
{
	struct xhci_mem_pool *pool = NULL;
	//unsigned long page;
	struct page *page;
	int i, j;

	pool = (struct xhci_mem_pool *)
		kzalloc(sizeof(struct xhci_mem_pool) * MAX_POOL_ENTRY, GFP_KERNEL);
	if(!pool)
		return -ENOMEM;
	
	ctler->mem_pool = pool;

	for(i = 0; i < MAX_POOL_ENTRY; i++)
	{
		ctler->mem_pool[i].order = (MAX_POOL_ENTRY - 1 - i);
		ctler->mem_pool[i].length_in_page = (1 << (MAX_POOL_ENTRY - 1 - i));
	}
	ctler->mem_pool[MAX_POOL_ENTRY - 1].target_num = 10;
	ctler->mem_pool[0].target_num = 8;	/* 8*16 pages max, 512K */

	for(i = 0; i < MAX_POOL_ENTRY; i++)
	{
		pool = &(ctler->mem_pool[i]);
		for(j = 0; j < pool->target_num; j++)
		{
			//page = __get_free_pages(GFP_KERNEL, pool->order);
			page = alloc_pages(GFP_KERNEL, pool->order);
			//if (page == 0) {
			if(page == NULL)
			{
				if(pool->order == 0)
					goto free_mem_pool;
				ctler->mem_pool[i + 1].target_num +=
					(pool->target_num - pool->actual_num) * 2;
				break;
			}
			else
			{
				//pool->page_table[j] = virt_to_page(page);
				pool->page_table[j] = page;
				pool->actual_num += 1;
			}
		}
	}

	return 0;

  free_mem_pool:
	for(i = 0; i < MAX_POOL_ENTRY; i++)
	{
		pool = &(ctler->mem_pool[i]);
		for(j = 0; j < pool->actual_num; j++)
		{
			if(pool->page_table[j] != NULL)
				__free_pages(pool->page_table[j], pool->order);
		}
	}
	kfree(ctler->mem_pool);

	return -ENOMEM;

}

int xhci_verify_free_mem_pool(struct xhci_ctler *ctler)
{
	struct xhci_mem_pool *pool = NULL;
	int i, j;

	for(i = 0; i < MAX_POOL_ENTRY; i++)
	{
		pool = &(ctler->mem_pool[i]);
		for(j = 0; j < pool->actual_num; j++)
		{
			if(pool->page_table[j] != NULL)
				__free_pages(pool->page_table[j], pool->order);
		}
	}
	kfree(ctler->mem_pool);

	return 0;

}

int xhci_verify_reset_mem_pool_data(struct xhci_ctler *ctler)
{
	int i, j;
	unsigned char *addr;
	struct page *page;
	struct xhci_mem_pool *pool = NULL;

	for(i = 0; i < MAX_POOL_ENTRY; i++)
	{
		pool = &(ctler->mem_pool[i]);
		for(j = 0; j < pool->actual_num; j++)
		{
			page = pool->page_table[j];
			if(page != 0)
			{
				addr = (unsigned char *) page_address(page);
				memset(addr, 0, pool->length_in_page * PAGE_SIZE);
			}
		}
	}
	return 0;
}
 