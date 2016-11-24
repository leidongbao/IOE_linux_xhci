/*
 * xhci_ctler_verify.c 	- VIA xHCI controller verification
 *			  			 (C) 2010-2011 VIA Tech Inc
 * Histroy:
 *
 * 2011-04-03: create this driver based on the VIA SATA/AHCI controller
 * driver, which is developed by Louis Qi
 *
 * 2015-01-23: adapted to linux-3.16 by Jeff Zhao
 *
 */

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
#include <linux/usb/storage.h>

#include <linux/dmapool.h>
#include <linux/string.h>
#include <linux/irq.h>

#include "xhci.h"
#include "xhci_ctler_verify.h"
#include "xhci_ctler_common.h"
#include "xhci-ring.h"
#include "xhci-dbg.h"
#include "xhci-mem.h"
#include "xhci_cmd_ring.h"
#include "hub.h"
/*weitao define in order to use in cnd001&&IOE*/
#define	xhci11_verify 123

unsigned int tag = 0;

static unsigned int xhci_ctler_index = 0;
static LIST_HEAD(xhci_ctler_list);
static DEFINE_SPINLOCK(xhci_ctler_list_lock);

void xhci_verify_urb_complete_fn(struct urb *urb)
{
	struct completion *urb_done_ptr = urb->context;
	pr_info("URB Complete, call complete function\n");
	complete(urb_done_ptr);
}

/*
 * if the transfer has been completed successfully,
 * we still need verify the data later.
 *
 */
int xhci_verify_parse_result(struct xhci_ctler *ctler, struct urb *urb)
{
	int result = urb->status;
	unsigned int length = urb->transfer_buffer_length;
	unsigned int partial = urb->actual_length;

	switch (result)
	{
	case 0:
		if((partial != length) && (urb->transfer_flags & URB_SHORT_NOT_OK))
		{
			write_log(ctler, "Fail: only transfer partial data\n");
			return -ENODATA;
		}
		return 0;
	/* stalled */
	case -EPIPE:
		write_log(ctler, "Fail: stall on pipe\n");
		return -EPIPE;
	/* babble - the device tried to send more than we wanted to read */
	case -EOVERFLOW:
		write_log(ctler, "Fail: babble occurs\n");
		return -EOVERFLOW;
	/* the transfer was cancelled by abort, disconnect, or timeout */
	case -ECONNRESET:
		write_log(ctler, "Fail: transfer cancelled\n");
		return -ECONNRESET;
	/* short scatter-gather read transfer */
	case -EREMOTEIO:
		write_log(ctler, "Fail: short scatter gather read transfer\n");
		return -EREMOTEIO;
	/* endpoint shutdown */
	case -ESHUTDOWN:
		write_log(ctler, "Fail: cannot send after transport endpoint shutdown\n");
		return -ESHUTDOWN;
	/* abort or disconnect in progress */
	case -EIO:
		write_log(ctler, "Fail: device disconnect\n");
		return -EIO;
	/* the catch-all error case */
	default:
		write_log(ctler, "Fail: Unknown error\n");
		
		return result;
	}
}

int xhci_verify_stor_transport(struct xhci_ctler *ctler,
	struct usb_device *udev, struct mass_storage_dev_info *mass_stor_info,
	unsigned int start_sector, unsigned int sector_cnt,
	struct xhci_mem_table *normal_table, struct xhci_mem_table *special_table,
	unsigned char is_write, unsigned char is_event_full_verify,
	unsigned char is_zero_len_trb)
{
	int ret = 0;
	int actual_len;
	unsigned char *scsi_cmd, *status_buffer;
	unsigned transfer_length = sector_cnt * 512;

	struct urb *urb;
	struct bulk_cb_wrap *bcb = NULL;
	struct bulk_cs_wrap *bcs = NULL;
	struct xhci_hcd *xhci = ctler->xhci;

	pr_info("\n**************************start******************************\n");
	pr_info("enter xhci_verify_stor_transport");
	pr_info("is_write: %d, is_event_full: %d, is_zero_len_trb: %d\n", is_write,
		is_event_full_verify, is_zero_len_trb);

	xhci->is_event_full_happen = 0;

	/* Construct the cdb, add 2 bytes for status stage buffer for CBI */
	scsi_cmd = (unsigned char *) kzalloc(10 + 2, GFP_KERNEL);
	if(check_null(scsi_cmd, "scsi_cmd"))
		goto out;

	if(is_write)
		scsi_cmd[0] = WRITE_10;
	else
		scsi_cmd[0] = READ_10;

	if(is_zero_len_trb)
	{
		if(is_write)
		{
			sector_cnt += 2;
			transfer_length = sector_cnt * 512;
		}
	}
	scsi_cmd[1] = 0;
	scsi_cmd[2] = (unsigned char)(start_sector >> 24) & 0xff;
	scsi_cmd[3] = (unsigned char)(start_sector >> 16) & 0xff;
	scsi_cmd[4] = (unsigned char)(start_sector >> 8) & 0xff;
	scsi_cmd[5] = (unsigned char)start_sector & 0xff;
	scsi_cmd[6] =  scsi_cmd[9] = 0;
	scsi_cmd[7] = (unsigned char) (sector_cnt >> 8) & 0xff;
	scsi_cmd[8] = (unsigned char) sector_cnt & 0xff;

	/* Command stage */
	pr_info("command stage begin\n");
	if(mass_stor_info->protocol == USB_PR_CB || mass_stor_info->protocol == USB_PR_CBI)
	{
		ret = usb_control_msg(udev, mass_stor_info->bulk_out_pipe, 0,
				USB_TYPE_CLASS | USB_RECIP_INTERFACE, 0,
			mass_stor_info->active_intf->cur_altsetting->desc.bInterfaceNumber, scsi_cmd,
			10, 0);

		if(ret < 0 || ret != 10)
		{
			ret = -EPIPE;
			pr_info("ret is %d\n", ret);
			write_log(ctler, "Fail: transfer the command fail\n");
			goto free_scsi_cmd;
		}
	}
	else if(mass_stor_info->protocol == USB_PR_BULK)
	{
		/* allocate the bcb and bcs together */
		bcb = (struct bulk_cb_wrap *)kzalloc(sizeof(struct bulk_cb_wrap) +
					sizeof(struct bulk_cs_wrap), GFP_KERNEL);
		
		if(bcb == NULL)
		{
			ret = -ENOMEM;
			write_log(ctler, "Fail: alloc bulk cb wrap failed\n");
			goto free_scsi_cmd;
		}
		/* set up the command wrapper */
		bcb->Signature 			= cpu_to_le32(US_BULK_CB_SIGN);
		bcb->Tag 				= ++tag;
		bcb->DataTransferLength = cpu_to_le32(transfer_length);
		if(is_write)
			bcb->Flags = 0;		/* for read, it is 1 << 7 */
		else
			bcb->Flags = 1 << 7;
		bcb->Lun = 0;
		bcb->Length = 10;
		memset(bcb->CDB, 0, sizeof(bcb->CDB));
		memcpy(bcb->CDB, scsi_cmd, bcb->Length);

		/* ret==0 means successful, US_BULK_CB_WRAP_LEN == 31 */
		ret = usb_bulk_msg(udev, mass_stor_info->bulk_out_pipe, 
				bcb, US_BULK_CB_WRAP_LEN, &actual_len, 0);
		pr_info("ret is %d, transfer length is %d\n", ret, actual_len);
		//if(ret < 0 || actual_len != US_BULK_CB_WRAP_LEN)
		if(ret)
		{
			pr_info("ret = %d\n", ret);
			write_log(ctler, "Fail: transfer the command fail\n");
			pr_err("Fail: transfer the command fail, %s\n", err_str(ret));
			goto free_cbw;
		}
		pr_info("command stage completed\n");
	}

	if(is_zero_len_trb)
	{
		if(is_write)
		{
			sector_cnt -= 2;
			transfer_length = sector_cnt * 512;
		}
	}

	/* Data stage */
	pr_info("data stage begin\n");

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if(check_urb(ctler, urb))
		goto out;

	urb->num_sgs = normal_table->nents + special_table->nents;
	urb->sg = ctler->sgtable->sgl;
	urb->transfer_buffer = NULL;

	urb->dev = udev;
	if(is_write)
	{
		urb->ep = mass_stor_info->bulk_out;
		urb->pipe = mass_stor_info->bulk_out_pipe;
	}
	else
	{
		urb->ep = mass_stor_info->bulk_in;
		urb->pipe = mass_stor_info->bulk_in_pipe;
	}
	urb->stream_id = 0;
	//urb->transfer_flags = URB_SHORT_NOT_OK;
	//urb->transfer_flags = URB_ZERO_PACKET;

	if(is_zero_len_trb)
		urb->transfer_buffer_length = 0;
	else
		urb->transfer_buffer_length = transfer_length;

	urb->complete = xhci_verify_urb_complete_fn;
	urb->context = &ctler->test.xhci_urb_completion;
	init_completion(&ctler->test.xhci_urb_completion);

	if(is_event_full_verify)
	{
		pr_info("before queue the trb, the transfer ring information:\n");
		print_transfer_ring(xhci, udev, urb->ep);
		pr_info("before queue the trb, the event ring information:\n");
		print_event_ring(xhci);
	}

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if(ret < 0)
	{
		write_log(ctler, "Fail: submit URB failed\n");
		goto free_urb;
	}
	pr_info("data stage urb submited\n");

	if(is_event_full_verify)
	{
		pr_info("after queue the trb, the transfer ring information:\n");
		print_transfer_ring(xhci, udev, urb->ep);
	}

	/* we don't know how long this transfer needs.
	 * when we verify the event full, ISR will complete this completion
	 * when the event full TRB encounter, but the URB has not been
	 * completed. then we re-init this completion and wait it, because
	 * the ISR will complete it another time when this URB is completed
	 */
	if(is_event_full_verify)
	{
		if(!wait_for_completion_timeout(&ctler->test.xhci_urb_completion, 20 * HZ))
		{
			pr_info("good, the event full event happends\n");
			pr_info("after queue the trb, the event ring information:\n");
			print_event_ring(xhci);
		}
		else
		{
			write_log(ctler, "Fail: the event full event doesn't happen at all\n");
			pr_info("after queue the trb, the event ring information:\n");
			print_event_ring(xhci);
			xhci->is_event_full_happen = 0;
			ret = -EPIPE;
			goto free_urb;
		}
	}
	else
		wait_for_completion(&ctler->test.xhci_urb_completion);

	if(is_event_full_verify)
		pr_info("good, the event full happens\n");
	else
		pr_info("data stage completed\n");

	if(is_event_full_verify)
	{
		u64 temp_64;
		dma_addr_t deq;

		init_completion(&ctler->test.xhci_urb_completion2);
		urb->context = &ctler->test.xhci_urb_completion2;

		/* I think 2 seconds is enough to transfer left  */
		if(!wait_for_completion_timeout(&ctler->test.xhci_urb_completion2, 2 * HZ))
		{
			pr_info("good, the left urb is not executed at all\n");
		}
		else
		{
			write_log(ctler,
				"Fail: the left urb is executed when the event ring is full\n");
			ret = -EPIPE;
			goto free_urb;
		}

		xhci->is_event_full_happen = 0;
		init_completion(&ctler->test.xhci_urb_completion2);
		urb->context = &ctler->test.xhci_urb_completion2;

		/* update the dequeue pointer of event ring to trigger xHCI to continue
		 * and the target value is stored in enqueue pointer
		 */
		temp_64 = xhci_read_64(xhci, &xhci->ir_set->erst_dequeue);
		deq = xhci_trb_virt_to_dma(xhci->event_ring->deq_seg, xhci->event_ring->dequeue);
		if(deq == 0)
			pr_info("WARN something wrong with SW event ring dequeue ptr.\n");
		/* Update HC event ring dequeue pointer */
		temp_64 &= ERST_PTR_MASK;
		temp_64 |= ((u64) deq & (u64) ~ ERST_PTR_MASK);

		/* Clear the event handler busy flag (RW1C); event ring is empty. */
		temp_64 |= ERST_EHB;
		xhci_write_64(xhci, temp_64, &xhci->ir_set->erst_dequeue);

		//wait_for_completion(&ctler->test.xhci_urb_completion2);
		
		if(!wait_for_completion_timeout(&ctler->test.xhci_urb_completion2, 5*HZ))
		{
			pr_info("Time out for xhci_urb_completion2\n");
		}
		else
		{
			pr_info("good, the left TRB is continued to be executed\n");
		}

	}

	ret = xhci_verify_parse_result(ctler, urb);
	pr_info("xhci_verify_parse_result ret: %d\n", ret);
	if(ret)
	{
		if(is_event_full_verify && ret == -ESHUTDOWN)
		{
			ret=0;
			write_log(ctler, "Sucess: event full really happens\n");
		}
		else
		{
			ret = -EIO;
			write_log(ctler, "Fail: data stage transfer error\n");
		}
		goto free_urb;
	}
	else
	{
		if(is_zero_len_trb)
		{
			pr_info("urb->status is 0, means successful\n");
			pr_info("do not verify data for zero_length_trb\n");
			ret = 0;
			goto out;
		}
	}
	
	msleep(100);
	/* Status stage for write */
	pr_info("status stage begin\n");
	if(mass_stor_info->protocol == USB_PR_CBI)
	{
		status_buffer = scsi_cmd + 10;
		ret = usb_bulk_msg(udev, mass_stor_info->int_in_pipe, status_buffer, 2,
			&actual_len, 0);
		pr_info("usb_bulk_msg1 ret: %d\n", ret);

		if(ret < 0 || actual_len != 2)
		{
			ret = -EPIPE;
			write_log(ctler, "Fail: status stage transfer fail\n");
			goto free_urb;
		}

		if(status_buffer[0] != 0 || status_buffer[1] != 0)
		{
			ret = -EPIPE;
			write_log(ctler, "Fail: status shows command fail\n");
			goto free_urb;
		}
	}
	else if(mass_stor_info->protocol == USB_PR_BULK)
	{
		unsigned char *tmp;
		tmp = (unsigned char *) bcb;
		bcs = (struct bulk_cs_wrap *) (tmp + sizeof(*bcb));

		memset(bcs, 0, sizeof(*bcs));

		ret = usb_bulk_msg(udev, mass_stor_info->bulk_in_pipe, 
				bcs, US_BULK_CS_WRAP_LEN, &actual_len, 0);
			
		pr_info("status stage, usb_bulk_msg2 ret: %d\n", ret);
		if(ret < 0)
		{
			ret = -EPIPE;
			write_log(ctler, "Fail: status stage transfer fail\n");
			goto free_urb;
		}

		if((bcs->Signature != cpu_to_le32(US_BULK_CS_SIGN))
			|| (bcs->Status != US_BULK_STAT_OK) || (bcs->Tag != tag)
			|| (bcs->Residue != 0))
		{
			ret = -EPIPE;
			write_log(ctler, "Fail: status shows command fail\n");
			goto free_urb;
		}
	}
	else
	{
		pr_info("mass_stor_info->protocol is not bulk only!\n");
	}
	ret = 0;
	pr_info("status stage complete\n");

  free_urb:
	if(urb != NULL)
		usb_free_urb(urb);

  free_cbw:
	if(bcb != NULL)
		kfree(bcb);

  free_scsi_cmd:
	if(scsi_cmd != NULL)
		kfree(scsi_cmd);

  out:
  	pr_info("get out of xhci_verify_stor_transport\n");
	pr_info("**************************end********************************\n");
	return ret;
}

int xhci_verify_stor_transport_with_setdeq(struct xhci_ctler *ctler,
	struct usb_device *udev, struct mass_storage_dev_info *mass_stor_info)
{
	int i, ret;
	unsigned char *scsi_cmd = NULL;
	struct bulk_cb_wrap *bcb = NULL;
	struct xhci_hcd *xhci = ctler->xhci;

	unsigned int start_sector;
	unsigned int sector_cnt;
	struct xhci_mem_table *normal_table, *special_table;
	struct scatterlist *sg;
	struct xhci_command *command = NULL;
	unsigned int transfer_length;

	int ep_index;
	struct xhci_ring *ep_ring;

	unsigned long flags;
	dma_addr_t addr;

	pr_info("enter into xhci_verify_stor_transport_with_setdeq\n");
	for(i = 0; i < MAX_POOL_ENTRY; i++)
	{
		if(ctler->mem_pool[i].actual_num > 0)
		{
			transfer_length = ctler->mem_pool[i].length_in_page * PAGE_SIZE;
			break;
		}
	}
	if(i == MAX_POOL_ENTRY)
	{
		write_log(ctler, "Fail: there is no valid mem pool entry\n");
		ret = -ENOMEM;
		goto out;
	}

	normal_table = xhci_verify_create_normal_mem_table(ctler, transfer_length);
	if(normal_table->nents != 1 || normal_table->entry[0].length != transfer_length)
	{
		write_log(ctler, "Fail: nomal mem table invalid\n");
		ret = -ENOMEM;
		goto free_normal_table;
	}
	xhci_verify_reset_mem_pool_data(ctler);
	xhci_verify_init_normal_mem_table(normal_table);

	special_table = xhci_verify_create_special_mem_table(0);
	if(special_table == NULL)
	{
		write_log(ctler, "Fail: alloc special mem_table failed\n");
		ret = -ENOMEM;
		goto free_normal_table;
	}
	xhci_verify_init_special_mem_table(special_table);

	if(unlikely(sg_alloc_table(ctler->sgtable,
				normal_table->nents + special_table->nents, GFP_KERNEL)))
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc sg table failed\n");
		goto free_special_table;
	}

	for_each_sg(ctler->sgtable->sgl, sg, normal_table->nents, i) sg_set_page(sg,
		normal_table->entry[i].page, normal_table->entry[i].length,
		normal_table->entry[i].offset);
	for_each_sg(ctler->sgtable->sgl + normal_table->nents, sg, special_table->nents,
		i) sg_set_page(sg, special_table->entry[i].page, special_table->entry[i].length,
		special_table->entry[i].offset);

	ep_index = xhci_get_endpoint_index(&mass_stor_info->bulk_out->desc);
	ep_ring = xhci->devs[udev->slot_id]->eps[ep_index].ring;
	if(ep_ring == NULL)
	{
		write_log(ctler, "Fail: the bulk out endpoint has no transfer ring\n");
		ret = -EINVAL;
		goto out;
	}

	/* according Spec, the set dequeue pointer command need endpoint stop or error */
	init_completion(&xhci->cmd_verify_complet);
	xhci->is_cmd_verify = 1;
	spin_lock_irqsave(&xhci->lock, flags);

	pr_info("prepare to queue command TRB_STOP_RING\n");
	command = xhci_alloc_command(xhci, false, false, GFP_ATOMIC);
	if(!command)
		return -1;
	ret = queue_command(xhci, command, 0, 0, 0, SLOT_ID_FOR_TRB(udev->slot_id)
		| EP_ID_FOR_TRB(ep_index) | TRB_TYPE(TRB_STOP_RING), false);
	if(ret < 0)
	{
		write_log(ctler, "Fail: fail to issue the set dequeue command\n");
		spin_unlock_irqrestore(&xhci->lock, flags);
		goto free_cbw;
	}
	pr_info("done to queue command TRB_STOP_RING\n");
	xhci_ring_cmd_db(xhci);
	spin_unlock_irqrestore(&xhci->lock, flags);

	/* I think 10 seconds is enough to transfer left  */
	ret = wait_for_completion_timeout(&xhci->cmd_verify_complet, 10 * HZ);

	if(ret == 0)
	{
		write_log(ctler, "Fail: the NO OP command timeout(10s)\n");
		ret = -EPIPE;
		goto free_sg_table;
	}
	else
	{
		pr_info("setdeq1:the target command executed successfully\n");
	}
	xhci->is_cmd_verify = 0;

	/* construct a command bulk out TRB first. this TRB will be discarded */
	sector_cnt = transfer_length / 512;
	start_sector = 160;

	scsi_cmd = (unsigned char *) kzalloc(10, GFP_KERNEL);
	if(scsi_cmd == NULL)
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc scsi command failed\n");
		goto out;
	}
	scsi_cmd[0] = WRITE_10;
	scsi_cmd[1] = 0;
	scsi_cmd[2] = (unsigned char) (start_sector >> 24) & 0xff;
	scsi_cmd[3] = (unsigned char) (start_sector >> 16) & 0xff;
	scsi_cmd[4] = (unsigned char) (start_sector >> 8) & 0xff;
	scsi_cmd[5] = (unsigned char) start_sector & 0xff;
	scsi_cmd[6] = scsi_cmd[9] = 0;
	scsi_cmd[7] = (unsigned char) (sector_cnt >> 8) & 0xff;
	scsi_cmd[8] = (unsigned char) sector_cnt & 0xff;

	/* allocate the bcb and bcs together */
	bcb =
		(struct bulk_cb_wrap *) kzalloc(sizeof(struct bulk_cb_wrap) +
		sizeof(struct bulk_cs_wrap), GFP_KERNEL);
	if(bcb == NULL)
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc bulk cb wrap failed\n");
		goto free_scsi_cmd;
	}
	/* set up the command wrapper */
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = cpu_to_le32(transfer_length);

	bcb->Flags = 0;				//for read, it is 1 << 7
	bcb->Tag = ++tag;
	tag--;
	bcb->Lun = 0;
	bcb->Length = 10;
	memset(bcb->CDB, 0, sizeof(bcb->CDB));
	memcpy(bcb->CDB, scsi_cmd, bcb->Length);
	ret =
		xhci_queue_stor_cmd_trb(ctler, udev, mass_stor_info->bulk_out,
		(unsigned char *) bcb, sizeof(*bcb));
	if(ret != 0)
	{
		ret = -EPIPE;
		write_log(ctler, "Fail: fail to queue the fake command TRB\n");
		goto free_cbw;
	}

	/* then, set the dequeue pointer */
	init_completion(&xhci->cmd_verify_complet);
	xhci->is_cmd_verify = 1;
	spin_lock_irqsave(&xhci->lock, flags);
	addr = xhci_trb_virt_to_dma(ep_ring->enq_seg, ep_ring->enqueue);

	pr_info("prepare to queue command TRB_SET_DEQ\n");
	command = xhci_alloc_command(xhci, false, false, GFP_ATOMIC);
	if(!command)
		return -1;
	ret = queue_command(xhci, command, lower_32_bits(addr) | ep_ring->cycle_state,
		upper_32_bits(addr), 0,
		SLOT_ID_FOR_TRB(udev->slot_id) | EP_ID_FOR_TRB(ep_index) | TRB_TYPE(TRB_SET_DEQ),
		false);
	if(ret < 0)
	{
		write_log(ctler, "Fail: fail to issue the set dequeue command\n");
		spin_unlock_irqrestore(&xhci->lock, flags);
		goto free_cbw;
	}
	pr_info("done to queue command TRB_SET_DEQ\n");
	xhci_ring_cmd_db(xhci);
	spin_unlock_irqrestore(&xhci->lock, flags);

	/* I think 10 seconds is enough to transfer left  */
	if(!wait_for_completion_timeout(&xhci->cmd_verify_complet, 10 * HZ))
	{
		write_log(ctler, "Fail: the NO OP command timeout(10s)\n");
		ret = -EPIPE;
		goto free_cbw;
	}
	else
	{
		pr_info("setdeq2:the target command executed successfully\n");
	}
	xhci->is_cmd_verify = 0;

	/* write the buffer to USB mass storage device */
	ret = xhci_verify_stor_transport(ctler, udev, mass_stor_info, 260, sector_cnt,
		normal_table, special_table, 1, 0, 0);
	if(ret < 0)
	{
		write_log(ctler, "Fail: fail to write data to usb disk\n");
		goto free_cbw;
	}

	xhci_verify_reset_mem_pool_data(ctler);
	xhci_verify_reset_special_mem_table_data(special_table);

	/* read the data back with the same buffer, which has been reset */
	ret =
		xhci_verify_stor_transport(ctler, udev, mass_stor_info, 260, sector_cnt,
		normal_table, special_table, 0, 0, 0);
	if(ret < 0)
	{
		write_log(ctler, "Fail: fail to read data from usb disk\n");
		goto free_cbw;
	}

	/* verify the data now */
	if((xhci_verify_normal_mem_table(normal_table) != 0)
		|| (xhci_verify_special_mem_table(special_table) != 0))
	{
		ret = -ENOTSYNC;
		write_log(ctler, "Fail: data verify fail\n");
		goto free_cbw;
	}

	if(ret == 0)
		write_log(ctler, "Success\n");

  free_cbw:
	if(bcb != NULL)
		kfree(bcb);

  free_scsi_cmd:
	if(scsi_cmd != NULL)
		kfree(scsi_cmd);

  free_sg_table:
	sg_free_table(ctler->sgtable);

  free_special_table:
	if(special_table != NULL);
	xhci_verify_free_special_mem_table(special_table);

  free_normal_table:
	if(normal_table != NULL)
		xhci_verify_free_normal_mem_table(normal_table);

  out:
	return ret;
}

/* Synopsis:
 * cmd_code:xhci_cmd:route_string:endpoint_index
 * %d:%d:0x%x:0x%x
 * for get port bandwidth command, the endpoint_index describes the speed, 
 * and the route_string describes the target location, 0 is for root hub.
 */
int cmd_verify(struct xhci_ctler *ctler)
{
	int i, ret;
	unsigned char ep_num = 0;
	unsigned char direction = 0;
	unsigned cmd_code, xhci_cmd, route_string, endpoint_index;

	struct usb_device *udev = NULL;
	struct usb_host_endpoint *ep = NULL;
	struct xhci_hcd *xhci = ctler->xhci;
	struct xhci_command *command = NULL;

	unsigned int value;
	unsigned long flags;
	int buf_len = 0;
	unsigned char *buf = NULL;
	dma_addr_t buf_dma;

	unsigned int rs_valid = 0;
	unsigned int ep_valid = 0;

	unsigned int field1, field2, field3, field4;
	int ep_index = 0;
	
	struct mass_storage_dev_info mass_stor_info;

	ret = sscanf(ctler->test.cmd_str, "%d:%d:0x%x:0x%x",
		&cmd_code, &xhci_cmd, &route_string, &endpoint_index);

	if(check_num_parameter(ctler, ret, 4))
		goto out;

	if(check_cmd_code(ctler, cmd_code, CMD_VERIFY))
		goto out;

	if(xhci_cmd < TRB_ENABLE_SLOT || xhci_cmd > TRB_CMD_NOOP)
	{
		write_log(ctler, "Fail: the xhci command code invalid\n");
		ret = -EINVAL;
		goto out;
	}

	field1 = field2 = field3 = field4 = 0;

	rs_valid = check_route_string_valid(xhci_cmd);
	ep_valid = check_endpoint_valid(xhci_cmd);

	if(rs_valid)
	{
		if(check_route_string(ctler, route_string))
			goto out;
		udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
		if(check_udev(ctler, udev))
			goto out;
	}
	if(ep_valid)
	{
		direction = ((endpoint_index & USB_DIR_IN) >> 7);
		ep_num = endpoint_index & 0x0F;
		ep = xhci_verify_get_ep_by_address(udev, direction, ep_num);
		if(check_endpoint(ctler, ep))
			goto out;
	}

	/* for the eveluate context command, it can only be used for default control endpoint
	 * and change the max packet size. but, for non-full-speed device, the control endpoint
	 * max packet size is fixed. so we don't verify this command here, and it will be verified
	 * by xhci driver when emulate the full speed device
	 */
	switch (xhci_cmd)
	{
	case TRB_CMD_NOOP:
		cmd_no_op(&field1, &field2, &field3, &field4);
		break;
	case TRB_RESET_EP:
		cmd_reset_endpoint(xhci, ep, udev, &field1, &field2, &field3, &field4);
		break;
	case TRB_FORCE_HEADER:
		pr_info("construct force header command\n");
		cmd_force_header(udev, &field1, &field2, &field3, &field4);
		break;
	case TRB_STOP_RING: /* stop transfer ring */
		ret = xhci_verify_get_mass_storage_info(udev, &mass_stor_info);
		if(ret < 0)
		{
			write_log(ctler,
				"Fail: the target USB device doesn't follow the Spec in ep definition\n");
			goto out;
		}
		ep_index = xhci_get_endpoint_index(&(mass_stor_info.bulk_out->desc));
		field1 = field2 = field3 = 0;
		field4 = TRB_TYPE(TRB_STOP_RING) | SLOT_ID_FOR_TRB(udev->slot_id) | EP_ID_FOR_TRB(ep_index);
		break;
	case TRB_SET_DEQ:
		ret = xhci_verify_get_mass_storage_info(udev, &mass_stor_info);
		if(ret < 0)
		{
			write_log(ctler,
				"Fail: the target USB device doesn't follow the Spec in ep definition\n");
			goto out;
		}
		if(mass_stor_info.protocol != USB_PR_BULK)
		{
			write_log(ctler,
				"Fail: the target USB device doesn't follow the BBB protocol\n");
			ret = -EINVAL;
			goto out;
		}
		return xhci_verify_stor_transport_with_setdeq(ctler, udev, &mass_stor_info);
	case TRB_RESET_DEV:
		ret = usb_reset_device(udev);
		if(ret != 0)
			write_log(ctler, "Fail: fail to reset the device\n");
		else
			write_log(ctler, "Success\n");
		goto out;
	case TRB_NEG_BANDWIDTH:
		value = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
		if(!(HCC_BANDWIDTH_NEG(value)))
		{
			write_log(ctler, "Success: the controller doesn't support LTM\n");
			ret = 0;
			goto out;
		}
		field1 = field2 = field3 = 0;
		field4 = TRB_TYPE(TRB_NEG_BANDWIDTH);
		field4 |= SLOT_ID_FOR_TRB(udev->slot_id);
		break;
	case TRB_GET_BW:
		if(endpoint_index < 1 || endpoint_index > 5)
		{
			write_log(ctler, "Fail: the dev speed field invalid\n");
			ret = -EINVAL;
			goto out;
		}
		if(route_string == 0)//////////////////weitao need to modify????
		{
		     if(udev->speed==HCD_USB3)
		        buf_len = xhci->shared_hcd->self.root_hub->maxchild + 1;
                     else		
		       buf_len = ctler->hcd->self.root_hub->maxchild + 1;
		}
		else
		{
			buf_len = udev->maxchild + 1;
		}
		/* must be less than 2112 ports */
		buf = dma_pool_alloc(xhci->device_pool, GFP_KERNEL, &buf_dma);
		if(buf == NULL)
		{
			write_log(ctler, "Fail: fail to allocate the port bandwidth context\n");
			ret = -ENOMEM;
			goto out;
		}
		memset(buf, 0, buf_len);
		field1 = lower_32_bits(buf_dma);
		field2 = lower_32_bits(buf_dma);
		field3 = 0;
		field4 = TRB_TYPE(TRB_GET_BW);
		if(route_string != 0)
			field4 |= SLOT_ID_FOR_TRB(udev->slot_id);
		field4 |= (endpoint_index << 16);
		break;

	case TRB_ENABLE_SLOT:
	case TRB_DISABLE_SLOT:
	case TRB_ADDR_DEV:
	case TRB_CONFIG_EP:
	/* Eval: this command has been executed successfully when full speed device emunated s*/
	case TRB_EVAL_CONTEXT:
	case TRB_FORCE_EVENT:
	case TRB_SET_LT:
		ret = cmd_not_support(ctler, xhci_cmd);
		goto out;
	/*command not found */
	default:
		ret = -1;
		goto out;
	}

	init_completion(&xhci->cmd_verify_complet);
	xhci->is_cmd_verify = 1;

	spin_lock_irqsave(&xhci->lock, flags);
	/* alloc command */
	command = xhci_alloc_command(xhci, false, false, GFP_ATOMIC);
	if(!command)
		return -1;
	/* queue command */
	pr_info("queue command\n");
	ret = queue_command(xhci, command, field1, field2, field3, field4, false);
	if(ret != 0)
	{
		write_log(ctler, "Fail: fail to issue the target command\n");
		spin_unlock_irqrestore(&xhci->lock, flags);
		goto out;
	}
	/* ring doorbell */
	xhci_ring_cmd_db(xhci);
	spin_unlock_irqrestore(&xhci->lock, flags);

	/* I think 10 seconds is enough to transfer left  */
	ret = wait_for_completion_timeout(&xhci->cmd_verify_complet, 10 * HZ);
	
	/* return 0 means, wait to the end, it doesn't happen. */
	if(ret == 0) 
	{
		write_log(ctler, "Fail: command timeout(10s)\n");
		ret = -EPIPE;
		goto out;
	}
	else
	{
		pr_info("cmd_verify:the target command executed successfully\n");
	}

	xhci->is_cmd_verify = 0;
	
	/* restore for some commands */
	switch (xhci_cmd)
	{
	case TRB_FORCE_HEADER:
		pr_info("****second, deasserted\n");
		field1 = 0x020;
		field2 = field3 = 0;
		field4 = TRB_TYPE(TRB_FORCE_HEADER);
		pr_info("udev->portnum: %d\n", udev->portnum);
		field4 |= (udev->portnum << 24);
		//field4 |= (2 << 24);
		break;
	case TRB_GET_BW:
		ret = 0;
		write_log(ctler, "Success: with the following port bandwidth value:\n");
		for(i = 0; i < buf_len; i++)
			write_log(ctler, "%d, ", buf[i]);
		write_log(ctler, "\n");
		goto out;
	case TRB_STOP_RING:
		for(i = 0; i < 20; i++)
		{
			pr_info("the %dth loop after stop the bulk out endpoint\n", i + 1);
			msleep(500);
		}
		printk(KERN_ALERT "doorbell the bulk out endpoint after stop\n");
		xhci_ring_ep_doorbell(xhci, udev->slot_id, ep_index, 0);
		ret = 0;
		write_log(ctler, "Success\n");
		goto out;
	default:
		ret = 0;
		write_log(ctler, "Success\n");
		goto out;
	}

	spin_lock_irqsave(&xhci->lock, flags);
	command = xhci_alloc_command(xhci, false, false, GFP_ATOMIC);
	if(!command)
		return -1;
	pr_info("second, we queue command\n");
	ret = queue_command(xhci, command, field1, field2, field3, field4, false);
	if(ret < 0)
	{
		write_log(ctler, "Fail: fail to issue the target command\n");
		spin_unlock_irqrestore(&xhci->lock, flags);
		goto out;
	}
	xhci_ring_cmd_db(xhci);
	spin_unlock_irqrestore(&xhci->lock, flags);

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	if(buf != NULL)
		dma_pool_free(xhci->device_pool, buf, buf_dma);
	xhci->is_cmd_verify = 0;
	return ret;
}

void print_transfer_ring(struct xhci_hcd *xhci, struct usb_device *udev,
	struct usb_host_endpoint *ep)
{
	struct xhci_ring *ring;
	unsigned int ep_index;
	struct xhci_virt_ep *virt_ep;
	struct xhci_segment *seg;
	struct xhci_segment *first_seg;
	int i;
	u32 addr;
	union xhci_trb *trb;

	ep_index = xhci_get_endpoint_index(&ep->desc);
	virt_ep = &xhci->devs[udev->slot_id]->eps[ep_index];
	ring = virt_ep->ring;
	first_seg = ring->first_seg;
	pr_info("weitao:the num_trbs rest is %d\n", ring->num_trbs_free);
	pr_info("cycle state: %d, dequeue seg: 0x%p, dequeue pointer: 0x%p, "
		"enqueue seg: 0x%p, enqueue pointer: 0x%p\n", ring->cycle_state, ring->deq_seg,
		ring->dequeue, ring->enq_seg, ring->enqueue);
	pr_info("\n The segment information:\n");

	pr_info("segment: 0x%p, first TRB virtual addr: 0x%p\n", first_seg, first_seg->trbs);
	addr = first_seg->dma;
	for(i = 0; i < TRBS_PER_SEGMENT; ++i)
	{
		trb = &first_seg->trbs[i];
		pr_info("physical addr %08x %08x %08x %08x %08x\n", addr,
			lower_32_bits(trb->link.segment_ptr), upper_32_bits(trb->link.segment_ptr),
			(unsigned int) trb->link.intr_target, (unsigned int) trb->link.control);
		addr += sizeof(*trb);
	}

	for(seg = first_seg->next; seg != first_seg; seg = seg->next)
	{
		pr_info("segment: 0x%p, first TRB virtual addr: 0x%p\n", seg, seg->trbs);
		addr = seg->dma;
		for(i = 0; i < TRBS_PER_SEGMENT; ++i)
		{
			trb = &seg->trbs[i];
			pr_info("physical addr %08x %08x %08x %08x %08x\n", addr,
				lower_32_bits(trb->link.segment_ptr),
				upper_32_bits(trb->link.segment_ptr),
				(unsigned int) trb->link.intr_target, (unsigned int) trb->link.control);
			addr += sizeof(*trb);
		}
	}
}

void print_event_ring(struct xhci_hcd *xhci)
{
	struct xhci_ring *ring;
	struct xhci_segment *seg;
	struct xhci_segment *first_seg;
	int i;
	u32 addr;
	union xhci_trb *trb;

	ring = xhci->event_ring;
	first_seg = ring->first_seg;

	pr_info("cycle state: %d, dequeue seg: 0x%p, dequeue pointer: 0x%p, "
		"enqueue seg: 0x%p, enqueue pointer: 0x%p\n", ring->cycle_state, ring->deq_seg,
		ring->dequeue, ring->enq_seg, ring->enqueue);
	pr_info("\n The segment information:\n");

	pr_info("segment: 0x%p, first TRB virtual addr: 0x%p\n", first_seg, first_seg->trbs);
	addr = first_seg->dma;
	for(i = 0; i < TRBS_PER_SEGMENT; ++i)
	{
		trb = &first_seg->trbs[i];
		pr_info("physical addr %08x %08x %08x %08x %08x\n", addr,
			lower_32_bits(trb->link.segment_ptr), upper_32_bits(trb->link.segment_ptr),
			(unsigned int) trb->link.intr_target, (unsigned int) trb->link.control);
		addr += sizeof(*trb);
	}

	for(seg = first_seg->next; seg != first_seg; seg = seg->next)
	{
		pr_info("segment: 0x%p, first TRB virtual addr: 0x%p\n", seg, seg->trbs);
		addr = seg->dma;
		for(i = 0; i < TRBS_PER_SEGMENT; ++i)
		{
			trb = &seg->trbs[i];
			printk(KERN_ALERT "physical addr %08x %08x %08x %08x %08x\n", addr,
				lower_32_bits(trb->link.segment_ptr),
				upper_32_bits(trb->link.segment_ptr),
				(unsigned int) trb->link.intr_target, (unsigned int) trb->link.control);
			addr += sizeof(*trb);
		}
	}
}

/*
 * Synopsis:
 * cmd_code:route_string:request_code
 * %d:0x%x:%d
 * request_code must be 0(GetHubStatus) and 6(GetHubDescriptor)
 * by this, we can verify the sg transfer(complexed/multiple TRBs
 * with variable length and buffer align) for data stage for Hub device
 */
int hub_request(struct xhci_ctler *ctler)
{
	unsigned int cmd_code, route_string, request_code;
	unsigned int valid_len = 0;
	unsigned char *target_buf = NULL;
	int i, ret;
	struct usb_device *udev = NULL;
	struct usb_interface *intf = NULL;
	struct usb_hub *hub = NULL;

	struct hc_driver *xhci_drv = ctler->xhci_drv;
	struct urb *urb = NULL;
	unsigned int special_len = 0;
	struct xhci_mem_table *special_table;
	struct scatterlist *sg;

	unsigned char bmRequestType;
	unsigned short wIndex = 0;
	unsigned short wValue = 0;
	unsigned short wLength = 0;
	struct usb_ctrlrequest *dr;

	void *urb_enqueue_routine_bakeup;

	struct usb3_hub_descriptor u3_desc;
	struct usb_hub_descriptor u2_desc;
	struct usb_hub_status hub_status;
	unsigned char *tmp = NULL;

	cmd_code = route_string = request_code = 0;
	ret = read_cmd_str(ctler, &cmd_code, &route_string, &request_code);

	if(check_num_parameter(ctler, ret, 3))
		goto out;
	if(check_cmd_code(ctler, cmd_code, HUB_REQUEST))
		goto out;
	if((request_code != USB_REQ_GET_DESCRIPTOR) && (request_code != USB_REQ_GET_STATUS))
	{
		write_log(ctler, "Fail: request code not support,receive request code[%d]\n",
			request_code);
		return -EINVAL;
	}
	if(check_route_string(ctler, route_string))
		goto out;
	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;
	ret = check_hub_device(ctler, udev);
	if(ret)
		goto out;
	hub = hdev_to_struct_hub(udev);
    if(!hub)
	{
		pr_info("hub is NULL, This device is no Hub\n");
		goto out;
	}
	//pr_info("hub_descriptor is bDescLength=%d,bDescriptorType=%d,bNbrPorts=%d",
	//		   	hub->descriptor->bDescLength,hub->descriptor->bDescriptorType,
	//		   	hub->descriptor->bNbrPorts);
	
	intf = to_usb_interface(hub->intfdev);
	if(!intf)
	{
		pr_info("intf is NULL, to_usb_interface fail\n");
		goto out;
	}
	/* Fill some value */
	switch (request_code)
	{
	case USB_REQ_GET_DESCRIPTOR:
		if(udev->speed == USB_SPEED_SUPER)
		{
			pr_info("%s is Super Speed\n", udev->product);
			special_len = sizeof(struct usb3_hub_descriptor);
			wValue = 0x2A00;
		}
		if(udev->speed == USB_SPEED_HIGH)
		{
			pr_info("%s is high Speed\n", udev->product);
			special_len = sizeof(struct usb_hub_descriptor);
			wValue = 0x2900;
		}
		bmRequestType = USB_DIR_IN | USB_RT_HUB;
		wIndex = 0;
		wLength = special_len;
		break;
	case USB_REQ_GET_STATUS:
		special_len = wLength = sizeof(struct usb_hub_status);
		wValue = wIndex = 0;
		bmRequestType = USB_DIR_IN | USB_RT_HUB;
		break;
	default:
		break;
	}

	dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_NOIO);
	if(!dr)
	{
		write_log(ctler, "Fail: create setup stage buffer fail\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Special Mem Init */
	special_table = xhci_verify_create_special_mem_table(special_len);
	if(special_table == NULL)
	{
		write_log(ctler, "Fail: create the special table fail\n");
		ret = -ENOMEM;
		goto free_control_request_block;
	}

	ret = xhci_verify_init_special_mem_table(special_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	pr_info("dump mem_table for transfer length: %x\n", special_len);
	xhci_verify_dump_special_mem_table(special_table);

	if(unlikely(sg_alloc_table(ctler->sgtable, special_table->nents, GFP_KERNEL)))
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc sg table failed\n");
		goto free_special_table;
	}

	for_each_sg(ctler->sgtable->sgl, sg, special_table->nents, i) sg_set_page(sg,
		special_table->entry[i].page, special_table->entry[i].length,
		special_table->entry[i].offset);

	/* Fill URB */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if(urb == NULL)
	{
		write_log(ctler, "Fail: allocate urb failed\n");
		ret = -ENOMEM;
		goto free_sg_table;
	}
	urb->num_sgs   = special_table->nents;
	urb->sg        = ctler->sgtable->sgl;
	urb->dev       = udev;
	urb->ep        = &(udev->ep0);
	urb->pipe      = usb_rcvctrlpipe(udev, 0);
	urb->stream_id = 0;

	/* Transfer Flag */
	if(request_code == USB_REQ_GET_STATUS)
		urb->transfer_flags = URB_SHORT_NOT_OK;

	if(request_code == USB_REQ_GET_DESCRIPTOR)
	{
		if(udev->speed == USB_SPEED_SUPER)
			urb->transfer_flags = URB_SHORT_NOT_OK;
		else
			urb->transfer_flags = 0;
	}

	urb->transfer_buffer_length = wLength;
	urb->setup_packet = (unsigned char *) dr;

	dr->bRequestType = bmRequestType;
	dr->bRequest 	 = request_code;
	dr->wValue 		 = wValue;
	dr->wIndex 		 = wIndex;
	dr->wLength 	 = wLength;

	urb->complete = xhci_verify_urb_complete_fn;
	urb->context = &ctler->test.xhci_urb_completion;
	init_completion(&ctler->test.xhci_urb_completion);

	/* Hub may in suspend state, so we need resume it firstly */
	usb_autopm_get_interface(intf);

	urb_enqueue_routine_bakeup = xhci_drv->urb_enqueue;
	/* Enter into our own urb_enqueue, not the built-in one,
	 * Because we verify control sg transfer
	 */
	//xhci_drv->urb_enqueue = xhci_urb_enqueue_ctrl_sg_tx;
	xhci_drv->urb_enqueue = xhci_verify_urb_enqueue;
	
	/* Sbumit this sg urb */
	ret = usb_submit_urb(urb, GFP_KERNEL);

	if(ret < 0)
	{
		write_log(ctler, "Fail: submit URB failed\n");
		xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;
		goto free_urb;
	}

	/* we don't know how long this transfer needs */
	wait_for_completion(&ctler->test.xhci_urb_completion);

	/* restore the urb_enqueue routine */
	xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;

	ret = xhci_verify_parse_result(ctler, urb);
	pr_info("pass xhci_verify_parse_result\n");
	if(ret < 0)
	{
		ret = -EIO;
		write_log(ctler, "Fail: data stage transfer error\n");
		goto free_urb;
	}

	/* Verify the data reading from hub device */
	switch (request_code)
	{
	case USB_REQ_GET_DESCRIPTOR:
		if(udev->speed == USB_SPEED_SUPER)
		{
			tmp = (unsigned char *)(&u3_desc);
			valid_len = special_len;
		}
		if(udev->speed == USB_SPEED_HIGH)
		{
			tmp = (unsigned char *)(&u2_desc);
			valid_len = special_len;
		}
		ret = xhci_verify_copy_special_mem_table(special_table, tmp, valid_len);
		if(ret < 0)
		{
			write_log(ctler, "Fail: get Hub descriptor error\n");
			goto free_urb;
		}
		/* Fetch hub descriptor in xHCI to target_buf */
		target_buf = (unsigned char *)(hub->descriptor);
		if(!target_buf)
		{
			pr_info("target_buf is NULL\n");
			ret = -1;
			goto out;
		}

		/* we just judge some special field because some field depends on implementation */
		if(udev->speed == USB_SPEED_SUPER)
		{
			if(u3_desc.bDescLength != 12 || u3_desc.bDescriptorType != 0x2A
				|| u3_desc.bNbrPorts > 15)
			{
				pr_info("In USB_SPEED_SUPER,the Hub Descriptoer verify fail,"
					" Length is 0x%x, Type is 0x%x\n, NrPorts is %x, Removable is 0x%x\n",
					u3_desc.bDescLength, u3_desc.bDescriptorType, u3_desc.bNbrPorts,
					u3_desc.DeviceRemovable);
				write_log(ctler,
					"Fail: get Hub descriptor error,"
					"some field doesn't follow the Spec\n");
				ret = -ENOTSYNC;
				goto free_urb;
			}
			valid_len -= 2;//why?????????????????????
			if (memcmp(tmp, target_buf, valid_len) != 0) 
			{  
			 //  pr_info("u3_hub_descriptor is bDescLength=%d,bDescriptorType=%d,bNbrPorts=%d",
			 //  	*tmp,*(tmp+1),*(tmp+2));
			 //  pr_info("target_buf is bDescLength=%d,bDescriptorType=%d,bNbrPorts=%d",
			 //  	*target_buf,*(target_buf+1),*(target_buf+2));
			   write_log(ctler, "Fail: get Hub descriptor error," 
			   "some field doesn't match\n");
			   ret = -ENOTSYNC;
			   goto free_urb;
			 } 
		}
		if(udev->speed == USB_SPEED_HIGH)
		{
			if(u2_desc.bDescLength < 7 || u2_desc.bDescriptorType != 0x29)
			{
				pr_info("In USB_SPEED_HIGH, the Hub Descriptoer verify fail,"
					" Length is 0x%x, Type is 0x%x\n, NrPorts is %x\n",
					u2_desc.bDescLength, u2_desc.bDescriptorType, u2_desc.bNbrPorts);
				write_log(ctler,
					"Fail: get Hub descriptor error,"
					"some field doesn't follow the Spec\n");
				ret = -ENOTSYNC;
				goto free_urb;
			}
			valid_len = 7;
			
			if (memcmp(tmp, target_buf, valid_len) != 0) 
			{  
			   pr_info("u2_hub_descriptor is bDescLength=%d,bDescriptorType=%d,bNbrPorts=%d",
			   	*tmp,*(tmp+1),*(tmp+2));
			   pr_info("target_buf is bDescLength=%d,bDescriptorType=%d,bNbrPorts=%d",
			   	*target_buf,*(target_buf+1),*(target_buf+2));
			   write_log(ctler, "Fail: get Hub descriptor error,"
			   "some field doesn't match\n");
			   ret = -ENOTSYNC;
			   goto free_urb;
			}
		}
		break;
	case USB_REQ_GET_STATUS:
		ret = xhci_verify_copy_special_mem_table(special_table,
			(unsigned char *)(&hub_status), wLength);
		if(ret < 0)
		{
			write_log(ctler, "Fail: get Hub status error\n");
			goto free_urb;
		}

		/* for the Hub, we assume that the hub status is zero. that means
		 * that the local power is OK, there is no overcurrent happens
		 */
		if(hub_status.wHubStatus != 0 || hub_status.wHubChange != 0)
		{
			pr_info("the Hub status is not zero,"
				" HubStatus is 0x%x, HubChange is 0x%x\n", hub_status.wHubStatus,
				hub_status.wHubChange);
			write_log(ctler, "Fail: get Hub status error, the status is not zero\n");
			ret = -ENOTSYNC;
			goto free_urb;
		}
		break;
	default:
		break;
	}

	if(ret == 0)
		write_log(ctler, "Success\n");

  free_urb:
	if(urb != NULL)
		usb_free_urb(urb);
	usb_autopm_put_interface(intf);

  free_sg_table:
	sg_free_table(ctler->sgtable);

  free_special_table:
	if(special_table != NULL)
		xhci_verify_free_special_mem_table(special_table);

  free_control_request_block:
	if(dr != NULL)
		kfree(dr);
	
  out:
	return ret;
}
#ifdef xhci11_verify

int u3_entry_enable(struct xhci_ctler *ctler)
{
            struct usb_device *udev;
                unsigned int cmd_code, route_string, target_state;
                unsigned int port_index;
                unsigned int temp;
                unsigned int val2,hcc_params2;

                int ret = 0;
               // u32 port_status_change;
               // u32 port_status, port_change;
                struct xhci_hcd *xhci = ctler->xhci;
                u32 value = 0;
                u32 __iomem *addr;
                ret = sscanf(ctler->test.cmd_str, "%d:0x%x:%d", &cmd_code, &route_string,
                        &target_state);
                if(ret != 3)
                {
                        write_log(ctler,"Fail: Missing arguments, 3 arguments required, but only %d received\n",
                                ret);
                        ret = -EINVAL;
                        goto out;

                }
                else if(cmd_code != U3_ENTRY_CAPABILITY)
                {
                        write_log(ctler, "Fail: Command code mismatch, expect cmd[%d],but receive cmd[%d]\n",
                                U3_ENTRY_CAPABILITY, cmd_code);
                        ret = -EINVAL;
                        goto out;
                }
                /* need verify the device exist and endpoint valid */
                port_index = route_string & 0xFF;
                if(route_string == 0 || port_index == 0)
                {
                       write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);
                        ret = -EINVAL;
                        goto out;
                }
                temp = route_string;
                while(temp != 0)
                {
                        port_index = temp & 0xFF;
                        temp >>= 4;
                        if(port_index == 0 && temp != 0)
                        {
                                write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);
                                ret = -EINVAL;
                                goto out;
                        }
                }
                udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
                if(udev == NULL)
                {
                        write_log(ctler, "Fail: the target USB device doesn't exist\n");
                        ret = -EINVAL;
                        goto out;
                }
                if(udev->speed != USB_SPEED_SUPER)
                {
                        if(udev->parent == ctler->hcd->self.root_hub)
                        {
                                if(target_state != 3)
                                {
                                     write_log(ctler, "Fail: the target device is not SS device, "
                                        "the target state in not u3 state.\n");
                                        ret = -EINVAL;
                                        goto out;
                                }
                        }
                        else
                         {
                                  write_log(ctler, "Fail: the device doesn't on the root hub!\n");
                                  ret = -EINVAL;
                                  goto out;
                        }
                }
                else
                {
                        pr_info("udev->parent                      : %p\n", udev->parent);
                        pr_info("ctler->hcd->self.root_hub : %p\n", ctler->hcd->self.root_hub);
                        pr_info("xhci->share_hcd->self.root_hub : %p\n", xhci->shared_hcd->self.root_hub);
                        if(udev->parent == xhci->shared_hcd->self.root_hub)
                        {
                                if(target_state != 3)
                                {
                                       write_log(ctler, "Fail: the super speed device doesn't support the target state\n");
                                        ret = -EINVAL;
                                        goto out;
                                }
                        }
                        else
                        {
                              write_log(ctler, "Fail: device doesn't on the root hub!\n");
                                        ret = -EINVAL;
                                       goto out;
                       }
                }
                //weitao modify....on the root hub

                if(udev->parent == ctler->hcd->self.root_hub || udev->parent == xhci->shared_hcd->self.root_hub)
                {
                        port_index = route_string & 0xFF;
                        port_index--;
                        addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * port_index;
                        value = xhci_readl(xhci, addr);
                        if(((value & PORT_PLS_MASK) >> 5) == target_state)
                        {
                                write_log(ctler, "Success: the link is already in the target state\n");
                                ret = 0;
                                goto out;
                        }
                        if((value & PORT_PE) == 0 || (value & PORT_RESET))
                        {
                                write_log(ctler, "Fail: the target device is disabled or be resetting\n");
                                ret = -EINVAL;
                                goto out;
                        }
                         hcc_params2 = xhci_readl(xhci, &xhci->cap_regs->hcc_params2);
                        if(!HCC2_U3C(hcc_params2))
                         {
                                write_log(ctler, "success: the host control not support U3 Entry Capability!\n");
                                ret = 0;
                                goto out;
                        }
                        else
                        {
                                val2 = readl(&xhci->op_regs->config_reg);
                            if(!(CONFIG_U3E & val2))
                             {
                                   val2 |= CONFIG_U3E;
                                   writel(val2, &xhci->op_regs->config_reg);
                                   val2 = readl(&xhci->op_regs->config_reg);
                                   pr_info("we need to enable the u3 entry capability!\n");
                                   pr_info("after set this regs the val2=%x!\n",val2);
                                }
                         }
                        value = xhci_readl(xhci, addr);
                        value = xhci_port_state_to_neutral(value);
                        pr_info("the original value with neutral is 0x%x\n", value);
                        value &= ~PORT_PLS_MASK;
                        value |= (PORT_LINK_STROBE | (target_state << 5));
                        pr_info("write value 0x%x to the target register\n", value);
                        xhci_writel(xhci, value, addr);
                        //xhci->suspended_ports[port_index >> 5] |= 1 << (port_index & (31));
                        xhci->bus_state[0].suspended_ports |= 1 << (port_index & (31));
                        //xhci->bus_state[1].port_c_suspend[port_index >> 5] |= 1 << (port_index & 31);
                        usb_set_device_state(udev, USB_STATE_SUSPENDED);
                        msleep(20);
                        value = xhci_readl(xhci, addr);
                        pr_info("after write, the register value is 0x%x\n", value);
                        if(((value & PORT_PLS_MASK) >> 5) != target_state)
                        {
                           write_log(ctler,"Fail: the link doesn't enter the target state, now it is %d\n",
                                       ((value & PORT_PLS_MASK) >> 5));
                                ret = -EPIPE;
                                goto out;
                        }
                        ret = 0;
                        write_log(ctler, "Success\n");
                        goto out;
                }
        out:

        return ret;

}
#else

int u3_entry_enable(struct xhci_ctler *ctler)
{
     pr_info("xhci1.0 not support u3 entry capability!\n");
	 return 0;
}

#endif

/* For future test use */
int test(struct xhci_ctler *ctler)
{
	int ret;
	/*unsigned cmd_code, route_string;
	struct usb_device *udev   = NULL;
	struct hc_driver *xhci_drv = ctler->xhci_drv;
	struct urb *urb = NULL;
	struct xhci_hcd *xhci = ctler->xhci;*/

	//read_cmd_str(ctler, &cmd_code, &route_string);
	//udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	/*if(!udev) 
	{
		pr_info("udev is NULL\n");
		return -1;
	}*/
	ret=0;
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:descriptor_type
 * %d:0x%x:%d
 * descriptor_type must be 1(GetDeviceDescriptor) and 2(GetConfigurationDescriptor)
 * by this, we can verify the sg transfer(complexed/multiple TRBs
 * with variable length and buffer align) for data stage for normal USB device
 *
 */
int get_device_descriptor(struct xhci_ctler *ctler)
{
	int i, ret;
	unsigned cmd_code, route_string, descriptor_type;

	struct usb_device *udev;
	struct hc_driver *xhci_drv = ctler->xhci_drv;
	struct urb *urb;
	unsigned int special_len = 0;
	struct xhci_mem_table *special_table;
	struct scatterlist *sg;
	struct usb_ctrlrequest *dr;
	void *urb_enqueue_routine_bakeup;

	unsigned char *desc_buf = NULL;
	unsigned char *target_buf = NULL;

	struct usb_interface *intf = NULL;
	struct usb_hub *hub = NULL;

	ret = read_cmd_str(ctler, &cmd_code, &route_string, &descriptor_type);
	if((ret = check_num_parameter(ctler, ret, 3)))
		goto out;
	if((ret = check_cmd_code(ctler, cmd_code, GET_DEV_DESC)))
		goto out;
	if((descriptor_type != USB_DT_DEVICE) && (descriptor_type != USB_DT_CONFIG))
	{
		write_log(ctler,
			"Fail: descriptor type not support, receive descriptor type[%d]\n",
			descriptor_type);
		return -EINVAL;
	}
	if((ret = check_route_string(ctler, route_string)))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(udev == NULL)
	{
		write_log(ctler, "Fail: the target USB device doesn't exist\n");
		ret = -EINVAL;
		goto out;
	}
	switch (descriptor_type)
	{
	case USB_DT_DEVICE:
		special_len = USB_DT_DEVICE_SIZE; /* set length as 18 */
		break;
	case USB_DT_CONFIG:
		pr_info("\nthe size of configuration is 0x%x\n",udev->actconfig->desc.wTotalLength);
		special_len = udev->actconfig->desc.wTotalLength;
		break;
	default:
		break;
	}

	/* Descriptor buffer */
	desc_buf = kzalloc(special_len, GFP_KERNEL);
	if(desc_buf == NULL)
	{
		write_log(ctler, "Fail: create descriptor buffer fail\n");
		ret = -ENOMEM;
		goto out;
	}
	
	memset(desc_buf, 0 , sizeof(udev->descriptor));
	
	dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_NOIO);
	if(!dr)
	{
		write_log(ctler, "Fail: create setup stage buffer fail\n");
		ret = -ENOMEM;
		goto free_desc_buffer;
	}
	/* Create a special mem table with special length */
	special_table = xhci_verify_create_special_mem_table(special_len);
	if(special_table == NULL)
	{
		write_log(ctler, "Fail: create the special table fail\n");
		ret = -ENOMEM;
		goto free_control_request_block;
	}
	/* Init a special mem table with special table */
	ret = xhci_verify_init_special_mem_table(special_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	pr_info("Dump mem_table for transfer length: 0x%x\n", special_len);
	xhci_verify_dump_special_mem_table(special_table);

	if(unlikely(sg_alloc_table(ctler->sgtable, special_table->nents, GFP_KERNEL)))
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc sg table failed\n");
		pr_info("fail to alloc sg table\n");
		goto free_special_table;
	}

	pr_info("\n\nAlloc sg table success\n");

	for_each_sg(ctler->sgtable->sgl, sg, special_table->nents, i) 
		sg_set_page(sg,special_table->entry[i].page, special_table->entry[i].length,
		special_table->entry[i].offset);
	
	pr_info("\nAllocate the urb\n");
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if(urb == NULL)
	{
		write_log(ctler, "Fail: allocate urb failed\n");
		pr_info("fail to alloc urb\n");
		ret = -ENOMEM;
		goto free_sg_table;
	}
	urb->num_sgs = special_table->nents;
	urb->sg = ctler->sgtable->sgl;

	urb->dev  = udev;
	urb->ep   = &(udev->ep0);
	urb->pipe = usb_rcvctrlpipe(udev, 0);
	//urb->stream_id = 0;
	//urb->transfer_flags = URB_SHORT_NOT_OK;
	urb->transfer_buffer_length = special_len;
	urb->actual_length = 0;

	urb->setup_packet = (unsigned char *)dr;
	dr->bRequestType = USB_DIR_IN;
	dr->bRequest = USB_REQ_GET_DESCRIPTOR;
	dr->wValue = cpu_to_le16(descriptor_type << 8);   /* descriptor_type in high byte */
	if(descriptor_type == USB_DT_CONFIG)
		dr->wValue |= cpu_to_le16(udev->actconfig->desc.bConfigurationValue);
	dr->wIndex = cpu_to_le16(0);
	dr->wLength = cpu_to_le16(special_len);
	
	urb->complete = xhci_verify_urb_complete_fn;
	urb->context = &ctler->test.xhci_urb_completion;
	init_completion(&ctler->test.xhci_urb_completion);

	hub = hdev_to_struct_hub(udev);
	if(!hub)
	{
		pr_info("hub is NULL, This device is no Hub\n");
		goto normal_device;
	}
	intf = to_usb_interface(hub->intfdev);
	if(!intf)
	{
		pr_info("intf is NULL, to_usb_interface fail\n");
		goto out;
	}
	/* Hub may in suspend state, so we need resume it firstly */
	usb_autopm_get_interface(intf);

normal_device: /* For normal device */
	urb_enqueue_routine_bakeup = xhci_drv->urb_enqueue;
	xhci_drv->urb_enqueue = xhci_verify_urb_enqueue;

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if(ret < 0)
	{
		write_log(ctler, "Fail: submit URB failed\n");
		pr_info("***Fail to submit the urb\n");
		xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;
		goto free_urb;
	}
	/* we don't know how long this transfer needs */
	wait_for_completion(&ctler->test.xhci_urb_completion);
	
	xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;

	
	ret = xhci_verify_parse_result(ctler, urb);
	if(ret < 0)
	{
		ret = -EIO;
		write_log(ctler, "Fail: data stage transfer error\n");
		goto free_urb;
	}
	
	xhci_verify_dump_special_mem_table(special_table);
	
	/* Copy special data in one buffer */
	ret = xhci_verify_copy_special_mem_table(special_table, desc_buf, special_len);
	if(ret < 0)
	{
		write_log(ctler, "Fail: get Device descriptor error\n");
		goto free_urb;
	}
	
	if(descriptor_type == USB_DT_DEVICE)
		target_buf = (unsigned char *)(&udev->descriptor);
	else
	{
		/* the configuration index is same with bConfigurationValue?
		 * I don't know. but just find it out
		 */
		for(i = 0; i < udev->descriptor.bNumConfigurations; i++)
		{
			target_buf = udev->rawdescriptors[i];
			if(*(target_buf + 5) == udev->actconfig->desc.bConfigurationValue)
			{
				target_buf = udev->rawdescriptors[i];
				break;
			}
		}
		if(i == udev->descriptor.bNumConfigurations)
		{
			write_log(ctler, "Fail: the active configuration is not exist\n");
			ret = -EPIPE;
			goto free_urb;
		}
	}

	/* dump descriptor */
	pr_info("\n****the descriptor****\n\nthe original is: ");
	for(i = 0; i < special_len; i++)
		pr_info("0x%x, ", target_buf[i]);
	
	pr_info("\nand we get:      \n");
	for(i = 0; i < special_len; i++)
		pr_info("0x%x, ", desc_buf[i]);
	pr_info("\n\n\n");

	if(memcmp(desc_buf, target_buf, special_len) != 0)
	{
		write_log(ctler, "Fail: Get device descriptor 0x%x fail\n", descriptor_type);
		ret = -ENOTSYNC;
		goto free_urb;
	}

	write_log(ctler, "Success\n");

  free_urb:
	if(urb != NULL)
		usb_free_urb(urb);
	if(intf != NULL)
		usb_autopm_put_interface(intf);

  free_sg_table:
	sg_free_table(ctler->sgtable);

  free_special_table:
	if(special_table != NULL)
		xhci_verify_free_special_mem_table(special_table);

  free_control_request_block:
	if(dr != NULL)
		kfree(dr);

  free_desc_buffer:
	if(desc_buf != NULL)
		kfree(desc_buf);

  out:
  	pr_info("ret is %d\n",ret);
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:endpoint_index:target_num
 * %d:0x%x:0x%x:%d
 * you can further verify the multiple segment function by copy compare
 * furthermore, we just process the request to add the segment number
 */
int set_transfer_seg_num(struct xhci_ctler *ctler)
{
	int i, j, ret;
	unsigned int cmd_code, route_string, endpoint_index, target_num;
	unsigned char direction = 0;
	unsigned char ep_num = 0;

	struct usb_device *udev;
	struct usb_host_endpoint *ep;
	struct xhci_hcd *xhci = ctler->xhci;
	unsigned int ep_index;
	struct xhci_virt_device *virt_dev;
	struct xhci_virt_ep *virt_ep;
	struct xhci_ring *ring;
	struct xhci_segment *seg_cache[16];
	struct xhci_segment *next_seg, *prev_seg;
	unsigned int seg_num;
	struct xhci_segment *deq_seg, *enq_seg;
	union xhci_trb *trb;
	u32 control_backup;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x:0x%x:%d", &cmd_code, &route_string,
		&endpoint_index, &target_num);

	if(check_num_parameter(ctler, ret, 4))
		goto out;
	if(check_cmd_code(ctler, cmd_code, SET_TRANSFER_SEG_NUM))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	virt_dev = xhci->devs[udev->slot_id];
	if(udev->slot_id == 0 || virt_dev == NULL)
	{
		write_log(ctler, "Fail: the target USB device has invalid slot id\n");
		ret = -EINVAL;
		goto out;
	}

	if((endpoint_index & USB_ENDPOINT_NUMBER_MASK) == 0)
	{
		write_log(ctler,
			"Fail: the target endpoint is default control endpoint, not support\n");
		ret = -EINVAL;
		goto out;
	}

	direction = ((endpoint_index & USB_DIR_IN) >> 7);
	pr_info("The Endpoint Direction is %d\n", direction);
	ep_num = endpoint_index & 0x0F;
	ep = xhci_verify_get_ep_by_address(udev, direction, ep_num);
	if(ep == NULL)
	{
		write_log(ctler, "Fail: the target endpoint doesn't exist or enabled\n");
		ret = -EINVAL;
		goto out;
	}

	/* just for simple, we assume that the endpoint is in idle state
	 * we just verify the controller, we need not process complexed
	 * situation because we can control the situation to be simple.
	 * as we know, we are verifing hardware, we are not verify the driver!
	 */
	ep_index = xhci_get_endpoint_index(&ep->desc);
	virt_ep = &(virt_dev->eps[ep_index]);
	if(virt_ep == NULL)
	{
		write_log(ctler, "Fail: the target endpoint has no virtual target\n");
		ret = -EINVAL;
		goto out;
	}
	ring = virt_ep->ring;
	if(ring == NULL)
	{
		write_log(ctler, "Fail: the target endpoint has no transfer ring\n");
		ret = -EINVAL;
		goto out;
	}

	next_seg = ring->first_seg->next;
	seg_num = 1;
	while(next_seg != ring->first_seg)
	{
		seg_num++;
		next_seg = next_seg->next;
	}

	if(seg_num >= target_num)
	{
		write_log(ctler, "Success: need not decrease the segment number\n");
		ret = 0;
		goto out;
	}

	for(i = 0; i < 16; i++)
		seg_cache[i] = NULL;

	pr_info("need enlarge the segments from %d to %d\n", seg_num, target_num);

	for(i = 0; i < (target_num - seg_num); i++)
	{
		seg_cache[i] = xhci_segment_alloc(xhci, 1, GFP_NOIO);
		if(seg_cache[i] == NULL)
		{
			write_log(ctler, "Fail, fail to alloc the required transfer ring segment\n");
			pr_info("fail to alloc the %d th transfer ring segment\n", i);
			ret = -ENOMEM;
			goto free_segment;
		}
	}

	deq_seg = ring->deq_seg;
	enq_seg = ring->enq_seg;
	next_seg = enq_seg->next;
	control_backup = enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.control;
	if((enq_seg != deq_seg) || (enq_seg == deq_seg && ring->dequeue <= ring->enqueue))
	{
		prev_seg = enq_seg;
		for(i = 0; i < (target_num - seg_num); i++)
		{
			xhci_link_segments(xhci, prev_seg, seg_cache[i], true);
			prev_seg = seg_cache[i];
		}
		xhci_link_segments(xhci, prev_seg, next_seg, true);

		if(ring->cycle_state == 0)
		{
			for(i = 0; i < (target_num - seg_num); i++)
			{
				trb = seg_cache[i]->trbs;
				for(j = 0; j < TRBS_PER_SEGMENT; j++)
				{
					trb->generic.field[3] |= TRB_CYCLE;
					trb++;
				}
			}
		}

		if(control_backup & LINK_TOGGLE)
		{
			enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.control &= ~LINK_TOGGLE;
			prev_seg->trbs[TRBS_PER_SEGMENT - 1].link.control |= LINK_TOGGLE;
		}

	}
	else
	{
		write_log(ctler,
			"Fail: the transfer ring overlap and can't increase the segment\n");
		pr_info("the transfer ring overlap and can't increase the segment\n");
		ret = -EINVAL;
		goto free_segment;
	}

	write_log(ctler,
		"Success: enlarge the transfer ring successfully, please do the further verify by C&C\n");
	ret = 0;

	for(i = 0; i < (target_num - seg_num); i++)
	{
		seg_cache[i] = NULL;
	}

  free_segment:
	for(i = 0; i < 16; i++)
		if(seg_cache[i] != NULL)
			xhci_segment_free(xhci, seg_cache[i]);

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:endpoint_index:transfer_length
 * %d:0x%x:0x%x:%d
 *
 */
int sg_transfer(struct xhci_ctler *ctler)
{
	int i, ret;
	unsigned cmd_code, route_string, endpoint_index;
	unsigned transfer_length;
	unsigned char direction = 0;
	unsigned char ep_num = 0;
	struct usb_host_endpoint *ep;
	struct xhci_ep_ctx *ep_ctx;
	struct usb_device *udev;
	struct urb *urb = NULL;
	struct xhci_hcd *xhci = ctler->xhci;

	unsigned int max_packet_size;
	unsigned int special_len, normal_len;
	struct xhci_mem_table *normal_table, *special_table;
	struct scatterlist *sg;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x:0x%x:%d",
		&cmd_code, &route_string, &endpoint_index, &transfer_length);

	if(check_num_parameter(ctler, ret, 4))
		goto out;
	if(check_cmd_code(ctler, cmd_code, SG_TRANSFER))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	direction = ((endpoint_index & USB_DIR_IN) >> 7);
	ep_num = endpoint_index & 0x0F;
	ep = xhci_verify_get_ep_by_address(udev, direction, ep_num);
	if(ep == NULL)
	{
		write_log(ctler, "Fail: the target endpoint doesn't exist or enabled\n");
		ret = -EINVAL;
		goto out;
	}

	if(usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_CONTROL)
	{
		write_log(ctler,
			"Fail: the target endpoint is control endpoint, please use the dev_request command\n");
		ret = -EINVAL;
		goto out;
	}

	ep_ctx = xhci_get_ep_ctx(xhci, xhci->devs[udev->slot_id]->out_ctx,
		xhci_get_endpoint_index(&ep->desc));
	if((ep_ctx->ep_info & EP_STATE_MASK) != EP_STATE_RUNNING)
	{
		write_log(ctler, "Fail: the target endpoint is not in running state\n");
		ret = -EINVAL;
		goto out;
	}

	/* reserve two sg entry for normal */
	if(udev->bus->sg_tablesize < ONE_MAX_PACKET_MEM_ENTRY_CNT + 2)
	{
		write_log(ctler,
			"Fail: scsi_host->sg_tablesize is too small to "
			"accommodate the testing sg list\n");
		ret = -EINVAL;
		goto out;
	}
	max_packet_size = ep->desc.wMaxPacketSize & 0x7FF;
	if(transfer_length % max_packet_size == 0)
	{
		special_len = max_packet_size;
		normal_len = transfer_length - max_packet_size;
	}
	else
	{
		special_len = (transfer_length % max_packet_size);
		normal_len = transfer_length - special_len;
	}
	normal_table = xhci_verify_create_normal_mem_table(ctler, normal_len);
	if(normal_table == NULL)
	{
		write_log(ctler, "Fail: alloc normal mem_table failed\n");
		ret = -ENOMEM;
		goto out;
	}
	special_table = xhci_verify_create_special_mem_table(special_len);
	if(special_table == NULL)
	{
		write_log(ctler, "Fail: alloc special mem_table failed\n");
		ret = -ENOMEM;
		goto free_normal_table;
	}

	/* reset all the buffer data value */
	xhci_verify_reset_mem_pool_data(ctler);

	ret = xhci_verify_init_normal_mem_table(normal_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}
	ret = xhci_verify_init_special_mem_table(special_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	pr_info("dump mem_table for transfer length: %x\n", transfer_length);
	xhci_verify_dump_normal_mem_table(normal_table);
	xhci_verify_dump_special_mem_table(special_table);

	if((usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_BULK)
		|| (usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_INT))
	{
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if(check_urb(ctler, urb))
			goto free_special_table;

		if(unlikely(sg_alloc_table(ctler->sgtable,
					normal_table->nents + special_table->nents, GFP_KERNEL)))
		{
			ret = -ENOMEM;
			write_log(ctler, "Fail: alloc sg table failed\n");
			goto free_urb;
		}

		for_each_sg(ctler->sgtable->sgl, sg, normal_table->nents, i) sg_set_page(sg,
			normal_table->entry[i].page, normal_table->entry[i].length,
			normal_table->entry[i].offset);
		for_each_sg(ctler->sgtable->sgl + normal_table->nents, sg, special_table->nents,
			i) sg_set_page(sg, special_table->entry[i].page,
			special_table->entry[i].length, special_table->entry[i].offset);

		urb->num_sgs = normal_table->nents + special_table->nents;
		urb->sg = ctler->sgtable->sgl;

		urb->dev = udev;
		urb->ep = ep;
		if(usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_BULK)
		{
			if(direction)
				urb->pipe = usb_rcvbulkpipe(udev, ep_num);
			else
				urb->pipe = usb_sndbulkpipe(udev, ep_num);
		}
		else if(usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_INT)
		{
			if(direction)
				urb->pipe = usb_rcvintpipe(udev, ep_num);
			else
				urb->pipe = usb_sndintpipe(udev, ep_num);
		}
		urb->stream_id = 0;
		//urb->transfer_flags = URB_SHORT_NOT_OK;
		urb->transfer_buffer_length = transfer_length;
		urb->interval = ep->desc.bInterval;
		if((usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_INT)
			&& (urb->interval == 0))
			urb->interval = 1;

		urb->complete = xhci_verify_urb_complete_fn;
		urb->context = &ctler->test.xhci_urb_completion;
		init_completion(&ctler->test.xhci_urb_completion);

		ret = usb_submit_urb(urb, GFP_KERNEL);
		if(ret < 0)
		{
			pr_info("usb_submit_urb retval is %d\n", ret);
			write_log(ctler, "Fail: submit URB failed\n");
			goto free_sg_table;
		}

		/* we don't know how long this transfer needs */
		wait_for_completion(&ctler->test.xhci_urb_completion);

		ret = xhci_verify_parse_result(ctler, urb);

		/* if the transfer is completed successfully, and the transfer is IN, need verify the
		 * buffer has been modified. but we can't assume that all buffer unit has been modified,
		 * we can assume that it is impossiable that all buffer unit is not modified
		 */
		if((ret == 0) && direction)
		{
			if((xhci_verify_normal_mem_table(normal_table) == 0)
				&& (xhci_verify_special_mem_table(special_table) == 0))
			{
				ret = -EIO;
				write_log(ctler, "Fail: In transfer error\n");
				goto free_sg_table;
			}
		}

		if(ret == 0)
			write_log(ctler, "Success\n");

	}
	else if(usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_ISOC)
	{
		/* for ischronous endpoint */
		/* we can't process it as bulk/interrupt, the inbox kernel routine has so many limitation,
		 * we have to queue the TRB by ourself
		 */
	}

  free_sg_table:
	sg_free_table(ctler->sgtable);

  free_urb:
	usb_free_urb(urb);

  free_special_table:
	xhci_verify_free_special_mem_table(special_table);

  free_normal_table:
	xhci_verify_free_normal_mem_table(normal_table);

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:sector_addr:sector_cnt
 * %d:0x%x:%d:%d
 *
 */
int sg_verify(struct xhci_ctler *ctler)
{
	int i, ret;
	unsigned cmd_code, route_string;
	unsigned transfer_length, start_sector, sector_cnt;
	struct usb_device *udev;

	unsigned int special_len, normal_len;
	struct xhci_mem_table *normal_table, *special_table;
	struct scatterlist *sg;
	struct mass_storage_dev_info mass_stor_info;

	ret =
		sscanf(ctler->test.cmd_str, "%d:0x%x:%d:%d", &cmd_code, &route_string,
		&start_sector, &sector_cnt);

	if(check_num_parameter(ctler, ret, 4))
		goto out;
	if(check_cmd_code(ctler, cmd_code, SG_VERIFY))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	ret = xhci_verify_get_mass_storage_info(udev, &mass_stor_info);
	if(ret < 0)
	{
		write_log(ctler,
			"Fail: the target USB device doesn't follow  Spec in ep definition\n");
		goto out;
	}

	transfer_length = sector_cnt * 512;
	if(transfer_length == 0)
	{
		write_log(ctler, "Fail: the sector count is zero\n");
		ret = -EINVAL;
		goto out;
	}

	/* reserve two sg entry for nomal */
	if(udev->bus->sg_tablesize < ONE_MAX_PACKET_MEM_ENTRY_CNT + 2)
	{
		write_log(ctler,
			"Fail: scsi_host->sg_tablesize is too small"
			"to accommodate the testing sg list\n");
		ret = -EINVAL;
		goto out;
	}

	special_len = 1024;
	normal_len = transfer_length - 1024;

	//special_len = 0;
	//normal_len = transfer_length;

	normal_table = xhci_verify_create_normal_mem_table(ctler, normal_len);
	if(check_normal_table(ctler, normal_table))
		goto out;
	special_table = xhci_verify_create_special_mem_table(special_len);
	if(check_special_table(ctler, special_table))
		goto free_normal_table;

	/* reset all the buffer data value */
	xhci_verify_reset_mem_pool_data(ctler);

	ret = xhci_verify_init_normal_mem_table(normal_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}
	ret = xhci_verify_init_special_mem_table(special_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	pr_info("\nDump mem_table for transfer length: %x\n", transfer_length);
	xhci_verify_dump_normal_mem_table(normal_table);
	xhci_verify_dump_special_mem_table(special_table);

	if(unlikely(sg_alloc_table(ctler->sgtable,
				normal_table->nents + special_table->nents, GFP_KERNEL)))
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc sg table failed\n");
		goto free_special_table;
	}

	for_each_sg(ctler->sgtable->sgl, sg, normal_table->nents, i) sg_set_page(sg,
		normal_table->entry[i].page, normal_table->entry[i].length,
		normal_table->entry[i].offset);

	for_each_sg(ctler->sgtable->sgl + normal_table->nents, sg, special_table->nents,
		i) sg_set_page(sg, special_table->entry[i].page, special_table->entry[i].length,
		special_table->entry[i].offset);

	for_each_sg(ctler->sgtable->sgl, sg, normal_table->nents + special_table->nents, i)
		pr_info("for %d th sg, Page=0x%x, length=%d, offset=%d\n", i,
		(unsigned int) sg->page_link, sg->length, sg->offset);

	/* write the buffer to USB mass storage device */
	ret = xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
		sector_cnt, normal_table, special_table, 1, 0, 0);
	if(ret < 0)
		goto free_sg_table;

	xhci_verify_reset_mem_pool_data(ctler);
	xhci_verify_reset_special_mem_table_data(special_table);

	/* read the data back with the same buffer, which has been reset */
	ret =
		xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
		sector_cnt, normal_table, special_table, 0, 0, 0);
	if(ret < 0)
		goto free_sg_table;

	/* verify the data now */
	if((xhci_verify_normal_mem_table(normal_table) != 0)
		|| (xhci_verify_special_mem_table(special_table) != 0))
	{
		ret = -ENOTSYNC;
		write_log(ctler, "Fail: data verify fail\n");
		pr_info("dump mem_table for transfer length: %x\n", transfer_length);
		xhci_verify_dump_normal_mem_table(normal_table);
		xhci_verify_dump_special_mem_table(special_table);
		goto free_sg_table;
	}

	if(ret == 0)
		write_log(ctler, "Success\n");

  free_sg_table:
	sg_free_table(ctler->sgtable);

  free_special_table:
	xhci_verify_free_special_mem_table(special_table);

  free_normal_table:
	xhci_verify_free_normal_mem_table(normal_table);

  out:
	return ret;

}

/*
 * Synopsis:
 * cmd_code:route_string:endpoint_index
 * %d:0x%x:0x%x
 * we issue only one URB to the Isoch endpoint to trigger this endpoint
 * to enter into running state. after this URB is transfered, there is no
 * transfer any more, and the underrun/overrun happens.
 *
 */

int isoch_underrun_overrun(struct xhci_ctler *ctler)
{
	struct usb_device *udev;
	unsigned int cmd_code, route_string, endpoint_index;
	unsigned int transfer_length;
	unsigned int port_index;
	unsigned int temp;
	unsigned char direction = 0;
	unsigned char ep_num = 0;
	struct usb_host_endpoint *ep;
	struct xhci_ep_ctx *ep_ctx;
	int i;
	int ret = 0;

	struct xhci_hcd *xhci = ctler->xhci;
	struct urb *urb = NULL;
	unsigned int max_packet_size;
	unsigned int len_per_isoc_td;
	unsigned int td_number;
	struct usb_iso_packet_descriptor *frame_desc;
	unsigned int normal_len;
	struct xhci_mem_table *normal_table;

	unsigned char *buffer;

	struct hc_driver *xhci_drv = ctler->xhci_drv;
	void *urb_enqueue_routine_bakeup;

	ret =
		sscanf(ctler->test.cmd_str, "%d:0x%x:0x%x", &cmd_code, &route_string,
		&endpoint_index);

	if(ret != 3)
	{
		write_log(ctler,
			"Fail: Missing arguments, " "4 arguments required, but only %d received\n",
			ret);

		ret = -EINVAL;
		goto out;

	}
	else if(cmd_code != ISOCH_UNDERRN_OVERRUN)
	{
		write_log(ctler,
			"Fail: Command code mismatch, " "expect cmd[%d],but receive cmd[%d]\n",
			ISOCH_UNDERRN_OVERRUN, cmd_code);
		ret = -EINVAL;
		goto out;
	}

	/* need verify the device exist and endpoint valid */
	port_index = route_string & 0xF;
	if(route_string == 0 || port_index == 0)
	{
		write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);
		ret = -EINVAL;
		goto out;
	}
	temp = route_string;
	while(temp != 0)
	{
		port_index = temp & 0xF;
		temp >>= 4;
		if(port_index == 0 && temp != 0)
		{
			write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);
			ret = -EINVAL;
			goto out;
		}
	}
	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(udev == NULL)
	{
		write_log(ctler, "Fail: the target USB device doesn't exist\n");
		ret = -EINVAL;
		goto out;
	}

	direction = ((endpoint_index & USB_DIR_IN) >> 7);
	ep_num = endpoint_index & 0x0F;
	ep = xhci_verify_get_ep_by_address(udev, direction, ep_num);
	if(ep == NULL)
	{
		write_log(ctler, "Fail: the target endpoint doesn't exist or enabled\n");
		ret = -EINVAL;
		goto out;
	}

	//debug
	pr_info("usb_endpoint_type is %d\n", usb_endpoint_type(&ep->desc));

	if(usb_endpoint_type(&ep->desc) != USB_ENDPOINT_XFER_ISOC)
	{
		write_log(ctler, "Fail: the target endpoint is not isochronous endpoint\n");
		ret = -EINVAL;
		goto out;
	}

	printk(KERN_ALERT "in isoch_unrun, come here1\n");

	ep_ctx =
		xhci_get_ep_ctx(xhci, xhci->devs[udev->slot_id]->out_ctx,
		xhci_get_endpoint_index(&ep->desc));
	if((ep_ctx->ep_info & EP_STATE_MASK) != EP_STATE_RUNNING)
	{
		write_log(ctler, "Fail: the target endpoint is not in running state\n");
		ret = -EINVAL;
		goto out;
	}

	for(i = 0; i < MAX_POOL_ENTRY; i++)
	{
		if(ctler->mem_pool[i].actual_num > 0)
			break;
	}
	if(i == MAX_POOL_ENTRY)
	{
		write_log(ctler, "Fail: there is no memory for the transfer\n");
		ret = -ENOMEM;
		goto out;
	}
	printk(KERN_ALERT "in isoch_unrun, come here2\n");

	transfer_length = ctler->mem_pool[i].length_in_page * PAGE_SIZE;
	max_packet_size = ep->desc.wMaxPacketSize & 0x7FF;
	transfer_length = (transfer_length / max_packet_size) * max_packet_size;
	normal_len = transfer_length;
	printk(KERN_ALERT "the transfer length is %d\n", normal_len);
	normal_table = xhci_verify_create_normal_mem_table(ctler, normal_len);
	printk(KERN_ALERT "has allocate the normal mem table\n");
	if(normal_table == NULL)
	{
		write_log(ctler, "Fail: alloc normal mem_table failed\n");
		ret = -ENOMEM;
		goto out;
	}
	printk(KERN_ALERT "in isoch_unrun, come here3\n");

	/* reset all the buffer data value */
	xhci_verify_reset_mem_pool_data(ctler);

	ret = xhci_verify_init_normal_mem_table(normal_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_normal_table;
	}

	printk(KERN_ALERT "in isoch_unrun, come here4\n");

	if(normal_table->nents != 1)
	{
		write_log(ctler, "Fail: internal error, invalid buffer entry\n");
		ret = -EINVAL;
		goto free_normal_table;
	}

	pr_info("dump mem_table for transfer length: %x\n", transfer_length);
	xhci_verify_dump_normal_mem_table(normal_table);

	if(usb_endpoint_type(&ep->desc) == USB_ENDPOINT_XFER_ISOC)
	{
		len_per_isoc_td = xhci_get_max_esit_payload(xhci, udev, ep);
		if(len_per_isoc_td == 0)
		{
			write_log(ctler, "Fail: the max palyload ESIT is 0\n");
			ret = -EINVAL;
			goto free_normal_table;
		}
		td_number = (transfer_length + len_per_isoc_td - 1) / len_per_isoc_td;
		if(td_number == 0)
		{
			write_log(ctler, "Fail: need 0 td, it is error\n");
			ret = -EINVAL;
			goto free_normal_table;
		}
		urb = usb_alloc_urb(td_number, GFP_KERNEL);
		if(urb == NULL)
		{
			write_log(ctler, "Fail: allocate urb failed\n");
			ret = -ENOMEM;
			goto free_normal_table;
		}

		buffer =
			page_address(normal_table->entry[0].page) + normal_table->entry[0].offset;
		urb->transfer_buffer = buffer;
		urb->transfer_buffer_length = transfer_length;

		urb->number_of_packets = td_number;
		for(i = 0; i < td_number; i++)
		{
			frame_desc = &urb->iso_frame_desc[i];
			frame_desc->offset = len_per_isoc_td * i;
			frame_desc->length =
				min(len_per_isoc_td, transfer_length - i * len_per_isoc_td);
		}

		urb->dev = udev;
		urb->ep = ep;
		if(direction)
			urb->pipe = usb_rcvisocpipe(udev, ep_num);
		else
			urb->pipe = usb_sndisocpipe(udev, ep_num);

		urb->stream_id = 0;
		urb->transfer_flags = URB_ISO_ASAP;

		urb->interval = ep->desc.bInterval;
		if(urb->interval == 0)
			urb->interval = 1;

		urb->complete = xhci_verify_urb_complete_fn;
		urb->context = &ctler->test.xhci_urb_completion;
		init_completion(&ctler->test.xhci_urb_completion);
		init_completion(&xhci->isoch_complet);
		xhci->is_isoch_verify = 1;

		ret = usb_submit_urb(urb, GFP_KERNEL);
		if(ret < 0)
		{
			write_log(ctler, "Fail: submit URB failed\n");
			goto free_urb;
		}

		/* we don't know how long this transfer needs */
		wait_for_completion(&ctler->test.xhci_urb_completion);

		ret = xhci_verify_parse_result(ctler, urb);

		/* if the transfer is completed successfully, and the transfer is IN, need verify the
		 * buffer has been modified. but we can't assume that all buffer unit has been modified,
		 * we can assume that it is impossiable that all buffer unit is not modified
		 */
		if((ret == 0) && direction)
		{
			if(xhci_verify_normal_mem_table(normal_table) == 0)
			{
				ret = -EIO;
				write_log(ctler, "Fail: In transfer error\n");
				xhci->is_isoch_verify = 0;
				goto free_urb;
			}
		}

		wait_for_completion(&xhci->isoch_complet);

		pr_info("the isoch endpoint has complete"
			" transfer and the overrun/underrun event occurs\n");

		xhci->is_isoch_verify = 0;

		ep_ctx =
			xhci_get_ep_ctx(xhci, xhci->devs[udev->slot_id]->out_ctx,
			xhci_get_endpoint_index(&ep->desc));
		if((ep_ctx->ep_info & EP_STATE_MASK) != EP_STATE_RUNNING)
		{
			write_log(ctler,
				"Fail: the target endpoint is not in running state after overrun/underrun event occurs\n");
			ret = -EINVAL;
			goto free_urb;
		}
		pr_info("the isoch endpoint is in running state"
			" after the overrun/underrun event occurs\n");

		/* reset all the buffer data value */
		xhci_verify_reset_mem_pool_data(ctler);
		xhci_verify_init_normal_mem_table(normal_table);

		urb_enqueue_routine_bakeup = xhci_drv->urb_enqueue;
		xhci_drv->urb_enqueue = xhci_verify_urb_enqueue;
		init_completion(&ctler->test.xhci_urb_completion);
		ret = usb_submit_urb(urb, GFP_KERNEL);
		if(ret < 0)
		{
			write_log(ctler, "Fail: submit URB failed\n");
			goto free_urb;
		}
		/* I think 10 seconds is enough to transfer this  */
		if(!wait_for_completion_timeout(&ctler->test.xhci_urb_completion, 10 * HZ))
		{
			pr_info("good, the second urb is not executed at all\n");
		}
		else
		{
			write_log(ctler,
				"Fail: the second urb is executed without doorbell the endpoint\n");
			ret = -EPIPE;
			goto free_urb;
		}

		xhci_ring_ep_doorbell(xhci, udev->slot_id, xhci_get_endpoint_index(&ep->desc), 0);

		wait_for_completion(&ctler->test.xhci_urb_completion);

		/* restore the urb_enqueue routine */
		xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;

		pr_info("the second URB has been completed\n");

		ret = xhci_verify_parse_result(ctler, urb);

		/* if the transfer is completed successfully, and the transfer is IN, need verify the
		 * buffer has been modified. but we can't assume that all buffer unit has been modified,
		 * we can assume that it is impossiable that all buffer unit is not modified
		 */
		if((ret == 0) && direction)
		{
			if(xhci_verify_normal_mem_table(normal_table) == 0)
			{
				ret = -EIO;
				write_log(ctler, "Fail: In transfer error\n");
				goto free_urb;
			}
		}

		if(ret == 0)
			write_log(ctler, "Success\n");

	}

  free_urb:
	usb_free_urb(urb);

  free_normal_table:
	xhci_verify_free_normal_mem_table(normal_table);

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:endpoint_index
 * %d:0x%x:0x%x
 * if the target device is a mass storage device,
 * we will verify the event data TRB for bulk in/out endpoint as sg_verify.
 * otherwise, we just issue this URB with the last
 * TRB as event data TRB
 * 
 */
int event_data_trb(struct xhci_ctler *ctler)
{
	int i, ret;

	unsigned int cmd_code, route_string, endpoint_index;
	unsigned char direction = 0;
	unsigned char ep_num = 0;
	struct usb_device *udev;
	struct usb_host_endpoint *ep;
	struct hc_driver *xhci_drv = ctler->xhci_drv;

	unsigned int special_len, normal_len;
	struct xhci_mem_table *normal_table = NULL;
	struct xhci_mem_table *special_table = NULL;

	struct scatterlist *sg;
	void *urb_enqueue_routine_bakeup;

	struct mass_storage_dev_info mass_stor_info;
	unsigned int start_sector, sector_cnt;
	unsigned int transfer_length;

	ret = read_cmd_str(ctler, &cmd_code, &route_string, &endpoint_index);
	if(check_num_parameter(ctler, ret, 3))
		goto out;
	if(check_cmd_code(ctler, cmd_code, EVENT_DATA_TRB))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	ret = xhci_verify_get_mass_storage_info(udev, &mass_stor_info);
	if(ret < 0)
	{
		pr_info("the target device is not usb mass storage device\n");
		direction = ((endpoint_index & USB_DIR_IN) >> 7);
		ep_num = endpoint_index & 0x0F;
		ep = xhci_verify_get_ep_by_address(udev, direction, ep_num);
		if(ep == NULL)
		{
			write_log(ctler, "Fail: the target endpoint doesn't exist or enabled\n");
			ret = -EINVAL;
			goto out;
		}

		if(usb_endpoint_type(&ep->desc) != USB_ENDPOINT_XFER_ISOC)
		{
			write_log(ctler, "Fail: the target endpoint is not isochronous endpoint\n");
			ret = -EINVAL;
			goto out;
		}
	}
	else
	{
		for(i = 0; i < MAX_POOL_ENTRY; i++)
		{
			if(ctler->mem_pool[i].actual_num > 0)
				break;
		}
		pr_info("in the memory pool,we find the %d item\n",i);
		if(i == MAX_POOL_ENTRY)
		{
			write_log(ctler, "Fail: there is no memory for the transfer\n");
			ret = -ENOMEM;
			goto out;
		}

		transfer_length = ctler->mem_pool[i].length_in_page * PAGE_SIZE;
		while(transfer_length > (4 * PAGE_SIZE))//16k
		{
			transfer_length = transfer_length >> 1;
		}
		normal_len = transfer_length;
		normal_table = xhci_verify_create_normal_mem_table(ctler, normal_len);
		if(normal_table == NULL)
		{
			write_log(ctler, "Fail: alloc normal mem_table failed\n");
			ret = -ENOMEM;
			goto out;
		}

		special_len = 0;
		special_table = xhci_verify_create_special_mem_table(special_len);
		if(special_table == NULL)
		{
			write_log(ctler, "Fail: alloc special mem_table failed\n");
			ret = -ENOMEM;
			goto free_normal_table;
		}

		/* reset all the buffer data value */
		xhci_verify_reset_mem_pool_data(ctler);

		ret = xhci_verify_init_normal_mem_table(normal_table);
		if(ret < 0)
		{
			write_log(ctler, "Fail: init normal mem_table failed\n");
			ret = -ENOMEM;
			goto free_special_table;
		}

		ret = xhci_verify_init_special_mem_table(special_table);
		if(ret < 0)
		{
			write_log(ctler, "Fail: init normal mem_table failed\n");
			ret = -ENOMEM;
			goto free_special_table;
		}

		pr_info("\ndump mem_table for transfer length: %x\n", transfer_length);
		xhci_verify_dump_normal_mem_table(normal_table);
		xhci_verify_dump_special_mem_table(special_table);

		if(unlikely(sg_alloc_table(ctler->sgtable,
					normal_table->nents + special_table->nents, GFP_KERNEL)))
		{
			ret = -ENOMEM;
			write_log(ctler, "Fail: alloc sg table failed\n");
			goto free_special_table;
		}

		for_each_sg(ctler->sgtable->sgl, sg, normal_table->nents, i) sg_set_page(sg,
			normal_table->entry[i].page, normal_table->entry[i].length,
			normal_table->entry[i].offset);

		/* Write/Read Data To/From */
		start_sector = 60;
		sector_cnt = transfer_length / 512;

		/* Write the buffer to USB mass storage device */
		ret = xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
			sector_cnt, normal_table, special_table, 1, 0, 0);

		if(ret < 0)
			goto free_sg_table;
		xhci_verify_reset_mem_pool_data(ctler);
		xhci_verify_reset_special_mem_table_data(special_table);

		/* When read, substitute urb_enqueue */
		urb_enqueue_routine_bakeup = xhci_drv->urb_enqueue;
		xhci_drv->urb_enqueue = xhci_urb_enqueue_event_data_trb;
		/* Read the data back with the same buffer, which has been reset */
		ret = xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
			sector_cnt, normal_table, special_table, 0, 0, 0);

		xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;

		if(ret < 0)
			goto free_sg_table;

		/* verify the data now */
		if((xhci_verify_normal_mem_table(normal_table) != 0)
			|| (xhci_verify_special_mem_table(special_table) != 0))
		{
			ret = -ENOTSYNC;
			write_log(ctler, "Fail: data verify fail\n");
			pr_info("dump mem_table for transfer length: %x\n", transfer_length);
			xhci_verify_dump_normal_mem_table(normal_table);
			xhci_verify_dump_special_mem_table(special_table);
			goto free_sg_table;
		}

		if(ret == 0)
			write_log(ctler, "Success\n");
	}

  free_sg_table:
	sg_free_table(ctler->sgtable);

  free_special_table:
	if(special_table != NULL)
		xhci_verify_free_special_mem_table(special_table);

  free_normal_table:
	if(normal_table != NULL)
		xhci_verify_free_normal_mem_table(normal_table);

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:target_num
 * %d:%d
 * you can further verify the multiple segment function by copy compare
 * furthermore, we just process the request to add the segment number
 */
int set_event_seg_num(struct xhci_ctler *ctler)
{
	unsigned int cmd_code, target_num;
	int ret = 0;

	struct xhci_hcd *xhci = ctler->xhci;
	unsigned int seg_num;

	int size;
	dma_addr_t dma;
	unsigned int val;
	struct xhci_segment *seg;
	u64 val_64;
	u64 temp;
	dma_addr_t deq;

	pr_info("enter into set_event_seg_num\n");
	ret = sscanf(ctler->test.cmd_str, "%d:%d", &cmd_code, &target_num);

	ret = check_num_parameter(ctler, ret, 2);
	if(ret)
		goto out;
	ret = check_cmd_code(ctler, cmd_code, SET_EVENT_SEG_NUM);
	if(ret)
		goto out;

	seg_num = xhci_readl(xhci, &xhci->ir_set->erst_size);
	seg_num &= (~ERST_SIZE_MASK);
	pr_info("the original event ring segment num is %d\n", seg_num);

	if(seg_num >= target_num)
	{
		write_log(ctler, "Success: need not decrease the segment number\n");
		ret = 0;
		goto out;
	}

	/* we need stop the xHCI host controller first, then setup the newer event ring
	 * and then start the xHCI host controller again. according to Spec P304, we must
	 * init the register sets registers before start the controller, I think this means that
	 * xHCI host hardware will init it's internal information about event ring only when
	 * software starts the controller. may be this is the only way to change the setting about
	 * event ring
	 */
	ret = xhci_halt(xhci);
	if(ret)
	{
		write_log(ctler, "Fail: can't stop the controller\n");
		pr_info("xhci_halt execute fail\n");
		goto out;
	}

	/* Free the Event Ring Segment Table and the actual Event Ring */
	if(xhci->ir_set)
	{
		xhci_writel(xhci, 0, &xhci->ir_set->erst_size);
		xhci_write_64(xhci, 0, &xhci->ir_set->erst_base);
		xhci_write_64(xhci, 0, &xhci->ir_set->erst_dequeue);
	}
	size = sizeof(struct xhci_erst_entry) * (xhci->erst.num_entries);
	if(xhci->erst.entries)
		pci_free_consistent(ctler->pcidev, size, xhci->erst.entries, xhci->erst.erst_dma_addr);
			
	xhci->erst.entries = NULL;
	if(xhci->event_ring)
		xhci_ring_free(xhci, xhci->event_ring);
	xhci->event_ring = NULL;

	/*
	 * Event ring setup: Allocate a normal ring, but also setup
	 * the event ring segment table (ERST).  Section 4.9.3.
	 */
	pr_info("// Allocating event ring\n");
	xhci->event_ring = xhci_ring_alloc(xhci, target_num, 1, false, GFP_KERNEL);
	if(!xhci->event_ring)
	{
		write_log(ctler, "Fail: can't allocate the newer event ring"
			" with target segment num, you must reboot\n");
		ret = -ENOMEM;
		pr_info("fail to allocate the newer event ring\n");
		goto out;
	}

	if(xhci_check_trb_in_td_math(xhci, GFP_KERNEL) < 0)
	{
		write_log(ctler, "Fail: xhci check trb allignment fail, you need reboot\n");
		ret = -ENOMEM;
		pr_info("fail to execute xhci_check_trb_in_td_math\n");
		goto out;
	}

	xhci->erst.entries = pci_alloc_consistent(ctler->pcidev,
				sizeof(struct xhci_erst_entry) * target_num, &dma);
	if(!xhci->erst.entries)
	{
		write_log(ctler,
			"Fail: fail to allocate the event ring segment table, you need reboot\n");
		ret = -ENOMEM;
		pr_info("fail to allocate event ring segment tabel\n");
		goto out;
	}

	pr_info("// Allocated event ring segment table at 0x%llx\n",
		(unsigned long long) dma);

	memset(xhci->erst.entries, 0, sizeof(struct xhci_erst_entry) * target_num);
	xhci->erst.num_entries = target_num;
	xhci->erst.erst_dma_addr = dma;
	pr_info("Set ERST to 0; private num segs = %i, virt addr = %p, dma addr = 0x%llx\n",
		xhci->erst.num_entries, xhci->erst.entries,
		(unsigned long long) xhci->erst.erst_dma_addr);

	/* set ring base address and size for each segment table entry */
	for(val = 0, seg = xhci->event_ring->first_seg; val < target_num; val++)
	{
		struct xhci_erst_entry *entry = &xhci->erst.entries[val];
		entry->seg_addr = seg->dma;
		entry->seg_size = TRBS_PER_SEGMENT;
		entry->rsvd = 0;
		seg = seg->next;
	}

	/* set ERST count with the number of entries in the segment table */
	val = xhci_readl(xhci, &xhci->ir_set->erst_size);
	val &= ERST_SIZE_MASK;
	val |= target_num;
	pr_info("// Write ERST size = %i to ir_set 0 (some bits preserved)\n", val);
	xhci_writel(xhci, val, &xhci->ir_set->erst_size);

	pr_info("// Set ERST entries to point to event ring.\n");
	/* set the segment table base address */
	pr_info("// Set ERST base address for ir_set 0 = 0x%llx\n",
		(unsigned long long) xhci->erst.erst_dma_addr);
	val_64 = xhci_read_64(xhci, &xhci->ir_set->erst_base);
	val_64 &= ERST_PTR_MASK;
	val_64 |= (xhci->erst.erst_dma_addr & (u64) ~ ERST_PTR_MASK);
	xhci_write_64(xhci, val_64, &xhci->ir_set->erst_base);

	/* Set the event ring dequeue address */
	deq = xhci_trb_virt_to_dma(xhci->event_ring->deq_seg, xhci->event_ring->dequeue);
	if(deq == 0 && !in_interrupt())
		pr_info("WARN something wrong with SW event ring " "dequeue ptr.\n");
	/* Update HC event ring dequeue pointer */
	temp = xhci_read_64(xhci, &xhci->ir_set->erst_dequeue);
	temp &= ERST_PTR_MASK;
	/* Don't clear the EHB bit (which is RW1C) because
	 * there might be more events to service.
	 */
	temp &= ~ERST_EHB;
	xhci_write_64(xhci, ((u64) deq & (u64) ~ ERST_PTR_MASK) | temp,
		&xhci->ir_set->erst_dequeue);
	pr_info("Wrote ERST address to ir_set 0.\n");

	/* set the interrupt moderation register */
	pr_info("// Set the interrupt modulation register\n");
	temp = xhci_readl(xhci, &xhci->ir_set->irq_control);
	temp &= ~ER_IRQ_INTERVAL_MASK;
	temp |= (u32) 160;
	xhci_writel(xhci, temp, &xhci->ir_set->irq_control);

	/* set the interrupt management register */
	temp = xhci_readl(xhci, &xhci->ir_set->irq_pending);
	pr_info("// Enabling event ring interrupter %p by writing 0x%x to irq_pending\n",
		xhci->ir_set, (unsigned int) ER_IRQ_ENABLE(temp));
	xhci_writel(xhci, ER_IRQ_ENABLE(temp), &xhci->ir_set->irq_pending);

	/* start the controller with interrupt enable */

	/* Set the HCD state before we enable the irqs */
	ctler->hcd->state = HC_STATE_RUNNING;
	xhci->shared_hcd->state= HC_STATE_RUNNING; //weitao add
	temp = xhci_readl(xhci, &xhci->op_regs->command);
	temp |= (CMD_EIE);
	pr_info("// Enable interrupts, cmd = 0x%x.\n", (u32) temp);
	xhci_writel(xhci, temp, &xhci->op_regs->command);

	ret = xhci_start(xhci);
	if(ret)
	{
		write_log(ctler, "Fail: can't restart the controller\n");
		pr_info("xhci_start execute fail\n");
		goto out;
	}

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string
 * %d:0x%x
 * the target device must be a mass storage device,
 * and we will write data to the storage device,
 * then read data from the same address.
 * when reading, we set the interrpt blocking flag
 * and verify whether partial TRB is transferred.
 * originally, xHCI driver use single event ring with 64 TRBs
 * and single transfer ring TRB with 64 TRBs too.
 * In order to verify the event full,
 * we need enlarge the bulk in endpoint transfer ring firstly.
 * we will transfer 1K byte data with one transfer TRB.
 *
 */
int verify_event_full(struct xhci_ctler *ctler)
{
	int i, ret;
	unsigned int temp;
	unsigned int cmd_code, route_string;

	struct usb_device *udev;
	struct xhci_hcd *xhci = ctler->xhci;
	struct hc_driver *xhci_drv = ctler->xhci_drv;

	unsigned int transfer_length;
	unsigned int special_len, normal_len;
	struct scatterlist *sg;
	struct xhci_mem_table *normal_table, *special_table;

	void *urb_enqueue_routine_bakeup;

	struct mass_storage_dev_info mass_stor_info;
	unsigned int start_sector, sector_cnt;

	unsigned int ep_index;
	struct xhci_virt_device *virt_dev = NULL;
	struct xhci_virt_ep *virt_ep = NULL;
	struct xhci_ring *ring;
	struct xhci_segment *next_seg;
	unsigned int seg_num;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x", &cmd_code, &route_string);
	if(check_num_parameter(ctler, ret, 2))
		goto out;
	if(check_cmd_code(ctler, cmd_code, VERIFY_EVENT_FULL))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	ret = xhci_verify_get_mass_storage_info(udev, &mass_stor_info);
	if(ret < 0)
	{
		pr_info("the target device is not usb mass storage device\n");
		write_log(ctler, "Fail: the target device is not mass storage device\n");
		ret = -EINVAL;
		goto out;
	}

	virt_dev = xhci->devs[udev->slot_id];
	ep_index = xhci_get_endpoint_index(&(mass_stor_info.bulk_in->desc));
	virt_ep = &(virt_dev->eps[ep_index]);
	if(virt_ep == NULL)
	{
		write_log(ctler, "Fail: the bulk in endpoint has no virtual target\n");
		ret = -EINVAL;
		goto out;
	}
	ring = virt_ep->ring;
	if(ring == NULL)
	{
		write_log(ctler, "Fail: the bulk in endpoint has no transfer ring\n");
		ret = -EINVAL;
		goto out;
	}

	next_seg = ring->first_seg->next;
	seg_num = 1;
	while(next_seg != ring->first_seg)
	{
		seg_num++;
		next_seg = next_seg->next;
	}

	if(seg_num <= 1)
	{
		write_log(ctler,
			"Fail: the bulk in endpoint has only one segment,"
			"need enlarge it firstly\n");
		ret = -EINVAL;
		pr_info("the bulk in endpoint has only on segment\n");
		goto out;
	}

	temp = xhci_readl(xhci, &xhci->ir_set->erst_size);
	temp &= 0xFFFF;
	if(temp > 1)
	{
		write_log(ctler,
			"Fail: the event ring has more than one"
			" segment, and can't trigger the event full condition\n");
		ret = -EINVAL;
		goto out;
	}

	pr_info("PAGE_SIZE is %ld\n", PAGE_SIZE);/* 4096 = 4k */
	transfer_length = (16 + 8) * PAGE_SIZE;
	normal_len = transfer_length;
	normal_table = xhci_verify_create_normal_mem_table(ctler, normal_len);
	if(normal_table == NULL)
	{
		write_log(ctler, "Fail: alloc normal mem_table failed\n");
		ret = -ENOMEM;
		goto out;
	}

	special_len = 0;
	special_table = xhci_verify_create_special_mem_table(special_len);
	if(special_table == NULL)
	{
		write_log(ctler, "Fail: alloc special mem_table failed\n");
		ret = -ENOMEM;
		goto free_normal_table;
	}

	/* Reset all the buffer data value */
	xhci_verify_reset_mem_pool_data(ctler);

	ret = xhci_verify_init_normal_mem_table(normal_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	ret = xhci_verify_init_special_mem_table(special_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	pr_info("\nDump mem_table for transfer length: %x\n", transfer_length);
	xhci_verify_dump_normal_mem_table(normal_table);
	xhci_verify_dump_special_mem_table(special_table);

	if(unlikely(sg_alloc_table(ctler->sgtable,
		normal_table->nents + special_table->nents, GFP_KERNEL)))
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc sg table failed\n");
		goto free_special_table;
	}

	for_each_sg(ctler->sgtable->sgl, sg, normal_table->nents, i) sg_set_page(sg,
		normal_table->entry[i].page, normal_table->entry[i].length,
		normal_table->entry[i].offset);

	start_sector = 60;
	sector_cnt = transfer_length / 512;

	/* Write the buffer to USB mass storage device */
	ret = xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
		sector_cnt, normal_table, special_table, 1, 0, 0);

	if(ret < 0)
		goto free_sg_table;
	xhci_verify_reset_mem_pool_data(ctler);
	xhci_verify_reset_special_mem_table_data(special_table);

	urb_enqueue_routine_bakeup = xhci_drv->urb_enqueue;
	xhci_drv->urb_enqueue = xhci_urb_enqueue_with_little_trb;
	/* Read the data back with the same buffer, which has been reset */
	ret = xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
		sector_cnt, normal_table, special_table, 0, 1, 0);

	xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;

	if(ret < 0)
		goto free_sg_table;

	/* Verify the data now */
	if((xhci_verify_normal_mem_table(normal_table) != 0)
		|| (xhci_verify_special_mem_table(special_table) != 0))
	{
		ret = -ENOTSYNC;
		write_log(ctler, "Fail: data verify fail\n");
		pr_info("dump mem_table for transfer length: %x\n", transfer_length);
		xhci_verify_dump_normal_mem_table(normal_table);
		xhci_verify_dump_special_mem_table(special_table);
		goto free_sg_table;
	}

	if(ret == 0)
		write_log(ctler, "Success\n");

  free_sg_table:
	sg_free_table(ctler->sgtable);

  free_special_table:
	if(special_table != NULL)
		xhci_verify_free_special_mem_table(special_table);

  free_normal_table:
	if(normal_table != NULL)
		xhci_verify_free_normal_mem_table(normal_table);
  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:target_interval
 * %d:%d
 * target_interval is based on ms(microsecond)
 * we can judge this feature by the calling frequency of xhci_irq routine
 * uner heavy I/O condition.
 */
int set_intr_moderation(struct xhci_ctler *ctler)
{
	unsigned int cmd_code, target_interval;
	int ret = 0;

	unsigned int value;
	unsigned int temp;

	struct xhci_hcd *xhci = ctler->xhci;

	ret = sscanf(ctler->test.cmd_str, "%d:%d", &cmd_code, &target_interval);

	if(ret != 2)
	{
		write_log(ctler,
			"Fail: Missing arguments, 2 arguments required, but only %d received\n", ret);
		ret = -EINVAL;
		goto out;

	}
	else if(cmd_code != SET_INTR_MODERATION)
	{
		write_log(ctler,
			"Fail: Command code mismatch, expect cmd[%d],but receive cmd[%d]\n",
			SET_INTR_MODERATION, cmd_code);
		ret = -EINVAL;
		goto out;
	}

	value = xhci_readl(xhci, &xhci->ir_set->irq_control);
	value &= ER_IRQ_INTERVAL_MASK;
	target_interval *= (1000 * 1000 / 250);
	if(target_interval > 0xFFFF)
	{
		write_log(ctler,
			"Fail: the target interval is too large, it should not execeed 16\n");
		ret = -EINVAL;
		goto out;
	}
	pr_info("the original event interval is 0x%x, the target is 0x%x\n", value,
		target_interval);
	if(value == target_interval)
	{
		write_log(ctler, "Success: the current value is just the target value\n");
		ret = 0;
		goto out;
	}

	/* we need stop the xHCI host controller first, then setup the newer interval.
	 * and then start the xHCI host controller again. according to Spec P304, we must
	 * init the register sets registers before start the controller, I think this means that
	 * xHCI host hardware will init it's internal information about event ring only when
	 * software starts the controller. may be this is the only way to change the setting about
	 * event ring
	 */
	/*ret = xhci_halt(xhci);
 	if (ret)
 	{
	   write_log(ctler, "Fail: can't stop the controller\n");
	   pr_info( "xhci_halt execute fail\n");
	   goto out;
	} 
	msleep(5000);*/
	/* set the interrupt moderation register */
	pr_info("// Set the interrupt modulation register\n");
	temp = xhci_readl(xhci, &xhci->ir_set->irq_control);
	temp &= ~ER_IRQ_INTERVAL_MASK;
	temp |= target_interval;
	xhci_writel(xhci, temp, &xhci->ir_set->irq_control);

	/* Set the HCD state before we enable the irqs */
	/*ctler->hcd->state = HC_STATE_RUNNING;
	xhci->shared_hcd->state= HC_STATE_RUNNING; //weitao add*/
	/*ret = xhci_start(xhci);
	if (ret)
	{
	   write_log(ctler, "Fail: can't restart the controller\n");
	   pr_info( "xhci_start execute fail\n");
	   goto out;
	}*/

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:u1_timeout_value:u2_timeout_value:u1_enable:u2_enable
 * %d:0x%x:%d:%d:%d:%d
 *
 */
int set_u1u2_timeout(struct xhci_ctler *ctler)
{
	struct usb_device *udev;
	unsigned int cmd_code, route_string;
	unsigned int u1_value, u2_value, u1_enable, u2_enable;
	u16 dev_status;
	unsigned int u1_target, u2_target;
	unsigned port_index;
	int ret = 0;
	struct usb_host_bos *bos = NULL;
	struct usb_ss_cap_descriptor *ss_cap=NULL;

	struct xhci_hcd *xhci = ctler->xhci;
	u32 value;
	u32 __iomem *addr;
	

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x:%d:%d:%d:%d", &cmd_code, &route_string,
		&u1_value, &u2_value, &u1_enable, &u2_enable);
	if(check_num_parameter(ctler, ret, 6))
		goto out;
	if(check_cmd_code(ctler, cmd_code, SET_U1U2_TIMEOUT))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;
	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	if(udev->speed != USB_SPEED_SUPER)
	{
		write_log(ctler,
			"Fail: the target device is not SS device,it doesn't support Ux state\n");
		ret = -EINVAL;
		goto out;
	}

	if(udev->bos)
	{
		bos = udev->bos;
		if(bos->ss_cap)
		{
			ss_cap = bos->ss_cap;
		}
		else
		{
			pr_info("ss_cap is NULL\n");
		}
	}
	else
	{
		pr_info("udev->bos is NULL\n");
	}

	pr_info("ss_cap->bmAttributes is 0x%x\n", ss_cap->bmAttributes);
	pr_info("ss_cap->bU1devExitLat is 0x%x\n",ss_cap->bU1devExitLat);
	pr_info("ss_cap->bU2DevExitLat is 0x%x\n",ss_cap->bU2DevExitLat);
	
	
	/*ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_GET_STATUS,
		USB_DIR_IN | USB_RECIP_DEVICE, 0, 0, &dev_status, 2, 0);
	pr_info("dev_status is 0x%x\n", dev_status);
	if(ret < 0 || ret != 2)
	{
		write_log(ctler, "Fail: fail to get status of target device\n");
		ret = -EPIPE;
		goto out;
	}*/
	pr_info("u1_value: %d, u2_value: %d, u1_enable: %d, u2_enable: %d\n",
				u1_value, u2_value, u1_enable, u2_enable);

	/* Firstly, set the U1 timeout, current xhci_hub_control doesn't support
	 * SetPortFeature(U1_TIMEOUT), we need do it by ourselves
	 */
	//if(udev->parent == ctler->hcd->self.root_hub)
	if(udev->parent != NULL )
	{
		pr_info(" on root hub\n");
		port_index = route_string & 0xFF;
		port_index--;
		addr = &xhci->op_regs->port_power_base + NUM_PORT_REGS * port_index;
		pr_info("port addr is 0x%p\n", addr);
		value = xhci_readl(xhci, addr);
		pr_info("value read form port is 0x%x\n", value);
		value &= 0xFFFFFF00;
		value |= u1_value;
		xhci_writel(xhci, value, addr);
		pr_info("after write: we read from it: 0x%x\n", xhci_readl(xhci, addr));
	}
	/*else
	{
		pr_info("not on root hub \n");
		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_U1_TIMEOUT,
			((u1_value << 8) | udev->portnum), NULL, 0, 0);
		if(ret != 0)
		{
			write_log(ctler, "Fail: fail to issue SetPortFeature(U1_TIMEOUT) to Hub\n");
			ret = -EPIPE;
			goto out;
		}
	}*/

	/* secondly, set the U2 timeout. current xhci_hub_control doesn't support
	 * SetPortFeature(U2_TIMEOUT), we need do it by ourselves
	 */
	//if(udev->parent == ctler->hcd->self.root_hub)
	if(udev->parent != NULL )
	{
		pr_info(" on root hub second\n");
		port_index = route_string & 0xFF;
		port_index--;
		addr = &xhci->op_regs->port_power_base + NUM_PORT_REGS * port_index;
		value = xhci_readl(xhci, addr);
		value &= 0xFFFF00FF;
		value |= (u2_value << 8);
		xhci_writel(xhci, value, addr);
		pr_info("after write: we read from it: 0x%x\n", xhci_readl(xhci, addr));
	}
	/*else
	{
		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_U2_TIMEOUT,
			((u2_value << 8) | udev->portnum), NULL, 0, 0);
		if(ret != 0)
		{
			write_log(ctler, "Fail: fail to issue SetPortFeature(U2_TIMEOUT) to Hub\n");
			ret = -EPIPE;
			goto out;
		}
	}*/

	/* thirdly, we should enable/disable the U1_enable feature of this device */
	if(u1_enable)
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_SET_FEATURE, 0,
			USB_DEVICE_U1_ENABLE, 0, NULL, 0, 0);
	else
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_CLEAR_FEATURE, 0,
			USB_DEVICE_U1_ENABLE, 0, NULL, 0, 0);
	if(ret != 0)
	{
		write_log(ctler, "Fail: fail to enable/disable U1 in device side\n");
		ret = -EPIPE;
		goto out;
	}

	/* thirdly, we should enable/disable the U2_enable feature of this device */
	if(u2_enable)
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_SET_FEATURE, 0,
			USB_DEVICE_U2_ENABLE, 0, NULL, 0, 0);
	else
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_CLEAR_FEATURE, 0,
			USB_DEVICE_U2_ENABLE, 0, NULL, 0, 0);
	if(ret != 0)
	{
		write_log(ctler, "Fail: fail to enable/disable U2 in device side\n");
		ret = -EPIPE;
		goto out;
	}

	/* get the device status to judge whether the setting is OK */
	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_GET_STATUS,
		USB_DIR_IN | USB_RECIP_DEVICE, 0, 0, &dev_status, 2, 0);
	
	pr_info("dev_status is 0x%x\n", dev_status);
	if(ret < 0 || ret != 2)
	{
		write_log(ctler, "Fail: fail to get status of target device\n");
		ret = -EPIPE;
		goto out;
	}
	u1_target = u2_target = 0;
	u1_target = (dev_status >> 2) & 0x1;
	u2_target = (dev_status >> 3) & 0x1;
	pr_info("dev_status is 0x%x\n", dev_status);
	pr_info("u1_target: %d, u2_target: %d, u1_enable: %d, u2_enable: %d\n",
		u1_target, u2_target, u1_enable, u2_enable);
	if(u1_target != u1_enable || u2_target != u2_enable)
	{
		write_log(ctler,
			"Fail: the target enable state doesn't match the expect, "
			"the device status is 0x%x\n", dev_status);
		ret = -EPIPE;
		goto out;
	}

	ret = 0;
	write_log(ctler,
		"Success: setting successfully, and you can verify the Ux by Catc\n");

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:target_state
 * %d:0x%x:%d
 * lpm: link power management
 */
int set_lpm_state(struct xhci_ctler *ctler)
{
	struct usb_device *udev;
	unsigned int cmd_code, route_string, target_state;
	unsigned int port_index;
	unsigned int temp;

	int ret = 0;
	u32 port_status_change;
	u32 port_status, port_change;

	struct xhci_hcd *xhci = ctler->xhci;
	u32 value = 0;
	u32 __iomem *addr;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x:%d", &cmd_code, &route_string,
		&target_state);

	if(ret != 3)
	{
		write_log(ctler,
			"Fail: Missing arguments, 3 arguments required, but only %d received\n",
			ret);

		ret = -EINVAL;
		goto out;

	}
	else if(cmd_code != SET_LPM_STATE)
	{
		write_log(ctler,
			"Fail: Command code mismatch, expect cmd[%d],but receive cmd[%d]\n",
			SET_LPM_STATE, cmd_code);

		ret = -EINVAL;
		goto out;
	}

	/* need verify the device exist and endpoint valid */
	port_index = route_string & 0xFF;
	if(route_string == 0 || port_index == 0)
	{
		write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);

		ret = -EINVAL;
		goto out;
	}
	temp = route_string;
	while(temp != 0)
	{
		port_index = temp & 0xFF;
		temp >>= 4;
		if(port_index == 0 && temp != 0)
		{
			write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);
			ret = -EINVAL;
			goto out;
		}
	}
	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(udev == NULL)
	{
		write_log(ctler, "Fail: the target USB device doesn't exist\n");
		ret = -EINVAL;
		goto out;
	}

	if(udev->speed != USB_SPEED_SUPER)
	{
		if(target_state != 0 && target_state != 3)
		{
			write_log(ctler,
				"Fail: the target device is not SS device, "
				"it doesn't support the target state\n");
			ret = -EINVAL;
			goto out;
		}
	}
	else
	{
		pr_info("udev->parent              : %p\n", udev->parent);
		pr_info("ctler->hcd->self.root_hub : %p\n", ctler->hcd->self.root_hub);
		pr_info("xhci->share_hcd->self.root_hub : %p\n", xhci->shared_hcd->self.root_hub);
		if(udev->parent == xhci->shared_hcd->self.root_hub)
		{
			if(target_state != 0 && target_state != 3 && target_state != 5)
			{
				write_log(ctler, "Fail: device doesn't support the target state\n");
				ret = -EINVAL;
				goto out;
			}
		}
		else
		{
			if(target_state > 5 || target_state == 4)
			{
				write_log(ctler, "Fail: device doesn't support the target state\n");
				ret = -EINVAL;
				goto out;
			}
		}
	}
	//weitao modify....on the root hub
	if(udev->parent == ctler->hcd->self.root_hub || udev->parent == xhci->shared_hcd->self.root_hub)
	{
		port_index = route_string & 0xFF;
		port_index--;
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * port_index;
		value = xhci_readl(xhci, addr);

		if((target_state == 5) && (((value & PORT_PLS_MASK) >> 5) != 0x04))
		{
			write_log(ctler, "Success: the link is not in disable state\n");
			ret = 0;
			goto out;
		}

		if(((value & PORT_PLS_MASK) >> 5) == target_state)
		{
			write_log(ctler, "Success: the link is already in the target state\n");
			ret = 0;
			goto out;
		}

		if((target_state != 5) && ((value & PORT_PE) == 0 || (value & PORT_RESET)))
		{
			write_log(ctler, "Fail: the target device is disabled or be resetting\n");
			ret = -EINVAL;
			goto out;
		}

		if(target_state >= 3)
		{
			value = xhci_readl(xhci, addr);
			value = xhci_port_state_to_neutral(value);
			pr_info("the original value with neutral is 0x%x\n", value);
			value &= ~PORT_PLS_MASK;
			value |= (PORT_LINK_STROBE | (target_state << 5));
			pr_info("write value 0x%x to the target register\n", value);
			xhci_writel(xhci, value, addr);

			//xhci->suspended_ports[port_index >> 5] |= 1 << (port_index & (31));
			xhci->bus_state[0].suspended_ports |= 1 << (port_index & (31));
			//xhci->bus_state[1].port_c_suspend[port_index >> 5] |= 1 << (port_index & 31);

			usb_set_device_state(udev, USB_STATE_SUSPENDED);

			msleep(20);
			value = xhci_readl(xhci, addr);
			pr_info("after write, the register value is 0x%x\n", value);
		}
		else
		{
			value = xhci_readl(xhci, addr);
			if(DEV_SUPERSPEED(value))
			{
				value = xhci_port_state_to_neutral(value);
				pr_info("the original value with neutral is 0x%x\n", value);
				value &= ~PORT_PLS_MASK;
				value |= PORT_LINK_STROBE | XDEV_U0;
				//value |= PORT_PLC;
				pr_info("write value 0x%x to the target register\n", value);
				xhci_writel(xhci, value, addr);
				//xhci->port_c_suspend[port_index >> 5] |= 1 << (port_index & 31);

				xhci->bus_state[0].port_c_suspend |= 1 << (port_index & 31);
				//xhci->bus_state[1].port_c_suspend[port_index >> 5] |= 1 << (port_index & 31);

				usb_set_device_state(udev, USB_STATE_CONFIGURED);
				msleep(20);
				value = xhci_readl(xhci, addr);
				pr_info("after write, the register value is 0x%x\n", value);
			}
			else
			{
				value = xhci_port_state_to_neutral(value);
				pr_info("2.0 the original value with neutral is 0x%x\n", value);
				value &= ~PORT_PLS_MASK;
				value |= PORT_LINK_STROBE | XDEV_RESUME;
				pr_info("write value 0x%x to the target register\n", value);
				xhci_writel(xhci, value, addr);

				msleep(20);

				value = xhci_readl(xhci, addr);
				value = xhci_port_state_to_neutral(value);
				pr_info("the original value with neutral is 0x%x\n", value);
				value &= ~PORT_PLS_MASK;
				value |= PORT_LINK_STROBE | XDEV_U0;
				//value |= PORT_PLC;
				pr_info("write value 0x%x to the target register\n", value);
				xhci_writel(xhci, value, addr);
				//xhci->port_c_suspend[port_index >> 5] |= 1 << (port_index & 31);

				xhci->bus_state[0].port_c_suspend |= 1 << (port_index & 31);
				//xhci->bus_state[1].port_c_suspend[port_index >> 5] |= 1 << (port_index & 31);

				usb_set_device_state(udev, USB_STATE_CONFIGURED);

				msleep(20);
				value = xhci_readl(xhci, addr);
				pr_info("after write, the register value is 0x%x\n", value);
			}
		}

		if(((value & PORT_PLS_MASK) >> 5) != target_state)
		{
			write_log(ctler,
				"Fail: the link doesn't enter the target state, now it is %d\n",
				((value & PORT_PLS_MASK) >> 5));
			ret = -EPIPE;
			goto out;
		}

		ret = 0;
		write_log(ctler, "Success\n");
		goto out;
	}

	ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
		USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, udev->portnum,
		&port_status_change, 4, 0);
	if(ret < 0 || ret != 4)
	{
		write_log(ctler, "Fail: can't get the port status and change information\n");
		ret = -EPIPE;
		goto out;
	}
	port_status = port_status_change & 0x0FFFF;
	port_change = ((port_status_change >> 16) & 0x0FFFF);

	if(udev->parent->speed != USB_SPEED_SUPER)
	{
		if((target_state == 0) && ((port_status & USB_PORT_STAT_SUSPEND) == 0))
		{	
		    pr_info("the port_status is %x\n",port_status);
			write_log(ctler, "Success: the target device is already in U0 state\n");
			ret = 0;
			goto out;
		}
		if((target_state == 3) && ((port_status & USB_PORT_STAT_SUSPEND) != 0))
		{
			write_log(ctler, "Success: the target device is already in U3 state\n");
			ret = 0;
			goto out;
		}
		if(target_state == 0)
		{
			ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
				USB_REQ_CLEAR_FEATURE, USB_RT_PORT, USB_PORT_FEAT_SUSPEND, udev->portnum,
				NULL, 0, 0);
			if(ret < 0)
			{
				write_log(ctler, "Fail: fail to resume the target device\n");
				ret = -EPIPE;
				goto out;
			}
			msleep(25);
			/*need issue get_port_status to terminate the resume signal .for to read port status..*/
			ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
				USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, udev->portnum,
				&port_status_change, 4, 0);
			if(ret < 0 || ret != 4)
			{
				write_log(ctler,
					"Fail: can't get the port status and change information\n");
				ret = -EPIPE;
				goto out;
			}
			msleep(10);

			port_status = port_status_change & 0x0FFFF;
			port_change = ((port_status_change >> 16) & 0x0FFFF);
			if(port_change & USB_PORT_STAT_C_SUSPEND)
			{
				ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
					USB_REQ_CLEAR_FEATURE, USB_RT_PORT, USB_PORT_FEAT_C_SUSPEND,
					udev->portnum, NULL, 0, 0);
				if(ret < 0)
				{
					write_log(ctler, "Fail: fail to cleare the suspend change\n");
					ret = -EPIPE;
					goto out;
				}
			}

			if(udev->do_remote_wakeup)
			{
				ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
					USB_REQ_CLEAR_FEATURE, USB_RECIP_DEVICE, USB_DEVICE_REMOTE_WAKEUP, 0,
					NULL, 0, 0);
				if(ret < 0)
				{
					write_log(ctler, "Fail: fail to disable remote wakeup function\n");
					ret = -EPIPE;
					goto out;
				}
			}
		}
		else
		{
			if(udev->do_remote_wakeup)
			{
				ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_SET_FEATURE,
					USB_RECIP_DEVICE, USB_DEVICE_REMOTE_WAKEUP, 0, NULL, 0, 0);
				if(ret < 0)
				{
					write_log(ctler, "Fail: fail to set remote wakeup function\n");
					ret = -EPIPE;
					goto out;
				}
			}

			ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
				USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_SUSPEND, udev->portnum,
				NULL, 0, 0);
			if(ret < 0)
			{
				write_log(ctler, "Fail: fail to suspend the target device\n");
				ret = -EPIPE;
				goto out;
			}
			msleep(10);
		}
	}
	else
	{	pr_info("device not in the root hub,in the 3.0 speed hub\n");
		if(((port_status & PORT_PLS_MASK) >> 5) == target_state)
		{
			write_log(ctler, "Success: the link is still in the target state\n");
			ret = 0;
			goto out;
		}
		if((target_state == 5) && (((port_status & PORT_PLS_MASK) >> 5) != 0x04))
		{
			write_log(ctler, "Success: the link is not in disable state\n");
			ret = 0;
			goto out;
		}

		if((target_state != 5) && ((port_status & PORT_PE) == 0 || (value & PORT_RESET)))
		{
			write_log(ctler, "Fail: the target device is disabled or be resetting\n");
			ret = -EINVAL;
			goto out;
		}

		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_L1,
			((target_state << 8) | udev->portnum), NULL, 0, 0);
		if(ret < 0)
		{
			write_log(ctler, "Fail: fail to set the link into the target state\n");
			ret = -EPIPE;
			goto out;
		}

		msleep(10);

		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, udev->portnum,
			&port_status_change, 4, 0);
		if(ret < 0 || ret != 4)
		{
			write_log(ctler, "Fail: can't get the port status and change information\n");
			ret = -EPIPE;
			goto out;
		}

		port_status = port_status_change & 0x0FFFF;
		port_change = ((port_status_change >> 16) & 0x0FFFF);
		if(((port_status & PORT_PLS_MASK) >> 5) != target_state)
		{
			write_log(ctler, "Fail: the link is not in the target state\n");
			ret = -EPIPE;
			goto out;
		}
		if(port_change & 0x20)   ////////////??///
		{
			ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
				//FIXME@PATIENCE : USB_PORT_FEAT_C_LINK_STATE -->USB_PORT_FEAT_C_PORT_LINK_STATE, it's 25.
				USB_REQ_CLEAR_FEATURE, USB_RT_PORT, USB_PORT_FEAT_C_PORT_LINK_STATE,
				udev->portnum, NULL, 0, 0);
			if(ret < 0)
			{
				write_log(ctler, "Fail: fail to cleare the suspend change\n");
				ret = -EPIPE;
				goto out;
			}
		}
	}

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:interface_index
 * %d:0x%x:%d
 *
 */
int function_suspend(struct xhci_ctler *ctler)
{
	struct usb_device *udev;
	unsigned int cmd_code, route_string, intf_index;
	unsigned int port_index;
	unsigned int temp;

	int ret = 0;
	int i;

	u8 is_support_remote_wakeup;
	struct usb_interface *intf;
	u16 intf_status;
	u32 port_status_change;
	u32 port_status, port_change;

	u32 reg_value;

	struct xhci_hcd *xhci = ctler->xhci;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x:%d", &cmd_code, &route_string, &intf_index);

	if(ret != 3)
	{
		write_log(ctler,
			"Fail: Missing arguments, 3 arguments required, but only %d received\n", ret);
		ret = -EINVAL;
		goto out;

	}
	else if(cmd_code != FUNC_SUSPEND)
	{
		write_log(ctler,
			"Fail: Command code mismatch, expect cmd[%d],but receive cmd[%d]\n",
			FUNC_SUSPEND, cmd_code);

		ret = -EINVAL;
		goto out;
	}

	/* need verify the device exist and endpoint valid */
	port_index = route_string & 0xFF;
	if(route_string == 0 || port_index == 0)
	{
		write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);

		ret = -EINVAL;
		goto out;
	}
	
	temp = route_string;
	while(temp != 0)
	{
		port_index = temp & 0xFF;
		temp >>= 8;
		if(port_index == 0 && temp != 0)
		{
			write_log(ctler, "Fail: route string[0x%x] invalid\n", route_string);
			ret = -EINVAL;
			goto out;
		}
	}
	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(udev == NULL)
	{
		write_log(ctler, "Fail: the target USB device doesn't exist\n");
		ret = -EINVAL;
		goto out;
	}

	if((udev->speed != USB_SPEED_SUPER) && (udev->parent->speed != USB_SPEED_SUPER))
	{
		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, udev->portnum,
			&port_status_change, 4, 0);

		if(ret < 0 || ret != 4)
		{
			write_log(ctler, "Fail: can't get the port status and change information\n");
			ret = -EPIPE;
			goto out;
		}
		port_status = port_status_change & 0x0FFFF;
		port_change = ((port_status_change >> 16) & 0x0FFFF);

		if((port_status & USB_PORT_STAT_SUSPEND) != 0)
		{
			write_log(ctler, "Success: the target device is already in suspend state\n");
			ret = 0;
			goto out;
		}

		if(udev->actconfig->desc.bmAttributes & USB_CONFIG_ATT_WAKEUP)
		{
			ret =
				usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_SET_FEATURE,
				USB_RECIP_DEVICE, USB_DEVICE_REMOTE_WAKEUP, 0, NULL, 0, 0);
			if(ret < 0)
			{
				write_log(ctler, "Fail: fail to disable remote wakeup function\n");
				ret = -EPIPE;
				goto out;
			}
		}

		ret =
			usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_SET_FEATURE, USB_RT_PORT, USB_PORT_FEAT_SUSPEND, udev->portnum, NULL,
			0, 0);
		if(ret < 0)
		{
			write_log(ctler, "Fail: fail to suspend the target device\n");
			ret = -EPIPE;
			goto out;
		}

		msleep(10);

		write_log(ctler, "Success\n");
		ret = 0;
		goto out;
	}
	//only 3.0 speed will go to this area...
	for(i = 0; i < udev->actconfig->desc.bNumInterfaces; i++)
	{
		intf = udev->actconfig->interface[i];
		if(intf == NULL)
			continue;

		if((intf->cur_altsetting != NULL)
			&& (intf->cur_altsetting->desc.bInterfaceNumber == intf_index))
			break;
	}
	if(i == udev->actconfig->desc.bNumInterfaces)
	{
		write_log(ctler, "Fail: the target interface doesn't exist\n");
		ret = -EINVAL;
		goto out;
	}

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_GET_STATUS,
		USB_DIR_IN | USB_RECIP_INTERFACE, 0, intf_index, &intf_status, 2, 0);
	if(ret < 0 || ret != 2)
	{
		write_log(ctler, "Fail: fail to get status of target interface\n");
		ret = -EPIPE;
		goto out;
	}

	if(intf_status & 0x01)
		is_support_remote_wakeup = 1;
	else
		is_support_remote_wakeup = 0;
    pr_info("the 3.0 interface remote_wakeup ability is %d\n",is_support_remote_wakeup);
	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_SET_FEATURE,
		USB_RECIP_INTERFACE, 0, ((is_support_remote_wakeup << 9) | intf_index), NULL, 0,
		0);
	if(ret != 0)
	{
		write_log(ctler, "Fail: fail to issue function suspend request to device\n");
		ret = -EPIPE;
		goto out;
	}

	msleep(10);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_GET_STATUS,
		USB_DIR_IN | USB_RECIP_INTERFACE, 0, intf_index, &intf_status, 2, 0);
	if(ret < 0 || ret != 2)
	{
		write_log(ctler, "Fail: fail to get status of target interface\n");
		ret = -EPIPE;
		goto out;
	}

	if(is_support_remote_wakeup && ((intf_status & 0x02) == 0))
	{
		write_log(ctler, "Success: but fail to enable the function remote wakeup\n");
		ret = 0;
		goto out;
	}

	/* enable the remote wakeup notification event */
	if(is_support_remote_wakeup)
	{
		reg_value = xhci_readl(xhci, &xhci->op_regs->dev_notification);
		reg_value |= 0x02;
		xhci_writel(xhci, reg_value, &xhci->op_regs->dev_notification);
	}

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string
 * %d:0x%x
 * enable latency tolerance message
 */
int enable_ltm(struct xhci_ctler *ctler)
{
	int ret = 0;
	unsigned int cmd_code, route_string;
	struct usb_device *udev;
	struct xhci_hcd *xhci = ctler->xhci;
	struct xhci_command *command = NULL;

	unsigned int value;
	u16 len;
	u32 reg_value;
	u16 dev_status;

	struct usb_bos_descriptor *bos = NULL;
	unsigned char *buf;
	struct usb_dev_cap_header *header;
	struct usb_ss_cap_descriptor *ss_desc;
	u8 is_support_LTM = 0;

	unsigned long flags;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x", &cmd_code, &route_string);

	if(check_num_parameter(ctler, ret, 2))
		goto out;
	if(check_cmd_code(ctler, cmd_code, ENABLE_LTM))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	if(udev->speed != USB_SPEED_SUPER)
	{
		write_log(ctler, "Fail: the target device is not SS device\n");
		ret = -EINVAL;
		goto out;
	}

	value = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
	if(!(HCC_LTC(value)))
	{
		write_log(ctler, "Success: the controller doesn't support LTM\n");
		ret = 0;
		goto out;
	}

	bos = (struct usb_bos_descriptor *)kmalloc(32, GFP_KERNEL);
	if(bos == NULL)
	{
		write_log(ctler, "Fail: fail to allocate the bos descriptor\n");
		ret = -ENOMEM;
		goto out;
	}
	ret = usb_get_descriptor(udev, USB_DT_BOS, 0, bos, 4);
	if(ret < 0 || ret != 4)
	{
		write_log(ctler, "Fail: fail to get the BOS descriptor header\n");
		ret = -EPIPE;
		goto out;
	}

	len = bos->wTotalLength;

	if(bos->wTotalLength > 32)
	{
		kfree(bos);
		bos = NULL;
		bos = (struct usb_bos_descriptor *)kmalloc(len, GFP_KERNEL);
		if(bos == NULL)
		{
			write_log(ctler, "Fail: fail to allocate the bos descriptor\n");
			ret = -ENOMEM;
			goto out;
		}
	}

	ret = usb_get_descriptor(udev, USB_DT_BOS, 0, bos, len);
	if(ret < 0 || ret != len)
	{
		write_log(ctler, "Fail: fail to get the BOS descriptor\n");
		ret = -EPIPE;
		goto out;
	}

	len = sizeof(*bos);
	buf = (unsigned char *)bos;
	buf += len;
	while(len < bos->wTotalLength)
	{
		header = (struct usb_dev_cap_header *) buf;
		if(header->bDescriptorType != USB_DT_DEVICE_CAPABILITY
			|| header->bDevCapabilityType > 0x04)
		{
			write_log(ctler, "Fail: the BOS descriptor corrupted\n");
			ret = -EPIPE;
			goto out;
		}
		pr_info("header->bDevCapabilityType: %d\n", header->bDevCapabilityType);
		switch (header->bDevCapabilityType)
		{
		case USB_SS_CAP_TYPE:
			ss_desc = (struct usb_ss_cap_descriptor *) buf;
			pr_info("ss_desc->bmAttributes is %d\n", ss_desc->bmAttributes);
			if(ss_desc->bmAttributes & USB_LTM_SUPPORT)
				is_support_LTM = 1;
			len += header->bLength;
			buf += header->bLength;
			break;
		default:
			len += header->bLength;
			buf += header->bLength;
			break;
		}
	}

	if(is_support_LTM == 0)
	{
		write_log(ctler, "Fail: the target device doesn't support LTM\n");
		//ret = -EINVAL;
		//goto out;
	}

	/* set the current BELT based on the Spec 4.13.1 request */
	init_completion(&xhci->cmd_verify_complet);
	spin_lock_irqsave(&xhci->lock, flags);

	command = xhci_alloc_command(xhci, false, false, GFP_ATOMIC);
	if(!command)
		return -1;

	/*multiple is 32768, latency value is 30, about 1ms. refer to USB 3.0 Spec 8.5.6.2 */
	ret = queue_command(xhci, command, 0, 0, 0, (TRB_TYPE(TRB_SET_LT) | (0x81E << 16)), true);
		
	if(ret < 0)
	{
		write_log(ctler, "Fail: fail to issue the set latency tolerance value command\n");
		spin_unlock_irqrestore(&xhci->lock, flags);
		goto out;
	}
	xhci_ring_cmd_db(xhci);
	spin_unlock_irqrestore(&xhci->lock, flags);

	/* I think 10 seconds is enough to transfer left  */
	if(!wait_for_completion_timeout(&xhci->cmd_verify_complet, 10 * HZ))
	{
		write_log(ctler, "Fail: the set latency tolerance value command timeout(10s)\n");
		ret = -EPIPE;
		goto out;
	}
	else
	{
		pr_info("good, the set LV command executed successfully\n");
	}

	/* enable the LTM notification event */
	reg_value = xhci_readl(xhci, &xhci->op_regs->dev_notification);
	reg_value |= 0x04;
	xhci_writel(xhci, reg_value, &xhci->op_regs->dev_notification);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_SET_FEATURE, 0,
		USB_DEVICE_LTM_ENABLE, 0, NULL, 0, 0);
	if(ret != 0)
	{
		write_log(ctler, "Fail: fail to issue Enable LTM request\n");
		ret = -EPIPE;
		goto out;
	}

	msleep(10);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), USB_REQ_GET_STATUS,
		USB_DIR_IN | USB_RECIP_DEVICE, 0, 0, &dev_status, 2, 0);
	if(ret < 0 || ret != 2)
	{
		write_log(ctler, "Fail: fail to get status of target device\n");
		ret = -EPIPE;
		goto out;
	}

	if((dev_status & (1 << USB_DEV_STAT_LTM_ENABLED)) == 0)
	{
		write_log(ctler,
			"Fail: the target device's status shows that LTM is not enabled yet\n");
		ret = -EPIPE;
		goto out;
	}

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	if(bos != NULL)
		kfree(bos);
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string:power_on
 * %d:0x%x:%d
 *
 */
int verify_ppc(struct xhci_ctler *ctler)
{
	struct usb_device *udev;
	unsigned int cmd_code, route_string, poweron;
	unsigned int port_index;

	int ret = 0;
	int i = 0;

	struct xhci_hcd *xhci = ctler->xhci;
	u32 value;
	u32 __iomem *addr;

	unsigned int current_state;

	struct usb_hub *hub;
	u32 port_status_change;
	u32 port_status, port_change;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x:%d", &cmd_code, &route_string, &poweron);
	if(check_num_parameter(ctler, ret, 3))
		goto out;
	if(check_cmd_code(ctler, cmd_code, VERIFY_PPC))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;
	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	pr_info("--------------------Verify PPC----------------------------\n");
	pr_info("the root_port it attached on: %s\n", udev->product);
	pr_info("the 2.0  ctler_>hcd->self.root_hub is 0x%p\n", ctler->hcd->self.root_hub);
	pr_info("the 3.0  xhci->shared_hcd->self.root_hub is 0x%p\n", xhci->shared_hcd->self.root_hub);	
	pr_info("the udev->parent is 0x%p\n", udev->parent);
	//if (udev->parent == ctler->hcd->self.root_hub) 
	if(udev->parent == ctler->hcd->self.root_hub || udev->parent == xhci->shared_hcd->self.root_hub)
	{
		value = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
		pr_info("the hcc_params is %x\n", value);
		if(!(HCC_PPC(value)))
		{
			write_log(ctler,
				"Fail: this controller doesn't"
				" support port power control for root hub\n");
			ret = 0;
			goto out;
		}
		port_index = route_string & 0xFF;

		/*if(port_index != udev->portnum)
		{
		    pr_info("the port address doesn't match,port_index=%d,udev->portnum=%d\n",port_index,udev->portnum);
		    write_log(ctler, "the port address doesn't match\n");
		    ret = -EPIPE;
		    goto out;
		}*/
		port_index--;

		//pr_info("add is %x",addr);
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * port_index;
		//return 0;

		value = xhci_readl(xhci, addr);

		pr_info("the original port status/control register is 0x%x\n", value);

		current_state = (value & PORT_POWER) >> 9;
		pr_info("before power on\n");
		pr_info("power on: %d\n", poweron);
		if(current_state == poweron)
		{
			write_log(ctler, "Success: the device is still in the target power state\n");
			ret = 0;
			goto out;
		}
		pr_info("after power on\n");
		//return 0;
		if(poweron)
		{
			value = xhci_readl(xhci, addr);
			value = xhci_port_state_to_neutral(value);
			value |= PORT_POWER;
			pr_info("write 0x%x to the status/control register to power on the port\n",
				value);
			xhci_writel(xhci, value, addr);
			msleep(2);
			value = xhci_readl(xhci, addr);
			current_state = (value & PORT_POWER) >> 9;
			while(!current_state && i < 10)
			{
				msleep(10);
				value = xhci_readl(xhci, addr);
				current_state = (value & PORT_POWER) >> 9;
				i++;
			}
			pr_info("the current port status/control register is 0x%x\n", value);
			if(i == 10)
			{
				write_log(ctler,
					"Fail: the target port can't be power on state after 12ms\n");
				ret = -EPIPE;
				goto out;
			}

			/* after power on the device, this device will be re-emulate by usbcore
			 * we need wait a moment and check whether the device is emulated
			 * sucessfully. we also need reset the device if there is no re-emulation
			 */
			msleep(15);
			udev = NULL;
			udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
			if(udev == NULL)
			{
				write_log(ctler,
					"Fail: the target USB device doesn't emulated after power on\n");
				ret = -EINVAL;
				goto out;
			}

			ret = 0;
			write_log(ctler, "Success\n");
			goto out;
		}
		else
		{
			value = xhci_readl(xhci, addr);
			value = xhci_port_state_to_neutral(value);
			value &= (~PORT_POWER);
			pr_info("write 0x%x to the status/control register to power off the port\n",
				value);
			xhci_writel(xhci, value, addr);
			msleep(2);
			value = xhci_readl(xhci, addr);
			current_state = (value & PORT_POWER) >> 9;
			while(current_state && i < 10)
			{
				msleep(10);
				value = xhci_readl(xhci, addr);
				current_state = (value & PORT_POWER) >> 9;
				i++;
			}
			pr_info("the current port status/control register is 0x%x\n", value);
			if(i == 10)
			{
				write_log(ctler,
					"Fail: the target port can't be power off state after 12ms\n");
				ret = -EPIPE;
				goto out;
			}

			ret = 0;
			write_log(ctler, "Success\n");
			goto out;
		}
	}

	hub = hdev_to_struct_hub(udev->parent);
	if(hub == NULL)
	{
		write_log(ctler, "Fail: the target device's parent is not hub\n");
		ret = -EPIPE;
		goto out;
	}
	if((hub->descriptor->wHubCharacteristics & HUB_CHAR_LPSM) != 0x01)
	{
		write_log(ctler, "Success: the parent hub doesn't support port power switch\n");
		ret = 0;
		goto out;
	}

	ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
		USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, udev->portnum,
		&port_status_change, 4, 0);
	if(ret < 0 || ret != 4)
	{
		write_log(ctler, "Fail: can't get the port status and change information\n");
		ret = -EPIPE;
		goto out;
	}
	port_status = port_status_change & 0x0FFFF;
	port_change = ((port_status_change >> 16) & 0x0FFFF);

	if(udev->parent->speed == USB_SPEED_SUPER)
		current_state = (port_status & (1 << 9)) >> 9;
	else
		current_state = (port_status & USB_PORT_STAT_POWER) >> 8;

	if(current_state == poweron)
	{
		write_log(ctler, "Success: the device is still in the target power state\n");
		ret = 0;
		goto out;
	}

	if(poweron)
	{
		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev, 0), USB_REQ_SET_FEATURE,
			USB_RT_PORT, USB_PORT_FEAT_POWER, udev->portnum, NULL, 0, 0);
		if(ret != 0)
		{
			write_log(ctler, "Fail: fail to issue the port power on request\n");
			ret = -EPIPE;
			goto out;
		}
		msleep(12);
		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, udev->portnum,
			&port_status_change, 4, 0);
		if(ret < 0 || ret != 4)
		{
			write_log(ctler, "Fail: can't get the port status and change information\n");
			ret = -EPIPE;
			goto out;
		}
		port_status = port_status_change & 0x0FFFF;
		port_change = ((port_status_change >> 16) & 0x0FFFF);

		if(udev->parent->speed == USB_SPEED_SUPER)
			current_state = (port_status & (1 << 9)) >> 9;
		else
			current_state = (port_status & USB_PORT_STAT_POWER) >> 8;
		if(current_state != poweron)
		{
			write_log(ctler,
				"Fail: the target port can't be power on state after 12ms\n");
			ret = -EPIPE;
			goto out;
		}

		/* after power on the device, this device will be re-emulate by usbcore
		 * we need wait a moment and check whether the device is emulated
		 * sucessfully. we also need reset the device if there is no re-emulation
		 */
		msleep(15);
		udev = NULL;
		udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
		if(udev == NULL)
		{
			write_log(ctler,
				"Fail: the target USB device doesn't emulated after power on\n");
			ret = -EINVAL;
			goto out;
		}

		ret = 0;
		write_log(ctler, "Success\n");
		goto out;
	}
	else
	{
		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev, 0),
			USB_REQ_CLEAR_FEATURE, USB_RT_PORT, USB_PORT_FEAT_POWER, udev->portnum, NULL,
			0, 0);
		if(ret != 0)
		{
			write_log(ctler, "Fail: fail to issue the port power off request\n");
			ret = -EPIPE;
			goto out;
		}
		msleep(12);
		ret = usb_control_msg(udev->parent, usb_sndctrlpipe(udev->parent, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, udev->portnum,
			&port_status_change, 4, 0);
		if(ret < 0 || ret != 4)
		{
			write_log(ctler, "Fail: can't get the port status and change information\n");
			ret = -EPIPE;
			goto out;
		}
		port_status = port_status_change & 0x0FFFF;
		port_change = ((port_status_change >> 16) & 0x0FFFF);

		if(udev->parent->speed == USB_SPEED_SUPER)
			current_state = (port_status & (1 << 9)) >> 9;
		else
			current_state = (port_status & USB_PORT_STAT_POWER) >> 8;
		if(current_state != poweron)
		{
			write_log(ctler,
				"Fail: the target port can't be power off state after 12ms\n");
			ret = -EPIPE;
			goto out;
		}

		ret = 0;
		write_log(ctler, "Success\n");
		goto out;
	}

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code
 * %d
 *
 */
 void protrol_parameter_analysis(struct xhci_ctler *ctler)
 {
		u32 value,value1;
		u32 __iomem *addr;
		u32 offset;
		u32 capability_id;
		int i = 0;
		
	  struct xhci_hcd *xhci = ctler->xhci;
	  value = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
	  offset = HCC_EXT_CAPS(value);
	  
	  addr = &xhci->cap_regs->hc_capbase + offset;
	  value = xhci_readl(xhci, addr);
	  capability_id = XHCI_EXT_CAPS_ID(value);
	  offset = XHCI_EXT_CAPS_NEXT(value);
	 while(offset != 0)
	 {
		 addr += offset;
		 value = xhci_readl(xhci, addr);
		 capability_id = XHCI_EXT_CAPS_ID(value);
		 offset = XHCI_EXT_CAPS_NEXT(value);
		 if(capability_id == XHCI_EXT_CAPS_PROTOCOL){
		    pr_info("Protocol Capability Offset 00h is %x\n", value);
			pr_info("Protocol Capability Offset 04h is %x\n", xhci_readl(xhci, addr + 1));
			pr_info("Protocol Capability Offset 08h is %x\n", xhci_readl(xhci, addr + 2));
			pr_info("Protocol Capability Offset 0ch is %x\n", xhci_readl(xhci, addr + 3));
			value1=((xhci_readl(xhci, addr + 3))>>28)&0x0f;
			pr_info("support kinds of the value1=%d\n",value1);
			for(i=0;i<value1;i++)
			{
			  pr_info("Protocol speed id is %x\n", xhci_readl(xhci, addr + 3+i));  	
			}
		 }
		 if(capability_id == XHCI_EXT_CAPS_VIRT)
		   pr_info("xhci support i/o virtualization!\n");  	
	  }
  }
int verify_handoff(struct xhci_ctler *ctler)
{

	int i = 0;
	int ret = 0;

	u32 value;
	u32 __iomem *addr;
	u32 offset;
	u32 capability_id;
	u32 legacy_cap;
	u32 legacy_ctl;
	u32 bios_owned;
	u32 os_owned;

	unsigned cmd_code, target_os_owned;
	struct xhci_hcd *xhci = ctler->xhci;

	ret = sscanf(ctler->test.cmd_str, "%d", &cmd_code);
	if(check_num_parameter(ctler, ret, 1))
		goto out;
	if(check_cmd_code(ctler, cmd_code, VERIFY_HANDOFF))
		goto out;
	//weitao,for extend capability test.
	protrol_parameter_analysis(ctler);
	value = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
	pr_info("hcc_params is %x\n", value);
	offset = HCC_EXT_CAPS(value);
	if(offset == 0)
	{
		write_log(ctler,
			"Success: this xHCI controller doesn't supoort BIOS/OS handoff\n");
		ret = 0;
		goto out;
	}

	addr = &xhci->cap_regs->hc_capbase + offset;
	value = xhci_readl(xhci, addr);
	capability_id = XHCI_EXT_CAPS_ID(value);
	offset = XHCI_EXT_CAPS_NEXT(value);
	while(offset != 0)
	{
		if(capability_id == XHCI_EXT_CAPS_LEGACY)
			break;
		addr += offset;
		value = xhci_readl(xhci, addr);
		capability_id = XHCI_EXT_CAPS_ID(value);
		offset = XHCI_EXT_CAPS_NEXT(value);
	}

	value = xhci_readl(xhci, addr);
	capability_id = XHCI_EXT_CAPS_ID(value);
	if(capability_id != XHCI_EXT_CAPS_LEGACY)
	{
		write_log(ctler,
			"Success1: this xHCI controller doesn't support BIOS/OS handoff\n");
		ret = 0;
		goto out;
	}

	legacy_cap = xhci_readl(xhci, addr);
	legacy_ctl = xhci_readl(xhci, addr + 1);
	pr_info("the usb legacy capability support control/status register value is 0x%x\n",
		legacy_ctl);
	//legacy_ctl |= ((1 << 19) | (1 << 13));
	legacy_ctl |= ((1 << 29) | (1 << 13));//weitao modify
	xhci_writel(xhci, legacy_ctl, addr + 1);

	bios_owned = ((legacy_cap & (1 << 16)) >> 16);
	os_owned = ((legacy_cap & (1 << 24)) >> 24);
	target_os_owned = 1;
	if(bios_owned == 0 && os_owned == 1 && target_os_owned == 1)
	{
		write_log(ctler, "Success: the OS have owned this controller yet\n");
		ret = 0;
		goto out;
	}

	legacy_cap = xhci_readl(xhci, addr);
	legacy_cap |= (1 << 24);
	xhci_writel(xhci, legacy_cap, addr);

	msleep(2);

	legacy_cap = xhci_readl(xhci, addr);
	bios_owned = ((legacy_cap & (1 << 16)) >> 16);
	os_owned = ((legacy_cap & (1 << 24)) >> 24);
	while((!(os_owned == 1 && bios_owned == 0)) && (i < 10))
	{
		legacy_cap = xhci_readl(xhci, addr);
		bios_owned = ((legacy_cap & (1 << 16)) >> 16);
		os_owned = ((legacy_cap & (1 << 24)) >> 24);
		i++;
		msleep(1);
	}

	if(i == 10)
	{
		write_log(ctler, "Fail: OS can't owned this controller after 12ms\n");
		ret = -EPIPE;
		goto out;
	}

	/* I think we can't clear the ownership change bit in control register,
	 * maybe BIOS will use this information and clear it.
	 * OS triggers a SMI interrupt and BIOS will process this interrupt.
	 * who process the interrupt, who should ack the interrupt by clear it
	 */

	legacy_ctl = xhci_readl(xhci, addr + 1);
	if(legacy_ctl & (1 << 29))
	{
		pr_info("the ownership change status bit has been set\n");
	}

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string
 * %d:0x%x
 *
 */
int warm_reset(struct xhci_ctler *ctler)
{
	unsigned cmd_code;
	unsigned route_string;
	unsigned port_index;
	int ret = 0;
	u32 value = 0;
	u32 __iomem *addr;
	struct usb_device *udev;
	struct xhci_hcd *xhci = ctler->xhci;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x", &cmd_code, &route_string);
	if(check_num_parameter(ctler, ret, 2))
		goto out;
	if(check_cmd_code(ctler, cmd_code, WARM_RESET))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;
	pr_info("the root_port it attached on: %s\n", udev->product);
	
	if(udev->speed != USB_SPEED_SUPER)
	{
		write_log(ctler,
			"Fail: the target device is not SS device, "
			"it doesn't support the warm reset\n");
		ret = -EINVAL;
		goto out;
	}

	pr_info("the 3.0 root hub: xhci->shared_hcd->self.root_hub is %p\n", xhci->shared_hcd->self.root_hub);
	pr_info("the udev->parent is              %p\n", udev->parent);
	pr_info("the udev is                      %p\n", udev);
	if (udev->parent == xhci->shared_hcd->self.root_hub)
	//if(udev->parent != NULL)
	{
		port_index = route_string & 0xFF;
		pr_info("port index is 0x%x\n", port_index);
		addr  = &xhci->op_regs->port_status_base + NUM_PORT_REGS * (port_index-1);
		pr_info("The address of port 0x%x is %p\n", port_index, addr);
		
		value = xhci_readl(xhci, addr);
		pr_info("the original value              is 0x%x\n", value);
		value = xhci_port_state_to_neutral(value);
		pr_info("the original value with neutral is 0x%x\n", value);

		value |= PORT_WR;
		pr_info("write value 0x%x to the 0x%x port register\n", value, port_index);
		xhci_writel(xhci, value, addr);
		msleep(2);
		value = xhci_readl(xhci, addr);
		pr_info("after write, the register value is 0x%x\n", value);
	}
	else
	{
		write_log(ctler, "Fail: the target device must attach to root port\n");
		ret = -EPIPE;
		goto out;
	}

	ret = 0;
	write_log(ctler, "Success\n");

  out:
	return ret;
}

/*
 * Synopsis:
 * cmd_code:route_string
 * %d      :0x%x
 * the target device must be a mass storage device,
 * and we will write data to the storage device,
 * then read data from the same address.
 * when reading and writing,
 * we will queue a zero-length TRB in the data stage.
 *
 */
int zero_length_trb(struct xhci_ctler *ctler)
{
	int i, ret;
	unsigned int cmd_code, route_string;
	struct usb_device *udev;
	struct hc_driver *xhci_drv = ctler->xhci_drv;

	unsigned int transfer_length;
	unsigned int special_len, normal_len;
	unsigned int start_sector, sector_cnt;
	struct scatterlist *sg;
	struct mass_storage_dev_info mass_stor_info;
	struct xhci_mem_table *normal_table, *special_table;

	void *urb_enqueue_routine_bakeup;

	ret = sscanf(ctler->test.cmd_str, "%d:0x%x", &cmd_code, &route_string);

	if(check_num_parameter(ctler, ret, 2))
		goto out;
	if(check_cmd_code(ctler, cmd_code, ZERO_LENGTH_TRB))
		goto out;
	if(check_route_string(ctler, route_string))
		goto out;

	udev = xhci_verify_get_dev_by_route_string(ctler, route_string);
	if(check_udev(ctler, udev))
		goto out;

	ret = xhci_verify_get_mass_storage_info(udev, &mass_stor_info);
	if(ret)
	{
		pr_info("the target device is not usb mass storage device\n");
		write_log(ctler, "Fail: the target device is not mass storage device\n");
		ret = -EINVAL;
		goto out;
	}

	transfer_length = 4 * PAGE_SIZE;
	//transfer_length = 0 ;

	normal_len = transfer_length;
	normal_table = xhci_verify_create_normal_mem_table(ctler, normal_len);
	if(normal_table == NULL)
	{
		write_log(ctler, "Fail: alloc normal mem_table failed\n");
		ret = -ENOMEM;
		goto out;
	}

	special_len = 0;
	special_table = xhci_verify_create_special_mem_table(special_len);
	if(special_table == NULL)
	{
		write_log(ctler, "Fail: alloc special mem_table failed\n");
		ret = -ENOMEM;
		goto free_normal_table;
	}

	/* reset all the buffer data value */
	xhci_verify_reset_mem_pool_data(ctler);

	ret = xhci_verify_init_normal_mem_table(normal_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	ret = xhci_verify_init_special_mem_table(special_table);
	if(ret < 0)
	{
		write_log(ctler, "Fail: init normal mem_table failed\n");
		ret = -ENOMEM;
		goto free_special_table;
	}

	pr_info("\nDump mem_table for transfer length: %x\n", transfer_length);
	xhci_verify_dump_normal_mem_table(normal_table);
	xhci_verify_dump_special_mem_table(special_table);

	if(unlikely(sg_alloc_table(ctler->sgtable,
				normal_table->nents + special_table->nents, GFP_KERNEL)))
	{
		ret = -ENOMEM;
		write_log(ctler, "Fail: alloc sg table failed\n");
		goto free_special_table;
	}

	for_each_sg(ctler->sgtable->sgl, sg, normal_table->nents, i) sg_set_page(sg,
		normal_table->entry[i].page, normal_table->entry[i].length,
		normal_table->entry[i].offset);

	start_sector = 60;
	sector_cnt = transfer_length / 512;

	urb_enqueue_routine_bakeup = xhci_drv->urb_enqueue;
	xhci_drv->urb_enqueue = xhci_urb_enqueue_zero_len_trb;
	/* write the buffer to USB mass storage device */
	ret = xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
		sector_cnt, normal_table, special_table, 1, 0, 1);
	//reset the urb_enqueue routine
	xhci_drv->urb_enqueue = urb_enqueue_routine_bakeup;

	//if(ret < 0)
		//goto free_sg_table;
	//xhci_verify_reset_mem_pool_data(ctler);
	//xhci_verify_reset_special_mem_table_data(special_table);

	/* read the data back with the same buffer, which has been reset. for read,
	 * we can't emulate the situation that device issue a zero-length data packet
	 */
	//ret = xhci_verify_stor_transport(ctler, udev, &mass_stor_info, start_sector,
		//sector_cnt, normal_table, special_table, 0, 0, 1);

	//if(ret < 0)
		//goto free_sg_table;

	/* verify the data now */
	/*if((xhci_verify_normal_mem_table(normal_table) != 0)
		|| (xhci_verify_special_mem_table(special_table) != 0))
	{
		ret = -ENOTSYNC;
		write_log(ctler, "Fail: data verify fail\n");
		pr_info("dump mem_table for transfer length: %x\n", transfer_length);
		xhci_verify_dump_normal_mem_table(normal_table);
		xhci_verify_dump_special_mem_table(special_table);
		goto free_sg_table;
	}*/

	//if(ret == 0)
		write_log(ctler, "Success\n");

 //free_sg_table:
	//sg_free_table(ctler->sgtable);

 free_special_table:
	if(special_table != NULL)
		xhci_verify_free_special_mem_table(special_table);

 free_normal_table:
	if(normal_table != NULL)
		xhci_verify_free_normal_mem_table(normal_table);

  out:
	return ret;
}

struct verify_func verify_func_array[] = 
{
	{HUB_REQUEST, hub_request},
	{GET_DEV_DESC, get_device_descriptor},
	{SET_TRANSFER_SEG_NUM, set_transfer_seg_num},
	{SG_TRANSFER, sg_transfer},
	{SG_VERIFY, sg_verify},
	{ISOCH_UNDERRN_OVERRUN, isoch_underrun_overrun},
	{EVENT_DATA_TRB, event_data_trb},
	{CMD_VERIFY, cmd_verify},
	{SET_EVENT_SEG_NUM, set_event_seg_num},
	{VERIFY_EVENT_FULL, verify_event_full},
	{SET_INTR_MODERATION, set_intr_moderation},
	{SET_U1U2_TIMEOUT, set_u1u2_timeout},
	{SET_LPM_STATE, set_lpm_state},
	{FUNC_SUSPEND, function_suspend},
	{ENABLE_LTM, enable_ltm},
	{VERIFY_PPC, verify_ppc},
	{VERIFY_HANDOFF, verify_handoff},
	{WARM_RESET, warm_reset},
	{ZERO_LENGTH_TRB, zero_length_trb},
	{TEST, test},
	{U3_ENTRY_CAPABILITY, u3_entry_enable},
};

#define VERIFY_FUNC_ARRAY_SIZE \
	((sizeof(verify_func_array))/(sizeof(verify_func_array[0])))

static ssize_t xhci_ctler_attr_hw_info_show(struct xhci_ctler *ctler, char *buf)
{
	size_t remain_buf_len;
	int max_slots;
	int slot_id;
	int i, j;
	int ports;
	u32 __iomem *addr;
	char *cur_buf_ptr;

	struct pci_dev *pci_dev = ctler->pcidev;
	struct xhci_hcd *xhci = ctler->xhci;
	struct usb_hcd *hcd_2=xhci->main_hcd;
	struct usb_hcd *hcd_3=xhci->shared_hcd;
	struct xhci_virt_device *virt_dev;
	struct xhci_slot_ctx *slot_ctx;
	struct usb_device *udev;
	struct usb_interface *intf;
	struct usb_host_endpoint *ep;
	unsigned char ep_type[4][20] = { "control", "isochronous", "bulk", "interrupt" };

	unsigned int hc_capbase;
	unsigned int hcs_params1;
	unsigned int hcs_params2;
	unsigned int hcs_params3;
	unsigned int hcc_params;
	unsigned int hcc_params2;
	unsigned int db_off;
	unsigned int run_regs_off;

	cur_buf_ptr = buf;
	remain_buf_len = MAX_ATTR_SHOW_DATA_LEN ;

	hc_capbase   = xhci_readl(xhci, &xhci->cap_regs->hc_capbase);
	hcs_params1  = xhci_readl(xhci, &xhci->cap_regs->hcs_params1);
	hcs_params2  = xhci_readl(xhci, &xhci->cap_regs->hcs_params2);
	hcs_params3  = xhci_readl(xhci, &xhci->cap_regs->hcs_params3);
	hcc_params   = xhci_readl(xhci, &xhci->cap_regs->hcc_params);

	#ifdef xhci11_verify
	hcc_params2  = xhci_readl(xhci, &xhci->cap_regs->hcc_params2);
	#endif

	db_off       = xhci_readl(xhci, &xhci->cap_regs->db_off);
	run_regs_off = xhci_readl(xhci, &xhci->cap_regs->run_regs_off);

	/* Look dmesg for this */
	xhci_print_ports(xhci);
	xhci_print_command_reg(xhci);
	xhci_dbg_regs(xhci);
	xhci_print_run_regs(xhci);

	//xhci pci information
	pr_info("\n\n########Controller infomation########\n\n");
	pr_info("Vendor Id   : 0x%4x\n", pci_dev->vendor);
	pr_info("Device ID   : 0x%4x\n", pci_dev->device);
	pr_info("IRQ Number  : %d\n", pci_dev->irq);
	pr_info("Driver      : %s\n", pci_dev->driver->name);

	//xhci capability registers
	pr_info("\n***xhci capability registers***\n");
	pr_info("Cap Register length\t: 0x%x\n", HC_LENGTH(hc_capbase));
	pr_info("xHCI Version\t\t: 0x%x\n", HC_VERSION(hc_capbase));
	pr_info("Max Device Slots\t: 0x%x\n", HCS_MAX_SLOTS(hcs_params1));
	pr_info("Max Interrupters\t: 0x%x\n", HCS_MAX_INTRS(hcs_params1));
	pr_info("Max Ports\t\t: 0x%x\n", HCS_MAX_PORTS(hcs_params1));
	pr_info("Iso Scheduling Threshold: 0x%x\n", HCS_IST(hcs_params2));
	pr_info("Max Event Ring Segment\t: 0x%x\n", HCS_ERST_MAX(hcs_params2));
	pr_info("Scratchpad buffers\t: 0x%x\n", HCS_MAX_SCRATCHPAD(hcs_params2));
	pr_info("U1 Device Exit Latency\t: less than %d us\n", HCS_U1_LATENCY(hcs_params3));
	pr_info("U2 Device Exit Latency\t: less than %d us\n", HCS_U2_LATENCY(hcs_params3));
	pr_info("Support 64 bit address\t: 0x%x\n", HCC_64BIT_ADDR(hcc_params));
	pr_info("BW Negotiation Capability\t: 0x%x\n", HCC_BANDWIDTH_NEG(hcc_params));
	pr_info("Context size is 64 bytes: 0x%x\n", HCC_64BYTE_CONTEXT(hcc_params));
	pr_info("Port Power Control\t: 0x%x\n", HCC_PPC(hcc_params));
	pr_info("Port indicators\t\t: 0x%x\n", HCS_INDICATOR(hcc_params));
	pr_info("Light HC Reset Capbi\t: 0x%x\n", HCC_LIGHT_RESET(hcc_params));
	//LTM means Latency Tolerance Messaging 
	pr_info("LTM Capability \t\t: 0x%x\n", HCC_LTC(hcc_params));
	pr_info("No Secondary SID Support: 0x%x\n", HCC_NSS(hcc_params));
	
	pr_info("Max Pri Strream Array\t: 0x%x\n", HCC_MAX_PSA(hcc_params));
        pr_info("xHCI ext cap pointer\t: 0x%x\n", HCC_EXT_CAPS(hcc_params));
	
	#ifdef xhci11_verify
	//xhci1.1 add some new feature..
        pr_info("HC Stopped - Short Packet \t: 0x%x\n",HCC_SPC(hcc_params));
        pr_info("HC Parse all event data capbi \t: 0x%x\n",hcc_params &(1<<8));
        pr_info("HC stoppend EDTLA capbi \t: 0x%x\n",hcc_params &(1<<10));
        pr_info("HC Contiguous Frame ID Capability: 0x%x\n", HCC_CFC(hcc_params));
	
	 /* HCCPARAMS2 - hcc_params2*/
        pr_info("HC U3 entry Capability supports\t: 0x%x\n", HCC2_U3C(hcc_params2));
        pr_info("HC Configure endpoint command Max exit latency too large Capability\t: 0x%x\n", HCC2_CMC(hcc_params2));
        pr_info("HC Force Save context Capability supports\t: 0x%x\n", HCC2_FSC(hcc_params2));
        pr_info("HC Compliance Transition Capability supports\t: 0x%x\n", HCC2_CTC(hcc_params2));
        pr_info("HC Large ESIT payload Capability support\t: 0x%x\n", HCC2_LEC(hcc_params2));
        pr_info("HC Configuration Information Capability support\t: 0x%x\n", HCC2_CIC(hcc_params2));
        //xhci1.1 add some new feature.. end
	#endif	

	//xhci operational registers
	pr_info("\nxhci operational registers\n");
	pr_info("command			: 0x%x\n", xhci_readl(xhci, &xhci->op_regs->command));
	pr_info("status			: 0x%x\n", xhci_readl(xhci, &xhci->op_regs->status));
	pr_info("page size		: 0x%x\n", xhci_readl(xhci, &xhci->op_regs->page_size));
	pr_info("notification		: 0x%x\n", xhci_readl(xhci,
			&xhci->op_regs->dev_notification));
	pr_info("dev slot enabled		: 0x%x\n", HCS_MAX_SLOTS(xhci_readl(xhci,
				&xhci->op_regs->config_reg)));
	//read root information from hcs_params1
	pr_info("\n");
	pr_info("root port information:\n");
	ports = HCS_MAX_PORTS(xhci_readl(xhci, &xhci->cap_regs->hcs_params1));
	
	#ifdef xhci11_verify
	for(i = 0; i < ports; i++)
	{
		//number of registers per ports
		addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * i;
		pr_info("root port %d:  status control :0x%x, PM :0x%x, Link Info :0x%x, HLPMControl :0x%x\n",
			i + 1, xhci_readl(xhci, addr), 
			xhci_readl(xhci, addr + 1), xhci_readl(xhci,addr + 2),xhci_readl(xhci,addr + 3));
	}
	#else
	for(i = 0; i < ports; i++)
        {
                //number of registers per ports
                addr = &xhci->op_regs->port_status_base + NUM_PORT_REGS * i;
                pr_info("root port %d:  status control :0x%x, PM :0x%x, Link Info :0x%x\n",
                        i + 1, xhci_readl(xhci, addr),
                        xhci_readl(xhci, addr + 1), xhci_readl(xhci,addr + 2));
        }
	#endif
	
	pr_info("Running time register-the event ring size is :0x%x\n",xhci_readl(xhci,
		&xhci->ir_set->erst_size));
	pr_info("the xhci control wakeup ability for usb2=%d\n",device_can_wakeup(hcd_2->self.controller));
	pr_info("the xhci control wakeup ability for usb3=%d\n",device_can_wakeup(hcd_3->self.controller));

	//polling all device
	pr_info("\n########Device infomation########\n");
	pr_info("--------------------------------------------------------------------------\n");
	max_slots = HCS_MAX_SLOTS(xhci_readl(xhci, &xhci->op_regs->config_reg));
	// slot polling
	for(slot_id = 0; slot_id < max_slots; slot_id++)
	{
		virt_dev = xhci->devs[slot_id];
		if(virt_dev == NULL)
			continue;
		udev = virt_dev->udev;
		slot_ctx = xhci_get_slot_ctx(xhci, virt_dev->out_ctx);
		pr_info("USB device %d: VendorID 0x%x, DeviceID 0x%x, "
			"USB Version 0x%x, Manufacturer: %s, Product: %s, "
			"EP0 Max Packet Size 0x%x, bDeviceClass 0x%x, "
			"bDeviceSubClass 0x%x, bDeviceProtocol 0x%x, config num 0x%x\n",
			slot_id,
			udev->descriptor.idVendor,
			udev->descriptor.idProduct,
			udev->descriptor.bcdUSB,
			udev->manufacturer,
			udev->product, udev->descriptor.bMaxPacketSize0,
			udev->descriptor.bDeviceClass, udev->descriptor.bDeviceSubClass,
			udev->descriptor.bDeviceProtocol, udev->descriptor.bNumConfigurations);
		pr_info("address 0x%x, root port 0x%x, "
			"route string 0x%x, active config index 0x%x, interface num 0x%x\n",
			(slot_ctx->dev_state & 0xFF), ((slot_ctx->dev_info2 >> 16) & 0xFF),
			(slot_ctx->dev_info & 0xFFFFF), udev->actconfig->desc.bConfigurationValue,
			udev->actconfig->desc.bNumInterfaces);
		//interface polling
		for(i = 0; i < udev->actconfig->desc.bNumInterfaces; i++)
		{
			intf = udev->actconfig->interface[i];
			if(intf == NULL)
				continue;
			pr_info("    interface %d: interface index 0x%x, intf alter setting num 0x%x, "
				"active intf setting index 0x%x, endpoint num 0x%x, intf class 0x%x, "
				"intf subclass 0x%x, intf protocol 0x%x, string descp: %s\n", i,
				intf->cur_altsetting->desc.bInterfaceNumber, intf->num_altsetting,
				intf->cur_altsetting->desc.bAlternateSetting,
				intf->cur_altsetting->desc.bNumEndpoints,
				intf->cur_altsetting->desc.bInterfaceClass,
				intf->cur_altsetting->desc.bInterfaceSubClass,
				intf->cur_altsetting->desc.bInterfaceProtocol,
				intf->cur_altsetting->string ? intf->cur_altsetting->string : "NULL");
			//endpoint polling
			for(j = 0; j < intf->cur_altsetting->desc.bNumEndpoints; j++)
			{
				ep = &(intf->cur_altsetting->endpoint[j]);
				if(ep == NULL)
					continue;
				pr_info("        EP %d: ep address 0x%x, ep direction %s, "
					"ep type %s, max packet size 0x%x, interval 0x%x, max burst 0x%x, "
					"companion attributes 0x%x, BytesPerInterval 0x%x\n",
					j,
					usb_endpoint_num(&ep->desc),
					usb_endpoint_dir_in(&ep->desc) ? "IN" : "OUT",
					ep_type[usb_endpoint_type(&ep->desc)], ep->desc.wMaxPacketSize,
					ep->desc.bInterval, ep->ss_ep_comp.bMaxBurst,
					ep->ss_ep_comp.bmAttributes, ep->ss_ep_comp.wBytesPerInterval);
			}
		}
		pr_info("--------------------------------------------------------------------------\n");
	}
	pr_info("\n\n########## End ###########\n");
	return 0;

  //out:
	//return (ssize_t)(MAX_ATTR_SHOW_DATA_LEN - remain_buf_len);
}

static ssize_t xhci_ctler_attr_verify_cmd_store(struct xhci_ctler *ctler,
	const char *buf, size_t count)
{
	char *cmd;
	unsigned int cmd_code;
	size_t len;
	unsigned long flags;
	int i;
	int ret = -EINVAL;

	spin_lock_irqsave(&ctler->lock, flags);
	if(ctler->verify_status == TEST_RUNNING)
	{
		spin_unlock_irqrestore(&ctler->lock, flags);
		return -EBUSY;
	}
	ctler->verify_status = TEST_RUNNING;	
	spin_unlock_irqrestore(&ctler->lock, flags);

	memset(ctler->test.result, 0, MAX_TEST_RESULT_LEN);
	ctler->test.rb_cur_ptr = ctler->test.result;
	ctler->test.rb_remain_len = MAX_TEST_RESULT_LEN;

	len = strlen(buf);

	if((!len) || (len >= MAX_TEST_CMD_LEN))
	{
		write_log(ctler, "Fail: Command string invalid\n");
		goto out;
	}

	strcpy(ctler->test.cmd_str, buf);

	cmd = strim(ctler->test.cmd_str);

	len = strlen(cmd);

	if(!len)
	{
		write_log(ctler, "Fail: Command string invalid\n");
		goto out;
	}

	//string to unsigned long, transfer string to integer
	cmd_code = (unsigned int)simple_strtoul(cmd, NULL, 10);

	//pr_info("cmd_code is %d\n", cmd_code);

	if(cmd_code >= MAX_CMD_CODE)
	{
		write_log(ctler, "Fail: Command code invalid\n");
		goto out;
	}

	//find verify function pointer
	for(i = 0; i < VERIFY_FUNC_ARRAY_SIZE; i++)
		if(verify_func_array[i].cmd_code == cmd_code)
			break;

	//pr_info("i = %d\n", i);
	//pr_info("VERIFY_FUNC_ARRAY_SIZE is %ld\n", VERIFY_FUNC_ARRAY_SIZE);
	if(i >= VERIFY_FUNC_ARRAY_SIZE)
	{
		pr_info("here");
		write_log(ctler, "Fail: No matched routine for cmd[%d]\n", cmd_code);
		goto out;
	}

	//check function pointer is valid
	if(!verify_func_array[i].func)
	{
		pr_info("verify_func_array[i].func == NULL\n");	
		write_log(ctler, "Fail: No matched routine for cmd[%d]\n", cmd_code);
		goto out;
	}

	strcpy(ctler->test.cmd_str, cmd);
	
	//excute verify function pointer, such as cmd_verify
	//call verify function
	ret = verify_func_array[i].func(ctler);
	//pr_info("ret = %d\n" , ret);
	//ret = 0;
	if(ret)
	{
		if(ctler->test.result[0] == 0)
			write_log(ctler, "Fail\n");
	}
	else
	{
		if(ctler->test.result[0] == 0)
			write_log(ctler, "Success\n");
		ret = (int) count;
	}

out:
	//set verify_status, when you cat verify_result, it will be complete.
	ctler->verify_status = TEST_COMPLETE;
	return (ssize_t) ret;
}

static ssize_t xhci_ctler_attr_verify_status_show(struct xhci_ctler *ctler, char *buf)
{
	char *cur_buf_ptr;
	size_t remain_buf_len;

	cur_buf_ptr = buf;
	remain_buf_len = MAX_ATTR_SHOW_DATA_LEN;

	switch (ctler->verify_status)
	{
	case TEST_IDLE:
		SYSFS_PRINT("idle\n");
		break;
	case TEST_RUNNING:
		SYSFS_PRINT("running\n");
		break;
	case TEST_COMPLETE:
		SYSFS_PRINT("complete\n");
		break;
	default:
		SYSFS_PRINT("invalid\n");
		break;
	}

  out:
	return (ssize_t) (MAX_ATTR_SHOW_DATA_LEN - remain_buf_len);
}

static ssize_t xhci_ctler_attr_verify_result_show(struct xhci_ctler *ctler, char *buf)
{
	char *cur_buf_ptr;
	size_t remain_buf_len;

	cur_buf_ptr = buf;
	remain_buf_len = MAX_ATTR_SHOW_DATA_LEN;

	switch (ctler->verify_status)
	{
	case TEST_IDLE:
		SYSFS_PRINT("No buffered test result\n");
		break;
	case TEST_RUNNING:
		SYSFS_PRINT("Test is still running\n");
		break;
	case TEST_COMPLETE:
		SYSFS_PRINT("%s", ctler->test.result);
		ctler->verify_status = TEST_IDLE;
		break;
	default:
		SYSFS_PRINT("invalid\n");
		break;
	}

  out:
	return (ssize_t) (MAX_ATTR_SHOW_DATA_LEN - remain_buf_len);
}

/*
 * The buf length is 1 page.
 */
static ssize_t xhci_ctler_attr_show(struct kobject *kobj, struct attribute *attr,
	char *buf)
{
	struct xhci_ctler_sysfs_attr *xhci_ctler_sysfs_attr;
	struct xhci_ctler *xhci_ctler;

	xhci_ctler_sysfs_attr = container_of(attr, struct xhci_ctler_sysfs_attr, attr);
	if(!xhci_ctler_sysfs_attr->show)
		return -EIO;

	xhci_ctler = container_of(kobj, struct xhci_ctler, kobj);
	if(&xhci_ctler->kobj != kobj)
		return -EINVAL;

	return xhci_ctler_sysfs_attr->show(xhci_ctler, buf);
}

static ssize_t xhci_ctler_attr_store(struct kobject *kobj, struct attribute *attr,
	const char *buf, size_t count)
{
	struct xhci_ctler_sysfs_attr *xhci_ctler_sysfs_attr;
	struct xhci_ctler *xhci_ctler;

	xhci_ctler_sysfs_attr = container_of(attr, struct xhci_ctler_sysfs_attr, attr);
	if(!xhci_ctler_sysfs_attr->store)
		return -EIO;

	xhci_ctler = container_of(kobj, struct xhci_ctler, kobj);
	if(&xhci_ctler->kobj != kobj)
		return -EINVAL;

	return xhci_ctler_sysfs_attr->store(xhci_ctler, buf, count);
}

static XHCI_CTLER_ATTR_RO(hw_info);
static XHCI_CTLER_ATTR_RO(verify_status);
static XHCI_CTLER_ATTR_RO(verify_result);
static XHCI_CTLER_ATTR_WO(verify_cmd);

static struct kobject null_kobj;
static struct kobj_type null_ktype = 
{
	.sysfs_ops = NULL,
	.default_attrs = NULL,
};

static struct attribute *xhci_ctler_attrs[] = 
{
	&xhci_ctler_attr_hw_info.attr,
	&xhci_ctler_attr_verify_status.attr,
	&xhci_ctler_attr_verify_cmd.attr,
	&xhci_ctler_attr_verify_result.attr,
	NULL,
};

const static struct sysfs_ops xhci_ctler_sysfs_ops = 
{
	.show = xhci_ctler_attr_show,
	.store = xhci_ctler_attr_store,
};

static struct kobj_type xhci_ctler_ktype = 
{
	.sysfs_ops     = &xhci_ctler_sysfs_ops,
	.default_attrs = xhci_ctler_attrs,
};

int remove_xhci_ctler(struct usb_hcd *host)
{
	struct xhci_ctler *cur = NULL;
	struct xhci_ctler *next;
	unsigned long flags;

	if(!host)
		return -EINVAL;

	spin_lock_irqsave(&xhci_ctler_list_lock, flags);
	list_for_each_entry_safe(cur, next, &xhci_ctler_list, list)
	{
		if(cur->hcd == host)
		{
			list_del(&cur->list);
			break;
		}
	}
	spin_unlock_irqrestore(&xhci_ctler_list_lock, flags);

	if(!cur)
	{
		return -ENOENT;
	}
	else if(cur->hcd == host)
	{
		if(cur->kobj_valid)
		{
			sysfs_remove_link(&cur->kobj, "device");
			kobject_put(&cur->kobj);
		}
		xhci_verify_free_mem_pool(cur);
		kfree(cur->sgtable);
		free_page((unsigned long) cur);
		return 0;
	}
	else
	{
		return -ENOENT;
	}
}

EXPORT_SYMBOL(remove_xhci_ctler);

int add_xhci_ctler(struct usb_hcd *host)
{
	int ret = 0;
	unsigned long flags;
	struct xhci_ctler *xhci_ctler;
	pr_info("weitao:goto function add_xhci_ctler!!\n");
	if(!host)
	{
		ret = -EINVAL;
		goto out;
	}
	/* alloc strcut xhci_ctler */
	xhci_ctler = (struct xhci_ctler *)get_zeroed_page(GFP_KERNEL);
	if(!xhci_ctler)
	{
		ret = -ENOMEM;
		goto out;
	}
	/* init mem pool */
	ret = xhci_verify_init_mem_pool(xhci_ctler);
	if(ret)
	{
		ret = -ENOMEM;
		goto free_page;
	}
	/* init sgtable */
	xhci_ctler->sgtable = 
		(struct sg_table *)kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if(xhci_ctler->sgtable == NULL)
	{
		ret = -ENOMEM;
		goto free_mem_pool;
	}

	spin_lock_irqsave(&xhci_ctler_list_lock, flags);
	xhci_ctler->index = xhci_ctler_index++;
	list_add(&xhci_ctler->list, &xhci_ctler_list);
	spin_unlock_irqrestore(&xhci_ctler_list_lock, flags);

	ret = kobject_init_and_add(&xhci_ctler->kobj, 
		&xhci_ctler_ktype, &null_kobj, "ctler%d", xhci_ctler->index);
	if(ret)
		goto remove_from_list;

	/* Fetch xhci private data */
	sysfs_create_link(&xhci_ctler->kobj, &host->self.controller->kobj, "device");
	xhci_ctler->kobj_valid = 1;
	xhci_ctler->pcidev = 
		(struct pci_dev *)(container_of(host->self.controller, struct pci_dev, dev));
	xhci_ctler->xhci = hcd_to_xhci(host);
	xhci_ctler->hcd = host;
	xhci_ctler->xhci_drv = host->driver;

	return 0;

  remove_from_list:
	spin_lock_irqsave(&xhci_ctler_list_lock, flags);
	list_del(&xhci_ctler->list);
	spin_unlock_irqrestore(&xhci_ctler_list_lock, flags);
	kfree(xhci_ctler->sgtable);
  free_mem_pool:
	xhci_verify_free_mem_pool(xhci_ctler);
  free_page:
	free_page((unsigned long) xhci_ctler);
  out:
	return ret;
}

EXPORT_SYMBOL(add_xhci_ctler);

static int __init xhci_ctler_verify_init(void)
{
	int ret;

	pr_info("-----------------insert xhci verify---------------------------\n");
	memset(&null_kobj, 0, sizeof(struct kobject));

	ret = kobject_init_and_add(&null_kobj, &null_ktype, NULL, "%s", "xhci_ctler_verify");
	pr_info("-----------------insert xhci verify done-----------------------\n");
	return ret;
}

static void __exit xhci_ctler_verify_exit(void)
{
	unsigned long flags;

	spin_lock_irqsave(&xhci_ctler_list_lock, flags);
	if(!list_empty(&xhci_ctler_list))
	{
		spin_unlock_irqrestore(&xhci_ctler_list_lock, flags);
		printk(KERN_ALERT "sata controller list not empty, shit!");
		return;
	}
	spin_unlock_irqrestore(&xhci_ctler_list_lock, flags);

	kobject_put(&null_kobj);
}

module_init(xhci_ctler_verify_init);
module_exit(xhci_ctler_verify_exit);

MODULE_DESCRIPTION("xhci controller verify");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Cobe Chen, Refactored by Weitao Wang");
MODULE_LICENSE("GPL");
