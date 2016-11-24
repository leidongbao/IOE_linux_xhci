#ifndef __XHCI_CTLER_VERIFY
#define __XHCI_CTLER_VERIFY

#define MAX_LENGTH_PER_TRB    (64*1024)
#define SECTORS_PER_PAGE 	  (PAGE_SIZE / 512)

struct usb3_hub_descriptor
{
	__u8 bDescLength;
	__u8 bDescriptorType;
	__u8 bNbrPorts;
	__le16 wHubCharacteristics;
	__u8 bPwrOn2PwrGood;
	__u8 bHubContrCurrent;
	__u8 bHubHdrDecLat;
	__le16 wHubDealy;
	__le16 DeviceRemovable;
} __attribute__ ((packed));

struct xhci_mem_entry
{
	struct page *page;
	unsigned int high_mem;
	unsigned int offset;
	unsigned int length;
	unsigned int data;			/* will be cast to type "unsigned char" */
	unsigned int data_gen_type;	/* 0: fixed; 1: increase; 2 decrease */
};

static const struct xhci_mem_entry one_max_packet_mem_entries[] = 
{
	{NULL, 0, 512, 32, 0, 0},
	{NULL, 0, 296, 256, 0, 0},
	{NULL, 1, 168, 128, 0, 1},
	{NULL, 0, 104, 64, 0, 2},
	{NULL, 1, 72, 20, 0, 0},
	{NULL, 0, 56, 10, 0, 1},
	{NULL, 1, 42, 1000, 0, 2},
	{NULL, 0, 30, 8, 0, 0},
	{NULL, 1, 6, 6, 0, 1},
	{NULL, 0, 4, 4, 0, 2},
	{NULL, 1, 3, 3, 0, 0},
	{NULL, 0, 2, 2, 0, 1},
	{NULL, 1, 1, 1, 0, 2},
};

#define ONE_MAX_PACKET_MEM_ENTRY_CNT \
	(sizeof(one_max_packet_mem_entries) / sizeof(one_max_packet_mem_entries[0]))

struct xhci_mem_table
{
	unsigned int nents;
	struct xhci_mem_entry entry[0];
};

struct xhci_mem_pool
{
	unsigned int target_num;
	unsigned int actual_num;
	int order;
	int length_in_page;
	struct page *page_table[512];
};

struct mass_storage_dev_info
{
	struct usb_host_endpoint *control;
	struct usb_host_endpoint *bulk_in;
	struct usb_host_endpoint *bulk_out;
	struct usb_host_endpoint *int_in;
	unsigned int snd_ctl_pipe;
	unsigned int rcv_ctl_pipe;
	unsigned int bulk_in_pipe;
	unsigned int bulk_out_pipe;
	unsigned int int_in_pipe;

	struct usb_interface *active_intf;
	unsigned char protocol;
};

#define MAX_POOL_ENTRY 5

struct xhci_ctler;

enum verify_cmd_code
{
	HUB_REQUEST 		    = 1,
	GET_DEV_DESC 		    = 2,
	SET_TRANSFER_SEG_NUM    = 3,
	SG_TRANSFER 		    = 4,
	SG_VERIFY 			    = 5,
	ISOCH_UNDERRN_OVERRUN   = 6,
	EVENT_DATA_TRB 		    = 7,
	CMD_VERIFY 			    = 8,
	SET_EVENT_SEG_NUM       = 9,
	VERIFY_EVENT_FULL 	    = 10,
	SET_INTR_MODERATION     = 11,
	SET_U1U2_TIMEOUT 	    = 12,
	SET_LPM_STATE 			= 13,
	FUNC_SUSPEND 			= 14,
	ENABLE_LTM 				= 15,
	VERIFY_PPC 				= 16,
	VERIFY_HANDOFF 			= 17,
	WARM_RESET 				= 18,
	ZERO_LENGTH_TRB 		= 19,
	TEST					= 20,
	U3_ENTRY_CAPABILITY		=21,
	MAX_CMD_CODE 			= 22,
};

struct verify_func
{
	enum verify_cmd_code cmd_code;
	int (*func)(struct xhci_ctler * ctler);
};

#define MAX_TEST_CMD_LEN 256

/* Here left 8 bytes as null-terminate character */
#define MAX_TEST_RESULT_LEN (PAGE_SIZE - sizeof(struct xhci_ctler) - 8)

enum test_state
{
	TEST_IDLE     = 0,
	TEST_RUNNING  = 1,
	TEST_COMPLETE = 2,
};

struct xhci_ctler_test
{
	enum test_state state;

	struct completion xhci_urb_completion;
	struct completion xhci_urb_completion2;

	/* This cmd_str is not scsi command cdb.
	 * It's the command string from user space test tool.
	 */
	char cmd_str[MAX_TEST_CMD_LEN];

	char *rb_cur_ptr;			/* result buffer current pointer */
	int  rb_remain_len;			/* result buffer remain length */
	char result[0];
};

struct xhci_ctler
{
	spinlock_t lock;
	unsigned int index;

	struct pci_dev   *pcidev;	/* pcidev contains xhci */
	struct xhci_hcd  *xhci;		/* xhci contains usb_hcd */
	struct usb_hcd   *hcd;		/* usb_hcd contains hc_driver and usb_bus */
	struct hc_driver *xhci_drv;

	/* for 64K, 32K, 16K, 8K, 4K */
	struct xhci_mem_pool *mem_pool;
	struct sg_table *sgtable;

	struct list_head list;
	struct kobject kobj;
	unsigned int kobj_valid;

	/* the verify status */
	enum test_state verify_status;

	/* Note: This element must be put at the last of the structure */
	struct xhci_ctler_test test;
};

/* sys file operation related */
#define MAX_ATTR_SHOW_DATA_LEN (PAGE_SIZE - 1)

/* print sys file in some format */
#define SYSFS_PRINT(fmt, args...) \
do{								  \
    int str_len;				  \
    if(remain_buf_len == 0)		  \
    {							  \
        goto out;				  \
    }							  \
    str_len = snprintf(cur_buf_ptr, remain_buf_len,fmt,## args);\
    if(str_len < remain_buf_len)  \
    {							  \
        cur_buf_ptr += str_len;   \
        remain_buf_len -= str_len;\
    }							  \
    else						  \
    {							  \
    	/* It's better to add a "line return" in the last */   \
        *(((char *)cur_buf_ptr) + remain_buf_len - 2) = '\n'; \
        *(((char *)cur_buf_ptr) + remain_buf_len - 1) = 0;    \
        remain_buf_len = 0;									   \
        goto out;											   \
    }                                                          \
}while(0);

struct xhci_ctler_sysfs_attr
{
	struct attribute attr;
	  ssize_t(*show) (struct xhci_ctler *, char *);
	  ssize_t(*store) (struct xhci_ctler *, const char *, size_t count);
};

#define XHCI_CTLER_ATTR_RO(_name) \
struct xhci_ctler_sysfs_attr xhci_ctler_attr_##_name = \
			__ATTR(_name, S_IRUGO, xhci_ctler_attr_##_name##_show, NULL)

#define XHCI_CTLER_ATTR_WO(_name) \
struct xhci_ctler_sysfs_attr xhci_ctler_attr_##_name = \
			__ATTR(_name, S_IWUSR, NULL, xhci_ctler_attr_##_name##_store)

#define XHCI_CTLER_ATTR_RW(_name) \
struct xhci_ctler_sysfs_attr xhci_ctler_attr_##_name = \
			__ATTR(_name, S_IRUGO | S_IWUSR, xhci_ctler_attr_##_name##_show, \
			       xhci_ctler_attr_##_name##_store)

static inline int write_log(struct xhci_ctler *ctler, const char *fmt, ...)
{
	va_list args;
	int i;

	va_start(args, fmt);
	i = vscnprintf(ctler->test.rb_cur_ptr, ctler->test.rb_remain_len, fmt, args);
	va_end(args);
	ctler->test.rb_cur_ptr += i;
	ctler->test.rb_remain_len -= i;
	return i;
}

int add_xhci_ctler(struct usb_hcd *hcd);
int remove_xhci_ctler(struct usb_hcd *hcd);

void print_transfer_ring(struct xhci_hcd *xhci, struct usb_device *udev,
	struct usb_host_endpoint *ep);
void print_event_ring(struct xhci_hcd *xhci);

#endif
