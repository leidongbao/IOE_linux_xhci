#ifndef __XHCI_CTLER_SPY_H
#define __XHCI_CTLER_SPY_H

#include "xhci.h"
#include "xhci_ctler_verify.h"

struct usb_device *xhci_verify_get_dev_by_route_string(struct xhci_ctler *ctler,
	unsigned int route_string);

struct usb_host_endpoint *xhci_verify_get_ep_by_address(struct usb_device *udev,
	unsigned char direction, unsigned char ep_address);

int xhci_verify_get_mass_storage_info(struct usb_device *udev,
	struct mass_storage_dev_info *info);

int read_cmd_str(struct xhci_ctler *ctler, unsigned int *cmd_code,
	unsigned int *route_string, unsigned int *request_code);
int check_num_parameter(struct xhci_ctler *ctler, int ret, int num_of_parameter);
int check_cmd_code(struct xhci_ctler *ctler, unsigned cmd_code, unsigned CMD_CODE);
int check_request_code(struct xhci_ctler *ctler, unsigned request_code,
	unsigned REQUEST_CODE);
int check_route_string(struct xhci_ctler *ctler, unsigned int route_string);
int check_endpoint(struct xhci_ctler *ctler, struct usb_host_endpoint *endpoint);
int check_udev(struct xhci_ctler *ctler, struct usb_device *udev);
int check_hub_device(struct xhci_ctler *ctler, struct usb_device *udev);

int check_urb(struct xhci_ctler *ctler, struct urb *urb);
int check_normal_table(struct xhci_ctler *ctler, struct xhci_mem_table *normal_table);
int check_special_table(struct xhci_ctler *ctler, struct xhci_mem_table *special_table);
int check_null(void *p, char *s);

char *err_str(int ret);

//Dump TRB Field
void dump_field(unsigned field1, unsigned field2, unsigned field3, unsigned field4);

//USB Transer API
struct usb_hub *hdev_to_struct_hub(struct usb_device *hdev);
int xhci_queue_stor_cmd_trb(struct xhci_ctler *ctler, struct usb_device *udev,
	struct usb_host_endpoint *ep, unsigned char *buffer, int length);

//Special SG Table API
struct xhci_mem_table *xhci_verify_create_normal_mem_table(struct xhci_ctler *ctler,
	unsigned long normal_len);
struct xhci_mem_table *xhci_verify_create_special_mem_table(unsigned long special_len);
int xhci_verify_reset_mem_pool_data(struct xhci_ctler *ctler);

int xhci_verify_init_normal_mem_table(struct xhci_mem_table *mem_tbl);
int xhci_verify_init_special_mem_table(struct xhci_mem_table *mem_tbl);
int xhci_verify_reset_special_mem_table_data(struct xhci_mem_table *mem_tbl);
int xhci_verify_normal_mem_table(struct xhci_mem_table *mem_tbl);
int xhci_verify_special_mem_table(struct xhci_mem_table *mem_tbl);
void xhci_verify_free_normal_mem_table(struct xhci_mem_table *mem_tbl);
void xhci_verify_free_special_mem_table(struct xhci_mem_table *mem_tbl);
inline void xhci_verify_dump_normal_mem_table(struct xhci_mem_table *mem_tbl);
inline void xhci_verify_dump_special_mem_table(struct xhci_mem_table *mem_tbl);
int xhci_verify_copy_special_mem_table(struct xhci_mem_table *mem_tbl,
	unsigned char *dst, unsigned int length);
int xhci_verify_free_mem_pool(struct xhci_ctler *ctler);
int xhci_verify_init_mem_pool(struct xhci_ctler *ctler);

#endif
