// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) 2018-2019  Realtek Corporation
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include "debug.h"
#include "fw.h"
#include "usb.h"

#define RTW_USB_CONTROL_MSG_TIMEOUT	30000 /* (us) */
#define RTW_USB_MSG_TIMEOUT	30000 /* (ms) */
#define RTW_USB_MAX_RXQ_LEN	128

struct rtw_usb_txcb_t {
	struct rtw89_dev *rtwdev;
	struct sk_buff_head tx_ack_queue;
};

struct rtw_usb_ctrlcb_t {
	atomic_t done;
	__u8 req_type;
	int status;
};

/*
 * usb read/write register functions
 */

static void rtw_usb_ctrl_atomic_cb(struct urb *urb)
{
	struct rtw_usb_ctrlcb_t *ctx;

	if (unlikely(!urb))
		return;

	ctx = (struct rtw_usb_ctrlcb_t *)urb->context;
	if (likely(ctx)) {
		atomic_set(&ctx->done, 1);
		ctx->status = urb->status;
	}

	/* free dr */
	kfree(urb->setup_packet);
}

static int rtw_usb_ctrl_atomic(struct rtw89_dev *rtwdev,
			       struct usb_device *dev, unsigned int pipe,
			       __u8 req_type, __u16 value, __u16 index,
			       void *databuf, __u16 size)
{
	struct usb_ctrlrequest *dr = NULL;
	struct rtw_usb_ctrlcb_t *ctx = NULL;
	struct urb *urb = NULL;
	bool done;
	int ret = -ENOMEM;

	ctx = kmalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx)
		goto out;

	dr = kmalloc(sizeof(*dr), GFP_ATOMIC);
	if (!dr)
		goto err_free_ctx;

	dr->bRequestType = req_type;
	dr->bRequest = RTW_USB_CMD_REQ;
	dr->wValue = cpu_to_le16(value);
	dr->wIndex = cpu_to_le16(index);
	dr->wLength = cpu_to_le16(size);

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto err_free_dr;

	atomic_set(&ctx->done, 0);
	ctx->req_type = req_type;
	usb_fill_control_urb(urb, dev, pipe, (unsigned char *)dr, databuf, size,
			     rtw_usb_ctrl_atomic_cb, ctx);
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (unlikely(ret)) {
		rtw89_err(rtwdev, "failed to submit urb, ret=%d\n", ret);
		goto err_free_urb;
	}

	done = false;
	read_poll_timeout_atomic(atomic_read, done, done, 100,
				 RTW_USB_CONTROL_MSG_TIMEOUT, false,
				 &ctx->done);
	if (!done) {
		usb_kill_urb(urb);
		rtw89_err(rtwdev, "failed to wait usb ctrl req:%u\n", req_type);
		ret = (ctx->status == -ENOENT ? -ETIMEDOUT : ctx->status);
	} else {
		ret = 0;
	}

	kfree(ctx);
	usb_free_urb(urb);
	return ret;

err_free_urb:
	usb_free_urb(urb);
err_free_dr:
	kfree(dr);
err_free_ctx:
	kfree(ctx);
out:
	return ret;
}

static u8 rtw_usb_read8_atomic(struct rtw89_dev *rtwdev, u32 addr)
{
	struct rtw_usb *rtwusb = (struct rtw_usb *)rtwdev->priv;
	struct usb_device *udev = rtwusb->udev;
	u8 *buf = NULL, data;

	buf = kmalloc(sizeof(*buf), GFP_ATOMIC);
	if (!buf)
		return 0;

	rtw_usb_ctrl_atomic(rtwdev, udev, usb_rcvctrlpipe(udev, 0),
			    RTW_USB_CMD_READ, addr, 0, buf, sizeof(*buf));
	data = *buf;
	kfree(buf);

	return data;
}

static u16 rtw_usb_read16_atomic(struct rtw89_dev *rtwdev, u32 addr)
{
	struct rtw_usb *rtwusb = (struct rtw_usb *)rtwdev->priv;
	struct usb_device *udev = rtwusb->udev;
	__le16 *buf = NULL;
	u16 data;

	buf = kmalloc(sizeof(*buf), GFP_ATOMIC);
	if (!buf)
		return 0;

	rtw_usb_ctrl_atomic(rtwdev, udev, usb_rcvctrlpipe(udev, 0),
			    RTW_USB_CMD_READ, addr, 0, buf, sizeof(*buf));
	data = *buf;
	kfree(buf);

	return data;
}

static u32 rtw_usb_read32_atomic(struct rtw89_dev *rtwdev, u32 addr)
{
	struct rtw_usb *rtwusb = (struct rtw_usb *)rtwdev->priv;
	struct usb_device *udev = rtwusb->udev;
	__le32 *buf;
	u32 data;

	buf = kmalloc(sizeof(*buf), GFP_ATOMIC);
	if (!buf)
		return 0;

	rtw_usb_ctrl_atomic(rtwdev, udev, usb_rcvctrlpipe(udev, 0),
			    RTW_USB_CMD_READ, addr, 0, buf, sizeof(*buf));
	data = *buf;
	kfree(buf);

	return data;
}

static void rtw_usb_write8_atomic(struct rtw89_dev *rtwdev, u32 addr, u8 val)
{
	struct rtw_usb *rtwusb = (struct rtw_usb *)rtwdev->priv;
	struct usb_device *udev = rtwusb->udev;
	u8 *buf;

	buf = kmalloc(sizeof(*buf), GFP_ATOMIC);
	if (!buf)
		return;

	*buf = val;
	rtw_usb_ctrl_atomic(rtwdev, udev, usb_sndctrlpipe(udev, 0),
			    RTW_USB_CMD_WRITE, addr, 0, buf, sizeof(*buf));
	kfree(buf);
}

static void rtw_usb_write16_atomic(struct rtw89_dev *rtwdev, u32 addr, u16 val)
{
	struct rtw_usb *rtwusb = (struct rtw_usb *)rtwdev->priv;
	struct usb_device *udev = rtwusb->udev;
	__le16 *buf;

	buf = kmalloc(sizeof(*buf), GFP_ATOMIC);
	if (!buf)
		return;

	*buf = cpu_to_le16(val);
	rtw_usb_ctrl_atomic(rtwdev, udev, usb_sndctrlpipe(udev, 0),
			    RTW_USB_CMD_WRITE, addr, 0, buf, sizeof(*buf));
	kfree(buf);
}

static void rtw_usb_write32_atomic(struct rtw89_dev *rtwdev, u32 addr, u32 val)
{
	struct rtw_usb *rtwusb = (struct rtw_usb *)rtwdev->priv;
	struct usb_device *udev = rtwusb->udev;
	__le32 *buf;

	buf = kmalloc(sizeof(*buf), GFP_ATOMIC);
	if (!buf)
		return;

	*buf = cpu_to_le32(val);
	rtw_usb_ctrl_atomic(rtwdev, udev, usb_sndctrlpipe(udev, 0),
			    RTW_USB_CMD_WRITE, addr, 0, buf, sizeof(*buf));
	kfree(buf);
}

static int rtw_usb_parse(struct rtw89_dev *rtwdev,
			 struct usb_interface *interface)
{
	struct rtw_usb *rtwusb;
	struct usb_interface_descriptor *interface_desc;
	struct usb_host_interface *host_interface;
	struct usb_endpoint_descriptor *endpoint;
	struct device *dev;
	struct usb_device *usbd;
	int i, j = 0, endpoints;
	u8 dir, xtype, num;
	int ret = 0;

	rtwusb = rtw_get_usb_priv(rtwdev);

	dev = &rtwusb->udev->dev;

	usbd = interface_to_usbdev(interface);
	host_interface = &interface->altsetting[0];
	interface_desc = &host_interface->desc;
	endpoints = interface_desc->bNumEndpoints;

	rtwusb->num_in_pipes = 0;
	rtwusb->num_out_pipes = 0;
	for (i = 0; i < endpoints; i++) {
		endpoint = &host_interface->endpoint[i].desc;
		dir = endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK;
		num = usb_endpoint_num(endpoint);
		xtype = usb_endpoint_type(endpoint);
		rtw89_info(rtwdev, "\nusb endpoint descriptor (%i):\n", i);
		rtw89_info(rtwdev, "bLength=%x\n", endpoint->bLength);
		rtw89_info(rtwdev, "bDescriptorType=%x\n",
			 endpoint->bDescriptorType);
		rtw89_info(rtwdev, "bEndpointAddress=%x\n",
			 endpoint->bEndpointAddress);
		rtw89_info(rtwdev, "wMaxPacketSize=%d\n",
			 le16_to_cpu(endpoint->wMaxPacketSize));
		rtw89_info(rtwdev, "bInterval=%x\n", endpoint->bInterval);

		if (usb_endpoint_dir_in(endpoint) &&
		    usb_endpoint_xfer_bulk(endpoint)) {
			rtw89_info(rtwdev, "USB: dir in endpoint num %i\n", num);

			if (rtwusb->pipe_in) {
				rtw89_err(rtwdev,
					"failed to get many IN pipes\n");
				ret = -EINVAL;
				goto exit;
			}

			rtwusb->pipe_in = num;
			rtwusb->num_in_pipes++;
		}

		if (usb_endpoint_dir_in(endpoint) &&
		    usb_endpoint_xfer_int(endpoint)) {
			rtw89_info(rtwdev, "USB: interrupt endpoint num %i\n",
				 num);

			if (rtwusb->pipe_interrupt) {
				rtw89_err(rtwdev,
					"failed to get many INTERRUPT pipes\n");
				ret = -EINVAL;
				goto exit;
			}

			rtwusb->pipe_interrupt = num;
		}

		if (usb_endpoint_dir_out(endpoint) &&
		    usb_endpoint_xfer_bulk(endpoint)) {
			rtw89_info(rtwdev, "USB: out endpoint num %i\n", num);
			if (j >= 4) {
				rtw89_err(rtwdev,
					"failed to get many OUT pipes\n");
				ret = -EINVAL;
				goto exit;
			}

			/* for out enpoint, address == number */
			rtwusb->out_ep[j++] = num;
			rtwusb->num_out_pipes++;
		}
	}

	switch (usbd->speed) {
	case USB_SPEED_LOW:
		rtw89_info(rtwdev, "USB_SPEED_LOW\n");
		rtwusb->usb_speed = RTW_USB_SPEED_1_1;
		break;
	case USB_SPEED_FULL:
		rtw89_info(rtwdev, "USB_SPEED_FULL\n");
		rtwusb->usb_speed = RTW_USB_SPEED_1_1;
		break;
	case USB_SPEED_HIGH:
		rtw89_info(rtwdev, "USB_SPEED_HIGH\n");
		rtwusb->usb_speed = RTW_USB_SPEED_2;
		break;
	case USB_SPEED_SUPER:
		rtw89_info(rtwdev, "USB_SPEED_SUPER\n");
		rtwusb->usb_speed = RTW_USB_SPEED_3;
		break;
	default:
		rtw89_err(rtwdev, "failed to get USB speed\n");
		break;
	}

exit:
	return ret;
}

/*
 * driver status relative functions
 */
static
bool rtw_usb_is_bus_ready(struct rtw89_dev *rtwdev)
{
	struct rtw_usb *rtwusb = rtw_get_usb_priv(rtwdev);

	return (atomic_read(&rtwusb->is_bus_drv_ready) == true);
}

static
void rtw_usb_set_bus_ready(struct rtw89_dev *rtwdev, bool ready)
{
	struct rtw_usb *rtwusb = rtw_get_usb_priv(rtwdev);

	atomic_set(&rtwusb->is_bus_drv_ready, ready);
}

static void rtw_usb_interface_configure(struct rtw89_dev *rtwdev)
{
	struct rtw_usb *rtwusb = rtw_get_usb_priv(rtwdev);

	if (RTW_USB_IS_SUPER_SPEED(rtwusb))
		rtwusb->bulkout_size = RTW_USB_SUPER_SPEED_BULK_SIZE;
	else if (RTW_USB_IS_HIGH_SPEED(rtwusb))
		rtwusb->bulkout_size = RTW_USB_HIGH_SPEED_BULK_SIZE;
	else
		rtwusb->bulkout_size = RTW_USB_FULL_SPEED_BULK_SIZE;
	rtw89_info(rtwdev, "USB: bulkout_size: %d\n", rtwusb->bulkout_size);
}

static int rtw_usb_intf_init(struct rtw89_dev *rtwdev,
			     struct usb_interface *intf)
{
	struct rtw_usb *rtwusb = rtw_get_usb_priv(rtwdev);
	struct usb_device *udev = usb_get_dev(interface_to_usbdev(intf));
	int ret;

	rtwusb->udev = udev;
	rtwusb->rtwdev = rtwdev;
	ret = rtw_usb_parse(rtwdev, intf);
	if (ret) {
		rtw89_err(rtwdev, "failed to check USB configuration, ret=%d\n",
			ret);
		return ret;
	}

	usb_set_intfdata(intf, rtwdev->hw);
	rtw_usb_interface_configure(rtwdev);
	SET_IEEE80211_DEV(rtwdev->hw, &intf->dev);

	return 0;
}

static void rtw_usb_intf_deinit(struct rtw89_dev *rtwdev,
				struct usb_interface *intf)
{
	struct rtw_usb *rtwusb = rtw_get_usb_priv(rtwdev);

	usb_put_dev(rtwusb->udev);
	usb_set_intfdata(intf, NULL);
}

static int rtw_usb_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct ieee80211_hw *hw;
	int drv_data_size;

	pr_info("%s enter\n", __func__);

	drv_data_size = sizeof(struct rtw89_dev) + sizeof(struct rtw_usb);
	hw = ieee80211_alloc_hw(drv_data_size, &rtw89_ops);
	if (!hw)
		return -ENOMEM;

	return 0;
}

static void rtw_usb_disconnect(struct usb_interface *intf)
{
	struct ieee80211_hw *hw = usb_get_intfdata(intf);

	pr_info("%s\n", __func__);

	if (!hw)
		return;

	ieee80211_free_hw(hw);
}

static const struct usb_device_id rtw_usb_id_table[] = {
	{ RTK_USB_DEVICE(RTW_USB_VENDOR_ID_REALTEK,
			 0x885a, rtw8852a_chip_info) },
	{},
};
MODULE_DEVICE_TABLE(usb, rtw_usb_id_table);

static struct usb_driver rtw_usb_driver = {
	.name = "rtwifi_usb",
	.id_table = rtw_usb_id_table,
	.probe = rtw_usb_probe,
	.disconnect = rtw_usb_disconnect,
};

module_usb_driver(rtw_usb_driver);

MODULE_AUTHOR("Realtek Corporation");
MODULE_DESCRIPTION("Realtek 802.11ax wireless USB driver");
MODULE_LICENSE("Dual BSD/GPL");
