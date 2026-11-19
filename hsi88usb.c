// SPDX-License-Identifier: GPL-2.0
/*
 * hsi88usb.c - clean-room Linux driver for LDT HSI-88-USB, interface 1
 *
 * Reconstructed from the public command-code document, USBPcap traces and
 * black-box/static analysis of the supplied Windows driver.  It deliberately
 * binds only to USB\VID_1BBF&PID_0064 interface 1 (HSI-88-USB), leaving the
 * DiCoStation interface (interface 0) untouched.
 *
 * Userspace API: /dev/hsi88usbN, using the same documented byte stream as the
 * Windows legacy device: s/L/M/R/CR, m/CR, v/CR, t/CR and asynchronous i frames.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/idr.h>
#include <linux/kref.h>
#include <linux/slab.h>
#include <linux/poll.h>

#define DRV_NAME            "hsi88usb"
#define HSI_VID             0x1bbf
#define HSI_PID             0x0064
#define HSI_INTERFACE       1
#define HSI_EP_OUT          0x02
#define HSI_EP_IN           0x84
#define HSI_IN_MAX          64
#define HSI_FIFO_SIZE       8192

#define HSI_OP_VERSION      0x80
#define HSI_OP_SET_MODULES  0x81
#define HSI_OP_QUERY        0x82
#define HSI_OP_STOP         0x83

#define HSI_LINUX_VERSION_TEXT "hsi88usb-linux 1.1"

struct hsi_parser {
	u8 command;
	u8 args[3];
	u8 argc;
	u8 high_nibble;
	bool have_high_nibble;
	bool t_explicit;
	bool t_value;
};

struct hsi_dev {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct miscdevice misc;
	char *misc_name;
	int unit_number;

	struct urb *in_urb;
	u8 *in_buf;
	dma_addr_t in_dma;
	unsigned int in_interval;

	spinlock_t fifo_lock;
	u8 fifo[HSI_FIFO_SIZE];
	unsigned int fifo_head;
	unsigned int fifo_tail;
	unsigned int fifo_count;
	wait_queue_head_t read_wq;

	struct mutex io_mutex;       /* serializes writes/parser/terminal mode */
	struct hsi_parser parser;
	bool terminal_mode;
	atomic_t opened;
	bool disconnected;
	struct kref kref;
};

static DEFINE_IDA(hsi_units);

static void hsi_delete(struct kref *kref)
{
	struct hsi_dev *dev = container_of(kref, struct hsi_dev, kref);

	if (dev->in_urb)
		usb_free_urb(dev->in_urb);
	if (dev->in_buf)
		usb_free_coherent(dev->udev, HSI_IN_MAX, dev->in_buf, dev->in_dma);
	if (dev->unit_number > 0)
		ida_free(&hsi_units, dev->unit_number);
	usb_put_dev(dev->udev);
	kfree(dev->misc_name);
	kfree(dev);
}

static u8 hsi_lower(u8 c)
{
	return (c >= 'A' && c <= 'Z') ? c + ('a' - 'A') : c;
}

static bool hsi_hex_nibble(u8 c, u8 *v)
{
	if (c >= '0' && c <= '9') { *v = c - '0'; return true; }
	if (c >= 'a' && c <= 'f') { *v = c - 'a' + 10; return true; }
	if (c >= 'A' && c <= 'F') { *v = c - 'A' + 10; return true; }
	return false;
}

static void hsi_parser_reset(struct hsi_dev *dev)
{
	memset(&dev->parser, 0, sizeof(dev->parser));
}

static void hsi_fifo_put(struct hsi_dev *dev, const u8 *data, size_t len)
{
	unsigned long flags;
	size_t i;

	spin_lock_irqsave(&dev->fifo_lock, flags);
	for (i = 0; i < len; ++i) {
		if (dev->fifo_count == HSI_FIFO_SIZE) {
			/* Keep the newest feedback if userspace stalls. */
			dev->fifo_tail = (dev->fifo_tail + 1) % HSI_FIFO_SIZE;
			dev->fifo_count--;
		}
		dev->fifo[dev->fifo_head] = data[i];
		dev->fifo_head = (dev->fifo_head + 1) % HSI_FIFO_SIZE;
		dev->fifo_count++;
	}
	spin_unlock_irqrestore(&dev->fifo_lock, flags);
	wake_up_interruptible(&dev->read_wq);
}

static void hsi_append_byte(u8 *out, size_t cap, size_t *n, u8 b)
{
	if (*n < cap)
		out[(*n)++] = b;
}

static void hsi_append_value(struct hsi_dev *dev, u8 *out, size_t cap,
			     size_t *n, u8 value)
{
	static const u8 hex[] = "0123456789ABCDEF";

	if (READ_ONCE(dev->terminal_mode)) {
		hsi_append_byte(out, cap, n, hex[(value >> 4) & 0xf]);
		hsi_append_byte(out, cap, n, hex[value & 0xf]);
	} else {
		hsi_append_byte(out, cap, n, value);
	}
}

static int hsi_usb_out(struct hsi_dev *dev, const u8 *data, size_t len)
{
	int actual = 0;
	int ret;
	u8 *dma_buf;

	if (READ_ONCE(dev->disconnected))
		return -ENODEV;
	if (!len)
		return 0;

	/*
	 * usb_interrupt_msg() may DMA-map its transfer buffer.  Callers build
	 * packets on the kernel stack, which is not a valid DMA buffer on many
	 * host controllers.  Give USB core a kmalloc-backed buffer instead.
	 */
	dma_buf = kmemdup(data, len, GFP_KERNEL);
	if (!dma_buf)
		return -ENOMEM;

	ret = usb_interrupt_msg(dev->udev,
				usb_sndintpipe(dev->udev, HSI_EP_OUT),
				dma_buf, len, &actual, 1000);
	kfree(dma_buf);

	if (ret)
		return ret;
	return actual == len ? 0 : -EIO;
}

/*
 * Decode the raw HSI firmware frames.  USB IN packets have no length prefix:
 *   s,total
 *   I,count,(HB,LB)*count             initial/full state, no module numbers
 *   m,count,(HB,LB)*count             query response, no module numbers
 *   i,count,(module,HB,LB)*count      changed modules only
 *   v,firmware-major,firmware-minor
 */
static void hsi_decode_in(struct hsi_dev *dev, const u8 *data, size_t len)
{
	u8 out[256];
	size_t n = 0, i, count;

	if (!len)
		return;

	/* terminal_mode is changed under io_mutex; a byte read is sufficient here. */
	switch (data[0]) {
	case 's':
		hsi_append_byte(out, sizeof(out), &n, 's');
		if (len >= 2)
			hsi_append_value(dev, out, sizeof(out), &n, data[1]);
		hsi_append_byte(out, sizeof(out), &n, '\r');
		break;

	case 'i':
		hsi_append_byte(out, sizeof(out), &n, 'i');
		if (len < 2) {
			hsi_append_byte(out, sizeof(out), &n, '\r');
			break;
		}
		count = data[1];
		hsi_append_value(dev, out, sizeof(out), &n, count);
		for (i = 0; i < count * 3 && 2 + i < len; ++i)
			hsi_append_value(dev, out, sizeof(out), &n, data[2 + i]);
		hsi_append_byte(out, sizeof(out), &n, '\r');
		break;

	case 'I':
	case 'm':
		hsi_append_byte(out, sizeof(out), &n, data[0] == 'm' ? 'm' : 'i');
		if (len < 2) {
			hsi_append_byte(out, sizeof(out), &n, '\r');
			break;
		}
		/* Both I and m are capture-proven as [tag,count,hi,lo,...],
		 * with no module numbers on USB. Insert 1..count for the
		 * legacy/documented userspace byte stream. */
		count = min_t(size_t, data[1], 31);
		hsi_append_value(dev, out, sizeof(out), &n, count);
		for (i = 0; i < count; ++i) {
			size_t off = 2 + i * 2;
			if (off + 1 >= len)
				break;
			hsi_append_value(dev, out, sizeof(out), &n, i + 1);
			hsi_append_value(dev, out, sizeof(out), &n, data[off]);
			hsi_append_value(dev, out, sizeof(out), &n, data[off + 1]);
		}
		hsi_append_byte(out, sizeof(out), &n, '\r');
		break;

	case 'v':
		if (len >= 3) {
			char v[128];
			int l = scnprintf(v, sizeof(v),
				"V. %u.%02u / HSI-88-USB / %s / clean-room\r",
				data[1], data[2], HSI_LINUX_VERSION_TEXT);
			hsi_fifo_put(dev, (u8 *)v, l);
		}
		return;

	default:
		return;
	}

	hsi_fifo_put(dev, out, n);
}

static void hsi_in_complete(struct urb *urb)
{
	struct hsi_dev *dev = urb->context;
	int ret;

	if (urb->status == 0 && urb->actual_length)
		hsi_decode_in(dev, dev->in_buf, urb->actual_length);

	if (READ_ONCE(dev->disconnected))
		return;

	switch (urb->status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		return;
	default:
		break;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret && ret != -ENODEV)
		dev_dbg(&dev->intf->dev, "IN resubmit failed: %d\n", ret);
}

static int hsi_get_stored_unit(struct hsi_dev *dev, u8 *unit)
{
	u8 *dma_buf;
	int ret;

	*unit = 0;

	/* usb_control_msg() also DMA-maps its data buffer on this HCD. */
	dma_buf = kmalloc(1, GFP_KERNEL);
	if (!dma_buf)
		return -ENOMEM;
	*dma_buf = 0;

	ret = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
			      0, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_ENDPOINT,
			      0, 0, dma_buf, 1, 1000);
	if (ret == 1)
		*unit = *dma_buf;
	else if (ret >= 0)
		ret = -EIO;

	kfree(dma_buf);
	return ret < 0 ? ret : 0;
}

static int hsi_allocate_unit(struct hsi_dev *dev)
{
	u8 stored = 0;
	int n;

	if (!hsi_get_stored_unit(dev, &stored) && stored >= 1) {
		n = ida_alloc_range(&hsi_units, stored, stored, GFP_KERNEL);
		if (n >= 0)
			return n;
	}
	/* Do not write a new persistent number: the corresponding SET transaction
	 * is inferred from the old driver but absent from the supplied captures. */
	return ida_alloc_range(&hsi_units, 1, 255, GFP_KERNEL);
}

static int hsi_execute(struct hsi_dev *dev)
{
	u8 p[5];
	u8 reply[3];

	switch (dev->parser.command) {
	case 'v':
		/* hsi88usb-v.pcapng proves v\r -> 02 80, reply 76 01 08 for FW 1.08. */
		p[0] = 2; p[1] = HSI_OP_VERSION;
		return hsi_usb_out(dev, p, 2);
	case 'm':
		/* hsi88usb-m.pcapng proves m\r -> 02 82. */
		p[0] = 2; p[1] = HSI_OP_QUERY;
		return hsi_usb_out(dev, p, 2);
	case 's':
		p[0] = 5; p[1] = HSI_OP_SET_MODULES;
		p[2] = dev->parser.args[0];
		p[3] = dev->parser.args[1];
		p[4] = dev->parser.args[2];
		return hsi_usb_out(dev, p, 5);
	case 't':
		if (dev->parser.t_explicit)
			dev->terminal_mode = dev->parser.t_value;
		else
			dev->terminal_mode = !dev->terminal_mode;
		reply[0] = 't'; reply[1] = dev->terminal_mode ? '1' : '0'; reply[2] = '\r';
		hsi_fifo_put(dev, reply, sizeof(reply));
		return 0;
	default:
		return -EINVAL;
	}
}

static int hsi_feed_byte(struct hsi_dev *dev, u8 c)
{
	u8 lower, nib;
	int ret;

	if (!dev->parser.command) {
		lower = hsi_lower(c);
		if (lower == 's' || lower == 'm' || lower == 'v' || lower == 't') {
			dev->parser.command = lower;
			return 0;
		}
		if (c == '\r' || c == '\n')
			return 0;
		c = '\r';
		hsi_fifo_put(dev, &c, 1);
		return 0;
	}

	if (dev->parser.command == 't') {
		if (c == '\r') {
			ret = hsi_execute(dev);
			hsi_parser_reset(dev);
			return ret;
		}
		if (!dev->parser.t_explicit && (c == '0' || c == '1')) {
			dev->parser.t_explicit = true;
			dev->parser.t_value = (c == '1');
			return 0;
		}
		hsi_parser_reset(dev);
		return -EINVAL;
	}

	if (dev->parser.command == 'm' || dev->parser.command == 'v') {
		if (c != '\r') {
			hsi_parser_reset(dev);
			return -EINVAL;
		}
		ret = hsi_execute(dev);
		hsi_parser_reset(dev);
		return ret;
	}

	if (dev->parser.command == 's') {
		if (dev->parser.argc < 3) {
			if (READ_ONCE(dev->terminal_mode)) {
				if (!hsi_hex_nibble(c, &nib)) {
					hsi_parser_reset(dev);
					return -EINVAL;
				}
				if (!dev->parser.have_high_nibble) {
					dev->parser.high_nibble = nib;
					dev->parser.have_high_nibble = true;
				} else {
					dev->parser.args[dev->parser.argc++] =
						(dev->parser.high_nibble << 4) | nib;
					dev->parser.have_high_nibble = false;
				}
			} else {
				dev->parser.args[dev->parser.argc++] = c;
			}
			return 0;
		}
		if (c != '\r' || dev->parser.have_high_nibble) {
			hsi_parser_reset(dev);
			return -EINVAL;
		}
		ret = hsi_execute(dev);
		hsi_parser_reset(dev);
		return ret;
	}

	return -EINVAL;
}

static int hsi_open(struct inode *inode, struct file *file)
{
	struct miscdevice *m = file->private_data;
	struct hsi_dev *dev = container_of(m, struct hsi_dev, misc);

	if (atomic_cmpxchg(&dev->opened, 0, 1) != 0)
		return -EBUSY;
	if (READ_ONCE(dev->disconnected)) {
		atomic_set(&dev->opened, 0);
		return -ENODEV;
	}
	kref_get(&dev->kref);
	file->private_data = dev;
	nonseekable_open(inode, file);
	return 0;
}

static int hsi_release(struct inode *inode, struct file *file)
{
	struct hsi_dev *dev = file->private_data;
	u8 stop[2] = { 2, HSI_OP_STOP };

	mutex_lock(&dev->io_mutex);
	if (!READ_ONCE(dev->disconnected))
		(void)hsi_usb_out(dev, stop, sizeof(stop));
	WRITE_ONCE(dev->terminal_mode, false);
	hsi_parser_reset(dev);
	mutex_unlock(&dev->io_mutex);
	atomic_set(&dev->opened, 0);
	kref_put(&dev->kref, hsi_delete);
	return 0;
}

static ssize_t hsi_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos)
{
	struct hsi_dev *dev = file->private_data;
	unsigned long flags;
	u8 *tmp;
	size_t n = 0, i;
	int ret;

	if (!count)
		return 0;

	for (;;) {
		spin_lock_irqsave(&dev->fifo_lock, flags);
		if (dev->fifo_count)
			break;
		spin_unlock_irqrestore(&dev->fifo_lock, flags);

		if (READ_ONCE(dev->disconnected))
			return -ENODEV;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(dev->read_wq,
			READ_ONCE(dev->fifo_count) || READ_ONCE(dev->disconnected));
		if (ret)
			return ret;
	}

	n = min_t(size_t, count, dev->fifo_count);
	spin_unlock_irqrestore(&dev->fifo_lock, flags);

	tmp = kmalloc(n, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	spin_lock_irqsave(&dev->fifo_lock, flags);
	n = min_t(size_t, n, dev->fifo_count);
	for (i = 0; i < n; ++i) {
		tmp[i] = dev->fifo[dev->fifo_tail];
		dev->fifo_tail = (dev->fifo_tail + 1) % HSI_FIFO_SIZE;
		dev->fifo_count--;
	}
	spin_unlock_irqrestore(&dev->fifo_lock, flags);

	if (copy_to_user(buf, tmp, n)) {
		/* Stream semantics: like a serial driver, a failed userspace copy is an
		 * exceptional condition; consumed bytes are not put back. */
		kfree(tmp);
		return -EFAULT;
	}
	kfree(tmp);
	return n;
}

static ssize_t hsi_write(struct file *file, const char __user *buf, size_t count,
			 loff_t *ppos)
{
	struct hsi_dev *dev = file->private_data;
	u8 *tmp;
	size_t i;
	int ret = 0;

	if (!count)
		return 0;
	if (READ_ONCE(dev->disconnected))
		return -ENODEV;
	if (count > 4096)
		return -EMSGSIZE;

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	mutex_lock(&dev->io_mutex);
	for (i = 0; i < count; ++i) {
		ret = hsi_feed_byte(dev, tmp[i]);
		if (ret)
			break;
	}
	mutex_unlock(&dev->io_mutex);
	kfree(tmp);
	return ret ? ret : count;
}

static __poll_t hsi_poll(struct file *file, poll_table *wait)
{
	struct hsi_dev *dev = file->private_data;
	__poll_t mask = 0;
	unsigned long flags;

	poll_wait(file, &dev->read_wq, wait);
	spin_lock_irqsave(&dev->fifo_lock, flags);
	if (dev->fifo_count)
		mask |= EPOLLIN | EPOLLRDNORM;
	spin_unlock_irqrestore(&dev->fifo_lock, flags);
	if (READ_ONCE(dev->disconnected))
		mask |= EPOLLHUP | EPOLLERR;
	return mask;
}

static const struct file_operations hsi_fops = {
	.owner          = THIS_MODULE,
	.open           = hsi_open,
	.release        = hsi_release,
	.read           = hsi_read,
	.write          = hsi_write,
	.poll           = hsi_poll,
};

static int hsi_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_host_interface *alt = intf->cur_altsetting;
	struct hsi_dev *dev;
	struct usb_endpoint_descriptor *ep;
	unsigned int i;
	bool have_in = false, have_out = false;
	int ret;

	if (alt->desc.bInterfaceNumber != HSI_INTERFACE)
		return -ENODEV;

	for (i = 0; i < alt->desc.bNumEndpoints; ++i) {
		ep = &alt->endpoint[i].desc;
		if (usb_endpoint_is_int_in(ep) && ep->bEndpointAddress == HSI_EP_IN)
			have_in = true;
		if (usb_endpoint_is_int_out(ep) && ep->bEndpointAddress == HSI_EP_OUT)
			have_out = true;
	}
	if (!have_in || !have_out)
		return -ENODEV;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	kref_init(&dev->kref);
	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	dev->intf = intf;
	spin_lock_init(&dev->fifo_lock);
	init_waitqueue_head(&dev->read_wq);
	mutex_init(&dev->io_mutex);
	atomic_set(&dev->opened, 0);

	dev->unit_number = hsi_allocate_unit(dev);
	if (dev->unit_number < 0) {
		ret = dev->unit_number;
		dev->unit_number = 0;
		goto err_put;
	}
	dev->misc_name = kasprintf(GFP_KERNEL, "hsi88usb%d", dev->unit_number);
	if (!dev->misc_name) { ret = -ENOMEM; goto err_put; }

	dev->misc.minor = MISC_DYNAMIC_MINOR;
	dev->misc.name = dev->misc_name;
	dev->misc.fops = &hsi_fops;
	dev->misc.parent = &intf->dev;
	dev->misc.mode = 0660;

	/* Find the actual IN interval from the descriptor. */
	dev->in_interval = 2;
	for (i = 0; i < alt->desc.bNumEndpoints; ++i) {
		ep = &alt->endpoint[i].desc;
		if (ep->bEndpointAddress == HSI_EP_IN) {
			dev->in_interval = ep->bInterval;
			break;
		}
	}

	dev->in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->in_urb) { ret = -ENOMEM; goto err_put; }
	dev->in_buf = usb_alloc_coherent(dev->udev, HSI_IN_MAX, GFP_KERNEL, &dev->in_dma);
	if (!dev->in_buf) { ret = -ENOMEM; goto err_put; }

	usb_fill_int_urb(dev->in_urb, dev->udev,
			 usb_rcvintpipe(dev->udev, HSI_EP_IN), dev->in_buf, HSI_IN_MAX,
			 hsi_in_complete, dev, dev->in_interval);
	dev->in_urb->transfer_dma = dev->in_dma;
	dev->in_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	ret = misc_register(&dev->misc);
	if (ret)
		goto err_put;

	usb_set_intfdata(intf, dev);
	ret = usb_submit_urb(dev->in_urb, GFP_KERNEL);
	if (ret) {
		usb_set_intfdata(intf, NULL);
		misc_deregister(&dev->misc);
		goto err_put;
	}

	dev_info(&intf->dev,
		 "HSI-88-USB interface ready as /dev/%s (VID:PID %04x:%04x, interface %u)\n",
		 dev->misc_name, HSI_VID, HSI_PID, HSI_INTERFACE);
	return 0;

err_put:
	kref_put(&dev->kref, hsi_delete);
	return ret;
}

static void hsi_disconnect(struct usb_interface *intf)
{
	struct hsi_dev *dev = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);
	if (!dev)
		return;

	WRITE_ONCE(dev->disconnected, true);
	misc_deregister(&dev->misc);
	usb_kill_urb(dev->in_urb);
	wake_up_interruptible(&dev->read_wq);
	kref_put(&dev->kref, hsi_delete);
	dev_info(&intf->dev, "HSI-88-USB disconnected\n");
}

static const struct usb_device_id hsi_table[] = {
	{ USB_DEVICE_INTERFACE_NUMBER(HSI_VID, HSI_PID, HSI_INTERFACE) },
	{ }
};
MODULE_DEVICE_TABLE(usb, hsi_table);

static struct usb_driver hsi_driver = {
	.name       = DRV_NAME,
	.probe      = hsi_probe,
	.disconnect = hsi_disconnect,
	.id_table   = hsi_table,
};

module_usb_driver(hsi_driver);

MODULE_AUTHOR("Clean-room compatibility implementation");
MODULE_DESCRIPTION("LDT HSI-88-USB S88 feedback interface driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
