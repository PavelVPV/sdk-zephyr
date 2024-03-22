/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/usb/usb_device.h>

#include <zephyr/net/buf.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/hci_raw.h>

#define LOG_LEVEL 3
#define LOG_MODULE_NAME hci_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <hal/nrf_mwu.h>
#include <soc.h>

#define NRF_PX(port) ((port == 0) ? NRF_P0 : NRF_P1)

static inline void pin_set(uint32_t port, uint32_t pin)
{
	NRF_GPIO_Type *gpio = NRF_PX(port);
	gpio->OUTSET = 1 << pin;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
}

static inline void pin_clr(uint32_t port, uint32_t pin)
{
	NRF_GPIO_Type *gpio = NRF_PX(port);
	gpio->OUTCLR = 1 << pin;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
}

static inline void pin_toggle(uint32_t port, uint32_t pin, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		pin_set(port, pin);
		pin_clr(port, pin);
	}
}

static void pin_cfg(uint32_t port, uint32_t pin)
{
	static uint32_t pins_configured_mask[GPIO_COUNT];
	if (pins_configured_mask[port] & (1 << pin)) {
		return;
	}

	pins_configured_mask[port] |= (1 << pin);

	NRF_GPIO_Type *gpio = NRF_PX(port);
	gpio->PIN_CNF[pin] = 0;
	gpio->PIN_CNF[pin] |= (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) & GPIO_PIN_CNF_DIR_Msk;
	gpio->PIN_CNF[pin] |= (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) & GPIO_PIN_CNF_PULL_Msk;
	gpio->PIN_CNF[pin] |= (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) & GPIO_PIN_CNF_DRIVE_Msk;
	gpio->PIN_CNF[pin] |= (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) & GPIO_PIN_CNF_SENSE_Msk;
	gpio->OUTCLR = 1 << pin;

	pin_toggle(port, pin, 1);
}

static const struct device *const hci_uart_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_c2h_uart));
static K_THREAD_STACK_DEFINE(tx_thread_stack, CONFIG_BT_HCI_TX_STACK_SIZE);
static struct k_thread tx_thread_data;
static K_FIFO_DEFINE(tx_queue);

/* RX in terms of bluetooth communication */
static K_FIFO_DEFINE(uart_tx_queue);

#define H4_CMD 0x01
#define H4_ACL 0x02
#define H4_SCO 0x03
#define H4_EVT 0x04
#define H4_ISO 0x05

/* Receiver states. */
#define ST_IDLE 0	/* Waiting for packet type. */
#define ST_HDR 1	/* Receiving packet header. */
#define ST_PAYLOAD 2	/* Receiving packet payload. */
#define ST_DISCARD 3	/* Dropping packet. */

/* Length of a discard/flush buffer.
 * This is sized to align with a BLE HCI packet:
 * 1 byte H:4 header + 32 bytes ACL/event data
 * Bigger values might overflow the stack since this is declared as a local
 * variable, smaller ones will force the caller to call into discard more
 * often.
 */
#define H4_DISCARD_LEN 33

static int h4_read(const struct device *uart, uint8_t *buf, size_t len)
{
	int rx = uart_fifo_read(uart, buf, len);

	LOG_DBG("read %d req %d", rx, len);

	return rx;
}

static bool valid_type(uint8_t type)
{
	return (type == H4_CMD) | (type == H4_ACL) | (type == H4_ISO);
}

/* Function expects that type is validated and only CMD, ISO or ACL will be used. */
static uint32_t get_len(const uint8_t *hdr_buf, uint8_t type)
{
	switch (type) {
	case H4_CMD:
		return ((const struct bt_hci_cmd_hdr *)hdr_buf)->param_len;
	case H4_ISO:
		return bt_iso_hdr_len(
			sys_le16_to_cpu(((const struct bt_hci_iso_hdr *)hdr_buf)->len));
	case H4_ACL:
		return sys_le16_to_cpu(((const struct bt_hci_acl_hdr *)hdr_buf)->len);
	default:
		LOG_ERR("Invalid type: %u", type);
		return 0;
	}
}

/* Function expects that type is validated and only CMD, ISO or ACL will be used. */
static int hdr_len(uint8_t type)
{
	switch (type) {
	case H4_CMD:
		return sizeof(struct bt_hci_cmd_hdr);
	case H4_ISO:
		return sizeof(struct bt_hci_iso_hdr);
	case H4_ACL:
		return sizeof(struct bt_hci_acl_hdr);
	default:
		LOG_ERR("Invalid type: %u", type);
		return 0;
	}
}

static void rx_isr(void)
{
	static struct net_buf *buf;
	static int remaining;
	static uint8_t state;
	static uint8_t type;
	static uint8_t hdr_buf[MAX(sizeof(struct bt_hci_cmd_hdr),
			sizeof(struct bt_hci_acl_hdr))];
	int read;

	do {
		switch (state) {
		case ST_IDLE:
			/* Get packet type */
			read = h4_read(hci_uart_dev, &type, sizeof(type));
			/* since we read in loop until no data is in the fifo,
			 * it is possible that read = 0.
			 */
			if (read) {
				if (valid_type(type)) {
					/* Get expected header size and switch
					 * to receiving header.
					 */
					remaining = hdr_len(type);
					state = ST_HDR;
				} else {
					LOG_WRN("Unknown header %d", type);
				}
			}
			break;
		case ST_HDR:
			read = h4_read(hci_uart_dev,
				       &hdr_buf[hdr_len(type) - remaining],
				       remaining);
			remaining -= read;
			if (remaining == 0) {
				/* Header received. Allocate buffer and get
				 * payload length. If allocation fails leave
				 * interrupt. On failed allocation state machine
				 * is reset.
				 */
				buf = bt_buf_get_tx(BT_BUF_H4, K_NO_WAIT,
						    &type, sizeof(type));
				if (!buf) {
					LOG_ERR("No available command buffers!");
					state = ST_IDLE;
					return;
				}

				remaining = get_len(hdr_buf, type);

				net_buf_add_mem(buf, hdr_buf, hdr_len(type));
				if (remaining > net_buf_tailroom(buf)) {
					LOG_ERR("Not enough space in buffer");
					net_buf_unref(buf);
					state = ST_DISCARD;
				} else {
					state = ST_PAYLOAD;
				}

			}
			break;
		case ST_PAYLOAD:
			read = h4_read(hci_uart_dev, net_buf_tail(buf),
				       remaining);
			buf->len += read;
			remaining -= read;
			if (remaining == 0) {
				/* Packet received */
				LOG_DBG("putting RX packet in queue.");
				net_buf_put(&tx_queue, buf);
				state = ST_IDLE;
			}
			break;
		case ST_DISCARD:
		{
			uint8_t discard[H4_DISCARD_LEN];
			size_t to_read = MIN(remaining, sizeof(discard));

			read = h4_read(hci_uart_dev, discard, to_read);
			remaining -= read;
			if (remaining == 0) {
				state = ST_IDLE;
			}

			break;

		}
		default:
			read = 0;
			__ASSERT_NO_MSG(0);
			break;

		}
	} while (read);
}

static atomic_t in_tx_isr;
static volatile uint32_t cnt = 0;

#define MWU_WATCH() \
	do { \
		unsigned int key = irq_lock(); \
		__ASSERT_NO_MSG(cnt < 0xffff); \
		if (!cnt) { \
			nrf_mwu_region_watch_enable(NRF_MWU, NRF_MWU_WATCH_REGION0_WRITE); \
			__DMB(); \
			__ISB(); \
			__DSB(); \
		} \
		cnt++; \
		irq_unlock(key); \
	} while (false)

#define MWU_UNWATCH() \
	do { \
		unsigned int key = irq_lock(); \
		cnt--; \
		if (!cnt) { \
			nrf_mwu_region_watch_disable(NRF_MWU, NRF_MWU_WATCH_REGION0_WRITE); \
			__DMB(); \
			__ISB(); \
			__DSB(); \
		} \
		irq_unlock(key); \
	} while (false)

static void tx_isr(void)
{
	static struct net_buf *buf;
	static volatile bool once = true;
	int len;

	atomic_inc(&in_tx_isr);

	pin_toggle(1, 4, 1);

	if (once) {
		nrf_mwu_user_region_range_set(NRF_MWU, 0, (uint32_t)&buf, (uint32_t)&buf);
		once = false;
	}

	if (!buf) {
		buf = net_buf_get(&uart_tx_queue, K_NO_WAIT);
		if (!buf) {
			uart_irq_tx_disable(hci_uart_dev);

			pin_toggle(1, 4, 3);

			atomic_dec(&in_tx_isr);
			return;
		}
	}

	MWU_WATCH();

	__ASSERT_NO_MSG(atomic_get(&in_tx_isr) == 1);
	len = uart_fifo_fill(hci_uart_dev, buf->data, buf->len);
	__ASSERT_NO_MSG(atomic_get(&in_tx_isr) == 1);
	net_buf_pull(buf, len);
	__ASSERT_NO_MSG(atomic_get(&in_tx_isr) == 1);
	if (!buf->len) {
		net_buf_unref(buf);

		MWU_UNWATCH();

		buf = NULL;
	} else {
		MWU_UNWATCH();
	}

	__ASSERT_NO_MSG(atomic_get(&in_tx_isr) == 1);
	atomic_dec(&in_tx_isr);
	pin_toggle(1, 4, 2);
}

static void bt_uart_isr(const struct device *unused, void *user_data)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);

	if (!(uart_irq_rx_ready(hci_uart_dev) ||
	      uart_irq_tx_ready(hci_uart_dev))) {
		LOG_DBG("spurious interrupt");
	}

	if (uart_irq_tx_ready(hci_uart_dev)) {
		tx_isr();
	}

	if (uart_irq_rx_ready(hci_uart_dev)) {
		rx_isr();
	}
}

static void tx_thread(void *p1, void *p2, void *p3)
{
	while (1) {
		struct net_buf *buf;
		int err;

		/* Wait until a buffer is available */
		buf = net_buf_get(&tx_queue, K_FOREVER);
		/* Pass buffer to the stack */
		err = bt_send(buf);
		if (err) {
			LOG_ERR("Unable to send (err %d)", err);
			net_buf_unref(buf);
		}

		/* Give other threads a chance to run if tx_queue keeps getting
		 * new data all the time.
		 */
		k_yield();
	}
}

static int h4_send(struct net_buf *buf)
{
	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf),
		    buf->len);

	net_buf_put(&uart_tx_queue, buf);
	uart_irq_tx_enable(hci_uart_dev);

	return 0;
}

#if defined(CONFIG_BT_CTLR_ASSERT_HANDLER)
void bt_ctlr_assert_handle(char *file, uint32_t line)
{
	uint32_t len = 0U, pos = 0U;

	/* Disable interrupts, this is unrecoverable */
	(void)irq_lock();

	uart_irq_rx_disable(hci_uart_dev);
	uart_irq_tx_disable(hci_uart_dev);

	if (file) {
		while (file[len] != '\0') {
			if (file[len] == '/') {
				pos = len + 1;
			}
			len++;
		}
		file += pos;
		len -= pos;
	}

	uart_poll_out(hci_uart_dev, H4_EVT);
	/* Vendor-Specific debug event */
	uart_poll_out(hci_uart_dev, 0xff);
	/* 0xAA + strlen + \0 + 32-bit line number */
	uart_poll_out(hci_uart_dev, 1 + len + 1 + 4);
	uart_poll_out(hci_uart_dev, 0xAA);

	if (len) {
		while (*file != '\0') {
			uart_poll_out(hci_uart_dev, *file);
			file++;
		}
		uart_poll_out(hci_uart_dev, 0x00);
	}

	uart_poll_out(hci_uart_dev, line >> 0 & 0xff);
	uart_poll_out(hci_uart_dev, line >> 8 & 0xff);
	uart_poll_out(hci_uart_dev, line >> 16 & 0xff);
	uart_poll_out(hci_uart_dev, line >> 24 & 0xff);

	while (1) {
	}
}
#endif /* CONFIG_BT_CTLR_ASSERT_HANDLER */

static int hci_uart_init(void)
{
	LOG_DBG("");

	if (IS_ENABLED(CONFIG_USB_CDC_ACM)) {
		if (usb_enable(NULL)) {
			LOG_ERR("Failed to enable USB");
			return -EINVAL;
		}
	}

	if (!device_is_ready(hci_uart_dev)) {
		LOG_ERR("HCI UART %s is not ready", hci_uart_dev->name);
		return -EINVAL;
	}

	uart_irq_rx_disable(hci_uart_dev);
	uart_irq_tx_disable(hci_uart_dev);

	uart_irq_callback_set(hci_uart_dev, bt_uart_isr);

	uart_irq_rx_enable(hci_uart_dev);

	return 0;
}

SYS_INIT(hci_uart_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

int main(void)
{
	/* incoming events and data from the controller */
	static K_FIFO_DEFINE(rx_queue);
	int err;

	pin_cfg(1, 4);
	pin_cfg(1, 2);
	pin_cfg(1,11);
	pin_cfg(1,13);
	pin_cfg(1,15);

	pin_set(1, 4);
	pin_set(1, 2);
	pin_set(1,11);
	pin_set(1,13);
	pin_set(1,15);

	pin_clr(1, 4);
	pin_clr(1, 2);
	pin_clr(1,11);
	pin_clr(1,13);
	pin_clr(1,15);

	nrf_mwu_nmi_enable(NRF_MWU, NRF_MWU_INT_REGION0_WRITE_MASK);

	LOG_DBG("Start");
	__ASSERT(hci_uart_dev, "UART device is NULL");

	/* Enable the raw interface, this will in turn open the HCI driver */
	bt_enable_raw(&rx_queue);

	if (IS_ENABLED(CONFIG_BT_WAIT_NOP)) {
		/* Issue a Command Complete with NOP */
		int i;

		const struct {
			const uint8_t h4;
			const struct bt_hci_evt_hdr hdr;
			const struct bt_hci_evt_cmd_complete cc;
		} __packed cc_evt = {
			.h4 = H4_EVT,
			.hdr = {
				.evt = BT_HCI_EVT_CMD_COMPLETE,
				.len = sizeof(struct bt_hci_evt_cmd_complete),
			},
			.cc = {
				.ncmd = 1,
				.opcode = sys_cpu_to_le16(BT_OP_NOP),
			},
		};

		for (i = 0; i < sizeof(cc_evt); i++) {
			uart_poll_out(hci_uart_dev,
				      *(((const uint8_t *)&cc_evt)+i));
		}
	}

	/* Spawn the TX thread and start feeding commands and data to the
	 * controller
	 */
	k_thread_create(&tx_thread_data, tx_thread_stack,
			K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "HCI uart TX");

	while (1) {
		struct net_buf *buf;

		buf = net_buf_get(&rx_queue, K_FOREVER);
		err = h4_send(buf);
		if (err) {
			LOG_ERR("Failed to send");
		}
	}
	return 0;
}
