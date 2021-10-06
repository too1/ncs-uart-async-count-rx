/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <sys/ring_buffer.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <string.h>

#define UART_BUF_SIZE		16
#define UART_TX_TIMEOUT_MS	100
#define UART_RX_TIMEOUT_MS	100

K_SEM_DEFINE(tx_done, 1, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

#define UART_RX_MSG_QUEUE_SIZE	8
struct uart_msg_queue_item {
	uint8_t bytes[UART_BUF_SIZE];
	uint32_t length;
};

// UART RX primary buffers
uint8_t uart_double_buffer[2][UART_BUF_SIZE];
uint8_t *uart_buf_next = uart_double_buffer[1];

// UART RX message queue
K_MSGQ_DEFINE(uart_rx_msgq, sizeof(struct uart_msg_queue_item), UART_RX_MSG_QUEUE_SIZE, 4);

static const struct device *dev_uart;

void uart_async_callback(const struct device *uart_dev,
				struct uart_event *evt, void *user_data)
{
	static struct uart_msg_queue_item new_message;

	switch (evt->type) {
		case UART_TX_DONE:
			k_sem_give(&tx_done);
			break;
		
		case UART_RX_RDY:
			memcpy(new_message.bytes, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
			new_message.length = evt->data.rx.len;
			if(k_msgq_put(&uart_rx_msgq, &new_message, K_NO_WAIT) != 0){
				printk("Error: Uart RX message queue full!\n");
			}
			break;
		
		case UART_RX_BUF_REQUEST:
			uart_rx_buf_rsp(dev_uart, uart_buf_next, UART_BUF_SIZE);
			break;

		case UART_RX_BUF_RELEASED:
			uart_buf_next = evt->data.rx_buf.buf;
			break;

		case UART_RX_DISABLED:
			k_sem_give(&rx_disabled);
			break;
		
		default:
			break;
	}
}

static void uart_init(void)
{
	dev_uart = device_get_binding("UART_0");
	if (dev_uart == NULL) {
		printk("Failed to get UART binding\n");
		return;
	}

	uart_callback_set(dev_uart, uart_async_callback, NULL);
	uart_rx_enable(dev_uart, uart_double_buffer[0], UART_BUF_SIZE, UART_RX_TIMEOUT_MS);
}

static int uart_send(const uint8_t * data_ptr, uint32_t data_len)
{
	int err = k_sem_take(&tx_done, K_MSEC(UART_TX_TIMEOUT_MS));
	if(err != 0) return err;
	return uart_tx(dev_uart, data_ptr, data_len, UART_TX_TIMEOUT_MS);
}

void main(void)
{
	printk("UART Async example started\n");
	
	uart_init();

	uint8_t test_string[] = "Hello world through the UART async driver\r\n";
	uart_send(test_string, strlen(test_string));

	struct uart_msg_queue_item incoming_message;

	while (1) {
		// This function will not return until a new message is ready
		k_msgq_get(&uart_rx_msgq, &incoming_message, K_FOREVER);

		// Process the message here.
		static uint8_t string_buffer[UART_BUF_SIZE + 1];
		memcpy(string_buffer, incoming_message.bytes, incoming_message.length);
		string_buffer[incoming_message.length] = 0;
		printk("RX %i: %s\n", incoming_message.length, string_buffer);
	}
}
