/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <string.h>

#define UART_BUF_SIZE		16
#define UART_RX_TIMEOUT_MS	100

K_SEM_DEFINE(tx_done, 0, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

#define UART_RX_MSG_QUEUE_SIZE	8
struct uart_rx_msg_queue {
	uint8_t bytes[UART_BUF_SIZE];
	uint32_t length;
};

char __aligned(4) uart_rx_msgq_buffer[UART_RX_MSG_QUEUE_SIZE * sizeof(struct uart_rx_msg_queue)];
struct k_msgq uart_rx_msgq;

static const struct device *dev_uart;

uint8_t uart_double_buffer[2][UART_BUF_SIZE];
uint8_t *uart_buf_next = uart_double_buffer[1];

void uart_async_callback(const struct device *uart_dev,
				struct uart_event *evt, void *user_data)
{
	static struct uart_rx_msg_queue new_message;

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

void main(void)
{
	printk("UART Async example started\n");
	
	uart_init();

	k_msgq_init(&uart_rx_msgq, uart_rx_msgq_buffer, sizeof(struct uart_rx_msg_queue), UART_RX_MSG_QUEUE_SIZE);

	struct uart_rx_msg_queue incoming_message;

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
