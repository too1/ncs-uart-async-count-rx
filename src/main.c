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
K_SEM_DEFINE(rx_rdy, 0, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

static const struct device *dev_uart;

uint8_t uart_double_buffer[2][UART_BUF_SIZE];
uint8_t *uart_buf_next = uart_double_buffer[1];
uint8_t *read_ptr;
uint32_t read_len;

void uart_async_callback(const struct device *uart_dev,
				struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
		case UART_TX_DONE:
			k_sem_give(&tx_done);
			break;
		
		case UART_RX_RDY:
			read_ptr = evt->data.rx.buf + evt->data.rx.offset;
			read_len = evt->data.rx.len;
			k_sem_give(&rx_rdy);
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

	while (1) {
		// When the RX RDY semaphore is given, print out the received UART data
		if(k_sem_take(&rx_rdy, K_MSEC(50)) == 0)
		{
			static uint8_t string_buffer[UART_BUF_SIZE + 1];
			memcpy(string_buffer, read_ptr, read_len);
			string_buffer[read_len] = 0;
			printk("RX %i: %s\n", read_len, string_buffer);
		}
	}
}
