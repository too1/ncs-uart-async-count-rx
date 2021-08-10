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
#ifdef DPPI_PRESENT
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif
#include <helpers/nrfx_gppi.h>
#include <string.h>

#define UART_BUF_SIZE		16
#define UART_RX_TIMEOUT_MS	1000

#define COUNT_RX_TIMER		NRF_TIMER1

K_SEM_DEFINE(tx_done, 0, 1);
K_SEM_DEFINE(rx_rdy, 0, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

static const struct device *dev_uart;

uint8_t uart_double_buffer[2][UART_BUF_SIZE];
uint8_t *uart_buf_next = uart_double_buffer[1];
uint8_t *read_ptr;
uint32_t read_len;

#ifdef DPPI_PRESENT
typedef uint8_t ppi_channel_t;
#else
typedef nrf_ppi_channel_t ppi_channel_t;
#endif

static ppi_channel_t ppi_ch_count_rx_received;

static void timer_init(void)
{
	COUNT_RX_TIMER->MODE = TIMER_MODE_MODE_Counter << TIMER_MODE_MODE_Pos;
	COUNT_RX_TIMER->TASKS_START = 1;
}

static uint32_t timer_get_count_and_reset(void)
{
	COUNT_RX_TIMER->TASKS_CAPTURE[0] = 1;
	COUNT_RX_TIMER->TASKS_CLEAR = 1;
	return COUNT_RX_TIMER->CC[0];
}

static void ppi_init(void)
{
	// Assign a PPI channel to increment the timer each time a byte is received over the UART
	if(nrfx_ppi_channel_alloc(&ppi_ch_count_rx_received) == NRFX_ERROR_NO_MEM);
	nrfx_ppi_channel_assign(ppi_ch_count_rx_received, 
							(uint32_t)&NRF_UARTE0->EVENTS_RXDRDY,
							(uint32_t)&COUNT_RX_TIMER->TASKS_COUNT);
	nrfx_ppi_channel_enable(ppi_ch_count_rx_received);
}

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
	
	timer_init();

	ppi_init();

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
		printk("RX bytes counted: %i\n", timer_get_count_and_reset());
		k_sleep(K_MSEC(500));

	}
}
