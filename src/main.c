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

#define UART_TX_BUF_SIZE  		256
#define UART_RX_MSG_QUEUE_SIZE	8
struct uart_msg_queue_item {
	uint8_t bytes[UART_BUF_SIZE];
	uint32_t length;
};

// UART TX fifo
RING_BUF_DECLARE(app_tx_fifo, UART_TX_BUF_SIZE);
volatile int bytes_claimed;

// UART RX primary buffers
uint8_t uart_double_buffer[2][UART_BUF_SIZE];
uint8_t *uart_buf_next = uart_double_buffer[1];

// UART RX message queue
K_MSGQ_DEFINE(uart_rx_msgq, sizeof(struct uart_msg_queue_item), UART_RX_MSG_QUEUE_SIZE, 4);

static const struct device *dev_uart;

static int uart_tx_get_from_queue(void)
{
	uint8_t *data_ptr;
	// Try to claim any available bytes in the FIFO
	bytes_claimed = ring_buf_get_claim(&app_tx_fifo, &data_ptr, UART_TX_BUF_SIZE);

	if(bytes_claimed > 0) {
		// Start a UART transmission based on the number of available bytes
		uart_tx(dev_uart, data_ptr, bytes_claimed, SYS_FOREVER_MS);
	}
	return bytes_claimed;
}

void app_uart_async_callback(const struct device *uart_dev,
							 struct uart_event *evt, void *user_data)
{
	static struct uart_msg_queue_item new_message;

	switch (evt->type) {
		case UART_TX_DONE:
			// Free up the written bytes in the TX FIFO
			ring_buf_get_finish(&app_tx_fifo, bytes_claimed);

			// If there is more data in the TX fifo, start the transmission
			if(uart_tx_get_from_queue() == 0) {
				// Or release the semaphore if the TX fifo is empty
				k_sem_give(&tx_done);
			}
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

static void app_uart_init(void)
{
	dev_uart = device_get_binding("UART_1");
	if (dev_uart == NULL) {
		printk("Failed to get UART binding\n");
		return;
	}

	uart_callback_set(dev_uart, app_uart_async_callback, NULL);
	uart_rx_enable(dev_uart, uart_double_buffer[0], UART_BUF_SIZE, UART_RX_TIMEOUT_MS);
}

// Function to send UART data, by writing it to a ring buffer (FIFO) in the application
// WARNING: This function is not thread safe! If you want to call this function from multiple threads a semaphore should be used
static int app_uart_send(const uint8_t * data_ptr, uint32_t data_len)
{
	while(1) {
		// Try to move the data into the TX ring buffer
		uint32_t written_to_buf = ring_buf_put(&app_tx_fifo, data_ptr, data_len);
		data_len -= written_to_buf;
		
		// In case the UART TX is idle, start transmission
		if(k_sem_take(&tx_done, K_NO_WAIT) == 0) {
			uart_tx_get_from_queue();
		}	
		
		// In case all the data was written, exit the loop
		if(data_len == 0) break;

		// In case some data is still to be written, sleep for some time and run the loop one more time
		k_msleep(10);
		data_ptr += written_to_buf;
	}

	return 0;
}

void main(void)
{
	printk("UART Async example started\n");
	
	app_uart_init();

	const uint8_t test_string[] = "Hello world through the UART async driver\r\n";
	app_uart_send(test_string, strlen(test_string));

	struct uart_msg_queue_item incoming_message;

	int counter = 0;

	while (1) {
		// This function will not return until a new message is ready
		if (k_msgq_get(&uart_rx_msgq, &incoming_message, K_MSEC(50)) == 0) {
			// Process the message here.
			static uint8_t string_buffer[UART_BUF_SIZE + 1];
			memcpy(string_buffer, incoming_message.bytes, incoming_message.length);
			string_buffer[incoming_message.length] = 0;
			printk("RX %i: %s\n", incoming_message.length, string_buffer);
		}
		if(counter++ > 40) {
			counter = 0;
			app_uart_send("Test", 4);
		}
	}
}
