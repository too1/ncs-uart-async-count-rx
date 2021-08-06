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

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

static const struct device *dev_uart;

static uint8_t uart_buf[1024];

void uart_cb(const struct device *x, void *user_data)
{
	uart_irq_update(x);
	int data_length = 0;

	if (uart_irq_rx_ready(x)) {
		data_length = uart_fifo_read(x, uart_buf, sizeof(uart_buf));
		uart_buf[data_length] = 0;
	}
	printk("%s", uart_buf);
}

static void uart_init(void)
{
	dev_uart = device_get_binding("UART_0");
	if (dev_uart == NULL) {
		return;
	}

	uart_irq_callback_set(dev_uart, uart_cb);
	uart_irq_rx_enable(dev_uart);
}

void main(void)
{
	const struct device *dev;
	bool led_is_on = true;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	uart_init();

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
	}
}
