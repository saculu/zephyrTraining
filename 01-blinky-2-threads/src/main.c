/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>


/* Thread stack size & priority */
#define STACK_SIZE   4096
#define PRIORITY     5

/* Thread function prototypes */
void th_second_led(void *, void *, void *);

K_THREAD_DEFINE(th_led, STACK_SIZE, th_second_led, NULL, NULL, NULL,
		    PRIORITY, 0, 0);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);


int main(void)
{
	int ret;
	bool led_state = true;

	SEGGER_SYSVIEW_SendSysDesc("I#15=Systick_IRQ");

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

void th_second_led(void *arg1, void *arg2, void *arg3)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led1)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led1);
		if (ret < 0) {
			return;
		}

		led_state = !led_state;
		printf("LED1 state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS/2);
	}
}