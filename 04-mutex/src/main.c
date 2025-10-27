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
#define PRIORITY_LOW     14
#define PRIORITY_HIGH    5

/* Thread function prototypes */
void th_higher_led(void *, void *, void *);
void th_lower_led(void *, void *, void *);

K_THREAD_DEFINE(th_high, STACK_SIZE, th_higher_led, NULL, NULL, NULL,
		    PRIORITY_HIGH, 0, 0);

K_THREAD_DEFINE(th_low, STACK_SIZE, th_lower_led, NULL, NULL, NULL,
		    PRIORITY_LOW, 0, 0);

K_SEM_DEFINE(my_sem, 0, 1);

K_MUTEX_DEFINE(my_mutex);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)


/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);



int main(void)
{
	int ret;

	SEGGER_SYSVIEW_SendSysDesc("I#37=RTC1_IRQ");

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}


	return 0;
}

void th_higher_led(void *arg1, void *arg2, void *arg3)
{
	int ret;
	bool led_state = true;


	while (1) {
		k_sem_take(&my_sem, K_FOREVER);
		k_mutex_lock(&my_mutex, K_FOREVER);
		ret = gpio_pin_toggle_dt(&led);
		k_mutex_unlock(&my_mutex);
		if (ret < 0) {
			return;
		}

		led_state = !led_state;
		printf("HIGHER LED state: %s\n", led_state ? "ON" : "OFF");

	}
}

void th_lower_led(void *arg1, void *arg2, void *arg3)
{
	int ret;
	bool led_state = true;
	while (1) {
		k_sem_give(&my_sem);
		k_mutex_lock(&my_mutex, K_FOREVER);
		ret = gpio_pin_toggle_dt(&led);
		k_mutex_unlock(&my_mutex);
		if (ret < 0) {
			return;
		}

		led_state = !led_state;
		printf("LOWER LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
}