/*
 * Copyright 2025 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_toggle, LOG_LEVEL_INF);

#define MY_SIGNAL_NODE DT_NODELABEL(my_signal)

static const struct gpio_dt_spec my_signal = GPIO_DT_SPEC_GET(MY_SIGNAL_NODE, gpios);

static int cmd_toggle(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ret = gpio_pin_toggle_dt(&my_signal);
	if (ret < 0) {
		shell_error(sh, "Failed to toggle pin: %d", ret);
		return ret;
	}

	shell_print(sh, "Pin PTD31 toggled");
	return 0;
}

static int cmd_on(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ret = gpio_pin_set_dt(&my_signal, 1);
	if (ret < 0) {
		shell_error(sh, "Failed to set pin: %d", ret);
		return ret;
	}

	shell_print(sh, "Pin PTD31 set HIGH");
	return 0;
}

static int cmd_off(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ret = gpio_pin_set_dt(&my_signal, 0);
	if (ret < 0) {
		shell_error(sh, "Failed to clear pin: %d", ret);
		return ret;
	}

	shell_print(sh, "Pin PTD31 set LOW");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(gpio_cmds,
	SHELL_CMD(toggle, NULL, "Toggle PTD31 pin", cmd_toggle),
	SHELL_CMD(on, NULL, "Set PTD31 pin HIGH", cmd_on),
	SHELL_CMD(off, NULL, "Set PTD31 pin LOW", cmd_off),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gpio, &gpio_cmds, "GPIO control commands", NULL);

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&my_signal)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&my_signal, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO: %d", ret);
		return ret;
	}

	LOG_INF("GPIO PTD31 configured. Use shell commands:");
	LOG_INF("  gpio toggle - Toggle the pin");
	LOG_INF("  gpio on     - Set pin HIGH");
	LOG_INF("  gpio off    - Set pin LOW");

	return 0;
}
