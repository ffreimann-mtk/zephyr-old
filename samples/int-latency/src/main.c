/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>


#define UART_RX_DATA_SIZE  256


static const struct device* const  uart_dev = DEVICE_DT_GET (DT_CHOSEN (zephyr_console));

static const struct gpio_dt_spec trigger  = GPIO_DT_SPEC_GET (DT_ALIAS (trigger),  gpios);
static const struct gpio_dt_spec response = GPIO_DT_SPEC_GET (DT_ALIAS (response), gpios);


static volatile uint64_t wait_cnt;

static struct gpio_callback  trigger_cb_data;

static bool  response_is_set = false;

static uint32_t evt_cnt = 0;

static uint32_t rx_Data_wr_idx = 0;
static uint32_t rx_Data_rd_idx = 0;

static uint8_t  uart_rx_data [UART_RX_DATA_SIZE];

static int response_wait_cnt;


static void wait_loop (void)
{
    wait_cnt = 0;

    while (wait_cnt < 50000)
    {
        wait_cnt++;
    }
}

static void uart_fifo_callback (const struct device*  dev,
                                void*                 user_data)
{
    while (uart_irq_rx_ready (dev))
    {
        rx_Data_wr_idx += uart_fifo_read (dev, &(uart_rx_data [rx_Data_wr_idx]), (UART_RX_DATA_SIZE - rx_Data_wr_idx));

        if (rx_Data_wr_idx >= UART_RX_DATA_SIZE)
        {
            rx_Data_wr_idx = 0;
        }
    }
}

void trigger_evt (const struct device*   dev,
                  struct gpio_callback*  cb,
		          uint32_t               pins)
{
    gpio_pin_set_dt (&response, 1);

    response_wait_cnt = 0;
    response_is_set = true;

    evt_cnt++;
}

int main(void)
{
    int ret;
    int print_wait_cnt;


	printf ("Hello from Interrupt Latency ... %s\n", CONFIG_BOARD_TARGET);

    if (! (gpio_is_ready_dt (&trigger)))
    {
        return (0);
    }

	ret = gpio_pin_configure_dt (&trigger, GPIO_INPUT);
	if (ret < 0)
    {
		return 0;
	}

    printf ("  Initialized the trigger GPIO port ....\n");

    ret = gpio_pin_interrupt_configure_dt (&trigger, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0)
    {
        return (0);
    }    

	gpio_init_callback (&trigger_cb_data, trigger_evt, BIT (trigger.pin));
	gpio_add_callback (trigger.port, &trigger_cb_data);

    printf ("  Initialized the trigger interrupt ....\n");

    if (! (gpio_is_ready_dt (&response)))
    {
        return (0);
    }

	ret = gpio_pin_configure_dt (&response, GPIO_OUTPUT_INACTIVE);
	if (ret < 0)
    {
		return 0;
	}

    printf ("  Initialized the response GPIO port ....\n");

	uart_irq_callback_set (uart_dev, uart_fifo_callback);
	uart_irq_rx_enable (uart_dev);

    print_wait_cnt = 0;
    response_wait_cnt = 0;

	while (1)
    {
        print_wait_cnt++;
        if (print_wait_cnt >= 10000)
        {
        	printf("  Interrupt count: %d \\ %d\n", evt_cnt, rx_Data_wr_idx);
            print_wait_cnt = 0;
        }

        if (response_is_set)
        {
            response_wait_cnt++;
            if (response_wait_cnt >= 2)
            {
                gpio_pin_set_dt (&response, 0);

                response_is_set = false;
                response_wait_cnt = 0;
            }
        }
        wait_loop ();
	}

	return 0;
}
