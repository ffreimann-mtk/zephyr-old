/*
 * Copyright (c) 2025 MediaTek
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>

#include "uart_mtk_common.h"


#define UART_OFFSET_RBR            0x0000   /* (RO) RX buffer */
#define UART_OFFSET_THR            0x0000   /* (WO) TX holding buffer */
#define UART_OFFSET_IER            0x0004   /* (RW) Interrupt enable */
#define UART_OFFSET_IIR            0x0008   /* (RO) Interrupt identification */
#define UART_OFFSET_FCR            0x0008   /* (WO) FIFO control */
#define UART_OFFSET_LCR            0x000c   /* (RW) Line control */
#define UART_OFFSET_MCR            0x0010   /* (RW) Modem control */
#define UART_OFFSET_LSR            0x0014   /* (RO) Line status */
#define UART_OFFSET_MSR            0x0018   /* (RW) Modem status */
#define UART_OFFSET_SCR            0x001c   /* (RW) Scratch */
#define UART_OFFSET_AUTOBAUD_EN    0x0020   /* (RW) Autobaud detect enable */
#define UART_OFFSET_HIGH_SPEED     0x0024   /* (RW) Highspeed */
#define UART_OFFSET_SAMPLE_COUNT   0x0028   /* (RW) Sample count */
#define UART_OFFSET_SAMPLE_POINT   0x002c   /* (RW) Sample point */
#define UART_OFFSET_AUTOBAUD       0x0030   /* (RO) Autobaud monitor */
#define UART_OFFSET_DLL            0x0090   /* (RW) Divisor latch (LSB) */
#define UART_OFFSET_DLM            0x0094   /* (RW) Divisor latch (MSB) */
#define UART_OFFSET_EFR            0x0098   /* (RW) Enhanced feature */
#define UART_OFFSET_XON_1          0x00a0   /* (RW) XON_1 */
#define UART_OFFSET_XON_2          0x00a4   /* (RW) XON_2 */
#define UART_OFFSET_XOFF_1         0x00a8   /* (RW) XOFF_1 */
#define UART_OFFSET_XOFF_2         0x00ac   /* (RW) XOFF_2 */

#define UART_IER_TX_INTERRUPT	(0x02)
#define UART_IER_RX_INTERRUPT	(0x01)

#define UART_IIR_RX_INTERRUPT	(0x04)
#define UART_IIR_TX_INTERRUPT	(0x02)

#define UART_FCR_RX_FIFO_1      (0x00)
#define UART_FCR_RX_FIFO_6      (0x40)
#define UART_FCR_RX_FIFO_12     (0x80)
#define UART_FCR_RX_FIFO_TRIG   (0xc0)
#define UART_FCR_TX_FIFO_1      (0x00)
#define UART_FCR_TX_FIFO_4      (0x10)
#define UART_FCR_TX_FIFO_8      (0x20)
#define UART_FCR_TX_FIFO_14     (0x30)
#define UART_FCR_CLR_TX_FIFO    (0x04)
#define UART_FCR_CLR_RX_FIFO    (0x02)
#define UART_FCR_FIFO_EN        (0x01)

#define UART_LSR_TX_IDLE        BIT (6)
#define UART_LSR_TX_EMPTY       BIT (5)
#define UART_LSR_RX_OVERRUN     BIT (1)
#define UART_LSR_RX_READY       BIT (0)


static bool is_rx_avail(mem_addr_t base);
static bool is_tx_avail(mem_addr_t base);


static bool is_rx_avail(mem_addr_t base)
{
	return ((sys_read32(base + UART_OFFSET_LSR) & UART_LSR_RX_READY) != 0);
}

static bool is_tx_avail(mem_addr_t base)
{
	return ((sys_read32(base + UART_OFFSET_LSR) & UART_LSR_TX_EMPTY) != 0);
}

void uart_mtk_poll_out(const struct device *dev, unsigned char c)
{
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	/* Wait until there is space in the FIFO */
	while (!(is_tx_avail(DEVICE_MMIO_GET(dev)))) {
		/* Ensure there is always some time for other threads. */
		k_spin_unlock(&(uart_data->lock), key);
		key = k_spin_lock(&(uart_data->lock));
	}

	/* Send the character */
	sys_write32((uint32_t)c, (DEVICE_MMIO_GET(dev) + UART_OFFSET_THR));

	k_spin_unlock(&(uart_data->lock), key);
}

int uart_mtk_poll_in(const struct device *dev, unsigned char *c)
{
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	if (!(is_rx_avail(DEVICE_MMIO_GET(dev)))) {
		k_spin_unlock(&(uart_data->lock), key);

		return (-1);
	}

	(*c) = (unsigned char)(sys_read32(DEVICE_MMIO_GET(dev) + UART_OFFSET_RBR) & 0x000000ff);

	k_spin_unlock(&(uart_data->lock), key);

	return (0);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
int uart_mtk_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	int idx = 0;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	while ((idx < size) && (is_tx_avail(DEVICE_MMIO_GET(dev)))) {
		/* Send the character */
		sys_write32((uint32_t)(tx_data[idx]), (DEVICE_MMIO_GET(dev) + UART_OFFSET_THR));

		idx++;
	}

	k_spin_unlock(&(uart_data->lock), key);

	return (idx);
}

int uart_mtk_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	int idx = 0;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	while ((idx < size) && (is_rx_avail(DEVICE_MMIO_GET(dev)))) {
		rx_data[idx] =
			(uint8_t)(sys_read32(DEVICE_MMIO_GET(dev) + UART_OFFSET_RBR) & 0x000000ff);

		idx++;
	}

	k_spin_unlock(&(uart_data->lock), key);

	return (idx);
}

void uart_mtk_irq_tx_enable(const struct device *dev)
{
	uint8_t ier;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	ier = (uint8_t)sys_read32(DEVICE_MMIO_GET(dev) + UART_OFFSET_IER);

	if ((ier & UART_IER_TX_INTERRUPT) == 0) {
		sys_write32((uint32_t)(ier | UART_IER_TX_INTERRUPT),
			    (DEVICE_MMIO_GET(dev) + UART_OFFSET_IER));
	}

	k_spin_unlock(&(uart_data->lock), key);
}

void uart_mtk_irq_tx_disable(const struct device *dev)
{
	uint8_t ier;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	ier = (uint8_t)sys_read32(DEVICE_MMIO_GET(dev) + UART_OFFSET_IER);

	if ((ier & UART_IER_TX_INTERRUPT) != 0) {
		sys_write32((uint32_t)(ier & (~(UART_IER_TX_INTERRUPT))),
			    (DEVICE_MMIO_GET(dev) + UART_OFFSET_IER));
	}

	k_spin_unlock(&(uart_data->lock), key);
}

int uart_mtk_irq_tx_ready(const struct device *dev)
{
	int tx_is_ready;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	tx_is_ready = (is_tx_avail(DEVICE_MMIO_GET(dev)) ? 1 : 0);

	k_spin_unlock(&(uart_data->lock), key);

	return (tx_is_ready);
}

void uart_mtk_irq_rx_enable(const struct device *dev)
{
	uint8_t ier;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	ier = (uint8_t)sys_read32(DEVICE_MMIO_GET(dev) + UART_OFFSET_IER);

	if ((ier & UART_IER_RX_INTERRUPT) == 0) {
		sys_write32((uint32_t)(ier | UART_IER_RX_INTERRUPT),
			    (DEVICE_MMIO_GET(dev) + UART_OFFSET_IER));
	}

	k_spin_unlock(&(uart_data->lock), key);
}

void uart_mtk_irq_rx_disable(const struct device *dev)
{
	uint8_t ier;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	ier = (uint8_t)sys_read32(DEVICE_MMIO_GET(dev) + UART_OFFSET_IER);

	if ((ier & UART_IER_RX_INTERRUPT) != 0) {
		sys_write32((uint32_t)(ier & (~(UART_IER_RX_INTERRUPT))),
			    (DEVICE_MMIO_GET(dev) + UART_OFFSET_IER));
	}

	k_spin_unlock(&(uart_data->lock), key);
}

int uart_mtk_irq_rx_ready(const struct device *dev)
{
	int rx_is_ready;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	rx_is_ready = (is_rx_avail(DEVICE_MMIO_GET(dev)) ? 1 : 0);

	k_spin_unlock(&(uart_data->lock), key);

	return (rx_is_ready);
}

int uart_mtk_irq_is_pending(const struct device *dev)
{
	uint8_t iir;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	iir = (uint8_t)sys_read32(DEVICE_MMIO_GET(dev) + UART_OFFSET_IIR);

	k_spin_unlock(&(uart_data->lock), key);

	return ((((iir & UART_IIR_RX_INTERRUPT) != 0) || ((iir & UART_IIR_TX_INTERRUPT) != 0)) ? 1
											       : 0);
}

int uart_mtk_irq_update(const struct device *dev)
{
	return 1;
}

void uart_mtk_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				      void *cb_data)
{
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	uart_data->callback = cb;
	uart_data->cb_data = cb_data;

	k_spin_unlock(&(uart_data->lock), key);
}

void uart_mtk_isr(const struct device *dev)
{
	uart_mtk_data_t *uart_data = dev->data;

	if (uart_data->callback != NULL) {
		uart_data->callback(dev, uart_data->cb_data);
	}
}
#endif

int uart_mtk_init(const struct device *dev)
{
	int ret;
	const uart_mtk_config_t *uart_config = dev->config;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	sys_write32((uint32_t)(UART_FCR_RX_FIFO_1 | UART_FCR_TX_FIFO_4 | UART_FCR_CLR_TX_FIFO |
			       UART_FCR_CLR_RX_FIFO | UART_FCR_FIFO_EN),
		    (DEVICE_MMIO_GET(dev) + UART_OFFSET_FCR));

#ifdef CONFIG_PINCTRL
	ret = pinctrl_apply_state(uart_config->pinctrl_config, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return (ret);
	}
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_config->irq_config_func(dev);
#endif
	return (0);
}
