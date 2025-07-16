/*
 * Copyright (c) 2025 MediaTek
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mediatek_mt8370_uart


#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>


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

typedef struct {
	DEVICE_MMIO_ROM; /* Must be first */
	uint32_t baud_rate;
	uint32_t clocks;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pinctrl_config;
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif
} uart_mtk_config_t;

typedef struct {
	DEVICE_MMIO_RAM; /* Must be first */
	struct k_spinlock lock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
} uart_mtk_data_t;

static bool is_rx_avail(mem_addr_t base);
static bool is_tx_avail(mem_addr_t base);

static void uart_mtk_poll_out(const struct device *dev, unsigned char c);
static int uart_mtk_poll_in(const struct device *dev, unsigned char *c);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_mtk_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size);
static int uart_mtk_fifo_read(const struct device *dev, uint8_t *rx_data, const int size);
static void uart_mtk_irq_tx_enable(const struct device *dev);
static void uart_mtk_irq_tx_disable(const struct device *dev);
static int uart_mtk_irq_tx_ready(const struct device *dev);
static void uart_mtk_irq_rx_enable(const struct device *dev);
static void uart_mtk_irq_rx_disable(const struct device *dev);
static int uart_mtk_irq_rx_ready(const struct device *dev);
static int uart_mtk_irq_is_pending(const struct device *dev);
static int uart_mtk_irq_update(const struct device *dev);
static void uart_mtk_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				      void *cb_data);
static void uart_mtk_isr(const struct device *dev);
#endif

static int uart_mtk_init(const struct device *dev);

static bool is_rx_avail(mem_addr_t base)
{
	return ((sys_read32(base + UART_OFFSET_LSR) & UART_LSR_RX_READY) != 0);
}

static bool is_tx_avail(mem_addr_t base)
{
	return ((sys_read32(base + UART_OFFSET_LSR) & UART_LSR_TX_EMPTY) != 0);
}

static void uart_mtk_poll_out(const struct device *dev, unsigned char c)
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

static int uart_mtk_poll_in(const struct device *dev, unsigned char *c)
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
static int uart_mtk_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
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

static int uart_mtk_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
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

static void uart_mtk_irq_tx_enable(const struct device *dev)
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

static void uart_mtk_irq_tx_disable(const struct device *dev)
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

static int uart_mtk_irq_tx_ready(const struct device *dev)
{
	int tx_is_ready;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	tx_is_ready = (is_tx_avail(DEVICE_MMIO_GET(dev)) ? 1 : 0);

	k_spin_unlock(&(uart_data->lock), key);

	return (tx_is_ready);
}

static void uart_mtk_irq_rx_enable(const struct device *dev)
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

static void uart_mtk_irq_rx_disable(const struct device *dev)
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

static int uart_mtk_irq_rx_ready(const struct device *dev)
{
	int rx_is_ready;
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	rx_is_ready = (is_rx_avail(DEVICE_MMIO_GET(dev)) ? 1 : 0);

	k_spin_unlock(&(uart_data->lock), key);

	return (rx_is_ready);
}

static int uart_mtk_irq_is_pending(const struct device *dev)
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

static int uart_mtk_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_mtk_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				      void *cb_data)
{
	uart_mtk_data_t *uart_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&(uart_data->lock));

	uart_data->callback = cb;
	uart_data->cb_data = cb_data;

	k_spin_unlock(&(uart_data->lock), key);
}

static void uart_mtk_isr(const struct device *dev)
{
	uart_mtk_data_t *uart_data = dev->data;

	if (uart_data->callback != NULL) {
		uart_data->callback(dev, uart_data->cb_data);
	}
}
#endif

static int uart_mtk_init(const struct device *dev)
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

static DEVICE_API(uart, uart_mtk_driver_api) = {
	.poll_in = uart_mtk_poll_in,
	.poll_out = uart_mtk_poll_out,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_mtk_fifo_fill,
	.fifo_read = uart_mtk_fifo_read,
	.irq_tx_enable = uart_mtk_irq_tx_enable,
	.irq_tx_disable = uart_mtk_irq_tx_disable,
	.irq_tx_ready = uart_mtk_irq_tx_ready,
	.irq_rx_enable = uart_mtk_irq_rx_enable,
	.irq_rx_disable = uart_mtk_irq_rx_disable,
	.irq_rx_ready = uart_mtk_irq_rx_ready,
	.irq_is_pending = uart_mtk_irq_is_pending,
	.irq_update = uart_mtk_irq_update,
	.irq_callback_set = uart_mtk_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define UART_DECLARE_CFG(n, IRQ_FUNC_INIT)                                                   \
	static const uart_mtk_config_t uart_mtk_##n##_config = {                                 \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                                \
		.baud_rate = DT_INST_PROP(n, current_speed),                                         \
		.clocks = DT_INST_PROP(n, clock_frequency),                                          \
		IF_ENABLED (CONFIG_PINCTRL, (.pinctrl_config = PINCTRL_DT_INST_DEV_CONFIG_GET (n),)) \
		IRQ_FUNC_INIT                                                                        \
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_IRQ_CONFIG_FUNC(n)                                                         \
	static void irq_config_func_##n(const struct device *dev)                           \
	{                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_mtk_isr,            \
			    DEVICE_DT_INST_GET(n), 0);                                              \
		irq_enable(DT_INST_IRQN(n));                                                    \
	}

#define UART_IRQ_CFG_FUNC_INIT(n) .irq_config_func = irq_config_func_##n,
#define UART_INIT_CFG(n)          UART_DECLARE_CFG(n, UART_IRQ_CFG_FUNC_INIT(n))
#else
#define UART_IRQ_CONFIG_FUNC(n)
#define UART_IRQ_CFG_FUNC_INIT
#define UART_INIT_CFG(n) UART_DECLARE_CFG(n, UART_IRQ_CFG_FUNC_INIT)
#endif

#define UART_INIT(n)                                                                   \
	IF_ENABLED (CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE (n);))                         \
	static uart_mtk_data_t uart_mtk_##n##_data;                                        \
	static const uart_mtk_config_t uart_mtk_##n##_config;                              \
	UART_IRQ_CONFIG_FUNC(n)                                                            \
	DEVICE_DT_INST_DEFINE(n, &uart_mtk_init, NULL, &uart_mtk_##n##_data,               \
			      &uart_mtk_##n##_config, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,   \
			      &uart_mtk_driver_api);                                               \
	UART_INIT_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(UART_INIT)
