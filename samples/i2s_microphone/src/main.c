#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2s_demo, LOG_LEVEL_INF);

#define I2S_DEV_NODE DT_LABEL(DT_NODELABEL(i2s))

#define I2S_BLOCK_SIZE 128  // bytes, for 32 samples * 4 bytes
#define I2S_TIMEOUT_MS 1000

void main(void)
{
	const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s));
	struct i2s_config i2s_cfg = {
		.word_size = 32,
		.channels = 1,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_FRAME_CLK_SLAVE | I2S_OPT_BIT_CLK_SLAVE,
		.frame_clk_freq = 48000,
		.block_size = I2S_BLOCK_SIZE,
		.timeout = I2S_TIMEOUT_MS,
		.mem_slab = NULL,
	};

	LOG_INF("I2S starting...");

	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device not ready");
		return;
	}

	if (i2s_configure(i2s_dev, I2S_DIR_RX, &i2s_cfg) < 0) {
		LOG_ERR("I2S config failed");
		return;
	}

	if (i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START) < 0) {
		LOG_ERR("I2S start failed");
		return;
	}

	LOG_INF("I2S started, waiting for mic data...");

	while (1) {
		void *rx_data;
		size_t rx_size;

		int ret = i2s_read(i2s_dev, &rx_data, &rx_size);
		if (ret == 0) {
			LOG_INF("Received I2S data block");
			// Optionally print or store raw mic data
		} else {
			LOG_WRN("No data available");
		}

		k_sleep(K_MSEC(100));
	}
}
