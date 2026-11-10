/*
 * ADXL345 driver that reads the sensor with SPI burst reads and keeps its own
 * FIFO and "interrupt" in software.
 *
 * It talks to the ECW emulated ADXL345 sensor (HIL) using the standard ADXL345
 * protocol: SPI mode 3, MSB first, 2 us CS setup and hold.
 *   0x80  reads DEVID (register 0x00), which should be 0xE5
 *   then  DATA_FORMAT = 0x08 (+-2 g, full resolution), BW_RATE, POWER_CTL = 0x08 (measure)
 *   0xF2  burst-reads registers 0x32..0x37: X, Y, Z as little-endian int16
 *
 * The demo doesn't wire up the INT1 pin and doesn't use the sensor's own FIFO.
 * The driver does both in software, behind the normal Zephyr sensor API, so the
 * application can't tell the difference. Here's how one sample travels:
 *
 *   1. A k_timer fires at the sampling rate and queues kick_work.
 *   2. kick_work starts an async SPI burst (0xF2 + 6 bytes, usually over DMA).
 *   3. When the transfer completes, burst_done() decodes the sample and pushes it
 *      into the FIFO.
 *   4. Once the FIFO holds fifo-watermark samples, we queue trig_work. That's our
 *      equivalent of the INT1 edge on a real part.
 *   5. trig_work calls the application's handler, which pops samples with
 *      sensor_sample_fetch() and reads them with sensor_channel_get().
 *
 * If CONFIG_SPI_ASYNC is off, step 2 uses a blocking transfer instead, so the
 * driver still works with any Zephyr SPI driver.
 */

#define DT_DRV_COMPAT ecw_adxl345_burst

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "adxl345_burst.h"

LOG_MODULE_REGISTER(adxl345_burst, CONFIG_SENSOR_LOG_LEVEL);

#define CMD_READ        0x80   /* R/W=1; bit 7 clear = write */
#define CMD_BURST       0xF2   /* R/W=1, MB=1, reg 0x32 */
#define REG_DEVID       0x00
#define REG_BW_RATE     0x2C
#define REG_POWER_CTL   0x2D
#define REG_DATA_FORMAT 0x31
#define DATA_FORMAT_FULL_RES_2G 0x08
#define POWER_CTL_MEASURE       0x08
#define ADXL345_DEVID   0xE5
#define LSB_PER_G       256    /* full resolution */
#define BURST_LEN       7      /* command byte + 6 data bytes */
#define MAX_ODR_HZ      1000
#define DEVID_ATTEMPTS  20     /* x DEVID_RETRY_MS = up to 2 s at boot */
#define DEVID_RETRY_MS  100

#define FIFO_SIZE       CONFIG_ADXL345_BURST_FIFO_SIZE

#define SPI_OP (SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB | \
		SPI_MODE_CPOL | SPI_MODE_CPHA)

struct adxl345_burst_config {
	struct spi_dt_spec bus;
	uint16_t odr_hz;
	uint8_t watermark;
};

struct adxl345_burst_sample {
	int16_t x, y, z;
};

struct adxl345_burst_data {
	const struct device *dev;

	/* DMA buffers live in RAM and are aligned, since some DMA engines can't read from flash */
	uint8_t tx[8] __aligned(16);
	uint8_t rx[8] __aligned(16);
	struct spi_buf tx_buf, rx_buf;
	struct spi_buf_set tx_set, rx_set;

	struct k_timer timer;
	struct k_work kick_work;
	struct k_work trig_work;
	uint16_t odr_hz;
	uint8_t watermark;
	bool running;
	atomic_t busy;          /* bit 0: a burst is in flight */
	atomic_t trig_pending;  /* bit 0: trig_work submitted, handler not done */

	/* Software FIFO: burst_done() fills it, sample_fetch() empties it */
	struct k_spinlock lock;
	struct adxl345_burst_sample fifo[FIFO_SIZE];
	uint8_t head, count;
	bool fifo_overrun;
	struct adxl345_burst_sample cur;   /* last sample returned by sample_fetch() */

	sensor_trigger_handler_t wm_handler;
	const struct sensor_trigger *wm_trig;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trig;

	/* Counters for the status line, exposed through sensor_attr_get(ADXL345_BURST_ATTR_*) */
	uint32_t n_samples, n_overrun, n_err, n_fifo_drops;
	uint32_t t_start_cyc, xfer_cyc_last, xfer_cyc_max;
};

static K_THREAD_STACK_DEFINE(burst_q_stack, CONFIG_ADXL345_BURST_THREAD_STACK_SIZE);
static struct k_work_q burst_q;

/*
 * Called when a burst finishes. With async SPI this runs in interrupt context,
 * otherwise in the driver's work queue thread.
 */
static void burst_done(const struct device *spi, int result, void *user_data)
{
	struct adxl345_burst_data *data = user_data;
	const uint8_t *b = &data->rx[1];
	bool fire;

	uint32_t xfer = k_cycle_get_32() - data->t_start_cyc;

	ARG_UNUSED(spi);

	if (result != 0) {
		data->n_err++;
		atomic_clear_bit(&data->busy, 0);
		return;
	}

	struct adxl345_burst_sample s = {
		.x = (int16_t)sys_get_le16(&b[0]),
		.y = (int16_t)sys_get_le16(&b[2]),
		.z = (int16_t)sys_get_le16(&b[4]),
	};

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (data->count == FIFO_SIZE) {
		/* FIFO is full: drop the oldest sample, the same way the real ADXL345 does in stream mode */
		data->head = (data->head + 1) % FIFO_SIZE;
		data->count--;
		data->fifo_overrun = true;
		data->n_fifo_drops++;
	}
	data->fifo[(data->head + data->count) % FIFO_SIZE] = s;
	data->count++;
	data->n_samples++;
	data->xfer_cyc_last = xfer;
	data->xfer_cyc_max = MAX(data->xfer_cyc_max, xfer);

	/* DATA_READY fires on every sample, FIFO_WATERMARK only once enough samples are waiting */
	fire = (data->drdy_handler != NULL) ||
	       (data->wm_handler != NULL && data->count >= data->watermark);
	k_spin_unlock(&data->lock, key);

	atomic_clear_bit(&data->busy, 0);

	/* Our software INT1. Only one notification is in flight at a time; the handler drains the FIFO. */
	if (fire && !atomic_test_and_set_bit(&data->trig_pending, 0)) {
		k_work_submit(&data->trig_work);
	}
}

static void kick_work_handler(struct k_work *work)
{
	struct adxl345_burst_data *data = CONTAINER_OF(work, struct adxl345_burst_data, kick_work);
	const struct adxl345_burst_config *cfg = data->dev->config;
	int rc;

	if (atomic_test_and_set_bit(&data->busy, 0)) {
		data->n_overrun++;   /* the previous burst hasn't finished yet, skip this tick */
		return;
	}

	data->t_start_cyc = k_cycle_get_32();
#ifdef CONFIG_SPI_ASYNC
	rc = spi_transceive_cb(cfg->bus.bus, &cfg->bus.config, &data->tx_set, &data->rx_set,
			       burst_done, data);
	if (rc != 0) {
		/* -ENOTSUP usually means this SPI driver can't do async transfers with this config */
		LOG_WRN_ONCE("burst start failed: %d", rc);
		data->n_err++;
		atomic_clear_bit(&data->busy, 0);
	}
#else
	rc = spi_transceive_dt(&cfg->bus, &data->tx_set, &data->rx_set);
	burst_done(cfg->bus.bus, rc, data);
#endif
}

static void timer_expiry(struct k_timer *timer)
{
	struct adxl345_burst_data *data = CONTAINER_OF(timer, struct adxl345_burst_data, timer);

	k_work_submit_to_queue(&burst_q, &data->kick_work);
}

static void trig_work_handler(struct k_work *work)
{
	struct adxl345_burst_data *data = CONTAINER_OF(work, struct adxl345_burst_data, trig_work);

	if (data->wm_handler != NULL) {
		data->wm_handler(data->dev, data->wm_trig);
	}
	if (data->drdy_handler != NULL) {
		data->drdy_handler(data->dev, data->drdy_trig);
	}
	atomic_clear_bit(&data->trig_pending, 0);
}

static void sampling_update(struct adxl345_burst_data *data)
{
	bool want = (data->wm_handler != NULL) || (data->drdy_handler != NULL);

	if (want) {
		k_timeout_t period = K_USEC(USEC_PER_SEC / data->odr_hz);

		k_timer_start(&data->timer, period, period);
	} else {
		k_timer_stop(&data->timer);
	}
	data->running = want;
}

/* Zephyr sensor API */
static int adxl345_burst_trigger_set(const struct device *dev,
				     const struct sensor_trigger *trig,
				     sensor_trigger_handler_t handler)
{
	struct adxl345_burst_data *data = dev->data;
	k_spinlock_key_t key;

	if (trig->chan != SENSOR_CHAN_ACCEL_XYZ && trig->chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);
	switch (trig->type) {
	case SENSOR_TRIG_FIFO_WATERMARK:
		data->wm_handler = handler;
		data->wm_trig = trig;
		break;
	case SENSOR_TRIG_DATA_READY:
		data->drdy_handler = handler;
		data->drdy_trig = trig;
		break;
	default:
		k_spin_unlock(&data->lock, key);
		LOG_ERR("Unsupported trigger %d", trig->type);
		return -ENOTSUP;
	}
	data->head = 0;
	data->count = 0;
	data->fifo_overrun = false;
	k_spin_unlock(&data->lock, key);

	sampling_update(data);
	return 0;
}

static int reg_xfer(const struct device *dev, uint8_t b0, uint8_t b1, uint8_t *out);

/*
 * Pick the lowest BW_RATE code that is still at least odr_hz.
 * Code 0x0F is 3200 Hz and each step down halves the rate.
 */
static uint8_t bw_rate_code(uint16_t odr_hz)
{
	uint8_t code = 0x06;   /* 6.25 Hz */

	while (code < 0x0F && (3200U >> (0x0F - code)) < odr_hz) {
		code++;
	}
	return code;
}

static int adxl345_burst_attr_set(const struct device *dev, enum sensor_channel chan,
				  enum sensor_attribute attr, const struct sensor_value *val)
{
	struct adxl345_burst_data *data = dev->data;

	if (attr != SENSOR_ATTR_SAMPLING_FREQUENCY) {
		return -ENOTSUP;
	}
	if (val->val1 < 1 || val->val1 > MAX_ODR_HZ) {
		return -EINVAL;
	}

	data->odr_hz = val->val1;
	(void)reg_xfer(dev, REG_BW_RATE, bw_rate_code(data->odr_hz), NULL);
	if (data->running) {
		sampling_update(data);
	}
	return 0;
}

static int adxl345_burst_attr_get(const struct device *dev, enum sensor_channel chan,
				  enum sensor_attribute attr, struct sensor_value *val)
{
	struct adxl345_burst_data *data = dev->data;

	val->val2 = 0;
	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		val->val1 = data->odr_hz;
		break;
	case ADXL345_BURST_ATTR_BURSTS:
		val->val1 = (int32_t)data->n_samples;
		break;
	case ADXL345_BURST_ATTR_OVERRUNS:
		val->val1 = (int32_t)data->n_overrun;
		break;
	case ADXL345_BURST_ATTR_ERRORS:
		val->val1 = (int32_t)data->n_err;
		break;
	case ADXL345_BURST_ATTR_FIFO_DROPS:
		val->val1 = (int32_t)data->n_fifo_drops;
		break;
	case ADXL345_BURST_ATTR_XFER_US_LAST:
		val->val1 = (int32_t)k_cyc_to_us_floor32(data->xfer_cyc_last);
		break;
	case ADXL345_BURST_ATTR_XFER_US_MAX:
		val->val1 = (int32_t)k_cyc_to_us_floor32(data->xfer_cyc_max);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

/*
 * Two behaviours depending on how the app uses the driver:
 * - With a trigger set, each call pops the oldest sample from the FIFO and returns
 *   -ENODATA once it's empty, so the handler can loop on sensor_sample_fetch() == 0.
 * - Without a trigger, it does one blocking burst read, like a regular sensor driver.
 */
static int adxl345_burst_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct adxl345_burst_config *cfg = dev->config;
	struct adxl345_burst_data *data = dev->data;
	k_spinlock_key_t key;
	int rc = 0;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_ACCEL_XYZ) {
		return -ENOTSUP;
	}

	if (!data->running) {
		rc = spi_transceive_dt(&cfg->bus, &data->tx_set, &data->rx_set);
		if (rc == 0) {
			data->cur.x = (int16_t)sys_get_le16(&data->rx[1]);
			data->cur.y = (int16_t)sys_get_le16(&data->rx[3]);
			data->cur.z = (int16_t)sys_get_le16(&data->rx[5]);
		}
		return rc;
	}

	key = k_spin_lock(&data->lock);
	if (data->count == 0) {
		rc = -ENODATA;
	} else {
		data->cur = data->fifo[data->head];
		data->head = (data->head + 1) % FIFO_SIZE;
		data->count--;
	}
	k_spin_unlock(&data->lock, key);

	return rc;
}

static void lsb_to_ms2(int16_t lsb, struct sensor_value *val)
{
	/* 256 LSB per g, and SENSOR_G is 1 g in micro-m/s^2 */
	sensor_value_from_micro(val, ((int64_t)lsb * SENSOR_G) / LSB_PER_G);
}

static int adxl345_burst_channel_get(const struct device *dev, enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct adxl345_burst_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		lsb_to_ms2(data->cur.x, val);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		lsb_to_ms2(data->cur.y, val);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		lsb_to_ms2(data->cur.z, val);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		lsb_to_ms2(data->cur.x, &val[0]);
		lsb_to_ms2(data->cur.y, &val[1]);
		lsb_to_ms2(data->cur.z, &val[2]);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static DEVICE_API(sensor, adxl345_burst_api) = {
	.attr_set = adxl345_burst_attr_set,
	.attr_get = adxl345_burst_attr_get,
	.trigger_set = adxl345_burst_trigger_set,
	.sample_fetch = adxl345_burst_sample_fetch,
	.channel_get = adxl345_burst_channel_get,
};

/* Initialization */

/* A single 2-byte transfer: write (reg, val) or read (CMD_READ | reg, 0) */
static int reg_xfer(const struct device *dev, uint8_t b0, uint8_t b1, uint8_t *out)
{
	const struct adxl345_burst_config *cfg = dev->config;
	uint8_t tx[2] __aligned(4) = { b0, b1 };
	uint8_t rx[2] __aligned(4) = { 0 };
	const struct spi_buf tb = { .buf = tx, .len = 2 };
	const struct spi_buf rb = { .buf = rx, .len = 2 };
	const struct spi_buf_set ts = { .buffers = &tb, .count = 1 };
	const struct spi_buf_set rs = { .buffers = &rb, .count = 1 };
	int rc;

	rc = spi_transceive_dt(&cfg->bus, &ts, &rs);
	if (out != NULL) {
		*out = rx[1];
	}
	return rc;
}

static int adxl345_burst_init(const struct device *dev)
{
	static bool q_started;
	const struct adxl345_burst_config *cfg = dev->config;
	struct adxl345_burst_data *data = dev->data;
	uint8_t id;
	int rc;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	if (cfg->watermark < 1 || cfg->watermark > FIFO_SIZE) {
		LOG_ERR("fifo-watermark %u out of range 1..%d", cfg->watermark, FIFO_SIZE);
		return -EINVAL;
	}

	data->dev = dev;
	data->odr_hz = cfg->odr_hz;
	data->watermark = cfg->watermark;

	data->tx[0] = CMD_BURST;
	data->tx_buf = (struct spi_buf){ .buf = data->tx, .len = BURST_LEN };
	data->rx_buf = (struct spi_buf){ .buf = data->rx, .len = BURST_LEN };
	data->tx_set = (struct spi_buf_set){ .buffers = &data->tx_buf, .count = 1 };
	data->rx_set = (struct spi_buf_set){ .buffers = &data->rx_buf, .count = 1 };

	/*
	 * Give the sensor time to power up: at least 10 ms, or longer if a start-up delay
	 * is configured. Since device init holds up the rest of the boot, the boot banner
	 * and all app output wait too. That's on purpose: the terminal usually connects a
	 * few seconds after reset, and this way it doesn't miss the first messages. It also
	 * gives the emulated sensor time to get ready before we read DEVID.
	 */
	k_msleep(MAX(10, CONFIG_ADXL345_BURST_STARTUP_DELAY_MS));

	/*
	 * Right after a reset the emulated sensor might not answer yet,
	 * so keep trying for a bit before giving up.
	 */
	for (int attempt = 1; ; attempt++) {
		id = 0;
		rc = reg_xfer(dev, CMD_READ | REG_DEVID, 0x00, &id);
		if (rc == 0 && id == ADXL345_DEVID) {
			break;
		}
		if (attempt == DEVID_ATTEMPTS) {
			LOG_ERR("DEVID 0x%02X (rc %d), expected 0x%02X: "
				"0xFF = MISO stuck high, 0x00 = no response", id, rc, ADXL345_DEVID);
			return -EIO;
		}
		if (attempt == 1) {
			LOG_WRN("DEVID 0x%02X (rc %d), retrying for up to %d ms", id, rc,
				DEVID_ATTEMPTS * DEVID_RETRY_MS);
		}
		k_msleep(DEVID_RETRY_MS);
	}

	/* Set +-2 g full resolution and the sample rate, then start measuring */
	rc = reg_xfer(dev, REG_DATA_FORMAT, DATA_FORMAT_FULL_RES_2G, NULL);
	rc = rc ? rc : reg_xfer(dev, REG_BW_RATE, bw_rate_code(data->odr_hz), NULL);
	rc = rc ? rc : reg_xfer(dev, REG_POWER_CTL, POWER_CTL_MEASURE, NULL);
	if (rc != 0) {
		LOG_ERR("register init failed: %d", rc);
		return rc;
	}

	if (!q_started) {
		const struct k_work_queue_config q_cfg = { .name = "adxl345_burst" };

		k_work_queue_start(&burst_q, burst_q_stack, K_THREAD_STACK_SIZEOF(burst_q_stack),
				   K_PRIO_COOP(CONFIG_ADXL345_BURST_THREAD_PRIORITY), &q_cfg);
		q_started = true;
	}

	k_timer_init(&data->timer, timer_expiry, NULL);
	k_work_init(&data->kick_work, kick_work_handler);
	k_work_init(&data->trig_work, trig_work_handler);

	LOG_INF("%s: DEVID 0x%02X, %u Hz, watermark %u, %s", dev->name, id, data->odr_hz,
		data->watermark, IS_ENABLED(CONFIG_SPI_ASYNC) ? "async SPI" : "blocking SPI");
	return 0;
}

#define ADXL345_BURST_DEFINE(inst)                                                         \
	static struct adxl345_burst_data adxl345_burst_data_##inst;                         \
	static const struct adxl345_burst_config adxl345_burst_config_##inst = {            \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_OP),                                  \
		.odr_hz = DT_INST_PROP(inst, sampling_frequency),                           \
		.watermark = DT_INST_PROP(inst, fifo_watermark),                            \
	};                                                                                  \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, adxl345_burst_init, NULL,                        \
				     &adxl345_burst_data_##inst,                            \
				     &adxl345_burst_config_##inst, POST_KERNEL,             \
				     CONFIG_SENSOR_INIT_PRIORITY, &adxl345_burst_api);

DT_INST_FOREACH_STATUS_OKAY(ADXL345_BURST_DEFINE)
