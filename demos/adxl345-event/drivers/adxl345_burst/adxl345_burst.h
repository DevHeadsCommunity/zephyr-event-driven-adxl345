/*
 * Extra counters from the adxl345_burst driver. Read them with sensor_attr_get() on
 * SENSOR_CHAN_ALL. Other sensor drivers just return -ENOTSUP, so the app can ask for
 * these without depending on this specific driver.
 */
#ifndef ADXL345_BURST_H_
#define ADXL345_BURST_H_

#include <zephyr/drivers/sensor.h>

enum adxl345_burst_attribute {
	/* Completed burst transfers (one per sample, one transfer-complete IRQ each) */
	ADXL345_BURST_ATTR_BURSTS = SENSOR_ATTR_PRIV_START,
	/* Timer ticks skipped because the previous burst was still in flight */
	ADXL345_BURST_ATTR_OVERRUNS,
	/* Failed transfers */
	ADXL345_BURST_ATTR_ERRORS,
	/* Samples dropped because the FIFO was full (application too slow) */
	ADXL345_BURST_ATTR_FIFO_DROPS,
	/* Burst start -> transfer-complete IRQ, last and max, in microseconds */
	ADXL345_BURST_ATTR_XFER_US_LAST,
	ADXL345_BURST_ATTR_XFER_US_MAX,
};

#endif /* ADXL345_BURST_H_ */
