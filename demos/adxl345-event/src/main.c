/*
 * Event-driven accelerometer demo, built on Zephyr's sensor trigger API.
 *
 * Instead of polling the sensor in a loop, we register a callback and let the
 * driver tell us when there is data. There are two kinds of events here:
 *
 * 1. Sensor events come from the driver. Each sample is one SPI burst. When
 *    the transfer finishes, the sample goes into the driver's FIFO. Once 10 samples
 *    are waiting (the fifo-watermark), the driver fires SENSOR_TRIG_FIFO_WATERMARK
 *    and watermark_handler() below empties the FIFO and summarizes the batch.
 *
 * 2. Application events are what the user actually cares about: BASELINE,
 *    MOTION START, MOTION END and ORIENTATION CHANGE. The main thread looks at
 *    each batch and only reports something when it changes. Quiet batches are just
 *    counted. To keep the console readable, everything that happened during one
 *    second is printed on a single line, and a short status line shows up every
 *    few seconds. Set CONFIG_APP_EVENT_DETAILS=y for a detailed block per event.
 *
 * This file doesn't know which board it runs on. The sensor comes from the
 * "accel0" devicetree alias and the LED from "led0", both set in the board overlay.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "adxl345_burst.h"   /* extra driver counters; other drivers simply return -ENOTSUP */

#define SAMPLING_HZ         100
#define MOTION_PP_MG        CONFIG_APP_MOTION_THRESHOLD_MG  /* peak-to-peak in one batch */
#define MOTION_END_BATCHES  CONFIG_APP_MOTION_END_BATCHES   /* quiet batches that end motion */
#define POSTURE_DELTA_MG    200   /* mean change that counts as a new orientation */
#define STATUS_PERIOD       K_SECONDS(CONFIG_APP_STATUS_PERIOD_S)
#define PRINT_PERIOD        K_MSEC(CONFIG_APP_PRINT_PERIOD_MS)

static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

static const char axis_name[3] = { 'X', 'Y', 'Z' };

/* Switch the LED (does nothing on boards without one) */
static void led_set(int on)
{
	if (led.port != NULL) {
		gpio_pin_set_dt(&led, on);
	}
}

/* Sensor events: one summary per batch of samples */
struct batch_event {
	uint32_t batch;          /* sensor event number, 1-based */
	uint32_t first_sample;   /* 1-based index of the first sample = burst number */
	uint32_t t_ms;           /* when the batch was handed to the app */
	uint16_t n;              /* samples in this batch */
	int32_t mean[3], min[3], max[3];
	uint16_t min_at[3], max_at[3];   /* offset in the batch of each min / max */
};

K_MSGQ_DEFINE(batch_q, sizeof(struct batch_event), 4, 4);

static uint32_t n_batches, n_samples, n_dropped;

static int32_t to_mg(const struct sensor_value *v)
{
	/* m/s^2 -> mg */
	return (int32_t)((sensor_value_to_micro(v) * 1000) / SENSOR_G);
}

/*
 * Called by the driver every time the FIFO reaches the watermark. This runs on the
 * system work queue, so we keep it short: read out the samples, compute mean/min/max,
 * and pass the summary to the main thread. No printing here.
 */
static void watermark_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	struct batch_event ev = {
		.first_sample = n_samples + 1,
		.min = { INT32_MAX, INT32_MAX, INT32_MAX },
		.max = { INT32_MIN, INT32_MIN, INT32_MIN },
	};
	int64_t sum[3] = { 0 };
	struct sensor_value xyz[3];

	ARG_UNUSED(trig);

	while (sensor_sample_fetch(dev) == 0) {
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, xyz);
		for (int i = 0; i < 3; i++) {
			int32_t mg = to_mg(&xyz[i]);

			sum[i] += mg;
			if (mg < ev.min[i]) {
				ev.min[i] = mg;
				ev.min_at[i] = ev.n;
			}
			if (mg > ev.max[i]) {
				ev.max[i] = mg;
				ev.max_at[i] = ev.n;
			}
		}
		ev.n++;
	}
	if (ev.n == 0) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		ev.mean[i] = (int32_t)(sum[i] / ev.n);
	}
	n_samples += ev.n;
	ev.batch = ++n_batches;
	ev.t_ms = k_uptime_get_32();

	if (k_msgq_put(&batch_q, &ev, K_NO_WAIT) != 0) {
		n_dropped++;   /* main thread is behind; drop rather than block the sensor */
	}
}

/* Application events: what we report to the user */
enum app_state { QUIET, MOTION };

static struct {
	enum app_state state;
	bool have_posture;
	int32_t posture[3];        /* mean of the batch that set the current orientation */
	uint32_t n_events;
	uint32_t quiet_run;        /* quiet batches since the last moving one */
	uint32_t motion_start_ms, motion_last_ms, motion_batches;
	int32_t motion_peak_mg;
	char motion_peak_axis;
	bool motion_told;          /* a long motion was already shown as "moving" */
} app;

struct posture {
	uint32_t g_mg;     /* |a| */
	char axis;         /* axis closest to vertical */
	bool up;           /* that axis reads +g */
	int tilt_deg;      /* angle between that axis and vertical */
};

static struct posture posture_of(const int32_t m[3])
{
	struct posture p = { 0 };
	float g = sqrtf((float)m[0] * m[0] + (float)m[1] * m[1] + (float)m[2] * m[2]);
	int dom = 0;

	for (int i = 1; i < 3; i++) {
		if (abs(m[i]) > abs(m[dom])) {
			dom = i;
		}
	}
	p.g_mg = (uint32_t)g;
	p.axis = axis_name[dom];
	p.up = m[dom] >= 0;
	p.tilt_deg = g > 0.0f ? (int)(acosf((float)abs(m[dom]) / g) * 57.29578f + 0.5f) : 0;
	return p;
}

static void print_posture(const char *label, const int32_t m[3])
{
	struct posture p = posture_of(m);

	printk("  %-8s: |a| = %u mg, %c axis %s, tilted %d deg\n", label, p.g_mg, p.axis,
	       p.up ? "up" : "down", p.tilt_deg);
}

static const char *state_name(void)
{
	return app.state == MOTION ? "MOTION" : "QUIET";
}

#define FLAT_DEG 5   /* below this tilt we just say "flat" */

/*
 * The orientation in plain words: "flat", "30 deg toward +X", "upside down",
 * "on its side (+X up)". The direction is the other axis that reads the most.
 */
static void position_text(char *buf, size_t size, const int32_t m[3])
{
	struct posture p = posture_of(m);
	int dom = p.axis - 'X';
	int a = (dom + 1) % 3;
	int b = (dom + 2) % 3;
	int lean = abs(m[a]) >= abs(m[b]) ? a : b;
	char tilt[32] = "";

	if (p.tilt_deg >= FLAT_DEG) {
		snprintk(tilt, sizeof(tilt), "%d deg toward %c%c", p.tilt_deg,
			 m[lean] >= 0 ? '+' : '-', axis_name[lean]);
	}
	if (p.axis == 'Z' && p.up) {
		snprintk(buf, size, "%s", tilt[0] != '\0' ? tilt : "flat");
	} else if (p.axis == 'Z') {
		snprintk(buf, size, "upside down%s%s", tilt[0] != '\0' ? ", " : "", tilt);
	} else {
		snprintk(buf, size, "on its side (%c%c up)%s%s", p.up ? '+' : '-', p.axis,
			 tilt[0] != '\0' ? ", " : "", tilt);
	}
}

/* mg as g with one decimal, e.g. 1800 -> "1.8", -550 -> "-0.6" */
static void g_text(char *buf, size_t size, int32_t mg)
{
	int32_t tenths = (abs(mg) + 50) / 100;

	snprintk(buf, size, "%s%d.%d", mg < 0 ? "-" : "", tenths / 10, tenths % 10);
}

/*
 * Count a new application event. In detailed mode this also prints
 * the header of the event block; otherwise the text is added to the current line with note().
 */
static void announce(const char *what, const struct batch_event *ev)
{
	app.n_events++;
	if (IS_ENABLED(CONFIG_APP_EVENT_DETAILS)) {
		printk("\n==== APP EVENT %u: %s ", app.n_events, what);
		for (int i = strlen(what); i < 40; i++) {
			printk("=");
		}
		printk(" t=%u.%03u s\n", ev->t_ms / 1000, ev->t_ms % 1000);
	}
}

/*
 * Everything that changed during the print period is collected here and printed
 * as one line. Changes of the same kind in a row share one label, e.g.
 *   [   23.0 s] orientation: flat, 30 deg toward +X | motion: Z 1.8 g (0.1 s)
 */
static const char KIND_ORIENTATION[] = "orientation";
static const char KIND_MOTION[] = "motion";

static struct {
	char text[256];
	size_t len;
	uint32_t count;     /* events collected since the last line */
	uint32_t untold;    /* events that did not fit into text */
	const char *kind;   /* kind of the last item on the line */
} changes;

static void note(const char *kind, const char *fmt, ...)
{
	char item[64];
	char piece[96];
	va_list ap;
	int n;

	va_start(ap, fmt);
	vsnprintk(item, sizeof(item), fmt, ap);
	va_end(ap);

	if (changes.len == 0) {
		n = snprintk(piece, sizeof(piece), "%s: %s", kind, item);
	} else if (kind == changes.kind) {
		n = snprintk(piece, sizeof(piece), ", %s", item);
	} else {
		n = snprintk(piece, sizeof(piece), " | %s: %s", kind, item);
	}
	n = CLAMP(n, 0, (int)sizeof(piece) - 1);

	changes.count++;
	/* Once the line is full, the rest only show up as "+N more" so the order stays correct */
	if (changes.untold > 0 || changes.len + n >= sizeof(changes.text)) {
		changes.untold++;
		return;
	}
	memcpy(&changes.text[changes.len], piece, n + 1);
	changes.len += n;
	changes.kind = kind;
}

static void print_changes(void)
{
	uint32_t now = k_uptime_get_32();

	/* A motion is reported when it ends. If one goes on for a while, say so once. */
	if (app.state == MOTION && !app.motion_told && now - app.motion_start_ms >= 1000) {
		note(KIND_MOTION, "moving");
		app.motion_told = true;
	}
	if (changes.count == 0) {
		return;
	}
	printk("[%6u.%u s] %s", now / 1000, (now % 1000) / 100, changes.text);
	if (changes.untold > 0) {
		printk(", +%u more", changes.untold);
	}
	printk("\n");
	changes.len = 0;
	changes.text[0] = '\0';
	changes.count = 0;
	changes.untold = 0;
	changes.kind = NULL;
}

static void print_batch(const struct batch_event *ev)
{
	printk("  trigger : sensor event #%u (FIFO watermark), %u samples = bursts #%u..#%u\n",
	       ev->batch, ev->n, ev->first_sample, ev->first_sample + ev->n - 1);
	printk("  accel   :     mean      min      max      p-p   (mg)\n");
	for (int i = 0; i < 3; i++) {
		int32_t pp = ev->max[i] - ev->min[i];

		printk("        %c : %8d %8d %8d %8d%s\n", axis_name[i], ev->mean[i], ev->min[i],
		       ev->max[i], pp, pp > MOTION_PP_MG ? "   <-- over threshold" : "");
	}
}

static const char *led_text(void)
{
	return led.port != NULL ? "LED" : "(no LED on this board)";
}

/*
 * The LED stays on for as long as a motion lasts, and gives a short blink for the
 * start position and every orientation change. So: steady on = moving, a blink =
 * tilted, off = still.
 */
static void led_blink_done(struct k_work *work)
{
	ARG_UNUSED(work);
	if (app.state != MOTION) {
		led_set(0);   /* a motion that started during the blink keeps the LED on */
	}
}

static K_WORK_DELAYABLE_DEFINE(led_blink_work, led_blink_done);

static void led_blink(void)
{
	if (CONFIG_APP_LED_BLINK_MS == 0 || app.state == MOTION) {
		return;
	}
	led_set(1);
	k_work_reschedule(&led_blink_work, K_MSEC(CONFIG_APP_LED_BLINK_MS));
}

/*
 * The little state machine that turns batches into events. It runs once per batch:
 *
 *   - First batch after boot: store it as the reference orientation (BASELINE).
 *   - QUIET, and one axis swings more than MOTION_PP_MG within the batch: MOTION START.
 *   - MOTION, and MOTION_END_BATCHES calm batches in a row: back to QUIET, MOTION END.
 *   - QUIET, and the average on an axis moved more than POSTURE_DELTA_MG since the
 *     reference: ORIENTATION CHANGE.
 *
 * Anything else changes nothing and is just counted.
 */
static void update_device_state(const struct batch_event *ev)
{
	int32_t pp_max = 0;
	int ax = 0;

	for (int i = 0; i < 3; i++) {
		if (ev->max[i] - ev->min[i] > pp_max) {
			pp_max = ev->max[i] - ev->min[i];
			ax = i;
		}
	}
	bool moving = pp_max > MOTION_PP_MG;
	char pos[64];
	char g[16];

	if (IS_ENABLED(CONFIG_APP_LOG_EVERY_BATCH)) {
		printk("[batch #%u t=%u ms n=%u] mean %d/%d/%d mg, p-p %d mg on %c%s\n",
		       ev->batch, ev->t_ms, ev->n, ev->mean[0], ev->mean[1], ev->mean[2], pp_max,
		       axis_name[ax], moving ? " MOTION" : "");
	}

	if (!app.have_posture) {
		announce("BASELINE", ev);
		if (IS_ENABLED(CONFIG_APP_EVENT_DETAILS)) {
			printk("  why     : first sensor event since boot, reference orientation stored\n");
			print_batch(ev);
			print_posture("posture", ev->mean);
			printk("  action  : %s blinks, waiting for changes\n", led_text());
		} else {
			position_text(pos, sizeof(pos), ev->mean);
			note(KIND_ORIENTATION, "%s (start)", pos);
		}
		memcpy(app.posture, ev->mean, sizeof(app.posture));
		app.have_posture = true;
		led_blink();
		return;
	}

	if (moving) {
		/* The peak is whichever extreme sits furthest from the mean on the busiest axis */
		bool hi = (ev->max[ax] - ev->mean[ax]) >= (ev->mean[ax] - ev->min[ax]);
		int32_t peak = hi ? ev->max[ax] : ev->min[ax];
		uint16_t at = hi ? ev->max_at[ax] : ev->min_at[ax];

		app.quiet_run = 0;
		app.motion_last_ms = ev->t_ms;
		app.motion_batches++;
		if (abs(peak) > abs(app.motion_peak_mg)) {
			app.motion_peak_mg = peak;
			app.motion_peak_axis = axis_name[ax];
		}
		if (app.state == MOTION) {
			return;   /* still moving; the summary comes with MOTION END */
		}

		app.state = MOTION;
		app.motion_start_ms = ev->t_ms;
		app.motion_batches = 1;
		app.motion_peak_mg = peak;
		app.motion_peak_axis = axis_name[ax];
		app.motion_told = false;
		k_work_cancel_delayable(&led_blink_work);
		led_set(1);

		/* Nothing goes on the line yet: the whole motion is shown once it's over */
		announce("MOTION START", ev);
		if (IS_ENABLED(CONFIG_APP_EVENT_DETAILS)) {
			printk("  why     : p-p %d mg on %c > %d mg threshold within one batch\n",
			       pp_max, axis_name[ax], MOTION_PP_MG);
			print_batch(ev);
			printk("  peak    : %d mg on %c at burst #%u (sample %u of %u)\n", peak,
			       axis_name[ax], ev->first_sample + at, at + 1, ev->n);
			printk("  action  : %s on until the motion ends\n", led_text());
		}
		return;
	}

	if (app.state == MOTION) {
		if (++app.quiet_run < MOTION_END_BATCHES) {
			return;
		}
		app.state = QUIET;
		led_set(0);

		uint32_t lasted_ms = app.motion_last_ms - app.motion_start_ms +
				     1000 * ev->n / SAMPLING_HZ;

		announce("MOTION END", ev);
		if (IS_ENABLED(CONFIG_APP_EVENT_DETAILS)) {
			printk("  why     : %d quiet batches in a row (%d ms without motion)\n",
			       MOTION_END_BATCHES, MOTION_END_BATCHES * 1000 * ev->n / SAMPLING_HZ);
			printk("  motion  : lasted ~%u ms over %u moving batch(es), peak %d mg on %c\n",
			       lasted_ms, app.motion_batches, app.motion_peak_mg,
			       app.motion_peak_axis);
			print_posture("posture", ev->mean);
			printk("  action  : %s off\n", led_text());
		} else {
			g_text(g, sizeof(g), app.motion_peak_mg);
			note(KIND_MOTION, "%c %s g (%u.%u s)", app.motion_peak_axis, g, lasted_ms / 1000,
			     (lasted_ms % 1000) / 100);
		}
		memcpy(app.posture, ev->mean, sizeof(app.posture));
		return;
	}

	/* Nothing is moving, so the only thing left to check is whether the board was tilted */
	for (int i = 0; i < 3; i++) {
		if (abs(ev->mean[i] - app.posture[i]) > POSTURE_DELTA_MG) {
			announce("ORIENTATION CHANGE", ev);
			if (IS_ENABLED(CONFIG_APP_EVENT_DETAILS)) {
				printk("  why     : mean moved more than %d mg since the last "
				       "posture, no motion in batch\n", POSTURE_DELTA_MG);
				print_batch(ev);
				print_posture("before", app.posture);
				print_posture("after", ev->mean);
				printk("  action  : new reference orientation stored, %s blinks\n",
				       led_text());
			} else {
				position_text(pos, sizeof(pos), ev->mean);
				note(KIND_ORIENTATION, "%s", pos);
			}
			memcpy(app.posture, ev->mean, sizeof(app.posture));
			led_blink();
			return;
		}
	}
}

/* Periodic status line */
static int32_t drv_stat(enum adxl345_burst_attribute attr)
{
	struct sensor_value v;

	if (sensor_attr_get(accel, SENSOR_CHAN_ALL, (enum sensor_attribute)attr, &v) != 0) {
		return -1;
	}
	return v.val1;
}

static void print_status(void)
{
	static uint32_t last_bursts, last_batches, last_events;
	static uint64_t last_exec, last_idle;
	uint32_t now = k_uptime_get_32();
	int32_t bursts = drv_stat(ADXL345_BURST_ATTR_BURSTS);
	int32_t busy_permille = -1;   /* stays -1 if runtime stats are off */

#ifdef CONFIG_THREAD_RUNTIME_STATS
	k_thread_runtime_stats_t rt;

	if (k_thread_runtime_stats_all_get(&rt) == 0) {
		uint64_t exec = rt.execution_cycles - last_exec;
		uint64_t idle = rt.idle_cycles - last_idle;

		if (exec > 0) {
			busy_permille = (int32_t)(((exec - idle) * 1000) / exec);
		}
		last_exec = rt.execution_cycles;
		last_idle = rt.idle_cycles;
	}
#endif

	if (IS_ENABLED(CONFIG_APP_EVENT_DETAILS)) {
		printk("\n---- status t=%u.%03u s, last %d s ", now / 1000, now % 1000,
		       CONFIG_APP_STATUS_PERIOD_S);
		printk("-----------------------------------\n");
		if (bursts >= 0) {
			printk("  sensor  : %d bursts (+%u), transfer %d us (max %d us)\n", bursts,
			       (uint32_t)bursts - last_bursts,
			       drv_stat(ADXL345_BURST_ATTR_XFER_US_LAST),
			       drv_stat(ADXL345_BURST_ATTR_XFER_US_MAX));
			printk("            %d overruns, %d errors, %d FIFO drops\n",
			       drv_stat(ADXL345_BURST_ATTR_OVERRUNS),
			       drv_stat(ADXL345_BURST_ATTR_ERRORS),
			       drv_stat(ADXL345_BURST_ATTR_FIFO_DROPS));
		}
		printk("  events  : %u sensor events (+%u) -> %u app events (+%u), %u dropped\n",
		       n_batches, n_batches - last_batches, app.n_events,
		       app.n_events - last_events, n_dropped);
		printk("  state   : %s\n", state_name());
		if (busy_permille >= 0) {
			printk("  cpu     : %d.%d %% busy, %d.%d %% idle (threads incl. console output)\n",
			       busy_permille / 10, busy_permille % 10,
			       (1000 - busy_permille) / 10, (1000 - busy_permille) % 10);
		}
	} else {
		/*
		 * Looks like this:
		 * [   45.0 s] status: OK, 500 samples, 0 errors, cpu 1.0%
		 * Samples cover the last period. Errors add up every problem counter since
		 * boot, so a healthy board always shows 0; if not, the line says which one.
		 */
		int32_t ovr = MAX(drv_stat(ADXL345_BURST_ATTR_OVERRUNS), 0);
		int32_t err = MAX(drv_stat(ADXL345_BURST_ATTR_ERRORS), 0);
		int32_t drop = MAX(drv_stat(ADXL345_BURST_ATTR_FIFO_DROPS), 0);
		uint32_t problems = (uint32_t)(ovr + err + drop) + n_dropped;

		printk("[%6u.%u s] status: %s", now / 1000, (now % 1000) / 100,
		       problems == 0 ? "OK" : "CHECK");
		if (bursts >= 0) {
			printk(", %u samples", (uint32_t)bursts - last_bursts);
		}
		printk(", %u errors", problems);
		if (problems > 0) {
			printk(" (overrun %d, spi %d, fifo full %d, queue full %u)", ovr, err, drop,
			       n_dropped);
		}
		if (busy_permille >= 0) {
			printk(", cpu %d.%d%%", busy_permille / 10, busy_permille % 10);
		}
		printk("\n");
	}

	if (bursts >= 0) {
		last_bursts = bursts;
	}
	last_batches = n_batches;
	last_events = app.n_events;
}

int main(void)
{
	const struct sensor_trigger trig = {
		.type = SENSOR_TRIG_FIFO_WATERMARK,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};
	const struct sensor_value odr = { .val1 = SAMPLING_HZ };
	k_timepoint_t next_status, next_print;
	int rc;

	printk("\n==================================================\n");
	printk(" ADXL345 event demo on %s\n", CONFIG_BOARD_TARGET);

	if (!device_is_ready(accel)) {
		printk("sensor %s not ready (check wiring / DEVID in the log)\n", accel->name);
		return 0;
	}
	if (led.port != NULL && gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	rc = sensor_attr_set(accel, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (rc != 0) {
		printk("sensor_attr_set(SAMPLING_FREQUENCY) failed: %d\n", rc);
		return 0;
	}

	rc = sensor_trigger_set(accel, &trig, watermark_handler);
	if (rc != 0) {
		printk("sensor_trigger_set(FIFO_WATERMARK) failed: %d\n", rc);
		return 0;
	}

	printk(" Shows motion and orientation changes every %d.%d s\n",
	       CONFIG_APP_PRINT_PERIOD_MS / 1000, (CONFIG_APP_PRINT_PERIOD_MS % 1000) / 100);
	if (CONFIG_APP_STATUS_PERIOD_S > 0) {
		printk(" Status every %d s\n", CONFIG_APP_STATUS_PERIOD_S);
	}
	printk("==================================================\n");

	next_status = sys_timepoint_calc(STATUS_PERIOD);
	next_print = sys_timepoint_calc(PRINT_PERIOD);
	while (1) {
		struct batch_event ev;

		/* Sleep until a batch arrives or it's time to print. No polling. */
		if (k_msgq_get(&batch_q, &ev, sys_timepoint_timeout(next_print)) == 0) {
			update_device_state(&ev);
		}
		if (sys_timepoint_expired(next_print)) {
			print_changes();   /* prints nothing if nothing happened */
			next_print = sys_timepoint_calc(PRINT_PERIOD);
		}
		if (CONFIG_APP_STATUS_PERIOD_S > 0 && sys_timepoint_expired(next_status)) {
			print_status();
			next_status = sys_timepoint_calc(STATUS_PERIOD);
		}
	}
	return 0;
}
