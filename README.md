# Event-driven ADXL345 demo for Zephyr

A small Zephyr application that reads an ADXL345 accelerometer **without polling**. It runs on three boards (nRF52 DK, NUCLEO-C562RE and PIC32CX SG61 Curiosity Ultra) against the ECW emulated ADXL345 sensor (hardware-in-the-loop).

**Tested on all three boards**.

A typical sensor demo reads X/Y/Z every few milliseconds and prints every sample. This one asks the driver to notify it when there's data with `sensor_trigger_set(SENSOR_TRIG_FIFO_WATERMARK)`, then sleeps. It only prints when something actually changes.

## How it works

There are two layers of events.

1. **Sensor events.** Every 10 samples (100 ms at 100 Hz) the driver fires the FIFO watermark trigger. The callback reads out the batch and computes mean/min/max per axis. These events are counted, not printed.
2. **Application events.** The main thread turns batches into things a person cares about:
   - `BASELINE`: the first reading after boot, saved as the reference orientation
   - `MOTION START` / `MOTION END`: one axis swings more than 500 mg within a batch, then calms down
   - `ORIENTATION CHANGE`: the board was tilted more than 200 mg on an axis

Every second the app prints one plain line with what changed, for example `orientation: 30 deg toward +X` or `motion: Z 1.8 g (0.1 s)`. Every 5 s a short status line says whether the board is healthy.

### The driver

The emulated sensor doesn't provide an INT1 pin or a hardware FIFO, so the driver in `drivers/adxl345_burst/` does both in software


From the application's point of view it behaves like a real ADXL345 with its FIFO watermark on INT1. If you switch to the real part, `main.c` doesn't need to change.

`src/main.c` and the driver are identical on every board. Only the board overlay and `.conf` in `source/boards/` are different.


## Supported boards

| Folder | Board | SPI | LED | Console (115200 8N1) | Flash base |
|---|---|---|---|---|---|
| `nrf52-dk` | nRF52 DK (PCA10040) | SPIM2 with EasyDMA | LED1 (P0.17) | J-Link VCOM (UART0) | `0x00000000` |
| `nucleo-c562re` | NUCLEO-C562RE | SPI1, interrupt-driven | none (LD1 shares the SCK pin) | ST-LINK VCOM (USART2) | `0x08000000` |
| `pic32cx-sg61` | PIC32CX SG61 Curiosity Ultra | SERCOM0 with DMA | LED1 (PC21) | PKoB4 VCOM (SERCOM4) | `0x00000000` |

The LED shows what the board is doing:

- **Steady on:** a motion is going on (on at `MOTION START`, off at `MOTION END`).
- **Short blink (100 ms):** the start position, and every orientation change. Set the length with `CONFIG_APP_LED_BLINK_MS`; 0 turns the blink off.
- **Off:** the board is still.

The NUCLEO-C562RE has no LED for this, because LD1 is on the SPI clock pin.


## What you should see

**Nothing appears for the first 5 seconds after reset.** That's on purpose. The terminal usually connects a few seconds after the board starts, so the driver waits before talking to the sensor and all output (including the Zephyr boot banner) waits with it. That way you never miss the start-up messages. To change the delay, set `CONFIG_ADXL345_BURST_STARTUP_DELAY_MS` in `prj.conf` (0 turns it off).

After that, you get at most one line per second saying what changed in that second. A quiet second prints nothing. Here's what `test1_orientation.csv` and then `test2_motion.csv` look like:

```
==================================================
 ADXL345 event demo on pic32cx_sg61_cult/pic32cx1025sg61128
 Shows motion and orientation changes every 1.0 s
 Status every 5 s
==================================================
[     6.0 s] orientation: flat (start), 23 deg toward +X, 20 deg toward +Y, 15 deg toward -X, 20 deg toward -Y
[     7.0 s] orientation: flat, 23 deg toward +X, 20 deg toward +Y, 15 deg toward -X, 20 deg toward -Y
[    10.0 s] status: OK, 500 samples, 0 errors, cpu 1.7%

[    61.0 s] motion: X 0.7 g (0.1 s), Z 1.9 g (0.1 s)
[    65.0 s] status: OK, 500 samples, 0 errors, cpu 1.7%
```

How to read a line:

- `[ t s]`: time since boot.
- `orientation:` the board moved to a new orientation and stayed there: `flat`, `23 deg toward +X`, `upside down` or `on its side`. `(start)` marks the first reading after boot.
- `motion:` a motion that has just finished: the axis that moved most, its strongest reading, and how long it lasted. If a motion goes on for more than a second, `moving` appears first.
- When both happen in the same second, they're separated by `|`, for example `orientation: 23 deg toward +X | motion: Z 1.9 g (0.1 s)`.
- `+N more`: some changes didn't fit on the line.
- `status`: `OK` or `CHECK`, samples read in the last 5 s, errors since boot (with a breakdown if there are any) and CPU load.

**A healthy board shows `status: OK`, about `500 samples` (499 or 501 is normal) and `0 errors`.**

### Expected results

**Before each test, reflash the baord and load one file only.** Otherwise data from the previous file stays at the end of the sensor's table and gets played back too. That shows up as positions or motions the file doesn't contain.


**Single values (`value_entries.txt`):**

| Values loaded | Expected output |
|---|---|
| Test A (flat) | `orientation: flat (start)` once, then only status lines |
| Test B (spike) | `orientation: ... (start)`, then `motion: moving` a second later, then only status lines (the motion never ends) |
| A, then C (tilt) | `orientation: 30 deg toward +X` |
| B, then A | `motion: Z ... g (... s)` once the spike stops |

### If the sensor isn't found

The log shows `DEVID 0x.. (rc ..), expected 0xE5` followed by `sensor adxl345@0 not ready`.

- `0xFF`: MISO is stuck high. Check that MOSI and MISO aren't swapped.
- `0x00`: nothing answered. Check CS and GND, and that the emulated sensor has data loaded.

## Configuration

| Kconfig option | Default | What it does |
|---|---|---|
| `CONFIG_ADXL345_BURST_STARTUP_DELAY_MS` | 5000 (set in `prj.conf`) | wait before the first sensor access, 0 = no wait |
| `CONFIG_APP_PRINT_PERIOD_MS` | 1000 | how often the line of changes is printed |
| `CONFIG_APP_STATUS_PERIOD_S` | 5 | status line period, 0 = off |
| `CONFIG_APP_MOTION_THRESHOLD_MG` | 500 | peak-to-peak within one batch that counts as motion |
| `CONFIG_APP_MOTION_END_BATCHES` | 3 | quiet batches in a row that end a motion |
| `CONFIG_APP_LED_BLINK_MS` | 100 | LED blink length for the start position and orientation changes, 0 = no blink |
| `CONFIG_APP_EVENT_DETAILS` | n | y = a detailed multi-line block per event |
| `CONFIG_APP_LOG_EVERY_BATCH` | n | y = also print every batch (noisy) |

To change the batch size or the sample rate, edit `fifo-watermark` (1 to 32) and `sampling-frequency` in the board overlay. Note that `src/main.c` also sets 100 Hz itself (`SAMPLING_HZ`).

## Building from source

From your Zephyr workspace, with its Python environment active:

```bash
# nRF52 DK
west build -p -b nrf52dk/nrf52832 demos/adxl345-event -- \
  -DEXTRA_DTC_OVERLAY_FILE=boards/nrf52-dk/nrf52dk_nrf52832.overlay
west flash

# NUCLEO-C562RE
west build -p -b nucleo_c562re demos/adxl345-event -- \
  -DEXTRA_DTC_OVERLAY_FILE=boards/nucleo-c562re/nucleo_c562re.overlay \
  -DEXTRA_CONF_FILE=boards/nucleo-c562re/nucleo_c562re.conf
west flash

# PIC32CX SG61 Curiosity Ultra
west build -p -b pic32cx_sg61_cult demos/adxl345-event -- \
  -DEXTRA_DTC_OVERLAY_FILE=boards/pic32-sg61/pic32cx_sg61_cult.overlay \
  -DEXTRA_CONF_FILE=boards/pic32-sg61/pic32cx_sg61_cult.conf
west flash
```

| Board | `build_target` |
|---|---|
| nRF52 DK | `nrf52dk/nrf52832` |
| NUCLEO-C562RE | `nucleo_c562re` |
| PIC32CX SG61 | `pic32cx_sg61_cult` |

The build target for each board is also in its `manifest.json`.

