/*
 * HealthyPi Move
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Protocentral Electronics
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/zbus/zbus.h>
#include <string.h>
#include <math.h>

#include "haptic_module.h"
#include "hpi_common_types.h"

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

/* Slow baseline EMA — alpha = 1/100 = 0.01 */
#define HAPTIC_BASELINE_ALPHA_NUM    1
#define HAPTIC_BASELINE_ALPHA_DEN    100

/* Warmup: minimum valid samples before baseline is trusted */
#define HAPTIC_HR_BASELINE_WARMUP    60    /* ~60s at 1 HR/s */
#define HAPTIC_GSR_BASELINE_WARMUP   120   /* ~2 min of valid GSR batches */
#define HAPTIC_HRV_BASELINE_WARMUP   30    /* 30 RMSSD computations */

/* Deviation thresholds — percentage rise/drop from personal baseline */
#define HAPTIC_HR_RISE_PCT           50    /* HR > baseline * 1.50 */
#define HAPTIC_GSR_RISE_PCT          60    /* GSR EMA > baseline * 1.60 */
#define HAPTIC_HRV_DROP_PCT          30    /* RMSSD < baseline * 0.70 */

/* All three sensors must flag stress within this window to trigger */
#define HAPTIC_COMBINED_WINDOW_MS    60000

/* Minimum time between combined triggers */
#define HAPTIC_COOLDOWN_MS           30000

/* GSR signal filter parameters */
#define HAPTIC_GSR_ROC_THRESHOLD     500    /* Max batch-to-batch change before motion reject */
#define HAPTIC_GSR_EMA_ALPHA_NUM     2      /* Fast EMA alpha = 2/100 = 0.02 — slower response to transients */
#define HAPTIC_GSR_EMA_ALPHA_DEN     100
#define HAPTIC_GSR_DEBOUNCE_BATCHES  32     /* Consecutive batches above threshold before flag */
#define HAPTIC_GSR_MIN_VALID         1      /* Raw ADC lower bound */
#define HAPTIC_GSR_MAX_VALID         10000  /* Raw ADC upper bound */
#define HAPTIC_GSR_EMA_MAX_DELTA     30     /* Max EMA change per batch — slew-limits transient spikes */
#define HAPTIC_GSR_EMA_ROC_THRESHOLD 20     /* Max EMA rise per batch before spike recovery triggered */
#define HAPTIC_GSR_BASELINE_SCALE    100    /* Baseline stored ×100 to preserve sub-unit precision through alpha=0.01 EMA */

/* Require this many consecutive below-threshold RMSSD samples before flagging HRV */
#define HAPTIC_HRV_DEBOUNCE_COUNT    3

/* Reject RMSSD computations this far above baseline as movement artifacts */
#define HAPTIC_HRV_SPIKE_PCT         25     /* rmssd > baseline * 1.25 → spike */

/* Cap on the very first warmup seed value. If the initial RMSSD computation
 * occurs during movement (common when first putting the device on), an
 * uncapped seed of 300+ ms propagates through the one-sided warmup and leaves
 * the baseline too high for the settled resting state. */
#define HAPTIC_HRV_WARMUP_SEED_MAX_MS    200

/* HRV rolling window */
#define HAPTIC_HRV_WINDOW            20

/* HRV warmup uses a faster alpha (2/10 = 0.2) and only allows downward movement.
 * This prevents early inflated RMSSD values from permanently elevating the baseline. */
#define HAPTIC_HRV_WARMUP_ALPHA_NUM  2
#define HAPTIC_HRV_WARMUP_ALPHA_DEN  10

/* HRV baseline is stored scaled by this factor to preserve sub-ms precision
 * through the alpha=0.01 EMA. Without scaling, integer truncation freezes the
 * baseline — e.g. (134*99 + 165)/100 = 134 forever. */
#define HAPTIC_HRV_BASELINE_SCALE    100

LOG_MODULE_REGISTER(haptic_module, LOG_LEVEL_DBG);

#define DOG_DEVICE_NAME "Dog Device Receiver"

/* ------------------------------------------------------------------ */
/* BLE state                                                           */
/* ------------------------------------------------------------------ */

/* Service UUID: 19B10000-E8F2-537E-4F6C-D104768A1214 */
static struct bt_uuid_128 dog_svc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x19B10000, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));

/* Characteristic UUID: 19B10001-E8F2-537E-4F6C-D104768A1214 */
static struct bt_uuid_128 dog_chr_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x19B10001, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));

static struct bt_conn *dog_conn;
static uint16_t dog_chr_handle;
static bool chr_handle_valid;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_write_params write_params;
static uint8_t write_buf;

/* ------------------------------------------------------------------ */
/* Per-sensor dynamic baseline state                                   */
/* ------------------------------------------------------------------ */

struct haptic_sensor_state {
	int32_t  baseline;
	uint16_t warmup_count;
	bool     stressed;
	int64_t  flagged_at_ms;
};

static struct haptic_sensor_state hr_state;
static struct haptic_sensor_state gsr_state;
static struct haptic_sensor_state hrv_state;

static int64_t last_trigger_time;

/* ------------------------------------------------------------------ */
/* HRV circular buffer                                                 */
/* ------------------------------------------------------------------ */

static uint16_t hrv_rr_buf[HAPTIC_HRV_WINDOW];
static uint8_t  hrv_rr_count;
static uint8_t  hrv_rr_head;

/* ------------------------------------------------------------------ */
/* Scanning                                                            */
/* ------------------------------------------------------------------ */

static void start_scan(void);

static void scan_start_work_handler(struct k_work *work)
{
	LOG_ERR("haptic: starting scan");
	start_scan();
}

static K_WORK_DELAYABLE_DEFINE(scan_start_work, scan_start_work_handler);

/* ------------------------------------------------------------------ */
/* GATT discovery                                                      */
/* ------------------------------------------------------------------ */

static uint8_t discover_cb(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   struct bt_gatt_discover_params *params)
{
	if (!attr) {
		LOG_INF("GATT discovery complete");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY) {
		LOG_INF("Dog service found, discovering characteristic");
		discover_params.uuid = &dog_chr_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.end_handle =
			((struct bt_gatt_service_val *)attr->user_data)->end_handle;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		int err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			LOG_ERR("Characteristic discovery failed: %d", err);
		}
		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

		dog_chr_handle = chrc->value_handle;
		chr_handle_valid = true;
		LOG_INF("Dog characteristic ready, handle: 0x%04X", dog_chr_handle);
		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

/* ------------------------------------------------------------------ */
/* Scanning                                                            */
/* ------------------------------------------------------------------ */

struct adv_parse_data {
	bool found;
};

static bool parse_adv(struct bt_data *data, void *user_data)
{
	struct adv_parse_data *d = user_data;

	if (data->type != BT_DATA_NAME_COMPLETE &&
	    data->type != BT_DATA_NAME_SHORTENED) {
		return true;
	}

	if (data->data_len == sizeof(DOG_DEVICE_NAME) - 1 &&
	    memcmp(data->data, DOG_DEVICE_NAME, data->data_len) == 0) {
		d->found = true;
	}

	return false;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	struct adv_parse_data d = { .found = false };

	if (dog_conn) {
		return;
	}

	bt_data_parse(buf, parse_adv, &d);
	if (!d.found) {
		return;
	}

	LOG_INF("Found Dog Device Receiver (RSSI %d), connecting...", rssi);

	int err = bt_le_scan_stop();
	if (err) {
		LOG_ERR("Scan stop failed: %d", err);
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &dog_conn);
	if (err) {
		LOG_ERR("Create connection failed: %d", err);
		start_scan();
	}
}

static void start_scan(void)
{
	int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);

	if (err) {
		LOG_ERR("haptic: scan start failed: %d", err);
		return;
	}
	LOG_ERR("haptic: scanning for Dog Device Receiver");
}

/* ------------------------------------------------------------------ */
/* Connection callbacks                                                */
/* ------------------------------------------------------------------ */

static void haptic_connected(struct bt_conn *conn, uint8_t err)
{
	if (conn != dog_conn) {
		return;
	}

	if (err) {
		LOG_ERR("Dog device connection failed (err %u)", err);
		bt_conn_unref(dog_conn);
		dog_conn = NULL;
		start_scan();
		return;
	}

	LOG_INF("Connected to Dog Device Receiver");
	chr_handle_valid = false;

	discover_params.uuid = &dog_svc_uuid.uuid;
	discover_params.func = discover_cb;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type         = BT_GATT_DISCOVER_PRIMARY;

	int ret = bt_gatt_discover(conn, &discover_params);

	if (ret) {
		LOG_ERR("Service discovery failed: %d", ret);
	}
}

static void haptic_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != dog_conn) {
		return;
	}

	LOG_INF("Disconnected from Dog Device Receiver (reason %u)", reason);
	bt_conn_unref(dog_conn);
	dog_conn = NULL;
	chr_handle_valid = false;

	start_scan();
}

BT_CONN_CB_DEFINE(haptic_conn_callbacks) = {
	.connected    = haptic_connected,
	.disconnected = haptic_disconnected,
};

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

static void write_cb(struct bt_conn *conn, uint8_t err,
		     struct bt_gatt_write_params *params)
{
	if (err) {
		LOG_ERR("Alert write error: %d", err);
	} else {
		LOG_INF("Alert write confirmed by dog device");
	}
}

int haptic_send_alert(uint8_t value)
{
	if (!dog_conn || !chr_handle_valid) {
		LOG_WRN("Dog device not ready, alert dropped");
		return -ENODEV;
	}

	write_buf = value;
	write_params.func   = write_cb;
	write_params.handle = dog_chr_handle;
	write_params.offset = 0;
	write_params.data   = &write_buf;
	write_params.length = sizeof(write_buf);

	int err = bt_gatt_write(dog_conn, &write_params);
	if (err) {
		LOG_ERR("Write failed: %d", err);
		return err;
	}

	LOG_INF("Alert sent: %u", value);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Combined alert work item                                            */
/* ------------------------------------------------------------------ */

static void haptic_combined_alert_work_handler(struct k_work *work)
{
	haptic_send_alert(1);
}

static K_WORK_DEFINE(haptic_combined_alert_work, haptic_combined_alert_work_handler);

/* ------------------------------------------------------------------ */
/* Combined trigger check                                              */
/* ------------------------------------------------------------------ */

static void haptic_check_combined(void)
{
	if (!hr_state.stressed || !gsr_state.stressed || !hrv_state.stressed) {
		return;
	}

	int64_t now = k_uptime_get();

	if ((now - last_trigger_time) < HAPTIC_COOLDOWN_MS) {
		return;
	}

	/* Find the oldest flag timestamp — all three must be within the window */
	int64_t oldest = hr_state.flagged_at_ms;

	if (gsr_state.flagged_at_ms < oldest) {
		oldest = gsr_state.flagged_at_ms;
	}
	if (hrv_state.flagged_at_ms < oldest) {
		oldest = hrv_state.flagged_at_ms;
	}

	if ((now - oldest) > HAPTIC_COMBINED_WINDOW_MS) {
		return;
	}

	last_trigger_time = now;
	LOG_INF("haptic: COMBINED TRIGGER — HR %lld ms ago, GSR %lld ms ago, HRV %lld ms ago",
		(long long)(now - hr_state.flagged_at_ms),
		(long long)(now - gsr_state.flagged_at_ms),
		(long long)(now - hrv_state.flagged_at_ms));

	hr_state.stressed      = false;
	hr_state.flagged_at_ms = 0;
	gsr_state.stressed      = false;
	gsr_state.flagged_at_ms = 0;
	hrv_state.stressed      = false;
	hrv_state.flagged_at_ms = 0;

	k_work_submit(&haptic_combined_alert_work);
}

/* ------------------------------------------------------------------ */
/* HR-based stress detection                                           */
/* ------------------------------------------------------------------ */

static void haptic_hr_listener(const struct zbus_channel *chan)
{
	const struct hpi_hr_t *hr = zbus_chan_const_msg(chan);

	if (!hr->hr_ready_flag || hr->hr == 0) {
		return;
	}

	int32_t current_hr = (int32_t)hr->hr;

	/* Physiological sanity gate */
	if (current_hr < 30 || current_hr > 220) {
		return;
	}

	/* Warmup — accumulate baseline, no stress detection yet */
	if (hr_state.warmup_count < HAPTIC_HR_BASELINE_WARMUP) {
		hr_state.baseline = (hr_state.warmup_count == 0)
			? current_hr
			: (hr_state.baseline * (HAPTIC_BASELINE_ALPHA_DEN - HAPTIC_BASELINE_ALPHA_NUM) +
			   current_hr * HAPTIC_BASELINE_ALPHA_NUM) / HAPTIC_BASELINE_ALPHA_DEN;
		hr_state.warmup_count++;

		static int64_t last_hr_warmup_log;
		int64_t now_w = k_uptime_get();

		if ((now_w - last_hr_warmup_log) >= 5000) {
			LOG_INF("HR: warming up (%u/%u) hr=%d baseline=%d",
				hr_state.warmup_count, HAPTIC_HR_BASELINE_WARMUP,
				current_hr, hr_state.baseline);
			last_hr_warmup_log = now_w;
		}
		return;
	}

	/* Update baseline — freeze while stressed to preserve resting reference */
	if (!hr_state.stressed) {
		hr_state.baseline = (hr_state.baseline * (HAPTIC_BASELINE_ALPHA_DEN - HAPTIC_BASELINE_ALPHA_NUM) +
				     current_hr * HAPTIC_BASELINE_ALPHA_NUM) / HAPTIC_BASELINE_ALPHA_DEN;
	}

	/* Threshold: baseline + 20% */
	int32_t threshold = hr_state.baseline + (hr_state.baseline * HAPTIC_HR_RISE_PCT / 100);

	static int64_t last_hr_log;
	int64_t now = k_uptime_get();

	if ((now - last_hr_log) >= 5000) {
		LOG_INF("HR: current=%d baseline=%d threshold=%d stressed=%d",
			current_hr, hr_state.baseline, threshold, (int)hr_state.stressed);
		last_hr_log = now;
	}

	if (current_hr > threshold) {
		if (!hr_state.stressed) {
			hr_state.stressed      = true;
			hr_state.flagged_at_ms = k_uptime_get();
			LOG_INF("HR flagged: %d > %d (baseline=%d)",
				current_hr, threshold, hr_state.baseline);
		}
	} else {
		hr_state.stressed      = false;
		hr_state.flagged_at_ms = 0;
	}

	haptic_check_combined();
}

ZBUS_LISTENER_DEFINE(haptic_hr_lis, haptic_hr_listener);

/* ------------------------------------------------------------------ */
/* HRV (RMSSD) based stress detection                                  */
/* ------------------------------------------------------------------ */

void haptic_process_rtor(uint16_t rtor_ms)
{
	/* Reject physiologically impossible intervals */
	if (rtor_ms < 300 || rtor_ms > 1500) {
		return;
	}

	/* Add to circular buffer */
	hrv_rr_buf[hrv_rr_head] = rtor_ms;
	hrv_rr_head = (hrv_rr_head + 1) % HAPTIC_HRV_WINDOW;
	if (hrv_rr_count < HAPTIC_HRV_WINDOW) {
		hrv_rr_count++;
	}

	/* Wait for a full window before computing RMSSD — partial-buffer values
	 * (e.g. rmssd=29 from only 2 intervals) corrupt the baseline if fed
	 * during warmup. */
	if (hrv_rr_count < HAPTIC_HRV_WINDOW) {
		return;
	}

	/* Compute RMSSD over the window */
	uint64_t sum_sq = 0;
	uint8_t  pairs  = 0;

	for (uint8_t i = 1; i < hrv_rr_count; i++) {
		uint8_t idx_curr = (hrv_rr_head - 1 - (hrv_rr_count - 1 - i) +
				    HAPTIC_HRV_WINDOW) % HAPTIC_HRV_WINDOW;
		uint8_t idx_prev = (idx_curr - 1 + HAPTIC_HRV_WINDOW) % HAPTIC_HRV_WINDOW;
		int32_t diff = (int32_t)hrv_rr_buf[idx_curr] - (int32_t)hrv_rr_buf[idx_prev];

		sum_sq += (uint64_t)(diff * diff);
		pairs++;
	}

	if (pairs == 0) {
		return;
	}

	uint32_t rmssd = (uint32_t)sqrt((double)(sum_sq / pairs));

	/* Warmup phase — baseline stored scaled by HAPTIC_HRV_BASELINE_SCALE to
	 * preserve fractional precision through the alpha=0.01 EMA. */
	if (hrv_state.warmup_count < HAPTIC_HRV_BASELINE_WARMUP) {
		/* Cap the seed to prevent a movement spike on first wear from
		 * permanently inflating the warmup baseline. */
		uint32_t rmssd_seed = (rmssd > HAPTIC_HRV_WARMUP_SEED_MAX_MS)
					? HAPTIC_HRV_WARMUP_SEED_MAX_MS : rmssd;
		int32_t rmssd_fp = (int32_t)rmssd * HAPTIC_HRV_BASELINE_SCALE;
		if (hrv_state.warmup_count == 0) {
			hrv_state.baseline = (int32_t)rmssd_seed * HAPTIC_HRV_BASELINE_SCALE;
		} else {
			int32_t new_baseline =
				(hrv_state.baseline * (HAPTIC_HRV_WARMUP_ALPHA_DEN - HAPTIC_HRV_WARMUP_ALPHA_NUM) +
				 rmssd_fp * HAPTIC_HRV_WARMUP_ALPHA_NUM) /
				HAPTIC_HRV_WARMUP_ALPHA_DEN;
			/* Only allow downward movement — prevents early inflated RMSSD
			 * values from permanently elevating the threshold. */
			if (new_baseline < hrv_state.baseline) {
				hrv_state.baseline = new_baseline;
			}
		}
		hrv_state.warmup_count++;

		static int64_t last_hrv_warmup_log;
		int64_t now_w = k_uptime_get();

		if ((now_w - last_hrv_warmup_log) >= 5000) {
			LOG_INF("HRV: warming up (%u/%u) rmssd=%u baseline=%d",
				hrv_state.warmup_count, HAPTIC_HRV_BASELINE_WARMUP,
				rmssd, hrv_state.baseline / HAPTIC_HRV_BASELINE_SCALE);
			last_hrv_warmup_log = now_w;
		}
		return;
	}

	/* Unscale BEFORE any update so spike readings never contaminate the
	 * baseline used for threshold comparison or the log. */
	int32_t baseline_ms = hrv_state.baseline / HAPTIC_HRV_BASELINE_SCALE;
	int32_t threshold   = baseline_ms - (baseline_ms * HAPTIC_HRV_DROP_PCT / 100);

	static int64_t last_hrv_log;
	int64_t now = k_uptime_get();

	if ((now - last_hrv_log) >= 5000) {
		LOG_INF("HRV: rtor=%u rmssd=%u baseline=%d threshold=%d stressed=%d",
			rtor_ms, rmssd, baseline_ms, threshold, (int)hrv_state.stressed);
		last_hrv_log = now;
	}

	static uint8_t hrv_below_count;
	static bool    hrv_spike_recovery;

	int32_t spike_threshold = baseline_ms + (baseline_ms * HAPTIC_HRV_SPIKE_PCT / 100);

	/* Upward spike rejection — return before updating baseline so that spike
	 * readings cannot inflate the baseline and raise the drop threshold. */
	if ((int32_t)rmssd > spike_threshold) {
		if (!hrv_spike_recovery) {
			hrv_spike_recovery = true;
			hrv_below_count    = 0;
		}
		LOG_INF("HRV: upward spike rejected rmssd=%u > spike_threshold=%d (baseline=%d)",
			rmssd, spike_threshold, baseline_ms);
		return;
	}
	if (hrv_spike_recovery) {
		LOG_INF("HRV: spike recovery cleared, rmssd=%u <= spike_threshold=%d",
			rmssd, spike_threshold);
		hrv_spike_recovery = false;
		hrv_below_count    = 0;
	}

	/* Update baseline — only non-spike readings reach here. Freeze while stressed. */
	if (!hrv_state.stressed) {
		hrv_state.baseline = (hrv_state.baseline * (HAPTIC_BASELINE_ALPHA_DEN - HAPTIC_BASELINE_ALPHA_NUM) +
				      (int32_t)rmssd * HAPTIC_HRV_BASELINE_SCALE * HAPTIC_BASELINE_ALPHA_NUM) /
				     HAPTIC_BASELINE_ALPHA_DEN;
	}

	if ((int32_t)rmssd < threshold) {
		hrv_below_count++;
		if (hrv_below_count >= HAPTIC_HRV_DEBOUNCE_COUNT) {
			if (!hrv_state.stressed) {
				hrv_state.stressed      = true;
				hrv_state.flagged_at_ms = k_uptime_get();
				LOG_INF("HRV flagged: rmssd=%u < threshold=%d (baseline=%d) [debounce %u/%u]",
					rmssd, threshold, baseline_ms,
					hrv_below_count, HAPTIC_HRV_DEBOUNCE_COUNT);
			}
			haptic_check_combined();
		}
	} else {
		if (hrv_below_count > 0) {
			LOG_DBG("HRV: debounce reset (%u/%u), rmssd=%u recovered above threshold=%d",
				hrv_below_count, HAPTIC_HRV_DEBOUNCE_COUNT, rmssd, threshold);
		}
		hrv_below_count         = 0;
		hrv_state.stressed      = false;
		hrv_state.flagged_at_ms = 0;
	}
}

/* ------------------------------------------------------------------ */
/* GSR-based stress detection                                          */
/* ------------------------------------------------------------------ */

void haptic_process_gsr(const int32_t *samples, uint8_t num_samples, uint8_t lead_off)
{
	static int32_t gsr_prev_mean = INT32_MIN;
	static int32_t gsr_ema       = INT32_MIN;
	static uint16_t gsr_above_count;
	static bool     gsr_spike_recovery;

	if (num_samples == 0) {
		return;
	}

	/* Reset all state on lead-off — electrode environment has changed */
	if (lead_off) {
		gsr_prev_mean      = INT32_MIN;
		gsr_ema            = INT32_MIN;
		gsr_above_count    = 0;
		gsr_spike_recovery = false;
		gsr_state          = (struct haptic_sensor_state){0};
		return;
	}

	/* Compute batch mean */
	int64_t sum = 0;

	for (uint8_t i = 0; i < num_samples; i++) {
		sum += samples[i];
	}
	int32_t mean = (int32_t)(sum / num_samples);

	/* Valid range gate — skip negatives and saturation artifacts, retain EMA */
	if (mean < HAPTIC_GSR_MIN_VALID || mean > HAPTIC_GSR_MAX_VALID) {
		return;
	}

	/* ROC motion rejection — only reject large upward spikes.
	 * Downward movement is allowed freely: a falling GSR cannot cause a false
	 * stress flag, and blocking drops prevents the boot transient from decaying. */
	if (gsr_prev_mean != INT32_MIN) {
		int32_t roc = mean - gsr_prev_mean;

		if (roc > HAPTIC_GSR_ROC_THRESHOLD) {
			LOG_DBG("GSR: upward spike rejected (roc=%d)", roc);
			gsr_prev_mean      = mean;
			gsr_spike_recovery = true;
			gsr_above_count    = 0;
			return;
		}
	}
	gsr_prev_mean = mean;

	/* Fast EMA — denoises before feeding into slow baseline.
	 * Upward slew-rate limited: sudden contact spikes cannot propagate upward
	 * faster than HAPTIC_GSR_EMA_MAX_DELTA per batch. Downward movement is
	 * unrestricted so the boot transient decays quickly. */
	if (gsr_ema == INT32_MIN) {
		gsr_ema = mean;
	} else {
		int32_t new_ema = (gsr_ema * (HAPTIC_GSR_EMA_ALPHA_DEN - HAPTIC_GSR_EMA_ALPHA_NUM) +
				   mean * HAPTIC_GSR_EMA_ALPHA_NUM) / HAPTIC_GSR_EMA_ALPHA_DEN;
		int32_t delta = new_ema - gsr_ema;

		if (delta > HAPTIC_GSR_EMA_MAX_DELTA) {
			delta = HAPTIC_GSR_EMA_MAX_DELTA;
		}
		/* No downward limit — allows boot transient to decay naturally */
		gsr_ema = gsr_ema + delta;

		/* EMA ROC check — fast upward movement in the smoothed signal
		 * indicates a sustained contact/conductance spike; suppress debounce
		 * accumulation until the signal settles. */
		if (delta > HAPTIC_GSR_EMA_ROC_THRESHOLD) {
			LOG_DBG("GSR: EMA spike rejected (ema_delta=%d)", delta);
			gsr_spike_recovery = true;
			gsr_above_count    = 0;
		}
	}

	/* Warmup — feed slow baseline from denoised EMA output.
	 * Baseline stored scaled by HAPTIC_GSR_BASELINE_SCALE to preserve
	 * sub-unit precision through the alpha=0.01 EMA. */
	if (gsr_state.warmup_count < HAPTIC_GSR_BASELINE_WARMUP) {
		gsr_state.baseline = (gsr_state.warmup_count == 0)
			? gsr_ema * HAPTIC_GSR_BASELINE_SCALE
			: (gsr_state.baseline * (HAPTIC_BASELINE_ALPHA_DEN - HAPTIC_BASELINE_ALPHA_NUM) +
			   gsr_ema * HAPTIC_GSR_BASELINE_SCALE * HAPTIC_BASELINE_ALPHA_NUM) / HAPTIC_BASELINE_ALPHA_DEN;
		gsr_state.warmup_count++;

		int64_t now_w = k_uptime_get();
		static int64_t last_gsr_warmup_log;

		if ((now_w - last_gsr_warmup_log) >= 5000) {
			LOG_INF("GSR: warming up (%u/%u) ema=%d baseline=%d",
				gsr_state.warmup_count, HAPTIC_GSR_BASELINE_WARMUP,
				gsr_ema, gsr_state.baseline / HAPTIC_GSR_BASELINE_SCALE);
			last_gsr_warmup_log = now_w;
		}
		return;
	}

	/* Unscale baseline for comparisons and threshold computation */
	int32_t baseline = gsr_state.baseline / HAPTIC_GSR_BASELINE_SCALE;

	/* Update slow baseline — asymmetric tracking:
	 * - Upward (ema >= baseline): alpha = 0.01 — tracks genuine long-term rises.
	 * - Downward (ema < baseline): alpha = 0.001 — 10× slower, so temporary
	 *   contact loss or dry-skin dips cannot drag the baseline to near-zero
	 *   and create a false flag when skin contact recovers.
	 * - Stressed: upward-only at alpha = 0.001 — prevents inflation during
	 *   genuine stress events while still allowing slow recovery. */
	if (!gsr_state.stressed) {
		if (gsr_ema >= baseline) {
			gsr_state.baseline = (gsr_state.baseline * (HAPTIC_BASELINE_ALPHA_DEN - HAPTIC_BASELINE_ALPHA_NUM) +
					      gsr_ema * HAPTIC_GSR_BASELINE_SCALE * HAPTIC_BASELINE_ALPHA_NUM) / HAPTIC_BASELINE_ALPHA_DEN;
		} else {
			int32_t new_bl = (gsr_state.baseline * 999 + gsr_ema * HAPTIC_GSR_BASELINE_SCALE) / 1000;

			if (new_bl < gsr_state.baseline) {
				gsr_state.baseline = new_bl;
			}
		}
	} else {
		int32_t new_bl = (gsr_state.baseline * 999 + gsr_ema * HAPTIC_GSR_BASELINE_SCALE) / 1000;

		if (new_bl > gsr_state.baseline) {
			gsr_state.baseline = new_bl;
		}
	}

	/* Re-read unscaled baseline after update */
	baseline = gsr_state.baseline / HAPTIC_GSR_BASELINE_SCALE;

	int64_t now = k_uptime_get();
	static int64_t last_gsr_log;

	/* Threshold: baseline + 60% */
	int32_t threshold = baseline + (baseline * HAPTIC_GSR_RISE_PCT / 100);

	if ((now - last_gsr_log) >= 5000) {
		LOG_INF("GSR: ema=%d baseline=%d threshold=%d stressed=%d",
			gsr_ema, baseline, threshold, (int)gsr_state.stressed);
		last_gsr_log = now;
	}

	/* Debounce: require sustained elevation above dynamic threshold */
	if (gsr_ema < threshold) {
		if (gsr_spike_recovery) {
			LOG_DBG("GSR: spike recovery cleared, ema=%d < threshold=%d",
				gsr_ema, threshold);
			gsr_spike_recovery = false;
		}
		gsr_above_count         = 0;
		gsr_state.stressed      = false;
		gsr_state.flagged_at_ms = 0;
		return;
	}

	/* Still above threshold: suppress debounce accumulation while in spike recovery */
	if (gsr_spike_recovery) {
		gsr_above_count = 0;
		return;
	}

	gsr_above_count++;
	if (gsr_above_count < HAPTIC_GSR_DEBOUNCE_BATCHES) {
		return;
	}

	/* Debounce satisfied — raise stress flag */
	if (!gsr_state.stressed) {
		gsr_state.stressed      = true;
		gsr_state.flagged_at_ms = k_uptime_get();
		LOG_INF("GSR flagged: ema=%d > threshold=%d (baseline=%d)",
			gsr_ema, threshold, baseline);
	}

	haptic_check_combined();
}

/* ------------------------------------------------------------------ */
/* Module initialisation                                               */
/* ------------------------------------------------------------------ */

int haptic_module_init(void)
{
	dog_conn          = NULL;
	chr_handle_valid  = false;
	dog_chr_handle    = 0;
	last_trigger_time = 0;

	memset(&hr_state,  0, sizeof(hr_state));
	memset(&gsr_state, 0, sizeof(gsr_state));
	memset(&hrv_state, 0, sizeof(hrv_state));

	hrv_rr_count = 0;
	hrv_rr_head  = 0;

	LOG_INF("haptic: module init — dynamic baseline (warmup HR=%u GSR=%u HRV=%u)",
		HAPTIC_HR_BASELINE_WARMUP, HAPTIC_GSR_BASELINE_WARMUP, HAPTIC_HRV_BASELINE_WARMUP);

	k_work_schedule(&scan_start_work, K_SECONDS(5));
	return 0;
}
