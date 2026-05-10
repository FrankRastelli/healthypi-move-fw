/*
 * HealthyPi Move GSR Screen
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Protocentral Electronics
 *
 * Live EMA display updated via haptic_gsr_chan zbus channel.
 * "View EMA Plot" button opens SCR_SPL_GSR_LIVE_PLOT.
 *
 * EMA values are in µS × 100 (fixed-point), displayed as X.XX µS.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <lvgl.h>
#include <stdio.h>

#include "hpi_common_types.h"
#include "ui/move_ui.h"

LOG_MODULE_REGISTER(hpi_disp_scr_gsr, LOG_LEVEL_ERR);

// GUI Objects
lv_obj_t *scr_gsr;
static lv_obj_t *label_gsr_ema;
static lv_obj_t *label_gsr_baseline;
static lv_obj_t *label_gsr_threshold;
static lv_obj_t *label_gsr_stress_state;
static lv_obj_t *label_gsr_last_update;

// Externs - Modern style system
extern lv_style_t style_body_medium;
extern lv_style_t style_numeric_large;
extern lv_style_t style_caption;

#define COLOR_GSR_TEAL  0x00897B

void draw_scr_gsr(enum scroll_dir m_scroll_dir)
{
    scr_gsr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_gsr, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(scr_gsr, LV_OBJ_FLAG_SCROLLABLE);

    /*
     * LAYOUT FOR 390x390 ROUND AMOLED
     * ================================
     *   y=40:  Title "GSR"
     *   y=130: EMA value + " µS" unit (flex row, large font)
     *   y=215: Baseline label
     *   y=240: Threshold label
     *   y=265: Stressed/Normal state label
     *   y=290: Last update timestamp
     *   y=315: "View EMA Plot" button (200×60)
     */

    // Title
    lv_obj_t *label_title = lv_label_create(scr_gsr);
    lv_label_set_text(label_title, "GSR");
    lv_obj_set_pos(label_title, 0, 40);
    lv_obj_set_width(label_title, 390);
    lv_obj_add_style(label_title, &style_body_medium, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_title, lv_color_white(), LV_PART_MAIN);

    // EMA value container with inline "µS" unit
    lv_obj_t *cont_value = lv_obj_create(scr_gsr);
    lv_obj_remove_style_all(cont_value);
    lv_obj_set_size(cont_value, 390, 70);
    lv_obj_set_pos(cont_value, 0, 130);
    lv_obj_set_style_bg_opa(cont_value, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_flex_flow(cont_value, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_value, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);

    label_gsr_ema = lv_label_create(cont_value);
    lv_label_set_text(label_gsr_ema, "--");
    lv_obj_set_style_text_color(label_gsr_ema, lv_color_white(), LV_PART_MAIN);
    lv_obj_add_style(label_gsr_ema, &style_numeric_large, LV_PART_MAIN);

    lv_obj_t *label_gsr_unit = lv_label_create(cont_value);
    lv_label_set_text(label_gsr_unit, " uS");
    lv_obj_set_style_text_color(label_gsr_unit, lv_color_hex(COLOR_GSR_TEAL), LV_PART_MAIN);
    lv_obj_add_style(label_gsr_unit, &style_body_medium, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(label_gsr_unit, 8, LV_PART_MAIN);

    // Baseline label
    label_gsr_baseline = lv_label_create(scr_gsr);
    lv_label_set_text(label_gsr_baseline, "Baseline: --");
    lv_obj_set_pos(label_gsr_baseline, 0, 215);
    lv_obj_set_width(label_gsr_baseline, 390);
    lv_obj_set_style_text_color(label_gsr_baseline, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);
    lv_obj_add_style(label_gsr_baseline, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_gsr_baseline, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    // Threshold label
    label_gsr_threshold = lv_label_create(scr_gsr);
    lv_label_set_text(label_gsr_threshold, "Threshold: --");
    lv_obj_set_pos(label_gsr_threshold, 0, 240);
    lv_obj_set_width(label_gsr_threshold, 390);
    lv_obj_set_style_text_color(label_gsr_threshold, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);
    lv_obj_add_style(label_gsr_threshold, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_gsr_threshold, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    // Stressed/Normal state label
    label_gsr_stress_state = lv_label_create(scr_gsr);
    lv_label_set_text(label_gsr_stress_state, "");
    lv_obj_set_pos(label_gsr_stress_state, 0, 265);
    lv_obj_set_width(label_gsr_stress_state, 390);
    lv_obj_add_style(label_gsr_stress_state, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_gsr_stress_state, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    // Last update
    label_gsr_last_update = lv_label_create(scr_gsr);
    lv_label_set_text(label_gsr_last_update, "Waiting for data...");
    lv_obj_set_pos(label_gsr_last_update, 0, 290);
    lv_obj_set_width(label_gsr_last_update, 390);
    lv_obj_set_style_text_color(label_gsr_last_update, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);
    lv_obj_add_style(label_gsr_last_update, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_gsr_last_update, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    hpi_disp_set_curr_screen(SCR_GSR);
    hpi_show_screen(scr_gsr, m_scroll_dir);
}

void hpi_disp_gsr_update(int32_t ema, int32_t baseline, int32_t threshold, bool stressed)
{
    if (label_gsr_ema == NULL) {
        return;
    }

    /* EMA, baseline, threshold are in µS × 100 — display as X.XX */
    lv_label_set_text_fmt(label_gsr_ema, "%d.%02d",
                           ema / 100, (ema < 0 ? -ema : ema) % 100);

    if (label_gsr_baseline != NULL) {
        lv_label_set_text_fmt(label_gsr_baseline, "Baseline: %d.%02d uS",
                               baseline / 100, (baseline < 0 ? -baseline : baseline) % 100);
    }

    if (label_gsr_threshold != NULL) {
        lv_label_set_text_fmt(label_gsr_threshold, "Threshold: %d.%02d uS",
                               threshold / 100, (threshold < 0 ? -threshold : threshold) % 100);
    }

    if (label_gsr_stress_state != NULL) {
        if (stressed) {
            lv_label_set_text(label_gsr_stress_state, "Stressed");
            lv_obj_set_style_text_color(label_gsr_stress_state,
                                         lv_color_hex(COLOR_CRITICAL_RED), LV_PART_MAIN);
        } else {
            lv_label_set_text(label_gsr_stress_state, "Normal");
            lv_obj_set_style_text_color(label_gsr_stress_state,
                                         lv_color_hex(COLOR_SUCCESS_GREEN), LV_PART_MAIN);
        }
    }

    if (label_gsr_last_update != NULL) {
        lv_label_set_text(label_gsr_last_update, "Live");
    }
}
