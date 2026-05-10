/*
 * HealthyPi Move HRV Screen
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Protocentral Electronics
 *
 * Live RMSSD display updated via haptic_hrv_chan zbus channel.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <lvgl.h>
#include <stdio.h>

#include "hpi_common_types.h"
#include "ui/move_ui.h"

LOG_MODULE_REGISTER(hpi_disp_scr_hrv, LOG_LEVEL_ERR);

// GUI Objects
lv_obj_t *scr_hrv;
static lv_obj_t *label_hrv_rmssd;
static lv_obj_t *label_hrv_baseline;
static lv_obj_t *label_hrv_threshold;
static lv_obj_t *label_hrv_stress_state;
static lv_obj_t *label_hrv_last_update;

// Externs - Modern style system
extern lv_style_t style_body_medium;
extern lv_style_t style_numeric_large;
extern lv_style_t style_caption;

#define COLOR_HRV_PURPLE  0x8000FF

void draw_scr_hrv(enum scroll_dir m_scroll_dir, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
    ARG_UNUSED(arg1); ARG_UNUSED(arg2); ARG_UNUSED(arg3); ARG_UNUSED(arg4);

    scr_hrv = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_hrv, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(scr_hrv, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *label_title = lv_label_create(scr_hrv);
    lv_label_set_text(label_title, "HRV");
    lv_obj_set_pos(label_title, 0, 40);
    lv_obj_set_width(label_title, 390);
    lv_obj_add_style(label_title, &style_body_medium, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_title, lv_color_white(), LV_PART_MAIN);

    // Heart icon (purple)
    lv_obj_t *img_hrv = lv_img_create(scr_hrv);
    lv_img_set_src(img_hrv, &img_heart_48px);
    lv_obj_set_pos(img_hrv, (390 - 48) / 2, 75);
    lv_obj_set_style_img_recolor(img_hrv, lv_color_hex(COLOR_HRV_PURPLE), LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(img_hrv, LV_OPA_COVER, LV_PART_MAIN);

    // RMSSD value container with inline "ms" unit
    lv_obj_t *cont_value = lv_obj_create(scr_hrv);
    lv_obj_remove_style_all(cont_value);
    lv_obj_set_size(cont_value, 390, 70);
    lv_obj_set_pos(cont_value, 0, 130);
    lv_obj_set_style_bg_opa(cont_value, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_flex_flow(cont_value, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(cont_value, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);

    label_hrv_rmssd = lv_label_create(cont_value);
    lv_label_set_text(label_hrv_rmssd, "--");
    lv_obj_set_style_text_color(label_hrv_rmssd, lv_color_white(), LV_PART_MAIN);
    lv_obj_add_style(label_hrv_rmssd, &style_numeric_large, LV_PART_MAIN);

    lv_obj_t *label_hrv_unit = lv_label_create(cont_value);
    lv_label_set_text(label_hrv_unit, " ms");
    lv_obj_set_style_text_color(label_hrv_unit, lv_color_hex(COLOR_HRV_PURPLE), LV_PART_MAIN);
    lv_obj_add_style(label_hrv_unit, &style_body_medium, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(label_hrv_unit, 8, LV_PART_MAIN);

    // Baseline label
    label_hrv_baseline = lv_label_create(scr_hrv);
    lv_label_set_text(label_hrv_baseline, "Baseline: -- ms");
    lv_obj_set_pos(label_hrv_baseline, 0, 215);
    lv_obj_set_width(label_hrv_baseline, 390);
    lv_obj_set_style_text_color(label_hrv_baseline, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);
    lv_obj_add_style(label_hrv_baseline, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_hrv_baseline, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    // Threshold label
    label_hrv_threshold = lv_label_create(scr_hrv);
    lv_label_set_text(label_hrv_threshold, "Threshold: -- ms");
    lv_obj_set_pos(label_hrv_threshold, 0, 240);
    lv_obj_set_width(label_hrv_threshold, 390);
    lv_obj_set_style_text_color(label_hrv_threshold, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);
    lv_obj_add_style(label_hrv_threshold, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_hrv_threshold, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    // Stressed/Normal state label
    label_hrv_stress_state = lv_label_create(scr_hrv);
    lv_label_set_text(label_hrv_stress_state, "");
    lv_obj_set_pos(label_hrv_stress_state, 0, 270);
    lv_obj_set_width(label_hrv_stress_state, 390);
    lv_obj_add_style(label_hrv_stress_state, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_hrv_stress_state, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    // Last update
    label_hrv_last_update = lv_label_create(scr_hrv);
    lv_label_set_text(label_hrv_last_update, "Waiting for data...");
    lv_obj_set_pos(label_hrv_last_update, 0, 300);
    lv_obj_set_width(label_hrv_last_update, 390);
    lv_obj_set_style_text_color(label_hrv_last_update, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);
    lv_obj_add_style(label_hrv_last_update, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_hrv_last_update, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    hpi_disp_set_curr_screen(SCR_HRV);
    hpi_show_screen(scr_hrv, m_scroll_dir);
}

void hpi_disp_hrv_update(uint32_t rmssd, int32_t baseline, int32_t threshold, bool stressed)
{
    if (label_hrv_rmssd == NULL) {
        return;
    }

    lv_label_set_text_fmt(label_hrv_rmssd, "%u", rmssd);

    if (label_hrv_baseline != NULL) {
        lv_label_set_text_fmt(label_hrv_baseline, "Baseline: %d ms", baseline);
    }

    if (label_hrv_threshold != NULL) {
        lv_label_set_text_fmt(label_hrv_threshold, "Threshold: %d ms", threshold);
    }

    if (label_hrv_stress_state != NULL) {
        if (stressed) {
            lv_label_set_text(label_hrv_stress_state, "Stressed");
            lv_obj_set_style_text_color(label_hrv_stress_state,
                                         lv_color_hex(COLOR_CRITICAL_RED), LV_PART_MAIN);
        } else {
            lv_label_set_text(label_hrv_stress_state, "Normal");
            lv_obj_set_style_text_color(label_hrv_stress_state,
                                         lv_color_hex(COLOR_SUCCESS_GREEN), LV_PART_MAIN);
        }
    }

    if (label_hrv_last_update != NULL) {
        lv_label_set_text(label_hrv_last_update, "Live");
    }
}
