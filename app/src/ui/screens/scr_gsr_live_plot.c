/*
 * HealthyPi Move — GSR Live EMA Plot Screen
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Protocentral Electronics
 *
 * Displays the last 60 GSR EMA samples (µS × 100) as a live-updating line chart.
 * A horizontal line at the stress threshold is drawn using a separate chart series.
 * Back swipe returns to SCR_GSR.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>
#include <stdio.h>

#include "hpi_common_types.h"
#include "ui/move_ui.h"

LOG_MODULE_REGISTER(hpi_disp_scr_gsr_live_plot, LOG_LEVEL_ERR);

static lv_obj_t *scr_gsr_live_plot;
static lv_obj_t *chart_gsr;
static lv_chart_series_t *ser_gsr;
static lv_chart_series_t *ser_threshold;
static lv_obj_t *label_gsr_plot_info;

extern lv_style_t style_body_medium;
extern lv_style_t style_caption;

#define COLOR_GSR_TEAL  0x00897B

void gesture_down_scr_gsr_live_plot(void)
{
    hpi_load_screen(SCR_GSR, SCROLL_DOWN);
}

void draw_scr_gsr_live_plot(enum scroll_dir m_scroll_dir,
                             uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
    ARG_UNUSED(arg1); ARG_UNUSED(arg2); ARG_UNUSED(arg3); ARG_UNUSED(arg4);

    scr_gsr_live_plot = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_gsr_live_plot, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_clear_flag(scr_gsr_live_plot, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *label_title = lv_label_create(scr_gsr_live_plot);
    lv_label_set_text(label_title, "GSR — Live EMA");
    lv_obj_set_pos(label_title, 0, 35);
    lv_obj_set_width(label_title, 390);
    lv_obj_add_style(label_title, &style_body_medium, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_title, lv_color_white(), LV_PART_MAIN);

    // Info label (EMA / threshold values)
    label_gsr_plot_info = lv_label_create(scr_gsr_live_plot);
    lv_label_set_text(label_gsr_plot_info, "EMA: --  Threshold: --");
    lv_obj_set_pos(label_gsr_plot_info, 0, 60);
    lv_obj_set_width(label_gsr_plot_info, 390);
    lv_obj_add_style(label_gsr_plot_info, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_gsr_plot_info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_gsr_plot_info, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);

    // Back hint
    lv_obj_t *label_back = lv_label_create(scr_gsr_live_plot);
    lv_label_set_text(label_back, LV_SYMBOL_DOWN " Swipe down to go back");
    lv_obj_set_pos(label_back, 0, 355);
    lv_obj_set_width(label_back, 390);
    lv_obj_add_style(label_back, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_back, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_back, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);

    // Chart
    chart_gsr = lv_chart_create(scr_gsr_live_plot);
    lv_obj_set_size(chart_gsr, 360, 250);
    lv_obj_set_pos(chart_gsr, 15, 85);
    lv_chart_set_type(chart_gsr, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart_gsr, 60);
    /* Y range will auto-expand when data arrives; start with a sensible default */
    lv_chart_set_range(chart_gsr, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);
    lv_chart_set_div_line_count(chart_gsr, 4, 0);

    lv_obj_set_style_bg_color(chart_gsr, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(chart_gsr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(chart_gsr, 0, LV_PART_MAIN);
    lv_obj_set_style_line_color(chart_gsr, lv_color_hex(0x333333), LV_PART_MAIN);

    ser_gsr = lv_chart_add_series(chart_gsr, lv_color_hex(COLOR_GSR_TEAL),
                                   LV_CHART_AXIS_PRIMARY_Y);
    ser_threshold = lv_chart_add_series(chart_gsr, lv_color_hex(COLOR_CRITICAL_RED),
                                         LV_CHART_AXIS_PRIMARY_Y);

    lv_chart_set_all_value(chart_gsr, ser_gsr, LV_CHART_POINT_NONE);
    lv_chart_set_all_value(chart_gsr, ser_threshold, LV_CHART_POINT_NONE);

    lv_obj_set_style_size(chart_gsr, 0, 0, LV_PART_INDICATOR);

    hpi_disp_set_curr_screen(SCR_SPL_GSR_LIVE_PLOT);
    hpi_show_screen(scr_gsr_live_plot, m_scroll_dir);
}

void hpi_disp_gsr_live_plot_update(const int32_t *ema_buf, uint8_t buf_count, int32_t threshold)
{
    if (chart_gsr == NULL || ser_gsr == NULL) {
        return;
    }

    if (buf_count == 0) {
        return;
    }

    /* Find y-axis range from data + threshold */
    int32_t y_min = ema_buf[0];
    int32_t y_max = ema_buf[0];

    for (uint8_t i = 1; i < buf_count; i++) {
        if (ema_buf[i] < y_min) y_min = ema_buf[i];
        if (ema_buf[i] > y_max) y_max = ema_buf[i];
    }
    if (threshold > y_max) y_max = threshold;
    if (threshold < y_min) y_min = threshold;

    /* Add 10% headroom */
    int32_t margin = (y_max - y_min) / 10;
    if (margin < 10) margin = 10;
    y_min -= margin;
    y_max += margin;
    if (y_min < 0) y_min = 0;

    lv_chart_set_range(chart_gsr, LV_CHART_AXIS_PRIMARY_Y, (lv_coord_t)y_min, (lv_coord_t)y_max);

    /* Repopulate EMA series */
    lv_chart_set_all_value(chart_gsr, ser_gsr, LV_CHART_POINT_NONE);
    for (uint8_t i = 0; i < buf_count; i++) {
        lv_chart_set_next_value(chart_gsr, ser_gsr, (lv_coord_t)ema_buf[i]);
    }

    /* Fill threshold series with flat line at threshold value */
    lv_chart_set_all_value(chart_gsr, ser_threshold, (lv_coord_t)threshold);

    lv_chart_refresh(chart_gsr);

    /* Update info label */
    if (label_gsr_plot_info != NULL) {
        int32_t last_ema = ema_buf[buf_count - 1];
        lv_label_set_text_fmt(label_gsr_plot_info, "EMA: %d.%02d  Thr: %d.%02d uS",
                               last_ema / 100, (last_ema < 0 ? -last_ema : last_ema) % 100,
                               threshold / 100, (threshold < 0 ? -threshold : threshold) % 100);
    }
}
