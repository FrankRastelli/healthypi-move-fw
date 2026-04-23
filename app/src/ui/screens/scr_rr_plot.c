/*
 * HealthyPi Move — RR Interval History Plot Screen
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025 Protocentral Electronics
 *
 * Displays the last 60 RR intervals (ms) as a live-updating line chart.
 * Y-axis: 300–1200 ms (physiological bounds).
 * Back swipe returns to SCR_HRV.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>
#include <stdio.h>

#include "hpi_common_types.h"
#include "ui/move_ui.h"

LOG_MODULE_REGISTER(hpi_disp_scr_rr_plot, LOG_LEVEL_ERR);

#define RR_Y_MIN  300
#define RR_Y_MAX  1200

static lv_obj_t *scr_rr_plot;
static lv_obj_t *chart_rr;
static lv_chart_series_t *ser_rr;

extern lv_style_t style_body_medium;
extern lv_style_t style_caption;

void gesture_down_scr_rr_plot(void)
{
    hpi_load_screen(SCR_HRV, SCROLL_DOWN);
}

void draw_scr_rr_plot(enum scroll_dir m_scroll_dir,
                      uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
    ARG_UNUSED(arg1); ARG_UNUSED(arg2); ARG_UNUSED(arg3); ARG_UNUSED(arg4);

    scr_rr_plot = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_rr_plot, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_clear_flag(scr_rr_plot, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *label_title = lv_label_create(scr_rr_plot);
    lv_label_set_text(label_title, "RR Intervals");
    lv_obj_set_pos(label_title, 0, 35);
    lv_obj_set_width(label_title, 390);
    lv_obj_add_style(label_title, &style_body_medium, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_title, lv_color_white(), LV_PART_MAIN);

    // Y-axis range label
    lv_obj_t *label_range = lv_label_create(scr_rr_plot);
    lv_label_set_text(label_range, "300–1200 ms");
    lv_obj_set_pos(label_range, 0, 60);
    lv_obj_set_width(label_range, 390);
    lv_obj_add_style(label_range, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_range, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_range, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);

    // Back hint
    lv_obj_t *label_back = lv_label_create(scr_rr_plot);
    lv_label_set_text(label_back, LV_SYMBOL_DOWN " Swipe down to go back");
    lv_obj_set_pos(label_back, 0, 355);
    lv_obj_set_width(label_back, 390);
    lv_obj_add_style(label_back, &style_caption, LV_PART_MAIN);
    lv_obj_set_style_text_align(label_back, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_back, lv_color_hex(COLOR_TEXT_SECONDARY), LV_PART_MAIN);

    // Chart
    chart_rr = lv_chart_create(scr_rr_plot);
    lv_obj_set_size(chart_rr, 360, 250);
    lv_obj_set_pos(chart_rr, 15, 85);
    lv_chart_set_type(chart_rr, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart_rr, 60);
    lv_chart_set_range(chart_rr, LV_CHART_AXIS_PRIMARY_Y, RR_Y_MIN, RR_Y_MAX);
    lv_chart_set_div_line_count(chart_rr, 4, 0);

    lv_obj_set_style_bg_color(chart_rr, lv_color_hex(0x111111), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(chart_rr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(chart_rr, 0, LV_PART_MAIN);
    lv_obj_set_style_line_color(chart_rr, lv_color_hex(0x333333), LV_PART_MAIN);

    ser_rr = lv_chart_add_series(chart_rr, lv_color_hex(0x8000FF), LV_CHART_AXIS_PRIMARY_Y);

    /* Initialise all points to LV_CHART_POINT_NONE so the chart starts empty */
    lv_chart_set_all_value(chart_rr, ser_rr, LV_CHART_POINT_NONE);

    lv_obj_set_style_size(chart_rr, 0, 0, LV_PART_INDICATOR);

    hpi_disp_set_curr_screen(SCR_SPL_RR_PLOT);
    hpi_show_screen(scr_rr_plot, m_scroll_dir);
}

void hpi_disp_rr_plot_update(const uint16_t *rr_buf, uint8_t rr_count)
{
    if (chart_rr == NULL || ser_rr == NULL) {
        return;
    }

    /* Repopulate chart from the full history buffer */
    lv_chart_set_all_value(chart_rr, ser_rr, LV_CHART_POINT_NONE);

    for (uint8_t i = 0; i < rr_count; i++) {
        lv_chart_set_next_value(chart_rr, ser_rr, (lv_coord_t)rr_buf[i]);
    }

    lv_chart_refresh(chart_rr);
}
