/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include <stdio.h>
#include "BoilerAutoFillUI.h"
#include "lvgl.h"

LV_FONT_DECLARE(font_msyh_20)
LV_FONT_DECLARE(font_sa_digital_number_64)

static float pressureLow = 1.5;
static float pressureHigh = 2.5;
static float pressure = 2.1;

void lvgl_clean_screen()
{
    lv_obj_clean(lv_scr_act());
    lv_refr_now(NULL);
}

static lv_obj_t *mainScreen = NULL;

void lvgl_baf_ui(lv_display_t *disp)
{
    mainScreen = lv_display_get_screen_active(disp);
    lv_obj_set_style_bg_color(mainScreen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(mainScreen, LV_OPA_COVER, 0);

    lv_obj_t* mainDiv = lv_obj_create(mainScreen);
    lv_obj_remove_style_all(mainDiv);
    lv_obj_set_style_bg_opa(mainDiv, LV_OPA_TRANSP, 0);
    lv_obj_set_size(mainDiv, LV_PCT(100), LV_PCT(100));
    lv_obj_center(mainDiv);
    lv_obj_set_flex_flow(mainDiv, LV_FLEX_FLOW_COLUMN);

    // Add a label for title and center it in mainDiv: "Boiler Pressure"
    lv_obj_t* titleLabel = lv_label_create(mainDiv);
    lv_label_set_long_mode(titleLabel, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(titleLabel, LV_PCT(100));
    lv_label_set_text(titleLabel, "水压 (Bar)");
    lv_obj_set_style_text_color(titleLabel, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_text_font(titleLabel, &font_msyh_20, 0);
    lv_obj_set_style_text_align(titleLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_margin_top(titleLabel, 6, 0);

    // Add a scale (meter) widget to display the pressure value
    lv_obj_t* scaleDiv = lv_obj_create(mainDiv);
    lv_obj_remove_style_all(scaleDiv);
    lv_obj_set_style_bg_opa(scaleDiv, LV_OPA_TRANSP, 0);
    lv_obj_set_size(scaleDiv, LV_PCT(100), 50);
    lv_obj_set_flex_flow(scaleDiv, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(scaleDiv, 10, 0);
    lv_obj_set_style_pad_ver(scaleDiv, 2, 0);

    lv_obj_t* scale = lv_scale_create(scaleDiv);
    lv_obj_set_size(scale, LV_PCT(100), 30);
    lv_scale_set_label_show(scale, true);
    lv_scale_set_mode(scale, LV_SCALE_MODE_HORIZONTAL_TOP);
    lv_obj_center(scale);

    lv_scale_set_total_tick_count(scale, 16);
    lv_scale_set_major_tick_every(scale, 5);

    lv_obj_set_style_length(scale, 10, LV_PART_INDICATOR);
    lv_obj_set_style_length(scale, 5, LV_PART_ITEMS);
    lv_scale_set_range(scale, 0, 300);

    static const char * custom_labels[] = {"0", "1", "2", "3", NULL};
    lv_scale_set_text_src(scale, custom_labels);

    static lv_style_t major_tick_style;
    lv_style_init(&major_tick_style);

    /* Label style properties */
    lv_style_set_text_font(&major_tick_style, LV_FONT_DEFAULT);
    lv_style_set_text_color(&major_tick_style, lv_palette_lighten(LV_PALETTE_GREY, 3));
    // Set label text size to 20% larger than default

    /* Major tick properties */
    lv_style_set_line_color(&major_tick_style, lv_palette_lighten(LV_PALETTE_GREY, 3));
    lv_style_set_width(&major_tick_style, 10U);          // Tick length
    lv_style_set_line_width(&major_tick_style, 4U);      // Tick width
    lv_obj_add_style(scale, &major_tick_style, LV_PART_INDICATOR);

    /* Minor tick properties */
    static lv_style_t minor_tick_style;
    lv_style_init(&minor_tick_style);
    lv_style_set_line_color(&minor_tick_style, lv_palette_lighten(LV_PALETTE_GREY, 1));
    lv_style_set_width(&minor_tick_style, 5U);         // Tick length
    lv_style_set_line_width(&minor_tick_style, 2U);    // Tick width
    lv_obj_add_style(scale, &minor_tick_style, LV_PART_ITEMS);

    /* Main line properties */
    static lv_style_t main_line_style;
    lv_style_init(&main_line_style);
    lv_style_set_line_color(&main_line_style, lv_palette_lighten(LV_PALETTE_GREY, 3));
    lv_style_set_line_width(&main_line_style, 2U);      // Tick width
    lv_obj_add_style(scale, &main_line_style, LV_PART_MAIN);

    /* Add a section */
    static lv_style_t section_major_tick_style;
    static lv_style_t section_minor_tick_style;
    static lv_style_t section_main_line_style;

    lv_style_init(&section_major_tick_style);
    lv_style_init(&section_minor_tick_style);
    lv_style_init(&section_main_line_style);

    /* Label style properties */
    // lv_style_set_text_font(&section_major_tick_style, LV_FONT_DEFAULT);
    // lv_style_set_text_color(&section_major_tick_style, lv_palette_lighten(LV_PALETTE_GREEN, 1));

    /* Major tick properties */
    lv_style_set_line_color(&section_major_tick_style, lv_palette_lighten(LV_PALETTE_GREEN, 1));
    lv_style_set_line_width(&section_major_tick_style, 4U);

    /* Minor tick properties */
    // lv_style_set_line_color(&section_minor_tick_style, lv_palette_darken(LV_PALETTE_GREEN, 2));
    // lv_style_set_line_width(&section_minor_tick_style, 2U);

    /* Main line properties */
    lv_style_set_line_color(&section_main_line_style, lv_palette_lighten(LV_PALETTE_GREEN, 1));
    lv_style_set_line_width(&section_main_line_style, 4U);

    /* Configure section styles */
    lv_scale_section_t * section = lv_scale_add_section(scale);
    lv_scale_set_section_range(scale, section, pressureLow*100, pressureHigh*100);
    lv_scale_set_section_style_indicator(scale, section, &section_major_tick_style);
    lv_scale_set_section_style_items(scale, section, &section_minor_tick_style);
    lv_scale_set_section_style_main(scale, section, &section_main_line_style);

    lv_obj_set_style_bg_color(scale, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_bg_opa(scale, LV_OPA_TRANSP, 0);

    lv_obj_t* needleSlider = lv_slider_create(scaleDiv);
    lv_obj_remove_style_all(needleSlider);
    lv_obj_clear_flag(needleSlider, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(needleSlider, LV_PCT(100), 4);
    lv_slider_set_range(needleSlider, 0, 300);
    lv_slider_set_value(needleSlider, pressure * 100, LV_ANIM_OFF);
    lv_obj_set_style_margin_left(needleSlider, 1, 0);
    lv_obj_set_style_bg_opa(needleSlider, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_opa(needleSlider, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(needleSlider, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_border_opa(needleSlider, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_width(needleSlider, 0, LV_PART_INDICATOR);
    lv_obj_set_style_height(needleSlider, LV_PCT(100), LV_PART_KNOB);
    lv_obj_set_style_width(needleSlider, 50, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(needleSlider, LV_OPA_COVER, LV_PART_KNOB);
    lv_obj_set_style_bg_color(needleSlider, lv_palette_main(LV_PALETTE_RED), LV_PART_KNOB);
    lv_obj_set_style_border_opa(needleSlider, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_set_style_margin_top(needleSlider, -3, 0);
    lv_obj_set_style_pad_all(needleSlider, 0, LV_PART_KNOB);
    lv_obj_set_style_pad_ver(needleSlider, 7, LV_PART_KNOB);

    lv_obj_t* pressureDiv = lv_obj_create(mainDiv);
    lv_obj_remove_style_all(pressureDiv);
    lv_obj_set_style_bg_opa(pressureDiv, LV_OPA_TRANSP, 0);
    lv_obj_set_width(pressureDiv, LV_PCT(100));
    lv_obj_set_flex_grow(pressureDiv, 1);
    lv_obj_set_style_pad_bottom(pressureDiv, 20, 0);

    lv_obj_t* pressureLabel = lv_label_create(pressureDiv);
    lv_label_set_long_mode(pressureLabel, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(pressureLabel, LV_PCT(100));
    lv_obj_set_style_text_color(pressureLabel, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_text_font(pressureLabel, &font_sa_digital_number_64, 0);
    lv_obj_set_style_text_align(pressureLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(pressureLabel);

    char pressure_text[10];
    snprintf(pressure_text, sizeof(pressure_text), "%.2f", pressure);
    lv_label_set_text(pressureLabel, pressure_text);

    lv_obj_t* buttonDiv = lv_obj_create(mainDiv);
    lv_obj_remove_style_all(buttonDiv);
    lv_obj_set_style_bg_opa(buttonDiv, LV_OPA_TRANSP, 0);
    lv_obj_set_size(buttonDiv, LV_PCT(100), 80);
    lv_obj_set_flex_flow(buttonDiv, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_all(buttonDiv, 10, 0);

    lv_obj_t* reliefButton = lv_btn_create(buttonDiv);
    lv_obj_set_height(reliefButton, LV_PCT(100));
    lv_obj_set_flex_grow(reliefButton, 1);
    lv_obj_set_style_bg_color(reliefButton, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_bg_opa(reliefButton, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(reliefButton, 5, 0);
    lv_obj_set_style_text_color(reliefButton, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_text_font(reliefButton, &font_msyh_20, 0);
    lv_obj_set_style_text_align(reliefButton, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t* reliefLabel = lv_label_create(reliefButton);
    lv_label_set_text(reliefLabel, "泄压");
    lv_obj_center(reliefLabel);

    // Add space between the two buttons
    lv_obj_t* spacer = lv_obj_create(buttonDiv);
    lv_obj_remove_style_all(spacer);
    lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0);
    lv_obj_set_width(spacer, 10);

    lv_obj_t* fillButton = lv_btn_create(buttonDiv);
    lv_obj_set_height(fillButton, LV_PCT(100));
    lv_obj_set_flex_grow(fillButton, 1);
    lv_obj_set_style_bg_color(fillButton, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_bg_opa(fillButton, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(fillButton, 5, 0);
    lv_obj_set_style_text_color(fillButton, lv_palette_lighten(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_style_text_font(fillButton, &font_msyh_20, 0);
    lv_obj_set_style_text_align(fillButton, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t* fillLabel = lv_label_create(fillButton);
    lv_label_set_text(fillLabel, "注水");
    lv_obj_center(fillLabel);
}
