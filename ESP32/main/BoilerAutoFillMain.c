/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_cst816s.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "lvgl.h"
#include "BoilerAutoFillUI.h"

static const char *TAG = "BAF";

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PH_LCD_PIXEL_CLOCK_HZ       (20 * 1000 * 1000)
#define PH_LCD_BK_LIGHT_ON_LEVEL    0
#define PH_LCD_BK_LIGHT_OFF_LEVEL   !PH_LCD_BK_LIGHT_ON_LEVEL
#define PH_PIN_NUM_SCLK             5
#define PH_PIN_NUM_MOSI             4
#define PH_PIN_NUM_MISO             19
#define PH_PIN_NUM_MISO             19
#define PH_PIN_NUM_SCL              8
#define PH_PIN_NUM_SDA              18
#define PH_PIN_NUM_LCD_DC           6
#define PH_PIN_NUM_LCD_RST          14
#define PH_PIN_NUM_LCD_CS           7
#define PH_PIN_NUM_BK_LIGHT         15
#define PH_PIN_NUM_TOUCH_RST        21
#define PH_PIN_NUM_TOUCH_INT        22
#define PH_PIN_NUM_PRESSURE_SENSOR  3

// ExtIO TCA9554 I2C parameters
#define PH_TCA9554_ADDR             0x20
#define PH_TCA9554_FREQ_HZ          400000
#define PH_TCA9554_TIMEOUT_MS       1000
#define PH_TCA9554_CMD_INPUT        0x00
#define PH_TCA9554_CMD_OUTPUT       0x01
#define PH_TCA9554_CMD_POLARITY     0x02
#define PH_TCA9554_CMD_CONFIG       0x03
#define PH_PIN_NUM_RELAY_IN         0       // Using TCA9554 P0 as relay control of valve input
#define PH_PIN_NUM_RELAY_OUT        1       // Using TCA9554 P1 as relay control of valve output

// The pixel number in horizontal and vertical
#define PH_LCD_H_RES                240
#define PH_LCD_V_RES                320
// Bit number used to represent command and parameter
#define PH_LCD_CMD_BITS             8
#define PH_LCD_PARAM_BITS           8

#define PH_LVGL_DRAW_BUF_LINES      20 // number of display lines in each draw buffer
#define PH_LVGL_TICK_PERIOD_MS      2
#define PH_LVGL_TASK_MAX_DELAY_MS   500
#define PH_LVGL_TASK_MIN_DELAY_MS   1000 / CONFIG_FREERTOS_HZ
#define PH_LVGL_TASK_STACK_SIZE     (4 * 1024)
#define PH_LVGL_TASK_PRIORITY       2

i2c_master_dev_handle_t tca9554_dev;
void extio_init_i2c_dev(i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PH_TCA9554_ADDR,
        .scl_speed_hz = PH_TCA9554_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &tca9554_dev));

    uint8_t cmd_data[2] = {0};

    // Set all output low
    cmd_data[0] = PH_TCA9554_CMD_OUTPUT;
    cmd_data[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(tca9554_dev, cmd_data, sizeof(cmd_data), PH_TCA9554_TIMEOUT_MS));
    // Set all pins as output
    cmd_data[0] = PH_TCA9554_CMD_CONFIG;
    cmd_data[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(tca9554_dev, cmd_data, sizeof(cmd_data), PH_TCA9554_TIMEOUT_MS));
}

void extio_set_pin_level(uint8_t pin, uint8_t level) {
    assert(pin < 8);

    uint8_t cmd_data[2] = {0};
    cmd_data[0] = PH_TCA9554_CMD_OUTPUT;

    // Read current output state
    ESP_ERROR_CHECK(i2c_master_transmit_receive(tca9554_dev, cmd_data, 1, cmd_data+1, 1, PH_TCA9554_TIMEOUT_MS));

    if (level) {
        cmd_data[1] |= (0x1U << pin);
    } else {
        cmd_data[1] &= ~(0x1U << pin);
    }

    // Write back new output state
    ESP_ERROR_CHECK(i2c_master_transmit(tca9554_dev, cmd_data, sizeof(cmd_data), PH_TCA9554_TIMEOUT_MS));
}

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

static bool lvgl_notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
    esp_lcd_touch_read_data(touch_pad);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGI(TAG, "Touch pressed at x[%d] y[%d]", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void lvgl_increase_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(PH_LVGL_TICK_PERIOD_MS);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, PH_LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, PH_LVGL_TASK_MAX_DELAY_MS);
        usleep(1000 * time_till_next_ms);
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PH_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(PH_PIN_NUM_BK_LIGHT, PH_LCD_BK_LIGHT_OFF_LEVEL);

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PH_PIN_NUM_SCLK,
        .mosi_io_num = PH_PIN_NUM_MOSI,
        .miso_io_num = PH_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PH_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PH_PIN_NUM_SDA,
        .scl_io_num = PH_PIN_NUM_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    ESP_LOGI(TAG, "Initialize TCA9554 IO Expander");
    extio_init_i2c_dev(bus_handle);

    ESP_LOGI(TAG, "Initialize ADC");
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PH_PIN_NUM_PRESSURE_SENSOR, &config));

    // Calibration
    adc_cali_handle_t adc1_cali_ps_handle = NULL;
    bool adc_calibration1 = adc_calibration_init(ADC_UNIT_1, PH_PIN_NUM_PRESSURE_SENSOR, config.atten, &adc1_cali_ps_handle);
    if (adc_calibration1) {
        ESP_LOGI(TAG, "ADC1 channel %d calibration success", PH_PIN_NUM_PRESSURE_SENSOR);
    } else {
        ESP_LOGW(TAG, "ADC1 channel %d calibration failed", PH_PIN_NUM_PRESSURE_SENSOR);
    }

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PH_PIN_NUM_LCD_DC,
        .cs_gpio_num = PH_PIN_NUM_LCD_CS,
        .pclk_hz = PH_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = PH_LCD_CMD_BITS,
        .lcd_param_bits = PH_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PH_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ST7789 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(PH_LCD_H_RES, PH_LCD_V_RES);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = PH_LCD_H_RES * PH_LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_increase_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, PH_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = lvgl_notify_flush_ready,
    };
    /* Register done callback */
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    // Attach the TOUCH to the I2C bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus_handle, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = PH_LCD_H_RES,
        .y_max = PH_LCD_V_RES,
        .rst_gpio_num = PH_PIN_NUM_TOUCH_RST,
        .int_gpio_num = PH_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_touch_handle_t touch_pad = NULL;

    ESP_LOGI(TAG, "Initialize touch controller CST816S");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_pad));

    static lv_indev_t *indev;
    indev = lv_indev_create(); // Input device driver (Touch)
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, touch_pad);
    lv_indev_set_read_cb(indev, lvgl_touch_cb);

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(lvgl_port_task, "LVGL", PH_LVGL_TASK_STACK_SIZE, NULL, PH_LVGL_TASK_PRIORITY, NULL);

    // Clear screen to fix the garbage display issue during initialization
    _lock_acquire(&lvgl_api_lock);
    lvgl_clean_screen();
    _lock_release(&lvgl_api_lock);

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(PH_PIN_NUM_BK_LIGHT, PH_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    lvgl_baf_ui(display);
    _lock_release(&lvgl_api_lock);

    int level = 0;
    while (true) {
        extio_set_pin_level(PH_PIN_NUM_RELAY_IN, level);
        extio_set_pin_level(PH_PIN_NUM_RELAY_OUT, level);
        ESP_LOGI(TAG, "ExtIO%d level: %d", PH_PIN_NUM_RELAY_IN, level);
        ESP_LOGI(TAG, "ExtIO%d level: %d", PH_PIN_NUM_RELAY_OUT, level);
        level = !level;

        int raw;
        int voltage;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PH_PIN_NUM_PRESSURE_SENSOR, &raw));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, PH_PIN_NUM_PRESSURE_SENSOR, raw);
        if (adc_calibration1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_ps_handle, raw, &voltage));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, PH_PIN_NUM_PRESSURE_SENSOR, voltage);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
