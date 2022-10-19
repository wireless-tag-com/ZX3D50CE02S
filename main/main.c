#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/soc_caps.h"
#include "soc/gdma_reg.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lvgl.h"
#include "i2c_device.h"
#include "touch_panel.h"
#include "screen_driver.h"
#include "board.h"


#define TAG "MAIN"

static lv_disp_drv_t disp_drv;     
static scr_driver_t g_lcd;
static touch_panel_driver_t g_touch;

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

static void lvgl_touchpad_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    data->state = LV_INDEV_STATE_REL;
    touch_panel_points_t points;
    g_touch.read_point_data(&points);

    // please be sure that your touch driver every time return old (last clcked) value.
    if (TOUCH_EVT_PRESS == points.event) {
        int32_t x = points.curx[0];
        int32_t y = points.cury[0];
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PR;
    }
}

static void lvgl_touch_init(void)
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TP_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = TP_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_bus_handle_t i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);

    touch_panel_config_t touch_cfg = {
        .interface_i2c = {
            .i2c_bus = i2c_bus,
            .clk_freq = 100000,
            .i2c_addr = 0x38,
        },
        .interface_type = TOUCH_PANEL_IFACE_I2C,
        .pin_num_int = -1,
        .direction = TOUCH_DIR_BTLR,
        .width = 320,
        .height = 480,
    };
    touch_panel_find_driver(TOUCH_PANEL_CONTROLLER_FT5X06, &g_touch);
    g_touch.init(&touch_cfg);
}

static void lvgl_drv_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    g_lcd.draw_bitmap(area->x1, area->y1, (uint16_t)(area->x2 - area->x1 + 1), (uint16_t)(area->y2 - area->y1 + 1), (uint16_t *)color_map);
    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(drv);
}

void lvgl_task(void* arg) {
    lv_init();

    static lv_disp_draw_buf_t draw_buf;

    lv_color_t *buf1;
    lv_color_t *buf2;
    buf1 = heap_caps_aligned_calloc(64, 1, LCD_WIDTH * LCD_HIGHT * 2, MALLOC_CAP_SPIRAM);
    buf2 = heap_caps_aligned_calloc(64, 1, LCD_WIDTH * LCD_HIGHT * 2, MALLOC_CAP_SPIRAM);
    
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_WIDTH * LCD_HIGHT);

    lv_disp_drv_init(&disp_drv);         
    disp_drv.flush_cb = lvgl_drv_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.hor_res = 480;
    disp_drv.ver_res = 320;
    lv_disp_drv_register(&disp_drv);

    lvgl_touch_init();
    static lv_indev_drv_t indev_drv;           /*Descriptor of a input device driver*/
    lv_indev_drv_init(&indev_drv);             /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = lvgl_touchpad_read;      /*Set your driver function*/
    lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/
   
    // Tick interface for LVGL
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = increase_lvgl_tick,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, portTICK_PERIOD_MS * 1000);

    extern void lv_demo_widgets(void);
    lv_demo_widgets();

    gpio_set_level(LCD_BL_PIN, 1);

    for (;;) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    extern void screen_init(scr_driver_t* s_lcd, SemaphoreHandle_t semaphore);
    screen_init(&g_lcd, NULL);

    xTaskCreatePinnedToCore(lvgl_task, NULL, 8 * 1024, NULL, 5, NULL, 1);
}
