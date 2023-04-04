#include <stdio.h>
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "soc/system_reg.h"
#include "screen_driver.h"
#include "board.h"
#include "driver/gpio.h"

void screen_init(scr_driver_t* s_lcd, SemaphoreHandle_t semaphore) {
#ifdef LCD_RD_PIN
    if (LCD_RD_PIN > -1) {
        gpio_pad_select_gpio(LCD_RD_PIN);
        gpio_set_level(LCD_RD_PIN, 1);
        gpio_set_direction(LCD_RD_PIN, GPIO_MODE_OUTPUT);
    }
#endif

    i2s_lcd_config_t i2s_lcd_cfg = {
        .data_width = 8,
        .pin_data_num = {
            LCD_D0_PIN,
            LCD_D1_PIN,
            LCD_D2_PIN,
            LCD_D3_PIN,
            LCD_D4_PIN,
            LCD_D5_PIN,
            LCD_D6_PIN,
            LCD_D7_PIN,
        },
        .pin_num_cs = LCD_CS_PIN,
        .pin_num_wr = LCD_WR_PIN,                          
        .pin_num_rs = LCD_RS_PIN,
        .clk_freq = 40000000,
        .i2s_port = I2S_NUM_0,
        .buffer_size = 16000,
        .swap_data = true,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_8080, &i2s_lcd_cfg, &iface_drv);
    extern scr_driver_t lcd_st7796_default_driver;

    *s_lcd = lcd_st7796_default_driver;

    scr_controller_config_t lcd_cfg = {
        .interface_drv = iface_drv,
        .pin_num_rst = LCD_RESET_PIN,
        .pin_num_bckl = -1,
        .rst_active_level = 0,
        .bckl_active_level = 1,
        .offset_hor = 0,
        .offset_ver = 0,
        .width = LCD_WIDTH,
        .height = LCD_HIGHT,
        .rotate = SCR_DIR_TBLR,
    };

    lcd_cfg.rotate = SCR_SWAP_XY;
    s_lcd->init(&lcd_cfg);

	gpio_pad_select_gpio(LCD_TE_PIN);
	gpio_set_direction(LCD_TE_PIN, GPIO_MODE_INPUT);

    if (LCD_BL_PIN > -1) {
        gpio_pad_select_gpio(LCD_BL_PIN);
        gpio_set_level(LCD_BL_PIN, 0);
        gpio_set_direction(LCD_BL_PIN, GPIO_MODE_OUTPUT);
    }
}
