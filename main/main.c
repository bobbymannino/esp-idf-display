#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_sh1107.h"

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_CLK_SPEED_HZ (400 * 1000)
#define LCD_I2C_ADDR 0x3C

#define LCD_H_RES 128
#define LCD_V_RES 128
#define LCD_BUFFER_SIZE (LCD_H_RES * LCD_V_RES / 8)

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C Bus");
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    ESP_LOGI(TAG, "Initializing Panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = LCD_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_SPEED_HZ,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 0,
        .flags = {
            .disable_control_phase = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(bus_handle, &io_config, &io_handle));

    ESP_LOGI(TAG, "Initializing Panel Driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));

    ESP_LOGI(TAG, "Configuring Display");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    /* SH1107 typically needs color inversion */
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    ESP_LOGI(TAG, "Filling screen with white pixels");

    /* Allocate buffer for 1-bit display: width * height / 8 bytes */
    uint8_t *screen_buffer = heap_caps_malloc(LCD_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (screen_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate screen buffer");
        return;
    }

    /* Fill buffer with 0xFF (all pixels on for 1-bit display) */
    memset(screen_buffer, 0xFF, LCD_BUFFER_SIZE);

    /* Draw the entire screen */
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, screen_buffer));

    ESP_LOGI(TAG, "Display initialized successfully");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
