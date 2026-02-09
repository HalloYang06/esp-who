#include "who_lvgl_lcd.hpp"
#include "esp_lcd_panel_ops.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LVGL";

#if !BSP_CONFIG_NO_GRAPHIC_LIB

namespace who {
namespace lcd {

static lv_disp_draw_buf_t disp_buf;    // LVGL显示缓冲区
static lv_disp_drv_t disp_drv;          // LVGL显示驱动
static lv_color_t *buf1 = NULL;         // 缓冲区1
static lv_color_t *buf2 = NULL;         // 缓冲区2（双缓冲）
static uint32_t flush_count = 0;        // 刷新计数器

// LVGL tick回调（提供时间基准）
static void lv_tick_task(void *arg)
{
    (void) arg;
    lv_tick_inc(10);  // 每10ms增加一次tick
}

// LVGL刷新回调（将数据发送到LCD）
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsety1 = area->y1;
    int offsetx2 = area->x2;
    int offsety2 = area->y2;
    int width = offsetx2 - offsetx1 + 1;
    int height = offsety2 - offsety1 + 1;

    // 每50次刷新打印一次日志，避免日志过多
    if (flush_count % 50 == 0) {
        ESP_LOGI(TAG, "LVGL刷新LCD区域: [%d,%d]->[%d,%d] (%dx%d), color_map=%p",
                 offsetx1, offsety1, offsetx2, offsety2, width, height, color_map);
    }
    flush_count++;

    // 发送像素数据到LCD
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

// LVGL任务（处理UI更新）
static void lvgl_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();  // 处理LVGL任务 (LVGL 8.x API)
    }
}

#if CONFIG_IDF_TARGET_ESP32S3
void WhoLCD::init()
{
    ESP_LOGI(TAG, "Starting LVGL initialization...");

    // 1. 初始化LVGL库
    lv_init();
    ESP_LOGI(TAG, "lv_init() completed");

    // 2. 创建LVGL tick定时器（10ms周期）
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lv_tick_task,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 10 * 1000));  // 10ms
    ESP_LOGI(TAG, "LVGL tick timer created");

    // 3. 初始化LCD硬件
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(bsp_display_new(&bsp_disp_cfg, &m_panel_handle, &m_io_handle));
    esp_lcd_panel_disp_on_off(m_panel_handle, true);
    ESP_LOGI(TAG, "LCD hardware initialized");

    // 4. 分配LVGL显示缓冲区
    size_t buf_size = BSP_LCD_DRAW_BUFF_SIZE * sizeof(lv_color_t);
    buf1 = (lv_color_t*) heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf1 != NULL);

#if BSP_LCD_DRAW_BUFF_DOUBLE
    buf2 = (lv_color_t*) heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(buf2 != NULL);
    ESP_LOGI(TAG, "Buffers allocated: %d bytes each (double buffer)", buf_size);
#else
    ESP_LOGI(TAG, "Buffer allocated: %d bytes (single buffer)", buf_size);
#endif

    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, BSP_LCD_DRAW_BUFF_SIZE);

    // 5. 注册显示驱动
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = m_panel_handle;
    m_disp = lv_disp_drv_register(&disp_drv);
    ESP_LOGI(TAG, "Display driver registered (%dx%d)", BSP_LCD_H_RES, BSP_LCD_V_RES);

    // 6. 创建LVGL任务
    xTaskCreate(lvgl_task, "lvgl", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "LVGL task created");

    // 7. 打开背光
    ESP_ERROR_CHECK(bsp_display_backlight_on());
    ESP_LOGI(TAG, "LVGL initialization completed!");

    // 设置屏幕背景为黑色
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    ESP_LOGI(TAG, "Screen background set to black");
}

void WhoLCD::deinit()
{
    // 释放缓冲区
    if (buf1) heap_caps_free(buf1);
    if (buf2) heap_caps_free(buf2);
    // TODO: 更完整的清理
}

#elif CONFIG_IDF_TARGET_ESP32P4
#error "ESP32P4 support not yet implemented without esp_lvgl_port"
#endif

} // namespace lcd
} // namespace who
#endif
