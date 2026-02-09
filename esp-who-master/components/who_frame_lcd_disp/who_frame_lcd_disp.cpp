#include "who_frame_lcd_disp.hpp"
#include "esp_log.h"
#include "bsp/esp-bsp.h"  // 包含 BSP 头文件以访问 panel_handle
#include "esp_lcd_panel_ops.h"  // LCD panel 操作API
#include "bsp_lcd.h"  // 使用BSP的分块传输函数

// C语言全局变量声明（在文件顶部）
extern "C" {
    extern esp_lcd_panel_handle_t panel_handle;
}

static const char *TAG = "FrameLCDDisp";

using namespace who::lcd;

namespace who {
namespace lcd_disp {
WhoFrameLCDDisp::WhoFrameLCDDisp(const std::string &name, frame_cap::WhoFrameCapNode *frame_cap_node, int peek_index) :
    task::WhoTask(name), m_lcd(new lcd::WhoLCD()), m_frame_cap_node(frame_cap_node), m_peek_index(peek_index)
{
    ESP_LOGI(TAG, "Creating WhoFrameLCDDisp...");
    frame_cap_node->add_new_frame_signal_subscriber(this);
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    bsp_display_lock(0);

    // 创建Canvas用于绘制检测框（透明背景）
    m_canvas = lv_canvas_create(lv_scr_act());
    ESP_LOGI(TAG, "Canvas created: %p", m_canvas);
    lv_obj_set_size(m_canvas, frame_cap_node->get_fb_width(), frame_cap_node->get_fb_height());
    lv_obj_align(m_canvas, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(m_canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(m_canvas, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(m_canvas, 0, LV_PART_MAIN);

    // 初始化图像描述符，用于显示摄像头数据
    memset(&m_img_dsc, 0, sizeof(m_img_dsc));
    m_img_dsc.header.always_zero = 0;
    m_img_dsc.header.w = frame_cap_node->get_fb_width();
    m_img_dsc.header.h = frame_cap_node->get_fb_height();
    m_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;  // RGB565
    m_img_dsc.data_size = frame_cap_node->get_fb_width() * frame_cap_node->get_fb_height() * 2;
    m_img_dsc.data = NULL;  // 将在每帧更新时设置

    ESP_LOGI(TAG, "Image descriptor initialized: %dx%d", m_img_dsc.header.w, m_img_dsc.header.h);

    bsp_display_unlock();
#endif
    ESP_LOGI(TAG, "WhoFrameLCDDisp created successfully");
}

WhoFrameLCDDisp::~WhoFrameLCDDisp()
{
#if !BSP_CONFIG_NO_GRAPHIC_LIB
    bsp_display_lock(0);
    lv_obj_del(m_canvas);
    bsp_display_unlock();
#endif
    delete m_lcd;
}

void WhoFrameLCDDisp::set_lcd_disp_cb(const std::function<void(who::cam::cam_fb_t *)> &lcd_disp_cb)
{
    m_lcd_disp_cb = lcd_disp_cb;
}

#if !BSP_CONFIG_NO_GRAPHIC_LIB
lv_obj_t *WhoFrameLCDDisp::get_canvas()
{
    return m_canvas;
}
#endif

void WhoFrameLCDDisp::task()
{
    ESP_LOGI(TAG, "FrameLCDDisp task started, waiting for frames...");
    while (true) {
        EventBits_t event_bits =
            xEventGroupWaitBits(m_event_group, NEW_FRAME | TASK_PAUSE | TASK_STOP, pdTRUE, pdFALSE, portMAX_DELAY);
        if (event_bits & TASK_STOP) {
            break;
        } else if (event_bits & TASK_PAUSE) {
            xEventGroupSetBits(m_event_group, TASK_PAUSED);
            EventBits_t pause_event_bits =
                xEventGroupWaitBits(m_event_group, TASK_RESUME | TASK_STOP, pdTRUE, pdFALSE, portMAX_DELAY);
            if (pause_event_bits & TASK_STOP) {
                break;
            } else {
                continue;
            }
        }
        auto fb = m_frame_cap_node->cam_fb_peek(m_peek_index);
        if (fb == NULL) {
            ESP_LOGW(TAG, "Frame buffer is NULL!");
            continue;
        }

        ESP_LOGD(TAG, "Got frame: %dx%d, format=%d", fb->width, fb->height, (int)fb->format);

#if BSP_CONFIG_NO_GRAPHIC_LIB
        if (m_lcd_disp_cb) {
            m_lcd_disp_cb(fb);
        }
        m_lcd->draw_bitmap(fb->buf, (int)fb->width, (int)fb->height, 0, 0);
#else
        // 方法：直接通过LCD驱动绘制，不使用Canvas
        // 使用BSP的分块传输函数来避免PSRAM DMA问题
        if (panel_handle) {
            // 使用BSP提供的分块传输函数（自动处理PSRAM到内部RAM的复制和字节序转换）
            lcd_draw_bitmap(0, 0, fb->width, fb->height, fb->buf);

            // 适中延迟，平衡帧率和稳定性
            vTaskDelay(pdMS_TO_TICKS(40));
        } else {
            ESP_LOGW(TAG, "panel_handle is NULL!");
        }

        // 在Canvas上绘制检测框（透明背景，不遮挡摄像头画面）
        bsp_display_lock(0);
        if (m_lcd_disp_cb) {
            m_lcd_disp_cb(fb);
            ESP_LOGD(TAG, "Detection boxes drawn");
        }
        bsp_display_unlock();
#endif
    }
    xEventGroupSetBits(m_event_group, TASK_STOPPED);
    vTaskDelete(NULL);
}
} // namespace lcd_disp
} // namespace who
