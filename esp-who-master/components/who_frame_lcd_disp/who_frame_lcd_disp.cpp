#include "who_frame_lcd_disp.hpp"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"

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

    // 创建Canvas，用于显示摄像头画面 + 检测框叠加
    m_canvas = lv_canvas_create(lv_scr_act());
    ESP_LOGI(TAG, "Canvas created: %p", m_canvas);
    lv_obj_align(m_canvas, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(m_canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(m_canvas, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(m_canvas, 0, LV_PART_MAIN);

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
        // 通过LVGL Canvas显示：将摄像头帧设为Canvas缓冲区，LVGL统一刷新到LCD
        bsp_display_lock(0);

        // 将摄像头帧数据设置为Canvas的缓冲区（零拷贝，直接引用帧数据）
        lv_canvas_set_buffer(m_canvas, fb->buf, fb->width, fb->height, LV_IMG_CF_TRUE_COLOR);

        // 在Canvas上叠加绘制检测框
        if (m_lcd_disp_cb) {
            m_lcd_disp_cb(fb);
        }

        // 标记Canvas需要刷新，LVGL任务会在下次lv_timer_handler()时将数据发送到LCD
        lv_obj_invalidate(m_canvas);

        bsp_display_unlock();
#endif
    }
    xEventGroupSetBits(m_event_group, TASK_STOPPED);
    vTaskDelete(NULL);
}
} // namespace lcd_disp
} // namespace who
