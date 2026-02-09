#include "who_detect.hpp"
#include "esp_heap_caps.h"
#include <cstring>

namespace who {
namespace detect {
WhoDetect::WhoDetect(const std::string &name, frame_cap::WhoFrameCapNode *frame_cap_node) :
    task::WhoTask(name),
    m_frame_cap_node(frame_cap_node),
    m_model(nullptr),
    m_interval(0),
    m_inv_rescale_x(0),
    m_inv_rescale_y(0),
    m_rescale_max_w(0),
    m_rescale_max_h(0),
    m_result_cb_mutex(xSemaphoreCreateRecursiveMutex())
{
    frame_cap_node->add_new_frame_signal_subscriber(this);
}

WhoDetect::~WhoDetect()
{
    vSemaphoreDelete(m_result_cb_mutex);
    if (m_model) {
        delete m_model;
    }
}

void WhoDetect::set_model(dl::detect::Detect *model)
{
    m_model = model;
}

void WhoDetect::set_rescale_params(float rescale_x, float rescale_y, uint16_t rescale_max_w, uint16_t rescale_max_h)
{
    m_inv_rescale_x = 1.f / rescale_x;
    m_inv_rescale_y = 1.f / rescale_y;
    m_rescale_max_w = rescale_max_w;
    m_rescale_max_h = rescale_max_h;
}

void WhoDetect::set_fps(float fps)
{
    if (fps > 0) {
        m_interval = pdMS_TO_TICKS((int)(1000.f / fps));
    }
}

void WhoDetect::set_detect_result_cb(const std::function<void(const result_t &result)> &result_cb)
{
    xSemaphoreTakeRecursive(m_result_cb_mutex, portMAX_DELAY);
    m_result_cb = result_cb;
    xSemaphoreGiveRecursive(m_result_cb_mutex);
}

void WhoDetect::set_cleanup_func(const std::function<void()> &cleanup_func)
{
    m_cleanup = cleanup_func;
}

void WhoDetect::task()
{
    ESP_LOGI("WhoDetect", "检测任务启动，等待NEW_FRAME事件...");
    TickType_t last_wake_time = xTaskGetTickCount();
    int frame_count = 0;
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
                last_wake_time = xTaskGetTickCount();
                continue;
            }
        }

        frame_count++;
        if (frame_count % 10 == 0) {
            ESP_LOGI("WhoDetect", "收到第%d个NEW_FRAME事件，开始检测...", frame_count);
        }

        auto fb = m_frame_cap_node->cam_fb_peek();
        struct timeval timestamp = fb->timestamp;

        // 创建图像副本用于检测（避免修改原始帧缓冲区）
        dl::image::img_t img = static_cast<dl::image::img_t>(*fb);

        ESP_LOGI("WhoDetect", "原始图像: 尺寸=%dx%d, 格式=%d, 数据地址=%p",
                 img.width, img.height, img.pix_type, img.data);

        // ESP32-S3: ESP-DL模型需要大端RGB565，但摄像头输出小端
        // 在这里转换，不影响LCD显示路径
#if CONFIG_IDF_TARGET_ESP32S3
        ESP_LOGI("WhoDetect", "CONFIG_IDF_TARGET_ESP32S3 已定义，准备转换字节序");
        if (img.pix_type == dl::image::DL_IMAGE_PIX_TYPE_RGB565) {
            ESP_LOGI("WhoDetect", "像素格式确认为 RGB565，开始分配缓冲区");
            // 分配临时缓冲区用于字节序转换
            size_t img_size = img.width * img.height * 2;
            uint8_t *converted_buf = (uint8_t *)heap_caps_malloc(img_size, MALLOC_CAP_SPIRAM);
            if (converted_buf) {
                ESP_LOGI("WhoDetect", "缓冲区分配成功，开始复制和转换 %zu 字节", img_size);
                memcpy(converted_buf, img.data, img_size);
                uint16_t *data = (uint16_t *)converted_buf;
                size_t pixel_count = img.width * img.height;

                // 打印转换前后的前几个像素值
                uint16_t *orig_data = (uint16_t *)img.data;
                ESP_LOGI("WhoDetect", "转换前前3个像素: 0x%04x, 0x%04x, 0x%04x",
                         orig_data[0], orig_data[1], orig_data[2]);

                for (size_t i = 0; i < pixel_count; i++) {
                    uint16_t pixel = data[i];
                    data[i] = (pixel >> 8) | (pixel << 8);  // 小端转大端
                }

                ESP_LOGI("WhoDetect", "转换后前3个像素: 0x%04x, 0x%04x, 0x%04x",
                         data[0], data[1], data[2]);

                img.data = converted_buf;
                ESP_LOGI("WhoDetect", "字节序转换完成，新缓冲区地址=%p", converted_buf);
            } else {
                ESP_LOGE("WhoDetect", "无法分配转换缓冲区(%zu字节)，使用原始数据", img_size);
            }
        } else {
            ESP_LOGW("WhoDetect", "像素格式不是 RGB565 (实际=%d)，跳过转换", img.pix_type);
        }
#else
        ESP_LOGW("WhoDetect", "CONFIG_IDF_TARGET_ESP32S3 未定义，跳过字节序转换");
#endif

        ESP_LOGI("WhoDetect", "运行模型推理: 图像尺寸=%dx%d, 格式=%d", img.width, img.height, img.pix_type);
        auto &res = m_model->run(img);
        ESP_LOGI("WhoDetect", "模型推理完成，检测到%d个目标", res.size());

#if CONFIG_IDF_TARGET_ESP32S3
        // 释放临时缓冲区
        if (img.data != fb->buf) {
            heap_caps_free((void *)img.data);
        }
#endif

        if (m_inv_rescale_x && m_inv_rescale_y && m_rescale_max_w && m_rescale_max_h) {
            rescale_detect_result(res);
        }
        if (m_result_cb) {
            xSemaphoreTakeRecursive(m_result_cb_mutex, portMAX_DELAY);
            m_result_cb({res, timestamp, img});
            xSemaphoreGiveRecursive(m_result_cb_mutex);
        }
        if (m_interval) {
            vTaskDelayUntil(&last_wake_time, m_interval);
        }
    }
    xEventGroupSetBits(m_event_group, TASK_STOPPED);
    vTaskDelete(NULL);
}

void WhoDetect::rescale_detect_result(std::list<dl::detect::result_t> &result)
{
    for (auto &r : result) {
        r.box[0] *= m_inv_rescale_x;
        r.box[1] *= m_inv_rescale_y;
        r.box[2] *= m_inv_rescale_x;
        r.box[3] *= m_inv_rescale_y;
        r.limit_box(m_rescale_max_w, m_rescale_max_h);
        if (!r.keypoint.empty()) {
            assert(r.keypoint.size() == 10);
            for (int i = 0; i < 5; i++) {
                r.keypoint[2 * i] *= m_inv_rescale_x;
                r.keypoint[2 * i + 1] *= m_inv_rescale_y;
            }
            r.limit_keypoint(m_rescale_max_w, m_rescale_max_h);
        }
    }
}

bool WhoDetect::run(const configSTACK_DEPTH_TYPE uxStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID)
{
    if (!m_model) {
        ESP_LOGE("WhoDetect", "detect model is nullptr, please call set_model() first.");
        return false;
    }
    return task::WhoTask::run(uxStackDepth, uxPriority, xCoreID);
}

bool WhoDetect::stop_async()
{
    if (task::WhoTask::stop_async()) {
        xTaskAbortDelay(m_task_handle);
        return true;
    }
    return false;
}

bool WhoDetect::pause_async()
{
    if (task::WhoTask::pause_async()) {
        xTaskAbortDelay(m_task_handle);
        return true;
    }
    return false;
}

void WhoDetect::cleanup()
{
    if (m_cleanup) {
        m_cleanup();
    }
}
} // namespace detect
} // namespace who
