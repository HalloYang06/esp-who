#include "who_detect_result_handle.hpp"
#if !BSP_CONFIG_NO_GRAPHIC_LIB
#include "who_lvgl_utils.hpp"
#endif
#include "dl_image_pixel_cvt_dispatch.hpp"

namespace who {
namespace detect {
void draw_detect_results_on_img(const dl::image::img_t &img,
                                const std::list<dl::detect::result_t> &detect_res,
                                const std::vector<std::vector<uint8_t>> &palette)
{
    for (const auto &res : detect_res) {
        dl::image::draw_hollow_rectangle(img, res.box[0], res.box[1], res.box[2], res.box[3], palette[res.category], 2);
        if (!res.keypoint.empty()) {
            assert(res.keypoint.size() == 10);
            for (int i = 0; i < 5; i++) {
                dl::image::draw_point(img, res.keypoint[2 * i], res.keypoint[2 * i + 1], palette[res.category], 3);
            }
        }
    }
}

#if !BSP_CONFIG_NO_GRAPHIC_LIB
void draw_detect_results_on_canvas(lv_obj_t *canvas,
                                   const std::list<dl::detect::result_t> &detect_res,
                                   const std::vector<lv_color_t> &palette)
{
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_opa = LV_OPA_TRANSP;
    rect_dsc.border_width = 2;

    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.width = 5;
    line_dsc.round_start = 1;
    line_dsc.round_end = 1;

    lv_area_t coords_rect;
    for (const auto &res : detect_res) {
        coords_rect.x1 = (lv_coord_t)res.box[0];
        coords_rect.y1 = (lv_coord_t)res.box[1];
        coords_rect.x2 = (lv_coord_t)res.box[2];
        coords_rect.y2 = (lv_coord_t)res.box[3];
        rect_dsc.border_color = palette[res.category];
        lv_canvas_draw_rect(canvas, coords_rect.x1, coords_rect.y1,
                           coords_rect.x2 - coords_rect.x1,
                           coords_rect.y2 - coords_rect.y1, &rect_dsc);

        if (!res.keypoint.empty()) {
            line_dsc.color = palette[res.category];
            assert(res.keypoint.size() == 10);
            for (int i = 0; i < 5; i++) {
                lv_point_t point = {(lv_coord_t)res.keypoint[2 * i], (lv_coord_t)res.keypoint[2 * i + 1]};
                // Draw a small cross to represent keypoint (LVGL 8.x doesn't have circle drawing on canvas)
                lv_point_t points[2];
                points[0].x = point.x - 3;
                points[0].y = point.y;
                points[1].x = point.x + 3;
                points[1].y = point.y;
                lv_canvas_draw_line(canvas, points, 2, &line_dsc);

                points[0].x = point.x;
                points[0].y = point.y - 3;
                points[1].x = point.x;
                points[1].y = point.y + 3;
                lv_canvas_draw_line(canvas, points, 2, &line_dsc);
            }
        }
    }
}
#endif

void print_detect_results(const std::list<dl::detect::result_t> &detect_res)
{
    int i = 0;
    const char *TAG = "detect";
    if (!detect_res.empty()) {
        if (detect_res.begin()->keypoint.empty()) {
            ESP_LOGI(TAG, "----------------------------------------");
        } else {
            ESP_LOGI(
                TAG,
                "---------------------------------------------------------------------------------------------------"
                "---------------------------------------------------");
        }
    }
    for (const auto &r : detect_res) {
        if (r.keypoint.empty()) {
            ESP_LOGI(TAG, "%d, bbox: [%f, %d, %d, %d, %d]", i, r.score, r.box[0], r.box[1], r.box[2], r.box[3]);
        } else {
            assert(r.keypoint.size() == 10);
            ESP_LOGI(TAG,
                     "%d, bbox: [%f, %d, %d, %d, %d], left_eye: [%d, %d], left_mouth: [%d, %d], nose: [%d, %d], "
                     "right_eye: [%d, %d], right_mouth: [%d, %d]",
                     i,
                     r.score,
                     r.box[0],
                     r.box[1],
                     r.box[2],
                     r.box[3],
                     r.keypoint[0],
                     r.keypoint[1],
                     r.keypoint[2],
                     r.keypoint[3],
                     r.keypoint[4],
                     r.keypoint[5],
                     r.keypoint[6],
                     r.keypoint[7],
                     r.keypoint[8],
                     r.keypoint[9]);
        }
        i++;
    }
}
} // namespace detect

namespace lcd_disp {
#if !BSP_CONFIG_NO_GRAPHIC_LIB
WhoDetectResultLCDDisp::WhoDetectResultLCDDisp(task::WhoTask *task,
                                               lv_obj_t *canvas,
                                               const std::vector<std::vector<uint8_t>> &palette) :
    m_task(task), m_res_mutex(xSemaphoreCreateMutex()), m_result(), m_canvas(canvas)
{
    m_palette = cvt_to_lv_palette(palette);
}
#else
WhoDetectResultLCDDisp::WhoDetectResultLCDDisp(task::WhoTask *task, const std::vector<std::vector<uint8_t>> &palette) :
    m_task(task),
    m_res_mutex(xSemaphoreCreateMutex()),
    m_result(),
    m_rgb888_palette(palette),
    m_rgb565_palette(palette.size(), std::vector<uint8_t>(2))
{
#if CONFIG_IDF_TARGET_ESP32P4
    uint32_t caps = 0;
#else
    uint32_t caps = dl::image::DL_IMAGE_CAP_RGB565_BIG_ENDIAN;
#endif
    for (int i = 0; i < m_rgb888_palette.size(); i++) {
        dl::image::cvt_pix(m_rgb888_palette[i].data(),
                           m_rgb565_palette[i].data(),
                           dl::image::DL_IMAGE_PIX_TYPE_RGB888,
                           dl::image::DL_IMAGE_PIX_TYPE_RGB565,
                           caps);
    }
}
#endif

WhoDetectResultLCDDisp::~WhoDetectResultLCDDisp()
{
    vSemaphoreDelete(m_res_mutex);
}

void WhoDetectResultLCDDisp::save_detect_result(const detect::WhoDetect::result_t &result)
{
    xSemaphoreTake(m_res_mutex, portMAX_DELAY);
    m_results.push(result);
    xSemaphoreGive(m_res_mutex);
}

void WhoDetectResultLCDDisp::lcd_disp_cb(who::cam::cam_fb_t *fb)
{
    if (!m_task->is_active()) {
        return;
    }
    xSemaphoreTake(m_res_mutex, portMAX_DELAY);
    // Try to sync camera frame and result, skip the future result.
    auto compare_timestamp = [](const struct timeval &t1, const struct timeval &t2) -> bool {
        if (t1.tv_sec == t2.tv_sec) {
            return t1.tv_usec < t2.tv_usec;
        }
        return t1.tv_sec < t2.tv_sec;
    };
    struct timeval t1 = fb->timestamp;
    // If detect fps higher than display fps, the result queue may be more than 1. May happen when using lvgl.
    while (!m_results.empty()) {
        detect::WhoDetect::result_t result = m_results.front();
        if (!compare_timestamp(t1, result.timestamp)) {
            m_result = result;
            m_results.pop();
        } else {
            break;
        }
    }

    // 输出人脸检测结果到日志
    if (!m_result.det_res.empty()) {
        const char *TAG = "FaceDetect";
        ESP_LOGI(TAG, "检测到 %d 个人脸:", m_result.det_res.size());
        int face_idx = 0;
        for (const auto &r : m_result.det_res) {
            ESP_LOGI(TAG, "  人脸%d: 坐标[x1=%d, y1=%d, x2=%d, y2=%d], 置信度=%.3f",
                     face_idx++, r.box[0], r.box[1], r.box[2], r.box[3], r.score);
            if (!r.keypoint.empty() && r.keypoint.size() >= 10) {
                ESP_LOGI(TAG, "    关键点: 左眼(%d,%d) 右眼(%d,%d) 鼻子(%d,%d) 左嘴角(%d,%d) 右嘴角(%d,%d)",
                         r.keypoint[0], r.keypoint[1], r.keypoint[2], r.keypoint[3],
                         r.keypoint[4], r.keypoint[5], r.keypoint[6], r.keypoint[7],
                         r.keypoint[8], r.keypoint[9]);
            }
        }
    }

    xSemaphoreGive(m_res_mutex);
#if BSP_CONFIG_NO_GRAPHIC_LIB
    if (fb->format == cam::cam_fb_fmt_t::CAM_FB_FMT_RGB565) {
        detect::draw_detect_results_on_img(*fb, m_result.det_res, m_rgb565_palette);
    } else if (fb->format == cam::cam_fb_fmt_t::CAM_FB_FMT_RGB888) {
        detect::draw_detect_results_on_img(*fb, m_result.det_res, m_rgb888_palette);
    }
#else
    ESP_LOGI("DetectResult", "绘制检测结果到canvas, 检测数=%d", m_result.det_res.size());
    detect::draw_detect_results_on_canvas(m_canvas, m_result.det_res, m_palette);
    ESP_LOGI("DetectResult", "检测结果绘制完成");
#endif
}

void WhoDetectResultLCDDisp::cleanup()
{
    xSemaphoreTake(m_res_mutex, portMAX_DELAY);
    std::queue<detect::WhoDetect::result_t>().swap(m_results);
    m_result = {};
    xSemaphoreGive(m_res_mutex);
}
} // namespace lcd_disp
} // namespace who
