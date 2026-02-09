#include "who_s3_cam.hpp"
#include "esp_err.h"
#include "esp_log.h"

// 包含立创实战派BSP（如果定义了相应的宏）
#ifdef CONFIG_BSP_BOARD_LICHUANG
#include "bsp_lichuang.h"
#endif

static const char *TAG = "WhoS3Cam";

namespace who {
namespace cam {

WhoS3Cam::WhoS3Cam(const pixformat_t pixel_format,
                   const framesize_t frame_size,
                   const uint8_t fb_count,
                   bool vertical_flip,
                   bool horizontal_flip) :
    WhoCam(fb_count, resolution[frame_size].width, resolution[frame_size].height), m_format(pixel_format)
{
#ifdef CONFIG_BSP_BOARD_LICHUANG
    // BSP已在app_main中初始化，这里只需确保I2C可用
    // 摄像头电源已由bsp_board_init()打开
    ESP_LOGI(TAG, "Using Lichuang BSP configuration");
#else
    ESP_ERROR_CHECK(bsp_i2c_init());
#endif
    camera_config_t camera_config = BSP_CAMERA_DEFAULT_CONFIG;
    camera_config.pixel_format = pixel_format;
    camera_config.frame_size = frame_size;
    camera_config.fb_count = fb_count;
    camera_config.grab_mode = CAMERA_GRAB_LATEST;
    if (pixel_format == PIXFORMAT_JPEG) {
        camera_config.xclk_freq_hz = 20000000;
    }

    ESP_LOGI(TAG, "Initializing camera: format=%d, size=%dx%d, fb_count=%d",
             pixel_format, resolution[frame_size].width, resolution[frame_size].height, fb_count);
    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", ret);
        return;
    }
    ESP_ERROR_CHECK(esp_camera_set_psram_mode(true));
    ESP_ERROR_CHECK(set_flip(!vertical_flip, !horizontal_flip));
    ESP_LOGI(TAG, "Camera initialized successfully");
}

WhoS3Cam::~WhoS3Cam()
{
    ESP_ERROR_CHECK(esp_camera_deinit());
}

cam_fb_t *WhoS3Cam::cam_fb_get()
{
    camera_fb_t *fb = esp_camera_fb_get();
    int i = get_cam_fb_index();
    m_cam_fbs[i] = cam_fb_t(*fb);
    return &m_cam_fbs[i];
}

void WhoS3Cam::cam_fb_return(cam_fb_t *fb)
{
    esp_camera_fb_return((camera_fb_t *)fb->ret);
}

esp_err_t WhoS3Cam::set_flip(bool vertical_flip, bool horizontal_flip)
{
    if (!vertical_flip & !horizontal_flip) {
        return ESP_OK;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (vertical_flip) {
        if (s->set_vflip(s, 1) != 0) {
            ESP_LOGE(TAG, "Failed to mirror the frame vertically.");
            return ESP_FAIL;
        }
    }
    if (horizontal_flip) {
        if (s->set_hmirror(s, 1) != 0) {
            ESP_LOGE(TAG, "Failed to mirror the frame horizontally.");
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

int WhoS3Cam::get_cam_fb_index()
{
    static int i = 0;
    int index = i;
    i = (i + 1) % m_fb_count;
    return index;
}

} // namespace cam
} // namespace who
