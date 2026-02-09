#include "bsp_camera.h"
#include "bsp_i2cbus.h"
#ifdef BSP_CAMERA_USE_PCA9557_PWDN
#include "bsp_pca9557.h"
#endif
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_log.h"
static const char *TAG = "bsp_camera";

// Camera configuration
static camera_config_t camera_config = {
    .pin_pwdn  = BSP_CAMERA_PIN_PWDN,
    .pin_reset = BSP_CAMERA_PIN_RESET,
    .pin_xclk = BSP_CAMERA_PIN_XCLK,
    .pin_sccb_sda = -1,  // Use I2C bus
    .pin_sccb_scl = -1,

    .pin_d7 = BSP_CAMERA_PIN_D7,
    .pin_d6 = BSP_CAMERA_PIN_D6,
    .pin_d5 = BSP_CAMERA_PIN_D5,
    .pin_d4 = BSP_CAMERA_PIN_D4,
    .pin_d3 = BSP_CAMERA_PIN_D3,
    .pin_d2 = BSP_CAMERA_PIN_D2,
    .pin_d1 = BSP_CAMERA_PIN_D1,
    .pin_d0 = BSP_CAMERA_PIN_D0,
    .pin_vsync = BSP_CAMERA_PIN_VSYNC,
    .pin_href = BSP_CAMERA_PIN_HREF,
    .pin_pclk = BSP_CAMERA_PIN_PCLK,

    .xclk_freq_hz = BSP_CAMERA_XCLK_FREQ_HZ,
    .ledc_timer = LEDC_TIMER_1,     // Use TIMER_1 to avoid conflict with LCD backlight
    .ledc_channel = LEDC_CHANNEL_1,

    .pixel_format = BSP_CAMERA_PIXEL_FORMAT,
    .frame_size = BSP_CAMERA_FRAME_SIZE,
    .jpeg_quality = BSP_CAMERA_JPEG_QUALITY,
    .fb_count = BSP_CAMERA_FB_COUNT,
    .fb_location = BSP_CAMERA_FB_LOCATION,
    .grab_mode = BSP_CAMERA_GRAB_MODE,

    .sccb_i2c_port = BSP_I2C_PORT,
};

esp_err_t bsp_camera_init(void)
{
    esp_err_t ret;

    // Initialize camera power control pin if using direct GPIO
#ifndef BSP_CAMERA_USE_PCA9557_PWDN
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BSP_CAMERA_PWDN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
#endif

    // Power on camera
    ESP_LOGI(TAG, "Powering on camera");
    BSP_CAMERA_POWER_ON();
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for camera to stabilize (min 50ms)

    // Initialize camera driver
    ESP_LOGI(TAG, "Initializing camera driver");
    ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

esp_err_t bsp_camera_capture(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }

    // Process frame here if needed

    esp_camera_fb_return(fb);
    return ESP_OK;
}

/****************   Camera Task System   *************************************/
// Shared frame buffer for task system
static camera_fb_t *g_latest_fb = NULL;
static SemaphoreHandle_t g_fb_mutex = NULL;
static SemaphoreHandle_t g_fb_ready_sem = NULL;

// Camera capture task
static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Camera capture task started on Core %d", xPortGetCoreID());
    const TickType_t frame_delay = pdMS_TO_TICKS(50); // ~20fps

    while (1) {
        camera_fb_t *new_fb = esp_camera_fb_get();
        if (new_fb == NULL) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Lock and update shared frame
        xSemaphoreTake(g_fb_mutex, portMAX_DELAY);

        if (g_latest_fb != NULL) {
            esp_camera_fb_return(g_latest_fb);
        }

        g_latest_fb = new_fb;
        xSemaphoreGive(g_fb_mutex);

        // Signal new frame available
        xSemaphoreGive(g_fb_ready_sem);

        vTaskDelay(frame_delay);
    }

    vTaskDelete(NULL);
}

esp_err_t bsp_camera_tasks_init(void)
{
    ESP_LOGI(TAG, "Initializing camera task system");

    // Create mutex
    g_fb_mutex = xSemaphoreCreateMutex();
    if (g_fb_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create frame mutex");
        return ESP_FAIL;
    }

    // Create counting semaphore (max 10 consumers, initial 0)
    g_fb_ready_sem = xSemaphoreCreateCounting(10, 0);
    if (g_fb_ready_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create frame ready semaphore");
        vSemaphoreDelete(g_fb_mutex);
        return ESP_FAIL;
    }

    // Create camera capture task (Core 1, priority 4)
    BaseType_t ret = xTaskCreatePinnedToCore(
        camera_capture_task,
        "cam_capture",
        8192,
        NULL,
        4,
        NULL,
        1  // Core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create camera capture task");
        vSemaphoreDelete(g_fb_mutex);
        vSemaphoreDelete(g_fb_ready_sem);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Camera task system initialized successfully");
    return ESP_OK;
}

camera_fb_t* bsp_camera_get_frame(TickType_t timeout_ms)
{
    if (xSemaphoreTake(g_fb_ready_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return NULL;
    }

    camera_fb_t *fb_copy = NULL;

    xSemaphoreTake(g_fb_mutex, portMAX_DELAY);

    if (g_latest_fb != NULL) {
        fb_copy = (camera_fb_t *)malloc(sizeof(camera_fb_t));
        if (fb_copy != NULL) {
            memcpy(fb_copy, g_latest_fb, sizeof(camera_fb_t));

            fb_copy->buf = (uint8_t *)malloc(g_latest_fb->len);
            if (fb_copy->buf != NULL) {
                memcpy(fb_copy->buf, g_latest_fb->buf, g_latest_fb->len);
            } else {
                ESP_LOGE(TAG, "Failed to allocate frame buffer");
                free(fb_copy);
                fb_copy = NULL;
            }
        } else {
            ESP_LOGE(TAG, "Failed to allocate frame copy");
        }
    }

    xSemaphoreGive(g_fb_mutex);

    return fb_copy;
}

void bsp_camera_frame_free(camera_fb_t *fb)
{
    if (fb != NULL) {
        if (fb->buf != NULL) {
            free(fb->buf);
        }
        free(fb);
    }
}

void LcdDisplayCameraTaskCreate(void)
{
    bsp_camera_tasks_init();
}
