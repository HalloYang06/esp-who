/**
 * @file esp-bsp.h
 * @brief BSP (Board Support Package) for Lichuang ESP32-S3 Board
 *
 * This is the main BSP header file that provides a unified interface
 * for the Lichuang ESP32-S3 development board.
 */

#ifndef BSP_ESP_BSP_H
#define BSP_ESP_BSP_H

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// Include the main Lichuang BSP header
#include "bsp_lichuang.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * BSP Capabilities - Define what features this board supports
 **************************************************************************************************/
#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          0
#define BSP_CAPS_BUTTONS        0
#define BSP_CAPS_AUDIO          0
#define BSP_CAPS_AUDIO_SPEAKER  0
#define BSP_CAPS_AUDIO_MIC      0
#define BSP_CAPS_SDCARD         0
#define BSP_CAPS_IMU            0

/**************************************************************************************************
 * LCD Configuration (already defined in bsp_lichuang.h, but re-exported here for compatibility)
 **************************************************************************************************/
// LCD resolution is defined in bsp_lichuang.h:
// BSP_LCD_H_RES = 320
// BSP_LCD_V_RES = 240

// Ensure no graphics library is disabled (ESP-WHO needs display)
#ifndef BSP_CONFIG_NO_GRAPHIC_LIB
#define BSP_CONFIG_NO_GRAPHIC_LIB 0
#endif

/**************************************************************************************************
 * Camera Configuration
 **************************************************************************************************/
// Camera configuration is handled by BSP_CAMERA_DEFAULT_CONFIG in bsp_lichuang.h

/**************************************************************************************************
 * Board Initialization
 **************************************************************************************************/

// bsp_board_init() is already defined in bsp_lichuang.h (included above)

#ifdef __cplusplus
}
#endif

#endif /* BSP_ESP_BSP_H */
