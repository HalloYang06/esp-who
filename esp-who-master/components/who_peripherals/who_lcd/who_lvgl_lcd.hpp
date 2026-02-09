#pragma once
#include "esp_lcd_types.h"
#include "bsp/esp-bsp.h"
#if !BSP_CONFIG_NO_GRAPHIC_LIB
#include "lvgl.h"

namespace who {
namespace lcd {
class WhoLCD {
public:
    WhoLCD() { init(); }
    ~WhoLCD() { deinit(); }
    void init();
    void deinit();

private:
    lv_disp_t *m_disp;
    esp_lcd_panel_io_handle_t m_io_handle;
    esp_lcd_panel_handle_t m_panel_handle;
};
} // namespace lcd
} // namespace who
#endif
