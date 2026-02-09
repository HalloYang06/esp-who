#pragma once

#include "lvgl.h"
#include <vector>

namespace who {
inline lv_color_t cvt_to_lv_color(const std::vector<uint8_t> &color)
{
    // 直接使用 lv_color_make，LVGL 会根据 LV_COLOR_16_SWAP 配置自动处理字节序
    return lv_color_make(color[0], color[1], color[2]);
}

inline std::vector<lv_color_t> cvt_to_lv_palette(const std::vector<std::vector<uint8_t>> &palette)
{
    std::vector<lv_color_t> lv_palette;
    for (int i = 0; i < palette.size(); i++) {
        lv_palette.emplace_back(cvt_to_lv_color(palette[i]));
    }
    return lv_palette;
}

lv_obj_t *create_lvgl_btn(const char *text, const lv_font_t *font, lv_obj_t *parent = lv_scr_act());
lv_obj_t *create_lvgl_label(const char *text,
                            const lv_font_t *font,
                            const std::vector<uint8_t> &color = {255, 0, 0},
                            lv_obj_t *parent = lv_scr_act());
} // namespace who
