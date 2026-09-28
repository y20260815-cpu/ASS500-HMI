cmake_minimum_required(VERSION 3.16)

# ─────────────────────────────────────────────────────────────────────
#  AS500-HMI — ESP32-P4 7-inch HMI project
#  보드 : JC1060P470C_I_W  (ESP32-P4 v1.x + 1024×600 MIPI-DSI)
#
#  컴포넌트 구조 (brookesia 경로 참조 없음)
#    components/
#      espressif__esp32_p4_function_ev_board/  ← BSP (로컬 복사)
#    managed_components/
#      lvgl__lvgl, espressif__esp_lvgl_port,
#      espressif__esp_lcd_ek79007/jd9165/touch_gt911/touch,
#      espressif__esp_codec_dev, espressif__cmake_utilities,
#      espressif__esp_lcd_ili9881c/lt8912b, espressif__esp_sccb_intf
# ─────────────────────────────────────────────────────────────────────

# 경고 억제 (-Wno-deprecated-enum-enum-conversion 은 C++ 전용 → CXX_FLAGS 에만)
add_compile_options(
    -Wno-format
    -Wno-unused-variable
    -Wno-missing-field-initializers
    -Wno-ignored-qualifiers
    -Wno-cast-function-type
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-enum-enum-conversion")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(as500_hmi VERSION 1.0.0)

