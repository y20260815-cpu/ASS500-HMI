/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

/* JC1060P470 panel revision. Keep this compile-time selection aligned with
 * DY-EP4: V0 is the original panel and V2 is SKU 10153004-V2 (2632). */
#define LCD_V0 0
#define LCD_V2 2
#define LCD_VERSION LCD_V2

#if LCD_VERSION != LCD_V0 && LCD_VERSION != LCD_V2
#error "LCD_VERSION must be LCD_V0 or LCD_V2"
#endif

/**************************************************************************************************
 * BSP configuration
 **************************************************************************************************/
// By default, this BSP is shipped with LVGL graphical library. Enabling this option will exclude it.
// If you want to use BSP without LVGL, select BSP version with 'noglib' suffix.
#if !defined(BSP_CONFIG_NO_GRAPHIC_LIB) // Check if the symbol is not coming from compiler definitions (-D...)
#define BSP_CONFIG_NO_GRAPHIC_LIB (0)
#endif
