/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "ssd1306.h"


void app_main(void)
{   
    i2c_master_init();
    ssd1306_init();
    const char *test_string="CAI CON CACCCCCCCCCCCCCCCCCCCCCCCCCCCC";
    task_ssd1306_display_text(test_string);
}
