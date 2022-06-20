/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "oled/SSD1306Wire.h"
#include "oled\OLEDDisplayFonts.h"

void app_main(void)
{
    SSD1306Wire *display;
    display = new SSD1306Wire(0x3c, 4, 15, 16, GEOMETRY_128_64);
    display->init();
    // display->flipScreenVertically();
    display->setFont(ArialMT_Plain_10);
    display->drawString(0, 0, "OLED initial done!");
    display->display();
}
