# BSP: Adafruit ESP32 TFT-Feather

ESP-IDF Board Support Package for Adafruit ESP32-S3 TFT Feather.

Configures of this component can be find in "menuconfig->Component config->Board Support Package".

* [Hardware Reference](https://github.com/adafruit/Adafruit-ESP32-S3-TFT-Feather-PCB)
* [Primary Guide: Adafruit ESP32-S3 TFT Feather](https://learn.adafruit.com/adafruit-esp32-s3-tft-feather)

![image](https://github.com/adafruit/Adafruit-ESP32-S3-TFT-Feather-PCB/raw/main/assets/5483.jpg?raw=true)

### Sample code
```c

#include "bsp/tft-feather.h"

void app_main(void)
{
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 挂载SPIFFS
    bsp_spiffs_mount();

    // 初始化显示设备
    bsp_display_start();

    // 开启显示器背光
    bsp_display_backlight_on();
    
    // 启动LVGL UI绘制（自行实现）
    ESP_ERROR_CHECK(ui_main_start());
}
```