idf_component_register(
    SRCS "ada_esp32_feather.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs
    PRIV_REQUIRES esp_lcd
)
