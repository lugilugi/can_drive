idf_component_register(
    SRCS "can_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_twai freertos log
)