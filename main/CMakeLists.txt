idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_netif nvs_flash spi_flash mpu6050 hmc5883l bmp180 wifi_manager esp_http_client json espp__filters
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)