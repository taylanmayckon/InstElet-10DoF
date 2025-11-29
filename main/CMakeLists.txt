idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver spi_flash mpu6050 hmc5883l bmp180
)
