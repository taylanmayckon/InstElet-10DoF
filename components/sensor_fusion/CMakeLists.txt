idf_component_register(
    SRCS "sensor_fusion.c"
    INCLUDE_DIRS "."
    REQUIRES driver mpu6050 hmc5883l sensor_fusion
)