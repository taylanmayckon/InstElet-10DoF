#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#define MPU6050_I2C_ADDRESS 0x68

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_raw_data_t;

typedef struct{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float pitch;
    float roll;
    float temp;
} mpu6050_data_t;


void mpu6050_reset(i2c_master_dev_handle_t mpu6050_handle);
esp_err_t mpu6050_read_raw(i2c_master_dev_handle_t mpu6050_handle, mpu6050_raw_data_t *data);
void mpu6050_proccess_data(mpu6050_raw_data_t raw_data, mpu6050_data_t *final_data);



#endif