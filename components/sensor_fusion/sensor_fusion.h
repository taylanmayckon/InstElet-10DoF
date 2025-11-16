#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "mpu6050.h"
#include "hmc5883l.h"

typedef struct{
    float pitch_output[2];
    float roll_output[2];
} mpu6050_filtered_t;

void mpu6050_kalmann_filter(mpu6050_data_t data, mpu6050_filtered_t *mpu6050_filtered);

#endif