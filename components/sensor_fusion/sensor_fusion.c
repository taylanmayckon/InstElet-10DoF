#include "sensor_fusion.h"
#include "mpu6050.h"
#include "hmc5883l.h"

// Função para calculo do filtro de Kalmann 1d
static void mpu6050_kalman_1d(float *kalmanState, float *kalmanUncertainty, float kalmanInput, float kalmanMeasurement){
    float dt = 0.1;

    *kalmanState = *kalmanState + dt * kalmanInput;

    // Predição da Incerteza
    *kalmanUncertainty = *kalmanUncertainty + dt * dt * 10.0 * 10.0;
    // Cálculo do Ganho de Kalman
    float kalmanGain = *kalmanUncertainty / (*kalmanUncertainty + 3.0 * 3.0);
    // Atualização do Estado
    *kalmanState = *kalmanState + kalmanGain * (kalmanMeasurement - *kalmanState);
    // Atualização da Incerteza 
    *kalmanUncertainty = (1.0 - kalmanGain) * *kalmanUncertainty;
}

// Função para um Filtro de Kalmann Simplificado
static float kalmanAngleRoll = 0.0;
static float kalmanUncertaintyAngleRoll = 2.0 * 2.0; // 4.0
static float kalmanAnglePitch = 0.0;
static float kalmanUncertaintyAnglePitch = 2.0 * 2.0; // 4.0
void mpu6050_kalmann_filter(mpu6050_data_t data, mpu6050_filtered_t *mpu6050_filtered){
    // Roll
    mpu6050_kalman_1d(&kalmanAngleRoll, &kalmanUncertaintyAngleRoll, data.gyro_x, data.roll);
    mpu6050_filtered->roll_output[0] = kalmanAngleRoll;
    mpu6050_filtered->roll_output[1] = kalmanUncertaintyAngleRoll;

    // Pitch
    mpu6050_kalman_1d(&kalmanAnglePitch, &kalmanUncertaintyAnglePitch, data.gyro_y, data.pitch);
    mpu6050_filtered->pitch_output[0] = kalmanAnglePitch;
    mpu6050_filtered->pitch_output[1] = kalmanUncertaintyAnglePitch;
}