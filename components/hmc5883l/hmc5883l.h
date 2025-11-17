#ifndef HMC5883L_H
#define HMC5883L_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#define HMC5883L_I2C_ADDRESS 0x1E

// Definições de registradores do HMC5883L
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE     0x02
#define HMC5883L_DATA_X_MSB 0x03
#define HMC5883L_DATA_X_LSB 0x04
#define HMC5883L_DATA_Z_MSB 0x05
#define HMC5883L_DATA_Z_LSB 0x06
#define HMC5883L_DATA_Y_MSB 0x07
#define HMC5883L_DATA_Y_LSB 0x08


#define HMC5883L_SCALE_FACTOR 1090.0f // Escala para conversao, tabelado no Datasheet de acordo com ganho

// Estrutura para armazenar dados do HMC5883L
typedef struct {
    float x;
    float y;
    float z;
} magnetometer_data_t;


void hmc5883l_init(i2c_master_dev_handle_t hmc5883l_handle);
esp_err_t hmc5883l_read_data(i2c_master_dev_handle_t hmc5883l_handle, magnetometer_data_t *data);

#endif