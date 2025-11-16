#include "mpu6050.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"

#define I2C_MASTER_NUM I2C_NUM_0

static const char *TAG = "MPU6050";

// Função para resetar e inicializar o MPU6050
void mpu6050_reset(i2c_master_dev_handle_t mpu6050_handle){
    esp_err_t ret;
    uint8_t buf[2];

    // Resetando (registrador 0x6B, valor 0x80)
    buf[0] = 0x6B;
    buf[1] = 0x80;
    ret = i2c_master_transmit(mpu6050_handle, buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Falha ao resetar MPU6050: %s", esp_err_to_name(ret));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Espera o reset

    // Saindo do Modo Sleep (registrador 0x6B, valor 0x00)
    buf[1] = 0x00;
    ret = i2c_master_transmit(mpu6050_handle, buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Falha ao sair do modo sleep: %s", esp_err_to_name(ret));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Espera estabilizar

    // Colocando no modo Bypass para detectar o HMC5883L
    buf[0] = 0x37;
    buf[1] = 0x02;
    ret = i2c_master_transmit(mpu6050_handle, buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Falha ao configurar modo bypass: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI("MPU6050", "MPU6050 resetado e inicializado com sucesso");
}

// Função para ler os dados brutos do MPU6050
esp_err_t mpu6050_read_raw(i2c_master_dev_handle_t mpu6050_handle, mpu6050_raw_data_t *data) {
    esp_err_t ret;
    uint8_t reg;
    uint8_t buffer[6];

    int16_t accel[3], gyro[3];

    // ACELERAÇÃO (registrador 0x3B, 6 bytes) 
    reg = 0x3B;
    ret = i2c_master_transmit_receive(mpu6050_handle, &reg, 1, buffer, 6, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler acelerômetro: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    data->accel_x = accel[0];
    data->accel_y = accel[1];
    data->accel_z = accel[2];

    // GIROSCÓPIO (registrador 0x43, 6 bytes)
    reg = 0x43;
    ret = i2c_master_transmit_receive(mpu6050_handle, &reg, 1, buffer, 6, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler giroscópio: %s", esp_err_to_name(ret));
        return ret;
    }

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    data->gyro_x = gyro[0];
    data->gyro_y = gyro[1];
    data->gyro_z = gyro[2];

    // TEMPERATURA (registrador 0x41, 2 bytes) 
    reg = 0x41;
    ret = i2c_master_transmit_receive(mpu6050_handle, &reg, 1, buffer, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler temperatura: %s", esp_err_to_name(ret));
        return ret;
    }

    data->temp = (buffer[0] << 8) | buffer[1];

    return ESP_OK;
}


// Função para processar os dados brutos do MPU6050
void mpu6050_proccess_data(mpu6050_raw_data_t raw_data, mpu6050_data_t *final_data){
    final_data->accel_x = raw_data.accel_x/16384.0f;
    final_data->accel_y = raw_data.accel_y/16384.0f;
    final_data->accel_z = raw_data.accel_z/16384.0f;

    final_data->gyro_x = raw_data.gyro_x/131.0f;
    final_data->gyro_y = raw_data.gyro_y/131.0f;
    final_data->gyro_z = raw_data.gyro_z/131.0f;

    final_data->roll = atan2(final_data->accel_y, final_data->accel_z) * 180.0f / M_PI;
    final_data->pitch = atan2(-final_data->accel_x, sqrt(final_data->accel_y*final_data->accel_y + final_data->accel_z*final_data->accel_z)) * 180.0f /M_PI;
}
