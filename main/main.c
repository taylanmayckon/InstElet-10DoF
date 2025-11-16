#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "hmc5883l.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define AQQUISITION_INTERVAL 200 // Intervalo de captura de dados em ms

#define MPU6050_I2C_ADDRESS 0x68

// Estruturas para informação dos sensores
typedef struct {
    float x;
    float y;
    float z;
} axis_t;

// Dados do MPU6050
typedef struct {
    axis_t accel;
    axis_t gyro;
    float temperature;
} MPU_data_t;

// Dados do BMP180
typedef struct {
    float temperature;
    float pressure;
} BMP180_data_t;

// Dados derivados de fusão sensorial
typedef struct {
    float pitch;
    float roll;
    float yaw;
    float altitude;
} Orientation_data_t;

// Estrutura completa dos sensores do GY-87
typedef struct __attribute__((packed)) {
    MPU_data_t mpu6050;
    axis_t magnetometer; // HMC5883L
    BMP180_data_t bmp180;
    Orientation_data_t orientation;
} SensorData_t;

// Fila para enviar dados entre tasks
QueueHandle_t xQueueSensorsData;

void vTaskSensorsI2C(void *arg) {
    // Configurando barramento I2C Master
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(pcTaskGetName(NULL), "Barramento I2C master criado com sucesso");


    // Adicionando MPU6050 ao barramento 
    i2c_device_config_t mpu6050_cfg = {
        .device_address = MPU6050_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_dev_handle_t mpu6050_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu6050_cfg, &mpu6050_handle));

    // Configurando MPU6050
    mpu6050_reset(mpu6050_handle);
    ESP_LOGI(pcTaskGetName(NULL), "MPU6050 inicializado");
    mpu6050_raw_data_t raw_data;
    mpu6050_data_t processed_data;
    mpu6050_filtered_t filtered_data;



    // Adicionando HMC5883L ao barramento 
    i2c_device_config_t hmc5883l_cfg = {
        .device_address = HMC5883L_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_dev_handle_t hmc5883l_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &hmc5883l_cfg, &hmc5883l_handle));

    hmc5883l_init(hmc5883l_handle);
    ESP_LOGI(pcTaskGetName(NULL), "HMC5883L inicializado");

    // Struct para enviar para fila
    SensorData_t sensor_data;

    while (1) {
        esp_err_t ret = mpu6050_read_raw(mpu6050_handle, &raw_data);
        if (ret == ESP_OK) {
            // Processa dados e aplica filtro de Kalmann
            mpu6050_proccess_data(raw_data, &processed_data);

            // Alocando os valores na struct da fila
            axis_t accel = {processed_data.accel_x, processed_data.accel_y, processed_data.accel_z};
            axis_t gyro = {processed_data.gyro_x, processed_data.gyro_y, processed_data.gyro_z};
            sensor_data.mpu6050.accel = accel;
            sensor_data.mpu6050.gyro = gyro;
            sensor_data.mpu6050.temperature = processed_data.temp;
            // Debug 
            // mpu6050_debug_data(processed_data, filtered_data);
        } else {
            ESP_LOGW(pcTaskGetName(NULL), "Falha ao ler MPU6050");
        }


        magnetometer_data_t magnetometer_data;
        esp_err_t ret_hmc = hmc5883l_read_data(hmc5883l_handle, &magnetometer_data);
        if (ret_hmc == ESP_OK){
            sensor_data.magnetometer.x = magnetometer_data.x;
            sensor_data.magnetometer.y = magnetometer_data.y;
            sensor_data.magnetometer.z = magnetometer_data.z;
        }
        else
            ESP_LOGW(pcTaskGetName(NULL), "Falha ao ler HMC5883L: %s", esp_err_to_name(ret_hmc));
            


        // Enviando os dados para a fila
        xQueueSend(xQueueSensorsData, &sensor_data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(AQQUISITION_INTERVAL));
    }
}


void vTaskWiFi(void *arg){
    SensorData_t received_data;

    while(1){
        // Verificando se tem dados a serem coletados da fila
        if(xQueueReceive(xQueueSensorsData, &received_data, portMAX_DELAY) == pdTRUE){
            // Processamento desses dados para o wi-fi e envio
            // ESP_LOGI(pcTaskGetName(NULL), "Dado da fila consumido");
            printf("\n\n%s: DADOS RECEBIDOS", pcTaskGetName(NULL));
            printf("\n[ORIENTACAO (graus)] Pitch: %.2f | Roll: %.2f | Yaw: %.2f | Altitude: %.2f", received_data.orientation.pitch, received_data.orientation.roll, 
                                                            received_data.orientation.yaw, received_data.orientation.altitude);
            printf("\n[ACELERACAO (m/s2) X: %.2f | Y: %.2f | Z: %.2f", received_data.mpu6050.accel.x, received_data.mpu6050.accel.y, received_data.mpu6050.accel.z);
            printf("\n[GIROSCOPIO (dps)] X: %.2f | Y: %.2f | Z: %.2f", received_data.mpu6050.gyro.x, received_data.mpu6050.gyro.y, received_data.mpu6050.gyro.z);
            printf("\n[MAGNETOMETRO (gauss)] X: %.2f | Y: %.2f | Z: %.2f", received_data.magnetometer.x, received_data.magnetometer.y, received_data.magnetometer.z);
            printf("\n[BMP180] Temperatura (°C): %.2f | Pressao (kPa): %.2f\n", received_data.bmp180.temperature, received_data.bmp180.pressure);
        }
    }
}


void app_main(void) {
    // Criando filas
    xQueueSensorsData = xQueueCreate(10, sizeof(SensorData_t));

    // Criando tasks
    xTaskCreate(vTaskSensorsI2C, "vTaskSensorsI2C", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskWiFi, "vTaskWiFi", 4096, NULL, 5, NULL);
}
