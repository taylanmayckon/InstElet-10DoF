#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define AQQUISITION_INTERVAL 200 // Intervalo de captura de dados em ms

// Estruturas para informaçao dos sensores
typedef struct{
    float x;
    float y;
    float z;
} axis_t;

// Dados do MPU6050
typedef struct{
    axis_t accel;
    axis_t gyro;
    float temperature;
} MPU_data_t;

// Dados do BMP180
typedef struct{
    float temperature;
    float pressure;
} BMP180_data_t;

// Dados derivados de fusão sensorial 
typedef struct{
    float pitch;
    float roll;
    float yaw;
    float altitude;
} Orientation_data_t;;

// Estrutura completa dos sensores do GY-87
typedef struct __attribute__((packed)){
    MPU_data_t mpu6050;
    axis_t magnetometer; // HMC5883L
    BMP180_data_t bmp180;
    Orientation_data_t orientation;
} SensorData_t;

// Fila para enviar dados entre tasks
QueueHandle_t xQueueSensorsData;


static const char *TAG_SensorsI2C = "vTaskSensorsI2C";
// Task para leitura dos sensores
void vTaskSensorsI2C(void *arg){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG_SensorsI2C, "I2C inicializado com sucesso");

    SensorData_t sensor_data;

    while (1){
        

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}



void app_main(void){
    // Filas
    xQueueSensorsData = xQueueCreate(10, sizeof(SensorData_t));

    // Tasks
    xTaskCreate(vTaskSensorsI2C, "vTaskSensorsI2C", 4096, NULL, 5, NULL);
}
