#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define AQQUISITION_INTERVAL 200 // Intervalo de captura de dados em ms

static const char *TAG_SensorsI2C = "vTaskSensorsI2C";

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
    ESP_LOGI(TAG_SensorsI2C, "Barramento I2C master criado com sucesso");

    SensorData_t sensor_data;

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(AQQUISITION_INTERVAL));
    }
}

void app_main(void) {
    // Criar fila
    xQueueSensorsData = xQueueCreate(10, sizeof(SensorData_t));

    // Criar task
    xTaskCreate(vTaskSensorsI2C, "vTaskSensorsI2C", 4096, NULL, 5, NULL);
}
