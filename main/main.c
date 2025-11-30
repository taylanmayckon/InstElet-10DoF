#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "bmp180.h" // Incluindo o header do BMP180
// Libs do Wi-Fi
#include "wifi_manager.h"
#include "esp_http_client.h"
#include "cJSON.h"

// Defines para Wi-Fi
// #define WIFI_SSID "Galaxy S25+ Taylan"
// #define WIFI_PASSWORD "sanfoninha"
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""

// Defines para I2C e aquisição de dados
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define AQQUISITION_INTERVAL 600 // Intervalo de captura de dados em ms

#define MPU6050_I2C_ADDRESS 0x68

#define SizeSensorsDataFIFO 30 // Tamanho da fila para os dados dos sensores

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
    float pressure; // Usaremos kPa para facilitar a visualização no log
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

// Endereco do servidor para envio dos dados
char url[128];


// Task para aquisição de dados dos sensores via I2C
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

    
    // Adicionando HMC5883L ao barramento 
    i2c_device_config_t hmc5883l_cfg = {
        .device_address = HMC5883L_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_dev_handle_t hmc5883l_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &hmc5883l_cfg, &hmc5883l_handle));

    hmc5883l_init(hmc5883l_handle);
    ESP_LOGI(pcTaskGetName(NULL), "HMC5883L inicializado");

    // --- BMP180 (Barômetro/Termômetro) ---
    // Estrutura do driver do BMP180
    bmp180_dev_t bmp180_dev; 
    
    // Inicializa o BMP180 e lê os parâmetros de calibração
    esp_err_t ret_bmp = bmp180_init(bus_handle, &bmp180_dev);
    if (ret_bmp == ESP_OK) {
        ESP_LOGI(pcTaskGetName(NULL), "BMP180 inicializado com sucesso");
    } else {
        ESP_LOGE(pcTaskGetName(NULL), "Falha ao inicializar BMP180: %s", esp_err_to_name(ret_bmp));
    }


    // Struct para enviar para fila
    SensorData_t sensor_data = {0}; // Inicializa a struct com zeros para segurança

    while (1) {
        // --- Leitura do MPU6050 ---
        esp_err_t ret_mpu = mpu6050_read_raw(mpu6050_handle, &raw_data);
        if (ret_mpu == ESP_OK) {
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
            ESP_LOGW(pcTaskGetName(NULL), "Falha ao ler MPU6050: %s", esp_err_to_name(ret_mpu));
        }

        // --- Leitura do HMC5883L ---
        magnetometer_data_t magnetometer_data;
        esp_err_t ret_hmc = hmc5883l_read_data(hmc5883l_handle, &magnetometer_data);
        if (ret_hmc == ESP_OK){
            sensor_data.magnetometer.x = magnetometer_data.x;
            sensor_data.magnetometer.y = magnetometer_data.y;
            sensor_data.magnetometer.z = magnetometer_data.z;
        }
        else
            ESP_LOGW(pcTaskGetName(NULL), "Falha ao ler HMC5883L: %s", esp_err_to_name(ret_hmc));
            
        // --- Leitura do BMP180 ---
        if (ret_bmp == ESP_OK) {
            int32_t raw_temp, raw_pressure;

            // Leitura da Temperatura (retorna em 0.1 C)
            if (bmp180_read_temperature(&bmp180_dev, &raw_temp) == ESP_OK) {
                // Converte de 0.1 C para float C
                sensor_data.bmp180.temperature = (float)raw_temp / 10.0f; 
            } else {
                ESP_LOGW(pcTaskGetName(NULL), "Falha ao ler Temp BMP180");
                sensor_data.bmp180.temperature = 0.0f;
            }

            // Leitura da Pressão (retorna em Pa)
            // Nota: A leitura de pressão deve ser feita APÓS a leitura de temperatura
            // pois depende do valor de compensação B5 gerado na leitura de temperatura.
            if (bmp180_read_pressure(&bmp180_dev, &raw_pressure) == ESP_OK) {
                // Converte de Pa para kPa (1 kPa = 1000 Pa)
                sensor_data.bmp180.pressure = (float)raw_pressure / 1000.0f;
            } else {
                ESP_LOGW(pcTaskGetName(NULL), "Falha ao ler Pressão BMP180");
                sensor_data.bmp180.pressure = 0.0f;
            }
        }


        // Enviando os dados para a fila
        // Fazer um tratamento melhor para quando a fila encher e o Wi-Fi nao tiver consumido ainda
        if (xQueueSend(xQueueSensorsData, &sensor_data, 0) != pdPASS) { // Usamos 0 de tempo de espera (não bloqueante)
            UBaseType_t count = uxQueueMessagesWaiting(xQueueSensorsData);
            ESP_LOGW(pcTaskGetName(NULL), "Fila cheia! Dados descartados. TAMANHO ATUAL: %u", count);
        } else {
            // Verificando se a fila parou de ser consumida
            UBaseType_t count = uxQueueMessagesWaiting(xQueueSensorsData);
            if(count > SizeSensorsDataFIFO - 5){
                ESP_LOGI(pcTaskGetName(NULL), "Wi-Fi parou de consumir. TAMANHO DA FILA: %u", count);
            }
        }
        

        vTaskDelay(pdMS_TO_TICKS(AQQUISITION_INTERVAL));
    }
}



// Funcao para enviar o JSON via HTTP POST
void send_json_to_server(SensorData_t *data) {
    // Montando JSON
    cJSON *json = cJSON_CreateObject();

    // Acelerometro
    cJSON *accel = cJSON_CreateObject();
    cJSON_AddNumberToObject(accel, "x", data->mpu6050.accel.x);
    cJSON_AddNumberToObject(accel, "y", data->mpu6050.accel.y);
    cJSON_AddNumberToObject(accel, "z", data->mpu6050.accel.z);
    cJSON_AddItemToObject(json, "accel", accel);

    // Giroscópio
    cJSON *gyro = cJSON_CreateObject();
    cJSON_AddNumberToObject(gyro, "x", data->mpu6050.gyro.x);  
    cJSON_AddNumberToObject(gyro, "y", data->mpu6050.gyro.y);
    cJSON_AddNumberToObject(gyro, "z", data->mpu6050.gyro.z);
    cJSON_AddItemToObject(json, "gyro", gyro);
    // Magnetometro
    cJSON *mag = cJSON_CreateObject();
    cJSON_AddNumberToObject(mag, "x", data->magnetometer.x);
    cJSON_AddNumberToObject(mag, "y", data->magnetometer.y);
    cJSON_AddNumberToObject(mag, "z", data->magnetometer.z);
    cJSON_AddItemToObject(json, "mag", mag);
    // BMP180
    cJSON *bmp = cJSON_CreateObject();
    cJSON_AddNumberToObject(bmp, "temperature", data->bmp180.temperature);
    cJSON_AddNumberToObject(bmp, "pressure", data->bmp180.pressure);
    cJSON_AddItemToObject(json, "bmp", bmp);
    // Orientação
    cJSON *orientation = cJSON_CreateObject();
    cJSON_AddNumberToObject(orientation, "pitch", data->orientation.pitch);
    cJSON_AddNumberToObject(orientation, "roll", data->orientation.roll);
    cJSON_AddNumberToObject(orientation, "yaw", data->orientation.yaw);
    cJSON_AddNumberToObject(orientation, "altitude", data->orientation.altitude);
    cJSON_AddItemToObject(json, "orientation", orientation);

    // Enviando os dados para o endpoint 
    char *json_str = cJSON_PrintUnformatted(json);
    esp_http_client_config_t config = {
        .url = url, 
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_str, strlen(json_str));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI("HTTP", "Dados enviados com sucesso, status: %d", esp_http_client_get_status_code(client));
        ESP_LOGI("HTTP", "URL: %s", url);
    } else {
        ESP_LOGE("HTTP", "Erro ao enviar dados: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    cJSON_Delete(json);
    if(json_str) {
        free(json_str);
    }
}


// Task para gerenciar o Wi-Fi e envio dos dados
void vTaskWiFi(void *arg){
    SensorData_t received_data;

    while(1){
        // Verificando se tem dados a serem coletados da fila
        if(xQueueReceive(xQueueSensorsData, &received_data, portMAX_DELAY) == pdTRUE){
            // Processamento desses dados para o wi-fi e envio
            send_json_to_server(&received_data);

            // Debug dos dados recebidos
            // // ESP_LOGI(pcTaskGetName(NULL), "Dado da fila consumido");
            // printf("\n=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
            // printf("\n%s: DADOS RECEBIDOS", pcTaskGetName(NULL));
            // printf("\n[ORIENTACAO (graus)] Pitch: %.2f | Roll: %.2f | Yaw: %.2f | Altitude: %.2f", received_data.orientation.pitch, received_data.orientation.roll, 
            //          received_data.orientation.yaw, received_data.orientation.altitude);
            // printf("\n[ACELERACAO (m/s2) X: %.2f | Y: %.2f | Z: %.2f", received_data.mpu6050.accel.x, received_data.mpu6050.accel.y, received_data.mpu6050.accel.z);
            // printf("\n[GIROSCOPIO (dps)] X: %.2f | Y: %.2f | Z: %.2f", received_data.mpu6050.gyro.x, received_data.mpu6050.gyro.y, received_data.mpu6050.gyro.z);
            // printf("\n[MAGNETOMETRO (gauss)] X: %.2f | Y: %.2f | Z: %.2f", received_data.magnetometer.x, received_data.magnetometer.y, received_data.magnetometer.z);
            // printf("\n[BMP180] Temperatura (°C): %.2f | Pressao (kPa): %.2f\n", received_data.bmp180.temperature, received_data.bmp180.pressure);
        }
    }
}


void app_main(void) {
    // Inicializando Wi-Fi
    wifi_manager_init(WIFI_SSID, WIFI_PASSWORD);
    while(!wifi_manager_is_connected()){
        ESP_LOGI("MAIN", "Aguardando conexão Wi-Fi...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // ESP_LOGI("MAIN", "Conectado ao Wi-Fi! IP: %s", wifi_manager_get_ip());
    // Montando a URL do servidor com o IP obtido
    snprintf(url, sizeof(url), "http://%s:8080/sensores", wifi_manager_get_ip());

    // Criando filas
    xQueueSensorsData = xQueueCreate(SizeSensorsDataFIFO, sizeof(SensorData_t));

    // Criando tasks
    xTaskCreate(vTaskSensorsI2C, "vTaskSensorsI2C", 8192, NULL, 5, NULL); // Aumentei a stack size para acomodar o BMP180
    xTaskCreate(vTaskWiFi, "vTaskWiFi", 4096, NULL, 5, NULL);
}