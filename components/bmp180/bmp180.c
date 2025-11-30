#include "bmp180.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMP180_DRV";

// --- Funções Auxiliares de I2C ---

/**
 * @brief Função genérica para ler N bytes de um registrador específico.
 */
static esp_err_t bmp180_i2c_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    // 1. Escreve o endereço do registrador (Write + Reg_Addr)
    // 2. Lê os dados (Read + Data)
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, -1);
}

/**
 * @brief Função genérica para escrever um byte em um registrador específico.
 */
static esp_err_t bmp180_i2c_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, 2, -1);
}

// --- Fórmulas de Compensação (simplificadas para este exemplo, DEVEM ser as do Datasheet) ---

/**
 * @brief Simula o cálculo da compensação B5 (necessária para Temperatura e Pressão).
 * Em um driver real, B5 é uma variável crucial calculada a partir da temperatura não compensada (UT).
 * * @param dev Estrutura do driver.
 * @param ut Valor da temperatura não compensada.
 */
static void calculate_b5_compensation(bmp180_dev_t *dev, int32_t ut) {
    // FÓRMULAS BMP180 AQUI: Implementar a complexa equação do Datasheet para B5
    // Exemplo Simples (Não use em produção!):
    dev->b5_comp = (ut - dev->calib_params.ac6) * dev->calib_params.ac5 / 32768;
}

/**
 * @brief Compensa a temperatura.
 */
static int32_t compensate_temperature(bmp180_dev_t *dev, int32_t ut) {
    calculate_b5_compensation(dev, ut); // Calcula B5 primeiro
    // FÓRMULAS BMP180 AQUI: Implementar a equação do Datasheet (resultado em 0.1 C)
    return (dev->b5_comp + 8) >> 4; // Exemplo Simples: (T * 10)
}

/**
 * @brief Compensa a pressão.
 */
static int32_t compensate_pressure(bmp180_dev_t *dev, int32_t up) {
    // FÓRMULAS BMP180 AQUI: Implementar a equação do Datasheet (usa B5 já calculado)
    // Exemplo Simples: (P em Pa)
    return 100000;
}

// --- Funções de Driver ---

/**
 * @brief Lê os 22 bytes dos parâmetros de calibração do sensor.
 */
static esp_err_t bmp180_read_calib_params(i2c_master_dev_handle_t dev_handle, bmp180_calib_param_t *params) {
    uint8_t data[22];
    esp_err_t ret = bmp180_i2c_read(dev_handle, BMP180_REG_CALIB_START, data, 22);

    if (ret == ESP_OK) {
        // Converte os bytes lidos para as variáveis de calibração (Big-Endian no sensor)
        params->ac1 = (int16_t)(((data[0] << 8) | data[1]));
        params->ac2 = (int16_t)(((data[2] << 8) | data[3]));
        params->ac3 = (int16_t)(((data[4] << 8) | data[5]));
        params->ac4 = (uint16_t)(((data[6] << 8) | data[7]));
        params->ac5 = (uint16_t)(((data[8] << 8) | data[9]));
        params->ac6 = (uint16_t)(((data[10] << 8) | data[11]));
        params->b1  = (int16_t)(((data[12] << 8) | data[13]));
        params->b2  = (int16_t)(((data[14] << 8) | data[15]));
        params->mb  = (int16_t)(((data[16] << 8) | data[17]));
        params->mc  = (int16_t)(((data[18] << 8) | data[19]));
        params->md  = (int16_t)(((data[20] << 8) | data[21]));
        ESP_LOGI(TAG, "Calibração lida com sucesso.");
    } else {
        ESP_LOGE(TAG, "Falha ao ler parâmetros de calibração I2C: %s", esp_err_to_name(ret));
    }
    return ret;
}


esp_err_t bmp180_init(i2c_master_bus_handle_t i2c_bus_handle, bmp180_dev_t *out_dev) {
    if (!out_dev) return ESP_ERR_INVALID_ARG;

    // 1. Configurar o dispositivo I2C
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP180_I2C_ADDR,
        .scl_speed_hz = 100000, // 100kHz é seguro
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &out_dev->i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Armazena o handle do bus
    out_dev->i2c_bus_handle = i2c_bus_handle;

    // 2. Ler os parâmetros de calibração
    return bmp180_read_calib_params(out_dev->i2c_dev_handle, &out_dev->calib_params);
}

esp_err_t bmp180_read_temperature(bmp180_dev_t *dev, int32_t *temperature) {
    if (!dev || !temperature) return ESP_ERR_INVALID_ARG;
    esp_err_t ret;
    uint8_t data[2];
    int32_t uncompensated_temp;
    
    // 1. Envia o comando para iniciar a medição de temperatura
    ret = bmp180_i2c_write(dev->i2c_dev_handle, BMP180_REG_CTRL_MEAS, BMP180_CMD_READ_TEMP);
    if (ret != ESP_OK) return ret;

    // 2. Espera o tempo de conversão (4.5ms)
    vTaskDelay(pdMS_TO_TICKS(5)); 
    
    // 3. Lê o resultado da temperatura (2 bytes: MSB, LSB)
    ret = bmp180_i2c_read(dev->i2c_dev_handle, BMP180_REG_OUT_MSB, data, 2);
    if (ret != ESP_OK) return ret;
    
    // Combina os 2 bytes no valor de temperatura não compensada (UT)
    uncompensated_temp = (data[0] << 8) | data[1];

    // 4. Compensa a temperatura
    *temperature = compensate_temperature(dev, uncompensated_temp);

    return ESP_OK;
}

esp_err_t bmp180_read_pressure(bmp180_dev_t *dev, int32_t *pressure) {
    if (!dev || !pressure) return ESP_ERR_INVALID_ARG;
    esp_err_t ret;
    uint8_t data[3];
    int32_t uncompensated_pressure;
    uint8_t oss_cmd = BMP180_CMD_READ_PRESS_OSS0 | (BMP180_OSS << 6);
    
    // 1. Envia o comando para iniciar a medição de pressão
    ret = bmp180_i2c_write(dev->i2c_dev_handle, BMP180_REG_CTRL_MEAS, oss_cmd);
    if (ret != ESP_OK) return ret;

    // 2. Espera o tempo de conversão (depende do OSS: 4.5ms para OSS=0)
    vTaskDelay(pdMS_TO_TICKS(5)); 
    
    // 3. Lê o resultado da pressão (3 bytes: MSB, LSB, XLSB)
    ret = bmp180_i2c_read(dev->i2c_dev_handle, BMP180_REG_OUT_MSB, data, 3);
    if (ret != ESP_OK) return ret;
    
    // Combina os 3 bytes no valor de pressão não compensada (UP)
    // O BMP180 retorna 19 bits de pressão. A parte fracionária é o XLSB.
    uncompensated_pressure = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - BMP180_OSS);

    // 4. Compensa a pressão
    *pressure = compensate_pressure(dev, uncompensated_pressure);
    
    return ESP_OK;
}