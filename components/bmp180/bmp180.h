#ifndef BMP180_H
#define BMP180_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <stdint.h>

// Endereço I2C do BMP180
#define BMP180_I2C_ADDR             0x77

// Endereços dos Registradores do Sensor
#define BMP180_REG_CALIB_START      0xAA    // Início dos registradores de calibração
#define BMP180_REG_CTRL_MEAS        0xF4    // Registrador de controle de medição
#define BMP180_REG_OUT_MSB          0xF6    // MSB dos dados de medição

// Comandos de Medição
#define BMP180_CMD_READ_TEMP        0x2E    // Comando para iniciar medição de temperatura
#define BMP180_CMD_READ_PRESS_OSS0  0x34    // Comando para iniciar medição de pressão (OSS=0, Standard)

// Fator de Oversampling (OSS) - ajustável se necessário (0 a 3)
#define BMP180_OSS                  0 // Simplificação para este exemplo

/**
 * @brief Estrutura para armazenar os coeficientes de calibração lidos do sensor.
 */
typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp180_calib_param_t;

/**
 * @brief Estrutura principal do driver do sensor.
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle;  // Handle do barramento I2C
    i2c_master_dev_handle_t i2c_dev_handle;  // Handle do dispositivo I2C
    bmp180_calib_param_t calib_params;       // Parâmetros de calibração
    int32_t b5_comp;                         // Variável de compensação B5
} bmp180_dev_t;


// --- Protótipos de Função ---

/**
 * @brief Inicializa o driver do BMP180, configura o I2C e lê os dados de calibração.
 * @param i2c_bus_handle Handle do barramento I2C já inicializado.
 * @param out_dev Estrutura do driver BMP180 a ser inicializada.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t bmp180_init(i2c_master_bus_handle_t i2c_bus_handle, bmp180_dev_t *out_dev);

/**
 * @brief Lê a temperatura do sensor e retorna em 0.1 C (ex: 275 para 27.5 C).
 * @param dev Estrutura do driver BMP180.
 * @param temperature Ponteiro para armazenar a temperatura compensada.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t bmp180_read_temperature(bmp180_dev_t *dev, int32_t *temperature);

/**
 * @brief Lê a pressão do sensor e retorna em Pa.
 * @param dev Estrutura do driver BMP180.
 * @param pressure Ponteiro para armazenar a pressão compensada.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t bmp180_read_pressure(bmp180_dev_t *dev, int32_t *pressure);

/**
 * @brief Calcula a altitude baseada na pressão lida e na pressão ao nível do mar padrão (101325 Pa).
 * @param pressure Pressão atual em Pa.
 * @param altitude Ponteiro para armazenar a altitude calculada em metros.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t bmp180_read_altitude(int32_t pressure, float *altitude);

#endif // BMP180_H