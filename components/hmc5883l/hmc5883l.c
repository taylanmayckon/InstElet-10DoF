#include "hmc5883l.h"

#include "driver/i2c_master.h"
#include "esp_err.h"


void hmc5883l_init(i2c_master_dev_handle_t hmc5883l_handle){
    esp_err_t ret;
    uint8_t buf[2];

    // Configurando o registrador Config A
    buf[0] = HMC5883L_CONFIG_A;
    buf[1] = 0x70; // 8 amostras por medicao, taxa de 15Hz (66ms) e sem offset
    ret = i2c_master_transmit(hmc5883l_handle, buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE("HMC5883L", "Falha ao iniciar HMC5883L: %s", esp_err_to_name(ret));
        return;
    }

    // Configurando o registrador Config B
    buf[0] = HMC5883L_CONFIG_B;
    buf[1] = 0x20; // Ganho de 1.3Ga
    ret = i2c_master_transmit(hmc5883l_handle, buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE("HMC5883L", "Falha ao iniciar HMC5883L: %s", esp_err_to_name(ret));
        return;
    }

    // Configurando o registrador Mode 
    buf[0] = HMC5883L_MODE;
    buf[1] = 0x00; // Config para leituras continuas
    ret = i2c_master_transmit(hmc5883l_handle, buf, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE("HMC5883L", "Falha ao iniciar HMC5883L: %s", esp_err_to_name(ret));
        return;
    } 
}


esp_err_t hmc5883l_read_data(i2c_master_dev_handle_t hmc5883l_handle, axis_t *data){
    esp_err_t ret;
    
    uint8_t raw_data[6]; // O sensor retorna 6 bytes: X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB
    uint8_t start_reg = HMC5883L_DATA_X_MSB; // Leitura comeca nesse

    // Escrevendo o endereco do registrador onde a leitura deve comecar
    ret = i2c_master_transmit(hmc5883l_handle, &start_reg, 1, -1);
    if(ret != ESP_OK)
        return ret;

    //   Lendo 6 bytes de dados a partir desse registrador
    ret = i2c_master_receive(hmc5883l_handle, raw_data, 6, -1);
    if(ret != ESP_OK)
        return ret;
    
    // Combinando os Bytes MSB e LSB em valores int16_t (Na ordem X, Z, Y)
    int16_t x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    int16_t z = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    int16_t y = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    // Convertendo os dados brutos para Gauss (Ga)
    data->x = (float)x / HMC5883L_SCALE_FACTOR;
    data->y = (float)y / HMC5883L_SCALE_FACTOR;
    data->z = (float)z / HMC5883L_SCALE_FACTOR;

    return ESP_OK;
}