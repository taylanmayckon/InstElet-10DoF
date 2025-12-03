#include "wifi_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <string.h>
#include <stdbool.h>

static const char *TAG = "WIFI";
static bool is_connected = false;
static esp_netif_ip_info_t ip_info;

// -> EVENT HANDLERS

// Evento de WiFi (conexao, desconexao, tentativa de conexao...)
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if(event_base == WIFI_EVENT){
        switch(event_id){
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                ESP_LOGI(TAG, "Wi-Fi iniciado, conectando...");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                is_connected = false;
                ESP_LOGW(TAG, "Desconectado do ponto de acesso. Tentando reconectar...");
                esp_wifi_connect();
                break;
            default:
                break;
        }
    }
}

// Evento de IP (quando recebe IP da rede)
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ip_info = event->ip_info;
        is_connected = true;
        ESP_LOGI(TAG, "Conectado! IP Obtido: " IPSTR, IP2STR(&ip_info.ip));
    }
}





// -> FUNÇOES GERADAS POR MIM PARA INICIALIZAÇÃO E CONTROLE DO WIFI

// Inicializando o Wi-Fi da ESP
esp_err_t wifi_manager_init(const char *ssid, const char *password){
    // Iniciando NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // Iniciando netif
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    // Config do WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL);

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, password);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "Inicializado com sucesso.");
    return ESP_OK;
}

// Verifica se o Wi-Fi está conectado
bool wifi_manager_is_connected(void){
    return is_connected;
}

// Obtém o endereço IP atual
char *wifi_manager_get_ip(void){
    if (is_connected) {
        static char ip_str[16];
        sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
        return ip_str;
    }
    return NULL;
}