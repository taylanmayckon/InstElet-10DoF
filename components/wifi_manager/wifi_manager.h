#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>

esp_err_t wifi_manager_init(const char *ssid, const char *password);
bool wifi_manager_is_connected(void);
char *wifi_manager_get_ip(void);

#endif
