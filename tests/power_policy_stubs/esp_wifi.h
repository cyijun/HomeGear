#pragma once
enum wifi_ps_type_t { WIFI_PS_NONE, WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM };
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
esp_err_t esp_wifi_get_ps(wifi_ps_type_t *mode);
