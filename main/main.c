#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mqtt_client.h"

#include "esp_check.h"

#include "MEF_ALGORITMO_CONTROL_LUCES.h"
#include "AUXILIARES_ALGORITMO_CONTROL_LUCES.h"

#include "MEF_ALGORITMO_CONTROL_VAR_AMB.h"
#include "AUXILIARES_ALGORITMO_CONTROL_VAR_AMB.h"

#include "MQTT_PUBL_SUSCR.h"
#include "WiFi_STA.h"
#include "MCP23008.h"

#include "DEBUG_DEFINITIONS.h"


/* Tag para imprimir información en el LOG. */
static const char *TAG = "MAIN";


void app_main(void)
{
    //=======================| CONEXION WIFI |=======================//

    // wifi_network_t network = {
    //     .ssid = "Claro2021",
    //     .pass = "Lavalle1402abcd",
    // };

    wifi_network_t network = {
        .ssid = "MOVISTAR WIFI4196",
        .pass = "yoot7267",
    };

    connect_wifi(&network);

    while(!wifi_check_connection()){vTaskDelay(pdMS_TO_TICKS(100));}

    //=======================| CONEXION MQTT |=======================//

    esp_mqtt_client_handle_t Cliente_MQTT = NULL;

    // mqtt_initialize_and_connect("mqtt://192.168.100.4:1883", &Cliente_MQTT);
    mqtt_initialize_and_connect("mqtt://192.168.201.173:1883", &Cliente_MQTT);

    while(!mqtt_check_connection()){vTaskDelay(pdMS_TO_TICKS(100));}

    //=======================| INIT MCP23008 |=======================//

    ESP_ERROR_CHECK_WITHOUT_ABORT(MCP23008_init());

    //=======================| INIT ALGORITMO CONTROL LUCES |=======================//

    #ifdef DEBUG_ALGORITMO_CONTROL_LUCES
    aux_control_luces_init(Cliente_MQTT);
    mef_luces_init(Cliente_MQTT);
    #endif

    //=======================| INIT ALGORITMO CONTROL VAR AMB |=======================//

    #ifdef DEBUG_ALGORITMO_CONTROL_VARIABLES_AMBIENTALES
    aux_control_var_amb_init(Cliente_MQTT);
    mef_var_amb_init(Cliente_MQTT);
    #endif

}

