#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mqtt_client.h"

#include "MEF_ALGORITMO_CONTROL_LUCES.h"
#include "AUXILIARES_ALGORITMO_CONTROL_LUCES.h"

#include "MQTT_PUBL_SUSCR.h"
#include "WiFi_STA.h"
#include "MCP23008.h"

void app_main(void)
{
    //=======================| CONEXION WIFI |=======================//

    wifi_network_t network = {
        .ssid = "Claro2021",
        .pass = "Lavalle1402abcd",
    };

    connect_wifi(&network);

    while(!wifi_check_connection()){vTaskDelay(pdMS_TO_TICKS(100));}

    //=======================| CONEXION MQTT |=======================//

    esp_mqtt_client_handle_t Cliente_MQTT = NULL;

    mqtt_initialize_and_connect("mqtt://192.168.100.4:1883", &Cliente_MQTT);

    while(!mqtt_check_connection()){vTaskDelay(pdMS_TO_TICKS(100));}

    //=======================| INIT MCP23008 |=======================//

    MCP23008_init();

    //=======================| INIT ALGORITMO CONTROL LUCES |=======================//

    aux_control_luces_init(Cliente_MQTT);
    mef_luces_init(Cliente_MQTT);
}
