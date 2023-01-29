/*

    WiFi library

*/

#ifndef WiFi_STA_H_
#define WiFi_STA_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"

/*============================[DEFINES AND MACROS]=====================================*/

/* Macros de mensajes de verificación de funcionamiento de la librería. */
#define WIFI_SUCCES 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCES 1 << 0
#define TCP_FAILURE 1 << 1

/**
 *  Estructura con la información de la red WiFi a conectarse:
 *  
 *  -ssid: Nombre de la red WiFi.
 *  -pass: Contraseña de la red WiFi.
 */
typedef struct
{
    char ssid[50]; //Nombre de la red WiFi
    char pass[50]; //Contraseña de la red WiFi
} wifi_network_t;

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

extern void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
extern void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
esp_err_t connect_wifi(wifi_network_t* wifi_network);
bool wifi_check_connection();

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // WiFi_STA_H_