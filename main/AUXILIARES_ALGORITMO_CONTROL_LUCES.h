/*

    Funcionalidades del algoritmo de control de las luces de las unidades secundarias.

*/

#ifndef AUXILIARES_ALGORITMO_CONTROL_LUCES_H_
#define AUXILIARES_ALGORITMO_CONTROL_LUCES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "mqtt_client.h"

/*============================[DEFINES AND MACROS]=====================================*/

/**
 *  Definición de los tópicos MQTT a suscribirse.
 */
#define NEW_LIGHTS_ON_TIME_MQTT_TOPIC   "/Tiempos/Luces/Tiempo_encendido"
#define LIGHTS_MANUAL_MODE_MQTT_TOPIC  "/Luces/Modo"
#define MANUAL_MODE_LIGHTS_STATE_MQTT_TOPIC    "/Luces/Modo_Manual/Luces"
#define LIGHTS_STATE_MQTT_TOPIC   "Actuadores/Luces"

/**
 *  Constante de conversión de horas a ms:
 * 
 *  -1 hora = 60 min
 *  -1 min = 60 seg
 *  -1 seg = 1000 ms
 * 
 *  => 1 hora = 60*60*1000 = 3.600.000 ms
 * 
 */
//#define HOURS_TO_MS 3600000
#define HOURS_TO_MS 1000

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t aux_control_luces_init(esp_mqtt_client_handle_t mqtt_client);
TimerHandle_t aux_control_luces_get_timer_handle(void);

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // AUXILIARES_ALGORITMO_CONTROL_LUCES_H_