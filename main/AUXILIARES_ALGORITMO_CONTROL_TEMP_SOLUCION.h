/*

    Funcionalidades del algoritmo de control de la temperatura de la solución nutritiva.

*/

#ifndef AUXILIARES_ALGORITMO_CONTROL_TEMP_SOLUCION_H_
#define AUXILIARES_ALGORITMO_CONTROL_TEMP_SOLUCION_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

/*============================[DEFINES AND MACROS]=====================================*/

/**
 *  Definición de los tópicos MQTT a suscribirse.
 */
#define NEW_TEMP_SP_MQTT_TOPIC   "/SP/TempSoluc"
#define TEMP_SOLUC_MQTT_TOPIC    "Sensores de solucion/Temperatura"
#define MANUAL_MODE_MQTT_TOPIC  "/TempSoluc/Modo"
#define MANUAL_MODE_REFRIGERADOR_STATE_MQTT_TOPIC    "/TempSoluc/Modo_Manual/Refrigerador"
#define MANUAL_MODE_CALEFACTOR_STATE_MQTT_TOPIC   "/TempSoluc/Modo_Manual/Calefactor"

/**
 *  Definición del rango de temperatura de la solución considerado como válido, en °C.
 */
#define LIMITE_INFERIOR_RANGO_VALIDO_TEMPERATURA_SOLUC 10
#define LIMITE_SUPERIOR_RANGO_VALIDO_TEMPERATURA_SOLUC 40

/* Código de error que se carga en el valor de temperatura de la solución al detectar un error de sensado. */
#define CODIGO_ERROR_SENSOR_TEMPERATURA_SOLUC -10

/* Definición del pin GPIO al cual está conectado el sensor DS18B20. */
#define GPIO_PIN_CO2_SENSOR 18

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t aux_control_temp_soluc_init(esp_mqtt_client_handle_t mqtt_client);

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // AUXILIARES_ALGORITMO_CONTROL_TEMP_SOLUCION_H_