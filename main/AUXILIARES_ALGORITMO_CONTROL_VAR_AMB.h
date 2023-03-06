/*

    Funcionalidades del algoritmo de control de las variables ambientales, que son la temperatura, humedad relativa
    y nivel de CO2 ambiente.

*/

#ifndef AUXILIARES_ALGORITMO_CONTROL_VAR_AMB_H_
#define AUXILIARES_ALGORITMO_CONTROL_VAR_AMB_H_

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
 *  Definición de los tópicos MQTT a publicar o suscribirse.
 */
#define NEW_TEMP_SP_MQTT_TOPIC   "NodeRed/Sensores ambientales/Temperatura/SP"
#define VAR_AMB_MANUAL_MODE_MQTT_TOPIC  "/VarAmb/Modo"
#define MANUAL_MODE_VENTILADORES_STATE_MQTT_TOPIC    "/VarAmb/Modo_Manual/Ventiladores"
#define MANUAL_MODE_CALEFACCION_STATE_MQTT_TOPIC   "/VarAmb/Modo_Manual/Calefaccion"
#define VENTILADORES_STATE_MQTT_TOPIC   "Actuadores/Ventiladores"
#define CALEFACCION_STATE_MQTT_TOPIC   "Actuadores/Calefaccion"

#define CO2_AMB_MQTT_TOPIC  "Sensores ambientales/CO2"
#define TEMP_AMB_MQTT_TOPIC "Sensores ambientales/Temperatura"
#define HUM_AMB_MQTT_TOPIC  "Sensores ambientales/Humedad"

/* Código de error que se carga en el valor de temperatura al detectar un error de sensado. */
#define CODIGO_ERROR_SENSOR_DHT11_TEMP_AMB -5
/* Código de error que se carga en el valor de humedad relativa al detectar un error de sensado. */
#define CODIGO_ERROR_SENSOR_DHT11_HUM_AMB -6
/* Código de error que se carga en el valor de CO2 al detectar un error de sensado. */
#define CODIGO_ERROR_SENSOR_CO2 -5

/* Cantidad de unidades secundarias presentes en el sistema. */
#define AUX_CONTROL_VAR_AMB_CANT_UNIDADES_SECUNDARIAS 1

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t aux_control_var_amb_init(esp_mqtt_client_handle_t mqtt_client);

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // AUXILIARES_ALGORITMO_CONTROL_VAR_AMB_H_