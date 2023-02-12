/*

    MEF del algoritmo de control de las luces de todas las unidades secundarias.

*/

#ifndef MEF_ALGORITMO_CONTROL_LUCES_H_
#define MEF_ALGORITMO_CONTROL_LUCES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "MCP23008.h"

/*============================[DEFINES AND MACROS]=====================================*/

/* Tiempos estándar de encendido y apagado de las luces de las unidades secundarias, en horas y en ciclos de 24 horas. */
#define MEF_LUCES_TIEMPO_LUCES_ON  2
#define MEF_LUCES_TIEMPO_LUCES_OFF  3

/**
 *  Enumeración correspondiente a los actuadores del control de las luces de las unidades secundarias.
 * 
 *  NOTA: CUANDO SE SEPA BIEN QUÉ RELÉ SE ASOCIA A QUÉ ACTUADOR, MODIFICAR LOS NÚMEROS.
 */
typedef enum actuadores_control_luces{
    LUCES = RELE_4,
};


/**
 *  Enumeración correspondiente a los estados de la MEF de control de las luces de las unidades secundarias.
 */
typedef enum {
    ESPERA_ILUMINACION_CULTIVOS = 0,
    ILUMINACION_CULTIVOS,
} estado_MEF_control_luces_t;


/**
 *  Enumeración correspondiente a los estados de la MEF principal del algoritmo de control de las luces de las unidades secundarias.
 */
typedef enum {
    ALGORITMO_CONTROL_LUCES = 0,
    MODO_MANUAL,
} estado_MEF_principal_control_luces_t;


/**
 *  Tipo de variable que representa los tiempos de iluminación de las unidades secundarias, en horas y en ciclos de 24 horas.
 */
typedef float light_time_t;

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t mef_luces_init(esp_mqtt_client_handle_t mqtt_client);
TaskHandle_t mef_luces_get_task_handle(void);
void mef_luces_set_lights_on_time_hours(light_time_t tiempo_luces_on);
void mef_luces_set_manual_mode_flag_value(bool manual_mode_flag_state);
void mef_luces_set_timer_flag_value(bool timer_flag_state);

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // MEF_ALGORITMO_CONTROL_LUCES_H_