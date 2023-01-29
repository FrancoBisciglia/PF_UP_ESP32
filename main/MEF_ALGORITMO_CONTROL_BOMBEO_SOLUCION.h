/*

    MEF del algoritmo de control del bombeo de la solución nutritiva.

*/

#ifndef MEF_ALGORITMO_CONTROL_BOMBEO_SOLUCION_H_
#define MEF_ALGORITMO_CONTROL_BOMBEO_SOLUCION_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

/*============================[DEFINES AND MACROS]=====================================*/

/* Tiempos estándar de encendido y apagado de la bomba de solución, en min. */
#define MEF_BOMBEO_TIEMPO_BOMBA_ON  15
#define MEF_BOMBEO_TIEMPO_BOMBA_OFF  60

/* Periodo de tiempo de control de flujo de solución en los canales, en ms. */
#define MEF_BOMBEO_TIEMPO_CONTROL_SENSOR_FLUJO 5000

/**
 *  Enumeración correspondiente a los actuadores del control de bombeo de solución..
 * 
 *  NOTA: CUANDO SE SEPA BIEN QUÉ RELÉ SE ASOCIA A QUÉ ACTUADOR, MODIFICAR LOS NÚMEROS.
 */
typedef enum actuadores_control_bombeo_soluc{
    BOMBA = 6,
};


/**
 *  Enumeración correspondiente a los estados de la MEF de control de bombeo de la solución.
 */
typedef enum {
    ESPERA_BOMBEO = 0,
    BOMBEO_SOLUCION,
} estado_MEF_control_bombeo_soluc_t;


/**
 *  Enumeración correspondiente a los estados de la MEF principal del algoritmo de control de bombeo de solución.
 */
typedef enum {
    ALGORITMO_CONTROL_BOMBEO_SOLUC = 0,
    MODO_MANUAL,
} estado_MEF_principal_control_bombeo_soluc_t;


/**
 *  Tipo de variable que representa los tiempos de bombeo de solución, en minutos.
 */
typedef unsigned int pump_time_t

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t mef_bombeo_init(esp_mqtt_client_handle_t mqtt_client);
TaskHandle_t mef_bombeo_get_task_handle(void);
void mef_bombeo_set_pump_on_time_min(pump_time_t tiempo_bomba_on);
void mef_bombeo_set_pump_off_time_min(pump_time_t tiempo_bomba_off);
void mef_bombeo_set_manual_mode_flag_value(bool manual_mode_flag_state);
void mef_bombeo_set_timer_flag_value(bool timer_flag_state);

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // MEF_ALGORITMO_CONTROL_BOMBEO_SOLUCION_H_