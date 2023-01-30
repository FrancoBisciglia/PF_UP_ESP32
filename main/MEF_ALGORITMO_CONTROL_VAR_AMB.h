/*

    MEF del algoritmo de control de la temperatura de la solución nutritiva.

*/

#ifndef MEF_ALGORITMO_CONTROL_TEMP_SOLUCION_H_
#define MEF_ALGORITMO_CONTROL_TEMP_SOLUCION_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "DS18B20_SENSOR.h"

/*============================[DEFINES AND MACROS]=====================================*/

/**
 *  Enumeración correspondiente al número de relés del calefactor y refrigerador de control
 *  de la temperatura de la solución nutritiva.
 * 
 *  NOTA: CUANDO SE SEPA BIEN QUÉ RELÉ SE ASOCIA A QUÉ ACTUADOR, MODIFICAR LOS NÚMEROS.
 */
typedef enum actuadores_control_temp_soluc{
    CALEFACTOR_SOLUC = 3,
    REFRIGERADOR_SOLUC,
};


/**
 *  Enumeración correspondiente a los estados de la MEF de control de temperatura de la solución.
 */
typedef enum {
    TEMP_SOLUCION_CORRECTA = 0,
    TEMP_SOLUCION_BAJA,
    TEMP_SOLUCION_ELEVADA,
} estado_MEF_control_temp_soluc_t;


/**
 *  Enumeración correspondiente a los estados de la MEF principal del algoritmo de control de
 *  la temperatura de la solución.
 */
typedef enum {
    ALGORITMO_CONTROL_TEMP_SOLUC = 0,
    MODO_MANUAL,
} estado_MEF_principal_control_temp_soluc_t;

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t mef_temp_soluc_init(esp_mqtt_client_handle_t mqtt_client);
TaskHandle_t mef_temp_soluc_get_task_handle(void);
DS18B20_sensor_temp_t mef_temp_soluc_get_delta_temp(void);
void mef_temp_soluc_set_temp_control_limits(DS18B20_sensor_temp_t nuevo_limite_inferior_temp_soluc, DS18B20_sensor_temp_t nuevo_limite_superior_temp_soluc);
void mef_temp_soluc_set_temp_soluc_value(DS18B20_sensor_temp_t nuevo_valor_temp_soluc);
void mef_temp_soluc_set_manual_mode_flag_value(bool manual_mode_flag_state);
void mef_temp_soluc_set_sensor_error_flag_value(bool sensor_error_flag_state);

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // MEF_ALGORITMO_CONTROL_TEMP_SOLUCION_H_