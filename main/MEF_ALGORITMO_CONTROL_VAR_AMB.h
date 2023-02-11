/*

    MEF del algoritmo de control de las variables ambientales del sistema, que son la temperatura, humedad
    relativa y nivel de CO2 ambiente.

*/

#ifndef MEF_ALGORITMO_CONTROL_VAR_AMB_H_
#define MEF_ALGORITMO_CONTROL_VAR_AMB_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "DHT11_SENSOR.h"
#include "CO2_SENSOR.h"

/*============================[DEFINES AND MACROS]=====================================*/

/**
 *  Enumeración correspondiente al número de relés de los ventiladores y la calefaccion de control
 *  de la temperatura, humedad y CO2 ambiente del sistema.
 * 
 *  NOTA: CUANDO SE SEPA BIEN QUÉ RELÉ SE ASOCIA A QUÉ ACTUADOR, MODIFICAR LOS NÚMEROS.
 */
typedef enum actuadores_control_var_amb{
    VENTILADORES = 3,
    CALEFACCION = 4,
};


/**
 *  Enumeración correspondiente a los estados de la MEF de control de la temperatura, humedad relativa
 *  y CO2 del ambiente.
 */
typedef enum {
    VAR_AMB_CORRECTAS = 0,
    CO2_BAJO_O_HUM_AMB_ALTA,
    TEMP_AMB_BAJA,
    TEMP_AMB_ELEVADA,
} estado_MEF_control_var_amb_t;


/**
 *  Enumeración correspondiente a los estados de la MEF principal del algoritmo de control de
 *  la temperatura, humedad relativa y CO2 ambiente.
 */
typedef enum {
    ALGORITMO_CONTROL_VAR_AMB = 0,
    MODO_MANUAL_CONTROL_VAR_AMB,
} estado_MEF_principal_control_var_amb_t;

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t mef_var_amb_init(esp_mqtt_client_handle_t mqtt_client);
TaskHandle_t mef_var_amb_get_task_handle(void);
DHT11_sensor_temp_t mef_var_amb_get_delta_temp(void);
void mef_var_amb_set_temp_control_limits(DHT11_sensor_temp_t nuevo_limite_inferior_temp_amb, DHT11_sensor_temp_t nuevo_limite_superior_temp_amb);
void mef_var_amb_set_temp_amb_value(DHT11_sensor_temp_t nuevo_valor_temp_amb);
void mef_var_amb_set_hum_amb_value(DHT11_sensor_hum_t nuevo_valor_hum_amb);
void mef_var_amb_set_CO2_amb_value(CO2_sensor_ppm_t nuevo_valor_CO2_amb);
void mef_var_amb_set_manual_mode_flag_value(bool manual_mode_flag_state);
void mef_var_amb_set_temp_DHT11_sensor_error_flag_value(bool sensor_error_flag_state);
void mef_var_amb_set_hum_DHT11_sensor_error_flag_value(bool sensor_error_flag_state);
void mef_var_amb_set_CO2_sensor_error_flag_value(bool sensor_error_flag_state);

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // MEF_ALGORITMO_CONTROL_VAR_AMB_H_