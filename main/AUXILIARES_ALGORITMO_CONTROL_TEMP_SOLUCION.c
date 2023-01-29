/**
 * @file AUXILIARES_ALGORITMO_CONTROL_TEMP_SOLUCION.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Funcionalidades del algoritmo de control de la temperatura de la solución nutritiva, como funciones callback o de inicialización.
 * @version 0.1
 * @date 2023-01-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//==================================| INCLUDES |==================================//

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "MQTT_PUBL_SUSCR.h"
#include "DS18B20_SENSOR.h"
#include "MEF_ALGORITMO_CONTROL_TEMP_SOLUCION.h"
#include "AUXILIARES_ALGORITMO_CONTROL_TEMP_SOLUCION.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *aux_control_temp_soluc_tag = "AUXILIAR_CONTROL_TEMP_SOLUCION";

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t Cliente_MQTT = NULL;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

void CallbackManualMode(void *pvParameters);
void CallbackManualModeNewActuatorState(void *pvParameters);
void CallbackGetTempSolucData(void *pvParameters);
void CallbackNewTempSolucSP(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función de callback que se ejecuta cuando llega un mensaje MQTT en el tópico
 *          correspondiente al modo MANUAL, para indicar si se quiere pasar a modo
 *          MANUAL o AUTOMÁTICO.
 * 
 * @param pvParameters 
 */
void CallbackManualMode(void *pvParameters)
{
    /**
     *  Se obtiene el mensaje del tópico de modo MANUAL o AUTO.
     */
    char buffer[10];
    mqtt_get_char_data_from_topic(MANUAL_MODE_MQTT_TOPIC, buffer);

    /**
     *  Dependiendo si el mensaje fue "MANUAL" o "AUTO", se setea o resetea
     *  la bandera correspondiente para señalizarle a la MEF de control de
     *  TDS que debe pasar al estado de modo MANUAL o AUTOMATICO.
     */
    if(!strcmp("MANUAL", buffer))
    {
        mef_temp_soluc_set_manual_mode_flag_value(1);
    }

    else if(!strcmp("AUTO", buffer))
    {
        mef_temp_soluc_set_manual_mode_flag_value(0);
    }

    /**
     * Se le envía un Task Notify a la tarea de la MEF de control de TDS.
     */
    xTaskNotifyGive(mef_temp_soluc_get_task_handle());
}



/**
 *  @brief  Función de callback que se ejecuta cuando llega un mensaje MQTT en el tópico
 *          correspondiente al estado de los actuadores de control de temperatura de solución
 *          en el modo MANUAL.
 *          Es decir, cuando estando en modo MANUAL, se quiere accionar el refrigerador
 *          o el calefactor de la solución.
 * 
 * @param pvParameters 
 */
void CallbackManualModeNewActuatorState(void *pvParameters)
{
    /**
     * Se le envía un Task Notify a la tarea de la MEF de control de temperatura de solución.
     */
    xTaskNotifyGive(mef_temp_soluc_get_task_handle());
}



/**
 *  @brief  Función de callback que se ejecuta cuando se completa una nueva medición del
 *          sensor de temperatura sumergible.
 * 
 * @param pvParameters 
 */
void CallbackGetTempSolucData(void *pvParameters)
{
    /**
     *  Variable donde se guarda el retorno de la función de obtención del valor
     *  del sensor, para verificar si se ejecutó correctamente o no.
     */
    esp_err_t return_status = ESP_FAIL;

    /**
     *  Se obtiene el nuevo dato de temperatura de la solución nutritiva.
     */
    DS18B20_sensor_temp_t temp_soluc;
    return_status = DS18B20_getTemp(&temp_soluc);

    /**
     *  Se verifica que la función de obtención del valor de temperatura de solución no haya retornado con error, y que el valor de 
     *  temperatura retornado este dentro del rango considerado como válido para dicha variable.
     * 
     *  En caso de que no se cumplan estas condiciones, se setea la bandera de error de sensor, utilizada por la MEF
     *  de control de temperatura de solución, y se le carga al valor de temperatura un código de error preestablecido 
     *  (-10), para que así, al leerse dicho valor, se pueda saber que ocurrió un error.
     */
    if(return_status == ESP_FAIL || temp_soluc < LIMITE_INFERIOR_RANGO_VALIDO_TEMPERATURA_SOLUC || temp_soluc > LIMITE_SUPERIOR_RANGO_VALIDO_TEMPERATURA_SOLUC)
    {
        temp_soluc = CODIGO_ERROR_SENSOR_TEMPERATURA_SOLUC;
        mef_temp_soluc_set_sensor_error_flag_value(1);
        ESP_LOGE(aux_control_temp_soluc_tag, "SOLUTION TEMPERATURE SENSOR ERROR DETECTED");
    }

    else
    {
        mef_tds_set_sensor_error_flag_value(0);
        ESP_LOGW(aux_control_temp_soluc_tag, "NEW MEASURMENT ARRIVED: %.3f", temp_soluc);
    }


    /**
     *  Si hay una conexión con el broker MQTT, se publica el valor de temperatura sensado.
     */
    if(mqtt_check_connection())
    {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%.3f", temp_soluc);
        esp_mqtt_client_publish(Cliente_MQTT, TEMP_SOLUC_MQTT_TOPIC, buffer, 0, 0, 0);
    }

    mef_temp_soluc_set_temp_soluc_value(temp_soluc);
}



/**
 *  @brief  Función de callback que se ejecuta cuando llega un mensaje al tópico MQTT
 *          correspondiente con un nuevo valor de set point de temperatura de la solución.
 * 
 * @param pvParameters 
 */
void CallbackNewTempSolucSP(void *pvParameters)
{
    /**
     *  Se obtiene el nuevo valor de SP de temperatura de solución.
     */
    DS18B20_sensor_temp_t SP_temp_soluc = 0;
    mqtt_get_float_data_from_topic(NEW_TEMP_SP_MQTT_TOPIC, &SP_temp_soluc);

    ESP_LOGI(aux_control_temp_soluc_tag, "NUEVO SP: %.3f", SP_temp_soluc);

    /**
     *  A partir del valor de SP de temperatura, se calculan los límites superior e inferior
     *  utilizados por el algoritmo de control de temperatura de solución, teniendo en cuenta el valor
     *  del delta de temperatura que se estableció.
     * 
     *  EJEMPLO:
     * 
     *  SP_TEMP_SOLUC = 25 °C
     *  DELTA_TEMP = 2 °C
     * 
     *  LIM_SUPERIOR_TEMP_SOLUC = SP_TEMP_SOLUC + DELTA_TEMP = 27 °C
     *  LIM_INFERIOR_TEMP_SOLUC = SP_TEMP_SOLUC - DELTA_TEMP = 23 °C
     */
    DS18B20_sensor_temp_t limite_inferior_temp_soluc, limite_superior_temp_soluc;
    limite_inferior_temp_soluc = SP_temp_soluc - mef_temp_soluc_get_delta_temp();
    limite_superior_temp_soluc = SP_temp_soluc + mef_temp_soluc_get_delta_temp();

    /**
     *  Se actualizan los límites superior e inferior de temperatura de solución en la MEF.
     */
    mef_temp_soluc_set_temp_control_limits(limite_inferior_temp_soluc, limite_superior_temp_soluc);

    ESP_LOGI(aux_control_temp_soluc_tag, "LIMITE INFERIOR: %.3f", limite_inferior_temp_soluc);
    ESP_LOGI(aux_control_temp_soluc_tag, "LIMITE SUPERIOR: %.3f", limite_superior_temp_soluc);
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el módulo de funciones auxiliares del algoritmo de control de
 *          temperatura de solución. 
 * 
 * @param mqtt_client   Handle del cliente MQTT.
 * @return esp_err_t 
 */
esp_err_t aux_control_temp_soluc_init(esp_mqtt_client_handle_t mqtt_client)
{
    /**
     *  Copiamos el handle del cliente MQTT en la variable interna.
     */
    Cliente_MQTT = mqtt_client;

    //=======================| INIT SENSOR DS18B20 |=======================//

    /**
     *  Se inicializa el sensor DS18B20. En caso de detectar error,
     *  se retorna con error.
     */
    if(DS18B20_sensor_init(GPIO_PIN_CO2_SENSOR) != ESP_OK)
    {
        ESP_LOGE(aux_control_temp_soluc_tag, "FAILED TO INITIALIZE DS18B20 SENSOR.");
        return ESP_FAIL;
    }

    /**
     *  Se asigna la función callback que será llamada al completarse una medición del
     *  sensor de temperatura sumergible.
     */
    DS18B20_callback_function_on_new_measurment(CallbackGetTempSolucData);

    //=======================| TÓPICOS MQTT |=======================//

    /**
     *  Se inicializa el array con los tópicos MQTT a suscribirse, junto
     *  con las funciones callback correspondientes que serán ejecutadas
     *  al llegar un nuevo dato en el tópico.
     */
    mqtt_topic_t list_of_topics[] = {
        [0].topic_name = NEW_TEMP_SP_MQTT_TOPIC,
        [0].topic_function_cb = CallbackNewTempSolucSP,
        [1].topic_name = MANUAL_MODE_MQTT_TOPIC,
        [1].topic_function_cb = CallbackManualMode,
        [2].topic_name = MANUAL_MODE_REFRIGERADOR_STATE_MQTT_TOPIC,
        [2].topic_function_cb = CallbackManualModeNewActuatorState,
        [3].topic_name = MANUAL_MODE_CALEFACTOR_STATE_MQTT_TOPIC,
        [3].topic_function_cb = CallbackManualModeNewActuatorState
    };

    /**
     *  Se realiza la suscripción a los tópicos MQTT y la asignación de callbacks correspondientes.
     */
    if(mqtt_suscribe_to_topics(list_of_topics, 4, Cliente_MQTT, 0) != ESP_OK)
    {
        ESP_LOGE(aux_control_temp_soluc_tag, "FAILED TO SUSCRIBE TO MQTT TOPICS.");
        return ESP_FAIL;
    }

    return ESP_OK;
}