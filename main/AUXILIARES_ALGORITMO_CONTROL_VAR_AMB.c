/**
 * @file AUXILIARES_ALGORITMO_CONTROL_VAR_AMB.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Funcionalidades del algoritmo de control de las variables ambientales (tempratura, humedad, CO2), 
 *          como funciones callback o de inicialización.
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
#include "MEF_ALGORITMO_CONTROL_VAR_AMB.h"
#include "AUXILIARES_ALGORITMO_CONTROL_VAR_AMB.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *aux_control_var_amb_tag = "AUXILIAR_CONTROL_VAR_AMB";

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t Cliente_MQTT = NULL;

/* Cantidad de unidades secundarias presentes en el sistema. */
unsigned int aux_control_var_amb_cantidad_unidades_sec = 0;

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
void CallbackGetTempAmbData(void *pvParameters)
{
    /**
     *  Se inicializa un array dinámico en donde se irán guardando los datos de
     *  temperatura ambiente sensados por cada una de las unidades secundarias,
     *  para luego obtener la mediana del mismo.
     */
    float *temperaturas_unidades_sec = NULL;

    /**
     *  Variable que representa la cantidad de datos que llegan desde las unidades secundarias
     *  que son considerados como correctos, esto es, que no tienen el código de error 
     *  "CODIGO_ERROR_SENSOR_DHT11_TEMP_AMB".
     */
    unsigned int cantidad_datos_correctos = 0;

    /**
     *  Se obtienen los datos de temperatura ambiente de cada una de las unidades secundarias.
     */
    for(int i = 1; i < aux_control_var_amb_cantidad_unidades_sec; i++)
    {
        float buffer = CODIGO_ERROR_SENSOR_DHT11_TEMP_AMB;
        mqtt_get_float_data_from_topic(TEMP_AMB_MQTT_TOPIC, &buffer);

        /**
         *  Se controla que el dato obtenido no tenga el código de error, en caso de que sí, 
         *  no se considera dicho dato para el posterior cálculo de la mediana.
         */
        if(buffer != CODIGO_ERROR_SENSOR_DHT11_TEMP_AMB)
        {
            if(temperaturas_unidades_sec == NULL)
            {
                temperaturas_unidades_sec = calloc(1, sizeof(float));
            }

            else
            {
                temperaturas_unidades_sec = (float*) realloc(temperaturas_unidades_sec, cantidad_datos_correctos * sizeof(float));
            }

            temperaturas_unidades_sec[cantidad_datos_correctos] = buffer;
            cantidad_datos_correctos++;
        }
    }

    SortData(temperaturas_unidades_sec, cantidad_datos_correctos);

    ME QUEDE ACA

    if ( cantidad_datos_correctos % 2 = = 0 )  
        median = ( temperaturas_unidades_sec[ cantidad_datos_correctos / 2 ] + temperaturas_unidades_sec[ cantidad_datos_correctos / 2 + 1 ] ) / 2 . 0 ;  
    
    else  
        median = temperaturas_unidades_sec[ cantidad_datos_correctos / 2 + 1 ] ;
}



/**
 * @brief   Función utilizada para ordenar de forma ascendente un array de datos.
 * 
 * @param data_array    El array de datos a ordenar.
 * @param cantidad_datos    Cantidad de datos que posee el array.
 */
void SortData(float *data_array, unsigned int cantidad_datos)
{
    float temp;

    /**
     *  Se ordenan los valores del menor al mayor con el método de la burbuja.
     */
    if(cantidad_datos > 1)
    {
        for(int i = 1; i < cantidad_datos; i++)
        {
            for(int j=0; j < cantidad_datos-i; j++)
            {
                if(data_array[j] > data_array[j+1])
                {
                    temp = data_array[j];
                    data_array[j]=data_array[j+1];
                    data_array[j+1]=temp;
                }
            }  
        }
    }
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

    ESP_LOGI(aux_control_var_amb_tag, "NUEVO SP: %.3f", SP_temp_soluc);

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

    ESP_LOGI(aux_control_var_amb_tag, "LIMITE INFERIOR: %.3f", limite_inferior_temp_soluc);
    ESP_LOGI(aux_control_var_amb_tag, "LIMITE SUPERIOR: %.3f", limite_superior_temp_soluc);
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
        ESP_LOGE(aux_control_var_amb_tag, "FAILED TO INITIALIZE DS18B20 SENSOR.");
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
        ESP_LOGE(aux_control_var_amb_tag, "FAILED TO SUSCRIBE TO MQTT TOPICS.");
        return ESP_FAIL;
    }

    return ESP_OK;
}