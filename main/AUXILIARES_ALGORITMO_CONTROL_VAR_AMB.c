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
#include "DHT11_SENSOR.h"
#include "CO2_SENSOR.h"
#include "MEF_ALGORITMO_CONTROL_VAR_AMB.h"
#include "AUXILIARES_ALGORITMO_CONTROL_VAR_AMB.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *aux_control_var_amb_tag = "AUXILIAR_CONTROL_VAR_AMB";

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t Cliente_MQTT = NULL;

/* Lista de tópicos en donde las unidades secundarias publican los datos de temperatura ambiente. */
static const char aux_control_var_amb_topicos_datos_temp[AUX_CONTROL_VAR_AMB_CANT_UNIDADES_SECUNDARIAS][100] = {
    TEMP_AMB_MQTT_TOPIC
}

/* Lista de tópicos en donde las unidades secundarias publican los datos de humedad ambiente. */
static const char aux_control_var_amb_topicos_datos_hum[AUX_CONTROL_VAR_AMB_CANT_UNIDADES_SECUNDARIAS][100] = {
    HUM_AMB_MQTT_TOPIC
}

/* Lista de tópicos en donde las unidades secundarias publican los datos de CO2 ambiente. */
static const char aux_control_var_amb_topicos_datos_co2[AUX_CONTROL_VAR_AMB_CANT_UNIDADES_SECUNDARIAS][100] = {
    CO2_AMB_MQTT_TOPIC
}

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

void CallbackManualMode(void *pvParameters);
void CallbackManualModeNewActuatorState(void *pvParameters);
void CallbackGetTempAmbData(void *pvParameters);
void CallbackGetHumAmbData(void *pvParameters);
void CallbackGetCO2AmbData(void *pvParameters);
void CallbackNewTempAmbSP(void *pvParameters);

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
     *  variables ambientales que debe pasar al estado de modo MANUAL o AUTOMATICO.
     */
    if(!strcmp("MANUAL", buffer))
    {
        mef_var_amb_set_manual_mode_flag_value(1);
    }

    else if(!strcmp("AUTO", buffer))
    {
        mef_var_amb_set_manual_mode_flag_value(0);
    }

    /**
     * Se le envía un Task Notify a la tarea de la MEF de control de variables ambientales.
     */
    xTaskNotifyGive(mef_var_amb_get_task_handle());
}



/**
 *  @brief  Función de callback que se ejecuta cuando llega un mensaje MQTT en el tópico
 *          correspondiente al estado de los actuadores de control de variables ambientales
 *          en el modo MANUAL.
 * 
 *          Es decir, cuando estando en modo MANUAL, se quiere accionar los ventiladores
 *          o la calefacción.
 * 
 * @param pvParameters 
 */
void CallbackManualModeNewActuatorState(void *pvParameters)
{
    /**
     * Se le envía un Task Notify a la tarea de la MEF de control de variables ambientales.
     */
    xTaskNotifyGive(mef_var_amb_get_task_handle());
}



/**
 *  @brief  Función de callback que se ejecuta cuando se completa una nueva medición de
 *          temperatura de alguno de los sensores DHT11 de las unidades secundarias.
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
    DHT11_sensor_temp_t *temperaturas_unidades_sec = NULL;

    /**
     *  Variable que representa la cantidad de datos que llegan desde las unidades secundarias
     *  que son considerados como correctos, esto es, que no tienen el código de error 
     *  "CODIGO_ERROR_SENSOR_DHT11_TEMP_AMB".
     */
    unsigned int cantidad_datos_correctos = 0;

    /**
     *  Se obtienen los datos de temperatura ambiente de cada una de las unidades secundarias.
     */
    for(int i = 1; i <= AUX_CONTROL_VAR_AMB_CANT_UNIDADES_SECUNDARIAS; i++)
    {
        DHT11_sensor_temp_t buffer = CODIGO_ERROR_SENSOR_DHT11_TEMP_AMB;
        mqtt_get_float_data_from_topic(aux_control_var_amb_topicos_datos_temp[i-1], &buffer);

        /**
         *  Se controla que el dato obtenido no tenga el código de error, en caso de que sí, 
         *  no se considera dicho dato para el posterior cálculo de la mediana.
         */
        if(buffer != CODIGO_ERROR_SENSOR_DHT11_TEMP_AMB)
        {
            if(temperaturas_unidades_sec == NULL)
            {
                temperaturas_unidades_sec = calloc(1, sizeof(DHT11_sensor_temp_t));
            }

            else
            {
                temperaturas_unidades_sec = (DHT11_sensor_temp_t*) realloc(temperaturas_unidades_sec, cantidad_datos_correctos * sizeof(DHT11_sensor_temp_t));
            }

            temperaturas_unidades_sec[cantidad_datos_correctos] = buffer;
            cantidad_datos_correctos++;
        }
    }

    /**
     *  Se ordenan los datos obtenidos de menor a mayor.
     */
    SortData(temperaturas_unidades_sec, cantidad_datos_correctos);

    /**
     *  Se obtiene la mediana de los datos recopilados.
     */
    DHT11_sensor_temp_t mediana_temperaturas_unidades_sec;

    if ( cantidad_datos_correctos % 2 = = 0 )  
        mediana_temperaturas_unidades_sec = (temperaturas_unidades_sec[cantidad_datos_correctos / 2] + temperaturas_unidades_sec[(cantidad_datos_correctos / 2) + 1]) / 2.0;  
    
    else  
        mediana_temperaturas_unidades_sec = temperaturas_unidades_sec[(cantidad_datos_correctos / 2) + 1];


    /**
     *  Se le pasa la mediana del array de datos obtenido a la MEF de control de variables ambientales.
     */
    mef_var_amb_set_temp_amb_value(mediana_temperaturas_unidades_sec);
}



/**
 *  @brief  Función de callback que se ejecuta cuando se completa una nueva medición de
 *          humedad relativa de alguno de los sensores DHT11 de las unidades secundarias.
 * 
 * @param pvParameters 
 */
void CallbackGetHumAmbData(void *pvParameters)
{
    /**
     *  Se inicializa un array dinámico en donde se irán guardando los datos de
     *  humedad relativa ambiente sensados por cada una de las unidades secundarias,
     *  para luego obtener la mediana del mismo.
     */
    DHT11_sensor_hum_t *humedades_unidades_sec = NULL;

    /**
     *  Variable que representa la cantidad de datos que llegan desde las unidades secundarias
     *  que son considerados como correctos, esto es, que no tienen el código de error 
     *  "CODIGO_ERROR_SENSOR_DHT11_HUM_AMB".
     */
    unsigned int cantidad_datos_correctos = 0;

    /**
     *  Se obtienen los datos de temperatura ambiente de cada una de las unidades secundarias.
     */
    for(int i = 1; i <= AUX_CONTROL_VAR_AMB_CANT_UNIDADES_SECUNDARIAS; i++)
    {
        DHT11_sensor_hum_t buffer = CODIGO_ERROR_SENSOR_DHT11_HUM_AMB;
        mqtt_get_float_data_from_topic(aux_control_var_amb_topicos_datos_hum[i-1], &buffer);

        /**
         *  Se controla que el dato obtenido no tenga el código de error, en caso de que sí, 
         *  no se considera dicho dato para el posterior cálculo de la mediana.
         */
        if(buffer != CODIGO_ERROR_SENSOR_DHT11_HUM_AMB)
        {
            if(humedades_unidades_sec == NULL)
            {
                humedades_unidades_sec = calloc(1, sizeof(DHT11_sensor_hum_t));
            }

            else
            {
                humedades_unidades_sec = (DHT11_sensor_hum_t*) realloc(humedades_unidades_sec, cantidad_datos_correctos * sizeof(DHT11_sensor_hum_t));
            }

            humedades_unidades_sec[cantidad_datos_correctos] = buffer;
            cantidad_datos_correctos++;
        }
    }

    /**
     *  Se ordenan los datos obtenidos de menor a mayor.
     */
    SortData(humedades_unidades_sec, cantidad_datos_correctos);

    /**
     *  Se obtiene la mediana de los datos recopilados.
     */
    DHT11_sensor_hum_t mediana_humedades_unidades_sec;

    if ( cantidad_datos_correctos % 2 = = 0 )  
        mediana_humedades_unidades_sec = (humedades_unidades_sec[cantidad_datos_correctos / 2] + humedades_unidades_sec[(cantidad_datos_correctos / 2) + 1]) / 2.0;  
    
    else  
        mediana_humedades_unidades_sec = humedades_unidades_sec[(cantidad_datos_correctos / 2) + 1];


    /**
     *  Se le pasa la mediana del array de datos obtenido a la MEF de control de variables ambientales.
     */
    mef_var_amb_set_hum_amb_value(mediana_humedades_unidades_sec);
}



/**
 *  @brief  Función de callback que se ejecuta cuando se completa una nueva medición de
 *          CO2 de alguno de los sensores de CO2 de las unidades secundarias.
 * 
 * @param pvParameters 
 */
void CallbackGetCO2AmbData(void *pvParameters)
{
    /**
     *  Se inicializa un array dinámico en donde se irán guardando los datos de
     *  CO2 ambiente sensados por cada una de las unidades secundarias,
     *  para luego obtener la mediana del mismo.
     */
    CO2_sensor_ppm_t *nivel_CO2_unidades_sec = NULL;

    /**
     *  Variable que representa la cantidad de datos que llegan desde las unidades secundarias
     *  que son considerados como correctos, esto es, que no tienen el código de error 
     *  "CODIGO_ERROR_SENSOR_CO2".
     */
    unsigned int cantidad_datos_correctos = 0;

    /**
     *  Se obtienen los datos de temperatura ambiente de cada una de las unidades secundarias.
     */
    for(int i = 1; i <= AUX_CONTROL_VAR_AMB_CANT_UNIDADES_SECUNDARIAS; i++)
    {
        CO2_sensor_ppm_t buffer = CODIGO_ERROR_SENSOR_CO2;
        mqtt_get_float_data_from_topic(aux_control_var_amb_topicos_datos_co2[i-1], &buffer);

        /**
         *  Se controla que el dato obtenido no tenga el código de error, en caso de que sí, 
         *  no se considera dicho dato para el posterior cálculo de la mediana.
         */
        if(buffer != CODIGO_ERROR_SENSOR_CO2)
        {
            if(nivel_CO2_unidades_sec == NULL)
            {
                nivel_CO2_unidades_sec = calloc(1, sizeof(CO2_sensor_ppm_t));
            }

            else
            {
                nivel_CO2_unidades_sec = (CO2_sensor_ppm_t*) realloc(nivel_CO2_unidades_sec, cantidad_datos_correctos * sizeof(CO2_sensor_ppm_t));
            }

            nivel_CO2_unidades_sec[cantidad_datos_correctos] = buffer;
            cantidad_datos_correctos++;
        }
    }

    /**
     *  Se ordenan los datos obtenidos de menor a mayor.
     */
    SortData(nivel_CO2_unidades_sec, cantidad_datos_correctos);

    /**
     *  Se obtiene la mediana de los datos recopilados.
     */
    CO2_sensor_ppm_t mediana_nivel_CO2_unidades_sec;

    if ( cantidad_datos_correctos % 2 = = 0 )  
        mediana_nivel_CO2_unidades_sec = (nivel_CO2_unidades_sec[cantidad_datos_correctos / 2] + nivel_CO2_unidades_sec[(cantidad_datos_correctos / 2) + 1]) / 2.0;  
    
    else  
        mediana_nivel_CO2_unidades_sec = nivel_CO2_unidades_sec[(cantidad_datos_correctos / 2) + 1];


    /**
     *  Se le pasa la mediana del array de datos obtenido a la MEF de control de variables ambientales.
     */
    mef_var_amb_set_CO2_amb_value(mediana_nivel_CO2_unidades_sec);
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
 *          correspondiente con un nuevo valor de set point de temperatura ambiente.
 * 
 * @param pvParameters 
 */
void CallbackNewTempAmbSP(void *pvParameters)
{
    /**
     *  Se obtiene el nuevo valor de SP de temperatura ambiente.
     */
    DHT11_sensor_temp_t SP_temp_amb = 0;
    mqtt_get_float_data_from_topic(NEW_TEMP_SP_MQTT_TOPIC, &SP_temp_amb);

    ESP_LOGI(aux_control_var_amb_tag, "NUEVO SP: %.3f", SP_temp_amb);

    /**
     *  A partir del valor de SP de temperatura, se calculan los límites superior e inferior
     *  utilizados por el algoritmo de control de variables ambientales, teniendo en cuenta el valor
     *  del delta de temperatura que se estableció.
     * 
     *  EJEMPLO:
     * 
     *  SP_temp_amb = 25 °C
     *  DELTA_TEMP = 2 °C
     * 
     *  LIM_SUPERIOR_TEMP_AMB = SP_temp_amb + DELTA_TEMP = 27 °C
     *  LIM_INFERIOR_TEMP_AMB = SP_temp_amb - DELTA_TEMP = 23 °C
     */
    DHT11_sensor_temp_t limite_inferior_temp_amb, limite_superior_temp_amb;
    limite_inferior_temp_amb = SP_temp_amb - mef_var_amb_get_delta_temp();
    limite_superior_temp_amb = SP_temp_amb + mef_var_amb_get_delta_temp();

    /**
     *  Se actualizan los límites superior e inferior de temperatura ambiente en la MEF.
     */
    mef_var_amb_set_temp_control_limits(limite_inferior_temp_amb, limite_superior_temp_amb);

    ESP_LOGI(aux_control_var_amb_tag, "LIMITE INFERIOR TEMP AMB: %.3f", limite_inferior_temp_amb);
    ESP_LOGI(aux_control_var_amb_tag, "LIMITE SUPERIOR TEMP AMB: %.3f", limite_superior_temp_amb);
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el módulo de funciones auxiliares del algoritmo de control de
 *          variables ambientales. 
 * 
 * @param mqtt_client   Handle del cliente MQTT.
 * @return esp_err_t 
 */
esp_err_t mef_var_amb_init(esp_mqtt_client_handle_t mqtt_client)
{
    /**
     *  Copiamos el handle del cliente MQTT en la variable interna.
     */
    Cliente_MQTT = mqtt_client;

    //=======================| TÓPICOS MQTT |=======================//

    /**
     *  Se inicializa el array con los tópicos MQTT a suscribirse, junto
     *  con las funciones callback correspondientes que serán ejecutadas
     *  al llegar un nuevo dato en el tópico.
     */
    mqtt_topic_t list_of_topics[] = {
        [0].topic_name = NEW_TEMP_SP_MQTT_TOPIC,
        [0].topic_function_cb = CallbackNewTempAmbSP,
        [1].topic_name = MANUAL_MODE_MQTT_TOPIC,
        [1].topic_function_cb = CallbackManualMode,
        [2].topic_name = MANUAL_MODE_VENTILADORES_STATE_MQTT_TOPIC,
        [2].topic_function_cb = CallbackManualModeNewActuatorState,
        [3].topic_name = MANUAL_MODE_CALEFACCION_STATE_MQTT_TOPIC,
        [3].topic_function_cb = CallbackManualModeNewActuatorState
        [4].topic_name = TEMP_AMB_MQTT_TOPIC,
        [4].topic_function_cb = CallbackGetTempAmbData
        [5].topic_name = HUM_AMB_MQTT_TOPIC,
        [5].topic_function_cb = CallbackGetHumAmbData
        [6].topic_name = CO2_AMB_MQTT_TOPIC,
        [6].topic_function_cb = CallbackGetCO2AmbData
    };

    /**
     *  Se realiza la suscripción a los tópicos MQTT y la asignación de callbacks correspondientes.
     */
    if(mqtt_suscribe_to_topics(list_of_topics, 7, Cliente_MQTT, 0) != ESP_OK)
    {
        ESP_LOGE(aux_control_var_amb_tag, "FAILED TO SUSCRIBE TO MQTT TOPICS.");
        return ESP_FAIL;
    }

    return ESP_OK;
}