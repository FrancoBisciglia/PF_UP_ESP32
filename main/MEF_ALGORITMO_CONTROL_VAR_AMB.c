/**
 * @file MEF_ALGORITMO_CONTROL_VAR_AMB.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Implementación de las diferentes MEF's del algoritmo de control de las variables ambientales del sistema, 
 *          que son la temperatura, humedad relativa y nivel de CO2 ambiente.
 * @version 0.1
 * @date 2023-01-16
 *
 * @copyright Copyright (c) 2023
 *
 */

//==================================| INCLUDES |==================================//

#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mqtt_client.h"

#include "MQTT_PUBL_SUSCR.h"
#include "DHT11_SENSOR.h"
#include "CO2_SENSOR.h"
#include "MCP23008.h"
#include "AUXILIARES_ALGORITMO_CONTROL_VAR_AMB.h"
#include "MEF_ALGORITMO_CONTROL_VAR_AMB.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *mef_var_amb_tag = "MEF_CONTROL_VAR_AMB";

/* Task Handle de la tarea del algoritmo de control de las variables ambientales del sistema. */
static TaskHandle_t xMefVarAmbAlgoritmoControlTaskHandle = NULL;

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t MefVarAmbClienteMQTT = NULL;

/* Variable donde se guarda el valor de la temperatura ambiente sensada en °C. */
static DHT11_sensor_temp_t mef_var_amb_temp = 25;
/* Límite inferior de temperatura ambiente del rango considerado como correcto en el algoritmo de control de variables ambientes, en °C. */
static DHT11_sensor_temp_t mef_var_amb_limite_inferior_temp = 18;
/* Límite superior de temperatura ambiente del rango considerado como correcto en el algoritmo de control de variables ambientes, en °C. */
static DHT11_sensor_temp_t mef_var_amb_limite_superior_temp = 28;
/* Ancho de la ventana de histeresis de temperatura ambiente, posicionada alrededor de los límites del rango considerado como correcto, en °C. */
static DHT11_sensor_temp_t mef_var_amb_ancho_ventana_hist_temp = 1;
/* Delta de temperatura considerado, en °C. */
static DHT11_sensor_temp_t mef_var_amb_delta_temp = 5;

/* Variable donde se guarda el valor de la humedad relativa ambiente sensada en %. */
static DHT11_sensor_hum_t mef_var_amb_hum = 0;
/* Límite superior de humedad ambiente del rango considerado como correcto en el algoritmo de control de variables ambientes, en %. */
static DHT11_sensor_hum_t mef_var_amb_limite_superior_hum = 85;
/* Ancho de la ventana de histeresis de humedad ambiente, posicionada alrededor de los límites del rango considerado como correcto, en %. */
static DHT11_sensor_hum_t mef_var_amb_ancho_ventana_hist_hum = 5;

/* Variable donde se guarda el valor del CO2 ambiente sensado en ppm. */
static CO2_sensor_ppm_t mef_var_amb_CO2 = 600;
/* Límite inferior de CO2 ambiente del rango considerado como correcto en el algoritmo de control de variables ambientes, en ppm. */
static CO2_sensor_ppm_t mef_var_amb_limite_inferior_CO2 = 400;
/* Ancho de la ventana de histeresis de CO2 ambiente, posicionada alrededor de los límites del rango considerado como correcto, en ppm. */
static CO2_sensor_ppm_t mef_var_amb_ancho_ventana_hist_CO2 = 50;

/* Bandera utilizada para controlar si se está o no en modo manual en el algoritmo de control de las variables ambientales. */
static bool mef_var_amb_manual_mode_flag = 0;
/* Bandera utilizada para controlar las transiciones con reset de la MEF de control de las variables ambientales. */
static bool mef_var_amb_reset_transition_flag_control_var_amb = 0;
/* Bandera utilizada para verificar si hubo error de sensado de temperatura del sensor DHT11. */
static bool mef_var_amb_temp_DHT11_sensor_error_flag = 0;
/* Bandera utilizada para verificar si hubo error de sensado de humedad relativa del sensor DHT11. */
static bool mef_var_amb_hum_DHT11_sensor_error_flag = 0;
/* Bandera utilizada para verificar si hubo error de sensado del sensor de CO2. */
static bool mef_var_amb_CO2_sensor_error_flag = 0;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

void MEFControlVarAmb(void);
void vTaskVarAmbControl(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función de la MEF de control de las variables ambientales del sistema, que son la temperatura,
 *          humedad relativa y nivel de CO2 ambiente. Mediante un control de ventana de histéresis, se 
 *          accionan los ventiladores o la calefacción según sea requerido para mantener la temperatura,
 *          humedad y nivel de CO2 del ambiente dentro de los límites inferior y superior establecidos.
 * 
 *          Respecto al CO2, solo se controla que el mismo no baje por debajo del valor promedio del 
 *          exterior, alrededor de 400 ppm, y se ventila el ambiente si esto sucede.
 * 
 *          Respecto a la humedad, solo se controla que la misma no suba por encima de un límite establecido,
 *          y se ventila el ambiente si esto sucede.
 */
void MEFControlVarAmb(void)
{
    /**
     * Variable que representa el estado de la MEF de control de las variables ambientales.
     */
    static estado_MEF_control_var_amb_t est_MEF_control_var_amb = VAR_AMB_CORRECTAS;

    /**
     *  Se controla si se debe hacer una transición con reset, caso en el cual se vuelve al estado
     *  de VAR_AMB_CORRECTAS, con los ventiladores y la calefacción apagados.
     */
    if (mef_var_amb_reset_transition_flag_control_var_amb)
    {
        set_relay_state(VENTILADORES, OFF);
        set_relay_state(CALEFACCION, OFF);

        /**
         *  Se publica el nuevo estado de la calefacción y ventiladores en los tópicos MQTT correspondientes.
         */
        if(mqtt_check_connection())
        {
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%s", "OFF");
            esp_mqtt_client_publish(MefVarAmbClienteMQTT, VENTILADORES_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            esp_mqtt_client_publish(MefVarAmbClienteMQTT, CALEFACCION_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
        }

        ESP_LOGW(mef_var_amb_tag, "VENTILADORES APAGADOS");
        ESP_LOGW(mef_var_amb_tag, "CALEFACCIÓN APAGADA");

        est_MEF_control_var_amb = VAR_AMB_CORRECTAS;
        mef_var_amb_reset_transition_flag_control_var_amb = 0;

        return;
    }

    switch (est_MEF_control_var_amb)
    {

    case VAR_AMB_CORRECTAS:

        /**
         *  En caso de que el nivel de CO2 caiga por debajo del límite inferior establecido, o que el nivel de humedad relativa suba
         *  por encima del límite superior establecido, se cambia al estado en donde se encienden los ventiladores. 
         * 
         *  Además, no debe estar levantada ninguna bandera de error de sensor.
         * 
         *  También, debe haber conexión con el broker MQTT.
         */
        if ((mef_var_amb_CO2 < (mef_var_amb_limite_inferior_CO2 - (mef_var_amb_ancho_ventana_hist_CO2 / 2)) 
            || mef_var_amb_hum > (mef_var_amb_limite_superior_hum + (mef_var_amb_ancho_ventana_hist_hum / 2)))
            && (mef_var_amb_temp >= (mef_var_amb_limite_inferior_temp - (mef_var_amb_ancho_ventana_hist_temp / 2)))
            && !mef_var_amb_temp_DHT11_sensor_error_flag && !mef_var_amb_hum_DHT11_sensor_error_flag && !mef_var_amb_CO2_sensor_error_flag
            && mqtt_check_connection())
        {
            set_relay_state(VENTILADORES, ON);
            /**
             *  Se publica el nuevo estado de la calefacción en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "ON");
                esp_mqtt_client_publish(MefVarAmbClienteMQTT, VENTILADORES_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

             ESP_LOGW(mef_var_amb_tag, "VENTILADORES ENCENDIDOS");

            est_MEF_control_var_amb = CO2_BAJO_O_HUM_AMB_ALTA;
        }


        /**
         *  En caso de que el nivel de temperatura ambiente baje por debajo del límite inferior de la ventana de histeresis
         *  centrada en el límite inferior del nivel de temperatura establecido, se cambia al estado en el cual se enciende
         *  la calefacción.
         * 
         *  Además, no debe estar levantada la bandera de error de sensor.
         * 
         *  También, debe haber conexión con el broker MQTT.
         */
        else if (mef_var_amb_temp < (mef_var_amb_limite_inferior_temp - (mef_var_amb_ancho_ventana_hist_temp / 2)) 
            && !mef_var_amb_temp_DHT11_sensor_error_flag
            && mqtt_check_connection())
        {
            set_relay_state(CALEFACCION, ON);
            /**
             *  Se publica el nuevo estado de la calefacción en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "ON");
                esp_mqtt_client_publish(MefVarAmbClienteMQTT, CALEFACCION_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }
             
            ESP_LOGW(mef_var_amb_tag, "CALEFACCIÓN ENCENDIDA");

            est_MEF_control_var_amb = TEMP_AMB_BAJA;
        }


        /**
         *  En caso de que el nivel de temperatura ambiente suba por encima del límite superior de la ventana de histeresis
         *  centrada en el límite superior del nivel de temperatura establecido, se cambia al estado en el cual se encienden
         *  los ventiladores.
         * 
         *  Además, no debe estar levantada la bandera de error de sensor.
         * 
         *  También, debe haber conexión con el broker MQTT.
         */
        else if (mef_var_amb_temp > (mef_var_amb_limite_superior_temp + (mef_var_amb_ancho_ventana_hist_temp / 2)) 
            && !mef_var_amb_temp_DHT11_sensor_error_flag
            && mqtt_check_connection())
        {
            set_relay_state(VENTILADORES, ON);
            /**
             *  Se publica el nuevo estado de la calefacción en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "ON");
                esp_mqtt_client_publish(MefVarAmbClienteMQTT, VENTILADORES_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_var_amb_tag, "VENTILADORES ENCENDIDOS");

            est_MEF_control_var_amb = TEMP_AMB_ELEVADA;
        }

        break;
    
    case CO2_BAJO_O_HUM_AMB_ALTA:

        /**
         *  En caso de que el nivel de CO2 suba por encima del límite inferior establecido, o que el nivel de humedad relativa baje
         *  por debajo del límite superior establecido, o que la temperatura ambiente baje por debajo del límite inferior establecido,
         *  ya que se le da prioridad a dicha variable, o que se levante alguna de las banderas de error de sensado, se cambia al 
         *  estado en donde se apagan los actuadores.
         * 
         *  También, si se produce una deconexión del broker MQTT, al no poder recibir nuevos datos, se vuelve al estado
         *  con los actuadores apagados.
         */
        if (((mef_var_amb_CO2 > (mef_var_amb_limite_inferior_CO2 + (mef_var_amb_ancho_ventana_hist_CO2 / 2)) 
            && mef_var_amb_hum < (mef_var_amb_limite_superior_hum - (mef_var_amb_ancho_ventana_hist_hum / 2)))
            || mef_var_amb_temp < (mef_var_amb_limite_inferior_temp - (mef_var_amb_ancho_ventana_hist_temp / 2)))
            || (mef_var_amb_temp_DHT11_sensor_error_flag || mef_var_amb_hum_DHT11_sensor_error_flag || mef_var_amb_CO2_sensor_error_flag)
            || !mqtt_check_connection())
        {
            set_relay_state(VENTILADORES, OFF);
            /**
             *  Se publica el nuevo estado de la calefacción en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "OFF");
                esp_mqtt_client_publish(MefVarAmbClienteMQTT, VENTILADORES_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_var_amb_tag, "VENTILADORES APAGADOS");

            est_MEF_control_var_amb = VAR_AMB_CORRECTAS;
        }

        break;

    case TEMP_AMB_BAJA:

        /**
         *  Cuando el nivel de temperatura sobrepase el límite superior de la ventana de histeresis centrada en el límite inferior
         *  del rango de temperatura correcto, se transiciona al estado con los actuadores apagados. 
         * 
         *  Además, si se levanta la bandera de error de sensor, se transiciona a dicho estado.
         * 
         *  También, si se produce una deconexión del broker MQTT, al no poder recibir nuevos datos, se vuelve al estado
         *  con los actuadores apagados.
         */
        if (mef_var_amb_temp > (mef_var_amb_limite_inferior_temp + (mef_var_amb_ancho_ventana_hist_temp / 2))
            || mef_var_amb_temp_DHT11_sensor_error_flag
            || !mqtt_check_connection())
        {
            set_relay_state(CALEFACCION, OFF);
            /**
             *  Se publica el nuevo estado de la calefacción en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "OFF");
                esp_mqtt_client_publish(MefVarAmbClienteMQTT, CALEFACCION_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_var_amb_tag, "CALEFACCIÓN APAGADA");

            est_MEF_control_var_amb = VAR_AMB_CORRECTAS;
        }

        break;

    case TEMP_AMB_ELEVADA:

        /**
         *  Cuando el nivel de temperatura caiga por debajo del límite inferior de la ventana de histeresis centrada en el límite 
         *  superior del rango de temperatura correcto, se transiciona al estado con los actuadores apagados.
         * 
         *  Además, si se levanta la bandera de error de sensor, se transiciona a dicho estado.
         * 
         *  También, si se produce una deconexión del broker MQTT, al no poder recibir nuevos datos, se vuelve al estado
         *  con los actuadores apagados.
         */
        if (mef_var_amb_temp < (mef_var_amb_limite_superior_temp - (mef_var_amb_ancho_ventana_hist_temp / 2))
            || mef_var_amb_temp_DHT11_sensor_error_flag
            || !mqtt_check_connection())
        {
            set_relay_state(VENTILADORES, OFF);
            /**
             *  Se publica el nuevo estado de la calefacción en el tópico MQTT correspondiente.
             */
            if(mqtt_check_connection())
            {
                char buffer[10];
                snprintf(buffer, sizeof(buffer), "%s", "OFF");
                esp_mqtt_client_publish(MefVarAmbClienteMQTT, VENTILADORES_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
            }

            ESP_LOGW(mef_var_amb_tag, "VENTILADORES APAGADOS");

            est_MEF_control_var_amb = VAR_AMB_CORRECTAS;
        }

        break;
    }
}



/**
 * @brief   Tarea encargada del control de la MEF de mayor jerarquía del algoritmo de control de las variables
 *          ambientales, que son la temperatura, humedad relativa y nivel de CO2 ambiente.
 * 
 * @param pvParameters 
 */
void vTaskVarAmbControl(void *pvParameters)
{
    /**
     * Variable que representa el estado de la MEF de jerarquía superior del algoritmo de control de las variables ambientales.
     */
    static estado_MEF_principal_control_var_amb_t est_MEF_principal = ALGORITMO_CONTROL_VAR_AMB;

    while (1)
    {
        /**
         *  Se realiza un Notify Take a la espera de señales que indiquen:
         *
         *  -Que se debe pasar a modo MANUAL o modo AUTO.
         *  -Que estando en modo MANUAL, se deba cambiar el estado de los ventiladores o la calefacción.
         *
         *  Además, se le coloca un timeout para evaluar las transiciones de las MEFs periódicamente, en caso
         *  de que no llegue ninguna de las señales mencionadas, y para controlar los valores de sensado que
         *  llegan.
         */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        switch (est_MEF_principal)
        {

        case ALGORITMO_CONTROL_VAR_AMB:

            /**
             *  En caso de que se levante la bandera de modo MANUAL, se debe transicionar a dicho estado,
             *  en donde el accionamiento de los ventiladores y la calefacción será manejado por el usuario
             *  vía mensajes MQTT.
             */
            if (mef_var_amb_manual_mode_flag)
            {
                est_MEF_principal = MODO_MANUAL_CONTROL_VAR_AMB;
                mef_var_amb_reset_transition_flag_control_var_amb = 1;
            }

            MEFControlVarAmb();

            break;

        case MODO_MANUAL_CONTROL_VAR_AMB:

            /**
             *  En caso de que se baje la bandera de modo MANUAL, se debe transicionar nuevamente al estado
             *  de modo AUTOMATICO, en donde se controlan las variables ambientales a partir de los
             *  valores de los sensores DHT11 y de CO2 de las unidades secundarias y los ventiladores y
             *  calefacción.
             * 
             *  Además, en caso de que se produzca una desconexión del broker MQTT, se vuelve también
             *  al modo AUTOMATICO, y se limpia la bandera de modo MANUAL.
             */
            if (!mef_var_amb_manual_mode_flag || !mqtt_check_connection())
            {
                est_MEF_principal = ALGORITMO_CONTROL_VAR_AMB;

                mef_var_amb_manual_mode_flag = 0;

                /**
                 *  Se setea la bandera de reset de la MEF de control de variables ambientales de modo
                 *  que se resetee el estado de los actuadores correspondientes.
                 */
                mef_var_amb_reset_transition_flag_control_var_amb = 1;

                break;
            }

            /**
             *  Se obtiene el nuevo estado en el que deben estar los ventiladores y la calefacción, y se accionan
             *  los relés correspondientes.
             */
            float manual_mode_ventiladores_state = -1;
            float manual_mode_calefaccion_state = -1;

            mqtt_get_float_data_from_topic(MANUAL_MODE_VENTILADORES_STATE_MQTT_TOPIC, &manual_mode_ventiladores_state);
            mqtt_get_float_data_from_topic(MANUAL_MODE_CALEFACCION_STATE_MQTT_TOPIC, &manual_mode_calefaccion_state);

            if (manual_mode_ventiladores_state == 0 || manual_mode_ventiladores_state == 1)
            {
                set_relay_state(VENTILADORES, manual_mode_ventiladores_state);
                /**
                 *  Se publica el nuevo estado de los ventiladores en el tópico MQTT correspondiente.
                 */
                if(mqtt_check_connection())
                {
                    char buffer[10];

                    if(manual_mode_ventiladores_state == 0)
                    {
                        snprintf(buffer, sizeof(buffer), "%s", "OFF");    
                    }

                    else if(manual_mode_ventiladores_state == 1)
                    {
                        snprintf(buffer, sizeof(buffer), "%s", "ON");
                    }
                    
                    esp_mqtt_client_publish(MefVarAmbClienteMQTT, VENTILADORES_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
                }

                ESP_LOGW(mef_var_amb_tag, "MANUAL MODE VENTILADORES: %.0f", manual_mode_ventiladores_state);
            }

            if (manual_mode_calefaccion_state == 0 || manual_mode_calefaccion_state == 1)
            {
                set_relay_state(CALEFACCION, manual_mode_calefaccion_state);
                /**
                 *  Se publica el nuevo estado de la calefacción en el tópico MQTT correspondiente.
                 */
                if(mqtt_check_connection())
                {
                    char buffer[10];

                    if(manual_mode_calefaccion_state == 0)
                    {
                        snprintf(buffer, sizeof(buffer), "%s", "OFF");    
                    }

                    else if(manual_mode_calefaccion_state == 1)
                    {
                        snprintf(buffer, sizeof(buffer), "%s", "ON");
                    }
                    
                    esp_mqtt_client_publish(MefVarAmbClienteMQTT, CALEFACCION_STATE_MQTT_TOPIC, buffer, 0, 0, 0);
                }

                ESP_LOGW(mef_var_amb_tag, "MANUAL MODE CALEFACCIÓN: %.0f", manual_mode_calefaccion_state);
            }

            break;
        }
    }
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el módulo de MEFs del algoritmo de control de variables ambientales.
 *
 * @param mqtt_client   Handle del cliente MQTT.
 * @return esp_err_t
 */
esp_err_t mef_var_amb_init(esp_mqtt_client_handle_t mqtt_client)
{
    /**
     *  Copiamos el handle del cliente MQTT en la variable interna.
     */
    MefVarAmbClienteMQTT = mqtt_client;

    //=======================| CREACION TAREAS |=======================//

    /**
     *  Se crea la tarea mediante la cual se controlará la transicion de las
     *  MEFs del algoritmo de control de variables ambientales.
     */
    if (xMefVarAmbAlgoritmoControlTaskHandle == NULL)
    {
        xTaskCreate(
            vTaskVarAmbControl,
            "vTaskVarAmbControl",
            4096,
            NULL,
            2,
            &xMefVarAmbAlgoritmoControlTaskHandle);

        /**
         *  En caso de que el handle sea NULL, implica que no se pudo crear la tarea, y se retorna con error.
         */
        if (xMefVarAmbAlgoritmoControlTaskHandle == NULL)
        {
            ESP_LOGE(mef_var_amb_tag, "Failed to create vTaskVarAmbControl task.");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}



/**
 * @brief   Función que devuelve el Task Handle de la tarea principal del algoritmo de control de variables ambientales.
 *
 * @return TaskHandle_t Task Handle de la tarea.
 */
TaskHandle_t mef_var_amb_get_task_handle(void)
{
    return xMefVarAmbAlgoritmoControlTaskHandle;
}



/**
 * @brief   Función que devuelve el valor del delta de temperatura ambiente establecido.
 *
 * @return DHT11_sensor_temp_t Delta de temperatura en °C.
 */
DHT11_sensor_temp_t mef_var_amb_get_delta_temp(void)
{
    return mef_var_amb_delta_temp;
}



/**
 * @brief   Función para establecer nuevos límites del rango de temperatura ambiente considerado como correcto para el 
 *          algoritmo de control de variables ambientales.
 *
 * @param nuevo_limite_inferior_temp_amb   Límite inferior del rango.
 * @param nuevo_limite_superior_temp_amb   Límite superior del rango.
 */
void mef_var_amb_set_temp_control_limits(DHT11_sensor_temp_t nuevo_limite_inferior_temp_amb, DHT11_sensor_temp_t nuevo_limite_superior_temp_amb)
{
    mef_var_amb_limite_inferior_temp = nuevo_limite_inferior_temp_amb;
    mef_var_amb_limite_superior_temp = nuevo_limite_superior_temp_amb;
}



/**
 * @brief   Función para actualizar el valor de temperatura ambiente sensado.
 *
 * @param nuevo_valor_temp_amb Nuevo valor de temperatura ambiente, en °C.
 */
void mef_var_amb_set_temp_amb_value(DHT11_sensor_temp_t nuevo_valor_temp_amb)
{
    mef_var_amb_temp = nuevo_valor_temp_amb;
}



/**
 * @brief   Función para actualizar el valor de humedad relativa ambiente sensado.
 *
 * @param nuevo_valor_hum_amb Nuevo valor de humedad relativa ambiente, en %.
 */
void mef_var_amb_set_hum_amb_value(DHT11_sensor_hum_t nuevo_valor_hum_amb)
{
    mef_var_amb_hum = nuevo_valor_hum_amb;
}



/**
 * @brief   Función para actualizar el valor de CO2 ambiente sensado.
 *
 * @param nuevo_valor_CO2_amb Nuevo valor de CO2 ambiente, en ppm.
 */
void mef_var_amb_set_CO2_amb_value(CO2_sensor_ppm_t nuevo_valor_CO2_amb)
{
    mef_var_amb_CO2 = nuevo_valor_CO2_amb;
}



/**
 * @brief   Función para cambiar el estado de la bandera de modo MANUAL, utilizada por
 *          la MEF para cambiar entre estado de modo MANUAL y AUTOMATICO.
 *
 * @param manual_mode_flag_state    Estado de la bandera.
 */
void mef_var_amb_set_manual_mode_flag_value(bool manual_mode_flag_state)
{
    mef_var_amb_manual_mode_flag = manual_mode_flag_state;
}



/**
 * @brief   Función para cambiar el estado de la bandera de error de temperatura del sensor DHT11.
 *
 * @param sensor_error_flag_state    Estado de la bandera.
 */
void mef_var_amb_set_temp_DHT11_sensor_error_flag_value(bool sensor_error_flag_state)
{
    mef_var_amb_temp_DHT11_sensor_error_flag = sensor_error_flag_state;
}



/**
 * @brief   Función para cambiar el estado de la bandera de error de humedad del sensor DHT11.
 *
 * @param sensor_error_flag_state    Estado de la bandera.
 */
void mef_var_amb_set_hum_DHT11_sensor_error_flag_value(bool sensor_error_flag_state)
{
    mef_var_amb_hum_DHT11_sensor_error_flag = sensor_error_flag_state;
}



/**
 * @brief   Función para cambiar el estado de la bandera de error del sensor de CO2.
 *
 * @param sensor_error_flag_state    Estado de la bandera.
 */
void mef_var_amb_set_CO2_sensor_error_flag_value(bool sensor_error_flag_state)
{
    mef_var_amb_CO2_sensor_error_flag = sensor_error_flag_state;
}