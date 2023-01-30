/**
 * @file MEF_ALGORITMO_CONTROL_TEMP_SOLUCION.c
 * @author Franco Bisciglia, David Kündinger
 * @brief   Implementación de las diferentes MEF's del algoritmo de control de la temperatura de la solución.
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

#include "MQTT_PUBL_SUSCR.h"
#include "DS18B20_SENSOR.h"
#include "MCP23008.h"
#include "AUXILIARES_ALGORITMO_CONTROL_TEMP_SOLUCION.h"
#include "MEF_ALGORITMO_CONTROL_TEMP_SOLUCION.h"

//==================================| MACROS AND TYPDEF |==================================//

//==================================| INTERNAL DATA DEFINITION |==================================//

/* Tag para imprimir información en el LOG. */
static const char *mef_temp_soluc_tag = "MEF_CONTROL_TEMP_SOLUCION";

/* Task Handle de la tarea del algoritmo de control de la temperatura de la solución. */
static TaskHandle_t xMefTempSolucAlgoritmoControlTaskHandle = NULL;

/* Handle del cliente MQTT. */
static esp_mqtt_client_handle_t MefTempSolucClienteMQTT = NULL;

/* Variable donde se guarda el valor de la temperatura de la solución sensada en °C. */
static DS18B20_sensor_temp_t mef_temp_soluc_temp = 0;
/* Límite inferior del rango considerado como correcto en el algoritmo de control de la temperatura de la solución en °C. */
static DS18B20_sensor_temp_t mef_temp_soluc_limite_inferior_temp_soluc = 18;
/* Límite superior del rango considerado como correcto en el algoritmo de control de la temperatura de la solución en °C. */
static DS18B20_sensor_temp_t mef_temp_soluc_limite_superior_temp_soluc = 28;
/* Ancho de la ventana de histeresis posicionada alrededor de los límites del rango considerado como correcto, en °C. */
static DS18B20_sensor_temp_t mef_temp_soluc_ancho_ventana_hist = 1;
/* Delta de temperatura considerado, en °C. */
static DS18B20_sensor_temp_t mef_temp_soluc_delta_temp_soluc = 2;

/* Bandera utilizada para controlar si se está o no en modo manual en el algoritmo de control de temperatura solución. */
static bool mef_temp_soluc_manual_mode_flag = 0;
/* Bandera utilizada para controlar las transiciones con reset de la MEF de control de temperatura solución. */
static bool mef_temp_soluc_reset_transition_flag_control_temp = 0;
/* Bandera utilizada para verificar si hubo error de sensado del sensor de temperatura de la solución. */
static bool mef_temp_soluc_sensor_error_flag = 0;

//==================================| EXTERNAL DATA DEFINITION |==================================//

//==================================| INTERNAL FUNCTIONS DECLARATION |==================================//

void MEFControlTempSoluc(void);
void vTaskSolutionTempControl(void *pvParameters);

//==================================| INTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función de la MEF de control de la temperatura en la solución nutritiva. Mediante un control
 *          de ventana de histéresis, se accionan el calefactor o refrigerador según sea requerido para
 *          mantener la temperatura de la solución dentro de los límites inferior y superior establecidos.
 */
void MEFControlTempSoluc(void)
{
    /**
     * Variable que representa el estado de la MEF de control de temperatura de la solución.
     */
    static estado_MEF_control_temp_soluc_t est_MEF_control_temp_soluc = TEMP_SOLUCION_CORRECTA;

    /**
     *  Se controla si se debe hacer una transición con reset, caso en el cual se vuelve al estado
     *  de TEMP_SOLUCION_CORRECTA, con el refrigerador y calefactor de solución apagados.
     */
    if (mef_temp_soluc_reset_transition_flag_control_temp)
    {
        est_MEF_control_temp_soluc = TEMP_SOLUCION_CORRECTA;
        mef_temp_soluc_reset_transition_flag_control_temp = 0;
    }

    switch (est_MEF_control_temp_soluc)
    {

    case TEMP_SOLUCION_CORRECTA:

        /**
         *  En caso de que el nivel de temperatura de la solución caiga por debajo del límite inferior de la ventana de histeresis
         *  centrada en el límite inferior de nivel de temperatura establecido, se cambia al estado en el cual se enciende
         *  el calefactor de solución. Además, no debe estar levantada la bandera de error de sensor.
         */
        if (mef_temp_soluc_temp < (mef_temp_soluc_limite_inferior_temp_soluc - (mef_temp_soluc_ancho_ventana_hist / 2)) && !mef_temp_soluc_sensor_error_flag)
        {
            set_relay_state(CALEFACTOR_SOLUC, ON);
            est_MEF_control_temp_soluc = TEMP_SOLUCION_BAJA;
        }

        /**
         *  En caso de que el nivel de temperatura de la solución suba por encima del límite superior de la ventana de histeresis
         *  centrada en el límite superior de nivel de temperatura establecido, se cambia al estado en el cual se enciende
         *  el refrigerador de solución. Además, no debe estar levantada la bandera de error de sensor.
         */
        if (mef_temp_soluc_temp > (mef_temp_soluc_limite_superior_temp_soluc + (mef_temp_soluc_ancho_ventana_hist / 2)) && !mef_temp_soluc_sensor_error_flag)
        {
            set_relay_state(REFRIGERADOR_SOLUC, ON);
            est_MEF_control_temp_soluc = TEMP_SOLUCION_ELEVADA;
        }

        break;

    case TEMP_SOLUCION_BAJA:

        /**
         *  Cuando el nivel de temperatura sobrepase el límite superior de la ventana de histeresis centrada en el límite inferior
         *  del rango de temperatura correcto, se transiciona al estado con el refrigerador y calefactor apagados. Además, si se 
         *  levanta la bandera de error de sensor, se transiciona a dicho estado.
         */
        if (mef_temp_soluc_temp > (mef_temp_soluc_limite_inferior_temp_soluc + (mef_temp_soluc_ancho_ventana_hist / 2)) || mef_temp_soluc_sensor_error_flag)
        {
            set_relay_state(CALEFACTOR_SOLUC, OFF);
            est_MEF_control_temp_soluc = TEMP_SOLUCION_CORRECTA;
        }

        break;

    case TEMP_SOLUCION_ELEVADA:

        /**
         *  Cuando el nivel de temperatura caiga por debajo del límite inferior de la ventana de histeresis centrada en el límite 
         *  superior del rango de temperatura correcto, se transiciona al estado con el refrigerador y calefactor apagados. Además, 
         *  si se levanta la bandera de error de sensor, se transiciona a dicho estado.
         */
        if (mef_temp_soluc_temp < (mef_temp_soluc_limite_superior_temp_soluc - (mef_temp_soluc_ancho_ventana_hist / 2)) || mef_temp_soluc_sensor_error_flag)
        {
            set_relay_state(REFRIGERADOR_SOLUC, OFF);
            est_MEF_control_temp_soluc = TEMP_SOLUCION_CORRECTA;
        }

        break;
    }
}

void vTaskSolutionTempControl(void *pvParameters)
{
    /**
     * Variable que representa el estado de la MEF de jerarquía superior del algoritmo de control del temperatura de la solución.
     */
    static estado_MEF_principal_control_temp_soluc_t est_MEF_principal = ALGORITMO_CONTROL_TEMP_SOLUC;

    while (1)
    {
        /**
         *  Se realiza un Notify Take a la espera de señales que indiquen:
         *
         *  -Que se debe pasar a modo MANUAL o modo AUTO.
         *  -Que estando en modo MANUAL, se deba cambiar el estado del refrigerador o calefactor.
         *
         *  Además, se le coloca un timeout para evaluar las transiciones de las MEFs periódicamente, en caso
         *  de que no llegue ninguna de las señales mencionadas, y para controlar el nivel de temperatura que llega
         *  desde el sensor de temperatura sumergible.
         */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        switch (est_MEF_principal)
        {

        case ALGORITMO_CONTROL_TEMP_SOLUC:

            /**
             *  En caso de que se levante la bandera de modo MANUAL, se debe transicionar a dicho estado,
             *  en donde el accionamiento del calefactor y refrigerador será manejado por el usuario
             *  vía mensajes MQTT.
             */
            if (mef_temp_soluc_manual_mode_flag)
            {
                est_MEF_principal = MODO_MANUAL;
                mef_temp_soluc_reset_transition_flag_control_temp = 1;
            }

            MEFControlTempSoluc();

            break;

        case MODO_MANUAL:

            /**
             *  En caso de que se baje la bandera de modo MANUAL, se debe transicionar nuevamente al estado
             *  de modo AUTOMATICO, en donde se controla la temperatura de la solución a partir de los
             *  valores del sensor de temperatura sumergible y el calefactor y refrigerador.
             */
            if (!mef_temp_soluc_manual_mode_flag)
            {
                est_MEF_principal = ALGORITMO_CONTROL_TEMP_SOLUC;
                break;
            }

            /**
             *  Se obtiene el nuevo estado en el que deben estar el calefactor y refrigerador, y se accionan
             *  los relés correspondientes.
             */
            float manual_mode_refrigerador_state = -1;
            float manual_mode_calefactor_state = -1;
            mqtt_get_float_data_from_topic(MANUAL_MODE_VALVULA_AUM_TDS_STATE_MQTT_TOPIC, &manual_mode_refrigerador_state);
            mqtt_get_float_data_from_topic(MANUAL_MODE_VALVULA_DISM_TDS_STATE_MQTT_TOPIC, &manual_mode_calefactor_state);

            if (manual_mode_refrigerador_state == 0 || manual_mode_refrigerador_state == 1)
            {
                set_relay_state(REFRIGERADOR_SOLUC, manual_mode_refrigerador_state);
                ESP_LOGW(mef_temp_soluc_tag, "MANUAL MODE REFRIGERADOR: %.0f", manual_mode_refrigerador_state);
            }

            if (manual_mode_calefactor_state == 0 || manual_mode_calefactor_state == 1)
            {
                set_relay_state(CALEFACTOR_SOLUC, manual_mode_calefactor_state);
                ESP_LOGW(mef_temp_soluc_tag, "MANUAL MODE CALEFACTOR: %.0f", manual_mode_calefactor_state);
            }

            break;
        }
    }
}

//==================================| EXTERNAL FUNCTIONS DEFINITION |==================================//

/**
 * @brief   Función para inicializar el módulo de MEFs del algoritmo de temperatura de la solución.
 *
 * @param mqtt_client   Handle del cliente MQTT.
 * @return esp_err_t
 */
esp_err_t mef_temp_soluc_init(esp_mqtt_client_handle_t mqtt_client)
{
    /**
     *  Copiamos el handle del cliente MQTT en la variable interna.
     */
    MefTempSolucClienteMQTT = mqtt_client;

    //=======================| CREACION TAREAS |=======================//

    /**
     *  Se crea la tarea mediante la cual se controlará la transicion de las
     *  MEFs del algoritmo de control de temperatura de solución.
     */
    if (xMefTempSolucAlgoritmoControlTaskHandle == NULL)
    {
        xTaskCreate(
            vTaskSolutionTempControl,
            "vTaskSolutionTempControl",
            4096,
            NULL,
            2,
            &xMefTempSolucAlgoritmoControlTaskHandle);

        /**
         *  En caso de que el handle sea NULL, implica que no se pudo crear la tarea, y se retorna con error.
         */
        if (xMefTempSolucAlgoritmoControlTaskHandle == NULL)
        {
            ESP_LOGE(mef_temp_soluc_tag, "Failed to create vTaskSolutionTempControl task.");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}



/**
 * @brief   Función que devuelve el Task Handle de la tarea principal del algoritmo de control de temperatura de solución.
 *
 * @return TaskHandle_t Task Handle de la tarea.
 */
TaskHandle_t mef_temp_soluc_get_task_handle(void)
{
    return xMefTempSolucAlgoritmoControlTaskHandle;
}



/**
 * @brief   Función que devuelve el valor del delta de temperatura establecido.
 *
 * @return DS18B20_sensor_temp_t Delta de temperatura en °C.
 */
DS18B20_sensor_temp_t mef_temp_soluc_get_delta_temp(void)
{
    return mef_temp_soluc_delta_temp_soluc;
}



/**
 * @brief   Función para establecer nuevos límites del rango de temperatura considerado como correcto para el 
 *          algoritmo de control de temperatura de solución.
 *
 * @param nuevo_limite_inferior_temp_soluc   Límite inferior del rango.
 * @param nuevo_limite_superior_temp_soluc   Límite superior del rango.
 */
void mef_temp_soluc_set_temp_control_limits(DS18B20_sensor_temp_t nuevo_limite_inferior_temp_soluc, DS18B20_sensor_temp_t nuevo_limite_superior_temp_soluc)
{
    mef_temp_soluc_limite_inferior_temp_soluc = nuevo_limite_inferior_temp_soluc;
    mef_temp_soluc_limite_superior_temp_soluc = nuevo_limite_superior_temp_soluc;
}



/**
 * @brief   Función para actualizar el valor de temperatura de la solución sensado.
 *
 * @param nuevo_valor_temp_soluc Nuevo valor de temperatura de la solución en °C.
 */
void mef_temp_soluc_set_temp_soluc_value(DS18B20_sensor_temp_t nuevo_valor_temp_soluc)
{
    mef_temp_soluc_temp = nuevo_valor_temp_soluc;
}



/**
 * @brief   Función para cambiar el estado de la bandera de modo MANUAL, utilizada por
 *          la MEF para cambiar entre estado de modo MANUAL y AUTOMATICO.
 *
 * @param manual_mode_flag_state    Estado de la bandera.
 */
void mef_temp_soluc_set_manual_mode_flag_value(bool manual_mode_flag_state)
{
    mef_temp_soluc_manual_mode_flag = manual_mode_flag_state;
}



/**
 * @brief   Función para cambiar el estado de la bandera de error de sensor de temperatura de solución.
 *
 * @param sensor_error_flag_state    Estado de la bandera.
 */
void mef_temp_soluc_set_sensor_error_flag_value(bool sensor_error_flag_state)
{
    mef_temp_soluc_sensor_error_flag = sensor_error_flag_state;
}