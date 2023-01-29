/*

    Funcionalidades del algoritmo de control de bombeo de la solución nutritiva.

*/

#ifndef ALARMAS_USUARIO_H_
#define ALARMAS_USUARIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================================[INCLUDES]=============================================*/

/*============================[DEFINES AND MACROS]=====================================*/

/* Tópico donde se públican los códigos de las diferentes alarmas a visualizar por el usuario. */
#define ALARMS_MQTT_TOPIC   "Alarmas"

/**
 *  Definición de los códigos de alarmas a enviar al usuario.
 */
typedef enum{
    ALARMA_ERROR_SENSOR_DS18B20 = 1,
    ALARMA_ERROR_SENSOR_PH,
    ALARMA_ERROR_SENSOR_TDS,
    ALARMA_ERROR_SENSOR_CO2,
    ALARMA_ERROR_SENSOR_DTH11_TEMP,
    ALARMA_ERROR_SENSOR_DTH11_HUM,
    ALARMA_ERROR_SENSOR_NIVEL_TANQUE_PRINC,
    ALARMA_ERROR_SENSOR_NIVEL_TANQUE_ACIDO,
    ALARMA_ERROR_SENSOR_NIVEL_TANQUE_ALCALINO,
    ALARMA_ERROR_SENSOR_NIVEL_TANQUE_AGUA,
    ALARMA_ERROR_SENSOR_NIVEL_TANQUE_NUTRIENTES,
    ALARMA_FALLA_ILUMINACION,
    ALARMA_FALLA_BOMBA,
    ALARMA_NIVEL_TANQUE_PRINCIPAL_BAJO,
    ALARMA_NIVEL_TANQUE_ACIDO_BAJO,
    ALARMA_NIVEL_TANQUE_ALCALINO_BAJO,
    ALARMA_NIVEL_TANQUE_AGUA_BAJO,
    ALARMA_NIVEL_TANQUE_SUSTRATO_BAJO,
} alarms_t;

/*======================[EXTERNAL DATA DECLARATION]==============================*/

/*=====================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

/*==================[END OF FILE]============================================*/
#ifdef __cplusplus
}
#endif

#endif // ALARMAS_USUARIO_H_