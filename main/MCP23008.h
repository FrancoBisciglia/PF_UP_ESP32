#ifndef MCP23008_H_   /* Include guard */
#define MCP23008_H_

/*==================[INCLUDES]=============================================*/

#include "esp_err.h"
#include <stdio.h>
#include <stdbool.h>

/*==================[DEFINES AND MACROS]=====================================*/

#define I2C_MASTER_SCL_IO                   22                    //Pin de SCL del master ESP32
#define I2C_MASTER_SDA_IO                   21                    //Pin de SDA del master ESP32
#define I2C_MASTER_NUM                      0                     //Número de puerto I2C del master ESP32
#define I2C_MASTER_FREQ_HZ                  400000                //Frecuencia del clock SCL del master ESP32
#define I2C_MASTER_TX_BUF_DISABLE           0                     //Al utilizar al ESP32 como master, no se necesita buffer auxiliar
#define I2C_MASTER_RX_BUF_DISABLE           0                     //Al utilizar al ESP32 como master, no se necesita buffer auxiliar
#define I2C_MASTER_TIMEOUT_MS               1000                  //Tiempo que está dispuesto a esperar el master ESP32 para realizar una escritura/lectura

#define MCP23008_ADDR                       0x20                  //Dirección I2C de esclavo del MCP23008
#define MCP23008_GPIO_PORT_REG_ADDR         0x09                  //Dirección del registro de GPIO's del MCP23008
#define MCP23008_IO_CONFIG_REG_ADDR         0x00                  //Dirección del registro de GPIO's del MCP23008

/* Se definen algunos macros de operaciones de bits que serán de utilidad para la función "set_relay_state" */

// BIT(b) regresa el bit 'b' puesto a uno y los demas bits en cero. Ej: BIT(3) regresa 00001000
// BIT_SET(x,b) establece en '1' el bit 'b' de 'x'
#define BIT_SET(x,b)   ((x) |= BIT(b))
// BIT_CLEAR(x,b) establece a '0' el bit 'b' de 'x'
#define BIT_CLEAR(x,b) ((x) &= ~BIT(b))
// BIT_WRITE(x,b,v) establece el valor 'v' en el bit 'b' de 'x'
#define BIT_WRITE(x,b,v) ((v)? BIT_SET(x,b) : BIT_CLEAR(x,b))

/*==================[EXTERNAL DATA DECLARATION]==============================*/

/* Enumeración correspondiente a los 7 reles que posee la placa */
enum relays
{
    RELE_1 = 0,
    RELE_2,
    RELE_3,
    RELE_4,
    RELE_5,
    RELE_6,
    RELE_7
};

/* Enumeración correspondiente a los estados de los reles */
enum state_relays
{
    OFF = 0,
    ON
}; 

/*==================[EXTERNAL FUNCTIONS DECLARATION]=========================*/

esp_err_t MCP23008_init(void);
bool read_pH_trigger(void);
esp_err_t set_relay_state(int8_t relay_num, bool relay_state);
bool get_relay_state(int8_t relay_num);

/*==================[END OF FILE]============================================*/
#endif // MCP23008_H_