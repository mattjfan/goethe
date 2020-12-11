#include "Arduino.h"
//#include <avr/pgmspace.h>
#include <Wire.h>

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
* GPIO 
**/
#define PWM_PIN 24
#define INT_PIN 25

/**
 * GPIO read and write
**/
#define DEV_Digital_Write(_pin, _value) digitalWrite(_pin, _value == 0? LOW:HIGH)
#define DEV_Digital_Read(_pin) digitalRead(_pin)

/**
* IIC 
**/
#define IIC_Addr              0x29

/**
 * delay x ms
**/
#define DEV_Delay_ms(__xms)    delay(__xms)

/**
 * PWM_BL
**/
#define  DEV_Set_PWM(_Value)  analogWrite(PWM_PIN, _Value)
#define DEV_PWM_value         255
/*-----------------------------------------------------------------------------*/
void Config_Init();

void DEV_Set_I2CAddress(UBYTE Add);
void DEV_I2C_WriteByte(UBYTE add_, UBYTE data_);
void DEV_I2C_WriteWord(UBYTE add_, UWORD data_);
UBYTE DEV_I2C_ReadByte(UBYTE add_);
UWORD DEV_I2C_ReadWord(UBYTE add_);
