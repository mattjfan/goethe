#include "DEV_Config.h"
UBYTE IIC_Addr_t = IIC_Addr;
void GPIO_Init()
{
  pinMode(PWM_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  
  
}
void Config_Init()
{
  
  GPIO_Init();
  //Serial
  Serial.begin(115200);
  //i2c config
  Wire.begin(); 
  
}

void DEV_I2C_WriteByte(UBYTE add_, UBYTE data_)
{
  Wire.beginTransmission(IIC_Addr_t);

  Wire.write(add_);
  Wire.write(data_ & 0xFF);
  Wire.endTransmission();
}

void DEV_I2C_WriteWord(UBYTE add_, UWORD data_)
{
  Wire.beginTransmission(IIC_Addr_t);
  Wire.write(add_);
  Wire.write(data_ & 0xFF);
  Wire.write((data_ >> 8) & 0xFF);
  Wire.endTransmission();
}

UBYTE DEV_I2C_ReadByte(UBYTE add_)
{
  Wire.beginTransmission(IIC_Addr_t);
  Wire.write(add_);
  Wire.endTransmission();
  Wire.requestFrom(IIC_Addr_t, 1);
  if (Wire.available()) {
   return Wire.read();
  }
  return 0;;
}

UWORD DEV_I2C_ReadWord(UBYTE add_)
{
 uint16_t x; uint16_t t;

  Wire.beginTransmission(IIC_Addr_t);
  Wire.write(add_); 
  Wire.endTransmission();
  Wire.requestFrom(IIC_Addr_t, 2);
  t = Wire.read();
  x = Wire.read();
  x <<= 8;
  x |= t;
  return x;
}

void DEV_Set_I2CAddress(UBYTE add_)
{
  IIC_Addr_t = add_;
 }
