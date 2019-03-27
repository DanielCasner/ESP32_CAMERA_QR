/*
 MPL3115A2 Barometric Pressure Sensor Library
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 22nd, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 Rewritten for C by Daniel Casner, March 2019

 This library allows an Arduino to read from the MPL3115A2 low-cost high-precision pressure sensor.

 If you have feature suggestions or need support please use the github support page: https://github.com/sparkfun/MPL3115A2_Breakout

 Hardware Setup: The MPL3115A2 lives on the I2C bus. Attach the SDA pin to A4, SCL to A5. Use inline 10k resistors
 if you have a 5V board. If you are using the SparkFun breakout board you *do not* need 4.7k pull-up resistors
 on the bus (they are built-in).

 Link to the breakout board product:

 Software:
 .begin() Gets sensor on the I2C bus.
 .readAltitude() Returns float with meters above sealevel. Ex: 1638.94
 .readAltitudeFt() Returns float with feet above sealevel. Ex: 5376.68
 .readPressure() Returns float with barometric pressure in Pa. Ex: 83351.25
 .readTemp() Returns float with current temperature in Celsius. Ex: 23.37
 .readTempF() Returns float with current temperature in Fahrenheit. Ex: 73.96
 .setModeBarometer() Puts the sensor into Pascal measurement mode.
 .setModeAltimeter() Puts the sensor into altimetery mode.
 .setModeStandy() Puts the sensor into Standby mode. Required when changing CTRL1 register.
 .setModeActive() Start taking measurements!
 .setOversampleRate(unsigned char) Sets the # of samples from 1 to 128. See datasheet.
 .enableEventFlags() Sets the fundamental event flags. Required during setup.

 */

#include "MPL3115A2.h"
#include "wiring.h"

#define true 1
#define false 0

// These are the two I2C functions in this sketch.
unsigned char IIC_Read(MPL3115A2* self, unsigned char regAddr)
{
  // This function reads one unsigned char over IIC
  unsigned char ret[1];
  twi_writeTo(&(self->twi), self->address, &regAddr, 1, false);
  twi_readFrom(&(self->twi), self->address, ret, 1, true);
  return ret[0];
}

void IIC_Write(MPL3115A2* self, unsigned char regAddr, unsigned char value)
{
  // This function writes one byto over IIC
  unsigned char payload[2];
  payload[0] = regAddr;
  payload[1] = value;
  twi_writeTo(&(self->twi), self->address, payload, 2, true);
}

//Clears then sets the OST bit which causes the sensor to immediately take another reading
//Needed to sample faster than 1Hz
void toggleOneShot(MPL3115A2* self)
{
  unsigned char tempSetting = IIC_Read(self, CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<1); //Clear OST bit
  IIC_Write(self, CTRL_REG1, tempSetting);

  tempSetting = IIC_Read(self, CTRL_REG1); //Read current settings to be safe
  tempSetting |= (1<<1); //Set OST bit
  IIC_Write(self, CTRL_REG1, tempSetting);
}

//Start I2C communication
void MPL3115A2_begin(MPL3115A2* self, unsigned char address, unsigned char sda, unsigned char scl)
{
  self->address = address;
  twi_init(&(self->twi), sda, scl);
}


//Returns the number of meters above sea level
//Returns -1 if no new data is available
int readAltitude(MPL3115A2* self)
{
  unsigned char tx_buf[4];
  unsigned char rx_buf[4];
  toggleOneShot(self); //Toggle the OST bit causing the sensor to immediately take another reading

	//Wait for PDR bit, indicates we have new pressure data
	int counter = 0;
	while( (IIC_Read(self, MPL_STATUS) & (1<<1)) == 0)
	{
		if(++counter > 600) return(-999); //Error out after max of 512ms for a read
		delay(1);
	}

  tx_buf[0] = OUT_P_MSB;
  twi_writeTo(&(self->twi), self->address, tx_buf, 1, false);
  twi_readFrom(&(self->twi), self->address, rx_buf, 3, true);

  return ((int)rx_buf[0] << 16) | rx_buf[1];
}


int readTemp(MPL3115A2* self)
{
  int scratch = 0;
  int negative = 0;
  int temperature;
  unsigned char tx_buf[4];
  unsigned char rx_buf[4];
	if((IIC_Read(self, MPL_STATUS) & (1<<1)) == 0) toggleOneShot(self); //Toggle the OST bit causing the sensor to immediately take another reading

	//Wait for TDR bit, indicates we have new temp data
	int counter = 0;
	while( (IIC_Read(self, MPL_STATUS) & (1<<1)) == 0)
	{
		if(++counter > 600) return(-999); //Error out after max of 512ms for a read
		delay(1);
	}

  tx_buf[0] = OUT_T_MSB;
  twi_writeTo(&(self->twi), self->address, tx_buf, 1, false);
  twi_readFrom(&(self->twi), self->address, rx_buf, 2, true);

  if (rx_buf[0] > 0x7f) {
    scratch = ~((rx_buf[0] << 8) + rx_buf[1] + 1); //2’s complement
    rx_buf[0] = scratch >> 8;
    rx_buf[1] = scratch & 0x00f0;
    negative = 1;
  }

  temperature = rx_buf[0];
  if (negative) {
    return 0 - temperature;
  }

  return temperature;
}


//Sets the mode to Barometer
//CTRL_REG1, ALT bit
void setModeBarometer(MPL3115A2* self)
{
  unsigned char tempSetting = IIC_Read(self, CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<7); //Clear ALT bit
  IIC_Write(self, CTRL_REG1, tempSetting);
}

//Sets the mode to Altimeter
//CTRL_REG1, ALT bit
void setModeAltimeter(MPL3115A2* self)
{
  unsigned char tempSetting = IIC_Read(self, CTRL_REG1); //Read current settings
  tempSetting |= (1<<7); //Set ALT bit
  IIC_Write(self, CTRL_REG1, tempSetting);
}

//Puts the sensor in standby mode
//This is needed so that we can modify the major control registers
void setModeStandby(MPL3115A2* self)
{
  unsigned char tempSetting = IIC_Read(self, CTRL_REG1); //Read current settings
  tempSetting &= ~(1<<0); //Clear SBYB bit for Standby mode
  IIC_Write(self, CTRL_REG1, tempSetting);
}

//Puts the sensor in active mode
//This is needed so that we can modify the major control registers
void setModeActive(MPL3115A2* self)
{
  unsigned char tempSetting = IIC_Read(self, CTRL_REG1); //Read current settings
  tempSetting |= (1<<0); //Set SBYB bit for Active mode
  IIC_Write(self, CTRL_REG1, tempSetting);
}

//Call with a rate from 0 to 7. See page 33 for table of ratios.
//Sets the over sample rate. Datasheet calls for 128 but you can set it
//from 1 to 128 samples. The higher the oversample rate the greater
//the time between data samples.
void setOversampleRate(MPL3115A2* self, unsigned char sampleRate)
{
  if(sampleRate > 7) sampleRate = 7; //OS cannot be larger than 0b.0111
  sampleRate <<= 3; //Align it for the CTRL_REG1 register

  unsigned char tempSetting = IIC_Read(self, CTRL_REG1); //Read current settings
  tempSetting &= 0b11000111; //Clear out old OS bits
  tempSetting |= sampleRate; //Mask in new OS bits
  IIC_Write(self, CTRL_REG1, tempSetting);
}

//Enables the pressure and temp measurement event flags so that we can
//test against them. This is recommended in datasheet during setup.
void enableEventFlags(MPL3115A2* self)
{
  IIC_Write(self, PT_DATA_CFG, 0x07); // Enable all three pressure and temp event flags
}
