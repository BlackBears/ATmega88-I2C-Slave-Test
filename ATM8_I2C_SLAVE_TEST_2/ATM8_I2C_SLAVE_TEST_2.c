/******************************************************************+
|																	|
|	FILE:			ATM_I2C_SLAVE_TEST_2.c 							|
|	MCU:			ATmega 88										|
|	DATE:			2013-03-21 06-24-37								|
|	AUTHOR:			Alan Duncan										|
|	ENVIRONMENT:	AVR Studio 6									|
|	DESCRIPTION:	This program allows the ATmega 88 or similar	|
|		MCU to act as a slave on the I2C bus, reporting ADC values  |
|		according to commands sent on the bus.						|
|																	|
|	CONTEXT:		This project is meant to provide the Raspberry  |
|		Pi with ADC capabilities since it has none natively.  		|
|		Further, for some reason there are few discrete ADC chips   |
|		that use the I2C bus.  Although the RPi can do SPI, 		|
|		the MCP3xxx series of ADC chips requires a lot of bit		|
|		banging.  Therefore, I put together a project that uses		|
|		an AVR as an ADC on the I2C bus.							|
+*******************************************************************/

#include "global.h"
#include <avr/io.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "i2c.h"
#include "a2d.h"

#define DEBUG 1

//	for RPi to 'see' the correct address, it must be left-shifted
//	otherwise i2cdetect -y 1 shows the device at 0x13
static u08 LOCAL_I2C_ADDR;
#define DEFAULT_LOCAL_I2C_ADDR 0x4C

//	conversion mode
static u08 ConversionMode;

//	reference
static u08 Vref;

//	debugging mode
static u08 DebugMode;

// local data buffer
unsigned char localBuffer[3];
unsigned char localBufferLength = 0x03;

enum { CONVERSION_MODE_8BIT, CONVERSION_MODE_10BIT};
enum { DEBUG_OFF, DEBUG_ON};

u16 A2D[4];

//	EEPROM values
u08 EEMEM _DeviceAddress = DEFAULT_LOCAL_I2C_ADDR;
u08 EEMEM _ConversionMode = CONVERSION_MODE_10BIT;
u08 EEMEM _VRef = ADC_REFERENCE_AVCC;
u08 EEMEM _DebugMode = DEBUG_OFF;

//	operational codes
static const u08 READ_ADC0_H = 		0x30;
static const u08 READ_ADC0_L = 		0x31;
static const u08 READ_ADC1_H = 		0x32;
static const u08 READ_ADC1_L = 		0x33;
static const u08 READ_ADC2_H = 		0x34;
static const u08 READ_ADC2_L = 		0x35;
static const u08 READ_ADC3_H = 		0x36;
static const u08 READ_ADC3_L = 		0x37;
static const u08 SET_ADC_MODE =		0x41;
static const u08 SET_I2C_ADDR =		0x42;
static const u08 SET_DEBUG_MODE =	0x43;
static const u08 SET_VREF =			0x44;

//	command acknowledgment
static const u08 ACK_ADC_MODE =		0x51;
static const u08 ACK_I2C_MODE =		0x52;
static const u08 ACK_DEBUG_MODE =	0x53;
static const u08 ACK_VREF =			0x54;

//	command failures
static const u08 FAIL_ADC_MODE =	0x61;
static const u08 FAIL_I2C_MODE =	0x62;
static const u08 FAIL_DEBUG_MODE =	0x63;
static const u08 FAIL_VREF =		0x64;

//	function prototypes
void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData);
u08 i2cSlaveTransmitService(u08 transmitDataLengthMax, u08* transmitData);

int main(void) {
#ifdef DEBUG
	DDRB |= (1<<PB0);
#endif
	//	read our startup values from EEPROM
	LOCAL_I2C_ADDR = eeprom_read_byte(&_DeviceAddress);
	ConversionMode = eeprom_read_byte(&_ConversionMode);
	Vref = eeprom_read_byte(&_VRef);
	DebugMode = eeprom_read_byte(&_DebugMode);
	
	if( DebugMode == DEBUG_ON ) {
		DDRB |= (1<<PB0);
	}
	//	initialize our I2C bus
	i2cInit();
	i2cSetLocalDeviceAddr(LOCAL_I2C_ADDR,0);
	i2cSetSlaveReceiveHandler(i2cSlaveReceiveService);
	i2cSetSlaveTransmitHandler(i2cSlaveTransmitService);
	//	initialize ADC
	a2dInit();
	a2dSetReference(Vref);
	
	//	main loop reads the ADC's in a round-robin fashion
	u08 current_adc = 0;
    while(1) {
		if( ConversionMode == CONVERSION_MODE_10BIT ) {
			A2D[current_adc] = a2dConvert10bit(current_adc);
		}			
        else {
			A2D[current_adc] = (u16)0x0000 | a2dConvert8bit(current_adc);
		}
        current_adc++;
        if( current_adc > 3 ) {
        	current_adc = 0;
        }
    }
}

#pragma mark Slave callback services

//	this function is called when a master on the bus addresses us 
//	and wishes to write data to us.  In this application, the data
//	to be written is the ADC "register"
/*	As an example, if the caller writes 0x31 (read ADC0_L ), then
*	0x31 - 0x30 = 0x01 (0b0001) so the channel is 0b0001 >> 1 which is 0b0000
*	Now we test for whether bit 0 was set in the adjusted command:
*	The adjusted command is 0b00000001; and 0b00000001 & 0b00000001
*	is TRUE, so we select the ADC0_L
*/
void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData) 
{ 
	if( DebugMode == DEBUG_ON ) {
		PORTB ^= (1<<PB0);
	}
	
	u08 command = receiveData[0];
	if( command < 0x40 ) {
		command -= 0x30;
		u08 channel = command >> 1;
		if( command & 0b00000001 ) {
			localBuffer[0] = (u08)A2D[channel];
		}
		else {
			localBuffer[0] = (u08)(A2D[channel]>>8);
		}
	}  
	else {
		localBuffer[0] = receiveDataLength;
	}		
		//switch( command ) {
			/*
			case (const u08)SET_ADC_MODE: {
				if( receiveDataLength > 1 ) {
					u08 raw_mode = localBuffer[1];
					if( raw_mode == 0 ) {
						ConversionMode = CONVERSION_MODE_8BIT;
					}
					else {
						ConversionMode = CONVERSION_MODE_10BIT;
					}
					eeprom_update_byte(&_ConversionMode,ConversionMode);
					localBuffer[0] = ACK_ADC_MODE;
				}
				else {
					localBuffer[0] = FAIL_ADC_MODE;
				}
				break;
			}
			case (const u08)SET_DEBUG_MODE: {
				if( receiveDataLength > 1 ) {
					u08 raw_mode = localBuffer[1];
					if( raw_mode == 0 ) {
						DebugMode = DEBUG_OFF;
					}
					else {
						DebugMode = DEBUG_ON;
					}
					eeprom_update_byte(&_DebugMode,DebugMode);
					localBuffer[0] = ACK_DEBUG_MODE;
				}
				else {
					localBuffer[0] = FAIL_DEBUG_MODE;
				}
				break;	
			}		
			case (const u08)SET_I2C_ADDR: {
				if( receiveDataLength > 1 ) {
					LOCAL_I2C_ADDR = localBuffer[1];
					eeprom_update_byte(&_DeviceAddress,LOCAL_I2C_ADDR);
					localBuffer[0] = ACK_I2C_MODE;
				}
				else {
					localBuffer[0] = FAIL_I2C_MODE;
				}
				break;
			}	
			case (const u08)SET_VREF: {
				if( receiveDataLength > 1 ) {
					Vref = localBuffer[1];
					eeprom_update_byte(&_VRef,Vref);
					localBuffer[0] = ACK_VREF;
				}
				else {
					localBuffer[1] = FAIL_VREF;
				}
				break;
			}
		}
		*/
	//}
}

//	this function is called when a master on the bus addresses and wishes
//	to read data
u08 i2cSlaveTransmitService(u08 transmitDataLengthMax, u08* transmitData)
{
	PORTB |= (1<<PB0);
	transmitData[0] = localBuffer[0];
	return 1;
}