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
#include <util/delay.h>
#include "i2c.h"
#include "a2d.h"

#define DEBUG 1

//	for RPi to 'see' the correct address, it must be left-shifted
//	otherwise i2cdetect -y 1 shows the device at 0x13
static const u08 LOCAL_I2C_ADDR = (0x26 << 1);

// local data buffer
unsigned char localBuffer[2];
unsigned char localBufferLength = 0x02;

u16 A2D[4];

//	operational codes
static const u08 READ_ADC0_H = 	0x30;
static const u08 READ_ADC0_L = 	0x31;
static const u08 READ_ADC1_H = 	0x32;
static const u08 READ_ADC1_L = 	0x33;
static const u08 READ_ADC2_H = 	0x34;
static const u08 READ_ADC2_L = 	0x35;
static const u08 READ_ADC3_H = 	0x36;
static const u08 READ_ADC3_L = 	0x37;

//	function prototypes
void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData);
u08 i2cSlaveTransmitService(u08 transmitDataLengthMax, u08* transmitData);

int main(void) {
#ifdef DEBUG
	DDRB |= (1<<PB0);
#endif
	i2cInit();
	i2cSetLocalDeviceAddr(LOCAL_I2C_ADDR,0);
	i2cSetSlaveReceiveHandler(i2cSlaveReceiveService);
	i2cSetSlaveTransmitHandler(i2cSlaveTransmitService);
	
	a2dInit();
	a2dSetReference(ADC_REFERENCE_AVCC);
	
	//	main loop reads the ADC's in a round-robin fashion
	u08 current_adc = 0;
    while(1) {
        A2D[current_adc] = a2dConvert10bit(current_adc);
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

#ifdef DEBUG
	PORTB ^= (1<<PB0);
#endif
	u08 command = receiveData[0];
	if( command < 0x40 ) {
		command -= 0x30;
		u08 channel = command >> 1;
		if( command & 0b00000001 ) {
			localBuffer[0] = (u08)(A2D[channel] & ~0b11110000 );
			
		}
		else {
			localBuffer[0] = (u08)(A2D[channel]>>8);
		}
	}  
}

//	this function is called when a master on the bus addresses and wishes
//	to read data
u08 i2cSlaveTransmitService(u08 transmitDataLengthMax, u08* transmitData)
{
	transmitData[0] = localBuffer[0];
	return 1;
}