/*
 * ATM8_I2C_SLAVE_TEST_2.c
 *
 * Created: 3/20/2013 10:06:34 PM
 *  Author: Alan Duncan
 */ 

#include "global.h"
#include <avr/io.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2c.h"
#include "a2d.h"

static const u08 LOCAL_I2C_ADDR = (0x26 << 1);

// local data buffer
unsigned char localBuffer[2];
unsigned char localBufferLength = 0x02;

u16 ADC_0;
u16 ADC_1;

void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData);
u08 i2cSlaveTransmitService(u08 transmitDataLengthMax, u08* transmitData);

int main(void)
{
	DDRB |= (1<<PB0);
	i2cInit();
	i2cSetLocalDeviceAddr(LOCAL_I2C_ADDR,0);
	i2cSetSlaveReceiveHandler(i2cSlaveReceiveService);
	i2cSetSlaveTransmitHandler(i2cSlaveTransmitService);
	
	a2dInit();
	a2dSetReference(ADC_REFERENCE_AVCC);
	
    while(1)
    {
        _delay_ms(10);
		ADC_0 = a2dConvert10bit(0);
		ADC_1 = a2dConvert10bit(1);
    }
}

// slave operations 
void i2cSlaveReceiveService(u08 receiveDataLength, u08* receiveData) 
{ 
	PORTB |= (1<<PB0);
	if( receiveData[0] == 0x30 ) {
		localBuffer[0] = (u08)ADC_0;
	}
	else if( receiveData[0] == 0x31 ) {
		localBuffer[0] = (u08)ADC_1;
	}
	// this function will run when a master somewhere else on the bus 
   // addresses us and wishes to write data to us    
}

u08 i2cSlaveTransmitService(u08 transmitDataLengthMax, u08* transmitData)
{
	u08 i;
	
	// this function will run when a master somewhere else on the bus
	// addresses us and wishes to read data from us

	//showByte(*transmitData);
	//cbi(PORTB, PB7);

	// copy the local buffer to the transmit buffer
	///for(i=0; i<localBufferLength; i++)
	//{
	//	*transmitData++ = localBuffer[i];
	//}

	//localBuffer[0]++;
	transmitData[0] = localBuffer[0];
	return 1;
	//return localBufferLength;
}