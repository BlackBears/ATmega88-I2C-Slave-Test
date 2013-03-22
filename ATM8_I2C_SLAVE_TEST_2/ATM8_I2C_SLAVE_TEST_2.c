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
#include <compat/twi.h>
#include <util/delay.h>
#include "i2c.h"
#include "a2d.h"

#define DEBUG 1

//	for RPi to 'see' the correct address, it must be left-shifted
//	otherwise i2cdetect -y 1 shows the device at 0x13
static u08 LOCAL_I2C_ADDR;
#define DEFAULT_LOCAL_I2C_ADDR 0x4C

//	requested register address
unsigned char regaddr;		
//	requested address data
unsigned char regdata;
//	control register
u08 ctlreg;

enum { CONVERSION_MODE_8BIT, CONVERSION_MODE_10BIT};
typedef uint8_t conversion_mode_t;

enum { DEBUG_OFF, DEBUG_ON};
typedef uint8_t debug_mode_t;

enum {
	rwStatusRead = 0,
	rwStatusWrite,
};
typedef uint8_t rw_status_t;

//	machine states
enum {
	i2cStateRegisterAddressRequired = 0,
	i2cStateRegisterAddressReceived = 1,
	i2cStateRegisterDataReceived = 2
};
typedef uint8_t i2c_state_t;

enum {
	DebugReadADC = 2,
	DebugAttemptWriteADCError = 3;
	DebugStartup;
}

//	conversion mode
static conversion_mode_t ConversionMode;

//	reference
static uint8_t Vref;

//	debugging mode
static debug_mode_t DebugMode;
static uint8_t blinkCount;

// local data buffer
unsigned char localBuffer[3];
unsigned char localBufferLength = 0x03;

u16 A2D[4];

//	Control register bits
#define CTDBG 	0 		//	if set, debugging is enabled
#define CTADM	1 		//	if set, conversion mode is 10 bits, otherwise 8 bits
#define CTVR0	2 		//	bits 2 and 3 determine VREF setting; 0b00 = AREF, 0b01 = AVCC, 0b10 = RSVD, 0b11 = 2.56V
#define	CTVR1	3 		//	bits 4..7 are reserved
#define CTRV0	4
#define CTRV1	5
#define CTRV2	6
#define CTRV3	7

//	EEPROM values
u08 EEMEM _ControlRegister = 0b0000110;	//	by default no debugging, 

//	registers
static const u08 READ_ADC0_H = 		0x30;		//	ADC0H
static const u08 READ_ADC0_L = 		0x31;		//	ADC0L
static const u08 READ_ADC1_H = 		0x32;		//	ADC1H
static const u08 READ_ADC1_L = 		0x33;		//	ADC1L
static const u08 READ_ADC2_H = 		0x34;		//	ADC2H
static const u08 READ_ADC2_L = 		0x35;		//	ADC2L
static const u08 READ_ADC3_H = 		0x36;		//	ADC3H
static const u08 READ_ADC3_L = 		0x37;		//	ADC3L
static const u08 CTLREG = 			0x50;		//	CTLREG = control register

//	function prototypes
void blink(u08 count);
void read_device_addr(void);
void set_params_with_ctlreg(void);


//	this service is called when we rec'v a packet as slave
void i2c_slave_service(rw_status_t rw) {
	//	rw_status Read = rwStatusRead (0), Write = rwStatusWrite (1)
	if( (command >= READ_ADC0_H) && (command <= READ_ADC3_L) ) {
		//	this is an ADC register
		if( rw == rwStatusRead ) {
			//	these registers are read-only
			u08 command = regaddr - READ_ADC0_H;
			u08 channel = command >> 1;
			if( command &0b00000001 ) {
				regdata = (u08)A2D[channel];
			} 
			else {
				regdata = (u08)(A2D[channel]>>8);
			}
		}
		else {
			//	tried to write to read-only ADC register
			asm("nop");
		}
	}
	else {
		//	this is not an ADC register
		//	it this is the control register being addressed
		if( regaddr == CTLREG ) {
			if( rw == rwStatusWrite ) {
				//	we're writing to the control register
				if( regdata != ctlreg ) {
					//	it's being changed
					ctlreg = regdata;
					eeprom_update_byte(&_ControlRegister,ctlreg);
					set_params_with_ctlreg()
				}
				else {
					//	writing, but nothing actually changed
					asm("nop");
				}
			}
			else {
				//	reading the control register
				regdata = ctlreg;
			}
		}
		else {
			//	unrecognized register
			if( rw == rwStatusWrite ) {
				asm("nop");
			}
			else {
				//	reading from an unrecognized register, return 0;
				regdata = 0x00;
			}
		}
	}
}

int main(void) {
	//	get our hardware address and start the I2C interface
	read_device_addr();
	//	read our control register value from EEPROM on startup and set params
	ctlreg = eeprom_read_byte(&_ControlRegister);
	set_params_with_ctlreg();

	blinkCount = 0;
	
	if( DebugMode == DEBUG_ON ) {
		DDRB |= (1<<PB0);
		blink(DebugStartup);
	}

	//	initialize ADC
	a2dInit();
	a2dSetReference(Vref);

	//	enable global interrupt
	sei();

	//	initialize our global register address and data vars
	regaddr = 0;
	regdata = 0;
	
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
        //	take care of any debugging operations that we have
        if( DebugMode == DEBUG_ON ) {
        	blink(blinkCount);
        }
        //	make sure our hardware address hasn't changed
        read_device_addr();
    }
}

void read_device_addr(void) {
	//	read our hardware address from PD5..7
	u08 hw_addr = ((PIND & 0b11100000) >> 5);
	u08 addr = 0x26 | hw_addr;
	//	if our address has changed
	if( addr != LOCAL_I2C_ADDR ) {
		//	change our local address and (re)start the I2C interface
		LOCAL_I2C_ADDR = addr;
		//	set our slave register address to our local address, no respond to general call
		TWAR = LOCAL_I2C_ADDR & ~(1<<TWGCE);
		TWDR = 0x00;
		//	start slave, clear TWINT, enable ACK, enable TWI, enable TWI interrupt
		TWCR = (1<<TWINT) | (1<TWEA) | (1<<TWEN) | (1<<TWIE);
	}
}


//	this function should be called whenever ctlreg changes, so that we can update 
//	the functionality of the device
void set_params_with_ctlreg(void) {
	//	get the conversion mode
	if( ctlreg & (1<<CTADM) ) {
		ConversionMode = CONVERSION_MODE_10BIT;
	}
	else {
		CONVERSION_MODE_8BIT;
	}
	ConversionMode = (ctlreg & (1<<CTADM) );
	//	get the Vref
	u08 new_vref = ( (ctlreg & 0b00001100) >> 2 );
	if( new_vref != Vref ) {
		//	vref was changed
		Vref = new_vref;
		a2dSetReference(Vref);
	}
	//	get the debug mode
	DebugMode = (ctlreg & (1<<CTDBG));
}

void blink(u08 count) {
	for( u08 i = 0; i < count; i++ ) {
		PORTB |= (1<<PB0);
		_delay_ms(100);
		PORTB &= ~(1<<PB0);
		_delay_ms(100);
	}
}

ISR(TWI_vect) {
	static i2c_state_t state;
	u08 twi_status;

	//	disable global interrupt
	cli();

	//	get TWI status register, masking the prescaler bits
	twi_status = TWSR & 0xF8;

	switch( twi_status ) {
		case TW_SR_SLA_ACK:
			//	handle SLA+W, ACK returned
			state = i2cStateRegisterAddressRequired;			//	start i2c state for "register address required"
			TWCR |= (1<<TWINT);									//	clear TWINT flag
			break;
		case TW_SR_DATA_ADCT:
			//	handle 0x80: data received, ACK returned
			if( state == i2cStateRegisterAddressRequired ) {
				regaddr = TWDR;
				state = i2cStateRegisterAddressReceived;
			}
			else {
				regdata = TWDR;
				state = i2cStateRegisterDataReceived;
			}
			TWCR |= (1<<TWINT);									//	clear TWINT flag
			break;
		case TW_SR_STOP:
			//	handle condition 0xA): stop or repeated start condition while we're selected
			//	if we've received reg data, call the service for write mode and reset state
			if( state = i2cStateRegisterDataReceived ) {
				i2c_slave_service(rwStatusWrite);
				state = i2cStateRegisterAddressRequired;
			}
			TWCR |= (1<<TWINT);	
			break;

		case TW_ST_SLA_ACK:		//	0xA8, SLA+R received, ACK returned
		case TW_ST_DATA_ACK:	//	0aB8, data transmitted, ACK received
			if( state == i2cStateRegisterAddressReceived ) {
				//	we're reading, so call the service for read mode and reset state
				i2c_slave_service(rwStatusRead);
				TWDR = regdata;
				state = i2cStateRegisterAddressRequired;
			}				
			TWCR |= (1<<TWINT);
			break;

		case TW_ST_DATA_NACK:	//	0xC0: data xmitted, NACK received
		case TW_ST_LAST_DATA:	//	0xC8: last data byte transmitted, ACK rec'd
		case TW_BUS_ERROR:		//	0x00: illegal start of stop condition
		default:
			TWCR |= (1<<TWINT);
			state = i2cStateRegisterAddressRequired;
	}

	//	re-enable global interrupt
	sei();
}

