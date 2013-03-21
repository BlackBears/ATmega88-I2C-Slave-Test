### ATmega 88 I2C Slave Test ###

####About the code####

This code was written to demonstrate the I2C slave functionality of ATmega x8 series of MCU, such as the [ATmega 48](http://www.atmel.com/devices/atmega48.aspx), [ATmega 88](http://www.atmel.com/devices/atmega88.aspx), [ATmega 168](http://www.atmel.com/devices/atmega168.aspx), and [ATmega 328](http://www.atmel.com/devices/atmega168.aspx).  I believe that it will run on any member of the series, but I've only tested it with ATmega 88.

####Context####

The project is meant to support the missing ADC functionality on the Raspberry Pi.  The RPi has no on-board ADC.  To achieve ADC capabilities, we could use a discrete ADC chip; but most of them use a custom SPI protocol that requires a lot of bit-banging.  I'd rather just write an I2C-based ADC on AVR than deal with all of that on the RPi.  

####Requirements####

To use this code, you will need:

- An **I2C master** device somewhere on the bus
- An ATmega 88 MCU or a device in its family _(vide supra)_.
- [AVR Studio 6](http://www.atmel.com/microsite/atmel_studio6/)

####Testing the code with Raspberry Pi####

You will need to compile the code in AVR Studio 6 or whatever similar tool you use.  _(I like AVR Studio 6.  There are all sorts of arguments on forums like [AVR Freaks](http://www.avrfreaks.net) about what the best environment is.  It's mostly an opinion based thing.)_  You will need to load it onto the MCU.  How to do that is beyond the scope of these instructions, but some of the tools that I use are:  [AVRISP mkII](http://www.atmel.com/tools/AVRISPMKII.aspx), and [AVR programming adapter](https://www.sparkfun.com/products/8508) from Sparkfun.

On the Raspberry Pi side, you need to set up I2C functionality of the device.  Basically, it's just:

    sudo apt-get install python-smbus
    sudo apt-get install i2c-tools

You will also need to edit `/etc/modules` to include the following lines:

	i2c-dev
	i2c-bcm2708

For the hardware, I use the [Adafruit Pi Cobbler]() to make it easier to prototype with the RPi.  Once you have everything connected you can check if the device is on the bus with:

    sudo i2cdetect -y 1

or, if you have the original Raspberry Pi, it's

	sudo i2cdetect -y 0

This will print a grid to the console; and you should see device `26` in the grid.

Now you are ready to read something from the device. To read ADC0H, for example:

	sudo i2cget -y 1 0x26 0x30 b

This should return the lower byte of the 10 bit ADC0 value.  _(Note that for original RPi devices, it's `sudo i2cget -y 0 0x26 0x30 b`)_