@echo off
"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.3.2.31\AVRToolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "ATM8_I2C_SLAVE_TEST_2.elf" "ATM8_I2C_SLAVE_TEST_2.hex"
