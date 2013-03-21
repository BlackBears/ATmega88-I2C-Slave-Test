################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../a2d.c \
../ATM8_I2C_SLAVE_TEST_2.c \
../i2c.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
a2d.o \
ATM8_I2C_SLAVE_TEST_2.o \
i2c.o


OBJS_AS_ARGS +=  \
a2d.o \
ATM8_I2C_SLAVE_TEST_2.o \
i2c.o


C_DEPS +=  \
a2d.d \
ATM8_I2C_SLAVE_TEST_2.d \
i2c.d


C_DEPS_AS_ARGS +=  \
a2d.d \
ATM8_I2C_SLAVE_TEST_2.d \
i2c.d


OUTPUT_FILE_PATH +=ATM8_I2C_SLAVE_TEST_2.elf

OUTPUT_FILE_PATH_AS_ARGS +=ATM8_I2C_SLAVE_TEST_2.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler







./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler
	$(QUOTE)C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.3.2.31\AVRToolchain\bin\avr-gcc.exe$(QUOTE)  -funsigned-char -funsigned-bitfields -O1 -fpack-struct -fshort-enums -g2 -Wall -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -mmcu=atmega88a  -o"$@" "$<" 
	@echo Finished building: $<



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU C Linker
	$(QUOTE)C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.3.2.31\AVRToolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="ATM8_I2C_SLAVE_TEST_2.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group  -mmcu=atmega88a  
	@echo Finished building target: $@
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.3.2.31\AVRToolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "ATM8_I2C_SLAVE_TEST_2.elf" "ATM8_I2C_SLAVE_TEST_2.hex"
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.3.2.31\AVRToolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "ATM8_I2C_SLAVE_TEST_2.elf" "ATM8_I2C_SLAVE_TEST_2.eep" || exit 0
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.3.2.31\AVRToolchain\bin\avr-objdump.exe" -h -S "ATM8_I2C_SLAVE_TEST_2.elf" > "ATM8_I2C_SLAVE_TEST_2.lss"
	"C:\Program Files\Atmel\Atmel Studio 6.0\extensions\Atmel\AVRGCC\3.3.2.31\AVRToolchain\bin\avr-size.exe" -C --mcu=atmega88a  "ATM8_I2C_SLAVE_TEST_2.elf"
	




# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "ATM8_I2C_SLAVE_TEST_2.hex" "ATM8_I2C_SLAVE_TEST_2.lss" "ATM8_I2C_SLAVE_TEST_2.eep" "ATM8_I2C_SLAVE_TEST_2.map"
	