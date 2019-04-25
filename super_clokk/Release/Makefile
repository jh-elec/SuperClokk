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
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../Datasheets


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../BMP180.c \
../build_info.c \
../convert.c \
../dcf.c \
../delay.c \
../ht1632c.c \
../rx8564.c \
../Super_Clokk.c


PREPROCESSING_SRCS +=  \
../i2cmaster.S


ASM_SRCS += 


OBJS +=  \
BMP180.o \
build_info.o \
convert.o \
dcf.o \
delay.o \
ht1632c.o \
i2cmaster.o \
rx8564.o \
Super_Clokk.o

OBJS_AS_ARGS +=  \
BMP180.o \
build_info.o \
convert.o \
dcf.o \
delay.o \
ht1632c.o \
i2cmaster.o \
rx8564.o \
Super_Clokk.o

C_DEPS +=  \
BMP180.d \
build_info.d \
convert.d \
dcf.d \
delay.d \
ht1632c.d \
i2cmaster.d \
rx8564.d \
Super_Clokk.d

C_DEPS_AS_ARGS +=  \
BMP180.d \
build_info.d \
convert.d \
dcf.d \
delay.d \
ht1632c.d \
i2cmaster.d \
rx8564.d \
Super_Clokk.d

OUTPUT_FILE_PATH +=Super_Clokk.elf

OUTPUT_FILE_PATH_AS_ARGS +=Super_Clokk.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler

















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.0.106\include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega32 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.0.106\gcc\dev\atmega32" -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler
./i2cmaster.o: .././i2cmaster.S
	@echo Building file: $<
	@echo Invoking: AVR/GNU Assembler : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -Wa,-gdwarf2 -x assembler-with-cpp -c -mmcu=atmega32 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.0.106\gcc\dev\atmega32" -I "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.0.106\include"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	


./%.o: .././%.S
	@echo Building file: $<
	@echo Invoking: AVR/GNU Assembler : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -Wa,-gdwarf2 -x assembler-with-cpp -c -mmcu=atmega32 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.0.106\gcc\dev\atmega32" -I "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.0.106\include"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="Super_Clokk.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega32 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\ATmega_DFP\1.0.106\gcc\dev\atmega32"  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Super_Clokk.elf" "Super_Clokk.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "Super_Clokk.elf" "Super_Clokk.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "Super_Clokk.elf" > "Super_Clokk.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "Super_Clokk.elf" "Super_Clokk.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "Super_Clokk.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Super_Clokk.elf" "Super_Clokk.a" "Super_Clokk.hex" "Super_Clokk.lss" "Super_Clokk.eep" "Super_Clokk.map" "Super_Clokk.srec" "Super_Clokk.usersignatures"
	