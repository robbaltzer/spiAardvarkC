################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../aadetect.c \
../aagpio.c \
../aai2c_eeprom.c \
../aai2c_file.c \
../aai2c_slave.c \
../aalights.c \
../aamonitor.c \
../aardvark.c \
../aaspi_eeprom.c \
../aaspi_file.c \
../aaspi_slave.c \
../main.c 

OBJS += \
./aadetect.o \
./aagpio.o \
./aai2c_eeprom.o \
./aai2c_file.o \
./aai2c_slave.o \
./aalights.o \
./aamonitor.o \
./aardvark.o \
./aaspi_eeprom.o \
./aaspi_file.o \
./aaspi_slave.o \
./main.o 

C_DEPS += \
./aadetect.d \
./aagpio.d \
./aai2c_eeprom.d \
./aai2c_file.d \
./aai2c_slave.d \
./aalights.d \
./aamonitor.d \
./aardvark.d \
./aaspi_eeprom.d \
./aaspi_file.d \
./aaspi_slave.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


