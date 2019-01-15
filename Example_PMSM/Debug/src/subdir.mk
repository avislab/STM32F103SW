################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/adc_dma.c \
../src/main.c \
../src/pmsm.c \
../src/syscalls.c \
../src/system_stm32f10x.c \
../src/systickdelay.c \
../src/usart_dma.c 

OBJS += \
./src/adc_dma.o \
./src/main.o \
./src/pmsm.o \
./src/syscalls.o \
./src/system_stm32f10x.o \
./src/systickdelay.o \
./src/usart_dma.o 

C_DEPS += \
./src/adc_dma.d \
./src/main.d \
./src/pmsm.d \
./src/syscalls.d \
./src/system_stm32f10x.d \
./src/systickdelay.d \
./src/usart_dma.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DSTM32F10X_MD -DDEBUG -DUSE_STDPERIPH_DRIVER -I"/home/andre/workspace/Example_PMSM/StdPeriph_Driver/inc" -I"/home/andre/workspace/Example_PMSM/inc" -I"/home/andre/workspace/Example_PMSM/CMSIS/device" -I"/home/andre/workspace/Example_PMSM/CMSIS/core" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


