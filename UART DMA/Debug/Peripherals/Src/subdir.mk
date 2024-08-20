################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripherals/Src/nvic.c \
../Peripherals/Src/uart.c 

OBJS += \
./Peripherals/Src/nvic.o \
./Peripherals/Src/uart.o 

C_DEPS += \
./Peripherals/Src/nvic.d \
./Peripherals/Src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
Peripherals/Src/%.o Peripherals/Src/%.su Peripherals/Src/%.cyclo: ../Peripherals/Src/%.c Peripherals/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103xB -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/OneDrive/Disk D/SOFTWARE/STM32/Video/9. UART/Code/UART DMA/Peripherals" -I"D:/OneDrive/Disk D/SOFTWARE/STM32/Video/9. UART/Code/UART DMA/Peripherals/Inc" -I"D:/OneDrive/Disk D/SOFTWARE/STM32/Video/9. UART/Code/UART DMA/Peripherals/Src" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Peripherals-2f-Src

clean-Peripherals-2f-Src:
	-$(RM) ./Peripherals/Src/nvic.cyclo ./Peripherals/Src/nvic.d ./Peripherals/Src/nvic.o ./Peripherals/Src/nvic.su ./Peripherals/Src/uart.cyclo ./Peripherals/Src/uart.d ./Peripherals/Src/uart.o ./Peripherals/Src/uart.su

.PHONY: clean-Peripherals-2f-Src

