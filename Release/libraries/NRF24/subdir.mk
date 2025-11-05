################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/NRF24/NRF24.c 

OBJS += \
./libraries/NRF24/NRF24.o 

C_DEPS += \
./libraries/NRF24/NRF24.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/NRF24/%.o libraries/NRF24/%.su libraries/NRF24/%.cyclo: ../libraries/NRF24/%.c libraries/NRF24/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32C031xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qk/gnu/ -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/config -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/libraries/OLED_SSD1306/include" -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/libraries/NRF24/includes" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-libraries-2f-NRF24

clean-libraries-2f-NRF24:
	-$(RM) ./libraries/NRF24/NRF24.cyclo ./libraries/NRF24/NRF24.d ./libraries/NRF24/NRF24.o ./libraries/NRF24/NRF24.su

.PHONY: clean-libraries-2f-NRF24

