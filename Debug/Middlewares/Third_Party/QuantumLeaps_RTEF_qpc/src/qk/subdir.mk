################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/qk.c 

OBJS += \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/qk.o 

C_DEPS += \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/qk.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/%.o Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/%.su Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/%.cyclo: ../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/%.c Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C031xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qk/gnu/ -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/config -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/Middlewares/Third_Party/OLED_SSD1306/include" -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/Middlewares/Third_Party/NRF24/includes" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qk

clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qk:
	-$(RM) ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/qk.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/qk.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/qk.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qk/qk.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qk

