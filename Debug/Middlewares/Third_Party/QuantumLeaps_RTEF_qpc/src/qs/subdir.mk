################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/qstamp.c 

OBJS += \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/qstamp.o 

C_DEPS += \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/qstamp.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/%.o Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/%.su Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/%.cyclo: ../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/%.c Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C031xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qk/gnu/ -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/config -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qs

clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qs:
	-$(RM) ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/qstamp.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/qstamp.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/qstamp.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qs/qstamp.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qs

