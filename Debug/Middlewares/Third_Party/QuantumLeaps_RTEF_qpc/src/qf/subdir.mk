################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.c \
../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.c 

OBJS += \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.o \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.o 

C_DEPS += \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.d \
./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/%.o Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/%.su Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/%.cyclo: ../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/%.c Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C031xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qk/gnu/ -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/config -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qf

clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qf:
	-$(RM) ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_hsm.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qep_msm.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_act.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_actq.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_defer.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_dyn.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_mem.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_ps.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qact.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qeq.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_qmact.su ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.cyclo ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.d ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.o ./Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/src/qf/qf_time.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-QuantumLeaps_RTEF_qpc-2f-src-2f-qf

