################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/NRF_chip.c \
../Core/Src/accelerometer.c \
../Core/Src/bsp.c \
../Core/Src/display.c \
../Core/Src/main.c \
../Core/Src/main_app.c \
../Core/Src/plant_sensor.c \
../Core/Src/rfbutton.c \
../Core/Src/sensor.c \
../Core/Src/stm32c0xx_hal_msp.c \
../Core/Src/stm32c0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32c0xx.c \
../Core/Src/temp_sensor.c \
../Core/Src/tetris_board.c \
../Core/Src/tetris_input_handler.c \
../Core/Src/tetromino.c 

OBJS += \
./Core/Src/NRF_chip.o \
./Core/Src/accelerometer.o \
./Core/Src/bsp.o \
./Core/Src/display.o \
./Core/Src/main.o \
./Core/Src/main_app.o \
./Core/Src/plant_sensor.o \
./Core/Src/rfbutton.o \
./Core/Src/sensor.o \
./Core/Src/stm32c0xx_hal_msp.o \
./Core/Src/stm32c0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32c0xx.o \
./Core/Src/temp_sensor.o \
./Core/Src/tetris_board.o \
./Core/Src/tetris_input_handler.o \
./Core/Src/tetromino.o 

C_DEPS += \
./Core/Src/NRF_chip.d \
./Core/Src/accelerometer.d \
./Core/Src/bsp.d \
./Core/Src/display.d \
./Core/Src/main.d \
./Core/Src/main_app.d \
./Core/Src/plant_sensor.d \
./Core/Src/rfbutton.d \
./Core/Src/sensor.d \
./Core/Src/stm32c0xx_hal_msp.d \
./Core/Src/stm32c0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32c0xx.d \
./Core/Src/temp_sensor.d \
./Core/Src/tetris_board.d \
./Core/Src/tetris_input_handler.d \
./Core/Src/tetromino.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -DUSE_HAL_DRIVER -DSTM32C031xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qk/gnu/ -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/config -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/libraries/OLED_SSD1306/include" -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/libraries/NRF24/includes" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/NRF_chip.cyclo ./Core/Src/NRF_chip.d ./Core/Src/NRF_chip.o ./Core/Src/NRF_chip.su ./Core/Src/accelerometer.cyclo ./Core/Src/accelerometer.d ./Core/Src/accelerometer.o ./Core/Src/accelerometer.su ./Core/Src/bsp.cyclo ./Core/Src/bsp.d ./Core/Src/bsp.o ./Core/Src/bsp.su ./Core/Src/display.cyclo ./Core/Src/display.d ./Core/Src/display.o ./Core/Src/display.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/main_app.cyclo ./Core/Src/main_app.d ./Core/Src/main_app.o ./Core/Src/main_app.su ./Core/Src/plant_sensor.cyclo ./Core/Src/plant_sensor.d ./Core/Src/plant_sensor.o ./Core/Src/plant_sensor.su ./Core/Src/rfbutton.cyclo ./Core/Src/rfbutton.d ./Core/Src/rfbutton.o ./Core/Src/rfbutton.su ./Core/Src/sensor.cyclo ./Core/Src/sensor.d ./Core/Src/sensor.o ./Core/Src/sensor.su ./Core/Src/stm32c0xx_hal_msp.cyclo ./Core/Src/stm32c0xx_hal_msp.d ./Core/Src/stm32c0xx_hal_msp.o ./Core/Src/stm32c0xx_hal_msp.su ./Core/Src/stm32c0xx_it.cyclo ./Core/Src/stm32c0xx_it.d ./Core/Src/stm32c0xx_it.o ./Core/Src/stm32c0xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32c0xx.cyclo ./Core/Src/system_stm32c0xx.d ./Core/Src/system_stm32c0xx.o ./Core/Src/system_stm32c0xx.su ./Core/Src/temp_sensor.cyclo ./Core/Src/temp_sensor.d ./Core/Src/temp_sensor.o ./Core/Src/temp_sensor.su ./Core/Src/tetris_board.cyclo ./Core/Src/tetris_board.d ./Core/Src/tetris_board.o ./Core/Src/tetris_board.su ./Core/Src/tetris_input_handler.cyclo ./Core/Src/tetris_input_handler.d ./Core/Src/tetris_input_handler.o ./Core/Src/tetris_input_handler.su ./Core/Src/tetromino.cyclo ./Core/Src/tetromino.d ./Core/Src/tetromino.o ./Core/Src/tetromino.su

.PHONY: clean-Core-2f-Src

