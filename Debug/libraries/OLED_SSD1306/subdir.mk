################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/OLED_SSD1306/ssd1306.c \
../libraries/OLED_SSD1306/ssd1306_fonts.c \
../libraries/OLED_SSD1306/ssd1306_tests.c 

OBJS += \
./libraries/OLED_SSD1306/ssd1306.o \
./libraries/OLED_SSD1306/ssd1306_fonts.o \
./libraries/OLED_SSD1306/ssd1306_tests.o 

C_DEPS += \
./libraries/OLED_SSD1306/ssd1306.d \
./libraries/OLED_SSD1306/ssd1306_fonts.d \
./libraries/OLED_SSD1306/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/OLED_SSD1306/%.o libraries/OLED_SSD1306/%.su libraries/OLED_SSD1306/%.cyclo: ../libraries/OLED_SSD1306/%.c libraries/OLED_SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C031xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qk/gnu/ -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/config -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/libraries/OLED_SSD1306/include" -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/libraries/NRF24/includes" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-libraries-2f-OLED_SSD1306

clean-libraries-2f-OLED_SSD1306:
	-$(RM) ./libraries/OLED_SSD1306/ssd1306.cyclo ./libraries/OLED_SSD1306/ssd1306.d ./libraries/OLED_SSD1306/ssd1306.o ./libraries/OLED_SSD1306/ssd1306.su ./libraries/OLED_SSD1306/ssd1306_fonts.cyclo ./libraries/OLED_SSD1306/ssd1306_fonts.d ./libraries/OLED_SSD1306/ssd1306_fonts.o ./libraries/OLED_SSD1306/ssd1306_fonts.su ./libraries/OLED_SSD1306/ssd1306_tests.cyclo ./libraries/OLED_SSD1306/ssd1306_tests.d ./libraries/OLED_SSD1306/ssd1306_tests.o ./libraries/OLED_SSD1306/ssd1306_tests.su

.PHONY: clean-libraries-2f-OLED_SSD1306

