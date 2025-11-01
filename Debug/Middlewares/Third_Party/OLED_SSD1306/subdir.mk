################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/OLED_SSD1306/ssd1306.c \
../Middlewares/Third_Party/OLED_SSD1306/ssd1306_fonts.c 

OBJS += \
./Middlewares/Third_Party/OLED_SSD1306/ssd1306.o \
./Middlewares/Third_Party/OLED_SSD1306/ssd1306_fonts.o 

C_DEPS += \
./Middlewares/Third_Party/OLED_SSD1306/ssd1306.d \
./Middlewares/Third_Party/OLED_SSD1306/ssd1306_fonts.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/OLED_SSD1306/%.o Middlewares/Third_Party/OLED_SSD1306/%.su Middlewares/Third_Party/OLED_SSD1306/%.cyclo: ../Middlewares/Third_Party/OLED_SSD1306/%.c Middlewares/Third_Party/OLED_SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32C031xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/include -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/qk/gnu/ -I../Middlewares/Third_Party/QuantumLeaps_RTEF_qpc/ports/arm-cm/config -I"M:/embedded/stm32C0_Nucleus/qpc-tetris-water-system/Middlewares/Third_Party/OLED_SSD1306/include" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-OLED_SSD1306

clean-Middlewares-2f-Third_Party-2f-OLED_SSD1306:
	-$(RM) ./Middlewares/Third_Party/OLED_SSD1306/ssd1306.cyclo ./Middlewares/Third_Party/OLED_SSD1306/ssd1306.d ./Middlewares/Third_Party/OLED_SSD1306/ssd1306.o ./Middlewares/Third_Party/OLED_SSD1306/ssd1306.su ./Middlewares/Third_Party/OLED_SSD1306/ssd1306_fonts.cyclo ./Middlewares/Third_Party/OLED_SSD1306/ssd1306_fonts.d ./Middlewares/Third_Party/OLED_SSD1306/ssd1306_fonts.o ./Middlewares/Third_Party/OLED_SSD1306/ssd1306_fonts.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-OLED_SSD1306

