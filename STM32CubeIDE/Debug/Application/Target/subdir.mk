################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/SubGHz_Phy/Target/radio_board_if.c 

OBJS += \
./Application/Target/radio_board_if.o 

C_DEPS += \
./Application/Target/radio_board_if.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Target/radio_board_if.o: C:/Users/Dat\ UTC/Documents/Rang_Dong_tren_cty/1.\ LoRa\ ST50H/STM32Cube_FW_WL_V1.0.0/Projects/NUCLEO-WL55JC/Applications/SubGHz_Phy/SubGHz_I2C/SubGHz_Phy/Target/radio_board_if.c Application/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../../Core/Inc -I../../SubGHz_Phy/App -I../../SubGHz_Phy/Target -I../../../../../../../Drivers/STM32WLxx_HAL_Driver/Inc -I../../../../../../../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy -I../../../../../../../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../../../../../../../Utilities/lpm/tiny_lpm -I../../../../../../../Utilities/misc -I../../../../../../../Utilities/sequencer -I../../../../../../../Utilities/timer -I../../../../../../../Utilities/trace/adv_trace -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/Target/radio_board_if.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

