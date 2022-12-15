################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/ov5640/ov5640.c 

OBJS += \
./Drivers/BSP/Components/ov5640/ov5640.o 

C_DEPS += \
./Drivers/BSP/Components/ov5640/ov5640.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/ov5640/%.o Drivers/BSP/Components/ov5640/%.su: ../Drivers/BSP/Components/ov5640/%.c Drivers/BSP/Components/ov5640/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -IC:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -IC:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/MSI-NB/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/BSP/Components/l3gd20" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-ov5640

clean-Drivers-2f-BSP-2f-Components-2f-ov5640:
	-$(RM) ./Drivers/BSP/Components/ov5640/ov5640.d ./Drivers/BSP/Components/ov5640/ov5640.o ./Drivers/BSP/Components/ov5640/ov5640.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-ov5640
