################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/ls016b8uy/ls016b8uy.c 

OBJS += \
./Drivers/BSP/Components/ls016b8uy/ls016b8uy.o 

C_DEPS += \
./Drivers/BSP/Components/ls016b8uy/ls016b8uy.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/ls016b8uy/%.o Drivers/BSP/Components/ls016b8uy/%.su: ../Drivers/BSP/Components/ls016b8uy/%.c Drivers/BSP/Components/ls016b8uy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Core/Inc -IC:/Users/Admin/STM32CubeIDE/workspace_1.10.1/USB/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/Admin/STM32CubeIDE/workspace_1.10.1/USB/STM32Cube_FW_F4_V1.27.1/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/Admin/STM32CubeIDE/workspace_1.10.1/USB/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/Admin/STM32CubeIDE/workspace_1.10.1/USB/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include -I"C:/Users/Admin/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/BSP/STM32F429I-Discovery" -I../USB_DEVICE/App -I../USB_DEVICE/Target -IC:/Users/Admin/STM32CubeIDE/workspace_1.10.1/USB/STM32Cube_FW_F4_V1.27.1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -IC:/Users/Admin/STM32CubeIDE/workspace_1.10.1/USB/STM32Cube_FW_F4_V1.27.1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/Admin/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/BSP/Components" -I"C:/Users/Admin/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/BSP/Components/l3gd20" -I"C:/Users/Admin/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/BSP" -I"C:/Users/Admin/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/BSP/attributes" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-ls016b8uy

clean-Drivers-2f-BSP-2f-Components-2f-ls016b8uy:
	-$(RM) ./Drivers/BSP/Components/ls016b8uy/ls016b8uy.d ./Drivers/BSP/Components/ls016b8uy/ls016b8uy.o ./Drivers/BSP/Components/ls016b8uy/ls016b8uy.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-ls016b8uy

