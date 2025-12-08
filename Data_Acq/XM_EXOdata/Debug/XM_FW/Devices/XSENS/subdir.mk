################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
..//XM_FW/Devices/XSENS/mti-630.c 

OBJS += \
./XM_FW/Devices/XSENS/mti-630.o 

C_DEPS += \
./XM_FW/Devices/XSENS/mti-630.d 


# Each subdirectory must supply rules for building sources it contributes
XM_FW/Devices/XSENS/%.o XM_FW/Devices/XSENS/%.su XM_FW/Devices/XSENS/%.cyclo: ../XM_FW/Devices/XSENS/%.c XM_FW/Devices/XSENS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"X:/XM_Apps/User_Algorithm" -I"X:/XM_FW/XM_API" -I"X:/XM_FW/System/Board/Button" -I"X:/XM_FW/System/Board/GPIO" -I"X:/XM_FW/System/Board/LED" -I"X:/XM_FW/System/Comm/CANFD" -I"X:/XM_FW/System/Comm/UART" -I"X:/XM_FW/System/Comm/USB" -I"X:/XM_FW/System/Config" -I"X:/XM_FW/System/Core" -I"X:/XM_FW/System/Links/Control_Module" -I"X:/XM_FW/System/Links/GRF_Module" -I"X:/XM_FW/System/Links/IMU_Module" -I"X:/XM_FW/System/Links/PnP_Manager" -I"X:/XM_FW/Devices/Marveldex" -I"X:/XM_FW/Devices/XSENS" -I"X:/XM_FW/Devices/AGR/Control_Module" -I"X:/XM_FW/Services/Buff_Mngr/Ring_Buffer/Inc" -I"X:/XM_FW/Services/Task_State_Machine/Inc" -I"X:/XM_FW/Services/DOP_Mngr/Inc" -I"X:/XM_FW/Services/Risk_Mngr/Inc" -I"X:/XM_FW/IOIF/Inc" -I"X:/Compatible/Drivers/STM32H7xx_HAL_Driver/Inc" -I"X:/Compatible/FATFS/Target" -I"X:/Compatible/STM32_USB_Device_Library/Class/CDC/Inc" -I"X:/Compatible/STM32_USB_Device_Library/Core/Inc" -I"X:/Compatible/STM32_USB_Host_Library/Class/MSC/Inc" -I"X:/Compatible/STM32_USB_Host_Library/Core/Inc" -I"X:/Compatible/USB_DEVICE/App" -I"X:/Compatible/USB_DEVICE/Target" -I"X:/Compatible/USB_HOST/App" -I"X:/Compatible/USB_HOST/Target" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-XM_FW-2f-Devices-2f-XSENS

clean-XM_FW-2f-Devices-2f-XSENS:
	-$(RM) ./XM_FW/Devices/XSENS/mti-630.cyclo ./XM_FW/Devices/XSENS/mti-630.d ./XM_FW/Devices/XSENS/mti-630.o ./XM_FW/Devices/XSENS/mti-630.su

.PHONY: clean-XM_FW-2f-Devices-2f-XSENS

