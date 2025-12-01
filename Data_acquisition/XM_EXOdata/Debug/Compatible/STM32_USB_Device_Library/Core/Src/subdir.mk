################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Compatible/STM32_USB_Device_Library/Core/Src/usbd_core.c \
../Compatible/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
../Compatible/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Compatible/STM32_USB_Device_Library/Core/Src/usbd_core.o \
./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o \
./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o 

C_DEPS += \
./Compatible/STM32_USB_Device_Library/Core/Src/usbd_core.d \
./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d \
./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Compatible/STM32_USB_Device_Library/Core/Src/%.o Compatible/STM32_USB_Device_Library/Core/Src/%.su Compatible/STM32_USB_Device_Library/Core/Src/%.cyclo: ../Compatible/STM32_USB_Device_Library/Core/Src/%.c Compatible/STM32_USB_Device_Library/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_Apps/User_Algorithm" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/XM_API" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Board/Button" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Board/GPIO" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Board/LED" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Comm/CANFD" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Comm/UART" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Comm/USB" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Config" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Core" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Links/Control_Module" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Links/GRF_Module" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Links/IMU_Module" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/System/Links/PnP_Manager" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/Devices/Marveldex" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/Devices/XSENS" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/Devices/AGR/Control_Module" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/Services/Buff_Mngr/Ring_Buffer/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/Services/Task_State_Machine/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/Services/DOP_Mngr/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/Services/Risk_Mngr/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/XM_FW/IOIF/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/FATFS/Target" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/STM32_USB_Device_Library/Core/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/STM32_USB_Host_Library/Class/MSC/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/STM32_USB_Host_Library/Core/Inc" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/USB_DEVICE/App" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/USB_DEVICE/Target" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/USB_HOST/App" -I"C:/Users/ChanyoungKo/STM32CubeIDE/workspace_1.14.1/XM_EXOdata/Compatible/USB_HOST/Target" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Compatible-2f-STM32_USB_Device_Library-2f-Core-2f-Src

clean-Compatible-2f-STM32_USB_Device_Library-2f-Core-2f-Src:
	-$(RM) ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_core.cyclo ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_core.d ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_core.o ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_core.su ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.cyclo ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.d ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.o ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.su ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ioreq.cyclo ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ioreq.d ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ioreq.o ./Compatible/STM32_USB_Device_Library/Core/Src/usbd_ioreq.su

.PHONY: clean-Compatible-2f-STM32_USB_Device_Library-2f-Core-2f-Src

