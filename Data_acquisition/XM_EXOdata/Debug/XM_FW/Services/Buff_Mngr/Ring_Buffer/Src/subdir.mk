################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/ring_buffer.c 

OBJS += \
./XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/ring_buffer.o 

C_DEPS += \
./XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/ring_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/%.o XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/%.su XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/%.cyclo: ../XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/%.c XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_Apps/User_Algorithm" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/XM_API" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Board/Button" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Board/GPIO" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Board/LED" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Comm/CANFD" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Comm/UART" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Comm/USB" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Config" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Core" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/Control_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/GRF_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/IMU_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/PnP_Manager" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Devices/Marveldex" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Devices/XSENS" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Devices/AGR/Control_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/Buff_Mngr/Ring_Buffer/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/Task_State_Machine/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/DOP_Mngr/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/Risk_Mngr/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/IOIF/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/FATFS/Target" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Device_Library/Core/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Host_Library/Class/MSC/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Host_Library/Core/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_DEVICE/App" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_DEVICE/Target" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_HOST/App" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_HOST/Target" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-XM_FW-2f-Services-2f-Buff_Mngr-2f-Ring_Buffer-2f-Src

clean-XM_FW-2f-Services-2f-Buff_Mngr-2f-Ring_Buffer-2f-Src:
	-$(RM) ./XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/ring_buffer.cyclo ./XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/ring_buffer.d ./XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/ring_buffer.o ./XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/ring_buffer.su

.PHONY: clean-XM_FW-2f-Services-2f-Buff_Mngr-2f-Ring_Buffer-2f-Src

