################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h743xihx.s 

OBJS += \
./Core/Startup/startup_stm32h743xihx.o 

S_DEPS += \
./Core/Startup/startup_stm32h743xihx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/FATFS/Target" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Device_Library/Core/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Host_Library/Class/MSC/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Host_Library/Core/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_DEVICE/App" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_DEVICE/Target" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_HOST/App" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_HOST/Target" -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32h743xihx.d ./Core/Startup/startup_stm32h743xihx.o

.PHONY: clean-Core-2f-Startup

