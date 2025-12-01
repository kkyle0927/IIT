################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_hal_timebase_tim.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_hal_timebase_tim.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_hal_timebase_tim.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu18 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -DUSE_PWR_LDO_SUPPLY -c -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_Apps/User_Algorithm" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/XM_API" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Board/Button" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Board/GPIO" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Board/LED" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Comm/CANFD" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Comm/UART" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Comm/USB" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Config" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Core" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/Control_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/GRF_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/IMU_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/System/Links/PnP_Manager" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Devices/Marveldex" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Devices/XSENS" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Devices/AGR/Control_Module" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/Buff_Mngr/Ring_Buffer/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/Task_State_Machine/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/DOP_Mngr/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/Services/Risk_Mngr/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/XM_FW/IOIF/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/Drivers/STM32H7xx_HAL_Driver/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/FATFS/Target" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Device_Library/Class/CDC/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Device_Library/Core/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Host_Library/Class/MSC/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/STM32_USB_Host_Library/Core/Inc" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_DEVICE/App" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_DEVICE/Target" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_HOST/App" -I"C:/Github/IIT/Data_acquisition/XM_EXOdata/Compatible/USB_HOST/Target" -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_hal_timebase_tim.cyclo ./Core/Src/stm32h7xx_hal_timebase_tim.d ./Core/Src/stm32h7xx_hal_timebase_tim.o ./Core/Src/stm32h7xx_hal_timebase_tim.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

