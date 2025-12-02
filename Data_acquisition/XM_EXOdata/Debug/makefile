################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include XM_FW/XM_API/subdir.mk
-include XM_FW/System/Links/PnP_Manager/subdir.mk
-include XM_FW/System/Links/IMU_Module/subdir.mk
-include XM_FW/System/Links/GRF_Module/subdir.mk
-include XM_FW/System/Links/Control_Module/subdir.mk
-include XM_FW/System/Core/subdir.mk
-include XM_FW/System/Comm/USB/subdir.mk
-include XM_FW/System/Comm/UART/subdir.mk
-include XM_FW/System/Comm/CANFD/subdir.mk
-include XM_FW/System/Board/LED/subdir.mk
-include XM_FW/System/Board/GPIO/subdir.mk
-include XM_FW/System/Board/Button/subdir.mk
-include XM_FW/Services/Task_State_Machine/Src/subdir.mk
-include XM_FW/Services/Risk_Mngr/Src/subdir.mk
-include XM_FW/Services/DOP_Mngr/Src/subdir.mk
-include XM_FW/Services/Buff_Mngr/Ring_Buffer/Src/subdir.mk
-include XM_FW/IOIF/Src/subdir.mk
-include XM_FW/Devices/XSENS/subdir.mk
-include XM_FW/Devices/Marveldex/subdir.mk
-include XM_FW/Devices/AGR/Control_Module/subdir.mk
-include XM_Apps/User_Algorithm/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Compatible/USB_HOST/Target/subdir.mk
-include Compatible/USB_HOST/App/subdir.mk
-include Compatible/USB_DEVICE/Target/subdir.mk
-include Compatible/USB_DEVICE/App/subdir.mk
-include Compatible/STM32_USB_Host_Library/Core/Src/subdir.mk
-include Compatible/STM32_USB_Host_Library/Class/MSC/Src/subdir.mk
-include Compatible/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Compatible/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Compatible/FATFS/Target/subdir.mk
-include Compatible/Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Extension_Module
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Extension_Module.elf \

MAP_FILES += \
Extension_Module.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Extension_Module.list \

OBJCOPY_HEX += \
Extension_Module.hex \

OBJCOPY_BIN += \
Extension_Module.bin \


# All Target
all: main-build

# Main-build Target
main-build: Extension_Module.elf secondary-outputs

# Tool invocations
Extension_Module.elf Extension_Module.map: $(OBJS) $(USER_OBJS) C:\Github\IIT\Data_acquisition\XM_EXOdata\STM32H743XIHX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Extension_Module.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Github\IIT\Data_acquisition\XM_EXOdata\STM32H743XIHX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Extension_Module.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Extension_Module.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Extension_Module.list"
	@echo 'Finished building: $@'
	@echo ' '

Extension_Module.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Extension_Module.hex"
	@echo 'Finished building: $@'
	@echo ' '

Extension_Module.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Extension_Module.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Extension_Module.bin Extension_Module.elf Extension_Module.hex Extension_Module.list Extension_Module.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
