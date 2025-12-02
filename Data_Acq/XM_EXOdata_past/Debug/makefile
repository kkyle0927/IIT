################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
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


BUILD_ARTIFACT_NAME := XM_EXOdata_past
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
XM_EXOdata_past.elf \

MAP_FILES += \
XM_EXOdata_past.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
XM_EXOdata_past.list \

OBJCOPY_HEX += \
XM_EXOdata_past.hex \

OBJCOPY_BIN += \
XM_EXOdata_past.bin \


# All Target
all: main-build

# Main-build Target
main-build: XM_EXOdata_past.elf secondary-outputs

# Tool invocations
XM_EXOdata_past.elf XM_EXOdata_past.map: $(OBJS) $(USER_OBJS) C:\Github\IIT\Data_Acq\XM_EXOdata_past\STM32H743XIHX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "XM_EXOdata_past.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Github\IIT\Data_Acq\XM_EXOdata_past\STM32H743XIHX_FLASH.ld" --specs=nosys.specs -Wl,-Map="XM_EXOdata_past.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

XM_EXOdata_past.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "XM_EXOdata_past.list"
	@echo 'Finished building: $@'
	@echo ' '

XM_EXOdata_past.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "XM_EXOdata_past.hex"
	@echo 'Finished building: $@'
	@echo ' '

XM_EXOdata_past.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "XM_EXOdata_past.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) XM_EXOdata_past.bin XM_EXOdata_past.elf XM_EXOdata_past.hex XM_EXOdata_past.list XM_EXOdata_past.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
