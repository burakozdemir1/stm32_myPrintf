################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := BOOTLOADER_MY_PRINTF
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BOOTLOADER_MY_PRINTF.elf \

MAP_FILES += \
BOOTLOADER_MY_PRINTF.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BOOTLOADER_MY_PRINTF.list \

OBJCOPY_HEX += \
BOOTLOADER_MY_PRINTF.hex \


# All Target
all: main-build

# Main-build Target
main-build: BOOTLOADER_MY_PRINTF.elf secondary-outputs

# Tool invocations
BOOTLOADER_MY_PRINTF.elf BOOTLOADER_MY_PRINTF.map: $(OBJS) $(USER_OBJS) C:\denemeler2\BOOTLOADER_MY_PRINTF\STM32F072RBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "BOOTLOADER_MY_PRINTF.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\denemeler2\BOOTLOADER_MY_PRINTF\STM32F072RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BOOTLOADER_MY_PRINTF.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BOOTLOADER_MY_PRINTF.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BOOTLOADER_MY_PRINTF.list"
	@echo 'Finished building: $@'
	@echo ' '

BOOTLOADER_MY_PRINTF.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "BOOTLOADER_MY_PRINTF.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) BOOTLOADER_MY_PRINTF.elf BOOTLOADER_MY_PRINTF.hex BOOTLOADER_MY_PRINTF.list BOOTLOADER_MY_PRINTF.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
