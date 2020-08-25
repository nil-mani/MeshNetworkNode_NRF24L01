################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Node_MeshNetwork.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Node_MeshNetwork.list \

OBJCOPY_BIN += \
Node_MeshNetwork.bin \


# All Target
all: Node_MeshNetwork.elf secondary-outputs

# Tool invocations
Node_MeshNetwork.elf: $(OBJS) $(USER_OBJS) C:\Users\nilus\Documents\STM32\Workspace\Node_MeshNetwork\STM32F303RETX_FLASH.ld
	arm-none-eabi-gcc -o "Node_MeshNetwork.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\nilus\Documents\STM32\Workspace\Node_MeshNetwork\STM32F303RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Node_MeshNetwork.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Node_MeshNetwork.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Node_MeshNetwork.list"
	@echo 'Finished building: $@'
	@echo ' '

Node_MeshNetwork.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Node_MeshNetwork.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
