################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := F:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./main.o" \
"./ucdriver/delay/delay.o" \
"./ucdriver/huidu/huidu.o" \
"./ucdriver/key/key.o" \
"./ucdriver/motor/motor.o" \
"./ucdriver/mpu6050/inv_mpu.o" \
"./ucdriver/mpu6050/inv_mpu_dmp_motion_driver.o" \
"./ucdriver/mpu6050/mpu_port.o" \
"./ucdriver/oled/oled.o" \
"./ucdriver/uart/uart.o" \
"./ucdriver/ball/ball.o" \
"./ucdriver/stepper/stepper.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include ucdriver/delay/subdir_vars.mk
-include ucdriver/huidu/subdir_vars.mk
-include ucdriver/key/subdir_vars.mk
-include ucdriver/motor/subdir_vars.mk
-include ucdriver/mpu6050/subdir_vars.mk
-include ucdriver/oled/subdir_vars.mk
-include ucdriver/uart/subdir_vars.mk
-include ucdriver/ball/subdir_vars.mk
-include ucdriver/stepper/subdir_vars.mk
-include subdir_rules.mk
-include ucdriver/delay/subdir_rules.mk
-include ucdriver/huidu/subdir_rules.mk
-include ucdriver/key/subdir_rules.mk
-include ucdriver/motor/subdir_rules.mk
-include ucdriver/mpu6050/subdir_rules.mk
-include ucdriver/oled/subdir_rules.mk
-include ucdriver/uart/subdir_rules.mk
-include ucdriver/ball/subdir_rules.mk
-include ucdriver/stepper/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
car_demo.out 

EXE_OUTPUTS__QUOTED += \
"car_demo.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "car_demo.out"

# Tool invocations
car_demo.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"F:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wall -Wl,-m"car_demo.map" -Wl,-i"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source" -Wl,-i"C:/Users/Administrator/workspace_ccstheia/car_demo" -Wl,-i"C:/Users/Administrator/workspace_ccstheia/car_demo/Debug/syscfg" -Wl,-i"F:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="car_demo_linkInfo.xml" -Wl,--rom_model -o "car_demo.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "main.o" "ucdriver\delay\delay.o" "ucdriver\huidu\huidu.o" "ucdriver\key\key.o" "ucdriver\motor\motor.o" "ucdriver\mpu6050\inv_mpu.o" "ucdriver\mpu6050\inv_mpu_dmp_motion_driver.o" "ucdriver\mpu6050\mpu_port.o" "ucdriver\oled\oled.o" "ucdriver\uart\uart.o" "ucdriver\ball\ball.o" "ucdriver\stepper\stepper.o" 
	-$(RM) "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "main.d" "ucdriver\delay\delay.d" "ucdriver\huidu\huidu.d" "ucdriver\key\key.d" "ucdriver\motor\motor.d" "ucdriver\mpu6050\inv_mpu.d" "ucdriver\mpu6050\inv_mpu_dmp_motion_driver.d" "ucdriver\mpu6050\mpu_port.d" "ucdriver\oled\oled.d" "ucdriver\uart\uart.d" "ucdriver\ball\ball.d" "ucdriver\stepper\stepper.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

