################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ucdriver/stepper/%.o: ../ucdriver/stepper/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Administrator/workspace_ccstheia/car_demo" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/Debug" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/oled" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/uart" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/delay" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/key" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/huidu" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/motor" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/ball" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/stepper" -gdwarf-3 -Wall -MMD -MP -MF"ucdriver/stepper/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '
