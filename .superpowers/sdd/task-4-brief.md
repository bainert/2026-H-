### Task 4: 鏋勫缓闆嗘垚锛坰ources.mk / subdir 鏂囦欢 / makefile锛?
**Files:**
- Modify: `Debug/sources.mk:120-129`锛圫UBDIRS锛?- Create: `Debug/ucdriver/ball/subdir_vars.mk`
- Create: `Debug/ucdriver/ball/subdir_rules.mk`
- Create: `Debug/ucdriver/stepper/subdir_vars.mk`
- Create: `Debug/ucdriver/stepper/subdir_rules.mk`
- Modify: `Debug/makefile:12-24`锛圤RDERED_OBJS锛夈€乣Debug/makefile:37-51`锛?include锛夈€乣Debug/makefile:162-163`锛坈lean锛?
**Interfaces:**
- Consumes: Task 1/2 鐨勬簮鏂囦欢
- Produces: 鍙湪 CCS 涓瀯寤虹殑宸ョ▼

- [ ] **Step 1: `Debug/sources.mk` 鈥?SUBDIRS 杩藉姞涓ょ洰褰?*

鍦?`ucdriver/uart \` 琛屽悗杩藉姞锛?
```makefile
ucdriver/ball \
ucdriver/stepper \
```

锛堟敞鎰忎繚鐣欐瘡琛岀粨灏剧殑 `\`锛屾渶鍚庝竴琛?`ucdriver/uart \` 鍘熸湁琛屽熬缁绗︿笉鍙橈級

- [ ] **Step 2: 鍒涘缓 `Debug/ucdriver/ball/subdir_vars.mk`**

```makefile
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ucdriver/ball/ball.c 

C_DEPS += \
./ucdriver/ball/ball.d 

OBJS += \
./ucdriver/ball/ball.o 

OBJS__QUOTED += \
"ucdriver\ball\ball.o" 

C_DEPS__QUOTED += \
"ucdriver\ball\ball.d" 

C_SRCS__QUOTED += \
"../ucdriver/ball/ball.c" 
```

- [ ] **Step 3: 鍒涘缓 `Debug/ucdriver/ball/subdir_rules.mk`**

锛堢洰褰曞悕 ball锛岀紪璇戝懡浠?-I 鍒楄〃杩藉姞 ball/stepper 涓や釜璺緞锛屽叾浣欑収鎶勭幇鏈?`Debug/ucdriver/uart/subdir_rules.mk` 鏍煎紡锛?
```makefile
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ucdriver/ball/%.o: ../ucdriver/ball/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2050/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Administrator/workspace_ccstheia/car_demo" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/Debug" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/ccs2050/mspm0_sdk_2_10_00_04/source" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/oled" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/uart" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/delay" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/key" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/huidu" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/motor" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/ball" -I"C:/Users/Administrator/workspace_ccstheia/car_demo/ucdriver/stepper" -gdwarf-3 -Wall -MMD -MP -MF"ucdriver/ball/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '
```

- [ ] **Step 4: 鍒涘缓 `Debug/ucdriver/stepper/subdir_vars.mk`**

```makefile
################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ucdriver/stepper/stepper.c 

C_DEPS += \
./ucdriver/stepper/stepper.d 

OBJS += \
./ucdriver/stepper/stepper.o 

OBJS__QUOTED += \
"ucdriver\stepper\stepper.o" 

C_DEPS__QUOTED += \
"ucdriver\stepper\stepper.d" 

C_SRCS__QUOTED += \
"../ucdriver/stepper/stepper.c" 
```

- [ ] **Step 5: 鍒涘缓 `Debug/ucdriver/stepper/subdir_rules.mk`**

锛堢洰褰曞悕 stepper锛?I 鍒楄〃鍚屾牱杩藉姞 ball/stepper 涓や釜璺緞锛?
```makefile
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
```

- [ ] **Step 6: 淇敼 `Debug/makefile`**

`ORDERED_OBJS` 鍧楋紙`"./ucdriver/uart/uart.o" \` 琛屽悗锛夎拷鍔狅細

```makefile
"./ucdriver/ball/ball.o" \
"./ucdriver/stepper/stepper.o" \
```

`-include` 鐨?subdir_vars.mk 鍧楋紙`ucdriver/uart/subdir_vars.mk` 琛屽悗锛夎拷鍔狅細

```makefile
-include ucdriver/ball/subdir_vars.mk
-include ucdriver/stepper/subdir_vars.mk
```

`-include` 鐨?subdir_rules.mk 鍧楋紙`ucdriver/uart/subdir_rules.mk` 琛屽悗锛夎拷鍔狅細

```makefile
-include ucdriver/ball/subdir_rules.mk
-include ucdriver/stepper/subdir_rules.mk
```

clean 鐩爣鐨勪袱琛?`-$(RM) "..." "ucdriver\uart\uart.o" ...`锛?62 琛岋級鍜?`... "ucdriver\uart\uart.d" ...`锛?63 琛岋級鏈熬鍒嗗埆杩藉姞锛?
```makefile
"ucdriver\ball\ball.o" "ucdriver\stepper\stepper.o"
"ucdriver\ball\ball.d" "ucdriver\stepper\stepper.d"
```

- [ ] **Step 7: 鑷**

妫€鏌ワ細`Debug/ucdriver/ball/`銆乣Debug/ucdriver/stepper/` 鐩綍瀛樺湪锛堟柊寤?subdir_vars.mk/rules.mk 鍓嶅厛 `mkdir`锛夛紱sources.mk 缁绗﹀畬鏁达紱makefile 涓夊锛圤RDERED_OBJS / 2 涓?-include 鍧?/ clean锛夐兘鍔犱簡瀵瑰簲鏉＄洰锛?I 鍒楄〃鍖呭惈涓や釜鏂拌矾寰勩€?
- [ ] **Step 8: Commit**

```bash
git add Debug/sources.mk Debug/makefile Debug/ucdriver/ball Debug/ucdriver/stepper
git commit -m "build: 娉ㄥ唽 ball/stepper 妯″潡鍒?CCS 鏋勫缓绯荤粺"
```

---

