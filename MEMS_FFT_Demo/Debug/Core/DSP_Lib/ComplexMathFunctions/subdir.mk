################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_f32.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q15.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q31.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_f32.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q15.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q31.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_f32.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q15.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q31.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q15.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q31.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_f32.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q15.c \
../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q31.c 

OBJS += \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_f32.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q15.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q31.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_f32.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q15.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q31.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_f32.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q15.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q31.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_f32.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q15.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q31.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_f32.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q15.o \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q31.o 

C_DEPS += \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_f32.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q15.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q31.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_f32.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q15.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q31.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_f32.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q15.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q31.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_f32.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q15.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q31.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_f32.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q15.d \
./Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_f32.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_f32.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q15.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q15.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q31.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q31.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_conj_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_f32.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_f32.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q15.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q15.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q31.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q31.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_dot_prod_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_f32.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_f32.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q15.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q15.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q31.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q31.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_f32.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q15.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q15.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q31.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q31.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mag_squared_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_cmplx_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_f32.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_f32.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q15.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q15.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q31.o: ../Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q31.c Core/DSP_Lib/ComplexMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_FULL_LL_DRIVER -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER -DARM_MATH_CM4 '-D__FPU_PRESENT=1' -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/DSP_Lib/ComplexMathFunctions/arm_cmplx_mult_real_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

