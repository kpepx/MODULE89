################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CHESS.c \
../Core/Src/IK.c \
../Core/Src/PATH.c \
../Core/Src/PID.c \
../Core/Src/QEI.c \
../Core/Src/STEPPER.c \
../Core/Src/Serial.c \
../Core/Src/TASKSPACE.c \
../Core/Src/TRAJECTORY.c \
../Core/Src/TRAJECTORY_CIRCLE.c \
../Core/Src/cartesianJog.c \
../Core/Src/forwardKinematic.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/CHESS.o \
./Core/Src/IK.o \
./Core/Src/PATH.o \
./Core/Src/PID.o \
./Core/Src/QEI.o \
./Core/Src/STEPPER.o \
./Core/Src/Serial.o \
./Core/Src/TASKSPACE.o \
./Core/Src/TRAJECTORY.o \
./Core/Src/TRAJECTORY_CIRCLE.o \
./Core/Src/cartesianJog.o \
./Core/Src/forwardKinematic.o \
./Core/Src/main.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/CHESS.d \
./Core/Src/IK.d \
./Core/Src/PATH.d \
./Core/Src/PID.d \
./Core/Src/QEI.d \
./Core/Src/STEPPER.d \
./Core/Src/Serial.d \
./Core/Src/TASKSPACE.d \
./Core/Src/TRAJECTORY.d \
./Core/Src/TRAJECTORY_CIRCLE.d \
./Core/Src/cartesianJog.d \
./Core/Src/forwardKinematic.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core_A/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CHESS.d ./Core/Src/CHESS.o ./Core/Src/IK.d ./Core/Src/IK.o ./Core/Src/PATH.d ./Core/Src/PATH.o ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/QEI.d ./Core/Src/QEI.o ./Core/Src/STEPPER.d ./Core/Src/STEPPER.o ./Core/Src/Serial.d ./Core/Src/Serial.o ./Core/Src/TASKSPACE.d ./Core/Src/TASKSPACE.o ./Core/Src/TRAJECTORY.d ./Core/Src/TRAJECTORY.o ./Core/Src/TRAJECTORY_CIRCLE.d ./Core/Src/TRAJECTORY_CIRCLE.o ./Core/Src/cartesianJog.d ./Core/Src/cartesianJog.o ./Core/Src/forwardKinematic.d ./Core/Src/forwardKinematic.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o

.PHONY: clean-Core-2f-Src

