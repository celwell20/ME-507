################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/VL53L0X/core/src/vl53l0x_api.cpp \
../Drivers/VL53L0X/core/src/vl53l0x_api_calibration.cpp \
../Drivers/VL53L0X/core/src/vl53l0x_api_core.cpp \
../Drivers/VL53L0X/core/src/vl53l0x_api_ranging.cpp \
../Drivers/VL53L0X/core/src/vl53l0x_api_strings.cpp 

OBJS += \
./Drivers/VL53L0X/core/src/vl53l0x_api.o \
./Drivers/VL53L0X/core/src/vl53l0x_api_calibration.o \
./Drivers/VL53L0X/core/src/vl53l0x_api_core.o \
./Drivers/VL53L0X/core/src/vl53l0x_api_ranging.o \
./Drivers/VL53L0X/core/src/vl53l0x_api_strings.o 

CPP_DEPS += \
./Drivers/VL53L0X/core/src/vl53l0x_api.d \
./Drivers/VL53L0X/core/src/vl53l0x_api_calibration.d \
./Drivers/VL53L0X/core/src/vl53l0x_api_core.d \
./Drivers/VL53L0X/core/src/vl53l0x_api_ranging.d \
./Drivers/VL53L0X/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X/core/src/%.o Drivers/VL53L0X/core/src/%.su: ../Drivers/VL53L0X/core/src/%.cpp Drivers/VL53L0X/core/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/celwe/STM32CubeIDE/workspace_1.9.0/TOF Test/Drivers/VL53L0X" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X-2f-core-2f-src

clean-Drivers-2f-VL53L0X-2f-core-2f-src:
	-$(RM) ./Drivers/VL53L0X/core/src/vl53l0x_api.d ./Drivers/VL53L0X/core/src/vl53l0x_api.o ./Drivers/VL53L0X/core/src/vl53l0x_api.su ./Drivers/VL53L0X/core/src/vl53l0x_api_calibration.d ./Drivers/VL53L0X/core/src/vl53l0x_api_calibration.o ./Drivers/VL53L0X/core/src/vl53l0x_api_calibration.su ./Drivers/VL53L0X/core/src/vl53l0x_api_core.d ./Drivers/VL53L0X/core/src/vl53l0x_api_core.o ./Drivers/VL53L0X/core/src/vl53l0x_api_core.su ./Drivers/VL53L0X/core/src/vl53l0x_api_ranging.d ./Drivers/VL53L0X/core/src/vl53l0x_api_ranging.o ./Drivers/VL53L0X/core/src/vl53l0x_api_ranging.su ./Drivers/VL53L0X/core/src/vl53l0x_api_strings.d ./Drivers/VL53L0X/core/src/vl53l0x_api_strings.o ./Drivers/VL53L0X/core/src/vl53l0x_api_strings.su

.PHONY: clean-Drivers-2f-VL53L0X-2f-core-2f-src

