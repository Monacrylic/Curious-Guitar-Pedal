################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../SSD1306/RJA_SSD1306.cpp 

OBJS += \
./SSD1306/RJA_SSD1306.o 

CPP_DEPS += \
./SSD1306/RJA_SSD1306.d 


# Each subdirectory must supply rules for building sources it contributes
SSD1306/RJA_SSD1306.o: ../SSD1306/RJA_SSD1306.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Drivers/CMSIS/Include -I"D:/GuitarPedalWorkspace/Curious_Pedal/SSD1306" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"SSD1306/RJA_SSD1306.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

