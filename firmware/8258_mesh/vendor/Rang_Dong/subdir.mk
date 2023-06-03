################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/Rang_Dong/RD_Light_Sensor.c \
../vendor/Rang_Dong/RD_control.c \
../vendor/Rang_Dong/RD_flash.c \
../vendor/Rang_Dong/RD_pir_DC_Control.c \
../vendor/Rang_Dong/RD_type_device.c \
../vendor/Rang_Dong/SceneControl.c \
../vendor/Rang_Dong/Sensor_Flash.c \
../vendor/Rang_Dong/SoftUart.c 

OBJS += \
./vendor/Rang_Dong/RD_Light_Sensor.o \
./vendor/Rang_Dong/RD_control.o \
./vendor/Rang_Dong/RD_flash.o \
./vendor/Rang_Dong/RD_pir_DC_Control.o \
./vendor/Rang_Dong/RD_type_device.o \
./vendor/Rang_Dong/SceneControl.o \
./vendor/Rang_Dong/Sensor_Flash.o \
./vendor/Rang_Dong/SoftUart.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/Rang_Dong/%.o: ../vendor/Rang_Dong/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\libs" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\mijia_ble_api" -D__PROJECT_MESH__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


