################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/common/mi_api/i2c_msc.c \
../vendor/common/mi_api/mijia_pub_proc.c \
../vendor/common/mi_api/telink_sdk_mible_api.c 

OBJS += \
./vendor/common/mi_api/i2c_msc.o \
./vendor/common/mi_api/mijia_pub_proc.o \
./vendor/common/mi_api/telink_sdk_mible_api.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/common/mi_api/%.o: ../vendor/common/mi_api/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\libs" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\mijia_ble_api" -D__PROJECT_MESH__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


