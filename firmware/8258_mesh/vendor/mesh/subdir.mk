################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/mesh/app.c \
../vendor/mesh/app_att.c \
../vendor/mesh/main.c 

OBJS += \
./vendor/mesh/app.o \
./vendor/mesh/app_att.o \
./vendor/mesh/main.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/mesh/%.o: ../vendor/mesh/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\libs" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\mijia_ble_api" -D__PROJECT_MESH__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


