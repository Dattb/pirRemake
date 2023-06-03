################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/common/app_beacon.c \
../vendor/common/app_health.c \
../vendor/common/app_heartbeat.c \
../vendor/common/app_provison.c \
../vendor/common/app_proxy.c \
../vendor/common/beacon.c \
../vendor/common/ble_ll_ota.c \
../vendor/common/blt_led.c \
../vendor/common/blt_soft_timer.c \
../vendor/common/cmd_interface.c \
../vendor/common/dual_mode_adapt.c \
../vendor/common/ev.c \
../vendor/common/factory_reset.c \
../vendor/common/fast_provision_model.c \
../vendor/common/flash_fw_check.c \
../vendor/common/generic_model.c \
../vendor/common/light.c \
../vendor/common/lighting_model.c \
../vendor/common/lighting_model_HSL.c \
../vendor/common/lighting_model_LC.c \
../vendor/common/lighting_model_xyl.c \
../vendor/common/mesh_common.c \
../vendor/common/mesh_fn.c \
../vendor/common/mesh_lpn.c \
../vendor/common/mesh_node.c \
../vendor/common/mesh_property.c \
../vendor/common/mesh_test_cmd.c \
../vendor/common/myprintf.c \
../vendor/common/ota_fw_ow.c \
../vendor/common/scene.c \
../vendor/common/scheduler.c \
../vendor/common/sensors_model.c \
../vendor/common/system_time.c \
../vendor/common/time_model.c \
../vendor/common/user_ali.c \
../vendor/common/user_ali_time.c \
../vendor/common/user_proc.c \
../vendor/common/vendor_model.c 

OBJS += \
./vendor/common/app_beacon.o \
./vendor/common/app_health.o \
./vendor/common/app_heartbeat.o \
./vendor/common/app_provison.o \
./vendor/common/app_proxy.o \
./vendor/common/beacon.o \
./vendor/common/ble_ll_ota.o \
./vendor/common/blt_led.o \
./vendor/common/blt_soft_timer.o \
./vendor/common/cmd_interface.o \
./vendor/common/dual_mode_adapt.o \
./vendor/common/ev.o \
./vendor/common/factory_reset.o \
./vendor/common/fast_provision_model.o \
./vendor/common/flash_fw_check.o \
./vendor/common/generic_model.o \
./vendor/common/light.o \
./vendor/common/lighting_model.o \
./vendor/common/lighting_model_HSL.o \
./vendor/common/lighting_model_LC.o \
./vendor/common/lighting_model_xyl.o \
./vendor/common/mesh_common.o \
./vendor/common/mesh_fn.o \
./vendor/common/mesh_lpn.o \
./vendor/common/mesh_node.o \
./vendor/common/mesh_property.o \
./vendor/common/mesh_test_cmd.o \
./vendor/common/myprintf.o \
./vendor/common/ota_fw_ow.o \
./vendor/common/scene.o \
./vendor/common/scheduler.o \
./vendor/common/sensors_model.o \
./vendor/common/system_time.o \
./vendor/common/time_model.o \
./vendor/common/user_ali.o \
./vendor/common/user_ali_time.o \
./vendor/common/user_proc.o \
./vendor/common/vendor_model.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/common/%.o: ../vendor/common/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\libs" -I"E:\D\source_code\Telink\CODE_Telink_BLE\firm_ver_1\PIR\pir_dc_sram_consite\firmware\vendor\common\mi_api\mijia_ble_api" -D__PROJECT_MESH__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


