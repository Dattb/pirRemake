/*
 * RD_Pir_AC_Control.h
 *
 *  Created on: May 4, 2021
 *      Author: Dat_UTC
 */
#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"

#include "vendor/common/scene.h"
#ifndef RD_PIR_AC_CONTROL_H_
#define RD_PIR_AC_CONTROL_H_

#define PIR_HIGH	GPIO_PB4
#define PIR_LOW		GPIO_PB1

#define PIR_HANG_TIME  				6   // khi co chuyen dong thi chan pir duoc keo len muc cao trong 6 s

#define PIR_INPUT    			    GPIO_PB4
#define	PIR_RESTORE_CNT				3000

#define PIR_LOOP_TIME				1
#define PIR_NOISE_CNT				400
#define PIR_FLAG_TIMEOUT					5000 // ms
#define PIR_MOTION_TIMEOUT					15000 // ms



#define CONFIRM_ENABLE		1

#define RD_PIR_SENSOR		1


void check_clock_time_uart();
void pir_calculate_motion_time(unsigned char cnt,unsigned int pir_flag_time_out,unsigned int motion_time_out);
void pir_detect_edge(unsigned char pir_high,unsigned char pir_low);
unsigned char  pir_check(unsigned char cnt , unsigned int pir_flag_time_out);
void Pir_loop (unsigned int pir_loop_time,unsigned char cnt,unsigned int pir_flag_time_out,unsigned int motion_time_out);
void RD_SensorSendRGBScene(u16 app_rgbid);
unsigned int RD_Light_Sensor_read_raw_val();
void RD_PIR_check_uart();
void pir_intput_init();
void pirSendData(unsigned char State);
#endif /* RD_PIR_AC_CONTROL_H_ */
