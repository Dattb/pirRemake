/*
 * RD_Light_Sensor.h
 *
 *  Created on: May 10, 2021
 *      Author: Dat_UTC
 */

#ifndef RD_LIGHT_SENSOR_H_
#define RD_LIGHT_SENSOR_H_
#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"




#define OPT3001_RESULT_REGISTER                 0x00
#define OPT3001_RESULT_REGISTER_LEN             1
#define OPT3001_CONFIG_REGISTER                 0x01
#define OPT3001_CONFIG_REGISTER_LEN             1
#define RSP_MAX                                 2
#define GATEWAY_ADDR                            0x0001
#define OPT3001_CONFIG_REG_HIGH                 0xC4   // xem trong datasheet phan configure register
//#define OPT3001_CONFIG_REG_HIGH                 0xC4   // xem trong datasheet phan configure register
#define OPT3001_CONFIG_REG_LOW                  0x10   // xem trong datasheet phan configure register
//#define OPT3001_CONFIG_REG_LOW                  0x14   // xem trong datasheet phan configure register
#define     I2C_CLK_SPEED                       200000
#define     SLAVE_DEVICE_ADDR                   0x88



#define RD_OPCODE_TYPE_DEVICE_SEND			(0xE0)
#define RD_OPCODE_TYPE_DEVICE_RSP			(0xE1)
#define RD_OPCODE_SCENE_SEND				(0xE2)
#define RD_OPCODE_SCENE_RSP					(0xE3)

#define LUX_THRESH_HOLD						30
#define LUX_LOOP_TIME						5000  //ms

#ifndef RD_LIGHSENSOR
	#define RD_LIGHSENSOR	1
#endif
#define LUX_UART_DEBUG		0

u16 Light_Balance_P_control(u16 Set_Point,u16 Process_Val);
void RD_Light_Sensor_Lux_Balance(u16 Balance_Lux,u16 Sensor_Lux,u16 Group_ID);

/* brief : this function to convert lux data from IC OTP3001 to real lux, value is calibed
 * rsp_lux : is raw data read from IC
 * return is real lux after calculate and calib
 */

unsigned int RD_Light_Sensor_read_raw_val();
unsigned int CalculateLux(unsigned int rsp_lux);
void RD_Send_raw_Lux(u16 Lux_raw_val,unsigned int lux_real);
void Lux_loop (unsigned int Lux_loop_time);

unsigned short luxRead();
unsigned char LuxDetect(unsigned int threshold);
unsigned int genI2cLuxAlgorithm(unsigned int luxIn);
void Light_sensor_i2c_init();
void lightSendData(unsigned char luxI2C);
void pollSend();
#endif /* RD_LIGHT_SENSOR_H_ */



