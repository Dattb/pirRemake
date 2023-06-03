/*
 * RD_type_device.h
 *
 *  Created on: Apr 18, 2021
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

#ifndef RD_TYPE_DEVICE_H_
#define RD_TYPE_DEVICE_H_


#define TYPE_DEVICE_BUFF_LEN       				11
#define RD_SENSOR_TYPE_DEVICE 					3


#define RD_HEADER_PIR_SENSOR_SEND_RGB_SCENE      0x0045
#define RD_HEADER_PIR_SENSOR_SAVE_SCENE          0x0145
#define RD_HEADER_PIR_SENSOR_CLEAR_SCENE         0x0245
#define RD_HEADER_SAVE_PIR_TIME			         0x0345


#define RD_GET_TYPE_DEVICE_RSP_HEADER       0x0003
#define RD_SAVE_GW_ADDR_HEADER        		0x0002
#define RD_TYPE_DEVICE_RSP_HEADER        	0x0003

#define RD_OPCODE_TYPE_DEVICE_SEND			(0xE0)
#define RD_OPCODE_TYPE_DEVICE_RSP			(0xE1)
#define RD_OPCODE_SCENE_SEND				(0xE2)
#define RD_OPCODE_SCENE_RSP					(0xE3)

#define RD_OPODE_SEND_DATA					(0xE5)
#define RD_OPODE_END						(0xE6)

int RD_Messenger_Process_Type_Device(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int RD_Messenger_Process_Scene(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int RD_Messenger_Process_Null(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
int RD_Messenger_Process_Null_1(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);

void Join_confirm_check();

void confirm_receive_check();
int RD_mesh_cmd_sig_cfg_bind(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);

void ledShow();
#endif /* RD_TYPE_DEVICE_H_ */
