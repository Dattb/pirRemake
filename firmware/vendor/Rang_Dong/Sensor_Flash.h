/*
 * Sensor_Flash.h
 *
 *  Created on: Jan 28, 2021
 *      Author: Dat UTC
 */
#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"

#ifndef SENSOR_FLASH_H_
#define SENSOR_FLASH_H_

#define RD_FLASH_ADDR								0x7f000


#define SCENE_FLASH_ADDR                            0x78000
#define RD_SAVE_GW_FLASH_ADDR						0x79000
# define FLASH_JOINT_STATE							0x7a000


#define MAX_SCENE_CAN_STORE							4
#define NUMBER_OF_SCENE                        		2
#define BYTE_OF_ONE_SCENE_OBJ				   		7
#define GW_ADDR_LEN							   		2
#define FLASH_BUFF_LEN                         		(NUMBER_OF_SCENE*BYTE_OF_ONE_SCENE_OBJ)


#define GW_SAVE_BUFF_LEN 							6


#define CONFIRM_DONE		0xaa
#define JOIN_FAIL			0xfe
#define JOIN_DONE			0x55

enum{
	typeJoinState = 0,
	typeConfirm,
	typePirTimes,
	typeGwAddr,
	typeScene0,
	typeScene1,
};

typedef struct{
	union{
		u32 data;
		struct{
			u32 store				:8;//8 bit not use
			u32 Lux_hi				:10;//10 bit lux hi
			u32 Lux_low				:10;//10 bit lux low
			u32 Light_Conditon		:3; // 3 bit low
			u32 Pir_Conditon		:1; // 1 bit hight (bit trong so cao nhat)
		};
	};
}RD_Sensor_data_tdef;



enum{
	NOT_MOTION					= 0,
	MOTION						= 1,
};

enum{
	NOT_USE                     = 0,
	LESS_THAN_EQUAL				= 1,
	GRE_LOW_LES_HIGHT 			= 2,
	GREATER_THAN				= 3,
};

typedef struct{
	u8 Header[2];
	u8 SceneID [2];
	u8 sensor_data[3];
	/*
	 * sensor_data  bao gom:  - pir condition 1 bit
	 * 							- light condition 3 bit
	 * 							- lux low 10 bit
	 * 							- lux hight 10 bit
	 */
}RD_SensorSencesStoreTypedef;

#define PIR_TIME_MIN	36 //s  default is 30s


void debugUartFlash();
int FlashReadData(unsigned char *joinState,unsigned char *confirmState,unsigned short *pirTime,unsigned short *gatewayAddr,unsigned char *scene0,unsigned char *scene1);
void FlashSaveData(unsigned char *Data,unsigned char type);

#endif /* SENSOR_FLASH_H_ */
