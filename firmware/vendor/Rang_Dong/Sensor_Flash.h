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




#define SCENE_FLASH_ADDR                            0x78000//0x022000
#define RD_SAVE_GW_FLASH_ADDR						0x79000
#define RD_SAVE_LIGHT_SENSOR						0x7a000



#define FACTORY_RESET_FLASH_ADDR                    0x23000//0x022000


#define MAX_SCENE_CAN_STORE							4
#define NUMBER_OF_SCENE                        		2
#define BYTE_OF_ONE_SCENE_OBJ				   		7
#define GW_ADDR_LEN							   		2
#define FLASH_BUFF_LEN                         		(NUMBER_OF_SCENE*BYTE_OF_ONE_SCENE_OBJ)


#define GW_SAVE_BUFF_LEN 							6


#define RD_FACTORY_RESET_CNT_LEN  1
unsigned char RD_Factory_Reset_Flash_Write_Buff[RD_FACTORY_RESET_CNT_LEN];


enum{
	save_gw = 0,
	save_radar_time = 1,
	save_lux_threshold = 2,
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

void Clear_GwAddr_PirTime();
//void RD_FlashWrite_pir_time(u16 time);
void RD_FlashSaveSenceData(u8 *RD_SencePar);
void RD_FlashReadSceneData ();
void RD_ClearAllSceneInFlash();
void RD_ClearSceneInFlash(u8 *RD_ScenePar);
void RD_FlashWriteGwAddr(u16 *data);
void RD_Flash_read_GW_Addr(u16 *data);
#endif /* SENSOR_FLASH_H_ */
