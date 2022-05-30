/*
 * SceneControl.c
 *
 *  Created on: May 29, 2022
 *      Author: PC5
 */

#include "RD_Pir_AC_Control.h"
#include "SceneControl.h"
#include "Sensor_Flash.h"

extern unsigned int real_lux_old;
extern RD_SensorSencesStoreTypedef RD_Sence_Store_obj[];

void checkMotion(unsigned char motionState){
	static unsigned char motionOld = 0;
	if(motionOld != motionState){
		motionOld = motionState;
		if(motionOld){
			RD_LightSensorControlSence(real_lux_old,motionOld);
		}
		else{
			RD_LightSensorControlSence(real_lux_old,motionOld);
		}
	}
}

unsigned char pirSendBuff[6] = {0};
void RD_LightSensorControlSence(u16 Sensor_Lux,unsigned char pirSceneState)
{
	pirSendBuff[0] = 0x05;
	RD_Send_Pir_motion(pirSceneState,0x0000,pirSendBuff);
	// Cho nay xu ly data nhan ve tu gateway, tach ra nhung phan data can su dung
	for(u8 i=0;i<2;i++){
		/*
		 * - Su dung Pir_condition de luu canh
		 * - Chi luu 2 canh :
		 * 		+ Co chuyen dong (co the ket hop anh sang hoac khong)
		 * 		+ Khong co chuyen dong (co the ket hop anh sang hoac khong)
		 */
		RD_Sensor_data_tdef Condition;
		u16 SceneID = (RD_Sence_Store_obj[i].SceneID[1]<<8)|RD_Sence_Store_obj[i].SceneID[0];
		Condition.data = RD_Sence_Store_obj[i].sensor_data[0]<<24|
						 RD_Sence_Store_obj[i].sensor_data[1]<<16|
						 RD_Sence_Store_obj[i].sensor_data[2]<<8;
		u8 PIR_Condition   = Condition.Pir_Conditon;
		u8 Light_Condition = Condition.Light_Conditon;
		u16 LuxLow = Condition.Lux_low*10;
		u16 LuxHight = Condition.Lux_hi*10;
			switch(Light_Condition)
			{
				case NOT_USE:{
					switch(PIR_Condition){
						case MOTION:{
							if (pirSceneState){
									access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
									access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
									RD_mesh_loop();
									RD_Send_Pir_motion(pirSceneState,SceneID,pirSendBuff);
							}
							break;
						}
						case NOT_MOTION:{
							if (!pirSceneState){
									access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
									access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
									RD_mesh_loop();
									RD_Send_Pir_motion(pirSceneState,SceneID,pirSendBuff);
							}
							break;
						}
					}
					break;

				}
				case GRE_LOW_LES_HIGHT:{
					if((Sensor_Lux >= LuxLow) && (Sensor_Lux <= LuxHight)){
						switch(PIR_Condition){
							case MOTION:{
								if(pirSceneState){
									if((SceneID != 0xffff) && (SceneID != 0x00)){
										access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
										access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
										RD_mesh_loop();
										RD_Send_Pir_motion(pirSceneState,SceneID,pirSendBuff);
									}
								}
								break;
							}
							case NOT_MOTION:{
								if (!pirSceneState){
									if((SceneID != 0xffff) && (SceneID != 0x00)){
										access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
										access_cmd_scene_recall(0xffff, 0, SceneID, CMD_NO_ACK, 2);
										RD_mesh_loop();
										RD_Send_Pir_motion(pirSceneState,SceneID,pirSendBuff);
									}
								}
								break;
							}
						}
					}
					break;
				}
			}
	}
	mesh_tx_cmd2normal_primary (SENSOR_STATUS, (u8 *)pirSendBuff, 6, 0x0001, 2);
}

void RD_mesh_loop()
{
	blt_sdk_main_loop ();
	mesh_loop_process();
}

void RD_Send_Pir_motion(u8 PIR_Condition,u16 SceneID,unsigned char *buff)
{
	*(buff+2) = PIR_Condition;
	*(buff+3) = 0x00;
	*(buff+4) = (u8)(SceneID);
	*(buff+5) = (u8)(SceneID>>8);
}



