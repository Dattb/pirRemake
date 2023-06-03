/*
 * SceneControl.c
 *
 *  Created on: May 29, 2022
 *      Author: PC5
 */

#include "RD_pir_DC_Control.h"
#include "SceneControl.h"
#include "Sensor_Flash.h"

extern u16 gwAddr;
extern unsigned int real_lux_old;


unsigned char pirSendBuff[6] = {0};
extern unsigned char scene0_Buffer[];
extern unsigned char scene1_Buffer[];
unsigned char callSceneFlag = 0;
extern unsigned int luxOld;
void RD_LightSensorControlSence(u16 Sensor_Lux,unsigned char pirSceneState)
{
	luxOld = Sensor_Lux;
	pirSendBuff[0] = 0x05;
	RD_Send_Pir_motion(pirSceneState,0x0000,Sensor_Lux,pirSendBuff);
	unsigned char dataScene[2][6];
	for(unsigned char i=0;i<6;i++){
		dataScene[0][i] = scene0_Buffer[i];
	}
	for(unsigned char i=0;i<6;i++){
		dataScene[1][i] = scene1_Buffer[i];
	}
	// Cho nay xu ly data nhan ve tu gateway, tach ra nhung phan data can su dung
	for(u8 i=0;i<2;i++){
		/*
		 * - Su dung Pir_condition de luu canh
		 * - Chi luu 2 canh :
		 * 		+ Co chuyen dong (co the ket hop anh sang hoac khong)
		 * 		+ Khong co chuyen dong (co the ket hop anh sang hoac khong)
		 */
		RD_Sensor_data_tdef Condition;
		u16 SceneID = (dataScene[i][1] << 8) | dataScene[i][0];
		Condition.data = (dataScene[i][2] << 24) | (dataScene[i][3] << 16) | (dataScene[i][4] << 8);
		u8 typeScene = dataScene[i][5];
		u8 PIR_Condition   = Condition.Pir_Conditon;
		u8 Light_Condition = Condition.Light_Conditon;
		u16 LuxLow = Condition.Lux_low*10;
		u16 LuxHight = Condition.Lux_hi*10;
		switch(Light_Condition)
		{
			case GRE_LOW_LES_HIGHT:{
				if((Sensor_Lux >= LuxLow) && (Sensor_Lux <= LuxHight)){
					switch(PIR_Condition){
						case MOTION:{
							if(pirSceneState){
								if((SceneID != 0xffff) && (SceneID != 0x00)){
									if(!typeScene){
										RD_Call_Scene(SceneID,SYS_32K_TICK_S);
										callSceneFlag = 1;
										rdMeshLoop();
									}
									RD_Send_Pir_motion(pirSceneState,SceneID,Sensor_Lux,pirSendBuff);
								}
							}
							break;
						}
						case NOT_MOTION:{
							if (!pirSceneState){
								if((SceneID != 0xffff) && (SceneID != 0x00)){
									if(!typeScene){
										callSceneFlag = 1;
										RD_Call_Scene(SceneID,SYS_32K_TICK_S);
										rdMeshLoop();
									}
									RD_Send_Pir_motion(pirSceneState,SceneID,Sensor_Lux,pirSendBuff);
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
	main_loop();
	mesh_tx_cmd2normal_primary (SENSOR_STATUS, (u8 *)pirSendBuff,8,gwAddr,2);
	main_loop();
}


void RD_mesh_loop()
{
	blt_sdk_main_loop ();
	mesh_loop_process();
}

void RD_Send_Pir_motion(u8 PIR_Condition,u16 SceneID,unsigned short lux,unsigned char *buff){
	*(buff+2) = PIR_Condition;
	*(buff+3) = 0x00;
	*(buff+4) = (u8)(SceneID);
	*(buff+5) = (u8)(SceneID>>8);
	*(buff+6) = (u8)(lux);
	*(buff+7) = (u8)(lux>>8);
}



void RD_Call_Scene(uint16_t Scene_ID, uint8_t Mess_ID){
	Call_Scene_Str Scene_Mess_Buff;
	Scene_Mess_Buff.Scene_ID[0] = (uint8_t) ((Scene_ID) & 0xFF );
	Scene_Mess_Buff.Scene_ID[1] = (uint8_t) ((Scene_ID >> 8) & 0xFF );
	Scene_Mess_Buff.Null_Byte   = Mess_ID;
	Scene_Mess_Buff.Tss[0]		= (uint8_t) ((TSS_DEFAULT) & 0xFF);
	Scene_Mess_Buff.Tss[1]		= (uint8_t) ((TSS_DEFAULT>>8) & 0xFF);
	Scene_Mess_Buff.Future[0]   = 0x00;
	Scene_Mess_Buff.Future[1]   = 0x00;
	Scene_Mess_Buff.Future[2]   = 0x00;
	uint8_t* Mess_Buff;
	Mess_Buff = (uint8_t *) (&Scene_Mess_Buff);

	mesh_tx_cmd2normal_primary(SCENE_RECALL_NOACK, Mess_Buff, 8, 0xffff, 2);
//	rdPrintf("scene buff:");
//	for(unsigned char i=0;i<8;i++){
//		rdPrintf("%x-",Mess_Buff[i]);
//	}
//	rdPrintf("\n");
}
