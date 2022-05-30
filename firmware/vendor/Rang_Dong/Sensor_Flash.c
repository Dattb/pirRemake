#include "RD_Light_Sensor.h"
#include "RD_Pir_AC_Control.h"
#include "Sensor_Flash.h"
#include "RD_type_device.h"

RD_SensorSencesStoreTypedef RD_Sence_Store_obj[NUMBER_OF_SCENE];
u8 SceneID_Store[NUMBER_OF_SCENE];
u8  RD_Sence_Flash_Read_Buff [FLASH_BUFF_LEN];
u8  RD_Sence_Flash_Write_Buff[FLASH_BUFF_LEN];

u8  RD_Save_GW_Flash_Write_Buff[GW_SAVE_BUFF_LEN];
u8  RD_Save_GW_Flash_Read_Buff[GW_SAVE_BUFF_LEN];

extern u16 rd_box_addr;

void RD_FlashSaveSenceData(u8 *RD_ScenePar)
{
	RD_Sensor_data_tdef  COndition;
	COndition.data = (RD_ScenePar[4]<<24)|(RD_ScenePar[5]<<16)|(RD_ScenePar[6]<<8);
    RD_SensorSencesStoreTypedef *RD_Sence_Store_pointer;
    RD_Sence_Store_pointer = (RD_SensorSencesStoreTypedef *) RD_ScenePar;
	RD_Sence_Store_obj[COndition.Pir_Conditon] = *RD_Sence_Store_pointer;
	unsigned char *savePointer = (unsigned char *)(RD_Sence_Store_obj);
	flash_erase_sector(SCENE_FLASH_ADDR);
	flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,savePointer);
	RD_FlashReadSceneData();
	mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)RD_ScenePar,BYTE_OF_ONE_SCENE_OBJ, 0x0001, 2);
}

void RD_FlashReadSceneData()
{
	unsigned char *savePointer = (unsigned char *)RD_Sence_Store_obj;
	flash_read_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,savePointer);

	unsigned int sceneId[2] = {0};
	sceneId[0] = RD_Sence_Store_obj[0].SceneID[1]<<8|RD_Sence_Store_obj[0].SceneID[0];
	sceneId[1] = RD_Sence_Store_obj[1].SceneID[1]<<8|RD_Sence_Store_obj[1].SceneID[0];
	if(sceneId[0] != 0xffff || sceneId[1] != 0xffff){
		//rdPrintf("static_scene:%x - motion_scene:%x\n",sceneId[0],sceneId[1]);
	}
	else{
		//rdPrintf("no flash scene\n");
	}
}

void RD_FlashWriteGwAddr(u16 *data){
		unsigned char saveBuff[2];
		saveBuff[0] = *data;
		saveBuff[1] = *data>>8;
		flash_erase_sector(RD_SAVE_GW_FLASH_ADDR);
		flash_write_page(RD_SAVE_GW_FLASH_ADDR,2,saveBuff);
		flash_read_page(RD_SAVE_GW_FLASH_ADDR,2,saveBuff);
		*data = saveBuff[1]<<8|saveBuff[0];
		//rdPrintf("gw2:%x\n",*data);
}


void RD_Flash_read_GW_Addr(u16 *data){
	unsigned char buff_read[2] = {0};
	flash_read_page (RD_SAVE_GW_FLASH_ADDR, 2, buff_read);
	u16 Temp_data = 0;
	Temp_data = buff_read[1]<<8|buff_read[0];
	if(Temp_data != 0 && Temp_data != 0xffff){
		*data = Temp_data;
	}
}


void Clear_GwAddr_PirTime(){
	flash_erase_sector(RD_SAVE_GW_FLASH_ADDR);
}

void RD_ClearAllSceneInFlash()
{
	flash_erase_sector(SCENE_FLASH_ADDR);
	RD_FlashReadSceneData ();
}


void RD_ClearSceneInFlash(u8 *par)
{
	RD_SensorSencesStoreTypedef RD_Clear_scene = {0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	u16 sId = par[3]<<8|par[2];
	if(sId != 0x0000 && sId != 0xffff){
		for(u8 i=0;i<NUMBER_OF_SCENE;i++){
			if(( RD_Sence_Store_obj[i].SceneID[0] == par[2]) && (RD_Sence_Store_obj[i].SceneID[1] == par[3])){
				RD_Sence_Store_obj[i]= RD_Clear_scene;
				mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)par,4, rd_box_addr, 2);
			}
		}
	}
	unsigned char *clear_p = (unsigned char *)RD_Sence_Store_obj;
	flash_erase_sector(SCENE_FLASH_ADDR);
	flash_write_page(SCENE_FLASH_ADDR,FLASH_BUFF_LEN,clear_p);
	RD_FlashReadSceneData();
}

