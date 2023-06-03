/*
 * RD_type_device.c
 *
 *  Created on: Apr 18, 2021
 *      Author: Dat_UTC
 */
#include"RD_type_device.h"
#include"Sensor_Flash.h"
#include "RD_Light_Sensor.h"
#include "RD_pir_DC_Control.h"
unsigned short gwAddr = 0x0001;
unsigned int confirm_fail_time = 0;
unsigned char confirm_fail = 0;
unsigned char bind_all_flag = 0;
unsigned int bind_time = 0;

u8 save_gw_addr_to_gw_tx_buff[8];
u8 save_gw_tx_buff[8] = {0};
u8 type_device_to_gw_tx_buff[11] = {0};
extern unsigned char broadcast[11];
extern u16 pirMotionTime;
extern unsigned short pirTime;


extern unsigned char confirmDoneFlag;
extern unsigned int endConfigTime;
unsigned char  rebootAfterJoinFlag = 0;
int RD_Messenger_Process_Type_Device(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	uint16_t Message_Header = (par[1]<<8)|par[0];
	if(Message_Header == RD_TYPE_DEVICE_RSP_HEADER){
		for(uint8_t i=0;i<8;i++){
			type_device_to_gw_tx_buff[i] = 0;
		}
		type_device_to_gw_tx_buff[0] = (u8)(RD_TYPE_DEVICE_RSP_HEADER);
		type_device_to_gw_tx_buff[1] = (u8)(RD_TYPE_DEVICE_RSP_HEADER>>8);
		type_device_to_gw_tx_buff[2] = broadcast[0];
		type_device_to_gw_tx_buff[3] = broadcast[1];
		type_device_to_gw_tx_buff[4] = broadcast[2];
		type_device_to_gw_tx_buff[5] = broadcast[3];
		type_device_to_gw_tx_buff[6] = broadcast[6];
		type_device_to_gw_tx_buff[7] = broadcast[7];
		if(is_provision_success()){
			confirmDoneFlag = 1;
			endConfigTime = SYS_32K_TICK_MS;
		}
		rebootAfterJoinFlag = 1;
		mesh_tx_cmd2normal_primary(RD_OPCODE_TYPE_DEVICE_RSP, (u8 *)type_device_to_gw_tx_buff, 8,gwAddr, 2);
	}
	else if(Message_Header == RD_SAVE_GW_ADDR_HEADER){
		save_gw_tx_buff[0]  = (u8)(RD_SAVE_GW_ADDR_HEADER);
		save_gw_tx_buff[1]  = (u8)(RD_SAVE_GW_ADDR_HEADER >> 8);
		save_gw_tx_buff[2]  = (u8)(gwAddr);
		save_gw_tx_buff[3 ] = (u8)(gwAddr>>8);
		mesh_tx_cmd2normal_primary(RD_OPCODE_TYPE_DEVICE_RSP, (u8 *)save_gw_tx_buff,8,gwAddr, 2);
		gwAddr = cb_par->adr_src;
		FlashSaveData((unsigned char *)&gwAddr,typeGwAddr);
	}
	return 0;
}

extern unsigned char scene0_Buffer[];
extern unsigned char scene1_Buffer[];
int RD_Messenger_Process_Scene(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	uint16_t Message_Header;
	Message_Header = (par[1]<<8)|par[0];
	if(Message_Header == RD_HEADER_PIR_SENSOR_SAVE_SCENE){
		if(par[4] & 0x80) {
			FlashSaveData(&par[2],typeScene1);
		}
		else{
			FlashSaveData(&par[2],typeScene0);
		}
		mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)par,8,gwAddr,RSP_MAX);
	}
	else if(Message_Header == RD_HEADER_PIR_SENSOR_CLEAR_SCENE){
		if(par[2] == scene0_Buffer[0] && par[3] == scene0_Buffer[1]){
			for(unsigned char i=0;i<6;i++){
				scene0_Buffer[i] = 0xff;
			}
			FlashSaveData(scene0_Buffer,typeScene0);
			mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)par,4,gwAddr,RSP_MAX);
		}
		else if(par[2] == scene1_Buffer[0] && par[3] == scene1_Buffer[1]){
			for(unsigned char i=0;i<6;i++){
				scene1_Buffer[i] = 0xff;
			}
			FlashSaveData(scene1_Buffer,typeScene1);
			mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)par,4,gwAddr,RSP_MAX);
		}
	}
	else if( Message_Header == RD_HEADER_SAVE_PIR_TIME){
		pirTime = (par[3]<<8)|par[2];
		FlashSaveData((unsigned char *)&pirTime,typePirTimes);
		mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)par,4,gwAddr,RSP_MAX);
	}
	ledShow();
	return 0;
}


void ledShow(){
	RDReceviceMessageShow(3,100*1000);
	for(unsigned char i=0;i<100;i++){
		wd_clear();
		main_loop();
	}
}

int RD_Messenger_Process_Null(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	return 0;
}

int RD_Messenger_Process_Null_1(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	return 0;
}


void Join_confirm_check(){
	#if(CONFIRM_ENABLE)
		if(confirm_fail){
			if(clock_time_ms() - confirm_fail_time >= 1000){
				#if(SUART_DEBUG)
					rdPrintf("confirm fail");
				#endif
				kick_out();
			}
		}
	#endif
}

void confirm_receive_check(){

		if(bind_all_flag){
			if(clock_time_ms() - bind_time >= 120000){
				kick_out();
			}
		}

}
extern int mesh_cmd_sig_cfg_bind(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par);
extern unsigned char confirmState;
extern unsigned char joinState;

int RD_mesh_cmd_sig_cfg_bind(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	FlashReadData(&joinState,&confirmState,&pirTime,&gwAddr,scene0_Buffer,scene1_Buffer);
	if(joinState == JOIN_DONE){
		return(mesh_cmd_sig_cfg_bind(par, par_len,  cb_par));
	}
	unsigned char joinNet = 0x55;
	FlashSaveData(&joinNet,typeJoinState);
	return(mesh_cmd_sig_cfg_bind(par, par_len,  cb_par));
}





