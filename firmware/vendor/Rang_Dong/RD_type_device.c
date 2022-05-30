/*
 * RD_type_device.c
 *
 *  Created on: Apr 18, 2021
 *      Author: Dat_UTC
 */

#include"RD_type_device.h"
#include"Sensor_Flash.h"
#include "RD_Light_Sensor.h"
#include "RD_Pir_AC_Control.h"
extern u16 Gw_addr;
unsigned int confirm_fail_time = 0;
unsigned char confirm_fail = 0;
unsigned char bind_all_flag = 0;
unsigned int bind_time = 0;
u16 rd_box_addr = 0;
u8 type_device_to_gw_tx_buff[8];
u8 save_gw_addr_to_gw_tx_buff[8];
u8 save_gw_tx_buff[8] = {0};
u8 type_device_to_gw_tx_buff[8] = {0};


int RD_Messenger_Process_Type_Device(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	uint16_t Message_Header = (par[1]<<8)|par[0];
	if(Message_Header == RD_TYPE_DEVICE_RSP_HEADER){
		for(uint8_t i=0;i<8;i++)
			type_device_to_gw_tx_buff[i] = 0;

		type_device_to_gw_tx_buff[0] = (u8)(RD_TYPE_DEVICE_RSP_HEADER);
		type_device_to_gw_tx_buff[1] = (u8)(RD_TYPE_DEVICE_RSP_HEADER>>8);
		type_device_to_gw_tx_buff[2] = RD_SENSOR_TYPE_DEVICE;
		type_device_to_gw_tx_buff[3] = 02;
		type_device_to_gw_tx_buff[4] = 01;
		type_device_to_gw_tx_buff[5] = 0;
		type_device_to_gw_tx_buff[6] = 01;
		type_device_to_gw_tx_buff[7] = 00;
		mesh_tx_cmd2normal_primary(RD_OPCODE_TYPE_DEVICE_RSP, (u8 *)type_device_to_gw_tx_buff, TYPE_DEVICE_BUFF_LEN, 0x0001, 2);
	}
	else if(Message_Header == RD_SAVE_GW_ADDR_HEADER){
		Gw_addr = cb_par->adr_src;
		RD_FlashWriteGwAddr(Gw_addr);
		save_gw_tx_buff[0]=(u8)(RD_SAVE_GW_ADDR_HEADER);
		save_gw_tx_buff[1]=(u8)(RD_SAVE_GW_ADDR_HEADER >> 8);
		save_gw_tx_buff[2]=(u8)(Gw_addr);
		save_gw_tx_buff[3]=(u8)(Gw_addr>>8);
		mesh_tx_cmd2normal_primary(RD_OPCODE_TYPE_DEVICE_RSP, (u8 *)save_gw_tx_buff, 6, 0x0001, 2);
	}
	return 0;
}

int RD_Messenger_Process_Scene(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	uint16_t Message_Header;
	Message_Header = (par[1]<<8)|par[0];
	if(Message_Header == RD_HEADER_PIR_SENSOR_SAVE_SCENE)RD_FlashSaveSenceData(par);
	else if(Message_Header == RD_HEADER_PIR_SENSOR_CLEAR_SCENE)RD_ClearSceneInFlash(par);
	else if( Message_Header == RD_HEADER_SAVE_PIR_TIME){
		u16 time;
		time = (par[3]<<8)|par[2];
		//RD_FlashWrite_pir_time(time);
		mesh_tx_cmd2normal_primary(RD_OPCODE_SCENE_RSP,(u8 *)par,4, 0x0001, RSP_MAX);
	}
	return 0;
}


int RD_Messenger_Process_Null(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	return 0;
}

int RD_Messenger_Process_Null_1(u8 *par, int par_len, mesh_cb_fun_par_t *cb_par){
	return 0;
}




