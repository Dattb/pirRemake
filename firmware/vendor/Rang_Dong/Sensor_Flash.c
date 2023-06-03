/*
 * Sensor_Flash.c
 *
 *  Created on: Jun 24, 2022
 *      Author: PC5
 */



#include "RD_Light_Sensor.h"
#include "RD_pir_DC_Control.h"
#include "Sensor_Flash.h"
#include "RD_type_device.h"
#include "SoftUart.h"



extern unsigned char joinState;
extern unsigned short pirTime;
extern unsigned short gwAddr;
extern unsigned char confirmState;
unsigned char scene0_Buffer[6];
unsigned char scene1_Buffer[6];



void debugUartFlash(){
	sUartInit(&sUart1);
	sleep_ms(2);
	rdPrintf("************\nJoint:%x\nconfirm:%x\npTime:%x\ngwAddr:%x\n",joinState,confirmState,pirTime,gwAddr);
	rdPrintf("scene0:");
	for(unsigned char i=0;i<6;i++){
		rdPrintf("%x-",scene0_Buffer[i]);
	}
	rdPrintf("\nscene1:");
	for(unsigned char i=0;i<6;i++){
		rdPrintf("%x-",scene1_Buffer[i]);
	}
	rdPrintf("\n");
}

int FlashReadData(unsigned char *joinState,unsigned char *confirmState,unsigned short *pirTime,unsigned short *gatewayAddr,unsigned char *scene0,unsigned char *scene1){
	unsigned char flashReadBuffer[18];
	flash_read_page(RD_FLASH_ADDR,18,flashReadBuffer);

	*joinState = flashReadBuffer[0];

	*confirmState = flashReadBuffer[1];

	*pirTime = (flashReadBuffer[3]<<8) | (flashReadBuffer[2]);
	if(*pirTime < PIR_TIME_MIN || *pirTime == 0xffff) {
		*pirTime = PIR_TIME_MIN;
	}

	*gatewayAddr = (flashReadBuffer[5]<<8) | (flashReadBuffer[4]);
	if(*gatewayAddr == 0x0000 || *gatewayAddr == 0xffff){
		*gatewayAddr = 0x0001;
	}

	for(unsigned char i=0;i<6;i++){
		scene0[i] = flashReadBuffer[6 + i];
	}

	for(unsigned char i=0;i<6;i++){
		scene1[i] = flashReadBuffer[12 + i];
	}
	//debugUartFlash();
	return 1;
}

void FlashSaveData(unsigned char *Data,unsigned char type){
	unsigned char flashReadBuffer[18];
	flash_read_page(RD_FLASH_ADDR,18,flashReadBuffer);

	if(type == typeJoinState){
		flashReadBuffer[0] = *Data;
	}
	else if(type == typeConfirm){
		flashReadBuffer[1] = *Data;    //joined
	}
	else if(type == typePirTimes){
		flashReadBuffer[2] = Data[0];
		flashReadBuffer[3] = Data[1];
	}
	else if(type == typeGwAddr){
		flashReadBuffer[4] = Data[0];
		flashReadBuffer[5] = Data[1];
	}
	else if(type == typeScene0){
		for(unsigned char i=0;i<6;i++){
			flashReadBuffer[6 + i] = Data[i];
		}
	}
	else if(type == typeScene1){
		for(unsigned char i=0;i<6;i++){
			flashReadBuffer[12 + i] = Data[i];
		}
	}

	flash_erase_sector(RD_FLASH_ADDR);
	flash_write_page(RD_FLASH_ADDR,18,flashReadBuffer);
	FlashReadData(&joinState,&confirmState,&pirTime,&gwAddr,scene0_Buffer,scene1_Buffer);
	//debugUartFlash();
}




