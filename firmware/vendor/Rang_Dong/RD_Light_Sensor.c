/*
 * RD_Light_Sensor.c
 *
 *  Created on: May 10, 2021
 *      Author: Dat_UTC
 */




#include "RD_Light_Sensor.h"
#include "RD_pir_DC_Control.h"
#include "SoftUart.h"


unsigned int luxSendTime = 0;
unsigned int luxCalibToSend = 0;

u8 RD_i2c_rx_buff[2] = {0};
u8 RD_i2c_tx_buff[2] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};
u8 light_sensor_to_gw_tx_buff[6]= {0};

extern u16 Lux_val_new;
extern unsigned short gwAddr;

unsigned int CalculateLux(unsigned int rsp_lux)
{
	unsigned int lux_LSB = 0;
	unsigned char lux_MSB = 0;
	unsigned int lux_Value = 0;
	unsigned int pow = 1;
	unsigned char i;
	lux_LSB = rsp_lux & 0x0FFF;
	lux_MSB = ((rsp_lux>>12) & 0x0F);
	for(i=0;i<lux_MSB;i++){
		pow=pow*2;
	}
	lux_Value = 0.01 * pow * lux_LSB;
	// RD_EDIT: calib lux
	#if	LUX_UART_DEBUG
		char UART_TempSend[128];
		sprintf(UART_TempSend,"Lux befor calib %d \n",lux_Value);
		uart_CSend(UART_TempSend);
	#endif
	if(lux_Value > 60){
		int calib_lux = 1.7339 * lux_Value - 39.868;
		lux_Value = calib_lux;
	}
	return lux_Value;
}


u16 light_sensor_read_cnt=0;
u32 time_send_lux;
unsigned int  real_lux_new = 0;
unsigned int luxOld = 0;
unsigned int rawLux = 0;

unsigned short luxRead(){
	unsigned short luxRead = CalculateLux(RD_Light_Sensor_read_raw_val());
	if(luxRead > 0xffff) luxRead = 0xffff;
	luxSendTime = SYS_32K_TICK_S;

	return luxRead;
}



extern unsigned char startBootFlag;
unsigned char LuxDetect(unsigned int threshold){
	if(startBootFlag){
		startBootFlag = 0;
		luxSendTime = SYS_32K_TICK_S;
	}
	if(SYS_32K_TICK_S - luxSendTime >= 600){ //s
		luxSendTime = SYS_32K_TICK_S;
		rawLux = RD_Light_Sensor_read_raw_val();
		real_lux_new = CalculateLux(rawLux);
		luxCalibToSend = genI2cLuxAlgorithm(real_lux_new);
		if(real_lux_new > 0xffff) real_lux_new = 0xffff;
		if(luxOld > real_lux_new){
			if(luxOld - real_lux_new >= threshold){
				luxOld = real_lux_new;
				return 1;
			}
		}
		else if(luxOld < real_lux_new) {
			if((real_lux_new - luxOld >= threshold)){
				luxOld = real_lux_new;
				return 1;
			}
		}
	}
	else if (luxSendTime > SYS_32K_TICK_S){
		luxSendTime = SYS_32K_TICK_S;
	}
	return 0;
}



unsigned int genI2cLuxAlgorithm(unsigned int luxIn){

	unsigned int mantisa = 0;
	unsigned int luxI2C = 0;
	unsigned int tempLux = 0;

	//printf("i2c lux:%d\n",tempLux);
	for(unsigned char i=0;i<15;i++){
		mantisa = 1<<i;
		tempLux = luxIn * 100 / mantisa;
		if(tempLux < 0x0fff){
			luxI2C = tempLux;
			break;
		}
	}
	unsigned int mantisaTemp = mantisa;
	mantisa = 0;
	for (unsigned char i=0;i<15;i++){
		if(mantisaTemp % 2 == 0 && mantisaTemp != 0){
			mantisaTemp = mantisaTemp/2;
			mantisa++;
		}
	}
	unsigned int i2cFrame = (mantisa <<12)|luxI2C;
	//printf("i2c frame :%x - mantisa: %x  -  i2c data: %x",i2cFrame,mantisa,luxI2C);
	return i2cFrame;
}


void lightSendData(unsigned char luxI2C){
	unsigned char buffSend[6] = {0x04,0x00,0x00,0x00,0x00,0x00};
	buffSend[2] = luxCalibToSend >> 8;
	buffSend[3] = luxCalibToSend;
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)buffSend,6, gwAddr, 2);
}


void pollSend(){
	unsigned char powerRead =  RD_power_read();
	unsigned char buffSend[4] = {0x01,0x00,0x00,powerRead};
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)buffSend,4, gwAddr, 2);
}

unsigned int RD_Light_Sensor_read_raw_val() {
	u16  lux_read_data;
	u8 RD_i2c_rx_buff[3] = {0};
	u8 RD_i2c_tx_buff[2] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};

	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(SLAVE_DEVICE_ADDR,(unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));

	i2c_write_series(OPT3001_CONFIG_REGISTER,OPT3001_CONFIG_REGISTER_LEN,(u8 *)RD_i2c_tx_buff, 2);
	i2c_read_series(OPT3001_RESULT_REGISTER,OPT3001_RESULT_REGISTER_LEN, (u8 *)RD_i2c_rx_buff, 3);

	lux_read_data = (RD_i2c_rx_buff[0]<<8) | RD_i2c_rx_buff[1];
	return lux_read_data;
}


void RD_Send_raw_Lux(u16 Lux_raw_val,unsigned int lux_real)
{
	*(light_sensor_to_gw_tx_buff)   = 0x04;
	*(light_sensor_to_gw_tx_buff+1) = 0x00;
	*(light_sensor_to_gw_tx_buff+2)   = (u8)(Lux_raw_val>>8);
	*(light_sensor_to_gw_tx_buff+3) = (u8)(Lux_raw_val);
	*(light_sensor_to_gw_tx_buff+4)   = (u8)(lux_real>>8);
	*(light_sensor_to_gw_tx_buff+5) = (u8)(lux_real);
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)light_sensor_to_gw_tx_buff, 6, 0x0001, 2);
}

void Light_sensor_i2c_init(){
	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(SLAVE_DEVICE_ADDR,(unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));
}
unsigned int Lux_clock_time_read = 0;
void Lux_loop (unsigned int Lux_loop_time){
	if(clock_time_ms() - Lux_clock_time_read >= Lux_loop_time){
		Lux_clock_time_read = clock_time_ms();
		//Lux_send(LUX_THRESH_HOLD);
	}
}
