/*
 * RD_pir_DC_Control.c
 *
 *  Created on: Apr 22, 2022
 *      Author: PC5
 */

#include "RD_pir_DC_Control.h"
#include "SoftUart.h"
#include "RD_Light_Sensor.h"
#include "RD_control.h"


extern unsigned char uartBuff[64];
unsigned int MotionTimeLine = 0;

pirPinTypeDef rdPir;


void pirDCInit(pirPinTypeDef *pir){

}

unsigned char timeHighFlag = 0,timeLowFlag = 0;

#define PIR_NOISE_CNT	16   //180ms
unsigned char pirDetect_poll_edge(pirPinTypeDef *pin){
	pin->flag.motionOneTime = 0;

	if(pin->timeOut.motionTimeOut <= SYS_32K_TICK_S){
		if(SYS_32K_TICK_S - pin->timeOut.motionTimeOut >= PIR_DC_FLAG_TIMEOUT ){
			pin->timeOut.motionTimeOut = SYS_32K_TICK_S;
			pin->flag.pirHighFlag = 0;
			pin->flag.pirLowFlag = 0;
		}
	}
	else{
		pin->timeOut.motionTimeOut = SYS_32K_TICK_S;
		pin->flag.pirHighFlag = 0;
		pin->flag.pirLowFlag = 0;
	}


	if(pin->state.pirHighState){
		pin->timeOut.flagHighTimeOut = SYS_32K_TICK_10MS;
		timeHighFlag = 1;
		pin->timeOut.motionTimeOut = SYS_32K_TICK_S;
	}
	else if(!pin->state.pirHighState){
		if(timeHighFlag){
			timeHighFlag = 0;
			if(SYS_32K_TICK_10MS - pin->timeOut.flagHighTimeOut >= PIR_NOISE_CNT){  
				pin->flag.pirHighFlag = 1;
			}
			else if(SYS_32K_TICK_10MS <  pin->timeOut.flagHighTimeOut ){
				pin->timeOut.flagHighTimeOut = SYS_32K_TICK_10MS;
			}
		}
	}

	if(pin->state.pirLowState){
		pin->timeOut.flagLowTimeOut = SYS_32K_TICK_10MS;
		timeLowFlag = 1;
		pin->timeOut.motionTimeOut = SYS_32K_TICK_S;
	}
	else if(!pin->state.pirLowState){
		if(timeLowFlag){
			timeLowFlag = 0;
			if(SYS_32K_TICK_10MS - pin->timeOut.flagLowTimeOut >= PIR_NOISE_CNT){ 
				pin->flag.pirLowFlag = 1;
			}
			else if(SYS_32K_TICK_10MS <  pin->timeOut.flagLowTimeOut ){
				pin->timeOut.flagLowTimeOut = SYS_32K_TICK_10MS;
			}
		}
	}

	if(pin->flag.pirHighFlag && pin->flag.pirLowFlag){
		pin->flag.pirHighFlag = 0;
		pin->flag.pirLowFlag = 0;
		pin->flag.motionFlag = 1;
		pin->flag.motionOneTime = 1;

		analogSave.motionFlagSave = pin->flag.motionFlag;
		pirSaveRegData(analogSave.data);

		MotionTimeLine = SYS_32K_TICK_MS;
	}
	return (pin->flag.motionFlag);
}

unsigned short pirGetRegData(){
	return (analog_read(PIR_TIMEOUT_REG0) << 8 | analog_read(PIR_TIMEOUT_REG1));
}


void pirSaveRegData(unsigned short data){
	analog_write(PIR_TIMEOUT_REG0,(data >> 8));
	analog_write(PIR_TIMEOUT_REG1,data);
}


void RD_ADC_init (unsigned int gpio)
{
	adc_init();
	adc_base_init(gpio);
	adc_power_on_sar_adc(1);
}

unsigned int RD_power_read()
{
    RD_ADC_init (GPIO_PC4);
	unsigned int power_read;
	float power_persent = 0;
	power_read = adc_sample_and_get_result();
	if(power_read <= 1150){
		power_persent = 0;
		return power_persent;
	}

	power_persent = (2.0*power_read-2200)/(3200-2200)*100;
	power_persent = (unsigned int) power_persent;
	if(power_persent < 0) power_persent = 0;
	else if(power_persent > 100) power_persent = 100;
	
	return power_persent;
	//return power_read;
}


void ledShowProvState(){
	if(is_provision_success()){
		gpio_write(LED_RED_PIN,1);
		gpio_write(LED_BLUE_PIN,0);
	}else{
		gpio_write(LED_RED_PIN,0);
		gpio_write(LED_BLUE_PIN,1);
	}
}

unsigned int motionTimeNow = 0;
unsigned char pirMotionCOntrol(pirFlagTypeDef *flag,unsigned char pirTime){
	if(flag->motionFlag){
		if( SYS_32K_TICK_MS >= MotionTimeLine){
			motionTimeNow = SYS_32K_TICK_MS - MotionTimeLine;
			if(motionTimeNow >= pirTime * 1000 ){
				flag->motionFlag = 0;
				analogSave.motionFlagSave = flag->motionFlag;
				pirSaveRegData(analogSave.data);
				MotionTimeLine = SYS_32K_TICK_MS;
				return 1;
			}
		}
		else{
			motionTimeNow = 0xffffffff - MotionTimeLine + SYS_32K_TICK_MS;
			if(motionTimeNow >= pirTime * 1000){
				flag->motionFlag = 0;
				analogSave.motionFlagSave = flag->motionFlag;
				pirSaveRegData(analogSave.data);
				MotionTimeLine = SYS_32K_TICK_MS;
				return 1;
			}
		}
	}
	return 0;
}

#define RD_POLL_TIME_10S		360 //  3600 giay

unsigned int pollTime = 0;

unsigned char pollCheck(){
	if(SYS_32K_TICK_10S >= pollTime){
		if(SYS_32K_TICK_10S - pollTime >= RD_POLL_TIME_10S){
			pollTime = SYS_32K_TICK_10S;

			analogSave.batteryTimeSave = pollTime;
			pirSaveRegData(analogSave.data);
			rdPir.flag.pirHighFlag = 0;
			rdPir.flag.pirLowFlag = 0;
			return 1;
		}
	}
	else{
		pollTime = SYS_32K_TICK_10S;
		analogSave.batteryTimeSave = pollTime;
		pirSaveRegData(analogSave.data);
		return 0;
	}
	return 0;
}
