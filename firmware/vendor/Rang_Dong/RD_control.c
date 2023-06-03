/*
 * RD_control.c
 *
 *  Created on: Mar 12, 2022
 *      Author: PC5
 */

#include "RD_control.h"
#include "RD_pir_DC_Control.h"
#include "SoftUart.h"
#include "RD_Light_Sensor.h"
#include "SceneControl.h"
#include "Sensor_Flash.h"
extern pirPinTypeDef rdPir;
unsigned int btn_cnt = 0;
unsigned char btn_status = 1;
unsigned char factory_reset_cnt = 0;
unsigned char btn_flag_old = 1,btn_flag_new = 1;


unsigned int power_cnt_reset = 0;
unsigned char check_prov_status_old = 0;
extern pirPinTypeDef rdPir;


extern unsigned char MotionTimeLine;
otaTypeDef rdOta;

unsigned short pirTime = 30;
extern u16 gwAddr;

extern unsigned char pollTime;
extern unsigned char deleteNode;


unsigned char btnPressUnProv = 0;
extern unsigned char btnPressFlag;
unsigned char otaModeCnt = 0;
unsigned char rd_factory_reset(unsigned int cnt, unsigned char btnState,unsigned char confirm){
		if(!btnState){
			btnPressFlag = 1;
			led_init();
			if(confirm == CONFIRM_DONE){
				gpio_write(LED_BLUE_PIN,LED_ON);
				gpio_write(LED_RED_PIN,LED_OFF);
			}
			else{
				gpio_write(LED_BLUE_PIN,LED_OFF);
				gpio_write(LED_RED_PIN,LED_ON);
			}

			rdPir.timeOut.motionTimeOut = SYS_32K_TICK_S;
			sleep_ms(10);
			otaModeCnt = 0;
			while(!gpio_read(BUTTON_PIN)){
				wd_clear();
				if(resetDetect()){
					kick_out();
				}
			}
			if(confirm != CONFIRM_DONE){
				btnPressUnProv = 1;
				cpu_set_gpio_wakeup(BUTTON_PIN,0,1);
				gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
				gpio_core_wakeup_enable_all (1);
				cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K,PM_WAKEUP_PAD,clock_time() + 180000*CLOCK_SYS_CLOCK_1MS);
			}
		}
		else {
			gpio_write(LED_RED_PIN,LED_OFF);
			gpio_write(LED_BLUE_PIN,LED_OFF);
		}
	return 0;
}


unsigned char btn_detect(unsigned char btn_threshold){
	 if (!gpio_read(BUTTON_PIN)){
		 btn_cnt++;
		 if(btn_cnt > btn_threshold){
			 btn_status = 0;
		 }
	 }
	 else {
		 btn_cnt = 0;
		 btn_status = 1;
	 }
	 return btn_status;
}



void led_init(){
	gpio_set_func(LED_RED_PIN, AS_GPIO);
	gpio_set_output_en(LED_RED_PIN, 1);
	gpio_set_input_en(LED_RED_PIN, 0);
	gpio_write(LED_RED_PIN, 1);

	gpio_set_func(LED_BLUE_PIN, AS_GPIO);
	gpio_set_output_en(LED_BLUE_PIN, 1);
	gpio_set_input_en(LED_BLUE_PIN, 0);
	gpio_write(LED_BLUE_PIN, 0);
}


void btn_init(){
	gpio_set_func(BUTTON_PIN, AS_GPIO);
	gpio_set_output_en(BUTTON_PIN,0);
	gpio_set_input_en(BUTTON_PIN,1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_10K);
}

unsigned long addr[5] = {0x7f000,0x7e000,0x7d000,0x7c000,0x7b000};
unsigned long flash_addr_power = 0x7f000;

void flash_test_data(unsigned long addr, unsigned long data){
	unsigned char test_buff[4] = {0};
	test_buff[0] = data>>16;
	test_buff[1] = data>>8;
	test_buff[2] = data;
	flash_erase_sector(addr);
	flash_write_page(flash_addr_power,4,test_buff);
}



void led_show_provision_success(unsigned char cycle, unsigned int cnt,unsigned char *flag){
	static unsigned int loop_cnt = 0;
	static unsigned int effect_cycle = 0;
	if(*flag){
		if(effect_cycle < cycle){
			loop_cnt++;
			if(loop_cnt < cnt){
				gpio_write(LED_BLUE_PIN,LED_ON);
				gpio_write(LED_RED_PIN,LED_ON);
			}
			else if(loop_cnt < cnt){
				gpio_write(LED_BLUE_PIN,LED_OFF);
				gpio_write(LED_RED_PIN,LED_OFF);
			}
			else{
				effect_cycle ++;
				loop_cnt = 0;
			}
		}
		else {
			*flag = 0;
		}
	}
}

void pirInit(){

	gpio_set_input_en(PIR_DC_HIGH,1);
	gpio_setup_up_down_resistor(PIR_DC_HIGH,PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(PIR_DC_LOW,1);
	gpio_setup_up_down_resistor(PIR_DC_LOW,PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(BUTTON_PIN,1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_UP_DOWN_FLOAT);
}

void rd_get_provision_state(){
	if(check_prov_status_old != is_provision_success()){
		check_prov_status_old = is_provision_success();
		RD_light_prov_success_with_sleep(5,200*1000);
		//RD_light_ev_with_sleep(10, 100*1000);	//1Hz shine for  2.5 second
	}
}

void sleepBeforSend(unsigned char motionFlag,unsigned char motionFlagOld,unsigned char luxFlag, unsigned char poll){
	if( (motionFlagOld == motionFlag) && !luxFlag && !poll) {
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_HIGH)) cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 0);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_LOW)) cpu_set_gpio_wakeup (PIR_DC_LOW, 1, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
		gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
		gpio_core_wakeup_enable_all (1);
		if(motionFlag){
			if(pirTime / 60 < 1){
				cpu_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 10000*CLOCK_SYS_CLOCK_1MS);
			}
			else if(pirTime / 60 < 3){
				cpu_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 20000*CLOCK_SYS_CLOCK_1MS);
			}
			else if(pirTime / 60 < 5){
				cpu_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 30000*CLOCK_SYS_CLOCK_1MS);
			}
			else {
				cpu_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 60000*CLOCK_SYS_CLOCK_1MS);
			}
		}
		else
			cpu_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 120000*CLOCK_SYS_CLOCK_1MS);
	}
}

extern unsigned int luxOld;


extern unsigned int luxCalibToSend;
void sensorSendData(unsigned char motionFlag, unsigned char motionFlagOld, unsigned char luxSendFlag,unsigned char btnPress,unsigned char poll){
	if(!btnPress){
		if(motionFlagOld != motionFlag){
			led_init();
			main_loop();
			RD_LightSensorControlSence(luxRead(),motionFlag);
			//for(unsigned int i=0;i<150;i++) main_loop();
		}
		else {
			if(poll){
				pollSend();
				pollTime = SYS_32K_TICK_10S;
				for(unsigned int i=0;i<550;i++) main_loop();
				if(deleteNode){
					for(unsigned int i=0;i<1500;i++) main_loop();
				}
			}
		}

		if(luxSendFlag){
			main_loop();
			lightSendData(luxOld);
			//for(unsigned int i=0;i<150;i++) main_loop();
		}
	}
	else {
		pollSend();
		pollTime = SYS_32K_TICK_10S;
		for(unsigned int i=0;i<2000;i++) main_loop();
		if(deleteNode){
			for(unsigned int i=0;i<1500;i++) main_loop();
		}
	}
}


//void lightSendData(unsigned short lux){
//	lux_send_buff[0] = 0x0d;
//	lux_send_buff[1] = lux;
//	lux_send_buff[2] = lux >>8;
//	lux_send_buff[3] = powerData;
//	mesh_tx_cmd2normal(0xe5,(u8 *) lux_send_buff,4, rd_node_addr ,rd_box_addr, 2);
//}

void pirSendData(unsigned char motionSendState,unsigned char power){
	unsigned char pir_send_buff[3] = {0x0c,motionSendState,0x00};
	pir_send_buff[2] = power;
	mesh_tx_cmd2normal_primary (SENSOR_STATUS, (u8 *)pir_send_buff,3,gwAddr,2);
}

void sleepAfterSend(){
	if(!rdPir.flag.motionFlag){
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_HIGH)) cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 0);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_LOW)) cpu_set_gpio_wakeup (PIR_DC_LOW, 1, 0);     // level : 1 (high); 0 (low)
	}
	cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
	gpio_core_wakeup_enable_all (1);
	cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K,PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 5000*CLOCK_SYS_CLOCK_1MS);
}



void sleepMode(){
	cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
	gpio_core_wakeup_enable_all (1);
	cpu_sleep_wakeup(DEEPSLEEP_MODE,PM_WAKEUP_PAD,clock_time() + 5000*CLOCK_SYS_CLOCK_1MS);
}

unsigned char resetDetect(){
	
	sleep_ms(100);
	otaModeCnt++;
	if(otaModeCnt > 50){
		return 1;
	}
	return 0;
}


extern unsigned char btnPressFlag;
unsigned char  rdOtaDetect(otaTypeDef *ota){
	if(!btnPressFlag){
		return 0;
	}
	static unsigned char btn_flag_new = 0,btn_flag_old = 0;
	btn_flag_new = btn_detect(20);
	if(btn_flag_new == btn_flag_old){
		return 0;
	}
	btn_flag_old = btn_flag_new;
	if(!btn_flag_old){
		ota->otaCnt++;
		if(ota->otaCnt >= 4){
			ota->otaCnt = 0;
			ota->otaFlag = 1;
			ota->otaTime = SYS_32K_TICK_MS;
			RD_light_ota_with_sleep(8,100*1000);
		}
	}
	return 0;
}

void otaConcol(otaTypeDef *ota){
	if(ota->otaFlag && (SYS_32K_TICK_MS - ota->otaTime) >= 120000){
		ota->otaFlag = 0;
	}
}

void rd_deep_sleep_setup(){
	cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
	cpu_set_gpio_wakeup (PIR_DC_HIGH, 0, 1);     // level : 1 (high); 0 (low)
	cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
	cpu_set_gpio_wakeup (PIR_DC_LOW,0, 1);     // level : 1 (high); 0 (low)
	if(gpio_read(PIR_DC_HIGH)){
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 0, 1);     // level : 1 (high); 0 (low)
	}
	if(!gpio_read(PIR_DC_HIGH)) {
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 0, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
	}

	if(gpio_read(PIR_DC_LOW)){
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,0, 1);     // level : 1 (high); 0 (low)
	}
	if(!gpio_read(PIR_DC_LOW)){
		cpu_set_gpio_wakeup (PIR_DC_LOW,0, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
	}
	cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
	gpio_core_wakeup_enable_all (1);
}


extern unsigned char confirmState;
 void provFailProc(){
	 if(is_provision_success() && confirmState != CONFIRM_DONE){
		 kick_out();
	 }
 }


 int is_clock_time_done(unsigned int *time,unsigned int period){
 	if(SYS_32K_TICK_MS - *time >= period){
 		return 1;
 	}
 	else if(SYS_32K_TICK_MS < *time){
 		*time = SYS_32K_TICK_MS;
 		return 0;
 	}
 	return 0;
 }
