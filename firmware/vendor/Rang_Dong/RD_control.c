/*
 * RD_control.c
 *
 *  Created on: Mar 12, 2022
 *      Author: PC5
 */

#include "RD_control.h"



unsigned int btn_cnt = 0;
unsigned char btn_status = 1;
unsigned char factory_reset_cnt = 0;

unsigned char btn_flag_old = 0,btn_flag_new = 0;

unsigned int factory_reset_timeout  = 0;
unsigned int power_cnt_reset = 0;
unsigned char check_prov_status_old = 0;
unsigned char rd_factory_reset_control = 0;
unsigned char rd_factory_reset(unsigned int cnt, unsigned char btn_threshold){
	if(clock_time_ms() - factory_reset_timeout >= BUTTON_TIMEOUT){
		factory_reset_cnt = 0;
		rd_factory_reset_control = 0;
		//uart_CSend("factory reset timeout\n");
		//gpio_write(LED_BLUE_PIN,1);
	}
	btn_flag_new = btn_detect(btn_threshold);
	if(btn_flag_old != btn_flag_new){
		factory_reset_timeout = clock_time_ms();
		btn_flag_old = btn_flag_new;
		if(!btn_flag_old){
			if(is_provision_success()){
				gpio_write(LED_BLUE_PIN,0);
			}
			else{
				gpio_write(LED_RED_PIN,0);
			}
			rd_factory_reset_control = 1;
			factory_reset_cnt++;
			char UART_TempSend[128];
			sprintf(UART_TempSend," factory_reset_cnt --- %d\n",factory_reset_cnt);
			uart_CSend(UART_TempSend);
			if(factory_reset_cnt >= cnt){
				factory_reset_cnt = 0;

				kick_out();
			}
		}
		else {
			gpio_write(LED_RED_PIN,1);
			gpio_write(LED_BLUE_PIN,1);
		}
	}
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
	gpio_write(LED_BLUE_PIN, 1);
}


void btn_init(){
	gpio_set_func(BUTTON_PIN, AS_GPIO);
	gpio_set_output_en(BUTTON_PIN,0);
	gpio_set_input_en(BUTTON_PIN,1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_10K);
}

unsigned long addr[5] = {0x7f000,0x7e000,0x7d000,0x7c000,0x7b000};
unsigned long flash_addr_power = 0x7f000;
void _on_off_power_factory_reset(){
	if(is_provision_success()) gpio_write(LED_BLUE_PIN, 0);
	else gpio_write(LED_RED_PIN, 0);
	sleep_ms(800);
	wd_clear();
	unsigned char flash_read_buff[4] = {0};
	unsigned long flash_write_cnt  = 0;
	for(unsigned char i=0;i<5;i++){
		uart_CSend("flash check\n");
		flash_read_page(addr[i],4,flash_read_buff);
		flash_write_cnt = (flash_read_buff[0]<<16)|(flash_read_buff[1]<<8)|flash_read_buff[2];
		flash_addr_power = addr[i];
//		char UART_TempSend[128];
//		sprintf(UART_TempSend," flash: %x --- flash read_write: %d --- reset_cnt: %d\n",addr[i],flash_write_cnt,flash_read_buff[3]);
//		uart_CSend(UART_TempSend);
		if(flash_write_cnt < 80000){

			flash_write_cnt++;
			flash_read_buff[0] = flash_write_cnt>>16;
			flash_read_buff[1] = flash_write_cnt>>8;
			flash_read_buff[2] = flash_write_cnt;

			flash_read_buff[3] += 1;
			char UART_TempSend[128];
			sprintf(UART_TempSend," flash: %x --- flash read_write: %d --- reset_cnt: %d\n",flash_addr_power,flash_write_cnt,flash_read_buff[3]);
			uart_CSend(UART_TempSend);
			if(flash_read_buff[3] >= 8){
				flash_read_buff[3] = 0;
				flash_erase_sector(flash_addr_power);
				flash_write_page(flash_addr_power,4,flash_read_buff);
				kick_out();

			}
			else{
				flash_erase_sector(flash_addr_power);
				flash_write_page(flash_addr_power,4,flash_read_buff);
			}
			break;
		}
		else if (flash_write_cnt == 0xffffff){
			uart_CSend("reset cnt first write\n");
			flash_read_buff[0] = 0;
			flash_read_buff[1] = 0;
			flash_read_buff[2] = 1;
			flash_read_buff[3] = 1;
			flash_write_cnt = (flash_read_buff[0]<<16)|(flash_read_buff[1]<<8)|(flash_read_buff[2]<<0);
			flash_erase_sector(flash_addr_power);
			flash_write_page(flash_addr_power,4,flash_read_buff);
			flash_read_page(flash_addr_power,4,flash_read_buff);
//			char UART_TempSend[128];
//			sprintf(UART_TempSend," flash: %x --- flash read_write: %d --- reset_cnt: %d\n",addr[i],flash_write_cnt,flash_read_buff[3]);
//			uart_CSend(UART_TempSend);
			break;
		}
	}
}






unsigned char _on_off_power_factory_reset_1(){
	if(is_provision_success()) gpio_write(LED_BLUE_PIN,0);
	else gpio_write(LED_RED_PIN, 0);
	wd_clear();
	sleep_ms(1000);
	wd_clear();

	unsigned char flash_read_buff[4] = {0};

	flash_read_page(0x7a000,4,flash_read_buff);
	if(flash_read_buff[3] ==0xff){
		flash_read_buff[3] = 0;
		flash_erase_sector(0x7a000);
		flash_write_page(0x7a000,4,flash_read_buff);
	}
	flash_read_buff[3] += 1;

	char UART_TempSend[128];
	sprintf(UART_TempSend," --- reset_cnt: %d\n",flash_read_buff[3]);
	uart_CSend(UART_TempSend);

	if(flash_read_buff[3] >= 8){
		flash_read_buff[3] = 0;
		flash_erase_sector(0x7a000);
		flash_write_page(0x7a000,4,flash_read_buff);
		kick_out();

	}
	else{
		flash_erase_sector(0x7a000);
		flash_write_page(0x7a000,4,flash_read_buff);
	}
	return 0;
}

void flash_clear_power_cnt(){
	static unsigned char clear_power_cnt_flag = 0;
	if(clock_time_ms() - power_cnt_reset >= 5000){
		if(!clear_power_cnt_flag){
			clear_power_cnt_flag = 0;
			unsigned char flash_read_buff[4] = {0};
			flash_read_page(flash_addr_power,4,flash_read_buff);
			flash_read_buff[3] = 0;
			flash_erase_sector(flash_addr_power);
			flash_write_page(flash_addr_power,4,flash_read_buff);
			flash_read_page(flash_addr_power,4,flash_read_buff);
			char UART_TempSend[128];
			sprintf(UART_TempSend,"reset cnt clear, cnt is %d\n",flash_read_buff[3]);
			uart_CSend(UART_TempSend);
			clear_power_cnt_flag = 1;
		}
	}
}

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
				gpio_write(LED_BLUE_PIN,0);
				gpio_write(LED_RED_PIN,0);
			}
			else if(loop_cnt < cnt){
				gpio_write(LED_BLUE_PIN,0);
				gpio_write(LED_RED_PIN,0);
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



void rd_get_provision_state(){
	if(check_prov_status_old != is_provision_success()){
		check_prov_status_old = is_provision_success();
		RD_light_prov_success_with_sleep(5,200*1000);
		//RD_light_ev_with_sleep(10, 100*1000);	//1Hz shine for  2.5 second
	}
}
