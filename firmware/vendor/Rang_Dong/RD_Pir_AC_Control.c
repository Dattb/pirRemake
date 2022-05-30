/*
 * RD_Pir_AC_Control.c
 *
 *  Created on: May 4, 2021
 *      Author: Dat_UTC
 */



#include "RD_Light_Sensor.h"
#include "RD_Pir_AC_Control.h"
#include "Sensor_Flash.h"
#include"RD_type_device.h"
#include"RD_control.h"
extern u16 rd_box_addr;
unsigned int pir_clock_time_read = 0;
extern unsigned char rd_factory_reset_control;
u16 Gw_addr = 0x0001;
u8 Pir_H_status_old,Pir_H_status_new,Pir_L_status_old,Pir_L_status_new;
u16 Pir_motion_cnt;
u8 RD_pir_motion_status=0;
u8 Pir_motion_status_old,Pir_motion_status_new;
u8 Pir_wakeup_flag = 0;
u16 Pir_wakeup_cnt = 0;
u16 pir_time;

u8 lux_changle;
u16 lux_raw_val,Lux_val_old = 0,Lux_val_new = 0,Lux_val;
u8 Pir_sensor_to_gw_tx_buff[8] = {0};

unsigned char _motion_old = 0;
unsigned char _motion_new = 0;
unsigned int motion_time_ = 0;
unsigned char motion_detected = 0;
#define MOTION_TIME_SET			5000  //ms

// #define	PIR_HI					PIR_HIGH
// #define	PIR_LOW					PIR_LOW
void RD_PIR_check_uart()
{
	//uart_CSend
	Pir_H_status_new = gpio_read(PIR_HIGH);
	Pir_L_status_new = gpio_read(PIR_LOW);

	if(Pir_H_status_old != Pir_H_status_new){
		if(gpio_read(PIR_HIGH)) uart_CSend("H = 1\r\n");
		else uart_CSend("H = 0 \r\n");
		Pir_H_status_old = Pir_H_status_new;
	}
	if(Pir_L_status_old != Pir_L_status_new){
		if(gpio_read(PIR_LOW)) uart_CSend("H = 1\r\n");
		else uart_CSend("H = 0 \r\n");
		Pir_L_status_old = Pir_L_status_new;
	}
	if(Pir_H_status_new != Pir_H_status_old){
		if(gpio_read(PIR_HIGH)) uart_CSend("H = 1\r\n");
		else uart_CSend("H = 0 \r\n");
		Pir_H_status_old = Pir_H_status_new;
	}
}


void pir_intput_init(){
	gpio_set_func(PIR_HIGH, AS_GPIO);
	gpio_set_output_en(PIR_HIGH,0);
	gpio_set_input_en(PIR_HIGH,1);
	gpio_setup_up_down_resistor(PIR_HIGH,PM_PIN_PULLUP_1M);

	gpio_set_func(PIR_LOW, AS_GPIO);
	gpio_set_output_en(PIR_LOW,0);
	gpio_set_input_en(PIR_LOW,1);
	gpio_setup_up_down_resistor(PIR_LOW,PM_PIN_PULLUP_1M);
}


unsigned long check_time_pir = 0;
unsigned char pir_flag_low = 0;
unsigned char pir_flag_high = 0;
unsigned int pir_low_cnt = 0;
unsigned int pir_high_cnt = 0;

unsigned int pir_timeout_high = 0;
unsigned int pir_timeout_low = 0;



void pir_detect_edge(unsigned char pir_high,unsigned char pir_low){
	static unsigned char read_high_old = 0,read_low_old = 0;
	if(read_high_old != pir_high){
		read_high_old = pir_high;
		if(read_high_old){
			pir_timeout_high = clock_time_ms();
			uart_CSend("H - 1\n");
		}
		else uart_CSend("H - 0\n");
	}

	if(read_low_old != pir_low){
		read_low_old = pir_low;
		if(read_low_old){
			pir_timeout_low = clock_time_ms();
			uart_CSend("L - 1\n");
		}
		else uart_CSend("L - 0\n");
	}

}
unsigned char motion_old = 0;
unsigned char pir_check(unsigned char cnt , unsigned int pir_flag_time_out){
	static unsigned char pir_flag  = 0;
	 if(clock_time_ms() - pir_timeout_low >= pir_flag_time_out){
		 pir_timeout_low = clock_time_ms();
		 pir_flag_low = 0;
		 pir_flag = 0;
	 }
	 if(clock_time_ms() - pir_timeout_high >= pir_flag_time_out){
		 pir_timeout_high = clock_time_ms();
		 pir_flag_high = 0;
		 pir_flag = 0;
	 }
	 if (gpio_read(PIR_HIGH)){
		 pir_high_cnt++;
		 if(pir_high_cnt > cnt){
			 pir_flag_high = 1;
		 }
	 }
	 else pir_high_cnt = 0;
	 if (gpio_read(PIR_LOW)){
		 pir_low_cnt++;
		 if(pir_low_cnt > cnt){
			 pir_flag_low = 1;
		 }
	 }
	 else pir_low_cnt = 0;
	 pir_detect_edge(pir_flag_high,pir_flag_low);
	 if(pir_flag_low && pir_flag_high){
		 pir_high_cnt = 0;
		 pir_low_cnt = 0;
		 pir_flag_low = 0;
		 pir_flag_high = 0;
		 pir_flag = 1;
		 if(!rd_factory_reset_control){
			if(is_provision_success()){
				gpio_write(LED_BLUE_PIN,0);
			}
			else{
				gpio_write(LED_RED_PIN,0);
			}
	     }
		 uart_CSend("pir flag ---- ON \n");
	 }
	 if(!rd_factory_reset_control){
		 if(!pir_flag){
			 gpio_write(LED_BLUE_PIN,1);
			 gpio_write(LED_RED_PIN,1);
		 }
	 }
	 return pir_flag;
}


// time is ms




#define  PIR_AC_RD_OR_FPT			0   // 1 is FPT 0 is RD

void pir_calculate_motion_time(unsigned char cnt,unsigned int pir_flag_time_out,unsigned int motion_time_out){
	_motion_new = pir_check(cnt,pir_flag_time_out);
	#if (PIR_AC_RD_OR_FPT)
		if(_motion_new){
			motion_time_ = clock_time_ms();
		}
	#endif
	
	if(_motion_old != _motion_new){
		_motion_old = _motion_new;
		if(_motion_old) {
			#if (!PIR_AC_RD_OR_FPT)
				motion_time_ = clock_time_ms();
			#endif
			motion_detected = 1;
			//access_cmd_scene_recall(0xffff,0,0x0005,CMD_NO_ACK, 2);
			uart_CSend("motion detected");
			check_clock_time_uart();
			//pirSendData(1);
			//uart_CSend("motion detected \n");
		}
	}

	if(motion_detected){
		if(clock_time_ms() - motion_time_ >= motion_time_out){
			motion_detected = 0;
			#if (!PIR_AC_RD_OR_FPT)
				_motion_old = motion_detected;
			#endif
			uart_CSend("motion off");
			check_clock_time_uart();
			//pirSendData(0);
		}
	}

}

void pirSendData(unsigned char State){
	unsigned char sendBuff[2]  = {0x05,State};
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)sendBuff, 2, 0x0001, 2);
}

void Pir_loop (unsigned int pir_loop_time,unsigned char cnt,unsigned int pir_flag_time_out,unsigned int motion_time_out){
	if(clock_time_ms() - pir_clock_time_read >= pir_loop_time){
		pir_clock_time_read = clock_time_ms();
		pir_calculate_motion_time(cnt,pir_flag_time_out,motion_time_out);
		rd_factory_reset(8,20);
	}
}


void check_clock_time_uart(){
	char UART_TempSend[128];
	sprintf(UART_TempSend," --- %d s\n",clock_time_s());
	uart_CSend(UART_TempSend);
}


//void rd_uart_printf(unsigned char *buff){
//	char UART_TempSend[128];
//	sprintf(UART_TempSend,buff,clock_time_s());
//	uart_CSend(UART_TempSend);
//}






