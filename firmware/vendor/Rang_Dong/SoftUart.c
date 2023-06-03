/*
 * SoftUart.c
 *
 *  Created on: Apr 23, 2022
 *      Author: PC5
 */

#include "SoftUart.h"
#include "proj/common/printf.h"
sUartTypeDef sUart1;


void sUartInit(sUartTypeDef *uart){
	uart->txPin = SUART_TX_PIN;
	uart->rxPin = SUART_RX_PIN;
	sUartIoInit(uart->txPin,sUartTxMode);
	#if(SUART_RX_EN)
		sUartIoInit(uart->rxPin,sUartRxMode);
	#endif
	gpio_write(uart->txPin,1);
}

void sUartIoInit(GPIO_PinTypeDef pin, unsigned char mode){
	if(mode == sUartTxMode){
		gpio_set_func(pin, AS_GPIO);
		gpio_set_output_en(pin,1);
		gpio_set_input_en(pin,0);
	}
	else if(sUartRxMode){
		gpio_set_func(pin, AS_GPIO);
		gpio_set_output_en(pin,0);
		gpio_set_input_en(pin,1);
		#if(LOW_POWER_PULL_UP)
			gpio_setup_up_down_resistor(PIR_LOW,PM_PIN_PULLUP_1M);
		#endif
		#if(NORMAL_PULL_UP)
			gpio_setup_up_down_resistor(pin,PM_PIN_PULLUP_10K);
		#endif
	}
}


//RD_EDIT: baurate: 9600
//void softUartSend(sUartTypeDef *uart,unsigned char *buffer, unsigned char lenght){
//
//
//	for(unsigned char i=0;i<lenght;i++){
//		gpio_write(pin,0);
//		sleep_us(90);
//		for(unsigned char j=0;j<8;j++){
//			if(buffer[i] & (1<<j)){
//				gpio_write(pin,1);
//			}
//			else {
//				gpio_write(pin,0);
//			}
//			sleep_us(102);
//		}
//		gpio_write(pin,1);
//		sleep_us(102);
//	}
//
//}

void softUartSend(sUartTypeDef *uart,unsigned char *data){
	while(*data != '\0'){
		softUartSendByte(uart,*(data++));
	}
}


void softUartSendByte(sUartTypeDef *uart,unsigned char data){
	GPIO_PinTypeDef pin = uart->txPin;
	unsigned char r = irq_disable();
	gpio_write(pin,0);
	sleep_us(UART_TIME_BIT_START);
	for(unsigned char j=0;j<8;j++){
		if(data & (1<<j)){
			gpio_write(pin,1);
		}
		else {
			gpio_write(pin,0);
		}
		sleep_us(UART_BIT_TIME);
	}
	gpio_write(pin,1);
	sleep_us(UART_BIT_TIME);
	irq_restore(r);
}


void softUartSendInt(unsigned short data){
	static unsigned char temp[6] = {0};
	temp[0] = data / 10000 + 48;
	temp[1] = data % 10000 / 1000 + 48;
	temp[2] = data % 1000 / 100 + 48;
	temp[3] = data % 100 / 10 + 48;
	temp[4] = data % 10 + 48;
	temp[5] = '\n';
	for(unsigned char i=0;i<6;i++)
		softUartSendByte(&sUart1,temp[i]);
}



