/*
 * SoftUart.h
 *
 *  Created on: Apr 23, 2022
 *      Author: PC5
 */

#ifndef SOFTUART_H_
#define SOFTUART_H_

#include <stdarg.h>
#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"
#include "vendor/common/scene.h"


#define SOFT_UART_DEBUG		0

#define SUART_RX_EN		0


#define SUART_TX_PIN			GPIO_PA0
#define SUART_RX_PIN			GPIO_PC1


#define BAUD_RATE_4800	0
#define BAUD_RATE_9600	1
#define BAUD_RATE_19200	2
#define BAUD_RATE_57600	3

#define BAUD_RATE		BAUD_RATE_9600


#if(BAUD_RATE == BAUD_RATE_4800)

#elif(BAUD_RATE == BAUD_RATE_9600)
#define	UART_TIME_BIT_START	90			//us
#define UART_BIT_TIME	104			//us
#elif(BAUD_RATE == BAUD_RATE_19200)
#endif


enum{
	sUartTxMode,
	sUartRxMode,
};

typedef struct {
	GPIO_PinTypeDef txPin;
	GPIO_PinTypeDef rxPin;
#if(SUART_RX_EN)
	unsigned char rxBuffer[128];
	unsigned char rxReceiveFlag;
#endif
}sUartTypeDef;



extern sUartTypeDef sUart1;




#define LOW_POWER_PULL_UP	0
#define NORMAL_PULL_UP		1

void sUartInit(sUartTypeDef *uart);
void sUartIoInit(GPIO_PinTypeDef pin, unsigned char mode);
void softUartSendByte(sUartTypeDef *uart,unsigned char data);
void softUartSend(sUartTypeDef *uart,unsigned char *data);
void softUartSendInt(unsigned short data);

#endif /* SOFTUART_H_ */
