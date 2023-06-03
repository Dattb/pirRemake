/*
 * RD_control.h
 *
 *  Created on: Mar 12, 2022
 *      Author: PC5
 */

#ifndef RD_CONTROL_H_
#define RD_CONTROL_H_
#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"

typedef struct {
	unsigned char otaFlag;
	unsigned char otaCnt;
	unsigned int otaTime;
}otaTypeDef;

#define CLCK_TIME_32K_MS		32767
#define SYSTEM_32K_TICK			32767


#define BUTTON_PIN 		GPIO_PD4
#define LED_RED_PIN 	GPIO_PB6
#define LED_BLUE_PIN 	GPIO_PB5


#define LED_ON		0
#define LED_OFF		1
#define BUTTON_TIMEOUT		3000



void btn_init();
void led_init();
unsigned char btn_detect(unsigned char cnt);
unsigned char rd_factory_reset(unsigned int cnt, unsigned char btnState,unsigned char confirm);
void led_test();
void led_write(GPIO_PinTypeDef pin,unsigned char status);
void flash_test_data(unsigned long addr, unsigned long data);
void led_show_provision_success(unsigned char cycle, unsigned int cnt,unsigned char *flag);
void rd_get_provision_state();
#endif /* RD_CONTROL_H_ */

void sleepBeforSend(unsigned char motionFlag,unsigned char motionFlagOld,unsigned char luxFlag,unsigned char poll);
void sleepAfterSend();
void sleepMode();
void sensorSendData(unsigned char motionFlag, unsigned char motionFlagOld, unsigned char luxSendFlag,unsigned char btnPress,unsigned char poll);
void pirSendData(unsigned char motionSendState,unsigned char power);
//void deepSleepNoRam();
void pirInit();
unsigned char resetDetect();
unsigned char  rdOtaDetect(otaTypeDef *ota);
void otaConcol(otaTypeDef *ota);
void rd_deep_sleep_setup();
void provFailProc();
int is_clock_time_done(unsigned int *time,unsigned int period);
