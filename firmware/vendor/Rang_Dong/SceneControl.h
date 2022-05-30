/*
 * SceneControl.h
 *
 *  Created on: May 29, 2022
 *      Author: PC5
 */

#ifndef SCENECONTROL_H_
#define SCENECONTROL_H_


#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"


void checkMotion(unsigned char motionState);
void RD_LightSensorControlSence(u16 Sensor_Lux,unsigned char pirSceneState);
void RD_mesh_loop();
void RD_Send_Pir_motion(u8 PIR_Condition,u16 SceneID,unsigned char *buff);
#endif /* SCENECONTROL_H_ */
