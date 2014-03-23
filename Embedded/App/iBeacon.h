/***************************************************************************************************
 *   Copyright (C) 2014-2020 LinkEssence Inc.
 *   All rights reserved.
 *
 *   Filename:      iBeacon.h
 *
 *   Author:        Tao Zhang
 *   Verifier:
 *
 *   Description:
 *
 *   This file contains the definitions and prototypes of the iBeacon BLE Broadcaster application
 *   for use with the Texas Instruments CC2540 / CC2541 Bluetooth Low Energy Protocol Stack.
 *
 *   ----------------------------------------------------------------------------------------------
 *   Version                Date                   Modifier                 Description
 *   ----------------------------------------------------------------------------------------------
 *   1.0.0                  March 23, 2014         Tao Zhang                Initial edition
 *
 **************************************************************************************************/

#ifndef __IBEACON_H_
#define __IBEACON_H_

#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// iBeacon Task Events
#define IBEACON_START_DEVICE_EVT                              0x0001
#define IBEACON_PERIODIC_EVT                                  0x0002
#define IBEACON_ADV_IN_CONNECTION_EVT                         0x0004  
/*
 * Task Initialization for the BLE Broadcaster Application
 */
extern void iBeacon_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Broadcaster Application
 */
extern uint16 iBeacon_ProcessEvent( uint8 task_id, uint16 events );
  
#ifdef __cplusplus
}
#endif
  
#endif /* #ifndef __IBEACON_H_ */