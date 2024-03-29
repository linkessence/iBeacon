/***************************************************************************************************
 *   Copyright (C) 2014-2020 LinkEssence Inc.
 *   All rights reserved.
 *
 *   Filename:      BNotification.h
 *
 *   Author:        Tao Zhang
 *   Verifier:
 *
 *   Description:
 *
 *   This file contains the definitions and prototypes of the iBeacon ANCS Notification application
 *   for use with the Texas Instruments CC2540 / CC2541 Bluetooth Low Energy Protocol Stack.
 *
 *   ----------------------------------------------------------------------------------------------
 *   Version                Date                   Modifier                 Description
 *   ----------------------------------------------------------------------------------------------
 *   1.0.0                  May 7, 2014         Tao Zhang                Initial edition
 *
 **************************************************************************************************/

#ifndef __BNOTIFICATION_H_
#define __BNOTIFICATION_H_

#include <hal_types.h>
#include <gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

// BNotification Application Task Events
#define APP_START_DEVICE_EVT                              0x0001
#define APP_START_DISCOVERY_EVT                           0x0002
  
/*********************************************************************
 * EXTERNAL VARIABLES
 */
// Task ID
extern uint8 BNotification_TaskID;

// Connect handle
extern uint16 BNotification_ConnHandle;

/*
 * Task Initialization for the BLE Notification Application
 */
extern void BNotification_App_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Notification Application
 */
extern uint16 BNotification_App_ProcessEvent( uint8 task_id, uint16 events );

/*
 * Start the service discovery, and returns the next discovery state.
 */
extern uint8 BNotification_StartDiscovery ();

/*
 * Handle GATT messages for characteristic discovery.
 */
extern uint8 BNotification_App_DiscGattMsg( uint8 state, gattMsgEvent_t *pMsg );

// Service discovery states
enum
{
  DISC_IDLE = 0x00,                       // Idle state
  
  DISC_ANCS_START = 0x10,                 // Apple notification center service
  DISC_ANCS_SVC,                          // Discover service
  DISC_ANCS_CHAR,                         // Discover all characteristics
  DISC_ANCS_CCCD,                         // Discover ANCS CCCD
  
  DISC_ALERT_NTF_START = 0x50,            // Alert notification service
  DISC_ALERT_NTF_SVC,                     // Discover service
  DISC_ALERT_NTF_CHAR,                    // Discover all characteristics
  DISC_ALERT_NTF_NEW_CCCD,                // Discover new alert CCCD
  DISC_ALERT_NTF_UNREAD_CCCD,             // Discover unread alert status CCCD

  DISC_BATT_START = 0x60,                 // Battery service
  DISC_BATT_SVC,                          // Discover service
  DISC_BATT_CHAR,                         // Discover all characteristics
  DISC_BATT_LVL_CCCD,                     // Discover battery level CCCD

  DISC_FAILED = 0xFF                      // Discovery failed
};

// BNotification App handle cache indexes
enum
{
  HDL_ANCS_NTF_START,                     // ANCS notification start handle
  HDL_ANCS_NTF_END,                       // ANCS notification end handle
  HDL_ANCS_NTF_CCCD,                      // ANCS notification CCCD
  
  HDL_ALERT_NTF_NEW_START,                // New alert start handle
  HDL_ALERT_NTF_NEW_END,                  // New alert end handle
  HDL_ALERT_NTF_UNREAD_START,             // Unread alert status start handle
  HDL_ALERT_NTF_UNREAD_END,               // Unread alert status end handle
  HDL_ALERT_NTF_CTRL,                     // Alert notification control point
  HDL_ALERT_NTF_NEW_CAT,                  // Supported New Alert Category
  HDL_ALERT_NTF_UNREAD_CAT,               // Supported Unread Alert Category
  HDL_ALERT_NTF_NEW_CCCD,                 // New alert CCCD
  HDL_ALERT_NTF_UNREAD_CCCD,              // Alert unread alert status CCCD

  HDL_BATT_LEVEL_START,                   // Battery level start handle
  HDL_BATT_LEVEL_END,                     // Battery level end handle
  HDL_BATT_LEVEL_CCCD,                    // Battery level CCCD

  HDL_CACHE_LEN
};

// Attribute handle cache
extern uint16 BNotification_HdlCache[HDL_CACHE_LEN];


#ifdef __cplusplus
}
#endif
  
#endif /* #ifndef __BNOTIFICATION_H_ */