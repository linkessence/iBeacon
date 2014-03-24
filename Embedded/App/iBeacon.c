/***************************************************************************************************
 *   Copyright (C) 2014-2020 LinkEssence Inc.
 *   All rights reserved.
 *
 *   Filename:      iBeacon.c
 *
 *   Author:        Tao Zhang
 *   Verifier:
 *
 *   Description:
 *
 *   This file contains the iBeacon BLE Broadcaster application for use with the 
 *   Texas Instruments CC2540 / CC2541 Bluetooth Low Energy Protocol Stack.
 *
 *   ----------------------------------------------------------------------------------------------
 *   Version                Date                   Modifier                 Description
 *   ----------------------------------------------------------------------------------------------
 *   1.0.0                  March 23, 2014         Tao Zhang                Initial edition
 *
 **************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "hci.h"
#include "gap.h"
#include "gapbondmgr.h"

#include "peripheralBroadcaster.h"


#include "iBeacon.h"

/*********************************************************************
 * CONSTANTS
 */

// What is the advertising interval when device is discoverable (units of 625us, 1600=1000ms)
#define DEFAULT_ADVERTISING_INTERVAL          1600

// Wait time for advertisement in connection
#define ADV_IN_CONN_WAIT                      500 // delay 500 ms

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         10

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 iBeacon_TaskID;   // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x08,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'i',
  'B',
  'e',
  'a',
  'c',
  'o',
  'n',

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm  
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] = 
{ 
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02, // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL|GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  0x1A, // length of this data (26 Bytes )
  GAP_ADTYPE_MANUFACTURER_SPECIFIC, 
  /*Apple Pre-Amble*/
  0x4C,
  0x00,
  0x02,
  0x15,

  /*Device UUID (16 Bytes)*/
  0x02, 0xA0, 0xCF, 0x35, 0xE7, 0x5C, 0x4C, 0x48, 0xBF, 0x9D, 0x56, 0x35, 0xFF, 0x67, 0x46, 0xA9,

  /*Major Value (2 Bytes)*/
  0x00, 0x00,

  /*Minor Value (2 Bytes)*/
  0x00, 0x00,

  /*Measured Power: -53dB*/
  0xC5
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void iBeacon_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );

#if defined( CC2540_MINIDK )
static void iBeacon_HandleKeys( uint8 shift, uint8 keys );
#endif

#if (defined HAL_LCD) && (HAL_LCD == TRUE) 
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE) 

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t iBeacon_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t iBeacon_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      iBeacon_Init
 *
 * @brief   Initialization function for the Simple BLE Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void iBeacon_Init( uint8 task_id )
{
  iBeacon_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    // Do not start device advertising upon initialization, until a button is
    // pushed
    uint8 initial_advertising_enable = FALSE;

    uint16 gapRole_AdvertOffTime = 6400; // (units of 625us, 6400=4000ms)
      
    //uint8 advType = GAP_ADTYPE_ADV_NONCONN_IND;   // use non-connectable advertisements
    uint8 advType = GAP_ADTYPE_ADV_IND;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
  }

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
#if defined( CC2540_MINIDK )
 
  // Register for all key events - This app will handle all key events
  RegisterForKeys( iBeacon_TaskID );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.
  
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output
  
  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low  
  
#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)  

  HalLcdWriteString( "BLE iBeacon", HAL_LCD_LINE_1 );
  
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)  
  
  // Setup a delayed profile startup
  osal_set_event( iBeacon_TaskID, IBEACON_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      iBeacon_ProcessEvent
 *
 * @brief   Simple BLE Broadcaster Application Task event processor. This
 *          function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 iBeacon_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( iBeacon_TaskID )) != NULL )
    {
      iBeacon_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & IBEACON_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &iBeacon_PeripheralCBs );
    
    // Start Bond Manager
    VOID GAPBondMgr_Register( &iBeacon_BondMgrCBs );
    
    return ( events ^ IBEACON_START_DEVICE_EVT );
  }
  
  if ( events & IBEACON_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      iBeacon_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void iBeacon_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      iBeacon_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // CC2540_MINIDK
      
  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      iBeacon_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void iBeacon_HandleKeys( uint8 shift, uint8 keys )
{
  VOID shift;  // Intentionally unreferenced parameter

  if ( (keys & HAL_KEY_SW_2) != 0 )
  {
    // ressing the right key should toggle advertising on and off
    uint8 current_adv_enabled_status;
    uint8 new_adv_enabled_status;
    
    //Find the current GAP advertisement status
    GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
    
    if( current_adv_enabled_status == FALSE )
    {
      new_adv_enabled_status = TRUE;
    }
    else
    {
      new_adv_enabled_status = FALSE;
    }
    
    //change the GAP advertisement status to opposite of current status
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
  }
}
#endif // CC2540_MINIDK

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {    
        uint8 ownAddress[B_ADDR_LEN];
        
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
    
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address 
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)    
        #ifdef CC2540_MINIDK
          // Visual feedback that we have initialized
          HalLedSet( HAL_LED_1 | HAL_LED_2, HAL_LED_MODE_ON );
        #endif //CC2540_MINIDK
      }
      break;
      
    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)            
          
        #ifdef CC2540_MINIDK
          // Visual feedback (RED) that we are advertising.
          HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
          HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
        #endif //CC2540_MINIDK
      }
      break;

    case GAPROLE_CONNECTED:
      {
        // Enable advertisement in connection
        osal_start_timerEx( iBeacon_TaskID, IBEACON_ADV_IN_CONNECTION_EVT, ADV_IN_CONN_WAIT );
        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)            
          
        #ifdef CC2540_MINIDK
          // Visual feedback (GREEN) that we are connected.
          HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
          HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
        #endif //CC2540_MINIDK
      }
      break;
      
    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Waiting",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        #ifdef CC2540_MINIDK
          HalLedSet( HAL_LED_1 | HAL_LED_2, HAL_LED_MODE_OFF );
        #endif //CC2540_MINIDK
      }
      break;          

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)            
        #ifdef CC2540_MINIDK
          // Visual feedback that we have an error
          HalLedSet( HAL_LED_1 | HAL_LED_2, HAL_LED_MODE_ON );
        #endif //CC2540_MINIDK
      }
      break;      
      
    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)     
      }        
      break; 
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE) 

/*********************************************************************
*********************************************************************/
