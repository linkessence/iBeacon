/***************************************************************************************************
 *   Copyright (C) 2014-2020 LinkEssence Inc.
 *   All rights reserved.
 *
 *   Filename:      BNotification.c
 *
 *   Author:        Tao Zhang
 *   Verifier:
 *
 *   Description:
 *
 *   This file contains the iBeacon ANCS Notification application for use with the 
 *   Texas Instruments CC2540 / CC2541 Bluetooth Low Energy Protocol Stack.
 *
 *   ----------------------------------------------------------------------------------------------
 *   Version                Date                   Modifier                 Description
 *   ----------------------------------------------------------------------------------------------
 *   1.0.0                  May 07, 2014           Tao Zhang                Initial edition
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
#include "gapgattserver.h"
#include "gapbondmgr.h"

#include "gatt.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"

#include "linkdb.h"

#include "peripheralBroadcaster.h"


#include "BNotification.h"

/*********************************************************************
 * CONSTANTS
 */

// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             160

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Delay to begin discovery from start of connection in ms
#define DEFAULT_DISCOVERY_DELAY               100



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

// GAP Profile - Name attribute for SCAN RSP data
static uint8 BNotification_App_ScanData[] =
{
  // local name
  12,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'B',
  'L',
  'E',
  ' ',
  'j',
  'e',
  'w',
  'e',
  'l',
  'r',
  'y'
};

// GAP Profile -- Application advertisement data
static uint8 BNotification_App_AdvData[] = 
{
  // flags
  0x02,
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  // service solicitation
  0x05,
  GAP_ADTYPE_SERVICES_LIST_16BIT,
  LO_UINT16(ALERT_NOTIF_SERV_UUID),
  HI_UINT16(ALERT_NOTIF_SERV_UUID),
  LO_UINT16(BATT_SERV_UUID),
  HI_UINT16(BATT_SERV_UUID)
};


// Device name attribute value
static uint8 BNotification_App_DeviceName[GAP_DEVICE_NAME_LEN] = "BLE jewelry";

// Task ID
static uint8 BNotification_TaskID = 0;

// Connect handle
static uint16 BNotification_ConnHandle = 0;

// Bonded peer address
static uint8 BNotification_BondedAddr[B_ADDR_LEN];

static bool BNotification_isBonded = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void BNotification_App_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void passcodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void pairStateCB( uint16 connHandle, uint8 state, uint8 status );

#if defined( CC2540_MINIDK )
static void BNotification_App_HandleKeys( uint8 shift, uint8 keys );
#endif



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t BNotification_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t BNotification_BondMgrCBs =
{
  passcodeCB,                     // Passcode callback (not used by application)
  pairStateCB                     // Pairing / Bonding state Callback (not used by application)
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BNotification_App_Init
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
void BNotification_App_Init( uint8 task_id )
{
  BNotification_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    // Do not start device advertising upon initialization, until a button is
    // pushed
    uint8 initial_advertising_enable = TRUE;
    uint16 gapRole_AdvertOffTime = 640; // (units of 625us, 640=400ms)
    uint8 updateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 minInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 maxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 slaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
      
    uint8 advType = GAP_ADTYPE_ADV_IND;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &updateRequest );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &minInterval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &maxInterval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &slaveLatency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &connTimeout );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( BNotification_App_ScanData ), BNotification_App_ScanData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( BNotification_App_AdvData ), BNotification_App_AdvData );

    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
  }

  // Setup GAP
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, BNotification_App_DeviceName );
  
  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }
  
  // Initialize GATT Client
  GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( BNotification_TaskID );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
 
#if defined( CC2540_MINIDK )
 
  // Register for all key events - This app will handle all key events
  RegisterForKeys( BNotification_TaskID );
  
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
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_ON );
  
#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)  

  HalLcdWriteString( "BLE Notification", HAL_LCD_LINE_1 );
  
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)  
  
  // Setup a delayed profile startup
  osal_set_event( BNotification_TaskID, APP_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      BNotification_App_ProcessEvent
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
uint16 BNotification_App_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( BNotification_TaskID )) != NULL )
    {
      BNotification_App_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & APP_START_DEVICE_EVT )
  {
    // Start the Device
    GAPRole_StartDevice( &BNotification_PeripheralCBs );
    
    // Start Bond Manager
    GAPBondMgr_Register( &BNotification_BondMgrCBs );
    
    // return unprocessed events
    return ( events ^ APP_START_DEVICE_EVT );
  }
  
  if ( events & APP_START_DISCOVERY_EVT )
  {
    // Start discovery
    BNotification_StartDiscovery();
    
    // return unprocessed events
    return ( events ^ APP_START_DISCOVERY_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      BNotification_App_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void BNotification_App_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
  case KEY_CHANGE:
    BNotification_App_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
    break;
  #endif // CC2540_MINIDK
      
  case GATT_MSG_EVENT:
    break;
  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      BNotification_HandleKeys
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
static void BNotification_App_HandleKeys( uint8 shift, uint8 keys )
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
  static gaprole_States_t oldState = GAPROLE_INIT;
  
  if(oldState == GAPROLE_CONNECTED && newState != GAPROLE_CONNECTED)
  {
    // Disconnected, advertising
    uint16 advInt;
    uint16 advDuration;
    
    if(newState == GAPROLE_WAITING_AFTER_TIMEOUT)
    {
      advInt = DEFAULT_FAST_ADV_INTERVAL;
      advDuration = DEFAULT_FAST_ADV_DURATION;
    }
    else
    {
      advInt = DEFAULT_SLOW_ADV_INTERVAL;
      advDuration = 0; // continous advertisement
    }
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, advDuration );
  }

  switch ( newState )
  {
    case GAPROLE_STARTED:
    {
      uint16 advInt = DEFAULT_SLOW_ADV_INTERVAL;
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
      GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, 0 );
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
      linkDBItem_t  *pItem;
      
      // Get the connection handle
      GAPRole_GetParameter( GAPROLE_CONNHANDLE, &BNotification_ConnHandle);
      
      // Get peer bd address
      if ( (pItem = linkDB_Find( BNotification_ConnHandle )) != NULL)
      {
        // Connected
        
        // If connected to device without bond do service discovery
        if ( !BNotification_isBonded )
        {
            osal_start_timerEx( BNotification_TaskID, APP_START_DISCOVERY_EVT, DEFAULT_DISCOVERY_DELAY );
        }
        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)            
          
        #ifdef CC2540_MINIDK
          // Visual feedback (GREEN) that we are connected.
          HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
          HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
        #endif //CC2540_MINIDK
      }
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
    }        
    break; 
  }
  
  oldState = newState;
}

/*********************************************************************
 * @fn      passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void passcodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                        uint8 uiInputs, uint8 uiOutputs )
{

}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void pairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      linkDBItem_t  *pItem;
      
      if ( (pItem = linkDB_Find( BNotification_ConnHandle )) != NULL )
      {
      }
      
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    // Successfully bonded
  }
}


/*********************************************************************
*********************************************************************/
