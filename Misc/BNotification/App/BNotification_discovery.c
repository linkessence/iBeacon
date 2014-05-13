/***************************************************************************************************
*   Copyright (C) 2014-2020 LinkEssence Inc.
*   All rights reserved.
*
*   Filename:      BNotification_discovery.c
*
*   Author:        Tao Zhang
*   Verifier:
*
*   Description:
*
*   This file contains the ANCS Notification service discovery routines for use with the 
*   Texas Instruments CC2540 / CC2541 Bluetooth Low Energy Protocol Stack.
*
*   ----------------------------------------------------------------------------------------------
*   Version                Date                   Modifier                 Description
*   ----------------------------------------------------------------------------------------------
*   1.0.0                  May 08, 2014           Tao Zhang                Initial edition
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
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"

#include "linkdb.h"

#include "peripheralBroadcaster.h"


#include "BNotification.h"


/*********************************************************************
* DEFINITIONS
*/

// Length of Characteristic declaration + handle with 16 bit UUID
#define CHAR_DESC_HDL_UUID16_LEN        7
#define CHAR_DESC_HDL_UUID128_LEN       21

#define ANCS_NOTIF_CHAR_UUID    0x1DBD
#define ANCS_CONTROL_CHAR_UUID  0xD9D9
#define ANCS_DATA_CHAR_UUID     0x7BFB


/*********************************************************************
* EXTERNAL VARIABLES
*/

// Attribute handle cache
uint16 BNotification_HdlCache[HDL_CACHE_LEN];

/*********************************************************************
* LOCAL VARIABLES
*/

// Attribute handles used during discovery
static uint16 BNotification_SvcStartHdl;
static uint16 BNotification_SvcEndHdl;
static uint8 BNotification_EndHdlIdx;

/*********************************************************************
* LOCAL FUNCTIONS
*/
static uint8 BNotification_App_DiscANCS( uint8 state, gattMsgEvent_t *pMsg );
static uint8 BNotification_App_DiscAlertNtf( uint8 state, gattMsgEvent_t *pMsg );
static uint8 BNotification_App_DiscBatt( uint8 state, gattMsgEvent_t *pMsg );

/*********************************************************************
* @fn      BNotification_StartDiscovery
*
* @brief   Discovery services
*
* @return  none
*/
uint8 BNotification_StartDiscovery ()
{
  // Clear handle cache
  osal_memset( BNotification_HdlCache, 0, sizeof(BNotification_HdlCache) );
  
  // Start discovery with first service
  return BNotification_App_DiscGattMsg( DISC_ANCS_START, NULL );
}

/*********************************************************************
* @fn      BNotification_App_DiscGattMsg
*
* @brief   Handle GATT messages for characteristic discovery. 
*
* @param   state - Discovery state.
* @param   pMsg - GATT message.
*
* @return  New discovery state.
*/
uint8 BNotification_App_DiscGattMsg( uint8 state, gattMsgEvent_t *pMsg )
{
  // Execute discovery function for service
  do
  {
    switch ( state & 0xF0 )
    {
    case DISC_ANCS_START:
      state = BNotification_App_DiscANCS( state, pMsg );
      if ( state == DISC_FAILED || state == DISC_IDLE )
      {
        state = DISC_ALERT_NTF_START;
      }
      break;
    case DISC_ALERT_NTF_START:
      state = BNotification_App_DiscAlertNtf( state, pMsg );
      if ( state == DISC_FAILED || state == DISC_IDLE )
      {
        state = DISC_BATT_START;
      }
      break;
    case DISC_BATT_START:
      state = BNotification_App_DiscBatt( state, pMsg );
      if ( state == DISC_FAILED )
      {
        state = DISC_IDLE;
      }
    default:
      break;
    }
  } while ( (state != 0) && ((state & 0x0F) == 0) );
  
  return state;
}

/*********************************************************************
* @fn      BNotification_App_DiscANCS()
*
* @brief   Alert notification service and characteristic discovery. 
*
* @param   state - Discovery state.
* @param   pMsg - GATT message.
*
* @return  New discovery state.
*/
static uint8 BNotification_App_DiscANCS( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
  case DISC_ANCS_START:
    {
      uint8 uuid[ATT_UUID_SIZE] = {0xd0, 0x00, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 
        0xa4, 0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x79};
      
      // Initialize service discovery variables
      BNotification_SvcStartHdl = BNotification_SvcEndHdl = 0;
      BNotification_EndHdlIdx = 0;
      
      // Discover service by UUID
      GATT_DiscPrimaryServiceByUUID( BNotification_ConnHandle, uuid,
                                    ATT_UUID_SIZE, BNotification_TaskID );  
      
      
      newState = DISC_ANCS_SVC;
      
    }
    break;
    
  case DISC_ANCS_SVC:
    {
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        BNotification_SvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        BNotification_SvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( BNotification_SvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( BNotification_ConnHandle, BNotification_SvcStartHdl,
                            BNotification_SvcEndHdl, BNotification_TaskID );
          
          newState = DISC_ANCS_CHAR;
        }
        else
        {
          // Service not found
          newState = DISC_FAILED;
        }
      }
    }    
    break;
    
  case DISC_ANCS_CHAR:
    {
      uint8   i;
      uint8   *p;
      uint16  handle;
      uint16  uuid;
      
      // Characteristics found
      if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
          pMsg->msg.readByTypeRsp.numPairs > 0 && 
            pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID128_LEN )
      {
        // For each characteristic declaration
        p = pMsg->msg.readByTypeRsp.dataList;
        for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
        {
          // Parse characteristic declaration
          handle = BUILD_UINT16(p[3], p[4]);
          uuid = BUILD_UINT16(p[5], p[6]);
          
          // If looking for end handle
          if ( BNotification_EndHdlIdx != 0 )
          {
            // End handle is one less than handle of characteristic declaration
            BNotification_HdlCache[BNotification_EndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
            BNotification_EndHdlIdx = 0;
          }
          
          
          // If UUID is of interest, store handle
          switch ( uuid )
          {
          case ANCS_NOTIF_CHAR_UUID:
            BNotification_HdlCache[HDL_ANCS_NTF_START] = handle;
            BNotification_EndHdlIdx = HDL_ANCS_NTF_END;
            break;
            
            
          default:
            break;
          }
          
          p += CHAR_DESC_HDL_UUID128_LEN;
        }
        
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // Special case of end handle at end of service
        if ( BNotification_EndHdlIdx != 0 )
        {
          BNotification_HdlCache[BNotification_EndHdlIdx] = BNotification_SvcEndHdl;
          BNotification_EndHdlIdx = 0;
        }
        // If didn't find mandatory characteristic
        if ( BNotification_HdlCache[HDL_ANCS_NTF_START] == 0 )
        {
          newState = DISC_FAILED;
        }
        else if ( BNotification_HdlCache[HDL_ANCS_NTF_START] <
                 BNotification_HdlCache[HDL_ANCS_NTF_END] )
        {
          // Discover characteristic descriptors
          GATT_DiscAllCharDescs( BNotification_ConnHandle,
                                BNotification_HdlCache[HDL_ANCS_NTF_START] + 1,
                                BNotification_HdlCache[HDL_ANCS_NTF_END],
                                BNotification_TaskID );
          
          newState = DISC_ANCS_CCCD;
        }
        else
        {
          newState = DISC_IDLE;
        }
      }
    }
    break;
    
  case DISC_ANCS_CCCD:
    {
      uint8 i;
      
      // Characteristic descriptors found
      if ( pMsg->method == ATT_FIND_INFO_RSP &&
          pMsg->msg.findInfoRsp.numInfo > 0 && 
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
      {
        // For each handle/uuid pair
        for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
        {
          // Look for CCCD
          if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
              (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
               HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
          {
            // CCCD found
            BNotification_HdlCache[HDL_ANCS_NTF_CCCD] =
              pMsg->msg.findInfoRsp.info.btPair[i].handle;
            
            break;
          }
        }
      }
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        newState = DISC_IDLE;
      }
    }
    break;
    
  default:
    break;
  }
  
  return newState;  
}

/*********************************************************************
* @fn      BNotification_App_DiscAlertNtf()
*
* @brief   Alert notification service and characteristic discovery. 
*
* @param   state - Discovery state.
* @param   pMsg - GATT message.
*
* @return  New discovery state.
*/
static uint8 BNotification_App_DiscAlertNtf( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
  case DISC_ALERT_NTF_START:  
    {
      uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(ALERT_NOTIF_SERV_UUID),
      HI_UINT16(ALERT_NOTIF_SERV_UUID) };
      
      // Initialize service discovery variables
      BNotification_SvcStartHdl = BNotification_SvcEndHdl = 0;
      BNotification_EndHdlIdx = 0;
      
      // Discover service by UUID
      GATT_DiscPrimaryServiceByUUID( BNotification_ConnHandle, uuid,
                                    ATT_BT_UUID_SIZE, BNotification_TaskID );      
      
      newState = DISC_ALERT_NTF_SVC;
    } 
    break;
    
  case DISC_ALERT_NTF_SVC:
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      BNotification_SvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      BNotification_SvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
          pMsg->hdr.status == bleProcedureComplete ) ||
        ( pMsg->method == ATT_ERROR_RSP ) )
    {
      // If service found
      if ( BNotification_SvcStartHdl != 0 )
      {
        // Discover all characteristics
        GATT_DiscAllChars( BNotification_ConnHandle, BNotification_SvcStartHdl,
                          BNotification_SvcEndHdl, BNotification_TaskID );
        
        newState = DISC_ALERT_NTF_CHAR;
      }
      else
      {
        // Service not found
        newState = DISC_FAILED;
      }
    }    
    break;
    
  case DISC_ALERT_NTF_CHAR:
    {
      uint8   i;
      uint8   *p;
      uint16  handle;
      uint16  uuid;
      
      // Characteristics found
      if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
          pMsg->msg.readByTypeRsp.numPairs > 0 && 
            pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN )
      {
        // For each characteristic declaration
        p = pMsg->msg.readByTypeRsp.dataList;
        for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
        {
          // Parse characteristic declaration
          handle = BUILD_UINT16(p[3], p[4]);
          uuid = BUILD_UINT16(p[5], p[6]);
          
          // If looking for end handle
          if ( BNotification_EndHdlIdx != 0 )
          {
            // End handle is one less than handle of characteristic declaration
            BNotification_HdlCache[BNotification_EndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
            
            BNotification_EndHdlIdx = 0;
          }
          
          // If UUID is of interest, store handle
          switch ( uuid )
          {
          case ALERT_NOTIF_CTRL_PT_UUID:
            BNotification_HdlCache[HDL_ALERT_NTF_CTRL] = handle;
            break;
            
          case UNREAD_ALERT_STATUS_UUID:
            BNotification_HdlCache[HDL_ALERT_NTF_UNREAD_START] = handle;
            BNotification_EndHdlIdx = HDL_ALERT_NTF_UNREAD_END;
            break;
            
          case NEW_ALERT_UUID:
            BNotification_HdlCache[HDL_ALERT_NTF_NEW_START] = handle;
            BNotification_EndHdlIdx = HDL_ALERT_NTF_NEW_END;
            break;
            
          case SUP_NEW_ALERT_CAT_UUID:
            BNotification_HdlCache[HDL_ALERT_NTF_NEW_CAT] = handle;
            break;
            
          case SUP_UNREAD_ALERT_CAT_UUID:
            BNotification_HdlCache[HDL_ALERT_NTF_UNREAD_CAT] = handle;
            break;
            
          default:
            break;
          }
          
          p += CHAR_DESC_HDL_UUID16_LEN;
        }
        
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // Special case of end handle at end of service
        if ( BNotification_EndHdlIdx != 0 )
        {
          BNotification_HdlCache[BNotification_EndHdlIdx] = BNotification_SvcEndHdl;
          BNotification_EndHdlIdx = 0;
        }
        
        // If didn't find new alert characteristic
        if ( BNotification_HdlCache[HDL_ALERT_NTF_NEW_START] == 0 )
        {
          newState = DISC_FAILED;
        }
        else if ( BNotification_HdlCache[HDL_ALERT_NTF_NEW_START] <
                 BNotification_HdlCache[HDL_ALERT_NTF_NEW_END] )
        {
          // Discover incoming alert characteristic descriptors
          GATT_DiscAllCharDescs( BNotification_ConnHandle,
                                BNotification_HdlCache[HDL_ALERT_NTF_NEW_START] + 1,
                                BNotification_HdlCache[HDL_ALERT_NTF_NEW_END],
                                BNotification_TaskID );
          
          newState = DISC_ALERT_NTF_NEW_CCCD;
        }
        else
        {
          // Missing required characteristic descriptor
          BNotification_HdlCache[HDL_ALERT_NTF_NEW_START] = 0;
          newState = DISC_FAILED;
        }
      }
    }      
    break;
    
  case DISC_ALERT_NTF_NEW_CCCD:
    {
      uint8 i;
      
      // Characteristic descriptors found
      if ( pMsg->method == ATT_FIND_INFO_RSP &&
          pMsg->msg.findInfoRsp.numInfo > 0 && 
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
      {
        // For each handle/uuid pair
        for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
        {
          // Look for CCCD
          if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
              (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
               HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
          {
            // CCCD found
            BNotification_HdlCache[HDL_ALERT_NTF_NEW_CCCD] =
              pMsg->msg.findInfoRsp.info.btPair[i].handle;
            
            break;
          }
        }
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If CCCD found
        if ( BNotification_HdlCache[HDL_ALERT_NTF_NEW_CCCD] != 0 )
        {
          // Should we look for unread category status CCCD
          if ( BNotification_HdlCache[HDL_ALERT_NTF_UNREAD_START] <
              BNotification_HdlCache[HDL_ALERT_NTF_UNREAD_END] )
          {
            // Discover unread category status characteristic descriptors
            GATT_DiscAllCharDescs( BNotification_ConnHandle,
                                  BNotification_HdlCache[HDL_ALERT_NTF_UNREAD_START] + 1,
                                  BNotification_HdlCache[HDL_ALERT_NTF_UNREAD_END],
                                  BNotification_TaskID );
            
            newState = DISC_ALERT_NTF_UNREAD_CCCD;
          }
          else
          {
            // Done
            newState = DISC_IDLE;
          }
        }
        else
        {
          // Missing required characteristic descriptor
          BNotification_HdlCache[HDL_ALERT_NTF_NEW_START] = 0;
          newState = DISC_FAILED;
        }          
      }
    }
    break;
    
  case DISC_ALERT_NTF_UNREAD_CCCD:
    {
      uint8 i;
      
      // Characteristic descriptors found
      if ( pMsg->method == ATT_FIND_INFO_RSP &&
          pMsg->msg.findInfoRsp.numInfo > 0 && 
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
      {
        // For each handle/uuid pair
        for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
        {
          // Look for CCCD
          if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
              (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
               HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
          {
            // CCCD found
            BNotification_HdlCache[HDL_ALERT_NTF_UNREAD_CCCD] =
              pMsg->msg.findInfoRsp.info.btPair[i].handle;
            
            break;
          }
        }
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        newState = DISC_IDLE;
      }
    }
    break;
    
  default:
    break;
  }
  
  return newState;
}

/*********************************************************************
* @fn      BNotification_App_DiscBatt()
*
* @brief   Battery service and characteristic discovery. 
*
* @param   state - Discovery state.
* @param   pMsg - GATT message.
*
* @return  New discovery state.
*/
static uint8 BNotification_App_DiscBatt( uint8 state, gattMsgEvent_t *pMsg )
{
  uint8 newState = state;
  
  switch ( state )
  {
  case DISC_BATT_START:  
    {
      uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(BATT_SERV_UUID),
      HI_UINT16(BATT_SERV_UUID) };
      
      // Initialize service discovery variables
      BNotification_SvcStartHdl = BNotification_SvcEndHdl = 0;
      BNotification_EndHdlIdx = 0;
      
      // Discover service by UUID
      GATT_DiscPrimaryServiceByUUID( BNotification_ConnHandle, uuid,
                                    ATT_BT_UUID_SIZE, BNotification_TaskID );      
      
      newState = DISC_BATT_SVC;
    } 
    break;
    
  case DISC_BATT_SVC:
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      BNotification_SvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      BNotification_SvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
          pMsg->hdr.status == bleProcedureComplete ) ||
        ( pMsg->method == ATT_ERROR_RSP ) )
    {
      // If service found
      if ( BNotification_SvcStartHdl != 0 )
      {
        // Discover all characteristics
        GATT_DiscAllChars( BNotification_ConnHandle, BNotification_SvcStartHdl,
                          BNotification_SvcEndHdl, BNotification_TaskID );
        
        newState = DISC_BATT_CHAR;
      }
      else
      {
        // Service not found
        newState = DISC_FAILED;
      }
    }    
    break;
    
  case DISC_BATT_CHAR:
    {
      uint8   i;
      uint8   *p;
      uint16  handle;
      uint16  uuid;
      
      // Characteristics found
      if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
          pMsg->msg.readByTypeRsp.numPairs > 0 && 
            pMsg->msg.readByTypeRsp.len == CHAR_DESC_HDL_UUID16_LEN )
      {
        // For each characteristic declaration
        p = pMsg->msg.readByTypeRsp.dataList;
        for ( i = pMsg->msg.readByTypeRsp.numPairs; i > 0; i-- )
        {
          // Parse characteristic declaration
          handle = BUILD_UINT16(p[3], p[4]);
          uuid = BUILD_UINT16(p[5], p[6]);
          
          // If looking for end handle
          if ( BNotification_EndHdlIdx != 0 )
          {
            // End handle is one less than handle of characteristic declaration
            BNotification_HdlCache[BNotification_EndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
            
            BNotification_EndHdlIdx = 0;
          }
          
          // If UUID is of interest, store handle
          switch ( uuid )
          {
          case BATT_LEVEL_UUID:
            BNotification_HdlCache[HDL_BATT_LEVEL_START] = handle;
            BNotification_EndHdlIdx = HDL_BATT_LEVEL_END;
            break;
            
          default:
            break;
          }
          
          p += CHAR_DESC_HDL_UUID16_LEN;
        }
        
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_READ_BY_TYPE_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // Special case of end handle at end of service
        if ( BNotification_EndHdlIdx != 0 )
        {
          BNotification_HdlCache[BNotification_EndHdlIdx] = BNotification_SvcEndHdl;
          BNotification_EndHdlIdx = 0;
        }
        
        // If didn't find mandatory characteristic
        if ( BNotification_HdlCache[HDL_BATT_LEVEL_START] == 0 )
        {
          newState = DISC_FAILED;
        }
        else if ( BNotification_HdlCache[HDL_BATT_LEVEL_START] <
                 BNotification_HdlCache[HDL_BATT_LEVEL_END] )
        {
          // Discover characteristic descriptors
          GATT_DiscAllCharDescs( BNotification_ConnHandle,
                                BNotification_HdlCache[HDL_BATT_LEVEL_START] + 1,
                                BNotification_HdlCache[HDL_BATT_LEVEL_END],
                                BNotification_TaskID );
          
          newState = DISC_BATT_LVL_CCCD;
        }
        else
        {
          newState = DISC_IDLE;
        }
      }
    }      
    break;
    
  case DISC_BATT_LVL_CCCD:
    {
      uint8 i;
      
      // Characteristic descriptors found
      if ( pMsg->method == ATT_FIND_INFO_RSP &&
          pMsg->msg.findInfoRsp.numInfo > 0 && 
            pMsg->msg.findInfoRsp.format == ATT_HANDLE_BT_UUID_TYPE )
      {
        // For each handle/uuid pair
        for ( i = 0; i < pMsg->msg.findInfoRsp.numInfo; i++ )
        {
          // Look for CCCD
          if ( (pMsg->msg.findInfoRsp.info.btPair[i].uuid[0] ==
                LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) &&
              (pMsg->msg.findInfoRsp.info.btPair[i].uuid[1] ==
               HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID)) )
          {
            // CCCD found
            BNotification_HdlCache[HDL_BATT_LEVEL_CCCD] =
              pMsg->msg.findInfoRsp.info.btPair[i].handle;
            
            break;
          }
        }
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_INFO_RSP  && 
            pMsg->hdr.status == bleProcedureComplete ) ||
          ( pMsg->method == ATT_ERROR_RSP ) )
      {
        newState = DISC_IDLE;
      }
    }
    break;
    
  default:
    break;
  }
  
  return newState;
}
