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
#include "gattservapp.h"
#include "gatt_profile_uuid.h"

#include "linkdb.h"

#include "peripheralBroadcaster.h"


#include "BNotification.h"

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
      uint8 uuid[ATT_UUID_SIZE] = {};
      
  
  return 0;
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
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_ALERT_NTF_SVC;
      } 
      break;

    case DISC_ALERT_NTF_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
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
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case ALERT_NOTIF_CTRL_PT_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_CTRL] = handle;
                break;

              case UNREAD_ALERT_STATUS_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_UNREAD_START] = handle;
                timeAppEndHdlIdx = HDL_ALERT_NTF_UNREAD_END;
                break;

              case NEW_ALERT_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_NEW_START] = handle;
                timeAppEndHdlIdx = HDL_ALERT_NTF_NEW_END;
                break;

              case SUP_NEW_ALERT_CAT_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_NEW_CAT] = handle;
                break;

              case SUP_UNREAD_ALERT_CAT_UUID:
                timeAppHdlCache[HDL_ALERT_NTF_UNREAD_CAT] = handle;
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
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          
          // If didn't find new alert characteristic
          if ( timeAppHdlCache[HDL_ALERT_NTF_NEW_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_ALERT_NTF_NEW_START] <
                    timeAppHdlCache[HDL_ALERT_NTF_NEW_END] )
          {
            // Discover incoming alert characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_ALERT_NTF_NEW_START] + 1,
                                   timeAppHdlCache[HDL_ALERT_NTF_NEW_END],
                                   timeAppTaskId );
                                        
            newState = DISC_ALERT_NTF_NEW_CCCD;
          }
          else
          {
            // Missing required characteristic descriptor
            timeAppHdlCache[HDL_ALERT_NTF_NEW_START] = 0;
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
              timeAppHdlCache[HDL_ALERT_NTF_NEW_CCCD] =
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
          if ( timeAppHdlCache[HDL_ALERT_NTF_NEW_CCCD] != 0 )
          {
            // Should we look for unread category status CCCD
            if ( timeAppHdlCache[HDL_ALERT_NTF_UNREAD_START] <
                 timeAppHdlCache[HDL_ALERT_NTF_UNREAD_END] )
            {
              // Discover unread category status characteristic descriptors
              GATT_DiscAllCharDescs( timeAppConnHandle,
                                     timeAppHdlCache[HDL_ALERT_NTF_UNREAD_START] + 1,
                                     timeAppHdlCache[HDL_ALERT_NTF_UNREAD_END],
                                     timeAppTaskId );
                                          
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
            timeAppHdlCache[HDL_ALERT_NTF_NEW_START] = 0;
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
              timeAppHdlCache[HDL_ALERT_NTF_UNREAD_CCCD] =
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
        timeAppSvcStartHdl = timeAppSvcEndHdl = 0;
        timeAppEndHdlIdx = 0;
        
        // Discover service by UUID
        GATT_DiscPrimaryServiceByUUID( timeAppConnHandle, uuid,
                                       ATT_BT_UUID_SIZE, timeAppTaskId );      

        newState = DISC_BATT_SVC;
      } 
      break;

    case DISC_BATT_SVC:
      // Service found, store handles
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
           pMsg->msg.findByTypeValueRsp.numInfo > 0 )
      {
        timeAppSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
        timeAppSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      }
      
      // If procedure complete
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
             pMsg->hdr.status == bleProcedureComplete ) ||
           ( pMsg->method == ATT_ERROR_RSP ) )
      {
        // If service found
        if ( timeAppSvcStartHdl != 0 )
        {
          // Discover all characteristics
          GATT_DiscAllChars( timeAppConnHandle, timeAppSvcStartHdl,
                             timeAppSvcEndHdl, timeAppTaskId );
          
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
            if ( timeAppEndHdlIdx != 0 )
            {
              // End handle is one less than handle of characteristic declaration
              timeAppHdlCache[timeAppEndHdlIdx] = BUILD_UINT16(p[0], p[1]) - 1;
              
              timeAppEndHdlIdx = 0;
            }

            // If UUID is of interest, store handle
            switch ( uuid )
            {
              case BATT_LEVEL_UUID:
                timeAppHdlCache[HDL_BATT_LEVEL_START] = handle;
                timeAppEndHdlIdx = HDL_BATT_LEVEL_END;
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
          if ( timeAppEndHdlIdx != 0 )
          {
            timeAppHdlCache[timeAppEndHdlIdx] = timeAppSvcEndHdl;
            timeAppEndHdlIdx = 0;
          }
          
          // If didn't find mandatory characteristic
          if ( timeAppHdlCache[HDL_BATT_LEVEL_START] == 0 )
          {
            newState = DISC_FAILED;
          }
          else if ( timeAppHdlCache[HDL_BATT_LEVEL_START] <
                    timeAppHdlCache[HDL_BATT_LEVEL_END] )
          {
            // Discover characteristic descriptors
            GATT_DiscAllCharDescs( timeAppConnHandle,
                                   timeAppHdlCache[HDL_BATT_LEVEL_START] + 1,
                                   timeAppHdlCache[HDL_BATT_LEVEL_END],
                                   timeAppTaskId );
                                        
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
              timeAppHdlCache[HDL_BATT_LEVEL_CCCD] =
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
