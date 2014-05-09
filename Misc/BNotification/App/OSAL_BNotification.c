/***************************************************************************************************
 *   Copyright (C) 2014-2020 LinkEssence Inc.
 *   All rights reserved.
 *
 *   Filename:      OSAL_BNotification.c
 *
 *   Author:        Tao Zhang
 *   Verifier:
 *
 *   Description:
 *
 *   This file contains the OSAL functions that allows user setup tasks.
 *
 *   ----------------------------------------------------------------------------------------------
 *   Version                Date                   Modifier                 Description
 *   ----------------------------------------------------------------------------------------------
 *   1.0.0                  May 07, 2014           Tao Zhang                Initial edition
 *
 **************************************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/

#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* HAL */
#include "hal_drivers.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbTimer.h"
#endif

/* gap */
#include "gap.h"

/* Profiles */
#include "broadcaster.h"

/* Application */
#include "BNotification.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,                                                  // task 0
  Hal_ProcessEvent,                                                 // task 1
  HCI_ProcessEvent,                                                 // task 2
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),           // task 3
#endif
  GAP_ProcessEvent,                                                 // task 4
  GAPRole_ProcessEvent,                                             // task 5
  BNotification_App_ProcessEvent                                    // task 6
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  uint8 taskID = 0;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  /* LL Task */
  LL_Init( taskID++ );

  /* Hal Task */
  Hal_Init( taskID++ );

  /* HCI Task */
  HCI_Init( taskID++ );

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

  /* GAP Task */
  GAP_Init( taskID++ );

  /* Profiles */
  GAPRole_Init( taskID++ );

  /* Application */
  BNotification_App_Init( taskID );
}