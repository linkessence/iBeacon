/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2012-10-31 16:16:01 -0700 (Wed, 31 Oct 2012) $
  Revision:       $Revision: 32004 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2006-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
/*********************************************************************
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"
#include "osal_clock.h"
#include "hal_sleep.h"

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

#define HAL_KEY_BIT0            0x01
#define HAL_KEY_BIT1            0x02
#define HAL_KEY_BIT2            0x04
#define HAL_KEY_BIT3            0x08
#define HAL_KEY_BIT4            0x10
#define HAL_KEY_BIT5            0x20
#define HAL_KEY_BIT6            0x40
#define HAL_KEY_BIT7            0x80

#define HAL_KEY_RISING_EDGE     0
#define HAL_KEY_FALLING_EDGE    1

#define HAL_KEY_PDUP2           0x80
#define HAL_KEY_PDUP1           0x40
#define HAL_KEY_PDUP0           0x20

#define HAL_KEY_DEBOUNCE_VALUE  25  // TODO: adjust this value
#define HAL_KEY_POLLING_VALUE   100

#define HAL_KEY_CODE_NOKEY      0xFF

/* Define number of rows and columns in keypad matrix */
#define HAL_KEY_NUM_ROWS        3
#define HAL_KEY_NUM_COLUMNS     16

/* The following define which port pins are being used by keypad service */
#define HAL_KEY_P0_GPIO_PINS  ( HAL_KEY_BIT0 | HAL_KEY_BIT1 | HAL_KEY_BIT2 | HAL_KEY_BIT3 | HAL_KEY_BIT4)
#define HAL_KEY_P1_GPIO_PINS  ( HAL_KEY_BIT4 )

/* These defines indicate the direction of each pin */
#define HAL_KEY_P0_INPUT_PINS   ( HAL_KEY_BIT0 | HAL_KEY_BIT1 | HAL_KEY_BIT2 )
#define HAL_KEY_P0_OUTPUT_PINS  ( HAL_KEY_BIT3 | HAL_KEY_BIT4 )
#define HAL_KEY_P1_INPUT_PINS   0x00
#define HAL_KEY_P1_OUTPUT_PINS  ( HAL_KEY_BIT4 )

/* Which pins are used for key interrupts */
#define HAL_KEY_P0_INTERRUPT_PINS   ( HAL_KEY_BIT0 | HAL_KEY_BIT1 | HAL_KEY_BIT2 )

/* Defines for each output pin assignment */
#define HAL_KEY_SHIFT_REGISTER_CLOCK_PIN  P0_3
#define HAL_KEY_SHIFT_REGISTER_POWER_PIN  P0_4
#define HAL_KEY_SHIFT_REGISTER_DATA_PIN   P1_4
#define HAL_KEY_SHIFT_REGISTER_POWER_OFF 0
#define HAL_KEY_SHIFT_REGISTER_POWER_ON 1


//ghostyu
#define HAL_KEY_RISING_EDGE   0
#define HAL_KEY_FALLING_EDGE  1

#define HAL_KEY_DEBOUNCE_VALUE  25

/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF P0IF
#define HAL_KEY_CPU_PORT_2_IF P2IF

/* SW_6 is at P0.1 */
#define HAL_KEY_SW_6_PORT   P0
#define HAL_KEY_SW_6_BIT    BV(1)
#define HAL_KEY_SW_6_SEL    P0SEL
#define HAL_KEY_SW_6_DIR    P0DIR

/* edge interrupt */
#define HAL_KEY_SW_6_EDGEBIT  BV(0)
#define HAL_KEY_SW_6_EDGE     HAL_KEY_FALLING_EDGE

/* SW_6 interrupts */
#define HAL_KEY_SW_6_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_6_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_6_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_6_ICTLBIT  BV(1) /* P0IEN - P0.1 enable/disable bit */
#define HAL_KEY_SW_6_PXIFG    P0IFG /* Interrupt flag at source */

/* Joy stick move at P2.0 */
#define HAL_KEY_JOY_MOVE_PORT   P2
#define HAL_KEY_JOY_MOVE_BIT    BV(0)
#define HAL_KEY_JOY_MOVE_SEL    P2SEL
#define HAL_KEY_JOY_MOVE_DIR    P2DIR

/* edge interrupt */
#define HAL_KEY_JOY_MOVE_EDGEBIT  BV(3)
#define HAL_KEY_JOY_MOVE_EDGE     HAL_KEY_FALLING_EDGE

/* Joy move interrupts */
#define HAL_KEY_JOY_MOVE_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_JOY_MOVE_IENBIT   BV(1) /* Mask bit for all of Port_2 */
#define HAL_KEY_JOY_MOVE_ICTL     P2IEN /* Port Interrupt Control register */
#define HAL_KEY_JOY_MOVE_ICTLBIT  BV(0) /* P2IENL - P2.0<->P2.3 enable/disable bit */
#define HAL_KEY_JOY_MOVE_PXIFG    P2IFG /* Interrupt flag at source */

#define HAL_KEY_JOY_CHN   HAL_ADC_CHANNEL_6

uint8 halGetJoyKeyInput(void);






/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */
uint8 halSaveIntKey;              /* used by ISR to save state of interrupt-driven keys */

static uint8 HalKeyConfigured;
static uint8 halKeyTimerRunning;  // Set to true while polling timer is running in interrupt
                                  // enabled mode

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt (void);
void halClockShiftRegister (void);
void halPowerDownShiftRegister (void);
void halSetShiftRegisterData( uint8 data );
void halPowerUpShiftRegister( void );

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/
/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
#if (HAL_KEY == TRUE)
  /* Initialize previous key to 0 */
  halKeySavedKeys = HAL_KEY_CODE_NOKEY;

  /* The advanced remote doesn't have the same 8X8 row/column matrix as in other
   * products. Instead, a 3X16 row/column matrix is used, with the rows continuing
   * to be utilized by GPIOs, but the columns are generated via a 16 bit
   * shift register. Controls for the shift register are, however, utilized with
   * GPIOs.
   *
   * Another difference is that the GPIOs utilized for the rows are split between
   * P0 and P1.
   */
   //ghostyu from EM
  HAL_KEY_SW_6_SEL &= ~(HAL_KEY_SW_6_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_6_DIR &= ~(HAL_KEY_SW_6_BIT);    /* Set pin direction to Input */
  HAL_KEY_JOY_MOVE_SEL &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin function to GPIO */
  HAL_KEY_JOY_MOVE_DIR &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin direction to Input */

  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;

  halKeyTimerRunning = FALSE;
#endif /* HAL_KEY */
}

/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
#if (HAL_KEY == TRUE)
  /* Enable/Disable Interrupt */
  Hal_KeyIntEnable = interruptEnable;

  /* Register the callback fucntion */
  pHalKeyProcessFunction = cback;

  /* Determine if interrupt is enabled or not */
  if (Hal_KeyIntEnable)
  {

  //ghostyu from EM
    /* Rising/Falling edge configuratinn */
    PICTL &= ~(HAL_KEY_SW_6_EDGEBIT);    /* Clear the edge bit */
    /* For rising edge, the bit must be set. */
    PICTL &= ~(HAL_KEY_SW_6_EDGEBIT);



    /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
    HAL_KEY_SW_6_ICTL |= HAL_KEY_SW_6_ICTLBIT;
    HAL_KEY_SW_6_IEN |= HAL_KEY_SW_6_IENBIT;
    HAL_KEY_SW_6_PXIFG = ~(HAL_KEY_SW_6_BIT);



    /* Rising/Falling edge configuratinn */

    HAL_KEY_JOY_MOVE_ICTL &= ~(HAL_KEY_JOY_MOVE_EDGEBIT);    /* Clear the edge bit */
    /* For falling edge, the bit must be set. */

    HAL_KEY_JOY_MOVE_ICTL |= HAL_KEY_JOY_MOVE_EDGEBIT;


    /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
    HAL_KEY_JOY_MOVE_ICTL |= HAL_KEY_JOY_MOVE_ICTLBIT;
    HAL_KEY_JOY_MOVE_IEN |= HAL_KEY_JOY_MOVE_IENBIT;
    HAL_KEY_JOY_MOVE_PXIFG = ~(HAL_KEY_JOY_MOVE_BIT);

	
    /* Do this only after the hal_key is configured - to work with sleep stuff */
    if (HalKeyConfigured == TRUE)
    {
      osal_stop_timerEx( Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
    }
  }
  else    /* Interrupts NOT enabled */
  {
    HAL_KEY_SW_6_ICTL &= ~(HAL_KEY_SW_6_ICTLBIT); /* don't generate interrupt */
    HAL_KEY_SW_6_IEN &= ~(HAL_KEY_SW_6_IENBIT);   /* Clear interrupt enable bit */


    osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_POLLING_VALUE);    /* Kick off polling */
  }

  /* Key now is configured */
  HalKeyConfigured = TRUE;
#endif /* HAL_KEY */
}

/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
  uint8 keys = 0;

  //注意这里的SW6对应SmartRF开发板上的S1,是高电平有效
  if ((HAL_KEY_SW_6_PORT & HAL_KEY_SW_6_BIT))    /* Key is active HIGH */
  {
    keys |= HAL_KEY_SW_6;
  }

  if ((HAL_KEY_JOY_MOVE_PORT & HAL_KEY_JOY_MOVE_BIT))  /* Key is active low */
  {
    keys |= halGetJoyKeyInput();
  }
  return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
#define GHOSTYU_CODE_NOKEY	0x00
void HalKeyPoll (void)
{
#if (HAL_KEY == TRUE)

  uint8 keys = 0;
  /*
  *  If interrupts are enabled, get the status of the interrupt-driven keys from 'halSaveIntKey'
  *  which is updated by the key ISR.  If Polling, read these keys directly.
  */
  keys = HalKeyRead();

  /* Exit if polling and no keys have changed */
  if (!Hal_KeyIntEnable)
  {
    if (keys == halKeySavedKeys)
    {
      return;
    }
    halKeySavedKeys = keys;     /* Store the current keys for comparation next time */
  }

  if(keys==0){
     (pHalKeyProcessFunction) (0, HAL_KEY_STATE_NORMAL);
  }

  /* Invoke Callback if new keys were depressed */
  if ((keys != 0 /*HAL_KEY_CODE_NOKEY*/) && Hal_KeyIntEnable &&
      (pHalKeyProcessFunction))
  {
    // When interrupt is enabled, send HAL_KEY_CODE_NOKEY as well so that
    // application would know the previous key is no longer depressed.
    
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);

  }
  
  //如果使能了中断触发，在中断触发后，poll按键，先是去抖动，去抖动结束后，如果该按键仍存在，
  //则调用按键回调函数，向用户传递按键数据
  //然后执行下面的代码，再延时50ms，看时候该按键是否仍然没有释放。如果没有释放，则继续调用按键的回调函数。
  
  if (Hal_KeyIntEnable)
  {
    if (keys != 0/*HAL_KEY_CODE_NOKEY*/)
    {
      // In order to trigger callback again as far as the key is depressed,
      // timer is called here.
      osal_start_timerEx(Hal_TaskID, HAL_KEY_EVENT, 50);
	  
    }
    else
    {
      halKeyTimerRunning = FALSE;
		
    }
  }
#endif /* HAL_KEY */

}

/**************************************************************************************************
 * @fn      halGetJoyKeyInput
 *
 * @brief   Map the ADC value to its corresponding key.
 *
 * @param   None
 *
 * @return  keys - current joy key status
 **************************************************************************************************/
uint8 halGetJoyKeyInput(void)
{
  /* The joystick control is encoded as an analog voltage.
   * Read the JOY_LEVEL analog value and map it to joy movement.
   */
  uint8 adc;
  uint8 ksave0 = 0;
  uint8 ksave1;

  /* Keep on reading the ADC until two consecutive key decisions are the same. */
  do
  {
    ksave1 = ksave0;    /* save previouse key reading */

    adc = HalAdcRead (HAL_KEY_JOY_CHN, HAL_ADC_RESOLUTION_8);

    if ((adc >= 2) && (adc <= 38))
    {
       ksave0 |= HAL_KEY_UP;
    }
    else if ((adc >= 74) && (adc <= 88))
    {
      ksave0 |= HAL_KEY_RIGHT;
    }
    else if ((adc >= 60) && (adc <= 73))
    {
      ksave0 |= HAL_KEY_LEFT;
    }
    else if ((adc >= 39) && (adc <= 59))
    {
      ksave0 |= HAL_KEY_DOWN;
    }
    else if ((adc >= 89) && (adc <= 100))
    {
      ksave0 |= HAL_KEY_CENTER;
    }else{
	  ksave0=0;
	}

  } while (ksave0 != ksave1);

  return ksave0;
}
/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{

#if (HAL_KEY == TRUE)
//ghostyu  from EM
  bool valid=FALSE;
  if (HAL_KEY_SW_6_PXIFG & HAL_KEY_SW_6_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_SW_6_PXIFG = ~(HAL_KEY_SW_6_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }

  if (HAL_KEY_JOY_MOVE_PXIFG & HAL_KEY_JOY_MOVE_BIT)  /* Interrupt Flag has been set */
  {
    HAL_KEY_JOY_MOVE_PXIFG = ~(HAL_KEY_JOY_MOVE_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  
  if (valid)
  {
    if (!halKeyTimerRunning)
    {
      halKeyTimerRunning = TRUE;
      osalTimeUpdate();
      osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
    }
  }
#endif /* HAL_KEY */
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
  /* Sleep!!!
   * Nothing to do.
   */
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  /* Wakeup!!!
   * Nothing to do. In fact. HalKeyRead() may not be called here.
   * Calling HalKeyRead() will trigger key scanning and interrupt flag clearing in the end,
   * which is no longer compatible with hal_sleep.c module.
   */
  /* Wake up and read keys */
  return TRUE;
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINES
 ***************************************************************************************************/

/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  HAL_ENTER_ISR();
  if (HAL_KEY_SW_6_PXIFG & HAL_KEY_SW_6_BIT)
  {
    halProcessKeyInterrupt();
  }

#if HAL_KEY
//ghostyu
  HAL_KEY_SW_6_PXIFG = 0;
  HAL_KEY_CPU_PORT_0_IF = 0;
#endif

  HAL_EXIT_ISR();
}

/**************************************************************************************************
 * @fn      halKeyPort2Isr
 *
 * @brief   Port2 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort2Isr, P2INT_VECTOR )
{
  if (HAL_KEY_JOY_MOVE_PXIFG & HAL_KEY_JOY_MOVE_BIT)
  {
    halProcessKeyInterrupt();
  }

  /*
    Clear the CPU interrupt flag for Port_2
    PxIFG has to be cleared before PxIF
    Notes: P2_1 and P2_2 are debug lines.
  */
  HAL_KEY_JOY_MOVE_PXIFG = 0;
  HAL_KEY_CPU_PORT_2_IF = 0;
}
/**************************************************************************************************
 * @fn      halClockShiftRegister
 *
 * @brief   Simply provides a single clock pulse to the shift register.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halClockShiftRegister( void )
{
    HAL_KEY_SHIFT_REGISTER_CLOCK_PIN = 1;
    HAL_KEY_SHIFT_REGISTER_CLOCK_PIN = 0;
}

/**************************************************************************************************
 * @fn      halPowerDownShiftRegister
 *
 * @brief   Disables the shift register to save power.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halPowerDownShiftRegister( void )
{
  HAL_KEY_SHIFT_REGISTER_CLOCK_PIN = 0;
  HAL_KEY_SHIFT_REGISTER_POWER_PIN = HAL_KEY_SHIFT_REGISTER_POWER_OFF;
  HAL_KEY_SHIFT_REGISTER_DATA_PIN = 0;
}

/**************************************************************************************************
 * @fn      halPowerUpShiftRegister
 *
 * @brief   Supplies power to the shift register.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halPowerUpShiftRegister( void )
{
  HAL_KEY_SHIFT_REGISTER_POWER_PIN = HAL_KEY_SHIFT_REGISTER_POWER_ON;
}

/**************************************************************************************************
 * @fn      halSetShiftRegisterData
 *
 * @brief   Writes data to the input of the shift register.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halSetShiftRegisterData( uint8 data )
{
  /* Data input is 1 bit, so make sure we only use LSB */
  HAL_KEY_SHIFT_REGISTER_DATA_PIN = (data & 0x01);
}

/**************************************************************************************************
**************************************************************************************************/
