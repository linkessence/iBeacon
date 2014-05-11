/**************************************************************************************************
  Filename:       hal_gpiodbg.c
  Revised:        $Date: 2011-09-23 10:34:21 -0700 (Fri, 23 Sep 2011) $
  Revision:       $Revision: 27694 $

  Description:    This file contains the interface to the HAL GPIO Debug module.


  Copyright 2006-2011 Texas Instruments Incorporated. All rights reserved.

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
#include "string.h"

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* The following define which port pins are spare and can be used for debug */
#define HAL_GPIODBG_P1_GPIO_PINS  ( BV( 7 ) | BV( 6 ))
#define HAL_GPIODBG_P2_GPIO_PINS  ( BV( 4 ) | BV( 3 ) | BV( 0 ) )


#define HAL_UART_TX_PORT 1
#define HAL_UART_TX_PIN  5

#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )
#define GPIOUART_TX_HEIGH()     HAL_IO_SET(HAL_UART_TX_PORT,  HAL_UART_TX_PIN,  1);
#define GPIOUART_TX_LOW()     HAL_IO_SET(HAL_UART_TX_PORT,  HAL_UART_TX_PIN,  0);

/* P0SEL and P1SEL select functionality by directly mapping pin number to
 * the corresponding bit in the register. P2SEL only allows mapping of
 * pins 0, 3, and 4, and the pin functionality of each of those pins is
 * mapped to bits 0-2 of P2SEL.
 */
#define HAL_GPIODBG_P2SEL_BITS    ( BV( 2 ) | BV( 1 ) | BV( 0 ) )

/* These defines indicate the direction of each pin */
#define HAL_GPIODBG_P1_OUTPUT_PINS  HAL_GPIODBG_P1_GPIO_PINS
#define HAL_GPIODBG_P2_OUTPUT_PINS  HAL_GPIODBG_P2_GPIO_PINS

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/

/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
uint16 bandrate;
uint8 gUartByte;
uint8 gCompliteFlag=0;
uint8 gNewData = 0;
/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void HalGpioUartSetBand(uint8 band);
void HalGpioUartSendByte(int8 data);
void HalGpioUartSendString(int8 *ptr,uint8 len);
void HalGpioUartWaitUs(uint16 microSecs);
void HalTimer3Init();
void HalTimer3Start();
void HalTimer3Stop();
void HalGpioUartCallBack();
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/
void HalTimer3Init()
{
    T3CTL &= ~0x03;
	T3CTL = 0xAE;	//32分频，中断使能，模模式
	T3CC0 = 0x69;	// 1Mhz~1us  105us中断
	T3CCTL0 = 0x44;	//一定要设置比较的方式，不然不会中断的!!!,T3CCTL0.IM==1,T3CCTL0.MODE==1
}
void HalTimer3Start()
{
	T3CTL |= 0x10; 
}
void HalTimer3Stop()
{
	T3CTL &= ~0x10; 
}
void HalTimer3EnableInt()
{
        //EA = 1;
	//T3IE = 1;
	IEN1 |= BV(3);
}
void HalTimer3DisableInt()
{
	//T3IE = 0;
	IEN1 &= ~ BV(3);
}
void HalGpioUartCallBack()
{
	static uint8 i= 0;
	if(gNewData ==1){
		if(gCompliteFlag==0){
			GPIOUART_TX_LOW();//start
			gCompliteFlag=1;
		}else if(gCompliteFlag==1){
			if(gUartByte&0x01){
	    		GPIOUART_TX_HEIGH();//data 1
			}else{
	        	GPIOUART_TX_LOW();//data 0
		  	}
			i++;
			gUartByte>>=1;
			if(i==8){
				i = 0;
				gCompliteFlag = 2;
				
			}
		}else if(gCompliteFlag==2){
			GPIOUART_TX_HEIGH();//data 1
			gCompliteFlag = 3;
		}else if(gCompliteFlag==3){
			gCompliteFlag = 4;
			gNewData = 0;
		}
		
	}
	
}
void HalGpioUartSendByte(int8 data)
{
	while(gNewData ==1 );
	gUartByte = data;
	gCompliteFlag = 0;
	gNewData = 1;
}
void HalGpioUartSendString(int8 *ptr,uint8 len)
{
	uint8 i;
	for(i=0;i<len;i++){
		HalGpioUartSendByte(ptr[i]);
	}
}
/**************************************************************************************************
 * @fn      HalGpioDbgInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalGpioDbgInit( void )
{
  /* The spare GPIO pins will be initialized to be outputs driven low to save power. */
  P1 &= ~HAL_GPIODBG_P1_GPIO_PINS;
  P2 &= ~HAL_GPIODBG_P2_GPIO_PINS;

  //TX init HEIGH
  P1 |= BV(HAL_UART_TX_PIN);
  P1SEL &= (uint8) ~BV(HAL_UART_TX_PIN);
  P1DIR |= (uint8) BV(HAL_UART_TX_PIN);
  

  
  
  /* Configure spare pin function as GPIO */
  //P1SEL &= (uint8) ~HAL_GPIODBG_P1_GPIO_PINS;
  //P2SEL &= (uint8) ~HAL_GPIODBG_P2SEL_BITS;

  /* Configure spare pin direction to output */
  //P1DIR |= (uint8) HAL_GPIODBG_P1_OUTPUT_PINS;
  //P2DIR |= (uint8) HAL_GPIODBG_P2_OUTPUT_PINS;
  HalTimer3Init();
  HalTimer3EnableInt();
  HalTimer3Start();
#if 0
  unsigned char string[]="liuyu love chenglan";
  while(1){
    HalGpioUartSendByte(0xaa);
    HalGpioUartWaitUs(1500);
    HalGpioUartSendByte(0x55);
    HalGpioUartWaitUs(1500);
    HalGpioUartSendString(string,strlen(string));
    HalGpioUartWaitUs(1500);
  }
#endif
}
void HalGpioUartSetBand(uint8 band)
{
	if(band ==1){//115200
		bandrate = 8;//87us
	}else if(band == 2){//34800
		bandrate = 26;//260us
	}else if(band == 3){//19200
		bandrate = 52;//520us
	}else if(band ==4){//9600
		bandrate = 104;//
	}else if(band == 5){//2400
		bandrate = 410;//4166us
	}else{//115200
		bandrate = 8;//87us
	}
		

	
}

/**************************************************************************************************
 * @fn      HalGpioUartWaitUs
 *
 * @brief   wait for x us. @ 32MHz MCU clock it takes 32 "nop"s for 1 us delay.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void HalGpioUartWaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

HAL_ISR_FUNCTION( halTimer3Isr,  T3_VECTOR )
{
	HAL_ENTER_ISR();
	HalGpioUartCallBack();
	HAL_EXIT_ISR();
}

int8* IntToStr(int8* buf, int m)
{
    char tmp[16];
    int isNegtive = 0;
    int index;

    if(m < 0)
    {
        isNegtive = 1;
        m = - m;
    }

    tmp[15] = '\0';
    index = 14;
    do 
    {
        tmp[index--] = m % 10 + '0';
        m /= 10;
    } while (m > 0);

    if(isNegtive)
        tmp[index--] = '-';
    
    strcpy(buf, tmp + index + 1);

    return buf;
}
uint8 LenOfStr(int8 *str)
{
	return strlen(str);
}
/**************************************************************************************************
**************************************************************************************************/
