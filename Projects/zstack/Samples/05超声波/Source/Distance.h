/**************************************************************************************************
  Filename:       Distance.h
  Revised:        $Date: 2012-02-12 15:58:41 -0800 (Sun, 12 Feb 2012) $
  Revision:       $Revision: 29216 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#ifndef Distance_H
#define Distance_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"

/*********************************************************************
 * CONSTANTS
 */
/* SW_6 is at P0.1 */
#define HAL_KEY_SW_7_PORT   P1
#define HAL_KEY_SW_7_BIT    BV(3)
#define HAL_KEY_SW_7_SEL    P1SEL
#define HAL_KEY_SW_7_DIR    P1DIR

/* edge interrupt */
#define HAL_KEY_SW_7_EDGEBIT  BV(0)
#define HAL_KEY_SW_7_EDGE     HAL_KEY_RISING_EDGE


/* SW_6 interrupts */
#define HAL_KEY_SW_7_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_7_IENBIT   BV(4) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_7_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_7_ICTLBIT  BV(3) /* P0IEN - P0.1 enable/disable bit */
#define HAL_KEY_SW_7_PXIFG    P1IFG /* Interrupt flag at source */

// These constants are only for example and should be changed to the
// device's needs
#define Distance_ENDPOINT           10

#define Distance_PROFID             0x0F04
#define Distance_DEVICEID           0x0001
#define Distance_DEVICE_VERSION     0
#define Distance_FLAGS              0

#define Distance_MAX_CLUSTERS       1
#define Distance_CLUSTERID          5

// Send Message Timeout
#define Distance_SEND_MSG_TIMEOUT   4000     // Every 3 seconds
  

// Application Events (OSAL) - These are bit weighted definitions.
#define Distance_SEND_MSG_EVT       0x0001
  
  
/////////////////////////////////////////////////////////////  
#define DISTANCEMEASUREMENT_EVT       0x0001
  
#define DISTANCEMEASUREMENT_TIMEOUT   500

#if defined( IAR_ARMCM3_LM )
#define Distance_RTOS_MSG_EVT       0x0002
#endif  

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void Distance_Init( byte task_id );

//extern void DistanceMeasurement_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 Distance_ProcessEvent( byte task_id, UINT16 events );

//extern UINT16 DistanceMeasurement_ProcessEvent( byte task_id, UINT16 events );

///////////////////////////////////////////////////////////////////////

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* Distance_H */
