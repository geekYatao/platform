/**************************************************************************************************
  Filename:       OSAL_SerialApp.c
  Revised:        $Date: 2008-02-07 12:10:00 -0800 (Thu, 07 Feb 2008) $
  Revision:       $Revision: 16360 $

  Description:    This file contains all the settings and other functions
                  that the user should set and change.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
 * INCLUDES
 */
#include "ZComDef.h"
#include "hal_drivers.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

#if defined ( MT_TASK )
  #include "MT.h"
  #include "MT_TASK.h"
#endif

#include "nwk.h"
#include "APS.h"
#include "ZDApp.h"
#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  #include "ZDNwkMgr.h"
#endif
#if defined ( ZIGBEE_FRAGMENTATION )
  #include "aps_frag.h"
#endif

#ifdef COOR
  #include "Coordinator.h"
#endif

#ifdef COORSHOW
  #include "CoordinatorShow.h"
#endif

#ifdef ROUTER
  #include "Router.h"
#endif

#ifdef MOTOR
  #include "Motor.h"
#endif

#ifdef PHOTO
  #include "Photo.h"
#endif

#ifdef SHAKE
  #include "Shake.h"
#endif

#ifdef SMOKE
  #include "Smoke.h"
#endif

#ifdef HALL
  #include "Hall.h"
#endif

#ifdef REED
  #include "Reed.h"
#endif

#ifdef DOPPLER
  #include "Doppler.h"
#endif

#ifdef TOUCH
  #include "Touch.h"
#endif

#ifdef SOUND
  #include "Sound.h"
#endif

#ifdef TEMPANDHUM
  #include "TempAndHum.h"
#endif

#ifdef ACCELE
  #include "Accele.h"
#endif

#ifdef PWM
  #include "PWM.h"
#endif

#ifdef ULTRASOUND
  #include "Ultrasound.h"
#endif

#ifdef RFID
  #include "rfid.h"
#endif

#ifdef FM1702SL
  #include "fm1702sl.h"
#endif

#ifdef FM1702_IDE
  #include "FM1702_IDE.h"
#endif

#ifdef EM4095
  #include "EM4095.h"
#endif

#ifdef COLOR
  #include "Color.h"
#endif

#ifdef FIRE
  #include "Fire.h"
#endif

#ifdef PARTICLE
  #include "Particle.h"
#endif

#ifdef GYROSCOPE
  #include "Gyroscope.h"
#endif

#ifdef IR
  #include "IR.h"
#endif

#ifdef ALCOHOL
  #include "Alcohol.h"
#endif

#ifdef RELAY
  #include "Relay.h"
#endif

#ifdef AS3992
  #include "AS3992.h"
#endif

#ifdef LD3320
  #include "LD3320.h"
#endif

#ifdef PRESS
  #include "Press.h"
#endif

#ifdef CO2
#include "CO2.h"
#endif

#ifdef IR_SW
#include "IR_SW.h"
#endif

#ifdef LED_SCREEN
#include "LED_Screen.h"
#endif

#ifdef IR_SAFE
#include "IR_safe.h"
#endif

#ifdef ALARMLAMP
#include "AlarmLamp.h"
#endif

#ifdef CH3
#include "CH3.h"
#endif

#ifdef WINDOW
#include "Window.h"
#endif

#ifdef SMARTLAMP
#include "SmartLamp.h"
#endif

#ifdef HUMIDIFIER
#include "Humidifier.h"
#endif

#ifdef TABLELAMP
#include "TableLamp.h"
#endif

#ifdef FAN
#include "Fan.h"
#endif

#ifdef LAMPHOLDER1
#include "LampHolder1.h"
#endif

#ifdef LAMPHOLDER2
#include "LampHolder2.h"
#endif

#ifdef LAMPHOLDER3
#include "LampHolder3.h"
#endif

#ifdef LAMPHOLDER4
#include "LampHolder4.h"
#endif

#ifdef Pwm_Ac1
#include "PWM.h"
#endif

#ifdef Pwm_Ac2
#include "PWM.h"
#endif

#ifdef Pwm_Ac3
#include "PWM.h"
#endif

#ifdef Pwm_Ac4
#include "PWM.h"
#endif

#ifdef Adxl345
#include "adxl345.h"
#endif

#ifdef FINGER
#include "fingerprint.h"
#endif

#ifdef IR_AIR
#include "IR_Air.h"
#endif

#ifdef IR_FAN
#include "IR_Fan.h"
#endif

#ifdef IR
#include "IR.h"
#endif
/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] = {
  macEventLoop,
  nwk_event_loop,
  Hal_ProcessEvent,
#if defined( MT_TASK )
  MT_ProcessEvent,
#endif
  APS_event_loop,
#if defined ( ZIGBEE_FRAGMENTATION )
  APSF_ProcessEvent,
#endif
  ZDApp_event_loop,
#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  ZDNwkMgr_event_loop,
#endif
  SerialApp_ProcessEvent
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

  macTaskInit( taskID++ );
  nwk_init( taskID++ );
  Hal_Init( taskID++ );
#if defined( MT_TASK )
  MT_TaskInit( taskID++ );
#endif
  APS_Init( taskID++ );
#if defined ( ZIGBEE_FRAGMENTATION )
  APSF_Init( taskID++ );
#endif
  ZDApp_Init( taskID++ );
#if defined ( ZIGBEE_FREQ_AGILITY ) || defined ( ZIGBEE_PANID_CONFLICT )
  ZDNwkMgr_Init( taskID++ );
#endif
  SerialApp_Init( taskID );
}

/*********************************************************************
*********************************************************************/