#ifndef MOTOR_H
#define MOTOR_H

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

// These constants are only for example and should be changed to the
// device's needs
#define SERIALAPP_ENDPOINT           11

#define SERIALAPP_PROFID             0x0F05
#define SERIALAPP_DEVICEID           0x0001
#define SERIALAPP_DEVICE_VERSION     0
#define SERIALAPP_FLAGS              0

#define SERIALAPP_MAX_CLUSTERS       2
#define SERIALAPP_CLUSTERID1         1
#define SERIALAPP_CLUSTERID2         2

#define SERIALAPP_SEND_EVT           0x0001
#define PERIOD_EVT                   0x0002
  
#define START_EVT                   0x0004
#define IMAGE1_EVT                  0x0008  
#define CHAR1_EVT                   0x0010
#define IMAGE2_EVT                  0x0020
#define CHAR2_EVT                   0x0040
#define MATCH_EVT                   0x0080  
#define MODEL_EVT                   0x0100   
#define STORE_EVT                   0x0200   
#define SEARCH_EVT                  0x0400 
#define TEMPLATENUM_EVT             0x0800   
#define EMPTY_EVT                   0x1000  
  

// OTA Flow Control Delays
#define SERIALAPP_ACK_DELAY          1
#define SERIALAPP_NAK_DELAY          16

// OTA Flow Control Status
#define OTA_SUCCESS                  ZSuccess
#define OTA_DUP_MSG                 (ZSuccess+1)
#define OTA_SER_BUSY                (ZSuccess+2)

#define START                                 0x09
#define FINGERINPUT                           0x01
#define FINGERSEARCH                          0x02
#define EMPTY                                 0x03
#define DELETECHAR                            0x04
#define TEMPLATENUM                           0x05  
#define RESULT                                0x08
#define OPEN                                  0x0a
 
  
#define RIGHT                      0x03
#define WRONG                      0x01  
#define LOCK P0_0 = 0
#define ULOCK P0_0 = 1

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern byte SerialApp_TaskID;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Serial Transfer Application
 */
extern void SerialApp_Init( byte task_id );

/*
 * Task Event Processor for the Serial Transfer Application
 */
extern UINT16 SerialApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif 
