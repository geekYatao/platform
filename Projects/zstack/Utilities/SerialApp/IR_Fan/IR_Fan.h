#ifndef IR_FAN_H
#define IR_FAN_H

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

// OTA Flow Control Delays
#define SERIALAPP_ACK_DELAY          1
#define SERIALAPP_NAK_DELAY          16

// OTA Flow Control Status
#define OTA_SUCCESS                  ZSuccess
#define OTA_DUP_MSG                 (ZSuccess+1)
#define OTA_SER_BUSY                (ZSuccess+2)

/*********************************************************************
 * MACROS
 */
#define IR_RX                        0x01  //����ѧϰ����
#define IR_TX                        0x02  //���ⷢ������
#define IR_ESC                       0x03
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
