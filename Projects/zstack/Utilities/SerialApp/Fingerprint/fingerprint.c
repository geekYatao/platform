/**************************************************************************************************
  Filename:       SerialApp.c
  Revised:        $Date: 2009-03-29 10:51:47 -0700 (Sun, 29 Mar 2009) $
  Revision:       $Revision: 19585 $

  Description -   Serial Transfer Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

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

#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "hal_drivers.h"
#include "hal_key.h"
#if defined ( LCD_SUPPORTED )
  #include "hal_lcd.h"
#endif
#include "hal_led.h"
#include "hal_uart.h"
#include "Public.h"
#include "fingerprint.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_57600
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  64
#endif
  
#if !defined( SERIAL_APP_RX_SZ )
#define SERIAL_APP_RX_SZ  128
#endif

#if !defined( SERIAL_APP_TX_SZ )
#define SERIAL_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SERIAL_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SERIAL_APP_LOOPBACK )
#define SERIAL_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  20
#endif

#define SERIAL_APP_RSP_CNT  4

// This list should be filled with Application specific Cluster IDs.
const cId_t SerialApp_ClusterList[SERIALAPP_MAX_CLUSTERS] =
{
  SERIALAPP_CLUSTERID1,
  SERIALAPP_CLUSTERID2
};

const SimpleDescriptionFormat_t SerialApp_SimpleDesc =
{
  SERIALAPP_ENDPOINT,              //  int   Endpoint;
  SERIALAPP_PROFID,                //  uint16 AppProfId[2];
  SERIALAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SERIALAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SERIALAPP_FLAGS,                 //  int   AppFlags:4;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SerialApp_ClusterList,  //  byte *pAppInClusterList;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)SerialApp_ClusterList   //  byte *pAppOutClusterList;
};

const endPointDesc_t SerialApp_epDesc =
{
  SERIALAPP_ENDPOINT,
 &SerialApp_TaskID,
  (SimpleDescriptionFormat_t *)&SerialApp_SimpleDesc,
  noLatencyReqs
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.
devStates_t  SampleApp_NwkState;
static UART_Format UART0_Format;
unsigned char Flag=0;
unsigned char Flag_1=0;
unsigned char Search_Flag=0;
unsigned char Position=0xff;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 SerialApp_MsgID;
static afAddrType_t SerialApp_TxAddr;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX];
static uint8 SerialApp_TxLen;
static uint8 Device_type = DeviceType_none;
static char Busy_flag = 1;//kongxian
uint8 oldCMD;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
void SerialApp_OTAData(afAddrType_t *txaddr,uint8 ID,void *p,uint8 len);
static void SerialApp_CallBack(uint8 port, uint8 event);

/*********************************************************************
 * @fn      SerialApp_Init
 *
 * @brief   This is called during OSAL tasks' initialization.
 *
 * @param   task_id - the Task ID assigned by OSAL.
 *
 * @return  none
 */
void SerialApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;

  SerialApp_TaskID = task_id;
  afRegister( (endPointDesc_t *)&SerialApp_epDesc );
  RegisterForKeys( task_id );

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;//SerialApp_CallBack
  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);
  
  UART0_Format.Header_1 = 0xee;
  UART0_Format.Header_2 = 0xcc;
  UART0_Format.NodeSeq  = 0x01;
  UART0_Format.NodeID   = Fingerprint;
  UART0_Format.Tailer   = 0xff;
  
  SerialApp_TxAddr.addrMode =(afAddrMode_t)Addr16Bit;//发送地址初始化
  SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
  SerialApp_TxAddr.addr.shortAddr = 0x0000;
  TXPOWER = 0xf5;
  P0DIR |= 0x01;
}

/*********************************************************************
 * @fn      SerialApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events   - Bit map of events to process.
 *
 * @return  Event flags of all unprocessed events.
 */
UINT16 SerialApp_ProcessEvent( uint8 task_id, UINT16 events )
{
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    afIncomingMSGPacket_t *MSGpkt;

    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SerialApp_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {         
      case KEY_CHANGE:
        //SerialApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;

      case AF_INCOMING_MSG_CMD:
        SerialApp_ProcessMSGCmd( MSGpkt );
        break;

      case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if(SampleApp_NwkState == DEV_END_DEVICE)       //判定当前设备类型
          {
            Device_type = 2;
            HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
            HalLedBlink(HAL_LED_2,5,50,500);           
            osal_set_event(SerialApp_TaskID, PERIOD_EVT); //启动周期消息
            osal_set_event(SerialApp_TaskID, START_EVT);
            osal_set_event(SerialApp_TaskID, TEMPLATENUM_EVT);
          }
        break;
      default:
        break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    return ( events ^ SYS_EVENT_MSG );
  }
  
  if ( events & PERIOD_EVT ) //周期消息处理
  {
    uint16 my_nwk_addr = 0xffff;
    uint16 parent_nwk_addr = 0xffff;
    my_nwk_addr = NLME_GetShortAddr();
    parent_nwk_addr = NLME_GetCoordShortAddr();
    UART0_Format.Command = MSG_PERIOD;
    UART0_Format.Data[0] = 0x00; //信号强度,不用处理
    UART0_Format.Data[1] = Device_type; //节点类型  协调时=0 路由=1 终端节点=2
    UART0_Format.Data[2] = my_nwk_addr>>8; //节点自身网络地址 高字节
    UART0_Format.Data[3] = my_nwk_addr; //节点自身网络地址 低字节
    UART0_Format.Data[4] = parent_nwk_addr>>8; //父节点网络地址 高字节
    UART0_Format.Data[5] = parent_nwk_addr; //父节点身网络地址 低字节
    SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    if((Busy_flag)&&(oldCMD != 0X01))
    {
                Search_Flag=1;
            osal_set_event(SerialApp_TaskID, IMAGE1_EVT);   
    }
    osal_start_timerEx(SerialApp_TaskID, PERIOD_EVT, 5000);
    return ( events ^ PERIOD_EVT );
  }

  if ( events & SERIALAPP_SEND_EVT )  //将串口数据通过RF消息发送
  {
    SerialApp_OTAData(&SerialApp_TxAddr,SERIALAPP_CLUSTERID1,SerialApp_TxBuf, sizeof(UART_Format));
    return ( events ^ SERIALAPP_SEND_EVT );
  }
  
  if ( events & TEMPLATENUM_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x1d,0x00,0x21};
     Flag_1 = 1;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  } 
  
  if ( events & EMPTY_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x0d,0x00,0x11};
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }  
  
  if ( events & START_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x07,0x13,0x00,0x00,0x00,0x00,0x00,0x1b};
     Flag=1;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }  
  
  if ( events & IMAGE1_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x01,0x00,0x05};
     Flag=2;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    

     return 0;
  }
  
  if ( events & CHAR1_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x04,0x02,0x01,0x00,0x08};
     Flag=3;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }  
  
  if ( events & IMAGE2_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x01,0x00,0x05};
     Flag=4;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }  
  
  if ( events & CHAR2_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x04,0x02,0x02,0x00,0x09};
     Flag=5;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }

  if ( events & MATCH_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x03,0x00,0x07};
     Flag=6;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }

  if ( events & MODEL_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x05,0x00,0x09};
     Flag=7;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }

  if ( events & STORE_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x06,0x06,0x01,0x00,0xff,0x00,0x00};
     buf[12]=Position;
     buf[14]=Position+14;
     Flag=8;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  } 
  
  if ( events & SEARCH_EVT ) 
  {
     uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x08,0x04,0x01,0x00,0x00,0x00,0xFF,0x01,0x0D};
     Flag=9;
     HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));    
     return 0;
  }
  
  return ( 0 );  // Discard unknown events.
}

/*********************************************************************
 * @fn      SerialApp_ProcessMSGCmd
 *
 * @brief   Data message processor callback. This function processes
 *          any incoming data - probably from other devices. Based
 *          on the cluster ID, perform the intended action.
 *
 * @param   pkt - pointer to the incoming message packet
 *
 * @return  TRUE if the 'pkt' parameter is being used and will be freed later,
 *          FALSE otherwise.
 */
void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )  //处理接收到的RF消息
{
  static UART_Format *receiveData;
/*  static UART_Format Rsp;
  Rsp.Header_1 = 0xee;
  Rsp.Header_2 = 0xcc;
  Rsp.NodeSeq  = 0x01;
  Rsp.NodeID   = Breath;
  Rsp.Command  = MSG_RSP;
  Rsp.Tailer   = 0xff;*/
  switch ( pkt->clusterId )
  {
   case SERIALAPP_CLUSTERID1:  //处理各个传感器节数据    
     receiveData = (UART_Format *)(pkt->cmd.Data);
     HalLedBlink(HAL_LED_1,1,50,200);
     if((receiveData->Header_1==0xcc)&&(receiveData->Header_2==0xee)&&(receiveData->Tailer==0xff)) //校验包头包尾
     {
       if(receiveData->NodeID == Fingerprint) //地址
       {
         Busy_flag = 0;//设置0为忙，不会被打断
         oldCMD =  receiveData->Command;
         if(receiveData->Command == OPEN)
         {
                 LOCK;
                 MicroWait(60000);
                 ULOCK;      
         
         }
         if(receiveData->Command == FINGERINPUT)
         {
//           Position = receiveData->Data[0];
            Position=Position+1;//position has initialed in state change.
            if(Position>=162)  Position=162;
            osal_set_event(SerialApp_TaskID, IMAGE1_EVT);         
         }
         else if(receiveData->Command == FINGERSEARCH)
         {
            Search_Flag=1;
            osal_set_event(SerialApp_TaskID, IMAGE1_EVT);  
                  Busy_flag = 1;
         }  
         else if(receiveData->Command == DELETECHAR)
         {
         
         }
         else if(receiveData->Command == EMPTY)
         {
            osal_set_event(SerialApp_TaskID, EMPTY_EVT);
         }         
        
       }
     }
                                
    break;

  case SERIALAPP_CLUSTERID2:
    break;

    default:
      break;
  }
}

/*********************************************************************
 */

void SerialApp_OTAData(afAddrType_t *txaddr, uint8 cID, void *p, uint8 len) //发送函数
{
  if (afStatus_SUCCESS != AF_DataRequest(txaddr, //发送地址
                                           (endPointDesc_t *)&SerialApp_epDesc, //endpoint描述
                                            cID, //clusterID
                                            len, p, //发送数据包的长度和地址
                                            &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
  {
  }
  else
  {
    HalLedBlink(HAL_LED_1,2,50,200);
  }
}

/*********************************************************************
 * @fn      SerialApp_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void SerialApp_CallBack(uint8 port, uint8 event)
{
//  static uint8 cnt = 0;
  (void)port;
  UART_Format Rsp;
  Rsp.Header_1 = 0xee;
  Rsp.Header_2 = 0xcc;
  Rsp.NodeSeq  = 0x01;
  Rsp.NodeID   = Fingerprint;
  Rsp.Command  = MSG_RSP;
  Rsp.Tailer   = 0xff;  
  
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
      (SerialApp_TxLen < SERIAL_APP_TX_MAX))
#else
      !SerialApp_TxLen)
#endif
  {
    SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX); //将串口数据读入buf
    if(SerialApp_TxLen  > 0)
    {
      if(Flag_1 == 1)    
      {
         Flag_1=0;
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 Position=SerialApp_TxBuf[11]-1;
                 Rsp.Data[1] = SerialApp_TxBuf[11];
              }
              else
              {
                osal_set_event(SerialApp_TaskID,TEMPLATENUM_EVT);
                Rsp.Data[1] = 0XFF;
              }
          }
         Rsp.Data[0] = TEMPLATENUM;
         SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, sizeof(UART_Format));
       }      
      if(Flag == 1)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]!=0x00)
              {
                 osal_set_event(SerialApp_TaskID, START_EVT);
              }
          }
       }

      if(Flag == 2)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 osal_set_event(SerialApp_TaskID, CHAR1_EVT);
              }
              else
              {
                osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
              }
          }
         else
         {
              osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
          }
       }      
      
      if(Flag == 3)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                if(Search_Flag==1)
                {
                  osal_set_event(SerialApp_TaskID, SEARCH_EVT);
                }
                else
                {
                  osal_start_timerEx(SerialApp_TaskID, IMAGE2_EVT, 2500);
                }                  
              }
              else
              {
//                osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
                osal_start_timerEx(SerialApp_TaskID, IMAGE1_EVT, 2500);                
              }
          }
         else
         {
//              osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
                osal_start_timerEx(SerialApp_TaskID, IMAGE1_EVT, 2500);           
          }
       }       
      

      if(Flag == 4)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 osal_set_event(SerialApp_TaskID, CHAR2_EVT);
              }
              else
              {
                osal_set_event(SerialApp_TaskID, IMAGE2_EVT);
              }
          }
         else
         {
              osal_set_event(SerialApp_TaskID, IMAGE2_EVT);
          }
       }

      if(Flag == 5)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 osal_set_event(SerialApp_TaskID, MATCH_EVT);
              }
              else
              {
                osal_set_event(SerialApp_TaskID, CHAR2_EVT);
              }
          }
         else
         {
              osal_set_event(SerialApp_TaskID, CHAR2_EVT);
          }
       }
      
      //match      
      if(Flag == 6)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 osal_set_event(SerialApp_TaskID, MODEL_EVT);
              }
              else
              {
                osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
              }
          }
         else
         {
              osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
          }
       }      
      
      if(Flag == 7)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 osal_set_event(SerialApp_TaskID, STORE_EVT);
              }
              else
              {
                osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
              }
          }
         else
         {
              osal_set_event(SerialApp_TaskID, IMAGE1_EVT);
          }
       }       

      if(Flag == 8)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 Rsp.Data[1] = RIGHT;
              }
              else
              {
                Rsp.Data[1] = WRONG;
              }
          }
         else
         {
              Rsp.Data[1] = WRONG;
          }
         Rsp.Data[0] = FINGERINPUT;
         SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, sizeof(UART_Format));
//         Rsp.Data[0] = 0;
       }
 //search finger     
      if(Flag == 9)
      {
         if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
          {
              if(SerialApp_TxBuf[9]==0x00)
              {
                 LOCK;
                 //delay(1000);
                 
                 Rsp.Data[1] = RIGHT;
                 Rsp.Data[2] = SerialApp_TxBuf[11];
                 MicroWait(60000);
                 ULOCK;
              }
              else
              {
                ULOCK;
                Rsp.Data[1] = WRONG;
              }
          }
         else
         {
              Rsp.Data[1] = WRONG;
          }
         Search_Flag=0;  //否则执行一次搜索后再执行输入指纹会出错
         Rsp.Data[0] = FINGERSEARCH;
         SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, sizeof(UART_Format));
//         Rsp.Data[0] = 0;
       }      
    SerialApp_TxLen = 0; 
  }
  }
}

/*********************************************************************
*********************************************************************/





/*         if(receiveData->Command == START) //
         {
            unsigned char cnt=0;
            uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x07,0x13,0x00,0x00,0x00,0x00,0x00,0x1b};
            HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));
            
          do{
               SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
               if(SerialApp_TxLen)
                {
                    if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                    {
                          if((SerialApp_TxBuf[9]==0x00)&&(SerialApp_TxBuf[11]==0x0a))//EF 01 FF FF FF FF 07 00 03 00 00 0A
                          {
                            Rsp.Data[1] = RIGHT;
                          }
                          else
                          {
                            Rsp.Data[1] = WRONG;
                          }
                     }                   
                }
                cnt++;
               
            }while((!SerialApp_TxLen)&&(cnt<5));
            SerialApp_TxBuf[0]=0;
            SerialApp_TxLen = 0;
            Rsp.Data[0] = START;            
         }        
         else if(receiveData->Command == FINGERINPUT) //停止采集呼吸波ef 01 ff ff ff ff 01 00 03 01 00 05
         {
           生成特征1
          do{
                unsigned char cnt=0;
//                unsigned char cnt2=0;
                uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x01,0x00,0x05};
                HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf)); 
                HalLedBlink(HAL_LED_1,2,50,500);
                do{
                     SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
                     if(SerialApp_TxLen)
                      {
                          if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                          {
                                if((SerialApp_TxBuf[9]==0x00)&&(SerialApp_TxBuf[11]==0x0a))//EF 01 FF FF FF FF 07 00 03 00 00 0A
                                {
                                    Flag = 10;
                                }
                           }                   
                      }
                      cnt++;                 
                  }while((!SerialApp_TxLen)&&(cnt<5));
                  SerialApp_TxBuf[0]=0;
                  SerialApp_TxLen = 0;
                  HalLedBlink(HAL_LED_2,2,50,500);
//                  cnt2++;
           }while(Flag != 10);//
              
              if(Flag == 10)
              {
                  unsigned char cnt=0;
                  uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x04,0x02,0x01,0x00,0x08};
                  HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));           
                  do{
                       SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
                       if(SerialApp_TxLen)
                        {
                            if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                            {
                                  if((SerialApp_TxBuf[9]==0x00)&&(SerialApp_TxBuf[11]==0x0a))//
                                  {
                                      Flag = 2;
                                  }
                             }                   
                        }
                        cnt++;                 
                    }while((!SerialApp_TxLen)&&(cnt<5)); 
                  SerialApp_TxLen = 0;
              }
              
              生成特征2
              if(Flag == 2)
              {
                do{
                    unsigned char cnt=0;
                    uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x01,0x00,0x05};
                    HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));           
                    do{
                         SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
                         if(SerialApp_TxLen)
                          {
                              if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                              {
                                    if((SerialApp_TxBuf[9]==0x00)&&(SerialApp_TxBuf[11]==0x0a))//
                                    {
                                        Flag = 3;
                                    }
                               }                   
                          }
                          cnt++;                 
                      }while((!SerialApp_TxLen)&&(cnt<3));
                    SerialApp_TxLen = 0;
                }while(Flag != 3);
              }
              
              if(Flag == 3)
              {
                  unsigned char cnt=0;
                  uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x04,0x02,0x02,0x00,0x09};
                  HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));           
                  do{
                       SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
                       if(SerialApp_TxLen)
                        {
                            if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                            {
                                  if((SerialApp_TxBuf[9]==0x00)&&(SerialApp_TxBuf[11]==0x0a))//
                                  {
                                      Flag = 4;
                                  }
                             }                   
                        }
                        cnt++;                 
                    }while((!SerialApp_TxLen)&&(cnt<3));
                  SerialApp_TxLen = 0;
              }
              
              特征比对
              if(Flag == 4)
              {
                  unsigned char cnt=0;
                  uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x03,0x00,0x07};
                  HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));           
                  do{
                       SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
                       if(SerialApp_TxLen)
                        {
                            if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                            {
                                  if(SerialApp_TxBuf[9]==0x00)//
                                  {
                                      Flag = 5;
                                  }
                             }                   
                        }
                        cnt++;                 
                    }while((!SerialApp_TxLen)&&(cnt<3)); 
                  SerialApp_TxLen = 0;
              }              
              
              
              特征合并成模板
              if(Flag == 5)
              {
                  unsigned char cnt=0;
                  uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x05,0x00,0x09};
                  HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));           
                  do{
                       SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
                       if(SerialApp_TxLen)
                        {
                            if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                            {
                                  if(SerialApp_TxBuf[9]==0x00)//
                                  {
                                      Flag = 6;
                                  }
                             }                   
                        }
                        cnt++;                 
                    }while((!SerialApp_TxLen)&&(cnt<3));
                  SerialApp_TxLen = 0;
              }              
              
              存储模板
              if(Flag == 6)
              {
                  unsigned char cnt=0;
                  uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x06,0x01,0x00,0x07,0x00,0x0f};
                  HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf));           
                  do{
                       SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX);
                       if(SerialApp_TxLen)
                        {
                            if((SerialApp_TxBuf[0]==0xEF)&&(SerialApp_TxBuf[1]==0x01)&&(SerialApp_TxBuf[6]==0x07))//包头包尾校验
                            {
                                  if(SerialApp_TxBuf[9]==0x00)//
                                  {
                                    Rsp.Data[1] = RIGHT;
                                  }
                                  else
                                  {
                                    Rsp.Data[1] = WRONG;
                                  }
                             }                   
                        }
                        cnt++;                 
                    }while((!SerialApp_TxLen)&&(cnt<3)); 
                  SerialApp_TxLen = 0;
               }              
              
                                        
            Rsp.Data[0] = FINGERINPUT; 

         }
         else if(receiveData->Command == FINGERSEARCH)  
         {

            uint8 buf[] = {0xef,0x01,0xff,0xff,0xff,0xff,0x01,0x00,0x03,0x0f,0x00,0x13};
            HalUARTWrite(SERIAL_APP_PORT, buf, sizeof(buf)); 
              
            
 //           Rsp.Data[0] = PA;       
//            Rsp.Data[1] = receiveData->Data[0];  
         }
         SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, sizeof(UART_Format));
         Rsp.Data[1] = 0X00;
       }
     }*/










