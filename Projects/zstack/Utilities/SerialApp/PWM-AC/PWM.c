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
#include "PWM.h"

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
#define SERIAL_APP_BAUD  HAL_UART_BR_115200
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
static uint8 Device_type = DeviceType_none;

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

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
void SerialApp_OTAData(afAddrType_t *txaddr,uint8 ID,void *p,uint8 len);
static void SerialApp_CallBack(uint8 port, uint8 event);
static void Init_T1(void);

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
  
  Init_T1();

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);
  
  UART0_Format.Header_1 = 0xee;
  UART0_Format.Header_2 = 0xcc;
  UART0_Format.NodeSeq  = 0x01;
  UART0_Format.NodeID   = PWM_AC;
  UART0_Format.Tailer   = 0xff;
  
  SerialApp_TxAddr.addrMode =(afAddrMode_t)Addr16Bit;//���͵�ַ��ʼ��
  SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
  SerialApp_TxAddr.addr.shortAddr = 0x0000;
  TXPOWER = 0xf5;
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
          if(SampleApp_NwkState == DEV_END_DEVICE)       //�ж���ǰ�豸����
          {
            Device_type = 2;
            HalLedBlink(HAL_LED_1,1,50,500);
            HalLedBlink(HAL_LED_2,1,50,500);
            osal_set_event(SerialApp_TaskID, PERIOD_EVT); //����������Ϣ
          }
        break;
      default:
        break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    return ( events ^ SYS_EVENT_MSG );
  }
  
  if ( events & PERIOD_EVT ) //������Ϣ����
  {
    uint16 my_nwk_addr = 0xffff;
    uint16 parent_nwk_addr = 0xffff;
    my_nwk_addr = NLME_GetShortAddr();
    parent_nwk_addr = NLME_GetCoordShortAddr();
    UART0_Format.Command = MSG_PERIOD;
    UART0_Format.Data[0] = 0x00; //�ź�ǿ��,���ô���
    UART0_Format.Data[1] = Device_type; //�ڵ�����  Э��ʱ=0 ·��=1 �ն˽ڵ�=2
    UART0_Format.Data[2] = my_nwk_addr>>8; //�ڵ����������ַ ���ֽ�
    UART0_Format.Data[3] = my_nwk_addr; //�ڵ����������ַ ���ֽ�
    UART0_Format.Data[4] = parent_nwk_addr>>8; //���ڵ������ַ ���ֽ�
    UART0_Format.Data[5] = parent_nwk_addr; //���ڵ��������ַ ���ֽ�
    SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    osal_start_timerEx(SerialApp_TaskID, PERIOD_EVT, 5000);
    return ( events ^ PERIOD_EVT );
  }

  if ( events & SERIALAPP_SEND_EVT )  //����������ͨ��RF��Ϣ����
  {
    SerialApp_OTAData(&SerialApp_TxAddr,SERIALAPP_CLUSTERID1,SerialApp_TxBuf, SerialApp_TxLen);
    return ( events ^ SERIALAPP_SEND_EVT );
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
void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )  //�������յ���RF��Ϣ
{
  static UART_Format *receiveData;
  static UART_Format Rsp;
  Rsp.Header_1 = 0xee;
  Rsp.Header_2 = 0xcc;
  Rsp.NodeSeq  = 0x01;
  Rsp.NodeID   = PWM_AC;
  Rsp.Command  = MSG_RSP;
  Rsp.Tailer   = 0xff;
  switch ( pkt->clusterId )
  {
     case SERIALAPP_CLUSTERID1:  //��������������������    
     receiveData = (UART_Format *)(pkt->cmd.Data);
     HalLedBlink(HAL_LED_1,1,50,200);
     if((receiveData->Header_1==0xcc)&&(receiveData->Header_2==0xee)&&(receiveData->Tailer==0xff)) //У���ͷ��β
     {
       if(receiveData->NodeID == PWM_AC) //��ַ
       {
         switch (receiveData->Command)
         {
          /*case PWM_9:  //WSN
            T1CC1L   = 500%256; 
            T1CC1H   = 500/256;
               break;
           case PWM_8:
            T1CC1L   = 1000%256; 
            T1CC1H   = 1000/256;             
               break;
           case PWM_7:
            T1CC1L   = 1600%256; 
            T1CC1H   = 1600/256;             
               break;   
           case PWM_6:
            T1CC1L   = 2200%256; 
            T1CC1H   = 2200/256;             
               break;
           case PWM_5:
            T1CC1L   = 2800%256; 
            T1CC1H   = 2800/256;             
               break;
           case PWM_4:
            T1CC1L   = 3400%256; 
            T1CC1H   = 3400/256;             
               break;
           case PWM_3:
            T1CC1L   = 4000%256; 
            T1CC1H   = 4000/256;
               break;
           case PWM_2:
             T1CC1L   = 4600%256; 
             T1CC1H   = 4600/256;   
               break;
           case PWM_1:
             T1CC1L   = 5000%256; 
             T1CC1H   = 5000/256;             
               break;
           case PWM_0:
            T1CC1L   = 0%256; 
            T1CC1H   = 0/256;   
               break;  */
           /* case PWM_9:  //5V��3V����1��
            T1CC1L   = 5%256; 
            T1CC1H   = 5/256;
               break;
           case PWM_8:
            T1CC1L   = 200%256; 
            T1CC1H   = 200/256;             
               break;
           case PWM_7:
            T1CC1L   = 500%256; 
            T1CC1H   = 500/256;             
               break;   
           case PWM_6:
            T1CC1L   = 800%256; 
            T1CC1H   = 800/256;             
               break;
           case PWM_5:
            T1CC1L   = 1200%256; 
            T1CC1H   = 1200/256;             
               break;
           case PWM_4:
            T1CC1L   = 1600%256; 
            T1CC1H   = 1600/256;             
               break;
           case PWM_3:
            T1CC1L   = 2000%256; 
            T1CC1H   = 2000/256;
               break;
           case PWM_2:
             T1CC1L   = 2400%256; 
             T1CC1H   = 2400/256;   
               break;
           case PWM_1:
             T1CC1L   = 3000%256; 
             T1CC1H   = 3000/256;             
               break;
           case PWM_0:
            T1CC1L   = 0%256; 
            T1CC1H   = 0/256;   
               break;  */
          case PWM_9:  //12V,3V����3��
            T1CC1L   = 5%256; 
            T1CC1H   = 5/256;
               break;
           case PWM_8:
            T1CC1L   = 500%256; 
            T1CC1H   = 500/256;             
               break;
           case PWM_7:
            T1CC1L   = 1000%256; 
            T1CC1H   = 1000/256;             
               break;   
           case PWM_6:
            T1CC1L   = 1700%256; 
            T1CC1H   = 1700/256;             
               break;
           case PWM_5:
            T1CC1L   = 2400%256; 
            T1CC1H   = 2400/256;             
               break;
           case PWM_4:
            T1CC1L   = 3000%256; 
            T1CC1H   = 3000/256;             
               break;
           case PWM_3:
            T1CC1L   = 3600%256; 
            T1CC1H   = 3600/256;
               break;
           case PWM_2:
             T1CC1L   = 4200%256; 
             T1CC1H   = 4200/256;   
               break;
           case PWM_1:
             T1CC1L   = 4800%256; 
             T1CC1H   = 4800/256;             
               break;
           case PWM_0:
            T1CC1L   = 0%256; 
            T1CC1H   = 0/256;             
               break;         
         }

       }

       Rsp.Data[0] = receiveData->Command;
       SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, sizeof(UART_Format));
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

void SerialApp_OTAData(afAddrType_t *txaddr, uint8 cID, void *p, uint8 len) //���ͺ���
{
  if (afStatus_SUCCESS != AF_DataRequest(txaddr, //���͵�ַ
                                           (endPointDesc_t *)&SerialApp_epDesc, //endpoint����
                                            cID, //clusterID
                                            len, p, //�������ݰ��ĳ��Ⱥ͵�ַ
                                            &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
  {
  }
  else
  {
    HalLedBlink(HAL_LED_1,1,50,200);
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
  (void)port;

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) && !SerialApp_TxLen) //���ڽ��յ����ݰ�
  {
    SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX); //���������ݶ���buf
    SerialApp_TxLen = 0;  
  }
}

/*********************************************************************
*********************************************************************/



static void Init_T1(void)
{
      //��ʼ��T1
      P0SEL |= 0X08;//P0_3Ϊ���蹦��
      P0DIR |= 0X08;//P0_3 ���
      PERCFG &= ~0X40;//T1CFG��Ϊ0����T1��I/Oλ����λ��1����P0_2/3/4/5/6
      P2DIR &= ~0XC0;
      P2DIR |= 0X80;//very important���������ȼ�  û�еĻ��Ͳ���PWM��
      
      T1CTL   = 0x08;//ֹͣ��������32��Ƶ
      

      T1CCTL0 |= 0x04; //ͨ��0�Ƚ�ģʽ���趨T1CC0L��T1CC0H������Ϊcompareģʽ��
      //�趨ģģʽʱ������1���յ�ֵ
      T1CC0L   = 5000%256; //��ǰ200hz
      T1CC0H   = 5000/256; 

//      T1CNTL = 0XFF;
//      T1CCTL1 |= 0X7<<3;//��ʼ������Ƚ�
      
      T1CCTL1 &= ~0x3f;
      T1CCTL1 |= 0x1C; //T1ͨ��1����Ƚ�ģʽ����T1CC1��1����T1CC0��0��  1��  1W  led �����ģʽ(NO 1)1
//      T1CCTL1 |= 0x24;  // 6��  1W leds �����ģʽ(NO 2)
      
      //�趨PWM��ռ�ձ�
      T1CC1L   = 0%256; 
      T1CC1H   = 0/256;
      
      //T1 RUN
      T1CTL |= 0x02;//������������ģģʽ��0��T1CC0����������
}

/*************************************************
���� 1W led pwm ���� PT4115 3W LED PWM (NO 1)

         switch (receiveData->Command)
         {
           case PWM_9:
            T1CC1L   = 5%256; 
            T1CC1H   = 5/256;
               break;
           case PWM_8:
            T1CC1L   = 500%256; 
            T1CC1H   = 500/256;             
               break;
           case PWM_7:
            T1CC1L   = 1000%256; 
            T1CC1H   = 1000/256;             
               break;   
           case PWM_6:
            T1CC1L   = 1700%256; 
            T1CC1H   = 1700/256;             
               break;
           case PWM_5:
            T1CC1L   = 2400%256; 
            T1CC1H   = 2400/256;             
               break;
           case PWM_4:
            T1CC1L   = 3000%256; 
            T1CC1H   = 3000/256;             
               break;
           case PWM_3:
            T1CC1L   = 3600%256; 
            T1CC1H   = 3600/256;
               break;
           case PWM_2:
             T1CC1L   = 4200%256; 
             T1CC1H   = 4200/256;   
               break;
           case PWM_1:
             T1CC1L   = 4800%256; 
             T1CC1H   = 4800/256;             
               break;
           case PWM_0:
            T1CC1L   = 0%256; 
            T1CC1H   = 0/256;             
               break;             
         }

*******************************/



/*************************************************
6 leds pwm(NO 2)

         switch (receiveData->Command)
         {
         case PWM_9:
          T1CC1L   = 1%256; 
          T1CC1H   = 1/256;
             break;
         case PWM_8:
          T1CC1L   = 300%256; 
          T1CC1H   = 300/256;             
             break;
         case PWM_7:
          T1CC1L   = 900%256; 
          T1CC1H   = 900/256;             
             break;   
         case PWM_6:
          T1CC1L   = 1400%256; 
          T1CC1H   = 1400/256;             
             break;
         case PWM_5:
          T1CC1L   = 2200%256; 
          T1CC1H   = 2200/256;             
             break;
         case PWM_4:
          T1CC1L   = 3000%256; 
          T1CC1H   = 3000/256;             
             break;
         case PWM_3:
          T1CC1L   = 3800%256;          
          T1CC1H   = 3800/256;
             break;
         case PWM_2:
           T1CC1L   = 4500%256; 
           T1CC1H   = 4500/256;   
             break;
         case PWM_1:
           T1CC1L   = 5000%256; 
           T1CC1H   = 5000/256;             
             break;
         case PWM_0:
          T1CC1L   = 0%256; 
          T1CC1H   = 0/256;             
             break;             
         }

*******************************/