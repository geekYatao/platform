#ifndef PUBLIC_H
#define PUBLIC_H

#ifdef __cplusplus
extern "C"
{
#endif

#define  MSG_PERIOD    0xAA
#define  MSG_RSP       0xDD
  
typedef enum 
{
  DeviceType_coor = 0,
  DeviceType_router,
  DeviceType_end,
  DeviceType_none,
}DeviceType;
  
typedef enum
{
  Coor = 0,   //����
  Hall,       //����
  PhotoRes,   //����
  TempAndHum, //��ʪ��
  Shake,      //��
  Reed,       //�ɻɹ�
  Accele,     //���ٶ�
  Smoke,      //����
  Doppler,    //������
  Motor,      //���
  LED_PWM,    //LED���� a
  Sound,      //���� b
  Voltage,    //��ѹ  ���� c
  Current,    //����  ���� d
  Touch,      //���� e
  Ultrasound, //������ f
  RFID_1356,  //13.56M��Ƶ��   �ƶ�֧��
  RFID_125K,  //125K��Ƶ�� 11
  Flame,      //���� 12
  Particle,   //΢�� 13
  Color,      //��ɫ 14
  Gyroscope,  //������ 15
  IR_Code,    //�������� 16   ����
  Alcohol,     //�ƾ� 17
  Relay,      //�̵��� 18
  RFID_900M,  //����ƵRFID 19
  Router_LED, //LED���б����� 1a
  Press,      //ѹ�� 1b
  Co2,        //������̼ 1c
  Ir_sw,      //��翪�� 1d
  Ir_safe,    //������� 1e    
  LED_Screen, //LED��Ļ 1f
  AlarmLamp,  //������ 20
  Ch3,       //��ȩ 21
  Window,     //���� 22
  SmartLamp,  //���ܵ� 23
  Humidifier, //��ʪ�� 24
  TableLamp,  //̨�� 25
  Fan,        //���� 26
  Asr,        //����ʶ��
  LampHolder_1,     //���ص�ͷ1
  LampHolder_2,     //���ص�ͷ2
  LampHolder_3,     //���ص�ͷ3
  LampHolder_4,     //���ص�ͷ4
  SmartLamp1,  //���ܵ� 2C
  PWM_AC,    //�����1  2d
  PWM_AC1,   //�����2  2e
  PWM_AC2, // 2f
  PWM_AC3, // 30
  ADXL345, // 31
  RFID_1356_id,  //13.56M��Ƶ��  ʶ�� 32
  Fingerprint,  //ָ�� 33
  IR_Air, //�������� �յ� 34
  IR_Fan, //�������� ���� 35 
  Gas  //ȼ��36
}DeviceAddrList;

typedef struct
{
  uint8 Header_1;
  uint8 Header_2;
  uint8 NodeSeq;
  uint8 NodeID;
  uint8 Command;
  uint8 Data[10];
  uint8 Tailer;
}UART_Format;


#ifdef __cplusplus
}
#endif

#endif 
