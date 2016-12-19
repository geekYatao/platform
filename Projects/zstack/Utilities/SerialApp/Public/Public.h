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
  Coor = 0,   //网关
  Hall,       //霍尔
  PhotoRes,   //光照
  TempAndHum, //温湿度
  Shake,      //震动
  Reed,       //干簧管
  Accele,     //加速度
  Smoke,      //烟雾
  Doppler,    //多普勒
  Motor,      //电机
  LED_PWM,    //LED调光 a
  Sound,      //声音 b
  Voltage,    //电压  保留 c
  Current,    //电流  保留 d
  Touch,      //触摸 e
  Ultrasound, //超声波 f
  RFID_1356,  //13.56M射频卡   移动支付
  RFID_125K,  //125K射频卡 11
  Flame,      //火焰 12
  Particle,   //微粒 13
  Color,      //颜色 14
  Gyroscope,  //陀螺仪 15
  IR_Code,    //红外编解码 16   电视
  Alcohol,     //酒精 17
  Relay,      //继电器 18
  RFID_900M,  //超高频RFID 19
  Router_LED, //LED集中报警灯 1a
  Press,      //压力 1b
  Co2,        //二氧化碳 1c
  Ir_sw,      //光电开关 1d
  Ir_safe,    //红外对射 1e    
  LED_Screen, //LED屏幕 1f
  AlarmLamp,  //报警灯 20
  Ch3,       //甲醛 21
  Window,     //窗帘 22
  SmartLamp,  //智能灯 23
  Humidifier, //加湿器 24
  TableLamp,  //台灯 25
  Fan,        //风扇 26
  Asr,        //语音识别
  LampHolder_1,     //开关灯头1
  LampHolder_2,     //开关灯头2
  LampHolder_3,     //开关灯头3
  LampHolder_4,     //开关灯头4
  SmartLamp1,  //智能灯 2C
  PWM_AC,    //调光灯1  2d
  PWM_AC1,   //调光灯2  2e
  PWM_AC2, // 2f
  PWM_AC3, // 30
  ADXL345, // 31
  RFID_1356_id,  //13.56M射频卡  识别 32
  Fingerprint,  //指纹 33
  IR_Air, //红外编解码 空调 34
  IR_Fan, //红外编解码 风扇 35 
  Gas  //燃气36
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
