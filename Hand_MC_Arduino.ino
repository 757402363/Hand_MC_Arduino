#include <SoftwareSerial.h> /*调用默认软串口库*/

//------------------------------------------------------------------------
/*新建对应骨骼数据通讯的SoftwareSerial对象，rx:,tx:*/
SoftwareSerial MPUSerial[12] = 
{
  /*统一初始化*/
  {10,14},{11,15},{12,16},{13,17},
  {50,46},{51,47},{52,48},{53,49},
  {A8,A0},{A9,A1},{A10,A2},{A11,A3}
};

//------------------------------------------------------------------------
/*定义结构体变量记录各MPU状态*/
struct struc_MPU
{
  long int Time[2];  /*软串口监听起始、终止时间*/
  /*微控制器ATmega2560内核位数为8位，int型为16位，避免数值超出取值范围，要用long型32位。*/  
  unsigned char RxBuf[8];  /*MPU6050一帧数据*/
  unsigned char RxCounter;  /*接收字节计数器,每个软串口接收字节的计数都是独立的，涉及逻辑，且需要在关闭软串口监听时清零，所以放在结构体中*/
  unsigned char TxBuf[9];  /*通过蓝牙发送给UE端的一帧数据*/
}MPU[12];  

//-------------------------------------------------------------------------
/*每个软串口计时监听10ms获取一帧姿态数据*/
int SoftSerialTiming = 10; 

/*软串口计数器*/
unsigned char SoftSerialCounter;

/*发送字节计数器*/
int TxCounter;

//-------------------------------------------------------------------------
/*函数声明*/
/*按前置声明顺序*/

/*输入软串口号，检查MPU一帧数据并转发PC*/
void CheckMPUDataSend(unsigned char SSC);

/*输入软串口号，读取MPU数据*/
void ReadMPUData(unsigned char SSC);

//-------------------------------------------------------------------------
/*设置*/
void setup() 
{
  /*开启硬串口*/  
  Serial.begin(115200);

  /*开启软串口，各MPU进入自动模式*/
  for(SoftSerialCounter=0 ; SoftSerialCounter<12 ; SoftSerialCounter++)
  {
    MPUSerial[SoftSerialCounter].begin(115200);
    MPUSerial[SoftSerialCounter].listen();/*开启当前软串口监听再发送指令*/
    MPUSerial[SoftSerialCounter].write(0xA5);
    MPUSerial[SoftSerialCounter].write(0x52);
  }
}

//-------------------------------------------------------------------------
/*主体循环，类似int main{ while(1){loop;} return 0;} */
void loop() 
{
  /*12个MPU对应软串口循环开启监听*/
  for(SoftSerialCounter=0 ; SoftSerialCounter<12 ; SoftSerialCounter++)
  {
    /*开启软串口监听，并清空软串口*/
    MPUSerial[SoftSerialCounter].listen();
    MPUSerial[SoftSerialCounter].flush();
  
    /*软串口计时监听*/
    MPU[SoftSerialCounter].Time[0]=millis();
    while( (MPU[SoftSerialCounter].Time[0]+SoftSerialTiming) > MPU[SoftSerialCounter].Time[1] )
    {
      MPU[SoftSerialCounter].Time[1]=millis();

      /*输入软串口号，读取MPU数据*/
      ReadMPUData(SoftSerialCounter);
    }
    
    /*在此软串口监听计时结束时将接收字节计数器清零，避免影响下一次该软串口的读取逻辑*/
    MPU[SoftSerialCounter].RxCounter=0;
  }
}

//-------------------------------------------------------------------------
/*输入软串口号，读取MPU数据*/
void ReadMPUData(unsigned char SSC)
{
  /*软串口缓冲区有数据时读取*/
  if(MPUSerial[SSC].available()) 
  {
    MPU[SSC].RxBuf[MPU[SSC].RxCounter]=(unsigned char)MPUSerial[SSC].read();
     
    /* 检查MPU数据帧头，正确则RxCounter开始计数*/  
    if(0==MPU[SSC].RxCounter && 0xAA!=MPU[SSC].RxBuf[0]);             
    else MPU[SSC].RxCounter++;
 
    /*从软串口缓冲区读取并存储了以0xAA开头的8个字节*/
    if(8 == MPU[SSC].RxCounter)
    {
      MPU[SSC].RxCounter=0;
      /*输入串口号，检查MPU一帧数据并转发PC*/
      CheckMPUDataSend(SSC);
    }
  }  
}

//---------------------------------------------------------------------------
/*输入软串口号，检查MPU一帧数据并转发PC*/
void CheckMPUDataSend(unsigned char SSC)
{
  if(0xAA==MPU[SSC].RxBuf[0] && 0x55==MPU[SSC].RxBuf[7])  /*检查MPU数据帧头帧尾*/
  {
    /*按通讯协议发送*/
    MPU[SSC].TxBuf[0] = 0xAA;
    MPU[SSC].TxBuf[1] = SSC;
    MPU[SSC].TxBuf[2] = MPU[SSC].RxBuf[1];
    MPU[SSC].TxBuf[3] = MPU[SSC].RxBuf[2];
    MPU[SSC].TxBuf[4] = MPU[SSC].RxBuf[3];
    MPU[SSC].TxBuf[5] = MPU[SSC].RxBuf[4];              
    MPU[SSC].TxBuf[6] = MPU[SSC].RxBuf[5];
    MPU[SSC].TxBuf[7] = MPU[SSC].RxBuf[6];
    MPU[SSC].TxBuf[8] = 0x55;
    for(TxCounter=0; TxCounter<9; TxCounter++)
    {
     Serial.write(MPU[SSC].TxBuf[TxCounter]);
    }
  }
}
