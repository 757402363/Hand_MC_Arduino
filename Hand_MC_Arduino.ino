#include <SoftwareSerial.h> /*调用默认软串口库*/

//------------------------------------------------------------------------
/*新建对应骨骼数据通讯的SoftwareSerial对象，rx:,tx:*/
SoftwareSerial MPUSerial[12] = 
{
  /*统一初始化*/
  {20,21},{22,23},{24,25},{26,27},
  {28,29},{30,31},{32,33},{34,35},
  {36,37},{38,39},{40,41},{42,43}
  /*0x00~0x0B*/
};

//------------------------------------------------------------------------
/*定义结构体变量记录各MPU状态*/
struct struc_MPU
{
  long int Time[2];  /*软串口监听起始、终止时间*/
  /*微控制器ATmega2560内核位数为8位，int型为16位，避免数值超出取值范围，要用long型32位。*/  
  unsigned char RxBuf[8];  /*MPU6050一帧数据*/
  unsigned char RxCounter;  /*接收字节计数器*/
  int YPR_ini[3];  /*姿态初始值（即直接收到的数据）Yaw偏航角、Pitch俯仰角、Roll横滚角*/
  int YPR_dif[3];  /*姿态角度差，即与base的角度差，有方向*/
  /*发送一个角度差为2个字节16位，用int型避免发送数据出错*/
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

/*输入角度初始值和基准值，计算姿态角度差*/
int CalculateYPRdif(int YPRini,int YPRbase);

/*输入软串口号，发送姿态角度差*/
void SendYPRdif(unsigned char SSC);

/*输入软串口号，检查MPU一帧数据并判断发送*/
void CheckMPUData(unsigned char SSC);

/*输入软串口号，读取MPU数据并判断发送*/
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

      /*输入软串口号，读取MPU数据并判断发送*/
      ReadMPUData(SoftSerialCounter);
    }
    
    /*在此软串口监听计时结束时将接收字节计数器清零，避免影响下一次该软串口的读取逻辑*/
    MPU[SoftSerialCounter].RxCounter=0;
  }
}

//-------------------------------------------------------------------------
/*输入软串口号，读取MPU数据并判断发送*/
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
      /*输入串口号，检查MPU一帧数据并判断发送*/
      CheckMPUData(SSC);
    }
  }  
}

//---------------------------------------------------------------------------
/*输入软串口号，检查MPU一帧数据并判断发送*/
void CheckMPUData(unsigned char SSC)
{
  if(0xAA==MPU[SSC].RxBuf[0] && 0x55==MPU[SSC].RxBuf[7])  /*检查MPU数据帧头帧尾*/
  {
    /*存储姿态初始值（去掉小数点后两位）*/
    /*int型范围0x7FFF<36000,如果角度值保留小数点后两位，在计算姿态角度差时可能出错，所以不保留*/
    MPU[SSC].YPR_ini[0] = (MPU[SSC].RxBuf[1]<<8|MPU[SSC].RxBuf[2]) / 100;
    MPU[SSC].YPR_ini[1] = (MPU[SSC].RxBuf[3]<<8|MPU[SSC].RxBuf[4]) / 100;
    MPU[SSC].YPR_ini[2] = (MPU[SSC].RxBuf[5]<<8|MPU[SSC].RxBuf[6]) / 100;
            
    /*如果本次开启的不是hand_base对应的0号软串口，则需要蓝牙发送姿态角度差*/
    if(0!= SSC)
    {
      /*输入软串口号，发送姿态角度差*/
      SendYPRdif(SSC);          
    }
  }
}

//--------------------------------------------------------------------------
/*输入软串口号，发送姿态角度差*/
void SendYPRdif(unsigned char SSC)
{
  /*输入角度初始值和基准值,计算姿态角度差*/
  MPU[SSC].YPR_dif[0] = CalculateYPRdif( MPU[SSC].YPR_ini[0] , MPU[0].YPR_ini[0] );
  MPU[SSC].YPR_dif[1] = CalculateYPRdif( MPU[SSC].YPR_ini[1] , MPU[0].YPR_ini[1] );
  MPU[SSC].YPR_dif[2] = CalculateYPRdif( MPU[SSC].YPR_ini[2] , MPU[0].YPR_ini[2] );
  
  /*按通讯协议发送*/
  MPU[SSC].TxBuf[0] = 0xAA;
  MPU[SSC].TxBuf[1] = SSC;
  MPU[SSC].TxBuf[2] = MPU[SSC].YPR_dif[0] >> 8;
  MPU[SSC].TxBuf[3] = MPU[SSC].YPR_dif[0] & 0xFF;
  MPU[SSC].TxBuf[4] = MPU[SSC].YPR_dif[1] >> 8;
  MPU[SSC].TxBuf[5] = MPU[SSC].YPR_dif[1] & 0xFF;              
  MPU[SSC].TxBuf[6] = MPU[SSC].YPR_dif[2] >> 8;
  MPU[SSC].TxBuf[7] = MPU[SSC].YPR_dif[2] & 0xFF;
  MPU[SSC].TxBuf[8] = 0x55;
  for(TxCounter=0; TxCounter<9; TxCounter++)
  {
    Serial.write(MPU[SSC].TxBuf[TxCounter]); 
  }      
}

//--------------------------------------------------------------------------
/*输入角度初始值和基准值，计算姿态角度差。初始姿态值范围为0->180、-180->0*/
int CalculateYPRdif(int YPRini,int YPRbase)
{
  if(YPRini<0 && YPRini>=(-180)) YPRini += 360;  /*-180~0映射到180~360*/
  int YPRdif = YPRini - YPRbase;  /*定义姿态角度差变量*/

  /*如果姿态角差值（记为A）的绝对值超过180，说明实际经过的姿态角度差为360-A的绝对值，其方向（即正负）与A相反*/
  if(abs(YPRdif)>180)
  {
    int temp_YPRdif = 360 - abs(YPRdif);
    if(YPRdif>=0) YPRdif = 0 - temp_YPRdif;
    else YPRdif = temp_YPRdif;
  }
  
  return YPRdif;
}

//-------------------------------------------------------------------------
