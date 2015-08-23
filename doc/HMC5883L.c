================== HMC5883L测试程序===============
** 3-轴数字罗盘
** 协议：IIC    详细读写协议，请参考相关资料
** 时间：2012.2.2
** Author:Keliwen
**************************************************/
#include <reg52.h>
#include <math.h>
#include <stdio.h>
#include <intrins.h>

typedef unsigned char uchar;
typedef unsigned int uint;

/*12232液晶引脚定义*/       
sbit CS=P0^2;
sbit SID=P0^1;
sbit SCLK=P0^0;

sbit SCL=P1^0;      //IIC时钟引脚定义
sbit SDA=P1^1;      //IIC数据引脚定义

#define SlaveAddress 0x3c   //定义器件在IIC总线中的从地址
uchar Rec_Data[6];


/*延时函数*/
void Delay(uint t)
{
    while(t--)
    {}
}
/********************************************
**LCD显示函数
********************************************/
/*写入一个数据或指令*/
void Send_DATA(uchar Data,uchar A)
{
    uchar i,Dat;
    CS=0;
    switch(A)
    {
        case 0 : Dat=0xf8; break;//写指令
        case 1 : Dat=0xfa; break;//写数据
        default : break;
    } 
    SCLK=0;
    CS=1;
    for(i=0;i<8;i++) 
    {
        SID=Dat&0x80;
        SCLK=0;
        SCLK=1;
        Dat<<=1;
    }   
    Dat=Data&0xf0;
    for(i=0;i<8;i++) 
    {
        SID=Dat&0x80;
        SCLK=0;
        SCLK=1;
        Dat<<=1;
    } 
    Dat=(Data<<4);
    for(i=0;i<8;i++) 
    {
        SID=Dat&0x80;
        SCLK=0;
        SCLK=1;
        Dat<<=1;
    }   
    CS=0;
    Delay(20);  
}
/*12232液晶初始化*/
void LCD_Init(void)
{
    Delay(60000);
    Send_DATA(0x02,0);
    Send_DATA(0x0c,0);
    Send_DATA(0x30,0);
    Send_DATA(0x06,0);
    Send_DATA(0x01,0);
    Delay(3000);
}
/*显示字符串*/
void Dis_str(uchar Addr,uchar *str)
{
    Send_DATA(Addr,0);
    Delay(10);
    while(*str!='\0')
    {
        Send_DATA(*str,1);
        str++;
        Delay(10);
    }
}
/*起始信号*/
void IIC_Start(void)
{
    SDA=1;
    SCL=1;
    Delay(5);
    SDA=0;
    Delay(5);
    SCL=0;
}
/*停止信号*/
void IIC_Stop(void)
{
    SDA=0;
    SCL=1;
    Delay(5);
    SDA=1;
    Delay(5);
}
/*发送应答信号*/
void IIC_SendAck(bit Ack)
{
    SDA=Ack;//ack (0:ACK 1:NACK)
    SCL=1;
    Delay(5);
    SCL=0;
    Delay(5);
}
/*接收应答信号*/
bit IIC_RecAck(void)
{
    SCL=1;
    Delay(5);
    CY=SDA;
    SCL=0;
    Delay(5);
    return CY;
}
/*向IIC总线发送一个字节数据*/
void HMC5883_Send_Byte(uchar Dat)
{
    uchar i;
    for(i=0;i<8;i++)
    {
        Dat<<=1;
        SDA=CY;
        SCL=1;
        Delay(5);
        SCL=0;
        Delay(5);
    }
    IIC_RecAck();
}
/*从IIC总线接收一个字节数据*/
uchar HMC5883_Rec_Byte(void)
{
    uchar i,Dat=0;
    SDA=1;
    for(i=0;i<8;i++)
    {
        Dat<<=1;
        SCL=1;
        Delay(5);
        Dat |=SDA;
        SCL=0;
        Delay(5);
    }
    return Dat;
}
/*单字节写HMC5833*/
void Single_Write_HMC5883(uchar Address,uchar Dat)
{
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(Address);
    HMC5883_Send_Byte(Dat);
    IIC_Stop();
}
/*单字节读HMC5833*/
/*uchar Single_Read_HMC5883(uchar Addr)
{
    uchar Value;
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(Addr);
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress+1);
    Value=HMC5883_Rec_Byte();
    IIC_SendAck(1);
    IIC_Stop();
    return Value;
}*/
/*多字节读HMC5833*/
void Multiple_Read_HMC5883(void)
{
    uchar i;  //连续读出HMC5883内部角度数据，地址范围0x3~0x5
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(0x03);//发送存储单元地址，从0x03开始 
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress+1);
    for(i=0;i<6;i++) //连续读取6个地址数据，存储在Rec_Data
    {
        Rec_Data[i]=HMC5883_Rec_Byte();
        if(i==5)
            IIC_SendAck(1); //最后一个数据需要回NOACK
        else
            IIC_SendAck(0); //回应ACK
    }
    IIC_Stop();
    Delay(100);
}
//初始化HMC5883，根据需要请参考pdf进行修改****
void HMC5883_Init(void)
{
     Single_Write_HMC5883(0x02,0x00); 
}
/*主函数*/
void main(void)
{
    int X,Y,Z;
    double Angle;
    uint Acr;

    LCD_Init();//LCD12232液晶初始化
    Dis_str(0x80,"3 轴数字罗盘");
    HMC5883_Init();//HMC5883初始化

    do
    {
        Multiple_Read_HMC5883();//连续读出数据，存储在Rec_Data[]中
        X=Rec_Data[0]<<8 | Rec_Data[1];//Combine MSB and LSB of X Data output register
        Z=Rec_Data[2]<<8 | Rec_Data[3];//Combine MSB and LSB of Z Data output register
        Y=Rec_Data[4]<<8 | Rec_Data[5];//Combine MSB and LSB of Y Data output register
        Angle= atan2((double)Y,(double)X)*(180/3.14159265)+180;//单位：角度 (0~360)
        Angle*=10;
        Acr=(uint)Angle;

        Send_DATA(0x92,0);
        Send_DATA(Acr%10000/1000+0x30,1);
        Send_DATA(Acr%1000/100+0x30,1);
        Send_DATA(Acr%100/10+0x30,1);
        Send_DATA('.',1);
        Send_DATA(Acr%10+0x30,1);

        Delay(50000);
    }
    while(1);
}
