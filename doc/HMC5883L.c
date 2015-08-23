================== HMC5883L���Գ���===============
** 3-����������
** Э�飺IIC    ��ϸ��дЭ�飬��ο��������
** ʱ�䣺2012.2.2
** Author:Keliwen
**************************************************/
#include <reg52.h>
#include <math.h>
#include <stdio.h>
#include <intrins.h>

typedef unsigned char uchar;
typedef unsigned int uint;

/*12232Һ�����Ŷ���*/       
sbit CS=P0^2;
sbit SID=P0^1;
sbit SCLK=P0^0;

sbit SCL=P1^0;      //IICʱ�����Ŷ���
sbit SDA=P1^1;      //IIC�������Ŷ���

#define SlaveAddress 0x3c   //����������IIC�����еĴӵ�ַ
uchar Rec_Data[6];


/*��ʱ����*/
void Delay(uint t)
{
    while(t--)
    {}
}
/********************************************
**LCD��ʾ����
********************************************/
/*д��һ�����ݻ�ָ��*/
void Send_DATA(uchar Data,uchar A)
{
    uchar i,Dat;
    CS=0;
    switch(A)
    {
        case 0 : Dat=0xf8; break;//дָ��
        case 1 : Dat=0xfa; break;//д����
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
/*12232Һ����ʼ��*/
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
/*��ʾ�ַ���*/
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
/*��ʼ�ź�*/
void IIC_Start(void)
{
    SDA=1;
    SCL=1;
    Delay(5);
    SDA=0;
    Delay(5);
    SCL=0;
}
/*ֹͣ�ź�*/
void IIC_Stop(void)
{
    SDA=0;
    SCL=1;
    Delay(5);
    SDA=1;
    Delay(5);
}
/*����Ӧ���ź�*/
void IIC_SendAck(bit Ack)
{
    SDA=Ack;//ack (0:ACK 1:NACK)
    SCL=1;
    Delay(5);
    SCL=0;
    Delay(5);
}
/*����Ӧ���ź�*/
bit IIC_RecAck(void)
{
    SCL=1;
    Delay(5);
    CY=SDA;
    SCL=0;
    Delay(5);
    return CY;
}
/*��IIC���߷���һ���ֽ�����*/
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
/*��IIC���߽���һ���ֽ�����*/
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
/*���ֽ�дHMC5833*/
void Single_Write_HMC5883(uchar Address,uchar Dat)
{
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(Address);
    HMC5883_Send_Byte(Dat);
    IIC_Stop();
}
/*���ֽڶ�HMC5833*/
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
/*���ֽڶ�HMC5833*/
void Multiple_Read_HMC5883(void)
{
    uchar i;  //��������HMC5883�ڲ��Ƕ����ݣ���ַ��Χ0x3~0x5
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress);
    HMC5883_Send_Byte(0x03);//���ʹ洢��Ԫ��ַ����0x03��ʼ 
    IIC_Start();
    HMC5883_Send_Byte(SlaveAddress+1);
    for(i=0;i<6;i++) //������ȡ6����ַ���ݣ��洢��Rec_Data
    {
        Rec_Data[i]=HMC5883_Rec_Byte();
        if(i==5)
            IIC_SendAck(1); //���һ��������Ҫ��NOACK
        else
            IIC_SendAck(0); //��ӦACK
    }
    IIC_Stop();
    Delay(100);
}
//��ʼ��HMC5883��������Ҫ��ο�pdf�����޸�****
void HMC5883_Init(void)
{
     Single_Write_HMC5883(0x02,0x00); 
}
/*������*/
void main(void)
{
    int X,Y,Z;
    double Angle;
    uint Acr;

    LCD_Init();//LCD12232Һ����ʼ��
    Dis_str(0x80,"3 ����������");
    HMC5883_Init();//HMC5883��ʼ��

    do
    {
        Multiple_Read_HMC5883();//�����������ݣ��洢��Rec_Data[]��
        X=Rec_Data[0]<<8 | Rec_Data[1];//Combine MSB and LSB of X Data output register
        Z=Rec_Data[2]<<8 | Rec_Data[3];//Combine MSB and LSB of Z Data output register
        Y=Rec_Data[4]<<8 | Rec_Data[5];//Combine MSB and LSB of Y Data output register
        Angle= atan2((double)Y,(double)X)*(180/3.14159265)+180;//��λ���Ƕ� (0~360)
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
