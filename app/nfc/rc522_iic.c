#include "stm32f10x.h"
#include "rc522_iic.h"
#include <MFRC522.h>
//////////////////////////////////////////////////////////////////////////////////	 
//��?3��D��??1??��?�㨺1��?��??��?-����??D��?����?2?��?��?����???����?o?��?��?
//ALIENTEK???��STM32?a�����?
//IIC?y?�� �䨲??	   
//?y��??-����@ALIENTEK
//??��???��3:www.openedv.com
//DT??��??��:2012/9/9
//��?��?��oV1.0
//��?����?����D��?�̨���?��????��
//Copyright(C) 1??Y��DD?������?����????��D?T1??? 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//3?��??��IIC
//IO��??������??

#if NFC_IIC

unsigned char volatile g_rc522_iic_addr;
static void delay_us(unsigned int nus)
{
	  int  i = 0; 
		while(nus--)
		{
			i = 2;//10  1us	
			while(i>0)
			{
				__asm("NOP");
				i--;
			}
				
		}
}
static void NFC_delay_us(unsigned int nus)
{
	  int  i = 0; 
		while(nus--)
		{
			i = 3;//10  1us
			
//			__asm("NOP");
//			__asm("NOP");
			while(i>0)
			{
				__asm("NOP");
				i--;
			}
				
		}
}
#if 0
static u8  fac_us=0;//us��ʱ������			   
static u16 fac_ms=0;//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��
void delay_init(void)
{

 	SysTick->CTRL&=~(1<<2);	//SYSTICKʹ���ⲿʱ��Դ	 
	fac_us=72/8;		//�����Ƿ�ʹ��ucos,fac_us����Ҫʹ��    

	fac_ms=(u16)fac_us*1000;//��ucos��,����ÿ��ms��Ҫ��systickʱ����   
}	
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
#endif
void rc522_test_iic_gpio_delay(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//RCC_APB2Periph_GPIOD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                    //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;							// �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                    //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;							// �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	while(1)
	{
//	PBout(10) = 1;
//		
//	PBout(12) = 1;
//	PBout(13) = 1;
//	PBout(14) = 1;
//		
 	PDout(9)	= 1;
//		
	PBout(11) = 0;
	//	GPIO_SetBits(GPIOD,9);
	NFC_delay_us(1);
	//	GPIO_ResetBits(GPIOD,9);
//	PBout(10) = 0;
//	
//	PBout(12) = 0;
//	PBout(13) = 0;
//	PBout(14) = 0;		
  	PDout(9)	= 0;	
	PBout(11) = 1;
	NFC_delay_us(1);		
	}

}
//static void delay_ms(unsigned int nms)
//{		
//			unsigned int i = 0;
//	    while(nms--)
//			{
//				i = 12000;
//				while(i--);
//			}
//							    
//}

//2������IIC?e��?D?o?
static void IIC_Start(void)
{
	SDA_OUT();     //sda??��?3?
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//?����?I2C����??��?��?��?����?��?��?����?��y?Y 
}	  
//2������IIC����?1D?o?
static void IIC_Stop(void)
{
	SDA_OUT();//sda??��?3?
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����?��I2C����???����?D?o?
	delay_us(4);							   	
}
//�̨���y��|��eD?o?��?����
//����???�̡�o1��??����?��|��e����㨹
//        0��??����?��|��e3��1|
static u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����???a��?��?  
	IIC_SDA=1;//delay_us(5);	   
	IIC_SCL=1;//delay_us(5);	 
	while(READ_SDA)
	{
		ucErrTime++;
		delay_us(1);
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//����?����?3?0 	   
	return 0;  
} 
//2������ACK��|��e
static void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//2?2������ACK��|��e		    
static void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����?����???��??��
//����??�䨮?����D?T��|��e
//1��?��D��|��e
//0��??T��|��e	

static void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//��-�̨�����?��?a��?��y?Y��?��?
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   //??TEA5767?a��y???������??��?��?D?��?
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	    
//?��1??��??����?ack=1������?����?��ACK��?ack=0��?����?��nACK   
static u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����???a��?��?
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(1);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����?��nACK
    else
        IIC_Ack(); //����?��ACK   
    return receive;
}





void RC522_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//��-�̨�����?��?a��?��y?Y��?��?
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   //??TEA5767?a��y???������??��?��?D?��?
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	    
//?��1??��??����?ack=1������?����?��ACK��?ack=0��?����?��nACK   
u8 RC522_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����???a��?��?
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(1);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����?��nACK
    else
        IIC_Ack(); //����?��ACK   
    return receive;
}





//?��RC522??��??��
//DZ:??��??�¦�??��
//����??��o?��3?��??��
static void RC522_WR_Reg(u8 RCsla,u8 addr,u8 val) 
{
	IIC_Start();  				 
	IIC_Send_Byte(RCsla);     	//����?��D��?��?t??��?	 
	IIC_Wait_Ack();	   
    IIC_Send_Byte(addr);   			//����?��??��??�¦�??��
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(val);     		//����?��?��					   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();						//2��������???����?1��??t 	   
}

 static u8 RC522_RD_Reg(u8 RCsla,u8 addr) 		
{
	u8 temp=0;		 
	IIC_Start();  				 
	IIC_Send_Byte(RCsla);	//����?��D��?��?t??��?	 
	temp=IIC_Wait_Ack();	   
    IIC_Send_Byte(addr);   		//����?��??��??�¦�??��
	temp=IIC_Wait_Ack(); 	 										  		   
	IIC_Start();  	 	   		//??D????��
	IIC_Send_Byte(RCsla+1);	//����?��?��?��?t??��?	 
	temp=IIC_Wait_Ack();	   
    temp=IIC_Read_Byte(0);		//?����?��???��??��,2??��D??��?��,����?��NAK 	    	   
    IIC_Stop();					//2��������???����?1��??t 	    
	return temp;				//����???����?��??��
} 
u8 IIC_R_RC522(u8 DZ)
{						   
	u8 DAT=0;	
    DAT = RC522_RD_Reg(g_rc522_iic_addr,  DZ);
	return DAT;          		//����??��?��?��?��y?Y
}

//1|    ?����oD��RC522??��??��
//2?��y?��?�¡�oDZ:??��??�¦�??��
//          DATA:D�䨨?��??��

void IIC_W_RC522(u8 DZ,u8 DATA)
{
    RC522_WR_Reg(g_rc522_iic_addr,  DZ, DATA);
}


void IIC_W_N_RC522(unsigned char reg,		///< The register to write to. One of the PCD_Register enums.
									unsigned char count,		///< The number of bytes to write to the register
									unsigned char *values)
{
		u8 index;
		IIC_Start();  				 
		IIC_Send_Byte(g_rc522_iic_addr);     	//����?��D��?��?t??��?	 
		IIC_Wait_Ack();	   

		IIC_Send_Byte(reg);   			//����?��??��??�¦�??��
		IIC_Wait_Ack(); 
		for (index = 0; index < count; index++)
		{	
			IIC_Send_Byte(values[index]);     		//����?��?��					   
			IIC_Wait_Ack();  	
		}			
		IIC_Stop();						//2��������???����?1��??t 	   
}

void IIC_R_N_RC522(unsigned char reg,		///< The register to write to. One of the PCD_Register enums.
									unsigned char count,		///< The number of bytes to write to the register
									unsigned char *values)
{
		u8 index;

		IIC_Start();  				 
		IIC_Send_Byte(g_rc522_iic_addr);	//����?��D��?��?t??��?	 
		IIC_Wait_Ack();	   
		IIC_Send_Byte(reg);   		//����?��??��??�¦�??��
		IIC_Wait_Ack(); 	 										  		   
		IIC_Start();  	 	   		//??D????��
		IIC_Send_Byte(g_rc522_iic_addr+1);	//����?��?��?��?t??��?	 
		IIC_Wait_Ack();
		for (index = 0; index < count; index++)
		{
			values[index]=IIC_Read_Byte(1);		//?����?��???��??��,2??��D??��?��,����?��NAK 	    
			//IIC_Wait_Ack();
		}			
		IIC_Stop();					//2��������???����?1��??t 	    

}

void rc_522_IIC_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//RCC_APB2Periph_GPIOD
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                    //
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;							// �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                    //
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;							// �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                    //
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;							// �����������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
//		RCC->APB2ENR|=1<<3;//��ʹ������IO PORTBʱ�� 							 
//		GPIOB->CRH&=0XFFF000FF;//PB1/11 �������
//		GPIOB->CRH|=0X00033300;	   
//		GPIOB->ODR|=3<<10;     //PB10,11 �����		 
//		
//		RCC->APB2ENR|=1<<3;//��ʹ������IO PORTBʱ�� 	
//		GPIOB->CRL&=0XFF0FFF0F;//PB1��5 �������        //PB1 == rc522��λ
//		GPIOB->CRL|=0X00300030;	   
//		GPIOB->ODR|=1<<1; 
//		GPIOB->ODR|=1<<5; 
//		GPIOB->CRH&=0XFFFFFFF0;//PB8 �������
//		GPIOB->CRH|=0X00000003;	   
//		GPIOB->ODR|=0<<8;     //PB8 �����

}

 #endif










