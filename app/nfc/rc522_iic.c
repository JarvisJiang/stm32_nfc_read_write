#include "stm32f10x.h"
#include "rc522_iic.h"
#include <MFRC522.h>
//////////////////////////////////////////////////////////////////////////////////	 
//±?3ìDò??1??§?°ê1ó?￡??′?-×÷??Dí?é￡?2?μ?ó?óú???üè?o?ó?í?
//ALIENTEK???￠STM32?a·￠°?
//IIC?y?ˉ ′ú??	   
//?yμ??-×ó@ALIENTEK
//??ê???ì3:www.openedv.com
//DT??è??ú:2012/9/9
//°?±?￡oV1.0
//°?è¨?ùóD￡?μá°?±????￡
//Copyright(C) 1??YêDD?òíμ?×ó????óD?T1??? 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//3?ê??ˉIIC
//IO·??òéè??

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
static u8  fac_us=0;//us延时倍乘数			   
static u16 fac_ms=0;//ms延时倍乘数,在ucos下,代表每个节拍的ms数
void delay_init(void)
{

 	SysTick->CTRL&=~(1<<2);	//SYSTICK使用外部时钟源	 
	fac_us=72/8;		//不论是否使用ucos,fac_us都需要使用    

	fac_ms=(u16)fac_us*1000;//非ucos下,代表每个ms需要的systick时钟数   
}	
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
#endif
void rc522_test_iic_gpio_delay(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//RCC_APB2Periph_GPIOD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                    //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;							// 复用推免输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                    //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;							// 复用推免输出
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

//2úéúIIC?eê?D?o?
static void IIC_Start(void)
{
	SDA_OUT();     //sda??ê?3?
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//?ˉ×?I2C×ü??￡?×?±?·￠?í?ò?óê?êy?Y 
}	  
//2úéúIICí￡?1D?o?
static void IIC_Stop(void)
{
	SDA_OUT();//sda??ê?3?
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//·￠?íI2C×ü???áê?D?o?
	delay_us(4);							   	
}
//μè′yó|′eD?o?μ?à′
//·μ???μ￡o1￡??óê?ó|′eê§°ü
//        0￡??óê?ó|′e3é1|
static u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDAéè???aê?è?  
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
	IIC_SCL=0;//ê±?óê?3?0 	   
	return 0;  
} 
//2úéúACKó|′e
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
//2?2úéúACKó|′e		    
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
//IIC·￠?íò???×??ú
//·μ??′ó?úóD?Tó|′e
//1￡?óDó|′e
//0￡??Tó|′e	

static void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//à-μíê±?ó?aê?êy?Y′?ê?
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   //??TEA5767?aèy???óê±??ê?±?D?μ?
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	    
//?á1??×??ú￡?ack=1ê±￡?·￠?íACK￡?ack=0￡?·￠?ínACK   
static u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDAéè???aê?è?
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
        IIC_NAck();//·￠?ínACK
    else
        IIC_Ack(); //·￠?íACK   
    return receive;
}





void RC522_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//à-μíê±?ó?aê?êy?Y′?ê?
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   //??TEA5767?aèy???óê±??ê?±?D?μ?
		IIC_SCL=1;
		delay_us(1); 
		IIC_SCL=0;	
		delay_us(1);
    }	 
} 	    
//?á1??×??ú￡?ack=1ê±￡?·￠?íACK￡?ack=0￡?·￠?ínACK   
u8 RC522_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDAéè???aê?è?
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
        IIC_NAck();//·￠?ínACK
    else
        IIC_Ack(); //·￠?íACK   
    return receive;
}





//?áRC522??′??÷
//DZ:??′??÷μ??·
//·μ??￡o?á3?μ??μ
static void RC522_WR_Reg(u8 RCsla,u8 addr,u8 val) 
{
	IIC_Start();  				 
	IIC_Send_Byte(RCsla);     	//·￠?íD′?÷?t??á?	 
	IIC_Wait_Ack();	   
    IIC_Send_Byte(addr);   			//·￠?í??′??÷μ??·
	IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(val);     		//·￠?í?μ					   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();						//2úéúò???í￡?1ì??t 	   
}

 static u8 RC522_RD_Reg(u8 RCsla,u8 addr) 		
{
	u8 temp=0;		 
	IIC_Start();  				 
	IIC_Send_Byte(RCsla);	//·￠?íD′?÷?t??á?	 
	temp=IIC_Wait_Ack();	   
    IIC_Send_Byte(addr);   		//·￠?í??′??÷μ??·
	temp=IIC_Wait_Ack(); 	 										  		   
	IIC_Start();  	 	   		//??D????ˉ
	IIC_Send_Byte(RCsla+1);	//·￠?í?á?÷?t??á?	 
	temp=IIC_Wait_Ack();	   
    temp=IIC_Read_Byte(0);		//?áè?ò???×??ú,2??ìD??ù?á,·￠?íNAK 	    	   
    IIC_Stop();					//2úéúò???í￡?1ì??t 	    
	return temp;				//·μ???áμ?μ??μ
} 
u8 IIC_R_RC522(u8 DZ)
{						   
	u8 DAT=0;	
    DAT = RC522_RD_Reg(g_rc522_iic_addr,  DZ);
	return DAT;          		//·μ??ê?μ?μ?êy?Y
}

//1|    ?ü￡oD′RC522??′??÷
//2?êy?μ?÷￡oDZ:??′??÷μ??·
//          DATA:D′è?μ??μ

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
		IIC_Send_Byte(g_rc522_iic_addr);     	//·￠?íD′?÷?t??á?	 
		IIC_Wait_Ack();	   

		IIC_Send_Byte(reg);   			//·￠?í??′??÷μ??·
		IIC_Wait_Ack(); 
		for (index = 0; index < count; index++)
		{	
			IIC_Send_Byte(values[index]);     		//·￠?í?μ					   
			IIC_Wait_Ack();  	
		}			
		IIC_Stop();						//2úéúò???í￡?1ì??t 	   
}

void IIC_R_N_RC522(unsigned char reg,		///< The register to write to. One of the PCD_Register enums.
									unsigned char count,		///< The number of bytes to write to the register
									unsigned char *values)
{
		u8 index;

		IIC_Start();  				 
		IIC_Send_Byte(g_rc522_iic_addr);	//·￠?íD′?÷?t??á?	 
		IIC_Wait_Ack();	   
		IIC_Send_Byte(reg);   		//·￠?í??′??÷μ??·
		IIC_Wait_Ack(); 	 										  		   
		IIC_Start();  	 	   		//??D????ˉ
		IIC_Send_Byte(g_rc522_iic_addr+1);	//·￠?í?á?÷?t??á?	 
		IIC_Wait_Ack();
		for (index = 0; index < count; index++)
		{
			values[index]=IIC_Read_Byte(1);		//?áè?ò???×??ú,2??ìD??ù?á,·￠?íNAK 	    
			//IIC_Wait_Ack();
		}			
		IIC_Stop();					//2úéúò???í￡?1ì??t 	    

}

void rc_522_IIC_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//RCC_APB2Periph_GPIOD
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                    //
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;							// 复用推免输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                    //
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;							// 复用推免输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                    //
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;							// 复用推免输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
//		RCC->APB2ENR|=1<<3;//先使能外设IO PORTB时钟 							 
//		GPIOB->CRH&=0XFFF000FF;//PB1/11 推挽输出
//		GPIOB->CRH|=0X00033300;	   
//		GPIOB->ODR|=3<<10;     //PB10,11 输出高		 
//		
//		RCC->APB2ENR|=1<<3;//先使能外设IO PORTB时钟 	
//		GPIOB->CRL&=0XFF0FFF0F;//PB1、5 推挽输出        //PB1 == rc522复位
//		GPIOB->CRL|=0X00300030;	   
//		GPIOB->ODR|=1<<1; 
//		GPIOB->ODR|=1<<5; 
//		GPIOB->CRH&=0XFFFFFFF0;//PB8 推挽输出
//		GPIOB->CRH|=0X00000003;	   
//		GPIOB->ODR|=0<<8;     //PB8 输出低

}

 #endif










