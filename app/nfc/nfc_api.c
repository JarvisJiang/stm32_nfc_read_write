#include <stdio.h>
#include "mifare_s50.h"
#include <MFRC522.h>
#define  M_NFC_DEBUG 1
//#if M_NFC_DEBUG
//#define     NFC_DEBUG( fmt, ... )                printf ( fmt, ##__VA_ARGS__ )    
//#else
//#define     NFC_DEBUG( fmt, ... )
//#endif
/*
tips: nfc_connent  need big buf  lager than 18 bytes  less than  
*/
static void delay_ms(unsigned int nms)
{		
			unsigned int i = 0;
	    while(nms--)
			{
				i = 12000;
				while(i--);
			}
							    
}
extern unsigned char volatile g_rc522_iic_addr;
#define RC522_READ_NFC_CMD 0X88
#define RC522_WRITE_NFC_CMD 0X98
char mifare_s50_data_operate(char cmd,unsigned char *buf, int len) ;
char ntag21x_data_operate(char cmd,unsigned char *buf, int len) ;
char get_nfc_connent(unsigned char addr, unsigned char *nfc_connent, int size)
{
		PICC_Type piccType;
		char rtn;
		g_rc522_iic_addr = addr;
	  piccType = nfc_scan();
		printf("get_nfc_connent:%d\r\n",size);
		switch (piccType) {

		case PICC_TYPE_MIFARE_1K:		;
											
		case PICC_TYPE_MIFARE_4K:
											rtn =	mifare_s50_data_operate(RC522_READ_NFC_CMD,nfc_connent,size);
															break;	
		case PICC_TYPE_MIFARE_UL:
											rtn =	ntag21x_data_operate(RC522_READ_NFC_CMD,nfc_connent,size);
										break;	
		case PICC_TYPE_UNKNOWN:
		default:						printf("can not read and write\r\n");
										rtn = 1;//fail	
		}
		return rtn;
}

/*
tips: nfc_connent  need big buf  lager than 18 bytes  less than  
*/
char set_nfc_connent(unsigned char addr, unsigned char *nfc_connent, int size)
{
		PICC_Type piccType;
		char rtn;
		g_rc522_iic_addr = addr;
	  piccType = nfc_scan();
		printf("%s  SIZE =%d\r\n", __FUNCTION__,size);
		switch (piccType) 
		{

		case PICC_TYPE_MIFARE_1K:		;
											
		case PICC_TYPE_MIFARE_4K:
											rtn =	mifare_s50_data_operate(RC522_WRITE_NFC_CMD,nfc_connent,size);
															break;	
		case PICC_TYPE_MIFARE_UL:
											rtn =	ntag21x_data_operate(RC522_WRITE_NFC_CMD,nfc_connent,size);
										break;	
		case PICC_TYPE_UNKNOWN:
		default:						printf("can not read and write\r\n");
										rtn = 1;//fail	
		}
		return rtn;
}
	unsigned char rbuf[512];
	unsigned char wbuf[512];
void test_nfc_api(void)
{	


	unsigned char i;
	unsigned char cnt;

	unsigned char iic_addr  = 0x00;
	
		for(i = 0; i < 128; i++)
			{
			wbuf[i] = i+8;
			}
	  cnt = 0;
	 while(1)
	 {
		 printf("\r\nstart test nfc__ rc522_iic_addr:%x\r\n",iic_addr);
		
		 cnt++;
  	 delay_ms(20);
//		 NFC_DEBUG("cnt = %d\r\n",cnt);
//			for(i = 0; i < 128; i++)
//			{
//			wbuf[i] = 0x4e;
//			}
//		 if(set_nfc_connent(iic_addr,wbuf,128))
//		 {
//			 NFC_DEBUG("ERR:set_nfc_connent \r\n");
//			 continue;
//		 }
//		 printf("write success\r\n");
//		 while(1);
		 if(get_nfc_connent(iic_addr,rbuf,128))
		 {
			 NFC_DEBUG("ERR:get_nfc_connent \r\n");
			 continue;
		 }
		 printf("\r\n*****************success******************\r\n");
		 printf("\r\n*****************success******************\r\n");
		 printf("\r\n*****************success******************rc522_iic_addr:%x\r\n",g_rc522_iic_addr);
			for(i = 0; i < 512; i++)
			{
			NFC_DEBUG("RBUF[%d] =%x\r\n",i ,rbuf[i]);
			}
			
	 }
	
 
}
