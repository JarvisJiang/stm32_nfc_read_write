#include "stm32f10x.h"
#include "rc522_iic.h"
#include <string.h>
#include <stdio.h>
#include <MFRC522.h>
extern Uid g_uid;
void test_mifare_s50(PICC_Type piccType);
/*******************************
*连线说明：
*1--SS(SDA)  <----->PB0
*2--SCK <----->PB13
*3--MOSI<----->PB15
*4--MISO<----->PB14
*5--悬空
*6--GND <----->GND
*7--RST <----->PB1
*8--VCC <----->VCC

#if NFC_IIC

*1--SCK(MISO) <--->  PB10
*2--SDA <--->  PB11
*3--RST <--->  PB1


#endif
************************************/

/************************************/
PICC_Type get_nfc_type(unsigned char ack);
//static void delay_us(unsigned int nus)
//{
//	  unsigned int i = 0; 
//		while(nus--)
//		{
//			i = 10;
//			while(i--);
//		}
//}

static void delay_ms(unsigned int nms)
{		
			unsigned int i = 0;
	    while(nms--)
			{
				i = 12000;
				while(i--);
			}
							    
}
void MFRC522_init(void);
void test_ntag21x_read_write(void);
unsigned char BBC_check_out(unsigned char *buf, int size);
void change_ntag21x_uid(void);
char test_ntag21x(void) {
  // Look for new cards
//	unsigned char WBuff[] = {0x00, 0x00, 0x00, 0x04};
//	unsigned char RBuff[18];
	unsigned char pACK[2];
	unsigned char PSWBuff[] = {0xFF, 0xFF, 0xFF, 0xFF}; //32 bit PassWord default FFFFFFFF
	int i;
	printf("Scan PICC to see UID, type, and data blocks...\r\n");
	while(1)
	{
		
	//delay_ms(5);
  if( !PICC_IsNewCardPresent()){
	//	delay_ms(200);
		printf("scaner erro\r\n");
		continue;
 }
	
		delay_ms(58);
  // Select one of the cards
  if (!PICC_ReadCardSerial()) {
		printf("read card serial erro\r\n");
    continue;
  }
	printf("uid:");
	for(i = 0; i<10;i++)
	{
		printf("%x ",g_uid.uidByte[i]);
	}
	printf("sak=%x\r\n",g_uid.sak);
	printf("\r\n");//sak
//  printf("Auth: ");
  PCD_NTAG216_AUTH(&PSWBuff[0], pACK); //Request Authentification if return STATUS_OK we are good

  //Print PassWordACK

	printf("%x %x\r\n",pACK[0],pACK[1]);


	//	change_ntag21x_uid();
		test_ntag21x_read_write();
  // PICC_DumpMifareUltralightToSerial(); //This is a modifier dunp just cghange the for cicle to < 232 instead of < 16 in order to see all the pages on NTAG216

  delay_ms(200);
}
}
/*


*/



void test_nfc(void)
{
	PICC_Type type;
	unsigned char i;
	unsigned char try_cnt = 0;
	MFRC522_init();
	printf("Scan PICC to see UID, type, and data blocks...\r\n");
	//test_ntag21x();
	while(1)
	{
		
	//delay_ms(5);
		if(try_cnt>9)
		{
			
			MFRC522_init();
			printf("MFRC522_init\r\n");
			
			
			PCD_AntennaOff();
			delay_ms(200);
			PCD_AntennaOn();
			try_cnt = 0;
		}
		if( !PICC_IsNewCardPresent()){
		//	delay_ms(200);
			printf("scaner erro\r\n");
			try_cnt++;
			continue;
		}

		//	delay_ms(58);
		// Select one of the cards
		if (!PICC_ReadCardSerial()) 
		{
			printf("read card serial erro\r\n");
			try_cnt++;
			continue;
		}
//	break;
	
	printf("uid:");
	for(i = 0; i<10;i++)
	{
		printf("%x ",g_uid.uidByte[i]);
	}
	printf("sak=%x\r\n",g_uid.sak);
	printf("\r\n");//sak
	type = get_nfc_type(g_uid.sak);
	
	
	
	switch (type) {
		
		case PICC_TYPE_MIFARE_1K:		;
											
		case PICC_TYPE_MIFARE_4K:		test_mifare_s50(type);
												try_cnt = 10;
															break;	
		case PICC_TYPE_MIFARE_UL:	test_ntag21x_read_write();
												try_cnt = 10;
										break;	
		case PICC_TYPE_UNKNOWN:
		default:						printf("can not read and write\r\n");
										break;	
	}
}
	
	//test_ntag21x();
}

/*


*/

static unsigned char g_try_cnt = 0;
PICC_Type nfc_scan(void)
{
	PICC_Type type;
	unsigned char i;
	unsigned char try_rtn = 0;
	MFRC522_init();
	printf("Scan PICC to see UID, type, and data blocks...\r\n");
	//test_ntag21x();
	while(1)
	{

		if(g_try_cnt>9)
		{
			
			MFRC522_init();
			printf("MFRC522_init\r\n");
			
			
			PCD_AntennaOff();
			delay_ms(200);
			PCD_AntennaOn();
			g_try_cnt = 0;
			try_rtn++;
			if(5==try_rtn)
			{
				return PICC_TYPE_UNKNOWN;
			}
		}
		if( !PICC_IsNewCardPresent()){
			printf("scaner erro\r\n");
			g_try_cnt++;
			continue;
		}

		//	delay_ms(58);
		// Select one of the cards
		if (!PICC_ReadCardSerial()) 
		{
			printf("read card serial erro\r\n");
			g_try_cnt++;
			continue;
		}
//	break;
	
		printf("uid:");
		for(i = 0; i<10;i++)
		{
			printf("%x ",g_uid.uidByte[i]);
		}
		printf("sak=%x\r\n",g_uid.sak);
		printf("\r\n");//sak
		type = get_nfc_type(g_uid.sak);
	
	
	
		switch (type) {
			
			case PICC_TYPE_MIFARE_1K:		;
												
			case PICC_TYPE_MIFARE_4K:		test_mifare_s50(type);
													g_try_cnt = 10;
																break;	
			case PICC_TYPE_MIFARE_UL:	test_ntag21x_read_write();
													g_try_cnt = 10;
											break;	
			case PICC_TYPE_UNKNOWN:
			default:						printf("can not read and write\r\n");
											break;	
		}
		break;
	}
		return type;

}


unsigned char nfc_compare_buf(unsigned char *src,unsigned char *dst,unsigned char size)
{
	int i = 0;
	for(i = 0; i<size; i++)
	{
		if(src[i]!=dst[i])
			return 0;
	}
	return 1;
}
static unsigned char  buffer[800];
static unsigned char 	wrt_buf[800] ={
															1,2,3,4,5,6,7,8,9,0xa5,
															1,2,3,4,5,6,7,8,9,0xa5,
															1,2,3,4,5,6,7,8,9,0xa5,
															1,2,3,4,5,6,7,8,9,0xa5,
															1,2,3,4,5,6,7,8};
void test_ntag21x_read_write(void) {
	StatusCode status;
	unsigned char  byteCount;
	
	unsigned char  i,page,offset,index,wrt_byte_cnt,wrt_page_cnt;
	
	
	// Try the mpages of the original Ultralight. Ultralight C has more pages.
	
		// Read pages
		byteCount = 128;
		for(wrt_byte_cnt=0,wrt_page_cnt =4;wrt_page_cnt<226;)//888byte
		{
			status = MIFARE_Ultralight_Write(wrt_page_cnt,&wrt_buf[wrt_byte_cnt],4);
			if (status != STATUS_OK) {
			printf("MIFARE_Write(page=%d) failed:  ",wrt_page_cnt);
			GetStatusCodeName(status);
			return;
			//break;
			}
			
			status = MIFARE_Read(wrt_page_cnt, buffer, &byteCount);
			if (status != STATUS_OK) {
			printf("MIFARE_Read() failed: ");
			GetStatusCodeName(status);
			return ;
			}
			if(!nfc_compare_buf(&wrt_buf[wrt_byte_cnt],buffer,4))
			{
				printf("not eq\r\n");
				return;
			}
				
			wrt_byte_cnt+=4;
			wrt_page_cnt++;
			if(wrt_byte_cnt>48)
			{
				wrt_byte_cnt = 0;
			}
		}
		printf("\r\n");
		printf("Page  0  1  2  3\r\n");
		for (page = 0; page < 231; page +=4) { // Read returns data for 4 pages at a time.
		status = MIFARE_Read(page, buffer, &byteCount);
		if (status != STATUS_OK) {
			printf("MIFARE_Read() failed: ");
			GetStatusCodeName(status);
			return;
		}
		// Dump data
		for ( offset = 0; offset < 4; offset++) {
			i = page + offset;
			if(i < 10)
				printf("  "); // Pad with spaces
			else
				printf(" "); // Pad with spaces
			printf("%d",i);
			printf("  ");
			for (index = 0; index < 4; index++) {
				i = 4 * offset + index;
				if(buffer[i] < 0x10)
					printf(" 0");
				else
					printf(" ");
				printf("%x",buffer[i]);
			}
			printf("\r\n");
		}
			}
	
} // End PICC_DumpMifareUltralightToSerial()

/*
CT  = 0X88
CT 	^ SN0 ^ SN1 ^ SN2 = CHECK BYTE 1
SN3 ^ SN4 ^ SN5 ^ SN6	= CHECK BYTE 2
*/
unsigned char BBC_check_out(unsigned char *buf, int size)
{
	unsigned char BCC;
	int i;
	for(i = 0; i<size; i++)
	BCC = BCC^buf[i];
	printf("BBC = %x\r\n",BCC);
	return BCC;
}
										
PICC_Type get_nfc_type(unsigned char ack)
{
	PICC_Type type;
	
	type = PICC_GetType(ack);

	PICC_GetTypeName(type);
	
	return type;
}
