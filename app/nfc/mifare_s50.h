#ifndef __MIFARE_S50_
#define __MIFARE_S50_
#include <MFRC522.h>

char mifare_s50_data_operate(char cmd,unsigned char *buf, int len) ;
char ntag21x_data_operate(char cmd,unsigned char *buf, int len) ;
PICC_Type nfc_scan(void);
#endif

