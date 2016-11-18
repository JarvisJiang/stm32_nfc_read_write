#include <stdio.h>
#define  M_NFC_DEBUG 1
#if M_NFC_DEBUG
#define     NFC_DEBUG( fmt, ... )                printf ( fmt, ##__VA_ARGS__ )    
#else
#define     NFC_DEBUG( fmt, ... )
#endif
/*
tips: nfc_connent  need big buf  lager than 18 bytes  less than  
*/
char get_nfc_connent(unsigned char addr, unsigned char *nfc_connent, int size)
{
	
}

/*
tips: nfc_connent  need big buf  lager than 18 bytes  less than  
*/
char get_nfc_connent(unsigned char addr, unsigned char *nfc_connent, int size)
{
	
}


