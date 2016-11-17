/****************************************************************************

  Defined UART pins
	TXD1----- PA9-US1-TX
	RXD1----- PA10-US1-RX
	BoardRate 115200,n,8,1 
	
 	TXD2----- PA2-US2-TX
	RXD2----- PA3-US2-RX
	BordRate 15200,n,8,1 	
	

* v0.10    2014-6-21 sun68  Created this file for stadio lighting
*
*/
/* Includes ------------------------------------------------------------------*/




#include "bsp_usart1.h"
//#include "bsp_gpio.h"  


#include "stm32f10x.h"

#include "stm32f10x_usart.h"
#include "misc.h"
#include "stdarg.h"
#include <string.h>
#include <stdio.h>


void USARTx_Config(void);
void test_nfc(void);
int main(void)
{


  USARTx_Config ();                                                          

	printf ( "\r\n nfc iic  ntag21x M1 S50\r\n" );                          
	
	test_nfc();

}





