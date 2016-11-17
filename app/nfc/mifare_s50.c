/**
 * ----------------------------------------------------------------------------
 * This is a MFRC522 library example; see https://github.com/miguelbalboa/rfid
 * for further details and other examples.
 * 
 * NOTE: The library file MFRC522.h has a lot of useful info. Please read it.
 * 
 * Released into the public domain.
 * ----------------------------------------------------------------------------
 * This sample shows how to setup blocks on a MIFARE Classic PICC (= card/tag)
 * to be in "Value Block" mode: in this mode the operations Increment/Decrement,
 * Restore and Transfer can be used.
 * 
 * BEWARE: Data will be written to the PICC, in sector #1 (blocks #4 to #7).
 * 
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno           Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 * 
 */


#include <MFRC522.h>
#include "stdio.h"
#include "stm32f10x.h"
extern Uid g_uid;
void formatValueBlock_s50(unsigned char blockAddr);
unsigned char g_get_sdata[48];
unsigned char g_set_sdata[48];
void test_mifare_s50(PICC_Type piccType) {
	// In this sample we use the second sector,
    // that is: sector #1, covering block #4 up to and including block #7
//    unsigned char sector         = 1;


//    unsigned char trailerBlock   = 7;
//    StatusCode status;

  //  unsigned char buffer[18];

		unsigned char  i;
		unsigned char sector = 5;
	//unsigned char trailerBuffer[] = {
  //      255, 255, 255, 255, 255, 255,       // Keep default key A
   //     0, 0, 0,
  //      0,
  //      255, 255, 255, 255, 255, 255};      // Keep default key B
	MIFARE_Key key;
    // Look for new cards
	while(1)
	{

		for ( i = 0; i < 6; i++) {
		key.keyByte[i] = 0xFF;
		}

		// Show the whole sector as it currently is
	printf("Current data in sector:\r\n");
//	PICC_DumpMifareClassicSectorToSerial(&(g_uid), &key, sector);
		for(i = 0 ; i < 48 ; i++)
		{
			g_set_sdata[i] = 0xa5;
		}
		printf("PICC_WriteMifareClassicSector\r\n");
		for(sector = 4; sector < 16; sector++)// start should be 4
		{
			PICC_WriteMifareClassicSector(&(g_uid),&key,sector,g_set_sdata);
		}
		printf("test :PICC_ReadMifareClassicSector\r\n");
		for(sector = 4; sector < 16; sector++)
		{
		PICC_ReadMifareClassicSector(&(g_uid),&key,sector,g_get_sdata);

			printf("sector:%d\r\n",sector);
			for(i = 0; i<48; i++)
			{
				if(0==i%16)
				{
					printf("\r\nblock %d: ",i/16);
				}
				printf("%x ",g_get_sdata[i]);
				
			}
			
			printf("\r\n");
		}
		return;

	}
    
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_block_array(unsigned char *buffer, unsigned char bufferSize) {
   unsigned char i;
   for (i = 0; i < bufferSize; i++) {
        printf((buffer[i] < 0x10) ? " 0" : " ");
        printf("%x",buffer[i]);
    }
}

/**
 * Ensure that a given block is formatted as a Value Block.
 */
void formatValueBlock_s50(unsigned char blockAddr) {
    unsigned char buffer[18];
    unsigned char size = sizeof(buffer),i;
    StatusCode status;
		unsigned char  valueBlock[18] = {
            0, 0, 0, 0,
            255, 255, 255, 255,
            0, 0, 0, 0,
            0, 0, 0, 0};
		MIFARE_Key key;
		valueBlock[12] = blockAddr;
		valueBlock[13] = ~blockAddr;
		valueBlock[14] = blockAddr;
		valueBlock[15] = ~blockAddr;
		printf("format...................\r\n");
    printf("Reading block:%x\r\n",blockAddr);
						// Authenticate using key A
			for ( i = 0; i < 6; i++) {
		key.keyByte[i] = 0xFF;
		}

		PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, 7, &key, &(g_uid));
		if (status != STATUS_OK) {
		printf("PCD_Authenticate() failed: \r\n");
		GetStatusCodeName(status);
		return;
		}
    status = MIFARE_Read(blockAddr, buffer, &size);
    if (status != STATUS_OK) {
        printf("MIFARE_Read() failed: \r\n");
        GetStatusCodeName(status);
        return;
    }

    if (    (buffer[0] == (unsigned char)~buffer[4])
        &&  (buffer[1] == (unsigned char)~buffer[5])
        &&  (buffer[2] == (unsigned char)~buffer[6])
        &&  (buffer[3] == (unsigned char)~buffer[7])

        &&  (buffer[0] == buffer[8])
        &&  (buffer[1] == buffer[9])
        &&  (buffer[2] == buffer[10])
        &&  (buffer[3] == buffer[11])

        &&  (buffer[12] == (unsigned char)~buffer[13])
        &&  (buffer[12] ==        buffer[14])
        &&  (buffer[12] == (unsigned char)~buffer[15])) {
        printf("Block has correct Value Block format.\r\n");
    }
    else {
        printf("Formatting as Value Block...\r\n");
        
			status = MIFARE_Write(blockAddr, valueBlock, 16);
        if (status != STATUS_OK) {
            printf("MIFARE_Write() failed: ");
            GetStatusCodeName(status);
        }
    }
}
