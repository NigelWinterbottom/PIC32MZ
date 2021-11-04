/*
 * File name: main.c for PIC32MZ Basic USB CDC ACM, version 2.0
 * Released under FreeBSD License
 *
 * (C)2019, 2020 - Mark Serran Lewis
 * email: mark@lewistechnogroup.com, misterhemi@yahoo.com
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of the FreeBSD Project.
 *
 */


#define SYS_FREQ (200000000L)
#define GetSystemClock() (200000000ul)      // PIC32MZ System Clock in Hertz.


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/attribs.h>
#include <sys/kmem.h>
#include "usb.h"

/************* Declare functions *************/
void initBoard(void);

#define LED_On()    	do {LATECLR = _LATB_LATB8_MASK;} while (0)
#define LED_Off()       do {LATESET = _LATB_LATB8_MASK;} while (0)
#define LED_Toggle()	do {LATEINV = _LATB_LATB8_MASK;} while (0)
#define LED_Get()       ((PORTB >> _LATB_LATB8_POSITION) & 0x1)


int main (void)
{
    initBoard();

    /* Configure interrupts */
    PRISS = 0x07654321; // PRIORITY 1 use Shadow Set 1 - PRIORITY 7 use Shadow Set 7
    INTCONbits.MVEC = 1;    // Enable system wide multi-vectored interrupts.

    initUSB();

    __builtin_enable_interrupts();

    enableUSB = TRUE;

    while(1){
        /* Check if USB is connected/disconnected by monitoring VBUS */
        if(USBOTGbits.VBUS == VBUS_VALID && enableUSB == TRUE){
            LED_On();
            connectUSB();
            enableUSB = FALSE;
        } else if(USBOTGbits.VBUS != VBUS_VALID && enableUSB == FALSE){
            LED_Off();
            disableUSB();
            enableUSB = TRUE;
        }
    }
}

void initBoard(){

	//GPIO Connections for PCB (PC50611)

	/* --PORTA-- */
	// DPIN  APIN   DIR  INIT  Function
	//---------------------------------
	// RA15  AN10   O/P  0     CS_MUX1
	// RA14  AN9    O/P  0     CS_MUX
	// RA13  ----   ---  ----  N/A
	// RA12  ----   ---  ----  N/A
	// RA11  ----   ---  ----  N/A
	// RA10  AN28   I/P  ANAL  VREF+
	// RA9   AN27   I/P  ANAL  VREF-
	// RA8   ----   ---  ---   N/A
	// RA7          O/P  0     PRES_SW
	// RA6          O/P  0     LCD_DATA_EN
	// RA5   AN34   O/P  0     PRG_PUMP
	// RA4          I/P  P/U   KEY_HOME
	// RA3          I/P  P/U   KEY_MEAS
	// RA2          O/P  0     SOLENOID
	// RA1   AN29   O/P  1     PWR_EN
	// RA0   AN24   O/P  1     ADC_CS

	ANSELA= 0b0000011000000000;
	ODCA  = 0b0000000000000000;
	LATA  = 0b0000000000000011;
	TRISA = 0b0000011000011000;
	CNPUA = 0b0000000000011000;
	CNPDA = 0b0000000000000000;
	do { __CNCONAbits_t val = {.ON=1, .EDGEDETECT=0}; CNCONAbits = val; } while (0);
	CNENA = 0;
	CNNEA = 0;


	/* --PORTB-- */
	// DPIN  APIN   DIR  INIT  Function
	//---------------------------------
	// RB15  AN10   I/P  ----  KEY_POWER
	// RB14  AN9    I/P  P/D   LK5/LK6
	// RB13  AN8    I/P  ANAL  PCB_ID
	// RB12  AN7    I/P  ANAL  HC_TEMP
	// RB11  AN6    I/P  ANAL  INT_TEMP
	// RB10  AN5    I/P  ANAL  BAT_TEMP
	// RB9   AN49   O/P  0     IR_PRINT
	// RB8   AN48   O/P  1     H_SYNC
	// RB7   AN47   I/P  P/D   PGED2
	// RB6   AN46   I/P  P/D   PGEC2
	// RB5   AN45   I/P  P/U   BT_RTS
	// RB4   AN4    I/P  P/U   KEY_DOWN
	// RB3   AN3    I/P  P/U   KEY_LEFT
	// RB2   AN2    I/P  P/U   KEY_RIGHT
	// RB1   AN1    I/P  ANAL  PP_MON
	// RB0   AN0    I/P  ANAL  MP_MON

	ANSELB= 0b0011110000000011;
	ODCB  = 0b0000000000000000;
	LATB  = 0b0000000100000000;
	TRISB = 0b1111110011111111;
	CNPUB = 0b0000000000111100;
	CNPDB = 0b0100000011000000;
	do { __CNCONBbits_t val = {.ON=1, .EDGEDETECT=0}; CNCONBbits = val; } while (0);
	CNENB = 0;
	CNNEB = 0;


	/* --PORTC-- */
	// DPIN  APIN   DIR  INIT  Function
	//---------------------------------
	// RC15  ----   O/P  0     TP22
	// RC14  ----   I/P  ----  OSC_32KHZ
	// RC13  ----   O/P  0     BT_RESET
	// RC12  ----   I/P  ----  OSC_12MHZ
	// RC11  ----   ---  ----  N/A
	// RC10  ----   ---  ----  N/A
	// RC9   ----   ---  ----  N/A
	// RC8   ----   ---  ----  N/A
	// RC7   ----   ---  ----  N/A
	// RC6   ----   ---  ----  N/A
	// RC5   ----   ---  ----  N/A
	// RC4   AN19   O/P  0     DCLK
	// RC3   AN20   O/P  1     V_SYNC
	// RC2   AN21   O/P  0     SW_VOLTS
	// RC1   AN22   O/P  0     BULB_ON
	// RC0   ----   ---  ----  N/A

	ANSELC= 0b0000000000000000;
	ODCC  = 0b0000000000000000;
	LATC  = 0b0000000000001000;
	TRISC = 0b0101000000000000;
	CNPUC = 0b0000000000000000;
	CNPDC = 0b0000000000000000;
	do { __CNCONCbits_t val = {.ON=0, .EDGEDETECT=0}; CNCONCbits = val; } while (0);
	CNENC = 0;
	CNNEC = 0;


	/* --PORTD-- */
	// DPIN  APIN   DIR  INIT  Function
	//---------------------------------
	// RD15  AN33   O/P  0     BLEEP0
	// RD14  AN32   O/P  0     BLEEP1
	// RD13  ----   O/P  0     RED2
	// RD12  ----   O/P  0     RED1
	// RD11  ----   O/P  0     ADC_SCK
	// RD10  ----   I/P  P/U   ADC_SDO
	// RD9   ----   O/P  0     LCD_RESET
	// RD8   ----   ---  ----  N/A
	// RD7   ----   ---  ----  N/A
	// RD6   ----   ---  ----  N/A
	// RD5   ----   O/P  0     MAIN_PUMP
	// RD4   ----   I/P  ----  EXT_DC
	// RD3   ----   O/P  0     RED4
	// RD2   ----   O/P  0     RED3
	// RD1   ----   O/P  0     BACLKIGHT
	// RD0   ----   O/P  1     LCD_SHUTDOWN

	ANSELD= 0b0000000000000000;
	ODCD  = 0b0000000000000000;
	LATD  = 0b0000000000000001;
	TRISD = 0b0000010000010000;
	CNPUD = 0b0000010000000000;
	CNPDD = 0b0000000000000000;
	do { __CNCONDbits_t val = {.ON=1, .EDGEDETECT=0}; CNCONDbits = val; } while (0);
	CNEND = 0;
	CNNED = 0;

	/* --PORTE-- */
	// DPIN  APIN   DIR  INIT  Function
	//---------------------------------
	// RE15  ----   ---  ----  N/A
	// RE14  ----   ---  ----  N/A
	// RE13  ----   ---  ----  N/A
	// RE12  ----   ---  ----  N/A
	// RE11  ----   ---  ----  N/A
	// RE10  ----   ---  ----  N/A
	// RE9   AN26   I/P  P/U   KEY_UP
	// RE8   AN25   O/P  1     WTRAP_LED
	// RE7   AN15   O/P  0     GREEN2
	// RE6   AN16   O/P  0     GREEN1
	// RE5   AN17   O/P  0     GREEN0
	// RE4   AN18   O/P  0     BLUE4
	// RE3   ----   O/P  0     BLUE3
	// RE2   ----   O/P  0     BLUE2
	// RE1   ----   O/P  0     BLUE1
	// RE0   ----   O/P  0     BLUE0

	ANSELE= 0b0000000000000000;
	ODCE  = 0b0000000000000000;
	LATE  = 0b0000000100000000;
	TRISE = 0b0000001000000000;
	CNPUE = 0b0000001000000000;
	CNPDE = 0b0000000000000000;
	do { __CNCONEbits_t val = {.ON=1, .EDGEDETECT=0}; CNCONEbits = val; } while (0);
	CNENE = 0;
	CNNEE = 0;


	/* --PORTF-- */
	// DPIN  APIN   DIR  INIT  Function
	//---------------------------------
	// RF15  ----   ---  ----  N/A
	// RF14  ----   ---  ----  N/A
	// RF13  AN30   O/P  0     FLASH_SCK
	// RF12  AN31   O/P  0     FLASH_CE
	// RF11  ----   ---  ----  N/A
	// RF10  ----   ---  ----  N/A
	// RF9   ----   ---  ----  N/A
	// RF8   ----   O/P  0     FLASH_SI
	// RF7   ----   ---  ----  N/A
	// RF6   ----   ---  ----  N/A
	// RF5   ----   I/P  P/U   KEY_PRINT
	// RF4   ----   I/P  P/U   KEY_SAVE
	// RF3   ----   I/P  P/U   (USBID) N/C
	// RF2   ----   I/P  P/D   FLASH_SO
	// RF1   ----   O/P  0     GREEN5
	// RF0   ----   O/P  0     RED0

	ANSELF= 0b0000000000000000;
	ODCF  = 0b0000000000000000;
	LATF  = 0b0000000000000000;
	TRISF = 0b0000000000111100;
	CNPUF = 0b0000000000111000;
	CNPDF = 0b0000000000000100;
	do { __CNCONFbits_t val = {.ON=1, .EDGEDETECT=0}; CNCONFbits = val; } while (0);
	CNENF = 0;
	CNNEF = 0;

	/* --PORTG-- */
	// DPIN  APIN   DIR  INIT  Function
	//---------------------------------
	// RG15  AN23   O/P  0     CHARGE_EN
	// RG14  ----   O/P  0     ADC_C
	// RG13  ----   O/P  0     ADC_B
	// RG12  ----   O/P  0     ADC_A
	// RG11  ----   ---  ----  N/A
	// RG10  ----   ---  ----  N/A
	// RG9   AN11   I/P  P/U   BT_TXD
	// RG8   AN12   O/P  O/D   I2C_SCL
	// RG7   AN13   O/P  O/D   I2C_SDA
	// RG6   AN14   O/P  1     BT_RXD
	// RG5   ----   ---  ----  N/A
	// RG4   ----   ---  ----  N/A
	// RG3   ----   ---  ----  N/A
	// RG2   ----   ---  ----  N/A
	// RG1   ----   O/P  0     GREEN_4
	// RG0   ----   O/P  0     GREEN_3

	ANSELG= 0b0000000000000000;
	ODCG  = 0b0000000110000000;
	LATG  = 0b0000000111000000;
	TRISG = 0b0000001000000000;
	CNPUG = 0b0000001000000000;
	CNPDG = 0b0000000000000000;
	do { __CNCONGbits_t val = {.ON=0, .EDGEDETECT=0}; CNCONGbits = val; } while (0);
	CNENG = 0;
	CNNEG = 0;
}
