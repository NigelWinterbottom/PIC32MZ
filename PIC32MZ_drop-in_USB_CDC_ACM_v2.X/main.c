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

#define CLOCK12MHZ // E.g. - Curiosity PIC32MZEF Dev Board 2.0
//#define CLOCK24MHZ // E.g. - Curiosity Pic32MZEF Development Board (V1.0)

// Setup for PIC32MZ EF device for 200 MHz operation - Device Configuration Bits.
#ifdef CLOCK24MHZ
/*** DEVCFG0 ***/
#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_LEVEL_3
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_LEVEL_3
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/
#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    ON
#pragma config IESO =       ON
#pragma config POSCMOD =    EC
#pragma config OSCIOFNC =   ON
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF

/*** DEVCFG2 ***/
#pragma config FPLLIDIV =   DIV_3
#pragma config FPLLRNG =    RANGE_5_10_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_50
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ

/*** DEVCFG3 ***/
#pragma config USERID =     0xFFFF
#pragma config FMIIEN =     ON
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    OFF
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/
#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xFFFF
#endif

#ifdef CLOCK12MHZ
/*** DEVCFG0 ***/
#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_LEVEL_3
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_LEVEL_3
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/
#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    EC
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
//#pragma config WDTPS =      PS1048576
#pragma config WDTPS =      PS16384
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF

/*** DEVCFG2 ***/
#pragma config FPLLIDIV = DIV_1
#pragma config FPLLRNG = RANGE_8_16_MHZ
#pragma config FPLLICLK = PLL_POSC
#pragma config FPLLMULT = MUL_33
#pragma config FPLLODIV = DIV_2
#pragma config UPLLFSEL = FREQ_12MHZ

/*** DEVCFG3 ***/
#pragma config USERID =     0xFFFF
#pragma config FMIIEN =     ON
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    OFF
#pragma config PMDL1WAY =   OFF
#pragma config IOL1WAY =    OFF
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/
#pragma config TSEQ =       0xFFFF
#pragma config CSEQ =       0x0000
#endif


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




int main (void)
{
    initBoard();
    initUSB();
    enableUSB = TRUE;

    while(1){
        /* Check if USB is connected/disconnected by monitoring VBUS */
        if(USBOTGbits.VBUS == VBUS_VALID && enableUSB == TRUE){
            connectUSB();
            enableUSB = FALSE;
        } else if(USBOTGbits.VBUS != VBUS_VALID && enableUSB == FALSE){
            disableUSB();
            enableUSB = TRUE;
        }
    }
}

void initBoard(){

    ANSELF = 0x00000000;    // Disable analog functions on Port F (digital only).


    TRISF = 0x0008; // Set all port F bits as outputs except F3 (USBID).

    /* Enable pull-up resistor(s) */
    CNPUFbits.CNPUF3 = 1;       // Pull-up enables on RPF3 (To set USB as a B-Device )

    /* Configure interrupts */
    PRISSbits.PRI1SS = 0x1; //PRIORITY 1 use Shadow Set 1
    PRISSbits.PRI2SS = 0x2; //.
    PRISSbits.PRI3SS = 0x3; //.
    PRISSbits.PRI4SS = 0x4; //.
    PRISSbits.PRI5SS = 0x5; //.
    PRISSbits.PRI6SS = 0x6; //.
    PRISSbits.PRI7SS = 0x7; //PRIORITY 7 use Shadow Set 7
    PRISSbits.SS0 = 0;

    INTCONbits.MVEC = 1;    // Enable system wide multi-vectored interrupts.
    __builtin_enable_interrupts();

}
