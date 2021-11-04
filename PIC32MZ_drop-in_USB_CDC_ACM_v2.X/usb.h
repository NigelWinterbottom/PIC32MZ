/*
 * File name: usb.h for PIC32MZ Basic USB CDC ACM, version 2.0
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

#ifndef _USB_H    /* Guard against multiple inclusion */
#define _USB_H

#include <stdint.h>

/* USB related values */
#define EP0BUFFSIZE     8   // In multiples of 8, e.g.- 8 (x8) equals a size of 64
#define EP1TXBUFFSIZE   2   // Also in multiples of 8, e.g.- 1 (x8) equals a size of 8
#define EP2TXBUFFSIZE   8   // Also in multiples of 8, e.g.- 8 (x8) equals a size of 64
#define EP3RXBUFFSIZE   8   // Also in multiples of 8, e.g.- 8 (x8) equals a size of 64
#define EP0_MAX_PACKET_SIZE  (EP0BUFFSIZE * 8)
#define EP1_MAX_PACKET_SIZE  (EP1TXBUFFSIZE * 8)
#define EP2_MAX_PACKET_SIZE  (EP2TXBUFFSIZE * 8)
#define EP3_MAX_PACKET_SIZE  (EP3RXBUFFSIZE * 8)
#define VBUS_VALID 0x3

/*********** Variable Declarations ***********/
volatile uint32_t ep0rbc;   // Endpoint 0 - Received Bytes Count (USB).
volatile uint32_t ep3rbc;   // Endpoint 3 - Received Bytes Count (USB).
volatile uint32_t i;
volatile uint32_t i2;
uint8_t *ep0usbData;
uint8_t *ep1usbData;
uint8_t *ep2usbData;
uint8_t endpoint;
void    *ep0BlockData;
void    *ep1BlockData;
void    *ep2BlockData;
uint8_t enableUSB;
uint16_t NumBytes, packetSize;
uint16_t packetSize_ep0, packetSize_ep1, packetSize_ep2;
uint16_t ep0RemainingBytes;
uint16_t ep1RemainingBytes;
uint16_t ep2RemainingBytes;
uint16_t timeout_ep0;
volatile int usbReset;
uint8_t ep0data[(EP0BUFFSIZE * 8)];   // USB end point 0 data
uint8_t ep0usbTemp[(EP0BUFFSIZE * 8)];// USB end point 0 temporary data;
uint8_t ep1data[(EP1TXBUFFSIZE * 8)]; // USB end point 1 data
uint8_t ep2data[(EP2TXBUFFSIZE * 8)]; // USB end point 2 data
uint8_t ep3data[(EP3RXBUFFSIZE * 8)]; // USB end point 2 data
volatile uint8_t *tempData;
volatile uint16_t bmbRequest; // bmRequestType, bRequest
volatile uint16_t wValue;
volatile uint16_t wIndex;
volatile uint16_t wLength;
volatile uint8_t usbAddress;
volatile uint8_t setDevAddr;
volatile uint32_t ep0ArrPos;
volatile uint32_t ep1ArrPos;
volatile uint32_t ep2ArrPos;
uint8_t usbConfiguration;
uint8_t usbInterface;

enum {FALSE,TRUE};

/*********************************************************************
 CDC ACM Related Variables
*********************************************************************/
uint32_t dwDTERate;
uint8_t bCharFormat;
uint8_t bParityType;
uint8_t bDataBits;

uint8_t defaultLineCoding[8];
uint8_t *serialState;
uint16_t controlState;
/******* End of Variable Declarations ********/

/************* Declare functions *************/
void initUSB(void);
uint8_t connectUSB(void);
uint8_t disableUSB(void);
void setAddress(uint8_t address);
void controlTrans(void);
void txUSB_ep0(uint32_t ep0TXLen);
int txBlock_ep0(uint16_t NumBytes_ep0);
int txBlock_ep1(uint16_t NumBytes_ep1);
void txByte_ep2(uint16_t ep2ArrPos);
int txBlock_ep2(uint16_t NumBytes_ep2);
void rxUSB_ep0(uint32_t ep0rbc);
void rxUSB_ep3(void);
void sendMessage_ep2(void);
/******** End of Function Declarations *******/


#endif /* _USB_H */

/* *****************************************************************************
 End of File
 */
