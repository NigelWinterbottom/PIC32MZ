/*/*
 * File name: usb_descriptors.h for PIC32MZ Basic USB CDC ACM, version 2.0
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

#ifndef _USB_DESCRIPTORS_H    /* Guard against multiple inclusion */
#define _USB_DESCRIPTORS_H


/*********************************************************************
 Device Descriptors
 *********************************************************************/
unsigned char dev_desc[] = {
/* Descriptor Length						*/ 18, /* Always 18 or 0x12 */
/* DescriptorType: DEVICE					*/ 0x01,
/* bcdUSB (ver 2.0)							*/ 0x00,0x02,
/* bDeviceClass								*/ 0x00,
/* bDeviceSubClass							*/ 0x00,
/* bDeviceProtocol							*/ 0x00,
/* bMaxPacketSize0							*/ 0x40, /* Always 64 or 0x40 for High Speed USB */
/* idVendor									*/ 0xD8,0x04, /* e.g. - 0x04D8 - Microchip VID */
/* idProduct								*/ 0x0A,0x00, /* e.g. - 0x000A */
/* bcdDevice								*/ 0x00,0x02, /* e.g. - 02.00 */
/* iManufacturer							*/ 0x01,
/* iProduct									*/ 0x02,
/* iSerialNumber							*/ 0x00,
/* bNumConfigurations						*/ 0x01
};

unsigned char CDC_ConfigDescriptorArray[] = {
// Referenced from: https://gist.github.com/tai/acd59b125a007ad47767
/*********************************************************************
 Configuration Descriptors
 *********************************************************************/
/*  bLength (Descriptor Length)             */ 9,
/*  bDescriptorType: CONFIG					*/ 0x02,
/*  wTotalLength							*/ 0x43,0x00,
/*  bNumInterfaces							*/ 2,
/*  bConfigurationValue						*/ 1,
/*  iConfiguration							*/ 0,
/*  bmAttributes							*/ 0x80, /* bit 6 set = bus powered = 0x80, 0xC0 is for self powered */
/*  bMaxPower								*/ 0x32, /* Value x 2mA, set to 0 for self powered, was 0x32 */
/*********************************************************************
  Interface 0 - Communications Class
 *********************************************************************/
/* bLength                                  */ 0x09,
/* bDescriptorType: INTERFACE               */ 0x04,
/* bInterfaceNumber                         */ 0x00,
/* bAlternateSetting                        */ 0x00,
/* bNumEndpoints: 0 endpoint(s)             */ 0x01,
/* bInterfaceClass: CDC                     */ 0x02,
/* bInterfaceSubclass:Abstract Control Model*/ 0x02,
/* bInterfaceProtocol:AT Commands V.25ter   */ 0x01,
/* iInterface                               */ 0x00,
/*********************************************************************
 Header Functional Descriptor
 *********************************************************************/
/* bLength                                  */ 0x05,
/* bDescriptorType: CS_INTERFACE            */ 0x24,
/* bDescriptorSubtype: HEADER FD            */ 0x00,
/* bcdADC                                   */ 0x10,0x01,
/*********************************************************************
 Abstract Control Model Functional Descriptor
 *********************************************************************/
/* bLength                                  */ 0x04,
/* bDescriptorType: CS_INTERFACE            */ 0x24,
/* bDescriptorSubtype: ACM-FD               */ 0x02,
/* bmCapabilities                           */ 0x02,
/*********************************************************************
 Union Functional Descriptor
 *********************************************************************/
/* bLength                                  */ 0x05,
/* bDescriptorType: CS_INTERFACE            */ 0x24,
/* bDescriptorSubtype: Union FD             */ 0x06,
/* bControlInterface                        */ 0x00,
/* bSubordinateInterface                    */ 0x01,
/*********************************************************************
 Call Management Functional Descriptor
 *********************************************************************/
/* bLength                                  */ 0x05,
/* bDescriptorType: CS_INTERFACE            */ 0x24,
/* bDescriptorSubtype: CM-FD                */ 0X01,
/* bmCapabilities                           */ 0x00,
/* bDataInterface                           */ 0x01,
/*********************************************************************
 Interrupt IN Endpoint - 1
 *********************************************************************/
/* bLength                                  */ 0x07,
/* bDescriptorType: ENDPOINT                */ 0x05,
/* bEndpointAddress: IN Endpoint 1          */ 0x81,
/* bmAttributes: INTERRUPT                  */ 0x03,
/* max packet size (LSB)                    */ 0x08,
/* max packet size (MSB)                    */ 0x00,
/* polling interval                         */ 0xFF,
/*********************************************************************
 Interface 1 - Data Class Interface
 *********************************************************************/
/* bLength                                  */ 0x09,
/* bDescriptorType: INTERFACE               */ 0x04,
/* interface index                          */ 0x01,
/* alt setting index                        */ 0x00,
/* bNumEndpoints: 2 endpoint(s)             */ 0x02,
/* bInterfaceClass: CDC-Data                */ 0x0A,
/* bInterfaceSubclass: unused               */ 0x00,
/* bInterfaceProtocol: None                 */ 0x00,
/* iInterface                               */ 0x00,
/*********************************************************************
 Endpoint 2 (Bulk IN)
 *********************************************************************/
/* bLength                                  */ 0x07,
/* bDescriptorType: ENDPOINT                */ 0x05,
/* bEndpointAddress: IN Endpoint 2          */ 0x82,
/* bmAttributes: BULK                       */ 0x02,
/* max packet size (LSB)                    */ 0x40,
/* max packet size (MSB)                    */ 0x00,
/* bInterval: None for BULK                 */ 0x00,
/*********************************************************************
 Endpoint 3 (Bulk OUT)
 *********************************************************************/
/* bLength                                  */ 0x07,
/* bDescriptorType: ENDPOINT                */ 0x05,
/* bEndpointAddress: OUT Endpoint 3         */ 0x03,
/* bmAttributes: BULK                       */ 0x02,
/* max packet size (LSB)                    */ 0x40,
/* max packet size (MSB)                    */ 0x00,
/* bInterval: None for BULK                 */ 0x00
};

/*********************************************************************
 String Descriptor Table (below) - These must be Unicode 16 (UTF-16)
 *********************************************************************/

 /*********************************************************************
Language ID Descriptor
*********************************************************************/
unsigned char string0[] = {
/* 	Descriptor Length                       */	4,
/*  DescriptorType: STRING  				*/	0x03,
/* 	Language ID: English					*/	0x09,0x04
												}; // 0x0409
/* *******************************************************************
String Descriptor: "Lewis Technologies Group"
*********************************************************************/
unsigned char string1[] = {
/* 	Descriptor Length                       */	38,
/*  DescriptorType: STRING  				*/  0x03,
/*	Vendor Name     						*/	'P',0,'I',0,'C',0,'3',0,'2',0,
'M',0,'Z',0,' ',0,'C',0,'D',0,'C',0,' ',0,'A',0,'C',0,'M',0,' ',0,'v',0,'2',0
};

/*********************************************************************
 String Descriptor (Product Name): "USB Example"
 *********************************************************************/
 unsigned char string2[] = {
 /* Descriptor Length                       */ 	40,
/*  DescriptorType: STRING  				*/  0x03,
 /* Product Name                            */	'V',0,'i',0,'r',0,'t',0,
'u',0,'a',0,'l',0,' ',0,'S',0,'e',0,'r',0,'i',0,'a',0,'l',0,' ',0,'P',0,
'o',0,'r',0,'t',0
 };

/*********************************************************************
 String Descriptor (Serial Number): "xxx-xxxx"
*********************************************************************/
 unsigned char string3[] = {
 /* Descriptor Length                       */ 	18,
/*  DescriptorType: STRING  				*/  0x03,
 /* Serial Number                           */	'1',0,'2',0,'3',0,'-',0,
'4',0,'5',0,'6',0,'7',0
 };

 unsigned char string4[] = {
 /* Descriptor Length                       */ 	4,
/*  DescriptorType: STRING  				*/  0x03,
 /* Configuration                           */	0x01, 0x00
 };

 unsigned char string5[] = {
 /* Descriptor Length                       */ 	10,
/*  DescriptorType: STRING  				*/  0x03,
 /* Interface                               */	'T',0,'e',0,'s',0,'t',0
 };

#endif /* _USB_DESCRIPTORS_H */

/* *****************************************************************************
 End of File
 */
