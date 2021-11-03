/*
 * File name: usb_cdc_acm.c for PIC32MZ Basic USB CDC ACM, version 2.0
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

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/attribs.h>
#include <sys/kmem.h>
#include "usb.h"
#include "usb_descriptors.h"

void __ISR_AT_VECTOR(_USB_VECTOR, IPL4SRS) _USB_Handler(void){
    
    if(USBCSR2bits.RESETIF){ // In Device mode, indicates reset signaling is detected on the bus.
        usbReset = TRUE; // Indicate the bus was reset and should be IDLE now.
        
        USBE1CSR0bits.MODE = 1;     // 1 = Endpoint is TX.
        USBE2CSR0bits.MODE = 1;     // 1 = Endpoint is TX
        USBE3CSR0bits.MODE = 0;     // 0 = Endpoint is RX.
        
        // Note that USB reset will reset a lot of the registers and you should 
        // do most initialization in the USB reset interrupt handler.
        USBE0CSR0bits.TXMAXP = EP0BUFFSIZE;
        USBE0CSR2bits.SPEED = 1;        // Endpoint 0 Operating Speed Control bits
        USBE1CSR2bits.SPEED = 1;        // Endpoint 1: TX Endpoint Operating Speed Control bits - High speed
        USBE2CSR2bits.SPEED = 1;        // Endpoint 2: TX Endpoint Operating Speed Control bits - High speed
        USBE3CSR3bits.SPEED = 1;        // Endpoint 3: RX Endpoint Operating Speed Control bits - High speed
        
        /* These are multiples of 8, e.g.- "1" is 8 bytes, "8" is 64 bytes */
        USBE1CSR0bits.TXMAXP = EP1TXBUFFSIZE;   // Endpoint 1 - Maximum TX Payload Per Transaction Control bits
        USBE2CSR0bits.TXMAXP = EP2TXBUFFSIZE;   // Endpoint 2 - Maximum TX Payload per transaction Control bits
        USBE3CSR1bits.RXMAXP = EP3RXBUFFSIZE;   // Endpoint 3 - Maximum RX Payload Per Transaction Control bits
         
        USBE1CSR2bits.PROTOCOL = 3; // Endpoint 1 - TX Endpoint Protocol Control bits 
        USBE2CSR2bits.PROTOCOL = 2; // Endpoint 2 - TX Endpoint Protocol Control bits
        USBE3CSR3bits.PROTOCOL = 2; // Endpoint 3 - RX Endpoint Protocol Control bits
        //PROTOCOL<1:0>: RX/TX Endpoint Protocol Control bits 
        //11 = Interrupt
        //10 = Bulk
        //01 = Isochronous
        //00 = Control
        
        USBCSR1bits.EP1TXIE = 1;    // Endpoint 1 TX interrupt enable
        USBCSR1bits.EP2TXIE = 1;    // Endpoint 2 TX interrupt enable
        USBCSR2bits.EP3RXIE = 1;    // Endpoint 3 RX interrupt enable
        
        USBCSR2bits.RESETIF = 0;
    } 
    
    /* Endpoint 0 Interrupt Handler */
    if(USBCSR0bits.EP0IF == 1){
        //led_2 = 1;
        usbReset = FALSE;
        
        if(setDevAddr == TRUE){ // Set Address, upon first IN transaction
            setAddress(usbAddress);
            setDevAddr = FALSE;
        }
        
        if(USBE0CSR0bits.RXRDY){
            ep0rbc = USBE0CSR2bits.RXCNT; // Endpoint 0 - Received Bytes Count
            tempData = (uint8_t *)&USBFIFO0;
               
                for(i = 0; i < ep0rbc; i++){
                    ep0data[i] = *(tempData + (i & 3));
                }
            USBE0CSR0bits.RXRDYC = 1;
               
            // The packet length of control transfers in low speed devices must be 
            // 8 bytes, high speed devices allow a packet size of 8, 16, 32 or 64 bytes 
            // and full speed devices must have a packet size of 64 bytes.
            bmbRequest = (ep0data[0] << 8) | ep0data[1]; // bmRequestType, bRequest
            wValue = (ep0data[3] << 8) | ep0data[2];
            wIndex = (ep0data[5] << 8) | ep0data[4];
            wLength = (ep0data[7] << 8) | ep0data[6];
            
            controlTrans();  // Process Control Transfers and Packets.
                
        }
        
        
        if(USBE0CSR0bits.SETEND){
                USBE0CSR0bits.SETENDC = 1;
        }
    
    USBCSR0bits.EP0IF = 0;  // Clear the USB EndPoint 0 Interrupt Flag.
    } 
    
    /* Endpoint 1 TX Interrupt Handler */
    if(USBCSR0bits.EP1TXIF == 1){ // INTERRUPT Endpoint 1 Transmit A Packet.
        //
    USBCSR0bits.EP1TXIF = 0;        // Supposedly Cleared By Hardware (Clear Just In Case).
    } 
    
    /* Endpoint 2 TX Interrupt Handler */
    if(USBCSR0bits.EP2TXIF == 1){ // BULK Endpoint 2 Transmit A Packet.
        ep2BlockData = &ep3data[0];
        txBlock_ep2(sizeof(ep2BlockData));
    USBCSR0bits.EP2TXIF = 0;        // Supposedly Cleared By Hardware (Clear Just In Case). 
    } 
    
    /* Endpoint 3 RX Interrupt Handler */
    if(USBCSR1bits.EP3RXIF == 1){ // BULK Endpoint 3 Received A Packet.
        rxUSB_ep3();
        
        ep2BlockData = ep3data;
        txByte_ep2(0);
        
    USBCSR1bits.EP3RXIF = 0;    
    }

       
       
    IFS4bits.USBIF = 0;  // Reset the USB Interrupt flag
}

void initUSB(){
    // Configure the USB hardware and registers.
    USBCSR0bits.SOFTCONN = 0;   // 1 = The USB D+/D- lines are enabled and active, 
                                // 0 = The USB D+/D- lines are disabled and are tri-stated.
    
    USBE1CSR0bits.MODE = 1;     // 1 = Endpoint is TX.
    USBE2CSR0bits.MODE = 1;     // 1 = Endpoint is TX
    USBE3CSR0bits.MODE = 0;     // 0 = Endpoint is RX.
    
    IFS4bits.USBIF = 0;         // Clear the USB interrupt flag.
    /* Clear the Endpoint interrupt flags */
    USBCSR0bits.EP0IF = 0;
    USBCSR0bits.EP1TXIF = 0;
    USBCSR0bits.EP2TXIF = 0;
    USBCSR1bits.EP3RXIF = 0;

    IPC33bits.USBIP = 4;        // USB Interrupt Priority.
    IPC33bits.USBIS = 0;        // USB Interrupt Sub-Priority.
    
    usbAddress = 0;
    USBCSR0bits.FUNC = 0;       // Initially set the USB address to 0 until
                                 // later when the host assigns an address
        
    // Configure endpoint 0 first.
    USBE0CSR0bits.TXMAXP = EP0BUFFSIZE; // Set endpoint 0 buffer to 64 bytes (multiples of 8).
    
    /* These are multiples of 8, e.g.- "1" is 8 bytes, "8" is 64 bytes */
    USBE1CSR0bits.TXMAXP = EP1TXBUFFSIZE;   // Endpoint 1 - Maximum TX Payload Per Transaction Control bits
    USBE2CSR0bits.TXMAXP = EP2TXBUFFSIZE;   // Endpoint 2 - Maximum TX Payload per transaction Control bits
    USBE3CSR1bits.RXMAXP = EP3RXBUFFSIZE;   // Endpoint 3 - Maximum RX Payload Per Transaction Control bits
    
    USBE1CSR2bits.PROTOCOL = 3; // Endpoint 1 - TX Endpoint Protocol Control bits 
    USBE2CSR2bits.PROTOCOL = 2; // Endpoint 2 - TX Endpoint Protocol Control bits
    USBE3CSR3bits.PROTOCOL = 2; // Endpoint 3 - RX Endpoint Protocol Control bits
        //PROTOCOL<1:0>: RX/TX Endpoint Protocol Control bits 
        //11 = Interrupt
        //10 = Bulk
        //01 = Isochronous
        //00 = Control
    
    // See datasheet DS60001320E-page 228:
    USBE1CSR1bits.PIDERR = 1; // See DISNYET (same bit as PIDERR).
    USBE2CSR1bits.PIDERR = 1; // See DISNYET (same bit as PIDERR).
    USBE3CSR1bits.PIDERR = 1; // See DISNYET (same bit as PIDERR).
        // DISNYET: Disable NYET Handshakes Control/PID Error Status bit (Device mode)
        // 1 = In Bulk/Interrupt transactions, disables the sending of NYET handshakes. 
        // All successfully received RX packets are ACKed including at the point at 
        // which the FIFO becomes full. 
        // 0 = Normal operation.
        // In Bulk/Interrupt transactions, this bit only has any effect in Hi-Speed mode, 
        // in which mode it should be set for all Interrupt endpoints.
        //
        // PIDERR: PID Error Status bit (Host mode)
        // 1 = In ISO transactions, this indicates a PID error in the received packet. 
        // 0 = No error
    
    USBCSR3bits.ENDPOINT = 1;
    USBFIFOAbits.TXFIFOAD = 0x0000; // Receive Endpoint FIFO Address bits
    
    USBCSR3bits.ENDPOINT = 2;
    USBFIFOAbits.TXFIFOAD = 0x0040; // Transmit Endpoint FIFO Address bits
    
    USBCSR3bits.ENDPOINT = 3;
    USBFIFOAbits.RXFIFOAD = 0x0080; // Receive Endpoint FIFO Address bits
    
    // See data sheet DS60001320E-page 232
    USBE1CSR3bits.TXFIFOSZ = 0x4;   // Transmit FIFO Size bits - 16 bytes
    USBE2CSR3bits.TXFIFOSZ = 0x6;   // Transmit FIFO Size bits - 64 bytes
    USBE3CSR3bits.RXFIFOSZ = 0x6;   // Receive FIFO Size bits - 64 bytes
        // 1101 = 8192 bytes 
        // 1100 = 4096 bytes
        // . 
        // . 
        // .
        // 0011 = 8 bytes
        // 0010 = Reserved
        // 0001 = Reserved
        // 0000 = Reserved or endpoint has not been configured
    
    USBE1CSR0bits.ISO = 0;      // Isochronous TX Endpoint Disable bit (Device mode).
    USBE2CSR0bits.ISO = 0;      // Isochronous TX Endpoint Disable bit (Device mode).
    USBE3CSR1bits.ISO = 0;      // Isochronous RX Endpoint Disable bit (Device mode).
    
    /* Enable Endpoint interrupts */
    USBCSR1bits.EP0IE = 1;      // Endpoint 0 interrupt enable
    USBCSR1bits.EP1TXIE = 1;    // Endpoint 1 TX interrupt enable
    USBCSR1bits.EP2TXIE = 1;    // Endpoint 2 TX interrupt enable
    USBCSR2bits.EP3RXIE = 1;    // Endpoint 3 RX interrupt enable
    
    USBCSR2bits.RESETIE = 1;    // Disabled. When enabled IN packets/Set Address 
                                // aren't received (unknown reason).
    
    
    IEC4bits.USBIE = 1;         // Enable the USB interrupt.
    USBCRCONbits.USBIE = 1;     // 1 = Enables general interrupt from USB module
    
    // USBHS_DeviceAttach_Default
    USBCSR0bits.HSEN = 1;       // 1 = Enable High Speed (480Mbps) USB mode.
}

uint8_t connectUSB(){
    USBCSR0bits.SOFTCONN = 1;   // 1 = The USB D+/D- lines are enabled and active
    return 0;
}

uint8_t disableUSB(){
    USBCSR0bits.SOFTCONN = 0;   // 0 = The USB D+/D- lines are disabled and are tri-stated.
    return 1;
}

void setAddress(volatile uint8_t usbAddress){
    // The Set_Address request changes the address after the status stage. 
    // You must update the FUNC field only after you get the status interrupt.
    USBCSR0bits.FUNC = usbAddress & 0x7F;
}

void controlTrans(){
    
    switch (bmbRequest) {
    
        case 0x8000:    // Get Status
            USBE0CSR0bits.STALL = 1;
            break;
           
        case 0x0005:    // Save Address
            USBE0CSR0bits.RXRDYC = 1;
            USBE0CSR0bits.DATAEND = 1;
            usbAddress = ep0data[2];
            setDevAddr = TRUE;
            break;
            
        case 0x0007:    // Set Descriptor
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0x8008:    // Get Configuration
            ep0BlockData = &usbConfiguration;
            txBlock_ep0(sizeof(ep0BlockData));
            break;
            
        case 0x0009:    // Set Configuration
            usbConfiguration = ep0data[2]; // wValue
            break;
            
        case 0x000A:    // Get Interface
            ep0BlockData = &usbInterface;
            txBlock_ep0(sizeof(ep0BlockData));
            break;
            
        case 0x000B:    // Set Interface
            usbInterface = ep0data[4]; // wIndex
            break;
            
        case 0x000C:    // Synchronize Frame
            USBE0CSR0bits.STALL = 1;
            break;     
            
            
        case 0x8100:    // Get Status (Interface)
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0x8200:    // Get Status (Endpoint)
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0x8002:    // Get Configuration
            ep0BlockData = &usbConfiguration;
            txBlock_ep0(sizeof(ep0BlockData));
            break;
        
        case 0x0006:    
        case 0x8006:{   // Get Descriptor(s)  
            switch (ep0data[3]) {
                case 0x00: { // Undefined
                    USBE0CSR0bits.STALL = 1;
                }
                break;
                
                case 0x01: { // Device Descriptor
                        //led_4r = 0;
                        ep0BlockData = dev_desc;
                        txBlock_ep0(sizeof(dev_desc));
                    }
                    break;

                case 0x02: { // Configuration Descriptor
                   /* Per the USB Specifications:
                    * A request for a configuration descriptor returns the 
                    * configuration descriptor, all interface descriptors, and 
                    * endpoint descriptors for all of the interfaces in a single 
                    * request. The first interface descriptor follows the 
                    * configuration descriptor. 
                    * 
                    * The endpoint descriptors for the first interface follow the 
                    * first interface descriptor. 
                    * 
                    * If there are additional interfaces, their interface 
                    * descriptor and endpoint descriptors follow the first 
                    * interface's endpoint descriptors. 
                    * 
                    * Class-specific and/or vendor-specific descriptors follow the 
                    * standard descriptors they extend or modify.
                    * 
                    * All devices must provide a device descriptor and at least one 
                    * configuration descriptor. If a device does not support a 
                    * requested descriptor, it responds with a Request Error.
                    */   
                    ep0BlockData = CDC_ConfigDescriptorArray;
                    txBlock_ep0(wLength);
                    }
                    break;

                case 0x03: { // String Descriptors
                        switch (ep0data[2]) {  
                            case 0x00: { // Language ID
                                ep0usbData = string0;
                                txUSB_ep0(sizeof(string0));
                            }
                                break;

                            case 0x01: { // Manufacturer
                                ep0usbData = string1;
                                txUSB_ep0(sizeof(string1));
                            }
                                break;

                            case 0x02: { // Product
                                ep0usbData = string2;
                                txUSB_ep0(sizeof(string2)); 
                            }
                                break;

                            case 0x03: { // Serial
                                ep0usbData = string3;
                                txUSB_ep0(sizeof(string3));
                            }
                                break;
                          
                            case 0x04: { // Configuration
                                ep0usbData = string4;
                                txUSB_ep0(sizeof(string4));
                            }
                                break;
                                
                            case 0x05: { // Interface
                                ep0usbData = string5;
                                txUSB_ep0(sizeof(string5));
                            }
                                break;
                                
                        } // End of case ep0data[2] switch
                        break;
                        
                    } // End of case 0x03 switch - String Descriptors
                break;
                
                case 0x04: {  // Get Interface
                    switch (ep0data[2]){
                        //case 0x00:
                            //break;
                            
                        case 0x00:
                            USBE0CSR0bits.STALL = 1;
                            break;
                            
                        case 0x01:
                            USBE0CSR0bits.STALL = 1;
                            break;
                    }
                    
                } // End of case 0x04 switch
                    break;
                    
                case 0x05: {  // Get Endpoint
                    switch (ep0data[2]){
                        //case 0x00:
                                
                            //break;
                            
                        case 0x00:
                            USBE0CSR0bits.STALL = 1;
                            break;
                            
                        case 0x01:
                            USBE0CSR0bits.STALL = 1;
                            break;
                    }
                } // End of case 0x05 switch
                    
                    break;
                    
                case 0x06: // Get Device Qualifier
                    
                    break;
                    
                case 0x07:  // Other Speed COnfiguration
                    
                    break;
                } // End of switch ep0data[3].
            
        } // End of case 0x8006 switch - Get Descriptor(s).
            
        case 0x810A: // Get Interface
            ep0BlockData = &usbInterface;
            txBlock_ep0(sizeof(usbInterface));
            break;
            
           
        /*********************************************************************
         CDC ACM Class Request
        *********************************************************************/
        case 0x2100: // SEND_ENCAPSULATED_COMMAND (Required for CDC ACM) - INTERFACE
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0x2101: // GET_ENCAPSULATED_RESPONSE (Required for CDC ACM) - INTERFACE
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0x2120: // SET_LINE_CODING - INTERFACE
            rxUSB_ep0(wLength);
            serialState = ep0usbTemp;
            dwDTERate = ((serialState[3] << 24) | (serialState[2] << 16) | (serialState[1] << 8) | serialState[0]); // BAUD
            bCharFormat = serialState[4] & 0x03; // Stop Bits - 7, 8, etc
            bParityType = serialState[5] & 0x07; // Parity
            bDataBits = serialState[6] & 0x1F; // Data bits
            break;
            
        case 0x2121: // GET_LINE_CODING - INTERFACE
            // Requests current DTE rate, stop-bits, parity, and number-of-character bits.
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0x2122: // SET_CONTROL_LINE_STATE - INTERFACE (called when host/terminal disconnects)
            // RS232 signal used to tell the DCE device the DTE device is now present.
            // NOTE: This is briefly (quickly) called when the device goes on/offline 
            controlState = wValue;
            /* Send initial message to the terminal/console */
            if(controlState != 0){
                // NOTE: When controlState is zero (0) the CDC device is offline (when the terminal is disconnected).
                // This message is sent each time the device goes online (when the terminal connects to it).
                char message[] = "PIC32MZ connected. Welcome!\n";
                ep2BlockData = message;
                sendMessage_ep2();
            }
            break;
            
        case 0xA101: // RESPONSE_AVAILABLE (required for CDC ACM)
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0xA120: // SERIAL STATE
            USBE0CSR0bits.STALL = 1;
            break;
            
        case 0xA121: // GET_LINE_CODING - INTERFACE
            USBE0CSR0bits.STALL = 1;
            break;
            
        default:
            USBE0CSR0bits.STALL = 1;
            break;
    }

    return;
}

void txUSB_ep0(uint32_t ep0TXLen){
            volatile uint8_t *ep0fifo = NULL;
            ep0fifo = (uint8_t *)&USBFIFO0;
            
            for(ep0ArrPos = 0; ep0ArrPos < ep0TXLen; ep0ArrPos++){
                *ep0fifo = ep0usbData[ep0ArrPos];
            }
            
                USBE0CSR0bits.TXRDY = 1;
}

int txBlock_ep0(uint16_t NumBytes_ep0){
    
    volatile uint8_t *ep0fifo = NULL;
    ep0fifo = (uint8_t *)&USBFIFO0;
    uint16_t ep0ArraySize = packetSize_ep0;
    char setEnd = FALSE;
    
    packetSize_ep0 = NumBytes_ep0;
    if(packetSize_ep0 > EP0_MAX_PACKET_SIZE)
        {
            packetSize_ep0 = EP0_MAX_PACKET_SIZE;
        }
    ep0RemainingBytes = NumBytes_ep0; 

    ep0ArrPos = 0; // Reset the array position.
    do{
        
            int timeout = 0;
            do{
                timeout++;
            }while(USBE0CSR0bits.TXRDY && timeout < 400);
            
            if(timeout >= 400){
                return -1;
            }
            
            if(ep0RemainingBytes < packetSize_ep0){ // Check if this is  last packet
                packetSize_ep0 = ep0RemainingBytes;
                setEnd = TRUE;
            }
            
            for(i = 0; i < packetSize_ep0; i++) {
                *ep0fifo = ep0BlockData[ep0ArrPos++];
                ep0RemainingBytes--;
            }
            
            if(setEnd == TRUE)
                {
                    USBIE0CSR0bits.DATAEND = 1; // End of Data Control bit (Device mode) 
                    // The software sets this bit when:
                    // * Setting TXPKTRDY for the last data packet
                    // * Clearing RXPKTRDY after unloading the last data packet 
                    // * Setting TXPKTRDY for a zero length data packet
                    // Hardware clears this bit.
                }
            USBE0CSR0bits.TXRDY = 1;
            
    }while(ep0RemainingBytes != 0);
    
return 0;
}

int txBlock_ep1(uint16_t NumBytes_ep1){
    
    volatile uint8_t *ep1fifo = NULL;
    ep1fifo = (uint8_t *)&USBFIFO1;
    uint16_t ep1ArraySize = sizeof(ep1BlockData);
    char setEnd = FALSE;
    
    packetSize_ep1 = NumBytes_ep1;
    if(packetSize_ep1 > EP1_MAX_PACKET_SIZE)
        {
            packetSize_ep1 = EP1_MAX_PACKET_SIZE;
        }
    ep1RemainingBytes = NumBytes_ep1;

    ep1ArrPos = 0; // Reset the array position.
    do{
        
            int timeout = 0;
            do{
                timeout++;
            }while(USBE1CSR0bits.TXPKTRDY && timeout < 400);
            
            if(timeout >= 400){
                return -1;
            }
            
            if(ep1RemainingBytes < packetSize_ep1){ // Check if this is  last packet
                packetSize_ep1 = ep1RemainingBytes;
                setEnd = TRUE;
            }
            
            for(i = 0; i < packetSize_ep1; i++) {
                *ep1fifo = ep1BlockData[ep1ArrPos++];
                ep1RemainingBytes--;
            }
            
            if(setEnd == TRUE && packetSize_ep1 == EP1_MAX_PACKET_SIZE)
                {
                    // Set TXPKTRDY for a zero length data packet
                    // when the last TX data equals the max. packet size
                    // Hardware clears this bit.
                
                    *ep1fifo = (uint32_t)NULL;
                }
            
            USBE1CSR0bits.TXPKTRDY = 1;
            
            
            
    }while(ep1RemainingBytes != 0);
    
return 0;
}

void txByte_ep2(uint16_t ep2ArrPos){
            volatile uint8_t *ep2fifo = NULL;
            ep2fifo = (uint8_t *)&USBFIFO2;
            
            int timeout = 0;
            do{
                timeout++;
            }while(USBE2CSR0bits.FIFONE && timeout < 800);
                
                *ep2fifo = (uint8_t)ep2BlockData[ep2ArrPos];
            
                USBE2CSR0bits.TXPKTRDY = 1;
}

int txBlock_ep2(uint16_t NumBytes_ep2){
    
    volatile uint8_t *ep2fifo = NULL;
    ep2fifo = (uint8_t *)&USBFIFO2;
    uint16_t ep2ArraySize = sizeof(ep2BlockData);
    char setEnd_ep2 = FALSE;
    
    packetSize_ep2 = NumBytes_ep2;
    if(packetSize_ep2 > EP2_MAX_PACKET_SIZE)
        {
            packetSize_ep2 = EP2_MAX_PACKET_SIZE;
        }
    ep2RemainingBytes = NumBytes_ep2;

    ep2ArrPos = 0; // Reset the array position.
    do{
        
            int timeout_ep2 = 0;
            do{
                timeout_ep2++;
            }while(USBE2CSR0bits.TXPKTRDY && timeout_ep2 < 400);
            
            if(timeout_ep2 >= 400){
                return -1;
            }
            
            if(ep2RemainingBytes < packetSize_ep2){ // Check if this is  last packet
                packetSize_ep2 = ep2RemainingBytes;
                setEnd_ep2 = TRUE;
            }
            
            for(i2 = 0; i2 < packetSize_ep2; i2++) {
            
                *ep2fifo = (uint8_t)ep2BlockData[ep2ArrPos++];
                ep2RemainingBytes--;
            }
            
            if(setEnd_ep2 == TRUE && packetSize_ep2 == EP2_MAX_PACKET_SIZE)
                {
                    // Set TXPKTRDY for a zero length data packet
                    // when the last TX data equals the max. packet size
                    // Hardware clears this bit.
                    *ep2fifo = (uint32_t)NULL;
                }
            
            USBE2CSR0bits.TXPKTRDY = 1;
            
    }while(ep2RemainingBytes != 0);
    
return 0;
}

int rxUSB_ep0(volatile int ep0rbc){
    timeout_ep0 = 0;
            do{
                timeout_ep0++;
            }while(!USBE0CSR0bits.RXRDY && timeout_ep0 < 8000);
            
    tempData = (uint8_t *)&USBFIFO0; // The FIFO is 32 bits wide, we only want 8 bits at a time.
    for(i = 0; i < ep0rbc; i++){
        ep0usbTemp[i] = *(tempData + (i & 3)); // Store the received bytes in array ep0data[].
    }
    USBE0CSR0bits.RXRDYC = 0; // 0 = Written by software to clear this bit 
                                // when the packet has been unloaded from the RX FIFO.
}

void rxUSB_ep3(){
    ep3rbc = USBE3CSR2bits.RXCNT;  // Endpoint 3 - Received Bytes Count
    tempData = (uint8_t *)&USBFIFO3; // The FIFO is 32 bits wide, we only want 8 bits at a time.
    for(i = 0; i < ep3rbc; i++){
        ep3data[i] = *(tempData + (i & 3)); // Store the received bytes in array ep3data[].
    }
    USBE3CSR1bits.RXPKTRDY = 0; // 0 = Written by software to clear this bit 
                                // when the packet has been unloaded from the RX FIFO.
}

void sendMessage_ep2(){
    /* Send a message to the USB terminal console */
    uint32_t txSize = (sizeof(ep2BlockData) * 8) - 4;
    txBlock_ep2(txSize);
}