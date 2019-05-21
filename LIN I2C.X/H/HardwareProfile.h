/********************************************************************
 FileName:      HardwareProfile.h
 Dependencies:  See INCLUDES section
 Processor:     PIC18, PIC24, or PIC32 USB Microcontrollers
 Hardware:      The code is natively intended to be used on the 
                  following hardware platforms: 
                    PICDEM™ FS USB Demo Board
                    PIC18F46J50 FS USB Plug-In Module
                    PIC18F87J50 FS USB Plug-In Module
                    Explorer 16 + PIC24 or PIC32 USB PIMs
                    PIC24F Starter Kit
                    Low Pin Count USB Development Kit
                  The firmware may be modified for use on other USB 
                    platforms by editing this file (HardwareProfile.h)
 Compiler:  	Microchip C18 (for PIC18), C30 (for PIC24), 
                  or C32 (for PIC32)
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
********************************************************************/

#define HARDWARE_PROFILE_H

//#define DEMO_BOARD USE

#include <math.h>




#define  SYS_CLOCK 80000000
#define GetSystemClock()            SYS_CLOCK
#define GetPeripheralClock()        SYS_CLOCK
#define GetInstructionClock()       GetSystemClock()

#define LED1_TRIS		_TRISA5
#define LED2_TRIS		_TRISD5
#define LED_LIN1_TRIS		_TRISD8
#define LED_LIN2_TRIS		_TRISD12

#define LEDHB_TRIS		_TRISF12
#define LEDBL_TRIS		_TRISD1

#define LIN1TX_TRIS		_TRISD15

#define LIN2TX_TRIS		_TRISA1
#define LIN2CS_TRIS		_TRISA0

#define U2TX_TRIS		_TRISA6

#define PI_ON_TRIS		_TRISA4
#define PI_SW_TRIS		_TRISG14
#define PI_RST_TRIS		_TRISF13

#define LED1		_LATA5
#define LED2		_LATD5
#define LED_LIN1		_LATD8
#define LED_LIN2		_LATD12

#define LEDHB		_LATF12
#define LEDBL		_LATD1

#define LIN1TX		_LATD15
#define LIN2TX		_LATA1
#define LIN2CS		_LATA0

#define U2TX		_LATA6

#define PI_ON		_LATA4
#define PI_SW		_LATG14


#define PI_RST		_LATF13

 //TRISDbits.TRISD14 







   //     SSD1306 SPI
#define TRIS_DC_PIN                TRISAbits.TRISA0//PORTB |= 0x01  
#define DC_PIN           			LATAbits.LATA0  //PORTB &= ~0x01
 
#define TRIS_CS_PIN              TRISAbits.TRISA1// 
#define CS_PIN        			LATAbits.LATA1   //
        
 
#define TRIS_RST_PIN              TRISAbits.TRISA4// PORTB |= 0x10
#define RST_PIN         			LATAbits.LATA4   // PORTB |= 0x10




// TFT not used
// eval T2CK 
#define _dc         LATCbits.LATC1
#define TRIS_dc     TRISCbits.TRISC1
#define _dc_high()  {LATCSET = 2;}
#define _dc_low()   {LATCCLR = 2;}

// eval T3CK
#define _cs         LATCbits.LATC2
#define TRIS_cs     TRISCbits.TRISC2
#define _cs_high()  {LATCSET = 4;}
#define _cs_low()   {LATCCLR = 4;}

// eval T4CK
#define _rst        LATCbits.LATC3
#define TRIS_rst    TRISCbits.TRISC3
#define _rst_high() {LATCSET = 8;}
#define _rst_low()  {LATCCLR = 8;}




    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0
    #define self_power          1

    //#define USE_USB_BUS_SENSE_IO
   // #define tris_usb_bus_sense  TRISBbits.TRISB5    // Input
    #define USB_BUS_SENSE       1 
