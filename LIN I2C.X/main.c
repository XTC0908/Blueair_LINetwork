/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

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
********************************************************************/

#ifndef JOYSTICKMAIN_C
#define JOYSTICKMAIN_C

/** INCLUDES *******************************************************/
//#include "./USB/usb.h"
#include "HardwareProfile.h"
//#include "./USB/usb_function_hid.h"
//#include "I2Ctnk.h"

//#include "SPItank.h"
//#include <plib.h>
//#include <p32xxxx.h>
//#include <peripheral\system.h>
//#include <pwm.h>
//#include "TimeDelay.h"
/** CONFIGURATION **************************************************/

#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1, WDTPS=PS512
#pragma config ICESEL   = ICS_PGx1      // ICE/ICD Comm Channel Select
       #pragma config UPLLEN   = ON        // USB PLL Enabled
        #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider

    #pragma config DEBUG    = OFF           // Debugger Disabled for Starter Kit





#define SYS_FREQ 				(80000000L)
/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
BOOL Switch2IsPressed(void);
BOOL Switch3IsPressed(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void Joystick(void);


char buff[20],bfr[3];


// this is the modules Slave Address
#define SLAVE_ADDRESS 0x40

// volatile variables to hold the switch and led states
volatile unsigned char  dataVoutH = 0,dataVoutL = 0,dataIoutH=0,dataIoutL=0,dataTemp=0, dataVin=0, dataStatus=0,dataselect=0;
UINT16 dataVout = 0,dataIout=0;
// I2C defs0
 
#define SYSCLK	(80000000)
#define PBCLK  (SYSCLK)
#define I2C_baud	40000
#define BRG_VAL 	((PBCLK/2/I2C_baud)-2)
// UART defs

#define DESIRED_BAUDRATE              (19200)      //The desired BaudRate


void PutCharacter1(const char);
void WriteString1(const char *string);
void ADCread (void);
void PutCharacter2(const char);
void WriteString2(const char *string);
char LIN2RXBUF[11] = {0,0,0,0,0,0,0,0,0,0};
char LIN2RXindex=0;
char myBufOFF[4] = {0x55, 0x73, 0xFE, 0x8D};
char myBufON[4] = {0x55, 0x73, 0xFF, 0x8C};

char *txPtr;
char last2=0, data2=0;
char txsw=0;
char SLAVEctr=0xFE,chksum=0;
char scheduleCTR=0;
char TXEn=1;
char RXed=0;
int timeout=5;
int i=0;


char temp;
unsigned int dIndex;
char I2CS=0;
char LIN1_error;
float B_VIN=12;
int under_voltage_timer=0,under_voltage_timer_limit=10;// 1s UV timer limit
int LIN1_timeout=0;


int main(void)

{   

    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    InitializeSystem();

	DDPCON=0; 
	mJTAGPortEnable(0);

   
//AD init
	AD1PCFGSET = 0x00ff;
	AD1CON1=0x00E0;//SSRC=111 autosample start 

	AD1CHS=0x00020000;//ch0 AN2 at CH0
	AD1CSSL=0xFFFF;//scan all
	AD1CON3=0x9F3F;			//2TAD sampling
	AD1CON2=0x043c;		//scan inputs;Select l Vref+- pins AVDD AVSS INTERRUPT after 16 smpl
	AD1CON1SET=0x8000; //turn on AD

	txPtr = myBufOFF;
	
	//TRISA=0;
	
//       POR reset for MCP2021  
	LIN2CS_TRIS	=0;
	LIN2CS=0;
	DelayMs(2);
	LIN2CS=1;
	DelayMs(2);
	
	LED2_TRIS=0;
	LED1_TRIS=0;
	LED_LIN1_TRIS=0;
	LED_LIN2_TRIS=0;
	LEDHB_TRIS=0;
	LEDBL_TRIS=0;
		
	PI_ON_TRIS=0;
	PI_SW_TRIS=0;
	
	LED1=1;
	LED2=1;
	LED_LIN1=1;
	LED_LIN2=1;
	
	LEDHB=1;
	
	PI_ON=1;

	LEDHB=1;


	PI_RST_TRIS=0;
	PI_RST=0;
// PI SUPERCAP CHARGE
   for (i=1; i<=60; i++)
   {
	PI_SW=1;
		DelayMs(2);
		LEDHB=!LEDHB;
	PI_SW=0;
		DelayMs(50); 
		LEDHB=!LEDHB;
	      
   }
	
   for (i=1; i<=20; i++)
   {
	PI_SW=1;
		DelayMs(25);
		LEDHB=!LEDHB;
	PI_SW=0;
		DelayMs(100); 
		LEDHB=!LEDHB;
	      
   }
		
	
	PI_SW=1;


	// LEDs off
	LEDHB=0;
	LED1=0;
	LED2=0;
	LED_LIN1=0;
	LED_LIN2=0;



// PI RESET
	DelayMs(100);
	PI_RST_TRIS=0;
	DelayMs(100);
	PI_RST=0;
	DelayMs(100);
	PI_RST=1;
	DelayMs(100);
	PI_RST_TRIS=1;
	DelayMs(100);
	
	U2TX_TRIS=0;
	TRISF=0;
	
	U2TX=0;
	//DelayMs(1);
	U2TX=1;
	//#define FeedbackLED			_LATA5
	
// I2C slave init
	InitI2C();
	
//LIN INIT

  // This initialization assumes 36MHz Fpb clock. If it changes,
  // you will have to modify baud rate initializer.
  UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART1, GetPeripheralClock(), DESIRED_BAUDRATE);
  UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // Configure UART1 RX Interrupt
  INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
  INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_3);
  INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);


  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART2, GetPeripheralClock(), DESIRED_BAUDRATE);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // Configure UART2 RX Interrupt
  INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
  INTSetVectorPriority(INT_VECTOR_UART(UART2), INT_PRIORITY_LEVEL_3);
  INTSetVectorSubPriority(INT_VECTOR_UART(UART2), INT_SUB_PRIORITY_LEVEL_0);
  // configure for multi-vectored mode
  //INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

		//
		
		
		
		//LIN1TX_TRIS=0;
		//LIN2TX_TRIS=0;
		//
		//LIN1TX=1;
		//LIN2TX=1;



	Timer1Init();	    /* Function is defined in TimerFunctions.c	*/

    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();


//INTEnable(INT_U1TX, INT_ENABLED);
//PutCharacter(0x00);




    while(1)
    {


		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{

	
//	mInitAllLEDs();
//INTEnableSystemMultiVectoredInt();
//INTEnableInterrupts();

 
//    UserInit();

    					//variables to known states.
}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
    //Initialize all of the LED pins


}//end UserInit


void ProcessIO(void)
{   
    //Blink the LEDs according to the USB device status


	if (I2CS)
	{

	I2CS=0;
	}



		if(IsOneSecondUp() == TRUE)
			{				    
			LEDHB=!LEDHB;
			}


		if(IsHUNDREDmiliSecondUp() == TRUE)
		{
		ADCread();
		
		B_VIN=ADC1BUF2*0.0357; //35mV / LSB

		if (B_VIN<7)
			{
			under_voltage_timer++;
						LEDHB=!LEDHB;
			}
			else
			{
			under_voltage_timer=0;
						LEDBL=0;
			}

		if (under_voltage_timer>under_voltage_timer_limit)
			{
			//PI_ON=0;
			//PI_SW=0;
			LED1=0;
			LED2=0;
			LED_LIN1=0;
			LED_LIN2=1;
			LEDHB=1;

				PI_RST_TRIS=0;
				PI_RST=0;
			//keeping RPI in reset for a minute after a power glitch longer than 1s
				   for (i=1; i<=600; i++)
				   {
						DelayMs(100);
						LEDHB=!LEDHB;
						LED_LIN1=!LED_LIN1;
						LED_LIN2=!LED_LIN2;
				   }
			//release RPI reset
				PI_RST=1;					
				PI_RST_TRIS=1;
				under_voltage_timer=0;			
				
			}
		}



		if(IsTENmiliSecondUp() == TRUE)
		{
		timeout++;
					
		   			 //if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
					//char myBufOFF[4] = {0x55, 0x73, 0xFE, 0x8D};
					//char myBufON[4] = {0x55, 0x73, 0xFF, 0x8C}
					
					
					//RX data SWITCH
					//LIN2RXBUF[LIN2RXindex]
					
					
					
				// LIN2 TX ENGINE			****************************************************				
					

					if (scheduleCTR>0)
					{


						txsw++;
						if (txsw>2)txsw=0;

						if (txsw==1)
						{
						  INTEnable(INT_SOURCE_UART_RX(UART2), INT_DISABLED);
							switch(scheduleCTR)
							{
							case 10:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x3C);// PID
								PutCharacter2(0x68);//NAD
								PutCharacter2(0x06);//PCI
								PutCharacter2(0xB2);//SID 
								PutCharacter2(0x38);//Supply Voltage 8bit 0.1V/bit
								PutCharacter2(0x07);//
								PutCharacter2(0x00);// 
								PutCharacter2(0x00);// 
								PutCharacter2(0x01);// 
								PutCharacter2(0x9E);// 						
							break;						
	
							case 9:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x7D);// PID		
							  INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);				
							break;	
		
							case 8:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x3C);// PID
								PutCharacter2(0x68);//NAD
								PutCharacter2(0x06);//PCI
								PutCharacter2(0xB2);//SID  
								PutCharacter2(0x39);//Output Voltage 16bit 1V/bit
								PutCharacter2(0x07);//
								PutCharacter2(0x00);// 
								PutCharacter2(0x00);// 
								PutCharacter2(0x01);// 
								PutCharacter2(0x9D);// 									
							break;
			
							case 7:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x7D);// PID				
							  INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);				
							break;	

							case 6:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x3C);// PID
								PutCharacter2(0x68);//NAD
								PutCharacter2(0x06);//PCI
								PutCharacter2(0xB2);//SID 
								PutCharacter2(0x3A);//Output Current 16bit 0.5uA/bit ? or 0.1uA/bit
								PutCharacter2(0x07);//
								PutCharacter2(0x00);// 
								PutCharacter2(0x00);// 
								PutCharacter2(0x01);// 
								PutCharacter2(0x9C);// 						
							break;						
	
							case 5:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x7D);// PID			
							  INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);			
							break;	
		
							case 4:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x3C);// PID
								PutCharacter2(0x68);//NAD
								PutCharacter2(0x06);//PCI
								PutCharacter2(0xB2);//SID 
								PutCharacter2(0x3B);//Output temp, 8bit 1C/bit 50C offset
								PutCharacter2(0x07);//
								PutCharacter2(0x00);// 
								PutCharacter2(0x00);// 
								PutCharacter2(0x01);// 
								PutCharacter2(0x9B);// 									
							break;
			
							case 3:
								U2STA=0x1d10;//set UT2BRK
								PutCharacter2(0x00);//SYNC byte
								PutCharacter2(0x55);//SYNC byte
								PutCharacter2(0x7D);// PID			
							  INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);					
							break;		
	
							case 2:
							U2STA=0x1d10;//set UT2BRK
							PutCharacter2(0x00);//dummy byte
							
							PutCharacter2(0x55);//SYNC byte
							PutCharacter2(0x73);// PID
							
							PutCharacter2(SLAVEctr);//0xFE off =xFF ON
							PutCharacter2(0x73^SLAVEctr); //ENHANCED CHKSUM XORING PID + DATA						
							break;
			
							case 1:
							U2STA=0x1d10;//set UT2BRK
							PutCharacter2(0x00);
							PutCharacter2(0x55);
							PutCharacter2(0x11);//request response 
							
							
							//STATUS OFF:0x70
							//STATUS ON:0x71
						INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);				
							break;	


							default:
							break;
							}
							scheduleCTR--;
							  //INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
						}
					}
					else 
					{



	
	
						txsw++;
						if (txsw>2)txsw=0;
						   			
						if (txsw==0)
						{
						  INTEnable(INT_SOURCE_UART_RX(UART2), INT_DISABLED);
							//myBuf[] = {0x55, 0x73, 0xFE, 0x8D}
							//  MASTER REQUEST
							//U2STA.UT2BRK = 1;
							//U2STASET=11;
							U2STA=0x1d10;//set UT2BRK
							PutCharacter2(0x00);//dummy byte
							
							PutCharacter2(0x55);//SYNC byte
							PutCharacter2(0x73);// PID
							
							PutCharacter2(SLAVEctr);//0xFE off =xFF ON
							PutCharacter2(0x73^SLAVEctr); //ENHANCED CHKSUM XORING PID + DATA
						  //INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
						}
	
						if ((txsw==1))
						{
						INTEnable(INT_SOURCE_UART_RX(UART2), INT_DISABLED);
							//  SLAVE RESPONSE
							U2STA=0x1d10;//set UT2BRK
							PutCharacter2(0x00);
							PutCharacter2(0x55);
							PutCharacter2(0x11);//request response 
							
							
							//STATUS OFF:0x70
							//STATUS ON:0x71
						INTEnable(INT_SOURCE_UART_RX(UART2), INT_ENABLED);
						}
						
					}		
	

		}







}


///////////////////////////////////////////////////////////////////
//
//	InitI2C
//
// 	Perform initialisation of the I2C module to operate as a slave
//
///////////////////////////////////////////////////////////////////
void InitI2C(void)
{
	unsigned char temp;
	
	// Enable the I2C module with clock stretching enabled
	OpenI2C1(I2C_ON | I2C_7BIT_ADD | I2C_STR_EN, BRG_VAL);
	
	// set the address of the slave module, address matching is with bits
	// 7:1 of the message compared with bits 6:0 of the ADD SFR so we
	// need to shift the desired address 1 bit. 
	I2C1ADD = SLAVE_ADDRESS; // >> 1;
	I2C1MSK = 0;
	
	// configure the interrupt priority for the I2C peripheral
	mI2C1SetIntPriority(I2C_INT_PRI_3 | I2C_INT_SLAVE);

	// clear pending interrupts and enable I2C interrupts
	mI2C1SClearIntFlag();
	EnableIntSI2C1;
}



///////////////////////////////////////////////////////////////////
//
// Slave I2C interrupt handler
// This handler is called when a qualifying I2C events occurs
// this means that as well as Slave events 
// Master and Bus Collision events will also trigger this handler.
//
///////////////////////////////////////////////////////////////////
void __ISR(_I2C_1_VECTOR, ipl3) _SlaveI2CHandler(void)
{
	//mLED_1_On();

	// dataVout = 111, dataIout=10, dataTemp=30, dataVin=128;
	// check for MASTER and Bus events and respond accordingly
	if (IFS0bits.I2C1MIF == 1) {
		mI2C1MClearIntFlag();
		return;		
	}
	if (IFS0bits.I2C1BIF == 1) {
		mI2C1BClearIntFlag();
		return;
	}
//	mLED_1_Off();
//	mLED_2_On();
	
	// handle the incoming message
	if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0)) {
		// R/W bit = 0 --> indicates data transfer is input to slave
		// D/A bit = 0 --> indicates last byte was address  
		
		// reset any state variables needed by a message sequence	
		// perform a dummy read of the address
		temp = SlaveReadI2C1();
		
	//	mLED_3_On();
	//	mLED_2_Off();
		// release the clock to restart I2C
		I2C1CONbits.SCLREL = 1; // release the clock

	} else if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 1)) {
		// R/W bit = 0 --> indicates data transfer is input to slave
		// D/A bit = 1 --> indicates last byte was data
		
	//	mLED_3_On();
	//	mLED_2_On();
		// writing data to our module, just store it in adcSample
		dataselect = SlaveReadI2C1();
		if (dataselect==0xf1)EnableIonizer();
		if (dataselect==0xf0)DisableIonizer();		
		// release the clock to restart I2C
		I2C1CONbits.SCLREL = 1; // release clock stretch bit

	} else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0)) {
		// R/W bit = 1 --> indicates data transfer is output from slave
		// D/A bit = 0 --> indicates last byte was address
	//	mLED_0_On();
	//	mLED_2_Off();
		// read of the slave device, read the address 
		temp = SlaveReadI2C1();
		dIndex = 0;
	if (timeout>150)
	{
	timeout=151;
		switch(dataselect)
		{

		case 0:
		SlaveWriteI2C1(0);
		break;
		case 1:
		SlaveWriteI2C1(0);
		break;
		case 2:
		SlaveWriteI2C1(0);
		break;
		case 3:
		SlaveWriteI2C1(0);
		break;
		case 4:
		SlaveWriteI2C1(0);
		break;
		case 5:
		SlaveWriteI2C1(0);
		break;

		case 6:
		SlaveWriteI2C1(0);
	scheduleCTR=10;
		break;

		default:
		SlaveWriteI2C1(0);
		break;


		}
	}	
	else
	{
		switch(dataselect)
		{

		case 0:
		SlaveWriteI2C1(dataStatus);
		break;
		case 1:

		SlaveWriteI2C1(dataVin);
		break;
		case 2:
		SlaveWriteI2C1(dataVoutH);
		break;
		case 3:
		SlaveWriteI2C1(dataVoutL);
		break;
		case 4:
		SlaveWriteI2C1(dataIoutH);
		break;
		case 5:
		SlaveWriteI2C1(dataIoutL);
		break;

		case 6:
		SlaveWriteI2C1(dataTemp);
		scheduleCTR=10;
		break;

		default:
		SlaveWriteI2C1(0);
		break;


		}
	}	



	} else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 1)) {
		// R/W bit = 1 --> indicates data transfer is output from slave
		// D/A bit = 1 --> indicates last byte was data

		
		// output the data until the MASTER terminates the
		// transfer with a NACK, continuing reads return 0
		if (dIndex == 0) {
			SlaveWriteI2C1(dataVout);
			dIndex++;
		} else
			SlaveWriteI2C1(0);
	}
	
	// finally clear the slave interrupt flag
	mI2C1SClearIntFlag();		
}




void EnableIonizer (void)
{

SLAVEctr=0xFF;

}
void DisableIonizer (void)
{

SLAVEctr=0xFE;

}








void PutCharacter1(const char character)
{
  while (!UARTTransmitterIsReady(UART1))
    ;

  UARTSendDataByte(UART1, character);

  while (!UARTTransmissionHasCompleted(UART1))
    ;
}


void PutCharacter2(const char character)
{
  while (!UARTTransmitterIsReady(UART2))
    ;

  UARTSendDataByte(UART2, character);

  while (!UARTTransmissionHasCompleted(UART2))
    ;
}

void WriteString1(const char *string)
{
  while (*string != '\0')
    {
      while (!UARTTransmitterIsReady(UART1))
        ;

      UARTSendDataByte(UART1, *string);

      string++;

      while (!UARTTransmissionHasCompleted(UART1))
        ;
    }
}


void WriteString2(const char *string)
{
  while (*string != '\0')
    {
      while (!UARTTransmitterIsReady(UART2))
        ;

      UARTSendDataByte(UART2, *string);

      string++;

      while (!UARTTransmissionHasCompleted(UART2))
        ;
    }
}

//
//void __ISR(_UART1_VECTOR, IPL3SOFT) IntUart1Handler(void)
//{
//  // Is this an RX interrupt?
//  if (INTGetFlag(INT_SOURCE_UART_RX(UART1)))
//    {
//      // Echo what we just received.
//      //PutCharacter(UARTGetDataByte(UART1));
//		LED1=!LED1;
//      // Clear the RX interrupt Flag
//      INTClearFlag(INT_SOURCE_UART_RX(UART1));
//
//      // Toggle LED to indicate UART activity
//      //mPORTAToggleBits(BIT_7);
//
//    }
//
//  // We don't care about TX interrupt
//  if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )
//    {
//      INTClearFlag(INT_SOURCE_UART_TX(UART1));
//    }
////INTClearFlag(INT_SOURCE_UART_TX(UART1));
//
//}

void __ISR(_UART1_VECTOR, IPL3SOFT) UART1InterruptVector(void)
{
	if (INTGetFlag(INT_U1TX) && INTGetEnable(INT_U1TX))	// transmit buffer empty?
	{
		if (*txPtr != '\0')
		{
		UARTSendDataByte(UART1,*txPtr++);
		INTClearFlag(INT_U1TX);	// buffer is ready, clear interrupt flag
		}
		else
		{
		// modemBuffer is empty, clear interrupt enable flag to stop TX
		INTEnable(INT_U1TX, INT_DISABLED);
		}
	}

	if (INTGetFlag(INT_U1RX) && INTGetEnable(INT_U1RX))	// something in the receive buffer?
	{
		while (UARTReceivedDataIsAvailable(UART1))
		{
		/*place to store rx here = */
		dataVout=UARTGetDataByte(UART1);	// store bytes in buffer
		LED1=!LED1;
		}
	INTClearFlag(INT_U1RX);	// buffer is empty, clear interrupt flag
	}
}


void __ISR(_UART2_VECTOR, IPL3SOFT) UART2InterruptVector(void)
{
	if (INTGetFlag(INT_U2TX) && INTGetEnable(INT_U2TX))	// transmit buffer empty?
	{
		if (*txPtr != '\0')
		{
		UARTSendDataByte(UART2,*txPtr++);
		INTClearFlag(INT_U2TX);	// buffer is ready, clear interrupt flag
		}
		else
		{
		// modemBuffer is empty, clear interrupt enable flag to stop TX


		INTEnable(INT_U2TX, INT_DISABLED);


		//clear uart2 receiver buffer: 
		U2STAbits.OERR = 0;
		while ((INTGetFlag(INT_U2RX) && INTGetEnable(INT_U2RX)));
		while (UARTReceivedDataIsAvailable(UART2))
		{
		data2=UARTGetDataByte(UART2);
		}
		INTClearFlag(INT_U2RX);

		}
	}

	if (INTGetFlag(INT_U2RX) && INTGetEnable(INT_U2RX))	// something in the receive buffer?
	{
		DisableIntSI2C1;
		while (UARTReceivedDataIsAvailable(UART2))
		{
		/*place to store rx here = */
		data2=UARTGetDataByte(UART2);	// store bytes in buffer
	//	LED1=!LED1;
		LIN2RXindex++;
		if((last2==0x00)&&(data2==0x55))
		{

	
		LIN2RXindex=0;
								switch (LIN2RXBUF[1])
								{
									case 0x11:
									dataStatus=LIN2RXBUF[2];
									LED_LIN1=!LED_LIN1;
									break;
									
									
									default:
									break;
								}

							switch(scheduleCTR+2)
							{
							case 0:

							break;
						
							case 9://Supply Voltage 8bit 0.1V/bit
								switch (LIN2RXBUF[2])
								{
									case 0x68:
									dataVin=LIN2RXBUF[5];

									break;
									
									
									default:
									break;
								}
							break;
						
						
							case 7://Output Voltage 16bit 1V/bit
								switch (LIN2RXBUF[2])
								{
									case 0x68:
									dataVout=LIN2RXBUF[5]*256+LIN2RXBUF[6];
									dataVoutH=LIN2RXBUF[5];
									dataVoutL=LIN2RXBUF[6];
									break;
									
									
									default:
									break;
								}
							break;
						
							case 5://Output Current 16bit 0.5uA/bit
								switch (LIN2RXBUF[2])
								{
									case 0x68:
									dataIout=LIN2RXBUF[5]*256+LIN2RXBUF[6];
									dataIoutH=LIN2RXBUF[5];
									dataIoutL=LIN2RXBUF[6];
									break;
									
									
									default:
									break;
								}
							break;
						
							case 3://Output temp, 8bit 1C/bit 50C offset
								switch (LIN2RXBUF[2])
								{
									case 0x68:
									dataTemp=LIN2RXBUF[5];
									LIN1_timeout=0;
									timeout=0;
									LED_LIN2=!LED_LIN2;
									break;
									
									
									default:
									break;
								}
							break;

							case 1://Output temp, 8bit 1C/bit 50C offset
								switch (LIN2RXBUF[1])
								{
									case 0x11:
									dataStatus=LIN2RXBUF[2];
									break;
									
									
									default:
									break;
								}
							break;
							default:
							break;
							}



		}

		LIN2RXBUF[LIN2RXindex]=data2;

		last2=data2;
		}


		if(LIN2RXindex==0)
		{

		}	




	EnableIntSI2C1;
	INTClearFlag(INT_U2RX);	// buffer is empty, clear interrupt flag
	}
}



void ADCread (void)
{

		while(!AD1CON1&0x0001);

		while(!IFS1&0x0002);
		IFS1CLR=0X0002;
		AD1CON1SET=0X0004;
		AD1CON1CLR=0x0002;

}




































/** EOF main.c *************************************************/
#endif



























