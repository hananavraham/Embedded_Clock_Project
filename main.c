/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM™ FS USB Demo Board, 
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

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
********************************************************************/


//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"

#include <math.h>

#include "OledGraphics.h"


//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
     #pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
   //PLL Prescaler Selection bits:
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
   //Stack Overflow/Underflow Reset Enable bit:
     #pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
   //Extended Instruction Set Enable bit:
     #pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
   //CPU System Clock Postscaler:
     #pragma config CPUDIV = OSC1        //No CPU system clock divide
   //Code Protection bit:
     #pragma config CP0 = OFF            //Program memory is not code-protected
   //Oscillator Selection bits:
     #pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
   //Secondary Clock Source T1OSCEN Enforcement:
     #pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
   //Low-Power Timer1 Oscillator Enable bit:
     #pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
   //Fail-Safe Clock Monitor Enable bit:
     #pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
   //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
     #pragma config IESO = OFF           //Two-Speed Start-up disabled
   //Watchdog Timer Postscaler Select bits:
     #pragma config WDTPS = 32768        //1:32768
   //DSWDT Reference Clock Select bit:
     #pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
   //RTCC Reference Clock Select bit:
     #pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
   //Deep Sleep BOR Enable bit:
     #pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
   //Deep Sleep Watchdog Timer Enable bit:
     #pragma config DSWDTEN = OFF        //Disabled
   //Deep Sleep Watchdog Timer Postscale Select bits:
     #pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
   //IOLOCK One-Way Set Enable bit:
     #pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
   //MSSP address mask:
     #pragma config MSSP7B_EN = MSK7     //7 Bit address masking
   //Write Protect Program Flash Pages:
     #pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
   //Write Protection End Page (valid when WPDIS = 0):
     #pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
   //Write/Erase Protect Last Page In User Flash bit:
     #pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
   //Write Protect Disable bit:
     #pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored
  
#else
    #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();
void addSecond();
void addMinute();
void addHour();
void addDay();
void changeClockFormat();
void printAnalogTime(int,int,int);

BOOL CheckButtonPressed(void);

int Time[6]  = {0,0,0,0,0,0};		//{hours_high,hours_low,minutes_high,minutes_low,seconds_high,seconds_low}
int Date[2]  = {1,1};	// {day,month}
int radius = 32;
BOOL IsDigitalMode = TRUE;
BOOL EnableAlarm = FALSE;
BOOL Is12Hours = FALSE;
BOOL IsAM = FALSE;
BOOL IsClockSet = FALSE;
BOOL UpdateStatus = FALSE;
BOOL MenuFlag = FALSE;
//int x[60] = {95,93,90,84,77,69,61,53,45,40,36,35,35,39,44,51,59,68,76,83,89,93,95,94,91,85,79,71,62,54,47,41,36,35,35,38,43,50,58,66,74,82,88,92,95,94,91,87,80,72,64,55,48,42,37,35,35,37,42,48,56};
//int y[60] = {31,22,14,8,3,1,1,3,7,14,21,30,38,46,52,57,60,60,58,54,48,40,32,24,16,9,4,1,1,2,6,12,20,28,37,45,51,57,60,60,59,55,49,42,34,25,17,10,5,2,1,2,6,11,18,27,35,43,50,56,59};
int x0 = 65;
int y0 = 31;

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
  //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
  //the reset, high priority interrupt, and low priority interrupt
  //vectors.  However, the current Microchip USB bootloader 
  //examples are intended to occupy addresses 0x00-0x7FF or
  //0x00-0xFFF depending on which bootloader is used.  Therefore,
  //the bootloader code remaps these vectors to new locations
  //as indicated below.  This remapping is only necessary if you
  //wish to program the hex file generated from this project with
  //the USB bootloader.  If no bootloader is used, edit the
  //usb_config.h file and comment out the following defines:
  //#define PROGRAMMABLE_WITH_SD_BOOTLOADER
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
  #else 
    #define REMAPPED_RESET_VECTOR_ADDRESS     0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
  #endif
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  extern void _startup (void);        // See c018i.c in your C18 compiler dir
  #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
  void _reset (void)
  {
      _asm goto _startup _endasm
  }
  #endif
  #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
  void Remapped_High_ISR (void)
  {
       _asm goto YourHighPriorityISRCode _endasm
  }
  #pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
  void Remapped_Low_ISR (void)
  {
       _asm goto YourLowPriorityISRCode _endasm
  }
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  //Note: If this project is built while one of the bootloaders has
  //been defined, but then the output hex file is not programmed with
  //the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
  //As a result, if an actual interrupt was enabled and occured, the PC would jump
  //to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
  //executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
  //(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
  //would effective reset the application.
  
  //To fix this situation, we should always deliberately place a 
  //"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
  //"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
  //hex file of this project is programmed with the bootloader, these sections do not
  //get bootloaded (as they overlap the bootloader space).  If the output hex file is not
  //programmed using the bootloader, then the below goto instructions do get programmed,
  //and the hex file still works like normal.  The below section is only required to fix this
  //scenario.
  #pragma code HIGH_INTERRUPT_VECTOR = 0x08
  void High_ISR (void)
  {
       _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #pragma code LOW_INTERRUPT_VECTOR = 0x18
  void Low_ISR (void)
  {
       _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

  #pragma code
  
//	========================	Application Interrupt Service Routines	========================
  //These are your actual interrupt handling routines.
  #pragma interrupt YourHighPriorityISRCode
  void YourHighPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.

	//if (INTCONbits.T0IF)
	//{
		addSecond();
	//}
	INTCONbits.TMR0IF = 0;	

  } //This return will be a "retfie fast", since this is in a #pragma interrupt section 
  #pragma interruptlow YourLowPriorityISRCode
  void YourLowPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif




//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

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
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/

#define FILE_FETCH		0
#define SCREEN_UPDATE	1
#define USER_INPUT		2



void UserInit(void)
{

  /* Initialize the mTouch library */
  mTouchInit();

  /* Call the mTouch callibration function */
  mTouchCalibrate();

  /* Initialize the accelerometer */
  InitBma150(); 

  /* Initialize the oLED Display */
   ResetDevice();  
   FillDisplay(0x00);
   //oledPutROMString((ROM_STRING)" PIC18F Starter Kit  ",0,0);
}//end UserInit


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
	// Soft Start the APP_VDD
	while(!AppPowerReady())
		;

    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
       
    UserInit();

}//end InitializeSystem


//	========================	Application Code	========================

BOOL CheckButtonPressed(void)
{
	static char buffer[10];
    static char buttonPressed = FALSE;
    static unsigned long buttonPressCounter = 0;

    if(PORTBbits.RB0 == 0)
    {
        if(buttonPressCounter++ > 10)
        {
            buttonPressCounter = 0;
            buttonPressed = TRUE;
        }
    }
    else
    {
        if(buttonPressed == TRUE)
        {
            if(buttonPressCounter == 0)
            {
                buttonPressed = FALSE;
                return TRUE;
            }
            else
            {
                buttonPressCounter--;
            }
        }
    }
    return FALSE;
}



/* this method check the potentiometer status */
int checkPotNavigate()
{
	int pot = 0;

	ADCON0 = 0x13;
	pot = ((int)ADRESH << 8) + ADRESL;

	if (pot >= 0 && pot < 204)
		return 2;
	else if(pot >= 204 && pot < 404)
		return 3;
	else if(pot >= 404 && pot < 606)
		return 4;
	else if(pot >= 606 && pot < 808)
		return 5;
	else
		return 6;
}

/* this method print '_' under the current field inside setDate and setTime menu */
void printCurrentField(int lineSelect)
{
	switch (lineSelect)
	{
		case 0:
			oledWriteChar1x(0X5F,0xB4,30);
			oledWriteChar1x(0X5F,0xB4,33);
			oledWriteChar1x(0X5F,0xB4,36);
			break;

		case 1:
			oledWriteChar1x(0X5F,0xB4,47);
			oledWriteChar1x(0X5F,0xB4,50);
			oledWriteChar1x(0X5F,0xB4,53);
			break;

		case 2:
			oledWriteChar1x(0X5F,0xB4,65);
			oledWriteChar1x(0X5F,0xB4,68);
			oledWriteChar1x(0X5F,0xB4,71);
			break;
	}
}

/* ********************* Sub Menus **********************************************/

/* In this SubMenu Navigate with Potentiometer */
void DisplayModeMenu()
{
	int Right_button, Left_button , button1;
	int lineSelect = 1;
	BOOL status = TRUE;

	while(1)
	{
		if(status)
		{
			FillDisplay(0x00);
			oledPutROMString("Display Mode Menu", 0, 3*6);
		
			oledPutROMString("Digital Clock", 3, 3* 6);
			oledPutROMString("Analog Clock", 5, 3* 6);
			
			status = FALSE;
		}
	
		if(lineSelect == 1)
			FillInverseDisplay(0xFF, 3,0,0,4);
		else
			FillInverseDisplay(0xFF, 5,0,0,4);	

	
		if(CheckButtonPressed())
		{
			if(lineSelect == 1)
				IsDigitalMode = TRUE;
			else
				IsDigitalMode = FALSE;
			return;		
		}

		mTouchCalibrate();
		/* check if Up Button pressed to back the Main Menu */
		button1 = mTouchReadButton(1);
		if (button1 < 900)
		{
			/* wait for the slider to be released */
			while (button1 < 900)
			{
				button1 = mTouchReadButton(1);
			}

			return;
		}

		
		/* Read the R button - RA0 */
		Right_button = mTouchReadButton(0);
		/* Read the L button - RA3 */
		Left_button = mTouchReadButton(3);
	
		/* Check if the R button was pressed */
		if (Right_button < 900)
		{
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}

			if(lineSelect < 2)
			{
				lineSelect++;	
				status = TRUE;
			}	
		}
	
		if (Left_button < 900)
		{
			/* wait for the button to be released */
			while (Left_button < 900)
			{
				Left_button = mTouchReadButton(3);
			}
			if(lineSelect > 1)
			{
				lineSelect--;
				status = TRUE;
			}	
		}
	}
}


/* In this SubMenu Navigate with Tilt X */
void HoursIntervalMenu()
{
	int Right_button, Left_button , button1;
	int lineSelect = 1;
	BOOL status = TRUE;

	while(1)
	{
		if(status)
		{
			FillDisplay(0x00);
			oledPutROMString("Hours Interval", 0, 3*6);
		
			oledPutROMString("12H", 3, 3* 6);
			oledPutROMString("24H", 5, 3* 6);
			
			status = FALSE;
		}
	
		if(lineSelect == 1)
			FillInverseDisplay(0xFF, 3,0,0,4);
		else
			FillInverseDisplay(0xFF, 5,0,0,4);	

	
		if(CheckButtonPressed())
		{
			if(lineSelect == 1)
				Is12Hours = TRUE;
			else
				Is12Hours = FALSE;
			changeClockFormat();		// changing the clock format to the current choice
			return;		
		}

		mTouchCalibrate();
		/* check if Up Button pressed to back the Main Menu */
		button1 = mTouchReadButton(1);
		if (button1 < 900)
		{
			/* wait for the slider to be released */
			while (button1 < 900)
			{
				button1 = mTouchReadButton(1);
			}

			return;
		}

		
		/* Read the R button - RA0 */
		Right_button = mTouchReadButton(0);
		/* Read the L button - RA3 */
		Left_button = mTouchReadButton(3);
	
		/* Check if the R button was pressed */
		if (Right_button < 900)
		{
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}

			if(lineSelect < 2)
			{
				lineSelect++;	
				status = TRUE;
			}	
		}
	
		if (Left_button < 900)
		{
			/* wait for the button to be released */
			while (Left_button < 900)
			{
				Left_button = mTouchReadButton(3);
			}
			if(lineSelect > 1)
			{
				lineSelect--;
				status = TRUE;
			}	
		}
	}
}


/* In this SubMenu Navigate with R , L buttons */
void SetTimeMenu()
{
	int Right_button, Left_button, button1, button2;
	char buffer[10];
	int lineSelect = 0;
	BOOL status = TRUE;
	int tempTime[6] = {0,0,0,0,0,0};
	tempTime[0] = Time[0];
	tempTime[1] = Time[1];
	tempTime[2] = Time[2];
	tempTime[3] = Time[3];
	tempTime[4] = Time[4];
	tempTime[5] = Time[5];
	while(1)
	{	
		if(status)
		{
			FillDisplay(0x00);
					
			oledPutROMString("Set Time Menu", 0, 5*5);
			sprintf(buffer,"%d%d:%d%d:%d%d",tempTime[0],tempTime[1],tempTime[2],tempTime[3],tempTime[4],tempTime[5]);
			oledPutString(buffer, 3, 5*6);

			// if 12H need to print also AM\ PM
			if(Is12Hours)
			{
				if(IsAM)
					oledPutROMString("AM", 3, 14*6);
				else
					oledPutROMString("PM", 3, 14*6);
			}
		}

		printCurrentField(lineSelect);

	
		if(CheckButtonPressed())
		{
			Time[0] = tempTime[0];
			Time[1] = tempTime[1];
			Time[2] = tempTime[2];
			Time[3] = tempTime[3];
			Time[4] = tempTime[4];
			Time[5] = tempTime[5];
			IsClockSet = TRUE;
			return;		
		}		


		mTouchCalibrate();
		/* Read the R button - RA0 */
		Right_button = mTouchReadButton(0);
		/* Read the L button - RA3 */
		Left_button = mTouchReadButton(3);
	
		/* Check if the R button was pressed */
		if (Right_button < 900)
		{
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}

			if(lineSelect < 2)
			{
				lineSelect++;	
				status = TRUE;
			}	
		}
	
		if (Left_button < 900)
		{
			/* wait for the button to be released */
			while (Left_button < 900)
			{
				Left_button = mTouchReadButton(3);
			}
			if(lineSelect > 0)
			{
				lineSelect--;
				status = TRUE;
			}	
		}

		mTouchCalibrate();
		/* Scroll bar - RA1/RA2*/
		button1 = mTouchReadButton(1);
		button2 = mTouchReadButton(2);

		if ((button1 < 800) || (button2 < 800))
		{
			if (button1 < button2)		// Up Button Pressed
			{
				switch(lineSelect)
				{
					case 0:  // hours
						if (!Is12Hours)
						{
							if(tempTime[0] == 2 && tempTime[1] == 3)
							{
								tempTime[0] = 0;
								tempTime[1] = 0;
								IsAM = TRUE;
							}
							else if(tempTime[0] != 2 && tempTime[1] == 9)
							{
								tempTime[0]++;
								tempTime[1] = 0;
							}
							else if((tempTime[0] != 2 && tempTime[1] != 9) || (tempTime[0] == 2 && tempTime[1] != 3))
							{
								tempTime[1]++;	
								if(tempTime[0] > 0 && tempTime[1] > 2)
									IsAM = FALSE;
							}

							if(tempTime[0] == 0 && tempTime[0] == 0)
								IsAM = TRUE;		
						}
						else
						{
							if(tempTime[0] == 1 && tempTime[1] == 2)
							{
								tempTime[0] = 0;
								tempTime[1] = 1;
								if(IsAM)
									IsAM = FALSE;	
								else
									IsAM = TRUE;
							}
							else if(tempTime[0] == 1 && tempTime[1] < 2)
							{
								tempTime[1]++;
							}
							else if(tempTime[0] != 1 && tempTime[1] == 9)
							{
								tempTime[0]++;
								tempTime[1] = 0;						
							}
							else
								tempTime[1]++;							
						}
						status = TRUE;
						break;
					case 1:
						if(tempTime[3] < 9)
							tempTime[3]++;
						else if(tempTime[2] == 5 && tempTime[3] == 9)
						{
							tempTime[2] = 0;
							tempTime[3] = 0;
						}
						else if(tempTime[2] != 5 && tempTime[3] == 9)
						{
							tempTime[2]++;
							tempTime[3] = 0;
						}
						status = TRUE;
						break;
					case 2:
						if(tempTime[5] < 9)
							tempTime[5]++;
						else if(tempTime[4] == 5 && tempTime[5] == 9)
						{
							tempTime[4] = 0;
							tempTime[5] = 0;
						}
						else if(tempTime[4] != 5 && tempTime[5] == 9)
						{
							tempTime[4]++;
							tempTime[5] = 0;
						}
						status = TRUE;
						break;
				}
			}		
			else		// Down Button Pressed
			{
				switch(lineSelect)
				{
					case 0:  // hours
						if (!Is12Hours)
						{
							if(tempTime[1] > 0)
								tempTime[1]--;
							else if(tempTime[0] != 0 && tempTime[1] == 0)
							{
								tempTime[0]--;
								tempTime[1] = 9;
							}
							
							else
							{
								tempTime[0] = 2;
								tempTime[1] = 3;
							}
							if(tempTime[0] == 0 && tempTime[0] == 0)
								IsAM = TRUE;						
						}
						else
						{
							if(tempTime[0] != 1 && tempTime[1] != 0)
							{
								tempTime[1]--;						
							}
							else if(tempTime[0] == 1 && tempTime[1] == 0)
							{	
								tempTime[0] = 0;
								tempTime[1] = 9;
							}	
							else if(tempTime[0] == 0 && tempTime[1] == 1)
							{
								tempTime[0] = 1;
								tempTime[1] = 2;
								if(IsAM)
									IsAM = FALSE;	
								else
									IsAM = TRUE;
							}
							else
							{
								tempTime[1]--;
							}																			
						}
						status = TRUE;
						break;
					case 1:		// Minutes
						if(tempTime[3] > 0)
							tempTime[3]--;
						else if(tempTime[2] != 0 && tempTime[3] == 0)
						{
							tempTime[2]--;
							tempTime[3] = 9;
						}
//						else if(tempTime[2] == 0 && tempTime[3] != 0)
//						{
//							tempTime[3]--;
//						}
						else
						{	
							tempTime[2] = 5;
							tempTime[3] = 9;
						}
						status = TRUE;
						break;
					case 2:		// Seconds
						if(tempTime[5] > 0)
							tempTime[5]--;
						else if(tempTime[4] != 0 && tempTime[5] == 0)
						{
							tempTime[4]--;
							tempTime[5] = 9;
						}
//						else if(tempTime[4] == 0 && tempTime[5] != 0)
//						{
//							tempTime[5]--;
//						}
						else
						{	
							tempTime[4] = 5;
							tempTime[5] = 9;
						}
						status = TRUE;
						break;
				}
				
			}
			/* wait for the slider to be released */
			while ((button1 < 800))
			{
				button1 = mTouchReadButton(1);
				button2 = mTouchReadButton(2);
			}
		}


	}
}


void SetDateMenu()
{
	int Right_button, Left_button, button1, button2;
	int lineSelect = 0;
	char buffer[10];
	BOOL status = TRUE;
	int tempDate[2] = {0,0};
	tempDate[0] = Date[0];
	tempDate[1] = Date[1];
	while(1)
	{	
		if(status)
		{
			FillDisplay(0x00);
					
			oledPutROMString("Set Date Menu", 0, 5*5);
			sprintf(buffer,"%02d/%02d",tempDate[0],tempDate[1]);
			oledPutString(buffer, 3, 5*6);
		}

		//lineSelect =  checkPotNavigate();
		printCurrentField(lineSelect);

	
		if(CheckButtonPressed())
		{
			Date[0] = tempDate[0];
			Date[1] = tempDate[1];
			return;		
		}		


		mTouchCalibrate();
		/* Read the R button - RA0 */
		Right_button = mTouchReadButton(0);
		/* Read the L button - RA3 */
		Left_button = mTouchReadButton(3);
	
		/* Check if the R button was pressed */
		if (Right_button < 900)
		{
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}

			if(lineSelect < 1)
			{
				lineSelect++;	
				status = TRUE;
			}	
		}
	
		if (Left_button < 900)
		{
			/* wait for the button to be released */
			while (Left_button < 900)
			{
				Left_button = mTouchReadButton(3);
			}
			if(lineSelect > 0)
			{
				lineSelect--;
				status = TRUE;
			}	
		}

		/* Scroll bar - RA1/RA2*/
		button1 = mTouchReadButton(1);
		button2 = mTouchReadButton(2);

		if ((button1 < 800) || (button2 < 800))
		{
			if (button1 < button2)
			{
				switch(lineSelect)
				{
					case 0:  // Days
						if(tempDate[lineSelect] < 31)
							tempDate[lineSelect]++;
						else
						{	
							tempDate[lineSelect] = 1;
						}							
						
						status = TRUE;
						break;
					case 1:	// Months
						if(tempDate[lineSelect] < 12)
							tempDate[lineSelect]++;
						else
							tempDate[lineSelect] = 1;
						status = TRUE;
						break;
				}
			}		
			else
			{
				switch(lineSelect)
				{
					case 0:  // Days
						if(tempDate[lineSelect] > 1)
							tempDate[lineSelect]--;
						else
						{	
							tempDate[lineSelect] = 31;
						}							
						status = TRUE;
						break;
					case 1:	 // Months
						if(tempDate[lineSelect] > 1)
							tempDate[lineSelect]--;
						else
							tempDate[lineSelect] = 12;
						status = TRUE;
						break;
				}
				
			}
			/* wait for the slider to be released */
			while ((button1 < 800))
			{
				button1 = mTouchReadButton(1);
				button2 = mTouchReadButton(2);
			}
		}

	}
}

void AlarmMenu()
{
	int Right_button, Left_button , button1;
	int lineSelect = 1;
	BOOL status = TRUE;

	while(1)
	{
		if(status)
		{
			FillDisplay(0x00);
			oledPutROMString("Alarm Menu", 0, 5*6);
		
			oledPutROMString("ON", 3, 3* 6);
			oledPutROMString("OFF", 5, 3* 6);
			
			status = FALSE;
		}
	
		if(lineSelect == 1)
			FillInverseDisplay(0xFF, 3,0,0,4);
		else
			FillInverseDisplay(0xFF, 5,0,0,4);	

	
		if(CheckButtonPressed())
		{
			if(lineSelect == 1)
				EnableAlarm = TRUE;
			else
				EnableAlarm = FALSE;
			return;		
		}

		/* check if Up Button pressed to back the Main Menu */
		button1 = mTouchReadButton(1);
		if (button1 < 900)
		{
			/* wait for the slider to be released */
			while (button1 < 900)
			{
				button1 = mTouchReadButton(1);
			}

			return;
		}

		mTouchCalibrate();
		/* Read the R button - RA0 */
		Right_button = mTouchReadButton(0);
		/* Read the L button - RA3 */
		Left_button = mTouchReadButton(3);
	
		/* Check if the R button was pressed */
		if (Right_button < 900)
		{
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}

			if(lineSelect < 2)
			{
				lineSelect++;	
				status = TRUE;
			}	
		}
	
		if (Left_button < 900)
		{
			/* wait for the button to be released */
			while (Left_button < 900)
			{
				Left_button = mTouchReadButton(3);
			}
			if(lineSelect > 1)
			{
				lineSelect--;
				status = TRUE;
			}	
		}
	}
}

/* **********************************************************************/

void EnterSubMenu(int lineSelect)
{
	switch (lineSelect)
	{
		case 2:
			DisplayModeMenu();
			break;

		case 3:
			HoursIntervalMenu();
			break;

		case 4:
			SetTimeMenu();
			break;

		case 5:
			SetDateMenu();
			break;
		
		case 6:
			AlarmMenu();
			break;

		default:
			break;
	}
	FillDisplay(0x00);
}


/* priting small clock inside menu */
void PrintSmallClock()
{
	char buffer[10];
	INTCONbits.T0IE = 0 ;			//Timer0 Overflow Interrupt Disable
	sprintf(buffer,"%d%d:%d%d:%d%d",Time[0],Time[1],Time[2],Time[3],Time[4],Time[5]);
	oledPutString(buffer, 0, 10*6);
	if(Is12Hours)
	{
		if(IsAM)
		{
			oledPutROMString("AM", 0, 19*6);;
		}
		else
		{
			oledPutROMString("PM", 0, 19*6);
		}
	}
	

	INTCONbits.T0IE = 1 ;			//Timer0 Overflow Interrupt Enabled
}

/* method to print all 60 clock points */
void drawClockPoints()
{
	int i;
	BYTE _x0,_y0;
	//BYTE x[60] = {95,93,90,84,77,69,61,53,45,40,36,35,35,39,44,51,59,68,76,83,89,93,95,94,91,85,79,71,62,54,47,41,36,35,35,38,43,50,58,66,74,82,88,92,95,94,91,87,80,72,64,55,48,42,37,35,35,37,42,48,56};
	//BYTE y[60] = {31,22,14,8,3,1,1,3,7,14,21,30,38,46,52,57,60,60,58,54,48,40,32,24,16,9,4,1,1,2,6,12,20,28,37,45,51,57,60,60,59,55,49,42,34,25,17,10,5,2,1,2,6,11,18,27,35,43,50,56,59};
	for (i =0 ; i <60; i++)
	{
		_x0 = x0+radius*(cos(i*6));
		_y0 = y0+radius*(sin(i*6));
		//drawLine(x,y,x+0.2*(30*(int)cos(i*6)),y+0.2*(30*(int)sin(i*6)),thin);
		drawLine(_x0,_y0,_x0,_y0,thin);
	}	
}

/* getting value and type[sec,min,hour] */
void printLine(int val, int type)
{
	int tempX,tempY;
	switch (type)
	{
		case 0:
			tempX = x0+radius*(sin(val*30));
			tempY = y0+radius*(cos(val*30));
			drawLine( x0, y0, x0+(tempX-x0)/2, y0+(tempY-y0)/2, fat ) ;		
			break;
		case 1:
			tempX = x0+radius*(sin(val*6));
			tempY = y0+radius*(cos(val*6));
			drawLine( x0, y0, tempX-(tempX-x0)/5, tempY-(tempY-y0)/5, thick ) ;
			break;
		case 2:
			tempX = x0+radius*(sin(val*6));
			tempY = y0+radius*(cos(val*6));
			drawLine( x0, y0, tempX-(tempX-x0)/5, tempY-(tempY-y0)/5, thin ) ;
			break;
	}

}


/************** Add Time Methods ***************************/

/* method invoke from interrupt for adding second */ 
void addSecond()
{
	int s;
	s = (Time[4] * 10) + Time[5];

	if(Time[5] != 9)
		Time[5]++;
	else if(Time[5] == 9 && Time[4] != 5)
	{
		Time[4]++;
		Time[5] = 0;
	}
	else if(Time[4] == 5 && Time[5] == 9)
	{
		Time[4] = 0;
		Time[5] = 0;
		addMinute();
	}
	
	if(IsDigitalMode)
		UpdateStatus = TRUE;
	else if(!MenuFlag)
	{
		printLine(s,2);

		s = (Time[4] * 10) + Time[5];
		printLine(s,2);	
	}
}

void addMinute()
{
	int m;
	m = (Time[2] * 10) + Time[3];
	
	if(Time[3] != 9)
		Time[3]++;
	else if(Time[3] == 9 && Time[2] != 5)
	{
		Time[2]++;
		Time[3] = 0;
	}
	else if(Time[2] == 5 && Time[3] == 9)
	{
		Time[2] = 0;
		Time[3] = 0;
		addHour();
	}
	if(!IsDigitalMode && !MenuFlag)
	{
		printLine(m,1);
		m = (Time[2] * 10) + Time[3];
		printLine(m,1);
	}
}

void addHour()
{
	int h;
	h = (Time[0] * 10) + Time[1];
	if(Is12Hours)		// if we working in 12H mode
	{
		if(Time[0] != 1 && Time[1] != 9)
			Time[1]++;
		else if(Time[0] != 1 && Time[1] == 9)
		{
			Time[0]++;
			Time[1] = 0;
		}
		else if(Time[0] == 1 && Time[1] != 2)
			Time[1]++;
		else
		{
			Time[0] = 0;
			Time[1] = 1;
			if(IsAM)
				IsAM = FALSE;	
			else
			{
				IsAM = TRUE;
				addDay();
			}	
		}
	}
	else
	{
		if((Time[0] != 2 && Time[1] != 9) || (Time[0] == 2 && Time[1] != 3))
			Time[1]++;
		else if(Time[0] != 2 && Time[1] == 9)
		{
			Time[0]++;
			Time[1] = 0;
		}
		else
		{			
			Time[0] = 0;
			Time[1] = 0;	
			addDay();
		}
	}
	if(!IsDigitalMode && !MenuFlag)
	{	
		printLine(h,0);
		
		h = (Time[0] * 10) + Time[1];
		printLine(h,0);
	}
}

void addDay()
{
	if(Date[0] != 31)
		Date[0]++;
	else
	{
		Date[0] = 1;
		if(Date[1] != 12)		// end of year
			Date[1]++;
		else
			Date[1] = 1;
	}
	UpdateStatus = TRUE;	
}

void changeClockFormat()
{
	if(Is12Hours)
	{
		if(!IsAM)
		{
			if(Time[0] == 2 && Time[1] >= 2)
			{
				Time[1] -= 2;
				Time[0] = 1;
			}
			else if(Time[0] == 2 && Time[1] == 1)
			{
				Time[0] = 0;
				Time[1] = 9;
			}
			else if(Time[0] == 2 && Time[1] == 0)
			{
				Time[0] = 0;
				Time[0] = 8;
			}
			else if(Time[0] == 1)
			{
				Time[0] = 0;
				Time[1] -= 2;
			}
		}	
	}

	else
	{
		if(!IsAM)
		{
			Time[0] = 1;
			Time[1] += 2;
		}

	}


}
/*******************************************************************/


/************** Printing Methods ****************************/
/* printing the current Date */
void PrintDate()
{
	char buffer[10];
	sprintf(buffer,"%02d/%02d",Date[0],Date[1]);
	oledPutString(buffer, 7, 0*6);
}

void PrintAlarm()
{
	if(EnableAlarm)
		oledWriteChar1x(0x41,0 , 15*6);	
}

/* printing the Digital Clock */
void printDigitalClock()
{
	int i,j;
	BYTE temp;
	for(i = 0; i < 6 ; ++i)
	{
		temp = Time[i];
		switch (Time[i])
		{
			case 0:		//{ 0x3e, 0x51, 0x49, 0x45, 0x3e }
				// 3 => 0011 => 0000-0000-1111-1111, e => 1110 => 1111-1111-1111-0000
				oledWriteChar1x(0x00,5,3*6);
				oledWriteChar1x(0x00,5,4*6);
				oledWriteChar1x(0x00,5,5*6);
				oledWriteChar1x(0xff,4,3*6);
				oledWriteChar1x(0xff,4,4*6);
				oledWriteChar1x(0xff,4,5*6);
				oledWriteChar1x(0xff,3,3*6);
				oledWriteChar1x(0xff,3,4*6);
				oledWriteChar1x(0xff,3,5*6);
				oledWriteChar1x(0xf0,2,3*6);
				oledWriteChar1x(0xf0,2,4*6);
				oledWriteChar1x(0xf0,2,5*6); 
				break;
			case 1:		//{ 0x00, 0x42, 0x7f, 0x40, 0x00 }
			
			case 2:		//{ 0x42, 0x61, 0x51, 0x49, 0x46 }
		
			case 3:		//{ 0x21, 0x41, 0x45, 0x4b, 0x31 }

			case 4:		//{ 0x18, 0x14, 0x12, 0x7f, 0x10 }
	
			case 5:		//{ 0x27, 0x45, 0x45, 0x45, 0x39 }

			case 6:		//{ 0x3c, 0x4a, 0x49, 0x49, 0x30 }

			case 7:		//{ 0x01, 0x71, 0x09, 0x05, 0x03 }

			case 8:		//{ 0x36, 0x49, 0x49, 0x49, 0x36 }

			case 9:		//{ 0x06, 0x49, 0x49, 0x29, 0x1e }
		
			default:
				break;
		}
	}
}


/* printing the Setup Menu */
void PrintMenu()
{
	int temp,button1;
	int Right_button = 0;
	int Left_button = 0;
	int lineSelect = 2;
	MenuFlag = TRUE;
	while(1)
	{
		ADCON0 = 0x13;
		temp = lineSelect;

		/********* Dual Thread *******/
		INTCONbits.T0IE = 0 ;			//Disable Timer Interrupts
		PrintSmallClock();
		INTCONbits.T0IE = 1 ;			//Enable Timer Interrupts
		/*****************************/
		
		oledPutROMString("Display Mode", 2, 0);
		oledPutROMString("12H/24H Interval", 3, 0);
		oledPutROMString("Set Time", 4, 0);
		oledPutROMString("Set Date", 5, 0);
		oledPutROMString("Alarm", 6, 0);
		lineSelect = checkPotNavigate();	
	
		PrintAlarm();
		PrintDate();
	
		FillInverseDisplay(0xFF, lineSelect,0,0,100);
		
		if(CheckButtonPressed())
		{
			EnterSubMenu(lineSelect);	
		}
	
		if(temp != lineSelect)
		{
			FillDisplay(0x00);
		}

		mTouchCalibrate();
		/* check if Up Button pressed to back the Main Menu */
		button1 = mTouchReadButton(1);
		if (button1 < 900)
		{
			/* wait for the slider to be released */
			while (button1 < 900)
			{
				button1 = mTouchReadButton(1);
			}
			FillDisplay(0x00);
			return;
		}		
	}
}	
/********************************************************************/


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void main(void)
{
	
	BYTE x = 2;
	BOOL status = FALSE;
	InitializeSystem();

//	T0CON = 0B00010100 ;		// init (16 bits, prescaler- 1:32)
//	T0CON = T0CON | 0x80;		
//	RCONbits.IPEN = 1 ;			
//	INTCON2bits.TMR0IP = 1 ;
//	INTCON = 0B11100000 ;
//

	// Initialize Timer 0
	T0CON = 0x05;
	// Initialize Timer Interrupt
	RCONbits.IPEN = 1 ;			//Prio Enable
	INTCON2bits.TMR0IP = 1 ;	//Use Hi-Prio
	INTCON = 0xE0 ;				//Enable Timer Interrupt

	T0CON |= 0x80 ;				//Start the Timer

    while(1)							//Main is Usualy an Endless Loop
    {		
		while(x)	
		{
			x = x&ADCON0;	
		}
			
		if(IsClockSet)
		{
			if(IsDigitalMode)		// Digital Clock Mode
				printDigitalClock();

			else
			{
				if(!status)
				{
					drawClockPoints();	// Analog Clock Mode
					printLine(Time[4] * 10 + Time[5],2);
					printLine(Time[2] * 10 + Time[3],1);
					printLine(Time[0] * 10 + Time[1],0);
					status = TRUE;
				}
			}
			//PrintAlarm();
			//PrintDate();
		}
		
		else
			PrintMenu();

		MenuFlag = FALSE;	

		if(CheckButtonPressed())
		{
			PrintMenu();
			status = FALSE;	
		}

		if(UpdateStatus)	// if we need to update Time \ Date - delete the previous data
		{
			FillInverseDisplay(0x00, 0,0,0,130);
			FillInverseDisplay(0x00, 7,0,0,130);
			UpdateStatus = FALSE;
		}	

    }	// end while

}   //end main


/** EOF main.c *************************************************/
//#endif
