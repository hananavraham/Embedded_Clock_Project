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
//static void YourHighPriorityISRCode();
//static void YourLowPriorityISRCode();

BOOL CheckButtonPressed(void);

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
   oledPutROMString((ROM_STRING)" PIC18F Starter Kit  ",0,0);
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
			//sprintf(buffer,"%c",'@');
			//oledPutString(buffer, 0, 20*6);
        }
    }
    else
    {
        if(buttonPressed == TRUE)
        {
            if(buttonPressCounter == 0)
            {
                buttonPressed = FALSE;
				//sprintf(buffer,"%c",'O');
				//oledPutString(buffer, 0, 20*6);
                return TRUE;
            }
            else
            {
                buttonPressCounter--;
				//sprintf(buffer,"%c",'@');
				//oledPutString(buffer, 0, 20*6);
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

	if (pot >= 0 && pot < 256)
		return 2;
	else if(pot >= 256 && pot < 512)
		return 4;

	else if(pot >= 512 && pot < 768)
		return 6;

	else
		return 7;
}



/* In this SubMenu Navigate with Potentiometer */
void SubMenu1()
{
	int Right_button, Left_button;
	int lineSelect, temp;
	BOOL status = TRUE;

	while(1)
	{	
		temp = lineSelect;
				
		oledPutROMString("Sub Menu 1", 0, 5*6);
		oledPutROMString("Execute Action 1", 2, 0);
		oledPutROMString("Execute Action 2", 4, 0);
		oledPutROMString("Sub Sub Menu", 6, 0);
		oledPutROMString("Execute Action 4", 7, 0);
		

		lineSelect =  checkPotNavigate();
		FillInverseDisplay(0xFF, lineSelect,0,0,100);
		mTouchCalibrate();
		/* Read the R button - RA0 */
		Right_button = mTouchReadButton(0);
		/* Read the L button - RA3 */
		Left_button = mTouchReadButton(3);
		ADCON0 = 0x13;
	
		/* wait for a button to be pressed */
		if (Right_button < 900)
		{	
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}
			
			if(lineSelect != 6)
			{
				printExecuteChoice(lineSelect);
				FillDisplay(0x00);
			}
	
			else
			{
				FillDisplay(0x00);
				/* entering to Sub Sub Menu */
				while(1)
				{
					temp = lineSelect;			
					oledPutROMString("Sub Sub Menu", 0, 5*6);
					oledPutROMString("Execute Action 1", 2, 0);
					oledPutROMString("Execute Action 2", 4, 0);
					oledPutROMString("Execute Action 3", 6, 0);
					oledPutROMString("Execute Action 4", 7, 0);
	
					lineSelect =  checkPotNavigate();
					FillInverseDisplay(0xFF, lineSelect,0,0,100);
					mTouchCalibrate();	
					/* Read the R button - RA0 */
					Right_button = mTouchReadButton(0);
					/* Read the L button - RA3 */
					Left_button = mTouchReadButton(3);
					ADCON0 = 0x13;
				
					/* wait for a button to be pressed */
					if (Right_button < 900)
					{	
						/* wait for the button to be released */
						while (Right_button < 900)
						{
							Right_button = mTouchReadButton(0);
						}
						
						printExecuteChoice(lineSelect);
						FillDisplay(0x00);
					}

					if (Left_button < 900)
					{
						/* wait for the button to be released */
						while (Left_button < 900)
						{
							Left_button = mTouchReadButton(3);
						}
						break;
					}

					if(temp != lineSelect)
					{
						FillDisplay(0x00);
					}
				}
			}
			FillDisplay(0x00);
		}

		if (Left_button < 900)
		{
			/* wait for the button to be released */
			while (Left_button < 900)
			{
				Left_button = mTouchReadButton(3);
			}
			return;
		}

		if(temp != lineSelect)
		{
			FillDisplay(0x00);
		}
	}

}

int fixTheIndex(int lineSelect)
{
	switch (lineSelect)
	{
		case 1:
			return 2;
		case 2:
			return 4;
		case 3:
			return 6;
		case 4:
			return 7;
	}
}		

/* In this SubMenu Navigate with Tilt X */
void SubMenu2()
{
	int Right_button, Left_button, Tilt, temp;
	int lineSelect = 1;
	BOOL status = TRUE;
	while (1)
	{
		if(status)
		{
			FillDisplay(0x00);
			oledPutROMString("Sub Menu 2", 0, 5*6);
			oledPutROMString("Execute Action 1", 2, 0);
			oledPutROMString("Execute Action 2", 4, 0);
			oledPutROMString("Execute Action 3", 6, 0);
			oledPutROMString("Execute Action 4", 7, 0);
			status = FALSE;
		}
		temp = (check_XY_Navigate() / 10) * (-1);
		if(temp > 0 && temp < 5)
		{
			lineSelect = temp;
			status = TRUE;
		}

		FillInverseDisplay(0xFF, fixTheIndex(lineSelect),0,0,100);
		mTouchCalibrate();
		/* Read the R button - RA0 */
		Right_button = mTouchReadButton(0);
		/* Read the L button - RA3 */
		Left_button = mTouchReadButton(3);
	
		/* wait for a button to be pressed */
		if (Right_button < 900)
		{	
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}
			switch (lineSelect)
			{
				case 1:	
					printExecuteChoice(2);
					break;
				case 2:
					printExecuteChoice(4);
					break;
				case 3:
					printExecuteChoice(6);
					break;
				case 4:
					printExecuteChoice(7);
					break;
			}
			status = TRUE;
		}

		if (Left_button < 900)
		{
			/* wait for the button to be released */
			while (Left_button < 900)
			{
				Left_button = mTouchReadButton(3);
			}
			return;
		}
	}
}


/* In this SubMenu Navigate with R , L buttons */
void SubMenu3()
{
	int Right_button, Left_button , button1;
	int lineSelect = 1;
	BOOL status = TRUE;

	while(1)
	{
		if(status)
		{
			FillDisplay(0x00);
			oledPutROMString("Sub Menu 3", 0, 5*6);
	
			oledPrintRectangle(2,0);
			oledPrintRectangle(2,5*6);
			oledPrintRectangle(2,11*6);
			oledPrintRectangle(2,16*6);
	
	
			oledPutROMString("Ex.", 3, 1*6);
			oledPutROMString("Ac1", 5, 1*6);
			oledPutROMString("Ex.", 3, 6*6);
			oledPutROMString("Ac2", 5, 6*6);
			oledPutROMString("Ex.", 3, 12*6);
			oledPutROMString("Ac3", 5, 12*6);
			oledPutROMString("Ex.", 3, 17*6);
			oledPutROMString("Ac4", 5, 17*6);
			status = FALSE;
		}
	
		printCurrentHorizontalLine(lineSelect);	

	
		if(CheckButtonPressed())
		{
			printExecuteChoice(lineSelect *2);
			status = TRUE;		
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

			if(lineSelect < 4)
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

void printSubMenu4(int i)
{
	int j,temp;
	unsigned char x;
	x = 0x30;
	temp = i % 10;
	FillDisplay(0x00);
	oledPutROMString("Sub Menu 4", 0, 5*6);

	for(j = 2 ; j <= 7; j++ ,i++)
	{
		oledPutROMString("Long Execute Action", j, 0);
		if( i <= 9)
			oledWriteChar1x(x + i, j, 19*6);

		else
		{
			oledWriteChar1x(x + (i /10), j, 19*6);
			oledWriteChar1x(x + (i % 10), j, 20*6);
		}
	}
}

void SubMenu4()
{
	char* buf;
	int button1, button2, Left_button;
	int lineSelect = 1;
	int i =1;
	BOOL status = TRUE;

	while(1)
	{
		if(status)
		{
			printSubMenu4(i);
			status = FALSE;
		}
		
		if(lineSelect -i !=0)	
			FillInverseDisplay(0xFF, (lineSelect - i +1) ,0,0,130);
		else
			FillInverseDisplay(0xFF, (lineSelect - i +2) ,0,0,130);

		if(CheckButtonPressed())
		{
			if(lineSelect == 1)
				printLongMenuExecuteChoice(lineSelect);
			else
				printLongMenuExecuteChoice(lineSelect -1);
			status = TRUE;
		}

		mTouchCalibrate();
		/* Scroll bar - RA1/RA2*/
		button1 = mTouchReadButton(1);
		button2 = mTouchReadButton(2);

		if ((button1 < 800) || (button2 < 800))
		{
			if (button1 < button2)
			{
				if (lineSelect > 1)
				{
					status = TRUE;
					lineSelect--;
					if(lineSelect >1)
						if(lineSelect == i)
							--i;
				}
			}		
			else
			{
				if(lineSelect < 16)
				{
					status = TRUE;
					lineSelect++;
					if (lineSelect - i > 6)
						++i;
				}
				
			}
			/* wait for the slider to be released */
			while ((button1 < 800))
			{
				button1 = mTouchReadButton(1);
				button2 = mTouchReadButton(2);
			}
		}

		/* check if user shake the board to return to Main Menu */
		if(!check_XY_Navigate())
			return;
	}
}


void EnterSubMenu(int lineSelect)
{
	switch (lineSelect)
	{
		case 1:
			SubMenu1();
			break;

		case 2:
			SubMenu2();
			break;

		case 3:
			SubMenu3();
			break;

		case 4:
			SubMenu4();
			break;

		default:
			break;
	}
}



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
	int Right_button = 0;
	int Left_button = 0;
	int lineSelect, temp;
	InitializeSystem();

    while(1)							//Main is Usualy an Endless Loop
    {
		
		while(x)	
		{
			x = x&ADCON0;	
		}

		ADCON0 = 0x13;	
		
		temp = lineSelect;
		
		oledPutROMString("00:00:00", 0, 15*6);			
		oledPutROMString("Display Mode", 1, 0);
		oledPutROMString("12H/24H Interval", 3, 0);
		oledPutROMString("Set Time", 5, 0);
		oledPutROMString("Set Date", 6, 0);
		oledPutROMString("Alarm", 7, 0);

		lineSelect =  checkPotNavigate();	
		FillInverseDisplay(0xFF, lineSelect,0,0,60);

		mTouchCalibrate();
		Right_button = mTouchReadButton(0);
		Left_button = mTouchReadButton(3);
		//ADCON0 = 0x13;

		/* Check if the R button was pressed */
		if (Right_button < 900)
		{
			/* wait for the button to be released */
			while (Right_button < 900)
			{
				Right_button = mTouchReadButton(0);
			}
			ADCON0 = 0x13;

			if(lineSelect == 7)
				EnterSubMenu(4);			// moving to the selected subMenu
			else
				EnterSubMenu(lineSelect / 2);

			FillDisplay(0x00);
		}

		if(temp != lineSelect)
		{
			FillDisplay(0x00);
		}

    }	// end while
}//end main


/** EOF main.c *************************************************/
//#endif
