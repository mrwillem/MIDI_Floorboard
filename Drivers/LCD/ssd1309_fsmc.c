/**
  ******************************************************************************
  * @file    stm324xg_discovery_lcd.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   This file includes the LCD driver for AM-240320L8TNQW00H (LCD_ILI9320)
  *          and AM240320D5TOQW01H (LCD_ILI9325) Liquid Crystal Display Modules
  *          of STM324xG-EVAL evaluation board(MB786) RevB.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; Portions COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/**
  ******************************************************************************
  * <h2><center>&copy; Portions COPYRIGHT 2012 Embest Tech. Co., Ltd.</center></h2>
  * @file    stm32f4_discovery_lcd.c
  * @author  CMP Team
  * @version V1.0.0
  * @date    28-December-2012
  * @brief   LCD LOW_LEVEL Drive
  *          Modified to support the STM32F4DISCOVERY, STM32F4DIS-BB, STM32F4DIS-CAM
  *          and STM32F4DIS-LCD modules.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Embest SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
  * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
  * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1309_fsmc.h"
#include "fonts.c"
#include <stdint.h>


//----------------------------------------------------------------------
//EASTRISING TECHNOLOGY CO,.LTD.//
// Module    : ER-OLED015-2 Series
// Lanuage   : C51 Code
// Create    : JAVEN
// Date      : March-4-2014
// Drive IC  : SSD1309
// INTERFACE : 8-bit 68XX/80XX Parallel, 4-wire SPI
// MCU 		 : AT89LS52
// VDD		 : 3.0V   VCC: 12V
// SA0 connected to VSS. Slave address:White:0x70 Read 0x71
//----------------------------------------------------------------------

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Pin Definition
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define xData	P1				// Parallel Data Input/Output

#define SCLK	P1_0				// Serial Clock Input
#define SDIN	P1_1				// Serial Data Input

#define RES	P3_4				// Reset
#define CS	P3_5				// Chip Select
#define DC	P3_3				// Data/Command Control

#define E	P3_0				// Read/Write Enable
#define RW	P3_1				// Read/Write Select

#define RD	P3_0				// Read Signal
#define WR	P3_1				// Write Signal

#define stop P0_0				// Write Signal

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0x8F


#define		I80				// 8-bit 80XX Parallel
							//   BS1=1; BS2=1

/* Note: LCD /CS is NE1 - Bank 1 of NOR/SRAM Bank 1~4 */
#define  LCD_BASE_Data               ((uint32_t)(0x6C000000|0x00000001))
#define  LCD_BASE_Addr               ((uint32_t)(0x6C000000|0x00000000))
#define  LCD_CMD                     (*(__IO uint8_t *)LCD_BASE_Addr)
#define  LCD_Data                    (*(__IO uint8_t *)LCD_BASE_Data)

#ifndef USE_Delay
	static void delay(__IO uint32_t nCount);
#endif /* USE_Delay*/

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void OLED_Init(void)
{
	uint8_t i;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	for(i=0;i<200;i++)
	{
		delay(200);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

	Set_Command_Lock(0x12);			// Unlock Driver IC (0x12/0x16)
	Set_Display_On_Off(0xAE);		// Display Off (0xAE/0xAF)
	Set_Display_Clock(0xA0);		// Set Clock as 116 Frames/Sec
	Set_Multiplex_Ratio(0x3F);		// 1/64 Duty (0x0F~0x3F)
	Set_Display_Offset(0x00);		// Shift Mapping RAM Counter (0x00~0x3F)
	Set_Start_Line(0x00);			// Set Mapping RAM Display Start Line (0x00~0x3F)
	Set_Addressing_Mode(0x02);		// Set Page Addressing Mode (0x00/0x01/0x02)
	Set_Segment_Remap(0xA1);		// Set SEG/Column Mapping (0xA0/0xA1)
	Set_Common_Remap(0xC8);			// Set COM/Row Scan Direction (0xC0/0xC8)
	Set_Common_Config(0x12);		// Set Alternative Configuration (0x02/0x12)
	Set_Contrast_Control(Brightness);	// Set SEG Output Current
	Set_Precharge_Period(0x25);		// Set Pre-Charge as 2 Clocks & Discharge as 5 Clocks
	Set_VCOMH(0x34);				// Set VCOM Deselect Level
	Set_Entire_Display(0xA4);		// Disable Entire Display On (0xA4/0xA5)
	Set_Inverse_Display(0xA6);		// Disable Inverse Display On (0xA6/0xA7)

	Fill_RAM(0x00);					// Clear Screen

	Set_Display_On_Off(0xAF);		// Display On (0xAE/0xAF)


}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Connection Test
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
/*
void Test()
{
unsigned char i;


	RES=0;
	for(i=0;i<200;i++)
	{
		uDelay(200);
	}
	RES=1;

	Set_Entire_Display(0xA5);		// Enable Entire Display On (0xA4/0xA5)

	while(1)
	{
		Set_Display_On_Off(0xAF);	// Display On (0xAE/0xAF)
		Delay(2);
		Set_Display_On_Off(0xAE);	// Display Off (0xAE/0xAF)
		Delay(2);
	}
}


*/


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Delay Time
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#ifndef USE_Delay
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0;
  for(index = (100 * nCount); index != 0; index--)
  {
  }
}
#endif /* USE_Delay*/


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Read/Write Sequence
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


#ifdef I80					// 8-bit 80XX Parallel
void Write_Command(unsigned char Data)
{
	uint8_t var;
	var=(uint8_t) Data;
	//DC=0;
	//CS=0;
	//WR=0;
	LCD_CMD=var;
	//_nop_();
	//WR=1;
	//CS=1;
	//DC=1;
}

//void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
//{
  /* Write 16-bit Index, then Write Reg */
  //LCD_CMD = LCD_Reg;
  /* Write 16-bit Reg */
  //LCD_Data = LCD_RegValue;
//}

void Write_Data(unsigned char Data)
{
	uint8_t var;
	var=(uint8_t) Data;
	//var=var+0xFF00;
	//DC=1;
	//CS=0;
	//WR=0;
	LCD_Data=var;
	//_nop_();
	//WR=1;
	//CS=1;
	//DC=1;
}
#endif


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Instruction Setting
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Set_Start_Column(unsigned char d)
{
	Write_Command(0x00+d%16);		// Set Lower Column Start Address for Page Addressing Mode
						//   Default => 0x00
	Write_Command(0x10+d/16);		// Set Higher Column Start Address for Page Addressing Mode
						//   Default => 0x10
}


void Set_Addressing_Mode(unsigned char d)
{
	Write_Command(0x20);			// Set Memory Addressing Mode
	Write_Command(d);			//   Default => 0x02
						//     0x00 => Horizontal Addressing Mode
						//     0x01 => Vertical Addressing Mode
						//     0x02 => Page Addressing Mode
}


void Set_Column_Address(unsigned char a, unsigned char b)
{
	Write_Command(0x21);			// Set Column Address
	Write_Command(a);			//   Default => 0x00 (Column Start Address)
	Write_Command(b);			//   Default => 0x7F (Column End Address)
}


void Set_Page_Address(unsigned char a, unsigned char b)
{
	Write_Command(0x22);			// Set Page Address
	Write_Command(a);			//   Default => 0x00 (Page Start Address)
	Write_Command(b);			//   Default => 0x07 (Page End Address)
}


void Set_Start_Line(unsigned char d)
{
	Write_Command(0x40|d);			// Set Display Start Line
						//   Default => 0x40 (0x00)
}


void Set_Contrast_Control(unsigned char d)
{
	Write_Command(0x81);			// Set Contrast Control for Bank 0
	Write_Command(d);			//   Default => 0x7F
}


void Set_Segment_Remap(unsigned char d)
{
	Write_Command(d);			// Set Segment Re-Map
						//   Default => 0xA0
						//     0xA0 => Column Address 0 Mapped to SEG0
						//     0xA1 => Column Address 0 Mapped to SEG127
}


void Set_Entire_Display(unsigned char d)
{
	Write_Command(d);			// Set Entire Display On / Off
						//   Default => 0xA4
						//     0xA4 => Normal Display
						//     0xA5 => Entire Display On
}


void Set_Inverse_Display(unsigned char d)
{
	Write_Command(d);			// Set Inverse Display On/Off
						//   Default => 0xA6
						//     0xA6 => Normal Display
						//     0xA7 => Inverse Display On
}


void Set_Multiplex_Ratio(unsigned char d)
{
	Write_Command(0xA8);			// Set Multiplex Ratio
	Write_Command(d);			//   Default => 0x3F (1/64 Duty)
}


void Set_Display_On_Off(unsigned char d)
{
	Write_Command(d);	//     Set Display On/Off
						//     Default => 0xAE
						//     0xAE => Display Off
						//     0xAF => Display On
	if(d==0xAF)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	}
}


void Set_Start_Page(unsigned char d)
{
	Write_Command(0xB0|d);			// Set Page Start Address for Page Addressing Mode
						//   Default => 0xB0 (0x00)
}


void Set_Common_Remap(unsigned char d)
{
	Write_Command(d);			// Set COM Output Scan Direction
						//   Default => 0xC0
						//     0xC0 => Scan from COM0 to 63
						//     0xC8 => Scan from COM63 to 0
}


void Set_Display_Offset(unsigned char d)
{
	Write_Command(0xD3);			// Set Display Offset
	Write_Command(d);			//   Default => 0x00
}


void Set_Display_Clock(unsigned char d)
{
	Write_Command(0xD5);			// Set Display Clock Divide Ratio / Oscillator Frequency
	Write_Command(d);			//   Default => 0x70
						//     D[3:0] => Display Clock Divider
						//     D[7:4] => Oscillator Frequency
}


void Set_Precharge_Period(unsigned char d)
{
	Write_Command(0xD9);			// Set Pre-Charge Period
	Write_Command(d);			//   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
						//     D[3:0] => Phase 1 Period in 1~15 Display Clocks
						//     D[7:4] => Phase 2 Period in 1~15 Display Clocks
}


void Set_Common_Config(unsigned char d)
{
	Write_Command(0xDA);			// Set COM Pins Hardware Configuration
	Write_Command(d);			//   Default => 0x12
						//     Alternative COM Pin Configuration
						//     Disable COM Left/Right Re-Map
}


void Set_VCOMH(unsigned char d)
{
	Write_Command(0xDB);			// Set VCOMH Deselect Level
	Write_Command(d);			//   Default => 0x34 (0.78*VCC)
}


void Set_NOP()
{
	Write_Command(0xE3);			// Command for No Operation
}


void Set_Command_Lock(unsigned char d)
{
	Write_Command(0xFD);			// Set Command Lock
	Write_Data(d);				//   Default => 0x12
						//     0x12 => Driver IC interface is unlocked from entering command.
						//     0x16 => All Commands are locked except 0xFD.
}



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fill_RAM(unsigned char Data)
{
unsigned char i,j;

	for(i=0;i<8;i++)
	{
		Set_Start_Page(i);
		Set_Start_Column(0x00);

		for(j=0;j<128;j++)
		{
			//Set_Start_Column(j);
			Write_Data(Data);
		}
	}
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (1 Line)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Blank_Line(unsigned char Data, unsigned char page)
{
	unsigned char j;
	Set_Start_Page(page);
	Set_Start_Column(0x00);

	for(j=0;j<128;j++)
	{
		//Set_Start_Column(j);
		Write_Data(Data);
	}
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Regular Pattern (Partial or Full Screen)
//
//    a: Start Page
//    b: End Page
//    c: Start Column
//    d: Total Columns
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fill_Block(unsigned char Data, unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
unsigned char i,j;

	for(i=a;i<(b+1);i++)
	{
		Set_Start_Page(i);
		Set_Start_Column(c);

		for(j=0;j<d;j++)
		{
			Write_Data(Data);
		}
	}
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Checkboard (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Checkerboard()
{
unsigned char i,j;

	for(i=0;i<8;i++)
	{
		Set_Start_Page(i);
		Set_Start_Column(0x00);

		for(j=0;j<64;j++)
		{
			Write_Data(0x55);
			Write_Data(0xaa);
		}
	}
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Frame (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Frame()
{
unsigned char i,j;

	Set_Start_Page(0x00);
	Set_Start_Column(XLevel);

	for(i=0;i<Max_Column;i++)
	{
		Write_Data(0x01);
	}

	Set_Start_Page(0x07);
	Set_Start_Column(XLevel);

	for(i=0;i<Max_Column;i++)
	{
		Write_Data(0x80);
	}

	for(i=0;i<8;i++)
	{
		Set_Start_Page(i);

		for(j=0;j<Max_Column;j+=(Max_Column-1))
		{
			Set_Start_Column(XLevel+j);

			Write_Data(0xFF);
		}
	}
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Character (5x7)
//
//    a: Database
//    b: Ascii
//    c: Start Page
//    d: Start Column
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_Font57(unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
unsigned char *Src_Pointer = NULL;
unsigned char i;
	if(b==32)
		b=(96+32);
	switch(a)
	{
		case 1:
			Src_Pointer=&Ascii_1[(b-0x21)][0];
			break;
		case 2:
			Src_Pointer=&Ascii_2[(b-1)][0];
			break;
	}

	Set_Start_Page(c);
	Set_Start_Column(d);

	for(i=0;i<5;i++)
	{

		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Write_Data(0x00);
}
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Character (5x7)
//
//    a: Database
//    b: Ascii
//    c: Start Page
//    d: Start Column
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_Font1014(/*unsigned char a,*/ unsigned char b, unsigned char c, unsigned char d)
{
unsigned char *Src_Pointer;
unsigned char i;
/*
	switch(a)
	{
		case 1:
			Src_Pointer=&Ascii_1[(b-0x21)][0];
			break;
		case 2:
			Src_Pointer=&Ascii_2[(b-1)][0];
			break;
	}
	*/
	Src_Pointer=&Big[0][0];
	Set_Start_Page(c);
	Set_Start_Column(d);

	for(i=0;i<10;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Write_Data(0x00);
	Src_Pointer=&Big[1][0];
	Set_Start_Page(c+1);
	Set_Start_Column(d);

	for(i=0;i<10;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Write_Data(0x00);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Character (5x7)
//
//    a: Database
//    b: Ascii
//    c: Start Page
//    d: Start Column
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_Font_LucidaSans30(unsigned char character, unsigned char page, unsigned char column)
{
	unsigned char *Src_Pointer;
	unsigned char i;

	Src_Pointer=&lucidaSansUnicode_30ptBitmaps[(character*84)];
	Set_Start_Page(page);
	Set_Start_Column(column);

	for(i=0;i<21;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}

	Src_Pointer=&lucidaSansUnicode_30ptBitmaps[(character*84)+21];
	Set_Start_Page(page+1);
	Set_Start_Column(column);

	for(i=0;i<21;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Src_Pointer=&lucidaSansUnicode_30ptBitmaps[(character*84)+42];
	Set_Start_Page(page+2);
	Set_Start_Column(column);

	for(i=0;i<21;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Src_Pointer=&lucidaSansUnicode_30ptBitmaps[(character*84)+63];
	Set_Start_Page(page+3);
	Set_Start_Column(column);

	for(i=0;i<21;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}

}

void ShowPitch(unsigned char value, unsigned char page, unsigned char column)
{
	unsigned char *Src_Pointer;
	unsigned char i;
	uint8_t pitch;


	if(column>90)
		column=90;


	  switch(value)
	  {
	  	  case 0:
	  		  pitch=0;  // C
	  		  break;
	  	  case 1:
	  		  pitch=0+10; // C#
	  		  break;
		  case 2:
			  pitch=1; // D
			  break;
		  case 3:
			  pitch=1+10; // D#
			  break;
		  case 4:
			  pitch=2; // E
			  break;
		  case 5:
			  pitch=3; // F
			  break;
		  case 6:
			  pitch=3+10; // F#
			  break;
		  case 7:
			  pitch=4;  // G
			  break;
		  case 8:
			  pitch=4+10;  // G#
			  break;
		  case 9:
			  pitch=5;     // A
			  break;
		  case 10:
			  pitch=5+10;  //A#
			  break;
		  case 11:
			  pitch=6;      // B
			  break;
		  default:
			  pitch=0;
			  break;
	  }

		Src_Pointer=&microsoftSansSerif_14ptBitmaps[20*(pitch/10)];
		Set_Start_Page(page);
		Set_Start_Column(column+27);

		for(i=0;i<10;i++)
		{
			Write_Data(*Src_Pointer);
			Src_Pointer++;
		}
		Src_Pointer=&microsoftSansSerif_14ptBitmaps[30*(pitch/10)];
		Set_Start_Page(page+1);
		Set_Start_Column(column+27);

		for(i=0;i<10;i++)
		{
			Write_Data(*Src_Pointer);
			Src_Pointer++;
		}
		pitch=pitch%10;
	Src_Pointer=&lucidaSansUnicode_30ptBitmaps_pitch[((pitch%10)*108)];
	Set_Start_Page(page);
	Set_Start_Column(column);

	for(i=0;i<27;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}

	Src_Pointer=&lucidaSansUnicode_30ptBitmaps_pitch[((pitch%10)*108)+27];
	Set_Start_Page(page+1);
	Set_Start_Column(column);

	for(i=0;i<27;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Src_Pointer=&lucidaSansUnicode_30ptBitmaps_pitch[((pitch%10)*108)+54];
	Set_Start_Page(page+2);
	Set_Start_Column(column);

	for(i=0;i<27;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Src_Pointer=&lucidaSansUnicode_30ptBitmaps_pitch[((pitch%10)*108)+81];
	Set_Start_Page(page+3);
	Set_Start_Column(column);

	for(i=0;i<27;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}

}

void ShowFinePitch(unsigned char value)
{
	unsigned char *Src_Pointer;
	unsigned char i;
	uint8_t pitch;

	Set_Start_Page(5);
	Set_Start_Column(0);
	for(i=0;i<128;i++)
	{
			Write_Data(0x00);
	}
	Set_Start_Page(6);
	Set_Start_Column(0);
	for(i=0;i<128;i++)
	{
			Write_Data(0x00);
	}
	if(value==0x17)
	{
		Src_Pointer=&microsoftSansSerif_24ptBitmaps[30];
		Set_Start_Page(5);
		Set_Start_Column(48);
		for(i=0;i<15;i++)
		{
				Write_Data(*Src_Pointer);
				Src_Pointer++;
		}
		Src_Pointer=&microsoftSansSerif_24ptBitmaps[45];
		Set_Start_Page(6);
		Set_Start_Column(48);
		for(i=0;i<15;i++)
		{
				Write_Data(*Src_Pointer);
				Src_Pointer++;
		}
		Src_Pointer=&microsoftSansSerif_24ptBitmaps[0];
		Set_Start_Page(5);
		Set_Start_Column(65);
		for(i=0;i<15;i++)
		{
				Write_Data(*Src_Pointer);
				Src_Pointer++;
		}
		Src_Pointer=&microsoftSansSerif_24ptBitmaps[15];
		Set_Start_Page(6);
		Set_Start_Column(65);
		for(i=0;i<15;i++)
		{
				Write_Data(*Src_Pointer);
				Src_Pointer++;
		}
	}
	else
	{
		if(value < 0x17)
		{
 			Set_Start_Page(5);
			Set_Start_Column((2*value)-2);
			for(i=0;i<(46-(2*value));i++)
			{
					Write_Data(0xFF);
			}
			Set_Start_Page(6);
			Set_Start_Column((2*value)-2);
			for(i=0;i<(46-(2*value));i++)
			{
					Write_Data(0xFF);
			}

			Src_Pointer=&microsoftSansSerif_24ptBitmaps[30];
			Set_Start_Page(5);
			Set_Start_Column(48);
			for(i=0;i<15;i++)
			{
					Write_Data(*Src_Pointer);
					Src_Pointer++;
			}
			Src_Pointer=&microsoftSansSerif_24ptBitmaps[45];
			Set_Start_Page(6);
			Set_Start_Column(48);
			for(i=0;i<15;i++)
			{
					Write_Data(*Src_Pointer);
					Src_Pointer++;
			}
		}
		else
		{

			Src_Pointer=&microsoftSansSerif_24ptBitmaps[0];
			Set_Start_Page(5);
			Set_Start_Column(65);
			for(i=0;i<15;i++)
			{
					Write_Data(*Src_Pointer);
					Src_Pointer++;
			}
			Src_Pointer=&microsoftSansSerif_24ptBitmaps[15];
			Set_Start_Page(6);
			Set_Start_Column(65);
			for(i=0;i<15;i++)
			{
					Write_Data(*Src_Pointer);
					Src_Pointer++;
			}
			Set_Start_Page(5);
			Set_Start_Column(81);
			for(i=0;i<((2*value)-23);i++)
			{
					Write_Data(0xFF);
			}
			Set_Start_Page(6);
			Set_Start_Column(81);
			for(i=0;i<((2*value)-23);i++)
			{
					Write_Data(0xFF);
			}
		}
	}
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Character (5x7)
//
//    a: Database
//    b: Ascii
//    c: Start Page
//    d: Start Column
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_Font1523(/*unsigned char a,*/ unsigned char b, unsigned char c, unsigned char d)
{
unsigned char *Src_Pointer;
unsigned char i;
/*
	switch(a)
	{
		case 1:
			Src_Pointer=&Ascii_1[(b-0x21)][0];
			break;
		case 2:
			Src_Pointer=&Ascii_2[(b-1)][0];
			break;
	}
	*/
	Src_Pointer=&Xtra[0][0];
	Set_Start_Page(c);
	Set_Start_Column(d);

	for(i=0;i<15;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}

	Src_Pointer=&Xtra[1][0];
	Set_Start_Page(c+1);
	Set_Start_Column(d);

	for(i=0;i<15;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}
	Src_Pointer=&Xtra[2][0];
	Set_Start_Page(c+2);
	Set_Start_Column(d);

	for(i=0;i<15;i++)
	{
		Write_Data(*Src_Pointer);
		Src_Pointer++;
	}

}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show String
//
//    a: Database
//    b: Start Page
//    c: Start Column
//    * Must write "0" in the end...
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_Numbers(uint8_t n_char, unsigned char *Data_Pointer, unsigned char page, unsigned char column)
{

	uint8_t i;

	for(i=0;i<n_char;i++)
	{
		Show_Font_LucidaSans30(Data_Pointer[i], page, column);
		column+=21;
	}

}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show String
//
//    a: Database
//    b: Start Page
//    c: Start Column
//    * Must write "0" in the end...
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_String(unsigned char a, unsigned char *Data_Pointer, unsigned char b, unsigned char c)
{
unsigned char *Src_Pointer;

	Src_Pointer=Data_Pointer;
	Show_Font57(1,96,b,c);			// No-Break Space
						//   Must be written first before the string start...

	while(1)
	{
		Show_Font57(a,*Src_Pointer,b,c);
		Src_Pointer++;
		c+=6;
		if(*Src_Pointer == 0) break;
	}
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Show Pattern (Partial or Full Screen)
//
//    a: Start Page
//    b: End Page
//    c: Start Column
//    d: Total Columns
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Show_Pattern(unsigned char *Data_Pointer, unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
unsigned char *Src_Pointer;
unsigned char i,j;

	Src_Pointer=Data_Pointer;
	for(i=a;i<(b+1);i++)
	{
		Set_Start_Page(i);
		Set_Start_Column(c);

		for(j=0;j<d;j++)
		{
			Write_Data(*Src_Pointer);
			Src_Pointer++;
		}
	}
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Vertical / Fade Scrolling (Full Screen)
//
//    a: Scrolling Direction
//       "0x00" (Upward)
//       "0x01" (Downward)
//    b: Set Numbers of Row Scroll per Step
//    c: Set Time Interval between Each Scroll Step
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Vertical_Scroll(unsigned char a, unsigned char b, unsigned char c)
{
unsigned int i,j;

	switch(a)
	{
		case 0:
			for(i=0;i<Max_Row;i+=b)
			{
				Set_Start_Line(i);
				for(j=0;j<c;j++)
				{
					delay(60);
				}
			}
			break;
		case 1:
			for(i=0;i<Max_Row;i+=b)
			{
				Set_Start_Line(Max_Row-i);
				for(j=0;j<c;j++)
				{
					delay(60);
				}
			}
			break;
	}
	Set_Start_Line(0x00);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade In (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fade_In()
{
unsigned int i;

	Set_Display_On_Off(0xAF);
	for(i=0;i<(Brightness+1);i++)
	{
		Set_Contrast_Control(i);
		delay(300);

	}
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade Out (Full Screen)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Fade_Out()
{
unsigned int i;

	for(i=(Brightness+1);i>0;i--)
	{
		Set_Contrast_Control(i-1);
		delay(300);
	}
	Set_Display_On_Off(0xAE);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Sleep Mode
//
//    "0x00" Enter Sleep Mode
//    "0x01" Exit Sleep Mode
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Sleep(unsigned char a)
{
	switch(a)
	{
		case 0:
			Set_Display_On_Off(0xAE);
			Set_Entire_Display(0xA5);
			break;
		case 1:
			Set_Entire_Display(0xA4);
			Set_Display_On_Off(0xAF);
			break;
	}
}


void Display_Picture(unsigned char *p)
{unsigned char *picture;
    unsigned char i,j,num=0;
		picture=p;
	for(i=0;i<0x08;i++)
	{
	Set_Start_Page(i);
	Set_Start_Column(XLevel);
        for(j=0;j<0x80;j++)
		{
		    Write_Data(*picture);
			picture++;
		}
	}
}








//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Main Program
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
/*
void main()
{
unsigned char code Name[]={'E','A','S','T','R','I','S','I','N','G',0};
unsigned char code UAL[]={'w','w','w','.','b','u','y','-','d','i','s','p','l','a','y','.','c','o','m',0};
unsigned char code Greek[]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,0};

	P1=0xFF;
	P3=0xFF;
	OLED_Init();

	while(1)
	{// Show Pictrue
	Display_Picture(pic);
		Delay(5);
	Display_Picture(pic1);
		Delay(5);
	Display_Picture(pic2);
		Delay(5);
	Display_Picture(pic4);
		Delay(5);

		Fill_RAM(0x00);			// Clear Screen

		// Show Pattern -
		Show_Pattern(UniV,0x02,0x05,XLevel+32,64);
		Delay(2);

	// Fade In/Out (Full Screen)
		Fade_Out();
		Fade_In();
		Fade_Out();
		Fade_In();
		Delay(2);

	// Scrolling (Full Screen)
		Vertical_Scroll(0x00,0x01,0x60);
						// Upward
		Delay(2);
		Vertical_Scroll(0x01,0x01,0x60);
						// Downward
		Delay(2);

	// All Pixels On (Test Pattern)
		Fill_RAM(0xFF);
		Delay(5);

	// Checkerboard (Test Pattern)
		Checkerboard();
		Delay(5);
		Fill_RAM(0x00);		// Clear Screen

	// Frame (Test Pattern)
		Frame();
		Delay(5);

	// Show String -
		Show_String(1,&Name,0x02,XLevel+0x20);
		Show_String(1,&UAL,0x04,XLevel+0x08);
		Show_String(2,&Greek,0x06,XLevel+1);
		Delay(5);
		Fill_RAM(0x00);			// Clear Screen

	}
}
*/



