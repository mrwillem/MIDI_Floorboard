/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SSD1309_FSMC_H
#define __SSD1309_FSMC_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
void OLED_Init(void);

void Blank_Line(unsigned char , unsigned char );

void Set_Addressing_Mode(unsigned char);

void Set_Display_Clock(unsigned char);
void Set_Display_Offset(unsigned char);
void Set_Display_On_Off(unsigned char);

void Set_Common_Config(unsigned char);
void Set_Contrast_Control(unsigned char);
void Set_Command_Lock(unsigned char);
void Set_Common_Remap(unsigned char);

void Set_Entire_Display(unsigned char);

void ShowFinePitch(unsigned char);

void ShowPitch(unsigned char , unsigned char , unsigned char );

void Fill_RAM(unsigned char);

void Set_Inverse_Display(unsigned char);
void Set_Multiplex_Ratio(unsigned char);

void Show_Font1014(/*unsigned char,*/ unsigned char, unsigned char, unsigned char);
void Show_Font1523(/*unsigned char,*/ unsigned char, unsigned char, unsigned char);
void Show_Numbers(uint8_t , unsigned char *, unsigned char , unsigned char);

void Set_Precharge_Period(unsigned char);
void Set_Segment_Remap(unsigned char);
void Set_Start_Line(unsigned char);
void Set_VCOMH(unsigned char);



#ifdef __cplusplus
}
#endif

#endif /* __SSD1309_FSMC_H */
