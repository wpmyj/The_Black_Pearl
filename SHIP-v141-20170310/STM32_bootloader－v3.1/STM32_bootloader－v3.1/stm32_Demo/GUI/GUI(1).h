#ifndef __AD_H
#define __AD_H
void LCD_DrawPoint(u16 x,u16 y);
void LCD_Clear(u16 Color);
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
void Draw_Circle(u16 x0,u16 y0,u8 r);
void LCD_ShowBigChar(u8 x,u16 y,u8 num);
void LCD_ShowBigNum2(u8 x,u16 y,u8 num);
void LCD_ShowBigNum3(u8 x,u16 y,u16 num);
void LCD_ShowBigNum4(u8 x,u16 y,u16 num);
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode);
//void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode);
void Show_Str(u8 x,u8 y,u8*str,u8 size,u8 mode);
void Show_Str24(u8 x,u16 y,u8*str,u8 mode);
void GUI_DisPicture(u8 x, u16 y, u16 length, u16 high);
#endif
