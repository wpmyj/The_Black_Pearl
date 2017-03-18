#include "lcd.h"
#include "delay.h"
//#include "bmp2.h"
#include "image2lcd.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//FLASHͼƬ��ʾ	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2011/10/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved											  
////////////////////////////////////////////////////////////////////////////////// 	
 	 

//��8λ���ݻ��16λ��ɫ
//mode:0,��λ��ǰ,��λ�ں�.
//     1,��λ��ǰ,��λ�ں�.
//str:����
u16 image_getcolor(u8 mode,u8 *str)
{
	u16 color;
	if(mode)
	{
		color=((u16)*str++)<<8;
		color|=*str;
	}else
	{
		color=*str++;
		color|=((u16)*str)<<8;
	}
	return color;	
}
//��Һ���ϻ�ͼ
//xsta,ysta,xend,yend:��ͼ����
//scan:��image2lcd V2.9��˵��.
//*p:ͼ������
void image_show(u16 xsta,u16 ysta,u16 xend,u16 yend,u8 scan,u8 *p)
{  
	u32 i;
	u32 len=0;
	LCD_setwindow(xsta,ysta,xend,yend);
	if((scan&0x03)==0)//ˮƽɨ��
	{
		switch(scan>>6)//����ɨ�跽ʽ
		{
			case 0:
				LCD_Scan_Dir(L2R_U2D);//������,���ϵ���
				LCD_SetCursor(xsta,ysta);//���ù��λ�� 
				break; 
			case 1:
				LCD_Scan_Dir(L2R_D2U);//������,���µ���
				LCD_SetCursor(xsta,yend);//���ù��λ�� 
				break; 
			case 2:
				LCD_Scan_Dir(R2L_U2D);//���ҵ���,���ϵ���
				LCD_SetCursor(xend,ysta);//���ù��λ�� 
				break; 
			case 3:
				LCD_Scan_Dir(R2L_D2U);//���ҵ���,���µ���
				LCD_SetCursor(xend,yend);//���ù��λ�� 
				break; 
		}
	}else  //��ֱɨ��
	{
		switch(scan>>6)//����ɨ�跽ʽ
		{
			case 0:
				LCD_Scan_Dir(U2D_L2R);//���ϵ���,������
				LCD_SetCursor(xsta,ysta);//���ù��λ�� 
				break; 
			case 1:
				LCD_Scan_Dir(D2U_L2R);//���µ��ϴ�,����
				LCD_SetCursor(xsta,yend);//���ù��λ�� 
				break; 
			case 2:
				LCD_Scan_Dir(U2D_R2L);//���ϵ���,���ҵ��� 
				LCD_SetCursor(xend,ysta);//���ù��λ�� 
				break; 
			case 3:
				LCD_Scan_Dir(D2U_R2L);//���µ���,���ҵ���
				LCD_SetCursor(xend,yend);//���ù��λ�� 
				break; 
		}
	}
	LCD_WriteRAM_Prepare();     		//��ʼд��GRAM
	len=(xend-xsta+1)*(yend-ysta+1);	//д������ݳ���
	for(i=0;i<len;i++)
	{
		LCD_WR_DATA(image_getcolor(scan&(1<<4),p));
		p+=2;
	} 	 
#if USE_HORIZONTAL  //ʹ�ú���	
	LCD_setwindow(0,0,319,239);
#else
	LCD_setwindow(0,0,239,319);
#endif	    					  	    
}  

//��ָ����λ����ʾһ��ͼƬ
//�˺���������ʾimage2lcd������ɵ�����16λ���ɫͼƬ.
//����:1,�ߴ粻�ܳ�����Ļ������.
//     2,��������ʱ���ܹ�ѡ:��λ��ǰ(MSB First)
//     3,�������ͼƬ��Ϣͷ����
//x,y:ָ��λ��
//imgx:ͼƬ����(�������ͼƬ��Ϣͷ,"4096ɫ/16λ���ɫ/18λ���ɫ/24λ���ɫ/32λ���ɫ����ͼ������ͷ)
//ע��:���STM32,����ѡ��image2lcd��"��λ��ǰ(MSB First)"ѡ��,����imginfo�����ݽ�����ȷ!!
void image_display(u16 x,u16 y,u8 * imgx)
{
	HEADCOLOR *imginfo;
 	u8 ifosize=sizeof(HEADCOLOR);//�õ�HEADCOLOR�ṹ��Ĵ�С
	imginfo=(HEADCOLOR*)imgx;
 	image_show(x,y,x+imginfo->h-1,y+imginfo->w-1,imginfo->scan,imgx+ifosize);		
}















