#include "delay.h"
#include "lcd.h"
#include "GUI.h"
#include "bigascii.h"
//#include "font.h" 
//#include "text.h"
//#include "bmp2.h"
//����
//x:0~239
//y:0~319
//POINT_COLOR:�˵����ɫ
void LCD_DrawPoint(u16 x,u16 y)
{
	 LCD_SetCursor(x,y);		//���ù��λ�� 
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	//LCD_WR_DATA(0xf800);
	LCD_WR_DATA(POINT_COLOR);  
	//LCD_WR_DATA(255);
}
//��������
//Color:Ҫ���������ɫ
void LCD_Clear(u16 Color)
{
	u32 index=0;	        
	LCD_SetCursor(0x00,0x0000);//���ù��λ�� 
	LCD_WriteRAM_Prepare();     //��ʼд��GRAM	 	  
	for(index=0;index<76800;index++)
	{
		LCD_WR_DATA(Color);   	  
	}
}
//��ָ�����������ָ����ɫ
//�����С:
//  (xend-xsta)*(yend-ysta)
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j;
	u16 xlen=0;

	xlen=xend-xsta+1;	   
	for(i=ysta;i<=yend;i++)
	{
	 	LCD_SetCursor(xsta,i);      //���ù��λ�� 
		LCD_WriteRAM_Prepare();     //��ʼд��GRAM	  
		for(j=0;j<xlen;j++)LCD_WR_DATA(color);//���ù��λ�� 	    
	}
						  	    
} 
 //������
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//��ָ��λ�û�һ��ָ����С��Բ
//(x,y):���ĵ�
//r    :�뾶
void Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a);             //3           
		LCD_DrawPoint(x0+b,y0-a);             //0           
		LCD_DrawPoint(x0-a,y0+b);             //1       
		LCD_DrawPoint(x0-b,y0-a);             //7           
		LCD_DrawPoint(x0-a,y0-b);             //2             
		LCD_DrawPoint(x0+b,y0+a);             //4               
		LCD_DrawPoint(x0+a,y0-b);             //5
		LCD_DrawPoint(x0+a,y0+b);             //6 
		LCD_DrawPoint(x0-b,y0+a);             
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 
		LCD_DrawPoint(x0+a,y0+b);
	}
}

void LCD_ShowBigChar(u8 x,u16 y,u8 num)
{
    u8 n,t;
	u8 temp;
	u8 t1,deadline;
	u16 colortemp=POINT_COLOR; 
	u8 x0=x;     
 	if(num==':')t1=150;
	else if(num=='.')t1=165;
	else if(num=='C')t1=180;
	else t1=15*num;
	deadline=t1+15;
 	for(;t1<deadline;t1++)
	{	 
		for(n=0;n<16;n++)
		{
			temp=BIG_ASCII[t1][n];
			for(t=0;t<8;t++)
			{
		        if(temp&0x80)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				LCD_DrawPoint(x,y);	   
				temp<<=1;
				x++;
				if(((n%4)==3)&&t==5)
				{
					x=x0;
					y++;  
					break;
				}
			}
		}
	} 	  
	POINT_COLOR=colortemp;	    	   	 	  
}   
//��ʾһ������
//x,y:�������
//num:��ֵ(0~9);
void LCD_ShowBigNum2(u8 x,u16 y,u8 num)
{   
    LCD_ShowBigChar(x,y,(num/10)%10); 
    LCD_ShowBigChar(x+30,y,num%10);        							     
}
void LCD_ShowBigNum3(u8 x,u16 y,u16 num)
{   
    LCD_ShowBigChar(x,y,num/100); 
	  LCD_ShowBigChar(x+30,y,(num%100)/10);
    LCD_ShowBigChar(x+60,y,num%10);        							     
} 
void LCD_ShowBigNum4(u8 x,u16 y,u16 num)
{   
    LCD_ShowBigChar(x,y,(num/1000)); 
    LCD_ShowBigChar(x+30,y,(num%1000/100)); 
	LCD_ShowBigChar(x+60,y,(num%100/10));
	LCD_ShowBigChar(x+90,y,num%10); 							     
}
//��ָ��λ�ÿ�ʼ��ʾһ���ַ���	    
//֧���Զ�����
//(x,y):��ʼ����
//str  :�ַ���
//size :�����С
//mode:0,�ǵ��ӷ�ʽ;1,���ӷ�ʽ    
//�ൺ������о�Ƽ�
//�绰��13012418100			   
/*void Show_Str(u8 x,u8 y,u8*str,u8 size,u8 mode)
{												  	  
    u8 bHz=0;     //�ַ���������  	    				    				  	  
    while(*str!=0)//����δ����
    { 
        if(!bHz)
        {
	        if(*str>0x80)bHz=1;//���� 
	        else              //�ַ�
	        {      
                if(x>(LCD_W-size/2))//����
				{				   
					y+=size;
					x=0;	   
				}							    
		        if(y>(LCD_H-size))break;//Խ�緵��      
		        if(*str==13)//���з���
		        {         
		            y+=size;
					x=0;
		            str++; 
		        }  
		        else LCD_ShowChar(x,y,*str,size,mode);//��Ч����д�� 
				str++; 
		        x+=size/2; //�ַ�,Ϊȫ�ֵ�һ�� 
	        }
        }else//���� 
        {     
            bHz=0;//�к��ֿ�    
            if(x>(LCD_W-size))//����
			{	    
				y+=size;
				x=0;		  
			}
	        if(y>(LCD_H-size))break;//Խ�緵��  						     
	        Show_Font(x,y,str,size,mode); //��ʾ�������,������ʾ 
	        str+=2; 
	        x+=size;//��һ������ƫ��	    
        }						 
    }   
}  			 	 	
//��ָ��λ����ʾһ���ַ�
//x:0~234
//y:0~308
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
//��ָ��λ����ʾһ���ַ�
//x:0~234
//y:0~308
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  
 
    u8 temp;
    u8 pos,t;
	u16 x0=x;
	u16 colortemp=POINT_COLOR;      
    if(x>232||y>312)return;	    
	//���ô���		   
	num=num-' ';//�õ�ƫ�ƺ��ֵ
	if(!mode) //�ǵ��ӷ�ʽ
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//����1206����
			else temp=asc2_1608[num][pos];		 //����1608����
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				LCD_DrawPoint(x,y);	
				temp>>=1; 
				x++;
		    }
			x=x0;
			y++;
		}	
	}else//���ӷ�ʽ
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//����1206����
			else temp=asc2_1608[num][pos];		 //����1608����
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);//��һ����     
		        temp>>=1; 
		    }
		}
	}
	POINT_COLOR=colortemp;	    	   	 	  
}
void Show_Font24(u8 x,u16 y,u8 *font,u8 mode)
{
	u8 temp,t,t1;
	u16 y0=y;
	u8 dzk[72];
	u16 tempcolor;

	Get_HzMat24(font,dzk);//�õ���Ӧ��С�ĵ�������
	if(mode==0)//������ʾ
	{	 
	    for(t=0;t<72;t++)
	    {   												   
		    temp=dzk[t];//�õ�12����                          
	        for(t1=0;t1<8;t1++)
			{
				if(temp&0x80)LCD_DrawPoint(x,y);
	 			else 
				{
					tempcolor=POINT_COLOR;
					POINT_COLOR=BACK_COLOR;
					LCD_DrawPoint(x,y);
					POINT_COLOR=tempcolor;//��ԭ
				}
				temp<<=1;
				y++;
				if((y-y0)==24)
				{
					y=y0;
					x++;
					break;
				}
			}  	 
    	} 
	}else//������ʾ
	{
	    for(t=0;t<72;t++)
	    {   												   
		    temp=dzk[t];//�õ�12����                          
	        for(t1=0;t1<8;t1++)
			{
				if(temp&0x80)LCD_DrawPoint(x,y);   
				temp<<=1;
				y++;
				if((y-y0)==24)
				{
					y=y0;
					x++;
					break;
				}
			}  	 
    	} 
	}    
}*/
/**********************************************************
��ʾͼƬ(ͼ��)
��ڲ�����(x��y)�ǿ�ʼ������꣬length��ͼƬ���ȣ�high��ͼƬ�߶ȡ�//pic ͼƬ�����ָ��
���ڲ���: ��
˵������ָ��λ������ʾ���ȶ����ͼƬ��
Ҫ��ʾ��ͼƬ���ȶ�����bmp.h�е�pic[]�����У�
������޸�ͼƬ��С�����ݣ����޸�bmp.h�е�pic[]���飬
������Image2Lcd�������Ҫ��ʾ��ͼ���Զ�ת��Ϊ�������ݡ� 
************************************************************/
//void GUI_DisPicture(uchar x, uint y, uchar length, uint high ,const uchar *pic)
/*void GUI_DisPicture(u8 x, u16 y, u16 length, u16 high)
{
   u16 temp=0,tmp=0,num=0;
	LCD_setwindow(x,y,x+length-1,y+high-1);
	num=length*high*2;
	do
	{  
	   //temp=pic[tmp]|( pic[tmp+1]<<8);
	   temp=gImage_bmp2[tmp+1];
	   temp=temp<<8;
	   temp=temp|gImage_bmp2[tmp];
	   LCD_WR_DATA(temp);//�����ʾ
	   tmp+=2;
	}while(tmp<num);
}*/
  	
//����
//x1,y1:�������
//x2,y2:�յ�����  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		LCD_DrawPoint(uRow,uCol);//���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}

