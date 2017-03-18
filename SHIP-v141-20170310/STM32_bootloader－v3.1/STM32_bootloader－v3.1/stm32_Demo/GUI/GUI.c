#include "delay.h"
#include "lcd.h"
#include "GUI.h"
#include "bigascii.h"
//#include "font.h" 
//#include "text.h"
//#include "bmp2.h"
//画点
//x:0~239
//y:0~319
//POINT_COLOR:此点的颜色
void LCD_DrawPoint(u16 x,u16 y)
{
	 LCD_SetCursor(x,y);		//设置光标位置 
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	//LCD_WR_DATA(0xf800);
	LCD_WR_DATA(POINT_COLOR);  
	//LCD_WR_DATA(255);
}
//清屏函数
//Color:要清屏的填充色
void LCD_Clear(u16 Color)
{
	u32 index=0;	        
	LCD_SetCursor(0x00,0x0000);//设置光标位置 
	LCD_WriteRAM_Prepare();     //开始写入GRAM	 	  
	for(index=0;index<76800;index++)
	{
		LCD_WR_DATA(Color);   	  
	}
}
//在指定区域内填充指定颜色
//区域大小:
//  (xend-xsta)*(yend-ysta)
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j;
	u16 xlen=0;

	xlen=xend-xsta+1;	   
	for(i=ysta;i<=yend;i++)
	{
	 	LCD_SetCursor(xsta,i);      //设置光标位置 
		LCD_WriteRAM_Prepare();     //开始写入GRAM	  
		for(j=0;j<xlen;j++)LCD_WR_DATA(color);//设置光标位置 	    
	}
						  	    
} 
 //画矩形
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
void Draw_Circle(u16 x0,u16 y0,u8 r)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //判断下个点位置的标志
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
		//使用Bresenham算法画圆     
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
//显示一个数字
//x,y:起点坐标
//num:数值(0~9);
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
//在指定位置开始显示一个字符串	    
//支持自动换行
//(x,y):起始坐标
//str  :字符串
//size :字体大小
//mode:0,非叠加方式;1,叠加方式    
//青岛理想智芯科技
//电话：13012418100			   
/*void Show_Str(u8 x,u8 y,u8*str,u8 size,u8 mode)
{												  	  
    u8 bHz=0;     //字符或者中文  	    				    				  	  
    while(*str!=0)//数据未结束
    { 
        if(!bHz)
        {
	        if(*str>0x80)bHz=1;//中文 
	        else              //字符
	        {      
                if(x>(LCD_W-size/2))//换行
				{				   
					y+=size;
					x=0;	   
				}							    
		        if(y>(LCD_H-size))break;//越界返回      
		        if(*str==13)//换行符号
		        {         
		            y+=size;
					x=0;
		            str++; 
		        }  
		        else LCD_ShowChar(x,y,*str,size,mode);//有效部分写入 
				str++; 
		        x+=size/2; //字符,为全字的一半 
	        }
        }else//中文 
        {     
            bHz=0;//有汉字库    
            if(x>(LCD_W-size))//换行
			{	    
				y+=size;
				x=0;		  
			}
	        if(y>(LCD_H-size))break;//越界返回  						     
	        Show_Font(x,y,str,size,mode); //显示这个汉字,空心显示 
	        str+=2; 
	        x+=size;//下一个汉字偏移	    
        }						 
    }   
}  			 	 	
//在指定位置显示一个字符
//x:0~234
//y:0~308
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16
//mode:叠加方式(1)还是非叠加方式(0)
//在指定位置显示一个字符
//x:0~234
//y:0~308
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16
//mode:叠加方式(1)还是非叠加方式(0)
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode)
{  
 
    u8 temp;
    u8 pos,t;
	u16 x0=x;
	u16 colortemp=POINT_COLOR;      
    if(x>232||y>312)return;	    
	//设置窗口		   
	num=num-' ';//得到偏移后的值
	if(!mode) //非叠加方式
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//调用1206字体
			else temp=asc2_1608[num][pos];		 //调用1608字体
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
	}else//叠加方式
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//调用1206字体
			else temp=asc2_1608[num][pos];		 //调用1608字体
			for(t=0;t<size/2;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);//画一个点     
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

	Get_HzMat24(font,dzk);//得到相应大小的点阵数据
	if(mode==0)//正常显示
	{	 
	    for(t=0;t<72;t++)
	    {   												   
		    temp=dzk[t];//得到12数据                          
	        for(t1=0;t1<8;t1++)
			{
				if(temp&0x80)LCD_DrawPoint(x,y);
	 			else 
				{
					tempcolor=POINT_COLOR;
					POINT_COLOR=BACK_COLOR;
					LCD_DrawPoint(x,y);
					POINT_COLOR=tempcolor;//还原
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
	}else//叠加显示
	{
	    for(t=0;t<72;t++)
	    {   												   
		    temp=dzk[t];//得到12数据                          
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
显示图片(图标)
入口参数：(x，y)是开始点的坐标，length是图片长度，high是图片高度。//pic 图片数组的指针
出口参数: 无
说明：用指定位置上显示事先定义的图片。
要显示的图片事先定义在bmp.h中的pic[]数组中，
如果想修改图片大小、内容，请修改bmp.h中的pic[]数组，
建议用Image2Lcd软件将你要显示的图象自动转换为数组数据。 
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
	   LCD_WR_DATA(temp);//逐点显示
	   tmp+=2;
	}while(tmp<num);
}*/
  	
//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{  
		LCD_DrawPoint(uRow,uCol);//画点 
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

