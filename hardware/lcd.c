#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "main.h"
#include <string.h>
#include <stdio.h>






/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
                color       要填充的颜色
      返回值：  无
******************************************************************************/
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint32_t i, j;
	
	// 确保坐标不超出屏幕范围
	if(xsta >= LCD_W) xsta = LCD_W-1;
	if(xend >= LCD_W) xend = LCD_W-1;
	if(ysta >= LCD_H) ysta = LCD_H-1;
	if(yend >= LCD_H) yend = LCD_H-1;
	
	// 保证x1<=x2,y1<=y2
	if(xsta > xend) {uint16_t temp = xsta; xsta = xend; xend = temp;}
	if(ysta > yend) {uint16_t temp = ysta; ysta = yend; yend = temp;}
	
	// 设置显示区域
	LCD_Address_Set(xsta, ysta, xend, yend);
	
	// 准备颜色数据
	uint8_t colorH = color >> 8;
	uint8_t colorL = color & 0xFF;
	
	// 直接逐行逐列填充
	for(i = ysta; i <= yend; i++) {
		for(j = xsta; j <= xend; j++) {
			LCD_WR_DATA8(colorH);
			LCD_WR_DATA8(colorL);
		}
	}
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	// 检查坐标是否在有效范围内
	if(x >= LCD_W || y >= LCD_H) return;
	
	// 设置绘制点的坐标范围
	LCD_Address_Set(x,y,x,y);
	
	// 写入颜色数据
	LCD_WR_DATA(color);
}


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t color)
{
	uint16_t t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
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


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  ?
******************************************************************************/
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  ?
******************************************************************************/
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r,uint16_t color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要绘制的点是否过圆心
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示汉字字符
      入口数据：x,y显示坐标
                *s 要显示的汉字字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 12 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  ?
******************************************************************************/
void LCD_ShowChinese(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	// 循环显示所有汉字，直到遇到结束符
	while(*s!=0)
	{
		// 根据字号选择不同的显示函数
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		
		// 每个汉字占用2个字节，所以指针+2
		s+=2;
		// 移动到下一个汉字位置
		x+=sizey;
	}
}

/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  ?
******************************************************************************/
void LCD_ShowChinese12x12(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  ?
******************************************************************************/
void LCD_ShowChinese16x16(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j;
	uint16_t k;
	uint16_t HZnum;     // 汉字数目
	uint16_t TypefaceNum; // 一个字符所占字节大小
	uint16_t x0=x;
	
	// 计算一个汉字所占的字节数
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	// 统计汉字库中的汉字数量
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);
	
	// 查找要显示的汉字在字库中的位置
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			// 设置显示区域
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			
			// 按照字节遍历汉字点阵
			for(i=0;i<TypefaceNum;i++)
			{
				// 处理每个字节的8个位
				for(j=0;j<8;j++)
				{	
					// 根据显示模式处理
					if(!mode) // 非叠加模式
					{
						// 若对应位为1，则显示前景色；否则显示背景色
						if(tfont16[k].Msk[i]&(0x01<<j))
							LCD_WR_DATA(fc);
						else 
							LCD_WR_DATA(bc);
					}
					else // 叠加模式
					{
						// 只有对应位为1时才画点
						if(tfont16[k].Msk[i]&(0x01<<j))
							LCD_DrawPoint(x,y,fc);
						
						// 更新横向坐标
						x++;
						
						// 如果一行已满，转到下一行
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  // 找到汉字后跳出循环
	}
} 


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  ?
******************************************************************************/
void LCD_ShowChinese24x24(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  ?
******************************************************************************/
void LCD_ShowChinese32x32(uint16_t x,uint16_t y,uint8_t *s,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t i,j,m=0;
	uint16_t k;
	uint16_t HZnum;//汉字数目
	uint16_t TypefaceNum;//一个字符所占字节大小
	uint16_t x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  ?
******************************************************************************/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
	uint8_t temp, sizex, t;
	uint16_t i, TypefaceNum;//一个字符所占字节大小
	uint16_t x0 = x;
	
	// 字体宽度为高度的一半
	sizex = sizey/2;
	
	// 计算字符的字节数
	TypefaceNum = (sizex/8 + ((sizex%8) ? 1 : 0)) * sizey;
	
	// ASCII码偏移，得到在字库中的位置
	num = num - ' ';
	
	// 设置显示区域
	LCD_Address_Set(x, y, x+sizex-1, y+sizey-1);
	
	// 非叠加模式下，直接写入整块区域
	if(!mode) {
		for(i=0; i<TypefaceNum; i++) {
			if(sizey == 12) temp = ascii_1206[num][i];
			else if(sizey == 16) temp = ascii_1608[num][i];
			else if(sizey == 24) temp = ascii_2412[num][i];
			else if(sizey == 32) temp = ascii_3216[num][i];
			else return;
			
			for(t=0; t<8; t++) {
				if(temp & 0x01) 
					LCD_WR_DATA(fc);
				else 
					LCD_WR_DATA(bc);
				temp >>= 1;
			}
		}
	} else { // 叠加模式，只绘制字符部分
		for(i=0; i<TypefaceNum; i++) {
			if(sizey == 12) temp = ascii_1206[num][i];
			else if(sizey == 16) temp = ascii_1608[num][i];
			else if(sizey == 24) temp = ascii_2412[num][i];
			else if(sizey == 32) temp = ascii_3216[num][i];
			else return;
			
			for(t=0; t<8; t++) {
				if(temp & 0x01) LCD_DrawPoint(x, y, fc);
				x++;
				if(x - x0 == sizex) {
					x = x0;
					y++;
					break;
				}
				temp >>= 1;
			}
		}
	}
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  ?
******************************************************************************/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	// 逐个显示字符串中的每个字符，直到遇到字符串结束符'\0'
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2; // 移动到下一个字符位置（字符宽度为高度的一半）
		p++;        // 指向下一个字符
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  ?
******************************************************************************/
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  ?
******************************************************************************/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;
	uint8_t sizex=sizey/2;
	
	// 逐位显示整数
	for(t=0;t<len;t++)
	{
		// 计算当前位的数字
		temp=(num/mypow(10,len-t-1))%10;
		
		// 处理前导零
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		}
		
		// 显示数字（ASCII码中数字0的编码为48，即'0'）
		LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 

/******************************************************************************
      函数说明：显示带符号的整数变量（正数不显示正号）
      入口数据：x,y显示坐标
                num 要显示的带符号整数变量
                len 要显示的位数(包括符号)
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  ?
******************************************************************************/
void LCD_ShowSignedNum(uint16_t x,uint16_t y,int16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
    uint8_t t,temp;
    uint8_t enshow=0;
    uint8_t sizex=sizey/2;
    
    // 处理符号
    if(num < 0)
    {
        LCD_ShowChar(x,y,'-',fc,bc,sizey,0);
        num = -num;
    }
    else
    {
        LCD_ShowChar(x,y,'+',fc,bc,sizey,0);
    }
    
    // 从符号位后开始显示数字
    x += sizex;
    len--;  // 减去符号位
    
    for(t=0;t<len;t++)
    {
        temp=(num/mypow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
                continue;
            }else enshow=1; 
        }
        LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
    }
} 

/******************************************************************************
      函数说明：显示带符号的整数变量（正数不显示正号）
      入口数据：x,y显示坐标
                num 要显示的带符号整数变量
                len 要显示的位数(包括符号)
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  ?
******************************************************************************/
void LCD_ShowSignedNum2(uint16_t x,uint16_t y,int16_t num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
    uint8_t t,temp;
    uint8_t enshow=0;
    uint8_t sizex=sizey/2;
    
    // 处理符号和调整位数
    if(num < 0)
    {
        LCD_ShowChar(x,y,'-',fc,bc,sizey,0);
        num = -num;
        x += sizex;
        len--;  // 减去符号位
    }
    
    // 显示数字部分
    for(t=0;t<len;t++)
    {
        temp=(num/mypow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
                continue;
            }else enshow=1; 
        }
        LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
    }
} 

/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  ?
******************************************************************************/
void LCD_ShowFloatNum1(uint16_t x,uint16_t y,float num,uint8_t len,uint16_t fc,uint16_t bc,uint8_t sizey)
{         	
	uint8_t t,temp,sizex;
	uint16_t num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  ?
******************************************************************************/
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[])
{
	uint16_t i,j;
	uint32_t k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}

void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
    // 加入偏移量（行/列黑边校正）
    x1 += COL_OFFSET;
    x2 += COL_OFFSET;
    y1 += ROW_OFFSET;
    y2 += ROW_OFFSET;

    // 确保坐标在有效范围内 (含偏移后)
    if(x1 >= LCD_W + COL_OFFSET) x1 = LCD_W + COL_OFFSET - 1;
    if(x2 >= LCD_W + COL_OFFSET) x2 = LCD_W + COL_OFFSET - 1;
    if(y1 >= LCD_H + ROW_OFFSET) y1 = LCD_H + ROW_OFFSET - 1;
    if(y2 >= LCD_H + ROW_OFFSET) y2 = LCD_H + ROW_OFFSET - 1;
    
    LCD_WR_REG(0x2A);    // 列地址设置
    LCD_WR_DATA8(x1 >> 8);
    LCD_WR_DATA8(x1 & 0xFF);
    LCD_WR_DATA8(x2 >> 8);
    LCD_WR_DATA8(x2 & 0xFF);
    
    LCD_WR_REG(0x2B);    // 行地址设置
    LCD_WR_DATA8(y1 >> 8);
    LCD_WR_DATA8(y1 & 0xFF);
    LCD_WR_DATA8(y2 >> 8);
    LCD_WR_DATA8(y2 & 0xFF);
    
    LCD_WR_REG(0x2C);    // 准备写入显示RAM
}

/******************************************************************************
      函数说明：清屏函数
      入口数据：color 要清屏的颜色
      返回值：  无
******************************************************************************/
void LCD_Clear(uint16_t color)
{
	// 清屏就是用指定颜色填充整个屏幕
	LCD_Fill(0,0,LCD_W-1,LCD_H-1,color);
}

/******************************************************************************
  
******************************************************************************/
void LCD_DisplayInit(void)
{
	// 先清屏为黑色
	LCD_Clear(BLACK);
	
	// 延时一小段时间确保显示稳定
	delay_ms(100);
	
	// 可以在这里添加初始化显示的内容
	// 例如显示欢迎信息或程序标题等
	LCD_ShowString(10, 10, (uint8_t*)"ST7789V2 LCD", WHITE, BLACK, 16, 0);
	LCD_ShowString(10, 30, (uint8_t*)"240x320 SPI LCD", WHITE, BLACK, 16, 0);
	
	// 尝试显示一些中文内容
	LCD_ShowChinese(10, 50, (uint8_t*)"中文显示", WHITE, BLACK, 16, 0);
	
	// 显示一些数字
	LCD_ShowIntNum(10, 70, 12345, 5, WHITE, BLACK, 16);
	
	// 在中间区域绘制一个矩形
	LCD_DrawRectangle(60, 100, 180, 220, WHITE);
	
	// 在底部显示当前时间或其他状态信息
	LCD_ShowString(10, 280, (uint8_t*)"LCD Test OK", GREEN, BLACK, 16, 0);
}

/******************************************************************************
      函数说明：LCD调试函数，用于逐步测试LCD功能
      入口数据：无
      返回值：  无
******************************************************************************/
void LCD_Debug(void)
{
    // 基本绘图功能测试
    LCD_Fill(0, 0, LCD_W-1, LCD_H-1, BLACK);  // 全黑背景
    delay_ms(500);
    
    // 测试画点功能
    for(uint16_t i = 0; i < LCD_W; i += 5) {
        LCD_DrawPoint(i, LCD_H/2, RED);       // 在中间画红点
        delay_ms(1);
    }
    delay_ms(500);
    
    // 测试画线功能
    LCD_Fill(0, 0, LCD_W-1, LCD_H-1, BLACK);  // 清屏
    LCD_DrawLine(0, 0, LCD_W-1, LCD_H-1, GREEN);  // 对角线
    LCD_DrawLine(0, LCD_H-1, LCD_W-1, 0, BLUE);   // 反对角线
    delay_ms(500);
    
    // 测试区域填充
    LCD_Fill(10, 10, 60, 60, RED);            // 左上角红色方块
    LCD_Fill(LCD_W-60, 10, LCD_W-10, 60, GREEN); // 右上角绿色方块
    LCD_Fill(10, LCD_H-60, 60, LCD_H-10, BLUE);  // 左下角蓝色方块
    LCD_Fill(LCD_W-60, LCD_H-60, LCD_W-10, LCD_H-10, YELLOW); // 右下角黄色方块
    delay_ms(500);
    
    // 测试文本显示（逐像素方式）
    LCD_Fill(0, 0, LCD_W-1, LCD_H-1, BLACK);  // 清屏
    
    // 在中间区域逐像素绘制"LCD TEST"字样
    uint16_t centerX = LCD_W/2 - 40;
    uint16_t centerY = LCD_H/2 - 8;
    
    // 字母L
    for(uint8_t i = 0; i < 16; i++) {
        LCD_DrawPoint(centerX, centerY+i, WHITE);
    }
    for(uint8_t i = 0; i < 8; i++) {
        LCD_DrawPoint(centerX+i, centerY+15, WHITE);
    }
    centerX += 12;
    
    // 字母C
    for(uint8_t i = 0; i < 16; i++) {
        LCD_DrawPoint(centerX, centerY+i, WHITE);
    }
    for(uint8_t i = 0; i < 8; i++) {
        LCD_DrawPoint(centerX+i, centerY, WHITE);
        LCD_DrawPoint(centerX+i, centerY+15, WHITE);
    }
    centerX += 12;
    
    // 字母D
    for(uint8_t i = 0; i < 16; i++) {
        LCD_DrawPoint(centerX, centerY+i, WHITE);
    }
    for(uint8_t i = 1; i < 7; i++) {
        LCD_DrawPoint(centerX+8, centerY+i, WHITE);
        LCD_DrawPoint(centerX+8, centerY+15-i, WHITE);
    }
    for(uint8_t i = 0; i < 8; i++) {
        LCD_DrawPoint(centerX+i, centerY, WHITE);
        LCD_DrawPoint(centerX+i, centerY+15, WHITE);
    }
    
    delay_ms(1000);
}

/**
 * @brief LCD菜单显示 		
 * 
 */
void windowMenu (void){
	char text[50];
	sprintf (text,"roll:%.2f",imu.roll);
	LCD_ShowString(20,20,(uint8_t*)text,RED,BLACK,16,0);
}
