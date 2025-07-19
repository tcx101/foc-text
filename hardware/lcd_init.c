#include "lcd_init.h"
#include "lcd.h"

// 删除extern声明，改用HAL库延时函数
// HAL库延时函数
void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIOC和GPIOD时钟，因为LCD引脚分布在这两个GPIO上
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    // 设置所有引脚为输出模式，并初始化为高电平
    // GPIOD上的引脚：SCL(PD5)、SDA(PD7)、RES(PD3)、DC(PD0)、BLK(PD1)、CS(PD4)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_SET);
    
    // 初始化GPIOD控制引脚 - 提高速度到VERY_HIGH
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // 无上拉下拉，减少电容
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 最高速度
    
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 初始状态设置
    LCD_CS_Set();   // CS默认高电平
    LCD_SCLK_Set(); // SCK默认高电平
    LCD_MOSI_Set(); // MOSI默认高电平
}

/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat) 
{	
    uint8_t i;
    LCD_CS_Clr();
    
    // 移除多余延时
    
    for(i=0; i<8; i++)
    {			  
        LCD_SCLK_Clr();
        
        if(dat & 0x80)
        {
           LCD_MOSI_Set();
        }
        else
        {
           LCD_MOSI_Clr();
        }
        
        LCD_SCLK_Set();
        
        dat <<= 1;
    }
    	
    LCD_CS_Set();
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(uint8_t dat)
{
    LCD_Writ_Bus(dat);
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
    LCD_Writ_Bus(dat>>8);
    LCD_Writ_Bus(dat);
}

/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(uint8_t dat)
{
    LCD_DC_Clr();//写命令
    LCD_Writ_Bus(dat);
    LCD_DC_Set();//写数据
}

void LCD_Init(void)
{
    LCD_GPIO_Init(); // 初始化GPIO
    
    // 确保所有控制信号处于非活动状态
    LCD_CS_Set();  // CS高电平
    LCD_DC_Set();  // DC高电平
    LCD_BLK_Clr(); // 背光关闭
    
    // 复位序列
    LCD_RES_Clr();
    delay_ms(100);  // 增加复位时间
    LCD_RES_Set();
    delay_ms(100);  // 增加复位恢复时间
    
    // ST7789V2初始化序列
    LCD_WR_REG(0x11); // 退出睡眠模式
    delay_ms(120);    // 必须延时120ms以上
    
    // 初始化显示控制参数
    LCD_WR_REG(0x36); // 内存数据访问控制 (MADCTL)
    LCD_WR_DATA8(0x08); // 竖屏模式，BGR像素格式（修复颜色通道）
    
    // 接口像素格式
    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05); // 16位/像素 (65K色)
    
    // 显示功能控制
    LCD_WR_REG(0xB6);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x82);
    
    // PORCH控制
    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);
    
    // VGH VGL设置
    LCD_WR_REG(0xB7);
    LCD_WR_DATA8(0x35);
    
    // VCOM设置
    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x28); // 修改VCOM值
    
    // LCM控制
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x0C);
    
    // VDV和VRH命令使能
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01);
    
    // VRH设置
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x10);
    
    // VDV设置
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20);
    
    // 帧率控制
    LCD_WR_REG(0xC6);
    LCD_WR_DATA8(0x0F); // 60Hz
    
    // 电源控制
    LCD_WR_REG(0xD0);
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);
    
    // 正极性伽马校正
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x02);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x32);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x42);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x17);
    
    // 负极性伽马校正
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x02);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x31);
    LCD_WR_DATA8(0x54);
    LCD_WR_DATA8(0x47);
    LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x1C);
    LCD_WR_DATA8(0x17);
    LCD_WR_DATA8(0x1B);
    LCD_WR_DATA8(0x1E);
    
    // 设置地址范围
    LCD_WR_REG(0x2A);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(COL_OFFSET); // 起始列
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8((LCD_W-1)+COL_OFFSET); // 结束列
    
    LCD_WR_REG(0x2B);
    LCD_WR_DATA8(ROW_OFFSET>>8);
    LCD_WR_DATA8(ROW_OFFSET & 0xFF); // 起始行
    uint16_t row_end = ROW_OFFSET + LCD_H - 1;
    LCD_WR_DATA8(row_end >> 8);
    LCD_WR_DATA8(row_end & 0xFF); // 结束行
    
    // 设置像素极性，修复黑白反转问题（打开显示反相）
    LCD_WR_REG(0x21); // 显示反相
    
    // 开启显示
    LCD_WR_REG(0x29);
    
    // 准备写入RAM
    LCD_WR_REG(0x2C);
    
    // 打开背光
    LCD_BLK_Set();
    
    delay_ms(10);
} 








