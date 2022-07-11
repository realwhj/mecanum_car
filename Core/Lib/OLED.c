
 /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled.c/h
  * @brief      0.96 inch oled use SSD1306 driver. the file includes oled initialization function,
  *             and some OLED setting function, GRAM operate function, oled show num ,char and string function,
  *             show RoboMaster LOGO function.
  *             0.96OLED使用SSD1306驱动器，本文件包括初始化函数以及其他OLED设置函数， GRAM操作函数，oled显示数字，字符，字符串函数
  *             以及显示RoboMaster LOGO函数
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done（借用大疆代码，大疆强强强）
  * 
  * 横向为列（128），纵向为页（8） 每页可看作8列？
  * 页地址模式和水平/垂直模式 设置起始终止地址不同（页地址无终止地址）
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "OLED.h" 
#include "oledfont.h"
#include "main.h"
#include "robot.h"
#include <stdio.h>
#include <stdarg.h>
#include <robot_def.h>



extern ROBOT_T*robot_t;

extern I2C_HandleTypeDef hi2c1;
extern float speed_fdb;
static uint8_t OLED_GRAM[128][8];





/**
  * @brief          write data/command to OLED, if you use spi, please rewrite the function
  * @param[in]      dat: the data ready to write
  * @param[in]      cmd: OLED_CMD means command; OLED_DATA means data
  * @retval         none
  */
/**
  * @brief          写数据或者指令到OLED， 如果使用的是SPI，请重写这个函数
  * @param[in]      dat: 要写入的字节
  * @param[in]      cmd: OLED_CMD 代表写入的字节是指令（0x00）; OLED_DATA 代表写入的字节是数据（0x40）
  * @retval         none
  */
void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    static uint8_t cmd_data[2];
    if(cmd == OLED_CMD)
    {
        cmd_data[0] = 0x00;
    }
    else
    {
        cmd_data[0] = 0x40;
    }
    cmd_data[1] = dat;
    HAL_I2C_Master_Transmit(OLED_Port, OLED_I2C_ADDRESS, cmd_data, 2, 10);
}


/**
  * @brief          initialize the oled device
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          初始化OLED模块，
  * @param[in]      none
  * @retval         none
  */
void OLED_init(void)
{
    oled_write_byte(0xAE, OLED_CMD);    //display off
    oled_write_byte(0x20, OLED_CMD);    //Set Memory Addressing Mode	
    oled_write_byte(0x10, OLED_CMD);    //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
                                        //列地址指针将自动增加1。如果列地址指针到达列终止地址, 列地址指针将复位到列起始地址, 但页地址指针不会改变。
    oled_write_byte(0xb0, OLED_CMD);    //Set Page Start Address for Page Addressing Mode,0-7(B0h-B7h)（设置页地址从第1页开始）
    oled_write_byte(0xc8, OLED_CMD);    //Set COM Output Scan Direction（设置列输出扫描方向）
    oled_write_byte(0x00, OLED_CMD);    //---set low column address(00h-0Fh)
    oled_write_byte(0x10, OLED_CMD);    //---set high column address(10h-1Fh) 从列地址为0开始，未设置终止页列地址？？？（页地址模式无法设置终止地址）
    oled_write_byte(0x40, OLED_CMD);    //--set start line address(40h-7Fh)设置起始行
    oled_write_byte(0x81, OLED_CMD);    //--set contrast control register
    oled_write_byte(0xff, OLED_CMD);    //brightness 0x00~0xff
    oled_write_byte(0xa1, OLED_CMD);    //--set segment re-map 0 to 127（设置段重映射）
    oled_write_byte(0xa6, OLED_CMD);    //--set normal display（ram数据为1点亮，0灭）
    oled_write_byte(0xa8, OLED_CMD);    //--set multiplex ratio(1 to 64)
    oled_write_byte(0x3F, OLED_CMD);    //
    oled_write_byte(0xa4, OLED_CMD);    //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_byte(0xd3, OLED_CMD);    //-set display offset
    oled_write_byte(0x00, OLED_CMD);    //-not offset
    oled_write_byte(0xd5, OLED_CMD);    //--set display clock divide ratio/oscillator frequency
    oled_write_byte(0xf0, OLED_CMD);    //--set divide ratio
    oled_write_byte(0xd9, OLED_CMD);    //--set pre-charge period
    oled_write_byte(0x22, OLED_CMD);    //
    oled_write_byte(0xda, OLED_CMD);    //--set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xdb, OLED_CMD);    //--set vcomh
    oled_write_byte(0x20, OLED_CMD);    //0x20,0.77xVcc
    oled_write_byte(0x8d, OLED_CMD);    //--set DC-DC enable
    oled_write_byte(0x14, OLED_CMD);    //
    oled_write_byte(0xaf, OLED_CMD);    //--turn on oled panel

}

/**
  * @brief          turn on OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          打开OLED显示
  * @param[in]      none
  * @retval         none
  */
void OLED_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
  * @brief          turn off OLED display
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          关闭OLED显示
  * @param[in]      none
  * @retval         none
  */
void OLED_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}

/**
  * @brief          operate the graphic ram(size: 128*8 char)
  * @param[in]      pen: the type of operate.
                    PEN_CLEAR: set ram to 0x00
                    PEN_WRITE: set ram to 0xff
                    PEN_INVERSION: bit inversion 
  * @retval         none
  */
/**
  * @brief          操作GRAM内存(128*8char数组)
  * @param[in]      pen: 操作类型.
                    PEN_CLEAR: 设置为0x00
                    PEN_WRITE: 设置为0xff
                    PEN_INVERSION: 按位取反
  * @retval         none
  */
void OLED_operate_gram(pen_typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == PEN_WRITE)
            {
                OLED_GRAM[n][i] = 0xff;
            }
            else if (pen == PEN_CLEAR)
            {
                OLED_GRAM[n][i] = 0x00;
            }
            else
            {
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
            }
        }
    }
}

/**
  * @brief          cursor set to (x,y) point
  * @param[in]      x:X-axis, from 0 to 127
  * @param[in]      y:Y-axis, from 0 to 7
  * @retval         none
  */
/**
  * @brief          设置光标起点(x,y)设置了从那一行列开始显示
  * @param[in]      x:x轴, 从 0 到 127
  * @param[in]      y:y轴, 从 0 到 7
  * @retval         none
  */
void OLED_set_pos(uint8_t x, uint8_t y)
{
    oled_write_byte((0xb0 + y), OLED_CMD);              //set page address y（设置起始页）
    oled_write_byte(((x&0xf0)>>4)|0x10, OLED_CMD);      //set column high address（|0X10：要求高4位 起点为0x10）
    oled_write_byte((x&0x0f), OLED_CMD);                //set column low address
}


/**
  * @brief          draw one bit of graphic raw, operate one point of screan(128*64)
  * @param[in]      x: x-axis, [0, X_WIDTH-1]
  * @param[in]      y: y-axis, [0, Y_WIDTH-1]
  * @param[in]      pen: type of operation,
                        PEN_CLEAR: set (x,y) to 0
                        PEN_WRITE: set (x,y) to 1
                        PEN_INVERSION: (x,y) value inversion 
  * @retval         none
  */
/**
  * @brief          操作GRAM中的一个位，相当于操作屏幕的一个点
  * @param[in]      x:x轴,  [0,X_WIDTH-1]
  * @param[in]      y:y轴,  [0,Y_WIDTH-1]
  * @param[in]      pen: 操作类型,
                        PEN_CLEAR: 设置 (x,y) 点为 0
                        PEN_WRITE: 设置 (x,y) 点为 1
                        PEN_INVERSION: (x,y) 值反转
  * @retval         none
  */
void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ((x < 0) || (x > (X_WIDTH - 1)) || (y < 0) || (y > (Y_WIDTH - 1)))//超出范围
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        OLED_GRAM[x][page] |= 1 << row;   //位操作置要操作的点为1（点亮）
    }
    else if (pen == PEN_INVERSION)
    {
        OLED_GRAM[x][page] ^= 1 << row;   //异或操作取反
    }
    else
    {
        OLED_GRAM[x][page] &= ~(1 << row);
    }
}




/**
  * @brief          draw a line from (x1, y1) to (x2, y2)
  * @param[in]      x1: the start point of line
  * @param[in]      y1: the start point of line
  * @param[in]      x2: the end point of line
  * @param[in]      y2: the end point of line
  * @param[in]      pen: type of operation,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
/**
  * @brief          画一条直线，从(x1,y1)到(x2,y2)
  * @param[in]      x1: 起点
  * @param[in]      y1: 起点
  * @param[in]      x2: 终点
  * @param[in]      y2: 终点
  * @param[in]      pen: 操作类型,PEN_CLEAR,PEN_WRITE,PEN_INVERSION.
  * @retval         none
  */
  
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);   //设置起点和终点

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_draw_point(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}


/**
  * @brief          show a character
  * @param[in]      row: start row of character
  * @param[in]      col: start column of character
  * @param[in]      chr: the character ready to show
  * @retval         none
  */
/**
  * @brief          显示一个字符（相当于字符解码工作）
  * @param[in]      row: 字符的开始行(从0开始)
  * @param[in]      col: 字符的开始列
  * @param[in]      chr: 字符
  * @retval         大小为12*6，12为行宽，6为列宽
  */
void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr)
{
    uint8_t x = col * 5;
    uint8_t y = row * 10;
 
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';  //去掉了前32个字符

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;   //一位一位取该字符（每一位代表该点），如果非0则点亮，0则灭
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}


/**
  * @brief          show a character string
  * @param[in]      row: row of character string begin
  * @param[in]      col: column of character string begin
  * @param[in]      chr: the pointer to character string
  * @retval         none
  */
/**
  * @brief          显示一个字符串
  * @param[in]      row: 字符串的开始行
  * @param[in]      col: 字符串的开始列
  * @param[in]      chr: 字符串
  * @retval         none
  */
void OLED_show_string(uint8_t row, uint8_t col, char chr[20])
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_show_char(row, col, (uint8_t)chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
        n++;
    }
}


//显示1位小数
void OLED_show_float(uint8_t row, uint8_t col,float num)
{
  uint8_t num_integer_ASCII=(int)num+48;
  uint8_t num_decimal_ASCII=(int)(num*10)%10+48;

  uint8_t float_num[3]={num_integer_ASCII,'.',num_decimal_ASCII};

  OLED_show_string(row,col,(char *)float_num);
}


//显示中文
void OLED_show_chinese(uint8_t row, uint8_t col, const unsigned char chinese[][12])
{
	uint8_t x = col*6;
  uint8_t y = row*12;
 
  uint8_t temp, t, t1,str_row;
  uint8_t y0 = y;
	
	for(str_row=0;str_row<18;str_row++)		//根据数组大小修改
	{
    for (t = 0; t < 12; t++)
    {
      temp = chinese[str_row][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;   //一位一位取该字符（每一位代表该点），如果非0则点亮，0则灭
            y++;
            if ((y - y0) == 13)			//根据字体宽度修改
            {
                y = y0;
                x++;
                break;
            }
        }
    }
	}
}


/**
  * @brief          formatted output in oled 128*64
  * @param[in]      row: row of character string begin, 0 <= row <= 4;
  * @param[in]      col: column of character string begin, 0 <= col <= 20;
  * @param          *fmt: the pointer to format character string
  * @note           if the character length is more than one row at a time, the extra characters will be truncated
  * @retval         none
  */
/**
  * @brief          格式输出
  * @param[in]      row: 开始列，0 <= row <= 4;
  * @param[in]      col: 开始行， 0 <= col <= 20;
  * @param[in]      *fmt:格式化输出字符串
  * @note           如果字符串长度大于一行，额外的字符会换行
  * @retval         none
  */

 //可变参数函数
void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...)
{
   static uint8_t LCD_BUF[128] = {0};
   static va_list ap;                   ////用于指向可变参数列表的指针ap
   uint8_t remain_size = 0;

   if ((row > 4) || (col > 20) )        //超出范围
   {
       return;
   }
   va_start(ap, fmt);                   //指针指向列表的第一个可变参数

   vsprintf((char *)LCD_BUF, fmt, ap);  //将fmt写入LCD中,并使ap指向下一个可变参数

   va_end(ap);                            //终止可变参数移动

   remain_size = 21 - col;

   LCD_BUF[remain_size] = '\0';

   OLED_show_string(row, col, (char*)LCD_BUF);
}

/**
  * @brief          send the data of gram to oled sreen
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送数据到OLED的GRAM
  * @param[in]      none
  * @retval         none
  */
void OLED_refresh_gram(void)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        //OLED_set_pos(0, i);   //设置光标起点
        for (n = 0; n < 128; n++)
        {
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}

void OLED_BMP(const unsigned char BMP[128][8])
{
    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;
    OLED_operate_gram(PEN_CLEAR);


    for(; y < 64; y += 8)
    {
        for(x = 0; x < 128; x++)
        {
            temp_char = BMP[x][y/8];
            for(i = 0; i < 8; i++)
            {
                if(temp_char & 0x80)
                {
                    OLED_draw_point(x, y + i,PEN_WRITE);
                }
                else
                {
                    OLED_draw_point(x,y + i,PEN_CLEAR);
                }
                temp_char <<= 1;
            }
        }
    }
    OLED_refresh_gram();

}


//OLED滚动显示
void OLED_Roll(int RollMode)
{
  oled_write_byte(0x2E,OLED_CMD);        //关闭滚动
  if(RollMode==RollHorizontal)
  {
    oled_write_byte(0x27,OLED_CMD);        //水平滚动 26（从左向右滑动）  /  27（从右向左滑动）
    oled_write_byte(0x00,OLED_CMD);        //虚拟字节
    oled_write_byte(0x00,OLED_CMD);        //起始页 0
    oled_write_byte(0x07,OLED_CMD);        //滚动时间间隔
    oled_write_byte(0x07,OLED_CMD);        //终止页 7
    oled_write_byte(0x00,OLED_CMD);        //虚拟字节
    oled_write_byte(0xFF,OLED_CMD);        //虚拟字节      
  }
  else if(RollMode==RollVerticalHor)
  {
    oled_write_byte(0x2a,OLED_CMD);        //垂直和水平滚动 29(垂直向上，水平向右)/2a(垂直向上，水平向左)
    oled_write_byte(0x00,OLED_CMD);        //虚拟字节
    oled_write_byte(0x00,OLED_CMD);        //起始页 0
    oled_write_byte(0x07,OLED_CMD);        //滚动时间间隔
    oled_write_byte(0x07,OLED_CMD);        //终止页 1
    oled_write_byte(0x01,OLED_CMD);        //垂直滚动偏移量(不设置滚动偏移量则和水平一样)
  }
  oled_write_byte(0x2F,OLED_CMD);        //开启滚动
}


//初始化成功后的显示
void InitShow(void)
{
	//清空gram并关闭滚动
	oled_write_byte(0x2E,OLED_CMD);
	OLED_operate_gram(PEN_CLEAR);
	OLED_refresh_gram();
	HAL_Delay(100);
  //显示“初始化成功”字样
  OLED_display_on();
  OLED_show_string(3,2,"Inited Successfully");
  OLED_refresh_gram();
  HAL_Delay(500);

  //显示照片并开始滚动
  OLED_BMP(BMP_Good);
  OLED_Roll(RollHorizontal);

  
}

//robot的状态用oled显示(输出字符串时0<行<4)
void StateOledShow(void)
{
	//关闭滚动
  oled_write_byte(0x2E,OLED_CMD);
  //清空gram
  OLED_operate_gram(PEN_CLEAR);

  //显示motor和control mode状态
  if(robot_t->MotorMode==MotorOff)
	{
    OLED_show_string(0,0,"Motor:Off");
  }
  else OLED_show_string(0,0,"Motor:On");
	if(robot_t->CtrMode==ModeBlueTooth)
	{
		OLED_show_string(1,0,"Mode:BlueTooth");
	}
	else OLED_show_string(1,0,"Mode:Patrol");
 
  //显示期望速度和反馈速度
  OLED_show_string(2,0,"speed_ref:");
	OLED_show_float(2,10,robot_t->robot_config->v_ref);
  OLED_show_string(3,0,"speed_fdb:");
	OLED_show_float(3,10,robot_t->robot_config->v_fdb);

	
	//显示“by：哈工大（深圳）”
	OLED_show_chinese(4,0,HITSZ);
	
	//更新gram
  OLED_refresh_gram();
}








