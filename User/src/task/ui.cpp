//
// Created by 59794 on 2024/12/15.
//
#include "task/ui.h"

#include "i2c.h"
#include "main.h"
#include "periph/oled.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include "fsm.hpp"
#include "task/chassis.h"
#include "task/com.h"

periph::oled OLED(&hi2c1);
uint8_t count;
uint8_t car_velocity;
float speed = 111.1;
uint8_t stop_num;
uint8_t car_direction;
char speedcharbuff[4] = {};
/*
 *遥控状态：方向(前进、后退、左转、右转)、车速（快、慢）
 *正常行驶状态：ON、第几站
 *停靠状态：STOP、第几站、停靠倒计时
 *避障：BLOCK
 */

char* SpeedfloatToString(float num, char* buffer, size_t size)
{
    // 使用sprintf格式化浮动数，保留4位有效数字
    std::snprintf(buffer, size, "%.4g m/s", num);  // 保留4位有效数字并附加单位
    return buffer;

}

/*显示方向*/
void DirectionShow(uint8_t direction)
{
    switch (direction)
    {
    case 0:/*前进*/
        {
            OLED.ShowChinese16x16(40,0,0);
            OLED.ShowChinese16x16(40+16,0,1);
            break;
        }
    case 1:/*后退*/
        {
            OLED.ShowChinese16x16(40,0,2);
            OLED.ShowChinese16x16(40+16,0,3);
            break;
        }
    case 2:/*左转*/
        {
            OLED.ShowChinese16x16(40,0,4);
            OLED.ShowChinese16x16(40+16,0,6);
            break;
        }
    case 3:/*右转*/
        {
            OLED.ShowChinese16x16(40,0,5);
            OLED.ShowChinese16x16(40+16,0,6);
            break;
        }
    default:
        {
            break;
        }
    }
}

/*显示速度和挡位*/
void VelocityShow(uint8_t velocity)
{
    switch (velocity)
    {
    case 1:/*一挡*/
        {
            OLED.ShowChinese16x16(36,2,7);
            OLED.ShowChinese16x16(36+16,2,11);
            OLED.ShowString8x16(36+16+16,2,SpeedfloatToString(speed,speedcharbuff,4));
            break;
        }
    case 2:/*二挡*/
        {
            OLED.ShowChinese16x16(40,2,8);
            OLED.ShowChinese16x16(40+16,2,11);
            OLED.ShowString8x16(36+16+16,2,SpeedfloatToString(speed,speedcharbuff,4));
            break;
        }
    case 3:/*三挡*/
        {
            OLED.ShowChinese16x16(40,2,9);
            OLED.ShowChinese16x16(40+16,2,11);
            OLED.ShowString8x16(36+16+16,2,SpeedfloatToString(speed,speedcharbuff,4));
            break;
        }
    case 4:/*四挡*/
        {
            OLED.ShowChinese16x16(40,2,10);
            OLED.ShowChinese16x16(40+16,2,11);
            OLED.ShowString8x16(36+16+16,2,SpeedfloatToString(speed,speedcharbuff,4));
            break;
        }
    default:
        {
            break;
        }
    }
}

/*显示状态*/
void StateShow()
{
    switch (GetState())
    {
    case 0:
        {
            OLED.ShowString8x16(40,4,const_cast<char*>("ON"));
            break;
        }
    case 6:
        {
            OLED.ShowString8x16(8,4,const_cast<char*>("STOP"));
            break;
        }
    case 2:
    case 3:
        {
            OLED.ShowString8x16(8,4,const_cast<char*>("BLOCK"));
            break;
        }
    default:
        {
            break;
        }
    }

}

void ShowStop()
{
    OLED.ShowChinese16x16(32,2,12);
    OLED.ShowChar8x16(32+16,2,static_cast<char>(stop_num+48));
    OLED.ShowChinese16x16(32+16+16,2,13);
}

char intToStringChars(uint8_t num) {
    if (num < 0 || num > 99) {
        std::cerr << "Error: Input must be a two-digit integer (0-99)." << std::endl;
        return '\0';
    }

    char tens = '0' + (num / 10);
    char ones = '0' + (num % 10);

    return tens;
}

void ShowStopTime()
{
    OLED.ShowChinese16x16(40,4,14);
    OLED.ShowChinese16x16(40+16,4,15);
    OLED.ShowChar8x16(40+16+16,4,':');
    if(count >= 10)
    {
        char tens = '0' + (count / 10);
        char ones = '0' + (count % 10);
        OLED.ShowChar8x16(40+16+16+16,4,tens);
        OLED.ShowChar8x16(40+16+16+16+16,4,ones);
    }else
    {
        char num = '0' + count;
        OLED.ShowChar8x16(40+16+16+16,4,num);
    }
}
/*总体显示*/
void Show()
{
    if(GetLastState() != GetState())
    {
        OLED.clear();/*切换状态清屏*/
    }
    switch (GetState())
    {
        case 1:
            {
                VelocityShow(car_velocity);
                DirectionShow(car_direction);
                break;
            }
        case 2:
            {
                StateShow();
                ShowStop();
                break;
            }
        case 3:
            {
                StateShow();
                ShowStop();
                ShowStopTime();
                break;
            }
        case 4:
            {
                StateShow();
                break;
            }
        default:
            {
                break;
            }
    }

}

void UI_Task(void*argument)
{
  for(;;)
  {
        Show();
  }
}
void UI_TaskStart(void){
    xTaskCreate(UI_Task,"UI_Task",256,NULL,7,NULL);
}