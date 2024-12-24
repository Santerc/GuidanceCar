      
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
#include "task/ins.h"

periph::oled OLED(&hi2c1);
uint8_t car_velocity;
float speed = 0.0;
uint8_t stop_num;
uint8_t stopnumber=0;
uint8_t car_direction;
char speedcharbuff[4] = {};
/*
 *遥控状态：方向(前进、后退、左转、右转)、车速（快、慢）
 *正常行驶状态：ON、第几站
 *停靠状态：STOP、第几站、停靠倒计时
 *避障：BLOCK
 */

void floatToStr(float num, char* str) {
    if (fabs(num) < 10.0) {
        sprintf(str, "%.4f", num);
    } else if (fabs(num) >= 10.0 && fabs(num) < 1000.0) {
        sprintf(str, "%.3f", num);
    } else {
        sprintf(str, "%.3e", num);
    }

    str[4] = '\0';  // 截断多余的字符
}

/*显示方向*/
void DirectionShow(uint8_t direction)
{
    switch (direction)
    {
    case 1:/*前进*/
        {
            OLED.ShowChinese16x16(40,0,0);
            OLED.ShowChinese16x16(40+16,0,1);
            break;
        }
    case 2:/*后退*/
        {
            OLED.ShowChinese16x16(40,0,2);
            OLED.ShowChinese16x16(40+16,0,3);
            break;
        }
    case 4:/*左转*/
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
            OLED.ShowString8x16(40,0,const_cast<char*>("STOP"));
            break;
        }
    }
}

/*显示速度和挡位*/
void VelocityShow(uint8_t velocity)
{
    // floatToStr(GetSpeed(),speedcharbuff);
    // OLED.ShowString8x16(40,4,speedcharbuff);
    // OLED.ShowString8x16(40+32,4,const_cast<char*>("m/s"));
    switch (velocity)
    {
    case 1:/*一挡*/
        {
            OLED.ShowChinese16x16(40,2,7);
            OLED.ShowChinese16x16(40+16,2,11);
            break;
        }
    case 2:/*二挡*/
        {
            OLED.ShowChinese16x16(40,2,8);
            OLED.ShowChinese16x16(40+16,2,11);
            break;
        }
    case 3:/*三挡*/
        {
            OLED.ShowChinese16x16(40,2,9);
            OLED.ShowChinese16x16(40+16,2,11);
            break;
        }
    case 4:/*四挡*/
        {
            OLED.ShowChinese16x16(40,2,10);
            OLED.ShowChinese16x16(40+16,2,11);
            break;
        }
    default:
        {
            OLED.ShowChinese16x16(40,2,14);
            OLED.ShowChinese16x16(40+16,2,15);
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
            OLED.ShowString8x16(0,0,const_cast<char*>("ON"));
            break;
        }
    case 6:
        {
            OLED.ShowString8x16(0,0,const_cast<char*>("STOP"));
            break;
        }
    case 2:
    case 3:
        {
            OLED.ShowString8x16(32,4,const_cast<char*>("BLOCK"));
            break;
        }
    default:
        {
        OLED.ShowString8x16(0,0,const_cast<char*>("ON"));
            break;
        }
    }

}

void ShowStop()
{
    OLED.ShowChinese16x16(32,2,12);
    OLED.ShowChar8x16(32+16,2,static_cast<char>(stopnumber+48));
    OLED.ShowChinese16x16(32+16+16,2,13);
}

uint8_t ctm = 0;

void ShowStopTime()
{
    OLED.ShowChinese16x16(40,4,14);
    OLED.ShowChinese16x16(40+16,4,15);
    OLED.ShowChar8x16(40+16+16,4,':');
    char cnt[3];
    ctm = Get_cnt_time();
    cnt[0] = '0' + (Get_cnt_time()/ 10);
    cnt[1] = '0' + (Get_cnt_time() % 10);
    cnt[2] = '\0';
    OLED.ShowString8x16(40+16+16+16,4,cnt);
}


/*总体显示*/
uint8_t laststate = 4;

void Show()
{
    if(laststate != GetState())
    {
        OLED.clear();/*切换状态清屏*/
        if (GetState()==kStop) {
            stopnumber++;
        }
        laststate = GetState();


    }
    switch (GetState())
    {
        case 0: {
            StateShow();
            ShowStop();
            break;
        }
        case 4:
            {
                DirectionShow(GetMoveState());
                VelocityShow(rxMsg.speed_);
                break;
            }
        case 6: {
            StateShow();
            ShowStop();
            ShowStopTime();
            break;

        }
        case 2:
        case 3:
            {
                StateShow();
                break;
            }
        default:
            {
            StateShow();
            ShowStop();
                break;
            }
    }

}

void UI_Task(void*argument)
{
    OLED.init();
    OLED.clear();
  for(;;)
  {
        Show();
        osDelay(10);
  }
}
void UI_TaskStart(void){
    xTaskCreate(UI_Task,"UI_Task",512,NULL,7,NULL);
}

    