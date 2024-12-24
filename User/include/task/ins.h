//
// Created by 59794 on 2024/12/15.
//

#ifndef INS_H
#define INS_H

#include "FreeRTOS.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif
    typedef struct {
        float x;
        float y;
        float z;
    }Axis;

    typedef struct {
        float yaw;
        float pitch;
        float roll;
    }Euler;

    typedef struct {
        Axis space_vel_;
        Axis space_pos_;
        Euler space_att_;
        Euler space_omega_;
        Axis self_vel_;
    }insData_t;

    const insData_t* GetInsData();
    const float GetSpeed();
    static void Ins_Task(void* parameter);
    void InsTaskStart();

#ifdef __cplusplus
}
#endif

#endif //INS_H
