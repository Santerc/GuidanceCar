      
//
// Created by 59794 on 2024/12/15.
//

#ifndef UI_H
#define UI_H

#include "FreeRTOS.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

    static void UI_Task(void* parameter);
    void UI_TaskStart();

#ifdef __cplusplus
}
#endif
#endif //UI_H

    