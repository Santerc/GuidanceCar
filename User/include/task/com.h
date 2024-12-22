//
// Created by 59794 on 2024/12/20.
//

#ifndef PCCOM_H
#define PCCOM_H

#include "proto/proto_pc.h"

#ifdef __cplusplus
extern "C" {
#endif
    extern PC2Board_t rxMsg;
    const uint8_t isConnect();
    static void Com_Task(void* parameter);
    void ComTaskStart();
    void PC_CallBack();

#ifdef __cplusplus
}
#endif
PC2Board_t GetRxData();
#endif //PCCOM_H
