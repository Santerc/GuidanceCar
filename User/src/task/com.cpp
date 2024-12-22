//
// Created by 59794 on 2024/12/20.
//
#include <cmsis_os.h>
#include "task/com.h"

#include "task/chassis.h"
#include "uart_api.h"
#include <cstring>

Board2PC_t txMsg;
PC2Board_t rxMsg;
bsp::Uart pc_port(&huart1);
bsp::UartTxConfig Transmit(reinterpret_cast<uint8_t *>(&txMsg), sizeof(txMsg), 10);
bool connect_on = true;
uint32_t last_stamp = 10091009;

void PC_CallBack() {
    last_stamp = rxMsg.timestamp;
    memcpy(&rxMsg, pc_port.rx_data_, sizeof(rxMsg));
    if (rxMsg.timestamp == last_stamp) {
        connect_on = false;
    }else {
        connect_on = true;
    }
}


void ComInit() {
    pc_port.InitAndStart();
    // pc_port.AddCallback(PC_CallBack);
}



void SetTxData() {
    txMsg.timestamp = HAL_GetTick();
    txMsg.stopFlag_ = GetStopFlag();
    txMsg.move_ = GetMoveState();
    Transmit.Init(reinterpret_cast<uint8_t *>(&txMsg), sizeof(txMsg), 10);
}

void DataSend() {
    pc_port.Send(&Transmit);
}

const uint8_t isConnect() {
    return connect_on;
}

 PC2Board_t GetRxData() {
    return rxMsg;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        PC_CallBack();
    }
}

static void Com_Task(void* parameter)
{

    ComInit();

    while (true)
    {
        if (1) {
            SetTxData();
            DataSend();
        }

        osDelay(10);
    }
}
void ComTaskStart(void)
{
    xTaskCreate(Com_Task, "ComTask", 256, NULL, 7, NULL);
}




