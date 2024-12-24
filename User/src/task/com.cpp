//
// Created by 59794 on 2024/12/20.
//
#include <cmsis_os.h>
#include "task/com.h"

#include "task/chassis.h"
#include "uart_api.h"
#include "stm32h7xx_hal_uart.h"
#include <cstring>
#include "can_api.h"

Board2PC_t txMsg;
PC2Board_t rxMsg;
bsp::Uart pc_port(&huart1);
bsp::UartTxConfig Transmit(reinterpret_cast<uint8_t *>(&txMsg), sizeof(txMsg), 1000);
bool connect_on = true;
uint32_t last_stamp = 10091009;

uint8_t buffer[16];

void PC_CallBack() {
    // last_stamp = rxMsg.timestamp;
    if (buffer[0] == 25) {
        // rxMsg.connect_ = pc_port.rx_data_[1];
        // rxMsg.timestamp = pc_port.rx_data_[4];
        // rxMsg.move_ = pc_port.rx_data_[2];
        // rxMsg.mode_ = pc_port.rx_data_[3];
        // rxMsg.supercap_ = pc_port.rx_data_[4];
        // rxMsg.speed_ = pc_port.rx_data_[5];
        // rxMsg.stoptime_ = pc_port.rx_data_[6];
        memcpy(&rxMsg, buffer, sizeof(rxMsg));
        // if (rxMsg.timestamp == last_stamp) {
        //     connect_on = false;
        // }else {
            connect_on = true;
        // }
    }
}


void ComInit() {
    BSP_FDCAN_Init();
}

uint8_t TxData[3];


void SetTxData() {
    // TxData[0] = HAL_GetTick();
    TxData[1] = GetStopFlag();
    TxData[0] = GetMoveState();

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


static void Com_Task(void* parameter)
{

    ComInit();
    // rxMsg.mode_ = kTrackingMode;


    while (true)
    {
        rxMsg.counter = can_data[0];
        rxMsg.connect_ = can_data[1];
        rxMsg.mode_ = can_data[2];
        rxMsg.move_ = can_data[3];
        rxMsg.speed_ = can_data[4];
        rxMsg.stoptime_ = can_data[5];
        if (rxMsg.stoptime_<=1) {//YBC
            rxMsg.stoptime_ = 10;//YBC
        }//YBC
        rxMsg.supercap_ = can_data[6];
        TxFrame.Header.Identifier = 0x237;
        TxFrame.Header.DataLength = 2;
        SetTxData();
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxFrame.Header,TxData);
        osDelay(2);
    }
}
void ComTaskStart(void)
{
    xTaskCreate(Com_Task, "ComTask", 256, NULL, 7, NULL);
}




