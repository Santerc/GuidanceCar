#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"
#include "fdcan.h"



void BSP_FDCAN_Init(void);

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t				Data[8];
}FDCAN_TxFrame_TypeDef;

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			Data[8];
} FDCAN_RxFrame_TypeDef;

extern  FDCAN_TxFrame_TypeDef   TxFrame;

extern uint8_t can_data[8];

extern  uint8_t bit8tofloat[4];

#ifdef __cplusplus
    }
#endif

#endif