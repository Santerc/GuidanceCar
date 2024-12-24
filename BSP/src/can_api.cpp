
#include "can_api.h"

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;

uint8_t bit8tofloat[4];

uint8_t can_data[8];

FDCAN_TxFrame_TypeDef TxFrame;

void BSP_FDCAN_Init(void){

		TxFrame.hcan = &hfdcan1;
		  TxFrame.Header.Identifier = 0x237;
		TxFrame.Header.IdType = FDCAN_STANDARD_ID;
		TxFrame.Header.TxFrameType = FDCAN_DATA_FRAME;
		TxFrame.Header.DataLength = 3;
		  TxFrame.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
		TxFrame.Header.BitRateSwitch = FDCAN_BRS_OFF;
		TxFrame.Header.FDFormat =  FDCAN_CLASSIC_CAN;
		TxFrame.Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
		TxFrame.Header.MessageMarker = 0;


  FDCAN_FilterTypeDef FDCAN1_FilterConfig;

	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_FilterConfig.FilterIndex = 0;
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_FilterConfig.FilterID1 = 0x00000000;
  FDCAN1_FilterConfig.FilterID2 = 0x00000000;

	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void FDCAN1_RxFifo0RxHandler(uint32_t StdId,uint8_t Data[8])
{

	if (StdId == 0x407) {
	    for (int i = 6; i >= 0; i --) {
	        can_data[i] = Data[i];
	    }
	}


}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data);

  FDCAN1_RxFifo0RxHandler(FDCAN1_RxFrame.Header.Identifier,FDCAN1_RxFrame.Data);

}



