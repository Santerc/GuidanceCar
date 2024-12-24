//
// Created by 18759 on 2024/11/23.
//
#include "fdcan_api.h"

namespace bsp {
    void FdcanTxConfig::Init(uint32_t id, uint32_t dlc, uint32_t baud_rate_switch, uint32_t can_type) {
        tx_header_.Identifier = id;
        if (id >= 0x800)
        {
            tx_header_.IdType = FDCAN_EXTENDED_ID;
        }
        else
        {
            tx_header_.IdType = FDCAN_STANDARD_ID;
        }
        tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        tx_header_.DataLength = dlc;
        tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header_.BitRateSwitch = baud_rate_switch;
        tx_header_.FDFormat = can_type;
        tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_header_.MessageMarker = 0;
    }

    FdcanTxConfig::FdcanTxConfig(uint32_t id, uint32_t dlc, uint32_t baud_rate_switch, uint32_t can_type) {
        tx_header_.Identifier = id;
        if (id >= 0x800)
        {
            tx_header_.IdType = FDCAN_EXTENDED_ID;
        }
        else
        {
            tx_header_.IdType = FDCAN_STANDARD_ID;
        }
        tx_header_.TxFrameType = FDCAN_DATA_FRAME;
        tx_header_.DataLength = dlc;
        tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header_.BitRateSwitch = baud_rate_switch;
        tx_header_.FDFormat = can_type;
        tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_header_.MessageMarker = 0;
    }

    void Fdcan::InitAndStart() {
        if (init_flag_) return;

        FDCAN_FilterTypeDef sFilterConfig;
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = 0;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = 0x0400;
        sFilterConfig.FilterID2 = 0x0000;

        if (HAL_FDCAN_ConfigFilter(p_handle_, &sFilterConfig) != HAL_OK)
        {
            ErrorHandler();
        }
        if (HAL_FDCAN_ConfigGlobalFilter(p_handle_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
        {
            ErrorHandler();
        }
        if (HAL_FDCAN_Start(p_handle_) != HAL_OK)
        {
            ErrorHandler();
        }
        if (HAL_FDCAN_ActivateNotification(p_handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
            ErrorHandler();
        }
        if (HAL_FDCAN_ActivateNotification(p_handle_, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
        {
            ErrorHandler();
        }

        init_flag_ = true;
    }

    void Fdcan::Send(FdcanTxConfig *p_config) {
        [[likely]]if (init_flag_) {
            if (HAL_FDCAN_AddMessageToTxFifoQ(p_handle_, &(p_config->tx_header_), p_config->tx_data_) != HAL_OK)
            {
                ErrorHandler();
            }
        } else {
            InitAndStart();
        }
    }

    void Fdcan::ErrorHandler(){
        while(true){

        }
    }

    Fdcan fdcan1{&hfdcan1};


}


