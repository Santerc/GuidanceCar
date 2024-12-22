//
// Created by 18759 on 2024/11/26.
//
#include "uart_api.h"

namespace bsp{

    void UartTxConfig::Init(uint8_t *p_data,uint16_t data_len, uint32_t timeout) {
        p_data_ = p_data;
        data_length_ = data_len;
        timeout_ = timeout;
    }


    HAL_StatusTypeDef Uart::InitAndStart() {
        // HAL_UART_Init(p_handle_);
        // __HAL_UART_CLEAR_IDLEFLAG(p_handle_);
        // __HAL_UART_ENABLE_IT(p_handle_, UART_IT_IDLE);
        //
        // if (p_handle_->RxState == HAL_UART_STATE_READY)
        // {
        //
        //     __HAL_LOCK(p_handle_);
        //     p_handle_->ReceptionType = HAL_UART_RECEPTION_STANDARD;
        //     if (!(IS_LPUART_INSTANCE(p_handle_->Instance)))
        //     {
        //         if (READ_BIT(p_handle_->Instance->CR2, USART_CR2_RTOEN) != 0U)
        //         {
        //             ATOMIC_SET_BIT(p_handle_->Instance->CR1, USART_CR1_RTOIE);
        //         }
        //     }
        //     init_flag_ = true;
            return (HAL_UART_Receive_IT(p_handle_, rx_data_, 128));
            // return HAL_OK;
        // }
        // else
        // {
        //     return HAL_BUSY;
        // }
    }

    void Uart::Send(UartTxConfig *p_config) {

        if (HAL_UART_Transmit(p_handle_, p_config->p_data_, p_config->data_length_, p_config->timeout_) != HAL_OK)
        {
            ErrorHandler();
        }
    }

    void Uart::ErrorHandler() {
        while(true){

        }
    }

}