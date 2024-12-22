//
// Created by 18759 on 2024/11/26.
//

#ifndef UART_API_H
#define UART_API_H
#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "stm32h7xx_hal_uart.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "callback.h"

namespace bsp {
    struct UartTxConfig {
        uint8_t *p_data_{nullptr};
        uint16_t data_length_{0};
        uint32_t timeout_{0};

        [[maybe_unused]] explicit UartTxConfig(uint8_t *p_data, uint16_t data_len, uint32_t timeout) :
                p_data_(p_data), data_length_(data_len), timeout_(timeout) {};

        void Init(uint8_t *p_data, uint16_t data_len, uint32_t timeout);
    };

    class Uart final : public Callback<3,uint8_t*, uint16_t> {
    public:
        UART_HandleTypeDef *p_handle_;
        uint8_t rx_data_[128]{0};
        bool init_flag_{false};

        Uart() = delete;

        explicit Uart(UART_HandleTypeDef *huart) : p_handle_(huart) {};

        HAL_StatusTypeDef InitAndStart();

        void Send(UartTxConfig *p_config);

        void ErrorHandler();
    };
}


#endif
#endif //CHASSIS_UART_API_H
