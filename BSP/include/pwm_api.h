#pragma once

#include "tim.h"

namespace bsp{
    enum class PWM_State {
        kOff = 0,
        kOn = 1
    };

    class PWMGenerator {
    public:
        PWMGenerator(TIM_HandleTypeDef *htim, uint32_t ch)
                : htim_(htim), channel_(ch), duty_(0), state_(PWM_State::kOff) {
        }

        PWMGenerator() = default;

        ~PWMGenerator() = default;

        void Init();

        void Start();

        void Stop();

        void SetDuty(float duty);

        void SetPsc(int16_t psc);

        void Output();

    private:
        TIM_HandleTypeDef *htim_; // Timer handle
        uint32_t channel_;         // Channel
        float duty_;              // Duty cycle
        PWM_State state_;         // PWM state
    };

}