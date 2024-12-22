// Created by 59794 on 2024/12/15

#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


#include "gpio_api.h"
#include "pwm_api.h"
#include "stm32h7xx_hal_tim.h"
#include "periph_state.h"

namespace periph {

class Ultrasound {
public:
    Ultrasound(bsp::GPIO trigger, TIM_HandleTypeDef timer)
        : trigger_gpio_(trigger), timer_(timer), distance_(0.0f) {}

    ~Ultrasound() = default;

    float GetDistance() const;
    periph::periphState Measure();

private:
    void TriggerSignal();
    bool WaitForCaptureFlags();
    void CalculateDistance();

    bsp::GPIO trigger_gpio_;
    TIM_HandleTypeDef timer_;
    float distance_;

    static constexpr uint32_t kTimeoutMs = 50;
};

} // namespace periph

#endif

#endif // ULTRASOUND_H
