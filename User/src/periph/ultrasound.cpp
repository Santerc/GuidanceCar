//
// Created by 59794 on 2024/12/15.
//
#include "periph/ultrasound.h"
#include "cmsis_os.h"

using namespace periph;

bool dbg1, dbg2;


// Implementation
void Ultrasound::TriggerSignal() {
    // HAL_GPIO_WritePin(&trigger_gpio_.handle_, trigger_gpio_.pin_, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
    trigger_gpio_.Set();
    osDelay(1);
    trigger_gpio_.Reset();
    // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(&trigger_gpio_.handle_, trigger_gpio_.pin_, GPIO_PIN_RESET);
}

bool Ultrasound::WaitForCaptureFlags() {
    uint32_t expire_time = HAL_GetTick() + kTimeoutMs;

    while (HAL_GetTick() < expire_time) {
        bool cc1_flag = __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1);
        bool cc2_flag = __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC2);
        dbg1 = cc1_flag;
        dbg2 = cc2_flag;

        if (cc1_flag && cc2_flag) {
            return true;
        }
    }
    return false;
}

void Ultrasound::CalculateDistance() {
    uint32_t ccr1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    uint32_t ccr2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);

    float pulse_width = (ccr2 - ccr1) * 1e-6f;
    distance_ = 340.0f * pulse_width / 2.0f;
}

periph::periphState Ultrasound::Measure() {
    // Reset timer and flags
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC2);

    // Start input capture
    HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_4);

    // Trigger the ultrasound sensor
    TriggerSignal();

    // Wait for capture flags
    bool success = WaitForCaptureFlags();

    // Stop input capture
    HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_4);

    if (success) {
        CalculateDistance();
        return periph::periphState::kOK;
    } else {
        return periph::periphState::kFail;
    }
}

float Ultrasound::GetDistance() const {
    return distance_;
}