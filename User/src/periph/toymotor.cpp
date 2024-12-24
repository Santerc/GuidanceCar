//
// Created by 59794 on 2024/12/15.
//
#include "periph/toymotor.h"

using namespace periph;

void toyMotor::init() {
    speed_port_.Init();
    speed_port_.Start();
}

void toyMotor::SetDir(Direction dir) {
    direction_ = dir;
}

void toyMotor::SetSpeed(float speed) {
    speed *= (float)direction_;
    // if (last_speed_ ==0 || (fabs(speed_ - last_speed_) >= 50)) {
    //     speed *= 1.25;
    // }
    last_speed_ = speed_;
    speed_ = speed;
}

float toyMotor::GetSpeed() {
    return speed_;
}

float lmt(float num, float max, float min) {
    if (num > max) {
        return max;
    }
    if (num < min) {
        return min;
    }
    return num;
}

periph::periphState toyMotor::Output() {
    if (speed_ == 0) {
        forward_gpio_.Set();
        backward_gpio_.Set();
    }else {
        if(speed_ > 0){
            forward_gpio_.Set();
            backward_gpio_.Reset();
        }else {
            forward_gpio_.Reset();
            backward_gpio_.Set();
        }
    }

    const auto duty = (lmt(fabs(speed_) / 5000.F, 0.8F, -0.8F));
    speed_port_.SetDuty(duty);
    speed_port_.Output();
    return periph::periphState::kOK;
}




