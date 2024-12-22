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
    speed *= static_cast<int>(direction_);
    speed_ = speed;
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
    const auto duty = (fabs(speed_) / 5000.F);
    speed_port_.SetDuty(duty);
    speed_port_.Output();
    return periph::periphState::kOK;
}




