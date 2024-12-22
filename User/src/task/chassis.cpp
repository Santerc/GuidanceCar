//
// Created by 59794 on 2024/12/15.
//
#include "cmsis_os.h"
#include "task/chassis.h"
#include "task/com.h"
#include "task/ins.h"

#include "periph/toymotor.h"
#include "periph/ultrasound.h"
#include "periph/boson.h"
#include "fsm.hpp"

bsp::GPIO left_f(*GPIOG, GPIO_PIN_5);
    bsp::GPIO left_b(*GPIOG, GPIO_PIN_6);
    bsp::GPIO right_f(*GPIOG, GPIO_PIN_7);
    bsp::GPIO right_b(*GPIOG, GPIO_PIN_8);

    bsp::PWMGenerator left_en(&htim3, TIM_CHANNEL_1);
    bsp::PWMGenerator right_en(&htim3, TIM_CHANNEL_2);

    bsp::GPIO us_trigger(*GPIOG, GPIO_PIN_4);

    bsp::GPIO right_gpio(*GPIOG, GPIO_PIN_3);
    bsp::GPIO middle_gpio(*GPIOG, GPIO_PIN_2);
    bsp::GPIO left_gpio(*GPIOD, GPIO_PIN_15);
    bsp::GPIO road_left_gpio(*GPIOD, GPIO_PIN_14);
    bsp::GPIO road_right_gpio(*GPIOD, GPIO_PIN_11);

/*
 *
 */

    class Chassis {
    public:
        Chassis() :
        motor_left_(left_en, left_f, left_b),
        motor_right_(right_en, right_f, right_b),
        distancer_(us_trigger, htim1),
        boson_left_road_(road_left_gpio),
        boson_right_road_(road_right_gpio),
        boson_middle_(middle_gpio),
        boson_left_(left_gpio),
        boson_right_(right_gpio)
        {
            pos_ = TrackingState::kMiddle;
            on_road_ = true;
            obstacle_ = false;
            station_arrive_ = false;
            state_ = StateMachine::kTracking;
            latest_side_road_ = RoadSide::kLeftRoad;
            waiting_start_time_ = 0;
            fsmInit();
            chassis_sm_.set(kInit);
        }
        ~Chassis() = default;

        const float speed[5][2] = {
            {1000, 0},
            {-1000, 0},
            {0, 0},
            {0, 1000},
            {0, -1000}
        };

        bool on_road_;
        bool obstacle_;
        bool station_arrive_;
        float vx_ = 0;
        float vw_ = 0;
        TrackingState pos_;
        StateMachine state_;
        StateMachine last_state_;
        RoadSide latest_side_road_;
        fsm::stack chassis_sm_;
        uint32_t waiting_start_time_;



        void Init() {
            motor_left_.init();
            motor_right_.init();
            motor_left_.SetDir(periph::toyMotor::Direction::kBack);
            motor_right_.SetDir(periph::toyMotor::Direction::kFront);

        }

        void fsmInit() {
            chassis_sm_.on(kInit, 'tick') = [&](const fsm::args &args) {
                if (1) {
                    chassis_sm_.push(kRemote);
                }
            };

            chassis_sm_.on(kTracking, 'tick') = [&](const fsm::args &args) {
                TrackingAction();
                if (GetRxData().mode_ == kRemoteMode) {
                    chassis_sm_.push(kRemote);
                    return;
                }
                if (obstacle_) {
                    chassis_sm_.push(kLeave);
                }else if (station_arrive_) {
                    chassis_sm_.push(kStop);
                }else if (!on_road_) {
                    chassis_sm_.push(kAngle);
                }
            };

            chassis_sm_.on(kStop, 'push') = [&](const fsm::args &args) {
                waiting_start_time_ = HAL_GetTick();
            };

            chassis_sm_.on(kStop, 'tick') = [&](const fsm::args &args) {
                while ((HAL_GetTick() - waiting_start_time_) >= GetRxData().timestamp * 1000) {
                    SetSpeed(0, 0);
                }
                station_arrive_ = false;
                chassis_sm_.push(kTracking);
            };

            chassis_sm_.on(kAngle, 'tick') = [&](const fsm::args &args) {
                AngleAction();
                if (GetRxData().mode_ == kRemoteMode) {
                    chassis_sm_.push(kRemote);
                    return;
                }
                if (obstacle_) {
                    chassis_sm_.push(kLeave);
                }else if (on_road_) {
                    chassis_sm_.push(kTracking);
                }
            };

            chassis_sm_.on(kLeave, 'tick') = [&](const fsm::args &args) {
                LeavingAction();
                if (GetRxData().mode_ == kRemoteMode) {
                    chassis_sm_.push(kRemote);
                    return;
                }
                if (!obstacle_) {
                    chassis_sm_.push(kBack);
                }
            };

            chassis_sm_.on(kBack, 'tick') = [&](const fsm::args &args) {
                BackAction();
                if (GetRxData().mode_ == kRemoteMode) {
                    chassis_sm_.push(kRemote);
                    return;
                }
                if (obstacle_) {
                    chassis_sm_.push(kLeave);
                }else if (on_road_) {
                    chassis_sm_.push(kTracking);
                }
            };

            chassis_sm_.on(kLeave, 'push') = [&](const fsm::args &args) {
                CorrectDistance();
            };

            chassis_sm_.on(kRemote, 'tick') = [&](const fsm::args &args) {
                RemoteAction();
                if (GetRxData().mode_ == kTrackingMode) {
                    chassis_sm_.push(kTracking);
                }

            };
        }

        void DistanceMeasure() {
            distancer_.Measure();
            // if (distancer_.GetDistance() < 0.2) {
            //     obstacle_ = true;
            // }else {
            //     obstacle_ = false;
            // }
            obstacle_ = false;
        }

        void SideRoadDetect() {
            boson_left_road_.Detect();
            boson_right_road_.Detect();
            if (boson_left_road_.GetColor()) {
                latest_side_road_ = kLeftRoad;
            }else if (boson_right_road_.GetColor()) {
                latest_side_road_ = kRightRoad;
            }
        }

        void TrackingDetect() {
            boson_middle_.Detect();
            boson_left_.Detect();
            boson_right_.Detect();
            if (boson_left_.GetColor() || boson_right_.GetColor() || boson_middle_.GetColor()) {
                on_road_ = true;
            }else {
                on_road_ = false;
            }
            if (boson_middle_.GetColor()) {
                if (boson_left_.GetColor()) {
                    if (boson_right_.GetColor()) {
                        pos_ = TrackingState::kMiddle;
                        if (!boson_left_road_.GetColor() && !boson_right_road_.GetColor()) {
                            station_arrive_ = true;
                        }
                    }else {
                        pos_ = TrackingState::kMidLeft;
                    }
                }else if (boson_right_.GetColor()){
                    pos_ = TrackingState::kMidRight;
                }else {
                    pos_ = TrackingState::kMiddle;
                }
            }else if (boson_left_.GetColor()) {
                pos_ = TrackingState::kLeft;
            }else if (boson_right_.GetColor()) {
                pos_ = TrackingState::kRight;
            }
        }

        void Detect() {
            DistanceMeasure();
            SideRoadDetect();
            TrackingDetect();
        }

        void SetSpeed(float vx, float vw) {
            vx_ = vx;
            vw_ = vw;
            motor_left_.SetSpeed(vx + vw);
            motor_right_.SetSpeed(vx - vw);
        }

        void Move() {
            motor_left_.Output();
            motor_right_.Output();
        }

        void TrackingAction() {
            switch (pos_) {
                case kMiddle:
                    SetSpeed(800, 0);
                    break;
                case kLeft:
                    SetSpeed(600, -200);
                    break;
                case kRight:
                    SetSpeed(600, 200);
                    break;
                case kMidLeft:
                    SetSpeed(800, -100);
                    break;
                case kMidRight:
                    SetSpeed(800, 100);
                    break;
                default:
                    break;
            }
        }

        void AngleAction() {
            float dir = 1;
            dir = latest_side_road_ == kLeftRoad ? 1 : -1;
            SetSpeed(0, -600 * dir);
        }

        void LeavingAction() {
            SetSpeed(1000, 300);
        }

        void BackAction() {
            SetSpeed(1000, -300);
        }

        void CorrectDistance() {
            while (distancer_.GetDistance() < 0.1) {
                SetSpeed(-1000, 0);
            }
        }

        void RemoteAction() {
            float buff = GetRxData().supercap_ ? 2 : 1;
            SetSpeed(speed[GetRxData().move_][0] * buff,
                speed[GetRxData().move_][1] * buff);
        }

    private:
        periph::toyMotor motor_left_;
        periph::toyMotor motor_right_;
        periph::Ultrasound distancer_;
        periph::boson boson_left_road_;
        periph::boson boson_right_road_;
        periph::boson boson_middle_;
        periph::boson boson_left_;
        periph::boson boson_right_;
    };

Chassis chassis;

const uint8_t GetStopFlag() {
    return chassis.station_arrive_;
}

const uint8_t GetMoveState() {
    if (chassis.vw_ == 0 && chassis.vx_ == 0) {
        return kStopMove;
    }
    if (GetInsData()->space_omega_.yaw > 0.1) {
        return kLeftMove;
    }
    if (GetInsData()->space_omega_.yaw < 0.1) {
        return kLeftMove;
    }
    if (GetInsData()->self_vel_.x > 0.1) {
        return kFrontMove;
    }
    if (GetInsData()->self_vel_.x < 0.1) {
        return kBackMove;
    }
    return kStopMove;
}
const uint8_t GetState()
{
    return chassis.state_;
}

const uint8_t GetLastState()
{
    return chassis.last_state_;
}
void Chassis_Task(void *parameter) {
    chassis.Init();
    while (true) {
        chassis.chassis_sm_.command('tick');
        chassis.Move();
        osDelay(1);
    }
}

void Detect_Task(void *parameter) {
    while (true) {
        chassis.Detect();
        osDelay(1);
    }
}

void ChassisTaskStart(void)
{
    xTaskCreate(Chassis_Task, "ChassisTask", 256, NULL, 7, NULL);
}

void DetectTaskStart(void)
{
    xTaskCreate(Detect_Task, "DetectTask", 256, NULL, 7, NULL);
}



