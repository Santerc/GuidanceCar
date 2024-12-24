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
#include "stm32h7xx_it.h"
#include "pid.h"

int temp = 0;


bsp::GPIO left_f(*GPIOG, GPIO_PIN_5);
    bsp::GPIO left_b(*GPIOG, GPIO_PIN_6);
    bsp::GPIO right_f(*GPIOG, GPIO_PIN_7);
    bsp::GPIO right_b(*GPIOG, GPIO_PIN_8);

    bsp::PWMGenerator left_en(&htim3, TIM_CHANNEL_1);
    bsp::PWMGenerator right_en(&htim3, TIM_CHANNEL_2);

    bsp::GPIO us_trigger(*GPIOG, GPIO_PIN_4);

    bsp::GPIO right_gpio(*GPIOG, GPIO_PIN_2);
    bsp::GPIO middle_gpio(*GPIOD, GPIO_PIN_15);
    bsp::GPIO left_gpio(*GPIOG, GPIO_PIN_3);

    bsp::GPIO road_left_gpio(*GPIOD, GPIO_PIN_14);
    bsp::GPIO road_right_gpio(*GPIOD, GPIO_PIN_11);

PID_PIDTypeDef pid = {0, 0, 0, 0, 0, 0};
PID_PIDParamTypeDef pparam = {0.3, 0, 0, 45, 100, 0.8};

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
            state_ = kTracking;
            last_state_ = kTracking;
            latest_side_road_ = RoadSide::kLeftRoad;
            waiting_start_time_ = 0;
        }
        ~Chassis() = default;

        const float speed[5][2] = {
            {0, 0},
            {800, 0},
            {-800, 0},
            {0, 800},
            {0, -800}
        };

        bool on_road_;
        bool last_on_road_;
        bool obstacle_;
        bool station_arrive_;
        bool in_wait_;//YBC
        float vx_ = 0;
        float vw_ = 0;
        TrackingState pos_;
        StateMachine state_;
        StateMachine last_state_;
        RoadSide latest_side_road_;
        uint32_t waiting_start_time_;



        void Init() {
            motor_left_.init();
            motor_right_.init();
            motor_left_.SetDir(periph::toyMotor::Direction::kBack);
            motor_right_.SetDir(periph::toyMotor::Direction::kFront);

        }
        uint32_t rest_time_ = 0;
        uint32_t rushing_start_time_ = 0;
        int level_ = 0;
        void StateChange() {
             switch (state_) {
                 case kRemote:
                     RemoteAction();
                     if (GetRxData().mode_ == kTrackingMode) {
                         last_state_ = kTracking;
                         state_ = kTracking;
                     }
                     break;
                 case kInit:
                     if (1) {
                         last_state_ = kInit;
                         state_ = kRemote;
                     }
                     break;
                 case kTracking:
                     TrackingAction();
                     // if (GetRxData().mode_ == kRemoteMode) {
                     //     last_state_ = kTracking;
                     //     state_ = kRemote;
                     //     break;
                     // }
                     if (obstacle_) {
                         last_state_ = kTracking;
                         CorrectDistance();
                         state_ = kLeave;
                     }
                 // else if (station_arrive_) {
                 //         last_state_ = kTracking;
                 //         waiting_start_time_ = HAL_GetTick();
                 //         state_ = kStop;
                 //         temp ++;
                 //     }
                 else if (!on_road_) {
                         last_state_ = kTracking;
                         if (level_ == 0) {
                             level_ ++;
                             rushing_start_time_ = HAL_GetTick();
                             //state_ = kWait;//YBC
                         }else {
                             state_ = kAngle;
                             level_ = 0;
                         }

                     }else if (in_wait_) {//YBC
                         state_ = kWait;//YBC
                     }//YBC
                     break;
                 case kAngle:

                     AngleAction();
                     // if (GetRxData().mode_ == kRemoteMode) {
                     //     last_state_ = kAngle;
                     //     state_ = kRemote;
                     //     break;
                     // }
                     if (obstacle_) {
                         last_state_ = kAngle;
                         CorrectDistance();
                         state_ = kLeave;
                     }else if (boson_middle_.GetColor()) {
                         last_state_ = kAngle;
                         state_ = kTracking;
                     }
                     break;
                 case kStop:
                     if ((HAL_GetTick() - waiting_start_time_) <= (GetRxData().stoptime_ * 1000)) {
                         rest_time_ = (HAL_GetTick() - waiting_start_time_);
                         SetSpeed(0, 0);
                         break;
                     }
                     station_arrive_ = false;
                     last_state_ = kStop;
                     state_ = kTracking;
                     break;
                 case kWait:
                 //     if ((HAL_GetTick() - rushing_start_time_) <= 2000 ){
                 //         break;
                 //     }
                 // last_state_ = kWait;
                 // state_ = kTracking;//YBC
                     if (boson_left_.GetColor() && boson_right_.GetColor() && boson_middle_.GetColor() && boson_left_road_.GetColor() && boson_right_road_.GetColor()) {//YBC
                         station_arrive_ = true;//YBC
                         in_wait_= false;//YBC
                         waiting_start_time_ = HAL_GetTick();
                         state_ = kStop;
                         // state_ =kGo;
                     }else {
                         state_ = kTracking;
                     }
                 break;
                 // case kGo:
                 //     SetSpeed(800, 0);
                 //    if (!((boson_left_.GetColor() || boson_right_.GetColor() || boson_middle_.GetColor()) && boson_left_road_.GetColor() && boson_right_road_.GetColor())) {
                 //        last_state_ = kGo;
                 //    }
                 case kLeave:
                     LeavingAction();
                     // if (GetRxData().mode_ == kRemoteMode) {
                     //     last_state_ = kLeave;
                     //     state_ = kRemote;
                     //     break;
                     // }
                     if (!obstacle_) {
                         last_state_ = kLeave;
                         state_ = kBack;
                     }
                     break;
                 case kBack:
                     BackAction();
                     // if (GetRxData().mode_ == kRemoteMode) {
                     //     last_state_ = kBack;
                     //     state_ = kRemote;
                     //     break;
                     // }
                     if (obstacle_) {
                         last_state_ = kBack;
                         CorrectDistance();
                         state_ = kLeave;
                     }else if (on_road_) {
                         last_state_ = kBack;
                         state_ = kTracking;
                     }
                     break;
                 default:
                     break;

             }
        }
        bool last_on_middle_ = false;
        bool on_millde_ = false;
        void DistanceMeasure() {
            distancer_.Measure();
            if (distancer_.GetDistance() < 0.2) {
                obstacle_ = true;
            }else {
                obstacle_ = false;
            }
            // obstacle_ = false;
        }

        void SideRoadDetect() {
            boson_left_road_.Detect();
            boson_right_road_.Detect();
            if (on_road_) {
                if (boson_left_road_.GetColor()) {
                    latest_side_road_ = kLeftRoad;
                }else if (boson_right_road_.GetColor()) {
                    latest_side_road_ = kRightRoad;
                }
            }
        }

        bool cross_sig = false;
        void TrackingDetect() {
            boson_middle_.Detect();
            boson_left_.Detect();
            boson_right_.Detect();
            if (boson_left_.GetColor() && boson_right_.GetColor() && boson_middle_.GetColor() && boson_left_road_.GetColor() && boson_right_road_.GetColor()) {
                //station_arrive_ = true;//YBC
                in_wait_= true;//YBC
            }
            if (boson_left_.GetColor() || boson_right_.GetColor() || boson_middle_.GetColor() || boson_left_road_.GetColor() || boson_right_road_.GetColor()) {
                if (boson_left_.GetColor() || boson_right_.GetColor() || boson_middle_.GetColor()) {
                    on_millde_ = true;
                    last_on_middle_ = on_millde_;
                }else {
                    if (last_on_middle_) {
                        on_millde_ = true;
                        last_on_middle_ = false;
                        return;
                    }
                }
                on_road_ = true;
                last_on_road_ = on_road_;
            }else {
                if (last_on_road_) {
                    on_road_ = true;
                    last_on_road_ = false;
                    return;

                }else {

                    on_road_ = false;
                    last_on_road_ = on_road_;
                }
            }
            if (boson_left_road_.GetColor() && boson_right_road_.GetColor()) {
                pos_ = kMiddle;
            }else if (boson_left_.GetColor()) {
                pos_ = kMidLeft;
            }else  if (boson_right_.GetColor()){
                pos_ = kMidRight;
            }else if (boson_middle_.GetColor()) {
                pos_ = kMiddle;
            }else if (boson_left_road_.GetColor()) {
                pos_ = kLeft;
            }else if (boson_right_road_.GetColor()) {
                pos_ = kRight;
            }
            // if (boson_middle_.GetColor()) {
            //     if (boson_left_.GetColor()) {
            //         if (boson_right_.GetColor()) {
            //             pos_ = TrackingState::kMiddle;
            //             if (!boson_left_road_.GetColor() && !boson_right_road_.GetColor()) {
            //                 if (state_ != kRemote) {
            //                     station_arrive_ = true;
            //                 }
            //             }
            //         }else {
            //             pos_ = TrackingState::kMidLeft;
            //         }
            //     }else if (boson_right_.GetColor()){
            //         pos_ = TrackingState::kMidRight;
            //     }else {
            //         pos_ = TrackingState::kMiddle;
            //     }
            // }else
            if (boson_left_.GetColor()) {
                pos_ = TrackingState::kLeft;
            }else if (boson_right_.GetColor()) {
                pos_ = TrackingState::kRight;
            }
        }

        void Detect() {
            // DistanceMeasure();
            SideRoadDetect();
            TrackingDetect();
        }

        void SetSpeed(float vx, float vw) {
            float gain = 1.F;
            // if (fabs(fabs(vx) - fabs(vw)) <= 500) {
            //     gain = 2.5F;
            // }else if (fabs(fabs(vx) - fabs(vw)) <= 600) {
            //     gain = 2.F;
            // }
            // if (fabs(motor_left_.GetSpeed() - (vx +vw)) >= 50 || fabs(motor_right_.GetSpeed() - (vx - vw)) >= 50) {
            //     gain = 2.F;
            // }
            vx_ = vx * gain;
            vw_ = vw * gain;
            PID_SetRef(&pid, vw / 6);
            PID_SetFdb(&pid, GetInsData()->space_omega_.yaw * 600.F);
            PID_Calc(&pid, &pparam);
            gain = PID_GetOutput(&pid);
            gain = 0;
            if (gain > 0) {
                    motor_left_.SetSpeed((vx + vw) * (1 + gain) * 1.2);
                    motor_right_.SetSpeed(1.0 * (vx + vw));
            }else {
                motor_left_.SetSpeed((vx + vw) * 1.2);
                motor_right_.SetSpeed(1.0 * (vx - vw) / (1 + gain));;
            }
            if (state_ == kRemote) {
                motor_left_.SetSpeed(1.2 * (vx + vw) * 1.0);
                motor_right_.SetSpeed(1.0 * (vx - vw));
            }
            // motor_left_.SetSpeed(vx + vw);
            // motor_right_.SetSpeed(vx - vw);
            // motor_left_.SetSpeed(1000);
            // motor_right_.SetSpeed(0);
        }

        void Move() {
            motor_left_.Output();
            motor_right_.Output();
        }

        void TrackingAction() {
            switch (pos_) {
                case kMiddle:
                    SetSpeed(500, 0);
                    break;
                case kLeft:
                    SetSpeed(300, 300);
                    break;
                case kRight:
                    SetSpeed(300, -300);
                    break;
                case kMidLeft:
                    SetSpeed(300, 300);
                    break;
                case kMidRight:
                    SetSpeed(300, -300);
                    break;
                default:
                    break;
            }
        }

        void AngleAction() {
            float dir = 1;
            dir = latest_side_road_ == kLeftRoad ? 1 : -1;
            SetSpeed(0, 600 * dir);
        }

        void LeavingAction() {
            SetSpeed(500, 600);
        }

        void BackAction() {
            SetSpeed(500, -600);
        }

        void CorrectDistance() {
            if (distancer_.GetDistance() < 0.1) {
                SetSpeed(-800, 0);
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
    return !(chassis.station_arrive_);
}

const uint8_t GetMoveState() {
    if (chassis.vw_ == 0 && chassis.vx_ == 0) {
        return kStopMove;
    }
    if (chassis.vw_ < 0) {
        return kLeftMove;
    }
    if (chassis.vw_ > 0) {
        return kRightMove;
    }
    if (chassis.vx_ > 0) {
        return kFrontMove;
    }
    return kBackMove;
}
const uint8_t GetState()
{
    return chassis.state_;
}

const uint8_t GetLastState()
{
    return chassis.last_state_;
}

const uint8_t Get_cnt_time() {
    return chassis.rest_time_/1000;
}
void Chassis_Task(void *parameter) {
    chassis.Init();
    while (true) {
        // chassis.Detect();
        chassis.StateChange();
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
    xTaskCreate(Detect_Task, "DetectTask", 512, NULL, 7, NULL);
}



