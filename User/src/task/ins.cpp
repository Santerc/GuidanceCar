//
// Created by 59794 on 2024/12/12.
//
#include <cmsis_os.h>

#include "task/ins.h"
#include "dwt_api.h"
#include "periph/bmi088.h"

#include <vector>
#include <cmath>

extern "C"{
#include "bsxlite_interface.h"
#include "arm_math.h"
}

bsp::SPI imu_spi(hspi1);
bsp::GPIO accel_cs(*GPIOF, GPIO_PIN_3);
bsp::GPIO gyro_cs(*GPIOF, GPIO_PIN_4);
imu::Bmi088 ins(imu_spi, accel_cs, gyro_cs);

bsxlite_instance_t ins_instance;
vector_3d_t accel_data = {0, 0, 0};
vector_3d_t gyro_data = {0, 0, 0};

bsxlite_out_t bsx_out;
insData_t ins_data;

uint32_t last_tick = 0;

uint32_t prev_time = 0;

void eulerToRotationMatrix(float32_t roll, float32_t pitch, float32_t yaw, float32_t* R) {
    float32_t cr = cosf(roll), sr = sinf(roll);
    float32_t cp = cosf(pitch), sp = sinf(pitch);
    float32_t cy = cosf(yaw), sy = sinf(yaw);

    R[0] = cy * cp;
    R[1] = cy * sp * sr - sy * cr;
    R[2] = cy * sp * cr + sy * sr;

    R[3] = sy * cp;
    R[4] = sy * sp * sr + cy * cr;
    R[5] = sy * sp * cr - cy * sr;

    R[6] = -sp;
    R[7] = cp * sr;
    R[8] = cp * cr;
}


//荣格库塔积分
void rk4Integrate(float32_t* velocity, float32_t* acc_global) {
    float32_t k1_vel[3], k2_vel[3], k3_vel[3], k4_vel[3];
    float32_t k1_pos[3], k2_pos[3], k3_pos[3], k4_pos[3];

    float32_t position[3] = {0, 0, 0};
    float32_t time_step = ((HAL_GetTick() - last_tick) / 1000.F);
    for (int i = 0; i < 3; ++i) {
        k1_vel[i] = acc_global[i];
        k1_pos[i] = velocity[i];

        k2_vel[i] = acc_global[i];
        k2_pos[i] = velocity[i] + 0.5f * time_step * k1_vel[i];

        k3_vel[i] = acc_global[i];
        k3_pos[i] = velocity[i] + 0.5f * time_step * k2_vel[i];

        k4_vel[i] = acc_global[i];
        k4_pos[i] = velocity[i] + time_step * k3_vel[i];

        velocity[i] += time_step / 6.0f * (k1_vel[i] + 2 * k2_vel[i] + 2 * k3_vel[i] + k4_vel[i]);
        position[i] += time_step / 6.0f * (k1_pos[i] + 2 * k2_pos[i] + 2 * k3_pos[i] + k4_pos[i]);
    }
    last_tick = HAL_GetTick();
    ins_data.space_vel_.x = velocity[0];
    ins_data.space_vel_.y = velocity[1];
    ins_data.space_vel_.z = velocity[2];
    ins_data.space_pos_.x = position[0];
    ins_data.space_pos_.y = position[1];
    ins_data.space_pos_.z = position[2];
}

void updateEuler() {
    ins_data.space_att_.yaw = bsx_out.orientation.yaw;
    ins_data.space_att_.pitch = bsx_out.orientation.pitch;
    ins_data.space_att_.roll = bsx_out.orientation.roll;
}

void Navigation() {
        float32_t ax = ins.accel_data_.x;
        float32_t ay = ins.accel_data_.y;
        float32_t az = ins.accel_data_.z;

        updateEuler();

        float32_t R[9];
        eulerToRotationMatrix(ins_data.space_att_.yaw, ins_data.space_att_.pitch, ins_data.space_att_.roll, R);

        float32_t acc_local[3] = {ax, ay, az};
        float32_t acc_global[3];
        // arm_matrix_instance_f32 R_mat, acc_local_mat, acc_global_mat;
        //
        // arm_mat_init_f32(&R_mat, 3, 3, R);
        // arm_mat_init_f32(&acc_local_mat, 3, 1, acc_local);
        // arm_mat_init_f32(&acc_global_mat, 3, 1, acc_global);
        //
        // arm_mat_mult_f32(&R_mat, &acc_local_mat, &acc_global_mat);
        for (int row = 0; row < 3; ++row) {
            acc_global[row] = R[row * 3 + 0] * acc_local[0] +
                              R[row * 3 + 1] * acc_local[1] +
                              R[row * 3 + 2] * acc_local[2];
        }

        acc_global[2] += 9.81F;
        float32_t vel[3] = {ins_data.space_vel_.x, ins_data.space_vel_.y, ins_data.space_vel_.z};
        rk4Integrate(vel, acc_global);
}

void Inti() {
    float DeltaTime = (HAL_GetTick() - prev_time) / 1000.F;
    ins_data.self_vel_.x += ins.accel_data_.x * DeltaTime;
    ins_data.self_vel_.y += ins.accel_data_.y * DeltaTime;
    ins_data.self_vel_.z += ins.accel_data_.z * DeltaTime;
}


static void INS_Task(void* parameter)
{
    static bool init_flag{false};
    ins_data = {
        0,0,0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0};
    if (!init_flag) {
        bsp::DWT_Init(170);
        ins.Init();
        while(ins.error_ != imu::Bmi088::Error::NO_ERROR){
            ins.Init();
        }
        init_flag  = true;
    }
    bsxlite_init(&ins_instance);
    while (true)
    {
        ins.Decode();

        accel_data.x = ins.accel_data_.x;
        accel_data.y = ins.accel_data_.y;
        accel_data.z = ins.accel_data_.z;
        gyro_data.x = ins.gyro_data_.pitch;
        gyro_data.y = ins.gyro_data_.row;
        gyro_data.z = ins.gyro_data_.yaw;

        bsxlite_do_step(&ins_instance, 1000*HAL_GetTick(), &accel_data, &gyro_data, &bsx_out);

        // ins_data = {
        //     0,
        //     0,
        //     0,
        //     bsx_out.orientation.yaw,
        //     bsx_out.orientation.pitch,
        //     bsx_out.orientation.roll};
        // Navigation();

        ins_data.space_omega_ = {
            ins.gyro_data_.yaw,
            ins.gyro_data_.pitch,
            ins.gyro_data_.row
        };
        ins_data.space_att_ = {
            bsx_out.orientation.yaw,
            bsx_out.orientation.pitch,
            bsx_out.orientation.roll
        };

        osDelay(10);
    }
}

const insData_t* GetInsData() {
    return &ins_data;
}

void InsTaskStart(void)
{
    xTaskCreate(INS_Task, "LEDTask", 256, NULL, 7, NULL);
}

