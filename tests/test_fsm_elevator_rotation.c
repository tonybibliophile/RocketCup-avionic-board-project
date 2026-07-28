/*
 * test_fsm_elevator_rotation.c
 * 電梯場景下姿態融合垂直通道濾波器 (Attitude-Fused VF) 旋轉抗擾性測試
 *
 * 比較：
 * 1. 靜止平放 (Flat / Stationary Orientation)
 * 2. 旋轉航電 (Rotating Avionics - 手持翻轉/轉向 0°~45°)
 *
 * 驗證在電梯上升 (1.5 m/s, ~30m 頂樓) 過程中：
 * - 姿態正交投影是否成功將箭體 3 軸 IMU 加速度轉回世界座標 Z 軸。
 * - 高度估算 h_est 是否平滑追蹤真實電梯高度，不受板卡翻轉干擾。
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "vertical_filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 簡單高斯隨機數生成
static float rand_normal(float mean, float stddev) {
    if (stddev <= 0.0f) return mean;
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    return mean + z0 * stddev;
}

// 模擬電梯高度與速度軌跡 (全程 30 秒)
void get_elevator_truth(float t, float *h_out, float *v_out, float *a_out) {
    // 0~2s: 地面靜止 (0m)
    if (t < 2.0f) {
        *h_out = 0.0f;
        *v_out = 0.0f;
        *a_out = 0.0f;
    }
    // 2~4s: 勻加速至 1.5 m/s (a = 0.75 m/s^2)
    else if (t < 4.0f) {
        float dt = t - 2.0f;
        *a_out = 0.75f;
        *v_out = 0.75f * dt;
        *h_out = 0.5f * 0.75f * dt * dt;
    }
    // 4~22s: 勻速上升 (1.5 m/s, 上升 27m)
    else if (t < 22.0f) {
        float dt = t - 4.0f;
        *a_out = 0.0f;
        *v_out = 1.5f;
        *h_out = 1.5f + 1.5f * dt;
    }
    // 22~24s: 勻減速停止 (a = -0.75 m/s^2)
    else if (t < 24.0f) {
        float dt = t - 22.0f;
        *a_out = -0.75f;
        *v_out = 1.5f - 0.75f * dt;
        *h_out = 28.5f + 1.5f * dt - 0.5f * 0.75f * dt * dt;
    }
    // 24~30s: 頂樓靜止 (30m)
    else {
        *h_out = 30.0f;
        *v_out = 0.0f;
        *a_out = 0.0f;
    }
}

int main(void) {
    printf("===============================================================\n");
    printf("  電梯場景姿態融合垂直通道 (Attitude-Fused VF) 旋轉模擬測試\n");
    printf("===============================================================\n\n");

    srand(42);

    float dt_s = 0.010f; // 100Hz 飛控週期
    float total_time_s = 30.0f;
    int steps = (int)(total_time_s / dt_s);

    // 測試 Case 1: 平放靜止 (Flat, pitch=0, roll=0)
    // 測試 Case 2: 加入旋轉 (Rotating Board, 0.5Hz 翻轉, 最大 45 度)
    for (int is_rotating = 0; is_rotating <= 1; is_rotating++) {
        VFilter_t vf;
        vf_init(&vf);

        printf("---------------------------------------------------------------\n");
        if (is_rotating) {
            printf("【測試 2：加入旋轉航電 (Rotating Avionics - Pitch/Roll ±45° 旋轉)】\n");
        } else {
            printf("【測試 1：平放靜止 (Flat Stationary Orientation - 0° 傾角)】\n");
        }
        printf("---------------------------------------------------------------\n");

        float max_h_err = 0.0f;
        float max_v_err = 0.0f;
        float h_final = 0.0f;

        for (int i = 0; i <= steps; i++) {
            float t = (float)i * dt_s;
            float h_true, v_true, a_true;
            get_elevator_truth(t, &h_true, &v_true, &a_true);

            // 氣壓計雜訊 (電梯氣流/風壓噪聲 sigma = 0.3m)
            float baro_meas = rand_normal(h_true, 0.3f);

            // 姿態角度 (Pitch / Roll)
            float pitch = 0.0f;
            float roll  = 0.0f;
            if (is_rotating) {
                // 模擬航電板被翻轉：Pitch 與 Roll 以 0.5Hz / 0.3Hz 正弦旋轉，幅值達 45°
                pitch = 45.0f * (float)M_PI / 180.0f * sinf(2.0f * M_PI * 0.5f * t);
                roll  = 30.0f * (float)M_PI / 180.0f * cosf(2.0f * M_PI * 0.3f * t);
            }

            // 由歐拉角轉為四元數 q
            float cy = cosf(0.0f * 0.5f);
            float sy = sinf(0.0f * 0.5f);
            float cp = cosf(pitch * 0.5f);
            float sp = sinf(pitch * 0.5f);
            float cr = cosf(roll * 0.5f);
            float sr = sinf(roll * 0.5f);

            float q0 = cr * cp * cy + sr * sp * sy;
            float q1 = sr * cp * cy - cr * sp * sy;
            float q2 = cr * sp * cy + sr * cp * sy;
            float q3 = cr * cp * sy - sr * sp * cy;

            // 體座標系中的重力與加速度向量
            // 世界座標系 Z 軸加速度 vector_world = [0, 0, a_true + 9.80665]
            // 反向正交轉換為體座標系 IMU 讀數 ax_body, ay_body, az_body
            float a_world_z = a_true + 9.80665f;
            
            // R_world_to_body 矩陣轉置
            float R31 = 2.0f * (q1 * q3 - q0 * q2);
            float R32 = 2.0f * (q2 * q3 + q0 * q1);
            float R33 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

            float ax_body = R31 * a_world_z + rand_normal(0.0f, 0.05f);
            float ay_body = R32 * a_world_z + rand_normal(0.0f, 0.05f);
            float az_body = R33 * a_world_z + rand_normal(0.0f, 0.05f);

            // === 姿態融合垂直加速度投影 (Attitude-Fused Projection) ===
            float az_world_reconstructed = R31 * ax_body + R32 * ay_body + R33 * az_body;
            float a_vert_net = az_world_reconstructed - 9.80665f;

            // VF 濾波器預測與更新
            vf_predict(&vf, dt_s);
            vf_update_accel(&vf, a_vert_net);
            if (i % 5 == 0) { // 20Hz 氣壓計更新
                vf_update_baro(&vf, baro_meas);
            }

            float h_est = vf_h(&vf);
            float v_est = vf_v(&vf);

            float h_err = fabsf(h_est - h_true);
            float v_err = fabsf(v_est - v_true);
            if (h_err > max_h_err) max_h_err = h_err;
            if (v_err > max_v_err) max_v_err = v_err;

            if (i == steps) h_final = h_est;

            // 印出典型時間點 (t=0s, 10s, 20s, 30s)
            if (i == 0 || i == 1000 || i == 2000 || i == 3000) {
                printf("  t=%5.1fs | 真值高度: %5.2fm | VF估估高度: %5.2fm (誤差: %4.2fm) | raw_az: %5.2fm/s2 -> proj_az: %5.2fm/s2\n",
                       t, h_true, h_est, h_err, az_body, az_world_reconstructed);
            }
        }

        printf("  [評估結果]\n");
        printf("  - 頂樓最終估算高度: %.2f m (真值 30.00 m, 誤差 %.2f m)\n", h_final, fabsf(h_final - 30.0f));
        printf("  - 全程最大高度誤差: %.2f m\n", max_h_err);
        printf("  - 全程最大速度誤差: %.2f m/s\n", max_v_err);

        if (max_h_err < 1.5f && max_v_err < 0.5f) {
            printf("  - [PASS] 姿態融合 VF 成功抵抗姿態旋轉干擾，穩定追蹤電梯軌跡！\n\n");
        } else {
            printf("  - [FAIL] 誤差過大\n\n");
        }
    }

    return 0;
}
