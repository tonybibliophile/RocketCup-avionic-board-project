#ifndef CORE_INC_EKF_H_
#define CORE_INC_EKF_H_

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 陀螺 1000Hz（TIM7）/ 加速度 400Hz（TIM6）雙速率取樣：EKF_Task 逐樣本迴圈以陀螺
 * 速率跑（姿態積分 1000Hz），加速度用零階保持(ZOH)標 has_accel，Predict(位置/共
 * 變異數)僅在 has_accel=1 時以累積 dt 執行(400Hz)，見 ekf.c EKF_Task。 */
#define EKF_GYRO_RATE_HZ    1000U
#define EKF_ACCEL_RATE_HZ   400U
#define EKF_GYRO_PER_FRAME  (EKF_GYRO_RATE_HZ  * 10U / 1000U)   /* 每 10ms frame：10 顆陀螺樣本 */
#define EKF_ACC_PER_FRAME   (EKF_ACCEL_RATE_HZ * 10U / 1000U)   /* 每 10ms frame：4 顆加速度樣本 */
#define EKF_FRAMES_PER_BUF  10U                                 /* 100ms 一批，緩衝延遲不變 */
#define EKF_BUFFER_SIZE     (EKF_GYRO_PER_FRAME * EKF_FRAMES_PER_BUF)  /* 100 */

/* BMP388 50Hz ODR + IIR coeff 3（1-pole EMA，α=1/4 → 群延遲 ≈ 3 個 ODR 樣本）：
 * 3 × 20ms + 半個轉換時間(18.9ms/2 ≈ 9.5ms) ≈ 70ms（datasheet 推估，須由硬體階躍
 * 測試——見發射檢核表——實測確認並視需要修正本值）。樣本數換算依 EKF_GYRO_RATE_HZ
 * （EKF_Task 逐樣本迴圈的實際呼叫率＝陀螺速率），兩者變動時自動連動。 */
#define EKF_BARO_GROUP_DELAY_MS      70U
#define EKF_BARO_GROUP_DELAY_SAMPLES ((EKF_BARO_GROUP_DELAY_MS * EKF_GYRO_RATE_HZ) / 1000U)
#define EKF_Z_HISTORY_LEN            (EKF_BARO_GROUP_DELAY_SAMPLES + 12U)  /* 樣本數 + 餘裕 */

/* 靜止偵測窗（原 main.c/ekf.c 假設 1000 樣本＝1.0 秒，需與實際陀螺速率連動）。 */
#define EKF_REST_SAMPLES    EKF_GYRO_RATE_HZ

/* 靜態校準所需陀螺樣本數（=EKF_calib_samples 門檻，1000Hz 下 5000 樣本＝5.0 秒）。
 * 加速度只在 has_accel=1（400Hz ZOH 新樣本）時累加，獨立計數 EKF_accel_calib_n
 * （見 ekf.c），約 ~2000 顆 accel 樣本参与平均——維持舊值 3000 會讓 accel 偏差雜訊
 * 比 400Hz 時代（3000 樣本/7.5秒）上升 ~1.58×，故拉長至 5000 樣本(5.0秒)彌補。 */
#define EKF_CALIB_SAMPLES   5000U

// Memory partition macro for CCM RAM allocation
#define CCMRAM __attribute__((section(".ccmram")))

// Structure for a single 1000 Hz synchronized IMU + Baro sample.
// 加速度為零階保持(ZOH)：僅 has_accel=1 時是新樣本，其餘沿用上次值供 Mahony 姿態
// 融合的重力向量參考用（ZOH 對重力向量而言足夠，其變化時間常數遠慢於 2.5ms 保持期）。
typedef struct {
    float ax, ay, az;         // Linear acceleration in body frame (m/s^2), ZOH between has_accel samples
    float gx, gy, gz;         // Angular velocity in body frame (rad/s), every sample @1000Hz
    uint32_t timestamp_cyc;   // Raw DWT->CYCCNT at sample capture — wrap-safe delta in cycle domain
    uint8_t has_accel;        // 1 if this is a fresh accelerometer sample (drives EKF_Predict @400Hz)
    uint8_t has_baro;         // 1 if barometer sample is present, 0 otherwise
    uint16_t _rsv;
    float baro_alt;           // Barometer altitude in meters
} EKF_Sample_t;

// Double-buffered container representing a 100ms EKF cycle
typedef struct {
    EKF_Sample_t samples[EKF_BUFFER_SIZE];
} EKF_Buffer_t;

// State structure returned by the EKF
typedef struct {
    float pos_x, pos_y, pos_z;  // 3D Position in ENU (meters)
    float vel_x, vel_y, vel_z;  // 3D Velocity in ENU (m/s)
    float q[4];                 // 3D Attitude quaternion [qw, qx, qy, qz]
    float accel_bias[3];        // Estimated accelerometer biases (m/s^2)
} EKF_State_t;

// EKF thread attributes and static control definitions
extern const osThreadAttr_t EKF_Task_attributes;
extern osMessageQueueId_t xEKFQueue;

// Public EKF Status Flags for FSM Synchronization
extern uint8_t EKF_calibrated;
extern uint8_t EKF_in_flight;
extern uint8_t g_mag_yaw_lock;

// Public functions
void DWT_Init(void);
uint32_t DWT_GetMicroseconds(void);

void EKF_Task(void *argument);
void EKF_Init(void);
void EKF_ResetOrientation(void);
void EKF_Predict(float ax, float ay, float az, float dt);
void EKF_UpdateBaro(float baro_alt);
void EKF_UpdateBaroDelayed(float baro_alt, float z_pred);
void EKF_AttitudeUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);

/* GPS 水平位置注入（由 defaultTask 呼叫，~1Hz）。
 * 校準完成後第一次有效定位鎖定發射台原點 (lat0/lon0)，之後以等距投影換算
 * ENU 水平位移並排程一次 (E/N) 位置量測更新，於 EKF_Task 內套用。
 * 多次提交僅保留最新一筆；衛星數越少量測噪聲 R 越大（越不信任）。 */
void EKF_SubmitGPS(int32_t lat_1e6, int32_t lon_1e6, uint8_t satellites, uint32_t hacc_mm);

/* 地磁計航向 (yaw) 注入（由 defaultTask 呼叫，~10Hz）。
 * mx,my,mz 為 IMU body frame 下的磁場向量（任意單位，僅取方向）。
 * 僅於發射台階段（校準後、未升空）做傾斜補償後的 yaw 互補修正，
 * 用以鎖定絕對航向並抑制重力回授無法觀測的陀螺 yaw 漂移；
 * 升空後不使用（與既有設計一致：飛行中姿態為純陀螺遞推）。
 * 注意：mx,my,mz 軸向須與 IMU body frame 對齊；若 bench 測試發現航向往反向
 * 收斂，於呼叫端對相應軸取負號即可。 */
void EKF_SubmitMag(float mx, float my, float mz);

float EKF_GetCPUUsage(void);
EKF_State_t EKF_GetState(void);

/* P0-C：EKF 防護（ekf_guard.h）健康介面。
 * EKF_GetHealthBits()==0 且 (now − EKF_GetLastUpdateTick()) ≤ 300ms 才視為 healthy；
 * unhealthy 時 FSM 切換 raw-baro 降級開傘鏈（fsm.c），遙測置 TELEM_FLAG_EKF_UNHEALTHY。 */
#include "ekf_guard.h"
uint8_t  EKF_GetHealthBits(void);
uint32_t EKF_GetLastUpdateTick(void);

void EKF_SaveCalibrationToFlash(void);

/* P0-x：校準政策 —— 上電必重校。
 * EKF_LoadCalibrationFromFlash()：讀 Flash → 套 mag 硬鐵偏移 + 把加計/陀螺偏置
 *   與 baro 基準存為「比對參考值」；不再直接套用、不設 EKF_calibrated。
 * EKF_ApplyFlashCalibration()：把參考值真正套用並設 EKF_calibrated=1；僅供
 *   FEATURE_HOTSTART 空中恢復路徑（空中無法靜置 3 秒重校）。 */
void EKF_LoadCalibrationFromFlash(void);
void EKF_ApplyFlashCalibration(void);

void EKF_HotRestartRestore(float last_altitude, float est_vel_z, const float *last_q);
void EKF_ResetCalibration(void);
void EKF_SaveMagCalibration(float cx, float cy, float cz);

#endif /* CORE_INC_EKF_H_ */
