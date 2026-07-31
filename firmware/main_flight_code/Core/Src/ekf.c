#include "ekf.h"
#include "board_config.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "w25qxx.h"
#include "mmc5983.h"
#include "attitude_math.h"   /* TRIAD 與四元數工具（與 host 測試共用同一份） */
#include "ekf_guard.h"       /* P0-C：創新值閘控 / 協方差夾限 / 狀態界限 / NaN 掃描 */

/* current_fsm_state（main.c 全域，由 FSM_Update 鏡射 g_fsm_ctx.state）。
 * 原僅在 EKF_AttitudeUpdate 內以區域 extern 宣告；靜止偵測器歸零防護（下方）
 * 亦需要，故提升到檔案頂部宣告一次，兩處共用。 */
extern FlightState_t current_fsm_state;

// -------------------------------------------------------------------------
// Global EKF variables residing in CCM RAM
// -------------------------------------------------------------------------

// EKF state vector
// State elements:
// x[0] = Position East (m)
// x[1] = Position North (m)
// x[2] = Position Up (m) / Altitude
// x[3] = Velocity East (m/s)
// x[4] = Velocity North (m/s)
// x[5] = Velocity Up (m/s)
static float EKF_x[6] CCMRAM;

// Covariance matrix (6x6)
static float EKF_P[6][6] CCMRAM;

// Attitude quaternion (qw, qx, qy, qz)
static float EKF_q[4] CCMRAM;

// Dynamic stationary calibration variables
uint8_t EKF_calibrated CCMRAM = 0;
static uint32_t EKF_calib_samples CCMRAM = 0;      // 陀螺樣本計數（每筆 1000Hz 樣本遞增）
static uint32_t EKF_accel_calib_n CCMRAM = 0;       // 加速度樣本計數（僅 has_accel=1 時遞增，避免 ZOH 重複值灌入平均）
static float EKF_accel_bias[3] CCMRAM = {0.0f, 0.0f, 0.0f};
static float EKF_accel_sum[3] CCMRAM = {0.0f, 0.0f, 0.0f};
static float EKF_gyro_bias[3] CCMRAM = {0.0f, 0.0f, 0.0f};
static float EKF_gyro_sum[3] CCMRAM = {0.0f, 0.0f, 0.0f};

// Barometer launchpad reference altitude
static float EKF_baro_launchpad CCMRAM = 0.0f;
static float EKF_baro_sum CCMRAM = 0.0f;
static uint32_t EKF_baro_samples CCMRAM = 0;

/* P0-x：校準政策改為「上電必重校，Flash 僅作比對參考」——
 * EKF_LoadCalibrationFromFlash() 只把 Flash 內容存入下列參考值，不再直接覆寫
 * EKF_accel_bias/EKF_gyro_bias/EKF_baro_launchpad、也不設 EKF_calibrated=1。
 * 現場 3 秒靜態校準完成時會與此比對偵測溫漂；FEATURE_HOTSTART 空中恢復路徑
 * （地面無法二次靜置校準）則呼叫 EKF_ApplyFlashCalibration() 真正套用。 */
static float   EKF_flash_calib_accel_bias[3] CCMRAM = {0.0f, 0.0f, 0.0f};
static float   EKF_flash_calib_gyro_bias[3]  CCMRAM = {0.0f, 0.0f, 0.0f};
static float   EKF_flash_calib_baro_launchpad CCMRAM = 0.0f;
static uint8_t EKF_flash_calib_valid CCMRAM = 0;

#define EKF_CALIB_DRIFT_GYRO_MAX  0.02f   // 陀螺偏置與 Flash 參考差異上限 (rad/s)，超過視為溫漂
#define EKF_CALIB_DRIFT_ACCEL_MAX 0.3f    // 加速度計偏置與 Flash 參考差異上限 (m/s^2)

// Flight state transition flag, launch counter, and rest counter
uint8_t EKF_in_flight CCMRAM = 0;
uint8_t g_mag_yaw_lock = 1;
static uint32_t EKF_launch_counter CCMRAM = 0;
static uint32_t EKF_rest_counter CCMRAM = 0;

// --- GPS horizontal-position fusion (East/North) ---
// Written by defaultTask via EKF_SubmitGPS(), consumed once per buffer in EKF_Task.
static volatile uint8_t EKF_gps_pending CCMRAM = 0;     // 1 = a measurement awaits application
static uint8_t  EKF_gps_origin_set CCMRAM = 0;          // 1 = launchpad lat/lon origin captured
static int32_t  EKF_gps_lat0_1e6 CCMRAM = 0;            // launchpad origin latitude  (deg x1e6)
static int32_t  EKF_gps_lon0_1e6 CCMRAM = 0;            // launchpad origin longitude (deg x1e6)
static float    EKF_gps_coslat0 CCMRAM = 1.0f;          // cos(lat0): East-scale for equirectangular proj
static float    EKF_gps_meas_E CCMRAM = 0.0f;           // pending measured East  (m, rel. launchpad)
static float    EKF_gps_meas_N CCMRAM = 0.0f;           // pending measured North (m, rel. launchpad)
static float    EKF_gps_R CCMRAM = 25.0f;               // pending measurement variance (m^2)

// Launchpad origin is locked from the mean of several fixes rather than a single
// sample: one lucky/unlucky fix under a hard hAcc threshold still carries its full
// noise into a permanent, never-revisited reference; averaging N fixes divides
// that noise by sqrt(N) instead.
#define EKF_GPS_ORIGIN_AVG_N        25U      // ~1s of fixes @ GPS's configured 25Hz rate
#define EKF_GPS_ORIGIN_MAX_HACC_MM  50000U   // loose sanity ceiling (50m): reject only clearly-broken
                                              // outlier fixes (e.g. multipath) from entering the average;
                                              // the averaging itself is what brings the noise down, this
                                              // just stops one bad sample from skewing a small-N mean
static int64_t  EKF_gps_origin_lat_sum CCMRAM = 0;      // accumulating deg x1e6 for origin averaging
static int64_t  EKF_gps_origin_lon_sum CCMRAM = 0;
static uint16_t EKF_gps_origin_sample_count CCMRAM = 0;

// --- Magnetometer heading (yaw) fusion ---
// Written by defaultTask via EKF_SubmitMag(), consumed once per buffer in EKF_Task.
static volatile uint8_t EKF_mag_pending CCMRAM = 0;     // 1 = a mag vector awaits application
static float    EKF_mag_x CCMRAM = 0.0f;                // body-frame magnetic field X
static float    EKF_mag_y CCMRAM = 0.0f;                // body-frame magnetic field Y
static float    EKF_mag_z CCMRAM = 0.0f;                // body-frame magnetic field Z

// Process noise covariances (diagonals)
static const float Q_pos = 0.005f;  // Position process noise variance
static const float Q_vel = 0.1f;    // Velocity process noise variance

// Measurement noise covariance for Barometer altitude
static const float R_baro = 0.04f;  // ~0.2m altitude measurement stddev (2026-07-13 static bench measurement, was 0.36f/0.6m)

// Gravity constant in ENU
static const float GRAVITY = 9.80665f;

/* P0-x：靜止偵測器歸零防護 —— 僅在「地面狀態」且「baro 相對高度貼近地面」
 * 皆成立時，才允許 EKF_rest_counter 觸發把 EKF_x 歸零（見 EKF_Task 第 9 步）。
 * 10.0f：略大於一般降落點誤差與 baro 噪聲，仍遠低於任何有意義的飛行高度。 */
#define EKF_REST_RESET_MAX_BARO_M 10.0f

// GPS equirectangular-projection constants
static const float EARTH_RADIUS_M = 6378137.0f;            // WGS84 semi-major axis
static const float DEG2RAD = 0.017453292519943295f;        // pi / 180

static volatile float g_ekf_cpu_usage CCMRAM = 0.0f;

// --- P0-C: EKF guard (ekf_guard.h) bookkeeping ---
static volatile uint8_t  EKF_health_bits CCMRAM = 0;          // EKF_HB_*（0 = 全健康）
static uint32_t EKF_baro_reject_streak CCMRAM = 0;            // baro 創新值連續拒收計數
static uint32_t EKF_last_baro_accept_tick CCMRAM = 0;         // 最後一筆被接受的 baro tick
static float    EKF_last_baro_accepted_rel CCMRAM = 0.0f;     // 最後被接受的 baro 相對高度（NaN 重建用）
static uint32_t EKF_last_oob_tick CCMRAM = 0;                 // 最後一次狀態夾限 tick（0=從未）
static uint32_t EKF_last_nan_tick CCMRAM = 0;                 // 最後一次 NaN 重建 tick（0=從未）
static uint32_t EKF_diverge_since_tick CCMRAM = 0;            // P1：DIVERGE 起算 tick（垂直自救用）
static volatile uint32_t EKF_last_update_tick CCMRAM = 0;     // EKF_Task 最後完成一個 buffer 的 tick

// -------------------------------------------------------------------------
// FreeRTOS Task and Queue definitions mapped to CCM RAM
// -------------------------------------------------------------------------
osMessageQueueId_t xEKFQueue;

static StaticTask_t EKF_TaskControlBlock CCMRAM;
static StackType_t EKF_TaskStack[1024] CCMRAM;

const osThreadAttr_t EKF_Task_attributes = {
  .name = "ekfTask",
  .cb_mem = &EKF_TaskControlBlock,
  .cb_size = sizeof(EKF_TaskControlBlock),
  .stack_mem = EKF_TaskStack,
  .stack_size = sizeof(EKF_TaskStack),
  .priority = (osPriority_t) osPriorityHigh,
};

// -------------------------------------------------------------------------
// DWT High-Resolution Clock Implementation (Cortex-M4 Cycle Counter)
// -------------------------------------------------------------------------

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetMicroseconds(void) {
    // STM32F407 clock runs at 168 MHz
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

// -------------------------------------------------------------------------
// EKF State & Math Functions
// -------------------------------------------------------------------------

void EKF_Init(void) {
    // Reset state to origin with zero velocity
    memset(EKF_x, 0, sizeof(EKF_x));

    // Initialize state covariance P as diagonal matrix
    memset(EKF_P, 0, sizeof(EKF_P));
    for (int i = 0; i < 3; i++) {
        EKF_P[i][i] = 1.0f;       // Initial position uncertainty (1.0 m^2)
        EKF_P[i+3][i+3] = 2.0f;   // Initial velocity uncertainty (2.0 m^2/s^2)
    }

    // Initialize attitude quaternion as identity (no rotation)
    EKF_q[0] = 1.0f;
    EKF_q[1] = 0.0f;
    EKF_q[2] = 0.0f;
    EKF_q[3] = 0.0f;

    // Reset calibration and launchpad variables
    EKF_calibrated = 0;
    EKF_calib_samples = 0;
    EKF_accel_calib_n = 0;
    memset(EKF_accel_bias, 0, sizeof(EKF_accel_bias));
    memset(EKF_accel_sum, 0, sizeof(EKF_accel_sum));
    memset(EKF_gyro_bias, 0, sizeof(EKF_gyro_bias));
    memset(EKF_gyro_sum, 0, sizeof(EKF_gyro_sum));
    
    EKF_baro_launchpad = 0.0f;
    EKF_baro_sum = 0.0f;
    EKF_baro_samples = 0;

    /* P0-x：Flash 校準參考值歸零（防 CCMRAM 殘值；隨後 LoadCalibrationFromFlash
     * 讀取成功才重新置 valid） */
    memset(EKF_flash_calib_accel_bias, 0, sizeof(EKF_flash_calib_accel_bias));
    memset(EKF_flash_calib_gyro_bias, 0, sizeof(EKF_flash_calib_gyro_bias));
    EKF_flash_calib_baro_launchpad = 0.0f;
    EKF_flash_calib_valid = 0;

    // Reset health check and guard variables (P0-C CCMRAM safety)
    EKF_health_bits = 0;
    EKF_baro_reject_streak = 0;
    EKF_last_baro_accept_tick = 0;
    EKF_last_baro_accepted_rel = 0.0f;
    EKF_last_oob_tick = 0;
    EKF_last_nan_tick = 0;
    EKF_diverge_since_tick = 0;
    EKF_last_update_tick = 0;
    
    EKF_in_flight = 0;
    EKF_launch_counter = 0;
    EKF_rest_counter = 0;

    // GPS fusion: clear origin/pending so a fresh launchpad origin is captured
    EKF_gps_pending = 0;
    EKF_gps_origin_set = 0;
    EKF_gps_lat0_1e6 = 0;
    EKF_gps_lon0_1e6 = 0;
    EKF_gps_coslat0 = 1.0f;
    EKF_gps_meas_E = 0.0f;
    EKF_gps_meas_N = 0.0f;
    EKF_gps_R = 25.0f;
    EKF_gps_origin_lat_sum = 0;
    EKF_gps_origin_lon_sum = 0;
    EKF_gps_origin_sample_count = 0;

    // Magnetometer fusion: clear pending
    EKF_mag_pending = 0;
    EKF_mag_x = 0.0f;
    EKF_mag_y = 0.0f;
    EKF_mag_z = 0.0f;

    // Load static calibration parameters from flash if they exist
    EKF_LoadCalibrationFromFlash();
}

void EKF_ResetOrientation(void) {
    taskENTER_CRITICAL();
    // Reset position and velocity to 0
    memset(EKF_x, 0, sizeof(EKF_x));
    
    // Reset state covariance P to standard diagonal elements
    memset(EKF_P, 0, sizeof(EKF_P));
    for (int i = 0; i < 3; i++) {
        EKF_P[i][i] = 1.0f;       // Initial position uncertainty (1.0 m^2)
        EKF_P[i+3][i+3] = 2.0f;   // Initial velocity uncertainty (2.0 m^2/s^2)
    }
    
    // Reset attitude quaternion to identity (defines current physical alignment as upright)
    EKF_q[0] = 1.0f;
    EKF_q[1] = 0.0f;
    EKF_q[2] = 0.0f;
    EKF_q[3] = 0.0f;
    taskEXIT_CRITICAL();
    
    printf("[EKF] Orientation & Position manually reset to zero (upright)!\r\n");
}

void EKF_AttitudeUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float qw = EKF_q[0];
    float qx = EKF_q[1];
    float qy = EKF_q[2];
    float qz = EKF_q[3];

    // If we are stationary on the launchpad (not in flight), we apply a Mahony
    // Complementary Filter gravity feedback loop. This uses the accelerometer's
    // gravity vector to estimate roll and pitch errors, and feeds them back
    // to correct gyroscope drift, forcing the attitude to pull back to absolute level.
    if ((1 && !EKF_in_flight) || (current_fsm_state >= STATE_DESCENT)) {
        float norm_a = sqrtf(ax*ax + ay*ay + az*az);
        /* P0-x：加速度規範閘 —— 原本 norm_a>0.01f 幾乎恆真，任何非重力加速度
         * （電梯啟停瞬間、地面晃動、震動）都會被當成「重力方向」回授進姿態，
         * 造成誤傾斜估計。改為同時要求量測值落在 1g 附近（|norm_a-g|<1.0 m/s²）
         * 才回授；非重力加速度期間直接跳過，姿態暫由陀螺遞推，待加速度計
         * 讀數回到 1g 附近再繼續收斂。 */
        if (norm_a > 0.01f && fabsf(norm_a - GRAVITY) < 1.0f) {
            float ax_n = ax / norm_a;
            float ay_n = ay / norm_a;
            float az_n = az / norm_a;

            // Predicted gravity direction in the body frame (third row of R(q))
            float vx = 2.0f * (qx*qz - qw*qy);
            float vy = 2.0f * (qy*qz + qw*qx);
            float vz = 1.0f - 2.0f * (qx*qx + qy*qy);

            // Error is the cross product between measured gravity and predicted gravity
            float ex = (ay_n * vz - az_n * vy);
            float ey = (az_n * vx - ax_n * vz);
            float ez = (ax_n * vy - ay_n * vx);

            // Apply feedback correction (Kp = 2.0f pulls the tilt back to level within 1s)
            float Kp = 2.0f;
            gx += Kp * ex;
            gy += Kp * ey;
            gz += Kp * ez;
        }
    }

    // Quaternion derivative integration
    float d_qw = 0.5f * dt * (-qx * gx - qy * gy - qz * gz);
    float d_qx = 0.5f * dt * ( qw * gx + qy * gz - qz * gy);
    float d_qy = 0.5f * dt * ( qw * gy - qx * gz + qz * gx);
    float d_qz = 0.5f * dt * ( qw * gz + qx * gy - qy * gx);

    qw += d_qw;
    qx += d_qx;
    qy += d_qy;
    qz += d_qz;

    // Re-normalize quaternion to prevent accumulation of numerical errors
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 1e-6f) {
        EKF_q[0] = qw / norm;
        EKF_q[1] = qx / norm;
        EKF_q[2] = qy / norm;
        EKF_q[3] = qz / norm;
    }
}

// Compute initial attitude quaternion from gravity vector + magnetometer vector using TRIAD.
// grav_b : body-frame gravity (averaged accel during calibration, points toward +Z when upright).
// mag_b  : body-frame magnetic field (body-frame, already remapped).
// Sets EKF_q to the correct absolute attitude so Mahony has nothing left to converge.
static void EKF_InitAttitudeFromAccelMag(const float grav_b[3], const float mag_b[3]) {
    // TRIAD：由 body 重力 + 磁場求 body->nav 初始四元數（純數學在 attitude_math.h，
    // 已由 tests/test_sensor_axis.c 獨立驗證）。退化時 q 保持不變，交由 Mahony 收斂。
    float q[4];
    if (att_triad_grav_mag_to_quat(grav_b, mag_b, q)) {
        EKF_q[0] = q[0]; EKF_q[1] = q[1]; EKF_q[2] = q[2]; EKF_q[3] = q[3];
    }
}

// Magnetometer heading (yaw) correction, Mahony-style, in the SAME body->nav
// rotation convention as EKF_AttitudeUpdate/EKF_Predict (a_nav = R*a_body, with
// the gravity/up direction in body = third row of R). The horizontal-collapse of
// the reference field constrains heading only, leaving roll/pitch to gravity.
// Applied on the launchpad only, to pin absolute heading and arrest the gyro yaw
// drift that accelerometer (gravity-only) feedback cannot observe.
//   mx,my,mz : magnetometer vector in the IMU body frame (any unit; direction only)
//   dt       : integration interval for the complementary correction (s)
static void EKF_MagYawUpdate(float mx, float my, float mz, float dt) {
    float nm = sqrtf(mx*mx + my*my + mz*mz);
    if (nm < 1e-6f) return;                 // degenerate / no field
    float mxn = mx / nm, myn = my / nm, mzn = mz / nm;

    float qw = EKF_q[0], qx = EKF_q[1], qy = EKF_q[2], qz = EKF_q[3];

    // Rotation matrix R (body -> nav), identical to EKF_Predict
    float r11 = 1.0f - 2.0f*(qy*qy + qz*qz);
    float r12 = 2.0f*(qx*qy - qw*qz);
    float r13 = 2.0f*(qx*qz + qw*qy);
    float r21 = 2.0f*(qx*qy + qw*qz);
    float r22 = 1.0f - 2.0f*(qx*qx + qz*qz);
    float r23 = 2.0f*(qy*qz - qw*qx);
    float r31 = 2.0f*(qx*qz - qw*qy);
    float r32 = 2.0f*(qy*qz + qw*qx);
    float r33 = 1.0f - 2.0f*(qx*qx + qy*qy);

    // Rotate body mag into nav frame: h = R * m
    float hx = r11*mxn + r12*myn + r13*mzn;
    float hy = r21*mxn + r22*myn + r23*mzn;
    float hz = r31*mxn + r32*myn + r33*mzn;

    // Reference field b = [0, by, bz]: horizontal magnitude collapsed onto nav-North,
    // measured inclination kept on Up. Fixes the heading origin without touching tilt.
    float by = sqrtf(hx*hx + hy*hy);
    float bz = hz;

    // Predicted mag direction back in body frame: w = R^T * b   (bx = 0)
    float wx = r21*by + r31*bz;
    float wy = r22*by + r32*bz;
    float wz = r23*by + r33*bz;

    // Error = measured x predicted (body frame); predominantly a yaw error vector
    float ex = (myn*wz - mzn*wy);
    float ey = (mzn*wx - mxn*wz);
    float ez = (mxn*wy - myn*wx);

    // Apply as a gyro-like correction integrated over dt (Kp_mag tunable on bench)
    const float Kp_mag = 10.0f;  /* 提高地面收斂速度，從原本 1.0f 提高到 10.0f */
    float gx = Kp_mag * ex;
    float gy = Kp_mag * ey;
    float gz = Kp_mag * ez;

    float d_qw = 0.5f * dt * (-qx*gx - qy*gy - qz*gz);
    float d_qx = 0.5f * dt * ( qw*gx + qy*gz - qz*gy);
    float d_qy = 0.5f * dt * ( qw*gy - qx*gz + qz*gx);
    float d_qz = 0.5f * dt * ( qw*gz + qx*gy - qy*gx);

    qw += d_qw; qx += d_qx; qy += d_qy; qz += d_qz;
    float qn = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (qn > 1e-6f) {
        EKF_q[0] = qw/qn; EKF_q[1] = qx/qn; EKF_q[2] = qy/qn; EKF_q[3] = qz/qn;
    }
}

void EKF_Predict(float ax, float ay, float az, float dt) {
    // 1. Get rotation matrix R from attitude quaternion (body to local ENU navigation frame)
    float qw = EKF_q[0];
    float qx = EKF_q[1];
    float qy = EKF_q[2];
    float qz = EKF_q[3];

    float r11 = 1.0f - 2.0f * (qy*qy + qz*qz);
    float r12 = 2.0f * (qx*qy - qw*qz);
    float r13 = 2.0f * (qx*qz + qw*qy);

    float r21 = 2.0f * (qx*qy + qw*qz);
    float r22 = 1.0f - 2.0f * (qx*qx + qz*qz);
    float r23 = 2.0f * (qy*qz - qw*qx);

    float r31 = 2.0f * (qx*qz - qw*qy);
    float r32 = 2.0f * (qy*qz + qw*qx);
    float r33 = 1.0f - 2.0f * (qx*qx + qy*qy);

    // 2. Rotate body acceleration into navigation frame
    float a_nav_x = r11 * ax + r12 * ay + r13 * az;
    float a_nav_y = r21 * ax + r22 * ay + r23 * az;
    float a_nav_z = r31 * ax + r32 * ay + r33 * az - GRAVITY; // Subtract gravity along ENU z-axis

    // 3. Propagate state vector x
    // p = p + v * dt + 0.5 * a * dt^2
    // v = v + a * dt
    float dt2 = 0.5f * dt * dt;
    EKF_x[0] += EKF_x[3] * dt + a_nav_x * dt2;
    EKF_x[1] += EKF_x[4] * dt + a_nav_y * dt2;
    EKF_x[2] += EKF_x[5] * dt + a_nav_z * dt2;

    EKF_x[3] += a_nav_x * dt;
    EKF_x[4] += a_nav_y * dt;
    EKF_x[5] += a_nav_z * dt;

    // 4. Analytically propagate 6x6 covariance matrix P = F * P * F^T + Q
    // Partition P into four 3x3 matrices: P_pp (top-left), P_pv (top-right), P_vp (bottom-left), P_vv (bottom-right)
    float P_pp_new[3][3];
    float P_pv_new[3][3];
    float P_vp_new[3][3];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // P_pp = P_pp + dt * (P_vp + P_pv) + dt^2 * P_vv
            P_pp_new[i][j] = EKF_P[i][j] + dt * (EKF_P[i+3][j] + EKF_P[i][j+3]) + dt * dt * EKF_P[i+3][j+3];
            // P_pv = P_pv + dt * P_vv
            P_pv_new[i][j] = EKF_P[i][j+3] + dt * EKF_P[i+3][j+3];
            // P_vp = P_vp + dt * P_vv
            P_vp_new[i][j] = EKF_P[i+3][j] + dt * EKF_P[i+3][j+3];
        }
    }

    // Apply updates back to P, adding process noise Q along the diagonal
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EKF_P[i][j] = P_pp_new[i][j];
            EKF_P[i][j+3] = P_pv_new[i][j];
            EKF_P[i+3][j] = P_vp_new[i][j];
        }
        EKF_P[i][i] += Q_pos;       // Add process noise to position diagonal
        EKF_P[i+3][i+3] += Q_vel;   // Add process noise to velocity diagonal
    }

    /* P0-C：協方差對角夾限 —— 防 baro 長時間中斷時 P 無界增長，
     * 量測恢復瞬間 Kalman 增益爆炸把狀態拉飛。 */
    ekf_guard_clamp_P(EKF_P);
}

// Group-delay compensated measurement update using the Analytical Joseph Form
void EKF_UpdateBaroDelayed(float baro_alt, float z_pred) {
    // 1. Innovation using prediction at correct historical time
    float y = baro_alt - z_pred;

    // 2. Innovation Covariance S = H * P * H^T + R = P[2][2] + R_baro
    float S = EKF_P[2][2] + R_baro;
    if (S < 1e-6f) return; // Prevent division by zero

    /* P0-C：創新值閘控 |y| > max(5σ, 25m) 拒收 —— 單筆 ±10km 壞值不再直接
     * 餵進濾波器。連續拒收 ≥200 次（@200Hz ≈1s）視為「估計已發散或 baro 已壞」，
     * 置 EKF_HB_BARO_DIVERGE 健康位（FSM 據此切 raw-baro 降級開傘鏈），
     * 不強行吞入壞值。任一筆被接受即清除位與計數。 */
    if (!ekf_guard_baro_accept(y, S)) {
        if (EKF_baro_reject_streak < 0xFFFFFFFFU) {
            EKF_baro_reject_streak++;
        }
        if (EKF_baro_reject_streak >= EKF_GUARD_BARO_REJECT_N) {
            if ((EKF_health_bits & EKF_HB_BARO_DIVERGE) == 0U) {
                EKF_diverge_since_tick = HAL_GetTick();
            }
            EKF_health_bits |= EKF_HB_BARO_DIVERGE;

            /* P1：發散自救 —— DIVERGE 持續 >3s 且 baro 仍持續供數
             * （sensor_health 若標 BMP388 故障則 has_baro=0、根本不會進到這裡，
             * 故此時 baro 本身可信）→ 垂直通道重置：高度拍至當下 baro、垂直
             * 速度歸零（P 放大讓 200Hz 量測在 ~0.3s 內重新拉出真實速度）、
             * P 的 z/vz 行列重設 —— 讓濾波器重新收斂而非永久降級。
             * 不清 DIVERGE 位：之後 baro 被接受時自然清除（innovation 已歸零；
             * z_history 群延遲補償殘留的 ≤EKF_BARO_GROUP_DELAY_SAMPLES 筆舊預測會再拒收
             * ~EKF_BARO_GROUP_DELAY_MS ms，無妨）。 */
            if ((HAL_GetTick() - EKF_diverge_since_tick) > EKF_GUARD_DIVERGE_RESET_MS) {
                EKF_x[2] = baro_alt;
                EKF_x[5] = 0.0f;
                for (int i = 0; i < 6; i++) {
                    EKF_P[2][i] = 0.0f; EKF_P[i][2] = 0.0f;
                    EKF_P[5][i] = 0.0f; EKF_P[i][5] = 0.0f;
                }
                EKF_P[2][2] = 5.0f;     /* 與 EKF_HotRestartRestore 相同的空中高度不確定度 */
                EKF_P[5][5] = 100.0f;   /* 速度未知（σ=10m/s），讓量測快速重建 */
                EKF_baro_reject_streak = 0;
                EKF_diverge_since_tick = HAL_GetTick();   /* 若再度發散，重新計時 3s */
                printf("[EKF] [WARNING] BARO_DIVERGE >3s — vertical channel reset (alt -> baro %d cm)\r\n",
                       (int)(baro_alt * 100.0f));
            }
        }
        return;
    }
    EKF_baro_reject_streak = 0;
    EKF_health_bits &= (uint8_t)~EKF_HB_BARO_DIVERGE;
    EKF_last_baro_accept_tick  = HAL_GetTick();
    EKF_last_baro_accepted_rel = baro_alt;

    float invS = 1.0f / S;

    // 3. Kalman Gain K = P * H^T * invS (H^T is the 3rd column of P, index 2)
    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = EKF_P[i][2] * invS;
    }

    // 4. State correction x = x + K * y
    for (int i = 0; i < 6; i++) {
        EKF_x[i] += K[i] * y;
    }

    // 5. Covariance correction using robust Analytical Joseph Form:
    // P_new = (I - KH)*P*(I - KH)^T + K*R_baro*K^T
    // First, compute intermediate P' = (I - K*H)*P
    float P_prime[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_prime[i][j] = EKF_P[i][j] - K[i] * EKF_P[2][j];
        }
    }

    // Then compute P_new[i][j] = P'[i][j] - P'[i][2]*K[j] + K[i]*R_baro*K[j]
    for (int i = 0; i < 6; i++) {
        float P_prime_i2 = P_prime[i][2];
        for (int j = 0; j < 6; j++) {
            EKF_P[i][j] = P_prime[i][j] - P_prime_i2 * K[j] + K[i] * R_baro * K[j];
        }
    }
}

// Fallback interface to support standard direct barometer updates
void EKF_UpdateBaro(float baro_alt) {
    float baro_rel = baro_alt - EKF_baro_launchpad;
    EKF_UpdateBaroDelayed(baro_rel, EKF_x[2]);
}

// Scalar position measurement update on a single state axis (idx: 0=E, 1=N, 2=U)
// using the same Analytical Joseph Form as the barometer update. H selects row idx.
static void EKF_UpdateScalarPos(int idx, float z, float R) {
    float y = z - EKF_x[idx];               // Innovation
    float S = EKF_P[idx][idx] + R;          // Innovation covariance
    if (S < 1e-6f) return;
    float invS = 1.0f / S;

    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = EKF_P[i][idx] * invS;        // Kalman gain (column idx of P)
    }
    for (int i = 0; i < 6; i++) {
        EKF_x[i] += K[i] * y;               // State correction
    }

    // Joseph form: P_new = (I-KH)P(I-KH)^T + K R K^T
    float P_prime[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_prime[i][j] = EKF_P[i][j] - K[i] * EKF_P[idx][j];
        }
    }
    for (int i = 0; i < 6; i++) {
        float P_prime_ii = P_prime[i][idx];
        for (int j = 0; j < 6; j++) {
            EKF_P[i][j] = P_prime[i][j] - P_prime_ii * K[j] + K[i] * R * K[j];
        }
    }
}

// Apply the pending GPS horizontal fix as two independent scalar updates on
// East (idx 0) and North (idx 1). Altitude (Up) is left to the barometer, which
// is far more accurate than GPS vertical. Called from EKF_Task only.
static void EKF_UpdateGPS(float meas_E, float meas_N, float R) {
    EKF_UpdateScalarPos(0, meas_E, R);
    EKF_UpdateScalarPos(1, meas_N, R);
}

// GPS injection from defaultTask (~1 Hz). Captures the launchpad origin on the
// first valid post-calibration fix, then publishes a pending ENU measurement.
void EKF_SubmitGPS(int32_t lat_1e6, int32_t lon_1e6, uint8_t satellites, uint32_t hacc_mm) {
    // Only fuse once stationary calibration is done, so the GPS origin coincides
    // with the same launchpad reference the rest of the EKF state is relative to.
    if (!EKF_calibrated) return;

    // Launchpad origin: accumulate EKF_GPS_ORIGIN_AVG_N fixes and lock the origin
    // to their mean (dE = dN = 0 at the origin by definition) instead of trusting
    // a single fix — averaging divides the origin's noise by sqrt(N) rather than
    // just hoping one sample happens to be good.
    if (!EKF_gps_origin_set) {
        if (hacc_mm > 0U && hacc_mm > EKF_GPS_ORIGIN_MAX_HACC_MM) {
            return; // clearly-broken outlier fix: don't let it into the average
        }

        EKF_gps_origin_lat_sum += lat_1e6;
        EKF_gps_origin_lon_sum += lon_1e6;
        EKF_gps_origin_sample_count++;

        if (EKF_gps_origin_sample_count < EKF_GPS_ORIGIN_AVG_N) {
            return; // still accumulating
        }

        EKF_gps_lat0_1e6 = (int32_t)(EKF_gps_origin_lat_sum / (int64_t)EKF_gps_origin_sample_count);
        EKF_gps_lon0_1e6 = (int32_t)(EKF_gps_origin_lon_sum / (int64_t)EKF_gps_origin_sample_count);
        EKF_gps_coslat0  = cosf((float)EKF_gps_lat0_1e6 * 1e-6f * DEG2RAD);
        EKF_gps_origin_set = 1;
        return;
    }

    // Equirectangular projection about the origin (accurate to <0.1% over the
    // few-km horizontal extent of a sounding-rocket flight).
    float dlat_deg = (float)(lat_1e6 - EKF_gps_lat0_1e6) * 1e-6f;
    float dlon_deg = (float)(lon_1e6 - EKF_gps_lon0_1e6) * 1e-6f;
    float meas_N = dlat_deg * DEG2RAD * EARTH_RADIUS_M;
    float meas_E = dlon_deg * DEG2RAD * EARTH_RADIUS_M * EKF_gps_coslat0;

    // Measurement noise from the receiver's own per-fix horizontal accuracy
    // estimate (UBX-NAV-PVT hAcc, mm) instead of a satellite-count guess: the
    // module already solves for this every fix, so R = hAcc^2 directly. Floored
    // at 1 m sigma (a real fix is never trusted better than that here, even if
    // hAcc briefly reports less right after acquisition) and capped at the old
    // worst-case tier so a garbage value can't blow up P. hacc_mm == 0 means no
    // UBX PVT accuracy estimate was available (e.g. NMEA-only fallback path) —
    // fall back to the old satellite-count tiers in that case only.
    float R;
    if (hacc_mm > 0U) {
        float hacc_m = (float)hacc_mm * 0.001f;
        R = hacc_m * hacc_m;
        if (R < 1.0f) R = 1.0f;
        if (R > 2500.0f) R = 2500.0f;
    } else if (satellites >= 8) {
        R = 6.25f;    // ~2.5 m sigma
    } else if (satellites >= 6) {
        R = 25.0f;    // ~5 m
    } else if (satellites >= 4) {
        R = 100.0f;   // ~10 m
    } else {
        R = 2500.0f;  // barely trust a 3-sat fix
    }

    // Publish atomically; EKF_Task (higher priority) may preempt mid-write.
    taskENTER_CRITICAL();
    EKF_gps_meas_E = meas_E;
    EKF_gps_meas_N = meas_N;
    EKF_gps_R = R;
    EKF_gps_pending = 1;
    taskEXIT_CRITICAL();
}

// Magnetometer injection from defaultTask (~10 Hz). Publishes the latest body-
// frame field vector; EKF_Task applies it as a pad-phase yaw correction.
void EKF_SubmitMag(float mx, float my, float mz) {
    taskENTER_CRITICAL();
    EKF_mag_x = mx;
    EKF_mag_y = my;
    EKF_mag_z = mz;
    EKF_mag_pending = 1;
    taskEXIT_CRITICAL();
}

EKF_State_t EKF_GetState(void) {
    EKF_State_t state;
    /* 改善項目 O：原子快照。EKF_Task（High prio）會連續寫入 EKF_x/EKF_q/EKF_accel_bias，
     * 而 defaultTask（Normal prio）於遙測 / FSM / SD 路徑呼叫本函式讀取；若不保護，
     * High-prio 任務可能在拷貝中途搶佔，造成「位置取自舊一輪、速度取自新一輪」的撕裂讀取。
     * 以臨界區封住拷貝期間（純記憶體搬移，數十週期），讓回傳的 state 為一致快照。
     * 註：EKF_Task 內部亦呼叫本函式（同任務），FreeRTOS 臨界區可巢狀，安全。 */
    taskENTER_CRITICAL();
    state.pos_x = EKF_x[0];
    state.pos_y = EKF_x[1];
    state.pos_z = EKF_x[2];
    state.vel_x = EKF_x[3];
    state.vel_y = EKF_x[4];
    state.vel_z = EKF_x[5];
    memcpy(state.q, EKF_q, sizeof(EKF_q));
    memcpy(state.accel_bias, EKF_accel_bias, sizeof(state.accel_bias));
    taskEXIT_CRITICAL();
    return state;
}

/* P0-C：健康位快照（單 byte volatile，ARM 上原子讀，無需臨界區）。0 = 全健康。 */
uint8_t EKF_GetHealthBits(void) {
    return EKF_health_bits;
}

/* P0-C：EKF_Task 最後完成 buffer 的 tick。FSM 端以 (now − 此值) > 300ms
 * 判定 EKF_Task 餓死/queue 斷流 —— 光看健康位不夠，EKF 死了就不會再更新位。 */
uint32_t EKF_GetLastUpdateTick(void) {
    return EKF_last_update_tick;
}

// Helper function to format floats safely using fast integer arithmetic
// Incorporates robust NaN, Infinity, and Overflow checks to prevent labs() HardFaults
static void EKF_PrintFloat(float val, char separator) {
    if (isnan(val)) {
        printf("NaN%c", separator);
        return;
    }
    if (isinf(val)) {
        if (val < 0.0f) {
            printf("-INF%c", separator);
        } else {
            printf("INF%c", separator);
        }
        return;
    }

    float scaled = val * 1000.0f;
    // Bound check against INT32 limits to prevent undefined casting/labs behavior
    if (scaled > 2147483647.0f) {
        printf("OVR%c", separator);
        return;
    }
    if (scaled < -2147483647.0f) {
        printf("-OVR%c", separator);
        return;
    }

    int32_t val_mm = (int32_t)scaled;
    if (val_mm < 0) {
        printf("-%ld.%03ld%c", labs(val_mm) / 1000, labs(val_mm) % 1000, separator);
    } else {
        printf("%ld.%03ld%c", val_mm / 1000, val_mm % 1000, separator);
    }
}

// -------------------------------------------------------------------------
// EKF Thread Implementation (Waiting for pointers via Queue)
// -------------------------------------------------------------------------

void EKF_Task(void *argument) {
    (void)argument;
    EKF_Buffer_t* p_buf = NULL;
    uint32_t last_cyc = 0;      // 上一顆樣本的 DWT->CYCCNT；0 = 尚未有上一顆（任務剛啟動）

    // Local circular history buffer for barometer group-delay compensation.
    // 群延遲樣本數/緩衝長度依 ekf.h 的 EKF_BARO_GROUP_DELAY_SAMPLES / EKF_Z_HISTORY_LEN
    // （隨 BMP388 ODR 與 EKF_GYRO_RATE_HZ 連動換算，見該處註解）。
    float z_history[EKF_Z_HISTORY_LEN] = {0.0f};
    uint8_t z_history_idx = 0;

    // P0-x：靜止偵測器歸零防護 —— 追蹤最新一筆 baro 相對高度（直接以原始樣本
    // 計算，不經 EKF_UpdateBaroDelayed 的閘控/接受邏輯），供第 9 步的地面狀態
    // 判斷式使用。與 z_history 同為函式作用域局部變數，EKF_Task 為無限迴圈，
    // 生命週期涵蓋整個任務執行期間。
    float ekf_rest_last_baro_rel = 0.0f;

    // Predict()(位置/共變異數，400Hz) 累積的 dt：AttitudeUpdate 每顆樣本(1000Hz)都跑，
    // 但 Predict 只在 has_accel=1（真正的新加速度樣本）時才跑一次，用「上次 Predict
    // 之後累積的所有 dt」作為其積分區間，涵蓋期間所有 ZOH 樣本的時間。函式作用域，
    // 不因 buffer 邊界重置——buffer 間的 10ms frame 本就連續，不應在邊界處丟失累積。
    float dt_pred_accum = 0.0f;

    EKF_Init();

    // Enable CPU Cycle Counter for microsecond timestamps
    DWT_Init();

    printf("[EKF] EKF High-Performance Task successfully spawned in CCM RAM!\r\n");

    for (;;) {
        // Wait forever until a pointer to a filled buffer is placed on the queue
        if (osMessageQueueGet(xEKFQueue, &p_buf, NULL, osWaitForever) == osOK) {
            if (p_buf == NULL) continue;

            uint32_t start_cycles = DWT->CYCCNT;

            // Playback the EKF on the 100 accumulated samples sequentially
            for (int i = 0; i < EKF_BUFFER_SIZE; i++) {
                EKF_Sample_t* sample = &p_buf->samples[i];

                float dt;
                if (last_cyc == 0) {
                    dt = 1.0f / (float)EKF_GYRO_RATE_HZ; // Standard interval as baseline
                } else {
                    // Cycle-domain 差值：uint32 減法本身對 2^32 回繞安全（無論
                    // sample->timestamp_cyc 是否已繞過 last_cyc，結果都是正確的
                    // 正向經過 cycle 數），不像先除後減的 us 版本會在 ~25.6s
                    // 週期處算錯。DWT->CYCCNT 本身在 168MHz 下約 25.6s 回繞一次，
                    // 但 diff 只要小於半個回繞週期（實際遠小於，正常樣本間隔僅 ~1ms）
                    // 就恆正確。
                    uint32_t diff = sample->timestamp_cyc - last_cyc;
                    dt = (float)diff * (1.0f / (float)SystemCoreClock);
                }
                last_cyc = sample->timestamp_cyc;

                // Protect against outliers/system startup timing gaps
                if (dt <= 0.0f || dt > 0.05f) {
                    dt = 1.0f / (float)EKF_GYRO_RATE_HZ;
                }

                // --- 1. Dynamic Accelerometer & Barometer Stationary Calibration Phase ---
                if (!EKF_calibrated) {
                    // accel 為 400Hz ZOH：只在 has_accel=1（真正新樣本）時累加，否則
                    // 同一筆值會被 1000Hz 迴圈重複灌入 2~3 次，稀釋不了平均值但會讓
                    // 「除以樣本數」的分母與實際獨立樣本數脫鉤，故獨立計數
                    // EKF_accel_calib_n，不與陀螺共用 EKF_calib_samples。
                    if (sample->has_accel) {
                        EKF_accel_sum[0] += sample->ax;
                        EKF_accel_sum[1] += sample->ay;
                        EKF_accel_sum[2] += sample->az;
                        EKF_accel_calib_n++;
                    }
                    EKF_gyro_sum[0] += sample->gx;
                    EKF_gyro_sum[1] += sample->gy;
                    EKF_gyro_sum[2] += sample->gz;
                    EKF_calib_samples++;

                    if (sample->has_baro) {
                        EKF_baro_sum += sample->baro_alt;
                        EKF_baro_samples++;
                    }

                    if (EKF_calib_samples >= EKF_CALIB_SAMPLES) {
                        EKF_accel_bias[0] = EKF_accel_sum[0] / (float)EKF_accel_calib_n;
                        EKF_accel_bias[1] = EKF_accel_sum[1] / (float)EKF_accel_calib_n;

                        // Universal bias calculation: forces az_corr to equal +g when stationary upright
                        float avg_az = EKF_accel_sum[2] / (float)EKF_accel_calib_n;
                        EKF_accel_bias[2] = avg_az - GRAVITY;

                        EKF_gyro_bias[0] = EKF_gyro_sum[0] / (float)EKF_calib_samples;
                        EKF_gyro_bias[1] = EKF_gyro_sum[1] / (float)EKF_calib_samples;
                        EKF_gyro_bias[2] = EKF_gyro_sum[2] / (float)EKF_calib_samples;

                        if (EKF_baro_samples > 0) {
                            EKF_baro_launchpad = EKF_baro_sum / (float)EKF_baro_samples;
                        } else {
                            EKF_baro_launchpad = 0.0f;
                        }

                        EKF_calibrated = 1;
                        EKF_SaveCalibrationToFlash();

                        // Initialize attitude from accel (gravity direction) + mag (heading).
                        // This gives the correct absolute attitude immediately — no Mahony convergence needed.
                        // grav_b = raw average accel during calibration (before bias removal; points toward +Z_body when upright)
                        float grav_b[3] = {
                            EKF_accel_sum[0] / (float)EKF_accel_calib_n,
                            EKF_accel_sum[1] / (float)EKF_accel_calib_n,
                            EKF_accel_sum[2] / (float)EKF_accel_calib_n
                        };
                        // Use the most recently submitted mag vector (body-frame, from EKF_SubmitMag).
                        // If mag is not yet available (EKF_mag_x/y/z still 0), TRIAD will return early
                        // and q stays identity — Mahony will then converge normally from the pad.
                        float mag_b[3] = { EKF_mag_x, EKF_mag_y, EKF_mag_z };
                        float mag_norm = sqrtf(mag_b[0]*mag_b[0] + mag_b[1]*mag_b[1] + mag_b[2]*mag_b[2]);
                        if (mag_norm > 0.01f) {
                            EKF_InitAttitudeFromAccelMag(grav_b, mag_b);
                            printf("[EKF] TRIAD initialized successfully.\r\n");
                        } else {
                            printf("[EKF] [WARNING] TRIAD skipped: Magnetometer data not ready yet.\r\n");
                        }

                        printf("[EKF] Stationary Calibration Done!\n");
                        printf("  -> Accel Bias X:");
                        EKF_PrintFloat(EKF_accel_bias[0], ' ');
                        printf("Y:");
                        EKF_PrintFloat(EKF_accel_bias[1], ' ');
                        printf("Z:");
                        EKF_PrintFloat(EKF_accel_bias[2], '\n');
                        printf("  -> Gyro Bias X:");
                        EKF_PrintFloat(EKF_gyro_bias[0], ' ');
                        printf("Y:");
                        EKF_PrintFloat(EKF_gyro_bias[1], ' ');
                        printf("Z:");
                        EKF_PrintFloat(EKF_gyro_bias[2], '\n');
                        printf("  -> Launchpad Baro Alt: ");
                        EKF_PrintFloat(EKF_baro_launchpad, ' ');
                        printf("m\r\n");

                        /* P0-x：陳舊校準偵測 —— 現場新校準與 Flash 參考值比對，
                         * 任一軸偏置差異超過門檻即警告（多半是溫漂或安裝變動；
                         * 新值已生效，警告僅提示 Flash 中的舊校準已不可信）。 */
                        if (EKF_flash_calib_valid) {
                            uint8_t drift = 0U;
                            for (int ci = 0; ci < 3; ci++) {
                                if (fabsf(EKF_gyro_bias[ci]  - EKF_flash_calib_gyro_bias[ci])  > EKF_CALIB_DRIFT_GYRO_MAX)  drift = 1U;
                                if (fabsf(EKF_accel_bias[ci] - EKF_flash_calib_accel_bias[ci]) > EKF_CALIB_DRIFT_ACCEL_MAX) drift = 1U;
                            }
                            if (drift) {
                                printf("[EKF] [WARNING] Stale Flash calibration detected (bias drift > %d mrad/s or %d mm/s^2) — 溫漂或安裝變動，Flash 舊值不可信。\r\n",
                                       (int)(EKF_CALIB_DRIFT_GYRO_MAX * 1000.0f), (int)(EKF_CALIB_DRIFT_ACCEL_MAX * 1000.0f));
                            }
                        }
                    }
                    
                    // Keep position/velocity locked to zero during calibration.
                    // Do NOT reset q here — after calibration completes, TRIAD has already set the
                    // correct initial attitude; resetting it to identity would undo that work.
                    memset(EKF_x, 0, sizeof(EKF_x));
                    continue; // Skip propagation during active calibration
                }

                // --- 2. Correct Sensor Biases ---
                float ax_corr = sample->ax - EKF_accel_bias[0];
                float ay_corr = sample->ay - EKF_accel_bias[1];
                float az_corr = sample->az - EKF_accel_bias[2];

                float gx_corr = sample->gx - EKF_gyro_bias[0];
                float gy_corr = sample->gy - EKF_gyro_bias[1];
                float gz_corr = sample->gz - EKF_gyro_bias[2];

                // --- 3. Attitude Integration with Mahony Gravity Feedback（每顆樣本，1000Hz）---
                // ax_corr/ay_corr/az_corr 在 has_accel=0 樣本上是 ZOH 值：Mahony 的重力回授
                // 時間常數(Kp=2.0，τ~0.5s)遠慢於 2.5ms 保持期，ZOH 對此輸入足夠精確；
                // Kp 本身加在角速度上再乘 dt 積分（EKF_AttitudeUpdate 內部），時間常數與
                // 呼叫速率無關，1000Hz 化不需重新調參。
                EKF_AttitudeUpdate(gx_corr, gy_corr, gz_corr, ax_corr, ay_corr, az_corr, dt);

                // --- 4. Linear Propagation (Predict)（僅新加速度樣本，400Hz）---
                // 用「上次 Predict 之後累積的所有 dt」積分，涵蓋期間所有 ZOH 樣本的時間，
                // 位置/速度/共變異數傳播的物理意義才正確（不是每顆 1000Hz 樣本都重複用
                // 同一顆舊加速度值再 Predict 一次）。
                dt_pred_accum += dt;
                if (sample->has_accel) {
                    EKF_Predict(ax_corr, ay_corr, az_corr, dt_pred_accum);
                    dt_pred_accum = 0.0f;
                }

                // --- 5. Stationary Launchpad Lock (ZUPT) with Baro-Only Trigger ---
                if (!EKF_in_flight) {
                    float relative_baro_alt = 0.0f;
                    if (sample->has_baro) {
                        relative_baro_alt = sample->baro_alt - EKF_baro_launchpad;
                    }

                    // For desktop testing, we only trigger launch when altitude change exceeds 5 meters
                    if (sample->has_baro && relative_baro_alt > 5.0f) {
                        EKF_in_flight = 1;
                        printf("[EKF] Launch Detected! (Baro relative altitude: ");
                        EKF_PrintFloat(relative_baro_alt, ' ');
                        printf("m)\r\n");
                    } else {
                        // Force states to 0 to eliminate all pre-launch horizontal and vertical drift
                        memset(EKF_x, 0, sizeof(EKF_x));
                    }
                }

                // --- 6. Prior State Clipping (Physical Constraints) ---
                if (EKF_in_flight) {
                    if (EKF_x[2] < 0.0f) {
                        EKF_x[2] = 0.0f; // Altitude cannot go below ground level
                        if (EKF_x[5] < 0.0f) {
                            EKF_x[5] = 0.0f; // Velocity cannot go negative at the ground boundary
                        }
                    }
                }

                // --- 7. Save predicted Z-altitude to historical circular buffer ---
                z_history[z_history_idx] = EKF_x[2];
                z_history_idx = (z_history_idx + 1) % EKF_Z_HISTORY_LEN;

                // --- 8. Delayed Measurement Update from Barometer ---
                if (sample->has_baro) {
                    float z_pred;
                    if (EKF_calib_samples > EKF_BARO_GROUP_DELAY_SAMPLES) {
                        // 回看 EKF_BARO_GROUP_DELAY_SAMPLES 個樣本前的預測值，補償 BMP388 群延遲
                        uint8_t hist_idx = (uint8_t)((z_history_idx + EKF_Z_HISTORY_LEN
                                                       - EKF_BARO_GROUP_DELAY_SAMPLES) % EKF_Z_HISTORY_LEN);
                        z_pred = z_history[hist_idx];
                    } else {
                        z_pred = EKF_x[2];
                    }

                    float baro_rel = sample->baro_alt - EKF_baro_launchpad;
                    ekf_rest_last_baro_rel = baro_rel;   // P0-x：供第 9 步歸零防護判斷用的最新值
                    EKF_UpdateBaroDelayed(baro_rel, z_pred);
                }

                // --- 9. Dynamic Rest / Stationary Detector ---
                if (EKF_in_flight) {
                    float gyro_mag = sqrtf(gx_corr*gx_corr + gy_corr*gy_corr + gz_corr*gz_corr);
                    float accel_mag = sqrtf(ax_corr*ax_corr + ay_corr*ay_corr + az_corr*az_corr);

                    // Quiet gyro and gravity magnitude close to 1g
                    if (gyro_mag < 0.15f && fabsf(accel_mag - GRAVITY) < 0.8f) {
                        EKF_rest_counter++;
                    } else {
                        if (EKF_rest_counter > 0) {
                            EKF_rest_counter--;
                        }
                    }

                    // Reset to launchpad mode if resting still for 1.0 second (EKF_REST_SAMPLES @ 1000Hz)
                    if (EKF_rest_counter >= EKF_REST_SAMPLES) {
                        /* P0-x：歸零防護 —— 陀螺安靜 + 加速度計恆 1g 這兩個條件，
                         * 電梯等速段（無論上升或下降途中）1 秒內就會滿足，但此刻
                         * 明明還在空中。原本一律歸零 EKF_x，把電梯等速段誤判成
                         * 「已回到發射台靜止」，是實測誤判主因。
                         * 改為：唯有「FSM 處於地面狀態」（<=STATE_PAD_ARMED，即
                         * INIT/PAD/PAD_ARMED，或 >=STATE_LANDED）且「baro 相對高度
                         * 貼近地面」（< EKF_REST_RESET_MAX_BARO_M）同時成立，才是
                         * 真正靜止在地面（如落地後尚未關機），才可安全歸零高度/速度。
                         * 否則只重新累計計數器、不動狀態，並列印原因供排查。 */
                        uint8_t on_ground_state = (current_fsm_state <= STATE_PAD_ARMED ||
                                                   current_fsm_state >= STATE_LANDED) ? 1U : 0U;
                        uint8_t near_ground_alt  = (ekf_rest_last_baro_rel < EKF_REST_RESET_MAX_BARO_M) ? 1U : 0U;
                        if (on_ground_state && near_ground_alt) {
                            EKF_in_flight = 0;
                            EKF_launch_counter = 0;
                            EKF_rest_counter = 0;
                            memset(EKF_x, 0, sizeof(EKF_x)); // Reset position and velocity to 0 to stop drift
                            printf("[EKF] Rest/Stationary Detected! Resetting to Launchpad Mode.\r\n");
                        } else {
                            EKF_rest_counter = 0;   // 重新累計，不歸零狀態（仍在空中，如電梯等速段）
                            printf("[EKF] [WARNING] Rest counter tripped but still airborne (fsm_state=%d, baro_rel=",
                                   (int)current_fsm_state);
                            EKF_PrintFloat(ekf_rest_last_baro_rel, ' ');
                            printf("m) — zero-reset rejected.\r\n");
                        }
                    }
                } else {
                    EKF_rest_counter = 0;
                }
            }

            /* === P0-C：每 buffer（10Hz）守護掃描 === */
            {
                uint32_t guard_now = HAL_GetTick();

                /* NaN/Inf → 全重建：NaN 會在一個乘加內污染整個濾波器，夾限無效，
                 * 必須重建 —— P 回初值、速度清零、高度拍至最後被接受的 baro、q 回 identity
                 * （Mahony 會自行重新收斂）。 */
                if (ekf_guard_scan_nan(EKF_x, EKF_P, EKF_q)) {
                    taskENTER_CRITICAL();
                    memset(EKF_x, 0, sizeof(EKF_x));
                    EKF_x[2] = EKF_last_baro_accepted_rel;
                    memset(EKF_P, 0, sizeof(EKF_P));
                    for (int gi = 0; gi < 3; gi++) {
                        EKF_P[gi][gi]     = 1.0f;   /* 與 EKF_Init 相同的初始不確定度 */
                        EKF_P[gi+3][gi+3] = 2.0f;
                    }
                    EKF_q[0] = 1.0f; EKF_q[1] = 0.0f; EKF_q[2] = 0.0f; EKF_q[3] = 0.0f;
                    taskEXIT_CRITICAL();
                    EKF_last_nan_tick = guard_now;
                    printf("[EKF] [WARNING] NaN/Inf detected — filter rebuilt (alt snapped to last baro %d cm)\r\n",
                           (int)(EKF_last_baro_accepted_rel * 100.0f));
                }
                if (EKF_last_nan_tick != 0U &&
                    (guard_now - EKF_last_nan_tick) < EKF_GUARD_NAN_STICKY_MS) {
                    EKF_health_bits |= EKF_HB_NAN;
                } else {
                    EKF_health_bits &= (uint8_t)~EKF_HB_NAN;
                }

                /* 狀態理智界限：越界即夾限並置位；回界 1s 後解除。 */
                if (ekf_guard_clamp_state(EKF_x)) {
                    EKF_last_oob_tick = guard_now;
                }
                if (EKF_last_oob_tick != 0U &&
                    (guard_now - EKF_last_oob_tick) < EKF_GUARD_OOB_CLEAR_MS) {
                    EKF_health_bits |= EKF_HB_STATE_OOB;
                } else {
                    EKF_health_bits &= (uint8_t)~EKF_HB_STATE_OOB;
                }

                /* baro 接受逾時（校準完成後才有意義；涵蓋 baro 斷流與持續拒收前期） */
                if (EKF_calibrated &&
                    (guard_now - EKF_last_baro_accept_tick) > EKF_GUARD_BARO_TIMEOUT_MS) {
                    EKF_health_bits |= EKF_HB_BARO_TIMEOUT;
                } else {
                    EKF_health_bits &= (uint8_t)~EKF_HB_BARO_TIMEOUT;
                }

                /* FSM 端以本 tick 判斷 EKF_Task 是否餓死（>300ms 視同 unhealthy） */
                EKF_last_update_tick = guard_now;
            }

            // --- Pending GPS horizontal-position update (applied once per buffer) ---
            // Consume atomically. Only fuse during flight: on the pad the ZUPT lock
            // zeroes horizontal state every sample, so a pad-time fix would be wiped
            // anyway — drop it and wait for fresh in-flight fixes.
            if (EKF_gps_pending) {
                float gps_E, gps_N, gps_R;
                taskENTER_CRITICAL();
                gps_E = EKF_gps_meas_E;
                gps_N = EKF_gps_meas_N;
                gps_R = EKF_gps_R;
                EKF_gps_pending = 0;
                taskEXIT_CRITICAL();

                if (EKF_in_flight) {
                    EKF_UpdateGPS(gps_E, gps_N, gps_R);
                }
            }

            // --- Pending magnetometer heading (yaw) correction (pad phase only) ---
            // Consume atomically. Only correct heading before launch (calibrated &&
            // !in_flight); in flight the attitude is gyro dead-reckoning by design,
            // and the rocket's spin/EMI make the magnetometer unreliable.
            if (EKF_mag_pending) {
                float m_x, m_y, m_z;
                taskENTER_CRITICAL();
                m_x = EKF_mag_x;
                m_y = EKF_mag_y;
                m_z = EKF_mag_z;
                EKF_mag_pending = 0;
                taskEXIT_CRITICAL();

                if (g_mag_yaw_lock && EKF_calibrated && !EKF_in_flight) { // Ground magnetometer heading correction enabled
                    EKF_MagYawUpdate(m_x, m_y, m_z, 0.1f);  // ~10Hz buffer cadence
                }
            }

            uint32_t end_cycles = DWT->CYCCNT;
            uint32_t elapsed_cycles = end_cycles - start_cycles;
            float elapsed_ms = (float)elapsed_cycles / (float)SystemCoreClock * 1000.0f;

            // Accumulate cycles for CPU usage calculation
            static uint32_t ekf_accumulated_cycles = 0;
            static uint32_t last_cpu_calc_tick = 0;
            uint32_t current_tick = HAL_GetTick();
            if (last_cpu_calc_tick == 0) {
                last_cpu_calc_tick = current_tick;
            }
            ekf_accumulated_cycles += elapsed_cycles;
            if (current_tick - last_cpu_calc_tick >= 1000) {
                uint32_t elapsed_ms_time = current_tick - last_cpu_calc_tick;
                uint32_t total_possible_cycles = elapsed_ms_time * (SystemCoreClock / 1000);
                if (total_possible_cycles > 0) {
                    g_ekf_cpu_usage = (float)ekf_accumulated_cycles / (float)total_possible_cycles * 100.0f;
                }
                ekf_accumulated_cycles = 0;
                last_cpu_calc_tick = current_tick;
            }

            // Report estimated position, velocity, attitude and execution duration
            EKF_State_t current = EKF_GetState();

            // 1. Output diagnostic logs [EKF]
            printf("[EKF] playback done in ");
            EKF_PrintFloat(elapsed_ms, ' ');
            printf("ms. EST H:");
            EKF_PrintFloat(current.pos_z, ' ');
            printf("v:");
            EKF_PrintFloat(current.vel_z, ' ');
            printf("bias:[");
            EKF_PrintFloat(current.accel_bias[0], ',');
            EKF_PrintFloat(current.accel_bias[1], ',');
            EKF_PrintFloat(current.accel_bias[2], ']');
            printf("\r\n");

            // 2. Output parseable telemetry packets [TELE]
            printf("[TELE] pos:");
            EKF_PrintFloat(current.pos_x, ',');
            EKF_PrintFloat(current.pos_y, ',');
            EKF_PrintFloat(current.pos_z, ' ');
            printf("vel:");
            EKF_PrintFloat(current.vel_x, ',');
            EKF_PrintFloat(current.vel_y, ',');
            EKF_PrintFloat(current.vel_z, ' ');
            printf("q:");
            EKF_PrintFloat(current.q[0], ',');
            EKF_PrintFloat(current.q[1], ',');
            EKF_PrintFloat(current.q[2], ',');
            EKF_PrintFloat(current.q[3], '\r'); // carriage return + newline printed next
            printf("\n");
        }
    }
}

float EKF_GetCPUUsage(void) {
    return g_ekf_cpu_usage;
}

/* ============================================================
 *  EKF 靜態偏置與空中熱啟動狀態恢復
 * ============================================================ */

void EKF_SaveCalibrationToFlash(void)
{
#if FEATURE_FLASH
    FlashSysFlags_t sys_flags;
    // 1. 讀取既有旗標以保留其他欄位 (如重啟次數等)
    if (Flash_ReadSysFlags(&sys_flags) != W25QXX_OK) {
        memset(&sys_flags, 0, sizeof(FlashSysFlags_t));
    }

    // 2. 更新 EKF 校準參數
    sys_flags.calib.magic = 0xC0DEB1A5;
    sys_flags.calib.baro_launchpad = EKF_baro_launchpad;
    sys_flags.calib.accel_bias[0] = EKF_accel_bias[0];
    sys_flags.calib.accel_bias[1] = EKF_accel_bias[1];
    sys_flags.calib.accel_bias[2] = EKF_accel_bias[2];
    sys_flags.calib.gyro_bias[0] = EKF_gyro_bias[0];
    sys_flags.calib.gyro_bias[1] = EKF_gyro_bias[1];
    sys_flags.calib.gyro_bias[2] = EKF_gyro_bias[2];

    // 3. 讀取並更新地磁計硬鐵偏移
    const MMC5983_Data_t *mag = MMC5983_GetData();
    sys_flags.mag_offsets[0] = (float)mag->offset[0];
    sys_flags.mag_offsets[1] = (float)mag->offset[1];
    sys_flags.mag_offsets[2] = (float)mag->offset[2];

    // 4. 寫入 Flash Sector 0
    if (Flash_WriteSysFlags(&sys_flags) == W25QXX_OK) {
        printf("[EKF] Static Calibration Saved to Flash (Sector 0)!\r\n");
    } else {
        printf("[EKF] ERROR: Failed to save calibration to Flash!\r\n");
    }
#else
    printf("[EKF] Flash disabled, skip saving calibration.\r\n");
#endif
}

/* P0-x：校準政策 —— 上電必重校，Flash 僅作比對參考。
 * 原行為：讀到有效 Flash 校準即直接套用偏置並設 EKF_calibrated=1，跳過現場
 * 3 秒靜態校準 —— 陳舊的溫漂偏置直接上場（上次校準可能是數天前、不同溫度）。
 * 改為：mag 硬鐵偏移照常還原（不受溫漂影響、且電磁環境校準昂貴），但加計/
 * 陀螺偏置與 baro 基準只存入 EKF_flash_calib_* 參考值，不覆寫工作偏置、
 * 不設 EKF_calibrated —— 現場靜態校準照常執行，完成時與參考值比對偵測溫漂
 * （見 EKF_Task 校準完成區塊）。空中熱啟動（無法靜置重校）改呼叫
 * EKF_ApplyFlashCalibration() 真正套用。 */
void EKF_LoadCalibrationFromFlash(void)
{
#if FEATURE_FLASH
    FlashSysFlags_t sys_flags;
    if (Flash_ReadSysFlags(&sys_flags) == W25QXX_OK) {
        if (sys_flags.calib.magic == 0xC0DEB1A5) {
            /* 僅存入參考值（不覆寫工作偏置、不設 EKF_calibrated） */
            EKF_flash_calib_baro_launchpad = sys_flags.calib.baro_launchpad;
            EKF_flash_calib_accel_bias[0]  = sys_flags.calib.accel_bias[0];
            EKF_flash_calib_accel_bias[1]  = sys_flags.calib.accel_bias[1];
            EKF_flash_calib_accel_bias[2]  = sys_flags.calib.accel_bias[2];
            EKF_flash_calib_gyro_bias[0]   = sys_flags.calib.gyro_bias[0];
            EKF_flash_calib_gyro_bias[1]   = sys_flags.calib.gyro_bias[1];
            EKF_flash_calib_gyro_bias[2]   = sys_flags.calib.gyro_bias[2];
            EKF_flash_calib_valid          = 1;

            // 還原地磁計硬鐵偏移（經對齊區域陣列中轉，避免取 packed 成員位址）
            // mag 硬鐵偏移不受溫漂影響且需 8 字校準程序，維持直接套用。
            float mag_off[3] = { sys_flags.mag_offsets[0],
                                 sys_flags.mag_offsets[1],
                                 sys_flags.mag_offsets[2] };
            MMC5983_SetOffsets(mag_off);

            printf("[EKF] 已載入 Flash 校準作比對參考（上電仍執行現場靜態校準）。\r\n");
            printf("  -> Flash Launchpad Baro Alt: ");
            EKF_PrintFloat(EKF_flash_calib_baro_launchpad, ' ');
            printf("m\r\n");
            return;
        }
    }
    printf("[EKF] No valid static calibration found in Flash.\r\n");
#else
    printf("[EKF] Flash disabled, skip loading calibration from Flash.\r\n");
#endif
}

/* P0-x：把 Flash 校準參考值真正套用並標記校準完成。
 * 僅供 FEATURE_HOTSTART 空中恢復路徑使用 —— 空中無法靜置 3 秒重校，陳舊偏置
 * 仍優於全零偏置。地面正常開機一律走現場靜態校準，不得呼叫本函式。 */
void EKF_ApplyFlashCalibration(void)
{
    if (!EKF_flash_calib_valid) {
        printf("[EKF] [WARNING] No Flash calibration reference to apply (hot-restart with cold biases).\r\n");
        return;
    }
    EKF_baro_launchpad = EKF_flash_calib_baro_launchpad;
    EKF_accel_bias[0]  = EKF_flash_calib_accel_bias[0];
    EKF_accel_bias[1]  = EKF_flash_calib_accel_bias[1];
    EKF_accel_bias[2]  = EKF_flash_calib_accel_bias[2];
    EKF_gyro_bias[0]   = EKF_flash_calib_gyro_bias[0];
    EKF_gyro_bias[1]   = EKF_flash_calib_gyro_bias[1];
    EKF_gyro_bias[2]   = EKF_flash_calib_gyro_bias[2];
    EKF_calibrated     = 1;   // 熱啟動：跳過現場校準（空中無法靜置）
    printf("[EKF] Flash calibration APPLIED (hot-restart path).\r\n");
}

void EKF_HotRestartRestore(float last_altitude, float est_vel_z, const float *last_q)
{
    if (last_q == NULL) return;

    // 1. 注入高度與垂直速度
    EKF_x[0] = 0.0f; // 重置水平位置
    EKF_x[1] = 0.0f;
    EKF_x[2] = last_altitude;
    EKF_x[3] = 0.0f; // 重置水平速度
    EKF_x[4] = 0.0f;
    EKF_x[5] = est_vel_z;

    // 2. 注入最後記錄的姿態四元數
    EKF_q[0] = last_q[0];
    EKF_q[1] = last_q[1];
    EKF_q[2] = last_q[2];
    EKF_q[3] = last_q[3];

    // 3. 正規化四元數
    float norm = sqrtf(EKF_q[0]*EKF_q[0] + EKF_q[1]*EKF_q[1] + EKF_q[2]*EKF_q[2] + EKF_q[3]*EKF_q[3]);
    if (norm > 1e-6f) {
        EKF_q[0] /= norm;
        EKF_q[1] /= norm;
        EKF_q[2] /= norm;
        EKF_q[3] /= norm;
    } else {
        // Fallback
        EKF_q[0] = 1.0f;
        EKF_q[1] = 0.0f;
        EKF_q[2] = 0.0f;
        EKF_q[3] = 0.0f;
    }

    // 4. 重置協方差矩陣 P 為適當的空中初始不確定度
    memset(EKF_P, 0, sizeof(EKF_P));
    EKF_P[0][0] = 10.0f; // 東向位置
    EKF_P[1][1] = 10.0f; // 北向位置
    EKF_P[2][2] = 5.0f;  // 垂直高度
    EKF_P[3][3] = 2.0f;  // 東向速度
    EKF_P[4][4] = 2.0f;  // 北向速度
    EKF_P[5][5] = 1.0f;  // 垂直速度

    printf("[EKF] Hot-Restart State Injected!\r\n");
    printf("  -> Alt: ");
    EKF_PrintFloat(EKF_x[2], ' ');
    printf("VelZ: ");
    EKF_PrintFloat(EKF_x[5], '\n');
    printf("  -> Q: ");
    EKF_PrintFloat(EKF_q[0], ' ');
    EKF_PrintFloat(EKF_q[1], ' ');
    EKF_PrintFloat(EKF_q[2], ' ');
    EKF_PrintFloat(EKF_q[3], '\n');
}

void EKF_ResetCalibration(void)
{
    taskENTER_CRITICAL();
    EKF_calibrated = 0;
    EKF_calib_samples = 0;
    memset(EKF_accel_bias, 0, sizeof(EKF_accel_bias));
    memset(EKF_accel_sum, 0, sizeof(EKF_accel_sum));
    memset(EKF_gyro_bias, 0, sizeof(EKF_gyro_bias));
    memset(EKF_gyro_sum, 0, sizeof(EKF_gyro_sum));
    EKF_baro_launchpad = 0.0f;
    EKF_baro_sum = 0.0f;
    EKF_baro_samples = 0;
    taskEXIT_CRITICAL();
    
#if FEATURE_FLASH
    // Clear flash calibration magic number
    FlashSysFlags_t sys_flags;
    if (Flash_ReadSysFlags(&sys_flags) == W25QXX_OK) {
        sys_flags.calib.magic = 0;
        Flash_WriteSysFlags(&sys_flags);
    }
#endif
    
    printf("[EKF] Static calibration reset triggered!\r\n");
}

void EKF_SaveMagCalibration(float cx, float cy, float cz) {
#if FEATURE_FLASH
    FlashSysFlags_t sys_flags;
    // 1. 讀取既有旗標以保留其他欄位 (如重啟次數等)
    if (Flash_ReadSysFlags(&sys_flags) != W25QXX_OK) {
        memset(&sys_flags, 0, sizeof(FlashSysFlags_t));
    }
    
    // 2. 更新地磁偏置
    sys_flags.mag_offsets[0] = cx;
    sys_flags.mag_offsets[1] = cy;
    sys_flags.mag_offsets[2] = cz;
    
    // 3. 寫入 Flash Sector 0
    if (Flash_WriteSysFlags(&sys_flags) == W25QXX_OK) {
        // 套用到感測器
        float mag_off[3] = { cx, cy, cz };
        MMC5983_SetOffsets(mag_off);
        printf("[CAL] Mag hard-iron offset saved to Flash (x10): %d, %d, %d\r\n",
               (int)(cx * 10.0f), (int)(cy * 10.0f), (int)(cz * 10.0f));
    } else {
        printf("[CAL] ERROR: Failed to save mag offsets to Flash!\r\n");
    }
#else
    float mag_off[3] = { cx, cy, cz };
    MMC5983_SetOffsets(mag_off);
    printf("[CAL] Flash disabled, applied mag offsets in-memory (x10): %d, %d, %d\r\n",
           (int)(cx * 10.0f), (int)(cy * 10.0f), (int)(cz * 10.0f));
#endif
}
