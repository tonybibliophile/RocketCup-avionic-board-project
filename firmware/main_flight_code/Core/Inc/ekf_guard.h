/*
 * ekf_guard.h — EKF 防護最小集（P0-C，純 header-only，host 可測）
 * ===========================================================================
 * 修復 改進計劃.md R3：EKF 原本毫無防護 —— 協方差無上界、baro 創新值無閘控
 * （±10km 跳變照收）、無 NaN 偵測、無狀態理智界限。任一感測器壞值即可讓
 * 估計發散，而 FSM 開傘決策完全依賴 h_est/v_est。
 *
 * 本檔僅含純函式與參數（不依賴 HAL / RTOS），由 tests/test_ekf_guard.c 驗證；
 * ekf.c 於三個掛載點呼叫：
 *   1. EKF_UpdateBaroDelayed 入口  → ekf_guard_baro_accept()（創新值閘控）
 *   2. EKF_Predict 收尾           → ekf_guard_clamp_P()（協方差對角夾限）
 *   3. EKF_Task 每 buffer 一次     → ekf_guard_scan_nan() + ekf_guard_clamp_state()
 *
 * 健康位由 ekf.c 維護，FSM 經 EKF_GetHealthBits()==0 判定 healthy，
 * unhealthy 時 fsm.c 切換 raw-baro 降級開傘鏈（見 fsm.c P0-C 註解）。
 */
#ifndef EKF_GUARD_H
#define EKF_GUARD_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === 健康位（EKF_GetHealthBits()；0 = 全健康） === */
#define EKF_HB_BARO_DIVERGE   0x01U  /* baro 創新值連續拒收 ≥1s：估計已發散或 baro 已壞 */
#define EKF_HB_STATE_OOB      0x02U  /* 狀態超出理智界限（曾被夾限，1s 內未再越界則解除） */
#define EKF_HB_NAN            0x04U  /* 狀態/協方差/四元數出現 NaN/Inf，已重建（黏滯 5s） */
#define EKF_HB_BARO_TIMEOUT   0x08U  /* >500ms 無被接受的 baro 量測（含 baro 斷流） */

/* === 參數（理由見 改進計劃.md P0-C 表） === */
#define EKF_GUARD_BARO_GATE_SIGMA  5.0f     /* 5σ 標準卡方閘 */
#define EKF_GUARD_BARO_GATE_MIN_M  25.0f    /* 絕對下限：防收斂後(5σ≈3m)誤拒真實氣壓暫態 */
#define EKF_GUARD_BARO_REJECT_N    200U     /* 連續拒收 200 次（BMP388 現為 50Hz ODR，@50Hz ≈4.0s；
                                              * 原註解假設的 200Hz 已隨 S4 改動不再成立，此值本身
                                              * 是否要retune 待 S7 實測 σ 後一併決定，暫不變動） → DIVERGE */
#define EKF_GUARD_BARO_TIMEOUT_MS  500U     /* 無接受 baro 逾時 */
#define EKF_GUARD_P_POS_MAX        1.0e4f   /* 位置共變異數上限（σ=100m） */
#define EKF_GUARD_P_VEL_MAX        2.5e3f   /* 速度共變異數上限（σ=50m/s） */
#define EKF_GUARD_P_MIN            1.0e-6f  /* 對角下限（防奇異） */
#define EKF_GUARD_POS_Z_MIN       -100.0f   /* 本箭剖面（數百米頂點）放大一個數量級 */
#define EKF_GUARD_POS_Z_MAX        2000.0f
#define EKF_GUARD_VEL_Z_MIN       -200.0f
#define EKF_GUARD_VEL_Z_MAX        350.0f
#define EKF_GUARD_POS_H_MAX        5000.0f  /* 水平位置 |E|,|N| 上限 */
#define EKF_GUARD_VEL_H_MAX        200.0f   /* 水平速度上限 */
#define EKF_GUARD_OOB_CLEAR_MS     1000U    /* 回界後 1s 解除 OOB 位 */
#define EKF_GUARD_NAN_STICKY_MS    5000U    /* NaN 重建後黏滯 5s（提示地面站） */
#define EKF_GUARD_DIVERGE_RESET_MS 3000U    /* P1：DIVERGE 持續 3s → 垂直通道自救重置 */

/* baro 創新值閘控：|y| > max(5·sqrt(S), 25m) 拒收（回傳 0）。
 * 場測期間曾改為「永遠接受」（電梯測試以氣壓為唯一基準）；恢復飛行版行為 ——
 * 單筆 ±10km 壞值不得直接餵進濾波器，連續拒收由 ekf.c 的 DIVERGE 機制接手
 * （P1 垂直通道自救亦依賴本閘的拒收計數，停用時整條鏈為死碼）。 */
static inline uint8_t ekf_guard_baro_accept(float y, float S)
{
    float gate = EKF_GUARD_BARO_GATE_SIGMA * sqrtf(S);
    if (gate < EKF_GUARD_BARO_GATE_MIN_M) {
        gate = EKF_GUARD_BARO_GATE_MIN_M;   /* 絕對下限：防收斂後(5σ≈3m)誤拒真實氣壓暫態 */
    }
    return (fabsf(y) <= gate) ? 1U : 0U;
}

/* 協方差對角夾限。回傳 1 = 有夾限發生（僅診斷用）。 */
static inline uint8_t ekf_guard_clamp_P(float P[6][6])
{
    uint8_t clamped = 0U;
    for (int i = 0; i < 6; i++) {
        float max = (i < 3) ? EKF_GUARD_P_POS_MAX : EKF_GUARD_P_VEL_MAX;
        if (P[i][i] > max)            { P[i][i] = max;              clamped = 1U; }
        if (P[i][i] < EKF_GUARD_P_MIN){ P[i][i] = EKF_GUARD_P_MIN;  clamped = 1U; }
    }
    return clamped;
}

/* 狀態理智界限夾限（x = [pE,pN,pU,vE,vN,vU]）。回傳 1 = 有夾限發生。 */
static inline uint8_t ekf_guard_clamp_state(float x[6])
{
    uint8_t clamped = 0U;
    /* 水平位置 */
    for (int i = 0; i < 2; i++) {
        if (x[i] >  EKF_GUARD_POS_H_MAX) { x[i] =  EKF_GUARD_POS_H_MAX; clamped = 1U; }
        if (x[i] < -EKF_GUARD_POS_H_MAX) { x[i] = -EKF_GUARD_POS_H_MAX; clamped = 1U; }
    }
    /* 垂直位置 */
    if (x[2] > EKF_GUARD_POS_Z_MAX) { x[2] = EKF_GUARD_POS_Z_MAX; clamped = 1U; }
    if (x[2] < EKF_GUARD_POS_Z_MIN) { x[2] = EKF_GUARD_POS_Z_MIN; clamped = 1U; }
    /* 水平速度 */
    for (int i = 3; i < 5; i++) {
        if (x[i] >  EKF_GUARD_VEL_H_MAX) { x[i] =  EKF_GUARD_VEL_H_MAX; clamped = 1U; }
        if (x[i] < -EKF_GUARD_VEL_H_MAX) { x[i] = -EKF_GUARD_VEL_H_MAX; clamped = 1U; }
    }
    /* 垂直速度 */
    if (x[5] > EKF_GUARD_VEL_Z_MAX) { x[5] = EKF_GUARD_VEL_Z_MAX; clamped = 1U; }
    if (x[5] < EKF_GUARD_VEL_Z_MIN) { x[5] = EKF_GUARD_VEL_Z_MIN; clamped = 1U; }
    return clamped;
}

/* NaN/Inf 掃描（狀態、協方差對角、四元數）。回傳 1 = 發現非有限值，必須重建。 */
static inline uint8_t ekf_guard_scan_nan(const float x[6], const float P[6][6], const float q[4])
{
    for (int i = 0; i < 6; i++) {
        if (!isfinite(x[i]))    return 1U;
        if (!isfinite(P[i][i])) return 1U;
    }
    for (int i = 0; i < 4; i++) {
        if (!isfinite(q[i]))    return 1U;
    }
    return 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* EKF_GUARD_H */
