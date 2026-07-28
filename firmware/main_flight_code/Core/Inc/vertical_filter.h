/*
 * vertical_filter.h — 垂直通道 3 狀態 Kalman 濾波器（純邏輯，header-only，host 可測）
 * ===========================================================================
 * Schultz 火箭高度計架構：狀態 x = [h, v, a]（相對高度 m、垂直速度 m/s、垂直
 * 加速度 m/s²），等加速度模型傳播 + baro / accel 兩路標量量測更新。與 6 狀態
 * EKF 相互獨立，作為開傘決策的輕量替代/對照鏈（board_config.h 的
 * FEATURE_VFILTER / FEATURE_VFILTER_FSM 控制接線，見 main.c FSM_Update）。
 *
 * 比照 lora_calc.h 模式：不依賴 HAL / RTOS（僅 stdint / string / math），
 * static inline，firmware 與 tests/test_vertical_filter.c 共用同一份。
 * 必須 header-only：CubeIDE 產生的 Debug/makefile 不會自動納入新 .c。
 *
 * 參數依據（由 tests/test_vertical_filter.c 五案例調校鎖定）：
 *   VF_R_BARO 0.36  — BMP388 高度噪聲 σ≈0.6m（與 EKF R_baro 同值）
 *   VF_R_ACC  4.0   — ADXL375 高G 換算垂直加速度 σ≈2 m/s²（含傾斜誤差餘裕）
 *   VF_Q_A    120.0 — 加速度隨機游走密度 (m/s²)²/s：需足以在燒完瞬間
 *                     （+50 → −12 m/s² 步階）數十 ms 內跟上，且 baro-only
 *                     模式仍能單靠氣壓觀測出 coast 段減速度（測試 [2]）
 *   VF_Q_H/Q_V      — 模型失配餘裕；取小值使地面/電梯段 K_h 足夠低，
 *                     單筆 3m 尖刺對 h 影響 <1m（測試 [5]）
 *
 * 涵蓋（tests/test_vertical_filter.c）：
 *   [1] 標稱火箭剖面：頂點時刻 ±0.15s、頂點高度 <3m
 *   [2] baro-only：頂點時刻 ±0.3s
 *   [3] baro 中斷 2s（predict+accel）：v 誤差 <3 m/s、恢復 5 步收斂
 *   [4] 創新值閘控：|y| > max(5σ, 10m) 拒收
 *   [5] 電梯剖面：不產生假下墜、尖刺影響 <1m
 */
#ifndef VERTICAL_FILTER_H
#define VERTICAL_FILTER_H

#include <stdint.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === 參數（調校依據見檔頭） === */
#define VF_R_BARO        0.36f    /* baro 高度量測噪聲變異數 (m²) */
#define VF_R_ACC         4.0f     /* 垂直加速度量測噪聲變異數 ((m/s²)²) */
#define VF_Q_H           0.01f    /* 高度過程噪聲密度 (m²/s) */
#define VF_Q_V           0.10f    /* 速度過程噪聲密度 ((m/s)²/s) */
#define VF_Q_A           120.0f   /* 加速度過程噪聲密度 ((m/s²)²/s) */
#define VF_GATE_SIGMA    5.0f     /* baro 創新值閘：5σ */
#define VF_GATE_MIN_M    10.0f    /* baro 創新值閘：絕對下限 (m) */
#define VF_P0_H          1.0f     /* 初始高度不確定度 (m²) */
#define VF_P0_V          2.0f     /* 初始速度不確定度 ((m/s)²) */
#define VF_P0_A          10.0f    /* 初始加速度不確定度 ((m/s²)²) */

typedef struct {
    float x[3];      /* [h, v, a]：相對高度 (m)、垂直速度 (m/s)、垂直加速度 (m/s²) */
    float P[3][3];   /* 狀態協方差 */
    uint8_t consec_gate_rejects; /* 連續閘扣拒收計數器（滿 10 週期自動重置對齊） */
} VFilter_t;

/* 讀值（inline 取代巨集，型別安全） */
static inline float vf_h(const VFilter_t *f) { return f->x[0]; }
static inline float vf_v(const VFilter_t *f) { return f->x[1]; }
static inline float vf_a(const VFilter_t *f) { return f->x[2]; }

/** @brief 初始化：狀態清零，P 對角 [VF_P0_H, VF_P0_V, VF_P0_A]。 */
static inline void vf_init(VFilter_t *f)
{
    memset(f, 0, sizeof(*f));
    f->P[0][0] = VF_P0_H;
    f->P[1][1] = VF_P0_V;
    f->P[2][2] = VF_P0_A;
}

/* 內部：P 對稱化（浮點累積誤差防護，兩路標量更新後皆呼叫）。 */
static inline void vf__symmetrize(VFilter_t *f)
{
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            float m = 0.5f * (f->P[i][j] + f->P[j][i]);
            f->P[i][j] = m;
            f->P[j][i] = m;
        }
    }
}

/* 內部：標量量測更新共用核心（H = e_idx，創新值 y、創新協方差 S 已由呼叫端算好）。
 * Joseph form：P' = (I−KH)P(I−KH)ᵀ + K·R·Kᵀ，數值穩健；更新後對稱化。 */
static inline void vf__update_scalar(VFilter_t *f, int idx, float y, float S, float R)
{
    float invS = 1.0f / S;
    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = f->P[i][idx] * invS;
    }
    for (int i = 0; i < 3; i++) {
        f->x[i] += K[i] * y;
    }

    /* P_prime = (I − K·H) · P */
    float P_prime[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_prime[i][j] = f->P[i][j] - K[i] * f->P[idx][j];
        }
    }
    /* P = P_prime·(I − K·H)ᵀ + K·R·Kᵀ */
    for (int i = 0; i < 3; i++) {
        float P_prime_ii = P_prime[i][idx];
        for (int j = 0; j < 3; j++) {
            f->P[i][j] = P_prime[i][j] - P_prime_ii * K[j] + K[i] * R * K[j];
        }
    }
    vf__symmetrize(f);
}

/**
 * @brief 等加速度模型傳播（F = [[1,dt,dt²/2],[0,1,dt],[0,0,1]]）。
 * dt 非法（<=0 或 >0.5s，如首步 / tick 迴繞 / 排程延遲）直接略過本步。
 * Q 以密度 × dt 加在對角（主要在 a：加速度隨機游走驅動整個模型）。
 */
static inline void vf_predict(VFilter_t *f, float dt)
{
    if (dt <= 0.0f || dt > 0.5f) return;

    const float d2 = 0.5f * dt * dt;

    /* 狀態傳播 */
    f->x[0] += f->x[1] * dt + f->x[2] * d2;
    f->x[1] += f->x[2] * dt;
    /* x[2]（加速度）維持：隨機游走模型 */

    /* P = F·P·Fᵀ + Q：先 A = F·P，再 P = A·Fᵀ（F 上三角，逐列展開） */
    float A[3][3];
    for (int j = 0; j < 3; j++) {
        A[0][j] = f->P[0][j] + dt * f->P[1][j] + d2 * f->P[2][j];
        A[1][j] = f->P[1][j] + dt * f->P[2][j];
        A[2][j] = f->P[2][j];
    }
    for (int i = 0; i < 3; i++) {
        f->P[i][0] = A[i][0] + dt * A[i][1] + d2 * A[i][2];
        f->P[i][1] = A[i][1] + dt * A[i][2];
        f->P[i][2] = A[i][2];
    }

    f->P[0][0] += VF_Q_H * dt;
    f->P[1][1] += VF_Q_V * dt;
    f->P[2][2] += VF_Q_A * dt;

    vf__symmetrize(f);
}

/**
 * @brief baro 相對高度量測更新（H = [1,0,0]）。
 * 創新值閘控：|y| > max(VF_GATE_SIGMA·√S, VF_GATE_MIN_M) 拒收 —— 單筆壞值
 * （SPI 位翻轉、氣壓瞬變）不得直接進濾波器。
 * @return 1 = 已接受並更新；0 = 被閘拒（狀態未動）。
 */
static inline int vf_update_baro(VFilter_t *f, float h_meas)
{
    float y = h_meas - f->x[0];
    float S = f->P[0][0] + VF_R_BARO;
    if (S < 1e-9f) return 0;

    float gate = VF_GATE_SIGMA * sqrtf(S);
    if (gate < VF_GATE_MIN_M) gate = VF_GATE_MIN_M;
    if (fabsf(y) > gate) {
        if (f->consec_gate_rejects < 255U) f->consec_gate_rejects++;
        if (f->consec_gate_rejects < 10U) {
            return 0; /* 偶發單筆壞值：拒收 */
        }
        /* 連續 10 週期 (100ms) 拒收：極可能是狀態發散或大幅位移，強制重估對齊 */
        f->x[0] = h_meas;
        f->consec_gate_rejects = 0U;
        return 1;
    }

    f->consec_gate_rejects = 0U;
    vf__update_scalar(f, 0, y, S, VF_R_BARO);
    return 1;
}

/**
 * @brief 垂直加速度量測更新（H = [0,0,1]，無閘控）。
 * a_meas 為「運動加速度」(m/s²)：呼叫端以 (a_z_g − 1g) 換算（高G軸向近垂直假設）。
 */
static inline void vf_update_accel(VFilter_t *f, float a_meas)
{
    float y = a_meas - f->x[2];
    float S = f->P[2][2] + VF_R_ACC;
    if (S < 1e-9f) return;
    vf__update_scalar(f, 2, y, S, VF_R_ACC);
}

#ifdef __cplusplus
}
#endif

#endif /* VERTICAL_FILTER_H */
