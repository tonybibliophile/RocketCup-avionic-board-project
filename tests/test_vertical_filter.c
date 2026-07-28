/*
 * test_vertical_filter.c — 垂直通道 3 狀態 Kalman 濾波器單元測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * 決定性偽噪聲：線性同餘 LCG（固定種子），每次執行結果完全相同。
 *
 * 涵蓋（vertical_filter.h）：
 *   [1] 標稱火箭剖面（100Hz 解析真值 + 噪聲/偏置）：頂點時刻 ±0.15s、高度誤差 <3m
 *   [2] baro-only（不餵 accel）：頂點時刻 ±0.3s
 *   [3] baro 中斷（頂點前後各 1s，僅 predict+accel）：v 誤差 <3 m/s、恢復 5 步收斂
 *   [4] 創新值閘控：單筆 +500m 壞值拒收（回傳 0 且 h_est 幾乎不動）
 *   [5] 電梯剖面（1.5 m/s 上升 + 尖刺）：不產生假下墜、尖刺影響 <1m
 *
 * 最終參數依據（vertical_filter.h 檔頭同步記載）：
 *   VF_Q_A=120：baro-only 模式（案例[2]）需單靠氣壓觀測 coast 段 −12 m/s² 減速，
 *   Q_A 過小則 a 估計跟不上、v 過零時刻嚴重滯後；Q_A 過大則電梯尖刺（案例[5]）
 *   影響超標。120 (m/s²)²/s 在兩端皆有餘裕。VF_Q_H=0.01 / VF_Q_V=0.1 取小，
 *   使穩態 K_h 低（單筆 3m 尖刺對 h 影響 <1m）。R 沿用感測器實測噪聲。
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "vertical_filter.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

/* === 決定性偽噪聲：LCG（Numerical Recipes 常數），固定種子 === */
static uint32_t g_lcg = 0x12345678u;
static void lcg_seed(uint32_t s) { g_lcg = s; }
static float lcg_uniform(void) {              /* [-1, 1] 均勻 */
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return ((float)(g_lcg >> 8) / 8388608.0f) - 1.0f;   /* 24-bit 精度 */
}
/* 均勻 [-1,1] 標準差 = 1/√3 → 乘 √3 得 σ=1；再乘目標 σ */
static float noise(float sigma) { return lcg_uniform() * 1.7320508f * sigma; }

/* === 標稱火箭剖面解析真值（100Hz）===
 *   pad   [0, 2s)      ：h=0, v=0, a=0
 *   boost [2, 3.5s)    ：a=+50 → v=50τ, h=25τ²（τ=t−2）
 *   coast [3.5s, ...)  ：a=−12 → v=75−12τ, h=56.25+75τ−6τ²（τ=t−3.5）
 *   頂點  t*=9.75s（v 過零），h*=290.625m
 *   下降  ：a=−12 續至 v=−20（t≈11.417s）後等速 −20 m/s
 */
#define ROCKET_T_APOGEE   9.75f
#define ROCKET_H_APOGEE   290.625f

static void rocket_truth(float t, float *h, float *v, float *a) {
    if (t < 2.0f)        { *h = 0.0f; *v = 0.0f; *a = 0.0f; return; }
    if (t < 3.5f) {
        float tau = t - 2.0f;
        *a = 50.0f; *v = 50.0f * tau; *h = 25.0f * tau * tau;
        return;
    }
    float tau = t - 3.5f;
    float v_c = 75.0f - 12.0f * tau;
    if (v_c >= -20.0f) {
        *a = -12.0f; *v = v_c; *h = 56.25f + 75.0f * tau - 6.0f * tau * tau;
        return;
    }
    /* v = −20 於 τ0 = 95/12；此後等速下降 */
    float tau0 = 95.0f / 12.0f;
    float h0   = 56.25f + 75.0f * tau0 - 6.0f * tau0 * tau0;
    *a = 0.0f; *v = -20.0f; *h = h0 - 20.0f * (tau - tau0);
}

#define DT        0.01f
#define ACC_BIAS  0.5f     /* accel 量測固定偏置 (m/s²) */
#define BARO_SIG  0.6f     /* baro 量測噪聲 σ (m) */
#define ACC_SIG   1.0f     /* accel 量測噪聲 σ (m/s²) */

/* ---------------------------------------------------------------- */
static void test_nominal_rocket(void) {
    printf("[1] 標稱火箭剖面（baro + accel 融合）\n");
    lcg_seed(0x12345678u);
    VFilter_t f; vf_init(&f);

    float t_zero_cross = -1.0f;   /* v_est 由正轉負的時刻 */
    float h_at_cross   = 0.0f;
    float prev_v = 0.0f;

    for (int k = 1; k <= 1300; k++) {          /* 0 → 13s */
        float t = (float)k * DT;
        float h, v, a;
        rocket_truth(t, &h, &v, &a);

        vf_predict(&f, DT);
        vf_update_baro(&f, h + noise(BARO_SIG));
        vf_update_accel(&f, a + ACC_BIAS + noise(ACC_SIG));

        if (t > 5.0f && t_zero_cross < 0.0f && prev_v > 0.0f && vf_v(&f) <= 0.0f) {
            t_zero_cross = t;
            h_at_cross   = vf_h(&f);
        }
        prev_v = vf_v(&f);
    }

    printf("      v 過零時刻 %.3fs（真值 %.2fs），頂點高度 %.2fm（真值 %.3fm）\n",
           (double)t_zero_cross, (double)ROCKET_T_APOGEE,
           (double)h_at_cross, (double)ROCKET_H_APOGEE);
    check("v_est 過零時刻與真頂點差 ≤ ±0.15s",
          t_zero_cross > 0.0f && fabsf(t_zero_cross - ROCKET_T_APOGEE) <= 0.15f);
    check("頂點高度誤差 < 3m", fabsf(h_at_cross - ROCKET_H_APOGEE) < 3.0f);
}

/* ---------------------------------------------------------------- */
static void test_baro_only(void) {
    printf("[2] baro-only（不餵 accel）\n");
    lcg_seed(0xBEEF0001u);
    VFilter_t f; vf_init(&f);

    float t_zero_cross = -1.0f;
    float prev_v = 0.0f;

    for (int k = 1; k <= 1300; k++) {
        float t = (float)k * DT;
        float h, v, a;
        rocket_truth(t, &h, &v, &a);

        vf_predict(&f, DT);
        vf_update_baro(&f, h + noise(BARO_SIG));
        /* 無 accel 更新：a 狀態全靠 baro 觀測 + Q_A 隨機游走 */

        if (t > 5.0f && t_zero_cross < 0.0f && prev_v > 0.0f && vf_v(&f) <= 0.0f) {
            t_zero_cross = t;
        }
        prev_v = vf_v(&f);
    }

    printf("      v 過零時刻 %.3fs（真值 %.2fs）\n",
           (double)t_zero_cross, (double)ROCKET_T_APOGEE);
    check("baro-only：頂點時刻差 ≤ ±0.3s",
          t_zero_cross > 0.0f && fabsf(t_zero_cross - ROCKET_T_APOGEE) <= 0.3f);
}

/* ---------------------------------------------------------------- */
static void test_baro_outage(void) {
    printf("[3] baro 中斷（頂點前後各 1s，僅 predict + accel）\n");
    lcg_seed(0xCAFE0002u);
    VFilter_t f; vf_init(&f);

    const float t_out_lo = ROCKET_T_APOGEE - 1.0f;   /* 8.75s */
    const float t_out_hi = ROCKET_T_APOGEE + 1.0f;   /* 10.75s */
    float max_v_err_outage = 0.0f;
    int   resumed_steps = -1;      /* 恢復後第幾步重新收斂（|h 誤差|<1.5m） */
    int   steps_since_resume = 0;

    for (int k = 1; k <= 1300; k++) {
        float t = (float)k * DT;
        float h, v, a;
        rocket_truth(t, &h, &v, &a);
        int in_outage = (t >= t_out_lo && t < t_out_hi);

        vf_predict(&f, DT);
        if (!in_outage) {
            vf_update_baro(&f, h + noise(BARO_SIG));
        }
        vf_update_accel(&f, a + ACC_BIAS + noise(ACC_SIG));

        if (in_outage) {
            float e = fabsf(vf_v(&f) - v);
            if (e > max_v_err_outage) max_v_err_outage = e;
        }
        if (t >= t_out_hi && resumed_steps < 0) {
            steps_since_resume++;
            if (fabsf(vf_h(&f) - h) < 1.5f) resumed_steps = steps_since_resume;
        }
    }

    printf("      中斷期 max |v 誤差| = %.2f m/s，恢復後 %d 步收斂\n",
           (double)max_v_err_outage, resumed_steps);
    check("中斷期間 v 誤差 < 3 m/s", max_v_err_outage < 3.0f);
    check("恢復後 5 步內重新收斂（|h 誤差|<1.5m）",
          resumed_steps >= 1 && resumed_steps <= 5);
}

/* ---------------------------------------------------------------- */
static void test_innovation_gate(void) {
    printf("[4] 創新值閘控（單筆 +500m 壞值）\n");
    lcg_seed(0xDEAD0003u);
    VFilter_t f; vf_init(&f);

    /* 地面靜置 2s 收斂 */
    for (int k = 0; k < 200; k++) {
        vf_predict(&f, DT);
        vf_update_baro(&f, noise(BARO_SIG));
        vf_update_accel(&f, noise(ACC_SIG));
    }
    float h_before = vf_h(&f);

    vf_predict(&f, DT);
    int accepted = vf_update_baro(&f, 500.0f);   /* SPI 位翻轉級壞值 */

    check("+500m 壞值回傳 0（拒收）", accepted == 0);
    check("拒收後 h_est 移動 < 0.5m", fabsf(vf_h(&f) - h_before) < 0.5f);
}

/* ---------------------------------------------------------------- */
static void test_elevator(void) {
    printf("[5] 電梯剖面（1.5 m/s 上升 + 單筆 ±3m 尖刺）\n");
    lcg_seed(0xE1E70004u);
    VFilter_t f; vf_init(&f);

    /* 靜置 1s → 等速上升 1.5 m/s（電梯加速段短，簡化為 0.5s 線性加速） */
    float min_v_ascent = 1e9f;
    float spike_h_delta_pos = 0.0f, spike_h_delta_neg = 0.0f;

    for (int k = 1; k <= 2000; k++) {            /* 0 → 20s */
        float t = (float)k * DT;
        float h, v, a;
        if (t < 1.0f)        { h = 0.0f; v = 0.0f; a = 0.0f; }
        else if (t < 1.5f)   { a = 3.0f;  v = 3.0f * (t - 1.0f); h = 1.5f * (t - 1.0f) * (t - 1.0f); }
        else                 { a = 0.0f;  v = 1.5f; h = 0.75f + 1.5f * (t - 1.5f); }

        float baro = h + noise(0.3f);            /* 電梯井內氣壓噪聲 σ0.3m */
        /* 單筆尖刺：t=8s 給 +3m、t=14s 給 −3m */
        if (k == 800)  baro = h + 3.0f;
        if (k == 1400) baro = h - 3.0f;

        float h_pre = vf_h(&f);
        vf_predict(&f, DT);
        float h_pred = vf_h(&f);
        (void)h_pre;
        vf_update_baro(&f, baro);
        vf_update_accel(&f, a + noise(0.3f));    /* 電梯 accel 噪聲小 */

        if (k == 800)  spike_h_delta_pos = fabsf(vf_h(&f) - h_pred);
        if (k == 1400) spike_h_delta_neg = fabsf(vf_h(&f) - h_pred);

        /* 等速上升段（含尖刺當步與其後）：v_est 不得為假下墜 */
        if (t >= 2.0f && vf_v(&f) < min_v_ascent) min_v_ascent = vf_v(&f);
    }

    printf("      上升段 min v_est = %.2f m/s；尖刺對 h 影響 +%.2f / −%.2f m\n",
           (double)min_v_ascent, (double)spike_h_delta_pos, (double)spike_h_delta_neg);
    check("上升期間 v_est 不得 < −0.5（不產生假下墜）", min_v_ascent >= -0.5f);
    check("±3m 尖刺對 h 影響 < 1m",
          spike_h_delta_pos < 1.0f && spike_h_delta_neg < 1.0f);
}

/* ---------------------------------------------------------------- */
int main(void) {
    printf("=== test_vertical_filter：垂直通道 3 狀態 Kalman（Schultz 架構） ===\n");
    test_nominal_rocket();
    test_baro_only();
    test_baro_outage();
    test_innovation_gate();
    test_elevator();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
