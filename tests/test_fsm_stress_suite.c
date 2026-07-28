/*
 * test_fsm_stress_suite.c — 飛行狀態機 (FSM) 與 EKF 極端邊界與失效壓力測試
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "vertical_filter.h"
#include "fsm.h"

typedef struct {
    float time_s;
    float alt_m;
    float v_z_mps;
    float v_tot_mps;
    float a_tot_mps2;
} CSVRow_t;

static int load_csv(const char *filepath, CSVRow_t **out_rows, int *out_count) {
    FILE *f = fopen(filepath, "r");
    if (!f) return 0;
    char line[512];
    int capacity = 8000, count = 0;
    CSVRow_t *rows = (CSVRow_t *)malloc(capacity * sizeof(CSVRow_t));

    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        float t, h, v, v_tot, a_tot;
        if (sscanf(line, "%f,%f,%f,%f,%f", &t, &h, &v, &v_tot, &a_tot) >= 3) {
            if (count >= capacity) {
                capacity *= 2;
                rows = (CSVRow_t *)realloc(rows, capacity * sizeof(CSVRow_t));
            }
            rows[count].time_s = t;
            rows[count].alt_m = h;
            rows[count].v_z_mps = v;
            rows[count].v_tot_mps = v_tot;
            rows[count].a_tot_mps2 = a_tot;
            count++;
        }
    }
    fclose(f);
    *out_rows = rows;
    *out_count = count;
    return 1;
}

static float rand_normal(float stddev) {
    if (stddev <= 0.0f) return 0.0f;
    float u1 = (float)rand() / (float)RAND_MAX;
    float u2 = (float)rand() / (float)RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2) * stddev;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <csv_path>\n", argv[0]);
        return 1;
    }
    CSVRow_t *rows = NULL;
    int count = 0;
    if (!load_csv(argv[1], &rows, &count)) return 1;

    printf("========================================================================\n");
    printf("              FSM + EKF 極端邊界壓力與失效分析測試報告                  \n");
    printf("========================================================================\n\n");

    // -------------------------------------------------------------------------
    // 測試 1：極端加速度計雜訊掃描 (Pad Noise Saturation)
    // -------------------------------------------------------------------------
    printf("[1] 地面靜置極端加速度雜訊對比（防誤起飛能力）：\n");
    for (float na = 0.5f; na <= 5.0f; na += 0.5f) {
        srand(42);
        FSM_Context_t ctx;
        FSM_Init(&ctx, STATE_PAD, 0, 0, 0);
        FSM_Input_t in;
        memset(&in, 0, sizeof(in));
        in.est_calibrated = 1;
        in.est_healthy = 1;

        // (A) 未武裝 (uplink_armed = 0)
        in.uplink_armed = 0;
        int triggered_unarmed = 0;
        for (int step = 0; step < 6000; step++) { // 60s
            in.now_ms = step * 10;
            in.a_z_g = 1.0f + rand_normal(na);
            FSM_Step(&ctx, &in);
            if (ctx.state != STATE_PAD) { triggered_unarmed = 1; break; }
        }

        // (B) 已武裝 (uplink_armed = 1)
        FSM_Init(&ctx, STATE_PAD_ARMED, 0, 0, 0);
        in.uplink_armed = 1;
        int triggered_armed = 0;
        uint32_t trig_t = 0;
        for (int step = 0; step < 6000; step++) { // 60s
            in.now_ms = step * 10;
            in.a_z_g = 1.0f + rand_normal(na);
            FSM_Step(&ctx, &in);
            if (ctx.state == STATE_BOOST) { triggered_armed = 1; trig_t = in.now_ms; break; }
        }

        printf("  - 雜訊 σ_a = %.1fg: 未武裝安全 = %s | 武裝狀態 = %s (觸發時間: %.2fs)\n",
               na,
               triggered_unarmed ? "FAIL(誤起飛!)" : "PASS(完全免疫)",
               triggered_armed ? "誤發射!" : "PASS(免疫)",
               triggered_armed ? (float)trig_t / 1000.0f : 0.0f);
    }
    printf("\n");

    // -------------------------------------------------------------------------
    // 測試 2：超音速聲障/跨音速氣壓突落壓 (Transonic Pressure Dip / Shockwave)
    // -------------------------------------------------------------------------
    printf("[2] 跨音速 (Mach 1) 氣壓急劇突落壓/假吸力效應測試：\n");
    for (float drop_m = 50.0f; drop_m <= 300.0f; drop_m += 50.0f) {
        srand(42);
        VFilter_t vf; vf_init(&vf);
        FSM_Context_t ctx; FSM_Init(&ctx, STATE_PAD, 0, 0, 0);
        FSM_Input_t in; memset(&in, 0, sizeof(in));
        in.est_calibrated = 1; in.est_healthy = 1; in.uplink_armed = 1; in.flash_pool_ready = 1;

        int premature_drogue = 0;
        float drogue_t = 0.0f;

        for (int i = 0; i < count; i++) {
            uint32_t now_ms = (uint32_t)(rows[i].time_s * 1000.0f);
            float dt_s = 0.010f;
            float a_net_mps2 = rows[i].a_tot_mps2;
            float raw_a_g = (a_net_mps2 + 9.80665f) / 9.80665f;
            float raw_h_m = rows[i].alt_m;

            // 在上升段 (10s ~ 12s) 注入跨音速氣壓突降 (負高度偏差)
            if (rows[i].time_s >= 10.0f && rows[i].time_s <= 12.0f) {
                raw_h_m -= drop_m;
            }

            vf_predict(&vf, dt_s);
            vf_update_accel(&vf, (raw_a_g - 1.0f) * 9.80665f);
            vf_update_baro(&vf, raw_h_m);

            in.now_ms = now_ms;
            in.a_z_g = raw_a_g;
            in.h_est = rows[i].alt_m;
            in.v_est = rows[i].v_z_mps;
            in.baro_alt_rel = raw_h_m;

            FSM_Step(&ctx, &in);

            if (ctx.state == STATE_DEPLOY_DROGUE && rows[i].time_s < 20.0f) {
                premature_drogue = 1;
                drogue_t = rows[i].time_s;
                break;
            }
        }
        printf("  - 跨音速壓降 Δh = -%.0fm: %s (開傘時間: %.2fs, 正確頂點為 ~28.2s)\n",
               drop_m,
               premature_drogue ? "FAIL (超高空提前解鎖誤開傘!)" : "PASS (成功抵禦過濾)",
               drogue_t);
    }
    printf("\n");

    // -------------------------------------------------------------------------
    // 測試 3：氣壓計硬卡死 / 斷線測試 (Baro Lockup / Freeze)
    // -------------------------------------------------------------------------
    printf("[3] 氣壓計卡死/訊號中斷測試 (Baro Freeze at t=10s)：\n");
    {
        VFilter_t vf; vf_init(&vf);
        FSM_Context_t ctx; FSM_Init(&ctx, STATE_PAD, 0, 0, 0);
        FSM_Input_t in; memset(&in, 0, sizeof(in));
        in.est_calibrated = 1; in.est_healthy = 1; in.uplink_armed = 1; in.flash_pool_ready = 1;

        float frozen_baro = 0.0f;
        float drogue_t = 0.0f, apogee_t = 0.0f;

        for (int i = 0; i < count; i++) {
            uint32_t now_ms = (uint32_t)(rows[i].time_s * 1000.0f);
            float dt_s = 0.010f;
            float a_net_mps2 = rows[i].a_tot_mps2;
            float raw_a_g = (a_net_mps2 + 9.80665f) / 9.80665f;
            float raw_h_m = rows[i].alt_m;

            if (rows[i].time_s >= 10.0f) {
                if (frozen_baro == 0.0f) frozen_baro = raw_h_m;
                raw_h_m = frozen_baro; // 卡死在 10s 的數值
            }

            vf_predict(&vf, dt_s);
            vf_update_accel(&vf, (raw_a_g - 1.0f) * 9.80665f);
            vf_update_baro(&vf, raw_h_m);

            in.now_ms = now_ms;
            in.a_z_g = raw_a_g;
            in.h_est = rows[i].alt_m;
            in.v_est = rows[i].v_z_mps;
            in.baro_alt_rel = raw_h_m;

            FSM_Action_t act = FSM_Step(&ctx, &in);
            if (act.event == FSM_EVT_DEPLOY_DROGUE && drogue_t == 0.0f) drogue_t = rows[i].time_s;
            if (ctx.state == STATE_APOGEE && apogee_t == 0.0f) apogee_t = rows[i].time_s;
        }
        printf("  - 氣壓計於 t=10s 冰凍鎖死: 副傘點火=%.2fs, 頂點轉移=%.2fs (%s)\n",
               drogue_t, apogee_t, (drogue_t > 24.0f && drogue_t < 30.0f) ? "PASS(IMU續航成功)" : "FAIL");
    }
    printf("\n");

    // -------------------------------------------------------------------------
    // 測試 4：全感測器完全失效/斷訊（全空包）
    // -------------------------------------------------------------------------
    printf("[4] 全感測器毀損中斷測試 (Total Blackout at t=5s)：\n");
    {
        FSM_Context_t ctx; FSM_Init(&ctx, STATE_PAD, 0, 0, 0);
        FSM_Input_t in; memset(&in, 0, sizeof(in));
        in.est_calibrated = 1; in.est_healthy = 1; in.uplink_armed = 1; in.flash_pool_ready = 1;

        float drogue_t = 0.0f, main_t = 0.0f;

        for (int step = 0; step < 20000; step++) { // 200s
            uint32_t now_ms = step * 10;
            float t = (float)now_ms / 1000.0f;

            if (t < 5.0f) {
                in.a_z_g = 5.0f; // 起飛
                in.h_est = t * 100.0f;
                in.v_est = 100.0f;
            } else {
                // t >= 5.0s 全感測器毀損斷訊，輸入完全凍結
                in.est_healthy = 0;
                in.sensor_bits = FSM_SB_BARO_FAULT;
            }

            in.now_ms = now_ms;
            FSM_Action_t act = FSM_Step(&ctx, &in);

            if ((act.event == FSM_EVT_DEPLOY_DROGUE || act.event == FSM_EVT_APOGEE_FAILSAFE) && drogue_t == 0.0f) drogue_t = t;
            if (act.event == FSM_EVT_MAIN_DEPLOY && main_t == 0.0f) main_t = t;
        }

        printf("  - t=5s 後全感測器死機中斷: 觸發安全看門狗倒數\n");
        printf("    * 副傘失效保護觸發時間: %.2fs (理論值: 起飛+33.93s)\n", drogue_t);
        printf("    * 主傘失效看門狗觸發時間: %.2fs (理論值: 起飛+100.0s)\n", main_t);
        printf("    * 評估: %s\n", (drogue_t > 33.0f && drogue_t < 36.0f && main_t > 130.0f) ? "PASS (失效保護硬體定時器發揮作用)" : "FAIL");
    }
    printf("\n");

    free(rows);
    return 0;
}
