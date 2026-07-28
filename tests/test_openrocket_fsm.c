/*
 * test_openrocket_fsm.c — 基於真實 OpenRocket 模擬數據的 FSM 狀態機可靠度評估測試
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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
    if (!f) {
        printf("Error opening %s\n", filepath);
        return 0;
    }

    char line[512];
    int capacity = 8000;
    int count = 0;
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

static void run_fsm_simulation(const char *name, CSVRow_t *rows, int count, float noise_a_std, float noise_h_std) {
    printf("========================================================================\n");
    printf(" 模擬分析名稱: %s (共 %d 筆數據, 噪聲 std_a=%.2fg, std_h=%.2fm)\n", name, count, noise_a_std, noise_h_std);
    printf("========================================================================\n");

    FSM_Context_t ctx;
    uint32_t t0 = 0;
    FSM_Init(&ctx, STATE_PAD, t0, 0, 0);

    FSM_Input_t in;
    memset(&in, 0, sizeof(in));
    in.est_calibrated = 1;
    in.est_healthy = 1;
    in.uplink_armed = 1;
    in.flash_pool_ready = 1;

    FlightState_t prev_state = STATE_PAD;

    // 找 OpenRocket 的物理事件時刻（作對照基準）
    float physical_apogee_t = 0.0f;
    float physical_apogee_h = 0.0f;
    for (int i = 0; i < count; i++) {
        if (rows[i].alt_m > physical_apogee_h) {
            physical_apogee_h = rows[i].alt_m;
            physical_apogee_t = rows[i].time_s;
        }
    }

    printf(" OpenRocket 物理參考值: 頂點高度 = %.2f m, 頂點時間 = %.2f s\n", physical_apogee_h, physical_apogee_t);
    printf(" 狀態轉移歷程:\n");

    uint32_t sim_now_ms = 0;
    for (int i = 0; i < count; i++) {
        sim_now_ms = (uint32_t)(rows[i].time_s * 1000.0f);

        // 加噪聲
        float a_z_g = (rows[i].a_tot_mps2 / 9.80665f); // 垂直 / 合加速度 (g)
        if (noise_a_std > 0.0f) {
            float r = ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f;
            a_z_g += r * noise_a_std;
        }

        float h_est = rows[i].alt_m;
        if (noise_h_std > 0.0f) {
            float r = ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f;
            h_est += r * noise_h_std;
        }

        in.now_ms = sim_now_ms;
        in.a_z_g = a_z_g;
        in.h_est = h_est;
        in.v_est = rows[i].v_z_mps;
        in.baro_alt_rel = h_est;

        FSM_Action_t act = FSM_Step(&ctx, &in);

        if (ctx.state != prev_state) {
            printf("   t = %6.3f s [%6.1f m] | 轉移: %d -> %d | 事件=%d",
                   rows[i].time_s, rows[i].alt_m, prev_state, ctx.state, act.event);
            if (act.fire_drogue) printf(" [🔥 副傘點火]");
            if (act.deploy_main) printf(" [🪂 主傘釋放]");
            if (ctx.state == STATE_APOGEE) printf(" [📍 頂點紀錄 max_alt=%.1fm]", ctx.max_altitude);
            printf("\n");
            prev_state = ctx.state;
        }
    }
    printf(" 最終狀態: %d | 紀錄最大高度: %.2f m\n\n", ctx.state, ctx.max_altitude);
}

int main(void) {
    srand(42);
    CSVRow_t *rows1 = NULL, *rows2 = NULL;
    int count1 = 0, count2 = 0;

    if (load_csv("simulation_and_data/flight_data/台灣盃2026時間高度垂直速度合速度合加速度.csv", &rows1, &count1)) {
        run_fsm_simulation("台灣盃 2026 (無雜訊 標稱模擬)", rows1, count1, 0.0f, 0.0f);
        run_fsm_simulation("台灣盃 2026 (高雜訊模擬 ±0.5g, ±3m)", rows1, count1, 0.5f, 3.0f);
    }

    if (load_csv("simulation_and_data/flight_data/v2模擬.csv", &rows2, &count2)) {
        run_fsm_simulation("v2模擬 (無雜訊 標稱模擬)", rows2, count2, 0.0f, 0.0f);
    }

    if (rows1) free(rows1);
    if (rows2) free(rows2);
    return 0;
}
