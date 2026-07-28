/*
 * test_fsm_noise_suite.c — 使用 C 語言原生 VFilter (3-state EKF) + FSM 進行全動態噪聲掃描
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

// 簡易偽常態雜訊 (Box-Muller)
static float rand_normal(float mean, float stddev) {
    if (stddev <= 0.0f) return mean;
    float u1 = (float)rand() / (float)RAND_MAX;
    float u2 = (float)rand() / (float)RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    float z0 = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
    return mean + z0 * stddev;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        printf("Usage: %s <csv_path> <noise_a_g> <noise_h_m> [pitch_deg]\n", argv[0]);
        return 1;
    }
    const char *csv_path = argv[1];
    float noise_a = atof(argv[2]);
    float noise_h = atof(argv[3]);
    float pitch_deg = (argc >= 5) ? atof(argv[4]) : 0.0f;
    float cos_theta = cosf(pitch_deg * 3.14159265f / 180.0f);

    CSVRow_t *raw_rows = NULL;
    int raw_count = 0;
    if (!load_csv(csv_path, &raw_rows, &raw_count)) {
        printf("Error loading CSV\n");
        return 1;
    }

    srand(42);

    VFilter_t vf;
    vf_init(&vf);

    FSM_Context_t ctx;
    uint32_t t0 = (uint32_t)(raw_rows[0].time_s * 1000.0f);
    FSM_Init(&ctx, STATE_PAD, t0, 0, 0);

    FSM_Input_t in;
    memset(&in, 0, sizeof(in));
    in.est_calibrated = 1;
    in.est_healthy = 1;
    in.uplink_armed = 1;
    in.flash_pool_ready = 1;

    float t_end = raw_rows[raw_count - 1].time_s;
    float dt_s = 0.010f; // 固定 100Hz (10ms)
    int raw_idx = 0;

    for (float t = 0.0f; t <= t_end; t += dt_s) {
        uint32_t now_ms = (uint32_t)(t * 1000.0f);

        // 線性插值取得 100Hz 數據
        while (raw_idx < raw_count - 1 && raw_rows[raw_idx + 1].time_s <= t) {
            raw_idx++;
        }
        float h_true_m = raw_rows[raw_idx].alt_m * cos_theta;
        float v_true_mps = raw_rows[raw_idx].v_z_mps * cos_theta;
        float a_raw = raw_rows[raw_idx].a_tot_mps2;

        if (raw_idx < raw_count - 1) {
            float t1 = raw_rows[raw_idx].time_s;
            float t2 = raw_rows[raw_idx + 1].time_s;
            if (t2 > t1) {
                float frac = (t - t1) / (t2 - t1);
                h_true_m = (raw_rows[raw_idx].alt_m + frac * (raw_rows[raw_idx + 1].alt_m - raw_rows[raw_idx].alt_m)) * cos_theta;
                v_true_mps = (raw_rows[raw_idx].v_z_mps + frac * (raw_rows[raw_idx + 1].v_z_mps - raw_rows[raw_idx].v_z_mps)) * cos_theta;
                a_raw = raw_rows[raw_idx].a_tot_mps2 + frac * (raw_rows[raw_idx + 1].a_tot_mps2 - raw_rows[raw_idx].a_tot_mps2);
            }
        }

        float a_net_mps2 = (t > 6.5f) ? -a_raw : a_raw;
        a_net_mps2 *= cos_theta;

        // 雜訊注入
        float raw_a_g = (a_net_mps2 + 9.80665f) / 9.80665f + rand_normal(0.0f, noise_a);
        float raw_h_m = rand_normal(h_true_m, noise_h);

        // Vertical Filter & EKF Fusion
        float h_est = 0.0f, v_est = 0.0f;
        if (noise_h <= 0.01f && noise_a <= 0.01f) {
            h_est = h_true_m;
            v_est = v_true_mps;
        } else {
            float a_meas_mps2 = (raw_a_g - 1.0f) * 9.80665f;
            vf_predict(&vf, dt_s);
            vf_update_accel(&vf, a_meas_mps2);
            vf_update_baro(&vf, raw_h_m);
            h_est = vf_h(&vf);
            v_est = vf_v(&vf);
        }

        in.now_ms = now_ms;
        in.a_z_g = raw_a_g;
        in.h_est = h_est;
        in.v_est = v_est;
        in.baro_alt_rel = raw_h_m;

        FSM_Action_t act = FSM_Step(&ctx, &in);

        // 輸出 JSON/CSV 格式數據供 Python 畫圖
        printf("%.3f,%.2f,%.2f,%.2f,%d,%d\n",
               t, h_true_m, h_est, v_est, ctx.state, act.event);
    }

    free(raw_rows);
    return 0;
}
