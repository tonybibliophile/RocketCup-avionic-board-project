/*
 * test_fsm.c — 飛行狀態機黃金剖面單元測試（純 host 編譯，不需硬體）
 * ===========================================================================
 *   cd tests && make run
 *
 * 目的（P0-A 行為保存）：
 *   fsm.c 為 main.c FSM_Update 的逐字搬移；本測試以「完全決定性的輸入波形」
 *   鎖定現行為（各狀態轉移時刻 ±1 cycle、各硬體動作恰好一次），之後 P0-B
 *   起的安全功能必須在不破壞本檔既有案例的前提下疊加新案例。
 *
 * 涵蓋：
 *   [1] 標稱飛行剖面 PAD→BOOST→COAST→DEPLOY_DROGUE→APOGEE→DESCENT→MAIN_DEPLOY→LANDED
 *   [2] PAD 雜訊不誤觸 / 未校準不起飛
 *   [3] 頂點備用判定（速度過零 / 高度自峰值下降 5m）
 *   [4] tick 無號溢位（uint32 wrap）下計時正確
 *   [5] FSM_Init 熱啟動進入各狀態
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "fsm.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

/* === 模擬器：100Hz 推進，累計動作次數與事件時刻 === */
typedef struct {
    FSM_Context_t ctx;
    FSM_Input_t   in;
    uint32_t      now;
    int fire_n, release_n, main_n, buzzer_n;
    uint32_t t_liftoff, t_burnout, t_apogee, t_drogue_done, t_main, t_main_open, t_touchdown;
    uint32_t t_failsafe;
    float apogee_t_pred;
} Sim_t;

static void sim_init(Sim_t *s, FlightState_t s0, uint32_t t0,
                     uint32_t flight_start, uint8_t drogue_fired) {
    memset(s, 0, sizeof(*s));
    s->now = t0;
    FSM_Init(&s->ctx, s0, t0, flight_start, drogue_fired);
    s->in.est_calibrated = 1;
    s->in.est_healthy    = 1;
    s->in.a_z_g          = 1.0f;   /* 靜置 1g */
    s->in.uplink_armed   = 1;      /* 預設已武裝，使舊測試不需改動即可通過 */
    s->in.flash_pool_ready = 1;    /* 預設 flash pool 就緒，使舊測試不需改動即可通過 */
}

static FSM_Action_t sim_step(Sim_t *s) {
    s->in.now_ms = s->now;
    FSM_Action_t a = FSM_Step(&s->ctx, &s->in);
    s->fire_n    += a.fire_drogue;
    s->release_n += a.release_drogue;
    s->main_n    += a.deploy_main;
    s->buzzer_n  += a.start_buzzer;
    switch ((FSM_Event_t)a.event) {
        case FSM_EVT_LIFTOFF:        s->t_liftoff     = s->now; break;
        case FSM_EVT_BURNOUT:        s->t_burnout     = s->now; break;
        case FSM_EVT_DEPLOY_DROGUE:  s->t_apogee      = s->now; s->apogee_t_pred = a.apogee_t_pred; break;
        case FSM_EVT_APOGEE_FAILSAFE: s->t_failsafe   = s->now; break;
        case FSM_EVT_DROGUE_DONE:    s->t_drogue_done = s->now; break;
        case FSM_EVT_MAIN_DEPLOY:    s->t_main        = s->now; break;
        case FSM_EVT_MAIN_OPEN:      s->t_main_open   = s->now; break;
        case FSM_EVT_TOUCHDOWN:      s->t_touchdown   = s->now; break;
        default: break;
    }
    s->now += FSM_STEP_PERIOD_MS;
    return a;
}

/* 推進至指定絕對時刻（不含該時刻本身的 step）。用 < 而非 != ：門檻常數若非
 * FSM_STEP_PERIOD_MS(10) 的倍數，now 會直接跨過精確值，!= 永遠不成立而無窮迴圈
 * （曾實際踩過：FSM_FAILSAFE_APOGEE_MS 改成非 10 倍數值時整個測試套件卡死）。 */
static void sim_run_until(Sim_t *s, uint32_t t_end) {
    while (s->now < t_end) sim_step(s);
}

static int near_ms(uint32_t actual, uint32_t expect, uint32_t tol) {
    uint32_t d = (actual > expect) ? (actual - expect) : (expect - actual);
    return d <= tol;
}

/* ---------------------------------------------------------------- */
static void test_nominal_profile(void) {
    printf("[1] 標稱飛行剖面（決定性輸入波形）\n");
    Sim_t s;
    sim_init(&s, STATE_PAD, 0, 0, 0);

    /* PAD 0~2s：靜置 */
    s.in.h_est = 0.0f; s.in.v_est = 0.0f; s.in.a_z_g = 1.0f;
    sim_run_until(&s, 2000);
    check("PAD 2s 靜置不轉移", s.ctx.state == STATE_PAD_ARMED && s.t_liftoff == 0);

    /* 起飛：a_z=8g（>3g 門檻），連續 FSM_LIFTOFF_ACCEL_CONSEC_N(20)週期(200ms)
     * 後於 t=2190 觸發（防手震；t=2000 首次滿足，第20個週期 2000+19*10=2190）。 */
    s.in.a_z_g = 8.0f;
    sim_run_until(&s, 2200);
    check("LIFTOFF 於 a_z>3g 持續 200ms 後觸發 (t=2190)", s.t_liftoff == 2190 && s.ctx.state == STATE_BOOST);

    /* BOOST：a_z=8g 維持到 t=3000，之後 a_z=0.3（<0.5g）；
     * 燒完轉移受 4000ms 最短時間鎖 → 預期 t=6200 (state_entered=2190) */
    while (s.now < 3000) sim_step(&s);
    s.in.a_z_g = 0.3f;
    while (s.ctx.state == STATE_BOOST && s.now < 7000) sim_step(&s);
    check("BURNOUT 受 4000ms 時間鎖 (t=6200)", s.t_burnout == 6200 && s.ctx.state == STATE_COAST);

    /* COAST：v 線性 60 → −12 m/s²（落在動態 decel 視窗 [−25,−5]），h 緩升。
     * t_to_apogee = v/12 ≤ DROGUE_LEAD_TIME_S(飛行 4.0s，曾改 3.0s 後改回) 自
     * v≤48（dt=1.0s，t=7200）起成立；連續 5 週期 → 觸發於 t≈7240（實際執行輸出核對）。 */
    {
        float v0 = 60.0f; uint32_t t0 = s.now;  /* t0 = 6200 */
        while (s.ctx.state == STATE_COAST && s.now < 10000) {
            float dt_s = (float)(s.now - t0) / 1000.0f;
            s.in.v_est = v0 - 12.0f * dt_s;
            s.in.h_est = 100.0f + 5.0f * dt_s;   /* 緩升，不觸發高度下降備用判定 */
            sim_step(&s);
        }
    }
    check("DEPLOY_DROGUE 觸發於預測式提前量+5週期 (t≈7260)",
          near_ms(s.t_apogee, 7260, 10) && s.ctx.state == STATE_DEPLOY_DROGUE);
    check("馬達啟動恰一次 + drogue_fired 鎖存", s.fire_n == 1 && s.ctx.drogue_fired == 1);
    check("預估頂點時間合理 (3.0~4.0s)", s.apogee_t_pred >= 3.0f && s.apogee_t_pred <= 4.0f);

    /* DEPLOY_DROGUE：4.0s 預判倒數結束 (+4000ms) → 轉 APOGEE（僅存續 1 週期）→ DESCENT
     * （h_est/v_est 於本迴圈內未更新，凍結在 COAST 迴圈最後一筆值，is_past_peak 不成立，
     * 故僅由 lead_expired 觸發轉移，同舊版邏輯）。 */
    while (s.ctx.state == STATE_DEPLOY_DROGUE && s.now < 18000) sim_step(&s);
    check("到達頂點轉移至 APOGEE (+4000ms)", s.t_drogue_done == s.t_apogee + 4000 && s.ctx.state == STATE_APOGEE);
    s.in.v_est = -20.0f;
    s.in.h_est = 400.0f;
    sim_step(&s);   /* APOGEE 純記錄，同步轉 DESCENT */
    check("APOGEE 僅存續 1 週期即轉 DESCENT", s.ctx.state == STATE_DESCENT);

    /* DESCENT：v=−20 m/s，h 自 400m 下降；trigger = TARGET_MAIN_ALTITUDE(300) +
     * |v_fall|×MAIN_DEPLOY_DELAY_S(3.5) = 300 + 20×3.5 = 370m */
    {
        uint32_t t0 = s.now;
        while (s.ctx.state == STATE_DESCENT && s.now < 24000) {
            float dt_s = (float)(s.now - t0) / 1000.0f;
            s.in.v_est = -20.0f;
            s.in.h_est = 400.0f - 20.0f * dt_s;
            sim_step(&s);
        }
    }
    check("MAIN 部署於動態高度 (t≈12820)", near_ms(s.t_main, 12820, 20) && s.ctx.state == STATE_MAIN_DEPLOY);
    check("舵機動作恰一次", s.main_n == 1);

    /* 推進至馬達背景 8s 定時器到期 (+8000ms，自 t_apogee 起算) */
    while (s.release_n == 0 && s.now < s.t_apogee + 9000) sim_step(&s);
    check("馬達停止動作恰一次 (+8000ms)", s.release_n == 1 && s.now == s.t_apogee + 8000 + FSM_STEP_PERIOD_MS);

    /* MAIN_DEPLOY：3s 充氣 → LANDED */
    while (s.ctx.state == STATE_MAIN_DEPLOY && s.now < s.t_main + 4000) sim_step(&s);
    check("充氣 3s 後進 LANDED", near_ms(s.t_main_open, s.t_main + 3000, 10) && s.ctx.state == STATE_LANDED);

    /* LANDED：先持續下降（不觸發），之後 |v|<0.3 且 h<20 → 落地一次性 */
    {
        uint32_t t_hold_end = s.now + 2000U;
        while (s.now < t_hold_end) { s.in.v_est = -15.0f; s.in.h_est = 100.0f; sim_step(&s); }
        s.in.v_est = -0.1f; s.in.h_est = 5.0f;
        sim_step(&s);
        check("TOUCHDOWN 於條件成立當步觸發", s.t_touchdown == t_hold_end);
    }
    check("蜂鳴器恰一次（一次性鎖存）", s.buzzer_n == 1 && s.ctx.touchdown_latched == 1);

    /* 整體不變量 */
    check("全程各硬體動作各恰一次", s.fire_n == 1 && s.release_n == 1 && s.main_n == 1 && s.buzzer_n == 1);
}

/* ---------------------------------------------------------------- */
static void test_pad_noise(void) {
    printf("[2] PAD 雜訊免疫 / 校準閘\n");
    Sim_t s;

    /* 2.9g 突波 + 9.5m 高度抖動（皆低於門檻）持續 60s */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    while (s.now < 60000) {
        s.in.a_z_g = ((s.now / 10U) % 2U) ? 2.9f : 1.0f;
        s.in.h_est = 9.5f;
        s.in.v_est = 0.0f;
        sim_step(&s);
    }
    check("60s 次門檻雜訊不誤起飛", s.ctx.state == STATE_PAD_ARMED && s.t_liftoff == 0 && s.fire_n == 0);

    /* 未校準時即使 8g 也不起飛（現行行為鎖定） */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    s.in.est_calibrated = 0;
    s.in.a_z_g = 8.0f;
    sim_run_until(&s, 5000);
    check("EKF 未校準不起飛（現行行為）", s.ctx.state == STATE_PAD_ARMED && s.t_liftoff == 0);
}

/* ---------------------------------------------------------------- */
static void test_apogee_backup_paths(void) {
    printf("[3] 頂點備用判定\n");
    Sim_t s;

    /* 3a. 速度過零（v < −0.2）：v 由 +50 突降 −0.3（差分超出 decel 視窗 → decel 取預設） */
    sim_init(&s, STATE_COAST, 10000, 5000, 0);  /* 飛行時間鎖已過 (10000−5000>3000) */
    s.in.h_est = 300.0f;
    s.in.v_est = 50.0f;                          /* t_to = 50/9.8 = 5.1 > 3 → 主路徑不觸發 */
    sim_run_until(&s, 10500);
    check("v=+50 時主路徑未觸發", s.ctx.state == STATE_COAST && s.fire_n == 0);
    s.in.v_est = -0.3f;
    sim_run_until(&s, 11000);
    check("速度過零備用：5 週期後點火 (t=10540)", s.t_apogee == 10540 && s.fire_n == 1);

    /* 3b. 高度自峰值下降 5m（v 介於 [−0.2, 0] 不觸發其他路徑） */
    sim_init(&s, STATE_COAST, 10000, 5000, 0);
    s.in.v_est = -0.1f;                          /* 非正 → 主路徑不觸發；> −0.2 → 過零不觸發 */
    s.in.h_est = 200.0f;                         /* 建立峰值 */
    sim_run_until(&s, 10200);
    check("峰值持平不觸發", s.ctx.state == STATE_COAST && s.fire_n == 0);
    s.in.h_est = 194.0f;                         /* 自峰值下降 6m > 5m */
    sim_run_until(&s, 10500);
    check("高度下降備用：5 週期後點火 (t=10240)", s.t_apogee == 10240 && s.fire_n == 1);
}

/* ---------------------------------------------------------------- */
static void test_tick_overflow(void) {
    printf("[4] tick 無號溢位\n");
    Sim_t s;

    /* DEPLOY_DROGUE 8000ms 導通限時橫跨 uint32 wrap：
     * 進 COAST 於 t0 = 0xFFFFFFFF−1000，速度過零 → 點火於 t0+50，
     * 停止馬達應於點火 +8000ms（橫跨 wrap）。 */
    uint32_t t0 = 0xFFFFFFFFu - 1000u;
    sim_init(&s, STATE_COAST, t0, t0 - 5000u, 0);
    s.in.h_est = 300.0f;
    s.in.v_est = -0.3f;
    int steps_to_fire = 0, steps_fire_to_release = 0;
    while (s.fire_n == 0 && steps_to_fire < 1000)   { sim_step(&s); steps_to_fire++; }
    while (s.release_n == 0 && steps_fire_to_release < 1000) { sim_step(&s); steps_fire_to_release++; }
    check("溢位前點火（5 週期）", steps_to_fire == 5);
    check("橫跨 wrap 停止馬達於 +8000ms（800 週期）", steps_fire_to_release == 800);
    check("wrap 後狀態正確 (DESCENT)", s.ctx.state >= STATE_DEPLOY_DROGUE);
}

/* ---------------------------------------------------------------- */
static void test_hot_restart_init(void) {
    printf("[5] FSM_Init 熱啟動\n");
    Sim_t s;

    /* 5a. 恢復至 DESCENT（drogue 已點火）：絕不重複點火，主傘高度邏輯照常 */
    sim_init(&s, STATE_DESCENT, 50000, 50000 - 10000, 1);
    check("drogue_fired 自封包旗標還原", s.ctx.drogue_fired == 1);
    s.in.h_est = 180.0f;   /* < 220 trigger */
    s.in.v_est = -20.0f;
    sim_run_until(&s, 50100);
    check("恢復 DESCENT：主傘照常部署、不重複點火", s.main_n == 1 && s.fire_n == 0);

    /* 5b. 恢復至 COAST（未點火）：頂點判定照常可點火 */
    sim_init(&s, STATE_COAST, 50000, 50000 - 8000, 0);
    s.in.h_est = 300.0f;
    s.in.v_est = -1.0f;    /* 速度過零備用 */
    sim_run_until(&s, 50200);
    check("恢復 COAST：仍可正常點火", s.fire_n == 1 && s.ctx.state >= STATE_DEPLOY_DROGUE);

    /* 5c. 恢復至 BOOST：燒完計時自恢復時刻起算 */
    sim_init(&s, STATE_BOOST, 50000, 50000 - 1000, 0);
    s.in.a_z_g = 0.3f;
    sim_run_until(&s, 54100);
    check("恢復 BOOST：4000ms 後燒完轉移", s.t_burnout == 54010 && s.ctx.state == STATE_COAST);
}

/* ---------------------------------------------------------------- */
static void test_failsafe_and_baro_crosscheck(void) {
    printf("[6] P0-B：頂點失效保護 + baro 趨勢交叉檢查\n");
    Sim_t s;

    /* 6a. EKF 向上發散（v 恆 +50、h 爬升 → 三條 EKF 路徑全失效）+ baro 正常：
     * baro 趨勢交叉檢查應在 baro 峰值後短時間內點火。
     * baro 以 30 m/s 下降：回落 10m 需 340ms + 內層 20 週期 + 外層 5 週期防雜訊
     * → 峰值後 570ms 點火。 */
    sim_init(&s, STATE_COAST, 6000, 6000 - 4000, 0);   /* 時間鎖已過 */
    {
        const uint32_t t_peak = 12000;
        while (s.now < 16000 && s.fire_n == 0) {
            s.in.v_est = 50.0f;                          /* EKF 卡升 */
            s.in.h_est = 100.0f + 0.5f * (float)((s.now - 6000) / 10);
            s.in.baro_alt_rel = (s.now <= t_peak)
                ? 250.0f
                : 250.0f - 30.0f * (float)(s.now - t_peak) / 1000.0f;
            sim_step(&s);
        }
        check("EKF 發散時 baro 趨勢於峰值後 570ms 點火", s.t_apogee == t_peak + 570 && s.fire_n == 1);
        check("baro 路徑先於失效保護計時器（未走 failsafe）", s.t_failsafe == 0 && s.ctx.failsafe_fired == 0);
    }

    /* 6b. EKF 與 baro 雙雙失效（v 卡 +50、baro 凍結）：
     * 失效保護計時器於起飛 + FSM_FAILSAFE_APOGEE_MS 強制點火，恰一次。 */
    sim_init(&s, STATE_COAST, 6000, 2000, 0);
    s.in.v_est = 50.0f;
    s.in.h_est = 500.0f;
    s.in.baro_alt_rel = 100.0f;                          /* 凍結：永無 10m 回落 */
    sim_run_until(&s, 2000 + FSM_FAILSAFE_APOGEE_MS + 3000);
    /* near_ms 容忍 10ms：FSM_FAILSAFE_APOGEE_MS 不保證是 FSM_STEP_PERIOD_MS(10) 的
     * 倍數，實際觸發 tick 是「≥ 門檻」的第一個 10ms 格點，可能比門檻晚最多 9ms。 */
    check("全失效：計時器於起飛+FSM_FAILSAFE_APOGEE_MS 強制點火",
          near_ms(s.t_failsafe, 2000 + FSM_FAILSAFE_APOGEE_MS, 10) && s.fire_n == 1);
    check("failsafe 鎖存 + 轉入 DEPLOY_DROGUE", s.ctx.failsafe_fired == 1 && s.ctx.state >= STATE_DEPLOY_DROGUE);

    /* 6c. 燒完判定失效卡 BOOST（a_z 恆 5g）：計時器在 BOOST 也生效。 */
    sim_init(&s, STATE_BOOST, 5000, 4000, 0);
    s.in.a_z_g = 5.0f;                                   /* 永不低於 0.5g → 永不燒完 */
    s.in.v_est = 50.0f;
    s.in.h_est = 300.0f;
    sim_run_until(&s, 4000 + FSM_FAILSAFE_APOGEE_MS + 3000);
    check("卡 BOOST：計時器於起飛+FSM_FAILSAFE_APOGEE_MS 仍點火",
          near_ms(s.t_failsafe, 4000 + FSM_FAILSAFE_APOGEE_MS, 10) && s.fire_n == 1);

    /* 6d. baro 失效（FSM_SB_BARO_FAULT）+ EKF 正常：原 EKF 主路徑不受影響，
     * 且亂值 baro 不得干擾。v 線性 60 → −12 m/s²，t_to≤4（飛行 lead，改回 4.0s）自
     * v≤48（dt=1.0s）起，即 t=11000，+5 週期防雜訊 → t=11050。 */
    sim_init(&s, STATE_COAST, 10000, 6000, 0);
    s.in.sensor_bits = FSM_SB_BARO_FAULT;
    s.in.baro_alt_rel = -500.0f;                         /* 亂值，必須被閘控忽略 */
    while (s.ctx.state == STATE_COAST && s.now < 15000) {
        float dt_s = (float)(s.now - 10000) / 1000.0f;
        s.in.v_est = 60.0f - 12.0f * dt_s;
        s.in.h_est = 100.0f + 5.0f * dt_s;
        sim_step(&s);
    }
    check("baro 失效：EKF 主路徑照常 (t=11050)", s.t_apogee == 11050 && s.fire_n == 1);

    /* 6e. 起飛第三冗餘：ADXL 死（a_z=1g）+ EKF 死（h=0）+ baro 相對高度 25m → 起飛；
     * baro 失效位設起時則不得起飛。 */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    s.in.a_z_g = 1.0f;
    s.in.h_est = 0.0f;
    s.in.baro_alt_rel = 25.0f;
    sim_step(&s); /* step 1: PAD -> PAD_ARMED */
    sim_step(&s); /* step 2: PAD_ARMED -> BOOST */
    check("baro 起飛冗餘：baro_rel>20m 即起飛", s.ctx.state == STATE_BOOST);

    sim_init(&s, STATE_PAD, 0, 0, 0);
    s.in.a_z_g = 1.0f;
    s.in.h_est = 0.0f;
    s.in.baro_alt_rel = 25.0f;
    s.in.sensor_bits = FSM_SB_BARO_FAULT;
    sim_run_until(&s, 5000);
    check("baro 失效位起時不誤起飛", s.ctx.state == STATE_PAD_ARMED);
}

/* ---------------------------------------------------------------- */
static void test_ekf_unhealthy_fallback(void) {
    printf("[7] P0-C：EKF unhealthy 降級鏈（raw-baro）\n");
    Sim_t s;

    /* 7a. EKF unhealthy 時，發散的 h_est/v_est 不得觸發 EKF 頂點路徑：
     * v=-5（健康時會走速度過零）、h 自峰值大跌（健康時會走高度回落），
     * 但 baro 持平 → 不得點火。 */
    sim_init(&s, STATE_COAST, 10000, 6000, 0);
    s.in.est_healthy = 0;
    s.in.h_est = 300.0f;
    sim_step(&s);                       /* 建立 max_altitude=300 */
    s.in.v_est = -5.0f;                 /* 假裝 EKF 報告下墜 */
    s.in.h_est = 100.0f;                /* 假裝高度自峰值大跌 200m */
    s.in.baro_alt_rel = 150.0f;         /* baro 持平：真實仍在上升段 */
    sim_run_until(&s, 14000);
    check("unhealthy：發散 EKF 值不觸發點火", s.fire_n == 0 && s.ctx.state == STATE_COAST);

    /* baro 開始下降 → 趨勢路徑點火（降級鏈接手） */
    {
        uint32_t t_fall = s.now;        /* 14000 */
        while (s.now < 16000 && s.fire_n == 0) {
            s.in.baro_alt_rel = 150.0f - 25.0f * (float)(s.now - t_fall) / 1000.0f;
            sim_step(&s);
        }
        check("unhealthy：baro 趨勢路徑照常點火", s.fire_n == 1 && s.t_apogee != 0);
    }

    /* 7b. 全程降級飛行（EKF 死亡、ekf_calibrated=0）：僅靠 a_z + baro 完成全序列 */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    s.in.est_healthy    = 0;
    s.in.est_calibrated = 0;            /* EKF 死亡 → 永遠不會校準完成 */
    s.in.h_est = 0.0f; s.in.v_est = 0.0f;

    /* PAD 1s 靜置（確認降級下不誤觸發） */
    s.in.a_z_g = 1.0f; s.in.baro_alt_rel = 0.0f;
    sim_run_until(&s, 1000);
    check("降級：PAD 靜置不誤起飛", s.ctx.state == STATE_PAD_ARMED);

    /* 起飛（a_z 路徑，無視 ekf_calibrated；連續 200ms 防手震，t=1000+190=1190 觸發） */
    s.in.a_z_g = 8.0f;
    sim_run_until(&s, 1200);
    check("降級：a_z 起飛（不需 EKF 校準）", s.ctx.state == STATE_BOOST && s.t_liftoff == 1190);

    /* BOOST：燒至 t=2000 → a_z=0.3；燒完於 1190+4000 後第一步 = 5200 */
    while (s.now < 2000) { s.in.baro_alt_rel = 290.0f * (float)(s.now - 1000) / 5000.0f; sim_step(&s); }
    s.in.a_z_g = 0.3f;
    while (s.ctx.state == STATE_BOOST && s.now < 5500) {
        s.in.baro_alt_rel = 290.0f * (float)(s.now - 1000) / 5000.0f;
        sim_step(&s);
    }
    check("降級：a_z 燒完轉移 (t=5200)", s.t_burnout == 5200 && s.ctx.state == STATE_COAST);

    /* COAST：baro 升至 t=6000 峰值 290，後以 25 m/s 下降 → 趨勢點火（峰值後 ~640ms） */
    while (s.ctx.state == STATE_COAST && s.now < 9000) {
        s.in.baro_alt_rel = (s.now <= 6000)
            ? 290.0f * (float)(s.now - 1000) / 5000.0f
            : 290.0f - 25.0f * (float)(s.now - 6000) / 1000.0f;
        sim_step(&s);
    }
    check("降級：baro 趨勢點火（峰值後 <1s）", s.fire_n == 1 &&
          s.t_apogee > 6000 && s.t_apogee < 7000);

    /* DEPLOY_DROGUE 4s → APOGEE(1週期) → DESCENT；主傘於 baro_rel ≤ 8m（電梯降級門檻常數
     * 已改依 profile 分組，host 測試沿用 fsm.h 現行預設常數值，見下方 near_ms 寬容窗）。 */
    while (s.ctx.state != STATE_MAIN_DEPLOY && s.now < 16000) {
        s.in.baro_alt_rel = (s.now <= 6000)
            ? 290.0f
            : ((290.0f - 25.0f * (float)(s.now - 6000) / 1000.0f > 0.0f)
                ? 290.0f - 25.0f * (float)(s.now - 6000) / 1000.0f : 0.0f);
        sim_step(&s);
    }
    check("降級：主傘於 baro 降級門檻部署", s.main_n == 1);

    /* MAIN_DEPLOY 3s 充氣 → LANDED；LANDED 內先餵持續下降 baro（>2m/2s，仍變動）→ 不誤判落地；
     * 之後凍結 baro 低點 → 2s 視窗穩定 → 落地。以 t_main 相對計時（不受 8s 馬達位移影響）。 */
    uint32_t t_main_fired = s.t_main;
    /* Phase A：main 後 6s（含 3s 充氣 + 3s LANDED 下降）餵 40→10m 持續下降（<30 但變動）→ 不落地 */
    while (s.now < t_main_fired + 6000) {
        float dt = (float)(s.now - t_main_fired) / 1000.0f;
        float b = 40.0f - 5.0f * dt;
        s.in.baro_alt_rel = (b > 10.0f) ? b : 10.0f;
        sim_step(&s);
    }
    check("降級：下降中不誤判落地", s.t_touchdown == 0 && s.ctx.state == STATE_LANDED);
    /* Phase B：凍結 baro 於 10m → 連續 2s 視窗穩定（需跨 2 個取樣點 ref 皆為 10）→ 落地 */
    s.in.baro_alt_rel = 10.0f;
    sim_run_until(&s, t_main_fired + 12000);
    check("降級：baro 穩定 2s 視窗後落地", s.t_touchdown != 0 && s.buzzer_n == 1);

    /* 7c. unhealthy 時 h_est 發散不得誤觸起飛 */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    s.in.est_healthy = 0;
    s.in.a_z_g = 1.0f;
    s.in.h_est = 50.0f;                 /* 發散：健康時會誤判起飛 */
    s.in.baro_alt_rel = 0.0f;
    sim_run_until(&s, 5000);
    check("unhealthy：發散 h_est 不誤起飛", s.ctx.state == STATE_PAD_ARMED);
}

/* ---------------------------------------------------------------- */
static void test_hotstart_decide(void) {
    printf("[8] P0-F：熱啟動驗證鏈（FSM_HotStartDecide）\n");
    FSM_HotStartDecision_t d;

    /* 封包無效 → PAD */
    d = FSM_HotStartDecide(0, STATE_COAST, 8000, 250.0f, 240.0f, 0);
    check("封包 CRC 無效 → PAD", d.restore == 0 && d.state == STATE_PAD);

    /* 上次飛行正常收尾（末筆 LANDED）→ PAD */
    d = FSM_HotStartDecide(1, STATE_LANDED, 30000, 100.0f, 100.0f, 1);
    check("末筆 LANDED → PAD（正常地面開機）", d.restore == 0);

    /* 標準空中重啟：COAST 未點火 → 恢復 COAST */
    d = FSM_HotStartDecide(1, STATE_COAST, 8000, 250.0f, 200.0f, 0);
    check("COAST 未點火 → 恢復 COAST", d.restore == 1 && d.state == STATE_COAST && d.drogue_fired == 0);

    /* 防二次點火：COAST/DEPLOY_DROGUE/APOGEE 已點火 → 一律恢復 DESCENT */
    d = FSM_HotStartDecide(1, STATE_COAST, 8000, 250.0f, 200.0f, 1);
    check("COAST 已點火 → 強制 DESCENT", d.restore == 1 && d.state == STATE_DESCENT && d.drogue_fired == 1);
    d = FSM_HotStartDecide(1, STATE_DEPLOY_DROGUE, 9000, 280.0f, 250.0f, 1);
    check("DEPLOY_DROGUE 已點火 → 強制 DESCENT", d.restore == 1 && d.state == STATE_DESCENT);
    d = FSM_HotStartDecide(1, STATE_APOGEE, 9000, 280.0f, 250.0f, 1);
    check("APOGEE 已點火 → 強制 DESCENT", d.restore == 1 && d.state == STATE_DESCENT);
    d = FSM_HotStartDecide(1, STATE_DESCENT, 12000, 180.0f, 150.0f, 1);
    check("DESCENT 已點火 → 維持 DESCENT", d.restore == 1 && d.state == STATE_DESCENT);

    /* 合理性檢查 */
    d = FSM_HotStartDecide(1, STATE_COAST, 60000, 250.0f, 240.0f, 0);
    check("tick ≥ 60s → PAD（陳舊資料）", d.restore == 0);
    d = FSM_HotStartDecide(1, STATE_COAST, 8000, 550.0f, 100.0f, 0);
    check("高度差 ≥ 300m → PAD（殘留封包）", d.restore == 0);
    d = FSM_HotStartDecide(1, STATE_COAST, 8000, 100.0f, 550.0f, 0);
    check("高度差雙向皆檢查", d.restore == 0);

    /* BOOST 未點火 → 恢復 BOOST（燒完/失效計時器照常運作） */
    d = FSM_HotStartDecide(1, STATE_BOOST, 1200, 60.0f, 30.0f, 0);
    check("BOOST 未點火 → 恢復 BOOST", d.restore == 1 && d.state == STATE_BOOST);
}

static void test_fsm_arm_requirement(void) {
    printf("[9] EKF 武裝起飛限制\n");
    Sim_t s;
    sim_init(&s, STATE_PAD, 0, 0, 0);

    /* 1. 未武裝（uplink_armed = 0），即使高G震動也應拒絕起飛，維持在 STATE_PAD */
    s.in.uplink_armed = 0;
    s.in.a_z_g = 8.0f; /* 8G 劇烈加速度 */
    for (int i = 0; i < 50; i++) sim_step(&s); // 500ms
    check("未武裝：8G 加速度持續 500ms 依然維持 STATE_PAD", s.ctx.state == STATE_PAD && s.t_liftoff == 0);

    /* 2. 送出武裝（uplink_armed = 1），應轉移至 STATE_PAD_ARMED */
    s.in.uplink_armed = 1;
    sim_step(&s);
    check("武裝：轉移至 STATE_PAD_ARMED", s.ctx.state == STATE_PAD_ARMED);

    /* 3. 在 STATE_PAD_ARMED 下，受到持續 8G 加速度 200ms 後應正常起飛轉入 STATE_BOOST */
    s.in.a_z_g = 8.0f;
    for (int i = 0; i < 19; i++) sim_step(&s); // 190ms
    check("已武裝：未滿 200ms 高G前維持 STATE_PAD_ARMED", s.ctx.state == STATE_PAD_ARMED && s.t_liftoff == 0);

    sim_step(&s); // 第 20 週期 (200ms)
    check("已武裝：滿 200ms 高G起飛轉入 STATE_BOOST", s.ctx.state == STATE_BOOST && s.t_liftoff > 0);

    /* 4. 武裝解除：若在 STATE_PAD_ARMED 時 uplink_armed 變回 0，應退回 STATE_PAD */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    s.in.uplink_armed = 1;
    sim_step(&s);
    check("武裝：轉為 STATE_PAD_ARMED", s.ctx.state == STATE_PAD_ARMED);
    s.in.uplink_armed = 0;
    sim_step(&s);
    check("解除武裝：變回 STATE_PAD", s.ctx.state == STATE_PAD);
}

/* ---------------------------------------------------------------- */
/* D1：副傘 PD13 加法 OR 互救（peer_drogue_cmd）。嚴格加法 + arm-interlock。 */
static void test_d1_peer_drogue_rescue(void) {
    printf("[D1] 副傘加法 OR 互救（peer_drogue_cmd）\n");

    /* (a) 台上安全：PAD 未起飛，對端聲稱已開副傘 → 本板不點火（drogue 判定僅在 COAST 跑）。 */
    {
        Sim_t s; sim_init(&s, STATE_PAD, 0, 0, 0);
        s.in.a_z_g = 1.0f; s.in.v_est = 0.0f; s.in.h_est = 0.0f;
        s.in.peer_drogue_cmd = 1;
        sim_run_until(&s, 3000);
        check("PAD：peer 開傘不使本板點火", s.fire_n == 0 && s.ctx.state != STATE_DEPLOY_DROGUE);
    }

    /* (b) 互救：已在 COAST 上升中、自身各頂點路徑皆不成立；無 peer 命令不誤開，
     *     給 peer 命令後連續 5 週期開副傘一次。 */
    {
        Sim_t s; sim_init(&s, STATE_COAST, 4000, 0, 0);  /* flight_start=0，已飛 4s（過 3s 鎖） */
        s.in.est_healthy = 1; s.in.v_est = 50.0f; s.in.h_est = 500.0f;
        s.in.baro_alt_rel = 500.0f; s.in.a_z_g = 0.0f;

        s.in.peer_drogue_cmd = 0;
        sim_run_until(&s, 4500);
        check("上升中且無 peer → 不誤開", s.fire_n == 0 && s.ctx.state == STATE_COAST);

        s.in.peer_drogue_cmd = 1;
        uint32_t t_cmd = s.now;                          /* = 4500 */
        while (s.ctx.state == STATE_COAST && s.now < t_cmd + 500) sim_step(&s);
        check("peer 命令 → 本板開副傘一次", s.fire_n == 1 && s.ctx.state == STATE_DEPLOY_DROGUE);
        check("恰於命令後 5 週期(50ms)觸發",
              s.t_apogee == t_cmd + (FSM_APOGEE_CONSEC_N - 1U) * FSM_STEP_PERIOD_MS);
    }
}

/* ---------------------------------------------------------------- */
static void test_main_max_alt_limit(void) {
    printf("[P0-H] 主傘開傘高度限制（必須 <= 600m 且正在下降）\n");
    Sim_t s;
    sim_init(&s, STATE_DESCENT, 50000, 50000 - 30000, 1);
    s.in.v_est = -250.0f;    /* 高速下墜：動態預估 h_trigger_main = 300 + 250*1.5 = 675m，應被限制於 600m */
    s.in.h_est = 700.0f;     /* 700m 高度 > 600m 上限 */
    s.in.baro_alt_rel = 700.0f;

    sim_run_until(&s, 50500);
    check("高度 700m (>600m 上限) 即使下墜中亦不部署主傘", s.main_n == 0 && s.ctx.state == STATE_DESCENT);

    s.in.h_est = 550.0f;     /* 降至 550m <= 600m 上限 */
    s.in.baro_alt_rel = 550.0f;
    sim_run_until(&s, 50600);
    check("降至 550m (<=600m 上限) 部署主傘", s.main_n == 1 && s.ctx.state == STATE_MAIN_DEPLOY);
}

/* ---------------------------------------------------------------- */
/* 燒完 / 主傘連續週期防雜訊：a_z_g（無濾波原始取樣）與 h_est 單筆離群值
 * 不得直接觸發燒完或主傘展開，須連續 N 週期成立才算數（比照起飛 a_z 防手震）。 */
static void test_burnout_and_main_debounce(void) {
    printf("[10] 燒完/主傘連續週期防雜訊（防單筆離群值誤觸發）\n");
    Sim_t s;

    /* 10a. BOOST 期單筆 a_z 瞬間掉點（<0.5g 僅 1 週期）不得誤判燒完；
     * state_entered 早已跨過 4000ms 時間鎖，僅由連續週期防護把關（持續高G至
     * elapsed=4200ms 才開始後續檢查，確保時間鎖不再是干擾因素）。 */
    sim_init(&s, STATE_BOOST, 10000, 10000 - 3000, 0);
    s.in.a_z_g = 5.0f;
    sim_run_until(&s, 14200);
    check("持續高G不誤判燒完", s.ctx.state == STATE_BOOST && s.t_burnout == 0);

    s.in.a_z_g = 0.2f;   /* 單筆瞬間掉點 */
    sim_step(&s);
    s.in.a_z_g = 5.0f;   /* 立即回復高G */
    sim_run_until(&s, 14300);
    check("單筆掉點（1週期）不誤判燒完", s.ctx.state == STATE_BOOST && s.t_burnout == 0);

    s.in.a_z_g = 0.3f;   /* 持續低於門檻 */
    sim_run_until(&s, 14400);
    check("持續低G達連續週期數才觸發燒完", s.ctx.state == STATE_COAST && s.t_burnout != 0);

    /* 10b. DESCENT 期單筆 h_est 瞬間掉到門檻以下（1 週期）不得誤判主傘部署。 */
    sim_init(&s, STATE_DESCENT, 20000, 20000 - 15000, 0);
    s.in.v_est = -20.0f;
    s.in.h_est = 500.0f;   /* 遠高於觸發高度 (300+20*3.5=370m) */
    sim_run_until(&s, 20200);
    check("高空不誤判主傘", s.ctx.state == STATE_DESCENT && s.main_n == 0);

    s.in.h_est = 300.0f;   /* 單筆瞬間掉到門檻以下 */
    sim_step(&s);
    s.in.h_est = 500.0f;   /* 立即回復高空 */
    sim_run_until(&s, 20300);
    check("單筆離群值（1週期）不誤判主傘部署", s.ctx.state == STATE_DESCENT && s.main_n == 0);

    s.in.h_est = 300.0f;   /* 持續低於觸發高度 */
    sim_run_until(&s, 20400);
    check("持續低於觸發高度達連續週期數才觸發主傘",
          s.ctx.state == STATE_MAIN_DEPLOY && s.main_n == 1);
}

/* ---------------------------------------------------------------- */
int main(void) {
    printf("=== test_fsm：飛行狀態機黃金剖面（P0-A 行為保存） ===\n");
    test_nominal_profile();
    test_pad_noise();
    test_apogee_backup_paths();
    test_tick_overflow();
    test_hot_restart_init();
    test_failsafe_and_baro_crosscheck();
    test_ekf_unhealthy_fallback();
    test_hotstart_decide();
    test_fsm_arm_requirement();
    test_d1_peer_drogue_rescue();
    test_main_max_alt_limit();
    test_burnout_and_main_debounce();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
