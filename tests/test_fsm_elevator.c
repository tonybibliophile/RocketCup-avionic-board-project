/*
 * test_fsm_elevator.c — 電梯場測 profile 黃金剖面單元測試（純 host 編譯，不需硬體）
 * ===========================================================================
 *   cd tests && make run                （或單獨：make test_fsm_elevator && ./test_fsm_elevator）
 *   編譯旗標：-DFLIGHT_PROFILE_ELEVATOR=1（見 tests/Makefile），
 *   使 fsm.h 的 profile 分組常數切換為電梯門檻（board_config.h FLIGHT_PROFILE_ELEVATOR）。
 *
 * 目的：
 *   電梯場測與真實飛行共用同一份 fsm.c 邏輯，僅門檻不同。board_config.h 的
 *   FEATURE_FORCE_BARO_ONLY 現已固定為 0（電梯不再整條強制降級）——電梯改用
 *   估計器（VF）路徑，僅 fsm.h 的 FSM_APOGEE_DYNAMIC_PREDICT_ENABLED 對電梯關閉
 *   COAST 頂點判定的「路徑1：動態預測」，因為該路徑的 decel fallback 假設
 *   「v_est 隨時間由大降到0」僅真實彈道成立，電梯全程近似等速違反此假設
 *   （曾實測：只要 est_healthy=1 且路徑1未關，爬升第 40ms 即誤點副傘）。
 *   本測試鎖定電梯剖面在「est_healthy=1（電梯用 VF）+ 路徑1關閉」下的行為。
 *
 * 涵蓋：
 *   [1] 標稱電梯行程：PAD→BOOST→COAST→APOGEE→DESCENT→MAIN_DEPLOY→LANDED
 *       全程四動作（fire/release/main/buzzer）各恰一次，時刻皆由 FSM 實際輸出鎖定
 *       （黃金值來自跑一次程式取得，而非手推——COAST 頂點改由路徑2(速度過零)
 *       主導，較舊版純 baro 趨勢檢查更早觸發，詳見下方註解）。
 *   [2] 干擾免疫：頂樓單週期 baro 尖刺（consec 40 防護）/ 上升期抖動 / PAD 抖動
 *   [3] 路徑1關閉驗證：真實電梯等速爬升全程（含頂樓持平）絕不誤觸發頂點
 *   [4] 路徑2（速度過零）仍正確運作：真實下降開始後才觸發頂點/主傘
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

/* === 模擬器：100Hz 推進，累計動作次數與事件時刻（精簡複製自 test_fsm.c） === */
typedef struct {
    FSM_Context_t ctx;
    FSM_Input_t   in;
    uint32_t      now;
    int fire_n, release_n, main_n, buzzer_n;
    uint32_t t_liftoff, t_burnout, t_apogee, t_drogue_done, t_main, t_main_open, t_touchdown;
} Sim_t;

/* 電梯 profile 現用 est_healthy=1（電梯改用 VF，見檔頭）：h_est/v_est 由呼叫端
 * 逐步餵入真實電梯剖面對應值（見 elevator_profile/elevator_velocity），而非
 * 舊版的固定垃圾值——因為現在 est_healthy=1 時 h_est/v_est 會真的參與判斷
 * （路徑2/3、主傘動態高度、下降/落地判定），必須餵物理合理值。 */
static void sim_init(Sim_t *s, FlightState_t s0, uint32_t t0,
                     uint32_t flight_start, uint8_t drogue_fired) {
    memset(s, 0, sizeof(*s));
    s->now = t0;
    FSM_Init(&s->ctx, s0, t0, flight_start, drogue_fired);
    s->in.est_calibrated  = 1;      /* 電梯改用 VF：pad 基準視為已就緒 */
    s->in.est_healthy     = 1;      /* 電梯改用 VF：VF 健康（最近 300ms 內有 baro 被接受） */
    s->in.a_z_g           = 1.0f;   /* 電梯全程等速，恆 1g（無淨加速度） */
    s->in.h_est           = 0.0f;
    s->in.v_est           = 0.0f;
    s->in.uplink_armed    = 1;       /* 預設已武裝，使電梯測試正常執行 */
    s->in.flash_pool_ready = 1;      /* 預設 flash pool 就緒，使舊測試不需改動即可通過 */
}

static FSM_Action_t sim_step(Sim_t *s) {
    s->in.now_ms = s->now;
    FSM_Action_t a = FSM_Step(&s->ctx, &s->in);
    s->fire_n    += a.fire_drogue;
    s->release_n += a.release_drogue;
    s->main_n    += a.deploy_main;
    s->buzzer_n  += a.start_buzzer;
    switch ((FSM_Event_t)a.event) {
        case FSM_EVT_LIFTOFF:     s->t_liftoff     = s->now; break;
        case FSM_EVT_BURNOUT:     s->t_burnout     = s->now; break;
        case FSM_EVT_DEPLOY_DROGUE: s->t_apogee    = s->now; break;
        case FSM_EVT_DROGUE_DONE: s->t_drogue_done = s->now; break;
        case FSM_EVT_MAIN_DEPLOY: s->t_main        = s->now; break;
        case FSM_EVT_MAIN_OPEN:   s->t_main_open   = s->now; break;
        case FSM_EVT_TOUCHDOWN:   s->t_touchdown   = s->now; break;
        default: break;
    }
    s->now += FSM_STEP_PERIOD_MS;
    return a;
}

static int near_ms(uint32_t actual, uint32_t expect, uint32_t tol) {
    uint32_t d = (actual > expect) ? (actual - expect) : (expect - actual);
    return d <= tol;
}

/* 電梯高度剖面：PAD 靜置至 t0 → 爬升(1.5m/s) → 頂樓 30m 持平 → 下降(1.5m/s) → 地面持平。
 * 回傳 baro 相對高度 (m)。爬升/下降時間 = 30 / 1.5 = 20s。
 * elevator_velocity() 為同一剖面的瞬時垂直速度 (m/s)，供 est_healthy=1 時餵 v_est
 * 使用（電梯改用 VF 後，h_est/v_est 須與 baro 描述同一條真實軌跡，理想化為 VF
 * 完美跟蹤——雜訊免疫另由 [2] 驗證）。 */
#define ELEV_T0        2000U    /* PAD 靜置結束、開始爬升 */
#define ELEV_RISE_MS   20000U   /* 30m ÷ 1.5 m/s */
#define ELEV_HOLD_MS   5000U    /* 頂樓持平 */
#define ELEV_FALL_MS   20000U   /* 30m ÷ 1.5 m/s */
#define ELEV_TOP_M     30.0f
#define ELEV_RATE_MPS  1.5f

static float elevator_profile(uint32_t t) {
    if (t < ELEV_T0) return 0.0f;
    uint32_t dt = t - ELEV_T0;
    if (dt < ELEV_RISE_MS) return ELEV_TOP_M * (float)dt / (float)ELEV_RISE_MS;
    dt -= ELEV_RISE_MS;
    if (dt < ELEV_HOLD_MS) return ELEV_TOP_M;
    dt -= ELEV_HOLD_MS;
    if (dt < ELEV_FALL_MS) return ELEV_TOP_M - ELEV_TOP_M * (float)dt / (float)ELEV_FALL_MS;
    return 0.0f;
}

static float elevator_velocity(uint32_t t) {
    if (t < ELEV_T0) return 0.0f;
    uint32_t dt = t - ELEV_T0;
    if (dt < ELEV_RISE_MS) return ELEV_RATE_MPS;
    dt -= ELEV_RISE_MS;
    if (dt < ELEV_HOLD_MS) return 0.0f;
    dt -= ELEV_HOLD_MS;
    if (dt < ELEV_FALL_MS) return -ELEV_RATE_MPS;
    return 0.0f;
}

/* ---------------------------------------------------------------- */
/*
 * [1] 標稱電梯行程 —— 黃金值來自實際執行 FSM（電梯改用 VF 後，COAST 頂點判定
 * 由路徑2（速度過零）主導，非舊版純 baro 趨勢檢查，故時序整段改變，不再手推）：
 *   liftoff：baro>FSM_LIFTOFF_BARO_ALT_M(5.0，原 3.0m) 首次成立於
 *            (t-2000)*1.5/1000>5.0 → t>5333.33 → 首個 10ms 網格點 t=5340
 *            （baro=5.01；三冗餘 OR 中 baro 最先達門檻，與 est_healthy 無關）。
 *   burnout：a_z=1.0<2.0（電梯恆成立）僅受 FSM_BURNOUT_MIN_MS(4000，原 1500ms)
 *            時間鎖，state_entered=liftoff(5340)+4000+10=9350。
 *   apogee ：頂樓持平段 v_est=0，一進入下降段(t=27000) v_est 立即為 -1.5（< -0.2
 *            門檻），連續 5 週期(50ms)確認 → 首次於 t=27000 成立，第5週期
 *            t=27040 點火——遠早於舊版純 baro 趨勢檢查（需先回落 2m 又連續
 *            40+5 週期，約 t=28770）。此時序不受 liftoff/burnout 門檻調整影響
 *            （由絕對時刻 t 驅動之 elevator_profile()/elevator_velocity() 決定，
 *            與燒完轉移發生的實際時刻無關）。
 *   main   ：動態高度公式 h_trigger=TARGET_MAIN_ALTITUDE(10)+|v_fall|·
 *            MAIN_DEPLOY_DELAY_S(3.5)=10+1.5×3.5=15.25m（電梯改用 VF 後不再是
 *            舊版固定 FSM_FB_MAIN_ALT_M=8m）。下降段 h(t)=30−1.5(t−27000)/1000，
 *            h<=15.25 於 dt>=9833.33 即 t>=36833.33，首個 10ms 網格點 t=36840
 *            （h=15.24）；連續 5 週期確認 → t=36880 觸發（同上，不受 liftoff/
 *            burnout 調整影響）。
 *   其餘（drogue_done/main_open/touchdown）為上述鏈式推進的結果。
 * 斷言以 near_ms 容忍（±1~2 步階），數值見下方 EXPECT_* 常數（皆已對照實際
 * 執行輸出核對，非純手推）。
 */
#define EXPECT_LIFTOFF     5340U
#define EXPECT_BURNOUT     9350U
#define EXPECT_APOGEE      27040U
#define EXPECT_MAIN        36880U

static void test_nominal_elevator(void) {
    printf("[1] 標稱電梯行程（電梯改用 VF：est_healthy=1，路徑1關閉）\n");
    Sim_t s;
    sim_init(&s, STATE_PAD, 0, 0, 0);

    const uint32_t END_MS = 51000U;
    while (s.now < END_MS) {
        s.in.baro_alt_rel = elevator_profile(s.now);
        s.in.h_est        = elevator_profile(s.now);
        s.in.v_est         = elevator_velocity(s.now);
        sim_step(&s);
    }

    check("LIFTOFF 於 baro>5.0m (t=5340)", s.t_liftoff == EXPECT_LIFTOFF && s.fire_n <= 1);
    check("BURNOUT 受 4000ms 時間鎖 (t=9350)", near_ms(s.t_burnout, EXPECT_BURNOUT, 10));
    check("APOGEE 於下降開始+速度過零 5 週期 (t=27040)", near_ms(s.t_apogee, EXPECT_APOGEE, 20));
    check("到達頂點轉移至 APOGEE（is_past_peak 立即成立，+10~20ms 內）",
          near_ms(s.t_drogue_done, s.t_apogee, 30U));
    check("MAIN 部署於動態高度 h<=15.25m + 5週期防雜訊確認", near_ms(s.t_main, EXPECT_MAIN, 20));
    check("MAIN_OPEN 於 +3000ms", near_ms(s.t_main_open, s.t_main + 3000U, 10));
    check("狀態機走完整序列至 LANDED", s.ctx.state == STATE_LANDED);
    check("全程四動作各恰一次",
          s.fire_n == 1 && s.release_n == 1 && s.main_n == 1 && s.buzzer_n == 1);
}

/* ---------------------------------------------------------------- */
static void test_interference_immunity(void) {
    printf("[2] 干擾免疫（電梯 profile，est_healthy=1）\n");
    Sim_t s;

    /* 2a. 頂樓持平期單週期 +2.5m baro 尖刺：consec 40（400ms）防護——
     * 尖刺後短時間窗（<400ms）內不得觸發。h_est/v_est 維持「真的在持平」
     * （v_est=0，不隨 baro 尖刺擾動），故此測試仍是純 baro 路徑(path4)的免疫測試。 */
    sim_init(&s, STATE_COAST, 20000, 20000 - 5000, 0);  /* 時間鎖已過 */
    s.in.baro_alt_rel = 30.0f;
    s.in.h_est = 30.0f;
    s.in.v_est = 0.0f;
    for (int i = 0; i < 50; i++) sim_step(&s);   /* 建立穩定頂樓峰值 30.0 */
    check("頂樓穩定持平不誤觸", s.fire_n == 0 && s.ctx.state == STATE_COAST);
    s.in.baro_alt_rel = 32.5f;                    /* 單週期尖刺（h_est/v_est 不隨之擾動） */
    sim_step(&s);
    s.in.baro_alt_rel = 30.0f;                    /* 立即回穩 */
    for (int i = 0; i < 30; i++) sim_step(&s);     /* 尖刺後 300ms（<40 週期 consec 門檻）*/
    check("單週期尖刺後 300ms 內未觸發（consec 40 防護生效中）",
          s.fire_n == 0 && s.ctx.state == STATE_COAST);

    /* 2b. 上升期 baro ±0.5m 交替抖動（遠低於電梯 2.0m 門檻）：不得誤判頂點。
     * h_est/v_est 餵「真實」上升值（v_est=+1.5，h_est 不含抖動），代表 VF 本身
     * 已濾掉這種量級的量測雜訊；baro 抖動只驗證 path4 的獨立雜訊免疫。 */
    sim_init(&s, STATE_COAST, 10000, 10000 - 5000, 0);  /* 時間鎖已過 */
    {
        float base = 10.0f;
        for (int i = 0; i < 300; i++) {   /* 3s：上升 1.5m/s + baro ±0.5m 抖動 */
            base += 1.5f * (float)FSM_STEP_PERIOD_MS / 1000.0f;
            s.in.baro_alt_rel = base + ((i % 2) ? 0.5f : -0.5f);
            s.in.h_est = base;
            s.in.v_est = 1.5f;
            sim_step(&s);
        }
    }
    check("上升期抖動不誤判頂點", s.fire_n == 0 && s.ctx.state == STATE_COAST);

    /* 2c. PAD 期 baro 抖動 ±2m（皆 < FSM_LIFTOFF_BARO_ALT_M=3.0m 門檻）：不得誤起飛。
     * h_est 維持 0（預設），遠低於 FSM_LIFTOFF_ALT_M(10m)，兩路徑皆不誤觸。 */
    sim_init(&s, STATE_PAD, 0, 0, 0);
    for (int i = 0; i < 400; i++) {   /* 4s */
        s.in.baro_alt_rel = ((i % 2) ? 2.0f : -2.0f);
        sim_step(&s);
    }
    check("PAD 抖動（<3m 門檻）不誤起飛", s.ctx.state == STATE_PAD_ARMED && s.t_liftoff == 0);
}

/* ---------------------------------------------------------------- */
static void test_dynamic_predict_disabled(void) {
    printf("[3] 路徑1（動態預測）對電梯關閉：真實等速爬升全程不誤觸發\n");
    Sim_t s;

    /* 真實電梯爬升 10 秒（+1.5m/s，5m→20m）+ 頂樓持平 3 秒（v_est=0）。
     * 若路徑1未關閉，COAST 進入後 decel fallback（−9.80665）會使
     * t_to_apogee=v_est/9.80665≈0.15s 遠低於 lead time，幾乎立即誤點火
     * （曾實測：t=40ms 誤觸發）。本測試鎖定「全程 13 秒不得有任何一次
     * fire_drogue」。 */
    sim_init(&s, STATE_COAST, 10000, 10000 - 5000, 0);   /* 時間鎖已過 */
    float alt = 5.0f;
    for (int i = 0; i < 1000; i++) {   /* 10 秒爬升：5m -> 20m，v_est=+1.5 */
        alt += ELEV_RATE_MPS * (float)FSM_STEP_PERIOD_MS / 1000.0f;
        s.in.baro_alt_rel = alt;
        s.in.h_est        = alt;
        s.in.v_est        = ELEV_RATE_MPS;
        sim_step(&s);
    }
    check("等速爬升 10 秒（+1.5m/s）不誤觸發（路徑1已對電梯關閉）",
          s.fire_n == 0 && s.ctx.state == STATE_COAST);

    for (int i = 0; i < 300; i++) {   /* 頂樓持平 3 秒：v_est=0 */
        s.in.baro_alt_rel = alt;
        s.in.h_est        = alt;
        s.in.v_est        = 0.0f;
        sim_step(&s);
    }
    check("頂樓持平 3 秒（v_est=0）不誤觸發", s.fire_n == 0 && s.ctx.state == STATE_COAST);
}

/* ---------------------------------------------------------------- */
static void test_path2_still_fires_on_real_descent(void) {
    printf("[4] 路徑2（速度過零）正確運作：真實下降開始後才觸發頂點/主傘\n");
    Sim_t s;

    /* 承接 [3] 的爬升+持平剖面，接著真的開始下降（v_est=-1.5），驗證：
     *   (a) 下降開始前（爬升+持平）全程未觸發（見 [3]，此處重跑一次確保獨立）
     *   (b) 下降開始後，路徑2（v_est<-0.2）於 5 週期內正確觸發頂點
     *   (c) 隨後動態主傘高度亦能正確觸發、走完整序列 */
    sim_init(&s, STATE_COAST, 10000, 10000 - 5000, 0);
    float alt = 5.0f;
    for (int i = 0; i < 1000; i++) {
        alt += ELEV_RATE_MPS * (float)FSM_STEP_PERIOD_MS / 1000.0f;
        s.in.baro_alt_rel = alt; s.in.h_est = alt; s.in.v_est = ELEV_RATE_MPS;
        sim_step(&s);
    }
    for (int i = 0; i < 300; i++) {
        s.in.baro_alt_rel = alt; s.in.h_est = alt; s.in.v_est = 0.0f;
        sim_step(&s);
    }
    check("爬升+持平未誤觸發（前置條件）", s.fire_n == 0 && s.ctx.state == STATE_COAST);

    uint32_t descent_start = s.now;
    for (int i = 0; i < 1500 && s.ctx.state != STATE_LANDED; i++) {   /* 最多 15 秒下降 */
        alt -= ELEV_RATE_MPS * (float)FSM_STEP_PERIOD_MS / 1000.0f;
        if (alt < 0.0f) alt = 0.0f;
        s.in.baro_alt_rel = alt; s.in.h_est = alt; s.in.v_est = -ELEV_RATE_MPS;
        sim_step(&s);
    }
    check("下降開始後正確觸發頂點（路徑2）", s.fire_n == 1 && near_ms(s.t_apogee, descent_start, 60U));
    check("隨後進入 DESCENT 並觸發主傘（動態高度）", s.main_n == 1);
}

/* ---------------------------------------------------------------- */
int main(void) {
    printf("=== test_fsm_elevator：電梯場測 profile 黃金剖面（FLIGHT_PROFILE_ELEVATOR=1） ===\n");
    test_nominal_elevator();
    test_interference_immunity();
    test_dynamic_predict_disabled();
    test_path2_still_fires_on_real_descent();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
