/*
 * test_servo_arb.c — 主傘 PD14 共開時序情境測試（純 host 編譯）
 * ===========================================================================
 *   cd tests && make run
 *
 * ★ 舊版本測的是「互斥握手」（兩板絕不同時佔用）。主傘改為不啟 PWM、純 GPIO 拉高
 *   1.5s 後，互斥的前提整個消失——現在要鎖定的是相反的性質：**兩板同時共開**。
 *
 * 兩板各跑一份 ServoArb，逐 cycle 交換 broadcast_arb（模擬 1 週期鏈路延遲），
 * 並複製 main.c 的接線：對端廣播 MAIN_HIGH/DONE 且本板已在飛行中 → 本板也 want。
 * 鎖定：
 *   [1] 共開：兩板同時判到 → 同一 cycle 一起拉高，各拉滿 HIGH_MS、各釋放一次
 *   [2] 呼叫對端：只有主板判到 → 副板經鏈路被呼叫，僅落後 1 週期就跟上（無 guard）
 *   [3] 鏈路斷：自身判到者照樣拉高（單板仍開主傘）
 *   [4] 台上安全：want=0 / in-flight 閘未成立 → 不拉高，對端訊號也拉不動
 *   [5] 一次性：拉高窗結束回 DONE，不再重複拉高
 *   [6] tick wrap：跨 uint32 溢位仍準確拉滿 HIGH_MS
 */
#include <stdio.h>
#include <string.h>
#include "servo_arb.h"

static int g_fail = 0, g_total = 0;
static void check(const char *n, int c) {
    g_total++;
    if (c) printf("  [PASS] %s\n", n);
    else { printf("  [FAIL] %s\n", n); g_fail++; }
}

#define HIGH_MS  1500U   /* 對應 board_config.h 的 SERVO_MAIN_HIGH_MS */
#define STEP       10U   /* 飛控週期 100Hz */

/* 拉高窗涵蓋的週期數：t 進入 HIGH，t+HIGH_MS 那一拍才釋放 → 恰 HIGH_MS/STEP 拍。 */
#define EXPECT_HIGH_CYCLES  ((int)(HIGH_MS / STEP))

typedef struct {
    ServoArb_t arb;
    uint8_t  last_bcast;       /* 上一週期廣播（對端本週期觀察到的值） */
    uint8_t  want_latched;     /* 對應 main.c 的 g_servo_want_latched */
    uint8_t  in_flight;        /* 對應 main.c 的 in-flight 閘（state >= STATE_COAST） */
    int      highs;            /* 累計 high_servo=1 的週期數 */
    int      releases;
    uint32_t first_high_ms, last_high_ms;
} Board;

static void board_init(Board *b, uint8_t in_flight) {
    memset(b, 0, sizeof(*b));
    ServoArb_Init(&b->arb);
    b->last_bcast    = SERVO_ARB_MSG_NONE;
    b->in_flight     = in_flight;
    b->first_high_ms = 0xFFFFFFFFu;
}

/* 跑兩板共開模擬（複製 main.c 的 want 接線）。
 *   wa / wb : 該板 FSM 是否判到主傘（act.deploy_main，全程維持）
 *   link_up : 0 表兩板互相看不到（模擬對端斷電/鏈路斷）
 * 回傳「是否曾有任一 cycle 兩板同時拉高」——新設計下這是期望發生的事。 */
static int run_two(Board *A, Board *B, uint8_t wa, uint8_t wb, uint8_t link_up,
                   uint32_t t0, uint32_t dur_ms) {
    int both_ever = 0;
    for (uint32_t elapsed = 0; elapsed < dur_ms; elapsed += STEP) {
        uint32_t t = t0 + elapsed;                                   /* 刻意讓 t 自然 wrap */
        uint8_t pa = link_up ? B->last_bcast : SERVO_ARB_MSG_NONE;   /* A 看到 B 上一輪廣播 */
        uint8_t pb = link_up ? A->last_bcast : SERVO_ARB_MSG_NONE;

        /* --- main.c 接線：自身判定 OR 對端呼叫（含 in-flight 閘），一旦成立即鎖存 --- */
        if (wa) A->want_latched = 1U;
        if (wb) B->want_latched = 1U;
        if (A->in_flight && (pa == SERVO_ARB_MSG_MAIN_HIGH || pa == SERVO_ARB_MSG_DONE))
            A->want_latched = 1U;
        if (B->in_flight && (pb == SERVO_ARB_MSG_MAIN_HIGH || pb == SERVO_ARB_MSG_DONE))
            B->want_latched = 1U;

        ServoArbAction_t oa = ServoArb_Step(&A->arb, A->want_latched, t, HIGH_MS);
        ServoArbAction_t ob = ServoArb_Step(&B->arb, B->want_latched, t, HIGH_MS);

        if (oa.high_servo && ob.high_servo) both_ever = 1;
        if (oa.high_servo) { A->highs++; if (A->first_high_ms == 0xFFFFFFFFu) A->first_high_ms = elapsed; A->last_high_ms = elapsed; }
        if (ob.high_servo) { B->highs++; if (B->first_high_ms == 0xFFFFFFFFu) B->first_high_ms = elapsed; B->last_high_ms = elapsed; }
        A->releases += oa.release_servo; B->releases += ob.release_servo;
        A->last_bcast = oa.broadcast_arb; B->last_bcast = ob.broadcast_arb;
    }
    return both_ever;
}

static void test_cofire(void) {
    printf("[1] 兩板同時判到 → 同時共開（無讓位、無 guard）\n");
    Board A, B; board_init(&A, 1); board_init(&B, 1);
    int both = run_two(&A, &B, 1, 1, 1, 0, 6000);
    check("兩板曾同時拉高（共開，正是設計目的）", both == 1);
    check("兩板同一 cycle 起拉高", A.first_high_ms == 0 && B.first_high_ms == 0);
    check("主板拉滿 1.5s", A.highs == EXPECT_HIGH_CYCLES);
    check("副板拉滿 1.5s", B.highs == EXPECT_HIGH_CYCLES);
    check("各釋放恰一次", A.releases == 1 && B.releases == 1);
}

static void test_peer_call(void) {
    printf("[2] 只有主板判到 → 經鏈路呼叫副板一起開\n");
    Board A, B; board_init(&A, 1); board_init(&B, 1);
    run_two(&A, &B, 1, 0, 1, 0, 6000);
    check("主板拉滿 1.5s", A.highs == EXPECT_HIGH_CYCLES);
    check("副板被呼叫後也拉滿 1.5s", B.highs == EXPECT_HIGH_CYCLES);
    /* 1 週期鏈路延遲：A 於 t=0 廣播 MAIN_HIGH，B 於 t=10 看到即拉高（不是等 guard）。 */
    check("副板僅落後 1 週期（無 guard 延遲）", B.first_high_ms == A.first_high_ms + STEP);
    check("兩板拉高窗幾乎完全重疊", B.first_high_ms < A.last_high_ms);
}

static void test_link_down(void) {
    printf("[3] 鏈路斷：自身判到者照樣拉高（單板仍開主傘）\n");
    Board A, B; board_init(&A, 1); board_init(&B, 1);
    run_two(&A, &B, 1, 0, 0, 0, 6000);
    check("主板照常拉滿 1.5s", A.highs == EXPECT_HIGH_CYCLES && A.releases == 1);
    check("副板聽不到 → 不拉高（自身未判到）", B.highs == 0 && B.releases == 0);
}

static void test_pad_safe(void) {
    printf("[4] 台上安全：in-flight 閘未成立 → 對端訊號拉不動\n");
    Board A, B; board_init(&A, 1); board_init(&B, 0);   /* B 仍在 PAD：in_flight=0 */
    run_two(&A, &B, 1, 0, 1, 0, 6000);
    check("主板拉高", A.highs == EXPECT_HIGH_CYCLES);
    check("副板未起飛 → 不被對端拉動", B.highs == 0 && B.releases == 0);

    Board C, D; board_init(&C, 1); board_init(&D, 1);
    run_two(&C, &D, 0, 0, 1, 0, 6000);
    check("兩板皆未判到 → 皆不拉高", C.highs == 0 && D.highs == 0);
    check("兩板皆不釋放", C.releases == 0 && D.releases == 0);
}

static void test_once_only(void) {
    printf("[5] 一次性：拉高窗結束後不再重複拉高\n");
    Board A, B; board_init(&A, 1); board_init(&B, 1);
    run_two(&A, &B, 1, 1, 1, 0, 20000);   /* want 全程鎖存為 1，跑遠超過一個窗 */
    check("主板僅拉高一個窗", A.highs == EXPECT_HIGH_CYCLES && A.releases == 1);
    check("副板僅拉高一個窗", B.highs == EXPECT_HIGH_CYCLES && B.releases == 1);
    check("結束後持續廣播 DONE", A.last_bcast == SERVO_ARB_MSG_DONE &&
                                  B.last_bcast == SERVO_ARB_MSG_DONE);
}

static void test_tick_wrap(void) {
    printf("[6] tick 跨 uint32 溢位仍準確拉滿 1.5s\n");
    Board A, B; board_init(&A, 1); board_init(&B, 1);
    run_two(&A, &B, 1, 1, 1, 0xFFFFF000u, 6000);   /* 起點距溢位僅 4096ms */
    check("跨 wrap 仍恰好拉滿 1.5s", A.highs == EXPECT_HIGH_CYCLES && A.releases == 1);
    check("對端同樣不受 wrap 影響", B.highs == EXPECT_HIGH_CYCLES && B.releases == 1);
}

int main(void) {
    printf("=== test_servo_arb：主傘 PD14 共開時序（HIGH_MS=%u，已無互斥握手）===\n",
           (unsigned)HIGH_MS);
    test_cofire();
    test_peer_call();
    test_link_down();
    test_pad_safe();
    test_once_only();
    test_tick_wrap();
    printf("----------------------------------------\n");
    printf("%s：%d/%d 通過\n", g_fail ? "FAIL" : "ALL PASS", g_total - g_fail, g_total);
    return g_fail ? 1 : 0;
}
