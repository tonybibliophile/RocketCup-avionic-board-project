/*
 * servo_arb.h — 主傘 PD14 共開時序（純邏輯，host 可測）
 * ===========================================================================
 * ★ 本模組原為「主傘舵機時間錯開互斥握手」（D2）。主傘機構改版後 **不再驅動 PWM 舵機**：
 *   主/副兩板判到主傘高度時，各自把 PD14 以純 GPIO 拉高 SERVO_MAIN_HIGH_MS（1.5s）後拉回低。
 *   兩板 PD14 在開傘板以 diode 做 OR-in gate —— 會互相破壞的是兩串非同相 50Hz PWM 脈衝，
 *   純準位訊號同時拉高完全無害。故互斥握手（INTENT/guard/讓位/PREHIGH→PWM 兩段式）
 *   整段取消，改為 **兩板同時共開**：
 *     - 任一板 FSM 判到主傘（TARGET_MAIN_ALTITUDE，仍是 300m）→ 立刻拉高，
 *       同時經板間鏈路廣播 MAIN_HIGH「呼叫另一板一起開」。
 *     - 對端收到 MAIN_HIGH/DONE → 立刻跟著拉高（不等待、不讓位、不需握手隔離）。
 *       台上誤觸防護由呼叫端的 in-flight 閘負責（見 main.c：本板須已進入 STATE_COAST
 *       之後才接受對端呼叫），不再靠握手時序。
 *   檔名與 LinkPacket.main_arb 欄位沿用舊名，避免動到板間封包格式與 GUI 解析器。
 *
 * 不依賴 HAL/RTOS；HAL 端（Servo_MainHigh / Servo_HoldLow、廣播 main_arb）由 main.c 接線。
 * 逐 cycle 行為見 tests/test_servo_arb.c。
 */
#ifndef SERVO_ARB_H
#define SERVO_ARB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 主傘共開 / BENCH 協同狀態（在板間鏈路 LinkPacket.main_arb 廣播；對端據此跟著動作）。
 * 數值刻意沿用舊互斥握手的列舉值，避免動到板間封包格式：
 *   1 原為 INTENT，改作「BENCH 序列開始宣告」（副板據此自動跟隨進入自測）。
 *   2 原為 DRIVING（PWM 驅動中），已隨 PWM 一起廢除，新韌體永不廣播。 */
#define SERVO_ARB_MSG_NONE           0U   /* 未參與（IDLE） */
#define SERVO_ARB_MSG_BENCH_START    1U   /* BENCH：序列開始宣告（副板據此跟隨） */
#define SERVO_ARB_MSG_LEGACY_DRIVING 2U   /* 【已廢除】舊互斥握手：PWM 驅動中 */
#define SERVO_ARB_MSG_DONE           3U   /* 拉高窗結束、PD14 已回低（持續廣播） */
#define SERVO_ARB_MSG_BENCH_PRI_FIRE 4U   /* BENCH 階段 1a：主板 PD13 通電中 */
#define SERVO_ARB_MSG_BENCH_SEC_FIRE 5U   /* BENCH 階段 1c：副板 PD13 通電中（主板已通知） */
#define SERVO_ARB_MSG_BENCH_MAIN_HIGH 6U  /* BENCH 階段 3：雙板 PD14 同時拉高（共開） */
#define SERVO_ARB_MSG_MAIN_HIGH      7U   /* 飛行：PD14 純 GPIO 拉高中（＝呼叫對端一起開） */

typedef enum {
    SERVO_ARB_IDLE = 0,   /* FSM 尚未要求主傘 */
    SERVO_ARB_HIGH = 1,   /* PD14 純 GPIO 拉高中，持續 high_ms */
    SERVO_ARB_DONE = 2    /* 拉高窗結束、已回低（一次性，不重複拉高） */
} ServoArbState_t;

typedef struct {
    ServoArbState_t state;
    uint32_t        entered_ms;   /* 進入當前狀態的 tick */
} ServoArb_t;

typedef struct {
    uint8_t high_servo;     /* 1 = 本週期 PD14 應為拉高（HIGH 期間持續 1） */
    uint8_t release_servo;  /* 1 = 本週期拉回低（HIGH→DONE 交界發一次） */
    uint8_t broadcast_arb;  /* 要廣播的 SERVO_ARB_MSG_*（填入 LinkStatus.main_arb） */
} ServoArbAction_t;

/* 初始化（無角色差異：主/副兩板行為完全對稱，同時共開）。 */
void ServoArb_Init(ServoArb_t *ctx);

/**
 * @brief 每飛控週期呼叫一次。回傳本週期 PD14 動作與要廣播的狀態。
 * @param want     本板是否要開主傘。呼叫端把「自身 FSM 判定」與「對端呼叫（peer_arb ==
 *                 MAIN_HIGH/DONE 且本板已在飛行中）」OR 起來後鎖存傳入（一旦 1 應維持 1）。
 * @param now_ms   當前 tick（uint32 無號相減，自然處理 wrap）。
 * @param high_ms  PD14 拉高時間（SERVO_MAIN_HIGH_MS，1500ms）。
 * @note want 上升緣即刻進入 HIGH —— 不等 guard、不讓位，兩板可（且應該）同時拉高。
 */
ServoArbAction_t ServoArb_Step(ServoArb_t *ctx, uint8_t want, uint32_t now_ms,
                               uint32_t high_ms);

/* SERVO_ARB_MSG_* → 短字串，供 [LINK] 診斷列印與地面站解析。 */
const char *ServoArb_MsgName(uint8_t msg);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_ARB_H */
