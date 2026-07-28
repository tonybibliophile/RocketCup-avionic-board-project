/*
 * servo_arb.h — 主傘舵機時間錯開互斥握手（純邏輯，host 可測）
 * ===========================================================================
 * 為什麼需要：主/副兩板共用同一塊開傘板，主傘舵機 PD14(TIM4_CH3 PWM) 以 diode 做
 * OR-in gate。兩路 PWM「同時」驅動會在 diode-OR 產生被合併破壞的脈衝（兩個非同相
 * 50Hz 脈衝串 OR → 脈寬亂掉、舵機亂轉/超程）。故舵機**同一時間只能一板驅動**，
 * 必須時間錯開。副傘 PD13（DC 馬達準位訊號）不受此限，走 fsm.c 的加法 OR（D1）。
 *
 * 本模組是一個對稱互斥握手（主/副跑同一份）：
 *   優先序：主板先驅動 4s → 廣播 DONE → 副板才驅動 4s（正常「主先副後 +4s」）。
 *   容錯：主板失效/鏈路斷（對端 arb 視為 NONE）→ 副板經 1s guard 自行驅動（主傘仍開）。
 *   安全不變量：任一時刻至多一板 drive_servo=1（見 tests/test_servo_arb.c 逐 cycle 斷言）。
 *
 * 不依賴 HAL/RTOS；HAL 端（PWM 驅動 Servo_DeployMain / 釋放 Servo_HoldLow、
 * 廣播 main_arb）由 main.c 接線。arm-interlock（峰值高度 > 目標才允許 want）在
 * main.c 計算後以 want 傳入 —— 彈射台 baro≈0 永不 want，擋台上誤驅/spoof。
 */
#ifndef SERVO_ARB_H
#define SERVO_ARB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 握手訊息（在板間鏈路 LinkPacket.main_arb 廣播；對端據此讓位/接手） */
#define SERVO_ARB_MSG_NONE           0U   /* 未參與（IDLE） */
#define SERVO_ARB_MSG_INTENT         1U   /* 想驅動、arbitration 中（WANT） */
#define SERVO_ARB_MSG_DRIVING        2U   /* 正在驅動 4s 窗（DRIVE） */
#define SERVO_ARB_MSG_DONE           3U   /* 4s 完成、已釋放 PWM（DONE） */
#define SERVO_ARB_MSG_BENCH_PRI_FIRE 4U   /* BENCH 階段 1a：主板單獨點火 (2s) */
#define SERVO_ARB_MSG_BENCH_SEC_FIRE 5U   /* BENCH 階段 1b：副板單獨點火 (2s) */
#define SERVO_ARB_MSG_BENCH_BOTH_FIRE 6U  /* BENCH 階段 1c：雙板同時點火 (4s) */

typedef enum {
    SERVO_ARB_IDLE  = 0,   /* FSM 尚未要求主傘 */
    SERVO_ARB_WANT  = 1,   /* 廣播 INTENT，guard 計時 + 依優先序讓位 */
    SERVO_ARB_DRIVE = 2,   /* 驅動舵機（180°）持續 drive_ms */
    SERVO_ARB_DONE  = 3    /* 完成並釋放 PWM（舵機鬆弛停在 180°，飛行不回 0°） */
} ServoArbState_t;

typedef struct {
    ServoArbState_t state;
    uint32_t        entered_ms;   /* 進入當前狀態/最近一次刷新 guard 的 tick */
    uint8_t         is_primary;   /* 優先序：主板(1) 先驅動；副板(0) 讓位主板 */
} ServoArb_t;

typedef struct {
    uint8_t drive_servo;    /* 1 = 本週期驅動舵機到部署角（DRIVE 期間持續 1） */
    uint8_t release_servo;  /* 1 = 本週期釋放 PWM（DRIVE→DONE 交界發一次） */
    uint8_t broadcast_arb;  /* 要廣播的 SERVO_ARB_MSG_*（填入 LinkStatus.main_arb） */
} ServoArbAction_t;

/* 初始化。is_primary 決定優先序（IS_PRIMARY）。 */
void ServoArb_Init(ServoArb_t *ctx, uint8_t is_primary);

/**
 * @brief 每飛控週期呼叫一次。回傳本週期的舵機動作與要廣播的握手狀態。
 * @param want     本板是否想驅動主傘（已含 arm-interlock；main.c 一旦 1 應維持 1）。
 * @param peer_arb 對端最近廣播的 SERVO_ARB_MSG_*。**對端失聯時呼叫端須傳 NONE**
 *                 （不可信任 stale arb）—— 這正是主板失效時副板自行接手的依據。
 * @param now_ms   當前 tick。
 * @param guard_ms 讓位/聆聽 guard（SERVO_ARB_GUARD_MS，如 1000ms）。
 * @param drive_ms 舵機驅動窗（SERVO_MAIN_DRIVE_MS，如 4000ms）。
 * @note  互斥保證：只有 DRIVE 態輸出 drive_servo=1；進入 DRIVE 需 guard 到期且對端
 *        未在 DRIVING、且（對端非 INTENT 或本板有優先權）。優先序打破同時競爭之平手。
 */
ServoArbAction_t ServoArb_Step(ServoArb_t *ctx, uint8_t want, uint8_t peer_arb,
                               uint32_t now_ms, uint32_t guard_ms, uint32_t drive_ms);

/* SERVO_ARB_MSG_* → 短字串（"NONE"/"INTENT"/"DRIVING"/"DONE"），供 [LINK] 診斷列印。 */
const char *ServoArb_MsgName(uint8_t msg);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_ARB_H */
