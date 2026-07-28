/*
 * servo_arb.c — 主傘舵機時間錯開互斥握手（純邏輯，host 可測）
 * ===========================================================================
 * 無 HAL/RTOS 依賴。tick 差以 uint32_t 無號相減，自然處理 wrap。
 * 詳見 servo_arb.h 的設計說明與安全不變量。
 */
#include "servo_arb.h"
#include <string.h>

void ServoArb_Init(ServoArb_t *ctx, uint8_t is_primary)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state      = SERVO_ARB_IDLE;
    ctx->is_primary = is_primary ? 1U : 0U;
}

ServoArbAction_t ServoArb_Step(ServoArb_t *ctx, uint8_t want, uint8_t peer_arb,
                               uint32_t now_ms, uint32_t guard_ms, uint32_t drive_ms)
{
    ServoArbAction_t out;
    out.drive_servo   = 0U;
    out.release_servo = 0U;
    out.broadcast_arb = SERVO_ARB_MSG_NONE;

    switch (ctx->state) {
        case SERVO_ARB_IDLE:
            if (want) {
                ctx->state      = SERVO_ARB_WANT;
                ctx->entered_ms = now_ms;
            }
            break;

        case SERVO_ARB_WANT: {
            out.broadcast_arb = SERVO_ARB_MSG_INTENT;   /* 宣告意圖，讓對端得知 */

            /* 讓位規則（確保 diode-OR PWM 不打架）：
             *   - 對端正在 DRIVING → 一定讓位（絕不與對端同時驅動）。
             *   - 對端也在 INTENT 且對端優先序較高（我是副板）→ 讓位（主先副後）。
             * 讓位期間刷新 guard 起點，待對端清空後才重新聆聽 guard_ms。 */
            uint8_t peer_driving = (peer_arb == SERVO_ARB_MSG_DRIVING);
            uint8_t peer_intent  = (peer_arb == SERVO_ARB_MSG_INTENT);
            uint8_t must_yield    = peer_driving || (peer_intent && !ctx->is_primary);

            if (must_yield) {
                ctx->entered_ms = now_ms;               /* 對端活動中 → guard 不計時 */
            } else if ((now_ms - ctx->entered_ms) >= guard_ms) {
                ctx->state      = SERVO_ARB_DRIVE;      /* guard 到期、對端未擋 → 取得舵機 */
                ctx->entered_ms = now_ms;
            }
            break;
        }

        case SERVO_ARB_DRIVE:
            out.broadcast_arb = SERVO_ARB_MSG_DRIVING;
            out.drive_servo   = 1U;                     /* 驅動舵機到部署角（180°） */
            if ((now_ms - ctx->entered_ms) >= drive_ms) {
                out.drive_servo   = 0U;
                out.release_servo = 1U;                 /* 4s 到 → 釋放 PWM（交界發一次） */
                ctx->state        = SERVO_ARB_DONE;
                ctx->entered_ms   = now_ms;
            }
            break;

        case SERVO_ARB_DONE:
        default:
            out.broadcast_arb = SERVO_ARB_MSG_DONE;     /* 持續廣播 DONE：對端據此接手其 4s */
            break;
    }

    return out;
}

const char *ServoArb_MsgName(uint8_t msg)
{
    switch (msg) {
        case SERVO_ARB_MSG_INTENT:         return "INTENT";
        case SERVO_ARB_MSG_DRIVING:        return "DRIVING";
        case SERVO_ARB_MSG_DONE:           return "DONE";
        case SERVO_ARB_MSG_BENCH_PRI_FIRE: return "BENCH_PRI_FIRE";
        case SERVO_ARB_MSG_BENCH_SEC_FIRE: return "BENCH_SEC_FIRE";
        case SERVO_ARB_MSG_BENCH_BOTH_FIRE:return "BENCH_BOTH_FIRE";
        default:                           return "NONE";
    }
}
