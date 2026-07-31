/*
 * servo_arb.c — 主傘 PD14 共開時序（純邏輯，host 可測）
 * ===========================================================================
 * 無 HAL/RTOS 依賴。tick 差以 uint32_t 無號相減，自然處理 wrap。
 * 詳見 servo_arb.h：互斥握手已取消，兩板同時拉高 1.5s（不再輸出 PWM）。
 */
#include "servo_arb.h"
#include <string.h>

void ServoArb_Init(ServoArb_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = SERVO_ARB_IDLE;
}

ServoArbAction_t ServoArb_Step(ServoArb_t *ctx, uint8_t want, uint32_t now_ms,
                               uint32_t high_ms)
{
    ServoArbAction_t out;
    out.high_servo    = 0U;
    out.release_servo = 0U;
    out.broadcast_arb = SERVO_ARB_MSG_NONE;

    switch (ctx->state) {
        case SERVO_ARB_IDLE:
            if (want) {
                /* 立刻拉高：不等 guard、不看對端 —— 兩板同時共開即為設計目的。
                 * 同一週期就輸出 high_servo/廣播 MAIN_HIGH，對端據此跟著開。 */
                ctx->state      = SERVO_ARB_HIGH;
                ctx->entered_ms = now_ms;
                out.high_servo    = 1U;
                out.broadcast_arb = SERVO_ARB_MSG_MAIN_HIGH;
            }
            break;

        case SERVO_ARB_HIGH:
            out.broadcast_arb = SERVO_ARB_MSG_MAIN_HIGH;   /* 持續呼叫對端一起開 */
            out.high_servo    = 1U;
            if ((now_ms - ctx->entered_ms) >= high_ms) {
                out.high_servo    = 0U;
                out.release_servo = 1U;                    /* 1.5s 到 → 拉回低（交界發一次） */
                ctx->state        = SERVO_ARB_DONE;
                ctx->entered_ms   = now_ms;
            }
            break;

        case SERVO_ARB_DONE:
        default:
            /* 持續廣播 DONE：對端若尚未開（例如剛上電/剛恢復鏈路）仍據此補開一次。 */
            out.broadcast_arb = SERVO_ARB_MSG_DONE;
            break;
    }

    return out;
}

const char *ServoArb_MsgName(uint8_t msg)
{
    switch (msg) {
        case SERVO_ARB_MSG_BENCH_START:     return "BENCH_START";
        case SERVO_ARB_MSG_LEGACY_DRIVING:  return "LEGACY_DRIVING";
        case SERVO_ARB_MSG_DONE:            return "DONE";
        case SERVO_ARB_MSG_BENCH_PRI_FIRE:  return "BENCH_PRI_FIRE";
        case SERVO_ARB_MSG_BENCH_SEC_FIRE:  return "BENCH_SEC_FIRE";
        case SERVO_ARB_MSG_BENCH_MAIN_HIGH: return "BENCH_MAIN_HIGH";
        case SERVO_ARB_MSG_MAIN_HIGH:       return "MAIN_HIGH";
        default:                            return "NONE";
    }
}
