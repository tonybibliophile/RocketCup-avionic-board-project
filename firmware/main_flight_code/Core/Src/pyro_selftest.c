/*
 * pyro_selftest.c — 開傘地面點火自測（重啟後跑一次；主傘舵機以板間鏈路時間錯開：主先→副後）
 * 詳見 pyro_selftest.h 的序列 / 腳位 / 啟用 / 刪除說明。整檔以 FEATURE_PYRO_SELFTEST 包住。
 */
#include "pyro_selftest.h"

#if PYRO_SELFTEST_AVAILABLE

#include <stdio.h>
#include <string.h>
#include "main.h"        /* FIRE(PD13) / PWM_Servo(PD14) / LED_SYS(PE2) / LED_STAT2(PE4) 腳位 */
#include "fsm.h"         /* FSM_DROGUE_MOTOR_RUN_MS：副傘 DC 馬達導通時間（與飛行一致，8s） */
#if FEATURE_LINK
#include "link_hw.h"     /* D2-bench：舵機掃描以板間鏈路錯開（主先→副後，diode-OR 不打架） */
#include "servo_arb.h"   /* SERVO_ARB_MSG_*（重用握手狀態值） */
#endif

/* 由 main.c 建立的周邊 handle。 */
extern TIM_HandleTypeDef  htim2;   /* Buzzer   : TIM2 CH1 */
extern TIM_HandleTypeDef  htim4;   /* 開傘舵機 : TIM4 CH3（PD14） */
extern IWDG_HandleTypeDef hiwdg;   /* 看門狗約 2.05s 逾時 */

/* LED 腳位（皆 GPIOE）：State1=PE3 於 main.h 未命名，直接用腳號。 */
#define PYRO_LED_STATE1_Pin   GPIO_PIN_3

/* 依系統時鐘更新 SYS LED（1Hz 閃）。 */
static void sys_led_pump(void)
{
    uint8_t on = ((HAL_GetTick() / PYRO_SELFTEST_SYS_BLINK_MS) & 1U) != 0U;
    HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, on ? PYRO_LED_ON : PYRO_LED_OFF);
}

#if FEATURE_LINK
/* D2-bench：selftest 期間廣播本板舵機握手狀態（重用 LinkPacket.main_arb）。RX(DMA/IDLE)
 * 與 TX(IT) 皆 ISR 驅動，故在阻塞序列中仍運作，兩板得以在舵機掃描前後互通、時間錯開。 */
static uint8_t s_bench_arb = SERVO_ARB_MSG_NONE;
static void bench_bcast(void)
{
    LinkStatus_t ls;
    memset(&ls, 0, sizeof(ls));
    ls.board_id  = IS_BACKUP ? LINK_BOARD_BACKUP : LINK_BOARD_PRIMARY;
    ls.fsm_state = (uint8_t)STATE_PAD;
    ls.flags     = (HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_SET) ? TELEM_FLAG_DROGUE_FIRED : 0U;
    ls.main_arb  = s_bench_arb;
    Link_SendStatus(&ls);
}

/* 舵機協調期間（s_bench_arb != NONE）以 ~2Hz 印一行 [LINK] 風格的 arb 狀態，讓
 * gui_monitor 用既有 [LINK] 解析器顯示 bench 握手（阻塞序列中飛控迴圈的 [LINK] 不會跑）。 */
static void bench_arb_report(void)
{
    if (s_bench_arb == SERVO_ARB_MSG_NONE &&
        HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_RESET) return;
    static uint32_t s_last_ms = 0U;
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ms) < 200U) return;   /* 提升遙測印出頻率至 ~5Hz，確保 GUI 即時抓取狀態 */
    s_last_ms = now;
    const LinkPeer_t *peer = Link_GetPeer();
    const char *link_state = !peer->valid ? "NONE" : (Link_PeerFresh(now) ? "OK" : "STALE");
    printf("[LINK] self=%s peer=%s link=%s state=PAD flags=0x%02X age=%lums "
           "sync=OK lost=0 desync=0 self_arb=%s peer_arb=%s\r\n",
           IS_BACKUP ? "BACKUP" : "PRIMARY",
           !peer->valid ? "NONE" : ((peer->board_id == LINK_BOARD_BACKUP) ? "BACKUP" : "PRIMARY"),
           link_state,
           peer->flags,
           (unsigned long)(peer->valid ? (now - peer->last_rx_ms) : 0U),
           ServoArb_MsgName(s_bench_arb),
           ServoArb_MsgName(peer->peer_main_arb));
    fflush(stdout);
}
#endif

/* 分段延遲：續閃 SYS + 餵看門狗（IWDG≈2.05s，以 50ms 粒度）；FEATURE_LINK 下順帶
 * 以 ~20Hz 廣播 bench 舵機握手狀態。 */
static void delay_fed(uint32_t ms)
{
    while (ms > 0U) {
        uint32_t chunk = (ms > 50U) ? 50U : ms;
        HAL_Delay(chunk);
        sys_led_pump();
        HAL_IWDG_Refresh(&hiwdg);
#if FEATURE_LINK
        bench_bcast();
        bench_arb_report();
#endif
        ms -= chunk;
    }
}

#if FEATURE_LINK
/* 主傘舵機 diode-OR PWM 不可兩板同時驅動。主板優先先掃；副板等主板掃完（見對端
 * main_arb==DONE）才輪到；若完全未見主板封包（單板 bench）則 guard 逾時後自行掃。 */
static void bench_wait_servo_turn(void)
{
    if (IS_PRIMARY) return;                    /* 主板優先，直接掃 */

    const uint32_t BACKUP_ALONE_GUARD_MS = 8000U;
    uint32_t t0 = HAL_GetTick();
    s_bench_arb = SERVO_ARB_MSG_INTENT;        /* 讓主板/觀察者得知副板在等（非必要，純觀測） */
    printf("[PYRO-SELFTEST] [3a] 副板：等待主板舵機掃完（或未見主板則 %lus 後自行掃）\r\n",
           (unsigned long)(BACKUP_ALONE_GUARD_MS / 1000U));
    fflush(stdout);
    for (;;) {
        uint32_t now = HAL_GetTick();
        uint8_t peer_fresh = Link_PeerFresh(now);
        uint8_t peer_arb   = peer_fresh ? Link_GetPeer()->peer_main_arb : SERVO_ARB_MSG_NONE;
        if (peer_arb == SERVO_ARB_MSG_DONE) {
            printf("[PYRO-SELFTEST] [3a] 副板：見主板 DONE → 接手掃\r\n"); fflush(stdout);
            break;
        }
        if (!peer_fresh && (now - t0) >= BACKUP_ALONE_GUARD_MS) {
            printf("[PYRO-SELFTEST] [3a] 副板：未見主板封包 → 單板 bench 自行掃\r\n"); fflush(stdout);
            break;
        }
        delay_fed(50U);                        /* 期間持續廣播 INTENT + 讀對端 */
    }
}
#endif

static void servo_set_us(uint32_t us)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, us);
}

/* 開機 Buzzer：ARR 決定音高，CCR1=ARR/2 出聲、=0 靜音。 */
static void buzzer_beep(uint32_t ms)
{
    htim2.Instance->ARR  = PYRO_BUZZER_ARR;
    htim2.Instance->CCR1 = PYRO_BUZZER_ARR / 2U;
    htim2.Instance->EGR  = TIM_EGR_UG;
    delay_fed(ms);
    htim2.Instance->CCR1 = 0U;
}

void PyroSelfTest_RunSequence_Ex(uint8_t skip_countdown)
{
    printf("\r\n============================================================\r\n");
    printf("[PYRO-SELFTEST] 開傘電火自測（每次重啟/BENCH跑一次；主副協同 3 階段通電 + 舵機交接）\r\n");
    printf("[PYRO-SELFTEST] 序列：PD13 通電 8s (主2s→副2s→雙板4s) → 等 %ums → 舵機主板掃完呼叫副板\r\n",
           (unsigned)PYRO_SELFTEST_GAP_MS);
#if FEATURE_LINK
    printf("[PYRO-SELFTEST] 主/副協同：主板 2s 單觸發 → 副板 2s 單觸發 → 雙板 4s 同時觸發（驗證 Diode-OR）\r\n");
    printf("[PYRO-SELFTEST] 主傘舵機時間錯開：主板先掃完廣播 DONE 呼叫副板，副板接收後接手掃\r\n");
#endif
    printf("[PYRO-SELFTEST] LED: SYS(PE2)閃 / State1(PE3)=PD13高 / State2(PE4)=舵機PWM\r\n");
    printf("[PYRO-SELFTEST] ⚠ PD13 會實際導通引爆 MOSFET，確認負載安全再繼續！\r\n");
    fflush(stdout);

    /* 起始安全狀態：FIRE 拉低、State1/State2 熄、SYS 起閃。
     * PD14 已於 main.c Servo_HoldLow()（開機）拉為 GPIO 低，此處不動，維持部署前無訊號。 */
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_OFF);
    sys_led_pump();

    /* 開機 Buzzer 兩聲。 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    buzzer_beep(PYRO_BUZZER_BEEP_MS);
    delay_fed(100U);
    buzzer_beep(PYRO_BUZZER_BEEP_MS);

#if FEATURE_LINK
    /* 發送 INTENT 宣告兩板準備進行 BENCH 測試 */
    s_bench_arb = SERVO_ARB_MSG_INTENT;
#endif

    if (!skip_countdown) {
        /* 退避倒數（讓人員遠離點火頭/舵機連桿；期間持續廣播 INTENT 通知副板準備）。 */
        for (uint32_t s = PYRO_SELFTEST_COUNTDOWN_S; s > 0U; s--) {
            printf("[PYRO-SELFTEST] 點火倒數 %lu ...\r\n", (unsigned long)s);
            fflush(stdout);
            delay_fed(1000U);
        }
    } else {
        /* 副板由板間鏈路帶動：跳過倒數，等待主板開始 Stage 1a (BENCH_PRI_FIRE) 以達成精確同脈 */
        printf("[PYRO-SELFTEST] 副板由板間鏈路帶動，跳過倒數，與主板時間同脈同步中...\r\n");
        fflush(stdout);
        uint32_t wait_start = HAL_GetTick();
        while (HAL_GetTick() - wait_start < 6000U) {
            const LinkPeer_t *peer = Link_GetPeer();
            if (peer->valid && peer->peer_main_arb == SERVO_ARB_MSG_BENCH_PRI_FIRE) {
                break;  /* 精確對齊主板 1a 起始瞬間 */
            }
            delay_fed(20U);
        }
    }

    /* === 步驟 1：PD13 (FIRE / 副傘 DC 馬達) 3 階段通電 (總共 8s) ===
     * 1a. 主板單獨通電 2s (s_bench_arb = BENCH_PRI_FIRE)
     * 1b. 副板單獨通電 2s (s_bench_arb = BENCH_SEC_FIRE)
     * 1c. 雙板同時通電 4s (s_bench_arb = BENCH_BOTH_FIRE) */

    /* --- 階段 1a：主板單獨通電 2 秒 --- */
#if FEATURE_LINK
    s_bench_arb = SERVO_ARB_MSG_BENCH_PRI_FIRE;
#endif
    if (IS_PRIMARY) {
        printf("[PYRO-SELFTEST] [1a] 主板：PD13(FIRE)=HIGH (主板單獨通電 2s) + State1 亮\r\n");
        fflush(stdout);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_ON);
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
    } else {
        printf("[PYRO-SELFTEST] [1a] 副板：主板單獨通電中 (副板 PD13 保持 LOW)\r\n");
        fflush(stdout);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    }
    delay_fed(2000U);

    /* --- 階段 1b：副板單獨通電 2 秒 --- */
#if FEATURE_LINK
    s_bench_arb = SERVO_ARB_MSG_BENCH_SEC_FIRE;
#endif
    if (IS_BACKUP) {
        printf("[PYRO-SELFTEST] [1b] 副板：PD13(FIRE)=HIGH (副板單獨通電 2s) + State1 亮\r\n");
        fflush(stdout);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_ON);
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
    } else {
        printf("[PYRO-SELFTEST] [1b] 主板：副板單獨通電中 (主板 PD13 保持 LOW)\r\n");
        fflush(stdout);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    }
    delay_fed(2000U);

    /* --- 階段 1c：雙板同時通電 4 秒 (驗證 Diode-OR 雙點火) --- */
#if FEATURE_LINK
    s_bench_arb = SERVO_ARB_MSG_BENCH_BOTH_FIRE;
#endif
    printf("[PYRO-SELFTEST] [1c] %s：雙板同時通電中 (PD13=HIGH 4s) + State1 亮\r\n", IS_PRIMARY ? "主板" : "副板");
    fflush(stdout);
    HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_ON);
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
    delay_fed(4000U);

    /* 結束 步驟 1 通電，復位 PD13 腳位與 State1 LED */
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
#if FEATURE_LINK
    s_bench_arb = SERVO_ARB_MSG_NONE;
#endif
    printf("[PYRO-SELFTEST] [1] %s：PD13 通電測試結束，復位為 LOW + State1 熄\r\n", IS_PRIMARY ? "主板" : "副板");
    fflush(stdout);

    /* === 步驟 2：冷卻等待 === */
    printf("[PYRO-SELFTEST] [2] 等待 %ums\r\n", (unsigned)PYRO_SELFTEST_GAP_MS);
    fflush(stdout);
    delay_fed(PYRO_SELFTEST_GAP_MS);

    /* === 步驟 3：PD14 由 GPIO 低切回 TIM4 AF，舵機 0°→180°→(停)→0°，PWM 期間 State2 LED 亮 ===
     * D2-bench：主傘舵機與對端 diode-OR，兩板不可同時掃。
     * 主板先掃完並廣播 DONE 呼叫副板；副板接收到主板 DONE 訊號後接手啟動舵機掃描。 */
#if FEATURE_LINK
    if (IS_PRIMARY) {
        /* 主板：發送 1s Guard 意圖確認 (廣播 INTENT) */
        s_bench_arb = SERVO_ARB_MSG_INTENT;
        printf("[PYRO-SELFTEST] [3a] 主板：發送 1s Guard 意圖確認 (廣播 INTENT)...\r\n");
        fflush(stdout);
        delay_fed(1000U);

        printf("[PYRO-SELFTEST] [3a] 主板：1s Guard 完成 → 取得獨佔權限 (DRIVING)\r\n");
        fflush(stdout);
        s_bench_arb = SERVO_ARB_MSG_DRIVING;
    } else {
        /* 副板：等待主板掃完廣播 DONE (或單板逾時) */
        bench_wait_servo_turn();

        /* 副板接手：發送 1s Guard 意圖確認 (廣播 INTENT 並等待 1s) */
        s_bench_arb = SERVO_ARB_MSG_INTENT;
        printf("[PYRO-SELFTEST] [3b] 副板：見主板 DONE，發送 1s Guard 意圖確認 (廣播 INTENT)...\r\n");
        fflush(stdout);
        delay_fed(1000U);

        printf("[PYRO-SELFTEST] [3b] 副板：1s Guard 完成 → 取得獨佔權限 (DRIVING)\r\n");
        fflush(stdout);
        s_bench_arb = SERVO_ARB_MSG_DRIVING;
    }
#endif
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin       = PWM_Servo_Pin;
        gi.Mode      = GPIO_MODE_AF_PP;
        gi.Pull      = GPIO_NOPULL;
        gi.Speed     = GPIO_SPEED_FREQ_LOW;
        gi.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(PWM_Servo_GPIO_Port, &gi);
    }
    printf("[PYRO-SELFTEST] [3] %s：舵機 PWM 啟動 + State2 亮：0°(%uus)→180°(%uus)\r\n",
           IS_PRIMARY ? "主板" : "副板", (unsigned)PYRO_SELFTEST_SERVO_0DEG_US, (unsigned)PYRO_SELFTEST_SERVO_180DEG_US);
    fflush(stdout);
    servo_set_us(PYRO_SELFTEST_SERVO_0DEG_US);         /* 先定 0° 再啟 PWM，避免首幀跳到殘值 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_ON);
    delay_fed(500U);                                   /* 歸 0° 定位 */
    servo_set_us(PYRO_SELFTEST_SERVO_180DEG_US);       /* 轉 180° */
    delay_fed(PYRO_SELFTEST_SERVO_HOLD_MS);
    printf("[PYRO-SELFTEST] [3] %s：舵機 180°→0°(%uus)\r\n",
           IS_PRIMARY ? "主板" : "副板", (unsigned)PYRO_SELFTEST_SERVO_0DEG_US);
    fflush(stdout);
    servo_set_us(PYRO_SELFTEST_SERVO_0DEG_US);
    delay_fed(1000U);                                  /* 給舵機回程時間 */
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);           /* 停 PWM */
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_OFF);
#if FEATURE_LINK
    /* 掃完廣播 DONE：主板廣播 DONE 呼叫副板；主板原地等待副板等待(INTENT) -> 接手(DRIVING) -> 完成(DONE)。 */
    s_bench_arb = SERVO_ARB_MSG_DONE;
    if (IS_PRIMARY) {
        printf("[PYRO-SELFTEST] [3a] 主板：舵機掃描完成，廣播 DONE 呼叫副板接手...\r\n");
        fflush(stdout);
        uint32_t wait_start = HAL_GetTick();
        uint8_t sec_intent_reported = 0;
        uint8_t sec_driving_reported = 0;
        while (HAL_GetTick() - wait_start < 12000U) {
            const LinkPeer_t *peer = Link_GetPeer();
            if (peer->valid && Link_PeerFresh(HAL_GetTick())) {
                if (peer->peer_main_arb == SERVO_ARB_MSG_INTENT) {
                    if (!sec_intent_reported) {
                        sec_intent_reported = 1;
                        printf("[PYRO-SELFTEST] [3b] 主板：對端副板廣播 INTENT 意圖確認中 (等待 1s Guard 通過)...\r\n");
                        fflush(stdout);
                    }
                } else if (peer->peer_main_arb == SERVO_ARB_MSG_DRIVING) {
                    if (!sec_driving_reported) {
                        sec_driving_reported = 1;
                        printf("[PYRO-SELFTEST] [3b] 主板：對端副板 1s Guard 通過，取得獨佔權限 (DRIVING 掃描中)...\r\n");
                        fflush(stdout);
                    }
                } else if (peer->peer_main_arb == SERVO_ARB_MSG_DONE && sec_driving_reported) {
                    printf("[PYRO-SELFTEST] [3b] 主板：確認副板舵機掃描完成 (見對端 DONE)！\r\n");
                    fflush(stdout);
                    break;
                }
            }
            delay_fed(50U);
        }
    } else {
        printf("[PYRO-SELFTEST] [3b] 副板：舵機掃描完成，廣播 DONE...\r\n");
        fflush(stdout);
        for (int i = 0; i < 40; i++) delay_fed(50U);
    }
    s_bench_arb = SERVO_ARB_MSG_NONE;
#endif

    printf("[PYRO-SELFTEST] 序列完成，返回呼叫端。\r\n");
    fflush(stdout);
}

void PyroSelfTest_RunSequence(void)
{
    PyroSelfTest_RunSequence_Ex(0);
}

#if FEATURE_PYRO_SELFTEST
void PyroSelfTest_RunOnce(void)
{
    PyroSelfTest_RunSequence();
    printf("[PYRO-SELFTEST] 停在此處（SYS 續閃，不進飛控）——電源重置可再測一次。\r\n");
    fflush(stdout);
    /* 停在原地：SYS 續閃 + 餵狗，確保「一次重啟＝一次測試」，不落入正常 FSM。 */
    for (;;) {
        sys_led_pump();
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(50);
    }
}
#endif /* FEATURE_PYRO_SELFTEST */

#endif /* PYRO_SELFTEST_AVAILABLE */
