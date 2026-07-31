/*
 * pyro_selftest.c — 開傘地面點火自測（重啟後/BENCH 跑一次；引傘主副錯開、主傘雙板同時共開）
 * 詳見 pyro_selftest.h 的序列 / 腳位 / 啟用 / 刪除說明。整檔以 FEATURE_PYRO_SELFTEST 包住。
 */
#include "pyro_selftest.h"

#if PYRO_SELFTEST_AVAILABLE

#include <stdio.h>
#include "main.h"        /* FIRE(PD13) / PWM_Servo(PD14) / LED_SYS(PE2) / LED_STAT2(PE4) 腳位 */
#include "fsm.h"         /* FSM_DROGUE_MOTOR_RUN_PRIMARY/BACKUP_MS、DROGUE_LEAD_TIME_S：與飛行同源 */
#if FEATURE_LINK
#include "link_hw.h"     /* bench 協同：以板間鏈路對齊主/副時序（引傘錯開、主傘同時共開） */
#include "servo_arb.h"   /* SERVO_ARB_MSG_*（重用協同狀態值） */
#endif

/* 由 main.c 建立的周邊 handle。 */
extern TIM_HandleTypeDef  htim2;   /* Buzzer   : TIM2 CH1 */
extern IWDG_HandleTypeDef hiwdg;   /* 看門狗約 2.05s 逾時 */
extern FlightState_t current_fsm_state;   /* main.c：本板目前 FSM 狀態（bench 期間仍在跑） */

/* LED 腳位（皆 GPIOE）：State1=PE3 於 main.h 未命名，直接用腳號。 */
#define PYRO_LED_STATE1_Pin   GPIO_PIN_3

/* 依系統時鐘更新 SYS LED（1Hz 閃）。 */
static void sys_led_pump(void)
{
    uint8_t on = ((HAL_GetTick() / PYRO_SELFTEST_SYS_BLINK_MS) & 1U) != 0U;
    HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, on ? PYRO_LED_ON : PYRO_LED_OFF);
}

#if FEATURE_LINK
/* bench 協同：selftest 期間廣播本板 bench 階段狀態（重用 LinkPacket.main_arb）。RX(DMA/IDLE)
 * 與 TX(IT) 皆 ISR 驅動，故在阻塞序列中仍運作，兩板得以在各階段互通、對齊時序。 */
volatile uint8_t g_bench_arb = SERVO_ARB_MSG_NONE;
/* ★封包一律走 Link_BuildOwnStatus() 組裝（main.c），不再自己拼一份殘缺的。
 * 理由：bench 序列跑在 StartDiagnosticTask，飛控迴圈（StartDefaultTask）並沒有停，
 * 其 Link_PublishTick 同樣以 20Hz 送 LinkStatus。舊版這裡自拼封包、fsm_state 硬寫
 * STATE_PAD、其餘欄位清 0，於是對端每 50ms 交替收到兩套內容：
 *   - fsm_state 在 PAD / PAD_ARMED 間跳 → 副板 LINK_SYNC 判定「主板 DISARM」，
 *     bench 全程 20Hz 反覆 DISARM/ARM，且「自身 ARMED + 見 BENCH_START」這組跟隨
 *     條件分屬不同封包，變成競態（歷史症狀：副板整場不跟隨）。
 *   - main_arb 在 BENCH_* 與 NONE 間跳（2026-07-31 實測 log 已見）。
 *   - h_est/erase_pct 等欄位被清 0 的封包覆蓋。
 * 改為同源組裝後只有 main_arb 一欄由 bench 覆寫，而 Link_BuildOwnStatus 本身也會在
 * g_bench_arb != NONE 時送同一個值，兩路內容完全一致。 */
static void bench_bcast(void)
{
    LinkStatus_t ls;
    Link_BuildOwnStatus(&ls);
    ls.main_arb = g_bench_arb;
    Link_SendStatus(&ls);
}

/* bench 協同期間（g_bench_arb != NONE）以 ~5Hz 印一行 [LINK] 風格的 arb 狀態，讓
 * gui_monitor 用既有 [LINK] 解析器顯示 bench 進度（阻塞序列中飛控迴圈的 [LINK] 不會跑）。 */
static void bench_arb_report(void)
{
    if (g_bench_arb == SERVO_ARB_MSG_NONE &&
        HAL_GPIO_ReadPin(FIRE_GPIO_Port, FIRE_Pin) == GPIO_PIN_RESET) return;
    static uint32_t s_last_ms = 0U;
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ms) < 200U) return;   /* 提升遙測印出頻率至 ~5Hz，確保 GUI 即時抓取狀態 */
    s_last_ms = now;
    const LinkPeer_t *peer = Link_GetPeer();
    const char *link_state = !peer->valid ? "NONE" : (Link_PeerFresh(now) ? "OK" : "STALE");
    /* 格式與 main.c 診斷任務的 [LINK] 行一致：state/flags 都是「對端」的值。
     * ★不再硬寫 state=PAD——bench 期間兩板實際處於 PAD_ARMED，硬寫會讓 GUI 的
     *   STATE 欄在 bench 全程顯示成未武裝。 */
    printf("[LINK] self=%s peer=%s link=%s state=%s flags=0x%02X age=%lums "
           "sync=OK lost=0 desync=0 self_arb=%s peer_arb=%s\r\n",
           IS_BACKUP ? "BACKUP" : "PRIMARY",
           !peer->valid ? "NONE" : ((peer->board_id == LINK_BOARD_BACKUP) ? "BACKUP" : "PRIMARY"),
           link_state,
           link_fsm_state_name(peer->valid ? peer->fsm_state : (uint8_t)current_fsm_state),
           peer->flags,
           (unsigned long)(peer->valid ? (now - peer->last_rx_ms) : 0U),
           ServoArb_MsgName(g_bench_arb),
           ServoArb_MsgName(peer->peer_main_arb));
    fflush(stdout);
}
#endif

/* 分段延遲：續閃 SYS + 餵看門狗（IWDG≈2.05s，以 50ms 粒度）；FEATURE_LINK 下順帶
 * 以 ~20Hz 廣播 bench 階段狀態。 */
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
/* 等待對端廣播指定 arb 訊號（副板用來與主板對齊時序）。逾時即自行往下走，
 * 避免單板 bench / 鏈路異常時卡死。回傳 1 = 真的等到訊號，0 = 逾時。 */
static uint8_t bench_wait_peer_arb(uint8_t want_arb, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        const LinkPeer_t *peer = Link_GetPeer();
        if (peer->valid && peer->peer_main_arb == want_arb) return 1U;
        delay_fed(20U);
    }
    return 0U;
}
#endif

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
    printf("[PYRO-SELFTEST] 開傘電火自測（每次重啟/BENCH跑一次；時序 1:1 對應飛行邏輯）\r\n");
    printf("[PYRO-SELFTEST] 序列：引傘 PD13 主板 %us（t=0起）/ 副板延後 %us 後 %us → 等 %ums → 主傘 PD14 雙板同時拉高 %ums\r\n",
           (unsigned)(FSM_DROGUE_MOTOR_RUN_PRIMARY_MS / 1000U),
           (unsigned)DROGUE_LEAD_TIME_S,
           (unsigned)(FSM_DROGUE_MOTOR_RUN_BACKUP_MS / 1000U),
           (unsigned)PYRO_SELFTEST_GAP_MS,
           (unsigned)SERVO_MAIN_HIGH_MS);
#if FEATURE_LINK
    printf("[PYRO-SELFTEST] 引傘對應飛行：主板提前 %us 開、拉高 %us；副板真頂點才開、拉高 %us（兩窗重疊，PD13 準位訊號無妨）\r\n",
           (unsigned)DROGUE_LEAD_TIME_S,
           (unsigned)(FSM_DROGUE_MOTOR_RUN_PRIMARY_MS / 1000U),
           (unsigned)(FSM_DROGUE_MOTOR_RUN_BACKUP_MS / 1000U));
    printf("[PYRO-SELFTEST] 主傘對應飛行：★不啟 PWM，兩板同時純 GPIO 拉高 %ums（互斥握手已取消，一板開即呼叫對端一起開）\r\n",
           (unsigned)SERVO_MAIN_HIGH_MS);
#endif
    printf("[PYRO-SELFTEST] LED: SYS(PE2)閃 / State1(PE3)=PD13高 / State2(PE4)=PD14高\r\n");
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
    /* 宣告「BENCH 序列開始」：副板據此自動跟隨進入自測（見 main.c 的 IS_BACKUP 跟隨區塊）。 */
    g_bench_arb = SERVO_ARB_MSG_BENCH_START;
#endif

    if (!skip_countdown) {
        /* 退避倒數（讓人員遠離點火頭/開傘機構；期間持續廣播 BENCH_START 通知副板準備）。 */
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

    /* === 步驟 1：PD13 (FIRE / 引傘 DC 馬達) —— 1:1 重現飛行的主/副開傘時序 ===
     * 1a. 主板 t=0 起拉高 FSM_DROGUE_MOTOR_RUN_PRIMARY_MS(8s)，廣播 BENCH_PRI_FIRE
     *     —— 對應飛行「主板提前 DROGUE_LEAD_TIME_S 開引傘、拉高 8s」。
     * 1b. 副板見 BENCH_PRI_FIRE 後等 DROGUE_LEAD_TIME_S —— 模擬提前量，之後才輪到副板
     *     （＝飛行的「真頂點」時刻）。★lead 隨 profile：電梯場測 1s / 飛行 4s（fsm.h）。
     * 1c. 副板 t=lead 起拉高 FSM_DROGUE_MOTOR_RUN_BACKUP_MS(3s)，廣播 BENCH_SEC_FIRE。
     *     ★兩板窗會重疊（主 0~8s、副 lead~lead+3s），這正是飛行的真實情形：PD13 是
     *     diode-OR 的準位訊號，同時拉高無害，不需要（也不該）刻意錯開成互斥。 */
    const uint32_t drogue_lead_ms = (uint32_t)(DROGUE_LEAD_TIME_S * 1000.0f);

    if (IS_PRIMARY) {
        /* --- 主板：t=0 立刻拉高 8s --- */
#if FEATURE_LINK
        g_bench_arb = SERVO_ARB_MSG_BENCH_PRI_FIRE;
#endif
        printf("[PYRO-SELFTEST] [1a] 主板：PD13(FIRE)=HIGH %us（提前開引傘，對應飛行提前 %us）+ State1 亮\r\n",
               (unsigned)(FSM_DROGUE_MOTOR_RUN_PRIMARY_MS / 1000U), (unsigned)DROGUE_LEAD_TIME_S);
        fflush(stdout);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_ON);
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);

        /* 主板前 lead 段：對應「尚未到真頂點」，副板仍應為 LOW。 */
        delay_fed(drogue_lead_ms);

#if FEATURE_LINK
        /* 到達模擬頂點：通知副板該開了（副板據此起算自己的 3s）。 */
        g_bench_arb = SERVO_ARB_MSG_BENCH_SEC_FIRE;
        printf("[PYRO-SELFTEST] [1c] 主板：已達模擬頂點，廣播通知副板開引傘（主板 PD13 仍 HIGH，兩窗重疊）\r\n");
        fflush(stdout);
#endif
        /* 主板剩餘導通時間（8s − lead）。 */
        delay_fed(FSM_DROGUE_MOTOR_RUN_PRIMARY_MS - drogue_lead_ms);
    } else {
        /* --- 副板：等主板宣告開始 → 等滿 lead → 才拉高 3s --- */
        printf("[PYRO-SELFTEST] [1a] 副板：主板提前開引傘中，副板 PD13 保持 LOW（等真頂點）\r\n");
        fflush(stdout);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);

#if FEATURE_LINK
        printf("[PYRO-SELFTEST] [1b] 副板：等待主板通知到達模擬頂點（最長 %us；未見主板則自行等 %us）\r\n",
               (unsigned)((drogue_lead_ms + 2000U) / 1000U), (unsigned)DROGUE_LEAD_TIME_S);
        fflush(stdout);
        if (!bench_wait_peer_arb(SERVO_ARB_MSG_BENCH_SEC_FIRE, drogue_lead_ms + 2000U)) {
            printf("[PYRO-SELFTEST] [1b] 副板：未見主板通知（單板 bench / 鏈路異常）→ 自行往下走\r\n");
            fflush(stdout);
        }
        g_bench_arb = SERVO_ARB_MSG_BENCH_SEC_FIRE;
#else
        printf("[PYRO-SELFTEST] [1b] 副板：等待 %us（模擬主板提前量）\r\n", (unsigned)DROGUE_LEAD_TIME_S);
        fflush(stdout);
        delay_fed(drogue_lead_ms);
#endif
        printf("[PYRO-SELFTEST] [1c] 副板：PD13(FIRE)=HIGH %us（真頂點才開）+ State1 亮\r\n",
               (unsigned)(FSM_DROGUE_MOTOR_RUN_BACKUP_MS / 1000U));
        fflush(stdout);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_ON);
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_SET);
        delay_fed(FSM_DROGUE_MOTOR_RUN_BACKUP_MS);

        /* 副板 3s 窗先結束（主板仍在導通），先拉低自己這一路。 */
        HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
        printf("[PYRO-SELFTEST] [1c] 副板：%us 到，PD13 回 LOW（主板可能仍在導通，等其結束）\r\n",
               (unsigned)(FSM_DROGUE_MOTOR_RUN_BACKUP_MS / 1000U));
        fflush(stdout);
#if FEATURE_LINK
        g_bench_arb = SERVO_ARB_MSG_NONE;
        /* 對齊主板 8s 窗尾，兩板才一起進入步驟 2（單板/逾時則自行往下）。 */
        {
            uint32_t remain = FSM_DROGUE_MOTOR_RUN_PRIMARY_MS -
                              (drogue_lead_ms + FSM_DROGUE_MOTOR_RUN_BACKUP_MS);
            delay_fed(remain);
        }
#endif
    }

    /* 結束步驟 1：復位 PD13 腳位與 State1 LED（兩板皆確實拉低一次）。 */
    HAL_GPIO_WritePin(FIRE_GPIO_Port, FIRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, PYRO_LED_STATE1_Pin, PYRO_LED_OFF);
#if FEATURE_LINK
    g_bench_arb = SERVO_ARB_MSG_NONE;
#endif
    printf("[PYRO-SELFTEST] [1] %s：PD13 通電測試結束，復位為 LOW + State1 熄\r\n", IS_PRIMARY ? "主板" : "副板");
    fflush(stdout);

    /* === 步驟 2：冷卻等待 === */
    printf("[PYRO-SELFTEST] [2] 等待 %ums\r\n", (unsigned)PYRO_SELFTEST_GAP_MS);
    fflush(stdout);
    delay_fed(PYRO_SELFTEST_GAP_MS);

    /* === 步驟 3：主傘 PD14 —— 雙板「同時」純 GPIO 拉高 SERVO_MAIN_HIGH_MS(1.5s) ===
     * ★互斥握手（INTENT/guard/讓位/DONE 交接）與 PWM 舵機掃描已整段取消：主傘改吃準位
     * 訊號，兩板 diode-OR 同時拉高無害，飛行時本來就是「一板判到即呼叫對端一起開」。
     * 這裡以主板廣播 BENCH_MAIN_HIGH 當作「呼叫」，副板見訊號立刻跟上（單板/逾時自行開）。 */
#if FEATURE_LINK
    if (IS_PRIMARY) {
        g_bench_arb = SERVO_ARB_MSG_BENCH_MAIN_HIGH;   /* 呼叫副板一起開 */
        printf("[PYRO-SELFTEST] [3] 主板：廣播 BENCH_MAIN_HIGH 呼叫副板一起開主傘（無握手隔離）\r\n");
        fflush(stdout);
    } else {
        printf("[PYRO-SELFTEST] [3] 副板：等待主板呼叫一起開主傘（最長 3s；未見則自行開）\r\n");
        fflush(stdout);
        if (!bench_wait_peer_arb(SERVO_ARB_MSG_BENCH_MAIN_HIGH, 3000U)) {
            printf("[PYRO-SELFTEST] [3] 副板：未見主板呼叫（單板 bench / 鏈路異常）→ 自行開\r\n");
            fflush(stdout);
        }
        g_bench_arb = SERVO_ARB_MSG_BENCH_MAIN_HIGH;
    }
#endif
    printf("[PYRO-SELFTEST] [3] %s：PD14 拉高 %ums（純 GPIO，不啟 PWM；雙板同時共開）...\r\n",
           IS_PRIMARY ? "主板" : "副板", (unsigned)SERVO_MAIN_HIGH_MS);
    fflush(stdout);
    {
        GPIO_InitTypeDef gi = {0};
        gi.Pin   = PWM_Servo_Pin;
        gi.Mode  = GPIO_MODE_OUTPUT_PP;
        gi.Pull  = GPIO_NOPULL;
        gi.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(PWM_Servo_GPIO_Port, &gi);
        HAL_GPIO_WritePin(PWM_Servo_GPIO_Port, PWM_Servo_Pin, GPIO_PIN_SET);   /* 純 GPIO 拉高，不啟 PWM */
    }
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_ON);
    delay_fed(SERVO_MAIN_HIGH_MS);
    printf("[PYRO-SELFTEST] [3] %s：拉高 %ums 完成 → 拉回 LOW\r\n",
           IS_PRIMARY ? "主板" : "副板", (unsigned)SERVO_MAIN_HIGH_MS);
    fflush(stdout);
    HAL_GPIO_WritePin(PWM_Servo_GPIO_Port, PWM_Servo_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_STAT2_GPIO_Port, LED_STAT2_Pin, PYRO_LED_OFF);
#if FEATURE_LINK
    /* 拉高窗結束，廣播 DONE；主板再花一小段時間確認副板也回報 DONE（純觀測，逾時即往下）。 */
    g_bench_arb = SERVO_ARB_MSG_DONE;
    if (IS_PRIMARY) {
        printf("[PYRO-SELFTEST] [3] 主板：主傘拉高完成，等待副板回報 DONE（確認雙板共開）...\r\n");
        fflush(stdout);
        if (bench_wait_peer_arb(SERVO_ARB_MSG_DONE, 4000U)) {
            printf("[PYRO-SELFTEST] [3] 主板：✅ 已確認副板同步完成主傘拉高（雙板共開成功）\r\n");
        } else {
            printf("[PYRO-SELFTEST] [3] 主板：⚠ 未收到副板 DONE（單板 bench 或鏈路異常）\r\n");
        }
        fflush(stdout);
    } else {
        printf("[PYRO-SELFTEST] [3] 副板：主傘拉高完成，廣播 DONE...\r\n");
        fflush(stdout);
        for (int i = 0; i < 20; i++) delay_fed(50U);   /* 續播 ~1s 讓主板收得到 */
    }
    g_bench_arb = SERVO_ARB_MSG_NONE;
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
