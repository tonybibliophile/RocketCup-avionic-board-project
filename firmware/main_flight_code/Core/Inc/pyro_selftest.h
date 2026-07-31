/*
 * pyro_selftest.h — 開傘地面點火自測（PD13 引爆 + PD14 舵機 + LED/Buzzer 指示）
 * ===========================================================================
 * 用途：在工作台上驗證「開傘」完整電火動作。MCU 一上電/重啟就自動執行「一次」序列，
 *       跑完停在原地（持續餵看門狗、SYS LED 續閃），不進正常飛控 FSM，故「每次重啟＝一次測試」。
 *
 * ─── 測試序列（主/副兩板同一序列，各自獨立；★時序刻意 1:1 對應飛行邏輯）───
 *   開機：Buzzer 響兩聲（TIM2 CH1）。
 *   全程：SYS LED (PE2) 1Hz 持續閃爍 = 韌體存活。
 *   1. PD13 (FIRE / 引傘 DC 馬達) —— 對應飛行「主板提前 DROGUE_LEAD_TIME_S 開、副板真頂點才開」：
 *      1a 主板 t=0 拉高 FSM_DROGUE_MOTOR_RUN_PRIMARY_MS(8s)（State1 LED / PE3 亮）；
 *      1b 副板見主板通電後等 DROGUE_LEAD_TIME_S（模擬提前量；★依 FLIGHT_PROFILE_ELEVATOR
 *         分流：電梯場測 1s / 飛行 4s，見 fsm.h），期間雙板時序錯開；
 *      1c 副板 t=lead 起拉高 FSM_DROGUE_MOTOR_RUN_BACKUP_MS(3s)，與主板窗重疊——PD13 為
 *         diode-OR 準位訊號，兩板同時拉高無害，飛行時本來就會重疊。
 *   2. 等待 PYRO_SELFTEST_GAP_MS。
 *   3. PD14 主傘 —— 對應飛行「不啟 PWM、純 GPIO 拉高 1.5s、兩板同時共開」：
 *      雙板**同時**拉高 SERVO_MAIN_HIGH_MS（主板廣播 BENCH_MAIN_HIGH，副板見訊號即跟上），
 *      State2 LED (PE4) 同步亮，時間到拉回 LOW、State2 熄。★不再有互斥握手/讓位/PWM 掃描。
 *   （PD14 部署前為硬體低、無訊號，比照飛行；序列最前面另有退避倒數 PYRO_SELFTEST_COUNTDOWN_S。）
 *
 * ─── LED / Buzzer 腳位（GPIOE，active-high；如硬體為 active-low 改 PYRO_LED_ON/OFF）───
 *   SYS    = PE2 (LED_SYS)      : 持續閃爍
 *   State1 = PE3               : PD13 拉高時亮
 *   State2 = PE4 (LED_STAT2)   : PD14 拉高時亮
 *   Buzzer = TIM2 CH1          : 開機兩聲
 *
 * ─── 如何啟用 ───  board_config.h 把 FEATURE_PYRO_SELFTEST 設 1，重新燒錄。
 * ─── 如何刪除 ───
 *   1. 最快：FEATURE_PYRO_SELFTEST 設回 0 —— 本模組與 main.c 呼叫全部 #if 編譯掉。
 *   2. 徹底：刪本檔 + pyro_selftest.c，移除 main.c「PYRO SELF-TEST」段（含 #include），
 *      並從 Main_Code/Debug/objects.list 與 subdir.mk 移除 pyro_selftest.o / .c。
 *
 * ⚠ 安全：步驟 1 會實際導通引傘 DC 馬達（PD13）——主板 8s（t=0~8s）、副板 3s（t=4~7s，與主板
 *   重疊），若已接火藥/電熱絲會真的點火！上台前務必確認負載安全或以電表/假負載替代；
 *   序列開頭保留退避倒數供人員退避。
 */
#ifndef PYRO_SELFTEST_H
#define PYRO_SELFTEST_H

#include <stdint.h>
#include "board_config.h"

/* 本模組編入條件：開機自測（FEATURE_PYRO_SELFTEST）、遠端桌面測試（FEATURE_UPLINK_DEPLOY
 * 經 433 BENCH 命令觸發，見 uplink_cmd.c / main.c），或副板板間鏈路自動跟隨（FEATURE_LINK
 * && IS_BACKUP，見 main.c 的 IS_BACKUP 自動跟隨區塊）。任一開啟即需要序列函式與參數。
 * ⚠ FEATURE_UPLINK_DEPLOY 恆等於 IS_PRIMARY（board_config.h），若只靠它，副板會整個模組
 *   編譯不進去、PyroSelfTest_RunSequence_Ex 不存在，導致副板永遠無法自動跟隨 bench 測試
 *   （曾發生過的 bug）。故明確補上 (FEATURE_LINK && IS_BACKUP) 這條件。 */
#define PYRO_SELFTEST_AVAILABLE  (FEATURE_PYRO_SELFTEST || FEATURE_UPLINK_DEPLOY || \
                                   (FEATURE_LINK && IS_BACKUP))

#if PYRO_SELFTEST_AVAILABLE

/* === 可調參數 ===
 * ★步驟 1/3 的時間一律直接沿用飛行常數，桌面測試與飛行 1:1 對應、不另立一套數字：
 *   步驟 1 PD13：FSM_DROGUE_MOTOR_RUN_PRIMARY_MS(8s) / _BACKUP_MS(3s) + DROGUE_LEAD_TIME_S
 *               （1s 電梯 / 4s 飛行，隨 profile）錯開。
 *   步驟 3 PD14：SERVO_MAIN_HIGH_MS(1.5s)，雙板同時。
 * 下方 PYRO_SELFTEST_FIRE_MS / PYRO_SELFTEST_SERVO_HIGH_MS 已不再被 pyro_selftest.c 使用（保留相容）。 */
#ifndef PYRO_SELFTEST_FIRE_MS
#define PYRO_SELFTEST_FIRE_MS       8000U   /* 已由 FSM_DROGUE_MOTOR_RUN_*_MS 取代（步驟 1 不再引用） */
#endif
/* 步驟 2：引傘通電結束後、動主傘前的等待 */
#ifndef PYRO_SELFTEST_GAP_MS
#define PYRO_SELFTEST_GAP_MS        5000U   /* 等待 5s */
#endif

#ifndef PYRO_SELFTEST_SERVO_HIGH_MS
#define PYRO_SELFTEST_SERVO_HIGH_MS    4000U   /* 已由 SERVO_MAIN_HIGH_MS 取代（步驟 3 不再引用） */
#endif

/* 序列最前面的退避倒數（0 = 立即開始點火） */
#ifndef PYRO_SELFTEST_COUNTDOWN_S
#define PYRO_SELFTEST_COUNTDOWN_S      5U
#endif

/* SYS LED 閃爍半週期（500ms → 1Hz 閃） */
#ifndef PYRO_SELFTEST_SYS_BLINK_MS
#define PYRO_SELFTEST_SYS_BLINK_MS     500U
#endif

/* LED 亮/滅電位（active-high；硬體若為 active-low 對調此二值） */
#ifndef PYRO_LED_ON
#define PYRO_LED_ON   GPIO_PIN_SET
#endif
#ifndef PYRO_LED_OFF
#define PYRO_LED_OFF  GPIO_PIN_RESET
#endif

/* 開機 Buzzer：TIM2 時基 1MHz → 頻率 = 1e6/(ARR+1)。ARR=499 ≈ 2kHz。 */
#ifndef PYRO_BUZZER_ARR
#define PYRO_BUZZER_ARR   499U
#endif
#ifndef PYRO_BUZZER_BEEP_MS
#define PYRO_BUZZER_BEEP_MS  120U
#endif

#if FEATURE_LINK
/* bench 序列目前的階段（SERVO_ARB_MSG_*；NONE = 未在跑 bench）。
 * 由 pyro_selftest.c 於序列各階段更新，main.c 的 Link_BuildOwnStatus 讀取後填進
 * LinkStatus.main_arb —— bench 與飛控迴圈兩路廣播因此送出同一個值。 */
extern volatile uint8_t g_bench_arb;
#endif

/*
 * 執行「一次」開傘電火自測序列後「返回」（含退避倒數、Buzzer、LED、PD13/PD14 動作）。
 * 阻塞約 ~20s，過程自行餵 IWDG。呼叫端須確保周邊已初始化，且結束後負責把輸出恢復到
 * 部署前安全狀態（PD13/PD14 皆已於序列尾拉低；呼叫端仍再 Servo_HoldLow 復位一次——遠端
 * BENCH 走此函式，見 main.c 診斷任務）。開機自測（FEATURE_PYRO_SELFTEST）由 RunOnce 呼叫。
 */
void PyroSelfTest_RunSequence(void);
void PyroSelfTest_RunSequence_Ex(uint8_t skip_countdown);

#if FEATURE_PYRO_SELFTEST
/*
 * 開機自測：跑一次序列後「不返回」（停在無窮迴圈，續閃 SYS 並餵 IWDG），確保「每次重啟＝
 * 一次測試」不落入正常 FSM。應在周邊初始化與開機橫幅之後、進入正常飛控前呼叫。
 */
void PyroSelfTest_RunOnce(void);
#endif

#endif /* PYRO_SELFTEST_AVAILABLE */
#endif /* PYRO_SELFTEST_H */
