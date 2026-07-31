/*
 * fsm.h — 飛行狀態機純邏輯模組（P0-A 自 main.c FSM_Update 抽離）
 * ===========================================================================
 * 設計原則（比照 sensor_axis.h / attitude_math.h 的 host 共測模式）：
 *   - 本檔與 fsm.c 不依賴任何 HAL / CMSIS / FreeRTOS，僅 <stdint.h>，
 *     可在 host 上以 tests/test_fsm.c 模擬整段飛行剖面逐 cycle 驗證。
 *   - FSM_Step() 為純函式：輸入快照 (FSM_Input_t) → 狀態轉移 → 動作 (FSM_Action_t)。
 *     GPIO / PWM / 蜂鳴器 / printf 一律由呼叫端（main.c 的 FSM_Update 包裝）執行，
 *     且硬體動作必須先於事件列印（點火不被 UART 阻塞延遲）。
 *   - 呼叫頻率契約：100 Hz（每 10ms 一次）。速度差分與「連續 N 週期」防雜訊
 *     計數皆以此為前提。
 */
#ifndef FSM_H
#define FSM_H

#include <stdint.h>
#include "board_config.h"   /* FLIGHT_PROFILE_ELEVATOR：門檻依 profile 分組（見下）；
                              * 純巨集、host 安全，比照 tests/test_link.c 的 include 模式 */

#ifdef __cplusplus
extern "C" {
#endif

/* === 飛行狀態（原 main.h USER CODE ET 區塊移入；數值不變） === */
typedef enum {
    STATE_INIT = 0,           // 系統初始化與自檢
    STATE_PAD = 1,            // 發射架上（等待校準完成）
    STATE_PAD_ARMED = 2,      // 發射架已武裝（等待起飛）
    STATE_BOOST = 3,          // 動力上升（馬達燃燒）
    STATE_COAST = 4,          // 慣性滑行（頂點預測與監控啟用）
    STATE_DEPLOY_DROGUE = 5,  // 預測式提前開傘：驅動副傘 DC 馬達（PD13 HIGH）持續
                              // FSM_DROGUE_MOTOR_RUN_MS，機構本身需時間展開，故不等
                              // 真正頂點才觸發，提前 DROGUE_LEAD_TIME_S 秒下令
    STATE_APOGEE = 6,         // 頂點狀態記錄（純遙測標記，馬達已停，不驅動任何硬體，
                              // 同一週期即轉入 DESCENT）
    STATE_DESCENT = 7,        // 副傘下降（監控主傘部署高度）
    STATE_MAIN_DEPLOY = 8,    // 主傘部署（動態高度觸發：PD14 純 GPIO 拉高
                              // SERVO_MAIN_HIGH_MS，不再驅動 PWM 舵機）
    STATE_LANDED = 9          // 安全著陸（尋標蜂鳴器與安全關檔）
} FlightState_t;

/* === FSM 參數（原 main.h 三常數 + FSM_Update 內魔術數字集中；值逐字不變） === */
/* TARGET_MAIN_ALTITUDE 依 profile 分組（見下方 Profile 隔離區塊）：電梯井道僅
 * ~35m，150m 飛行目標不合理，電梯改回落 10m 觸發主傘。 */
#define MAIN_DEPLOY_DELAY_S      3.5f    // 主傘機構部署延遲時間 (s)
/* DROGUE_LEAD_TIME_S（副傘頂點預估提前開傘時間）依 profile 分流，見下方 Profile 隔離區塊：
 * 飛行 4.0s（DC 馬達機構需時展開）、電梯 1.0s（井道僅 ~30m，4s lead 不合尺度）。
 * ⚠ 僅主航電（IS_PRIMARY）採用此提前預測路徑開引傘；副航電改為只在真正頂點才開
 * （fsm.c STATE_COAST 的路徑 2/3/4：速度過零 / 高度回落 / baro 趨勢交叉），兩板故意
 * 錯開時間，非同時點火。詳見 fsm.c apogee_condition 判定區塊註解。 */

#define FSM_LIFTOFF_ACCEL_G      3.0f    // 起飛觸發：高G垂直加速度門檻 (g)
#define FSM_LIFTOFF_ACCEL_CONSEC_N 20U   // a_z 路徑防手震：連續 20 週期(200ms)超過門檻才算數。
                                          // 真實點火持續遠超此窗（燒完最短時間鎖 1.5s）；
                                          // 手持晃動的瞬間尖峰通常 <100ms，藉此區分（實測
                                          // 手震曾產生 3.97g 瞬間值誤觸發，單一取樣點無法
                                          // 分辨「手震」與「馬達點火」，需靠持續時間）。
                                          // h_est/baro_alt_rel 兩條路徑本身即為已累積位移量，
                                          // 不需要額外防手震。
/* ⚠ FSM_LIFTOFF_ALT_M（估計器高度起飛門檻）仍兩 profile 共用 10.0f。
 * 舊註解聲稱「電梯 profile 已停用 FEATURE_FORCE_BARO_ONLY」，但 board_config.h 目前
 * 該巨集仍綁定 FLIGHT_PROFILE_ELEVATOR（見該檔），此變更從未真的完成/驗證——電梯
 * profile 目前仍強制 est_healthy=0，本門檻在電梯 profile 走不到。10m 對電梯起飛
 * 偵測是否合理待評估（同類問題已修 TARGET_MAIN_ALTITUDE，這裡尚未訂出電梯專用值，
 * 暫維持共用，待電梯實測後再決定是否分組）。 */
#define FSM_LIFTOFF_ALT_M        10.0f   // 起飛觸發：估計器（VF，或 EKF）高度門檻 (m)（恢復原飛行值）
#define FSM_APOGEE_MIN_FLIGHT_MS 3000U   // 頂點判定：起飛時間鎖 (ms)
#define FSM_APOGEE_CONSEC_N      5U      // 頂點判定：連續成立週期數（5×10ms=50ms 防雜訊）
#define FSM_APOGEE_VFALL_MPS     0.2f    // 頂點備用判定：速度過零門檻 (v_est < -0.2)
#define FSM_APOGEE_ALT_DROP_M    5.0f    // 頂點備用判定：自峰值下降高度 (m)
/* 副傘（引傘）DC 馬達導通時間 (ms)：PD13 為馬達驅動準位訊號（非點火 MOSFET 瞬間脈衝），
 * ★主/副航電不同值（使用者決策，對應兩板刻意錯開的開傘時序）：
 *   主航電：頂點前提前 DROGUE_LEAD_TIME_S(4s) 觸發，拉高 8s——提前量已吃掉一部分時間，
 *           且主航電是主要開傘路徑，給滿機構完整展開所需的導通時間。
 *   副航電：只在「真頂點」才觸發（見 fsm.c apogee_condition：副航電不走提前預測路徑），
 *           拉高 3s——此時主航電多半已把引傘拉出來，副板只是補一段冗餘推力，不需 8s。
 * 兩板 PD13 於開傘板以 diode-OR 合流，準位訊號同時拉高無害，故兩窗重疊不需互斥。
 * main.c 手動副傘上行指令沿用同一常數（各板依自身角色取值）。 */
#define FSM_DROGUE_MOTOR_RUN_PRIMARY_MS 8000U   // 主航電：提前 4s 觸發，拉高 8s
#define FSM_DROGUE_MOTOR_RUN_BACKUP_MS  3000U   // 副航電：真頂點觸發，拉高 3s
#define FSM_DROGUE_MOTOR_RUN_MS  (IS_PRIMARY ? FSM_DROGUE_MOTOR_RUN_PRIMARY_MS \
                                              : FSM_DROGUE_MOTOR_RUN_BACKUP_MS)
                                          // ⚠ 這是三元運算式（非整數字面值），不可用於 #if；
                                          //   IS_PRIMARY 為編譯期常數，最佳化後與常數等價。
#define FSM_MAIN_INFLATE_MS      3000U   // 主傘充氣張開等待時間 (ms)
#define FSM_TOUCHDOWN_V_MPS      0.3f    // 落地判定：|v_est| 門檻 (m/s)
#define FSM_TOUCHDOWN_ALT_M      20.0f   // 落地判定：高度門檻 (m)
#define FSM_MAIN_TRIGGER_CONSEC_N 5U     // 主傘動態高度觸發：連續 5 週期(50ms)防雜訊。
                                          // 比照起飛 a_z 路徑／頂點判定：h_est 單筆離群值
                                          // 不得直接觸發主傘展開，須連續成立才算數。

#define FSM_STEP_PERIOD_MS       10U     // 呼叫頻率契約：100 Hz
#define FSM_STEP_PERIOD_S        0.010f  // COAST 速度差分用的週期 (s)

/* === Profile 隔離：以下常數依 FLIGHT_PROFILE_ELEVATOR（board_config.h）分組 ===
 * 0 = 飛行 profile（預設，真實彈道門檻）；1 = 電梯測試 profile（門檻縮放 + 強制
 * baro 降級鏈，見 board_config.h 的 FEATURE_FORCE_BARO_ONLY 推導）。 */
#if FLIGHT_PROFILE_ELEVATOR

#define FSM_LIFTOFF_BARO_ALT_M   5.0f     // 電梯井道淺（頂樓僅 30m），10m/20m 飛行門檻不會越過（原 3.0m，使用者改為 5.0m）
#define FSM_BURNOUT_ACCEL_G      2.0f     // 電梯全程恆 1g，門檻恆滿足 → 實質由 FSM_BURNOUT_MIN_MS 時間制燒完
#define FSM_BARO_APOGEE_DROP_M   2.0f     // 頂樓僅 30m，10m 飛行門檻在電梯剖面不會回落
#define FSM_BARO_APOGEE_CONSEC   40U      // 400ms：電梯氣壓瞬變（開關門/風壓）比火箭噪聲更慢，需更長防雜訊窗
#define FSM_FAILSAFE_APOGEE_MS   120000U  // 電梯單趟可達分鐘級，15s 飛行版失效保護會誤點火，大幅放寬
#define FSM_FAILSAFE_APOGEE_BACKUP_MS FSM_FAILSAFE_APOGEE_MS  // 電梯 profile 動態預測本就關閉
                                          // （FSM_APOGEE_DYNAMIC_PREDICT_ENABLED=0），主/副無提前量
                                          // 差異，副航電失效保護沿用同一值即可。
#define FSM_MAIN_WATCHDOG_MS     300000U  // 同上，看門狗跟著放寬避免誤觸發主傘
#define FSM_FB_MAIN_ALT_M        8.0f     // 電梯頂樓 30m，150m 飛行降級門檻不會觸發，改用接近地面高度
#define FSM_FB_TOUCHDOWN_ALT_M   5.0f     // 電梯地面基準附近即視為「落地」（頂樓/1樓皆遠低於 30m 飛行門檻）
#define TARGET_MAIN_ALTITUDE     10.0f    // 電梯 EKF 開啟後主傘於下降回到 10m 觸發
                                          // （使用者決策：井道僅 ~35m，150m 飛行值不合理）
#define DROGUE_LEAD_TIME_S       1.0f     // 電梯副傘頂點提前開傘 (s)：井道僅 ~30m，4s lead 不合尺度
#define FSM_MAIN_MAX_ALT_LIMIT_M 600.0f   // 主傘高度上限門檻 (m)：高度至少低於 600m 才允許觸發主傘
#define FSM_APOGEE_DYNAMIC_PREDICT_ENABLED 0  // 電梯頂點判定路徑1（動態預測）關閉：該路徑於減速度
                                          // 不可信時 fallback 假設自由落體重力（−9.80665），此假設
                                          // 僅在「v_est 隨時間由大降到 0」的真實彈道 COAST 段成立；
                                          // 電梯全程近似等速（~1.5m/s，從未「大」過），t_to_apogee=
                                          // v/g 從爬升第一秒就落在 lead time 內，會在爬升初期誤判
                                          // 「快到頂點」而誤點副傘（實測：t=40ms 即誤觸發）。電梯僅
                                          // 留路徑2（v_est 過零）/路徑3（高度自峰值回落），兩者皆為
                                          // 「已發生的事實」判斷、不依賴彈道 decel 假設，對電梯成立。

#else /* !FLIGHT_PROFILE_ELEVATOR：飛行 profile（預設，台灣盃 2026 3.16km 彈道） */

#define FSM_LIFTOFF_BARO_ALT_M   20.0f    // 起飛第三冗餘：baro 相對高度門檻 (m)
#define FSM_BURNOUT_ACCEL_G      0.5f     // 馬達燒完：加速度低於此值 (g)
#define FSM_BARO_APOGEE_DROP_M   10.0f    // baro 原始高度自峰值回落門檻 (m)（≈BMP388 噪聲 16σ）
#define FSM_BARO_APOGEE_CONSEC   20U      // 連續 20 週期（200ms）成立（防雜訊）
#define FSM_FAILSAFE_APOGEE_MS   28221U   // 起飛起算強制點火副傘 (ms)：依 OpenRocket V3 模擬最高點時間 (t_apogee=27.221s) + 1.0s 設定 (27.221s + 1.0s = 28.221s = 28221ms)
                                          // 強制失效保護點火 timer 設定為最高點 + 1s，確保感測器失靈時於頂點過後 1 秒內強制開引傘。
                                          // ⚠ 僅供主航電使用（主航電正常觸發點在 t_apogee−DROGUE_LEAD_TIME_S
                                          // ≈23.2s，此失效保護與之相距 5s 餘裕）；副航電專用值見下方
                                          // DROGUE_LEAD_TIME_S 定義後的 FSM_FAILSAFE_APOGEE_BACKUP_MS。
#define FSM_MAIN_WATCHDOG_MS     248000U  // 主傘部署：飛行總時間看門狗 (ms)。依 v2/v3 兩份 OpenRocket 模擬重推：
                                          // 須 > 標稱主傘高度路徑觸發時間 (v2 t=241.57s / v3 t=241.47s) + 餘裕，
                                          // 且 < 「高度路徑全程未觸發」失效情境下副傘單獨墜地的時間估計
                                          // （依實測副傘下降率 ~13.8 m/s 外插：v2≈262.4s / v3≈263.1s）− 餘裕，
                                          // 因為看門狗正是為此失效情境存在（感測器全盲、高度路徑永不成立時的
                                          // 最後補開手段），若晚於墜地估計即形同失效。248s：距標稱主傘 +6.4~
                                          // 6.5s（防抖動綽綽有餘）、距最壞墜地估計仍有 14.4~15.1s 安全餘裕。
                                          // （舊值 277800=1.15×t_main 已晚於兩模擬的墜地估計與實際落地紀錄，
                                          // 形同失效，已改正）
#define FSM_FB_MAIN_ALT_M        350.0f   // 降級主傘高度 (m)：對齊 300m 開傘目標 + 50m 下降展開餘裕
#define FSM_FB_TOUCHDOWN_ALT_M   30.0f    // 降級落地高度門檻 (m)
#define TARGET_MAIN_ALTITUDE     300.0f   // 目標主傘完全張開高度 (m)
#define DROGUE_LEAD_TIME_S       4.0f     // 飛行副傘頂點提前開傘 (s)：改回 4.0s（曾一度調整為 3.0s，
                                          // 使用者決策改回原值）
#define FSM_FAILSAFE_APOGEE_BACKUP_MS (FSM_FAILSAFE_APOGEE_MS + (uint32_t)(DROGUE_LEAD_TIME_S * 1000.0f))
                                          // 副航電專用失效保護 (ms)：副航電已改為只在「真頂點」才開引傘
                                          // （見 fsm.c apogee_condition），正常觸發點在 t_apogee≈27.221s，
                                          // 比主航電晚了 DROGUE_LEAD_TIME_S(4s)。若沿用主航電那顆
                                          // FSM_FAILSAFE_APOGEE_MS(28.221s)，與副航電正常觸發點只差 1s，
                                          // 餘裕過薄、容易與正常偵測搶跑；故整段順延 DROGUE_LEAD_TIME_S，
                                          // 得 32.221s（32221ms），與副航電正常觸發點維持與主航電相同的
                                          // 5s 餘裕。
#define FSM_MAIN_MAX_ALT_LIMIT_M 600.0f   // 主傘高度上限門檻 (m)：高度至少低於 600m 才允許觸發主傘
#define FSM_APOGEE_DYNAMIC_PREDICT_ENABLED 1  // 飛行 profile：COAST 段 v_est 由大降到 0，動態預測
                                          // fallback 假設（減速度收斂到重力）成立，見電梯 profile
                                          // 分支的對應註解。

#endif /* FLIGHT_PROFILE_ELEVATOR */

#define FSM_BURNOUT_MIN_MS       4000U   // 馬達燒完：起飛後最短時間 (ms)（兩 profile 共用；電梯以此時間制燒完）。
                                          // 原 1500ms，真實燒完 ~6.6s（見 OpenRocket v3 分析），1500~6600ms
                                          // 之間若推力瞬跌+50ms 連續成立可能提前誤轉 COAST（無回頭路）；
                                          // 使用者決策改為 4000ms，加大對推力段中段誤判的餘裕，仍安全低於
                                          // 真實燒完時間。
#define FSM_BURNOUT_ACCEL_CONSEC_N 5U    // 燒完判定：連續 5 週期(50ms)低於門檻才算數。
                                          // a_z_g 為單筆原始取樣（無濾波，見 main.c），比照
                                          // 起飛 a_z 路徑防手震機制：單一取樣點的振動/雜訊
                                          // 掉點不得使 BOOST 永久提前結束（COAST 無回頭路）。

/* === P0-B：頂點失效保護與 baro 原始趨勢交叉檢查（皆不依賴 EKF） === */
/* FSM_FAILSAFE_APOGEE_MS（飛行 profile，主航電專用）已依 OpenRocket 模擬數據推導完成，
 * 見下方 Profile 隔離區塊之飛行 profile 分支（唯一真實來源，勿在此重複抄值）。
 * 日後若重新模擬或更換火箭組態，務必回到該處更新，並仍需滿足：
 * ≥ t_apogee_sim + 4s（使用者決策：寧可餘裕薄一點提早於頂點附近點火，也不要餘裕
 * 拉大到下降已有明顯速度才觸發——副傘高速展開有繩索斷裂風險，見該巨集旁註解）、
 * ≤ FSM_MAIN_WATCHDOG_MS − 5s，保留副傘→主傘序列安全餘裕。
 * ⚠ 主/副航電各自有獨立的失效保護計時器（fsm.c 依 IS_PRIMARY 二選一）：主航電用
 * FSM_FAILSAFE_APOGEE_MS，副航電用 FSM_FAILSAFE_APOGEE_BACKUP_MS——因副航電已改為
 * 只在真頂點才開引傘（不再提前 DROGUE_LEAD_TIME_S），若沿用同一顆計時器，餘裕會被
 * 提前量吃掉，見 FSM_FAILSAFE_APOGEE_BACKUP_MS 定義旁註解。 */

/* sensor_bits 輸入位元契約（P0-D 起由 sensor_health 餵入；P0-B 起 FSM 即依此閘控 baro 路徑） */
#define FSM_SB_BARO_FAULT        0x01U   // baro 失效/不可信 → 停用 baro 交叉檢查與 baro 起飛冗餘

/* === P0-C：估計器（est，見 FSM_Input_t 下方註解）unhealthy 時的 raw-baro 降級開傘鏈參數 ===
 * est_healthy=0（現行 main.c 接線下實為 VF 健康位：VF_baro 創新值 300ms 內無被接受，
 * 或 EKF_GetHealthBits()!=0/EKF_Task >300ms 無更新（僅 FEATURE_VFILTER_FSM=0 時才是
 * 真 EKF 健康位，見 fsm.h 下方 FSM_Input_t 註解）；電梯 profile 則透過
 * FEATURE_FORCE_BARO_ONLY 強制恆為 0）時：
 *   起飛   = a_z 或 baro 冗餘（h_est 路徑停用）
 *   頂點   = baro 趨勢規則（P0-B；est 三路徑停用，防發散值誤點火）
 *   主傘   = baro 相對高度 ≤ FSM_FB_MAIN_ALT_M 或既有看門狗
 *   落地   = 2s 視窗內 |Δbaro| < 2m 且 baro < FSM_FB_TOUCHDOWN_ALT_M */
#define FSM_FB_TOUCHDOWN_WIN_MS  2000U   // 降級落地判定視窗
#define FSM_FB_TOUCHDOWN_DELTA_M 2.0f    // 視窗內 baro 變化量門檻

/* === P0-F：熱啟動驗證鏈參數 === */
/* ★ 這個上限比對的是「起飛後經過時間」（FlashRingPacket_t::flight_tick_ms），
 * 不是封包的 tick_ms（開機以來的絕對 tick，重啟後歸零、跨重啟無意義）。
 *
 * ★2026-07-31：60000 → 300000（使用者決策：完整飛行約 278s，取整加餘裕 300s）。
 * 舊值 60s 只蓋到頂點（t≈27s）後約 30s，整段傘降都落在窗外——而開傘衝擊、低溫、
 * 電池接觸不良這些 brownout 來源正好集中在傘降段，等於把最需要熱重啟的時段排除掉。
 * 上界的物理意義：熱重啟只還原 BOOST..DESCENT（見 FSM_HotStartDecide），板子最晚
 * 還寫得出這個範圍封包的時刻＝離開 DESCENT，最壞情況由主傘看門狗
 * FSM_MAIN_WATCHDOG_MS（飛行 248s／電梯 300s，皆自 flight_start_ms 起算）決定；
 * 300s 已涵蓋飛行 profile 全程並留 52s 餘裕。
 * ⚠ 本上限「不是」防陳舊資料的機制：flight_tick_ms 是寫入當下就凍住的值，上一場飛行
 *   t+45s 的封包與這一場 t+45s 的封包數值完全相同，此閘分不出來。真正在擋殘留資料的是
 *   ①冷開機預擦會擦掉上一場最後一筆（CRC 直接不過）②FSM_HOTSTART_MAX_ALT_DIFF_M 高度
 *   連續性（★電梯 profile 井道僅 ~30m，該閘形同虛設）。若要真正防陳舊，應改讀 RCC 重置
 *   原因暫存器（IWDG/SOFT/PIN reset ⇒ 板子上一刻仍在跑；POR ⇒ 重新上電），main.c 已讀取
 *   reset_csr 但目前僅供列印。
 * ⚠ 電梯 profile 的看門狗恰好也是 300s，故電梯測試中「看門狗觸發前最後一瞬間」的封包會
 *   剛好落在窗外（門檻用 >=）。實務上無妨；若在意可改 305000。 */
#define FSM_HOTSTART_MAX_TICK_MS    300000U // 起飛後經過時間上限（完整飛行 ~278s + 餘裕）
#define FSM_HOTSTART_MAX_ALT_DIFF_M 300.0f  // 封包 baro 與當下 baro 容許差（IWDG 2.05s + 開機期下落餘裕）

/* === 事件（供呼叫端列印 / 記錄；一次 FSM_Step 至多一個事件） === */
typedef enum {
    FSM_EVT_NONE = 0,
    FSM_EVT_ARMED,         // PAD → PAD_ARMED (地面站已發出並收到武裝命令)
    FSM_EVT_DISARMED,      // PAD_ARMED → PAD (地面站解除武裝或武裝逾時自動解除)
    FSM_EVT_LIFTOFF,       // PAD_ARMED → BOOST (起飛)
    FSM_EVT_BURNOUT,       // BOOST → COAST
    FSM_EVT_DEPLOY_DROGUE, // COAST → DEPLOY_DROGUE（預測式提前觸發，同時 fire_drogue 啟動馬達）
    FSM_EVT_APOGEE_FAILSAFE, // 失效保護計時器強制觸發（BOOST/COAST → DEPLOY_DROGUE，同時 fire_drogue，
                              // 走同一顆馬達狀態，非直接跳 APOGEE，確保失效保護也完整跑滿 4s 展開）
    FSM_EVT_DROGUE_DONE,   // DEPLOY_DROGUE → APOGEE（4s 到，同時 release_drogue 停馬達）
    FSM_EVT_MAIN_DEPLOY,   // DESCENT → MAIN_DEPLOY（同時 deploy_main）
    FSM_EVT_MAIN_OPEN,     // MAIN_DEPLOY → LANDED（充氣等待結束）
    FSM_EVT_TOUCHDOWN      // LANDED 內落地確認（同時 start_buzzer）
} FSM_Event_t;

/* === 輸入快照（呼叫端 main.c 組裝） === */
typedef struct {
    uint32_t now_ms;         // HAL_GetTick()
    float    h_est;          // 高度估計 (m)。現行由 main.c 接線為垂直濾波器 VF（FEATURE_VFILTER_FSM=1，
                              // vertical_filter.h 的 3 狀態 Kalman）；EKF 僅供姿態/遙測/記錄，不參與此欄位。
    float    v_est;          // 垂直速度估計 (m/s)。來源同上（VF），非 EKF。
    float    a_z_g;          // 高 G 垂直加速度 body-frame (g)
    float    baro_alt_rel;   // baro 原始高度 − pad 基準 (m)；P0-B 起使用
    uint8_t  est_calibrated; // h_est/v_est 估計器（現行 VF）已可用：main.c 接線為 pad_ref_valid
                              // （VF 無獨立校準階段，pad 氣壓基準建立即視為就緒）。
                              // 命名歷史：原欄位名 ekf_calibrated，因 main.c 早已把 h_est/v_est 換成
                              // VF 輸出、卻仍覆寫這兩個「EKF」命名的欄位，造成審查誤讀，已更名為 est_*。
    uint8_t  est_healthy;    // h_est/v_est 估計器健康：現行 main.c 接線為 VF 健康（最近 300ms 內有 baro
                              // 創新值被接受）。僅當 FEATURE_VFILTER_FSM=0 時才會是真正的
                              // EKF_GetHealthBits()==0 && EKF_Task 300ms 內有更新（見 main.c FSM_Update）。
                              // 命名歷史同上，原名 ekf_healthy。
    uint8_t  sensor_bits;    // 感測器故障位元（P0-D 起接 sensor_health；目前恆 0）
    uint8_t  uplink_armed;   // 1 = 地面站已傳送上行 ARM 指令且未逾時
    uint8_t  peer_drogue_cmd; // D1 加法互救：對端已開副傘（LinkPeer.drogue_latched）。
                              // 嚴格 OR：僅「額外」促成本板開副傘，對端沉默則各路徑照舊獨立；
                              // arm-interlock = 已處 COAST（必經 BOOST）+ 起飛時間鎖，擋台上誤觸。
    uint8_t  flash_pool_ready; // 1 = flash 預擦池已達 FLASH_RING_PREERASE_TARGET。
                              // ★2026-08-01：已無 ARM 閘（使用者決策：開機自動補擦到達標），
                              // main.c 固定餵 1；本欄與下方 arm_blocked_flash 保留為純邏輯層
                              // 防線（tests/test_fsm.c 仍覆蓋），要重啟防護見 FSM_Update 註解。
} FSM_Input_t;

/* === 動作（呼叫端立即執行；硬體動作先於 printf） === */
typedef struct {
    uint8_t fire_drogue;     // 1 = 啟動副傘 DC 馬達（PD13 HIGH，持續 FSM_DROGUE_MOTOR_RUN_MS）
    uint8_t release_drogue;  // 1 = 停止馬達（PD13 LOW）
    uint8_t deploy_main;     // 1 = 主傘釋放：PD14 純 GPIO 拉高 SERVO_MAIN_HIGH_MS(1.5s)，
                              //     不啟動 PWM；同時經板間鏈路呼叫對端一起拉高（無互斥握手）
    uint8_t start_buzzer;    // 1 = 開啟尋標蜂鳴器
    uint8_t event;           // FSM_Event_t
    float   apogee_t_pred;   // EVT_APOGEE 時的預估頂點時間 (s)，供事件列印
    uint8_t arm_blocked_flash; // 1 = 本週期地面站已送 ARM，但 flash_pool_ready=0 故仍留在
                              // STATE_PAD 未轉移；呼叫端據此印出原因並經 LoRa/USB 回報。
} FSM_Action_t;

/* === 內部狀態（原 FSM_Update 的 static 區域變數 + main.c 全域收納） === */
typedef struct {
    FlightState_t state;
    uint32_t flight_start_ms;     // 起飛基準 tick（原 flight_start_tick）
    uint32_t state_entered_ms;    // 當前狀態進入 tick
    float    max_altitude;        // 觀測到的最大 EKF 高度
    float    last_vel_z;          // 上一週期速度（COAST 差分估加速度用）
    uint8_t  consec_apogee_counts;
    uint8_t  touchdown_latched;   // 落地一次性觸發（原 g_touchdown_tick==0 判斷）
    uint8_t  drogue_fired;        // 副傘已點火鎖存（熱啟動防二次點火，P0-F 使用）
    float    max_alt_baro;        // COAST 期 baro 原始相對高度滾動峰值（P0-B 交叉檢查）
    uint8_t  consec_baro_drop;    // baro 自峰值回落連續週期計數（P0-B）
    uint8_t  failsafe_fired;      // 失效保護計時器已觸發（遙測 TELEM_FLAG_FAILSAFE）
    float    fb_td_ref_alt;       // 降級落地判定：2s 視窗基準 baro 高度（P0-C）
    uint32_t fb_td_ref_tick;      // 降級落地判定：視窗起始 tick（0=未初始化）
    uint8_t  consec_liftoff_az;   // 起飛 a_z 路徑連續超門檻週期數（防手震瞬間尖峰）
    uint32_t drogue_start_ms;     // 副傘 DC 馬達啟動 tick（獨立背景 8s 定時器解耦用）
    uint8_t  consec_burnout_az;   // 燒完判定連續低於門檻週期數（防單筆 a_z 掉點誤判）
    uint8_t  consec_main_trigger; // 主傘動態高度觸發連續成立週期數（防單筆離群值誤展開）
} FSM_Context_t;

/* === P0-F：熱啟動決策（純函式，main.c 收集輸入後呼叫） === */
typedef struct {
    uint8_t       restore;       // 1 = 恢復飛行狀態；0 = 回 STATE_PAD 完整重校準
    FlightState_t state;         // 恢復目標狀態（已套用防二次點火政策）
    uint8_t       drogue_fired;  // 交給 FSM_Init 的點火鎖存
} FSM_HotStartDecision_t;

/*
 * 熱啟動驗證鏈（任一失敗 → restore=0 回 PAD）：
 *   1. pkt_valid：ring 末筆封包 CRC 有效
 *   2. 封包狀態 ∈ BOOST..DESCENT（正常落地後末筆為 LANDED → 自然回 PAD）
 *   3. pkt_tick_ms < 60s：防上次飛行殘留的飛行中封包在地面誤恢復
 *   4. |封包 baro − 當下 baro| < 300m：高度連續性
 * 重點火政策：pkt_drogue_fired=1 → 恢復目標強制 ≥ STATE_DESCENT（杜絕二次點火）。
 */
FSM_HotStartDecision_t FSM_HotStartDecide(uint8_t pkt_valid,
                                          uint8_t pkt_fsm_state,
                                          uint32_t pkt_tick_ms,
                                          float pkt_baro_alt_m,
                                          float cur_baro_alt_m,
                                          uint8_t pkt_drogue_fired);

/*
 * 初始化 / 熱啟動進入指定狀態。
 *   s0                  : 初始狀態（正常開機 STATE_PAD；熱啟動為恢復狀態）
 *   now_ms              : 當前 tick（state_entered_ms 基準）
 *   flight_start_ms     : 起飛基準 tick（正常開機 0；熱啟動 = now − 封包 tick）
 *   drogue_already_fired: 熱啟動時自 Flash 封包 flags 還原（防二次點火）
 */
void FSM_SetState(FSM_Context_t *ctx, FlightState_t target_state);

void FSM_Init(FSM_Context_t *ctx, FlightState_t s0, uint32_t now_ms,
              uint32_t flight_start_ms, uint8_t drogue_already_fired);

/* 單步執行（100 Hz）。回傳本週期需執行的動作與事件。 */
FSM_Action_t FSM_Step(FSM_Context_t *ctx, const FSM_Input_t *in);

#ifdef __cplusplus
}
#endif

#endif /* FSM_H */
