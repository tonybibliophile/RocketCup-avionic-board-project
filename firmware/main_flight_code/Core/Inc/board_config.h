/*
 * board_config.h — 主/備雙航電編譯期角色與功能旗標（單一程式碼、雙板）
 * ===========================================================================
 * 同一份 firmware 以 BOARD_ROLE 切出三個 binary：
 *   - ROLE_PRIMARY（主航電）：完整功能（GPS / 磁力計 / LoRa 433+920）+ 板間鏈路。
 *   - ROLE_BACKUP （備援航電）：只跑 MCU-layer 飛控（IMU/baro/highG → EKF → FSM
 *     → 點火），關閉 GPS / 磁力計 / LoRa；其餘飛控、Flash 記錄、板間鏈路照常。
 *   - ROLE_GROUND （地面站）：硬體與航電板相同，只「接收」LoRa 下行遙測（E22 433 +
 *     E80 920）+ 讀自身 GPS，對齊時間戳後記錄到 SD/Flash 並以 USB-CDC 串流給 PC；
 *     關閉所有飛控感測器（IMU/baro/highG/磁力計）、EKF/FSM/點火與板間鏈路。
 *
 * 板間實體層（已選定）：兩塊「相同」的 MCU 板各自跑 USART2 硬體全雙工（PA2=TX/
 * PA3=RX），於板間排線把 TX/RX 交叉一次（A.PA2→B.PA3、A.PA3→B.PA2）即可對接。
 * 兩板 UART 程式完全相同，無軟體 bit-bang（F407 USART 無硬體 SWAP 位元，硬體腳位
 * 不可對調，故以排線交叉取代「韌體互換腳位」，換取硬體 UART 的可靠度）。
 *
 * 角色設定方式（擇一）：
 *   - STM32CubeIDE：新增 build configuration「Debug_Backup」，preprocessor 加
 *     BOARD_ROLE=ROLE_BACKUP（其餘與 Debug 相同）。
 *   - 命令列：頂層 `make build-backup`（自動切換下方 BOARD_ROLE 行後建置）。
 *
 * 本檔不依賴 HAL / CMSIS / FreeRTOS，純常數巨集，可被純邏輯模組與 host 測試 include。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* === 版本號碼 === */
#define FIRMWARE_VERSION "v1.0.2"

/* === 角色 === */
#define ROLE_PRIMARY 0
#define ROLE_BACKUP  1
#define ROLE_GROUND  2   /* 地面站接收器：硬體與航電板相同，只收 LoRa 下行 + 自身 GPS */

#ifndef BOARD_ROLE
#define BOARD_ROLE   ROLE_PRIMARY   /* MAKE_ROLE_LINE */
#endif

#define IS_PRIMARY   (BOARD_ROLE == ROLE_PRIMARY)
#define IS_BACKUP    (BOARD_ROLE == ROLE_BACKUP)
#define IS_GROUND    (BOARD_ROLE == ROLE_GROUND)

/* === 地面站上行發射權限（僅 IS_GROUND 有意義；`make build-ground` 與
 * `make build-ground-tx` 共用同一份原始碼，靠這個旗標切出兩種 binary）===
 *   0 = RX-only（預設，`make flash-ground`）：地面站只能「接收」下行遙測。
 *       gs_lora_test.c 命令列裡會經 LoRa 發射的指令（ping/arm/disarm/bench/
 *       recalib/recovery/deploy/tx）一律被韌體拒絕並回傳提示——刻意不透過
 *       GUI 擋（GUI 一律照送，是否接受由韌體決定，與 BENCH/RECOVERY 既有
 *       的「GUI 不做本地攔截」原則一致），故按鈕仍會顯示、仍可點擊。
 *       本地 flash 清除、e22/e80 RF 參數調整、stats/help/role 等純本地
 *       命令不受影響，兩種 binary 都可用。
 *   1 = TX-capable（`make flash-ground-tx`）：完整功能，可發射上行指令。
 * `role` 命令回覆會附上 tx=0/1，供 GUI 顯示徽章區分兩種地面站（顯示用，
 * 非攔截用）。 */
#ifndef GS_LORA_TX_ENABLE
#define GS_LORA_TX_ENABLE 0   /* MAKE_GS_TX_LINE */
#endif

/* === 功能閘（main.c 以 #if FEATURE_* 包住對應驅動 init / task） ===
 * 備板關閉 GPS(UART6) / 磁力計(I2C1) / LoRa(E22 UART3 + E80 SPI3)。
 * 地面站：開 GPS + LoRa(改 RX) + USB-CDC；關 磁力計 + 整個飛控管線(FEATURE_FLIGHT)。
 * 注意：W25Q128 Flash 與 E80 共用 SPI3（見 w25qxx.c / spi3_bus.h），故 SPI3 周邊本身
 * 不關閉（備板/地面站仍需 Flash 記錄）；備板只是不啟用 E80 驅動並按住其 RST 釋放匯流排，
 * 地面站則啟用 E80 但走 RX（讀 FIFO 與 Flash 同以 SPI3 互斥鎖序列化）。 */
#define FEATURE_GPS    (IS_PRIMARY || IS_GROUND)  /* NEO-M9N GPS（UART6）：主航電 + 地面站 */
#define FEATURE_MAG    IS_PRIMARY                 /* MMC5983MA 磁力計（I2C1）：僅主航電 */
#define FEATURE_LORA   (IS_PRIMARY || IS_GROUND)  /* LoRa 硬體 init：E22 433(UART3) + E80 920(SPI3) */
#define FEATURE_LORA_TX IS_PRIMARY                /* 下行遙測「發送」：僅主航電 */
#define FEATURE_LORA_RX IS_GROUND                 /* 下行遙測「接收」：僅地面站 */
#define FEATURE_FLASH   1                         /* 電梯場測：開啟記錄，無法全程接筆電監看時事後回放用 */
#ifndef FEATURE_FLASH_BOOT_FULL_ERASE
#define FEATURE_FLASH_BOOT_FULL_ERASE 0           /* 開機整環全擦(~3min)：0=關閉方便 bench 反覆重開機測試；
                                                     * ★正式飛行前務必改回 1（見 main.c 開機序列說明） */
#endif
#ifndef FEATURE_HOTSTART
#define FEATURE_HOTSTART 1                        /* 空中熱啟動恢復（飛行中重啟直接回原狀態、略過開機預擦） */
#endif
#define FEATURE_BUZZER  1                         /* 蜂鳴器開關（1=開啟，0=靜音以防測試時噪音過大） */

/* === 板上使用者按鈕（USER_BT1=PA1 / USER_BT2=PC3）總開關 ===
 *   0 = 關閉（預設）：韌體完全不讀按鈕，開機不會因按鈕狀態改變行為。
 *   1 = 開啟：恢復下列按鈕功能。
 * 目前掛在按鈕上的功能只有一項：
 *   - USER_BT1 開機按住 → Flash_DumpAll() 把 W25Q128 三分區全 dump 到 USART2
 *     （見 main.c StartDefaultTask 開機序列）。需要救上一次飛行資料時再改回 1。
 * USER_BT2 從未被讀取，僅在 MX_GPIO_Init() 設定腳位（CubeMX 產生區，不動）。
 * 註：本旗標只關「軟體有沒有讀按鈕」，GPIO 初始化保留，改回 1 即可復原。 */
#ifndef FEATURE_USER_BUTTONS
#define FEATURE_USER_BUTTONS 0
#endif

/* === 飛行 profile：場測（電梯）與真實飛行的門檻/降級鏈切換 ===
 * 電梯場測（垂直電梯井道，無法完整重現彈道）沿用真實飛行同一份 FSM，但頂樓僅
 * 30m、全程等速 1g、不具備真實加速度剖面，因此需要一組縮放後的門檻（見
 * fsm.h 內以本旗標分組的 #if/#else 常數表）。
 *   0 = 飛行 profile（預設，真實彈道門檻 + 估計器路徑正常參與）
 *   1 = 電梯測試 profile（門檻縮放） */
#ifndef FLIGHT_PROFILE_ELEVATOR
#define FLIGHT_PROFILE_ELEVATOR 1
#endif

/* FEATURE_FORCE_BARO_ONLY：改為固定 0，不再由 FLIGHT_PROFILE_ELEVATOR 推導。
 * 原設計電梯 profile 全程強制純氣壓降級鏈，理由是「電梯無法讓估計器觀察到有意義的
 * 運動特徵，硬套用動態路徑只會誤判」——但這個「誤判」的真正根因只在 fsm.c COAST
 * 頂點判定的路徑1（動態預測）：其 decel fallback 假設「v_est 隨時間由大降到0」，僅
 * 真實彈道成立，電梯全程近似等速違反此假設（實測：爬升第 40ms 即誤判「快到頂點」
 * 而誤點副傘）。已改用 fsm.h 的 FSM_APOGEE_DYNAMIC_PREDICT_ENABLED 精準關掉電梯的
 * 路徑1（僅此一條），其餘依賴估計器 h_est/v_est 的路徑（頂點路徑2/3、主傘動態高度、
 * 下降/落地判定）對電梯剖面本身並無此問題，不需要整條強制降級——故電梯 profile
 * 現在也讓估計器（VF）路徑正常參與，僅動態預測路徑保持關閉。 */
#define FEATURE_FORCE_BARO_ONLY 0

/* === 垂直通道 Kalman 濾波器（vertical_filter.h，Schultz 火箭高度計架構） ===
 *   FEATURE_VFILTER      = 1：編譯並執行濾波器，1Hz 與 EKF 對照列印（供 A/B 比較）。
 *   FEATURE_VFILTER_FSM  = 1：【已啟用】VF 為開傘主估計器——FSM 的 h_est/v_est 直接
 *                             取自本濾波器，兩 profile 皆同；EKF 僅供姿態/遙測/記錄。
 *                             改用獨立垂直通道的動機：EKF 垂直速度靜置/地面實測會漂移。 */
#define FEATURE_VFILTER      1
#define FEATURE_VFILTER_FSM  1

/* 開傘（PWM 舵機）地面自測：設 1 → 每次重啟自動跑一次舵機釋放序列後停住，不進飛控。
 * 台面測試專用，飛行前務必設回 0。詳見 pyro_selftest.h（含刪除方式）。 */
#ifndef FEATURE_PYRO_SELFTEST
#define FEATURE_PYRO_SELFTEST 0
#endif

/* === 433 純發送實驗開關（台面診斷用，★飛行前務必設回 0）=========================
 * 設 1 → 主航電只發 433 下行、完全不讀 433 上行：
 *   ① 不呼叫 UplinkCmd_Init()，USART3 從頭到尾不掛 ReceiveToIdle（MCU 不讀 RX）
 *   ② 遙測任務走無上行分支：取消上行接收窗（main.c UPLINK_LISTEN_PERIOD_MS/HOLD_MS，
 *      每 3s 靜默 800ms）→ 433 發送密度增加 ~27%
 *   ③ 不發 ACK 幀（ACK 本來就是回應上行命令用的）
 * 用途：排除「同一顆 E22 邊收邊發 / 上行接收窗」是否干擾下行品質。
 * ⚠ 注意這只是「MCU 不讀 UART3」，不是「模組不收」——E22 透傳模式在不發射時
 *   硬體上仍在監聽，要真正停掉射頻接收得進 WOR/睡眠模式，那會連發射一起停掉。
 *   模組收到的雜訊仍會吐進 USART3、觸發 ORE，但無人讀取亦無人處理，無副作用。
 * ⚠⚠ 安全：設 1 時地面站的 ARM / DEPLOY / RECOVERY 遠端指令「全部失效」，
 *     火箭只剩 FSM 自動開傘。純台面測試用，絕對不可帶著這個設定飛行。 */
#ifndef LORA433_TX_ONLY
#define LORA433_TX_ONLY 0
#endif

/* === 433 純接收實驗開關（上行 bring-up 專用，★飛行前務必設回 0）================
 * 設 1 → 主航電「完全不發 433」（下行遙測與 ACK 都不走 433），但上行接收照常掛載：
 *   → E22 透傳模式在不發射時硬體上就是在接收，等於整段時間都是接收窗。
 * 用途：把「上行收不到」這件事一刀切成兩半 ——
 *   ① 設 1 之後地面站 arm 就能通 → 問題出在「半雙工／接收窗開得不夠」，
 *      調 UPLINK_LISTEN_HOLD_MS / UPLINK_LISTEN_PERIOD_MS 即可（main.c）。
 *   ② 設 1 之後 arm 還是不通 → 跟窗完全無關，是 RF 層問題：頻道(CH)/空中速率兩端
 *      不一致、模組位址(ADDH/ADDL)不同、天線/接線/供電，或地面站根本沒發出去
 *      （先看地面站那行 `[UPLINK] … burst 結束：ok=?`）。
 * 判讀就看主航電的 `[UPLINK_STAT] raw=` 有沒有在漲（見 main.c 1Hz 診斷行）。
 * ⚠ 設 1 時 433 下行遙測會完全消失（920 不受影響，仍可看遙測與 ACK）。
 * ⚠ 不可與 LORA433_TX_ONLY 同時設 1（互斥，見下方 #error）。 */
#ifndef LORA433_RX_ONLY
#define LORA433_RX_ONLY 0
#endif

#if LORA433_TX_ONLY && LORA433_RX_ONLY
#error "LORA433_TX_ONLY 與 LORA433_RX_ONLY 互斥，不可同時設 1"
#endif

/* 上行手動開傘：地面站經 433 反向打命令，火箭在下行之外空出 1/10 時槽接收。
 * 僅主航電（有 E22 TX/RX + 飛控點火輸出）；地面站送命令端走 IS_GROUND 的 gs_lora_test。
 * 安全：兩段式 ARM→DEPLOY + ARM 逾時自動解除（見 uplink_cmd.c / uplink_proto.h）。 */
#if LORA433_TX_ONLY
#define FEATURE_UPLINK_DEPLOY  0
#else
#define FEATURE_UPLINK_DEPLOY  IS_PRIMARY
#endif

/* 飛控管線（IMU/baro/highG → EKF → FSM → 點火/傘控）：主 + 備皆跑，地面站關閉省資源。 */
#define FEATURE_FLIGHT (!IS_GROUND)

/* 除錯用 USB-CDC printf 直通（台面測試開關，★本檔預設為 1＝開啟）：
 * 設 1 → 不論角色，一律初始化 USB_DEVICE 並讓 printf 改走 USB 虛擬序列埠（見 main.c
 * _write），免接 SWD 除錯器、免佔用被板間鏈路用掉的 USART2，插 USB 線用序列埠工具
 * 即可看 log。best-effort（PC 未讀取即丟棄），不阻塞飛控路徑。
 *
 * ★飛行前該不該改回 0——理由不是效能，先前註解寫「會多出常駐 CPU 佔用」是錯的：
 *   - SOF 週期中斷已關（usbd_conf.c Init.Sof_enable = DISABLE），沒有那顆 1kHz 中斷；
 *   - 無資料時 host 對 bulk IN 的輪詢由硬體 NAK，不進 CPU；
 *   - 實際 TX 流量＝診斷區塊 1Hz、約 12 行（main.c StartDiagnosticTask），約 15 個
 *     64B 封包/秒 × 數 µs ISR ≈ 0.01% CPU；
 *   - 飛行時 USB 線是拔的，沒有 host 就沒有 token 封包，拔線後只剩一次 suspend 中斷。
 *   OTG_FS_IRQn 優先權 4 確實高於感測器 EXTI/DMA（5），可搶佔，但搶佔窗＝一次
 *   HAL_PCD_IRQHandler 約數 µs，相對 EKF 1ms tick 是 <0.5% 抖動，有界可忽略。
 *   真正貴的是 printf 自身的格式化（%f → _dtoa_r，每個 double 數千 cycles，
 *   12 行/秒 ≈ 0.14% CPU）——而這筆成本改走 ITM 也一樣付，關本旗標省不掉。
 *
 * ★該關的真正理由是「指令通道」：本旗標＝1 時 StartDiagnosticTask 會持續受理 USB CDC
 *   文字命令，含 disarm、手動開傘，且不分飛行狀態（見 main.c Parse_Serial_Command）。
 *   這是實體風險，不是效能風險。正解是在那幾個危險指令上補 STATE_PAD/INIT 閘
 *   （比照 bench 點火已有的做法）；補上之後本旗標忘了改回 0 也不再致命。 */
/* ★7/26：曾一度懷疑 MX_USB_DEVICE_Init()（本旗標=1 時掛進主航電開機序列）是
 * 「開機完全無反應」的元兇而暫時關過（見 git log）。後以 ST-Link mode=HOTPLUG
 * 不重置直接 attach 讀 PC，抓到真正根因是 BOOT0 腳位被拉到 System Memory
 * bootloader（PC 落在 0x1FFFxxxx，不在 flash）——每次真重置（含實體 RESET 鍵）
 * 都直接跳 ROM bootloader，跟這顆旗標無關；USB 初始化本身未被證實有問題。
 * 確認 BOOT0 修正、板子已能跑進 FreeRTOS idle task 後，此處改回 1。 */
#ifndef FEATURE_USB_DEBUG_LOG
#define FEATURE_USB_DEBUG_LOG 1
#endif

/* USB 虛擬序列埠（CDC）：地面站固定啟用，串流接收到的遙測 + 自身 GPS 給 PC；
 * 其餘角色僅在上方 FEATURE_USB_DEBUG_LOG 開啟時（台面除錯）才啟用。
 * 設 0 時 MX_USB_DEVICE_Init 被守住 → 不列舉、不進 USB 中斷。但注意上方說明：
 * FEATURE_USB_DEBUG_LOG 目前預設為 1，故主/備航電的預設建置其實是「啟用」CDC；
 * 且啟用的成本主要在指令通道而非 CPU（USB 中斷佔用約 0.01%，飛行拔線後為 0）。 */
#define FEATURE_USB_CDC (IS_GROUND || FEATURE_USB_DEBUG_LOG)

/* 板間鏈路：主備皆需，地面站不參與（非配對飛控板）。啟用後 USART2 改跑二進制鏈路，
 * printf 改走 SWO/ITM 或 USB-CDC（見 FEATURE_USB_DEBUG_LOG / main.c _write 的優先序）。
 * 設 0 可回復「USART2 = printf 除錯橋」的單板開發建置（make monitor 可見，@460800）。 */
#ifndef FEATURE_LINK
#define FEATURE_LINK   (!IS_GROUND)   /* 雙板飛行版：主/備皆跑；地面站不參與 */
#endif

/* === 板間鏈路參數（USART2 硬體全雙工） === */
#define LINK_BAUD             38400U /* 主備兩板 USART2 同此值；短排線餘裕充足 */
#define LINK_TX_PERIOD_MS     50U    /* 自身狀態廣播週期（20 Hz；飛控 100 Hz 每 5 次送一次） */
#define LINK_PEER_TIMEOUT_MS  300U   /* 超過此值無有效封包 → 對端視為失聯（LINK_STATUS_LOST） */
#define LINK_SYNC_TIMEOUT_MS  300U   /* 我方狀態改變後，對端未於此時間內 echo-ACK → 失同步（DESYNC） */

/* === 主傘 PD14 共開時序（servo_arb.h）===
 * ★ 已取消舊的 D2「時間錯開互斥握手」：主傘不再驅動 PWM 舵機，改為 PD14 純 GPIO 拉高
 *   SERVO_MAIN_HIGH_MS 後拉回低。兩板 PD14 於開傘板 diode-OR 合流，純準位訊號同時拉高
 *   無害（會產生破壞性合併脈衝的是兩串非同相 50Hz PWM），故不需要互斥、不需要讓位
 *   guard，也不需要 PREHIGH→PWM 兩段式。使用者決策：兩板同時拉高，一板判到就呼叫對端
 *   一起開（見 main.c 的 peer co-fire 區塊與 servo_arb.h）。 */
#define SERVO_MAIN_HIGH_MS    1500U  /* 主傘 PD14 純 GPIO 拉高時間（不啟 PWM） */

/* === 主/副協同（對稱獨立冗餘；協同只做加法、不否決自身開傘） ===
 * 兩板跑同一份 FSM、各依自身感測器獨立開傘；板間鏈路廣播狀態（LinkPacket_t）供
 * 地面監看與加法協同。共用開傘板、以 diode 做 OR-in gate：
 *   - 副傘 PD13（DC 馬達經 MOSFET，準位訊號）：兩板同時拉高無妨 → 加法 OR 互救。
 *     導通時間依角色分流（主 8s / 副 3s，見 fsm.h FSM_DROGUE_MOTOR_RUN_MS）。
 *   - 主傘 PD14：★不再輸出 PWM（純 GPIO 拉高 SERVO_MAIN_HIGH_MS），故也不再需要
 *     時間錯開互斥；兩板同時拉高，一板判到即呼叫對端一起開（見上方 SERVO_MAIN_HIGH_MS）。 */

/* === GPS-ONLY 隔離除錯開關 ════════════════════════════════════════════════
 * 1 = 關閉「除 GPS 外」的所有射頻/匯流排活動（LoRa 433+920、IMU/baro/highG 飛控管線、
 *     磁力計、Flash 記錄、USB、蜂鳴器、板間鏈路、上行開傘…），只留 GPS(USART6) 與
 *     printf 除錯輸出(USART2)。用途：判斷 GPS 在板上收 0 顆是否為飛控板自身 desense
 *     ——把最可能的干擾源(LoRa/SMPS 負載/SPI/I2C)全靜音後，若衛星數開始上來即坐實干擾。
 *     ★純測試用，量完務必改回 0，否則飛控/遙測/記錄全不會啟動！
 * 0 = 正常（依 BOARD_ROLE 決定各功能）。 */
#ifndef GPS_ONLY_DEBUG
#define GPS_ONLY_DEBUG 0
#endif
#if GPS_ONLY_DEBUG
  #undef  FEATURE_MAG
  #undef  FEATURE_LORA
  #undef  FEATURE_LORA_TX
  #undef  FEATURE_LORA_RX
  #undef  FEATURE_UPLINK_DEPLOY
  #undef  FEATURE_FLIGHT
  #undef  FEATURE_USB_DEBUG_LOG
  #undef  FEATURE_USB_CDC
  #undef  FEATURE_FLASH
  #undef  FEATURE_VFILTER
  #undef  FEATURE_VFILTER_FSM
  #undef  FEATURE_BUZZER
  #undef  FEATURE_PYRO_SELFTEST
  #undef  FEATURE_LINK
  #define FEATURE_MAG           0
  #define FEATURE_LORA          0   /* E22 433 + E80 920 完全不 init、不發射 */
  #define FEATURE_LORA_TX       0
  #define FEATURE_LORA_RX       0
  #define FEATURE_UPLINK_DEPLOY 0
  #define FEATURE_FLIGHT        0   /* 關整條 IMU/baro/highG→EKF→FSM 管線 */
  #define FEATURE_USB_DEBUG_LOG 0
  #define FEATURE_USB_CDC       0
  #define FEATURE_FLASH         0   /* 停 W25Q128 記錄，降低 SPI3 活動 */
  #define FEATURE_VFILTER       0
  #define FEATURE_VFILTER_FSM   0
  #define FEATURE_BUZZER        0
  #define FEATURE_PYRO_SELFTEST 0
  #define FEATURE_LINK          0   /* 板間鏈路本身也是匯流排活動，一併靜音 */
  /* FEATURE_GPS 保持開啟；FEATURE_LINK 強制 0 → printf 走 USART2@460800 可監看 [GPS_RAW] */
#endif

#endif /* BOARD_CONFIG_H */
