/*
 * uplink_cmd.c — 火箭端上行命令處理（433 反向鏈路 → 手動開傘）
 * ===========================================================================
 * 整檔以 #if FEATURE_UPLINK_DEPLOY 包住：備援/地面站編譯為空。
 */
#include "board_config.h"
#if FEATURE_UPLINK_DEPLOY

#include "uplink_cmd.h"
#include "uplink_proto.h"
#include "uplink_text_proto.h"   /* 文字上行幀（0x55/0xBB）：帶參數指令 → Parse_Serial_Command */
#include "ack_proto.h"           /* ACK 狀態碼 ACK_OK/UNKNOWN/... */
#include "lora_e22.h"            /* LoRaE22_SetRxRearmCallback：設定模式後重掛 UART3 接收 */
#include "main.h"
#include "ekf.h"
#include <stdio.h>
#include <string.h>

/* ARM 武裝狀態：經 arm 命令啟動後持續保持，直至接收 disarm 命令才解除。 */

extern UART_HandleTypeDef huart3;   /* E22 433 透傳（main.c 定義） */

/* ---- USART3 位元組環形緩衝：ISR 推入、遙測任務取出 ---- */
#define U3R_SZ 256U
static volatile uint8_t  s_ring[U3R_SZ];
static volatile uint16_t s_head = 0, s_tail = 0;
/* 循環 DMA 目的緩衝（改自舊的 ReceiveToIdle_IT 暫存）。
 * 舊版每次事件都得在 callback 內重新掛載，重掛期間 RX 未武裝會漏位元組；
 * circular DMA 全程武裝、無空檔（比照 link_hw.c 的 Link_OnRxEvent 與 gps.c）。 */
#define U3R_DMA_SZ 64U
static uint8_t           s_rxbuf[U3R_DMA_SZ];
static volatile uint16_t s_dma_old_pos = 0;

static UplinkRx_t     s_rx;       /* 二進制幀（ARM/DEPLOY/BENCH） */
static UplinkTextRx_t s_trx;      /* 文字幀（帶參數指令） */

/* ---- 武裝 / 待辦開傘 ---- */
static uint8_t           s_armed = 0;
static uint32_t          s_arm_tick = 0;
static volatile uint8_t  s_pending_drogue = 0;
static volatile uint8_t  s_pending_main   = 0;

/* ---- 待辦文字命令（診斷任務取走 → Parse_Serial_Command） ---- */
static volatile uint8_t  s_pending_text = 0;
static char              s_text[UPLINK_TEXT_MAX + 1];
static uint8_t           s_text_seq = 0;

/* ---- 待辦 bench（已過 ARM 閘；pad-only 閘由執行端 main.c 再查） ---- */
static volatile uint8_t  s_pending_bench = 0;
static uint8_t           s_bench_seq = 0;

/* ---- 待辦 recovery（尋回指令：停止蜂鳴器與數據記錄） ---- */
static volatile uint8_t  s_pending_recovery = 0;
static uint8_t           s_recovery_seq = 0;

/* ---- 待送 ACK（執行端 SetAck 寫、遙測任務 TakePendingAck 取；best-effort 單槽） ---- */
static char              s_ack_text[ACK_TEXT_MAX + 1];
static uint8_t           s_ack_len = 0;
static uint8_t           s_ack_seq = 0;
static uint8_t           s_ack_status = 0;
static volatile uint8_t  s_ack_valid = 0;   /* 最後寫，確保緩衝先填妥 */

/* ---- 診斷 ----
 * s_raw_bytes：USART3 上收到的原始位元組總數（含雜訊、含下行 ACK 的回音）。
 * ★這個計數器是上行鏈路除錯的第一個問題「地面站到底有沒有打到我」的唯一答案：
 *   raw=0 → 射頻層面完全沒進來（頻道/空速不符、地面站根本沒發、或發射時本機正在發射）；
 *   raw↑ 但 ok=0 且 crc/resync↑ → 有訊號但幀壞掉（誤碼或被自己的發射截斷）；
 *   ok↑ → 命令真的收到了，問題在後面的 ARM/狀態閘。
 *   先前完全沒有這個資訊，地面站按 ARM 沒反應時無從判斷斷在哪一段。 */
static uint8_t  s_last_cmd = 0;
static volatile uint32_t s_raw_bytes = 0;

static void ring_push(uint8_t b)
{
    uint16_t nh = (uint16_t)((s_head + 1U) % U3R_SZ);
    if (nh != s_tail) { s_ring[s_head] = b; s_head = nh; }   /* 滿則丟 */
}
static int ring_pop(uint8_t *b)
{
    if (s_tail == s_head) return 0;
    *b = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) % U3R_SZ);
    return 1;
}

/* 重新掛載 USART3 循環 DMA 接收 + 重置解析器。兩個呼叫來源：
 *   1. UART 錯誤復原（ORE）——HAL 會中止 RX 且不會自己重掛。
 *   2. lora_e22.c 每次離開設定模式後（見 LoRaE22_SetRxRearmCallback）——設定模式
 *      要改 baud 而呼叫 HAL_UART_Init，那會把 RxState 打回 READY，進行中的
 *      ReceiveToIdle 就此失效；不重掛的話 E22 重試/改參數之後就再也收不到上行命令。
 * 兩種情況位元組流都斷過，故一併重置解析器，避免卡在半幀沿著錯誤邊界一直錯下去
 * （比照 link_hw.c 的 Link_OnError）。 */
static void uplink_u3_rx_rearm(void)
{
    HAL_UART_AbortReceive(&huart3);   /* 確保舊的 DMA 接收確實停掉，否則重掛會回 HAL_BUSY */
    s_dma_old_pos = 0;
    UplinkRx_Init(&s_rx);
    UplinkTextRx_Init(&s_trx);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rxbuf, sizeof(s_rxbuf));
}

void UplinkCmd_Init(void)
{
    UplinkRx_Init(&s_rx);
    UplinkTextRx_Init(&s_trx);
    s_armed = 0;
    s_pending_drogue = 0;
    s_pending_main = 0;
    s_pending_text = 0;
    s_pending_bench = 0;
    s_ack_valid = 0;

    /* 設定模式（LoRaE22_Init 重試、`e22 freq/pwr/air`）會呼叫 HAL_UART_Init，
     * 那會把 RxState 打回 READY 讓接收失效；註冊重掛回呼讓驅動自動救回來。
     * 必須在啟動接收之前註冊。 */
    LoRaE22_SetRxRearmCallback(uplink_u3_rx_rearm);
    uplink_u3_rx_rearm();
    printf("[UPLINK] 上行命令接收就緒（433 反向鏈路：二進制 ARM->DEPLOY/BENCH + 文字指令幀）\r\n");
}

/* 循環 DMA 接收事件：size = 自緩衝起點至目前 DMA 寫入位置的累計位元組數，
 * 以 s_dma_old_pos 環形差分取出新位元組（與 link_hw.c / gps.c 同法）。 */
void UplinkCmd_OnUart3RxEvent(uint16_t size)
{
    uint16_t old = s_dma_old_pos;
    if (size == old) return;

    if (size > old) {
        for (uint16_t i = old; i < size; i++) ring_push(s_rxbuf[i]);
        s_raw_bytes += (uint32_t)(size - old);
    } else {
        for (uint16_t i = old; i < U3R_DMA_SZ; i++) ring_push(s_rxbuf[i]);
        for (uint16_t i = 0; i < size; i++)         ring_push(s_rxbuf[i]);
        s_raw_bytes += (uint32_t)(U3R_DMA_SZ - old) + size;
    }
    s_dma_old_pos = (size >= U3R_DMA_SZ) ? 0U : size;
}

/* USART3(433 上行 RX) 錯誤復原：清 ORE/雜訊旗標並重啟循環 DMA 接收 + 重置解析器。
 * 由 main.c 的 HAL_UART_ErrorCallback 在 USART3 出錯時轉接。不做則一次溢位就讓
 * 上行接收永久停擺（之後收不到地面站的 ARM/DEPLOY 命令）。
 * 解析器一併重置：溢位代表位元組流缺了一段，UplinkRx 可能卡在半幀，
 * 不重置會沿著錯誤邊界一直錯下去（比照 link_hw.c 的 Link_OnError）。 */
void UplinkCmd_OnUart3Error(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    (void)huart3.Instance->SR;
    (void)huart3.Instance->DR;

    /* ★DMA 接收下 ORE 屬「非阻斷錯誤」：HAL 只發錯誤回呼，接收其實還活著。
     * 這種情況不可重掛（會回 HAL_BUSY），也不可把 s_dma_old_pos 歸零——DMA 寫入
     * 位置沒有跟著回到 0，歸零會讓下一次事件重讀一整段舊資料。
     * 只有 HAL 真的把接收停掉時才需要、也才能重掛。
     * ★但「RxState != BUSY_RX」不等於「現在可以重掛」——lora_e22.c 的設定模式一開始
     *   就用 HAL_UART_AbortReceive() 把 RxState 打回 READY，一路到離開設定模式前都
     *   是這個狀態；若這個 ISR（不受任何任務優先權節制）在這段窗口內被 M1 切換/baud
     *   改變觸發的雜訊 ORE 打進來，重掛的 DMA 會硬生生把 lora_e22.c 正在阻塞輪詢等待
     *   的模組回應位元組搶走（DMA 永遠比軟體輪詢快），造成 `e22 freq/pwr/air` 每次
     *   都 st=3(HAL_TIMEOUT)、回讀全 0。故先查 LoRaE22_IsInConfigMode()：正在設定
     *   模式就不搶，離開時 lora_e22.c 自己會呼叫這支重掛。 */
    if (huart3.RxState != HAL_UART_STATE_BUSY_RX) {
        if (!LoRaE22_IsInConfigMode()) {
            uplink_u3_rx_rearm();
        }
    } else {
        UplinkRx_Init(&s_rx);        /* 位元組流缺了一段：解析器仍需回到重找 sync */
        UplinkTextRx_Init(&s_trx);
    }
}

void UplinkCmd_Poll(uint32_t now_ms)
{
    uint8_t b, cmd = 0, arg = 0, seq = 0;
    char    tseq_text[UPLINK_TEXT_MAX + 1];
    uint8_t tlen = 0, tseq = 0;
    while (ring_pop(&b)) {
        /* 同一位元組流並排餵兩個解析器：各自忽略不符自身 sync 的位元組，互不干擾。 */

        /* --- 文字命令幀（0x55/0xBB）：帶參數指令 → 交診斷任務餵 Parse_Serial_Command --- */
        if (UplinkTextRx_Feed(&s_trx, b, tseq_text, &tlen, &tseq)) {
            if (!s_pending_text) {            /* 前一筆尚未被取走則丟棄本筆（極少見；避免覆蓋） */
                memcpy(s_text, tseq_text, (size_t)tlen + 1U);
                s_text_seq = tseq;
                s_pending_text = 1;           /* 最後設，確保 s_text 已填妥 */
                printf("[UPLINK] 文字命令 seq=%u: \"%s\"\r\n", (unsigned)tseq, s_text);
            }
            continue;
        }

        /* --- 二進制幀（0x55/0xAA）：ARM/DISARM/DEPLOY/BENCH --- */
        if (!UplinkRx_Feed(&s_rx, b, &cmd, &arg, &seq)) continue;
        s_last_cmd = cmd;
        switch (cmd) {
            case UPLINK_CMD_PING:
                printf("[UPLINK] PING seq=%u（armed=%u）\r\n", (unsigned)seq, (unsigned)s_armed);
                UplinkCmd_SetAck(seq, ACK_OK, "ping");
                break;
            case UPLINK_CMD_ARM:
                {
                    extern volatile uint8_t g_flash_erase_in_progress;
                    if (g_flash_erase_in_progress) {
                        /* flash erase 阻塞式跑在另一個任務，此時擦除區塊可能尚未淨空，
                         * 不可讓 ARM 生效並開始記錄（見 main.c 的 g_flash_erase_in_progress）。 */
                        printf("[UPLINK] ARM 忽略：flash erase 進行中 seq=%u\r\n", (unsigned)seq);
                        UplinkCmd_SetAck(seq, ACK_REJECTED, "arm");
                        break;
                    }
                }
                s_armed = 1; s_arm_tick = now_ms;
                {
                    extern FlightState_t current_fsm_state;
                    extern FSM_Context_t g_fsm_ctx;
                    extern void ExtremaTrack_Reset(void);
                    if (current_fsm_state == STATE_INIT || current_fsm_state == STATE_PAD) {
                        FSM_SetState(&g_fsm_ctx, STATE_PAD_ARMED);
                    }
                    /* 下鏈滾動極值歸零：ARM 即本次飛行的起算點，把先前地面測試/搬動
                     * 累積的最大高度/速度/G 清掉（見 main.c ExtremaTrack_Reset）。 */
                    ExtremaTrack_Reset();
                }
                printf("[UPLINK] *** ARMED -> STATE_PAD_ARMED ***\r\n");
                UplinkCmd_SetAck(seq, ACK_OK, "arm");
                break;
            case UPLINK_CMD_DISARM:
                s_armed = 0;
                {
                    extern FlightState_t current_fsm_state;
                    extern FSM_Context_t g_fsm_ctx;
                    if (current_fsm_state == STATE_PAD_ARMED || current_fsm_state == STATE_INIT) {
                        FSM_SetState(&g_fsm_ctx, STATE_PAD);
                    }
                }
                printf("[UPLINK] DISARMED -> STATE_PAD\r\n");
                UplinkCmd_SetAck(seq, ACK_OK, "disarm");
                break;
            case UPLINK_CMD_DEPLOY_DROGUE:
            case UPLINK_CMD_DEPLOY_MAIN:
            case UPLINK_CMD_DEPLOY_BOTH:
                {
                    extern FlightState_t current_fsm_state;
                    uint8_t in_flight = (current_fsm_state >= STATE_BOOST && current_fsm_state <= STATE_MAIN_DEPLOY) ? 1U : 0U;
                    if (!in_flight && !s_armed) {
                        printf("[UPLINK] 開傘命令忽略：未武裝且在地面（先送 ARM）\r\n");
                        UplinkCmd_SetAck(seq, ACK_UNARMED, "deploy");
                        break;
                    }
                    if (cmd != UPLINK_CMD_DEPLOY_MAIN)   s_pending_drogue = 1;
                    if (cmd != UPLINK_CMD_DEPLOY_DROGUE) s_pending_main   = 1;
                    printf("[UPLINK] *** 手動開傘 *** drogue=%u main=%u (in_flight=%u)\r\n",
                           (unsigned)(cmd != UPLINK_CMD_DEPLOY_MAIN),
                           (unsigned)(cmd != UPLINK_CMD_DEPLOY_DROGUE),
                           (unsigned)in_flight);
                    UplinkCmd_SetAck(seq, ACK_OK, "deploy");
                }
                break;
            case UPLINK_CMD_BENCH:
                /* 桌面測試（跑一次 pyro/servo 自測後回歸）：ARM 閘在此，pad-only 閘由執行端
                 * main.c 再查（那裡有 current_fsm_state）。未 ARM → 立即回 UNARMED、不排程。 */
                if (!s_armed) {
                    printf("[UPLINK] BENCH 忽略：未武裝（先送 ARM）\r\n");
                    UplinkCmd_SetAck(seq, ACK_UNARMED, "bench");
                    break;
                }
                if (!s_pending_bench) {
                    s_bench_seq = seq;
                    s_pending_bench = 1;
                    printf("[UPLINK] *** BENCH 已排程 ***（過 ARM 閘；執行端再查未起飛閘）\r\n");
                }
                break;
            case UPLINK_CMD_RECALIBRATE:
                if (!s_armed) {
                    printf("[UPLINK] RECALIBRATE 忽略：未武裝（先送 ARM）\r\n");
                    UplinkCmd_SetAck(seq, ACK_UNARMED, "recalib");
                    break;
                }
                {
                    extern FlightState_t current_fsm_state;
                    if (current_fsm_state <= STATE_PAD || current_fsm_state == STATE_PAD_ARMED) {
                        EKF_ResetCalibration();
                        printf("[UPLINK] *** EKF 重新校準已觸發 ***\r\n");
                        UplinkCmd_SetAck(seq, ACK_OK, "recalib");
                    } else {
                        printf("[UPLINK] RECALIBRATE 拒絕：非地面狀態（state=%u）\r\n", (unsigned)current_fsm_state);
                        UplinkCmd_SetAck(seq, ACK_REJECTED, "recalib");
                    }
                }
                break;
            case UPLINK_CMD_RECOVERY:
                {
                    extern FlightState_t current_fsm_state;
                    /* 僅允許已著陸 (STATE_LANDED) 才執行：避免發射前在 PAD/PAD_ARMED
                     * 誤觸就停掉蜂鳴器與 SD/Flash 記錄（先前只擋飛行中 BOOST..MAIN_DEPLOY，
                     * PAD/PAD_ARMED 會被放行，非預期）。 */
                    if (current_fsm_state != STATE_LANDED) {
                        printf("[UPLINK] 尋回指令拒絕：僅限已著陸 (STATE_LANDED) 才可執行（state=%u）\r\n", (unsigned)current_fsm_state);
                        UplinkCmd_SetAck(seq, ACK_REJECTED, "recovery");
                        break;
                    }
                    if (!s_pending_recovery) {
                        s_recovery_seq = seq;
                        s_pending_recovery = 1;
                        printf("[UPLINK] *** 尋回指令已排程 ***（停止蜂鳴器與數據記錄）\r\n");
                        UplinkCmd_SetAck(seq, ACK_OK, "recovery");
                    }
                }
                break;
            default:
                printf("[UPLINK] 未知命令 0x%02X\r\n", (unsigned)cmd);
                UplinkCmd_SetAck(seq, ACK_UNKNOWN, NULL);
                break;
        }
    }
}

uint8_t UplinkCmd_TakeDeploy(uint8_t *want_drogue, uint8_t *want_main)
{
    uint8_t any = 0;
    if (s_pending_drogue) { if (want_drogue) *want_drogue = 1; s_pending_drogue = 0; any = 1; }
    else if (want_drogue) *want_drogue = 0;
    if (s_pending_main)   { if (want_main)   *want_main   = 1; s_pending_main   = 0; any = 1; }
    else if (want_main)   *want_main = 0;
    return any;
}

void UplinkCmd_ForceDeploy(uint8_t want_drogue, uint8_t want_main)
{
    if (want_drogue) s_pending_drogue = 1;
    if (want_main)   s_pending_main   = 1;
}

uint8_t UplinkCmd_TakeTextCmd(char *out, uint16_t sz, uint8_t *seq)
{
    if (!s_pending_text) return 0;
    if (out && sz > 0U) {
        size_t n = strlen(s_text);
        if (n > (size_t)(sz - 1U)) n = (size_t)(sz - 1U);
        memcpy(out, s_text, n);
        out[n] = '\0';
    }
    if (seq) *seq = s_text_seq;
    s_pending_text = 0;   /* 消費後清，讓下一筆可入 */
    return 1;
}

uint8_t UplinkCmd_TakeBench(uint8_t *seq)
{
    if (!s_pending_bench) return 0;
    if (seq) *seq = s_bench_seq;
    s_pending_bench = 0;
    return 1;
}

uint8_t UplinkCmd_TakeRecovery(uint8_t *seq)
{
    if (!s_pending_recovery) return 0;
    if (seq) *seq = s_recovery_seq;
    s_pending_recovery = 0;
    return 1;
}

void UplinkCmd_SetAck(uint8_t seq, uint8_t status, const char *text)
{
    uint8_t len = 0;
    if (text) {
        size_t n = strlen(text);
        if (n > ACK_TEXT_MAX) n = ACK_TEXT_MAX;
        memcpy(s_ack_text, text, n);
        s_ack_text[n] = '\0';
        len = (uint8_t)n;
    } else {
        s_ack_text[0] = '\0';
    }
    s_ack_len    = len;
    s_ack_seq    = seq;
    s_ack_status = status;
    s_ack_valid  = 1;     /* 最後設，確保上面欄位已填妥 */
}

uint8_t UplinkCmd_TakePendingAck(uint8_t *seq, uint8_t *status, char *out_text, uint8_t *out_len)
{
    if (!s_ack_valid) return 0;
    if (seq)    *seq    = s_ack_seq;
    if (status) *status = s_ack_status;
    if (out_text) { memcpy(out_text, s_ack_text, (size_t)s_ack_len + 1U); }
    if (out_len)  *out_len = s_ack_len;
    s_ack_valid = 0;
    return 1;
}

uint8_t UplinkCmd_IsArmed(void) { return s_armed; }

void UplinkCmd_SetArmedState(uint8_t armed)
{
    s_armed = (armed != 0U) ? 1U : 0U;
    s_arm_tick = HAL_GetTick();
}

void UplinkCmd_GetStats(UplinkCmdStats_t *out)
{
    if (!out) return;
    out->raw_bytes    = s_raw_bytes;
    out->bin_ok       = s_rx.ok;
    out->bin_crc_err  = s_rx.crc_err;
    out->bin_resync   = s_rx.resync;
    out->text_ok      = s_trx.ok;
    out->text_crc_err = s_trx.crc_err;
    out->last_cmd     = s_last_cmd;
}

#endif /* FEATURE_UPLINK_DEPLOY */
