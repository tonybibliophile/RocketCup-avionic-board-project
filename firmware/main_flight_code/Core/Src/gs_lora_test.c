/*
 * gs_lora_test.c — LoRa 通訊測試模組（地面站 UART2 命令介面 + 雙鏈路統計）
 * ===========================================================================
 * 整檔以 #if IS_GROUND 包住：航電板編譯為空。
 * 純換算 / 統計委派 lora_calc.h（與 host 測試 tests/test_lora_calc.c 共用）。
 */
#include "board_config.h"
#if IS_GROUND

#include "gs_lora_test.h"
#include "gs_log.h"        /* GS_LINK_433/920, GS_RSSI_NA, GS_SNR_NA */
#include "telemetry.h"     /* TELEM_PACKET_SIZE（空中時間估算用） */
#include "lora_calc.h"     /* 純換算 + lora_stats_t */
#include "uplink_proto.h"  /* 上行手動開傘命令框架（與火箭端共用） */
#include "uplink_text_proto.h" /* 上行文字命令框架（tx 中繼；與火箭端共用） */
#include "lora_e22.h"
#include "lora_e80.h"
#include "w25qxx.h"        /* FlashRing_EraseAll()：`flash erase` 命令重用飛控角色既有的擦除實作 */
#include "spi3_bus.h"      /* SPI3_Bus_Lock/Unlock：`flash dump`/`flash export` 讀取期間鎖 SPI3 */
#include "ground_station.h" /* GroundStation_FlashResetAfterErase() */
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef  hspi3;    /* E80 重新初始化用 */
extern IWDG_HandleTypeDef hiwdg;    /* 上行 burst 期間餵狗 */

/* ===== 上行命令 burst =========================================================
 * ★2026-07-30 修正：舊版是「固定重複呼叫 LoRaE22_Send 24 次、每次間隔 120ms」，
 *   而且完全不看回傳值。但 LoRaE22_Send 在 AUX(BUSY 腳) 為低時是「直接回 HAL_BUSY、
 *   一個位元組都不送」——本機 E22 只要正在收火箭的下行封包（116B@2.4k 空中約 390ms）
 *   就是這個狀態。結果是整個 burst 可能一次都沒真的發射，console 卻照樣印「送出完畢」，
 *   操作者完全看不出來。實測 log（20260727_004510）：連按 10 次 ARM（seq 6~15）
 *   全部印「送出完畢」，火箭端 fsm 一直是 1(PAD)、[ACK] 一筆都沒有。
 * 改法：
 *   ① 忙線不是放棄而是短間隔重試，AUX 一放開就立刻送 —— 火箭的上行接收窗
 *      （main.c UPLINK_LISTEN_HOLD_MS）期間本機也剛好收不到東西、AUX 會拉高，
 *      這個「忙就等、閒就送」的行為等於自動對準對方的窗口。
 *   ② burst 時間必須 > 火箭的接收窗週期（main.c UPLINK_LISTEN_PERIOD_MS=3000ms），
 *      否則整段 burst 可能塞在兩個窗口之間。故不做「送夠 N 次就提早結束」。
 *   ③ 結束時印 ok/busy/err 實際次數，讓「到底有沒有發出去」變成看得見的事實。 */
#define UPLINK_TX_WINDOW_MS    4500U  /* burst 總時長：須 > 火箭 UPLINK_LISTEN_PERIOD_MS */
#define UPLINK_TX_GAP_MS        150U  /* 成功送出一筆後的間隔（別把鏈路完全佔滿） */
#define UPLINK_TX_BUSY_POLL_MS   10U  /* AUX 忙線時的重試輪詢間隔 */
static uint8_t s_uplink_seq = 0;

/* ============================================================
 *  UART2 環形接收緩衝
 * ============================================================ */
#define U2_RING_SZ  512U
static volatile uint8_t  s_u2_ring[U2_RING_SZ];
static volatile uint16_t s_u2_head = 0, s_u2_tail = 0;
static uint8_t           s_u2_rxbuf[64];   /* ReceiveToIdle 暫存 */

static void u2_push(uint8_t b)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t nh = (uint16_t)((s_u2_head + 1U) % U2_RING_SZ);
    if (nh != s_u2_tail) { s_u2_ring[s_u2_head] = b; s_u2_head = nh; }
    __set_PRIMASK(primask);
}
static int u2_pop(uint8_t *b)
{
    if (s_u2_tail == s_u2_head) return 0;
    *b = s_u2_ring[s_u2_tail];
    s_u2_tail = (uint16_t)((s_u2_tail + 1U) % U2_RING_SZ);
    return 1;
}

/* ============================================================
 *  狀態：統計 + 目前 RF 參數 + 自動列印
 * ============================================================ */
static lora_stats_t s_stat[2];   /* [0]=E22 433, [1]=E80 920 */

/* E80 影子參數：GsLoraTest_Init 以 LoRaE80_GetParams 從驅動同步實際套用值
 * （避免此處預設與 lora_e80.c 板級 #define 漂移，開機 `e80 show` 印錯值）。 */
static uint32_t s_e80_freq_hz  = 920000000UL;
static uint8_t  s_e80_sf       = 0x09U;  /* SF9 */
static uint8_t  s_e80_bw       = 0x05U;  /* 250kHz */
static uint8_t  s_e80_cr       = 0x01U;  /* 4/5 */
static int8_t   s_e80_pwr_dbm  = 22;
static uint16_t s_e80_preamble = 8U;
static uint32_t s_e22_freq_mhz = 432U;

static uint8_t  s_auto_stats   = 0;       /* 自動列印統計開關 */
static uint32_t s_auto_period_ms = 5000U;
static uint32_t s_auto_last_ms = 0;

static uint8_t  s_e22_raw_dump = 0;       /* 433 原始位元組 hex dump 開關（`e22 dump on`） */

uint8_t GsLoraTest_RawDumpEnabled(void) { return s_e22_raw_dump; }

static void stats_reset_all(void)
{
    lora_stats_reset(&s_stat[0]);
    lora_stats_reset(&s_stat[1]);
}

/* ============================================================
 *  列印
 * ============================================================ */
static void print_help(void)
{
    printf("\r\n=== LoRa 通訊測試命令（UART2 460800baud, 換行結尾）===\r\n"
           "  help              顯示此說明\r\n"
           "  role              回報角色/版本（GUI 自動偵測用）\r\n"
           "  ver               顯示 E80(LR1121) 版本/初始化診斷\r\n"
           "  stats             顯示雙鏈路統計\r\n"
           "  stats reset       清除統計\r\n"
           "  stats auto <sec>  每 N 秒自動列印統計（0=關閉）\r\n"
           "  e22 freq <mhz>    設 E22 頻率 (410-493)\r\n"
           "  e22 pwr  <0-3>    設 E22 功率 0=30 1=27 2=24 3=21dBm（3V3 供電建議 3）\r\n"
           "  e22 air  <0-7>    設 E22 空速 0=0.3k 1=1.2k 2=2.4k ... 7=62.5k（兩端須一致）\r\n"
           "  e22 show          顯示 E22 目前頻率\r\n"
           "  e22 dump on/off   433 原始位元組 hex dump（診斷 RSSI byte 位置/數量，預設關閉）\r\n"
           "  e80 freq <hz>     設 E80 中心頻率 Hz (e.g. 915000000)\r\n"
           "  e80 sf   <7-12>   設 E80 展頻因子\r\n"
           "  e80 bw   <idx>    設 E80 頻寬 (3=62.5k 4=125k 5=250k 6=500k)\r\n"
           "  e80 cr   <1-4>    設 E80 編碼率 (1=4/5 2=4/6 3=4/7 4=4/8)\r\n"
           "  e80 pwr  <dbm>    設 E80 發射功率 (-9~22 dBm)\r\n"
           "  e80 pre  <n>      設 E80 前導碼長度 (6~65535)\r\n"
           "  e80 show          顯示 E80 RF 參數 + 空中時間估算\r\n"
           "  e80 init          重新初始化 E80 並進入接收\r\n"
           "  e80 rxstart       重新進入連續接收\r\n"
           "  e80 airtime <len> 估算指定 payload 長度的空中時間\r\n"
           "  flash erase       清空本機接收紀錄（約 30-60 秒，期間停止收包；兩種地面站\r\n"
           "                    binary 皆可用，與下方發射鎖無關）\r\n"
           "  flash dump        人眼 hex dump 整顆晶片前段區塊（沿用飛控角色共用實作）\r\n"
           "  flash export      結構化匯出：Sector0 hex + 本機收到的下行遙測紀錄 CSV\r\n"
           "                    （GsLog 格式，與 SD 卡 CSV 同源，供 GUI/分析工具解析）\r\n"
#if GS_LORA_TX_ENABLE
           "  --- 遠端指令中繼到主航電（433 反向）---\r\n"
           "  tx <指令>         把整串原文送給主航電執行（複用其命令台：role/help/\r\n"
           "                    e22.../e80.../CMD_MAG_CAL:.../CMD_MAG_YAW_LOCK:...），\r\n"
           "                    火箭回下行 [ACK] 確認。例：tx role / tx e80 sf 9\r\n"
           "  --- 上行手動開傘 / 桌面測試（兩段式安全，須先 arm）---\r\n"
           "  ping              連線測試（火箭印出，不動作）\r\n"
           "  arm               武裝（火箭 30s 內允許開傘/bench）\r\n"
           "  disarm            解除武裝\r\n"
           "  deploy drogue     手動開副傘（須先 arm）\r\n"
           "  deploy main       手動開主傘（須先 arm）\r\n"
           "  deploy both       手動同時開副傘+主傘（須先 arm）\r\n"
           "  bench             桌面測試：跑一次 pyro/servo 自測後回歸（須先 arm＋僅限未起飛）\r\n"
           "  recalib           重新校準 EKF（須先 arm＋僅限未起飛）\r\n"
           "  recovery          尋回指令（落地後停止蜂鳴器與數據記錄）\r\n"
#else
           "  --- 遠端指令中繼 / 上行手動開傘（本機為 RX-only 版本，以下指令一律被拒） ---\r\n"
           "  tx / ping / arm / disarm / deploy / bench / recalib / recovery\r\n"
           "                    本機 make flash-ground 為純接收版，不接受發射；\r\n"
           "                    如需發射請改燒 make flash-ground-tx\r\n"
#endif
           "=====================================================\r\n");
}

static void print_one_stat(int i, const char *name)
{
    lora_stats_t *s = &s_stat[i];
    printf("\r\n[STATS] --- %s ---\r\n", name);
    printf("[STATS] pkt_ok=%lu  crc_err=%lu\r\n",
           (unsigned long)s->pkt_ok, (unsigned long)s->crc_err);

    uint32_t rate_x10 = lora_stats_rate_x10(s);
    if (rate_x10 > 0) {
        uint32_t el = (s->last_ms > s->first_ms) ? (s->last_ms - s->first_ms) : 0;
        printf("[STATS] rate=%lu.%lu pkt/s  elapsed=%lus\r\n",
               (unsigned long)(rate_x10 / 10), (unsigned long)(rate_x10 % 10),
               (unsigned long)(el / 1000));
    } else {
        printf("[STATS] rate=-- pkt/s\r\n");
    }

    /* ★433 也可能有 RSSI：E22 開啟 REG3 bit7 後每包附加一個 RSSI 位元組
     * （見 lora_e22.c 的 E22_RSSI_BYTE_EN），舊版寫死「透傳模式無此資訊」已過時。
     * E22 沒有 SNR，故只有 920 印 SNR。 */
    if (s->rssi_cnt > 0) {
        printf("[STATS] RSSI: last=%d min=%d max=%d avg=%d dBm  (n=%lu)\r\n",
               (int)s->rssi_last, (int)s->rssi_min, (int)s->rssi_max,
               (int)lora_stats_rssi_avg(s), (unsigned long)s->rssi_cnt);
    } else {
        printf("[STATS] RSSI: N/A\r\n");
    }
    if (i == GS_LINK_920) {
        printf("[STATS] SNR:  last=%d min=%d max=%d avg=%d (x0.25dB)\r\n",
               (int)s->snr_last, (int)s->snr_min, (int)s->snr_max,
               (int)lora_stats_snr_avg(s));
    }
}

static void print_stats(void)
{
    print_one_stat(GS_LINK_433, "E22-433");
    print_one_stat(GS_LINK_920, "E80-920");
    printf("\r\n");
}

static void print_airtime(uint8_t payload_len)
{
    uint32_t toa = lora_time_on_air_us(s_e80_sf, s_e80_bw, s_e80_cr,
                                       s_e80_preamble, payload_len);
    if (toa == 0) { printf("[E80] airtime: 參數非法\r\n"); return; }
    /* 等效 bitrate = payload*8 bits / ToA ；以 bps 顯示 */
    uint32_t bps = (uint32_t)payload_len * 8u * 1000000u / toa;
    printf("[E80] payload=%u B  airtime=%lu.%03lu ms  ~%lu bps\r\n",
           (unsigned)payload_len,
           (unsigned long)(toa / 1000), (unsigned long)(toa % 1000),
           (unsigned long)bps);
}

static void print_e80_params(void)
{
    uint32_t bw_khz = lora_bw_to_khz(s_e80_bw);
    printf("[E80] freq=%lu Hz  SF%u  BW%lu kHz (idx=%u)  CR 4/%u  pwr=%d dBm  pre=%u  ldro=%u\r\n",
           (unsigned long)s_e80_freq_hz, (unsigned)s_e80_sf,
           (unsigned long)bw_khz, (unsigned)s_e80_bw,
           (unsigned)(s_e80_cr + 4U), (int)s_e80_pwr_dbm,
           (unsigned)s_e80_preamble, (unsigned)lora_ldro_required(s_e80_sf, s_e80_bw));
    print_airtime((uint8_t)TELEM_PACKET_SIZE);
}

static void print_e22_params(void)
{
    LoRaE22_PrintConfig();
}

static void print_version(void)
{
    int rd_st = -1; uint8_t busy = 0xFF, hw = 0, type = 0, gs = 0xFF;
    LoRaE80_GetInitDiag(&rd_st, &busy, &hw, &type, &gs);
    printf("[E80] LR1121 GetVersion HW=0x%02X Type=0x%02X (0x03=LR1121)  Stat1=0x%02X  init_rd=%d busy=%u\r\n",
           (unsigned)hw, (unsigned)type, (unsigned)gs, rd_st, (unsigned)busy);
}

static void apply_e80_reconfig(const char *subcmd)
{
    print_e80_params();
    HAL_StatusTypeDef st = LoRaE80_Reconfig(s_e80_freq_hz, s_e80_sf, s_e80_bw,
                                             s_e80_cr, s_e80_pwr_dbm, s_e80_preamble);
    if (st == HAL_OK) {
        printf("[E80] reconfig OK -> RX restarted\r\n");
        printf("[ACK] status:OK cmd:\"e80 %s\"\r\n", subcmd ? subcmd : "config");
    } else {
        printf("[E80] reconfig FAIL (st=%d)\r\n", (int)st);
        printf("[ACK] status:REJECTED cmd:\"e80 %s\"\r\n", subcmd ? subcmd : "config");
    }
}

/* ============================================================
 *  上行發射權限檢查（GS_LORA_TX_ENABLE，見 board_config.h）
 *  刻意不由 GUI 攔截：RX-only binary 一律照樣解析／回應，只是在真正呼叫
 *  LoRaE22_Send 前擋下並印出拒絕訊息，讓地面站操作者從 console 就能看到
 *  「這台是收機」，不必依賴 GUI 是否正確禁用按鈕。
 * ============================================================ */
static uint8_t gs_tx_allowed(const char *label)
{
#if GS_LORA_TX_ENABLE
    (void)label;
    return 1U;
#else
    printf("[UPLINK] REJECTED: 本機為 RX-only 地面站（make flash-ground），不接受發射 '%s'。"
           "如需發射請改燒 make flash-ground-tx 版本。\r\n", label);
    return 0U;
#endif
}

/* ============================================================
 *  上行命令發送（手動開傘等，經 E22 433 反向打給火箭）
 * ============================================================ */
/* 在 UPLINK_TX_WINDOW_MS 內反覆嘗試送同一筆幀：AUX 忙就短間隔重試、閒就立刻送。
 * 回傳實際成功推入 E22 的次數（0 = 整段期間本機 E22 一次都沒空，命令根本沒上空中）。 */
static uint32_t uplink_burst(const uint8_t *frame, uint8_t len, const char *label, uint8_t seq)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t ok = 0, busy = 0, err = 0;

    while ((HAL_GetTick() - t0) < UPLINK_TX_WINDOW_MS) {
        HAL_StatusTypeDef st = LoRaE22_Send(frame, len);
        HAL_IWDG_Refresh(&hiwdg);
        if (st == HAL_OK) {
            ok++;
            osDelay(UPLINK_TX_GAP_MS);
        } else if (st == HAL_BUSY) {
            busy++;                       /* AUX 低：本機 E22 正在收/發，等一下再試 */
            osDelay(UPLINK_TX_BUSY_POLL_MS);
        } else {
            err++;
            osDelay(UPLINK_TX_GAP_MS);
        }
    }

    printf("[UPLINK] %s seq=%u burst 結束：實際送出 ok=%lu busy=%lu err=%lu（歷時 %lums）\r\n",
           label, (unsigned)seq, (unsigned long)ok, (unsigned long)busy, (unsigned long)err,
           (unsigned long)(HAL_GetTick() - t0));
    if (ok == 0U) {
        printf("[UPLINK] ⚠ ok=0：本機 E22 全程忙線（AUX 一直為低），這筆命令沒有上空中。"
               "檢查 433 是否被下行流量佔滿，或模組 AUX 接線/供電。\r\n");
    }
    return ok;
}

static void uplink_send(uint8_t cmd, uint8_t arg, const char *label)
{
    if (!gs_tx_allowed(label)) return;
    uint8_t f[UPLINK_FRAME_SIZE];
    uint8_t seq = s_uplink_seq++;
    UplinkProto_Build(f, cmd, arg, seq);
    printf("[UPLINK] 送 %s (cmd=0x%02X seq=%u) burst %lums…\r\n",
           label, (unsigned)cmd, (unsigned)seq, (unsigned long)UPLINK_TX_WINDOW_MS);
    if (uplink_burst(f, UPLINK_FRAME_SIZE, label, seq) > 0U) {
        printf("[UPLINK] %s 已上空中（等火箭下行 [ACK]；開傘另看 DROGUE_FIRED/MAIN_DEPLOYED 旗標）\r\n",
               label);
    }
}

/* 中繼文字命令到主航電（tx <原文>）：以 uplink_text_proto 幀 burst 送。text 為「原始大小寫」
 * ——火箭 Parse_Serial_Command 的 CMD_MAG_CAL: 等以大寫比對，故不可先轉小寫。 */
static void uplink_send_text(const char *text)
{
    uint8_t len = (uint8_t)strlen(text);
    if (len == 0U) { printf("[UPLINK] tx 需要指令內容，例：tx role / tx e80 sf 9\r\n"); return; }
    if (!gs_tx_allowed("tx")) return;
    if (len > UPLINK_TEXT_MAX) {
        printf("[UPLINK] 指令過長 %u > %u，已截斷\r\n", (unsigned)len, (unsigned)UPLINK_TEXT_MAX);
        len = UPLINK_TEXT_MAX;
    }
    uint8_t f[UPLINK_TEXT_FRAME_MAX];
    uint8_t seq = s_uplink_seq++;
    uint8_t n = UplinkTextProto_Build(f, text, len, seq);
    printf("[UPLINK] 送文字命令 seq=%u \"%.*s\" burst %lums…\r\n",
           (unsigned)seq, (int)len, text, (unsigned long)UPLINK_TX_WINDOW_MS);
    if (uplink_burst(f, n, "TEXT", seq) > 0U) {
        printf("[UPLINK] 文字命令已上空中（等火箭下行 [ACK]）\r\n");
    }
}

/* ============================================================
 *  命令解析
 * ============================================================ */
static void str_tolower(char *s) { for (; *s; s++) *s = (char)tolower((unsigned char)*s); }

static void dispatch_cmd(char *line)
{
    /* 中繼文字命令 tx <原文>：須在 str_tolower「之前」擷取原始大小寫（火箭 CMD_MAG_CAL:
     * 等以大寫比對）。只有 "tx" 關鍵字本身容忍大小寫。 */
    if ((line[0] == 't' || line[0] == 'T') && (line[1] == 'x' || line[1] == 'X') &&
        (line[2] == ' ' || line[2] == '\t')) {
        const char *p = line + 2;
        while (*p == ' ' || *p == '\t') p++;
        uplink_send_text(p);
        return;
    }

    str_tolower(line);

    char *tok[4] = {NULL, NULL, NULL, NULL};
    int   n = 0;
    char *p = line;
    while (*p && n < 4) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    if (n == 0) return;

    if (strcmp(tok[0], "help") == 0) {
        print_help();

    } else if (strcmp(tok[0], "role") == 0) {
        /* 與航電端 main.c Parse_Serial_Command 同格式（GUI 三角色自動偵測共用）；
         * tx=0/1 供 GUI 顯示 RX-only / TX-capable 徽章（僅顯示用，非攔截用）。 */
        printf("[ROLE_ID] role=GROUND fw=%s tx=%d\r\n", FIRMWARE_VERSION, (int)GS_LORA_TX_ENABLE);

    } else if (strcmp(tok[0], "ping") == 0) {
        uplink_send(UPLINK_CMD_PING, 0, "PING");

    } else if (strcmp(tok[0], "arm") == 0) {
        uplink_send(UPLINK_CMD_ARM, 0, "ARM");

    } else if (strcmp(tok[0], "disarm") == 0) {
        uplink_send(UPLINK_CMD_DISARM, 0, "DISARM");

    } else if (strcmp(tok[0], "bench") == 0) {
        /* 桌面測試：跑一次 pyro/servo 自測後回歸。火箭端須先 arm ＋僅限未起飛才會執行。 */
        uplink_send(UPLINK_CMD_BENCH, 0, "BENCH");

    } else if (strcmp(tok[0], "recalib") == 0) {
        /* 重新校準 EKF。火箭端須先 arm ＋僅限未起飛才會執行。 */
        uplink_send(UPLINK_CMD_RECALIBRATE, 0, "RECALIB");

    } else if (strcmp(tok[0], "recovery") == 0) {
        /* 尋回指令：落地後停止蜂鳴器與數據記錄（SD/Flash） */
        uplink_send(UPLINK_CMD_RECOVERY, 0, "RECOVERY");

    } else if (strcmp(tok[0], "deploy") == 0 && n >= 2) {
        if (strcmp(tok[1], "drogue") == 0) {
            uplink_send(UPLINK_CMD_DEPLOY_DROGUE, 0, "DEPLOY-DROGUE");
        } else if (strcmp(tok[1], "main") == 0) {
            uplink_send(UPLINK_CMD_DEPLOY_MAIN, 0, "DEPLOY-MAIN");
        } else if (strcmp(tok[1], "both") == 0) {
            uplink_send(UPLINK_CMD_DEPLOY_BOTH, 0, "DEPLOY-BOTH");
        } else {
            printf("[UPLINK] 用法：deploy drogue|main|both（須先 arm）\r\n");
        }

    } else if (strcmp(tok[0], "ver") == 0) {
        print_version();

    } else if (strcmp(tok[0], "stats") == 0) {
        if (n >= 2 && strcmp(tok[1], "reset") == 0) {
            stats_reset_all();
            printf("[STATS] reset OK\r\n");
        } else if (n >= 3 && strcmp(tok[1], "auto") == 0) {
            uint32_t sec = (uint32_t)strtoul(tok[2], NULL, 10);
            if (sec == 0) {
                s_auto_stats = 0;
                printf("[STATS] auto off\r\n");
            } else {
                s_auto_stats = 1;
                s_auto_period_ms = sec * 1000U;
                s_auto_last_ms = HAL_GetTick();
                printf("[STATS] auto every %lus\r\n", (unsigned long)sec);
            }
        } else {
            print_stats();
        }

    } else if (strcmp(tok[0], "show") == 0) {
        print_e22_params();
        print_e80_params();

    } else if (strcmp(tok[0], "e22") == 0 && n >= 2) {
        if (strcmp(tok[1], "show") == 0) {
            print_e22_params();
        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t mhz = (uint32_t)strtoul(tok[2], NULL, 10);
            uint8_t ch;
            if (!e22_mhz_to_ch(mhz, &ch)) {
                printf("[E22] freq 範圍 410-493 MHz\r\n");
                printf("[ACK] status:BADARG cmd:\"e22 freq\"\r\n");
                return;
            }
            s_e22_freq_mhz = mhz;
            HAL_StatusTypeDef st = LoRaE22_SetFreqMHz(mhz);
            if (st == HAL_OK) {
                printf("[E22] freq set %lu MHz (CH=%u) OK\r\n", (unsigned long)mhz, (unsigned)ch);
                printf("[ACK] status:OK cmd:\"e22 freq\"\r\n");
            } else {
                printf("[E22] freq set FAIL (st=%d)\r\n", (int)st);
                printf("[ACK] status:REJECTED cmd:\"e22 freq\"\r\n");
            }
        } else if (strcmp(tok[1], "pwr") == 0 && n >= 3) {
            uint32_t lvl = (uint32_t)strtoul(tok[2], NULL, 10);
            if (lvl > 3U) {
                printf("[E22] pwr 等級 0=30dBm 1=27dBm 2=24dBm 3=21dBm（3V3 供電建議 3）\r\n");
                printf("[ACK] status:BADARG cmd:\"e22 pwr\"\r\n");
                return;
            }
            HAL_StatusTypeDef st = LoRaE22_SetPowerLevel((uint8_t)lvl);
            if (st == HAL_OK) {
                printf("[E22] pwr set level %lu OK\r\n", (unsigned long)lvl);
                printf("[ACK] status:OK cmd:\"e22 pwr\"\r\n");
            } else {
                printf("[E22] pwr set FAIL (st=%d)\r\n", (int)st);
                printf("[ACK] status:REJECTED cmd:\"e22 pwr\"\r\n");
            }
        } else if (strcmp(tok[1], "air") == 0 && n >= 3) {
            uint32_t ar = (uint32_t)strtoul(tok[2], NULL, 10);
            if (ar > 7U) {
                printf("[E22] air 速率 0=0.3k 1=1.2k 2=2.4k 3=4.8k 4=9.6k 5=19.2k 6=38.4k 7=62.5k\r\n");
                printf("[ACK] status:BADARG cmd:\"e22 air\"\r\n");
                return;
            }
            HAL_StatusTypeDef st = LoRaE22_SetAirRate((uint8_t)ar);
            if (st == HAL_OK) {
                printf("[E22] air rate set %lu OK（兩端須一致）\r\n", (unsigned long)ar);
                printf("[ACK] status:OK cmd:\"e22 air\"\r\n");
            } else {
                printf("[E22] air rate set FAIL (st=%d)\r\n", (int)st);
                printf("[ACK] status:REJECTED cmd:\"e22 air\"\r\n");
            }
        } else if (strcmp(tok[1], "dump") == 0 && n >= 3) {
            if (strcmp(tok[2], "on") == 0) {
                s_e22_raw_dump = 1U;
                printf("[E22] raw dump ON —— 433 原始位元組將以 hex 印出（記得測完關掉，很洗版）\r\n");
            } else if (strcmp(tok[2], "off") == 0) {
                s_e22_raw_dump = 0U;
                printf("[E22] raw dump OFF\r\n");
            } else {
                printf("[E22] dump 子命令：on / off\r\n");
            }
        } else {
            printf("[E22] 未知子命令，輸入 help\r\n");
        }

    } else if (strcmp(tok[0], "e80") == 0 && n >= 2) {
        if (strcmp(tok[1], "show") == 0) {
            print_e80_params();

        } else if (strcmp(tok[1], "init") == 0) {
            HAL_StatusTypeDef st = LoRaE80_Init(&hspi3);
            if (st == HAL_OK) st = LoRaE80_StartRx();
            printf(st == HAL_OK ? "[E80] init + RX OK\r\n"
                                : "[E80] init FAIL (st=%d) — 檢查 SPI/RF開關/天線\r\n", (int)st);
            print_version();

        } else if (strcmp(tok[1], "rxstart") == 0) {
            HAL_StatusTypeDef st = LoRaE80_StartRx();
            printf(st == HAL_OK ? "[E80] RX restarted\r\n"
                                : "[E80] RX start FAIL (st=%d)\r\n", (int)st);

        } else if (strcmp(tok[1], "airtime") == 0 && n >= 3) {
            print_airtime((uint8_t)strtoul(tok[2], NULL, 10));

        } else if (strcmp(tok[1], "freq") == 0 && n >= 3) {
            uint32_t hz = (uint32_t)strtoul(tok[2], NULL, 10);
            if (hz < 862000000UL || hz > 928000000UL)
                printf("[E80] 注意：頻率建議 862-928 MHz，仍套用\r\n");
            s_e80_freq_hz = hz;
            apply_e80_reconfig("freq");

        } else if (strcmp(tok[1], "sf") == 0 && n >= 3) {
            uint8_t sf = (uint8_t)atoi(tok[2]);
            if (sf < 7 || sf > 12) {
                printf("[E80] SF 範圍 7-12\r\n");
                printf("[ACK] status:BADARG cmd:\"e80 sf\"\r\n");
                return;
            }
            s_e80_sf = sf; apply_e80_reconfig("sf");

        } else if (strcmp(tok[1], "bw") == 0 && n >= 3) {
            uint8_t bw = (uint8_t)strtoul(tok[2], NULL, 10);
            if (!lora_bw_valid(bw)) {
                printf("[E80] BW idx 合法值: 0 1 2 3 4(125k) 5(250k) 6(500k) 8 9 10\r\n");
                printf("[ACK] status:BADARG cmd:\"e80 bw\"\r\n");
                return;
            }
            s_e80_bw = bw; apply_e80_reconfig("bw");

        } else if (strcmp(tok[1], "cr") == 0 && n >= 3) {
            uint8_t cr = (uint8_t)atoi(tok[2]);
            if (cr < 1 || cr > 4) {
                printf("[E80] CR 範圍 1-4\r\n");
                printf("[ACK] status:BADARG cmd:\"e80 cr\"\r\n");
                return;
            }
            s_e80_cr = cr; apply_e80_reconfig("cr");

        } else if (strcmp(tok[1], "pwr") == 0 && n >= 3) {
            int pwr = atoi(tok[2]);
            if (pwr < -9 || pwr > 22) {
                printf("[E80] pwr 範圍 -9~22 dBm\r\n");
                printf("[ACK] status:BADARG cmd:\"e80 pwr\"\r\n");
                return;
            }
            s_e80_pwr_dbm = (int8_t)pwr; apply_e80_reconfig("pwr");

        } else if (strcmp(tok[1], "pre") == 0 && n >= 3) {
            long pre = atol(tok[2]);
            if (pre < 6 || pre > 65535) {
                printf("[E80] preamble 6~65535\r\n");
                printf("[ACK] status:BADARG cmd:\"e80 pre\"\r\n");
                return;
            }
            s_e80_preamble = (uint16_t)pre; apply_e80_reconfig("pre");

        } else {
            printf("[E80] 未知子命令，輸入 help\r\n");
        }

    } else if (strcmp(tok[0], "flash") == 0 && n >= 2 && strcmp(tok[1], "erase") == 0) {
        /* 地面站自己的接收紀錄（GsLogRecord_t ring，非火箭飛行紀錄）——只有整段清除，
         * 沒有 ring/all 之分（地面站不記校準/總結）。仍接受 `flash erase ring`/`flash erase all`
         * 當作同義詞：gui_monitor.py 的 Flash 面板固定送這兩種字串，讓它接上地面站板也不會
         * 撞到「未知命令」。
         * 直接重用 FlashRing_EraseAll()（w25qxx.c，已被 primary/backup 使用、已測）——地面站
         * 與飛控角色共用同一段 Flash 位址範圍（FLASH_RINGBUF_ADDR/END），只是寫入者/紀錄格式
         * 不同，block-erase 本身與格式無關，可以直接借用。
         * 擦除期間（~30-60s）GsLoraTest_Tick() 所在的接收主迴圈整段被佔用：433/920 收包全部
         * 停擺，事後會自我恢復（920 靜默看門狗會重新 StartRx），先印警告讓操作者知情。 */
        if (n >= 3 && strcmp(tok[2], "ring") != 0 && strcmp(tok[2], "all") != 0) {
            printf("[FLASH] 未知擦除範圍 '%s'（地面站僅支援 flash erase，無 ring/all 之分）\r\n", tok[2]);
        } else {
            printf("[FLASH] ⚠ 擦除期間停止接收下行遙測約 30-60 秒，請勿於飛行中執行。\r\n");
            {
                IWDG_HandleTypeDef iwdg_wide = hiwdg;
                iwdg_wide.Init.Prescaler = IWDG_PRESCALER_256;
                iwdg_wide.Init.Reload    = 1250;   /* 32kHz/256=125Hz -> 1250/125 = 10s，比照 main.c flash erase */
                HAL_IWDG_Init(&iwdg_wide);
            }
            W25QXX_StatusTypeDef st = FlashRing_EraseAll();
            HAL_IWDG_Init(&hiwdg);       /* 還原正常視窗 */
            HAL_IWDG_Refresh(&hiwdg);
            GroundStation_FlashResetAfterErase();
            if (st == W25QXX_OK) {
                printf("[FLASH] 接收紀錄已清空並重設寫入頭 OK\r\n");
            } else {
                printf("[FLASH] ERROR: 擦除回報失敗 (st=%d)，已強制重設寫入頭\r\n", (int)st);
            }
        }

    } else if (strcmp(tok[0], "flash") == 0 && n >= 2 && strcmp(tok[1], "dump") == 0) {
        /* 人眼讀 hex dump（沿用飛控角色共用的 Flash_DumpAll，純角色無關的原始讀取，
         * 與寫入者無關）——地面站晶片若曾被燒錄過飛控角色會在這裡看到殘留校準值，
         * 全新/已清空的地面站專用晶片則第一區塊會是一片 FF。 */
        SPI3_Bus_Lock();
        Flash_DumpAll();
        SPI3_Bus_Unlock();

    } else if (strcmp(tok[0], "flash") == 0 && n >= 2 && strcmp(tok[1], "export") == 0) {
        /* 結構化匯出（供 GUI 解析存檔）：
         *   ① Sector 0 hex —— 與 main.c 飛控角色 `flash export` 完全同格式（SYSFLAGS_START/
         *      END + 16-byte hex 行），故 GUI／flash_analyzer.py 的 sysflags 解析器可原封不動
         *      重用；地面站本身不寫這區，讀出來若非全 FF 表示晶片曾被燒成飛控角色。
         *   ② 地面站自己收到的下行遙測紀錄（GsLogRecord_t ring）——CSV 表頭/欄位改用
         *      gs_log.c 既有的 GsLog_CsvHeader()/GsLog_FormatCsvRow()，與 SD 卡 CSV 同源
         *      同格式，讓 ground_station_analyzer.py --csv 可直接分析剛匯出的這份檔案。 */
        SPI3_Bus_Lock();
        printf("--- SYSFLAGS_START ---\r\n");
        {
            uint8_t dump_buf[16];
            for (uint32_t dump_addr = FLASH_SYSFLAGS_ADDR;
                 dump_addr < FLASH_SYSFLAGS_ADDR + FLASH_SYSFLAGS_SIZE;
                 dump_addr += 16U) {
                if (W25QXX_ReadData(dump_addr, dump_buf, sizeof(dump_buf)) != W25QXX_OK) break;
                printf("%06lX:", (unsigned long)dump_addr);
                for (uint32_t i = 0; i < sizeof(dump_buf); i++) {
                    printf(" %02X", dump_buf[i]);
                }
                printf("\r\n");
                if (((dump_addr - FLASH_SYSFLAGS_ADDR) & 0x3FFU) == 0U) HAL_IWDG_Refresh(&hiwdg);
            }
        }
        printf("--- SYSFLAGS_END ---\r\n");

        printf("--- CSV_START ---\r\n");
        {
            char hdr[GS_LOG_CSV_MAX];
            if (GsLog_CsvHeader(hdr, sizeof(hdr)) > 0) printf("%s", hdr);
        }

        {
            GsLogRecord_t rec;
            char row[GS_LOG_CSV_MAX];
            uint32_t addr = FLASH_RINGBUF_ADDR;
            uint32_t read_count = 0;

            while (addr + GS_LOG_RECORD_SIZE <= FLASH_RINGBUF_END + 1UL) {
                if (W25QXX_ReadData(addr, (uint8_t*)&rec, sizeof(rec)) != W25QXX_OK) break;
                if (rec.magic0 == 0xFFU && rec.magic1 == 0xFFU) break;   /* 擦除後未寫入區 */

                if (GsLog_RecordValid(&rec)) {
                    if (GsLog_FormatCsvRow(row, sizeof(row), &rec) > 0) printf("%s", row);
                    read_count++;
                }
                addr += GS_LOG_RECORD_SIZE;
                if ((read_count & 0x3FU) == 0U) HAL_IWDG_Refresh(&hiwdg);
            }
            printf("--- CSV_END ---\r\n");
            if (read_count == 0U) {
                printf("[FLASH] Notice: 地面站接收紀錄為空（尚未收到下行遙測，或剛清空過）。\r\n");
            }
            printf("[FLASH] Export finished. Total %lu records.\r\n", (unsigned long)read_count);
        }
        SPI3_Bus_Unlock();

    } else if (strcmp(tok[0], "flash") == 0) {
        printf("[FLASH] 子命令：erase / dump / export\r\n");

    } else {
        printf("[TEST] 未知命令 '%s'，輸入 help\r\n", tok[0]);
    }
}

/* ============================================================
 *  公開 API
 * ============================================================ */
void GsLoraTest_Init(void)
{
    stats_reset_all();
    /* 影子參數同步驅動實際套用值（LoRaE80_Init 已跑完；失敗時維持上方預設） */
    LoRaE80_GetParams(&s_e80_freq_hz, &s_e80_sf, &s_e80_bw,
                      &s_e80_cr, &s_e80_pwr_dbm, &s_e80_preamble);
    /* E22 已於 main() 初始化區（IS_GROUND 分支）init 完成（s_inited=1），故此處不重複
     * init —— 上行命令直接用 LoRaE22_Send 即可（透傳模式 TX/RX 皆已就緒）。 */
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_u2_rxbuf, sizeof(s_u2_rxbuf));
    printf("[TEST] LoRa 通訊測試模組就緒（UART2 460800），輸入 help\r\n");
}

void GsLoraTest_OnUart2RxEvent(uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) u2_push(s_u2_rxbuf[i]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_u2_rxbuf, sizeof(s_u2_rxbuf));
}

void GsLoraTest_FeedRxBuffer(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        u2_push(buf[i]);
    }
}

void GsLoraTest_UpdateStats(uint8_t link, int16_t rssi_dbm, int16_t snr_q, uint8_t crc_ok)
{
    if (link > 1U) return;
    /* ★433 也可能帶 RSSI（E22 REG3 bit7 每包附加一個 RSSI 位元組），舊版把 433 硬擋掉，
     * 導致 bit7 修好之後 [STATS] 仍恆顯示 N/A、無從驗收。改成看值是否為哨兵即可。
     * SNR 仍只有 920 有（E80/LR1121 由晶片給；E22 無此資訊）。 */
    int has_rssi = (rssi_dbm != GS_RSSI_NA);
    int has_snr  = (link == GS_LINK_920) && (snr_q != GS_SNR_NA);
    lora_stats_on_packet(&s_stat[link], crc_ok ? 1 : 0,
                         has_rssi, rssi_dbm, has_snr, snr_q, HAL_GetTick());
}

void GsLoraTest_Tick(void)
{
    /* 掃描 UART2 命令（換行結尾） */
    static char    s_cmd_buf[128];
    static uint8_t s_cmd_len = 0;

    uint8_t b;
    while (u2_pop(&b)) {
        if (b == '\r' || b == '\n') {
            /* 行終止符：\r、\n 或 \r\n 皆可（空行如 \r\n 的第二字元忽略） */
            if (s_cmd_len > 0) {
                s_cmd_buf[s_cmd_len] = '\0';
                dispatch_cmd(s_cmd_buf);
                s_cmd_len = 0;
            }
        } else if (s_cmd_len < (uint8_t)(sizeof(s_cmd_buf) - 1U)) {
            s_cmd_buf[s_cmd_len++] = (char)b;
        }
    }

    /* 自動列印統計 */
    if (s_auto_stats) {
        uint32_t now = HAL_GetTick();
        if ((now - s_auto_last_ms) >= s_auto_period_ms) {
            s_auto_last_ms = now;
            print_stats();
        }
    }
}

#endif /* IS_GROUND */

/* ============================================================
 *  航電板 USB CDC CLI 橋接
 *  IS_GROUND 下由完整 GsLoraTest 模組提供；航電板提供同名 stub，
 *  將 USB 鍵入的文字透傳進 UART2（PA2）的環形緩衝，令 DiagTask
 *  裡的 arm / disarm / help 等文字命令同樣能透過 USB 觸發。
 * ============================================================ */
#if !IS_GROUND && FEATURE_USB_CDC

#include "gs_lora_test.h"   /* 僅為宣告；實體於此提供 */
#include <string.h>

/* 小型線性緩衝：USB CDC 中斷路徑 → DiagTask 輪詢消費 */
#define _USB_CLI_BUF_SZ 256U
static uint8_t  s_usb_cli_buf[_USB_CLI_BUF_SZ];
static volatile uint16_t s_usb_cli_head = 0U;
static volatile uint16_t s_usb_cli_tail = 0U;

void GsLoraTest_FeedRxBuffer(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0U; i < len; i++) {
        uint16_t next = (s_usb_cli_head + 1U) % _USB_CLI_BUF_SZ;
        if (next != s_usb_cli_tail) {       /* 防溢位 */
            s_usb_cli_buf[s_usb_cli_head] = buf[i];
            s_usb_cli_head = next;
        }
    }
}

/**
 * @brief 航電板：從 USB CLI 環形緩衝取出一個字元；無資料回傳 0。
 *        由 DiagTask（gs_lora_test 地面站版的 u2_pop 等效）透過此 API 消費。
 */
int GsLoraTest_PopUsbByte(uint8_t *out)
{
    if (s_usb_cli_head == s_usb_cli_tail) return 0;
    *out = s_usb_cli_buf[s_usb_cli_tail];
    s_usb_cli_tail = (s_usb_cli_tail + 1U) % _USB_CLI_BUF_SZ;
    return 1;
}

#endif /* !IS_GROUND && FEATURE_USB_CDC */
