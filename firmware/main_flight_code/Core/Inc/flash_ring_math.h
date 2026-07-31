/*
 * flash_ring_math.h — Flash 環形緩衝區幾何數學（P1，header-only，host 可測）
 * ===========================================================================
 * 自 w25qxx.c 抽出的純位址計算：池量、寫入/擦除指標推進、迴繞、熱啟動回讀
 * 位址。不依賴 HAL / SPI（僅 stdint），由 tests/test_flash_ring.c 驗證。
 * w25qxx.c 僅保留實際 SPI 讀寫/擦除與狀態變數。
 *
 * 本次抽離同時修復迴繞歧義 bug（P1 flash ring 加固項）：
 *   舊版 erased_end 採「可停在 FLASH_RINGBUF_END+1」的 exclusive-end 語意
 *   （滾動擦除以 `> END+1` 才迴繞、PreEraseOne 以 `> END` 補正規化，兩處
 *   已不一致）。當擦除推進到環尾(erased_end=END+1)且寫入指標隨後也迴繞回
 *   BASE 時，pool = erased_end − write = 整個環的假池量 —— 實際上 BASE 起
 *   的扇區是最舊資料、根本未擦。之後整圈寫入全部落在未擦區（NOR flash
 *   只能 1→0，覆寫=資料損毀），且因 W25QXX_WriteData 不回讀驗證而silent。
 *   長時間 bench 浸泡（30min ≈ 2.9MB）累積數次即會繞滿 15.9MB 觸發。
 *
 *   新版：erased_end 一律正規化 —— 推進後 ≥ END+1 即迴繞回 BASE，環上恆有
 *   erased_end ∈ [BASE, END]。語意：pool = write→erased_end 的環向距離，
 *   erased_end == write ⇒ pool = 0（空池）。「全環已擦」與「空池」的兩針
 *   歧義由池上限消除：FLASH_RING_PREERASE_TARGET（見下方定義）遠小於
 *   環容量 15.9MB（4080 sectors），pool 永不合法地達到整環。
 */
#ifndef FLASH_RING_MATH_H
#define FLASH_RING_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === 環形緩衝區幾何（自 w25qxx.h 移入；單一真相來源） === */
#define FLASH_RINGBUF_ADDR       0x010000UL   /* Ring Buffer 起始（64KB 對齊） */
#define FLASH_RINGBUF_END        0xFFFFFFUL   /* Ring Buffer 結束（inclusive） */
#define FLASH_RINGBUF_SIZE       0xFF0000UL   /* ~15.9 MB */
#define FLASH_RING_PACKET_SIZE   128UL        /* 每筆封包大小 128 bytes（含 flight_id，4KB 對齊） */
#define FLASH_RING_PREERASE_N    10           /* 開機預擦 Sector 數量 */

/* P0-E：飛行預擦目標池。每 sector 4096/128 = 32 封包，飛行態寫入率固定 100Hz
 * （main.c：STATE_BOOST..STATE_MAIN_DEPLOY 之 ring_period_ms=10，兩 profile 皆同）。
 * 飛行態（含 STATE_PAD_ARMED，見 main.c FlashRing_SetEraseAllowed 呼叫處）禁止
 * 同步滾動擦除（最壞 ~400ms 阻塞主迴圈，FSM 停擺、EKF 斷饋且持 SPI3 mutex）——
 * 池必須撐滿整段「同步擦除被禁止」的視窗，即 STATE_PAD_ARMED 進入到
 * STATE_MAIN_DEPLOY 結束。
 *
 * ★2026-07-31 更正：舊註解寫「PAD_ARMED 完全不寫入 flash，地面待命再久也不消耗」，
 * 與 main.c 現況不符 —— PAD_ARMED 以 1Hz 寫入（記錄武裝後、起飛前的地面狀態），
 * 且該段同步擦除已被禁止、背景 PreEraseOne 也只在 state <= STATE_PAD 才跑，池在
 * 武裝期只出不進。故池的真實預算是「武裝等待 + 飛行」兩筆帳：
 *   飛行可記錄秒數 = (池封包數 − 武裝秒數) ÷ 100
 * 舊值 960 sectors（30720 封包）下，武裝 12 分鐘就把飛行量壓到 300s、與
 * FSM_HOTSTART_MAX_TICK_MS 相等（熱重啟窗口再也無法靠「通電放久」老化出去，見
 * fsm.h 該常數說明）；電梯 profile 更只需武裝 7 分鐘就蓋不滿看門狗視窗。
 *
 * 本檔不 include board_config.h（維持 header-only、不依賴 HAL/profile 巨集），
 * 故此常數無法依 FLIGHT_PROFILE_ELEVATOR 分流，須直接取兩 profile 中「較大」的
 * 硬上限（此視窗長度 = FSM_MAIN_WATCHDOG_MS + FSM_MAIN_INFLATE_MS，見 fsm.h）：
 *   飛行 profile：248000 + 3000 = 251000ms = 251s（唯一真實來源見 fsm.h，勿抄值）
 *   電梯 profile：300000 + 3000 = 303000ms = 303s ← 較大，取此為硬上限
 * 303s + ~4s 邊界餘裕 ≈ 307.2s → 307.2s × 100Hz ÷ 32 封包/sector = 960 sectors，
 * 這是「飛行段本身」的硬需求下限（舊值即取此）。
 *
 * ★2026-07-31：960 → 1500（使用者決策）。理由是上面那筆武裝帳：
 *   1500 sectors = 48000 封包。武裝 60 分鐘（3600 封包）後仍剩 (48000−3600)/100
 *   = 444s 飛行量，涵蓋電梯 profile 的 303s 視窗仍有 141s 餘裕；同時遠高於
 *   FSM_HOTSTART_MAX_TICK_MS(300s)，維持「卡在飛行狀態的板子通電放久就能老化出
 *   熱重啟窗口」這條逃生路徑（見 fsm.h 該常數說明）。
 * ★放大池「不」增加擦除時間：擦除已改為使用者觸發，`flash erase` 本來就整環全擦
 *   （255 次 64KB block erase），之後整環皆 0xFF，池由 FlashRing_ProbePoolNoErase()
 *   唯讀認領，零額外擦除。1500 仍遠小於環容量 4080 sectors，不影響上方迴繞歧義
 *   防呆的池上限設計。
 * ⚠️ 若 fsm.h 任一 profile 的 FSM_MAIN_WATCHDOG_MS／FSM_MAIN_INFLATE_MS 之後調整
 * （例如換 profile 或重推 OpenRocket），此值須同步依上式重算（取兩 profile較大者
 * 再加上預期武裝時間），並於發射檢核表確認 [FLASH] pool 達標後才起飛。
 * ★2026-08-01：本值是「開機自動補擦」的目標——開機序列以 FlashRing_TopUpPool() 把池補到
 * 此值（見 main.c StartDefaultTask），故正常開機完成後必定達標。已無 ARM 閘（使用者決策），
 * fsm.c 的 flash_pool_ready 保留但 main.c 固定餵 1。 */
#define FLASH_RING_PREERASE_TARGET 1500U      /* ARM 前要求的最低已擦池（sectors） */

/* 擦除粒度。必須等於 W25QXX_SECTOR_SIZE（w25qxx.h 以 _Static_assert 鎖定）。 */
#define FLASH_RING_SECTOR_SIZE   4096UL

/* 幾何不變量：封包與 sector 都恰好鋪滿整環（無封包跨環尾、環尾 sector 對齊）。
 * 0xFF0000 / 128 = 130560 整；0xFF0000 / 4096 = 4080 整。
 * ring_last_packet_addr() 的迴繞回讀與 erased_end 正規化皆依賴此性質。 */
#if (FLASH_RINGBUF_SIZE % FLASH_RING_PACKET_SIZE) != 0
#error "FLASH_RINGBUF_SIZE 必須是 FLASH_RING_PACKET_SIZE 的整數倍"
#endif
#if (FLASH_RINGBUF_SIZE % FLASH_RING_SECTOR_SIZE) != 0
#error "FLASH_RINGBUF_SIZE 必須是 FLASH_RING_SECTOR_SIZE 的整數倍"
#endif

/* 池量（bytes）：write → erased_end 的環向距離。兩針相等 ⇒ 0（空池）。 */
static inline uint32_t ring_pool_bytes_calc(uint32_t write_addr, uint32_t erased_end)
{
    if (erased_end >= write_addr) {
        return erased_end - write_addr;
    }
    return (FLASH_RINGBUF_END + 1UL - write_addr) + (erased_end - FLASH_RINGBUF_ADDR);
}

/* 寫入指標推進一筆封包（含迴繞）。因封包恰好鋪滿整環，推進後恰落 END+1 即迴繞。 */
static inline uint32_t ring_write_advance(uint32_t write_addr)
{
    write_addr += FLASH_RING_PACKET_SIZE;
    if (write_addr + FLASH_RING_PACKET_SIZE > FLASH_RINGBUF_END + 1UL) {
        write_addr = FLASH_RINGBUF_ADDR;
    }
    return write_addr;
}

/* 下一個待擦 sector 的位址（= 正規化後的 erased_end 本身；防衛性收斂）。 */
static inline uint32_t ring_erase_target(uint32_t erased_end)
{
    return (erased_end > FLASH_RINGBUF_END) ? FLASH_RINGBUF_ADDR : erased_end;
}

/* 擦除指標推進一個 sector（正規化迴繞：≥ END+1 即回 BASE，恆 ∈ [BASE, END]）。 */
static inline uint32_t ring_erase_advance(uint32_t erased_end)
{
    erased_end = ring_erase_target(erased_end) + FLASH_RING_SECTOR_SIZE;
    if (erased_end >= FLASH_RINGBUF_END + 1UL) {
        erased_end = FLASH_RINGBUF_ADDR;
    }
    return erased_end;
}

/* 給定位址所在 sector 的下一個 sector 邊界（正規化迴繞：≥ END+1 即回 BASE）。
 * 熱重啟池起點用：寫入頭「之後、同 sector 內」的剩餘空間必定仍是已擦狀態
 * （該 sector 進入寫入前已擦淨，且封包嚴格由低位址往高寫），故那段可直接計入池，
 * 而 erased_end 收在此邊界即維持 sector 對齊不變量。 */
static inline uint32_t ring_sector_end(uint32_t addr)
{
    uint32_t end = (addr & ~(FLASH_RING_SECTOR_SIZE - 1UL)) + FLASH_RING_SECTOR_SIZE;
    return (end >= FLASH_RINGBUF_END + 1UL) ? FLASH_RINGBUF_ADDR : end;
}

/* 最後一筆已寫封包的位址（熱啟動回讀）。write==BASE 時為環尾最後一格。 */
static inline uint32_t ring_last_packet_addr(uint32_t write_addr)
{
    if (write_addr == FLASH_RINGBUF_ADDR) {
        return FLASH_RINGBUF_END + 1UL - FLASH_RING_PACKET_SIZE;
    }
    return write_addr - FLASH_RING_PACKET_SIZE;
}

/* 給定封包位址的前一筆封包位址（環向後退一格）。 */
static inline uint32_t ring_prev_packet_addr(uint32_t pkt_addr)
{
    if (pkt_addr == FLASH_RINGBUF_ADDR) {
        return FLASH_RINGBUF_END + 1UL - FLASH_RING_PACKET_SIZE;
    }
    return pkt_addr - FLASH_RING_PACKET_SIZE;
}

/* 區間 [addr, addr+len) 是否完整落在已擦池 [write, erased_end) 環區間內。
 * 寫入路徑的安全不變量（host 測試用；firmware 寫入前置檢查亦可呼叫）。 */
static inline uint8_t ring_span_in_pool(uint32_t write_addr, uint32_t erased_end,
                                        uint32_t addr, uint32_t len)
{
    uint32_t pool = ring_pool_bytes_calc(write_addr, erased_end);
    /* addr 相對 write 的環向偏移 */
    uint32_t off = (addr >= write_addr)
                   ? (addr - write_addr)
                   : (FLASH_RINGBUF_END + 1UL - write_addr) + (addr - FLASH_RINGBUF_ADDR);
    return (off + len <= pool) ? 1U : 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* FLASH_RING_MATH_H */
