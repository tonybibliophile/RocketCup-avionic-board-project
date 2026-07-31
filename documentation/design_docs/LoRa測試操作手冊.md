# 📡 LoRa 通訊測試操作手冊（地面站 / ROLE_GROUND）

本手冊說明如何用一台電腦，透過地面站板的 **UART2** 即時調整 LoRa RF 參數
（E22 433MHz / E80 920MHz）並觀察通訊品質統計，用於 bring-up 與鏈路調校。

> 對應程式：
> - [`gs_lora_test.c`](../Main_Code/Core/Src/gs_lora_test.c) — UART2 命令介面 + 雙鏈路統計
> - [`lora_calc.h`](../Main_Code/Core/Inc/lora_calc.h) — 純換算 / 統計（host 可測，見 [`tests/test_lora_calc.c`](../tests/test_lora_calc.c)）
> - [`lora_e80.c`](../Main_Code/Core/Src/lora_e80.c) — **LR1121** 驅動（E80）
> - [`lora_e22.c`](../Main_Code/Core/Src/lora_e22.c) — E22 透傳驅動

---

## 1. 架構概觀

```
                 ┌─────────────────── 地面站板 (ROLE_GROUND) ───────────────────┐
   ┌────────┐    │  UART2 (PA2/PA3, 460800 8N1)                                 │
   │   PC   │◄──►│   ├─ RX：解析文字命令（調頻率/SF/BW…、查統計）              │
   │ 終端機 │    │   └─ TX：printf 回應、統計輸出（測試「控制台」）            │
   └────────┘    │                                                              │
                 │  E22 433MHz (UART3 透傳)  ─┐                                  │
                 │  E80 920MHz (LR1121, SPI3)─┴► telem_rx 解析 → 統計           │
   ┌────────┐    │                                                              │
   │USB-CDC │◄───┤  USB 虛擬序列埠：串流收到的遙測 CSV（資料「資料流」）       │
   │ (資料) │    │                                                              │
   └────────┘    └──────────────────────────────────────────────────────────────┘
                            ▲ 下行遙測        ▼ 上行手動開傘命令（433 反向，見 §7）
                 ┌──────────┴────────────────┴──┐
                 │        火箭主航電             │  下行：E22 433 + E80 920
                 │  E22 每 3s 靜默 1s 聽上行     │  上行：地面 433 → 火箭手動開傘
                 └──────────────────────────────┘
```

**兩條序列埠各司其職：**
| 埠 | 用途 | 鮑率 |
| :-- | :-- | :-- |
| **UART2** (PA2/PA3) | 測試控制台：打命令、看統計與回應 | 460800 8N1 |
| **USB-CDC** | 純資料流：收到的遙測 CSV（給記錄/繪圖） | USB 虛擬埠 |

---

## 2. 硬體接線

E22 / E80 已焊在航電板上（接線見 [連線和基本硬體規格表.md](連線和基本硬體規格表.md)）。
測試只需把 **UART2** 接到電腦：

| 地面站板 | USB-TTL 轉接器 (如 CP2102/CH340) |
| :-- | :-- |
| `PA2` (USART2_TX) | RXD |
| `PA3` (USART2_RX) | TXD |
| `GND` | GND |

> ⚠️ TX↔RX 必須交叉。USB-TTL 的 VCC **不要**接（板子自有供電）。
> ⚠️ 轉接器邏輯準位須為 **3.3V**（勿用 5V TTL 直連 STM32）。

天線：E22 / E80 **務必接好對應頻段天線**再上電，空載發射可能損壞 PA。

---

## 3. 編譯與燒錄

```bash
make build-ground      # 編譯地面站 binary（BOARD_ROLE=ROLE_GROUND）
make flash-ground      # 編譯 + 透過 ST-Link 燒錄
```

> 主/備航電不受影響：`gs_lora_test` 整檔以 `#if IS_GROUND` 包住。

---

## 4. 連線終端機

序列埠參數：**460800 / 8 / N / 1**，無流量控制。

macOS（內建 screen）：
```bash
ls /dev/cu.usbserial-*          # 找出轉接器埠號
screen /dev/cu.usbserial-XXXX 460800
# 離開：Ctrl-A 再按 K，然後 y
```
或用 `minicom -D /dev/cu.usbserial-XXXX -b 460800`、CoolTerm、PuTTY 等。

> 命令以 **Enter** 結尾即可（程式接受 `\r`、`\n`、`\r\n`）。
> 上電後會看到：`[TEST] LoRa 通訊測試模組就緒（UART2 460800），輸入 help`

---

## 5. 命令參考

| 命令 | 說明 |
| :-- | :-- |
| `help` | 顯示命令清單 |
| `ver` | 顯示 E80(LR1121) 版本與初始化診斷（HW / Type=0x03 / Stat1） |
| `stats` | 顯示雙鏈路統計 |
| `stats reset` | 清除統計計數器 |
| `stats auto <sec>` | 每 N 秒自動列印統計（`stats auto 0` 關閉） |
| `e22 freq <mhz>` | 設 E22 頻率 410–493 MHz（寫入 EEPROM，掉電保留） |
| `e22 pwr <0-3>` | 設 E22 發射功率（0=30dBm 1=27 2=24 3=21；本板 3V3 用 3） |
| `e22 air <0-7>` | 設 E22 空中速率（2=2.4k；★兩端必須相同才能通訊） |
| `e22 show` | 顯示 E22 全部設定暫存器（含 `SubPkt` / `RSSIbyte` / `RSSInoise`） |
| `e22 dump on\|off` | 逐位元組印出 433 原始 hex（`[E22RAW]`），用來核對 RSSI 位元組落點 |
| `e80 freq <hz>` | 設 E80 中心頻率（Hz，例 `e80 freq 915000000`） |
| `e80 sf <7-12>` | 設 E80 展頻因子 |
| `e80 bw <idx>` | 設 E80 頻寬 index（見 §8） |
| `e80 cr <1-4>` | 設 E80 編碼率（1=4/5 … 4=4/8） |
| `e80 pwr <dbm>` | 設 E80 發射功率 −9~22 dBm |
| `e80 pre <n>` | 設 E80 前導碼長度 6~65535 |
| `e80 show` | 顯示 E80 RF 參數 + 空中時間估算 |
| `e80 airtime <len>` | 估算指定 payload 長度的空中時間 / 等效 bitrate |
| `e80 init` | 重新初始化 E80（LR1121）並進入接收 |
| `e80 rxstart` | 重新進入連續接收 |

**統計輸出範例：**
```
[STATS] --- E80-920 ---
[STATS] pkt_ok=412  crc_err=3
[STATS] rate=9.8 pkt/s  elapsed=42s
[STATS] RSSI: last=-87 min=-95 max=-71 avg=-86 dBm
[STATS] SNR:  last=36 min=8 max=44 avg=30 (x0.25dB)   ← dB = 數值 ÷ 4
```
> E22 **無 SNR**（模組不提供）。逐包 RSSI 則取決於 REG3 bit7「RSSI 位元組致能」
> （韌體以 `lora_e22.c` 的 `E22_RSSI_BYTE_EN` 明確寫入）：開啟後模組會在每一次
> **無線接收**的酬載之後多吐一個 RSSI 位元組，接收端剝掉它並換算 `-(256-v)` dBm。
>
> ⚠ 這個功能對「一個應用封包 = 一個空中子封包」有硬性依賴。E22 是
> *auto sub-packaging*（datasheet p.12 §5.6.2）：若 116-byte 遙測封包被拆成多個
> 子封包，模組會**每個子封包各附一個 RSSI 位元組**，等於插進封包中間，
> CRC 會幾乎全錯。韌體已用兩道措施保證不拆：
> 1. REG1 bit[7:6] 明確寫成 **240B**（`E22_SUBPKT_CODE`），116 < 240；
> 2. `LoRaE22_Send()` 走 **UART DMA**，位元組流中間不會因任務被搶佔而出現 idle 空隙。
>
> 出問題時先用 `e22 show` 確認 `SubPkt=240B`、再用 `e22 dump on` 確認兩個
> `A5 5A` 之間剛好 117 bytes（116 酬載 + 1 個尾端 RSSI）。

---

## 6. 典型測試流程

### A. 快速健檢（確認 E80 在線）
```
ver           → Type 應為 0x03（LR1121）、Stat1 非 0x00/0xFF
e80 show      → 確認頻率/SF/BW 與火箭 TX 一致
stats auto 5  → 每 5 秒看統計，確認 pkt_ok 持續增加
```

### B. 比較不同 SF 的距離 / 穩定度
```
stats reset
e80 sf 7      → 走遠，記錄 rate / RSSI / crc_err
stats reset
e80 sf 10     → 同一距離再測，比較靈敏度與封包率
```
> SF 越高靈敏度越好但空中時間越長（`e80 show` 可看 airtime）。

### C. 掃頻找乾淨頻道
```
e80 freq 920000000
stats reset
stats auto 3
... 觀察 → 換 e80 freq 921000000 ... 比較 crc_err / RSSI 底噪
```

> ⚠️ **關鍵：地面站改參數只改「接收端」。** 要實際通訊，火箭主航電 TX 必須用
> **相同**的頻率 / SF / BW / CR / sync word。本工具用途是「找到好參數」，
> 找到後把同一組值同步寫到主航電（`lora_e80.c` / `lora_e22.c` 的 `#define`）兩邊一致。

---

## 7. 上行手動開傘（433 反向鏈路）

除了接收下行，地面站可經 **433 反向**打命令給火箭，**手動觸發開傘**（範圍安全 /
備援用途）。對應 [`uplink_proto.h`](../Main_Code/Core/Inc/uplink_proto.h)（框架，
火箭/地面共用）、[`uplink_cmd.c`](../Main_Code/Core/Src/uplink_cmd.c)（火箭端 RX +
武裝/點火）、`gs_lora_test.c`（地面送命令）。

### 7.1 運作原理（時間分割）

433 是單頻、半雙工：火箭在發下行時，地面送的上行會撞包。因此火箭每 3 秒
（`UPLINK_LISTEN_PERIOD_MS`）空出一段**連續 1 秒完全不發 433**的時間，此時 E22
透傳即在接收。這 1 秒由兩段組成：

| 段 | 常數 | 長度 | 作用 |
| :--- | :--- | ---: | :--- |
| 發射守衛帶 | `UPLINK_TX_GUARD_MS` | 600ms | 窗**開始前**就停止餵資料，讓在途的下行封包在窗開始前送完 |
| 接收窗 | `UPLINK_LISTEN_HOLD_MS` | 400ms | 保證靜默的下限 |

地面命令以 **4.5 秒 burst 重複送**（`UPLINK_TX_WINDOW_MS`，必須長於火箭的窗週期），
且**忙線時不放棄、改短間隔重試**，AUX 一放開就立刻送出——本機收不到東西的時刻，
正好就是對方靜聽的時刻，等於自動對準窗口。920 下行不受影響、維持全速率。

```
火箭 433: [TX][TX][TX]...[TX]│← 守衛帶 600ms →│← 接收窗 400ms →│[TX][TX]...
                            └ 最後一包在此起飛，最晚 500ms 後送完
                                        └──── 保證靜默 ≥500ms ────┘
                                             ▲ 地面 burst 命中此段
```

> ⚠️ **守衛帶必須大於一包下行的最大空中時間，否則整個機制失效。**
> 目前 `TELEM_PACKET_SIZE=99B @2.4k`：99×8/2400 = 330ms 純資料，加 LoRa 前導/表頭/
> CRC 上限約 500ms，故守衛帶取 600ms 留餘裕。**改動封包大小、空中速率、
> `LORA_TELEM_PERIOD_MS` 或 `LORA433_TX_EVERY` 時都必須重算並確認守衛帶仍然夠大。**

> **兩次踩過的坑（都是同一個成因：名目窗長 ≠ 真正的靜默長度）**
> 1. 2026-07-30 之前是「跳過 1 個 100ms 排程時槽」，跳過時模組還在把前一包送上
>    空中，空中完全沒有靜默——實測連送 10 次 `arm` 全無 `[ACK]`、FSM 停在 PAD。
> 2. 改成「每 3 秒空 800ms」後**仍然不通**。`listen_slot` 只在迴圈開頭判斷一次，
>    一包在窗開始前一瞬間才起飛的封包會吃掉窗的前半段，真正靜默只剩 ~300ms，
>    而上行幀含前導碼要 80~150ms 空中時間、且模組必須在前導碼之前就進 RX ——
>    能塞進去的機會趨近 0。**判定實驗**：`LORA433_RX_ONLY=1`（433 完全不發）時
>    `arm` 立刻就通，設回 0 就不通，證明 RF 層無罪、問題 100% 在窗。
>    解法就是上面的發射守衛帶。
>
> 代價：433 不發射時間 1000/3000 ≈ 33%（920 不受影響）。

### 7.1.1 上行不通時怎麼查

兩端各有一行計數器，照順序看就能定位斷在哪一段：

| 看哪裡 | 現象 | 結論 |
| :--- | :--- | :--- |
| 地面站 `[UPLINK] … burst 結束：ok=N` | `ok=0` | 本機 E22 全程忙線，命令**根本沒上空中**；查 433 下行是否佔滿、AUX 接線/供電 |
| 火箭 `[UPLINK_STAT] raw=` | 一直是 0 | 射頻層沒東西進來。注意 `raw` 是**無條件累加原始位元組**、不看內容，所以 baud 不符只會讓它收到亂碼、仍會漲；恆為 0 代表模組一個位元組都沒吐出來。用 `LORA433_RX_ONLY=1` 一刀切：通了＝窗的問題，不通＝頻道/空速/位址/天線/接線 |
| 火箭 `[UPLINK_STAT] bin_crc/bin_rsync` | 只有這兩個在漲 | 有訊號但幀壞掉（誤碼，或被自己的發射截斷） |
| 火箭 `[UPLINK_STAT] bin_ok` | 有漲 | 命令**確實收到了**，問題在後面的 ARM／狀態閘，看 `[UPLINK]` 那幾行的判定結果 |

### 7.2 安全：兩段式 ARM → DEPLOY

- **DEPLOY 命令只有在火箭「已武裝」時才會點火。** 先送 `arm`，再送 `deploy`。
- 武裝後 **30 秒**（`UPLINK_ARM_TIMEOUT_MS`）未開傘自動解除，避免遺留武裝。
- 命令框架有獨立 sync（`0x55 0xAA`）+ CRC-16，雜訊/誤碼不會誤觸（host 測試覆蓋）。
- 手動副傘 MOSFET 導通限時 `FSM_PYRO_HOLD_MS`(2s) 後自動斷開；主傘舵機維持釋放角度。
- 與既有 FSM 自動開傘**並存**：手動點火會鎖存 `drogue_fired`，FSM 不重複點火。
- 整功能由 `FEATURE_UPLINK_DEPLOY`（預設僅主航電）開關。

> ⚠️ 手動開傘一旦武裝後即可在**任何飛行狀態**生效（真正的人工覆寫）。地面操作者
> 須自行判斷時機（例如副傘誤在上升段開傘可能造成結構破壞）。ARM 步驟即為防誤觸閘。

### 7.3 操作步驟（地面站 UART2）

```
ping            → 確認上行通：火箭主控台應印出 [UPLINK] PING…（約 3s 內）
arm             → 火箭印 [UPLINK] *** ARMED ***
deploy drogue   → 火箭印 [UPLINK] *** 手動開傘 *** drogue=1
                  ↳ 確認：下行遙測 flags 出現 DROGUE_FIRED（PD13 實際導通推導）
disarm          → 不開傘時解除武裝
```

> **確認回路**：火箭點火後，下行 `DROGUE_FIRED` / `MAIN_DEPLOYED` 旗標（由 PD13 GPIO /
> TIM4 舵機實際狀態推導，非軟體自報）會翻為 1，地面站立即可見 —— 即「真的點了」的證據。

### 7.4 上板測試（不裝火藥）

1. 火箭端把 PD13 接 LED / 萬用表（或示波器），主傘看 TIM4_CH3 舵機。
2. 地面 `ping` 確認上行通 → `arm` → `deploy drogue`。
3. 觀察：火箭 LED 亮 2 秒後熄（限時保護）、下行 `DROGUE_FIRED` 旗標翻起再依硬體狀態。
4. 測 `arm` 逾時：`arm` 後等 >30s 再 `deploy`，應被拒（火箭印「未武裝」）。
5. 測未武裝保護：重開機後直接 `deploy`，應被拒。

---

## 8. ⚠️ 上板驗證清單（LR1121 板級項目）

E80-900M2213S 核心是 **Semtech LR1121**。下列為「板級」設定，集中在
[`lora_e80.c`](../Main_Code/Core/Src/lora_e80.c) 頂部 `#define`，**務必對照 E80
規格書逐項驗證**，否則最常見的症狀是「`ver` 正常但完全收不到封包」：

- [ ] **RF 開關真值表（最關鍵）** `E80_RFSW_*`
  LR1121 的 **DIO5=RFSW0、DIO6=RFSW1** 控制模組內建 RF 開關。預設採 Semtech
  LR1121 參考設計（RX=RFSW0 高、TX_HP=RFSW1 高）。若 E80 接線不同，**收發 RF 完全不通**。
  驗證：`ver` 能讀到 LR1121 但 `stats` 永遠 0 封包 → 高度懷疑此項。
- [ ] **參考振盪器** `E80_USE_TCXO`
  E80 內建 32MHz 主動式振盪器。預設 `0`（視為自供電 clipped-sine 進 XTA）。
  若 bring-up 顯示頻率嚴重偏移 / 收不到 → 試 `=1`（LR1121 供電 TCXO，須重校準）。
- [ ] **IRQ 對應的 LR1121 DIO**
  程式把 RX/TX 事件放在 `SetDioIrqParams` 的 **dio1 遮罩**，假設模組 INT 腳
  （板上 PD4/EXTI4）接 LR1121 **DIO9**。若實際接其他 DIO，需改 `e80_set_dio_irq()`。
  驗證：能收（`e80 init` 後手動 poll 有資料）但 `RxReady()` 一直 0 → 懷疑 IRQ 沒進。
- [ ] **頻率 / SF / BW / CR / sync word 與主航電一致**
  兩端任一不符即收不到。sync word 預設 `0x12`（私有）。
- [ ] **天線與功率** 發射端 22dBm，務必接天線；近距離測試可降 `e80 pwr` 避免飽和。

> 完成驗證後，請把確認過的數值更新到本手冊與 `lora_e80.c`，並於
> [進度.md](進度.md) / [待確認、改進清單.md](待確認、改進清單.md) 記錄。

---

## 9. 參數對照表

**LoRa 頻寬 index（`e80 bw <idx>`，LR1121 值）：**

| idx | 頻寬 | idx | 頻寬 |
| :-: | :-- | :-: | :-- |
| 0x03 | 62.5 kHz | 0x05 | 250 kHz |
| 0x04 | 125 kHz | 0x06 | 500 kHz |

（更窄：0x00=7.8k, 0x01=15.6k, 0x02=31.25k, 0x08=10.4k, 0x09=20.8k, 0x0A=41.7k）

**展頻因子：** SF7~SF12（值即數字）。**編碼率：** 1=4/5, 2=4/6, 3=4/7, 4=4/8。

**LDRO（低資料率最佳化）：** 由韌體自動依「符號時間 ≥ 16ms」判定
（例 BW125 下 SF11/SF12 自動開），無需手動設定。

**空中時間參考**（顯式表頭 + CRC on，preamble=8，payload≈79B 遙測封包，用 `e80 airtime` 即時算）：
SF 每 +1、空中時間約 ×2；BW ×2、空中時間約 ÷2。

---

## 10. 疑難排解

| 症狀 | 排查 |
| :-- | :-- |
| 終端機沒任何輸出 | 鮑率非 460800？TX/RX 沒交叉？GND 沒共地？ |
| 命令沒反應 | 有按 Enter？輸入 `help` 測試；確認轉接器 3.3V |
| `ver` Type≠0x03 / Stat1=0x00 或 0xFF | SPI 不通：MISO 接地(0x00)/浮空(0xFF)；查 CS/SCK/接線 |
| `ver` 正常但 `stats` 一直 0 封包（E80） | **RF 開關 `E80_RFSW_*`**（§8 第一項）；天線；與 TX 參數不一致 |
| E80 收不到、E22 正常 | LR1121 板級項（RF 開關 / IRQ DIO / TCXO） |
| E22 收不到、E80 正常 | E22 頻道與 TX 不符（`e22 show`）；UART3 接線；M0/M1 模式 |
| crc_err 很高 | 訊號弱（看 RSSI/SNR）、頻道干擾（換頻）、兩端參數不完全一致 |
| 433 只要開 RSSI 就幾乎全 CRC 錯 | E22 auto sub-packaging 把封包拆成多個子封包，模組每個子封包各附一個 RSSI 位元組、插進封包中間。`e22 show` 確認 `SubPkt=240B`；`e22 dump on` 確認兩個 `A5 5A` 之間是 117 bytes。發射端必須用 UART DMA 送出整包（勿改回阻塞傳輸） |
| 改了參數後反而全斷 | 只改了地面站；火箭 TX 未同步成相同參數 |
| `deploy` 火箭沒反應 | 先 `arm`？ARM 是否已逾時(30s)？`ping` 先確認上行通 |
| `ping`/`arm` 火箭收不到 | 上行需命中安靜窗：命令是 ~3s burst，勿中途打斷；E22 兩端同頻同 NETID |
| `deploy` 印「未武裝」 | 正常保護：須先 `arm` 且在 30s 窗內 |

---

## 11. 附錄：為什麼 E80 驅動要對 LR1121 重寫

E80-900M2213S 的核心是 **LR1121**，與 SX126x 命令協定截然不同。早期驅動誤用
SX126x 指令集，在 LR1121 上**完全不會動作**。主要差異：

| 項目 | SX126x（錯誤假設） | LR1121（E80 實際） |
| :-- | :-- | :-- |
| opcode | 1 byte（`0x86`） | **2 bytes**（`0x020B`） |
| 設頻率 | `Frf=f·2²⁵/32e6` 公式 | **直接 Hz**，4 bytes 大端 |
| 讀回應 | 同一次 SPI 交易 | **獨立第二次交易**（先 Stat1） |
| LoRa 封包型態 | `0x01` | **`0x02`** |
| LoRa sync word | 2-byte 暫存器 `0x0740` | **1-byte** 命令 `0x022B` |
| IRQ 遮罩 | 16-bit | **32-bit**；狀態含於 GetStatus |
| PA / RF 開關 | SX126x PA | **HP PA + DIO5/DIO6 RF 開關（必設）** |

opcode 與位元組佈局依 Semtech LR1121 User Manual / SWDR001 驅動核對。
板級項（RF 開關真值表、TCXO、IRQ DIO）仍須依 §7 於實機驗證。
