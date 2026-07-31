/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   共用二進制下行遙測封包 (binary + CRC16)
 *
 * 同一份 TelemetryPacket_t 由兩條 LoRa 下行鏈路同時發送：
 *   - E22-400T30S (433MHz, UART3 透傳)   → lora_e22.c
 *   - E80-900M2213S (920MHz, SX126x SPI) → lora_e80.c
 *
 * 封包為 packed binary，結尾附 CRC-16/CCITT-FALSE；地面站據 sync word 對齊、
 * 以 seq 偵測丟包、以 crc16 校驗完整性。所有浮點量皆乘倍率轉整數
 * （遵循專案黃金法則：避免 %f 消耗堆疊/CPU，見 教學.md）。
 *
 * 物理單位與倍率（地面端解碼契約，請勿任意更動欄位順序）：
 *   - 加速度 (BMI088 低G)：mg        = g × 1000   (±24g 量程，int16 足夠)
 *   - 角速度 (BMI088)     ：dps                    (±2000dps，int16 足夠)
 *   - 高G   (ADXL375)     ：cg(0.01g) = g × 100    (±200g 量程，故用 cg 而非 mg 以免溢位)
 *   - 磁場  (MMC5983)     ：mGauss    = Gauss × 1000 (body frame，±8G)
 *   - 高度/速度           ：cm / (cm/s) = m × 100
 *   - 四元數              ：×10000
 ******************************************************************************
 */
#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 同步字（封包起始）。地面端逐位元組掃描 0xA5,0x5A 對齊封包邊界。 */
#define TELEM_SYNC0  0xA5U
#define TELEM_SYNC1  0x5AU

/* flags 位元定義 */
#define TELEM_FLAG_DROGUE_FIRED   0x01U  /* PD13 FIRE MOSFET 導通（副傘點火） */
#define TELEM_FLAG_MAIN_DEPLOYED  0x02U  /* 主傘舵機已轉至釋放角度 */
#define TELEM_FLAG_SD_ACTIVE      0x04U  /* SD 卡正在記錄 */
#define TELEM_FLAG_GPS_STALE      0x08U  /* GPS 定位逾時 (>2s 無有效 fix) */
#define TELEM_FLAG_EKF_UNHEALTHY  0x10U  /* EKF 健康位非 0（P0-C；FSM 已切 raw-baro 降級鏈） */
#define TELEM_FLAG_SENSOR_FAULT   0x20U  /* 任一感測器失流/卡死/範圍失效（P0-D，詳見 [HEALTH] 行） */
#define TELEM_FLAG_FAILSAFE       0x40U  /* 失效保護計時器強制點火（P0-B；地面站需特別標示） */
#define TELEM_FLAG_HOTSTART       0x80U  /* 空中斷電熱啟動恢復成功（P0-F） */

/* arm_flags 位元定義（flags 8 位已滿，ARM 被擋原因獨立一個 byte，供地面站顯示） */
#define TELEM_ARM_BLOCKED_FLASH_POOL 0x01U  /* ARM 已送出但 flash 預擦池未達標，仍留 STATE_PAD
                                              * （fail-open：flash 停用/未偵測到/未記錄時不會設此位） */

/* peer_link 位（主/副協同下鏈：主板把副板鏈路健康中繼給地面，供雙板監看） */
#define TELEM_PEER_EVER    0x01U  /* 曾收過對端封包（valid） */
#define TELEM_PEER_FRESH   0x02U  /* 對端在線（LINK_PEER_TIMEOUT_MS 內收過） */
#define TELEM_PEER_LOST    0x04U  /* 曾失聯（Phase B 設定） */
#define TELEM_PEER_DESYNC  0x08U  /* 狀態失同步（Phase B 設定） */

/* profile_flags 位：board_config.h FLIGHT_PROFILE_ELEVATOR 醒目標示。正式版預設兩板
 * 皆為 0（真實飛行 profile）；只要任一板仍以電梯測試 profile 編譯/燒錄（例如換板未
 * 重燒、忘記切回），地面站/GUI 必須顯著警示，避免誤以測試門檻上真實彈道。
 * SELF 由本板 telemetry.c 依編譯期巨集直接填；PEER 由主板依板間鏈路（link_proto.h
 * 同一位元語意）中繼副板回報值換算而來，僅主板下鏈時填寫。 */
#define TELEM_PROFILE_SELF_ELEVATOR  0x01U  /* 本板韌體以 FLIGHT_PROFILE_ELEVATOR=1 編譯 */
#define TELEM_PROFILE_PEER_ELEVATOR  0x02U  /* 對端（副板）韌體亦為電梯測試 profile */

/* drogue_alt_m / main_alt_m 的「尚未開傘」哨兵。刻意不用 0：0 m 是合法的開傘高度
 * （地面誤觸發／發射台高度誤判），用 0 當「沒開」會把真正該警示的事件藏起來。
 * 與 w25qxx.h FlashRingPacket_t 的同名欄位共用同一個哨兵值。 */
#define TELEM_DEPLOY_ALT_NA  ((int16_t)-32768)

/* 下行遙測封包（packed，固定長度）。欄位順序即為地面端解碼契約。 */
typedef struct __attribute__((packed)) {
    uint8_t  sync0;          /* 0xA5 同步字 */
    uint8_t  sync1;          /* 0x5A 同步字 */
    uint8_t  seq;            /* 遞增序號（自動 wrap），地面端偵測丟包 */
    uint8_t  fsm_state;      /* FlightState_t 飛行狀態機代碼 */
    uint32_t tick_ms;        /* HAL_GetTick() 系統毫秒 */

    int32_t  ekf_pos_z_cm;   /* EKF 高度 (cm) */
    int32_t  ekf_vel_z_cms;  /* EKF 垂直速度 (cm/s) */
    int16_t  ekf_q0;         /* 四元數 qw ×10000 */
    int16_t  ekf_q1;         /* qx ×10000 */
    int16_t  ekf_q2;         /* qy ×10000 */
    int16_t  ekf_q3;         /* qz ×10000 */

    int32_t  baro_alt_cm;    /* BMP388 海拔 (cm) */

    int16_t  imu_ax_mg;      /* BMI088 加速度 X (mg, sensor frame) */
    int16_t  imu_ay_mg;      /* BMI088 加速度 Y (mg) */
    int16_t  imu_az_mg;      /* BMI088 加速度 Z (mg) */
    int16_t  gyro_x_dps;     /* BMI088 角速度 X (dps) */
    int16_t  gyro_y_dps;     /* BMI088 角速度 Y (dps) */
    int16_t  gyro_z_dps;     /* BMI088 角速度 Z (dps) */
    /* ADXL375 高G「模長」(cg = 0.01g)。★2026-07-30 由三軸縮成單一模長：本板 ADXL375
     * 三軸實測雜訊全爆（Z σ645mG），FSM 起飛/燒完判定早已改吃 BMI088，下鏈保留三軸
     * 只是浪費 4 bytes 空中時間；留模長供「高G 是否仍在動/是否過載」的粗判即可。
     * 完整三軸原始值仍完整寫進 Flash ring（FlashRingPacket_t adxl_x/y/z）。 */
    int16_t  hg_mag_cg;

    int32_t  gps_lat_1e6;    /* 緯度 deg ×1e6（+北/−南） */
    int32_t  gps_lon_1e6;    /* 經度 deg ×1e6（+東/−西） */
    int16_t  gps_alt_m;      /* GPS 海拔 (m) */
    uint8_t  gps_sats;       /* 可見衛星數 */
    uint8_t  gps_fix;        /* 定位有效 (0/1) */

    uint16_t bat_mv;         /* 電池電壓 (mV) */
    uint16_t cpu_main_x10;   /* MainTask+ISR CPU 佔用率 (% ×10) */
    uint8_t  flags;          /* TELEM_FLAG_* 位元旗標 */

    uint8_t  health_bits;    /* P1：EKF_HB_*（ekf_guard.h；0=EKF 全健康） */
    uint8_t  sensor_bits;    /* P1：SH_BIT_*（sensor_health.h；0=感測器全健康） */

    /* --- 本板垂直濾波器 (VF, vertical_filter.h)：與 EKF 並列輸出，供地面站
     *     同屏比對「EKF vs VF」估計差異（VF 為開傘決策實際採用之估計器）。 --- */
    int32_t  vf_pos_z_cm;    /* VF 高度 (cm) */
    int32_t  vf_vel_z_cms;   /* VF 垂直速度 (cm/s) */

    /* --- ★飛行滾動極值（2026-07-30 新增）------------------------------------
     * 火箭端以「感測器全速率」持續更新的最大值，每一包都重複攜帶。
     * 存在理由：433 下鏈實際只有約 2Hz（103B @ 2.4k 空中速率 ≈ 350ms/包），而推力段
     * 只有 2~3 秒、峰值 G 不到 1 秒、頂點是一個瞬間——用 2Hz 取樣去記錄這些，最大高度
     * 與最大加速度會系統性偏低。這三個欄位由飛控迴圈以 100Hz(高度/速度) 與 400Hz 逐筆
     * 掃批次(加速度) 追蹤，因此：
     *   ① 不受下鏈取樣率限制，是真峰值；
     *   ② 抗丟包——每包都帶滾動值，頂點後任何一包穿透就拿得到，不必剛好收到峰值那包；
     *   ③ 火箭無法回收時，這是唯一能知道「到底飛多高、多快、承受多少 G」的來源。
     * 於 ARM（STATE_PAD → PAD_ARMED）歸零，避免地面測試/搬動污染。
     * ⚠ 飛行中熱重啟會一併歸零（本版未做跨重啟保存），此時 TELEM_FLAG_HOTSTART 會設起來，
     *   判讀時需注意極值只涵蓋最後一次重啟之後。 */
    uint16_t max_alt_m;      /* 起飛後最大相對高度 (m，0..65535；來源同 FSM 決策用 h_est) */
    int16_t  max_vel_ms;     /* 起飛後最大垂直速度 (m/s，來源同 FSM 決策用 v_est) */
    uint16_t max_acc_cg;     /* 起飛後最大合加速度 |a| (cg = 0.01g)；來源 BMI088 逐筆批次掃描。
                              * ⚠ BMI088 量程 ±24g，真實峰值超過會被硬體削頂（讀到 ~2400cg
                              * 就代表「至少 24g」而非精確值）。ADXL375 因本板雜訊問題不採用。 */

    /* --- ★實際開傘高度（2026-07-30 新增，各 2B）---------------------------------
     * 回答「傘到底是在對的高度開的嗎」——不必回收火箭就能判定。上面的 max_alt_m 只給
     * 頂點，開傘時機是另一回事：下鏈 ~2.3Hz，開傘那一瞬間的封包很可能剛好丟掉，事後
     * 從稀疏取樣反推不出開傘高度。故於開傘動作發生的那個飛控週期就地鎖存（100Hz 解析度），
     * 之後每包重複攜帶、且跨熱重啟保存（見 w25qxx.h FlashRingPacket_t 同名欄位）。
     * 來源與 max_alt_m 相同（in.h_est，FSM 決策實際採用的估計值），故可直接互相比較。
     * 未開傘 = TELEM_DEPLOY_ALT_NA；用哨兵而非 0，因為 0 m 是合法的開傘高度（地面誤觸發）。 */
    int16_t  drogue_alt_m;   /* 副傘實際開傘相對高度 (m)；未開傘 = TELEM_DEPLOY_ALT_NA */
    int16_t  main_alt_m;     /* 主傘實際開傘相對高度 (m)；未開傘 = TELEM_DEPLOY_ALT_NA */

    /* --- 主/副協同：主板中繼「對端(副板)摘要」，供地面站雙板監看（Phase A/B/C）。
     *     來源為板間鏈路 LinkPeer_t；無對端時全 0、peer_link=0。
     *     ★2026-07-30：拿掉對端 EKF 高度/速度、高G、丟包率——地面站顯示一律改看 VF
     *     （開傘決策實際採用的估計器，見上方本板 vf_pos_z_cm 註解），EKF/高G/丟包率
     *     只留給板間鏈路本身的 LinkPeer_t／USB 直連診斷（main.c [LINK] 行）用，
     *     不必再佔下行封包位元組。 --- */
    uint8_t  peer_fsm_state; /* 對端 FlightState_t 飛行狀態碼 */
    uint8_t  peer_flags;     /* 對端 TELEM_FLAG_* 子集。★bit0 DROGUE_FIRED / bit1 MAIN_DEPLOYED
                              * 為「開過」鎖存值（Telemetry_Build 由 LinkPeer 的 drogue_latched/
                              * main_latched OR 進來），不是對端此刻的腳位現況——副板引傘只導通
                              * 3s，即時值在 ~2Hz 下鏈幾乎必漏。其餘位仍為即時值。 */
    int32_t  peer_baro_cm;   /* 對端 baro 相對高度 (cm) */
    uint8_t  peer_link;      /* TELEM_PEER_* 鏈路健康位 */
    int32_t  peer_vf_h_cm;   /* 對端 VF 高度 (cm) */
    int32_t  peer_vf_v_cms;  /* 對端 VF 垂直速度 (cm/s) */

    uint8_t  arm_flags;      /* TELEM_ARM_* 位元（ARM 被擋下的原因，供地面站顯示） */

    /* 對端(副板) 主傘共開 / BENCH 桌測狀態（servo_arb.h SERVO_ARB_MSG_*：
     * 0=NONE 1=BENCH_START 2=LEGACY_DRIVING(已廢除) 3=DONE 4=BENCH_PRI_FIRE
     * 5=BENCH_SEC_FIRE 6=BENCH_MAIN_HIGH 7=MAIN_HIGH）。★主傘互斥握手已取消，
     * 改為兩板同時把 PD14 純 GPIO 拉高 SERVO_MAIN_HIGH_MS。
     * 讓地面站不必接對端板 USB 也能經 LoRa 看到副板開傘進度（見 Link_BuildOwnStatus）。 */
    uint8_t  peer_bench_arb;

    uint8_t  profile_flags;  /* TELEM_PROFILE_*：本板/對端是否仍為電梯測試 profile */

    uint16_t crc16;          /* CRC-16/CCITT-FALSE，覆蓋本封包前面所有位元組 */
} TelemetryPacket_t;

/* 封包固定長度（bytes） */
#define TELEM_PACKET_SIZE  ((uint16_t)sizeof(TelemetryPacket_t))

/**
 * @brief CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF, 無反射, xorout=0x0000。
 *        與地面站解碼器須採同一參數。
 */
uint16_t telem_crc16(const uint8_t *data, uint16_t len);

/**
 * @brief 由現有感測器 / EKF / 系統全域打包一筆遙測。
 * @param out 輸出緩衝區，須至少 TELEM_PACKET_SIZE bytes。
 * @return 寫入的封包長度 (bytes)。自動填入遞增 seq 與結尾 CRC16。
 * @note  於 task context 呼叫（讀取無鎖全域，可容忍極輕微 tearing，與診斷任務同策略）。
 */
uint16_t Telemetry_Build(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H */
