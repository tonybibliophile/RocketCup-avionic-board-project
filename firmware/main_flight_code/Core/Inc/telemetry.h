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
    uint32_t baro_press_pa;  /* BMP388 氣壓 (Pa) */

    int16_t  imu_ax_mg;      /* BMI088 加速度 X (mg, sensor frame) */
    int16_t  imu_ay_mg;      /* BMI088 加速度 Y (mg) */
    int16_t  imu_az_mg;      /* BMI088 加速度 Z (mg) */
    int16_t  gyro_x_dps;     /* BMI088 角速度 X (dps) */
    int16_t  gyro_y_dps;     /* BMI088 角速度 Y (dps) */
    int16_t  gyro_z_dps;     /* BMI088 角速度 Z (dps) */
    int16_t  hg_ax_cg;       /* ADXL375 高G X (cg=0.01g, sensor frame) */
    int16_t  hg_ay_cg;       /* ADXL375 高G Y (cg) */
    int16_t  hg_az_cg;       /* ADXL375 高G Z (cg) */
    int16_t  mag_x_mg;       /* MMC5983 磁場 X (mGauss, body frame) */
    int16_t  mag_y_mg;       /* MMC5983 磁場 Y (mGauss) */
    int16_t  mag_z_mg;       /* MMC5983 磁場 Z (mGauss) */

    int32_t  gps_lat_1e6;    /* 緯度 deg ×1e6（+北/−南） */
    int32_t  gps_lon_1e6;    /* 經度 deg ×1e6（+東/−西） */
    int16_t  gps_alt_m;      /* GPS 海拔 (m) */
    uint8_t  gps_sats;       /* 可見衛星數 */
    uint8_t  gps_fix;        /* 定位有效 (0/1) */

    uint16_t bat_mv;         /* 電池電壓 (mV) */
    uint16_t cpu_main_x10;   /* MainTask+ISR CPU 佔用率 (% ×10) */
    uint16_t cpu_ekf_x10;    /* EKFTask CPU 佔用率 (% ×10) */
    uint8_t  flags;          /* TELEM_FLAG_* 位元旗標 */

    uint8_t  health_bits;    /* P1：EKF_HB_*（ekf_guard.h；0=EKF 全健康） */
    uint8_t  sensor_bits;    /* P1：SH_BIT_*（sensor_health.h；0=感測器全健康） */

    /* --- 本板垂直濾波器 (VF, vertical_filter.h)：與 EKF 並列輸出，供地面站
     *     同屏比對「EKF vs VF」估計差異（VF 為開傘決策實際採用之估計器）。 --- */
    int32_t  vf_pos_z_cm;    /* VF 高度 (cm) */
    int32_t  vf_vel_z_cms;   /* VF 垂直速度 (cm/s) */

    /* --- 主/副協同：主板中繼「對端(副板)摘要」，供地面站雙板監看（Phase A/B/C）。
     *     來源為板間鏈路 LinkPeer_t；無對端時全 0、peer_link=0。 --- */
    uint8_t  peer_fsm_state; /* 對端 FlightState_t 飛行狀態碼 */
    uint8_t  peer_flags;     /* 對端 TELEM_FLAG_* 子集 */
    int32_t  peer_h_cm;      /* 對端 EKF 高度 (cm) */
    int32_t  peer_v_cms;     /* 對端 EKF 垂直速度 (cm/s) */
    int32_t  peer_baro_cm;   /* 對端 baro 相對高度 (cm) */
    uint8_t  peer_link;      /* TELEM_PEER_* 鏈路健康位 */
    uint16_t peer_loss_pmil; /* 對端封包丟失率 (‰，0..1000) */
    int16_t  peer_az_cg;     /* 對端 高G 垂直加速度 (cg = 0.01g) */
    int32_t  peer_vf_h_cm;   /* 對端 VF 高度 (cm) */
    int32_t  peer_vf_v_cms;  /* 對端 VF 垂直速度 (cm/s) */

    uint8_t  arm_flags;      /* TELEM_ARM_* 位元（ARM 被擋下的原因，供地面站顯示） */

    /* 對端(副板) D2 主傘舵機/BENCH 桌測握手狀態（servo_arb.h SERVO_ARB_MSG_*：
     * 0=NONE 1=INTENT 2=DRIVING 3=DONE 4=BENCH_PRI_FIRE 5=BENCH_SEC_FIRE 6=BENCH_BOTH_FIRE）。
     * 讓地面站不必接對端板 USB 也能經 LoRa 看到副板舵機驅動進度（見 Link_BuildOwnStatus）。 */
    uint8_t  peer_bench_arb;

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
