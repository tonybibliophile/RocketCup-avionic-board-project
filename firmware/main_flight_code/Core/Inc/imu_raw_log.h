/*
 * imu_raw_log.h — SD 卡原始 IMU 記錄封包格式（S6，header-only，比照 flash_ring_math.h）
 * ===========================================================================
 * 目的：EKF 實際吃的是陀螺 1000Hz / 加速度 400Hz 原始樣本，但既有 HIL CSV
 * 只存每批最後 1 筆（100Hz，且是重映射過的浮點值），事後離線重跑 EKF/VF 時
 * 拿不到當下真正餵給濾波器的完整輸入。本檔定義一份與 CSV 並行的二進位記錄
 * （IMU_xxx.BIN），逐顆保存原始 LSB（remap 前、sensor frame），使地面工具
 * 能忠實重建飛行當下的積分輸入。
 *
 * 128 bytes packed，比照 FlashRingPacket_t（w25qxx.h）慣例：2-byte magic +
 * 滾動 seq + 尾端 CRC-16/CCITT-FALSE（crc16.h，覆蓋 [0..125]）。固定長度
 * 使第 N 筆位於檔案 offset N×128，離線解碼器可直接 seek 與重新同步。
 *
 * 純 struct 定義 + 位元旗標常數；實際填值/CRC 計算/寫檔在 main.c（比照
 * flash_ring 的 ring_crc16 定義在 w25qxx.c 而非其標頭的慣例）。
 * Python 對應格式字串見 ground_station/tools/imu_raw_decoder.py，兩邊手動同步
 * （該檔頂部註解會指回本檔）。
 */
#ifndef IMU_RAW_LOG_H
#define IMU_RAW_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_RAW_LOG_MAGIC0   'R'
#define IMU_RAW_LOG_MAGIC1   'I'
#define IMU_RAW_LOG_VERSION  1U

#define IMU_RAW_LOG_N_GYRO   10U   /* 對應 EKF_GYRO_PER_FRAME（ekf.h），每筆記錄涵蓋一個 10ms frame */
#define IMU_RAW_LOG_N_ACC    4U    /* 對應 EKF_ACC_PER_FRAME（ekf.h） */

/* flags 位元定義（已實作：BUF_DROP / BARO_NEW / EKF_QUEUE_DROP；GYRO_BATCH_OVR /
 * ACC_BATCH_OVR 保留給未來需要時用——真正偵測 ping-pong 批次被覆寫需在 ISR 內加計數，
 * 不在本次 S6 範圍，目前這兩位恆為 0，勿依賴其為「已驗證無覆寫」的證據）。 */
#define IMU_RAW_LOG_FLAG_BUF_DROP     0x01U  /* 上一筆記錄因 SRAM 緩衝已滿被丟棄（本筆之前有缺口） */
#define IMU_RAW_LOG_FLAG_GYRO_BATCH_OVR 0x02U /* 保留，未實作，恆 0 */
#define IMU_RAW_LOG_FLAG_ACC_BATCH_OVR  0x04U /* 保留，未實作，恆 0 */
#define IMU_RAW_LOG_FLAG_BARO_NEW       0x08U /* 本筆 baro_press_pa 為真新樣本（非 0 值本身已隱含，此位供快速篩選） */
#define IMU_RAW_LOG_FLAG_EKF_QUEUE_DROP 0x10U /* 本 frame 送出的 EKF buffer 造成 g_ekf_queue_drops 遞增 */

typedef struct __attribute__((packed)) {
    uint8_t  magic[2];        /* [0..1]   'R','I' */
    uint8_t  ver;              /* [2]      IMU_RAW_LOG_VERSION */
    uint8_t  n_gyro;            /* [3]      本筆陀螺樣本數，恆 IMU_RAW_LOG_N_GYRO */
    uint8_t  n_acc;               /* [4]      本筆加速度樣本數，恆 IMU_RAW_LOG_N_ACC */
    uint8_t  flags;                /* [5]      IMU_RAW_LOG_FLAG_* 位元旗標 */
    uint16_t seq;                   /* [6..7]   滾動序號（wrap） */
    uint32_t t_cyc;                  /* [8..11]  gyro[0] 樣本的 DWT->CYCCNT（與 EKF_Sample_t.timestamp_cyc 同源) */
    uint8_t  fsm_state;               /* [12]     FlightState_t */
    uint8_t  _rsv;                     /* [13]     保留對齊 */
    int16_t  baro_temp_c_x100;          /* [14..15] 氣壓計溫度 ×100（無新樣本時延續上次值） */
    uint32_t baro_press_pa;              /* [16..19] 氣壓 Pa；0 = 本 frame 無新 baro（S4 drdy 閘控） */
    int16_t  gyro[10][3];                 /* [20..79]  陀螺 X/Y/Z 原始 LSB，sensor frame（remap 前）*/
    int16_t  acc[4][3];                    /* [80..103] 加速度 X/Y/Z 原始 LSB，sensor frame */
    uint16_t gyro_dt_q[10];                 /* [104..123] 每顆陀螺樣本距前一顆的 cycles/8（uint16 夠涵蓋
                                              * ~1ms 間隔：168000cyc/8=21000 < 65536）；[0] 為距上一個
                                              * frame 最後一顆的間隔，保留跨 frame 的真實 ISR 抖動；
                                              * seq==0（第一筆記錄）之 [0] 無上一顆可比較，值不具意義 */
    int16_t  acc_t_off_q;                    /* [124..125] acc[0] 樣本時戳相對 t_cyc 的偏移，cycles/8，
                                               * 可正可負（TIM6/TIM7 為獨立 ISR，無相位鎖定）；
                                               * ±32767×8 cycles ≈ ±1.56ms @168MHz，覆蓋實際偏移綽綽有餘 */
    uint16_t crc16;                           /* [126..127] CRC-16/CCITT-FALSE，覆蓋 [0..125] */
} ImuRawRecord_t;

_Static_assert(sizeof(ImuRawRecord_t) == 128, "ImuRawRecord_t must be 128 bytes (SD offset = seq * 128)");

#ifdef __cplusplus
}
#endif

#endif /* IMU_RAW_LOG_H */
