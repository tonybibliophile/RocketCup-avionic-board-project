#ifndef CORE_INC_SENSOR_AXIS_H_
#define CORE_INC_SENSOR_AXIS_H_

/*
 * sensor_axis.h — 感測器原始軸 → body(real) frame 軸向映射
 * ============================================================
 * 這是「軸向映射的唯一真相來源」。main.c 內所有重映射都必須呼叫這裡，
 * 禁止再於別處手寫 `ax = -ay;` 之類的散裝映射 —— 那是之前每次結果不一致的根源。
 *
 * Body/real frame 定義（使用者規格）：
 *     X = 向右 (right)
 *     Y = 向前 (forward)
 *     Z = 朝上 (up)
 *   為右手系：X × Y = Z。
 *
 * 映射表（感測器軸 -> 實際軸）：
 *     IMU    (BMI088 accel & gyro):  X->Y,  Y->X,  Z->-Z
 *     High-G (ADXL375):              X->-X, Y->-Y, Z-> Z
 *     Mag    (MMC5983):              X->-Y, Y-> X,  Z->-Z
 *
 * 記號「sensor S -> real ±R」表示感測器 S 軸實體指向 real ±R 方向，
 * 故 real_R = ±sensor_S。展開為各 body 分量：
 *     IMU :  bx = +sy,  by = +sx,  bz = -sz
 *     HiG :  bx = -sx,  by = -sy,  bz = +sz
 *     Mag :  bx = +sy,  by = -sx,  bz = -sz
 *
 * IMU 與 High-G 映射為 proper rotation (行列式 det = +1)，磁力計晶片原生為左手系 (det = -1)，因此：
 *   - accel 與 gyro 用同一映射即自洽（陀螺為 axial vector，在 det=+1 下與一般向量同變換）；
 *   - 可安全餵入四元數姿態積分。
 * （獨立驗證見 tests/test_sensor_axis.c —— 純 host 編譯，不需硬體。）
 *
 * 本檔為純函式、無任何硬體 / HAL / FreeRTOS 相依，可被 firmware 與 host 測試同時 include。
 */

/* IMU (BMI088) 加速度計 / 陀螺儀：bx=+sy, by=+sx, bz=-sz
 * 輸入輸出可為同一變數（先以傳值複製 sx/sy/sz，再寫出，故 alias 安全）。 */
static inline void sensor_imu_to_body(float sx, float sy, float sz,
                                      float *bx, float *by, float *bz)
{
    *bx =  sy;   /* real X(右)  = +sensor_Y */
    *by =  sx;   /* real Y(前)  = +sensor_X */
    *bz = -sz;   /* real Z(上)  = -sensor_Z */
}

/* High-G (ADXL375)：bx=-sx, by=-sy, bz=+sz */
static inline void sensor_highg_to_body(float sx, float sy, float sz,
                                        float *bx, float *by, float *bz)
{
    *bx = -sx;   /* real X(右)  = -sensor_X */
    *by = -sy;   /* real Y(前)  = -sensor_Y */
    *bz =  sz;   /* real Z(上)  = +sensor_Z */
}

/* Magnetometer (MMC5983)：bx=+sy, by=-sx, bz=-sz（det=-1，晶片原生左手系）
 * ⚠ 符號判定注意：驅動 CTRL2 曾誤寫 0x15（Cmm_en 位元錯）導致連續模式未開，
 *   讀值凍結且停在 Recalibrate 的 RESET「負极性」量測（Output = −H + Offset）。
 *   在那段期間 bench 看到的磁場方向全部反相——據此推的任何符號翻轉都不可信。
 *   以「凍結值 = −H」還原 7/15 鼻錐朝北快照（顯示 my=−333 ⇒ 真實 H_chipX=−333）
 *   反而支持本組原始映射 by=−sx。CMM 修復（0x0D）後請以 GUI 測試 8/9 用
 *   「活數據」最終定案：若航向仍差 180°，屆時翻 by 與 bz 即可（det 仍 −1）。 */
static inline void sensor_mag_to_body(float sx, float sy, float sz,
                                      float *bx, float *by, float *bz)
{
    *bx =  sy;   /* real X(右)  = +sensor_Y */
    *by = -sx;   /* real Y(前)  = -sensor_X */
    *bz = -sz;   /* real Z(上)  = -sensor_Z */
}

#endif /* CORE_INC_SENSOR_AXIS_H_ */
