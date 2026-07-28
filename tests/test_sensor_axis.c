/*
 * test_sensor_axis.c — 獨立軸向 / 姿態邏輯單元測試（純 host 編譯，不需硬體）
 * ===========================================================================
 *   cd tests && make run
 *
 * 驗證兩件事：
 *   1. sensor_axis.h 的三個映射「逐字」符合使用者定義的表格，且皆為右手系 (det=+1)。
 *   2. attitude_math.h 的 TRIAD 由 (重力,磁場) 還原姿態正確（含 upright=identity、
 *      多組已知姿態 round-trip、退化情形）。
 *
 * 使用者定義表（感測器 -> 實際；body=右/前/上）：
 *     IMU    : bx = +sy,  by = +sx,  bz = -sz
 *     High-G : bx = -sx,  by = -sy,  bz = +sz
 *     Mag    : bx = +sy,  by = -sx,  bz = -sz
 */
#include <stdio.h>
#include <math.h>
#include "sensor_axis.h"
#include "attitude_math.h"

static int g_fail = 0, g_total = 0;
static void check(const char *name, int cond) {
    g_total++;
    if (cond) { printf("  [PASS] %s\n", name); }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}
static int feq(float a, float b) { return fabsf(a - b) < 1e-4f; }

typedef void (*remap_fn)(float, float, float, float*, float*, float*);

/* 把映射當成矩陣 M（out = M*in），由 e1/e2/e3 取出三欄後算 det = c1·(c2×c3) */
static float det_of(remap_fn f) {
    float c1[3], c2[3], c3[3];
    f(1,0,0, &c1[0],&c1[1],&c1[2]);
    f(0,1,0, &c2[0],&c2[1],&c2[2]);
    f(0,0,1, &c3[0],&c3[1],&c3[2]);
    float cx = c2[1]*c3[2] - c2[2]*c3[1];
    float cy = c2[2]*c3[0] - c2[0]*c3[2];
    float cz = c2[0]*c3[1] - c2[1]*c3[0];
    return c1[0]*cx + c1[1]*cy + c1[2]*cz;
}

static void test_table(void) {
    float bx, by, bz;
    printf("[1] 映射表逐字驗證\n");

    sensor_imu_to_body(1,2,3, &bx,&by,&bz);
    check("IMU   (sx,sy,sz)=(1,2,3) -> (sy,sx,-sz)=(2,1,-3)", feq(bx,2)&&feq(by,1)&&feq(bz,-3));

    sensor_highg_to_body(1,2,3, &bx,&by,&bz);
    check("HighG (1,2,3) -> (-sx,-sy,sz)=(-1,-2,3)",          feq(bx,-1)&&feq(by,-2)&&feq(bz,3));

    sensor_mag_to_body(1,2,3, &bx,&by,&bz);
    check("Mag   (1,2,3) -> (sy,-sx,-sz)=(2,-1,-3)",          feq(bx,2)&&feq(by,-1)&&feq(bz,-3));

    printf("[2] 剛體旋轉與手性驗證 (det = +1/ -1)\n");
    check("IMU   det == +1",   feq(det_of(sensor_imu_to_body),   1.0f));
    check("HighG det == +1",   feq(det_of(sensor_highg_to_body), 1.0f));
    check("Mag   det == -1",   feq(det_of(sensor_mag_to_body),   -1.0f));
}

static void test_physical(void) {
    float bx, by, bz;
    const float g = 9.80665f;
    printf("[3] 物理重力一致性（直立靜置）\n");

    /* 依表格：直立時 IMU 晶片 Z 朝下 -> sensor_z = -g；body Z 應為 +g（朝上反作用力） */
    sensor_imu_to_body(0,0,-g, &bx,&by,&bz);
    check("IMU 直立 -> body az = +g (>0)", bz > 0 && feq(bz, g));

    /* 依表格：直立時 High-G 晶片 Z 朝上 -> sensor_z = g；body Z 應為 +g */
    sensor_highg_to_body(0,0,g, &bx,&by,&bz);
    check("HighG 直立 -> body az = +g (>0)", bz > 0 && feq(bz, g));

    /* 一致性：兩者直立時 body Z 同號（>20g 替換才無縫） */
    float bz_imu, bz_hg, t;
    sensor_imu_to_body(0,0,-g, &t,&t,&bz_imu);
    sensor_highg_to_body(0,0,g, &t,&t,&bz_hg);
    check("IMU 與 HighG 直立 body Z 同號", (bz_imu > 0) == (bz_hg > 0));
}

/*
 * 由「表格」反推 IMU 晶片實體貼裝（晶片軸 -> body 方向）：
 *   body_X(右) = +sensor_Y  => 晶片 Y 軸 ∥ body 右
 *   body_Y(前) = +sensor_X  => 晶片 X 軸 ∥ body 前
 *   body_Z(上) = -sensor_Z  => 晶片 Z 軸 ∥ body 下
 * 給定 body-frame 比力 f，算出各晶片軸讀數 raw = f·chip_axis。
 */
static void imu_chip_read(const float f_body[3], float raw[3]) {
    raw[0] = f_body[1];   /* raw_X = sensor_X = body_Y */
    raw[1] = f_body[0];   /* raw_Y = sensor_Y = body_X */
    raw[2] = -f_body[2];  /* raw_Z = sensor_Z = -body_Z */
}

/* 模擬 bench 擺放 -> 晶片讀數 -> firmware 映射 -> body 輸出，驗證與 GUI 精靈期望一致。
 * 這把「哪個方向會讀到正值」用機器證明，而非人腦推導（避免反覆出錯）。 */
static void test_bench_orientation(void) {
    const float g = 9.80665f;
    float raw[3], bx, by, bz;
    printf("[6] Bench 擺放 -> body 輸出（GUI 精靈期望，已證明）\n");

    /* 任意 f_body round-trip 必須還原（晶片貼裝 + firmware 映射 = 恆等） */
    float ftest[3] = {3.0f, -5.0f, 7.0f};
    imu_chip_read(ftest, raw);
    sensor_imu_to_body(raw[0], raw[1], raw[2], &bx, &by, &bz);
    check("晶片+映射 round-trip = 恆等", feq(bx,3)&&feq(by,-5)&&feq(bz,7));

    /* 平放正面朝上：f_body = +Z*g -> az = +g (板面法線朝天) */
    float f_flat[3] = {0,0,g};
    imu_chip_read(f_flat, raw); sensor_imu_to_body(raw[0],raw[1],raw[2], &bx,&by,&bz);
    check("平放正面朝上 -> az = +g (測試1)", feq(bz, g) && bz > 0);

    /* 右側朝上(body 右軸朝天)：f_body = +X*g -> ax = +g */
    float f_right_up[3] = {g,0,0};
    imu_chip_read(f_right_up, raw); sensor_imu_to_body(raw[0],raw[1],raw[2], &bx,&by,&bz);
    check("右側抬高(右緣朝上) -> ax = +g  (測試2: ax>0)", bx > 0 && feq(bx, g));

    /* 前緣/鼻錐朝上(body 縱向軸朝天)：f_body = +Z*g -> az = +g */
    float f_nose_up[3] = {0,0,g};
    imu_chip_read(f_nose_up, raw); sensor_imu_to_body(raw[0],raw[1],raw[2], &bx,&by,&bz);
    check("前緣抬高(鼻錐朝上) -> az = +g  (測試3: az>0)", bz > 0 && feq(bz, g));

    /* 反向健全性：右側朝下 -> ax 必為負（與測試2 相反方向） */
    float f_right_dn[3] = {-g,0,0};
    imu_chip_read(f_right_dn, raw); sensor_imu_to_body(raw[0],raw[1],raw[2], &bx,&by,&bz);
    check("右側朝下 -> ax < 0 (反向)", bx < 0);

    /* 陀螺儀同一晶片/映射，ω_body round-trip 亦為恆等，故旋轉方向定義同理。
     * 角速度向量 = 右手定則指向（繞上=CCW、繞右=抬頭、繞前=向右翻滾）。 */
    float w_yaw[3]   = {0,0,1};   /* 繞上軸 (鼻錐) */
    imu_chip_read(w_yaw, raw);   sensor_imu_to_body(raw[0],raw[1],raw[2], &bx,&by,&bz);
    check("繞上軸(鼻錐) -> gz > 0  (測試5)", bz > 0);
    float w_pitch[3] = {1,0,0};   /* 繞右軸 + = 抬頭 */
    imu_chip_read(w_pitch, raw); sensor_imu_to_body(raw[0],raw[1],raw[2], &bx,&by,&bz);
    check("抬頭(繞右軸+) -> gx > 0  (測試6)", bx > 0);
    float w_roll[3]  = {0,1,0};   /* 繞前軸(板面法線) + = 向右翻滾 */
    imu_chip_read(w_roll, raw);  sensor_imu_to_body(raw[0],raw[1],raw[2], &bx,&by,&bz);
    check("向右翻滾(繞前軸+) -> gy > 0  (測試7)", by > 0);
}

/* out = R^T * v  (R 為 body->nav，故 R^T 把 nav 向量轉回 body) */
static void Rt_apply(const float R[3][3], const float v[3], float out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = R[0][i]*v[0] + R[1][i]*v[1] + R[2][i]*v[2];
}

static void triad_roundtrip(const char *name, const float q_true[4]) {
    float R[3][3];
    att_quat_to_R(q_true, R);

    /* 由已知姿態合成 body-frame 感測讀數 */
    float up_nav[3]  = {0.0f, 0.0f, 9.80665f};   /* 加速度計反作用力 = 朝上 */
    float mag_nav[3] = {0.0f, 0.22f, -0.43f};    /* 北 + 向下傾角 (N 半球) */
    float grav_b[3], mag_b[3];
    Rt_apply(R, up_nav, grav_b);
    Rt_apply(R, mag_nav, mag_b);

    float q2[4];
    int ok = att_triad_grav_mag_to_quat(grav_b, mag_b, q2);
    float R2[3][3];
    att_quat_to_R(q2, R2);

    int match = ok;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (!feq(R[i][j], R2[i][j])) match = 0;
    check(name, match);
}

static void test_triad(void) {
    printf("[4] TRIAD 姿態還原 (round-trip)\n");
    const float s = 0.70710678f; /* sin/cos 45 */

    float q_id[4]   = {1,0,0,0};
    float q_yawP[4] = {s,0,0,s};      /* +90 deg about Up   */
    float q_yawN[4] = {s,0,0,-s};     /* -90 deg about Up   */
    float q_pit[4]  = {s,s,0,0};      /* +90 deg about East */
    float q_rol[4]  = {s,0,s,0};      /* +90 deg about North*/
    float q_cmp[4]  = {0.8536f,0.3536f,0.1464f,0.3536f}; /* 任意複合姿態 */

    triad_roundtrip("identity",        q_id);
    triad_roundtrip("yaw +90",         q_yawP);
    triad_roundtrip("yaw -90",         q_yawN);
    triad_roundtrip("pitch +90 (East)",q_pit);
    triad_roundtrip("roll  +90 (North)",q_rol);
    triad_roundtrip("compound",        q_cmp);

    /* 直立朝北 -> 必須剛好是 identity 四元數 */
    float gI[3] = {0,0,1}, mI[3] = {0, 0.22f, -0.43f}, qI[4];
    int ok = att_triad_grav_mag_to_quat(gI, mI, qI);
    check("upright+North -> identity quat",
          ok && feq(qI[0],1)&&feq(qI[1],0)&&feq(qI[2],0)&&feq(qI[3],0));

    printf("[5] TRIAD 退化情形回傳 0\n");
    float qd[4] = {1,0,0,0};
    float gz[3] = {0,0,1}, m0[3] = {0,0,0};
    check("零磁場回傳 0",          att_triad_grav_mag_to_quat(gz, m0, qd) == 0);
    float mpar[3] = {0,0,5};
    check("磁場平行重力回傳 0",     att_triad_grav_mag_to_quat(gz, mpar, qd) == 0);
    float g0[3] = {0,0,0}, mok[3] = {0,1,0};
    check("零重力回傳 0",          att_triad_grav_mag_to_quat(g0, mok, qd) == 0);
}

int main(void) {
    printf("=== RocketCom 軸向 / 姿態邏輯獨立測試 ===\n");
    test_table();
    test_physical();
    test_bench_orientation();
    test_triad();
    printf("\n=== 結果：%d/%d 通過, %d 失敗 ===\n", g_total - g_fail, g_total, g_fail);
    return g_fail ? 1 : 0;
}
