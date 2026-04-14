#include "control.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jy61p_uart.h"
#include "supvc.h"
#include "usart.h"

extern int32_t Encoder_TIM1_Count;
extern int32_t Encoder_TIM3_Count;
extern int32_t Encoder_TIM4_Count;
extern int32_t Encoder_TIM5_Count;

/* 控制周期固定为 10ms，与 TIM6 中断频率一致。 */
#define CONTROL_DT_S                     0.01f
#define CONTROL_WHEEL_COUNT              4U
#define CONTROL_ULTRA_COUNT              SUPVC_CHANNEL_COUNT
#define CONTROL_CMD_MAX_TOKENS           10
#define CONTROL_CMD_LINE_LEN             96
#define CONTROL_DEG2RAD                  0.0174532925f
#define CONTROL_RAD2DEG                  57.2957795f
#define CONTROL_EPSILON                  0.0001f
#define CONTROL_LINE_SINGLE_LOSS_LIMIT   30U
#define CONTROL_TUNE_PERIOD_MS           120U
#define CONTROL_WHEEL_START_PWM_MIN      18.0f
#define CONTROL_WHEEL_START_SPEED_MIN    8.0f
#define CONTROL_TUNE_PERIOD_MS           120U
#define CONTROL_WHEEL_START_PWM_MIN      18.0f
#define CONTROL_WHEEL_START_SPEED_MIN    8.0f

/* 传感器通道布局映射（SUPVC 通道号从 0 开始）:
 * CH1(idx=0) = 前方(Front),  CH2(idx=1) = 后方(Rear)
 * CH3(idx=2) = 左前(LF),     CH4(idx=3) = 右前(RF)
 * CH5(idx=4) = 左后(LR),     CH6(idx=5) = 右后(RR) */
#define SENSOR_FRONT              0U   /* CH1: 前方 */
#define SENSOR_REAR               1U   /* CH2: 后方 */
#define SENSOR_LEFT_FRONT         2U   /* CH3: 左前 */
#define SENSOR_RIGHT_FRONT        3U   /* CH4: 右前 */
#define SENSOR_LEFT_REAR          4U   /* CH5: 左后 */
#define SENSOR_RIGHT_REAR         5U   /* CH6: 右后 */

/* 全局控制参数:
 * - 传感器归一化区间与滤波参数
 * - 速度/角速度限幅
 * - PID 初始参数
 * - 遥测开关 */
typedef struct {
    float min_raw[CONTROL_ULTRA_COUNT];
    float max_raw[CONTROL_ULTRA_COUNT];
    float iir_alpha[CONTROL_ULTRA_COUNT];

    float wheel_speed_lpf_alpha;
    float max_vx_mmps;
    float max_vy_mmps;
    float max_wz_dps;
    float max_wheel_mmps;
    float line_vy_limit_mmps;
    float line_forward_limit_mmps;
    float chassis_rot_radius_mm;

    float wheel_pid_kp;
    float wheel_pid_ki;
    float wheel_pid_kd;

    float line_pid_kp;
    float line_pid_ki;
    float line_pid_kd;

    float yaw_pid_kp;
    float yaw_pid_ki;
    float yaw_pid_kd;

    uint8_t telemetry_enabled;
} ControlConfig_t;

/* 单通道传感器运行状态:
 * raw           : 当前原始值（本工程对外统一用 SUPVC 的 raw）
 * filtered_raw  : 滤波后的值
 * normalized    : 归一化 0~100
 * history[]     : 3 点中值窗口 */
typedef struct {
    uint32_t raw;
    float filtered_raw;
    float normalized;
    uint8_t valid;
    uint8_t filter_initialized;
    uint32_t history[3];
    uint8_t history_count;
    uint8_t history_index;
} SensorChannelState_t;

/* 单轮速度闭环状态。 */
typedef struct {
    int32_t last_count;
    float measured_mmps;
    uint8_t initialized;
    PID_TypeDef pid;
} WheelControlState_t;

/* 校准采样状态机:
 * active        : 当前是否处于采样窗口
 * report_pending: 采样结束后，主循环发送结果 */
typedef struct {
    uint8_t active;
    uint8_t report_pending;
    uint8_t channel_mask;
    uint16_t target_samples;
    uint16_t collected_samples;
    uint32_t min_raw[CONTROL_ULTRA_COUNT];
    uint32_t max_raw[CONTROL_ULTRA_COUNT];
    uint64_t sum_raw[CONTROL_ULTRA_COUNT];
} CalibrationState_t;

/* FireWater 遥测快照（固定通道顺序）。 */
typedef struct {
    uint32_t mode_id;
    uint32_t time_ms;
    float left_raw;
    float right_raw;
    float front_raw;
    float left_norm;
    float right_norm;
    float line_error;
    float vx_meas;
    float vy_meas;
    float wz_meas;
    float yaw_deg;
} TelemetrySnapshot_t;

/* 控制主状态:
 * 统一聚合模式、指令、估计量、PID、校准和遥测缓存。 */
typedef struct {
    ControlMode_t mode;

    float cmd_vx_mmps;
    float cmd_vy_mmps;
    float cmd_wz_dps;

    float line_forward_mmps;
    float line_target;
    float line_error;
    float line_last_diff;
    uint8_t line_single_loss_ticks;

    /* 新增：状态误差解算相关 */
    float yaw_target;        /* 偏航角度目标（默认0，即与墙壁平行） */
    float lat_target;        /* 横向位置目标（默认0，即左右居中） */
    float yaw_error;         /* 偏航角度误差 = (LF-LR) - (RF-RR) */
    float lat_error;         /* 横向中心误差 = (LF+LR)/2 - (RF+RR)/2 */

    uint8_t yaw_hold_enabled;
    float yaw_target_deg;    /* IMU yaw 目标角度（兼容旧接口） */
    float yaw_meas_deg;

    float wheel_target_mmps[CONTROL_WHEEL_COUNT];
    float wheel_meas_mmps[CONTROL_WHEEL_COUNT];
    float wheel_pwm_cmd[CONTROL_WHEEL_COUNT];

    float vx_meas_mmps;
    float vy_meas_mmps;
    float wz_meas_dps;

    uint8_t encoder_initialized;

    SensorChannelState_t ultra[CONTROL_ULTRA_COUNT];
    WheelControlState_t wheel[CONTROL_WHEEL_COUNT];
    PID_TypeDef line_pid;    /* 兼容旧接口 */
    PID_TypeDef yaw_pid;     /* 偏航角度PID（控制W） */
    PID_TypeDef lat_pid;     /* 横向位置PID（控制Vy） */

    CalibrationState_t calibration;

    TelemetrySnapshot_t telemetry_snapshot;
    volatile uint8_t telemetry_pending;
} ControlState_t;

static ControlConfig_t g_cfg;
static ControlState_t g_state;

/* 基础限幅工具，避免各环节重复写边界判断。 */
static float clampf_local(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static uint32_t median3_u32(uint32_t a, uint32_t b, uint32_t c)
{
    if (a > b) {
        uint32_t t = a;
        a = b;
        b = t;
    }
    if (b > c) {
        uint32_t t = b;
        b = c;
        c = t;
    }
    if (a > b) {
        uint32_t t = a;
        a = b;
        b = t;
    }
    return b;
}

static uint8_t str_eq_ci(const char *a, const char *b)
{
    if ((a == NULL) || (b == NULL)) {
        return 0U;
    }

    while ((*a != '\0') && (*b != '\0')) {
        char ca = *a;
        char cb = *b;
        if ((ca >= 'a') && (ca <= 'z')) {
            ca = (char)(ca - 'a' + 'A');
        }
        if ((cb >= 'a') && (cb <= 'z')) {
            cb = (char)(cb - 'a' + 'A');
        }
        if (ca != cb) {
            return 0U;
        }
        a++;
        b++;
    }

    return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

static const char *mode_to_text(ControlMode_t mode)
{
    switch (mode) {
        case CONTROL_MODE_IDLE:
            return "IDLE";
        case CONTROL_MODE_MANUAL_VEL:
            return "MANUAL";
        case CONTROL_MODE_LINE_FOLLOW:
            return "LINE";
        case CONTROL_MODE_CALIBRATION:
            return "CAL";
        default:
            return "UNKNOWN";
    }
}

/* 将任意原始值映射到 0~100 归一化区间。
 * 注意: 每个通道可独立设置 min/max，便于现场标定后在线更新。 */
static float normalize_raw_to_0_100(uint8_t channel_index, float raw_value)
{
    float min_raw = g_cfg.min_raw[channel_index];
    float max_raw = g_cfg.max_raw[channel_index];

    if ((max_raw - min_raw) < 1.0f) {
        return 0.0f;
    }

    return clampf_local((raw_value - min_raw) * 100.0f / (max_raw - min_raw), 0.0f, 100.0f);
}

/* 运行态复位:
 * 仅清空控制状态，不重复初始化硬件外设。 */
static void reset_runtime_states(void)
{
    uint8_t i;

    g_state.mode = CONTROL_MODE_IDLE;
    g_state.cmd_vx_mmps = 0.0f;
    g_state.cmd_vy_mmps = 0.0f;
    g_state.cmd_wz_dps = 0.0f;
    g_state.line_forward_mmps = 0.0f;    /* 归中模式默认不前进 */
    g_state.line_target = 0.0f;
    g_state.line_error = 0.0f;
    g_state.line_last_diff = 0.0f;
    g_state.line_single_loss_ticks = 0U;

    /* 新增：状态误差解算相关初始化 */
    g_state.yaw_target = 0.0f;      /* 偏航角度目标：与墙壁平行 */
    g_state.lat_target = 0.0f;      /* 横向位置目标：左右居中 */
    g_state.yaw_error = 0.0f;
    g_state.lat_error = 0.0f;

    g_state.yaw_hold_enabled = 0U;
    g_state.yaw_target_deg = 0.0f;
    g_state.yaw_meas_deg = 0.0f;
    g_state.vx_meas_mmps = 0.0f;
    g_state.vy_meas_mmps = 0.0f;
    g_state.wz_meas_dps = 0.0f;
    g_state.encoder_initialized = 0U;
    g_state.telemetry_pending = 0U;

    PID_Reset(&g_state.line_pid);
    PID_Reset(&g_state.yaw_pid);
    PID_Reset(&g_state.lat_pid);

    for (i = 0U; i < CONTROL_ULTRA_COUNT; i++) {
        memset((void *)&g_state.ultra[i], 0, sizeof(g_state.ultra[i]));
    }

    for (i = 0U; i < CONTROL_WHEEL_COUNT; i++) {
        g_state.wheel_target_mmps[i] = 0.0f;
        g_state.wheel_meas_mmps[i] = 0.0f;
        g_state.wheel_pwm_cmd[i] = 0.0f;
        g_state.wheel[i].last_count = 0;
        g_state.wheel[i].measured_mmps = 0.0f;
        g_state.wheel[i].initialized = 0U;
        PID_Reset(&g_state.wheel[i].pid);
    }

    memset((void *)&g_state.calibration, 0, sizeof(g_state.calibration));
    memset((void *)&g_state.telemetry_snapshot, 0, sizeof(g_state.telemetry_snapshot));

    motor_Stop();
}

/* 参数默认值初始化。
 * 归一化区间设置：靠墙时 ≈ 20，离墙最远时 ≈ 80
 * raw 值直接使用 cm 单位距离
 * 根据实测数据校准（2026-04-07） */
static void init_default_config(void)
{
    /* VL53L0X 模块侧向安装深度不一，根据 22:17 实测数据校准：
     * - 左前(LF/CH3): 靠墙 ~12.3, 离墙 ~49.5
     * - 右前(RF/CH4): 靠墙 ~7.2,  离墙 ~44.2
     * - 左后(LR/CH5): 靠墙 ~7.8,  离墙 ~44.8
     * - 右后(RR/CH6): 靠墙 ~7.6,  离墙 ~44.3 */
    g_cfg.min_raw[SENSOR_LEFT_FRONT]  = 12.0f;
    g_cfg.max_raw[SENSOR_LEFT_FRONT]  = 50.0f;
    g_cfg.min_raw[SENSOR_RIGHT_FRONT] = 7.0f;
    g_cfg.max_raw[SENSOR_RIGHT_FRONT] = 45.0f;
    g_cfg.min_raw[SENSOR_LEFT_REAR]   = 7.5f;
    g_cfg.max_raw[SENSOR_LEFT_REAR]   = 46.0f;
    g_cfg.min_raw[SENSOR_RIGHT_REAR]  = 7.3f;
    g_cfg.max_raw[SENSOR_RIGHT_REAR]  = 45.0f;

    /* 前后传感器(CH1/CH2) 目前不参与寻迹闭环，设置宽泛范围 */
    g_cfg.min_raw[SENSOR_FRONT] = 2.0f;
    g_cfg.max_raw[SENSOR_FRONT] = 200.0f;
    g_cfg.min_raw[SENSOR_REAR]  = 2.0f;
    g_cfg.max_raw[SENSOR_REAR]  = 200.0f;

    g_cfg.iir_alpha[0] = 0.25f;
    g_cfg.iir_alpha[1] = 0.25f;
    g_cfg.iir_alpha[2] = 0.18f;
    g_cfg.iir_alpha[3] = 0.18f;
    g_cfg.iir_alpha[4] = 0.18f;
    g_cfg.iir_alpha[5] = 0.22f;

    g_cfg.wheel_speed_lpf_alpha = 0.45f;
    g_cfg.max_vx_mmps = 450.0f;
    g_cfg.max_vy_mmps = 450.0f;
    g_cfg.max_wz_dps = 260.0f;
    g_cfg.max_wheel_mmps = 800.0f;
    g_cfg.line_vy_limit_mmps = 550.0f;      /* 横移速度上限进一步增大 */
    g_cfg.line_forward_limit_mmps = 450.0f; /* 前进速度上限增大 */
    g_cfg.chassis_rot_radius_mm = 110.0f;

    /* 速度内环增强：原参数反馈过软，误差打不出有效PWM */
    g_cfg.wheel_pid_kp = 0.30f;
    g_cfg.wheel_pid_ki = 0.06f;
    g_cfg.wheel_pid_kd = 0.0f;

    /* 外环增强：居中和姿态纠偏输出更果断 */
    g_cfg.line_pid_kp = 12.0f;
    g_cfg.line_pid_ki = 0.25f;
    g_cfg.line_pid_kd = 0.10f;

    g_cfg.yaw_pid_kp = 7.0f;
    g_cfg.yaw_pid_ki = 0.02f;
    g_cfg.yaw_pid_kd = 0.12f;

    g_cfg.telemetry_enabled = 1U;
}

/* 初始化所有 PID，并配置积分分离、微分滤波与输出斜率限制。 */
static void init_control_pids(void)
{
    uint8_t i;

    for (i = 0U; i < CONTROL_WHEEL_COUNT; i++) {
        PID_Init(&g_state.wheel[i].pid,
                 g_cfg.wheel_pid_kp,
                 g_cfg.wheel_pid_ki,
                 g_cfg.wheel_pid_kd,
                 100.0f,
                 200.0f);
        g_state.wheel[i].pid.integral_separation = 800.0f;
        g_state.wheel[i].pid.derivative_alpha = 0.25f;
        g_state.wheel[i].pid.output_ramp = 12.0f;
    }

    PID_Init(&g_state.line_pid,
             g_cfg.line_pid_kp,
             g_cfg.line_pid_ki,
             g_cfg.line_pid_kd,
             g_cfg.line_vy_limit_mmps,
             150.0f);
    g_state.line_pid.integral_separation = 35.0f;
    g_state.line_pid.derivative_alpha = 0.22f;
    g_state.line_pid.output_ramp = 45.0f;

    /* 横向位置PID（控制Vy）：以 Lat_Err 为输入 */
    PID_Init(&g_state.lat_pid,
             g_cfg.line_pid_kp,
             g_cfg.line_pid_ki,
             g_cfg.line_pid_kd,
             g_cfg.line_vy_limit_mmps,
             150.0f);
    g_state.lat_pid.integral_separation = 35.0f;
    g_state.lat_pid.derivative_alpha = 0.22f;
    g_state.lat_pid.output_ramp = 45.0f;

    /* 偏航角度PID（控制W）：以 Yaw_Err 为输入 */
    PID_Init(&g_state.yaw_pid,
             g_cfg.yaw_pid_kp,
             g_cfg.yaw_pid_ki,
             g_cfg.yaw_pid_kd,
             g_cfg.max_wz_dps,
             90.0f);
    g_state.yaw_pid.integral_separation = 45.0f;
    g_state.yaw_pid.derivative_alpha = 0.20f;
    g_state.yaw_pid.output_ramp = 15.0f;
}

/* 模式切换内部实现。
 * 这里统一处理模式切换时的附加动作，避免分散在各处。 */
static void set_mode_internal(ControlMode_t mode)
{
    if (g_state.mode == mode) {
        return;
    }

    g_state.mode = mode;

    if (mode == CONTROL_MODE_LINE_FOLLOW) {
        PID_Reset(&g_state.line_pid);
        PID_Reset(&g_state.yaw_pid);
        g_state.yaw_target_deg = g_state.yaw_meas_deg;
        g_state.line_single_loss_ticks = 0U;
    }

    if ((mode == CONTROL_MODE_IDLE) || (mode == CONTROL_MODE_CALIBRATION)) {
        g_state.cmd_vx_mmps = 0.0f;
        g_state.cmd_vy_mmps = 0.0f;
        g_state.cmd_wz_dps = 0.0f;
    }
}

/* 单通道传感器处理：
 * 采用 物理极值剔除 + 5窗口中值滤波(Median) + IIR低通滤波 的超级组合策略，
 * 专治 VL53L0X 空旷或边缘反射时的 2.0 等乱跳跳变。 */
static void update_single_sensor(uint8_t index, uint32_t raw_value, uint8_t valid)
{
    SensorChannelState_t *ch = &g_state.ultra[index];
    float alpha = g_cfg.iir_alpha[index];
    float current_val = (float)raw_value;

    static float window[CONTROL_ULTRA_COUNT][5] = {0};
    static uint8_t win_idx[CONTROL_ULTRA_COUNT] = {0};
    static uint8_t win_count[CONTROL_ULTRA_COUNT] = {0};

    /* 1. 物理常识异常剔除（Outlier Rejection）
     * VL53L0X 测距失败或受边缘反光干扰时，常输出 2.0cm（极小值）或几百cm。
     * 由于小车靠墙极限不可能小于 4cm (会撞墙)，遇到 <= 4.0cm 的值直接视为盲区乱码丢弃。*/
    uint8_t is_noise = 0;
    if (valid && (current_val <= 4.0f || current_val > 150.0f)) {
        is_noise = 1;
    }

    /* 2. 失效保护逻辑 (Invalid Latching)
     * 允许最多丢失 5 帧（约 50ms），以跨越 --- 或连续 2.0 等噪点。 */
    static uint8_t loss_count[CONTROL_ULTRA_COUNT] = {0};
    if (!valid || is_noise) {
        if (ch->valid && (loss_count[index] < 5)) {
            loss_count[index]++;
            /* 维持之前的 filtered_raw 输出，不污染历史窗口缓存 */
        } else {
            ch->valid = 0U;
            ch->raw = 0;
            win_count[index] = 0; /* 长时间丢失，清空窗口缓存 */
        }
        return;
    }

    /* 数据正常 */
    loss_count[index] = 0;
    ch->raw = raw_value;

    /* 3. 滑动窗口中值滤波 (Median Filter) - 过滤突发毛刺 */
    window[index][win_idx[index]] = current_val;
    win_idx[index] = (win_idx[index] + 1) % 5;
    if (win_count[index] < 5) {
        win_count[index]++;
    }

    /* 对窗口内的数据进行排序，取中位数 */
    float sorted[5];
    int i, j;
    for (i = 0; i < win_count[index]; i++) {
        sorted[i] = window[index][i];
    }
    for (i = 0; i < win_count[index] - 1; i++) {
        for (j = i + 1; j < win_count[index]; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }
    float median_val = sorted[win_count[index] / 2];

    /* 4. IIR 平滑低通滤波 */
    if (!ch->valid) {
        /* 如果是重新获得的第一个有效值，直接初始化 */
        ch->filtered_raw = median_val;
        ch->valid = 1U;
    } else {
        ch->filtered_raw += alpha * (median_val - ch->filtered_raw);
    }

    ch->normalized = normalize_raw_to_0_100(index, ch->filtered_raw);
}

/* 多通道统一更新（按 SUPVC 通道序号 1..N）。
 * 直接使用 cm 单位距离作为 raw 值进行归一化 */
static void update_sensor_pipeline(void)
{
    uint8_t i;
    float distance_cm;

    for (i = 0U; i < CONTROL_ULTRA_COUNT; i++) {
        distance_cm = SUPVC_GetDistanceCm((uint8_t)(i + 1U));
        if (distance_cm < 0.0f) {
            update_single_sensor(i, 0U, 0U);
        } else {
            update_single_sensor(i, (uint32_t)(distance_cm), SUPVC_IsValid((uint8_t)(i + 1U)));
        }
    }
}

/* ============================================================================
 * 麦克纳姆轮运动学逆解分配
 * ============================================================================
 *
 * VL53L0X 传感器布局（小车朝向走廊前方）:
 *
 *              CH1(前方/Front)
 *                   ↑
 *     CH3(左前/LF)      CH4(右前/RF)
 *        |                |
 *        |    小车         |
 *        |                |
 *     CH5(左后/LR)      CH6(右后/RR)
 *              CH2(后方/Rear)
 *
 * 状态误差解算:
 * - 偏航角度误差(Yaw_Err): 判断车体与墙壁的平行度
 *   Yaw_Err = (D_LR - D_LF) - (D_RR - D_RF)
 *   >0 表示车头偏右，需要向左自转纠正
 *
 * - 横向中心误差(Lat_Err): 判断车体整体偏左还是偏右
 *   Lat_Err = (D_LR + D_LF)/2 - (D_RR + D_RF)/2
 *   >0 表示左侧平均距离大，小车偏左，需要向右平移纠正
 *
 * PID 闭环控制:
 *   Vy = Lat_PID(Lat_Err)   // 横移速度，控制小车居中
 *   W  = Yaw_PID(Yaw_Err)   // 自转角速度，控制小车姿态
 *   Vx = 设定值              // 前进速度
 *
 * 麦轮逆解公式:
 *   A(左前) = Vx + Vy - W
 *   B(右前) = Vx - Vy + W
 *   C(左后) = Vx + Vy + W
 *   D(右后) = Vx - Vy - W
 *
 * 物理意义:
 * - Vy > 0: 小车向右横移
 * - W  > 0: 小车逆时针旋转（车头向左转）
 * ============================================================================ */

/* 符号调整宏定义（方便现场调车） */
#define MECANUM_VX_SIGN_A    1.0f   /* 左前轮 Vx 符号 */
#define MECANUM_VX_SIGN_B    1.0f   /* 右前轮 Vx 符号 */
#define MECANUM_VX_SIGN_C    1.0f   /* 左后轮 Vx 符号 */
#define MECANUM_VX_SIGN_D    1.0f   /* 右后轮 Vx 符号 */

#define MECANUM_VY_SIGN_A    1.0f   /* 左前轮 Vy 符号：Vy>0 向右横移 */
#define MECANUM_VY_SIGN_B   -1.0f   /* 右前轮 Vy 符号：Vy>0 向右横移 */
#define MECANUM_VY_SIGN_C   -1.0f   /* 左后轮 Vy 符号：Vy>0 向右横移 */
#define MECANUM_VY_SIGN_D    1.0f   /* 右后轮 Vy 符号：Vy>0 向右横移 */

#define MECANUM_W_SIGN_A    -1.0f   /* 左前轮 W 符号：W>0 逆时针，左轮向后 */
#define MECANUM_W_SIGN_B     1.0f   /* 右前轮 W 符号：W>0 逆时针，右轮向前 */
#define MECANUM_W_SIGN_C    -1.0f   /* 左后轮 W 符号：W>0 逆时针，左轮向后 */
#define MECANUM_W_SIGN_D     1.0f   /* 右后轮 W 符号：W>0 逆时针，右轮向前 */

/* 麦克纳姆轮运动学逆解：Vx/Vy/W -> 四轮目标速度 */
static void mecanum_inverse_kinematics(float vx_mmps,
                                       float vy_mmps,
                                       float wz_dps,
                                       float wheel_targets[CONTROL_WHEEL_COUNT])
{
    float vw = wz_dps * CONTROL_DEG2RAD * g_cfg.chassis_rot_radius_mm;
    float max_abs;
    float scale;

    /* 运动学逆解分配（使用宏定义符号） */
    wheel_targets[0] = MECANUM_VX_SIGN_A * vx_mmps + MECANUM_VY_SIGN_A * vy_mmps + MECANUM_W_SIGN_A * vw;
    wheel_targets[1] = MECANUM_VX_SIGN_B * vx_mmps + MECANUM_VY_SIGN_B * vy_mmps + MECANUM_W_SIGN_B * vw;
    wheel_targets[2] = MECANUM_VX_SIGN_C * vx_mmps + MECANUM_VY_SIGN_C * vy_mmps + MECANUM_W_SIGN_C * vw;
    wheel_targets[3] = MECANUM_VX_SIGN_D * vx_mmps + MECANUM_VY_SIGN_D * vy_mmps + MECANUM_W_SIGN_D * vw;

    /* 速度归一化限幅 */
    max_abs = fabsf(wheel_targets[0]);
    if (fabsf(wheel_targets[1]) > max_abs) { max_abs = fabsf(wheel_targets[1]); }
    if (fabsf(wheel_targets[2]) > max_abs) { max_abs = fabsf(wheel_targets[2]); }
    if (fabsf(wheel_targets[3]) > max_abs) { max_abs = fabsf(wheel_targets[3]); }

    if ((max_abs > g_cfg.max_wheel_mmps) && (max_abs > CONTROL_EPSILON)) {
        scale = g_cfg.max_wheel_mmps / max_abs;
        wheel_targets[0] *= scale;
        wheel_targets[1] *= scale;
        wheel_targets[2] *= scale;
        wheel_targets[3] *= scale;
    }
}

/* 由编码器累计计数反推速度（mm/s）并做低通滤波。
 * 轮序映射:
 * wheel0=A(TIM5), wheel1=B(TIM3), wheel2=C(TIM4), wheel3=D(TIM1) */
static void update_wheel_speed_estimation(void)
{
    int32_t current_count[CONTROL_WHEEL_COUNT];
    int32_t delta;
    uint8_t i;
    float raw_speed;
    float vw;

    current_count[0] = Encoder_TIM5_Count;
    current_count[1] = Encoder_TIM3_Count;
    current_count[2] = Encoder_TIM4_Count;
    current_count[3] = Encoder_TIM1_Count;

    if (!g_state.encoder_initialized) {
        for (i = 0U; i < CONTROL_WHEEL_COUNT; i++) {
            g_state.wheel[i].last_count = current_count[i];
            g_state.wheel[i].measured_mmps = 0.0f;
            g_state.wheel[i].initialized = 1U;
        }
        g_state.encoder_initialized = 1U;
        g_state.vx_meas_mmps = 0.0f;
        g_state.vy_meas_mmps = 0.0f;
        g_state.wz_meas_dps = 0.0f;
        return;
    }

    for (i = 0U; i < CONTROL_WHEEL_COUNT; i++) {
        delta = current_count[i] - g_state.wheel[i].last_count;
        g_state.wheel[i].last_count = current_count[i];

        raw_speed = ((float)delta / PULSES_PER_MM) / CONTROL_DT_S;
        g_state.wheel[i].measured_mmps += g_cfg.wheel_speed_lpf_alpha *
                                          (raw_speed - g_state.wheel[i].measured_mmps);

        g_state.wheel_meas_mmps[i] = g_state.wheel[i].measured_mmps;
    }

    g_state.vx_meas_mmps = (g_state.wheel_meas_mmps[0] +
                            g_state.wheel_meas_mmps[1] +
                            g_state.wheel_meas_mmps[2] +
                            g_state.wheel_meas_mmps[3]) * 0.25f;

    g_state.vy_meas_mmps = (g_state.wheel_meas_mmps[0] -
                            g_state.wheel_meas_mmps[1] -
                            g_state.wheel_meas_mmps[2] +
                            g_state.wheel_meas_mmps[3]) * 0.25f;

    vw = (-g_state.wheel_meas_mmps[0] +
           g_state.wheel_meas_mmps[1] -
           g_state.wheel_meas_mmps[2] +
           g_state.wheel_meas_mmps[3]) * 0.25f;

    if (g_cfg.chassis_rot_radius_mm > 1.0f) {
        g_state.wz_meas_dps = (vw / g_cfg.chassis_rot_radius_mm) * CONTROL_RAD2DEG;
    } else {
        g_state.wz_meas_dps = 0.0f;
    }
}

/* 单电机下发:
 * pwm 符号决定方向，绝对值决定占空比。 */
static void apply_single_motor(float pwm, void (*motor_fn)(int, uint8_t))
{
    float abs_pwm = fabsf(pwm);

    if (abs_pwm < 0.8f) {
        motor_fn(0, 0U);
        return;
    }

    motor_fn((int)clampf_local(abs_pwm, 0.0f, 100.0f), (pwm >= 0.0f) ? 1U : 0U);
}

/* 四轮 PWM 统一归一后下发，保证比例不变且不超 100%。 */
static void apply_wheel_pwm(void)
{
    float max_abs = fabsf(g_state.wheel_pwm_cmd[0]);
    float scale;

    if (fabsf(g_state.wheel_pwm_cmd[1]) > max_abs) {
        max_abs = fabsf(g_state.wheel_pwm_cmd[1]);
    }
    if (fabsf(g_state.wheel_pwm_cmd[2]) > max_abs) {
        max_abs = fabsf(g_state.wheel_pwm_cmd[2]);
    }
    if (fabsf(g_state.wheel_pwm_cmd[3]) > max_abs) {
        max_abs = fabsf(g_state.wheel_pwm_cmd[3]);
    }

    if ((max_abs > 100.0f) && (max_abs > CONTROL_EPSILON)) {
        scale = 100.0f / max_abs;
        g_state.wheel_pwm_cmd[0] *= scale;
        g_state.wheel_pwm_cmd[1] *= scale;
        g_state.wheel_pwm_cmd[2] *= scale;
        g_state.wheel_pwm_cmd[3] *= scale;
    }

    apply_single_motor(g_state.wheel_pwm_cmd[0], motor_A);
    apply_single_motor(g_state.wheel_pwm_cmd[1], motor_B);
    apply_single_motor(g_state.wheel_pwm_cmd[2], motor_C);
    apply_single_motor(g_state.wheel_pwm_cmd[3], motor_D);
}

/* ============================================================================
 * 走廊模式外环 - 状态误差解算 + PID闭环控制
 * ============================================================================
 *
 * VL53L0X 走廊模式外环控制
 *
 * 使用四个侧向传感器: CH3(LF), CH4(RF), CH5(LR), CH6(RR)
 *
 * 偏航角度误差: Yaw_Err = (D_LR - D_LF) - (D_RR - D_RF)
 * 横向中心误差: Lat_Err = (D_LR + D_LF)/2 - (D_RR + D_RF)/2
 *
 * 要求四个侧向传感器都有效才工作
 * ============================================================================ */
static void run_line_mode_outer_loop(float *vx_cmd, float *vy_cmd, float *wz_cmd)
{
    uint8_t lf_valid = g_state.ultra[SENSOR_LEFT_FRONT].valid;    /* CH3 左前 */
    uint8_t rf_valid = g_state.ultra[SENSOR_RIGHT_FRONT].valid;   /* CH4 右前 */
    uint8_t lr_valid = g_state.ultra[SENSOR_LEFT_REAR].valid;     /* CH5 左后 */
    uint8_t rr_valid = g_state.ultra[SENSOR_RIGHT_REAR].valid;    /* CH6 右后 */

    float dist_LF;  /* CH3 左前距离 */
    float dist_RF;  /* CH4 右前距离 */
    float dist_LR;  /* CH5 左后距离 */
    float dist_RR;  /* CH6 右后距离 */

    float yaw_error;   /* 偏航角度误差 */
    float lat_error;   /* 横向中心误差 */

    float vy_output;   /* 平移速度输出 */
    float wz_output;   /* 自转角速度输出 */

    /* 四个侧向传感器必须都有效 */
    if (!lf_valid || !rf_valid || !lr_valid || !rr_valid) {
        g_state.yaw_error = 0.0f;
        g_state.lat_error = 0.0f;
        *vx_cmd = 0.0f;
        *vy_cmd = 0.0f;
        *wz_cmd = 0.0f;
        return;
    }

    /* 归中模式：固定不前进，只做横向归中 + 平行墙面姿态控制。 */
    *vx_cmd = 0.0f;

    /* 获取四个侧向传感器距离值（cm） */
    dist_LF = g_state.ultra[SENSOR_LEFT_FRONT].filtered_raw;     /* CH3 左前 */
    dist_RF = g_state.ultra[SENSOR_RIGHT_FRONT].filtered_raw;    /* CH4 右前 */
    dist_LR = g_state.ultra[SENSOR_LEFT_REAR].filtered_raw;      /* CH5 左后 */
    dist_RR = g_state.ultra[SENSOR_RIGHT_REAR].filtered_raw;     /* CH6 右后 */

    /* 偏航角度误差：判断车体与墙壁的平行度
     * Yaw_Err = (D_LF - D_LR) - (D_RF - D_RR)
     * >0：前部偏右（LF变大或RF变小）→ 车头偏右 → 对应 W < 0 (向左自转) */
    yaw_error = (dist_LF - dist_LR) - (dist_RF - dist_RR) - g_state.yaw_target;
    g_state.yaw_error = yaw_error;

    /* 横向中心误差：判断小车整体偏左还是偏右
     * Lat_Err = (D_RR + D_RF)/2 - (D_LR + D_LF)/2
     * >0：右侧距离小 → 小车偏右 → 对应 Vy < 0 (向左平移) */
    lat_error = ((dist_RR + dist_RF) * 0.5f) - ((dist_LR + dist_LF) * 0.5f) - g_state.lat_target;
    g_state.lat_error = lat_error;

    /* 平移速度 Vy：针对 Lat_Err 闭环
     * >0 (偏左时): 输出 Vy > 0 向右平移 */
    vy_output = PID_Calc(&g_state.lat_pid,
                         g_state.lat_target,
                         lat_error * -1.0f, // 翻转正负号匹配 PID 输入
                         CONTROL_DT_S);
    vy_output = clampf_local(vy_output, -g_cfg.line_vy_limit_mmps, g_cfg.line_vy_limit_mmps);
    *vy_cmd = vy_output;

    /* 自转角速度 W：针对 Yaw_Err 闭环
     * >0 (车头偏右): 输出 W < 0 向左自转 */
    wz_output = PID_Calc(&g_state.yaw_pid,
                         g_state.yaw_target,
                         yaw_error,
                         CONTROL_DT_S);
    wz_output = clampf_local(wz_output, -g_cfg.max_wz_dps, g_cfg.max_wz_dps);
    *wz_cmd = wz_output * -1.0f; // 翻转正负号匹配逆解符号（W>0为左转）
}

/* 四轮速度内环:
 * 外环给出 Vx/Vy/W -> 麦轮逆解成四轮目标 -> 每轮PID输出PWM */
static void run_speed_loop(float vx_cmd, float vy_cmd, float wz_cmd)
{
    uint8_t i;
    uint8_t stop_mode;
    float target_abs;
    float pwm_abs;

    mecanum_inverse_kinematics(vx_cmd, vy_cmd, wz_cmd, g_state.wheel_target_mmps);

    stop_mode = (g_state.mode == CONTROL_MODE_IDLE) || (g_state.mode == CONTROL_MODE_CALIBRATION);
    if (stop_mode) {
        for (i = 0U; i < CONTROL_WHEEL_COUNT; i++) {
            g_state.wheel_target_mmps[i] = 0.0f;
        }
    }

    for (i = 0U; i < CONTROL_WHEEL_COUNT; i++) {
        if (stop_mode) {
            PID_Reset(&g_state.wheel[i].pid);
            g_state.wheel_pwm_cmd[i] = 0.0f;
        } else {
            g_state.wheel_pwm_cmd[i] = PID_Calc(&g_state.wheel[i].pid,
                                                g_state.wheel_target_mmps[i],
                                                g_state.wheel_meas_mmps[i],
                                                CONTROL_DT_S);

            /* 低速起步补偿: 目标速度存在但 PID 输出太小会导致电机只蜂鸣不转。 */
            target_abs = fabsf(g_state.wheel_target_mmps[i]);
            pwm_abs = fabsf(g_state.wheel_pwm_cmd[i]);
            if ((target_abs > CONTROL_WHEEL_START_SPEED_MIN) &&
                (pwm_abs > CONTROL_EPSILON) &&
                (pwm_abs < CONTROL_WHEEL_START_PWM_MIN)) {
                g_state.wheel_pwm_cmd[i] = (g_state.wheel_pwm_cmd[i] >= 0.0f) ?
                                           CONTROL_WHEEL_START_PWM_MIN :
                                           -CONTROL_WHEEL_START_PWM_MIN;
            }
        }
    }

    apply_wheel_pwm();//下发 PWM 到电机
}

/* 校准窗口累计统计:
 * 对目标通道记录 min/max/sum，采样完成后置 report_pending。 */
static void update_calibration_state(void)
{
    CalibrationState_t *cal = &g_state.calibration;
    uint8_t ch;

    if (!cal->active) {
        return;
    }

    for (ch = 0U; ch < CONTROL_ULTRA_COUNT; ch++) {
        uint8_t bit = (uint8_t)(1U << ch);
        uint32_t raw;

        if ((cal->channel_mask & bit) == 0U) {
            continue;
        }

        raw = g_state.ultra[ch].raw;
        if (raw == 0U) {
            continue;
        }

        if (cal->collected_samples == 0U) {
            cal->min_raw[ch] = raw;
            cal->max_raw[ch] = raw;
        } else {
            if (raw < cal->min_raw[ch]) {
                cal->min_raw[ch] = raw;
            }
            if (raw > cal->max_raw[ch]) {
                cal->max_raw[ch] = raw;
            }
        }

        cal->sum_raw[ch] += raw;
    }

    cal->collected_samples++;
    if (cal->collected_samples >= cal->target_samples) {
        cal->active = 0U;
        cal->report_pending = 1U;
        set_mode_internal(CONTROL_MODE_IDLE);
    }
}

/* 采集本周期快照，交给主循环统一输出 FireWater。 */
static void update_telemetry_snapshot(float vx_cmd, float vy_cmd, float wz_cmd)
{
    g_state.telemetry_snapshot.mode_id = (uint32_t)g_state.mode;
    g_state.telemetry_snapshot.time_ms = HAL_GetTick();

    g_state.telemetry_snapshot.left_raw = (float)g_state.ultra[SENSOR_LEFT_FRONT].raw;
    g_state.telemetry_snapshot.right_raw = (float)g_state.ultra[SENSOR_RIGHT_FRONT].raw;
    g_state.telemetry_snapshot.front_raw = (float)g_state.ultra[SENSOR_FRONT].raw;

    g_state.telemetry_snapshot.left_norm = g_state.ultra[SENSOR_LEFT_FRONT].valid ? g_state.ultra[SENSOR_LEFT_FRONT].filtered_raw : -1.0f;
    g_state.telemetry_snapshot.right_norm = g_state.ultra[SENSOR_RIGHT_FRONT].valid ? g_state.ultra[SENSOR_RIGHT_FRONT].filtered_raw : -1.0f;
    g_state.telemetry_snapshot.line_error = g_state.line_error;

    g_state.telemetry_snapshot.vx_meas = g_state.vx_meas_mmps;
    g_state.telemetry_snapshot.vy_meas = g_state.vy_meas_mmps;
    g_state.telemetry_snapshot.wz_meas = g_state.wz_meas_dps;
    g_state.telemetry_snapshot.yaw_deg = g_state.yaw_meas_deg;

    if (g_cfg.telemetry_enabled) {
        g_state.telemetry_pending = 1U;
    }
}

static void emit_state_frame(void)
{
    uart_printf("STATE,%s,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%lu\r\n",
                mode_to_text(g_state.mode),
                (unsigned long)HAL_GetTick(),
                g_state.telemetry_snapshot.dist_lf,
                g_state.telemetry_snapshot.dist_rf,
                g_state.telemetry_snapshot.dist_lr,
                g_state.telemetry_snapshot.dist_rr,
                g_state.telemetry_snapshot.dist_front,
                g_state.telemetry_snapshot.dist_rear,
                g_state.telemetry_snapshot.yaw_error,
                g_state.telemetry_snapshot.lat_error,
                g_state.telemetry_snapshot.vy_cmd,
                g_state.telemetry_snapshot.wz_cmd,
                g_state.telemetry_snapshot.vy_meas,
                g_state.telemetry_snapshot.wz_meas,
                g_state.telemetry_snapshot.yaw_deg,
                (unsigned long)g_state.telemetry_snapshot.raw_valid_mask);
}

static void emit_tune_frame(void)
{
    uart_printf("TUNE,%lu,%.1f,%.1f,%.1f,%.1f,%lu,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                (unsigned long)g_state.telemetry_snapshot.time_ms,
                g_state.telemetry_snapshot.dist_lf,
                g_state.telemetry_snapshot.dist_rf,
                g_state.telemetry_snapshot.dist_lr,
                g_state.telemetry_snapshot.dist_rr,
                (unsigned long)g_state.telemetry_snapshot.valid_mask,
                g_state.telemetry_snapshot.yaw_error,
                g_state.telemetry_snapshot.lat_error,
                g_state.telemetry_snapshot.vy_cmd,
                g_state.telemetry_snapshot.wz_cmd,
                g_state.telemetry_snapshot.vy_meas,
                g_state.telemetry_snapshot.wz_meas,
                g_state.telemetry_snapshot.yaw_deg);
}

static void emit_cal_frame(void)
{
    uart_printf("CAL,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.1f,%.1f,%.1f,%.1f\r\n",
                (unsigned long)g_state.telemetry_snapshot.time_ms,
                (unsigned long)g_state.telemetry_snapshot.raw_distance[SENSOR_FRONT],
                (unsigned long)g_state.telemetry_snapshot.raw_distance[SENSOR_REAR],
                (unsigned long)g_state.telemetry_snapshot.raw_distance[SENSOR_LEFT_FRONT],
                (unsigned long)g_state.telemetry_snapshot.raw_distance[SENSOR_RIGHT_FRONT],
                (unsigned long)g_state.telemetry_snapshot.raw_distance[SENSOR_LEFT_REAR],
                (unsigned long)g_state.telemetry_snapshot.raw_distance[SENSOR_RIGHT_REAR],
                (unsigned long)g_state.telemetry_snapshot.raw_valid_mask,
                g_state.telemetry_snapshot.dist_lf,
                g_state.telemetry_snapshot.dist_rf,
                g_state.telemetry_snapshot.dist_lr,
                g_state.telemetry_snapshot.dist_rr);
}

/* 轻量 CSV 分词器（就地修改字符串）。 */
static uint8_t tokenize_csv(char *input, char *tokens[], uint8_t max_tokens)
{
    uint8_t count = 0U;
    char *tok = strtok(input, ",");

    while ((tok != NULL) && (count < max_tokens)) {
        tokens[count++] = tok;
        tok = strtok(NULL, ",");
    }

    return count;
}

/* 解析校准命令中的通道字段:
 * "1".."N" 或 "ALL" -> 位掩码。 */
static uint8_t parse_channel_mask(const char *token, uint8_t *mask)
{
    int channel_id;

    if ((token == NULL) || (mask == NULL)) {
        return 0U;
    }

    if (str_eq_ci(token, "ALL")) {
        *mask = (uint8_t)((1U << CONTROL_ULTRA_COUNT) - 1U);
        return 1U;
    }

    channel_id = atoi(token);
    if ((channel_id >= 1) && (channel_id <= (int)CONTROL_ULTRA_COUNT)) {
        *mask = (uint8_t)(1U << ((uint8_t)channel_id - 1U));
        return 1U;
    }

    return 0U;
}

/* 控制模块统一初始化入口。 */
void Chassis_PID_Init(void)
{
    init_default_config();
    init_control_pids();
    reset_runtime_states();
}

/* 对外复位接口。 */
void Chassis_Control_ResetState(void)
{
    reset_runtime_states();
}

/* 兼容旧接口: 当前工程累计计数由中断侧维护，这里直接返回。 */
int32_t Read_Encoder_Total(TIM_HandleTypeDef *htim, int32_t *encoder_count)
{
    (void)htim;
    return *encoder_count;
}

void Chassis_SetMode(ControlMode_t mode)
{
    set_mode_internal(mode);
}

ControlMode_t Chassis_GetMode(void)
{
    return g_state.mode;
}

/* 手动速度模式入口，内部会自动切到 MANUAL_VEL。 */
void Chassis_SetManualVelocity(float vx_mmps, float vy_mmps, float wz_dps)
{
    g_state.cmd_vx_mmps = clampf_local(vx_mmps, -g_cfg.max_vx_mmps, g_cfg.max_vx_mmps);
    g_state.cmd_vy_mmps = clampf_local(vy_mmps, -g_cfg.max_vy_mmps, g_cfg.max_vy_mmps);
    g_state.cmd_wz_dps = clampf_local(wz_dps, -g_cfg.max_wz_dps, g_cfg.max_wz_dps);
    set_mode_internal(CONTROL_MODE_MANUAL_VEL);
}

/* 启动走廊直线模式:
 * forward_mmps 为前进速度，line_target 为左右归一化差值目标。 */
void Chassis_StartLineFollow(float forward_mmps, float line_target)
{
    g_state.line_forward_mmps = clampf_local(forward_mmps,
                                             -g_cfg.line_forward_limit_mmps,
                                             g_cfg.line_forward_limit_mmps);
    g_state.line_target = clampf_local(line_target, -100.0f, 100.0f);
    g_state.line_last_diff = 0.0f;
    g_state.line_error = 0.0f;
    set_mode_internal(CONTROL_MODE_LINE_FOLLOW);
}

void Chassis_SetLineTarget(float line_target)
{
    g_state.line_target = clampf_local(line_target, -100.0f, 100.0f);
}

void Chassis_StopLineFollow(void)
{
    set_mode_internal(CONTROL_MODE_IDLE);
}

/* 启动校准模式（只采样回传，不自动改参数）。 */
void Chassis_StartCalibration(uint8_t channel_mask, uint16_t sample_count)
{
    uint8_t i;

    if (channel_mask == 0U) {
        return;
    }

    if (sample_count == 0U) {
        sample_count = 50U;
    }

    memset((void *)&g_state.calibration, 0, sizeof(g_state.calibration));
    g_state.calibration.active = 1U;
    g_state.calibration.channel_mask = channel_mask;
    g_state.calibration.target_samples = sample_count;

    for (i = 0U; i < CONTROL_ULTRA_COUNT; i++) {
        g_state.calibration.min_raw[i] = 0xFFFFFFFFUL;
    }

    set_mode_internal(CONTROL_MODE_CALIBRATION);
}

/* 手动写入某一通道归一化上下限。 */
void Chassis_SetSensorNormRawRange(uint8_t channel, float min_raw, float max_raw)
{
    uint8_t idx;

    if ((channel < 1U) || (channel > CONTROL_ULTRA_COUNT)) {
        return;
    }

    if ((max_raw - min_raw) < 1.0f) {
        return;
    }

    idx = (uint8_t)(channel - 1U);
    g_cfg.min_raw[idx] = min_raw;
    g_cfg.max_raw[idx] = max_raw;
}

/* 10ms 控制主任务（中断环境）:
 * 1) 采样与滤波
 * 2) 模式外环
 * 3) 速度内环
 * 4) 快照缓存 */
void Control_10ms_Task(void)
{
    float vx_cmd = 0.0f;
    float vy_cmd = 0.0f;
    float wz_cmd = 0.0f;

    SUPVC_Service_10ms();// 先服务底层驱动，更新原始数据和有效性
    g_state.yaw_meas_deg = JY61P_Data.angle_z;

    update_sensor_pipeline();// 更新传感器处理链，得到归一化距离和 line_error
    update_wheel_speed_estimation();// 由编码器累计计数反推速度，并更新全局测量值
    update_calibration_state();// 如果在校准模式，累计统计 min/max/sum，并在完成后置 report_pending

    switch (g_state.mode) {
        case CONTROL_MODE_MANUAL_VEL:// 直接使用命令给定的速度
            vx_cmd = g_state.cmd_vx_mmps;
            vy_cmd = g_state.cmd_vy_mmps;
            wz_cmd = g_state.cmd_wz_dps;
            break;

        case CONTROL_MODE_LINE_FOLLOW:// 走廊模式外环生成 vx/vy/wz 给内环
            run_line_mode_outer_loop(&vx_cmd, &vy_cmd, &wz_cmd);
            break;

        case CONTROL_MODE_CALIBRATION:// 校准模式不控制运动，仅采样回传
        case CONTROL_MODE_IDLE:// 空闲模式不控制运动
        default:
            vx_cmd = 0.0f;
            vy_cmd = 0.0f;
            wz_cmd = 0.0f;
            break;
    }

    run_speed_loop(vx_cmd, vy_cmd, wz_cmd);// 内环下发 PWM
    update_telemetry_snapshot(vx_cmd, vy_cmd, wz_cmd);// 采集当前周期的 telemetry 快照，等待主循环发送
}

/* 主循环后台任务（非中断）:
 * VL53L0X 数据轮询与日志输出。 */
void Control_MainLoop_Task(void)
{
    static uint32_t sensor_stream_last_ms = 0U;
    uint32_t now = HAL_GetTick();

    SUPVC_Service_MainLoop();

    /* VL53L0X 数据回传由 supvc.c 内部的驱动负责发送日志 */
}

/* 单字符命令解析（推荐蓝牙控制方式）:
 * 1: 开启走廊模式（默认前进速度与目标）
 * 2: 停止走廊模式（切 IDLE）
 * 3: 开启遥测
 * 4: 关闭遥测
 * 5: 进入校准观测模式（持续输出 RAW）
 * 6: line_target +5
 * 7: line_target -5
 * 8: 前进速度 +20 mm/s（上限保护）
 * 9: 前进速度 -20 mm/s（下限保护）
 * 0: 复位控制状态
 * [/: 前对目标 -5（小车前部偏左）
 * ]: 前对目标 +5（小车前部偏右）
 * .: 后对目标 -5（小车后部偏左）
 * ,: 后对目标 +5（小车后部偏右）
 * ?: 查询当前状态 */
void UART_Command_ProcessByte(uint8_t cmd)
{
    switch (cmd) {
        case '1':
            PID_Reset(&g_state.lat_pid);
            PID_Reset(&g_state.yaw_pid);
            Chassis_StartLineFollow(g_state.line_forward_mmps, g_state.line_target);
            g_cfg.telemetry_enabled = 1U;
            uart_printf("ACK,1,LINE_ON\r\n");
            break;
        case '2':
            Chassis_StopLineFollow();
            update_telemetry_snapshot(0.0f, 0.0f, 0.0f);
            uart_printf("ACK,2,LINE_OFF\r\n");
            emit_state_frame();
            break;
        case '3':
            g_cfg.telemetry_enabled = 1U;
            uart_printf("ACK,3,TEL_ON\r\n");
            break;
        case '4':
            g_cfg.telemetry_enabled = 0U;
            uart_printf("ACK,4,TEL_OFF\r\n");
            break;
        case '5':
            set_mode_internal(CONTROL_MODE_CALIBRATION);
            g_cfg.telemetry_enabled = 1U;
            uart_printf("ACK,5,CAL_MODE_ON\r\n");
            break;
        case '6':
            g_state.lat_target = clampf_local(g_state.lat_target + 5.0f, -100.0f, 100.0f);
            uart_printf("ACK,LAT_TGT,%.1f\r\n", g_state.lat_target);
            break;
        case '7':
            g_state.lat_target = clampf_local(g_state.lat_target - 5.0f, -100.0f, 100.0f);
            uart_printf("ACK,LAT_TGT,%.1f\r\n", g_state.lat_target);
            break;
        case '8':
            g_state.line_forward_mmps = clampf_local(g_state.line_forward_mmps + 20.0f,
                                                     -g_cfg.line_forward_limit_mmps,
                                                     g_cfg.line_forward_limit_mmps);
            uart_printf("ACK,8,VX,%.1f\r\n", g_state.line_forward_mmps);
            break;
        case '9':
            g_state.line_forward_mmps = clampf_local(g_state.line_forward_mmps - 20.0f,
                                                     -g_cfg.line_forward_limit_mmps,
                                                     g_cfg.line_forward_limit_mmps);
            uart_printf("ACK,9,VX,%.1f\r\n", g_state.line_forward_mmps);
            break;
        case '0':
            Chassis_Control_ResetState();
            update_telemetry_snapshot(0.0f, 0.0f, 0.0f);
            uart_printf("ACK,0,RESET\r\n");
            emit_state_frame();
            break;
        /* 偏航角度目标调整 */
        case '[':
            g_state.yaw_target = clampf_local(g_state.yaw_target - 5.0f, -100.0f, 100.0f);
            uart_printf("ACK,YAW_TGT,%.1f\r\n", g_state.yaw_target);
            break;
        case ']':
            g_state.yaw_target = clampf_local(g_state.yaw_target + 5.0f, -100.0f, 100.0f);
            uart_printf("ACK,YAW_TGT,%.1f\r\n", g_state.yaw_target);
            break;
        /* 横向位置目标调整 */
        case '.':
            g_state.lat_target = clampf_local(g_state.lat_target - 5.0f, -100.0f, 100.0f);
            uart_printf("ACK,LAT_TGT,%.1f\r\n", g_state.lat_target);
            break;
        case ',':
            g_state.lat_target = clampf_local(g_state.lat_target + 5.0f, -100.0f, 100.0f);
            uart_printf("ACK,LAT_TGT,%.1f\r\n", g_state.lat_target);
            break;
        /* 查询当前状态 */
        case '?':
            update_telemetry_snapshot(0.0f, 0.0f, 0.0f);
            emit_state_frame();
            break;
        default:
            break;
    }
}

/* 命令协议解析器。
 * 支持命令:
 * CMD,MODE,<IDLE|MANUAL|LINE|CAL>
 * CMD,VEL,<vx>,<vy>,<wz>
 * CMD,LINE,START,<vx>,<target>
 * CMD,LINE,TARGET,<target>
 * CMD,LINE,STOP
 * CMD,PAIR,FRONT,<target>      ← 设置前对目标（控制小车前部）
 * CMD,PAIR,REAR,<target>       ← 设置后对目标（控制小车后部）
 * CMD,PAIR,GET                 ← 查询前后对状态
 * CMD,CAL,START,<1|2|3|ALL>,<samples>
 * CMD,CAL,SET,<ch>,<min_raw>,<max_raw>
 * CMD,TEL,ON|OFF
 * CMD,YAW,ON|OFF|TARGET,<deg>
 * CMD,PID,LINE,<kp>,<ki>,<kd>
 * CMD,PID,WHEEL,<kp>,<ki>,<kd>
 * CMD,PID,FRONT,<kp>,<ki>,<kd> ← 设置前对PID参数
 * CMD,PID,REAR,<kp>,<ki>,<kd>  ← 设置后对PID参数
 * CMD,PARAM,GET */
void UART_Command_ProcessLine(const char *line)
{
    char buffer[CONTROL_CMD_LINE_LEN];
    char *tokens[CONTROL_CMD_MAX_TOKENS];
    uint8_t count;

    if (line == NULL) {
        return;
    }

    memset(buffer, 0, sizeof(buffer));
    strncpy(buffer, line, sizeof(buffer) - 1U);

    count = tokenize_csv(buffer, tokens, CONTROL_CMD_MAX_TOKENS);
    if (count < 2U) {
        return;
    }

    if (!str_eq_ci(tokens[0], "CMD")) {
        return;
    }

    if (str_eq_ci(tokens[1], "MODE")) {
        if (count < 3U) {
            return;
        }
        if (str_eq_ci(tokens[2], "IDLE")) {
            set_mode_internal(CONTROL_MODE_IDLE);
        } else if (str_eq_ci(tokens[2], "MANUAL")) {
            set_mode_internal(CONTROL_MODE_MANUAL_VEL);
        } else if (str_eq_ci(tokens[2], "LINE")) {
            set_mode_internal(CONTROL_MODE_LINE_FOLLOW);
        } else if (str_eq_ci(tokens[2], "CAL")) {
            set_mode_internal(CONTROL_MODE_CALIBRATION);
        }
        return;
    }

    if (str_eq_ci(tokens[1], "VEL")) {
        if (count >= 5U) {
            Chassis_SetManualVelocity(strtof(tokens[2], NULL), strtof(tokens[3], NULL), strtof(tokens[4], NULL));
        }
        return;
    }

    if (str_eq_ci(tokens[1], "LINE")) {
        if (count < 3U) {
            return;
        }

        if (str_eq_ci(tokens[2], "START") && (count >= 5U)) {
            PID_Reset(&g_state.lat_pid);
            PID_Reset(&g_state.yaw_pid);
            Chassis_StartLineFollow(strtof(tokens[3], NULL), strtof(tokens[4], NULL));
        } else if (str_eq_ci(tokens[2], "TARGET") && (count >= 4U)) {
            g_state.lat_target = clampf_local(strtof(tokens[3], NULL), -100.0f, 100.0f);
        } else if (str_eq_ci(tokens[2], "STOP")) {
            Chassis_StopLineFollow();
        }
        return;
    }

    /* 误差目标设置命令 */
    if (str_eq_ci(tokens[1], "ERR")) {
        if (count < 3U) {
            return;
        }

        if (str_eq_ci(tokens[2], "YAW") && (count >= 4U)) {
            g_state.yaw_target = clampf_local(strtof(tokens[3], NULL), -100.0f, 100.0f);
            uart_printf("ACK,ERR,YAW,%.1f\r\n", g_state.yaw_target);
        } else if (str_eq_ci(tokens[2], "LAT") && (count >= 4U)) {
            g_state.lat_target = clampf_local(strtof(tokens[3], NULL), -100.0f, 100.0f);
            uart_printf("ACK,ERR,LAT,%.1f\r\n", g_state.lat_target);
        } else if (str_eq_ci(tokens[2], "GET")) {
            uart_printf("ERR,YAW_TGT,%.1f,YAW_ERR,%.2f\r\n",
                        g_state.yaw_target, g_state.yaw_error);
            uart_printf("ERR,LAT_TGT,%.1f,LAT_ERR,%.2f\r\n",
                        g_state.lat_target, g_state.lat_error);
            uart_printf("ERR,D1,%.1f,D2,%.1f,D4,%.1f,D5,%.1f\r\n",
                        g_state.ultra[0].normalized,
                        g_state.ultra[1].normalized,
                        g_state.ultra[3].normalized,
                        g_state.ultra[4].normalized);
        }
        return;
    }

    if (str_eq_ci(tokens[1], "CAL")) {
        if (count < 3U) {
            return;
        }

        if (str_eq_ci(tokens[2], "START") && (count >= 5U)) {
            uint8_t mask = 0U;
            uint16_t sample_count;
            if (!parse_channel_mask(tokens[3], &mask)) {
                return;
            }
            sample_count = (uint16_t)atoi(tokens[4]);
            Chassis_StartCalibration(mask, sample_count);
        } else if (str_eq_ci(tokens[2], "SET") && (count >= 6U)) {
            uint8_t ch = (uint8_t)atoi(tokens[3]);
            float min_raw = strtof(tokens[4], NULL);
            float max_raw = strtof(tokens[5], NULL);
            Chassis_SetSensorNormRawRange(ch, min_raw, max_raw);
        }
        return;
    }


    if (str_eq_ci(tokens[1], "TEL")) {
        if (count < 3U) {
            return;
        }

        if (str_eq_ci(tokens[2], "ON")) {
            g_cfg.telemetry_enabled = 1U;
        } else if (str_eq_ci(tokens[2], "OFF")) {
            g_cfg.telemetry_enabled = 0U;
        }
        return;
    }

    if (str_eq_ci(tokens[1], "YAW")) {
        if (count < 3U) {
            return;
        }

        if (str_eq_ci(tokens[2], "ON")) {
            g_state.yaw_hold_enabled = 1U;
            g_state.yaw_target_deg = g_state.yaw_meas_deg;
            PID_Reset(&g_state.yaw_pid);
        } else if (str_eq_ci(tokens[2], "OFF")) {
            g_state.yaw_hold_enabled = 0U;
        } else if (str_eq_ci(tokens[2], "TARGET") && (count >= 4U)) {
            g_state.yaw_target_deg = strtof(tokens[3], NULL);
            PID_Reset(&g_state.yaw_pid);
        }
        return;
    }

    if (str_eq_ci(tokens[1], "PID")) {
        if (count < 3U) {
            return;
        }

        if (str_eq_ci(tokens[2], "LINE") && (count >= 6U)) {
            g_cfg.line_pid_kp = strtof(tokens[3], NULL);
            g_cfg.line_pid_ki = strtof(tokens[4], NULL);
            g_cfg.line_pid_kd = strtof(tokens[5], NULL);
            PID_Init(&g_state.line_pid,
                     g_cfg.line_pid_kp,
                     g_cfg.line_pid_ki,
                     g_cfg.line_pid_kd,
                     g_cfg.line_vy_limit_mmps,
                     150.0f);
        } else if (str_eq_ci(tokens[2], "WHEEL") && (count >= 6U)) {
            uint8_t i;
            g_cfg.wheel_pid_kp = strtof(tokens[3], NULL);
            g_cfg.wheel_pid_ki = strtof(tokens[4], NULL);
            g_cfg.wheel_pid_kd = strtof(tokens[5], NULL);
            for (i = 0U; i < CONTROL_WHEEL_COUNT; i++) {
                PID_Init(&g_state.wheel[i].pid,
                         g_cfg.wheel_pid_kp,
                         g_cfg.wheel_pid_ki,
                         g_cfg.wheel_pid_kd,
                         100.0f,
                         200.0f);
                g_state.wheel[i].pid.integral_separation = 800.0f;
                g_state.wheel[i].pid.derivative_alpha = 0.25f;
                g_state.wheel[i].pid.output_ramp = 12.0f;
            }
        } else if (str_eq_ci(tokens[2], "LAT") && (count >= 6U)) {
            /* 横向位置PID参数设置 */
            float kp = strtof(tokens[3], NULL);
            float ki = strtof(tokens[4], NULL);
            float kd = strtof(tokens[5], NULL);
            PID_Init(&g_state.lat_pid, kp, ki, kd, g_cfg.line_vy_limit_mmps, 150.0f);
            g_state.lat_pid.integral_separation = 35.0f;
            g_state.lat_pid.derivative_alpha = 0.22f;
            g_state.lat_pid.output_ramp = 45.0f;
            uart_printf("ACK,PID,LAT,%.3f,%.3f,%.3f\r\n", kp, ki, kd);
        } else if (str_eq_ci(tokens[2], "YAW") && (count >= 6U)) {
            /* 偏航角度PID参数设置 */
            float kp = strtof(tokens[3], NULL);
            float ki = strtof(tokens[4], NULL);
            float kd = strtof(tokens[5], NULL);
            PID_Init(&g_state.yaw_pid, kp, ki, kd, g_cfg.max_wz_dps, 90.0f);
            g_state.yaw_pid.integral_separation = 45.0f;
            g_state.yaw_pid.derivative_alpha = 0.20f;
            g_state.yaw_pid.output_ramp = 15.0f;
            uart_printf("ACK,PID,YAW,%.3f,%.3f,%.3f\r\n", kp, ki, kd);
        }
        return;
    }

    if (str_eq_ci(tokens[1], "PARAM") && (count >= 3U) && str_eq_ci(tokens[2], "GET")) {
        uint8_t ch;
        uart_printf("PARAM,MODE,%u\r\n", (unsigned int)g_state.mode);
        uart_printf("PARAM,VX,%.1f,YAW_TGT,%.1f,LAT_TGT,%.1f\r\n",
                    g_state.line_forward_mmps,
                    g_state.yaw_target,
                    g_state.lat_target);
        for (ch = 0U; ch < CONTROL_ULTRA_COUNT; ch++) {
            uart_printf("PARAM,RANGE,%u,%.1f,%.1f\r\n",
                        (unsigned int)(ch + 1U),
                        g_cfg.min_raw[ch],
                        g_cfg.max_raw[ch]);
        }
        uart_printf("PARAM,LINE,%.3f,%.3f,%.3f\r\n", g_cfg.line_pid_kp, g_cfg.line_pid_ki, g_cfg.line_pid_kd);
        uart_printf("PARAM,WHEEL,%.3f,%.3f,%.3f\r\n", g_cfg.wheel_pid_kp, g_cfg.wheel_pid_ki, g_cfg.wheel_pid_kd);
        uart_printf("PARAM,LAT_PID,%.3f,%.3f,%.3f\r\n",
                    g_state.lat_pid.Kp, g_state.lat_pid.Ki, g_state.lat_pid.Kd);
        uart_printf("PARAM,YAW_PID,%.3f,%.3f,%.3f\r\n",
                    g_state.yaw_pid.Kp, g_state.yaw_pid.Ki, g_state.yaw_pid.Kd);
        return;
    }
}

/* Legacy wrappers */
void Move_Forward(float speed) { Chassis_SetManualVelocity(speed, 0.0f, 0.0f); }
void Move_Backward(float speed) { Chassis_SetManualVelocity(-speed, 0.0f, 0.0f); }
void Move_Left(float speed) { Chassis_SetManualVelocity(0.0f, -speed, 0.0f); }
void Move_Right(float speed) { Chassis_SetManualVelocity(0.0f, speed, 0.0f); }
void Move_TopLeft(float speed) { Chassis_SetManualVelocity(speed, -speed, 0.0f); }
void Move_TopRight(float speed) { Chassis_SetManualVelocity(speed, speed, 0.0f); }
void Move_BottomLeft(float speed) { Chassis_SetManualVelocity(-speed, -speed, 0.0f); }
void Move_BottomRight(float speed) { Chassis_SetManualVelocity(-speed, speed, 0.0f); }
void Rotate_In_Place(float speed) { Chassis_SetManualVelocity(0.0f, 0.0f, speed); }
