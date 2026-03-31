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
#define CONTROL_ULTRA_COUNT              3U
#define CONTROL_CMD_MAX_TOKENS           10
#define CONTROL_CMD_LINE_LEN             96
#define CONTROL_DEG2RAD                  0.0174532925f
#define CONTROL_RAD2DEG                  57.2957795f
#define CONTROL_EPSILON                  0.0001f
#define CONTROL_LINE_SINGLE_LOSS_LIMIT   30U

/* 全局控制参数:
 * - 超声归一化区间与滤波参数
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

/* 单通道超声运行状态:
 * raw           : 当前原始值（本工程对外统一用 SUPVC 的 raw）
 * filtered_raw  : 中值 + IIR 后的值
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
} UltrasonicChannelState_t;

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

    uint8_t yaw_hold_enabled;
    float yaw_target_deg;
    float yaw_meas_deg;

    float wheel_target_mmps[CONTROL_WHEEL_COUNT];
    float wheel_meas_mmps[CONTROL_WHEEL_COUNT];
    float wheel_pwm_cmd[CONTROL_WHEEL_COUNT];

    float vx_meas_mmps;
    float vy_meas_mmps;
    float wz_meas_dps;

    uint8_t encoder_initialized;

    UltrasonicChannelState_t ultra[CONTROL_ULTRA_COUNT];
    WheelControlState_t wheel[CONTROL_WHEEL_COUNT];
    PID_TypeDef line_pid;
    PID_TypeDef yaw_pid;

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

    g_state.mode = CONTROL_MODE_LINE_FOLLOW;//默认走廊模式
    g_state.cmd_vx_mmps = 0.0f;
    g_state.cmd_vy_mmps = 0.0f;
    g_state.cmd_wz_dps = 0.0f;
    g_state.line_forward_mmps = 180.0f;
    g_state.line_target = 0.0f;// 0 表示左右平衡，正值表示向右偏，负值表示向左偏，方向目标值
    g_state.line_error = 0.0f;
    g_state.line_last_diff = 0.0f;
    g_state.line_single_loss_ticks = 0U;
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
 * 这些值是“可跑起步值”，后续建议现场在线调参。 */
static void init_default_config(void)
{
    /* 左右超声归一化区间（基于 2026-03-31 实测）:
     * 左贴墙: L=191,   R=2258
     * 中线:   L=1188,  R=1227
     * 右贴墙: L=2225,  R=197
     *
     * 约定:
     * - min_raw 对应“贴墙近距离”
     * - max_raw 对应“离墙远距离”
     * 这样归一化后 0~100 与距离单调一致。 */
    g_cfg.min_raw[0] = 191.0f;   /* 左超声近墙 */
    g_cfg.max_raw[0] = 2225.0f;  /* 左超声远墙 */
    g_cfg.min_raw[1] = 197.0f;   /* 右超声近墙 */
    g_cfg.max_raw[1] = 2260.0f;  /* 右超声远墙 */

    /* 前向超声当前未用于闭环，仅保留默认区间。 */
    g_cfg.min_raw[2] = 200.0f;
    g_cfg.max_raw[2] = 3800.0f;

    g_cfg.iir_alpha[0] = 0.25f;
    g_cfg.iir_alpha[1] = 0.25f;
    g_cfg.iir_alpha[2] = 0.18f;

    g_cfg.wheel_speed_lpf_alpha = 0.45f;
    g_cfg.max_vx_mmps = 450.0f;
    g_cfg.max_vy_mmps = 450.0f;
    g_cfg.max_wz_dps = 180.0f;
    g_cfg.max_wheel_mmps = 800.0f;
    g_cfg.line_vy_limit_mmps = 260.0f;
    g_cfg.line_forward_limit_mmps = 350.0f;
    g_cfg.chassis_rot_radius_mm = 110.0f;

    g_cfg.wheel_pid_kp = 0.08f;
    g_cfg.wheel_pid_ki = 0.015f;
    g_cfg.wheel_pid_kd = 0.0f;

    g_cfg.line_pid_kp = 3.2f;
    g_cfg.line_pid_ki = 0.10f;
    g_cfg.line_pid_kd = 0.02f;

    g_cfg.yaw_pid_kp = 1.0f;
    g_cfg.yaw_pid_ki = 0.0f;
    g_cfg.yaw_pid_kd = 0.0f;

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

/* 单通道超声处理链:
 * 1) 原始值入队
 * 2) 3点中值抑制毛刺
 * 3) IIR 低通平滑
 * 4) 按通道标定区间归一化到 0~100 */
static void update_single_ultrasonic(uint8_t index, uint32_t raw_value, uint8_t valid)
{
    UltrasonicChannelState_t *ch = &g_state.ultra[index];
    float median_value;

    ch->raw = raw_value;

    if ((!valid) || (raw_value == 0U)) {
        ch->valid = 0U;
        return;
    }

    ch->history[ch->history_index] = raw_value;
    ch->history_index = (uint8_t)((ch->history_index + 1U) % 3U);
    if (ch->history_count < 3U) {
        ch->history_count++;
    }

    if (ch->history_count < 3U) {
        median_value = (float)raw_value;
    } else {
        median_value = (float)median3_u32(ch->history[0], ch->history[1], ch->history[2]);
    }

    if (!ch->filter_initialized) {
        ch->filtered_raw = median_value;
        ch->filter_initialized = 1U;
    } else {
        ch->filtered_raw += g_cfg.iir_alpha[index] * (median_value - ch->filtered_raw);
    }

    ch->normalized = normalize_raw_to_0_100(index, ch->filtered_raw);
    ch->valid = 1U;
}

/* 三通道统一更新（左、右、前）。 */
static void update_ultrasonic_pipeline(void)
{
    update_single_ultrasonic(0U, SUPVC_GetEchoWidthUs(1U), SUPVC_IsValid(1U));
    update_single_ultrasonic(1U, SUPVC_GetEchoWidthUs(2U), SUPVC_IsValid(2U));
    update_single_ultrasonic(2U, SUPVC_GetEchoWidthUs(3U), SUPVC_IsValid(3U));
}

/* 车体速度 -> 四轮目标线速度(mm/s)。
 * wz 先转换成等效切向速度，再做麦轮逆运动学分解。 */
static void body_to_wheels(float vx_mmps,
                           float vy_mmps,
                           float wz_dps,
                           float wheel_targets[CONTROL_WHEEL_COUNT])
{
    float vw = wz_dps * CONTROL_DEG2RAD * g_cfg.chassis_rot_radius_mm;
    float max_abs;
    float scale;

    wheel_targets[0] = vx_mmps + vy_mmps - vw;
    wheel_targets[1] = vx_mmps - vy_mmps + vw;
    wheel_targets[2] = vx_mmps - vy_mmps - vw;
    wheel_targets[3] = vx_mmps + vy_mmps + vw;

    max_abs = fabsf(wheel_targets[0]);
    if (fabsf(wheel_targets[1]) > max_abs) {
        max_abs = fabsf(wheel_targets[1]);
    }
    if (fabsf(wheel_targets[2]) > max_abs) {
        max_abs = fabsf(wheel_targets[2]);
    }
    if (fabsf(wheel_targets[3]) > max_abs) {
        max_abs = fabsf(wheel_targets[3]);
    }

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

/* 走廊模式外环:
 * diff = right_norm - left_norm
 * line_error = diff - line_target
 * vy_cmd 由方向环 PID 生成；vx_cmd 来自前进给定。
 * 丢失策略:
 * - 单侧短时丢失: 保持最近 diff
 * - 双侧丢失: 退回 IDLE，防止盲行 */
static void run_line_mode_outer_loop(float *vx_cmd, float *vy_cmd, float *wz_cmd)
{
    uint8_t left_valid = g_state.ultra[0].valid;
    uint8_t right_valid = g_state.ultra[1].valid;
    float diff = g_state.line_last_diff;

    *vx_cmd = clampf_local(g_state.line_forward_mmps,
                           -g_cfg.line_forward_limit_mmps,
                           g_cfg.line_forward_limit_mmps);

    if (left_valid && right_valid) {
        diff = g_state.ultra[1].normalized - g_state.ultra[0].normalized;
        g_state.line_last_diff = diff;
        g_state.line_single_loss_ticks = 0U;
    } else if (left_valid || right_valid) {
        if (g_state.line_single_loss_ticks < 255U) {
            g_state.line_single_loss_ticks++;
        }
        if (g_state.line_single_loss_ticks > CONTROL_LINE_SINGLE_LOSS_LIMIT) {
            diff = g_state.line_target;
        }
    } else {
        /* 双侧都无效时:
         * 保持 LINE_FOLLOW 模式，不自动退回 IDLE。
         * 这样开机阶段即使传感器还未稳定，也会在数据恢复后自动进入闭环。 */
        g_state.line_error = 0.0f;
        *vx_cmd = 0.0f;
        *vy_cmd = 0.0f;
        *wz_cmd = 0.0f;
        return;
    }

    g_state.line_error = diff - g_state.line_target;

    /* 现场标定发现底盘横移正方向与超声差值误差方向相反，
     * 这里对方向环输出取反，避免“越纠越偏”。 */
    *vy_cmd = -PID_Calc(&g_state.line_pid, g_state.line_target, diff, CONTROL_DT_S);
    *vy_cmd = clampf_local(*vy_cmd, -g_cfg.line_vy_limit_mmps, g_cfg.line_vy_limit_mmps);

    if (g_state.yaw_hold_enabled) {
        *wz_cmd = PID_Calc(&g_state.yaw_pid, g_state.yaw_target_deg, g_state.yaw_meas_deg, CONTROL_DT_S);
        *wz_cmd = clampf_local(*wz_cmd, -g_cfg.max_wz_dps, g_cfg.max_wz_dps);
    } else {
        *wz_cmd = 0.0f;
    }
}

/* 四轮速度内环:
 * 外环给出 vx/vy/wz -> 逆解成轮目标 -> 每轮 PID 输出 PWM。 */
static void run_speed_loop(float vx_cmd, float vy_cmd, float wz_cmd)
{
    uint8_t i;
    uint8_t stop_mode;

    body_to_wheels(vx_cmd, vy_cmd, wz_cmd, g_state.wheel_target_mmps);

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
static void update_telemetry_snapshot(void)
{
    g_state.telemetry_snapshot.mode_id = (uint32_t)g_state.mode;
    g_state.telemetry_snapshot.time_ms = HAL_GetTick();

    g_state.telemetry_snapshot.left_raw = (float)g_state.ultra[0].raw;
    g_state.telemetry_snapshot.right_raw = (float)g_state.ultra[1].raw;
    g_state.telemetry_snapshot.front_raw = (float)g_state.ultra[2].raw;

    g_state.telemetry_snapshot.left_norm = g_state.ultra[0].valid ? g_state.ultra[0].normalized : -1.0f;
    g_state.telemetry_snapshot.right_norm = g_state.ultra[1].valid ? g_state.ultra[1].normalized : -1.0f;
    g_state.telemetry_snapshot.line_error = g_state.line_error;

    g_state.telemetry_snapshot.vx_meas = g_state.vx_meas_mmps;
    g_state.telemetry_snapshot.vy_meas = g_state.vy_meas_mmps;
    g_state.telemetry_snapshot.wz_meas = g_state.wz_meas_dps;
    g_state.telemetry_snapshot.yaw_deg = g_state.yaw_meas_deg;

    if (g_cfg.telemetry_enabled) {
        g_state.telemetry_pending = 1U;
    }
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
 * \"1\"/\"2\"/\"3\" 或 \"ALL\" -> 位掩码。 */
static uint8_t parse_channel_mask(const char *token, uint8_t *mask)
{
    if ((token == NULL) || (mask == NULL)) {
        return 0U;
    }

    if (str_eq_ci(token, "ALL")) {
        *mask = 0x07U;
        return 1U;
    }

    if (str_eq_ci(token, "1")) {
        *mask = 0x01U;
        return 1U;
    }
    if (str_eq_ci(token, "2")) {
        *mask = 0x02U;
        return 1U;
    }
    if (str_eq_ci(token, "3")) {
        *mask = 0x04U;
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
    uart_printf("ACK,INIT,OK\r\n");
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
void Chassis_SetUltrasonicNormRawRange(uint8_t channel, float min_raw, float max_raw)
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

    update_ultrasonic_pipeline();// 更新超声波处理链，得到归一化距离和 line_error
    update_wheel_speed_estimation();// 由编码器累计计数反推速度，并更新全局测量值
    update_calibration_state();// 如果在校准模式，累计统计 min/max/sum，并在完成后置 report_pending

    switch (g_state.mode) {
        case CONTROL_MODE_MANUAL_VEL:// 直接使用命令给定的速度
            vx_cmd = g_state.cmd_vx_mmps;
            vy_cmd = g_state.cmd_vy_mmps;
            wz_cmd = g_state.cmd_wz_dps;
            break;

        case CONTROL_MODE_LINE_FOLLOW:// 走廊模式外环生成 vx/vy/wz 给内环
            run_line_mode_outer_loop(&vx_cmd, &vy_cmd, &wz_cmd);//方向环计算得到 vy_cmd 和 wz_cmd，vx_cmd 给速度环
            break;

        case CONTROL_MODE_CALIBRATION:// 校准模式不控制运动，仅采样回传
        case CONTROL_MODE_IDLE:// 空闲模式不控制运动
        default:
            vx_cmd = 0.0f;
            vy_cmd = 0.0f;
            wz_cmd = 0.0f;
            break;
    }

    run_speed_loop(vx_cmd, vy_cmd, wz_cmd);// 内环下发 PWM，越快越好以降低延迟，此为速度环计算得到 PWM 给电机
    update_telemetry_snapshot();// 采集当前周期的 telemetry 快照，等待主循环发送
}

/* 主循环后台任务（非中断）:
 * - 发送校准结果
 * - 发送 FireWater 遥测 */
void Control_MainLoop_Task(void)
{
    CalibrationState_t cal_copy;
    static uint32_t raw_stream_last_ms = 0U;

    if (g_state.calibration.report_pending) {
        uint8_t ch;

        __disable_irq();
        cal_copy = g_state.calibration;
        g_state.calibration.report_pending = 0U;
        __enable_irq();

        for (ch = 0U; ch < CONTROL_ULTRA_COUNT; ch++) {
            uint8_t bit = (uint8_t)(1U << ch);
            float avg;

            if ((cal_copy.channel_mask & bit) == 0U) {
                continue;
            }

            if (cal_copy.collected_samples == 0U) {
                avg = 0.0f;
            } else {
                avg = (float)((double)cal_copy.sum_raw[ch] / (double)cal_copy.collected_samples);
            }

            uart_printf("CAL,%u,%lu,%lu,%.2f\r\n",
                        (unsigned int)(ch + 1U),
                        (unsigned long)cal_copy.min_raw[ch],
                        (unsigned long)cal_copy.max_raw[ch],
                        avg);
        }
    }

    if (g_cfg.telemetry_enabled && g_state.telemetry_pending) {
        TelemetrySnapshot_t snap;

        __disable_irq();
        snap = g_state.telemetry_snapshot;
        g_state.telemetry_pending = 0U;
        __enable_irq();

        /* FireWater 固定通道顺序:
         * mode_id,time_ms,left_raw,right_raw,front_raw,left_norm,right_norm,
         * line_error,vx_meas,vy_meas,wz_meas,yaw_deg */
        uart_printf("%lu,%lu,%.0f,%.0f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    (unsigned long)snap.mode_id,
                    (unsigned long)snap.time_ms,
                    snap.left_raw,
                    snap.right_raw,
                    snap.front_raw,
                    snap.left_norm,
                    snap.right_norm,
                    snap.line_error,
                    snap.vx_meas,
                    snap.vy_meas,
                    snap.wz_meas,
                    snap.yaw_deg);
    }

    /* 校准观测模式: 周期输出三路超声原始值，便于人工摆位采样。
     * 输出格式:
     * RAW,left_raw,right_raw,front_raw,left_valid,right_valid,front_valid */
    if (g_state.mode == CONTROL_MODE_CALIBRATION) {
        uint32_t now = HAL_GetTick();
        if ((now - raw_stream_last_ms) >= 100U) { /* 10Hz */
            raw_stream_last_ms = now;
            uart_printf("RAW,%lu,%lu,%lu,%u,%u,%u\r\n",
                        (unsigned long)g_state.ultra[0].raw,
                        (unsigned long)g_state.ultra[1].raw,
                        (unsigned long)g_state.ultra[2].raw,
                        (unsigned int)g_state.ultra[0].valid,
                        (unsigned int)g_state.ultra[1].valid,
                        (unsigned int)g_state.ultra[2].valid);
        }
    } else {
        raw_stream_last_ms = HAL_GetTick();
    }
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
 * 0: 复位控制状态 */
void UART_Command_ProcessByte(uint8_t cmd)
{
    switch (cmd) {
        case '1':
            Chassis_StartLineFollow(g_state.line_forward_mmps, g_state.line_target);
            uart_printf("ACK,1,LINE_ON\r\n");
            break;
        case '2':
            Chassis_StopLineFollow();
            uart_printf("ACK,2,LINE_OFF\r\n");
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
            g_cfg.telemetry_enabled = 0U;
            uart_printf("ACK,5,CAL_MODE_ON\r\n");
            break;
        case '6':
            Chassis_SetLineTarget(g_state.line_target + 5.0f);
            uart_printf("ACK,6,TGT,%.1f\r\n", g_state.line_target);
            break;
        case '7':
            Chassis_SetLineTarget(g_state.line_target - 5.0f);
            uart_printf("ACK,7,TGT,%.1f\r\n", g_state.line_target);
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
            uart_printf("ACK,0,RESET\r\n");
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
 * CMD,CAL,START,<1|2|3|ALL>,<samples>
 * CMD,CAL,SET,<ch>,<min_raw>,<max_raw>
 * CMD,TEL,ON|OFF
 * CMD,YAW,ON|OFF|TARGET,<deg>
 * CMD,PID,LINE,<kp>,<ki>,<kd>
 * CMD,PID,WHEEL,<kp>,<ki>,<kd>
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
            Chassis_StartLineFollow(strtof(tokens[3], NULL), strtof(tokens[4], NULL));
        } else if (str_eq_ci(tokens[2], "TARGET") && (count >= 4U)) {
            Chassis_SetLineTarget(strtof(tokens[3], NULL));
        } else if (str_eq_ci(tokens[2], "STOP")) {
            Chassis_StopLineFollow();
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
            Chassis_SetUltrasonicNormRawRange(ch, min_raw, max_raw);
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
        }
        return;
    }

    if (str_eq_ci(tokens[1], "PARAM") && (count >= 3U) && str_eq_ci(tokens[2], "GET")) {
        uart_printf("PARAM,MODE,%u\r\n", (unsigned int)g_state.mode);
        uart_printf("PARAM,RANGE,1,%.1f,%.1f\r\n", g_cfg.min_raw[0], g_cfg.max_raw[0]);
        uart_printf("PARAM,RANGE,2,%.1f,%.1f\r\n", g_cfg.min_raw[1], g_cfg.max_raw[1]);
        uart_printf("PARAM,RANGE,3,%.1f,%.1f\r\n", g_cfg.min_raw[2], g_cfg.max_raw[2]);
        uart_printf("PARAM,LINE,%.3f,%.3f,%.3f\r\n", g_cfg.line_pid_kp, g_cfg.line_pid_ki, g_cfg.line_pid_kd);
        uart_printf("PARAM,WHEEL,%.3f,%.3f,%.3f\r\n", g_cfg.wheel_pid_kp, g_cfg.wheel_pid_ki, g_cfg.wheel_pid_kd);
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
