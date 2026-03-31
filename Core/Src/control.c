#include "control.h"
#include "math.h"
#include "supvc.h"
#include "usart.h"

extern int32_t Encoder_TIM3_Count;
extern int32_t Encoder_TIM4_Count;
extern int32_t Encoder_TIM5_Count;
extern int32_t Encoder_TIM1_Count;

PID_TypeDef PID_Pos_A;
PID_TypeDef PID_Pos_B;
PID_TypeDef PID_Pos_C;
PID_TypeDef PID_Pos_D;

float Target_Pulses_A = 0.0f;
float Target_Pulses_B = 0.0f;
float Target_Pulses_C = 0.0f;
float Target_Pulses_D = 0.0f;

static PID_TypeDef PID_Pos_X;
static PID_TypeDef PID_Pos_Y;

static const float CONTROL_DT = 0.01f;
static const float POSITION_TOLERANCE_MM = 6.0f;
static const float POSITION_BRAKE_WINDOW_MM = 80.0f;
static const float MIN_EFFECTIVE_PWM = 12.0f;

static float manual_vx_cmd = 0.0f;
static float manual_vy_cmd = 0.0f;
static float manual_vw_cmd = 0.0f;

static uint8_t manual_mode_enabled = 0U;
static uint8_t position_mode_enabled = 0U;

static float start_pulses_a = 0.0f;
static float start_pulses_b = 0.0f;
static float start_pulses_c = 0.0f;
static float start_pulses_d = 0.0f;

static float target_x_mm = 0.0f;
static float target_y_mm = 0.0f;

static uint8_t ultrasonic_centerline_enabled = 0U;
static float ultrasonic_follow_forward_speed = ULTRA_CENTERLINE_DEFAULT_VX;

/* 对输入值进行限幅，约束在给定区间内。 */
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

/* 将超声距离映射到 0~100 的归一化区间。 */
static float normalize_distance_0_100(float distance_cm)
{
    float clipped = clampf_local(distance_cm, ULTRA_NORM_MIN_CM, ULTRA_NORM_MAX_CM);
    return ((clipped - ULTRA_NORM_MIN_CM) * 100.0f) / (ULTRA_NORM_MAX_CM - ULTRA_NORM_MIN_CM);
}

/* 读取 A 轮编码器累计脉冲。 */
static float read_wheel_a_pulses(void)
{
    return (float)Read_Encoder_Total(&htim5, &Encoder_TIM5_Count);
}

/* 读取 B 轮编码器累计脉冲。 */
static float read_wheel_b_pulses(void)
{
    return (float)Read_Encoder_Total(&htim3, &Encoder_TIM3_Count);
}

/* 读取 C 轮编码器累计脉冲。 */
static float read_wheel_c_pulses(void)
{
    return (float)Read_Encoder_Total(&htim4, &Encoder_TIM4_Count);
}

/* 读取 D 轮编码器累计脉冲。 */
static float read_wheel_d_pulses(void)
{
    return (float)Read_Encoder_Total(&htim1, &Encoder_TIM1_Count);
}

/* 锁存当前编码器值，作为相对位移任务的起点。 */
static void latch_motion_start(void)
{
    start_pulses_a = read_wheel_a_pulses();
    start_pulses_b = read_wheel_b_pulses();
    start_pulses_c = read_wheel_c_pulses();
    start_pulses_d = read_wheel_d_pulses();
}

/* 更新四轮目标脉冲（用于调试观测）。 */
static void update_debug_wheel_targets(void)
{
    float delta_a = (target_x_mm + target_y_mm) * PULSES_PER_MM;
    float delta_b = (target_x_mm - target_y_mm) * PULSES_PER_MM;
    float delta_c = (target_x_mm - target_y_mm) * PULSES_PER_MM;
    float delta_d = (target_x_mm + target_y_mm) * PULSES_PER_MM;

    Target_Pulses_A = start_pulses_a + delta_a;
    Target_Pulses_B = start_pulses_b + delta_b;
    Target_Pulses_C = start_pulses_c + delta_c;
    Target_Pulses_D = start_pulses_d + delta_d;
}

/* 计算车体坐标系下的当前位移（mm）。 */
static void get_body_displacement_mm(float *x_mm, float *y_mm)
{
    float delta_a = read_wheel_a_pulses() - start_pulses_a;
    float delta_b = read_wheel_b_pulses() - start_pulses_b;
    float delta_c = read_wheel_c_pulses() - start_pulses_c;
    float delta_d = read_wheel_d_pulses() - start_pulses_d;

    *x_mm = (delta_a + delta_b + delta_c + delta_d) / (4.0f * PULSES_PER_MM);
    *y_mm = (delta_a - delta_b - delta_c + delta_d) / (4.0f * PULSES_PER_MM);
}

/* 将车体速度指令分解为四轮指令。 */
static void body_to_wheels(float vx, float vy, float vw,
                           float *wheel_a, float *wheel_b, float *wheel_c, float *wheel_d)
{
    *wheel_a = vx + vy - vw;
    *wheel_b = vx - vy + vw;
    *wheel_c = vx - vy - vw;
    *wheel_d = vx + vy + vw;
}

/* 对四轮指令做幅值归一，保持比例不变。 */
static void normalize_wheel_outputs(float *wheel_a, float *wheel_b, float *wheel_c, float *wheel_d, float max_magnitude)
{
    float max_abs = fabsf(*wheel_a);

    if (fabsf(*wheel_b) > max_abs) {
        max_abs = fabsf(*wheel_b);
    }
    if (fabsf(*wheel_c) > max_abs) {
        max_abs = fabsf(*wheel_c);
    }
    if (fabsf(*wheel_d) > max_abs) {
        max_abs = fabsf(*wheel_d);
    }

    if ((max_abs > max_magnitude) && (max_abs > 0.001f)) {
        float scale = max_magnitude / max_abs;
        *wheel_a *= scale;
        *wheel_b *= scale;
        *wheel_c *= scale;
        *wheel_d *= scale;
    }
}

/* 单电机输出：符号决定方向，绝对值决定占空比。 */
static void apply_single_motor(float pwm, void (*motor_fn)(int, uint8_t))
{
    if (fabsf(pwm) < 1.0f) {
        motor_fn(0, 0U);
        return;
    }

    motor_fn((int)fabsf(clampf_local(pwm, -100.0f, 100.0f)), (pwm >= 0.0f) ? 1U : 0U);
}

/* 将四轮输出下发到电机驱动。 */
static void apply_wheel_outputs(float wheel_a, float wheel_b, float wheel_c, float wheel_d)
{
    normalize_wheel_outputs(&wheel_a, &wheel_b, &wheel_c, &wheel_d, 100.0f);
    apply_single_motor(wheel_a, motor_A);
    apply_single_motor(wheel_b, motor_B);
    apply_single_motor(wheel_c, motor_C);
    apply_single_motor(wheel_d, motor_D);
}

/* 清空相对运动目标并重置起点。 */
static void reset_motion_targets(void)
{
    latch_motion_start();
    target_x_mm = 0.0f;
    target_y_mm = 0.0f;
    update_debug_wheel_targets();
}

/* 设置 PID 的附加调参项（积分分离/微分滤波/斜率限制）。 */
static void apply_position_pid_tuning(PID_TypeDef *pid)
{
    pid->integral_separation = 120.0f;
    pid->derivative_alpha = 0.15f;
    pid->output_ramp = 6.0f;
}

/* 配置相对位移目标（X/Y，单位 mm）。 */
static void set_relative_motion_target(float x_mm, float y_mm)
{
    latch_motion_start();

    target_x_mm = x_mm;
    target_y_mm = y_mm;
    update_debug_wheel_targets();

    PID_Reset(&PID_Pos_X);
    PID_Reset(&PID_Pos_Y);

    manual_mode_enabled = 0U;
    manual_vx_cmd = 0.0f;
    manual_vy_cmd = 0.0f;
    manual_vw_cmd = 0.0f;
    position_mode_enabled = 1U;
}

/* 初始化底盘 PID 与运行时目标状态。 */
void Chassis_PID_Init(void)
{
    PID_Init(&PID_Pos_A, 0.20f, 0.010f, 0.008f, 100.0f, 35.0f);
    PID_Init(&PID_Pos_B, 0.20f, 0.010f, 0.008f, 100.0f, 35.0f);
    PID_Init(&PID_Pos_C, 0.20f, 0.010f, 0.008f, 100.0f, 35.0f);
    PID_Init(&PID_Pos_D, 0.20f, 0.010f, 0.008f, 100.0f, 35.0f);
    PID_Init(&PID_Pos_X, 0.30f, 0.003f, 0.020f, 65.0f, 30.0f);
    PID_Init(&PID_Pos_Y, 0.30f, 0.003f, 0.020f, 65.0f, 30.0f);

    apply_position_pid_tuning(&PID_Pos_A);
    apply_position_pid_tuning(&PID_Pos_B);
    apply_position_pid_tuning(&PID_Pos_C);
    apply_position_pid_tuning(&PID_Pos_D);
    apply_position_pid_tuning(&PID_Pos_X);
    apply_position_pid_tuning(&PID_Pos_Y);

    reset_motion_targets();
}

/* 复位底盘状态、PID 历史项与电机输出。 */
void Chassis_Control_ResetState(void)
{
    PID_Reset(&PID_Pos_A);
    PID_Reset(&PID_Pos_B);
    PID_Reset(&PID_Pos_C);
    PID_Reset(&PID_Pos_D);
    PID_Reset(&PID_Pos_X);
    PID_Reset(&PID_Pos_Y);

    reset_motion_targets();

    manual_vx_cmd = 0.0f;
    manual_vy_cmd = 0.0f;
    manual_vw_cmd = 0.0f;
    manual_mode_enabled = 0U;
    position_mode_enabled = 0U;

    motor_Stop();
}

/* 统一的编码器累计值读取接口。 */
int32_t Read_Encoder_Total(TIM_HandleTypeDef *htim, int32_t *encoder_count)
{
    (void)htim;
    return *encoder_count;
}

/* 设置速度控制指令，并退出位置模式。 */
void Chassis_Drive_Speed(float Vx, float Vy, float Vw)
{
    manual_vx_cmd = Vx;
    manual_vy_cmd = Vy;
    manual_vw_cmd = Vw;
    manual_mode_enabled = (fabsf(Vx) > 0.01f) || (fabsf(Vy) > 0.01f) || (fabsf(Vw) > 0.01f);
    position_mode_enabled = 0U;
}

/* 开启超声中线跟随，设置前进速度。 */
void Chassis_UltrasonicCenterline_Start(float forward_speed)
{
    ultrasonic_follow_forward_speed = clampf_local(forward_speed, -ULTRA_CENTERLINE_MAX_ABS_VX, ULTRA_CENTERLINE_MAX_ABS_VX);
    ultrasonic_centerline_enabled = 1U;
    position_mode_enabled = 0U;
}

/* 关闭超声中线跟随并清空运动指令。 */
void Chassis_UltrasonicCenterline_Stop(void)
{
    ultrasonic_centerline_enabled = 0U;
    Chassis_Drive_Speed(0.0f, 0.0f, 0.0f);
}

/* 根据左右超声更新中线跟随横向修正。 */
void Chassis_UltrasonicCenterline_Update(void)
{
    float left_cm;
    float right_cm;
    float left_norm;
    float right_norm;
    float lateral_error;
    float vy_cmd;
    uint8_t left_valid;
    uint8_t right_valid;

    if (!ultrasonic_centerline_enabled || position_mode_enabled) {
        return;
    }

    left_cm = SUPVC_GetDistanceCm(1);
    right_cm = SUPVC_GetDistanceCm(2);
    left_valid = (left_cm >= ULTRA_NORM_MIN_CM) && (left_cm <= ULTRA_NORM_MAX_CM);
    right_valid = (right_cm >= ULTRA_NORM_MIN_CM) && (right_cm <= ULTRA_NORM_MAX_CM);

    if (!left_valid && !right_valid) {
        Chassis_Drive_Speed(0.0f, 0.0f, 0.0f);
        return;
    }

    if (left_valid) {
        left_norm = normalize_distance_0_100(left_cm);
    } else {
        left_norm = ULTRA_NORM_CENTER_VALUE;
    }

    if (right_valid) {
        right_norm = normalize_distance_0_100(right_cm);
    } else {
        right_norm = ULTRA_NORM_CENTER_VALUE;
    }

    lateral_error = right_norm - left_norm;
    vy_cmd = clampf_local(ULTRA_CENTERLINE_KP * lateral_error,
                          -ULTRA_CENTERLINE_LATERAL_LIMIT,
                          ULTRA_CENTERLINE_LATERAL_LIMIT);

    Chassis_Drive_Speed(ultrasonic_follow_forward_speed, vy_cmd, 0.0f);
}

/* 常用方向运动封装。 */
/* 车体坐标系前进。 */
void Move_Forward(float speed) { Chassis_Drive_Speed(speed, 0.0f, 0.0f); }
/* 车体坐标系后退。 */
void Move_Backward(float speed) { Chassis_Drive_Speed(-speed, 0.0f, 0.0f); }
/* 车体坐标系左移。 */
void Move_Left(float speed) { Chassis_Drive_Speed(0.0f, -speed, 0.0f); }
/* 车体坐标系右移。 */
void Move_Right(float speed) { Chassis_Drive_Speed(0.0f, speed, 0.0f); }
/* 向左前方向运动。 */
void Move_TopLeft(float speed) { Chassis_Drive_Speed(speed, -speed, 0.0f); }
/* 向右前方向运动。 */
void Move_TopRight(float speed) { Chassis_Drive_Speed(speed, speed, 0.0f); }
/* 向左后方向运动。 */
void Move_BottomLeft(float speed) { Chassis_Drive_Speed(-speed, -speed, 0.0f); }
/* 向右后方向运动。 */
void Move_BottomRight(float speed) { Chassis_Drive_Speed(-speed, speed, 0.0f); }
/* 原地旋转。 */
void Rotate_In_Place(float speed) { Chassis_Drive_Speed(0.0f, 0.0f, speed); }

/* 对外接口：设置相对位移目标；W 仅为兼容保留参数。 */
void Chassis_Set_Target_Distance(float X_mm, float Y_mm, float W_deg)
{
    (void)W_deg;
    set_relative_motion_target(X_mm, Y_mm);
}

/* 判断是否在容差内到达目标位移。 */
uint8_t Chassis_Is_Position_Reached(void)
{
    float current_x_mm;
    float current_y_mm;

    get_body_displacement_mm(&current_x_mm, &current_y_mm);

    if (fabsf(target_x_mm - current_x_mm) > POSITION_TOLERANCE_MM) {
        return 0U;
    }
    if (fabsf(target_y_mm - current_y_mm) > POSITION_TOLERANCE_MM) {
        return 0U;
    }

    return 1U;
}

/* 100Hz 底盘控制环：位置模式或手动速度模式。 */
void Chassis_Position_Control_Loop(void)
{
    float wheel_a = 0.0f;
    float wheel_b = 0.0f;
    float wheel_c = 0.0f;
    float wheel_d = 0.0f;

    if (position_mode_enabled) {
        float current_x_mm;
        float current_y_mm;
        float error_x;
        float error_y;
        float cmd_vx;
        float cmd_vy;

        get_body_displacement_mm(&current_x_mm, &current_y_mm);

        error_x = target_x_mm - current_x_mm;
        error_y = target_y_mm - current_y_mm;

        PID_Pos_A.error = error_x;
        PID_Pos_B.error = error_y;
        PID_Pos_C.actual = current_x_mm;
        PID_Pos_D.actual = current_y_mm;

        cmd_vx = PID_Calc(&PID_Pos_X, target_x_mm, current_x_mm, CONTROL_DT);
        cmd_vy = PID_Calc(&PID_Pos_Y, target_y_mm, current_y_mm, CONTROL_DT);

        if (fabsf(error_x) < POSITION_BRAKE_WINDOW_MM) {
            cmd_vx *= 0.55f;
        }
        if (fabsf(error_y) < POSITION_BRAKE_WINDOW_MM) {
            cmd_vy *= 0.55f;
        }

        if ((fabsf(error_x) > POSITION_TOLERANCE_MM) && (fabsf(cmd_vx) < MIN_EFFECTIVE_PWM)) {
            cmd_vx = (cmd_vx >= 0.0f) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
        }
        if ((fabsf(error_y) > POSITION_TOLERANCE_MM) && (fabsf(cmd_vy) < MIN_EFFECTIVE_PWM)) {
            cmd_vy = (cmd_vy >= 0.0f) ? MIN_EFFECTIVE_PWM : -MIN_EFFECTIVE_PWM;
        }

        body_to_wheels(cmd_vx, cmd_vy, 0.0f, &wheel_a, &wheel_b, &wheel_c, &wheel_d);

        if (Chassis_Is_Position_Reached()) {
            position_mode_enabled = 0U;
            PID_Reset(&PID_Pos_X);
            PID_Reset(&PID_Pos_Y);
            reset_motion_targets();
            wheel_a = 0.0f;
            wheel_b = 0.0f;
            wheel_c = 0.0f;
            wheel_d = 0.0f;
        }
    } else if (manual_mode_enabled) {
        body_to_wheels(manual_vx_cmd, manual_vy_cmd, manual_vw_cmd, &wheel_a, &wheel_b, &wheel_c, &wheel_d);
    }

    if (!position_mode_enabled && !manual_mode_enabled) {
        wheel_a = 0.0f;
        wheel_b = 0.0f;
        wheel_c = 0.0f;
        wheel_d = 0.0f;
    }

    apply_wheel_outputs(wheel_a, wheel_b, wheel_c, wheel_d);
}

/* 车体坐标系相对位移封装（单位 mm）。 */
/* 相对前进指定距离。 */
void Move_Distance_Forward(float distance_mm)
{
    uart_printf("Moving forward %.1f mm\r\n", distance_mm);
    Chassis_Set_Target_Distance(distance_mm, 0.0f, 0.0f);
}

/* 相对后退指定距离。 */
void Move_Distance_Backward(float distance_mm)
{
    uart_printf("Moving backward %.1f mm\r\n", distance_mm);
    Chassis_Set_Target_Distance(-distance_mm, 0.0f, 0.0f);
}

/* 相对左移指定距离。 */
void Move_Distance_Left(float distance_mm)
{
    uart_printf("Moving left %.1f mm\r\n", distance_mm);
    Chassis_Set_Target_Distance(0.0f, -distance_mm, 0.0f);
}

/* 相对右移指定距离。 */
void Move_Distance_Right(float distance_mm)
{
    uart_printf("Moving right %.1f mm\r\n", distance_mm);
    Chassis_Set_Target_Distance(0.0f, distance_mm, 0.0f);
}

/* 解析并执行蓝牙串口命令。 */
void UART_Command_Process(uint8_t command)
{
    static float move_speed = 30.0f;

    switch (command) {
        case 'w':
        case 'W':
            Move_Forward(move_speed);
            break;
        case 's':
        case 'S':
            Move_Backward(move_speed);
            break;
        case 'a':
        case 'A':
            Move_Left(move_speed);
            break;
        case 'd':
        case 'D':
            Move_Right(move_speed);
            break;
        case 'q':
        case 'Q':
            Move_TopLeft(move_speed);
            break;
        case 'e':
        case 'E':
            Move_TopRight(move_speed);
            break;
        case 'z':
        case 'Z':
            Move_BottomLeft(move_speed);
            break;
        case 'c':
        case 'C':
            Move_BottomRight(move_speed);
            break;
        case 'x':
        case 'X':
            uart_printf("Emergency stop\r\n");
            Chassis_UltrasonicCenterline_Stop();
            Chassis_Control_ResetState();
            break;
        case 'r':
        case 'R':
            Rotate_In_Place(move_speed);
            break;
        case 't':
        case 'T':
            uart_printf("Ultrasonic centerline mode ON\r\n");
            Chassis_UltrasonicCenterline_Start(18.0f);
            break;
        case 'g':
        case 'G':
            uart_printf("Ultrasonic centerline mode OFF\r\n");
            Chassis_UltrasonicCenterline_Stop();
            break;
        case '4':
            Move_Distance_Forward(100.0f);
            break;
        case '5':
            Move_Distance_Backward(100.0f);
            break;
        case '6':
            Move_Distance_Left(100.0f);
            break;
        case '7':
            Move_Distance_Right(100.0f);
            break;
        case '8':
            Move_Distance_Forward(200.0f);
            break;
        case '9':
            Move_Distance_Backward(200.0f);
            break;
        case '0':
            Move_Distance_Left(200.0f);
            break;
        case '-':
        case '_':
            Move_Distance_Right(200.0f);
            break;
        default:
            uart_printf("Unknown command: %c\r\n", command);
            break;
    }
}



