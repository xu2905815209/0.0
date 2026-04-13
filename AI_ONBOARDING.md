# AI 快速上手文档

> 本文档专为 AI 助手设计，帮助快速理解项目背景和核心控制逻辑。

## 项目背景

- **赛事**：奥凯杯智能小车竞赛
- **任务**：麦克纳姆轮小车在走廊环境中自主循迹导航
- **核心要求**：小车沿走廊中线前进 + 保持正姿态（与墙壁平行）
- **当前版本**：v0.3（状态误差解算版）

## 硬件配置

### 麦克纳姆轮底盘（X型布局）
```
    前方（小车前进方向）
        ↑
  A(左前)    B(右前)
     |          |
     |   小车   |
     |          |
  C(左后)    D(右后)
```

### 超声波传感器布局

```
    前方
      ↑
 D5(左前/LF)  D4(右前/RF)
     |          |
     |   小车   |
     |          |
 D1(左后/LR)  D2(右后/RR)
```

| 通道 | 位置 | 型号 | 变量名 |
|------|------|------|--------|
| D5 | 左前 | US016 | LF (Left Front) |
| D4 | 右前 | US016 | RF (Right Front) |
| D1 | 左后 | HCSR04 | LR (Left Rear) |
| D2 | 右后 | HCSR04 | RR (Right Rear) |

## 核心控制策略（v0.3）

### 状态误差解算

**1. 偏航角度误差（Yaw Error）**：判断车体与墙壁的平行度
```
Yaw_Err = (LF - LR) - (RF - RR)
```
- `> 0`：车头偏右，需要向左自转纠正（W < 0）
- `< 0`：车头偏左，需要向右自转纠正（W > 0）

**2. 横向中心误差（Lateral Error）**：判断车体整体偏左还是偏右
```
Lat_Err = (LF + LR)/2 - (RF + RR)/2
```
- `> 0`：车体整体偏左，需要向右平移纠正（Vy > 0）
- `< 0`：车体整体偏右，需要向左平移纠正（Vy < 0）

### PID 闭环控制

- **前进速度 (Vx)**：设定为恒定的目标基准速度
- **平移速度 (Vy)**：以 `Lat_Err` 为输入，经过位置式 PID 计算输出
- **自转角速度 (W)**：以 `Yaw_Err` 为输入，经过位置式 PID 计算输出

### 麦克纳姆轮运动学逆解

```c
/* 符号调整宏定义（control.c 顶部，方便现场调车） */
#define MECANUM_VY_SIGN_A   -1.0f   /* 左前轮 Vy 符号 */
#define MECANUM_VY_SIGN_B    1.0f   /* 右前轮 Vy 符号 */
#define MECANUM_VY_SIGN_C    1.0f   /* 左后轮 Vy 符号 */
#define MECANUM_VY_SIGN_D   -1.0f   /* 右后轮 Vy 符号 */

#define MECANUM_W_SIGN_A    -1.0f   /* 左前轮 W 符号 */
#define MECANUM_W_SIGN_B     1.0f   /* 右前轮 W 符号 */
#define MECANUM_W_SIGN_C    -1.0f   /* 左后轮 W 符号 */
#define MECANUM_W_SIGN_D     1.0f   /* 右后轮 W 符号 */

/* 逆解公式 */
左前轮(A) = Vx + MECANUM_VY_SIGN_A * Vy + MECANUM_W_SIGN_A * W
右前轮(B) = Vx + MECANUM_VY_SIGN_B * Vy + MECANUM_W_SIGN_B * W
左后轮(C) = Vx + MECANUM_VY_SIGN_C * Vy + MECANUM_W_SIGN_C * W
右后轮(D) = Vx + MECANUM_VY_SIGN_D * Vy + MECANUM_W_SIGN_D * W
```

### 物理意义

- `Vy > 0`：小车向右横移（左侧轮向后转，右侧轮向前转）
- `W > 0`：小车逆时针旋转（左侧轮向后转，右侧轮向前转）

## 代码结构

### 核心文件
- `Core/Src/control.c`: 控制主逻辑
  - `mecanum_inverse_kinematics()`: 麦轮逆解（符号宏定义在函数前）
  - `run_line_mode_outer_loop()`: 状态误差解算 + PID 闭环
  - `run_speed_loop()`: 速度内环
  - `Control_10ms_Task()`: 10ms 控制主任务

### 关键数据结构
```c
typedef struct {
    float yaw_target;    /* 偏航角度目标（默认0） */
    float lat_target;    /* 横向位置目标（默认0） */
    float yaw_error;     /* 偏航角度误差 */
    float lat_error;     /* 横向中心误差 */

    PID_TypeDef yaw_pid; /* 偏航角度PID（控制W） */
    PID_TypeDef lat_pid; /* 横向位置PID（控制Vy） */
} ControlState_t;
```

## 调试命令

### 单字符命令（蓝牙推荐）
- `1`: 开启走廊循迹模式
- `2`: 停止循迹
- `6`/`7`: lat_target ±5（整体横向偏移）
- `[`/`]`: yaw_target ±5（偏航角度偏移）
- `?`: 查询当前状态

### CSV 命令
```
CMD,ERR,YAW,<target>     # 设置偏航角度目标
CMD,ERR,LAT,<target>     # 设置横向位置目标
CMD,ERR,GET              # 查询误差状态
CMD,PID,LAT,<kp>,<ki>,<kd>  # 设置横向位置PID
CMD,PID,YAW,<kp>,<ki>,<kd>  # 设置偏航角度PID
```

## 调参建议

### 调参顺序
1. **速度内环**（WHEEL PID）：先让轮子能跟踪目标速度
2. **横向位置环**（LAT PID）：调整横移响应
3. **偏航角度环**（YAW PID）：调整姿态修正

### 验证方向
小车靠近左墙时（LF,LR 值大，RF,RR 值小）：
- `Lat_Err > 0` → `Vy > 0` → 向右横移（远离左墙）✓
- 如果方向反了，修改 `MECANUM_VY_SIGN_*` 宏

---

**最后更新**：2026-04-07