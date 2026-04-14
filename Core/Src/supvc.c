/**
 * @file supvc.c
 * @brief VL53L0X 六通道软件模拟 I2C 驱动
 *
 * 每个通道使用独立的 GPIO 引脚对模拟 I2C 总线，
 * 所有传感器均使用默认地址 0x29 (8位写地址 0x52)。
 */
#include "supvc.h"
#include "usart.h"
#include <string.h>

/* ============================================================================
 * VL53L0X 寄存器定义（精简版）
 * ============================================================================ */
#define VL53L0X_ADDR_W                  0x52  /* 0x29 << 1 */
#define VL53L0X_ADDR_R                  0x53  /* 0x29 << 1 | 1 */

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV 0x89
#define VL53L0X_REG_MSRC_CONFIG_CONTROL             0x60
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG           0x01
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define VL53L0X_REG_SYSRANGE_START                   0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS          0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS              0x14
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR           0x0B
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH         0x84
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO     0x0A

#define VL53L0X_MODEL_ID_EXPECTED       0xEE

/* 置 1 可打开底层六路测距原始日志；调参阶段默认关闭，避免串口被刷屏。 */
#define SUPVC_STREAM_LOG_ENABLE          0

/* ============================================================================
 * 数据存储
 * ============================================================================ */
static float    supvc_distance_cm[SUPVC_CHANNEL_COUNT];
static uint8_t  supvc_distance_valid[SUPVC_CHANNEL_COUNT];
static uint8_t  supvc_sensor_online[SUPVC_CHANNEL_COUNT]; /* 传感器初始化是否成功 */

/* ============================================================================
 * 软件 I2C 通道定义
 * ============================================================================ */
typedef struct {
    GPIO_TypeDef *SCL_Port;
    uint16_t      SCL_Pin;
    GPIO_TypeDef *SDA_Port;
    uint16_t      SDA_Pin;
} SoftI2C_Channel_t;

static const SoftI2C_Channel_t i2c_ch[SUPVC_CHANNEL_COUNT] = {
    {GPIOB, GPIO_PIN_0,  GPIOA, GPIO_PIN_5},   /* Ch1: PB0/PA5  */
    {GPIOB, GPIO_PIN_1,  GPIOB, GPIO_PIN_11},  /* Ch2: PB1/PB11 */
    {GPIOB, GPIO_PIN_5,  GPIOA, GPIO_PIN_6},   /* Ch3: PB5/PA6  */
    {GPIOC, GPIO_PIN_1,  GPIOA, GPIO_PIN_4},   /* Ch4: PC1/PA4  */
    {GPIOC, GPIO_PIN_2,  GPIOC, GPIO_PIN_0},   /* Ch5: PC2/PC0  */
    {GPIOB, GPIO_PIN_6,  GPIOB, GPIO_PIN_7}    /* Ch6: PB6/PB7  */
};

/* ============================================================================
 * 软件 I2C 底层实现
 * ============================================================================ */

/* 简单可靠的延迟：volatile 循环，约 5us @168MHz */
static void I2C_DELAY(void)
{
    volatile uint32_t i = 168;  /* ~5us at 168MHz */
    while (i--);
}

static void SoftI2C_SCL_High(const SoftI2C_Channel_t *ch)
{
    HAL_GPIO_WritePin(ch->SCL_Port, ch->SCL_Pin, GPIO_PIN_SET);
}
static void SoftI2C_SCL_Low(const SoftI2C_Channel_t *ch)
{
    HAL_GPIO_WritePin(ch->SCL_Port, ch->SCL_Pin, GPIO_PIN_RESET);
}
static void SoftI2C_SDA_High(const SoftI2C_Channel_t *ch)
{
    HAL_GPIO_WritePin(ch->SDA_Port, ch->SDA_Pin, GPIO_PIN_SET);
}
static void SoftI2C_SDA_Low(const SoftI2C_Channel_t *ch)
{
    HAL_GPIO_WritePin(ch->SDA_Port, ch->SDA_Pin, GPIO_PIN_RESET);
}
static uint8_t SoftI2C_SDA_Read(const SoftI2C_Channel_t *ch)
{
    return HAL_GPIO_ReadPin(ch->SDA_Port, ch->SDA_Pin);
}

static void SoftI2C_Start(const SoftI2C_Channel_t *ch)
{
    SoftI2C_SDA_High(ch);
    SoftI2C_SCL_High(ch);
    I2C_DELAY();
    SoftI2C_SDA_Low(ch);
    I2C_DELAY();
    SoftI2C_SCL_Low(ch);
    I2C_DELAY();
}

static void SoftI2C_Stop(const SoftI2C_Channel_t *ch)
{
    SoftI2C_SDA_Low(ch);
    I2C_DELAY();
    SoftI2C_SCL_High(ch);
    I2C_DELAY();
    SoftI2C_SDA_High(ch);
    I2C_DELAY();
}

static uint8_t SoftI2C_WaitAck(const SoftI2C_Channel_t *ch)
{
    uint16_t timeout = 0;
    SoftI2C_SDA_High(ch);  /* 释放 SDA，等待从机拉低 */
    I2C_DELAY();
    SoftI2C_SCL_High(ch);
    I2C_DELAY();
    while (SoftI2C_SDA_Read(ch) == GPIO_PIN_SET) {
        timeout++;
        if (timeout > 2000) {
            SoftI2C_Stop(ch);
            return 1; /* 超时，无应答 */
        }
        I2C_DELAY();  /* 每次检查之间等一下，给从机反应时间 */
    }
    SoftI2C_SCL_Low(ch);
    I2C_DELAY();
    return 0; /* 收到 ACK */
}

static void SoftI2C_Ack(const SoftI2C_Channel_t *ch)
{
    SoftI2C_SCL_Low(ch);
    SoftI2C_SDA_Low(ch);
    I2C_DELAY();
    SoftI2C_SCL_High(ch);
    I2C_DELAY();
    SoftI2C_SCL_Low(ch);
    I2C_DELAY();
}

static void SoftI2C_NAck(const SoftI2C_Channel_t *ch)
{
    SoftI2C_SCL_Low(ch);
    SoftI2C_SDA_High(ch);
    I2C_DELAY();
    SoftI2C_SCL_High(ch);
    I2C_DELAY();
    SoftI2C_SCL_Low(ch);
    I2C_DELAY();
}

static void SoftI2C_SendByte(const SoftI2C_Channel_t *ch, uint8_t byte)
{
    uint8_t i;
    SoftI2C_SCL_Low(ch);
    for (i = 0; i < 8; i++) {
        if (byte & 0x80) {
            SoftI2C_SDA_High(ch);
        } else {
            SoftI2C_SDA_Low(ch);
        }
        byte <<= 1;
        I2C_DELAY();
        SoftI2C_SCL_High(ch);
        I2C_DELAY();
        SoftI2C_SCL_Low(ch);
        I2C_DELAY();
    }
}

static uint8_t SoftI2C_ReadByte(const SoftI2C_Channel_t *ch, uint8_t ack)
{
    uint8_t i, res = 0;
    SoftI2C_SDA_High(ch);  /* 释放 SDA */
    for (i = 0; i < 8; i++) {
        SoftI2C_SCL_Low(ch);
        I2C_DELAY();
        SoftI2C_SCL_High(ch);
        I2C_DELAY();
        res <<= 1;
        if (SoftI2C_SDA_Read(ch) == GPIO_PIN_SET) {
            res |= 0x01;
        }
    }
    if (ack) {
        SoftI2C_Ack(ch);
    } else {
        SoftI2C_NAck(ch);
    }
    return res;
}

/* ============================================================================
 * VL53L0X 寄存器读写
 * ============================================================================ */
static uint8_t VL53L0X_WriteReg8(const SoftI2C_Channel_t *ch, uint8_t reg, uint8_t val)
{
    SoftI2C_Start(ch);
    SoftI2C_SendByte(ch, VL53L0X_ADDR_W);
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_SendByte(ch, reg);
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_SendByte(ch, val);
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_Stop(ch);
    return 0;
}

static uint8_t VL53L0X_WriteReg16(const SoftI2C_Channel_t *ch, uint8_t reg, uint16_t val)
{
    SoftI2C_Start(ch);
    SoftI2C_SendByte(ch, VL53L0X_ADDR_W);
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_SendByte(ch, reg);
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_SendByte(ch, (uint8_t)(val >> 8));
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_SendByte(ch, (uint8_t)(val & 0xFF));
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_Stop(ch);
    return 0;
}

static uint8_t VL53L0X_ReadReg8(const SoftI2C_Channel_t *ch, uint8_t reg, uint8_t *val)
{
    SoftI2C_Start(ch);
    SoftI2C_SendByte(ch, VL53L0X_ADDR_W);
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_SendByte(ch, reg);
    if (SoftI2C_WaitAck(ch)) return 1;

    SoftI2C_Start(ch);
    SoftI2C_SendByte(ch, VL53L0X_ADDR_R);
    if (SoftI2C_WaitAck(ch)) return 1;
    *val = SoftI2C_ReadByte(ch, 0); /* NACK for last byte */
    SoftI2C_Stop(ch);
    return 0;
}

static uint8_t VL53L0X_ReadReg16(const SoftI2C_Channel_t *ch, uint8_t reg, uint16_t *val)
{
    uint8_t hi, lo;
    SoftI2C_Start(ch);
    SoftI2C_SendByte(ch, VL53L0X_ADDR_W);
    if (SoftI2C_WaitAck(ch)) return 1;
    SoftI2C_SendByte(ch, reg);
    if (SoftI2C_WaitAck(ch)) return 1;

    SoftI2C_Start(ch);
    SoftI2C_SendByte(ch, VL53L0X_ADDR_R);
    if (SoftI2C_WaitAck(ch)) return 1;
    hi = SoftI2C_ReadByte(ch, 1);  /* ACK */
    lo = SoftI2C_ReadByte(ch, 0);  /* NACK */
    SoftI2C_Stop(ch);

    *val = ((uint16_t)hi << 8) | lo;
    return 0;
}

/* ============================================================================
 * VL53L0X 传感器初始化（精简版，兼容裸机无需官方 API）
 * 参考 Pololu VL53L0X Arduino Library 的 init() 流程
 * ============================================================================ */
static uint8_t VL53L0X_InitSensor(const SoftI2C_Channel_t *ch)
{
    uint8_t val8;
    uint32_t timeout;

    /* 1. 等待芯片启动完成：读 Model ID 直到返回 0xEE */
    timeout = HAL_GetTick();
    do {
        if (VL53L0X_ReadReg8(ch, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &val8) != 0) {
            return 1; /* I2C 通讯失败 */
        }
        if (val8 == VL53L0X_MODEL_ID_EXPECTED) break;
        if ((HAL_GetTick() - timeout) > 500) return 2; /* 超时 */
    } while (1);

    /* 2. 设置 2.8V I/O 模式（如果模块使用 2.8V 逻辑） */
    VL53L0X_ReadReg8(ch, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &val8);
    VL53L0X_WriteReg8(ch, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, val8 | 0x01);

    /* 3. 标准初始化序列（来自 ST API DataInit） */
    VL53L0X_WriteReg8(ch, 0x88, 0x00);
    VL53L0X_WriteReg8(ch, 0x80, 0x01);
    VL53L0X_WriteReg8(ch, 0xFF, 0x01);
    VL53L0X_WriteReg8(ch, 0x00, 0x00);

    /* 读取 stop_variable 用于后续单次测量 */
    VL53L0X_ReadReg8(ch, 0x91, &val8);
    /* 保存 stop_variable（全局或局部均可，此处简化处理） */

    VL53L0X_WriteReg8(ch, 0x00, 0x01);
    VL53L0X_WriteReg8(ch, 0xFF, 0x00);
    VL53L0X_WriteReg8(ch, 0x80, 0x00);

    /* 4. 配置 MSRC（最小信号率检查） */
    VL53L0X_ReadReg8(ch, VL53L0X_REG_MSRC_CONFIG_CONTROL, &val8);
    VL53L0X_WriteReg8(ch, VL53L0X_REG_MSRC_CONFIG_CONTROL, val8 | 0x12);

    /* 5. 设置信号速率限制为 0.25 MCPS (固定点 9.7 格式 = 0.25 * 128 = 32) */
    VL53L0X_WriteReg16(ch, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);

    /* 6. 设置测量序列配置 */
    VL53L0X_WriteReg8(ch, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* 7. 配置 GPIO 中断：新数据就绪时触发 */
    VL53L0X_WriteReg8(ch, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    VL53L0X_ReadReg8(ch, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, &val8);
    VL53L0X_WriteReg8(ch, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, val8 & ~0x10);
    VL53L0X_WriteReg8(ch, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* 8. 执行一次校准测量以验证传感器工作正常 */
    VL53L0X_WriteReg8(ch, VL53L0X_REG_SYSRANGE_START, 0x01);

    timeout = HAL_GetTick();
    do {
        VL53L0X_ReadReg8(ch, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &val8);
        if ((val8 & 0x07) != 0) break;
        if ((HAL_GetTick() - timeout) > 500) return 3; /* 首次测量超时 */
    } while (1);

    VL53L0X_WriteReg8(ch, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return 0; /* 初始化成功 */
}

/* ============================================================================
 * 公共接口
 * ============================================================================ */

void SUPVC_Init(void)
{
    uint8_t i;
    uint8_t init_result;

    /* 使能所有需要的 GPIO 时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 初始化所有通道的 GPIO 为开漏输出 + 内部上拉 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    for (i = 0; i < SUPVC_CHANNEL_COUNT; i++) {
        GPIO_InitStruct.Pin = i2c_ch[i].SCL_Pin;
        HAL_GPIO_Init(i2c_ch[i].SCL_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = i2c_ch[i].SDA_Pin;
        HAL_GPIO_Init(i2c_ch[i].SDA_Port, &GPIO_InitStruct);

        /* 释放总线（空闲态：高电平） */
        SoftI2C_SCL_High(&i2c_ch[i]);
        SoftI2C_SDA_High(&i2c_ch[i]);

        supvc_distance_cm[i]    = -1.0f;
        supvc_distance_valid[i] = 0;
        supvc_sensor_online[i]  = 0;
    }

    /* 等待传感器上电稳定（VL53L0X 需要至少 1.2ms，保守等待） */
    HAL_Delay(100);

    /* 逐个初始化 VL53L0X 传感器 */
    uart_printf("=== VL53L0X Init ===\r\n");
    for (i = 0; i < SUPVC_CHANNEL_COUNT; i++) {
        init_result = VL53L0X_InitSensor(&i2c_ch[i]);
        if (init_result == 0) {
            supvc_sensor_online[i] = 1;
            uart_printf("CH%d: OK\r\n", i + 1);
        } else {
            supvc_sensor_online[i] = 0;
            uart_printf("CH%d: FAIL (err=%d)\r\n", i + 1, init_result);
        }
    }
    uart_printf("=== Init Done ===\r\n");
}

void SUPVC_Service_10ms(void)
{
    /* 空函数，保留兼容接口 */
}

/* 非阻塞状态机：触发 -> 等待 -> 读取 */
static uint8_t supvc_measuring = 0;

void SUPVC_Service_MainLoop(void)
{
    static uint32_t last_ms = 0;
    uint32_t now_ms = HAL_GetTick();
    int i;

    if (!supvc_measuring) {
        if (now_ms - last_ms >= 50) { /* 20Hz */
            /* 触发所有在线传感器 */
            for (i = 0; i < SUPVC_CHANNEL_COUNT; i++) {
                if (!supvc_sensor_online[i]) continue;
                VL53L0X_WriteReg8(&i2c_ch[i], VL53L0X_REG_SYSRANGE_START, 0x01);
            }
            supvc_measuring = 1;
            last_ms = now_ms;
        }
    } else {
        if (now_ms - last_ms >= 40) { /* 等待 40ms 让测量完成 */
            for (i = 0; i < SUPVC_CHANNEL_COUNT; i++) {
                if (!supvc_sensor_online[i]) {
                    supvc_distance_valid[i] = 0;
                    supvc_distance_cm[i] = -1.0f;
                    continue;
                }

                uint8_t int_status = 0;
                if (VL53L0X_ReadReg8(&i2c_ch[i], VL53L0X_REG_RESULT_INTERRUPT_STATUS, &int_status) != 0) {
                    supvc_distance_valid[i] = 0;
                    supvc_distance_cm[i] = -1.0f;
                    continue;
                }

                if ((int_status & 0x07) != 0) {
                    /* 读取距离值（寄存器 0x14 + 10 = 0x1E） */
                    uint16_t range_mm = 0;
                    if (VL53L0X_ReadReg16(&i2c_ch[i], 0x14 + 10, &range_mm) == 0) {
                        VL53L0X_WriteReg8(&i2c_ch[i], VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

                        if (range_mm > 0 && range_mm < 8190) {
                            supvc_distance_cm[i] = range_mm / 10.0f;
                            supvc_distance_valid[i] = 1;
                        } else {
                            supvc_distance_cm[i] = -1.0f;
                            supvc_distance_valid[i] = 0;
                        }
                    }
                } else {
                    supvc_distance_valid[i] = 0;
                    supvc_distance_cm[i] = -1.0f;
                }
            }

            #if SUPVC_STREAM_LOG_ENABLE
            uart_printf("VL53L0X: ");
            for (i = 0; i < SUPVC_CHANNEL_COUNT; i++) {
                if (!supvc_sensor_online[i]) {
                    uart_printf("[%d:OFFLINE] ", i + 1);
                } else if (supvc_distance_valid[i]) {
                    uart_printf("[%d:%.1f] ", i + 1, supvc_distance_cm[i]);
                } else {
                    uart_printf("[%d:---] ", i + 1);
                }
            }
            uart_printf("\r\n");
            #endif

            supvc_measuring = 0;
            last_ms = now_ms;
        }
    }
}

float SUPVC_GetDistanceCm(uint8_t ch)
{
    if (ch < 1 || ch > SUPVC_CHANNEL_COUNT) return -1.0f;
    if (!supvc_distance_valid[ch - 1]) return -1.0f;
    return supvc_distance_cm[ch - 1];
}

uint8_t SUPVC_IsValid(uint8_t ch)
{
    if (ch < 1 || ch > SUPVC_CHANNEL_COUNT) return 0;
    return supvc_distance_valid[ch - 1];
}
