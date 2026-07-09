#include "jy901s.h"

#include "REG.h"
#include "delay.h"
#include "ti_msp_dl_config.h"
#include "uart.h"
#include "wit_c_sdk.h"

#define JY901S_GYRO_OUTPUT_RATE_HZ      20L
#define JY901S_GYRO_RAW_TO_DPS_NUM      2000L
#define JY901S_GYRO_RAW_TO_DPS_DEN      32768L
#define JY901S_GYRO_DEADBAND_RAW        3L
#define JY901S_MDEG_PER_DEG             1000L

static volatile uint32_t g_rx_count = 0;
static volatile uint32_t g_gyro_frame_count = 0;
static volatile uint32_t g_angle_frame_count = 0;
static volatile uint8_t g_last_frame_type = 0;
static volatile int32_t g_gyro_z_bias_raw = 0;
static volatile int32_t g_gyro_z_turn_angle_mdeg = 0;

static void JY901S_UpdateGyroZTurnAngle(int16_t gz_raw)
{
    int32_t corrected_raw = (int32_t)gz_raw - g_gyro_z_bias_raw;
    int32_t delta_mdeg;

    if ((corrected_raw > -JY901S_GYRO_DEADBAND_RAW) &&
        (corrected_raw < JY901S_GYRO_DEADBAND_RAW)) {
        corrected_raw = 0;
    }

    delta_mdeg = (int32_t)(((int64_t)corrected_raw *
                            JY901S_GYRO_RAW_TO_DPS_NUM *
                           JY901S_MDEG_PER_DEG) /
                           ((int64_t)JY901S_GYRO_RAW_TO_DPS_DEN *
                            JY901S_GYRO_OUTPUT_RATE_HZ));
    g_gyro_z_turn_angle_mdeg += delta_mdeg;
}

static void JY901S_RegUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
    (void)uiReg;
    (void)uiRegNum;
}

static void JY901S_SerialWrite(uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        UART_send_char(PRINT_INST, data[i]);
    }
}

static void JY901S_DelayMs(uint16_t ms)
{
    delay_ms(ms);
}

void JY901S_Init(void)
{
    uint8_t i;

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitRegisterCallBack(JY901S_RegUpdate);
    WitSerialWriteRegister(JY901S_SerialWrite);
    WitDelayMsRegister(JY901S_DelayMs);

    delay_ms(200);
    for (i = 0; i < 3U; i++) {
        WitSetContent(RSW_GYRO | RSW_ANGLE);
        delay_ms(50);
        WitSetOutputRate(RRATE_20HZ);
        delay_ms(50);
    }
}

void JY901S_ReceiveByte(uint8_t data)
{
    static uint8_t frame[11];
    static uint8_t frame_index = 0;
    uint8_t sum = 0;
    uint8_t i;

    g_rx_count++;
    WitSerialDataIn(data);

    if (frame_index == 0U && data != 0x55U) {
        return;
    }

    frame[frame_index++] = data;

    if (frame_index < 11U) {
        return;
    }

    frame_index = 0;
    for (i = 0; i < 10U; i++) {
        sum += frame[i];
    }

    if (sum != frame[10]) {
        return;
    }

    g_last_frame_type = frame[1];

    if (frame[1] == WIT_ACC) {
        sReg[AX] = (int16_t)((uint16_t)frame[3] << 8 | frame[2]);
        sReg[AY] = (int16_t)((uint16_t)frame[5] << 8 | frame[4]);
        sReg[AZ] = (int16_t)((uint16_t)frame[7] << 8 | frame[6]);
        sReg[TEMP] = (int16_t)((uint16_t)frame[9] << 8 | frame[8]);
    } else if (frame[1] == WIT_GYRO) {
        sReg[GX] = (int16_t)((uint16_t)frame[3] << 8 | frame[2]);
        sReg[GY] = (int16_t)((uint16_t)frame[5] << 8 | frame[4]);
        sReg[GZ] = (int16_t)((uint16_t)frame[7] << 8 | frame[6]);
        g_gyro_frame_count++;
        JY901S_UpdateGyroZTurnAngle(sReg[GZ]);
    } else if (frame[1] == WIT_ANGLE) {
        sReg[Roll] = (int16_t)((uint16_t)frame[3] << 8 | frame[2]);
        sReg[Pitch] = (int16_t)((uint16_t)frame[5] << 8 | frame[4]);
        sReg[Yaw] = (int16_t)((uint16_t)frame[7] << 8 | frame[6]);
        g_angle_frame_count++;
    }
}

void JY901S_GetAngleRaw(int16_t *roll, int16_t *pitch, int16_t *yaw)
{
    if (roll != 0) {
        *roll = sReg[Roll];
    }
    if (pitch != 0) {
        *pitch = sReg[Pitch];
    }
    if (yaw != 0) {
        *yaw = sReg[Yaw];
    }
}

int32_t JY901S_RawToAngleX10(int16_t raw)
{
    return ((int32_t)raw * 1800L) / 32768L;
}

void JY901S_GetAngle(JY901S_Angle_t *angle)
{
    if (angle == 0) {
        return;
    }

    JY901S_GetAngleRaw(&angle->roll_raw, &angle->pitch_raw, &angle->yaw_raw);
    angle->roll_x10 = JY901S_RawToAngleX10(angle->roll_raw);
    angle->pitch_x10 = JY901S_RawToAngleX10(angle->pitch_raw);
    angle->yaw_x10 = JY901S_RawToAngleX10(angle->yaw_raw);
}

uint8_t JY901S_CalibrateGyroZ(uint16_t sample_count, uint16_t timeout_ms)
{
    uint32_t start_count;
    uint32_t last_count;
    int32_t sum = 0;
    uint16_t collected = 0;
    uint16_t elapsed_ms = 0;

    if (sample_count == 0U) {
        return 0U;
    }

    __disable_irq();
    start_count = g_gyro_frame_count;
    last_count = start_count;
    __enable_irq();

    while ((collected < sample_count) && (elapsed_ms < timeout_ms)) {
        uint32_t current_count;
        int16_t gz_raw;

        __disable_irq();
        current_count = g_gyro_frame_count;
        gz_raw = sReg[GZ];
        __enable_irq();

        if (current_count != last_count) {
            last_count = current_count;
            sum += gz_raw;
            collected++;
        } else {
            delay_ms(1);
            elapsed_ms++;
        }
    }

    if (collected == 0U) {
        return 0U;
    }

    __disable_irq();
    g_gyro_z_bias_raw = sum / (int32_t)collected;
    g_gyro_z_turn_angle_mdeg = 0;
    __enable_irq();

    return 1U;
}

void JY901S_ResetGyroZTurnAngle(void)
{
    __disable_irq();
    g_gyro_z_turn_angle_mdeg = 0;
    __enable_irq();
}

int32_t JY901S_GetGyroZTurnAngleX10(void)
{
    int32_t angle_x10;

    __disable_irq();
    angle_x10 = g_gyro_z_turn_angle_mdeg / 100L;
    __enable_irq();

    return angle_x10;
}

uint32_t JY901S_GetRxCount(void)
{
    return g_rx_count;
}

uint32_t JY901S_GetGyroFrameCount(void)
{
    return g_gyro_frame_count;
}

uint32_t JY901S_GetAngleFrameCount(void)
{
    return g_angle_frame_count;
}

uint8_t JY901S_GetLastFrameType(void)
{
    return g_last_frame_type;
}
