#include "jy901s.h"

#include "REG.h"
#include "delay.h"
#include "ti_msp_dl_config.h"
#include "uart.h"
#include "wit_c_sdk.h"

static volatile uint32_t g_rx_count = 0;
static volatile uint32_t g_angle_frame_count = 0;

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
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitRegisterCallBack(JY901S_RegUpdate);
    WitSerialWriteRegister(JY901S_SerialWrite);
    WitDelayMsRegister(JY901S_DelayMs);

    WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);
    delay_ms(50);
    WitSetOutputRate(RRATE_10HZ);
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

    if (frame[1] == WIT_ACC) {
        sReg[AX] = (int16_t)((uint16_t)frame[3] << 8 | frame[2]);
        sReg[AY] = (int16_t)((uint16_t)frame[5] << 8 | frame[4]);
        sReg[AZ] = (int16_t)((uint16_t)frame[7] << 8 | frame[6]);
        sReg[TEMP] = (int16_t)((uint16_t)frame[9] << 8 | frame[8]);
    } else if (frame[1] == WIT_GYRO) {
        sReg[GX] = (int16_t)((uint16_t)frame[3] << 8 | frame[2]);
        sReg[GY] = (int16_t)((uint16_t)frame[5] << 8 | frame[4]);
        sReg[GZ] = (int16_t)((uint16_t)frame[7] << 8 | frame[6]);
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

uint32_t JY901S_GetRxCount(void)
{
    return g_rx_count;
}

uint32_t JY901S_GetAngleFrameCount(void)
{
    return g_angle_frame_count;
}
