#include "uart.h"
#include "wit_c_sdk.h"

static volatile uint32_t g_jy901s_rx_count = 0;
static volatile uint32_t g_jy901s_angle_count = 0;
static volatile uint32_t g_jy901s_header_count = 0;
static volatile uint32_t g_jy901s_error_count = 0;
static volatile uint32_t g_jy901s_baud_rate = 115200;
static volatile uint8_t g_jy901s_last_frame_type = 0;
static volatile uint8_t g_jy901s_recent_bytes[4] = {0};

static void JY901S_RecordByte(uint8_t data)
{
    g_jy901s_recent_bytes[3] = g_jy901s_recent_bytes[2];
    g_jy901s_recent_bytes[2] = g_jy901s_recent_bytes[1];
    g_jy901s_recent_bytes[1] = g_jy901s_recent_bytes[0];
    g_jy901s_recent_bytes[0] = data;

    if (data == 0x55U) {
        g_jy901s_header_count++;
    }
}

static void JY901S_ParseByte(uint8_t data)
{
    static uint8_t frame[11];
    static uint8_t frame_index = 0;
    uint8_t sum = 0;
    uint8_t i;

    if (frame_index == 0 && data != 0x55U) {
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

    g_jy901s_last_frame_type = frame[1];

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
        g_jy901s_angle_count++;
    }
}

void UART_send_char(UART_Regs *uart, const uint8_t chr)
{
    DL_UART_transmitDataBlocking(uart, chr);
}

void UART_send_string(UART_Regs *uart, const char *str)
{
    while (*str) {
        UART_send_char(uart, (uint8_t) *str);
        str++;
    }
}

uint32_t UART_get_jy901s_rx_count(void)
{
    return g_jy901s_rx_count;
}

uint32_t UART_get_jy901s_angle_count(void)
{
    return g_jy901s_angle_count;
}

uint32_t UART_get_jy901s_header_count(void)
{
    return g_jy901s_header_count;
}

uint8_t UART_get_jy901s_last_frame_type(void)
{
    return g_jy901s_last_frame_type;
}

uint8_t UART_get_jy901s_recent_byte(uint8_t index)
{
    if (index >= 4U) {
        return 0;
    }

    return g_jy901s_recent_bytes[index];
}

uint32_t UART_get_jy901s_error_count(void)
{
    return g_jy901s_error_count;
}

uint32_t UART_get_jy901s_baud_rate(void)
{
    return g_jy901s_baud_rate;
}

void UART_set_jy901s_baud_rate(uint32_t baud_rate)
{
    DL_UART_changeConfig(PRINT_INST);

    if (baud_rate == 9600U) {
        DL_UART_setBaudRateDivisor(PRINT_INST, 260U, 27U);
        g_jy901s_baud_rate = 9600U;
    } else {
        DL_UART_setBaudRateDivisor(PRINT_INST, PRINT_IBRD_40_MHZ_115200_BAUD,
                                   PRINT_FBRD_40_MHZ_115200_BAUD);
        g_jy901s_baud_rate = 115200U;
    }

    DL_UART_enable(PRINT_INST);
}

void PRINT_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(PRINT_INST))
    {
    case DL_UART_IIDX_RX:
        {
            while (!DL_UART_isRXFIFOEmpty(PRINT_INST)) {
                uint8_t rec = DL_UART_receiveData(PRINT_INST);
                g_jy901s_rx_count++;
                JY901S_RecordByte(rec);
                JY901S_ParseByte(rec);
                WitSerialDataIn(rec);
            }
            break;
        }

    case DL_UART_IIDX_OVERRUN_ERROR:
    case DL_UART_IIDX_BREAK_ERROR:
    case DL_UART_IIDX_PARITY_ERROR:
    case DL_UART_IIDX_FRAMING_ERROR:
    case DL_UART_IIDX_RX_TIMEOUT_ERROR:
    case DL_UART_IIDX_NOISE_ERROR:
        g_jy901s_error_count++;
        break;

    default:
        while (!DL_UART_isRXFIFOEmpty(PRINT_INST)) {
            uint8_t rec = DL_UART_receiveData(PRINT_INST);
            g_jy901s_rx_count++;
            JY901S_RecordByte(rec);
            JY901S_ParseByte(rec);
            WitSerialDataIn(rec);
        }
        break;
    }
}
