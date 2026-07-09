#include "uart.h"
#include "jy901s.h"

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

void PRINT_INST_IRQHandler()
{
    switch (DL_UART_getPendingInterrupt(PRINT_INST))
    {
    case DL_UART_IIDX_RX:
        {
            while (!DL_UART_isRXFIFOEmpty(PRINT_INST)) {
                uint8_t rec = DL_UART_receiveData(PRINT_INST);
                JY901S_ReceiveByte(rec);
            }
            break;
        }

    case DL_UART_IIDX_OVERRUN_ERROR:
    case DL_UART_IIDX_BREAK_ERROR:
    case DL_UART_IIDX_PARITY_ERROR:
    case DL_UART_IIDX_FRAMING_ERROR:
    case DL_UART_IIDX_RX_TIMEOUT_ERROR:
    case DL_UART_IIDX_NOISE_ERROR:
        break;

    default:
        while (!DL_UART_isRXFIFOEmpty(PRINT_INST)) {
            uint8_t rec = DL_UART_receiveData(PRINT_INST);
            JY901S_ReceiveByte(rec);
        }
        break;
    }
}
