#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

void UART_send_string(UART_Regs *uart, const char *str);
void UART_send_char(UART_Regs *uart, const uint8_t chr);
uint32_t UART_get_jy901s_rx_count(void);
uint32_t UART_get_jy901s_angle_count(void);
uint32_t UART_get_jy901s_header_count(void);
uint8_t UART_get_jy901s_last_frame_type(void);
uint8_t UART_get_jy901s_recent_byte(uint8_t index);
uint32_t UART_get_jy901s_error_count(void);
uint32_t UART_get_jy901s_baud_rate(void);
void UART_set_jy901s_baud_rate(uint32_t baud_rate);

#endif /* UART_H */
