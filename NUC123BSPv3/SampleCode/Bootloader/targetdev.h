
#include "NUC123.h"
#include "isp_user.h"

#define DetectPin       PA13

/* rename for uart_transfer.c */
#define UART_N                          UART1
#define UART_N_IRQHandler       UART1_IRQHandler
#define UART_N_IRQn                 UART1_IRQn

