#include <stdio.h>
#include "..\..\MyLibrary\HAL_Driver\hal_sys_init.h"
#include "cdc_serial.h"

#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

extern volatile uint8_t comRbuf[];
extern volatile uint8_t comTbuf[];
extern uint8_t gRxBuf[64];


#define UART_PORT   0

#if UART_PORT == 0
# define UART            UART0
# define UART_IRQn       UART0_IRQn
# define UART_IRQHandler UART0_IRQHandler
# define SYS_Init_UART() SYS_Init_UART0()
#elif UART_PORT == 1
# define UART            UART1
# define UART_IRQn       UART1_IRQn
# define UART_IRQHandler UART1_IRQHandler
# define SYS_Init_UART() SYS_Init_UART1()
#else
# define UART            UART0
# define UART_IRQn       UART0_IRQn
# define UART_IRQHandler UART0_IRQHandler
# define SYS_Init_UART() SYS_Init_UART0()
#endif

typedef void (*BRIDGE_FUNC)(void);

extern volatile BRIDGE_FUNC s_BridgeInitFn;
extern volatile BRIDGE_FUNC s_BridgeMainFn;
