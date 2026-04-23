/*
 * uart.h
 *
 *  Created on: Apr 22, 2026
 *      Author: Ian Mcconachie
 */

#ifndef LIBRARIES_UART_H_
#define LIBRARIES_UART_H_

#include <stdint.h>

// Initialize UART0 on PA0/PA1 at 115200 baud for console I/O
void UART0_init(uint32_t ui32SysClock);

#endif /* LIBRARIES_UART_H_ */
