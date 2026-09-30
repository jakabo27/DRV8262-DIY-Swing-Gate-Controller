/*******************************************************************************
 * uart.h - raw, non-blocking UCA1 UART driver (115200 8N1). Bytes only - no
 * formatting, no printing. console.c builds human-readable I/O on top of this.
 ******************************************************************************/
#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

void uart_Init(void);

/* Enqueues one byte for transmission; returns false (byte dropped) if the TX
 * ring is full. Never blocks. Safe to call only from main-loop context. */
bool uart_PutByte(uint8_t b);

/* Pops one received byte if available. Never blocks. */
bool uart_GetByte(uint8_t *out);

bool uart_TxIdle(void);

/* Bytes that can be queued right now without dropping (TX ring free space).
 * Used by the capture dump to pace a large CSV burst against the drain rate. */
uint16_t uart_TxFree(void);

uint16_t uart_TxOverrunCount(void);
uint16_t uart_RxOverrunCount(void);

#endif /* UART_H */
