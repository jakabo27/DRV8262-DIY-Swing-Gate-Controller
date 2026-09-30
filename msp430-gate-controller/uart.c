/*******************************************************************************
 * uart.c - see uart.h. The ISR here ONLY moves raw bytes and updates counters -
 * it must never call into console.c/dbg_Print (project_plan.md Rev 3 Sec 9.4).
 ******************************************************************************/
#include <msp430.h>
#include "uart.h"
#include "gate_config.h"

#define TX_MASK (UART_TX_RING_LEN - 1u)
#define RX_MASK (UART_RX_RING_LEN - 1u)

static volatile uint8_t  s_txBuf[UART_TX_RING_LEN];
static volatile uint16_t s_txHead = 0, s_txTail = 0;
static volatile uint8_t  s_rxBuf[UART_RX_RING_LEN];
static volatile uint16_t s_rxHead = 0, s_rxTail = 0;
static volatile uint16_t s_txOverrun = 0, s_rxOverrun = 0;

void uart_Init(void)
{
    UCA1CTL1  = UCSWRST;                   /* hold USCI in reset while configuring */
    UCA1CTL1 |= UCSSEL_2;                  /* BRCLK = SMCLK (~20 MHz)              */
    UCA1BR0   = UART_UCA1BR0;
    UCA1BR1   = UART_UCA1BR1;
    UCA1MCTL  = UART_UCA1MCTL;             /* UCBRF_14 | UCOS16 -> ~115 kbps ~0.22%*/
    UCA1CTL1 &= ~UCSWRST;
    UCA1IE   |= UCRXIE;                    /* TX interrupt enabled only while data pending */
}

bool uart_PutByte(uint8_t b)
{
    uint16_t next = (uint16_t)((s_txHead + 1u) & TX_MASK);
    if (next == s_txTail) {
        s_txOverrun++;
        return false;
    }
    s_txBuf[s_txHead] = b;

    /* Re-arm the interrupt-driven drain. This must be atomic against the TX ISR:
     * if the ISR's "queue now empty -> UCA1IE &= ~UCTXIE" lands right after a
     * bare "UCA1IE |= UCTXIE" here, the byte just queued sits forever with the
     * TX interrupt disabled (observed on the bench: ~230 bytes of console output
     * stuck in the ring). Advancing s_txHead and re-enabling under __disable_
     * interrupt() closes that window. UCA1IE |= UCTXIE by itself does NOT
     * reliably re-trigger once TX has gone fully idle, so also set UCTXIFG to
     * force the first interrupt. uart_PutByte is only ever called from the
     * super-loop (never an ISR - project_plan Rev 3 Sec 9.4), so GIE is on. */
    __disable_interrupt();
    s_txHead = next;
    if ((UCA1IE & UCTXIE) == 0) {
        UCA1IE  |= UCTXIE;
        UCA1IFG |= UCTXIFG;
    }
    __enable_interrupt();
    return true;
}

bool uart_GetByte(uint8_t *out)
{
    if (s_rxTail == s_rxHead) {
        return false;
    }
    *out = s_rxBuf[s_rxTail];
    s_rxTail = (uint16_t)((s_rxTail + 1u) & RX_MASK);
    return true;
}

bool uart_TxIdle(void)
{
    return s_txTail == s_txHead;
}

uint16_t uart_TxFree(void)
{
    /* one slot is always kept empty to distinguish full from empty */
    uint16_t used = (uint16_t)((s_txHead - s_txTail) & TX_MASK);
    return (uint16_t)(TX_MASK - used);
}

uint16_t uart_TxOverrunCount(void) { return s_txOverrun; }
uint16_t uart_RxOverrunCount(void) { return s_rxOverrun; }

#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch (__even_in_range(UCA1IV, 4)) {
        case 0:                            /* no interrupt pending */
            break;
        case 2: {                          /* UCRXIFG */
            uint8_t b = UCA1RXBUF;
            uint16_t next = (uint16_t)((s_rxHead + 1u) & RX_MASK);
            if (next != s_rxTail) {
                s_rxBuf[s_rxHead] = b;
                s_rxHead = next;
            } else {
                s_rxOverrun++;              /* console isn't keeping up - drop it */
            }
            break;
        }
        case 4:                            /* UCTXIFG */
            if (s_txTail != s_txHead) {
                UCA1TXBUF = s_txBuf[s_txTail];
                s_txTail = (uint16_t)((s_txTail + 1u) & TX_MASK);
            } else {
                UCA1IE &= ~UCTXIE;          /* nothing left to send */
            }
            break;
        default:
            break;
    }
}
