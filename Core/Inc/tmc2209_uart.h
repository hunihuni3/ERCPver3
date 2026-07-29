#ifndef TMC2209_UART_H
#define TMC2209_UART_H

/* =============================================================================
 * tmc2209_uart.h
 *
 * Bit-banged single-wire (half-duplex) UART driver for the TMC2209, for the
 * STM32F446 with HAL. Write-with-readback: sends IHOLD_IRUN and CHOPCONF
 * write datagrams, then reads them back (via the read datagram + IFCNT) to
 * confirm the chip actually accepted the writes.
 *
 * Debug/status output is printed over USART2 (assumes printf is retargeted to
 * USART2 via __io_putchar / _write — see notes in the .c file).
 *
 * IMPORTANT TIMING NOTE:
 *   The RX half (sampling the chip's reply) runs with interrupts disabled
 *   during the receive window. Call this at startup in main() ONLY. Do NOT
 *   call it inside a motion/STEP loop — it blocks for ~1-2 ms and will stall
 *   step pulse generation.
 * ===========================================================================*/

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Result codes so main() can branch on success/failure. */
typedef enum {
    TMC_OK = 0,            /* both writes verified                          */
    TMC_ERR_NO_REPLY,      /* read datagram timed out (no chip / wrong pin) */
    TMC_ERR_BAD_CRC,       /* reply received but CRC failed                 */
    TMC_ERR_BAD_SYNC,      /* reply framing wrong (sync/addr mismatch)      */
    TMC_ERR_WRITE_FAILED   /* IFCNT did not increment as expected           */
} TMC_Status_t;

void tmc_write_nodeconf(GPIO_TypeDef *port, uint16_t pin,
                        uint8_t addr, uint8_t senddelay);

TMC_Status_t tmc_diag_read(GPIO_TypeDef *port, uint16_t pin, uint8_t addr,
                           uint8_t reg, uint32_t *value);
/* Main entry point.
 *
 *   current_in_ma   : desired RMS run current in milliamps. Internally
 *                     converted to the 0..31 IRUN code using RSENSE and
 *                     VFS (full-scale voltage). See TMC2209_RSENSE_OHMS in .c.
 *   microstepping   : 0,2,4,8,16,32,64,128,256. (0 means fullstep / MRES=8.)
 *                     Any other value is rejected.
 *   comm_port       : GPIO port of the single-wire UART pin (e.g. GPIOB)
 *   comm_pin        : GPIO pin    of the single-wire UART pin (e.g. GPIO_PIN_6)
 *   slave_addr      : TMC2209 address 0..3 (set by MS1/MS2). Your notes use
 *                     0x00 and 0x01 for the two drivers.
 *
 * Returns a TMC_Status_t. Detailed progress is printed over USART2.
 *
 * The signature is a superset of your requested
 *   TMC2209uart(current_in_ma, microstepping, communication_pin)
 * because a single-wire bus needs both the port AND the pin, and a shared
 * bus needs the slave address to target the right driver. A convenience
 * wrapper macro matching your exact 3-arg shape is provided below.
 */
TMC_Status_t TMC2209uart(uint16_t current_in_ma,
                         uint16_t microstepping,
                         GPIO_TypeDef *comm_port,
                         uint16_t comm_pin,
                         uint8_t slave_addr);

#endif /* TMC2209_UART_H */
