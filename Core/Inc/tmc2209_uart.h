#ifndef TMC2209_UART_H
#define TMC2209_UART_H

/* Bit-banged single-wire TMC2209 UART driver. Call configuration and read
 * functions only outside the motion loop because receive sampling briefly
 * disables interrupts. */

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TMC_OK = 0,
    TMC_ERR_NO_REPLY,
    TMC_ERR_BAD_CRC,
    TMC_ERR_BAD_SYNC,
    TMC_ERR_WRITE_FAILED
} TMC_Status_t;

void tmc_write_nodeconf(GPIO_TypeDef *port, uint16_t pin,
                        uint8_t addr, uint8_t senddelay);

TMC_Status_t tmc_diag_read(GPIO_TypeDef *port, uint16_t pin, uint8_t addr,
                           uint8_t reg, uint32_t *value);

TMC_Status_t TMC2209uart(uint16_t current_in_ma,
                         uint16_t microstepping,
                         GPIO_TypeDef *comm_port,
                         uint16_t comm_pin,
                         uint8_t slave_addr);

#endif /* TMC2209_UART_H */
