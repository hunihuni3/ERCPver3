/* =============================================================================
 * tmc2209_uart.c
 *
 * Bit-banged single-wire half-duplex UART for the TMC2209 on STM32F446 + HAL.
 *
 * --- WIRING ASSUMPTION (per your CubeMX setup) -------------------------------
 *   The comm pin is configured as GPIO_MODE_OUTPUT_OD (open-drain) with an
 *   external 1k pull-up to VIO (3.3V or 5V — match the TMC2209 VIO!). Because
 *   it is open-drain, driving the pin HIGH actually means "release the line and
 *   let the pull-up + the chip drive it" — which is exactly what half-duplex
 *   single-wire needs. We never need to flip the pin to input mode: in
 *   open-drain, writing 1 releases the line so the chip can pull it low to
 *   reply, and reading the input register still reflects the actual line state.
 *
 *   >> ARDUINO ANALOGY: open-drain output where writing HIGH = Hi-Z is like
 *   >> pinMode(pin, INPUT_PULLUP) that you can ALSO yank LOW on demand. One pin
 *   >> does both TX and RX. That's why single-wire UART works on it.
 *
 * --- TIMING ------------------------------------------------------------------
 *   Bit timing uses the DWT cycle counter (CPU clock = SystemCoreClock).
 *   No hardware timer/UART peripheral is used. Interrupts are disabled only
 *   during the receive sampling window to keep bit alignment.
 *
 * --- DEBUG OUTPUT ------------------------------------------------------------
 *   printf() goes to USART2. This requires you to have retargeted printf, e.g.
 *   in main.c or syscalls.c:
 *
 *       extern UART_HandleTypeDef huart2;
 *       int __io_putchar(int ch) {
 *           HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
 *           return ch;
 *       }
 *   (and add  #include <stdio.h>  + set "Use float with printf" off to save flash)
 * ===========================================================================*/

#include "tmc2209_uart.h"
#include <stdio.h>

/* ----------------------------- CONFIG --------------------------------------*/

/* Sense resistor. You selected 0.11 ohm (StepStick). NOTE: MKS TMC2209 v2.0
 * boards typically use 0.10 ohm — if your actual board is MKS, change this. */
#define TMC2209_RSENSE_OHMS   0.11f

/* Internal full-scale sense voltage of the TMC2209 current comparator.
 * Datasheet VFS is 325 mV (typ) in the default (high-sensitivity) mode used
 * here with VSENSE bit = 0 in CHOPCONF. */
#define TMC2209_VFS_VOLTS     0.325f

/* Bit-bang baud. 115200 is reliable at 168 MHz. Lower this for more margin. */
#define TMC2209_BAUD          115200u

/* Receive timeout: how long to wait for the chip to start its reply, in bit
 * periods. The chip replies ~8 bit times after the request; we allow generous
 * slack for line turnaround. */
#define TMC2209_RX_TIMEOUT_BITS  64u

/* TMC2209 UART datagram constants */
#define TMC_SYNC              0x05u   /* sync nibble: lower nibble 0b0101      */
#define TMC_WRITE_FLAG        0x80u   /* OR'd into register addr for a write   */

/* Register addresses */
#define TMC_REG_GCONF         0x00u
#define TMC_REG_IFCNT         0x02u   /* increments on each successful write   */
#define TMC_REG_IHOLD_IRUN    0x10u
#define TMC_REG_CHOPCONF      0x6Cu


#define TMC_REG_NODECONF      0x03u
/* ---------------------- DWT cycle-counter timing ---------------------------*/

static uint32_t g_cycles_per_bit;

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;   /* DWT->CYCCNT = 0; line DELETED */
}


/* Busy-wait until the cycle counter has advanced by one bit period from
 * *next_edge, then advance *next_edge. Using an absolute reference avoids
 * accumulating per-call overhead drift across a byte. */
static inline void wait_bit(uint32_t *next_edge)
{
    *next_edge += g_cycles_per_bit;
    while ((int32_t)(DWT->CYCCNT - *next_edge) < 0) { /* spin */ }
}

/* --------------------------- pin helpers -----------------------------------*/
/* Open-drain semantics: SET = release line (pull-up pulls high or chip drives),
 * RESET = actively pull low. */

static inline void line_release(GPIO_TypeDef *port, uint16_t pin) /* logic 1 */
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}
static inline void line_pull_low(GPIO_TypeDef *port, uint16_t pin) /* logic 0 */
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
static inline uint8_t line_read(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
}

/* --------------------------- CRC (TMC spec) --------------------------------*/
/* TMC UART CRC8, polynomial x^8 + x^2 + x + 1 (0x07), MSB-first, computed over
 * all datagram bytes except the CRC byte itself. This matches the reference
 * implementation in the TMC2209 datasheet.
 * Verified vectors: read GCONF {05,00,00}->0x48; CHOPCONF write
 * {05,00,EC,10,00,00,53}->0x9C. */
static uint8_t tmc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            b >>= 1;
        }
    }
    return crc;
}

/* --------------------------- UART TX byte ----------------------------------*/
/* Standard 8N1, LSB first. Start bit = 0, stop bit = 1. */
static void uart_tx_byte(GPIO_TypeDef *port, uint16_t pin, uint8_t b)
{
    uint32_t primask = __get_PRIMASK();
	__disable_irq();


	uint32_t edge = DWT->CYCCNT;

    /* Start bit */
    line_pull_low(port, pin);
    wait_bit(&edge);

    /* 8 data bits, LSB first */
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x01) line_release(port, pin);
        else          line_pull_low(port, pin);
        b >>= 1;
        wait_bit(&edge);
    }

    /* Stop bit (release line high) */
    line_release(port, pin);
    wait_bit(&edge);

    __set_PRIMASK(primask);
}

/* --------------------------- UART RX byte ----------------------------------*/
/* Waits for a start bit (line goes low) within `timeout_bits`, then samples
 * 8 data bits at mid-bit. Returns true on success. Caller disables IRQs. */
static bool uart_rx_byte(GPIO_TypeDef *port, uint16_t pin,
                         uint8_t *out, uint32_t timeout_bits)
{
    /* Wait for falling edge = start bit. */
    uint32_t deadline = DWT->CYCCNT + timeout_bits * g_cycles_per_bit;
    while (line_read(port, pin) != 0u) {
        if ((int32_t)(DWT->CYCCNT - deadline) >= 0) {
            return false; /* timed out, no start bit */
        }
    }

    /* We are now at the start-bit falling edge. Set our edge reference here,
     * then sample each data bit at its CENTRE (0.5 bit into each bit cell). */
    uint32_t edge = DWT->CYCCNT;

    /* Move to centre of bit 0: 1.5 bit periods from the falling edge
     * (0.5 to skip rest of start bit, +1.0 to reach centre of data bit 0). */
    edge += (g_cycles_per_bit * 3u) / 2u;
    while ((int32_t)(DWT->CYCCNT - edge) < 0) { }

    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (line_read(port, pin)) val |= (uint8_t)(1u << i); /* LSB first */
        edge += g_cycles_per_bit;
        while ((int32_t)(DWT->CYCCNT - edge) < 0) { }
    }
    /* We don't strictly need to verify the stop bit. */
    *out = val;
    return true;
}

/* --------------------------- datagram helpers ------------------------------*/

/* Send a 4-byte read-access request and receive the 8-byte reply.
 * Returns a TMC_Status_t; on TMC_OK, *value holds the 32-bit register value. */
static TMC_Status_t tmc_read_reg(GPIO_TypeDef *port, uint16_t pin,
                                 uint8_t slave_addr, uint8_t reg,
                                 uint32_t *value)
{
    uint8_t req[4];
    req[0] = TMC_SYNC;
    req[1] = slave_addr;
    req[2] = reg & 0x7Fu;                 /* read: top bit clear */
    req[3] = tmc_crc8(req, 3);

    /* Send request. */
    for (uint8_t i = 0; i < 4; i++) uart_tx_byte(port, pin, req[i]);

    /* Receive 8-byte reply. Disable IRQs so bit timing isn't disturbed. */
    uint8_t reply[8];
    bool ok = true;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0; i < 8; i++) {
        if (!uart_rx_byte(port, pin, &reply[i],
                          (i == 0) ? TMC2209_RX_TIMEOUT_BITS : 8u)) {
            ok = false;
            break;
        }
    }
    __set_PRIMASK(primask);

    if (!ok) return TMC_ERR_NO_REPLY;

    /* Reply framing: sync, master-addr (0xFF), reg, data[4], crc */
    if (reply[0] != TMC_SYNC || reply[1] != 0xFFu ||
        (reply[2] & 0x7Fu) != (reg & 0x7Fu)) {
        return TMC_ERR_BAD_SYNC;
    }
    if (tmc_crc8(reply, 7) != reply[7]) {
        return TMC_ERR_BAD_CRC;
    }

    *value = ((uint32_t)reply[3] << 24) | ((uint32_t)reply[4] << 16) |
             ((uint32_t)reply[5] << 8)  |  (uint32_t)reply[6];
    return TMC_OK;
}

/* Send a 8-byte write-access datagram. (No reply from chip on writes.) */
static void tmc_write_reg(GPIO_TypeDef *port, uint16_t pin,
                          uint8_t slave_addr, uint8_t reg, uint32_t value)
{
    uint8_t dg[8];
    dg[0] = TMC_SYNC;
    dg[1] = slave_addr;
    dg[2] = (uint8_t)(reg | TMC_WRITE_FLAG); /* write: top bit set */
    dg[3] = (uint8_t)(value >> 24);
    dg[4] = (uint8_t)(value >> 16);
    dg[5] = (uint8_t)(value >> 8);
    dg[6] = (uint8_t)(value);
    dg[7] = tmc_crc8(dg, 7);

    for (uint8_t i = 0; i < 8; i++) uart_tx_byte(port, pin, dg[i]);
}

/* Read IFCNT (write counter). Returns 0xFFFFFFFF on read failure. */
static uint32_t tmc_read_ifcnt(GPIO_TypeDef *port, uint16_t pin, uint8_t addr)
{
    uint32_t v;
    if (tmc_read_reg(port, pin, addr, TMC_REG_IFCNT, &v) == TMC_OK) {
        return v & 0xFFu;
    }
    return 0xFFFFFFFFu;
}

/* --------------------------- conversions -----------------------------------*/

/* Convert microstep count to the MRES nibble in CHOPCONF[27:24].
 * MRES is INVERSE: 0 = 256 usteps ... 8 = fullstep. Returns 0xFF if invalid. */
static uint8_t microsteps_to_mres(uint16_t usteps)
{
    switch (usteps) {
        case 256: return 0;
        case 128: return 1;
        case 64:  return 2;
        case 32:  return 3;
        case 16:  return 4;
        case 8:   return 5;
        case 4:   return 6;
        case 2:   return 7;
        case 0:   /* treat 0 as fullstep */
        case 1:   return 8;
        default:  return 0xFFu;
    }
}

/* Convert RMS current (mA) to the 5-bit IRUN code (0..31).
 *   I_rms = (CS+1)/32 * VFS / (Rsense + 0.02ohm) / sqrt(2)
 * Solve for CS, clamp to 0..31. The +0.02 accounts for internal sense path
 * resistance per the datasheet formula. */
static uint8_t current_ma_to_cs(uint16_t current_ma)
{
    float i_rms = (float)current_ma / 1000.0f;
    float denom = TMC2209_VFS_VOLTS /
                  ((TMC2209_RSENSE_OHMS + 0.02f) * 1.41421356f);
    /* i_rms = (CS+1)/32 * denom  ->  CS = 32*i_rms/denom - 1 */
    float cs_f = (32.0f * i_rms / denom) - 1.0f;
    if (cs_f < 0.0f)  cs_f = 0.0f;
    if (cs_f > 31.0f) cs_f = 31.0f;
    /* round to nearest */
    return (uint8_t)(cs_f + 0.5f);
}

/* --------------------------- public function -------------------------------*/

TMC_Status_t TMC2209uart(uint16_t current_in_ma,
                         uint16_t microstepping,
                         GPIO_TypeDef *comm_port,
                         uint16_t comm_pin,
                         uint8_t slave_addr)
{
    printf("\r\n[TMC2209] --- config start (addr=%u) ---\r\n", slave_addr);

    /* --- validate microstepping early --- */
    uint8_t mres = microsteps_to_mres(microstepping);
    if (mres == 0xFFu) {
        printf("[TMC2209] ERROR: invalid microstepping %u (use 0/1/2/4/8/16/32/64/128/256)\r\n",
               microstepping);
        return TMC_ERR_WRITE_FAILED;
    }

    /* --- one-time timing setup --- */
    dwt_init();
    g_cycles_per_bit = SystemCoreClock / TMC2209_BAUD;
    printf("[TMC2209] SysClk=%lu Hz, baud=%u, %lu cycles/bit\r\n",
           (unsigned long)SystemCoreClock, (unsigned)TMC2209_BAUD,
           (unsigned long)g_cycles_per_bit);

    /* idle = line released (high) */
    line_release(comm_port, comm_pin);

    /* --- sanity: can we even talk to the chip? read GCONF --- */
    uint32_t gconf = 0;
    TMC_Status_t rs = tmc_read_reg(comm_port, comm_pin, slave_addr,
                                   TMC_REG_GCONF, &gconf);
    if (rs != TMC_OK) {
        printf("[TMC2209] ERROR: no valid reply to GCONF read (code=%d). "
               "Check pin, pull-up, VIO, address.\r\n", (int)rs);
        return rs;
    }
    printf("[TMC2209] link OK. GCONF=0x%08lX\r\n", (unsigned long)gconf);


    /* baseline IFCNT before our writes */
    uint32_t ifcnt_before = tmc_read_ifcnt(comm_port, comm_pin, slave_addr);
    if (ifcnt_before == 0xFFFFFFFFu) {
        printf("[TMC2209] ERROR: IFCNT read failed\r\n");
        return TMC_ERR_NO_REPLY;
    }
    printf("[TMC2209] IFCNT before = %lu\r\n", (unsigned long)ifcnt_before);

    /* GCONF fixups — must precede IHOLD_IRUN and CHOPCONF writes.
         * Reset default 0x101 = I_scale_analog | multistep_filt.
         * Target 0x1C0 = pdn_disable | mstep_reg_select | multistep_filt. */
        uint32_t gconf_new = gconf;
        gconf_new &= ~(1u << 0);     /* I_scale_analog   = 0: IRUN sets current, not VREF pot */
        gconf_new |=  (1u << 6);     /* pdn_disable      = 1: PDN_UART is the UART line */
        gconf_new |=  (1u << 7);     /* mstep_reg_select = 1: MRES from CHOPCONF, not MS1/MS2 */
        gconf_new |=  (1u << 2);     /* en_spreadcycle = 1: no StealthChop AT dependency */
        printf("[TMC2209] GCONF 0x%08lX -> 0x%08lX\r\n",
                   (unsigned long)gconf, (unsigned long)gconf_new);
        tmc_write_reg(comm_port, comm_pin, slave_addr, TMC_REG_GCONF, gconf_new);


    /* --- compute and write IHOLD_IRUN --- */
    uint8_t irun  = current_ma_to_cs(current_in_ma);
    uint8_t ihold = (irun > 1) ? (uint8_t)(irun / 2) : irun; /* ~50% hold */
    uint8_t iholddelay = 8;

    uint32_t ihold_irun = ((uint32_t)(ihold & 0x1F)) |
                          ((uint32_t)(irun  & 0x1F) << 8) |
                          ((uint32_t)(iholddelay & 0x0F) << 16);

    printf("[TMC2209] target %u mA -> IRUN(CS)=%u, IHOLD=%u\r\n",
           current_in_ma, irun, ihold);
    tmc_write_reg(comm_port, comm_pin, slave_addr,
                  TMC_REG_IHOLD_IRUN, ihold_irun);

    /* --- read-modify-write CHOPCONF for MRES --- */
    uint32_t chopconf = 0;
    rs = tmc_read_reg(comm_port, comm_pin, slave_addr,
                      TMC_REG_CHOPCONF, &chopconf);
    if (rs != TMC_OK) {
        printf("[TMC2209] ERROR: CHOPCONF read failed (code=%d)\r\n", (int)rs);
        return rs;
    }
    /* If CHOPCONF is fully zero the chip likely has no usable TOFF; set a
     * sane default driver-enable config (TOFF=3, basic chopper). */
    if (chopconf == 0) {
        /* TOFF=3, HSTRT=4(-1 enc=4), HEND=0, TBL=2 (0x10000 area) — minimal
         * working SpreadCycle. Adjust to taste. */
        chopconf = (3u << 0) | (5u << 4) | (0u << 7) | (2u << 15);
        printf("[TMC2209] CHOPCONF was 0, applying default chopper bits\r\n");
    }
    chopconf &= ~(0x0Fu << 24);                 /* clear MRES field [27:24] */
    chopconf |=  ((uint32_t)mres << 24);        /* set new MRES             */
    /* ensure VSENSE=0 (high-current/full VFS) bit 17 — matches our calc */
    chopconf &= ~(1u << 17);

    printf("[TMC2209] microstep %u -> MRES=%u, CHOPCONF=0x%08lX\r\n",
           microstepping, mres, (unsigned long)chopconf);
    tmc_write_reg(comm_port, comm_pin, slave_addr,
                  TMC_REG_CHOPCONF, chopconf);

    /* --- verify both writes via IFCNT (should have +2) --- */
    uint32_t ifcnt_after = tmc_read_ifcnt(comm_port, comm_pin, slave_addr);
    if (ifcnt_after == 0xFFFFFFFFu) {
        printf("[TMC2209] ERROR: IFCNT readback failed\r\n");
        return TMC_ERR_NO_REPLY;
    }
    uint8_t delta = (uint8_t)(ifcnt_after - ifcnt_before); /* 8-bit wrap-safe */
    printf("[TMC2209] IFCNT after = %lu (delta=%u, expected 3)\r\n",
           (unsigned long)ifcnt_after, delta);

    if (delta < 3) {
        printf("[TMC2209] ERROR: writes not all accepted (delta=%u)\r\n", delta);
        return TMC_ERR_WRITE_FAILED;
    }

    /* --- optional content verify: read CHOPCONF back, check MRES --- */
    uint32_t chk = 0;
    if (tmc_read_reg(comm_port, comm_pin, slave_addr,
                     TMC_REG_CHOPCONF, &chk) == TMC_OK) {
        uint8_t mres_rb = (uint8_t)((chk >> 24) & 0x0F);
        if (mres_rb == mres) {
            printf("[TMC2209] verify OK: MRES readback=%u matches\r\n", mres_rb);
        } else {
            printf("[TMC2209] WARN: MRES readback=%u != %u\r\n", mres_rb, mres);
        }
    }

    printf("[TMC2209] --- config done OK ---\r\n");
    return TMC_OK;
}

void tmc_write_nodeconf(GPIO_TypeDef *port, uint16_t pin,
                        uint8_t addr, uint8_t senddelay)
{
    dwt_init();
    g_cycles_per_bit = SystemCoreClock / TMC2209_BAUD;
    line_release(port, pin);
    tmc_write_reg(port, pin, addr, TMC_REG_NODECONF,
                  ((uint32_t)(senddelay & 0x0F)) << 8);
}



TMC_Status_t tmc_diag_read(GPIO_TypeDef *port, uint16_t pin, uint8_t addr,
                           uint8_t reg, uint32_t *value)
{
    dwt_init();
    g_cycles_per_bit = SystemCoreClock / TMC2209_BAUD;
    line_release(port, pin);
    return tmc_read_reg(port, pin, addr, reg, value);
}
