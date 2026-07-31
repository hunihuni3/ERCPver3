/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc2209_uart.h"
#include <math.h> //for sqrtf and fasbf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MICROSTEP 16
#define CPR 2048

/* Temporary bench mode:
 * 1 = keep M2/M3 disabled, skip their homing, and run M1/M4/M5 only.
 * 0 = restore the complete machine including mandatory M2/M3 homing. */
#define M45_ONLY_TEST_MODE 1u

/* Exact counts->microsteps ratio.
 *   3200 microsteps/rev  /  8192 enc counts/rev  =  25/64  (exact)
 * If your AMT102 DIPs are NOT at 2048 PPR (8192 counts), change ENC_CPR. */
#define MICROSTEPS_PER_REV   (200L * MICROSTEP)
#define ENC_CPR              (CPR * 4)
#define RATIO_NUM            MICROSTEPS_PER_REV   /* 3200 */
#define RATIO_DEN            ENC_CPR              /* 8192 -> reduces to 25/64 */

/* STEP pulse high time, in CPU cycles. At 168 MHz, 168 cycles ~= 1 us. */
#define STEP_PULSE_CYCLES    (SystemCoreClock / 1000000u)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
typedef enum {

    MODE_PRECISION = 2,

    MODE_FAST = 3

} Mode;

typedef enum {

    TOOL_CATHETER = 0,

    TOOL_GUIDE = 1,

    TOOL_CATHETER_RETRACT_WIRE_HOLD = 2

} Tool;

volatile Mode current_mode = MODE_PRECISION; 					// MODE_PRECISION is just 1:1 matching mode. encoders turned out to be accurate enough to just 1:1 match.

volatile Tool current_tool = TOOL_CATHETER;

/* DWT cycle counter init — call once before the loop. Used for the ~1us pulse. */
static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Busy-wait a number of CPU cycles using the DWT cycle counter. */
static inline void dwt_delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { /* spin */ }
}

/* Emit one STEP pulse on Motor 1. DIR must already be set before calling. */
static inline void step_pulse(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    dwt_delay_cycles(STEP_PULSE_CYCLES);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    dwt_delay_cycles(STEP_PULSE_CYCLES);
}

#define V_MAX 51200.0f //steps/s ceiling
#define A_MAX 160000.0f //steps/s^2 max
#define V_MIN 0.5f 	// below this treat as stopped
//values are all calibrated in reality

//axis struct + config table..
/* ---- 4 input encoders: enc1..enc4 = TIM1,2,3,4 ---- */
typedef struct { TIM_HandleTypeDef *tim; uint8_t is32; uint32_t prev; } InEnc;

static InEnc in_enc[4] = {
    { &htim1, 0, 0 },   /* enc1 = TIM1 (16-bit) -> M1 */
    { &htim2, 1, 0 },   /* enc2 = TIM2 (32-bit) -> M2 */
    { &htim3, 0, 0 },   /* enc3 = TIM3 (16-bit) -> M3 */
    { &htim4, 0, 0 },   /* enc4 = TIM4 (16-bit) -> M4 or M5 (button) */
};

static int32_t in_enc_delta(InEnc *e) {
    uint32_t now = __HAL_TIM_GET_COUNTER(e->tim);
    int32_t d = e->is32 ? (int32_t)(now - e->prev)
                        : (int16_t)((uint16_t)now - (uint16_t)e->prev);
    e->prev = now;
    return d;
}

/* ---- 5 motors ---- */
typedef struct {
    GPIO_TypeDef *step_port; uint16_t step_pin;
    GPIO_TypeDef *dir_port;  uint16_t dir_pin;
    GPIO_TypeDef *en_port;   uint16_t en_pin;
    GPIO_TypeDef *pdn_port;  uint16_t pdn_pin;   /* this motor's own TMC single-wire line */
    uint8_t  tmc_addr;
    uint8_t  gain_enabled;   /* 1 = mode gain (100/300) applies; 0 = fixed 100 (1:1 absolute) */
    int32_t  gear_num;       /* controller->motor gear correction, exact rational */
    int32_t  gear_den;
    int32_t  dir_sign;       /* +1 normal, -1 inverted */
    int64_t  accum_num;
    int32_t  target_pos, current_pos;
    float    vel, emit_accum;
} Motor;

#define N_MOT 5
#define TMC_INIT_RETRIES 3u
#define TMC_INIT_RETRY_DELAY_MS 100u
#define TMC_INIT_AXIS_DELAY_MS 1000u

/* STEP/DIR/EN below are read from your schematic and are confident.
 * pdn_pin for M3/M4/M5 and all tmc_addr are NOT — see confirm-list. */

// gain_en determines if motor is subject to gain variation by gain button; only enable for feeder, not position tracking knobs.
// gear D is input gear, gearN is output gear.
static Motor mot[N_MOT] = {
  /* step               dir                en                pdn(TMC)          addr gain gearN gearD sign runtime state */
  { GPIOC,GPIO_PIN_1,  GPIOC,GPIO_PIN_2,  GPIOC,GPIO_PIN_0,  GPIOB,GPIO_PIN_0,  0, 0,  95,  26, -1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOC,GPIO_PIN_4,  GPIOC,GPIO_PIN_5,  GPIOC,GPIO_PIN_3,  GPIOB,GPIO_PIN_0,  1, 0, 27803, 100000, +1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOC,GPIO_PIN_9,  GPIOC,GPIO_PIN_10, GPIOC,GPIO_PIN_8,  GPIOB,GPIO_PIN_0,  2, 0, 25961, 100000, +1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOC,GPIO_PIN_12, GPIOC,GPIO_PIN_13, GPIOC,GPIO_PIN_11, GPIOB,GPIO_PIN_0,  3, 1,   1,   1, +1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOA,GPIO_PIN_11, GPIOD,GPIO_PIN_2,  GPIOA,GPIO_PIN_12, GPIOB,GPIO_PIN_1,  0, 1,   1,   1, +1, 0, 0, 0, 0.0f, 0.0f },
};

/* A driver remains disabled unless its UART configuration was verified. */
static uint8_t motor_ready[N_MOT] = {0};

static uint8_t motor_enabled_for_mode(uint8_t mi)
{
#if M45_ONLY_TEST_MODE
    if (mi == 1u || mi == 2u) return 0u; /* M2/M3 */
#endif
    return 1u;
}

/* In coupled mode M5 receives the opposite logical command. Change this to
 * +1 only if the installed roller geometry requires the same motor direction
 * to produce opposite catheter/wire travel. */
#define COUPLED_M5_ENCODER_SIGN (-1)

typedef enum {
    SEL_M4 = 0,
    SEL_M5 = 1,
    SEL_M4_M5_OPPOSED = 2
} Enc4Sel;
static volatile Enc4Sel enc4_sel = SEL_M4;   /* set by your button */

static const char *enc4_selection_name(Enc4Sel selection)
{
    switch (selection) {
    case SEL_M4: return "M4 catheter";
    case SEL_M5: return "M5 guidewire";
    case SEL_M4_M5_OPPOSED: return "M4/M5 opposed";
    default: return "unknown";
    }
}

static uint8_t motor_driver_init(uint8_t mi)
{
    Motor *m = &mot[mi];

    motor_ready[mi] = 0u;
    HAL_GPIO_WritePin(m->en_port, m->en_pin, GPIO_PIN_SET);

    for (uint32_t attempt = 1u; attempt <= TMC_INIT_RETRIES; attempt++) {
        TMC_Status_t st = TMC2209uart(500, MICROSTEP,
                                     m->pdn_port, m->pdn_pin, m->tmc_addr);
        if (st == TMC_OK) {
            motor_ready[mi] = 1u;
            HAL_GPIO_WritePin(m->en_port, m->en_pin, GPIO_PIN_RESET);
            printf("[MAIN] M%d driver ready attempt=%lu EN=0\r\n",
                   mi + 1, (unsigned long)attempt);
            return 1u;
        }

        printf("[MAIN] M%d config failed attempt=%lu code=%d\r\n",
               mi + 1, (unsigned long)attempt, (int)st);
        HAL_Delay(TMC_INIT_RETRY_DELAY_MS);
    }

    printf("[MAIN] M%d disabled after %u attempts\r\n",
           mi + 1, (unsigned int)TMC_INIT_RETRIES);
    return 0u;
}

static void control_tick(Motor *m, float dt)
{
    int32_t error   = m->target_pos - m->current_pos;
    int32_t abs_err = (error < 0) ? -error : error;

    float v_stop = sqrtf(2.0f * A_MAX * (float)abs_err);
    float v_cmd  = (v_stop < V_MAX) ? v_stop : V_MAX;
    if (error < 0) v_cmd = -v_cmd;

    float dv_max = A_MAX * dt;
    float dv = v_cmd - m->vel;
    if (dv >  dv_max) dv =  dv_max;
    if (dv < -dv_max) dv = -dv_max;
    m->vel += dv;

    if (abs_err == 0 && fabsf(m->vel) < V_MIN) { m->vel = 0.0f; m->emit_accum = 0.0f; return; }

    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, (m->vel >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    m->emit_accum += m->vel * dt;
    int32_t n = (int32_t)m->emit_accum;
    m->emit_accum -= (float)n;

    if      (error == 0)             { n = 0;     m->emit_accum = 0.0f; }
    else if (error > 0 && n > error) { n = error; m->emit_accum = 0.0f; }
    else if (error < 0 && n < error) { n = error; m->emit_accum = 0.0f; }

    int32_t k = (n < 0) ? -n : n;
    for (int32_t i = 0; i < k; i++) step_pulse(m->step_port, m->step_pin);
    m->current_pos += n;
}

/* ============ Encoder homing for M2 / M3 (blocking) ============ */

#define HOME_NORMAL_CURRENT_MA 500u
#define HOME_M2_TOUCH_CURRENT_MA 175u
#define HOME_M3_TOUCH_CURRENT_MA 300u
#define HOME_APPROACH_V        800u /* slow touch-off */
#define HOME_RETURN_V          800u
#define HOME_WIN                16u
#define HOME_ARM_WINDOWS         3u
#define HOME_RNUM               25
#define HOME_RDEN               64
#define HOME_TRIP_LAG           80
#define HOME_CONFIRM_WINDOWS     2u
#define HOME_SECOND_ARM_COUNTS 6000
#define HOME_M2_LIMIT_STEPS   12000u
#define HOME_M3_LIMIT_STEPS   12288u
#define HOME_M2_SEAT_STEPS       32u
#define HOME_M3_SEAT_STEPS        0u
#define HOME_START_EPS_CNT         4
#define HOME_RETURN_TOL_CNT      2
#define HOME_RETURN_MARGIN      64u
#define HOME_MIN_SPAN_CNT      12000
#define HOME_MAX_SPAN_CNT      30000

/* CAD angles are stored in 0.1 degree units so homing uses no float math. */
#define HOME_M2_SPAN_DEG10           2539u
#define HOME_M2_RETURN2_DEG10        1390u
#define HOME_M3_SPAN_DEG10           2452u
#define HOME_M3_RETURN2_DEG10        1383u

typedef struct {
    uint8_t            mi;      /* index into mot[] */
    TIM_HandleTypeDef *fenc;
    uint8_t            is32;
    int8_t             esign;   /* +1 if CNT rises when motor turns in +dir */
    int8_t             first_dir;
    uint16_t            span_deg10;
    uint16_t            return2_deg10;
    uint32_t            limit_steps;
    uint16_t            touch_current_ma;
    uint16_t            seat_steps;
} HomeAxis;

/* Document M1 -> firmware M2, document M2 -> firmware M3.
 * Bench-verified follower wiring: M2=TIM8, M3=TIM5.
 * Runtime encoder direction: M2 esign=-1, M3 esign=-1.
 * Each axis touches the first stop, traverses to the opposite stop, measures
 * the actual encoder span, then returns from the second stop by the CAD ratio.
 * M3 first_dir=+ reaches the 132.7-degree side, so the return from the second
 * stop is 112.5 degrees. */
static const HomeAxis home_ax[2] = {
    { 1, &htim8, 0, -1, -1, HOME_M2_SPAN_DEG10,
      HOME_M2_RETURN2_DEG10, HOME_M2_LIMIT_STEPS,
      HOME_M2_TOUCH_CURRENT_MA, HOME_M2_SEAT_STEPS },
    { 2, &htim5, 1, -1, +1, HOME_M3_SPAN_DEG10,
      HOME_M3_RETURN2_DEG10, HOME_M3_LIMIT_STEPS,
      HOME_M3_TOUCH_CURRENT_MA, HOME_M3_SEAT_STEPS },
};

typedef enum {
    HOME_OK = 0,
    HOME_NO_HARDSTOP,
    HOME_ENCODER_FAULT,
    HOME_SPAN_FAULT,
    HOME_RETURN_FAULT,
    HOME_DRIVER_FAULT
} HomeStatus;

static uint8_t knob_axes_homed = 0;

static int32_t home_encoder_delta(const HomeAxis *a, uint32_t now, uint32_t prev)
{
    return a->is32 ? (int32_t)(now - prev)
                   : (int16_t)((uint16_t)now - (uint16_t)prev);
}

static int32_t home_position_from(const HomeAxis *a, uint32_t origin)
{
    int32_t delta = home_encoder_delta(a, __HAL_TIM_GET_COUNTER(a->fenc), origin);
    return delta * a->esign;
}

static void home_set_direction(const HomeAxis *a, int8_t logical_dir)
{
    Motor *m = &mot[a->mi];
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      (logical_dir * m->dir_sign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    dwt_delay_cycles(SystemCoreClock / 100000u); /* 10 us DIR setup */
}

typedef struct {
    uint32_t stop_pos;
    uint32_t moved_steps;
} HomeStop;

typedef struct {
    uint32_t prev;
    uint32_t moved;
    uint32_t in_window;
    uint32_t window_n;
    int64_t  advanced_counts;
    int64_t  lag0;
    int64_t  lag_delta;
    int64_t  max_lag_delta;
    uint8_t  tripped;
    uint8_t  detection_armed;
    uint8_t  confirm;
} HomeSeekState;

static HomeStatus home_set_axis_currents(uint8_t touch_mode)
{
    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        Motor *m = &mot[a->mi];
        uint16_t current = touch_mode ?
                           a->touch_current_ma : HOME_NORMAL_CURRENT_MA;
        if (TMC2209uart(current, MICROSTEP, m->pdn_port,
                        m->pdn_pin, m->tmc_addr) != TMC_OK) {
            printf("[HOME] M%d current setup failed\r\n", a->mi + 1);
            return HOME_DRIVER_FAULT;
        }
    }
    return HOME_OK;
}

static HomeStatus home_seek_pair(const int8_t seek_dir[2],
                                 const uint8_t allow_already[2],
                                 const int32_t min_arm_counts[2],
                                 HomeStop result[2])
{
    HomeSeekState s[2] = {0};
    const uint32_t step_cyc = SystemCoreClock / HOME_APPROACH_V;
    uint8_t completed = 0;

    for (int i = 0; i < 2; i++) {
        home_set_direction(&home_ax[i], seek_dir[i]);
        s[i].prev =