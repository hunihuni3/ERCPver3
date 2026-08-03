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

/* Coupled mode only: M5 travel command relative to M4, in percent.
 * 100 = same command, 95 = M5 5% less, 105 = M5 5% more.
 * M4-only and M5-only modes remain unchanged. */
#define COUPLED_M5_RATIO_PCT 100

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
        s[i].prev = __HAL_TIM_GET_COUNTER(home_ax[i].fenc);
    }

    uint32_t t = DWT->CYCCNT;
    while (completed < 2u) {
        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;

        for (int i = 0; i < 2; i++) {
            const HomeAxis *a = &home_ax[i];
            Motor *m = &mot[a->mi];
            HomeSeekState *q = &s[i];

            if (q->tripped) continue;
            if (q->moved >= a->limit_steps) {
                printf("[HOME] M%d no-stop moved=%lu lag=%ld max-lag=%ld\r\n",
                       a->mi + 1, (unsigned long)q->moved,
                       (long)q->lag_delta, (long)q->max_lag_delta);
                return HOME_NO_HARDSTOP;
            }

            step_pulse(m->step_port, m->step_pin);
            q->moved++;
            if (++q->in_window < HOME_WIN) continue;
            q->in_window = 0;

            uint32_t now = __HAL_TIM_GET_COUNTER(a->fenc);
            int32_t d = home_encoder_delta(a, now, q->prev);
            q->prev = now;
            int32_t adv = d * a->esign * seek_dir[i];
            q->advanced_counts += adv;
            q->window_n++;

            int64_t expected_counts =
                ((int64_t)q->moved * HOME_RDEN) / HOME_RNUM;
            int64_t lag = expected_counts - q->advanced_counts;

            if (q->window_n == HOME_ARM_WINDOWS) {
                printf("[HOME] M%d arm advanced=%ld\r\n",
                       a->mi + 1, (long)q->advanced_counts);
                if (allow_already[i] &&
                    q->advanced_counts >= -HOME_START_EPS_CNT &&
                    q->advanced_counts <=  HOME_START_EPS_CNT) {
                    printf("[HOME] M%d already at stop\r\n", a->mi + 1);
                    q->tripped = 1;
                    result[i].stop_pos =
                        __HAL_TIM_GET_COUNTER(a->fenc);
                    result[i].moved_steps = q->moved;
                    completed++;
                    continue;
                }
                if (q->advanced_counts < -HOME_START_EPS_CNT) {
                    printf("[HOME] M%d encoder-dir advanced=%ld\r\n",
                           a->mi + 1, (long)q->advanced_counts);
                    return HOME_ENCODER_FAULT;
                }
                if (min_arm_counts[i] == 0) {
                    q->lag0 = lag;
                    q->detection_armed = 1;
                }
                continue;
            }
            if (q->window_n < HOME_ARM_WINDOWS) continue;

            if (q->advanced_counts < 0) {
                printf("[HOME] M%d encoder-dir advanced=%ld\r\n",
                       a->mi + 1, (long)q->advanced_counts);
                return HOME_ENCODER_FAULT;
            }

            if (!q->detection_armed) {
                if (q->advanced_counts < min_arm_counts[i]) continue;
                q->lag0 = lag;
                q->lag_delta = 0;
                q->max_lag_delta = 0;
                q->detection_armed = 1;
                printf("[HOME] M%d detector armed at %ld counts\r\n",
                       a->mi + 1, (long)q->advanced_counts);
                continue;
            }

            q->lag_delta = lag - q->lag0;
            if (q->lag_delta > q->max_lag_delta)
                q->max_lag_delta = q->lag_delta;

            if (q->lag_delta >= HOME_TRIP_LAG) {
                if (++q->confirm >= HOME_CONFIRM_WINDOWS) {
                    printf("[HOME] M%d stop detected moved=%lu lag=%ld\r\n",
                           a->mi + 1, (unsigned long)q->moved,
                           (long)q->lag_delta);
                    q->tripped = 1;
                    result[i].stop_pos =
                        __HAL_TIM_GET_COUNTER(a->fenc);
                    result[i].moved_steps = q->moved;
                    completed++;
                }
            } else {
                q->confirm = 0;
            }
        }
    }

    /* Re-seat each stopped axis immediately before capturing its endpoint. */
    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        Motor *m = &mot[a->mi];
        home_set_direction(a, seek_dir[i]);
        t = DWT->CYCCNT;
        for (uint16_t n = 0; n < a->seat_steps; n++) {
            while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
            t += step_cyc;
            step_pulse(m->step_port, m->step_pin);
        }
        if (a->seat_steps > 0u)
            result[i].stop_pos = __HAL_TIM_GET_COUNTER(a->fenc);
        result[i].moved_steps += a->seat_steps;
    }
    return HOME_OK;
}

static HomeStatus home_move_pair(const uint32_t origin[2],
                                 const int8_t move_dir[2],
                                 const int32_t target_counts[2],
                                 const uint32_t step_limit[2])
{
    uint32_t moved[2] = {0};
    uint8_t done[2] = {0};
    uint8_t completed = 0;
    const uint32_t step_cyc = SystemCoreClock / HOME_RETURN_V;

    for (int i = 0; i < 2; i++)
        home_set_direction(&home_ax[i], move_dir[i]);

    uint32_t t = DWT->CYCCNT;
    while (completed < 2u) {
        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;

        for (int i = 0; i < 2; i++) {
            const HomeAxis *a = &home_ax[i];
            Motor *m = &mot[a->mi];
            if (done[i]) continue;

            int32_t position =
                home_position_from(a, origin[i]) * move_dir[i];
            if (position >= target_counts[i] - HOME_RETURN_TOL_CNT) {
                done[i] = 1;
                completed++;
                printf("[HOME] M%d return complete counts=%ld steps=%lu\r\n",
                       a->mi + 1, (long)position,
                       (unsigned long)moved[i]);
                continue;
            }
            if (moved[i] >= step_limit[i]) {
                printf("[HOME] M%d return failed counts=%ld steps=%lu\r\n",
                       a->mi + 1, (long)position,
                       (unsigned long)moved[i]);
                return HOME_RETURN_FAULT;
            }

            step_pulse(m->step_port, m->step_pin);
            moved[i]++;
        }
    }
    return HOME_OK;
}

static HomeStatus home_m2_m3(void)
{
    HomeStop first[2] = {0};
    HomeStop second[2] = {0};
    int8_t first_dir[2];
    int8_t second_dir[2];
    uint8_t allow_first[2] = {1u, 1u};
    uint8_t allow_second[2] = {0u, 0u};
    int32_t first_arm[2] = {0, 0};
    int32_t second_arm[2] = {
        HOME_SECOND_ARM_COUNTS, HOME_SECOND_ARM_COUNTS
    };
    int32_t home_counts[2];
    uint32_t return_limit[2];

    knob_axes_homed = 0;
    for (int i = 0; i < 2; i++) {
        first_dir[i] = home_ax[i].first_dir;
        second_dir[i] = (int8_t)-home_ax[i].first_dir;
    }

    HomeStatus status = home_set_axis_currents(1u);
    if (status != HOME_OK) return status;

    printf("[HOME] simultaneous first-stop search\r\n");
    status = home_seek_pair(first_dir, allow_first, first_arm, first);
    if (status != HOME_OK) {
        (void)home_set_axis_currents(0u);
        return status;
    }
    for (int i = 0; i < 2; i++)
        printf("[HOME] M%d stop-A pos=%lu steps=%lu\r\n",
               home_ax[i].mi + 1, (unsigned long)first[i].stop_pos,
               (unsigned long)first[i].moved_steps);

    printf("[HOME] simultaneous second-stop search\r\n");
    status = home_seek_pair(second_dir, allow_second, second_arm, second);
    if (status != HOME_OK) {
        (void)home_set_axis_currents(0u);
        return status;
    }

    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        printf("[HOME] M%d stop-B pos=%lu steps=%lu\r\n",
               a->mi + 1, (unsigned long)second[i].stop_pos,
               (unsigned long)second[i].moved_steps);

        int32_t span_counts =
            home_encoder_delta(a, second[i].stop_pos, first[i].stop_pos) *
            a->esign * second_dir[i];
        if (span_counts < HOME_MIN_SPAN_CNT ||
            span_counts > HOME_MAX_SPAN_CNT) {
            (void)home_set_axis_currents(0u);
            printf("[HOME] M%d span invalid counts=%ld\r\n",
                   a->mi + 1, (long)span_counts);
            return HOME_SPAN_FAULT;
        }

        home_counts[i] =
            (int32_t)(((int64_t)span_counts * a->return2_deg10 +
                       a->span_deg10 / 2u) / a->span_deg10);
        return_limit[i] =
            (uint32_t)(((uint64_t)second[i].moved_steps *
                        a->return2_deg10 + a->span_deg10 - 1u) /
                       a->span_deg10) + HOME_RETURN_MARGIN;
        uint32_t ratio_x1000 =
            (uint32_t)(((int64_t)span_counts * 3600 * 1000 +
                        (int64_t)ENC_CPR * a->span_deg10 / 2) /
                       ((int64_t)ENC_CPR * a->span_deg10));

        printf("[HOME] M%d span=%ldcnt/%lustep ratio=%lu.%03lu:1 "
               "home=%ldcnt\r\n",
               a->mi + 1, (long)span_counts,
               (unsigned long)second[i].moved_steps,
               (unsigned long)(ratio_x1000 / 1000u),
               (unsigned long)(ratio_x1000 % 1000u),
               (long)home_counts[i]);
    }

    status = home_set_axis_currents(0u);
    if (status != HOME_OK) return status;

    printf("[HOME] simultaneous alignment return\r\n");
    status = home_move_pair(
        (const uint32_t[2]){second[0].stop_pos, second[1].stop_pos},
        first_dir, home_counts, return_limit);
    if (status != HOME_OK) return status;

    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        Motor *m = &mot[a->mi];
        m->current_pos = 0;
        m->target_pos  = 0;
        m->accum_num   = 0;
        m->vel         = 0.0f;
        m->emit_accum  = 0.0f;
        __HAL_TIM_SET_COUNTER(a->fenc, 0);
        printf("[HOME] M%d aligned\r\n", a->mi + 1);
    }

    /* Input encoders counted while homing was blocking. Re-seed them before
       normal control so no accumulated knob motion is replayed. */
    for (int i = 0; i < 4; i++)
        in_enc[i].prev = __HAL_TIM_GET_COUNTER(in_enc[i].tim);

    knob_axes_homed = 1;
    return HOME_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* first thing: all drivers OFF (EN active-low) */
    for (int m = 0; m < N_MOT; m++)
        HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_SET);


  dwt_delay_init();   /* MUST run before any m1_step_pulse() — enables DWT->CYCCNT */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    for (int i = 0; i < 4; i++) { __HAL_TIM_SET_COUNTER(in_enc[i].tim, 0); in_enc[i].prev = 0; }

    HAL_Delay(100); //wait for VMOT to rise, without this TMC drivers fail to init.--> fixed bug in code that caused drivers to drive full 1.7A at boot.



    /* SENDDELAY=2 (3*8 bit times) on every node BEFORE any read access.
     * Datasheet 4.1.2: default (8 bit times) must not be used multi-node. */
    for (int m = 0; m < N_MOT; m++) {
        if (!motor_enabled_for_mode((uint8_t)m)) continue;
        tmc_write_nodeconf(mot[m].pdn_port, mot[m].pdn_pin, mot[m].tmc_addr, 2);
    }

    HAL_Delay(10);

    printf("[FW] HOME_REV=20260731_16 M45_COUPLED_TEST\r\n");

    /* 3. Configure; enable ONLY drivers whose config verified */
    for (int m = 0; m < N_MOT; m++) {
        if (!motor_enabled_for_mode((uint8_t)m)) {
            motor_ready[m] = 0u;
            HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_SET);
            printf("[MAIN] M%d disabled by M45 test mode EN=1\r\n", m + 1);
            continue;
        }
        (void)motor_driver_init((uint8_t)m);
        HAL_Delay(TMC_INIT_AXIS_DELAY_MS);
    }

#if M45_ONLY_TEST_MODE
    /* M2/M3 and their follower encoders are not installed for this bench test.
       Keep both drivers disabled and enter normal control for M1/M4/M5. */
    HAL_GPIO_WritePin(mot[1].en_port, mot[1].en_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(mot[2].en_port, mot[2].en_pin, GPIO_PIN_SET);
    for (int i = 0; i < 4; i++)
        in_enc[i].prev = __HAL_TIM_GET_COUNTER(in_enc[i].tim);
    knob_axes_homed = 1u;
    printf("[HOME] M2/M3 disabled - homing skipped for M4/M5 test\r\n");
#else
    if (home_m2_m3() != HOME_OK) {
        for (int m = 0; m < N_MOT; m++)
            HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_SET);
        Error_Handler();
    }
#endif

    /* Homing only operates M2/M3. Restore the verified M4/M5 enable state
       explicitly before accepting ENC4 commands. */
    for (int m = 3; m <= 4; m++) {
        HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin,
                          motor_ready[m] ? GPIO_PIN_RESET : GPIO_PIN_SET);
        printf("[ENC4] M%d ready=%u EN=%u\r\n",
               m + 1, (unsigned int)motor_ready[m],
               (unsigned int)HAL_GPIO_ReadPin(mot[m].en_port, mot[m].en_pin));
    }

    uint32_t last_cyc = DWT->CYCCNT;
    Enc4Sel reported_enc4_sel = enc4_sel;
    uint32_t enc4_diag_tick = 0u;
    uint8_t enc4_diag_remaining = 8u;

   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (!knob_axes_homed) {
          for (int m = 0; m < N_MOT; m++)
              HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_SET);
          Error_Handler();
      }

	  /* --- dt since last tick, exact from DWT (wrap-safe unsigned sub) --- */
	  uint32_t now_cyc = DWT->CYCCNT;
	      float dt = (float)(now_cyc - last_cyc) / (float)SystemCoreClock;
	      last_cyc = now_cyc;

	      int32_t gain_pct = 100;


	      //set gain according to current gain setting
	      if (current_mode == MODE_PRECISION){
	    	  gain_pct = 100;
	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	      }
	      else if (current_mode == MODE_FAST){
	    	  gain_pct = 300;
	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	      }

	      /* Tool LED:
	       * catheter/M4 = solid on, guidewire/M5 = off,
	       * opposed M4/M5 = blink at 2 Hz. */
	      if (current_tool == TOOL_CATHETER){
	    	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	      }
	      else if (current_tool == TOOL_GUIDE){
	    	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	      }
	      else {
	    	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
	    	                   ((HAL_GetTick() / 250u) & 1u) ?
	    	                   GPIO_PIN_SET : GPIO_PIN_RESET);
	      }




	      int32_t d[4];
	      for (int i = 0; i < 4; i++) {
	          d[i] = in_enc_delta(&in_enc[i]);
	      }

	      int32_t routed[N_MOT] = {0};
	      routed[0] = d[0];                               /* M1<-enc1 */
#if !M45_ONLY_TEST_MODE
	      routed[1] = d[1];                               /* M2<-enc2 */
	      routed[2] = d[2];                               /* M3<-enc3 */
#endif
	      if (enc4_sel == SEL_M4) {
	    	  routed[3] = d[3];                            /* M4<-enc4, M5 holds */
	      } else if (enc4_sel == SEL_M5) {
	    	  routed[4] = d[3];                            /* M5<-enc4, M4 holds */
	      } else {
	    	  routed[3] = d[3];                            /* M4 follows ENC4 */
	    	  routed[4] = d[3] * COUPLED_M5_ENCODER_SIGN; /* M5 moves opposite */
	      }

	      for (int m = 0; m < N_MOT; m++) {
	    	  if (!motor_ready[m]) continue;
	    	  int32_t g = mot[m].gain_enabled ? gain_pct : 100;
	    	  int32_t coupled_ratio_pct =
	    	      (enc4_sel == SEL_M4_M5_OPPOSED && m == 4) ?
	    	      COUPLED_M5_RATIO_PCT : 100;
	    	  int64_t one_step =
	    	      (int64_t)100 * 100 * RATIO_DEN * mot[m].gear_den;
	      	  mot[m].accum_num +=
	      	      (int64_t)routed[m] * g * coupled_ratio_pct *
	      	      RATIO_NUM * mot[m].gear_num * mot[m].dir_sign;
	      	  while (mot[m].accum_num >=  one_step) { mot[m].accum_num -= one_step; mot[m].target_pos++; }
	      	  while (mot[m].accum_num <= -one_step) { mot[m].accum_num += one_step; mot[m].target_pos--; }
	      	  control_tick(&mot[m], dt);
	      }

	      if (reported_enc4_sel != enc4_sel) {
	    	  reported_enc4_sel = enc4_sel;
	    	  printf("[ENC4] selected %s\r\n",
	    	         enc4_selection_name(reported_enc4_sel));
	      }

	      /* Commissioning trace: at most eight rate-limited lines. This proves
	         TIM4 counting, routing, target generation and driver enable state. */
	      if (d[3] != 0 && enc4_diag_remaining > 0u &&
	          (uint32_t)(HAL_GetTick() - enc4_diag_tick) >= 250u) {
	    	  enc4_diag_tick = HAL_GetTick();
	    	  enc4_diag_remaining--;
	    	  printf("[ENC4] mode=%s cnt=%lu d=%ld "
	    	         "M4 target=%ld pos=%ld ready=%u EN=%u "
	    	         "M5 target=%ld pos=%ld ready=%u EN=%u\r\n",
	    	         enc4_selection_name(enc4_sel),
	    	         (unsigned long)__HAL_TIM_GET_COUNTER(&htim4),
	    	         (long)d[3],
	    	         (long)mot[3].target_pos, (long)mot[3].current_pos,
	    	         (unsigned int)motor_ready[3],
	    	         (unsigned int)HAL_GPIO_ReadPin(mot[3].en_port,
	    	                                        mot[3].en_pin),
	    	         (long)mot[4].target_pos, (long)mot[4].current_pos,
	    	         (unsigned int)motor_ready[4],
	    	         (unsigned int)HAL_GPIO_ReadPin(mot[4].en_port,
	    	                                        mot[4].en_pin));
	      }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC0 PC1
                           PC2 PC3 PC4 PC5
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5) { 								//gain button ISR
        static uint32_t last = 0;
        uint32_t now = DWT->CYCCNT;

        /* 200 ms window @ 84 MHz = 84000 cycles/ms * 200 */
        if ((uint32_t)(now - last) >= (84000UL * 200UL)) {
            last = now;
  	      if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5) == 1){
  	    	  if (current_mode == MODE_PRECISION){
  	    		  current_mode = MODE_FAST;
  	    	  }
  	    	  else if (current_mode == MODE_FAST){
  	    	  	  current_mode = MODE_PRECISION;
  	    	  }
  	      }


        }
    }


    if (GPIO_Pin == GPIO_PIN_4) {								//mode button ISR
            static uint32_t last = 0;
            uint32_t now = DWT->CYCCNT;

            /* 200 ms window @ 84 MHz = 84000 cycles/ms * 200 */
            if ((uint32_t)(now - last) >= (84000UL * 200UL)) {
                last = now;
      	      if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == 1){
      	    	  if (current_tool == TOOL_CATHETER){
      	    		  current_tool = TOOL_GUIDE;
      	    		  enc4_sel = SEL_M5;
      	    	  }
      	    	  else if (current_tool == TOOL_GUIDE){
      	    	  	  current_tool = TOOL_CATHETER_RETRACT_WIRE_HOLD;
      	    	  	  enc4_sel = SEL_M4_M5_OPPOSED;
      	    	  }
      	    	  else {
      	    	  	  current_tool = TOOL_CATHETER;
      	    	  	  enc4_sel = SEL_M4;
      	    	  }
      	      }


            }
        }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
