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

    TOOL_GUIDE = 1

} Tool;

volatile Mode current_mode = MODE_PRECISION; 					// MODE_PRECISION is just 1:1 matching mode. encoders turned out to be accurate enough to just 1:1 match.

volatile Tool current_tool = TOOL_CATHETER;

/* DWT cycle counter init ??call once before the loop. Used for the ~1us pulse. */
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

/* STEP/DIR/EN below are read from your schematic and are confident.
 * pdn_pin for M3/M4/M5 and all tmc_addr are NOT ??see confirm-list. */

// gain_en determines if motor is subject to gain variation by gain button; only enable for feeder, not position tracking knobs.
// gear D is input gear, gearN is output gear.
static Motor mot[N_MOT] = {
  /* step               dir                en                pdn(TMC)          addr gain gearN gearD sign runtime state */
  { GPIOC,GPIO_PIN_1,  GPIOC,GPIO_PIN_2,  GPIOC,GPIO_PIN_0,  GPIOB,GPIO_PIN_0,  0, 0,  95,  26, -1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOC,GPIO_PIN_4,  GPIOC,GPIO_PIN_5,  GPIOC,GPIO_PIN_3,  GPIOB,GPIO_PIN_0,  1, 0, 171, 181, +1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOC,GPIO_PIN_9,  GPIOC,GPIO_PIN_10, GPIOC,GPIO_PIN_8,  GPIOB,GPIO_PIN_0,  2, 0,  18,  70, +1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOC,GPIO_PIN_12, GPIOC,GPIO_PIN_13, GPIOC,GPIO_PIN_11, GPIOB,GPIO_PIN_0,  3, 1,   1,   1, +1, 0, 0, 0, 0.0f, 0.0f },
  { GPIOA,GPIO_PIN_11, GPIOD,GPIO_PIN_2,  GPIOA,GPIO_PIN_12, GPIOB,GPIO_PIN_1,  0, 1,   1,   1, +1, 0, 0, 0, 0.0f, 0.0f },
};

/* enc4 routes to exactly one of M4/M5 at a time */
typedef enum { SEL_M4 = 0, SEL_M5 = 1 } Enc4Sel;
static volatile Enc4Sel enc4_sel = SEL_M4;   /* set by your button */

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
#define HOME_TRIP_LAG           10
#define HOME_M2_LIMIT_STEPS    8192u
#define HOME_M3_LIMIT_STEPS   12288u
#define HOME_M2_SEAT_STEPS       32u
#define HOME_M3_SEAT_STEPS        0u
#define HOME_START_EPS_CNT         4
#define HOME_RETURN_TOL_CNT      2
#define HOME_RETURN_MARGIN      64u

/* CAD angles are stored in 0.1 degree units so homing uses no float math. */
#define HOME_M2_OFFSET_DEG10   1270u
#define HOME_M2_TIMEOUT_DEG10  2740u
#define HOME_M3_OFFSET_DEG10   1327u
#define HOME_M3_TIMEOUT_DEG10  2650u

typedef struct {
    uint8_t            mi;      /* index into mot[] */
    TIM_HandleTypeDef *fenc;
    uint8_t            is32;
    int8_t             esign;   /* +1 if CNT rises when motor turns in +dir */
    int8_t             dir;     /* direction toward the hard stop */
    uint16_t            offset_deg10;
    uint16_t            timeout_deg10;
    uint32_t            limit_steps;
    uint16_t            touch_current_ma;
    uint16_t            seat_steps;
} HomeAxis;

/* Document M1 -> firmware M2, document M2 -> firmware M3.
 * Bench-verified follower wiring: M2=TIM8, M3=TIM5.
 * Runtime encoder direction: M2 esign=-1, M3 esign=-1.
 * Until a separate motor-to-drum ratio is measured, the inverse of the
 * existing output/input ratios is used for motor turns per drum turn:
 * M2=181/171, M3=70/18.
 * M3's + HOME_DIR is provisionally paired with the 132.7 degree stop.
 * If bench jogging shows that + reaches the opposite CAD stop, change only
 * HOME_M3_OFFSET_DEG10 to 1125 (and keep the direction/offset paired). */
static const HomeAxis home_ax[2] = {
    { 1, &htim8, 0, -1, -1, HOME_M2_OFFSET_DEG10,
      HOME_M2_TIMEOUT_DEG10, HOME_M2_LIMIT_STEPS,
      HOME_M2_TOUCH_CURRENT_MA, HOME_M2_SEAT_STEPS },
    { 2, &htim5, 1, -1, +1, HOME_M3_OFFSET_DEG10,
      HOME_M3_TIMEOUT_DEG10, HOME_M3_LIMIT_STEPS,
      HOME_M3_TOUCH_CURRENT_MA, HOME_M3_SEAT_STEPS },
};

typedef enum {
    HOME_OK = 0,
    HOME_NO_HARDSTOP,
    HOME_ENCODER_FAULT,
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

static uint32_t home_deg10_to_steps(const HomeAxis *a, uint16_t deg10)
{
    const Motor *m = &mot[a->mi];
    return (uint32_t)(((int64_t)deg10 * RATIO_NUM * m->gear_den +
                       (int64_t)3600 * m->gear_num - 1) /
                      ((int64_t)3600 * m->gear_num));
}

static int32_t home_deg10_to_counts(const HomeAxis *a, uint16_t deg10)
{
    const Motor *m = &mot[a->mi];
    return (int32_t)(((int64_t)deg10 * ENC_CPR * m->gear_den +
                      (int64_t)3600 * m->gear_num / 2) /
                     ((int64_t)3600 * m->gear_num));
}

static HomeStatus home_move_from_stop_to_alignment(const HomeAxis *a,
                                                   uint32_t stop_pos)
{
    Motor *m = &mot[a->mi];
    const int32_t target = home_deg10_to_counts(a, a->offset_deg10);
    const uint32_t return_limit =
        home_deg10_to_steps(a, a->offset_deg10) + HOME_RETURN_MARGIN;
    const uint32_t step_cyc = SystemCoreClock / HOME_RETURN_V;

    home_set_direction(a, (int8_t)-a->dir);
    uint32_t t = DWT->CYCCNT;

    for (uint32_t moved = 0; moved < return_limit; moved++) {
        int32_t away = -home_position_from(a, stop_pos) * a->dir;
        if (away >= target - HOME_RETURN_TOL_CNT) {
            return HOME_OK;
        }

        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;
        step_pulse(m->step_port, m->step_pin);
    }

    int32_t away = -home_position_from(a, stop_pos) * a->dir;
    return (away >= target - HOME_RETURN_TOL_CNT) ?
           HOME_OK : HOME_RETURN_FAULT;
}

static HomeStatus home_one(const HomeAxis *a)
{
    Motor *m = &mot[a->mi];

    const uint32_t approach_limit = a->limit_steps;
    const uint32_t step_cyc = SystemCoreClock / HOME_APPROACH_V;

    if (TMC2209uart(a->touch_current_ma, MICROSTEP, m->pdn_port,
                    m->pdn_pin, m->tmc_addr) != TMC_OK) {
        return HOME_DRIVER_FAULT;
    }

    home_set_direction(a, a->dir);

    uint32_t t      = DWT->CYCCNT;
    uint32_t prev   = __HAL_TIM_GET_COUNTER(a->fenc);
    uint32_t moved  = 0, in_window = 0, window_n = 0;
    int64_t  advanced_counts = 0;
    int64_t  lag0 = 0;
    int64_t  lag_delta = 0;
    int64_t  max_lag_delta = 0;
    uint8_t  tripped = 0;

    while (moved < approach_limit) {
        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;
        step_pulse(m->step_port, m->step_pin);
        moved++;

        if (++in_window < HOME_WIN) continue;
        in_window = 0;

        uint32_t now = __HAL_TIM_GET_COUNTER(a->fenc);
        int32_t  d   = home_encoder_delta(a, now, prev);
        prev = now;
        int32_t adv = d * a->esign * a->dir;   /* >0 when advancing to the stop */
        advanced_counts += adv;

        window_n++;
        int64_t expected_counts =
            ((int64_t)moved * HOME_RDEN) / HOME_RNUM;
        int64_t lag = expected_counts - advanced_counts;

        if (window_n == HOME_ARM_WINDOWS) {
            printf("[HOME] M%d arm advanced=%ld\r\n",
                   a->mi + 1, (long)advanced_counts);
            if (advanced_counts >= -HOME_START_EPS_CNT &&
                advanced_counts <=  HOME_START_EPS_CNT) {
                printf("[HOME] M%d already at stop\r\n", a->mi + 1);
                tripped = 1;
                break;
            }
            if (advanced_counts < -HOME_START_EPS_CNT) {
                (void)TMC2209uart(HOME_NORMAL_CURRENT_MA, MICROSTEP,
                                  m->pdn_port, m->pdn_pin, m->tmc_addr);
                printf("[HOME] M%d encoder-dir advanced=%ld\r\n",
                       a->mi + 1, (long)advanced_counts);
                return HOME_ENCODER_FAULT;
            }
            lag0 = lag;
            continue;
        }
        if (window_n < HOME_ARM_WINDOWS) continue;

        lag_delta = lag - lag0;
        if (lag_delta > max_lag_delta) max_lag_delta = lag_delta;
        if (advanced_counts < 0) {
            (void)TMC2209uart(HOME_NORMAL_CURRENT_MA, MICROSTEP, m->pdn_port,
                              m->pdn_pin, m->tmc_addr);
            printf("[HOME] M%d encoder-dir advanced=%ld\r\n",
                   a->mi + 1, (long)advanced_counts);
            return HOME_ENCODER_FAULT;
        }
        if (lag_delta >= HOME_TRIP_LAG) {
            printf("[HOME] M%d stop detected moved=%lu lag=%ld\r\n",
                   a->mi + 1, (unsigned long)moved, (long)lag_delta);
            tripped = 1;
            break;
        }
    }

    if (!tripped) {
        (void)TMC2209uart(HOME_NORMAL_CURRENT_MA, MICROSTEP, m->pdn_port,
                          m->pdn_pin, m->tmc_addr);
        printf("[HOME] M%d no-stop moved=%lu lag=%ld max-lag=%ld\r\n",
               a->mi + 1, (unsigned long)moved,
               (long)lag_delta, (long)max_lag_delta);
        return HOME_NO_HARDSTOP;
    }

    /* Seat M2 gently against the hard stop for a short, bounded interval
       before capturing STOP_POS. M3 currently uses zero seating steps. */
    t = DWT->CYCCNT;
    for (uint16_t i = 0; i < a->seat_steps; i++) {
        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;
        step_pulse(m->step_port, m->step_pin);
    }

    uint32_t stop_pos = __HAL_TIM_GET_COUNTER(a->fenc);

    /* Touch-off uses reduced current, but pull away from the mechanical stop
       at normal current so gearbox friction cannot leave the axis stuck. */
    if (TMC2209uart(HOME_NORMAL_CURRENT_MA, MICROSTEP, m->pdn_port,
                    m->pdn_pin, m->tmc_addr) != TMC_OK) {
        return HOME_DRIVER_FAULT;
    }

    HomeStatus status = home_move_from_stop_to_alignment(a, stop_pos);
    if (status != HOME_OK) return status;

    /* The CAD alignment pose, not the power-on pose, is the logical zero. */
    m->current_pos = 0;
    m->target_pos  = 0;
    m->accum_num   = 0;
    m->vel         = 0.0f;
    m->emit_accum  = 0.0f;
    __HAL_TIM_SET_COUNTER(a->fenc, 0);
    return HOME_OK;
}

static HomeStatus home_m2_m3(void)
{
    knob_axes_homed = 0;
    for (int i = 0; i < 2; i++) {
        HomeStatus status = home_one(&home_ax[i]);
        if (status != HOME_OK) {
            printf("[HOME] M%d failed, status=%d\r\n",
                   home_ax[i].mi + 1, (int)status);
            return status;
        }
        printf("[HOME] M%d aligned\r\n", home_ax[i].mi + 1);
    }

    /* Required. The input encoders kept counting in hardware while this
       blocked. Without the resync the next control_tick sees the whole
       accumulated knob delta at once and commands a burst move. */
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


  dwt_delay_init();   /* MUST run before any m1_step_pulse() ??enables DWT->CYCCNT */
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
        tmc_write_nodeconf(mot[m].pdn_port, mot[m].pdn_pin, mot[m].tmc_addr, 2);
    }

    HAL_Delay(10);

    printf("[FW] HOME_REV=20260730_04\r\n");

    /* 3. Configure; enable ONLY drivers whose config verified */
    for (int m = 0; m < N_MOT; m++) {
        TMC_Status_t st = TMC2209uart(500, MICROSTEP,
                                      mot[m].pdn_port, mot[m].pdn_pin, mot[m].tmc_addr);
        if (st == TMC_OK)
            HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_RESET);

        HAL_Delay(10);
    }

    if (home_m2_m3() != HOME_OK) {
        for (int m = 0; m < N_MOT; m++)
            HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_SET);
        Error_Handler();
    }

    uint32_t last_cyc = DWT->CYCCNT;

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

	      //set mode(tool) LED according to current
	      if (current_tool == TOOL_CATHETER){
	    	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	      }
	      else if (current_tool == TOOL_GUIDE){
	    	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	      }




	      int32_t d[4];
	      for (int i = 0; i < 4; i++) {
	          d[i] = in_enc_delta(&in_enc[i]);
	      }

	      int32_t routed[N_MOT] = {0};
	      routed[0] = d[0];                               /* M1<-enc1 */
	      routed[1] = d[1];                               /* M2<-enc2 */
	      routed[2] = d[2];                               /* M3<-enc3 */
	      if (enc4_sel == SEL_M4) routed[3] = d[3];       /* M4<-enc4, M5 holds */
	      else                    routed[4] = d[3];       /* M5<-enc4, M4 holds */

	      for (int m = 0; m < N_MOT; m++) {
	    	  int32_t g = mot[m].gain_enabled ? gain_pct : 100;
	    	  int64_t one_step = (int64_t)100 * RATIO_DEN * mot[m].gear_den;
	      	  mot[m].accum_num += (int64_t)routed[m] * g * RATIO_NUM * mot[m].gear_num * mot[m].dir_sign;
	      	  while (mot[m].accum_num >=  one_step) { mot[m].accum_num -= one_step; mot[m].target_pos++; }
	      	  while (mot[m].accum_num <= -one_step) { mot[m].accum_num += one_step; mot[m].target_pos--; }
	      	  control_tick(&mot[m], dt);
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

