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
int acc_steps = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MICROSTEP 16
#define CPR 2048

/* --- Motor 1 pin macros (open-drain not needed here; STEP/DIR are push-pull) */
#define M1_STEP_PORT   GPIOC
#define M1_STEP_PIN    GPIO_PIN_1
#define M1_DIR_PORT    GPIOC
#define M1_DIR_PIN     GPIO_PIN_2
#define M1_EN_PORT     GPIOC
#define M1_EN_PIN      GPIO_PIN_0

/* TMC2209 enable is ACTIVE LOW: drive EN low to energise the driver. */
#define M1_DRIVER_ENABLE()   HAL_GPIO_WritePin(M1_EN_PORT, M1_EN_PIN, GPIO_PIN_RESET)
#define M1_DRIVER_DISABLE()  HAL_GPIO_WritePin(M1_EN_PORT, M1_EN_PIN, GPIO_PIN_SET)

/* Exact counts->microsteps ratio.
 *   3200 microsteps/rev  /  8192 enc counts/rev  =  25/64  (exact)
 * If your AMT102 DIPs are NOT at 2048 PPR (8192 counts), change ENC_CPR. */
#define MICROSTEPS_PER_REV   (200L * MICROSTEP)
#define ENC_CPR              (CPR * 4)
#define RATIO_NUM            MICROSTEPS_PER_REV   /* 3200 */
#define RATIO_DEN            ENC_CPR              /* 8192 -> reduces to 25/64 */

/* STEP pulse high time, in CPU cycles. At 168 MHz, 168 cycles ~= 1 us. */
#define STEP_PULSE_CYCLES    (SystemCoreClock / 1000000u)

/* How often to emit a debug line, in milliseconds (does NOT throttle stepping) */
#define DEBUG_PRINT_PERIOD_MS  50u

/* file-scope state for the follow loop */
//static int32_t  enc_prev      = 0;     /* last TIM2 count we saw            */			//all obsolete now they live in the multi stepper struct now..
//static int64_t  step_accum_num = 0;    /* fractional-step accumulator       */
                                       /* (numerator units; /RATIO_DEN = us) */
extern TIM_HandleTypeDef htim2;        /* CubeMX-generated handle           */


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
char tx_buf[64];
#include <stdio.h>
#include <stdlib.h>

int _write(int file, char *ptr, int len)
{
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
//Mode current_mode = MODE_FAST;

volatile Tool current_tool = TOOL_CATHETER;
//
uint8_t rx;  // UART 입력용

//int32_t target_pos = 0;   			// obsolete



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

//Trapezoidal position follower position error law implementation
//static int32_t current_pos = 0; //microsteps actually emitted
//static float vel = 0.0f; //signed steps/s persists per tick
//tatic float emit_accum = 0.0f; //fractional step carry for emissio
//above are all obsolete and lives in struct


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
 * pdn_pin for M3/M4/M5 and all tmc_addr are NOT — see confirm-list. */

// gain_en determines if motor is subject to gain variation by gain button; only enable for feeder, not position tracking knobs.
// gear D is input gear, gearN is output gear.
static Motor mot[N_MOT] = {
  /* step               dir                en                pdn(TMC)          addr gain_en gearN gearD* dir_sign */
  { GPIOC,GPIO_PIN_1,  GPIOC,GPIO_PIN_2,  GPIOC,GPIO_PIN_0,  GPIOB,GPIO_PIN_0,  0 , 0 , 95 , 26 , -1}, /* M1 PDN_UART1=PB0 */
  { GPIOC,GPIO_PIN_4,  GPIOC,GPIO_PIN_5,  GPIOC,GPIO_PIN_3,  GPIOB,GPIO_PIN_0,  1 , 0,  171 , 181 , +1}, /* M2 PDN_UART2=PB0 */
  { GPIOC,GPIO_PIN_9,  GPIOC,GPIO_PIN_10, GPIOC,GPIO_PIN_8,  GPIOB,GPIO_PIN_0,  2 , 0 , 18 , 70 , +1}, /* M3 PDN_UART3=PB0 */
  { GPIOC,GPIO_PIN_12, GPIOC,GPIO_PIN_13, GPIOC,GPIO_PIN_11, GPIOB,GPIO_PIN_0,  3 , 1 , 1 , 1 , +1}, /* M4 PDN_UART4=PB0 */
  { GPIOA,GPIO_PIN_11, GPIOD,GPIO_PIN_2,  GPIOA,GPIO_PIN_12, GPIOB,GPIO_PIN_1,  0 , 1 , 1 , 1 , +1}, /* M5 PDN_UART5=PB1  */
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

/*static int32_t enc_delta(Axis *ax)
{
    uint32_t now = __HAL_TIM_GET_COUNTER(ax->enc);
    int32_t d = ax->is32 ? (int32_t)(now - ax->enc_prev)
                         : (int16_t)((uint16_t)now - (uint16_t)ax->enc_prev);
    ax->enc_prev = now;
    return d;
}*/

// core clock = 84 MHz → 84 cycles per µs
static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * 84u;
    while ((DWT->CYCCNT - start) < ticks);   // unsigned subtraction is wrap-safe
}


/* ============ Encoder homing for M2 / M3 (blocking) ============ */

#define HOME_V         800u   /* microsteps/s during homing */
#define HOME_WIN       16u    /* microsteps per stall-check window */
#define HOME_TRIP_PCT  30     /* trip below this % of expected counts */
#define HOME_ARM       3u     /* windows skipped at move start */
#define HOME_CONFIRM   2u     /* consecutive bad windows to trip */
#define HOME_RNUM      25     /* microsteps = counts * 25 / 64 */
#define HOME_RDEN      64

typedef struct {
    uint8_t            mi;      /* index into mot[] */
    TIM_HandleTypeDef *fenc;
    uint8_t            is32;
    int8_t             esign;   /* +1 if CNT rises when motor turns in +dir */
    int8_t             dir;     /* direction toward the hard stop */
    uint32_t           limit;   /* microsteps; abort if no stall by then */
} HomeAxis;

/*     mi  fenc     is32  esign  dir   limit   */
static const HomeAxis home_ax[2] = {
    { 1, &htim5, 1, -1, -1, 64000 },   /* M2 - set limit to real travel */
    { 2, &htim8, 0, +1, +1, 64000 },   /* M3 - set limit to real travel */
};

/* 0 = homed, 1 = no stall within limit, 2 = follower encoder not counting */

static void home_probe(const HomeAxis *a, uint32_t n_us)
{
    Motor *m = &mot[a->mi];
    const uint32_t step_cyc = SystemCoreClock / HOME_V;
    const int32_t  expect = (int32_t)(((int64_t)HOME_WIN * HOME_RDEN) / HOME_RNUM);

    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      (a->dir * m->dir_sign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    dwt_delay_cycles(SystemCoreClock / 100000u);

    uint32_t t = DWT->CYCCNT;
    uint32_t prev = __HAL_TIM_GET_COUNTER(a->fenc);
    uint32_t moved = 0, in_win = 0, win_n = 0;

    printf("[PROBE M%d] dir=%d esign=%d expect=%ld trip=%ld cnt0=%lu\r\n",
           a->mi + 1, (int)a->dir, (int)a->esign,
           (long)expect, (long)(expect * HOME_TRIP_PCT / 100),
           (unsigned long)prev);

    while (moved < n_us) {
        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;
        step_pulse(m->step_port, m->step_pin);
        moved++;
        if (++in_win < HOME_WIN) continue;
        in_win = 0;

        uint32_t now = __HAL_TIM_GET_COUNTER(a->fenc);
        int32_t d = a->is32 ? (int32_t)(now - prev)
                            : (int16_t)((uint16_t)now - (uint16_t)prev);
        prev = now;
        printf("w%02lu d=%+ld adv=%+ld\r\n", (unsigned long)++win_n,
               (long)d, (long)(d * a->esign * a->dir));
        t = DWT->CYCCNT;   /* re-anchor; printf consumed time */
    }
}

static void home_probe2(uint8_t mi, GPIO_PinState dirlevel, uint32_t n_us)
{
    Motor *m = &mot[mi];
    const uint32_t step_cyc = SystemCoreClock / HOME_V;

    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, dirlevel);
    dwt_delay_cycles(SystemCoreClock / 100000u);

    uint32_t p5 = __HAL_TIM_GET_COUNTER(&htim5);
    uint32_t p8 = __HAL_TIM_GET_COUNTER(&htim8);
    uint32_t t  = DWT->CYCCNT;

    for (uint32_t i = 0; i < n_us; i++) {
        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;
        step_pulse(m->step_port, m->step_pin);
    }

    int32_t d5 = (int32_t)(__HAL_TIM_GET_COUNTER(&htim5) - p5);
    int32_t d8 = (int16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim8) - (uint16_t)p8);
    printf("[P2 M%d dir=%d] steps=%lu  TIM5=%+ld  TIM8=%+ld  expect~4096 on own enc\r\n",
           mi + 1, (int)dirlevel, (unsigned long)n_us, (long)d5, (long)d8);
}

static int home_one(const HomeAxis *a)
{
    Motor *m = &mot[a->mi];

    const int32_t  expect   = (int32_t)(((int64_t)HOME_WIN * HOME_RDEN) / HOME_RNUM);
    const int32_t  trip     = (expect * HOME_TRIP_PCT) / 100;
    const uint32_t step_cyc = SystemCoreClock / HOME_V;

    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      (a->dir * m->dir_sign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    dwt_delay_cycles(SystemCoreClock / 100000u);      /* 10 us DIR setup */

    uint32_t t      = DWT->CYCCNT;
    uint32_t prev   = __HAL_TIM_GET_COUNTER(a->fenc);
    uint32_t moved  = 0, in_win = 0, win_n = 0;
    uint8_t  bad    = 0;
    int      tripped = 0;

    while (moved < a->limit) {
        while ((uint32_t)(DWT->CYCCNT - t) < step_cyc) { }
        t += step_cyc;
        step_pulse(m->step_port, m->step_pin);
        moved++;

        if (++in_win < HOME_WIN) continue;
        in_win = 0;

        uint32_t now = __HAL_TIM_GET_COUNTER(a->fenc);
        int32_t  d   = a->is32 ? (int32_t)(now - prev)
                               : (int16_t)((uint16_t)now - (uint16_t)prev);
        prev = now;
        int32_t adv = d * a->esign * a->dir;   /* >0 when advancing to the stop */

        if (++win_n <= HOME_ARM) continue;
        if (win_n == HOME_ARM + 1u && adv <= 0) return 2;

        if (adv < trip) { if (++bad >= HOME_CONFIRM) { tripped = 1; break; } }
        else            { bad = 0; }
    }
    if (!tripped) return 1;

    m->current_pos = 0;
    m->target_pos  = 0;
    m->accum_num   = 0;
    m->vel         = 0.0f;
    m->emit_accum  = 0.0f;
    __HAL_TIM_SET_COUNTER(a->fenc, 0);
    return 0;
}

int home_m2_m3(void)
{
    int r = 0;
    for (int i = 0; i < 2; i++) { int e = home_one(&home_ax[i]); if (e) r = e; }

    /* Required. The input encoders kept counting in hardware while this
       blocked. Without the resync the next control_tick sees the whole
       accumulated knob delta at once and commands a burst move. */
    for (int i = 0; i < 4; i++)
        in_enc[i].prev = __HAL_TIM_GET_COUNTER(in_enc[i].tim);

    return r;
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
        tmc_write_nodeconf(mot[m].pdn_port, mot[m].pdn_pin, mot[m].tmc_addr, 2);
    }

    HAL_Delay(10);

    /* 3. Configure; enable ONLY drivers whose config verified */
    for (int m = 0; m < N_MOT; m++) {
        TMC_Status_t st = TMC2209uart(500, MICROSTEP,
                                      mot[m].pdn_port, mot[m].pdn_pin, mot[m].tmc_addr);
        if (st == TMC_OK)
            HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_RESET);
        else
            printf("[MAIN] M%d config FAILED (code=%d) - left disabled\r\n", m + 1, (int)st);

        HAL_Delay(1000);
    }


    acc_steps =0; // reset accumulated steps.
    uint32_t last_cyc = DWT->CYCCNT;

   volatile int home_err = home_m2_m3();   /* 0=ok 1=no stall 2=encoder dead */

    //home_probe2(1, GPIO_PIN_SET,   1600);  HAL_Delay(500);
      //home_probe2(1, GPIO_PIN_RESET, 1600);  HAL_Delay(500);
     // home_probe2(2, GPIO_PIN_SET,   1600);  HAL_Delay(500);
      //home_probe2(2, GPIO_PIN_RESET, 1600);


   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

	      /* main.c — inside while(1), diagnosis only, remove afterwards.
	       * Each read blocks ~1-2 ms with IRQs off + printf blocks: motors WILL stutter every 2 s. */
	      /*
	      static uint32_t last_diag = 0;
	      if (HAL_GetTick() - last_diag >= 2000) {
	          last_diag = HAL_GetTick();
	          uint32_t gstat = 0, gconf = 0, ifcnt = 0;
	          TMC_Status_t r1 = tmc_diag_read(GPIOB, GPIO_PIN_1, 0, 0x01, &gstat);
	          TMC_Status_t r2 = tmc_diag_read(GPIOB, GPIO_PIN_1, 0, 0x00, &gconf);
	          TMC_Status_t r3 = tmc_diag_read(GPIOB, GPIO_PIN_1, 0, 0x02, &ifcnt);
	          printf("[M5] GSTAT: code=%d val=0x%02lX | GCONF: code=%d val=0x%08lX | IFCNT: code=%d val=%lu\r\n",
	                 (int)r1, (unsigned long)gstat,
	                 (int)r2, (unsigned long)gconf,
	                 (int)r3, (unsigned long)(ifcnt & 0xFFu));
	      }
	      */

	      //delay_us(1000);
	      //HAL_Delay(1);
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
