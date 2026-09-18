/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc2209_uart.h"
#include "stm32f4xx_hal_adc.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===== Motor and input-encoder hardware conversion ======================= */
#define MICROSTEP                              16u
#define MOTOR_FULL_STEPS_PER_REV             200L
#define INPUT_ENCODER_PULSES_PER_REV        2048L
#define INPUT_ENCODER_QUADRATURE_FACTOR        4L

#define MOTOR_MICROSTEPS_PER_REV \
    (MOTOR_FULL_STEPS_PER_REV * MICROSTEP)
#define INPUT_ENCODER_COUNTS_PER_REV \
    (INPUT_ENCODER_PULSES_PER_REV * INPUT_ENCODER_QUADRATURE_FACTOR)
#define INPUT_COUNT_TO_MICROSTEP_NUM MOTOR_MICROSTEPS_PER_REV
#define INPUT_COUNT_TO_MICROSTEP_DEN INPUT_ENCODER_COUNTS_PER_REV

#define STEP_PULSE_CYCLES       (SystemCoreClock / 1000000u)
#define MOTOR_COUNT                                  5
#define HOMING_MOTOR_COUNT                           3
#define INPUT_ENCODER_COUNT                          4

#define MOTOR_M1_INDEX                              0u
#define MOTOR_M2_INDEX                              1u
#define MOTOR_M3_INDEX                              2u
#define MOTOR_M4_INDEX                              3u
#define MOTOR_M5_INDEX                              4u

#define INPUT_ENCODER_M1_INDEX                      0u
#define INPUT_ENCODER_M2_INDEX                      1u
#define INPUT_ENCODER_M3_INDEX                      2u
#define INPUT_ENCODER_M45_INDEX                     3u

/* ===== Driver initialization and user interface ========================== */
#define TMC_INIT_RETRIES                            3u
#define TMC_INIT_RETRY_DELAY_MS                   100u
#define BUTTON_DEBOUNCE_MS                         200u
#define UART_BAUD_RATE                          115200u
#define UART_TX_TIMEOUT_MS                          30u

#define M1_RUN_CURRENT_MA                         1000u
#define M2_RUN_CURRENT_MA                         1000u
#define M3_RUN_CURRENT_MA                          800u
#define M4_RUN_CURRENT_MA                         1000u
#define M5_RUN_CURRENT_MA                         1000u

/* ===== Application mode selection ======================================== */
#define APP_MODE_INTEGRATED_CONTROL                  0u
#define APP_MODE_M23_FEEDER                          1u
#ifndef ACTIVE_APP_MODE
#define ACTIVE_APP_MODE              APP_MODE_INTEGRATED_CONTROL
#endif

/* ===== M2/M3 feeder drive ================================================ */
#define M23_FEEDER_STAGE_COUNT                       7u
#define M23_FEEDER_STAGE_MS                      10000u
#define M2_FEEDER_DIRECTION                         (-1)
#define M3_FEEDER_DIRECTION                         (-M2_FEEDER_DIRECTION)

/* ===== Manual-control values shared by all axes ========================== */
#define MANUAL_FIXED_GAIN_PCT                       100
#define MANUAL_STOP_SPEED_EPSILON_SPS              0.5f

/* ===== M1/M2 manual control ============================================== */
#define M1_MANUAL_SCALE_NUM                          95L
#define M1_MANUAL_SCALE_DEN                          26L
#define M2_MANUAL_SCALE_NUM                         171L
#define M2_MANUAL_SCALE_DEN                         181L
#define M12_MANUAL_MAX_SPEED_SPS                51200.0f
#define M12_MANUAL_ACCEL_SPS2                  160000.0f

#define M1_LIMIT_MARGIN_COUNTS                      200
#define M2_LIMIT_MARGIN_COUNTS                      200
#define M12_LIMIT_RELEASE_COUNTS                     50

/* ===== M3 manual control ================================================= */
#define M3_MANUAL_SCALE_NUM                       25961L
#define M3_MANUAL_SCALE_DEN                        1150L
#define M3_MANUAL_MAX_SPEED_SPS                120000.0f
#define M3_MANUAL_ACCEL_SPS2                    80000.0f
#define M3_POSITION_REPORT_PERIOD_MS                1000u

/* ===== M3 homing ========================================================= */
#define M3_ENDSTOP_ACTIVE_STATE              GPIO_PIN_SET
#define M3_HOME_SEEK_DIRECTION                       (-1)
#define M3_HOME_SEEK_MAX_STEPS                    128000u
#define M3_HOME_SEEK_RATE_SPS                        6000u
#define M3_HOME_BACKOFF_STEPS                       22000u
#define M3_HOME_BACKOFF_START_RATE_SPS                300u
#define M3_HOME_BACKOFF_RATE_SPS                     6000u
#define M3_HOME_BACKOFF_RAMP_STEPS                   1500u
#define M3_HOME_RELEASE_MAX_STEPS                    3000u
#define M3_HOME_RELEASE_CONFIRM_STEPS                   8u
#define M3_HOME_SETTLE_MS                             100u

/* Keep the physical endstop latched until M3 has backed away this far. */
#define M3_MANUAL_LIMIT_RELEASE_STEPS                   50L

/* ===== M4/M5 manual and coupled control ================================== */
#define M45_MANUAL_MAX_SPEED_SPS                51200.0f
#define M45_MANUAL_ACCEL_SPS2                  160000.0f
#define COUPLED_M4_BASE_SPEED_SPS                 800.0f
#define COUPLED_M4_RETRACT_SIGN                     1.0f
#define COUPLED_M5_RETRACT_SIGN                    -1.0f

#define POT_ADC_MAX_COUNTS                         4095u
#define POT_ADC_CENTER_COUNTS                      2048u
#define POT_ADC_DEADBAND_COUNTS                     128u
#define POT_SAMPLE_PERIOD_MS                         10u
#define POT_FILTER_SHIFT                              3u
#define POT_RATIO_SCALE                            1000L
#define POT_RATIO_MIN_X1000                         200L
#define POT_RATIO_CENTER_X1000                     1000L
#define POT_RATIO_MAX_X1000                        1500L
#define POT_ADC_REVERSED                               0u

#ifndef GAIN_LED_Pin
#define GAIN_LED_Pin GPIO_PIN_12
#endif
#ifndef GAIN_LED_GPIO_Port
#define GAIN_LED_GPIO_Port GPIOB
#endif
#ifndef M3_ENDSTOP_Pin
#ifdef M1_ENDSTOP_Pin
#define M3_ENDSTOP_Pin M1_ENDSTOP_Pin
#else
#define M3_ENDSTOP_Pin GPIO_PIN_14
#endif
#endif
#ifndef M3_ENDSTOP_GPIO_Port
#ifdef M1_ENDSTOP_GPIO_Port
#define M3_ENDSTOP_GPIO_Port M1_ENDSTOP_GPIO_Port
#else
#define M3_ENDSTOP_GPIO_Port GPIOB
#endif
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
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
typedef enum {

    MODE_PRECISION = 2,

    MODE_FAST = 3

} Mode;

static volatile Mode current_mode = MODE_PRECISION;


static void uart_printf(const char *format, ...)
{
    char buffer[192];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length <= 0)
        return;
    if (length >= (int)sizeof(buffer))
        length = (int)sizeof(buffer) - 1;

    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buffer,
                           (uint16_t)length, UART_TX_TIMEOUT_MS);
}


static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void dwt_delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { /* spin */ }
}

static inline void step_pulse(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    dwt_delay_cycles(STEP_PULSE_CYCLES);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    dwt_delay_cycles(STEP_PULSE_CYCLES);
}

typedef struct { TIM_HandleTypeDef *tim; uint8_t is32; uint32_t prev; } InEnc;

static InEnc in_enc[INPUT_ENCODER_COUNT] = {
    { &htim1, 0, 0 },  /* Encoder 1 -> M1 */
    { &htim2, 1, 0 },  /* Encoder 2 -> M2; TIM2 is 32-bit */
    { &htim3, 0, 0 },  /* Encoder 3 -> M3 */
    { &htim4, 0, 0 },  /* Encoder 4 -> M4/M5 */
};

static int32_t in_enc_delta(InEnc *e) {
    uint32_t now = __HAL_TIM_GET_COUNTER(e->tim);
    int32_t d = e->is32 ? (int32_t)(now - e->prev)
                        : (int16_t)((uint16_t)now - (uint16_t)e->prev);
    e->prev = now;
    return d;
}

typedef struct {
    GPIO_TypeDef *step_port; uint16_t step_pin;
    GPIO_TypeDef *dir_port;  uint16_t dir_pin;
    GPIO_TypeDef *en_port;   uint16_t en_pin;
    GPIO_TypeDef *pdn_port;  uint16_t pdn_pin;
    uint8_t  tmc_addr;
    int32_t  manual_scale_num;
    int32_t  manual_scale_den;
    int32_t  dir_sign;
    int64_t  accum_num;
    int32_t  target_pos, current_pos;
    float    vel, emit_accum;
} Motor;

static Motor mot[MOTOR_COUNT] = {
    [MOTOR_M1_INDEX] = {
        .step_port = GPIOC, .step_pin = GPIO_PIN_1,
        .dir_port = GPIOC,  .dir_pin = GPIO_PIN_2,
        .en_port = GPIOC,   .en_pin = GPIO_PIN_0,
        .pdn_port = GPIOB,  .pdn_pin = GPIO_PIN_0,
        .tmc_addr = 0u,
        .manual_scale_num = M1_MANUAL_SCALE_NUM,
        .manual_scale_den = M1_MANUAL_SCALE_DEN,
        .dir_sign = -1,
    },
    [MOTOR_M2_INDEX] = {
        .step_port = GPIOC, .step_pin = GPIO_PIN_4,
        .dir_port = GPIOC,  .dir_pin = GPIO_PIN_5,
        .en_port = GPIOC,   .en_pin = GPIO_PIN_3,
        .pdn_port = GPIOB,  .pdn_pin = GPIO_PIN_0,
        .tmc_addr = 1u,
        .manual_scale_num = M2_MANUAL_SCALE_NUM,
        .manual_scale_den = M2_MANUAL_SCALE_DEN,
        .dir_sign = +1,
    },
    [MOTOR_M3_INDEX] = {
        .step_port = GPIOC, .step_pin = GPIO_PIN_9,
        .dir_port = GPIOC,  .dir_pin = GPIO_PIN_10,
        .en_port = GPIOC,   .en_pin = GPIO_PIN_8,
        .pdn_port = GPIOB,  .pdn_pin = GPIO_PIN_0,
        .tmc_addr = 2u,
        .manual_scale_num = M3_MANUAL_SCALE_NUM,
        .manual_scale_den = M3_MANUAL_SCALE_DEN,
        .dir_sign = +1,
    },
    [MOTOR_M4_INDEX] = {
        .step_port = GPIOC, .step_pin = GPIO_PIN_12,
        .dir_port = GPIOC,  .dir_pin = GPIO_PIN_13,
        .en_port = GPIOC,   .en_pin = GPIO_PIN_11,
        .pdn_port = GPIOB,  .pdn_pin = GPIO_PIN_0,
        .tmc_addr = 3u,
        .manual_scale_num = 1L,
        .manual_scale_den = 1L,
        .dir_sign = +1,
    },
    [MOTOR_M5_INDEX] = {
        .step_port = GPIOA, .step_pin = GPIO_PIN_11,
        .dir_port = GPIOD,  .dir_pin = GPIO_PIN_2,
        .en_port = GPIOA,   .en_pin = GPIO_PIN_12,
        .pdn_port = GPIOB,  .pdn_pin = GPIO_PIN_1,
        .tmc_addr = 0u,
        .manual_scale_num = 1L,
        .manual_scale_den = 1L,
        .dir_sign = +1,
    },
};

static uint8_t motor_ready[MOTOR_COUNT] = {0};
static const uint16_t motor_run_current_ma[MOTOR_COUNT] = {
    [MOTOR_M1_INDEX] = M1_RUN_CURRENT_MA,
    [MOTOR_M2_INDEX] = M2_RUN_CURRENT_MA,
    [MOTOR_M3_INDEX] = M3_RUN_CURRENT_MA,
    [MOTOR_M4_INDEX] = M4_RUN_CURRENT_MA,
    [MOTOR_M5_INDEX] = M5_RUN_CURRENT_MA,
};

typedef enum {
    SEL_M4 = 0,
    SEL_M5,
    SEL_M4_M5_COUPLED
} Enc4Sel;

static volatile Enc4Sel enc4_sel = SEL_M4;
static uint32_t pot_filter_q3 = POT_ADC_CENTER_COUNTS << POT_FILTER_SHIFT;
static int32_t coupled_m5_ratio_x1000 = POT_RATIO_CENTER_X1000;

static const char *enc4_mode_name(Enc4Sel selected)
{
    switch (selected) {
    case SEL_M4:            return "M4";
    case SEL_M5:            return "M5";
    case SEL_M4_M5_COUPLED: return "M4_M5_COUPLED";
    default:                return "UNKNOWN";
    }
}

static uint32_t pot_read_adc(void)
{
    uint32_t value = POT_ADC_CENTER_COUNTS;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return value;

    if (HAL_ADC_PollForConversion(&hadc1, 2u) == HAL_OK)
        value = HAL_ADC_GetValue(&hadc1);

    (void)HAL_ADC_Stop(&hadc1);
    return value;
}

static int32_t pot_adc_to_ratio_x1000(uint32_t adc)
{
#if POT_ADC_REVERSED
    adc = POT_ADC_MAX_COUNTS - adc;
#endif

    const uint32_t low = POT_ADC_CENTER_COUNTS - POT_ADC_DEADBAND_COUNTS;
    const uint32_t high = POT_ADC_CENTER_COUNTS + POT_ADC_DEADBAND_COUNTS;

    if (adc >= low && adc <= high)
        return POT_RATIO_CENTER_X1000;

    if (adc < low) {
        return POT_RATIO_MIN_X1000 +
               (int32_t)(((uint64_t)adc *
                          (POT_RATIO_CENTER_X1000 - POT_RATIO_MIN_X1000) +
                          low / 2u) / low);
    }

    return POT_RATIO_CENTER_X1000 +
           (int32_t)(((uint64_t)(adc - high) *
                      (POT_RATIO_MAX_X1000 - POT_RATIO_CENTER_X1000) +
                      (POT_ADC_MAX_COUNTS - high) / 2u) /
                     (POT_ADC_MAX_COUNTS - high));
}

static void pot_ratio_update(void)
{
    static uint32_t last_sample_ms = 0u;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_sample_ms) < POT_SAMPLE_PERIOD_MS)
        return;
    last_sample_ms = now;

    uint32_t raw = pot_read_adc();
    int32_t error_q3 = (int32_t)(raw << POT_FILTER_SHIFT) -
                       (int32_t)pot_filter_q3;
    pot_filter_q3 = (uint32_t)((int32_t)pot_filter_q3 +
                               error_q3 / (int32_t)(1u << POT_FILTER_SHIFT));

    uint32_t filtered =
        (pot_filter_q3 + (1u << (POT_FILTER_SHIFT - 1u))) >> POT_FILTER_SHIFT;
    coupled_m5_ratio_x1000 = pot_adc_to_ratio_x1000(filtered);
}

static uint8_t motor_driver_init(uint8_t mi)
{
    Motor *m = &mot[mi];

    motor_ready[mi] = 0u;
    HAL_GPIO_WritePin(m->en_port, m->en_pin, GPIO_PIN_SET);

    for (uint32_t attempt = 1u; attempt <= TMC_INIT_RETRIES; attempt++) {
        TMC_Status_t status = TMC2209uart(motor_run_current_ma[mi], MICROSTEP,
                                         m->pdn_port, m->pdn_pin, m->tmc_addr);
        if (status == TMC_OK) {
            motor_ready[mi] = 1u;
            HAL_GPIO_WritePin(m->en_port, m->en_pin, GPIO_PIN_RESET);
            return 1u;
        }

        HAL_Delay(TMC_INIT_RETRY_DELAY_MS);
    }

    return 0u;
}

#if ACTIVE_APP_MODE == APP_MODE_M23_FEEDER
static inline void m23_feeder_step_pulse(Motor *m2, Motor *m3)
{
    if (m2->step_port == m3->step_port) {
        uint16_t step_pins = (uint16_t)(m2->step_pin | m3->step_pin);

        HAL_GPIO_WritePin(m2->step_port, step_pins, GPIO_PIN_SET);
        dwt_delay_cycles(STEP_PULSE_CYCLES);
        HAL_GPIO_WritePin(m2->step_port, step_pins, GPIO_PIN_RESET);
        dwt_delay_cycles(STEP_PULSE_CYCLES);
        return;
    }

    HAL_GPIO_WritePin(m2->step_port, m2->step_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m3->step_port, m3->step_pin, GPIO_PIN_SET);
    dwt_delay_cycles(STEP_PULSE_CYCLES);
    HAL_GPIO_WritePin(m2->step_port, m2->step_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m3->step_port, m3->step_pin, GPIO_PIN_RESET);
    dwt_delay_cycles(STEP_PULSE_CYCLES);
}

static void m23_feeder_run(void)
{
    static const uint32_t stage_speed_sps[M23_FEEDER_STAGE_COUNT] = {
        1200u, 1200u, 1700u, 1700u, 1200u, 1200u, 1700u
    };
    Motor *m2 = &mot[MOTOR_M2_INDEX];
    Motor *m3 = &mot[MOTOR_M3_INDEX];

    HAL_GPIO_WritePin(m2->dir_port, m2->dir_pin,
                      (M2_FEEDER_DIRECTION > 0) ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(m3->dir_port, m3->dir_pin,
                      (M3_FEEDER_DIRECTION > 0) ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);
    dwt_delay_cycles(SystemCoreClock / 100000u);

    while (1) {
        for (uint32_t stage = 0u; stage < M23_FEEDER_STAGE_COUNT;
             stage++) {
            uint32_t speed_sps = stage_speed_sps[stage];
            uint32_t step_period_cycles = SystemCoreClock / speed_sps;
            uint32_t stage_start_ms = HAL_GetTick();
            uint32_t last_step_cyc = DWT->CYCCNT;

            uart_printf("M2/M3 FEEDER STAGE %lu/%lu: %lu SPS, %lu ms, "
                        "OPPOSITE DIR\r\n",
                        (unsigned long)(stage + 1u),
                        (unsigned long)M23_FEEDER_STAGE_COUNT,
                        (unsigned long)speed_sps,
                        (unsigned long)M23_FEEDER_STAGE_MS);

            while ((uint32_t)(HAL_GetTick() - stage_start_ms) <
                   M23_FEEDER_STAGE_MS) {
                uint32_t now_cyc = DWT->CYCCNT;
                if ((uint32_t)(now_cyc - last_step_cyc) >=
                    step_period_cycles) {
                    last_step_cyc = now_cyc;
                    m23_feeder_step_pulse(m2, m3);
                }
            }
        }
    }
}
#endif

/* current_pos counts emitted STEP pulses; it is not encoder feedback. */
static void control_tick(Motor *m, float dt,
                         float max_speed_sps, float accel_sps2)
{
    int32_t error   = m->target_pos - m->current_pos;
    int32_t abs_err = (error < 0) ? -error : error;

    float v_stop = sqrtf(2.0f * accel_sps2 * (float)abs_err);
    float v_cmd  = (v_stop < max_speed_sps) ? v_stop : max_speed_sps;
    if (error < 0) v_cmd = -v_cmd;

    float dv_max = accel_sps2 * dt;
    float dv = v_cmd - m->vel;
    if (dv >  dv_max) dv =  dv_max;
    if (dv < -dv_max) dv = -dv_max;
    m->vel += dv;

    if (abs_err == 0 &&
        fabsf(m->vel) < MANUAL_STOP_SPEED_EPSILON_SPS) {
        m->vel = 0.0f;
        m->emit_accum = 0.0f;
        return;
    }

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

static void coupled_velocity_tick(Motor *m, float command_vel,
                                  float dt, float accel_sps2)
{
    float dv_max = accel_sps2 * dt;
    float dv = command_vel - m->vel;
    if (dv >  dv_max) dv =  dv_max;
    if (dv < -dv_max) dv = -dv_max;
    m->vel += dv;

    if (fabsf(command_vel) < MANUAL_STOP_SPEED_EPSILON_SPS &&
        fabsf(m->vel) < MANUAL_STOP_SPEED_EPSILON_SPS) {
        m->vel = 0.0f;
        m->emit_accum = 0.0f;
        m->target_pos = m->current_pos;
        m->accum_num = 0;
        return;
    }

    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      (m->vel >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    m->emit_accum += m->vel * dt;
    int32_t n = (int32_t)m->emit_accum;
    m->emit_accum -= (float)n;

    int32_t pulse_count = (n < 0) ? -n : n;
    for (int32_t i = 0; i < pulse_count; i++)
        step_pulse(m->step_port, m->step_pin);

    m->current_pos += n;
    m->target_pos = m->current_pos;
    m->accum_num = 0;
}

static void reset_m4_m5_motion_state(void)
{
    for (int m = 3; m <= 4; m++) {
        mot[m].target_pos = mot[m].current_pos;
        mot[m].vel = 0.0f;
        mot[m].emit_accum = 0.0f;
        mot[m].accum_num = 0;
        HAL_GPIO_WritePin(mot[m].step_port, mot[m].step_pin, GPIO_PIN_RESET);
    }
}


/* Encoder homing for M1/M2. Values come from the latest verified M2/M3 code. */
#define HOME_M1_TOUCH_CURRENT_MA     175u
#define HOME_M2_TOUCH_CURRENT_MA     300u
#define HOME_APPROACH_V              800u
#define HOME_RETURN_V                800u
#define HOME_WIN                      16u
#define HOME_ARM_WINDOWS               3u
#define HOME_RNUM                     25
#define HOME_RDEN                     64
#define HOME_TRIP_LAG                 80
#define HOME_CONFIRM_WINDOWS           2u
#define HOME_SECOND_ARM_COUNTS      6000
#define HOME_M1_LIMIT_STEPS        12000u
#define HOME_M2_LIMIT_STEPS        12288u
#define HOME_M1_SEAT_STEPS            32u
#define HOME_M2_SEAT_STEPS             0u
#define HOME_START_EPS_CNT              4
#define HOME_RETURN_TOL_CNT             2
#define HOME_RETURN_MARGIN             64u
#define HOME_MIN_SPAN_CNT           12000
#define HOME_MAX_SPAN_CNT           30000
#define HOME_M1_SPAN_DEG10           2539u
#define HOME_M1_RETURN2_DEG10        1390u
#define HOME_M2_SPAN_DEG10           2452u
#define HOME_M2_RETURN2_DEG10        1383u

typedef struct {
    uint8_t            mi;
    TIM_HandleTypeDef *fenc;
    uint8_t            is32;
    int8_t             esign;
    int8_t             first_dir;
    uint16_t           span_deg10;
    uint16_t           return2_deg10;
    uint32_t           limit_steps;
    uint16_t           touch_current_ma;
    uint16_t           seat_steps;
} HomeAxis;

static const HomeAxis home_ax[2] = {
    { 0, &htim5, 1, +1, -1, HOME_M1_SPAN_DEG10,
      HOME_M1_RETURN2_DEG10, HOME_M1_LIMIT_STEPS,
      HOME_M1_TOUCH_CURRENT_MA, HOME_M1_SEAT_STEPS },
    { 1, &htim8, 0, -1, -1, HOME_M2_SPAN_DEG10,
      HOME_M2_RETURN2_DEG10, HOME_M2_LIMIT_STEPS,
      HOME_M2_TOUCH_CURRENT_MA, HOME_M2_SEAT_STEPS },
};

typedef enum {
    HOME_OK = 0,
    HOME_NO_HARDSTOP,
    HOME_ENCODER_FAULT,
    HOME_SPAN_FAULT,
    HOME_RETURN_FAULT,
    HOME_DRIVER_FAULT,
    HOME_SWITCH_NOT_RELEASED
} HomeStatus;

static const char *home_status_name(HomeStatus status)
{
    switch (status) {
    case HOME_OK:                  return "OK";
    case HOME_NO_HARDSTOP:         return "NO_HARDSTOP";
    case HOME_ENCODER_FAULT:       return "ENCODER_FAULT";
    case HOME_SPAN_FAULT:          return "SPAN_FAULT";
    case HOME_RETURN_FAULT:        return "RETURN_FAULT";
    case HOME_DRIVER_FAULT:        return "DRIVER_FAULT";
    case HOME_SWITCH_NOT_RELEASED: return "SWITCH_NOT_RELEASED";
    default:                       return "UNKNOWN";
    }
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
    uint8_t  tripped;
    uint8_t  detection_armed;
    uint8_t  confirm;
} HomeSeekState;

typedef struct {
    int32_t hard_min_count;
    int32_t hard_max_count;
    int32_t min_count;
    int32_t max_count;
    int32_t position;
    uint8_t ready;
    uint8_t min_active;
    uint8_t max_active;
} ManualLimit;

static ManualLimit m12_limit[2] = {0};
static uint8_t m3_manual_endstop_latched = 0u;
static int32_t m3_manual_endstop_position = 0;

static HomeStatus m3_home_begin(void);
static HomeStatus m3_home_service(void);
static uint8_t m3_home_finished(void);
static void m3_home_abort(void);

static void reset_motor_motion_state(Motor *m)
{
    m->current_pos = 0;
    m->target_pos = 0;
    m->accum_num = 0;
    m->vel = 0.0f;
    m->emit_accum = 0.0f;
}

static int32_t home_encoder_delta(const HomeAxis *a,
                                  uint32_t now, uint32_t prev)
{
    return a->is32 ? (int32_t)(now - prev)
                   : (int16_t)((uint16_t)now - (uint16_t)prev);
}

static int32_t home_position_from(const HomeAxis *a, uint32_t origin)
{
    int32_t delta = home_encoder_delta(
        a, __HAL_TIM_GET_COUNTER(a->fenc), origin);
    return delta * a->esign;
}

static void stop_motor_motion(Motor *m)
{
    m->target_pos = m->current_pos;
    m->accum_num = 0;
    m->vel = 0.0f;
    m->emit_accum = 0.0f;
    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
}

static void apply_m12_limit(uint8_t axis, int32_t *input)
{
    ManualLimit *limit = &m12_limit[axis];
    Motor *m = &mot[axis];
    const HomeAxis *a = &home_ax[axis];

    if (!limit->ready)
        return;

    uint8_t old_min = limit->min_active;
    uint8_t old_max = limit->max_active;
    limit->position = home_position_from(a, 0u);

    if (limit->min_active)
        limit->min_active =
            limit->position < limit->min_count + M12_LIMIT_RELEASE_COUNTS;
    else
        limit->min_active = limit->position <= limit->min_count;

    if (limit->max_active)
        limit->max_active =
            limit->position > limit->max_count - M12_LIMIT_RELEASE_COUNTS;
    else
        limit->max_active = limit->position >= limit->max_count;

    uint8_t min_activated = !old_min && limit->min_active;
    uint8_t max_activated = !old_max && limit->max_active;

    uint8_t block_input =
        (limit->min_active && *input < 0) ||
        (limit->max_active && *input > 0);
    if (block_input) {
        *input = 0;
        m->accum_num = 0;
    }

    int32_t logical_error =
        (m->target_pos - m->current_pos) * m->dir_sign;
    float logical_vel = m->vel * (float)m->dir_sign;
    uint8_t moving_outward =
        (limit->min_active &&
         (logical_error < 0 || logical_vel < 0.0f)) ||
        (limit->max_active &&
         (logical_error > 0 || logical_vel > 0.0f));

    if (moving_outward)
        stop_motor_motion(m);

    /* Report only a newly reached manual limit, after motion is stopped. */
    if (min_activated)
        uart_printf("M%u MIN LIMIT pos=%ld\r\n", axis + 1u,
                    (long)limit->position);
    if (max_activated)
        uart_printf("M%u MAX LIMIT pos=%ld\r\n", axis + 1u,
                    (long)limit->position);
}

static void home_set_direction(const HomeAxis *a, int8_t logical_dir)
{
    Motor *m = &mot[a->mi];
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      (logical_dir * m->dir_sign > 0) ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);
    dwt_delay_cycles(SystemCoreClock / 100000u);
}

static HomeStatus home_set_axis_currents(uint8_t touch_mode)
{
    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        Motor *m = &mot[a->mi];
        uint16_t current = touch_mode ?
                           a->touch_current_ma :
                           motor_run_current_ma[a->mi];
        TMC_Status_t tmc_status = TMC_ERR_NO_REPLY;

        if (motor_ready[a->mi])
            tmc_status = TMC2209uart(current, MICROSTEP, m->pdn_port,
                                     m->pdn_pin, m->tmc_addr);

        if (!motor_ready[a->mi] || tmc_status != TMC_OK)
            return HOME_DRIVER_FAULT;
    }
    return HOME_OK;
}

static HomeStatus home_seek_pair(const int8_t seek_dir[2],
                                 const uint8_t allow_already[2],
                                 const int32_t min_arm_counts[2],
                                 HomeStop result[2])
{
    HomeSeekState state[2] = {0};
    const uint32_t step_cycles = SystemCoreClock / HOME_APPROACH_V;
    uint8_t completed = 0u;

    for (int i = 0; i < 2; i++) {
        home_set_direction(&home_ax[i], seek_dir[i]);
        state[i].prev = __HAL_TIM_GET_COUNTER(home_ax[i].fenc);
    }

    uint32_t tick = DWT->CYCCNT;
    while (completed < 2u) {
        while ((uint32_t)(DWT->CYCCNT - tick) < step_cycles) {
            HomeStatus status = m3_home_service();
            if (status != HOME_OK)
                return status;
        }
        tick += step_cycles;

        for (int i = 0; i < 2; i++) {
            const HomeAxis *a = &home_ax[i];
            Motor *m = &mot[a->mi];
            HomeSeekState *s = &state[i];

            if (s->tripped)
                continue;
            if (s->moved >= a->limit_steps)
                return HOME_NO_HARDSTOP;

            step_pulse(m->step_port, m->step_pin);
            s->moved++;
            if (++s->in_window < HOME_WIN)
                continue;
            s->in_window = 0u;

            uint32_t now = __HAL_TIM_GET_COUNTER(a->fenc);
            int32_t delta = home_encoder_delta(a, now, s->prev);
            s->prev = now;
            int32_t advance = delta * a->esign * seek_dir[i];
            s->advanced_counts += advance;
            s->window_n++;

            int64_t expected_counts =
                ((int64_t)s->moved * HOME_RDEN) / HOME_RNUM;
            int64_t lag = expected_counts - s->advanced_counts;

            if (s->window_n == HOME_ARM_WINDOWS) {
                if (allow_already[i] &&
                    s->advanced_counts >= -HOME_START_EPS_CNT &&
                    s->advanced_counts <=  HOME_START_EPS_CNT) {
                    s->tripped = 1u;
                    result[i].stop_pos =
                        __HAL_TIM_GET_COUNTER(a->fenc);
                    result[i].moved_steps = s->moved;
                    completed++;
                    continue;
                }
                if (s->advanced_counts < -HOME_START_EPS_CNT)
                    return HOME_ENCODER_FAULT;
                if (min_arm_counts[i] == 0) {
                    s->lag0 = lag;
                    s->detection_armed = 1u;
                }
                continue;
            }
            if (s->window_n < HOME_ARM_WINDOWS)
                continue;

            if (s->advanced_counts < 0)
                return HOME_ENCODER_FAULT;

            if (!s->detection_armed) {
                if (s->advanced_counts < min_arm_counts[i])
                    continue;
                s->lag0 = lag;
                s->lag_delta = 0;
                s->detection_armed = 1u;
                continue;
            }

            s->lag_delta = lag - s->lag0;

            if (s->lag_delta >= HOME_TRIP_LAG) {
                if (++s->confirm >= HOME_CONFIRM_WINDOWS) {
                    s->tripped = 1u;
                    result[i].stop_pos =
                        __HAL_TIM_GET_COUNTER(a->fenc);
                    result[i].moved_steps = s->moved;
                    completed++;
                }
            } else {
                s->confirm = 0u;
            }
        }
    }

    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        Motor *m = &mot[a->mi];
        home_set_direction(a, seek_dir[i]);
        tick = DWT->CYCCNT;

        for (uint16_t n = 0; n < a->seat_steps; n++) {
            while ((uint32_t)(DWT->CYCCNT - tick) < step_cycles) {
                HomeStatus status = m3_home_service();
                if (status != HOME_OK)
                    return status;
            }
            tick += step_cycles;
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
    uint8_t completed = 0u;
    const uint32_t step_cycles = SystemCoreClock / HOME_RETURN_V;

    for (int i = 0; i < 2; i++)
        home_set_direction(&home_ax[i], move_dir[i]);

    uint32_t tick = DWT->CYCCNT;
    while (completed < 2u) {
        while ((uint32_t)(DWT->CYCCNT - tick) < step_cycles) {
            HomeStatus status = m3_home_service();
            if (status != HOME_OK)
                return status;
        }
        tick += step_cycles;

        for (int i = 0; i < 2; i++) {
            const HomeAxis *a = &home_ax[i];
            Motor *m = &mot[a->mi];
            if (done[i])
                continue;

            int32_t position =
                home_position_from(a, origin[i]) * move_dir[i];
            if (position >= target_counts[i] - HOME_RETURN_TOL_CNT) {
                done[i] = 1u;
                completed++;
                continue;
            }
            if (moved[i] >= step_limit[i])
                return HOME_RETURN_FAULT;

            step_pulse(m->step_port, m->step_pin);
            moved[i]++;
        }
    }
    return HOME_OK;
}

static HomeStatus home_m1_m2_m3(void)
{
    HomeStop first[2] = {0};
    HomeStop second[2] = {0};
    int8_t first_dir[2];
    int8_t second_dir[2];
    const uint8_t allow_first[2] = {1u, 1u};
    const uint8_t allow_second[2] = {0u, 0u};
    const int32_t first_arm[2] = {0, 0};
    const int32_t second_arm[2] = {
        HOME_SECOND_ARM_COUNTS, HOME_SECOND_ARM_COUNTS
    };
    int32_t home_counts[2];
    int32_t span_counts[2];
    uint32_t return_limit[2];

    for (int i = 0; i < 2; i++) {
        first_dir[i] = home_ax[i].first_dir;
        second_dir[i] = (int8_t)-home_ax[i].first_dir;
    }

    HomeStatus status = home_set_axis_currents(1u);
    if (status != HOME_OK)
        return status;

    status = m3_home_begin();
    if (status != HOME_OK) {
        (void)home_set_axis_currents(0u);
        return status;
    }

    status = home_seek_pair(first_dir, allow_first, first_arm, first);
    if (status != HOME_OK) {
        m3_home_abort();
        (void)home_set_axis_currents(0u);
        return status;
    }

    status = home_seek_pair(second_dir, allow_second, second_arm, second);
    if (status != HOME_OK) {
        m3_home_abort();
        (void)home_set_axis_currents(0u);
        return status;
    }

    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        span_counts[i] =
            home_encoder_delta(a, second[i].stop_pos, first[i].stop_pos) *
            a->esign * second_dir[i];

        if (span_counts[i] < HOME_MIN_SPAN_CNT ||
            span_counts[i] > HOME_MAX_SPAN_CNT) {
            m3_home_abort();
            (void)home_set_axis_currents(0u);
            return HOME_SPAN_FAULT;
        }

        home_counts[i] =
            (int32_t)(((int64_t)span_counts[i] * a->return2_deg10 +
                       a->span_deg10 / 2u) / a->span_deg10);
        return_limit[i] =
            (uint32_t)(((uint64_t)second[i].moved_steps *
                        a->return2_deg10 + a->span_deg10 - 1u) /
                       a->span_deg10) + HOME_RETURN_MARGIN;

    }

    status = home_set_axis_currents(0u);
    if (status != HOME_OK) {
        m3_home_abort();
        return status;
    }

    status = home_move_pair(
        (const uint32_t[2]){second[0].stop_pos, second[1].stop_pos},
        first_dir, home_counts, return_limit);
    if (status != HOME_OK) {
        m3_home_abort();
        return status;
    }

    for (int i = 0; i < 2; i++) {
        const HomeAxis *a = &home_ax[i];
        int32_t margin = (i == 0) ?
                         M1_LIMIT_MARGIN_COUNTS :
                         M2_LIMIT_MARGIN_COUNTS;

        if (span_counts[i] <= margin * 2)
            return HOME_SPAN_FAULT;

        m12_limit[i].hard_min_count = home_counts[i] - span_counts[i];
        m12_limit[i].hard_max_count = home_counts[i];
        m12_limit[i].min_count =
            m12_limit[i].hard_min_count + margin;
        m12_limit[i].max_count =
            m12_limit[i].hard_max_count - margin;
        m12_limit[i].position = 0;
        m12_limit[i].min_active = 0u;
        m12_limit[i].max_active = 0u;
        m12_limit[i].ready = 1u;

        reset_motor_motion_state(&mot[a->mi]);
        __HAL_TIM_SET_COUNTER(a->fenc, 0);
    }

    while (!m3_home_finished()) {
        status = m3_home_service();
        if (status != HOME_OK) {
            m3_home_abort();
            return status;
        }
    }
    return HOME_OK;
}

static uint8_t m3_endstop_active(void)
{
    return HAL_GPIO_ReadPin(M3_ENDSTOP_GPIO_Port, M3_ENDSTOP_Pin) ==
           M3_ENDSTOP_ACTIVE_STATE;
}

static void m3_home_set_direction(Motor *m, int8_t direction)
{
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    dwt_delay_cycles(SystemCoreClock / 100000u);
}

typedef enum {
    M3_HOME_IDLE = 0,
    M3_HOME_SEEK,
    M3_HOME_SEEK_SETTLE,
    M3_HOME_BACKOFF,
    M3_HOME_RELEASE_SETTLE,
    M3_HOME_DONE,
    M3_HOME_FAILED
} M3HomeState;

typedef struct {
    M3HomeState state;
    HomeStatus error;
    uint32_t tick;
    uint32_t settle_ms;
    uint32_t seek_steps;
    uint32_t backoff_steps;
    uint32_t release_step;
    uint8_t release_confirm;
    uint8_t release_seen;
} M3HomeContext;

static M3HomeContext m3_home = {0};

static HomeStatus m3_home_fail(HomeStatus status)
{
    Motor *m = &mot[2];
    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
    m3_home.state = M3_HOME_FAILED;
    m3_home.error = status;
    return status;
}

static HomeStatus m3_home_begin(void)
{
    Motor *m = &mot[2];

    HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);

    if (!motor_ready[2])
        return HOME_DRIVER_FAULT;

    m3_home.error = HOME_OK;
    m3_home.seek_steps = 0u;
    m3_home.backoff_steps = 0u;
    m3_home.release_step = 0u;
    m3_home.release_confirm = 0u;
    m3_home.release_seen = 0u;

    if (m3_endstop_active()) {
        uart_printf("M3 HOME ENDSTOP seek_steps=0 backoff=%lu\r\n",
                    (unsigned long)M3_HOME_BACKOFF_STEPS);
        m3_home.settle_ms = HAL_GetTick();
        m3_home.state = M3_HOME_SEEK_SETTLE;
    } else {
        m3_home_set_direction(m, M3_HOME_SEEK_DIRECTION);
        m3_home.tick = DWT->CYCCNT;
        m3_home.state = M3_HOME_SEEK;
    }
    return HOME_OK;
}

static HomeStatus m3_home_service(void)
{
    Motor *m = &mot[2];
    uint32_t now;
    uint32_t backoff_rate_sps;

    switch (m3_home.state) {
    case M3_HOME_SEEK:
        if (m3_endstop_active()) {
            HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
            uart_printf("M3 HOME ENDSTOP seek_steps=%lu backoff=%lu\r\n",
                        (unsigned long)m3_home.seek_steps,
                        (unsigned long)M3_HOME_BACKOFF_STEPS);
            m3_home.settle_ms = HAL_GetTick();
            m3_home.state = M3_HOME_SEEK_SETTLE;
            return HOME_OK;
        }

        now = DWT->CYCCNT;
        if ((uint32_t)(now - m3_home.tick) <
            SystemCoreClock / M3_HOME_SEEK_RATE_SPS)
            return HOME_OK;

        m3_home.tick = now;
        step_pulse(m->step_port, m->step_pin);
        m3_home.seek_steps++;

        if (m3_endstop_active()) {
            HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
            uart_printf("M3 HOME ENDSTOP seek_steps=%lu backoff=%lu\r\n",
                        (unsigned long)m3_home.seek_steps,
                        (unsigned long)M3_HOME_BACKOFF_STEPS);
            m3_home.settle_ms = HAL_GetTick();
            m3_home.state = M3_HOME_SEEK_SETTLE;
        } else if (m3_home.seek_steps >= M3_HOME_SEEK_MAX_STEPS) {
            uart_printf("M3 HOME NO ENDSTOP seek_steps=%lu raw=%u dir_pin=%u\r\n",
                        (unsigned long)m3_home.seek_steps,
                        (unsigned)(HAL_GPIO_ReadPin(M3_ENDSTOP_GPIO_Port,
                                                   M3_ENDSTOP_Pin) ==
                                   GPIO_PIN_SET),
                        (unsigned)(HAL_GPIO_ReadPin(m->dir_port, m->dir_pin) ==
                                   GPIO_PIN_SET));
            return m3_home_fail(HOME_NO_HARDSTOP);
        }
        return HOME_OK;

    case M3_HOME_SEEK_SETTLE:
        if ((uint32_t)(HAL_GetTick() - m3_home.settle_ms) <
            M3_HOME_SETTLE_MS)
            return HOME_OK;

        m3_home_set_direction(m, (int8_t)-M3_HOME_SEEK_DIRECTION);
        uart_printf("M3 HOME BACKOFF START dir_pin=%u start=%lu max=%lu\r\n",
                    (unsigned)(HAL_GPIO_ReadPin(m->dir_port, m->dir_pin) ==
                               GPIO_PIN_SET),
                    (unsigned long)M3_HOME_BACKOFF_START_RATE_SPS,
                    (unsigned long)M3_HOME_BACKOFF_RATE_SPS);
        m3_home.tick = DWT->CYCCNT;
        m3_home.state = M3_HOME_BACKOFF;
        return HOME_OK;

    case M3_HOME_BACKOFF:
        backoff_rate_sps = M3_HOME_BACKOFF_RATE_SPS;
        if (m3_home.backoff_steps < M3_HOME_BACKOFF_RAMP_STEPS) {
            backoff_rate_sps =
                M3_HOME_BACKOFF_START_RATE_SPS +
                (uint32_t)(((uint64_t)(M3_HOME_BACKOFF_RATE_SPS -
                                       M3_HOME_BACKOFF_START_RATE_SPS) *
                            m3_home.backoff_steps) /
                           M3_HOME_BACKOFF_RAMP_STEPS);
        }

        now = DWT->CYCCNT;
        if ((uint32_t)(now - m3_home.tick) <
            SystemCoreClock / backoff_rate_sps)
            return HOME_OK;

        m3_home.tick = now;
        step_pulse(m->step_port, m->step_pin);
        m3_home.backoff_steps++;

        if (!m3_home.release_seen) {
            if (!m3_endstop_active()) {
                if (m3_home.release_confirm <
                    M3_HOME_RELEASE_CONFIRM_STEPS)
                    m3_home.release_confirm++;
            } else {
                m3_home.release_confirm = 0u;
            }

            if (m3_home.release_confirm >=
                M3_HOME_RELEASE_CONFIRM_STEPS) {
                m3_home.release_seen = 1u;
                m3_home.release_step = m3_home.backoff_steps;
                uart_printf("M3 HOME ENDSTOP RELEASED step=%lu\r\n",
                            (unsigned long)m3_home.release_step);
            } else if (m3_home.backoff_steps >=
                       M3_HOME_RELEASE_MAX_STEPS) {
                HAL_GPIO_WritePin(m->step_port, m->step_pin,
                                  GPIO_PIN_RESET);
                uart_printf("M3 HOME BACKOFF STALL steps=%lu raw=%u dir_pin=%u\r\n",
                            (unsigned long)m3_home.backoff_steps,
                            (unsigned)(HAL_GPIO_ReadPin(
                                           M3_ENDSTOP_GPIO_Port,
                                           M3_ENDSTOP_Pin) == GPIO_PIN_SET),
                            (unsigned)(HAL_GPIO_ReadPin(
                                           m->dir_port, m->dir_pin) ==
                                       GPIO_PIN_SET));
                return m3_home_fail(HOME_SWITCH_NOT_RELEASED);
            }
        }

        if (m3_home.backoff_steps >= M3_HOME_BACKOFF_STEPS) {
            HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
            uart_printf("M3 HOME BACKOFF DONE steps=%lu release_step=%lu raw=%u\r\n",
                        (unsigned long)m3_home.backoff_steps,
                        (unsigned long)m3_home.release_step,
                        (unsigned)(HAL_GPIO_ReadPin(M3_ENDSTOP_GPIO_Port,
                                                   M3_ENDSTOP_Pin) ==
                                   GPIO_PIN_SET));
            m3_home.settle_ms = HAL_GetTick();
            m3_home.state = M3_HOME_RELEASE_SETTLE;
        }
        return HOME_OK;

    case M3_HOME_RELEASE_SETTLE:
        if ((uint32_t)(HAL_GetTick() - m3_home.settle_ms) <
            M3_HOME_SETTLE_MS)
            return HOME_OK;

        if (m3_endstop_active())
            return m3_home_fail(HOME_SWITCH_NOT_RELEASED);

        reset_motor_motion_state(m);
        m3_home.state = M3_HOME_DONE;
        return HOME_OK;

    case M3_HOME_FAILED:
        return m3_home.error;

    case M3_HOME_IDLE:
    case M3_HOME_DONE:
    default:
        return HOME_OK;
    }
}

static uint8_t m3_home_finished(void)
{
    return m3_home.state == M3_HOME_DONE;
}

static void m3_home_abort(void)
{
    if (m3_home.state == M3_HOME_SEEK_SETTLE ||
        m3_home.state == M3_HOME_BACKOFF ||
        m3_home.state == M3_HOME_RELEASE_SETTLE) {
        uart_printf("M3 HOME SAFE BACKOFF BEFORE ABORT\r\n");
        while (!m3_home_finished() &&
               m3_home.state != M3_HOME_FAILED) {
            if (m3_home_service() != HOME_OK)
                break;
        }
    }
    HAL_GPIO_WritePin(mot[2].step_port, mot[2].step_pin, GPIO_PIN_RESET);
}

static void resync_input_encoders(void)
{
    for (int i = 0; i < INPUT_ENCODER_COUNT; i++)
        in_enc[i].prev = __HAL_TIM_GET_COUNTER(in_enc[i].tim);
}

/* ============ Post-homing manual control ============ */

static void manual_target_update(Motor *m, int32_t input, int32_t gain_pct)
{
    int64_t one_step =
        (int64_t)100 * INPUT_COUNT_TO_MICROSTEP_DEN *
        m->manual_scale_den;

    m->accum_num +=
        (int64_t)input * gain_pct * INPUT_COUNT_TO_MICROSTEP_NUM *
        m->manual_scale_num * m->dir_sign;

    while (m->accum_num >= one_step) {
        m->accum_num -= one_step;
        m->target_pos++;
    }
    while (m->accum_num <= -one_step) {
        m->accum_num += one_step;
        m->target_pos--;
    }
}

static void manual_position_tick(uint8_t mi, int32_t input,
                                 int32_t gain_pct, float dt,
                                 float max_speed_sps, float accel_sps2)
{
    Motor *m = &mot[mi];

    if (!motor_ready[mi])
        return;

    manual_target_update(m, input, gain_pct);
    control_tick(m, dt, max_speed_sps, accel_sps2);
}

static void m12_manual_tick(uint8_t axis, int32_t input, float dt)
{
    if (axis > MOTOR_M2_INDEX)
        return;

    apply_m12_limit(axis, &input);
    manual_position_tick(axis, input, MANUAL_FIXED_GAIN_PCT,
                         dt, M12_MANUAL_MAX_SPEED_SPS,
                         M12_MANUAL_ACCEL_SPS2);
}

static void m3_manual_tick(int32_t input, float dt)
{
    Motor *m = &mot[MOTOR_M3_INDEX];

    if (!motor_ready[MOTOR_M3_INDEX])
        return;

    int32_t raw_input = input;
    uint8_t endstop_active = m3_endstop_active();
    uint8_t endstop_activated = 0u;

    if (!m3_manual_endstop_latched && endstop_active) {
        m3_manual_endstop_latched = 1u;
        m3_manual_endstop_position = m->current_pos;
        endstop_activated = 1u;
    } else if (m3_manual_endstop_latched && !endstop_active) {
        uint8_t released_by_position =
            (M3_HOME_SEEK_DIRECTION < 0) ?
            (m->current_pos >= m3_manual_endstop_position +
                               M3_MANUAL_LIMIT_RELEASE_STEPS) :
            (m->current_pos <= m3_manual_endstop_position -
                               M3_MANUAL_LIMIT_RELEASE_STEPS);
        if (released_by_position)
            m3_manual_endstop_latched = 0u;
    }

    int32_t input_direction = input * m->dir_sign;
    uint8_t input_toward_endstop =
        (M3_HOME_SEEK_DIRECTION < 0) ?
        (input_direction < 0) : (input_direction > 0);
    if (m3_manual_endstop_latched && input_toward_endstop) {
        input = 0;
        m->accum_num = 0;
    }

    int32_t position_error = m->target_pos - m->current_pos;
    uint8_t target_toward_endstop =
        (M3_HOME_SEEK_DIRECTION < 0) ?
        (position_error < 0 || m->vel < 0.0f) :
        (position_error > 0 || m->vel > 0.0f);
    if (m3_manual_endstop_latched && target_toward_endstop)
        stop_motor_motion(m);

    /*
     * When the operator reverses M3, discard motion queued in the old
     * direction. Keep the current velocity so control_tick() decelerates and
     * reverses it using M3_MANUAL_ACCEL_SPS2 instead of changing it abruptly.
     */
    position_error = m->target_pos - m->current_pos;
    uint8_t input_reverses_queued_motion =
        (input_direction > 0 && position_error < 0) ||
        (input_direction < 0 && position_error > 0);
    if (input_reverses_queued_motion) {
        m->target_pos = m->current_pos;
        m->accum_num = 0;
        m->emit_accum = 0.0f;
    }

    /* Report a newly reached limit only after outward motion stops. */
    if (endstop_activated)
        uart_printf("M3 MIN ENDSTOP lower_steps=%ld\r\n",
                    (long)m3_manual_endstop_position);

    manual_target_update(m, input, MANUAL_FIXED_GAIN_PCT);
    control_tick(m, dt, M3_MANUAL_MAX_SPEED_SPS,
                 M3_MANUAL_ACCEL_SPS2);

    static uint32_t last_report_ms = 0u;
    uint32_t now_ms = HAL_GetTick();
    uint8_t motion_active =
        raw_input != 0 ||
        m->target_pos != m->current_pos ||
        fabsf(m->vel) >= MANUAL_STOP_SPEED_EPSILON_SPS;
    if (!endstop_activated && motion_active &&
        (uint32_t)(now_ms - last_report_ms) >=
        M3_POSITION_REPORT_PERIOD_MS) {
        last_report_ms = now_ms;
        uart_printf("M3 POS current_steps=%ld target_steps=%ld lag_steps=%ld\r\n",
                    (long)m->current_pos, (long)m->target_pos,
                    (long)(m->target_pos - m->current_pos));
    }
}

static void m45_manual_tick(Enc4Sel selected, int32_t input,
                            int32_t gain_pct, float dt)
{
    if (selected == SEL_M4_M5_COUPLED) {
        float m4_speed = COUPLED_M4_BASE_SPEED_SPS;

        if (motor_ready[MOTOR_M4_INDEX]) {
            coupled_velocity_tick(
                &mot[MOTOR_M4_INDEX], COUPLED_M4_RETRACT_SIGN * m4_speed,
                dt, M45_MANUAL_ACCEL_SPS2);
        }
        if (motor_ready[MOTOR_M5_INDEX]) {
            coupled_velocity_tick(
                &mot[MOTOR_M5_INDEX], COUPLED_M5_RETRACT_SIGN * m4_speed *
                ((float)coupled_m5_ratio_x1000 /
                 (float)POT_RATIO_SCALE),
                dt, M45_MANUAL_ACCEL_SPS2);
        }
        return;
    }

    int32_t m4_input = (selected == SEL_M4) ? input : 0;
    int32_t m5_input = (selected == SEL_M5) ? input : 0;

    manual_position_tick(MOTOR_M4_INDEX, m4_input, gain_pct, dt,
                         M45_MANUAL_MAX_SPEED_SPS,
                         M45_MANUAL_ACCEL_SPS2);
    manual_position_tick(MOTOR_M5_INDEX, m5_input, gain_pct, dt,
                         M45_MANUAL_MAX_SPEED_SPS,
                         M45_MANUAL_ACCEL_SPS2);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Phase 1: keep every driver disabled while clocks and feedback start. */
  for (int m = 0; m < MOTOR_COUNT; m++)
    HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_SET);

  dwt_delay_init();
#if ACTIVE_APP_MODE == APP_MODE_M23_FEEDER
  uart_printf("BOOT: M2/M3 FEEDER DRIVE\r\n");

  HAL_Delay(100u);
  tmc_write_nodeconf(mot[MOTOR_M2_INDEX].pdn_port,
                     mot[MOTOR_M2_INDEX].pdn_pin,
                     mot[MOTOR_M2_INDEX].tmc_addr, 2);
  tmc_write_nodeconf(mot[MOTOR_M3_INDEX].pdn_port,
                     mot[MOTOR_M3_INDEX].pdn_pin,
                     mot[MOTOR_M3_INDEX].tmc_addr, 2);
  HAL_Delay(10u);

  uint8_t m2_ready = motor_driver_init(MOTOR_M2_INDEX);
  uint8_t m3_ready = motor_driver_init(MOTOR_M3_INDEX);
  uart_printf("M2 DRIVER %s\r\n", m2_ready ? "OK" : "ERROR");
  uart_printf("M3 DRIVER %s\r\n", m3_ready ? "OK" : "ERROR");

  if (!m2_ready || !m3_ready) {
    HAL_GPIO_WritePin(mot[MOTOR_M2_INDEX].en_port,
                      mot[MOTOR_M2_INDEX].en_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(mot[MOTOR_M3_INDEX].en_port,
                      mot[MOTOR_M3_INDEX].en_pin, GPIO_PIN_SET);
    Error_Handler();
  }

  uart_printf("M2/M3 DRIVERS OK: FEEDER START\r\n");
  m23_feeder_run();
#endif

  uart_printf("BOOT: M1 M2 M3 M4 M5 CONTROL\r\n");
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  /* Phase 2: initialize only M1/M2/M3, which are required for homing. */
  HAL_Delay(100u);
  for (int m = 0; m < HOMING_MOTOR_COUNT; m++)
    tmc_write_nodeconf(mot[m].pdn_port, mot[m].pdn_pin, mot[m].tmc_addr, 2);

  HAL_Delay(10u);
  for (int m = 0; m < HOMING_MOTOR_COUNT; m++) {
    uint8_t ready = motor_driver_init((uint8_t)m);
    uart_printf("M%d DRIVER %s\r\n", m + 1,
               ready ? "OK" : "ERROR");
  }

  /* Phase 3: home M1/M2/M3. Any failure inhibits all motion. */
  uart_printf("M1 M2 M3 HOMING START\r\n");
  HomeStatus home_status = home_m1_m2_m3();

  if (home_status != HOME_OK) {
    uart_printf("HOME ERROR: %s (%u)\r\n",
               home_status_name(home_status), (unsigned)home_status);
    for (int m = 0; m < MOTOR_COUNT; m++)
      HAL_GPIO_WritePin(mot[m].en_port, mot[m].en_pin, GPIO_PIN_SET);
    Error_Handler();
  }

  uart_printf("M1 HOME hard=[%ld,%ld] safe=[%ld,%ld]\r\n",
             (long)m12_limit[0].hard_min_count,
             (long)m12_limit[0].hard_max_count,
             (long)m12_limit[0].min_count,
             (long)m12_limit[0].max_count);
  uart_printf("M2 HOME hard=[%ld,%ld] safe=[%ld,%ld]\r\n",
             (long)m12_limit[1].hard_min_count,
             (long)m12_limit[1].hard_max_count,
             (long)m12_limit[1].min_count,
             (long)m12_limit[1].max_count);
  uart_printf("M3 HOME OK lower=ENDSTOP upper=MECHANICAL origin=0\r\n");

  /* Phase 4: initialize M4/M5 only after homing has succeeded. */
  for (int m = MOTOR_M4_INDEX; m < MOTOR_COUNT; m++)
    tmc_write_nodeconf(mot[m].pdn_port, mot[m].pdn_pin, mot[m].tmc_addr, 2);

  HAL_Delay(10u);
  for (int m = MOTOR_M4_INDEX; m < MOTOR_COUNT; m++) {
    uint8_t ready = motor_driver_init((uint8_t)m);
    uart_printf("M%d DRIVER %s\r\n", m + 1,
               ready ? "OK" : "ERROR");
  }

  /* Phase 5: start operator encoders and enter manual control. */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  for (int i = 0; i < INPUT_ENCODER_COUNT; i++) {
    __HAL_TIM_SET_COUNTER(in_enc[i].tim, 0);
    in_enc[i].prev = 0;
  }

  uint32_t initial_pot = pot_read_adc();
  pot_filter_q3 = initial_pot << POT_FILTER_SHIFT;
  coupled_m5_ratio_x1000 = pot_adc_to_ratio_x1000(initial_pot);

  resync_input_encoders();
  uart_printf("CONTROL START: ENC1=M1 ENC2=M2 ENC3=M3 ENC4=%s\r\n",
             enc4_mode_name(enc4_sel));

  uint32_t last_cyc = DWT->CYCCNT;
  Enc4Sel motion_enc4_sel = enc4_sel;
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* One loop iteration updates UI, reads all inputs, then services M12/M3/M45. */
    uint32_t now_cyc = DWT->CYCCNT;
    float dt = (float)(now_cyc - last_cyc) / (float)SystemCoreClock;
    last_cyc = now_cyc;

    int32_t gain_pct = (current_mode == MODE_FAST) ? 300 : 100;
    HAL_GPIO_WritePin(GAIN_LED_GPIO_Port, GAIN_LED_Pin,
                      (current_mode == MODE_FAST) ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);

    pot_ratio_update();

    Enc4Sel selected = enc4_sel;
    if (selected == SEL_M4) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    } else if (selected == SEL_M5) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
                        ((HAL_GetTick() / 250u) & 1u) ?
                        GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    int32_t delta[INPUT_ENCODER_COUNT];
    for (int i = 0; i < INPUT_ENCODER_COUNT; i++)
      delta[i] = in_enc_delta(&in_enc[i]);

    if (selected != motion_enc4_sel) {
      reset_m4_m5_motion_state();
      motion_enc4_sel = selected;
      delta[INPUT_ENCODER_M45_INDEX] = 0;
    }

    m12_manual_tick(MOTOR_M1_INDEX, delta[INPUT_ENCODER_M1_INDEX], dt);
    m12_manual_tick(MOTOR_M2_INDEX, delta[INPUT_ENCODER_M2_INDEX], dt);
    m3_manual_tick(delta[INPUT_ENCODER_M3_INDEX], dt);
    m45_manual_tick(selected, delta[INPUT_ENCODER_M45_INDEX], gain_pct, dt);
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

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = UART_BAUD_RATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
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

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_11,
                          GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GAIN_LED_GPIO_Port, GAIN_LED_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PA6 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GAIN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GAIN_LED_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : M3_ENDSTOP */
  GPIO_InitStruct.Pin = M3_ENDSTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(M3_ENDSTOP_GPIO_Port, &GPIO_InitStruct);

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
    const uint32_t debounce_cycles =
        (SystemCoreClock / 1000u) * BUTTON_DEBOUNCE_MS;

    if (GPIO_Pin == GPIO_PIN_5) {
        static uint32_t last = 0u;
        uint32_t now = DWT->CYCCNT;

        if ((uint32_t)(now - last) >= debounce_cycles) {
            last = now;
            current_mode = (current_mode == MODE_PRECISION)
                ? MODE_FAST : MODE_PRECISION;
        }
    }

    if (GPIO_Pin == GPIO_PIN_4) {
        static uint32_t last = 0u;
        uint32_t now = DWT->CYCCNT;

        if ((uint32_t)(now - last) >= debounce_cycles) {
            last = now;
            if (enc4_sel == SEL_M4)
                enc4_sel = SEL_M5;
            else if (enc4_sel == SEL_M5)
                enc4_sel = SEL_M4_M5_COUPLED;
            else
                enc4_sel = SEL_M4;
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
  __disable_irq();
  while (1) { }
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
