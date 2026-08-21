# 모터 제어 펌웨어 설명서

이 문서는 `main.c`를 처음 보는 사람이 소스 파일을 위에서 아래로 읽을 수 있도록, 실제 코드 등장 순서와 같은 순서로 작성했다. 상세 설명은 이 문서에 두고 `main.c`에는 짧은 구역 제목과 안전상 중요한 주석만 남긴다.

## 입출력 연결

```text
사용자 입력
Encoder 1 / TIM1 -> M1
Encoder 2 / TIM2 -> M2
Encoder 3 / TIM3 -> M3
Encoder 4 / TIM4 -> M4 또는 M5

모터 피드백
TIM5 -> M1 호밍 및 수동 한계 감시, 32비트
TIM8 -> M2 호밍 및 수동 한계 감시, 16비트
```

TIM5와 TIM8은 사용자 조작용이 아니다. M1과 M2의 실제 움직임을 확인하는 피드백 엔코더다.

## 전체 실행 순서

```text
HAL 및 주변장치 초기화
-> 모든 모터 드라이버 비활성화
-> TIM5/TIM8 피드백 엔코더 시작
-> M1/M2/M3 드라이버 초기화
-> M1/M2/M3 동시 호밍
-> M4/M5 드라이버 초기화
-> TIM1~TIM4 사용자 엔코더 시작
-> M12, M3, M45 수동 제어 반복
```

호밍이 실패하면 모든 드라이버를 비활성화하고 수동 제어에 진입하지 않는다.

## 이름과 단위 읽는 법

| 표기 | 의미 |
|---|---|
| `u` | 부호 없는 정수, 예: `3u` |
| `L` | `long` 정수, 예: `200L` |
| `f` | `float`, 예: `0.5f` |
| `_SPS` | microsteps per second |
| `_SPS2` | microsteps per second squared |
| `_STEPS` | motor microsteps |
| `_COUNTS`, `_CNT` | encoder 또는 ADC counts |
| `_MS` | milliseconds |
| `_MA` | milliamperes |
| `_PCT` | percent |
| `_NUM`, `_DEN` | 분수의 분자와 분모 |
| `_X1000` | 실제 비율을 1000배 한 정수 |
| `_DEG10` | 각도를 10배 한 정수, 2539는 253.9도 |

# 1. `main.c` 상단 파라미터

이 장은 `/* USER CODE BEGIN PD */` 안의 매크로를 소스 등장 순서대로 설명한다.

## Motor and input-encoder hardware conversion

### 기본 회전 분해능

- `MICROSTEP=16`: TMC2209가 full step 하나를 16 microstep으로 나눈다.
- `MOTOR_FULL_STEPS_PER_REV=200`: 1.8도 스테퍼 모터의 1회전 full-step 수다.
- `INPUT_ENCODER_PULSES_PER_REV=2048`: 사용자 입력 엔코더의 기본 PPR이다.
- `INPUT_ENCODER_QUADRATURE_FACTOR=4`: A/B상 quadrature x4 decoding이다.

계산 결과는 다음과 같다.

```text
MOTOR_MICROSTEPS_PER_REV
= 200 x 16
= 3200 microsteps/revolution

INPUT_ENCODER_COUNTS_PER_REV
= 2048 x 4
= 8192 counts/revolution
```

`INPUT_COUNT_TO_MICROSTEP_NUM/DEN`은 공통 변환 비율 `3200/8192=25/64`를 나타낸다. 실제 축 이동량에는 축별 `MANUAL_SCALE_NUM/DEN`과 방향 부호가 추가로 곱해진다.

### STEP pulse와 배열 크기

- `STEP_PULSE_CYCLES`: 84 MHz에서 약 1 us에 해당하는 84 CPU cycle이다. `step_pulse()`의 HIGH와 LOW에 각각 사용한다.
- `MOTOR_COUNT=5`: M1~M5 전체 모터 수다.
- `HOMING_MOTOR_COUNT=3`: 먼저 초기화하고 호밍하는 M1~M3 수다.
- `INPUT_ENCODER_COUNT=4`: 사용자 입력 Encoder 1~4 수다.

### 배열 인덱스

`MOTOR_M1_INDEX`~`MOTOR_M5_INDEX`는 `mot[]`와 `motor_ready[]`의 위치다. GPIO 핀 번호나 TMC 주소가 아니다.

`INPUT_ENCODER_M1_INDEX`~`INPUT_ENCODER_M45_INDEX`는 `in_enc[]`와 `delta[]`의 위치다.

## Driver initialization and user interface

- `TMC_INIT_RETRIES=3`: 최초 시도를 포함한 TMC2209 초기화 총 시도 횟수다.
- `TMC_INIT_RETRY_DELAY_MS=100`: 실패 후 다음 시도까지 기다리는 시간이다.
- `BUTTON_DEBOUNCE_MS=200`: 같은 버튼에서 200 ms 이내에 다시 발생한 인터럽트를 무시한다.
- `UART_BAUD_RATE=115200`: USART2 디버그 출력 속도다.
- `UART_TX_TIMEOUT_MS=30`: blocking UART 송신의 최대 대기 시간이다.
- 수동 운전 중 주기 텔레메트리는 출력하지 않는다. M1/M2 limit 또는 M3 물리 endstop이 처음 활성화될 때만 이벤트를 출력한다.

## Manual-control values shared by all axes

- `MANUAL_FIXED_GAIN_PCT=100`: M1, M2, M3의 고정 100% 조작 배율이다. 이 축들은 100%/300% 버튼 영향을 받지 않는다.
- `MANUAL_STOP_SPEED_EPSILON_SPS=0.5`: 부동소수점 속도를 완전 정지로 처리하는 오차 범위다. 실제 최저 운전 속도가 아니다.

## M1/M2 manual control

### 조작 비율

```text
M1: 3200/8192 x 95/26  = 약 1.427 microsteps/input count
M2: 3200/8192 x 171/181 = 약 0.369 microsteps/input count
```

- `M1_MANUAL_SCALE_NUM/DEN=95/26`
- `M2_MANUAL_SCALE_NUM/DEN=171/181`
- M1에는 `dir_sign=-1`, M2에는 `dir_sign=+1`이 적용된다.

### 속도와 가속도

- `M12_MANUAL_MAX_SPEED_SPS=51200`: M1/M2 위치 추종 최대 속도다.
- `M12_MANUAL_ACCEL_SPS2=160000`: M1/M2 속도의 최대 변화율이다.

최대 속도가 너무 낮으면 손잡이를 빠르게 돌렸을 때 `target_pos`가 앞서가고 손을 놓은 뒤에도 모터가 오래 따라갈 수 있다. 가속도를 높이면 반응은 빨라지지만 출발과 정지가 거칠어질 수 있다.

### 수동 안전 범위

- `M1_LIMIT_MARGIN_COUNTS=200`
- `M2_LIMIT_MARGIN_COUNTS=200`
- `M12_LIMIT_RELEASE_COUNTS=50`

margin 단위는 motor step이 아니라 TIM5/TIM8 피드백 count다. release 값은 경계에서 LIMIT가 반복해서 켜지고 꺼지는 것을 막는 hysteresis다.

## M3 manual control

```text
M3: 3200/8192 x 25961/2000
  = 약 5.07 microsteps/input count
```

- `M3_MANUAL_SCALE_NUM/DEN=25961/2000`
- `M3_MANUAL_MAX_SPEED_SPS=51200`
- `M3_MANUAL_ACCEL_SPS2=160000`

M3 값은 M12와 분리되어 있으므로 M3 조작감만 독립적으로 조정할 수 있다.

현재 M3 수동 상한, 하한, 엔드스톱 방향 차단은 구현되지 않았다.

## M3 homing

- `M3_ENDSTOP_ACTIVE_STATE=GPIO_PIN_SET`: 엔드스톱 GPIO가 HIGH일 때 눌린 상태다.
- `M3_HOME_SEEK_DIRECTION=-1`: 엔드스톱 탐색 시 DIR LOW가 되는 논리 방향이다.
- `M3_HOME_SEEK_MAX_STEPS=64000`: 이 거리 안에 스위치를 찾지 못하면 오류다.
- `M3_HOME_SEEK_RATE_SPS=6000`: 탐색 속도다.
- `M3_HOME_BACKOFF_STEPS=27000`: 스위치 감지 후 반대 방향 이동 거리다.
- `M3_HOME_BACKOFF_RATE_SPS=6000`: 후퇴 속도다.
- `M3_HOME_SETTLE_MS=10`: 스위치 감지 또는 후퇴 완료 후 안정화 시간이다.

현재 코드는 27000 step 후퇴한 위치를 0으로 초기화한다. 따라서 엔드스톱 눌림 위치는 이론상 약 -27000이다.

## M4/M5 manual and coupled control

- `M45_MANUAL_MAX_SPEED_SPS=51200`: 단독 위치제어 최대 속도다.
- `M45_MANUAL_ACCEL_SPS2=160000`: 단독·동시 제어 가속도다.
- `COUPLED_M4_BASE_SPEED_SPS=800`: 동시 모드 M4의 100% 기준 속도다. 300% 모드에서는 2400 SPS다.
- `COUPLED_M4_RETRACT_SIGN=+1`
- `COUPLED_M5_RETRACT_SIGN=-1`

동시 모드에서는 Encoder 4의 위치 delta를 사용하지 않고 M4/M5에 연속 속도를 명령한다.

### 가변저항 파라미터

- `POT_ADC_MAX_COUNTS=4095`: 12비트 ADC 최대값
- `POT_ADC_CENTER_COUNTS=2048`: 중앙값
- `POT_ADC_DEADBAND_COUNTS=128`: 중앙 흔들림 무시 범위
- `POT_SAMPLE_PERIOD_MS=10`: 100 Hz 샘플링
- `POT_FILTER_SHIFT=3`: 새 값과 기존 값 차이의 약 1/8씩 반영
- `POT_RATIO_SCALE=1000`: 1000이 실제 1.0배
- `POT_RATIO_MIN_X1000=200`: 최소 0.2배
- `POT_RATIO_CENTER_X1000=1000`: 중앙 1.0배
- `POT_RATIO_MAX_X1000=1500`: 최대 1.5배
- `POT_ADC_REVERSED=0`: ADC가 증가하면 비율도 증가

### 핀 이름 fallback

CubeMX가 `main.h`에 핀 이름을 정의했다면 그 값을 사용한다. 이름이 없을 때는 다음 기본값을 사용한다.

- Gain LED: PB12
- M3 endstop: PB14

과거 프로젝트의 `M1_ENDSTOP_Pin` 이름이 남아 있으면 이를 M3 엔드스톱 alias로 사용한다. M3인데 M1 이름이 등장하는 이유는 과거 CubeMX label 호환 때문이다.

## `USER CODE END PD` 이후의 CubeMX 선언부

상단 파라미터가 끝나면 다음 구간이 나온다. 이곳은 모터를 움직이는 실행 코드가 아니라, CubeMX가 주변장치를 관리하기 위해 만든 **전역 핸들 선언과 초기화 함수의 사전 선언**이다.

### `Private macro`와 `USER CODE BEGIN PM`

```c
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
```

- `PM`은 Private Macro의 약자다.
- 사용자가 이 위치에 전용 매크로를 추가할 수 있도록 CubeMX가 마련한 보호 구간이다.
- 현재는 비어 있으며, 이 코드에 영향을 주는 매크로가 빠진 것이 아니다.
- 앞에서 설명한 모터·호밍 파라미터는 모두 `PD`(Private Defines) 사용자 구간에 들어 있다.

`USER CODE BEGIN/END` 바깥은 CubeMX에서 코드를 다시 생성할 때 변경될 수 있다. 직접 작성할 코드는 가급적 해당 사용자 구간 안에 둔다.

### ADC와 타이머 핸들

```c
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
```

`hadc1`과 `htim1`~`htim8`은 ADC 값이나 엔코더 count 자체를 저장하는 단순 변수가 아니다. STM32 HAL이 각 주변장치의 레지스터 주소, 설정, 상태를 관리하는 **핸들 객체**다.

| 핸들 | 연결된 주변장치 | 이 펌웨어에서의 역할 |
|---|---|---|
| `hadc1` | ADC1 | M4/M5 동시 모드의 가변저항 입력 |
| `htim1` | TIM1 | Encoder 1, M1 사용자 입력 |
| `htim2` | TIM2 | Encoder 2, M2 사용자 입력, 32비트 |
| `htim3` | TIM3 | Encoder 3, M3 사용자 입력 |
| `htim4` | TIM4 | Encoder 4, M4/M5 사용자 입력 |
| `htim5` | TIM5 | M1 피드백 엔코더, 32비트 |
| `htim8` | TIM8 | M2 피드백 엔코더, 16비트 |

예를 들어 `HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL)`은 `htim5`의 주소를 HAL에 넘겨 TIM5 엔코더 기능을 시작한다. 실제 count는 나중에 `__HAL_TIM_GET_COUNTER(&htim5)`로 TIM5 하드웨어 카운터에서 읽는다.

### `USER CODE BEGIN PV`

```c
/* USER CODE BEGIN PV */

/* USER CODE END PV */
```

- `PV`는 Private Variables의 약자다.
- 사용자가 전역 또는 파일 내부 상태 변수를 추가할 수 있는 보호 구간이다.
- 현재는 비어 있다.
- 이 펌웨어의 사용자 상태 변수는 이어지는 `USER CODE BEGIN 0` 안에서 자료형 및 관련 함수와 함께 선언되어 있다.

### 초기화 함수 원형

```c
void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* ... TIM2, TIM3, TIM4, TIM5, TIM8 ... */
```

이 문장들은 함수를 실행하는 코드가 아니라 **함수 원형(prototype)**이다. 컴파일러에 함수 이름, 반환형, 매개변수 형식을 먼저 알려준다.

- `SystemClock_Config()`는 CPU와 주변장치 clock을 84 MHz 구성에 맞춰 설정한다.
- `MX_ADC1_Init()`는 ADC1을 가변저항 입력용으로 설정한다.
- `MX_GPIO_Init()`는 STEP, DIR, EN, 버튼, LED 및 엔드스톱 GPIO를 설정한다.
- `MX_TIM1_Init()`~`MX_TIM8_Init()`는 해당 타이머를 encoder mode로 설정한다.
- `static` 함수는 이 `main.c` 내부에서만 호출할 수 있다.

실제 호출은 뒤의 `main()`에서 이뤄지고, 함수 본문은 파일 하단의 CubeMX 초기화 구간에 있다. 따라서 이 위치에서는 하드웨어가 아직 초기화되거나 시작되지 않는다.

### `USER CODE BEGIN PFP`

```c
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
```

- `PFP`는 Private Function Prototypes의 약자다.
- 사용자가 직접 만든 함수의 원형을 둘 수 있는 보호 구간이다.
- 현재는 비어 있다. 이어지는 `USER CODE BEGIN 0`에서 필요한 사용자 함수를 사용 전에 정의하고 있기 때문이다.

이 선언부 다음의 `/* Private user code */`와 `USER CODE BEGIN 0`부터 모드, UART 출력, 엔코더 계산, 모터 제어처럼 프로젝트 고유의 코드가 시작된다.

# 2. `USER CODE 0` 시작: 모드와 공통 보조 함수

이제부터는 `main.c`의 `/* USER CODE BEGIN 0 */` 아래에 실제로 등장하는 순서대로 설명한다.

## `Mode`와 `current_mode`

```text
MODE_PRECISION -> M4/M5 gain 100%
MODE_FAST      -> M4/M5 gain 300%
```

열거형 값 2와 3이 곧 배율을 의미하지 않는다. 메인 루프가 상태를 보고 100 또는 300을 선택한다.

`current_mode`는 PA5 버튼 ISR에서 쓰고 메인 루프에서 읽으므로 `volatile`이다. 초기 상태는 Precision이다. M1~M3에는 영향을 주지 않는다.

## `uart_printf()`

`vsnprintf()`로 최대 192바이트 문자열을 만든 뒤 USART2로 전송한다.

```text
format + arguments
-> 192-byte buffer
-> HAL_UART_Transmit()
-> USART2 PA2/PA3
-> ST-LINK Virtual COM Port
```

192바이트를 넘는 결과는 잘린다. 현재 송신은 blocking 방식이므로 긴 문자열을 자주 출력하면 루프 시간에 영향을 줄 수 있다.

## DWT와 STEP pulse

### `dwt_delay_init()`

Cortex-M4 DWT cycle counter를 활성화하고 0으로 만든다. STEP timing, 버튼 debounce, 루프 `dt`에 사용한다.

### `dwt_delay_cycles()`

요청한 cycle이 지날 때까지 CPU가 busy-wait한다. unsigned subtraction을 사용하므로 32비트 cycle counter wrap-around를 처리한다.

### `step_pulse()`

```text
STEP HIGH
-> 약 1 us 대기
-> STEP LOW
-> 약 1 us 대기
```

호출 한 번이 1 microstep 명령이다. busy-wait이므로 여러 축이 동시에 빠르게 움직이면 다른 축 처리가 늦어질 수 있다.

# 3. 사용자 입력 엔코더

## `InEnc`

`step_pulse()` 다음에 처음 등장하는 데이터 구조다.

| 필드 | 의미 |
|---|---|
| `tim` | 엔코더 모드 timer handle |
| `is32` | 1이면 32비트, 0이면 16비트 |
| `prev` | 이전 루프에서 읽은 counter |

## `in_enc[]`

```text
in_enc[0] -> TIM1 -> Encoder 1 -> M1, 16비트
in_enc[1] -> TIM2 -> Encoder 2 -> M2, 32비트
in_enc[2] -> TIM3 -> Encoder 3 -> M3, 16비트
in_enc[3] -> TIM4 -> Encoder 4 -> M4/M5, 16비트
```

## `in_enc_delta()`

현재 timer count에서 `prev`를 빼 이번 루프의 이동량을 구한 뒤 현재 값을 새 `prev`로 저장한다.

- 32비트 timer는 `int32_t` 차이를 사용한다.
- 16비트 timer는 하위 16비트 차이를 `int16_t`로 사용한다.
- 이 방식으로 timer overflow를 처리한다.

# 4. 모터 상태와 하드웨어 설정

`InEnc`와 `in_enc_delta()` 다음에 `Motor`가 등장한다. 이 순서 때문에 모터 구조 설명도 여기 배치한다.

## `Motor`

| 필드 | 의미 |
|---|---|
| `step_port/pin` | STEP 출력 |
| `dir_port/pin` | DIR 출력 |
| `en_port/pin` | TMC enable, active-low |
| `pdn_port/pin` | TMC2209 단선 UART |
| `tmc_addr` | 공유 UART 버스의 TMC 주소 |
| `manual_scale_num/den` | 축별 입력 조작 비율 |
| `dir_sign` | 수동 입력 방향 반전 |
| `accum_num` | 1 microstep 미만 입력의 정수 분수 누산값 |
| `target_pos` | 사용자가 요청한 목표 microstep 위치 |
| `current_pos` | 출력한 STEP 펄스 기준 위치 |
| `vel` | 현재 명령 속도, SPS |
| `emit_accum` | 1 STEP 미만 속도 적분 결과 |

`current_pos`는 실제 엔코더 위치가 아니다. motor stall 또는 step loss가 있어도 STEP을 출력하면 증가한다.

## `mot[]` 하드웨어 표

| 모터 | STEP | DIR | EN | TMC UART | 주소 | 수동 비율 | 방향 |
|---|---|---|---|---|---:|---:|---:|
| M1 | PC1 | PC2 | PC0 | PB0 | 0 | 95/26 | -1 |
| M2 | PC4 | PC5 | PC3 | PB0 | 1 | 171/181 | +1 |
| M3 | PC9 | PC10 | PC8 | PB0 | 2 | 25961/2000 | +1 |
| M4 | PC12 | PC13 | PC11 | PB0 | 3 | 1/1 | +1 |
| M5 | PA11 | PD2 | PA12 | PB1 | 0 | 1/1 | +1 |

PB0는 M1~M4가 공유하며 주소로 드라이버를 구분한다. M5는 PB1의 별도 버스를 쓴다.

## `motor_ready[]`

```text
0 -> TMC 설정 미검증, 제어 금지
1 -> TMC 설정 성공, 제어 가능
```

수동 제어는 준비되지 않은 모터에 STEP을 출력하지 않는다.

# 5. Encoder 4 모드와 가변저항

## `Enc4Sel`과 `enc4_sel`

```text
SEL_M4            -> Encoder 4로 M4 위치제어
SEL_M5            -> Encoder 4로 M5 위치제어
SEL_M4_M5_COUPLED -> M4/M5 연속 속도제어
```

`enc4_sel`은 PA4 버튼 ISR에서 변경되고 메인 루프에서 읽으므로 `volatile`이다. 초기값은 M4다.

`enc4_mode_name()`은 상태를 UART 로그 문자열로 바꾸고 알 수 없는 값에는 `UNKNOWN`을 반환한다.

## 가변저항 상태

- `pot_filter_q3`: ADC 값을 8배 확대해 보관하는 필터 내부 상태다.
- `coupled_m5_ratio_x1000`: M5/M4 동시 속도 비율이다. 1000은 1.0배다.

## `pot_read_adc()`

ADC1을 시작하고 최대 2 ms 동안 변환 완료를 기다려 값을 읽은 뒤 ADC를 정지한다. 시작 또는 변환 실패 시 중앙값 2048을 반환하여 1.0배에 가까운 안전한 값으로 돌아간다.

## `pot_adc_to_ratio_x1000()`

```text
ADC 0 부근    -> 200  -> 0.2배
중앙 deadband -> 1000 -> 1.0배
ADC 4095 부근 -> 1500 -> 1.5배
```

중앙 deadband는 정확히 1000을 반환한다. 양쪽 구간은 정수 선형 보간하고 `uint64_t` 중간값으로 overflow를 피한다.

## `pot_ratio_update()`

10 ms 주기가 되지 않았으면 바로 반환한다. 주기가 되면 ADC를 읽고 다음 1차 필터를 적용한다.

```text
filtered += (raw - filtered) / 8
```

필터 결과를 최종 M5/M4 속도 비율로 변환한다.

# 6. TMC 초기화와 공통 모션 함수

## `motor_driver_init()`

```text
motor_ready=0
-> EN HIGH로 disable
-> TMC2209uart(500 mA, 16 microsteps) 최대 3회 시도
-> 성공: motor_ready=1, EN LOW, return 1
-> 실패: disable 유지, return 0
```

`main()`은 read가 필요한 초기화 전에 `tmc_write_nodeconf(..., 2)`로 `SENDDELAY=2`를 먼저 설정한다. 부팅 직후에는 VMOT 안정화를 위해 100 ms를 기다린다.

## `control_tick()`

현재 위치 오차로 목표 속도를 만들고 STEP을 출력하는 공통 위치 추종 함수다.

```text
error = target_pos - current_pos
v_stop = sqrt(2 x acceleration x abs(error))
```

처리 순서:

1. 정지 가능한 속도 계산
2. 축별 최대 속도로 제한
3. 한 루프에서 `acceleration x dt`만큼만 속도 변경
4. 속도 부호로 DIR 결정
5. `vel x dt`를 `emit_accum`에 누적
6. 정수 부분만큼 STEP 출력
7. 목표를 지나지 않도록 STEP 수 제한
8. 출력한 만큼 `current_pos` 갱신

반환값은 없고 `Motor` 상태와 GPIO를 직접 변경한다.

## `coupled_velocity_tick()`

M4/M5 동시 모드의 속도 추종 함수다. 위치 목표가 아니라 `command_vel`을 가속도 제한으로 추종한다.

STEP 출력 후 `target_pos=current_pos`, `accum_num=0`으로 만들어 위치 목표가 뒤에 쌓이지 않게 한다.

## `reset_m4_m5_motion_state()`

Encoder 4 모드가 바뀔 때 M4/M5의 목표, 속도, 입력 누산, 출력 누산을 제거하고 STEP 핀을 LOW로 만든다. 이전 모드 이동이 새 모드에서 이어지는 것을 막는다.

# 7. M1/M2 호밍과 수동 한계

이 구역은 소스의 `/* Encoder homing for M1/M2 */`부터 시작한다. 호밍 로직은 별도 검증된 기존 코드를 유지한다.

## M1/M2 호밍 파라미터

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `HOME_NORMAL_CURRENT_MA` | 500 mA | 호밍 후 정상 전류 |
| `HOME_M1_TOUCH_CURRENT_MA` | 175 mA | M1 끝 접촉 전류 |
| `HOME_M2_TOUCH_CURRENT_MA` | 300 mA | M2 끝 접촉 전류 |
| `HOME_APPROACH_V` | 800 SPS | 양 끝 탐색 속도 |
| `HOME_RETURN_V` | 800 SPS | 최종 위치 복귀 속도 |
| `HOME_WIN` | 16 steps | 피드백 판정 window 크기 |
| `HOME_ARM_WINDOWS` | 3 | 시작 직후 무시할 window 수 |
| `HOME_RNUM/RDEN` | 25/64 | STEP과 예상 feedback count 변환 |
| `HOME_TRIP_LAG` | 80 counts | baseline보다 이만큼 lag가 증가하면 stall 후보 |
| `HOME_CONFIRM_WINDOWS` | 2 | 연속 stall 확인 횟수 |
| `HOME_SECOND_ARM_COUNTS` | 6000 counts | 두 번째 끝 검출을 허용하기 전 최소 이동량 |
| `HOME_M1_LIMIT_STEPS` | 12000 steps | M1 끝 미검출 중단 거리 |
| `HOME_M2_LIMIT_STEPS` | 12288 steps | M2 끝 미검출 중단 거리 |
| `HOME_M1_SEAT_STEPS` | 32 steps | M1 검출 후 추가 안착 |
| `HOME_M2_SEAT_STEPS` | 0 steps | M2 추가 안착 없음 |
| `HOME_START_EPS_CNT` | 4 counts | 시작부터 끝에 닿은 상태 허용 오차 |
| `HOME_RETURN_TOL_CNT` | 2 counts | 최종 위치 도착 허용 오차 |
| `HOME_RETURN_MARGIN` | 64 steps | 복귀 실패 판정 전 STEP 여유 |
| `HOME_MIN_SPAN_CNT` | 12000 counts | 정상 전체 범위 최솟값 |
| `HOME_MAX_SPAN_CNT` | 30000 counts | 정상 전체 범위 최댓값 |

최종 위치는 측정한 전체 span에 다음 비율을 적용한다.

```text
M1: 139.0 / 253.9
M2: 138.3 / 245.2
```

따라서 M1/M2 호밍 후 최종 정지 지점을 조정할 때는 `HOME_M1_RETURN2_DEG10`과 `HOME_M2_RETURN2_DEG10`을 중심으로 검토한다.

## `HomeAxis`와 `home_ax[]`

| 필드 | 의미 |
|---|---|
| `mi` | `mot[]` 모터 인덱스 |
| `fenc` | 피드백 timer |
| `is32` | timer 폭 |
| `esign` | feedback 논리 방향 보정 |
| `first_dir` | 첫 번째 끝 탐색 방향 |
| `span_deg10` | 전체 각도 x10 |
| `return2_deg10` | 두 번째 끝에서 복귀 각도 x10 |
| `limit_steps` | 끝 미검출 중단 STEP |
| `touch_current_ma` | 접촉 전류 |
| `seat_steps` | 검출 후 추가 안착 STEP |

M1은 TIM5 32비트와 `esign=+1`, M2는 TIM8 16비트와 `esign=-1`을 사용한다.

## `HomeStatus`와 `home_status_name()`

| 상태 | 의미 |
|---|---|
| `HOME_OK` | 정상 완료 |
| `HOME_NO_HARDSTOP` | 최대 거리 안에서 끝 또는 스위치 미검출 |
| `HOME_ENCODER_FAULT` | feedback encoder 방향 또는 count 이상 |
| `HOME_SPAN_FAULT` | 측정 전체 범위 이상 |
| `HOME_RETURN_FAULT` | 최종 위치 복귀 실패 |
| `HOME_DRIVER_FAULT` | TMC 준비 또는 전류 설정 실패 |
| `HOME_SWITCH_NOT_RELEASED` | M3 후퇴 후 스위치 미해제 |

`home_status_name()`은 UART 출력용 문자열로 변환한다.

## 임시 및 영구 상태 구조체

- `HomeStop`: 검출한 끝의 feedback position과 출력 STEP 수
- `HomeSeekState`: window, 진행 count, lag, arm, confirm 등 한 번의 탐색 임시 상태
- `ManualLimit`: 호밍 후 유지되는 hard/safe 범위와 현재 LIMIT 상태
- `m12_limit[0]`: M1, `m12_limit[1]`: M2

## 호밍 및 한계 보조 함수

### `reset_motor_motion_state()`

현재 위치, 목표 위치, 두 누산값과 속도를 모두 0으로 만든다. 새 호밍 좌표를 확정할 때 사용한다.

### `home_encoder_delta()`

HomeAxis의 16/32비트 설정에 맞게 feedback count 차이를 계산하고 wrap-around를 처리한다.

### `home_position_from()`

현재 feedback count에서 기준 origin을 빼고 `esign`을 곱해 논리 위치를 반환한다.

### `stop_motor_motion()`

`target_pos=current_pos`로 만들고 속도 및 누산값을 0으로 만든다. STEP 핀도 LOW로 만든다.

### `apply_m12_limit()`

1. TIM5/TIM8 현재 위치 계산
2. hysteresis를 적용해 MIN/MAX 상태 갱신
3. 바깥 방향 새 입력을 0으로 변경
4. 남아 있는 목표 또는 속도가 바깥 방향이면 `stop_motor_motion()` 호출
5. MIN/MAX가 새로 활성화된 경우에만 UART 이벤트를 한 번 출력

최소 한계에서는 더 작은 논리 위치로 가는 입력만 차단하고 반대 방향 복귀는 허용한다. 최대 한계도 같은 원리다.

### 수동 limit 이벤트 UART

주기 상태 출력 함수는 사용하지 않는다. `apply_m12_limit()`은 M1/M2의 MIN/MAX가 비활성에서 활성으로 바뀔 때만 이벤트를 출력하고, `m3_manual_tick()`은 M3 물리 endstop이 새로 활성화될 때만 이벤트를 출력한다. 해제 이벤트와 정상 수동 운전 상태는 출력하지 않는다.

### `home_set_direction()`

논리 방향에 `dir_sign`을 곱해 M1/M2 DIR GPIO를 정하고 약 10 us 기다린다.

### `home_set_axis_currents()`

touch mode에서는 M1 175 mA, M2 300 mA를 설정하고 normal mode에서는 두 축을 500 mA로 복구한다. 실패하면 `HOME_DRIVER_FAULT`다.

### `home_seek_pair()`

M1과 M2를 한 STEP씩 번갈아 구동하면서 두 끝을 동시에 찾는다. `HOME_WIN`마다 다음을 계산한다.

```text
expected_counts = moved_steps x 64 / 25
lag = expected_counts - actual_advanced_counts
lag_delta = lag - detection_start_lag
```

`lag_delta>=HOME_TRIP_LAG`가 연속 확인되면 끝으로 판정한다. 대기 중에는 `m3_home_service()`를 계속 호출하여 M3도 동시에 호밍한다.

### `home_move_pair()`

두 번째 끝에서 계산된 최종 feedback 위치까지 M1/M2를 함께 복귀시킨다. STEP 제한 안에 목표에 도달하지 못하면 `HOME_RETURN_FAULT`다. 이 함수도 대기 중 M3를 service한다.

### `home_m1_m2_m3()`

```text
M1/M2 접촉 전류 설정
-> M3 상태 머신 시작
-> M1/M2 첫 번째 끝 탐색
-> M1/M2 두 번째 끝 탐색
-> span 검증 및 최종 위치 계산
-> 정상 전류 복구
-> M1/M2 최종 위치 복귀
-> M1/M2 hard/safe 한계 생성
-> 모션 상태와 feedback counter 0 초기화
-> M3 완료 대기
```

호밍 후 범위는 다음처럼 계산한다.

```text
hard_min = home_counts - measured_span
hard_max = home_counts
safe_min = hard_min + margin
safe_max = hard_max - margin
```

# 8. M3 호밍 상태 머신

M1/M2 호밍 함수 다음에 M3 전용 상태 머신이 등장한다.

## `m3_endstop_active()`

M3 엔드스톱 GPIO가 active state와 같은지 반환한다.

## `m3_home_set_direction()`

방향이 양수면 DIR HIGH, 그렇지 않으면 DIR LOW다. 이 함수는 `Motor.dir_sign`을 곱하지 않는다. DIR 변경 후 약 10 us 기다린다.

## `M3HomeState`

```text
IDLE
-> SEEK
-> SEEK_SETTLE
-> BACKOFF
-> RELEASE_SETTLE
-> DONE

오류 -> FAILED
```

## `M3HomeContext`와 `m3_home`

현재 상태, 오류, 마지막 STEP cycle, settle 시작 ms, seek STEP 수, backoff STEP 수를 저장한다. 전역 상태이므로 M1/M2 blocking 호밍 중 여러 번 service할 수 있다.

## `m3_home_fail()`

STEP 핀을 LOW로 만들고 상태와 오류를 FAILED로 저장한다.

## `m3_home_begin()`

드라이버 준비와 초기 상태를 확인한다. 시작부터 엔드스톱이 눌렸으면 곧바로 settle 상태로 가고, 아니면 seek 방향으로 탐색을 시작한다.

## `m3_home_service()`

한 번 호출할 때 현재 상태에서 필요한 작업을 조금씩 진행한다.

- `SEEK`: 속도 간격마다 STEP 1개 출력, 스위치와 최대 거리 확인
- `SEEK_SETTLE`: 10 ms 후 반대 방향 설정
- `BACKOFF`: 27000 STEP까지 후퇴
- `RELEASE_SETTLE`: 10 ms 후 스위치 해제 확인
- 완료: 모션 상태 0 초기화 후 DONE

함수 한 번으로 전체 호밍을 끝내지 않으므로 M1/M2 호밍의 대기 구간에서 계속 호출해야 한다.

## `m3_home_finished()`와 `m3_home_abort()`

`m3_home_finished()`는 DONE일 때만 1이다. `m3_home_abort()`는 STEP 핀을 LOW로 만들지만 상태 전체를 초기화하지는 않는다.

## `resync_input_encoders()`

호밍이 끝난 시점의 TIM1~TIM4 count를 `prev`에 복사한다. 호밍 중 사용자가 돌린 손잡이 이동이 수동 제어 시작 직후 큰 delta로 적용되는 것을 막는다.

# 9. 호밍 이후 수동 제어 함수

이 함수들은 `/* Post-homing manual control */` 구역에 있으며 메인 루프가 직접 호출한다.

## `manual_position_tick()`

드라이버 준비를 확인하고 입력 delta를 정수 분수 누산기로 목표 위치에 반영한 뒤 `control_tick()`을 호출한다.

```text
input delta
x 3200/8192
x axis manual scale
x gain
x direction sign
-> target_pos
```

1 microstep 미만 입력은 `accum_num`에 보존하므로 작은 입력이 사라지지 않는다.

## `m12_manual_tick()`

M1/M2 축인지 확인하고 `apply_m12_limit()`을 먼저 적용한다. 이후 고정 100% gain과 M12 속도·가속도로 위치를 추종한다.

## `m3_manual_tick()`

M3에 고정 100% gain과 M3 전용 속도·가속도를 적용한다. 현재 수동 한계와 엔드스톱 검사는 없다.

## `m45_manual_tick()`

- coupled이면 M4/M5에 연속 속도를 명령한다.
- 단독이면 선택된 모터에만 Encoder 4 delta를 전달한다.
- M4/M5에는 현재 100% 또는 300% gain이 적용된다.

# 10. `main()` 실행 단계

수동 함수 정의가 끝난 다음 실제 `main()`이 등장한다.

## Cube/HAL 기본 초기화

```text
HAL_Init()
-> SystemClock_Config()
-> MX_GPIO_Init()
-> MX_ADC1_Init()
-> MX_TIM1_Init() ... MX_TIM8_Init()
```

## Phase 1: 안전한 초기 상태

모든 EN을 HIGH로 만들어 드라이버를 비활성화하고 DWT와 TIM5/TIM8 피드백을 시작한다.

## Phase 2: M1/M2/M3 초기화

VMOT 안정화를 기다리고 SENDDELAY를 설정한 뒤 호밍에 필요한 세 드라이버만 초기화한다.

## Phase 3: M1/M2/M3 호밍

`home_m1_m2_m3()`를 실행한다. 실패하면 모든 EN을 비활성화하고 `Error_Handler()`로 들어간다.

## Phase 4: M4/M5 초기화

호밍 성공 후에만 M4와 M5를 초기화한다.

## Phase 5: 사용자 입력 시작

TIM1~TIM4를 시작하고 count를 0으로 맞춘다. 가변저항 초기값을 읽고 입력 엔코더를 재동기화한다.

## 메인 루프

```text
dt 계산
-> gain과 LED 갱신
-> 가변저항 갱신
-> Encoder 1~4 delta 읽기
-> 모드 변경 시 M4/M5 상태 초기화
-> m12_manual_tick(M1)
-> m12_manual_tick(M2)
-> m3_manual_tick()
-> m45_manual_tick()
```

`dt`는 DWT cycle 차이를 `SystemCoreClock`으로 나눈 값이다. 제어 주기는 고정되어 있지 않고 STEP 처리량에 따라 달라진다. 수동 상태의 주기 UART 출력은 제거되어 있다.

### Encoder 4 모드 변경 처리

`motion_enc4_sel`은 마지막으로 모션에 적용한 모드다. ISR이 `enc4_sel`을 변경하면 메인 루프가 차이를 발견하여 M4/M5 상태를 초기화하고 그 루프의 Encoder 4 delta를 0으로 만든다.

### LED 상태

| 상태 | 표시 |
|---|---|
| Precision | Gain LED OFF |
| Fast 300% | Gain LED ON |
| M4 선택 | PA6 ON |
| M5 선택 | PA6 OFF |
| coupled | PA6가 250 ms마다 반전 |

# 11. CubeMX 및 주변장치 초기화 함수

`main()` 다음에는 생성된 주변장치 설정 함수가 등장한다.

## `SystemClock_Config()`

HSI 16 MHz와 PLL을 사용해 84 MHz SYSCLK를 만든다.

```text
PLLM=16, PLLN=336, PLLP=4
```

DWT, STEP pulse, 버튼 debounce, 호밍 속도가 시스템 클록에 의존한다.

## `MX_ADC1_Init()`

- PA7 / ADC1 Channel 7
- 12비트, right alignment
- software trigger, single conversion
- DMA 및 continuous mode 사용 안 함
- sampling time 144 cycles

가변저항을 읽을 때 `pot_read_adc()`가 매번 변환을 시작한다.

## Encoder timer 초기화

다음 함수들은 모두 `TIM_ENCODERMODE_TI12`, rising polarity, prescaler 1, digital filter 0을 사용한다.

| 함수 | 용도 | 폭 |
|---|---|---:|
| `MX_TIM1_Init()` | Encoder 1, M1 입력 | 16비트 |
| `MX_TIM2_Init()` | Encoder 2, M2 입력 | 32비트 |
| `MX_TIM3_Init()` | Encoder 3, M3 입력 | 16비트 |
| `MX_TIM4_Init()` | Encoder 4, M4/M5 입력 | 16비트 |
| `MX_TIM5_Init()` | M1 피드백 | 32비트 |
| `MX_TIM8_Init()` | M2 피드백 | 16비트 |

필터가 0이므로 노이즈가 있으면 배선과 timer filter를 검토해야 한다. 실제 Alternate Function 핀 설정은 보통 `stm32f4xx_hal_msp.c`의 `HAL_TIM_Encoder_MspInit()`에 있다.

## `MX_GPIO_Init()`

- M1~M5 STEP/DIR/EN 출력
- PB0/PB1 TMC 단선 UART open-drain
- PA4/PA5 버튼 rising-edge EXTI, pulldown
- PA6 모드 LED
- PB12 gain LED fallback
- PB14 M3 endstop fallback
- PD2 M5 DIR

# 12. 버튼 콜백과 오류 처리

주변장치 초기화 함수 다음에 `USER CODE 4`의 버튼 콜백이 등장한다.

## `HAL_GPIO_EXTI_Callback()`

각 버튼은 별도의 마지막 cycle 값을 갖고 DWT로 200 ms debounce를 적용한다.

### PA5

```text
MODE_PRECISION <-> MODE_FAST
```

### PA4

```text
SEL_M4 -> SEL_M5 -> SEL_M4_M5_COUPLED -> SEL_M4
```

ISR에서는 상태만 변경한다. 모터 상태 초기화와 LED 갱신은 메인 루프가 한다. ISR 안에서 UART 문자열을 직접 보내지 않아야 한다.

## `Error_Handler()`

인터럽트를 비활성화하고 무한 루프에 머문다. 호밍 실패 경로는 호출 전에 모든 모터 EN을 비활성화한다.

HAL 초기화 도중 직접 호출되는 다른 오류 경로에서는 호출 시점에 따라 EN 상태가 다를 수 있으므로 향후 전체 fault 정책에서 검토해야 한다.

## `assert_failed()`

`USE_FULL_ASSERT`가 활성화됐을 때만 포함된다. 현재는 `file`과 `line`을 사용하지 않는다.

# 13. UART 디버깅

현재 디버깅은 USART2와 ST-LINK Virtual COM Port를 사용한다.

```text
USART2: PA2 TX, PA3 RX
Baud rate: 115200
Data format: 8-N-1
Flow control: None
```

현재 `uart_printf()`는 `HAL_UART_Transmit()` 기반 blocking 송신이다. 다음 원칙을 적용한다.

- ISR과 모션 루프에서 긴 blocking 송신 금지
- 수동 운전 중 주기 텔레메트리 출력 금지
- M1/M2 limit 및 M3 물리 endstop 활성 이벤트만 1회 출력
- limit 이벤트는 모션을 먼저 정지한 뒤 출력
- 가능하면 DMA 또는 interrupt 기반 queue 사용

# 14. 남은 작업

1. M1/M2 호밍 후 최종 정지 위치 확정
2. M3 엔드스톱 눌림 높이를 기준으로 하한 좌표 결정
3. M3 상한 결정
4. M3 속도와 가속도 조정
5. 빠른 조작 시 목표 위치 누적량 측정
6. 필요 시 UART 이벤트 출력을 interrupt 또는 DMA 기반 queue로 변경

## 실제 장비 조정 주의사항

- 한 번에 한 파라미터 그룹만 변경한다.
- 호밍 전류, 최대 거리, 최종 위치를 동시에 크게 바꾸지 않는다.
- M3 하한 좌표 확정 전에는 엔드스톱 부근 고속 시험을 피한다.
- 한계에서는 바깥 방향만 차단하고 반대 방향 복귀는 허용한다.
- `current_pos`만으로 실제 stall 또는 step loss를 검출할 수 없다.
- TMC2209 진단 read는 bit-bang 통신과 interrupt 차단 때문에 운전 중 모션을 방해할 수 있다.
