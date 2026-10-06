# M2/M3 광센서 피더 모드 — 2026-10-06 최종 설정

기반: `0918` / `52190dbdcf8585623584d45764ad5ea3ea679025`.
현재 기본 실행 모드는 `APP_MODE_M23_FEEDER`이며 M2/M3만 활성화한다.

- 센서1: Omron EE-SX954-W 흰색 OUT2 → PB10 → M2.
- 센서2: Omron EE-SX954-W 흰색 OUT2 → PB13 → M3.
- 갈색 +5 V, 파란색 공통 GND, 검정색 OUT1 미연결. 각 출력에 별도의 1 kΩ 풀업 → 보드 3.3 V. GPIO INPUT/NOPULL.
- 각 축은 독립적으로 안정된 BLOCKED→CLEAR를 5회 세고 추가 3200 STEP 후 멈춘다. 2 ms 필터 적용. 시작부터 BLOCKED이면 첫 CLEAR는 제외.
- 두 축 완료 후 ENC4/TIM4로 피더를 앞뒤로 수동 제어한다. 반전 입력은 이전 대기 스텝과 변환 나머지를 지운다.
- 호밍 최대 이동량과 호밍 이후 이동 범위 제한을 제거했다. 호밍 시간 제한도 없다. 센서 감지가 누락되면 계속 이동한다.
- M2 feeder direction=+1, M3=-1. 양의 피더 이동에서 M2 DIR HIGH, M3 DIR LOW.
- 호밍 속도 1200 STEP/s, 수동 최고속도 1700 STEP/s, 시작속도 200 STEP/s, 가속도 3000 STEP/s².
- 200 full steps × 16 microsteps = 3200 STEP/모터축 1회전. 추가 3200 STEP은 기어 출력축/피더 길이가 아닌 모터축 1회전이다.
- ENC4는 2048 PPR × 4 = 8192 counts/회전이며 입력 변환은 25/64이다.
- USART2 115200, 8N1. M2 1000 mA, M3 800 mA. 드라이버 초기화 실패 시 두 축 비활성화.

`Core/Src/main.c`가 최종 소스이며 `motorAndEncoder.ioc`에는 PB10/PB13 전용 센서 입력이 반영되어 있다. 기존 엔코더 포트와 차동 수신기는 유지한다. 빌드 결과와 하드웨어 확인 범위 및 다음 채팅 인계는 [작업 기록](docs/FEEDER_WORKLOG_2026-10-06.md)을 참고한다.

기존 `Debug/` 파일은 기반 브랜치가 추적하던 과거 산출물이다. 현재 소스와 일치하는 실행 파일로 간주하지 말고 STM32CubeIDE에서 다시 빌드한다. 아래 문서는 기반 통합 제어 모드 설명이며 현재 기본 피더 실행과 구분해서 읽는다.

---

# ERCPver3 5축 모터 제어 펌웨어

STM32F446RE와 TMC2209 드라이버 5개를 사용하는 모터 제어 펌웨어다. 부팅 시 M1·M2·M3를 호밍하고, 호밍이 성공하면 네 개의 사용자 엔코더로 M1~M5를 조작한다.

이 README는 프로젝트를 처음 접한 사람이 다음 내용을 빠르게 파악하도록 작성했다.

- 장비가 어떤 순서로 시작하고 움직이는지
- 긴 `main.c`를 어떤 순서로 읽으면 되는지
- 속도, 방향, 회전비, 호밍 위치를 바꾸려면 무엇을 검색해야 하는지
- 숫자 조정과 알고리즘 변경을 어떻게 구분해야 하는지
- 코드를 바꾼 뒤 무엇을 확인해야 하는지

> 문서 기준: 2026-09-18 통합제어 작업본
>
> 현재 상태: M3 호밍 및 수동 반전 로직 보완 완료, 실제 장비에서 M3 최종 상·하한과 전원 상태 확인 필요

## 처음 보는 사람을 위한 빠른 안내

프로젝트를 처음 열었다면 다음 순서로 보면 된다.

1. 이 README의 **전체 동작 흐름**을 읽는다.
2. `Core/Src/main.c` 상단의 `#define` 구역에서 현재 설정값을 확인한다.
3. `mot[]`에서 모터별 STEP, DIR, EN, TMC UART 핀과 주소를 확인한다.
4. `main()`에서 초기화와 호밍 진입 순서를 확인한다.
5. 수정하려는 축에 따라 `m12_manual_tick()`, `m3_manual_tick()`, `m45_manual_tick()`을 읽는다.
6. 세부 설명이 필요하면 `docs/FIRMWARE_GUIDE.md`를 읽는다.
7. 원하는 동작의 수정 위치를 빠르게 찾으려면 `docs/CONTROL_TUNING_GUIDE.md`를 읽는다.

소스의 줄 번호는 수정할 때마다 달라진다. 이 문서에 적힌 **매크로명과 함수명을 STM32CubeIDE에서 `Ctrl+F`로 검색**하는 것이 가장 빠르다.

## 전체 시스템 구성

```text
사용자 입력 엔코더
  Encoder 1 / TIM1 ──> M1
  Encoder 2 / TIM2 ──> M2
  Encoder 3 / TIM3 ──> M3
  Encoder 4 / TIM4 ──> M4 또는 M5

호밍·한계 피드백
  TIM5 ──> M1 피드백 엔코더
  TIM8 ──> M2 피드백 엔코더
  PB14 ──> M3 물리 엔드스톱

모터 출력
  STM32F446RE ── STEP/DIR/EN ──> TMC2209 x5 ──> M1~M5

설정·진단
  USART2 PA2/PA3 ──> ST-LINK Virtual COM Port
  PA7 ADC ──> M4/M5 동시모드 속도 비율
```

## 하드웨어와 개발 환경

| 항목 | 설정 |
|---|---|
| MCU | STM32F446RETx |
| 보드 | NUCLEO-F446RE 기반 |
| 시스템 클록 | 84 MHz |
| STM32CubeMX | 6.15.0 |
| STM32Cube firmware | STM32Cube FW_F4 V1.28.3 |
| 모터 드라이버 | TMC2209 x5 |
| 마이크로스텝 | 16 |
| 입력 엔코더 | 2048 PPR, quadrature x4 |
| 디버그 UART | USART2, PA2/PA3, 115200 8-N-1 |

## 두 가지 애플리케이션 모드

`main.c`에는 통합제어와 피더 시험 모드가 함께 들어 있다.

```c
#define APP_MODE_INTEGRATED_CONTROL 0u
#define APP_MODE_M23_FEEDER         1u

#define ACTIVE_APP_MODE APP_MODE_INTEGRATED_CONTROL
```

### 통합제어 모드

`APP_MODE_INTEGRATED_CONTROL`은 실제 M1~M5 제어에 사용하는 모드다.

```text
HAL 및 GPIO·타이머·UART·ADC 초기화
-> 모든 드라이버 비활성화
-> M1/M2 피드백 엔코더 시작
-> M1/M2/M3 TMC2209 초기화
-> M1/M2/M3 호밍
-> 호밍 성공 확인
-> M4/M5 TMC2209 초기화
-> 사용자 입력 엔코더 시작
-> M1~M5 수동제어 반복
```

필수 드라이버 초기화나 호밍이 실패하면 모든 드라이버를 비활성화하고 수동제어에 진입하지 않는다.

### M2/M3 피더 시험 모드

`APP_MODE_M23_FEEDER`는 임시 피더의 두 모터를 M2/M3 출력에 연결해 시험하기 위한 독립 모드다.

- M2와 M3에 같은 STEP 펄스를 동시에 출력
- 두 모터의 DIR은 서로 반대
- 속도 단계: `1200, 1200, 1700, 1700, 1200, 1200, 1700 SPS`
- 각 단계는 10초
- 7단계가 끝나면 처음부터 반복
- 통합 호밍과 수동제어는 실행하지 않음

모드를 바꾸려면 `ACTIVE_APP_MODE`만 변경한다. 다른 조건부 컴파일 구역을 직접 지울 필요는 없다.

## 모터와 엔코더 연결

### 모터 출력과 TMC 주소

| 모터 | STEP | DIR | EN | TMC UART | 주소 | 운전 전류 |
|---|---|---|---|---|---:|---:|
| M1 | PC1 | PC2 | PC0 | PB0 | 0 | 1000 mA |
| M2 | PC4 | PC5 | PC3 | PB0 | 1 | 1000 mA |
| M3 | PC9 | PC10 | PC8 | PB0 | 2 | 800 mA |
| M4 | PC12 | PC13 | PC11 | PB0 | 3 | 1000 mA |
| M5 | PA11 | PD2 | PA12 | PB1 | 0 | 1000 mA |

PB0는 M1~M4가 공유하는 단선 TMC2209 UART이고, PB1은 M5 전용 UART다. 핀이나 주소를 바꿀 때는 `mot[]` 초기화 테이블을 수정한다.

### 엔코더 연결

| 구분 | 타이머 | 역할 | 타이머 폭 |
|---|---|---|---:|
| Encoder 1 | TIM1 | M1 사용자 조작 | 16비트 |
| Encoder 2 | TIM2 | M2 사용자 조작 | 32비트 |
| Encoder 3 | TIM3 | M3 사용자 조작 | 16비트 |
| Encoder 4 | TIM4 | M4/M5 사용자 조작 | 16비트 |
| Encoder 5 | TIM5 | M1 호밍·한계 피드백 | 32비트 |
| Encoder 6 | TIM8 | M2 호밍·한계 피드백 | 16비트 |

Encoder 5와 Encoder 6은 손잡이 입력이 아니다. M1과 M2의 실제 기구 움직임을 확인한다.

### 버튼과 보조 입력

| 핀 | 역할 |
|---|---|
| PA4 | M4 → M5 → M4/M5 동시모드 선택 |
| PA5 | 100%/300% gain 선택 |
| PA6 | Encoder 4 선택 상태 표시 |
| PA7 | 동시모드 M5 속도 비율 가변저항 ADC |
| PB12 | gain 상태 LED |
| PB14 | M3 엔드스톱 입력 |

M3 엔드스톱은 pull-down 입력이며 `GPIO_PIN_SET`, 즉 HIGH일 때 활성이다. 실제 배선도 엔드스톱 동작 시 PB14에 3.3V가 들어오도록 구성해야 한다.

## 호밍 동작

### M1/M2 호밍

M1과 M2는 TIM5/TIM8 피드백 엔코더를 사용해 기구 양쪽 끝을 찾는다.

```text
첫 번째 hard stop 탐색
-> 반대쪽 hard stop 탐색
-> 피드백 encoder 전체 span 측정
-> 기준 각도로 최종 복귀량 계산
-> 최종 위치로 복귀
-> 현재 위치를 0으로 설정
-> 수동 운전 안전 범위 생성
```

호밍 접촉 전류는 M1 175mA, M2 300mA다. 호밍 후 각각의 정상 운전 전류로 복원한다.

M1/M2 호밍의 탐색·판정·정지 알고리즘은 기구와 함께 검증된 영역이다. 단순 조정이라면 `HOME_...` 숫자만 변경하고, `home_seek_pair()`와 `home_move_pair()`의 흐름은 충분히 이해한 뒤 수정한다.

### M3 호밍

M3는 PB14 물리 엔드스톱을 사용한다.

```text
엔드스톱 방향으로 6000 SPS 탐색
-> 엔드스톱 감지 즉시 STEP 정지
-> 100 ms 대기
-> DIR 반전
-> 300 SPS로 백오프 시작
-> 첫 1500 STEP 동안 6000 SPS까지 가속
-> 엔드스톱 해제를 8 STEP 연속 확인
-> 총 22000 STEP 백오프
-> 현재 위치를 0으로 설정
```

3,000 STEP 안에 엔드스톱이 해제되지 않으면 `HOME_SWITCH_NOT_RELEASED` 오류로 정지한다. 탐색 중 128,000 STEP 안에 엔드스톱을 찾지 못하면 `HOME_NO_HARDSTOP` 오류가 발생한다.

현재 128,000 STEP은 **엔드스톱 탐색 실패를 판단하는 한도**다. 수동제어의 최종 상한을 뜻하지 않는다. M3 실제 상·하한을 측정한 뒤 수동 가동범위를 별도로 확정해야 한다.

## 수동제어 동작

### M1/M2

- Encoder 1로 M1 제어
- Encoder 2로 M2 제어
- 입력 엔코더 변화량을 목표 STEP 위치로 누적
- 피드백 엔코더로 양방향 수동 한계 감시
- PA5의 100%/300% 모드와 관계없이 고정 100% gain

### M3

- Encoder 3으로 제어
- 하단 물리 엔드스톱이 활성화되면 하단 방향만 차단
- 엔드스톱 반대 방향 복귀 허용
- 소프트웨어 상한은 아직 설정하지 않음
- 이동 중 1초마다 현재 위치와 목표 위치를 UART로 출력

한 방향 입력이 많이 누적된 상태에서 사용자가 반대 방향으로 조작하면, 기존 방향의 `target_pos`, `accum_num`, `emit_accum`을 지운다. 현재 속도는 강제로 뒤집지 않고 설정된 가속도로 감속한 뒤 반전한다. 이 처리는 조작 반응을 빠르게 하면서 갑작스러운 STEP 주파수 반전으로 발생할 수 있는 탈조를 줄인다.

### M4/M5

Encoder 4 선택 버튼을 누를 때마다 다음 순서로 바뀐다.

```text
M4 단독 -> M5 단독 -> M4/M5 동시 -> M4 단독
```

- 단독모드: Encoder 4의 위치 입력을 선택된 모터가 추종
- 단독모드: PA5의 100%/300% gain 적용
- 동시모드: M4는 고정 800 SPS
- 동시모드: gain 버튼을 적용하지 않고 gain 1로 고정
- 동시모드: M5는 가변저항에 따라 M4 속도의 0.2배~1.5배

## 현재 주요 제어값

### 수동 조작 비율

입력 엔코더 1 count가 모터 STEP으로 변환될 때 공통 변환값 `3200/8192=25/64`와 아래 축별 비율이 함께 적용된다.

| 모터 | 축별 비율 | 방향 부호 |
|---|---:|---:|
| M1 | 95/26 | -1 |
| M2 | 171/181 | +1 |
| M3 | 25961/1150 | +1 |
| M4 | 1/1 | +1 |
| M5 | 1/1 | +1 |

### 수동 속도와 가속도

| 제어 그룹 | 최고속도 | 가속도 |
|---|---:|---:|
| M1/M2 | 51,200 SPS | 160,000 SPS² |
| M3 | 120,000 SPS | 80,000 SPS² |
| M4/M5 단독 | 51,200 SPS | 160,000 SPS² |
| M4/M5 동시 | M4 기준 800 SPS | 160,000 SPS² |

## `main.c`를 읽는 순서

`main.c`에는 사용자 로직과 CubeMX 생성 코드가 함께 있다. 처음부터 끝까지 한 줄씩 읽기보다 다음 구역을 순서대로 보는 편이 이해하기 쉽다.

### 1. 상단 `#define`: 조정 가능한 값

`/* USER CODE BEGIN PD */` 안에 모드, 전류, 속도, 가속도, 비율, 호밍 거리와 같은 값이 모여 있다.

숫자 조정이 목적이라면 먼저 이 구역에서 해결할 수 있는지 확인한다.

### 2. `InEnc`, `Motor`, `mot[]`: 하드웨어와 상태

- `InEnc`: 사용자 엔코더 타이머와 이전 counter
- `Motor`: 모터 하나의 핀, 주소, 비율, 위치, 속도 상태
- `mot[]`: M1~M5의 실제 핀과 TMC 주소
- `motor_ready[]`: 드라이버 초기화 성공 여부

핀이나 모터 방향을 확인할 때 가장 먼저 볼 곳이다.

### 3. 공통 입출력 함수

- `uart_printf()`: USART2 메시지 출력
- `dwt_delay_init()`: DWT cycle counter 시작
- `step_pulse()`: STEP HIGH/LOW 펄스 한 개 출력
- `in_enc_delta()`: 엔코더 counter 변화량 계산
- `motor_driver_init()`: TMC2209 초기화와 재시도

### 4. 공통 모션 함수

- `control_tick()`: 목표 위치와 현재 위치 차이로 가감속하며 STEP 출력
- `coupled_velocity_tick()`: M4/M5 동시모드의 일정 속도 제어
- `manual_target_update()`: 엔코더 입력을 목표 STEP으로 변환

모든 축의 움직임이 이상할 때는 이 공통 함수가 관련될 수 있다. 한 축만 이상하면 먼저 해당 축 전용 함수와 설정을 확인한다.

### 5. M1/M2 호밍과 한계

- `home_seek_pair()`: 양쪽 hard stop 탐색
- `home_move_pair()`: 계산된 위치로 복귀
- `home_m1_m2_m3()`: M1/M2/M3 전체 호밍 순서 조정
- `apply_m12_limit()`: 수동제어 중 M1/M2 안전 범위 적용

### 6. M3 호밍 상태 머신

- `m3_endstop_active()`: PB14 상태를 논리적인 활성 여부로 변환
- `m3_home_begin()`: M3 호밍 상태 초기화
- `m3_home_service()`: 탐색·정지·백오프·해제 확인
- `m3_home_abort()`: 오류 발생 시 STEP 정지와 안전 처리

### 7. 축별 수동제어

- `m12_manual_tick()`: M1/M2
- `m3_manual_tick()`: M3 엔드스톱, 반전, UART 위치 출력
- `m45_manual_tick()`: M4/M5 단독 및 동시모드

### 8. `main()`: 실제 실행 순서

`main()`을 보면 어떤 주변장치를 먼저 초기화하고, 언제 호밍하며, 언제 사용자 입력을 시작하는지 알 수 있다.

### 9. CubeMX 주변장치 초기화

- `MX_ADC1_Init()`: PA7 가변저항
- `MX_TIM1_Init()`~`MX_TIM8_Init()`: 엔코더 타이머
- `MX_USART2_UART_Init()`: UART
- `MX_GPIO_Init()`: STEP/DIR/EN, 버튼, LED, 엔드스톱

핀 자체를 변경하려면 가능하면 `motorAndEncoder.ioc`도 함께 수정한다. 생성된 초기화 함수만 바꾸면 CubeMX 재생성 시 변경이 사라질 수 있다.

### 10. 버튼 interrupt

`HAL_GPIO_EXTI_Callback()`에서 PA4 모드 선택과 PA5 gain 선택을 처리한다.

## 원하는 동작별 수정 위치

아래 표에서 원하는 동작을 찾고, 매크로나 함수명을 `Ctrl+F`로 검색한다.

| 바꾸고 싶은 동작 | 먼저 찾을 항목 | 변경 종류 |
|---|---|---|
| 통합제어/피더 모드 선택 | `ACTIVE_APP_MODE` | 설정 |
| M1~M5 운전 전류 | `M1_RUN_CURRENT_MA`~`M5_RUN_CURRENT_MA` | 파라미터 |
| TMC 초기화 재시도 | `TMC_INIT_RETRIES`, `TMC_INIT_RETRY_DELAY_MS` | 파라미터 |
| STEP/DIR/EN 핀과 TMC 주소 | `mot[]` | 하드웨어 설정 |
| M1/M2 호밍 속도 | `HOME_APPROACH_V`, `HOME_RETURN_V` | 파라미터 |
| M1/M2 접촉 전류 | `HOME_M1_TOUCH_CURRENT_MA`, `HOME_M2_TOUCH_CURRENT_MA` | 파라미터 |
| M1/M2 최종 복귀 위치 | `HOME_M1_RETURN2_DEG10`, `HOME_M2_RETURN2_DEG10` | 파라미터 |
| M1/M2 hard stop 판정 | `HOME_TRIP_LAG`, `HOME_CONFIRM_WINDOWS` | 주의가 필요한 파라미터 |
| M1/M2 수동 조작 비율 | `M1_MANUAL_SCALE_...`, `M2_MANUAL_SCALE_...` | 파라미터 |
| M1/M2 수동 속도·가속도 | `M12_MANUAL_MAX_SPEED_SPS`, `M12_MANUAL_ACCEL_SPS2` | 파라미터 |
| M1/M2 수동 안전 여유 | `M1_LIMIT_MARGIN_COUNTS`, `M2_LIMIT_MARGIN_COUNTS` | 안전 파라미터 |
| M3 엔드스톱 논리 | `M3_ENDSTOP_ACTIVE_STATE` | 하드웨어 설정 |
| M3 호밍 방향 | `M3_HOME_SEEK_DIRECTION` | 하드웨어 설정 |
| M3 엔드스톱 탐색 속도 | `M3_HOME_SEEK_RATE_SPS` | 파라미터 |
| M3 탐색 최대 거리 | `M3_HOME_SEEK_MAX_STEPS` | 안전 파라미터 |
| M3 백오프 최종 위치 | `M3_HOME_BACKOFF_STEPS` | 파라미터 |
| M3 백오프 시작·최고속도 | `M3_HOME_BACKOFF_START_RATE_SPS`, `M3_HOME_BACKOFF_RATE_SPS` | 파라미터 |
| M3 백오프 가속 구간 | `M3_HOME_BACKOFF_RAMP_STEPS` | 파라미터 |
| M3 스위치 해제 판정 | `M3_HOME_RELEASE_MAX_STEPS`, `M3_HOME_RELEASE_CONFIRM_STEPS` | 안전 파라미터 |
| M3 수동 조작 비율 | `M3_MANUAL_SCALE_NUM/DEN` | 파라미터 |
| M3 수동 최고속도 | `M3_MANUAL_MAX_SPEED_SPS` | 파라미터 |
| M3 수동 가속도 | `M3_MANUAL_ACCEL_SPS2` | 파라미터 |
| M3 수동 반전 방식 | `m3_manual_tick()`의 `input_reverses_queued_motion` | 알고리즘 |
| M3 위치 출력 주기 | `M3_POSITION_REPORT_PERIOD_MS` | 파라미터 |
| M4/M5 단독 속도·가속도 | `M45_MANUAL_MAX_SPEED_SPS`, `M45_MANUAL_ACCEL_SPS2` | 파라미터 |
| 동시모드 M4 기준 속도 | `COUPLED_M4_BASE_SPEED_SPS` | 파라미터 |
| 동시모드 M4/M5 방향 | `COUPLED_M4_RETRACT_SIGN`, `COUPLED_M5_RETRACT_SIGN` | 파라미터 |
| 동시모드 M5 비율 범위 | `POT_RATIO_MIN/CENTER/MAX_X1000` | 파라미터 |
| 가변저항 방향 반전 | `POT_ADC_REVERSED` | 설정 |
| Encoder 4 모드 순서 | `HAL_GPIO_EXTI_Callback()` | 알고리즘 |
| 버튼 debounce | `BUTTON_DEBOUNCE_MS` | 파라미터 |
| UART 속도 | `UART_BAUD_RATE` | 설정 |

## 자주 하는 수정 예시

### M3 수동 가속이 너무 빠를 때

```c
#define M3_MANUAL_ACCEL_SPS2 80000.0f
```

이 값을 낮추면 목표 속도까지 천천히 올라가고 방향 반전도 부드러워진다. 최고속도를 바꾸는 값은 `M3_MANUAL_MAX_SPEED_SPS`다. 두 값을 한 번에 바꾸기보다 가속도를 먼저 조정한다.

### M3 호밍 탐색을 더 빠르게 하고 싶을 때

```c
#define M3_HOME_SEEK_RATE_SPS 6000u
```

이 값은 엔드스톱을 향해 가는 속도다. 엔드스톱 감지 후 반대로 빠져나오는 최고속도는 `M3_HOME_BACKOFF_RATE_SPS`다.

### M3가 엔드스톱에서 더 멀리 떨어진 곳을 원점으로 잡게 하고 싶을 때

```c
#define M3_HOME_BACKOFF_STEPS 22000u
```

이 값을 키우면 엔드스톱 감지 위치에서 더 멀리 후퇴한 뒤 0점으로 설정한다. `M3_HOME_SEEK_MAX_STEPS`와 혼동하면 안 된다.

### M3 반대 방향 전환이 늦을 때

`m3_manual_tick()`의 `input_reverses_queued_motion` 구역을 확인한다. 현재 코드는 새 입력이 기존 목표 방향과 반대이면 오래 쌓인 목표 위치를 현재 위치로 잘라낸다.

현재 `vel`을 강제로 0으로 만들면 움직이는 모터의 STEP 주파수가 순간적으로 끊기므로 탈조가 발생할 수 있다. 반전 반응을 조정할 때는 우선 `M3_MANUAL_ACCEL_SPS2`와 현재 큐 삭제 동작을 함께 관찰한다.

### M4/M5 동시모드 속도를 바꾸고 싶을 때

```c
#define COUPLED_M4_BASE_SPEED_SPS 800.0f
```

이 값은 M4 기준 속도다. M5는 가변저항 비율을 곱해 결정된다. 현재 동시모드는 gain 버튼을 무시하므로 300% 모드를 눌러도 기준 속도가 변하지 않는다.

### 가변저항으로 M5 속도 범위를 바꾸고 싶을 때

```c
#define POT_RATIO_MIN_X1000     200L   /* 0.2배 */
#define POT_RATIO_CENTER_X1000 1000L   /* 1.0배 */
#define POT_RATIO_MAX_X1000    1500L   /* 1.5배 */
```

예를 들어 최대 2배가 필요하면 `POT_RATIO_MAX_X1000`을 `2000L`로 바꾼다.

### 모터 하나의 방향만 반대로 바꾸고 싶을 때

`mot[]`에서 해당 모터의 `.dir_sign`을 찾는다.

```c
.dir_sign = +1
```

`+1`과 `-1`을 바꾸면 수동 입력에 대한 논리 방향이 반전된다. M3 호밍 방향은 별도의 `M3_HOME_SEEK_DIRECTION`이므로 따로 확인해야 한다.

## 이름과 단위 읽는 법

| 이름 끝 | 의미 |
|---|---|
| `_SPS` | 초당 microstep 수 |
| `_SPS2` | 1초당 속도 변화량 |
| `_STEPS` | 출력 STEP 펄스 수 |
| `_COUNTS`, `_CNT` | 엔코더 또는 ADC count |
| `_MS` | millisecond |
| `_MA` | milliampere |
| `_PCT` | percent |
| `_NUM`, `_DEN` | 분수의 분자와 분모 |
| `_X1000` | 실제 비율의 1000배 |
| `_DEG10` | 실제 각도의 10배 |

`STEPS`와 피드백 엔코더 `COUNTS`는 같은 단위가 아니다. M1/M2 호밍 값이나 수동 한계를 조정할 때 서로 직접 비교하지 않는다.

## UART에서 확인할 내용

```text
USART2: PA2 TX, PA3 RX
Baud rate: 115200
Data: 8 bit
Parity: None
Stop bit: 1
Flow control: None
```

부팅과 호밍 중에는 다음 내용을 확인한다.

- 각 TMC2209 드라이버 초기화 성공 여부
- M1/M2/M3 호밍 시작과 결과
- M1/M2 hard range와 safe range
- M3 엔드스톱 감지까지의 탐색 STEP
- M3 백오프 시작 방향과 속도
- M3 엔드스톱이 실제로 해제된 STEP
- 백오프 완료 또는 실패 원인

M3의 대표 메시지는 다음과 같다.

```text
M3 HOME ENDSTOP seek_steps=... backoff=22000
M3 HOME BACKOFF START dir_pin=... start=300 max=6000
M3 HOME ENDSTOP RELEASED step=...
M3 HOME BACKOFF DONE steps=22000 release_step=... raw=...
M3 HOME BACKOFF STALL ...
M3 HOME NO ENDSTOP ...
M3 POS current_steps=... target_steps=... lag_steps=...
```

## 프로젝트 구조

```text
ERCPver3/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── tmc2209_uart.h
│   ├── Src/
│   │   ├── main.c
│   │   ├── tmc2209_uart.c
│   │   ├── stm32f4xx_hal_msp.c
│   │   └── stm32f4xx_it.c
│   └── Startup/
├── Drivers/
├── docs/
│   ├── FIRMWARE_GUIDE.md
│   └── CONTROL_TUNING_GUIDE.md
├── HOMING_WORKLOG_2026-07-30.md
├── motorAndEncoder.ioc
├── .project
├── .cproject
└── README.md
```

주요 파일의 역할은 다음과 같다.

| 파일 | 역할 |
|---|---|
| `Core/Src/main.c` | 전체 초기화, 호밍, 수동제어, UI |
| `Core/Src/tmc2209_uart.c` | GPIO 단선 UART와 TMC2209 register 설정 |
| `Core/Inc/main.h` | CubeMX GPIO 이름과 공통 선언 |
| `motorAndEncoder.ioc` | CubeMX 핀·클록·주변장치 설정 |
| `docs/FIRMWARE_GUIDE.md` | `main.c`를 코드 순서대로 자세히 설명 |
| `docs/CONTROL_TUNING_GUIDE.md` | 원하는 동작별 수정 위치와 조정 방법 |

## STM32CubeIDE에서 열기

1. STM32CubeIDE에서 `File -> Import -> Existing Projects into Workspace`를 선택한다.
2. 저장소 루트를 프로젝트 경로로 지정한다.
3. 프로젝트 설정에서 STM32F446RE와 Debug configuration을 확인한다.
4. 프로젝트를 build한다.
5. ST-LINK로 다운로드한다.
6. ST-LINK Virtual COM Port를 115200 8-N-1로 연다.

CubeMX로 코드를 재생성하기 전에는 Git diff를 확인한다. 사용자 코드는 가능한 한 `/* USER CODE BEGIN ... */`과 `/* USER CODE END ... */` 사이에 유지한다.

## 변경 후 확인 순서

파라미터나 코드를 바꿨다면 다음 순서로 확인한다.

1. 변경한 매크로의 단위가 SPS, STEP, count 중 무엇인지 확인한다.
2. `ACTIVE_APP_MODE`가 시험하려는 모드인지 확인한다.
3. ARM GCC `-Wall -Wextra`로 컴파일한다.
4. `git diff --check`로 공백과 충돌 흔적을 확인한다.
5. 처음에는 모터 전원을 즉시 끌 수 있는 상태에서 시험한다.
6. UART에서 드라이버 초기화와 호밍 상태를 확인한다.
7. 한 번에 한 종류의 값만 바꾸고 결과를 기록한다.
8. 통합제어를 수정했다면 M1~M5 단독 동작과 M4/M5 동시모드를 각각 확인한다.

## 안전과 코드 해석 시 주의할 점

- `current_pos`는 실제 모터축 위치가 아니라 출력한 STEP 펄스로 계산한 명령 위치다.
- 탈조, stall 또는 기구 slip이 발생하면 `current_pos`와 실제 위치가 달라진다.
- M1/M2는 피드백 엔코더로 호밍과 수동 한계를 확인하지만, M3는 일반 수동운전 중 폐루프 위치제어를 하지 않는다.
- M3 하한은 물리 엔드스톱으로 막지만 상한은 아직 소프트웨어로 정하지 않았다.
- 호밍 방향과 최대 탐색 거리는 기구 끝에 힘을 가할 수 있는 안전 파라미터다.
- STEP 출력은 DWT busy-wait 기반이므로 여러 축을 동시에 고속으로 움직일 때 CPU 부하와 timing jitter가 증가할 수 있다.
- TMC2209 진단 read는 GPIO bit-bang UART timing에 영향을 주므로 모션 루프에서 무분별하게 호출하지 않는다.

## 현재 확인이 필요한 항목

- M3 실제 하단과 상단 위치 측정
- 측정값에 따른 M3 최종 수동 가동범위 결정
- M3 `25961/1150` 조작 비율 실기 확인
- M3 수동 가속도 `80000 SPS²`에서 탈조가 사라지는지 확인
- 급반전 시 이전 목표가 제거되고 자연스럽게 감속·반전하는지 확인
- M4/M5 동시모드가 gain 버튼과 관계없이 800 SPS를 유지하는지 확인
- 3.3V 로직 전원과 각 TMC2209 VIO가 정상 전압인지 확인
- PB0/PB1 TMC UART idle이 HIGH인지 확인

## 개발 기록

- `0730_double_homing`: M1/M2 피드백 엔코더 기반 양방향 hard-stop 호밍 개발
- `0820_integrated_control`: M1/M2/M3 호밍과 M1~M5 수동제어 통합
- 2026-09-18 작업본: M3 백오프 안정화, 엔드스톱 해제 확인, 수동 가속도 완화, 반전 시 목표 큐 제거, M4/M5 동시모드 gain 1 고정
