# 모터 제어 수정 위치 안내서

이 문서는 `main.c`의 전체 동작을 설명하는 문서가 아니다. **원하는 동작을 바꾸려면 어디를 수정해야 하는지** 빠르게 찾기 위한 작업용 안내서다.

코드의 줄 번호는 수정할 때마다 달라진다. 따라서 줄 번호 대신 이 문서에 적힌 매크로명이나 함수명을 STM32CubeIDE에서 `Ctrl+F`로 검색한다.

상세한 코드 동작은 `FIRMWARE_GUIDE.md`를 함께 참고한다.

---

# 1. 가장 먼저 보는 빠른 수정표

| 바꾸고 싶은 동작 | 먼저 찾을 항목 | 수정 종류 |
|---|---|---|
| M1 호밍 최종 정지점 | `HOME_M1_RETURN2_DEG10` | 파라미터 |
| M2 호밍 최종 정지점 | `HOME_M2_RETURN2_DEG10` | 파라미터 |
| M1/M2 호밍 속도 | `HOME_APPROACH_V`, `HOME_RETURN_V` | 파라미터 |
| M1/M2 hard stop 감지 감도 | `HOME_TRIP_LAG`, `HOME_CONFIRM_WINDOWS` | 주의가 필요한 파라미터 |
| M1/M2 호밍 탐색 최대 거리 | `HOME_M1_LIMIT_STEPS`, `HOME_M2_LIMIT_STEPS` | 안전 파라미터 |
| M3 호밍 방향 | `M3_HOME_SEEK_DIRECTION` | 배선·기구 확인 필수 |
| M3 엔드스톱까지 가는 속도 | `M3_HOME_SEEK_RATE_SPS` | 파라미터 |
| M3 엔드스톱 후 원점까지 이동량 | `M3_HOME_BACKOFF_STEPS` | 파라미터 |
| M3 엔드스톱 후 이동 속도 | `M3_HOME_BACKOFF_RATE_SPS` | 파라미터 |
| M1 조작 비율 | `M1_MANUAL_SCALE_NUM/DEN` | 파라미터 |
| M2 조작 비율 | `M2_MANUAL_SCALE_NUM/DEN` | 파라미터 |
| M3 조작 비율 | `M3_MANUAL_SCALE_NUM/DEN` | 파라미터 |
| M1/M2 조작 방향 | `mot[]`의 M1/M2 `.dir_sign` | 하드웨어 설정 |
| M3 조작 방향 | `mot[]`의 M3 `.dir_sign` | 하드웨어 설정 |
| M1/M2 수동 최고 속도·가속도 | `M12_MANUAL_MAX_SPEED_SPS`, `M12_MANUAL_ACCEL_SPS2` | 파라미터 |
| M3 수동 최고 속도·가속도 | `M3_MANUAL_MAX_SPEED_SPS`, `M3_MANUAL_ACCEL_SPS2` | 파라미터 |
| M4/M5 단독 최고 속도·가속도 | `M45_MANUAL_MAX_SPEED_SPS`, `M45_MANUAL_ACCEL_SPS2` | 파라미터 |
| M1/M2 수동 안전 여유 | `M1_LIMIT_MARGIN_COUNTS`, `M2_LIMIT_MARGIN_COUNTS` | 안전 파라미터 |
| M3 수동 상한·하한 | 현재 미구현, `m3_manual_tick()`에 추가 | 코드 구현 필요 |
| M3 수동 중 엔드스톱 방향 차단 | 현재 미구현, `m3_manual_tick()`에 추가 | 코드 구현 필요 |
| M4/M5 동시 모드 기본 속도 | `COUPLED_M4_BASE_SPEED_SPS` | 파라미터 |
| 동시 모드 M4/M5 방향 | `COUPLED_M4_RETRACT_SIGN`, `COUPLED_M5_RETRACT_SIGN` | 파라미터 |
| 가변저항의 M5 비율 범위 | `POT_RATIO_MIN/CENTER/MAX_X1000` | 파라미터 |
| 100%/300% 모드 | `gain_pct` 계산과 `HAL_GPIO_EXTI_Callback()` | 코드 또는 UI 변경 |
| Encoder 4 모드 순서 | `HAL_GPIO_EXTI_Callback()`의 `enc4_sel` 전환 | 코드 변경 |
| SWV 출력 주기 | `SWV_REPORT_PERIOD_MS` | 파라미터 |
| SWV 출력 내용 | `swv_report_manual()` | 코드 변경 |
| TMC 초기화 재시도 | `TMC_INIT_RETRIES`, `TMC_INIT_RETRY_DELAY_MS` | 파라미터 |
| 모터 운전 전류 | `motor_driver_init()`의 `TMC2209uart(500, ...)` | 코드 변경 |
| STEP/DIR/EN 핀 또는 TMC 주소 | `mot[]` | 하드웨어 설정 |
| 사용자 엔코더 타이머 연결 | `in_enc[]` | 하드웨어 설정 |

---

# 2. 수정 전에 지킬 기준

## 파라미터 변경과 알고리즘 변경을 구분한다

- 숫자만 조정하려면 해당 `#define`만 바꾼다.
- 움직임의 순서나 판정 방식을 바꾸려면 관련 함수를 수정한다.
- 여러 값을 동시에 바꾸면 어떤 값이 결과에 영향을 줬는지 알기 어렵다. 한 번에 한 종류만 바꾼다.

## 단위를 먼저 확인한다

| 이름 끝 | 단위 |
|---|---|
| `_SPS` | 초당 motor microstep 수 |
| `_SPS2` | 초당 microstep 속도의 변화량 |
| `_STEPS` | 출력 STEP 펄스 수, 현재 설정에서는 microstep 수 |
| `_COUNTS`, `_CNT` | 엔코더 또는 ADC count |
| `_MS` | 밀리초 |
| `_MA` | 밀리암페어 |
| `_PCT` | 퍼센트 |
| `_DEG10` | 실제 각도의 10배 정수 |
| `_X1000` | 실제 비율의 1000배 정수 |

`STEPS`와 피드백 엔코더 `COUNTS`는 같은 단위가 아니다. 특히 M1/M2 호밍 및 한계값을 조정할 때 혼동하지 않는다.

## CubeMX 보호 구간을 확인한다

사용자 코드는 가능한 한 `/* USER CODE BEGIN ... */`와 `/* USER CODE END ... */` 사이에서 수정한다. `MX_GPIO_Init()`나 `MX_TIMx_Init()` 같은 자동 생성 영역을 직접 수정하면 CubeMX 코드 재생성 시 사라질 수 있다. 핀과 타이머 설정은 가능하면 `.ioc`에서 변경한다.

## 호밍 파라미터는 안전 파라미터다

호밍 방향, 최대 이동 거리, hard stop 판정값을 잘못 바꾸면 기구 끝에 계속 힘을 가할 수 있다. 처음 시험할 때는 전원을 즉시 차단할 수 있는 상태에서 낮은 속도와 짧은 이동 한도로 검증한다.

---

# 3. M1/M2 호밍을 수정하려면

M1과 M2는 각각 TIM5와 TIM8 피드백 엔코더를 사용한다. 양쪽 기계적 끝을 찾아 전체 이동 범위를 측정한 뒤, 두 번째 hard stop에서 지정된 각도만큼 돌아와 원점을 잡는다.

M1/M2 호밍은 `main.c`의 다음 주석 바로 아래에 모여 있다.

```c
/* Encoder homing for M1/M2. ... */
```

## 최종 정지점을 바꾸고 싶을 때

먼저 다음 두 값을 수정한다.

```c
#define HOME_M1_RETURN2_DEG10 1390u
#define HOME_M2_RETURN2_DEG10 1383u
```

- `1390`은 139.0도다.
- `1383`은 138.3도다.
- 두 번째 hard stop에서 첫 번째 hard stop 방향으로 돌아오는 각도를 뜻한다.
- 값을 키우면 두 번째 hard stop에서 더 멀리 돌아온다.
- 값을 줄이면 두 번째 hard stop에 더 가까운 곳에서 멈춘다.

전체 기구 각도 기준은 다음 값이다.

```c
#define HOME_M1_SPAN_DEG10 2539u
#define HOME_M2_SPAN_DEG10 2452u
```

각각 253.9도와 245.2도를 뜻한다. 코드는 실제 측정한 피드백 엔코더 전체 span을 이 각도와 대응시켜 최종 복귀 count를 계산한다.

최종 위치 계산은 `home_m1_m2_m3()` 안의 `home_counts[]` 계산에서 수행한다. 단순히 최종 정지 위치만 조절할 때는 이 계산식 자체를 바꾸지 않고 `HOME_M1_RETURN2_DEG10` 또는 `HOME_M2_RETURN2_DEG10`만 조정한다.

## hard stop을 찾는 속도를 바꾸고 싶을 때

```c
#define HOME_APPROACH_V 800u
```

M1/M2가 기계적 끝을 찾을 때의 STEP 출력 속도다. 두 축에 공통 적용된다.

```c
#define HOME_RETURN_V 800u
```

두 번째 hard stop에서 최종 원점 위치로 돌아올 때의 속도다.

값을 높이면 빨라지지만 충격과 stall 판정 오차 가능성이 커진다. 값을 낮추면 부드러워지지만 전체 호밍 시간이 길어진다.

## hard stop 접촉 전류를 바꾸고 싶을 때

```c
#define HOME_M1_TOUCH_CURRENT_MA 175u
#define HOME_M2_TOUCH_CURRENT_MA 300u
#define HOME_NORMAL_CURRENT_MA   500u
```

- `HOME_M1_TOUCH_CURRENT_MA`: M1이 hard stop을 찾는 동안의 전류다.
- `HOME_M2_TOUCH_CURRENT_MA`: M2가 hard stop을 찾는 동안의 전류다.
- `HOME_NORMAL_CURRENT_MA`: hard stop 탐색 종료 후 복귀시키는 전류다.

접촉 전류를 높이면 끝단 힘과 발열이 증가한다. 너무 낮으면 실제 끝에 도달하기 전에 motor가 멈춰 잘못 검출될 수 있다.

## hard stop 검출 감도를 바꾸고 싶을 때

관련 값은 다음과 같다.

```c
#define HOME_WIN              16u
#define HOME_ARM_WINDOWS       3u
#define HOME_RNUM             25
#define HOME_RDEN             64
#define HOME_TRIP_LAG         80
#define HOME_CONFIRM_WINDOWS   2u
#define HOME_SECOND_ARM_COUNTS 6000
```

- `HOME_WIN`: 몇 STEP마다 피드백 이동을 검사할지 결정한다.
- `HOME_ARM_WINDOWS`: 출발 직후 몇 window를 관찰한 뒤 검출을 허용할지 결정한다.
- `HOME_RNUM/HOME_RDEN`: 출력 STEP에 대해 예상하는 피드백 count 관계다.
- `HOME_TRIP_LAG`: 예상 이동보다 피드백이 얼마나 뒤처졌을 때 접촉 후보로 볼지 결정한다.
- `HOME_CONFIRM_WINDOWS`: 접촉 후보가 몇 window 연속 이어져야 확정할지 결정한다.
- `HOME_SECOND_ARM_COUNTS`: 반대편 끝을 찾을 때 충분히 이동한 뒤 검출을 허용하는 기준이다.

이 값들은 서로 영향을 준다. 단순한 속도나 최종 위치 조정이 아니라 오검출·미검출 문제를 해결할 때만 수정한다. 실제 판정 로직은 `home_seek_pair()`에 있다.

## 최대 탐색 거리를 바꾸고 싶을 때

```c
#define HOME_M1_LIMIT_STEPS 12000u
#define HOME_M2_LIMIT_STEPS 12288u
```

각 축이 hard stop을 찾지 못했을 때 무한히 움직이지 않도록 제한하는 최대 STEP 수다. 너무 짧으면 정상 기구에서도 `HOME_NO_HARDSTOP`이 발생하고, 너무 길면 고장 상황에서 기구를 오래 밀 수 있다.

## hard stop에 추가로 밀착시키는 양을 바꾸고 싶을 때

```c
#define HOME_M1_SEAT_STEPS 32u
#define HOME_M2_SEAT_STEPS  0u
```

stall을 검출한 뒤 같은 방향으로 추가 출력하는 STEP 수다. 기구에 직접 힘이 가해지는 값이므로 신중히 조정한다.

## 측정된 전체 이동 범위 검사를 바꾸고 싶을 때

```c
#define HOME_MIN_SPAN_CNT 12000
#define HOME_MAX_SPAN_CNT 30000
```

두 hard stop 사이에서 측정된 피드백 엔코더 count가 이 범위 밖이면 `HOME_SPAN_FAULT`로 중단한다. 실제 장비에서 정상 span을 여러 번 측정한 뒤 충분한 허용폭을 두고 설정한다.

## 방향이나 피드백 부호가 잘못됐을 때

`home_ax[]`에서 다음 필드를 확인한다.

```text
M1: TIM5, is32=1, esign=+1, first_dir=-1
M2: TIM8, is32=0, esign=-1, first_dir=-1
```

- `esign`: 피드백 엔코더 count를 논리적 양의 방향으로 변환하는 부호다.
- `first_dir`: 첫 번째 hard stop을 찾는 논리 방향이다.

이 값은 조작감을 조절하는 값이 아니다. 모터 또는 피드백 엔코더 배선 방향이 실제 장비와 반대일 때만 수정한다.

## M1/M2 호밍 알고리즘 자체를 바꿀 때

함수 흐름은 다음과 같다.

```text
home_m1_m2_m3()
  -> home_set_axis_currents()
  -> home_seek_pair()       첫 번째와 두 번째 hard stop 탐색
  -> home_move_pair()       계산된 최종 위치로 복귀
  -> m12_limit[] 구성       수동 운전 안전 범위 생성
```

숫자 조정으로 해결할 수 없는 경우에만 이 함수들을 수정한다.

---

# 4. M3 호밍을 수정하려면

M3는 피드백 엔코더 stall 검출을 사용하지 않는다. GPIO 엔드스톱까지 이동한 뒤 반대 방향으로 정해진 STEP 수만큼 이동하고 그 위치를 0으로 설정한다.

## 엔드스톱 눌림 상태를 바꾸고 싶을 때

```c
#define M3_ENDSTOP_ACTIVE_STATE GPIO_PIN_SET
```

- 눌렀을 때 입력이 HIGH면 `GPIO_PIN_SET`을 사용한다.
- 눌렀을 때 입력이 LOW면 `GPIO_PIN_RESET`으로 바꾼다.

실제 입력 회로의 pull-up/pull-down 설정과 함께 확인한다. 판정 함수는 `m3_endstop_active()`다.

## 엔드스톱을 찾는 방향을 바꾸고 싶을 때

```c
#define M3_HOME_SEEK_DIRECTION (-1)
```

반대쪽으로 움직이면 `+1`과 `-1`을 바꾼다. 이 값은 수동 조작 방향인 `.dir_sign`과 별개다.

## 엔드스톱 탐색 속도와 최대 거리를 바꾸고 싶을 때

```c
#define M3_HOME_SEEK_RATE_SPS   6000u
#define M3_HOME_SEEK_MAX_STEPS 64000u
```

- `M3_HOME_SEEK_RATE_SPS`: 엔드스톱까지 이동하는 속도다.
- `M3_HOME_SEEK_MAX_STEPS`: 엔드스톱을 찾지 못했을 때 중단하는 최대 STEP 수다.

## 호밍 후 최종 정지점을 바꾸고 싶을 때

```c
#define M3_HOME_BACKOFF_STEPS    27000u
#define M3_HOME_BACKOFF_RATE_SPS  6000u
```

- `M3_HOME_BACKOFF_STEPS`: 엔드스톱이 검출된 위치에서 반대 방향으로 이동하는 STEP 수다.
- 값을 키우면 엔드스톱에서 더 멀리 떨어진 곳이 원점이 된다.
- 값을 줄이면 엔드스톱에 더 가까운 곳이 원점이 된다.
- `M3_HOME_BACKOFF_RATE_SPS`는 이 이동의 속도다.

M3의 현재 27,000 STEP 원점 위치를 조정하려면 우선 `M3_HOME_BACKOFF_STEPS`만 변경한다.

## 스위치 안정화 시간을 바꾸고 싶을 때

```c
#define M3_HOME_SETTLE_MS 10u
```

엔드스톱 검출 직후와 backoff 완료 후 스위치 상태가 안정되기를 기다리는 시간이다. backoff 후에도 스위치가 눌려 있으면 `HOME_SWITCH_NOT_RELEASED`로 실패한다.

## M3 호밍 순서 자체를 바꾸고 싶을 때

다음 함수와 상태 머신을 수정한다.

```text
m3_home_begin()     호밍 상태 초기화
m3_home_service()   SEEK -> SETTLE -> BACKOFF -> RELEASE 확인
m3_home_finished()  완료 여부 확인
m3_home_abort()     실패 시 STEP 정지
```

M3 호밍은 M1/M2 호밍 중 `m3_home_service()`가 반복 호출되는 방식으로 동시에 진행된다. 동시 호밍 구조를 바꾸려면 `home_m1_m2_m3()`와 `home_seek_pair()`, `home_move_pair()`도 함께 검토해야 한다.

---

# 5. M1/M2 수동 제어를 수정하려면

## 손잡이 한 count당 이동량을 바꾸고 싶을 때

```c
#define M1_MANUAL_SCALE_NUM  95L
#define M1_MANUAL_SCALE_DEN  26L
#define M2_MANUAL_SCALE_NUM 171L
#define M2_MANUAL_SCALE_DEN 181L
```

실제 변환은 다음과 같다.

```text
motor microstep/input count
= 3200/8192 x MANUAL_SCALE_NUM/MANUAL_SCALE_DEN
```

- 더 많이 움직이게 하려면 `NUM`을 키우거나 `DEN`을 줄인다.
- 덜 움직이게 하려면 `NUM`을 줄이거나 `DEN`을 키운다.
- 분자와 분모를 함께 같은 배수로 바꿔도 실제 비율은 변하지 않는다.

현재 값은 다음과 같다.

```text
M1: 3200/8192 x 95/26
M2: 3200/8192 x 171/181
```

## 수동 조작 방향을 바꾸고 싶을 때

`mot[]`의 M1 또는 M2 항목에서 다음 값을 변경한다.

```c
.dir_sign = -1,  /* M1 */
.dir_sign = +1,  /* M2 */
```

`+1`과 `-1`을 서로 바꾸면 사용자 입력에 대한 모터 방향이 반전된다. 이 값은 M1/M2 호밍 피드백 부호 `home_ax[].esign`과 역할이 다르다.

## 최대 속도와 가속도를 바꾸고 싶을 때

```c
#define M12_MANUAL_MAX_SPEED_SPS 51200.0f
#define M12_MANUAL_ACCEL_SPS2  160000.0f
```

- 최대 속도를 높이면 빠른 손잡이 입력을 더 빨리 따라간다.
- 최대 속도가 낮으면 빠르게 입력했을 때 목표가 쌓이고 손을 놓은 뒤에도 계속 따라갈 수 있다.
- 가속도를 높이면 시작·정지 반응이 빨라지지만 움직임이 거칠어질 수 있다.
- 가속도를 낮추면 부드러워지지만 반응이 늦어진다.

M1과 M2에 공통 적용된다. 두 축의 속도나 가속도를 서로 다르게 만들려면 M1/M2별 매크로를 새로 만들고 `m12_manual_tick()` 호출 시 축에 따라 선택하도록 코드 변경이 필요하다.

## 수동 운전 상한·하한 여유를 바꾸고 싶을 때

```c
#define M1_LIMIT_MARGIN_COUNTS  200
#define M2_LIMIT_MARGIN_COUNTS  200
#define M12_LIMIT_RELEASE_COUNTS 50
```

- `M1/M2_LIMIT_MARGIN_COUNTS`: 호밍에서 찾은 실제 hard stop보다 안쪽으로 확보하는 안전 여유다.
- 값을 키우면 사용 가능한 이동 범위가 줄지만 hard stop과 더 멀어진다.
- `M12_LIMIT_RELEASE_COUNTS`: 경계에서 limit가 반복해서 켜지고 꺼지는 것을 방지하는 hysteresis다.

이 값들의 단위는 출력 STEP이 아니라 TIM5/TIM8 피드백 엔코더 count다.

제한 동작은 `apply_m12_limit()`에서 처리한다. 현재 코드는 경계 바깥 방향의 새 입력을 0으로 만들고, 이미 누적된 목표나 속도가 바깥쪽을 향하면 즉시 `stop_motor_motion()`을 호출한다.

## M1/M2 수동 제어 방식 자체를 바꾸고 싶을 때

호출 흐름은 다음과 같다.

```text
main()의 while 루프
  -> m12_manual_tick()
     -> apply_m12_limit()
     -> manual_position_tick()
        -> control_tick()
```

- 한계 처리 변경: `apply_m12_limit()`
- 입력 count를 목표 위치로 바꾸는 방식 변경: `manual_position_tick()`
- 가속·감속 및 STEP 출력 방식 변경: `control_tick()`

상급자가 익숙한 M1/M2 조작감을 유지하려면 먼저 비율과 속도·가속도만 조정하고, 위 함수의 수학식은 필요성이 확인되기 전까지 유지한다.

---

# 6. M3 수동 제어를 수정하려면

## 조작 비율을 바꾸고 싶을 때

```c
#define M3_MANUAL_SCALE_NUM 25961L
#define M3_MANUAL_SCALE_DEN  2000L
```

현재 변환은 다음과 같다.

```text
M3 = 3200/8192 x 25961/2000
   = 약 5.07 motor microsteps/input count
```

비율 조정 방법은 M1/M2와 같다. `NUM`을 키우거나 `DEN`을 줄이면 같은 손잡이 회전에 더 많이 움직인다.

## 방향을 바꾸고 싶을 때

`mot[]`의 M3 항목에서 다음 값을 바꾼다.

```c
.dir_sign = +1,
```

이 값은 M3 수동 조작 방향이다. `M3_HOME_SEEK_DIRECTION`은 호밍 방향이므로 서로 독립적으로 확인한다.

## 최대 속도와 가속도를 바꾸고 싶을 때

```c
#define M3_MANUAL_MAX_SPEED_SPS 51200.0f
#define M3_MANUAL_ACCEL_SPS2  160000.0f
```

M3만 독립적으로 조정할 수 있다. 조작 비율을 크게 올렸는데 최고 속도가 충분하지 않으면 `target_pos`가 과도하게 앞서가고 손잡이를 놓은 뒤에도 계속 이동할 수 있다.

## M3 상한·하한을 만들고 싶을 때

현재 M3 수동 운전에는 상한·하한이 구현되어 있지 않다. `M1_LIMIT_MARGIN_COUNTS` 같은 값을 바꿔도 M3에는 적용되지 않는다.

구현 위치는 `m3_manual_tick()`이다. 권장 흐름은 다음과 같다.

```text
M3 현재 위치 또는 기준 센서 확인
-> 하한에서 아래 방향 입력 차단
-> 상한에서 위 방향 입력 차단
-> 이미 누적된 target과 velocity가 금지 방향이면 정지
-> manual_position_tick() 호출
```

M3에는 일반 운전용 피드백 엔코더가 없으므로 상한·하한의 기준을 먼저 결정해야 한다.

- `mot[MOTOR_M3_INDEX].current_pos`: 출력한 STEP 기준 상대 위치
- M3 엔드스톱 GPIO: 물리적인 하한 검출
- 별도 위치 센서: 실제 위치 기준이 필요할 때

현재 호밍 완료 시 `reset_motor_motion_state()`가 호출되므로 backoff 완료 지점이 `current_pos=0`이 된다. 이 기준을 사용할 경우 상한과 하한은 원점 기준 STEP 값으로 정의할 수 있다. 다만 stall이나 step loss가 발생하면 실제 위치와 달라질 수 있다.

## 수동 운전 중 엔드스톱 방향 차단을 만들고 싶을 때

현재 `m3_endstop_active()`는 호밍에서만 사용된다. 수동 제어에서도 사용하려면 `m3_manual_tick()`에서 다음 정책을 구현해야 한다.

1. 엔드스톱이 눌렸을 때 엔드스톱 쪽 입력만 차단한다.
2. 반대 방향으로 빠져나가는 입력은 허용한다.
3. 이미 엔드스톱 쪽으로 쌓인 `target_pos`와 `vel`은 즉시 정리한다.
4. 스위치 bounce가 있으면 별도의 debounce 또는 release hysteresis를 적용한다.

이 작업은 단순 매크로 조정이 아니라 코드 구현 작업이다. 수정 중심 함수는 `m3_manual_tick()`, `m3_endstop_active()`, `stop_motor_motion()`이다.

---

# 7. M4/M5 수동 제어를 수정하려면

Encoder 4는 다음 세 모드를 순환한다.

```text
M4 단독 -> M5 단독 -> M4/M5 동시 -> M4 단독
```

## M4 또는 M5 단독 모드 조작 비율을 바꾸고 싶을 때

`mot[]`의 M4와 M5 항목에 현재 다음 값이 들어 있다.

```c
.manual_scale_num = 1L,
.manual_scale_den = 1L,
```

M4와 M5를 서로 다르게 만들려면 각 항목의 분자와 분모를 독립적으로 변경한다. 공통 `3200/8192` 변환과 100%/300% gain은 그대로 적용된다.

## 단독 모드 최고 속도와 가속도를 바꾸고 싶을 때

```c
#define M45_MANUAL_MAX_SPEED_SPS 51200.0f
#define M45_MANUAL_ACCEL_SPS2  160000.0f
```

현재 M4와 M5에 공통 적용된다.

## 동시 모드 속도를 바꾸고 싶을 때

```c
#define COUPLED_M4_BASE_SPEED_SPS 800.0f
```

동시 모드는 Encoder 4 위치 입력을 사용하지 않고 일정 속도로 M4와 M5를 구동한다. 100%/300% 모드가 이 기본 속도에 곱해진다.

## 동시 모드 방향을 바꾸고 싶을 때

```c
#define COUPLED_M4_RETRACT_SIGN  1.0f
#define COUPLED_M5_RETRACT_SIGN -1.0f
```

각 값을 `+1.0f`와 `-1.0f` 사이에서 바꾸면 해당 모터 방향이 반전된다. 단독 모드의 `.dir_sign`과는 별개의 설정이다.

## 가변저항으로 정하는 M5 속도 비율을 바꾸고 싶을 때

```c
#define POT_RATIO_MIN_X1000     200L
#define POT_RATIO_CENTER_X1000 1000L
#define POT_RATIO_MAX_X1000    1500L
```

현재 의미는 다음과 같다.

```text
최소: M4 속도의 0.2배
중앙: M4 속도의 1.0배
최대: M4 속도의 1.5배
```

ADC 중앙 무감도와 필터는 다음 값으로 조정한다.

```c
#define POT_ADC_CENTER_COUNTS   2048u
#define POT_ADC_DEADBAND_COUNTS  128u
#define POT_SAMPLE_PERIOD_MS      10u
#define POT_FILTER_SHIFT           3u
#define POT_ADC_REVERSED            0u
```

- 중앙에서 비율이 흔들리면 `POT_ADC_DEADBAND_COUNTS`를 키운다.
- 반응이 너무 흔들리면 `POT_FILTER_SHIFT`를 키운다. 대신 반응은 느려진다.
- 가변저항을 돌린 방향과 비율 증감이 반대면 `POT_ADC_REVERSED`를 `1u`로 바꾼다.

변환 함수는 `pot_adc_to_ratio_x1000()`, 주기적 샘플링과 필터는 `pot_ratio_update()`에 있다.

## Encoder 4 모드 순서를 바꾸고 싶을 때

`HAL_GPIO_EXTI_Callback()`에서 GPIO PA4 처리 부분을 수정한다.

```text
SEL_M4 -> SEL_M5 -> SEL_M4_M5_COUPLED -> SEL_M4
```

모드가 바뀌면 메인 루프가 `reset_m4_m5_motion_state()`를 호출해 이전 목표와 속도를 제거한다. 모드 변경 시 남은 움직임이 이어지는 문제를 막는 코드이므로 유지하는 것이 좋다.

## M4/M5 제어 방식 자체를 바꾸고 싶을 때

- 단독 모드와 동시 모드 분기: `m45_manual_tick()`
- 단독 위치 추종: `manual_position_tick()`와 `control_tick()`
- 동시 일정 속도 제어: `coupled_velocity_tick()`
- 모드 변경 시 상태 정리: `reset_m4_m5_motion_state()`

---

# 8. 모든 축에 영향을 주는 공통 항목

## 모터와 입력 엔코더 기본 분해능

```c
#define MICROSTEP                        16u
#define MOTOR_FULL_STEPS_PER_REV        200L
#define INPUT_ENCODER_PULSES_PER_REV   2048L
#define INPUT_ENCODER_QUADRATURE_FACTOR   4L
```

이 값은 입력 count를 motor microstep으로 바꾸는 공통 비율에 영향을 준다. `MICROSTEP`을 바꾸면 TMC 설정뿐 아니라 1회전당 STEP 수, 기존 호밍 거리, 최고 속도, 가속도 의미까지 함께 바뀐다. 한 축의 조작감만 바꾸기 위해 `MICROSTEP`을 조정하지 않는다.

## 위치 추종 가감속 방식을 바꾸고 싶을 때

`control_tick()`을 수정한다.

이 함수는 M1, M2, M3와 M4/M5 단독 모드에 공통으로 사용된다. 한 축만 바꾸려면 함수 자체를 바로 수정하기보다 축별 인자나 별도 함수를 추가해야 다른 축의 조작감이 변하지 않는다.

## 동시 모드의 일정 속도 가감속을 바꾸고 싶을 때

`coupled_velocity_tick()`을 수정한다. 이 함수는 M4/M5 동시 모드에만 사용된다.

## STEP 펄스 폭을 바꾸고 싶을 때

```c
#define STEP_PULSE_CYCLES (SystemCoreClock / 1000000u)
```

`step_pulse()`에서 HIGH와 LOW 각각 약 1 us를 기다린다. TMC2209의 STEP timing 요구사항과 CPU blocking 시간을 함께 검토해야 한다.

## 손잡이를 놓아도 모터가 계속 따라오는 현상을 줄이고 싶을 때

먼저 다음 순서로 확인한다.

1. SWV에서 `current_pos`와 `target_pos` 차이가 계속 큰지 확인한다.
2. 축별 `MANUAL_MAX_SPEED_SPS`가 입력 속도를 따라갈 만큼 충분한지 확인한다.
3. 축별 `MANUAL_ACCEL_SPS2`가 너무 낮지 않은지 확인한다.
4. 조작 비율이 실제 필요한 값보다 지나치게 크지 않은지 확인한다.
5. 필요하면 `manual_position_tick()`에 목표 누적 제한 정책을 추가한다.

단순히 `target_pos=current_pos`로 매 루프 덮어쓰면 정상 위치 추종도 사라지므로 원인을 확인한 뒤 수정한다.

---

# 9. 드라이버 초기화와 전류를 수정하려면

## 초기화 재시도 횟수와 간격

```c
#define TMC_INIT_RETRIES          3u
#define TMC_INIT_RETRY_DELAY_MS 100u
```

TMC 통신이 일시적으로 실패할 때 재시도하는 횟수와 간격이다. 실제 처리는 `motor_driver_init()`에 있다.

## 일반 운전 전류

`motor_driver_init()`에서 다음 호출을 찾는다.

```c
TMC2209uart(500, MICROSTEP, ...)
```

여기서 `500`이 드라이버 초기화 시 설정되는 일반 운전 전류 mA다. 가독성을 높이려면 향후 `MOTOR_RUN_CURRENT_MA` 같은 매크로로 분리하는 것이 좋다.

M1/M2 호밍 탐색 중에는 별도의 `HOME_M1_TOUCH_CURRENT_MA`, `HOME_M2_TOUCH_CURRENT_MA`가 적용된다.

## 통신 실패 시 동작

`motor_driver_init()`는 성공한 축만 `motor_ready[]`를 1로 만들고 enable을 LOW로 내린다. M1/M2/M3 중 하나라도 호밍에 필요한 상태가 아니면 호밍 오류로 전체 수동 운전을 막는다.

---

# 10. 핀, 타이머, TMC 주소를 수정하려면

## 모터 STEP/DIR/EN/TMC UART 연결

`mot[]`의 해당 모터 항목을 수정한다.

```text
step_port/step_pin : STEP 출력
dir_port/dir_pin   : DIR 출력
en_port/en_pin     : TMC enable, active-low
pdn_port/pdn_pin   : TMC2209 단선 UART
tmc_addr           : TMC 공유 버스 주소
```

핀을 바꿀 때는 `mot[]`만 바꾸는 것으로 끝나지 않을 수 있다. `MX_GPIO_Init()` 또는 `.ioc`의 GPIO mode, pull, 초기 출력 상태도 같이 확인한다.

## 사용자 입력 엔코더 연결

`in_enc[]`에서 타이머와 비트 폭을 수정한다.

```c
{ &htim1, 0, 0 },  /* 16-bit */
{ &htim2, 1, 0 },  /* 32-bit */
```

두 번째 값 `is32`는 해당 타이머가 32비트이면 `1`, 16비트이면 `0`이다. 잘못 지정하면 counter overflow 시 큰 delta가 발생할 수 있다.

타이머 핀과 encoder mode 설정은 `.ioc`와 `MX_TIMx_Init()`도 함께 확인한다.

## M1/M2 피드백 엔코더 연결

`home_ax[]`에서 M1의 TIM5와 M2의 TIM8 연결을 확인한다. 타이머를 바꾸면 비트 폭 `is32`, 방향 부호 `esign`, `.ioc`의 encoder 설정도 같이 바꿔야 한다.

## M3 엔드스톱 핀

다음 매크로를 확인한다.

```c
M3_ENDSTOP_Pin
M3_ENDSTOP_GPIO_Port
```

현재는 CubeMX에서 같은 이름이 정의되지 않았을 경우 PB14를 fallback으로 사용한다. 실제 보드 핀을 바꾸려면 `main.h`, `.ioc`, `MX_GPIO_Init()`의 입력 및 pull 설정을 함께 확인한다.

---

# 11. 버튼과 표시를 수정하려면

## 버튼 debounce 시간

```c
#define BUTTON_DEBOUNCE_MS 200u
```

PA4와 PA5 버튼에 공통 적용된다. 실제 인터럽트 처리는 `HAL_GPIO_EXTI_Callback()`에 있다.

## 100%/300% 속도 모드

PA5를 누르면 `MODE_PRECISION`과 `MODE_FAST`가 전환된다. 실제 gain은 메인 루프의 다음 계산에서 결정된다.

```c
int32_t gain_pct = (current_mode == MODE_FAST) ? 300 : 100;
```

이 gain은 현재 M4/M5에 적용되고 M1/M2/M3에는 `MANUAL_FIXED_GAIN_PCT=100`이 적용된다. M1/M2/M3도 버튼 영향을 받게 하려면 각 수동 함수에 `gain_pct`를 전달하는 코드 변경이 필요하다.

## Encoder 4 모드 표시 LED

메인 루프에서 PA6을 제어한다.

```text
M4 단독: 켜짐
M5 단독: 꺼짐
M4/M5 동시: 점멸
```

점멸 주기는 `HAL_GetTick() / 250u` 계산으로 정해진다.

---

# 12. SWV 디버깅 내용을 수정하려면

## 출력 주기

```c
#define SWV_REPORT_PERIOD_MS 200u
```

값을 너무 작게 하면 ITM 출력이 제어 루프에 부담을 줄 수 있다.

## 주기적으로 출력하는 항목

`swv_report_manual()`을 수정한다. 현재 출력 항목은 다음과 같다.

- M1/M2 현재 피드백 위치
- M1/M2 안전 범위와 hard stop 범위
- M1/M2 현재 limit 상태
- M3 `current_pos`와 `target_pos`
- M4/M5 `current_pos`
- Encoder 4 선택 모드
- M4/M5 동시 모드 속도 비율

## 부팅·호밍 메시지

`main()`의 `swv_printf()` 호출과 `home_status_name()`을 확인한다.

현재 디버깅 방식은 STM32CubeIDE SWV ITM Data Console이며 Core Clock은 84 MHz, Stimulus Port는 0이다. UART 출력은 아직 구현하지 않았다.

---

# 13. 자주 하는 수정 예시

## “M1 손잡이를 같은 만큼 돌렸을 때 더 많이 움직이게 하고 싶다”

1. `M1_MANUAL_SCALE_NUM`을 조금 키운다.
2. `M1_MANUAL_SCALE_DEN`은 유지한다.
3. 빠르게 돌렸을 때 목표가 쌓이면 `M12_MANUAL_MAX_SPEED_SPS`와 `M12_MANUAL_ACCEL_SPS2`도 확인한다.

## “M2 방향만 반대로 움직이게 하고 싶다”

1. `mot[]`의 M2 `.dir_sign`을 `+1`에서 `-1`로 바꾼다.
2. 수동 방향만 확인한다.
3. 호밍까지 반대로 바꾸고 싶은 것이 아니라면 `home_ax[]`는 수정하지 않는다.

## “M1 호밍 후 정지 위치를 조금 더 안쪽으로 옮기고 싶다”

1. 어느 hard stop을 기준으로 더 안쪽인지 실제 방향을 확인한다.
2. `HOME_M1_RETURN2_DEG10`을 조금 키운다.
3. SWV의 hard/safe 범위와 실제 정지 위치를 확인한다.
4. 한 번에 큰 폭으로 바꾸지 않는다.

## “M3 원점을 엔드스톱에서 더 멀리 잡고 싶다”

1. `M3_HOME_BACKOFF_STEPS`를 키운다.
2. `M3_HOME_BACKOFF_RATE_SPS`는 우선 유지한다.
3. backoff 후 엔드스톱이 확실히 해제되는지 확인한다.

## “M3가 하한 엔드스톱을 누른 상태에서 더 내려가지 못하게 하고 싶다”

숫자만 바꾸는 작업이 아니다. `m3_manual_tick()`에서 `m3_endstop_active()`를 검사해 엔드스톱 방향 입력과 이미 누적된 목표를 차단하는 코드를 추가해야 한다. 반대 방향 탈출 입력은 허용해야 한다.

## “M3 상한을 정하고 싶다”

1. 호밍 완료 지점 `current_pos=0`을 기준으로 사용할지 결정한다.
2. 실제 장비에서 허용되는 최대 STEP 위치를 측정한다.
3. 의미 있는 `M3_MANUAL_MAX_POSITION_STEPS` 매크로를 추가한다.
4. `m3_manual_tick()`에서 상한 방향 입력과 누적 목표를 차단한다.
5. step loss 가능성을 허용할 수 있는지 별도로 판단한다.

## “M4/M5 동시 모드에서 M5만 더 빠르게 하고 싶다”

1. 가변저항 최대 범위라면 `POT_RATIO_MAX_X1000`을 키운다.
2. 중앙 위치의 기본 비율을 바꾸려면 `POT_RATIO_CENTER_X1000`을 바꾼다.
3. 전체 동시 속도는 `COUPLED_M4_BASE_SPEED_SPS`로 조정한다.

---

# 14. 변경 후 확인 순서

1. 수정 전 값을 기록한다.
2. 매크로 단위가 STEP인지 encoder count인지 확인한다.
3. 한 번에 한 축 또는 한 종류의 파라미터만 변경한다.
4. 컴파일 경고와 오류가 없는지 확인한다.
5. 처음에는 모터 전원을 즉시 차단할 수 있는 상태에서 시험한다.
6. 호밍 변경 후 모든 축의 호밍 성공 및 실패 정지를 확인한다.
7. 수동 변경 후 저속 입력, 빠른 입력, 방향 전환, 손을 놓은 뒤의 잔여 이동을 확인한다.
8. M1/M2는 SWV의 `safe`, `hard`, `limit` 값을 확인한다.
9. M3는 엔드스톱 검출·해제와 실제 하한 높이를 확인한다.
10. M4/M5는 세 모드 전환 시 이전 목표가 이어지지 않는지 확인한다.
11. 확정된 실제 장비 값을 이 문서와 `FIRMWARE_GUIDE.md`에 함께 반영한다.

---

# 15. 현재 반드시 기억할 미완료 항목

- M1/M2 호밍의 최종 정지점은 실제 장비에서 확정해야 한다.
- M1/M2 수동 제어와 한계 처리는 이식된 작동감을 실제 장비에서 검증해야 한다.
- M3의 25,961/2,000 조작 비율과 27,000 STEP 원점 위치를 실제 장비에서 확인해야 한다.
- M3 수동 상한·하한은 아직 구현되지 않았다.
- M3 수동 운전 중 엔드스톱 방향 차단은 아직 구현되지 않았다.
- 수동 조작 중 `target_pos`가 과도하게 누적되는지 확인해야 한다.
- UART 상태 출력은 아직 구현하지 않았으며 현재는 SWV를 사용한다.
