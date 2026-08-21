# ERCP Control Firmware (코드설명은 docs의 md 참고)

STM32F446RE와 TMC2209 드라이버를 사용하는 5개 모터 제어 펌웨어다.

현재 통합 버전은 부팅 후 M1·M2·M3를 동시에 호밍하고, 호밍 성공 후 M1~M5 수동 제어에 진입한다. M1·M2는 피드백 엔코더로 기계적 양쪽 끝과 수동 안전 범위를 확인하고, M3는 GPIO 엔드스톱을 기준으로 원점을 설정한다.

> 현재 개발 브랜치: `0820_integrated_control`
>
> 기준 날짜: 2026-08-20
>
> 상태: 실제 장비 파라미터 검증 및 M3 수동 한계 구현 진행 중

## 주요 기능

- STM32F446RE, 84 MHz system clock, TMC2209 스테퍼 드라이버 5개 제어
- 사용자 입력 엔코더 4개, M1·M2 피드백 엔코더 2개
- M1·M2·M3 동시 호밍
- M1·M2 피드백 기반 수동 상·하한 차단(각도는 원준님이 지정해주십니다)
- M1·M2, M3, M4·M5 수동 제어 로직 분리
- M4 단독, M5 단독, M4·M5 동시 모드
- ADC 가변저항을 이용한 동시 모드 M5 속도 비율 조정

## 하드웨어와 개발 환경

| 항목 | 설정 |
|---|---|
| MCU | STM32F446RETx |
| Board | NUCLEO-F446RE 기반 |
| System clock | 84 MHz |
| STM32CubeMX | 6.15.0 |
| STM32Cube firmware | STM32Cube FW_F4 V1.28.3 |
| Motor driver | TMC2209 x5 |
| Motor microstep | 16 |
| 입력 엔코더 | 2048 PPR, quadrature x4 |
| 디버깅 | SWV ITM Data Console |

## 엔코더 연결

| 구분 | 타이머 | 역할 | 타이머 폭 |
|---|---|---|---:|
| Encoder 1 | TIM1 | M1 사용자 조작 | 16비트 |
| Encoder 2 | TIM2 | M2 사용자 조작 | 32비트 |
| Encoder 3 | TIM3 | M3 사용자 조작 | 16비트 |
| Encoder 4 | TIM4 | M4/M5 사용자 조작 | 16비트 |
| Encoder 5 | TIM5 | M1 피드백 | 32비트 |
| Encoder 6 | TIM8 | M2 피드백 | 16비트 |

Encoder 5와 Encoder 6은 사용자 조작용이 아니다. M1과 M2의 호밍 및 수동 한계 감시에 사용한다. 코드상 Encoder 6은 TIM6이 아니라 TIM8에 연결된다.

## 부팅과 제어 진입 순서

```text
HAL 및 주변장치 초기화
-> 모든 모터 드라이버 비활성화
-> TIM5/TIM8 피드백 엔코더 시작
-> M1/M2/M3 TMC2209 초기화
-> M1/M2/M3 동시 호밍
-> 호밍 성공 확인
-> M4/M5 TMC2209 초기화
-> TIM1~TIM4 사용자 입력 엔코더 시작
-> M12, M3, M45 수동 제어 반복
```

호밍 또는 필수 드라이버 초기화가 실패하면 모든 드라이버를 비활성화하고 수동 제어에 진입하지 않는다.

## 축별 호밍 방식

### M1/M2

M1과 M2는 TIM5/TIM8 피드백 엔코더로 실제 움직임을 확인한다.

```text
첫 번째 hard stop 동시 탐색
-> 반대쪽 hard stop 동시 탐색
-> 실제 피드백 encoder span 측정
-> 측정 span과 기준 각도로 최종 복귀량 계산
-> 최종 위치로 동시 복귀
-> 현재 위치를 0으로 설정
-> 수동 운전 안전 범위 생성
```

### M3

M3는 GPIO 엔드스톱을 사용한다.

```text
엔드스톱 방향으로 이동
-> 엔드스톱 검출
-> 반대 방향으로 지정 STEP만큼 backoff
-> 엔드스톱 해제 확인
-> 현재 위치를 0으로 설정
```

현재 M3 backoff는 27,000 STEP이며 실제 장비에서 최종 확인해야 한다.

## 수동 제어 그룹

### M1/M2

- Encoder 1 → M1
- Encoder 2 → M2
- 기존 검증 코드의 조작감을 반영한 위치 추종 방식
- TIM5/TIM8 피드백을 이용한 양방향 한계 차단
- 100%/300% 모드와 관계없이 고정 100% gain 사용

### M3

- Encoder 3 → M3
- M1/M2와 분리된 조작 비율, 속도 및 가속도 사용
- 수동 상한·하한과 엔드스톱 방향 차단은 아직 구현되지 않음

### M4/M5

Encoder 4 버튼으로 다음 모드를 순환한다.

```text
M4 단독 -> M5 단독 -> M4/M5 동시 -> M4 단독
```

- 단독 모드: Encoder 4 위치 입력 추종
- 동시 모드: M4 기본 속도와 ADC 가변저항 비율로 M5 속도 결정
- PA5 버튼의 100%/300% gain 적용

## 현재 주요 파라미터

### 수동 조작 비율

```text
M1: 95 / 26
M2: 171 / 181
M3: 25961 / 2000
M4: 1 / 1
M5: 1 / 1
```

각 비율에는 공통 변환값 `3200/8192`와 축별 방향 부호가 추가로 적용된다.

### 수동 속도

```text
M1/M2 최대 속도: 51200 SPS
M1/M2 가속도:    160000 SPS²
M3 최대 속도:    51200 SPS
M3 가속도:       160000 SPS²
M4/M5 최대 속도: 51200 SPS
M4/M5 가속도:    160000 SPS²
```

### M3 호밍

```text
엔드스톱 탐색 속도: 6000 SPS
최대 탐색 거리:     64000 STEP
원점 backoff:        27000 STEP
backoff 속도:        6000 SPS
```

## 디버깅

현재 애플리케이션 상태 출력은 UART/PuTTY가 아니라 SWV를 사용한다.

```text
STM32CubeIDE -> SWV ITM Data Console
Core Clock: 84 MHz
Stimulus Port: 0
```

주기적으로 다음 상태를 확인할 수 있다.

- M1/M2 피드백 위치와 안전 범위
- M1/M2 limit 상태
- M3 `current_pos`와 `target_pos`
- M4/M5 명령 위치
- Encoder 4 선택 모드
- M4/M5 동시 모드 속도 비율

## 프로젝트 구조

```text
ERCPver3/
├── Core/
│   ├── Inc/
│   ├── Src/
│   │   ├── main.c
│   │   └── tmc2209_uart.c
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

## 문서

- [펌웨어 전체 설명](docs/FIRMWARE_GUIDE.md): `main.c`를 실제 코드 순서대로 설명한다.
- [제어 수정 위치 안내](docs/CONTROL_TUNING_GUIDE.md): 원하는 동작을 바꿀 때 검색할 매크로와 함수를 정리한다.
- [2026-07-30 호밍 개발 기록](HOMING_WORKLOG_2026-07-30.md): 이전 M2/M3 동시 호밍 개발 과정을 보존한다.

README는 현재 프로젝트 상태를 요약한다. 코드의 세부 동작과 파라미터 수정 방법은 위 두 문서를 기준으로 한다.

## STM32CubeIDE에서 열기

1. 저장소를 clone하거나 내려받는다.
2. STM32CubeIDE에서 `File -> Import -> Existing Projects into Workspace`를 선택한다.
3. 저장소 루트를 프로젝트 경로로 지정한다.
4. Debug configuration으로 빌드한다.
5. ST-LINK로 보드에 다운로드한다.
6. SWV ITM Data Console을 84 MHz, Port 0으로 설정한다.

CubeMX에서 코드를 다시 생성하기 전에 Git diff를 확인해야 한다. 사용자 코드는 가능한 한 `USER CODE BEGIN/END` 보호 구간에 유지한다.

## 안전상 주의사항

- `current_pos`는 실제 motor shaft 위치가 아니라 지금까지 출력한 STEP 펄스 기준 위치다.
- motor stall, step loss 또는 기구 slip이 발생하면 `current_pos`와 실제 위치가 달라질 수 있다.
- M1/M2 수동 한계는 피드백 엔코더를 사용하지만 M3에는 일반 운전용 위치 피드백이 없다.
- 호밍 방향, 최대 탐색 거리, hard stop 판정값은 기구에 직접 힘을 가할 수 있는 안전 파라미터다.
- 실제 장비 시험 시 전원을 즉시 차단할 수 있는 상태에서 한 번에 하나의 파라미터만 변경한다.

## 남은 작업

- M1/M2 호밍 최종 정지점 실기 확정
- M1/M2 수동 조작감 및 한계 처리 실기 검증
- M3 `25961/2000` 조작 비율 검증
- M3 27,000 STEP 원점 위치 검증
- M3 수동 상한·하한 결정 및 구현
- M3 수동 운전 중 엔드스톱 방향 차단 구현
- 수동 조작 중 `target_pos` 과도 누적 여부 확인
- 필요 시 UART 상태 출력 구조 별도 설계

## 개발 기록

- `0730_double_homing`: 피드백 엔코더 기반 2축 양방향 hard-stop 호밍 개발
- `0820_integrated_control`: M1/M2/M3 동시 호밍과 M1~M5 수동 제어 통합, 설명 문서 추가
