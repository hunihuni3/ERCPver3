# M2/M3 자동 호밍 개발 기록

- 작성일: 2026-07-30
- 대상 저장소: `hunihuni3/ERCPver3`
- 비교 기준: [idranoutof1d/ERCProbot](https://github.com/idranoutof1d/ERCProbot)
- 대상 파일: `Core/Src/main.c`

## 1. 7/30 작업 개요

스코프를 장착하지 않은 상태에서 M2와 M3 회전체가 양쪽 기계식 엔드스탑을 확인하고, 측정된 전체 가동 범위를 기준으로 결합 자세에 자동 정렬되도록 호밍 코드를 구현했다.

주요 작업 내용:

1. 코드 이해
   - M2/M3 팔로워 엔코더 매핑 확인
     - M2: TIM8, 16-bit
     - M3: TIM5, 32-bit
   - 엔코더 누적 지연(`lag`)을 이용한 하드스탑 검출
   - 첫 번째 스탑과 두 번째 스탑 사이의 실제 엔코더 범위 측정
2. M2/M3 순차 호밍을 동시 호밍으로 변경
3. 측정 범위와 CAD 각도 비율을 이용한 원점 자동 계산
4. 호밍 실패 시 일반 제어 진입을 막는 안전 처리 추가
5. 임시 진단 및 디버깅 코드 제거

전체 호밍 순서:

```text
M2/M3 첫 번째 엔드스탑 동시 탐색
→ 반대 방향으로 이동
→ M2/M3 두 번째 엔드스탑 동시 탐색
→ 양쪽 스탑 사이 엔코더 범위 측정
→ 측정 범위로 원점 복귀량 계산
→ M2/M3 원점 동시 복귀
→ 정렬 위치를 논리 위치 0으로 설정
```

원점 자동 계산:

```text
첫 번째 스탑 위치 A 기록
→ 두 번째 스탑 위치 B 기록
→ span_counts = |B - A|
→ home_counts = span_counts × 원점각 / 전체각
→ B에서 반대 방향으로 home_counts만큼 이동
→ 해당 위치를 0으로 설정
```
 
검증 결과:

 - M2/M3 양쪽 엔드스탑 검출 성공
 - 여러번 시도해도 안정적 동작 확인
 - 두 축 모두 측정 기반 원점 복귀  확인

## 2. 엔코더를 활용한 양쪽 엔드스탑 기반 원점 정렬

정확한 환산 관계:

```text
microsteps = encoder counts × 25 / 64
encoder counts = microsteps × 64 / 25
```

실측 매핑:

| 축 | 팔로워 엔코더 | 타이머 폭 | `esign` |
|---|---|---:|---:|
| M2 | TIM8 | 16-bit | -1 |
| M3 | TIM5 | 32-bit | -1 |

### 양쪽 스탑 측정과 원점 계산

각 축은 첫 번째 엔드스탑을 검출한 후 반대 방향으로 회전해 두 번째 엔드스탑에 도달한다. 이 과정에서 명령 microstep 수와 실제 엔코더 카운트를 기록한다.

하드스탑 접촉 중에는 탈조가 발생할 수 있으므로, 원점 계산에는 명령 스텝 수보다 실제 엔코더 span을 우선 사용한다.

```c
span_counts = abs(stop_b_counts - stop_a_counts);
home_counts = span_counts * return_angle / total_span_angle;
```

CAD 기준:

| 축 | 전체 범위 | 두 번째 스탑 기준 초기 복귀각 |
|---|---:|---:|
| M2 | 253.9° | 127.0° |
| M3 | 245.2° | 112.5° |

각도는 코드에서 0.1° 단위 정수로 표현한다.

```c
127.0° → 1270u
112.5° → 1125u
```

CAD 기준값으로 정렬했을 때 실제 기구에서 약간의 오차가 확인되어 다음과 같이 실기 조정했다.

```c
M2: 1390u  /* 139.0° */
M3: 1383u  /* 138.3° */
```

값을 늘리면 두 번째 엔드스탑에서 원점 방향으로 더 많이 이동하고, 줄이면 덜 이동한다. 향후 기구 감속비와 CAD 기준을 다시 확인하여 가능한 경우 `1270u`, `1125u`에 가깝게 정리할 예정이다.

### 실측 감속비

세 번의 반복 측정 결과:

| 축 | 평균 전체 span | 실효 모터→드럼 감속비 | 반복 범위 |
|---|---:|---:|---:|
| M2 | 약 20,781 counts | 약 3.5967:1 | 6 counts |
| M3 | 약 21,493 counts | 약 3.8520:1 | 2 counts |

문서 표기용 값:

```text
M2 = 3.597:1
M3 = 3.852:1
```

평상시 입력 노브 추종 코드에는 위 감속비의 역수를 적용했다.

```c
M2 gear_num / gear_den = 27803 / 100000
M3 gear_num / gear_den = 25961 / 100000
```

이 `mot[]` 비율은 평상시 입력 노브→모터 추종량에 사용된다. 양쪽 스탑 호밍의 원점 계산에는 고정 감속비 대신 매번 측정한 실제 `span_counts`를 사용한다.

### 엔드스탑 검출

TMC2209 StallGuard 대신 팔로워 엔코더의 누적 지연을 사용한다.

```text
lag = 명령 스텝으로 예상한 엔코더 카운트
      - 실제 누적 엔코더 카운트
```

주요 설정:

```c
HOME_APPROACH_V         = 800 microsteps/s
HOME_RETURN_V           = 800 microsteps/s
HOME_TRIP_LAG           = 80 counts
HOME_CONFIRM_WINDOWS    = 2
HOME_SECOND_ARM_COUNTS  = 6000 counts
```

방향 전환 직후 백래시와 엔코더 양자화 때문에 `lag=10~14`가 발생해 두 번째 스탑을 오검출하는 문제가 있었다. 다음과 같이 수정했다.

1. 첫 번째 스탑은 시작 직후 검출을 허용한다.
2. 두 번째 스탑은 최소 6000 counts 이동 전까지 검출하지 않는다.
3. 6000 counts에서 lag 기준을 다시 설정한다.
4. `lag >= 80`이 두 검사 구간 연속 발생해야 스탑으로 확정한다.

실제 하드스탑에서는 약 `lag=155~170`이 측정되어 정상 주행 오차와 충분히 분리됐다.

### M2/M3 동시 호밍

기존 순차 호밍을 듀얼축 blocking 처리 방식으로 변경했다.

- M2와 M3가 같은 시간축에서 STEP 펄스를 출력한다.
- 각 축의 엔코더, lag, 이동 한계 및 완료 상태를 독립적으로 관리한다.
- 한 축이 먼저 스탑에 도착하면 해당 축의 STEP만 중단한다.
- 다른 축은 자기 스탑에 도착할 때까지 계속 움직인다.
- 원점 복귀도 먼저 목표에 도착한 축만 정지한다.
- 한 축이라도 실패하면 전체 호밍을 실패 처리한다.
- 공유 통신선 충돌 방지를 위해 TMC2209 전류 설정은 순차 실행한다.

UART 단계 표기:

```text
simultaneous first-stop search
simultaneous second-stop search
simultaneous alignment return
```

현재 동시 호밍은 M2와 M3에 동일한 STEP 주파수를 사용한다. 기계 감속비가 다르기 때문에 두 드럼의 실제 각속도가 완전히 같지는 않다. 필요하면 추후 축별 `HOME_APPROACH_V` 또는 STEP 스케줄을 적용한다.

## 3. UART 안전 및 오류 표기법

### 정상 진행 로그

```text
[FW] HOME_REV=...
[HOME] simultaneous first-stop search
[HOME] M2 stop detected ...
[HOME] M3 stop detected ...
[HOME] simultaneous second-stop search
[HOME] M2 detector armed at ... counts
[HOME] M3 detector armed at ... counts
[HOME] M2 span=... ratio=... home=...
[HOME] M3 span=... ratio=... home=...
[HOME] simultaneous alignment return
[HOME] M2 return complete ...
[HOME] M3 return complete ...
[HOME] M2 aligned
[HOME] M3 aligned
```

### 오류 상태

| 상태 | UART 의미 |
|---:|---|
| 0 | 정상 완료 |
| 1 | 이동 한도 내 엔드스탑 미검출 |
| 2 | 엔코더 응답 또는 방향 오류 |
| 3 | 양쪽 스탑 사이 span 비정상 |
| 4 | 원점 복귀 실패 |
| 5 | TMC2209 전류 설정 실패 |

주요 오류 로그:

```text
[HOME] Mx no-stop ...
[HOME] Mx encoder-dir ...
[HOME] Mx span invalid ...
[HOME] Mx return failed ...
[HOME] Mx current setup failed
```

안전 설정:

- M2 최대 탐색 거리: 12000 microsteps
- M3 최대 탐색 거리: 12288 microsteps
- 정상 span 허용 범위: 12000~30000 counts
- 저전류 터치오프
  - M2: 175 mA
  - M3: 300 mA
- 원점 복귀 전 정상 전류 500 mA 복구
- M2 하드스탑 안착용 제한된 추가 STEP 적용
- 호밍 실패 시 모든 모터 드라이버 비활성화
- `knob_axes_homed`가 설정되지 않으면 일반 제어 진입 금지
- 호밍 완료 후 입력 엔코더를 재시드하여 급출발 방지

## 4. 디버깅 코드 제거

원본 [idranoutof1d/ERCProbot](https://github.com/idranoutof1d/ERCProbot) 대비 다음 디버깅 및 불필요 코드를 정리했다.

- `home_probe()` 제거
- `home_probe2()` 제거
- 임시 엔코더 매핑 진단 코드 제거
- 메인 루프의 TMC 진단 출력 블록 제거
- 사용하지 않는 M1 전용 매크로 제거
- 사용하지 않는 디버그 버퍼와 임시 변수 제거
- 사용하지 않는 지연 함수와 주석 처리된 이전 코드 제거
- `_write()`의 미사용 매개변수 경고 처리
- 모터 구조체의 런타임 상태 명시적 초기화
- 모터 초기화 대기 시간을 축당 1000 ms에서 10 ms로 단축
- 펌웨어 호밍 revision UART 출력 추가

남아 있는 후속 작업:

 M2/M3 원점 복귀각 최종 확정


