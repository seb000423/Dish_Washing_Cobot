# dish_washing_cobot — 식기 세척 로봇 제어 패키지

`dish_washing_cobot`은 Doosan M0609을 이용해 접시, 그릇, 컵 세척 공정을 실행하는 ROS2 Python 패키지입니다.

이 패키지는 다음 역할을 담당합니다.

- 컨트롤 박스 물리 버튼 입력 감시
- ROS2 명령 토픽 수신
- 식기 종류별 컨트롤러 실행
- `INIT · IDLE · RUN · DONE · STOP` 상태 관리
- 식기별 진행 단계 발행
- 작업 중 정지와 재시작 제어

[저장소 전체 소개로 돌아가기](../README.md)

---

## 패키지 구조

```text
dish_washing_cobot/
├── dish_washing_cobot/
│   ├── __init__.py
│   └── main_node.py          # ROS2 메인 노드와 상태머신
├── plate/
│   ├── config.py             # 접시 속도, 힘, 강성, 세척 파라미터
│   ├── controller.py         # PlateController
│   ├── food_check.py         # 무게 확인과 음식물 배출
│   ├── gripper.py            # 접시용 그리퍼 제어
│   ├── path_generator.py     # 세척 경로 생성
│   ├── waypoints.py          # 접시 Teaching Position
│   └── README.md
├── bowl/
│   ├── config.py             # 그릇 동작 파라미터
│   ├── controller.py         # BowlController
│   ├── food_check.py         # 음식물 확인과 배출
│   ├── gripper.py            # 그릇용 그리퍼 제어
│   ├── radial_movec_drl.py   # DRL 기반 나선·원호 모션 실행
│   └── waypoints.py          # 그릇 Teaching Position
├── cup/
│   ├── config.py             # 컵 동작 파라미터
│   ├── controller.py         # CupController
│   ├── food_check.py         # 컵 무게 확인과 배출
│   ├── gripper.py            # 컵용 그리퍼 제어
│   └── waypoints.py          # 컵 Teaching Position
├── common/
│   └── motion_guard.py       # 공통 abort 플래그와 모션 래퍼
├── nodes/
│   └── plate_test_node.py    # 접시 모듈 테스트 노드
├── resource/
├── test/
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

`plate`, `bowl`, `cup` 디렉터리는 일반 Python 모듈입니다. import만으로 로봇이 움직이지 않으며, 메인 노드가 각 컨트롤러의 `run_*_task()` 메서드를 호출할 때 실제 동작이 시작됩니다.

---

## 실행 구조

`DSR_ROBOT2` 모션 함수와 ROS2 통신 콜백이 같은 executor에 동시에 진입하면 executor 충돌이 발생할 수 있습니다. 이를 피하기 위해 `main_node.py`는 두 노드와 별도 작업 스레드를 사용합니다.

```text
main
 ├─ dsr_api 노드 (DR_init 등록 전용, spin 안 함)
 └─ dishwashing_main 노드 (SingleThreadedExecutor로 spin)
     ├─ /dishwasher/cmd 구독, /dishwasher/state·공정 단계 발행, 버튼 입력 처리
     └─ 작업 스레드 → Plate/Bowl/Cup Controller → DSR_ROBOT2 모션 함수 → dsr_api
```

- `dsr_api`: `DR_init.__dsr__node`에 등록되는 로봇 API 전용 노드입니다. 별도 executor에서 spin하지 않습니다.
- `dishwashing_main`: 명령, 상태, 진행 단계, 버튼 입력을 처리합니다.
- 작업 스레드: 긴 로봇 모션 시퀀스를 실행합니다.
- 버튼 폴링 스레드: 작업 중에도 정지 버튼을 감지합니다.

---

## 상태머신

| 상태 | 의미 | 허용되는 주요 명령 |
|---|---|---|
| `INIT` | Tool/TCP, HOME, 버튼 입력 초기화 중 | 없음 |
| `IDLE` | 작업 대기 | `plate`, `bowl`, `cup`, `home` |
| `RUN` | 식기 세척 또는 HOME 복귀 수행 중 | `stop` |
| `DONE` | 정상 완료 신호 유지 구간 | 내부 처리 후 자동 `IDLE` |
| `STOP` | 정지 요청으로 작업 중단 | `reset` |

```text
INIT --(setup 완료)--> IDLE --(plate/bowl/cup/home)--> RUN --(정상 완료)--> DONE --> IDLE
                                                         RUN --(stop)--> STOP --(reset)--> IDLE
```

`/dishwasher/state`는 `TRANSIENT_LOCAL` QoS를 사용합니다. UI가 늦게 실행되어도 마지막 상태를 받을 수 있으며, 메인 노드는 상태와 진행 단계를 주기적으로 다시 발행합니다.

---

## 명령 인터페이스

### 명령 토픽

```text
/dishwasher/cmd
```

메시지 타입:

```text
std_msgs/msg/String
```

| 명령 | 동작 조건 | 기능 |
|---|---|---|
| `plate` | `IDLE` | 접시 세척 시작 |
| `bowl` | `IDLE` | 그릇 세척 시작 |
| `cup` | `IDLE` | 컵 세척 시작 |
| `stop` | `RUN` | 현재 모션 정지 및 시퀀스 중단 |
| `reset` | `STOP` | abort 플래그 해제 후 `IDLE` 복귀 |
| `home` | `IDLE` | 넓은 열기 출력을 해제하고 그리퍼를 연 뒤 HOME 복귀 |

```bash
ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: plate"
ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: bowl"
ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: cup"
ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: stop"
ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: reset"
ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: home"
```

`RUN` 중 다른 식기 명령은 무시됩니다. `STOP` 상태에서는 `reset` 이외의 작업 시작 명령이 무시됩니다.

---

## 물리 버튼

메인 노드는 컨트롤 박스 Digital Input을 Rising Edge 방식으로 감시합니다.

| 입력 | 역할 |
|---|---|
| DI 13 | 접시 세척 시작 |
| DI 14 | 그릇 세척 시작 |
| DI 15 | 컵 세척 시작 |
| DI 16 | `RUN`: 정지 / `STOP`: 해제 / `IDLE`: 그리퍼 열기 + HOME |

버튼 입력은 우선 로봇 상태 토픽의 `ctrlbox_digital_input` 값을 사용합니다. 상태 토픽이 일정 시간 갱신되지 않으면 `io/get_ctrl_box_digital_input` 서비스로 폴백합니다.

실제 드라이버의 상태 토픽 이름이 다르면 `main_node.py`의 `TOPIC_ROBOT_STATE`를 수정합니다.

---

## 공정 진행 토픽

| 토픽 | 타입 | 값 |
|---|---|---|
| `/dishwasher/process_plate` | `std_msgs/msg/Int32` | 접시 단계 `0~8`, 완료 유지 값 `9` |
| `/dishwasher/process_bowl` | `std_msgs/msg/Int32` | 그릇 단계 `0~7` |
| `/dishwasher/process_cup` | `std_msgs/msg/Int32` | 컵 단계 `0~7`, 완료 유지 값 `8` |

비활성 식기는 `0`을 계속 발행합니다. 현재 구현에서 완료 여부는 `/dishwasher/state`의 `DONE` 상태와 함께 판단하는 것이 안전합니다.

### 접시 단계

| 단계 | 동작 |
|---:|---|
| 1 | 접시 파지 |
| 2 | 무게 확인 및 음식물 배출 |
| 3 | 세척 위치 1 — 원호 세척 |
| 4 | 세척 위치 2 — 직선 쓸기 |
| 5 | 접시 재파지 및 회전 |
| 6 | 회전 후 세척 위치 1 반복 |
| 7 | 회전 후 세척 위치 2 반복 |
| 8 | 건조 거치대 배치 |

접시는 그리퍼가 가린 영역을 세척하기 위해 중간에 재파지합니다. 볼록면은 사용자 좌표계 기준 원호 경로를 사용하고, 오목면은 툴 좌표계 기준 직선 쓸기 경로를 사용합니다.

### 그릇 단계

| 단계 | 동작 |
|---:|---|
| 1 | 그릇 파지 |
| 2 | 무게 확인 및 음식물 배출 |
| 3 | 그릇을 내려놓고 공 형태 도구로 내부 나선 세척 |
| 4 | 그릇 재파지 |
| 5 | 고정 수세미에서 원주 문지르기 |
| 6 | 툴 좌표계 기준 외부 나선 세척 |
| 7 | 건조 거치대 배치 |

### 컵 단계

| 단계 | 동작 |
|---:|---|
| 1 | 컵 파지 |
| 2 | 잔여물 판정 및 배출 |
| 3 | 손잡이 주변 겉면 세척 |
| 4 | 컵 뒤집기 및 재파지 |
| 5 | 컵 내부 세척 |
| 6 | 컵 외부 세척 |
| 7 | 최종 배치 |

---

## 정지 처리

모든 컨트롤러는 동일한 인터페이스를 제공합니다.

```python
controller.request_abort()
controller.clear_abort()
controller.stop_motion()
```

`request_abort()`는 두 작업을 함께 수행합니다.

1. `common.motion_guard`의 전역 abort 플래그 설정
2. `motion/move_stop` 서비스에 `DR_QSTOP` 비동기 요청

`common.motion_guard`가 감싼 모션 함수는 호출 전후에 abort 플래그를 확인합니다.

```text
movej / movel / movejx / movec / movesj / amovel / move_periodic ...
```

정지 요청이 들어오면 현재 모션을 끊고 `WorkAborted` 예외로 작업 스레드를 빠져나옵니다. 접촉 대기처럼 모션 함수가 아닌 반복문에도 abort 확인 지점이 포함되어야 합니다.

---

## 빌드

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select dish_washing_cobot
source install/setup.bash
```

등록된 실행 파일은 다음과 같습니다.

| 실행 파일 | Python 진입점 | 설명 |
|---|---|---|
| `main_node` | `dish_washing_cobot.main_node:main` | 전체 식기 세척 메인 노드 |
| `plate_test` | `nodes.plate_test_node:main` | 접시 모듈 테스트 노드 |

---

## 실행

### 1. 로봇 Bringup

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch m0609_rg2_bringup bringup.launch.py \
  mode:=real \
  host:=192.168.1.100 \
  model:=m0609
```

### 2. 메인 노드

```bash
cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dish_washing_cobot main_node
```

초기화가 정상적으로 완료되면 로봇이 HOME으로 이동한 뒤 `IDLE` 상태에서 명령을 기다립니다.

---

## 상태 및 토픽 확인

```bash
# 노드 확인
ros2 node list

# 상태 확인
ros2 topic echo /dishwasher/state

# 공정 단계 확인
ros2 topic echo /dishwasher/process_plate
ros2 topic echo /dishwasher/process_bowl
ros2 topic echo /dishwasher/process_cup

# 명령 토픽 정보
ros2 topic info /dishwasher/cmd -v

# Doosan 상태 토픽 확인
ros2 topic list | grep dsr01
```

---

## 실장 전 설정 항목

### 공통

- `plate/config.py`의 `ROBOT_ID`, `ROBOT_MODEL`
- 티치펜던트의 Tool/TCP 등록 이름
- 실제 로봇과 작업대에 맞는 각 `waypoints.py`
- 그리퍼 Digital Output 채널
- 실제 장비 기준 속도와 가속도

### 접시

- 빈 접시 무게와 음식물 판정 임계값
- 세척 위치별 힘 방향 부호
- 접근 힘, 접촉 판정값, 강성
- 원호 반지름과 반복 횟수
- 직선 쓸기 거리와 Y 오프셋

### 그릇

- 내부·외부 나선 반지름과 수행 시간
- 공 형태 도구와 고정 수세미 좌표
- 원주 문지르기 원호 각도
- 거치대 접근 자세

### 컵

- 컵 무게 판정 임계값
- 손잡이 주변 세척 자세
- 내부·외부 세척 회전 진폭과 반복 횟수
- 넓은 그리퍼 열기용 출력 채널
- 최종 거치대 접근·하강 거리

---

## 문제 해결

### `io/get_ctrl_box_digital_input 서비스 없음`

Doosan Bringup이 먼저 실행되었는지 확인합니다.

```bash
ros2 service list | grep get_ctrl_box_digital_input
```

로봇 ID가 `dsr01`이 아니라면 네임스페이스와 설정값을 확인합니다.

### 모션 중 정지 버튼이 늦게 반응함

메인 노드는 로봇 상태 토픽을 우선 사용합니다. 아래 명령으로 실제 상태 토픽 이름과 발행 여부를 확인합니다.

```bash
ros2 topic list | grep state
ros2 topic hz /dsr01/state
```

필요하면 `TOPIC_ROBOT_STATE`를 실제 토픽에 맞게 변경합니다.

### Tool/TCP 적용 실패

티치펜던트의 Tool/TCP 이름과 각 `config.py`의 이름이 일치하는지 확인합니다. Bowl 작업 후 다른 Tool/TCP가 활성화될 수 있으므로 Cup 작업은 시작 시 자체 설정을 다시 적용합니다.

### `generator already executing`

DSR 전용 노드를 별도 executor에 추가하거나, ROS 콜백 안에서 직접 긴 DSR 모션을 실행하지 않았는지 확인합니다. 현재 `main_node.py`의 두 노드·작업 스레드 구조를 유지해야 합니다.

### 정지 후 작업이 시작되지 않음

작업 스레드 종료 후 `reset` 명령을 보내야 합니다.

```bash
ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: reset"
```

---

## 안전 주의사항

- 초기 실장 시에는 속도, 힘, 가속도를 낮게 설정합니다.
- 첫 모션은 반드시 단일 단계 또는 테스트 노드로 검증합니다.
- 힘 제어 축과 부호는 실제 툴 장착 방향을 기준으로 확인합니다.
- 사용자 좌표계 원점과 방향이 코드의 가정과 일치해야 합니다.
- 수세미와 식기 사이의 과도한 접촉력에 대비해 비상정지를 준비합니다.
- `stop` 명령과 소프트웨어 가드는 산업용 안전장치나 하드웨어 비상정지를 대체하지 않습니다.