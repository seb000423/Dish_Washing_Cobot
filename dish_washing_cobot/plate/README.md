# plate/ — 접시 세척 모듈

접시(Plate) 파트 담당 코드. `dish_washing_cobot/main_node.py`가 `PlateController`를
생성해 사용하는 일반 Python 패키지이며, 그 자체로는 ROS Node가 아니다.
import만으로는 로봇이 움직이지 않고, 메서드를 호출할 때만 동작한다.

## 파일 구성

| 파일 | 역할 |
|---|---|
| `config.py` | 실험값·설정값 (속도, 힘, 강성, 세척 경로 파라미터 등) |
| `waypoints.py` | Teaching Position (실측 좌표) |
| `gripper.py` | 2-Finger Gripper 제어 (디지털 출력) |
| `food_check.py` | 무게 측정 → 음식물 유무 판정 → 배출(털기) |
| `controller.py` | `PlateController` — 파지/세척/회전/배치 전체 동작 |

## 동작 흐름 (`run_plate_task`)

```
HOME
 └─ [1] pick_plate()        접시 파지 (접근 → 하강 → grasp → 상승)
 └─ [2] food_check          Fz 50회 평균 → 음식물 있으면 배출(dispose_food)
 └─ [3] wash_face(위치1)     세척 위치1(앞면, 볼록) — wash_arcs
 └─ [4] wash_face(위치2)     세척 위치2(뒷면, 오목) — wash_sweeps
 └─ [5] rotate_plate()      재파지로 접시 180° 회전 (그리퍼가 가린 부분 커버)
 └─ [6] wash_face(위치1)     회전 후 반대쪽 면 재세척
 └─ [7] wash_face(위치2)     회전 후 반대쪽 면 재세척
 └─ [8] place_plate()       배치 경유 자세 → 접근 → release → 이탈
```

단계 번호는 `dish_washing_cobot/main_node.py`가 `/dishwasher/process_plate`
토픽으로 그대로 발행하는 값과 같다.

## 세척 방식 — 위치별로 고정

세척 위치는 **면 자체**가 아니라 펜던트에 등록한 **사용자 좌표계**로
구분한다 (`dish1` = 101). 원점은 접시 중심 = 수세미 접촉점.

- **위치1 (앞면, 볼록면) → `wash_arcs`**
  경로는 **유저 좌표계(dish1)** 기준 `movec`. 반지름 `WASH_RADII`
  (기본 `[25, 55]`mm) 동심 반원을 바깥→안쪽 순서로, 반지름마다
  `WASH_PASSES`회 왕복한다.
- **위치2 (뒷면, 오목면) → `wash_sweeps`**
  오목면에서 원호를 그리면 곡면이 수세미를 옆으로 밀어내 큰 힘이
  걸리므로 원호 대신 **직선 쓸기**를 쓴다. 이동·힘 모두 **툴
  좌표계** 기준. 원점 → 툴 Y 오프셋 이동 → 툴 Z로 바깥으로 쓸기
  (`amovel` 비동기 이동 + 0.05초 힘 폴링) → 지정 거리 도달/힘 이탈 시
  정지 → 수세미 반대 방향으로 `SWEEP_BACKOFF`만큼 빼서 접촉 해제 →
  시작 자세로 복귀 → 다음 Y 오프셋(`SWEEP_Y_OFFSETS`) 반복.

두 방식 모두 접근 단계는 공통이다: 힘/순응제어는 반드시 **툴
좌표계**로 걸어야 한다(힘제어 함수가 `ref` 인자를 안 받고
`set_ref_coord`로 지정한 전역 기준을 따르기 때문). 실측 결과 **툴
X축이 접시 법선**이고, 세척 위치에 따라 접시가 반대로 향하므로 미는
부호도 반대다 — 위치1은 툴 **−X**, 위치2는 툴 **+X**가 수세미 방향
(`config.py`의 `WASH1_FORCE_SIGN` / `WASH2_FORCE_SIGN`).

접근 흐름: `APPROACH_FORCE`(15N)로 접근 → `_wait_contact()`로 접촉
확인(0.05초 폴링, 최대 `CONTACT_TIMEOUT` 2초, 타임아웃이어도 경고만
남기고 진행) → 접촉되면 `release_force()`로 힘 지령만 해제하고
순응제어는 유지(접시 곡면이 자연히 눌러주므로 힘을 계속 줄 필요
없음) → 세척 진행 → 완료 후 `release_compliance_ctrl()` +
`set_ref_coord(DR_BASE)`로 복원.

강성(`WASH_STIFFNESS`, 툴 축 순서)은 축마다 다르게 준다: 힘 방향
축(툴 X)과 쓸기 진행 축(툴 Z)은 낮게(곡면을 따라 순응하도록), 옆
방향(툴 Y)은 높게(접시가 밀려나지 않도록).

## 정지(긴급정지) 처리

`controller.py`는 DSR 모션 함수(`movej`/`movel`/`movec`/`amovel`)를
`common.motion_guard`가 감싼 버전으로 가져다 쓴다. 정지 요청이 오면:

1. `request_abort()` — 전역 abort 플래그를 세우고 `stop_motion()`으로
   `motion/move_stop`(DR_QSTOP) 서비스를 호출해 현재 모션을 끊는다.
2. 가드된 모션 함수는 호출 전후로 플래그를 확인해, 정지 후 새 모션이
   시작되는 것과 끊긴 모션에서 남은 코드가 계속 실행되는 것을 모두
   막는다 (`common.motion_guard.WorkAborted` 예외로 탈출).
3. `_wait_contact()`처럼 모션 함수가 아닌 대기 루프에도 확인 지점을
   따로 심어뒀다.
4. 재시작 전에는 `clear_abort()`로 플래그를 지워야 한다.

## 알려진 미확정 사항

- `EMPTY_PLATE_WEIGHT` / `WEIGHT_THRESHOLD`(config.py) — 접시 자체
  무게 판정 값 미확정
- `SWEEP_CHECK_LEAVE` — 쓸기 중 힘 이탈로 조기 정지하는 기능. 실측
  임계값이 안 잡혀 현재 꺼져 있음(`False`), 접시 반지름만큼 끝까지
  쓸고 나옴
- `FORCE_LOG` — 힘 측정값을 임계값 튜닝용으로 로그에 남기는 진단
  플래그. 튜닝 끝나면 꺼도 됨
