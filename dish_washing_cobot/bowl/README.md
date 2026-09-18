# bowl/ — 그릇 세척 모듈

그릇(Bowl) 파트 코드. `dish_washing_cobot/main_node.py`가 `BowlController`를 생성해서
쓰는 일반 Python 패키지이고, 그 자체로는 ROS Node가 아니다.

접시나 컵과 다르게 그릇은 안쪽이 오목하고 곡면이라 사각지대가 생기기
쉽다. 그래서 내부는 공 모양 수세미로 나선을 그리며 닦고, 가장자리는
고정 수세미에 대고 원주를 그리며 문지르고, 바닥/외벽은 툴 좌표계
기준 나선으로 마무리한다.

## 파일 구성

| 파일 | 역할 |
|---|---|
| `config.py` | 속도, 나선 반지름/시간 등 세척 파라미터, 그리퍼 DO 채널 |
| `waypoints.py` | 실측 Teaching Position (홈, 파지점, 볼 수세미, 고정 수세미 3점, 거치대) |
| `gripper.py` | 2-Finger 그리퍼 개폐 (DO2/DO3) |
| `food_check.py` | Fz 힘 센서 50회 평균으로 음식물 유무 판정 후 위아래로 털어 배출 |
| `radial_movec_drl.py` | ROS2 기본 `movec`가 지원하지 않는 `DR_MV_ORI_RADIAL` 자세를 쓰기 위한 네이티브 DRL 실행기 |
| `controller.py` | `BowlController` — 파지부터 거치까지 전체 시퀀스와 정지 가드 |

## 동작 흐름 (`run_bowl_task`)

```text
HOME (그리퍼 열기)
 └─ [1] pick_bowl()          그릇 파지
 └─ [2] food_check           Fz 50회 평균 → 음식물 있으면 위아래 3회 털어 배출
 └─ [3] put_bowl + amove_spiral   그릇 내려놓고 볼 수세미로 내부 나선 세척
 └─ [4] pick_bowl()          세척된 그릇 재파지
 └─ [5] scrub_bowl_on_sponge 고정 수세미에 대고 원주 문지르기
 └─ [6] spiral_scrub_bowl    툴 좌표계 기준 외부 나선 세척
 └─ [7] place_bowl_on_rack   건조 거치대 배치
```

단계 번호는 `/dishwasher/process_bowl` 토픽으로 그대로 발행된다. 매
단계 진입 전 `common.motion_guard`의 abort 플래그를 확인한다.

## 세척 방식

**내부 나선 (`put_bowl` → `amove_spiral`)** — 그릇 안쪽 깊은 곡면은
수세미를 평면으로 대기 어려워서, 로봇이 공 모양 수세미(`POS_BALL`)를
직접 집어서 닦는다. 그릇 중심 상단 140mm(`SPIRAL_DOWN_DISTANCE`)에서
내려가면서, 베이스 좌표계(`DR_BASE`) 기준으로 `amove_spiral`을 돌려
중심에서 바깥으로 퍼지는 나선을 그린다. 3회전, 최대 반지름 15mm,
바닥 방향 5mm 하강, 20초.

**고정 수세미 원주 문지르기 (`scrub_bowl_on_sponge`)** — 그릇 가장자리를
고정 수세미(`POS_SPONGE`)에 대고 문지르려면 TCP가 수세미 곡면 중심을
계속 바라보면서 원호를 그려야 한다(`DR_MV_ORI_RADIAL`). 실측한 세 점
(`POS_SPONGE`, `POS_SPONGE_POS1`, `POS_SPONGE_POS2`)으로 원호가 놓인
평면의 법선과 중심 좌표를 벡터 연산으로 구하고, 거기서 ±60°/±120°
경유점을 만들어 정방향·역방향으로 왕복한다.

ROS2 기본 `movec` 서비스는 자세 구속 모드(`ori`)와 블렌딩(`app_type`)
옵션을 지원하지 않아서, `RadialMovecRunner`가 `DR_MV_ORI_RADIAL` /
`DR_MV_ORI_TEACH`가 들어간 네이티브 DRL 코드를 문자열로 조립해
`drl_start`로 실행한다.

```python
# controller.py 내 DRL 생성 예시
drl_code = (
    "set_singularity_handling(DR_AVOID)\r\n"
    f"movec({drl_position('posx', forward_via)}, {drl_position('posx', forward_end)}, "
    f"vel={cfg.VEL_SLOW:.6f}, acc={cfg.ACC_SLOW:.6f}, radius=0.0, ref=DR_BASE, "
    "ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_RADIAL, app_type=DR_MV_APP_NONE)\r\n"
    # 복귀 + 역방향 원호 왕복 코드가 이어짐
)
```

**외부 나선 (`spiral_scrub_bowl`)** — 고정 수세미 문지르기가 끝나면
밑면/외벽 잔여물을 닦기 위해 툴 좌표계(`DR_TOOL`) 기준 나선을 한 번
더 돈다. `POS_SPIRAL_APPROACH` → `POS_SPIRAL_CENTER`로 내려가서 TCP
Z축 기준 12mm 반지름으로 3회전, 20초.

## 주요 파라미터 (`config.py`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `GRIP_OPEN_CH` / `GRIP_CLOSE_CH` | 2 / 3 | 그리퍼 열기/닫기 DO 채널 |
| `GRIP_WAIT` | 2.0s | 그리퍼 개폐 후 대기 시간 |
| `VEL_FAST` / `ACC_FAST` | 100 / 150 | 허공 이동용 고속 |
| `VEL_SLOW` / `ACC_SLOW` | 50 / 100 | 파지·접촉·세척용 저속 |
| `SPIRAL_MAX_RADIUS` | 15mm | 내부 나선 최대 반지름 |
| `SPIRAL_AXIS_DISTANCE` | 5mm | 내부 나선 하강 거리 |
| `SPIRAL_DURATION` | 20s | 내부 나선 소요 시간 |
| `OUTER_SPIRAL_MAX_RADIUS` | 12mm | 외부 나선 최대 반지름 |
| `OUTER_SPIRAL_DURATION` | 20s | 외부 나선 소요 시간 |
| `SPONGE_ARC_ANGLE` | 120° | 고정 수세미 원호 각도 |
| `BALL_APPROACH_DISTANCE` | 140mm | 볼 수세미 파지/반납 접근 거리 |
| `PROCESS_TOPIC` | `/dishwasher/process_bowl` | 진행 단계 발행 토픽 |
