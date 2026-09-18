"""컵 공정 설정값. 로직을 건드리지 않고 동작을 바꾸려면 이 파일을 먼저 본다.

원본(`cup_config.py`)의 값은 그대로 유지하고 이름만 프로젝트 규칙에
맞췄다. 가장 크게 바뀐 것은 그리퍼 채널 이름이다 — 원본은 메서드 이름과
실제 동작이 뒤집혀 있었는데(open() 이 파지), plate/bowl 과 같은 의미로
정리했다. 채널 번호(DO1/DO2/DO3)와 배선은 원본과 동일하다.
"""

# ================= 실행 모드 & 로봇 설정 =================
VIRTUAL_MODE = False
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# plate/config.py 와 **같은 이름**이다. 컵 좌표도 이 툴/TCP 로 티칭됐다.
#
# 참고: bowl/food_check.py 만 "Tool Weight_2FG" / "2FG_TCP" 를 직접
# 하드코딩해 두고 있다(팀원 원본 코드). 그래서 그릇 작업 뒤에는 툴/TCP 가
# 그쪽으로 바뀐 채 남는다 — 컵은 run_cup_task 첫 줄에서 아래 값으로 다시
# 걸기 때문에 영향받지 않는다.
TOOL_NAME = "Tool Weight"
TCP_NAME = "GripperDA_v1"

# ================= 그리퍼 (DO 방식, 3단 개폐) =================
# DO1 = 넓게 열기(약 110 mm, 정렬용)
# DO2 = 일반 열기(약 105 mm, 컵 해제)
# DO3 = 닫기(컵 파지)
#
# 원본 cup 코드는 Gripper.open() 이 실제로는 파지(DO3)였다. plate/bowl 과
# 뜻이 반대라 통합 시 사고가 나므로 이름을 통일했다:
#   close() = 파지(DO3) / open() = 해제(DO2) / wide_open() = 정렬(DO1)
GRIP_WIDE_OPEN_CH = 1
GRIP_OPEN_CH = 2
GRIP_CLOSE_CH = 3
GRIP_WAIT = 0.10               # [s] 그리퍼 동작 기본 대기

# 뒤집기 전 손잡이 세척의 방향 정렬 — 컵을 바닥에 둔 채 DO1로 넓게 연 뒤
# J6 를 30도씩 세 번 회전하며 매번 짧게 파지한다.
ALIGN_ROTATION_STEP_DEG = 30.0
ALIGN_ROTATION_STEPS = 3
ALIGN_WIDE_OPEN_WAIT = 0.10    # [s]
ALIGN_CLOSE_WAIT = 0.50        # [s]

# ================= 모션 속도 / 가속도 =================
VEL_J = 40.0
ACC_J = 40.0
VEL_J_SLOW = 20.0
ACC_J_SLOW = 20.0

VEL_X = 100.0
ACC_X = 100.0
VEL_X_SLOW = 40.0
ACC_X_SLOW = 40.0

# ================= 음식물(잔여물) 판정 =================
# 실측: 빈 컵 약 -2.9 N, 음식물 든 컵 약 -3.6 ~ -3.9 N.
# 평균 Fz 가 임계값 이하이면 잔여물 있음으로 본다(BASE 기준 측정).
FOOD_FZ_THRESHOLD = -3.2
FOOD_FORCE_SAMPLES = 50
FOOD_FORCE_SAMPLE_INTERVAL = 0.05   # [s]
FOOD_FORCE_SETTLE = 2.0             # [s] 측정 전 진동 안정화

FOOD_TILT_VEL = 50.0
FOOD_TILT_ACC = 40.0
FOOD_SHAKE_COUNT = 3
FOOD_SHAKE_VEL = 300.0
FOOD_SHAKE_ACC = 250.0
FOOD_RETURN_VEL = 60.0
FOOD_RETURN_ACC = 50.0

# ================= 뒤집기 전 손잡이/아래쪽 겉면 세척 =================
# 같은 세척 위치에서 서로 다른 두 방향으로 파지해 TOOL RZ 왕복을 한 번씩.
PRE_FLIP_OUTER_RZ_AMP_DEG = 30.0
PRE_FLIP_OUTER_PERIOD = 2.0         # [s]
PRE_FLIP_OUTER_ATIME = 0.5          # [s]
PRE_FLIP_OUTER_REPEAT = 1

# ================= 뒤집기 후 재파지 정렬 =================
REGRIP_ALIGN_ROTATION_DEG = 90.0
REGRIP_ALIGN_WAIT = 0.65            # [s]

# ================= 내부 / 외부 주기 회전 세척 =================
INNER_PERIODIC_RZ_AMP_DEG = 50.0
INNER_PERIODIC_PERIOD = 2.0         # [s]
INNER_PERIODIC_ATIME = 0.5          # [s]
INNER_PERIODIC_REPEAT = 3

OUTER_PERIODIC_RZ_AMP_DEG = 70.0
OUTER_PERIODIC_PERIOD = 6.0         # [s]
OUTER_PERIODIC_ATIME = 1.0          # [s]
OUTER_PERIODIC_REPEAT = 2

# ================= 컵 내부 힘 / 순응 제어 =================
# TOOL +Z 로 수세미를 누른 채 RZ 왕복. plate 와 달리 힘 지령을 유지한 채
# 세척한다(접촉 감지 단계 없음).
USE_FORCE_CONTROL = True
INNER_FORCE_N = 13.0                # [N] 미확정 — 팀 합의값 7N 여부 확인 필요
INNER_FORCE_SIGN = 1.0              # +1: TOOL +Z. 반대로 밀리면 -1.0
FORCE_RAMP = 0.5                    # [s]
FORCE_SETTLE = 0.2                  # [s]
FORCE_RELEASE = 0.3                 # [s]
INNER_STIFFNESS = [700.0, 700.0, 30.0, 100.0, 100.0, 100.0]

# ================= 최종 거치대 배치 =================
# 상대 이동 2회로 거치대 위까지 간 뒤, 현재 자세에서 BASE -Z 로 고정 거리
# 하강한다. 접촉 감지가 없으므로 거치대 높이가 바뀌면 충돌하거나 컵이
# 공중에서 놓인다. 원본 주석의 150 mm 는 실제 값과 달라 삭제했다.
FINAL_PLACE_DESCENT_MM = 40.0       # 미확정 — 실측 후 확정할 것
FINAL_PLACE_DESCENT_VEL = 30.0
FINAL_PLACE_DESCENT_ACC = 30.0
FINAL_PLACE_GRIPPER_WAIT = 0.15     # [s]