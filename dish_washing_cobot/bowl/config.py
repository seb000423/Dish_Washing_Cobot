# ================= 실행 모드 & 로봇 설정 =================
VIRTUAL_MODE = False
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# ================= 그리퍼 (DO 방식 - 원본 release/grasp 동일) =================
GRIP_OPEN_CH = 2       # DO 2 = 열기 (release)
GRIP_CLOSE_CH = 3      # DO 3 = 닫기 (grasp)
GRIP_WAIT = 2.0        # [s] 원본 wait(2.0) 동일 적용

# ================= 모션 속도 / 가속도 (원본 파라미터 100% 동일) =================
VEL_FAST = 100.0
ACC_FAST = 150.0
VEL_SLOW = 50.0
ACC_SLOW = 100.0

# ================= 내부 나선 세척 파라미터 =================
SPIRAL_REVOLUTION = 3.0
SPIRAL_MAX_RADIUS = 15.0
SPIRAL_AXIS_DISTANCE = 5.0
BALL_APPROACH_DISTANCE = 140.0
SPIRAL_DOWN_DISTANCE = 140.0
SPIRAL_DURATION = 20.0

# ================= 외부 나선 세척 파라미터 =================
OUTER_SPIRAL_REVOLUTION = 3.0
OUTER_SPIRAL_MAX_RADIUS = 12.0
OUTER_SPIRAL_AXIS_DISTANCE = 0.0
OUTER_SPIRAL_DURATION = 20.0

# ================= 수세미 원주 구속 세척 파라미터 =================
SPONGE_ARC_ANGLE = 120.0

# ================= UI 토픽 설정 =================
PROCESS_TOPIC = "/dishwasher/process_bowl"
