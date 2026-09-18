"""식기 세척 메인 노드.

컨트롤 박스 디지털 입력(버튼)을 감시하다가 버튼이 눌리면 해당 식기의
세척 작업을 별도 스레드로 실행한다. 작업 중에도 정지 버튼이 동작한다.

상태: IDLE(대기, BTN_RESET=home) / RUN(작업 중, 정지만 받음) / STOP(리셋 대기).

노드를 dsr_api / dishwashing_main 둘로 나눈 이유: DSR_ROBOT2 함수는
내부적으로 rclpy 전역 executor로 응답을 받는데, 두 스레드가 동시에
호출하면 겹쳐 들어가 'generator already executing'이 난다. 그래서
dsr_api는 DR_init 등록만 하고 spin하지 않고(작업 스레드의 모션 호출
전용), dishwashing_main은 전용 SingleThreadedExecutor로 토픽/버튼을
처리한다. 버튼 읽기도 DSR_ROBOT2를 거치지 않고 자체 서비스 클라이언트로
call_async하므로 RUN 중에도 버튼이 계속 읽힌다. 정지는
<controller>.request_abort() 하나로 처리하며, move_stop(DR_QSTOP)을
call_async로 쏘고 motion_guard가 이후 모션을 WorkAborted로 막는다.

실행:  ros2 run dish_washing_cobot main_node
"""

import threading
import time
import traceback

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String

from dsr_msgs2.srv import GetCtrlBoxDigitalInput
from dsr_msgs2.msg import RobotState

import DR_init

from plate import config as cfg

DR_init.__dsr__id = cfg.ROBOT_ID
DR_init.__dsr__model = cfg.ROBOT_MODEL


# ================= 상태 =================
STATE_INIT = "INIT"   # setup 중. 명령을 받지 않는다
STATE_IDLE = "IDLE"
STATE_RUN = "RUN"
STATE_DONE = "DONE"   # 작업 정상 완료. 1회 발행 후 곧바로 IDLE 로 전환
STATE_STOP = "STOP"

# 상태 토픽 — UI 가 버튼 활성/비활성을 판단하는 근거.
# TRANSIENT_LOCAL(latched) 이라 UI 가 늦게 접속해도 현재 상태를 받는다.
TOPIC_STATE = "/dishwasher/state"

# ================= 버튼 (컨트롤 박스 디지털 입력) =================
BTN_PLATE = 13        # 접시 세척 시작
BTN_BOWL = 14         # 그릇 세척 시작
BTN_CUP = 15          # 컵 세척 시작
BTN_RESET = 16        # RUN: 정지 / STOP: 해제 / IDLE: 그리퍼 열고 HOME

BUTTON_POLL = 0.05    # [s] 버튼 확인 주기
IO_TIMEOUT = 0.5      # [s] 입력 서비스 응답 대기 한도 (폴백 경로)

# 드라이버 상태 토픽 — 모션 중에도 계속 발행되므로 버튼 읽기 1순위.
# 이름이 다르면 `ros2 topic list | grep dsr01` 로 확인해 수정할 것.
TOPIC_ROBOT_STATE = "state"    # 상대 이름 -> /dsr01/state
DIN_STALE = 0.5       # [s] 캐시가 이보다 오래되면 서비스 폴백으로 전환

# ================= 진단 로그 =================
HEARTBEAT_EVERY = 40      # 폴링 N회마다 살아있음 로그 (0.05*40 = 2초)

# ================= 명령 토픽 =================
# 버튼과 외부 UI 가 같은 진입점을 쓴다. 버튼 폴링은 눌림을 감지해
# 이 토픽에 발행만 하고, 실제 실행/정지 판단은 구독 콜백(_on_cmd)의
# 상태머신이 한다. 외부 UI 도 같은 토픽에 발행하면 된다.
#   ros2 topic pub --once /dishwasher/cmd std_msgs/msg/String "data: plate"
TOPIC_CMD = "/dishwasher/cmd"
CMD_PLATE = "plate"
CMD_BOWL = "bowl"
CMD_CUP = "cup"
CMD_STOP = "stop"      # RUN 중 정지
CMD_RESET = "reset"    # STOP 해제
CMD_HOME = "home"      # IDLE 에서 그리퍼 열고 HOME 복귀 (복구 동작)

# ================= 진행 단계 토픽 =================
TOPIC_PLATE = "/dishwasher/process_plate"
TOPIC_BOWL = "/dishwasher/process_bowl"
TOPIC_CUP = "/dishwasher/process_cup"

STEP_IDLE = 0         # 작업 중이 아님
STEP_DONE = {"plate": 9, "bowl": 7, "cup": 8}  # 식기별 작업 완료 신호
STEP_DONE_HOLD = 2.0  # [s] 완료 신호 유지 시간 — 이 동안 0.2초
                      #     주기로 반복 발행되므로 UI가 놓칠 수 없다
PUBLISH_PERIOD = 0.2  # [s] 진행 단계 발행 주기


class DishwashingMainNode(Node):
    """버튼 입력 -> 식기 세척 작업 실행. 상태머신으로 관리한다."""

    def __init__(self):
        super().__init__("dishwashing_main", namespace=cfg.ROBOT_ID)

        self.state = STATE_INIT
        self._ready = False        # setup 완료 전에는 명령을 받지 않는다
        self._worker = None        # 작업 스레드
        self._active_job = None    # 현재 RUN 중인 작업 키 ("plate"/"bowl"/"cup")
        self.plate = None          # attach_controllers() 에서 생성
        self.bowl = None           # attach_controllers() 에서 생성
        self.cup = None            # attach_controllers() 에서 생성

        # ---- 상태 토픽 (latched — UI 가 늦게 붙어도 현재 상태 수신) ----
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._pub_state = self.create_publisher(String, TOPIC_STATE, latched)
        self._publish_state()

        # ---- 서비스 클라이언트 (메인 노드 소유 = 전용 executor가 응답 처리) ----
        # 상대 이름이라 네임스페이스가 붙어 /dsr01/io/... 로 풀린다
        self._cli_din = self.create_client(
            GetCtrlBoxDigitalInput, "io/get_ctrl_box_digital_input")

        # ---- 진행 단계 발행 ----
        # 식기별 토픽에 현재 단계(대기=0, 작업=1~8, 완료=9)를 타이머로 상시 발행.
        # 이벤트성이 아니라 방송이라 UI가 늦게 붙어도 놓치지 않는다.
        self._steps = {"plate": STEP_IDLE, "bowl": STEP_IDLE,
                       "cup": STEP_IDLE}
        self._pubs = {
            "plate": self.create_publisher(Int32, TOPIC_PLATE, 10),
            "bowl": self.create_publisher(Int32, TOPIC_BOWL, 10),
            "cup": self.create_publisher(Int32, TOPIC_CUP, 10),
        }
        self._pub_timer = self.create_timer(PUBLISH_PERIOD, self._on_pub_timer)

        # ---- 드라이버 상태 토픽 (버튼 읽기 1순위) ----
        # 드라이버는 모션 서비스 처리 중엔 다른 서비스 응답을 못 주지만
        # 상태 토픽 발행은 계속하므로, 여기서 캐시해두면 RUN 중에도 버튼이 읽힌다.
        self._din_state = None     # ctrlbox_digital_input 캐시 (list)
        self._din_stamp = 0.0
        self._sub_state = self.create_subscription(
            RobotState, TOPIC_ROBOT_STATE, self._on_robot_state, 10)

        # ---- 명령 토픽 (버튼/외부 UI 공용 진입점) ----
        self._pub_cmd = self.create_publisher(String, TOPIC_CMD, 10)
        self._sub_cmd = self.create_subscription(
            String, TOPIC_CMD, self._on_cmd, 10)

        # ---- 버튼 폴링 (일반 스레드) ----
        self._prev = {BTN_PLATE: False, BTN_BOWL: False,
                      BTN_CUP: False, BTN_RESET: False}
        self._btn_lock = threading.Lock()   # _prev 를 폴링/워커가 함께 만짐
        self._last_raw = {}
        self._shutdown = threading.Event()
        self._btn_thread = None

        self.get_logger().info(
            f"Node created. poll={BUTTON_POLL}s "
            f"buttons={sorted(self._prev.keys())}")

    # ================= 초기화 =================
    def attach_controllers(self, dsr_node):
        """식기 컨트롤러를 생성한다.

        DR_init.__dsr__node 등록이 끝난 뒤에 호출해야 한다.
        컨트롤러에는 dsr_node 를 넘긴다 (DSR 자체 spin 경로와 일관).
        """
        self.get_logger().info("attach_controllers: importing controllers...")
        from plate.controller import PlateController
        from bowl.controller import BowlController
        from cup.controller import CupController

        self.plate = PlateController(
            dsr_node, on_step=lambda n: self._set_step("plate", n))
        self.bowl = BowlController(
            dsr_node, on_step=lambda n: self._set_step("bowl", n))
        self.cup = CupController(
            dsr_node, on_step=lambda n: self._set_step("cup", n))
        self.get_logger().info(f"Main node ready. state={self.state}")

    def setup_robot(self):
        """툴/TCP 설정 후 홈으로. 버튼을 받기 전에 한 번 수행한다.

        executor.spin() 이 시작된 뒤에 별도 스레드에서 호출된다.
        (버튼 읽기가 call_async 라 executor 가 돌고 있어야 응답이 온다)
        """
        self.get_logger().info("setup_robot: 서비스 대기")
        if not self._cli_din.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("io/get_ctrl_box_digital_input 서비스 없음")
            return False

        self.get_logger().info("setup_robot: plate.setup() 호출")
        if not self.plate.setup():
            self.get_logger().error("Robot setup failed")
            return False

        # BowlController는 원본 bowl.py와 동일하게 run_bowl_task() 내부에서
        # 초기 자세와 속도를 설정하므로 별도의 setup() 메서드를 요구하지 않는다.
        self.get_logger().info("setup_robot: bowl은 작업 시작 시 자체 초기화")

        # CupController는 plate와 같은 툴/TCP(Tool Weight / GripperDA_v1)를
        # 쓰지만, bowl 작업(bowl/food_check)이 툴/TCP를 Tool Weight_2FG로
        # 바꿔놓고 끝나므로 run_cup_task() 첫 줄에서 자기 setup()을 다시 건다.
        self.get_logger().info("setup_robot: cup은 작업 시작 시 자체 setup 수행")

        self.get_logger().info("setup_robot: move_home()")
        self.plate.move_home()

        self.get_logger().info("setup_robot: 버튼 초기값 동기화")
        if not self._sync_buttons():
            self.get_logger().error(
                "디지털 입력을 읽지 못했다. 포트 번호 / 서비스 이름 확인 필요")
            return False

        self._ready = True
        self.get_logger().info(f"초기 입력 상태: {self._last_raw}")
        if self._din_state is not None:
            self.get_logger().info("버튼 입력 소스: 상태 토픽 (모션 중에도 정지 버튼 동작)")
        else:
            self.get_logger().warn(
                "버튼 입력 소스: 서비스 폴백 — 상태 토픽이 안 잡힌다. "
                "모션 중 정지 버튼이 늦거나 무시될 수 있다. "
                "`ros2 topic list | grep dsr01` 로 토픽 이름 확인 후 "
                "TOPIC_ROBOT_STATE 를 수정할 것")
        self._set_state(STATE_IDLE)
        self.get_logger().info("Waiting for button...")
        return True

    # ================= 상태 발행 =================
    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self._pub_state.publish(msg)

    def _set_state(self, new):
        """상태 전환 + 상태 토픽 발행. 상태 변경은 반드시 이걸 거친다."""
        old = self.state
        self.state = new
        self._publish_state()
        self.get_logger().info(f"state: {old} -> {new}")

    # ================= 진행 단계 발행 =================
    def _set_step(self, job, step):
        """컨트롤러가 부르는 콜백(작업 스레드에서 호출됨). job: 'plate'/'bowl'."""
        self._steps[job] = step
        self.get_logger().info(f"[step:{job}] {step}")
        self._publish_job(job)

    def _publish_job(self, job):
        msg = Int32()
        msg.data = self._steps[job]
        self._pubs[job].publish(msg)

    def _on_pub_timer(self):
        # 항상 발행 — 모든 식기의 진행 단계와 상태를 0.2초마다 상시 방송한다.
        # (state 는 latched 이기도 하지만, 주기 발행을 겹쳐 두면
        #  UI 구독 방식과 무관하게 항상 최신 값을 받는다)
        for job in self._pubs:
            self._publish_job(job)
        self._publish_state()

    # ================= 정지 =================
    def stop_task(self):
        """진행 중인 작업을 중단시킨다.

        <controller>.request_abort() 가 abort 플래그를 세우고 motion/move_stop
        (DR_QSTOP) 을 call_async 로 쏜다. 응답을 기다리지 않으므로
        폴링 스레드/콜백 어디서 불러도 executor 와 충돌하지 않는다.
        이후의 모든 모션 호출은 common.motion_guard 가 막는다(정지
        플래그는 식기 종류와 무관하게 전역 공유).
        """
        if self.state != STATE_RUN:
            self.get_logger().info(
                f"stop_task 무시 (state={self.state}, RUN 아님)")
            return

        self._set_state(STATE_STOP)
        controller = getattr(self, self._active_job)
        controller.request_abort()

    def reset(self):
        if self.state != STATE_STOP:
            self.get_logger().info(
                f"reset 무시 (state={self.state}, STOP 아님)")
            return

        if self._worker is not None and self._worker.is_alive():
            self.get_logger().warn(
                "작업 스레드가 아직 정리 중이다. 잠시 후 다시 누를 것")
            return

        if self._active_job is not None:
            getattr(self, self._active_job).clear_abort()
        self._set_state(STATE_IDLE)

    # ================= 버튼 =================
    def _on_robot_state(self, msg):
        """드라이버 상태 토픽 수신 — 디지털 입력을 캐시한다."""
        try:
            self._din_state = list(msg.ctrlbox_digital_input)
            self._din_stamp = time.time()
        except Exception as e:
            self.get_logger().warn(f"[state] 입력 필드 파싱 실패: {e!r}")

    def _read_input(self, ch):
        """디지털 입력 1채널 읽기. 실패하면 None.

        1순위: 상태 토픽 캐시. 모션 중에도 갱신되므로 RUN 중 정지
               버튼이 여기서 잡힌다.
        2순위: 서비스 call_async 폴백. 드라이버가 모션 서비스를 처리
               하는 동안에는 응답이 밀려 타임아웃이 잦다(IDLE 은 정상).
        """
        snap = self._din_state
        if snap is not None and (time.time() - self._din_stamp) < DIN_STALE:
            idx = ch - 1        # DI13 -> index 12. echo 로 실측 확인할 것
            if 0 <= idx < len(snap):
                raw = snap[idx]
                if raw in (0, 1, True, False):
                    self._last_raw[ch] = int(bool(raw))
                    return bool(raw)
            self.get_logger().warn(
                f"[io] 상태 토픽 값 이상 ch={ch} idx={idx} "
                f"len={len(snap)} raw={snap[idx] if 0 <= idx < len(snap) else '-'}")
            return None

        return self._read_input_srv(ch)

    def _read_input_srv(self, ch):
        """서비스 폴백 경로."""
        req = GetCtrlBoxDigitalInput.Request()
        req.index = int(ch)

        try:
            future = self._cli_din.call_async(req)
        except Exception as e:
            self.get_logger().error(f"[io] call_async({ch}) 예외: {e!r}")
            return None

        done = threading.Event()
        future.add_done_callback(lambda f: done.set())
        if not done.wait(IO_TIMEOUT):
            future.cancel()
            # RUN 중에는 드라이버가 모션 서비스에 잡혀 있어 예상된
            # 상황이므로 debug 로만 남긴다 (로그 도배 방지)
            msg = f"[io] port {ch} 응답 없음 ({IO_TIMEOUT}s)"
            if self.state == STATE_RUN:
                self.get_logger().debug(msg)
            else:
                self.get_logger().warn(msg)
            return None

        res = future.result()
        if res is None:
            self.get_logger().warn(f"[io] port {ch} 결과 None")
            return None
        if not getattr(res, "success", True):
            self.get_logger().warn(f"[io] port {ch} success=False")
            return None

        raw = getattr(res, "value", None)
        # 정상 반환은 0/1 뿐. 에러 코드(-1 등)를 눌림으로 오인하면
        # 작업이 혼자 시작되므로 반드시 걸러낸다.
        if raw not in (0, 1, True, False):
            self.get_logger().warn(f"[io] port {ch} 비정상 값 {raw!r} -> 무시")
            return None

        self._last_raw[ch] = int(bool(raw))
        return bool(raw)

    def _sync_buttons(self):
        """현재 입력 상태를 기준으로 잡는다(작업 직후 오인식 방지)."""
        ok = True
        with self._btn_lock:
            for ch in self._prev:
                cur = self._read_input(ch)
                if cur is None:
                    ok = False
                    continue
                self._prev[ch] = cur
        return ok

    def start_button_thread(self):
        self._btn_thread = threading.Thread(
            target=self._button_loop, name="button_poll", daemon=True)
        self._btn_thread.start()

    def _button_loop(self):
        """버튼 폴링 본체. RUN 중에도 계속 돈다 (정지 버튼 수신용)."""
        log = self.get_logger()
        log.info("[poll] 버튼 폴링 스레드 시작")
        n = 0

        while rclpy.ok() and not self._shutdown.is_set():
            time.sleep(BUTTON_POLL)

            if not self._ready:
                continue

            n += 1
            try:
                with self._btn_lock:
                    for ch in list(self._prev):
                        cur = self._read_input(ch)
                        if cur is None:
                            continue

                        prev = self._prev[ch]
                        self._prev[ch] = cur

                        if cur and not prev:
                            log.info(f"[btn] ▲ RISING ch={ch} "
                                     f"state={self.state}")
                            self._on_button(ch)
                        elif prev and not cur:
                            log.debug(f"[btn] ▼ FALLING ch={ch}")

                if n % HEARTBEAT_EVERY == 0:
                    worker = ("alive" if self._worker and
                              self._worker.is_alive() else "-")
                    log.info(f"[poll] #{n} state={self.state} "
                             f"raw={self._last_raw} worker={worker}")
            except Exception:
                log.error(f"[poll] 폴링 중 예외:\n{traceback.format_exc()}")

        log.info("[poll] 버튼 폴링 스레드 종료")

    def _on_button(self, ch):
        """버튼 눌림 -> 명령 토픽 발행. 판단은 _on_cmd 가 한다.

        버튼 16 은 상태에 따라 의미가 다르므로 여기서 명령어를 고른다.
          RUN  -> stop  (정지)
          STOP -> reset (정지 해제)
          그 외(IDLE 등) -> home (그리퍼 열고 HOME 복귀)
        예전에는 IDLE 에서 눌러도 reset 이 나가 "STOP 아님"으로 무시됐다.
        외부 UI 는 명령어를 직접 쓴다.
        """
        cmd_map = {BTN_PLATE: CMD_PLATE, BTN_BOWL: CMD_BOWL,
                   BTN_CUP: CMD_CUP}
        if ch == BTN_RESET:
            if self.state == STATE_RUN:
                cmd = CMD_STOP
            elif self.state == STATE_STOP:
                cmd = CMD_RESET
            else:
                cmd = CMD_HOME
        else:
            cmd = cmd_map.get(ch)
            if cmd is None:
                return

        self.get_logger().info(f"Button {ch} -> cmd '{cmd}'")
        msg = String()
        msg.data = cmd
        self._pub_cmd.publish(msg)

    def _on_cmd(self, msg):
        """명령 토픽 수신 (버튼 발행분 + 외부 UI 발행분 공용)."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"[cmd] '{cmd}' (state={self.state})")

        if not self._ready:
            self.get_logger().warn("  ignored (setup 전)")
            return

        if cmd == CMD_STOP:
            self.stop_task()
            return

        if cmd == CMD_RESET:
            self.reset()
            return

        # ---- 작업 시작 명령 ----
        if self.state == STATE_RUN:
            self.get_logger().info("  ignored (running)")
            return
        if self.state == STATE_STOP:
            self.get_logger().info("  ignored (stopped, send 'reset')")
            return
        if self.state != STATE_IDLE:
            self.get_logger().info(f"  ignored (state={self.state})")
            return

        # IDLE
        if cmd == CMD_PLATE:
            self._start("plate", self.plate.run_plate_task)
        elif cmd == CMD_BOWL:
            self._start("bowl", self.bowl.run_bowl_task)
        elif cmd == CMD_CUP:
            self._start("cup", self.cup.run_cup_task)
        elif cmd == CMD_HOME:
            self._start_home()
        else:
            self.get_logger().warn(f"  unknown command '{cmd}'")

    # ================= 홈 복귀 (복구 동작) =================
    def _start_home(self):
        """그리퍼 열기 + HOME 복귀를 별도 스레드로 실행한다.

        식기 작업이 아니라 "정리" 동작이라 진행 단계 토픽(1~8)은 쓰지 않는다.
        모션이므로 반드시 워커 스레드에서 돌린다 — _on_cmd 는 executor
        콜백 안이고, 거기서 DSR 모션을 부르면 응답을 못 받아 멈춘다.
        """
        if self._worker is not None and self._worker.is_alive():
            self.get_logger().warn(
                "이전 작업 스레드가 아직 살아있다. 홈 복귀를 시작하지 않는다")
            return

        # 홈 복귀 모션은 PlateController 경로(plate.gripper / plate.move_home)를
        # 그대로 쓴다. 그래서 RUN 중 정지 버튼도 기존 경로로 정상 동작한다
        # (stop_task 가 getattr(self, self._active_job) 로 컨트롤러를 찾는다).
        self._active_job = "plate"
        self._set_state(STATE_RUN)
        self.get_logger().info("start 'home' (그리퍼 열기 + HOME 복귀)")

        self._worker = threading.Thread(
            target=self._run_home, name="home", daemon=True)
        self._worker.start()

    def _run_home(self):
        """홈 복귀 스레드 본체."""
        from common.motion_guard import WorkAborted
        from DSR_ROBOT2 import set_digital_output
        from cup.config import GRIP_WIDE_OPEN_CH

        log = self.get_logger()
        try:
            # 직전 정지의 abort 플래그가 남아 있으면 첫 모션에서 바로 끊긴다
            self.plate.clear_abort()

            # 컵 공정이 쓰는 넓은 열기(DO1)가 켜져 있을 수 있어 먼저 끈다.
            # plate.gripper 는 DO2/DO3 만 건드리므로 DO1 은 남아 있게 된다.
            set_digital_output(GRIP_WIDE_OPEN_CH, 0)

            log.info("[home] 그리퍼 열기")
            self.plate.gripper.open()

            log.info("[home] HOME 자세로 이동")
            self.plate.move_home()

            log.info("[home] 완료")
        except WorkAborted:
            log.warn("[home] 정지 요청으로 중단됨")
        except Exception:
            log.error(f"[home] 실패:\n{traceback.format_exc()}")
        finally:
            self._sync_buttons()
            if self.state == STATE_RUN:
                self._set_state(STATE_IDLE)
                log.info("Waiting for button...")
            else:
                log.warn(f"state stays {self.state} (send 'reset')")
            self._active_job = None

    # ================= 작업 실행 =================
    def _start(self, name, target):
        """작업을 별도 스레드로 시작한다."""
        if self._worker is not None and self._worker.is_alive():
            self.get_logger().warn(
                "이전 작업 스레드가 아직 살아있다. 새 작업을 시작하지 않는다")
            return

        self._active_job = name
        self._set_state(STATE_RUN)
        self.get_logger().info(f"start '{name}'")

        self._worker = threading.Thread(
            target=self._run, args=(name, target), daemon=True)
        self._worker.start()

    def _run(self, name, target):
        """작업 스레드 본체. 끝나면 상태를 되돌린다. name: 'plate'/'bowl' (진행 토픽 키)."""
        from common.motion_guard import WorkAborted

        self.get_logger().info(f"[job] '{name}' 스레드 시작")
        ok = False
        try:
            target()
            ok = True
            self.get_logger().info(f"'{name}' complete")
        except WorkAborted:
            self.get_logger().warn(f"'{name}' aborted")
        except Exception:
            self.get_logger().error(
                f"'{name}' error:\n{traceback.format_exc()}")
        finally:
            self._sync_buttons()

            if self.state == STATE_RUN:
                if ok:
                    # 완료 신호는 "유지 구간" — STEP_DONE_HOLD 동안 상시
                    # 발행 타이머가 식기별 완료값을 0.2초마다 계속 내보낸다.
                    # (여기는 워커 스레드라 sleep 해도 폴링/executor 는 돈다)
                    self._set_state(STATE_DONE)
                    self._steps[name] = STEP_DONE[name]
                    self._publish_job(name)
                    time.sleep(STEP_DONE_HOLD)

                self._steps[name] = STEP_IDLE
                self._publish_job(name)
                self._set_state(STATE_IDLE)
                self.get_logger().info("Waiting for button...")
            else:
                self._steps[name] = STEP_IDLE
                self._publish_job(name)
                self.get_logger().warn(
                    f"state stays {self.state} (send 'reset')")
            self._active_job = None


def main(args=None):
    rclpy.init(args=args)

    # 1) DSR 전용 노드. DR_init에 등록만 하고 절대 spin하지 않는다.
    dsr_node = rclpy.create_node("dsr_api", namespace=cfg.ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    # 2) 통신용 메인 노드. 전용 executor로 spin한다.
    #    (rclpy.spin(node)를 쓰면 "전역" executor를 쓰게 되는데, 그건
    #     DSR 내부 spin과 겹쳐 generator already executing이 난다)
    node = DishwashingMainNode()
    node.attach_controllers(dsr_node)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # setup은 spin이 돈 뒤에 해야 한다 — 버튼 읽기(call_async)의 응답을
    # executor가 처리해 주기 때문. 그래서 별도 스레드에서 부트한다.
    def _boot():
        try:
            if node.setup_robot():
                node.start_button_thread()
            else:
                node.get_logger().error("setup 실패 — 종료")
                rclpy.shutdown()
        except Exception:
            node.get_logger().error(f"boot 예외:\n{traceback.format_exc()}")
            rclpy.shutdown()

    threading.Thread(target=_boot, name="boot", daemon=True).start()

    try:
        node.get_logger().info("spin 시작 (전용 executor)")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    except Exception:
        node.get_logger().error(f"Error:\n{traceback.format_exc()}")
    finally:
        node._shutdown.set()
        executor.shutdown()
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()