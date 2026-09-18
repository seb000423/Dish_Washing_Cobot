from datetime import datetime
from threading import Lock, RLock, Thread
import time

from flask import Flask, jsonify, render_template, request

app = Flask(__name__)
lock = RLock()

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int32, String
    from dsr_msgs2.srv import GetRobotState, SetRobotControl, SetRobotMode

    ROS2_AVAILABLE = True
except ImportError:
    rclpy = None
    MultiThreadedExecutor = None
    ReentrantCallbackGroup = None
    DurabilityPolicy = None
    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    Bool = None
    Empty = None
    Float64 = None
    Float64MultiArray = None
    Int32 = None
    String = None
    GetRobotState = None
    SetRobotControl = None
    SetRobotMode = None
    ROS2_AVAILABLE = False


DISH_TYPES = ["접시", "밥그릇", "컵"]
COMMAND_BY_DISH = {
    "접시": "plate",
    "밥그릇": "bowl",
    "컵": "cup",
}

ROBOT_MODE_MANUAL = 0
ROBOT_MODE_AUTO = 1
CONTROL_RECOVERY_SAFE_STOP = 4
CONTROL_RESET_RECOVERY = 7
ROBOT_STATE_STANDBY = 1
ROBOT_STATE_COLLISION = 5

PLATE_STEPS = [
    {
        "code": 0,
        "key": "plate_waiting",
        "name": "대기",
        "description": "작업 명령을 기다리는 상태입니다.",
    },
    {
        "code": 1,
        "key": "plate_grip",
        "name": "접시 파지",
        "description": "접시의 지정 위치로 이동하여 안전하게 파지합니다.",
    },
    {
        "code": 2,
        "key": "plate_discard",
        "name": "음식물 확인·배출",
        "description": "음식물 상태를 확인하고 필요한 경우 배출합니다.",
    },
    {
        "code": 3,
        "key": "plate_front_1",
        "name": "세척 위치 1 · 앞면",
        "description": "첫 번째 세척 위치에서 접시 앞면을 세척합니다.",
    },
    {
        "code": 4,
        "key": "plate_back_1",
        "name": "세척 위치 2 · 뒷면",
        "description": "두 번째 세척 위치에서 접시 뒷면을 세척합니다.",
    },
    {
        "code": 5,
        "key": "plate_rotate",
        "name": "접시 회전·재파지",
        "description": "남은 영역을 세척할 수 있도록 접시를 회전하고 다시 파지합니다.",
    },
    {
        "code": 6,
        "key": "plate_front_2",
        "name": "회전 후 위치 1 세척",
        "description": "접시 회전 후 첫 번째 위치의 남은 영역을 세척합니다.",
    },
    {
        "code": 7,
        "key": "plate_back_2",
        "name": "회전 후 위치 2 세척",
        "description": "접시 회전 후 두 번째 위치의 남은 영역을 세척합니다.",
    },
    {
        "code": 8,
        "key": "plate_store",
        "name": "접시 보관",
        "description": "세척을 마친 접시를 지정된 완료 위치에 배치합니다.",
    },
    {
        "code": 9,
        "key": "plate_complete",
        "name": "작업 완료",
        "description": "접시 세척 작업이 완료되었습니다.",
    },
]

BOWL_STEPS = [
    {
        "code": 0,
        "key": "bowl_waiting",
        "name": "대기",
        "description": "식기 선택과 작업 시작 명령을 기다립니다.",
    },
    {
        "code": 1,
        "key": "bowl_grip",
        "name": "식기 파지",
        "description": "선택한 식기의 지정 위치로 이동하여 안전하게 파지합니다.",
    },
    {
        "code": 2,
        "key": "bowl_food_discard",
        "name": "음식물 판정 및 배출",
        "description": "힘 센서 값을 측정하여 식기 내부 음식물 유무를 판정합니다.",
    },
    {
        "code": 3,
        "key": "bowl_inside_wash",
        "name": "내부 세척",
        "description": "볼수세미로 식기 내부를 세척합니다.",
    },
    {
        "code": 4,
        "key": "bowl_regrip",
        "name": "식기 재파지",
        "description": "외부 세척을 위해 식기를 다시 파지합니다.",
    },
    {
        "code": 5,
        "key": "bowl_outside_wash",
        "name": "외부 세척",
        "description": "외부 오염을 제거하기 위해 세척합니다.",
    },
    {
        "code": 6,
        "key": "bowl_bottom_wash",
        "name": "그릇 밑면 세척",
        "description": "그릇 아래를 세척합니다.",
    },
    {
        "code": 7,
        "key": "bowl_return",
        "name": "원위치 배치",
        "description": "세척이 끝난 식기를 지정된 완료 위치에 내려놓습니다.",
    },
]

CUP_STEPS = [
    {
        "code": 0,
        "key": "cup_waiting",
        "name": "대기",
        "description": "컵 작업 시작 명령을 기다립니다.",
    },
    {
        "code": 1,
        "key": "cup_grip",
        "name": "컵 파지",
        "description": "컵을 파지하고 잔여물 확인 위치로 이동합니다.",
    },
    {
        "code": 2,
        "key": "cup_residue_discard",
        "name": "잔여물 판정 및 배출",
        "description": "힘 센서로 잔여물을 판정하고, 감지된 경우 컵을 기울여 흔들어 배출합니다.",
    },
    {
        "code": 3,
        "key": "cup_handle_wash",
        "name": "손잡이 주변 세척",
        "description": "컵을 두 방향으로 재파지하면서 손잡이 주변과 컵 아래쪽 겉면을 세척합니다.",
    },
    {
        "code": 4,
        "key": "cup_flip_regrip",
        "name": "컵 뒤집기 및 재파지",
        "description": "컵을 내부 세척이 가능한 방향으로 뒤집고 다시 파지합니다.",
    },
    {
        "code": 5,
        "key": "cup_inside_wash",
        "name": "컵 내부 세척",
        "description": "컵에 원통형 수세미를 삽입하고 힘·순응 제어와 회전 동작으로 내부를 세척합니다.",
    },
    {
        "code": 6,
        "key": "cup_outside_wash",
        "name": "컵 외부 세척",
        "description": "컵 외부를 수세미 옆면에 접촉시켜 회전하며 세척합니다.",
    },
    {
        "code": 7,
        "key": "cup_complete",
        "name": "최종 배치 및 작업 완료",
        "description": "세척이 끝난 컵을 지정된 완료 위치에 배치하고 작업을 완료합니다.",
    },
]


def steps_for(dish):
    if dish == "접시":
        return PLATE_STEPS
    if dish == "컵":
        return CUP_STEPS
    return BOWL_STEPS


def waiting_step_key(dish):
    if dish == "접시":
        return "plate_waiting"
    if dish == "밥그릇":
        return "bowl_waiting"
    return "cup_waiting"


def complete_step_key(dish):
    if dish == "접시":
        return "plate_complete"
    if dish == "밥그릇":
        return "bowl_return"
    return "cup_complete"


def is_gripper_closed_text(value):
    """/dishwasher/gripper/state 문자열을 기준으로 그리퍼 닫힘 여부를 판단합니다."""
    text = str(value or "").strip().lower()
    if not text:
        return False

    # 열림 표현이 섞여 있으면 닫힘보다 우선해서 False 처리합니다.
    open_keywords = ["open", "opened", "열림", "열기", "열려", "release", "released"]
    if any(keyword in text for keyword in open_keywords):
        return False

    closed_keywords = [
        "close",
        "closed",
        "닫",
        "grip",
        "gripped",
        "holding",
        "hold",
        "파지",
        "잡힘",
        "clamp",
        "clamped",
    ]
    return any(keyword in text for keyword in closed_keywords)


status = {
    "selected_dish": "접시",
    "main_state": "INIT",
    "process_code": 0,
    "progress": 0,
    "current_step": "plate_waiting",
    "running": False,
    "paused": False,
    "emergency_stop": False,
    "collision_stop": False,
    "collision_alert": False,
    "recovery": {
        "active": False,
        "entering": False,
        "completing": False,
        "message": "충돌 감지 대기",
        "error": "",
    },
    "completed": {"접시": 0, "밥그릇": 0, "컵": 0},
    "message": "메인 노드의 상태 토픽을 기다리는 중입니다.",
    "ros": {
        "available": ROS2_AVAILABLE,
        "ready": False,
        "state_received": False,
        "process_plate_received": False,
        "process_bowl_received": False,
        "process_cup_received": False,
        "last_command": "-",
    },
    # 기존 화면의 센서 영역이 깨지지 않도록 기본값을 유지합니다.
    "sensor": {
        "fz": 0.0,
        "force_magnitude": 0.0,
        "force_all": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "food_detected": False,
        "gripper": "미연동",
        "gripper_closed": False,
        "gripper_safety_stop": False,
        "gripper_restart_pending": False,
        "robot_mode": "초기화",
        "connection": "ROS2 연결 대기",
        "topics": {
            "force_all": False,
            "force_magnitude": False,
            "gripper": False,
            "gripper_safety_stop": False,
            "process_plate": False,
            "process_bowl": False,
            "process_cup": False,
        },
    },
}

settings = {
    "food_threshold": -0.9,
    "sample_count": 50,
    "wash_1_count": 3,
    "wash_2_count": 2,
    "cup_inside_count": 4,
    "robot_speed": 300,
    "robot_acc": 300,
}

alarms = [
    {
        "time": datetime.now().strftime("%H:%M:%S"),
        "level": "info",
        "title": "시스템 시작",
        "message": "설거지 로봇 HMI가 시작되었습니다.",
        "status": "초기화",
    }
]


def add_alarm(level, title, message, alarm_status):
    alarms.insert(
        0,
        {
            "time": datetime.now().strftime("%H:%M:%S"),
            "level": level,
            "title": title,
            "message": message,
            "status": alarm_status,
        },
    )
    del alarms[30:]


def build_status():
    steps = steps_for(status["selected_dish"])
    index = next(
        (i for i, step in enumerate(steps) if step["key"] == status["current_step"]),
        0,
    )
    current = steps[index]
    return {
        **status,
        "dish_types": DISH_TYPES,
        "steps": steps,
        "current_step_index": index,
        "current_process": current["name"],
        "step_description": current["description"],
    }


class RosBridge:
    """UI와 dishwashing_main 사이의 3개 ROS2 토픽만 연결합니다."""

    def __init__(self):
        self.node = None
        self.executor = None
        self.thread = None
        self.command_pub = None
        self.gripper_restart_pub = None
        self.gripper_open_pub = None
        self.home_pub = None
        self.ready = False
        self.last_main_state = None
        self.last_process_code = 0
        self.run_had_nonzero_step = False
        self.completion_counted_for_run = False
        self.robot_state_client = None
        self.service_callback_group = None
        self.set_robot_mode_client = None
        self.set_robot_control_client = None
        self.robot_state_timer = None
        self.recovery_lock = Lock()
        self.robot_state_request_in_progress = False
        self.last_robot_state = None

        if not ROS2_AVAILABLE:
            print("[WARN] rclpy를 찾지 못했습니다. UI 미리보기만 가능합니다.")
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.node = rclpy.create_node("dish_robot_hmi_bridge")

            self.command_pub = self.node.create_publisher(
                String,
                "/dishwasher/cmd",
                10,
            )

            self.gripper_restart_pub = self.node.create_publisher(
                Empty,
                "/dishwasher/gripper/restart",
                10,
            )

            self.gripper_open_pub = self.node.create_publisher(
                Empty,
                "/dishwasher/gripper/open",
                10,
            )

            self.home_pub = self.node.create_publisher(
                Empty,
                "/dishwasher/robot/home",
                10,
            )

            state_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            self.node.create_subscription(
                String,
                "/dishwasher/state",
                self._state_callback,
                state_qos,
            )
            self.node.create_subscription(
                Int32,
                "/dishwasher/process_plate",
                self._process_plate_callback,
                10,
            )
            self.node.create_subscription(
                Int32,
                "/dishwasher/process_bowl",
                self._process_bowl_callback,
                10,
            )
            self.node.create_subscription(
                Int32,
                "/dishwasher/process_cup",
                self._process_cup_callback,
                10,
            )
            self.node.create_subscription(
                Float64MultiArray,
                "/dishwasher/force/all",
                self._force_all_callback,
                10,
            )
            self.node.create_subscription(
                Float64,
                "/dishwasher/force/magnitude",
                self._force_magnitude_callback,
                10,
            )
            self.node.create_subscription(
                String,
                "/dishwasher/gripper/state",
                self._gripper_callback,
                10,
            )
            self.node.create_subscription(
                Bool,
                "/dishwasher/gripper/safety_stop",
                self._gripper_safety_stop_callback,
                state_qos,
            )

            # 예전에 잘 되던 방식:
            # 같은 ROS 노드 안에서 0.5초 타이머로 /dsr01/system/get_robot_state를 조회합니다.
            self.service_callback_group = ReentrantCallbackGroup()
            self.robot_state_client = self.node.create_client(
                GetRobotState,
                "/dsr01/system/get_robot_state",
                callback_group=self.service_callback_group,
            )
            self.set_robot_mode_client = self.node.create_client(
                SetRobotMode,
                "/dsr01/system/set_robot_mode",
                callback_group=self.service_callback_group,
            )
            self.set_robot_control_client = self.node.create_client(
                SetRobotControl,
                "/dsr01/system/set_robot_control",
                callback_group=self.service_callback_group,
            )
            self.robot_state_timer = self.node.create_timer(
                0.5,
                self._request_robot_state,
                callback_group=self.service_callback_group,
            )

            self.executor = MultiThreadedExecutor(num_threads=4)
            self.executor.add_node(self.node)
            self.thread = Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
            self.ready = True

            with lock:
                status["ros"]["ready"] = True
                status["sensor"]["connection"] = "ROS2 연결 완료"

            self.node.get_logger().info(
                "HMI ROS2 연결 완료: 명령/상태/진행, 힘·그리퍼 토픽, "
                "로봇 상태 서비스"
            )
        except Exception as exc:
            print(f"[WARN] ROS2 초기화 실패: {exc}")

    def publish_command(self, command):
        command = str(command).strip().lower()
        if command not in {"plate", "bowl", "cup", "stop", "reset"}:
            return False
        if not self.ready or self.command_pub is None:
            return False

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        with lock:
            status["ros"]["last_command"] = command

        self.node.get_logger().info(f"/dishwasher/cmd 발행: {command}")
        return True



    def publish_gripper_restart(self):
        """그리퍼 안전 정지 상태에서만 Empty 재시작 토픽을 발행합니다."""
        if not self.ready or self.gripper_restart_pub is None:
            return False

        self.gripper_restart_pub.publish(Empty())
        with lock:
            status["sensor"]["gripper_restart_pending"] = True
        self.node.get_logger().warning(
            "/dishwasher/gripper/restart 발행: 그리퍼 Power Cycle 요청"
        )
        return True

    def publish_gripper_open(self):
        """그리퍼 닫힘 상태에서 Empty 열기 토픽을 발행합니다."""
        if not self.ready or self.gripper_open_pub is None:
            return False

        self.gripper_open_pub.publish(Empty())
        with lock:
            status["ros"]["last_command"] = "gripper_open"

        self.node.get_logger().info(
            "/dishwasher/gripper/open 발행: 그리퍼 열기 요청"
        )
        return True

    def publish_home(self):
        """IDLE 상태에서 홈 위치 이동 Empty 토픽을 발행합니다."""
        if not self.ready or self.home_pub is None:
            return False

        self.home_pub.publish(Empty())

        with lock:
            status["ros"]["last_command"] = "home"

        self.node.get_logger().info(
            "/dishwasher/robot/home 발행: 홈 위치 이동 요청"
        )
        return True

    def _call_service_sync(self, client, request, description, timeout=10.0):
        """Flask/작업 스레드에서 ROS2 서비스를 호출하고 결과를 기다립니다."""
        if not self.ready or self.node is None or client is None:
            raise RuntimeError("ROS2 서비스 연결이 준비되지 않았습니다.")

        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"{description} 서비스를 찾을 수 없습니다.")

        done = __import__("threading").Event()
        result_holder = {"response": None, "error": None}

        try:
            future = client.call_async(request)

            def complete_callback(completed_future):
                try:
                    result_holder["response"] = completed_future.result()
                except Exception as exc:
                    result_holder["error"] = exc
                finally:
                    done.set()

            future.add_done_callback(complete_callback)
        except Exception as exc:
            raise RuntimeError(f"{description} 요청 실패: {exc}") from exc

        if not done.wait(timeout):
            raise TimeoutError(f"{description} 응답 시간 초과")

        if result_holder["error"] is not None:
            raise RuntimeError(
                f"{description} 호출 실패: {result_holder['error']}"
            )

        response = result_holder["response"]
        if response is None or not getattr(response, "success", False):
            raise RuntimeError(f"{description} 요청이 거부되었습니다.")

        return response

    def set_robot_mode_sync(self, mode):
        request = SetRobotMode.Request()
        request.robot_mode = int(mode)
        return self._call_service_sync(
            self.set_robot_mode_client,
            request,
            f"set_robot_mode({mode})",
        )

    def set_robot_control_sync(self, control):
        request = SetRobotControl.Request()
        request.robot_control = int(control)
        return self._call_service_sync(
            self.set_robot_control_client,
            request,
            f"set_robot_control({control})",
        )

    def _publish_stop_burst(self, count=5, interval=0.12):
        """충돌 복구 전에 메인 작업 노드에 stop을 여러 번 발행합니다."""
        for index in range(count):
            ok = self.publish_command("stop")
            if self.node is not None:
                level = self.node.get_logger().info if ok else self.node.get_logger().warning
                level(f"[recovery] /dishwasher/cmd stop 발행 {index + 1}/{count}")
            time.sleep(interval)

    def _wait_for_main_state(self, target_state, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with lock:
                current_state = status.get("main_state")
            if current_state == target_state:
                return True
            time.sleep(0.05)
        return False

    def _force_ui_stop_state(self, message):
        """토픽 수신이 늦어도 UI는 STOP 상태로 고정합니다."""
        with lock:
            status["main_state"] = "STOP"
            status["running"] = False
            status["paused"] = False
            status["emergency_stop"] = True
            status["process_code"] = 0
            status["progress"] = 0
            status["current_step"] = waiting_step_key(status["selected_dish"])
            status["message"] = message
            status["sensor"]["robot_mode"] = "작업 정지"

    def _begin_collision_recovery_async(self):
        """상태 5 최초 감지 시 먼저 작업 stop 후 Manual + Recovery Safe Stop으로 전환합니다."""
        with self.recovery_lock:
            with lock:
                recovery = status["recovery"]
                if recovery["active"] or recovery["entering"]:
                    return
                recovery["entering"] = True
                recovery["completing"] = False
                recovery["error"] = ""
                recovery["message"] = "작업 STOP 처리 후 수동 복구 모드로 전환 중입니다."

            Thread(
                target=self._enter_collision_recovery_worker,
                daemon=True,
            ).start()

    def _enter_collision_recovery_worker(self):
        try:
            # 1) 제일 먼저 메인 작업 노드에 stop을 보냅니다.
            #    그래야 Recovery 해제 후 남아 있던 다음 movej/movel이 이어지지 않습니다.
            self._publish_stop_burst(count=5, interval=0.12)

            stop_confirmed = self._wait_for_main_state("STOP", timeout=3.0)

            if stop_confirmed:
                self.node.get_logger().warning(
                    "[recovery] /dishwasher/state STOP 확인 완료"
                )
            else:
                # 실제 토픽 수신이 늦어도 UI 조작은 STOP 기준으로 막습니다.
                self.node.get_logger().warning(
                    "[recovery] STOP 상태 수신 시간 초과: UI를 STOP으로 고정하고 복구를 계속합니다."
                )
                self._force_ui_stop_state(
                    "충돌 감지 후 stop 명령을 보냈습니다. 안전 복구 후 정지 리셋이 필요합니다."
                )

            # 2) 그 다음 로봇을 수동 복구 상태로 전환합니다.
            self.set_robot_mode_sync(ROBOT_MODE_MANUAL)
            self.set_robot_control_sync(CONTROL_RECOVERY_SAFE_STOP)

            with lock:
                status["collision_alert"] = True
                status["collision_stop"] = True
                status["recovery"]["active"] = True
                status["recovery"]["entering"] = False
                status["recovery"]["message"] = (
                    "작업은 STOP 처리되었습니다. 수동 복구 모드입니다. "
                    "로봇을 손으로 안전한 위치로 이동한 뒤 '복구 완료 · 작업 취소'를 누르세요."
                )
                status["message"] = status["recovery"]["message"]
                add_alarm(
                    "warning",
                    "작업 STOP 후 수동 복구 진입",
                    "/dishwasher/cmd stop 발행 후 Manual Mode와 CONTROL_RECOVERY_SAFE_STOP(4)을 적용했습니다.",
                    "수동 이동 가능",
                )

            self.node.get_logger().warning(
                "[recovery] stop 선처리 → Manual Mode → CONTROL_RECOVERY_SAFE_STOP(4) 적용 완료"
            )
        except Exception as exc:
            with lock:
                status["recovery"]["active"] = False
                status["recovery"]["entering"] = False
                status["recovery"]["error"] = str(exc)
                status["recovery"]["message"] = "수동 복구 모드 진입에 실패했습니다."
                add_alarm(
                    "critical",
                    "수동 복구 진입 실패",
                    str(exc),
                    "확인 필요",
                )
            self.node.get_logger().error(f"[recovery] 진입 실패: {exc}")

    def start_complete_collision_recovery(self):
        """UI 요청을 즉시 반환하고 복구 완료 절차는 작업 스레드에서 수행합니다."""
        with self.recovery_lock:
            with lock:
                recovery = status["recovery"]
                if recovery["entering"]:
                    raise RuntimeError("아직 수동 복구 모드로 전환 중입니다.")
                if not recovery["active"]:
                    raise RuntimeError("현재 수동 복구 모드가 아닙니다.")
                if recovery["completing"]:
                    raise RuntimeError("이미 복구 완료 처리를 진행 중입니다.")
                recovery["completing"] = True
                recovery["error"] = ""
                recovery["message"] = "Recovery 종료 및 작업 취소 상태 정리 중입니다."

            Thread(target=self._complete_collision_recovery_worker, daemon=True).start()
        return True

    def _complete_collision_recovery_worker(self):
        """control=7 → state=1 → auto mode → UI STOP 유지 순서를 백그라운드에서 수행합니다."""
        with self.recovery_lock:
            try:
                # stop은 이미 충돌 감지 직후 선처리했습니다.
                # 여기서는 로봇 Recovery만 해제합니다.
                self.set_robot_control_sync(CONTROL_RESET_RECOVERY)

                deadline = time.monotonic() + 12.0
                while time.monotonic() < deadline:
                    if self.last_robot_state == ROBOT_STATE_STANDBY:
                        break
                    time.sleep(0.2)
                else:
                    raise TimeoutError(
                        "CONTROL_RESET_RECOVERY 후 robot_state=1 확인 시간 초과"
                    )

                self.set_robot_mode_sync(ROBOT_MODE_AUTO)

                with lock:
                    status["recovery"]["active"] = False
                    status["recovery"]["entering"] = False
                    status["recovery"]["completing"] = False
                    status["recovery"]["message"] = "충돌 복구가 완료되었습니다. 정지 리셋이 필요합니다."
                    status["recovery"]["error"] = ""
                    status["collision_alert"] = False
                    status["collision_stop"] = False
                    status["main_state"] = "STOP"
                    status["running"] = False
                    status["paused"] = False
                    status["emergency_stop"] = True
                    status["process_code"] = 0
                    status["progress"] = 0
                    status["current_step"] = waiting_step_key(status["selected_dish"])
                    status["sensor"]["robot_mode"] = "작업 정지"
                    status["message"] = (
                        "충돌 복구 완료: 기존 작업은 취소되었습니다. 다음 작업을 하려면 정지 리셋을 누르세요."
                    )
                    add_alarm(
                        "resolved",
                        "충돌 복구 완료 · 작업 취소",
                        "CONTROL_RESET_RECOVERY(7), robot_state=1 확인, Auto Mode 전환을 완료했고 UI는 STOP 상태로 유지합니다.",
                        "리셋 필요",
                    )

                self.node.get_logger().info(
                    "[recovery] control=7 → state=1 → Auto Mode → UI STOP 유지 완료"
                )
                return True

            except Exception as exc:
                with lock:
                    status["recovery"]["completing"] = False
                    status["recovery"]["error"] = str(exc)
                    status["recovery"]["message"] = (
                        "복구 완료 처리에 실패했습니다. Recovery 상태를 유지합니다."
                    )
                    add_alarm(
                        "critical",
                        "충돌 복구 완료 실패",
                        str(exc),
                        "Recovery 유지",
                    )
                self.node.get_logger().error(f"[recovery] 완료 실패: {exc}")
                return False

    def _force_all_callback(self, msg):
        values = [float(value) for value in msg.data]
        if len(values) < 6:
            self.node.get_logger().warning(
                f"/dishwasher/force/all 데이터 부족: {values}"
            )
            return

        values = values[:6]
        with lock:
            status["sensor"]["force_all"] = values
            status["sensor"]["fz"] = values[2]
            status["sensor"]["topics"]["force_all"] = True
            status["sensor"]["connection"] = "정상"

    def _force_magnitude_callback(self, msg):
        with lock:
            status["sensor"]["force_magnitude"] = float(msg.data)
            status["sensor"]["topics"]["force_magnitude"] = True
            status["sensor"]["connection"] = "정상"

    def _gripper_callback(self, msg):
        gripper_text = str(msg.data)
        with lock:
            status["sensor"]["gripper"] = gripper_text
            status["sensor"]["gripper_closed"] = is_gripper_closed_text(gripper_text)
            status["sensor"]["topics"]["gripper"] = True
            status["sensor"]["connection"] = "정상"

    def _gripper_safety_stop_callback(self, msg):
        safety_stop = bool(msg.data)
        with lock:
            previous = bool(status["sensor"].get("gripper_safety_stop", False))
            status["sensor"]["gripper_safety_stop"] = safety_stop
            status["sensor"]["topics"]["gripper_safety_stop"] = True
            status["sensor"]["connection"] = "정상"
            status["sensor"]["gripper"] = (
                "안전 모드 · 빨간 LED" if safety_stop else "정상"
            )

            if safety_stop:
                status["sensor"]["gripper_restart_pending"] = False
                if not previous:
                    add_alarm(
                        "critical",
                        "그리퍼 안전 정지",
                        "그리퍼의 s1_triggered, s2_triggered 또는 safety_error가 감지되었습니다.",
                        "재시작 필요",
                    )
            else:
                was_pending = bool(status["sensor"].get("gripper_restart_pending", False))
                status["sensor"]["gripper_restart_pending"] = False
                if previous:
                    add_alarm(
                        "resolved",
                        "그리퍼 안전 상태 해제",
                        "그리퍼 안전 정지 신호가 false로 변경되어 정상 상태를 확인했습니다.",
                        "정상",
                    )

    def _request_robot_state(self):
        """예전에 동작하던 방식의 충돌 감지 서비스 요청입니다."""
        if self.robot_state_request_in_progress:
            return

        if self.robot_state_client is None:
            return

        if not self.robot_state_client.service_is_ready():
            # 서비스가 아직 준비되지 않았으면 다음 타이머에서 다시 확인합니다.
            return

        self.robot_state_request_in_progress = True

        try:
            future = self.robot_state_client.call_async(
                GetRobotState.Request()
            )
            future.add_done_callback(
                self._robot_state_response
            )

        except Exception as exc:
            self.robot_state_request_in_progress = False
            if self.node is not None:
                self.node.get_logger().warning(
                    f"[collision] get_robot_state 요청 실패: {exc}"
                )

    def _robot_state_response(self, future):
        """robot_state=5이면 노란색 충돌 팝업과 알람을 활성화합니다."""
        self.robot_state_request_in_progress = False

        try:
            response = future.result()

            if response is None:
                return

            if not response.success:
                return

            robot_state = int(response.robot_state)
            previous_state = self.last_robot_state
            self.last_robot_state = robot_state

            if self.node is not None:
                self.node.get_logger().info(
                    f"[collision] robot_state={robot_state}, "
                    f"previous={previous_state}"
                )

            with lock:
                status["robot_state"] = robot_state
                status["sensor"]["robot_state_code"] = robot_state
                status["sensor"]["robot_mode"] = (
                    "충격 감지 정지"
                    if robot_state == 5
                    else f"로봇 상태 {robot_state}"
                )
                status["sensor"]["connection"] = "정상"

                if robot_state == ROBOT_STATE_COLLISION:
                    status["running"] = False
                    status["paused"] = False
                    status["collision_stop"] = True

                    # 핵심: 상태 5가 수신되는 동안에는 이전 상태와 관계없이
                    # 충돌 팝업을 계속 유지합니다. 브라우저 polling 사이에
                    # Recovery 상태로 전환되더라도 팝업을 놓치지 않습니다.
                    status["collision_alert"] = True
                    status["message"] = (
                        "로봇이 충격을 감지하여 정지했습니다. "
                        "로봇 본체, 식기 파지 상태 및 주변 장애물을 확인하세요."
                    )

                    # 알람 기록과 Recovery 진입은 상태 5 최초 진입 때만 실행합니다.
                    if previous_state != ROBOT_STATE_COLLISION:
                        add_alarm(
                            "warning",
                            "충격 감지 정지",
                            "로봇이 외부 충격을 감지하여 보호 정지 상태가 되었습니다.",
                            "확인 필요",
                        )

                        if self.node is not None:
                            self.node.get_logger().warning(
                                "[collision] 상태 5 감지: 팝업 및 알람 활성화"
                            )

                        # ROS executor 콜백을 막지 않도록 별도 스레드에서
                        # Manual Mode + CONTROL_RECOVERY_SAFE_STOP(4)을 적용합니다.
                        self._begin_collision_recovery_async()

                else:
                    recovery_busy = (
                        status["recovery"]["active"]
                        or status["recovery"]["entering"]
                        or status["recovery"]["completing"]
                    )

                    # Recovery Safe 상태에서는 robot_state가 5가 아니어도
                    # 사용자가 복구 완료를 누를 때까지 팝업을 유지합니다.
                    if recovery_busy or robot_state == 8:
                        status["collision_alert"] = True
                        status["collision_stop"] = True
                    else:
                        status["collision_stop"] = False

                        if previous_state == 5:
                            status["collision_alert"] = False
                            add_alarm(
                                "resolved",
                                "충격 상태 해제",
                                f"로봇 상태가 {robot_state}로 변경되어 충격 상태가 해제되었습니다.",
                                "해제",
                            )

        except Exception as exc:
            if self.node is not None:
                self.node.get_logger().warning(
                    f"[collision] 로봇 상태 조회 실패: {exc}"
                )

    def _state_callback(self, msg):
        new_state = str(msg.data).strip().upper()
        if new_state not in {"INIT", "IDLE", "RUN", "STOP"}:
            self.node.get_logger().warning(
                f"알 수 없는 /dishwasher/state 값: {new_state}"
            )
            return

        previous_state = self.last_main_state
        self.last_main_state = new_state

        with lock:
            status["main_state"] = new_state
            status["running"] = new_state == "RUN"
            status["paused"] = False
            status["emergency_stop"] = new_state == "STOP"
            status["ros"]["state_received"] = True
            status["sensor"]["connection"] = "정상"

            if new_state == "INIT":
                status["sensor"]["robot_mode"] = "초기화"
                status["message"] = "메인 노드가 로봇과 도구를 초기화하는 중입니다."

            elif new_state == "RUN":
                status["sensor"]["robot_mode"] = "자동 운전"
                status["message"] = f"{status['selected_dish']} 세척 작업을 수행 중입니다."
                if previous_state != "RUN":
                    self.run_had_nonzero_step = False
                    self.completion_counted_for_run = False

            elif new_state == "STOP":
                status["sensor"]["robot_mode"] = "작업 정지"
                status["process_code"] = 0
                status["progress"] = 0
                status["current_step"] = waiting_step_key(status["selected_dish"])
                status["message"] = "작업이 정지되었습니다. 안전 확인 후 리셋하세요."
                if previous_state != "STOP":
                    add_alarm(
                        "critical",
                        "STOP 상태 진입",
                        "작업이 정지되었습니다. 다음 작업을 하려면 정지 리셋 버튼을 눌러야 합니다.",
                        "리셋 필요",
                    )

            elif new_state == "IDLE":
                status["sensor"]["robot_mode"] = "대기"
                status["process_code"] = 0
                status["progress"] = 100 if previous_state == "RUN" else 0
                selected_dish = status["selected_dish"]
                status["current_step"] = (
                    complete_step_key(selected_dish)
                    if previous_state == "RUN"
                    else waiting_step_key(selected_dish)
                )

                completed_now = (
                    previous_state == "RUN"
                    and self.run_had_nonzero_step
                    and not self.completion_counted_for_run
                )
                if completed_now:
                    self.completion_counted_for_run = True
                    status["completed"][selected_dish] += 1
                    status["message"] = f"{selected_dish} 세척 작업이 완료되었습니다."
                    add_alarm(
                        "info",
                        f"{selected_dish} 작업 완료",
                        f"{selected_dish} 세척 공정이 정상 완료되었습니다.",
                        "완료",
                    )
                else:
                    status["message"] = "세척할 식기를 선택하고 작업 시작 버튼을 눌러주세요."

    def _process_plate_callback(self, msg):
        process_code = int(msg.data)
        step_by_code = {step["code"]: step for step in PLATE_STEPS}

        if process_code not in step_by_code:
            self.node.get_logger().warning(
                f"접시 진행 단계 범위 오류: {process_code} (허용 0~9)"
            )
            return

        step = step_by_code[process_code]
        self.last_process_code = process_code
        if 1 <= process_code <= 8:
            self.run_had_nonzero_step = True

        with lock:
            # 다른 식기의 대기(0) 토픽이 들어와도 Operation 선택을 덮어쓰지 않습니다.
            if status["selected_dish"] != "접시":
                status["ros"]["process_plate_received"] = True
                status["sensor"]["topics"]["process_plate"] = True
                return

            status["process_code"] = process_code
            status["current_step"] = step["key"]
            status["progress"] = 0 if process_code == 0 else 100 if process_code == 9 else round(process_code * 100 / 9)
            status["ros"]["process_plate_received"] = True
            status["sensor"]["topics"]["process_plate"] = True
            status["sensor"]["connection"] = "정상"

            if process_code == 0:
                if status["main_state"] == "STOP":
                    status["message"] = "작업이 중단되었습니다. 리셋이 필요합니다."
                else:
                    status["message"] = "작업 명령을 기다리는 대기 상태입니다."
            elif process_code == 9:
                status["message"] = "접시 세척 작업이 완료되었습니다."
            else:
                status["message"] = step["description"]

    def _process_bowl_callback(self, msg):
        process_code = int(msg.data)
        step_by_code = {step["code"]: step for step in BOWL_STEPS}

        if process_code not in step_by_code:
            self.node.get_logger().warning(
                f"밥그릇 진행 단계 범위 오류: {process_code} (허용 0~7)"
            )
            return

        step = step_by_code[process_code]
        self.last_process_code = process_code
        if 1 <= process_code <= 7:
            self.run_had_nonzero_step = True

        with lock:
            # 다른 식기의 대기(0) 토픽이 들어와도 Operation 선택을 덮어쓰지 않습니다.
            if status["selected_dish"] != "밥그릇":
                status["ros"]["process_bowl_received"] = True
                status["sensor"]["topics"]["process_bowl"] = True
                return

            status["process_code"] = process_code
            status["current_step"] = step["key"]
            status["progress"] = (
                0
                if process_code == 0
                else 100
                if process_code == 7
                else round(process_code * 100 / 7)
            )
            status["ros"]["process_bowl_received"] = True
            status["sensor"]["topics"]["process_bowl"] = True
            status["sensor"]["connection"] = "정상"

            if process_code == 0:
                if status["main_state"] == "STOP":
                    status["message"] = "작업이 중단되었습니다. 리셋이 필요합니다."
                else:
                    status["message"] = "식기 선택과 작업 시작 명령을 기다립니다."
            elif process_code == 7:
                status["message"] = "세척이 끝난 밥그릇을 지정된 완료 위치에 내려놓습니다."
            else:
                status["message"] = step["description"]


    def _process_cup_callback(self, msg):
        process_code = int(msg.data)
        step_by_code = {step["code"]: step for step in CUP_STEPS}

        if process_code not in step_by_code:
            self.node.get_logger().warning(
                f"컵 진행 단계 범위 오류: {process_code} (허용 0~7)"
            )
            return

        step = step_by_code[process_code]
        self.last_process_code = process_code
        if 1 <= process_code <= 7:
            self.run_had_nonzero_step = True

        with lock:
            # 다른 식기의 대기(0) 토픽이 들어와도 Operation 선택을 덮어쓰지 않습니다.
            if status["selected_dish"] != "컵":
                status["ros"]["process_cup_received"] = True
                status["sensor"]["topics"]["process_cup"] = True
                return

            status["process_code"] = process_code
            status["current_step"] = step["key"]
            status["progress"] = (
                0
                if process_code == 0
                else 100
                if process_code == 7
                else round(process_code * 100 / 7)
            )
            status["ros"]["process_cup_received"] = True
            status["sensor"]["topics"]["process_cup"] = True
            status["sensor"]["connection"] = "정상"

            if process_code == 0:
                if status["main_state"] == "STOP":
                    status["message"] = "작업이 중단되었습니다. 리셋이 필요합니다."
                else:
                    status["message"] = "컵 작업 시작 명령을 기다립니다."
            elif process_code == 7:
                status["message"] = "세척이 끝난 컵을 지정된 완료 위치에 배치하고 작업을 완료합니다."
            else:
                status["message"] = step["description"]


ros_bridge = RosBridge()


@app.route("/")
def index():
    with lock:
        return render_template(
            "index.html",
            status=build_status(),
            settings=settings,
            alarms=alarms,
        )


@app.get("/api/status")
def get_status():
    with lock:
        return jsonify(build_status())


@app.get("/api/alarms")
def get_alarms():
    with lock:
        return jsonify(alarms)


@app.get("/api/settings")
def get_settings():
    with lock:
        return jsonify(settings)


@app.post("/api/select")
def select_dish():
    data = request.get_json(silent=True) or {}
    dish = data.get("dish")

    if dish not in DISH_TYPES:
        return jsonify(ok=False, message="지원하지 않는 식기입니다."), 400

    with lock:
        status["selected_dish"] = dish
        status["progress"] = 0
        status["process_code"] = 0
        status["current_step"] = waiting_step_key(dish)
        status["message"] = f"{dish}이(가) 선택되었습니다. 작업 시작 버튼을 눌러주세요."
        return jsonify(ok=True, status=build_status())


@app.post("/api/start")
def start():
    with lock:
        main_state = status["main_state"]
        dish = status["selected_dish"]
        command = COMMAND_BY_DISH[dish]

        if main_state != "IDLE":
            message = f"현재 상태가 {main_state}라서 작업 시작을 누를 수 없습니다."
            status["message"] = message
            add_alarm(
                "warning",
                "작업 시작 불가",
                message,
                "명령 차단",
            )
            return jsonify(ok=False, message=message, status=build_status()), 409

    if not ros_bridge.publish_command(command):
        return jsonify(
            ok=False,
            message="/dishwasher/cmd 토픽 발행에 실패했습니다. ROS2 연결을 확인하세요.",
        ), 503

    with lock:
        status["message"] = (
            f"{dish} 작업 시작 명령({command})을 전송했습니다. "
            "/dishwasher/state의 RUN 응답을 기다리는 중입니다."
        )
        add_alarm(
            "info",
            "작업 시작 명령",
            f"/dishwasher/cmd 에 {command} 명령을 발행했습니다.",
            "RUN 대기",
        )
        return jsonify(ok=True, command=command, status=build_status())


@app.post("/api/stop")
def stop():
    with lock:
        main_state = status["main_state"]

        if main_state != "RUN":
            message = f"현재 상태가 {main_state}라서 작업 정지를 누를 수 없습니다."
            status["message"] = message
            add_alarm(
                "warning",
                "작업 정지 불가",
                message,
                "명령 차단",
            )
            return jsonify(ok=False, message=message, status=build_status()), 409

    if not ros_bridge.publish_command("stop"):
        return jsonify(
            ok=False,
            message="정지 명령 발행에 실패했습니다. ROS2 연결을 확인하세요.",
        ), 503

    with lock:
        status["message"] = "정지 명령을 전송했습니다. STOP 상태 응답을 기다리는 중입니다."
        add_alarm(
            "warning",
            "작업 정지 명령",
            "/dishwasher/cmd 에 stop 명령을 발행했습니다. STOP 상태가 되면 정지 리셋이 필요합니다.",
            "STOP 대기",
        )
        return jsonify(ok=True, command="stop", status=build_status())


@app.post("/api/emergency-stop")
def emergency_stop():
    # 메인 노드 스펙상 stop 명령 자체가 QSTOP이므로 같은 명령을 사용합니다.
    return stop()


@app.post("/api/reset-stop")
def reset_stop():
    with lock:
        main_state = status["main_state"]

        if main_state != "STOP":
            message = f"현재 상태가 {main_state}라서 정지 리셋을 누를 수 없습니다."
            status["message"] = message
            add_alarm(
                "warning",
                "정지 리셋 불가",
                message,
                "명령 차단",
            )
            return jsonify(ok=False, message=message, status=build_status()), 409

    if not ros_bridge.publish_command("reset"):
        return jsonify(
            ok=False,
            message="리셋 명령 발행에 실패했습니다. ROS2 연결을 확인하세요.",
        ), 503

    with lock:
        status["message"] = "리셋 명령을 전송했습니다. IDLE 상태 응답을 기다리는 중입니다."
        add_alarm(
            "resolved",
            "정지 리셋 명령",
            "/dishwasher/cmd 에 reset 명령을 발행했습니다.",
            "IDLE 대기",
        )
        return jsonify(ok=True, command="reset", status=build_status())


@app.post("/api/pause")
def pause_compatibility():
    # 기존 HTML/캐시 호환용. 실제 시스템에는 일시정지가 없고 stop만 있습니다.
    return stop()


@app.post("/api/home")
def go_home():
    with lock:
        main_state = status["main_state"]

        if main_state != "IDLE":
            message = (
                f"현재 상태가 {main_state}라서 홈 위치 이동을 실행할 수 없습니다. "
                "정지 후에는 정지 리셋을 눌러 IDLE 상태가 된 뒤 사용할 수 있습니다."
            )
            status["message"] = message
            add_alarm(
                "warning",
                "홈 위치 이동 불가",
                message,
                "IDLE 필요",
            )
            return jsonify(ok=False, message=message, status=build_status()), 409

    if not ros_bridge.publish_home():
        return jsonify(
            ok=False,
            message="/dishwasher/robot/home 토픽 발행에 실패했습니다. ROS2 연결을 확인하세요.",
            status=build_status(),
        ), 503

    with lock:
        status["message"] = "홈 위치 이동 명령을 전송했습니다."
        add_alarm(
            "info",
            "홈 위치 이동 명령",
            "/dishwasher/robot/home 토픽으로 Empty 메시지를 발행했습니다.",
            "명령 전송",
        )
        return jsonify(ok=True, command="home", status=build_status())


@app.post("/api/gripper/restart")
def restart_gripper():
    with lock:
        safety_stop = bool(status["sensor"].get("gripper_safety_stop", False))
        pending = bool(status["sensor"].get("gripper_restart_pending", False))

        if not safety_stop:
            return jsonify(
                ok=False,
                message="그리퍼가 안전 모드(빨간색)일 때만 다시 시작할 수 있습니다.",
                status=build_status(),
            ), 409

        if pending:
            return jsonify(
                ok=False,
                message="그리퍼 다시 시작 요청이 이미 진행 중입니다.",
                status=build_status(),
            ), 409

    if not ros_bridge.publish_gripper_restart():
        return jsonify(
            ok=False,
            message="/dishwasher/gripper/restart 토픽 발행에 실패했습니다.",
        ), 503

    with lock:
        add_alarm(
            "warning",
            "그리퍼 다시 시작 요청",
            "/dishwasher/gripper/restart 토픽으로 Empty 메시지를 발행했습니다.",
            "Power Cycle 진행",
        )
        status["message"] = "그리퍼 다시 시작 요청을 보냈습니다. 안전 상태 해제를 기다리는 중입니다."
        return jsonify(ok=True, status=build_status())


@app.post("/api/gripper/open")
def open_gripper():
    with lock:
        main_state = status["main_state"]
        gripper_text = status["sensor"].get("gripper", "")
        gripper_closed = bool(status["sensor"].get("gripper_closed", False)) or is_gripper_closed_text(gripper_text)
        safety_stop = bool(status["sensor"].get("gripper_safety_stop", False))

        if main_state != "IDLE":
            message = f"현재 상태가 {main_state}라서 그리퍼 열기를 실행할 수 없습니다. IDLE 상태에서만 사용할 수 있습니다."
            status["message"] = message
            add_alarm(
                "warning",
                "그리퍼 열기 불가",
                message,
                "IDLE 필요",
            )
            return jsonify(ok=False, message=message, status=build_status()), 409

        if safety_stop:
            message = "그리퍼 안전 정지 상태에서는 그리퍼 열기를 실행할 수 없습니다. 먼저 안전 상태를 해제하세요."
            status["message"] = message
            add_alarm(
                "warning",
                "그리퍼 열기 불가",
                message,
                "안전 정지",
            )
            return jsonify(ok=False, message=message, status=build_status()), 409

        if not gripper_closed:
            message = "그리퍼가 닫혀있을 때만 그리퍼 열기 버튼을 사용할 수 있습니다."
            status["message"] = message
            add_alarm(
                "warning",
                "그리퍼 열기 불가",
                f"현재 그리퍼 상태: {gripper_text or '미연동'}",
                "닫힘 필요",
            )
            return jsonify(ok=False, message=message, status=build_status()), 409

    if not ros_bridge.publish_gripper_open():
        return jsonify(
            ok=False,
            message="/dishwasher/gripper/open 토픽 발행에 실패했습니다. ROS2 연결을 확인하세요.",
            status=build_status(),
        ), 503

    with lock:
        status["message"] = "그리퍼 열기 명령을 전송했습니다."
        add_alarm(
            "info",
            "그리퍼 열기 명령",
            "/dishwasher/gripper/open 토픽으로 Empty 메시지를 발행했습니다.",
            "명령 전송",
        )
        return jsonify(ok=True, command="gripper_open", status=build_status())


@app.post("/api/collision-recovery/complete")
def complete_collision_recovery():
    try:
        ros_bridge.start_complete_collision_recovery()
        with lock:
            return jsonify(ok=True, message="복구 완료 처리를 시작했습니다.", status=build_status()), 202
    except Exception as exc:
        with lock:
            return jsonify(
                ok=False,
                message=str(exc),
                status=build_status(),
            ), 409


@app.post("/api/collision-alert/ack")
def acknowledge_collision_alert():
    with lock:
        status["collision_alert"] = False
        return jsonify(ok=True, status=build_status())


@app.post("/api/settings")
def update_settings():
    data = request.get_json(silent=True) or {}
    with lock:
        for key in settings:
            if key in data:
                settings[key] = data[key]
        add_alarm("info", "설정 변경", "UI 설정이 변경되었습니다.", "적용")
        return jsonify(ok=True, settings=settings)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
