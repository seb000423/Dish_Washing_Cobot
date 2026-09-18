#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Empty, String

from pymodbus.client import ModbusTcpClient


# ==================================================
# OnRobot Compute Box 설정
# ==================================================
ONROBOT_IP = "192.168.1.1"
ONROBOT_PORT = 502

# 일반 Quick Changer: 65
# Dual Quick Changer Primary: 66
# Dual Quick Changer Secondary: 67
GRIPPER_DEVICE_ID = 65

# Compute Box 자체의 Modbus 장치 주소
COMPUTE_BOX_DEVICE_ID = 63

# RG2/RG6 상태 레지스터
STATUS_REGISTER_ADDRESS = 268

# Compute Box Power Cycle 레지스터
POWER_CYCLE_REGISTER = 0
POWER_CYCLE_COMMAND = 2

# 상태 확인 주기
CHECK_PERIOD = 0.05


# ==================================================
# ROS2 토픽
# ==================================================
# 기존 식기세척 메인 노드 명령
DISHWASHER_CMD_TOPIC = "/dishwasher/cmd"
STOP_COMMAND = "stop"

# UI -> 그리퍼 감시 노드
# std_msgs/msg/Empty 한 번 발행하면 그리퍼 전원 사이클 실행
GRIPPER_RESTART_TOPIC = "/dishwasher/gripper/restart"

# 그리퍼 감시 노드 -> UI
# std_msgs/msg/Bool: True=빨간 LED/안전 오류, False=정상
GRIPPER_SAFETY_STOP_TOPIC = "/dishwasher/gripper/safety_stop"

# 안전센서 1회 발동 시 stop 발행 횟수/간격
STOP_PUBLISH_COUNT = 3
STOP_PUBLISH_INTERVAL = 0.2

# 노드 시작 직후 publisher/subscriber 매칭 대기 시간
SUBSCRIBER_WAIT_SEC = 2.0

# 전원 사이클 후 장치 복구 대기 시간
RESTART_WAIT_SEC = 5.0


class GripperSafetyStopPublisher(Node):

    def __init__(self):
        super().__init__("gripper_safety_stop_publisher")

        # 상태 감시 전용 Modbus 연결
        self.modbus_client = ModbusTcpClient(
            host=ONROBOT_IP,
            port=ONROBOT_PORT,
            timeout=1,
        )

        if not self.modbus_client.connect():
            raise RuntimeError(
                f"OnRobot Compute Box 연결 실패: {ONROBOT_IP}:{ONROBOT_PORT}"
            )

        self.get_logger().info(
            f"OnRobot Compute Box 연결 성공: {ONROBOT_IP}:{ONROBOT_PORT}"
        )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # UI가 늦게 접속해도 마지막 오류 상태를 즉시 받도록 TRANSIENT_LOCAL 사용
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_publisher = self.create_publisher(
            String,
            DISHWASHER_CMD_TOPIC,
            command_qos,
        )

        self.safety_status_publisher = self.create_publisher(
            Bool,
            GRIPPER_SAFETY_STOP_TOPIC,
            status_qos,
        )

        self.restart_subscription = self.create_subscription(
            Empty,
            GRIPPER_RESTART_TOPIC,
            self.on_restart_requested,
            command_qos,
        )

        self.get_logger().info(f"안전 정지 명령 토픽: {DISHWASHER_CMD_TOPIC}")
        self.get_logger().info(
            f"그리퍼 재시작 수신 토픽: {GRIPPER_RESTART_TOPIC} [std_msgs/msg/Empty]"
        )
        self.get_logger().info(
            f"그리퍼 안전 상태 발행 토픽: {GRIPPER_SAFETY_STOP_TOPIC} [std_msgs/msg/Bool]"
        )

        self.safety_latched = False
        self.last_status_raw = None
        self.last_published_safety_state = None

        self.stop_publish_remaining = 0
        self.stop_publish_timer = None

        self.restart_in_progress = False
        self.restart_lock = threading.Lock()

        self.subscriber_wait_deadline = (
            self.get_clock().now()
            + rclpy.duration.Duration(seconds=SUBSCRIBER_WAIT_SEC)
        )
        self.match_check_timer = self.create_timer(
            0.2,
            self.check_cmd_subscriber_match,
        )

        self.timer = self.create_timer(
            CHECK_PERIOD,
            self.check_gripper_safety,
        )

        # 초기 상태를 UI에 제공
        self.publish_safety_state(False, force=True)
        self.get_logger().info("그리퍼 안전센서 감시를 시작합니다.")

    def check_cmd_subscriber_match(self):
        sub_count = self.cmd_publisher.get_subscription_count()

        if sub_count > 0:
            self.get_logger().info(
                f"/dishwasher/cmd 구독자 감지됨: {sub_count}개"
            )
            self.match_check_timer.cancel()
            return

        if self.get_clock().now() >= self.subscriber_wait_deadline:
            self.get_logger().warning(
                "/dishwasher/cmd 구독자가 아직 없습니다. "
                "메인 식기세척 노드가 실행 중인지, ROS_DOMAIN_ID가 같은지 확인하세요."
            )
            self.match_check_timer.cancel()

    def publish_safety_state(self, safety_triggered: bool, force: bool = False):
        """UI에 현재 그리퍼 빨간 상태 여부를 발행한다."""
        if not force and safety_triggered == self.last_published_safety_state:
            return

        message = Bool()
        message.data = safety_triggered
        self.safety_status_publisher.publish(message)
        self.last_published_safety_state = safety_triggered

        state_text = "FAULT/RED" if safety_triggered else "NORMAL"
        self.get_logger().info(
            f"그리퍼 안전 상태 발행: {GRIPPER_SAFETY_STOP_TOPIC} -> "
            f"{safety_triggered} ({state_text})"
        )

    def check_gripper_safety(self):
        """그리퍼 상태 레지스터를 읽고 안전 정지 여부를 확인한다."""
        # 전원 사이클 중에는 연결이 일시적으로 끊길 수 있어 감시 읽기를 건너뛴다.
        if self.restart_in_progress:
            return

        try:
            result = self.modbus_client.read_holding_registers(
                address=STATUS_REGISTER_ADDRESS,
                count=1,
                device_id=GRIPPER_DEVICE_ID,
            )

            if result is None:
                self.get_logger().warning("그리퍼 상태 응답이 없습니다.")
                return

            if result.isError():
                self.get_logger().warning(f"Modbus 상태 읽기 실패: {result}")
                return

            if not hasattr(result, "registers"):
                self.get_logger().warning("Modbus 응답에 registers가 없습니다.")
                return

            status_raw = int(result.registers[0])

            busy = bool(status_raw & (1 << 0))
            grip = bool(status_raw & (1 << 1))
            s1_pressed = bool(status_raw & (1 << 2))
            s1_triggered = bool(status_raw & (1 << 3))
            s2_pressed = bool(status_raw & (1 << 4))
            s2_triggered = bool(status_raw & (1 << 5))
            safety_error = bool(status_raw & (1 << 6))

            if status_raw != self.last_status_raw:
                self.last_status_raw = status_raw
                self.get_logger().info(
                    "\n"
                    "========== 그리퍼 상태 ==========\n"
                    f"status raw    : {status_raw}\n"
                    f"status binary : {status_raw:016b}\n"
                    f"busy          : {busy}\n"
                    f"grip          : {grip}\n"
                    f"s1_pressed    : {s1_pressed}\n"
                    f"s1_triggered  : {s1_triggered}\n"
                    f"s2_pressed    : {s2_pressed}\n"
                    f"s2_triggered  : {s2_triggered}\n"
                    f"safety_error  : {safety_error}\n"
                    "================================="
                )

            safety_triggered = s1_triggered or s2_triggered or safety_error

            # UI에 빨간 상태/정상 상태 전달
            self.publish_safety_state(safety_triggered)

            if safety_triggered:
                if not self.safety_latched:
                    self.safety_latched = True
                    self.get_logger().error("그리퍼 안전센서 발동 감지!")
                    self.start_stop_publish_burst()
            else:
                if self.safety_latched:
                    self.safety_latched = False
                    self.get_logger().info("그리퍼 안전 상태가 해제되었습니다.")

        except Exception as error:
            self.get_logger().error(f"그리퍼 상태 확인 중 오류: {error}")

    def on_restart_requested(self, _message: Empty):
        """UI의 그리퍼 다시 시작 버튼 토픽을 받는다."""
        with self.restart_lock:
            if self.restart_in_progress:
                self.get_logger().warning(
                    "그리퍼 재시작이 이미 진행 중이므로 중복 명령을 무시합니다."
                )
                return
            self.restart_in_progress = True

        self.get_logger().warning(
            f"그리퍼 재시작 명령 수신: {GRIPPER_RESTART_TOPIC}"
        )

        worker = threading.Thread(
            target=self.restart_gripper_worker,
            daemon=True,
        )
        worker.start()

    def restart_gripper_worker(self):
        """Compute Box 전원 사이클을 별도 스레드에서 실행한다."""
        restart_client = ModbusTcpClient(
            host=ONROBOT_IP,
            port=ONROBOT_PORT,
            timeout=2,
        )

        try:
            self.get_logger().warning(
                "그리퍼 전원 사이클 시작: 안전센서 물체와 식기 지지 상태를 확인하세요."
            )

            if not restart_client.connect():
                self.get_logger().error("그리퍼 재시작 실패: Compute Box 연결 실패")
                return

            result = restart_client.write_register(
                address=POWER_CYCLE_REGISTER,
                value=POWER_CYCLE_COMMAND,
                device_id=COMPUTE_BOX_DEVICE_ID,
            )

            if result is None:
                self.get_logger().error("그리퍼 재시작 실패: 응답 없음")
                return

            if result.isError():
                self.get_logger().error(
                    f"그리퍼 재시작 실패: Power Cycle 명령 오류: {result}"
                )
                return

            self.get_logger().info(
                f"그리퍼 Power Cycle 명령 성공. {RESTART_WAIT_SEC:.0f}초 대기합니다."
            )
            time.sleep(RESTART_WAIT_SEC)

            # 재시작 후 감시 연결을 새로 맺는다.
            try:
                self.modbus_client.close()
            except Exception:
                pass

            connected = self.modbus_client.connect()
            if connected:
                self.get_logger().info(
                    "그리퍼 재시작 완료. 상태 레지스터 감시를 재개합니다."
                )
                self.last_status_raw = None
            else:
                self.get_logger().warning(
                    "Power Cycle은 전송됐지만 상태 감시용 Modbus 재연결에 실패했습니다."
                )

        except Exception as error:
            self.get_logger().error(f"그리퍼 재시작 중 오류: {error}")

        finally:
            try:
                restart_client.close()
            except Exception:
                pass

            with self.restart_lock:
                self.restart_in_progress = False

    def start_stop_publish_burst(self):
        """stop 명령을 타이머 기반으로 반복 발행한다."""
        self.stop_publish_remaining = STOP_PUBLISH_COUNT

        if self.stop_publish_timer is None:
            self.stop_publish_timer = self.create_timer(
                STOP_PUBLISH_INTERVAL,
                self.publish_stop_once,
            )

        self.publish_stop_once()

    def publish_stop_once(self):
        if self.stop_publish_remaining <= 0:
            if self.stop_publish_timer is not None:
                self.stop_publish_timer.cancel()
                self.stop_publish_timer = None
            return

        message = String()
        message.data = STOP_COMMAND
        self.cmd_publisher.publish(message)

        sent_index = STOP_PUBLISH_COUNT - self.stop_publish_remaining + 1
        sub_count = self.cmd_publisher.get_subscription_count()

        self.get_logger().error(
            f"안전 정지 명령 발행 {sent_index}/{STOP_PUBLISH_COUNT}: "
            f"{DISHWASHER_CMD_TOPIC} -> '{STOP_COMMAND}', 구독자 수={sub_count}"
        )

        self.stop_publish_remaining -= 1

        if self.stop_publish_remaining <= 0:
            if self.stop_publish_timer is not None:
                self.stop_publish_timer.cancel()
                self.stop_publish_timer = None

    def close(self):
        try:
            self.modbus_client.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = GripperSafetyStopPublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n사용자 종료")

    except Exception as error:
        print(f"실행 실패: {error}")

    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
