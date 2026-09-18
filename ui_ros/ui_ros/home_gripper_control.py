#!/usr/bin/env python3

import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Empty

import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

HOME_TOPIC = "/dishwasher/robot/home"
GRIPPER_OPEN_TOPIC = "/dishwasher/gripper/open"

# 홈 관절 위치
HOME_JOINT = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

HOME_VELOCITY = 30.0
HOME_ACCELERATION = 30.0

# 그리퍼 OPEN
# DO2 = 1
# DO3 = 0
GRIPPER_OPEN_PORT = 2
GRIPPER_CLOSE_PORT = 3

DIGITAL_ON = 1
DIGITAL_OFF = 0

GRIPPER_OPEN_WAIT = 1.0


class HomeOpenController:

    def __init__(self, ui_node, dsr):
        self.ui_node = ui_node
        self.dsr = dsr

        # 홈 이동과 그리퍼 출력이 동시에 DSR API를 호출하지 않도록 보호
        self.robot_lock = threading.Lock()

        # 실행 상태 변수 보호
        self.state_lock = threading.Lock()

        self.home_running = False
        self.gripper_running = False

        # 홈 이동 토픽 구독
        self.home_subscription = self.ui_node.create_subscription(
            Empty,
            HOME_TOPIC,
            self.home_callback,
            10,
        )

        # 그리퍼 OPEN 토픽 구독
        self.gripper_subscription = self.ui_node.create_subscription(
            Empty,
            GRIPPER_OPEN_TOPIC,
            self.gripper_open_callback,
            10,
        )

        self.ui_node.get_logger().info(
            "홈·그리퍼 제어 노드 시작"
        )
        self.ui_node.get_logger().info(
            f"홈 이동 수신: {HOME_TOPIC} [std_msgs/msg/Empty]"
        )
        self.ui_node.get_logger().info(
            f"그리퍼 OPEN 수신: {GRIPPER_OPEN_TOPIC} [std_msgs/msg/Empty]"
        )

    # ==========================================================
    # 홈 이동
    # ==========================================================

    def home_callback(self, _msg):
        with self.state_lock:
            if self.home_running:
                self.ui_node.get_logger().warning(
                    "홈 이동이 이미 실행 중이므로 중복 명령을 무시합니다."
                )
                return

            self.home_running = True

        self.ui_node.get_logger().info(
            f"{HOME_TOPIC} 명령 수신"
        )

        threading.Thread(
            target=self.move_home,
            daemon=True,
        ).start()

    def move_home(self):
        try:
            with self.robot_lock:
                self.ui_node.get_logger().info(
                    "홈 위치 이동 시작: [0, 0, 90, 0, 90, 0]"
                )

                home_position = self.dsr.posj(HOME_JOINT)

                result = self.dsr.movej(
                    home_position,
                    vel=HOME_VELOCITY,
                    acc=HOME_ACCELERATION,
                )

                if result == 0:
                    self.ui_node.get_logger().info(
                        "홈 위치 이동 완료"
                    )
                else:
                    self.ui_node.get_logger().warning(
                        f"홈 이동 반환값: {result}"
                    )

        except Exception as error:
            self.ui_node.get_logger().error(
                f"홈 이동 실패: {error}"
            )

        finally:
            with self.state_lock:
                self.home_running = False

    # ==========================================================
    # 그리퍼 OPEN
    # ==========================================================

    def gripper_open_callback(self, _msg):
        with self.state_lock:
            if self.gripper_running:
                self.ui_node.get_logger().warning(
                    "그리퍼 OPEN이 이미 실행 중이므로 "
                    "중복 명령을 무시합니다."
                )
                return

            self.gripper_running = True

        self.ui_node.get_logger().info(
            f"{GRIPPER_OPEN_TOPIC} 명령 수신"
        )

        threading.Thread(
            target=self.open_gripper,
            daemon=True,
        ).start()

    def open_gripper(self):
        try:
            with self.robot_lock:
                self.ui_node.get_logger().info(
                    "그리퍼 OPEN 시작"
                )

                # 닫힘 출력 해제
                close_result = self.dsr.set_digital_output(
                    GRIPPER_CLOSE_PORT,
                    DIGITAL_OFF,
                )

                # 열림 출력 활성화
                open_result = self.dsr.set_digital_output(
                    GRIPPER_OPEN_PORT,
                    DIGITAL_ON,
                )

                self.ui_node.get_logger().info(
                    "그리퍼 출력 설정 완료: DO2=1, DO3=0"
                )

                self.ui_node.get_logger().info(
                    f"API 반환값: DO2={open_result}, DO3={close_result}"
                )

                time.sleep(GRIPPER_OPEN_WAIT)

                self.ui_node.get_logger().info(
                    "그리퍼 OPEN 완료"
                )

        except Exception as error:
            self.ui_node.get_logger().error(
                f"그리퍼 OPEN 실패: {error}"
            )

        finally:
            with self.state_lock:
                self.gripper_running = False


def main(args=None):
    rclpy.init(args=args)

    ui_node = None
    dsr_node = None
    ui_executor = None
    controller = None

    try:
        # ------------------------------------------------------
        # UI 토픽 수신 전용 노드
        # ------------------------------------------------------
        ui_node = rclpy.create_node(
            "home_open",
            namespace=ROBOT_ID,
        )

        # ------------------------------------------------------
        # Doosan API 전용 노드
        #
        # 이 노드는 ui_executor에 추가하지 않는다.
        # DSR_ROBOT2 내부 서비스 호출에서 사용한다.
        # ------------------------------------------------------
        dsr_node = rclpy.create_node(
            "home_open_dsr",
            namespace=ROBOT_ID,
        )

        # 반드시 DSR_ROBOT2 import 전에 설정
        DR_init.__dsr__node = dsr_node
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL

        # 반드시 DR_init 설정 후 import
        import DSR_ROBOT2 as dsr

        controller = HomeOpenController(
            ui_node=ui_node,
            dsr=dsr,
        )

        # ------------------------------------------------------
        # 전역 rclpy.spin()을 사용하지 않고
        # UI 노드 전용 Executor를 사용한다.
        # ------------------------------------------------------
        ui_executor = SingleThreadedExecutor()
        ui_executor.add_node(ui_node)

        ui_node.get_logger().info(
            "UI 토픽 수신 Executor 시작"
        )

        ui_executor.spin()

    except KeyboardInterrupt:
        if ui_node is not None:
            ui_node.get_logger().info(
                "홈·그리퍼 제어 노드 종료"
            )

    except Exception as error:
        if ui_node is not None:
            ui_node.get_logger().error(
                f"홈·그리퍼 제어 노드 실행 실패: {error}"
            )
        else:
            print(
                f"홈·그리퍼 제어 노드 실행 실패: {error}"
            )

    finally:
        if ui_executor is not None:
            try:
                if ui_node is not None:
                    ui_executor.remove_node(ui_node)
            except Exception:
                pass

            try:
                ui_executor.shutdown()
            except Exception:
                pass

        if controller is not None:
            del controller

        if ui_node is not None:
            try:
                ui_node.destroy_node()
            except Exception:
                pass

        if dsr_node is not None:
            try:
                dsr_node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()