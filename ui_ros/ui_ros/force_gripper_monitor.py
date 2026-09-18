#!/usr/bin/env python3

import math
import time

import rclpy
import DR_init

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

ON = 1
OFF = 0

PUBLISH_INTERVAL = 0.5

FORCE_ALL_TOPIC = "/dishwasher/force/all"
FORCE_MAGNITUDE_TOPIC = "/dishwasher/force/magnitude"
GRIPPER_STATE_TOPIC = "/dishwasher/gripper/state"


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def judge_gripper_state(do2, do3):
    if do2 == ON and do3 == OFF:
        return "열림"

    if do2 == OFF and do3 == ON:
        return "닫힘"

    if do2 == OFF and do3 == OFF:
        return "출력 OFF"

    if do2 == ON and do3 == ON:
        return "출력 충돌"

    return "알 수 없음"


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "force_gripper_publisher",
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = node

    try:
        import DSR_ROBOT2 as dsr

        get_tool_force = dsr.get_tool_force
        get_digital_output = dsr.get_digital_output

    except (ImportError, AttributeError) as error:
        node.get_logger().error(
            f"Doosan API 불러오기 실패: {error}"
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        return

    force_all_publisher = node.create_publisher(
        Float64MultiArray,
        FORCE_ALL_TOPIC,
        10,
    )

    force_magnitude_publisher = node.create_publisher(
        Float64,
        FORCE_MAGNITUDE_TOPIC,
        10,
    )

    gripper_state_publisher = node.create_publisher(
        String,
        GRIPPER_STATE_TOPIC,
        10,
    )

    node.get_logger().info("힘 및 그리퍼 상태 발행 시작")
    node.get_logger().info(f"전체 힘: {FORCE_ALL_TOPIC}")
    node.get_logger().info(f"힘 크기: {FORCE_MAGNITUDE_TOPIC}")
    node.get_logger().info(f"그리퍼 상태: {GRIPPER_STATE_TOPIC}")
    node.get_logger().info("종료하려면 Ctrl+C를 누르세요.")

    cycle_count = 0

    try:
        while rclpy.ok():
            cycle_count += 1

            force_values = None
            force_magnitude = None
            gripper_state = "읽기 실패"
            do2 = None
            do3 = None

            # ==================================================
            # 1. 힘 센서 읽기 및 토픽 발행
            # ==================================================
            try:
                node.get_logger().debug(
                    f"[{cycle_count}] get_tool_force 호출 전"
                )

                force = get_tool_force(0)

                node.get_logger().debug(
                    f"[{cycle_count}] get_tool_force 응답: {force}"
                )

                if force is None:
                    raise RuntimeError("get_tool_force 반환값이 None입니다.")

                if len(force) < 6:
                    raise RuntimeError(
                        f"힘 데이터 길이가 6보다 작습니다: {force}"
                    )

                fx = float(force[0])
                fy = float(force[1])
                fz = float(force[2])
                mx = float(force[3])
                my = float(force[4])
                mz = float(force[5])

                force_values = [
                    fx,
                    fy,
                    fz,
                    mx,
                    my,
                    mz,
                ]

                force_magnitude = math.sqrt(
                    fx ** 2 +
                    fy ** 2 +
                    fz ** 2
                )

                force_all_msg = Float64MultiArray()
                force_all_msg.data = force_values
                force_all_publisher.publish(force_all_msg)

                force_magnitude_msg = Float64()
                force_magnitude_msg.data = force_magnitude
                force_magnitude_publisher.publish(
                    force_magnitude_msg
                )

                node.get_logger().info(
                    f"힘 토픽 발행 성공 | "
                    f"Fx={fx:.3f}, Fy={fy:.3f}, Fz={fz:.3f}, "
                    f"|F|={force_magnitude:.3f} N"
                )

            except Exception as error:
                node.get_logger().error(
                    f"힘 센서 읽기 또는 발행 실패: {error}"
                )

            # ==================================================
            # 2. 그리퍼 출력 읽기 및 토픽 발행
            # ==================================================
            try:
                node.get_logger().debug(
                    f"[{cycle_count}] DO2 읽기 전"
                )

                do2 = int(get_digital_output(2))

                node.get_logger().debug(
                    f"[{cycle_count}] DO2={do2}"
                )

                do3 = int(get_digital_output(3))

                node.get_logger().debug(
                    f"[{cycle_count}] DO3={do3}"
                )

                gripper_state = judge_gripper_state(
                    do2,
                    do3,
                )

                gripper_state_msg = String()
                gripper_state_msg.data = gripper_state
                gripper_state_publisher.publish(
                    gripper_state_msg
                )

                node.get_logger().info(
                    f"그리퍼 토픽 발행 성공 | "
                    f"DO2={do2}, DO3={do3}, "
                    f"상태={gripper_state}"
                )

            except Exception as error:
                node.get_logger().error(
                    f"그리퍼 상태 읽기 또는 발행 실패: {error}"
                )

                error_msg = String()
                error_msg.data = "읽기 실패"
                gripper_state_publisher.publish(error_msg)

            # ==================================================
            # 3. 터미널 상태 표시
            # ==================================================
            print("\033[2J\033[H", end="", flush=True)

            print("========== 센서 토픽 발행 ==========")
            print(f"주기: {cycle_count}")
            print()

            print("[전체 힘]")

            if force_values is not None:
                fx, fy, fz, mx, my, mz = force_values

                print(f"Fx : {fx:8.3f} N")
                print(f"Fy : {fy:8.3f} N")
                print(f"Fz : {fz:8.3f} N")
                print(f"Mx : {mx:8.3f} Nm")
                print(f"My : {my:8.3f} Nm")
                print(f"Mz : {mz:8.3f} Nm")
                print()
                print(
                    f"[힘 크기] {force_magnitude:.3f} N"
                )
            else:
                print("힘 센서 읽기 실패")

            print()
            print("[그리퍼]")

            if do2 is not None and do3 is not None:
                print(f"DO 2 : {do2}")
                print(f"DO 3 : {do3}")
                print(f"상태 : {gripper_state}")
            else:
                print("디지털 출력 읽기 실패")

            print()
            print("Ctrl+C를 누르면 종료됩니다.")
            print("====================================")
            print(flush=True)

            rclpy.spin_once(
                node,
                timeout_sec=0.01,
            )

            time.sleep(PUBLISH_INTERVAL)

    except KeyboardInterrupt:
        node.get_logger().info(
            "센서 토픽 발행을 종료합니다."
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()