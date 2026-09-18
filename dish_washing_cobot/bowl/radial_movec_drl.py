from __future__ import annotations

import time

import rclpy
from dsr_msgs2.srv import DrlStart, GetDrlState, GetRobotSystem
from rclpy.node import Node


ROBOT_NAMESPACE = "dsr01"
ROS_NODE_NAME = "rokey_radial_movec"

# DrlStart와 GetRobotSystem의 robot_system 값
ROBOT_SYSTEM_REAL = 0
ROBOT_SYSTEM_VIRTUAL = 1

# 서비스 및 프로그램 실행이 무한정 대기하지 않도록 제한한다.
SERVICE_WAIT_TIMEOUT_SEC = 10.0
SERVICE_CALL_TIMEOUT_SEC = 5.0
PROGRAM_START_TIMEOUT_SEC = 10.0
PROGRAM_FINISH_TIMEOUT_SEC = 180.0
STATE_POLL_INTERVAL_SEC = 0.1

# GetDrlState 응답값
DRL_STATE_PLAY = 0
DRL_STATE_STOP = 1
DRL_STATE_HOLD = 2
DRL_STATE_LAST = 3

# DSR_ROBOT2.movec()의 ROS 2 서비스에는 ori/app_type 필드가 없으므로,
# 원주 구속 자세(DR_MV_ORI_RADIAL)를 보존하기 위해 네이티브 DRL로 실행한다.
# posx의 A, B, C는 Z-Y-Z Euler 자세값이며 입력값을 그대로 사용한다.
#
# 중요: 컨트롤러의 DRL 파서는 전송 문자열 안의 한글 주석을 정상 처리하지
# 못할 수 있다. 따라서 Python 주석은 한글로 작성하되, 아래 DRL 전송 문자열은
# ASCII 문자만 사용하고 공식 API 예제와 동일하게 CRLF 줄바꿈을 사용한다.
DRL_CODE = (
    "set_singularity_handling(DR_AVOID)\r\n"
    "set_velj(60.0)\r\n"
    "set_accj(100.0)\r\n"
    "set_velx(250.0, 80.625)\r\n"
    "set_accx(1000.0, 322.5)\r\n"
    "gLoop160904300 = 0\r\n"
    "while gLoop160904300 < 1:\r\n"
    "    movej(posj(23.04, -4.80, 102.48, -19.78, 63.68, 210.21), "
    "vel=60.0, acc=100.0, radius=0.00, "
    "ra=DR_MV_RA_DUPLICATE)\r\n"
    "    movec(posx(499.06, 7.61, 219.67, 160.15, -153.33, -18.51), "
    "posx(506.92, 181.41, 194.82, 160.73, -153.53, -18.21), "
    "vel=[250.0, 80.625], acc=[1000.0, 322.5], radius=0.00, "
    "ref=DR_BASE, angle=[12.00, 100.00], "
    "ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_RADIAL, "
    "app_type=DR_MV_APP_NONE)\r\n"
    "    gLoop160904300 = gLoop160904300 + 1\r\n"
)


class RadialMovecRunner(Node):
    """현재 로봇 시스템을 확인하고 DRL 시작·상태 확인을 담당하는 ROS 2 노드."""

    def __init__(self, drl_code: str = DRL_CODE, quiet: bool = False) -> None:
        super().__init__(ROS_NODE_NAME, namespace=ROBOT_NAMESPACE)
        self._drl_code = drl_code
        self._quiet = quiet

        self._start_client = self.create_client(
            DrlStart,
            f"/{ROBOT_NAMESPACE}/drl/drl_start",
        )
        self._state_client = self.create_client(
            GetDrlState,
            f"/{ROBOT_NAMESPACE}/drl/get_drl_state",
        )
        self._robot_system_client = self.create_client(
            GetRobotSystem,
            f"/{ROBOT_NAMESPACE}/system/get_robot_system",
        )

        self._program_accepted = False

    def _wait_for_services(self) -> None:
        """필요한 DRL 서비스가 모두 준비될 때까지만 제한 시간 동안 기다린다."""
        services = (
            ("drl_start", self._start_client),
            ("get_drl_state", self._state_client),
            ("get_robot_system", self._robot_system_client),
        )

        for service_name, client in services:
            if not client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
                raise RuntimeError(
                    f"{service_name} 서비스를 {SERVICE_WAIT_TIMEOUT_SEC:.1f}초 안에 "
                    "찾지 못했습니다. bringup 모드와 namespace를 확인하세요."
                )

    def _call_service(self, client, request, service_name: str):
        """비동기 서비스 요청을 제한 시간 안에서 동기적으로 완료한다."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=SERVICE_CALL_TIMEOUT_SEC,
        )

        if not future.done():
            future.cancel()
            raise TimeoutError(
                f"{service_name} 서비스 응답 시간이 "
                f"{SERVICE_CALL_TIMEOUT_SEC:.1f}초를 초과했습니다."
            )

        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{service_name} 서비스 호출 실패: {exception}")

        response = future.result()
        if response is None:
            raise RuntimeError(f"{service_name} 서비스 응답이 없습니다.")

        return response

    def _get_drl_state(self) -> int:
        response = self._call_service(
            self._state_client,
            GetDrlState.Request(),
            "get_drl_state",
        )
        if not response.success:
            raise RuntimeError("get_drl_state 서비스가 실패를 반환했습니다.")
        return int(response.drl_state)

    def _get_robot_system(self) -> int:
        """현재 컨트롤러가 실로봇인지 가상 로봇인지 확인한다."""
        response = self._call_service(
            self._robot_system_client,
            GetRobotSystem.Request(),
            "get_robot_system",
        )
        if not response.success:
            raise RuntimeError("get_robot_system 서비스가 실패를 반환했습니다.")

        robot_system = int(response.robot_system)
        if robot_system not in (ROBOT_SYSTEM_REAL, ROBOT_SYSTEM_VIRTUAL):
            raise RuntimeError(f"알 수 없는 robot_system 값입니다: {robot_system}")
        return robot_system

    def _wait_without_sleep(self, duration_sec: float) -> None:
        """일반 Python 대기 함수 대신 ROS 2 이벤트 처리를 유지하며 기다린다."""
        deadline = time.monotonic() + duration_sec
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=min(remaining, duration_sec))

    def _wait_for_program_completion(self) -> None:
        """PLAY 확인 후 STOP까지 기다리되, 시작·완료 시간을 각각 제한한다."""
        start_deadline = time.monotonic() + PROGRAM_START_TIMEOUT_SEC
        finish_deadline = time.monotonic() + PROGRAM_FINISH_TIMEOUT_SEC
        play_observed = False
        hold_logged = False

        while rclpy.ok():
            now = time.monotonic()
            if not play_observed and now >= start_deadline:
                raise TimeoutError(
                    f"DRL 프로그램이 {PROGRAM_START_TIMEOUT_SEC:.1f}초 안에 "
                    "PLAY 상태로 진입하지 않았습니다."
                )
            if now >= finish_deadline:
                raise TimeoutError(
                    f"DRL 프로그램 실행 시간이 {PROGRAM_FINISH_TIMEOUT_SEC:.1f}초를 "
                    "초과했습니다."
                )

            state = self._get_drl_state()

            if state == DRL_STATE_PLAY:
                if not play_observed:
                    if not self._quiet:
                        self.get_logger().info("DRL 프로그램 실행을 확인했습니다.")
                play_observed = True
                hold_logged = False
            elif state == DRL_STATE_STOP:
                if play_observed:
                    self._program_accepted = False
                    if not self._quiet:
                        self.get_logger().info("DRL 프로그램이 정상 종료되었습니다.")
                    return
            elif state == DRL_STATE_HOLD:
                if not hold_logged:
                    self.get_logger().warning(
                        "DRL 프로그램이 HOLD 상태입니다. 안전 상태와 현장을 확인하세요."
                    )
                    hold_logged = True
            elif state == DRL_STATE_LAST:
                raise RuntimeError("유효하지 않은 DRL 상태값 LAST(3)를 수신했습니다.")
            else:
                raise RuntimeError(f"알 수 없는 DRL 상태값을 수신했습니다: {state}")

            self._wait_without_sleep(STATE_POLL_INTERVAL_SEC)

        raise RuntimeError("ROS 2가 종료되어 DRL 상태 확인을 중단했습니다.")

    def run(self) -> None:
        if not self._drl_code.isascii():
            raise RuntimeError(
                "DRL_CODE에는 컨트롤러 인코딩 오류를 방지하기 위해 "
                "ASCII 문자만 사용할 수 있습니다."
            )

        self._wait_for_services()
        robot_system = self._get_robot_system()
        robot_system_name = (
            "실로봇"
            if robot_system == ROBOT_SYSTEM_REAL
            else "가상 로봇"
        )

        request = DrlStart.Request()
        request.robot_system = robot_system
        request.code = self._drl_code

        if not self._quiet:
            self.get_logger().warning(
                f"{robot_system_name} DRL을 시작합니다. "
                "실로봇에서는 작업 반경을 비우고 비상 정지 수단을 확보하세요."
            )
        response = self._call_service(
            self._start_client,
            request,
            "drl_start",
        )
        if not response.success:
            raise RuntimeError("drl_start 서비스가 DRL 실행 요청을 거부했습니다.")

        self._program_accepted = True
        self._wait_for_program_completion()

    def warn_if_program_may_be_running(self) -> None:
        """자동 정지 명령 없이 노드를 종료한다는 사실만 알린다."""
        if self._program_accepted:
            self.get_logger().warning(
                "자동 DRL 정지 명령을 보내지 않습니다. "
                "DRL 프로그램은 컨트롤러 또는 에뮬레이터에서 계속 실행될 수 있습니다."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadialMovecRunner()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("사용자 중단 요청을 받았습니다.")
        node.warn_if_program_may_be_running()
    except Exception as error:
        node.get_logger().error(f"로봇 시퀀스 오류: {error}")
        node.warn_if_program_may_be_running()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
