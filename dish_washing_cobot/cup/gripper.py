"""컵용 3단 그리퍼 제어 (디지털 출력 방식).

plate/bowl 의 Gripper 와 **같은 의미**로 맞췄다:
    close()     = 파지  (DO3)
    open()      = 해제  (DO2, 약 105 mm)
    wide_open() = 정렬용 넓은 열기 (DO1, 약 110 mm)

원본 cup 코드는 open() 이 실제로는 파지였다. 통합하면 plate/bowl 과 뜻이
반대라 호출부를 잘못 읽는 순간 컵을 놓치므로 여기서 이름을 통일하고
controller 호출부도 전부 함께 바꿨다. 채널 번호와 배선은 원본 그대로다.

세 출력이 동시에 켜지지 않도록 항상 나머지 채널을 먼저 끈다.
"""

from DSR_ROBOT2 import set_digital_output, wait

from .config import (
    GRIP_WIDE_OPEN_CH, GRIP_OPEN_CH, GRIP_CLOSE_CH, GRIP_WAIT,
)

ON, OFF = 1, 0


class Gripper:
    """DO1=넓게 열기 / DO2=열기 / DO3=닫기 로 동작하는 3단 그리퍼."""

    def __init__(self, node=None):
        self.node = node

    def _wait(self, wait_time=None):
        wait(GRIP_WAIT if wait_time is None else wait_time)

    def close(self, wait_time=None):
        """DO3 — 컵을 잡는다."""
        set_digital_output(GRIP_WIDE_OPEN_CH, OFF)
        set_digital_output(GRIP_OPEN_CH, OFF)
        set_digital_output(GRIP_CLOSE_CH, ON)
        self._wait(wait_time)

    def open(self, wait_time=None):
        """DO2 — 약 105 mm 로 열어 컵을 놓는다."""
        set_digital_output(GRIP_WIDE_OPEN_CH, OFF)
        set_digital_output(GRIP_CLOSE_CH, OFF)
        set_digital_output(GRIP_OPEN_CH, ON)
        self._wait(wait_time)

    def wide_open(self, wait_time=None):
        """DO1 — 약 110 mm 로 넓게 열어 정렬 회전 여유를 만든다."""
        set_digital_output(GRIP_OPEN_CH, OFF)
        set_digital_output(GRIP_CLOSE_CH, OFF)
        set_digital_output(GRIP_WIDE_OPEN_CH, ON)
        self._wait(wait_time)