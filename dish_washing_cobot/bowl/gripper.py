from DSR_ROBOT2 import set_digital_output, wait
from .config import GRIP_OPEN_CH, GRIP_CLOSE_CH, GRIP_WAIT

ON, OFF = 1, 0


class Gripper:
    """DO2=열기 / DO3=닫기 메커니즘을 수행하는 그리퍼 클래스."""

    def __init__(self, node=None):
        self.node = node

    def open(self, wait_time=None):
        """원본 release() 동일: DO 2: ON, DO 3: OFF, wait(2.0)"""
        set_digital_output(GRIP_OPEN_CH, ON)
        set_digital_output(GRIP_CLOSE_CH, OFF)
        wait(GRIP_WAIT if wait_time is None else wait_time)

    def close(self, wait_time=None):
        """원본 grasp() 동일: DO 2: OFF, DO 3: ON, wait(2.0)"""
        set_digital_output(GRIP_OPEN_CH, OFF)
        set_digital_output(GRIP_CLOSE_CH, ON)
        wait(GRIP_WAIT if wait_time is None else wait_time)