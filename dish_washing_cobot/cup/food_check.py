"""컵 잔여물 판정 및 배출(털기).

독립 ROS2 노드가 아니다. CupController 가 logger 를 넘겨 호출한다.

원본(`cup_food_check.py`)과 다른 점:
  - dsr 모듈을 인자로 받던 것을 없애고 직접 import 한다(plate/bowl 과 동일)
  - 모션은 common.motion_guard 의 가드된 movej 를 쓴다 -> 정지 요청이 오면
    털기 루프 중간에서도 즉시 끊긴다
  - 하드코딩돼 있던 털기 자세 2개를 waypoints.py 로 옮겼다
동작 순서와 좌표/속도/판정식은 원본 그대로다.
"""

import time

from DSR_ROBOT2 import get_tool_force, DR_BASE

# 모션은 정지 가드를 거친 버전을 쓴다
from common.motion_guard import movej

from . import config as cfg
from . import waypoints as wp


def _log(logger, message):
    """ROS logger 가 있으면 INFO, 없으면 표준 출력."""
    if logger is None:
        print(message)
    else:
        logger.info(message)


def _motion_failed(result):
    """두산 모션 함수의 음수 반환값을 실패로 본다."""
    return (
        isinstance(result, (int, float))
        and not isinstance(result, bool)
        and result < 0
    )


def _shake_cup(logger=None):
    """컵을 배출 방향으로 기울여 위아래로 흔든 뒤 판정 위치로 복귀한다."""
    _log(logger, "1단계: 컵을 잔여물 배출 자세로 기울입니다.")
    result = movej(wp.CUP_SHAKE_DOWN_J,
                   vel=cfg.FOOD_TILT_VEL, acc=cfg.FOOD_TILT_ACC)
    if _motion_failed(result):
        raise RuntimeError(f"잔여물 배출 자세 이동 실패: 반환값={result}")

    _log(logger, f"2단계: 위아래 털기 {cfg.FOOD_SHAKE_COUNT}회를 시작합니다.")
    for index in range(cfg.FOOD_SHAKE_COUNT):
        _log(logger, f"털기 {index + 1}/{cfg.FOOD_SHAKE_COUNT} - 위")
        result = movej(wp.CUP_SHAKE_UP_J,
                       vel=cfg.FOOD_SHAKE_VEL, acc=cfg.FOOD_SHAKE_ACC)
        if _motion_failed(result):
            raise RuntimeError(f"컵 위쪽 털기 이동 실패: 반환값={result}")

        _log(logger, f"털기 {index + 1}/{cfg.FOOD_SHAKE_COUNT} - 아래")
        result = movej(wp.CUP_SHAKE_DOWN_J,
                       vel=cfg.FOOD_SHAKE_VEL, acc=cfg.FOOD_SHAKE_ACC)
        if _motion_failed(result):
            raise RuntimeError(f"컵 아래쪽 털기 이동 실패: 반환값={result}")

    _log(logger, "3단계: CUP_STEP_4_J 로 복귀합니다.")
    result = movej(wp.CUP_STEP_4_J,
                   vel=cfg.FOOD_RETURN_VEL, acc=cfg.FOOD_RETURN_ACC)
    if _motion_failed(result):
        raise RuntimeError(f"CUP_STEP_4_J 복귀 실패: 반환값={result}")


def run_cup_food_check(logger=None):
    """CUP_STEP_4_J 자세에서 평균 Fz 를 측정해 잔여물 유무를 판정한다.

    반환값:
        True  — 잔여물 감지 후 배출·털기 완료
        False — 미감지, CUP_STEP_4_J 자세 유지
    """
    _log(logger, "컵 잔여물 무게 측정을 시작합니다.")

    # 이동 직후 진동이 평균값에 섞이지 않도록 안정화 시간을 둔다.
    time.sleep(cfg.FOOD_FORCE_SETTLE)

    total_force = [0.0] * 6
    for _ in range(cfg.FOOD_FORCE_SAMPLES):
        measured = get_tool_force(DR_BASE)
        for axis in range(6):
            total_force[axis] += float(measured[axis])
        time.sleep(cfg.FOOD_FORCE_SAMPLE_INTERVAL)

    average = [value / cfg.FOOD_FORCE_SAMPLES for value in total_force]
    current_fz = average[2]
    food_exists = current_fz <= cfg.FOOD_FZ_THRESHOLD

    _log(logger,
         "평균 힘 [Fx, Fy, Fz, Mx, My, Mz] = "
         f"{[round(value, 3) for value in average]}")
    _log(logger, f"현재 Fz: {current_fz:.3f} N")
    _log(logger, f"판정 임계값: {cfg.FOOD_FZ_THRESHOLD:.3f} N")

    if not food_exists:
        _log(logger, "판정: 잔여물 없음 - 털기를 생략합니다.")
        return False

    _log(logger, "판정: 잔여물 있음 - 배출 동작을 시작합니다.")
    _shake_cup(logger)
    _log(logger, "잔여물 배출 후 CUP_STEP_4_J 복귀 완료")
    return True