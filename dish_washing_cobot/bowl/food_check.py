"""그릇의 음식물 판정 및 배출 동작.
"""

import time

import DSR_ROBOT2 as dsr


# ================= 음식물 판정 기준 =================
# Fz >= -2.0 N: 음식물 없음
# Fz <= -2.1 N: 음식물 있음
# -2.1 N < Fz < -2.0 N: 경계 구간이므로 재측정
NO_FOOD_FZ_THRESHOLD = -2.0
FOOD_FZ_THRESHOLD = -2.1

SAMPLES = 50
SAMPLE_INTERVAL = 0.05
RETRY_WAIT = 1.0


# ================= 동작 속도 =================
MOVE_VEL = 80.0
MOVE_ACC = 60.0

ROTATE_VEL = 60.0
ROTATE_ACC = 45.0

SHAKE_COUNT = 3
SHAKE_J_VEL = 300.0
SHAKE_J_ACC = 400.0

RETURN_VEL = 70.0
RETURN_ACC = 50.0

NEXT_VEL = 80.0
NEXT_ACC = 60.0


def average_force(get_tool_force, ref, samples=SAMPLES):
    """툴 힘을 여러 번 측정하여 6축 평균값을 반환한다."""
    total = [0.0] * 6

    for _ in range(samples):
        force = get_tool_force(ref)
        for index in range(6):
            total[index] += float(force[index])
        time.sleep(SAMPLE_INTERVAL)

    return [value / samples for value in total]


def measure_fz():
    """현재 툴 힘의 평균 Fz를 측정한다."""
    force = average_force(
        dsr.get_tool_force,
        dsr.DR_BASE,
        samples=SAMPLES,
    )
    return force[2]


def judge_food(current_fz):
    """Fz로 음식물 유무를 판정한다.

    Returns:
        True: 음식물 있음
        False: 음식물 없음
        None: 경계 구간
    """
    if current_fz <= FOOD_FZ_THRESHOLD:
        return True
    if current_fz >= NO_FOOD_FZ_THRESHOLD:
        return False
    return None


def check_food():
    """음식물 유무를 측정하고 경계 구간이면 한 번 더 판정한다."""
    print("음식물 무게 측정을 시작합니다.")
    time.sleep(2.0)

    current_fz = measure_fz()
    result = judge_food(current_fz)

    print("\n===== 1차 측정 결과 =====")
    print(f"현재 Fz: {current_fz:.3f} N")
    print(f"Fz 힘 크기: {abs(current_fz):.3f} N")

    if result is True:
        print("판정: 음식물이 올려져 있음")
        return True
    if result is False:
        print("판정: 음식물이 없음")
        return False

    print("판정: 경계 구간")
    print("정확한 판정을 위해 다시 측정합니다.")
    time.sleep(RETRY_WAIT)

    retry_fz = measure_fz()
    final_fz = (current_fz + retry_fz) / 2.0
    final_result = judge_food(final_fz)

    print("\n===== 2차 측정 결과 =====")
    print(f"2차 Fz: {retry_fz:.3f} N")
    print(f"최종 평균 Fz: {final_fz:.3f} N")

    if final_result is True:
        print("최종 판정: 음식물이 올려져 있음")
        return True
    if final_result is False:
        print("최종 판정: 음식물이 없음")
        return False

    print("최종 판정: 판정 불확실")
    print("안전을 위해 털기 동작을 실행하지 않습니다.")
    return None


def move_to_next_position():
    """그릇 바로 위의 다음 접근 위치로 이동한다."""
    pos_pick_up = dsr.posx(
        378.928,
        0.177,
        223.254,
        12.306,
        -155.026,
        20.238,
    )

    print("5단계: 그릇 바로 위 접근 위치로 이동합니다.")
    dsr.movel(pos_pick_up, vel=NEXT_VEL, acc=NEXT_ACC)
    print("그릇 바로 위 접근 위치 이동 완료")


def shake_bowl():
    """음식물 배출 위치로 이동해 그릇을 위아래로 털고 복귀한다."""
    carry_pose = dsr.posx(
        344.786,
        -211.384,
        199.817,
        173.219,
        -153.558,
        0.089,
    )
    shake_down_pose = dsr.posx(
        423.162,
        -265.004,
        269.205,
        32.143,
        -131.415,
        -158.487,
    )
    shake_up_j = dsr.posj(
        -16.416,
        37.514,
        37.789,
        -31.450,
        133.083,
        -224.611,
    )
    shake_down_j = dsr.posj(
        -16.989,
        42.462,
        36.790,
        -31.192,
        130.690,
        -224.085,
    )

    print("1단계: 음식물 털기 시작 위치로 이동합니다.")
    dsr.movel(carry_pose, vel=MOVE_VEL, acc=MOVE_ACC)

    print("2단계: 음식물을 버리는 자세로 이동합니다.")
    dsr.movel(shake_down_pose, vel=ROTATE_VEL, acc=ROTATE_ACC)

    print("3단계: 저속 위아래 털기를 시작합니다.")
    for index in range(SHAKE_COUNT):
        print(f"털기 {index + 1}/{SHAKE_COUNT}")
        dsr.movej(shake_up_j, vel=SHAKE_J_VEL, acc=SHAKE_J_ACC)
        dsr.movej(shake_down_j, vel=SHAKE_J_VEL, acc=SHAKE_J_ACC)

    print("위아래 털기 완료")
    print("4단계: 음식물 털기 시작 자세로 복귀합니다.")
    dsr.movel(carry_pose, vel=RETURN_VEL, acc=RETURN_ACC)


def run_food_check(log=None, publish_callback=None):
    """음식물 판정 후 필요하면 배출하고 다음 접근 위치로 이동한다."""
    if log:
        log.info("음식물 판정 및 배출 시작")

    dsr.set_tool("Tool Weight")
    dsr.set_tcp("GripperDA_v1")

    if publish_callback:
        publish_callback(2)

    food_exists = check_food()
    if food_exists is None:
        raise RuntimeError("음식물 판정이 불확실하여 후속 세척을 중단합니다.")

    if food_exists:
        if publish_callback:
            publish_callback(3)
        if log:
            log.info("음식물 배출 동작을 시작합니다.")
        shake_bowl()
    elif log:
        log.info("음식물 배출 동작을 생략합니다.")

    move_to_next_position()
    if log:
        log.info("음식물 판정 및 배출 완료")
    return food_exists
