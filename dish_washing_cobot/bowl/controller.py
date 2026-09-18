import math
from DSR_ROBOT2 import (
    set_tool, set_tcp, set_velx, set_accx,
    wait, fkin, DR_BASE, DR_TOOL, DR_AXIS_Z, DR_QSTOP
)
from DR_common2 import posx
from dsr_msgs2.srv import MoveStop
from common import motion_guard
from common.motion_guard import movej, movel, movejx, movec
from common.motion_guard import WorkAborted as BowlAborted

from . import config as cfg
from .gripper import Gripper
from . import food_check
from .waypoints import (
    POS_HOME, POS_PICK_BOWL, POS_PICK_UP, POS_BALL, POS_BALL_RUB,
    POS_SPONGE, POS_SPONGE_POS1, POS_SPONGE_POS2,
    POS_SPIRAL_APPROACH, POS_SPIRAL_CENTER,
    POS_RACK, POS_RACK_WP, POS_RACK_VIA, POS_RACK_VIA2
)
from .radial_movec_drl import RadialMovecRunner

# 그릇 자동 세척의 전체 공정(파지 -> 잔반 확인 -> 내/외부 세척 -> 거치)을 제어하는 컨트롤러
class BowlController:

    def __init__(self, node, on_step=None):
        self.node = node
        self.gripper = Gripper(node)
        self._log = node.get_logger()
        self._on_step = on_step

        # 비동기 정지 서비스 클라이언트
        self._stop_cli = node.create_client(MoveStop, "motion/move_stop")
# ================= [시스템 및 안전 제어 함수] =================
    def stop_motion(self, st_mode=DR_QSTOP):
        """진행 중인 모션을 즉시 정지시킨다."""
        req = MoveStop.Request()
        req.stop_mode = st_mode
        self._stop_cli.call_async(req)

    def report_step(self, status):
        """단계 번호를 main_node의 on_step 콜백으로 전달한다."""
        if self._on_step is not None:
            self._on_step(status)
        print(f" -> [STEP] bowl: {status}")

    def request_abort(self):
        """작업 중단 플래그 설정 및 즉시 로봇 정지 명령 호출"""
        motion_guard.set_abort()
        self._log.warn("BowlController: Abort requested")
        self.stop_motion()

    def clear_abort(self):
        motion_guard.clear_abort()

    @property
    def aborted(self):
        return motion_guard.is_aborted()

    def _check_abort(self):
        """각 모션 시퀀스 넘어가기 전 중단 요청 여부를 확인하여 예외 발생"""
        motion_guard.check_abort()

    def pick_bowl(self, target_pos=POS_PICK_BOWL):
        """[Step 1, 4] 그릇 파지: 안전 높이(POS_PICK_UP)를 경유하여 그릇을 잡고 다시 상승"""
        print("그릇 파지 시작")
        movel(POS_PICK_UP, vel=cfg.VEL_FAST, acc=cfg.ACC_FAST)
        movej(target_pos, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)

        self.gripper.close()
        print("그릇 파지 완료")
        movel(POS_PICK_UP, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)

    def throw(self):
        """[Step 2] 잔반 확인/처리: 외부 vision/비즈니스 로직(food_check) 모듈 실행"""
        food_check.run_food_check(log=self._log)

    def put_bowl(self, ball_pos=POS_BALL):
        """[Step 3] 내부 나선 세척:
        1. 그릇을 원위치에 내려놓음
        2. 세척용 공(Ball) 위치 상단으로 이동 후 파지
        3. 세척 위치(POS_BALL_RUB)로 수직 하강하여 DRL 나선형 모션(amove_spiral)으로 그릇 내부 세척
        4. 사용한 공을 원위치에 반납"""
        print("그릇 내려놓기 및 나선 세척 시작")

        print(" -> 그릇을 집었던 위치에 내려놓는 중...")
        #1. 그릇 원위치 반납
        movej(POS_PICK_BOWL, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
        self.gripper.open()
        movel(POS_PICK_UP, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)

        # 2. 순방향 기구학(fkin)을 통해 공 위치의 TCP 좌표 계산 후, 상단 접근 오프셋(Z+140mm) 생성
        ball_tcp_values = fkin(ball_pos, ref=DR_BASE)
        ball_tcp = posx(*(float(value) for value in ball_tcp_values))
        ball_above = posx(
            ball_tcp[0],
            ball_tcp[1],
            ball_tcp[2] + cfg.BALL_APPROACH_DISTANCE,
            ball_tcp[3],
            ball_tcp[4],
            ball_tcp[5],
        )
        # 공 파지
        print(" -> POS_BALL 상단 140 mm 위치로 직선 이동 중...")
        movel(ball_above, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> POS_BALL 위치에서 공을 집는 중...")
        movej(ball_pos, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
        self.gripper.close()

        # 3. 내부 세척 위치로 상단 경유(movejx) 후 수직 진입
        movel(ball_above, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)
        ball_rub_tcp_values = fkin(POS_BALL_RUB, ref=DR_BASE)
        ball_rub_tcp = posx(*(float(value) for value in ball_rub_tcp_values))
        ball_rub_above = posx(
            ball_rub_tcp[0],
            ball_rub_tcp[1],
            ball_rub_tcp[2] + cfg.SPIRAL_DOWN_DISTANCE,
            ball_rub_tcp[3],
            ball_rub_tcp[4],
            ball_rub_tcp[5],
        )

        print(" -> POS_BALL_RUB 상단 140 mm 위치로 이동 중...")
        movejx(ball_rub_above, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE, sol=2)

        print(" -> TCP 자세를 유지하며 POS_BALL_RUB 위치로 수직 하강 중...")
        movel(ball_rub_tcp, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> 그릇 내부 나선 세척 중...")
        # 베이스 좌표계 Z축 기준 안쪽에서 바깥쪽으로 나선형 이동하는 DRL 명령 생성 및 실행
        spiral_drl_code = (
            f"amove_spiral(rev={cfg.SPIRAL_REVOLUTION:.6f}, "
            f"rmax={cfg.SPIRAL_MAX_RADIUS:.6f}, "
            f"lmax={cfg.SPIRAL_AXIS_DISTANCE:.6f}, "
            f"v={cfg.VEL_SLOW:.6f}, a={cfg.ACC_SLOW:.6f}, "
            f"t={cfg.SPIRAL_DURATION:.6f}, axis=DR_AXIS_Z, ref=DR_BASE)\r\n"
            f"wait({cfg.SPIRAL_DURATION:.6f})\r\n"
        )

        spiral_runner = RadialMovecRunner(drl_code=spiral_drl_code, quiet=True)
        try:
            spiral_runner.run()
        finally:
            spiral_runner.destroy_node()

        print(f" -> 나선 세척 {cfg.SPIRAL_DURATION:.1f}초 완료, 다음 시퀀스로 진행합니다.")

        # 4. 홈 자세 경유 후 공 반납
        print(" -> 홈 위치로 이동 중...")
        movej(POS_HOME, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)

        movel(ball_above, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> POS_BALL 위치에 공을 내려놓는 중...")
        movej(ball_pos, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
        self.gripper.open()

        print("그릇 내부 나선 세척 완료")

    def scrub_bowl_on_sponge(self, sponge_pos=POS_SPONGE):
        """[Step 5] 수세미 세척: 
        수세미 면 위의 세 점(start, pos1, pos2)을 기준으로 3D 원호의 기하학적 정보(법선 벡터, 중심점)를 산출하고, 
        자세를 구속(ORI_TEACH / ORI_RADIAL)한 상태로 전진/후진 원호 이동(movec)을 반복하여 그릇을 문지르는 모션"""
        print(" -> [시작] 수세미 세척 위치로 이동 및 원주 구속 문지르기 모션 수행")

        pos1 = POS_SPONGE_POS1
        pos2 = POS_SPONGE_POS2
        # 백터 기본 연산 내부 함수 (덧셈, 뺄셈, 스칼라곱, 내적, 외적)
        def vector_add(left, right):
            return tuple(left[index] + right[index] for index in range(3))

        def vector_sub(left, right):
            return tuple(left[index] - right[index] for index in range(3))

        def vector_scale(vector, scale):
            return tuple(component * scale for component in vector)

        def dot(left, right):
            return sum(left[index] * right[index] for index in range(3))

        def cross(left, right):
            return (
                left[1] * right[2] - left[2] * right[1],
                left[2] * right[0] - left[0] * right[2],
                left[0] * right[1] - left[1] * right[0],
            )
        # 세 교시점의 TCP 좌표 추출
        sponge_tcp_values = fkin(sponge_pos, ref=DR_BASE)
        sponge_tcp = posx(*(float(value) for value in sponge_tcp_values))
        pos1_tcp_values = fkin(pos1, ref=DR_BASE)
        pos1_tcp = posx(*(float(value) for value in pos1_tcp_values))
        pos2_tcp_values = fkin(pos2, ref=DR_BASE)
        pos2_tcp = posx(*(float(value) for value in pos2_tcp_values))

        start_xyz = tuple(float(sponge_tcp[index]) for index in range(3))
        pos1_xyz = tuple(float(pos1_tcp[index]) for index in range(3))
        pos2_xyz = tuple(float(pos2_tcp[index]) for index in range(3))

        # 외적(Cross Product)을 통해 세 점이 이루는 평면의 법선 벡터 계산
        start_to_pos1 = vector_sub(pos1_xyz, start_xyz)
        start_to_pos2 = vector_sub(pos2_xyz, start_xyz)
        plane_normal = cross(start_to_pos1, start_to_pos2)
        normal_squared = dot(plane_normal, plane_normal)
        if normal_squared < 1e-12:
            raise ValueError("POS_SPONGE, pos1, pos2가 하나의 원호를 정의하지 못합니다.")

        # 원호의 중심점(circle_center) 및 반지름 백터 유도
        center_offset = vector_scale(
            vector_add(
                vector_scale(
                    cross(start_to_pos2, plane_normal),
                    dot(start_to_pos1, start_to_pos1),
                ),
                vector_scale(
                    cross(plane_normal, start_to_pos1),
                    dot(start_to_pos2, start_to_pos2),
                ),
            ),
            1.0 / (2.0 * normal_squared),
        )
        circle_center = vector_add(start_xyz, center_offset)
        normal_length = math.sqrt(normal_squared)
        unit_normal = vector_scale(plane_normal, 1.0 / normal_length)
        start_radius = vector_sub(start_xyz, circle_center)

        # 로드게스 회전 공식(Rodrigues' rotation formula)을 이용해 특정 각도만큼 회전한 원호 위의 좌표 반환
        def circular_pose(angle_degrees):
            angle_radians = math.radians(angle_degrees)
            cosine = math.cos(angle_radians)
            sine = math.sin(angle_radians)
            rotated_radius = vector_add(
                vector_add(
                    vector_scale(start_radius, cosine),
                    vector_scale(cross(unit_normal, start_radius), sine),
                ),
                vector_scale(
                    unit_normal,
                    dot(unit_normal, start_radius) * (1.0 - cosine),
                ),
            )
            xyz = vector_add(circle_center, rotated_radius)
            return posx(xyz[0], xyz[1], xyz[2], sponge_tcp[3], sponge_tcp[4], sponge_tcp[5])

        # 설정된 호 각도(arc_angle)를 기준으로 경유점과 도달점을 생성
        arc_angle = cfg.SPONGE_ARC_ANGLE
        via_angle = arc_angle / 2.0
        forward_via = circular_pose(via_angle)
        forward_end = circular_pose(arc_angle)
        reverse_via = circular_pose(-via_angle)
        reverse_end = circular_pose(-arc_angle)

        def drl_position(name, position):
            values = ", ".join(f"{float(value):.6f}" for value in position)
            return f"{name}({values})"

        # 특이점 회피(DR_AVOID)를 적용하고, 방사형/교시 자세를 유지하며 전진 및 후진 원호 문지르기 DRL 생성
        drl_code = (
            "set_singularity_handling(DR_AVOID)\r\n"
            f"movec({drl_position('posx', forward_via)}, "
            f"{drl_position('posx', forward_end)}, "
            f"vel={cfg.VEL_SLOW:.6f}, acc={cfg.ACC_SLOW:.6f}, radius=0.0, "
            "ref=DR_BASE, "
            "ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_RADIAL, "
            "app_type=DR_MV_APP_NONE)\r\n"
            f"movec({drl_position('posx', forward_via)}, "
            f"{drl_position('posx', sponge_tcp)}, "
            f"vel={cfg.VEL_SLOW:.6f}, acc={cfg.ACC_SLOW:.6f}, radius=0.0, "
            "ref=DR_BASE, "
            "ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_TEACH, "
            "app_type=DR_MV_APP_NONE)\r\n"
            f"movec({drl_position('posx', reverse_via)}, "
            f"{drl_position('posx', reverse_end)}, "
            f"vel={cfg.VEL_SLOW:.6f}, acc={cfg.ACC_SLOW:.6f}, radius=0.0, "
            "ref=DR_BASE, "
            "ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_RADIAL, "
            "app_type=DR_MV_APP_NONE)\r\n"
            f"movec({drl_position('posx', reverse_via)}, "
            f"{drl_position('posx', sponge_tcp)}, "
            f"vel={cfg.VEL_SLOW:.6f}, acc={cfg.ACC_SLOW:.6f}, radius=0.0, "
            "ref=DR_BASE, "
            "ra=DR_MV_RA_DUPLICATE, ori=DR_MV_ORI_TEACH, "
            "app_type=DR_MV_APP_NONE)\r\n"
        )

        movej(sponge_pos, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)

        runner = RadialMovecRunner(drl_code=drl_code, quiet=True)
        try:
            runner.run()
        finally:
            runner.destroy_node()

        print(" -> [완료] 원주 구속 문지르기 동작 완료")

    def spiral_scrub_bowl(self):
        """[Step 6] 외부 나선 세척: 툴 좌표계(DR_TOOL)의 Z축을 기준으로 그릇 외부 면을 따라 나선형으로 움직이며 세척"""
        print("그릇 외부 나선 세척 시작")

        print(" -> 나선 세척 접근 위치로 이동 중...")
        movel(POS_SPIRAL_APPROACH, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> 나선 세척 중심 위치로 이동 중...")
        movel(POS_SPIRAL_CENTER, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        spiral_drl_code = (
            f"amove_spiral(rev={cfg.OUTER_SPIRAL_REVOLUTION:.6f}, "
            f"rmax={cfg.OUTER_SPIRAL_MAX_RADIUS:.6f}, "
            f"lmax={cfg.OUTER_SPIRAL_AXIS_DISTANCE:.6f}, "
            f"v={cfg.VEL_SLOW:.6f}, a={cfg.ACC_SLOW:.6f}, "
            f"t={cfg.OUTER_SPIRAL_DURATION:.6f}, "
            "axis=DR_AXIS_Z, ref=DR_TOOL)\r\n"
            f"wait({cfg.OUTER_SPIRAL_DURATION:.6f})\r\n"
        )

        spiral_runner = RadialMovecRunner(drl_code=spiral_drl_code, quiet=True)
        try:
            spiral_runner.run()
        finally:
            spiral_runner.destroy_node()

        print("그릇 외부 나선 세척 완료")

    def place_bowl_on_rack(self, rack_pos=POS_RACK):
        """[Step 7] 거치대 적재: 주변 구조물 충돌 방지를 위해 3개의 경유점(WP, VIA, VIA2)을 거쳐 상단 80mm 지점에서 수직 하강하여 내려놓음"""
        print("거치대 배치 시작")

        print(" -> POS_RACK_WP 위치로 이동 중...")
        movel(POS_RACK_WP, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> POS_RACK_VIA 위치로 이동 중...")
        movel(POS_RACK_VIA, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> POS_RACK_VIA2 위치로 이동 중...")
        movej(POS_RACK_VIA2, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)

        rack_tcp_values = fkin(rack_pos, ref=DR_BASE)
        rack_tcp = posx(*(float(value) for value in rack_tcp_values))
        rack_above = posx(
            rack_tcp[0],
            rack_tcp[1],
            rack_tcp[2] + 80.0,
            rack_tcp[3],
            rack_tcp[4],
            rack_tcp[5],
        )

        print(" -> POS_RACK 상단 80 mm 위치로 이동 중...")
        movejx(rack_above, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE, sol=2)

        print(" -> POS_RACK 위치로 수직 하강 중...")
        movel(rack_tcp, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> 거치대에 그릇 내려놓는 중...")
        self.gripper.open()

        print(" -> POS_RACK 상단 80 mm 위치로 수직 상승 중...")
        movel(rack_above, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW, ref=DR_BASE)

        print(" -> 조인트 값 기준 홈 자세로 복귀 중...")
        movej(POS_HOME, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
        wait(3.0)

        print("거치 완료 및 로봇 복귀 완료")
    # ================= [메인 오케스트레이션 함수] =================
    def run_bowl_task(self):
        """[메인 루프] 초기 홈 복귀부터 1~7단계의 세척 시퀀스를 순차적으로 지휘합니다
        각 Step 사이에 반드시 _check_abort()를 호출하여 비상 정지나 중단 명령이 있는지 감시합니다"""
        self.clear_abort()
        print("=== [Start] 그릇 세척 모듈 시작 ===")

        self.gripper.open()
        print(" -> [초기화] 홈 자세로 이동합니다.")
        movej(POS_HOME, vel=cfg.VEL_SLOW, acc=cfg.ACC_SLOW)
        wait(2.0)

        set_velx(cfg.VEL_FAST)
        set_accx(cfg.ACC_FAST)

        self._check_abort()
        self.report_step(1)
        self.pick_bowl(POS_PICK_BOWL)

        self._check_abort()
        self.report_step(2)
        self.throw()

        self._check_abort()
        self.report_step(3)
        self.put_bowl(POS_BALL)

        self._check_abort()
        self.report_step(4)
        self.pick_bowl(POS_PICK_BOWL)

        self._check_abort()
        self.report_step(5)
        self.scrub_bowl_on_sponge(POS_SPONGE)

        self._check_abort()
        self.report_step(6)
        self.spiral_scrub_bowl()

        self._check_abort()
        self.report_step(7)
        self.place_bowl_on_rack(POS_RACK)

        print("=== [Success] 모든 시퀀스가 성공적으로 완료되었습니다! ===")
