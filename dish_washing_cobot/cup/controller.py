from time import sleep

from DSR_ROBOT2 import (
    set_tool, set_tcp, get_tool, get_tcp,
    set_velj, set_accj, set_velx, set_accx,
    set_ref_coord,
    task_compliance_ctrl, set_stiffnessx, set_desired_force,
    release_force, release_compliance_ctrl,
    DR_BASE, DR_TOOL, DR_FC_MOD_ABS, DR_MV_MOD_REL, DR_QSTOP,
)
from DR_common2 import posj, posx
from dsr_msgs2.srv import MoveStop

# 모션 함수는 정지 가드를 거친 버전을 쓴다. 호출 지점은 그대로 두고
# import 만 바꾸면 모든 이동이 abort 플래그를 확인한다.
from common import motion_guard
from common.motion_guard import movej, movel, movesj, move_periodic
from common.motion_guard import WorkAborted as CupAborted

from . import config as cfg
from . import food_check
from . import waypoints as wp
from .gripper import Gripper


# CupAborted 는 common.motion_guard.WorkAborted 의 별칭이다.


class CupController:
    """컵 파지 -> 잔여물 판정 -> 겉면/내부/외부 세척 -> 거치대 배치."""

    def __init__(self, node, on_step=None):
        self.node = node
        self.gripper = Gripper(node)
        self._log = node.get_logger()

        # 진행 단계를 바깥(메인 노드)에 알리는 콜백.
        # 컨트롤러는 ROS 통신을 모르고 숫자만 넘긴다.
        self._on_step = on_step

        # 이 배포판에는 stop() 이 없어 motion/move_stop 서비스를 직접 부른다.
        self._stop_cli = node.create_client(MoveStop, "motion/move_stop")

    # ================= 정지 =================
    def stop_motion(self, st_mode=DR_QSTOP):
        """진행 중인 모션을 정지시킨다(응답 대기 없음)."""
        req = MoveStop.Request()
        req.stop_mode = st_mode
        self._stop_cli.call_async(req)

    def request_abort(self):
        """긴급정지 요청. 진행 중인 모션을 끊고 이후 모션을 막는다."""
        motion_guard.set_abort()
        self._log.warn("CupController: Abort requested")
        self.stop_motion()

    def clear_abort(self):
        motion_guard.clear_abort()

    @property
    def aborted(self):
        return motion_guard.is_aborted()

    def _check_abort(self):
        motion_guard.check_abort()

    # ================= 진행 단계 =================
    def _step(self, n):
        """진행 단계를 바깥에 알린다(콜백이 없으면 아무것도 안 함)."""
        if self._on_step is not None:
            self._on_step(n)
        self._log.info(f"[step:cup] {n}")

    # ================= 초기화 =================
    def setup(self):
        """툴/TCP 적용 및 확인, 기본 속도 설정.

        컵은 plate 와 같은 Tool Weight / GripperDA_v1 을 쓴다. 다만 bowl 작업이
        툴/TCP 를 바꿔놓고 끝나므로 작업 시작마다 다시 건다(run_cup_task 첫 줄).
        """
        set_tool(cfg.TOOL_NAME)
        set_tcp(cfg.TCP_NAME)

        act_tool, act_tcp = get_tool(), get_tcp()
        self._log.info(f"active tool={act_tool!r}, tcp={act_tcp!r}")

        if act_tool != cfg.TOOL_NAME or act_tcp != cfg.TCP_NAME:
            if cfg.VIRTUAL_MODE:
                self._log.warn(
                    "Tool/TCP 미적용 상태지만 VIRTUAL_MODE 이므로 진행합니다.")
            else:
                self._log.error(
                    "Tool/TCP 적용 실패. 티치펜던트의 등록 이름을 확인하세요 "
                    f"({cfg.TOOL_NAME!r} / {cfg.TCP_NAME!r}).")
                return False

        set_velj(cfg.VEL_J)
        set_accj(cfg.ACC_J)
        set_velx(cfg.VEL_X, cfg.ACC_X)
        set_accx(cfg.ACC_X, cfg.ACC_X)
        return True

    # ================= 공통 유틸 =================
    @staticmethod
    def _motion_failed(result):
        """두산 모션 함수가 음수를 반환하면 실패로 본다."""
        return (
            isinstance(result, (int, float))
            and not isinstance(result, bool)
            and result < 0
        )

    def _movej(self, label, target, vel=None, acc=None):
        """movej 실행과 반환값 검사를 한 곳에서 처리한다."""
        self._log.info(f" -> {label}")
        result = movej(
            target,
            vel=cfg.VEL_J_SLOW if vel is None else vel,
            acc=cfg.ACC_J_SLOW if acc is None else acc,
        )
        if self._motion_failed(result):
            raise RuntimeError(f"{label} 이동 실패: 반환값={result}")

    def _movesj(self, label, targets, vel=None, acc=None):
        self._log.info(f" -> {label}")
        result = movesj(
            targets,
            vel=cfg.VEL_J_SLOW if vel is None else vel,
            acc=cfg.ACC_J_SLOW if acc is None else acc,
        )
        if self._motion_failed(result):
            raise RuntimeError(f"{label} movesj 실패: 반환값={result}")

    def _move_periodic_rz(self, label, amp_deg, period, atime, repeat):
        """TOOL RZ 왕복 회전. 세척 동작의 공통 형태."""
        self._log.info(f" -> {label}: TOOL RZ ±{amp_deg:.1f}도")
        result = move_periodic(
            amp=[0.0, 0.0, 0.0, 0.0, 0.0, amp_deg],
            period=period,
            atime=atime,
            repeat=repeat,
            ref=DR_TOOL,
        )
        if self._motion_failed(result):
            raise RuntimeError(f"{label} move_periodic 실패: 반환값={result}")

    # ================= 단계 1: 컵 파지 =================
    def pick_cup(self):
        """컵을 파지해 무게 판정 위치(CUP_STEP_4_J)까지 이동한다."""
        self._log.info("START: pick_cup")

        self._movej("CUP_STEP_1_J: 컵 들기 전 1", wp.CUP_STEP_1_J,
                    vel=cfg.VEL_J, acc=cfg.ACC_J)
        self._movej("CUP_STEP_1_TO_2_VIA_J: 안전 경유",
                    wp.CUP_STEP_1_TO_2_VIA_J, vel=cfg.VEL_J, acc=cfg.ACC_J)
        self._movej("CUP_STEP_2_J: 컵 들기 전 2", wp.CUP_STEP_2_J,
                    vel=cfg.VEL_J, acc=cfg.ACC_J)

        self._log.info(" -> 그리퍼 닫기: 컵 파지")
        self.gripper.close()

        self._movej("CUP_STEP_4_J: 무게 판정 자세", wp.CUP_STEP_4_J)
        self._log.info("DONE: pick_cup")

    def return_cup_and_home(self):
        """파지 경로를 역순으로 따라 컵을 내려놓고 HOME 으로 복귀한다."""
        self._log.info("START: return_cup_and_home")

        self._movej("CUP_STEP_2_J: 컵 원위치로 하강", wp.CUP_STEP_2_J)

        self._log.info(" -> 그리퍼 열기: 컵 내려놓기")
        self.gripper.open()

        self._movej("CUP_STEP_1_TO_2_VIA_J: 역순 안전 이탈",
                    wp.CUP_STEP_1_TO_2_VIA_J)
        self._movej("CUP_STEP_1_J: 컵 주변 이탈", wp.CUP_STEP_1_J)
        self._movej("HOME_J: 초기 자세 복귀", wp.HOME_J,
                    vel=cfg.VEL_J, acc=cfg.ACC_J)

        self._log.info("DONE: return_cup_and_home")

    # ================= 단계 3: 손잡이 주변 겉면 세척 =================
    def wash_handle_before_flip(self):
        """컵을 두 방향으로 파지해 손잡이 주변/아래쪽 겉면을 두 번 세척한다.

        끝나면 컵은 원래 바닥 위치에 놓여 있고 그리퍼는 열린 상태다.
        """
        self._log.info("START: wash_handle_before_flip")

        # ---- 첫 번째 방향 파지 ----
        self._movej("CUP_PICK_BEFORE_J: 컵 집기 전 안전 접근",
                    wp.CUP_PICK_BEFORE_J)
        self._movej("CUP_PICK_AFTER_J: 첫 번째 방향 파지 위치",
                    wp.CUP_PICK_AFTER_J)
        self._log.info(" -> 그리퍼 닫기: 첫 번째 방향으로 컵 파지")
        self.gripper.close(wait_time=cfg.REGRIP_ALIGN_WAIT)

        # ---- 1차 겉면 세척 ----
        self._movej("CUP_OUTER_WASH_BELOW_J: 첫 번째 겉면 세척 위치",
                    wp.CUP_OUTER_WASH_BELOW_J)
        self._move_periodic_rz("1차 겉면 세척",
                               cfg.PRE_FLIP_OUTER_RZ_AMP_DEG,
                               cfg.PRE_FLIP_OUTER_PERIOD,
                               cfg.PRE_FLIP_OUTER_ATIME,
                               cfg.PRE_FLIP_OUTER_REPEAT)

        # ---- 방향 정렬: 컵을 원위치에 둔 채 J6 를 30도씩 3회 ----
        # 컵 위로 상승해 한 번에 90도 돌리던 예전 동작은 쓰지 않는다.
        self._movej("CUP_PICK_AFTER_J: 첫 번째 세척 후 컵 원위치",
                    wp.CUP_PICK_AFTER_J)

        base_joints = [float(value) for value in wp.CUP_PICK_AFTER_J]
        for step_index in range(1, cfg.ALIGN_ROTATION_STEPS + 1):
            accumulated_deg = cfg.ALIGN_ROTATION_STEP_DEG * step_index
            tag = f"정렬 {step_index}/{cfg.ALIGN_ROTATION_STEPS}"

            self._log.info(f" -> {tag}: 넓게 열기(DO1, 약 110 mm)")
            self.gripper.wide_open(wait_time=cfg.ALIGN_WIDE_OPEN_WAIT)

            rotated_joints = base_joints.copy()
            rotated_joints[5] += accumulated_deg
            self._movej(f"{tag}: J6 누적 +{accumulated_deg:.1f}도",
                        posj(rotated_joints))

            self._log.info(
                f" -> {tag}: 파지 후 {cfg.ALIGN_CLOSE_WAIT:.2f}s 대기")
            self.gripper.close(wait_time=cfg.ALIGN_CLOSE_WAIT)

        # 세 번째 파지에서 컵을 90도 방향으로 잡은 상태 -> 그대로 상승
        self._movej("CUP_STEP4_REGRIP_SAFE_90_J: 90도 방향 파지 후 안전 상승",
                    wp.CUP_STEP4_REGRIP_SAFE_90_J)

        # ---- 2차 겉면 세척 ----
        self._movej("CUP_OUTER_WASH_BELOW_J: 두 번째 겉면 세척 위치",
                    wp.CUP_OUTER_WASH_BELOW_J)
        self._move_periodic_rz("2차 겉면 세척",
                               cfg.PRE_FLIP_OUTER_RZ_AMP_DEG,
                               cfg.PRE_FLIP_OUTER_PERIOD,
                               cfg.PRE_FLIP_OUTER_ATIME,
                               cfg.PRE_FLIP_OUTER_REPEAT)

        # ---- 컵을 원위치에 놓고 안전 이탈 ----
        self._movej("CUP_PICK_AFTER_J: 두 번째 세척 후 컵 원위치",
                    wp.CUP_PICK_AFTER_J)
        self._log.info(" -> 그리퍼 열기: 컵 원위치에 내려놓기")
        self.gripper.open(wait_time=cfg.REGRIP_ALIGN_WAIT)
        self._movej("CUP_PICK_BEFORE_J: 컵 주변 안전 이탈",
                    wp.CUP_PICK_BEFORE_J)

        self._log.info("DONE: wash_handle_before_flip")

    # ================= 단계 4: 뒤집기 및 수직 재파지 =================
    def flip_and_regrip(self):
        """컵을 뒤집어 내려놓고 수직 방향으로 다시 잡은 뒤 J6 를 정렬한다."""
        self._log.info("START: flip_and_regrip")

        self._movej("CUP_STEP_5_ROTATE_J: 높이 유지 회전",
                    wp.CUP_STEP_5_ROTATE_J,
                    vel=cfg.VEL_J_SLOW * 1.5, acc=cfg.ACC_J_SLOW * 1.5)
        self._movej("CUP_STEP_5_J: 뒤집은 컵 내려놓기", wp.CUP_STEP_5_J)

        self._log.info(" -> 그리퍼 열기: 뒤집은 컵 해제")
        self.gripper.open()

        self._movej("CUP_STEP_6_APPROACH_J: 수직 재파지 경유",
                    wp.CUP_STEP_6_APPROACH_J)
        self._movej("CUP_STEP_6_J: 수직 재파지 위치", wp.CUP_STEP_6_J)

        self._log.info(" -> 그리퍼 닫기: 수직 재파지")
        self.gripper.close(wait_time=cfg.REGRIP_ALIGN_WAIT)

        # CUP_STEP_6_J 에서 컵을 잠시 놓고 J6 를 +90도 돌려 다시 잡은 뒤,
        # 원래 J6 각도로 복귀해 최종 파지한다.
        base_joints = [float(value) for value in wp.CUP_STEP_6_J]
        rotated_joints = base_joints.copy()
        rotated_joints[5] += cfg.REGRIP_ALIGN_ROTATION_DEG

        self._log.info(" -> 그리퍼 열기: 정렬 전 컵 놓기")
        self.gripper.open(wait_time=cfg.REGRIP_ALIGN_WAIT)
        self._movej(f"J6 +{cfg.REGRIP_ALIGN_ROTATION_DEG:.1f}도 정렬 회전",
                    posj(rotated_joints))
        sleep(cfg.REGRIP_ALIGN_WAIT)

        self._log.info(" -> 그리퍼 닫기: 회전 방향에서 컵 잡기")
        self.gripper.close(wait_time=cfg.REGRIP_ALIGN_WAIT)
        self._log.info(" -> 그리퍼 열기: 컵 다시 놓기")
        self.gripper.open(wait_time=cfg.REGRIP_ALIGN_WAIT)

        self._movej("CUP_STEP_6_J: 원래 J6 각도로 복귀", wp.CUP_STEP_6_J)
        sleep(cfg.REGRIP_ALIGN_WAIT)
        self._log.info(" -> 그리퍼 닫기: 원래 방향 최종 파지")
        self.gripper.close(wait_time=cfg.REGRIP_ALIGN_WAIT)

        self._log.info("DONE: flip_and_regrip")

    # ================= 단계 5: 내부 세척 =================
    def _start_inner_force_control(self):
        """컵 내부 세척용 TOOL Z 힘/순응 제어를 시작한다."""
        if not cfg.USE_FORCE_CONTROL:
            self._log.info("컵 내부 힘/순응 제어 비활성화")
            return False

        if cfg.VIRTUAL_MODE:
            self._log.warn("VIRTUAL_MODE 에서는 힘/순응 제어를 건너뜁니다.")
            return False

        compliance_started = False
        force_vector = [0.0, 0.0,
                        cfg.INNER_FORCE_SIGN * cfg.INNER_FORCE_N,
                        0.0, 0.0, 0.0]
        force_direction = [0, 0, 1, 0, 0, 0]

        try:
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl()
            compliance_started = True
            set_stiffnessx(cfg.INNER_STIFFNESS, time=cfg.FORCE_RAMP)
            set_desired_force(force_vector, force_direction,
                              time=cfg.FORCE_RAMP, mod=DR_FC_MOD_ABS)
            sleep(cfg.FORCE_RAMP + cfg.FORCE_SETTLE)
            return True

        except Exception:
            # 시작 중 오류가 나도 순응 상태가 남지 않도록 즉시 정리한다.
            if compliance_started:
                try:
                    release_force(time=0.0)
                except Exception:
                    pass
                try:
                    release_compliance_ctrl()
                except Exception:
                    pass
            try:
                set_ref_coord(DR_BASE)
            except Exception:
                pass
            raise

    def _stop_force_control(self):
        """힘/순응 제어를 해제하고 기준 좌표계를 BASE 로 되돌린다."""
        try:
            release_force(time=cfg.FORCE_RELEASE)
            sleep(cfg.FORCE_RELEASE)
        except Exception as error:
            self._log.warn(f"release_force 실패: {error}")

        try:
            release_compliance_ctrl()
        except Exception as error:
            self._log.warn(f"release_compliance_ctrl 실패: {error}")
        finally:
            try:
                set_ref_coord(DR_BASE)
            except Exception as error:
                self._log.warn(f"BASE 좌표계 복구 실패: {error}")

    def wash_inner(self):
        """수세미 위로 이동 -> 컵 내부에 삽입 -> 힘 유지하며 RZ 왕복."""
        self._log.info("START: wash_inner")

        self._movesj("수세미 위 접근",
                     [wp.CUP_STEP_7_APPROACH_J, wp.CUP_STEP_7_J],
                     vel=cfg.VEL_J_SLOW * 1.8, acc=cfg.ACC_J_SLOW * 1.8)
        self._movej("CUP_STEP_8_J: 컵 내부에 수세미 삽입", wp.CUP_STEP_8_J)
        self._movej("CUP_STEP_10_J: 내부 세척 위치", wp.CUP_STEP_10_J)

        force_enabled = self._start_inner_force_control()
        try:
            self._move_periodic_rz("내부 세척",
                                   cfg.INNER_PERIODIC_RZ_AMP_DEG,
                                   cfg.INNER_PERIODIC_PERIOD,
                                   cfg.INNER_PERIODIC_ATIME,
                                   cfg.INNER_PERIODIC_REPEAT)
        finally:
            # 정지(CupAborted)로 빠져나가도 힘 제어는 반드시 푼다.
            if force_enabled:
                self._stop_force_control()

        self._log.info("DONE: wash_inner")

    # ================= 단계 6: 외부 세척 =================
    def wash_outer(self):
        """수세미에서 안전하게 빠져나와 외부 세척 위치에서 RZ 왕복."""
        self._log.info("START: wash_outer")

        # 수세미 이탈은 충돌 방지용 전환 동작이라 별도 UI 단계로 쓰지 않는다.
        self._movej("CUP_STEP_11_J: 외부 세척 전 안전 이탈", wp.CUP_STEP_11_J)

        self._movesj("외부 세척 접근",
                     [wp.CUP_STEP_9_APPROACH_J, wp.CUP_STEP_9_J])
        self._move_periodic_rz("외부 세척",
                               cfg.OUTER_PERIODIC_RZ_AMP_DEG,
                               cfg.OUTER_PERIODIC_PERIOD,
                               cfg.OUTER_PERIODIC_ATIME,
                               cfg.OUTER_PERIODIC_REPEAT)

        self._log.info("DONE: wash_outer")

    # ================= 단계 7: 최종 배치 =================
    def place_cup(self):
        """BASE 상대 이동 2회로 거치대 위로 간 뒤 고정 거리 하강해 놓은 후 HOME 위치로 복귀한다.

        주의: 접촉 감지 없이 고정 거리로 내려간다. 거치대 높이가 바뀌면
        충돌하거나 컵이 공중에서 놓인다(cfg.FINAL_PLACE_DESCENT_MM).
        """
        self._log.info("START: place_cup")

        result = movel(wp.PLACE_RELATIVE_X,
                       vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
                       ref=DR_BASE, mod=DR_MV_MOD_REL)
        if self._motion_failed(result):
            raise RuntimeError(f"1차 상대 movel 실패: 반환값={result}")

        result = movel(wp.PLACE_RELATIVE_AFTER_X,
                       vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
                       ref=DR_BASE, mod=DR_MV_MOD_REL)
        if self._motion_failed(result):
            raise RuntimeError(f"2차 상대 movel 실패: 반환값={result}")

        # 현재 자세에서 BASE -Z 로 고정 거리 하강(회전 자세 유지).
        final_descent = posx([0.0, 0.0, -cfg.FINAL_PLACE_DESCENT_MM,
                              0.0, 0.0, 0.0])
        self._log.info(
            f" -> 최종 거치 하강: BASE Z -{cfg.FINAL_PLACE_DESCENT_MM:.1f} mm")
        result = movel(final_descent,
                       vel=cfg.FINAL_PLACE_DESCENT_VEL,
                       acc=cfg.FINAL_PLACE_DESCENT_ACC,
                       ref=DR_BASE, mod=DR_MV_MOD_REL)
        if self._motion_failed(result):
            raise RuntimeError(f"최종 BASE -Z 상대 movel 실패: 반환값={result}")

        self._log.info(" -> 그리퍼 열기: 하강 완료 후 컵 놓기")
        self.gripper.open(wait_time=cfg.FINAL_PLACE_GRIPPER_WAIT)

        # 배치 위치에서 거치대와의 충돌을 피하기 위해 안전하게 상승 후 HOME 복귀
        ascent_lift = posx([0.0, 0.0, cfg.FINAL_PLACE_DESCENT_MM,
                            0.0, 0.0, 0.0])
        self._log.info(" -> 거치 후 안전 이탈: BASE Z 상승")
        result = movel(ascent_lift,
                       vel=cfg.VEL_X_SLOW, acc=cfg.ACC_X_SLOW,
                       ref=DR_BASE, mod=DR_MV_MOD_REL)
        if self._motion_failed(result):
            raise RuntimeError(f"거치 후 이탈 movel 실패: 반환값={result}")

        self._movej("HOME_J: 최종 초기 자세 복귀", wp.HOME_J,
                    vel=cfg.VEL_J, acc=cfg.ACC_J)

        self._log.info("DONE: place_cup")

    # ================= 공정 =================
    def run_weight_check_cycle(self):
        """원본 실행번호 4 — 진행 단계 1~2.

        반환값: 잔여물 감지 여부(True/False)
        """
        self._log.info("########## Cup: weight check START ##########")

        self._check_abort()
        self._step(1)                       # 컵 파지
        self.pick_cup()

        self._check_abort()
        self._step(2)                       # 잔여물 판정 및 배출
        food_exists = food_check.run_cup_food_check(self._log)
        self._log.info(
            "잔여물 배출 완료" if food_exists else "잔여물 없음 - 배출 생략")
        self.return_cup_and_home()

        self._log.info(
            f"########## Cup: weight check DONE ({food_exists}) ##########")
        return food_exists

    def run_full_wash_flow(self):
        """원본 실행번호 12 — 진행 단계 3~7.

        실행번호 4 에서 이미 판정을 마쳤으므로 여기서는 잔여물 판정을
        다시 하지 않는다. 마지막 단계(7)에서 컵을 놓은 후 HOME 으로 복귀한다.
        """
        self._log.info("########## Cup: full wash START ##########")

        self._check_abort()
        self._step(3)                       # 손잡이 주변 겉면 세척
        self.wash_handle_before_flip()

        self._check_abort()
        self._step(4)                       # 컵 뒤집기 및 재파지
        self.pick_cup()                     # 세척용 재파지(판정은 생략)
        self.flip_and_regrip()

        self._check_abort()
        self._step(5)                       # 컵 내부 세척
        self.wash_inner()

        self._check_abort()
        self._step(6)                       # 컵 외부 세척
        self.wash_outer()

        self._check_abort()
        self._step(7)                       # 최종 배치 (내려놓기 후 HOME 복귀)
        self.place_cup()

        self._log.info("########## Cup: full wash DONE ##########")
        return True

    def run_cup_task(self):
        """전체 컵 처리. 메인 노드가 부르는 진입점.

        진행 단계:
          1 컵 파지 / 2 잔여물 판정·배출 / 3 손잡이 주변 세척
          4 뒤집기·재파지 / 5 내부 세척 / 6 외부 세척 / 7 최종 배치
          0 완료(대기)

        중단 요청(request_abort)이 오면 CupAborted 를 던진다.
        툴/TCP 는 plate 와 같지만 bowl 작업이 바꿔놓을 수 있어 여기서 다시 건다.
        """
        self.clear_abort()
        self._log.info("########## Cup task: START ##########")

        if not self.setup():
            raise RuntimeError("컵 Tool/TCP 설정 실패 - 작업을 시작하지 않는다")

        first_check = self.run_weight_check_cycle()
        self.run_full_wash_flow()

        self._step(0)                       # 완료 -> 대기
        self._log.info(
            f"########## Cup task: COMPLETE (food={first_check}) ##########")
