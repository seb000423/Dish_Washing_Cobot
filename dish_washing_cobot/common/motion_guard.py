"""모션 가드 — 정지(abort) 요청을 모든 모션 호출에 강제한다.

plate/, bowl/, cup/ 컨트롤러가 공용으로 쓰는 모듈이라 common/ 에
둔다. 정지 플래그(_ABORT)는 프로세스 전역 하나뿐이다 — 메인 노드
상태머신이 애초에 한 번에 하나의 작업만 RUN 시키므로 식기별로 나눌
필요가 없다.

배경: request_abort 의 move_stop 은 "지금 도는 모션 하나"만 끊는다.
컨트롤러 시퀀스는 모션이 끝나면 곧장 다음 모션을 부르므로 단계 사이의
_check_abort 만으로는 그 사이 모션들이 정지 후에도 계속 실행된다.

여기서 DSR 모션 함수(movej/movel/movec/amovel/movejx/movesj/move_periodic)를
감싼 대체 함수를 제공한다. 각 호출 전후로 abort 플래그를 확인해:
  - 호출 전  : 정지 후 "다음 모션"이 새로 시작되는 것을 막고
  - 호출 후  : move_stop 으로 끊긴 직후 즉시 WorkAborted 로 빠져나온다

각 컨트롤러(plate/, bowl/, cup/controller.py)와 food_check 는
DSR_ROBOT2 대신 이 모듈에서 모션 함수를 import한다. 호출 지점은
하나도 바꾸지 않아도 전부 가드를 거친다.

주의: DSR_ROBOT2 를 import 하므로 이 모듈도 DR_init.__dsr__node 등록
이후(= 각 컨트롤러 import 시점)에만 import 돼야 한다.
"""

import threading

from DSR_ROBOT2 import (
    movej as _movej,
    movel as _movel,
    movec as _movec,
    amovel as _amovel,
    movejx as _movejx,
)

# movesj / move_periodic 은 cup 공정에서만 쓴다. 배포판에 없을 수도 있으므로
# import 실패가 plate/bowl 까지 막지 않도록 방어적으로 가져온다.
try:
    from DSR_ROBOT2 import movesj as _movesj
except ImportError:                                  # pragma: no cover
    _movesj = None
try:
    from DSR_ROBOT2 import move_periodic as _move_periodic
except ImportError:                                  # pragma: no cover
    _move_periodic = None


class WorkAborted(Exception):
    """긴급정지 요청으로 작업이 중단됨 (식기 종류 공용)."""


_ABORT = threading.Event()


def set_abort():
    """정지 플래그를 세운다. 이후 모든 모션 호출이 WorkAborted 를 던진다."""
    _ABORT.set()


def clear_abort():
    """정지 플래그 해제. 다음 작업 시작 전에 부른다."""
    _ABORT.clear()


def is_aborted():
    return _ABORT.is_set()


def check_abort():
    """정지 요청이 있으면 예외를 던져 작업을 끊는다."""
    if _ABORT.is_set():
        raise WorkAborted("aborted by request")


def _guarded(fn):
    def wrapper(*args, **kwargs):
        check_abort()                  # 정지 후 새 모션 시작 차단
        ret = fn(*args, **kwargs)
        check_abort()                  # 끊긴 모션에서 즉시 탈출
        return ret

    wrapper.__name__ = fn.__name__
    wrapper.__doc__ = f"abort 가드를 거치는 {fn.__name__}"
    return wrapper


def _missing(name):
    """배포판에 없는 모션 함수를 호출하면 그 자리에서 알려준다."""
    def stub(*args, **kwargs):
        raise NotImplementedError(
            f"이 DSR 배포판에는 {name} 이 없다. 해당 공정을 쓸 수 없다.")

    stub.__name__ = name
    return stub


movej = _guarded(_movej)
movel = _guarded(_movel)
movec = _guarded(_movec)
amovel = _guarded(_amovel)
movejx = _guarded(_movejx)

# movesj: 여러 관절 자세를 한 번에 잇는 이동 (cup 수세미 접근 등)
movesj = _guarded(_movesj) if _movesj is not None else _missing("movesj")

# move_periodic: 주기 왕복 회전 (cup 겉면/내부 세척).
# 주의 - repeat x period 동안 한 번의 호출로 도는 긴 모션이라, 정지 요청은
# move_stop(QSTOP) 이 모션을 끊고 반환된 "직후"의 사후 확인에서 잡힌다.
move_periodic = (_guarded(_move_periodic) if _move_periodic is not None
                 else _missing("move_periodic"))