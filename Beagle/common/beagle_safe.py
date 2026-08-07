"""이전 저장소(실습코드_정답포함)와의 호환 모듈.
옛 코드는 `from common.beagle_safe import SafeBeagle`로 로봇 래퍼를
불러온다. 이 저장소에서는 같은 클래스가 common/robot.py에 있으므로
그대로 다시 내보낸다(re-export). 인터페이스는 동일하다.
"""
from .robot import MockBeagle, SafeBeagle, build_scene

Segment = tuple[float, float, float, float]

__all__ = ["MockBeagle", "SafeBeagle", "build_scene", "Segment"]