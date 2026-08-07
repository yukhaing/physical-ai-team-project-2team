from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""팀별 최종 미션 템플릿.

1) sense()에서 센서를 읽고 단위를 통일
2) decide()에 상태와 우선순위 작성
3) act()에서 상태별 제한된 출력을 생성
4) verify()에서 timeout·목표·센서 실패를 확인
"""

from dataclasses import dataclass
import time


@dataclass(slots=True)
class MissionData:
    sensor_ok: bool
    front_mm: float
    left_mm: float
    right_mm: float
    camera_label: str
    camera_score: float


class Mission:
    def __init__(self) -> None:
        self.state="WAIT"
        self.state_started=time.monotonic()

    def sense(self) -> MissionData:
        # TODO: 실제 Beagle/카메라 또는 로그 재생 데이터를 연결하세요.
        return MissionData(True,500,400,420,"none",0.0)

    def decide(self,data:MissionData)->str:
        # TODO: SENSOR_FAIL > EMERGENCY > CAMERA_EVENT > NAVIGATION 우선순위를 작성하세요.
        return "WAIT"

    def act(self,state:str,data:MissionData)->tuple[float,float]:
        # TODO: 상태별 바퀴 명령. 알 수 없는 상태는 반드시 (0,0).
        return 0.0,0.0

    def verify(self,state:str,data:MissionData)->None:
        if time.monotonic()-self.state_started>8.0 and state not in {"WAIT","STOP","GOAL"}:
            self.state="TIMEOUT_STOP"

    def step(self)->tuple[str,float,float]:
        data=self.sense(); new_state=self.decide(data)
        if new_state!=self.state: self.state=new_state; self.state_started=time.monotonic()
        left,right=self.act(self.state,data); self.verify(self.state,data)
        return self.state,left,right


if __name__ == "__main__": print(Mission().step())
