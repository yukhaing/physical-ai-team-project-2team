from __future__ import annotations

from dataclasses import dataclass, field
import time

VALID_CLASSES = {"normal", "defect"}


@dataclass
class Mission:
    """YOLO class 신호 -> receiving zone 이동 -> OMX box-placed 신호 -> 목적지 이동
    -> 목적지에서 DWELL_SECONDS 대기 -> 복귀.

    상태: WAIT -> MOVE_TO_RECEIVING -> WAIT_FOR_BOX -> MOVE_TO_ZONE -> AT_DESTINATION
          -> RETURN_TO_START -> WAIT

    이 클래스는 이동 로직을 전혀 모릅니다. 외부(YOLO/OMX 통신 계층)가
    on_yolo_class()/on_omx_box_placed()를 호출하고, 주행 루프는 target_zone()이
    가리키는 곳으로 이동한 뒤 도착 시 notify_arrived()를 호출하는 식으로 씁니다.
    AT_DESTINATION은 시간이 지나면 자동으로 넘어가므로, 주행 루프가 매 프레임
    tick()을 호출해줘야 합니다.
    WAIT/WAIT_FOR_BOX가 아닐 때 들어오는 신호는 조용히 무시합니다(미션 도중
    중복/순서 어긋난 신호로 상태가 깨지지 않도록).
    """

    zones: dict[str, tuple[float, float]]
    state: str = "WAIT"
    box_class: str | None = None
    state_started: float = field(default_factory=time.monotonic)
    DWELL_SECONDS: float = 5.0

    def on_yolo_class(self, class_name: str) -> None:
        if class_name not in VALID_CLASSES:
            raise ValueError(f"unknown class: {class_name}")
        if self.state != "WAIT":
            return
        self.box_class = class_name
        self._set_state("MOVE_TO_RECEIVING")

    def on_omx_box_placed(self) -> None:
        if self.state != "WAIT_FOR_BOX":
            return
        self._set_state("MOVE_TO_ZONE")

    def target_zone(self) -> tuple[float, float] | None:
        if self.state == "MOVE_TO_RECEIVING":
            return self.zones["receiving"]
        if self.state == "MOVE_TO_ZONE":
            return self.zones[self.box_class]
        if self.state == "RETURN_TO_START":
            return self.zones["start"]
        return None

    def notify_arrived(self) -> None:
        if self.state == "MOVE_TO_RECEIVING":
            self._set_state("WAIT_FOR_BOX")
        elif self.state == "MOVE_TO_ZONE":
            self._set_state("AT_DESTINATION")
        elif self.state == "RETURN_TO_START":
            self.box_class = None
            self._set_state("WAIT")

    def tick(self) -> None:
        """매 프레임 호출: AT_DESTINATION에서 DWELL_SECONDS가 지나면 자동으로 복귀를 시작합니다."""
        if self.state == "AT_DESTINATION" and time.monotonic() - self.state_started >= self.DWELL_SECONDS:
            self._set_state("RETURN_TO_START")

    def dwell_remaining(self) -> float:
        """AT_DESTINATION에서 남은 대기 시간(초). 그 상태가 아니면 0."""
        if self.state != "AT_DESTINATION":
            return 0.0
        return max(0.0, self.DWELL_SECONDS - (time.monotonic() - self.state_started))

    def _set_state(self, new_state: str) -> None:
        self.state = new_state
        self.state_started = time.monotonic()
