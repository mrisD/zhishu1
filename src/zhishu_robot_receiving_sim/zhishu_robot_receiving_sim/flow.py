"""Pure receiving workflow state machine shared by the simulator and tests."""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional


class FlowState(str, Enum):
    IDLE = "idle"
    LES_RECEIVE_QC = "les_receive_qc"
    LES_DISPATCH = "les_dispatch"
    AGV_DELIVER = "agv_deliver"
    ROBOT_PICK = "robot_pick"
    PART_BARCODE_SCAN = "part_barcode_scan"
    LES_NOT_PUSHED = "les_not_pushed"
    RACK_SKU_SCAN = "rack_sku_scan"
    ROBOT_PLACE = "robot_place"
    LES_COMPLETE = "les_complete"
    AGV_RETURN = "agv_return"
    RECEIPT_COMPLETE = "receipt_complete"
    FAILED = "failed"


STATE_LABELS: Dict[FlowState, str] = {
    FlowState.IDLE: "等待任务",
    FlowState.LES_RECEIVE_QC: "LES收货及质检（Mock）",
    FlowState.LES_DISPATCH: "LES推送单据给机器人并叫箱（Mock）",
    FlowState.AGV_DELIVER: "AGV搬运料框至工作站槽位（Gazebo）",
    FlowState.ROBOT_PICK: "机器人从码垛搬运零件箱（MoveIt）",
    FlowState.PART_BARCODE_SCAN: "扫描零件箱条码并匹配单据（Mock）",
    FlowState.LES_NOT_PUSHED: "反馈LES系统是否未推单（Mock）",
    FlowState.RACK_SKU_SCAN: "扫描料框SKU条码（Mock）",
    FlowState.ROBOT_PLACE: "机器人将零件箱放入AGV料框（MoveIt）",
    FlowState.LES_COMPLETE: "机器人反馈LES任务完成（Mock）",
    FlowState.AGV_RETURN: "系统通知AGV载箱返回入库区（Gazebo）",
    FlowState.RECEIPT_COMPLETE: "入库成功，完成收货",
    FlowState.FAILED: "流程失败",
}


def is_subsequence(expected: List[str], observed: List[str]) -> bool:
    """Return true when all expected values occur in order in observed."""
    position = 0
    for value in observed:
        if position < len(expected) and value == expected[position]:
            position += 1
    return position == len(expected)


_NEXT_STATE: Dict[FlowState, FlowState] = {
    FlowState.IDLE: FlowState.LES_RECEIVE_QC,
    FlowState.LES_RECEIVE_QC: FlowState.LES_DISPATCH,
    FlowState.LES_DISPATCH: FlowState.AGV_DELIVER,
    FlowState.AGV_DELIVER: FlowState.ROBOT_PICK,
    FlowState.ROBOT_PICK: FlowState.PART_BARCODE_SCAN,
    FlowState.PART_BARCODE_SCAN: FlowState.RACK_SKU_SCAN,
    FlowState.LES_NOT_PUSHED: FlowState.PART_BARCODE_SCAN,
    FlowState.RACK_SKU_SCAN: FlowState.ROBOT_PLACE,
    FlowState.ROBOT_PLACE: FlowState.LES_COMPLETE,
    FlowState.LES_COMPLETE: FlowState.AGV_RETURN,
    FlowState.AGV_RETURN: FlowState.RECEIPT_COMPLETE,
}


@dataclass
class FlowSnapshot:
    state: FlowState
    label: str
    barcode_attempt: int
    max_barcode_attempts: int
    failure_reason: str


class ReceivingFlow:
    """Validate normal and barcode-retry transitions for one receiving run."""

    def __init__(self, max_barcode_attempts: int = 3) -> None:
        if max_barcode_attempts < 1 or max_barcode_attempts > 20:
            raise ValueError("max_barcode_attempts must be in [1, 20]")
        self.max_barcode_attempts = max_barcode_attempts
        self.reset()

    def reset(self) -> None:
        self.state = FlowState.IDLE
        self.barcode_attempt = 0
        self.failure_reason = ""

    def advance(self) -> FlowState:
        if self.state == FlowState.PART_BARCODE_SCAN:
            raise RuntimeError("use barcode_result while scanning the part box")
        if self.state not in _NEXT_STATE:
            raise RuntimeError(f"cannot advance terminal state {self.state.value}")
        self.state = _NEXT_STATE[self.state]
        if self.state == FlowState.PART_BARCODE_SCAN:
            self.barcode_attempt += 1
        return self.state

    def barcode_result(self, matched: bool) -> FlowState:
        if self.state != FlowState.PART_BARCODE_SCAN:
            raise RuntimeError("barcode_result is only valid while scanning the part box")
        if matched:
            self.state = FlowState.RACK_SKU_SCAN
        elif self.barcode_attempt >= self.max_barcode_attempts:
            self.fail("part barcode did not match before retry limit")
        else:
            self.state = FlowState.LES_NOT_PUSHED
        return self.state

    def fail(self, reason: str) -> FlowState:
        if not reason:
            raise ValueError("failure reason must not be empty")
        self.failure_reason = reason
        self.state = FlowState.FAILED
        return self.state

    def snapshot(self) -> FlowSnapshot:
        return FlowSnapshot(
            state=self.state,
            label=STATE_LABELS[self.state],
            barcode_attempt=self.barcode_attempt,
            max_barcode_attempts=self.max_barcode_attempts,
            failure_reason=self.failure_reason,
        )

    @property
    def terminal(self) -> bool:
        return self.state in (FlowState.RECEIPT_COMPLETE, FlowState.FAILED)

    def expected_next(self) -> Optional[FlowState]:
        return _NEXT_STATE.get(self.state)
