"""Deterministic LES and scanner models without ROS dependencies."""

from dataclasses import asdict, dataclass
from typing import Dict, Union


JsonValue = Union[str, bool, int]


@dataclass(frozen=True)
class ReceivingOrder:
    order_id: str
    part_barcode: str
    rack_sku: str


class LesMockModel:
    def __init__(self, order: ReceivingOrder, quality_pass: bool = True) -> None:
        self.order = order
        self.quality_pass = quality_pass
        self.not_pushed_feedback_count = 0
        self.completed = False

    def receive_and_inspect(self) -> Dict[str, JsonValue]:
        return {
            **asdict(self.order),
            "quality_pass": self.quality_pass,
            "accepted": self.quality_pass,
        }

    def dispatch_task(self) -> Dict[str, JsonValue]:
        return {**asdict(self.order), "dispatched": True}

    def feedback_not_pushed(self) -> Dict[str, JsonValue]:
        self.not_pushed_feedback_count += 1
        return {
            "order_id": self.order.order_id,
            "feedback_count": self.not_pushed_feedback_count,
            "repushed": True,
        }

    def complete_task(self) -> Dict[str, JsonValue]:
        self.completed = True
        return {"order_id": self.order.order_id, "completed": True}

    def notify_agv_return(self) -> Dict[str, JsonValue]:
        return {"order_id": self.order.order_id, "agv_return_requested": True}


class ScannerMockModel:
    def __init__(
        self,
        expected_part_barcode: str,
        expected_rack_sku: str,
        mismatch_attempts: int = 1,
    ) -> None:
        if mismatch_attempts < 0 or mismatch_attempts > 19:
            raise ValueError("mismatch_attempts must be in [0, 19]")
        self.expected_part_barcode = expected_part_barcode
        self.expected_rack_sku = expected_rack_sku
        self.mismatch_attempts = mismatch_attempts
        self.part_scan_count = 0

    def reset(self) -> None:
        self.part_scan_count = 0

    def scan_part_box(self) -> Dict[str, JsonValue]:
        self.part_scan_count += 1
        matched = self.part_scan_count > self.mismatch_attempts
        observed = self.expected_part_barcode if matched else (
            f"MISMATCH-{self.part_scan_count:02d}"
        )
        return {
            "expected": self.expected_part_barcode,
            "observed": observed,
            "matched": matched,
            "attempt": self.part_scan_count,
        }

    def scan_rack_sku(self) -> Dict[str, JsonValue]:
        return {
            "expected": self.expected_rack_sku,
            "observed": self.expected_rack_sku,
            "matched": True,
        }
