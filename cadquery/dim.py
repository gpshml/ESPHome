"""Locked cabin-bed dimensions — mirror of cabin_bed_model.js `DIM`.

Do not invent values. Source of truth: LOCKED-IN-SPEC.md + cabin_bed_model.js.
Coordinate system (mm):
  Origin = back-left-floor (rear wall ∩ foot wall ∩ floor)
  X: foot → head
  Y: back → front (into room)
  Z: floor → ceiling
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Dim:
    FOOT: float = 98.0
    INTERNAL_W: float = 2246.0
    D: float = 1242.0
    EXT: float = 237.0
    H: float = 2380.0
    REAR: float = 210.0
    HEAD_W: float = 326.0
    SHELF: float = 1200.0
    PLINTH: float = 170.0
    FRAME_H: float = 360.0
    MATT_W: float = 1900.0
    MATT_D: float = 900.0
    MATT_H: float = 220.0
    TOL: float = 10.0
    ACCESS_W: float = 1300.0
    ACCESS_BOT: float = 640.0
    ACCESS_TOP: float = 2080.0
    STUD_W: float = 70.0
    STUD_FL: float = 45.0
    TRACK_WEB: float = 72.0
    TRACK_LEG: float = 25.0
    PB: float = 12.5
    LEDGER: float = 70.0
    # Metal gauge (mm) — real UK stud ~0.5–0.7; slightly thickened for robust STEP
    GAUGE: float = 0.7
    DECK_T: float = 18.0
    LEDGER_H: float = 45.0

    @property
    def W(self) -> float:
        return self.INTERNAL_W + self.FOOT  # 2344

    @property
    def PLATFORM(self) -> float:
        return self.PLINTH + self.FRAME_H  # 530

    @property
    def FRAME_D(self) -> float:
        return self.MATT_D + 2 * self.TOL  # 920

    @property
    def ACCESS_X(self) -> float:
        return 300.0 + self.FOOT  # 398

    @property
    def FRONT_Y(self) -> float:
        return self.D - self.FOOT  # 1144

    @property
    def BED_BAY(self) -> float:
        return self.FRONT_Y - self.REAR

    @property
    def FRAME_TUCK(self) -> float:
        return (self.BED_BAY - self.FRAME_D) / 2.0

    @property
    def shelf_w(self) -> float:
        return self.INTERNAL_W - self.HEAD_W  # 1920


DIM = Dim()
