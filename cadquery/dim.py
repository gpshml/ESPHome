"""Locked cabin-bed dimensions — mirror of cabin_bed_model.js `DIM`.

Orientation = locked front aspect / top_plan_FINAL.png (see LOCKED-IN-SPEC.md):
  Looking into the alcove — FOOT LEFT, HEAD RIGHT.
  Narrow access pier (~300mm) at the FOOT (left).
  Origin = back-left-floor (rear ∩ foot wall ∩ floor)
  X: foot → head
  Y: back → front (into room)
  Z: floor → ceiling

Do NOT invert head/foot based on a mirrored 3D viewer — fix the view, not the room.
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
    GAUGE: float = 0.7
    DECK_T: float = 18.0
    LEDGER_H: float = 45.0
    NOGGIN_H: float = 40.0
    NOGGIN_ZS: tuple[float, ...] = (600.0, 1200.0, 1800.0)
    # Clear pier beside foot wall (opening set out from the foot / left)
    FOOT_PIER: float = 300.0

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
        """Left edge of access opening. Narrow FOOT_PIER at the foot (left)."""
        return self.FOOT + self.FOOT_PIER  # 398

    @property
    def FRONT_Y(self) -> float:
        return self.D - self.FOOT  # 1144

    @property
    def FRONT_ROOM_Y(self) -> float:
        return self.D  # room face of front wall / foot extension front

    @property
    def BED_BAY(self) -> float:
        return self.FRONT_Y - self.REAR

    @property
    def FRAME_TUCK(self) -> float:
        return (self.BED_BAY - self.FRAME_D) / 2.0

    @property
    def shelf_w(self) -> float:
        return self.INTERNAL_W - self.HEAD_W  # 1920

    @property
    def ledger_z(self) -> float:
        """Ledger bottom so its top meets deck underside."""
        return self.SHELF - self.DECK_T - self.LEDGER_H

    @property
    def fascia_h(self) -> float:
        """Shelf fascia uprights stop under the deck."""
        return self.SHELF - self.DECK_T


DIM = Dim()
