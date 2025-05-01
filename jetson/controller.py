"""
Hierarchical finite-state machine for the Jetson Nano.

* High-level actions are Python stubs that just transmit one-line serial
  commands – fill them in with real vision / control logic later.
* The ESP32-S3 runs firmware that parses those lines and drives motors / sensors.
"""
from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

import serial     


# ──────────────────────────────────────────────────────────────────────────────
# Serial link helper
# ──────────────────────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyUSB0"
BAUD        = 115200
SER_TIMEOUT = 0.1     # seconds

class SerialLink:
    def __init__(self, port: str, baud: int, timeout: float):
        self.ser = serial.Serial(port, baud, timeout=timeout)

    def tx(self, cmd: str, arg: Optional[int | str] = None) -> None:
        line = f"{cmd}{',' + str(arg) if arg is not None else ''}\n"
        self.ser.write(line.encode())
        print("TX:", line.strip())

    #  acknowledgements
    # def rx(self) -> str | None:
    #     data = self.ser.readline().decode().strip()
    #     return data or None

    def close(self):
        self.ser.close()

link = SerialLink(SERIAL_PORT, BAUD, SER_TIMEOUT)


# ──────────────────────────────────────────────────────────────────────────────
# Action primitives  (RETURN True when finished)
# ──────────────────────────────────────────────────────────────────────────────
def set_up() -> bool:                       # camera / joystick self-test
    return True

def align(tag: int) -> bool:                link.tx("ALIGN", tag);                 return True
def rotate(deg: int) -> bool:               link.tx("ROTATE", deg);                return True
def fine_align(tag: int) -> bool:           link.tx("FINE_ALIGN", tag);            return True
def approach_pickup_pose(tag: int) -> bool: link.tx("APPROACH_PICKUP", tag);       return True
def grab_bin() -> bool:                     link.tx("GRAB_BIN");                   return True
def fixed_backup(tag: int) -> bool:         link.tx("BACKUP_TAG", tag);            return True
def deposit_bin() -> bool:                  link.tx("DEPOSIT_BIN");                return True
def back_up() -> bool:                      link.tx("BACK_UP");                    return True
def s_maneuver_align(tag: int) -> bool:     link.tx("S_ALIGN", tag);               return True
def drive_up_ramp() -> bool:                link.tx("DRIVE_UP_RAMP");              return True
def drive_down_ramp() -> bool:              link.tx("DRIVE_DOWN_RAMP");            return True
def color_detect_store() -> bool:           link.tx("COLOR_DETECT");               return True
def drive_to_pickup(tag: int) -> bool:      link.tx("DRIVE_TO_PICKUP", tag);       return True


# ──────────────────────────────────────────────────────────────────────────────
# FSM scaffolding
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class State:
    name: str
    fn:   Callable[[], bool]
    nxt:  int                 # next state index (-1 → end of sequence)

FSM = List[State]

def build_setup_sequence() -> FSM:
    return [
        State("S1:set_up",             set_up,                     1),
        State("S2:align9",             lambda: align(9),           2),
        State("S3:rotate+90",          lambda: rotate(+90),        3),
        State("S4:fine_align9",        lambda: fine_align(9),      4),
        State("S5:approach_pickup9",   lambda: approach_pickup_pose(9), 5),
        State("S6:grab_bin",           grab_bin,                   6),
        State("S7:fixed_backup7",      lambda: fixed_backup(7),    7),
        State("S8:rotate-90",          lambda: rotate(-90),        8),
        State("S9:fine_align7",        lambda: fine_align(7),      9),
        State("S10:approach_pickup7",  lambda: approach_pickup_pose(7), 10),
        State("S11:deposit_bin",       deposit_bin,                11),
        State("S12:back_up",           back_up,                    12),
        State("S13:rotate-90",         lambda: rotate(-90),        13),
        State("S14:align5",            lambda: align(5),          -1),
    ]

def build_bin_sequence(tag_pick: int, tag_drop: int, tag_next: int) -> FSM:
    return [
        State("B1:align_pick"        , lambda: align(tag_pick),                 1),
        State("B2:rotate_pick"       , lambda: rotate(+90),                     2),
        State("B3:fine_align_pick"   , lambda: fine_align(tag_pick),            3),
        State("B4:approach_pickup"   , lambda: approach_pickup_pose(tag_pick),  4),
        State("B5:grab_bin"          , grab_bin,                                5),
        State("B6:fixed_backup"      , lambda: fixed_backup(tag_pick),          6),
        State("B7:rotate_ramp_entry" , lambda: rotate(+90),                     7),
        State("B8:s_maneuver_align8" , lambda: s_maneuver_align(8),             8),
        State("B9:rotate_ramp"       , lambda: rotate(+90),                     9),
        State("B10:drive_up_ramp"    , drive_up_ramp,                          10),
        State("B11:rotate_ramp_exit" , lambda: rotate(+90),                    11),
        State("B12:drive_down_ramp"  , drive_down_ramp,                        12),
        State("B13:color_detect"     , color_detect_store,                     13),
        State("B14:rotate_drop_entry", lambda: rotate(-90),                    14),
        State("B15:align_drop"       , lambda: align(tag_drop),                15),
        State("B16:rotate_drop"      , lambda: rotate(+90),                    16),
        State("B17:fine_align_drop"  , lambda: fine_align(tag_drop),           17),
        State("B18:approach_drop"    , lambda: approach_pickup_pose(tag_drop), 18),
        State("B19:deposit_bin"      , deposit_bin,                            19),
        State("B20:back_up"          , back_up,                                20),
        State("B21:rotate_to_next"   , lambda: rotate(+90),                    21),
        State("B22:drive_to_pickup"  , lambda: drive_to_pickup(tag_next),      -1),
    ]


# ──────────────────────────────────────────────────────────────────────────────
# Controller class
# ──────────────────────────────────────────────────────────────────────────────
class Phase(enum.Enum):
    SETUP      = 0
    BIN_YELLOW = 1
    BIN_BLUE   = 2
    BIN_RED    = 3
    FINISH     = 4
    RECOVERY   = 5

class Controller:
    def __init__(self):
        self.phase: Phase = Phase.SETUP
        self.fsm:   FSM   = build_setup_sequence()
        self.idx:   int   = 0

    # ---------------------------------------
    def tick(self) -> None:
        if self.phase in (Phase.FINISH, Phase.RECOVERY):
            return

        if self.idx >= len(self.fsm):        # sequence done → next phase
            self._advance_phase()
            return

        state = self.fsm[self.idx]
        done  = state.fn()
        if done:
            self.idx = state.nxt if state.nxt >= 0 else len(self.fsm)

    # ---------------------------------------
    def _advance_phase(self) -> None:
        if self.phase is Phase.SETUP:
            self.phase = Phase.BIN_YELLOW
            self.fsm  = build_bin_sequence(tag_pick=6, tag_drop=6, tag_next=5)
        elif self.phase is Phase.BIN_YELLOW:
            self.phase = Phase.BIN_BLUE
            self.fsm  = build_bin_sequence(tag_pick=5, tag_drop=5, tag_next=4)
        elif self.phase is Phase.BIN_BLUE:
            self.phase = Phase.BIN_RED
            self.fsm  = build_bin_sequence(tag_pick=4, tag_drop=4, tag_next=0)
        elif self.phase is Phase.BIN_RED:
            self.phase = Phase.FINISH
            print("Mission complete ✔")
            return
        else:
            return
        self.idx = 0
        print(f"→ Phase {self.phase.name}")

    # ---------------------------------------
    def shutdown(self) -> None:
        link.close()
