from enum import Enum, auto
from dataclasses import dataclass

BAUD = 115200
PORT = "/dev/ttyTHS0"          # Jetson’s UART0 on the J41 header
MSG_TERMINATOR = "\n"          # keep it human‑readable

class Command(str, Enum):
    # keep names identical to ESP enum
    SETUP = "SETUP"
    ALIGN = "ALIGN"
    ROTATE_CW = "ROTATE,1"
    ROTATE_CCW = "ROTATE,-1"
    FINE_ALIGN = "F_ALIGN"
    APPROACH_PICKUP = "A_PICKUP"
    GRAB_BIN = "G_BIN"
    BACKUP_FIXED = "B_TAG"
    DEPOSIT_BIN = "D_BIN"
    BACKUP = "B_UP"
    S_ALIGN = "S_ALIGN"
    COLOR_DETECT = "COLOR_DETECT"
    DRIVE_TO_PICKUP = "DRIVE_TO_PICKUP"
    FINISH = "FINISH"
    # STOP = "STOP"
    # leave out the ramp commands for now

class Phase(Enum):
    SETUP = auto()
    ACQUIRE_BIN = auto()
    TRANSPORT_BIN = auto()
    DROP_BIN = auto()
    RETURN_HOME = auto()
    FINISH = auto()

@dataclass
#dataclass carrying sensor or ESP feedback. 
class Event:
    """Sensor or internal trigger that can cause a state transition."""
    name: str
    data: dict | None = None
