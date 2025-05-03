# phases/setup.py
from common import Phase, Command, Event
from link import SerialLink
import time


class Setup:
    """
    First phase in the FSM.
    Behaviour: send one ROTATE_CW command, wait for the ESP to reply
    'ROTATE_DONE', then hand control back to the Controller so it can
    switch to Phase.FINISH (or whatever next phase you prefer).
    """

    def __init__(self, link: SerialLink):
        self._link = link
        self._sent = False          # True after we’ve issued the rotate
        self._step = 0
        self._driveduration = 2
        self._t0 = None

    # ---------------------------------------------------------------
    # Controller calls enter() exactly once when this phase becomes
    # active.  Good place to reset internal variables.
    # ---------------------------------------------------------------
    def enter(self) -> None:
        self._sent = False
        self._step = 0

    # ---------------------------------------------------------------
    # Controller calls tick(event) every 50 ms.
    # If the rotate is complete, return Phase.FINISH (causes shutdown).
    # Otherwise return None so the controller keeps us active.
    # ---------------------------------------------------------------
    def tick(self, event: Event | None) -> Phase | None:

        if self._step == 0 and not self._sent:
            self._link.enqueue(Command.ROTATE_CW.value)   # "ROTATE,1"
            self._sent = True
            self._step = 1 
            return None

        if self._step == 1 and event and event.name == "ROTATE_DONE":
            self._link.enqueue(Command.APPROACH_PICKUP.value) 
            self._sent = True
            self._t0 = time.monotonic()
            self._step = 2
            return None
        
        if self._step == 2 and event:
            if time.monotonic() - self._t0 >= self._duration:
                self._link.enqueue(Command.STOP.value) 
            self._sent = True
            return Phase.FINISH     
                   
        return None
