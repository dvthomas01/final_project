# phases/setup.py
from common import Phase, Command, Event
from link import SerialLink


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

    # ---------------------------------------------------------------
    # Controller calls enter() exactly once when this phase becomes
    # active.  Good place to reset internal variables.
    # ---------------------------------------------------------------
    def enter(self) -> None:
        self._sent = False

    # ---------------------------------------------------------------
    # Controller calls tick(event) every 50 ms.
    # If the rotate is complete, return Phase.FINISH (causes shutdown).
    # Otherwise return None so the controller keeps us active.
    # ---------------------------------------------------------------
    def tick(self, event: Event | None) -> Phase | None:
        if not self._sent:
            self._link.enqueue(Command.ROTATE_CW.value)   # "ROTATE,1"
            self._sent = True
            return None

        # Wait for ESP‑32 to acknowledge completion
        if event and event.name == "ROTATE_DONE":
            return Phase.FINISH

        return None
