from common import Phase, Command, Event
from link import SerialLink

class AcquireBin:
    def __init__(self, link: SerialLink):
        self._link = link
        self._step = 0           # small internal sub‑steps

    def enter(self):
        self._step = 0

    def tick(self, event: Event | None) -> Phase | None:
        """
        Runs every 50 ms from Controller.tick().
        Returns a *new* Phase when this one is finished, else None.
        """
        if self._step == 0:
            self._link.enqueue(Command.ALIGN.value)
            self._step = 1

        elif self._step == 1 and event and event.name == "ALIGNED":
            self._link.enqueue(Command.FINE_ALIGN.value)
            self._step = 2

        elif self._step == 2 and event and event.name == "FINE_ALIGNED":
            self._link.enqueue(Command.APPROACH_PICKUP.value)
            self._step = 3

        elif self._step == 3 and event and event.name == "AT_PICKUP_POSE":
            self._link.enqueue(Command.GRAB_BIN.value)
            self._step = 4

        elif self._step == 4 and event and event.name == "BIN_SECURED":
            return Phase.TRANSPORT_BIN

        return None
