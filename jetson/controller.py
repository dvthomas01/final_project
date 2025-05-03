import time
from common import Phase, Event #Phase: larger state machine, Event: feedback from sensor or esp
from link import SerialLink #UART Wrapper
from phases import  acquire_bin, setup, transport_bin, drop_bin, return_home


#Dictionary where the key is a Phase enum value and the value is the class that runs that phase.
PHASE_IMPLS = {
    Phase.SETUP:        setup.Setup,
    Phase.ACQUIRE_BIN:  acquire_bin.AcquireBin,
    Phase.TRANSPORT_BIN: transport_bin.TransportBin,
    Phase.DROP_BIN:     drop_bin.DropBin,
    Phase.RETURN_HOME:  return_home.ReturnHome,
}

class Controller:
    PERIOD = 0.05       # 50 ms

    def __init__(self):
        self._link = SerialLink()
        self._phase = Phase.SETUP
        self._impl = PHASE_IMPLS[self._phase](self._link)
        self._impl.enter()

    # ------------- public -------------
    def tick(self):
        # 1. Push queued UART writes & grab any replies
        replies = self._link.poll()

        # 2. Convert robot replies or vision info into Events
        event = self._interpret(replies)

        # 3. Let the current phase advance
        nxt = self._impl.tick(event)
        if nxt:
            self._phase = nxt
            self._impl = PHASE_IMPLS[nxt](self._link)
            self._impl.enter()

    # ------------- helpers ------------
    def _interpret(self, replies: list[str]) -> Event | None:
        for line in replies:
            if line == "ACK HELLO":
                return Event("ESP_READY")
            if line == "ALIGNED":
                return Event("ALIGNED")
            # …add more
        return None
