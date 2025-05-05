import time
from common import Phase, Event #Phase: larger state machine, Event: feedback from sensor or esp
from link import SerialLink #UART Wrapper
from phases import  acquire_bin, setup#, transport_bin, drop_bin, return_home


#Dictionary where the key is a Phase enum value and the value is the class that runs that phase.
PHASE_IMPLS = {
    Phase.SETUP:        setup.Setup,
    Phase.ACQUIRE_BIN:  acquire_bin.AcquireBin,
    #Phase.TRANSPORT_BIN: transport_bin.TransportBin,
    #Phase.DROP_BIN:     drop_bin.DropBin,
    #Phase.RETURN_HOME:  return_home.ReturnHome,
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
        t0 = time.monotonic()
        # 1. Push queued UART writes & grab any replies
        replies = self._link.poll()
        #print("Step 1 time:")
        #print(time.monotonic() - t0)
        #t0 = time.monotonic()

        # 2. Convert robot replies or vision info into Events
        event = self._interpret(replies)
        #print("Step 2 time:")
        #print(time.monotonic() - t0)
        #t0 = time.monotonic()

        # 3. Let the current phase advance
        if event:
            print("EVENT NAME: " + event.name)
        nxt = self._impl.tick(event)
        if nxt == Phase.FINISH:
              self._phase = nxt
              return  # Do not try to create another _impl
        if nxt:
            self._phase = nxt
            self._impl = PHASE_IMPLS[nxt](self._link)
            self._impl.enter()

        #print("Step 3 time:")
        #print(time.monotonic() - t0)

    # ------------- helpers ------------
    def _interpret(self, replies: list[str]) -> Event | None:
        for line in replies:
            if line == "ACK HELLO":
                return Event("ESP_READY")
            if line == "ALIGNED":
                return Event("ALIGNED")
            if line == "ROTATE_DONE":
                return Event("ROTATE_DONE")
            if line == "DRIVING":
                return Event("DRIVING")
            if line == "STOP":
                return Event("STOP")
            #TODO: add more
        return None
        
    @property
    def phase(self) -> Phase:
        return self._phase
    
