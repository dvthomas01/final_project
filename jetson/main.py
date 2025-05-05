from controller import Controller
from common import Phase
import time

ctrl = Controller()
print("Jetson FSM running – Ctrl‑C to quit")

try:
    while ctrl.phase is not Phase.FINISH:
        t0 = time.monotonic()
        ctrl.tick()
        #print(Controller.PERIOD - (time.monotonic() - t0))
        time.sleep(max(0, Controller.PERIOD - (time.monotonic() - t0)))
except KeyboardInterrupt:
    pass

print("Finished single rotation; shutting down.")
print("In multistate Branch")
