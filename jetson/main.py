from controller import Controller
import time

ctrl = Controller()
print("Jetson FSM running, Ctrl‑C to quit")
try:
    while True:
        start = time.monotonic()
        ctrl.tick()
        time.sleep(max(0, Controller.PERIOD - (time.monotonic() - start)))
except KeyboardInterrupt:
    print("Bye")
