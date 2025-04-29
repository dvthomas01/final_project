#!/usr/bin/env python3
"""
Entry-point wrapper that:
1. Creates the FSM controller.
2. Ticks it at 20 Hz until the mission reaches FINISH (or you ^C).
3. Ensures clean shutdown on exit.
"""
import time
import logging
from controller import Controller, Phase

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("MAIN")

ctrl = Controller()

try:
    log.info("Starting mobile-robot FSM")
    while ctrl.phase is not Phase.FINISH:
        ctrl.tick()
        time.sleep(0.05)           # 20 Hz loop
except KeyboardInterrupt:
    log.warning("User abort with Ctrl-C")
finally:
    ctrl.shutdown()
    log.info("Clean shutdown – bye")
