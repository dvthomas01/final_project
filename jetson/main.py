#!/usr/bin/env python3
"""
Entry-point wrapper that:
1. Creates the FSM controller.
2. Ticks it at 20 Hz until the mission reaches FINISH (or you ^C).
3. Listens for keyboard commands like:
     manual             → drop into raw serial mode
     resume / auto      → resume autonomous phase
     phase BIN_BLUE     → switch FSM phase
     state B10          → jump to specific FSM state
     send ROTATE,90     → send raw line to ESP
     quit               → end loop
"""
import time
import sys
import threading
import queue
import logging
from controller import Controller, Phase

# ─────────────────────────────────────────────────────────────
# Logging setup
# ─────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("MAIN")

# ─────────────────────────────────────────────────────────────
# Start controller
# ─────────────────────────────────────────────────────────────
ctrl = Controller()

# ─────────────────────────────────────────────────────────────
# Background thread to read user input
# ─────────────────────────────────────────────────────────────
cmd_q: "queue.Queue[str]" = queue.Queue()

def _stdin_reader():
    for line in sys.stdin:
        cmd_q.put(line.strip())

threading.Thread(target=_stdin_reader, daemon=True).start()

# ─────────────────────────────────────────────────────────────
# Main FSM loop
# ─────────────────────────────────────────────────────────────
log.info(" FSM started. Type commands any time (manual, phase NAME, state NAME, send TEXT, resume, quit).")

try:
    while ctrl.phase is not Phase.FINISH:
        # 1. Tick FSM if in autonomous mode
        ctrl.tick()

        

        # 2. Handle queued operator commands
        try:
            cmd = cmd_q.get_nowait()
        except queue.Empty:
            pass
        else:
            tokens = cmd.split()
            if not tokens:
                continue
            head, *rest = tokens

            match head.lower():
                case "quit":
                    log.warning("Manual quit")
                    break
                case "manual":
                    ctrl.enter_manual()
                case "resume" | "auto":
                    ctrl.resume_auto()
                case "phase" if rest:
                    ctrl.set_phase(rest[0].upper())
                case "state" if rest:
                    ctrl.jump_to_state(rest[0])
                case "send" if rest:
                    raw = " ".join(rest)
                    ctrl.raw_tx(raw)
                case _:
                    log.warning(f"Unrecognised command: {cmd}")

        time.sleep(0.05)  # 20 Hz loop

except KeyboardInterrupt:
    log.warning("User abort with Ctrl-C")

finally:
    ctrl.shutdown()
    log.info("Clean shutdown.")
