import serial, time
from collections import deque
from common import BAUD, PORT, MSG_TERMINATOR

class SerialLink:
    def __init__(self, port: str = PORT, baud: int = BAUD):
        self._ser = serial.Serial(port, baud, timeout=0)
        self._tx_q: deque[str] = deque()

    # ------------ public -------------
    def enqueue(self, cmd: str) -> None:
        self._tx_q.append(cmd + MSG_TERMINATOR)

    def poll(self) -> list[str]:
        """Send any queued messages, return list of complete lines received."""
        while self._tx_q:
            self._ser.write(self._tx_q.popleft().encode())

        lines = []
        while (l := self._ser.readline()):
            lines.append(l.decode().strip())
        return lines
