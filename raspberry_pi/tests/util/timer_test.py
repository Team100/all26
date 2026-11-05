from datetime import datetime
# pylint: disable=W0212
import time
import unittest

import ntcore

from app.util.timer import Timer


class TimerTest(unittest.TestCase):
    def test_timer(self) -> None:
        t0: int = Timer.time_ns()
        time.sleep(0.01)
        t1: int = Timer.time_ns()
        self.assertGreater(t1, t0)

    def test_compare(self) -> None:
        # On my desktop, this yields the machine uptime (about 15 days at the moment).
        t0: int = Timer.time_ns() // 1000
        # On my desktop, this yields Unix Epoch time (since 1970).
        t1: int = ntcore._now()
        print(f"[{datetime.now()}] " + "\nt0 ", t0)
        print(f"[{datetime.now()}] " + "\nt1 ", t1)
        # These two clocks are nothing like each other.
        self.assertNotEqual(t0, t1)
