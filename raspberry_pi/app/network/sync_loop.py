# pylint: disable=C0114,C0115,C0116,E0401,E1101,R0902,R0903,R0913,R0917,W2301
import time
from datetime import datetime
from queue import Full, Queue
from threading import Event
from typing import override

import ntcore

from app.config.identity import Identity
from app.framework.looper import Looper
from app.network.structs import SyncReply, SyncRequest


class SyncLoop(Looper):
    """Fills the queue with timing offset estimates.

    Run this in a separate thread to minimize jitter and aliasing.

    See lib/network/SYNC.md.
    """

    def __init__(
        self,
        inst: ntcore.NetworkTableInstance,
        queue: Queue[int],
        identity: Identity,
        done: Event,
    ) -> None:
        super().__init__(done)
        self._inst = inst
        self._queue = queue
        self._offset: int = 0
        self._request_pub = inst.getStructTopic(
            "sync/" + identity.value + "/request", SyncRequest
        ).publish()
        self._reply_sub = inst.getStructTopic(
            "sync/" + identity.value + "/reply", SyncReply
        ).subscribe(SyncReply(0, 0, 0))

    @override
    def execute(self) -> None:
        self._request_pub.set(SyncRequest(ntcore._now()))  # pylint: disable=W0212

        replies: list[ntcore.TimestampedStruct] = self._reply_sub.readQueue()
        if replies:
            syncreply: SyncReply = replies[-1].value
            now: int = ntcore._now()  # pylint: disable=W0212
            measurement = self.offset(syncreply.org, syncreply.rec, syncreply.xmt, now)
            self._offset = self.fuse(self._offset, measurement)
            try:
                self._queue.put_nowait(self._offset)
            except Full:
                print(
                    f"[{datetime.now()}] "
                    + "*** queue is full, which should never happen ***"
                )

        self._inst.flush()
        # avoid spinning too fast
        time.sleep(0.02)

    @override
    def end(self) -> None:
        pass

    @staticmethod
    def offset(org: int, rec: int, xmt: int, dst: int) -> int:
        return (rec + xmt - dst - org) // 2

    @staticmethod
    def fuse(estimate: int, measurement: int) -> int:
        diff = estimate - measurement
        if abs(diff) > 30000:
            # more than 30 ms off, go right to the measurement
            # note the clock granularity leads to about 20 ms of jitter
            return measurement

        # very gentle averaging, seems to yield
        # about 1 ms of noise
        return (estimate * 199 + measurement) // 200
