from datetime import datetime
# pylint: disable=R0902,R0903,W0212

import time
from queue import Queue, Full
from threading import Event
from typing import override

import ntcore
from app.config.identity import Identity
from app.framework.looper import Looper
from app.network.structs import SyncRequest, SyncReply


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
            self._offset: int = self.fuse(self._offset, measurement)
            try:
                self._queue.put_nowait(self._offset)
            except Full:
                print(f"[{datetime.now()}] " + "*** queue is full, which should never happen ***")

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
