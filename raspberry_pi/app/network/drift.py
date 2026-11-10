# pylint: disable=C0114,C0115,C0116,E0401,E1101,R0902,R0903,R0913,R0917,W2301
from queue import Empty, Queue

import ntcore

from app.config.identity import Identity


class Drift:
    """Consume updates from the queue, maintain the offset, and publish the drift."""

    def __init__(
        self, inst: ntcore.NetworkTableInstance, queue: Queue[int], identity: Identity
    ) -> None:
        self._inst: ntcore.NetworkTableInstance = inst
        self._queue: Queue[int] = queue
        self._first_estimate: int = 0
        self._offset: int = 0
        """Offset in microseconds."""
        self._offset_pub: ntcore.IntegerPublisher = self._inst.getIntegerTopic(
            "sync/" + identity.value + "/offset (us)"
        ).publish()
        self._drift_pub: ntcore.IntegerPublisher = inst.getIntegerTopic(
            "sync/" + identity.value + "/drift (us)"
        ).publish()

    def _get(self) -> int:
        """Get the current offset, in microseconds, updating if possible."""
        try:
            self._offset = self._queue.get_nowait()

            if self._first_estimate == 0:
                self._first_estimate = self._offset

            if abs(self._first_estimate - self._offset) > 1000000:
                # this is for when we restart the rio
                self._first_estimate = self._offset

            drift = self._offset - self._first_estimate
            self._offset_pub.set(self._offset)
            self._drift_pub.set(drift)

        except Empty:
            pass
            # nothing to do

        return self._offset

    def server_time(self, localtime: int) -> int:
        """Apply the drift to the supplied localtime to obtain
        the equivalent servertime, in microseconds."""
        return localtime + self._get()
