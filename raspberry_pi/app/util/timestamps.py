# pylint: disable=R0903,W0212
import ntcore
from app.network.network_protocol import Network
from app.util.timer import Timer


class Timestamps:
    """Translates between timebases."""

    def __init__(self, network: Network) -> None:
        """Network is required in order to access the drift measurement."""
        self._network = network

    def boot_time_to_server_time(self, timestamp_boottime_us: int) -> int:
        """timestamp_boottime_us: microsecond timestamp measured using CLOCK_BOOTTIME
        returns: microsecond timestamp measured in drift-corrected server time."""

        # It's important that these two "now" estimates happen nearly simultaneously.

        # "Now" relative to boot-time, in microseconds.
        now_boottime_us: int = Timer.time_ns() // 1000

        # "Now" relative to the client-time (wpi::now), in microseconds.
        now_clienttime_us = ntcore._now()

        # Microseconds between "now" and the timestamp.
        delta_us: int = now_boottime_us - timestamp_boottime_us

        # Timestamp relative to client-time, in microseconds.
        timestamp_clienttime_us: int = now_clienttime_us - delta_us

        # Timestamp relative to server time, in microseconds.
        servertime_us: int = self._network.server_time(timestamp_clienttime_us)

        return servertime_us

    @staticmethod
    def delta_us(timestamp_boottime_us: int) -> int:
        """Time since the supplied time.  Used to measure latency.

        timestamp_boottime_us: microsecond timestamp measured using CLOCK_BOOTTIME
        return: duration between the timestamp and the current instant."""

        # "Now" relative to boot-time, in microseconds.
        now_boottime_us: int = Timer.time_ns() // 1000

        # Microseconds between "now" and the timestamp.
        delta_us: int = now_boottime_us - timestamp_boottime_us

        return delta_us
