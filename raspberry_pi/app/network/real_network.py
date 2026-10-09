# pylint: disable=C0114,C0115,C0116,E0401,R0902,R0903,W0212

from queue import Queue
from threading import Event, Thread
from typing import override

import ntcore
from app.config.identity import Identity
from app.network.calibrate import Calibrate
from app.network.drift import Drift
from app.network.network_protocol import (
    DoubleSender,
    BlipSender,
    BlipWithCornersSender,
    TargetSender,
    Network,
)
from app.network.structs import Blip, BlipWithCorners, Target
from app.network.sync_loop import SyncLoop
from app.network.undistort_view import UndistortView


class RealDoubleSender(DoubleSender):
    def __init__(self, pub: ntcore.DoublePublisher) -> None:
        self.pub = pub

    @override
    def send(self, val: float) -> None:
        self.pub.set(val)


class RealBlipSender(BlipSender):
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    @override
    def send(self, val: list[Blip]) -> None:
        self.pub.set(val)


class RealBlipWithCornersSender(BlipWithCornersSender):
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    @override
    def send(self, val: list[BlipWithCorners]) -> None:
        self.pub.set(val)


class RealTargetSender(TargetSender):
    def __init__(self, pub: ntcore.StructArrayPublisher) -> None:
        self.pub = pub

    @override
    def send(self, val: list[Target]) -> None:
        self.pub.set(val)


class RealNetwork(Network):
    """Wraps Network Tables, and also handles clock sync
    with a separate thread (to minimize jitter and aliasing
    in the sync measurement).
    """

    def __init__(self, identity: Identity, done: Event) -> None:
        print("*** Network: RealNetwork")
        self._identity: Identity = identity
        self._inst: ntcore.NetworkTableInstance = (
            ntcore.NetworkTableInstance.getDefault()
        )
        self._inst.startClient4("raspberry_pi")

        # roboRio address. windows machines can impersonate this for simulation.
        # also localhost for testing
        match identity:
            case Identity.UNKNOWN | Identity.SIM0:
                # For testing.
                self._inst.setServer("localhost")
            case _:
                # The static RoboRIO IP address.
                self._inst.setServer("10.1.0.2")

        self._queue: Queue[int] = Queue()

        # Add the calibration switch
        self._calibrate = Calibrate(self._inst, identity)

        # Add the undistort switch
        self._undistort_view = UndistortView(self._inst, identity)

        # Fill the queue at 50 hz
        syncloop = SyncLoop(self._inst, self._queue, identity, done)
        self._drift = Drift(self._inst, self._queue, identity)
        Thread(target=syncloop.run).start()

    @override
    def calibrate(self) -> bool:
        return self._calibrate.get()

    @override
    def undistort_view(self) -> bool:
        return self._undistort_view.get()

    @override
    def flush(self) -> None:
        self._inst.flush()

    @override
    def get_double_sender(self, leaf: str) -> RealDoubleSender:
        """For logging, using a different root so the listener doesn't hear it."""
        name: str = "pi/" + self._identity.value + "/" + leaf
        return RealDoubleSender(self._inst.getDoubleTopic(name).publish())

    @override
    def get_blip_sender(self) -> RealBlipSender:
        """Send blips to the Rio."""
        name: str = "vision/" + self._identity.value + "/blips"
        return RealBlipSender(self._inst.getStructArrayTopic(name, Blip).publish())

    @override
    def get_blip_with_corners_sender(self) -> RealBlipWithCornersSender:
        """Send blips to the Rio."""
        name: str = "vision/" + self._identity.value + "/blips_with_corners"
        return RealBlipWithCornersSender(
            self._inst.getStructArrayTopic(name, BlipWithCorners).publish()
        )

    @override
    def get_target_sender(self) -> RealTargetSender:
        """Send targets to the rio."""
        name: str = "objectVision/" + self._identity.value + "/targets"
        return RealTargetSender(self._inst.getStructArrayTopic(name, Target).publish())

    @override
    def server_time(self, localtime: int) -> int:
        return self._drift.server_time(localtime)
