# from datetime import datetime
import unittest

from threading import Event

import ntcore

from wpimath.geometry import Transform3d
from app.config.identity import Identity
from app.network.real_network import RealNetwork
from app.network.structs import BlipWithCorners


class RealNetworkTest(unittest.TestCase):
    def test_send(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getDoubleTopic("pi/unknown/foo").subscribe(0.0)

        network = RealNetwork(Identity.UNKNOWN, Event())
        sender = network.get_double_sender("foo")
        sender.send(1.0)
        self.assertEqual(1.0, sub.get())

    def test_send_blips_with_corners(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructArrayTopic(
            "vision/unknown/blips_with_corners", BlipWithCorners
        ).subscribe([])

        network = RealNetwork(Identity.UNKNOWN, Event())
        sender = network.get_blip_with_corners_sender()
        sender.send(
            [BlipWithCorners.make(0, 0, [0, 0, 1, 1, 2, 2, 3, 3], Transform3d())]
        )
        blips = sub.get()
        self.assertEqual(1, len(blips))
        self.assertEqual(1.0, blips[0].x1)
