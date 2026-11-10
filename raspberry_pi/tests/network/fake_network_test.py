import unittest
from wpimath.geometry import Transform3d
from app.network.fake_network import FakeNetwork
from app.network.structs import BlipWithCorners


class FakeNetworkTest(unittest.TestCase):
    def test_double(self) -> None:
        network = FakeNetwork()
        sender = network.get_double_sender("foo")
        sender.send(1)
        self.assertEqual(1, network.doubles[0])

    def test_blip_with_corners(self) -> None:
        network = FakeNetwork()
        sender = network.get_blip_with_corners_sender()
        sender.send(
            [BlipWithCorners.make(0, 0, [0, 0, 1, 1, 2, 2, 3, 3], Transform3d())]
        )
        b: BlipWithCorners = network.blips_with_corners[0]
        self.assertEqual(1, b.x1)
