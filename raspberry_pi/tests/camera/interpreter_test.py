import unittest

from app.camera.camera_protocol import Camera
from app.camera.fake_camera import FakeCamera

from app.interpreter.interpreter_factory import InterpreterFactory
from app.config.identity import Identity
from app.dashboard.display_protocol import Display
from app.dashboard.fake_display import FakeDisplay
from app.interpreter.interpreter_protocol import Interpreter
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Network
from app.util.timestamps import Timestamps


class IntrinsicTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.DEV
        cam: Camera = FakeCamera("images/tag_and_board.jpg", (5504, 3096), -0.1)
        display1: Display = FakeDisplay()
        display2: Display = FakeDisplay()
        network: Network = FakeNetwork()
        timestamps = Timestamps(network)
        interpreter: Interpreter = InterpreterFactory.get(
            identity, cam, display1, display2, network, timestamps
        )
        self.assertIsNotNone(interpreter)
