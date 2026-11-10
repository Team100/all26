import unittest
import numpy as np
from numpy.typing import NDArray

from app.camera.camera_protocol import Camera
from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.display_factory import DisplayFactory
from app.dashboard.display_protocol import Display
from app.dashboard.display_util import DisplayUtil
from app.dashboard.fake_display import FakeDisplay


class DisplayTest(unittest.TestCase):
    def test_display(self) -> None:
        display: Display = FakeDisplay()
        img: NDArray[np.uint8] = np.zeros([100, 100], dtype=np.uint8)
        DisplayUtil.text(img, "hi", (0, 0), 0) # type: ignore
        display.put(img)

    def test_identity(self) -> None:
        identity: Identity = Identity.DEV
        cam: Camera = FakeCamera("images/tag_and_board.jpg", (5504, 3096), -0.1)
        DisplayFactory.get(identity, cam, "main", 1.0)
