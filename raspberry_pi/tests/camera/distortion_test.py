from datetime import datetime
import unittest

from app.camera.distortion import Distortion
from app.config.identity import Identity


class IntrinsicTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.DEV
        Distortion(identity)

    # TODO: fix this test
    @unittest.expectedFailure
    def test_all_identities(self) -> None:
        for identity in list(Identity):
            distortion: Distortion = Distortion(identity)
            self.assertTrue(distortion.valid())
