
import unittest

from app.camera.intrinsic import Intrinsic
from app.config.identity import Identity


class IntrinsicTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.DEV
        Intrinsic(identity)

    # TODO: fix this test
    @unittest.expectedFailure
    def test_all_identities(self) -> None:
        for identity in list(Identity):
            intrinsic: Intrinsic = Intrinsic(identity)
            self.assertTrue(intrinsic.valid())
