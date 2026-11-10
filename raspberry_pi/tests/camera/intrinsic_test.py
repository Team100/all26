import unittest

from app.camera.intrinsic import Intrinsic
from app.config.identity import Identity


class IntrinsicTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.DEV
        Intrinsic(identity)

    def test_all_identities(self) -> None:
        """Make sure all identities have intrinsics."""
        for identity in list(Identity):
            intrinsic: Intrinsic = Intrinsic(identity)
            self.assertEqual(9, intrinsic.get().size)
            if identity == Identity.UNKNOWN:
                continue
            self.assertTrue(intrinsic.valid())
