import unittest

from app.camera.distortion import Distortion
from app.config.identity import Identity


class IntrinsicTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.DEV
        Distortion(identity)

    def test_all_identities(self) -> None:
        """Make sure all identities have distortion."""
        for identity in list(Identity):
            distortion: Distortion = Distortion(identity)
            self.assertEqual(4, distortion.get().size)
            if identity == Identity.UNKNOWN:
                continue
            self.assertTrue(distortion.valid())
