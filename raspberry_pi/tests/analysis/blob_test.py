# pylint: disable=E1101,W0212

import unittest
import numpy as np
from app.camera.fake_camera import FakeCamera
from app.analysis.blobs import Blobs
from app.network.structs import Target
from app.network.fake_network import FakeNetwork


class BlobTest(unittest.TestCase):
    """Mostly tests that show the analyzer puts
    the right results on the network."""

    def test_one_note_found(self) -> None:
        network = FakeNetwork()
        # this has a blob that matches the
        # HSV range in the note detector
        # the blob is in the lower right quadrant, so the result
        # should be pitch-down yaw-right.
        # ORANGE TARGET
        # camera = FakeCamera("images/blob.jpg")
        # GREEN PRACTICE TARGET
        # no distortion
        camera = FakeCamera("images/green_blob.jpg")

        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))

        blobs = Blobs(camera, network, object_lower, object_higher)
        blobs.analyze_color(camera._img, None, 0, 100)  # type: ignore

        rots: list[Target] = network.targets
        self.assertEqual(1, len(rots))
        rot = rots[0].sight
        # ~zero
        self.assertAlmostEqual(-0.006, rot.x, delta=0.001)
        # pitch down
        self.assertAlmostEqual(0.121, rot.y, delta=0.001)
        # yaw right
        self.assertAlmostEqual(-0.104, rot.z, delta=0.001)
        rot2d = rot.toRotation2d()
        # right yaw is about 10 deg
        self.assertAlmostEqual(-5.983, rot2d.degrees(), delta=0.001)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), delta=0.001)
        self.assertAlmostEqual(0.060, q.Y(), delta=0.001)
        self.assertAlmostEqual(-0.052, q.Z(), delta=0.001)
        self.assertAlmostEqual(0.997, q.W(), delta=0.001)

    def test_target_undistort(self) -> None:
        # "barrel" to keep the blob in the frame
        camera = FakeCamera("images/green_blob.jpg", None, -0.1)
        network = FakeNetwork()

        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))

        blobs = Blobs(camera, network, object_lower, object_higher)
        blobs.analyze_color(camera._img, None, 0, 100)   # type: ignore

        # the extracted rotation should be undistorted.
        rots: list[Target] = network.targets
        self.assertEqual(1, len(rots))
        rot = rots[0].sight
        # NOTE: 0.01 rad resolution is all that can be expected.
        # ~zero
        self.assertAlmostEqual(-0.006, rot.x, delta=0.001)
        # pitch down
        self.assertAlmostEqual(0.121, rot.y, delta=0.001)
        # yaw right
        self.assertAlmostEqual(-0.104, rot.z, delta=0.001)
        rot2d = rot.toRotation2d()
        # almost right
        self.assertAlmostEqual(-5.938, rot2d.degrees(), delta=0.001)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), delta=0.001)
        self.assertAlmostEqual(0.06, q.Y(), delta=0.001)
        self.assertAlmostEqual(-0.052, q.Z(), delta=0.001)
        self.assertAlmostEqual(0.997, q.W(), delta=0.001)

    def test_zero_notes_found(self) -> None:
        network = FakeNetwork()

        # nothing in this image
        camera = FakeCamera("images/white_square.jpg")

        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))

        blobs = Blobs(camera, network, object_lower, object_higher)
        blobs.analyze_color(camera._img, None, 0, 100)  # type: ignore

        ## always publish even if empty

        rots: list[Target] = network.targets

        self.assertEqual(0, len(rots))
