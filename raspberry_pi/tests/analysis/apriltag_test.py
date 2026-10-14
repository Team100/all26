# pylint: disable=E1101,W0212

import unittest
from wpimath.geometry import Transform3d, Translation3d, Rotation3d

import numpy as np
import cv2

from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.analysis.apriltags import AprilTags
from app.network.fake_network import FakeNetwork


class AprilTagTest(unittest.TestCase):
    """Mostly tests that show the analyzer puts
    the right results on the network."""

    def test_big_sharp(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/big_sharp.png")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type: ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(0.000, t.x, delta=0.001)
        self.assertAlmostEqual(-0.006, t.y, delta=0.001)
        self.assertAlmostEqual(0.264, t.z, delta=0.001)
        r = pose.rotation()
        self.assertAlmostEqual(0, r.x, delta=0.001)
        self.assertAlmostEqual(0, r.y, delta=0.002)
        self.assertAlmostEqual(0, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(410, b.x0, delta=1)
        self.assertAlmostEqual(814, b.y0, delta=1)

    def test_big_sharp90(self) -> None:
        """This image is rotated."""
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/big_sharp90.png")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(0.000, t.x, delta=0.001)
        self.assertAlmostEqual(-0.006, t.y, delta=0.001)
        self.assertAlmostEqual(0.264, t.z, delta=0.001)
        r = pose.rotation()
        self.assertAlmostEqual(0, r.x, delta=0.002)
        self.assertAlmostEqual(0, r.y, delta=0.002)
        # tag is rotated 90 degrees clockwise.
        # z axis is down the barrel so the rotation appears here.
        self.assertAlmostEqual(1.571, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(409, b.x0, delta=1)
        self.assertAlmostEqual(229, b.y0, delta=1)

    def test_big_sharp_y(self) -> None:
        """This image uses the "perspective" transform in gimp."""
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/big_sharpY.png")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(0.011, t.x, delta=0.001)
        self.assertAlmostEqual(-0.006, t.y, delta=0.001)
        self.assertAlmostEqual(0.306, t.z, delta=0.001)
        r = pose.rotation()
        self.assertAlmostEqual(0.021, r.x, delta=0.001)
        # tag is rotated something like 30 degrees in negative Y
        # y axis is dowm so the rotation appears here.
        self.assertAlmostEqual(-0.658, r.y, delta=0.001)
        self.assertAlmostEqual(-0.012, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(512, b.x0, delta=1)
        self.assertAlmostEqual(815, b.y0, delta=1)

    def test_scale1(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/scale1.png")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(0.000, t.x, delta=0.001)
        self.assertAlmostEqual(-0.007, t.y, delta=0.001)
        self.assertAlmostEqual(0.528, t.z, delta=0.001)
        r = pose.rotation()
        self.assertAlmostEqual(0, r.x, delta=0.004)
        self.assertAlmostEqual(0, r.y, delta=0.004)
        self.assertAlmostEqual(0, r.z, delta=0.001)
        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(557, b.x0, delta=1)
        self.assertAlmostEqual(679, b.y0, delta=1)

    def test_scale2(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/scale2.png")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(-0.001, t.x, delta=0.001)
        self.assertAlmostEqual(-0.008, t.y, delta=0.001)
        self.assertAlmostEqual(1.055, t.z, delta=0.001)
        r = pose.rotation()
        self.assertAlmostEqual(0, r.x, delta=0.003)
        self.assertAlmostEqual(0, r.y, delta=0.007)
        self.assertAlmostEqual(0, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(630, b.x0, delta=1)
        self.assertAlmostEqual(610, b.y0, delta=1)

    def test_scale3(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/scale3.pnm")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(-0.003, t.x, delta=0.001)
        self.assertAlmostEqual(-0.007, t.y, delta=0.001)
        self.assertAlmostEqual(2.119, t.z, delta=0.01)
        r = pose.rotation()
        self.assertAlmostEqual(0, r.x, delta=0.04)
        self.assertAlmostEqual(0, r.y, delta=0.1)
        self.assertAlmostEqual(0, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(666, b.x0, delta=1)
        self.assertAlmostEqual(578, b.y0, delta=1)

    def test_scale4(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/scale4.pnm")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(-0.005, t.x, 2)
        self.assertAlmostEqual(-0.007, t.y, 2)
        self.assertAlmostEqual(4.242, t.z, 1)
        r = pose.rotation()
        self.assertAlmostEqual(0, r.x, delta=0.02)
        self.assertAlmostEqual(0, r.y, delta=0.001)
        self.assertAlmostEqual(0, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(685, b.x0, delta=1)
        self.assertAlmostEqual(561, b.y0, delta=1)

    def test_scale5(self) -> None:
        """It's kind of amazing that this one works.
        This does not work when quadDecimate is set to 4."""
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/scale5.pnm")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(0, t.x, 1)
        self.assertAlmostEqual(0, t.y, 1)
        self.assertAlmostEqual(8.497, t.z, 3)
        r = pose.rotation()
        self.assertAlmostEqual(0, r.x, delta=0.2)
        self.assertAlmostEqual(0, r.y, delta=0.004)
        self.assertAlmostEqual(0, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(694, b.x0, delta=1)
        self.assertAlmostEqual(552, b.y0, delta=1)

    def test_scale6(self) -> None:
        """This is too small."""
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/scale6.pnm")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore
        self.assertEqual(0, len(network.blips))
        self.assertEqual(0, len(network.blips_with_corners))

    def test_one_tag_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        camera = FakeCamera("images/tag_and_board.jpg", (1100, 620))
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        pose = network.blips[0].pose
        t = pose.translation()
        self.assertAlmostEqual(-0.186, t.x, 3)
        self.assertAlmostEqual(0.027, t.y, 3)
        self.assertAlmostEqual(0.642, t.z, 3)
        r = pose.rotation()
        print(r.getQuaternion())
        self.assertAlmostEqual(0.786, r.x, delta=0.001)
        self.assertAlmostEqual(-0.607, r.y, delta=0.001)
        self.assertAlmostEqual(-0.492, r.z, delta=0.001)

        self.assertEqual(1, len(network.blips_with_corners))
        b = network.blips_with_corners[0]
        self.assertAlmostEqual(191, b.x0, delta=1)
        self.assertAlmostEqual(496, b.y0, delta=1)

    def test_zero_tags_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        # nothing in this image
        camera = FakeCamera("images/white_square.jpg")
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(0, len(network.blips))
        self.assertEqual(0, len(network.blips_with_corners))

    def verify_pose(self, pose: Transform3d, delta: float) -> None:
        print("*** pose: ", pose)
        t: Translation3d = pose.translation()
        self.assertAlmostEqual(-0.186, t.x, delta=delta)
        self.assertAlmostEqual(0.027, t.y, delta=delta)
        self.assertAlmostEqual(0.642, t.z, delta=delta)
        r: Rotation3d = pose.rotation()
        self.assertAlmostEqual(0.786, r.x, delta=delta)
        self.assertAlmostEqual(-0.607, r.y, delta=delta)
        self.assertAlmostEqual(-0.492, r.z, delta=delta)

    def test_distortion(self) -> None:
        """How much distortion can there be in the image?"""
        identity = Identity.UNKNOWN
        network = FakeNetwork()

        # No distortion.
        camera = FakeCamera("images/tag_and_board.jpg", (1100, 620), 0)
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

        self.assertEqual(1, len(network.blips))
        self.verify_pose(network.blips[0].pose, 0.001)

        # A moderate amount of distortion
        network = FakeNetwork()
        camera = FakeCamera("images/tag_and_board.jpg", (1100, 620), -0.1)
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore
        self.assertEqual(1, len(network.blips))
        # A tiny bit more tolerance
        self.verify_pose(network.blips[0].pose, 0.002)

        # A realistic amount of distortion
        network = FakeNetwork()
        camera = FakeCamera("images/tag_and_board.jpg", (1100, 620), -0.3)
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore
        self.assertEqual(1, len(network.blips))
        # A bit more tolerance
        self.verify_pose(network.blips[0].pose, 0.003)

        # This is too much distortion, so detection fails.
        # Note: this is a truly enormous amount of distortion.
        network = FakeNetwork()
        camera = FakeCamera("images/tag_and_board.jpg", (1100, 620), -2)
        apriltags = AprilTags(identity, camera, network)
        apriltags.analyze_mono(camera._mono, None, 0)  # type:ignore

    def test_redistort(self) -> None:
        """This is just to see what it's doing."""
        height = 3
        width = 5
        grid_y, grid_x = np.indices((height, width), dtype=np.float32)
        print("grid_y", grid_y)
        print("grid_x", grid_x)
        flat_grid = np.stack([grid_x.ravel(), grid_y.ravel()], axis=-1).reshape(
            -1, 1, 2
        )
        print("flat_grid", flat_grid)
        intrinsic = np.array(
            [
                [6, 0, 3],
                [0, 6, 3],
                [0, 0, 1],
            ]
        )
        dist = np.array([-0.1, 0, 0, 0])
        distort_map = cv2.undistortPoints(flat_grid, intrinsic, dist, P=intrinsic)
        print("distort_map 1", distort_map)
        distort_map = distort_map.reshape(height, width, 2)
        print("distort_map 2", distort_map)
        map_x = distort_map[:, :, 0]
        print("map_x", map_x)
        map_y = distort_map[:, :, 1]
        print("map_y", map_y)
