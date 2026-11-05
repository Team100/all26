# pylint: disable=E1101,R0903

from typing import Any, override
import cv2
import numpy as np

from cv2.typing import MatLike
from numpy.typing import NDArray

from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator

from wpimath.geometry._geometry import Transform3d

from app.camera.camera_protocol import Camera
from app.config.identity import Identity
from app.dashboard.display_util import DisplayUtil
from app.analysis.analysis_protocol import MonoAnalysis
from app.network.network_protocol import Network
from app.network.structs import Blip, BlipWithCorners

# Tag corners for computing homography.
SRC_POINTS = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]])


class AprilTags(MonoAnalysis):
    """A wrapper for the AprilTag detector."""

    def __init__(
        self,
        identity: Identity,
        cam: Camera,
        network: Network,
    ) -> None:
        """Finds Apriltags.

        :identity: chooses tag size
        :cam: camera implementation
        :network: to send results
        """
        print("*** MonoAnalysis: AprilTags")
        self._mtx: NDArray[np.float32] = cam.get_intrinsic()
        self._dist: NDArray[np.float32] = cam.get_dist()
        self._blips = network.get_blip_sender()
        self._blips_with_corners = network.get_blip_with_corners_sender()
        self._at_detector = AprilTags._get_detector()
        if identity == Identity.DIST_TEST:
            # the distortion rig uses a 33 mm, 20% scale, tag.
            tag_size = 0.033
        elif identity == Identity.FUNNEL:
            # for rotation testing rig
            tag_size = 0.059
        else:
            # normal tag size is 6.5 inches
            tag_size = 0.1651
        print("*** Tag size:", tag_size)
        self._estimator = AprilTags._get_estimator(tag_size, self._mtx)

    @override
    def analyze_mono(
        self,
        img: MatLike,
        img_display: MatLike | None,
        servertime: int,
    ) -> None:
        """Detect from img, show on img_display, undistort, extract pose,
        and send pose and (raw) corners over the network.

        :img: 8-bit mono, distorted, for detection.
        :img_display: distorted, for display, annotated with tag positions
        :servertime: drift-corrected server-time microsecond image timestamp"""

        result: list[AprilTagDetection] = self._at_detector.detect(img.data)

        blips: list[Blip] = []
        blips_with_corners: list[BlipWithCorners] = []
        tag: AprilTagDetection
        for tag in result:
            if tag.getHamming() > 0:
                continue

            # Extract raw (x,y) corners from the tag.
            raw_corners: tuple[
                float, float, float, float, float, float, float, float
            ] = AprilTags._raw_corners(tag)

            # Undistort the corners.
            undistorted_corners: tuple[
                float, float, float, float, float, float, float, float
            ] = AprilTags._undistorted_corners(self._mtx, self._dist, raw_corners)

            # Redo the homography using the undistorted corners.
            homography: tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any] = (
                AprilTags._homography(undistorted_corners)
            )

            # Estimate the pose.
            pose: Transform3d = self._estimator.estimate(
                homography, undistorted_corners
            )
            blips.append(Blip(int(servertime), tag.getId(), pose))
            blips_with_corners.append(
                BlipWithCorners.make(int(servertime), tag.getId(), raw_corners, pose)
            )
            if img_display is not None:
                DisplayUtil.tag(img_display, tag, pose)

        self._blips.send(blips)
        self._blips_with_corners.send(blips_with_corners)

    @staticmethod
    def _get_detector() -> AprilTagDetector:
        at_detector = AprilTagDetector()
        config = at_detector.Config()
        # some of the detection steps can be done in parallel; this
        # should be the same as the number of cores on the machine,
        # which for the Raspberry Pi 5 is four.
        config.numThreads = 4
        # sharpening kernel is [0,-1,0;-1,4,-1;0,-1,0], see apriltag.c
        # this makes adjacent pixels more different.
        config.decodeSharpening = 0.0
        # this doesn't make much difference in FPS
        # config.decodeSharpening = 0.25

        # do not decimate: improves far-away (small) detections
        # config.quadDecimate = 1.0
        # this makes a huge difference in FPS
        # 1.0: 13 FPS
        # 2.0: 40 FPS
        # 4.0: 60 FPS
        # config.quadDecimate = 4.0
        config.quadDecimate = 2.0
        config.refineEdges = True

        # stddev of the blur kernel in pixels: seems to help with small tags
        config.quadSigma = 0.0

        # Write detector debug images to local storage.  Very slow.
        # config.debug = True

        at_detector.setConfig(config)
        qtp = at_detector.QuadThresholdParameters()
        # The apriltag default is 5.  WPI overrides this
        # with 300, which prevents far-away detections.
        # So set it back to 5.
        # it seems not to make much difference in FPS
        # qtp.minClusterPixels = 300
        qtp.minClusterPixels = 5
        at_detector.setQuadThresholdParameters(qtp)
        at_detector.addFamily("tag36h11")
        return at_detector

    @staticmethod
    def _get_estimator(tag_size: float, mtx: NDArray[np.float32]) -> AprilTagPoseEstimator:
        return AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size, mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]
            )
        )

    @staticmethod
    def _raw_corners(
        result_item: AprilTagDetection,
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Return corners from a detection as a tuple.
        The order is: lower left, lower right, upper right, upper left."""
        return result_item.getCorners((0, 0, 0, 0, 0, 0, 0, 0))

    @staticmethod
    def _undistorted_corners(
        mtx: NDArray[np.float32],
        dist: NDArray[np.float32],
        corners: tuple[float, float, float, float, float, float, float, float],
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Return undistorted tag corners.
        undistortPoints is at least 10X faster than undistort on the whole image."""

        # undistortImagePoints takes [u,v] pixel pairs
        # (MatOfPoint2f in c)
        pairs: MatLike = np.reshape(corners, [4, 2])
        # This is just undistortPoints with mtx as the new intrinsic.
        # The default iterates 5 times and often doesn't get there,
        # so we iterate more times.
        pairs = cv2.undistortImagePoints(
            pairs,
            mtx,
            dist,
            None,
            (cv2.TermCriteria_COUNT | cv2.TermCriteria_EPS, 40, 0.01),
        )

        # The estimator wants a flat tuple: [x0, y0, x1, y1, ...];
        # pairs has an extra dimension, so redo it:
        corners = (
            pairs[0][0][0],
            pairs[0][0][1],
            pairs[1][0][0],
            pairs[1][0][1],
            pairs[2][0][0],
            pairs[2][0][1],
            pairs[3][0][0],
            pairs[3][0][1],
        )
        return corners

    @staticmethod
    def _homography(
        corners: tuple[float, float, float, float, float, float, float, float],
    ):
        """Use OpenCV to compute the homography."""
        dst_points = np.array(
            [
                [corners[0], corners[1]],
                [corners[2], corners[3]],
                [corners[4], corners[5]],
                [corners[6], corners[7]],
            ]
        )
        h, _ = cv2.findHomography(SRC_POINTS, dst_points)
        return (
            h[0, 0],
            h[0, 1],
            h[0, 2],
            h[1, 0],
            h[1, 1],
            h[1, 2],
            h[2, 0],
            h[2, 1],
            h[2, 2],
        )
