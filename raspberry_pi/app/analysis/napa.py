# pylint: disable=E1101,R0903

from typing import override

from cv2.typing import MatLike

from wpimath.geometry._geometry import Transform3d, Pose3d

from app.dashboard.display_util import DisplayUtil
from app.analysis.analysis_protocol import MonoAnalysis
from app.network.network_protocol import Network
from app.network.structs import Blip, BlipWithCorners
from napa_vision import NapaDetector
from napa_vision.types import FiducialImageObservation, FiducialDetector

ZERO_POSE = Transform3d(Pose3d(), Pose3d())

class Napa(MonoAnalysis):
    """A wrapper for the Napa AprilTag detector."""

    def __init__(
        self,
        network: Network,
    ) -> None:
        """Finds Apriltags.

        :network: to send results
        """

        print("*** MonoAnalysis: Napa")
        self._blips = network.get_blip_sender()
        self._blips_with_corners = network.get_blip_with_corners_sender()
        self._at_detector: FiducialDetector = Napa._get_detector()
    

    @override
    def analyze_mono(
        self,
        img: MatLike,
        img_display: MatLike | None,
        servertime: int,
    ) -> None:
        """Detect from img, show on img_display, send (raw) corners over the network.
        :img: 8-bit mono, distorted, for detection.
        :img_display: distorted, for display, annotated with tag positions
        :servertime: drift-corrected server-time microsecond image timestamp"""

        result: list[FiducialImageObservation] = self._at_detector.detect(
            img
        )

        blips: list[Blip] = []
        blips_with_corners: list[BlipWithCorners] = []
        detection: FiducialImageObservation
        for detection in result:

            if detection.hamming > 0:  # Skip imperfect reads. TODO: Is this actually necessary? Why are we skipping these reads?
                continue

            # Extract raw (x,y) corners from the tag.
            raw_corners = Napa._raw_corners(detection)

            # Send dummy pose data
            blips.append(Blip(int(servertime), detection.tag_id, ZERO_POSE))
            blips_with_corners.append(
                BlipWithCorners.make(
                    int(servertime), detection.tag_id, raw_corners, ZERO_POSE
                )
            )
            if img_display is not None:
                DisplayUtil.tag(img_display, detection, ZERO_POSE)

        self._blips.send(blips)
        self._blips_with_corners.send(blips_with_corners)

    @staticmethod
    def _get_detector() -> FiducialDetector:

        config: dict[str, str | None | int | float] = {
            "family": "tag36h11",  # default is tag36h11. only one family supported at a time; for multiple at once, make another detector
            "max_hamming": None,  # default is None
            "min_border_cells": 22,  # default is 22
            "max_detections": 16,  # default is 64. it's unlikely we ever need to detect 64 tags, so 16 should be sufficient.
            "min_tag_px": 11,  # default is 32. lowering this to 11 allows us to detect very distant tags.
            "sharpening": 0.25,  # default is 0.25
            "min_contrast": 40,  # default is 20. higher threshold = less noise passing filter = less to process = less cpu usage
        }

        return NapaDetector(**config)

    @staticmethod
    def _raw_corners(
        result_item: FiducialImageObservation,
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Return corners from a detection as a tuple.
        The order is: lower left, lower right, upper right, upper left."""
        return tuple((result_item.corners[[1, 0, 3, 2]] + 0.5).flatten().tolist())
