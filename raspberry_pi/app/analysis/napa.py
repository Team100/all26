# pylint: disable=E1101,R0903

from typing import override

from cv2.typing import MatLike
from wpimath.geometry._geometry import Transform3d, Pose3d

from app.dashboard.display_util import DisplayUtil
from app.analysis.analysis_protocol import MonoAnalysis
from app.network.network_protocol import Network
from app.network.structs import BlipWithCorners
from app.util.napa_detections import NapaTagDetection
from napa_vision import NapaDetector
from napa_vision.types import FiducialImageObservation, FiducialDetector

ZERO_POSE = Transform3d(Pose3d(), Pose3d())

class Napa(MonoAnalysis):
    """A wrapper for the Napa AprilTag detector."""

    def __init__(
        self,
        network: Network,
    ) -> None:
        """Finds Apriltags & their corners.

        :network: to send results
        """

        print("*** MonoAnalysis: Napa")
        self._blips_with_corners = network.get_blip_with_corners_sender()
        self._at_detector: FiducialDetector = Napa._get_detector() # type: ignore
    

    @override
    def analyze_mono(
        self,
        img: MatLike,
        img_display: MatLike | None,
        servertime: int,
    ) -> None:
        """Detect from img, show on img_display, send (raw) corners over the network.
        :img: 8-bit mono, distorted, for detection.
        :img_display: distorted, for display, annotated with tag corners
        :servertime: drift-corrected server-time microsecond image timestamp"""

        result: list[FiducialImageObservation] = self._at_detector.detect(
            img
        )

        blips_with_corners: list[BlipWithCorners] = []
        detection: FiducialImageObservation
        for detection in result:
            if detection.hamming > 0:  # Skip imperfect reads. TODO: Is this actually necessary? Why are we skipping these reads? # type: ignore
                continue

            # Extract raw (x,y) corners from the tag.
            raw_corners = NapaTagDetection.raw_corners(detection)

            # Send dummy pose data
            blips_with_corners.append(
                BlipWithCorners.make(
                    int(servertime), detection.tag_id, raw_corners, ZERO_POSE
                )
            )
            if img_display is not None:
                DisplayUtil.tag(img_display, NapaTagDetection(detection), ZERO_POSE)

        self._blips_with_corners.send(blips_with_corners)

    @staticmethod
    def _get_detector() -> FiducialDetector: # type: ignore

        config: dict[str, str | None | bool | int | float] = {
            "family": "tag36h11",  # default is 'tag36h11'. only one family supported at a time; for multiple at once, make another detector
            "min_tag_px": 32,  # default is 32. lowering this allows detection of more distant tags, but uses more performance (cost grows 1/x^2)
            "max_hamming": None,  # default is None
            "min_contrast": 45,  # default is 20. higher threshold = less noise passing filter = less to process = less cpu usage | TODO: test this in the field to see if we need to change this
            "min_border_cells": 22,  # default is 22
            "sharpening": 0.25,  # default is 0.25. ranges 0 to 1
            "max_detections": 16,  # default is 64. it's unlikely we ever need to detect 64 tags, so 16 should be sufficient.
            "split_touching": False,  # bool or 'auto'
            "recover_shadows": False, # turn this off for higher speed
            "shadow_min_tag_px": None, # only affects the resampling for the above configuration, since we disable it, we can pass None here
            "soft_decode": False,
            "camera_gamma": None,
            "tag_shape": None, # 'tall', 'wide', or None; halves sampling rates of tags on the specified axis
            "regrow_small_tags": False, # experimental, tag36h11 only
            "threads": 1, # TODO: perhaps test to see if using 2 is any better and still viable when running 2 cameras?
        }

        return NapaDetector(**config)