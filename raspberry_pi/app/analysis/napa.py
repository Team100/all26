# pylint: disable=E1101,R0903

from typing import override

from cv2.typing import MatLike
from numpy.typing import NDArray
from numpy import float32
from wpimath.geometry._geometry import Transform3d, Pose3d

from app.dashboard.display_util import DisplayUtil
from app.analysis.analysis_protocol import MonoAnalysis
from app.camera.camera_protocol import Camera
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
        cam: Camera,
        network: Network,
    ) -> None:
        """Finds Apriltags & their corners.

        :cam: camera implementation
        :network: to send results
        """

        print("*** MonoAnalysis: Napa")
        self._mtx: NDArray[float32] = cam.get_intrinsic()
        self._dist: NDArray[float32] = cam.get_dist()
        self._blips_with_corners = network.get_blip_with_corners_sender()
        self._at_detector: FiducialDetector = Napa._get_detector((self._mtx,self._dist)) # type: ignore
    

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
            if detection.hamming > 1:  # Skip imperfect reads. TODO: Is this actually necessary? Why are we skipping these reads? # type: ignore
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
    def _get_detector(camera=None) -> FiducialDetector: # type: ignore

        config: dict[str, str | None | bool | int | float] = {
            "family": "tag36h11",  # default is tag36h11. only one family supported at a time; for multiple at once, make another detector
            "min_tag_px": 32,  # default is 32. lowering this allows detection of more distant tags; however it seems to use more performance.
            "max_hamming": None,  # default is None
            "min_contrast": 40,  # default is 20. higher threshold = less noise passing filter = less to process = less cpu usage
            "min_border_cells": 22,  # default is 22
            "sharpening": 0.25,  # default is 0.25
            "max_detections": 16,  # default is 64. it's unlikely we ever need to detect 64 tags, so 16 should be sufficient.
            "split_touching": 'auto',  # bool or 'auto'
            "recover_shadows": False,
            "shadow_min_tag_px": None,
            "soft_decode": False,
            "camera_gamma": None,
            "tag_shape": None,
            "regrow_small_tags": False,
            "camera": camera,
            "threads": 1, 
        }

        return NapaDetector(**config)