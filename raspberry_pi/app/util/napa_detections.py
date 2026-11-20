from app.util.detections_protocol import AprilTagDetection
from app.util.point import Point2D

from napa_vision.types import FiducialImageObservation
from typing import override

class NapaTagDetection(AprilTagDetection):

    def __init__(self, detection: FiducialImageObservation) -> None:

        self._detection = detection

    @override
    def get_center(self) -> Point2D:
        return Point2D(NapaTagDetection._raw_center(self._detection)[0], NapaTagDetection._raw_center(self._detection)[1])

    @override
    def get_corner(self, ndx: int) -> Point2D:
        raw = NapaTagDetection.raw_corners(self._detection)
        return Point2D(raw[(2 * ndx)], raw[(2 * ndx) + 1])

    @override
    def get_corners(self) -> tuple[Point2D, Point2D, Point2D, Point2D]:
        raw = NapaTagDetection.raw_corners(self._detection)
        return (
            Point2D(raw[0], raw[1]),
            Point2D(raw[2], raw[3]),
            Point2D(raw[4], raw[5]),
            Point2D(raw[6], raw[7]),
        )
    
    @override
    def get_id(self) -> int:
        return self._detection.tag_id
    
    @override
    def get_family(self) -> str:
        return self._detection.family
    
    @staticmethod
    def raw_corners(
        detection: FiducialImageObservation,
    ) -> tuple[float, float, float, float, float, float, float, float]: # type: ignore
        """
        Return corners from a detection as a tuple.
        The order is: lower left, lower right, upper right, upper left.
        """
        return tuple((detection.corners[[1, 0, 3, 2]] + 0.5).flatten().tolist())  # type: ignore
    
    @staticmethod
    def _raw_center(
        detection: FiducialImageObservation,
    ) -> tuple[float, float]:
        return tuple((detection.center + 0.5).flatten().tolist())  # type: ignore
