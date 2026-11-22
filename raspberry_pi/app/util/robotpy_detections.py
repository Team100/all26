from app.util.detections_protocol import AprilTagDetection
from app.util.point import Point2D

from robotpy_apriltag import AprilTagDetection as robotpy_AprilTagDetection
from typing import override

class RobotPyTagDetection(AprilTagDetection):

    def __init__(self, detection: robotpy_AprilTagDetection) -> None:

        self._detection = detection

    @override
    def get_center(self) -> Point2D:
        return Point2D(RobotPyTagDetection._raw_center(self._detection)[0], RobotPyTagDetection._raw_center(self._detection)[1])

    @override
    def get_corner(self, ndx: int) -> Point2D:
        raw = RobotPyTagDetection.raw_corners(self._detection)
        return Point2D(raw[(2 * ndx)], raw[(2 * ndx) + 1])

    @override
    def get_corners(self) -> tuple[Point2D, Point2D, Point2D, Point2D]:
        raw = RobotPyTagDetection.raw_corners(self._detection)
        return (
            Point2D(raw[0], raw[1]),
            Point2D(raw[2], raw[3]),
            Point2D(raw[4], raw[5]),
            Point2D(raw[6], raw[7]),
        )

    @override
    def get_id(self) -> int:
        return self._detection.getId()
    
    @override
    def get_family(self) -> str:
        return self._detection.getFamily()

    @staticmethod
    def raw_corners(
        detection: AprilTagDetection,
    ) -> tuple[float, float, float, float, float, float, float, float]:
        """Return corners from a detection as a tuple.
        The order is: lower left, lower right, upper right, upper left."""
        return detection.getCorners((0, 0, 0, 0, 0, 0, 0, 0)) # type: ignore
    
    @staticmethod
    def _raw_center(
        detection: robotpy_AprilTagDetection,
    ) -> tuple[float, float]:
        c = detection.getCenter()
        return (c.x, c.y) # type: ignore