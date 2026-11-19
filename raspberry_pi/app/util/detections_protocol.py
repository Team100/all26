from typing import Protocol
from app.util.point import Point2D


class AprilTagDetection(Protocol):
    """Interface for AprilTag detections"""

    def get_corners(self) -> tuple[Point2D, Point2D, Point2D, Point2D]:
        """
        Gets all four corners of the AprilTag

        :return: All four of the corners
        :rtype: tuple[Point2D, Point2D, Point2D, Point2D]
        """
        ...

    def get_corner(self, ndx: int) -> Point2D:
        """
        Gets the position of individual corner of the AprilTag, by index

        :param ndx: The index/number of the corner to get.
        :type ndx: int
        :return: The corner at the given index
        :rtype: Point2D
        """
        ...

    def get_center(self) -> Point2D:
        """
        Gets the position of the AprilTag's center

        :return: The position of the center, as a Point2D
        :rtype: Point2D
        """
        ...

    def get_id(self) -> int:
        """
        Gets the tag id of the AprilTag
        
        :return: Tag id within the family
        :rtype: int
        """
        ...

    def get_family(self) -> str:
        """
        Gets the tag family of the AprilTag
        
        :return: Tag family
        :rtype: str
        """
        ...