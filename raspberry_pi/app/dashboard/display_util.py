# pylint: disable=E1101,R0903

import cv2
from cv2.typing import MatLike
from robotpy_apriltag import AprilTagDetection
from wpimath.geometry import Transform3d


FONT = cv2.FONT_HERSHEY_PLAIN
SCALE = 3
THICKNESS = 1
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

class DisplayUtil:
    """Methods for manipulating images for display."""

    @staticmethod
    def note(image: MatLike, contour: MatLike, c_x: int, c_y: int) -> None:
        """Draw the contour with a circle at the center (c_x, c_y)."""
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 3)
        cv2.circle(image, (c_x, c_y), 7, (255, 0,0 ), -1)

    @staticmethod
    def text(image: MatLike, msg: str, loc: tuple[int, int], row: int) -> None:
        """Write text message in the image.

        Note opencv 5 fonts look very different (!)

        :image: to write in
        :msg: text to write
        :loc: pixel location to start, upper left corner
        :row: for multiline messages, which row"""
        x, y = loc
        (size_w, size_h), baseline = cv2.getTextSize(msg, FONT, SCALE, THICKNESS)
        y = y + size_h + row * (size_h + baseline)
        cv2.rectangle(image, (x, y + baseline), (x + size_w, y - size_h), BLACK, -1)
        cv2.putText(image, msg, (x, y), FONT, SCALE, WHITE, THICKNESS)

    @staticmethod
    def tag(image: MatLike, tag: AprilTagDetection, pose: Transform3d) -> None:
        """Draw the tag outline, and annotate its pose.

        :image: to draw on
        :tag: to draw
        :pose: of the tag, to print, transformed to x-forward (!)"""
        for i in range(4):
            j = (i + 1) % 4
            point1 = (int(tag.getCorner(i).x), int(tag.getCorner(i).y))
            point2 = (int(tag.getCorner(j).x), int(tag.getCorner(j).y))
            cv2.line(image, point1, point2, WHITE, 2)
        (c_x, c_y) = (int(tag.getCenter().x), int(tag.getCenter().y))
        cv2.circle(image, (c_x, c_y), 10, WHITE, -1)
        t = pose.translation()
        DisplayUtil.text(image, f"X: {t.z:5.2f}", (c_x, c_y), 0)
        DisplayUtil.text(image, f"Y: {-t.x:5.2f}", (c_x, c_y), 1)
        DisplayUtil.text(image, f"Z: {-t.y:5.2f}", (c_x, c_y), 2)

    @staticmethod
    def rectangle(image: MatLike, loc: tuple[int, int], size: tuple[int, int], color: tuple[int, int, int]) -> None:
        """Draws a rectangle around loc of size size"""
        x, y = loc
        w, l = size
        vertex_1 = (x - w//2, y - l//2)
        vertex_2 = (x + w//2, y + l//2)
        cv2.rectangle(image, vertex_1, vertex_2, color)

