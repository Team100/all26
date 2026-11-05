# pylint: disable=E1101,R0903,R1732

from typing import override
import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from app.camera.camera_protocol import Camera
from app.camera.fake_request import FakeRequest
from app.camera.size import Size
from app.util.timer import Timer


class NoCamera(Camera):
    """A camera with no image."""

    def __init__(self) -> None:
        print("*** Camera: NoCamera")
        self.w = 2000
        self.h = 1000
        self._img: MatLike = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        cv2.putText(
            self._img,
            "NO CAMERA AVAILABLE",
            (180, 550),
            cv2.FONT_HERSHEY_COMPLEX,
            5,
            (255, 255, 255)
        )
        self.frame_time = Timer.time_ns()

    @override
    def capture_request(self) -> FakeRequest:
        capture_start: int = Timer.time_ns()
        total_time_ms = (capture_start - self.frame_time) / 1000000
        self.frame_time = capture_start
        fps = 1000 / total_time_ms
        return FakeRequest(self._img, fps)

    @override
    def stop(self) -> None:
        pass

    @override
    def get_size(self) -> Size:
        return Size(
            sensor_width=self.w,
            sensor_height=self.h,
            width=self.w,
            height=self.h,
        )

    @override
    def get_intrinsic(self) -> NDArray[np.float32]:
        # convenient for experiments related to the GS camera
        return np.array(
            [
                [1000, 0, self.w // 2],
                [0, 1000, self.h // 2],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> NDArray[np.float32]:
        k1 = 0  # radial quadratic term
        k2 = 0  # radial quartic term
        p1 = 0  # tangential
        p2 = 0  # tangential
        return np.array([k1, k2, p1, p2])
