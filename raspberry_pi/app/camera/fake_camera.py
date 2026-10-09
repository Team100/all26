# pylint: disable=E1101,R0903,R1732

from pathlib import Path
from typing import Optional, override
import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from app.camera.camera_protocol import Camera
from app.camera.fake_request import FakeRequest
from app.camera.size import Size
from app.util.timer import Timer


class FakeCamera(Camera):
    """A camera for desktop testing."""

    def __init__(
        self,
        filename: str,
        size: Optional[tuple[int, int]] = None,
        k1: float = 0.0,
    ) -> None:
        """
        filename: in this directory.
        size: if no size is supplied, the native size is used.
        k1: quadratic distortion term for undistortion
        """
        print("*** Camera: FakeCamera")
        p = Path(__file__).parent / filename
        pathstr: str = str(p)
        # force the file to be read as three-channel BGR
        file = cv2.imread(pathstr, cv2.IMREAD_COLOR)
        if file is None:
            raise ValueError("no file: " + pathstr)
        # img is 3 channel BGR
        self._img: MatLike = file
        if size is not None:
            self._img = cv2.resize(self._img, size)
        self.h: int = self._img.shape[0]
        self.w: int = self._img.shape[1]
        self.c: int = self._img.shape[2]
        self.frame_time = Timer.time_ns()
        self.k1 = k1

        # Here we want to *distort* the image, so the the "undistort" below inverts the distortion.
        self._img: MatLike = self.redistort(self._img)
        self._mono: MatLike = cv2.cvtColor(self._img, cv2.COLOR_RGB2GRAY)
        """mono img for testing"""

        # uncomment to see the distorted thing
        # cv2.imwrite("debug.jpg", self.img)

    def redistort(self, undistorted_img: MatLike) -> MatLike:
        """Use "remap" to invert the undistortion function.

        undistortPoints() takes distorted points as input and yields true points.

        So we make a matrix of points, each of which is its own location.

        When we give undistortPoints() each location, it treats it as if it were from the
        distorted image, and so yields a location corresponding to the true image.
        This is, therefore, a "forward" map.

        remap() takes each point in the *destination* matrix, and applies the map to find
        the point in the *source* matrix that should be used. This is an "inverse" map.

        So using the "undistort" map with "remap" will invert it, becoming "distort."

        Instead, we could apply the analytic distortion model to the input image, but
        that would require interpolation in the destination (to fill the holes),
        and that seems harder.
        """

        # Each value is its location.
        grid: NDArray[np.float32] = np.indices((self.h, self.w), dtype=np.float32)
        grid_y: NDArray[np.float32] = grid[0]
        grid_x: NDArray[np.float32] = grid[1]

        # Flatten row-major.
        x_flat: NDArray[np.float32] = grid_x.ravel()
        y_flat: NDArray[np.float32] = grid_y.ravel()

        # Statck to make a 2d array where each row is a pair, like MatofPoint2f.
        points: MatLike = np.stack([x_flat, y_flat], axis=-1).reshape(-1, 1, 2)

        # Make the undistortion map.
        distort_map: MatLike = cv2.undistortPoints(
            points, self.get_intrinsic(), self.get_dist(), P=self.get_intrinsic()
        )
        distort_map = distort_map.reshape(self.h, self.w, 2)

        # Separate the x and y because that's what remap wants.
        map_x = distort_map[:, :, 0]
        map_y = distort_map[:, :, 1]

        # Apply the map.
        return cv2.remap(undistorted_img, map_x, map_y, interpolation=cv2.INTER_LINEAR)

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
                [935, 0, self.w // 2],
                [0, 935, self.h // 2],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> NDArray[np.float32]:
        k1 = self.k1  # radial quadratic term
        k2 = 0  # radial quartic term
        p1 = 0  # tangential
        p2 = 0  # tangential
        return np.array([k1, k2, p1, p2])
