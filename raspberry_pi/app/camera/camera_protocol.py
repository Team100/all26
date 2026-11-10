# pylint: disable=C0114,C0115,C0116,E0401,R0903,W2301
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from app.camera.request_protocol import Request
from app.camera.size import Size


class Camera(Protocol):
    """Interface spec for camera types."""

    def capture_request(self) -> Request: ...
    def stop(self) -> None: ...
    def get_size(self) -> Size: ...
    def get_intrinsic(self) -> NDArray[np.float32]: ...
    def get_dist(self) -> NDArray[np.float32]: ...
