# pylint: disable=C0114,R0903,W2301
from typing import Protocol

from cv2.typing import MatLike


class Display(Protocol):
    """Interface spec for dashboard video."""

    def put(self, img: MatLike) -> None:  # type: ignore
        """
        :img: a uint8 array, either single-channel (h, w) grayscale,
        or three-channel (h, w, 3) BGR.
        """
        ...
