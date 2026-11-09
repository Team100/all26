
# pylint: disable=R0903,W2301

from typing import Protocol
from collections.abc import Buffer

from cv2.typing import MatLike


class Decoder(Protocol):
    def mono(self, buffer: Buffer) -> MatLike | None:
        """Monochrome, one byte per pixel, OpenCV CV_8UC1,
        or None if decoding fails."""
        ...

    def color(self, buffer: Buffer) -> MatLike | None:
        """Color, 3 bytes per pixel, BGR OpenCV CV_8UC3,
        or None if decoding fails."""
        ...
