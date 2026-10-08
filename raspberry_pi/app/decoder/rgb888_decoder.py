# pylint: disable=E1101
from typing import override
from collections.abc import Buffer
import cv2
import numpy as np
from numpy.typing import NDArray
from cv2.typing import MatLike
from app.camera.size import Size
from app.decoder.decoder_protocol import Decoder


class Rgb888Decoder(Decoder):
    """Adapts buffers encoded as RGB888, which actually contains BGR tuples."""

    def __init__(self, size: Size):
        print("*** Decoder: Rgb888Decoder")
        self._size = size

    @override
    def mono(self, buffer: Buffer) -> MatLike | None:
        img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        img_bgr: NDArray[np.uint8] = img.reshape(
            (self._size.height, self._size.width, 3)
        )
        img_gray: MatLike = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2GRAY)
        return np.ascontiguousarray(img_gray)

    @override
    def color(self, buffer: Buffer) -> MatLike | None:
        img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        return img.reshape((self._size.height, self._size.width, 3))
