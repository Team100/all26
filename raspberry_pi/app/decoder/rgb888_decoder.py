# pylint: disable=C0114,C0115,C0116,E0401,E1101,R0903,W2301
from collections.abc import Buffer
from datetime import datetime
from typing import override

import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray

from app.camera.size import Size
from app.decoder.decoder_protocol import Decoder


class Rgb888Decoder(Decoder):
    """Adapts buffers encoded as RGB888, which actually contains BGR tuples."""

    def __init__(self, size: Size):
        print(f"[{datetime.now()}] " + "*** Decoder: Rgb888Decoder")
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
