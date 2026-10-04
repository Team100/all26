# pylint: disable=E1101

from typing import cast, override
from collections.abc import Buffer
import cv2
import numpy as np
from numpy.typing import NDArray
from cv2.typing import MatLike
from app.camera.size import Size
from app.decoder.decoder_protocol import Decoder


class Yuv420Decoder(Decoder):
    """Adapts buffers encoded as YUV420."""

    def __init__(self, size: Size):
        print("*** Decoder: Yuv420Decoder")
        self._size = size
        self._y_len = size.width * size.height

    @override
    def mono(self, buffer: Buffer) -> MatLike | None:
        # truncate, ignore chrominance. this makes a view, very fast (300 ns)
        img: NDArray[np.uint8] = cast(
            NDArray[np.uint8],
            np.frombuffer(buffer, dtype=np.uint8, count=self._y_len),  # type:ignore
        )
        # this makes a view, very fast (150 ns)
        return img.reshape((self._size.height, self._size.width))  # type:ignore

    @override
    def color(self, buffer: Buffer) -> MatLike | None:
        img: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        img = img.reshape((self._size.height * 3 // 2, self._size.width))  # type:ignore
        img_bgr: MatLike = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_I420)
        return np.ascontiguousarray(img_bgr)
