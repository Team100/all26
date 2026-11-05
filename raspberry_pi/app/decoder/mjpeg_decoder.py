# pylint: disable=E1101

from typing import override
from collections.abc import Buffer

import cv2
import numpy as np

from numpy.typing import NDArray
from cv2.typing import MatLike

from app.camera.size import Size
from app.decoder.decoder_protocol import Decoder


class MjpegDecoder(Decoder):
    """Adapts buffers encoded as JPEG."""

    def __init__(self, size: Size):
        print("*** Decoder: MjpegDecoder")
        self._size = size

    @override
    def mono(self, buffer: Buffer) -> MatLike | None:
        jpg: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        return cv2.imdecode(jpg, cv2.IMREAD_GRAYSCALE)

    @override
    def color(self, buffer: Buffer) -> MatLike | None:
        jpg: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        return cv2.imdecode(jpg, cv2.IMREAD_COLOR)
