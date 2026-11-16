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


class MjpegDecoder(Decoder):
    """Adapts buffers encoded as JPEG."""

    def __init__(self, size: Size):
        print(f"[{datetime.now()}] " + "*** Decoder: MjpegDecoder")
        self._size = size

    @override
    def mono(self, buffer: Buffer) -> MatLike | None:
        jpg: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        return cv2.imdecode(jpg, cv2.IMREAD_GRAYSCALE)

    @override
    def color(self, buffer: Buffer) -> MatLike | None:
        jpg: NDArray[np.uint8] = np.frombuffer(buffer, dtype=np.uint8)
        return cv2.imdecode(jpg, cv2.IMREAD_COLOR)
