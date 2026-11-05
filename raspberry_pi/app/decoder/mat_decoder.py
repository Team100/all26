from datetime import datetime
# pylint: disable=E1101

from typing import override
from collections.abc import Buffer
import cv2
import numpy as np
from cv2.typing import MatLike
from app.decoder.decoder_protocol import Decoder


class MatDecoder(Decoder):
    """Decoder for buffers encoded as CV2 MatLike."""

    @override
    def mono(self, buffer: Buffer) -> MatLike | None:
        # buffer is color, MatLike
        img_bgr = np.asarray(buffer)
        mono = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2GRAY)
        return np.ascontiguousarray(mono)

    @override
    def color(self, buffer: Buffer) -> MatLike | None:
        # buffer is color, MatLike
        return np.asarray(buffer)
