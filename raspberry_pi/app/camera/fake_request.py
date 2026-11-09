# from datetime import datetime
# pylint: disable=E1101,R0903,R1732

from contextlib import AbstractContextManager, nullcontext
from typing import  override
from collections.abc import Buffer
from cv2.typing import MatLike
from app.camera.request_protocol import Request
from app.decoder.mat_decoder import MatDecoder
from app.util.timer import Timer


class FakeRequest(Request):
    def __init__(self, img: MatLike, fps: float) -> None:
        """
        img: must be 3-channel cv2 BGR.
        """
        self.img = img
        self._fps = fps

    @override
    def decoder(self) -> MatDecoder:
        return MatDecoder()

    @override
    def fps(self) -> float:
        return self._fps

    @override
    def timestamp_boottime_us(self) -> int:
        return (Timer.time_ns() // 1000) - 500

    @override
    def buffer(self) -> AbstractContextManager[Buffer]:
        return nullcontext(self.img.copy().data)

    @override
    def release(self) -> None:
        pass
