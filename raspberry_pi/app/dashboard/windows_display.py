# pylint: disable=C0114,C0115,C0116,C0415,E0401,E0611,E1101,R0902,R0903
from datetime import datetime
from typing import override

import cv2
from cv2.typing import MatLike

from app.dashboard.display_protocol import Display
from app.dashboard.mjpeg_streamer import MjpegServer, Stream  # type: ignore


class WindowsDisplay(Display):
    """Annotate and show the captured image.
    On windows, CvSource breaks with cvnp contiguous-array error,
    so use MjpegServer instead."""

    def __init__(self, name: str, width: int, height: int) -> None:
        print(f"[{datetime.now()}] " + "*** Display: WindowsDisplay")
        self._width: int = width
        self._height: int = height
        self._stream = Stream(name, (width, height), quality=50, fps=30)
        self._server = MjpegServer("localhost", 1181)
        self._server.add_stream(self._stream)
        self._server.start()

    @override
    def put(self, img: MatLike) -> None:  # type: ignore
        img_out = cv2.resize(img, (self._width, self._height))
        self._stream.set_frame(img_out)
