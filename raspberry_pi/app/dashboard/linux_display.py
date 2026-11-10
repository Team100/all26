# pylint: disable=C0114,C0115,C0116,C0415,E0401,E0611,E1101,R0902,R0903
from datetime import datetime
from typing import override

import cv2
from cscore import CameraServer
from cv2.typing import MatLike

from app.dashboard.display_protocol import Display


class LinuxDisplay(Display):
    """Annotate and show the captured image, using CameraServer for Linux.
    See localhost: 1181, 1182, etc"""

    def __init__(self, name: str, width: int, height: int) -> None:
        print(f"[{datetime.now()}] " + "*** Display: LinuxDisplay")
        self._width: int = width
        self._height: int = height
        self._cvsource = CameraServer.putVideo(name, width, height)

    @override
    def put(self, img: MatLike) -> None:  # type: ignore
        img_out = cv2.resize(img, (self._width, self._height))
        self._cvsource.putFrame(img_out)
