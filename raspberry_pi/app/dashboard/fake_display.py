# pylint: disable=C0114,C0115,C0116,E0401,R0903,W0613
from datetime import datetime
from typing import override

from cv2.typing import MatLike

from app.dashboard.display_protocol import Display


class FakeDisplay(Display):
    """A display for unit tests."""

    def __init__(self) -> None:
        print(f"[{datetime.now()}] " + "*** Display: FakeDisplay")
        self.frame_count = 0

    @override
    def put(self, img: MatLike) -> None:  # type: ignore
        self.frame_count += 1
