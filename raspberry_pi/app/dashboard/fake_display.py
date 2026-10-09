# pylint: disable=R0903
from typing import override
from cv2.typing import MatLike
from app.dashboard.display_protocol import Display


class FakeDisplay(Display):
    """A display for unit tests."""

    def __init__(self) -> None:
        print("*** Display: FakeDisplay")
        self.frame_count = 0

    @override
    def put(self, img: MatLike) -> None:
        self.frame_count += 1
