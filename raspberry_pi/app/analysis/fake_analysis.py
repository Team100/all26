# pylint: disable=C0114,C0115,C0116,E0401,R0903,W0613
from datetime import datetime
from typing import override

from cv2.typing import MatLike

from app.analysis.analysis_protocol import ColorAnalysis, MonoAnalysis


class FakeMonoAnalysis(MonoAnalysis):
    def __init__(self) -> None:
        print(f"[{datetime.now()}] " + "*** MonoAnalysis: FakeMonoAnalysis")
        self.count = 0
        self.size = None

    @override
    def analyze_mono(
        self,
        img: MatLike,  # type: ignore
        img_display: MatLike,
        servertime: int,
    ) -> None:
        self.count += 1
        self.size = img.shape


class FakeColorAnalysis(ColorAnalysis):
    def __init__(self) -> None:
        print(f"[{datetime.now()}] " + "*** ColorAnalysis: FakeColorAnalysis")
        self.count = 0
        self.size = None

    @override
    def analyze_color(
        self, img: MatLike, img_display: MatLike, servertime: int, min_pixels: int
    ) -> None:
        self.count += 1
        self.size = img.shape
