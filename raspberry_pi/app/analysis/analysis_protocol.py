# pylint: disable=R0903,W2301

from typing import Protocol
from cv2.typing import MatLike


class MonoAnalysis(Protocol):
    """Interface for analysis of monochrome images."""

    def analyze_mono(
        self,
        img: MatLike,
        img_display: MatLike,
        servertime: int,
    ) -> None:
        """Analysis of a monochrome image.

        :img: must be 8 bit mono.
        :img_display: for display, may be mono or color.
        :servertime: drift-corrected server-time microsecond timestamp.
        """
        ...


class ColorAnalysis(Protocol):
    """Interface for analysis of color images."""

    def analyze_color(
        self,
        img: MatLike,
        img_display: MatLike,
        servertime: int,
        min_pixels: int
    ) -> None:
        """Analysis of a color image.

        :img: must be 24 bit color.
        :img_display: for display, may be mono or color.
        :servertime: drift-corrected server-time microsecond timestamp.
        """
        ...
