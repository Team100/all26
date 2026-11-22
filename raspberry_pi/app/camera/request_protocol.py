# pylint: disable=C0114,E0401,W2301

from collections.abc import Buffer
from contextlib import AbstractContextManager
from typing import Protocol

from app.decoder.decoder_protocol import Decoder


class Request(Protocol):
    """The request makes available both a context-managed zero-copy
    frame buffer, and a decoder that produces OpenCV-compatible frames.
    The decoder make a copy of the frame (e.g. to decode JPEG), but
    it might not (e.g. a color frame from RGB888, or a grayscale
    frame from YUV420), so don't use the decoder output outside
    the scope of the buffer."""

    def decoder(self) -> Decoder:
        """Decoder for the format of this request."""
        ...

    def fps(self) -> float:
        """FPS calculated from the previous capture."""
        ...

    def timestamp_boottime_us(self) -> int:
        """Timestamp of the capture instant, relative to "boot time", in microseconds."""
        ...

    def buffer(self) -> AbstractContextManager[Buffer]:
        """Context-managed Buffer containing a single frame.
        The encoding of this frame depends on the configuration
        of the camera."""
        ...

    def release(self) -> None:
        """Release the buffer back to the pool."""
        ...
